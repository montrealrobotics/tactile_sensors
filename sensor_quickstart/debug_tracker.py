#!/usr/bin/env python3
"""
Robotiq Modality Rate & Drop Diagnostic Tracker
Monitors arrival rates, timestamp deltas, and packet drop counts per finger.
"""

import sys
import time
import math
import shutil
import select
from collections import defaultdict
from typing import Dict, Tuple, List, Optional

from protocol import (
    UsbPacketParser, SENSOR_TYPE_STATIC_TACTILE, SENSOR_TYPE_DYNAMIC_TACTILE,
    SENSOR_TYPE_ACCELEROMETER, SENSOR_TYPE_GYROSCOPE, SENSOR_TYPE_TEMPERATURE,
    SENSOR_TYPE_TIMESTAMP, STATIC_TACTILE_SIZE, DYNAMIC_TACTILE_SIZE, IMU_SIZE,
    USB_PACKET_HEADER_SIZE
)
from core import TSF85TactileSensor, NUM_FINGERS, REFRESH_RATE_WINDOW

FIELD_LABEL = {
    SENSOR_TYPE_STATIC_TACTILE:  "Static Tactile",
    SENSOR_TYPE_DYNAMIC_TACTILE: "Dynamic Tactile",
    SENSOR_TYPE_ACCELEROMETER:   "Accelerometer",
    SENSOR_TYPE_GYROSCOPE:       "Gyroscope",
    SENSOR_TYPE_TEMPERATURE:     "Temperature",
    SENSOR_TYPE_TIMESTAMP:       "Timestamp",
}

FIELD_ORDER = [
    SENSOR_TYPE_STATIC_TACTILE,
    SENSOR_TYPE_DYNAMIC_TACTILE,
    SENSOR_TYPE_ACCELEROMETER,
    SENSOR_TYPE_GYROSCOPE,
    SENSOR_TYPE_TEMPERATURE,
    SENSOR_TYPE_TIMESTAMP,
]

_FIELD_BYTE_SIZES = {
    SENSOR_TYPE_STATIC_TACTILE:  STATIC_TACTILE_SIZE * 2,
    SENSOR_TYPE_DYNAMIC_TACTILE: DYNAMIC_TACTILE_SIZE * 2,
    SENSOR_TYPE_ACCELEROMETER:   IMU_SIZE * 2,
    SENSOR_TYPE_GYROSCOPE:       IMU_SIZE * 2,
    SENSOR_TYPE_TEMPERATURE:     2,
    SENSOR_TYPE_TIMESTAMP:       8,
}


def _delta_stats(deltas: List[int]) -> Optional[Dict]:
    """Return mean/std/min/max/count for a list of timestamp deltas, or None."""
    if not deltas:
        return None
    n = len(deltas)
    mean = sum(deltas) / n
    variance = sum((d - mean) ** 2 for d in deltas) / n if n > 1 else 0.0
    return {
        "mean":  mean,
        "std":   math.sqrt(variance),
        "min":   min(deltas),
        "max":   max(deltas),
        "count": n,
    }


class TrackingParser(UsbPacketParser):
    """
    UsbPacketParser subclass that counts individual field arrivals and collects
    firmware timestamp deltas per finger, without changing any parsing logic.
    """
    def __init__(self):
        super().__init__()
        # (sensor_type, finger_id) -> arrival count in current window
        self.field_counts: Dict[Tuple[int, int], int] = defaultdict(int)
        # raw timestamp delta list per finger (firmware ticks, uint16 wraps at 65535)
        self.ts_deltas: List[List[int]] = [[], []]
        self._ts_prev = [None, None]

    def snapshot_and_reset(self) -> Tuple[Dict, List[List[int]]]:
        """Atomically snapshot then clear the window counters."""
        counts = dict(self.field_counts)
        deltas = [list(d) for d in self.ts_deltas]
        self.field_counts = defaultdict(int)
        self.ts_deltas = [[], []]
        return counts, deltas

    def parse_sensor_packet(self, packet: bytes) -> bool:
        """
        Pre-scan the packet to count field arrivals and accumulate timestamp
        deltas per finger, then delegate actual parsing to the parent.
        """
        if len(packet) < USB_PACKET_HEADER_SIZE:
            return False

        data = packet[USB_PACKET_HEADER_SIZE:]
        data_length = len(data)
        idx = 0

        while idx < data_length:
            sensor_byte = data[idx]
            sensor_type = sensor_byte & 0xF0
            finger_id   = (sensor_byte >> 2) & 0x03
            idx += 1

            field_size = _FIELD_BYTE_SIZES.get(sensor_type)
            if field_size is None or idx + field_size > data_length:
                break

            if finger_id < NUM_FINGERS:
                self.field_counts[(sensor_type, finger_id)] += 1

            # Accumulate timestamp deltas per finger (uint64, big-endian)
            if sensor_type == SENSOR_TYPE_TIMESTAMP and finger_id < NUM_FINGERS:
                ts = (
                    (data[idx]     << 56) | (data[idx + 1] << 48) |
                    (data[idx + 2] << 40) | (data[idx + 3] << 32) |
                    (data[idx + 4] << 24) | (data[idx + 5] << 16) |
                    (data[idx + 6] << 8)  |  data[idx + 7]
                )
                prev = self._ts_prev[finger_id]
                if prev is not None and ts > prev:
                    self.ts_deltas[finger_id].append(ts - prev)
                self._ts_prev[finger_id] = ts

            idx += field_size

        new_data_available = super().parse_sensor_packet(packet)
        # Reset new_data_available flags after reading so counts stay per-packet
        for f in self.sensor_data.fingers:
            f.new_data_available = False
        return new_data_available


class FieldTracker:

    def __init__(self):
        self.monitor = TSF85TactileSensor()
        self.parser = TrackingParser()
        self.running = False

        # Running totals
        self.total_packets = 0
        self.total_bytes = 0
        self.frames_in_window = 0

        # Bytes/packets in current window (for data rate)
        self.packets_in_window = 0
        self.bytes_in_window = 0
        self.last_stats_time = time.time()

        # Displayed stats (updated each window)
        self.field_rates: Dict[Tuple[int, int], float] = {}
        self.frames_hz = 0.0
        self.data_rate_kbs = 0.0
        self.window_elapsed = 0.0
        self.ts_stats: List[Optional[Dict]] = [None, None]
        self.lost_packets_window: List[int] = [0, 0]  # gaps >1ms in last window
        self.lost_packets_total: List[int]  = [0, 0]  # running total

        self._display_initialized = False
        self._cursor_hidden = False
        self._alt_screen_enabled = False

    def _update_stats(self, num_packets: int, num_bytes: int):
        self.total_packets += num_packets
        self.total_bytes += num_bytes
        self.packets_in_window += num_packets
        self.bytes_in_window += num_bytes

        now = time.time()
        elapsed = now - self.last_stats_time

        if elapsed >= REFRESH_RATE_WINDOW:
            counts, deltas = self.parser.snapshot_and_reset()

            # Field rates
            self.field_rates = {key: cnt / elapsed for key, cnt in counts.items()}

            # Frame complete rate
            self.frames_hz = self.frames_in_window / elapsed

            # Data rate
            self.data_rate_kbs = (self.bytes_in_window / elapsed) / 1000.0

            # Timestamp delta stats per finger
            self.ts_stats = [_delta_stats(deltas[fi]) for fi in range(NUM_FINGERS)]

            # Lost packets: any gap > 1000us (1ms period) between consecutive timestamps
            window_lost = [
                sum(1 for d in deltas[fi] if d > 1000)
                for fi in range(NUM_FINGERS)
            ]
            self.lost_packets_window = window_lost
            self.lost_packets_total  = [
                self.lost_packets_total[fi] + window_lost[fi]
                for fi in range(NUM_FINGERS)
            ]

            self.window_elapsed = elapsed

            # Reset window
            self.packets_in_window = 0
            self.bytes_in_window = 0
            self.frames_in_window = 0
            self.last_stats_time = now

    def _render(self):
        W = 72
        lines = []
        lines.append("=" * W)
        lines.append(f"Robotiq Modality Rate Tracker    fw: {self.monitor.firmware_version}".center(W))
        lines.append("=" * W)
        lines.append(
            f"Data Rate: {self.data_rate_kbs:.2f} KB/s  |  "
            f"Frames: {self.frames_hz:.1f} Hz  |  "
            f"Packets: {self.total_packets}"
        )
        lines.append(f"Window: {self.window_elapsed:.3f} s")
        lines.append("=" * W)
        lines.append("")

        # Per-field rates table
        lines.append(f"  {'Field':<20}  {'F0 (Hz)':>10}  {'F1 (Hz)':>10}  {'Match':>6}")
        lines.append("  " + "-" * (W - 2))
        for stype in FIELD_ORDER:
            label = FIELD_LABEL[stype]
            hz0 = self.field_rates.get((stype, 0), 0.0)
            hz1 = self.field_rates.get((stype, 1), 0.0)
            # Flag if either finger is >5% below the other
            if hz0 > 0 and hz1 > 0:
                ratio = min(hz0, hz1) / max(hz0, hz1)
                flag = "OK" if ratio >= 0.95 else "DIFF"
            elif hz0 == 0 and hz1 == 0:
                flag = "----"
            else:
                flag = "MISS"
            lines.append(f"  {label:<20}  {hz0:>10.1f}  {hz1:>10.1f}  {flag:>6}")

        lines.append("")
        lines.append(f"  {'Frame Complete':<20}  {self.frames_hz:>10.1f}")
        lines.append("")
        lines.append("=" * W)
        lines.append("")

        # Timestamp delta table
        lines.append(f"  {'Timestamp Deltas (us)':24}  {'F0':>12}  {'F1':>12}")
        lines.append("  " + "-" * (W - 2))
        stat_rows = [
            ("Mean",    "mean",  ".1f"),
            ("Std Dev", "std",   ".1f"),
            ("Min",     "min",   ".0f"),
            ("Max",     "max",   ".0f"),
            ("Count",   "count", "d"),
        ]
        for row_label, key, fmt in stat_rows:
            vals = []
            for fi in range(NUM_FINGERS):
                s = self.ts_stats[fi]
                if s is None:
                    vals.append("  --")
                elif fmt == "d":
                    vals.append(f"{s[key]:>12d}")
                else:
                    vals.append(f"{s[key]:>12{fmt}}")
            lines.append(f"  {row_label:<24}  {vals[0]}  {vals[1]}")

        lines.append("  " + "-" * (W - 2))
        lw = self.lost_packets_window
        lt = self.lost_packets_total
        lines.append(
            f"  {'Timestamp Delta >1000 window':<32}  {lw[0]:>12d}  {lw[1]:>12d}"
        )
        lines.append(
            f"  {'Timestamp Delta >1000 total':<32}  {lt[0]:>12d}  {lt[1]:>12d}"
        )

        lines.append("")
        lines.append("=" * W)
        lines.append("Press Ctrl+C to exit")

        # Trim to terminal height
        th = shutil.get_terminal_size((80, 50)).lines
        if len(lines) > th - 1:
            lines = lines[:th - 2] + [f"... truncated ({len(lines)} lines needed)"]

        if not self._display_initialized:
            sys.stdout.write("\033[?1049h\033[?25l")
            self._alt_screen_enabled = True
            self._cursor_hidden = True
            self._display_initialized = True

        sys.stdout.write("\033[H\033[2J")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def run(self):
        port = self.monitor.find_sensor()
        if not port or not self.monitor.connect(port):
            print("Sensor not found. Check USB connection.")
            return

        if not self.monitor.start_autosend(period_ms=1):
            self.monitor.cleanup()
            return

        self.running = True
        self.last_stats_time = time.time()
        seen = [False] * NUM_FINGERS

        try:
            while self.running:
                waiting = self.monitor.serial_port.in_waiting
                if waiting > 0:
                    raw = self.monitor.serial_port.read(waiting)
                    packets = self.parser.feed_bytes(raw)

                    if packets:
                        self._update_stats(len(packets), len(raw))
                        for packet in packets:
                            new_data_available = self.parser.parse_sensor_packet(packet)
                            for i, v in enumerate(new_data_available[:NUM_FINGERS]):
                                if v:
                                    seen[i] = True
                            if all(seen):
                                self.frames_in_window += 1
                                seen = [False] * NUM_FINGERS
                                self._render()
                else:
                    try:
                        select.select([self.monitor.serial_port.fd], [], [], 0.001)
                    except (AttributeError, ValueError, OSError):
                        time.sleep(0.001)

        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.cleanup()

    def cleanup(self):
        self.monitor.stop_autosend()
        if self._cursor_hidden:
            sys.stdout.write("\033[?25h")
            sys.stdout.flush()
        if self._alt_screen_enabled:
            sys.stdout.write("\033[?1049l")
            sys.stdout.flush()
        if self.monitor.serial_port and self.monitor.serial_port.is_open:
            self.monitor.serial_port.close()


def main():
    tracker = FieldTracker()
    tracker.run()


if __name__ == "__main__":
    main()