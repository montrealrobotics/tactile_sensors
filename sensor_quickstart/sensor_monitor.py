#!/usr/bin/env python3
"""
Interactive Terminal Monitor & Data Recorder
"""

import sys
import os
import time
import shutil
import argparse
from typing import Optional
from core import TSF85TactileSensor, NUM_FINGERS

REFRESH_RATE_WINDOW = 1.0


def _kbhit_init():
    """Set up non-blocking keyboard input (cross-platform)."""
    if os.name == 'nt':
        import msvcrt
        return {'type': 'nt'}
    else:
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        return {'type': 'posix', 'fd': fd, 'old_settings': old_settings}


def _kbhit_check(ctx):
    """Check if a key has been pressed (non-blocking). Returns char or None."""
    if not ctx:
        return None
    if ctx['type'] == 'nt':
        import msvcrt
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8', errors='replace').lower()
    else:
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
    return None


def _kbhit_cleanup(ctx):
    """Restore terminal settings."""
    if ctx and ctx['type'] == 'posix':
        import termios
        termios.tcsetattr(ctx['fd'], termios.TCSADRAIN, ctx['old_settings'])


class TerminalRecorder:

    def __init__(self, monitor: TSF85TactileSensor, output_filename: Optional[str] = None, keep_baseline: bool = False):
        self.monitor = monitor
        self.keep_baseline = keep_baseline
        self.requested_filename = output_filename

        # Stats
        self.total_packets = 0
        self.total_bytes = 0
        self.displays_in_window = 0
        self.packets_in_window = 0
        self.bytes_in_window = 0
        self.last_stats_time = time.time()
        self.refresh_rate_hz = 0.0
        self.data_rate_kbs = 0.0

        # UI state
        self._display_initialized = False
        self._cursor_hidden = False
        self._alt_screen_enabled = False

    def update_statistics(self, num_packets: int, num_bytes: int):
        """Update data rate statistics"""
        self.total_packets += num_packets
        self.total_bytes += num_bytes
        self.packets_in_window += num_packets
        self.bytes_in_window += num_bytes

        current_time = time.time()
        elapsed = current_time - self.last_stats_time

        if elapsed >= REFRESH_RATE_WINDOW:
            # Calculate rates (matching TactileSensorUI calculations exactly)
            # Refresh rate = complete data sets per second (not packets per second)
            self.refresh_rate_hz = self.displays_in_window / elapsed
            # Data rate: bytes per second, displayed as KB/s
            # Matching TactileSensorUI: receivedBytes * 1000 / elapsed_ms = bytes/second
            self.data_rate_kbs = (self.bytes_in_window / elapsed) / 1000.0
            # Reset window counters
            self.packets_in_window = 0
            self.bytes_in_window = 0
            self.displays_in_window = 0
            self.last_stats_time = current_time

    def render(self, data):

        lines = ["=" * 80]
        lines.append(f"Robotiq Tactile Sensor Monitor (fw: {self.monitor.firmware_version})".center(80))
        lines.append("=" * 80)
        lines.append(f"Data Rate: {self.data_rate_kbs:.3f} KB/s  |  "
                     f"Refresh Rate: {self.refresh_rate_hz:.1f} Hz  |  "
                     f"Total Packets: {self.total_packets}")
        rec = self.monitor.recorder
        if rec and rec.is_recording:
            rec_str = f"RECORDING ({rec.recorded_count} samples)"
            active_path = rec.filepath
        else:
            rec_str = "STANDBY"
            active_path = self.requested_filename

        lines.append(f"Logging: {active_path} | Status: {rec_str}")
        lines.append("Controls: [s] Start Rec & Baseline | [c] Stop Rec | [q] Quit")

        for finger_id in range(NUM_FINGERS):
            lines.append(f"FINGER {finger_id}")
            lines.append("-" * 80)
            if finger_id not in self.monitor.connected_fingers:
                lines.append("  Replug Finger and Relaunch\n")
                continue

            finger = data.fingers[finger_id]
            # Static Tactile (7x4 grid)
            lines.append("  Static Tactile (7 rows × 4 columns):")
            # Subtract baseline from static tactile (element-wise)
            baseline_corrected = [s - b for s, b in zip(finger.static_tactile, self.monitor.baseline[finger_id])]
            for r in range(7):
                row_vals = baseline_corrected[r*4:(r+1)*4]
                lines.append("    " + " ".join(f"{v:5d}" for v in row_vals))
            lines.append("")

            # Dynamic Tactile
            lines.append(f"  Dynamic Tactile: {finger.dynamic_tactile:6d}\n")

            # IMU data
            lines.append(f"  Accelerometer: X={finger.accelerometer[0]:6d}  Y={finger.accelerometer[1]:6d}  Z={finger.accelerometer[2]:6d}")
            lines.append(f"  Gyroscope:     X={finger.gyroscope[0]:6d}  Y={finger.gyroscope[1]:6d}  Z={finger.gyroscope[2]:6d}")
            lines.append(f"  Timestamp: {finger.timestamp:6d}\n")

        lines.append("=" * 80)
        lines.append("Press Ctrl+C or 'q' to exit")

        # Trim to terminal height to avoid scrolling when the display is taller than the window
        term_height = shutil.get_terminal_size((80, 50)).lines
        max_lines = max(term_height - 1, 1)
        if len(lines) > max_lines:
            lines = lines[:max_lines - 1] + [f"... truncated ({len(lines)} lines required)"]

        # Render in place at the top of the screen to avoid scrolling noise
        if not self._display_initialized:
            # Switch to alternate screen to avoid scrollback growth
            sys.stdout.write("\033[?1049h\033[?25l")  # Hide cursor for a cleaner view
            self._alt_screen_enabled = True
            self._cursor_hidden = True
            self._display_initialized = True

        # Clear screen and move to top-left, then write the frame
        sys.stdout.write("\033[H\033[2J")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def run(self):
        self.monitor.running = True
        kb_ctx = _kbhit_init()

        try:
            while self.monitor.running:
                if kb_ctx:
                    key = _kbhit_check(kb_ctx)
                    if key == 's' and (not self.monitor.recorder or not self.monitor.recorder.is_recording):
                        self.monitor.reset_baseline(num_samples=500)
                        self.monitor.start_recording(self.requested_filename, keep_baseline=self.keep_baseline)
                    elif key == 'c' and self.monitor.recorder and self.monitor.recorder.is_recording:
                        self.monitor.stop_recording()
                    elif key == 'q':
                        break

                for data in self.monitor.poll_data():
                    self.displays_in_window += 1
                    self.update_statistics(1, 64)
                    self.render(data)

        except KeyboardInterrupt:
            pass
        finally:
            _kbhit_cleanup(kb_ctx)
            self.monitor.stop_recording()
            if self._cursor_hidden:
                sys.stdout.write("\033[?25h")
            if self._alt_screen_enabled:
                sys.stdout.write("\033[?1049l")
            sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description="Robotiq Tactile Sensor Monitor & Recorder")
    parser.add_argument('--output', '-o', type=str, default="test.csv", help='File path to record output (.csv or .h5)')
    parser.add_argument('--keep-baseline', action='store_true', help='Keep raw static values in recording')
    args = parser.parse_args()

    monitor = TSF85TactileSensor()
    port = monitor.find_sensor()
    if not port or not monitor.connect(port):
        print("Sensor connection failed.")
        return 1

    if not monitor.start_autosend(period_ms=1):
        monitor.cleanup()
        return 1

    if not monitor.detect_connected_fingers():
        print("No finger sensors detected.")
        monitor.cleanup()
        return 1

    app = TerminalRecorder(monitor, output_filename=args.output, keep_baseline=args.keep_baseline)

    print("Calibrating initial baseline...")
    monitor.reset_baseline(num_samples=1000)

    app = TerminalRecorder(monitor, output_filename=args.output, keep_baseline=args.keep_baseline)

    try:
        app.run()
    finally:
        monitor.cleanup()

    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
