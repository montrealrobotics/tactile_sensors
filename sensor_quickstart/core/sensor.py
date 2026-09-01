#!/usr/bin/env python3
"""
Core Sensor Hardware Interface for Robotiq Tactile Sensors
"""

import sys
import time
import select
from typing import Optional, List, Dict
import os

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed")
    print("Please install with: pip install pyserial")
    sys.exit(1)

from protocol import UsbPacketParser
from .recorder import create_recorder, BaseRecorder

# Serial port configuration
BAUD_RATE = 115200
DATA_BITS = 8
PARITY = 'N'
STOP_BITS = 1
TIMEOUT = 0.1  # 100ms timeout for reads

MASTER_HUB_APP_VID_OLD = 0x04B4
MASTER_HUB_APP_PID_OLD = 0xF232
MASTER_HUB_APP_VID = 0x16D0
MASTER_HUB_APP_PID = 0x14CC

# Display configuration
NUM_FINGERS = 2  # Currently 2 fingers
REFRESH_RATE_WINDOW = 1.0  # Calculate refresh rate over 1 second


class TSF85TactileSensor:
    def __init__(self):
        self.parser = UsbPacketParser()
        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        self.firmware_version = ""
        self.baseline = [[0] * 28 for _ in range(NUM_FINGERS)]
        self.connected_fingers: List[int] = list(range(NUM_FINGERS))
        self.recorder: Optional[BaseRecorder] = None

    def find_sensor(self) -> Optional[str]:
        """Find the tactile sensor device port via udev or VID:PID match."""
        for i in range(10):
            symlink = f"/dev/rq_tsf85_{i}"
            if os.path.exists(symlink):
                return symlink

        vid_pid_pairs = [
            (MASTER_HUB_APP_VID, MASTER_HUB_APP_PID),
            (MASTER_HUB_APP_VID_OLD, MASTER_HUB_APP_PID_OLD),
        ]
        for p in serial.tools.list_ports.comports():
            for vid, pid in vid_pid_pairs:
                if p.vid == vid and p.pid == pid:
                    return p.device
        return None

    def connect(self, port_name: str) -> bool:
        """Connect to the sensor on the specified port."""
        try:
            self.serial_port = serial.Serial(
                port=port_name, baudrate=BAUD_RATE, bytesize=DATA_BITS,
                parity=PARITY, stopbits=STOP_BITS, timeout=TIMEOUT, write_timeout=TIMEOUT
            )
            # Set DTR and RTS (may help wake up the sensor)
            self.serial_port.dtr = True
            self.serial_port.rts = False
            # Wait a bit for the port to stabilize
            time.sleep(0.2)
            # Clear any stale data
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.parser.buffer.clear()
            self.read_firmware_version()
            return True
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False

    def read_firmware_version(self):
        self.firmware_version = self.parser.print_firmware_version(self.serial_port)

    def start_autosend(self, period_ms: int = 1):
        """Start continuous sensor data streaming"""
        if not self.serial_port:
            return False
        command = self.parser.create_autosend_command(period_ms)
        self.serial_port.write(command)
        self.serial_port.flush()
        time.sleep(0.2)
        return True

    def detect_connected_fingers(self, duration_s: float = 0.5) -> List[int]:
        """
        Listen briefly after autosend has started and return the list of finger IDs
        that produced data. Run once at startup; hot-plugging is not supported.
        """

        seen = [False] * NUM_FINGERS
        deadline = time.time() + duration_s
        while time.time() < deadline:
            waiting = self.serial_port.in_waiting
            if waiting > 0:
                data = self.serial_port.read(waiting)
                for packet in self.parser.feed_bytes(data):
                    new_data_available = self.parser.parse_sensor_packet(packet)
                    for i, v in enumerate(new_data_available[:NUM_FINGERS]):
                        if v:
                            seen[i] = True
                    for f in self.parser.sensor_data.fingers:
                        f.new_data_available = False
            else:
                try:
                    select.select([self.serial_port.fd], [], [], 0.001)
                except (AttributeError, ValueError, OSError):
                    time.sleep(0.001)

        self.connected_fingers = [i for i, v in enumerate(seen) if v]
        return self.connected_fingers

    def stop_autosend(self):
        """Stop continuous sensor data streaming"""
        if not self.serial_port:
            return
        command = self.parser.create_autosend_command(0)  # period=0 stops autosend
        self.serial_port.write(command)
        time.sleep(0.1)

    def poll_data(self):
        if self.serial_port and self.serial_port.in_waiting > 0:
            data = self.serial_port.read(self.serial_port.in_waiting)
            for packet in self.parser.feed_bytes(data):
                new_data_available = self.parser.parse_sensor_packet(packet)
                if all(new_data_available[i] for i in self.connected_fingers):
                    sensor_data = self.parser.get_sensor_data()
                    for f in sensor_data.fingers:
                        f.new_data_available = False
                    if self.recorder and self.recorder.is_recording:
                        self.recorder.write_frame(sensor_data, self.baseline)

                    yield sensor_data
        else:
            self._sleep_os()

    def reset_baseline(self, num_samples: int = 500) -> bool:
        """
        Reset the baseline for all taxels by averaging static tactile data over num_samples.

        Args:
            num_samples: Number of data samples to average (default: 1000)
        """

        if not self.serial_port:
            return False

        # Initialize accumulator arrays for each finger's taxels
        # Structure: accumulators[finger_id][taxel_index] = sum of values
        accumulators = [[0] * 28 for _ in range(NUM_FINGERS)]
        samples_collected = 0

        # Collect samples
        start_time = time.time()
        timeout = 10.0

        while samples_collected < num_samples:
            if time.time() - start_time > timeout:
                return False

            # Read available data
            for sensor_data in self.poll_data():
                for finger_id in self.connected_fingers:
                    finger = sensor_data.fingers[finger_id]
                    for taxel_idx in range(28):
                        accumulators[finger_id][taxel_idx] += finger.static_tactile[taxel_idx]
                samples_collected += 1

        # Accumulate static tactile values for connected fingers
        for finger_id in self.connected_fingers:
            for taxel_idx in range(28):
                self.baseline[finger_id][taxel_idx] = accumulators[finger_id][taxel_idx] // num_samples

        return True

    def start_recording(self, filepath: str = "recording.h5", keep_baseline: bool = False) -> Optional[str]:
        self.recorder = create_recorder(filepath, keep_baseline=keep_baseline)
        actual_path = self.recorder.start(self.baseline)
        if actual_path:
            print(f"[TactileSensor] Started recording to: {actual_path}")
        return actual_path

    def stop_recording(self):
        if self.recorder and self.recorder.is_recording:
            count = self.recorder.recorded_count
            path = self.recorder.filepath
            self.recorder.stop()
            print(f"[TactileSensor] Saved {count} frames to {path}")

    def _sleep_os(self):
        try:
            select.select([self.serial_port.fd], [], [], 0.001)
        except (AttributeError, ValueError, OSError):
            time.sleep(0.001)

    def cleanup(self):
        self.stop_autosend()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
