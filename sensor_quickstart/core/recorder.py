from abc import ABC, abstractmethod
import csv
import h5py
import numpy as np
import os
import time
from typing import Optional, List
from datetime import datetime

NUM_FINGERS = 2


def get_unique_filepath(filename: str, default_dir: str = "data") -> str:
    dir_name = os.path.dirname(filename)
    base_filename = os.path.basename(filename)

    if not dir_name:
        dir_name = default_dir

    os.makedirs(dir_name, exist_ok=True)

    name_without_ext, ext = os.path.splitext(base_filename)
    if not ext:
        ext = ".csv"

    timestamp_str = datetime.now().strftime("%Y-%m-%d_%H%M%S%f")
    new_filename = f"{name_without_ext}_{timestamp_str}{ext}"

    return os.path.join(dir_name, new_filename)


class BaseRecorder(ABC):

    def __init__(self, filepath: str, keep_baseline: bool = False):
        self.requested_filepath = filepath
        self.filepath: Optional[str] = None
        self.keep_baseline = keep_baseline
        self.is_recording = False
        self.recorded_count = 0

    @abstractmethod
    def start(self, baseline: List[List[int]]) -> Optional[str]:
        pass

    @abstractmethod
    def write_frame(self, sensor_data, baseline: List[List[int]]):
        """
        Append a single frame of sensor data.
        """
        pass

    @abstractmethod
    def stop(self):
        pass


class CSVRecorder(BaseRecorder):

    def __init__(self, filepath: str = "recording.csv", keep_baseline: bool = False):
        super().__init__(filepath, keep_baseline)
        self.file = None
        self.writer = None

    def start(self, baseline: List[List[int]]) -> Optional[str]:
        self.filepath = get_unique_filepath(self.requested_filepath, default_dir="data")
        try:
            self.file = open(self.filepath, mode='w', newline='', buffering=1)
            self.writer = csv.writer(self.file, delimiter=';')

            header = ['Time(ms)']
            for f_id in range(NUM_FINGERS): header.append(f'D0_{f_id}')
            for f_id in range(NUM_FINGERS):
                for t_id in range(28): header.append(f'S{t_id}_{f_id}')
            for f_id in range(NUM_FINGERS): header.extend([f'Ax{f_id}', f'Ay{f_id}', f'Az{f_id}'])
            for f_id in range(NUM_FINGERS): header.extend([f'Gx{f_id}', f'Gy{f_id}', f'Gz{f_id}'])
            self.writer.writerow(header)

            # Write Baseline Row
            base_row = [int(time.time() * 1000)]
            for _ in range(NUM_FINGERS): base_row.append(0)
            for f_id in range(NUM_FINGERS): base_row.extend(baseline[f_id])
            for _ in range(NUM_FINGERS): base_row.extend([0, 0, 0])
            for _ in range(NUM_FINGERS): base_row.extend([0, 0, 0])
            self.writer.writerow(base_row)

            self.is_recording = True
            self.recorded_count = 0
            return self.filepath
        except Exception as e:
            print(f"[CSVRecorder Error] Could not start recording: {e}")
            self.stop()
            return None

    def write_frame(self, sensor_data, baseline: List[List[int]]):
        if not self.is_recording or not self.writer:
            return

        row = [int(time.time() * 1000)]
        for f in range(NUM_FINGERS):
            row.append(sensor_data.fingers[f].dynamic_tactile)
        for f in range(NUM_FINGERS):
            finger = sensor_data.fingers[f]
            if not self.keep_baseline:
                corrected = [s - b for s, b in zip(finger.static_tactile, baseline[f])]
                row.extend(corrected)
            else:
                row.extend(finger.static_tactile)
        for f in range(NUM_FINGERS):
            row.extend(sensor_data.fingers[f].accelerometer)
        for f in range(NUM_FINGERS):
            row.extend(sensor_data.fingers[f].gyroscope)

        self.writer.writerow(row)
        self.recorded_count += 1

    def stop(self):
        self.is_recording = False
        if self.file and not self.file.closed:
            self.file.flush()
            self.file.close()
        self.file = None
        self.writer = None


class HDF5Recorder(BaseRecorder):

    def __init__(self, filepath: str = "recording.h5", keep_baseline: bool = False):
        super().__init__(filepath, keep_baseline)
        self.file = None

    def start(self, baseline: List[List[int]]) -> Optional[str]:
        self.filepath = get_unique_filepath(self.requested_filepath, default_dir="data")
        try:
            self.file = h5py.File(self.filepath, "w")

            baseline_matrix = np.array(baseline, dtype=np.int16).reshape(NUM_FINGERS, 7, 4)
            self.file.create_dataset("baseline", data=baseline_matrix)
            self.file.attrs["keep_baseline"] = self.keep_baseline
            self.file.attrs["start_timestamp_ms"] = int(time.time() * 1000)

            self.timestamps = self.file.create_dataset(
                "timestamp", shape=(0,), maxshape=(None,), dtype="int64", chunks=(1000,)
            )
            self.static_tactile = self.file.create_dataset(
                "static_tactile", shape=(0, NUM_FINGERS, 7, 4), maxshape=(None, NUM_FINGERS, 7, 4), dtype="int16", chunks=(1000, NUM_FINGERS, 7, 4)
            )
            self.dynamic_tactile = self.file.create_dataset(
                "dynamic_tactile", shape=(0, NUM_FINGERS), maxshape=(None, NUM_FINGERS), dtype="int32", chunks=(1000, NUM_FINGERS)
            )
            self.imu_accel = self.file.create_dataset(
                "imu/accel", shape=(0, NUM_FINGERS, 3), maxshape=(None, NUM_FINGERS, 3), dtype="int16", chunks=(1000, NUM_FINGERS, 3)
            )
            self.imu_gyro = self.file.create_dataset(
                "imu/gyro", shape=(0, NUM_FINGERS, 3), maxshape=(None, NUM_FINGERS, 3), dtype="int16", chunks=(1000, NUM_FINGERS, 3)
            )

            self.is_recording = True
            self.recorded_count = 0
            return self.filepath
        except Exception as e:
            print(f"[HDF5Recorder Error] Could not start recording: {e}")
            self.stop()
            return None

    def write_frame(self, sensor_data, baseline: List[List[int]]):
        if not self.is_recording or not self.file:
            return

        idx = self.timestamps.shape[0]

        self.timestamps.resize(idx + 1, axis=0)
        self.static_tactile.resize(idx + 1, axis=0)
        self.dynamic_tactile.resize(idx + 1, axis=0)
        self.imu_accel.resize(idx + 1, axis=0)
        self.imu_gyro.resize(idx + 1, axis=0)

        static_frame = np.zeros((NUM_FINGERS, 7, 4), dtype=np.int16)
        dynamic_frame = np.zeros((NUM_FINGERS,), dtype=np.int32)
        accel_frame = np.zeros((NUM_FINGERS, 3), dtype=np.int16)
        gyro_frame = np.zeros((NUM_FINGERS, 3), dtype=np.int16)

        for f in range(NUM_FINGERS):
            finger = sensor_data.fingers[f]
            dynamic_frame[f] = finger.dynamic_tactile
            accel_frame[f] = finger.accelerometer
            gyro_frame[f] = finger.gyroscope

            if not self.keep_baseline:
                corrected = [s - b for s, b in zip(finger.static_tactile, baseline[f])]
                static_frame[f] = np.array(corrected, dtype=np.int16).reshape(7, 4)
            else:
                static_frame[f] = np.array(finger.static_tactile, dtype=np.int16).reshape(7, 4)

        self.timestamps[idx] = int(time.time() * 1000)
        self.static_tactile[idx] = static_frame
        self.dynamic_tactile[idx] = dynamic_frame
        self.imu_accel[idx] = accel_frame
        self.imu_gyro[idx] = gyro_frame

        self.recorded_count += 1

    def stop(self):
        self.is_recording = False
        if self.file:
            self.file.flush()
            self.file.close()
            self.file = None


def create_recorder(filepath: str, keep_baseline: bool = False) -> BaseRecorder:
    if filepath.endswith(".h5") or filepath.endswith(".hdf5"):
        return HDF5Recorder(filepath, keep_baseline)
    return CSVRecorder(filepath, keep_baseline)