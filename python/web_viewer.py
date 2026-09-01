"""
Web-based visualization server for Robotiq Tactile Sensor data.
Serves a real-time dashboard via WebSocket + HTTP.
"""

import asyncio
import json
import math
import os
import signal
import sys
import threading
import time
import traceback
import webbrowser
from collections import deque
from http.server import SimpleHTTPRequestHandler, HTTPServer
from functools import partial
from pathlib import Path
from typing import Optional

try:
    import websockets
except ImportError:
    print("Error: websockets package not installed. Run: pip install websockets")
    sys.exit(1)

from robotiq_tactile_sensor.protocol import NUM_FINGERS
from robotiq_tactile_sensor import TSF85TactileSensor

DISPLAY_POINTS = 500
BROADCAST_HZ = 5


class SensorDataBuffer:
    """Thread-safe circular buffers for sensor data."""

    def __init__(self):
        self._lock = threading.Lock()
        self.static_tactile = [None] * NUM_FINGERS
        self.dynamic_tactile = [deque(maxlen=4096) for _ in range(NUM_FINGERS)]
        self.accelerometer = [deque(maxlen=2000) for _ in range(NUM_FINGERS)]
        self.gyroscope = [deque(maxlen=2000) for _ in range(NUM_FINGERS)]
        self.baseline = [[0] * 28 for _ in range(NUM_FINGERS)]
        self.use_baseline = True
        self.adaptive_range = True
        self.default_range = 3000.0
        self.max_range = [300.0] * NUM_FINGERS  # adaptive starts from 0
        self.push_total = [0] * NUM_FINGERS
        self.push_corrupt = [0] * NUM_FINGERS

    def push(self, sensor_data):
        with self._lock:
            if sensor_data.fingers[0].timestamp != 0 and self.default_range != 1200.0:
                self.default_range = 1200.0

            # --- Update UI buffers ---
            for f in range(NUM_FINGERS):
                finger = sensor_data.fingers[f]
                st = list(finger.static_tactile)
                self.push_total[f] += 1
                if len(st) != 28:
                    self.push_corrupt[f] += 1
                    continue
                self.static_tactile[f] = st
                self.dynamic_tactile[f].append(finger.dynamic_tactile)
                self.accelerometer[f].append(list(finger.accelerometer))
                self.gyroscope[f].append(list(finger.gyroscope))

    def get_static_snapshot(self):
        with self._lock:
            result = []
            for f in range(NUM_FINGERS):
                raw = self.static_tactile[f]
                if raw is None or len(raw) != 28:
                    result.append([0] * 28)
                    continue
                if self.use_baseline:
                    values = [max(0, raw[i] - self.baseline[f][i]) for i in range(28)]
                else:
                    values = list(raw)
                if self.adaptive_range:
                    m = max(values) if values else 0
                    if m > self.max_range[f]:
                        self.max_range[f] = m
                result.append(values)
            return result, list(self.max_range)

    def get_dynamic_snapshot(self):
        """Return subsampled dynamic time-domain data."""
        with self._lock:
            dyn = []
            for f in range(NUM_FINGERS):
                dyn.append(_subsample_deque(self.dynamic_tactile[f], DISPLAY_POINTS))
            return dyn

    def get_imu_snapshot(self):
        """Return subsampled IMU data."""
        with self._lock:
            acc, gyr = [], []
            for f in range(NUM_FINGERS):
                acc.append(_subsample_deque_3axis(self.accelerometer[f], DISPLAY_POINTS))
                gyr.append(_subsample_deque_3axis(self.gyroscope[f], DISPLAY_POINTS))
            return acc, gyr

    def compute_fft(self):
        """Compute FFT from dynamic buffer. Zero-pads to 4096 if >= 512 samples."""
        FFT_SIZE = 4096
        MIN_SAMPLES = 512
        # Copy data under the lock, compute FFT outside it
        with self._lock:
            snapshots = [
                list(self.dynamic_tactile[f])[-FFT_SIZE:]
                if len(self.dynamic_tactile[f]) >= MIN_SAMPLES else None
                for f in range(NUM_FINGERS)
            ]
        results = []
        for s in snapshots:
            if s is None:
                results.append(None)
            else:
                # Zero-pad to FFT_SIZE
                if len(s) < FFT_SIZE:
                    s = s + [0] * (FFT_SIZE - len(s))
                results.append(_fft_magnitudes(s))
        return results

    def reset_baseline(self):
        with self._lock:
            reset_val = 300.0 if self.adaptive_range else self.default_range
            for f in range(NUM_FINGERS):
                if self.static_tactile[f]:
                    self.baseline[f] = list(self.static_tactile[f])
                self.max_range[f] = reset_val


def _subsample_deque(d, max_points):
    n = len(d)
    if n == 0:
        return []
    if n <= max_points:
        return list(d)
    step = n / max_points
    return [d[int(i * step)] for i in range(max_points)]


def _subsample_deque_3axis(d, max_points):
    """Subsample deque of [x,y,z] lists, returning {x:[], y:[], z:[]}."""
    n = len(d)
    if n == 0:
        return {"x": [], "y": [], "z": []}
    if n <= max_points:
        indices = range(n)
    else:
        step = n / max_points
        indices = [int(i * step) for i in range(max_points)]
    x, y, z = [], [], []
    for i in indices:
        s = d[i]
        x.append(s[0])
        y.append(s[1])
        z.append(s[2])
    return {"x": x, "y": y, "z": z}


def _fft_magnitudes(real_data):
    """Pure-Python iterative Cooley-Tukey FFT. Returns first N/2 magnitude bins."""
    N = len(real_data)
    buf_re = list(map(float, real_data))
    buf_im = [0.0] * N
    # Bit-reversal permutation
    j = 0
    for i in range(1, N):
        bit = N >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j ^= bit
        if i < j:
            buf_re[i], buf_re[j] = buf_re[j], buf_re[i]
            buf_im[i], buf_im[j] = buf_im[j], buf_im[i]
    # Butterfly
    length = 2
    while length <= N:
        ang = -2.0 * math.pi / length
        w_re = math.cos(ang)
        w_im = math.sin(ang)
        half = length // 2
        for i in range(0, N, length):
            cur_re, cur_im = 1.0, 0.0
            for k in range(half):
                u_idx = i + k
                v_idx = i + k + half
                t_re = cur_re * buf_re[v_idx] - cur_im * buf_im[v_idx]
                t_im = cur_re * buf_im[v_idx] + cur_im * buf_re[v_idx]
                buf_re[v_idx] = buf_re[u_idx] - t_re
                buf_im[v_idx] = buf_im[u_idx] - t_im
                buf_re[u_idx] += t_re
                buf_im[u_idx] += t_im
                cur_re, cur_im = cur_re * w_re - cur_im * w_im, cur_re * w_im + cur_im * w_re
        length *= 2
    return [math.sqrt(buf_re[i] ** 2 + buf_im[i] ** 2) for i in range(N // 2)]


class WebViewer:
    def __init__(self, monitor: TSF85TactileSensor, port=8080):
        self.monitor = monitor
        self.port = port
        self.buffer = SensorDataBuffer()
        self.clients = set()
        self.active_tab = "static"

    async def websocket_handler(self, websocket):
        self.clients.add(websocket)
        try:
            async for message in websocket:
                msg = json.loads(message)
                if msg.get("type") == "start_recording":
                    filename = msg.get("filename", "recording.csv")
                    actual_path = self.monitor.start_recording(filename)
                    if actual_path:
                        await websocket.send(json.dumps({
                            "type": "recording_started",
                            "filename": actual_path
                        }))
                elif msg.get("type") == "stop_recording":
                    self.monitor.stop_recording()
                    await websocket.send(json.dumps({"type": "recording_stopped"}))
                elif msg.get("type") == "tab_change":
                    self.active_tab = msg["tab"]
                elif msg.get("type") == "reset_baseline":
                    self.monitor.reset_baseline(num_samples=100)
                    for f in range(NUM_FINGERS):
                        self.buffer.baseline[f] = list(self.monitor.baseline[f])
                    self.buffer.reset_baseline()
                elif msg.get("type") == "set_raw_mode":
                    self.buffer.use_baseline = not msg.get("raw", False)
                elif msg.get("type") == "set_adaptive_range":
                    with self.buffer._lock:
                        self.buffer.adaptive_range = msg.get("adaptive", True)
                        if self.buffer.adaptive_range:
                            self.buffer.max_range = [300.0] * NUM_FINGERS
                        else:
                            self.buffer.max_range = [self.buffer.default_range] * NUM_FINGERS
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)

    async def broadcast_loop(self):
        interval = 1.0 / BROADCAST_HZ
        busy = set()  # clients still sending the previous frame

        async def _send(client, payload):
            try:
                await client.send(payload)
            except websockets.ConnectionClosed:
                pass
            finally:
                busy.discard(client)

        while True:
            try:
                if self.clients:
                    tab = self.active_tab
                    msg = {"type": "data", "tab": tab}

                    if tab == "static":
                        values, max_ranges = self.buffer.get_static_snapshot()
                        msg["static"] = values
                        msg["maxRange"] = max_ranges
                    elif tab == "dynamic":
                        msg["dynamic"] = self.buffer.get_dynamic_snapshot()
                    elif tab == "imu":
                        acc, gyr = self.buffer.get_imu_snapshot()
                        msg["accel"] = acc
                        msg["gyro"] = gyr

                    payload = json.dumps(msg)
                    for client in self.clients.copy():
                        if client not in busy:
                            busy.add(client)
                            asyncio.ensure_future(_send(client, payload))
            except Exception:
                traceback.print_exc(file=sys.stderr)
            await asyncio.sleep(interval)

    async def fft_loop(self):
        """Compute FFT at 1Hz and broadcast directly to clients."""
        loop = asyncio.get_event_loop()
        while True:
            try:
                fft_result = await loop.run_in_executor(None, self.buffer.compute_fft)
                if self.clients and self.active_tab == "dynamic":
                    payload = json.dumps({"type": "fft", "fft": fft_result})
                    await asyncio.gather(
                        *[c.send(payload) for c in self.clients.copy()],
                        return_exceptions=True
                    )
            except Exception:
                traceback.print_exc(file=sys.stderr)
            await asyncio.sleep(1.0)

    async def run_server(self):
        web_dir = Path(__file__).parent / "web"
        handler = partial(QuietHTTPHandler, directory=str(web_dir))
        httpd = HTTPServer(("0.0.0.0", self.port), handler)
        http_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        http_thread.start()
        print(f"  HTTP server:      http://localhost:{self.port}")

        ws_port = self.port + 1
        async with websockets.serve(self.websocket_handler, "0.0.0.0", ws_port):
            print(f"  WebSocket server: ws://localhost:{ws_port}")
            await asyncio.gather(self.broadcast_loop(), self.fft_loop())


class QuietHTTPHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass


def _serial_reader_loop(monitor: TSF85TactileSensor, buffer: SensorDataBuffer):
    """Background polling loop pushing data to SensorDataBuffer."""
    monitor.running = True
    while monitor.running:
        for sensor_data in monitor.poll_data():
            buffer.push(sensor_data)


def run_web_viewer(monitor: TSF85TactileSensor, port=8080):
    viewer = WebViewer(monitor, port)

    # Seed buffer baseline from calibration
    for f in range(NUM_FINGERS):
        viewer.buffer.baseline[f] = list(monitor.baseline[f])

    serial_thread = threading.Thread(
        target=_serial_reader_loop,
        args=(monitor, viewer.buffer),
        daemon=True
    )
    serial_thread.start()

    url = f"http://localhost:{port}"
    print("Web viewer starting...")
    print(f"  URL: {url}")
    print("  Press Ctrl+C to stop.\n")
    webbrowser.open(url)

    # All threads are daemon — hard exit on Ctrl+C is safe and responsive
    signal.signal(signal.SIGINT, lambda *_: os._exit(0))
    asyncio.run(viewer.run_server())


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Robotiq Tactile Sensor Web Viewer")
    parser.add_argument('--port', type=int, default=8080, help='Web server port (default: 8080)')
    args = parser.parse_args()

    sensor_monitor = TSF85TactileSensor()
    sensor_port = sensor_monitor.find_sensor()
    if not sensor_port or not sensor_monitor.connect(sensor_port):
        print("Sensor not found or failed to connect.")
        sys.exit(1)

    if not sensor_monitor.start_autosend(period_ms=1):
        sensor_monitor.cleanup()
        sys.exit(1)

    sensor_monitor.detect_connected_fingers()
    print("Calibrating baseline for Web Viewer...")
    sensor_monitor.reset_baseline(num_samples=500)

    try:
        run_web_viewer(sensor_monitor, port=args.port)
    finally:
        sensor_monitor.cleanup()
