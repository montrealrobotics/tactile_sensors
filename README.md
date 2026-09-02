# TSF-85 Tactile Sensor

SDK, sensor I/O, and tools for the Robotiq TSF-85 tactile sensor.

The TSF-85 provides per-finger data at 1 kHz over USB:
- **28-element tactile array** (7x4 grid) per finger — updated at 60 Hz
- **IMU** (accelerometer, gyroscope, magnetometer)
- **Dynamic tactile** sensor

For generating synthetic tactile maps of the sensor, checkout this [TSF-85 Isaac Sim Extension](https://github.com/Lab-CORO/TSF-85).

## Sensor Details

- Baud rate: 115200
- Format: 8N1 (8 data bits, no parity, 1 stop bit)
- USB VID:PID: 16d0:14cc (Robotiq) or 04b4:f232 (Cypress, older units)
- Data: 28 tactile sensors per finger (7×4 grid) + IMU + dynamic sensor

## Repository Structure

```
├── python/              Python SDK, terminal monitor, and web viewer
├── sdk_cpp/             C++ SDK for direct sensor access
├── firmware/            Standalone GPL-3.0 bootloader/flashing tool (see firmware/LICENSE)
└── utils/               Device setup and platform utilities
```

### [Quick start python](python/)

Run the unified setup script from utils/scripts to configure udev rules, serial permissions, and install the robotiq_tactile_sensor Python package:

```
# 1. Run environment & hardware setup
./utils/scripts/run_quick_install.sh

# 2. Activate virtual environment
source python/.venv/bin/activate

# 3. Launch terminal monitor or web viewer
python python/sensor_monitor.py
python python/web_viewer.py
```

### Installation

To install directly into your own existing Python environment:

```
cd python
pip install -r requirements.txt
pip install -e .
```

### [sdk_cpp](sdk_cpp/)

Lightweight C++ library with a threaded, callback-based API. A good starting point for developing with the tactile sensors. Includes a terminal visualizer (`Quick_start.cpp`) and diagnostic tools. See [sdk_cpp/README.md](sdk_cpp/README.md).

```bash
cd sdk_cpp
bash setup_and_run.sh
```

### [utils](utils/)

Platform utilities for device setup:
- **Linux**: udev rules, device detection, permission scripts
- **Windows**: WSL, USB passthrough, Docker Desktop setup

## Requirements

| Component | Dependencies |
|-----------|-------------|
| sdk_cpp | C++11 compiler, CMake 3.10+, libserialport |
| robotiq_tactile_sensor | Python 3.7+, pyserial |
| tactile_sensor_ui | Qt, CMake (or Docker) |

## Troubleshooting

### Sensor Not Found

#### Linux:

- Check USB connection
- Verify sensor is plugged in
- Try different USB port

#### Windows:

- Check Device Manager (Win+X → Device Manager → Ports)
- Sensor appears as "USB Serial Device" or "Cypress USB UART"
- Try different USB port
- VM users: USB passthrough may not work reliably for serial devices

### No Data Displayed

- Unplug and replug the sensor
- Close terminal and rerun script
- Sensor may need to be reset

### Python Not Found (Windows)
- Install Python from python.org
- Must check "Add Python to PATH" during installation
- Restart command prompt after installing

## License

This project is licensed under the BSD 3-Clause License. See [LICENSE](LICENSE) for details.

**Exception:** `firmware/bootloader_host.py` is a derivative of
[cyrozap/Cypress-HID-Bootloader-Host](https://github.com/cyrozap/Cypress-HID-Bootloader-Host)
and is licensed under the GNU General Public License v3.0, not BSD-3-Clause. See
[firmware/LICENSE](firmware/LICENSE). It is a standalone firmware-flashing tool;
the SDK and other components neither import nor link it.
