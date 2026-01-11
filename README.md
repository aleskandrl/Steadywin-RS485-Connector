# Steadywin RS485 Connector 🤖

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![Platform](https://img.shields.io/badge/platform-Windows-lightgrey.svg)
![Status](https://img.shields.io/badge/status-Stable-green.svg)

**Steadywin RS485 Connector** is a C++ library and utility for interfacing with Steadywin motors via the RS485 protocol. It provides a reliable abstraction layer for motor telemetry, control, and error handling.

![Steadywin Connector UI](screenshot.png)

This repository contains the core communication protocol, hardware abstraction for Windows serial ports, and a ready-to-use telemetry reader.

### Architecture

The project is organized into the following components:

#### 1. Core Protocol (`steadywin_protocol`)
Implementation of the Steadywin RS485 binary protocol.
*   **Command Generation:** Automated packet construction for position, speed, and torque control.
*   **Response Parsing:** Robust decoding of motor feedback packets.
*   **CRC Validation:** Integrated checksum verification for data integrity.

#### 2. Hardware Abstraction (`windows_serial_port`)
A Windows-specific implementation of the serial interface.
*   **Low-latency I/O:** Optimized for real-time motor control loops.
*   **Auto-discovery:** Tools for identifying and connecting to COM ports.

#### 3. Telemetry Utility (`steadywin_connector`)
The main application (formerly `read_telemetry_test`) for real-time monitoring.
*   **Live Data:** Reads position, velocity, and current from the motor.
*   **Diagnostics:** Displays motor error codes and status flags in real-time.

---

### Getting Started

#### Prerequisites
*   Windows 10/11
*   CMake 3.10+
*   GCC or equivalent C++17 compiler
*   RS485 to USB adapter

#### Build
```bash
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

#### Quick Start
You can find pre-compiled binaries in the `Release/` folder.
1. Connect your Steadywin motor via RS485.
2. Run `steadywin_connector.exe`.
3. The utility will begin polling telemetry data from the motor.

### Releases
The `Release/` directory contains the latest stable executables:
*   `steadywin_connector.exe`: Main telemetry and diagnostic tool.
*   `motor_tester.exe`: Basic functional testing utility.
*   `profile_move_example.exe`: Example of motion profile execution.

---
*Note: This project is part of the HexaKinetica ecosystem.*
