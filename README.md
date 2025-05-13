# SC2079 MDP Group 4

This project was developed as part of the NTU SC2079 Multi-Disciplinary Design Project (MDP).

This repository contains all the core components of the project. The project integrates autonomous robotics, mobile application control, embedded systems, and AI-based image recognition to complete two practical tasks.

## Repository Structure

- `algorithm` - Path planning and command generation for Task 1
- `android` - Android app for Bluetooth-based control
- `imgrpi` - Image recognition and Raspberry Pi (RPI) code
- `stm` - Embedded code for STM32 motor controller

## Overview

### Task 1: Autonomous Navigation + Image Recognition

- Robot navigates autonomously based on a generated path.
- Uses obstacle data sent from the Android app.
- Snaps images on obstacles, classifies them with a local model.
- Sends detection results back to the Android interface.

### Task 2: Fastest Car Challenge

- Navigate from start to goal based on real-time obstacle detection.
- Snaps images on obstacles for directions.
- Uses STM sensor inputs for fine tuned movement.

### Components of the project

| Module            | Responsibility                                |
| ----------------- | --------------------------------------------- |
| Android           | GUI and Bluetooth control                     |
| Algorithm         | Path computation                              |
| Image Recognition | Image recognition                             |
| RPI               | Command logic                                 |
| STM32             | Hardware execution, motion and sensor control |

## Repository Contents

### `algorithm/`

- Flask server to receive obstacle positions.
- Generates optimized path and STM32-readable command sequence.
- Endpoint `/generate_path` and `/commands`

> Python-based path planning for Task 1.

### `android/`

- Android Studio project.
- General - Allows users to:
  - Manually send command inputs for debugging.
  - Send custom strings if required.
  - Receive reply/debug messages from RPi
- Task 1 Features:
  - Draw obstacles on a 2D grid.
  - Send obstacles and start signal to the robot via Bluetooth.
  - Display robot position, image detection results.
- Task 2 Feature: Initiates Task 2 run from the app

> Java + XML-based GUI for robot control and monitoring.

### `imgrpi/`

- `rpi_task1.py` and `rpi_task2.py` runs on the RPI.
- Captures images via PiCamera.
- Image recognition server (`img_apitask1.py` and `img_apitask2.py`) runs on a team member's laptop.
- RPI receives and forwards recognition results to Android.
- Task 2 - RPI uses results to determine direction for movement.

> Real-time object detection during navigation.

### `stm/`

- STM32 embedded firmware.
- Receives commands via UART from RPi.
- Controls hardware such as wheels (via PWM) and reads IR sensors.
- Sends `ACK` after each movement.

> Robot motor control + sensor functions.
