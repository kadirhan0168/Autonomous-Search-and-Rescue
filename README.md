# Autonomous LED-Tracking car controller via MQTT & openCV

A system that autonomously controls cars using LED recognition with OpenCV, real-time video streaming via Flask, and communication over MQTT. Each car receives orientation and position data, calculates the distance and direction to a target, and avoids collisions with other vehicles.

---

## Table of contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [MQTT Topics](#mqtt-topics)
- [CarController Code Explained](#carcontroller-code-explained)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Overview

This project automates vehicle movement based on camera input and color-based LED detection. Each car follows a target (e.g., a block with a yellow LED) and avoids other vehicles in real time. Control is handled through MQTT commands.

---

## Features

- Real-time video stream & LED detection
- Per-car control via MQTT
- Dynamic navigation toward a target object
- Collision avoidance when approaching another vehicle
- "Silent mode" upon receiving STOP signal
- Color recognition using HSV thresholds

---

## System architecture

```
[Webcam + OpenCV (Flask)] → Detects positions
            │
            ▼
     MQTT Broker (Mosquitto)
      ▲              ▲
      │              │
[car_client.py]   [car_client.py]
 (Car 1)            (Car 2)
```

---

## Requirements

- Python 3.7+
- MQTT broker (e.g., Mosquitto)
- Webcam
- Cars with LEDs (red/green and blue/white)
- A block with a yellow LED
- Required Python libraries:

```bash
pip install flask opencv-python numpy paho-mqtt
```

---

## Installation

### 1. Find your IP address (for the MQTT broker):

On macOS:
```bash
ipconfig getifaddr en0
```

On Linux:
```bash
hostname -I
```

### 2. Set your IP in `car_client.py`

Find the following block in `car_client.py` and replace `""` with your **own IP address**:

```python
client.connect("YOUR_IP_HERE", 1883, 60)
```

### 3. Start MQTT broker (Mosquitto)

Start Mosquitto with this command (enter password if prompted):

```bash
sudo mosquitto -c /usr/local/etc/mosquitto/mosquitto.conf
```

> Make sure mosquitto is installed. Install via `brew install mosquitto` (Mac) or `sudo apt install mosquitto` (Linux)

---

## Usage

1. Start the MQTT broker (see above)
2. Start the Flask & OpenCV server:
```bash
python app.py
```

3. Start the car controller(s):
```bash
python car_client.py red
python car_client.py blue
```

---

## Project structure

```
project/
├── app.py                # Webcam + OpenCV + Flask detection server
├── car_client.py         # Car controller via MQTT
├── templates/
│   └── index.html        # Web interface for livestream
└── README.md             # Documentation
```

---

## MQTT topics

| Topic                       | Description                             | Payload                                |
|----------------------------|-----------------------------------------|----------------------------------------|
| `cars/orientation/red`     | Position + angle of red car             | `{"x": ..., "y": ..., "angle": ...}`   |
| `cars/orientation/blue`    | Position + angle of blue car            | `{"x": ..., "y": ..., "angle": ...}`   |
| `drone/block`              | Position of the target block            | `{"x": ..., "y": ...}`                 |
| `cars/control/<color>/`    | Movement command for the car            | `"forward"`, `"left"`, `"right"`, etc. |
| `cars/status/red`          | Special status (e.g., `"stop"`)         | `"stop"`                               |

---

## Carcontroller code explained (`car_client.py`)

This module controls an individual car (color `red` or `blue`). When starting the script, you provide the color as an argument.

### Functionality

- **Positioning**: Processes MQTT messages about its own position, the other car, and the target.
- **Navigation**: Calculates direction and distance to the target and chooses the appropriate control command.
- **Collision avoidance**: If another car is too close (< 80 pixels), it automatically evades.

### MQTT connection

```python
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect("YOUR_IP_HERE", 1883, 60)
client.loop_forever()
```

---


