# Autonomous LED-Tracking car controller via MQTT & openCV

A system that autonomously controls cars using LED recognition with OpenCV, real-time video streaming via Flask, and communication over MQTT. Each car receives orientation and position data. Then, the distance and direction to the target is calculated. Finally, the vehicles navigate toward the target while avoiding collisions with each other.

This system was built as part of a robotics/autonomous systems course and can be applied to warehouse bots, search-and-rescue testing environments, or educational platforms.

It’s designed to work both:
- In the **real world**, using LED-equipped Chariot vehicles
- In **Webots simulation**, enabling development without hardware

---

## Table of contents

- [Autonomous LED-Tracking car controller via MQTT \& openCV](#autonomous-led-tracking-car-controller-via-mqtt--opencv)
  - [Table of contents](#table-of-contents)
  - [Overview](#overview)
  - [Features](#features)
  - [System architecture](#system-architecture)
  - [Requirements](#requirements)
  - [Installation](#installation)
    - [1. Find your IP address (for the MQTT broker):](#1-find-your-ip-address-for-the-mqtt-broker)
    - [2. Set your IP in `car_client.py`](#2-set-your-ip-in-car_clientpy)
    - [3. Start MQTT broker (Mosquitto)](#3-start-mqtt-broker-mosquitto)
    - [4. Flash and set up Raspberry Pi Pico (Chariots)](#4-flash-and-set-up-raspberry-pi-pico-chariots)
  - [Usage](#usage)
    - [Central server \& image processing](#central-server--image-processing)
    - [Chariots (Raspberry Pi Pico)](#chariots-raspberry-pi-pico)
    - [Webots](#webots)
  - [Project structure](#project-structure)
  - [MQTT topics](#mqtt-topics)
  - [Carcontroller code explained (`car_client.py`)](#carcontroller-code-explained-car_clientpy)
    - [Functionality](#functionality)
    - [MQTT connection](#mqtt-connection)
  - [Chariot code explained (``mainB.py`` \& ``mainR.py``)](#chariot-code-explained-mainbpy--mainrpy)
    - [Functionality](#functionality-1)
  - [Webots simulation explained](#webots-simulation-explained)
    - [Running the Webots simulation](#running-the-webots-simulation)

---

## Overview

This project automates vehicle movement based on camera input and color-based LED detection. Each car follows a target (e.g., a block with a yellow LED) and avoids other vehicles in real time. Control is handled through MQTT commands.

---

## Features

- Real-time video stream & LED detection using OpenCV
- Individual car control via MQTT
- Dynamic navigation toward a target LED
- Collision avoidance when approaching another vehicle
- "Silent mode" upon receiving STOP signal
- Color recognition using adjustable HSV thresholds

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
- Webots
- MQTT broker (e.g., Mosquitto)
- Camera
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

### 4. Flash and set up Raspberry Pi Pico (Chariots)

- Flash **MicroPython** onto your Raspberry Pi Pico (W) if you haven’t already
- Upload the control script (`mainR.py`, `mainB.py`, etc.) using [Thonny](https://thonny.org/) or another MicroPython IDE
- In the script, update:
  - `ssid` and `password` with your **Wi-Fi credentials**
  - `mqtt_broker` with your **IP address**
  - `mqtt_topic` based on Chariot color (e.g., `cars/control/red/`)

---

## Usage

### Central server & image processing

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

### Chariots (Raspberry Pi Pico)

After flashing and uploading the correct script:

1. Power the Chariot.
2. It will start navigating if the WiFi and MQTT connections are set up, and the target is visible.

### Webots

1. Start the Webots world to get the location and orientation of the Chariots and target block.
2. Click the play button again to make the Chariots listen to navigation commands received over MQTT.

---

## Project structure

```
Autonomous-Search-and-Rescue/
├── central-server/
│   ├── templates/
│   │   └── index.html              # Web interface for livestream
│   ├── app.py                      # Webcam + OpenCV + Flask detection server
│   ├── car_client.py               # Car controller via MQTT
│   └── mosquitto.conf              # Mosquitto config (local broker)
│
├── hardware/
│   ├── 3D print/
│   │   └── design-ultrasonic.stl   # Sensor mount
│   └── chariots/
│       ├── mainB.py                # Code for Blue Chariot
│       └── mainR.py                # Code for Red Chariot
│
├── webots/
│   └── simulation/
│       ├── worlds/                 # Simulation world
│       ├── controllers/            # Python controllers for central server, cars, spawner
│       └── protos/                 # Chariot models
│
└── README.md
```

---

## MQTT topics

| Topic                       | Description                             | Payload                                |
|----------------------------|-----------------------------------------|----------------------------------------|
| `cars/orientation/red`     | Position + angle of red car             | `{"x": ..., "y": ..., "angle": ...}`   |
| `cars/orientation/blue`    | Position + angle of blue car            | `{"x": ..., "y": ..., "angle": ...}`   |
| `drone/block`              | Position of the target block            | `{"x": ..., "y": ...}`                 |
| `cars/control/<color>/`    | Movement command for the car            | `"forward"`, `"left"`, `"right"`, etc. |
| `cars/status/<color>`          | Special status (e.g., `"stop"`)         | `"stop"`                               |

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

## Chariot code explained (``mainB.py`` & ``mainR.py``)

### Functionality

The Chariots listen to MQTT messages from the central server and executes the navigation commands. On detecting an obstacle (distance between 10-18 cm), it:

1. Stops and scans both directions
2. Chooses the path with more clearance
3. Turns accordingly and continues moving

All motion is controlled through PWM duty cycles to the motors

---

## Webots simulation explained

The Webots simulation consists of two parts:
1. **Entity spawner (initialization phase)**
   When you first press "Play" in Webots, the simulation runs a script that listens for the latest MQTT messages containing the location and orientation of the Chariots, as well as the LED target block. Based on this data, it spawns and places the entities at their correct positions inside the Webots world. This ensures the simulation mirrors the real-world scenario or external vision input.

2. **Simulation (navigation phase)**
   When you press "Play" again, the entity spawner finishes, and the chariots activate. Each chariot starts subscribing to MQTT control messages, just like the real hardware.
   They interpret movement commands such as "forward", "left", "stop", etc., and navigate toward the target, avoiding collisions based on their controller logic.

### Running the Webots simulation

1. Open Webots and load the project world from webots/simulation/worlds/
2. Press Play once → Entities are spawned and placed correctly
3. Press Play again → Chariots begin listening to MQTT and start navigating

---