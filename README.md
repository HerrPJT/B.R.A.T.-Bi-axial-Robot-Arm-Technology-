# B.R.A.T. — Bi-axial Robot Arm Technology

> A low-cost, industrial-grade robotic arm controlled by gesture glove, voice recognition, and a PS4 gamepad, with an integrated Machine Learning safety layer.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Hardware Components](#hardware-components)
- [Control Modes](#control-modes)
- [Voice Commands](#voice-commands)
- [Glove Mapping](#glove-mapping)
- [PS4 Controller Mapping](#ps4-controller-mapping)
- [Requirements](#requirements)
- [Test Results](#test-results)
- [Known Limitations](#known-limitations)
- [Cost & Pricing](#cost--pricing)
- [Intellectual Property](#intellectual-property)
- [Team](#team)

---

## Overview

B.R.A.T. is a 2-axis robotic arm prototype developed for industrial and educational contexts, targeting beverage filling line automation. The system prioritises **low cost**, **natural human-machine interfaces**, and **on-edge Machine Learning** — removing the dependency on complex programming interfaces or cloud processing.

The arm was developed as part of a 4.5-month engineering project at the University of Coimbra (February–June 2026).

---

## Features

- **3 control modes:** PS4 gamepad, sensor glove (gestures), and voice recognition
- **On-edge ML (TinyML):** gesture classification and voice keyword spotting running locally on ESP32 / Raspberry Pi 4
- **Computer vision safety layer:** camera-based object detection using a trained YOLO v8 model
- **Fail-safe behaviour:** emergency stop, movement limits, and loss-of-signal handling
- **Bluetooth (BLE) + Wi-Fi** wireless communication
- **3D-printed structure** — open hardware, fully replicable
- **Total BOM cost under €150**

---

## System Architecture

The system is divided into five subsystems:

```
┌─────────────────────────────────────────────┐
│               B.R.A.T. System               │
├──────────────┬──────────────┬───────────────┤
│     HMI      │  Processing  │  Algorithms   │
│  ──────────  │  ──────────  │  ───────────  │
│  Glove       │  ESP32-WROOM │  BLE comms    │
│  PS4 ctrl    │  Raspberry   │  Voice / KWS  │
│  Microphone  │  Pi 4 Mod B  │  Flex / IMU   │
│  Infrared    │              │  Camera / ML  │
│  Pixy Camera │              │  Arm control  │
├──────────────┴──────────────┴───────────────┤
│  Physical Structure          Power Sources  │
│  ─────────────────           ─────────────  │
│  3D-printed arm & claw       9 V batteries  │
│  Infrared radar servo                       │
│  Servomotors (TD8125MG)                     │
└─────────────────────────────────────────────┘
```

Communication flows:
- **Glove ESP32 → Robot ESP32:** Wi-Fi
- **PS4 Controller → Robot ESP32:** Bluetooth (BLE)
- **Raspberry Pi 4 → Robot ESP32:** USB (serial) — required to avoid BLE/Wi-Fi interference
- **Microphone → Raspberry Pi 4:** I2S (SPH0645)
- **Camera Module → Raspberry Pi 4:** CSI
- **Robot ESP32 → Servomotors:** GPIO / PWM

---

## Hardware Components

| Component | Purpose |
|---|---|
| ESP32-WROOM-32E | Main arm controller & glove controller |
| Raspberry Pi 4 Module B | ML inference, voice recognition, computer vision |
| Servomotors TD8125MG (25 kg·cm) | Shoulder, elbow, base, and wrist axes |
| Servomotors SG90 | Gripper claw and radar pan/tilt |
| Flex sensors | Finger position detection in glove |
| IMU (accelerometer + gyroscope) | Wrist orientation in glove |
| Microphone SPH0645 | I2S voice input |
| Camera Module 5MP 1080P | Computer vision / safety mode |
| PS4 DualShock 4 Controller | Direct gamepad control mode |
| 9 V batteries | Power supply (one per pair of TD8125MG servos) |

---

## Control Modes

Mode switching is handled by the PS4 controller buttons:

| Button | Action |
|---|---|
| **△ (Triangle)** | Switch to Glove mode |
| **○ (Circle)** | Switch to Voice Recognition mode |
| **□ (Square)** | Switch to PS4 Controller mode |
| **✕ (Cross)** | Emergency stop / return to home position |

---

## Voice Commands

The voice model was trained on Edge Impulse with 14 keyword classes.

| Command | Action |
|---|---|
| `on` | Detect can, grab it, place it — continuous loop |
| `off` | Stop loop, return to home position |
| `grab` | Detect and grab a can, move to rest position |
| `go` | Place held can at destination |
| `abort` | Return to home position |
| `stop` | Halt all movement |
| `forward` | Move arm forward |
| `back` | Move arm backward |
| `left` | Rotate base left |
| `right` | Rotate base right |
| `open` | Open gripper |
| `close` | Close gripper |
| `noise` | (Training class — background noise) |
| `unknown` | (Training class — unrecognised words) |

---

## Glove Mapping

| Gesture | Arm action |
|---|---|
| Thumb flex | Open / close gripper |
| Index + middle + ring flex | Shoulder servo |
| Tilt wrist up / down | Elbow servo |
| Tilt wrist left / right | Base rotation |

---

## PS4 Controller Mapping

| Input | Action |
|---|---|
| PS + Share | Power on controller |
| Left analogue stick | Elbow servo |
| Right analogue stick | Shoulder servo |
| D-pad left / right | Base rotation |
| L2 (left trigger) | Open gripper progressively |
| R2 (right trigger) | Close gripper progressively |
| ✕ (Cross) | Emergency stop / home position |
| ○ (Circle) | Switch to Voice mode |
| △ (Triangle) | Switch to Glove mode |
| □ (Square) | Switch to Controller mode |
| Options | Power off controller |

---

## Requirements

| ID | Requirement | Status |
|---|---|---|
| REQ001 | System shall use ML to identify voice commands | ✅ Met |
| REQ002 | System shall move in response to glove gestures | ✅ Met |
| REQ003 | System shall execute tasks from voice commands | ✅ Met |
| REQ004 | Total budget shall not exceed €300 | ✅ Met |
| REQ005 | Arm control software shall be implemented in C++ | ✅ Met |
| REQ006 | System shall communicate wirelessly with its controls | ✅ Met |
| REQ007 | System shall be powered by interchangeable batteries | ✅ Met |
| REQ008 | Power supply voltage shall not exceed 12 V DC | ❌ Not met — 18–24 V required (see [Known Limitations](#known-limitations)) |
| REQ009 | Base footprint shall not exceed 50 cm × 50 cm | ✅ Met |
| REQ010 | Claw shall transport a 330 ml can | ✅ Met |
| REQ011 | Arm shall transport at least 5 cans per minute | ❌ Not met |

---

## Test Results

| Test | Description | Result |
|---|---|---|
| TC001 | ML identifies voice commands under ambient noise | ✅ Pass |
| TC002 | Glove gestures replicate on the arm and gripper | ✅ Pass |
| TC003 | Voice keyword triggers correct arm action | ✅ Pass |
| TC004 | Total BOM cost within €300 | ✅ Pass |
| TC005 | All firmware written in C++ | ✅ Pass |
| TC006 | BLE latency ≤ 300 ms | ✅ Pass |
| TC007 | System runs on removable batteries for ~60 min | ✅ Pass |
| TC008 | Supply voltage ≤ 12 V DC | ❌ Fail — 18–24 V needed |
| TC009 | Base area ≤ 50 cm × 50 cm | ✅ Pass |
| TC010 | Gripper transports a full 330 ml can without dropping | ✅ Pass |
| TC011 | Arm transports ≥ 5 cans/minute | ❌ Fail |

---

## Known Limitations

**Voltage (REQ008):** The TD8125MG servos (25 kg·cm torque) require peak currents that cause system failure below 9 V per servo pair. Two independent 9 V sources are used for the four arm servos, bringing the total operating voltage to 18–24 V. This formally violates REQ008 but was accepted to ensure mechanical integrity.

**Cans per minute (REQ011):** The 5 cans/minute throughput target was not achieved with the current motion profile and gripper cycle time.

**Pixy Camera incompatibility:** The original Pixy R1.3A camera was found to be incompatible with Raspberry Pi 4 over I2C/SPI (persistent data transfer timeout). It was replaced with a 5MP 1080P Camera Module.

**BLE/Wi-Fi interference:** Simultaneous BLE (PS4) and Wi-Fi (glove) communication caused data conflicts on the ESP32. The Raspberry Pi 4 is therefore connected to the robot ESP32 via USB serial.

**Safety mode scope:** The person-detection safety stop was removed from the automated operating mode because the camera is directed at the workspace (facing the arm) to calculate can coordinates, making person detection in that orientation impractical without a second camera.

**Claw friction:** The original 3D-printed gripper surface had insufficient friction for aluminium cans. Solved by applying weatherstrip foam tape to the inner contact surfaces.

---

## Cost & Pricing

| Item | Cost (€) |
|---|---|
| 3D printing — Robotics Club | 21.05 |
| Servomotors 180° | 37.49 |
| 3D printing — Ogami | 12.00 |
| Gloves + fasteners | 14.97 |
| Structure printing | 11.00 |
| Screws | 14.46 |
| Camera | 3.81 |
| Bearings | 17.18 |
| BB-style balls | 6.59 |
| **Total BOM** | **138.55** |

Suggested retail price (30% margin + 23% VAT): **€211.99**

---

## Intellectual Property

**Software:** All ESP32 firmware and image processing logic is released under the [MIT License](LICENSE).

**Hardware:** Structural models and assembly diagrams are released as Open Hardware under [Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/).

The base arm model used as a starting point for 3D printing:
🔗 https://www.printables.com/model/393600-robotic-arm

No patent-registrable inventive elements were identified — the solution uses standard kinematic mechanisms and off-the-shelf electronic components.

---

## Team

| Name | Student No. | Contact |
|---|---|---|
| David Ribeiro | 2020217631 | uc2020217631@student.uc.pt |
| Gonçalo Batista | 2023225470 | uc2023225470@student.uc.pt |
| Pedro Tomás | 2023211605 | uc2023211605@student.uc.pt |
| Samuel Figueira | 2023211662 | uc2023211662@student.uc.pt |

**Project period:** February 12 – June 25, 2026  
**Institution:** University of Coimbra  
**Course:** Projeto II (2025/2026)
