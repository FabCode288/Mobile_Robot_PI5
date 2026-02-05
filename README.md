# Mobile_Robot_PI5

This repository contains the software running on the **Raspberry Pi 5** of a mobile robot system.
The Raspberry Pi is responsible for the **ROS2-based control architecture** and direct control of
the robot hardware, including motor control.

---

## System Context

The robot system uses a distributed architecture.

A separate Jetson-based system is used for camera input, image recognition, and AI-based processing.
Processed data (e.g. detected persons or path-related information) is transmitted to the Raspberry Pi
via ROS2 topics.

Related repository (context only):
https://github.com/ManuKaiser/Mensa_Roboter_Jetson

This repository focuses exclusively on the Raspberry Pi side of the system.

---

## Scope of This Repository

The Raspberry Pi acts as the central **ROS2 control unit**.

Current responsibilities:
- ROS2 node architecture
- Robot steering
- Motor control
- Execution of movement commands based on ROS2 messages

Not included in this repository:
- Camera handling
- Image processing
- AI or machine learning models

---

## Current State

The current implementation focuses on:
- Establishing a stable ROS2-based control structure
- Hardware-level control of the robot platform
- Communication with external ROS2 nodes

---

## Future Work

Planned extensions include:
- Navigation
- SLAM
- Extended autonomy within the ROS2 framework

These features are not part of the current implementation.

---

## Hardware Requirements

- Raspberry Pi 5
- Compatible motor drivers
- Robot platform hardware

---

## Software Requirements

- Raspberry Pi 5
- ROS2

---

## Setup

Clone the repository on the Raspberry Pi:

```bash
git clone https://github.com/FabCode288/Mobile_Robot_PI5.git
cd Mobile_Robot_PI5

