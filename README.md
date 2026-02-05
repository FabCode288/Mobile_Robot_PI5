# Mobile_Robot_PI5

This repository contains the software running on the Raspberry Pi 5 of a mobile robot system.
The Raspberry Pi is responsible for the ROS2-based control architecture and the direct control
of the robot hardware, including motor control and state publishing.

---

## System Context

The robot system uses a distributed architecture.

A separate Jetson-based system performs camera processing, image recognition, and AI-based tasks
such as person detection and path prediction. The processed data is transmitted to the Raspberry Pi
via ROS2 topics, where it is used for robot control.

Related repository (context only):
https://github.com/ManuKaiser/Mensa_Roboter_Jetson

This repository focuses exclusively on the Raspberry Pi side of the system.

---

## Scope of This Repository

The Raspberry Pi acts as the central ROS2 control unit of the robot.

Responsibilities include:
- ROS2 node architecture
- Robot steering
- Motor control
- Hardware-level execution
- Robot state publishing
- Controller management via ROS 2 Control

Not included in this repository:
- Camera handling
- Image processing
- AI or machine learning models

---

## Current State

The current implementation focuses on:
- Bringing up the robot using ROS2
- Loading the robot description generated from Xacro
- Initializing ROS 2 Control
- Spawning required controllers
- Publishing robot state information for other ROS2 nodes

Navigation, SLAM, and higher-level autonomy are not implemented at this stage.

---

## Launch Files

### Robot Bringup (Raspberry Pi)

The bringup launch file is used during robot startup and initializes the core components
required for robot operation.

It performs the following tasks:
- Generates the robot description from Xacro
- Starts the ROS 2 Control node
- Launches the Robot State Publisher
- Spawns the joint state broadcaster
- Spawns the base controller for robot movement

Start command:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch mensabot_bringup mensabot_launch
