#!/bin/bash

# ============================================================

# ROS2 Jazzy Jalisco Setup für Raspberry Pi 5

# Installiert alle notwendigen Pakete für einen mobilen Roboter

# ohne SLAM, inkl. ROS2 Core, Controler, URDF/Xacro und GPIO-Tools

# ============================================================

set -e  # Script sofort stoppen, falls ein Fehler auftritt

echo ">>> 1. Systemupdate und Upgrade"
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y
sudo apt clean

echo ">>> 2. Grundlegende Pakete installieren (Locales, curl, gnupg, lsb-release)"
sudo apt install -y locales curl gnupg2 lsb-release software-properties-common
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt install
sudo apt update

echo ">>> 3. Locale auf en_US.UTF-8 setzen"
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo ">>> 4. SSH aktivieren und starten"
sudo systemctl enable ssh
sudo systemctl start ssh

echo ">>> 5. Build-Tools und Python-Pakete installieren"
sudo apt install -y build-essential cmake git python3-pip
sudo apt install -y python3-rosdep python3-colcon-common-extensions python3-vcstool python3-argcomplete python3-empy python3-numpy python3-yaml

echo ">>> 6. rosdep initialisieren"
sudo rosdep init || true  # Fehler ignorieren, falls bereits init ausgeführt
rosdep update

echo ">>> 7. ROS2 Jazzy Base installieren"
sudo apt install -y ros-jazzy-ros-base

echo ">>> 8. ROS2 Core-Pakete installieren (TF, Sensor, Geometry, Nav)"
sudo apt install -y ros-jazzy-tf2-tools ros-jazzy-tf2-ros ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-nav-msgs

echo ">>> 9. ROS2 Controler installieren (ros2_control, Controllers)"
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joint-state-broadcaster

echo ">>> 10. URDF/Xacro und Robot-State-Publisher installieren"
sudo apt install -y ros-jazzy-robot-state-publisher ros-jazzy-urdf ros-jazzy-xacro ros-jazzy-cv-bridge

echo ">>> 11. CycloneDDS installieren"
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/student/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0

echo ">>> 12. ROS2 Environment setzen"
if ! grep -Fxq "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi
if ! grep -Fxq "export ROS_DOMAIN_ID=3" ~/.bashrc; then
echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc
fi
source ~/.bashrc

echo ">>> ✅ Installation abgeschlossen! ROS2 Jazzy ist einsatzbereit."
