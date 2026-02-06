#!/usr/bin/env bash
set -euo pipefail

# Rubik Pi provisioning script for JetBot ROS2 stack (Ubuntu 24.04 / ROS2 Jazzy)
# Run on the Rubik Pi after flashing Canonical Ubuntu 24.04.

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root (sudo)."
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y --no-install-recommends \
  curl \
  gnupg \
  lsb-release \
  software-properties-common \
  python3-pip \
  python3-venv

# ROS2 Jazzy repo for Ubuntu 24.04 (noble)
install -d /etc/apt/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

apt-get update
apt-get install -y --no-install-recommends \
  ros-jazzy-ros-base \
  ros-jazzy-v4l2-camera \
  python3-colcon-common-extensions \
  python3-rosdep \
  git \
  udev

# Initialize rosdep (ignore if already done)
rosdep init 2>/dev/null || true
rosdep update || true

python3 -m pip install --upgrade pip
python3 -m pip install \
  --break-system-packages \
  sparkfun-qwiic \
  Adafruit-SSD1306

apt-get clean
rm -rf /var/lib/apt/lists/*

echo "Provisioning complete. Reboot recommended."