#!/bin/bash
set -e

echo "[+] Updating system..."
sudo apt update && sudo apt upgrade -y

echo "[+] Installing ROS 2 and MAVROS..."
sudo apt install -y curl git python3-pip cmake build-essential \
  libeigen3-dev libboost-all-dev libopencv-dev \
  ros-humble-desktop ros-humble-mavros ros-humble-mavros-extras \
  python3-colcon-common-extensions

echo "[+] Installing libcamera and dev tools..."
sudo apt install -y libcamera-apps libcamera-dev

echo "[+] Creating swap space..."
sudo fallocate -l 1G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo "/swapfile none swap sw 0 0" | sudo tee -a /etc/fstab

echo "[+] Enabling MAVROS autostart service..."
sudo cp systemd/mavros_autostart.service /etc/systemd/system/
sudo systemctl enable mavros_autostart.service

echo "[+] Done. Reboot recommended."
