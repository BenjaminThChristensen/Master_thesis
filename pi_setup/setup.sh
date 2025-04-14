#!/bin/bash
set -e

echo "[+] Creating swap space..."
sudo fallocate -l 5G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo "/swapfile none swap sw 0 0" | sudo tee -a /etc/fstab


echo "[+] Updating system..."
sudo apt update && sudo apt upgrade -y

echo "[+] Installing ROS 2 and MAVROS..."
sudo apt install -y curl git python3-pip cmake build-essential \
  libeigen3-dev libboost-all-dev libopencv-dev \
  ros-humble-desktop ros-humble-mavros ros-humble-mavros-extras \
  python3-colcon-common-extensions

echo "[+] Installing libcamera and dev tools..."
sudo apt install -y libcamera-apps libcamera-dev

echo "[+] Enabling MAVROS autostart service..."
sudo cp systemd/mavros_autostart.service /etc/systemd/system/
sudo systemctl enable mavros_autostart.service

echo "[+] Installing logger files"
git clone https://github.com/PX4/PX4-Autopilot.git --depth=1 --filter=blob:none

echo "[+] Creating log directories..."
mkdir -p /home/dronepi/flight_logs/ulg
mkdir -p /home/dronepi/flight_logs/bags

echo "[+] Installing systemd services..."

SERVICE_DIR="/home/dronepi/Master_thesis/pi_setup/systemd"

sudo cp "$SERVICE_DIR/mavros_autostart.service" /etc/systemd/system/
sudo cp "$SERVICE_DIR/px4_logger.service" /etc/systemd/system/

sudo systemctl enable mavros_autostart.service
sudo systemctl enable px4_logger.service

sudo systemctl start mavros_autostart.service
sudo systemctl start px4_logger.service
echo "[+] Done. Reboot recommended."
