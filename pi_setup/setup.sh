#!/bin/bash
set -e

echo "[+] Creating swap space..."
if [ -f /swapfile ]; then
    echo "[!] Swapfile already exists, skipping."
else
    sudo fallocate -l 5G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    echo "/swapfile none swap sw 0 0" | sudo tee -a /etc/fstab
fi

echo "[+] Updating system..."
sudo apt update && sudo apt upgrade -y

#setting up locales
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[+] Adding ROS 2 Humble sources..."
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo tee /etc/apt/trusted.gpg.d/ros.asc > /dev/null

# Add ROS 2 repo
echo "deb http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

echo "[+] Updating package list with ROS sources..."
sudo apt update && sudo apt upgrade -y

echo "[+] Installing minimal ROS 2 (ros-base + MAVROS)..."
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  python3-colcon-common-extensions
  
echo "[+] Sourcing ROS 2 (Humble) environment..."
source /opt/ros/humble/setup.bash
if ! grep -Fxq "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  echo "[+] Added ROS source line to ~/.bashrc"
else
  echo "[+] ROS source line already in ~/.bashrc"
fi

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
