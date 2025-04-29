#!/bin/bash
set -e

# --- Create a large swapfile (important for compiling or big tasks) ---
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

echo "[+] Updating system packages..."
sudo apt update && sudo apt upgrade -y

#install dependencies
echo "[+] Installing build dependencies..."
sudo apt install -y \
    git cmake ninja-build meson python3-pip python3-ply \
    libjpeg-dev libtiff5-dev libavcodec-dev libavformat-dev libswscale-dev \
    libdrm-dev libevent-dev libyaml-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libudev-dev libboost-all-dev
    
sudo apt install -y libcamera-dev v4l-utils
#installing the compile tool to install from source
echo "[+] Upgrading Meson if needed..."
pip3 install --user meson ninja


echo "[+] Cloning libcamera..."
cd ~
if [ -d "libcamera" ]; then
  echo "[!] libcamera already cloned. Skipping..."
else
  git clone https://git.libcamera.org/libcamera/libcamera.git
fi
#installing the functionalities to use rpi-cam
echo "[+] Building libcamera..."
cd ~/libcamera
rm -rf build
meson setup build || { echo "[!] Meson setup failed! Exiting."; exit 1; }
meson compile -C build
sudo meson install -C build

#installing the 
echo "[+] Cloning rpicam-apps..."
cd ~
if [ -d "rpicam-apps" ]; then
  echo "[!] rpicam-apps already cloned. Skipping..."
else
  git clone https://github.com/raspberrypi/rpicam-apps.git
fi

echo "[+] Building rpicam-apps..."
cd ~/rpicam-apps
rm -rf build
meson setup build || { echo "[!] Meson setup failed! Exiting."; exit 1; }
meson compile -C build
sudo meson install -C build

echo "[+] Setup Complete. Reboot recommended."
