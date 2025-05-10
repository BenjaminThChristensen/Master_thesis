# Raspberry Pi Setup (Companion Computer)

This setup turns the Pi Zero 2 W into a companion computer with ROS2 and automated scripts for MAVROS and logging.

---

## ✅ Features:
- Headless Ubuntu Server boot (with Wi-Fi auto-connect)
- SSH access enabled
- ROS 2 Humble + MAVROS
- Automatic MAVROS launch at boot
- ROS bag recording script

---

## 1. First Boot Setup

After flashing Ubuntu Server 22.04 to SD:

1. Copy `cloud-init/` files into the `system-boot` partition
2. Boot the Pi
3. SSH in via `dronepi.local` or your router IP

---

## 2. Run the Setup Script

```bash
cd ~/pi_setup
chmod +x setup.sh
./setup.sh

