# Raspberry Pi Setup (Companion Computer)

This setup turns the Pi Zero 2 W into a companion computer with ROS2 and automated scripts for MAVROS and logging.

---

## Folder Contents

setup.sh: Installs necessary packages and configures the system.

camera_setup.sh: Configuration of the Pi Camera for visual monitoring.

mavros_logger.py: MAVROS installation and configuration steps.

ulog_logger.sh:(not working) Logging telemetry data from PX4 with ulog files.

systemd/: Contains *.service files to autostart logging and MAVROS communication.

rosbag/:(not working) Scripts and topic configuration for bagging MAVROS data.

logs/: Directory for storing automatic recorded data.

---
## 1. Hardware Used

Raspberry Pi Zero 2 W

MicroSD Card (16GB+) with a good grade

5V 2.5A power supply

wires to connect to the UART on the kakute H7 min 

Optional: Pi Camera Module v2

---


## 2. First Boot Setup

After flashing Ubuntu Server 22.04 to SD:

1. Boot the Pi
2. SSH in via `dronepi.local` or your router IP
3. Run the Setup Script

```bash
cd ~/pi_setup
chmod +x setup.sh
./setup.sh

(this might take some time to setup, grap a cup of coffe and nap)


