# Sub-250g Fixed-Wing Drone for Wildlife Monitoring in Africa

---

## 🧩 Repository Structure

```console
Master_thesis/
├── flight_controller/          # PX4 setup for Kakute H7 Mini
│   ├── README.md
│   └── params/
│       └── px4_flight_config.param
├── flight_tests/               # Logs, test reports, checklists
│   ├── README.md
│   ├── checklists/
│   │   └── pre_flight_check.md
│   ├── logs/
│   │   └── flight_log_template.md
├── pi_setup/                   # Companion computer setup (Pi Zero 2 W)
│   ├── README.md
│   ├── systemd/
│   │   └── mavros_autostart.service
|   |   └── px4_logger.service
│   ├── rosbag/
│   │   ├── record_mavros.sh
│   │   └── rosbag_topics.yaml
│   ├── logs/
│   ├── setup.sh
│   ├── camera_setup.sh
│   ├── mavros_logger.py
│   ├── camera_setup.sh
│   └── ulog_logger.sh
└── README.md                   # You are here
```

---

## 🚀 Quick Start Guide

### 1. Raspberry Pi Zero 2 W Setup

- Flash **Ubuntu Server 22.04** to SD card  
- Add `user-data` and `network-config` to the `system-boot` partition  
- Boot the Pi and SSH into it  

### 2. Run Setup Script

```console
cd ~/pi_setup
chmod +x setup.sh
./setup.sh
```
3. Start MAVROS
```console
sudo systemctl start mavros_autostart.service
```
4. Record Flight Data
```console
cd ~/pi_setup/rosbag
./record_mavros.sh
```
5. Capture Images + GPS Data
```console
python3 ~/pi_setup/geotag_capture.py
```
📷 Hardware Overview

Flight Controller: Kakute H7 Mini

Companion Computer: Raspberry Pi Zero 2 W

OS: Ubuntu Server 22.04 (64-bit)

Camera: Pi Camera v2

Power Supply: ESC BEC (5V 2–3A)

👨‍💻 Author
Benjamin TH. Christensen
MSc Thesis Project – 2025
University of southern Denmark
📜 License

This repository is released under the MIT License. Use it, modify it, fly responsibly.
