#!/bin/bash
# ROS and logger startup wrapper

source /opt/ros/humble/setup.bash
export PATH="$HOME/.local/bin:$PATH"

# Start the MAVLink logger
exec python3 /home/dronepi/PX4-Autopilot/Tools/mavlink_ulog_streaming.py /dev/serial0 --baudrate 115200 --output /home/dronepi/flight_logs/ulg
