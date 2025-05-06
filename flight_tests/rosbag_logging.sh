#!/bin/bash

# Delay to ensure MAVROS is fully up and publishing
echo "[INFO] Waiting for MAVROS to start..."
sleep 5

# Directory and filename for rosbag
LOG_DIR=~/rosbags
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
LOG_NAME="flight_log_$TIMESTAMP"

mkdir -p "$LOG_DIR"

# Start rosbag with selected topics
echo "[INFO] Starting rosbag record: $LOG_NAME"

ros2 bag record \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/local_position/velocity_local \
  /mavros/global_position/global \
  /mavros/global_position/raw/fix \
  /mavros/global_position/rel_alt \
  /mavros/imu/data \
  /mavros/imu/data_raw \
  /mavros/battery \
  /mavros/sys_status \
  /mavros/diagnostics \
  /mavros/time_reference \
  /mavros/mission/reached \
  /mavros/mission/waypoints \
  /mavros/altitude \
  /mavros/vibration/raw/vibration \
  -o "$LOG_DIR/$LOG_NAME"
