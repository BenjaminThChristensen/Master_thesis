#!/bin/bash
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
FOLDER=~/flight_logs/bags/$TIMESTAMP
mkdir -p $FOLDER
cd $FOLDER

echo "[+] Recording ROS bag..."
screen -dmS rosbag ros2 bag record \
  /mavros/global_position/global \
  /mavros/imu/data \
  /mavros/local_position/pose \
  /mavros/state
