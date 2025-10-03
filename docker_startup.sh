#!/bin/bash
export ROS_DOMAIN_ID=0
echo 'Starting SAM2 ROS Node...'
source /root/ros2_ws/install/setup.bash
ros2 run sam2_server server