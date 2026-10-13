#!/bin/bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0

source /opt/ros/foxy/setup.bash
source /home/$USER/UNCW-G1-Humanoid-Robot/install/setup.bash
exec ros2 launch ros2_stark_controller stark_launch.py
