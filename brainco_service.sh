#!/bin/bash

export ROS_DOMAIN_ID=0
source /opt/ros/foxy/setup.bash
source /home/$USER/UNCW-G1-Humanoid-Robot/install/setup.bash
exec ros2 launch ros2_stark_controller stark_launch.py
