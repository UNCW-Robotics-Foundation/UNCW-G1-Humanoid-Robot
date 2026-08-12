#!/bin/bash

set -e

#source /home/unitree_ros2/setup.sh && cd UNCW-G1-Humanoid-Robot/ && source ./install/local_setup.bash && ./install/g1_lower/bin/wireless_controller_test
#source /home/unitree_ros2/setup.sh && cd UNCW-G1-Humanoid-Robot/ && source ./install/local_setup.bash && ros2 launch g1_lower g1_xbox_launch.xml
source /home/unitree_ros2/setup.sh && cd UNCW-G1-Humanoid-Robot/ && source ./install/local_setup.bash && ros2 launch g1_lower g1_gestures_launch.xml    #real
#source /home/unitree_ros2/setup_default.sh && export ROS_DOMAIN_ID=1 && cd UNCW-G1-Humanoid-Robot/ && source ./install/local_setup.bash && ros2 launch g1_lower g1_gestures_launch.xml #sim

exec "$@"