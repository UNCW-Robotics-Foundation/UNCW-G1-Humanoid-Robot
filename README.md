# UNCW-G1-Humanoid-Robot
Base scripts and foundational programs for uncw's G1 Humanoid Robot. Utilizes ROS2 as framework.

## Operating System
Linux Ubuntu 22.04

## Initial Setup
- Install Ros2 Humble
- Follow all of the setup instructions for [this repository](https://github.com/unitreerobotics/unitree_ros2).
- As long as the building steps in the other repository were successfull, and after being able to successfully connect to the robot, confirmed by seing the new topics in Ros2, this package should build correctly.

### Sorce repos for Nav2 Simulator: 
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [FAST-LIO (ROS2 branch)](https://github.com/hku-mars/FAST_LIO) → branch ROS2

## Required Dependencies
```
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui

```
### Nav2 Simulator
```
sudo apt install ros-humble-nav2-bringup 
sudo apt install ros-humble-slam-toolbox 
sudo apt install ros-humble-pointcloud-to-laserscan 
sudo apt install ros-humble-gazebo-ros-pkgs 
sudo apt install ros-humble-robot-state-publisher 
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-nav2-voxel-grid 
sudo apt install libeigen3-dev 
sudo apt install libpcl-dev

```
## Build
```
source ~/unitree_ros2/setup.sh
cd ~/UNCW-G1-Robot
colcon build
source install/local_setup.bash
```

Make sure to soure the unitree package before building since this package uses their message files.
The generated build, install, and log folders are part of the gitignore. I believe you will need to source the package everytime when using a new terminal.
The rviz simulator will not work if the unitree package is sourced, and the robot is not connected or turned off, but it will work if the robot is on.

### Nav2 Simulator Build
```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

```

## Run Commands 

### Nav2 Sim 
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch g1_nav2_sim_bringup sim_bringup_launch.py

```
Keyboard teleop commands also work for robot model
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To send 2D Nav Goals, switch the fixed frame topic from 'odom'
to 'map' and recheck global costmap. Also wait about 15 seconds to send goal as SLAM needs time to build initial map.  
NOTE: Costmaps and livox drivers use different timestamps, system may time out and cause maps to be out of sync or not receive properly in rviz.  
Fixed frames for viewing in rviz: 
- Global Costmap: map
- Local Costmap: odom
- Lidar: camera_init

