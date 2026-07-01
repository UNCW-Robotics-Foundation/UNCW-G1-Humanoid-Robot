# UNCW-G1-Humanoid-Robot
Base scripts and foundational programs for uncw's G1 Humanoid Robot. Utilizes ROS2 as framework.\

OS: Linux Ubuntu 22.04

## Initial Setup
- Install Ros2 Humble
- Follow all of the setup instructions for [this repository](https://github.com/unitreerobotics/unitree_ros2).
- As long as the building steps in the other repository were successfull, and after being able to successfully connect to the robot, confirmed by seing the new topics in Ros2, this package should build correctly.

Sorce repos for Nav2 Simulator: 
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [FAST-LIO (ROS2 branch)](https://github.com/hku-mars/FAST_LIO) → branch ROS2

## Required Dependencies
```
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui

```
Nav2 Simulator
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
