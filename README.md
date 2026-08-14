# UNCW-G1-Humanoid-Robot
Base scripts and foundational programs for uncw's G1 Humanoid Robot. Utilizes ROS2 as framework.

## Operating System
Linux Ubuntu 22.04

## Initial Setup
- Install Ros2 Humble
- Follow all of the setup instructions for [this repository](https://github.com/unitreerobotics/unitree_ros2).
- As long as the building steps in the other repository were successfull, and after being able to successfully connect to the robot, confirmed by seing the new topics in Ros2, this package should build correctly.

### Sorce repos for Nav2 Simulator: 
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) — follow install instructions on repo page
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
cd ~/UNCW-G1-Humanoid-Robot
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

To send 2D Nav Goals, switch the fixed frame topic from 'odom'to 'map' and recheck global costmap, the goal pose is published in the current fixed
frame, and Nav2 plans in the map frame. Also wait about 15 seconds to send goal as SLAM needs time to build initial map.  
NOTE: Costmaps and gazebo sim use different timestamps (system and sim time respectively), system may time out and cause maps to be out of sync or not receive properly in rviz.  
Fixed frames for viewing in rviz: 
- Global Costmap: map
- Local Costmap: odom
- Lidar: camera_init

Robot model will "glide" along the floor, this is intentional as bipedal simulation is heavy on processing. Nav2 reads velocity cmds as smooth, real robot will translate cmd_vel to bipedal motion  
Failed to find match for field errors from fastlio mapping will show in run terminal for sim, these are harmless warnings, havent figured out a way to disable them unfortunately.   
In cafe.world, tables and other thin fixtures are viewed as "dynamic obstacles", meaning they'll only be temporarily marked when approached. Plan will reajust when the robot approaches, but not permenately marked by global costmap 

## Real Robot Nav2

Runs Nav2 on the actual G1 with the head Livox MID360, FAST-LIO2 for odometry and slam_toolbox for SLAM. Bringup package is `g1_nav2_real_bringup`.

Note that livox_ros_driver2 and FAST_LIO are checked into this repo directly (not submodules) and both are modified from upstream, so dont clone fresh copies over them or you'll lose the deskew and frame changes.

### Turning on the G1
- Tap and hold on both the controller and the robot battery, wait for it to initialize
- Set to DAMP mode (L2 + B)
- Move to READY STATE (L2 + Up on the D-pad)
- Lower the gantry until the feet touch the ground
- Move to RUN mode (R2 + A)

### Environment Setup
The real stack uses a helper script that lives in your home directory, not in the repo. There's a template committed as `robot_setup.sh.example`, copy it over and fix the network interface for your machine:
```
cp robot_setup.sh.example ~/robot_setup.sh
# edit ~/robot_setup.sh and set NetworkInterface name="enp5s0" to your wired interface (check with `ip a`)
```
It sources ros2, the unitree cyclonedds workspace and this workspace, unsets ROS_DOMAIN_ID (so domain 0) and sets the rmw to cyclonedds. You have to source it in every terminal you use for the robot, not just once.

### Real Robot Build
```
source ~/robot_setup.sh
colcon build
source ~/robot_setup.sh
```

### Real Robot Run
```
source ~/robot_setup.sh
ros2 launch g1_nav2_real_bringup real_bringup_launch.py
```
Keep the robot standing still with the area clear for the first ~10 seconds, the floor anchor is collecting floor scans at startup and needs a clear flat floor in view. Same as the sim, set the fixed frame to map and wait ~15 seconds for slam to build the initial map before sending a goal.

Everything runs over a wired connection right now, wireless was never finished (see TODO).

### Known Issues
- The floor anchor doesnt always lock and then the frame starts tilted. It wants at least 400 points on the floor but sometimes only gets around 330 if the floor is sparse/dark or the robot is facing open space (the lidar is angled down so it needs floor in front of it within about 4m). Repositioning onto flat open floor usually fixes it, or bump scans_to_collect in the node. When it fails it falls back to a level guess so you get some tilt. Depends a lot on where you start it from.
- Floor still tilts a little after walking around, fast-lio pitch/z drift, the 2d slam only corrects x/y/yaw.
- /odom twist is all zeros (fast-lio doesnt fill it in) so DWB assumes the robot is always at zero velocity.
- the robots own legs show up in the cloud around 1.2m when its walking and we cant fully blind them out.

## TODO

### Nav2 Stack: 
- SLAM map saving and loading  

### Real Robot: 
- Make `floor_level_anchor` robust to sparse floors (starvation -> tilt)
- Periodic/continuous floor re-leveling to correct post-walk drift
- Fill `/odom` twist (differentiate pose) for DWB
- Save/load a static global map to reuse across sessions instead of SLAM-from-scratch
- Wireless control - was started but never got working, plan is a Zenoh bridge and probably a stronger GL.iNet
