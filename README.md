# UNCW-G1-Humanoid-Robot
Base scripts and foundational programs for uncw's G1 Humanoid Robot. Utilizes ROS2 as framework.

## Operating System
Linux Ubuntu 22.04

## Initial Setup
- Install Ros2 Humble
- Additional Ros2 dependencies:
```
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
```
- Install the [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) repository. You only need to install the cyclonedd_ws package. The examples are not necessary. Simplified instructions:
```
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl libyaml-cpp-dev

git clone https://github.com/unitreerobotics/unitree_ros2
cd unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build 
```  
- If the package failed to build, refer to repository for potential troubleshooting
- In order to see the Ros topics from the robot, follow the network configuration steps from the unitree_ros2 repo. Make sure to also change the ros source line from foxy to humble in the setup.sh files. If all of the steps were followed, and the topics are not showing up, a system restart should fix it.

### Sorce repos for Nav2 Simulator: 
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) — follow install instructions on repo page
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [FAST-LIO (ROS2 branch)](https://github.com/hku-mars/FAST_LIO) → branch ROS2

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

Make sure to source the unitree package before building since this package uses their message files.
The generated build, install, and log folders are part of the gitignore. I believe you will need to source the package everytime when using a new terminal.
The rviz simulator will not work if the unitree package is sourced, and the robot is not connected or turned off, but it will work if the robot is on.
The build command may display errors at the end, however, these should just be warnings. If you run colcon build again, and there isn't actually an error, it will show everything complete with no warning.

## IK Solver setup
- Install the miniforge version of conda - [Link](https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html#)
- Install the [Robostack](https://robostack.github.io/conda.html) packages and [xr-teleoperate](https://github.com/unitreerobotics/xr_teleoperate) dependencies with these commands:
```
conda config --env --remove channels defaults
conda create -n ros_env -c conda-forge -c robostack-humble ros-humble-desktop  python pinocchio numpy=1.26.4
conda activate ros_env
conda config --env --add channels robostack-humble
conda install -c conda-forge ros-dev-tools
pip install logging-mp
```
- You don't need to source ros in this environment. Test the environment by running - ros2 run rviz2 rviz2

- Next create a new workspace for the ik solver. There will be two instances of the UNCW repository, one outside the conda environment, and one inside. Make sure not to confuse the two. Make sure the conda environment is activated before installing and compiling the second copy of the repo and activated when developing with the ik package. Only the msgs and ik packages should be built in this environment, the other ones would require other dependencies and/or projects, but we are trying to keep the environment as minimal as possible.
```
mkdir ik_ws && cd ik_ws && git clone https://github.com/UNCW-Robotics-Foundation/UNCW-G1-Humanoid-Robot
cd UNCW-G1-Humanoid-Robot
colcon build --packages-select g1_msgs g1_ik
source install/setup.bash
```
- Test the ik solver - ros2 run g1_ik robot_arm_ik
- If the ik solver is working, it should output for a user input with the text:
```
Please enter the start signal (enter 's' to start the subsequent program):
```
- You should be able to run and start developing with the g1_ik_controller at this point. If the ik solver did not display this message, then it may be an issue with a missing dependency.
*** Conda may cause building errors outside of ros_env since it will probably auto activate base conda. To fix this, lookup the command to stop auto activating. If you accidentally build in the wrong environment, delete the build, install, and log folders, then try building again.

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

## Nav2 Real Implementation 

See Real Nav2 Stack branch for full nav2 usage and instructions to run on the G1

## TODO

### Nav2 Stack: 
- SLAM map saving and loading in sim 
- Real Robot implementation 
