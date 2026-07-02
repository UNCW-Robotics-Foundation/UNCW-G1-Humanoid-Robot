FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
        vim \
        ccls \
        ament-cmake \
        python3-pip \
        python3-colcon-common-extensions \
        libyaml-cpp-dev \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home

#RUN git clone https://github.com/Art3mi0/unitree_ros2.git
RUN git clone -b enp2s0 https://github.com/Art3mi0/unitree_ros2.git
RUN cd unitree_ros2 && git pull
RUN cd unitree_ros2/cyclonedds_ws && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build

RUN git clone https://github.com/UNCW-Robotics-Foundation/UNCW-G1-Humanoid-Robot
RUN cd UNCW-G1-Humanoid-Robot && git pull
RUN source unitree_ros2/setup.sh && cd UNCW-G1-Humanoid-Robot && colcon build --packages-select g1_lower

COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]