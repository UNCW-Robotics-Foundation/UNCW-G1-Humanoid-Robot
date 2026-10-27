FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
        vim \
        ccls \
        wget \
        unzip \
        ament-cmake \
        python3-pip \
        python3-colcon-common-extensions \
        libusb-1.0-0 \
        libusb-1.0-0-dev \
        libyaml-cpp-dev \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-ruckig && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home

RUN git clone https://github.com/Art3mi0/unitree_ros2.git
#RUN git clone -b enp2s0 https://github.com/Art3mi0/unitree_ros2.git
RUN cd unitree_ros2/cyclonedds_ws && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build

RUN git clone https://github.com/UNCW-Robotics-Foundation/UNCW-G1-Humanoid-Robot
RUN cd UNCW-G1-Humanoid-Robot && bash download-lib.sh && cd src/ros2_stark_controller && mkdir lib && cd - && cp dist/shared/linux/libbc_stark_sdk.so src/ros2_stark_controller/lib/
RUN source unitree_ros2/setup.sh && cd UNCW-G1-Humanoid-Robot && colcon build --packages-select g1_msgs g1_gestures ros2_stark_controller ros2_stark_interfaces

COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]