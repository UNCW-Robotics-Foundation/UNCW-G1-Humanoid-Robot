cmake_minimum_required(VERSION 3.5)
project(g1_tele)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set (
  DEPENDENCY_LIST
  g1_msgs
  geometry_msgs
  rclcpp
  rosbag2_cpp
  ruckig
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  unitree_api
  unitree_go
  unitree_hg
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ruckig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)

add_executable(g1_tele_controller src/g1_tele_controller.cpp)
ament_target_dependencies(g1_tele_controller ${DEPENDENCY_LIST})

install(TARGETS g1_tele_controller)

install(
DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

install(TARGETS

	DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
	scripts/g1_tele_middle.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
