cmake_minimum_required(VERSION 3.5)
project(g1_lower)

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

include_directories(include include/common)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
  sensor_msgs
  geometry_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(g1_arms_test src/g1_arms_test.cpp)
ament_target_dependencies(g1_arms_test ${DEPENDENCY_LIST})

add_executable(g1_arms_txt src/g1_arms_txt.cpp)
ament_target_dependencies(g1_arms_txt ${DEPENDENCY_LIST})

add_executable(g1_gesture_control src/g1_gesture_control.cpp)
ament_target_dependencies(g1_gesture_control ${DEPENDENCY_LIST})

add_executable(g1_ankle_swing_example src/g1_ankle_swing_example.cpp src/common/motor_crc_hg.cpp)
ament_target_dependencies(g1_ankle_swing_example ${DEPENDENCY_LIST})

add_executable(g1_lowcmd_test src/g1_lowcmd_test.cpp src/common/motor_crc_hg.cpp)
ament_target_dependencies(g1_lowcmd_test ${DEPENDENCY_LIST})

add_executable(g1_lowcmd_listener src/g1_lowcmd_listener.cpp src/common/motor_crc_hg.cpp)
ament_target_dependencies(g1_lowcmd_listener ${DEPENDENCY_LIST})

add_executable(txt_test src/txt_test.cpp src/common/motor_crc_hg.cpp)
ament_target_dependencies(txt_test ${DEPENDENCY_LIST})

add_executable(bag_test src/bag_test.cpp src/common/motor_crc_hg.cpp)
ament_target_dependencies(bag_test ${DEPENDENCY_LIST})

add_executable(bag_to_txt src/bag_to_txt.cpp)
ament_target_dependencies(bag_to_txt ${DEPENDENCY_LIST})

add_executable(hands_test src/hands_test.cpp)
ament_target_dependencies(hands_test ${DEPENDENCY_LIST})

add_executable(lowstate_jointstate_bridge src/lowstate_jointstate_bridge.cpp)
ament_target_dependencies(lowstate_jointstate_bridge ${DEPENDENCY_LIST})

add_executable(jointstate_lowcmd_bridge src/jointstate_lowcmd_bridge.cpp)
ament_target_dependencies(jointstate_lowcmd_bridge ${DEPENDENCY_LIST})

add_executable(loco_client_test src/loco_client_test.cpp)
ament_target_dependencies(loco_client_test ${DEPENDENCY_LIST})

add_executable(wireless_controller_test src/wireless_controller_test.cpp)
ament_target_dependencies(wireless_controller_test ${DEPENDENCY_LIST})


install(TARGETS g1_ankle_swing_example)
install(TARGETS g1_gesture_control)
install(TARGETS g1_lowcmd_test)
install(TARGETS g1_lowcmd_listener)
install(TARGETS lowstate_jointstate_bridge)
install(TARGETS jointstate_lowcmd_bridge)
install(TARGETS loco_client_test)
install(TARGETS wireless_controller_test)
install(TARGETS txt_test)
install(TARGETS bag_test)
install(TARGETS hands_test)
install(TARGETS bag_to_txt)
install(TARGETS g1_arms_test)
install(TARGETS g1_arms_txt)

install(
DIRECTORY launch meshes urdf
DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
	loco_client_test
	DESTINATION lib/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "src/msg/LowCmdListener.msg"
 )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
