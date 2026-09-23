cmake_minimum_required(VERSION 3.8)
project(g1_gestures)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include include/common)
link_directories(src)

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
find_package(Eigen3 REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(ruckig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)

add_executable(g1_gesture_control src/g1_gesture_control.cpp)
ament_target_dependencies(g1_gesture_control ${DEPENDENCY_LIST})
target_link_libraries(g1_gesture_control ruckig::ruckig)

add_executable(hands_controller src/hands_controller.cpp)
ament_target_dependencies(hands_controller ${DEPENDENCY_LIST})

add_executable(wireless_controller_test src/wireless_controller_test.cpp)
ament_target_dependencies(wireless_controller_test ${DEPENDENCY_LIST})

install(TARGETS g1_gesture_control)
install(TARGETS hands_controller)
install(TARGETS wireless_controller_test)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
