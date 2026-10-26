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

include_directories(include include/common)
link_directories(src)
include_directories(${OpenCV_INCLUDE_DIRS})

set (
  DEPENDENCY_LIST
  cv_bridge
  g1_msgs
  geometry_msgs
  OpenCV
  PCL
  pcl_conversions
  pcl_ros
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
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
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
find_package(PCL REQUIRED COMPONENTS common filters)

add_executable(crop_visualize_3d src/crop_visualize_3d.cpp)
ament_target_dependencies(crop_visualize_3d ${DEPENDENCY_LIST})
target_link_libraries(crop_visualize_3d ${PCL_LIBRARIES})

add_executable(g1_highcmd_ik_arms src/g1_highcmd_ik_arms.cpp)
ament_target_dependencies(g1_highcmd_ik_arms ${DEPENDENCY_LIST})
target_link_libraries(g1_highcmd_ik_arms ruckig::ruckig)

add_executable(g1_lowcmd_ik_arms_fast src/g1_lowcmd_ik_arms_fast.cpp src/common/motor_crc_hg.cpp)
ament_target_dependencies(g1_lowcmd_ik_arms_fast ${DEPENDENCY_LIST})
target_link_libraries(g1_lowcmd_ik_arms_fast ruckig::ruckig)

add_executable(g1_lowcmd_ik_arms_fast_r src/g1_lowcmd_ik_arms_fast_r.cpp src/common/motor_crc_hg.cpp)
ament_target_dependencies(g1_lowcmd_ik_arms_fast_r ${DEPENDENCY_LIST})
target_link_libraries(g1_lowcmd_ik_arms_fast_r ruckig::ruckig)

add_executable(g1_lowcmd_ik_arms_fast_l src/g1_lowcmd_ik_arms_fast_l.cpp src/common/motor_crc_hg.cpp)
ament_target_dependencies(g1_lowcmd_ik_arms_fast_l ${DEPENDENCY_LIST})
target_link_libraries(g1_lowcmd_ik_arms_fast_l ruckig::ruckig)

add_executable(g1_demo_grasp src/g1_demo_grasp.cpp)
ament_target_dependencies(g1_demo_grasp ${DEPENDENCY_LIST})

install(TARGETS crop_visualize_3d)
install(TARGETS g1_highcmd_ik_arms)
install(TARGETS g1_lowcmd_ik_arms_fast)
install(TARGETS g1_lowcmd_ik_arms_fast_r)
install(TARGETS g1_lowcmd_ik_arms_fast_l)
install(TARGETS g1_demo_grasp)

install(
DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

install(TARGETS

	DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
	scripts/g1_news_demo.py
	scripts/g1_tele_middle.py
	scripts/g1_tele_pen.py
	scripts/g1_tele_plan.py
	scripts/ball_center_detect.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
