cmake_minimum_required(VERSION 3.8)
project(g1_nav2_real_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  ../g1_lower/include
  ../g1_lower/include/common
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)

add_executable(nav2_cmd_controller src/nav2_cmd_controller.cpp)
ament_target_dependencies(nav2_cmd_controller rclcpp geometry_msgs unitree_api)

install(TARGETS nav2_cmd_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
