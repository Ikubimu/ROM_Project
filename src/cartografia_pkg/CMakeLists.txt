cmake_minimum_required(VERSION 3.8)
project(cartografia_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(amcl_odom
src/amcl_odom.cpp)

ament_target_dependencies(
  amcl_odom
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

install(TARGETS
amcl_odom
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

ament_package()
