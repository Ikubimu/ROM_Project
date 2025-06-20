cmake_minimum_required(VERSION 3.8)
project(turtlebot3_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encuentra las dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  ${nav2_map_server_INCLUDE_DIRS}
)

# Agrega tu ejecutable
add_executable(${PROJECT_NAME}_node src/turtlebot3_main.cpp)

# Enlaza tu ejecutable con las librerías necesarias
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  nav2_map_server
  sensor_msgs
)


# Instala el ejecutable para que esté disponible como ejecutable ROS 2
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Linter (opcional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
