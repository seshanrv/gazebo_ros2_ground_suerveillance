cmake_minimum_required(VERSION 3.5)
project(surveillance_bot_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++17 is required
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(ControllerNode src/ControllerNode.cpp)
ament_target_dependencies(ControllerNode rclcpp geometry_msgs sensor_msgs OpenCV cv_bridge)
target_include_directories(ControllerNode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ControllerNode
  DESTINATION lib/${PROJECT_NAME})

ament_package()
