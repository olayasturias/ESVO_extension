cmake_minimum_required(VERSION 3.5)
project(esvo_time_surface)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Find ROS2 and other package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dvs_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED) # Eigen3 is found through the eigen3_cmake_module in ROS2
find_package(OpenCV REQUIRED)
# Add other dependencies as necessary, e.g., dvs_msgs if it has been ported to ROS2

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Declare ROS2 executables
add_executable(esvo_time_surface
  src/TimeSurface.cpp
  src/TimeSurface_node.cpp
)
ament_target_dependencies(esvo_time_surface
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  Eigen3
  OpenCV
  dvs_msgs
)

# Another executable
add_executable(TimeSurface_global_timer
  src/TimeSurface_global_timer.cpp
)
ament_target_dependencies(TimeSurface_global_timer
  rclcpp
  std_msgs
  builtin_interfaces
  dvs_msgs
)

# Install the executables and/or libraries
install(TARGETS
  esvo_time_surface
  TimeSurface_global_timer
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and other directories if applicable
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Export dependencies
ament_export_dependencies(rclcpp std_msgs dvs_msgs sensor_msgs cv_bridge image_transport Eigen3 OpenCV)
ament_package()
