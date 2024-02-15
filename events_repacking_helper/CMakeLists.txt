cmake_minimum_required(VERSION 3.5)
project(esvo_code)  # Update the project name to esvo_code

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Add any additional packages specific to esvo_code here

# Include directories
include_directories(
  include
)

# Declare a C++ executable
# Update or add executables as per esvo_code requirements
# For example, if esvo_code has different executables, declare them here

# Install the executable
install(TARGETS
  # Update the targets according to the executables declared above
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, and any other directories esvo_code might need
install(DIRECTORY
  launch
  # Add any additional directories to install
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
