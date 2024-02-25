cmake_minimum_required(VERSION 3.5)
project(esvo_core)

# Set C++ standard explicitly to C++14 to remove errors from PCL library
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Find packages
find_package(ament_cmake REQUIRED)
find_package(minkindr_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dvs_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${minkindr_conversions_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


# Define library targets
add_library(esvo_core_lib
  src/core/DepthFusion.cpp
  src/core/DepthRegularization.cpp
  # Add all other source files
)

ament_target_dependencies(esvo_core_lib
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  tf2_eigen
  tf2_geometry_msgs
  OpenCV
  PCL
  tf2_ros
  dvs_msgs
  pcl_ros
  minkindr_conversions
)

# Define executable targets
add_executable(esvo_Mapping src/esvo_MappingNode.cpp src/esvo_Mapping.cpp)
ament_target_dependencies(esvo_Mapping rclcpp sensor_msgs geometry_msgs OpenCV PCL)
target_link_libraries(esvo_Mapping esvo_core_lib ${OpenCV_LIBRARIES} yaml-cpp)

# Repeat for other executables

# Install executables and libraries
install(TARGETS
  esvo_Mapping
  esvo_core_lib
  # Add other targets
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and other directories if needed
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
