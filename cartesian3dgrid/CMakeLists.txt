cmake_minimum_required(VERSION 3.5)
project(esvo_cartesian3dgrid)

# Use C++14 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED) # Uncomment if Eigen3 is used

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIRS} # Uncomment if Eigen3 is used
)

# Declare a C++ library
file(GLOB SOURCE_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

ament_target_dependencies(${PROJECT_NAME}
  # rclcpp # Uncomment if you use ROS2 functionalities
  OpenCV
)

# Uncomment if the library links to libraries of other packages
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  # ${EIGEN3_LIBRARIES} # Uncomment if Eigen3 is used
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install launch files and other directories if you have them
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Export dependencies
ament_export_dependencies(ament_cmake OpenCV) # Add other dependencies separated by spaces
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
