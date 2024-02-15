cmake_minimum_required(VERSION 3.5)
project(cnpy_colcon)

# Set C++ Standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
# Other find_package calls as necessary

# Include directories
include_directories(/usr/local/include)

# Define your library or executable target
add_library(dependency_tracker SHARED src/dependency_tracker.cc)
# Or add_library(your_library SHARED src/your_lib.cpp)

# Linking against cnpy
target_link_libraries(dependency_tracker /usr/local/lib/libcnpy.so)
# Or target_link_libraries(your_library /usr/local/lib/libcnpy.so)

# Install and Export (as needed for your project)
install(TARGETS dependency_tracker
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(/usr/local/lib/libcnpy.so)
ament_package()
