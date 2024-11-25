cmake_minimum_required(VERSION 3.5)
project(rcomponent)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robotnik_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Add include directories
include_directories(include)

# Create the library
add_library(rcomponent SHARED
  src/rcomponent.cpp
)

# Link the library with the dependencies
ament_target_dependencies(rcomponent
  rclcpp
  std_msgs
  std_srvs
  robotnik_msgs
  diagnostic_msgs
  diagnostic_updater
)

ament_export_targets(rcomponent HAS_LIBRARY_TARGET)

# Install the library
install(
  TARGETS rcomponent
  EXPORT rcomponent
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install include directory
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Install the launch files, config files, and other resources if needed
#ament_export_include_directories("include/${PROJECT_NAME}")

#ament_export_libraries(rcomponent)
#ament_export_include_directories(include)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  robotnik_msgs
  diagnostic_msgs
  diagnostic_updater
)

# Install the package.xml file
ament_package()
