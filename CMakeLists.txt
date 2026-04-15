cmake_minimum_required(VERSION 3.8)
project(rcomponent)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robotnik_common_msgs REQUIRED)

if(BUILD_TESTING)

endif()

# Create library
add_library(rcomponent
  src/rcomponent.cpp
	src/state/state_manager.cpp
	src/state/state_interfaces.cpp
	src/state/lifecycle/lifecycle_manager.cpp
	src/state/lifecycle/lifecycle_transitions.cpp
	src/state/communication/communication_monitor.cpp
)

target_include_directories(rcomponent
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rcomponent
  rclcpp
  rclcpp_lifecycle
	std_msgs
	std_srvs
	robotnik_common_msgs
)

# Install library
install(
  TARGETS rcomponent
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export library dependencies
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  std_msgs
	std_srvs
	robotnik_common_msgs
)

ament_export_include_directories(include) 
ament_export_libraries(rcomponent)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(rcomponent)

ament_package()