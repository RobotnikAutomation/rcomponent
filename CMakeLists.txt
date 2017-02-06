cmake_minimum_required(VERSION 2.8.3)
project(rcomponent)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  robotnik_msgs
  diagnostic_msgs
  diagnostic_updater
)

# catkin_python_setup()

catkin_package(

)

include_directories(
   include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(rcomponent
#   src/rcomponent.cpp
#)

## Declare a cpp executable
add_executable(rcomponent_server src/simple_service_component.cpp src/rcomponent.cpp)
add_executable(rcomponent_simple src/simple_component.cpp src/rcomponent.cpp)
add_executable(rcomponent_diagnostic src/simple_diagnostics_component.cpp src/rcomponent.cpp)
add_executable(rcomponent_template src/rcomponent_template.cpp)
add_executable(rscomponent_template src/rscomponent_template.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(rcomponent_node robotnik_msgs_generate_messages_cpp)
add_dependencies(rcomponent_server robotnik_msgs_generate_messages_cpp)
add_dependencies(rcomponent_simple robotnik_msgs_generate_messages_cpp)
add_dependencies(rcomponent_diagnostic robotnik_msgs_generate_messages_cpp)
add_dependencies(rcomponent_template robotnik_msgs_generate_messages_cpp)


target_link_libraries(rcomponent_server
   ${catkin_LIBRARIES}
)
target_link_libraries(rcomponent_simple
   ${catkin_LIBRARIES}
)
target_link_libraries(rcomponent_diagnostic
   ${catkin_LIBRARIES}
)
target_link_libraries(rcomponent_template
   ${catkin_LIBRARIES}
)

target_link_libraries(rscomponent_template
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rcomponent rcomponent_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


