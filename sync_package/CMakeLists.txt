cmake_minimum_required(VERSION 3.0.2)
project(sync_package)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sync_package
  CATKIN_DEPENDS roscpp sensor_msgs message_filters
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sync_node src/sync_node.cpp)
target_link_libraries(sync_node ${catkin_LIBRARIES})

install(TARGETS sync_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
