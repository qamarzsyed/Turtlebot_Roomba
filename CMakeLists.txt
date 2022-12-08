cmake_minimum_required(VERSION 2.8.3)
project(roomba)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  turtlebot_gazebo
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roomba
  CATKIN_DEPENDS roscpp rospy std_msgs gazebo_ros turtlebot_gazebo sensor_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_node src/move_node.cpp)
add_executable(rotate_node src/rotate_node.cpp)

target_link_libraries(move_node
  ${catkin_LIBRARIES}
)

target_link_libraries(rotate_node
  ${catkin_LIBRARIES}
)

install(TARGETS move_node rotate_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)