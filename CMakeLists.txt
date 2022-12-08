cmake_minimum_required(VERSION 3.5)
project(roomba)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(include)

# roomba includes
include_directories(include ${roomba_INCLUDE_DIRS})

ament_export_dependencies(rclcpp)
ament_export_dependencies(roomba)

# add executables
add_executable(move_node src/move_node.cpp)
add_executable(rotate_node src/rotate_node.cpp)
target_link_libraries(move_node ${roomba_LIBRARIES})
target_link_libraries(rotate_node ${roomba_LIBRARIES})

# install executables
install(TARGETS
  move_node
  rotate_node
  DESTINATION lib/${PROJECT_NAME}
)

# export the package for use from other packages
ament_export_include_directories(include)
ament_export_libraries(${roomba_LIBRARIES})
ament_package()