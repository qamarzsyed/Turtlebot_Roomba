### ROS2 Roomba Project

## Overview
ROS 2 package that uses a turtlebot as a roomba. This package should identify any objects in front using the laser scan package and has two nodes that do actions based off this:
move_node moves the roomba forward if nothing is in front
rotate_node rotates the roomba if something is in front

## Instructions to run
colcon build --packages-select roomba
source /opt/ros/humble/setup.bash
source . install/setup.bash

# Dependencies
ROS2 Humble w/ Gazebo and Turtlebot3

ros2 launch world.launch
ros2 launch turtlebot.launch
ros2 launch nodes.launch