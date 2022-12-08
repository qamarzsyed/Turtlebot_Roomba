/**
 * @file move_node.cpp
 * @author Qamar Syed
 * @brief cpp file for node to move turtlebot roomba
 * @version 0.1
 * @date 2022-12-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>


class TurtlebotMover
{
public:
/**
 * @brief Construct a new Turtlebot Mover object
 * 
 */
  TurtlebotMover()
  {
    //Topic to publish
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

    //Topic to subscribe
    sub_ = n_.subscribe("/scan", 1, &TurtlebotMover::scanCallback, this);
  }

/**
 * @brief gets info from laser scanner to see if object is in front
 * 
 * @param scan scanning sensor object
 */
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    //ROS_INFO_STREAM("Scan callback");
    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

    float closestRange = scan->ranges[minIndex];
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
      if (scan->ranges[currIndex] < closestRange) {
        closestRange = scan->ranges[currIndex];
      }
    }

    geometry_msgs::Twist twist;
    if (closestRange < MIN_SCAN_RANGE) {
      twist.linear.x = 0.0;
    } else {
      twist.linear.x = MAX_SPEED;
    }
    pub_.publish(twist);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

  // Parameters
  const double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
  const double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
  const float MIN_SCAN_RANGE = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max
  const float MAX_SPEED = 0.2;
};

/**
 * @brief creates node that moves forward if no object
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "turtlebot_mover");

  //Create an object of class TurtlebotMover
  TurtlebotMover turtlebotMover;

  ros::spin();

  return 0;
}