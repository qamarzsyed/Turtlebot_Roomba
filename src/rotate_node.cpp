#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class TurtlebotRotator
{
public:
  TurtlebotRotator()
  {
    //Topic you want to subscribe
    sub_ = n_.subscribe("/scan", 1, &TurtlebotRotator::scanCallback, this);

    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  }

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    //Find the closest range between the defined minimum and maximum angles
    int min_index = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int max_index = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

    float closest_range = scan->ranges[min_index];
    for (int curr_index = min_index + 1; curr_index <= max_index; curr_index++) {
      if (scan->ranges[curr_index] < closest_range) {
        closest_range = scan->ranges[curr_index];
      }
    }

    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;

    //If the closest range is less than the threshold, rotate
    if (closest_range < MIN_DIST_FROM_OBSTACLE) {
      twist.angular.z = MAX_ANGULAR_SPEED;
    }

    pub_.publish(twist);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

  //Define the minimum and maximum angles of the scan
  static const double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
  static const double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;

  //Define the minimum distance for the turtlebot to rotate
  static const float MIN_DIST_FROM_OBSTACLE = 1.0;

  //Define the maximum angular speed
  static const float MAX_ANGULAR_SPEED = 1.0;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "turtlebot_rotator");

  //Create an object of class TurtlebotRotator that will take care of everything
  TurtlebotRotator turtlebot_rotator;

  ros::spin();

  return 0;
}