#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class TurtlebotMover
{
public:
  TurtlebotMover()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/scan", 1, &TurtlebotMover::scanCallback, this);
  }

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

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "turtlebot_mover");

  //Create an object of class TurtlebotMover that will take care of everything
  TurtlebotMover turtlebotMover;

  ros::spin();

  return 0;
}