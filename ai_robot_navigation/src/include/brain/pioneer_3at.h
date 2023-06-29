#ifndef AI_ROBOT_NAVIGATION_PIONEER3AT_H_
#define AI_ROBOT_NAVIGATION_PIONEER3AT_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

class Pioneer3at {
public:
  Pioneer3at(const ros::NodeHandle& nh);
  ~Pioneer3at();

  // input should be m/s and rad/s
  void land();
  void move(float forward_x, float rotate_z);
  void takeOff();

private:
  void initPublisherSubscriber();

private:
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;
  ros::ServiceClient enable_srv_;
  ros::ServiceClient disable_srv_;
};

#endif  // AI_ROBOT_NAVIGATION_PIONEER3AT_H_
