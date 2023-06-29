#include <cstdlib>
#include <ctime>

#include <ros/ros.h>

// #include"posControl.h"
#include "ai_brain.h"

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "brain");
  ros::NodeHandle nh;
  ros::Publisher brainok_pub = nh.advertise<std_msgs::Bool>("/brain/ok", 1);
  ros::Rate r(50);

  AiBrain aib(nh);
  std_msgs::Bool brain_flag;
  brain_flag.data = true;

  ROS_INFO("[AIBRAIN] Brain wake up!");
  while (ros::ok()) {
    aib.think();
    brainok_pub.publish(brain_flag);
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}
