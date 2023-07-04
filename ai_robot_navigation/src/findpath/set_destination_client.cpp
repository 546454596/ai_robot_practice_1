#include <cstdlib>

#include <ros/ros.h>

#include "ai_robot_interfaces/set_destination.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "set_destination_client");

  if (argc != 4) {
    ROS_INFO("usage: set_destination_client X Y Z");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ai_robot_interfaces::set_destination>("/ai_robot_navigation/setdestination");
  ai_robot_interfaces::set_destination srv;
  srv.request.x = atof(argv[1]);
  srv.request.y = atof(argv[2]);
  srv.request.z = atof(argv[3]);

  if (client.call(srv)) {
    ROS_INFO("set ok!");
  } else {
    ROS_ERROR("Failed to call service set_destination");
    return 1;
  }
  
  return 0;
}