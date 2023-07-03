#include "rctl_server.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rctl_server");
  ros::NodeHandle nh;
  RemoteControlServer server(nh);
  ros::Rate r(1);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
