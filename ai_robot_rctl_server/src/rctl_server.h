#ifndef _AI_ROBOT_RCTL_SERVER_H_
#define _AI_ROBOT_RCTL_SERVER_H_

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

class RemoteControlServer {
public:
  RemoteControlServer(const ros::NodeHandle& nh);
  virtual ~RemoteControlServer();

private:
  void getMapName();
  void publishMapMetaData(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void initPublishersAndSubscribers();

private:
  ros::NodeHandle nh_;
  std::string map_name_;
  ros::Subscriber map_sub_;
  ros::Publisher map_pub_;
};

#endif
