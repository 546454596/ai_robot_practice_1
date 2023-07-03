#include "rctl_server.h"

#include <utility>

#include "ai_robot_rctl_server/RctlMapMetaData.h"

using ai_robot_rctl_server::RctlMapMetaData;

RemoteControlServer::RemoteControlServer(const ros::NodeHandle& nh)
    : nh_(nh) {
  nh_.getParam("/map_filename", map_filename_);
  initPublishersAndSubscribers();
}

RemoteControlServer::~RemoteControlServer() {}

void RemoteControlServer::initPublishersAndSubscribers() {
  map_sub_ = nh_.subscribe("/map", 1, &RemoteControlServer::publishMapMetaData, this);
  map_pub_ = nh_.advertise<RctlMapMetaData>("/remote/map_metadata", 1, true);
}

void RemoteControlServer::publishMapMetaData(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("map metadata resultion %f.", msg->info.resolution);
  RctlMapMetaData mapdata;
  mapdata.filename.data = map_filename_.c_str();
  mapdata.data = msg->info;
  map_pub_.publish(mapdata);
}
