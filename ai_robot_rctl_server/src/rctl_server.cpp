#include "rctl_server.h"

#include <utility>

#include <boost/filesystem.hpp>

#include "ai_robot_rctl_server/RctlMapMetaData.h"

namespace fs = boost::filesystem;

using ai_robot_rctl_server::RctlMapMetaData;

RemoteControlServer::RemoteControlServer(const ros::NodeHandle& nh)
    : nh_(nh) {
  getMapFilename();
  initPublishersAndSubscribers();
}

RemoteControlServer::~RemoteControlServer() {}

void RemoteControlServer::getMapFilename() {
  std::string map_path;
  nh_.getParam("/map_path", map_path);

  if (map_path.back() == '/') {
    map_path = map_path.substr(0, map_path.find("/"));
  }

  fs::path pathname(map_path);
  map_filename_ = pathname.filename().string();
}

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
