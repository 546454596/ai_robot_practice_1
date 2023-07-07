#include "rctl_server.h"

#include <utility>

#include <boost/filesystem.hpp>

#include "ai_robot_rctl_server/RctlMapMetaData.h"

namespace fs = boost::filesystem;

using ai_robot_rctl_server::RctlMapMetaData;

RemoteControlServer::RemoteControlServer(const ros::NodeHandle& nh)
    : nh_(nh) {
  initPublishersAndSubscribers();
}

RemoteControlServer::~RemoteControlServer() {}

void RemoteControlServer::getMapFilename() {
  std::string pathname;
  nh_.getParam("/map_path", pathname);

  if (pathname.back() == '/') {
    pathname = pathname.substr(0, pathname.find_last_not_of("/") + 1);
  }

  fs::path map_path(pathname);
  map_name_ = map_path.filename().stem().string();
}

void RemoteControlServer::initPublishersAndSubscribers() {
  map_sub_ = nh_.subscribe("/map", 1, &RemoteControlServer::publishMapMetaData, this);
  map_pub_ = nh_.advertise<RctlMapMetaData>("/remote/map_metadata", 1, true);
}

void RemoteControlServer::publishMapMetaData(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("map metadata resultion %f.", msg->info.resolution);
  if (map_name_.empty())
    getMapFilename();
  RctlMapMetaData mapdata;
  mapdata.name.data = map_name_;
  mapdata.data = msg->info;
  map_pub_.publish(mapdata);
}
