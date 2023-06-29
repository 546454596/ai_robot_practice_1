#include "brain/pioneer_3at.h"

using namespace std;

Pioneer3at::Pioneer3at(const ros::NodeHandle& nh)
    : nh_(nh) {
  initPublisherSubscriber();
}

Pioneer3at::~Pioneer3at() {}

void Pioneer3at::move(float forward_x, float rotate_z) {
  geometry_msgs::Twist vel;

  vel.linear.x = forward_x;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = rotate_z;

  vel_pub_.publish(vel);
}

void Pioneer3at::takeOff() {
  std_srvs::Empty srv;
  enable_srv_.call(srv);
  move(0, 0);
}

void Pioneer3at::land() {
  move(0, 0);
  std_srvs::Empty srv;
  disable_srv_.call(srv);
}

void Pioneer3at::initPublisherSubscriber() {
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  enable_srv_ = nh_.serviceClient<std_srvs::Empty>("RosAria/enable_motors");
  disable_srv_ = nh_.serviceClient<std_srvs::Empty>("RosAria/disable_motors");
}
