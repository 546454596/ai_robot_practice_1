#include <cmath>
#include <fstream>

#include <boost/tokenizer.hpp>
#include <boost/token_functions.hpp>

#include "point_to_point/point_to_point.h"

Point2Point::Point2Point(const ros::NodeHandle& nh)
    : nh_(nh),
      cntdown_(0),
      tgt_id_(-1) {
  nh_.getParam("pointToPointFile", p2p_filename_);
  nh_.param("time_to_pause", time_to_pause_, 50);
  tgt_point_pub_ = nh_.advertise<geometry_msgs::Pose>("/ai_robot/findpath/target_point", 1);
  readWayPoints();
}

Point2Point::~Point2Point() {}

bool Point2Point::readWayPoints() {
  std::ifstream in(p2p_filename_);

  if (!in) {
    ROS_INFO("[P2P] No input waypoint file!\n");
    return false;
  }

  std::string line;
  // std::vector<float> pt;
  std::array<float, 3> pt;
  boost::char_separator<char> sep(" ");

  path_.clear();
  // pt.resize(3);

  // Read point cloud from "freiburg format"
  while (!in.eof()) {
    std::getline(in, line);
    in.peek();
    boost::tokenizer<boost::char_separator<char>> tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

    if (tokens.size() != 3)
      continue;

    pt[0] = boost::lexical_cast<float>(tokens[0]);
    pt[1] = boost::lexical_cast<float>(tokens[1]);
    pt[2] = boost::lexical_cast<float>(tokens[2]);
    path_.push_back(pt);
  }

  tgt_id_ = 0;

  for (int i = 0; i < path_.size(); ++i) {
    printf("waypoint %d (%f,%f,%f)\n", i, path_[i][0], path_[i][1], path_[i][2]);
  }

  std::cout << "read waypoints----->" << '\n' << "  total " << path_.size() << " waypoints." << '\n';
  in.close();

  return true;
}

bool Point2Point::getTargetSpeed(float now_x, float now_y, float now_z, float now_yaw,
                                 float& v_x, float& v_y, float& v_z, float& v_yaw) {
  if (cntdown_ > 0) {
    --cntdown_;
    v_x = 0;
    v_y = 0;
    v_z = 0;
    v_yaw = 0;
    return true;
  }

  if (tgt_id_ < path_.size()) {
    float dist = sqrt(pow(now_x - path_[tgt_id_][0], 2) + pow(now_y - path_[tgt_id_][1], 2));
    calcDPotential2(now_x, now_y, now_z, path_[tgt_id_][0], path_[tgt_id_][1],
                   path_[tgt_id_][2], v_x, v_y, v_z);
    v_yaw = calcYawSpeed(v_x, v_y, v_z, now_yaw);
  
    // pub target point
    geometry_msgs::Pose tgtpose_msg;
    float twc[3] = {now_x, now_y, now_z}, rwc[9] = {cos(now_yaw), -sin(now_yaw), 0, sin(now_yaw), cos(now_yaw), 0, 0, 1}, tgtpose_frombody[3];
    rwc[0] = cos(now_yaw);
    rwc[1] = -sin(now_yaw);
    rwc[3] = sin(now_yaw);
    rwc[4] = cos(now_yaw);
    transformBodyFromNwuWorld(tgtpose_frombody[0], tgtpose_frombody[1], tgtpose_frombody[2], path_[tgt_id_][0],
                              path_[tgt_id_][1], path_[tgt_id_][2], rwc, twc);
    tgtpose_msg.position.x = tgtpose_frombody[0];
    tgtpose_msg.position.y = tgtpose_frombody[1];
    tgtpose_msg.position.z = tgtpose_frombody[2];

    float del_yaw = 0;
    if (tgt_id_ < path_.size() - 1) {
      // Try to dead to next point when going to this point
      float nexttgtpose_frombody[3], q_frombody[4];
      transformBodyFromNwuWorld(nexttgtpose_frombody[0], nexttgtpose_frombody[1], nexttgtpose_frombody[2],
                                path_[tgt_id_ + 1][0], path_[tgt_id_ + 1][1],
                                path_[tgt_id_ + 1][2], rwc, twc);
      del_yaw = atan2(nexttgtpose_frombody[1] - tgtpose_frombody[1], nexttgtpose_frombody[0] - tgtpose_frombody[0]);
      eulerToQ(0, 0, del_yaw, q_frombody);
      tgtpose_msg.orientation.w = q_frombody[0];
      tgtpose_msg.orientation.x = q_frombody[1];
      tgtpose_msg.orientation.y = q_frombody[2];
      tgtpose_msg.orientation.z = q_frombody[3];
    } else {
      tgtpose_msg.orientation.w = 0;
      tgtpose_msg.orientation.x = 0;
      tgtpose_msg.orientation.y = 0;
      tgtpose_msg.orientation.z = 0;
    }

    tgt_point_pub_.publish(tgtpose_msg);
 
    if (dist < 0.5) {
      ++tgt_id_;
      cntdown_ = time_to_pause_;
      ROS_INFO("[P2P] To the '%d' point and pause %f second.", tgt_id_, time_to_pause_ * 1.0 / 50);
    }

    return true;
  } else {
    v_x = .0;
    v_y = .0;
    v_z = .0;
    v_yaw = .0;
    return false;
  }
}

void Point2Point::calcDPotential2(float now_x, float now_y, float now_z, float tgt_x, float tgt_y, float tgt_z,
                                 float& v_x, float& v_y, float& v_z) {
  float dx = tgt_x - now_x;
  float dy = tgt_y - now_y;
  float dl = sqrt(pow(dx, 2) + pow(dy, 2));
  float tgt_v;

  if (dl > 2)
    tgt_v = 0.5;
  else
    tgt_v = dl / 4;

  float vo = tgt_v / dl;
  v_x = vo * dx;
  v_y = vo * dy;

  if (dx > 0)
    v_x = fabs(v_x);
  else
    v_x = -fabs(v_x);

  if (dy > 0)
    v_y = fabs(v_y);
  else
    v_y = -fabs(v_y);

  float dz = tgt_z - now_z;

  if (fabs(dz) > 0.3)
    v_z = 0.3 * fabs(dz) / dz;
  else
    v_z = dz;
}

// Calculates the yaw err between the current direction and target point direction
float Point2Point::calcYawSpeed(float v_x, float v_y, float v_z, float yaw_now) {
  float l = sqrt(pow(v_x, 2) + pow(v_y, 2));
  float yaw_tgt;

  if (v_y > 0)
    yaw_tgt = acos(v_x / l);
  else
    yaw_tgt = -acos(v_x / l);

  float y_err = yaw_tgt - yaw_now;

  if (y_err > M_PI)
    y_err -= 2 * M_PI;
  else if (y_err < -M_PI)
    y_err += 2 * M_PI;

  return y_err;
}

void Point2Point::clear() {
  path_.clear();
  tgt_id_ = 0;
}

void Point2Point::addPoint(float x, float y, float z) {
  std::array<float, 3> pt;

  pt[0] = x;
  pt[1] = y;
  pt[2] = z;

  // std::cout << x << " " << y << " " << z << '\n';
  path_.push_back(pt);
}
