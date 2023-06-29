#ifndef AI_ROBOT_NAVIGATION_POIN2TPOINT_H_
#define AI_ROBOT_NAVIGATION_POIN2TPOINT_H_

#include <array>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include "utils/math_aux.h"

class Point2Point {
public:
  Point2Point(const ros::NodeHandle& nh);
  ~Point2Point();

  // points are all in world frame
  void addPoint(float x, float y, float z);
  void clear();
  bool getTargetSpeed(float now_x, float now_y, float now_z, float now_yaw,
                      float& v_x, float& v_y, float& v_z, float& v_yaw);
  bool readWayPoints();

private:
  void calcDPotential2(float now_x, float now_y, float now_z, float tar_x, float tar_y, float tar_z, float& v_x, float&
                      v_y, float& v_z);
  float calcYawSpeed(float v_x, float v_y, float v_z, float yaw_now);

private:
  int cntdown_; 
  int tgt_id_;
  int time_to_pause_;
 
  ros::NodeHandle nh_;
  ros::Publisher tgt_point_pub_;
  std::vector<std::array<float, 3>> path_;
  std::string p2p_filename_;
};

#endif // AI_ROBOT_NAVIGATION_POIN2TPOINT_H_
