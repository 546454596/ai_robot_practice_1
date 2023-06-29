// tof      256  0.2
// lidar    10   0.05
// lidar3d  256  0.2
#include "p3at_obsavoid.h"

#include <cmath>
#include <cstdint>

#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/filter.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>

// -----------------------------------------------------------------------------
// Constants
constexpr int kImgRow = 250;
constexpr int kImgCol = 500;

// -----------------------------------------------------------------------------
// Globals

// Maximum angular speed for abstacle avoidance
double oa_max_angular_speed;
// An octree object used for obstacle detection
const unibn::OctreeParams oct_params = unibn::OctreeParams(10, false, 0.1f);

// -----------------------------------------------------------------------------
// SafeZone public member functions
SafeZone::SafeZone() {}

SafeZone::~SafeZone() {}

float SafeZone::getdTheta(float dyaw, float tgt_dist, float& dist, float& dir, bool& in_safezone) {
  if (right_p[0] < 0 || left_p[0] < 0) { // safezone not existing
    return -10.0;
  }

  float ret;

  in_safezone = false;
  if (dyaw <= safe_dir[1] && dyaw >= safe_dir[0]) {
    in_safezone = true;
    dist = (right_p[0] + left_p[0]) / 2;
    dir = dyaw;
    ret = .0;
  } else if (dyaw < safe_dir[0]) {
    dist = right_p[0];
    dir = (safe_dir[0] + safe_dir[1]) / 2;
    ret = dir - dyaw;
  } else {
    dist = left_p[0];
    dir = (safe_dir[0] + safe_dir[1]) / 2;
    ret = dir - dyaw;
  }

  return ret;
}

void SafeZone::getMidXYZ(double& x, double& y, double& z) {
  float mang = (right_p[1] + left_p[1]) / 2;
  float mdis = (left_p[0] + right_p[0]) / 2;
  x = mdis * cos(mang);
  y = mdis * sin(mang);
  z = 0;
}

bool SafeZone::setSafeZone(float ldis, float lang, float rdis, float rang) {
  left_p[0] = ldis;
  left_p[1] = lang;
  right_p[0] = rdis;
  right_p[1] = rang;

  if (ldis > rdis) {
    near = right_p;
    far = left_p;
  } else {
    near = left_p;
    far = right_p;
  }

  float dtheta = far[1] - near[1];
  float width_total = sqrt(near[0] * near[0] + far[0] * far[0] - 2 * far[0] * near[0] * cos(fabs(dtheta)));

  if (width_total < 0.6) {
    return false;
  } else {
    float theta_near = acos(1 - pow(0.6 / near[0], 2) / 2);
    if (theta_near > fabs(dtheta)) {
      // direction passable space is narrow
      safe_dir[0] = near[1] + fabs(dtheta) / dtheta * theta_near / 2;
      safe_dir[1] = safe_dir[0];
    } else {
      float theta_far = acos(1 - pow(0.6 / far[0], 2) / 2);
      safe_dir[0] = near[1] + fabs(dtheta) / dtheta * theta_near / 2;
      safe_dir[1] = far[1] - fabs(dtheta) / dtheta * theta_far / 2;
      // keep 1 as left and 0 as right
      if (safe_dir[0] > safe_dir[1]) {
        float ex = safe_dir[1];
        safe_dir[1] = safe_dir[0];
        safe_dir[0] = ex;
      }
    }
    return true;
  }
}

void SafeZone::setSimpleSafeZone(float ldis, float lang, float rdis, float rang, int llevel, int rlevel, float deld) {
  if (ldis < 20)
    left_p[0] = ldis;
  else
    left_p[0] = 20;
  left_p[1] = lang;

  if (rdis < 20)
    right_p[0] = rdis;
  else
    right_p[0] = 20;

  right_p[1] = rang;
  safe_dir[0] = rang + acos(1 - pow(0.15 / deld / (rlevel + 1), 2) / 2.3);
  safe_dir[1] = lang - acos(1 - pow(0.15 / deld / (llevel + 1), 2) / 2.3);

  if (safe_dir[0] > safe_dir[1]) {
    safe_dir[0] = (lang + rang) / 2;
    safe_dir[1] = safe_dir[0];
  }
}

// -----------------------------------------------------------------------------
// P3atObstacleAvoidance public member functions
P3atObstacleAvoidance::P3atObstacleAvoidance(const ros::NodeHandle& nh)
    : asonar_(0),
      center_dir_dist_(2),
      cnt_sonar_(0),
      just_replan_(0),
      should_print_time_(false),
      img_trans_(nh),
      in_danger_(false),
      last_desired_dir_(0),
      last_rz_(0),
      last_vx_(0),
      lidar_pc_xyz_(new pcl::PointCloud<pcl::PointXYZ>),
      lidar_used_(false),
      max_rz_(M_PI_2),
      max_vx_(0.5),
      mod_dir_(0),
      nh_(nh),
      orig_dir_(0),
      sonar_used_(false),
      tgt_orient_{1, 0, 0, 0},
      time_of_halfpi_(1.25),
      tof_used_(false) {
  readParameters();
  initPublishersSubscribers();

  if (is_view_safezone_lidar_ && lidar_used_)
    safezone_view_ = cv::Mat(kImgRow, kImgCol, CV_8UC3, cv::Scalar(255, 255, 255)).clone();
  else if (is_view_safezone_tof_ && tof_used_)
    safezone_view_ = cv::Mat(5, 5, CV_8UC3, cv::Scalar(255, 255, 255)).clone();
}

P3atObstacleAvoidance::~P3atObstacleAvoidance() {
  fs_logger_.close();
}

// Inputs a local target directally hence you do not have to publish a target point to this node.
// @local_tgt_p: Local target pose.
// @local_tgt_q: Local target quaternion.
//
// Return: true if the distance to target is not less than 0.1 else false.
bool P3atObstacleAvoidance::gotoLocalTarget(float& vx, float& rz, float local_tgt_p[], float local_tgt_q[], bool findpath) {
  tgt_point_.x = local_tgt_p[0];
  tgt_point_.y = local_tgt_p[1];
  tgt_point_.z = local_tgt_p[2];
  tgt_orient_[0] = local_tgt_q[0];
  tgt_orient_[1] = local_tgt_q[1];
  tgt_orient_[2] = local_tgt_q[2];
  tgt_orient_[3] = local_tgt_q[3];

  double dist = sqrt(pow(local_tgt_p[0], 2) + pow(local_tgt_p[1], 2));
  float dyaw = static_cast<float>(atan2(local_tgt_p[1], local_tgt_p[0]));

  vx = dist / 4;
  if (vx > max_vx_)
    vx = max_vx_;

  rz = dyaw / M_PI_2 * max_rz_;
  if (rz > max_rz_)
    rz = max_rz_;
  else if (rz < -max_rz_)
    rz = -max_rz_;

  modifyVelocity(vx, rz, dyaw, static_cast<float>(dist), findpath);

  if (dist < 0.1)
    return false;

  return true;
}

// Modify robot velocity with the orignal velocity as input. After target goal is reached, set `findpath`to false to do
// reset.
// @vx: Linear velocity along x-axis.
// @rz: Angular velocity along z-axis.
// @dyaw: Direction to target.
// @tgt_dist: Distance to target.
// @findpath: A boolean indicating if in findpath status.
//
// Return: true on velocity modified.
bool P3atObstacleAvoidance::modifyVelocity(float& vx, float& rz, float dyaw, float tgt_dist, bool findpath) {
  int64_t start1 = 0, end1 = 0;
  bool ret;

  start1 = cv::getTickCount();

  if (lidar_used_)
    ret = modifyVelocityByLidar(vx, rz, dyaw, tgt_dist, findpath);
  else if (tof_used_)
    ret = modifyVelocityByTof(vx, rz, dyaw, tgt_dist, findpath);
  else if (lidar3d_used_)
    ret = modifyVelocityByLidar3d(vx, rz, dyaw, tgt_dist, findpath);

  smooth(vx, rz);

  end1 = cv::getTickCount();
  time_intvl_ = 1000 * static_cast<double>(end1 - start1) / cv::getTickFrequency();

  publishSafeZone();

  if (findpath) {
    if (!should_print_time_) {
      should_print_time_ = true;
      cnt_calcpot_ = 0;
      time_calcpot_max_ = 0;
      time_calcpot_tot_ = 0;
    } else {
      if (time_intvl_ > time_calcpot_max_)
        time_calcpot_max_ = time_intvl_;

      fs_logger_ << "Calculated obs time = " << time_intvl_ << "(cv) " << "ms in loop " << cnt_calcpot_++ << "\n\n";
      time_calcpot_tot_ += time_intvl_;
    }
  } else {
    if (should_print_time_) {
      std::cout << "Average calculated obs time:" << time_calcpot_tot_ / cnt_calcpot_ << " ms, max calculated time:" << time_calcpot_max_ << " ms.\n";
      should_print_time_ = false;
      cnt_calcpot_ = 0;
      last_rz_ = 0;
      last_vx_ = 0;
      last_desired_dir_ = 0;
      time_calcpot_max_ = 0;
      time_calcpot_tot_ = 0;
    }
  }

  return ret;
}

bool P3atObstacleAvoidance::modifyVelocityByLidar(float& vx, float& rz, float dyaw, float tgt_dist, bool findpath) {
  if (vx <= 0) { // avoid moving backward
    vx = 0;
    return false;
  }

  if (fabs(dyaw) > 0.7 * half_fov_) { // moving direction is out of detection
    vx = 0.05;
    return true;
  }

  geometry_msgs::Pose msg;
  float ddir_between_now_and_chosen_dir, dist, err_dyaw_safe, new_dir;
  bool in_safezone = false;
  int num_of_nearest_safe = findNearestSafeZone(dyaw, tgt_dist, err_dyaw_safe, dist, new_dir,
                                                ddir_between_now_and_chosen_dir, in_safezone);

  fs_logger_ << "modify:\norigin vx=" << vx << ", rz=" << rz << ", dyaw=" << dyaw / M_PI * 180
    << ", err_dyaw_safe=" << err_dyaw_safe / M_PI * 180 << ", near safezone dist=" << dist << ", new_dir="
    << new_dir / M_PI * 180 << ", ddir_between_now_and_chosen_dir=" << ddir_between_now_and_chosen_dir / M_PI * 180 << '\n';

  if ((num_of_nearest_safe == -1 || err_dyaw_safe > max_err_dir_) && findpath) {
    rz = 0;
    vx = 0;
    if (just_replan_ == 0) {
      fs_logger_ << ros::Time::now().toSec() << " >>>>>>>> No near safe zone and research path.\n";
      ROS_INFO("[OA] no near safe zone and research path.");
      msg.orientation.w = -1;
      msg.orientation.x = -1;
      msg.orientation.y = tgt_point_blocked_;
      msg.position.x = 0;
      msg.position.y = 0;
      msg.position.z = 0;
      just_replan_ = 1000;
      return false;
    } else {
      if (just_replan_ > 0)
        --just_replan_;
      return false;
    }
  } else {
    if (just_replan_ > 0)
      --just_replan_;

    float tmpvx;
    if (center_dir_dist_ > 2)
      tmpvx = max_vx_;
    else
      tmpvx = center_dir_dist_ * max_vx_ / 2.0;

    if (tmpvx < vx)
      vx = tmpvx;

    // method 2: with orientation
    float tgt_yaw, tmp_pitch, tmp_roll;

    if (fabs(tgt_orient_[0]) + fabs(tgt_orient_[1]) + fabs(tgt_orient_[2]) + fabs(tgt_orient_[3]) < 0.1)
      tgt_yaw = 0;
    else
      qToEuler(tmp_roll, tmp_pitch, tgt_yaw, tgt_orient_);

    rz = (a_theta_ + b_alpha_) * new_dir - b_alpha_ * tgt_yaw;
    if (fabs(vx) > 0 && fabs(rz) > 0) {
      float dt_vx = dist / vx;
      float dt_rz = new_dir / rz;
      if (dt_rz > dt_vx)
        vx = dist / dt_rz;
    }

    if (vx > max_vx_)
      vx = max_vx_;
    else if (vx < 0.05)
      vx = 0.05;

    if (rz > max_rz_)
      rz = max_rz_;
    else if (rz < -max_rz_)
      rz = -max_rz_;

    // judge turning radius
    if (min_turn_radius_ > 0 && fabs(rz) != 0) {
      float turn_radius = fabs(vx / rz);
      if (turn_radius < min_turn_radius_) {
        if (in_safezone || center_dir_dist_ > lvl_abs_safe_)
          rz = vx / min_turn_radius_ * rz / fabs(rz);
        else
          vx = 0;
      }
      std::cout << "Turn radius: " << turn_radius << "; is in safe zone: " << in_safezone << ", center distance: " <<
          center_dir_dist_ << "; vx: " << vx << "; rz: " << rz << '\n';
    }
    fs_logger_ << "after vx=" << vx << ", rz=" << rz << '\n';
  }

  return true;
}

bool P3atObstacleAvoidance::modifyVelocityByLidar3d(float& vx, float& rz, float dyaw, float tgt_dist, bool findpath) {
  return modifyVelocityByLidar(vx, rz, dyaw, tgt_dist, findpath);
}

void P3atObstacleAvoidance::modifyVelocityBySonar(float& vx, float& rz) {
  if (!sonar_used_)
    return;

  bool is_going_near[8];
  int nearest_left_point = -1, nearest_right_point = -1;
  float delta_distance[8], distance_now[8], min_left_d = 100, min_right_d = 100;

  if (vx > 0.01) {
    float tmp;
    fs_logger_ << "originvx:" << vx << "; rz:" << rz << '\n';

    double tt;
    for (size_t i = 0; i < 8; ++i) {
      tt = sqrt(pow(sonar_pcl_vec_[0].points[i].x, 2) + pow(sonar_pcl_vec_[0].points[i].y, 2));
      fs_logger_ << tt << " ";
    }
    fs_logger_ << '\n';

    for (size_t i = 0; i < 4; ++i) {
      distance_now[i] = sqrt(pow(sonar_pcl_now_.points[i].x, 2) + pow(sonar_pcl_now_.points[i].y, 2));
      delta_distance[i] = distance_now[i] - sqrt(pow(sonar_pcl_vec_[0].points[i].x, 2) +
                                                 pow(sonar_pcl_vec_[0].points[i].y, 2));
      tmp = fabs(delta_distance[i]);
      fs_logger_ << distance_now[i] << " ";

      if (delta_distance[i] < -0.01 && tmp < 1.7 && distance_now[i] < 3) {
        is_going_near[i] = true;
        // choose fastest closing obs
        if (tmp < min_left_d) {
          min_left_d = tmp;
          nearest_left_point = i;
        }
      } else {
        is_going_near[i] = false;
      }
    }

    for (size_t i = 4; i < 8; ++i) {
      distance_now[i] = sqrt(pow(sonar_pcl_now_.points[i].x, 2) + pow(sonar_pcl_now_.points[i].y, 2));
      delta_distance[i] = distance_now[i] - sqrt(pow(sonar_pcl_vec_[0].points[i].x, 2) +
                                                 pow(sonar_pcl_vec_[0].points[i].y, 2));
      tmp = fabs(delta_distance[i]);
      fs_logger_ << distance_now[i] << " ";

      if (delta_distance[i] < -0.01 && tmp < 1.7 && distance_now[i] < 3) {
        is_going_near[i] = true;
        // choose fastest closing obs
        if (tmp < min_right_d) {
          min_right_d = tmp;
          nearest_right_point = i;
        }
      } else {
        is_going_near[i] = false;
      }
    }

    fs_logger_ << "\nnear left point:" << nearest_left_point << "; near right point:" << nearest_right_point << '\n';

    float delvx = 0, delrz = 0;

    if (nearest_left_point != -1 && (nearest_right_point == -1 || min_left_d < min_right_d)) {
      delvx += asonar_ * (a_sonar_[nearest_left_point][0] * delta_distance[nearest_left_point] /
          distance_now[nearest_left_point] / fabs(sonar_time_now_ - sonar_time_past_[0]));
      delrz += asonar_ * (a_sonar_[nearest_left_point][1] * delta_distance[nearest_left_point] /
          distance_now[nearest_left_point] / fabs(sonar_time_now_ - sonar_time_past_[0]));
      fs_logger_ << "left delta d:" << delta_distance[nearest_left_point] << "; time:"
          << fabs(sonar_time_now_ - sonar_time_past_[0]) << "\ndvx:" << delvx << "; drz:" << delrz << '\n';
    } else if (nearest_right_point != -1 && (nearest_left_point == -1 || min_right_d < min_left_d)) {
      delvx += asonar_ * (a_sonar_[nearest_right_point][0] * delta_distance[nearest_right_point] /
          distance_now[nearest_right_point] / fabs(sonar_time_now_ - sonar_time_past_[0]));
      delrz += asonar_ * (a_sonar_[nearest_right_point][1] * delta_distance[nearest_right_point] /
          (-distance_now[nearest_right_point]) / fabs(sonar_time_now_ - sonar_time_past_[0]));
      fs_logger_ << "right delta d:" << delta_distance[nearest_right_point] << "; time:"
          << fabs(sonar_time_now_ - sonar_time_past_[0]) << "\ndvx:" << delvx << "; drz:" << delrz << '\n';
    }

    // sonar (not used)
    // if (false) {
    //   if ((delrz * rz) < 0)
    //     rz = -rz;
    //   else
    //       rz += delrz;
    //
    //   if (distance_now[0] < distance_now[7] && distance_now[0] < 0.45)
    //     rz = -0.15;
    //   else if (distance_now[7] < distance_now[0] && distance_now[7] < 0.45)
    //       rz = 0.15;
    //
    //   vx += delvx;
    //   if (vx < 0)
    //       vx = 0;
    // }

    if (distance_now[2] < 0.7 || distance_now[3] < 0.9 || distance_now[4] < 0.9 || distance_now[5] < 0.7) {
      vx = 0;
      rz = 0;
    }

    fs_logger_ << "final vx:" << vx << "; rz:" << rz << '\n';
  }

  fs_logger_ << '\n';
}

void P3atObstacleAvoidance::modifyVelocityBySonarSafeZone(float& vx, float& rz) {
  if (!sonar_used_)
    return;

  if (vx > 0.01) {
    float delta_d, distance_now[8], min_d = 100;
    bool safezone[8];

    for (size_t i = 0; i < 8; ++i) {
      distance_now[i] = sqrt(pow(sonar_pcl_now_.points[i].x, 2) + pow(sonar_pcl_now_.points[i].y, 2));
      if (distance_now[i] < min_d)
        min_d = distance_now[i];
    }
    for (size_t i = 0; i < 8; ++i) {
      delta_d = distance_now[i] - min_d;
      if (delta_d > 1)
        safezone[i] = true;
      else
        safezone[i] = false;
    }
  }
}

bool P3atObstacleAvoidance::modifyVelocityByTof(float& vx, float& rz, float dyaw, float tgt_dist, bool findpath) {
  return modifyVelocityByLidar(vx, rz, dyaw, tgt_dist, findpath);
}

void P3atObstacleAvoidance::publishSafeZone() {
  int imglx, imgly, imgrx, imgry;
  std_msgs::Float32MultiArray msg;

  for (auto& elem : safezone_vec_) {
    msg.data.push_back(elem.left_p[0]); // distance to left corner of obstacle
    msg.data.push_back(elem.left_p[1]); // direction to left corner of obstacle
    msg.data.push_back(elem.right_p[0]); // distance to right corner of obstacle
    msg.data.push_back(elem.right_p[1]); // direction to right corner of obstacle

    // for view
    if (is_view_safezone_lidar_) {
      // draw safezone
      imglx = kImgCol / 2 - int(elem.left_p[0] * sin(elem.left_p[1]) * kImgRow / 5);
      imgly = kImgRow - 1 - int(elem.left_p[0] * cos(elem.left_p[1]) * kImgRow / 5);
      imgrx = kImgCol / 2 - int(elem.right_p[0] * sin(elem.right_p[1]) * kImgRow / 5);
      imgry = kImgRow - 1 - int(elem.right_p[0] * cos(elem.right_p[1]) * kImgRow / 5);
      cv::line(safezone_view_, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(0, 255, 0), 1);
    }
  }

  msg.data.push_back(orig_dir_);
  msg.data.push_back(mod_dir_);
  safezone_pub_.publish(msg);

  if (is_view_safezone_lidar_) {
    cv::imshow("obstacleview", safezone_view_);
    cv::waitKey(1);
  }
}

void P3atObstacleAvoidance::reset() {
  float tmp = 0.0;
  modifyVelocity(tmp, tmp, 0, 0, false);
}

// **** P3atObstacleAvoidance private member functions ****
void P3atObstacleAvoidance::initPublishersSubscribers() {
  replan_pub_ = nh_.advertise<geometry_msgs::Pose>("/ai_robot/restart_nav", 1);
  safezone_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/obstacle/safezone", 1);
  tgtpose_sub_ = nh_.subscribe("/ai_robot/findpath/target_point", 1, &P3atObstacleAvoidance::targetPoseCallback, this);

  if (lidar_used_)
    lidar_sub_ = nh_.subscribe("/base_scan", 1, &P3atObstacleAvoidance::lidarCallback, this);
  else if (lidar3d_used_)
    lidar3d_sub_ = nh_.subscribe("/points", 1, &P3atObstacleAvoidance::lidar3dCallback, this);
  else if (sonar_used_)
    sonar_pcl2_sub_ = nh_.subscribe("/RosAria/sonar_pointcloud2", 1, &P3atObstacleAvoidance::sonarPclCallback, this);
  else if (tof_used_)
    tof_pcl_sub_ = nh_.subscribe("/points", 1, &P3atObstacleAvoidance::tofPclCallback, this);
}

void P3atObstacleAvoidance::readParameters() {
  nh_.getParam("use_sonar", sonar_used_);
  nh_.getParam("use_lidar", lidar_used_);
  nh_.getParam("use_tof", tof_used_);
  nh_.getParam("use_lidar3d", lidar3d_used_);
  nh_.getParam("a00", a_sonar_[0][0]);
  nh_.getParam("a01", a_sonar_[0][1]);
  nh_.getParam("a10", a_sonar_[1][0]);
  nh_.getParam("a11", a_sonar_[1][1]);
  nh_.getParam("a20", a_sonar_[2][0]);
  nh_.getParam("a21", a_sonar_[2][1]);
  nh_.getParam("a30", a_sonar_[3][0]);
  nh_.getParam("a31", a_sonar_[3][1]);
  nh_.getParam("a30", a_sonar_[4][0]);
  nh_.getParam("a31", a_sonar_[4][1]);
  nh_.getParam("a20", a_sonar_[5][0]);
  nh_.getParam("a21", a_sonar_[5][1]);
  nh_.getParam("a10", a_sonar_[6][0]);
  nh_.getParam("a11", a_sonar_[6][1]);
  nh_.getParam("a00", a_sonar_[7][0]);
  nh_.getParam("a01", a_sonar_[7][1]);
  nh_.getParam("asonar", asonar_);

  nh_.param("largest_err_direction", max_err_dir_, 1.57);
  nh_.param("max_vx", max_vx_, 0.5);
  nh_.param("max_rz", max_rz_, M_PI_2);
  nh_.param("min_turn_radius", min_turn_radius_, 0.0);
  nh_.param("safezone_rest_wide", safezone_rest_wide_, 0.5);
  nh_.param("time_of_halfpi", time_of_halfpi_, 1.0);
  nh_.param("safezone_lvl_err_threshold", safezone_lvl_err_thresh_, 1.0);
  nh_.param("lvl_absolute_safe", lvl_abs_safe_, 1.0);
  nh_.param("safezone_min_err_range", safezone_min_err_range_, 5.0);

  nh_.getParam("view_safezone_lidar", is_view_safezone_lidar_);
  nh_.getParam("view_safezone_tof", is_view_safezone_tof_);

  nh_.param("lidar_x", t_lidar_body_[0], 0.0);
  nh_.param("lidar_y", t_lidar_body_[1], 0.0);
  nh_.param("lidar_z", t_lidar_body_[2], 0.0);
  nh_.param("half_fov", half_fov_, 0.52);

  if (tof_used_) {
    nh_.param("initial_height_of_cam", cam_initial_height_, -0.42);
    nh_.param("obs_times_of_deld_tof", times_of_deld_, 4);
    nh_.param("obs_dangle_tof", dangle_, 0.058);
    nh_.param("deld_tof", deld_, 0.46);
    nh_.param("num_of_layers", nlayers_, 1);
    nh_.param("deadzone", deadzone_, 0.2);
    nh_.param("range_step_size", range_step_size_, 0.2);
  } else if (lidar_used_) {
    okdata_half_fov_ = time_of_halfpi_ * M_PI_2;
    nh_.param("obs_times_of_deld", times_of_deld_, 4);
    nh_.param("obs_dangle", dangle_, 0.058);
    nh_.param("deld", deld_, 0.5);
    nh_.param("deadzone", deadzone_, 0.2);
    nh_.param("range_step_size", range_step_size_, 0.2);
  } else if (lidar3d_used_) {
    nh_.param("initial_height_of_lidar3d", lidar3d_initial_height_, -0.42);
    nh_.param("obs_times_of_deld", times_of_deld_, 4);
    nh_.param("obs_dangle", dangle_, 0.058);
    nh_.param("deld", deld_, 0.5);
    nh_.param("num_of_layers", nlayers_, 1);
    nh_.param("deadzone", deadzone_, 0.2);
    nh_.param("range_step_size", range_step_size_, 0.2);
  }

  nh_.param("oa_max_angular_speed", oa_max_angular_speed, M_PI_2);
  nh_.param("a_theta", a_theta_, 0.5);
  nh_.param("b_alpha", b_alpha_, 0.1);
  nh_.param("c_dist", c_dist_, 0.25);
  nh_.param("det_tolerance", det_tol_, 2.0);

  std::string oalog_filepath;
  nh_.getParam("oalog_filename", oalog_filepath);
  if (!oalog_filepath.empty())
    fs_logger_.open(oalog_filepath);
}

void P3atObstacleAvoidance::sonarPclCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  sonar_pcl_now_.clear();
  pcl::fromROSMsg(*msg, sonar_pcl_now_);
  sonar_time_now_ = ros::Time::now().toSec();
  // every time only one of 16 sonar data gets updated
  if (++cnt_sonar_ >= 16) {
    cnt_sonar_ = 0;
    sonar_time_past_.push_back(sonar_time_now_);
    sonar_pcl_vec_.push_back(sonar_pcl_now_);
    if (sonar_pcl_vec_.size() > 1) {
      sonar_pcl_vec_.erase(sonar_pcl_vec_.begin());
      sonar_time_past_.erase(sonar_time_past_.begin());
    }
  }
}

void P3atObstacleAvoidance::lidarCallback(const sensor_msgs::LaserScanConstPtr& msg) {
  int64_t tc_begin = 0, tc_end = 0;
  tc_begin = cv::getTickCount();
  findSafeZoneOctree(msg);
  tc_end = cv::getTickCount();
  time_intvl2_ = 1000 * double(tc_begin - tc_end) / cv::getTickFrequency();
}

// A utility function for code reuse.
void P3atObstacleAvoidance::checkSafeZone(pcl::PointXYZ& self1, pcl::PointXYZ& self2,
                                          pcl::PointXYZ& tgtp1, pcl::PointXYZ& tgtp2,
                                          cv::Point2f& start_point, cv::Point2f& end_point,
                                          std::vector<int>& all_levels, int& llevel, int& rlevel,
                                          int& num_of_dir, SafeZone& safezone) {
  int level, next_level, step;
  if (lidar_xyz_oct_.isBlock<unibn::L2Distance<pcl::PointXYZ>>(tgtp2, self2, 0.6) ||
      lidar_xyz_oct_.isBlock<unibn::L2Distance<pcl::PointXYZ>>(tgtp1, self1, 0.6)) {
    tgt_point_blocked_ = 1;
    safezone.setSimpleSafeZone(-1, 0, -1, 0, 0, 0, deld_);
    safezone_vec_.insert(safezone_vec_.begin(), safezone);
  } else {
    tgt_point_blocked_ = -1;
    level = all_levels[num_of_dir];
    next_level = level;
    step = 0;

    if (level >= 1) {
      while (next_level == level && step < 15 && (num_of_dir - step) > 0) {
        next_level = all_levels[num_of_dir - step];
        ++step;
      }

      if (level <= next_level)
        next_level = level - 1;

      start_point.x = (0.5 + next_level) * deld_;
      start_point.y = -half_fov_ + dangle_ * (num_of_dir - step + 1);
      llevel = level - 1;
      step = 0;
      next_level = level;

      while (next_level == level && step < 15 && (num_of_dir + step) < all_levels.size()) {
        next_level = all_levels[num_of_dir + step];
        ++step;
      }

      if (level <= next_level)
        next_level = level - 1;

      end_point.x = (0.5 + next_level) * deld_;
      end_point.y = dangle_ * (num_of_dir + step - 1) - half_fov_;
      rlevel = level - 1;
      safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
      safezone_vec_.insert(safezone_vec_.begin(), safezone);
    }
  }
}

void P3atObstacleAvoidance::lidar3dCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  bool obs_existed = false, sn_started = false, jumpout = false;
  int cnt_safescan = 0;
  int level, llevel = 0, rlevel = 0;
  int last_dist_level = times_of_deld_ - 1;
  int num_of_dir = -1;
  int closest_level;
  float mid_angle = 1;
  float min_err = 100;
  float target_dir = atan2(tgt_point_.y, tgt_point_.x);
  double tmp_angle;
  std::vector<int> all_levels;
  std::vector<int> nouse;
  pcl::PointXYZ tmp_p(0, 0, 0);
  cv::Point2f start_point(2.03, 0), end_point(2.03, 0);
  SafeZone tmp_safezone;

  lidar_pc_xyz_->clear();
  safezone_vec_.clear();
  pcl::fromROSMsg(*msg, *lidar_pc_xyz_);
  pcl::removeNaNFromPointCloud(*lidar_pc_xyz_, *lidar_pc_xyz_, nouse);
  lidar_xyz_oct_.initialize(lidar_pc_xyz_->points, oct_params);

  // find safezone
  int i = 0;
  int cnt_test = 0;
  float delx, dely, firstx, firsty;
  for (tmp_angle = -half_fov_; tmp_angle <= half_fov_; tmp_angle += dangle_) {
    delx = deld_ * cos(tmp_angle);
    dely = deld_ * sin(tmp_angle);
    firstx = deadzone_ * cos(tmp_angle);
    firsty = deadzone_ * sin(tmp_angle);
    closest_level = times_of_deld_;
    jumpout = false;
    obs_existed = false;
    tmp_p.x = delx / 2 - t_lidar_body_[0] + firstx;
    tmp_p.y = dely / 2 - t_lidar_body_[1] + firsty;

    for (i = 0; i < times_of_deld_; ++i) {
      for (int j = 0; j < nlayers_; ++j) {
        // search circle center
        tmp_p.z = lidar3d_initial_height_ + deld_ * j * 0.8;
        if (lidar_xyz_oct_.radiusNeighborsSimple<unibn::L2Distance<pcl::PointXYZ>>(tmp_p, deld_ / 2)) {
          jumpout = true;
          obs_existed = true;
          if (i < closest_level)
            closest_level = i;
          break;
        }
      }
      if (jumpout)
        break;
      tmp_p.x += delx;
      tmp_p.y += dely;
    }

    if (fabs(tmp_angle) < mid_angle) {
      center_dir_dist_ = closest_level * range_step_size_;
      mid_angle = fabs(tmp_angle);
    }

    all_levels.push_back(closest_level);
 
    fs_logger_ << tmp_angle / M_PI * 180 << ":" << closest_level << ",";
    if (++cnt_test % 10 == 0)
      fs_logger_ << '\n';

    if (fabs(tmp_angle - target_dir) < min_err) {
      num_of_dir = all_levels.size() - 1;
      min_err = fabs(tmp_angle - target_dir);
    }

    if (sn_started) {
      ++cnt_safescan;
      if (closest_level < (last_dist_level) || (last_dist_level >= times_of_deld_ && closest_level < last_dist_level)) {
          // find a safezone
          sn_started = false;
          end_point.x = closest_level * deld_ + deadzone_;
          end_point.y = tmp_angle - dangle_;
          llevel = closest_level;
          tmp_safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
          safezone_vec_.push_back(tmp_safezone);
      } else if (closest_level > last_dist_level) {
        start_point.x = last_dist_level * deld_ + deadzone_;
        start_point.y = tmp_angle;
        rlevel = last_dist_level;
        if (closest_level > (last_dist_level + 1))
          cnt_safescan = 1;
        else
          cnt_safescan = 0;
      }
    } else {
      // no obs or sudden start
      if ((closest_level > (last_dist_level)) || !obs_existed) {
        sn_started = true;
        start_point.x = last_dist_level * deld_ + deadzone_;
        start_point.y = tmp_angle;
        rlevel = closest_level;
        if (closest_level > (last_dist_level + 1))
          cnt_safescan = 1;
        else
          cnt_safescan = 0;
      }
    }
    last_dist_level = closest_level;
  }

  fs_logger_ << '\n';

  if (sn_started && last_dist_level >= times_of_deld_ - 1) {
    end_point.x = 2;
    end_point.y = tmp_angle - dangle_;
    llevel = i;
    tmp_safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
    safezone_vec_.push_back(tmp_safezone);
  }

  pcl::PointXYZ self1(0, 0, lidar3d_initial_height_ + deld_ * 0.8);
  pcl::PointXYZ self2(0, 0, lidar3d_initial_height_);
  pcl::PointXYZ tgtp1(tgt_point_.x, tgt_point_.y, lidar3d_initial_height_ + deld_ * 0.8);
  pcl::PointXYZ tgtp2(tgt_point_.x, tgt_point_.y, lidar3d_initial_height_);

  checkSafeZone(self1, self2, tgtp1, tgtp2, start_point, end_point, all_levels, llevel, rlevel, num_of_dir, tmp_safezone);

  // if (lidar_xyz_oct_.isBlock<unibn::L2Distance<pcl::PointXYZ>>(tgtp2, self2, 0.6) ||
  //     lidar_xyz_oct_.isBlock<unibn::L2Distance<pcl::PointXYZ>>(tgtp1, self1, 0.6)) {
  //   tgt_point_blocked_ = 1;
  //   tmp_safezone.setSimpleSafeZone(-1, 0, -1, 0, 0, 0, deld_);
  //   safezone_vec_.insert(safezone_vec_.begin(), tmp_safezone);
  // } else {
  //   tgt_point_blocked_ = -1;
  //   int level = all_levels[num_of_dir], next_level = level, step = 0;
  //   if (level >= 1) {
  //     while (next_level == level && step < 15 && (num_of_dir - step) > 0) {
  //       next_level = all_levels[num_of_dir - step];
  //       ++step;
  //     }
  //
  //     if (level <= next_level)
  //       next_level = level - 1;
  //
  //     start_point.x = (0.5 + next_level) * deld_;
  //     start_point.y = -half_fov_ + dangle_ * (num_of_dir - step + 1);
  //     llevel = level - 1;
  //     step = 0;
  //     next_level = level;
  //
  //     while (next_level == level && step < 15 && (num_of_dir + step) < all_levels.size()) {
  //       next_level = all_levels[num_of_dir + step];
  //       ++step;
  //     }
  //
  //     if (level <= next_level)
  //       next_level = level - 1;
  //
  //     end_point.x = (0.5 + next_level) * deld_;
  //     end_point.y = -half_fov_ + dangle_ * (num_of_dir + step - 1);
  //     rlevel = level - 1;
  //     tmp_safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
  //     safezone_vec_.insert(safezone_vec_.begin(), tmp_safezone);
  //   }
  // }
}

void P3atObstacleAvoidance::targetPoseCallback(const geometry_msgs::PoseConstPtr& msg) {
  tgt_point_.x = msg->position.x;
  tgt_point_.y = msg->position.y;
  tgt_point_.z = msg->position.z;
  tgt_orient_[0] = msg->orientation.w;
  tgt_orient_[1] = msg->orientation.x;
  tgt_orient_[2] = msg->orientation.y;
  tgt_orient_[3] = msg->orientation.z;
}

void P3atObstacleAvoidance::tofPclCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  lidar_pc_xyz_->clear();
  safezone_vec_.clear();

  pcl::PointCloud<pcl::PointXYZ> tmp_pcl;
  pcl::fromROSMsg(*msg, *lidar_pc_xyz_);
  std::vector<int> nouse;
  std::vector<int> all_levels;
  pcl::removeNaNFromPointCloud(*lidar_pc_xyz_, *lidar_pc_xyz_, nouse);

  // choose less points
  pcl::PointXYZ tmp_p(0, 0, 0);
  lidar_xyz_oct_.initialize(lidar_pc_xyz_->points, oct_params);

  bool obs_existed = false, sn_started = false;
  SafeZone tmp_safezone;
  cv::Point2f start_point(2.03, 0), end_point(2.03, 0); // this store x and y

  // every 10/3 degree
  int cnt_safescan = 0;
  int cnt_test = 0;
  int closest_level, llevel = 0, rlevel = 0;
  int i = 0;
  int last_dist_level = times_of_deld_ - 1;
  int num_of_dir = -1;
  float mid_tmp_angle = 1;
  float targetdir = atan2(tgt_point_.y, tgt_point_.x), min_err = 100;
  double tmp_angle;

  float delx, dely, fristx, firsty;
  for (tmp_angle = -half_fov_; tmp_angle <= half_fov_; tmp_angle += dangle_) {
    float delx = deld_ * cos(tmp_angle), dely = deld_ * sin(tmp_angle);
    float firstx = (deadzone_)*cos(tmp_angle), firsty = (deadzone_)*sin(tmp_angle);
    // two layers
    obs_existed = false;
    bool jumpout = false;
    closest_level = times_of_deld_;
    tmp_p.z = delx / 2 - t_lidar_body_[0] + firstx;
    tmp_p.x = -dely / 2 - t_lidar_body_[1] + firsty;
    for (i = 0; i < times_of_deld_; ++i) {
      for (int j = 0; j < nlayers_; ++j) {
        tmp_p.y = -cam_initial_height_ - deld_ * j;
        if (lidar_xyz_oct_.radiusNeighborsSimple<unibn::L2Distance<pcl::PointXYZ>>(tmp_p, deld_ / 2)) {
          obs_existed = true;
          jumpout = true;
          if (i < closest_level)
            closest_level = i;
          break;
        }
      }
      if (jumpout)
        break;
      tmp_p.z += delx;
      tmp_p.x -= dely;
    }

    if (fabs(tmp_angle) < mid_tmp_angle) {
      center_dir_dist_ = closest_level * range_step_size_;
      mid_tmp_angle = fabs(tmp_angle);
    }

    all_levels.push_back(closest_level);

    fs_logger_ << tmp_angle / M_PI * 180 << ":" << closest_level << ",";
    if (++cnt_test % 10 == 0)
      fs_logger_ << '\n';

    if (fabs(tmp_angle - targetdir) < min_err) {
      num_of_dir = all_levels.size() - 1;
      min_err = fabs(tmp_angle - targetdir);
    }

    if (sn_started) {
      ++cnt_safescan;
      if (closest_level < (last_dist_level) ||
        (last_dist_level >= times_of_deld_ &&
        closest_level < last_dist_level)) {
          // find a safezone
          sn_started = false;
          end_point.x = closest_level * deld_ + deadzone_;
          end_point.y = tmp_angle - dangle_;
          llevel = closest_level;
          tmp_safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
          safezone_vec_.push_back(tmp_safezone);
        } else if (closest_level > last_dist_level) {
        start_point.x = last_dist_level * deld_ + deadzone_;
        start_point.y = tmp_angle;
        rlevel = last_dist_level;
        if (closest_level > (last_dist_level + 1))
          cnt_safescan = 1;
        else
          cnt_safescan = 0;
      }
    } else {
      // no obs or sudden start
      if ((closest_level > (last_dist_level)) || !obs_existed) {
        sn_started = true;
        start_point.x = last_dist_level * deld_ + deadzone_;
        start_point.y = tmp_angle;
        rlevel = closest_level;
        if (closest_level > (last_dist_level + 1))
          cnt_safescan = 1;
        else
          cnt_safescan = 0;
      }
    }
    last_dist_level = closest_level;
  }

  fs_logger_ << '\n';

  if (sn_started && last_dist_level >= times_of_deld_ - 1) {
    end_point.x = 2;
    end_point.y = tmp_angle - dangle_; // + 0.149
    llevel = i;
    tmp_safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
    safezone_vec_.push_back(tmp_safezone);
  }

  pcl::PointXYZ self2(0, -cam_initial_height_, 0);
  pcl::PointXYZ self1(0, -cam_initial_height_ - deld_ * 0.8, 0);
  pcl::PointXYZ tgtp2(-tgt_point_.y, -cam_initial_height_, tgt_point_.x);
  pcl::PointXYZ tgtp1(-tgt_point_.y, -cam_initial_height_ - deld_ * 0.8, tgt_point_.x);

  checkSafeZone(self1, self2, tgtp1, tgtp2, start_point, end_point, all_levels, llevel, rlevel, num_of_dir, tmp_safezone);

  // if (lidar_xyz_oct_.isBlock<unibn::L2Distance<pcl::PointXYZ>>(tgtp2, self2, 0.6) ||
  //     lidar_xyz_oct_.isBlock<unibn::L2Distance<pcl::PointXYZ>>(tgtp1, self1, 0.6)) {
  //   tgt_point_blocked_ = 1;
  //   tmp_safezone.setSimpleSafeZone(-1, 0, -1, 0, 0, 0, deld_);
  //   safezone_vec_.insert(safezone_vec_.begin(), tmp_safezone);
  // } else {
  //   tgt_point_blocked_ = -1;
  //   int level = all_levels[num_of_dir], next_level = level, step = 0;
  //   if (level >= 1) {
  //     while (next_level == level && step < 15 && (num_of_dir - step) > 0) {
  //       next_level = all_levels[num_of_dir - step];
  //       ++step;
  //     }
  //
  //     if (level <= next_level) {
  //       next_level = level - 1;
  //     }
  //
  //     start_point.x = (0.5 + next_level) * deld_;
  //     start_point.y = -half_fov_ + dangle_ * (num_of_dir - step + 1);
  //     llevel = level - 1;
  //     step = 0;
  //     next_level = level;
  //
  //     while (next_level == level && step < 15 && (num_of_dir + step) < all_levels.size()) {
  //       next_level = all_levels[num_of_dir + step];
  //       ++step;
  //     }
  //
  //     if (level <= next_level)
  //       next_level = level - 1;
  //
  //     end_point.x = (0.5 + next_level) * deld_;
  //     end_point.y = -half_fov_ + dangle_ * (num_of_dir + step - 1);
  //     rlevel = level - 1;
  //     tmp_safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
  //     safezone_vec_.insert(safezone_vec_.begin(), tmp_safezone);
  //   }
  // }
}

void P3atObstacleAvoidance::findSafeZone(const sensor_msgs::LaserScanConstPtr& msg) {
  safezone_view_ = cv::Mat(kImgRow, kImgCol, CV_8UC3, cv::Scalar(255, 255, 255)).clone();
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 200, cv::Scalar(100, 100, 0), 1);
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 400, cv::Scalar(100, 100, 0), 1);
  float angle = msg->angle_min;
  lidar_range_now_.clear();
  // 0-right, max-left
  bool start_safe_zone = false;
  int num_of_range = 0;
  safezone_vec_.clear();
  SafeZone tmp;
  float ldis, lang, rdis, rang;
  int danger_points = 0;
  in_danger_ = false;

  // method 3
  std::vector<cv::Point2f> start_points, end_points,
  over2_points; // this store dis in x and angle in y
  bool isOver2Exist = false;

  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (angle > M_PI / 2) {
      if (msg->ranges[i] > 2) {
        end_points.push_back(cv::Point2f(2, angle - msg->angle_increment));
        if (isOver2Exist) {
          over2_points.push_back(cv::Point2f(2, angle - msg->angle_increment));
        }
      }
      break;
    }
    if (angle >= -M_PI / 2) {
      lidar_range_now_.push_back(msg->ranges[i]);
      if (fabs(angle) < 0.46 && msg->ranges[i] < 0.6)
        ++danger_points;
      // for view
      if (is_view_safezone_lidar_) {
        int imgx, imgy;
        imgx = kImgCol / 2 - int(msg->ranges[i] * sin(angle) * kImgRow / 5);
        imgy = kImgRow - 1 - int(msg->ranges[i] * cos(angle) * kImgRow / 5);
        if (imgx >= 0 && imgx < kImgCol && imgy >= 0 && imgy < kImgRow) {
          cv::circle(safezone_view_, cv::Point(imgx, imgy), 1, cv::Scalar(0, 0, 0), 2);
        }
      }

      // add distance over 2 possible safezone edge
      if (!isOver2Exist) {
        if (msg->ranges[i] > 2) {
          isOver2Exist = true;
          over2_points.push_back(
            cv::Point2f(msg->ranges[i - 1], angle - msg->angle_increment));
        }
      } else {
        if (msg->ranges[i] < 2) {
          isOver2Exist = false;
          over2_points.push_back(cv::Point2f(msg->ranges[i], angle));
        }
      }

      if (msg->ranges[i] < 4) {
        if ((msg->ranges[i] - msg->ranges[i + 1]) < -0.5 &&
          (msg->ranges[i] - msg->ranges[i + 2]) < -0.5 &&
          (msg->ranges[i] - msg->ranges[i + 3]) < -0.5) {
            start_points.push_back(
              cv::Point2f(msg->ranges[i - 1], angle - msg->angle_increment));
          }
          // sudden far to close
        else if ((msg->ranges[i] - msg->ranges[i - 1]) < -0.5 &&
            (msg->ranges[i] - msg->ranges[i - 2]) < -0.5 &&
            (msg->ranges[i] - msg->ranges[i - 3]) < -0.5) {
        end_points.push_back(cv::Point2f(msg->ranges[i], angle));
      }
      }
      ++num_of_range;
    } // end of method 3,not finish

    angle += msg->angle_increment;
  }

  // method 3, connect start end of safezone
  int ii = 0, jj = 0;
  if (is_view_safezone_lidar_) {
    int imgx, imgy;
    for (ii = 0; ii < start_points.size(); ++ii) {
      imgx = kImgCol / 2 - int(start_points[ii].x * sin(start_points[ii].y) * kImgRow / 5);
      imgy = kImgRow - 1 - int(start_points[ii].x * cos(start_points[ii].y) * kImgRow / 5);
      cv::circle(safezone_view_, cv::Point(imgx, imgy), 3, cv::Scalar(0, 255, 0), 3);
    }
    for (ii = 0; ii < end_points.size(); ++ii) {
      imgx = kImgCol / 2 - int(end_points[ii].x * sin(end_points[ii].y) * kImgRow / 5);
      imgy = kImgRow - 1 - int(end_points[ii].x * cos(end_points[ii].y) * kImgRow / 5);
      cv::circle(safezone_view_, cv::Point(imgx, imgy), 3, cv::Scalar(255, 0, 0), 3);
    }
    for (ii = 0; ii < over2_points.size(); ++ii) {
      imgx = kImgCol / 2 - int(over2_points[ii].x * sin(over2_points[ii].y) * kImgRow / 5);
      imgy = kImgRow - 1 - int(over2_points[ii].x * cos(over2_points[ii].y) * kImgRow / 5);
      cv::circle(safezone_view_, cv::Point(imgx, imgy), 3, cv::Scalar(70, 120, 230), 3);
    }
  }
  // sudden start end point safezone
  for (ii = 0; ii < start_points.size(); ++ii) {
    while (jj < end_points.size() && end_points[jj].y <= start_points[ii].y) {
      ++jj;
    }
    float dist_start_end = 10000, tmp_dist;
    int nearest_end = -1;
    for (int tj = jj; tj < end_points.size(); ++tj) {
      tmp_dist = start_points[ii].x * start_points[ii].x +
        end_points[tj].x * end_points[tj].x -
        2 * start_points[ii].x * end_points[tj].x *
        cos(fabs(start_points[ii].y - end_points[tj].y));
      if (tmp_dist < dist_start_end) {
        dist_start_end = tmp_dist;
        nearest_end = tj;
      }
    }
    int nearest_start = ii;
    dist_start_end = 10000;
    for (int ti = ii; ti < start_points.size() && start_points[ti].y < end_points[nearest_end].y; ++ti) {
      tmp_dist = start_points[ti].x * start_points[ti].x + end_points[nearest_end].x * end_points[nearest_end].x - 2 *
        start_points[ti].x * end_points[nearest_end].x * cos(fabs(start_points[ti].y - end_points[nearest_end].y));
      if (tmp_dist < dist_start_end) {
        dist_start_end = tmp_dist;
        nearest_start = ti;
      }
    }
    if (nearest_end != -1) {
      jj = nearest_end;
      if (ii < start_points.size() && nearest_end < end_points.size()) {
        if (tmp.setSafeZone(
          end_points[nearest_end].x, end_points[nearest_end].y,
          start_points[nearest_start].x, start_points[nearest_start].y)) {
          safezone_vec_.push_back(tmp);
        }
      }
      while (end_points[jj].y > start_points[ii].y) {
        ++ii;
      }
      --ii;
    } else {
      break;
    } // end of method 3.2
  }
  // over 2m safezone
  std::vector<SafeZone> over2_vec;
  for (ii = 0; ii + 1 < over2_points.size();) {
    if (tmp.setSafeZone(over2_points[ii + 1].x, over2_points[ii + 1].y, over2_points[ii].x, over2_points[ii].y))
      over2_vec.push_back(tmp);
    ii += 2;
  }
  // end of method 3

  // remove near safezone
  jj = 0;
  bool need_insert = true;
  int size_of_ori_safezone = safezone_vec_.size();
  for (ii = 0; ii < over2_vec.size(); ++ii) {
    for (; jj < size_of_ori_safezone; ++jj) {
      float rerr = over2_vec[ii].safe_dir[0] - safezone_vec_[jj].safe_dir[0];
      float lerr = over2_vec[ii].safe_dir[1] - safezone_vec_[jj].safe_dir[1];
      float err_dir = (lerr + rerr) / 2;
      if (fabs(err_dir) < 0.05) {
        if (lerr < 0 && rerr > 0) {
          safezone_vec_.erase(safezone_vec_.begin() + jj);
          safezone_vec_.insert(safezone_vec_.begin() + jj, over2_vec[ii]);
          need_insert = false;
          break;
        } else {
          need_insert = false;
          break;
        }
      } else if (err_dir < 0) {
        need_insert = true;
        break;
      }
    }
    if (need_insert)
      safezone_vec_.push_back(over2_vec[ii]);
    else 
      need_insert = true;
  }
  lidar_time_now_ = ros::Time::now().toSec();
  lidar_time_past_.push_back(lidar_time_now_);
  lidar_range_vec_.push_back(lidar_range_now_);
  if (danger_points > 20)
    in_danger_ = true;
  if (lidar_range_vec_.size() > 5) {
    lidar_range_vec_.erase(lidar_range_vec_.begin());
    lidar_time_past_.erase(lidar_time_past_.begin());
  }
}

void P3atObstacleAvoidance::findSafeZoneOctree(const sensor_msgs::LaserScanConstPtr &msg) {
  safezone_view_ = cv::Mat(kImgRow, kImgCol, CV_8UC3, cv::Scalar(255, 255, 255)).clone();
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 50, cv::Scalar(100, 100, 0), 1);
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 100, cv::Scalar(100, 100, 0), 1);
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 150, cv::Scalar(100, 100, 0), 1);
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 200, cv::Scalar(100, 100, 0), 1);
  cv::circle(safezone_view_, cv::Point(kImgCol / 2, kImgRow - 1), 400, cv::Scalar(100, 100, 0), 1);

  lidar_pc_xyz_->clear();
  safezone_vec_.clear();

  int start_in_msg = 0;
  float tmp_angle = msg->angle_min;
  std::vector<int> sudden_change_p;
  pcl::PointXYZ tmp_p(0, 0, 0);

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    if (tmp_angle > okdata_half_fov_)
      break;

    if (tmp_angle >= -okdata_half_fov_) {
      if (is_view_safezone_lidar_) {
        int imgx, imgy;
        // draw lidar points
        imgx = kImgCol / 2 - int((msg->ranges[i] * sin(tmp_angle) + t_lidar_body_[1]) * kImgRow / 5);
        imgy = kImgRow - 1 - int((msg->ranges[i] * cos(tmp_angle) + t_lidar_body_[0]) * kImgRow / 5);
        if (imgx >= 0 && imgx < kImgCol && imgy >= 0 && imgy < kImgRow) {
          cv::circle(safezone_view_, cv::Point(imgx, imgy), 1, cv::Scalar(0, 0, 0), 2);
        }
      }

      if (fabs(msg->ranges[i]) < 10) {
        polar2xy(msg->ranges[i], tmp_angle, tmp_p.x, tmp_p.y);
        lidar_pc_xyz_->push_back(tmp_p);
        // if (msg->ranges[i] < 0.3 || sqrt(tmp_p.x * tmp_p.x + tmp_p.y + tmp_p.y) < 0.3) {
        //   fs_logger_ <<"; errpoint" <<msg->ranges[i] <<" "<<tmp_angle<<", "
        //   <<tmp_p.x<<", "<<tmP.y;
        // }
      }
    } else {
      start_in_msg = i;
    }
    tmp_angle += msg->angle_increment;
  }

  ++start_in_msg;
  lidar_xyz_oct_.initialize(lidar_pc_xyz_->points, oct_params);

  bool obs_existed = false, sn_started = false;
  std::vector<uint32_t> res;
  SafeZone tmp_safezone;
  cv::Point2f start_point(2.03, 0), end_point(2.03, 0); // this store dis in x and angle in y

  // every 10/3 degree
  int cnt_safescan = 0;
  int i = 0;
  int last_obs_point = 0, last_dist_level = -1;
  bool last_obs_exist = false;
  int llevel = 0, rlevel = 0;
  int num_of_dir = -1;
  int sudden_change_id_in_v = 0;
  float mid_angle = 1;
  float tgt_dir = atan2(tgt_point_.y, tgt_point_.x), min_err = 100;
  double last_safezone_leftbound = -M_PI;
  std::vector<int> all_levels;

  for (tmp_angle = -half_fov_; tmp_angle <= half_fov_; tmp_angle += dangle_) {
    float delx = range_step_size_ * cos(tmp_angle),
    dely = range_step_size_ * sin(tmp_angle);
    float firstx = (deadzone_)*cos(tmp_angle), firsty = (deadzone_)*sin(tmp_angle);
    tmp_p.x = -t_lidar_body_[0] + firstx;
    tmp_p.y = -t_lidar_body_[1] + firsty;
    res.clear();
    obs_existed = false;

    for (i = 0; i < times_of_deld_; ++i) {
      lidar_xyz_oct_.radiusNeighbors<unibn::L2Distance<pcl::PointXYZ>>(tmp_p, deld_ / 2, res);
      if (res.size() > det_tol_) {
        obs_existed = true;
        break;
      }
      tmp_p.x += delx;
      tmp_p.y += dely;
    }

    if (fabs(tmp_angle) < mid_angle) {
      center_dir_dist_ = i * range_step_size_;
      mid_angle = fabs(tmp_angle);
    }
    all_levels.push_back(i);

    if (fabs(tmp_angle - tgt_dir) < min_err) {
      num_of_dir = all_levels.size() - 1;
      min_err = fabs(tmp_angle - tgt_dir);
    }

    if (last_dist_level < 0) {
      last_dist_level = i;
      continue;
    }

    if (obs_existed)
      fs_logger_ << i << " " << tmp_angle << '\n';

    if (sn_started) {
      ++cnt_safescan;
      if ((i <= (last_dist_level - safezone_lvl_err_thresh_) || (last_dist_level >= times_of_deld_ && i <
          last_dist_level))) {
        if (fabs(i * range_step_size_ + deadzone_ - start_point.x) < safezone_rest_wide_) {
          // end a safezone(obs_existed && last_dist_level==4) ||
          sn_started = false;
          // because it is the blocked one, so minus to last one
          end_point.x = i * range_step_size_ + deadzone_;
          end_point.y = tmp_angle - dangle_; // + 0.149
          llevel = i;
          fs_logger_ << " end:" << end_point.x << "," << end_point.y << ", cnt_safescan:" << cnt_safescan << '\n';
          float minedge = start_point.x;
          tmp_safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
          safezone_vec_.push_back(tmp_safezone);
          last_safezone_leftbound = end_point.y;
        } else {
          if (start_point.x > (i * range_step_size_ + deadzone_))
            // this start point is not useful
            sn_started = false;
        }
      } else if (i > (last_dist_level)) {
        start_point.x = last_dist_level * range_step_size_ + deadzone_;
        start_point.y = tmp_angle; // - 0.149
        fs_logger_ << "2 start:" << start_point.x << "," << start_point.y << '\n';
        rlevel = i;
        cnt_safescan = 1;

        if (i > (last_dist_level + 1))
          cnt_safescan = 1;
        else
          cnt_safescan = 0;
      }
    } else {
      if ((i >= (last_dist_level + safezone_lvl_err_thresh_)) || !obs_existed) {
        sn_started = true;
        start_point.x = last_dist_level * range_step_size_ + deadzone_;
        start_point.y = tmp_angle; // - 0.149
        rlevel = i;
        cnt_safescan = 1;
        fs_logger_ << "1 start:" << start_point.x << "," << start_point.y << '\n';
      } else if ((i <= (last_dist_level - safezone_lvl_err_thresh_))) {
        end_point.x = i * range_step_size_ + deadzone_;
        end_point.y = tmp_angle - dangle_;
        llevel = i;

        int tmplevel = last_dist_level, tmplastlvl = i, tmpcnt = all_levels.size() - 2;
        while (fabs(end_point.x - tmplevel * range_step_size_ + deadzone_) > safezone_rest_wide_ || (tmplevel >=
            (tmplastlvl - safezone_lvl_err_thresh_))) {
          tmplastlvl = tmplevel;
          tmplevel = all_levels[tmpcnt];
          if (--tmpcnt < 0)
            break;
        }

        start_point.x = tmplevel * range_step_size_ + deadzone_;
        start_point.y = dangle_ * (tmpcnt + 1) - half_fov_;
        // removed parts: && fabs(end_point.y-start_point.y) >= 0.5*dangle_
        if (start_point.y > last_safezone_leftbound) {
          // add new safezone when two safezones donot overlap
          rlevel = tmplevel;
          tmp_safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
          safezone_vec_.push_back(tmp_safezone);
          last_safezone_leftbound = end_point.y;
        } else {
          double diffdist1 = fabs(start_point.x - end_point.x);
          double diffdist2 = fabs(safezone_vec_[safezone_vec_.size() - 1].right_p[0] -
                                  safezone_vec_[safezone_vec_.size() - 1].left_p[0]);
          if (diffdist1 < diffdist2) {
            safezone_vec_[safezone_vec_.size() - 1].setSimpleSafeZone(end_point.x, end_point.y, start_point.x,
                                                                      start_point.y, llevel, rlevel, deld_);
            last_safezone_leftbound = end_point.y;
          }
        }
      }
    }
    last_dist_level = i;
  }
  if (sn_started && last_dist_level >= times_of_deld_) {
    end_point.x = times_of_deld_ * range_step_size_ + deadzone_;
    end_point.y = tmp_angle - dangle_; // + 0.149
    llevel = i;
    tmp_safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
    safezone_vec_.push_back(tmp_safezone);
  }

  pcl::PointXYZ self(0, 0, 0);
  if (lidar_xyz_oct_.isBlock<unibn::L2Distance<pcl::PointXYZ>>(tgt_point_, self, 0.6)) {
    tgt_point_blocked_ = 1;
    // insert a non-existent safezone
    tmp_safezone.setSimpleSafeZone(-1, 0, -1, 0, 0, 0, range_step_size_);
    safezone_vec_.insert(safezone_vec_.begin(), tmp_safezone);
  } else {
    tgt_point_blocked_ = -1;
    // if tgt_point_ is not blocked, add a safezone of tgt_point_
    int level = all_levels[num_of_dir], next_level = level, step = 0;
    if (level >= 1) {
      while (next_level == level && step < 15 && (num_of_dir - step) > 0) {
        next_level = all_levels[num_of_dir - step];
        ++step;
      }

      if (level <= next_level)
        next_level = level - 1;

      start_point.x = (0.5 + next_level) * range_step_size_;
      start_point.y = -half_fov_ + dangle_ * (num_of_dir - step + 1);
      llevel = level - 1;
      step = 0;
      next_level = level;

      while (next_level == level && step < 15 && (num_of_dir + step) < all_levels.size()) {
        next_level = all_levels[num_of_dir + step];
        ++step;
      }

      if (level <= next_level) {
        next_level = level - 1;
      }
      end_point.x = (0.5 + next_level) * range_step_size_;
      end_point.y = dangle_ * (num_of_dir + step - 1) - half_fov_;
      rlevel = level - 1;
      tmp_safezone.setSimpleSafeZone(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld_);
      safezone_vec_.insert(safezone_vec_.begin(), tmp_safezone);
    }
  }
}

// Find nearest safezone from the current position to target.
//
// @dyaw: Origin desired direction.
// @tgt_dist: Distance to target.
// @ddir_safe: Error between origin desired direction and chosen direction.
// @dist: Distance in chosen direction.
// @dir: Chosen direction (it is the same as the following one, because they are all in the self coordinate).
// @ddir_between_now_and_chosen_dir: Error between current direction and chosen direction.
//
// Return: nearest safezone number in safezone_vec, -1 if vec is null.
int P3atObstacleAvoidance::findNearestSafeZone(float dyaw, float tgt_dist, float& ddir_safe, float& dist, float& dir, float& ddir_between_now_and_chosen_dir, bool &in_safezone) {
  if (safezone_vec_.empty())
    return -1;

  in_safezone_ = false;
  int num_of_nearest = -1;
  float min_dtheta = 360, tmp_dtheta, tmp_dist, tmp_dir;
  fs_logger_ << "all " << safezone_vec_.size() << " safezone\n";

  // for view
  int imglx, imgly, imgrx, imgry;
  bool tmp_in_safezone;

  for (int i = 0; i < safezone_vec_.size(); ++i) {

    tmp_dtheta = safezone_vec_[i].getdTheta(dyaw, tgt_dist, tmp_dist, tmp_dir, tmp_in_safezone);

    fs_logger_ << "zone[" << i << "]:lp[0]=" << safezone_vec_[i].left_p[0] << ", lp[1]=" << safezone_vec_[i].left_p[1] *
        180 / M_PI << ", rp[0]=" << safezone_vec_[i].right_p[0] << ", rp[1]=" << safezone_vec_[i].right_p[1] * 180 / M_PI
        << ", safedir:" << safezone_vec_[i].safe_dir[0] * 180 / M_PI << ", " << safezone_vec_[i].safe_dir[1] * 180 / M_PI
        << ", dtheta:" << tmp_dtheta << '\n';

    if (tmp_dtheta < -7)
      continue;

    if (fabs(tmp_dtheta) < fabs(min_dtheta)) {
      // in_safezone_=tmp_in_safezone;
      min_dtheta = tmp_dtheta;
      dist = tmp_dist;
      dir = tmp_dir;
      num_of_nearest = i;
      ddir_between_now_and_chosen_dir = dir;
      if (safezone_vec_[i].safe_dir[1] > 0 && safezone_vec_[i].safe_dir[0] < 0) {
        in_safezone = true;
        in_safezone_ = true;
      } else {
        in_safezone = false;
        in_safezone_ = false;
      }
    }
  }

  ddir_safe = min_dtheta;
  fs_logger_ << "choose " << num_of_nearest << " safezone\n";
  orig_dir_ = dyaw;
  mod_dir_ = dir;

  // for view
  if (is_view_safezone_lidar_) {
    // draw nearest safezone
    imglx = kImgCol / 2 - int(safezone_vec_[num_of_nearest].left_p[0] * sin(safezone_vec_[num_of_nearest].left_p[1]) *
                              kImgRow / 5);
    imgly = kImgRow - 1 - int(safezone_vec_[num_of_nearest].left_p[0] * cos(safezone_vec_[num_of_nearest].left_p[1]) *
                              kImgRow / 5);
    imgrx = kImgCol / 2 - int(safezone_vec_[num_of_nearest].right_p[0] * sin(safezone_vec_[num_of_nearest].right_p[1]) *
                              kImgRow / 5);
    imgry = kImgRow - 1 - int(safezone_vec_[num_of_nearest].right_p[0] * cos(safezone_vec_[num_of_nearest].right_p[1]) *
                              kImgRow / 5);

    cv::line(safezone_view_, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(0, 0, 255), 4);

    // Draw original direction
    imglx = kImgCol / 2 - int(3 * sin(dyaw) * kImgRow / 5);
    imgly = kImgRow - 1 - int(3 * cos(dyaw) * kImgRow / 5);
    imgrx = kImgCol / 2;
    imgry = kImgRow - 1;
    cv::line(safezone_view_, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(255, 0, 0), 1);

    // Draw chosen direction
    imglx = kImgCol / 2 - int(3 * sin(mod_dir_) * kImgRow / 5);
    imgly = kImgRow - 1 - int(3 * cos(mod_dir_) * kImgRow / 5);
    imgrx = kImgCol / 2;
    imgry = kImgRow - 1;
    cv::line(safezone_view_, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(123, 211, 213), 3);

    // print time on screen
    char tmp[21];
    sprintf(tmp, "FPS: %05f", 1000.0 / time_intvl2_);
    cv::putText(safezone_view_, tmp, cvPoint(0, 40), cv::FONT_HERSHEY_COMPLEX, 1, cvScalar(1, 0, 0, 0));
  }

  return num_of_nearest;
}

bool P3atObstacleAvoidance::hasSuddenChange(float rang, float lang, const sensor_msgs::LaserScanConstPtr &msg, std::vector<int>&sudden_v, int& start_id_of_sc) {
  float angle = msg->angle_increment * sudden_v[start_id_of_sc] + msg->angle_min;

  while (angle < lang && start_id_of_sc < sudden_v.size()) {
    if (angle > rang)
      return true;
    angle = msg->angle_increment * sudden_v[start_id_of_sc] + msg->angle_min;
    ++start_id_of_sc;
  }

  return false;
}

void P3atObstacleAvoidance::smooth(float& vx, float& rz) {
  float max_dvx = 0.02, max_drz = 0.03;

  if (vx - last_vx_ > max_dvx)
    vx = last_vx_ + max_dvx;
  else if (vx - last_vx_ < -max_dvx)
    vx = last_vx_ - max_dvx;
  last_vx_ = vx;

  if (rz - last_rz_ > max_drz)
    rz = last_rz_ + max_drz;
  else if (rz - last_rz_ < -max_drz)
    rz = last_rz_ - max_drz;
  last_rz_ = rz;
}
