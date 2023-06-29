#ifndef AI_ROBOT_NAVIGATION_POSTRAJECTORY_H_
#define AI_ROBOT_NAVIGATION_POSTRAJECTORY_H_

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#ifdef OPENCV2
#include <opencv2/opencv.hpp>
#else
#include <opencv/cv.hpp>
#endif
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>

#include "utils/pid_controller.h"
#include "utils/pid_fuzzy.h"

// #define LINEAR (1)
constexpr int kCubic = 2;
constexpr int kMinimumJerk = 3;
constexpr int kMinimumSnap = 4;
constexpr int kNotInRange = -1;
constexpr int kBadData = -2;
constexpr int kBoundaryCondition = 1;  // 0:natural 1:clamped

// in nwu world frame
struct TrajectoryPoint {
  float x;
  float y;
  float z;
  float yaw;
  double reach_time;
};

struct TrajectoryPlanPoint {
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
  float ax;
  float ay;
  float az;
  double time_now;
};

class PostTrajectory {
public:
  PostTrajectory(const ros::NodeHandle& nh);
  void cubicTrajectoryPlanner();
  short firstDerivative(const double t, cv::Mat& dv);
  short firstDerivative(const int ti, const double t, cv::Mat& dv);
  short interPolating(const double t);
  short interPolating(const int ti, const double t, cv::Mat& s);
  void linearTrajectoryPlanner();
  void loadTrajectoryPoints();
  void minJerkTrajectoryPlanner();
  void minSnapTrajectoryPlanner();
  short secondDerivative(const double t, cv::Mat& da);
  short secondDerivative(const int ti, const double t, cv::Mat& da);
  void setMethod(int choose);
  void trajectoryPlanner(int select_flag);
  int trajectoryPlanTarget(float_t posNow[3], float_t velNow[3], double_t accCommand[3], double_t dt, double_t time);
  void trajectoryStart(int flag);

public:
  bool bad_data;
  bool change_frame_flag;
  bool start_trajectory;
  bool write_points;
  int choose_flag;
  int dimension;
  int record_fram_index;
  double_t start_velocity[3];
  double_t start_acc[3];
  double_t trajectory_start_time;
  float_t linear_inter_acc; // linear interplation
  float_t yaw_target;
  std::vector<TrajectoryPoint> trajectory_sequence;
  std::vector<TrajectoryPlanPoint> trajectory_plan_sequence;

  // cubic spline calculate matrix and following matrixs support 7 order spline
  cv::Mat ak;
  cv::Mat bk;
  cv::Mat ck;
  cv::Mat dk;
  cv::Mat ek;
  cv::Mat fk;
  cv::Mat gk;
  cv::Mat hk;
  cv::Mat pts;
  cv::Mat blend_time_i;
  cv::Mat blend_time_i_j;
  cv::Mat slopes;
  cv::Mat sigsn_s;

  Eigen::Quaterniond target_q;
  Eigen::Quaterniond target_q_from;
  Eigen::Quaterniond target_q_to;

  FuzzyPid* fuzzy_pid_trajector;
  PidController* pid_trajector;
  TrajectoryPlanPoint target_point_now;
  TrajectoryPoint trajectory_start_point;

private:
  void setup();
  float sign(float x);

private:
  ros::NodeHandle nh_;
};

#endif  // AI_ROBOT_NAVIGATION_POSTTRAJECTORY_H_
