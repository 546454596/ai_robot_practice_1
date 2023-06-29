#include <cmath>
#include <fstream>
#include <string>

#include "pos_trajectory.h"

constexpr int kLinear = 1;

// extern std::string trajectoryPIDPath;
std::string trajectory_points_path;

float trajectory_p = 1.0;
float trajectory_i = 0.0;
float trajectory_d = 2.0;
float trajectory_imax = 0.3;

// std::fstream trajectory_points_file(trajectory_points_path.c_str());
std::fstream trajectory_points_file;

PostTrajectory::PostTrajectory(const ros::NodeHandle& nh)
    : nh_(nh),
      bad_data(false),
      dimension(3),
      write_points(false),
      start_trajectory(false),
      linear_inter_acc(0.5),
      record_fram_index(-1),
      change_frame_flag(false),
      choose_flag(kCubic) {
  setup();
}

void PostTrajectory::setup() {
  ROS_INFO("[TRAJECTORY] Trajectory Setup...");

  nh_.getParam("trajectoryP", trajectory_p);
  nh_.getParam("trajectoryI", trajectory_i);
  nh_.getParam("trajectoryD", trajectory_d);
  nh_.getParam("trajectoryImax", trajectory_imax);
  nh_.getParam("trajectory_points_path", trajectory_points_path);

  trajectory_points_file.open(trajectory_points_path.c_str());

  loadTrajectoryPoints();

  pid_trajector = new PidController(trajectory_p, trajectory_i, trajectory_d, trajectory_imax);
  fuzzy_pid_trajector = new FuzzyPid(trajectory_p, trajectory_i, trajectory_d, trajectory_imax);

  for (int i = 0; i < dimension; ++i)
    start_velocity[i] = 0;

  ROS_INFO("[TRAJECTORY] Trajectory Setup done.");
}

float PostTrajectory::sign(float x) {
  if (x > 0.0)
    return 1.0f;
  else {
    if (x < 0.0)
      return -1.0f;
    else
      return 0.0f;
  }
}

void PostTrajectory::loadTrajectoryPoints() {
  trajectory_sequence.clear();

  TrajectoryPoint tmp_point;

  while (trajectory_points_file >> tmp_point.x >> tmp_point.y >>
         tmp_point.z >> tmp_point.yaw >> tmp_point.reach_time) {
    trajectory_sequence.push_back(tmp_point);
  }
}

void PostTrajectory::trajectoryPlanner(int select_flag) {
  switch (select_flag) {
    case kLinear:
      linearTrajectoryPlanner();
      break;
    case kCubic:
      cubicTrajectoryPlanner();
      break;
    case kMinimumJerk:
      minJerkTrajectoryPlanner();
      break;
    case kMinimumSnap:
      minSnapTrajectoryPlanner();
      break;
    default:
      break;
  }
}

void PostTrajectory::linearTrajectoryPlanner() {
  int n = trajectory_sequence.size();

  if (!n) {
    bad_data = true;
    ROS_INFO("[TRAJECTORY]PostTrajectory_linear: size of time vector is zero.");
    return;
  }

  if (n <= 2) {
    bad_data = true;
    ROS_INFO("[TRAJECTORY]PostTrajectory_linear: need at least 2 points to produce a spline.");
    return;
  }

  float *delta_t = new float[n - 1];
  pts = cv::Mat(n, dimension, CV_32FC1, cv::Scalar::all(0));

  for (int i = 0; i < n; ++i) {
    pts.at<float>(i, 0) = trajectory_sequence[i].x;
    pts.at<float>(i, 1) = trajectory_sequence[i].y;
    pts.at<float>(i, 2) = trajectory_sequence[i].z;
  }

  for (int i = 0; i < n - 1; ++i) {
    delta_t[i] = trajectory_sequence[i + 1].reach_time - trajectory_sequence[i].reach_time;
    if (delta_t[i] <= 0) {
      bad_data = true;
      ROS_INFO("[TRAJECTORY]PostTrajectory_linear:  time between input points is less than zero");
      return;
    }
  }

  blend_time_i = cv::Mat(n, dimension, CV_32FC1, cv::Scalar::all(0));
  blend_time_i_j = cv::Mat(n, dimension, CV_32FC1, cv::Scalar::all(0));
  slopes = cv::Mat(n, dimension, CV_32FC1, cv::Scalar::all(0));
  sigsn_s = cv::Mat(n, dimension, CV_32FC1, cv::Scalar::all(0));

  // Calculate slopes and blend_time_i
  for (int i = 0; i < dimension; i++) {
    sigsn_s.at<float>(0, i) = sign(pts.at<float>(1, i) - pts.at<float>(0, i));
    blend_time_i.at<float>(0, i) = delta_t[0] - sqrt(
        delta_t[0] * delta_t[0] - (2 * fabs(pts.at<float>(1, i) - pts.at<float>(0, i)) / linear_inter_acc));

    slopes.at<float>(0, i) = (pts.at<float>(1, i) - pts.at<float>(0, i)) / (delta_t[0] - 0.5f *
        blend_time_i.at<float>(0, i));
  }

  for (int j = 1; j < n - 1; ++j) {
    for (int i = 0; i < dimension; i++) {
      sigsn_s.at<float>(j, i) = sign(slopes.at<float>(j + 1, i) - slopes.at<float>(j, i));
      slopes.at<float>(j, i) = (pts.at<float>(j + 1, i) - pts.at<float>(j, i)) / delta_t[j];
      blend_time_i.at<float>(0, i) = fabs((slopes.at<float>(j, i) - slopes.at<float>(j - 1, i)) / linear_inter_acc);
    }
  }

  for (int i = 0; i < dimension; i++) {
    sigsn_s.at<float>(n - 1, i) = sign(pts.at<float>(n - 1, i) - pts.at<float>(n - 2, i));
    blend_time_i.at<float>(n - 1, i) = delta_t[n - 1] - sqrt(delta_t[n - 1] * delta_t[n - 1] - (2 * fabs(pts.at<float>(n, i) - pts.at<float>(n - 2, i)) / linear_inter_acc));

    slopes.at<float>(n - 1, i) = (pts.at<float>(n, i) - pts.at<float>(n - 1, i)) / (delta_t[n - 1] - 0.5f * blend_time_i.at<float>(n - 1, i));
  }

  // Calculate blend_time_ij
  for (int i = 0; i < dimension; i++)
    blend_time_i_j.at<float>(0, i) = delta_t[0] - blend_time_i.at<float>(0, i) - 0.5 * blend_time_i.at<float>(1, i);

  for (int j = 1; j < n - 1; ++j)
    for (int i = 0; i < dimension; i++)
      blend_time_i_j.at<float>(j, i) = delta_t[j] - 0.5 * (blend_time_i.at<float>(j, i) + blend_time_i.at<float>(j + 1, i));

  for (int i = 0; i < dimension; i++)
    blend_time_i_j.at<float>(n - 1, i) = delta_t[n - 1] - blend_time_i.at<float>(n - 1, i) - 0.5 * blend_time_i.at<float>(n - 2, i);
 
  delete[] delta_t;
}

void PostTrajectory::cubicTrajectoryPlanner() {
  pts.release();
  ak.release();
  bk.release();
  ck.release();
  dk.release();

  int n = trajectory_sequence.size();

  if (!n) {
    bad_data = true;
    ROS_INFO("[TRAJECTORY]PostTrajectory: size of time vector is zero.");
    return;
  }

  if (n <= 4) {
    bad_data = true;
    ROS_INFO("[TRAJECTORY]PostTrajectory: need at least 4 points to produce a spline.");
    return;
  }

  float *delta = new float[n - 1];
  float *beta = new float[n - 1];
  pts = cv::Mat(n, dimension, CV_32FC1, cv::Scalar::all(0));
  for (int i = 0; i < n; ++i) {
    pts.at<float>(i, 0) = trajectory_sequence[i].x;
    pts.at<float>(i, 1) = trajectory_sequence[i].y;
    pts.at<float>(i, 2) = trajectory_sequence[i].z;
  }

  for (int i = 0; i < n - 1; ++i) {
    delta[i] = trajectory_sequence[i + 1].reach_time - trajectory_sequence[i].reach_time;
    if (delta[i] <= 0) {
      bad_data = true;
      ROS_INFO("[TRAJECTORY]PostTrajectory:  time between input points is zero");
      return;
    }
    beta[i] = 1 / delta[i];
  }

  if (kBoundaryCondition == 0) { // natural boundary
    if (start_trajectory) {
      start_acc[0] = target_point_now.ax;
      start_acc[1] = target_point_now.ay;
      start_acc[2] = target_point_now.az;
    }

    cv::Mat cv_a(n - 2, n - 2, CV_32FC1, cv::Scalar::all(0.0));
    cv_a.at<float>(0, 0) = 2 * (delta[0] + delta[1]);
    cv_a.at<float>(0, 1) = delta[1];

    for (int j = 1; j < n - 2; ++j) {
      cv_a.at<float>(j, j - 1) = delta[j];
      cv_a.at<float>(j, j) = 2 * (delta[j] + delta[j + 1]);
      if ((j + 1) < cv_a.cols) 
        cv_a.at<float>(j, j + 1) = delta[j + 1];
    }

    cv::Mat cv_c(n - 2, n, CV_32FC1, cv::Scalar::all(0));

    for (int k = 0; k < n - 2; ++k) {
      cv_c.at<float>(k, k) = beta[k];
      cv_c.at<float>(k, k + 1) = -(beta[k] + beta[k + 1]);
      cv_c.at<float>(k, k + 2) = beta[k + 1];
    }

    cv::Mat cv_6ai_c = 6 * cv_a.inv() * cv_c;  // eq 5.58a Angeles
    // second spline derivative at sampling points
    cv::Mat dd_s(n, 1, CV_32FC1, cv::Scalar::all(0));  
    // second der is 0 on first and last point of natural splines.
    dd_s.at<float>(0, 0) = dd_s.at<float>(n - 1, 0) = 0;  

    ak = cv::Mat(n - 1, dimension, CV_32FC1, cv::Scalar::all(0));
    bk = cv::Mat(n - 1, dimension, CV_32FC1, cv::Scalar::all(0));
    ck = cv::Mat(n - 1, dimension, CV_32FC1, cv::Scalar::all(0));
    dk = cv::Mat(n - 1, dimension, CV_32FC1, cv::Scalar::all(0));

    cv::Rect roi1(0, 1, 1, n - 2);

    for (int i = 0; i < dimension; i++) {
      cv::Rect roi2(i, 0, 1, n);
      dd_s(roi1) = cv_6ai_c * (pts(roi2));
      if (start_trajectory) 
        dd_s.at<float>(0, 0) = start_acc[i];

      for (int j = 0; j < n - 1; j++) {
        // eq 5.55a - 5.55d Angeles
        ak.at<float>(j, i) = 1 / (6.0 * delta[j]) * (dd_s.at<float>(j + 1, 0) - dd_s.at<float>(j, 0));
        bk.at<float>(j, i) = 1 / 2.0 * dd_s.at<float>(j, 0);
        ck.at<float>(j, i) = (pts.at<float>(j + 1, i) - pts.at<float>(j, i)) / delta[j] - 1 / 6.0 * delta[j] * (dd_s.at<float>(j + 1, 0) + 2 * dd_s.at<float>(j, 0));
        dk.at<float>(j, i) = pts.at<float>(j, i);
      }
    }
  } else { // clamped boundary,set velocity eq to start_velocity
    if (start_trajectory) {
      start_velocity[0] = target_point_now.vx;
      start_velocity[1] = target_point_now.vy;
      start_velocity[2] = target_point_now.vz;
    } else {
      for (int k = 0; k < dimension; ++k) 
        start_velocity[k] = 0;
    }

    cv::Mat cv_a(n, n, CV_32FC1, cv::Scalar::all(0.0));
    cv_a.at<float>(0, 0) = 2 * delta[0];
    cv_a.at<float>(0, 1) = delta[0];
    cv_a.at<float>(n - 1, n - 2) = delta[n - 2];
    cv_a.at<float>(n - 1, n - 1) = 2 * delta[n - 2];

    for (int j = 1; j < n - 1; ++j) {
      cv_a.at<float>(j, j - 1) = delta[j - 1];
      cv_a.at<float>(j, j) = 2 * (delta[j - 1] + delta[j]);
      if ((j + 1) < cv_a.cols) 
        cv_a.at<float>(j, j + 1) = delta[j];
    }

    cv::Mat cv_c(n, n, CV_32FC1, cv::Scalar::all(0));
    cv_c.at<float>(0, 0) = -beta[0];
    cv_c.at<float>(0, 1) = beta[0];
    cv_c.at<float>(n - 1, n - 2) = beta[n - 2];
    cv_c.at<float>(n - 1, n - 1) = -beta[n - 2];

    for (int k = 1; k < n - 1; ++k) {
      cv_c.at<float>(k, k - 1) = beta[k - 1];
      cv_c.at<float>(k, k) = -(beta[k - 1] + beta[k]);
      cv_c.at<float>(k, k + 1) = beta[k];
    }

    cv::Mat cv_6ai_c = 6 * cv_a.inv() * cv_c;  // eq 5.58a Angeles
    // second spline derivative at sampling points
    cv::Mat dd_s(n, 1, CV_32FC1, cv::Scalar::all(0));  
    ak = cv::Mat(n - 1, dimension, CV_32FC1, cv::Scalar::all(0));
    bk = cv::Mat(n - 1, dimension, CV_32FC1, cv::Scalar::all(0));
    ck = cv::Mat(n - 1, dimension, CV_32FC1, cv::Scalar::all(0));
    dk = cv::Mat(n - 1, dimension, CV_32FC1, cv::Scalar::all(0));

    for (int i = 0; i < dimension; i++) {
      pts.at<float>(i, 0) -= delta[0] * start_velocity[i];
      // add the start velocity
      pts.at<float>(i, n - 1) -= delta[n - 2] * start_velocity[i];  
      cv::Rect roi2(i, 0, 1, n);

      dd_s = cv_6ai_c * (pts(roi2));
      for (int j = 0; j < n - 1; j++) {
        // eq 5.55a - 5.55d Angeles
        ak.at<float>(j, i) = 1 / (6.0 * delta[j]) * (dd_s.at<float>(j + 1, 0) - dd_s.at<float>(j, 0));
        bk.at<float>(j, i) = 1 / 2.0 * dd_s.at<float>(j, 0);
        ck.at<float>(j, i) = (pts.at<float>(j + 1, i) - pts.at<float>(j, i)) / delta[j] -
                             1 / 6.0 * delta[j] * (dd_s.at<float>(j + 1, 0) + 2 * dd_s.at<float>(j, 0));
        dk.at<float>(j, i) = pts.at<float>(j, i);
      }
    }
  }

  bad_data = false;
  delete[] delta;
  delete[] beta;
}

void PostTrajectory::minJerkTrajectoryPlanner() {
  pts.release();
  ak.release();
  bk.release();
  ck.release();
  dk.release();
  ek.release();
  fk.release();

  int n = trajectory_sequence.size();

  if (!n) {
    bad_data = true;
    ROS_INFO("[TRAJECTORY] PostTrajectory: size of time vector is zero.");
  } else if (n <= 6) {
    bad_data = true;
    ROS_INFO("[TRAJECTORY] PostTrajectory: need at least 6 points to produce a spline.");
  }

  return;
}

void PostTrajectory::minSnapTrajectoryPlanner() {
  pts.release();
  ak.release();
  bk.release();
  ck.release();
  dk.release();
  ek.release();
  fk.release();
  gk.release();
  hk.release();
  int n = trajectory_sequence.size();
  if (!n) {
    bad_data = true;
    ROS_INFO("[TRAJECTORY]PostTrajectory: size of time vector is zero.");
    return;
  }
  if (n <= 8) {
    bad_data = true;
    ROS_INFO("[TRAJECTORY]PostTrajectory: need at least 8 points to produce a spline.");
    return;
  }
}

short PostTrajectory::interPolating(const double t) {
  if (bad_data) {
    ROS_INFO("[TRAJECTORY] PostTrajectory::interpolating: data is not good. Problems occur in constructor.");
    return kBadData;
  }

  cv::Mat s, dv, da;

  for (int i = 0; i < trajectory_sequence.size() - 1; i++) {
    cv::Rect roi(0, i, dimension, 1);
    double reachtime_i = trajectory_sequence[i].reach_time;

    if ((t >= reachtime_i && (t < trajectory_sequence[i + 1].reach_time))) {
      if (record_fram_index != i) {
        float yaw = trajectory_sequence[i].yaw;
        target_q_from.w() = cos(yaw * .5f);
        target_q_from.x() = 0;
        target_q_from.y() = 0;
        target_q_from.z() = sin(yaw * .5f);

        yaw = trajectory_sequence[i + 1].yaw;
        target_q_to.w() = cos(yaw * .5f);
        target_q_to.x() = 0;
        target_q_to.y() = 0;
        target_q_to.z() = sin(yaw * .5f);

        record_fram_index = i;
        change_frame_flag = true;
      }

      interPolating(i, t, s);
      firstDerivative(i, t, dv);
      secondDerivative(i, t, da);
      target_point_now.x = s.at<float>(0, 0);
      target_point_now.y = s.at<float>(0, 1);
      target_point_now.z = s.at<float>(0, 2);
      target_point_now.vx = dv.at<float>(0, 0);
      target_point_now.vy = dv.at<float>(0, 1);
      target_point_now.vz = dv.at<float>(0, 2);
      target_point_now.ax = da.at<float>(0, 0);
      target_point_now.ay = da.at<float>(0, 1);
      target_point_now.az = da.at<float>(0, 2);
      target_point_now.time_now = t;
    }
    return 0;
  }

  ROS_INFO("[TRAJECTORY] PostTrajectory::interpolating: t is out of range.");

  return kNotInRange;
}

short PostTrajectory::interPolating(const int ti, const double t, cv::Mat& s) {
  if (choose_flag == kCubic) {
    double reachtime_i = trajectory_sequence[ti].reach_time;
    target_q = target_q_from.slerp((t - reachtime_i) / (trajectory_sequence[ti + 1].reach_time - reachtime_i), target_q_to);
    yaw_target = 2 * atan2(target_q.z(), target_q.w());
    cv::Rect roi(0, ti, dimension, 1);
    s = ak(roi) * pow(t - reachtime_i, 3) + bk(roi) * pow(t - reachtime_i, 2) + ck(roi) * (t - reachtime_i) + dk(roi);
  } else if (choose_flag == kLinear) {
    s = cv::Mat(1, dimension, CV_32FC1, cv::Scalar::all(0));
    double dt = t - trajectory_sequence[ti].reach_time;
    float ch1 = 0.5;

    if (ti == 0)
      ch1 = 1;

    if (ti == 0) {
      for (int j = 0; j < dimension; ++j) {
        if (dt <= ch1 * blend_time_i.at<float>(ti, j)) {
          s.at<float>(0, j) = pts.at<float>(ti, j) + 0.5 * sigsn_s.at<float>(ti, j) * linear_inter_acc * pow(ch1 * blend_time_i.at<float>(ti, j) + dt, 2);
        } else {
          if (dt < blend_time_i_j.at<float>(ti, j) + ch1 * blend_time_i.at<float>(ti, j))
            s.at<float>(0, j) = pts.at<float>(ti, j) + slopes.at<float>(ti, j) * (dt - 0.5 * blend_time_i.at<float>(ti, j));
          else
            s.at<float>(0, j) = pts.at<float>(ti, j) + slopes.at<float>(ti, j) *
                                (dt - 0.5 * blend_time_i.at<float>(ti, j)) +
                                0.5 * sigsn_s.at<float>(ti + 1, j) * linear_inter_acc *
                                pow(dt - ch1 * blend_time_i.at<float>(ti, j) -
                                blend_time_i_j.at<float>(ti, j), 2);
        }
      }
    } else {
      for (int j = 0; j < dimension; ++j) {
        if (dt <= ch1 * blend_time_i.at<float>(ti, j))
          s.at<float>(0, j) = pts.at<float>(ti, j) + slopes.at<float>(ti - 1, j) * dt +
                              0.5 * sigsn_s.at<float>(ti, j) * linear_inter_acc *
                              pow(ch1 * blend_time_i.at<float>(ti, j) + dt, 2);
        else {
          if (dt < blend_time_i_j.at<float>(ti, j) + ch1 * blend_time_i.at<float>(ti, j))
            s.at<float>(0, j) = pts.at<float>(ti, j) + slopes.at<float>(ti, j) *
                                (dt - 0.5 * blend_time_i.at<float>(ti, j));
          else
            s.at<float>(0, j) = pts.at<float>(ti, j) + slopes.at<float>(ti, j) *
                                (dt - 0.5 * blend_time_i.at<float>(ti, j)) +
                                0.5 * sigsn_s.at<float>(ti + 1, j) * linear_inter_acc *
                                pow(dt - ch1 * blend_time_i.at<float>(ti, j) -
                                blend_time_i_j.at<float>(ti, j), 2);
        }
      }
    }
  }

  return 0;
}

short PostTrajectory::firstDerivative(const double t, cv::Mat& dv) {
  if (bad_data) {
    ROS_INFO("[TRAJECTORY] PostTrajectory::first_derivative: data is not good. Problems occur in constructor.");
    return kBadData;
  }

  for (int i = 0; i < trajectory_sequence.size() - 1; i++)
    return firstDerivative(i, t, dv);

  ROS_INFO("[TRAJECTORY]PostTrajectory::first_derivative: t not in range.");

  return kNotInRange;
}

short PostTrajectory::firstDerivative(const int ti, const double t, cv::Mat& dv) {
  if (choose_flag == kCubic) {
    cv::Rect roi(0, ti, dimension, 1);
    double reachtime_i = trajectory_sequence[ti].reach_time;
    dv = 3 * ak(roi) * pow(t - reachtime_i, 2) + 2 * bk(roi) * (t - reachtime_i) + ck(roi);
  } else if (choose_flag == kLinear) {
    dv = cv::Mat(1, dimension, CV_32FC1, cv::Scalar::all(0));
    double dt = t - trajectory_sequence[ti].reach_time;
    float ch1 = 0.5;

    if (ti == 0) {
      ch1 = 1;
    }

    for (int j = 0; j < dimension; ++j) {
      if (dt <= ch1 * blend_time_i.at<float>(ti, j))
        dv.at<float>(0, j) = slopes.at<float>(ti, j) - sigsn_s.at<float>(ti, j) * 
                             linear_inter_acc * (ch1 * blend_time_i.at<float>(ti, j) - dt);
      else {
        if (dt < blend_time_i_j.at<float>(ti, j) + ch1 * blend_time_i.at<float>(ti, j))
          dv.at<float>(0, j) = slopes.at<float>(ti, j);
        else
          dv.at<float>(0, j) = slopes.at<float>(ti, j) + sigsn_s.at<float>(ti + 1, j) * 
                               linear_inter_acc * (dt - blend_time_i_j.at<float>(ti, j) -
                               ch1 * blend_time_i.at<float>(ti, j));
      }
    }
  }

  return 0;
}

short PostTrajectory::secondDerivative(const double t, cv::Mat& da) {
  if (bad_data) {
    ROS_INFO("[TRAJECTORY] Spl_kCubic::second_derivative: data is not good. Problems occur in constructor.");
    return kBadData;
  }

  for (int i = 0; i < trajectory_sequence.size() - 1; i++)
    return secondDerivative(i, t, da);

  ROS_INFO("[TRAJECTORY] PostTrajectory::second_derivative: t not in range.");

  return kNotInRange;
}

short PostTrajectory::secondDerivative(const int ti, const double t, cv::Mat& da) {
  if (choose_flag == kCubic) {
    cv::Rect roi(0, ti, dimension, 1);
    double reachtime_i = trajectory_sequence[ti].reach_time;
    da = 6 * ak(roi) * (t - reachtime_i) + 2 * bk(roi);
  } else if (choose_flag == kLinear) {
    da = cv::Mat(1, dimension, CV_32FC1, cv::Scalar::all(0));
    double dt = t - trajectory_sequence[ti].reach_time;
    float ch1 = 0.5;

    if (ti == 0) {
      ch1 = 1;
    }

    for (int i = 0; i < dimension; ++i) {
      if (dt <= ch1 * blend_time_i.at<float>(ti, i)) {
        da.at<float>(0, i) = sigsn_s.at<float>(ti, i) * linear_inter_acc;
      } else {
        if (dt < blend_time_i_j.at<float>(ti, i) + ch1 * blend_time_i.at<float>(ti, i))
          da.at<float>(0, i) = 0;
        else
          da.at<float>(0, i) = sigsn_s.at<float>(ti + 1, i) * linear_inter_acc;
      }
    }
  }

  return 0;
}

int PostTrajectory::trajectoryPlanTarget(float_t pos_now[3], float_t vel_now[3],
                                         double_t acc_csommand[3], double_t dt,
                                         double_t time) {
  float_t pos_target[3], vel_target[3], acc_target[3];
  int flag = interPolating(time - trajectory_start_time);

  if (flag < 0)
    return flag;
 
  pos_target[0] = target_point_now.x + trajectory_start_point.x;
  pos_target[1] = target_point_now.y + trajectory_start_point.y;
  pos_target[2] = target_point_now.z;

  vel_target[0] = target_point_now.vx;
  vel_target[1] = target_point_now.vy;
  vel_target[2] = target_point_now.vz;

  acc_target[0] = target_point_now.ax;
  acc_target[1] = target_point_now.ay;
  acc_target[2] = target_point_now.az;

  if (start_trajectory) {
    if (bad_data) {
      for (int i = 0; i < 3; ++i)
        acc_csommand[i] = 0;
      return 0;
    }

    for (int i = 0; i < 3; ++i)
      acc_csommand[i] = pid_trajector->getKp() * (pos_target[i] - pos_now[i]) + pid_trajector->getKd() * (vel_target[i] - vel_now[i]) + acc_target[i];

    return 1;
  } else {
    for (int i = 0; i < 3; ++i)
      acc_csommand[i] = 0;

    ROS_INFO("[TRAJECTORY]Trajectory does not start!");

    return 0;
  }
}

void PostTrajectory::trajectoryStart(int flag) {
  if (flag == 1) {
    start_trajectory = true;
    trajectory_start_time = ros::Time().now().toSec();
  } else if (flag == 0)
    start_trajectory = false;
}

void PostTrajectory::setMethod(int choose) { 
  choose_flag = choose; 
}
