#include "pos_control.h"

#include <cmath>
#include <fstream>
#include <string>

// extern std::string trajectory_points_path;
// std::fstream trajectory_points_file_hover(trajectory_points_path.c_str());
// std::fstream trajectory_points_file_hover;
 
int kControllerType = 0;
constexpr int kVisionHoverSwitch = 0;
float kGravityMss = 9.80665;
float kMaxAngle = 1.0;
float kPoscontrolAccelXY = 2.0;
float kPoscontrolSpeed = 2.0;
float kPoscontrolAccelXYMax = 9.80;
float kTargetDistanceMax = 1.0;
float kSonarLowerLpThreshold = 0.2;
float kSonarUpperLpthreshold = 0.5;
float kFeedForwardParam = 0.5;

// control parameters
float kHoverPosP = 0.5;
float kHoverPosI = 0.1;
float kHoverPosD = 0.0;
float kHoverPosImax = 0.3;
float kHoverVelP = 1.5;
float kHoverVelI = 0.0;
float kHoverVelD = 0.02;
float kHoverVelImax = 0.5;
float kHoverVelWo = 1.5;
float kHoverVelWc = 0.0;
float kHoverVelB0 = 0.02;
float kAltitudeP = 1.0;
float kAltitudeI = 0.0;
float kAltitudeD = 0.1;
float kYawP = 1.0;
float kYawI = 0.0;
float kYawD = 0.1;

constexpr float kMaxLinearSpeed = 0.5;
constexpr float kMaxAngularSpeed = M_PI / 6;

std::string pos_pid_path;
std::string vel_pid_path;
std::string trajectory_pid_path;

PosControl::PosControl(const ros::NodeHandle& nh)
    : nh_(nh),
      thread_quit_(false),
      trajectory_(nh_),
      is_control_outside_(false),
      is_set_vel_(false),
      is_set_point_(false),
      is_set_trajectory_(false),
      is_vision_hover_(false),
      set_hover_(false),
      max_linear_speed_(kMaxLinearSpeed),
      max_angular_speed_(kMaxAngularSpeed),
      roll_target(0),
      pitch_target(0),
      pos_record_count(0),
      pos_x_sum(0),
      pos_y_sum(0),
      pos_z_sum(0),
      reach_point_flag(false),
      cotrol_with_vicon(true),
      move_mode(kMoveByVelocity),
      latch_for_vel(false),
      drone(nh) {
  setup();
}

void PosControl::setup() {
  ROS_INFO("[PoseControl] Hover setup ....");

  vision_hover_pub_ = nh_.advertise<std_msgs::Bool>("/visionhover/hover", 1);
  pose_ekf_sub_ = nh_.subscribe("/est_pose", 10, &PosControl::posEkfCallback, this);
  vel_ekf_sub_ = nh_.subscribe("/est_vel", 10, &PosControl::velEkfCallback, this);
  speed_sub_ = nh_.subscribe("/speed", 1, &PosControl::speedCallback, this);

  nh_.param("gravity", kGravityMss, 9.80665f);
  // nh_.getParam("gravity", kGravityMss);

  nh_.getParam("max_angle", kMaxAngle);
  nh_.getParam("poscontrol_accel_xy", kPoscontrolAccelXY);
  nh_.getParam("poscontrol_speed", kPoscontrolSpeed);
  nh_.getParam("poscontrol_accel_xy_max", kPoscontrolAccelXYMax);
  nh_.getParam("max_target_distance", kTargetDistanceMax);
  nh_.getParam("sonar_lower_lp_threshold", kSonarLowerLpThreshold);
  nh_.getParam("sonar_upper_lp_threshold", kSonarUpperLpthreshold);
  nh_.getParam("feedforward_para", kFeedForwardParam);
  nh_.getParam("mave_mode", move_mode);
  nh_.getParam("posPIDPath", pos_pid_path);
  nh_.getParam("velPIDPath", vel_pid_path);
  // nh_.getParam("trajectoryPIDPath", trajectory_points_path);
  nh_.getParam("controllerType", kControllerType);
  nh_.getParam("yawP", kYawP);
  nh_.getParam("yawD", kYawI);
  nh_.getParam("yawI", kYawD);
  nh_.getParam("altitudeP", kAltitudeP);
  nh_.getParam("altitudeD", kAltitudeI);
  nh_.getParam("altitudeI", kAltitudeD);
  nh_.getParam("hoverPosP", kHoverPosP);
  nh_.getParam("hoverPosI", kHoverPosI);
  nh_.getParam("hoverPosD", kHoverPosD);
  nh_.getParam("hoverPosImax", kHoverPosImax);
  nh_.getParam("hoverVelP", kHoverVelP);
  nh_.getParam("hoverVelI", kHoverVelI);
  nh_.getParam("hoverVelD", kHoverVelD);
  nh_.getParam("hoverVelImax", kHoverVelImax);
  nh_.getParam("hoverVelWo", kHoverVelWo);
  nh_.getParam("hoverVelWc", kHoverVelWc);
  nh_.getParam("hoverVelB0", kHoverVelB0);
  // vijay's
  nh_.getParam("Kpos", k_pos);
  nh_.getParam("Kvel", k_vel);
  nh_.getParam("max_linear_speed", max_linear_speed_);
  nh_.getParam("max_angular_speed", max_angular_speed_);

  // PID controller
  pid_x = new PidController(kHoverPosP, kHoverPosI, kHoverPosD, kHoverPosImax);
  pid_vx = new PidController(kHoverVelP, kHoverVelI, kHoverVelD, kHoverVelImax); 
  pid_y = new PidController(kHoverPosP, kHoverPosI, kHoverPosD, kHoverPosImax);
  pid_vy = new PidController(kHoverVelP, kHoverVelI, kHoverVelD, kHoverVelImax); 
  pid_yaw = new PidController(kYawP, kYawI, kYawD, 0.2);
  pid_z = new PidController(kAltitudeP, kAltitudeI, kAltitudeD, 0.3);

  // Fuzzy PID controller
  fuzzy_pid_x = new FuzzyPid(kHoverPosP, kHoverPosI, kHoverPosD, kHoverPosImax);
  fuzzy_pid_y = new FuzzyPid(kHoverPosP, kHoverPosI, kHoverPosD, kHoverPosImax);
  fuzzy_pid_vx = new FuzzyPid(kHoverVelP, kHoverVelI, kHoverVelD, kHoverVelImax);
  fuzzy_pid_vy = new FuzzyPid(kHoverVelP, kHoverVelI, kHoverVelD, kHoverVelImax);

  // ADRC controller
  adrc_vx = new Adrc(kHoverVelWo, kHoverVelWc, kHoverVelB0, 1.0 / control_rate);
  adrc_vy = new Adrc(kHoverVelWo, kHoverVelWc, kHoverVelB0, 1.0 / control_rate);

  pid_time = ros::Time().now();
  check_outside_control = ros::Time().now();
  setpoint_loiter_time = ros::Time().now();
  set_point_pos.reserve(100);
  yaw_target = 0;

  for (size_t i = 0; i < 3; ++i) {
    acc[i] = 0;
    vel[i] = 0;
    pos_err[i] = 0;
    vel_err[i] = 0;
    acc_err[i] = 0;
    pos[i] = 0;
    target_pos[i] = 0;
    vel_last[i] = 0;
    hover_pos[i] = 0;
    trajector_vel[i] = 0;
    trajector_acc[i] = 0;
  }

  for (size_t i = 0; i < 20; ++i) {
    pos_x_record[i] = 0, pos_y_record[i] = 0;
  }

  ROS_INFO("[PoseControl] Hover setup done!");
}

// The origin data is in NWU frame
void PosControl::posEkfCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  sensor_local_position.pose = msg->pose;
  sensor_local_position.header.stamp = msg->header.stamp;

  qtoEuler();  // in NWU frame
  getRotation();

  pos[0] = msg->pose.position.x;
  pos[1] = msg->pose.position.y;  // convert to NWU frame
  pos[2] = msg->pose.position.z;

  pos_x_sum -= pos_x_record[pos_record_count];
  pos_y_sum -= pos_y_record[pos_record_count];
  pos_z_sum -= pos_z_record[pos_record_count];
  pos_x_record[pos_record_count] = pos[0];
  pos_y_record[pos_record_count] = pos[1];
  pos_z_record[pos_record_count] = pos[2];
  pos_record_count++;
  if (pos_record_count >= kMaxCountPosition) pos_record_count = 0;
  pos_x_sum += pos[0];
  pos_y_sum += pos[1];
  pos_z_sum += pos[2];

}

void PosControl::velEkfCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  vel_ekf[0] = msg->twist.linear.x;
  vel_ekf[1] = msg->twist.linear.y;
  vel_ekf[2] = msg->twist.linear.z;
}

void PosControl::speedCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  nh_.getParam("max_linear_speed", max_linear_speed_);
  nh_.getParam("max_angular_speed", max_angular_speed_);
}

float PosControl::safeSqrt(float v) {
  float ret = sqrtf(v);

  if (isnan(ret))
    return 0.0;

  return ret;
}

// Constrain a value
float PosControl::constrainFloat(float amt, float low, float high) {
  if (isnan(amt))
    return (low + high) * 0.5f;

  return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

float PosControl::fastAtan(float v) {
  float v2 = v * v;
  return (v * (1.6867629106f + v2 * 0.4378497304f) / (1.6867633134f + v2));
}

float PosControl::trapezoidalIntegral(float dt, float v_last, float v_now) {
  return (v_last + v_now) / 2 * dt;
}

// posToRateXY - horizontal position error to velocity controller
//   converts position (_pos_target) to target velocity (_vel_target)
//   when use_desired_rate is set to true:
//         desired velocity (_vel_desired) is combined into final target
//         velocity and velocity due to position error is reduce to a maximum
//         of 1m/s
void PosControl::posToRateXY(float dt, float ekf_nav_vel_gain_scaler, double vel_target[3]) {
  float linear_distance; // the distance we swap between linear and sqrt velocity response
  float kP = ekf_nav_vel_gain_scaler * pid_x->getKp(); // scale gains to compensate for noisy optical flow measurement in the EKF
  float px_i, py_i;

  // Avoid divide by zero
  if (kP <= 0.0f) {
    vel_target[0] = 0.0f;
    vel_target[1] = 0.0f;
  } else {
    // Calculate distance error
    pos_err[0] = target_pos[0] - pos[0];
    pos_err[1] = target_pos[1] - pos[1];

    // Constrain target position to within reasonable distance of current location
    float distance_to_target = sqrtf(pos_err[0] * pos_err[0] + pos_err[1] * pos_err[1]);

    if (distance_to_target > kTargetDistanceMax && distance_to_target > 0.0f) {
      target_pos[0] = pos[0] + kTargetDistanceMax * pos_err[0] / distance_to_target;
      target_pos[1] = pos[1] + kTargetDistanceMax * pos_err[1] / distance_to_target;
      // Re-calculate distance error
      pos_err[0] = target_pos[0] - pos[0];
      pos_err[1] = target_pos[1] - pos[1];
      distance_to_target = kTargetDistanceMax;
    }

    if (kControllerType == kFuzzyPidControl) {
      vel_target[0] = fuzzy_pid_x->getPid(pos_err[0], dt);
      vel_target[1] = fuzzy_pid_x->getPid(pos_err[1], dt);
    } else {
      // Calculate the distance at which we swap between linear and sqrt velocity response
      linear_distance = kPoscontrolAccelXY / (2.0f * kP * kP);

      if (distance_to_target > 2.0f * linear_distance) {
        // Velocity response grows with the square root of the distance
        float vel_sqrt = safeSqrt(2.0f * kPoscontrolAccelXY * (distance_to_target - linear_distance));
        vel_target[0] = vel_sqrt * pos_err[0] / distance_to_target;
        vel_target[1] = vel_sqrt * pos_err[1] / distance_to_target;
      } else { // Velocity response grows linearly with the distance
        vel_target[0] = pid_x->getKp() * pos_err[0];
        vel_target[1] = pid_y->getKp() * pos_err[1];
      }

      px_i = pid_x->getI(pos_err[0], dt);
      py_i = pid_y->getI(pos_err[1], dt);

      vel_target[0] += px_i;
      vel_target[1] += py_i;
    }
    // This mode is for loiter - rate-limiting the position correction allows the pilot to always override the position
    // correction in the event of a disturbance

    // Scale velocity within limit
    float vel_total = sqrtf(vel_target[0] * vel_target[0] + vel_target[1] * vel_target[1]);
    if (vel_total > kPoscontrolSpeed) {
      vel_target[0] = kPoscontrolSpeed * vel_target[0] / vel_total;
      vel_target[1] = kPoscontrolSpeed * vel_target[1] / vel_total;
    }
  }
}

// rateToAccelXY - horizontal desired rate to desired acceleration
//  converts desired velocities in lat/lon directions to accelerations in
//  lat/lon frame
void PosControl::rateToAccelXY(float dt, float ekf_nav_vel_gain_scaler,
                               double vel_target[3], double acc_target[3]) {
  float accel_total;  // total acceleration in m/s/s
  float vx_i = 0, vy_i = 0;
  float accel_feedforward[2];
  static float last_acc_target[3] = {0};

  // Feed forward desired acceleration calculation
  if (dt > 0.0f) {
    accel_feedforward[0] = (vel_target[0] - vel_last[0]) / dt;
    accel_feedforward[1] = (vel_target[1] - vel_last[1]) / dt;
  } else {
    accel_feedforward[0] = 0.0f;
    accel_feedforward[1] = 0.0f;
  }

  // Store this iteration's velocities for the next iteration here is problem: vel_last stores last iteration's vel,
  // that should be vel_ekf not vel_target?
  vel_last[0] = vel_target[0];
  vel_last[1] = vel_target[1];

  // Calculate velocity error
  vel_err[0] = vel_target[0] - vel_ekf[0];
  vel_err[1] = vel_target[1] - vel_ekf[1];

  // Combine feed forward accel with PID output from velocity error and scale PID output to compensate for optical flow
  // measurement induced EKF noise
  switch (kControllerType) {
    case kPidControl:
      acc_target[0] = kFeedForwardParam * accel_feedforward[0] +
                      (pid_vx->getPid(vel_err[0], dt)) * ekf_nav_vel_gain_scaler;
      acc_target[1] = kFeedForwardParam * accel_feedforward[1] +
                      (pid_vy->getPid(vel_err[1], dt)) * ekf_nav_vel_gain_scaler;
      break;
    case kFuzzyPidControl:
      acc_target[0] = kFeedForwardParam * accel_feedforward[0] +
                      (fuzzy_pid_vx->getPid(vel_err[0], dt)) * ekf_nav_vel_gain_scaler;
      acc_target[1] = kFeedForwardParam * accel_feedforward[1] +
                      (fuzzy_pid_vy->getPid(vel_err[1], dt)) * ekf_nav_vel_gain_scaler;
      break;
    case kAdrcControl:
      acc_target[0] = adrc_vx->updateController(last_acc_target[0], vel_ekf[0], vel_target[0]);
      acc_target[1] = adrc_vy->updateController(last_acc_target[1], vel_ekf[1], vel_target[1]);
      last_acc_target[0] = acc_target[0];
      last_acc_target[1] = acc_target[1];
      break;
    default:
      acc_target[0] = kFeedForwardParam * accel_feedforward[0] +
                      (pid_vx->getPid(vel_err[0], dt)) * ekf_nav_vel_gain_scaler;
      acc_target[1] = kFeedForwardParam * accel_feedforward[1] +
                      (pid_vx->getPid(vel_err[1], dt)) * ekf_nav_vel_gain_scaler;
      break;
  }

  // Scale desired acceleration if it's beyond acceptable limit To-Do: move this check down to the accel_to_lean_angle
  // method?
  accel_total = sqrtf(acc_target[0] * acc_target[0] + acc_target[1] * acc_target[1]);

  if (accel_total > kPoscontrolAccelXYMax && accel_total > 0.0f) {
    acc_target[0] = kPoscontrolAccelXYMax * acc_target[0] / accel_total;
    acc_target[1] = kPoscontrolAccelXYMax * acc_target[1] / accel_total;
  }
}

// AccelToLeanAngles - horizontal desired acceleration to lean angles converts desired accelerations provided in lat/lon
// frame to roll/pitch angles
void PosControl::accelToLeanAngles(float dt, float ekf_nav_vel_gain_scaler, double acc_target[3]) {
  float accel_north = acc_target[0];
  float accel_west = acc_target[1];
  float accel_left, accel_forward;
  float lean_angle_max = kMaxAngle;
  // Apply jerk limit of 17 m/s^3 - equates to a worst case of about 100 deg/sec/sec
  static float last_accel_north = 0.0f;
  static float last_accel_west = 0.0f;
  float max_delta_accel = dt * 17.0f;

  if (accel_north - last_accel_north > max_delta_accel)
    accel_north = last_accel_north + max_delta_accel;
  else if (accel_north - last_accel_north < -max_delta_accel)
    accel_north = last_accel_north - max_delta_accel;
 
  last_accel_north = accel_north;

  if (accel_west - last_accel_west > max_delta_accel)
    accel_west = last_accel_west + max_delta_accel;
  else if (accel_west - last_accel_west < -max_delta_accel)
    accel_west = last_accel_west - max_delta_accel;

  last_accel_west = accel_west;

  // 5Hz lowpass filter on NW accel
  float freq_cut = 5.0f * ekf_nav_vel_gain_scaler;
  float alpha = constrainFloat(dt / (dt + 1.0f / (2.0f * (float)M_PI * freq_cut)), 0.0f, 1.0f);
  static float accel_north_filtered = 0.0f;
  static float accel_west_filtered = 0.0f;

  accel_north_filtered += alpha * (accel_north - accel_north_filtered);
  accel_west_filtered += alpha * (accel_west - accel_west_filtered);

  // Rotate accelerations into body forward-left frame
  accel_forward = accel_north_filtered * cos(yaw) + accel_west_filtered * sin(yaw); // yaw is in NWU frame
  accel_left = -accel_north_filtered * sin(yaw) + accel_west_filtered * cos(yaw);

  // Update angle targets that will be passed to stabilize controller in nwu frame
  pitch_target = constrainFloat(atan2(accel_forward, kGravityMss), -lean_angle_max, lean_angle_max);

  float cos_pitch_target = cosf(pitch_target);
  roll_target = -constrainFloat(atan2(accel_left * cos_pitch_target, kGravityMss), -lean_angle_max, lean_angle_max);
}

// Position error to lean angles(method from vijay kumar)
void PosControl::posrateToLeanAngles(double vel_target[3], double acc_target[3]) {
  pos_err[0] = pos[0] - target_pos[0];
  pos_err[1] = pos[1] - target_pos[1];
  pos_err[2] = pos[2] - target_pos[2];
  vel_err[0] = vel_ekf[0] - vel_target[0];
  vel_err[1] = vel_ekf[1] - vel_target[1];
  vel_err[2] = vel_ekf[2] - vel_target[2];
  Eigen::Vector3d pErr, vErr, aTarget, Ades, xBdes, yBdes, zBdes, xCdes;

  pErr << pos_err[0], pos_err[1], pos_err[2];
  vErr << vel_err[0], vel_err[1], vel_err[2];
  aTarget << acc_target[0], acc_target[1], acc_target[2];

  // Kpos-K of pos pid; Kvel-K of vel pidKpos = 1,Kvel = 1,
  double ga = 9.8;

  // Desired force vector
  Ades = -k_pos * pErr - k_vel * vErr + ga * Eigen::Vector3d(0, 0, 1) + aTarget;
  zBdes = Ades / sqrt(Ades.dot(Ades));
  xCdes << cos(yaw_target), sin(yaw_target), 0;
  yBdes = zBdes.cross(xCdes) / sqrt(zBdes.cross(xCdes).dot(zBdes.cross(xCdes)));
  xBdes = yBdes.cross(zBdes);
 
  double r[9] = {xBdes(0), yBdes(0), zBdes(0), xBdes(1), yBdes(1), zBdes(1), xBdes(2), yBdes(2), zBdes(2)};
  float q[4];
  float test_roll_target, test_pitch_target, test_yaw_target;
  cv::Mat cv_r(3, 3, CV_64F, r);

  rotationToQ(q, cv_r);
  qToEuler(test_roll_target, test_pitch_target, test_yaw_target, q);

  roll_target = test_roll_target;
  pitch_target = test_pitch_target;
}

bool PosControl::isOnArrival(float target_pos_x, float target_pos_y, float target_pos_z) {
  if ((fabs(target_pos_x - pos_x_sum / kMaxCountPosition) < 0.1) &&
      (fabs(target_pos_y - pos_y_sum / kMaxCountPosition) < 0.1) &&
      (fabs(target_pos_z - pos_z_sum / kMaxCountPosition) < 0.1))
    return true;
  else
    return false;
}

// ---- Assistant functions ---- //
void PosControl::reset() {
  ROS_INFO("Reset!!!");

  for (int i = 0; i < 3; ++i) {
    pos_err[i] = 0;
    vel_err[i] = 0;
    acc_err[i] = 0;
    vel_last[i] = 0;
    hover_pos[i] = pos[i];
  }

  for (int i = 0; i < 20; ++i) {
    pos_x_record[i] = 0;
    pos_x_record[i] = 0;
    pos_z_record[i] = 0;
  }

  set_point_pos.clear();
  pos_x_sum = 0;
  pos_y_sum = 0;
  pos_z_sum = 0;
  roll_target = 0;
  pitch_target = 0;

  pid_x->resetIntegrator();
  pid_vx->resetIntegrator();
  pid_y->resetIntegrator();
  pid_vy->resetIntegrator();

  is_vision_hover_ = false;
  is_control_outside_ = false;
  is_set_vel_ = false;
}

/// Actuator
void PosControl::setState(float *pos_now, float *vel_now, float yaw_now) {
  for (int i = 0; i < 3; ++i) {
    vel_last[i] = vel_ekf[i];
    pos[i] = pos_now[i];
    vel_ekf[i] = vel_now[i];
  }
  yaw = yaw_now;
}

void PosControl::takeOff() { 
  drone.takeOff(); 
}

void PosControl::land() { 
  drone.land();
}

void PosControl::doHover(double dt, float *pose_target, float yaw_target, float *pose_now, float *vel_now, float
                         yaw_now) {
  static double vel_target[3], acc_target[3];

  if (kVisionHoverSwitch) {
    if (!is_vision_hover_) {
      std_msgs::Bool vision_hover_data;
      vision_hover_data.data = true;
      vision_hover_pub_.publish(vision_hover_data);
      is_vision_hover_ = true;
    }
  }

  for (int i = 0; i < 3; ++i) {
    target_pos[i] = pose_target[i];
    pos[i] = pose_now[i];
    vel_ekf[i] = vel_now[i];
  }

  yaw = yaw_now;
  yaw_target = yaw_target;

#if PIONEER3AT
  doMove(dt, 0, 0, 0, 0);
  drone.move(0, 0);
#else
  posToRateXY(dt, 1, vel_target);
  rateToAccelXY(dt, 1, vel_target, acc_target);
  float acc_body[2];
  acc_body[0] = acc_target[0] * cos(yaw) + acc_target[1] * sin(yaw);
  acc_body[1] = -acc_target[0] * sin(yaw) + acc_target[1] * cos(yaw);
  pitch_target = atan2(acc_body[0], kGravityMss);
  roll_target = -atan2(acc_body[1], kGravityMss);

  CLIP3(-kMaxAngle, pitch_target, kMaxAngle);
  CLIP3(-kMaxAngle, roll_target, kMaxAngle);
  float yaw_err = 0;

  if (yaw_target * yaw >= 0)
    yaw_err = yaw_target - yaw;
  else {
    if (fabs(yaw_target - yaw) <= M_PI)
      yaw_err = yaw_target - yaw;
    else {
      if (yaw_target - yaw < 0)
        yaw_err = 2 * M_PI - fabs(yaw_target - yaw);
      else
        yaw_err = fabs(yaw_target - yaw) - 2 * M_PI;
    }
  }

  float alt_err = target_pos[2] - pos[2];
  float yaw_speed = PID_yaw->get_pid(yaw_err, dt);
  float upd = PID_z->get_pid(alt_err, dt);

  drone.move(roll_target, pitch_target, upd, yaw_speed);
#endif

#if DEBUG
  std::cout << "roll:" << roll << "  " << "pitch:" << pitch << "  " << "yaw:" << yaw << '\n';
  std::cout << "acc[0]:" << acc[0] << "  " << "acc[1]:" << acc[1] << "  " << "acc[2]:" << acc[2] << '\n';
#endif
}

void PosControl::doSetPoint(double dt) {
#if PIONEER3AT
  drone.move(0, 0);
#else
  static double vel_target[3], acc_target[3];
  if (set_point_pos.size() != 0) {
    target_pos[0] = set_point_pos[0].x + hover_pos[0];
    target_pos[1] = set_point_pos[0].y + hover_pos[1];
    target_pos[2] = set_point_pos[0].z;
    yaw_target = set_point_pos[0].yaw;
    posToRateXY(dt, 1, vel_target);
    rateToAccelXY(dt, 1, vel_target, acc_target);
    accelToLeanAngles(dt, 1, acc_target);

    // >>> test by xwy
    // posrate_to_lean_angles(vel_target,acc_target);
    // <<< test by xwy

    CLIP3(-kMaxAngle, pitch_target, kMaxAngle);
    CLIP3(-kMaxAngle, roll_target, kMaxAngle);

    float yaw_err = 0;
    if (yaw_target * yaw >= 0) {
      yaw_err = yaw_target - yaw;
    } else {
      if (fabs(yaw_target - yaw) <= M_PI) {
        yaw_err = yaw_target - yaw;
      } else {
        if (yaw_target - yaw < 0)
          yaw_err = 2 * M_PI - fabs(yaw_target - yaw);
        else
          yaw_err = fabs(yaw_target - yaw) - 2 * M_PI;
      }
    }

    float alt_err = target_pos[2] - pos[2];
    float yaw_speed = PID_yaw->get_pid(yaw_err, dt);
    float upd = PID_z->get_pid(alt_err, dt);
    drone.move(roll_target, pitch_target, upd, yaw_speed);

    // Judge whether get the target point and loiter for specified time
    if (!reach_point_flag) { // when it doesn't reach the point
      if (isOnArrival(target_pos[0], target_pos[1], target_pos[2])) {
        reach_point_flag = true;
        setpoint_loiter_time = ros::Time().now();
      }
    } else { // after reach the target point, do loitering
      if ((ros::Time().now().toSec() - setpoint_loiter_time.toSec()) >= set_point_pos[0].loiter_time) {
        set_point_pos.erase(set_point_pos.begin());
        reach_point_flag = false;
      }
    }

    // Close vision hover
    if (kVisionHoverSwitch) {
      if (is_vision_hover_) {
        std_msgs::Bool vision_hover_data;
        vision_hover_data.data = false;
        vision_hover_pub_.publish(vision_hover_data);
        is_vision_hover_ = false;
      }
    }
  } else {
    for (int i = 0; i < 3; ++i) 
      hover_pos[i] = pos[i];
    is_set_point_ = false;
  }
#endif
}

void PosControl::doSetVelocity(double dt) {
#if PIONEER3AT
  drone.move(0, 0);
#else
  ROS_INFO("[POSECONTROL]Control by human on velocity!");

  static double vel_target[3], acc_target[3];
  vel_target[0] = target_velocity.data[0],
  vel_target[1] = target_velocity.data[1];

  if (sqrtf(vel_target[0] * vel_target[0] + vel_target[1] * vel_target[1]) < 0.01) {
    if (sqrtf(vel_ekf[0] * vel_ekf[0] + vel_ekf[1] * vel_ekf[1]) < 0.5)
      latch_for_vel = true;
  }

  if (sqrtf(vel_target[0] * vel_target[0] + vel_target[1] * vel_target[1]) > 0.05)
    latch_for_vel = false;

  if (latch_for_vel)
    return;

  // Close vision hover
  if (kVisionHoverSwitch) {
    if (is_vision_hover_) {
      std_msgs::Bool vision_hover_data;
      vision_hover_data.data = false;
      vision_hover_pub_.publish(vision_hover_data);
      is_vision_hover_ = false;
    }
  }

  rateToAccelXY(dt, 1, vel_target, acc_target);
  accelToLeanAngles(dt, 1, acc_target);

  // >>> test by xwy
  // posrate_to_lean_angles(vel_target,acc_target);
  // <<< test by xwy

  CLIP3(-kMazAngle, pitch_target, kMazAngle);
  CLIP3(-kMazAngle, roll_target, kMazAngle);

  drone.move(roll_target, pitch_target, target_velocity.data[2], target_velocity.data[3]);

  yaw_target = yaw;

  for (int i = 0; i < 3; ++i) 
    hover_pos[i] = pos[i];
#endif
}

void PosControl::brake(double dt) {
#if PIONEER3AT
  drone.move(0, 0);
#else
  float fb, lr;
  float vel_body[2] = {
    vel_ekf[0] * cos(yaw) + vel_ekf[1] * sin(yaw),
    -vel_ekf[0] * sin(yaw) + vel_ekf[1] * cos(yaw)
  };
  float absvel0 = fabs(vel_body[0]), absvel1 = fabs(vel_body[1]);

  if (absvel0 > absvel1) {
    fb = -vel_body[0] / absvel0;
    lr = -vel_body[1] / absvel0;
  } else {
    fb = -vel_body[0] / absvel1;
    lr = -vel_body[1] / absvel1;
  }

  doMove(dt, fb, lr, 0, 0);
#endif
}

void PosControl::calYaw(float vx, float vy, float vz, float& yaw) {
  float l = sqrt(pow(vx, 2) + pow(vy, 2));

  if (vy > 0)
    yaw = acos(vx / l);
  else
    yaw = -acos(vx / l);
}

// cal err yaw
float PosControl::calYawSpeed(float yaw_tar, float yaw_now) {
  float y_err = yaw_tar - yaw_now;

  if (y_err > M_PI)
    y_err -= 2 * M_PI;
  else if (y_err < -M_PI)
    y_err += 2 * M_PI;

  return y_err;
}

// Do not use this!
void PosControl::doMoveInWorld(double dt, float vx, float vy, float vz, float v_yaw) {
  if (vx == 0 && vy == 0 && v_yaw == 0) {
    drone.move(0, 0);
    return;
  }

  float yaw;
  calYaw(vx, vy, vz, yaw);
  float vyaw = calYawSpeed(yaw, yaw);

#if PIONEER3AT
  // cout << "yaw err=" << _yaw << ", vyaw=" << vyaw << ", _vyaw=" << _vyaw << '\n';
  // float v = sqrt(_vx*_vx + _vy*_vy) * (1 - fabs(vyaw) / M_PI);
  // float v;
  //
  // if(false){//fabs(vyaw) > M_PI/2)
  //   v=0;
  // else
  //   v = sqrt(_vx*_vx + _vy*_vy);
  //
  // float rz = vyaw / M_PI * MAX_ANGULAR_SPEED * 2;
  //
  // if (!p3atOA.modifyVel(v, rz, vyaw, true)) {
  //   if(fabs(vyaw) > M_PI_2)
  //     v = 0;
  //   else
  //     v = v * (1 - fabs(vyaw) *2 / M_PI);
  // }
  //
  // cout << "control:" << v << ", " << rz << '\n';
  //   drone.move(v, rz);
#else
  double vel_target[3], acc_target[3];
  target_pos[0] += vx;
  target_pos[1] += vy;
  target_pos[2] += vz;

  posToRateXY(dt, 1, vel_target);
  rateToAccelXY(dt, 1, vel_target, acc_target);
  accelToLeanAngles(dt, 1, acc_target);
  drone.move(roll_target, pitch_target, 2 * vz, v_yaw * 0.5);
#endif
}

void PosControl::doMove(double dt, float foreback, float left_right, float updown, float turnlr) {
  if (foreback > 1.01)
    foreback = 1;
  else if (foreback < -1.01)
    foreback = -1;

  if (left_right > 1.01)
    left_right = 1;
  else if (left_right < -1.01)
    left_right = -1;

  if (updown > 1.01)
    updown = 1;
  else if (updown < -1.01)
    updown = -1;

  if (turnlr > 1.01)
    turnlr = 1;
  else if (turnlr < -1.01)
    turnlr = -1;

#if PIONEER3AT
  float vx = foreback * max_linear_speed_, rz = turnlr * max_angular_speed_;
  float tvx = vx, trz = rz;
  drone.move(vx, rz);
#else
  double vel_nwu[3] = {
    foreback * cos(yaw) - left_right * sin(yaw),
    foreback * sin(yaw) + left_right * cos(yaw),
    updown * 0.35
  };
  double vel_target[3], acc_target[3];

  for (int i = 0; i < 3; ++i) 
    target_pos[i] += vel_nwu[i] / 2;

  posToRateXY(dt, 1, vel_target);
  rateToAccelXY(dt, 1, vel_target, acc_target);
  accelToLeanAngles(dt, 1, acc_target);
  drone.move(roll_target, pitch_target, 2 * vel_nwu[2], turnlr * 0.5);
#endif
}

// euler in body_nwu frame
void PosControl::qtoEuler() {
  float_t w, x, y, z;
  // diffrent coordinates
  w = sensor_local_position.pose.orientation.w;
  x = sensor_local_position.pose.orientation.x;
  y = sensor_local_position.pose.orientation.y;
  z = sensor_local_position.pose.orientation.z;

  roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));

  if (2 * (w * y - z * x) > 1)
    pitch = asin(1.0);
  else if (2 * (w * y - z * x) < -1.0)
    pitch = asin(-1.0);
  else
    pitch = asin(2 * (w * y - z * x));

  yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

void PosControl::qtoEuler(float& roll, float& pitch, float& yaw, float q[]) {
  float_t w, x, y, z;
  // diffrent coordinates
  w = q[0];
  x = q[1];
  y = q[2];
  z = q[3];

  roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));

  if (2 * (w * y - z * x) > 1)
    pitch = asin(1.0);
  else if (2 * (w * y - z * x) < -1.0)
    pitch = asin(-1.0);
  else
    pitch = asin(2 * (w * y - z * x));

  yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

void PosControl::getRotation() {
  float_t w, x, y, z;
  // coordinates in NWU
  w = sensor_local_position.pose.orientation.w;
  x = sensor_local_position.pose.orientation.x;
  y = sensor_local_position.pose.orientation.y;
  z = sensor_local_position.pose.orientation.z;

  att_r_[0] = w * w + x * x - y * y - z * z;
  att_r_[1] = 2 * (x * y - w * z);
  att_r_[2] = 2 * (x * z + w * y);
  att_r_[3] = 2 * (x * y + w * z);
  att_r_[4] = w * w - x * x + y * y - z * z;
  att_r_[5] = 2 * (y * z - w * x);
  att_r_[6] = 2 * (x * z - w * y);
  att_r_[7] = 2 * (y * z + w * x);
  att_r_[8] = w * w - x * x - y * y + z * z;
}

void PosControl::getRotation(float q[], float r[]) {
  float_t w, x, y, z;
  // coordinates in NWU
  w = q[0];
  x = q[1];
  y = q[2];
  z = q[3];

  r[0] = w * w + x * x - y * y - z * z;
  r[1] = 2 * (x * y - w * z);
  r[2] = 2 * (x * z + w * y);
  r[3] = 2 * (x * y + w * z);
  r[4] = w * w - x * x + y * y - z * z;
  r[5] = 2 * (y * z - w * x);
  r[6] = 2 * (x * z - w * y);
  r[7] = 2 * (y * z + w * x);
  r[8] = w * w - x * x - y * y + z * z;
}

void PosControl::eulerToQ(float q[]) {
  float f_cos_h_roll = cos(roll * .5f);
  float f_sin_h_roll = sin(roll * .5f);
  float f_cos_h_pitch = cos(pitch * .5f);
  float f_sin_h_pitch = sin(pitch * .5f);
  float f_cos_h_yaw = cos(yaw * .5f);
  float f_sin_h_yaw = sin(yaw * .5f);

  /// Cartesian coordinate System
  q[0] = f_cos_h_roll * f_cos_h_pitch * f_cos_h_yaw +
         f_sin_h_roll * f_sin_h_pitch * f_sin_h_yaw;
  q[1] = f_sin_h_roll * f_cos_h_pitch * f_cos_h_yaw -
         f_cos_h_roll * f_sin_h_pitch * f_sin_h_yaw;
  q[2] = f_cos_h_roll * f_sin_h_pitch * f_cos_h_yaw +
         f_sin_h_roll * f_cos_h_pitch * f_sin_h_yaw;
  q[3] = f_cos_h_roll * f_cos_h_pitch * f_sin_h_yaw -
         f_sin_h_roll * f_sin_h_pitch * f_cos_h_yaw;
}

void PosControl::eulerToQ(float_t roll, float_t pitch, float_t yaw, float q[]) {
  float f_cos_h_roll = cos(roll * .5f);
  float f_sin_h_roll = sin(roll * .5f);
  float f_cos_h_pitch = cos(pitch * .5f);
  float f_sin_h_pitch = sin(pitch * .5f);
  float f_cos_h_yaw = cos(yaw * .5f);
  float f_sin_h_yaw = sin(yaw * .5f);

  /// Cartesian coordinate System
  q[0] = f_cos_h_roll * f_cos_h_pitch * f_cos_h_yaw +
         f_sin_h_roll * f_sin_h_pitch * f_sin_h_yaw;
  q[1] = f_sin_h_roll * f_cos_h_pitch * f_cos_h_yaw -
         f_cos_h_roll * f_sin_h_pitch * f_sin_h_yaw;
  q[2] = f_cos_h_roll * f_sin_h_pitch * f_cos_h_yaw +
         f_sin_h_roll * f_cos_h_pitch * f_sin_h_yaw;
  q[3] = f_cos_h_roll * f_cos_h_pitch * f_sin_h_yaw -
         f_sin_h_roll * f_sin_h_pitch * f_cos_h_yaw;
}

void PosControl::transformBodyFromNwu(const float_t a_nwu[3], float_t a_body[3]) {
  for (int i = 0; i < 3; ++i) 
    a_body[i] = 0;

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      a_body[i] += att_r_[3 * j + i] * a_nwu[j];
}

void PosControl::transformNwuFromBody(const float_t a_body[3], float_t a_nwu[3]) {
  for (int i = 0; i < 3; ++i) {
    a_nwu[i] = 0;
  }

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      a_nwu[i] += a_body[j] * att_r_[3 * i + j];
}
