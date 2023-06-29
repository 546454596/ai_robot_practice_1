#ifndef AI_ROBOT_NAVIGATION_POSCONTROL_H_
#define AI_ROBOT_NAVIGATION_POSCONTROL_H_

#include <cmath>
#include <cstdint>
#include <vector>

#ifdef OPENCV2
#include <opencv2/opencv.hpp>
#else
#include <opencv/cv.hpp>
#endif

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/SetMode.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

#include "pos_trajectory.h"
#include "utils/adrc.h"
#include "utils/math_aux.h"
#include "utils/pid_controller.h"
#include "utils/pid_fuzzy.h"

#define ARDRONE 0
#define GAZEBO 0
#define PIONEER3AT 1

#if ARDRONE
#include "ar_drone.h"
#elif GAZEBO
#include "drone_object_ros.h"
#elif PIONEER3AT
#include "brain/pioneer_3at.h"
#endif

// #define CLIP3(n1, n, n2) \
//   do {                   \
//     if (n < n1)          \
//       n = n1;            \
//     else if (n > n2)     \
//       n = n2;            \
//   } while (0)

inline __attribute__((always_inline)) void clip3(float n1, float& n, float n2)  {
  if (n < n1)
    n = n1;
  else if (n > n2)
    n = n2;
}

constexpr int8_t kPidControl = 0;
constexpr int8_t kFuzzyPidControl = 1;
constexpr int8_t kAdrcControl = 2;
constexpr int8_t kDebug = 0;
constexpr int8_t kMoveByAttitude = 0;
constexpr int8_t kMoveByVelocity = 0;
constexpr int8_t kMaxCountPosition = 20;

struct Point {
  float x;
  float y;
  float z;
  float yaw;
  float loiter_time;
};

static void *startHoverThread(void *args);

class PosControl {
 public:
  PosControl(const ros::NodeHandle& nh);
  ~PosControl() {
    this->thread_quit_ = true;
    // delete ardrone;
    std::vector<Point>().swap(this->set_point_pos);
  }

  void accelToLeanAngles(float dt, float ekf_nav_vel_gain_scaler, double acc_target[3]);
  // slow down speed quickly
  void brake(double dt);
  void doHover(double dt, float *pose_target, float yaw_target, 
               float *pose_now, float *vel_now, float yaw_now); 
  // input all -1 to 1 except dt, only vel input, move command are in body c frame
  void doMove(double dt, float for_back, float left_right, float updown, float turnlr); 
  // move command are in world frame
  void doMoveInWorld(double dt, float vx, float vy, float vz, float vyaw);
  // another method for hover vel,acc
  // void get_hover_vel_acc(double vel_target[3], double acc_target[3]);
  bool isOnArrival(float target_pos_x, float target_pos_y, float target_pos_z);
  // for ground robot, land is to disable motor
  void land();
  // void startLoop();
  // void hover_loop();
  void posToRateXY(float dt, float ekf_nav_vel_gain_scaler, double vel_target[3]);
  // method from vijay kumar
  void posrateToLeanAngles(double vel_target[3], double acc_target[3]);
  void rateToAccelXY(float dt, float ekf_nav_vel_gain_scaler,
                     double vel_target[3], double acc_target[3]);
  float *pos_now;
  float *vel_now;
  float yaw_now;  // set state before control
  void setState(float *pos_now, float *vel_now, float yaw_now);
  void setup();
  // for ground robot, takeoff is to enable motor
  void takeOff();

public:
#if ARDRONE
    ArDrone drone;
#elif GAZEBO
    DroneObjectRos drone;
#elif PIONEER3AT
    Pioneer3at drone;
#endif

  bool init_flag;
  bool latch_for_vel;
  bool cotrol_with_vicon;
  bool reach_point_flag;
  // for vijay's
  double k_pos;
  double k_vel;
  double acc_err[3];
  double pos_err[3];
  double vel_err[3];
  double_t trajector_vel[3];
  double_t trajector_acc[3];

  float_t acc[3];
  float_t control_rate;
  float_t hover_pos[3];
  float_t get_pos[3];
  float_t pitch_target;
  float_t pitch;
  float_t pos[3];
  float_t pose_ekf[3];
  float_t pos_x_sum;
  float_t pos_y_sum;
  float_t pos_z_sum;
  float_t pos_x_record[kMaxCountPosition];
  float_t pos_y_record[kMaxCountPosition];
  float_t pos_z_record[kMaxCountPosition];
  float_t roll;
  float_t roll_target;
  float_t target_pos[3];
  float_t vel[3];
  float_t vel_opt[3];
  float_t vel_last[3];
  float_t vel_lp[3];
  float_t vel_ekf[3];
  float_t vicon_yaw;
  float_t vicon_pitch;
  float_t vicon_roll;
  float_t yaw;
  float_t yaw_target;
  
  int move_mode;
  int pos_record_count;

  std::vector<Point> set_point_pos;

  Adrc *adrc_vx;
  Adrc *adrc_vy;
  FuzzyPid *fuzzy_pid_x;
  FuzzyPid *fuzzy_pid_y;
  FuzzyPid *fuzzy_pid_vx;
  FuzzyPid *fuzzy_pid_vy;
  PidController *pid_x;
  PidController *pid_vx;
  PidController *pid_y;
  PidController *pid_vy;
  PidController *pid_yaw;
  PidController *pid_z;

  geometry_msgs::PoseStamped fmu_controller_setpoint_attitude;
  geometry_msgs::PoseStamped sensor_local_position;
  geometry_msgs::TwistStamped sensor_local_velocity;
  sensor_msgs::Imu sensor_imu;
  std_msgs::Float64 set_height;
  std_msgs::Float32MultiArray commander_euler;
  std_msgs::Float32MultiArray commander_move;
  std_msgs::Float32MultiArray target_velocity;
  
  ros::Time check_outside_control;
  ros::Time pid_time;
  ros::Time setpoint_loiter_time;
  
protected:
  pthread_t read_tid_;
  bool thread_quit_;

private:
  void calYaw(float vx, float vy, float vz, float& yaw);
  void doSetPoint(double dt);
  void doSetVelocity(double dt);
  void doTrajector(double dt, double trajectoryTime);
  void eulerToQ(float q[]);
  void eulerToQ(float_t roll, float_t pitch, float_t yaw, float q[]);
  void getRotation();
  void getRotation(float q[], float R[]);
  void posEkfCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void qMulq(float q1[], float q2[], float q_m[]);
  void qtoEuler();
  void qtoEuler(float& roll, float& pitch, float& yaw, float q[]);
  void reset();
  void speedCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void transformBodyFromNwu(const float_t a_nwu[3], float_t a_body[3]);
  void transformNwuFromBody(const float_t a_body[3], float_t a_nwu[3]);
  void velEkfCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  
  float calYawSpeed(float yaw_tar, float yaw_now);
  float constrainFloat(float amt, float low, float high);
  float fastAtan(float v);
  float safeSqrt(float v);
  float trapezoidalIntegral(float dt, float v_last, float v_now); 

private:
  bool is_control_outside_;
  bool is_set_vel_;         // is_vel_set ?
  bool set_hover_;          // is_hover_set ?
  bool is_set_point_;       // is_point_set?
  bool is_set_trajectory_;  // is_traj_set?
  bool is_vision_hover_;    // is_vision_hover (what's the meaning)
  float max_linear_speed_;
  float max_angular_speed_;

  ros::NodeHandle nh_;

  ros::Publisher vision_hover_pub_;
  ros::Publisher tmp_pub1_;
  ros::Publisher tmp_pub2_;
  ros::Publisher trajector_data_pub_;
  ros::Publisher autoreturn_start_record_pub_;
  // publisher for test purpose
  ros::Publisher test_pub_;
  ros::Publisher test_pub2_;
  ros::Publisher test_pub3_;
  ros::Publisher test_pub4_;

  ros::Subscriber pose_ekf_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber vel_ekf_sub_;

  std_msgs::Float64 tmp_;
  PostTrajectory trajectory_;
  float_t att_r_[9];
  
};

#endif  // AI_ROBOT_NAVIGATION_POSCONTROL_H_
