#ifndef AI_NAVIGATION_POSEKF_VICON_H_
#define AI_NAVIGATION_POSEKF_VICON_H_

#include <geometry_msgs/TransformStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
  
struct ViconPoint {
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
  double pitch;
  double roll;
  double yaw;
};

class Vicon {
public:
  Vicon(ros::NodeHandle *nh);
  void getQInworld(float q[]);
  void q2Euler(double& roll, double& pitch, double& yaw, double q[]);

public:
  bool convert_init_flag;
  bool convert_start_flag;
  bool markers_update_flag;
  bool posedata_flag;
  bool set_qbw_flag;
  bool veldata_flag;
  bool vicon_init_flag;
  bool vicon_update_flag;
  
  double data_time;
  double frame_count;
  double frame_count_last;
  double frame_count_start;  
  double frame_markers_count; 
  double frame_markers_count_start;
  double start_time;
  double time_between_frame;

  double qbw[4];
  double qbw_start[4];
  double q_w_from_v[4];
  double rwv[9];
  float pose_last[3];
  float translation[3];
  float vel_last[3];

  // **********KF for vicon velocity**********
  float kf_kk;
  float kf_p;
  float kf_q;
  float kf_r;
  float vicon_kf_vx;
  float vicon_kf_vy;
  float vicon_kf_vz;
  // **********KF for vicon acceleration******
  float acc_kf_kk;
  float acc_kf_p;
  float acc_kf_q;
  float acc_kf_r;
  float vicon_kf_ax;
  float vicon_kf_ay;
  float vicon_kf_az;

  ViconPoint state;
  ros::Subscriber vicondata_sub;
  ros::Publisher vicon_to_world_pub;
  geometry_msgs::Transform vicon_to_world_transform;

private:
  void getRotation(double q[], double r[]);
  void qMultiplyQ(double q1[], double q2[], double q_result[]);
  void setup();
  void transformWorldFromVicon(const double a_vicon[3], double a_world[3]);
  void viconDataCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
  
private:
  ros::NodeHandle nh_;
};

#endif // AI_NAVIGATION_POSEKF_VICON_H_
