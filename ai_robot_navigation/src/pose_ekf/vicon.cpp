#include "vicon.h"

#include <ros/time.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <deque>

extern std::deque<std::pair<double, Eigen::Vector3d>> vicon_pose_q;
extern std::deque<std::pair<double, Eigen::Vector3d>> vicon_vel_q;

Vicon::Vicon(ros::NodeHandle* nh)
    : nh_(*nh),
      convert_init_flag(false),
      vicon_init_flag(false),
      set_qbw_flag(false),
      convert_start_flag(false),
      posedata_flag(false),
      veldata_flag(false),
      vicon_update_flag(false),
      markers_update_flag(false),
      time_between_frame(0.01) {
  setup();
}

void Vicon::setup() {
  ROS_INFO("[VICON]Vicon Setup ....");

  this->state.x = 0;
  this->state.y = 0;
  this->state.z = 0;
  this->state.vx = 0;
  this->state.vy = 0;
  this->state.vz = 0;
  this->state.roll = 0;
  this->state.pitch = 0;
  this->state.yaw = 0;

  for (int i = 0; i < 4; i++) {
    this->q_w_from_v[i] = 0;
    this->qbw_start[i] = 0;
  }

  this->vicon_to_world_pub = this->nh_.advertise<geometry_msgs::Transform>("/vicon/transformVToW", 1);

  // *********************kf for vicon velocity*****************
  this->vicon_kf_vx = 0;
  this->vicon_kf_vy = 0;
  this->vicon_kf_vz = 0;
  this->kf_p = 1;
  this->kf_q = 0.00001;
  this->kf_r = 0.01;
  this->kf_kk = 0;
  // ***********************************************************

  // *********************kf for vicon acceleration*************
  this->vicon_kf_ax = 0;
  this->vicon_kf_ay = 0;
  this->vicon_kf_az = 0;
  this->acc_kf_p = 1;
  this->acc_kf_q = 0.00001;
  this->acc_kf_r = 0.01;
  this->acc_kf_kk = 0;
  // ************************************************************

  ROS_INFO("[VICON]Vicon Setup  over!");
}

void Vicon::viconDataCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  static double transform_send_time = ros::Time::now().toSec();
  double qbv[4], pose_vicon[3], pose[3];

  qbv[0] = msg->transform.rotation.w;
  qbv[1] = msg->transform.rotation.x;
  qbv[2] = msg->transform.rotation.y;
  qbv[3] = msg->transform.rotation.z;
  pose_vicon[0] = msg->transform.translation.x;
  pose_vicon[1] = msg->transform.translation.y;
  pose_vicon[2] = msg->transform.translation.z;

  if (!this->vicon_update_flag) {
    this->start_time = msg->header.stamp.toSec();
    this->frame_count_start = msg->header.seq;
    this->frame_count_last = this->frame_count = msg->header.seq;
    this->vicon_update_flag = true;
    return;
  }

  this->frame_count = msg->header.seq;
  double dt = (this->frame_count - this->frame_count_last) * this->time_between_frame;
  this->frame_count_last = this->frame_count;

  if (!this->vicon_init_flag) {
    if (this->set_qbw_flag) {
      // conjugate
      qbv[1] = -qbv[1];
      qbv[2] = -qbv[2];
      qbv[3] = -qbv[3];
      qMultiplyQ(this->qbw_start, qbv, this->q_w_from_v);
      getRotation(this->q_w_from_v, this->rwv);
      this->translation[0] = pose_vicon[0];
      this->translation[1] = pose_vicon[1];
      this->translation[2] = pose_vicon[2];

      this->vicon_to_world_transform.translation.x = this->translation[0];
      this->vicon_to_world_transform.translation.y = this->translation[1];
      this->vicon_to_world_transform.translation.z = this->translation[2];
      this->vicon_to_world_transform.rotation.w = this->q_w_from_v[0];
      this->vicon_to_world_transform.rotation.x = this->q_w_from_v[1];
      this->vicon_to_world_transform.rotation.y = this->q_w_from_v[2];
      this->vicon_to_world_transform.rotation.z = this->q_w_from_v[3];

      this->vicon_to_world_pub.publish(this->vicon_to_world_transform);

      this->state.x = 0;
      this->state.y = 0;
      this->state.z = 0;
      this->pose_last[0] = this->state.x;
      this->pose_last[1] = this->state.y;
      this->pose_last[2] = this->state.z;
      this->posedata_flag = true;
      this->vicon_init_flag = true;
    }
  } else {
    if (ros::Time::now().toSec() - transform_send_time > 3.3) {
      // reduce the sending frequence to 3Hz while ensure the transform
      // information is aqcuired by sdk
      transform_send_time = ros::Time::now().toSec();
      this->vicon_to_world_pub.publish(this->vicon_to_world_transform);
    }
    qMultiplyQ(this->q_w_from_v, qbv, this->qbw);
    q2Euler(this->state.roll, this->state.pitch, this->state.yaw, qbw);

    for (int i = 0; i < 3; ++i) pose_vicon[i] -= this->translation[i];
    transformWorldFromVicon(pose_vicon, pose);
    this->state.x = pose[0];
    this->state.y = pose[1];
    this->state.z = pose[2];

    if (this->posedata_flag) {
      this->state.vx = (pose[0] - this->pose_last[0]) / dt;
      this->state.vy = (pose[1] - this->pose_last[1]) / dt;
      this->state.vz = (pose[2] - this->pose_last[2]) / dt;
      this->pose_last[0] = this->state.x;
      this->pose_last[1] = this->state.y;
      this->pose_last[2] = this->state.z;

      if (!this->veldata_flag) {
        this->vel_last[0] = this->state.vx;
        this->vel_last[1] = this->state.vy;
        this->vel_last[2] = this->state.vz;
      }
      this->veldata_flag = true;
    }
  }
}

void Vicon::q2Euler(double& roll, double& pitch, double& yaw, double q[]) {
  float_t w, x, y, z;
  // diffrent coordinates
  w = q[0];
  x = q[1];
  y = q[2];
  z = q[3];
  // cout<<"q:"<<w<<"    "<<x<<"    "<<y<<"    "<<z<<endl;
  roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));

  if (2 * (w * y - z * x) > 1)
    pitch = asin(1.0);
  else if (2 * (w * y - z * x) < -1.0)
    pitch = asin(-1.0);
  else
    pitch = asin(2 * (w * y - z * x));
  yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

void Vicon::qMultiplyQ(double q1[], double q2[], double q_result[]) {
  q_result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
  q_result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
  q_result[2] = q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3];
  q_result[3] = q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1];
}

void Vicon::getQInworld(float q[]) {
  for (int i = 0; i < 4; ++i) this->qbw_start[i] = q[i];
  this->set_qbw_flag = true;

  // can't put this subscribe in setup, if put it in setup will cause conflict
  // of read and write of 'this->set_qbw_flag'
  this->vicondata_sub = this->nh_.subscribe("/vicon/uav_stereo02/uav_stereo02", 20, &Vicon::viconDataCallback, this);
}

void Vicon::getRotation(double q[], double r[]) {
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

void Vicon::transformWorldFromVicon(const double a_vicon[3], double a_world[3]) {
  for (int i = 0; i < 3; ++i) a_world[i] = 0;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      a_world[i] += this->rwv[j + 3 * i] * a_vicon[j];
    }
  }
}