#include <algorithm>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iostream>
#include <string>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MagneticModel.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "pose_ekf.h"
#include "utils/math_aux.h"
#include "vicon.h"

#define SIMULATION 0
#define ARDRONE 0
#define PIONEER3AT 1

#define ORB 0
#define CARTO 1

using namespace Eigen;

std::string drift_path;
std::string geoids_path;

std::fstream drift_file;

boost::shared_ptr<GeographicLib::Geoid> geoid;

int USE_GPS = 0;
int USE_VICON = 1;

bool get_slamrt_done = 0;
bool imuok = false;
float Rwslam[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, Twslam[3] = {0, 0, 0};
float q_bworld[4] = {0, 0, 0, 1}, q_worldfromslam[4] = {0, 0, 0, 1}, R_bworld[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1},
      rol_world = 0, pit_world = 0, yaw_world = 0, pos_gtworld[3] = {0, 0, 0}, deltaYaw = 0;
float err_cam_to_drone_center = 0.2;
int cnt_slam = 0, cntd_slam = 10; 
bool brain_ok = false;
float p3at_speed_modify = 1.0;
// slamtec's yaw
bool first_slamtec_sub_flag = true;
float err_slamtec_imu = 0, yaw_slamtec_mod = 0;
int cnt_slamtecyaw = 0;

PoseEkf pose_ekf;
Vicon *vicon;
ros::Publisher pose_pub;
ros::Publisher vel_pub;
ros::Publisher vicon_pose_pub;
ros::Publisher vicon_vel_pub;
ros::Publisher pub_pose_debug;
ros::Publisher pub_vel_debug;
ros::Publisher pub_px4pose_debug;
ros::Publisher pub_px4vel_debug;
ros::Publisher test_slam_pub;
ros::Publisher test_gt_pub;
ros::Publisher slam_pos_pub;
ros::Publisher qt_worldslam_pub;

nav_msgs::Path path_msg;
ros::Publisher pub_path;

std::deque<std::pair<double, geometry_msgs::PoseStamped>> slam_q;
std::deque<std::pair<double, sensor_msgs::Imu>> imu_q;
std::deque<std::pair<double, Vector2d>> vel_q;
std::deque<std::pair<double, double>> height_q;  // ardrone_autonomy::navdata_altitude
std::deque<std::pair<double, geometry_msgs::PoseStamped>> correct_pose_q;

void publishPose(PoseEkf pose_ekf) {
  geometry_msgs::PoseStamped pose;
  geometry_msgs::TwistStamped velocity;
  Quaterniond q;
  Vector3d p, v, bw, ba;
  pose_ekf.getState(q, p, v, ba);
  // float q[4]={q.w(), q.x(), q.y(), q.z()};
  q_bworld[0] = q.w();
  q_bworld[1] = q.x();
  q_bworld[2] = q.y();
  q_bworld[3] = q.z();
  qToEuler(rol_world, pit_world, yaw_world, q_bworld);
  qToRotation(q_bworld, R_bworld);
  pos_gtworld[0] = p(0);
  pos_gtworld[1] = p(1);
  pos_gtworld[2] = p(2);
  pose.header.stamp = ros::Time(pose_ekf.getTime());
  pose.header.frame_id = "/world";

  velocity.header.stamp = ros::Time(pose_ekf.getTime());
  velocity.header.frame_id = "/world";
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;

  pose.pose.orientation.w = vicon->qbw[0];
  pose.pose.orientation.x = vicon->qbw[1];
  pose.pose.orientation.y = vicon->qbw[2];
  pose.pose.orientation.z = vicon->qbw[3];  // q is NWU frame
  pose.pose.position.x = vicon->state.x;
  pose.pose.position.y = vicon->state.y;
  pose.pose.position.z = vicon->state.z;  // publish postion as NWU frame
  vicon_pose_pub.publish(pose);

  velocity.twist.linear.x = vicon->state.vx;
  velocity.twist.linear.y = vicon->state.vy;
  velocity.twist.linear.z = vicon->state.vz;  // publish as NWU frame
  vicon_vel_pub.publish(velocity);
  if (USE_VICON) {
    pose.pose.orientation.w = vicon->qbw[0];
    pose.pose.orientation.x = vicon->qbw[1];
    pose.pose.orientation.y = vicon->qbw[2];
    pose.pose.orientation.z = vicon->qbw[3];  // q is NWU frame
    pose.pose.position.x = vicon->state.x;
    pose.pose.position.y = vicon->state.y;
    pose.pose.position.z = vicon->state.z;  // publish postion as NWU frame
    pose_pub.publish(pose);

    velocity.twist.linear.x = vicon->state.vx;
    velocity.twist.linear.y = vicon->state.vy;
    velocity.twist.linear.z = vicon->state.vz;  // publish as NWU frame
    vel_pub.publish(velocity);
  } else {
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();  // q is NWU frame
    pose.pose.position.x = p(0);
    pose.pose.position.y = p(1);
    pose.pose.position.z = p(2);  // publish postion as NWU frame
    pose_pub.publish(pose);

    velocity.twist.linear.x = v(0);
    velocity.twist.linear.y = v(1);
    velocity.twist.linear.z = v(2);  // publish as NWU frame
    vel_pub.publish(velocity);
  }

  geometry_msgs::Vector3 data;
  data.x = pose.pose.position.x;
  data.y = pose.pose.position.y;
  data.z = pose.pose.position.z;
  pub_pose_debug.publish(data);
  data.x = velocity.twist.linear.x;
  data.y = velocity.twist.linear.y;
  data.z = velocity.twist.linear.z;
  pub_vel_debug.publish(data);
}

bool loadModels() {
  if (!geoid) {
    try {
      geoid = boost::make_shared<GeographicLib::Geoid>("egm84-15", geoids_path.c_str());
    } catch (GeographicLib::GeographicErr &e) {
      ROS_ERROR("[POS_EKF]Failed to load geoid. Reason: %s", e.what());
      return false;
    }
  }

  return true;
}

bool processSensorData() {
  if (imu_q.empty() || (imu_q.back().first - imu_q.front().first) < 0.05)
    return false;

  static int imu_cnt = 0;  // correct with acc every 10 times
  // find the first com sensor
  double t[8] = {DBL_MAX};
  for (int i = 0; i < 8; i++) t[i] = DBL_MAX;
  if (!imu_q.empty()) t[0] = imu_q.front().first;
  if (!vel_q.empty()) t[1] = vel_q.front().first;
  if (!height_q.empty()) t[2] = height_q.front().first;
  if (!correct_pose_q.empty()) t[3] = correct_pose_q.front().first;
  if (!slam_q.empty()) t[4] = slam_q.front().first;

  int min_id;
  min_id = std::min_element(t, t + 8) - t;
  
  if (t[min_id] == DBL_MAX) 
    return false;

  if (min_id == 0) { // imu
    double t = imu_q.front().first;
    sensor_msgs::Imu msg = imu_q.front().second;
    Eigen::Vector3d acc;
    Eigen::Vector4d q;
    float q_world[4];
    acc(0) = msg.linear_acceleration.x;
    acc(1) = msg.linear_acceleration.y;
    acc(2) = msg.linear_acceleration.z;
    q(0) = msg.orientation.w;
    q(1) = msg.orientation.x;
    q(2) = msg.orientation.y;
    q(3) = msg.orientation.z;
   
    if (get_slamrt_done) {  // false
      q_world[0] = q_worldfromslam[0];
      q_world[1] = q_worldfromslam[1];
      q_world[2] = q_worldfromslam[2];
      q_world[3] = q_worldfromslam[3];
      q(0) = q_worldfromslam[0];
      q(1) = q_worldfromslam[1];
      q(2) = q_worldfromslam[2];
      q(3) = q_worldfromslam[3];
    } else {
      q_world[0] = msg.orientation.w;
      q_world[1] = msg.orientation.x;
      q_world[2] = msg.orientation.y;
      q_world[3] = msg.orientation.z;
    }

    pose_ekf.predict(q, acc, t);
    imu_cnt++;
    if (!(vicon->set_qbw_flag)) 
      vicon->getQInworld(q_world);

    if (imu_cnt % 10 == 0) 
      pose_ekf.correctGravity(acc, t);

    imu_q.pop_front();
  } else if (min_id == 1) { // vel
    double t = vel_q.front().first;
    Eigen::Vector2d opt_velocity = vel_q.front().second;
    pose_ekf.correctOptVelocity(opt_velocity, t);
    vel_q.pop_front();
  } else if (min_id == 2) { // height
    double t = height_q.front().first;
    // ardrone_autonomy::navdata_altitude msg = height_q.front().second;
    double sonar_height = height_q.front().second;  // msg.altitude_raw*1.0/1000;
    pose_ekf.correctSonarHeight(sonar_height, t);
    height_q.pop_front();
  } else if (min_id == 3) { // correct pose
    double t = correct_pose_q.front().first;
    geometry_msgs::PoseStamped msg = correct_pose_q.front().second;
    Eigen::Vector2d d_position;
    Eigen::Vector4d d_q;
    d_position(0) = msg.pose.position.x;
    d_position(1) = msg.pose.position.y;
    d_q(0) = msg.pose.orientation.w;
    d_q(1) = msg.pose.orientation.x;
    d_q(2) = msg.pose.orientation.y;
    d_q(3) = msg.pose.orientation.z;
    pose_ekf.correctPoseCorrect(d_position, d_q, t);
    correct_pose_q.pop_front();
  } else if (min_id == 4) { // correct slam pose
    double _t = slam_q.front().first;
    geometry_msgs::PoseStamped msg = slam_q.front().second;
    Eigen::Vector3d slam_position;
    slam_position(0) = msg.pose.position.x;
    slam_position(1) = msg.pose.position.y;
    slam_position(2) = msg.pose.position.z;
    pose_ekf.correctSlamPose(slam_position, _t);
    slam_q.pop_front();
  }

  return true;
}

void slamCallback(const geometry_msgs::PoseStampedPtr& slam_msg) {
  if (!imuok) {
    return;
  }
  // test
  float rol, pit, yaw, q[4], postmp[3];
  if (ORB) {
    //    geometry_msgs::Vector3 data;
    q[0] = float(slam_msg->pose.orientation.w);
    q[1] = float(slam_msg->pose.orientation.z);
    q[2] = float(-slam_msg->pose.orientation.x);
    q[3] = float(-slam_msg->pose.orientation.y);

    // get slam position and change into nwu
    postmp[0] = slam_msg->pose.position.z;
    postmp[1] = -slam_msg->pose.position.x;
    postmp[2] = -slam_msg->pose.position.y;
    // cout << "slam call back" <<endl;
  } else if (CARTO) {
    q[0] = slam_msg->pose.orientation.w;
    q[1] = slam_msg->pose.orientation.x;
    q[2] = slam_msg->pose.orientation.y;
    q[3] = slam_msg->pose.orientation.z;
    postmp[0] = slam_msg->pose.position.x;
    postmp[1] = slam_msg->pose.position.y;
    postmp[2] = slam_msg->pose.position.z;
  }
  qToEuler(rol, pit, yaw, q);
  cnt_slam = 0;

  if (get_slamrt_done) {
    // then transform into drone world frame
    geometry_msgs::PoseStamped slam_msg;
    float pos[3];
    transformNwuWorldFromBody(postmp[0], postmp[1], postmp[2], pos[0],
                                 pos[1], pos[2], Rwslam, Twslam);
    // yaw += deltaYaw;
    float Rbodyworld[9], Rbodyslam[9];
    qToRotation(q, Rbodyslam);
    rMultiR(Rwslam, Rbodyslam, Rbodyworld);
    rotationToQ(q, Rbodyworld);
    qToEuler(rol, pit, yaw, q);
    slam_msg.pose.position.x = pos[0] - err_cam_to_drone_center * cos(yaw);
    slam_msg.pose.position.y = pos[1] - err_cam_to_drone_center * sin(yaw);
    slam_msg.pose.position.z = pos[2] + err_cam_to_drone_center * sin(pit);
    // eulerToQ(rol, pit, yaw, q);
    q_worldfromslam[0] = q[0];
    q_worldfromslam[1] = q[1];
    q_worldfromslam[2] = q[2];
    q_worldfromslam[3] = q[3];
    
    slam_msg.pose.orientation.w = q[0];
    slam_msg.pose.orientation.x = q[1];
    slam_msg.pose.orientation.y = q[2];
    slam_msg.pose.orientation.z = q[3];
    // cout << "pub slam pos after modify" <<endl;
    slam_pos_pub.publish(slam_msg);
    double t = slam_msg.header.stamp.toSec();  // ros::Time::now().toSec();
    slam_q.push_back(std::make_pair(t, slam_msg));
  } else {
    if (cntd_slam > 0) {
      --cntd_slam;
      return;
    }
    // if(isnan(pos_gtworld[0])) return;
    // calculate the R T from slam_nwu to drone_world
    float pos_worldtmp[3], R_bodyworld[9], R_bodyslam[9];
    Eigen::Matrix<float, 4, 4> Tslamworld, Tbodyworld, Tbodyslam;
    qToRotation(q_bworld, R_bodyworld);
    Tbodyworld << R_bodyworld[0], R_bodyworld[1], R_bodyworld[2],
                  pos_gtworld[0], R_bodyworld[3], R_bodyworld[4], R_bodyworld[5],
                  pos_gtworld[1], R_bodyworld[6], R_bodyworld[7], R_bodyworld[8],
                  pos_gtworld[2], 0, 0, 0, 1;
    
    qToRotation(q, R_bodyslam);
    
    Tbodyslam << R_bodyslam[0], R_bodyslam[1], R_bodyslam[2], postmp[0],
                 R_bodyslam[3], R_bodyslam[4], R_bodyslam[5], postmp[1], R_bodyslam[6],
                 R_bodyslam[7], R_bodyslam[8], postmp[2], 0, 0, 0, 1;
    
    Tslamworld = Tbodyworld * Tbodyslam.inverse();
    
    Rwslam[0] = Tslamworld(0, 0);
    Rwslam[1] = Tslamworld(0, 1);
    Rwslam[2] = Tslamworld(0, 2);
    Rwslam[3] = Tslamworld(1, 0);
    Rwslam[4] = Tslamworld(1, 1);
    Rwslam[5] = Tslamworld(1, 2);
    Rwslam[6] = Tslamworld(2, 0);
    Rwslam[7] = Tslamworld(2, 1);
    Rwslam[8] = Tslamworld(2, 2);
    Twslam[0] = Tslamworld(0, 3);
    Twslam[1] = Tslamworld(1, 3);
    Twslam[2] = Tslamworld(2, 3);
    float qq[4];
    // pub it
    rotationToQ(qq, Rwslam);
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = Twslam[0];
    msg.pose.position.y = Twslam[1];
    msg.pose.position.z = Twslam[2];
    msg.pose.orientation.w = qq[0];
    msg.pose.orientation.x = qq[1];
    msg.pose.orientation.y = qq[2];
    msg.pose.orientation.z = qq[3];
    ROS_INFO("[POS_EKF]slam frame ok!");
    // sleep(1);
    qt_worldslam_pub.publish(msg);

    get_slamrt_done = true;
    cntd_slam = 10;
  }
}

void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  double t = imu_msg->header.stamp.toSec();
  //    double t = ros::Time::now().toSec();
  imu_q.push_back(std::make_pair(t, *imu_msg));
  if (!imuok) {
    imuok = true;
  }
}

void slamtecYawCallback(const std_msgs::Float32ConstPtr& msg) {
  cnt_slamtecyaw = 0;
  if (first_slamtec_sub_flag) {
    if (imu_q.empty()) {
      return;
    }
    first_slamtec_sub_flag = false;
    float qimu[4], ro, pi, ya;
    sensor_msgs::Imu _imu = imu_q[imu_q.size() - 1].second;
    qimu[0] = _imu.orientation.w;
    qimu[1] = _imu.orientation.x;
    qimu[2] = _imu.orientation.y;
    qimu[3] = _imu.orientation.z;
    qToEuler(ro, pi, ya, qimu);
    err_slamtec_imu = msg->data - ya;
  } else {
    yaw_slamtec_mod = msg->data - err_slamtec_imu;
    if (yaw_slamtec_mod > M_PI) {
      yaw_slamtec_mod -= 2 * M_PI;
    } else if (yaw_slamtec_mod < -M_PI) {
      yaw_slamtec_mod += 2 * M_PI;
    }
  }
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg) {}

// correct pose by image information
// give the correct value os position
// then use ekf to fusion data
void correctPoseDataCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  double t = ros::Time::now().toSec();
  correct_pose_q.push_back(std::make_pair(t, *msg));
}


// vision hover :correct pose by image information
// give the correct value or position
// then use ekf to fusion data
void hoverCorrectPoseDataCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
  double t = ros::Time::now().toSec();

  Eigen::Vector2d d_position;
  Eigen::Vector4d d_q;

  d_position(0) = msg->pose.position.x;
  d_position(1) = msg->pose.position.y;
  d_q(0) = msg->pose.orientation.w;
  d_q(1) = msg->pose.orientation.x;
  d_q(2) = msg->pose.orientation.y;
  d_q(3) = msg->pose.orientation.z;
  pose_ekf.correctPoseCorrect(d_position, d_q, t);
}

#if SIMULATION
// test
void gtposeCallback(const geometry_msgs::PoseConstPtr& msg) {
  
  // test
  geometry_msgs::Vector3 data;
  float rol, pit, yaw,
      q[4] = {msg->orientation.w, msg->orientation.x, msg->orientation.y,
               msg->orientation.z};
  qToEuler(rol, pit, yaw, q);
  data.x = rol;
  data.y = pit;
  data.z = yaw;
  rol_world = rol;
  pit_world = pit;
  yaw_world = yaw;
  pos_gtworld[0] = msg->position.x;
  pos_gtworld[1] = msg->position.y;
  pos_gtworld[2] = msg->position.z;
  test_gt_pub.publish(data);
  // 3 seconds no slam reset the cnt to 0, use gt
  if (++cnt_slam > 300) {
    if (get_slamrt_done == true) {
      get_slamrt_done = false;
      Rwslam[0] = 1;
      Rwslam[4] = 1;
      Rwslam[1] = 0;
      Rwslam[3] = 0;
      Twslam[0] = 0;
      Twslam[1] = 0;
      Twslam[2] = 0;
      ROS_INFO("[POS_EKF]reuse gt pose vel.");
    }
  }

  if (!get_slamrt_done) { // use this as slam for control when no slam
    geometry_msgs::PoseStamped slam_msg;
    slam_msg.pose.position.x = msg->position.x;
    slam_msg.pose.position.y = msg->position.y;
    slam_msg.pose.position.z = msg->position.z;
    slam_msg.pose.orientation.w = q[0];
    slam_msg.pose.orientation.x = q[1];
    slam_msg.pose.orientation.y = q[2];
    slam_msg.pose.orientation.z = q[3];

    double t = ros::Time::now().toSec();
    slam_q.push_back(std::make_pair(t, slam_msg));
  }
  
}
// test
void gtvelCallback(const geometry_msgs::TwistPtr& msg) {
  double t = ros::Time::now().toSec();
  Eigen::Vector2d velocity;
  // convert to nwu_body frame
  velocity(0) = msg->linear.x * cos(yaw_world) + msg->linear.y * sin(yaw_world);  
  velocity(1) = -msg->linear.x * sin(yaw_world) + msg->linear.y * cos(yaw_world);
  vel_q.push_back(std::make_pair(t, velocity));
}

#endif

void p3atPoseCallback(const nav_msgs::OdometryConstPtr& msg) {
  static double last_speed_time = ros::Time::now().toSec();
  double t = msg->header.stamp.toSec();  // ros::Time::now().toSec();
  double dt = t - last_speed_time;
  last_speed_time = t;
  
  if (dt > 1) return;
  Vector2d velocity;
  velocity(0) = 0;
  velocity(1) = 0;
  
  if (false) {  
    double vx = msg->twist.twist.linear.x / p3at_speed_modify,
           rz = msg->twist.twist.angular.z / p3at_speed_modify;
    double r = fabs(vx / rz);
    double d = sqrt(2 * r * r * (1 - cos(rz * dt)));
    velocity(0) = d * cos(rz * dt / 2) / dt;
    velocity(1) = d * sin(rz * dt / 2) / dt;
  } else {
    velocity(0) = msg->twist.twist.linear.x / p3at_speed_modify;
    velocity(1) = msg->twist.twist.linear.y / p3at_speed_modify;
  }
  
  vel_q.push_back(std::make_pair(t, velocity));
  // lock height on plane ground, if not in a plane ground, diable it
  if (true) {
    double _height = 1.0;
    height_q.push_back(std::make_pair(t, _height));
  }
}

void mavrosHeightCallback(const sensor_msgs::RangeConstPtr& msg) {
  double t = ros::Time::now().toSec();
  double m = msg->range;
  height_q.push_back(std::make_pair(t, m));
}

void brainokCallback(const std_msgs::BoolConstPtr& msg) {
  brain_ok = msg->data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_estimator");
  ros::NodeHandle n;
  vicon = new Vicon(&n);

  pub_pose_debug = n.advertise<geometry_msgs::Vector3>("/debug/pose", 10);
  pub_vel_debug = n.advertise<geometry_msgs::Vector3>("/debug/vel", 10);
  pub_path = n.advertise<nav_msgs::Path>("path", 10);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/est_pose", 10);
  vel_pub = n.advertise<geometry_msgs::TwistStamped>("/est_vel", 10);
  vicon_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/vicon/est_pose", 10);
  vicon_vel_pub = n.advertise<geometry_msgs::TwistStamped>("/vicon/est_vel", 10);
  qt_worldslam_pub = n.advertise<geometry_msgs::PoseStamped>("/qtworldslam", 1);

  path_msg.header.frame_id = "world";

  // test
  test_slam_pub = n.advertise<geometry_msgs::Vector3>("/slam_rpy", 10);
  test_gt_pub = n.advertise<geometry_msgs::Vector3>("/gt_rpy", 10);
  slam_pos_pub = n.advertise<geometry_msgs::PoseStamped>("/slam_modify_pose", 10);

#if SIMULATION
  ros::Subscriber sub_imu = n.subscribe("/drone/imu", 100, imuCallback);
  ros::Subscriber pose_sub = n.subscribe("/drone/gt_pose", 1, gtposeCallback);
  ros::Subscriber vel_sub = n.subscribe("/drone/gt_vel", 1, gtvelCallback);
#endif

#if ARDRONE
  ros::Subscriber sub_imu = n.subscribe("/ardrone/imu", 100, imuCallback);
  ros::Subscriber sub_navdata = n.subscribe("ardrone/navdata", 100, navdataCallback);
  ros::Subscriber sub_navaltdata = n.subscribe("/ardrone/navdata_altitude", 100, navaltdataCallback);
  ros::Subscriber sub_odometrydata = n.subscribe("/ardrone/odometry", 100, odometryCallback);
#endif

#if PIONEER3AT
  ros::Subscriber sub_imu = n.subscribe("/mavros/imu/data", 1, imuCallback);
  ros::Subscriber sub_p3atpose = n.subscribe("/RosAria/pose", 1, p3atPoseCallback);
  ros::Subscriber sub_slamtecyaw = n.subscribe("/slamtec/yaw", 5, slamtecYawCallback);
#endif

  ros::Subscriber brainok_sub = n.subscribe("/brain/ok", 1, brainokCallback);
  ros::Subscriber slam_sub = n.subscribe("/slam/pose", 1, slamCallback);
  ros::Subscriber correctPoseData_sub = n.subscribe("/autoreturn/correctpose", 1, correctPoseDataCallback);
  ros::Subscriber hoverCorrectPoseData_sub = n.subscribe("/visionhover/correctpose", 1, hoverCorrectPoseDataCallback);
  // ros::Subscriber bleloc_sub=n.subscribe("/bleloc/pose", 1, blelocCallback);

  n.getParam("use_gps", USE_GPS);
  n.getParam("use_vicon", USE_VICON);
  n.getParam("drift_path", drift_path);
  n.getParam("geoidsPath", geoids_path);
  n.getParam("err_cam_to_drone_center", err_cam_to_drone_center);
  n.getParam("wheel_type", p3at_speed_modify);
  drift_file.open(drift_path.c_str());

  bool ret = loadModels();
  if (!ret) return -1;

  ros::Rate loop_rate(100);
  sleep(1);
  ROS_INFO("[POS_EKF]pose_ekf start!");
  while (ros::ok()) {
    ros::spinOnce();
    // if(!brain_ok) continue;

    // cnt = 0;
    if (cnt_slam < 100) {
      ++cnt_slam;
    } else if (cnt_slam < 101) {
      ++cnt_slam;
      get_slamrt_done = false;
      Twslam[0] = 0;
      Twslam[1] = 0;
      Twslam[2] = 0;
      ROS_INFO("[POS_EKF]miss slam data!");
    }
    if (cnt_slamtecyaw < 50) {
      ++cnt_slamtecyaw;
    } else if (cnt_slamtecyaw < 51) {
      ++cnt_slamtecyaw;
      first_slamtec_sub_flag = true;
      ROS_INFO("[POS_EKF]miss lidar yaw");
    }
    while (processSensorData()) {
    }  //++cnt;
    publishPose(pose_ekf);
    // cout << "process data " <<cnt<< " times"<<endl;
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
