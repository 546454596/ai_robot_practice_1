#ifndef AI_ROBOT_NAVIGATION_ARDRONE_ROS_H_
#define AI_ROBOT_NAVIGATION_ARDRONE_ROS_H_

#include "ros/forwards.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>

// A simple class to send the commands to the drone through the corresponding topics.
class DroneObjectRos {
protected:
  DroneObjectRos() {}

public:
  DroneObjectRos(const ros::NodeHandle& nh);

  void initRosVars();
  bool hover();
  bool hoverMove(float x, float y, float z, float yaw_speed);
  bool land();
  bool move(float v_lr, float v_fb, float v_du, float w_lr);
  bool moveTo(float x, float y, float z);
  bool pitch(float speed = 0.2);
  bool posCtrl(bool on);
  bool rise(float speed = 0.1);
  bool roll(float speed = 0.2);
  bool setAutoreturnRecord(bool set_record); // start record
  bool setAutoreturnReturn(bool set_return); // start return
  bool setHover(int set_hover); // start setHover node 
  bool setPoint(float_t x, float_t y, float_t z, float_t set_yaw, float_t seconds); // control with setHover,relative x and y, globle z
  bool setTrajectory(int set_traject); // start trajectory
  bool setVelocityMode(bool on);
  bool takeOff();
  bool yaw(float speed = 0.1);

public:
  bool is_auto_return;
  bool is_flying;
  bool is_hover;
  bool is_posctrl;
  bool is_set_point;
  bool is_set_trajectory;
  bool is_vel_mode;

  geometry_msgs::Twist twist_msg;
  std_msgs::Float32MultiArray controller_setpoint;

  ros::NodeHandle nh;
  ros::Publisher autoreturn_startRecord_pub;
  ros::Publisher autoreturn_startReturn_pub;
  ros::Publisher cmd_pub;
  ros::Publisher control_settrajectory_pub;
  ros::Publisher hover_cmd_pub;
  ros::Publisher land_pub;
  ros::Publisher pos_ctrl_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher reset_pub;
  ros::Publisher sethover_pub;
  ros::Publisher setpoint_pub;
  ros::Publisher vel_mode_pub;
};

#endif // AI_ROBOT_NAVIGATION_ARDRONE_ROS_H_
