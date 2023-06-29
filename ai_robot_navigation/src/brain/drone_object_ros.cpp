#include "drone_object_ros.h"

DroneObjectRos::DroneObjectRos(const ros::NodeHandle& nh)
    : is_hover(false),
      is_set_trajectory(false),
      is_set_point(false),
      is_auto_return(false),
      nh(nh) {
  initRosVars();
}

void DroneObjectRos::initRosVars() {
  is_flying = false;
  is_posctrl = false;
  is_vel_mode = false;
  takeoff_pub = nh.advertise<std_msgs::Empty>("/drone/takeoff", 1024);
  land_pub = nh.advertise<std_msgs::Empty>("/drone/land", 1024);
  reset_pub = nh.advertise<std_msgs::Empty>("/drone/reset", 1024);
  pos_ctrl_pub = nh.advertise<std_msgs::Bool>("/drone/posctrl", 1024);
  cmd_pub = nh.advertise<geometry_msgs::Twist>("/drone/cmd_val", 1024);
  vel_mode_pub = nh.advertise<std_msgs::Bool>("/drone/vel_mode", 1024);
  hover_cmd_pub = nh.advertise<geometry_msgs::Twist>("/control/velCmd", 10);
  sethover_pub = nh.advertise<std_msgs::Bool>("/control/sethover", 1);
  setpoint_pub = nh.advertise<std_msgs::Float32MultiArray>("/control/setpoint", 10);
  control_settrajectory_pub = nh.advertise<std_msgs::Bool>("/control/settrajectory", 10);
  autoreturn_startRecord_pub = nh.advertise<std_msgs::Bool>("/autoreturn/record", 10);
  autoreturn_startReturn_pub = nh.advertise<std_msgs::Bool>("/autoreturn/return", 10);
}

bool DroneObjectRos::takeOff() {
  if (is_flying)
    return false;

  takeoff_pub.publish(std_msgs::Empty());
  is_flying = true;

  return true;
}

bool DroneObjectRos::land() {
  if (!is_flying)
    return false;

  land_pub.publish(std_msgs::Empty());
  is_flying = false;

  return true;
}

bool DroneObjectRos::hover() {
  if (!is_flying)
    return false;

  twist_msg.linear.x = 0;
  twist_msg.linear.y = 0;
  twist_msg.linear.z = 0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  cmd_pub.publish(twist_msg);

  return true;
}

bool DroneObjectRos::posCtrl(bool on) {
  if (!is_flying)
    return false;

  is_posctrl = on;
  std_msgs::Bool bool_msg;
  bool_msg.data = on ? 1 : 0;

  pos_ctrl_pub.publish(bool_msg);

  if (on)
    ROS_INFO("Switching position control on...");
  else
    ROS_INFO("Switching position control off...");

  return true;
}

bool DroneObjectRos::setVelocityMode(bool on) {
  if (!is_flying)
    return false;

  is_vel_mode = on;
  std_msgs::Bool bool_msg;
  bool_msg.data = on ? 1 : 0;

  vel_mode_pub.publish(bool_msg);

  if (on)
    ROS_INFO("Switching velocity mode on...");
  else
    ROS_INFO("Switching velocity mode off...");

  return true;
}

bool DroneObjectRos::setHover(int set_hover) {
  std_msgs::Bool cmd_sethover;

  if (set_hover == 1) {
    cmd_sethover.data = true;
    is_hover = true;
    is_set_point = false;
    is_set_trajectory = false;
    is_auto_return = false;
  } else {
    cmd_sethover.data = false;
    is_hover = false;
  }

  sethover_pub.publish(cmd_sethover);
  return is_hover;
}

bool DroneObjectRos::setPoint(float_t x, float_t y, float_t z, float_t set_yaw, float_t seconds) {
  if (ros::ok()) {
    controller_setpoint.data.push_back(x);
    controller_setpoint.data.push_back(-y);  // change into NEU
    controller_setpoint.data.push_back(z);
    controller_setpoint.data.push_back(set_yaw);
    controller_setpoint.data.push_back(seconds);
    setpoint_pub.publish(controller_setpoint);
    controller_setpoint.data.clear();
  } else {
    return false;
  }

  return true;
}

bool DroneObjectRos::setTrajectory(int set_traject) {
  std_msgs::Bool cmd_settrajectory;

  if (set_traject == 1) {
    cmd_settrajectory.data = true;
    is_set_trajectory = true;
  } else {
    cmd_settrajectory.data = false;
    is_set_trajectory = false;
  }

  control_settrajectory_pub.publish(cmd_settrajectory);

  return is_set_trajectory;
}

bool DroneObjectRos::setAutoreturnRecord(bool set_record) {
  std_msgs::Bool record;
  record.data = set_record;

  if (ros::ok())
    autoreturn_startRecord_pub.publish(record);
  else
    return false;

  return true;
}

bool DroneObjectRos::setAutoreturnReturn(bool set_return) {
  std_msgs::Bool ret;
  ret.data = set_return;
  is_auto_return = set_return;

  if (ros::ok())
    autoreturn_startReturn_pub.publish(ret);
  else
    return false;

  return true;
}

bool DroneObjectRos::hoverMove(float x, float y, float z, float yaw_speed) {
  if (!is_flying)
    return false;

  twist_msg.linear.x = x;
  twist_msg.linear.y = y;
  twist_msg.linear.z = z;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.z = yaw_speed;

  hover_cmd_pub.publish(twist_msg);

  return true;
}

bool DroneObjectRos::move(float lr, float fb, float ud, float w) {
  if (!is_flying)
    return false;
 
  if ((lr == 0) && (fb == 0) && (ud == 0) && (w == 0)) {
    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
  } else {
    twist_msg.linear.x = 1.0;
    twist_msg.linear.y = 1.0;
    twist_msg.linear.z = ud;
    twist_msg.angular.x = lr;
    twist_msg.angular.y = fb;
    twist_msg.angular.z = w;
  }

  cmd_pub.publish(twist_msg);

  return true;
}

bool DroneObjectRos::moveTo(float x, float y, float z) {
  if (!is_flying)
    return false;

  twist_msg.linear.x = x;
  twist_msg.linear.y = y;
  twist_msg.linear.z = z;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.z = 0;

  cmd_pub.publish(twist_msg);

  return true;
}

bool DroneObjectRos::pitch(float speed) {
  if (!is_flying)
    return false;

  twist_msg.linear.x = 1.0;
  twist_msg.linear.y = 1.0;
  twist_msg.linear.z = 0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = speed;
  twist_msg.angular.z = 0.0;

  cmd_pub.publish(twist_msg);

  ROS_INFO("Pitching...");

  return true;
}

bool DroneObjectRos::roll(float speed) {
  if (!is_flying)
    return false;

  twist_msg.linear.x = 1.0;
  twist_msg.linear.y = 1.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = speed;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;
  cmd_pub.publish(twist_msg);

  ROS_INFO("Rolling...");

  return true;
}

bool DroneObjectRos::rise(float speed) {
  if (!is_flying)
    return false;

  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = speed;
  twist_msg.angular.x = 0.0;  // flag for preventing hovering
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  cmd_pub.publish(twist_msg);

  ROS_INFO("Rising...");

  return true;
}

bool DroneObjectRos::yaw(float speed) {
  if (!is_flying)
    return true;

  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;  // flag for preventing hovering
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = speed;

  cmd_pub.publish(twist_msg);

  ROS_INFO("Turning head...");

  return true;
}
