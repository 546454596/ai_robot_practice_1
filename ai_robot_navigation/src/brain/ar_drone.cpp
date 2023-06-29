#include "ar_drone.h"

#include <cstdio>
#include <string>

bool ArDrone::setup() {
  pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
  pub_empty_takeoff = nh.advertise<std_msgs::Empty>("/ArDrone/takeoff", 1); 
  pub_empty_land = nh.advertise<std_msgs::Empty>("/ArDrone/land", 1);  
  pub_empty_reset = nh.advertise<std_msgs::Empty>("/ArDrone/reset", 1); 

  twist_msg_hover.linear.x=0.0;
  twist_msg_hover.linear.y=0.0;
  twist_msg_hover.linear.z=0.0;
  twist_msg_hover.angular.x=0.0;
  twist_msg_hover.angular.y=0.0;
  twist_msg_hover.angular.z=0.0;

  return true;
}

bool ArDrone::open() {
  this->is_flying = false;
  int err;

  // Open a dgram(UDP) socket 
  this->control_client_ = socket(AF_INET, SOCK_DGRAM, 0);
  this->nav_client_ = socket(AF_INET, SOCK_DGRAM, 0);

  struct timeval tv;

  tv.tv_sec = 5; /* 30 Secs Timeout */
  tv.tv_usec = 0;  // Not init'ing this can cause strange errors

  setsockopt(this->nav_client_, SOL_SOCKET, SO_RCVTIMEO, (char *) &tv, sizeof(struct timeval));


  if (this->control_client_ < 0) {
    ROS_ERROR("Could not create socket");
    return false;
  }
  if (this->nav_client_ < 0) {
    ROS_ERROR("Erorr: Could not create socket");
    return false;
  }

  //Binding a socket with 192.168.1.1
  struct sockaddr_in sin, sin_navdata;
  sin.sin_port = htons(5556);
  sin.sin_addr.s_addr = inet_addr("192.168.1.1");
  sin.sin_family = AF_INET;

  sin_navdata.sin_port = htons(5554);
  sin_navdata.sin_addr.s_addr = inet_addr("192.168.1.1");
  sin_navdata.sin_family = AF_INET;

  if (connect(this->control_client_, (sockaddr*) &sin, sizeof(sin)) < 0) {
    ROS_ERROR("Sockcet connection failed!");
    return false;
  }
  if (connect(this->nav_client_, (sockaddr*) &sin_navdata, sizeof(sin_navdata)) < 0) {
    ROS_ERROR("Socket connection Failed");
    return false;
  }

  this->control_sequence_ = 1;
  sprintf(this->command_, "AT*CONFIG=%d,\"video:codec_fps\",\"30\"\r", this->control_sequence_);
  if (ArDrone::sendControlData(this->command_) == false)
    return false;

  ROS_INFO("Connecting to ArDrone...");

  if (!ArDrone::recvNav())
    return false;

  ROS_INFO("Connect to ArDrone successfully!");

  pokeNavPort();
  initNavPort();

  return true;
}

void ArDrone::shutdown(void) {
  close(this->control_client_);
}

bool ArDrone::sendControlData(std::string data) {
  
  int ret = send(this->control_client_, data.data(), data.length(), 0);
  this->control_sequence_++;

  if (ret < 0) {
    ROS_ERROR("error in sending control data!");
    return false;
  }

  return true;
}

bool ArDrone::pokeNavPort() {
  sprintf(this->command_, "\x01");
  std::string command_data = this->command_;
  return send(this->nav_client_, command_data.data(), command_data.length(), 0);
}

bool ArDrone::initNavPort() {
  sprintf(this->command_, "AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r", this->control_sequence_);
  if (ArDrone::sendControlData(this->command_) == false) {
    return false;
  }

  sprintf(this->command_, "AT*CTRL=%d,0\r", this->control_sequence_);
  if (ArDrone::sendControlData(this->command_) == false) {
    return false;
  }

  sprintf(this->command_, "AT*FTRIM=%d,\r", this->control_sequence_);
  if (ArDrone::sendControlData(this->command_) == false) {
    return false;
  }
  return true;
}

bool ArDrone::recvNav() {
  resetWatchdog();
  pokeNavPort();
  memset(navdata, 0, sizeof(navdata));
  if (recv(this->nav_client_, (char*) &navdata, sizeof(navdata), 0) > 0) {
    memcpy(nav.raw_data, navdata, sizeof(navdata));
    nav.setOptions();
    return true;
  } else {
    return false;
  }
}

bool ArDrone::sendWatchdogReset() {
  sprintf(this->command_, "AT*COMWDG=%d\r", this->control_sequence_);
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendTrim() {
  sprintf(this->command_, "AT*FTRIM=%d\r", this->control_sequence_);
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendTakeOff() {
  this->is_flying = true;
  sprintf(this->command_, "AT*REF=%d,290718208\r", this->control_sequence_);
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendLand() {
  this->is_flying = false;
  sprintf(this->command_, "AT*REF=%d,290717696\r", this->control_sequence_);
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendEmergency() {
  sprintf(this->command_, "AT*REF=%d,290717952\r", this->control_sequence_);
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendResumeNormal() {
  sprintf(this->command_, "AT*REF=%d,290717696\rAT*REF=%d,290717952\r",
          this->control_sequence_, this->control_sequence_ + 1);
  this->control_sequence_++;
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendHover() {
  sprintf(this->command_, "AT*PCMD=%d,0,0,0,0,0\r", this->control_sequence_);
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendMove(float v_lr = 0, float v_fb = 0, float v_du = 0, float w_lr = 0) {
  v_lr = (v_lr < 1) ? (v_lr > -1 ? v_lr : -1) : 1;
  v_fb = (v_fb < 1) ? (v_fb > -1 ? v_fb : -1) : 1;
  v_du = (v_du < 1) ? (v_du > -1 ? v_du : -1) : 1;
  w_lr = (w_lr < 1) ? (w_lr > -1 ? w_lr : -1) : 1;

  sprintf(this->command_, "AT*PCMD=%d,1,%d,%d,%d,%d\r", this->control_sequence_, *(int*)(&v_lr), *(int*)(&v_fb),
          *(int*)(&v_du), *(int*)(&w_lr));
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendMoveLeft(float speed) {
  sprintf(this->command_, "AT*PCMD=%d,1,%d,0,0,0\r", this->control_sequence_, *(int*) (&speed)); 
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendMoveRight(float speed) {
  sprintf(this->command_, "AT*PCMD=%d,1,%d,0,0,0\r", this->control_sequence_, *(int*)(&speed)); 
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendMoveForward(float speed) {
  sprintf(this->command_, "AT*PCMD=%d,1,0,%d,0,0\r", this->control_sequence_, *(int*)(&speed));
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendMoveBackward(float speed) {
  sprintf(this->command_, "AT*PCMD=%d,1,0,%d,0,0\r", this->control_sequence_, *(int*)(&speed));
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendMoveUp(float speed) {
  sprintf(this->command_, "AT*PCMD=%d,1,0,0,%d,0\r", this->control_sequence_, *(int*)(&speed));
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendMoveDown(float speed) {
  sprintf(this->command_, "AT*PCMD=%d,1,0,0,%d,0\r", this->control_sequence_, *(int*)(&speed));
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendTurnLeft(float speed) {
  sprintf(this->command_, "AT*PCMD=%d,1,0,0,0,%d\r", this->control_sequence_, *(int*)(&speed));
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendTurnRight(float speed) {
  sprintf(this->command_, "AT*PCMD=%d,1,0,0,0,%d\r", this->control_sequence_, *(int*)(&speed));
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendSelectFrontCamera() {
  sprintf(this->command_, "AT*ZAP=%i,0\r", this->control_sequence_);
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::sendSelectVerticalCamera() {
  sprintf(this->command_, "AT*ZAP=%i,2\r", this->control_sequence_);
  return ArDrone::sendControlData(this->command_);
}

bool ArDrone::resetWatchdog() {
  return true;
}

bool ArDrone::takeOff() {
  if (this->is_flying)
    return false;

  bool result = true;

  this->pub_empty_takeoff.publish(this->emp_msg); //launches the drone
  this->pub_twist.publish(this->twist_msg_hover); //drone is flat

  ROS_INFO("Taking off");

  ros::spinOnce();

  this->is_flying = true;

  return result;
}

bool ArDrone::land() {
  if (!this->is_flying)
    return false;

  this->pub_twist.publish(this->twist_msg_hover); //drone is flat
  this->pub_empty_land.publish(this->emp_msg); //lands the drone

  ROS_INFO("Landing");

  ros::spinOnce();

  this->is_flying = false;

  return true;
}

bool ArDrone::hover() {
  if (!this->is_flying)
    return false;

  this->pub_twist.publish(this->twist_msg_hover); //drone is flat

  ROS_INFO("Hovering");

  ros::spinOnce();

  return true;
}

bool ArDrone::move(float lr, float fb, float ud, float w) {
  if (!this->is_flying)
    return false;

  this->twist_msg.linear.x = fb;
  this->twist_msg.linear.y = -lr;
  this->twist_msg.linear.z = ud;
  this->twist_msg.angular.x = 0.0;
  this->twist_msg.angular.y = 0.0;
  this->twist_msg.angular.z = w;
  this->pub_twist.publish(this->twist_msg);

  return true;
}

bool ArDrone::moveLeft(float speed) {
  if (!this->is_flying)
    return false;

  this->twist_msg.linear.x = 0.0;
  this->twist_msg.linear.y = speed;
  this->twist_msg.linear.z = 0.0;
  this->twist_msg.angular.x = 0.0;
  this->twist_msg.angular.y = 0.0;
  this->twist_msg.angular.z = 0.0;
  this->pub_twist.publish(this->twist_msg);

  ROS_INFO("Moving");

  return true;
}

bool ArDrone::moveRight(float speed) {
  if (!this->is_flying)
    return false;

  this->twist_msg.linear.x = 0.0;
  this->twist_msg.linear.y = -speed;
  this->twist_msg.linear.z = 0.0;
  this->twist_msg.angular.x = 0.0;
  this->twist_msg.angular.y = 0.0;
  this->twist_msg.angular.z = 0.0;
  this->pub_twist.publish(this->twist_msg);

  ROS_INFO("Moving");

  return true;
}

bool ArDrone::moveForward(float speed) {
  if (!this->is_flying)
    return false;

  this->twist_msg.linear.x = speed;
  this->twist_msg.linear.y = 0.0;
  this->twist_msg.linear.z = 0.0;
  this->twist_msg.angular.x = 0.0;
  this->twist_msg.angular.y = 0.0;
  this->twist_msg.angular.z = 0.0;
  this->pub_twist.publish(this->twist_msg);

  ROS_INFO("Moving");

  return true;
}

bool ArDrone::moveBackward(float speed) {
  if (!this->is_flying)
    return false;

  this->twist_msg.linear.x = -speed;
  this->twist_msg.linear.y = 0.0;
  this->twist_msg.linear.z = 0.0;
  this->twist_msg.angular.x = 0.0;
  this->twist_msg.angular.y = 0.0;
  this->twist_msg.angular.z = 0.0;
  this->pub_twist.publish(this->twist_msg);

  ROS_INFO("Moving");

  return true;
}

bool ArDrone::moveUp(float speed) {
  if (!this->is_flying)
    return false;

  this->twist_msg.linear.x = 0.0;
  this->twist_msg.linear.y = 0.0;
  this->twist_msg.linear.z = speed;
  this->twist_msg.angular.x = 0.0;
  this->twist_msg.angular.y = 0.0;
  this->twist_msg.angular.z = 0.0;
  this->pub_twist.publish(this->twist_msg);

  ROS_INFO("Moving");

  return true;
}

bool ArDrone::moveDown(float speed) {
  if (!this->is_flying)
    return false;

  this->twist_msg.linear.x = 0.0;
  this->twist_msg.linear.y = 0.0;
  this->twist_msg.linear.z = -speed;
  this->twist_msg.angular.x = 0.0;
  this->twist_msg.angular.y = 0.0;
  this->twist_msg.angular.z = 0.0;
  this->pub_twist.publish(this->twist_msg);

  ROS_INFO("Moving");

  return true;
}

bool ArDrone::turnLeft(float speed) {
  if (!this->is_flying)
    return false;

  this->twist_msg.linear.x = 0.0;
  this->twist_msg.linear.y = 0.0;
  this->twist_msg.linear.z = 0.0;
  this->twist_msg.angular.x = 0.0;
  this->twist_msg.angular.y = 0.0;
  this->twist_msg.angular.z = speed;
  this->pub_twist.publish(this->twist_msg);

  ROS_INFO("Moving");

  return true;
}

bool ArDrone::turnRight(float speed) {
  if (!this->is_flying)
    return false;

  this->twist_msg.linear.x = 0.0;
  this->twist_msg.linear.y = 0.0;
  this->twist_msg.linear.z = 0.0;
  this->twist_msg.angular.x = 0.0;
  this->twist_msg.angular.y = 0.0;
  this->twist_msg.angular.z = -speed;
  this->pub_twist.publish(this->twist_msg);

  ROS_INFO("Moving");

  return true;
}

bool ArDrone::selectFrontCamera() {
  return true;
}

bool ArDrone::selectVerticalCamera() {
  return true;
}

bool ArDrone::resumeNormal() {
  this->is_flying = false;
  return true;
}

bool ArDrone::emergency() {
  return true;
}
