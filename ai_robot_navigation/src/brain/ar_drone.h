#ifndef AI_ROBOT_NAVIGATION_ARDRONE_H_
#define AI_ROBOT_NAVIGATION_ARDRONE_H_

#include <unistd.h>

#include <arpa/inet.h>
#include <geometry_msgs/Twist.h>
#include <netdb.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "nav_data.h"

class ArDrone {
 public:
  ArDrone(const ros::NodeHandle& nh) : is_flying(false) {}
  ~ArDrone() {}

  bool emergency();
  bool hover();
  bool land();
  bool move(float v_lr, float v_fb, float v_ud, float w_lr);
  bool moveLeft(float speed = -0.2);
  bool moveRight(float speed = 0.2);
  bool moveForward(float speed = 0.1);
  bool moveBackward(float speed = 0.1);
  bool moveUp(float speed = 0.1);
  bool moveDown(float speed = 0.1);
  bool open();
  bool pokeNavPort();
  bool recvNav();
  bool resetWatchdog();
  bool resumeNormal();
  bool selectFrontCamera();
  bool selectVerticalCamera();
  bool setup();
  bool takeOff();
  bool turnLeft(float speed = 0.1);
  bool turnRight(float speed = 0.1);
  void shutdown();

public:
  char navdata[2048];
  bool is_flying;
  int frames;
  NavData nav;

  geometry_msgs::Twist twist_msg;
  geometry_msgs::Twist twist_msg_hover;
  geometry_msgs::Twist twist_msg_pshover;
  std_msgs::Empty emp_msg;

  ros::NodeHandle nh;
  ros::Publisher pub_empty_land; 
  ros::Publisher pub_empty_takeoff;
  ros::Publisher pub_empty_reset;
  ros::Publisher pub_twist;

protected:
  bool initNavPort();
  bool sendControlData(std::string data);
  bool sendEmergency();
  bool sendLand();
  bool sendHover();
  bool sendMove(float lr, float fb, float ud, float w);
  bool sendMoveLeft(float speed);
  bool sendMoveRight(float speed);
  bool sendMoveForward(float speed);
  bool sendMoveBackward(float speed);
  bool sendMoveUp(float speed);
  bool sendMoveDown(float speed);
  bool sendResumeNormal();
  bool sendTakeOff();
  bool sendTrim();
  bool sendTurnLeft(float speed);
  bool sendTurnRight(float speed);
  bool sendSelectFrontCamera();
  bool sendSelectVerticalCamera();
  bool sendWatchdogReset();

protected:
  char command_[100];
  int control_client_;
  int control_sequence_;
  int nav_client_;
};

#endif  // AI_ROBOT_NAVIGATION_ARDRONE_H_
