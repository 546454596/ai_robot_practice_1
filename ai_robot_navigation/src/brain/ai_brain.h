#ifndef AI_ROBOT_NAVIGATION_BRAIN_AIBRAIN_H_
#define AI_ROBOT_NAVIGATION_BRAIN_AIBRAIN_H_

#include <cmath>

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Path.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

#include "ai_robot_interface/set_destination.h"
#include "findpath/findpath_srm.h"
#include "point_to_point/point_to_point.h"
#include "pos_control.h"
#include "pos_trajectory.h"
#include "utils/adrc.h"
#include "utils/math_aux.h"
#include "utils/pid_controller.h"
#include "utils/pid_fuzzy.h"

enum TaskNumber {
  kHoverTask = 0,
  kTakeoffTask,
  kLandTask,
  kMoveTask,
  kFindPathTask,
  kP2PTask,
  kFarP2PTask,
  kStopTask
};

enum class FindPathStatus {
  kNotStarted = 0,
  kFound, // path found
  kOnTheMove, // moving
  kReached, // goal reached
  kRemoteCancel // remote cancel
};

enum class MotionStatus {
  kLocked,
  kUnlocked,
  kResumed
};

enum class P2PStatus {
  kNotStarted = 0,
  kStarted,
  kDone
};

enum class FarP2PStatus {
  kNotStarted = 0,
  kStarted,
  kDone,
  kRemoteCancel
};

class AiBrain {
public:
  AiBrain(const ros::NodeHandle& nh);
  ~AiBrain();
  void think();

private:
  // Initiliazation
  void initPublishersAndSubscribers();
  void initServices();

  // Task handlers
  void doControllerTask(double dt);
  void doFarPointToPointTask(double dt);
  void doFindPathTask(double dt);
  void givePosToFindPathTask();
  void doHoverTask(double dt);
  void doLandTask();
  void doTakeoffTask();
  void doPointToPointTask(double dt);
  inline bool tasknumberChanged() { return last_tasknum_ == tasknum_; } 

  // Subscriber callbacks
  void axes0lrCallback(const std_msgs::Float64::ConstPtr& msg);
  void axes0udCallback(const std_msgs::Float64::ConstPtr& msg);
  void axes1lrCallback(const std_msgs::Float64::ConstPtr& msg);
  void axes1udCallback(const std_msgs::Float64::ConstPtr& msg);
  void btn1Callback(const std_msgs::Bool::ConstPtr& msg);
  void btn2Callback(const std_msgs::Bool::ConstPtr& msg);
  void btn3Callback(const std_msgs::Bool::ConstPtr& msg);
  void btn4Callback(const std_msgs::Bool::ConstPtr& msg);
  void btn5Callback(const std_msgs::Bool::ConstPtr& msg);
  void farp2pCallback(const nav_msgs::Path::ConstPtr& msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void moveBaseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void p2pCallback(const nav_msgs::Path::ConstPtr& msg);
  void qtSlamWorldCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  
  // Service callbacks
  bool setDestinationCallback(ai_robot_interface::set_destination::Request& req,
                              ai_robot_interface::set_destination::Response& res);

  // For remote control
  void taskCancelCallback(std_msgs::Empty::ConstPtr msg);
  void publishGoalReached();
  void abortObstacleAvoidance();

private:
  ros::NodeHandle nh_;
  ros::Time pid_time_;

  FindPathSRM findpath_srm_;
  PosControl posctl_;
  Point2Point p2p_;

  // Task number
  int last_tasknum_;
  int tasknum_;

  // Controller
  bool is_controller_input_;
  double forwardback_ctl_;
  double leftright_ctl_;
  double yaw_ctl_;
  double updown_ctl_;

  // Pose
  float pitch_now_;
  float pose_xyz_now_[3];
  float pose_xyz_tgt_[3];
  float qwxyz_now_[4];
  float roll_now_;
  float vel_xyz_now_[3];
  float yaw_now_;
  float yaw_tgt_;

  // Status
  bool is_hover_stable_;
  bool is_findpath_test_mode_;
  double hover_start_time_;
  FindPathStatus findpath_status_;
  P2PStatus p2p_status_;
  FarP2PStatus farp2p_status_;
  MotionStatus motion_status_;

  // Regard slam as body frame to use math_aux
  // yawslam + yawworldslam = yawworld
  int cntdown_;
  int farp2p_point_i_;
  int time_to_pause_;
  float r_worldslam_[9];
  float t_worldslam_[3];
  float pose_xyz_dest_[3];
  float yaw_worldslam_;
  nav_msgs::Path farp2p_path_;
 
  // Publishers and subscribers
  ros::Publisher pause_pub_;
  ros::Publisher oa_abort_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber qt_slam_world_sub_;
  ros::Subscriber move_base_goal_sub_;
  ros::Subscriber p2p_sub_;
  ros::Subscriber farp2p_sub_;
  ros::Subscriber slampose_sub_;
  ros::Subscriber remote_joy_sub_;
  ros::Subscriber remote_task_cancel_sub_;
  ros::Subscriber remote_move_base_goal_sub_;
  ros::Subscriber remote_farp2p_sub_;
  ros::Publisher remote_goal_reached_pub_;
  // Services
  ros::ServiceServer set_dest_srv_;

  int tasknum_before_pause_;
};

#endif  // AI_ROBOT_NAVIGATION_BRAIN_AIBRAIN_H_
