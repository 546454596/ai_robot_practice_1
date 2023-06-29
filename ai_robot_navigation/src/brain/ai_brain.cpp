#include "ai_brain.h"
#include "std_msgs/UInt8.h"

#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

constexpr float kButtonHoldThresh = 0.5;
constexpr float kHandleHoldThresh = 0.01;

static inline bool checkHandleHoldThresh(float input) {
  return fabs(input) > kHandleHoldThresh;
}

AiBrain::AiBrain(const ros::NodeHandle& nh)
    : findpath_srm_(nh, true),
      findpath_status_(FindPathStatus::kNotStarted),
      is_controller_input_(false),
      is_findpath_test_mode_(false),
      is_hover_stable_(true),
      motion_status_(MotionStatus::kUnlocked),
      nh_(nh),
      p2p_(nh),
      p2p_status_(P2PStatus::kNotStarted),
      pid_time_(ros::Time::now().toSec()),
      posctl_(nh),
      pose_xyz_dest_{0, 0, 2},
      pose_xyz_tgt_{0, 0, 0},
      tasknum_before_pause_(-1),
      time_to_pause_(50),
      t_worldslam_{0, 0, 0},
      yaw_worldslam_(0) {
    float q[4] = {1, 0, 0, 0};
    qToRotation(q, r_worldslam_);
    qToEuler(yaw_worldslam_, yaw_worldslam_, yaw_worldslam_, q);
    initPublishersAndSubscribers();
    initServices();
}

AiBrain::~AiBrain() {}

void AiBrain::initPublishersAndSubscribers() {
  pause_pub_ = nh_.advertise<std_msgs::Bool>("/remote/pause_or_unlock", 1);
  joy_sub_ = nh_.subscribe("/joy", 1, &AiBrain::joyCallback, this);
  pose_sub_ = nh_.subscribe("/est_pose", 1, &AiBrain::poseCallback, this);
  vel_sub_ = nh_.subscribe("/est_vel", 1, &AiBrain::velocityCallback, this);
  qt_slam_world_sub_ = nh_.subscribe("/qt_worldslam_", 1, &AiBrain::qtSlamWorldCallback, this);
  move_base_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &AiBrain::moveBaseGoalCallback, this);
  p2p_sub_ = nh_.subscribe("/point_to_point/path", 1, &AiBrain::p2pCallback, this);
  farp2p_sub_ = nh_.subscribe("/far_point_to_point/path", 1, &AiBrain::farp2pCallback, this);

  oa_abort_pub_ = nh_.advertise<std_msgs::Empty>("/oa_abort", 1);

  remote_joy_sub_ = nh_.subscribe("/remote/joy", 1, &AiBrain::joyCallback, this);
  remote_task_cancel_sub_ = nh_.subscribe("/remote/task_cancel", 1, &AiBrain::taskCancelCallback, this);
  remote_move_base_goal_sub_ = nh_.subscribe("/remote/move_base_simple/goal", 1, &AiBrain::moveBaseGoalCallback, this);
  remote_farp2p_sub_ = nh_.subscribe("/remote/farp2p/path", 1, &AiBrain::farp2pCallback, this);
  remote_goal_reached_pub_ = nh_.advertise<std_msgs::Int32>("/remote/goal_reached", 1);
}

void AiBrain::think() {
  double dt = ros::Time::now().toSec() - pid_time_.toSec();
  pid_time_ = ros::Time::now();
  givePosToFindPathTask();
  findpath_srm_.displayPointCloud();
  posctl_.setState(pose_xyz_now_, vel_xyz_now_, yaw_now_);

  switch (tasknum_) {
    case kHoverTask:
      doHoverTask(dt);
      break;
    case kTakeoffTask:
      doTakeoffTask();
      break;
    case kLandTask:
      doLandTask();
      break;
    case kMoveTask:
      doControllerTask(dt);
      break;
    case kFindPathTask:
      doFindPathTask(dt);
      break;
    case kP2PTask:
      doPointToPointTask(dt);
      break;
    case kFarP2PTask:
      doFarPointToPointTask(dt);
      break;
    default:
      doHoverTask(dt);
      break;
  }
  // std::cout << "******** Current tasknum: " << tasknum_ << '\n';
}

void AiBrain::doHoverTask(double dt) {
  if (last_tasknum_ != tasknum_) {
    ROS_INFO("[AiBrain] Hover!");
    last_tasknum_ = tasknum_;
    hover_start_time_ = ros::Time::now().toSec();
  }

  if (!is_hover_stable_) {
    if (sqrt((pow(vel_xyz_now_[0], 2) + pow(vel_xyz_now_[1], 2)) < 0.3)) {
      is_hover_stable_ = true;
      pose_xyz_tgt_[0] = pose_xyz_now_[0];
      pose_xyz_tgt_[1] = pose_xyz_now_[1];
      pose_xyz_tgt_[2] = pose_xyz_now_[2];
      yaw_tgt_ = yaw_now_;
      posctl_.doMove(dt, 0, 0, 0, 0);
    } else {
      posctl_.brake(dt);
    }
  } else {
    posctl_.doHover(dt, pose_xyz_tgt_, yaw_tgt_, pose_xyz_now_, vel_xyz_now_, yaw_now_);
  }

  if (is_findpath_test_mode_)
    doFindPathTask(dt);

  // Display position in slam
  float pos_slam[3];
  transformBodyFromNwuWorld(pos_slam[0], pos_slam[1], pos_slam[2], pose_xyz_tgt_[0], pose_xyz_tgt_[1], pose_xyz_tgt_[2],
                            r_worldslam_, t_worldslam_);
  printf("Target pos(%f,%f,%f)(%f,%f,%f)\r", pose_xyz_tgt_[0], pose_xyz_tgt_[1], pose_xyz_tgt_[2],
           pos_slam[0], pos_slam[1], pos_slam[2]);
}

void AiBrain::doTakeoffTask() {
  if (last_tasknum_ != tasknum_) {
    ROS_INFO("[AiBrain] Takeoff!");
    last_tasknum_ = tasknum_;
  }
  yaw_tgt_ = yaw_now_;
  pose_xyz_tgt_[0] = pose_xyz_now_[0];
  pose_xyz_tgt_[1] = pose_xyz_now_[1];
  pose_xyz_tgt_[2] = 1.0;
  is_hover_stable_ = true;
  posctl_.takeOff();
  tasknum_ = kHoverTask;
}

void AiBrain::doLandTask() {
  if (last_tasknum_ != tasknum_) {
    ROS_INFO("[AiBrain] Land!");
    last_tasknum_ = tasknum_;
  }
  posctl_.land();
  tasknum_ = kHoverTask;
  is_hover_stable_ = true;
}

void AiBrain::doControllerTask(double dt) {
  if (last_tasknum_ != tasknum_) {
    ROS_INFO("[AiBrain] Move!");
    last_tasknum_ = tasknum_;
  }
  posctl_.doMove(dt, forwardback_ctl_, leftright_ctl_,
                   updown_ctl_, yaw_ctl_);

  if (is_findpath_test_mode_) {
    doFindPathTask(dt);
  }

  if (fabs(forwardback_ctl_) + abs(leftright_ctl_) +
    fabs(updown_ctl_) + fabs(yaw_ctl_) < 0.01) {
    tasknum_ = kHoverTask;
  }
}

void AiBrain::doPointToPointTask(double dt) {
  if (last_tasknum_ != tasknum_) {
    ROS_INFO("[AiBrain] start point to point!");
    // state_ptop = 0;
    last_tasknum_ = tasknum_;
  }

  if (p2p_status_ == P2PStatus::kStarted) {
    float v[4], t[3] = {0, 0, 0};
    float pos_slam[3];
    transformBodyFromNwuWorld(pos_slam[0], pos_slam[1], pos_slam[2],
                              pose_xyz_now_[0], pose_xyz_now_[1],
                              pose_xyz_now_[2], r_worldslam_,
                              t_worldslam_);
    if (p2p_.getTargetSpeed(pos_slam[0], pos_slam[1], pos_slam[2], yaw_now_ - yaw_worldslam_, v[0], v[1], v[2], v[3])) {
      transformNwuWorldFromBody(v[0], v[1], v[2], pos_slam[0], pos_slam[1], pos_slam[2], r_worldslam_, t);
        // posctl_.doMoveInWorld(dt, pos_slam[0], pos_slam[1],
        // pos_slam[2], _v[3]);
    } else {
      p2p_status_ = P2PStatus::kDone;
      doHoverTask(dt);
    }
  } else if (p2p_status_ == P2PStatus::kNotStarted) {
    if (!p2p_.readWayPoints()) {
      p2p_status_ = P2PStatus::kDone;
    } else {
      p2p_status_ = P2PStatus::kStarted;
    }
    doHoverTask(dt);
  } else if (p2p_status_ == P2PStatus::kDone) {
    p2p_status_ = P2PStatus::kNotStarted;
    tasknum_ = kHoverTask;
    ROS_INFO("[AiBrain] destination reached!");
    doHoverTask(dt);
  }
}

void AiBrain::doFarPointToPointTask(double dt) {
  if (last_tasknum_ != tasknum_) {
    ROS_INFO("[AiBrain] start far point to point!");
    last_tasknum_ = tasknum_;
  }

  if (cntdown_ > 0) {
    --cntdown_;
    ROS_INFO("\rwait count down %.0f second.", 1.0 * cntdown_ / 50);
    doHoverTask(dt);
    return;
  }

  if (farp2p_status_ == FarP2PStatus::kStarted) {
    if (findpath_status_ == FindPathStatus::kReached) {
      // next point
      ++farp2p_point_i_;
      if (farp2p_point_i_ >= farp2p_path_.poses.size()) { // final target point
        farp2p_status_ = FarP2PStatus::kDone;
        return;
      }

      if (farp2p_point_i_ != 0) { // not final target point
        publishGoalReached();
        cntdown_ = time_to_pause_;
      }

      is_hover_stable_ = false;
      ROS_INFO("[FarP2P] to the %d point and pause %f second.", farp2p_point_i_, time_to_pause_ * 1.0 / 50);
      transformNwuWorldFromBody(farp2p_path_.poses[farp2p_point_i_].pose.position.x,
                                farp2p_path_.poses[farp2p_point_i_].pose.position.y,
                                farp2p_path_.poses[farp2p_point_i_].pose.position.z,
                                pose_xyz_dest_[0], pose_xyz_dest_[1],
                                pose_xyz_dest_[2], r_worldslam_, t_worldslam_);

      findpath_status_ = FindPathStatus::kNotStarted;
    }
    doFindPathTask(dt);
  } else if (farp2p_status_ == FarP2PStatus::kNotStarted) {
    farp2p_point_i_ = -1;
    farp2p_status_ = FarP2PStatus::kStarted;
    findpath_status_ = FindPathStatus::kReached;
  } else if (farp2p_status_ == FarP2PStatus::kDone) {
    ROS_INFO("[AiBrain] far point to point task done!");
    publishGoalReached();
    farp2p_status_ = FarP2PStatus::kNotStarted;
    farp2p_path_.poses.clear();
    tasknum_ = kHoverTask;
    is_hover_stable_ = false;
    // doHoverTask(dt);
  } else if (farp2p_status_ == FarP2PStatus::kRemoteCancel) {
    ROS_INFO("[AiBrain] far point to point task cancelled!");
    farp2p_status_ = FarP2PStatus::kNotStarted;
    farp2p_path_.poses.clear();
    tasknum_ = kHoverTask;
    is_hover_stable_ = false;
  }
}

void AiBrain::doFindPathTask(double dt) {
  if (is_findpath_test_mode_ && findpath_status_ == FindPathStatus::kNotStarted) {
    ROS_INFO("[AiBrain] Start findpath in test mode, please control with joystick!");
  } else if (last_tasknum_ != tasknum_) {
    ROS_INFO("[AiBrain] Start findpath!");
    findpath_status_ = FindPathStatus::kNotStarted;
    last_tasknum_ = tasknum_;
  }

  float pos_slam[3], pos_des_slam[3];
  transformBodyFromNwuWorld(pos_slam[0], pos_slam[1], pos_slam[2], pose_xyz_now_[0], pose_xyz_now_[1], pose_xyz_now_[2],
                            r_worldslam_, t_worldslam_);
  transformBodyFromNwuWorld(pos_des_slam[0], pos_des_slam[1], pos_des_slam[2], pose_xyz_dest_[0], pose_xyz_dest_[1],
                            pose_xyz_dest_[2], r_worldslam_, t_worldslam_);
  if (findpath_status_ == FindPathStatus::kNotStarted) { // not started yet
    ROS_INFO("[FindPath] Findpath in progress.");
    findpath_srm_.resetAll(pos_slam[0], pos_slam[1], pos_slam[2], pos_des_slam[0], pos_des_slam[1], pos_des_slam[2]);

    if (findpath_srm_.findPath()) {
      ROS_INFO("[FindPath] Findpath done.");
      findpath_status_ = FindPathStatus::kOnTheMove;
    } else {
      findpath_status_ = FindPathStatus::kReached;
    }

    if (!is_findpath_test_mode_) {
      doHoverTask(dt);  // avoid no control input to drone
    }
  } else if (findpath_status_ == FindPathStatus::kFound) { // path found
    if (!is_findpath_test_mode_)
      doHoverTask(dt);
  } else if (findpath_status_ == FindPathStatus::kOnTheMove) { // on the move
    float v[4];
    float t[3] = {0, 0, 0};
    transformBodyFromNwuWorld(v[0], v[1], v[2], vel_xyz_now_[0], vel_xyz_now_[1], vel_xyz_now_[2], r_worldslam_, t);
    // the yaw may no longer be used
    if (findpath_srm_.getTargetSpeed(pos_slam[0], pos_slam[1], pos_slam[2], yaw_now_ - yaw_worldslam_, v[0], v[1], v[2],
                                     v[3])) {
        // speed just need rotate
        transformNwuWorldFromBody(v[0], v[1], v[2], pos_slam[0], pos_slam[1], pos_slam[2], r_worldslam_, t);
    } else {
      findpath_status_ = FindPathStatus::kReached;
      transformNwuWorldFromBody(v[0], v[1], v[2], pos_slam[0], pos_slam[1], pos_slam[2], r_worldslam_, t_worldslam_);
      pose_xyz_tgt_[0] = pos_slam[0];
      pose_xyz_tgt_[1] = pos_slam[1];
      pose_xyz_tgt_[2] = pos_slam[2];
      yaw_tgt_ = v[3] + yaw_worldslam_;
    }
  } else if (findpath_status_ == FindPathStatus::kReached || findpath_status_ == FindPathStatus::kRemoteCancel) { // to the end
    if (tasknum_ == kFindPathTask) {
      if (findpath_status_ == FindPathStatus::kRemoteCancel) {
        ROS_INFO("[AiBrain] Task canceled by remote client!");
      } else {
        publishGoalReached();
        ROS_INFO("[AiBrain] Move base simple task done!");
      }
    }

    findpath_status_ = FindPathStatus::kNotStarted;
    if (is_findpath_test_mode_)
      is_findpath_test_mode_ = false;

    tasknum_ = kHoverTask;
    is_hover_stable_ = false;
  }
}

void AiBrain::initServices() {
  set_dest_srv_ = nh_.advertiseService("/ai_robot_navigation/setdestination", &AiBrain::setDestinationCallback, this);
}

void AiBrain::givePosToFindPathTask() {
  float pos_slam[3];
  float r_bodyworld[9];
  Eigen::Matrix<float, 3, 3> r_slam_world, r_body_world, r_body_slam;

  transformBodyFromNwuWorld(pos_slam[0], pos_slam[1], pos_slam[2], pose_xyz_now_[0], pose_xyz_now_[1], pose_xyz_now_[2],
                            r_worldslam_, t_worldslam_);
  qToRotation(qwxyz_now_, r_bodyworld);
  r_body_world << r_bodyworld[0], r_bodyworld[1], r_bodyworld[2], r_bodyworld[3], r_bodyworld[4], r_bodyworld[5],
      r_bodyworld[6], r_bodyworld[7], r_bodyworld[8];
  r_slam_world << r_worldslam_[0], r_worldslam_[1], r_worldslam_[2], r_worldslam_[3], r_worldslam_[4], r_worldslam_[5],
      r_worldslam_[6], r_worldslam_[7], r_worldslam_[8];
  r_body_slam = r_slam_world.inverse() * r_body_world;
  float r_bodyslam[9] = {
    r_body_slam(0, 0), r_body_slam(0, 1), r_body_slam(0, 2),
    r_body_slam(1, 0), r_body_slam(1, 1), r_body_slam(1, 2),
    r_body_slam(2, 0), r_body_slam(2, 1), r_body_slam(2, 2)
  };
  float qbs[4];
  float r, p, y;

  rotationToQ(qbs, r_bodyslam);
  // problem with qbs
  qToEuler(r, p, y, qbs);

  findpath_srm_.setPosForVisual(pos_slam[0], pos_slam[1], pos_slam[2], y);
}

void AiBrain::qtSlamWorldCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  float ro, pi;
  float q[4] = {static_cast<float>(msg->pose.orientation.w), static_cast<float>(msg->pose.orientation.x),
      static_cast<float>(msg->pose.orientation.y), static_cast<float>(msg->pose.orientation.z)};

  t_worldslam_[0] = static_cast<float>(msg->pose.position.x);
  t_worldslam_[1] = static_cast<float>(msg->pose.position.y);
  t_worldslam_[2] = static_cast<float>(msg->pose.position.z);

  qToRotation(q, r_worldslam_);
  qToEuler(ro, pi, yaw_worldslam_, q);

  ROS_INFO("[AiBrain] Rotation and Transformation of slam and world obtained!");

  std::cout << "{" << r_worldslam_[0] << ", " << r_worldslam_[1] << ", " << r_worldslam_[2] << '\n' << r_worldslam_[3]
      << ", " << r_worldslam_[4] << ", " << r_worldslam_[5] << '\n' << r_worldslam_[6] << ", " << r_worldslam_[7] << ", "
      << r_worldslam_[8] << "}" << '\n'; }

void AiBrain::moveBaseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  ROS_INFO("[AiBrain] Set destination");

  if (msg->header.frame_id == "map") {
    float pos_slam[3];
    transformNwuWorldFromBody(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, pose_xyz_dest_[0],
                              pose_xyz_dest_[1], pose_xyz_dest_[2], r_worldslam_, t_worldslam_);
    transformBodyFromNwuWorld(pos_slam[0], pos_slam[1], pos_slam[2], pose_xyz_now_[0], pose_xyz_now_[1],
                              pose_xyz_now_[2], r_worldslam_, t_worldslam_);
    findpath_srm_.resetAll(pos_slam[0], pos_slam[1], pos_slam[2], msg->pose.position.x, msg->pose.position.y,
                           msg->pose.position.z);

    ROS_INFO("[AiBrain] Get (%f,%f,%f) in slam coordinate and set destination to (%f,%f,%f) in world.\n",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, pose_xyz_dest_[0], pose_xyz_dest_[1],
             pose_xyz_dest_[2]);

    tasknum_ = kFindPathTask;
    findpath_status_ = FindPathStatus::kNotStarted;
  } else if (msg->header.frame_id == "base_link") {
    float pos_slam[3], r_world_body[9], tarslam[3];
    // get self pose in slam
    transformBodyFromNwuWorld(pos_slam[0], pos_slam[1], pos_slam[2], pose_xyz_now_[0], pose_xyz_now_[1],
                              pose_xyz_now_[2], r_worldslam_, t_worldslam_);
    qToRotation(qwxyz_now_, r_world_body);
    // relative target from body to world
    transformNwuWorldFromBody(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, pose_xyz_dest_[0],
                              pose_xyz_dest_[1], pose_xyz_dest_[2], r_world_body, pose_xyz_now_);
    // target from world to slam
    transformBodyFromNwuWorld(
      tarslam[0], tarslam[1], tarslam[2], pose_xyz_dest_[0], pose_xyz_dest_[1], pose_xyz_dest_[2], r_worldslam_,
      t_worldslam_);
    // set target
    findpath_srm_.resetAll(pos_slam[0], pos_slam[1], pos_slam[2], tarslam[0], tarslam[1], tarslam[2]);
    tasknum_ = kFindPathTask;
    findpath_status_ = FindPathStatus::kNotStarted;

    ROS_INFO("[AiBrain] Get (%f,%f,%f) in body coordinate and set destination to (%f,%f,%f) in world, to (%f,%f,%f) in slam.\n",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, pose_xyz_dest_[0], pose_xyz_dest_[1],
             pose_xyz_dest_[2], tarslam[0], tarslam[1], tarslam[2]); }
}

void AiBrain::p2pCallback(const nav_msgs::Path::ConstPtr& msg) {
  p2p_.clear();

  for (int i = 0; i < msg->poses.size(); ++i)
    p2p_.addPoint(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z);

  tasknum_ = kP2PTask;
  p2p_status_ = P2PStatus::kStarted;

  ROS_INFO("[AiBrain] Set point to point ok.");
}

void AiBrain::farp2pCallback(const nav_msgs::Path::ConstPtr& msg) {
  farp2p_path_ = std::move(*msg);
  cntdown_ = 0;
  for (int i = 0; i < farp2p_path_.poses.size(); ++i)
    ROS_INFO("~aypoint %d (%f,%f,%f)\n", i, farp2p_path_.poses[i].pose.position.x,
             farp2p_path_.poses[i].pose.position.y, farp2p_path_.poses[i].pose.position.z);

  ROS_INFO("[AiBrain] Set far point to point ok.");
  tasknum_ = kFarP2PTask;
  farp2p_status_ = FarP2PStatus::kNotStarted;
}

void AiBrain::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  pose_xyz_now_[0] = msg->pose.position.x;
  pose_xyz_now_[1] = msg->pose.position.y;
  pose_xyz_now_[2] = msg->pose.position.z;
  qwxyz_now_[0] = msg->pose.orientation.w;
  qwxyz_now_[1] = msg->pose.orientation.x;
  qwxyz_now_[2] = msg->pose.orientation.y;
  qwxyz_now_[3] = msg->pose.orientation.z;
  qToEuler(roll_now_, pitch_now_, yaw_now_, qwxyz_now_);
}

void AiBrain::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  vel_xyz_now_[0] = msg->twist.linear.x;
  vel_xyz_now_[1] = msg->twist.linear.y;
  vel_xyz_now_[2] = msg->twist.linear.z;
}

// Note: BTP-2185T2 joystick must be set to correct mode, i.e. green light lit at second quadrant
void AiBrain::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  if (msg->buttons[0] > kButtonHoldThresh && motion_status_ != MotionStatus::kLocked) {  // Button A - pause and lock
    if (tasknum_ == kFarP2PTask || tasknum_ == kFindPathTask)
      tasknum_before_pause_ = tasknum_;

    std_msgs::Bool msg;
    msg.data = true;
    pause_pub_.publish(msg);
    tasknum_ = kLandTask;
    is_hover_stable_ = false;
    motion_status_ = MotionStatus::kLocked;

    ROS_INFO("[AiBrain] Robot move locked with tasknum %d!", tasknum_before_pause_);

    if (is_findpath_test_mode_) {
      is_findpath_test_mode_ = false;
      ROS_INFO("[AiBrain] Exit findpath test mode!");
    }
  } else if (msg->buttons[3] > kButtonHoldThresh && motion_status_ == MotionStatus::kLocked) { // Button Y - unlock
    ROS_INFO("[AiBrain] Robot move unlocked with tasknum %d!", tasknum_);
    std_msgs::Bool msg;
    msg.data = false;
    pause_pub_.publish(msg);
    tasknum_ = kTakeoffTask;
    is_hover_stable_ = false;
    motion_status_ = MotionStatus::kUnlocked;
    if (is_findpath_test_mode_) {
      is_findpath_test_mode_ = false;
      ROS_INFO("[AiBrain] Exit findpath test mode!");
    }
  } else if (msg->buttons[1] > kButtonHoldThresh && motion_status_ == MotionStatus::kUnlocked) { // Button B - resume after unlock
    // if (tasknum_before_pause_ == kFarP2PTask) {
    //   tasknum_ = kFarP2PTask;
    // } else {
    //   tasknum_ = kFindPathTask;
    if (tasknum_before_pause_ != -1) {
      tasknum_ = tasknum_before_pause_;
      ROS_INFO("[AiBrain] Robot move resumed with tasknum %d!", tasknum_);
      is_hover_stable_ = false;
      motion_status_ = MotionStatus::kResumed;
    }
  } // else if (msg->buttons[2] > kButtonHoldThresh) { // Button X
  //   tasknum_ = kP2PTask;
  //   is_hover_stable_ = false;
  // }
#ifdef FINDPATH_TEST_MODE
  else if (msg->buttons[4] > kButtonHoldThresh) { // button LB -- enter findpath test mode
    is_findpath_test_mode_ = !is_findpath_test_mode_;
    findpath_status_ = FindPathStatus::kNotStarted;
    ROS_INFO("[AiBrain] Enter findpath test mode!");
  }
#endif

  leftright_ctl_ = msg->axes[0];
  forwardback_ctl_ = msg->axes[1];
  yaw_ctl_ = msg->axes[3];
  updown_ctl_ = msg->axes[4];

  if (checkHandleHoldThresh(leftright_ctl_)
    || checkHandleHoldThresh(yaw_ctl_)
    || checkHandleHoldThresh(updown_ctl_)
    || checkHandleHoldThresh(forwardback_ctl_)) {
    tasknum_ = kMoveTask;  // for safety
    is_controller_input_ = true;
    is_hover_stable_ = false;
  }
}

void AiBrain::taskCancelCallback(std_msgs::Empty::ConstPtr msg) {
  ROS_INFO("[taskcancel] Task required to cancel");
  abortObstacleAvoidance();
  tasknum_ = kLandTask;
  is_hover_stable_ = false;
}

// Do not publish empty string, use service instead
void AiBrain::abortObstacleAvoidance() {
  std_msgs::Empty msg;
  oa_abort_pub_.publish(msg);
}

void AiBrain::publishGoalReached() {
  std_msgs::Int32 n;
  if (tasknum_ == kFarP2PTask && farp2p_point_i_ != 0)
    n.data = farp2p_point_i_;
  else
    n.data = 1;
  remote_goal_reached_pub_.publish(n);
}

bool AiBrain::setDestinationCallback(ai_robot_interface::set_destination::Request& req,
                                     ai_robot_interface::set_destination::Response& res) {
  transformNwuWorldFromBody(req.x, req.y, req.z, pose_xyz_dest_[0], pose_xyz_dest_[1], pose_xyz_dest_[2], r_worldslam_,
                            t_worldslam_); float pos_slam[3];
  transformBodyFromNwuWorld(pos_slam[0], pos_slam[1], pos_slam[2], pose_xyz_now_[0], pose_xyz_now_[1], pose_xyz_now_[2],
                            r_worldslam_, t_worldslam_);
  findpath_srm_.resetAll(pos_slam[0], pos_slam[1], pos_slam[2], req.x, req.y, req.z);

  ROS_INFO("[AiBrain] Get (%f,%f,%f) in slam coordinate and set destination to (%f,%f,%f) in world.\n", req.x, req.y,
           req.z, pose_xyz_dest_[0], pose_xyz_dest_[1], pose_xyz_dest_[2]);

  res.set_ok = true;
  return true;
}
