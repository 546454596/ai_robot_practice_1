 // This is a stand alone obstacle avoidance module. when you publish a local target to it, it starts to move using
 // odometry. This is also a convient version of obstacle avoidance module. If you want an efficient obstacle avoidance
 // module, use the p3atObsAvoid.h directly in you program.
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include "brain/pioneer_3at.h"
#include "p3at_obsavoid.h"
#include "ros/forwards.h"
#include "sensor_msgs/Joy.h"

enum class MotionStatus {
  kLocked,
  kUnlocked
};

// Constants
constexpr float kButtonHoldThresh = 0.5;

// Globals
float tgt_pos[3] = {.0, .0, .0};         // x,y,z of target point, in odom frame
float tgt_orient[4] = {.0, .0, .0, 1.0};  // w,x,y,z of target point, in odom frame
float tgt_r[9] = {1.0, .0, .0, .0, 1.0, .0, .0, .0, 1.0};
float odom_pose[3] = {.0, .0, .0};      // body pose in odom frame
float odom_orient[4] = {.0, .0, .0, 1.0};  // body pose in odom frame
float odom_r[9] = {1.0, .0, .0, .0, 1.0, .0, .0, .0, 1.0};
bool final_orient_flag = false;
bool local_tgt_flag = false;
bool stop_flag = false;
P3atObstacleAvoidance *oa = nullptr;
Pioneer3at *drone = nullptr;
// MotionStatus motion_status = MotionStatus::kUnlocked;

// Transoform the input msg from body frame to odom frame.
void targetPointCallback(const geometry_msgs::PoseConstPtr& msg) {
  float tmp[3], tmp_r[9], tmp_rbt[9], tmp_rob[9];

  if (stop_flag)
    ROS_INFO("[OA] Obstacle avoidance is locked, press 'Y' to unlock!");

  tmp[0] = msg->position.x;
  tmp[1] = msg->position.y;
  tmp[2] = msg->position.z;

  qToRotation(odom_orient, tmp_r);
  transformNwuWorldFromBody(tmp[0], tmp[1], tmp[2], tgt_pos[0], tgt_pos[1], tgt_pos[2], tmp_r, odom_pose);

  tgt_orient[0] = msg->orientation.w;
  tgt_orient[1] = msg->orientation.x;
  tgt_orient[2] = msg->orientation.y;
  tgt_orient[3] = msg->orientation.z;

  if (fabs(tgt_orient[0]) + fabs(tgt_orient[1]) + fabs(tgt_orient[2]) + fabs(tgt_orient[3]) < 0.1) {
    final_orient_flag = false;
    tgt_orient[0] = .0;
    tgt_orient[1] = .0;
    tgt_orient[2] = .0;
    tgt_orient[3] = .0;
  } else {
    final_orient_flag = true;
    qToRotation(tgt_orient, tmp_rbt);
    qToRotation(odom_orient, tmp_rob);
    rMultiR(tmp_rob, tmp_rbt, tgt_r);
    rotationToQ(tgt_orient, tgt_r);
  }

  local_tgt_flag = true;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pose[0] = msg->pose.pose.position.x;
  odom_pose[1] = msg->pose.pose.position.y;
  odom_pose[2] = msg->pose.pose.position.z;
  odom_orient[0] = msg->pose.pose.orientation.w;
  odom_orient[1] = msg->pose.pose.orientation.x;
  odom_orient[2] = msg->pose.pose.orientation.y;
  odom_orient[3] = msg->pose.pose.orientation.z;
  qToRotation(odom_orient, odom_r);
}

void ekfposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  odom_pose[0] = msg->pose.position.x;
  odom_pose[1] = msg->pose.position.y;
  odom_pose[2] = msg->pose.position.z;
  odom_orient[0] = msg->pose.orientation.w;
  odom_orient[1] = msg->pose.orientation.x;
  odom_orient[2] = msg->pose.orientation.y;
  odom_orient[3] = msg->pose.orientation.z;
  qToRotation(odom_orient, odom_r);
}

// NOTE: race condition may occur as ai_brain node subscribes to joy topic as well.
// One case is that obsavoid node receives the joy topic first and this callback is called,
// while ai_brain node is running the findpath function that publishes `target_point` which has
// targetPointCallback called and local_tgt_flag is set to true, then a press on 'Y' button has robot move immediately
// without the press on 'B' button.
// Need fix!
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  if (oa == nullptr || drone == nullptr)
     return;

  if (msg->buttons[0] > kButtonHoldThresh && !stop_flag) { // btn A for pause
    ROS_INFO("[OA] Obstacle Avoidance locked!");
    stop_flag = true;
    local_tgt_flag = false;
    oa->reset();
    drone->move(0, 0);
  } else if (msg->buttons[3] > kButtonHoldThresh && stop_flag) { // btn Y for unlock
    ROS_INFO("[OA] Obstacle Avoidance unlocked!");
    stop_flag = false;
  }
}

void doPauseOrUnlock(const std_msgs::Bool::ConstPtr& msg) {
   if (msg->data) {
    stop_flag = true;
    local_tgt_flag = false;
    oa->reset();
    drone->move(0, 0);
   } else {
    stop_flag = false;
   }
 }

// Remote operations
void remoteTaskCancelCallback(const std_msgs::Empty::ConstPtr& msg) {
  if (oa == nullptr || drone == nullptr)
     return;

  ROS_INFO("[OA] Obstacle Avoidance cancelled!");
  stop_flag = true;
  local_tgt_flag = false;
  oa->reset();
  drone->move(.0, .0);
}

void obsAvoidAbortCallback(const std_msgs::Empty::ConstPtr& msg) {
  if (oa == nullptr || drone == nullptr)
     return;

  ROS_INFO("[OA] Obstacle Avoidance cancelled!");

  stop_flag = false;
  local_tgt_flag = false;
  oa->reset();
  drone->move(.0, .0);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "obsavoid");
  ros::NodeHandle nh;

  Pioneer3at tmp_drone(nh);
  P3atObstacleAvoidance tmp_oa(nh);
  drone = &tmp_drone;
  oa = &tmp_oa;

  ros::Subscriber pause_sub = nh.subscribe("/remote/pause_or_unlock", 1, doPauseOrUnlock);
  // ros::Subscriber joy_sub = nh.subscribe("/joy", 1, joyCallback);
  ros::Subscriber odom_sub = nh.subscribe("/RosAria/pose", 1, odomCallback);
  ros::Subscriber tgtpose_sub = nh.subscribe("/ai_robot/findpath/target_point", 1, targetPointCallback);
  // ros::Subscriber remote_joy_sub = nh.subscribe("/remote/joy", 1, joyCallback);
  // ros::Subscriber remote_task_cancel_sub = nh.subscribe("/remote/task_cancel", 1, remoteTaskCancelCallback);
  ros::Subscriber remote_oa_abort_sub = nh.subscribe("/oa_abort", 1, obsAvoidAbortCallback);
  ros::Rate rate(50);

  float local_tgt_pos[3], local_tgt_orient[4], local_tgt_rot[9];
  float vxc = .0, rzc = .0;

  drone->move(.0, .0);

  ROS_INFO("[OA] Waiting odom!");
  ros::topic::waitForMessage<nav_msgs::Odometry>("/RosAria/pose", nh);
  ROS_INFO("[OA] Obstacle avoidance wake up!");

  while (ros::ok()) {
    if (!stop_flag && local_tgt_flag) {
      transformBodyFromNwuWorld(local_tgt_pos[0], local_tgt_pos[1], local_tgt_pos[2], tgt_pos[0], tgt_pos[1], tgt_pos[2], odom_r, odom_pose);
      if (final_orient_flag) {
        rtMultiR(odom_r, tgt_r, local_tgt_rot);
        rotationToQ(local_tgt_orient, local_tgt_rot);
      } else {
        local_tgt_orient[0] = .0;
        local_tgt_orient[1] = .0;
        local_tgt_orient[2] = .0;
        local_tgt_orient[3] = .0;
      }

      if (oa->gotoLocalTarget(vxc, rzc, local_tgt_pos, local_tgt_orient, true)) {
        drone->move(vxc, rzc);
      } else {
        drone->move(.0, .0);
        oa->reset();
        local_tgt_flag = false;
      }
    } else {
      oa->publishSafeZone();
    }
    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
