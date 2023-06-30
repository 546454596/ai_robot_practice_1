#ifndef AI_ROBOT_NAVIGATION_P3AT_OBSAVOID_H
#define AI_ROBOT_NAVIGATION_P3AT_OBSAVOID_H

#include <cstdio>
#include <cstdlib>

#include <fstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <pcl/common/transforms.h>
// #include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// #include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
// #include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include "ai_robot_interfaces/restart_nav.h"
#include "octree.h"
#include "utils/math_aux.h"

class SafeZone {
public:
  SafeZone();
  ~SafeZone();

  float getdTheta(float dyaw, float tgt_dist, float& dist, float& dir, bool& in_safezone);
  void getMidXYZ(double& x, double& y, double& z);
  bool setSafeZone(float ldis, float lang, float rdis, float rang);
  void setSimpleSafeZone(float ldis, float lang, float rdis, float rang, int llevel, int rlevel, float deld);

public:
  float left_p[2];
  float right_p[2];
  float safe_dir[2];
  float *near;
  float *far;
};

class P3atObstacleAvoidance {
public:
  P3atObstacleAvoidance(const ros::NodeHandle& nh);
  ~P3atObstacleAvoidance();

  inline double getDeldValue() { return deld_; }
  bool gotoLocalTarget(float& vx, float& rz, float local_tgt_p[], float local_tgt_q[], bool findpath);
  bool modifyVelocity(float& vx, float& rz, float dyaw, float tgt_dist, bool findpath);
  bool modifyVelocityByLidar(float& vx, float& rz, float dyaw, float tgt_dist, bool findpath);
  bool modifyVelocityByLidar3d(float& vx, float& rz, float dyaw, float tgt_dist, bool findpath);
  void modifyVelocityBySonar(float&vx, float& rz);
  void modifyVelocityBySonarSafeZone(float& vx, float& rz);
  bool modifyVelocityByTof(float& vx, float& rz, float dyaw, float tgt_dist, bool findpath);
  void publishSafeZone();
  void reset();

private:
  void checkSafeZone(pcl::PointXYZ& self1, pcl::PointXYZ& self2, pcl::PointXYZ& tgtp1, pcl::PointXYZ& tgtp2,
                     cv::Point2f& start_point, cv::Point2f& end_point, std::vector<int>& all_levels, int& llevel,
                     int& rlevel, int& num_of_dir, SafeZone& safezone);
  inline void polar2xy(float dist, float angle, float& x, float& y) { x = dist * cos(angle), y = dist * sin(angle); }
  inline void xy2polar(float x, float y, float& dist, float& angle) { dist = sqrt(x * x + y * y), angle = atan2(y, x); }
  int findNearestSafeZone(float dyaw, float tgt_dist, float& ddir_safe, float& dist, float& dir,
                          float& ddir_between_now_and_chosen_dir, bool& in_safe_zone);
  void findSafeZone(const sensor_msgs::LaserScanConstPtr& msg);
  void findSafeZoneOctree(const sensor_msgs::LaserScanConstPtr& msg);
  bool hasSuddenChange(float rang, float lang, const sensor_msgs::LaserScanConstPtr& msg, std::vector<int>& sudden_v,
                       int& start_id_of_sc);
  void initPublishersSubscribers();
  void readParameters();
  void smooth(float& vx, float& rz);

  void lidar3dCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void lidarCallback(const sensor_msgs::LaserScanConstPtr& msg);
  void sonarPclCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void targetPoseCallback(const geometry_msgs::PoseConstPtr& msg);
  void tofPclCallback(const sensor_msgs::PointCloud2ConstPtr& msg);


private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport img_trans_;

  ros::Subscriber lidar_sub_;
  ros::Subscriber lidar3d_sub_;
  ros::Subscriber sonar_pcl2_sub_;
  ros::Subscriber tgtpose_sub_;
  ros::Subscriber tof_pcl_sub_;
  // This message uses pose as topic, position is the safezone desired point in body frame:
  // - `orientation.w` is the flag indicating whether exits a safezone (yes: >0, no: <0)
  // - `orientation.x` is the flag indicating whether velocity is modified (yes: >0, no: <0)
  // - `orientation.y` is the flag indicating whether the original target is block (yes: >0, no: <0)
  ros::Publisher replan_pub_;
  ros::Publisher safezone_pub_;

  std::ofstream fs_logger_;

  int nlayers_;
  double deadzone_;

  int just_replan_;
  bool in_danger_;

  double max_err_dir_;
  double max_rz_;
  double max_vx_;
  double min_turn_radius_;
  double safezone_rest_wide_;

  // sonar
  bool sonar_used_;
  pcl::PointCloud<pcl::PointXYZ> sonar_pcl_now_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> sonar_pcl_vec_;
  std::vector<double> sonar_time_past_;
  double sonar_time_now_;
  int cnt_sonar_;

  // tof
  bool tof_used_;
  bool is_view_safezone_tof_;
  pcl::PointCloud<pcl::PointXYZ> tof_pcl_now_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> tof_pcl_vec_;
  std::vector<double> tof_time_past_;
  double tof_time_now_;
  double cam_initial_height_;
  double half_fov_;
  double okdata_half_fov_;

  // lidar
  bool lidar_used_;
  std::vector<float> lidar_range_now_;
  std::vector<std::vector<float>> lidar_range_vec_;
  std::vector<double> lidar_time_past_;
  double lidar_time_now_;
  std::vector<SafeZone> safezone_vec_;
  cv::Mat safezone_view_;
  bool is_view_safezone_lidar_;
  float orig_dir_, mod_dir_;
  double t_lidar_body_[3];
  float center_dir_dist_;
  float last_desired_dir_;
  double time_of_halfpi_;
  double det_tol_;

  //robot's width   robot's safe gap
  double robot_width, robot_safe_gap;

  // lidar3d
  bool lidar3d_used_;
  double lidar3d_initial_height_;

  // lidar with octree
  double dangle_;
  int times_of_deld_;
  double deld_; // diameter of circle to search
  double range_step_size_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pc_xyz_;
  unibn::Octree<pcl::PointXYZ> lidar_xyz_oct_;
  std::vector<float> last_chosen_dir_;

  float a_sonar_[8][2];
  float asonar_;
  double safezone_lvl_err_thresh_;
  double lvl_abs_safe_; // absolute safe
  double safezone_min_err_range_;

  // time
  double time_begin_;
  double time_end_;
  double time_intvl_;
  double time_begin2_;
  double time_end2_;
  double time_intvl2_;

  // for time consumption and potential calculation
  double time_calcpot_tot_;
  double time_calcpot_max_;
  int cnt_calcpot_;
  bool should_print_time_;

  // for replan
  pcl::PointXYZ tgt_point_;
  float tgt_orient_[4]; // w,x,y,z
  int tgt_point_blocked_;

  // smooth velocity
  // for control over orientation
  // vx = c_dist * dist
  // rz = (a_theta + b_alpha) * new_dir - b_alpha * dyaw
  float last_vx_;
  float last_rz_;
  bool in_safezone_;
  double a_theta_;
  double b_alpha_;
  double c_dist_;
};

#endif
