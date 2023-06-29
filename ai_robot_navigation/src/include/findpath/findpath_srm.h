#ifndef AI_ROBOT_NAVIGATION__FINDPATH_FINDPATHSRM_H_
#define AI_ROBOT_NAVIGATION__FINDPATH_FINDPATHSRM_H_

#include <fstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include "octree.h"
#include "utils/math_aux.h"

class PathNode {
public:
  PathNode(float x, float y, float z, int id);
  ~PathNode();

  void addLink(int id_link, float dist);
  void removeLink(int id_link);
  void setXYZId(float x, float y, float z, int id);

public:
  // flag represent wether the node is dealed
  bool dealed_flag;
  // for A star or Dijkstra
  float dist_til_now;
  float dist_to_end;
  float dist_total;
  float x;
  float y;
  float z;
  int id_from_where;
  
  // link between this node and the other node
  std::vector<int> id_links;
  // store distance between node
  std::vector<float> dist_links;

private:
  // location in the queue
  int id_self_;
};

class FindPathSRM {
public:
  FindPathSRM(const ros::NodeHandle& nh, float start_x, float start_y, float start_z,
              float end_x, float end_y, float end_z, bool view_flag);
  FindPathSRM(const ros::NodeHandle& n, bool view_flag);
  ~FindPathSRM();

  void displayPointCloud();
  // find path, before findpath, make sure start point and end point are set
  bool findPath();
  // get target speed acording to position now,
  // if return true, get v; if false, to destination and get target pose
  // input vx,vy,vz should be vel_now initially
  bool getTargetSpeed(float now_x, float now_y, float now_z, float now_yaw,
                      float& v_x, float& v_y, float& v_z, float& v_yaw);
  double getShortestPathLength();
  // reload graph
  void reloadGraph(std::string filepath);
  void resetAll(float start_x, float start_y, float start_z, float end_x,
                float end_y, float end_z);
  void setPosForVisual(float now_x, float now_y, float now_z, float now_yaw);

private:
  // calculate derivation of potantial
  void calDPotantial2(float now_x, float now_y, float now_z, float tar_x,
                      float tar_y, float tar_z, float& v_x, float& v_y, float& v_z);
  // calculate yaw according to v
  void calYaw(float v_x, float v_y, float v_z, float& yaw);
  // clear data before reload
  void clearAll();
  float getYawSpeed(float yaw_tar, float yaw_now);
  void initPclViewer();
  void initService();
  // insert and sort path node in queue by distTotal, large at begin, small at end
  void insertSortByDistTotal(int id, std::vector<int>& nodeQueue);
  // pub target point for obstacle avoid
  void publishTargetPoint();
  bool readKeyPoint();
  // read link between key pos
  bool readLink();
  bool readMapPoint();
  bool readParams();
  // read file and reconstruct node graph
  bool reconstructGraph();
  // reload map from the path of msg
  void reloadCb(const std_msgs::StringConstPtr& msg);
  void restartNavCallback(const geometry_msgs::Pose::ConstPtr& msg);
  // for i=12345, change str(line00000) to str(line12345)
  void setString(std::string& str, int k);
  void testShowNode();
  void testShowQueue(std::vector<int>& vec);

  // a star
  template <typename Distance>
  bool astar();

  // find start and end node
  template <typename Distance>
  bool findStartEndNode();
  
private:
  bool all_file_exit_flag_;
  bool new_tar_flag_;
  int just_replan_;
  int pathid_near_node_;
  float map_point_sparse_;
  float rwc_[9];
  float vel_xyz_now_[3];

  unibn::Octree<pcl::PointXYZRGB> map_oct_;
  
  // for calculate potantial
  // obstacle
  //  potantial = 1 / (1 + exp(distance - adist)*bdist)
  // start/end
  //  potantial = 1 / (1 + exo(distance - adist*halfdist)*bdist)
  float a_end_dist_;
  float a_obs_dist_;
  float a_start_dist_;
  float b_end_dist_;
  float b_obs_dist_;
  float b_start_dist_;
  float half_start_end_dist_;

  // for calDpotantial2
  int kseg_;
  int kseg_now_;
  float lone_seg_;

  // for calculate time consume of calculate potantial
  int cnt_calpotan_;
  double time_calpotan_total;
  double time_max_calpotan_;
  

  // for time
  int64_t time_begin_;
  int64_t time_end_;

  // nearest node from start/end position
  int id_start_node_;
  int id_end_node_;

  // target point id when move along path
  int idp_tgt_node_;

  // for pcl viewer
  bool deal_view_flag_;
  bool first_view_flag_;
  int v1_;
  bool view_flag_;

  // the start and end position
  pcl::PointXYZRGB end_pose_;
  pcl::PointXYZRGB now_pose_;
  pcl::PointXYZRGB now_tar_pose_;
  pcl::PointXYZRGB now_vel_pose_;
  pcl::PointXYZRGB now_dir_pose_;
  pcl::PointXYZRGB now_fvi_pose_;
  pcl::PointXYZRGB now_left_pose_;
  pcl::PointXYZRGB now_up_pose_;
  pcl::PointXYZRGB start_pose_;
  // from keyframehandle for view, keyPosPC<->denkeyfPC,
  // keyPosLink<->denkeyfLine
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr key_pose_pc;
  std::vector<std::vector<int>> key_pos_link_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_pc_;


  ros::NodeHandle nh_;
  ros::Subscriber restart_sub_;
  ros::Subscriber reload_map_sub_;
  ros::Publisher tgt_point_pub_;

  std::ofstream fs_logger_;
  // log file
  std::string log_filepath_;
  std::string den_keyframe_filepath_;
  std::string den_keyframe_relation_filepath_;
  std::string map_points_pos_filepath_;
  std::string keyframe_params_filepath_;
  
  // id of node along path, link is (end
  // position)<-(path.begin)<-(path.end)<-(start position)
  std::vector<int> path_;
  std::vector<pcl::PointXYZ> path_point_;
  // keyframe position, the last one is destination
  std::vector<PathNode> path_node_;
  
};

#endif  // AI_ROBOT_NAVIGATION__FINDPATH_FINDPATHSRM_H_
