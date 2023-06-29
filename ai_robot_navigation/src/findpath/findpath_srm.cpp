#include "findpath/findpath_srm.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = std::make_shared<pcl::visualization::PCLVisualizer>("Map Viewer");

const unibn::OctreeParams &map_oct_params = unibn::OctreeParams(30, false, 0.02f);

PathNode::PathNode(float x, float y, float z, int id)
    : x(x),
      y(y),
      z(z),
      id_self_(id),
      id_from_where(-1),
      dealed_flag(false),
      dist_til_now(-1),
      dist_to_end(-1),
      dist_total(-1) {}

PathNode::~PathNode() {}

void PathNode::setXYZId(float x, float y, float z, int id) {
  this->x = x;
  this->y = y;
  this->z = z;
  id_self_ = id;
}

void PathNode::addLink(int id_link, float dist) {
  id_links.push_back(id_link);
  dist_links.push_back(dist);
}

void PathNode::removeLink(int id_link) {
  for (int i = 0; i < id_links.size(); ++i) {
    if (id_links[i] == id_link) {
      id_links.erase(id_links.begin() + i);
      dist_links.erase(dist_links.begin() + i);
    }
  }
}

FindPathSRM::FindPathSRM(const ros::NodeHandle &nh, float start_x, float start_y, float start_z, float end_x, float end_y,
                         float end_z, bool view_flag)
    : nh_(nh),
      all_file_exit_flag_(false),
      start_pose_(200, 0, 200),
      end_pose_(100, 50, 50),
      id_start_node_(-1),
      id_end_node_(-1),
      view_flag_(view_flag),
      map_point_sparse_(5.0),
      deal_view_flag_(0),
      new_tar_flag_(0),
      first_view_flag_(1),
      idp_tgt_node_(0),
      rwc_{1, 0, 0, 0, 1, 0, 0, 0, 1},
      just_replan_(0),
      a_start_dist_(0.5),
      b_start_dist_(0.05),
      a_end_dist_(1.5),
      b_end_dist_(0.05),
      a_obs_dist_(1),
      b_obs_dist_(2),
      key_pose_pc(new pcl::PointCloud<pcl::PointXYZRGB>),
      map_pc_(new pcl::PointCloud<pcl::PointXYZRGB>) {
  start_pose_.x = start_x;
  start_pose_.y = start_y;
  start_pose_.z = start_z;
  end_pose_.x = end_x;
  end_pose_.y = end_y;
  end_pose_.z = end_z;
  now_pose_.x = 0;
  now_pose_.y = 0;
  now_pose_.z = 0;
  now_tar_pose_.x = 0;
  now_tar_pose_.y = 0;
  now_tar_pose_.z = 0;
  now_vel_pose_.x = 0;
  now_vel_pose_.y = 0;
  now_vel_pose_.z = 0;
  now_dir_pose_.x = 0;
  now_dir_pose_.y = 0;
  now_dir_pose_.z = 0;
  nh_.getParam("logfile", log_filepath_);
  nh_.getParam("denKeyfPos", den_keyframe_filepath_);
  nh_.getParam("denKeyfPosRelation", den_keyframe_relation_filepath_);
  nh_.getParam("MapPointsPos", map_points_pos_filepath_);
  nh_.getParam("keyframe_params_filepath", keyframe_params_filepath_);

  fs_logger_.open(log_filepath_);
  initPclViewer();
  reconstructGraph();
  initService();
}

FindPathSRM::FindPathSRM(const ros::NodeHandle &n, bool view_flag)
    : nh_(n),
      all_file_exit_flag_(false),
      start_pose_(0, 0, 0),
      end_pose_(0, 0, 0),
      id_start_node_(-1),
      id_end_node_(-1),
      view_flag_(view_flag),
      map_point_sparse_(5.0),
      deal_view_flag_(0),
      new_tar_flag_(0),
      first_view_flag_(1),
      idp_tgt_node_(0),
      rwc_{1, 0, 0, 0, 1, 0, 0, 0, 1},
      just_replan_(0),
      kseg_now_(0),
      kseg_(0),
      a_start_dist_(0.5),
      b_start_dist_(0.05),
      a_end_dist_(1.5),
      b_end_dist_(0.05),
      a_obs_dist_(1),
      b_obs_dist_(2),
      key_pose_pc(new pcl::PointCloud<pcl::PointXYZRGB>),
      map_pc_(new pcl::PointCloud<pcl::PointXYZRGB>) {
  nh_.getParam("logfile", log_filepath_);
  nh_.getParam("denKeyfPos", den_keyframe_filepath_);
  nh_.getParam("denKeyfPosRelation", den_keyframe_relation_filepath_);
  nh_.getParam("MapPointsPos", map_points_pos_filepath_);
  nh_.getParam("keyframe_params_filepath", keyframe_params_filepath_);

  initPclViewer();
  reconstructGraph();
  now_pose_.x = 0;
  now_pose_.y = 0;
  now_pose_.z = 0;
  now_tar_pose_.x = 0;
  now_tar_pose_.y = 0;
  now_tar_pose_.z = 0;
  now_dir_pose_.x = 0;
  now_dir_pose_.y = 0;
  now_dir_pose_.z = 0;
  initService();
  fs_logger_.open(log_filepath_);
}

FindPathSRM::~FindPathSRM() { fs_logger_.close(); }

void FindPathSRM::initService() {
  // This demo does not load the /topo/savedone topic by service
  reload_map_sub_ = nh_.subscribe("/topo/savedone", 1, &FindPathSRM::reloadCb, this);
  // Subscribe from obsavoid node for navigation restart
  restart_sub_ = nh_.subscribe("/ai_robot/restart_nav", 1, &FindPathSRM::restartNavCallback, this);
  // Publish local target point to obstacle avoidance node
  tgt_point_pub_ = nh_.advertise<geometry_msgs::Pose>("/ai_robot/findpath/target_point", 1);
}

double FindPathSRM::getShortestPathLength() {
  return path_node_[id_end_node_].dist_til_now;
}

bool FindPathSRM::findPath() {
  if (!all_file_exit_flag_) {
    std::cout << "[FindPath] Some files not existing\n";
    return false;
  }
  int64_t start1 = 0, end1 = 0;
  double t;

  start1 = cv::getTickCount();
  if (findStartEndNode<unibn::L2Distance<pcl::PointXYZRGB>>()) {
    end1 = cv::getTickCount();
    double t = 1000 * double(end1 - start1) / cv::getTickFrequency();
    start1 = end1;
    fs_logger_ << t << " ";
    if (!astar<unibn::L2Distance<pcl::PointXYZRGB>>()) {  // Start A* planning
      fs_logger_ << 0 << '\n';
      return false;
    }
  } else {
    end1 = cv::getTickCount();
    t = 1000 * double(end1 - start1) / cv::getTickFrequency();
    fs_logger_ << t << " ";
    std::cout << "Start/End position is not reachable!\n";
    return 0;
  }
  end1 = cv::getTickCount();
  t = 1000 * double(end1 - start1) / cv::getTickFrequency();
  fs_logger_ << t << '\n';
 
  return true;
}

// visualization settings
void FindPathSRM::setPosForVisual(float now_x, float now_y, float now_z, float now_yaw) {
  now_fvi_pose_.x = now_x;
  now_fvi_pose_.y = now_y;
  now_fvi_pose_.z = now_z;
  now_dir_pose_.z = now_z;
  rwc_[0] = cos(now_yaw);
  rwc_[1] = -sin(now_yaw);
  rwc_[3] = sin(now_yaw);
  rwc_[4] = cos(now_yaw);
  now_dir_pose_.x = now_x + 2 * rwc_[0];  // cos(now_yaw);
  now_dir_pose_.y = now_y + 2 * rwc_[3];  // sin(now_yaw);
  now_left_pose_.z = now_z;
  now_left_pose_.x = now_x - 1.5 * rwc_[3];  // sin(now_yaw);
  now_left_pose_.y = now_y + 1.5 * rwc_[0];  // cos(now_yaw);
  now_up_pose_.x = now_x;
  now_up_pose_.y = now_y;
  now_up_pose_.z = now_z + 1.5;

  if (just_replan_ > 0) {
    --just_replan_;
  }
}

// visualization implementation
void FindPathSRM::displayPointCloud() {
  if (all_file_exit_flag_) {
    if (first_view_flag_) {
      viewer->addLine(now_fvi_pose_, now_dir_pose_, 200, 0, 0, "linedir", v1_);
      viewer->addLine(now_fvi_pose_, now_left_pose_, 0, 0, 170, "lineleft", v1_);
      viewer->addLine(now_fvi_pose_, now_up_pose_, 0, 0, 170, "lineup", v1_);
      viewer->addLine(now_fvi_pose_, now_fvi_pose_, 200, 0, 170, "linenode", v1_);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "linedir", v1_);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "lineleft", v1_);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "lineup", v1_);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "linenode", v1_);
      viewer->setCameraPosition(now_fvi_pose_.x - 2 * (now_dir_pose_.x - now_fvi_pose_.x) - 2 * rwc_[1],
                                now_fvi_pose_.y - 2 * (now_dir_pose_.y - now_fvi_pose_.y) + 2 * rwc_[0],
                                now_fvi_pose_.z + 15, now_dir_pose_.x, now_dir_pose_.y, now_dir_pose_.z, 0, 0, 1);
      first_view_flag_ = false;
    } else {
      viewer->removeShape("linedir", v1_);
      viewer->addLine(now_fvi_pose_, now_dir_pose_, 200, 0, 0, "linedir", v1_);
      viewer->removeShape("lineleft", v1_);
      viewer->addLine(now_fvi_pose_, now_left_pose_, 0, 0, 170, "lineleft", v1_);
      viewer->removeShape("lineup", v1_);
      viewer->addLine(now_fvi_pose_, now_up_pose_, 0, 0, 170, "lineup", v1_);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "linedir",v1_);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "lineleft",v1_);
      viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "lineup",v1_);

      if (path_point_.size() > 0) {
        viewer->removeShape("linenode", v1_);
        viewer->addLine(now_fvi_pose_, path_point_[pathid_near_node_], 20, 0, 17, "linenode", v1_);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "linenode", v1_);
      }
    }

    if (!deal_view_flag_ && view_flag_) {
      // display map point and dense keyframe position
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pch_map_pc(map_pc_);
      viewer->addPointCloud<pcl::PointXYZRGB>(map_pc_, pch_map_pc, "origin map point cloud", v1_);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "origin map point cloud", v1_);

      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pch_key_pose_pc(key_pose_pc);
      viewer->addPointCloud<pcl::PointXYZRGB>(key_pose_pc, pch_key_pose_pc, "keyframe pos", v1_);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keyframe pos", v1_);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr start_end_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

      start_end_pc->push_back(start_pose_);
      start_end_pc->push_back(end_pose_);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchsePC(start_end_pc);
      viewer->addPointCloud<pcl::PointXYZRGB>(start_end_pc, pchsePC, "startend", v1_);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "startend", v1_);

      viewer->addCoordinateSystem(1.0, "origin cloud", v1_);
      // Add line of keyframe pos
      std::string li("line00000");

      for (int i = 0; i < key_pos_link_.size(); ++i) {
        setString(li, i);
        viewer->addLine(key_pose_pc->points[key_pos_link_[i][0]], key_pose_pc->points[key_pos_link_[i][1]], 0, 255, 0,
                        li, v1_);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, li, v1_);
      }

      // Add path line
      if (path_.size() >= 2) {
        std::string pli("path00000");
        // the first one is the end point and last one is the start point
        for (int j = 0; j <= path_.size() - 2; ++j) {
          setString(pli, j);
          viewer->addLine(path_point_[j], path_point_[j + 1], 200, 0, 200, pli, v1_);
          viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, pli, v1_);
        }
        std::cout << path_[path_.size() - 1] << '\n';
      }

      viewer->addLine(now_pose_, now_tar_pose_, 0, 200, 25, "nowline", v1_);
      viewer->addLine(now_pose_, now_vel_pose_, 30, 0, 100, "velline", v1_);

      deal_view_flag_ = true;
    }

    if (new_tar_flag_) {
      viewer->removeShape("nowline", v1_);
      viewer->addLine(now_pose_, now_tar_pose_, 0, 200, 25, "nowline", v1_);
      new_tar_flag_ = false;
    }

    viewer->removeShape("velline", v1_);
    viewer->addLine(now_pose_, now_vel_pose_, 30, 0, 100, "velline", v1_);
    viewer->spinOnce(1);
  }
}

void FindPathSRM::resetAll(float start_x, float start_y, float start_z,
                           float end_x, float end_y, float end_z) {
  start_pose_.x = start_x;
  start_pose_.y = start_y;
  start_pose_.z = start_z;
  end_pose_.x = end_x;
  end_pose_.y = end_y;
  end_pose_.z = end_z;
  idp_tgt_node_ = 0;  // target point id when move along path
  cnt_calpotan_ = 0;
  time_calpotan_total = 0;
  time_max_calpotan_ = 0;
  kseg_now_ = 0;
  kseg_ = 0;
  id_end_node_ = -1;
  id_start_node_ = -1;
  pathid_near_node_ = 0;

  if (all_file_exit_flag_) {
    path_node_[path_node_.size() - 1].x = end_x;
    path_node_[path_node_.size() - 1].y = end_y;  
    path_node_[path_node_.size() - 1].z = end_z;
    path_node_[path_node_.size() - 2].x = start_x;
    path_node_[path_node_.size() - 2].y = start_y;
    path_node_[path_node_.size() - 2].z = start_z;
    path_.clear();
    path_point_.clear();
    if (deal_view_flag_) {
      viewer->removeAllShapes(v1_);
      viewer->removeAllPointClouds(v1_);
      deal_view_flag_ = 0;
    }

    for (int i = 0; i < path_node_.size(); ++i) {
      path_node_[i].dealed_flag = false;
      path_node_[i].dist_til_now = -1;
    }
  }
}

bool FindPathSRM::getTargetSpeed(float now_x, float now_y, float now_z,
                                 float now_yaw, float& v_x, float& v_y, float& v_z,
                                 float& v_yaw) {
  int64_t start1 = 0, end1 = 0;
  start1 = cv::getTickCount();
  pcl::PointXYZRGB tmp;
  now_pose_.x = now_x;
  now_pose_.y = now_y;
  now_pose_.z = now_z;
  tmp.x = now_x;
  tmp.y = now_y;
  tmp.z = now_z;
  vel_xyz_now_[0] = v_x;
  vel_xyz_now_[1] = v_y;
  vel_xyz_now_[2] = v_z;
  float dist = sqrt(pow(now_x - path_point_[idp_tgt_node_].x, 2) +
                    pow(now_y - path_point_[idp_tgt_node_].y, 2));
  float yaw_tar;
  // update which node drone is in
  if (pathid_near_node_ > 1) {
    float nodedist = (pow(now_x - path_point_[pathid_near_node_].x, 2) +
                      pow(now_y - path_point_[pathid_near_node_].y, 2));  
    float nextnodedist = (pow(now_x - path_point_[pathid_near_node_ - 1].x, 2) +
                          pow(now_y - path_point_[pathid_near_node_ - 1].y, 2));  
    if (nextnodedist < nodedist) {
      --pathid_near_node_;
    }
  }
  if (idp_tgt_node_ >= 0) {
    pcl::PointXYZRGB tar;
    if (kseg_now_ > (kseg_ - 1)) { // cal next segment path
      if (idp_tgt_node_ == 0) {
        if (dist < 0.5) {
          v_x = end_pose_.x;
          v_y = end_pose_.y;
          v_z = end_pose_.z;
          v_yaw = now_yaw;
          std::cout << "Average calculated potantial time:"
                    << time_calpotan_total / cnt_calpotan_
                    << " ms, max calculate time:" << time_max_calpotan_ << " ms." << '\n';
          fs_logger_ << "to target" << '\n' << '\n';
          time_max_calpotan_ = 0;
          cnt_calpotan_ = 0;
          time_calpotan_total = 0;
          return false;
        } else {
          tar.x = end_pose_.x;
          tar.y = end_pose_.y;
          tar.z = end_pose_.z;
        }
      } else {
        --idp_tgt_node_;
        kseg_now_ = 0;
        dist = sqrt(pow(path_point_[idp_tgt_node_].x -
                        path_point_[idp_tgt_node_ + 1].x, 2) +
                    pow(path_point_[idp_tgt_node_].y -
                        path_point_[idp_tgt_node_ + 1].y, 2));  
        kseg_ = dist / lone_seg_;
        tar.x = path_point_[idp_tgt_node_ + 1].x;
        tar.y = path_point_[idp_tgt_node_ + 1].y;
        tar.z = path_point_[idp_tgt_node_ + 1].z;
      }
      fs_logger_ << "~~~~next segment:" << idp_tgt_node_ << " node. "
                   << path_point_[idp_tgt_node_].x << ","
                   << path_point_[idp_tgt_node_].y << ","
                   << path_point_[idp_tgt_node_].z
                   << "---kseg_:" << kseg_ << '\n';
    } else { // cal this segment path
      ++kseg_now_;
      tar.x = (path_point_[idp_tgt_node_].x * kseg_now_ +
               path_point_[idp_tgt_node_ + 1].x *
               (kseg_ - kseg_now_)) / kseg_;
      tar.y = (path_point_[idp_tgt_node_].y * kseg_now_ +
               path_point_[idp_tgt_node_ + 1].y *
               (kseg_ - kseg_now_)) / kseg_;
      tar.z = (path_point_[idp_tgt_node_].z * kseg_now_ +
               path_point_[idp_tgt_node_ + 1].z *
               (kseg_ - kseg_now_)) / kseg_;
      dist = sqrt(pow(now_x - tar.x, 2) + pow(now_y - tar.y, 2)); 
      // if blocked, back to former point|| kseg_now_ > (kseg_)
      if (dist > 3 || map_oct_.isBlock<unibn::L2Distance<pcl::PointXYZRGB>>(tmp, tar, 1.2 * map_point_sparse_)) {
        --kseg_now_;
        // if(kseg_now_ < 0) kseg_now_ = 0;
        tar.x = (path_point_[idp_tgt_node_].x * kseg_now_ +
                 path_point_[idp_tgt_node_ + 1].x *
                 (kseg_ - kseg_now_)) / kseg_;
        tar.y = (path_point_[idp_tgt_node_].y * kseg_now_ +
                 path_point_[idp_tgt_node_ + 1].y *
                 (kseg_ - kseg_now_)) / kseg_;
        tar.z = (path_point_[idp_tgt_node_].z * kseg_now_ +
                 path_point_[idp_tgt_node_ + 1].z *
                 (kseg_ - kseg_now_)) / kseg_;
      }
      fs_logger_ << "to:" << tar.x << "," << tar.y << "," << tar.z
                   << "---kseg_now:" << kseg_now_ << '\n';
    }
    calDPotantial2(now_x, now_y, now_z, tar.x, tar.y, tar.z, v_x, v_y, v_z);
    calYaw(v_x, v_y, v_z, yaw_tar);
    v_yaw = getYawSpeed(yaw_tar, now_yaw);
    now_tar_pose_.x = tar.x;
    now_tar_pose_.y = tar.y;
    now_tar_pose_.z = tar.z;
    new_tar_flag_ = true;
  }

  end1 = cv::getTickCount();
  double ti = 1000 * double(end1 - start1) / cv::getTickFrequency();
  publishTargetPoint();
  // double _ti = ((double)(time_end - time_begin) / CLOCKS_PER_SEC);
  if (ti > time_max_calpotan_) {
    time_max_calpotan_ = ti;
  }
  time_calpotan_total += ti;
  ++cnt_calpotan_;

  now_vel_pose_.x = now_pose_.x + v_x * 2;
  now_vel_pose_.y = now_pose_.y + v_y * 2;
  now_vel_pose_.z = now_pose_.z + v_z * 2;

  fs_logger_ << "target position:"
               << path_node_[path_[idp_tgt_node_]].x << ","
               << path_node_[path_[idp_tgt_node_]].y << ","
               << path_node_[path_[idp_tgt_node_]].z
               << ",position now:" << now_x << "," << now_y << "," << now_z
               << '\n'
               << "target speed:" << v_x << "," << v_y << "," << v_z << " = "
               << sqrt(v_x * v_x + v_y * v_y) << " ==yaw now:" << now_yaw
               << ", yaw tar:" << yaw_tar << ", yaw speed:" << v_yaw << '\n';
  viewer->setCameraPosition(
      now_fvi_pose_.x - 2 * (now_dir_pose_.x - now_fvi_pose_.x) - 2 * rwc_[1],
      now_fvi_pose_.y - 2 * (now_dir_pose_.y - now_fvi_pose_.y) + 2 * rwc_[0],
      now_fvi_pose_.z + 15, now_dir_pose_.x, now_dir_pose_.y,
      now_dir_pose_.z, 0, 0, 1);

  return true;
}

void FindPathSRM::publishTargetPoint() {
  // publish target pose in body frame
  geometry_msgs::Pose tgt_p_msg;
  float twc[3] = {now_pose_.x, now_pose_.y, now_pose_.z}, tgt_p_frombody[3];
  // float rrwc[9]={rwc_[0],-rwc_[1],0, -rwc_[3],rwc_[4],0, 0,0,1};
  transformBodyFromNwuWorld(tgt_p_frombody[0], tgt_p_frombody[1], tgt_p_frombody[2], now_tar_pose_.x, now_tar_pose_.y,
                            now_tar_pose_.z, rwc_, twc);
  tgt_p_msg.position.x = tgt_p_frombody[0];
  tgt_p_msg.position.y = tgt_p_frombody[1];
  tgt_p_msg.position.z = tgt_p_frombody[2];
  float tgt_yawp_frombody[3], q[4];
  float tgt_dir[3] = {path_point_[idp_tgt_node_].x, path_point_[idp_tgt_node_].y, path_point_[idp_tgt_node_].z};
  transformBodyFromNwuWorld(tgt_yawp_frombody[0], tgt_yawp_frombody[1], tgt_yawp_frombody[2], tgt_dir[0], tgt_dir[1],
                            tgt_dir[2], rwc_, twc);
  float tar_yaw = atan2(tgt_yawp_frombody[1] - tgt_p_frombody[1], tgt_yawp_frombody[0] - tgt_p_frombody[0]);
  eulerToQ(0, 0, tar_yaw, q);
  
  if (idp_tgt_node_ > 0) {
    tgt_p_msg.orientation.w = q[0];
    tgt_p_msg.orientation.x = q[1];
    tgt_p_msg.orientation.y = q[2];
    tgt_p_msg.orientation.z = q[3];
    tgt_point_pub_.publish(tgt_p_msg);
  } else {
    tgt_p_msg.orientation.w = 0;
    tgt_p_msg.orientation.x = 0;
    tgt_p_msg.orientation.y = 0;
    tgt_p_msg.orientation.z = 0;
    tgt_point_pub_.publish(tgt_p_msg);  // for autolabor
  }
}

// for cal potantial
typedef std::pair<uint32_t, float> PAIR;
bool cmpByValue(const PAIR &lhs, const PAIR &rhs) {
  return lhs.second < rhs.second;
}

void FindPathSRM::calDPotantial2(float now_x, float now_y, float now_z,
                                 float tar_x, float tar_y, float tar_z,
                                 float& v_x, float& v_y, float& v_z) {
  float dx = tar_x - now_x;
  float dy = tar_y - now_y;
  float dl = sqrt(pow(dx, 2) + pow(dy, 2));
  float vtar;
  if (dl > 2) {
    vtar = 0.5;
  } else {
    vtar = dl / 4;
  }
  float vo = vtar / dl;
  v_x = vo * dx;
  v_y = vo * dy;

  if (dx > 0)
    v_x = fabs(v_x);
  else
    v_x = -fabs(v_x);

  if (dy > 0)
    v_y = fabs(v_y);
  else
    v_y = -fabs(v_y);

  float dz = tar_z - now_z;
  if (fabs(dz) > 0.3)
    v_z = 0.3 * fabs(dz) / dz;
  else
    v_z = dz;
}

void FindPathSRM::calYaw(float vx, float vy, float vz, float &yaw) {
  float L = sqrt(pow(vx, 2) + pow(vy, 2));
  if (vy > 0)
    yaw = acos(vx / L);
  else
    yaw = -acos(vx / L);
}

float FindPathSRM::getYawSpeed(float yaw_tar, float yaw_now) {
  float yErr = yaw_tar - yaw_now;
  if (yErr > 3.1415) {
    yErr -= 2 * 3.1415;
  } else if (yErr < -3.1415) {
    yErr += 2 * 3.1415;
  }

  return yErr;
}

bool FindPathSRM::reconstructGraph() {
  clearAll();
  std::cout << "Get start position(" << start_pose_.x << "," << start_pose_.y << "," << start_pose_.z
      << "), end position(" << end_pose_.x << "," << end_pose_.y << "," << end_pose_.z << ")." << '\n';
  if (readKeyPoint() && readLink() && readMapPoint()) {
    readParams();
    all_file_exit_flag_ = true;
    map_oct_.initialize(map_pc_->points, map_oct_params);
    return true;
  } else {
    all_file_exit_flag_ = false;
    return false;
  }
}

bool FindPathSRM::readKeyPoint() {
  //    cout<< "den_keyframe_filepath_"<<den_keyframe_filepath_;
  std::ifstream in(den_keyframe_filepath_);
  if (!in) {
    ROS_INFO("[FindPath] No key position file!\n");
    return false;
  }
  std::string line;
  boost::char_separator<char> sep(" ");
  pcl::PointXYZRGB pt;
  // float minx=3000, miny=3000,maxx=-3000,maxy=-3000, minz=3000, maxz=-3000;
  PathNode pn(0, 0, 0, 0);
  int i = 0;
  // read point cloud from "freiburg format"
  while (!in.eof()) {
    std::getline(in, line);
    in.peek();
    boost::tokenizer<boost::char_separator<char>> tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
    if (tokens.size() != 3) continue;
    pt.x = boost::lexical_cast<float>(tokens[0]);
    pt.y = boost::lexical_cast<float>(tokens[1]);
    pt.z = boost::lexical_cast<float>(tokens[2]);
    pt.r = 0;
    pt.g = 0;
    pt.b = 255;

    key_pose_pc->push_back(pt);

    pn.setXYZId(pt.x, pt.y, pt.z, i);
    path_node_.push_back(pn);
    ++i;
  }
  path_node_.push_back(pn);
  // add two to the last for destination and start position
  path_node_.push_back(pn); 

  std::cout << "Read key positions: " << key_pose_pc->size() <<  "in total.\n";
  in.close();

  return true;
}

bool FindPathSRM::readLink() {
  std::ifstream in(den_keyframe_relation_filepath_);
  if (!in) {
    ROS_INFO("[FindPath] No link file!\n");
    return false;
  }
  std::string line;
  boost::char_separator<char> sep(" ");
  int i, j;
  std::vector<int> tmp(2);
  float distmp;

  while (!in.eof()) {
    std::getline(in, line);
    in.peek();
    boost::tokenizer<boost::char_separator<char>> tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
    if (tokens.size() != 2) continue;
    i = boost::lexical_cast<float>(tokens[0]);
    j = boost::lexical_cast<float>(tokens[1]);
    tmp[0] = i;
    tmp[1] = j;
    key_pos_link_.push_back(tmp);

    distmp = unibn::L2Distance<pcl::PointXYZRGB>::sqrt(unibn::L2Distance<pcl::PointXYZRGB>::compute(
        key_pose_pc->points[i], key_pose_pc->points[j])); // 计算两拓扑节点之间的欧式距离

    path_node_[i].addLink(j, distmp);
    path_node_[j].addLink(i, distmp); // 构建无向图
  }
  std::cout << "Read link: " << key_pos_link_.size() << " link in total.\n";
  in.close();

  return true;
}

bool FindPathSRM::readMapPoint() {
  std::ifstream in(map_points_pos_filepath_);
  if (!in) {
    ROS_INFO("[FindPath] No mappoint file!\n");
    return false;
  }
  std::string line;
  boost::char_separator<char> sep(" ");
  pcl::PointXYZRGB pt;
  // read point cloud from "freiburg format"
  while (!in.eof()) {
    std::getline(in, line);
    in.peek();
    boost::tokenizer<boost::char_separator<char>> tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
    if (tokens.size() != 3) continue;
    pt.x = boost::lexical_cast<float>(tokens[0]);
    pt.y = boost::lexical_cast<float>(tokens[1]);
    pt.z = boost::lexical_cast<float>(tokens[2]);
    pt.r = 255;
    pt.g = 0;
    pt.b = 0;

    map_pc_->push_back(pt);
  }

  std::cout << "Read map points: " << map_pc_->size() << " map points in total.\n";
  in.close();
  return true;
}

bool FindPathSRM::readParams() {
  std::cout << "filepath: " << keyframe_params_filepath_ << '\n';
  cv::FileStorage fs(keyframe_params_filepath_, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "can not find parameter file params_for_key_frame.yaml\n"
              << "default parameters are used!\n";
    return false;
  }

  fs["mappointSparse"] >> map_point_sparse_;
  fs["astartdist"] >> a_start_dist_;
  fs["bstartdist"] >> b_start_dist_;
  fs["aenddist"] >> a_end_dist_;
  fs["benddist"] >> b_end_dist_;
  fs["aobsdist"] >> a_obs_dist_;
  fs["bobsdist"] >> b_obs_dist_;
  fs["loneseg"] >> lone_seg_;
  std::cout << "Read parameters...\n"
      << "  mappointSparse:" << map_point_sparse_ << '\n'
      << "  astartdist:" << a_start_dist_ << '\n'
      << "  bstartdist:" << b_start_dist_ << '\n'
      << "  aenddist:" << a_end_dist_ << '\n'
      << "  benddist:" << b_end_dist_ << '\n'
      << "  aobsdist:" << a_obs_dist_ << '\n'
      << "  bobsdist:" << b_obs_dist_ << '\n'
      << "  loneseg:" << lone_seg_ << '\n';
  return true;
}

void FindPathSRM::initPclViewer() {
  viewer->initCameraParameters();
  viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1_);
  viewer->setBackgroundColor(255, 255, 255, v1_);
  viewer->setCameraPosition(-2, 0, 15, 2, 0, 0, 0, 0, 1);
}

void FindPathSRM::setString(std::string& str, int k) {
  int t = 0, i = 0, size = str.size();
  char b = '0';
  while (k > 0) {
    t = k % 10;
    k = k / 10;
    b = b + t;
    str.insert(str.end() - i - 1, b);
    str.erase(size - i, 1);
    b = b - t;
    ++i;
  }
}

template <typename Distance>  
bool FindPathSRM::findStartEndNode() {
  float mindistS = 10000, mindistE = 10000, tmpdist;
  for (int i = 0; i < key_pose_pc->size(); ++i) {
    // 计算距离start_pose_的欧式距离
    tmpdist = Distance::compute(key_pose_pc->points[i], start_pose_);  
    if (tmpdist < mindistS && !map_oct_.isBlock<Distance>(start_pose_,
                                                        key_pose_pc->points[i],
                                                        map_point_sparse_)) {
      id_start_node_ = i;
      mindistS = tmpdist;
    }
    tmpdist = Distance::compute(key_pose_pc->points[i], end_pose_);
    if (tmpdist < mindistE &&
        !map_oct_.isBlock<Distance>(end_pose_, key_pose_pc->points[i],
                                  map_point_sparse_)) {
      id_end_node_ = i;
      mindistE = tmpdist;
    }
  }
  
  if (id_start_node_ < 0 || id_end_node_ < 0) {
    return 0;
  } else {
    return 1;
  }
}

template <typename Distance>
bool FindPathSRM::astar() {
  std::vector<int> node_queue;
  // first node
  path_node_[id_start_node_].id_from_where = id_start_node_;
  path_node_[id_start_node_].dist_til_now = 0;
  path_node_[id_start_node_].dist_to_end = Distance::sqrt(
      Distance::compute(key_pose_pc->points[id_start_node_],
                        key_pose_pc->points[id_end_node_]));
  path_node_[id_start_node_].dist_total =
      path_node_[id_start_node_].dist_til_now +
      path_node_[id_start_node_].dist_to_end;
  path_node_[id_start_node_].dealed_flag = true;
  node_queue.push_back(id_start_node_);

  int id_now, id_next, dealtime = 0;
  float dtn_tmp;
  bool valid_path = false;
  while (!node_queue.empty()) {
    if ((++dealtime) > 2000) {
      std::cout << "Path not found" << '\n';
      break;
    }

    id_now = node_queue[node_queue.size() - 1];

    if (id_now == id_end_node_) {
      valid_path = true;
      break;
    }
    node_queue.pop_back();

    for (int i = 0; i < path_node_[id_now].id_links.size(); ++i) {
      id_next = path_node_[id_now].id_links[i];

      if (id_next == path_node_[id_now].id_from_where) continue;

      if (!path_node_[id_next].dealed_flag) {
        path_node_[id_next].dist_til_now =
            path_node_[id_now].dist_total +
            path_node_[id_now].dist_links[i];
        path_node_[id_next].id_from_where = id_now;
        path_node_[id_next].dist_to_end = 0;
        path_node_[id_next].dist_total =
            path_node_[id_next].dist_to_end +
            path_node_[id_next].dist_til_now;
        path_node_[id_next].dealed_flag = true;

        insertSortByDistTotal(id_next, node_queue);
      } else {
        dtn_tmp = path_node_[id_now].dist_total +
                  path_node_[id_now].dist_links[i];
        if (dtn_tmp < path_node_[id_next].dist_til_now) {
          path_node_[id_next].dist_til_now = dtn_tmp;
          path_node_[id_next].id_from_where = id_now;
          path_node_[id_next].dist_total =
              path_node_[id_next].dist_to_end +
              path_node_[id_next].dist_til_now;

          int j = 0;
          // erase from queue and readd into queue
          for (j = 0; j < node_queue.size(); ++j) {
            if (node_queue[j] == id_next) {
              node_queue.erase(node_queue.begin() + j);
              insertSortByDistTotal(id_next, node_queue);
              break;
            }
          }
        }
      }
    }
  }
  fs_logger_ << getShortestPathLength() << " ";

  if (!valid_path) {
    std::cout << "no valid path. maybe topomap is broken." << '\n';
    return valid_path;
  }
  // find all nodes
  pcl::PointXYZ temp_point;
  path_.push_back(path_node_.size() - 1);  // push destination first
  temp_point.x = path_node_[path_node_.size() - 1].x;
  temp_point.y = path_node_[path_node_.size() - 1].y;
  temp_point.z = path_node_[path_node_.size() - 1].z;
  path_point_.push_back(temp_point);
  id_now = id_end_node_;
  while (id_now != id_start_node_) {
    path_.push_back(id_now);
    temp_point.x = path_node_[id_now].x;
    temp_point.y = path_node_[id_now].y;
    temp_point.z = path_node_[id_now].z;
    path_point_.push_back(temp_point);
    id_now = path_node_[id_now].id_from_where;
  }
  path_.push_back(id_start_node_);
  path_.push_back(path_node_.size() - 2);  // push start point

  temp_point.x = path_node_[id_start_node_].x;
  temp_point.y = path_node_[id_start_node_].y;
  temp_point.z = path_node_[id_start_node_].z;
  path_point_.push_back(temp_point);
  temp_point.x = path_node_[path_node_.size() - 2].x;
  temp_point.y = path_node_[path_node_.size() - 2].y;
  temp_point.z = path_node_[path_node_.size() - 2].z;
  path_point_.push_back(temp_point);

  // remove the too near node from start/end point
  pcl::PointXYZRGB startorend, nearone;
  startorend.x = path_point_[0].x;
  startorend.y = path_point_[0].y;
  startorend.z = path_point_[0].z;
  nearone.x = path_point_[2].x;
  nearone.y = path_point_[2].y;
  nearone.z = path_point_[2].z;
  if (path_point_.size() >= 3 &&
      !map_oct_.isBlock<Distance>(startorend, nearone, 1.2 * map_point_sparse_)) {
    path_point_.erase(path_point_.begin() + 1);
    path_.erase(path_.begin() + 1);
  }
  startorend.x = path_point_[path_point_.size() - 1].x;
  startorend.y = path_point_[path_point_.size() - 1].y;
  startorend.z = path_point_[path_point_.size() - 1].z;
  nearone.x = path_point_[path_point_.size() - 3].x;
  nearone.y = path_point_[path_point_.size() - 3].y;
  nearone.z = path_point_[path_point_.size() - 3].z;
  if (path_point_.size() >= 3 && !map_oct_.isBlock<Distance>(startorend, nearone, 1.2 * map_point_sparse_)) {
    path_point_.erase(path_point_.begin() + path_point_.size() - 2);
    path_.erase(path_.begin() + path_.size() - 2);
  }

  idp_tgt_node_ = path_.size() - 1;
  pathid_near_node_ = path_.size() - 1;  // ID of path node that drone is near

  just_replan_ = 500;
  return valid_path;
}

// insert and sort path node in queue by distTotal, large at begin, small at end
void FindPathSRM::insertSortByDistTotal(int id, std::vector<int>& node_queue) {
  if (node_queue.empty()) {
    node_queue.push_back(id);
  } else if (path_node_[id].dist_total >
             path_node_[node_queue[0]].dist_total) {
    node_queue.insert(node_queue.begin(), id);
  } else if (path_node_[id].dist_total <
             path_node_[node_queue[node_queue.size() - 1]].dist_total) {
    node_queue.push_back(id);
  } else {
    int start = 0, end = node_queue.size() - 1, half = 0;
    while ((end - start) > 1) {
      half = (end + start) / 2;
      if (path_node_[id].dist_total >
          path_node_[node_queue[half]].dist_total) {
        end = half;
      } else {
        start = half;
      }
    }
    node_queue.insert(node_queue.begin() + end, id);
  }
}

void FindPathSRM::testShowNode() {
  for (int i = 0; i < path_node_.size(); ++i) {
    std::cout << "~~node number:" << i << '\n' << " links:";
    for (int j = 0; j < path_node_[i].id_links.size(); ++j) {
      std::cout << path_node_[i].id_links[j] << " ";
    }
    std::cout << '\n';
  }
}

void FindPathSRM::testShowQueue(std::vector<int> &vec) {
  std::cout << "Queue:";
  for (int i = 0; i < vec.size(); ++i) {
    std::cout << vec[i] << " ";
  }
  std::cout << '\n';
  std::cout << "Distance in total:";
  for (int i = 0; i < vec.size(); ++i) {
    std::cout << path_node_[vec[i]].dist_total << " ";
  }
  std::cout << '\n';
}

void FindPathSRM::restartNavCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  std::cout << "[FindPath] Get restart message\n";
  if (id_end_node_ < 0) {
    return;
  }

  if (pathid_near_node_ < 1) {
    ROS_INFO("[FindPath] Near destinaton and not replaning.");
    return; 
  }

  if (msg->orientation.w < 0) {
    // no safezone
    fs_logger_ << ros::Time::now().toSec() << '\n' << ">>>>>>>>>replan" << '\n';
  } else if (msg->orientation.x > 0) {
    // Turn into wrong path so replan not finished
    float T[3]{now_fvi_pose_.x, now_fvi_pose_.y, now_fvi_pose_.z};
    float modify_tar[3];
    transformNwuWorldFromBody(msg->position.x, msg->position.y, msg->position.z, modify_tar[0], modify_tar[1],
                              modify_tar[2], rwc_, T);
    // cal angle of origin_target-posnow-modify target, if angle > 80degree, the origin road might be blocked
    float vec1[3] = {modify_tar[0] - now_fvi_pose_.x, modify_tar[1] - now_fvi_pose_.y, modify_tar[2] - now_fvi_pose_.z};
    float vec2[3] = {
      path_node_[path_[idp_tgt_node_]].x - now_fvi_pose_.x,
      path_node_[path_[idp_tgt_node_]].y - now_fvi_pose_.y,
      path_node_[path_[idp_tgt_node_]].z - now_fvi_pose_.z
    };
    float cos_angle = (vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]) /
        (sqrt(vec1[0] * vec1[0] + vec1[1] * vec1[1] + vec1[2] * vec1[2]) *
         sqrt(vec2[0] * vec2[0] + vec2[1] * vec2[1] + vec2[2] * vec2[2]));

    if (cos_angle >= 0.17)
      return;
  } else {
    return;
  }

  ROS_INFO("[FindPath] Start replan.");
  int removed_node1 = path_[pathid_near_node_], removed_node2 = path_[pathid_near_node_ - 1];
  float dist = sqrt(pow((path_node_[removed_node1].x - path_node_[removed_node2].x), 2) +
                    pow((path_node_[removed_node1].y - path_node_[removed_node2].y), 2) +
                    pow((path_node_[removed_node1].z - path_node_[removed_node2].z), 2));
  path_node_[path_[pathid_near_node_]].removeLink(path_[pathid_near_node_ - 1]);
  std::cout << "Near path node is:" << pathid_near_node_ << "," << path_node_[path_[pathid_near_node_]].x << "," <<
      path_node_[path_[pathid_near_node_]].y << "," << path_node_[path_[pathid_near_node_]].z << '\n';

  path_node_[path_[pathid_near_node_ - 1]].removeLink(path_[pathid_near_node_]);

  int tmp_nodeid_end = id_end_node_;
  int tmp_pathid_near = path_[pathid_near_node_];

  resetAll(now_pose_.x, now_pose_.y, now_pose_.z, end_pose_.x, end_pose_.y, end_pose_.z);
  id_end_node_ = tmp_nodeid_end;
  id_start_node_ = tmp_pathid_near;

  if (astar<unibn::L2Distance<pcl::PointXYZRGB>>())
    ROS_INFO("[FindPath] Replan ok.");
  else
    ROS_INFO("[FindPath] Replan failed.");

  // Add removed node back?
  path_node_[removed_node1].addLink(removed_node2, dist);
  path_node_[removed_node2].addLink(removed_node1, dist);
}

void FindPathSRM::reloadCb(const std_msgs::StringConstPtr &msg) {
  // den_keyframe_filepath_ = msg->data+"node.txt";
  den_keyframe_filepath_ = msg->data + "denKeyfPos.txt";
  // den_keyframe_relation_filepath_ = msg->data+"edge.txt";
  den_keyframe_relation_filepath_ = msg->data + "denKeyfPosRelation.txt";
  map_points_pos_filepath_ = msg->data + "MapPointsPos.txt";
  std::cout << "Loading new topomap from " << msg->data << '\n';
  if (reconstructGraph())
    ROS_INFO("[FindPath] Reload map done.");
  else
    ROS_WARN("[FindPath] Unknown error while reloading map.");
}

void FindPathSRM::reloadGraph(std::string filepath) {
  den_keyframe_filepath_ = filepath + "denKeyfPos.txt";
  den_keyframe_relation_filepath_ = filepath + "denKeyfPosRelation.txt";
  map_points_pos_filepath_ = filepath + "MapPointsPos.txt";
  std::cout << "Loading new topomap from " << filepath << '\n';
  if (reconstructGraph()) {
    ROS_INFO("[FindPath] Reload map done.");
  } else {
    ROS_WARN("[FindPath] Unknown error while reloading map.");
  }
}

void FindPathSRM::clearAll() {
  resetAll(start_pose_.x, start_pose_.y, start_pose_.z, end_pose_.x, end_pose_.y, end_pose_.z);
  key_pose_pc->clear();
  key_pos_link_.clear();
  map_pc_->clear();
  path_node_.clear();
}
