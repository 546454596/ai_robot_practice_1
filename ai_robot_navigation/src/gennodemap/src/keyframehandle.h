/*
 * This is used for transforming keyframe into graph that can be
 * used for path finding
 *
 * by XueWuyang
 */

#ifndef KEYFRAMEHANDLE_H
#define KEYFRAMEHANDLE_H

#include <iostream>
#include <cstdlib>
#include <time.h>

#include "Octree.hpp"
#include "utils.h"

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core/core.hpp>

//0 ORB-SLAM
//1 CARTOGRAPHER
//#define DEALMODE 1

using namespace std;

//maybe a adjacent matrix is better to represent edges
class PathNode
{
public:
    PathNode(float _x, float _y, float _z, int _id);
    ~PathNode();
    void setxyzid(float _x, float _y, float _z, int _id);
    void addLink(int _id_link, float _dist);
    void removeLink(int _id_link);
    void replaceLink(int _id_orilink, int _id_afterlink);
    //if is linked, return a positive float, else -1
    float isLinkded(int _id_link);
    //for A star or Dijkstra
    float distTilNow, distToEnd, distTotal;
    int id_fromWhere;
    //link between this node and the other node
    vector<int> id_link;
    //store distance between node
    vector<float> dist_link;
    //flag represent wether the node is dealed
    bool isDealed;
    //store how many nodes are deleted before this node
    int to_delete;
// private:
    float x, y, z;
    //location in the queue
    int id_self;
};

class KeyFrameHandler
{
public:
  KeyFrameHandler(const string &mappointfile, const string &keyframefile);
  KeyFrameHandler(const string &paramfile);
  ~KeyFrameHandler();
  //deal with keyframe
  void dealKeyFrame();
  //display map points and keyframes and result
  void display();
  //save keyframe position and link relation
  void saveResult();
  void saveResult(string savepath);
  void saveResultPN();
  void saveResultPN(string savepath);
  void saveResultOnlyFinal(string savepath);
  //reset input map points and trajectory
  void reset(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _mapPC, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _keyfPC);
  //reset input with save path
  void reset(string savepath);
  //clear all data
  void clearall();

  //private:
  int DEALMODE;
  //store map points pos
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPC;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPCorigin;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPCfinal;
  //octree of map point
  unibn::Octree<pcl::PointXYZRGB> mapOct;
  unibn::Octree<pcl::PointXYZRGB> mapOctfinal;
  float mappointSparse;
  //store keyframe pos
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyfPC;
  //octree of keyframe
  unibn::Octree<pcl::PointXYZRGB> keyfOct;
  //store dense keyframe pos
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr denkeyfPC;
  vector< vector<int> > denkeyfLine;
  // store graph in path nodes
  vector<PathNode> pNode;
  vector<int> to_delete_vec;
  float topPercent;
  float minKeyFdist;
  float maxlinkdist;
  //for pcl viewer
  int v1, v2;
  bool isview;
  //remove area
  vector< vector<float> > removeMappointArea;
  //
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  // is near zero position has a node?
  bool near_zero_dealt;
  // back up center should has node nearby
  vector<PathNode> backupNode;

  //read map points to mapPC
  void readMapPt(const string &mpfile);
  //read keyframe to keyfPC
  void readKeyFrame(const string &kffile);
  //read back up position for actloc
  void readBackupPos(const string &bcpfile);
  //read parameters from file
  void readParams(const string &paramfile);
  //read the region where you want to remove mappoint
  void readRemoveMapP();
  //init pcl viewer
  void initPclViewer();
  //for i=12345, change str(line00000) to str(line12345)
  void setstring(string &str, int k);
  //find dense keyframe pos
  void findDenseKeyFrame();
  //line all the dense keyframe pos
  void lineDenseKeyFrame();
  //remove redundant lines
  void removeRedundantLine();
  //remove redundant nodes
  void removeRedundantNode();
  //walk along path to next crossing
  void goToCheckNextCrossing(int _id_lastcrossing, int _id_nextnode);
  //kill wrong mappoint using keyframe
  void killErrMapP();
  //find all points along keyframe
  void findPointAlongTwoPos(std::vector<uint32_t>& results, pcl::PointXYZRGB p1, pcl::PointXYZRGB p2);

  // 2-norm of two point
  double norm(PathNode p1, PathNode p2);
};

#endif
