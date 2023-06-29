#include <boost/smart_ptr/make_shared_array.hpp>
#include <getopt.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdarg>
#include <cstdbool>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>

#ifdef OPENCV2
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#else
#include <opencv/cv.hpp>
#endif

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

constexpr int kTOF = 1;
constexpr int kLidar3D = 2;

int data_type = kLidar3D;
int vp1 = 5; // viewport
int last_safezone_size = 0;
std::shared_ptr<pcl::visualization::PCLVisualizer> obsviewer = std::make_shared<pcl::visualization::PCLVisualizer>("Obstacle Viewer");
pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

static void disang_to_xz(float dis, float ang, float& x, float& z) {
  z = dis * cos(ang);
  x = -dis * sin(ang);
}

static void disang_to_xy(float dis, float ang, float& x, float& y) {
  y = dis * sin(ang);
  x = dis * cos(ang);
}

void initPclViewer() {
  obsviewer->initCameraParameters();
  obsviewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp1);
  obsviewer->setBackgroundColor(255, 255, 255, vp1);
  obsviewer->addCoordinateSystem(0.2, "Obstacle Viewer", vp1);
  obsviewer->addPointCloud(pc, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pc, 255, 0, 0), "obstacle");
  pcl::PointXYZ lp0(.0, -0.3, .0), lp1(0, -0.3, 2);
  obsviewer->addLine(lp0, lp1, 0, 250, 0, "oridir", vp1);
  obsviewer->addLine(lp0, lp1, 0, 250, 160, "moddir", vp1);
  pcl::PointXYZ basic_point(.0, .0, .0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ppcc(new pcl::PointCloud<pcl::PointXYZ>);

  if (data_type == kTOF) {
    for (float i = .0; i <= 2 * M_PI; i += M_PI / 360) {
      basic_point.x = 2 * cos(i);
      basic_point.z = 2 * sin(i);
      ppcc->points.push_back(basic_point);
    }
  } else if (data_type == kLidar3D) {
    for (float i = .0; i <= 2 * M_PI; i += M_PI / 360) {
      basic_point.x = 2 * cos(i);
      basic_point.y = 2 * sin(i);
      ppcc->points.push_back(basic_point);
    }
  }

  obsviewer->addPointCloud(ppcc, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(ppcc, 0, 200, 0), "2m");
}

static void pclCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::fromROSMsg(*msg, *pc);
  obsviewer->removePointCloud("obstacle");
  obsviewer->addPointCloud(pc, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pc, 180, 0, 0),
                           "obstacle"); }

static void safezoneCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  int safezone_size = (msg->data.size() - 2) / 4;
  char num[5];
  std::string lname;
  std::string l = "line";
  std::string n = "dir";
  std::stringstream stream;
  pcl::PointXYZ lp0(0, 0, 0), lp1(0, 0, 0);

  for (int i = 0; i < last_safezone_size; ++i) {
    stream << i;
    lname = l + stream.str();
    obsviewer->removeShape(lname, vp1);
    lname.clear();
    stream.clear();
    stream.str("");
  }

  if (data_type == kTOF)
    disang_to_xz(1.5, msg->data[msg->data.size() - 1], lp0.x, lp0.z);
  else if (data_type == kLidar3D)
    disang_to_xy(1.5, msg->data[msg->data.size() - 1], lp0.x, lp0.y);

  obsviewer->removeShape("oridir", vp1);
  obsviewer->removeShape("moddir", vp1);
  obsviewer->addLine(lp0, lp1, 0, 250, 160, "moddir", vp1);

  if (data_type == kTOF)
    disang_to_xz(1.5, msg->data[msg->data.size() - 2], lp0.x, lp0.z);
  else if (data_type == kLidar3D)
    disang_to_xy(1.5, msg->data[msg->data.size() - 2], lp0.x, lp0.y);

  obsviewer->addLine(lp0, lp1, 0, 250, 0, "oridir", vp1);

  for (int i = 0; i < safezone_size; ++i) {
    stream << i;
    lname = l + stream.str();

    if (data_type == kTOF) {
      disang_to_xz(msg->data[4 * i], msg->data[4 * i + 1], lp0.x, lp0.z);
      disang_to_xz(msg->data[4 * i + 2], msg->data[4 * i + 3], lp1.x, lp1.z);
    } else if (data_type == kLidar3D) {
      disang_to_xy(msg->data[4 * i], msg->data[4 * i + 1], lp0.x, lp0.y);
      disang_to_xy(msg->data[4 * i + 2], msg->data[4 * i + 3], lp1.x, lp1.y);
    }

    obsviewer->addLine(lp0, lp1, 0, 0, 250, lname, vp1);
    lname.clear();
    stream.clear();
    stream.str("");
  }

  last_safezone_size = safezone_size;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pclviewer");
  ros::NodeHandle nh;
  ros::Rate rate(30);

  nh.param("data_type", data_type, kLidar3D);

  ros::Subscriber pcl_sub = nh.subscribe("/points", 1, pclCallback);
  ros::Subscriber sz_sub = nh.subscribe("/obstacle/safezone", 1, safezoneCallback);

  initPclViewer();

  switch (data_type) {
  case kTOF:
    ROS_INFO("[OBS] Point cloud viewer started with 3d lidar mode!");
    break;
  case kLidar3D:
    ROS_INFO("[OBS] Point cloud viewer started with depth camera mode!");
    break;
  default:
    break;
  }

  while (ros::ok())
  {
    obsviewer->spinOnce(1);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
