#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

#include "findpath/findpath_srm.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "testfp");
  ros::NodeHandle nh;
  ros::Rate r(50);
  int64_t start1 = 0, end1 = 0;
  FindPathSRM fpsrm(nh, true);

  // read start end nodes
  std::string paramfile;
  nh.getParam("paramfile", paramfile);
  std::cout << "st pos file path " << paramfile << '\n';
  cv::FileStorage fsSettings(paramfile.c_str(), cv::FileStorage::READ);
  cv::Mat tmp;
  fsSettings["startendpos"] >> tmp;
  std::vector<std::vector<double>> stenpos;
  std::vector<double> stentmp;
  stentmp.resize(6);
  for (int i = 0; i < tmp.rows; ++i) {
    stentmp[0] = double(tmp.at<double>(i, 0));
    stentmp[1] = double(tmp.at<double>(i, 1));
    stentmp[2] = double(tmp.at<double>(i, 2));
    stentmp[3] = double(tmp.at<double>(i, 3));
    stentmp[4] = double(tmp.at<double>(i, 4));
    stentmp[5] = double(tmp.at<double>(i, 5));
    stenpos.push_back(stentmp);
    // cout<<"read start end node ["<<stentmp[0]<<", "<<stentmp[1]<<"],
    // ["<<stentmp[2]<<", "<<stentmp[3]<<"]"<<endl;
  }
  fpsrm.displayPointCloud();
  int a;
  std::cin >> a;

  int i = 0, testtime = 10;
  while (true) {  // ros::ok()
    if (i >= stenpos.size()) break;
    std::cout << "start end node [" << stenpos[i][0] << ", " << stenpos[i][1]
              << ", " << stenpos[i][2] << "], [" << stenpos[i][3] << ", "
              << stenpos[i][4] << ", " << stenpos[i][5] << "]" << '\n';
    // start1 = cv::getTickCount();
    // findpath
    for (int j = 0; j < testtime; ++j) {
      fpsrm.resetAll(stenpos[i][0], stenpos[i][1], stenpos[i][2], 
                     stenpos[i][3], stenpos[0][4], stenpos[0][5]);
      fpsrm.findPath();
    }

    ++i;
  }
  std::cout << "test end." << '\n';

  ros::shutdown();
  return 0;
}