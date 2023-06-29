#ifndef AI_ROBOT_NAVIGATION_UTILS_DARW_MATCHES_H_
#define AI_ROBOT_NAVIGATION_UTILS_DARW_MATCHES_H_

#include <vector>

#ifdef OPENCV2
#include <opencv2/opencv.hpp>
#else
#include <opencv/cv.hpp>
#endif

const int kDrawShiftBits = 4;
const int kDrawMultiplier = 1 << kDrawShiftBits;

void drawMatches(const cv::Mat& img1, const std::vector<cv::Point2f>& keypoints1,
                 const cv::Mat& img2, const std::vector<cv::Point2f>& keypoints2, cv::Mat& out_img, 
                 const Scalar& match_color, const Scalar& single_point_color, int flags);

#endif //AI_ROBOT_NAVIGATION_UTILS_DARW_MATCHES_H_
