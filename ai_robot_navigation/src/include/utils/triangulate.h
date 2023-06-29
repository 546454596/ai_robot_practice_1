#ifndef AI_ROBOT_NAVIGATON_UTILS_TRANGULATE_H_
#define AI_ROBOT_NAVIGATON_UTILS_TRANGULATE_H_

#ifdef OPENCV2
#include <opencv2/opencv.hpp>
#else
#include <opencv/cv.hpp>
#endif

void cvTriangulatePoints(cv::CvMat *proj_matr1, cv::CvMat *proj_matr2,
                         cv::CvMat *proj_points1, cv::CvMat *proj_points2,
                         cv::CvMat *points_4d);

void triangulatePoints(cv::InputArray proj_matr1, cv::InputArray proj_matr2,
                       cv::InputArray proj_points1, cv::InputArray proj_points2,
                       cv::OutputArray points_4d);

#endif  // AI_ROBOT_NAVIGATON_UTILS_TRANGULATE_H_
