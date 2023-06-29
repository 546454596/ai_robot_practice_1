// please be attention that:
// pointBODY = Rt * (pointWORLD - T)
//     R0 R1 R2           x
// R = R3 R4 R5 , point = y, Rt * R = E, both q and T are grab from vicon and transfrom into R
//     R6 R7 R8           z
// and you should input the same R and T in forward transform and backward transform
// q = {w,x,y,z}

#ifndef AI_ROBOT_NAVIGATION_UTILS_MATHAUX_H_
#define AI_ROBOT_NAVIGATION_UTILS_MATHAUX_H_

#ifdef OPENCV2
#include <opencv2/opencv.hpp>
#else
#include <opencv/cv.hpp>
#endif
// #include <ros/ros.h>
void eulerToQ(float_t roll, float_t pitch, float_t yaw, float q[]);
double getDistance(float_t dx,float_t dy,float_t dz);
double getDistance(float_t ds[3]);
void qMultiplyQ(float q1[], float q2[], float q_result[]);
void qToEuler(float& roll, float& pitch, float& yaw, float q[]);
void qToRotation(float q[], float r[]);
void rMultiR(float r1[], float r2[], float rout[]);
void rotationToQ(float q[], float r[]);
void rotationToQ(float q[], cv::Mat r, int type=CV_64F);
void rotateBodyFromNwuWorld(const float_t a_nwu[], float_t a_body[], float_t r[]);
void rotateNwuWorldFromBody(const float_t a_body[], float_t a_nwu[], float_t r[]);
void rtMultiR(float r1[], float r2[], float rout[]);
void transformBodyFromNwuWorld(float& body_x, float& body_y, float& body_z, float world_x, 
                               float world_y, float world_z, float r[], float t[]);
void transformNwuWorldFromBody(float body_x, float body_y, float body_z, float& world_x, 
                               float& world_y, float& world_z, float r[], float t[]);

#endif //AI_ROBOT_NAVIGATION_UTILS_MATHAUX_H_
