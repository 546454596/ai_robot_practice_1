#include "utils/math_aux.h"

#include <cmath>

void qToEuler(float& roll, float& pitch, float& yaw, float q[]) {
  float_t w, x, y, z;
  // diffrent coordinates
  w = q[0];
  x = q[1];
  y = q[2];
  z = q[3];
  if (fabs(w) + fabs(x) + fabs(y) + fabs(z) < 0.1) {
    roll = 0;
    pitch = 0;
    yaw = 0;
  }
  roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));

  if (2 * (w * y - z * x) > 1)
    pitch = asin(1.0);
  else if (2 * (w * y - z * x) < -1.0)
    pitch = asin(-1.0);
  else
    pitch = asin(2 * (w * y - z * x));
  yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

void eulerToQ(float_t roll, float_t pitch, float_t yaw, float q[]) {
  float fCosHRoll = cos(roll * .5f);
  float fSinHRoll = sin(roll * .5f);
  float fCosHPitch = cos(pitch * .5f);
  float fSinHPitch = sin(pitch * .5f);
  float fCosHYaw = cos(yaw * .5f);
  float fSinHYaw = sin(yaw * .5f);

  /// Cartesian coordinate System
  q[0] = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
  q[1] = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
  q[2] = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
  q[3] = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
}

void qMultiplyQ(float q1[], float q2[], float q_result[]) {
  q_result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
  q_result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
  q_result[2] = q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3];
  q_result[3] = q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1];
}

void qToRotation(float q[], float r[]) {
  float_t w, x, y, z;
  // coordinates in NWU
  w = q[0];
  x = q[1];
  y = q[2];
  z = q[3];

  r[0] = w * w + x * x - y * y - z * z;
  r[1] = 2 * (x * y - w * z);
  r[2] = 2 * (x * z + w * y);
  r[3] = 2 * (x * y + w * z);
  r[4] = w * w - x * x + y * y - z * z;
  r[5] = 2 * (y * z - w * x);
  r[6] = 2 * (x * z - w * y);
  r[7] = 2 * (y * z + w * x);
  r[8] = w * w - x * x - y * y + z * z;
}

void rotationToQ(float q[], float r[]) {
  // q[0] = sqrt(1 + R[0] + R[4] + R[8]) / 2;
  float tr = r[0] + r[4] + r[8];
  if (tr <= 0) { // fabs(q[0]) < 0.001
    if (r[0] > r[4] && r[0] > r[8]) {
      float s = sqrt(1 + r[0] - r[4] - r[8]) * 2;
      q[0] = (r[7] - r[5]) / s;
      q[1] = s / 4;
      q[2] = (r[1] + r[3]) / s;
      q[3] = (r[2] + r[6]) / s;
    } else if (r[4] > r[8]) {
      float s = sqrt(1 - r[0] + r[4] - r[8]) * 2;
      q[0] = (r[2] - r[6]) / s;
      q[1] = (r[1] + r[3]) / s;
      q[2] = s / 4;
      q[3] = (r[5] + r[7]) / s;
    } else {
      float s = sqrt(1 - r[0] - r[4] + r[8]) * 2;
      q[0] = (r[3] - r[1]) / s;
      q[1] = (r[2] + r[6]) / s;
      q[2] = (r[5] + r[7]) / s;
      q[3] = s / 4;
    }
    return;
  }
  q[0] = sqrt(1 + r[0] + r[4] + r[8]) / 2;
  q[1] = (r[7] - r[5]) / 4 / q[0];
  q[2] = (r[2] - r[6]) / 4 / q[0];
  q[3] = (r[3] - r[1]) / 4 / q[0];
}

void rotationToQ(float q[], cv::Mat r, int type) {
  if (type == CV_32F) {
    q[0] = sqrt(1 + r.at<float>(0, 0) + r.at<float>(1, 1) + r.at<float>(2, 2)) / 2;
    q[1] = (r.at<float>(2, 1) - r.at<float>(1, 2)) / (4 * q[0]);
    q[2] = (r.at<float>(0, 2) - r.at<float>(2, 0)) / (4 * q[0]);
    q[3] = (r.at<float>(1, 0) - r.at<float>(0, 1)) / (4 * q[0]);
  } else if (type == CV_64F) {
    q[0] = sqrt(1 + r.at<double>(0, 0) + r.at<double>(1, 1) + r.at<double>(2, 2)) / 2;
    q[1] = (r.at<double>(2, 1) - r.at<double>(1, 2)) / (4 * q[0]);
    q[2] = (r.at<double>(0, 2) - r.at<double>(2, 0)) / (4 * q[0]);
    q[3] = (r.at<double>(1, 0) - r.at<double>(0, 1)) / (4 * q[0]);
  }
}

void rotateBodyFromNwuWorld(const float_t a_nwu[], float_t a_body[], float_t r[]) {
  for (int i = 0; i < 3; ++i) {
    a_body[i] = 0;
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      a_body[i] += r[3 * j + i] * a_nwu[j];
    }
  }
}

void rotateNwuWorldFromBody(const float_t a_body[], float_t a_nwu[], float_t r[]) {
  for (int i = 0; i < 3; ++i) {
    a_nwu[i] = 0;
  }
  
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      a_nwu[i] += r[3 * i + j] * a_body[j];
    }
  }
}

// pointBODY = R * (pointWORLD - T) --convert from transform_NWUworld_from_body
void transformBodyFromNwuWorld(float& body_x, float& body_y, float& body_z, float world_x, float world_y, float world_z,
                               float r[], float t[]) { float dx = world_x - t[0];
  float dy = world_y - t[1];
  float dz = world_z - t[2];
  body_x = r[0] * dx + r[3] * dy + r[6] * dz;
  body_y = r[1] * dx + r[4] * dy + r[7] * dz;
  body_z = r[2] * dx + r[5] * dy + r[8] * dz;
}

// R * pointBODY + T = pointWORLD -- test with q and T from vicon and rgbd camera
void transformNwuWorldFromBody(float body_x, float body_y, float body_z, float& world_x, float& world_y, float& world_z,
                               float r[], float t[]) {
  float dx = r[0] * body_x + r[1] * body_y + r[2] * body_z;
  float dy = r[3] * body_x + r[4] * body_y + r[5] * body_z;
  float dz = r[6] * body_x + r[7] * body_y + r[8] * body_z;
  world_x = dx + t[0];
  world_y = dy + t[1];
  world_z = dz + t[2];
}

double getDistance(float_t dx, float_t dy, float_t dz) {
  return sqrt(dx * dx + dy * dy + dz * dz);
}

double getDistance(float_t ds[3]) {
  return sqrt(ds[0] * ds[0] + ds[1] * ds[1] + ds[2] * ds[2]);
}

void rMultiR(float r1[], float r2[], float rout[]) {
  rout[0] = 0;
  rout[1] = 0;
  rout[2] = 0;
  rout[3] = 0;
  rout[4] = 0;
  rout[5] = 0;
  rout[6] = 0;
  rout[7] = 0;
  rout[8] = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        rout[i * 3 + j] += r1[i * 3 + k] * r2[k * 3 + j];
      }
    }
  }
}

void rtMultiR(float r1[], float r2[], float rout[]) {
  rout[0] = r1[0] * r2[0] + r1[3] * r2[3] + r1[6] * r2[6];
  rout[1] = r1[0] * r2[1] + r1[3] * r2[4] + r1[6] * r2[7];
  rout[2] = r1[0] * r2[2] + r1[3] * r2[5] + r1[6] * r2[8];
  rout[3] = r1[1] * r2[0] + r1[4] * r2[3] + r1[7] * r2[6];
  rout[4] = r1[1] * r2[1] + r1[4] * r2[4] + r1[7] * r2[7];
  rout[5] = r1[1] * r2[2] + r1[4] * r2[5] + r1[7] * r2[8];
  rout[6] = r1[2] * r2[0] + r1[5] * r2[3] + r1[8] * r2[6];
  rout[7] = r1[2] * r2[1] + r1[5] * r2[4] + r1[8] * r2[7];
  rout[8] = r1[2] * r2[2] + r1[5] * r2[5] + r1[8] * r2[8];
}
