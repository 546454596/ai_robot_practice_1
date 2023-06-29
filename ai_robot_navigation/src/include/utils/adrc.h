#ifndef AI_ROBOT_NAVIGATION_UTILS_ADRC_H_
#define AI_ROBOT_NAVIGATION_UTILS_ADRC_H_

#include <Eigen/Core>

#include "eso.h"

class Adrc {
public:
  Adrc(float wo, float wc, float b0, float dt);
  virtual ~Adrc(){};
  float updateController(float u, float y, float y_desired);

private:
  float getController(const Eigen::Vector3f& xhat, float y_desired);

private:
  Eso observer_;
  float kp_;
  float kd_;
  float b_;
};

#endif  // AI_ROBOT_NAVIGATION_UTILS_ADRC_H_
