#include "utils/adrc.h"

#include <cstdio>

using Eigen::Vector3f;

Adrc::Adrc(float wo, float wc, float b0, float dt)
    : observer_(wo, b0, dt), b_(b0) {
  this->kp_ = wc * wc;
  this->kd_ = wc + wc;
  printf("ADRC:wo is %f, wc is %f, b0 is %f,dt is %f\n", wo, wc, b0, dt);
}

float Adrc::getController(const Vector3f& xhat, float y_desired) {
  float u0 = this->kp_ * (y_desired - xhat[0]) - this->kd_ * xhat[1];
  return (u0 - xhat[2]) / this->b_;
}

float Adrc::updateController(float u, float y, float y_desired) {
  // Vector3f xhat = this->observer_.Update(u, y);
  return getController(this->observer_.update(u, y), y_desired);
}
