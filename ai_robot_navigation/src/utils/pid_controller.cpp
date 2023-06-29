#include <cmath>
#include <fstream>
#include <iostream>

#include "utils/pid_controller.h"

PidController::PidController(const float& initial_p,
                             const float& initial_i,
                             const float& initial_d,
                             const float& initial_imax)
    : integrator_(0),
      last_input_(0),
      last_derivative_(0),
      d_lpf_alpha_(kPidControllerDTermFilter) {
  this->kp_ = initial_p;
  this->ki_ = initial_i;
  this->kd_ = initial_d;
  this->imax_ = fabs(initial_imax);
  std::cout << "PID: kP:" << this->kp_ << " kI:" << this->ki_
            << " kD:" << this->kd_ << " imax:" << this->imax_ << '\n';
  // derivative is invalid on startup
  this->last_derivative_ = NAN;
}

float PidController::getP(float error) const {
  return (float)error * this->kp_;
}

float PidController::getI(float error, float dt) {
  if ((this->ki_ != 0) && (dt != 0)) {
    if (fabs(error) < kPidControllerIntegralE) {
      this->integrator_ += ((float)error * this->ki_) * dt;
      if (this->integrator_ < -this->imax_) {
        this->integrator_ = -this->imax_;
      } else if (this->integrator_ > this->imax_) {
        this->integrator_ = this->imax_;
      }
      return this->integrator_;
    } else {
      return 0;
    }
  }
  return 0;
}

float PidController::getD(float input, float dt) {
  if ((this->kd_ != 0) && (dt != 0)) {
    float derivative;
    if (std::isnan(this->last_derivative_)) {
      // we've just done a reset, suppress the first derivative
      // term as we don't want a sudden change in input to cause
      // a large D output change
      derivative = 0;
      this->last_derivative_ = 0;
    } else {
      // calculate instantaneous derivative
      derivative = (input - this->last_input_) / dt;
    }

    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    derivative = this->last_derivative_ +
                 this->d_lpf_alpha_ * (derivative - this->last_derivative_);

    // update state
    this->last_input_ = input;
    this->last_derivative_ = derivative;

    // add in derivative component
    return this->kd_ * derivative;
  }
  return 0;
}

float PidController::getPi(float error, float dt) {
  return getP(error) + getI(error, dt);
}

float PidController::getPd(float error, float dt) {
  return getP(error) + getD(error, dt);
}

float PidController::getPid(float error, float dt) {
  return getP(error) + getI(error, dt) + getD(error, dt);
}

void PidController::resetIntegrator() {
  this->integrator_ = 0;
  // mark derivative as invalid
  this->last_derivative_ = NAN;
}

void PidController::loadGains(const char *filePath) {
  std::ifstream file(filePath);
  file >> this->kp_ >> this->ki_ >> this->kd_ >> this->imax_;
  std::cout << " kP:" << this->kp_ << " kI:" << this->ki_ << " kD:" << this->kd_
            << " imax:" << this->imax_ << '\n';
  this->imax_ = fabs(this->imax_);
  file.close();
}

void PidController::saveGains(const char *filePath) {
  std::ofstream file(filePath);
  file << this->kp_ << " " << this->ki_ << " " << this->kd_ << " " << this->imax_;
  file.close();
}

void PidController::setDLpfAlpha(int16_t cutoff_frequency, float time_step) {
  // calculate alpha
  float rc = 1 / (2 * M_PI * cutoff_frequency);
  this->d_lpf_alpha_ = time_step / (time_step + rc);
}

void PidController::operator()(const float p, const float i, const float d,
                               const int16_t imaxval) {
  this->kp_ = p;
  this->ki_ = i;
  this->kd_ = d;
  this->imax_ = abs(imaxval);
}