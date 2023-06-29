#ifndef AI_ROBOT_NAVIGATION_UTILS_PIDCONTROLLER_H_
#define AI_ROBOT_NAVIGATION_UTILS_PIDCONTROLLER_H_

#include <cmath>

// @class	PIDController
// @brief	Object managing one PID control
class PidController {
public:
  // Constructor for PID that saves its settings to EEPROM
  // @note  PIDs must be named to avoid either multiple parameters with the
  //        same name, or an overly complex constructor.
  // @param  initial_p       Initial value for the P term.
  // @param  initial_i       Initial value for the I term.
  // @param  initial_d       Initial value for the D term.
  // @param  initial_imax    Initial value for the imax term.4
  PidController(const float& initial_p = 0.0, const float& initial_i = 0.0,
                const float& initial_d = 0.0, const float& initial_imax = 0.0);

  // Iterate the PID, return the new control value
  // Positive error produces positive output.
  // @param error	The measured error value
  // @param dt		The time delta in milliseconds (note
  // that update interval cannot be more
  // than 65.535 seconds due to limited range
  // of the data type).
  // @param scaler	An arbitrary scale factor
  // @returns		The updated control output.
  float getD(float error, float dt);
  float getI(float error, float dt);
  float getIntegrator() const { return this->integrator_; }
  float getKd() const { return this->kd_; }
  float getKi() const { return this->ki_; }
  float getKp() const { return this->kp_; }
  float getP(float error) const;
  float getPd(float error, float dt);
  float getPi(float error, float dt);
  float getPid(float error, float dt);
  float_t imax() const { return this->imax_; }
  void imax(const int16_t v) { this->imax_ = abs(v); }
  
  // Load gain properties
  void loadGains(const char* filePath);
  // @name	parameter accessors
  // Overload the function call operator to permit relatively easy
  // initialisation
  void operator()(const float p, const float i, const float d, const int16_t imaxval);
  // Reset the PID integrator
  void resetIntegrator();
  // Sets filter Alpha for D-term LPF
  void setDLpfAlpha(int16_t cutoff_frequency, float time_step);
  // accessors
  void setKd(const float v) { this->kd_ = v; }
  void setKi(const float v) { this->ki_ = v; }
  void setKp(const float v) { this->kp_ = v; }
  void setIntegrator(float i) { this->integrator_ = i; }
  // Save gain properties
  void saveGains(const char* filePath);

protected:
  float d_lpf_alpha_;      // alpha used in D-term LPF
  float integrator_;       // integrator value
  float last_input_;       // last input for derivative
  float last_derivative_;  // last derivative for low-pass filter
  float_t imax_;
  float_t kp_;
  float_t ki_;
  float_t kd_;

  // Examples for _filter:
  // f_cut = 10 Hz -> _alpha = 0.385869
  // f_cut = 15 Hz -> _alpha = 0.485194
  // f_cut = 20 Hz -> _alpha = 0.556864
  // f_cut = 25 Hz -> _alpha = 0.611015
  // f_cut = 30 Hz -> _alpha = 0.653373
  // Default 100Hz Filter Rate with 20Hz Cutoff Frequency
  const float kPidControllerDTermFilter = 0.556864;  
  const int kPidControllerIntegralE = 1;
};

#endif  // AI_ROBOT_NAVIGATION_UTILS_PIDCONTROLLER_H_
