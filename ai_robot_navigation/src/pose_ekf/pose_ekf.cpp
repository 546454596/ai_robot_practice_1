#include "pose_ekf.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

Matrix3d skewSymmetric(Vector3d v) {
  Matrix3d m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

// diff_(p*q) /diff_q
Matrix4d diffPqQ(Quaterniond p) {
  double p0 = p.w();
  Vector3d pv = p.vec();

  Matrix4d d;
  d(0, 0) = p0;
  d.block<1, 3>(0, 1) = -pv.transpose();
  d.block<3, 1>(1, 0) = pv;
  d.block<3, 3>(1, 1) = Matrix3d::Identity() * p0 + skewSymmetric(pv);
  return d;
}

// diff_(p*q)/ diff_p
Matrix4d diffPqP(Quaterniond q) {
  double q0 = q.w();
  Vector3d qv = q.vec();
  Matrix4d d;
  d(0, 0) = q0;
  d.block<1, 3>(0, 1) = -qv.transpose();
  d.block<3, 1>(1, 0) = qv;
  d.block<3, 3>(1, 1) = Matrix3d::Identity() * q0 - skewSymmetric(qv);
  return d;
}

// diff_(q*v*q_star)/ diff_q
MatrixXd diffQvqstarQ(Quaterniond q, Vector3d v) {
  double q0 = q.w();
  Vector3d qv = q.vec();
  MatrixXd d(3, 4);
  d.col(0) = 2 * (q0 * v + skewSymmetric(qv) * v);
  d.block<3, 3>(0, 1) = 2 * (-v * qv.transpose() + v.dot(qv) * Matrix3d::Identity() - q0 * skewSymmetric(v));
  return d;
}

// diff_(qstar*v*q)/ diff_q
MatrixXd diffQstarvqQ(Quaterniond q, Vector3d v) {
  double q0 = q.w();
  Vector3d qv = q.vec();
  MatrixXd d(3, 4);
  d.col(0) = 2 * (q0 * v - skewSymmetric(qv) * v);
  d.block<3, 3>(0, 1) = 2 * (-v * qv.transpose() + v.dot(qv) * Matrix3d::Identity() + q0 * skewSymmetric(v));
  return d;
}
// diff_(q*v*q_star)/ diff_v
Matrix3d diffQvqstarV(Quaterniond q) {
  double q0 = q.w();
  Vector3d qv = q.vec();
  Matrix3d d;
  d = (q0 * q0 - qv.dot(qv)) * Matrix3d::Identity() + 2 * qv * qv.transpose() + 2 * q0 * skewSymmetric(qv);
  return d;
}

PoseEkf::PoseEkf()
    : altimeter_initialized_flag_(false),
      fix_initialized_flag_(false),
      imu_initialized_flag_(false),
      initialized_flag_(false),
      opt_q(1.0),
      optical_initialized_flag_(false),
      sonar_initialized_flag_(false) {
  this->x_ = VectorXd::Zero(this->kNState);
  this->p_ = MatrixXd::Identity(this->kNState, this->kNState);
  this->q_ = MatrixXd::Zero(3, 3);
  this->q_.block<3, 3>(0, 0) = Matrix3d::Identity() * this->kAccCov;
}

PoseEkf::~PoseEkf() {}

void PoseEkf::predict(Vector4d quan, Vector3d acc, double t) {
  if (!this->imu_initialized_flag_) {
    this->imu_initialized_flag_ = true;
    this->initialized_flag_ = true;
    this->cur_time_ = t;
    return;
  }
  if (t <= this->cur_time_) return;

  double dt = t - this->cur_time_;
  VectorXd xdot(this->kNState);
  MatrixXd f(this->kNState, this->kNState);
  MatrixXd g(this->kNState, 3);  // G = dx/du

  process(quan, acc, xdot, f, g);

  this->x_ += xdot * dt;
  f = MatrixXd::Identity(this->kNState, this->kNState) +
      f * dt;  // continous F and discrete F
  g = g * dt;
  // cout << "G: " << G << endl;
  // cout << "GQG: " << G*Q*G << endl;

  this->p_ = f * this->p_ * f.transpose() + g * this->q_ * g.transpose();

  this->cur_time_ = t;
  this->acc_ = acc;
  this->quan_ = quan;
}

void PoseEkf::process(Vector4d quan, Vector3d acc, VectorXd& xdot, 
                      MatrixXd& f, MatrixXd& g) {
  Quaterniond q;
  Vector3d p, v, ba;
  getState(q, p, v, ba);

  xdot.setZero();
  f.setZero();
  g.setZero();

  xdot.segment<3>(0) = v;

  Quaterniond acc_b_q(0, 0, 0, 0);
  acc_b_q.vec() = acc - ba;
  Quaterniond acc_n_q = q * acc_b_q * q.inverse();
  xdot.segment<3>(3) = acc_n_q.vec() - this->gravity_;  // body frame to n frame

  f.block<3, 3>(0, 3) = Matrix3d::Identity();
  f.block<3, 3>(3, 6) = -diffQvqstarV(q);

  // G = d_xdot/du
  // diff_qvqstar_v(q);//diff(q*a*qstar)/diff(a)
  g.block<3, 3>(3, 0) = Matrix3d::Identity();  
}

void PoseEkf::getState(Quaterniond& q, Vector3d& p, Vector3d& v, Vector3d& ba) {
  q.w() = this->quan_(0); 
  q.vec() = this->quan_.segment<3>(1);
  p = this->x_.segment<3>(0);
  v = this->x_.segment<3>(3);
  ba = this->x_.segment<3>(6);
}

void PoseEkf::getPoseCorrectCov(Vector2d d_position) { 
  r_pose_correct_; 
}

void PoseEkf::measurementFix(Vector2d& position, MatrixXd& H) {
  position = this->x_.segment<2>(0);
  H = MatrixXd::Zero(2, this->kNState);
  H.block<2, 2>(0, 0) = Matrix2d::Identity();
}

void PoseEkf::measurementFixVelocity(Vector3d& velocity, MatrixXd& H) {
  velocity = this->x_.segment<3>(3);
  H = MatrixXd::Zero(3, this->kNState);
  H.block<3, 3>(0, 3) = Matrix3d::Identity();
}

void PoseEkf::measurementSonarHeight(VectorXd& sonar_height, MatrixXd& H) {
  sonar_height = VectorXd(1);
  sonar_height(0) = this->x_(2);
  H = MatrixXd::Zero(1, this->kNState);
  H(0, 2) = 1;
}

void PoseEkf::measurementOptcalFlow(Vector2d& opt_velocity, MatrixXd& H) {
  opt_velocity = this->x_.segment<2>(3);
  H = MatrixXd::Zero(2, this->kNState);
  H.block<2, 2>(0, 3) = Matrix2d::Identity();
}

void PoseEkf::measurementPoseCorrect(Vector2d& position, MatrixXd& H) {
  position = this->x_.segment<2>(0);
  H = MatrixXd::Zero(2, this->kNState);
  H.block<2, 2>(0, 0) = Matrix2d::Identity();
}

void PoseEkf::measurementSlamPose(Vector3d& position, MatrixXd& H) {
  position = this->x_.segment<3>(0);
  H = MatrixXd::Zero(3, this->kNState);
  H.block<3, 3>(0, 0) = Matrix3d::Identity();
}

void PoseEkf::measurementViconPose(Vector3d& position, MatrixXd& H) {
  position = this->x_.segment<3>(0);
  H = MatrixXd::Zero(3, this->kNState);
  H.block<3, 3>(0, 0) = Matrix3d::Identity();
}

void PoseEkf::measurementViconVel(Vector3d& velocity, MatrixXd& h) {
  velocity = this->x_.segment<3>(3);
  h = MatrixXd::Zero(3, this->kNState);
  h.block<3, 3>(0, 3) = Matrix3d::Identity();
}

void PoseEkf::measurementGravity(Vector3d& acc, MatrixXd& H) {
  Quaterniond q;
  q.w() = this->quan_(0);
  q.vec() = this->quan_.segment<3>(1);
  Vector3d ba = this->x_.segment<3>(6);
  Quaterniond g_n_q;
  g_n_q.w() = 0;
  g_n_q.vec() = Vector3d(0, 0, 1);              // only direction is used
  Quaterniond acc_q = q.inverse() * g_n_q * q;  // r_n to r_b
  acc = acc_q.vec();

  H = MatrixXd::Zero(3, this->kNState);
  H.block<3, 3>(0, 6) = Matrix3d::Identity();
}

void PoseEkf::correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R) {
  MatrixXd K = this->p_ * H.transpose() * (H * this->p_ * H.transpose() + R).inverse();
  this->x_ += K * (z - zhat);

  MatrixXd I = MatrixXd::Identity(this->kNState, this->kNState);
  this->p_ = (I - K * H) * this->p_;
}

void PoseEkf::correctFix(Vector3d position, double t) {
  if (!this->initialized_flag_) {
    this->initialized_flag_ = true;
    this->cur_time_ = t;
    return;
  }

  if (t < this->cur_time_) return;

  predict(this->quan_, this->acc_, t);
  double dt = t - this->cur_time_;
  Vector2d z = position.head(2);
  Vector2d zhat;
  MatrixXd H;
  measurementFix(zhat, H);
  correct(z, zhat, H, this->r_fix_);
}
void PoseEkf::correctFixVelocity(Vector3d velocity, double t) {
  if (!this->initialized_flag_) {
    this->initialized_flag_ = true;
    this->cur_time_ = t;
    return;
  }

  if (t < this->cur_time_) return;

  predict(this->quan_, this->acc_, t);

  Vector3d z = velocity;
  Vector3d zhat;
  MatrixXd H;
  measurementFixVelocity(zhat, H);
  correct(z, zhat, H, this->r_fix_velocity_);
}
void PoseEkf::correctSonarHeight(double sonar_height, double t) {
  if (!this->initialized_flag_) {
    this->initialized_flag_ = true;
    this->cur_time_ = t;
    return;
  }

  if (t < this->cur_time_) return;
  predict(this->quan_, this->acc_, t);

  VectorXd z(1);
  z(0) = sonar_height;
  VectorXd zhat(1);
  MatrixXd H;

  measurementSonarHeight(zhat, H);
  correct(z, zhat, H, this->r_sonar_height_);
}

void PoseEkf::correctOptVelocity(Vector2d opt_velocity, double t) {
  if (!this->initialized_flag_) {
    this->initialized_flag_ = true;
    this->cur_time_ = t;
    return;
  }

  if (t < this->cur_time_) return;

  predict(this->quan_, this->acc_, t);

  Quaterniond q;
  q.w() = this->quan_(0);
  q.vec() = this->quan_.segment<3>(1);

  Matrix3d OptM3 = quaternion2Mat(q);
  Vector3d velocityBody;
  velocityBody.segment<2>(0) = opt_velocity;
  velocityBody(2) = 0;
  Vector3d velocityWorld = OptM3 * velocityBody;
  Vector2d z = velocityWorld.segment<2>(0);

  Vector2d zhat;
  MatrixXd H;
  measurementOptcalFlow(zhat, H);
  correct(z, zhat, H, opt_q * this->r_optcal_velocity_);
}

void PoseEkf::correctGravity(Vector3d acc, double t) {
  if (!this->initialized_flag_) {
    this->initialized_flag_ = true;
    this->cur_time_ = t;
    return;
  }
  
  if (t < this->cur_time_) return;
  predict(this->quan_, this->acc_, t);

  Vector3d z = acc / acc.norm();
  Vector3d zhat;
  MatrixXd H;
  measurementGravity(zhat, H);
  correct(z, zhat, H, this->r_gravity_);
}

void PoseEkf::correctPoseCorrect(Vector2d d_position, Vector4d d_q, double t) {
  if (!this->initialized_flag_) {
    this->initialized_flag_ = true;
    this->cur_time_ = t;
    return;
  }

  if (t < this->cur_time_) return;
  Vector2d z = d_position.head(2);
  this->x_.head(2) = z;
}

void PoseEkf::correctSlamPose(Vector3d position, double t) {
  if (!this->initialized_flag_) {
    this->initialized_flag_ = true;
    this->cur_time_ = t;
    return;
  }

  if (t < this->cur_time_) return;

  double dt = t - this->cur_time_;
  Vector3d z = position;
  Vector3d zhat;
  MatrixXd H;
  measurementSlamPose(zhat, H);
  correct(z, zhat, H, this->r_vicon_pose_);
}

void PoseEkf::correctViconPose(Vector3d position, double t) {
  if (!this->initialized_flag_) {
    this->initialized_flag_ = true;
    this->cur_time_ = t;
    return;
  }

  if (t < this->cur_time_) return;

  double dt = t - this->cur_time_;
  Vector3d z = position;
  Vector3d zhat;
  MatrixXd h;
  measurementViconVel(zhat, h);
  correct(z, zhat, h, this->r_vicon_pose_);
}

void PoseEkf::correctViconVelocity(Vector3d velocity, double t) {
  if (!this->initialized_flag_) {
    this->initialized_flag_ = true;
    this->cur_time_ = t;
    return;
  }

  if (t < this->cur_time_) return;

  double dt = t - this->cur_time_;
  Vector3d z = velocity;
  Vector3d zhat;
  MatrixXd h;
  measurementViconVel(zhat, h);
  correct(z, zhat, h, this->r_vicon_pose_);
}

// Euler angle defination: zyx
// Rotation matrix: C_body2ned
Quaterniond euler2Quaternion(Vector3d euler) {
  double cr = cos(euler(0) / 2);
  double sr = sin(euler(0) / 2);
  double cp = cos(euler(1) / 2);
  double sp = sin(euler(1) / 2);
  double cy = cos(euler(2) / 2);
  double sy = sin(euler(2) / 2);
  Quaterniond q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;
  return q;
}

Matrix3d quaternion2Mat(Quaterniond q) {
  Matrix3d m;
  double a = q.w(), b = q.x(), c = q.y(), d = q.z();
  m << a * a + b * b - c * c - d * d, 2 * (b * c - a * d), 2 * (b * d + a * c),
       2 * (b * c + a * d), a * a - b * b + c * c - d * d, 2 * (c * d - a * b),
       2 * (b * d - a * c), 2 * (c * d + a * b), a * a - b * b - c * c + d * d;
  return m;
}

Vector3d mat2Euler(Matrix3d m) {
  double r = atan2(m(2, 1), m(2, 2));
  double p = asin(-m(2, 0));
  double y = atan2(m(1, 0), m(0, 0));
  Vector3d rpy(r, p, y);
  return rpy;
}

Quaterniond mat2Quaternion(Matrix3d m) {
  // return euler2quaternion(mat2euler(m));
  Quaterniond q;
  double a, b, c, d;
  a = sqrt(1 + m(0, 0) + m(1, 1) + m(2, 2)) / 2;
  b = (m(2, 1) - m(1, 2)) / (4 * a);
  c = (m(0, 2) - m(2, 0)) / (4 * a);
  d = (m(1, 0) - m(0, 1)) / (4 * a);
  q.w() = a;
  q.x() = b;
  q.y() = c;
  q.z() = d;
  return q;
}

Matrix3d euler2Mat(Vector3d euler) {
  double cr = cos(euler(0));
  double sr = sin(euler(0));
  double cp = cos(euler(1));
  double sp = sin(euler(1));
  double cy = cos(euler(2));
  double sy = sin(euler(2));
  Matrix3d m;
  m << cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy, cp * sy,
       cr * cy + sr * sp * sy, -sr * cy + cr * sp * sy, -sp, sr * cp, cr * cp;
  return m;
}

Vector3d quaternion2Euler(Quaterniond q) {
  return mat2Euler(quaternion2Mat(q));
}
