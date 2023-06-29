#include "utils/eso.h"

#include <unsupported/Eigen/MatrixFunctions>

using Eigen::Matrix;
using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::Vector2f;
using Eigen::Vector3f;

Eso::Eso(float wo, float b0, float dt) { 
  setParams(wo, b0, dt); 
}

void Eso::setParams(float wo, float b0, float dt) {
  Matrix3f A;
  Vector3f B;
  Vector3f L;
  Matrix<float, 1, 3> C;
  Matrix<float, 3, 2> B_obs_ct;
  Matrix3f A_obs_ct;

  A << 0, 1, 0, 0, 0, 1, 0, 0, 0;
  B << 0, b0, 0;
  C << 1, 0, 0;
  L << 3 * wo, 3 * wo * wo, wo * wo * wo;

  A_obs_ct = A - L * C;
  B_obs_ct << B, L;

  disCretize(A_obs_ct, B_obs_ct, dt);
}

void Eso::disCretize(const Matrix3f& A_obs_ct,
                     const Matrix<float, 3, 2>& B_obs_ct, float dt) {
  Matrix<float, 5, 5> discretization, discretization_exp;

  discretization << A_obs_ct, B_obs_ct, MatrixXf::Zero(2, 5);
  discretization_exp = (discretization * dt).exp();

  this->A_obs_dt_ = discretization_exp.block<3, 3>(0, 0);
  this->B_obs_dt_ = discretization_exp.block<3, 2>(0, 3);
}

void Eso::setState(const Eigen::Vector3f& xhat) { 
  this->Xhat_ = xhat; 
}

Vector3f Eso::update(float u, float y) {
  Vector2f u_obs;
  u_obs << u, y;
  this->Xhat_ = this->A_obs_dt_ * this->Xhat_ + this->B_obs_dt_ * u_obs;
  return this->Xhat_;
}
