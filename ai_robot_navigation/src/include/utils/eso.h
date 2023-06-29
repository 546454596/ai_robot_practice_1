#ifndef AI_ROBOT_NAVIGATION_UTILS_ESO_H_
#define AI_ROBOT_NAVIGATION_UTILS_ESO_H_

#include <Eigen/Core>

class Eso {
public:
  Eso(float wo, float b0, float dt);
  virtual ~Eso(){};
  void setState(const Eigen::Vector3f& xhat);
  void setParams(float wo, float b, float dt);
  Eigen::Vector3f update(float u, float y);

private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void disCretize(const Eigen::Matrix3f& A_obs_ct,
                  const Eigen::Matrix<float, 3, 2>& B_obs_ct, float dt);

private:
  Eigen::Vector3f Xhat_;
  Eigen::Matrix3f A_obs_dt_;
  Eigen::Matrix<float, 3, 2> B_obs_dt_;
};

#endif