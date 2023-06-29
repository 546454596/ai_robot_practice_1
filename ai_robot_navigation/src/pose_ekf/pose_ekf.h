#ifndef AI_NAVIGATION_POSEEKF_POSEEKF_H_
#define AI_NAVIGATION_POSEEKF_POSEEKF_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

constexpr double kGA = 9.80665;

Eigen::Quaterniond euler2Quaternion(Eigen::Vector3d euler);
Eigen::Matrix3d quaternion2Mat(Eigen::Quaterniond q);
Eigen::Vector3d mat2Euler(Eigen::Matrix3d m);
Eigen::Quaterniond mat2Quaternion(Eigen::Matrix3d m);
Eigen::Matrix3d euler2Mat(Eigen::Vector3d euler);
Eigen::Vector3d quaternion2Euler(Eigen::Quaterniond q);

// state for kalman filter
// 0-2 Px Py Pz
// 3-5 Vx Vy Vz
// 6-8 bax bay baz
// inertial frame: NWU

class PoseEkf {
public:
  PoseEkf();
  ~PoseEkf();

  void correct(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d mag, double t);
  void correct(Eigen::VectorXd z, Eigen::VectorXd zhat, Eigen::MatrixXd h, Eigen::MatrixXd r);
  void correctFix(Eigen::Vector3d position, double t);
  void correctFixVelocity(Eigen::Vector3d velocity, double t);
  void correctGravity(Eigen::Vector3d acc, double t);
  void correctOptVelocity(Eigen::Vector2d opt_velocity, double t);
  void correctPoseCorrect(Eigen::Vector2d d_position, Eigen::Vector4d d_q, double t);
  void correctSlamPose(Eigen::Vector3d position, double t);
  void correctSonarHeight(double sonar_height, double t);  // todo, without considering the roll and pitch
  void correctViconPose(Eigen::Vector3d position, double t);
  void correctViconVelocity(Eigen::Vector3d velocity, double t);

  void getPoseCorrectCov(Eigen::Vector2d d_position);
  void getState(Eigen::Quaterniond& q, Eigen::Vector3d& position,
                Eigen::Vector3d& velocity, Eigen::Vector3d& ba);
  double getTime() { return this->cur_time_; }

  void measurementFix(Eigen::Vector2d& position, Eigen::MatrixXd& h);
  void measurementFixVelocity(Eigen::Vector3d& velocity, Eigen::MatrixXd& h);
  void measurementGravity(Eigen::Vector3d& acc, Eigen::MatrixXd& h);
  void measurementOptcalFlow(Eigen::Vector2d& velocity, Eigen::MatrixXd& h);
  void measurementPoseCorrect(Eigen::Vector2d& dpos, Eigen::MatrixXd& h);
  void measurementSlamPose(Eigen::Vector3d& position, Eigen::MatrixXd& h);
  void measurementSonarHeight(Eigen::VectorXd& sonar_height, Eigen::MatrixXd& h);
  void measurementViconPose(Eigen::Vector3d& position, Eigen::MatrixXd& h);
  void measurementViconVel(Eigen::Vector3d& velocity, Eigen::MatrixXd& h);
  // void measurement_altimeter(double& altimeter_height, Eigen::MatrixXd H);

  void predict(Eigen::Vector4d quan, Eigen::Vector3d acc, double t);
  void process(Eigen::Vector4d quan, Eigen::Vector3d acc, Eigen::VectorXd& xdot,
               Eigen::MatrixXd& f, Eigen::MatrixXd& g);

  Eigen::MatrixXd computeF(Eigen::Vector3d gyro, Eigen::Vector3d acc);
  Eigen::MatrixXd computeH(Eigen::Vector3d mag);
  Eigen::VectorXd measurement(Eigen::VectorXd x, Eigen::Vector3d mag);

public:
  double opt_q;
  bool px4pose_initialized;
  Eigen::Vector3d px4PosInit;

private:
  // covariance parameter
  const double kAccCov = 0.1;
  const double kFixCov = 2.0;
  const double kFixVelocityCov = 2.0;
  const double kGravityCov = 5.0;
  const double kOpticalCov = 0.01;
  const double kPosecorrectCov = 0.1;
  const double kSlamCov = 0.001;
  const double kSonarHeightCov = 0.2;
  const double kViconPoseCov = 1e-10;
  const double kViconVelCov = 1e-8;
  const int kNState = 9;

  bool altimeter_initialized_flag_;
  double cur_time_;
  bool fix_initialized_flag_;
  bool imu_initialized_flag_;
  bool initialized_flag_;
  bool optical_initialized_flag_;
  bool sonar_initialized_flag_;

  Eigen::MatrixXd p_;  // covariance
  Eigen::MatrixXd q_;  // imu observation noise
  Eigen::Vector3d acc_;
  Eigen::Vector4d quan_;
  Eigen::VectorXd x_;  // state

  const Eigen::MatrixXd r_fix_ = Eigen::Matrix2d::Identity() * kFixCov;
  const Eigen::MatrixXd r_fix_velocity_ = Eigen::Matrix3d::Identity() * kFixVelocityCov;
  const Eigen::MatrixXd r_sonar_height_ = Eigen::MatrixXd::Identity(1, 1) * kSonarHeightCov;
  const Eigen::MatrixXd r_gravity_ = Eigen::Matrix3d::Identity() * kGravityCov;
  const Eigen::MatrixXd r_optcal_velocity_ = Eigen::Matrix2d::Identity() * kOpticalCov;
  const Eigen::MatrixXd r_pose_correct_ = Eigen::Matrix2d::Identity() * kPosecorrectCov;
  const Eigen::MatrixXd r_slam_pose_ = Eigen::Matrix3d::Identity() * kSlamCov;
  const Eigen::MatrixXd r_vicon_pose_ = Eigen::Matrix3d::Identity() * kViconPoseCov;
  const Eigen::MatrixXd r_vicon_vel_ = Eigen::Matrix3d::Identity() * kViconVelCov;
  const Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, kGA);
};

#endif  // AI_NAVIGATION_POSEEKF_POSEEKF_H_
