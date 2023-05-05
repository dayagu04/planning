/***************************************************************
 * @file       observers.cpp
 * @brief      for observers design
 * @author     xiaoliang.wang
 * @version    v0.0.0
 * @date       Sep-17-2021
 **************************************************************/

#include "observers.h"
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

namespace pnc {
namespace observers {
DOBwithIdealModel::DOBwithIdealModel(double fc, double beta2_gain,
                                     double model_gain, double fs) {
  Reset();
  InitDob(fc, beta2_gain, model_gain, fs);
}

void DOBwithIdealModel::Reset(void) {
  state_hat_ = 0.0;
  dist_hat_ = 0.0;
  dist_prior_ = 0.0;
  U_.setZero();
  Y_.setZero();
  X_.setZero();
}

void DOBwithIdealModel::InitDob(double fc, double beta2_gain, double model_gain,
                                double fs) {
  fs_ = fs;
  fc_ = fc;
  beta2_gain_ = beta2_gain;
  model_gain_ = model_gain;

  Eigen::Matrix<double, 2, 2> A;
  Eigen::Matrix<double, 2, 3> B;
  Eigen::Matrix<double, 2, 2> C;
  Eigen::Matrix<double, 2, 3> D;

  double omega = 2 * M_PI * fc;
  double beta1 = 2.0 * omega;
  double beta2 = omega * omega * beta2_gain;

  A << -beta1, 1.0, -beta2, 0.0;
  B << beta1, model_gain, 1.0, beta2, 0.0, 0.0;
  C << 1.0, 0.0, 0.0, 1.0;
  D << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  double a = 2.0 * fs;
  Eigen::Matrix<double, 2, 2> tmp1 = (a * Eigen::Matrix2d::Identity()) - A;
  Eigen::Matrix<double, 2, 2> tmp2 = (a * Eigen::Matrix2d::Identity()) + A;

  Ad_ = tmp1.inverse() * tmp2;
  Bd_ = tmp1.inverse() * B;
  Cd_ = C * (Ad_ + Eigen::Matrix2d::Identity());
  Dd_ = C * Bd_ + D;
  Cd_pinv_ = Cd_.inverse();

  Reset();
}

void DOBwithIdealModel::SetDynamicModelGain(double model_gain) {
  model_gain_ = model_gain;

  Eigen::Matrix<double, 2, 2> A;
  Eigen::Matrix<double, 2, 3> B;
  Eigen::Matrix<double, 2, 2> C;
  Eigen::Matrix<double, 2, 3> D;

  double omega = 2 * M_PI * fc_;
  double beta1 = 2.0 * omega;
  double beta2 = omega * omega * beta2_gain_;

  A << -beta1, 1.0, -beta2, 0.0;
  B << beta1, model_gain, 1.0, beta2, 0.0, 0.0;
  C << 1.0, 0.0, 0.0, 1.0;
  D << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  double a = 2.0 * fs_;
  Eigen::Matrix<double, 2, 2> tmp1 = (a * Eigen::Matrix2d::Identity()) - A;
  Eigen::Matrix<double, 2, 2> tmp2 = (a * Eigen::Matrix2d::Identity()) + A;

  Ad_ = tmp1.inverse() * tmp2;
  Bd_ = tmp1.inverse() * B;
  Cd_ = C * (Ad_ + Eigen::Matrix2d::Identity());
  Dd_ = C * Bd_ + D;
  Cd_pinv_ = Cd_.inverse();
}

void DOBwithIdealModel::Tick(void) {
  Y_ = Cd_ * X_ + Dd_ * U_;
  X_ = Ad_ * X_ + Bd_ * U_;
  state_hat_ = Y_[0];
  dist_hat_ = Y_[1];
}

void DOBwithIdealModel::Update(double y, double u) {
  if (std::isnan(y) || std::isnan(u)) {
    Reset();
  } else {
    input_ = u;
    U_ << y, u, 0.0;
    Tick();
  }
}

void DOBwithIdealModel::Update(double y, double u, double dist_prior) {
  if (std::isnan(y) || std::isnan(u) || std::isnan(dist_prior)) {
    Reset();
  } else {
    input_ = u;
    U_ << y, u, dist_prior;
    Tick();
  }
}

void DOBwithIdealModel::SwitchBuf(double y, double u) {
  if (std::isnan(y) || std::isnan(u)) {
    Reset();
  } else {
    Y_ << y, 0.0;
    U_ << y, u, 0.0;

    X_ = Cd_pinv_ * (Y_ - Dd_ * U_);
    Tick();
  }
}

void DOBwithIdealModel::SwitchBuf(double y, double u, double dist_prior) {
  if (std::isnan(y) || std::isnan(u) || std::isnan(dist_prior)) {
    Reset();
  } else {
    Y_ << y, dist_prior;
    U_ << y, u, dist_prior;

    X_ = Cd_pinv_ * (Y_ - Dd_ * U_);
    Tick();
  }
}

void DOBwithIdealModel::SwitchBuf2(double y, double u, double dist) {
  if (std::isnan(y) || std::isnan(u) || std::isnan(dist)) {
    Reset();
  } else {
    Y_ << y, dist;
    U_ << y, u, 0.0;

    X_ = Cd_pinv_ * (Y_ - Dd_ * U_);
    Y_ = Cd_ * X_ + Dd_ * U_;

    state_hat_ = Y_[0];
    dist_hat_ = Y_[1];
  }
}

// kalman filter
void KalmanFilter::Reset() {
  x_ = x0_;
  y_.setZero();
  yr_.setZero();
  P_.setZero();
  S_.setZero();
  K_.setZero();
}

void KalmanFilter::init(const Eigen::MatrixXd &F, const Eigen::MatrixXd &H,
                        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                        const Eigen::VectorXd &x0) {
  F_ = F;
  H_ = H;

  x0_ = x0;
  x_ = x0;

  Q_ = Q;
  R_ = R;

  auto n_state = F.rows();
  auto n_measure = H_.rows();

  x_.resize(n_state);
  y_.resize(n_measure);
  yr_.resize(n_measure);
  P_.resize(n_state, n_state);
  S_.resize(n_measure, n_measure);
  K_.resize(n_state, n_measure);

  Reset();
}

void KalmanFilter::SetMatrixF(const Eigen::MatrixXd &F) { F_ = F; }
void KalmanFilter::SetMatrixH(const Eigen::MatrixXd &H) { H_ = H; }

void KalmanFilter::SetState(const Eigen::VectorXd &x) { x_ = x; }

void KalmanFilter::SetDefaultState(const Eigen::VectorXd &x0) {
  x0_ = x0;
  x_ = x0;
}

void KalmanFilter::SetNoiseMatrixQ(const Eigen::MatrixXd &Q) { Q_ = Q; }

void KalmanFilter::SetNoiseMatrixR(const Eigen::MatrixXd &R) { R_ = R; }

void KalmanFilter::SetNoiseMatrix(const Eigen::MatrixXd &Q,
                                  const Eigen::MatrixXd &R) {
  SetNoiseMatrixQ(Q);
  SetNoiseMatrixR(R);
}

void KalmanFilter::Update(Eigen::VectorXd &z) {
  // prior update by prediction

  Eigen::MatrixXd x_prior = F_ * x_;
  Eigen::MatrixXd P_prior = F_ * P_ * (F_.transpose()) + Q_;

  // measurement pre-fit residual
  Eigen::MatrixXd y_r = z - H_ * x_prior;

  // pre-fit residual covariance
  S_ = H_ * P_prior * H_.transpose() + R_;

  // optimal kalman gain
  K_ = P_prior * H_.transpose() * S_.inverse();

  // post state update
  x_ = x_prior + K_ * y_r;

  // post covariance update
  Eigen::MatrixXd Eye = F_;
  Eye.setIdentity();

  P_ = (Eye - K_ * H_) * P_prior;
  y_ = H_ * x_;
  yr_ = z - y_;
}

void KalmanFilter::GetState(Eigen::VectorXd &x) { x = x_; }
void KalmanFilter::GetOutput(Eigen::VectorXd &y) { y = y_; }

} // namespace observers

} // namespace pnc
