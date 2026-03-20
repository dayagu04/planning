/***************************************************************
 * @file       observers.h
 * @brief      for observers design
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-17-2021
 **************************************************************/

#ifndef __OBSERVERS_H__
#define __OBSERVERS_H__

#include <Eigen/Core>
#include "statespace_sys.h"

namespace pnc {
namespace observers {
class DOBwithIdealModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DOBwithIdealModel(){};
  DOBwithIdealModel(double fc, double beta2_gain, double model_gain, double fs);
  void Reset();
  void InitDob(double fc, double beta2_gain, double model_gain, double fs);
  void SetDynamicModelGain(double model_gain);
  void Update(double y, double u);
  void Update(double y, double u, double dist_prior);
  void SwitchBuf(double y, double u, double dist_prior);
  void SwitchBuf(double y, double u);
  void SwitchBuf2(double y, double u, double dist);

  double GetState() { return state_hat_; }
  double GetDisturbance() { return dist_hat_; }
  double GetDisturbanceInput() { return dist_hat_ / model_gain_; }
  double GetInput() { return input_; }
  double GetModelGain() { return model_gain_; }
  double GetFc() { return fc_; }

 private:
  void Tick(void);
  double fc_;
  double beta2_gain_;
  double model_gain_;
  double fs_;
  double input_;
  double state_hat_;
  double dist_hat_;
  double dist_prior_;
  Eigen::Matrix<double, 3, 1> U_;
  Eigen::Matrix<double, 2, 1> Y_;
  Eigen::Matrix<double, 2, 1> X_;

  Eigen::Matrix2d Ad_;
  Eigen::Matrix<double, 2, 3> Bd_;
  Eigen::Matrix2d Cd_;
  Eigen::Matrix<double, 2, 3> Dd_;
  Eigen::Matrix2d Cd_pinv_;
};

class KalmanFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void init(const Eigen::MatrixXd &F, const Eigen::MatrixXd &H,
            const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
            const Eigen::VectorXd &x0);
  void Reset();

  void SetMatrixF(const Eigen::MatrixXd &F);
  void SetMatrixH(const Eigen::MatrixXd &H);

  void SetState(const Eigen::VectorXd &x);
  void SetDefaultState(const Eigen::VectorXd &x0);

  void SetNoiseMatrix(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);
  void SetNoiseMatrixQ(const Eigen::MatrixXd &Q);
  void SetNoiseMatrixR(const Eigen::MatrixXd &R);

  void Update(Eigen::VectorXd &z);
  void GetState(Eigen::VectorXd &x);
  void GetOutput(Eigen::VectorXd &y);

#ifndef DEBUG
 private:
#endif

  // system matrix in discrete form
  Eigen::MatrixXd F_;
  Eigen::MatrixXd H_;

  // process and measurement noise matrix
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  // init state for reset
  Eigen::VectorXd x0_;

  // system state
  Eigen::VectorXd x_;
  Eigen::VectorXd y_;
  Eigen::VectorXd yr_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd S_;
  Eigen::MatrixXd K_;
};

}  // namespace observers
}  // namespace pnc

#endif
