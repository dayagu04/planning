/***************************************************************
 * @file       statespace_sys.h
 * @brief      for statespace design
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-23-2021
 **************************************************************/

#ifndef __STATESPACE_SYS_H__
#define __STATESPACE_SYS_H__

#include "Eigen/Core"

#define SISO_SYS_ORDER_ZERO (0)
#define SISO_SYS_ORDER_1ST (1)
#define SISO_SYS_ORDER_2ND (2)
#define SISO_SYS_ORDER_3RD (3)
#define SISO_SYS_ORDER_4TH (4)
#define SISO_SYS_ORDER_5TH (5)
#define SISO_SYS_ORDER_6TH (6)
#define SISO_SYS_ORDER_7TH (7)
#define SISO_SYS_ORDER_8TH (8)

namespace pnc {
namespace statespace_sys {
class StatespaceSISOSys1st {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void Reset();
  void Update(double u);
  virtual void SwitchBuf(double u, double y);
  double GetOutput();

  void InitSSdiscrete(double a, double b, double c, double d);
  void InitSScontinuous(double a, double b, double c, double d, double fs);
  void InitTFcontinuous(const double *coef_p, const double *coef_z,
                        const double fs);

#ifndef DEBUG
 protected:
#endif
  /* contitous system state matrices and transfer function */
  double Ac_;
  double Bc_;
  double Cc_;
  double Dc_;
  double num_[SISO_SYS_ORDER_1ST + 1];
  double den_[SISO_SYS_ORDER_1ST + 1];

  /* discrete state matrices */
  double Ad_;
  double Bd_;
  double Cd_;
  double Dd_;
  double Cd_pinv_;
  double fs_;

  /* discrete states */
  bool inited_flag_ = false;
  double x_;
  double y_;
  double u_;
};

class StatespaceSISOSys2nd {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void Reset();
  void Update(double u);
  virtual void SwitchBuf(double u, double y);
  double GetOutput();

  void InitSSdiscrete(Eigen::Matrix2d a,
                      Eigen::Matrix<double, SISO_SYS_ORDER_2ND, 1> b,
                      Eigen::Matrix<double, 1, SISO_SYS_ORDER_2ND> c,
                      Eigen::Matrix<double, 1, 1> d);
  void InitSScontinuous(Eigen::Matrix2d a,
                        Eigen::Matrix<double, SISO_SYS_ORDER_2ND, 1> b,
                        Eigen::Matrix<double, 1, SISO_SYS_ORDER_2ND> c,
                        Eigen::Matrix<double, 1, 1> d, double fs);
  void InitTFcontinuous(const double *coef_p, const double *coef_z,
                        const double fs);

#ifndef DEBUG
 protected:
#endif
  /* contitous system state matrices and transfer function */
  Eigen::Matrix2d Ac_;
  Eigen::Matrix<double, SISO_SYS_ORDER_2ND, 1> Bc_;
  Eigen::Matrix<double, 1, SISO_SYS_ORDER_2ND> Cc_;
  Eigen::Matrix<double, 1, 1> Dc_;
  double num_[SISO_SYS_ORDER_2ND + 1];
  double den_[SISO_SYS_ORDER_2ND + 1];

  /* discrete state matrices */
  Eigen::Matrix2d Ad_;
  Eigen::Matrix<double, SISO_SYS_ORDER_2ND, 1> Bd_;
  Eigen::Matrix<double, 1, SISO_SYS_ORDER_2ND> Cd_;
  Eigen::Matrix<double, 1, 1> Dd_;
  Eigen::Matrix<double, SISO_SYS_ORDER_2ND, 1> Cd_pinv_;
  double fs_;

  /* discrete states */
  bool inited_flag_ = false;
  Eigen::Matrix<double, SISO_SYS_ORDER_2ND, 1> x_;
  Eigen::Matrix<double, 1, 1> y_;
  Eigen::Matrix<double, 1, 1> u_;
};

class StatespaceSISOSys3rd {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void Reset();
  void Update(double u);
  virtual void SwitchBuf(double u, double y);
  double GetOutput();

  void InitSSdiscrete(Eigen::Matrix3d a,
                      Eigen::Matrix<double, SISO_SYS_ORDER_3RD, 1> b,
                      Eigen::Matrix<double, 1, SISO_SYS_ORDER_3RD> c,
                      Eigen::Matrix<double, 1, 1> d);
  void InitSScontinuous(Eigen::Matrix3d a,
                        Eigen::Matrix<double, SISO_SYS_ORDER_3RD, 1> b,
                        Eigen::Matrix<double, 1, SISO_SYS_ORDER_3RD> c,
                        Eigen::Matrix<double, 1, 1> d, double fs);
  void InitTFcontinuous(const double *coef_p, const double *coef_z,
                        const double fs);

#ifndef DEBUG
 protected:
#endif
  /* contitous system state matrices and transfer function */
  Eigen::Matrix3d Ac_;
  Eigen::Matrix<double, SISO_SYS_ORDER_3RD, 1> Bc_;
  Eigen::Matrix<double, 1, SISO_SYS_ORDER_3RD> Cc_;
  Eigen::Matrix<double, 1, 1> Dc_;
  double num_[SISO_SYS_ORDER_3RD + 1];
  double den_[SISO_SYS_ORDER_3RD + 1];

  /* discrete state matrices */
  Eigen::Matrix3d Ad_;
  Eigen::Matrix<double, SISO_SYS_ORDER_3RD, 1> Bd_;
  Eigen::Matrix<double, 1, SISO_SYS_ORDER_3RD> Cd_;
  Eigen::Matrix<double, 1, 1> Dd_;
  Eigen::Matrix<double, SISO_SYS_ORDER_3RD, 1> Cd_pinv_;
  double fs_;

  /* discrete states */
  bool inited_flag_ = false;
  Eigen::Matrix<double, SISO_SYS_ORDER_3RD, 1> x_;
  Eigen::Matrix<double, 1, 1> y_;
  Eigen::Matrix<double, 1, 1> u_;
};

class StatespaceSISOSys4th {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void Reset();
  void Update(double u);
  virtual void SwitchBuf(double u, double y);
  double GetOutput();

  void InitSSdiscrete(Eigen::Matrix4d a,
                      Eigen::Matrix<double, SISO_SYS_ORDER_4TH, 1> b,
                      Eigen::Matrix<double, 1, SISO_SYS_ORDER_4TH> c,
                      Eigen::Matrix<double, 1, 1> d);
  void InitSScontinuous(Eigen::Matrix4d a,
                        Eigen::Matrix<double, SISO_SYS_ORDER_4TH, 1> b,
                        Eigen::Matrix<double, 1, SISO_SYS_ORDER_4TH> c,
                        Eigen::Matrix<double, 1, 1> d, double fs);
  void InitTFcontinuous(const double *coef_p, const double *coef_z,
                        const double fs);

#ifndef DEBUG
 protected:
#endif
  /* contitous system state matrices and transfer function */
  Eigen::Matrix4d Ac_;
  Eigen::Matrix<double, SISO_SYS_ORDER_4TH, 1> Bc_;
  Eigen::Matrix<double, 1, SISO_SYS_ORDER_4TH> Cc_;
  Eigen::Matrix<double, 1, 1> Dc_;
  double num_[SISO_SYS_ORDER_4TH + 1];
  double den_[SISO_SYS_ORDER_4TH + 1];

  /* discrete state matrices */
  Eigen::Matrix4d Ad_;
  Eigen::Matrix<double, SISO_SYS_ORDER_4TH, 1> Bd_;
  Eigen::Matrix<double, 1, SISO_SYS_ORDER_4TH> Cd_;
  Eigen::Matrix<double, 1, 1> Dd_;
  Eigen::Matrix<double, SISO_SYS_ORDER_4TH, 1> Cd_pinv_;
  double fs_;

  /* discrete states */
  bool inited_flag_ = false;
  Eigen::Matrix<double, SISO_SYS_ORDER_4TH, 1> x_;
  Eigen::Matrix<double, 1, 1> y_;
  Eigen::Matrix<double, 1, 1> u_;
};

class StatespaceSISOSys5th {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void Reset();
  void Update(double u);
  virtual void SwitchBuf(double u, double y);
  double GetOutput();

  void InitSSdiscrete(
      Eigen::Matrix<double, SISO_SYS_ORDER_5TH, SISO_SYS_ORDER_5TH> a,
      Eigen::Matrix<double, SISO_SYS_ORDER_5TH, 1> b,
      Eigen::Matrix<double, 1, SISO_SYS_ORDER_5TH> c,
      Eigen::Matrix<double, 1, 1> d);
  void InitSScontinuous(
      Eigen::Matrix<double, SISO_SYS_ORDER_5TH, SISO_SYS_ORDER_5TH> a,
      Eigen::Matrix<double, SISO_SYS_ORDER_5TH, 1> b,
      Eigen::Matrix<double, 1, SISO_SYS_ORDER_5TH> c,
      Eigen::Matrix<double, 1, 1> d, double fs);
  void InitTFcontinuous(const double *coef_p, const double *coef_z,
                        const double fs);

#ifndef DEBUG
 protected:
#endif
  /* contitous system state matrices and transfer function */
  Eigen::Matrix<double, SISO_SYS_ORDER_5TH, SISO_SYS_ORDER_5TH> Ac_;
  Eigen::Matrix<double, SISO_SYS_ORDER_5TH, 1> Bc_;
  Eigen::Matrix<double, 1, SISO_SYS_ORDER_5TH> Cc_;
  Eigen::Matrix<double, 1, 1> Dc_;
  double num_[SISO_SYS_ORDER_5TH + 1];
  double den_[SISO_SYS_ORDER_5TH + 1];

  /* discrete state matrices */
  Eigen::Matrix<double, SISO_SYS_ORDER_5TH, SISO_SYS_ORDER_5TH> Ad_;
  Eigen::Matrix<double, SISO_SYS_ORDER_5TH, 1> Bd_;
  Eigen::Matrix<double, 1, SISO_SYS_ORDER_5TH> Cd_;
  Eigen::Matrix<double, 1, 1> Dd_;
  Eigen::Matrix<double, SISO_SYS_ORDER_5TH, 1> Cd_pinv_;
  double fs_;

  /* discrete states */
  bool inited_flag_ = false;
  Eigen::Matrix<double, SISO_SYS_ORDER_5TH, 1> x_;
  Eigen::Matrix<double, 1, 1> y_;
  Eigen::Matrix<double, 1, 1> u_;
};

class StatespaceSISOSys6th {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void Reset();
  void Update(double u);
  virtual void SwitchBuf(double u, double y);
  double GetOutput();

  void InitSSdiscrete(
      Eigen::Matrix<double, SISO_SYS_ORDER_6TH, SISO_SYS_ORDER_6TH> a,
      Eigen::Matrix<double, SISO_SYS_ORDER_6TH, 1> b,
      Eigen::Matrix<double, 1, SISO_SYS_ORDER_6TH> c,
      Eigen::Matrix<double, 1, 1> d);
  void InitSScontinuous(
      Eigen::Matrix<double, SISO_SYS_ORDER_6TH, SISO_SYS_ORDER_6TH> a,
      Eigen::Matrix<double, SISO_SYS_ORDER_6TH, 1> b,
      Eigen::Matrix<double, 1, SISO_SYS_ORDER_6TH> c,
      Eigen::Matrix<double, 1, 1> d, double fs);
  void InitTFcontinuous(const double *coef_p, const double *coef_z,
                        const double fs);

#ifndef DEBUG
 protected:
#endif
  /* contitous system state matrices and transfer function */
  Eigen::Matrix<double, SISO_SYS_ORDER_6TH, SISO_SYS_ORDER_6TH> Ac_;
  Eigen::Matrix<double, SISO_SYS_ORDER_6TH, 1> Bc_;
  Eigen::Matrix<double, 1, SISO_SYS_ORDER_6TH> Cc_;
  Eigen::Matrix<double, 1, 1> Dc_;
  double num_[SISO_SYS_ORDER_6TH + 1];
  double den_[SISO_SYS_ORDER_6TH + 1];

  /* discrete state matrices */
  Eigen::Matrix<double, SISO_SYS_ORDER_6TH, SISO_SYS_ORDER_6TH> Ad_;
  Eigen::Matrix<double, SISO_SYS_ORDER_6TH, 1> Bd_;
  Eigen::Matrix<double, 1, SISO_SYS_ORDER_6TH> Cd_;
  Eigen::Matrix<double, 1, 1> Dd_;
  Eigen::Matrix<double, SISO_SYS_ORDER_6TH, 1> Cd_pinv_;
  double fs_;

  /* discrete states */
  bool inited_flag_ = false;
  Eigen::Matrix<double, SISO_SYS_ORDER_6TH, 1> x_;
  Eigen::Matrix<double, 1, 1> y_;
  Eigen::Matrix<double, 1, 1> u_;
};

class StatespaceMIMO {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StatespaceMIMO(const Eigen::MatrixXd &Ac, const Eigen::MatrixXd &Bc,
                 const Eigen::MatrixXd &Cc, const Eigen::MatrixXd &Dc,
                 const double fs) {
    Init(Ac, Bc, Cc, Dc, fs);
    Reset();
  };

  void Init(const Eigen::MatrixXd &Ac, const Eigen::MatrixXd &Bc,
            const Eigen::MatrixXd &Cc, const Eigen::MatrixXd &Dc,
            const double fs);
  void Reset();
  void SetInitState(const Eigen::MatrixXd &X0);
  void Update(const Eigen::MatrixXd &U);
  void GetState(Eigen::MatrixXd &state);
  void GetOutput(Eigen::MatrixXd &out);
  virtual void SwitchBuf(Eigen::MatrixXd &U, Eigen::MatrixXd &Y);

 private:
  bool init_flag_;
  Eigen::MatrixXd U_;
  Eigen::MatrixXd Y_;
  Eigen::MatrixXd X_;

  Eigen::MatrixXd Ad_;
  Eigen::MatrixXd Bd_;
  Eigen::MatrixXd Cd_;
  Eigen::MatrixXd Dd_;
  Eigen::MatrixXd Cd_pinv_;
};

}  // namespace statespace_sys
}  // namespace pnc

#endif
