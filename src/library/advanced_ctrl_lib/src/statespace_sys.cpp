/***************************************************************
 * @file       statespace_sys.cpp
 * @brief      for statespace_sys design
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-17-2021
 **************************************************************/

#include "statespace_sys.h"
#include <Eigen/LU>
#include <Eigen/QR>
#include <cmath>
#include <iostream>

namespace pnc {
namespace statespace_sys {
/* -------------------- for first-order system -------------------- */
void StatespaceSISOSys1st::Reset() {
  u_ = 0.0f;
  y_ = 0.0f;
  x_ = 0.0f;
}

void StatespaceSISOSys1st::InitSSdiscrete(double a, double b, double c,
                                          double d) {
  Reset();
  inited_flag_ = true;
  Ad_ = a;
  Bd_ = b;
  Cd_ = c;
  Dd_ = d;
  Cd_pinv_ = 1 / c;
}

void StatespaceSISOSys1st::InitSScontinuous(double a, double b, double c,
                                            double d, double fs) {
  Ac_ = a;
  Bc_ = b;
  Cc_ = c;
  Dc_ = d;
  fs_ = fs;

  double alpha = 2.0f * fs_;

  Ad_ = (alpha + Ac_) / (alpha - Ac_);
  Bd_ = Bc_ / (alpha - Ac_);
  Cd_ = Cc_ * (Ad_ + 1.0f);
  Dd_ = Cc_ * Bd_ + Dc_;

  InitSSdiscrete(Ad_, Bd_, Cd_, Dd_);
}

void StatespaceSISOSys1st::InitTFcontinuous(const double *coef_p,
                                            const double *coef_z,
                                            const double fs) {
  for (int i = 0; i < 2; i++) {
    num_[i] = coef_z[i];
    den_[i] = coef_p[i];
  }

  double a0 = coef_p[1] / coef_p[0];
  double b0 = coef_z[1] / coef_p[0];
  double b1 = coef_z[0] / coef_p[0];
  double h0 = b1;
  double h1 = b0 - a0 * h0;

  Ac_ = -a0;
  Bc_ = h1;
  Cc_ = 1.0f;
  Dc_ = h0;

  InitSScontinuous(Ac_, Bc_, Cc_, Dc_, fs);
}

void StatespaceSISOSys1st::Update(double u) {
  if (!std::isnan(u)) {
    u_ = u;
    y_ = Cd_ * x_ + Dd_ * u_;
    x_ = Ad_ * x_ + Bd_ * u_;
  } else {
    Reset();
  }
}

void StatespaceSISOSys1st::SwitchBuf(double u, double y) {
  if (std::isnan(u) || std::isnan(y)) {
    Reset();
  } else {
    u_ = u;
    y_ = y;
    x_ = Cd_pinv_ * (y_ - Dd_ * u_);
    x_ = Ad_ * x_ + Bd_ * u_;
  }
}

double StatespaceSISOSys1st::GetOutput() { return y_; }

/* -------------------- for second-order system -------------------- */
void StatespaceSISOSys2nd::Reset() {
  u_.setZero();
  y_.setZero();
  x_.setZero();
}

void StatespaceSISOSys2nd::InitSSdiscrete(
    Eigen::Matrix2d a, Eigen::Matrix<double, SISO_SYS_ORDER_2ND, 1> b,
    Eigen::Matrix<double, 1, SISO_SYS_ORDER_2ND> c,
    Eigen::Matrix<double, 1, 1> d) {
  Reset();
  inited_flag_ = true;
  Ad_ = a;
  Bd_ = b;
  Cd_ = c;
  Dd_ = d;
  double tmp = Cd_(0) * Cd_(0) + Cd_(1) * Cd_(1);
  for (int i = 0; i < 2; i++) {
    Cd_pinv_(i) = c(i) / tmp;
  }
}

void StatespaceSISOSys2nd::InitSScontinuous(
    Eigen::Matrix2d a, Eigen::Matrix<double, SISO_SYS_ORDER_2ND, 1> b,
    Eigen::Matrix<double, 1, SISO_SYS_ORDER_2ND> c,
    Eigen::Matrix<double, 1, 1> d, double fs) {
  Ac_ = a;
  Bc_ = b;
  Cc_ = c;
  Dc_ = d;
  fs_ = fs;

  Eigen::Matrix2d Eye_alpha;
  Eigen::Matrix2d Eye;
  Eye.setIdentity();
  Eye_alpha.setIdentity();

  double alpha = 2.0f * fs_;

  for (int i = 0; i < 2; i++) {
    Eye_alpha(i, i) = alpha;
  }

  Eigen::Matrix2d tmp;
  tmp = Eye_alpha - Ac_;

  Eigen::Matrix2d tmp_inv = tmp.inverse();
  Ad_ = tmp_inv * (Eye_alpha + Ac_);
  Bd_ = tmp_inv * Bc_;
  Cd_ = Cc_ * (Ad_ + Eye);
  Dd_ = Cc_ * Bd_ + Dc_;

  InitSSdiscrete(Ad_, Bd_, Cd_, Dd_);
}

void StatespaceSISOSys2nd::InitTFcontinuous(const double *coef_p,
                                            const double *coef_z,
                                            const double fs) {
  for (int i = 0; i < 3; i++) {
    num_[i] = coef_z[i];
    den_[i] = coef_p[i];
  }

  double a[2] = {};
  int i = 0;
  for (i = 0; i < 2; i++) {
    a[i] = coef_p[2 - i] / coef_p[0];
  }

  double b[3] = {};
  for (i = 0; i < 3; i++) {
    b[i] = coef_z[2 - i] / coef_p[0];
  }

  double h[3] = {};
  h[0] = b[2];
  h[1] = b[1] - a[1] * h[0];
  h[2] = b[0] - a[1] * h[1] - a[0] * h[0];

  Ac_ << 0.0f, 1.0f, -a[0], -a[1];

  Bc_ << h[1], h[2];

  Cc_.setZero();
  Cc_[0] = 1.0f;

  Dc_[0] = h[0];

  InitSScontinuous(Ac_, Bc_, Cc_, Dc_, fs);
}

void StatespaceSISOSys2nd::Update(double u) {
  if (!std::isnan(u)) {
    u_(0, 0) = u;
    y_ = Cd_ * x_ + Dd_ * u_;
    x_ = Ad_ * x_ + Bd_ * u_;
  } else {
    Reset();
  }
}

void StatespaceSISOSys2nd::SwitchBuf(double u, double y) {
  if (std::isnan(u) || std::isnan(y)) {
    Reset();
  } else {
    u_(0, 0) = u;
    y_(0, 0) = y;
    x_ = Cd_pinv_ * (y_ - Dd_ * u_);
    x_ = Ad_ * x_ + Bd_ * u_;
  }
}

double StatespaceSISOSys2nd::GetOutput() { return y_(0, 0); }

/* -------------------- for third-order system -------------------- */
void StatespaceSISOSys3rd::Reset() {
  u_.setZero();
  y_.setZero();
  x_.setZero();
}

void StatespaceSISOSys3rd::InitSSdiscrete(Eigen::Matrix3d a,
                                          Eigen::Matrix<double, 3, 1> b,
                                          Eigen::Matrix<double, 1, 3> c,
                                          Eigen::Matrix<double, 1, 1> d) {
  Reset();
  inited_flag_ = true;
  Ad_ = a;
  Bd_ = b;
  Cd_ = c;
  Dd_ = d;
  double tmp =
      Cd_(0, 0) * Cd_(0, 0) + Cd_(0, 1) * Cd_(0, 1) + Cd_(0, 2) * Cd_(0, 2);
  for (int i = 0; i < 3; i++) {
    Cd_pinv_(i, 0) = c(0, i) / tmp;
  }
}

void StatespaceSISOSys3rd::InitSScontinuous(Eigen::Matrix3d a,
                                            Eigen::Matrix<double, 3, 1> b,
                                            Eigen::Matrix<double, 1, 3> c,
                                            Eigen::Matrix<double, 1, 1> d,
                                            double fs) {
  Ac_ = a;
  Bc_ = b;
  Cc_ = c;
  Dc_ = d;
  fs_ = fs;

  Eigen::Matrix3d Eye_alpha;
  Eigen::Matrix3d Eye;
  Eye.setIdentity();
  Eye_alpha.setIdentity();

  double alpha = 2.0f * fs_;

  for (int i = 0; i < 3; i++) {
    Eye_alpha(i, i) = alpha;
  }

  Eigen::Matrix3d tmp;
  tmp = Eye_alpha - Ac_;

  Eigen::Matrix3d tmp_inv = tmp.inverse();
  Ad_ = tmp_inv * (Eye_alpha + Ac_);
  Bd_ = tmp_inv * Bc_;
  Cd_ = Cc_ * (Ad_ + Eye);
  Dd_ = Cc_ * Bd_ + Dc_;

  InitSSdiscrete(Ad_, Bd_, Cd_, Dd_);
}

void StatespaceSISOSys3rd::InitTFcontinuous(const double *coef_p,
                                            const double *coef_z,
                                            const double fs) {
  for (int i = 0; i < 4; i++) {
    num_[i] = coef_z[i];
    den_[i] = coef_p[i];
  }

  double a[3] = {};
  int i = 0;
  for (i = 0; i < 3; i++) {
    a[i] = coef_p[3 - i] / coef_p[0];
  }

  double b[4] = {};
  for (i = 0; i < 4; i++) {
    b[i] = coef_z[3 - i] / coef_p[0];
  }

  double h[4] = {};
  h[0] = b[3];
  h[1] = b[2] - a[2] * h[0];
  h[2] = b[1] - a[2] * h[1] - a[1] * h[0];
  h[3] = b[0] - a[2] * h[2] - a[1] * h[1] - a[0] * h[0];

  Ac_ << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, -a[0], -a[1], -a[2];

  Bc_ << h[1], h[2], h[3];

  Cc_.setZero();
  Cc_(0, 0) = 1.0f;

  Dc_(0, 0) = h[0];

  InitSScontinuous(Ac_, Bc_, Cc_, Dc_, fs);
}

void StatespaceSISOSys3rd::Update(double u) {
  if (!std::isnan(u)) {
    u_(0, 0) = u;
    y_ = Cd_ * x_ + Dd_ * u_;
    x_ = Ad_ * x_ + Bd_ * u_;
  } else {
    Reset();
  }
}

void StatespaceSISOSys3rd::SwitchBuf(double u, double y) {
  if (std::isnan(u) || std::isnan(y)) {
    Reset();
  } else {
    u_(0, 0) = u;
    y_(0, 0) = y;
    x_ = Cd_pinv_ * (y_ - Dd_ * u_);
    x_ = Ad_ * x_ + Bd_ * u_;
  }
}

double StatespaceSISOSys3rd::GetOutput() { return y_(0, 0); }

/* -------------------- for fourth-order system -------------------- */
void StatespaceSISOSys4th::Reset() {
  u_.setZero();
  y_.setZero();
  x_.setZero();
}

void StatespaceSISOSys4th::InitSSdiscrete(Eigen::Matrix4d a,
                                          Eigen::Matrix<double, 4, 1> b,
                                          Eigen::Matrix<double, 1, 4> c,
                                          Eigen::Matrix<double, 1, 1> d) {
  Reset();
  inited_flag_ = true;
  Ad_ = a;
  Bd_ = b;
  Cd_ = c;
  Dd_ = d;
  double tmp = Cd_(0, 0) * Cd_(0, 0) + Cd_(0, 1) * Cd_(0, 1) +
               Cd_(0, 2) * Cd_(0, 2) + Cd_(0, 3) * Cd_(0, 3);
  for (int i = 0; i < 4; i++) {
    Cd_pinv_(i, 0) = c(0, i) / tmp;
  }
}

void StatespaceSISOSys4th::InitSScontinuous(Eigen::Matrix4d a,
                                            Eigen::Matrix<double, 4, 1> b,
                                            Eigen::Matrix<double, 1, 4> c,
                                            Eigen::Matrix<double, 1, 1> d,
                                            double fs) {
  Ac_ = a;
  Bc_ = b;
  Cc_ = c;
  Dc_ = d;
  fs_ = fs;

  Eigen::Matrix4d Eye_alpha;
  Eigen::Matrix4d Eye;
  Eye.setIdentity();
  Eye_alpha.setIdentity();

  double alpha = 2.0f * fs_;

  for (int i = 0; i < 4; i++) {
    Eye_alpha(i, i) = alpha;
  }

  Eigen::Matrix4d tmp;
  tmp = Eye_alpha - Ac_;

  Eigen::Matrix4d tmp_inv = tmp.inverse();
  Ad_ = tmp_inv * (Eye_alpha + Ac_);
  Bd_ = tmp_inv * Bc_;
  Cd_ = Cc_ * (Ad_ + Eye);
  Dd_ = Cc_ * Bd_ + Dc_;

  InitSSdiscrete(Ad_, Bd_, Cd_, Dd_);
}

void StatespaceSISOSys4th::InitTFcontinuous(const double *coef_p,
                                            const double *coef_z,
                                            const double fs) {
  for (int i = 0; i < 5; i++) {
    num_[i] = coef_z[i];
    den_[i] = coef_p[i];
  }

  double a[4] = {};
  int i = 0;
  for (i = 0; i < 4; i++) {
    a[i] = coef_p[4 - i] / coef_p[0];
  }

  double b[5] = {};
  for (i = 0; i < 5; i++) {
    b[i] = coef_z[4 - i] / coef_p[0];
  }

  double h[5] = {};
  h[0] = b[4];
  h[1] = b[3] - a[3] * h[0];
  h[2] = b[2] - a[3] * h[1] - a[2] * h[0];
  h[3] = b[1] - a[3] * h[2] - a[2] * h[1] - a[1] * h[0];
  h[4] = b[0] - a[3] * h[3] - a[2] * h[2] - a[1] * h[1] - a[0] * h[0];

  Ac_ << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
      -a[0], -a[1], -a[2], -a[3];

  Bc_ << h[1], h[2], h[3], h[4];

  Cc_.setZero();
  Cc_(0, 0) = 1.0f;

  Dc_(0, 0) = h[0];

  InitSScontinuous(Ac_, Bc_, Cc_, Dc_, fs);
}

void StatespaceSISOSys4th::Update(double u) {
  if (!std::isnan(u)) {
    u_(0, 0) = u;
    y_ = Cd_ * x_ + Dd_ * u_;
    x_ = Ad_ * x_ + Bd_ * u_;
  } else {
    Reset();
  }
}

void StatespaceSISOSys4th::SwitchBuf(double u, double y) {
  if (std::isnan(u) || std::isnan(y)) {
    Reset();
  } else {
    u_(0, 0) = u;
    y_(0, 0) = y;
    x_ = Cd_pinv_ * (y_ - Dd_ * u_);
    x_ = Ad_ * x_ + Bd_ * u_;
  }
}

double StatespaceSISOSys4th::GetOutput() { return y_(0, 0); }

/* -------------------- for fifth-order system -------------------- */
void StatespaceSISOSys5th::Reset() {
  u_.setZero();
  y_.setZero();
  x_.setZero();
}

void StatespaceSISOSys5th::InitSSdiscrete(
    Eigen::Matrix<double, SISO_SYS_ORDER_5TH, SISO_SYS_ORDER_5TH> a,
    Eigen::Matrix<double, SISO_SYS_ORDER_5TH, 1> b,
    Eigen::Matrix<double, 1, SISO_SYS_ORDER_5TH> c,
    Eigen::Matrix<double, 1, 1> d) {
  Reset();
  inited_flag_ = true;
  Ad_ = a;
  Bd_ = b;
  Cd_ = c;
  Dd_ = d;
  double tmp = Cd_(0, 0) * Cd_(0, 0) + Cd_(0, 1) * Cd_(0, 1) +
               Cd_(0, 2) * Cd_(0, 2) + Cd_(0, 3) * Cd_(0, 3) +
               Cd_(0, 4) * Cd_(0, 4);
  for (int i = 0; i < 5; i++) {
    Cd_pinv_(i, 0) = c(0, i) / tmp;
  }
}

void StatespaceSISOSys5th::InitSScontinuous(
    Eigen::Matrix<double, SISO_SYS_ORDER_5TH, SISO_SYS_ORDER_5TH> a,
    Eigen::Matrix<double, 5, 1> b, Eigen::Matrix<double, 1, 5> c,
    Eigen::Matrix<double, 1, 1> d, double fs) {
  Ac_ = a;
  Bc_ = b;
  Cc_ = c;
  Dc_ = d;
  fs_ = fs;

  Eigen::Matrix<double, SISO_SYS_ORDER_5TH, SISO_SYS_ORDER_5TH> Eye_alpha;
  Eigen::Matrix<double, SISO_SYS_ORDER_5TH, SISO_SYS_ORDER_5TH> Eye;
  Eye.setIdentity();
  Eye_alpha.setIdentity();

  double alpha = 2.0f * fs_;

  for (int i = 0; i < 5; i++) {
    Eye_alpha(i, i) = alpha;
  }

  Eigen::Matrix<double, SISO_SYS_ORDER_5TH, SISO_SYS_ORDER_5TH> tmp;
  tmp = Eye_alpha - Ac_;

  Eigen::Matrix<double, SISO_SYS_ORDER_5TH, SISO_SYS_ORDER_5TH> tmp_inv =
      tmp.inverse();
  Ad_ = tmp_inv * (Eye_alpha + Ac_);
  Bd_ = tmp_inv * Bc_;
  Cd_ = Cc_ * (Ad_ + Eye);
  Dd_ = Cc_ * Bd_ + Dc_;

  InitSSdiscrete(Ad_, Bd_, Cd_, Dd_);
}

void StatespaceSISOSys5th::InitTFcontinuous(const double *coef_p,
                                            const double *coef_z,
                                            const double fs) {
  for (int i = 0; i < 6; i++) {
    num_[i] = coef_z[i];
    den_[i] = coef_p[i];
  }

  double a[5] = {};
  int i = 0;
  for (i = 0; i < 5; i++) {
    a[i] = coef_p[5 - i] / coef_p[0];
  }

  double b[6] = {};
  for (i = 0; i < 6; i++) {
    b[i] = coef_z[5 - i] / coef_p[0];
  }

  double h[6] = {};
  h[0] = b[5];
  h[1] = b[4] - a[4] * h[0];
  h[2] = b[3] - a[4] * h[1] - a[3] * h[0];
  h[3] = b[2] - a[4] * h[2] - a[3] * h[1] - a[2] * h[0];
  h[4] = b[1] - a[4] * h[3] - a[3] * h[2] - a[2] * h[1] - a[1] * h[0];
  h[5] = b[0] - a[4] * h[4] - a[3] * h[3] - a[2] * h[2] - a[1] * h[1] -
         a[0] * h[0];

  Ac_ << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -a[0], -a[1], -a[2],
      -a[3], -a[4];

  Bc_ << h[1], h[2], h[3], h[4], h[5];

  Cc_.setZero();
  Cc_(0, 0) = 1.0f;

  Dc_(0, 0) = h[0];

  InitSScontinuous(Ac_, Bc_, Cc_, Dc_, fs);
}

void StatespaceSISOSys5th::Update(double u) {
  if (!std::isnan(u)) {
    u_(0, 0) = u;
    y_ = Cd_ * x_ + Dd_ * u_;
    x_ = Ad_ * x_ + Bd_ * u_;
  } else {
    Reset();
  }
}

void StatespaceSISOSys5th::SwitchBuf(double u, double y) {
  if (std::isnan(u) || std::isnan(y)) {
    Reset();
  } else {
    u_(0, 0) = u;
    y_(0, 0) = y;
    x_ = Cd_pinv_ * (y_ - Dd_ * u_);
    x_ = Ad_ * x_ + Bd_ * u_;
  }
}

double StatespaceSISOSys5th::GetOutput() { return y_(0, 0); }

/* -------------------- for sixth-order system -------------------- */
void StatespaceSISOSys6th::Reset() {
  u_.setZero();
  y_.setZero();
  x_.setZero();
}

void StatespaceSISOSys6th::InitSSdiscrete(
    Eigen::Matrix<double, SISO_SYS_ORDER_6TH, SISO_SYS_ORDER_6TH> a,
    Eigen::Matrix<double, SISO_SYS_ORDER_6TH, 1> b,
    Eigen::Matrix<double, 1, SISO_SYS_ORDER_6TH> c,
    Eigen::Matrix<double, 1, 1> d) {
  Reset();
  inited_flag_ = true;
  Ad_ = a;
  Bd_ = b;
  Cd_ = c;
  Dd_ = d;
  double tmp = Cd_(0, 0) * Cd_(0, 0) + Cd_(0, 1) * Cd_(0, 1) +
               Cd_(0, 2) * Cd_(0, 2) + Cd_(0, 3) * Cd_(0, 3) +
               Cd_(0, 4) * Cd_(0, 4) + Cd_(0, 5) * Cd_(0, 5);
  for (int i = 0; i < 6; i++) {
    Cd_pinv_(i, 0) = c(0, i) / tmp;
  }
}

void StatespaceSISOSys6th::InitSScontinuous(
    Eigen::Matrix<double, SISO_SYS_ORDER_6TH, SISO_SYS_ORDER_6TH> a,
    Eigen::Matrix<double, SISO_SYS_ORDER_6TH, 1> b,
    Eigen::Matrix<double, 1, SISO_SYS_ORDER_6TH> c,
    Eigen::Matrix<double, 1, 1> d, double fs) {
  Ac_ = a;
  Bc_ = b;
  Cc_ = c;
  Dc_ = d;
  fs_ = fs;

  Eigen::Matrix<double, SISO_SYS_ORDER_6TH, SISO_SYS_ORDER_6TH> Eye_alpha;
  Eigen::Matrix<double, SISO_SYS_ORDER_6TH, SISO_SYS_ORDER_6TH> Eye;
  Eye.setIdentity();
  Eye_alpha.setIdentity();

  double alpha = 2.0f * fs_;

  for (int i = 0; i < 6; i++) {
    Eye_alpha(i, i) = alpha;
  }

  Eigen::Matrix<double, SISO_SYS_ORDER_6TH, SISO_SYS_ORDER_6TH> tmp;
  tmp = Eye_alpha - Ac_;

  Eigen::Matrix<double, SISO_SYS_ORDER_6TH, SISO_SYS_ORDER_6TH> tmp_inv =
      tmp.inverse();
  Ad_ = tmp_inv * (Eye_alpha + Ac_);
  Bd_ = tmp_inv * Bc_;
  Cd_ = Cc_ * (Ad_ + Eye);
  Dd_ = Cc_ * Bd_ + Dc_;

  InitSSdiscrete(Ad_, Bd_, Cd_, Dd_);
}

void StatespaceSISOSys6th::InitTFcontinuous(const double *coef_p,
                                            const double *coef_z,
                                            const double fs) {
  for (int i = 0; i < 7; i++) {
    num_[i] = coef_z[i];
    den_[i] = coef_p[i];
  }

  double a[6] = {};
  int i = 0;
  for (i = 0; i < 6; i++) {
    a[i] = coef_p[6 - i] / coef_p[0];
  }

  double b[7] = {};
  for (i = 0; i < 7; i++) {
    b[i] = coef_z[6 - i] / coef_p[0];
  }

  double h[7] = {};
  h[0] = b[6];
  h[1] = b[5] - a[5] * h[0];
  h[2] = b[4] - a[5] * h[1] - a[4] * h[0];
  h[3] = b[3] - a[5] * h[2] - a[4] * h[1] - a[3] * h[0];
  h[4] = b[2] - a[5] * h[3] - a[4] * h[2] - a[3] * h[1] - a[2] * h[0];
  h[5] = b[1] - a[5] * h[4] - a[4] * h[3] - a[3] * h[2] - a[2] * h[1] -
         a[1] * h[0];
  h[6] = b[0] - a[5] * h[5] - a[4] * h[4] - a[3] * h[3] - a[2] * h[2] -
         a[1] * h[1] - a[0] * h[0];

  Ac_ << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -a[0], -a[1], -a[2], -a[3], -a[4],
      -a[5];

  Bc_ << h[1], h[2], h[3], h[4], h[5], h[6];

  Cc_.setZero();
  Cc_(0, 0) = 1.0f;

  Dc_(0, 0) = h[0];

  InitSScontinuous(Ac_, Bc_, Cc_, Dc_, fs);
}

void StatespaceSISOSys6th::Update(double u) {
  if (!std::isnan(u)) {
    u_(0, 0) = u;
    y_ = Cd_ * x_ + Dd_ * u_;
    x_ = Ad_ * x_ + Bd_ * u_;
  } else {
    Reset();
  }
}

void StatespaceSISOSys6th::SwitchBuf(double u, double y) {
  if (std::isnan(u) || std::isnan(y)) {
    Reset();
  } else {
    u_(0, 0) = u;
    y_(0, 0) = y;
    x_ = Cd_pinv_ * (y_ - Dd_ * u_);
    x_ = Ad_ * x_ + Bd_ * u_;
  }
}

double StatespaceSISOSys6th::GetOutput() { return y_(0, 0); }

void StatespaceMIMO::Init(const Eigen::MatrixXd &Ac, const Eigen::MatrixXd &Bc,
                          const Eigen::MatrixXd &Cc, const Eigen::MatrixXd &Dc,
                          const double fs) {
  int n_state = Ac.cols();
  int n_input = Bc.cols();
  int n_ouput = Cc.rows();

  Ad_.resize(n_state, n_state);
  Bd_.resize(n_state, n_input);
  Cd_.resize(n_ouput, n_state);
  Dd_.resize(n_ouput, n_input);

  double a = 2.0 * fs;
  Eigen::MatrixXd tmp;
  Eigen::MatrixXd Eye;
  Eye.resize(n_state, n_state);
  tmp.resize(n_state, n_state);
  Eye.setIdentity();

  tmp = (a * Eye - Ac);
  Ad_ = tmp.inverse() * (a * Eye + Ac);
  Bd_ = tmp.inverse() * Bc;
  Cd_ = Cc * (Ad_ + Eye);
  Dd_ = Cc * Bd_ + Dc;

  Cd_pinv_ = Cd_.completeOrthogonalDecomposition().pseudoInverse();

  X_.resize(n_state, 1);
  U_.resize(n_input, 1);
  Y_.resize(n_ouput, 1);
  init_flag_ = true;
}

void StatespaceMIMO::Reset() {
  U_.setZero();
  Y_.setZero();
  X_.setZero();
}

void StatespaceMIMO::SetInitState(const Eigen::MatrixXd &X0) { X_ = X0; }

void StatespaceMIMO::GetState(Eigen::MatrixXd &state) { state = X_; }

void StatespaceMIMO::GetOutput(Eigen::MatrixXd &out) { out = Y_; }

void StatespaceMIMO::Update(const Eigen::MatrixXd &U) {
  U_ = U;
  Y_ = Cd_ * X_ + Dd_ * U_;
  X_ = Ad_ * X_ + Bd_ * U_;
}

void StatespaceMIMO::SwitchBuf(Eigen::MatrixXd &U, Eigen::MatrixXd &Y) {
  X_ = Cd_pinv_ * (Y - Dd_ * U);
  Update(U);
}

} // namespace statespace_sys
} // namespace pnc
