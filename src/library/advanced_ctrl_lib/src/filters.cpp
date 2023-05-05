/***************************************************************
 * @file       filters.cpp
 * @brief      for filters design
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-17-2021
 **************************************************************/

#include "filters.h"
#include "Eigen/LU"
#include "math_lib.h"
#include <cmath>

#define BUTTER_COEF_1ST                                                        \
  { 1.0f, 1.0f }
#define BUTTER_COEF_2ND                                                        \
  { 1.0f, 1.414213562373095f, 1.0f }
#define BUTTER_COEF_3RD                                                        \
  { 1.0f, 2.0f, 2.0f, 1.0f }
#define BUTTER_COEF_4TH                                                        \
  { 1.0f, 2.613125929752753f, 3.414213562373095f, 2.613125929752753f, 1.0f }
#define BUTTER_COEF_5TH                                                        \
  {                                                                            \
    1.0f, 3.236067977499789f, 5.236067977499789f, 5.236067977499789f,          \
        3.236067977499789f, 1.0f                                               \
  }
#define BUTTER_COEF_6TH                                                        \
  {                                                                            \
    1.0f, 3.863703305156273f, 7.464101615137753f, 9.141620172685640f,          \
        7.464101615137753f, 3.863703305156273f, 1.0f                           \
  }
#define BUTTER_COEF_7TH                                                        \
  {                                                                            \
    1.0f, 4.493959207434933f, 10.097834679044610f, 14.591793886479543f,        \
        14.591793886479543f, 10.097834679044610f, 4.493959207434933f, 1.0f     \
  }
#define BUTTER_COEF_8TH                                                        \
  {                                                                            \
    1.0f, 5.125830895483012f, 13.137071184544089f, 21.846150969207624f,        \
        25.688355931461274f, 21.846150969207628f, 13.137071184544089f,         \
        5.125830895483013f, 1.0f                                               \
  }

#ifndef C_PI_F
#define C_PI_F (3.141592653589793f)
#endif

namespace pnc {
namespace filters {
/* ------------------------------------------ Butterworth filters
 * ------------------------------------------ */
using namespace mathlib;
void ButterworthFilter::SetBasicCoef(double *coef, int Np, double fc,
                                     double fs) {
  /* predistoration to ensure accurate phase in fc */
  double fd = Predistoration(fc, fs);

  double coef0[MAX_BUTTER_ORDER + 1] = {};

  switch (Np) {
  case 0:
    break;
  case 1:
    Set1stCoef(coef0);
    break;
  case 2:
    Set2ndCoef(coef0);
    break;
  case 3:
    Set3rdCoef(coef0);
    break;
  case 4:
    Set4thCoef(coef0);
    break;
  case 5:
    Set5thCoef(coef0);
    break;
  case 6:
    Set6thCoef(coef0);
    break;
  case 7:
    Set7thCoef(coef0);
    break;
  case 8:
    Set8thCoef(coef0);
    break;
  default:
    coef0[0] = 1.0f;
  }

  memcpy(coef, coef0, (Np + 1) * sizeof(double));

  for (int i = 0; i < Np; i++) {
    *(coef + i) = *(coef + i) / pow((2.0f * C_PI_F * fd), (double)((Np - i)));
  }
}

void ButterworthFilter::SetTFCoef(int Np, int Nz, double fp, double fz,
                                  double fs) {
  Np_ = Np;
  Nz_ = Nz;
  fp_ = fp;
  fz_ = fz;
  fs_ = fs;
  SetBasicCoef(coef_p_, Np, fp, fs);
  SetBasicCoef(coef_z_, Nz, fz, fs);
}

void ButterworthFilter::GetCoefPolesZeros(double *coef_p, double *coef_z) {
  memcpy(coef_p, coef_p_, (Np_ + 1) * sizeof(double));
  memcpy(coef_z, coef_z_, (Nz_ + 1) * sizeof(double));
}

double ButterworthFilter::Predistoration(double fc, double fs) {
  return (fs / C_PI_F) * tanf(C_PI_F / fs * fc);
}

void ButterworthFilter::Set1stCoef(double *coef) {
  double coef_norm[] = BUTTER_COEF_1ST;
  memcpy(coef, coef_norm, 2 * sizeof(double));
}

void ButterworthFilter::Set2ndCoef(double *coef) {
  double coef_norm[] = BUTTER_COEF_2ND;
  memcpy(coef, coef_norm, 3 * sizeof(double));
}

void ButterworthFilter::Set3rdCoef(double *coef) {
  double coef_norm[] = BUTTER_COEF_3RD;
  memcpy(coef, coef_norm, 4 * sizeof(double));
}

void ButterworthFilter::Set4thCoef(double *coef) {
  double coef_norm[] = BUTTER_COEF_4TH;
  memcpy(coef, coef_norm, 5 * sizeof(double));
}

void ButterworthFilter::Set5thCoef(double *coef) {
  double coef_norm[] = BUTTER_COEF_5TH;
  memcpy(coef, coef_norm, 6 * sizeof(double));
}

void ButterworthFilter::Set6thCoef(double *coef) {
  double coef_norm[] = BUTTER_COEF_6TH;
  memcpy(coef, coef_norm, 7 * sizeof(double));
}

void ButterworthFilter::Set7thCoef(double *coef) {
  double coef_norm[] = BUTTER_COEF_7TH;
  memcpy(coef, coef_norm, 8 * sizeof(double));
}

void ButterworthFilter::Set8thCoef(double *coef) {
  double coef_norm[] = BUTTER_COEF_8TH;
  memcpy(coef, coef_norm, 9 * sizeof(double));
}

void ButterworthFilter1st::InitButterSysLowpass(int Nz, double fp, double fz,
                                                double fs) {
  if (Nz > 1) {
    Nz = 1;
  }
  SetTFCoef(1, Nz, fp, fz, fs);

  if (Nz_ < Np_) {
    for (int i = 0; i < Nz_ + 1; i++) {
      coef_z_[Np_ - i] = coef_z_[Nz_ - i];
    }
    for (int i = 0; i < Np_ - Nz_; i++) {
      coef_z_[i] = 0.0f;
    }
  }

  if (Nz == 0) {
    coef_z_[Np_] = 1.0;
  }
  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void ButterworthFilter1st::SwitchBuf(double u, double y) {
  u_ = u;
  y_ = y;
  x_ = 1.0 / (1 - Ad_) * Bd_ * u_;
}

void ButterworthFilter2nd::InitButterSysLowpass(int Nz, double fp, double fz,
                                                double fs) {
  if (Nz > 2) {
    Nz = 2;
  }

  SetTFCoef(2, Nz, fp, fz, fs);

  if (Nz_ < Np_) {
    for (int i = 0; i < Nz_ + 1; i++) {
      coef_z_[Np_ - i] = coef_z_[Nz_ - i];
    }
    for (int i = 0; i < Np_ - Nz_; i++) {
      coef_z_[i] = 0.0f;
    }
  }

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void ButterworthFilter2nd::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix2d tmp = Eigen::Matrix2d::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

void ButterworthFilter3rd::InitButterSysLowpass(int Nz, double fp, double fz,
                                                double fs) {
  if (Nz > 3) {
    Nz = 3;
  }

  SetTFCoef(3, Nz, fp, fz, fs);

  if (Nz_ < Np_) {
    for (int i = 0; i < Nz_ + 1; i++) {
      coef_z_[Np_ - i] = coef_z_[Nz_ - i];
    }
    for (int i = 0; i < Np_ - Nz_; i++) {
      coef_z_[i] = 0.0f;
    }
  }

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void ButterworthFilter3rd::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix3d tmp = Eigen::Matrix3d::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

void ButterworthFilter4th::InitButterSysLowpass(int Nz, double fp, double fz,
                                                double fs) {
  if (Nz > 4) {
    Nz = 4;
  }

  SetTFCoef(4, Nz, fp, fz, fs);

  if (Nz_ < Np_) {
    for (int i = 0; i < Nz_ + 1; i++) {
      coef_z_[Np_ - i] = coef_z_[Nz_ - i];
    }
    for (int i = 0; i < Np_ - Nz_; i++) {
      coef_z_[i] = 0.0f;
    }
  }

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void ButterworthFilter4th::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

void ButterworthFilter5th::InitButterSysLowpass(int Nz, double fp, double fz,
                                                double fs) {
  if (Nz > 5) {
    Nz = 5;
  }

  SetTFCoef(5, Nz, fp, fz, fs);

  if (Nz_ < Np_) {
    for (int i = 0; i < Nz_ + 1; i++) {
      coef_z_[Np_ - i] = coef_z_[Nz_ - i];
    }
    for (int i = 0; i < Np_ - Nz_; i++) {
      coef_z_[i] = 0.0f;
    }
  }

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void ButterworthFilter5th::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix<double, 5, 5> tmp =
      Eigen::Matrix<double, 5, 5>::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

void ButterworthFilter6th::InitButterSysLowpass(int Nz, double fp, double fz,
                                                double fs) {
  if (Nz > 6) {
    Nz = 6;
  }

  SetTFCoef(6, Nz, fp, fz, fs);

  if (Nz_ < Np_) {
    for (int i = 0; i < Nz_ + 1; i++) {
      coef_z_[Np_ - i] = coef_z_[Nz_ - i];
    }
    for (int i = 0; i < Np_ - Nz_; i++) {
      coef_z_[i] = 0.0f;
    }
  }

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void ButterworthFilter6th::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix<double, 6, 6> tmp =
      Eigen::Matrix<double, 6, 6>::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

/* ------------------------------------------ slope filters
 * ------------------------------------------ */
void SlopeFilter::Reset() {
  state_ = 0.0;
  last_state_ = 0.0;
}

void SlopeFilter::Init(double state, double min_update_rate,
                       double max_update_rate, double min_limit,
                       double max_limit, double dt) {
  init_flag_ = true;
  state_ = state;
  last_state_ = state;
  min_update_rate_ = min_update_rate;
  max_update_rate_ = max_update_rate;
  max_limit_ = max_limit;
  min_limit_ = min_limit;
  dt_ = dt;
}

void SlopeFilter::Init(double min_update_rate, double max_update_rate,
                       double min_limit, double max_limit, double dt) {
  min_update_rate_ = min_update_rate;
  max_update_rate_ = max_update_rate;
  max_limit_ = max_limit;
  min_limit_ = min_limit;
  dt_ = dt;
}

void SlopeFilter::Update(double u) {
  if (!init_flag_) {
    SetState(u);
    init_flag_ = true;
  } else if (std::isnan(u)) {
    Reset();
  } else {
    if (u - last_state_ > max_update_rate_ * dt_) {
      state_ = last_state_ + max_update_rate_ * dt_;
    } else if (u - last_state_ < min_update_rate_ * dt_) {
      state_ = last_state_ + min_update_rate_ * dt_;
    } else {
      state_ = u;
    }
    state_ = DoubleConstrain(state_, min_limit_, max_limit_);
    last_state_ = state_;
  }
}

void SlopeFilter::SetState(double state) {
  if (std::isnan(state)) {
    Reset();
  } else {
    state = DoubleConstrain(state, min_limit_, max_limit_);
    init_flag_ = true;
    state_ = state;
    last_state_ = state;
  }
}

void SlopeFilter::SetRate(double min_update_rate, double max_update_rate) {
  min_update_rate_ = min_update_rate;
  max_update_rate_ = max_update_rate;
}

void SlopeFilter::SetLimit(double min_limit, double max_limit) {
  min_limit_ = min_limit;
  max_limit_ = max_limit;
}

/* ------------------------------------------ Differentiators
 * ------------------------------------------ */
void Differentiator1st::InitDifferentiator(double fc, double fs) {
  SetBasicCoef(coef_p_, 1, fc, fs);
  int i = 0;
  for (i = 0; i < 2; i++) {
    coef_z_[i] = 0.0f;
  }
  coef_z_[i - 2] = 1.0f;

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void Differentiator1st::SwitchBuf(double u, double y) {
  u_ = u;
  y_ = y;
  x_ = 1.0 / (1 - Ad_) * Bd_ * u_;
}

void Differentiator2nd::InitDifferentiator(double fc, double fs) {
  SetBasicCoef(coef_p_, 2, fc, fs);
  int i = 0;
  for (i = 0; i < 3; i++) {
    coef_z_[i] = 0.0f;
  }
  coef_z_[i - 2] = 1.0f;

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void Differentiator2nd::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix2d tmp = Eigen::Matrix2d::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

void Differentiator3rd::InitDifferentiator(double fc, double fs) {
  SetBasicCoef(coef_p_, 3, fc, fs);
  int i = 0;
  for (i = 0; i < 4; i++) {
    coef_z_[i] = 0.0f;
  }
  coef_z_[i - 2] = 1.0f;

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void Differentiator3rd::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix3d tmp = Eigen::Matrix3d::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

void Differentiator4th::InitDifferentiator(double fc, double fs) {
  SetBasicCoef(coef_p_, 4, fc, fs);
  int i = 0;
  for (i = 0; i < 5; i++) {
    coef_z_[i] = 0.0f;
  }
  coef_z_[i - 2] = 1.0f;

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void Differentiator4th::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

void Differentiator5th::InitDifferentiator(double fc, double fs) {
  SetBasicCoef(coef_p_, 5, fc, fs);
  int i = 0;
  for (i = 0; i < 6; i++) {
    coef_z_[i] = 0.0f;
  }
  coef_z_[i - 2] = 1.0f;

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void Differentiator5th::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix<double, 5, 5> tmp =
      Eigen::Matrix<double, 5, 5>::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

void Differentiator6th::InitDifferentiator(double fc, double fs) {
  SetBasicCoef(coef_p_, 6, fc, fs);
  int i = 0;
  for (i = 0; i < 7; i++) {
    coef_z_[i] = 0.0f;
  }
  coef_z_[i - 2] = 1.0f;

  InitTFcontinuous(coef_p_, coef_z_, fs);
}

void Differentiator6th::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix<double, 6, 6> tmp =
      Eigen::Matrix<double, 6, 6>::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

double NotchFilter::Predistoration(double fc, double fs) {
  return (fs / C_PI_F) * tanf(C_PI_F / fs * fc);
}

void NotchFilter::InitNotchFilter(double fn, double bw, double gain,
                                  double fs) {
  fn_ = fn;
  gain_ = gain;
  bw_ = bw;

  // predistoration is indispensable for notch filter
  double fc = Predistoration(fn, fs);

  double coef_z[3] = {};
  double coef_p[3] = {};

  double ksi1 = bw;
  double ksi2 = bw / gain;
  double wn = 2.0f * C_PI_F * fc;

  coef_z[0] = 1.0f;
  coef_z[1] = 2.0f * ksi1 * wn;
  coef_z[2] = wn * wn;

  coef_p[0] = 1.0f;
  coef_p[1] = 2.0f * ksi2 * wn;
  coef_p[2] = wn * wn;

  InitTFcontinuous(coef_p, coef_z, fs);
}

void NotchFilter::SwitchBuf(double u, double y) {
  u_(0, 0) = u;
  y_(0, 0) = y;
  // x_ = Ad_ * x_ + Bd_ * u_;
  Eigen::Matrix2d tmp = Eigen::Matrix2d::Identity() - Ad_;
  x_ = tmp.inverse() * Bd_ * u_;
}

} // namespace filters
} // namespace pnc
