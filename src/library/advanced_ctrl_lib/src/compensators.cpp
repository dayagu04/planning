/***************************************************************
 * @file       compensators.cpp
 * @brief      for compensators design
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-17-2021
 **************************************************************/

#include "compensators.h"
#include "math_lib.h"

#ifndef C_PI_F
#define C_PI_F (3.141592653589793f)
#endif

#ifndef eps
#define eps (1e-6)
#endif

#define SQUARE(X) ((X) * (X))
namespace pnc {
namespace compensators {
using namespace mathlib;
void LeadSys::InitLeadSys(double kp, double lead_fc, double lead_gain, double fs) {
  lead_fc_ = lead_fc;
  lead_gain_ = lead_gain;
  kp_ = kp;

  double coef_z[2] = {};
  double coef_p[2] = {};

  coef_z[0] = 1.0f / (2.0f * C_PI_F * lead_fc) * kp;
  coef_z[1] = 1.0f * kp;
  coef_p[0] = 1.0f / (2.0f * C_PI_F * lead_fc * lead_gain);
  coef_p[1] = 1.0f;

  InitTFcontinuous(coef_p, coef_z, fs);
}

void LeadSys::SetOutBound(double out_min, double out_max) {
  out_min_ = out_min;
  out_max_ = out_max;
}

void LeadSys::UpdatewithSaturation(double u) {
  Update(u);
  y_ = DoubleConstrain(y_, out_min_, out_max_);
}

void LeadboostSys::InitLeadboostSys(double kp, double lead_fc, double lead_gain, double boost_fc, double boost_gain,
                                    double fs) {
  kp_ = kp;
  lead_fc_ = lead_fc;
  lead_gain_ = lead_gain;
  boost_fc_ = boost_fc;
  boost_gain_ = boost_gain;

  double coef_z[3] = {};
  double coef_p[3] = {};

  coef_z[0] = kp * boost_gain * lead_gain;
  coef_z[1] = kp * 2.0f * C_PI_F * boost_fc * boost_gain * lead_gain + 2.0f * C_PI_F * boost_gain * lead_fc * lead_gain;
  coef_z[2] = kp * 4.0f * SQUARE(C_PI_F) * boost_fc * boost_gain * lead_fc * lead_gain;

  coef_p[0] = boost_gain;
  coef_p[1] = 2.0f * C_PI_F * boost_fc + 2.0f * C_PI_F * boost_gain * lead_fc * lead_gain;
  coef_p[2] = 4.0f * SQUARE(C_PI_F) * boost_fc * lead_fc * lead_gain;

  InitTFcontinuous(coef_p, coef_z, fs);
}

void LeadboostSys::SetOutBound(double out_min, double out_max) {
  out_min_ = out_min;
  out_max_ = out_max;
}

void LeadboostSys::UpdatewithSaturation(double u) {
  Update(u);
  y_(0, 0) = DoubleConstrain(y_(0, 0), out_min_, out_max_);
}

void IntSubsys::InitIntSys(double fi, double i_out_min, double i_out_max, double reset_gain, double reset_dead_zone,
                           double fs) {
  i_out_min_ = i_out_min;
  i_out_max_ = i_out_max;
  reset_gain_ = reset_gain;
  reset_dead_zone_ = reset_dead_zone;
  i_gain_ = 2.0f * C_PI_F * fi;
  dynamic_i_gain_ = 1.0f;
  saturation_flag_ = false;
  double coef_z[2] = {0.0f, i_gain_};
  double coef_p[2] = {1.0f, 0.0f};
  InitTFcontinuous(coef_p, coef_z, fs);
}

void IntSubsys::SetSaturationFlag(bool saturation_flag) { saturation_flag_ = saturation_flag; }

void IntSubsys::SetDynamicIgain(double dynamic_i_gain) {
  if (std::fabs(dynamic_i_gain_ - dynamic_i_gain) <= eps) {
    /* record original input and output */
    double u = u_;
    double y = y_;

    /* set sys by dynamic_i_gain */
    dynamic_i_gain_ = dynamic_i_gain;
    double coef_z[2] = {0.0f, dynamic_i_gain_ * i_gain_};
    double coef_p[2] = {1.0f, 0.0f};

    InitTFcontinuous(coef_p, coef_z, fs_);

    /* Reset original input and output */
    SwitchBuf(u, y);
  }
}

void IntSubsys::UpdatewithSaturation(double u) {
  if ((u * (GetOutput() + reset_dead_zone_) < 0.0f) && (u * (GetOutput() - reset_dead_zone_) < 0.0f)) {
    SetDynamicIgain(reset_gain_);
  } else {
    SetDynamicIgain(1.0f);
  }

  if (!saturation_flag_) {
    Update(u);
  }

  if (GetOutput() > i_out_max_) {
    SwitchBuf(u, i_out_max_);
  } else if (GetOutput() < i_out_min_) {
    SwitchBuf(u, i_out_min_);
  }
}

double IntSubsys::GetIoutMin() { return i_out_min_; }
double IntSubsys::GetIoutMax() { return i_out_max_; }

void PISys::InitPISys(double kp, double fi, double out_min, double out_max, double i_out_min, double i_out_max,
                      double reset_gain, double reset_dead_zone, double fs) {
  kp_ = kp;
  out_min_ = out_min;
  out_max_ = out_max;
  intesys_.InitIntSys(fi, i_out_min, i_out_max, reset_gain, reset_dead_zone, fs);
  u_ = 0.0f;
  y_ = 0.0f;
  kp_term_ = 0.0f;
  ki_term_ = 0.0f;
  kp_gain_ = 1.0f;
  ki_gain_ = 1.0f;
}

void PISys::Update(double u) {
  kp_term_ = kp_gain_ * kp_ * u;
  intesys_.SetSaturationFlag(false);
  intesys_.UpdatewithSaturation(ki_gain_ * u);
  ki_term_ = intesys_.GetOutput();
  y_ = kp_term_ + ki_term_;
  y_ = DoubleConstrain(y_, out_min_, out_max_);
}

void PISys::Update(double u, bool saturation_flag) {
  kp_term_ = kp_gain_ * kp_ * u;
  intesys_.SetSaturationFlag(saturation_flag);
  intesys_.UpdatewithSaturation(ki_gain_ * u);
  ki_term_ = intesys_.GetOutput();
  y_ = kp_term_ + ki_term_;
  y_ = DoubleConstrain(y_, out_min_, out_max_);
}

void PISys::Reset() {
  u_ = 0.0f;
  y_ = 0.0f;
  kp_term_ = 0.0f;
  ki_term_ = 0.0f;
  intesys_.Reset();
  intesys_.SetSaturationFlag(false);
}

void PISys::SwitchBuf(double u, double y) {
  u_ = u;
  kp_term_ = kp_ * u;
  double ki_term = y - kp_term_;
  ki_term_ = DoubleConstrain(ki_term, intesys_.GetIoutMin(), intesys_.GetIoutMax());
  intesys_.SwitchBuf(u, ki_term_);
  y_ = DoubleConstrain(kp_term_ + ki_term_, out_min_, out_max_);
}

double PISys::GetOutput() { return y_; }

double PISys::GetkpTerm() { return kp_term_; }

double PISys::GetkiTerm() { return ki_term_; }

void PILeadSys::InitPILeadSys(double kp, double fi, double lead_fc, double lead_gain, double out_min, double out_max,
                              double i_out_min, double i_out_max, double reset_gain, double reset_dead_zone,
                              double fs) {
  kp_ = kp;
  out_min_ = out_min;
  out_max_ = out_max;
  pi_sys_.InitPISys(kp, fi, out_min, out_max, i_out_min, i_out_max, reset_gain, reset_dead_zone, fs);
  lead_sys_.InitLeadSys(1.0f, lead_fc, lead_gain, fs); /* lead kp = 1.0 */
  u_ = 0.0f;
  y_ = 0.0f;
  kp_term_ = 0.0f;
  ki_term_ = 0.0f;
  kcomp_term_ = 0.0f;
}

void PILeadSys::InitPILeadSys(double kp, double fi, double lead_fc, double lead_gain, double out_min, double out_max,
                              double fs) {
  kp_ = kp;
  out_min_ = out_min;
  out_max_ = out_max;
  pi_sys_.InitPISys(kp, fi, out_min, out_max, 0.0, 0.0, 1.0, 0.0, fs);
  lead_sys_.InitLeadSys(1.0f, lead_fc, lead_gain, fs); /* lead kp = 1.0 */
  u_ = 0.0f;
  y_ = 0.0f;
  kp_term_ = 0.0f;
  ki_term_ = 0.0f;
  kcomp_term_ = 0.0f;
}

void PILeadSys::Reset() {
  u_ = 0.0f;
  y_ = 0.0f;
  kp_term_ = 0.0f;
  ki_term_ = 0.0f;
  kcomp_term_ = 0.0f;
  pi_sys_.Reset();
  lead_sys_.Reset();
}

void PILeadSys::Update(double u) {
  /* update lead sys first without saturation */
  u_ = u;
  lead_sys_.Update(u);
  kcomp_term_ = lead_sys_.GetOutput();

  /* update pi sys then */
  pi_sys_.Update(kcomp_term_);
  kp_term_ = pi_sys_.GetkpTerm();
  ki_term_ = pi_sys_.GetkiTerm();

  y_ = pi_sys_.GetOutput();
}

void PILeadSys::Update(double u, bool saturation_flag) {
  /* update lead sys first without saturation */
  u_ = u;
  lead_sys_.Update(u);
  kcomp_term_ = lead_sys_.GetOutput();

  /* update pi sys then */
  pi_sys_.Update(kcomp_term_, saturation_flag);
  kp_term_ = pi_sys_.GetkpTerm();
  ki_term_ = pi_sys_.GetkiTerm();

  y_ = pi_sys_.GetOutput();
}

void PILeadSys::SwitchBuf(double u, double y) {
  /* when switch buf, kcomp_term gain can be 1.0 */
  u_ = u;
  lead_sys_.Reset();
  lead_sys_.SwitchBuf(u, u);
  pi_sys_.SwitchBuf(u, y);
  kcomp_term_ = u;
  kp_term_ = pi_sys_.GetkpTerm();
  ki_term_ = pi_sys_.GetkiTerm();
  y_ = pi_sys_.GetOutput();
}

double PILeadSys::GetOutput() { return y_; }

void PILeadboostSys::InitPILeadboostSys(double kp, double fi, double lead_fc, double lead_gain, double boost_fc,
                                        double boost_gain, double out_min, double out_max, double i_out_min,
                                        double i_out_max, double reset_gain, double reset_dead_zone, double fs) {
  kp_ = kp;
  out_min_ = out_min;
  out_max_ = out_max;
  pi_sys_.InitPISys(kp, fi, out_min, out_max, i_out_min, i_out_max, reset_gain, reset_dead_zone, fs);
  leadboost_sys_.InitLeadboostSys(1.0f, lead_fc, lead_gain, boost_fc, boost_gain, fs); /* lead kp = 1.0 */
  u_ = 0.0f;
  y_ = 0.0f;
  kp_term_ = 0.0f;
  ki_term_ = 0.0f;
  kcomp_term_ = 0.0f;
}

void PILeadboostSys::Reset() {
  u_ = 0.0f;
  y_ = 0.0f;
  kp_term_ = 0.0f;
  ki_term_ = 0.0f;
  kcomp_term_ = 0.0f;
  pi_sys_.Reset();
  leadboost_sys_.Reset();
}

void PILeadboostSys::Update(double u) {
  /* update lead sys first without saturation */
  u_ = u;
  leadboost_sys_.Update(u);
  kcomp_term_ = leadboost_sys_.GetOutput();

  /* update pi sys then */
  pi_sys_.Update(kcomp_term_);
  kp_term_ = pi_sys_.GetkpTerm();
  ki_term_ = pi_sys_.GetkiTerm();

  y_ = pi_sys_.GetOutput();
}

void PILeadboostSys::SwitchBuf(double u, double y) {
  /* when switch buf, kcomp_term gain can be 1.0 */
  u_ = u;
  leadboost_sys_.Reset();
  leadboost_sys_.SwitchBuf(u, u);
  pi_sys_.SwitchBuf(u, y);
  kcomp_term_ = u;
  kp_term_ = pi_sys_.GetkpTerm();
  ki_term_ = pi_sys_.GetkiTerm();
  y_ = pi_sys_.GetOutput();
}

double PILeadboostSys::GetOutput() { return y_; }

void VehCompSys::Reset(double y0) {
  x1_k_1_ = 0.0;
  y1_k_1_ = 0.0;
  x2_k_1_ = 0.0;
  y2_k_1_ = 0.0;
  x3_k_1_ = 0.0;
  y3_k_1_ = 0.0;
  x4_k_1_ = 0.0;
  y4_k_1_ = 0.0;
  output_k_1_ = y0;
}

double VehCompSys::OneStep(double u) {
  double y = 0.0;
  y = pn_1_ * x1_k_1_ + pn_2_ * x2_k_1_ + pn_3_ * x3_k_1_ + pn_4_ * x4_k_1_ -
      (pd_1_ * y1_k_1_ + pd_2_ * y2_k_1_ + pd_3_ * y3_k_1_ + pd_4_ * y4_k_1_);
  y4_k_1_ = y3_k_1_;
  y3_k_1_ = y2_k_1_;
  y2_k_1_ = y1_k_1_;
  y1_k_1_ = y;
  x4_k_1_ = x3_k_1_;
  x3_k_1_ = x2_k_1_;
  x2_k_1_ = x1_k_1_;
  x1_k_1_ = u;
  return y;
}

void VehCompSys::Config(double pn1, double pn2, double pn3, double pn4, double pd1, double pd2, double pd3,
                        double pd4) {
  pn_1_ = pn1;
  pn_2_ = pn2;
  pn_3_ = pn3;
  pn_4_ = pn4;
  pd_1_ = pd1;
  pd_2_ = pd2;
  pd_3_ = pd3;
  pd_4_ = pd4;
}

double VehCompSys::Output(double u) {
  double y_k = 0.0;
  y_k = 0.02 * u + output_k_1_;
  output_k_1_ = y_k;
  return y_k;
}

}  // namespace compensators

}  // namespace pnc
