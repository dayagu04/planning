/***************************************************************
 * @file       compensators.h
 * @brief      for compensators design
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-17-2021
 **************************************************************/

#ifndef __COMPENSANTORS_H__
#define __COMPENSANTORS_H__

#include "filters.h"
#include "statespace_sys.h"

namespace pnc {
namespace compensators {
/* basic ctrl sys: lead */
class LeadSys : public statespace_sys::StatespaceSISOSys1st {
 public:
  void InitLeadSys(double kp, double lead_fc, double lead_gain, double fs);
  void SetOutBound(double out_min, double out_max);
  void UpdatewithSaturation(double u);

 protected:
  double kp_;
  double lead_fc_;
  double lead_gain_;
  double out_min_;
  double out_max_;
};

/* basic ctrl sys: leadboost */
class LeadboostSys : public statespace_sys::StatespaceSISOSys2nd {
 public:
  void InitLeadboostSys(double kp, double lead_fc, double lead_gain, double boost_fc, double boost_gain, double fs);
  void SetOutBound(double out_min, double out_max);
  void UpdatewithSaturation(double u);

 protected:
  double kp_;
  double lead_fc_;
  double lead_gain_;
  double boost_fc_;
  double boost_gain_;
  double out_min_;
  double out_max_;
};

/* basic ctrl sys: integrade */
class IntSubsys : public statespace_sys::StatespaceSISOSys1st {
 public:
  void InitIntSys(double fi, double i_out_min, double i_out_max, double reset_gain, double reset_dead_zone, double fs);
  void UpdatewithSaturation(double u);
  void SetDynamicIgain(double i_gain);
  void SetInteOutbound(double i_out_min, double i_out_max);
  void SetResetGain(double reset_gain);
  double GetIoutMin();
  double GetIoutMax();
  void SetSaturationFlag(bool saturation_flag);

#ifndef DEBUG
 protected:
#endif
  bool saturation_flag_;
  double fi_;
  double out_max_;
  double out_min_;
  double i_out_max_;
  double i_out_min_;
  double reset_gain_;
  double reset_dead_zone_;
  double i_gain_;
  double dynamic_i_gain_;
};

/* derived ctrl sys: PI */
class PISys {
 public:
  void Reset();
  void Update(double u);
  void Update(double u, bool saturation_flag);
  void SwitchBuf(double u, double y);
  double GetOutput();
  void InitPISys(double kp, double fi, double out_min, double out_max, double i_out_min, double i_out_max,
                 double reset_gain, double reset_dead_zone, double fs);
  double GetkpTerm();
  double GetkiTerm();
  void SetKpGain(double kp_gain) { kp_gain_ = kp_gain; }
  void SetKiGain(double ki_gain) { ki_gain_ = ki_gain; }

#ifndef DEBUG
 protected:
#endif
  IntSubsys intesys_;
  double kp_;
  double out_min_;
  double out_max_;
  double u_;
  double y_;
  double kp_term_;
  double ki_term_;
  double kp_gain_;
  double ki_gain_;
};

/* derived ctrl sys: PI-lead */
class PILeadSys {
 public:
  void Reset();
  void Update(double u);
  void Update(double u, bool saturation_flag);
  void SwitchBuf(double u, double y);
  double GetOutput();
  double GetCompTerm() { return kcomp_term_; }
  double GetKPTerm() { return kp_term_; }
  double GetKITerm() { return ki_term_; }
  void InitPILeadSys(double kp, double fi, double lead_fc, double lead_gain, double out_min, double out_max,
                     double i_out_min, double i_out_max, double reset_gain, double reset_dead_zone, double fs);

  void InitPILeadSys(double kp, double fi, double lead_fc, double lead_gain, double out_min, double out_max, double fs);
  void SetKpGain(double kp_gain) { pi_sys_.SetKpGain(kp_gain); }
  void SetKiGain(double ki_gain) { pi_sys_.SetKiGain(ki_gain); }

#ifndef DEBUG
 protected:
#endif
  PISys pi_sys_;
  LeadSys lead_sys_;
  double kp_;
  double out_min_;
  double out_max_;
  double u_;
  double y_;
  double kcomp_term_;
  double kp_term_;
  double ki_term_;
};

/* derived ctrl sys: PI-leadboost */
class PILeadboostSys {
 public:
  void Reset();
  void Update(double u);
  void SwitchBuf(double u, double y);
  double GetOutput();
  void InitPILeadboostSys(double kp, double fi, double lead_fc, double lead_gain, double boost_fc, double boost_gain,
                          double out_min, double out_max, double i_out_min, double i_out_max, double reset_gain,
                          double reset_dead_zone, double fs);

#ifndef DEBUG
 protected:
#endif
  PISys pi_sys_;
  LeadboostSys leadboost_sys_;
  double kp_;
  double out_min_;
  double out_max_;
  double u_;
  double y_;
  double kcomp_term_;
  double kp_term_;
  double ki_term_;
};

/* vehicle compensator */
class VehCompSys {
 public:
  void Reset(double y0);
  double OneStep(double u);
  void Config(double pn1, double pn2, double pn3, double pn4, double pd1, double pd2, double pd3, double pd4);
  double Output(double u);
  double x1_k_1_ = 0.0;
  double y1_k_1_ = 0.0;
  double x2_k_1_ = 0.0;
  double y2_k_1_ = 0.0;
  double x3_k_1_ = 0.0;
  double y3_k_1_ = 0.0;
  double x4_k_1_ = 0.0;
  double y4_k_1_ = 0.0;
  double output_k_1_ = 0.0;
  double pn_1_ = 1.0;
  double pn_2_ = 0.0;
  double pn_3_ = 0.0;
  double pn_4_ = 0.0;
  double pd_1_ = 0.0;
  double pd_2_ = 0.0;
  double pd_3_ = 0.0;
  double pd_4_ = 0.0;
};

}  // namespace compensators
}  // namespace pnc

#endif
