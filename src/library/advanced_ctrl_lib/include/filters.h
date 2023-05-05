/***************************************************************
 * @file       filters.h
 * @brief      for filters design
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-27-2021
 **************************************************************/

#ifndef __ADVANCED_FILTERS_H__
#define __ADVANCED_FILTERS_H__

#include "statespace_sys.h"

#define MAX_BUTTER_ORDER (8)

namespace pnc {
namespace filters {
/* ------------------------------------------ Butterworth filters
 * ------------------------------------------ */
class ButterworthFilter {
public:
  void SetBasicCoef(double *coef, int Np, double fc, double fs);
  void SetTFCoef(int Np, int Nz, double fp, double fz, double fs);
  void GetCoefPolesZeros(double *coef_p, double *coef_z);

#ifndef DEBUG
protected:
#endif
  double fp_;
  double fz_;
  double fs_;
  double coef_p_[MAX_BUTTER_ORDER];
  double coef_z_[MAX_BUTTER_ORDER];
  int Np_;
  int Nz_;

  double Predistoration(double fc, double fs);
  void SetCoef(double *coef, int Np, double fd);
  void Set1stCoef(double *coef);
  void Set2ndCoef(double *coef);
  void Set3rdCoef(double *coef);
  void Set4thCoef(double *coef);
  void Set5thCoef(double *coef);
  void Set6thCoef(double *coef);
  void Set7thCoef(double *coef);
  void Set8thCoef(double *coef);
};

class ButterworthFilter1st : public statespace_sys::StatespaceSISOSys1st,
                             public ButterworthFilter {
public:
  void InitButterSysLowpass(int Nz, double fp, double fz,
                            double fs); /* Np is 1 for first-order filter */
  void SwitchBuf(double u, double y) override;

protected:
};

class ButterworthFilter2nd : public statespace_sys::StatespaceSISOSys2nd,
                             public ButterworthFilter {
public:
  void InitButterSysLowpass(int Nz, double fp, double fz,
                            double fs); /* Np is 1 for first-order filter */
  void SwitchBuf(double u, double y) override;

protected:
};

class ButterworthFilter3rd : public statespace_sys::StatespaceSISOSys3rd,
                             public ButterworthFilter {
public:
  void InitButterSysLowpass(int Nz, double fp, double fz,
                            double fs); /* Np is 1 for first-order filter */
  void SwitchBuf(double u, double y) override;

protected:
};
class ButterworthFilter4th : public statespace_sys::StatespaceSISOSys4th,
                             public ButterworthFilter {
public:
  void InitButterSysLowpass(int Nz, double fp, double fz,
                            double fs); /* Np is 1 for first-order filter */
  void SwitchBuf(double u, double y) override;

protected:
};

class ButterworthFilter5th : public statespace_sys::StatespaceSISOSys5th,
                             public ButterworthFilter {
public:
  void InitButterSysLowpass(int Nz, double fp, double fz,
                            double fs); /* Np is 1 for first-order filter */
  void SwitchBuf(double u, double y) override;

protected:
};

class ButterworthFilter6th : public statespace_sys::StatespaceSISOSys6th,
                             public ButterworthFilter {
public:
  void InitButterSysLowpass(int Nz, double fp, double fz,
                            double fs); /* Np is 1 for first-order filter */
  void SwitchBuf(double u, double y) override;

protected:
};

/* ------------------------------------------ slope filters
 * ------------------------------------------ */
class SlopeFilter {
public:
  void Init(double state, double min_update_rate, double max_update_rate,
            double min_limit, double max_limit, double dt);
  void Init(double min_update_rate, double max_update_rate, double min_limit,
            double max_limit, double dt);
  void SetRate(double min_update_rate, double max_update_rate);
  void SetLimit(double min_limit, double max_limit);
  void SetState(double state);
  void Update(double u);
  void Reset();
  double GetOutput() { return state_; }

private:
  bool init_flag_;
  double state_;
  double last_state_;
  double min_update_rate_;
  double max_update_rate_;
  double max_limit_;
  double min_limit_;
  double dt_;
};

/* ------------------------------------------ Differentiators
 * ------------------------------------------ */
class Differentiator1st : public statespace_sys::StatespaceSISOSys1st,
                          public ButterworthFilter {
public:
  void InitDifferentiator(double fc, double fs);
  void SwitchBuf(double u, double y) override;

#ifndef DEBUG
protected:
#endif
  double fc_;
};

class Differentiator2nd : public statespace_sys::StatespaceSISOSys2nd,
                          public ButterworthFilter {
public:
  void InitDifferentiator(double fc, double fs);
  void SwitchBuf(double u, double y) override;

#ifndef DEBUG
protected:
#endif
  double fc_;
};

class Differentiator3rd : public statespace_sys::StatespaceSISOSys3rd,
                          public ButterworthFilter {
public:
  void InitDifferentiator(double fc, double fs);
  void SwitchBuf(double u, double y) override;

#ifndef DEBUG
protected:
#endif
  double fc_;
};

class Differentiator4th : public statespace_sys::StatespaceSISOSys4th,
                          public ButterworthFilter {
public:
  void InitDifferentiator(double fc, double fs);
  void SwitchBuf(double u, double y) override;

#ifndef DEBUG
protected:
#endif
  double fc_;
};

class Differentiator5th : public statespace_sys::StatespaceSISOSys5th,
                          public ButterworthFilter {
public:
  void InitDifferentiator(double fc, double fs);
  void SwitchBuf(double u, double y) override;

#ifndef DEBUG
protected:
#endif
  double fc_;
};

class Differentiator6th : public statespace_sys::StatespaceSISOSys6th,
                          public ButterworthFilter {
public:
  void InitDifferentiator(double fc, double fs);
  void SwitchBuf(double u, double y) override;

#ifndef DEBUG
protected:
#endif
  double fc_;
};

class NotchFilter : public statespace_sys::StatespaceSISOSys2nd {
public:
  void InitNotchFilter(double fn, double bw, double gain, double fs);
  void SwitchBuf(double u, double y) override;

#ifndef DEBUG
protected:
#endif
  double fn_;
  double gain_;
  double bw_;
  double Predistoration(double fc, double fs);
};

} // namespace filters
} // namespace pnc

#endif
