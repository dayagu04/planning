/***************************************************************
 * @file       system_identification.cpp
 * @brief      for system_identification
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-17-2021
 **************************************************************/

#include "system_identification.h"
#include <cmath>

namespace pnc {
namespace identification {
void FrequencySweepInput::SetParam(double T_stop, double f_start, double f_stop, double amp) {
  T_stop_ = T_stop;
  f_start_ = f_start;
  f_stop_ = f_stop;
  amp_ = amp;

  Reset();
}

void FrequencySweepInput::Tick(double time_now) {  // time_now is by sec
  if (!init_flag_) {
    // init
    t_ = 0.0;
    dt_ = 0.0;
    theta_ = 0.0;
    u_ = 0.0;
    init_flag_ = true;
  } else if (t_ < T_stop_) {
    // calculate dt_
    dt_ = time_now - time_now_;

    // calculate omega
    double K = 0.0187 * (std::expm1(4.0 * t_ / T_stop_) - 1.0);
    double omega = 2.0 * M_PI * (f_start_ + K * (f_stop_ - f_start_));

    // calculate theta and u
    theta_ += omega * dt_;
    u_ = amp_ * std::sin(theta_);

    // update t_
    t_ += dt_;
  } else {
    u_ = 0.0;
    stop_flag_ = true;
  }

  time_now_ = time_now;
}

void FrequencySweepInput::Reset() {
  time_now_ = 0.0;
  t_ = 0.0;
  dt_ = 0.0;
  theta_ = 0.0;
  u_ = 0.0;
  init_flag_ = false;
  stop_flag_ = false;
}

void FrequencySweepInput::SwitchBuf(double time_now) {
  time_now_ = time_now;
  theta_ = 0.0;
}

}  // namespace identification
}  // namespace pnc
