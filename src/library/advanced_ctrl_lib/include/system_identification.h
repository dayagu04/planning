/***************************************************************
 * @file       system_identification.h
 * @brief      for system_identification
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-17-2021
 **************************************************************/
#ifndef __FREQUENCY_SWEEP_INPUT_H__
#define __FREQUENCY_SWEEP_INPUT_H__

namespace pnc {
namespace identification {
class FrequencySweepInput {
public:
  FrequencySweepInput(){};
  void SetParam(double T_stop, double f_start, double f_stop, double amp);
  void Tick(double time_now);
  void Reset();
  void SwitchBuf(double time_now);
  double GetSweepSignal() { return u_; }
  bool GetStopFlag() { return stop_flag_; };

private:
  /* parameters */
  double T_stop_;
  double f_start_;
  double f_stop_;
  double amp_;

  /* states */
  double time_now_;
  double t_;
  double dt_;
  double theta_;
  double u_;
  bool init_flag_;
  bool stop_flag_;
};

} // namespace identification
} // namespace pnc
#endif
