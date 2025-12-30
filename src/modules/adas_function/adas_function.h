#pragma once

#include "adas_function_context.h"
#include "adas_function_lib.h"
#include "adas_function_preprocess.h"
#include "adas_function_struct.h"
#include "base_function.h"
#include "base_task_pipeline.h"
#include "ihc_function/ihc_core.h"
#include "lkas_function/elk_core.h"
#include "lkas_function/ldp_core.h"
#include "lkas_function/ldw_core.h"
#include "meb_function/meb_core.h"
#include "session.h"
#include "tsr_function/tsr_core.h"

namespace planning {

class AdasFunction : public BaseFunction {
 public:
  AdasFunction(framework::Session *session);

  virtual ~AdasFunction() = default;

  bool Reset() override;

  bool Plan() override;

 private:
  void Init(void);

  void SetLkaTrajectory();

  void Log(void);

  void StoreInfoForNextCycle(void);

  void TestLkasForHmi(void);

  // Preprocess
  std::shared_ptr<adas_function::preprocess::Preprocess> preprocess_ptr_;

  // LdwCore
  std::shared_ptr<adas_function::ldw_core::LdwCore> ldw_core_ptr_;
  // LdpCore
  std::shared_ptr<adas_function::ldp_core::LdpCore> ldp_core_ptr_;
  // ElkCore
  std::shared_ptr<adas_function::elk_core::ElkCore> elk_core_ptr_;
  // TsrCore
  std::shared_ptr<adas_function::tsr_core::TsrCore> tsr_core_ptr_;
  // IhcCore
  std::shared_ptr<adas_function::ihc_core::IhcCore> ihc_core_ptr_;

  void LdpDriverhandsoffWarning();
  bool lkas_intervention = false;
  int ldp_intervention_count = 0;
  double ldp_intervention_count_2_dur = 0.0;
  double ldp_intervention_count_3_dur = 0.0;
  double ldp_intervention_count_4_dur = 0.0;
  double ldp_intervention_count_5_dur = 0.0;
  double ldp_intervention_duration_ = 0.0;
  double ldp_handsoff_duration_ = 0.0;  // 驾驶员手离方向盘计时器，单位:s
  bool ldp_driver_handsoff_flag = false;  // 驾驶员手离方向盘标志位
  iflyauto::LDPDriverhandsoffWarning ldp_driver_handsoff_warning_ =
      iflyauto::LDPDriverhandsoffWarning::LDP_DRIVER_HANDSOFF_WARNING_OFF;
  bool ldp_warning_audio_flag_ = false;
  bool lkas_intervention_rising_edge_ = false;
  iflyauto::LDPFunctionFSMWorkState ldp_state_ = iflyauto::
      LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
  // MebCore
  std::shared_ptr<adas_function::meb_core::MebCore> meb_core_ptr_;
};

}  // namespace planning