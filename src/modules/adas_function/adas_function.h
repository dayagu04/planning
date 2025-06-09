#pragma once

#include "adas_function_context.h"
#include "adas_function_lib.h"
#include "adas_function_preprocess.h"
#include "adas_function_struct.h"
#include "base_function.h"
#include "base_task_pipeline.h"
#include "ihc_function/intelligent_headlight_control.h"
#include "lkas_function/ldp_core.h"
#include "lkas_function/ldw_core.h"
#include "lkas_function/elk_core.h"
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
};

}  // namespace planning