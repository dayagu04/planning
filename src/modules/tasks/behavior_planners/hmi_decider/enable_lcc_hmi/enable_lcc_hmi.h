#pragma once
#include "planning_context.h"
#include "session.h"

namespace planning {
class EnableLCCHMIDecider {
 public:
  EnableLCCHMIDecider(framework::Session* session,
                      const HmiDeciderConfig& config);
  ~EnableLCCHMIDecider() = default;

  bool Execute();

 private:
  void GenerateHmiOutput();
  void Reset();

  framework::Session* session_;
  HmiDeciderConfig config_;
  std::shared_ptr<ReferencePath> reference_path_ptr_;

  bool last_enable_lcc_ = false;
};
}  // namespace planning