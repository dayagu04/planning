#pragma once

#include "autoplt/include/ADSComponent.h"
#include "autoplt/include/ADSNode.h"

#include "planning_adapter.h"

namespace planning {

using autoplt::ADSNode;

// 黑芝麻头文件中拼写错误
class PlanningComponent final : public autoplt::ADSTimerCoponent {
 public:
  PlanningComponent() = default;
  ~PlanningComponent();

  bool Init() override;
  bool Proc() override;

 private:
  void InitLogger();

 private:
  std::shared_ptr<ADSNode> planning_node_ = nullptr;
  std::shared_ptr<Writer<PlanningOutput::PlanningOutput>> planning_writer_ = nullptr;
  std::shared_ptr<Writer<planning::common::PlanningDebugInfo>> planning_debug_writer_ = nullptr;
  std::shared_ptr<Writer<PlanningHMI::PlanningHMIOutputInfoStr>> planning_hmi_info_writer_ = nullptr;

  std::unique_ptr<PlanningAdapter> planning_adapter_ = nullptr;
};

// register planning component
AUTOPLT_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning