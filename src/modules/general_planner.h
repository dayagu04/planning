#pragma once

#include <cmath>
#include <vector>

#include "framework/frame.h"
#include "framework/session.h"
#include "modules/context/ego_planning_config.h"
#include "src/common/ifly_time.h"

namespace planning {
namespace planner {
class GeneralPlanner {
 public:
  GeneralPlanner();
  bool Run(planning::framework::Frame *frame);
  void Init(planning::framework::Session *session);
  void InitContext();
  ~GeneralPlanner() = default;

 private:
  void SetPlanningResult(const PlanningResult &ego_prediction_result,
                         common::PlanningResult &pnc_result);
  void ClearPlanningResult(common::PlanningResult &pnc_result);

 private:
  // std::shared_ptr<EnvironmentalModel> environmental_model_ = nullptr;
  // session已包含
  planning::framework::Session *session_ = nullptr;
  planning::framework::Frame *frame_ = nullptr;
  //   std::shared_ptr<PlanningResultManager> planning_result_manager_ =
  //   nullptr; std::shared_ptr<EgoPoseManager> ego_pose_manager_ = nullptr;
  std::shared_ptr<ObjectSelector> object_selector_ = nullptr;
  std::shared_ptr<ScenarioStateMachine> scenario_state_machine_ = nullptr;
  std::shared_ptr<AdaptiveCruiseControl> adaptive_cruise_control_ = nullptr;
  std::shared_ptr<StartStopEnable> start_stop_ = nullptr;
  std::shared_ptr<MrcCondition> mrc_condition_ = nullptr;
};

}  // namespace planner
}  // namespace planning