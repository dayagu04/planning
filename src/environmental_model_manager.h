#pragma once

#include "framework/frame.h"
#include "framework/session.h"
#include "modules/context/ego_planning_config.h"

namespace planning {
namespace planner {
class EnvironmentalModelManager {
  public:
  EnvironmentalModelManager();
  bool Run(planning::framework::Frame *frame);
  void Init(planning::framework::Session *session);
  void InitContext();
  ~EnvironmentalModelManager() = default;

//  private:
//   void SetPlanningPesult(const PlanningResult &ego_prediction_result,
//                            common::PlanningResult &pnc_result);
//   void ClearPlanningResult(common::PlanningResult &pnc_result);

 private:
  // std::shared_ptr<EnvironmentalModel> environmental_model_ = nullptr; session已包含
  planning::framework::Session *session_ = nullptr;
  planning::framework::Frame *frame_ = nullptr;
//   std::shared_ptr<PlanningResultManager> planning_result_manager_ = nullptr;
//   std::shared_ptr<EgoPoseManager> ego_pose_manager_ = nullptr;
  std::shared_ptr<planning::CartEgoStateManager> cart_ego_state_manager_ptr_ = nullptr;
  std::shared_ptr<planning::EgoStateManager> ego_state_manager_ptr_ = nullptr;
  std::shared_ptr<planning::ObstacleManager> obstacle_manager_ptr_ = nullptr;
  std::shared_ptr<planning::VirtualLaneManager> virtual_lane_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::ReferencePathManager> reference_path_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::TrafficLightDecisionManager> traffic_light_decision_manager_ptr_ = nullptr;
};

}  // namespace planner
}  // namespace planning