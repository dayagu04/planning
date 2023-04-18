#pragma once

#include "modules/common/utils/ifly_time.h"
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
  bool ego_state_update(const LocalView& local_view);
  void vehicle_status_adaptor(const VehicleService::VehicleServiceOutputInfo &vehicel_service_output_info,
                              const LocalizationOutput::LocalizationEstimate &localization_estimate,
                              common::VehicleStatus &vehicle_status);
  void truncate_prediction_info(const Prediction::PredictionResult& prediction_result,
                                double cur_timestamp_us,
                                std::unordered_set<uint>& prediction_obj_id_set);
  void transform_fusion_to_prediction(const FusionObjects::FusionObject &fusion_object, double timestamp);
  void transform_surround_radar_to_prediction(const RadarPerceptionObjects::RadarPerceptionObject radar_perception_objects);
  bool obstacle_prediction_update(const LocalView& local_view);
  PredictionTrajectoryPoint GetPointAtTime(const std::vector<PredictionTrajectoryPoint>& trajectory_points, const double relative_time) const;
  // std::shared_ptr<EnvironmentalModel> environmental_model_ = nullptr; session已包含
  planning::framework::Session *session_ = nullptr;
  planning::framework::Frame *frame_ = nullptr;
//   std::shared_ptr<PlanningResultManager> planning_result_manager_ = nullptr;
//   std::shared_ptr<EgoPoseManager> ego_pose_manager_ = nullptr;
  std::shared_ptr<planning::EgoStateManager> ego_state_manager_ptr_ = nullptr;
  std::shared_ptr<planning::ObstacleManager> obstacle_manager_ptr_ = nullptr;
  std::shared_ptr<planning::VirtualLaneManager> virtual_lane_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::ReferencePathManager> reference_path_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::TrafficLightDecisionManager> traffic_light_decision_manager_ptr_ = nullptr;
  std::shared_ptr<planning::LateralObstacle> lateral_obstacle_ptr_ = nullptr;
};

}  // namespace planner
}  // namespace planning