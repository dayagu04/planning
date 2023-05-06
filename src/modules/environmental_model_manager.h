#pragma once

#include "frame.h"
#include "session.h"
#include "context/ego_planning_config.h"
#include "ifly_time.h"

namespace planning {
namespace planner {

enum FeedType {
  FEED_VEHICLE_DBW_STATUS = 0,
  FEED_EGO_VEL,
  FEED_EGO_STEER_ANGLE,
  FEED_EGO_ENU,
  FEED_WHEEL_SPEED_REPORT,
  FEED_EGO_ACC,
  FEED_MISC_REPORT,
  FEED_MAP_INFO,
  FEED_FUSION_INFO,
  FEED_SURR_RADAR_INFO,
  FEED_PREDICTION_INFO,
  FEED_FUSION_LANES_INFO,
  FEED_TYPE_MAX,
};

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
  bool ego_state_update(double current_time,  const LocalView& local_view);
  void vehicle_status_adaptor(double current_time,  const VehicleService::VehicleServiceOutputInfo &vehicel_service_output_info,
                              const LocalizationOutput::LocalizationEstimate &localization_estimate,
                              const HmiMcuInner::HmiMcuInner &hmi_mcu_inner_info,
                              common::VehicleStatus &vehicle_status);
  void truncate_prediction_info(const Prediction::PredictionResult& prediction_result,
                                double cur_timestamp_us,
                                std::unordered_set<uint>& prediction_obj_id_set);
  bool transform_fusion_to_prediction(const FusionObjects::FusionObject &fusion_object, double timestamp);
  void transform_surround_radar_to_prediction(const RadarPerceptionObjects::RadarPerceptionObject radar_perception_objects,
                                              const Common::SensorType sensor_type);
  bool obstacle_prediction_update(double current_time,  const LocalView& local_view);
  bool InputReady(double current_time, std::string &error_msg);
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
  std::shared_ptr<planning::TrafficLightDecisionManager>
      traffic_light_decision_manager_ptr_ = nullptr;
  std::shared_ptr<planning::LateralObstacle> lateral_obstacle_ptr_ = nullptr;
  double last_feed_time_[FEED_TYPE_MAX]{};
};

}  // namespace planner
}  // namespace planning