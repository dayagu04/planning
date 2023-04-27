#pragma once

#include <memory>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "common/log.h"
#include "modules/common/local_view.h"
#include "modules/context/ego_planning_config.h"
#include "modules/common/prediction_object.h"
// #include "planning/common/common.h"
// #include "common/baseline_info.h"

// #include "modules/context/virtual_lane_manager.h"
// #include "modules/context/frenet_ego_state.h"
// #include "modules/context/ego_state_manager.h"
// #include "modules/context/obstacle_manager.h"
// #include "modules/context/reference_path_manager.h"
// #include "modules/context/traffic_light_decision_manager.h"
#include "modules/context/parking_slot_manager.h"

#include "../res/include/proto/vehicle_service.pb.h"
#include "proto/generated_files/planning_config.pb.h"
#include "proto/generated_files/vehicle_status.pb.h"
#include "../res/include/proto/fusion_objects.pb.h"
#include "../res/include/proto/prediction.pb.h"

#include "src/modules/common/config/vehicle_param.h"

namespace planning {

struct RefPointFrenet {
  double s;
  double lane_width = 3.2;
  double left_lane_border = 1.6;
  double right_lane_border = 1.6;
  double left_road_border = 1.6;
  double right_road_border = 1.6;
  int left_road_border_type;
  int right_road_border_type;
};

enum class VirtualObstacleId : int {
  TRAFFIC_LIGHT = -2,
  LANE_CHANGE_FULL_LINE = -3,
};

struct ObstacleRawData {
  bool has_prediction{false};
  bool has_fusion{false};
  FusionObjects::FusionObject *fusion_info;
  Prediction::PredictionObject *prediction_info;
};

class EgoPlanningConfigBuilder;
class CartEgoStateManager;
class EgoStateManager;
class ObstacleManager;
class VirtualLaneManager;
class ReferencePathManager;
class TrafficLightDecisionManager;
class LateralObstacle;


class EnvironmentalModel {
 public:
  EnvironmentalModel();
  ~EnvironmentalModel() = default;

  bool Init(common::SceneType scene_type);  // init after construction

  bool Update();  // update infos each frame

  void UpdateVehicleDbwStatus(bool flag) {
    LOG_DEBUG("feed_vehicle_dbw_status : %d \n", flag);
    vehicle_dbw_status_ = flag;
  }

  void UpdateMembers();

  bool GetVehicleDbwStatus() const { return vehicle_dbw_status_; }

  bool IsOnRoute() const { return true; }

  const planning::VehicleParam &vehicle_param() const { return vehicle_param_; }

  void set_vehicle_param(const planning::VehicleParam &vehicle_param) {
    vehicle_param_ = vehicle_param;
  }

  const std::vector<PredictionObject> &get_prediction_info() const {
    return prediction_info_;
  }
  std::vector<PredictionObject> &get_mutable_prediction_info() {
    return prediction_info_;
  }

  const std::unordered_map<Common::SensorType, std::vector<PredictionObject>> &get_surr_radar_prediction_info() const {
    return surr_radar_map_info_;
  }

  std::unordered_map<Common::SensorType, std::vector<PredictionObject>> &get_mutable_surr_radar_prediction_info() {
    return surr_radar_map_info_;
  }

  const std::shared_ptr<EgoStateManager> &get_ego_state_manager() const { return ego_state_manager_; }

  void set_ego_state(std::shared_ptr<EgoStateManager> ego_state_manager) {
    ego_state_manager_ = ego_state_manager;
  }

  const std::shared_ptr<ObstacleManager> &get_obstacle_manager() const{
    return obstacle_manager_;
  }
  std::shared_ptr<ObstacleManager> &mutable_obstacle_manager() {
    return obstacle_manager_;
  }
  void set_obstacle_manager(
      std::shared_ptr<ObstacleManager> obstacle_manager) {
    obstacle_manager_ = obstacle_manager;
  }

  const std::shared_ptr<VirtualLaneManager> &get_virtual_lane_manager() const {
    return virtual_lane_manager_;
  }
  std::shared_ptr<VirtualLaneManager> &mutable_virtual_lane_manager() {
    return virtual_lane_manager_;
  }
  void set_virtual_lane_manager(std::shared_ptr<VirtualLaneManager> virtual_lane_manager) {
    virtual_lane_manager_ = virtual_lane_manager;
  }

  const std::shared_ptr<TrafficLightDecisionManager> &get_traffic_light_decision_manager() const {
    return traffic_light_decision_manager_;
  }
  void set_traffic_light_decision_manager(
      std::shared_ptr<TrafficLightDecisionManager> traffic_light_decision_manager) {
    traffic_light_decision_manager_ = traffic_light_decision_manager;
  }

  const std::shared_ptr<ReferencePathManager> &get_reference_path_manager() const {
    return reference_path_manager_;
  }
  void set_reference_path_manager(
      std::shared_ptr<ReferencePathManager> reference_path_manager) {
    reference_path_manager_ = reference_path_manager;
  }

  const std::shared_ptr<LateralObstacle> &get_lateral_obstacle() const {
    return lateral_obstacle_;
  }
  void set_lateral_obstacle(
      std::shared_ptr<LateralObstacle> lateral_obstacle) {
    lateral_obstacle_ = lateral_obstacle;
  }

  const std::string &get_module_config_file_dir() const { return config_file_dir_; }

  void set_module_config_file_dir(const std::string &config_file_dir) {
    config_file_dir_ = config_file_dir;
  }

  EgoPlanningConfigBuilder *config_builder(
      common::SceneType scene_type) const {
    if (scene_type == common::SceneType::PARKING) {
      return parking_config_builder_ptr_;
    } else if (scene_type == common::SceneType::URBAN) {
      return urban_config_builder_ptr_;
    } else {
      return highway_config_builder_ptr_;
    }
  }

  void set_urban_config_builder(
      EgoPlanningConfigBuilder *urban_config_builder_ptr) {
    urban_config_builder_ptr_ = urban_config_builder_ptr;
  }
  void set_parking_config_builder(
      EgoPlanningConfigBuilder *parking_config_builder_ptr) {
    parking_config_builder_ptr_ = parking_config_builder_ptr;
  }
  void set_highway_config_builder(
      EgoPlanningConfigBuilder *highway_config_builder_ptr) {
    highway_config_builder_ptr_ = highway_config_builder_ptr;
  }

  void feed_local_view(const LocalView& local_view) {
    local_view_ = local_view;
  }
  const LocalView& get_local_view() const { return local_view_; }

  bool get_hdmap_valid() const { return hdmap_valid_; }
  bool is_on_highway() const { return true; } //hack
  const HimMcuInner::HmiMcuInner& get_hmi_info() const { return local_view_.hmi_mcu_inner_info; }
private:
  // planning::framework::Session *session_ = nullptr;
  // planning::framework::Frame *frame_ = nullptr;
  LocalView local_view_;
  bool vehicle_dbw_status_{false};
  std::shared_ptr<EgoStateManager> ego_state_manager_ = nullptr;
  std::shared_ptr<ObstacleManager> obstacle_manager_ = nullptr;
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_ =
      nullptr;
  std::shared_ptr<ReferencePathManager> reference_path_manager_ =
      nullptr;
  std::shared_ptr<TrafficLightDecisionManager> traffic_light_decision_manager_ = nullptr;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;

  // // set 到session里
  // std::shared_ptr<CartEgoStateManager> cart_ego_state_manager_ptr_;
  // std::shared_ptr<EgoStateManager> ego_state_manager_ptr_;
  // std::shared_ptr<ObstacleManager> obstacle_manager_ptr_;
  // std::shared_ptr<VirtualLaneManager> virtual_lane_manager_ptr_;
  // std::shared_ptr<ReferencePathManager> reference_path_manager_ptr_;
  // std::shared_ptr<TrafficLightDecisionManager> traffic_light_decision_manager_ptr_;

//  private:
//   std::unique_ptr<LateralObstacle> lateral_obstacle_;
//   std::shared_ptr<TrafficLightDecision> traffic_light_decision_;

//   common::SceneType scene_type_ = common::SceneType::HIGHWAY;
//   EgoStateManager ego_state_manager_;
//   std::vector<MSDPerceptionFusionObjectData> fusion_info_;
  std::vector<PredictionObject> prediction_info_;
  std::unordered_map<Common::SensorType, std::vector<PredictionObject>> surr_radar_map_info_;
//   std::vector<PredictionObject> truncated_prediction_info_;
  //std::vector<PredictionObject> mixed_prediction_info_;

//   TrafficLight traffic_light_info_;
//   MapInfoManager map_info_manager_;
//   std::shared_ptr<PerceptionRangeEstimator> perception_range_;

//   std::unordered_map<int, std::array<std::vector<RefPointFrenet>, 3>> map_info_frenet_;

//   // baseline info
//   std::unordered_map<int, std::shared_ptr<BaseLineInfo>> baseline_infos_;
//   FrenetCoordinateSystemParameters frenet_parameters_;

//   bool hdmap_enable_status_{false};
  bool hdmap_valid_{false};
//   bool vehicle_dbw_status_{false};
//   bool dbw_status_{false};
//   bool pretreatment_status_{true};
//   int ignore_obs_id_ = -10000;

//   // Leadone perception range
//   double leadone_perception_range_ = 0;


//   common::WheelVelocity4d wheel_speed_report_;
  // common::VehicleStatus vehicle_status_;
  planning::VehicleParam vehicle_param_;
  std::string config_file_dir_;

  EgoPlanningConfigBuilder *urban_config_builder_ptr_ = nullptr;
  EgoPlanningConfigBuilder *parking_config_builder_ptr_ = nullptr;
  EgoPlanningConfigBuilder *highway_config_builder_ptr_ = nullptr;


};

}  // namespace planning
