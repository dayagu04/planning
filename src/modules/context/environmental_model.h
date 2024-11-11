#pragma once

#include <memory>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "ad_common/hdmap/hdmap.h"
#include "agent/agent_manager.h"
#include "config/vehicle_param.h"
#include "dynamic_world/dynamic_world.h"
#include "ego_planning_config.h"
#include "fusion_objects_c.h"
#include "groundline_decider.h"
#include "history_obstacle_manager.h"
#include "ifly_time.h"
#include "local_view.h"
#include "log.h"
#include "parking_slot_manager.h"
// #include "prediction_c.h"
#include "prediction_c.h"
#include "prediction_object.h"
#include "scene_type_config.pb.h"
// #include "vehicle_service_c.h"
#include "sdmap/sdmap.h"
#include "vehicle_service_c.h"
#include "vehicle_status.pb.h"
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
  iflyauto::FusionObject *fusion_info;
  // iflyauto::FusionObject *fusion_info;
  iflyauto::PredictionObject *prediction_info;
};

class EgoPlanningConfigBuilder;
class CartEgoStateManager;
class EgoStateManager;
class ObstacleManager;
class VirtualLaneManager;
class ReferencePathManager;
class TrafficLightDecisionManager;
class LateralObstacle;
class LaneTracksManager;
class AgentNodeManager;
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

  // const planning::VehicleParam &vehicle_param() const { return
  // vehicle_param_; }

  // void set_vehicle_param(const planning::VehicleParam &vehicle_param) {
  //   vehicle_param_ = vehicle_param;
  // }

  const std::vector<PredictionObject> &get_prediction_info() const {
    return prediction_info_;
  }
  std::vector<PredictionObject> &get_mutable_prediction_info() {
    return prediction_info_;
  }

  const std::vector<GroundLinePoint> &get_ground_line_point_info() const {
    return ground_line_point_info_;
  }
  std::vector<GroundLinePoint> &get_mutable_ground_line_point_info() {
    return ground_line_point_info_;
  }

  const std::shared_ptr<EgoStateManager> &get_ego_state_manager() const {
    return ego_state_manager_;
  }

  void set_ego_state(std::shared_ptr<EgoStateManager> ego_state_manager) {
    ego_state_manager_ = ego_state_manager;
  }

  const std::shared_ptr<ObstacleManager> &get_obstacle_manager() const {
    return obstacle_manager_;
  }
  std::shared_ptr<ObstacleManager> &mutable_obstacle_manager() {
    return obstacle_manager_;
  }
  void set_obstacle_manager(std::shared_ptr<ObstacleManager> obstacle_manager) {
    obstacle_manager_ = obstacle_manager;
  }

  const std::vector<PredictionObject> &get_fusion_info() const {
    return fusion_info_;
  }
  std::vector<PredictionObject> &get_mutable_fusion_info() {
    return fusion_info_;
  }

  const std::shared_ptr<VirtualLaneManager> &get_virtual_lane_manager() const {
    return virtual_lane_manager_;
  }
  std::shared_ptr<VirtualLaneManager> &mutable_virtual_lane_manager() {
    return virtual_lane_manager_;
  }
  void set_virtual_lane_manager(
      std::shared_ptr<VirtualLaneManager> virtual_lane_manager) {
    virtual_lane_manager_ = virtual_lane_manager;
  }

  const std::shared_ptr<TrafficLightDecisionManager>
      &get_traffic_light_decision_manager() const {
    return traffic_light_decision_manager_;
  }
  void set_traffic_light_decision_manager(
      std::shared_ptr<TrafficLightDecisionManager>
          traffic_light_decision_manager) {
    traffic_light_decision_manager_ = traffic_light_decision_manager;
  }

  const std::shared_ptr<ReferencePathManager> &get_reference_path_manager()
      const {
    return reference_path_manager_;
  }
  void set_reference_path_manager(
      std::shared_ptr<ReferencePathManager> reference_path_manager) {
    reference_path_manager_ = reference_path_manager;
  }
  std::shared_ptr<AgentNodeManager> &mutable_agent_node_manager() {
    return agent_node_manager_;
  }

  const std::shared_ptr<AgentNodeManager> &get_agent_node_manager() const {
    return agent_node_manager_;
  }
  void set_agent_node_manager(
      std::shared_ptr<AgentNodeManager> agent_node_manager) {
    agent_node_manager_ = agent_node_manager;
  }

  const std::shared_ptr<LateralObstacle> &get_lateral_obstacle() const {
    return lateral_obstacle_;
  }
  void set_lateral_obstacle(std::shared_ptr<LateralObstacle> lateral_obstacle) {
    lateral_obstacle_ = lateral_obstacle;
  }

  const std::shared_ptr<LaneTracksManager> &get_lane_tracks_manager() const {
    return lane_tracks_manager_;
  }
  void set_lane_tracks_manager(
      std::shared_ptr<LaneTracksManager> lane_tracks_manager) {
    lane_tracks_manager_ = lane_tracks_manager;
  }

  const std::shared_ptr<ParkingSlotManager> &get_parking_slot_manager() const {
    return parking_slot_manager_;
  }

  void set_parking_slot_manager(
      std::shared_ptr<ParkingSlotManager> parking_slot_manager) {
    parking_slot_manager_ = parking_slot_manager;
  }

  const std::shared_ptr<HistoryObstacleManager> &get_history_obstacle_manager()
      const {
    return history_obstacle_manager_;
  }
  void set_history_obstacle_manager(
      std::shared_ptr<HistoryObstacleManager> history_obstacle_manager) {
    history_obstacle_manager_ = history_obstacle_manager;
  }

  const std::shared_ptr<agent::AgentManager> &get_agent_manager() const {
    return agent_manager_;
  }
  std::shared_ptr<agent::AgentManager> &mutable_agent_manager() {
    return agent_manager_;
  }
  void set_agent_manager(
      const std::shared_ptr<agent::AgentManager> agent_manager) {
    agent_manager_ = agent_manager;
  }

  const std::shared_ptr<planning_data::DynamicWorld> &get_dynamic_world()
      const {
    return dynamic_world_;
  }
  void set_dynamic_world(
      const std::shared_ptr<planning_data::DynamicWorld> dynamic_world) {
    dynamic_world_ = dynamic_world;
  }

  const std::string &get_module_config_file_dir() const {
    return config_file_dir_;
  }

  void set_module_config_file_dir(const std::string &config_file_dir) {
    config_file_dir_ = config_file_dir;
  }

  EgoPlanningConfigBuilder *config_builder(common::SceneType scene_type) const {
    if (scene_type == common::SceneType::PARKING_APA) {
      return parking_config_builder_ptr_;
    } else if (scene_type == common::SceneType::HPP) {
      return hpp_config_builder_ptr_;
    } else {
      return highway_config_builder_ptr_;
    }
  }

  const EgoPlanningConfigBuilder *parking_config_builder() const {
    return parking_config_builder_ptr_;
  }

  void set_parking_config_builder(
      EgoPlanningConfigBuilder *parking_config_builder_ptr) {
    parking_config_builder_ptr_ = parking_config_builder_ptr;
  }

  const EgoPlanningConfigBuilder *highway_config_builder() const {
    return highway_config_builder_ptr_;
  }

  void set_highway_config_builder(
      EgoPlanningConfigBuilder *highway_config_builder_ptr) {
    highway_config_builder_ptr_ = highway_config_builder_ptr;
  }

  const EgoPlanningConfigBuilder *hpp_config_builder() const {
    return hpp_config_builder_ptr_;
  }

  void set_hpp_config_builder(
      EgoPlanningConfigBuilder *hpp_config_builder_ptr) {
    hpp_config_builder_ptr_ = hpp_config_builder_ptr;
  }

  void feed_local_view(const LocalView *local_view) {
    local_view_ = local_view;
  }

  void set_location_valid(bool flag) { location_valid_ = flag; }
  bool location_valid() const { return location_valid_; }
  const ad_common::hdmap::HDMap &get_hd_map() const { return hd_map_; }
  const ad_common::sdmap::SDMap &get_sd_map() const { return sd_map_; }
  const LocalView &get_local_view() const { return *local_view_; }

  bool get_hdmap_valid() const { return hdmap_valid_; }
  bool get_sdmap_valid() const { return sdmap_valid_; }
  bool is_on_highway() const { return true; }  // hack
  const iflyauto::HmiInner &get_hmi_info() const {
    return local_view_->hmi_inner_info;
  }

  // update function by system function state
  void set_function_info(
      common::DrivingFunctionInfo::DrivingFunctionMode mode,
      common::DrivingFunctionInfo::DrivingFunctionstate state) {
    function_info_.set_function_mode(mode);
    function_info_.set_function_state(state);
  }
  const common::DrivingFunctionInfo function_info() const {
    return function_info_;
  }

  void UpdateStaticMap(const LocalView &local_view);

  void UpdateSdMap(const LocalView &local_view);

 private:
  const LocalView *local_view_ = nullptr;
  uint64_t static_map_info_updated_timestamp_ = 0;
  ad_common::hdmap::HDMap hd_map_;
  uint64_t sd_map_info_updated_timestamp_ = 0;
  ad_common::sdmap::SDMap sd_map_;
  bool vehicle_dbw_status_{false};
  std::shared_ptr<EgoStateManager> ego_state_manager_ = nullptr;
  std::shared_ptr<ObstacleManager> obstacle_manager_ = nullptr;
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_ = nullptr;
  std::shared_ptr<ReferencePathManager> reference_path_manager_ = nullptr;
  std::shared_ptr<TrafficLightDecisionManager> traffic_light_decision_manager_ =
      nullptr;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;
  std::shared_ptr<LaneTracksManager> lane_tracks_manager_ = nullptr;
  std::shared_ptr<AgentNodeManager> agent_node_manager_ = nullptr;
  std::vector<PredictionObject> prediction_info_;
  std::vector<PredictionObject> fusion_info_;
  std::shared_ptr<HistoryObstacleManager> history_obstacle_manager_ = nullptr;
  std::shared_ptr<agent::AgentManager> agent_manager_ = nullptr;
  std::shared_ptr<planning_data::DynamicWorld> dynamic_world_ = nullptr;
  std::vector<GroundLinePoint> ground_line_point_info_;
  std::shared_ptr<ParkingSlotManager> parking_slot_manager_ = nullptr;
  bool hdmap_valid_{false};
  bool sdmap_valid_{false};
  bool location_valid_{true};
  // planning::VehicleParam vehicle_param_;
  std::string config_file_dir_;

  EgoPlanningConfigBuilder *parking_config_builder_ptr_ = nullptr;
  EgoPlanningConfigBuilder *highway_config_builder_ptr_ = nullptr;
  EgoPlanningConfigBuilder *hpp_config_builder_ptr_ = nullptr;
  common::DrivingFunctionInfo function_info_;
};

}  // namespace planning
