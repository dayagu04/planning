#pragma once

#include <memory>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "ad_common/hdmap/hdmap.h"
#include "config/vehicle_param.h"
#include "ego_planning_config.h"
#include "fusion_objects.pb.h"
#include "ifly_time.h"
#include "local_view.h"
#include "log.h"
#include "parking_slot_manager.h"
#include "prediction.pb.h"
#include "prediction_object.h"
#include "scene_type_config.pb.h"
#include "vehicle_service.pb.h"
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
class LaneTracksManager;

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

  const std::string &get_module_config_file_dir() const {
    return config_file_dir_;
  }

  void set_module_config_file_dir(const std::string &config_file_dir) {
    config_file_dir_ = config_file_dir;
  }

  EgoPlanningConfigBuilder *config_builder(common::SceneType scene_type) const {
    if (scene_type == common::SceneType::PARKING_APA) {
      return parking_config_builder_ptr_;
    } else {
      return highway_config_builder_ptr_;
    }
  }

  void set_parking_config_builder(
      EgoPlanningConfigBuilder *parking_config_builder_ptr) {
    parking_config_builder_ptr_ = parking_config_builder_ptr;
  }
  void set_highway_config_builder(
      EgoPlanningConfigBuilder *highway_config_builder_ptr) {
    highway_config_builder_ptr_ = highway_config_builder_ptr;
  }

  void feed_local_view(const LocalView &local_view) {
    if (local_view.static_map_info.header().timestamp() !=
        local_view_.static_map_info.header().timestamp()) {
      ad_common::hdmap::HDMap hd_map_tmp;
      const int res =
          hd_map_tmp.LoadMapFromProto(local_view.static_map_info.road_map());
      if (res == 0) {
        hd_map_ = std::move(hd_map_tmp);
        hdmap_valid_ = true;
      }
    }
    local_view_ = local_view;
  }
  void set_location_valid(bool flag) { location_valid_ = flag; }
  bool location_valid() const { return location_valid_; }
  const LocalView &get_local_view() const { return local_view_; }
  const ad_common::hdmap::HDMap &get_hd_map() const { return hd_map_; }

  bool get_hdmap_valid() const { return hdmap_valid_; }
  bool is_on_highway() const { return true; }  // hack
  const HmiMcuInner::HmiMcuInner &get_hmi_info() const {
    return local_view_.hmi_mcu_inner_info;
  }

 private:
  // planning::framework::Session *session_ = nullptr;
  // planning::framework::Frame *frame_ = nullptr;
  LocalView local_view_;
  ad_common::hdmap::HDMap hd_map_;
  bool vehicle_dbw_status_{false};
  std::shared_ptr<EgoStateManager> ego_state_manager_ = nullptr;
  std::shared_ptr<ObstacleManager> obstacle_manager_ = nullptr;
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_ = nullptr;
  std::shared_ptr<ReferencePathManager> reference_path_manager_ = nullptr;
  std::shared_ptr<TrafficLightDecisionManager> traffic_light_decision_manager_ =
      nullptr;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;
  std::shared_ptr<LaneTracksManager> lane_tracks_manager_ = nullptr;
  std::vector<PredictionObject> prediction_info_;
  bool hdmap_valid_{false};
  bool location_valid_{true};
  planning::VehicleParam vehicle_param_;
  std::string config_file_dir_;

  EgoPlanningConfigBuilder *parking_config_builder_ptr_ = nullptr;
  EgoPlanningConfigBuilder *highway_config_builder_ptr_ = nullptr;
};

}  // namespace planning
