#pragma once

#include <memory>
#include "agent/agent_manager.h"
#include "agent_node_manager.h"
#include "dynamic_world/dynamic_world.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "history_obstacle_manager.h"
#include "ifly_time.h"
#include "local_view.h"
#include "obstacle_manager.h"
#include "parking_slot_manager.h"
#include "prediction_object.h"
#include "route_info.h"
#include "session.h"
#include "vehicle_status.pb.h"

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
  FEED_PREDICTION_INFO,
  FEED_FUSION_LANES_INFO,
  FEED_TYPE_MAX,
};

enum TurnSwitchState {
  NONE = 0,
  LEFT_FIRMLY_TOUCH = 1,
  RIGHT_FIRMLY_TOUCH = 2,
  LEFT_LIGHTLY_TOUCH = 3,
  RIGHT_LIGHTLY_TOUCH = 4,
  ERROR = 5,
};

class EnvironmentalModelManager {
 public:
  EnvironmentalModelManager();
  bool Run();
  void Init(planning::framework::Session *session);
  void InitContext();
  ~EnvironmentalModelManager() = default;
  void setFaultcode(uint64_t faultcode);
  uint64_t getFaultcode();
  void SetConfig(const planning::common::SceneType scene_type);

  //  private:
  //   void SetPlanningPesult(const PlanningResult &ego_prediction_result,
  //                            common::PlanningResult &pnc_result);
  //   void ClearPlanningResult(common::PlanningResult &pnc_result);

 private:
  bool ego_state_update(double current_time, const LocalView &local_view);
  void vehicle_status_adaptor(double current_time, const LocalView &local_view,
                              common::VehicleStatus &vehicle_status);
  void truncate_prediction_info(
      const iflyauto::PredictionResult &prediction_result,
      double cur_timestamp_us, std::unordered_set<uint> &prediction_obj_id_set);
  bool transform_fusion_to_prediction(
      const iflyauto::FusionObject &fusion_object, double timestamp,
      std::vector<PredictionObject> &objects_infos);
  bool transform_fusion_to_prediction_longtime(
      const iflyauto::FusionObject &fusion_object, double timestamp,
      std::vector<PredictionObject> &objects_infos);
  bool obstacle_prediction_update(double current_time,
                                  const LocalView &local_view);

  bool InputReady(double current_time, std::string &error_msg);
  PredictionTrajectoryPoint GetPointAtTime(
      const std::vector<PredictionTrajectoryPoint> &trajectory_points,
      const double relative_time) const;

  EgoPlanningConfigBuilder *load_config_builder(const char *file_name);
  void RunBlinkState(
      const iflyauto::VehicleServiceOutputInfo &vehicle_service_output_info);
  bool CheckIfOversizeVehicle(const int type);
  bool CheckIfVru(const int type);
  bool CheckIfTrafficFacilities(const int type);
  bool CheckIfCar(const int type);
  bool IsStatic(const PredictionObject &prediction_object);

 private:
  planning::framework::Session *session_ = nullptr;
  //   std::shared_ptr<PlanningResultManager> planning_result_manager_ =
  //   nullptr; std::shared_ptr<EgoPoseManager> ego_pose_manager_ = nullptr;
  std::shared_ptr<planning::EgoStateManager> ego_state_manager_ptr_ = nullptr;
  std::shared_ptr<planning::ObstacleManager> obstacle_manager_ptr_ = nullptr;
  std::shared_ptr<planning::ParkingSlotManager> parking_slot_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::VirtualLaneManager> virtual_lane_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::ReferencePathManager> reference_path_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::TrafficLightDecisionManager>
      traffic_light_decision_manager_ptr_ = nullptr;
  std::shared_ptr<planning::LateralObstacle> lateral_obstacle_ptr_ = nullptr;
  std::shared_ptr<planning::LaneTracksManager> lane_tracks_mgr_ptr_ = nullptr;
  std::shared_ptr<planning::AgentNodeManager> agent_node_mgr_ptr_ = nullptr;
  std::shared_ptr<planning::HistoryObstacleManager> history_obstacle_ptr_ =
      nullptr;
  std::shared_ptr<planning::agent::AgentManager> agent_manager_ptr_ = nullptr;
  std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world_ =
      nullptr;
  std::shared_ptr<planning::RouteInfo> route_info_ptr_ = nullptr;
  double last_feed_time_[FEED_TYPE_MAX]{};
  EgoPlanningConfig ego_config_;
  int current_turn_signal_ = 0;
  int last_frame_turn_sinagl_ = 0;
  std::vector<int> history_lc_source_ = {0, 0};  // 0表示none，1表示ilc.
  uint64_t faultcode_ = 39999;
};

}  // namespace planner
}  // namespace planning