#include <memory>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "common/log.h"
// #include "planning/common/common.h"
// #include "common/baseline_info.h"

#include "environmental_model_manager.h"
#include "modules/context/virtual_lane_manager.h"
#include "modules/context/cart_ego_state.h"
#include "modules/context/frenet_ego_state.h"
#include "modules/context/ego_state_manager.h"
#include "modules/context/obstacle_manager.h"
#include "modules/context/reference_path_manager.h"
#include "modules/context/traffic_light_decision_manager.h"
#include "modules/context/parking_slot_manager.h"
#include "modules/context/ego_planning_config.h"

#include "../res/include/proto/bsw_proto_vehicle_service.pb.h"
#include "proto/generated_files/planning_config.pb.h"
#include "proto/generated_files/vehicle_status.pb.h"

namespace planning {
namespace planner {

EnvironmentalModelManager::EnvironmentalModelManager() { LOG_DEBUG("EnvironmentalModelManager created"); }

void EnvironmentalModelManager::Init(planning::framework::Session *session) {
  session_ = session;
  InitContext();
}

void EnvironmentalModelManager::InitContext() {
  planning::common::SceneType scene_type = session_->get_scene_type();
  auto config_builder =
      session_->environmental_model().config_builder(scene_type);
  auto config = config_builder->cast<planning::EgoPlanningConfig>();

  ego_state_manager_ptr_ = std::make_shared<planning::EgoStateManager>(session_);
  session_->mutable_environmental_model()->set_ego_state(ego_state_manager_ptr_);

  virtual_lane_manager_ptr_ = std::make_shared<planning::VirtualLaneManager>(session_);
  session_->mutable_environmental_model()->set_virtual_lane_manager(
      virtual_lane_manager_ptr_);

  obstacle_manager_ptr_ =
      std::make_shared<planning::ObstacleManager>(config_builder, session_);
  session_->mutable_environmental_model()->set_obstacle_manager(
      obstacle_manager_ptr_);

  traffic_light_decision_manager_ptr_ = std::make_shared<planning::TrafficLightDecisionManager>(
      config_builder, session_, virtual_lane_manager_);
  session_->mutable_environmental_model()->set_traffic_light_decision_manager(
      traffic_light_decision_manager_ptr_);

  reference_path_manager_ptr_ = std::make_shared<planning::ReferencePathManager>(session_);
  session_->mutable_environmental_model()->set_reference_path_manager(
      reference_path_manager_ptr_);

}

bool EnvironmentalModelManager::Run(planning::framework::Frame *frame) {

  frame_ = frame;

  auto start_time = IflyTime::Now_ms();

  // Step 2) update planning world
  ego_state_manager_ptr_->update();
  if (!virtual_lane_manager_ptr_->update()) {
    session_->mutable_planning_context()->mutable_planning_success() =
        false;
    LOG_ERROR("virtual_lane_manager update failed");
    return false;
  }
  obstacle_manager_ptr_->update();

  reference_path_manager_ptr_->update();
  // HACK (@Haowen) this operation depends on obstacle_manager_->update() and
  // reference_path_manager_->update()
  obstacle_manager_ptr_->assign_obstacles_to_lanes();

  traffic_light_decision_manager_ptr_->update();

  auto end_time = IflyTime::Now_ms();
  LOG_DEBUG("update time:%f", end_time - start_time);

  return true;
}

}  // namespace planner
}  // namespace planning
