#include <array>
#include <cmath>

#include "behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "frenet_ego_state.h"
#include "gtest/gtest.h"
#include "lateral_obstacle.h"
#include "log.h"
#include "math/linear_interpolation.h"
#include "obstacle_manager.h"
#include "parking_slot_manager.h"
#include "planning_scheduler.h"
#include "reference_path_manager.h"
#include "scene_type_config.pb.h"
#include "tasks/behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"
#include "traffic_light_decision_manager.h"
#include "vehicle_service_c.h"
#include "vehicle_status.pb.h"
#include "virtual_lane_manager.h"

namespace planning {

TEST(TestLateralBehavior, vision_lateral_behavior_planner) {
  std::string log_file = "/asw/Planning/log/planning_log";
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(),
                                    bst::DEBUG);

  printf("TestLateralBehavior: vision_lateral_behavior_planner");
  // std::unique_ptr<PlanningScheduler> planning_scheduler = nullptr;
  // planning_scheduler = std::make_unique<PlanningScheduler>();
  framework::Session session;
  session.Init();

  EgoPlanningConfigBuilder *config_builder;

  std::shared_ptr<planning::EgoStateManager> ego_state_manager_ptr_ = nullptr;
  std::shared_ptr<planning::ObstacleManager> obstacle_manager_ptr_ = nullptr;
  std::shared_ptr<planning::VirtualLaneManager> virtual_lane_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::ReferencePathManager> reference_path_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::LateralObstacle> lateral_obstacle_ptr_ = nullptr;
  ego_state_manager_ptr_ =
      std::make_shared<planning::EgoStateManager>(&session);
  session.mutable_environmental_model()->set_ego_state(ego_state_manager_ptr_);

  virtual_lane_manager_ptr_ =
      std::make_shared<planning::VirtualLaneManager>(&session);
  session.mutable_environmental_model()->set_virtual_lane_manager(
      virtual_lane_manager_ptr_);

  obstacle_manager_ptr_ =
      std::make_shared<planning::ObstacleManager>(config_builder, &session);
  session.mutable_environmental_model()->set_obstacle_manager(
      obstacle_manager_ptr_);

  reference_path_manager_ptr_ =
      std::make_shared<planning::ReferencePathManager>(&session);
  session.mutable_environmental_model()->set_reference_path_manager(
      reference_path_manager_ptr_);

  lateral_obstacle_ptr_ =
      std::make_shared<planning::LateralObstacle>(config_builder, &session);
  session.mutable_environmental_model()->set_lateral_obstacle(
      lateral_obstacle_ptr_);

  auto vision_lateral_behavior_planner_ptr =
      std::make_shared<VisionLateralBehaviorPlanner>(config_builder);

  vision_lateral_behavior_planner_ptr->Execute(&session);
}
}  // namespace planning