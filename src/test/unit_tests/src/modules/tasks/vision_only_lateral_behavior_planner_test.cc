#include "src/modules/tasks/behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"
#include "src/modules/tasks/task_pipeline_context.h"
#include "src/modules/general_planning.h"
#include "src/modules/context/virtual_lane_manager.h"
#include "src/modules/context/ego_state_manager.h"
#include "src/modules/context/frenet_ego_state.h"
#include "src/modules/context/ego_state_manager.h"
#include "src/modules/context/frenet_ego_state.h"
#include "src/modules/context/lateral_obstacle.h"
#include "src/modules/context/obstacle_manager.h"
#include "src/modules/context/parking_slot_manager.h"
#include "src/modules/context/reference_path_manager.h"
#include "src/modules/context/traffic_light_decision_manager.h"
#include "src/modules/context/parking_slot_manager.h"
#include "src/modules/context/lateral_obstacle.h"
#include "src/modules/context/ego_planning_config.h"
#include "src/modules/common/math/linear_interpolation.h"
#include "res/include/proto/vehicle_service.pb.h"
#include "src/proto/generated_files/planning_config.pb.h"
#include "src/proto/generated_files/vehicle_status.pb.h"
#include "src/common/log.h"


#include <array>
#include <cmath>

#include "gtest/gtest.h"

namespace planning {

TEST(TestLateralBehavior, vision_lateral_behavior_planner) {
  std::string log_file = "/asw/Planning/log/planning_log";
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(), bst::DEBUG);
  
  printf("TestLateralBehavior: vision_lateral_behavior_planner");
  // std::unique_ptr<GeneralPlanning> planning_base_ = nullptr;
  // planning_base_ = std::make_unique<GeneralPlanning>();
  framework::Session session;
  session.Init();
  
  EgoPlanningConfigBuilder *config_builder;
  // framework::Frame *frame;
  // frame(session);
  framework::Frame frame{&session};
  std::shared_ptr<planning::EgoStateManager> ego_state_manager_ptr_ = nullptr;
  std::shared_ptr<planning::ObstacleManager> obstacle_manager_ptr_ = nullptr;
  std::shared_ptr<planning::VirtualLaneManager> virtual_lane_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::ReferencePathManager> reference_path_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::LateralObstacle> lateral_obstacle_ptr_ = nullptr;
  ego_state_manager_ptr_ =
      std::make_shared<planning::EgoStateManager>(&session);
  (&frame)->mutable_session()->mutable_environmental_model()->set_ego_state(
      ego_state_manager_ptr_);

  virtual_lane_manager_ptr_ =
      std::make_shared<planning::VirtualLaneManager>(&session);
  (&frame)->mutable_session()->mutable_environmental_model()->set_virtual_lane_manager(
      virtual_lane_manager_ptr_);

  obstacle_manager_ptr_ =
      std::make_shared<planning::ObstacleManager>(config_builder, &session);
  (&frame)->mutable_session()->mutable_environmental_model()->set_obstacle_manager(
      obstacle_manager_ptr_);

  reference_path_manager_ptr_ =
      std::make_shared<planning::ReferencePathManager>(&session);
  (&frame)->mutable_session()->mutable_environmental_model()->set_reference_path_manager(
      reference_path_manager_ptr_);

  lateral_obstacle_ptr_ =
      std::make_shared<planning::LateralObstacle>(config_builder, &session);
  (&frame)->mutable_session()->mutable_environmental_model()->set_lateral_obstacle(
      lateral_obstacle_ptr_);

  std::shared_ptr<TaskPipelineContext> pipeline_context = nullptr;
  pipeline_context = std::make_shared<TaskPipelineContext>();

  auto vision_lateral_behavior_planner_ptr = std::make_shared<VisionLateralBehaviorPlanner>(
      config_builder, pipeline_context);

  vision_lateral_behavior_planner_ptr->Execute(&frame);


}
}  // namespace planning