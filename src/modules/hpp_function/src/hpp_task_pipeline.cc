#include "hpp_task_pipeline.h"

namespace planning {

HppTaskPipeline::HppTaskPipeline(const EgoPlanningConfigBuilder *config_builder,
                                 framework::Session *session)
    : BaseTaskPipeline(config_builder, session) {
  lane_change_decider_ =
      std::make_unique<LaneChangeDecider>(config_builder, session);
  lateral_obstacle_decider_ =
      std::make_unique<LateralObstacleDecider>(config_builder, session);
  hpp_general_lateral_decider_ =
      std::make_unique<HppGeneralLateralDecider>(config_builder, session);
  lateral_motion_planner_ =
      std::make_unique<LateralMotionPlanner>(config_builder, session);
  general_longitudinal_decider =
      std::make_unique<GeneralLongitudinalDecider>(config_builder, session);
  longitudinal_motion_planner_ =
      std::make_unique<LongitudinalMotionPlanner>(config_builder, session);
  steering_wheel_stationary_decider_ =
      std::make_unique<SteeringWheelStationaryDecider>(config_builder, session);
  result_trajectory_generator_ =
      std::make_unique<ResultTrajectoryGenerator>(config_builder, session);
  parking_switch_decider_ =
      std::make_unique<ParkingSwitchDecider>(config_builder, session);
}

bool HppTaskPipeline::Run() {
  auto time1 = IflyTime::Now_ms();
  bool ok = lane_change_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lane_change_decider_->Name());
    return false;
  }
  auto time2 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LaneChangeDeciderTime", time2 - time1);

  ok = lateral_obstacle_decider_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_obstacle_decider_->Name());
    return false;
  }
  auto time3 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LateralObstacleDeciderTime", time3 - time2);

  ok = hpp_general_lateral_decider_->Execute();
  if (!ok) {
    AddErrorInfo(hpp_general_lateral_decider_->Name());
    return false;
  }
  auto time4 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("HppGeneralLateralDeciderTime", time4 - time3);

  ok = lateral_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(lateral_motion_planner_->Name());
    return false;
  }
  auto time5 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LateralMotionPlannerTime", time5 - time4);

  ok = general_longitudinal_decider->Execute();
  if (!ok) {
    AddErrorInfo(general_longitudinal_decider->Name());
    return false;
  }
  auto time6 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("GeneralLongitudinalDeciderTime", time6 - time5);

  ok = longitudinal_motion_planner_->Execute();
  if (!ok) {
    AddErrorInfo(longitudinal_motion_planner_->Name());
    return false;
  }
  auto time7 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LongitudinalMotionPlannerTime", time7 - time6);

  ok = steering_wheel_stationary_decider_->Execute();
  if (!ok) {
    AddErrorInfo(steering_wheel_stationary_decider_->Name());
    return false;
  }
  auto time8 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("SteeringWheelStationaryDeciderTime", time8 - time7);

  ok = result_trajectory_generator_->Execute();
  if (!ok) {
    AddErrorInfo(result_trajectory_generator_->Name());
    return false;
  }
  auto time9 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("ResultTrajectoryGeneratorTime", time9 - time8);

  ok = parking_switch_decider_->Execute();
  if (!ok) {
    AddErrorInfo(parking_switch_decider_->Name());
    return false;
  }
  auto time10 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("ParkingSwitchDeciderTime", time10 - time9);

  return true;
}

}  // namespace planning