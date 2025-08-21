#include "ego_motion_preplanner.h"

#include "config/basic_type.h"
#include "dynamic_world/dynamic_world.h"
#include "environmental_model.h"
#include "lat_lon_vehicle_motion_simulator.h"
#include "log.h"
#include "planning_context.h"

namespace planning {
EgoMotionPreplanner::EgoMotionPreplanner(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session),
      config_(config_builder->cast<EgoMotionPreplannerConfig>()),
      lat_lon_vehicle_motion_simulator_() {
  name_ = "EgoMotionPreplanner";
}

bool EgoMotionPreplanner::Execute() {
  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }
  if (EgoMotionPreProcess() != ErrorType::kSuccess) {
    ILOG_ERROR << "EgoMotionPreProcess failed!!!";
    return false;
  }

  SaveToSession();

  return true;
}

ErrorType EgoMotionPreplanner::EgoMotionPreProcess() {
  const auto start_time = IflyTime::Now_ms();
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& planning_init_point = ego_state_manager->planning_init_point();
  // const auto cipv_node_id =
  //     dynamic_world->ego_front_node_id_without_prediction();
  // const auto cipv_agent_node = dynamic_world->GetNode(cipv_node_id);

  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  const auto cipv_id = cipv_info.cipv_id();
  const auto cipv_vel = cipv_info.v_frenet();
  const auto cipv_relative_s = cipv_info.relative_s();

  const auto& ref_traj = session_->planning_context()
                             .lane_change_decider_output()
                             .coarse_planning_info.trajectory_points;
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;

  const auto& ego_vehi_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  simulator::PPModelParam pp_model_param(config_.pure_pursuit_ld,
                                         ego_vehi_param.wheel_base);
  simulator::PPState pp_state(planning_init_point.x, planning_init_point.y,
                              planning_init_point.heading_angle,
                              planning_init_point.v);
  simulator::IDMModelParam idm_model_param{
      .kDesiredVelocity = ego_state_manager->ego_v_cruise(),
      .kVehicleLength = ego_vehi_param.length,
      .kMinimumSpacing = config_.idm_minimum_spacing,
      .kDesiredHeadwayTime = config_.idm_desired_time_headway,
      .kAcceleration = config_.idm_max_accel,
      .kComfortableBrakingDeceleration =
          config_.idm_comfortable_breaking_deceleration,
      .kHardBrakingDeceleration = config_.idm_hard_breaking_deceleration,
  };
  simulator::IDMState idm_state(0.0, planning_init_point.lon_init_state.v(),
                                300, 33.3);
  if (cipv_id != agent::AgentDefaultInfo::kNoAgentId) {
    idm_state.s_front = cipv_relative_s;
    idm_state.vel_front = cipv_vel;
  }
  // if (cipv_agent_node != nullptr) {
  //   idm_state.s_front =
  //       cipv_agent_node->node_back_edge_to_ego_front_edge_distance() +
  //       cipv_agent_node->node_length();
  //   idm_state.vel_front = cipv_agent_node->node_speed();
  // }

  lat_lon_vehicle_motion_simulator_.set_model_param(idm_model_param,
                                                    pp_model_param);
  lat_lon_vehicle_motion_simulator_.set_model_state(idm_state, pp_state,
                                                    planning_init_point.delta);
  lat_lon_vehicle_motion_simulator_.set_model_input(reference_path_ptr);
  lat_lon_vehicle_motion_simulator_.set_dt_resolution(
      config_.ego_motion_simulate_dt_resolution);
  // const double start_time_ref_sim = IflyTime::Now_ms();
  if (lat_lon_vehicle_motion_simulator_.Simulate(
          config_.trajectory_time_length) != ErrorType::kSuccess) {
    ILOG_ERROR << "LatLonVehicleMotionSimulator::Simulate failed";
    return ErrorType::kWrongStatus;
  };
  // const double end_time_ref_sim = IflyTime::Now_ms();
  // JSON_DEBUG_VALUE("sim_ref_time_cost", end_time_ref_sim -
  // start_time_ref_sim);
  const auto& ref_simulation_result =
      lat_lon_vehicle_motion_simulator_.get_simulation_result();
  JSON_DEBUG_VECTOR("ego_ref_sim_x_vec", ref_simulation_result->x_vec, 3)
  JSON_DEBUG_VECTOR("ego_ref_sim_y_vec", ref_simulation_result->y_vec, 3)
  JSON_DEBUG_VECTOR("ld_actual_length_vec",
                    ref_simulation_result->ld_actual_length_vec, 3)
  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("sim_ref_time_cost", end_time - start_time);
  ILOG_DEBUG << "sim_ref_time_cost:" << end_time - start_time;
  return ErrorType::kSuccess;
}

void EgoMotionPreplanner::SaveToSession() {
  auto& ego_motion_preplanner_output =
      session_->mutable_planning_context()
          ->mutable_ego_motion_preplanner_output();
  ego_motion_preplanner_output.mutable_ego_motion_simulation_result() =
      lat_lon_vehicle_motion_simulator_.get_simulation_result();
}
}  // namespace planning
