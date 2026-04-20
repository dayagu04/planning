#include "agent_trajectory_calculator.h"

#include <memory>
#include <vector>

#include "agent/agent.h"
#include "environmental_model.h"
#include "library/lc_pure_pursuit_lib/include/basic_pure_pursuit_model.h"
#include "planning_context.h"
#include "session.h"
#include "vehicle_model_simulation.h"

namespace planning {

AgentTrajectoryCalculator::AgentTrajectoryCalculator(
    framework::Session* session)
    : session_(session) {};

bool AgentTrajectoryCalculator::Process() {
  auto agent_longitudinal_decider_output =
      session_->mutable_planning_context()
          ->mutable_agent_longitudinal_decider_output();
  agent_longitudinal_decider_output->cutin_agent_ids.clear();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& lane_change_status = lane_change_decider_output.curr_state;
  bool is_in_lane_change_execution =
      lane_change_status == kLaneChangeExecution ||
      lane_change_status == kLaneChangeComplete;
  if (is_in_lane_change_execution) {
    return false;
  }

  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  auto mutable_agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  if (agent_manager == nullptr || mutable_agent_manager == nullptr) {
    return false;
  }
  const auto& agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    return false;
  }

  std::vector<int32_t> cutin_agent_ids;
  for (const auto ptr_agent : agents) {
    if (ptr_agent == nullptr) {
      continue;
    }
    const bool is_cutin_agent =
        ptr_agent->is_rule_base_cutin() || ptr_agent->is_prediction_cutin();
    const bool is_reverse_agent = ptr_agent->is_reverse();

    if (is_cutin_agent && !is_reverse_agent) {
      cutin_agent_ids.emplace_back(ptr_agent->agent_id());
      auto* mutable_agent =
          mutable_agent_manager->mutable_agent(ptr_agent->agent_id());
      if (mutable_agent == nullptr) {
        continue;
      }
      if (ptr_agent->is_rule_base_cutin() || ptr_agent->is_prediction_cutin()) {
        GeneratePurePursuitTrajectory(mutable_agent);
      }
    }
  }
  agent_longitudinal_decider_output->cutin_agent_ids =
      std::move(cutin_agent_ids);
  return true;
}

bool AgentTrajectoryCalculator::GeneratePurePursuitTrajectory(
    agent::Agent* ptr_agent) {
  if (ptr_agent == nullptr) {
    return false;
  }

  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& ego_lane = virtual_lane_manager->get_current_lane();
  if (virtual_lane_manager == nullptr || ego_lane == nullptr) {
    return false;
  }

  const auto& reference_path_ptr = ego_lane->get_reference_path();
  if (reference_path_ptr == nullptr) {
    return false;
  }

  const auto& frenet_coord = reference_path_ptr->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return false;
  }

  const auto& original_trajectories =
      ptr_agent->trajectories_used_by_st_graph();
  if (original_trajectories.empty()) {
    return false;
  }
  if (original_trajectories.front().empty()) {
    return false;
  }

  planning::BasicPurePursuitModel pp_model;
  if (pp_model.ProcessReferencePath(reference_path_ptr) !=
      ErrorType::kSuccess) {
    return false;
  }

  const double obs_wheelbase = ptr_agent->length() * 0.75;

  planning::BasicPurePursuitModel::ModelState pp_state(
      ptr_agent->x(), ptr_agent->y(), ptr_agent->theta(), ptr_agent->speed());

  trajectory::Trajectory cutin_trajectory(original_trajectories.front());

  for (size_t i = 1; i < cutin_trajectory.size(); ++i) {
    const double current_vel = pp_state.vel;

    const double ld = std::max(4.5, current_vel * 1.5);
    planning::BasicPurePursuitModel::ModelParam pp_param(ld, obs_wheelbase);

    pp_model.set_model_state(pp_state);
    pp_model.set_model_param(pp_param);

    if (pp_model.CalculateDesiredDelta(0.0) != ErrorType::kSuccess) {
      break;
    }

    double desired_delta = pp_model.get_delta();

    constexpr double kMaxSteerAngle = 0.5;
    desired_delta = std::clamp(desired_delta, -kMaxSteerAngle, kMaxSteerAngle);

    pnc::steerModel::VehicleSimulation vehicle_simulate;
    pnc::steerModel::VehicleParameter vehicle_param;
    vehicle_param.c1_ = 1.0 / obs_wheelbase;

    pnc::steerModel::VehicleState vehicle_state{pp_state.x, pp_state.y,
                                                pp_state.theta};
    pnc::steerModel::VehicleControl vehicle_control{pp_state.vel,
                                                    desired_delta};

    vehicle_simulate.Init(vehicle_state);
    vehicle_simulate.Update(vehicle_control, vehicle_param);
    const auto new_state = vehicle_simulate.GetState();

    double theta_value = new_state.phi_;
    if (i > 0) {
      double last_theta = cutin_trajectory[i - 1].theta();
      double dtheta = planning_math::NormalizeAngle(theta_value - last_theta);
      theta_value = last_theta + dtheta;
    }

    cutin_trajectory[i].set_x(new_state.x_);
    cutin_trajectory[i].set_y(new_state.y_);
    cutin_trajectory[i].set_theta(theta_value);

    pp_state.x = new_state.x_;
    pp_state.y = new_state.y_;
    pp_state.theta = theta_value;
  }

  if (cutin_trajectory.size() > 1) {
    ptr_agent->set_trajectory_cutin_postprocessed(cutin_trajectory);
    return true;
  }

  return false;
}

}  // namespace planning