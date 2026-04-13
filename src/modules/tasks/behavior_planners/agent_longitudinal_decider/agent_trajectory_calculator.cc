#include "agent_trajectory_calculator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "agent/agent.h"
#include "environmental_model.h"
#include "library/lc_pure_pursuit_lib/include/basic_pure_pursuit_model.h"
#include "planning_context.h"
#include "session.h"
#include "utils/kd_path.h"
#include "vehicle_model_simulation.h"

namespace planning {

namespace {
constexpr double kPlanningTs = 4.0;
constexpr double kHalf = 0.5;
constexpr double kExtraAdditionDistance = 10.0;
constexpr double kFakePredictionTrajectoryExtraLength = 3.0;
constexpr double kFilterWidthBuffer = 3.75;
constexpr double kMinSafeDistance = 5.0;
constexpr double kLargeCurvRadius = 2000.0;
constexpr double kDistanceCurvature = 30.0;
constexpr double kTimeCurvature = 2.0;
constexpr double kLargeAgentLength = 8.0;
constexpr double kPlanningTimeStep = 0.2;
constexpr int kNumPoints = 26;
}  // namespace

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

bool AgentTrajectoryCalculator::CalculateCutinAgentTrajectory(
    const bool is_in_lane_change_execution,
    const std::shared_ptr<agent::Agent>& ptr_agent) {
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& ego_lane = virtual_lane_manager->get_current_lane();
  const auto& mutable_agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  const auto& ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  const auto& init_point = ego_state_mgr->planning_init_point();
  if (virtual_lane_manager == nullptr || mutable_agent_manager == nullptr ||
      ego_lane == nullptr) {
    return false;
  }

  const auto& current_lane_coord = ego_lane->get_lane_frenet_coord();
  if (current_lane_coord == nullptr) {
    return false;
  }
  double agent_s = 0.0;
  double agent_l = 0.0;
  if (!current_lane_coord->XYToSL(ptr_agent->x(), ptr_agent->y(), &agent_s,
                                  &agent_l)) {
    return false;
  }
  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!current_lane_coord->XYToSL(init_point.x, init_point.y, &ego_s, &ego_l)) {
    return false;
  }
  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double front_edge_to_rear_axle = ego_vehicle_param.front_edge_to_rear_axle;
  const double half_agent_length = ptr_agent->length() * 0.5;
  const double longitudinal_distance =
      agent_s - ego_s - front_edge_to_rear_axle - half_agent_length;
  if (longitudinal_distance < 0.0) {
    return false;
  }

  // 1. if is near ego lane, do not cal new traj
  // if (std::fabs(agent_l) < ptr_agent->width() * 0.5) {
  //   return false;
  // }

  bool is_left = agent_l < 0.0;
  std::vector<QuinticPolylinePointInfo> quintic_poly_path;
  quintic_poly_path.reserve(30);
  // make target l with origin traj
  double target_l = 0.0;
  if (!MakeTargetLWithOriginTrajectory(ego_lane, current_lane_coord, ptr_agent,
                                       is_left, &target_l)) {
    return false;
  }
  // make poly path
  bool need_extend_st_boundary_backward = false;
  GenerateMaxJerkQuinticPolyPath(
      is_in_lane_change_execution, current_lane_coord, ptr_agent, is_left,
      target_l, quintic_poly_path, &need_extend_st_boundary_backward);
  if (quintic_poly_path.empty()) {
    return false;
  }
  // make trajectory
  const bool is_success = GenarateAgentTrajectoryWithPolyPath(
      is_in_lane_change_execution, ptr_agent, quintic_poly_path);
  if (!is_success) {
    return false;
  }

  auto* mutable_agent_decision =
      mutable_agent_manager->mutable_agent(ptr_agent->agent_id());
  if (mutable_agent_decision == nullptr) {
    return false;
  }
  if (need_extend_st_boundary_backward) {
    mutable_agent_decision->set_need_backward_extend(true);
  }

  return false;
}

bool AgentTrajectoryCalculator::MakeTargetLWithOriginTrajectory(
    const std::shared_ptr<VirtualLane> target_lane,
    const std::shared_ptr<planning_math::KDPath>& current_lane_coord,
    const std::shared_ptr<agent::Agent>& ptr_agent, bool is_left,
    double* const target_l) const {
  // 1. if agent is static,do not make targer L
  constexpr double kLowSpeedIgnreAgentSpeedThrMps = 0.8;
  constexpr double kStaticAgentSpeedThrMps = 0.5;
  const double agent_vel_average = CalculateAgentAverageSpeed(ptr_agent);
  const double agent_init_vel =
      std::fmax(ptr_agent->speed(), agent_vel_average);
  bool is_low_speed = agent_init_vel < kStaticAgentSpeedThrMps ||
                      ptr_agent->trajectories_used_by_st_graph().empty();
  bool is_ignore_traj = ptr_agent->speed() < kLowSpeedIgnreAgentSpeedThrMps &&
                        ptr_agent->has_low_spd_unstable_trajectory();
  if (is_low_speed) {
    return false;
  }
  const auto agent_center = ptr_agent->box().center();
  double center_s = 0.0;
  double center_l = 0.0;
  if (!current_lane_coord->XYToSL(agent_center.x(), agent_center.y(), &center_s,
                                  &center_l)) {
    return false;
  }
  double half_width = 0.5 * target_lane->width_by_s(center_s);
  // if (std::fabs(center_l) > matched_width + kFilterWidthBuffer) {
  //   return false;
  // }

  // 2. if agent is reverse,do not consider make target L
  if (ptr_agent->is_reverse()) {
    return false;
  }
  *target_l = 0.0;
  if (ptr_agent->trajectories_used_by_st_graph().empty()) {
    return false;
  }
  if (ptr_agent->trajectories_used_by_st_graph().front().empty()) {
    return false;
  }
  auto trajectory = ptr_agent->trajectories_used_by_st_graph().front();
  double traj_back_s = 0.0;
  double traj_back_l = 0.0;
  if (!current_lane_coord->XYToSL(trajectory.back().x(), trajectory.back().y(),
                                  &traj_back_s, &traj_back_l)) {
    return false;
  }
  constexpr double kNearCenterLaneThreshold = 0.1;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_half_width = vehicle_param.width * kHalf;
  // 3. if origin traj back point near center lane,use center point L 0.0
  if (std::fabs(traj_back_l) < kNearCenterLaneThreshold) {
    return true;
  }
  // 4. offset 1.0 m
  constexpr double kLateralOffset = 0.8;
  if (is_left) {
    if (traj_back_l < 0.0) {
      *target_l = std::fmin(traj_back_l + kLateralOffset, 0.0);
    }
    if (*target_l < -ego_half_width) {
      *target_l = 0.0;
    }
  } else {
    if (traj_back_l > 0.0) {
      *target_l = std::fmax(traj_back_l - kLateralOffset, 0.0);
    }
    if (*target_l > ego_half_width) {
      *target_l = 0.0;
    }
  }
  return true;
}

bool AgentTrajectoryCalculator::GenerateMaxJerkQuinticPolyPath(
    const bool is_in_lane_change_execution,
    const std::shared_ptr<planning_math::KDPath>& current_lane_coord,
    const std::shared_ptr<agent::Agent>& ptr_agent, const bool is_left,
    const double target_l,
    std::vector<QuinticPolylinePointInfo>& quintic_poly_path,
    bool* need_extend_st_boundary_backward) {
  const int quintic_poly_num_sample = 30;
  const double max_ref_jerk = 3.0;
  const double max_ref_acc = 5.0;
  const double max_overshoot_l = 1.2;
  const double ego_v = std::fmax(2.778, ptr_agent->speed());

  quintic_poly_path.reserve(quintic_poly_num_sample);

  double agent_s_target_lane, agent_l_target_lane;
  if (!current_lane_coord->XYToSL(ptr_agent->x(), ptr_agent->y(),
                                  &agent_s_target_lane, &agent_l_target_lane)) {
    return false;
  }
  if (ptr_agent->trajectories_used_by_st_graph().empty()) {
    return false;
  }
  if (ptr_agent->trajectories_used_by_st_graph().front().empty()) {
    return false;
  }
  auto trajectory = ptr_agent->trajectories_used_by_st_graph().front();
  double agent_traj_back_s, agent_traj_back_l;
  if (!current_lane_coord->XYToSL(trajectory.back().x(), trajectory.back().y(),
                                  &agent_traj_back_s, &agent_traj_back_l)) {
    return false;
  }

  const auto& ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  const auto& init_point = ego_state_mgr->planning_init_point();
  double ego_s, ego_l;
  if (!current_lane_coord->XYToSL(init_point.x, init_point.y, &ego_s, &ego_l)) {
    return false;
  }
  const double kEgoPreviewTimeThd = 2.0;
  const bool agent_is_in_scope =
      init_point.v * kEgoPreviewTimeThd + ego_s > agent_s_target_lane;
  if (is_in_lane_change_execution && agent_is_in_scope) {
    agent_traj_back_s =
        agent_s_target_lane +
        std::fmax((agent_traj_back_s - agent_s_target_lane) / 2.0, 0.0);
    *need_extend_st_boundary_backward = true;
  }

  double max_s_end =
      GetEndMaxSEnd(ego_v, agent_s_target_lane, agent_l_target_lane);

  const int s_sample_num =
      std::fmin(60, (max_s_end - agent_s_target_lane) / 1.);
  const double step_s = (max_s_end - agent_s_target_lane) / s_sample_num;

  int left = 1, right = s_sample_num;
  auto converter =
      CoordinateConverter(trajectory.front().x(), trajectory.front().y(),
                          trajectory.front().theta());
  double final_max_jerk = 0;
  double final_max_acc = 0;
  int s_sample_count = s_sample_num;

  std::vector<QuinticPolylinePointInfo> last_quintic_poly_path;
  while (s_sample_count > 0) {
    quintic_poly_path.clear();
    final_max_jerk = 0;
    final_max_acc = 0;
    double end_s = agent_s_target_lane + step_s * s_sample_count;
    const auto target_point = current_lane_coord->GetPathPointByS(end_s);

    // vehicle coordinate. {y, dy, ddy}
    std::array<double, 3> start_point, end_point;
    double end_x;
    GetStartEndPointInfo(ptr_agent, target_point, converter, target_l,
                         start_point, end_point, end_x);
    planning_math::QuinticPoly1d quintic_poly(start_point, end_point, end_x);

    bool sample_success = true;  // reach target point.
    bool has_large_jerk = false;
    bool has_large_acc = false;
    bool has_large_overshoot = false;
    bool is_back = false;
    double last_s = std::numeric_limits<double>::min();

    double step_x = end_x / quintic_poly_num_sample;
    for (size_t index = 0; index < quintic_poly_num_sample; ++index) {
      double vehicle_x = index * step_x;
      double vehicle_y = quintic_poly.Evaluate(0, vehicle_x);
      auto global_pos = converter.ConvertToGlobal(vehicle_x, vehicle_y);
      double global_theta =
          std::atan(quintic_poly.Evaluate(1, vehicle_x)) + converter.Theta();
      double front_axis_x = global_pos.first;
      double front_axis_y = global_pos.second;

      auto acc_and_jerk = quintic_poly.GetLateralAccAndJerk(vehicle_x, ego_v);

      double front_axis_s, front_axis_l, rear_axis_s, rear_axis_l;

      if (!current_lane_coord->XYToSL(front_axis_x, front_axis_y, &front_axis_s,
                                      &front_axis_l)) {
        continue;
      }

      if (!current_lane_coord->XYToSL(global_pos.first, global_pos.second,
                                      &rear_axis_s, &rear_axis_l)) {
        continue;
      }

      has_large_overshoot = is_left ? rear_axis_l > max_overshoot_l
                                    : rear_axis_l < -max_overshoot_l;
      has_large_jerk = std::fabs(acc_and_jerk.second) > max_ref_jerk;
      has_large_acc = std::fabs(acc_and_jerk.first) > max_ref_acc;
      is_back = rear_axis_s < last_s;
      if (has_large_overshoot || has_large_jerk || has_large_acc || is_back) {
        sample_success = false;
        break;
      }
      last_s = rear_axis_s;
      quintic_poly_path.emplace_back(QuinticPolylinePointInfo{
          .front_axis_x = front_axis_x,
          .front_axis_y = front_axis_y,
          .front_axis_s = front_axis_s,
          .front_axis_l = front_axis_l,
          .rear_axis_x = global_pos.first,
          .rear_axis_y = global_pos.second,
          .rear_axis_s = rear_axis_s,
          .rear_axis_l = rear_axis_l,
          .theta = global_theta,
      });
      final_max_jerk =
          std::fmax(std::fabs(acc_and_jerk.second), final_max_jerk);
      final_max_acc = std::fmax(std::fabs(acc_and_jerk.first), final_max_acc);
    }
    // save path point when left==right and break;
    if (!quintic_poly_path.empty()) {
      last_quintic_poly_path = quintic_poly_path;
    }
    if (end_s < agent_traj_back_s + kFakePredictionTrajectoryExtraLength) {
      break;
    }
    --s_sample_count;
  }

  if (quintic_poly_path.empty()) {
    quintic_poly_path = last_quintic_poly_path;
  }

  return false;
}

bool AgentTrajectoryCalculator::GenarateAgentTrajectoryWithPolyPath(
    const bool is_in_lane_change_execution,
    const std::shared_ptr<agent::Agent>& ptr_agent,
    const std::vector<QuinticPolylinePointInfo>& quintic_poly_path) const {
  if (quintic_poly_path.empty()) {
    return false;
  }
  if (ptr_agent->trajectories_used_by_st_graph().empty()) {
    return false;
  }
  if (ptr_agent->trajectories_used_by_st_graph().front().empty()) {
    return false;
  }
  std::vector<planning_math::Vec2d> points;
  points.reserve(quintic_poly_path.size());
  for (const auto& p : quintic_poly_path) {
    points.emplace_back(p.front_axis_x, p.front_axis_y);
  }
  if (points.empty()) {
    return false;
  }
  std::vector<planning_math::PathPoint> path_points;
  path_points.reserve(points.size());
  planning_math::KDPath::PointsToPathPoints(points, &path_points);
  if (path_points.size() < planning_math::KDPath::kKDPathMinPathPointSize) {
    return false;
  }
  std::shared_ptr<planning_math::KDPath> trajectory_path =
      std::make_shared<planning_math::KDPath>(std::move(path_points));

  constexpr double kTimeStep = 0.2;
  constexpr int32_t kPlanCounter = 25;
  const auto& original_trajectory =
      ptr_agent->trajectories_used_by_st_graph().front();
  const double trajectory_start_time =
      original_trajectory.front().absolute_time();
  const double agent_speed = ptr_agent->speed();
  std::vector<trajectory::TrajectoryPoint> trajectory_points;
  trajectory_points.reserve(kPlanCounter);
  for (int32_t i = 0; i <= kPlanCounter; ++i) {
    double releative_t = i * kTimeStep;
    double absolute_t = trajectory_start_time + releative_t;
    auto original_point = original_trajectory.Evaluate(absolute_t);
    auto point = trajectory_path->GetPathPointByS(original_point.s());
    trajectory_points.emplace_back(point.x(), point.y(), point.theta(),
                                   original_point.vel(), original_point.acc(),
                                   absolute_t, 0.0, 0.0, original_point.s(),
                                   point.kappa());
    if (trajectory_path->Length() <= original_point.s() &&
        !is_in_lane_change_execution) {
      break;
    }
  }
  if (trajectory_points.empty()) {
    return false;
  }
  trajectory::Trajectory new_trajectory(std::move(trajectory_points));

  auto agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  auto* mutable_agent = agent_manager->mutable_agent(ptr_agent->agent_id());
  if (mutable_agent->trajectories_used_by_st_graph().empty()) {
    mutable_agent->add_trajectory(new_trajectory);
  } else {
    mutable_agent->add_trajectories_used_by_st_graph(new_trajectory);
  }

  return false;
}

double AgentTrajectoryCalculator::GetEndMaxSEnd(
    const double ego_speed, const double ego_s_target_lane,
    const double ego_l_target_lane) const {
  // assume ego follow a line segment to reach target lane.
  double ego_travel_dis = ego_speed * kPlanningTs;
  if (ego_travel_dis <= std::fabs(ego_l_target_lane)) {
    return ego_s_target_lane + kMinSafeDistance;
  }
  double addition_dis = std::sqrt(ego_travel_dis * ego_travel_dis -
                                  ego_l_target_lane * ego_l_target_lane);
  return ego_s_target_lane + addition_dis + kExtraAdditionDistance;
}

void AgentTrajectoryCalculator::GetStartEndPointInfo(
    const std::shared_ptr<agent::Agent>& ptr_agent,
    const planning_math::PathPoint& target_point,
    const CoordinateConverter& converter, const double target_l,
    std::array<double, 3>& start_point, std::array<double, 3>& end_point,
    double& end_x) const {
  // 1.  make target point with targer L
  constexpr double kEps = 1e-2;
  planning_math::Vec2d final_target_point(target_point.x(), target_point.y());
  if (std::fabs(target_l) > kEps) {
    double offset_theta = target_l > 0.0 ? target_point.theta() + M_PI_2
                                         : target_point.theta() - M_PI_2;
    final_target_point =
        final_target_point +
        std::fabs(target_l) *
            planning_math::Vec2d::CreateUnitVec2d(offset_theta);
  }

  // 2. convert to vehicle axis
  auto vehicle_end_point = converter.ConvertToVehicle(final_target_point.x(),
                                                      final_target_point.y());

  // start_point info.
  double start_dy = 0.;
  double start_ddy = 0.;

  // end_point info.
  double relative_target_theta = target_point.theta() - converter.Theta();
  double end_dy = std::tan(relative_target_theta);
  double end_ddy =
      target_point.kappa() / std::pow(std::cos(relative_target_theta), 3);

  start_point[0] = 0.;
  start_point[1] = start_dy;
  start_point[2] = start_ddy;
  end_x = vehicle_end_point.first;
  end_point[0] = vehicle_end_point.second;
  end_point[1] = end_dy;
  end_point[2] = end_ddy;
}

double AgentTrajectoryCalculator::CalculateAgentAverageSpeed(
    const std::shared_ptr<agent::Agent>& ptr_agent) const {
  constexpr double kAgentAverageSpeedMinPreTime = 3.0;
  double agent_average_speed = ptr_agent->speed();
  if (ptr_agent->trajectories_used_by_st_graph().empty()) {
    return agent_average_speed;
  }
  if (ptr_agent->trajectories_used_by_st_graph().front().empty()) {
    return agent_average_speed;
  }

  double agent_speed_total = 0.0;
  int agent_speed_count = 0.0;

  const double trajectory_start_time =
      ptr_agent->trajectories_used_by_st_graph()
          .front()
          .front()
          .absolute_time();
  for (const auto& pre_point :
       ptr_agent->trajectories_used_by_st_graph().front()) {
    ++agent_speed_count;
    agent_speed_total += pre_point.vel();
    if (pre_point.absolute_time() < trajectory_start_time) {
      continue;
    }
    if (pre_point.absolute_time() - trajectory_start_time >
        kAgentAverageSpeedMinPreTime) {
      break;
    }
  }

  if (agent_speed_count > 0) {
    agent_average_speed = agent_speed_total / agent_speed_count;
  }

  return agent_average_speed;
}

double AgentTrajectoryCalculator::CalculateRoadCurvature(const double v_ego) {
  double preview_x = kDistanceCurvature + kTimeCurvature * v_ego;
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto& frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  bool is_ref_path_smoothed = reference_path_ptr->GetIsSmoothed();
  double road_radius = 10000.0;
  if (is_ref_path_smoothed) {
    double curv = 0.0001;
    ReferencePathPoint refpath_pt;
    if (reference_path_ptr->get_reference_point_by_lon(
            frenet_ego_state.s() + preview_x, refpath_pt)) {
      curv = std::fabs(refpath_pt.path_point.kappa());
    }
    road_radius = 1 / std::max(curv, 0.0001);
  } else {
    std::vector<double> curv_window_vec;
    for (int idx = -3; idx <= 3; ++idx) {
      double curv;
      ReferencePathPoint refpath_pt;
      if (reference_path_ptr->get_reference_point_by_lon(
              frenet_ego_state.s() + preview_x + idx * 2.0, refpath_pt)) {
        curv = std::fabs(refpath_pt.path_point.kappa());
      } else {
        curv = 0.0001;
      }
      curv_window_vec.emplace_back(curv);
    }
    double curv_sum = 0.0;
    for (int ind = 0; ind < curv_window_vec.size(); ++ind) {
      curv_sum = curv_sum + curv_window_vec[ind];
    }
    double avg_curv = curv_sum / curv_window_vec.size();
    road_radius = 1 / std::max(avg_curv, 0.0001);
  }
  return road_radius;
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