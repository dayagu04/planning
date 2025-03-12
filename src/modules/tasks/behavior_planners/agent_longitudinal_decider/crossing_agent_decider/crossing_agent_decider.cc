#include "crossing_agent_decider.h"
#include "environmental_model.h"
#include "planning_context.h"

namespace planning {
namespace {
constexpr double kYieldVRUSpeedThreshold = 25.0;
constexpr double kYieldVRUHeadingThreshold = 0.785;
constexpr double kExpandVRUMinHeadingThreshold = 60 / 57.3;
constexpr double kExpandVRUMaxHeadingThreshold = 120 / 57.3;
constexpr double kVirtualizationVRUHeadingMinThreshold = 45 / 57.3;
constexpr double kVirtualizationVRUHeadingMaxThreshold = 135 / 57.3;
constexpr double kPredictionReverseSDiffThd = 3.0;
constexpr double kVRUPredictionReverseSDiffThd = 5.0;
constexpr double kReverseHeadingThreshold = 90 / 57.3;
constexpr double kVehiclePredLastPointLThd = 2.0;
constexpr double kVehiclePredCrossingPathTimeThd = 3.5;
constexpr double kVehicleSafeLongitudinalThd = 0.5;
constexpr double kVehicleInverseCrossingSpeedThd = 30 / 3.6;

constexpr int32_t kVirtualVRUObstacleBaseId = 10000;
constexpr double kVirtualVRUObstacleLength = 5.0;
constexpr double kVirtualVRUObstacleWidth = 2.0;
constexpr double kVRUDangerWidthToReference = 1.5;
constexpr double kEgoPassVRUSafeLength = 3.0;
const int32_t kNumNots = 25;
const double kStepTime = 0.1;
}  // namespace

CrossingAgentDecider::CrossingAgentDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<CrossingAgentDeciderConfig>();
  name_ = "CrossingAgentDecider";
}

bool CrossingAgentDecider::Reset() {
  vru_id_reverse_crossing_map_.clear();
  vehicle_id_reverse_crossing_map_.clear();
  return true;
}

bool CrossingAgentDecider::Execute() {
  LOG_DEBUG("=======CrossingAgentDecider======= \n");
  virtual_agents_vru_ptr_.clear();
  virtual_agents_vehicle_ptr_.clear();

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  const auto& environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();
  auto* mutable_agent_manager = session_->environmental_model()
                                    .get_dynamic_world()
                                    ->mutable_agent_manager();
  const auto* agent_manager =
      session_->environmental_model().get_dynamic_world()->agent_manager();
  if (agent_manager == nullptr || mutable_agent_manager == nullptr) {
    return false;
  }
  const auto& agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    return true;
  }
  for (const auto agent : agents) {
    if (agent == nullptr) {
      continue;
    }
    if (agent->agent_decision().agent_decision_type() ==
        agent::AgentDecisionType::IGNORE) {
      continue;
    }
    MakeYieldToVRUDecision(agent.get());
    MakeYieldToVehicleDecision(agent.get());
  }

  ClearVRUIdReverseCrossingMap();
  AddVirtualAgentIntoAgentManager();
  return true;
}

bool CrossingAgentDecider::MakeYieldToVRUDecision(
    const agent::Agent* const agent) {
  if (agent == nullptr) {
    return false;
  }

  if (agent->type() != agent::AgentType::PEDESTRIAN &&
      agent->type() != agent::AgentType::CYCLE_RIDING &&
      agent->type() != agent::AgentType::MOTORCYCLE_RIDING &&
      agent->type() != agent::AgentType::TRICYCLE_RIDING) {
    return false;
  }

  const auto& veh_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto ego_half_width = veh_param.width * 0.5;

  const auto& environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();
  const auto& ego_lane =
      environmental_model.get_virtual_lane_manager()->get_current_lane();

  if (ego_lane == nullptr) {
    return false;
  }
  // get reference path from ego lane
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return false;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return false;
  }

  const auto& init_point = ego_state_mgr->planning_init_point();

  // 1.judge vru prediction
  if (agent->trajectories().empty()) {
    return false;
  }
  if (agent->trajectories().front().empty()) {
    return false;
  }
  auto vru_pred_last_point = agent->trajectories().front().back();
  double vru_current_s, vru_current_l;
  double vru_pred_last_point_s, vru_pred_last_point_l;
  if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &vru_current_s,
                              &vru_current_l)) {
    return false;
  }
  if (!ego_lane_coord->XYToSL(vru_pred_last_point.x(), vru_pred_last_point.y(),
                              &vru_pred_last_point_s, &vru_pred_last_point_l)) {
    return false;
  }

  double ego_s, ego_l;
  if (!ego_lane_coord->XYToSL(init_point.x, init_point.y, &ego_s, &ego_l)) {
    return false;
  }
  if (vru_current_s < ego_s + veh_param.front_edge_to_rear_axle) {
    return false;
  }

  const auto& corners = agent->box().GetAllCorners();
  double agent_min_s = std::numeric_limits<double>::max();
  double agent_max_s = std::numeric_limits<double>::min();
  for (const auto& corner : corners) {
    double project_l = 0.0;
    double projrct_s = 0.0;
    if (!ego_lane_coord->XYToSL(corner.x(), corner.y(), &projrct_s,
                                    &project_l)) {
      continue;
    }
    agent_min_s = std::fmin(agent_min_s, projrct_s);
    agent_max_s = std::fmax(agent_max_s, projrct_s);
  }

  if (agent_min_s < ego_s + veh_param.front_edge_to_rear_axle && agent->is_reverse()) {
    return false;
  }

  // Notice: vru prediction is crossing then judge crossing
  bool is_vru_prediction_crossing = false;
  const double pred_s_diff = vru_current_s - vru_pred_last_point_s;
  if (vru_pred_last_point_l * vru_current_l < 0.0 &&
      pred_s_diff > -kPredictionReverseSDiffThd) {
    is_vru_prediction_crossing = true;
  }

  bool is_vru_reverse_crossing = false;
  if (vru_pred_last_point_l * vru_current_l < 0.0 &&
      pred_s_diff > kVRUPredictionReverseSDiffThd) {
    vru_id_reverse_crossing_map_[agent->agent_id()] =
        vru_current_l > 0.0 ? 1 : -1;
    is_vru_reverse_crossing = true;
  }
  if (!is_vru_prediction_crossing &&
      vru_id_reverse_crossing_map_.count(agent->agent_id())) {
    if (vru_current_l * vru_id_reverse_crossing_map_[agent->agent_id()] > 0.0) {
      vru_id_reverse_crossing_map_.erase(agent->agent_id());
    }
  }

  // 2.judge heading_diff
  bool is_heading_diff_satisfied_for_virtual = false;
  bool is_vru_prediction_satisfied_for_virtual = false;
  auto vru_matched_point = ego_lane_coord->GetPathPointByS(vru_current_s);
  const auto vru_heading = agent->theta();
  const auto ego_heading = init_point.heading_angle;
  const auto heading_diff = std::fabs(
      planning_math::NormalizeAngle(vru_heading - vru_matched_point.theta()));
  if (heading_diff > kVirtualizationVRUHeadingMinThreshold &&
      heading_diff < kVirtualizationVRUHeadingMaxThreshold) {
    is_heading_diff_satisfied_for_virtual = true;
  }
  if (std::fabs(vru_pred_last_point_l) < ego_half_width ||
      (vru_pred_last_point_l * vru_current_l < 0.0)) {
    is_vru_prediction_satisfied_for_virtual = true;
  }

  double vru_danger_zone_pred_point_rel_time =
      std::numeric_limits<double>::max();
  double vru_danger_zone_pred_point_s = std::numeric_limits<double>::max();
  const auto& first_point = agent->trajectories().front().at(0);
  const double trajectory_start_time = first_point.absolute_time();
  for (int32_t i = 0; i <= kNumNots; ++i) {
    double releative_t = i * kStepTime;
    double absolute_t = trajectory_start_time + releative_t;
    auto pred_point = agent->trajectories().front().Evaluate(absolute_t);
    double pred_point_s, pred_point_l;
    if (!ego_lane_coord->XYToSL(pred_point.x(), pred_point.y(), &pred_point_s,
                                &pred_point_l)) {
      continue;
    }
    if (std::fabs(pred_point_l) < kVRUDangerWidthToReference) {
      vru_danger_zone_pred_point_rel_time = releative_t;
      vru_danger_zone_pred_point_s = pred_point_s;
      break;
    }
  }

  if (v_ego * vru_danger_zone_pred_point_rel_time + ego_s > vru_current_s + kEgoPassVRUSafeLength &&
      v_ego * vru_danger_zone_pred_point_rel_time + ego_s > vru_danger_zone_pred_point_s + kEgoPassVRUSafeLength) {
    return false;
  }

  bool is_vru_crossing = false;
  if (is_vru_prediction_crossing || (is_heading_diff_satisfied_for_virtual &&
                                     is_vru_prediction_satisfied_for_virtual)) {
    is_vru_crossing = true;
  }

  if (is_vru_crossing ||
      vru_id_reverse_crossing_map_.count(
          agent->agent_id())) {  // can not this ,will feed  element
    ConstructVirtualAgentByCrossing(agent, true, is_vru_reverse_crossing);
  }
  return true;
}

bool CrossingAgentDecider::ClearVRUIdReverseCrossingMap() {
  const auto& environmental_model = session_->environmental_model();
  const auto* agent_manager =
      environmental_model.get_dynamic_world()->agent_manager();

  if (agent_manager == nullptr) {
    return false;
  }
  const auto& agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    vru_id_reverse_crossing_map_.clear();
    return false;
  }

  const auto& ego_lane =
      environmental_model.get_virtual_lane_manager()->get_current_lane();

  if (ego_lane == nullptr) {
    return false;
  }
  // get reference path from ego lane
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return false;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return false;
  }

  const auto& veh_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_bumper_to_rear_axle = veh_param.front_edge_to_rear_axle;
  const double ego_half_width = veh_param.width * 0.5;
  const double ego_half_length = veh_param.length * 0.5;
  constexpr double kIsVRUSafeLateralThd = 0.5;
  constexpr double kIsVRUSafeLongitudinalThd = 0.5;
  const double k_vru_longitudinal_thd =
      front_bumper_to_rear_axle - kIsVRUSafeLongitudinalThd;
  const double k_vru_lateral_thd = ego_half_width + kIsVRUSafeLateralThd;
  const double k_vehicle_longitudinal_thd =
      front_bumper_to_rear_axle - kVehicleSafeLongitudinalThd;
  const double k_vehicle_lateral_thd = ego_half_width;

  std::vector<int32_t> all_agents_ids;
  for (const auto agent : agents) {
    if (agent == nullptr) {
      continue;
    }
    all_agents_ids.push_back(agent->agent_id());
  }

  // clear not exist vru
  for (auto iter = vru_id_reverse_crossing_map_.begin();
       iter != vru_id_reverse_crossing_map_.end();) {
    if (std::find(all_agents_ids.begin(), all_agents_ids.end(), iter->first) ==
        all_agents_ids.end()) {
      iter = vru_id_reverse_crossing_map_.erase(iter);
    } else {
      ++iter;
    }
  }

  for (auto iter = vehicle_id_reverse_crossing_map_.begin();
       iter != vehicle_id_reverse_crossing_map_.end();) {
    if (std::find(all_agents_ids.begin(), all_agents_ids.end(), iter->first) ==
        all_agents_ids.end()) {
      iter = vehicle_id_reverse_crossing_map_.erase(iter);
    } else {
      ++iter;
    }
  }

  auto IsVRUSafe = [&](const int32_t vru_id, const int32_t position) -> bool {
    const auto* vru_agent = agent_manager->GetAgent(vru_id);
    if (vru_agent == nullptr) {
      return true;
    }
    double vru_s, vru_l;
    if (!ego_lane_coord->XYToSL(vru_agent->x(), vru_agent->y(), &vru_s,
                                &vru_l)) {
      return true;
    }

    const double vru_half_width = vru_agent->width() * 0.5;
    const bool is_crossed =
        (position == 1 && vru_l < 0.0) || (position == -1 && vru_l > 0.0);
    const double vru_nearest_l_abs = std::fabs(vru_l) - vru_half_width;
    if (vru_s < k_vru_longitudinal_thd) {
      return true;
    }
    if (vru_nearest_l_abs > k_vru_lateral_thd && is_crossed) {
      return true;
    }
    return false;
  };

  auto IsVehicleSafe = [&](const int32_t vehicle_id,
                           const int32_t position) -> bool {
    const auto* vehicle_agent = agent_manager->GetAgent(vehicle_id);
    if (vehicle_agent == nullptr) {
      return true;
    }
    double vehicle_s, vehicle_l;
    if (!ego_lane_coord->XYToSL(vehicle_agent->x(), vehicle_agent->y(),
                                &vehicle_s, &vehicle_l)) {
      return true;
    }
    const double vehicle_half_length = vehicle_agent->length() * 0.5;
    const bool is_crossed = (position == 1 && vehicle_l < 0.0) ||
                            (position == -1 && vehicle_l > 0.0);
    const double vehicle_nearest_l_abs =
        std::fabs(vehicle_l) - vehicle_half_length;
    if (vehicle_s < k_vehicle_longitudinal_thd) {
      return true;
    }
    if (vehicle_nearest_l_abs > k_vehicle_lateral_thd && is_crossed) {
      return true;
    }
    return false;
  };

  // clear safe vru
  for (auto iter = vru_id_reverse_crossing_map_.begin();
       iter != vru_id_reverse_crossing_map_.end();) {
    if (IsVRUSafe(iter->first, vru_id_reverse_crossing_map_[iter->first])) {
      iter = vru_id_reverse_crossing_map_.erase(iter);
    } else {
      ++iter;
    }
  }

  for (auto iter = vehicle_id_reverse_crossing_map_.begin();
       iter != vehicle_id_reverse_crossing_map_.end();) {
    if (IsVehicleSafe(iter->first,
                      vehicle_id_reverse_crossing_map_[iter->first])) {
      iter = vehicle_id_reverse_crossing_map_.erase(iter);
    } else {
      ++iter;
    }
  }

  return true;
}

bool CrossingAgentDecider::MakeYieldToVehicleDecision(
    const agent::Agent* const agent) {
  if (agent == nullptr) {
    return false;
  }

  if (agent->type() == agent::AgentType::PEDESTRIAN ||
      agent->type() == agent::AgentType::CYCLE_RIDING ||
      agent->type() == agent::AgentType::MOTORCYCLE_RIDING ||
      agent->type() == agent::AgentType::TRICYCLE_RIDING) {
    return false;
  }

  if (agent->speed() > kVehicleInverseCrossingSpeedThd) {
    return false;
  }

  const auto& veh_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_bumper_to_rear_axle = veh_param.front_edge_to_rear_axle;
  const double ego_half_width = veh_param.width * 0.5;
  const double ego_half_length = veh_param.length * 0.5;
  const double k_vehicle_longitudinal_safe_thd =
      front_bumper_to_rear_axle - kVehicleSafeLongitudinalThd;

  const auto& environmental_model = session_->environmental_model();
  const auto& ego_lane =
      environmental_model.get_virtual_lane_manager()->get_current_lane();

  if (ego_lane == nullptr) {
    return false;
  }
  // get reference path from ego lane
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return false;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return false;
  }

  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto& init_point = ego_state_mgr->planning_init_point();

  if (agent->trajectories().empty()) {
    return false;
  }
  if (agent->trajectories().front().empty()) {
    return false;
  }
  const auto& vehicle_pred_trajectory = agent->trajectories().front();
  const auto& vehicle_pred_last_point = agent->trajectories().front().back();
  double vehicle_current_s, vehicle_current_l;
  double vehicle_pred_last_point_s, vehicle_pred_last_point_l;
  if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &vehicle_current_s,
                              &vehicle_current_l)) {
    return false;
  }
  if (!ego_lane_coord->XYToSL(
          vehicle_pred_last_point.x(), vehicle_pred_last_point.y(),
          &vehicle_pred_last_point_s, &vehicle_pred_last_point_l)) {
    return false;
  }
  if (vehicle_current_s < k_vehicle_longitudinal_safe_thd) {
    return false;
  }

  const auto& first_point = vehicle_pred_trajectory.at(0);
  const double trajectory_start_time = first_point.absolute_time();
  const auto second_point =
      vehicle_pred_trajectory.Evaluate(trajectory_start_time + 0.5);
  planning_math::LineSegment2d segment(first_point, second_point);
  const bool is_heading_normal =
      std::fabs(planning_math::NormalizeAngle(segment.heading() -
                                              agent->box().heading())) < M_PI_2;
  if (!is_heading_normal) {
    return false;
  }

  auto matched_path_point = ego_lane_coord->GetPathPointByS(vehicle_current_s);
  const double heading_diff = std::fabs(planning_math::NormalizeAngle(
      matched_path_point.theta() - agent->box().heading()));

  // judge reverse vehicle
  if (heading_diff < kReverseHeadingThreshold &&
      !vehicle_id_reverse_crossing_map_.count(agent->agent_id())) {
    return false;
  }

  bool is_vehicle_prediction_crossing = false;
  double vehicle_corssing_pred_point_rel_time =
      std::numeric_limits<double>::max();
  for (int32_t i = 0; i <= kNumNots; ++i) {
    double releative_t = i * kStepTime;
    double absolute_t = trajectory_start_time + releative_t;
    auto pred_point = vehicle_pred_trajectory.Evaluate(absolute_t);
    double pred_point_s, pred_point_l;
    if (!ego_lane_coord->XYToSL(pred_point.x(), pred_point.y(), &pred_point_s,
                                &pred_point_l)) {
      continue;
    }
    if ((vehicle_current_l > 0.0 && pred_point_l < 0.0) ||
        (vehicle_current_l < 0.0 && pred_point_l > 0.0)) {
      vehicle_corssing_pred_point_rel_time = releative_t;
      break;
    }
  }

  if (vehicle_pred_last_point_l * vehicle_current_l < 0.0 &&
      vehicle_corssing_pred_point_rel_time < kVehiclePredCrossingPathTimeThd) {
    is_vehicle_prediction_crossing = true;
    vehicle_id_reverse_crossing_map_[agent->agent_id()] =
        vehicle_current_l > 0.0 ? 1 : -1;
  }

  // clear when prediction unstable
  if (!is_vehicle_prediction_crossing &&
      vehicle_id_reverse_crossing_map_.count(agent->agent_id())) {
    if (vehicle_current_l *
            vehicle_id_reverse_crossing_map_[agent->agent_id()] >
        0.0) {
      vehicle_id_reverse_crossing_map_.erase(agent->agent_id());
    }
  }

  if (is_vehicle_prediction_crossing ||
      vehicle_id_reverse_crossing_map_.count(agent->agent_id())) {
    ConstructVirtualAgentByCrossing(agent, false, false);
  }
  return true;
}

bool CrossingAgentDecider::ConstructVirtualAgentByCrossing(
    const agent::Agent* const agent, const bool is_vru,
    const double is_vru_reverse_crossing) {
  if (agent == nullptr) {
    return false;
  }

  const auto& environmental_model = session_->environmental_model();
  const auto& ego_lane =
      environmental_model.get_virtual_lane_manager()->get_current_lane();

  if (ego_lane == nullptr) {
    return false;
  }
  // get reference path from ego lane
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return false;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return false;
  }

  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto& init_point = ego_state_mgr->planning_init_point();

  double agent_s = 0.0;
  double agent_l = 0.0;
  if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
    return false;
  }

  agent::Agent virtual_agent;
  agent::AgentType object_type = agent::AgentType::VIRTUAL;
  const auto& veh_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_front_to_center_distance =
      veh_param.front_edge_to_rear_axle - veh_param.rear_axle_to_center;
  double desired_virtual_s = agent_s;
  CalcDesiredVirtualObsS(is_vru_reverse_crossing, agent, &desired_virtual_s);
  auto obs_point = ego_lane_coord->GetPathPointByS(desired_virtual_s);

  virtual_agent.set_agent_id(kVirtualVRUObstacleBaseId + agent->agent_id());
  virtual_agent.set_type(object_type);

  virtual_agent.set_x(obs_point.x());
  virtual_agent.set_y(obs_point.y());

  virtual_agent.set_length(kVirtualVRUObstacleLength);
  virtual_agent.set_width(kVirtualVRUObstacleWidth);

  virtual_agent.set_speed(0.0);
  virtual_agent.set_theta(obs_point.theta());
  virtual_agent.set_accel(0.0);
  virtual_agent.set_fusion_source(1);  // camera avoiding filtered in st

  planning_math::Box2d box(
      planning_math::Vec2d(virtual_agent.x(), virtual_agent.y()),
      virtual_agent.theta(), virtual_agent.length(), virtual_agent.width());

  virtual_agent.set_box(box);
  virtual_agent.set_timestamp_s(0.0);
  virtual_agent.set_timestamp_us(0.0);

  // construct trajectory for virtual obs
  std::vector<trajectory::Trajectory> trajectories;
  trajectories.reserve(1);
  trajectory::Trajectory trajectory;
  for (int i = 0; i < kNumNots; ++i) {
    auto point = trajectory::TrajectoryPoint(
        obs_point.x(), obs_point.y(), init_point.heading_angle, 0.0, 0.0,
        i * kStepTime, 0.0, 0.0, agent_s, 0.0);
    trajectory.emplace_back(point);
  }

  trajectories.emplace_back(trajectory);
  virtual_agent.set_trajectories(trajectories);

  if (is_vru) {
    virtual_agents_vru_ptr_.emplace_back(
        std::make_unique<agent::Agent>(virtual_agent));
  } else {
    virtual_agents_vehicle_ptr_.emplace_back(
        std::make_unique<agent::Agent>(virtual_agent));
  }
  return true;
}

bool CrossingAgentDecider::CalcDesiredVirtualObsS(
    const bool is_vru_reverse_crossing, const agent::Agent* const agent,
    double* desired_virtual_s) {
  constexpr double kDesiredDistancePreviewTimeThd = 1.0;
  if (!is_vru_reverse_crossing) {
    return false;
  }
  const auto& environmental_model = session_->environmental_model();
  const auto& ego_lane =
      environmental_model.get_virtual_lane_manager()->get_current_lane();

  if (ego_lane == nullptr) {
    return false;
  }
  // get reference path from ego lane
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return false;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return false;
  }

  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto& init_point = ego_state_mgr->planning_init_point();

  if (agent->trajectories().empty()) {
    return false;
  }
  if (agent->trajectories().front().empty()) {
    return false;
  }
  const auto vru_pred_trajectory = agent->trajectories().front();
  const double trajectory_start_time =
      vru_pred_trajectory.front().absolute_time();
  double intersection_point_s = -1.0;
  double min_pred_l = std::numeric_limits<double>::max();
  for (int32_t i = 0; i <= kNumNots; ++i) {
    double releative_t = i * kStepTime;
    double absolute_t = trajectory_start_time + releative_t;
    auto pred_point = vru_pred_trajectory.Evaluate(absolute_t);
    double pred_point_s, pred_point_l;
    if (!ego_lane_coord->XYToSL(pred_point.x(), pred_point.y(), &pred_point_s,
                                &pred_point_l)) {
      continue;
    }
    if (std::fabs(pred_point_l) < min_pred_l) {
      intersection_point_s = pred_point_s;
    }
  }
  if (intersection_point_s < 0.0) {
    return false;
  }
  const double ego_preview_distance =
      std::fmax(1.0, init_point.v) * kDesiredDistancePreviewTimeThd;
  if (intersection_point_s < ego_preview_distance) {
    return false;
  }

  *desired_virtual_s = intersection_point_s;
  return true;
}

bool CrossingAgentDecider::AddVirtualAgentIntoAgentManager() {
  auto* mutable_agent_manager = session_->environmental_model()
                                    .get_dynamic_world()
                                    ->mutable_agent_manager();
  const auto* agent_manager =
      session_->environmental_model().get_dynamic_world()->agent_manager();
  if (agent_manager == nullptr || mutable_agent_manager == nullptr) {
    return false;
  }
  std::unordered_map<int32_t, agent::Agent> agent_table;

  for (auto& virtual_agent_ptr : virtual_agents_vru_ptr_) {
    if (virtual_agent_ptr == nullptr) {
      continue;
    }
    agent_table.insert(std::pair<int32_t, agent::Agent>(
        virtual_agent_ptr->agent_id(), *virtual_agent_ptr.get()));
  }
  for (auto& virtual_agent_ptr : virtual_agents_vehicle_ptr_) {
    if (virtual_agent_ptr == nullptr) {
      continue;
    }
    agent_table.insert(std::pair<int32_t, agent::Agent>(
        virtual_agent_ptr->agent_id(), *virtual_agent_ptr.get()));
  }

  mutable_agent_manager->Append(agent_table);
  return true;
}

}  // namespace planning