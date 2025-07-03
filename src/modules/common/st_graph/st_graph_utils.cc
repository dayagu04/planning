#include "st_graph_utils.h"

#include <cstdint>

#include "common_c.h"
#include "define/geometry.h"
#include "math/box2d.h"
#include "vehicle_config_context.h"

namespace planning {
namespace speed {

namespace {
using namespace planning_math;

constexpr double kDistanceThr = 0.01;
constexpr double kMathEpsilon = 1e-10;
constexpr double kSearchBuffer = 5.0;
constexpr double kLargeAgentLengthM = 8.0;
constexpr double kMpsToKph = 3.6;
constexpr double kTimeResolution = 0.2;
constexpr double kCautionYieldSpeedThreshold = 25.0;
constexpr double kDynamicBufferSpeedThresholdKph = 40.0;
constexpr double kCautionYieldMaxWidth = 0.7;
constexpr double kCautionYieldMinWidth = 0.3;
constexpr double kCautioYieldSlowSpeedThreshold = 25.0;
constexpr double kCautionYieldHeadingThres = 0.785;
constexpr double kCautionYieldLowBoundThres = 0.1;
constexpr double kCutInHeadingthres = 4.0 / 180.0 * 3.14;
constexpr double kLowSpeedIgnreAgentSpeedThrMps = 0.8;
constexpr double kStaticAgentSpeedThrMps = 0.5;
}  // namespace

bool StGraphUtils::IsStaticAgent(const agent::Agent& agent) {
  bool is_low_speed =
      agent.speed() < kStaticAgentSpeedThrMps || agent.trajectories().empty();
  bool is_ignore_traj = agent.speed() < kLowSpeedIgnreAgentSpeedThrMps &&
                        agent.has_low_spd_unstable_trajectory();
  // cone bucket
  bool is_sod = agent.is_sod();
  return is_low_speed || is_ignore_traj || is_sod;
}

const agent::Agent* StGraphUtils::GetFrontAgentOfTargetLane(
    const std::shared_ptr<planning_data::DynamicWorld>& dynamic_world,
    const std::string lane_change_status,
    const std::string& lane_change_request) {
  const auto* ptr_agent_manager = dynamic_world->agent_manager();
  if (nullptr == ptr_agent_manager) {
    return nullptr;
  }
  if (lane_change_status == "" || lane_change_request == "") {
    return nullptr;
  }
  int64_t target_front_agent_id = -1;
  int32_t target_lane_front_agent_id = -1;
  if (lane_change_request == "none") {
    target_front_agent_id = dynamic_world->ego_front_node_id();
  } else {
    if (lane_change_status == "none" ||
        lane_change_status == "left_lane_change_wait" ||
        lane_change_status == "right_lane_change_wait") {
      target_front_agent_id = dynamic_world->ego_front_node_id();
    } else if (lane_change_status == "left_lane_change") {
      target_front_agent_id = dynamic_world->ego_left_front_node_id();
    } else if (lane_change_status == "right_lane_change") {
      target_front_agent_id = dynamic_world->ego_right_front_node_id();
    } else {
      target_front_agent_id = dynamic_world->ego_front_node_id();
    }
  }

  if (target_front_agent_id != -1) {
    auto* target_lane_front_node =
        dynamic_world->GetNode(target_front_agent_id);
    if (target_lane_front_node != nullptr) {
      target_lane_front_agent_id = target_lane_front_node->node_agent_id();
    }
  }
  return dynamic_world->agent_manager()->GetAgent(target_lane_front_agent_id);
}

const agent::Agent* StGraphUtils::GetRearAgentOfTargetLane(
    const std::shared_ptr<planning_data::DynamicWorld>& dynamic_world,
    const std::string lane_change_status,
    const std::string& lane_change_request) {
  const auto* ptr_agent_manager = dynamic_world->agent_manager();
  if (nullptr == ptr_agent_manager) {
    return nullptr;
  }
  if (lane_change_status == "" || lane_change_request == "") {
    return nullptr;
  }
  int64_t target_rear_agent_id = -1;
  int32_t target_lane_rear_agent_id = -1;
  if (lane_change_request == "none") {
    target_rear_agent_id = dynamic_world->ego_rear_node_id();
  } else {
    if (lane_change_status == "none" ||
        lane_change_status == "left_lane_change_wait" ||
        lane_change_status == "right_lane_change_wait") {
      target_rear_agent_id = dynamic_world->ego_rear_node_id();
    } else if (lane_change_status == "left_lane_change") {
      target_rear_agent_id = dynamic_world->ego_left_rear_node_id();
    } else if (lane_change_status == "right_lane_change") {
      target_rear_agent_id = dynamic_world->ego_right_rear_node_id();
    } else {
      target_rear_agent_id = dynamic_world->ego_rear_node_id();
    }
  }

  if (target_rear_agent_id != -1) {
    auto* target_lane_front_node = dynamic_world->GetNode(target_rear_agent_id);
    if (target_lane_front_node != nullptr) {
      target_lane_rear_agent_id = target_lane_front_node->node_agent_id();
    }
  }
  return dynamic_world->agent_manager()->GetAgent(target_lane_rear_agent_id);
}

double StGraphUtils::CalculateLateralBufferForNormalLaneKeeping(
    const trajectory::TrajectoryPoint& init_point,
    const double lower_lateral_buffer_m, const double upper_lateral_buffer_m,
    const double lower_speed_kph, const double upper_speed_kph) {
  double speed_kph = init_point.vel() * kMpsToKph;
  return planning_math::LerpWithLimit(lower_lateral_buffer_m, lower_speed_kph,
                                      upper_lateral_buffer_m, upper_speed_kph,
                                      speed_kph);
}

double StGraphUtils::CalculateLateralBufferForTimeRange(
    const double lower_lateral_buffer_m, const double upper_lateral_buffer_m,
    const double lower_t, const double upper_t, const double t) {
  return planning_math::LerpWithLimit(upper_lateral_buffer_m, lower_t,
                                      lower_lateral_buffer_m, upper_t, t);
}

void StGraphUtils::DetermineCautionYieldDecision(
    const std::shared_ptr<StGraphInput>& st_graph_input,
    const std::string lane_change_status, const std::string lane_change_request,
    const std::unordered_map<int32_t, std::vector<int64_t>>&
        agent_id_st_boundaries_map,
    const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
        boundary_id_st_boundaries_map,
    std::vector<int32_t>& caution_yield_agent_ids) {
  caution_yield_agent_ids.clear();
  if (st_graph_input->time_aligned_ego_state().vel() >
      kCautionYieldSpeedThreshold / kMpsToKph) {
    return;
  }
  const bool is_lane_change = lane_change_request != "none" &&
                              (lane_change_status == "left_lane_change" ||
                               lane_change_status == "right_lane_change");
  if (is_lane_change) {
    return;
  }
  const auto& agents = st_graph_input->filtered_agents();
  const auto planned_kd_path = st_graph_input->processed_path();
  if (!planned_kd_path) {
    return;
  }
  const double half_ego_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width * 0.5;
  const double ego_heading = st_graph_input->planning_init_point().theta();
  for (const auto agent : agents) {
    if (agent == nullptr) {
      continue;
    }
    if (agent->is_cutin()) {
      continue;
    }
    if (agent->type() == agent::AgentType::PEDESTRIAN ||
        agent->type() == agent::AgentType::BICYCLE ||
        agent->type() == agent::AgentType::TRICYCLE ||
        agent->type() == agent::AgentType::CYCLE_RIDING ||
        agent->type() == agent::AgentType::TRICYCLE_RIDING) {
      continue;
    }
    if (agent->speed() > kCautioYieldSlowSpeedThreshold / kMpsToKph) {
      continue;
    }
    const auto agent_heading = agent->theta();
    const auto delta_heading =
        std::fabs(planning_math::NormalizeAngle(agent_heading - ego_heading));
    if (delta_heading > kCautionYieldHeadingThres) {
      continue;
    }
    const auto& agent_id = agent->agent_id();
    const auto& obs_box = agent->box();
    const auto& obs_corners = obs_box.GetAllCorners();
    double min_l = std::numeric_limits<double>::max();
    bool has_positive = false;
    bool has_negative = false;
    bool has_error = false;
    for (size_t i = 0; i < obs_corners.size(); ++i) {
      double project_s = 0.0, project_l = 0.0;
      if (!planned_kd_path->XYToSL(obs_corners[i].x(), obs_corners[i].y(),
                                   &project_s, &project_l)) {
        has_error = true;
      }
      has_positive = has_positive || (project_l > 0.0);
      has_negative = has_negative || (project_l < 0.0);
      const double delta_l = std::fabs(project_l) - half_ego_width;
      min_l = std::fmin(min_l, delta_l);
    }
    if (has_error) {
      continue;
    }
    if (has_negative && has_positive) {
      continue;
    }
    const auto signed_delta_heading =
        has_positive
            ? planning_math::NormalizeAngle(ego_heading - agent_heading)
            : planning_math::NormalizeAngle(agent_heading - ego_heading);
    if (signed_delta_heading > kCutInHeadingthres) {
      continue;
    }
    if (min_l > kCautionYieldMaxWidth || min_l < kCautionYieldMinWidth) {
      continue;
    }
    if (!agent_id_st_boundaries_map.count(agent_id)) {
      continue;
    }
    const auto& st_boundaries = agent_id_st_boundaries_map.at(agent_id);
    for (const auto& st_boundary_id : st_boundaries) {
      if (!boundary_id_st_boundaries_map.count(st_boundary_id)) {
        continue;
      }
      auto& ptr_st_boundary = boundary_id_st_boundaries_map.at(st_boundary_id);
      bool is_lower_than_low_bound = false;
      for (int idx = 0; idx < ptr_st_boundary->lower_points().size(); idx++) {
        if (ptr_st_boundary->lower_points()[idx].s() <
            kCautionYieldLowBoundThres) {
          is_lower_than_low_bound = true;
          break;
        }
      }
      if (is_lower_than_low_bound) {
        continue;
      }
      ptr_st_boundary->set_decision_type(
          STBoundary::DecisionType::CAUTION_YIELD);
      caution_yield_agent_ids.emplace_back(agent_id);
    }
  }
}

void StGraphUtils::PreUpdateStBoundaryForLaneChange(
    const std::unordered_map<int32_t, std::vector<int64_t>>&
        agent_id_st_boundaries_map,
    const agent::Agent* front_agent_of_target,
    const agent::Agent* rear_agent_of_target, const double safety_buffer_m,
    std::unordered_map<int64_t, std::unique_ptr<STBoundary>>* const
        boundary_id_st_boundaries_map) {
  if (nullptr == front_agent_of_target || nullptr == rear_agent_of_target ||
      nullptr == boundary_id_st_boundaries_map) {
    return;
  }
  // get front and rear agent of target lane
  const int32_t front_agent_id = front_agent_of_target->agent_id();
  const int32_t rear_agent_id = rear_agent_of_target->agent_id();
  if (agent_id_st_boundaries_map.count(front_agent_id) == 0 ||
      agent_id_st_boundaries_map.count(rear_agent_id) == 0) {
    return;
  }
  const auto& front_st_boundaries =
      agent_id_st_boundaries_map.at(front_agent_id);
  const auto& rear_st_boundaries = agent_id_st_boundaries_map.at(rear_agent_id);
  if (front_st_boundaries.empty() || rear_st_boundaries.empty()) {
    return;
  }
  const auto& front_st_id = front_st_boundaries.front();
  const auto& rear_st_id = rear_st_boundaries.front();
  if (boundary_id_st_boundaries_map->count(front_st_id) == 0 ||
      boundary_id_st_boundaries_map->count(rear_st_id) == 0) {
    return;
  }
  const auto& front_st_boundary =
      *boundary_id_st_boundaries_map->at(front_st_id);
  const auto& rear_st_boundary = *boundary_id_st_boundaries_map->at(rear_st_id);

  constexpr double kProcessT = 4.0;
  const double front_min_t = front_st_boundary.min_t();
  const double front_max_t = front_st_boundary.max_t();
  const double rear_min_t = rear_st_boundary.min_t();
  const double rear_max_t = rear_st_boundary.max_t();
  if (front_min_t > kProcessT || front_max_t < kProcessT ||
      rear_min_t > kProcessT || rear_max_t < kProcessT) {
    return;
  }

  STPoint front_process_lower_point;
  STPoint front_process_upper_point;
  STPoint rear_process_lower_point;
  STPoint rear_process_upper_point;
  if (!front_st_boundary.GetBoundaryBounds(
          kProcessT, &front_process_lower_point, &front_process_upper_point) ||
      !rear_st_boundary.GetBoundaryBounds(kProcessT, &rear_process_lower_point,
                                          &rear_process_upper_point) ||
      front_process_lower_point.s() < rear_process_upper_point.s()) {
    return;
  }
  const double delta_s =
      std::fmax(front_process_lower_point.s() - rear_process_upper_point.s(),
                safety_buffer_m);

  const auto& upper_points = rear_st_boundary.upper_points();
  const auto& lower_points = rear_st_boundary.lower_points();
  if (lower_points.size() != upper_points.size() || upper_points.size() < 2) {
    return;
  }
  std::vector<std::pair<STPoint, STPoint>> st_point_pairs;
  st_point_pairs.reserve(upper_points.size());
  double last_delta_s_of_rear_original_point =
      upper_points.front().s() - lower_points.front().s();
  for (int i = 0; i < upper_points.size(); ++i) {
    const auto& upper_point = upper_points.at(i);
    const auto& lower_point = lower_points.at(i);
    const double current_t = upper_point.t();
    // 1.keep original point
    if (current_t < kProcessT) {
      st_point_pairs.emplace_back(lower_point, upper_point);
      last_delta_s_of_rear_original_point = upper_point.s() - lower_point.s();
      continue;
    }

    // 2. use front lower point s - delta_s
    if (current_t < front_max_t) {
      STPoint front_upper_point;
      STPoint front_lower_point;
      if (!front_st_boundary.GetBoundaryBounds(current_t, &front_lower_point,
                                               &front_upper_point)) {
        return;
      }
      const double last_lower_s =
          st_point_pairs.empty() ? 0.0 : st_point_pairs.back().first.s();
      const double last_upper_s =
          st_point_pairs.empty() ? 0.0 : st_point_pairs.back().second.s();

      const double current_lower_s =
          std::fmax(last_lower_s, front_lower_point.s() - delta_s -
                                      last_delta_s_of_rear_original_point);
      const double current_upper_s =
          std::fmax(last_upper_s, front_lower_point.s() - delta_s);

      st_point_pairs.emplace_back(
          STPoint(current_lower_s, lower_point.t(), rear_agent_id, rear_st_id,
                  front_lower_point.velocity(),
                  front_lower_point.acceleration()),
          STPoint(current_upper_s, upper_point.t(), rear_agent_id, rear_st_id,
                  front_lower_point.velocity(),
                  front_lower_point.acceleration()));
      continue;
    }

    // 3.use the last rear point
    if (!st_point_pairs.empty()) {
      st_point_pairs.emplace_back(
          STPoint(st_point_pairs.back().first.s(), lower_point.t(),
                  rear_agent_id, rear_st_id, 0.0, 0.0),
          STPoint(st_point_pairs.back().second.s(), upper_point.t(),
                  rear_agent_id, rear_st_id, 0.0, 0.0));
    }
  }

  // std::cout << "[Pre]update st boundary,id:" << rear_st_id << ",rear agent
  // id:" << rear_agent_id
  //           << std::endl;
  std::unique_ptr<STBoundary> st_boundary(new STBoundary(st_point_pairs));
  st_boundary->set_id(rear_st_id);
  (*boundary_id_st_boundaries_map)[rear_st_id] = std::move(st_boundary);
}

void StGraphUtils::UpdateStBoundaryForLaneChange(
    const std::unordered_map<int32_t, std::vector<int64_t>>&
        agent_id_st_boundaries_map,
    const agent::Agent* front_agent_of_target,
    const agent::Agent* rear_agent_of_target, const double safety_buffer_m,
    std::unordered_map<int64_t, std::unique_ptr<STBoundary>>* const
        boundary_id_st_boundaries_map) {
  if (nullptr == front_agent_of_target || nullptr == rear_agent_of_target ||
      nullptr == boundary_id_st_boundaries_map) {
    return;
  }
  // get front and rear agent of target lane
  const int32_t front_agent_id = front_agent_of_target->agent_id();
  const int32_t rear_agent_id = rear_agent_of_target->agent_id();
  if (agent_id_st_boundaries_map.count(front_agent_id) == 0 ||
      agent_id_st_boundaries_map.count(rear_agent_id) == 0) {
    return;
  }
  const auto& front_st_boundaries =
      agent_id_st_boundaries_map.at(front_agent_id);
  const auto& rear_st_boundaries = agent_id_st_boundaries_map.at(rear_agent_id);
  if (front_st_boundaries.empty() || rear_st_boundaries.empty()) {
    return;
  }
  const auto& front_st_id = front_st_boundaries.front();
  const auto& rear_st_id = rear_st_boundaries.front();
  if (boundary_id_st_boundaries_map->count(front_st_id) == 0 ||
      boundary_id_st_boundaries_map->count(rear_st_id) == 0) {
    return;
  }
  const auto& front_st_boundary =
      *boundary_id_st_boundaries_map->at(front_st_id);
  const auto& rear_st_boundary = *boundary_id_st_boundaries_map->at(rear_st_id);
  // decide the min_t and max_t
  const double min_t =
      std::fmax(front_st_boundary.min_t(), rear_st_boundary.min_t());
  const double max_t =
      std::fmax(front_st_boundary.max_t(), rear_st_boundary.max_t());
  if (min_t > max_t - kTimeResolution) {
    return;
  }

  bool need_process = false;
  bool first_meet_invalid = false;

  double start_process_t = 0.0;
  double rear_processed_upper_s = 0.0;
  double rear_processed_lower_s = 0.0;

  double last_rear_upper_s = -1.0;
  double last_rear_lower_s = -1.0;
  double last_front_upper_s = -1.0;
  double last_front_lower_s = -1.0;
  double last_t = -1.0;

  // use deque to store the points of front agent that have collision with rear
  // agent
  std::deque<STPoint> collision_front_agent_lower_points;
  for (double t = max_t; t >= min_t; t -= kTimeResolution) {
    STPoint front_lower_point;
    STPoint front_upper_point;
    STPoint rear_lower_point;
    STPoint rear_upper_point;

    if (!front_st_boundary.GetBoundaryBounds(t, &front_lower_point,
                                             &front_upper_point) ||
        !rear_st_boundary.GetBoundaryBounds(t, &rear_lower_point,
                                            &rear_upper_point)) {
      continue;
    }
    double front_lower_s = front_lower_point.s();
    double front_upper_s = front_upper_point.s();
    double rear_lower_s = rear_lower_point.s();
    double rear_upper_s = rear_upper_point.s();

    // save the current s and t
    last_rear_upper_s = rear_upper_s;
    last_rear_lower_s = rear_lower_s;
    last_front_upper_s = front_upper_s;
    last_front_lower_s = front_lower_s;
    last_t = t;

    const bool is_valid = rear_upper_s <= front_lower_s - safety_buffer_m;
    if (!is_valid) {
      // collision case
      first_meet_invalid = true;
      collision_front_agent_lower_points.emplace_front(front_lower_point);
      continue;
    }
    if (is_valid && first_meet_invalid) {
      need_process = true;
      start_process_t = t;
      rear_processed_upper_s = rear_upper_s;
      rear_processed_lower_s = rear_lower_s;
      break;
    }
  }

  // bool need_fallback = false;
  if (!need_process && first_meet_invalid &&
      last_rear_upper_s < last_front_lower_s) {
    // fallback for that the safety_buffer_m is not small enough
    need_process = true;
    start_process_t = last_t;
    rear_processed_upper_s = last_rear_upper_s;
    rear_processed_lower_s = last_rear_lower_s;
  }

  const auto& upper_points = rear_st_boundary.upper_points();
  const auto& lower_points = rear_st_boundary.lower_points();

  double last_delta_s_of_rear_original_point = 0.0;
  const std::size_t collision_front_agent_lower_points_size =
      collision_front_agent_lower_points.size();
  std::size_t collision_front_agent_lower_point_idx = 0;

  if (need_process && upper_points.size() == lower_points.size() &&
      lower_points.size() > 2) {
    // std::cout << " process...\n";
    last_delta_s_of_rear_original_point =
        upper_points.front().s() - lower_points.front().s();
    std::vector<std::pair<STPoint, STPoint>> st_point_pairs;
    st_point_pairs.reserve(upper_points.size());
    for (int32_t i = 0; i < upper_points.size(); ++i) {
      const auto& upper_point = upper_points.at(i);
      const auto& lower_point = lower_points.at(i);
      // 1.keep original point
      if (upper_point.t() < start_process_t) {
        last_delta_s_of_rear_original_point = upper_point.s() - lower_point.s();
        st_point_pairs.emplace_back(lower_point, upper_point);
        // std::cout << "  1.original upper_s:" << upper_point.s() <<
        // ",lower_s:" << lower_point.s()
        //           << ",t:" << upper_point.t() << '\n';
        continue;
      }

      // 2.use collison front agent lower point - safety_buffer_m
      if (collision_front_agent_lower_point_idx <
          collision_front_agent_lower_points_size) {
        const double last_lower_s =
            st_point_pairs.empty() ? 0.0 : st_point_pairs.back().first.s();
        const double last_upper_s =
            st_point_pairs.empty() ? 0.0 : st_point_pairs.back().second.s();
        const auto& collision_point = collision_front_agent_lower_points.at(
            collision_front_agent_lower_point_idx);

        const double current_lower_s =
            std::fmax(last_lower_s, collision_point.s() - safety_buffer_m -
                                        last_delta_s_of_rear_original_point);
        const double current_upper_s =
            std::fmax(last_upper_s, collision_point.s() - safety_buffer_m);

        st_point_pairs.emplace_back(
            STPoint(current_lower_s, lower_point.t(), rear_agent_id, rear_st_id,
                    collision_point.velocity(), collision_point.acceleration()),
            STPoint(current_upper_s, upper_point.t(), rear_agent_id, rear_st_id,
                    collision_point.velocity(),
                    collision_point.acceleration()));
        collision_front_agent_lower_point_idx++;
        // std::cout << "  2.collison lower s:" << collision_point.s()
        //           << ",upper_s:" << current_upper_s << ",lower_s:" <<
        //           current_lower_s
        //           << ",t:" << upper_point.t() << '\n';
        continue;
      }

      // 3.no rest of collison front agent lower points,use the last rear point
      if (!st_point_pairs.empty()) {
        // std::cout << "  3.no rest upper_s:" <<
        // st_point_pairs.back().second.s()
        //           << ",lower_s:" << st_point_pairs.back().first.s() << ",t:"
        //           << upper_point.t()
        //           << '\n';
        st_point_pairs.emplace_back(
            STPoint(st_point_pairs.back().first.s(), lower_point.t(),
                    rear_agent_id, rear_st_id, 0.0, 0.0),
            STPoint(st_point_pairs.back().second.s(), upper_point.t(),
                    rear_agent_id, rear_st_id, 0.0, 0.0));
      }
    }
    // std::cout << "  update st boundary,id:" << rear_st_id << '\n';
    std::unique_ptr<STBoundary> st_boundary(new STBoundary(st_point_pairs));
    st_boundary->set_id(rear_st_id);
    (*boundary_id_st_boundaries_map)[rear_st_id] = std::move(st_boundary);
  }
}

void StGraphUtils::UpdateStBoundaryForOvertaking(
    const std::shared_ptr<StGraphInput>& st_graph_input,
    const std::unordered_map<int32_t, std::vector<int64_t>>&
        agent_id_st_boundaries_map,
    const agent::Agent* rear_agent_of_target,
    std::unordered_map<int64_t, std::unique_ptr<STBoundary>>* const
        boundary_id_st_boundaries_map) {
  if (st_graph_input->is_lane_keeping()) {
    return;
  }
  if (st_graph_input->max_acceleration_curve() == nullptr) {
    return;
  }
  if (nullptr == boundary_id_st_boundaries_map) {
    return;
  }
  const auto rear_st_id =
      GetAgentStBoundaryId(rear_agent_of_target, agent_id_st_boundaries_map);
  if (boundary_id_st_boundaries_map->find(rear_st_id) ==
      boundary_id_st_boundaries_map->end()) {
    return;
  }
  const auto& rear_st_boundary = *boundary_id_st_boundaries_map->at(rear_st_id);
  if (rear_st_boundary.max_t() - rear_st_boundary.min_t() < kTimeResolution) {
    return;
  }
  const auto& max_acceleration_curve = st_graph_input->max_acceleration_curve();
  double max_valid_time = rear_st_boundary.min_t();
  for (double time = rear_st_boundary.min_t(); time < rear_st_boundary.max_t();
       time += kTimeResolution) {
    double s_lower = 0.0;
    double s_upper = 0.0;
    if (!rear_st_boundary.GetBoundarySRange(time, &s_lower, &s_upper)) {
      break;
    }
    const double curve_s = max_acceleration_curve->Evaluate(0, time);
    if (curve_s <= s_upper) {
      break;
    }
    max_valid_time = time;
  }
  if (rear_st_boundary.max_t() - max_valid_time > 2. * kTimeResolution) {
    // std::cout << "max_acceleration_curve has collision with overtaking st
    // boundary: "
    //           << rear_agent_of_target->agent_id() << ", " << rear_st_id
    //           << ", crop and keep valid st boundary range: " <<
    //           rear_st_boundary.min_t() << ", "
    //           << rear_st_boundary.max_t() << ", " << max_valid_time <<
    //           std::endl;
    std::vector<std::pair<STPoint, STPoint>> st_point_pairs;
    st_point_pairs.reserve(rear_st_boundary.upper_points().size());
    for (auto i = 0; i < rear_st_boundary.upper_points().size(); ++i) {
      const auto& upper_point = rear_st_boundary.upper_points().at(i);
      if (upper_point.t() > max_valid_time) {
        break;
      }
      const auto& lower_point = rear_st_boundary.lower_points().at(i);
      st_point_pairs.emplace_back(lower_point, upper_point);
    }
    std::unique_ptr<STBoundary> st_boundary(new STBoundary(st_point_pairs));
    st_boundary->set_id(rear_st_id);
    (*boundary_id_st_boundaries_map)[rear_st_id] = std::move(st_boundary);
  }
}

int32_t StGraphUtils::GetAgentStBoundaryId(
    const agent::Agent* agent,
    const std::unordered_map<int32_t, std::vector<int64_t>>&
        agent_id_st_boundaries_map) {
  if (nullptr == agent) {
    return -1;
  }
  const int32_t agent_id = agent->agent_id();
  if (agent_id_st_boundaries_map.find(agent_id) ==
      agent_id_st_boundaries_map.end()) {
    return -1;
  }
  const auto& st_boundaries = agent_id_st_boundaries_map.at(agent_id);
  if (st_boundaries.empty()) {
    return -1;
  }
  return st_boundaries.front();
}

bool StGraphUtils::IsLargeAgent(const agent::Agent& agent) {
  return agent::AgentType::BUS == agent.type() ||
         agent::AgentType::TRUCK == agent.type() ||
         agent::AgentType::TRAILER == agent.type() ||
         agent.length() > kLargeAgentLengthM;
}

// 1.the agent is not virtual
// 2.the agent is in front of ego
// 3.the agent is at ego lane or neighbor to ego lane
// 4.lane keeping
// 5.agent is in ego time range
// 6.agent is not reverse
bool StGraphUtils::CheckCandicateAgentForClosePass(
    const bool is_lane_keeping, const trajectory::TrajectoryPoint& init_point,
    const std::shared_ptr<planning_math::KDPath>& planned_path,
    const std::shared_ptr<VirtualLane>& ego_lane,
    const std::shared_ptr<VirtualLane>& agent_lane, const agent::Agent& agent,
    const StBoundaryType& type, const bool is_rads_scene) {
  constexpr double kTimeThd = 5.0;
  if (type != StBoundaryType::NORMAL || !is_lane_keeping) {
    return false;
  }
  double ego_s = 0.0;
  double ego_l = 0.0;
  double agent_s = 0.0;
  double agent_l = 0.0;
  if (!planned_path->XYToSL(init_point.x(), init_point.y(), &ego_s, &ego_l) ||
      !planned_path->XYToSL(agent.x(), agent.y(), &agent_s, &agent_l)) {
    return false;
  }
  const double ego_v = init_point.vel();
  const auto& veh_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_edge_to_center = veh_param.front_edge_to_rear_axle;
  const double rear_edge_to_center = veh_param.rear_edge_to_rear_axle;
  const double agent_front_s = agent_s + agent.box().half_length();
  double ego_front_s = ego_s + front_edge_to_center;
  if (is_rads_scene) {
    ego_front_s = ego_s + rear_edge_to_center;
  }
  const bool is_virtual = agent.type() == agent::AgentType::VIRTUAL;
  const bool is_in_front_of_ego = agent_front_s > ego_front_s;
  const bool is_agent_in_lon_range = agent_s < (ego_s + ego_v * kTimeThd);
  const bool is_agent_neighbor =
      std::abs(agent_lane->get_virtual_id() - ego_lane->get_virtual_id()) <= 1;
  return !is_virtual && is_in_front_of_ego && is_agent_neighbor &&
         is_agent_in_lon_range && !agent.is_reverse();
}

double StGraphUtils::RecalculateLateralBufferForLargeAgent(
    const trajectory::TrajectoryPoint& planning_init_point,
    const agent::AgentManager& agent_manager, const agent::Agent& agent,
    const double expand_buffer, const double small_expand_buffer,
    const double default_lateral_buffer, int32_t* const prev_st_count) {
  if (agent.is_reverse() && agent.is_vehicle_type()) {
    return default_lateral_buffer;
  }
  constexpr double kMpsToKph = 3.6;
  constexpr double kEgoUpperSpeedKph = 30.0;
  constexpr double kAgentUpperSpeedKph = 30.0;
  const auto* ptr_prev_agent =
      agent_manager.GetHistoryAgentByIndex(agent.agent_id(), 1);
  // normal agent or no previous agent,or large agent is fast
  if (!StGraphUtils::IsLargeAgent(agent) ||
      agent.speed() * kMpsToKph > kAgentUpperSpeedKph ||
      nullptr == ptr_prev_agent || nullptr == prev_st_count) {
    return default_lateral_buffer;
  }

  const double ego_speed_kph = planning_init_point.vel() * kMpsToKph;
  const double final_expand_buffer =
      ego_speed_kph < kEgoUpperSpeedKph ? expand_buffer : small_expand_buffer;

  const auto& prev_st_info = ptr_prev_agent->agent_st_info();
  const bool prev_in_st_graph = prev_st_info.is_in_st_graph_;
  const double prev_lateral_buffer = prev_st_info.lateral_buffer_;
  // const double prev_min_t = prev_st_info.min_t_;
  *prev_st_count = prev_st_info.st_graph_count_;

  // previous is in st-graph,the first frame need expand buffer, other case just
  // keep the previous lateral buffer
  if (prev_in_st_graph) {
    return (1 == *prev_st_count) ? (prev_lateral_buffer + final_expand_buffer)
                                 : prev_lateral_buffer;
  }
  // previous is not in st-graph,use default value
  return default_lateral_buffer;
}

void StGraphUtils::CalculateAgentSLBoundary(
    const std::shared_ptr<planning_math::KDPath>& kd_path,
    const planning_math::Box2d& obs_box,
    const std::pair<double, double>& path_range, const StBoundaryType type,
    std::vector<double>* const agent_sl_boundary,
    std::vector<std::pair<int32_t, planning_math::Vec2d>>* const
        considered_corners) {
  // max_s min_s max_l min_l
  agent_sl_boundary->at(0) = std::numeric_limits<double>::lowest();
  agent_sl_boundary->at(1) = std::numeric_limits<double>::max();
  agent_sl_boundary->at(2) = std::numeric_limits<double>::lowest();
  agent_sl_boundary->at(3) = std::numeric_limits<double>::max();

  const auto& obs_corners = obs_box.GetAllCorners();
  const double vehicle_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width;
  const double left_border_position = vehicle_width * 0.5;
  const double right_border_position = -left_border_position;
  for (size_t i = 0; i < obs_corners.size(); ++i) {
    double project_s = 0.0, project_l = 0.0;
    kd_path->XYToSL(obs_corners[i].x(), obs_corners[i].y(), &project_s,
                    &project_l);
    agent_sl_boundary->at(3) = std::fmin(agent_sl_boundary->at(3), project_l);
    agent_sl_boundary->at(2) = std::fmax(agent_sl_boundary->at(2), project_l);
    agent_sl_boundary->at(1) = std::fmin(agent_sl_boundary->at(1), project_s);
    agent_sl_boundary->at(0) = std::fmax(agent_sl_boundary->at(0), project_s);
    if (project_l >= right_border_position &&
        project_l <= left_border_position) {
      considered_corners->emplace_back(i, Vec2d(project_s, project_l));
    }
  }
}

bool StGraphUtils::CalculateAgentSLBoundary(
    const std::shared_ptr<KDPath>& planned_path, const Box2d& agent_box,
    std::vector<double>* const agent_sl_boundary,
    std::vector<std::pair<int32_t, planning_math::Vec2d>>* const
        considered_corners) {
  // max_s min_s max_l min_l
  agent_sl_boundary->at(0) = std::numeric_limits<double>::lowest();
  agent_sl_boundary->at(1) = std::numeric_limits<double>::max();
  agent_sl_boundary->at(2) = std::numeric_limits<double>::lowest();
  agent_sl_boundary->at(3) = std::numeric_limits<double>::max();

  const auto& obs_corners = agent_box.GetAllCorners();
  const double vehicle_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width;
  const double left_border_position = vehicle_width * 0.5;
  const double right_border_position = -left_border_position;
  for (size_t i = 0; i < obs_corners.size(); ++i) {
    Point2D project_point{0.0, 0.0};
    if (!planned_path->XYToSL({obs_corners[i].x(), obs_corners[i].y()},
                              project_point)) {
      continue;
    };
    agent_sl_boundary->at(3) =
        std::fmin(agent_sl_boundary->at(3), project_point.y);
    agent_sl_boundary->at(2) =
        std::fmax(agent_sl_boundary->at(2), project_point.y);
    agent_sl_boundary->at(1) =
        std::fmin(agent_sl_boundary->at(1), project_point.x);
    agent_sl_boundary->at(0) =
        std::fmax(agent_sl_boundary->at(0), project_point.x);
    if (project_point.y >= right_border_position &&
        project_point.y <= left_border_position) {
      considered_corners->emplace_back(i,
                                       Vec2d(project_point.x, project_point.y));
    }
  }

  if (agent_sl_boundary->at(1) > 200.0 || agent_sl_boundary->at(0) < -200.0 ||
      agent_sl_boundary->at(3) > 50.0 || agent_sl_boundary->at(2) < -50.0) {
    return false;
  }
  return true;
}

bool StGraphUtils::CalculateSRange(
    const std::shared_ptr<planning_math::KDPath>& kd_path,
    const PathBorderQuerier& path_border_querier,
    const planning_math::Box2d& obs_box, const StBoundaryType type,
    const std::pair<double, double>& path_range,
    const std::vector<double>& agent_sl_boundary,
    std::vector<std::pair<int32_t, planning_math::Vec2d>>& considered_corners,
    const planning_math::Box2d& planning_init_point_box, double* const lower_s,
    double* const upper_s, const bool is_rads_scene) {
  // const auto& obs_corners = obs_box.GetAllCorners();
  // max_s min_s max_l min_l
  const double max_s = agent_sl_boundary[0];
  const double min_s = agent_sl_boundary[1];
  const double max_l = agent_sl_boundary[2];
  const double min_l = agent_sl_boundary[3];
  const double vehicle_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width;
  const double left_border_position = vehicle_width * 0.5;
  const double right_border_position = -left_border_position;
  double front_edge_to_center = VehicleConfigurationContext::Instance()
                                    ->get_vehicle_param()
                                    .front_edge_to_rear_axle;
  double back_edge_to_center = VehicleConfigurationContext::Instance()
                                   ->get_vehicle_param()
                                   .rear_edge_to_rear_axle;
  if (is_rads_scene) {
    front_edge_to_center = VehicleConfigurationContext::Instance()
                               ->get_vehicle_param()
                               .rear_edge_to_rear_axle;
    back_edge_to_center = VehicleConfigurationContext::Instance()
                              ->get_vehicle_param()
                              .front_edge_to_rear_axle;
  }

  if (type == StBoundaryType::NEIGHBOR) {
    *lower_s = min_s - front_edge_to_center;
    *upper_s = max_s + back_edge_to_center;
    return true;
  } else if (type == StBoundaryType::EXPAND) {
    *lower_s = min_s - front_edge_to_center;
    *upper_s = max_s + back_edge_to_center;
    *lower_s = std::fmax(0.0, std::fmin(*lower_s, path_range.second));
    *upper_s = std::fmax(0.0, std::fmin(*upper_s, path_range.second));
    return *lower_s > kMathEpsilon || *upper_s > kMathEpsilon;
  }

  if (min_s > path_range.second || max_s < path_range.first) {
    return false;
  }

  // case 1: path border has no intersect with obs_box
  if (min_l > left_border_position || max_l < right_border_position) {
    return false;
    // case 2: path border completly overlap obs_box
  } else if (min_l >= right_border_position && max_l <= left_border_position) {
    *lower_s = std::fmin(min_s - front_edge_to_center, *lower_s);
    *upper_s = std::fmax(max_s + back_edge_to_center, *upper_s);
    *lower_s = std::fmax(0.0, *lower_s);
    *upper_s = std::fmin(*upper_s, path_range.second);
    return true;
  }

  double obs_center_s = 0.0;
  double obs_center_l = 0.0;
  kd_path->XYToSL(obs_box.center_x(), obs_box.center_y(), &obs_center_s,
                  &obs_center_l);
  const double obs_diagonal = obs_box.diagonal();
  const double search_distance = obs_diagonal * 0.5 + kSearchBuffer;
  int32_t start_index = -1;
  int32_t end_index = -1;
  if (!path_border_querier.GetObjects(obs_center_s - search_distance,
                                      obs_center_s + search_distance,
                                      &start_index, &end_index)) {
    return false;
  }

  bool has_left_intersect_point = false;
  bool has_right_intersect_point = false;
  std::vector<LineSegment2d> left_intersective_edges;
  StGraphUtils::GetIntersectiveLineSegments(kd_path, true /*is_left*/, obs_box,
                                            agent_sl_boundary,
                                            &left_intersective_edges);
  std::vector<LineSegment2d> right_intersective_edges;
  StGraphUtils::GetIntersectiveLineSegments(kd_path, false /*is_left*/, obs_box,
                                            agent_sl_boundary,
                                            &right_intersective_edges);
  for (int index = start_index; index <= end_index; ++index) {
    PathBorderSegment path_border_segment =
        path_border_querier.GetPathBorderSegmentByIndex(index);
    const auto& left_border = path_border_segment.left_border();
    const auto& right_border = path_border_segment.right_border();
    const double path_border_start_s = path_border_segment.start_s();

    // case 3: obs_box completely cross path
    if (min_l < right_border_position && max_l > left_border_position) {
      if (!left_intersective_edges.empty()) {
        StGraphUtils::CalculateIntersectS(
            left_intersective_edges, left_border, path_border_start_s,
            &has_left_intersect_point, lower_s, upper_s);
      }
      if (!right_intersective_edges.empty()) {
        StGraphUtils::CalculateIntersectS(
            right_intersective_edges, right_border, path_border_start_s,
            &has_right_intersect_point, lower_s, upper_s);
      }
      // case 4: obs_box only intersect with path left border
    } else if (min_l < left_border_position && max_l > left_border_position) {
      if (!left_intersective_edges.empty()) {
        StGraphUtils::CalculateIntersectS(
            left_intersective_edges, left_border, path_border_start_s,
            &has_left_intersect_point, lower_s, upper_s);
      }
      // case 5: obs_box only intersect with path right border
    } else if (max_l > right_border_position && min_l < right_border_position) {
      if (!right_intersective_edges.empty()) {
        StGraphUtils::CalculateIntersectS(
            right_intersective_edges, right_border, path_border_start_s,
            &has_right_intersect_point, lower_s, upper_s);
      }
    }
  }
  if (considered_corners.empty() && !has_left_intersect_point &&
      !has_right_intersect_point) {
    return false;
  }
  for (const auto& corner : considered_corners) {
    if (corner.second.x() < *lower_s) {
      *lower_s = corner.second.x();
    }
    if (corner.second.x() > *upper_s) {
      *upper_s = corner.second.x();
    }
  }

  if (*upper_s < -back_edge_to_center) {
    return false;
  }
  if (*upper_s < 0.0 && !planning_init_point_box.HasOverlap(obs_box)) {
    return false;
  }
  *lower_s -= front_edge_to_center;
  *upper_s += back_edge_to_center;

  *lower_s = std::fmax(0.0, *lower_s);
  *upper_s = std::fmax(0.0, std::fmin(*upper_s, path_range.second));

  // Final protection
  const bool is_invalid = *lower_s < kMathEpsilon && *upper_s < kMathEpsilon;
  return !is_invalid;
}

void StGraphUtils::GetIntersectiveLineSegments(
    const std::shared_ptr<planning_math::KDPath>& kd_path, const bool is_left,
    const Box2d& obs_box, const std::vector<double>& agent_sl_boundary,
    std::vector<LineSegment2d>* const intersective_segments) {
  const double vehicle_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width;
  const double lat_border_position =
      is_left ? vehicle_width * 0.5 : -vehicle_width * 0.5;
  const double max_l = agent_sl_boundary[2];
  const double min_l = agent_sl_boundary[3];
  if (is_left && min_l > lat_border_position) {
    return;
  }
  if (!is_left && max_l < lat_border_position) {
    return;
  }
  const auto& edges = obs_box.GetAllEdges();
  intersective_segments->reserve(edges.size());
  for (const auto& edge : edges) {
    const auto& start_point = edge.start();
    const auto& end_point = edge.end();
    double start_project_s = 0.0;
    double start_project_l = 0.0;
    double end_project_s = 0.0;
    double end_project_l = 0.0;
    kd_path->XYToSL(start_point.x(), start_point.y(), &start_project_s,
                    &start_project_l);
    kd_path->XYToSL(end_point.x(), end_point.y(), &end_project_s,
                    &end_project_l);
    const double start_dl = start_project_l - lat_border_position;
    const double end_dl = end_project_l - lat_border_position;
    if (start_dl * end_dl < 0) {
      intersective_segments->emplace_back(edge);
    }
  }
}

void StGraphUtils::CalculateIntersectS(
    const std::vector<planning_math::LineSegment2d>& intersective_edges,
    const planning_math::LineSegment2d& border, const double start_s,
    bool* const has_intersect_point, double* const lower_s,
    double* const upper_s) {
  for (const auto& edge : intersective_edges) {
    planning_math::Vec2d intersect;
    if (edge.GetIntersect(border, &intersect)) {
      *has_intersect_point = true;
      const double delta_s = intersect.DistanceTo(border.start());
      *lower_s = std::fmin(delta_s + start_s, *lower_s);
      *upper_s = std::fmax(delta_s + start_s, *upper_s);
    }
  }
}

void StGraphUtils::DetermineClosetStBoundary(
    const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
        boundary_id_st_boundaries_map,
    int64_t& closest_boundary_id, double& closest_s) {
  closest_s = std::numeric_limits<double>::max();
  closest_boundary_id = -1;
  for (const auto& boundary_id_st_boundary : boundary_id_st_boundaries_map) {
    const auto& boundary_id = boundary_id_st_boundary.first;
    const auto& st_boundary = boundary_id_st_boundary.second;
    if (!st_boundary) {
      continue;
    }
    if (st_boundary->min_t() > kMathEpsilon) {
      continue;
    }
    const auto& lower_points = st_boundary->lower_points();
    if (lower_points.empty()) {
      continue;
    }
    const auto first_lower_point_s = lower_points.front().s();
    if (first_lower_point_s < kMathEpsilon) {
      continue;
    }
    if (closest_s > first_lower_point_s) {
      closest_s = first_lower_point_s;
      closest_boundary_id = boundary_id;
    }
  }
}

// Check whether need to adjust the lateral buffer of the agent by t.
bool StGraphUtils::CheckAdjustLateralBufferByT(
    const trajectory::TrajectoryPoint& init_point,
    const std::shared_ptr<VirtualLaneManager>& virtual_lane_manager,
    const std::shared_ptr<VirtualLane>& ego_lane,
    const std::shared_ptr<VirtualLane>& agent_lane, const agent::Agent& agent,
    const bool is_parallel_to_ego_lane, const bool is_lane_keeping,
    const bool is_rads_scene) {
  bool need_ajust_buffer_by_t = false;
  if (agent_lane->get_virtual_id() == ego_lane->get_virtual_id()) {
    return need_ajust_buffer_by_t;
  }

  // 1.agent is fast
  // 2.not cut in agent
  // 3.not vru
  // 4.agent is neighbor to ego
  constexpr double kObjLowerSpeedKph = 40.0;
  const bool obj_speed_meet_condition =
      agent.speed() * kMpsToKph >= kObjLowerSpeedKph;
  const bool is_cut_in = agent.is_cutin();
  const bool is_vru = agent.is_vru();
  const bool is_neighbor_to_ego_lane =
      std::abs(agent_lane->get_virtual_id() - ego_lane->get_virtual_id()) == 1;

  // 5.agent is in range
  const auto& ego_lane_coord =
      ego_lane->get_reference_path()->get_frenet_coord();
  double ego_s = 0.0;
  double ego_l = 0.0;
  ego_lane_coord->XYToSL(init_point.x(), init_point.y(), &ego_s, &ego_l);
  const auto vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  constexpr double kMaxConsiderRangeM = 60.0;
  constexpr double kMinConsiderRangeM = 20.0;
  double half_consider_range = init_point.vel() * 2.0;
  half_consider_range =
      std::fmin(kMaxConsiderRangeM * 0.5, half_consider_range);
  half_consider_range =
      std::fmax(kMinConsiderRangeM * 0.5, half_consider_range);
  double ego_front_consider_s =
      ego_s + vehicle_param.front_edge_to_rear_axle + half_consider_range;
  double ego_back_consider_s =
      ego_s - vehicle_param.rear_edge_to_rear_axle - half_consider_range;
  if (is_rads_scene) {
    ego_front_consider_s =
        ego_s + vehicle_param.rear_edge_to_rear_axle + half_consider_range;
    ego_back_consider_s =
        ego_s - vehicle_param.front_edge_to_rear_axle - half_consider_range;
  }
  double agent_s = 0.0;
  double agent_l = 0.0;
  ego_lane_coord->XYToSL(agent.x(), agent.y(), &agent_s, &agent_l);
  const double agent_front_s = agent_s + agent.box().half_length();
  const double agent_back_s = agent_s - agent.box().half_length();
  const bool is_agent_in_consider_range =
      !((agent_back_s > ego_front_consider_s) ||
        (agent_front_s < ego_back_consider_s));

  // 6.not large heading diff
  double heading_diff = 0.0;
  heading_diff = std::fabs(
      planning_math::NormalizeAngle(agent.theta() - init_point.theta()));
  constexpr double kMaxHeadingDiff = 0.5;
  const double large_heading_diff = heading_diff >= kMaxHeadingDiff;

  need_ajust_buffer_by_t =
      is_lane_keeping && is_parallel_to_ego_lane && is_neighbor_to_ego_lane &&
      is_agent_in_consider_range && obj_speed_meet_condition && !is_cut_in &&
      !is_vru && !large_heading_diff;

  return need_ajust_buffer_by_t;
}

bool StGraphUtils::NeedDynamicBufferForTimeRange(const agent::Agent& agent) {
  double speed_kph = agent.speed() * kMpsToKph;
  if (speed_kph < kDynamicBufferSpeedThresholdKph) {
    return false;
  }
  if (agent.is_cut_out_for_lane_change() || agent.is_cutin() ||
      agent.is_vru()) {
    return false;
  }
  return true;
}

bool StGraphUtils::LinearExtendTrajectory(
    const trajectory::Trajectory& trajectory, const double time,
    trajectory::TrajectoryPoint* point) {
  if (trajectory.empty()) {
    return false;
  }

  const double dt = time - trajectory.back().absolute_time();
  const double acc = trajectory.back().acc();
  const double vel = trajectory.back().vel();
  const double theta = trajectory.back().theta();
  const double pred_ds = std::fmax(0.0, vel * dt);
  if (trajectory.size() > 1) {
    const double dx = trajectory[trajectory.size() - 1].x() -
                      trajectory[trajectory.size() - 2].x();
    const double dy = trajectory[trajectory.size() - 1].x() -
                      trajectory[trajectory.size() - 2].y();
    const double ds = std::sqrt(dx * dx + dy * dy);
    if (ds < kDistanceThr) {
      point->set_x(trajectory.back().x());
      point->set_y(trajectory.back().y());
      point->set_theta(theta);
      point->set_vel(vel);
      point->set_acc(acc);
      point->set_absolute_time(time);
    } else {
      planning_math::Vec2d last_point(trajectory.back().x(),
                                      trajectory.back().y());
      auto pred_point =
          last_point + planning_math::Vec2d::CreateUnitVec2d(theta) * pred_ds;
      point->set_x(pred_point.x());
      point->set_y(pred_point.y());
      point->set_theta(theta);
      point->set_vel(std::fmax(0.0, vel));
      point->set_acc(acc);
      point->set_absolute_time(time);
    }
  } else {
    point->set_x(trajectory.back().x());
    point->set_y(trajectory.back().y());
    point->set_theta(theta);
    point->set_vel(0);
    point->set_acc(0);
    point->set_absolute_time(time);
  }
  return true;
}

// If agent's trajectory point is out of ddlane,then ignore the lateral buffer.
double StGraphUtils::AdjustLateralBufferByT(
    const trajectory::TrajectoryPoint& agent_point, const double default_buffer,
    const std::shared_ptr<VirtualLane>& ptr_agent_lane) {
  if (nullptr == ptr_agent_lane) {
    return default_buffer;
  }
  const auto agent_lane_coord = ptr_agent_lane->get_lane_frenet_coord();
  double s = 0.0;
  double l = 0.0;
  agent_lane_coord->XYToSL(agent_point.x(), agent_point.y(), &s, &l);
  const bool is_in_lane_range = s > 0.0 && s < agent_lane_coord->Length();
  return is_in_lane_range ? default_buffer : 0.0;
}

bool IsNeighborAgentNearBoundary(
    const bool is_large_agent, const bool is_neighbor_to_ego_lane,
    const agent::Agent& agent, const std::shared_ptr<VirtualLane>& ego_lane,
    const std::shared_ptr<VirtualLaneManager>& virtual_lane_manager) {
  constexpr double kEpsilon = 1.0e-4;
  constexpr double kNearBoundaryThresholdM = 0.2;
  constexpr double kNearBoundaryForLargeAgentThresholdM = 0.0;
  const double near_boundary_threshold =
      is_large_agent ? kNearBoundaryForLargeAgentThresholdM
                     : kNearBoundaryThresholdM;
  if (!is_neighbor_to_ego_lane) {
    return false;
  }
  bool is_near_boundary = false;
  const auto& corners = agent.box().GetAllCorners();
  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  for (const auto& corner : corners) {
    double corner_s = 0.0;
    double corner_l = 0.0;
    if (!ego_lane_coord->XYToSL(corner.x(), corner.y(), &corner_s, &corner_l)) {
      continue;
    }
    double current_left_width = 0.0;
    double current_right_width = 0.0;
    constexpr double kDefaultHalfWidth = 1.75;
    const auto& left_lane = virtual_lane_manager->get_lane_with_virtual_id(
        ego_lane->get_virtual_id() - 1);
    const auto& right_lane = virtual_lane_manager->get_lane_with_virtual_id(
        ego_lane->get_virtual_id() + 1);
    if (left_lane != nullptr) {
      current_left_width = left_lane->width_by_s(corner_s);
    } else {
      current_left_width = kDefaultHalfWidth;
    }
    if (right_lane != nullptr) {
      current_right_width = right_lane->width_by_s(corner_s);
    } else {
      current_right_width = kDefaultHalfWidth;
    }
    is_near_boundary =
        corner_l < kEpsilon
            ? corner_l > -(current_right_width + near_boundary_threshold)
            : corner_l < (current_left_width + near_boundary_threshold);
    if (is_near_boundary) {
      return true;
    }
  }
  return false;
}

bool StGraphUtils::CheckLonFarPositionSTBoundary(
    const agent::Agent& agent,
    const std::vector<std::pair<STPoint, STPoint>>& st_point_pairs,
    const std::shared_ptr<StGraphInput>& st_graph_input, const bool is_parallel,
    const std::shared_ptr<VirtualLane>& ego_lane,
    const std::shared_ptr<VirtualLane>& ptr_agent_lane,
    const std::shared_ptr<VirtualLaneManager>& virtual_lane_manager) {
  constexpr double kNormalAgentIgnoreTimeThd = 4.0;
  constexpr double kLargeAgentIgnoreTimeThd = 4.0;
  constexpr double kNormalAgentRadicalIgnoreTimeThd = 3.0;
  constexpr double kLargeAgentRadicalIgnoreTimeThd = 3.5;
  constexpr double kLowerHeadingDiff = 2.0 / 57.3;
  constexpr double kEgoPreTimeThd = 1.5;

  if (agent.is_cutin() || agent.is_rule_base_cutin() ||
      agent.is_prediction_cutin() || st_point_pairs.empty() ||
      nullptr == ptr_agent_lane) {
    // std::cout << " return error1\n";
    return false;
  }

  const auto planned_kd_path = st_graph_input->processed_path();
  const auto& init_point = st_graph_input->planning_init_point();
  if (planned_kd_path == nullptr) {
    return false;
  }
  double ego_s, ego_l;
  double agent_s, agent_l;
  if (!planned_kd_path->XYToSL(init_point.x(), init_point.y(), &ego_s,
                               &ego_l)) {
    return false;
  }
  if (!planned_kd_path->XYToSL(agent.x(), agent.y(), &agent_s, &agent_l)) {
    return false;
  }

  const bool is_large_agent = StGraphUtils::IsLargeAgentByLength(agent);

  // check whether use radical param(by heading diff,near to ego lane)
  const bool is_neighbor_to_ego_lane =
      std::abs(ptr_agent_lane->get_virtual_id() - ego_lane->get_virtual_id()) ==
      1;
  const auto& agent_lane_coord = ptr_agent_lane->get_lane_frenet_coord();
  if (!agent_lane_coord->XYToSL(agent.x(), agent.y(), &agent_s, &agent_l)) {
    return false;
  }
  const auto agent_lane_point = agent_lane_coord->GetPathPointByS(agent_s);
  const double heading_diff = std::fabs(
      planning_math::NormalizeAngle(agent.theta() - agent_lane_point.theta()));
  const bool heading_safe = heading_diff < kLowerHeadingDiff;
  const bool is_near_boundary =
      IsNeighborAgentNearBoundary(is_large_agent, is_neighbor_to_ego_lane,
                                  agent, ego_lane, virtual_lane_manager);
  const bool use_radical_param = is_parallel && is_neighbor_to_ego_lane &&
                                 heading_safe && !is_near_boundary;

  // std::cout << "use_radical_param:" << use_radical_param << ",is_parallel:"
  // << is_parallel
  //           << ",is_neighbor_to_ego_lane:" << is_neighbor_to_ego_lane
  //           << ",heading_safe:" << heading_safe << ",is_near_boundary:" <<
  //           is_near_boundary
  //           << std::endl;

  const double k_ego_preview_time = use_radical_param ? 0.0 : kEgoPreTimeThd;
  if (agent_s < ego_s + init_point.vel() * k_ego_preview_time) {
    // std::cout << " return error2,k_ego_preview_time:" << k_ego_preview_time
    // << '\n';
    return false;
  }

  const auto& init_st_point = st_point_pairs.front();
  const double k_agent_ignore_time_thd =
      is_large_agent ? kLargeAgentIgnoreTimeThd : kNormalAgentIgnoreTimeThd;
  const double k_agent_radical_ignore_time_thd =
      StGraphUtils::IsLargeAgent(agent) ? kLargeAgentRadicalIgnoreTimeThd
                                        : kNormalAgentRadicalIgnoreTimeThd;

  if (!init_st_point.first.valid()) {
    return false;
  }

  // 1.when min_t > 4.0, ignore the agent's st boundary
  if (init_st_point.first.t() > k_agent_ignore_time_thd) {
    return true;
  }

  // 2.when min_t > 3.5 and the neighbor agent has no cut-in behavior, ignore
  // the agent's st boundary
  if (use_radical_param &&
      init_st_point.first.t() > k_agent_radical_ignore_time_thd) {
    return true;
  }

  // std::cout << "return error3\n";
  return false;
}

bool StGraphUtils::IsLargeAgentByLength(const agent::Agent& agent) {
  return (agent::AgentType::BUS == agent.type() ||
          agent::AgentType::TRUCK == agent.type()) &&
         (agent.length() > kLargeAgentLengthM);
}

bool StGraphUtils::CheckLateralFarCutinAgent(
    const agent::Agent& agent,
    const std::vector<std::pair<STPoint, STPoint>> st_point_pairs,
    const std::shared_ptr<StGraphInput>& st_graph_input) {
  constexpr double kSTProjectInitTime = 3.0;
  constexpr double kEgoPreTimeThd = 1.5;

  if (agent.is_cutin() || agent.is_rule_base_cutin() ||
      agent.is_prediction_cutin()) {
    return false;
  }

  if (st_point_pairs.empty()) {
    return false;
  }

  const auto planned_kd_path = st_graph_input->processed_path();
  const auto& init_point = st_graph_input->planning_init_point();
  if (planned_kd_path == nullptr) {
    return false;
  }
  double ego_s, ego_l;
  double agent_s, agent_l;
  if (!planned_kd_path->XYToSL(init_point.x(), init_point.y(), &ego_s,
                               &ego_l)) {
    return false;
  }
  if (!planned_kd_path->XYToSL(agent.x(), agent.y(), &agent_s, &agent_l)) {
    return false;
  }

  if (agent_s < ego_s + init_point.vel() * kEgoPreTimeThd) {
    return false;
  }

  const auto init_st_point = st_point_pairs.front();
  if (init_st_point.first.valid() &&
      init_st_point.first.t() < kSTProjectInitTime) {
    return false;
  }

  return true;
}

bool StGraphUtils::CheckLateralFarCutinAgentIsLonSafe(
    const agent::Agent& agent,
    const std::vector<std::pair<STPoint, STPoint>> st_point_pairs,
    const trajectory::TrajectoryPoint& planning_init_point,
    const std::shared_ptr<StGraphInput>& st_graph_input) {
  constexpr double dt = 0.1;
  constexpr int32_t plan_num = 51;
  constexpr double kLonSafeBuffer = 2.0;
  if (st_point_pairs.empty()) {
    return false;
  }
  const auto init_st_point = st_point_pairs.front();

  auto max_deceleration_curve_by_agent_vel =
      StGraphUtils::GenerateMaxDecelerationCurveByAgentVel(agent.speed(),
                                                           planning_init_point);

  bool is_lon_safe = false;

  for (int32_t i = 0; i < plan_num; ++i) {
    double relative_t = i * dt;
    double s = max_deceleration_curve_by_agent_vel.Evaluate(0, relative_t);
    if (std::fabs(relative_t - init_st_point.first.t()) < 1e-3 &&
        s + kLonSafeBuffer < init_st_point.first.s()) {
      is_lon_safe = true;
    }
  }

  return is_lon_safe;
}

// ego_vel must larger than agent_vel
SecondOrderTimeOptimalTrajectory
StGraphUtils::GenerateMaxDecelerationCurveByAgentVel(
    const double agent_vel, const trajectory::TrajectoryPoint& init_point) {
  LonState init_state;
  init_state.p = init_point.s();
  init_state.v = init_point.vel();
  init_state.a = init_point.acc();

  StateLimit state_limit;

  const double acc_upper_bound = 2.0;
  const double acc_lower_bound = -1.0;
  const double jerk_upper_bound = 5.0;
  const double jerk_lower_bound = -2.0;

  // constexpr double kSlowAccLowerBound = -3.0;
  // state_limit.v_end = agent_vel;
  state_limit.v_end = 0.0;
  state_limit.a_max = acc_upper_bound;
  state_limit.a_min = acc_lower_bound;
  state_limit.j_max = jerk_upper_bound;
  state_limit.j_min = jerk_lower_bound;

  return SecondOrderTimeOptimalTrajectory(init_state, state_limit);
}

bool StGraphUtils::IsBoundaryAboveRearTargetBoundary(
    const STBoundary& st_boundary, const STBoundary* rear_st_boundary) {
  if (nullptr == rear_st_boundary) {
    return true;
  }
  const auto& bottom_left = rear_st_boundary->bottom_left_point();
  double s_lower = 0.0;
  double s_upper = 0.0;
  for (const auto& pnt : st_boundary.upper_points()) {
    if (pnt.t() >= rear_st_boundary->max_t()) {
      break;
    }
    if (pnt.t() <= rear_st_boundary->min_t()) {
      if (pnt.s() > bottom_left.s()) {
        return true;
      }
    } else {
      if (!rear_st_boundary->GetBoundarySRange(pnt.t(), &s_lower, &s_upper)) {
        continue;
      }
      if (pnt.s() > s_lower) {
        return true;
      }
    }
  }
  return false;
}

planning_math::Box2d StGraphUtils::MakeEgoBox(
    const std::shared_ptr<planning_math::KDPath>& planned_kd_path,
    const double s) {
  const auto& param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double vehicle_length = param.length;
  const double vehicle_width = param.width;
  const double back_axle_to_center_dist = param.rear_axle_to_center;
  auto point = planned_kd_path->GetPathPointByS(s);
  Vec2d center =
      point + Vec2d::CreateUnitVec2d(point.theta()) * back_axle_to_center_dist;
  Box2d ego_box(center, point.theta(), vehicle_length, vehicle_width);
  return ego_box;
}

}  // namespace speed
}  // namespace planning