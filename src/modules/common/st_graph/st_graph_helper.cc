#include "st_graph_helper.h"

#include "st_graph/st_point.h"
#include "st_graph_utils.h"

namespace planning {
namespace speed {

namespace {

using namespace planning_math;

constexpr double kMaxPathLength = 400.0;
constexpr double kMathEpsilon = 1e-10;
constexpr double kTimeResolution = 0.2;
// constexpr bool kEnablePlotLowSpeedAgentOriginStBoundary = false;
}  // namespace

StGraphHelper::StGraphHelper(const STGraph& st_graph) : st_graph_(st_graph) {}

const bool StGraphHelper::GetStBoundary(const int64_t boundary_id,
                                        STBoundary* const st_boundary) const {
  const auto& boundary_id_st_boundaries_map =
      st_graph_.boundary_id_st_boundaries_map();
  const auto it = boundary_id_st_boundaries_map.find(boundary_id);
  if (it == boundary_id_st_boundaries_map.end()) {
    return false;
  }
  *st_boundary = *(it->second.get());
  return true;
}

const bool StGraphHelper::GetAgentStBoundaries(
    const int32_t agent_id, std::vector<int64_t>* const st_boundaries) const {
  const auto& agent_id_st_boundaries_map =
      st_graph_.agent_id_st_boundaries_map();
  const auto it = agent_id_st_boundaries_map.find(agent_id);
  if (it == agent_id_st_boundaries_map.end()) {
    return false;
  }
  st_boundaries->reserve(it->second.size());
  *st_boundaries = it->second;
  return true;
}

const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
StGraphHelper::GetAllStBoundaries() const {
  return st_graph_.boundary_id_st_boundaries_map();
}

void StGraphHelper::DetermineIfConeBucketCIPV(
    const std::vector<std::shared_ptr<agent::Agent>>& agents) {
  std::vector<const planning::agent::Agent*> speed_limited_cone_buckets;
  speed_limited_cone_buckets.reserve(agents.size());
  std::unordered_map<int64_t, std::unique_ptr<STBoundary>>
      cone_boundary_id_st_boundaries_map;
  for (const auto agent : agents) {
    if (agent == nullptr) {
      continue;
    }
    if (!agent->need_speed_limit()) {
      continue;
    }
    MakeSpeedLimitedConeBucketStBoundary(*agent,
                                         cone_boundary_id_st_boundaries_map);
  }
  if (cone_boundary_id_st_boundaries_map.empty()) {
    return;
  }
  int64_t closest_cone_boundary_id = -1, closest_st_boundary_id = -1;
  double closest_cone_s = std::numeric_limits<double>::max();
  double closest_st_s = std::numeric_limits<double>::max();
  StGraphUtils::DetermineClosetStBoundary(cone_boundary_id_st_boundaries_map,
                                          closest_cone_boundary_id,
                                          closest_cone_s);
  StGraphUtils::DetermineClosetStBoundary(
      st_graph_.boundary_id_st_boundaries_map(), closest_st_boundary_id,
      closest_st_s);
  bool is_cone_bucket_cipv = false;
  if (closest_cone_boundary_id != -1) {
    if (closest_st_boundary_id != -1) {
      if (closest_cone_s < closest_st_s) {
        is_cone_bucket_cipv = true;
      }
    } else {
      is_cone_bucket_cipv = true;
    }
  }
  if (is_cone_bucket_cipv) {
    const auto agent_id = closest_cone_boundary_id >> 8;
    for (auto agent : agents) {
      if (agent == nullptr) {
        continue;
      }
      if (agent->agent_id() != agent_id) {
        continue;
      }
      agent->set_is_cone_bucket_cipv(true);
      break;
    }
  }
}

bool StGraphHelper::GetBorderByStPoint(double s, double t,
                                       STPoint* const lower_st_point,
                                       STPoint* const upper_st_point) const {
  const auto& path_range = st_graph_.path_range();
  const auto& time_range = st_graph_.time_range();
  const auto& st_points_table = st_graph_.st_points_table();
  if (s < path_range.first - kMathEpsilon ||
      s > path_range.second + kMathEpsilon) {
    return false;
  }

  if (t < time_range.first - kMathEpsilon ||
      t > time_range.second + kMathEpsilon) {
    return false;
  }

  const int index = int((t - time_range.first) / kTimeResolution + 0.51);
  if (index < 0 || index >= st_points_table.size()) {
    lower_st_point->set_s(-kMaxPathLength);
    lower_st_point->set_t(t);
    upper_st_point->set_s(kMaxPathLength);
    upper_st_point->set_t(t);
    return false;
  }
  const auto& intervals = st_points_table.at(index);
  if (intervals.empty()) {
    lower_st_point->set_s(-kMaxPathLength);
    lower_st_point->set_t(t);
    upper_st_point->set_s(kMaxPathLength);
    upper_st_point->set_t(t);
    return true;
  }

  if (s < intervals.front().first.s()) {
    lower_st_point->set_s(-kMaxPathLength);
    lower_st_point->set_t(t);
    *upper_st_point = intervals.front().first;
    return true;
  }

  if (s > intervals.back().second.s()) {
    *lower_st_point = intervals.back().second;
    upper_st_point->set_s(kMaxPathLength);
    upper_st_point->set_t(t);
    return true;
  }

  if (intervals.size() == 1) {
    const auto interval = intervals.front();
    if ((s > interval.first.s() + kMathEpsilon &&
         s < interval.second.s() - kMathEpsilon)) {
      return false;
    } else if (s > interval.second.s() - kMathEpsilon) {
      *lower_st_point = interval.second;
      upper_st_point->set_s(kMaxPathLength);
      upper_st_point->set_t(t);
      return true;
    } else if (s < interval.first.s() + kMathEpsilon) {
      *upper_st_point = interval.first;
      lower_st_point->set_s(-kMaxPathLength);
      lower_st_point->set_t(t);
      return true;
    }
  } else {
    for (int i = 0; i < intervals.size() - 1; ++i) {
      if ((s > intervals.at(i).first.s() + kMathEpsilon &&
           s < intervals.at(i).second.s() - kMathEpsilon) ||
          (s > intervals.at(i + 1).first.s() + kMathEpsilon &&
           s < intervals.at(i + 1).second.s() - kMathEpsilon)) {
        return false;
      }
      if (s > intervals.at(i).second.s() - kMathEpsilon &&
          s < intervals.at(i + 1).first.s() + kMathEpsilon) {
        *lower_st_point = intervals.at(i).second;
        *upper_st_point = intervals.at(i + 1).first;
        return true;
      }
    }
  }
  return true;
}

void StGraphHelper::MakeSpeedLimitedConeBucketStBoundary(
    const agent::Agent& agent,
    std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
        boundary_id_st_boundaries_map) {
  const auto& st_graph_input = st_graph_.st_graph_input();
  const auto* path_border_querier = st_graph_input->path_border_querier();
  const int32_t reserve_num = st_graph_.reserve_num();
  const auto planned_kd_path = st_graph_input->processed_path();
  const auto& planning_init_point_box =
      st_graph_input->planning_init_point_box();
  const bool is_rads_scene = st_graph_input->is_rads_scene();
  if (nullptr == planned_kd_path || nullptr == path_border_querier ||
      reserve_num <= 0) {
    return;
  }
  const auto& time_range = st_graph_.time_range();
  const auto& path_range = st_graph_.path_range();
  const double lat_buffer = st_graph_input->lat_buffer();
  auto obs_box = agent.box();
  obs_box.LateralExtend(lat_buffer * 2.0);
  // max_s min_s max_l min_l
  std::vector<double> agent_sl_boundary(4);
  std::vector<std::pair<int32_t, Vec2d>> considered_corners;
  StGraphUtils::CalculateAgentSLBoundary(
      planned_kd_path, obs_box, path_range, StBoundaryType::NORMAL,
      &agent_sl_boundary, &considered_corners);

  std::vector<std::pair<STPoint, STPoint>> st_point_pairs;
  st_point_pairs.reserve(reserve_num);
  double lower_s = std::numeric_limits<double>::max();
  double upper_s = std::numeric_limits<double>::lowest();
  const int64_t boundary_id = (agent.agent_id() << 8) + 0;

  if (StGraphUtils::CalculateSRange(planned_kd_path, *path_border_querier,
                                    obs_box, StBoundaryType::NORMAL, path_range,
                                    agent_sl_boundary, considered_corners,
                                    planning_init_point_box, &lower_s, &upper_s,
                                    is_rads_scene)) {
    for (double t = time_range.first; t < time_range.second;
         t += kTimeResolution) {
      st_point_pairs.emplace_back(
          STPoint(lower_s, t, agent.agent_id(), boundary_id, 0.0, 0.0),
          STPoint(upper_s, t, agent.agent_id(), boundary_id, 0.0, 0.0));
    }
  }
  if (!st_point_pairs.empty()) {
    std::unique_ptr<STBoundary> st_boundary(new STBoundary(st_point_pairs));
    st_boundary->set_id(boundary_id);
    boundary_id_st_boundaries_map.insert(
        std::make_pair(boundary_id, std::move(st_boundary)));
  }
}

bool StGraphHelper::GetFirstNeighborUpperBound(
    STPoint* const upper_point) const {
  const auto& neighbor_corridor = st_graph_.neighbor_corridor();
  const int32_t first_neighbor_yield_index =
      st_graph_.first_neighbor_yield_index();
  if (first_neighbor_yield_index >= neighbor_corridor.size() ||
      first_neighbor_yield_index < 0) {
    return false;
  }
  if (neighbor_corridor.at(first_neighbor_yield_index).first.agent_id() ==
      kNoAgentId) {
    return false;
  }
  *upper_point = neighbor_corridor.at(first_neighbor_yield_index).first;
  return true;
}

bool StGraphHelper::GetFirstNeighborLowerBound(
    STPoint* const lower_point) const {
  const auto& neighbor_corridor = st_graph_.neighbor_corridor();
  const int32_t first_neighbor_overtake_index =
      st_graph_.first_neighbor_overtake_index();
  if (first_neighbor_overtake_index >= neighbor_corridor.size() ||
      first_neighbor_overtake_index < 0) {
    return false;
  }
  if (neighbor_corridor.at(first_neighbor_overtake_index).second.agent_id() ==
      kNoAgentId) {
    return false;
  }
  *lower_point = neighbor_corridor.at(first_neighbor_overtake_index).second;
  return true;
}

const std::unordered_map<int32_t, std::vector<int64_t>>&
StGraphHelper::GetAgentIdSTBoundariesMap() const {
  return st_graph_.agent_id_st_boundaries_map();
}

const STPoint StGraphHelper::GetPassCorridorUpperBound(const double t) const {
  const auto& time_range = st_graph_.time_range();
  const auto& st_pass_corridor = st_graph_.st_pass_corridor();
  STPoint init_upper_point;
  init_upper_point.set_agent_id(kNoAgentId);
  init_upper_point.set_s(std::numeric_limits<double>::max());
  init_upper_point.set_velocity(std::numeric_limits<double>::max());
  init_upper_point.set_acceleration(std::numeric_limits<double>::max());
  if (!IsTimeInRange(t)) {
    return init_upper_point;
  }
  int32_t index = int32_t((t - time_range.first) / kTimeResolution);
  if (index < 0 || index >= st_pass_corridor.size()) {
    return init_upper_point;
  }
  return st_pass_corridor.at(index).first;
}

const STPoint StGraphHelper::GetPassCorridorLowerBound(const double t) const {
  const auto& time_range = st_graph_.time_range();
  const auto& st_pass_corridor = st_graph_.st_pass_corridor();
  STPoint init_lower_point;
  init_lower_point.set_agent_id(kNoAgentId);
  init_lower_point.set_s(std::numeric_limits<double>::lowest());
  init_lower_point.set_velocity(std::numeric_limits<double>::lowest());
  init_lower_point.set_acceleration(std::numeric_limits<double>::lowest());
  if (!IsTimeInRange(t)) {
    return init_lower_point;
  }
  int32_t index = int32_t((t - time_range.first) / kTimeResolution);
  if (index < 0 || index >= st_pass_corridor.size()) {
    return init_lower_point;
  }
  return st_pass_corridor.at(index).second;
}

const STPoint StGraphHelper::GetSoftPassCorridorUpperBound(
    const double t) const {
  const auto& time_range = st_graph_.time_range();
  STPoint init_upper_point;
  init_upper_point.set_agent_id(kNoAgentId);
  init_upper_point.set_s(std::numeric_limits<double>::max());
  init_upper_point.set_velocity(std::numeric_limits<double>::max());
  init_upper_point.set_acceleration(std::numeric_limits<double>::max());
  if (!IsTimeInRange(t)) {
    return init_upper_point;
  }
  const auto& boundary_id_st_boundaries_map =
      st_graph_.boundary_id_st_boundaries_map();
  for (const auto& pair : boundary_id_st_boundaries_map) {
    const auto& boundary_id = pair.first;
    const auto& boundary = pair.second;
    if (boundary->decision_type() != STBoundary::DecisionType::CAUTION_YIELD) {
      continue;
    }
    if (t < boundary->min_t() || t > boundary->max_t()) {
      continue;
    }
    STPoint lower_point, upper_point;
    if (!boundary->GetBoundaryBounds(t, &lower_point, &upper_point)) {
      continue;
    }
    if (lower_point.s() < init_upper_point.s()) {
      init_upper_point = lower_point;
    }
  }
  return init_upper_point;
}

const std::pair<double, double> StGraphHelper::GetPathRange() const {
  return st_graph_.path_range();
}

const std::pair<double, double> StGraphHelper::GetTimeRange() const {
  return st_graph_.time_range();
}

bool StGraphHelper::IsTimeInRange(double t) const {
  const auto& time_range = st_graph_.time_range();
  return !(t < time_range.first || t > time_range.second);
}

const int32 StGraphHelper::GetFirstNeighborYieldAgentId() const {
  return st_graph_.first_neighbor_yield_agent_id();
}

}  // namespace speed
}  // namespace planning