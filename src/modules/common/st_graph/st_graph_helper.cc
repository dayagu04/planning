#include "st_graph_helper.h"

#include "st_graph_utils.h"

namespace planning {
namespace speed {

namespace {

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

}  // namespace speed
}  // namespace planning