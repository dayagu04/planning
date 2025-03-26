#include "sample_space_base.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "st_graph/st_point.h"
#include "vec2d.h"

using planning::speed::STPoint;

namespace planning {

STSampleSpaceBase::STSampleSpaceBase(
    const std::vector<AgentInfo>& lane_change_veh_info, const double init_s,
    const double front_edge_to_rear_axle, const double rear_edge_to_rear_axle) {
  Clear();
  init_s_ = init_s;
  for (auto& agent : lane_change_veh_info) {
    LinearExtendAgentStBoundary(agent);
  }

  front_edge_to_rear_axle_ = front_edge_to_rear_axle;
  rear_edge_to_rear_axle_ = rear_edge_to_rear_axle;
}

STSampleSpaceBase::STSampleSpaceBase(const double front_edge_to_rear_axle,
                                     const double rear_edge_to_rear_axle) {
  front_edge_to_rear_axle_ = front_edge_to_rear_axle;
  rear_edge_to_rear_axle_ = rear_edge_to_rear_axle;
}

void STSampleSpaceBase::Init(const std::vector<AgentInfo>& lane_change_veh_info,
                             const double init_s) {
  Clear();
  init_s_ = init_s;
  for (auto& agent : lane_change_veh_info) {
    if (agent.v < kStaticObjVel) {
      continue;
    }
    LinearExtendAgentStBoundary(agent);
  }
  ConstructStPointsTable();
}

void STSampleSpaceBase::Clear() {
  agents_st_point_paris_.clear();
  st_points_table_.clear();
  sample_points_.clear();
}

void STSampleSpaceBase::SetInitS(const double s0) { init_s_ = s0; }

const double STSampleSpaceBase::init_s() const { return init_s_; }

void STSampleSpaceBase::LinearExtendAgentStBoundary(const AgentInfo& agent) {
  std::vector<std::pair<STPoint, STPoint>> st_points_pairs;
  st_points_pairs.reserve(kSampleSpaceReserveNum);
  const double agent_s_start = init_s_ + agent.center_s;
  const double& v = agent.v;
  const double& agent_half_length = agent.half_length;
  const size_t TimeHorizion = kPlanningDuration / kPlanningStep;
  for (size_t i = 0; i < TimeHorizion; ++i) {
    const double time_start = i * kPlanningStep;
    double time_end = (i + 1) * kPlanningStep;
    if (i == TimeHorizion - 1) {
      time_end = kPlanningDuration + 2 * planning_math::kMathEpsilon;
    }
    for (double t = time_start; t < time_end - planning_math::kMathEpsilon;
         t += kTimeResolution) {
      double s = agent_s_start + t * v;
      STPoint lower_point(s - agent_half_length, t);
      STPoint upper_point(s + agent_half_length, t);
      lower_point.set_agent_id(agent.id);
      upper_point.set_agent_id(agent.id);
      lower_point.set_velocity(v);
      upper_point.set_velocity(v);
      st_points_pairs.emplace_back(std::move(lower_point),
                                   std::move(upper_point));
    }
  }
  agents_st_point_paris_.emplace_back(std::move(st_points_pairs));
}

void STSampleSpaceBase::ConstructStPointsTable() {
  st_points_table_.clear();
  st_points_table_.resize(kSampleSpaceReserveNum);
  sample_points_.clear();

  for (const std::vector<std::pair<STPoint, STPoint>>& st_point_paris :
       agents_st_point_paris_) {
    for (size_t i = 0; i < st_point_paris.size(); ++i) {
      int index = (st_point_paris[i].first.t()) / kTimeResolution;
      STPoint lower_point(st_point_paris[i].first.s(),
                          st_point_paris[i].first.t(),
                          st_point_paris[i].first.agent_id(), -1,
                          st_point_paris[i].first.velocity(), 0.0);
      STPoint upper_point(st_point_paris[i].second.s(),
                          st_point_paris[i].second.t(),
                          st_point_paris[i].second.agent_id(), -1,
                          st_point_paris[i].second.velocity(), 0.0);
      std::pair<STPoint, STPoint> points_pair(std::move(lower_point),
                                              std::move(upper_point));
      st_points_table_[index].emplace_back(std::move(points_pair));
    }
  }

  auto cmp = [](const std::pair<STPoint, STPoint> ele1,
                const std::pair<STPoint, STPoint> ele2) {
    return ele1.first.s() < ele2.first.s();
  };

  for (auto& intervals : st_points_table_) {
    if (intervals.empty()) {
      continue;
    }
    std::sort(intervals.begin(), intervals.end(), cmp);
    // merge intervals
    std::vector<std::pair<STPoint, STPoint>> merged_intervals;
    for (int i = 0; i < intervals.size(); ++i) {
      double L = intervals[i].first.s(), R = intervals[i].second.s();
      if (merged_intervals.empty() || merged_intervals.back().second.s() < L) {
        merged_intervals.emplace_back(intervals[i]);
      } else {
        if (merged_intervals.back().second.s() < R) {
          merged_intervals.back().second = intervals[i].second;
        }
      }
    }
    intervals = merged_intervals;
  }
  return;
}

bool find_boundary_pair(
    const std::vector<std::pair<STPoint, STPoint>>& st_pairs, double s,
    std::pair<STPoint, STPoint>& result_pair1,
    std::pair<STPoint, STPoint>& result_pair2) {
  if (st_pairs.empty()) {
    return false;
  }

  int left = 0;
  int right = st_pairs.size() - 1;

  // Case 1: s is smaller than the smallest interval's start
  if (s < st_pairs[0].first.s()) {
    result_pair1 = st_pairs[0];
    result_pair1.first.set_agent_id(kNoAgentId);
    result_pair1.second.set_agent_id(kNoAgentId);
    result_pair2 = st_pairs[0];  // Single interval is returned
    return true;
  }

  // Case 2: s is larger than the largest interval's end
  if (s > st_pairs[right].second.s()) {
    result_pair1 = st_pairs[right];
    result_pair2 = st_pairs[right];  // Single interval is returned
    result_pair2.first.set_agent_id(kNoAgentId);
    result_pair2.second.set_agent_id(kNoAgentId);
    return true;
  }
  while (left <= right) {
    int mid = left + (right - left) / 2;

    const auto& current_pair = st_pairs[mid];

    // Check if s is within the current interval
    if (s >= current_pair.first.s() && s <= current_pair.second.s()) {
      // Find the closest neighbor (either the previous or next pair)
      if (mid - 1 >= 0 && mid + 1 < st_pairs.size()) {
        // Compare distance to the previous and next pair and choose the closest
        double dist_to_prev = std::fabs(s - st_pairs[mid - 1].second.s());
        double dist_to_next = std::fabs(s - st_pairs[mid + 1].first.s());

        if (dist_to_prev < dist_to_next) {
          result_pair1 = st_pairs[mid - 1];
          result_pair2 = current_pair;
        } else {
          result_pair1 = current_pair;
          result_pair2 = st_pairs[mid + 1];
        }
      } else if (mid - 1 >= 0) {
        result_pair2 = current_pair;  // Only check the previous pair
        result_pair1 = st_pairs[mid - 1];
      } else if (mid + 1 < st_pairs.size()) {
        result_pair2 = st_pairs[mid + 1];  // Only check the next pair
        result_pair1 = current_pair;
      } else {
        result_pair1 = current_pair;
        result_pair2 = current_pair;
      }

      return true;
    }

    // Check if s is between the current pair and the next pair
    if (mid + 1 < st_pairs.size() && s > current_pair.second.s() &&
        s < st_pairs[mid + 1].first.s()) {
      result_pair1 = current_pair;
      result_pair2 = st_pairs[mid + 1];
      return true;
    }

    // Binary search adjustment
    if (s < current_pair.first.s()) {
      right = mid - 1;
    } else {
      left = mid + 1;
    }
  }

  // If no valid boundary is found
  return false;
}

bool STSampleSpaceBase::GetBorderByAvailable(double s, double t,
                                             STPoint* const lower_st_point,
                                             STPoint* const upper_st_point) {
  const int index = int((t / kTimeResolution) + 0.51);
  if (index < 0 || index >= st_points_table_.size()) {
    lower_st_point->set_info(-kMaxPathLength, t, 0.0, kNoAgentId, -1);
    upper_st_point->set_info(kMaxPathLength, t, 100.0, kNoAgentId, -1);
    return false;
  }

  const auto& intervals = st_points_table_.at(index);
  if (intervals.empty()) {
    lower_st_point->set_info(-kMaxPathLength, t, 0.0, kNoAgentId, -1);
    upper_st_point->set_info(kMaxPathLength, t, 100.0, kNoAgentId, -1);
    return true;
  }

  if (s + front_edge_to_rear_axle_ < intervals.front().first.s()) {
    lower_st_point->set_info(-kMaxPathLength, t, 0.0, kNoAgentId, -1);
    upper_st_point->set_info(intervals.front().first.s(), t,
                             intervals.front().first.velocity(),
                             intervals.front().first.agent_id(), -1);
    return true;
  }

  if (s - rear_edge_to_rear_axle_ > intervals.back().second.s()) {
    lower_st_point->set_info(intervals.back().second.s(), t,
                             intervals.back().second.velocity(),
                             intervals.back().second.agent_id(), -1);
    upper_st_point->set_info(kMaxPathLength, t, 100.0, kNoAgentId, -1);
    return true;
  }

  if (intervals.size() == 1) {
    const auto interval = intervals.front();
    std::pair<double, double> ego_lon_area{s - rear_edge_to_rear_axle_,
                                           s + front_edge_to_rear_axle_};
    std::pair<double, double> obj_lon_area{interval.first.s(),
                                           interval.second.s()};

    if (std::fmax(ego_lon_area.first, obj_lon_area.first) <=
        std::fmin(ego_lon_area.second, obj_lon_area.second)) {
      upper_st_point->set_info(interval.second.s(), t,
                               interval.second.velocity(),
                               interval.second.agent_id(), -1);
      lower_st_point->set_info(interval.first.s(), t, interval.first.velocity(),
                               interval.first.agent_id(), -1);
      return false;
    } else if (ego_lon_area.first> interval.second.s()) {
      lower_st_point->set_info(interval.second.s(), t,
                               interval.second.velocity(),
                               interval.second.agent_id(), -1);
      upper_st_point->set_info(kMaxPathLength, t, 100.0, kNoAgentId, -1);
      return true;
    } else if (s + front_edge_to_rear_axle_ < interval.first.s()) {
      upper_st_point->set_info(interval.first.s(), t, interval.first.velocity(),
                               interval.first.agent_id(), -1);
      lower_st_point->set_info(-kMaxPathLength, t, 0.0, kNoAgentId, -1);
      return true;
    } else {
      upper_st_point->set_info(interval.second.s(), t,
                               interval.second.velocity(),
                               interval.second.agent_id(), -1);
      lower_st_point->set_info(interval.first.s(), t, interval.first.velocity(),
                               interval.first.agent_id(), -1);
      return true;
    }
  } else {
    std::pair<STPoint, STPoint> front_interval, back_interval;
    if (find_boundary_pair(intervals, s, front_interval, back_interval)) {
      upper_st_point->set_info(back_interval.first.s(), t,
                               back_interval.first.velocity(),
                               back_interval.first.agent_id(), -1);
      lower_st_point->set_info(front_interval.second.s(), t,
                               front_interval.second.velocity(),
                               front_interval.second.agent_id(), -1);

    } else {
      upper_st_point->set_info(kMaxPathLength, t, 100.0, kNoAgentId, -1);
      lower_st_point->set_info(-kMaxPathLength, t, 0.0, kNoAgentId, -1);
      std::cout << "find no boundary pair!" << std::endl;
    }
  }

  return true;
}

}  // namespace planning
