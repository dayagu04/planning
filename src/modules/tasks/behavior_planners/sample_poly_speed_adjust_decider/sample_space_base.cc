#include "sample_space_base.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "st_graph/st_point.h"
#include "st_graph/st_point_with_lateral.h"
#include "vec2d.h"

namespace planning {
using planning::planning_data::DynamicAgentNode;
using planning::speed::STPoint;
using planning::speed::STPointWithLateral;
STSampleSpaceBase::STSampleSpaceBase(
    const std::vector<const DynamicAgentNode*>& target_lane_nodes,
    const double init_s, const double front_edge_to_rear_axle,
    const double rear_edge_to_rear_axle) {
  Clear();
  init_s_ = init_s;
  for (auto& agent : target_lane_nodes) {
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

void STSampleSpaceBase::Init(
    const std::vector<const DynamicAgentNode*>& target_lane_nodes,
    const double init_s,
    const std::shared_ptr<planning::planning_math::KDPath>& target_lane_coord,
    int change_direction,const double sample_st_limit_lat_offset) {
  Clear();
  init_s_ = init_s;
  change_direction_ = change_direction;
  for (const auto* agent_node : target_lane_nodes) {
    if (!target_lane_coord) {
      LinearExtendAgentStBoundary(agent_node);
    } else {
      LinearExtendAgentStBoundaryWithFrenet(agent_node,target_lane_coord);
    }
  }
  ConstructStPointsTable(sample_st_limit_lat_offset);
}

void STSampleSpaceBase::Clear() {
  agents_st_point_paris_.clear();
  st_points_table_.clear();
  sample_points_.clear();
  gap_array_.clear();
}

void STSampleSpaceBase::SetInitS(const double s0) { init_s_ = s0; }

const double STSampleSpaceBase::init_s() const { return init_s_; }

void STSampleSpaceBase::LinearExtendAgentStBoundary(
    const DynamicAgentNode* agent_node) {
  const auto& primary_trajectories =
      agent_node->node_trajectories_used_by_st_graph();
  const auto& fallback_trajectories = agent_node->node_trajectories();
  const auto& selected_trajectories = primary_trajectories.empty()
                                          ? fallback_trajectories
                                          : primary_trajectories;
  if (selected_trajectories.empty() || selected_trajectories.front().empty()) {
    return;
  }
  const auto& current_trajectory = selected_trajectories.front();
  std::vector<std::pair<STPointWithLateral, STPointWithLateral>> st_points_pairs;
  st_points_pairs.reserve(kSampleSpaceReserveNum);
  const double agent_s_start = agent_node->node_s();
  const double agent_half_length = agent_node->node_length() / 2.0;
  const size_t TimeHorizion = kPlanningDuration / kPlanningStep;
  const size_t PredictionHorizon = current_trajectory.size() - 1;
  size_t min_horizion = std::min(TimeHorizion, PredictionHorizon);
  double end_v = 0.0;
  double end_s = 0.0;
  for (size_t i = 0; i <= min_horizion; ++i) {
    double t = i * kPlanningStep;
    end_s = agent_s_start + current_trajectory[i].s();
    end_v = current_trajectory[i].vel();
    double acc = current_trajectory[i].acc();
    STPointWithLateral lower_point(end_s - agent_half_length, t);
    STPointWithLateral upper_point(end_s + agent_half_length, t);
    lower_point.set_type(agent_node->type());
    upper_point.set_type(agent_node->type());
    lower_point.set_agent_id(agent_node->node_agent_id());
    upper_point.set_agent_id(agent_node->node_agent_id());
    lower_point.set_velocity(end_v);
    upper_point.set_velocity(end_v);
    lower_point.set_acceleration(acc);
    upper_point.set_acceleration(acc);
    lower_point.set_vehicle_length(2 * agent_half_length);
    upper_point.set_vehicle_length(2 * agent_half_length);
    st_points_pairs.emplace_back(std::move(lower_point),
                                 std::move(upper_point));
    if (i == min_horizion) {
      break;
    }
    t += kTimeResolution;
    end_s += kTimeResolution * end_v;
    end_v += kTimeResolution * acc;
    lower_point.set_t(t);
    upper_point.set_t(t);
    lower_point.set_type(agent_node->type());
    upper_point.set_type(agent_node->type());
    lower_point.set_s(end_s - agent_half_length);
    upper_point.set_s(end_s + agent_half_length);
    lower_point.set_agent_id(agent_node->node_agent_id());
    upper_point.set_agent_id(agent_node->node_agent_id());
    lower_point.set_velocity(end_v);
    upper_point.set_velocity(end_v);
    lower_point.set_acceleration(acc);
    upper_point.set_acceleration(acc);
    lower_point.set_vehicle_length(2 * agent_half_length);
    upper_point.set_vehicle_length(2 * agent_half_length);
    st_points_pairs.emplace_back(std::move(lower_point),
                                 std::move(upper_point));
  }
  if (PredictionHorizon < TimeHorizion) {
    for (size_t i = PredictionHorizon; i < TimeHorizion; ++i) {
      const double time_start = i * kPlanningStep + kTimeResolution;
      double time_end = (i + 1) * kPlanningStep;
      for (double t = time_start; t < time_end + planning_math::kMathEpsilon;
           t += kTimeResolution) {
        end_s += end_v * kTimeResolution;
        STPointWithLateral lower_point(end_s - agent_half_length, t);
        STPointWithLateral upper_point(end_s + agent_half_length, t);
        lower_point.set_agent_id(agent_node->node_agent_id());
        upper_point.set_agent_id(agent_node->node_agent_id());
        lower_point.set_type(agent_node->type());
        upper_point.set_type(agent_node->type());
        lower_point.set_velocity(end_v);
        upper_point.set_velocity(end_v);
        lower_point.set_acceleration(0.0);
        upper_point.set_acceleration(0.0);
        lower_point.set_vehicle_length(2 * agent_half_length);
        upper_point.set_vehicle_length(2 * agent_half_length);
        st_points_pairs.emplace_back(std::move(lower_point),
                                     std::move(upper_point));
      }
    }
  }
  agents_st_point_paris_.emplace_back(std::move(st_points_pairs));
}

void STSampleSpaceBase::LinearExtendAgentStBoundaryWithFrenet(
    const DynamicAgentNode* agent_node,
    const std::shared_ptr<planning::planning_math::KDPath>& target_lane_coord) {
  const auto& primary_trajectories =
      agent_node->node_trajectories_used_by_st_graph();
  const auto& fallback_trajectories = agent_node->node_trajectories();
  const auto& selected_trajectories = primary_trajectories.empty()
                                          ? fallback_trajectories
                                          : primary_trajectories;
  if (selected_trajectories.empty() || selected_trajectories.front().empty()) {
    return;
  }
  const auto& current_trajectory = selected_trajectories.front();
  std::vector<std::pair<STPointWithLateral, STPointWithLateral>> st_points_pairs;
  st_points_pairs.reserve(kSampleSpaceReserveNum);
  const double agent_s_start = agent_node->node_s();
  const double agent_half_length = agent_node->node_length() / 2.0;
  const size_t TimeHorizion = kPlanningDuration / kPlanningStep;
  const size_t PredictionHorizon = current_trajectory.size() - 1;
  size_t min_horizion = std::min(TimeHorizion, PredictionHorizon);
  double end_v = 0.0;
  double end_s = 0.0;
  double frenet_l = 0.0;
  for (size_t i = 0; i <= min_horizion; ++i) {
    double t = i * kPlanningStep;
    Point2D cur_point_temp = {current_trajectory[i].x(),
                              current_trajectory[i].y()};
    Point2D frenet_point_temp;
    frenet_l = target_lane_coord->XYToSL(cur_point_temp, frenet_point_temp)
                   ? frenet_point_temp.y
                   : 0.0;
    end_s = agent_s_start + current_trajectory[i].s();
    end_v = current_trajectory[i].vel();
    double acc = current_trajectory[i].acc();
    STPointWithLateral lower_point(end_s - agent_half_length, t);
    STPointWithLateral upper_point(end_s + agent_half_length, t);
    lower_point.set_agent_id(agent_node->node_agent_id());
    upper_point.set_agent_id(agent_node->node_agent_id());
    lower_point.set_type(agent_node->type());
    upper_point.set_type(agent_node->type());
    lower_point.set_velocity(end_v);
    upper_point.set_velocity(end_v);
    lower_point.set_acceleration(acc);
    upper_point.set_acceleration(acc);
    lower_point.set_vehicle_length(2 * agent_half_length);
    upper_point.set_vehicle_length(2 * agent_half_length);
    lower_point.set_l(frenet_l);
    upper_point.set_l(frenet_l);
    st_points_pairs.emplace_back(std::move(lower_point),
                                 std::move(upper_point));
    if (i == min_horizion) {
      break;
    }
    t += kTimeResolution;
    end_s += kTimeResolution * end_v;
    end_v += kTimeResolution * acc;
    lower_point.set_t(t);
    upper_point.set_t(t);
    lower_point.set_type(agent_node->type());
    upper_point.set_type(agent_node->type());
    lower_point.set_s(end_s - agent_half_length);
    upper_point.set_s(end_s + agent_half_length);
    lower_point.set_l(frenet_l);
    upper_point.set_l(frenet_l);
    lower_point.set_agent_id(agent_node->node_agent_id());
    upper_point.set_agent_id(agent_node->node_agent_id());
    lower_point.set_velocity(end_v);
    upper_point.set_velocity(end_v);
    lower_point.set_acceleration(acc);
    upper_point.set_acceleration(acc);
    lower_point.set_vehicle_length(2 * agent_half_length);
    upper_point.set_vehicle_length(2 * agent_half_length);
    st_points_pairs.emplace_back(std::move(lower_point),
                                 std::move(upper_point));
  }
  if (PredictionHorizon < TimeHorizion) {
    for (size_t i = PredictionHorizon; i < TimeHorizion; ++i) {
      const double time_start = i * kPlanningStep + kTimeResolution;
      double time_end = (i + 1) * kPlanningStep;
      for (double t = time_start; t < time_end + planning_math::kMathEpsilon;
           t += kTimeResolution) {
        end_s += end_v * kTimeResolution;
        STPointWithLateral lower_point(end_s - agent_half_length, t);
        STPointWithLateral upper_point(end_s + agent_half_length, t);
        lower_point.set_agent_id(agent_node->node_agent_id());
        upper_point.set_agent_id(agent_node->node_agent_id());
        lower_point.set_velocity(end_v);
        upper_point.set_velocity(end_v);
        lower_point.set_type(agent_node->type());
        upper_point.set_type(agent_node->type());
        lower_point.set_acceleration(0.0);
        upper_point.set_acceleration(0.0);
        lower_point.set_vehicle_length(2 * agent_half_length);
        upper_point.set_vehicle_length(2 * agent_half_length);
        lower_point.set_l(frenet_l);
        upper_point.set_l(frenet_l);
        st_points_pairs.emplace_back(std::move(lower_point),
                                     std::move(upper_point));
      }
    }
  }
  agents_st_point_paris_.emplace_back(std::move(st_points_pairs));
}

void STSampleSpaceBase::ConstructStPointsTable(const double sample_st_limit_lat_offset) {
  st_points_table_.clear();
  st_points_table_.resize(kSampleSpaceReserveNum);
  sample_points_.clear();

  for (const std::vector<std::pair<STPointWithLateral, STPointWithLateral>>& st_point_paris :
       agents_st_point_paris_) {
    for (size_t i = 0; i < st_point_paris.size(); ++i) {
      int index = (st_point_paris[i].first.t()) / kTimeResolution;
      if (st_point_paris[i].first.l() * change_direction_ > sample_st_limit_lat_offset) {
        continue;
      }
      STPointWithLateral lower_point(st_point_paris[i].first.s(),
                          st_point_paris[i].first.t(),
                          st_point_paris[i].first.agent_id(), -1,
                          st_point_paris[i].first.velocity(),
                          st_point_paris[i].first.acceleration(),
                          st_point_paris[i].first.vehicle_length(),
                          st_point_paris[i].first.l()* change_direction_);
      STPointWithLateral upper_point(st_point_paris[i].second.s(),
                          st_point_paris[i].second.t(),
                          st_point_paris[i].second.agent_id(), -1,
                          st_point_paris[i].second.velocity(),
                          st_point_paris[i].second.acceleration(),
                          st_point_paris[i].first.vehicle_length(),
                          st_point_paris[i].second.l()* change_direction_);
      std::pair<STPointWithLateral, STPointWithLateral> points_pair(std::move(lower_point),
                                              std::move(upper_point));
      st_points_table_[index].emplace_back(std::move(points_pair));
    }
  }
  for (auto& current_st_point : st_points_table_) {
    std::sort(current_st_point.begin(), current_st_point.end(),
              [](std::pair<STPointWithLateral, STPointWithLateral>& a,
                 std::pair<STPointWithLateral, STPointWithLateral>& b)
                  -> bool { return a.first.s() < b.first.s(); });
  }

  auto cmp = [](const std::pair<STPointWithLateral, STPointWithLateral> ele1,
                const std::pair<STPointWithLateral, STPointWithLateral> ele2) {
    return ele1.first.s() < ele2.first.s();
  };

  for (auto& intervals : st_points_table_) {
    if (intervals.empty()) {
      continue;
    }
    std::sort(intervals.begin(), intervals.end(), cmp);
    // merge intervals
    std::vector<std::pair<STPointWithLateral, STPointWithLateral>> merged_intervals;
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
    const std::vector<std::pair<STPointWithLateral, STPointWithLateral>>&
        st_pairs,
    double s, double v,
    std::pair<STPointWithLateral, STPointWithLateral>& result_pair1,
    std::pair<STPointWithLateral, STPointWithLateral>& result_pair2) {
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
        double prev_gap_dist =
            std::fabs(st_pairs[mid].first.s() - st_pairs[mid - 1].second.s());
        double next_gap_dist =
            std::fabs(st_pairs[mid + 1].first.s() - st_pairs[mid].second.s());

        double speed_match_dist = v * (v - st_pairs[mid].first.velocity()) / 2 * 1.0;
        if((speed_match_dist + s) > st_pairs[mid].second.s()){
          result_pair1 = current_pair;
          result_pair2 = st_pairs[mid + 1];
        } else if((speed_match_dist + s) < st_pairs[mid].first.s()){
          result_pair1 = st_pairs[mid - 1];
          result_pair2 = current_pair;
        } else {
          if(prev_gap_dist < next_gap_dist){
            result_pair1 = current_pair;
            result_pair2 = st_pairs[mid + 1];
          } else {
            result_pair1 = st_pairs[mid - 1];
            result_pair2 = current_pair;
          }
        }
        // if (dist_to_prev < dist_to_next) {
        //   result_pair1 = st_pairs[mid - 1];
        //   result_pair2 = current_pair;
        // } else {
        //   result_pair1 = current_pair;
        //   result_pair2 = st_pairs[mid + 1];
        // }
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

bool STSampleSpaceBase::GetBorderByAvailable(
    double s, double v, double t, STPointWithLateral* const lower_st_point,
    STPointWithLateral* const upper_st_point) const {
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
    upper_st_point->set_acceleration(intervals.front().first.acceleration());
    upper_st_point->set_vehicle_length(intervals.front().first.vehicle_length());
    return true;
  }

  if (s - rear_edge_to_rear_axle_ > intervals.back().second.s()) {
    lower_st_point->set_info(intervals.back().second.s(), t,
                             intervals.back().second.velocity(),
                             intervals.back().second.agent_id(), -1);
    lower_st_point->set_acceleration(intervals.back().second.acceleration());
    lower_st_point->set_vehicle_length(intervals.back().first.vehicle_length());
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
      upper_st_point->set_acceleration(interval.second.acceleration());
      upper_st_point->set_vehicle_length(interval.second.vehicle_length());
      lower_st_point->set_info(interval.first.s(), t, interval.first.velocity(),
                               interval.first.agent_id(), -1);
      lower_st_point->set_acceleration(interval.first.acceleration());
      lower_st_point->set_vehicle_length(interval.first.vehicle_length());
      return false;
    } else if (ego_lon_area.first > interval.second.s()) {
      lower_st_point->set_info(interval.second.s(), t,
                               interval.second.velocity(),
                               interval.second.agent_id(), -1);
      lower_st_point->set_acceleration(interval.second.acceleration());
      lower_st_point->set_vehicle_length(interval.second.vehicle_length());
      upper_st_point->set_info(kMaxPathLength, t, 100.0, kNoAgentId, -1);
      return true;
    } else if (s + front_edge_to_rear_axle_ < interval.first.s()) {
      upper_st_point->set_info(interval.first.s(), t, interval.first.velocity(),
                               interval.first.agent_id(), -1);
      upper_st_point->set_acceleration(interval.first.acceleration());
      upper_st_point->set_vehicle_length(interval.first.vehicle_length());
      lower_st_point->set_info(-kMaxPathLength, t, 0.0, kNoAgentId, -1);
      return true;
    } else {
      upper_st_point->set_info(interval.second.s(), t,
                               interval.second.velocity(),
                               interval.second.agent_id(), -1);
      upper_st_point->set_acceleration(interval.second.acceleration());
      upper_st_point->set_vehicle_length(interval.second.vehicle_length());
      lower_st_point->set_info(interval.first.s(), t, interval.first.velocity(),
                               interval.first.agent_id(), -1);
      lower_st_point->set_acceleration(interval.first.acceleration());
      lower_st_point->set_vehicle_length(interval.first.vehicle_length());
      return true;
    }
  } else {
    std::pair<STPointWithLateral, STPointWithLateral> front_interval, back_interval;
    if (find_boundary_pair(intervals, s, v, front_interval, back_interval)) {
      upper_st_point->set_info(back_interval.first.s(), t,
                               back_interval.first.velocity(),
                               back_interval.first.agent_id(), -1);
      upper_st_point->set_acceleration(back_interval.first.acceleration());
      upper_st_point->set_vehicle_length(back_interval.first.vehicle_length());
      lower_st_point->set_info(front_interval.second.s(), t,
                               front_interval.second.velocity(),
                               front_interval.second.agent_id(), -1);
      lower_st_point->set_acceleration(front_interval.second.acceleration());
      lower_st_point->set_vehicle_length(front_interval.second.vehicle_length());
    } else {
      upper_st_point->set_info(kMaxPathLength, t, 100.0, kNoAgentId, -1);
      lower_st_point->set_info(-kMaxPathLength, t, 0.0, kNoAgentId, -1);
      // std::cout << "find no boundary pair!" << std::endl;
    }
  }

  return true;
}

void STSampleSpaceBase::GetAvailableGap(const int index, double ego_s) {
  gap_array_.clear();
  if (index < 0 || index >= st_points_table_.size()) {
    return;
  }
  std::vector<std::pair<STPointWithLateral, STPointWithLateral>> temp_gap_array;
  double current_time = index * kTimeResolution;
  std::pair<STPointWithLateral, STPointWithLateral> current_gap;
  const auto& intervals = st_points_table_.at(index);
  if (intervals.empty()) {
    current_gap.first.set_info(-kMaxPathLength, current_time, 0.0, kNoAgentId,
                               -1);
    current_gap.second.set_info(kMaxPathLength, current_time, 100.0, kNoAgentId,
                                -1);
    gap_array_.emplace_back(std::move(current_gap));
  } else {
    STPointWithLateral lower_st_point;
    lower_st_point.set_info(-kMaxPathLength, current_time, 0.0, kNoAgentId, -1);
    for (auto& interval : intervals) {
      if (lower_st_point.s() == interval.first.s()) {
        continue;
      }
      current_gap.first.set_info(lower_st_point.s(), current_time,
                                 lower_st_point.velocity(),
                                 lower_st_point.agent_id(), -1);
      current_gap.second.set_info(interval.first.s(), current_time,
                                  interval.first.velocity(),
                                  interval.first.agent_id(), -1);
      lower_st_point.set_info(interval.second.s(), current_time,
                              interval.second.velocity(),
                              interval.second.agent_id(), -1);
      if ((current_gap.second.s() - current_gap.first.s()) >=
          (front_edge_to_rear_axle_ + rear_edge_to_rear_axle_ + 4.0)) {
        temp_gap_array.emplace_back(std::move(current_gap));
      }
    }
    if (lower_st_point.s() < kMaxPathLength) {
      current_gap.first.set_info(lower_st_point.s(), current_time,
                                 lower_st_point.velocity(),
                                 lower_st_point.agent_id(), -1);
      current_gap.second.set_info(kMaxPathLength, current_time, 100.0,
                                  kNoAgentId, -1);
      temp_gap_array.emplace_back(std::move(current_gap));
    }
    auto it = std::upper_bound(
        temp_gap_array.begin(), temp_gap_array.end(), ego_s,
        [](double val, const std::pair<STPointWithLateral, STPointWithLateral>& elem) {
          return val < elem.second.s();
        });
    if (it != temp_gap_array.end()) {
      if (it != temp_gap_array.begin()) {
        it--;
        gap_array_.insert(gap_array_.end(), it, temp_gap_array.end());
      } else {
        gap_array_.insert(gap_array_.end(), it, temp_gap_array.end());
      }
    }
  }
}
}  // namespace planning
