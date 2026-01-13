#include "simple_astar_path.h"
namespace planning {
LongitudinalAStar::LongitudinalAStar(
    const STNode& start_node, const GoalState& goal,
    const STSampleSpaceBase* sample_space_ptr, double merge_point_s,
    const LeadingAgentInfo& leading_agent_info,
    const StateLimit& state_limit_upper, const StateLimit& state_limit_lower,
    double front_edge_to_rear_axle, double rear_edge_to_rear_axle, double ego_s)
    : start_node_(start_node),
      goal_state_(goal),
      sample_space_ptr_(sample_space_ptr),
      merge_point_s_(merge_point_s),
      leading_agent_info_(leading_agent_info),
      state_limit_upper_(state_limit_upper),
      state_limit_lower_(state_limit_lower),
      front_edge_to_rear_axle_(front_edge_to_rear_axle),
      rear_edge_to_rear_axle_(rear_edge_to_rear_axle),
      ego_s_(ego_s) {
  //       state_limit_lower_.v_max = 120 / 3.6;
  // state_limit_upper_.v_max = 120 / 3.6;
  // state_limit_upper_.v_end = 120/3.6;
  CalculateHCost(start_node_);
  PlanTrajectory();
}

void LongitudinalAStar::PlanTrajectory() {
  goal_state_.target_s = std::max(goal_state_.target_s, start_node_.v * 4.0);
  // goal_state_.target_s = 1000;
  astar_traj_.clear();
  std::priority_queue<STNode, std::vector<STNode>, std::greater<STNode>> empty;
  std::swap(open_list_, empty);
  closed_list_.clear();
  all_list_.clear();
  open_list_.push(start_node_);
  all_list_[start_node_.getKey()] = start_node_;

  std::shared_ptr<STNode> goal_node = nullptr;
  bool found_goal = false;

  while (!open_list_.empty()) {
    count_++;
    // if (count_ > 250) {
    //   std::cout << "警告：A*搜索未收敛！" << std::endl;
    //   break;
    // }
    STNode current_node = open_list_.top();
    auto current_node_ptr = std::make_shared<STNode>(current_node);
    open_list_.pop();
    closed_list_.insert(current_node_ptr);
    if (IsGoalReached(current_node)) {
      goal_node = current_node_ptr;
      found_goal = true;
      break;
    }

    std::vector<STNode> children = GenerateChildren(current_node_ptr);
    for (auto& child : children) {
      std::string child_key = child.getKey();

      if (all_list_.find(child_key) != all_list_.end()) {
        if (child.f_cost >= all_list_[child_key].f_cost) {
          continue;
        }
      }

      all_list_[child_key] = child;
      open_list_.push(child);
    }
  }

  // 回溯轨迹
  if (found_goal && goal_node != nullptr) {
    BacktrackTrajectory(goal_node);
    // 反转轨迹（从起点到终点）
    reverse(astar_traj_.begin(), astar_traj_.end());
    valid_ = true;
  } else {
    std::cout << "警告：未找到可行的纵向参考轨迹！" << std::endl;
    valid_ = false;
  }
}

bool LongitudinalAStar::IsGoalReached(const STNode& node) const {
  double s_diff = node.s - goal_state_.target_s;
  return (s_diff > 0) && (node.t <= goal_state_.max_t);
}

std::vector<STNode> LongitudinalAStar::GenerateChildren(
    std::shared_ptr<STNode>& parent_node) {
  std::vector<STNode> children;
  double time_step = parent_node->t < 1.9 ? TIME_STEP_NEAR : TIME_STEP_FAR;
  double child_t = parent_node->t + time_step;
  if (child_t > goal_state_.max_t) {
    return children;
  }

  auto search_range = GetSearchRange(*parent_node);
  // double arrived_s = parent_node->s + parent_node->v * time_step;
  // std::pair<double,double> search_range = {arrived_s - 0.10, arrived_s +
  // 0.10};
  int search_range_index =
      int((search_range.second - search_range.first) / DISTANCE_STEP) + 1;
  for (int i = 0; i <= search_range_index; i++) {
    double child_s = i != search_range_index
                         ? search_range.first + i * DISTANCE_STEP
                         : search_range.second;
    if (child_s < 0.0) {
      continue;
    }
    auto node_ptr = std::make_shared<STNode>(child_t, child_s, 0.0, 0.0);
    if (closed_list_.find(node_ptr) != closed_list_.end()) {
      continue;
    }
    STNode temp_node(child_t, child_s, 0.0, 0.0);

    if (!CalcChildParam(parent_node, temp_node)) {
      continue;
    }

    if (!IsValidMotion(temp_node.v, temp_node.a)) {
      continue;
    }

    if (!CollisionSafetyCheck(temp_node)) {
      continue;
    }

    CalculateGCost(parent_node, temp_node);
    CalculateHCost(temp_node);
    CalculateFCost(temp_node);
    temp_node.parent = parent_node.get();
    children.emplace_back(std::move(temp_node));
  }

  return children;
}

bool LongitudinalAStar::IsValidMotion(double v, double a) const {
  return (v >= MIN_VELOCITY && v <= MAX_VELOCITY) &&
         (a >= MIN_ACCEL && a <= MAX_ACCEL);
}

bool LongitudinalAStar::CollisionSafetyCheck(STNode& node) const {
  int index = static_cast<int>(node.t / kTimeResolution + 0.51);
  if (index >= sample_space_ptr_->st_points_table().size()) {
    return false;
  }
  auto& state = sample_space_ptr_->st_points_table()[index];
  if (node.s > merge_point_s_) {
    double frenet_node_s = node.s + ego_s_;
    for (auto& interval : state) {
      if (frenet_node_s < interval.second.s() &&
          frenet_node_s > interval.first.s()) {
        return false;
      } else if (frenet_node_s > interval.second.s()) {
        double s_buffer = 3.5 + 0.3 * interval.second.velocity();
        // double s_buffer = 3.5;
        double gap = frenet_node_s - interval.second.s() - s_buffer -
                     rear_edge_to_rear_axle_;
        if (gap < 0.0) {
          std::cout << "Gap后车碰撞风险: " << node.getKey() << std::endl;
          return false;
        } else {
          double s_buffer_extra = 0.7 * interval.second.velocity();
          node.dis_to_gap_rear_cost =
              gap > s_buffer_extra
                  ? 0.0
                  : std::exp(1- gap / std::fmax(s_buffer_extra, kZeroEpsilon));
        }
      } else {
        double s_buffer = 3.5 + 0.3 * node.v;
        double gap = interval.first.s() - frenet_node_s - s_buffer -
                     front_edge_to_rear_axle_;
        if (gap < 0.0) {
          std::cout << "Gap前车碰撞风险: " << node.getKey() << std::endl;
          return false;
        } else {
          double s_buffer_extra = 0.7 * node.v;
          node.dis_to_gap_front_cost =
              gap > s_buffer_extra
                  ? 0.0
                  : 0.5 *
                        std::exp(1 - gap / std::fmax(s_buffer_extra, kZeroEpsilon));
        }
      }
    }
  }
  // 检查是否与前车碰撞
  if (leading_agent_info_.id != kNoAgentId && leading_agent_info_.id != -1) {
    double leading_agent_v = 0.0;
    double leading_agent_s = 0.0;
    if (index < leading_agent_info_.prediction_path.size()) {
      leading_agent_s = leading_agent_info_.prediction_path[index].first;
      leading_agent_v = leading_agent_info_.prediction_path[index].second;
    } else {
      if (leading_agent_info_.prediction_path_valid) {
        leading_agent_v = leading_agent_info_.prediction_path.back().second;
        leading_agent_s =
            leading_agent_info_.prediction_path.back().first +
            leading_agent_v *
                (index - leading_agent_info_.prediction_path.size()) *
                kTimeResolution;
      } else {
        leading_agent_v = leading_agent_info_.v;
        leading_agent_s = leading_agent_info_.v * node.t;
      }
    }
    double s_buffer = 3.5 + 0.3 * leading_agent_v;
    double follow_distance =
        node.v * std::fmax(node.v - leading_agent_v, 0.0) / (2.0 * 2.0);
    double thw = 0.3 * node.v;
    double safe_distance = std::fmax(thw + follow_distance, 3.5);
    double gap =
        leading_agent_s + leading_agent_info_.center_s - node.s - s_buffer;
    if (gap < 0.0) {
      std::cout << "前车碰撞风险: " << node.getKey() << std::endl;
      return false;
    }
  }
  return true;
}

void LongitudinalAStar::CalculateGCost(const std::shared_ptr<STNode>& parent,
                                       STNode& child) {
  // double dist_cost = WEIGHT_DIST * fabs(child.s - goal_state_.target_s);
  double time_cost = start_node_.v * fabs(child.t - parent->t);
  double vel_cost = WEIGHT_VEL * fabs(child.v - parent->v);
  double accel_cost = WEIGHT_ACCEL * fabs(child.a - parent->a);
  double time_step = parent->t < 1.9 ? TIME_STEP_NEAR : TIME_STEP_FAR;
  double jerk_cost = WEIGHT_JERK * std::fabs((child.a - parent->a) / time_step);
  child.g_cost = parent->g_cost + vel_cost + accel_cost + jerk_cost + time_cost;
}

void LongitudinalAStar::CalculateHCost(STNode& node) {
  const double GOAL_TOLERANCE = 5.0;  // 目标距离容忍误差 (m)
  bool is_goal_range = (node.s > goal_state_.target_s) &&
                       (node.s < goal_state_.target_s + GOAL_TOLERANCE);
  double s_h =
      is_goal_range
          ? 0.0
          : node.s < goal_state_.target_s
                ? fabs(node.s - goal_state_.target_s)
                : fabs(node.s - goal_state_.target_s - GOAL_TOLERANCE) / 3.0;
  double v_h = fabs(node.v - goal_state_.target_v);
  node.h_cost =
      s_h + 0.5 * v_h + node.dis_to_gap_front_cost + node.dis_to_gap_rear_cost;
}

void LongitudinalAStar::CalculateFCost(STNode& node) {
  node.f_cost = node.g_cost + node.h_cost;
}

void LongitudinalAStar::BacktrackTrajectory(std::shared_ptr<STNode> goal_node) {
  astar_traj_.clear();
  STNode* current_node = goal_node.get();
  while (current_node != nullptr) {
    astar_traj_.push_back(*current_node);
    current_node = current_node->parent;
  }
}

std::pair<double, double> LongitudinalAStar::GetSearchRange(
    const STNode& node) const {
  LonState lon_state = {node.s, node.v, node.a, 0.0, node.t};
  UniformJerkCurve jerk_curve_upper(state_limit_upper_, lon_state, true);
  UniformJerkCurve jerk_curve_lower(state_limit_lower_, lon_state, false);
  double time_step = node.t < 1.9 ? TIME_STEP_NEAR : TIME_STEP_FAR;
  return {jerk_curve_lower.CalcS(time_step), jerk_curve_upper.CalcS(time_step)};
}

double LongitudinalAStar::CalcS(const double t) const {
  if (t < kZeroEpsilon) {
    return 0.0;
  }
  for (int i = 0; i < astar_traj_.size(); ++i) {
    if (t < astar_traj_[i].t) {
      double delta_t = t - astar_traj_[i - 1].t;
      double s = astar_traj_[i - 1].s + astar_traj_[i - 1].v * delta_t +
                 0.5 * astar_traj_[i - 1].a * delta_t * delta_t +
                 astar_traj_[i].jerk * delta_t * delta_t * delta_t / 6.0;
      return s;
    }
  }
  return astar_traj_.back().s +
         astar_traj_.back().v * (t - astar_traj_.back().t);
}

double LongitudinalAStar::CalcVelSafeDistance(const double ego_v,
                                              const double obj_v,
                                              const double ego_a,
                                              const double obj_a,
                                              bool is_front_car) {
  double differ_acc =
      std::fabs(ego_a - obj_a) < kZeroEpsilon ? 0.001 : (ego_a - obj_a);
  differ_acc =
      std::fabs(differ_acc) * std::min(0.8, std::fabs(differ_acc)) / differ_acc;
  const double calculate_collision_time = (obj_v - ego_v) / differ_acc;
  if (calculate_collision_time < 0.0 ||
      calculate_collision_time > prediction_time_) {
    double limit_distance =
        (ego_v - obj_v) * prediction_time_ +
        0.5 * differ_acc * prediction_time_ * prediction_time_;
    if (ego_v > obj_v) {
      return is_front_car ? limit_distance : 0.0;
    } else {
      return is_front_car ? 0.0 : -limit_distance;
    }
  } else {
    double limit_distance =
        (ego_v - obj_v) * calculate_collision_time +
        0.5 * differ_acc * calculate_collision_time * calculate_collision_time;
    if (ego_v > obj_v) {
      return is_front_car ? limit_distance
                          : std::max((obj_v - ego_v) * prediction_time_ -
                                         0.5 * differ_acc * prediction_time_ *
                                             prediction_time_,
                                     0.0);
    } else {
      return is_front_car ? std::max((ego_v - obj_v) * prediction_time_ +
                                         0.5 * differ_acc * prediction_time_ *
                                             prediction_time_,
                                     0.0)
                          : -limit_distance;
    }
  }
}

bool LongitudinalAStar::CalcChildParam(const std::shared_ptr<STNode>& parent,
                                       STNode& node) const {
  double delta_s = node.s - parent->s;
  double delta_t = node.t - parent->t;

  if (std::fabs(delta_t) < 1e-6) {
    return false;
  }

  // 解析求解 jerk
  node.jerk =
      6.0 *
      (delta_s - 0.5 * parent->a * delta_t * delta_t - parent->v * delta_t) /
      (delta_t * delta_t * delta_t);
  node.a = parent->a + node.jerk * delta_t;
  node.v =
      parent->v + parent->a * delta_t + 0.5 * node.jerk * delta_t * delta_t;
  return true;
}
}  // namespace planning