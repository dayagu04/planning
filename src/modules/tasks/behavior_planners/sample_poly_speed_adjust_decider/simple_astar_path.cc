#include "simple_astar_path.h"

namespace planning {
LongitudinalAStar::LongitudinalAStar(
    const STNode& start_node, const GoalState& goal,
    const STSampleSpaceBase* sample_space_ptr, double merge_point_s,
    const LeadingAgentInfo& leading_agent_info,
    const StateLimit& state_limit_upper, const StateLimit& state_limit_lower,
    double front_edge_to_rear_axle, double rear_edge_to_rear_axle, double ego_s,
    SampleAstarTrajConfig* config,
    std::unordered_map<int32_t, double>& agent_lateral_offset_map)
    : start_node_(start_node),
      goal_state_(goal),
      sample_space_ptr_(sample_space_ptr),
      merge_point_s_(merge_point_s),
      leading_agent_info_(leading_agent_info),
      state_limit_upper_(state_limit_upper),
      state_limit_lower_(state_limit_lower),
      front_edge_to_rear_axle_(front_edge_to_rear_axle),
      rear_edge_to_rear_axle_(rear_edge_to_rear_axle),
      ego_s_(ego_s),
      config_(config),
      agent_lateral_offset_map_(agent_lateral_offset_map) {
  max_velocity_ = state_limit_upper_.v_max;
  min_velocity_ = state_limit_upper_.v_min;
  max_accel_ = state_limit_upper_.a_max;
  min_accel_ = state_limit_upper_.a_min;
  max_jerk_ = state_limit_upper_.j_max;
  min_jerk_ = state_limit_upper_.j_min;
  CalculateHCost(start_node_);
  PlanTrajectory();
}

void LongitudinalAStar::PlanTrajectory() {
  goal_state_.target_s =
      std::max(goal_state_.target_s, state_limit_upper_.v_max * 5.0);
  // goal_state_.target_s = 1000;
  // merge_point_s_ = 1000;
  astar_traj_.clear();
  open_list_.clear();
  closed_list_.clear();
  all_list_.clear();
  open_list_.insert(start_node_);
  all_list_[start_node_.getKey()] = start_node_;

  std::shared_ptr<STNode> goal_node = nullptr;
  bool found_goal = false;
  while (!open_list_.empty() && count_ <= MAX_ITERATION) {
    count_++;
    STNode current_node = *open_list_.begin();
    auto current_node_ptr = std::make_shared<STNode>(current_node);
    open_list_.erase(open_list_.begin());
    closed_list_.insert(current_node_ptr);
    all_list_.erase(current_node_ptr->getKey());
    if (IsGoalReached(current_node_ptr)) {
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
      auto it = open_list_.find(all_list_[child_key]);
      if (it != open_list_.end()) {
        open_list_.erase(it);
      }
      all_list_[child_key] = child;
      open_list_.insert(child);
    }
  }

  // 回溯轨迹
  if (found_goal && goal_node != nullptr) {
    BacktrackTrajectory(goal_node);
    // 反转轨迹（从起点到终点）
    reverse(astar_traj_.begin(), astar_traj_.end());
    valid_ = true;
  } else {
    // std::cout << "警告：未找到可行的纵向参考轨迹！" << std::endl;
    valid_ = false;
  }
}

bool LongitudinalAStar::IsGoalReached(
    const std::shared_ptr<STNode>& node) const {
  double s_diff = node->s - merge_point_s_;
  return (s_diff > 0) && (node->t > LIMIT_TIME);
}

std::vector<STNode> LongitudinalAStar::GenerateChildren(
    std::shared_ptr<STNode>& parent_node) {
  std::vector<STNode> children;
  double time_step = parent_node->t < 1.9 ? TIME_STEP_NEAR : TIME_STEP_FAR;
  double child_t = parent_node->t + time_step;
  if (child_t > goal_state_.max_t) {
    return children;
  }

  // auto search_range = GetSearchRange(*parent_node);
  // double arrived_s = parent_node->s + parent_node->v * time_step;
  // std::pair<double,double> search_range = {arrived_s - 0.10, arrived_s +
  // 0.10};
  // int search_range_index =
  //     int((search_range.second - search_range.first) / DISTANCE_STEP) + 1;
  int search_range_index = (max_accel_ - min_accel_) / ACC_STEP;
  for (int i = 0; i <= search_range_index; i++) {
    // double child_s = i != search_range_index
    //                      ? search_range.first + i * DISTANCE_STEP
    //                      : search_range.second;
    // if (child_s < 0.0) {
    //   continue;
    // }
    double child_acc = min_accel_ + i * ACC_STEP;
    double child_jerk = (child_acc - parent_node->a) / time_step;
    if (child_jerk > max_jerk_ || child_jerk < min_jerk_) {
      continue;
    }
    auto node_ptr =
        std::make_shared<STNode>(child_t, 0.0, 0.0, child_acc, child_jerk);
    if (!CalcChildParam(parent_node, node_ptr)) {
      continue;
    }
    if (closed_list_.find(node_ptr) != closed_list_.end()) {
      continue;
    }

    // if (!CalcChildParam(parent_node, temp_node)) {
    //   continue;
    // }

    if (!IsValidMotion(node_ptr->v, node_ptr->a)) {
      continue;
    }

    if (!CollisionSafetyCheck(*node_ptr)) {
      continue;
    }

    CalculateGCost(parent_node, *node_ptr);
    CalculateHCost(*node_ptr);
    CalculateFCost(*node_ptr);
    node_ptr->parent = parent_node.get();
    children.emplace_back(std::move(*node_ptr));
  }

  return children;
}

bool LongitudinalAStar::IsValidMotion(double v, double a) const {
  return (v > min_velocity_ - kZeroEpsilon &&
          v < max_velocity_ + kZeroEpsilon) &&
         (a > min_accel_ - kZeroEpsilon && a < max_accel_ + kZeroEpsilon);
}

bool LongitudinalAStar::CollisionSafetyCheck(STNode& node) const {
  int index = static_cast<int>(node.t / kTimeResolution + 0.51);
  if (index >= sample_space_ptr_->st_points_table().size()) {
    return false;
  }
  double frenet_node_s = node.s + ego_s_;
  std::vector<std::pair<planning::speed::STPointWithLateral,
                        planning::speed::STPointWithLateral>>
      gap_list;
  if (sample_space_ptr_->GetNearestGapListByAvailable(frenet_node_s, node.v,
                                                         node.t, &gap_list)) {
    bool is_safe = false;
    for (auto& gap_pair : gap_list) {
      auto& anchor_matched_lower_st_point = gap_pair.first;
      auto& anchor_matched_upper_st_point = gap_pair.second;
      // 校验gap后车碰撞风险
      double dis_to_gap_rear_cost = 0.0;
      double dis_to_gap_front_cost = 0.0;
      if (anchor_matched_lower_st_point.agent_id() != kNoAgentId) {
        double s_buffer = interp(config_->rear_vehicle_min_distance_map.rear_speed_kph_table,
                                config_->rear_vehicle_min_distance_map.rear_distance_table,
                                anchor_matched_lower_st_point.velocity());
        s_buffer =
            node.s > merge_point_s_
                ? s_buffer * std::exp((merge_point_s_ - node.s) /
                                      config_->gap_rear_buffer_decay_factor)
                : s_buffer;
        double gap = frenet_node_s - anchor_matched_lower_st_point.s() -
                     s_buffer - rear_edge_to_rear_axle_;
        int object_lateral_offset =
            static_cast<int>(config_->lateral_offset_scale_factor *
                             anchor_matched_lower_st_point.l());
        double collision_s = 0.0;
        if (object_lateral_offset < 0) {
          collision_s = merge_point_s_;
        } else {
          auto it = agent_lateral_offset_map_.find(object_lateral_offset);
          if (it != agent_lateral_offset_map_.end()) {
            collision_s = it->second;
          } else {
            collision_s = config_->default_collision_distance;
          }
        }
        if (gap < 0.0 && node.s > collision_s) {
          std::cout << "Gap后车碰撞风险: " << node.getKey() << node.getKey()
                    << " 剩余距离 :  " << gap << "  buffer:  " << s_buffer
                    << std::endl;
          continue;
        } else {
          double s_buffer_extra = config_->gap_rear_buffer_extra_coef *
                                  anchor_matched_lower_st_point.velocity();
          double left_time =
              (node.s - collision_s) / std::fmax(node.v, kZeroEpsilon);
          left_time = std::fmax(left_time, 0.0);
          dis_to_gap_rear_cost = CalcSafetyCollisionCost(
              anchor_matched_lower_st_point.velocity(), node.v,
              frenet_node_s - anchor_matched_lower_st_point.s() -
                  rear_edge_to_rear_axle_,
              anchor_matched_lower_st_point.vehicle_length(), left_time);
        }
      }
      // 校验gap前车碰撞风险
      if (anchor_matched_upper_st_point.agent_id() != kNoAgentId) {
        double follow_distance =
            node.v * (node.v - anchor_matched_upper_st_point.velocity()) /
            (2.0 * config_->gap_front_follow_decel);
        double thw = config_->gap_front_thw_coef * node.v;
        double s_buffer =
            std::fmax(thw + follow_distance + config_->gap_front_min_buffer,
                      config_->gap_front_min_buffer);
        double gap = anchor_matched_upper_st_point.s() - frenet_node_s -
                     s_buffer - front_edge_to_rear_axle_;
        int object_lateral_offset =
            static_cast<int>(config_->lateral_offset_scale_factor *
                             anchor_matched_upper_st_point.l());
        double collision_s = 0.0;
        if (object_lateral_offset < 0) {
          collision_s = merge_point_s_;
        } else {
          auto it = agent_lateral_offset_map_.find(object_lateral_offset);
          if (it != agent_lateral_offset_map_.end()) {
            collision_s = it->second;
          } else {
            collision_s = config_->default_collision_distance;
          }
        }
        if (gap < 0.0 &&
            node.s > (collision_s - std::max(follow_distance, thw))) {
          std::cout << "Gap前车碰撞风险: " << node.getKey()
                    << " 剩余距离 :  " << gap << "  buffer:  " << s_buffer
                    << std::endl;
          continue;
        } else {
          double left_time =
              (node.s - (collision_s - std::max(follow_distance, thw))) /
              std::fmax(node.v, kZeroEpsilon);
          left_time = std::fmax(left_time, 0.0);
          double s_buffer_extra = config_->gap_front_buffer_extra_coef * node.v;
          dis_to_gap_front_cost = CalcSafetyCollisionCost(
              node.v, anchor_matched_upper_st_point.velocity(),
              anchor_matched_upper_st_point.s() - frenet_node_s -
                  front_edge_to_rear_axle_,
              anchor_matched_upper_st_point.vehicle_length(), left_time);
        }
      }
      if ((dis_to_gap_rear_cost + dis_to_gap_front_cost) <
          (node.dis_to_gap_front_cost + node.dis_to_gap_rear_cost)) {
        node.dis_to_gap_front_cost = dis_to_gap_front_cost;
        node.dis_to_gap_rear_cost = dis_to_gap_rear_cost;
        is_safe = true;
      }
    }
    if (!is_safe) {
      return false;
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
    double follow_distance = node.v * (node.v - leading_agent_v) /
                             (2.0 * config_->leading_follow_decel);
    double thw = config_->leading_thw_coef * node.v;
    double safe_distance =
        std::fmax(thw + follow_distance + config_->leading_min_safe_distance,
                  config_->leading_min_safe_distance);
    double gap =
        leading_agent_s + leading_agent_info_.center_s - node.s - safe_distance;
    if (gap < 0.0) {
      // std::cout << "前车碰撞风险: " << node.getKey() << std::endl;
      return false;
    }
  }

  return true;
}

void LongitudinalAStar::CalculateGCost(const std::shared_ptr<STNode>& parent,
                                       STNode& child) {
  // double dist_cost = WEIGHT_DIST * fabs(child.s - goal_state_.target_s);
  double time_cost = start_node_.v * fabs(child.t - parent->t) / 10.0;
  double vel_cost = config_->weight_vel * fabs(child.v - parent->v);
  double accel_cost = config_->weight_accel * fabs(child.a - parent->a);
  // double time_step = parent->t < 1.9 ? TIME_STEP_NEAR : TIME_STEP_FAR;
  double jerk_cost = config_->weight_jerk * fabs(child.jerk);
  child.g_cost = parent->g_cost + vel_cost + accel_cost + jerk_cost + time_cost;
}

void LongitudinalAStar::CalculateHCost(STNode& node) {
  const double GOAL_TOLERANCE = 5.0;  // 目标距离容忍误差 (m)
  bool is_goal_range = (node.s > goal_state_.target_s - GOAL_TOLERANCE) &&
                       (node.s < goal_state_.target_s + GOAL_TOLERANCE);
  double s_h =
      is_goal_range ? 0.0
      : node.s < goal_state_.target_s - GOAL_TOLERANCE
          ? fabs(node.s - goal_state_.target_s + GOAL_TOLERANCE) / 10.0
          : fabs(node.s - goal_state_.target_s - GOAL_TOLERANCE) / 30.0;
  double v_h = config_->weight_vel * fabs(node.v - goal_state_.target_v);
  double acc_h = config_->weight_accel * fabs(node.a);
  node.h_cost = s_h + 0.5 * v_h + node.dis_to_gap_front_cost +
                node.dis_to_gap_rear_cost + acc_h;
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

double LongitudinalAStar::CalcV(const double t) const {
  if (t < kZeroEpsilon) {
    return astar_traj_[0].v;
  }
  for (int i = 0; i < astar_traj_.size(); ++i) {
    if (t < astar_traj_[i].t) {
      double delta_t = t - astar_traj_[i - 1].t;
      return astar_traj_[i - 1].v + astar_traj_[i - 1].a * delta_t +
             0.5 * astar_traj_[i].jerk * delta_t * delta_t;
    }
  }
  return astar_traj_.back().v;
}

double LongitudinalAStar::CalcA(const double t) const {
  if (t < kZeroEpsilon) {
    return astar_traj_[0].a;
  }
  for (int i = 0; i < astar_traj_.size(); ++i) {
    if (t < astar_traj_[i].t) {
      double delta_t = t - astar_traj_[i - 1].t;
      return astar_traj_[i - 1].a + astar_traj_[i].jerk * delta_t;
    }
  }
  return astar_traj_.back().a;
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
                                       std::shared_ptr<STNode>& node) const {
  // double delta_s = node->s - parent->s;
  double delta_t = node->t - parent->t;

  if (std::fabs(delta_t) < 1e-6) {
    return false;
  }

  node->v =
      parent->v + parent->a * delta_t + 0.5 * node->jerk * delta_t * delta_t;
  node->s = parent->s + parent->v * delta_t +
            0.5 * parent->a * delta_t * delta_t +
            node->jerk * delta_t * delta_t * delta_t / 6.0;
  if (node->s < 0.0 || node->v < min_velocity_ || node->v > max_velocity_) {
    return false;
  }
  return true;
}

double LongitudinalAStar::CalcSafetyCollisionCost(double rear_speed,
                                                  double front_speed,
                                                  double init_distance,
                                                  double vehicle_length,
                                                  double left_time) const {
  double thw = 1.5 * rear_speed;
  double s_buffer = (front_speed - rear_speed) * left_time;
  double left_safe_distance = init_distance + s_buffer;

  double cost = 0.0;
  if (left_safe_distance > thw ||
      left_safe_distance < (-2.0 * vehicle_length - thw)) {
    cost = 0.0;
  } else if (left_safe_distance > 0.0) {
    cost = config_->weight_normal_ttc *
           std::exp((1 - left_safe_distance / std::fmax(thw, kZeroEpsilon)) *
                    config_->safe_cost_gain);
  } else if (left_safe_distance > -vehicle_length) {
    cost = config_->weight_overlap_ttc *
           std::exp((1 - left_safe_distance /
                             std::fmax(vehicle_length, kZeroEpsilon)) *
                    config_->safe_cost_gain);
  } else if (left_safe_distance > -2 * vehicle_length) {
    cost = config_->weight_overlap_ttc *
           std::exp((1 - (-vehicle_length - left_safe_distance) /
                             std::fmax(vehicle_length, kZeroEpsilon)) *
                    config_->safe_cost_gain);
  } else {
    cost = config_->weight_normal_ttc *
           std::exp((1 - (-2 * vehicle_length - left_safe_distance) /
                             std::fmax(thw, kZeroEpsilon)) *
                    config_->safe_cost_gain);
  }
  return cost;
}
}  // namespace planning