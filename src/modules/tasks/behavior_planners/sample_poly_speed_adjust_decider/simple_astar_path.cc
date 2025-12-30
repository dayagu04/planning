#include "simple_astar_path.h"
namespace planning {
std::vector<STNode> LongitudinalAStar::PlanTrajectory() {
  std::priority_queue<STNode, std::vector<STNode>, std::greater<STNode>> open_list;
  std::unordered_map<std::string, STNode> closed_list;

  open_list.push(start_node_);
  closed_list[start_node_.getKey()] = start_node_;

  STNode* goal_node = nullptr;
  bool found_goal = false;

  // A*主搜索循环
  while (!open_list.empty()) {
      // 弹出f_cost最小的节点
      STNode current_node = open_list.top();
      open_list.pop();

      // 检查是否到达目标
      if (IsGoalReached(current_node)) {
          goal_node = &closed_list[current_node.getKey()];
          found_goal = true;
          break;
      }

      // 扩展当前节点的子节点（生成候选运动状态）
      std::vector<STNode> children = GenerateChildren(current_node);
      for (auto& child : children) {
          std::string child_key = child.getKey();

          // 检查子节点是否已访问
          if (closed_list.find(child_key) != closed_list.end()) {
              // 若已访问，且当前代价更高，跳过
              if (child.g_cost >= closed_list[child_key].g_cost) {
                  continue;
              }
          }

          // 更新子节点父节点与代价，加入开放列表和关闭列表
          closed_list[child_key] = child;
          open_list.push(child);
      }
  }

  // 回溯轨迹
  std::vector<STNode> trajectory;
  if (found_goal && goal_node != nullptr) {
      trajectory = BacktrackTrajectory(goal_node);
      // 反转轨迹（从起点到终点）
      reverse(trajectory.begin(), trajectory.end());
  } else {
      std::cout << "警告：未找到可行的纵向参考轨迹！" << std::endl;
  }

  return trajectory;
}

bool LongitudinalAStar::IsGoalReached(const STNode& node) const {
    // 距离目标足够近，且时间未超过最大规划时间
  double s_diff = fabs(node.s - goal_state_.target_s);
  return (s_diff <= GOAL_TOLERANCE) && (node.t <= goal_state_.max_t);
}

// 生成当前节点的子节点（候选运动状态）
std::vector<STNode> LongitudinalAStar::GenerateChildren(const STNode& parent_node) {
  std::vector<STNode> children;

  // 子节点时间 = 父节点时间 + 时间步长
  double child_t = parent_node.t + TIME_STEP;
  // 超过最大规划时间，不生成子节点
  if (child_t > goal_state_.max_t) {
      return children;
  }

  // 遍历候选距离（基于父节点距离，左右扩展若干步长）
  const int search_range = 5;  // 搜索范围（左右各5个步长）
  for (int i = -search_range; i <= search_range; ++i) {
      double child_s = parent_node.s + i * DISTANCE_STEP;
      if (child_s < 0.0) { // 纵向距离不能为负
          continue;
      }

      // 计算子节点速度：v = Δs / Δt（简化为平均速度，也可采用运动学公式）
      double child_v = (child_s - parent_node.s) / TIME_STEP;
      // 计算子节点加速度：a = Δv / Δt
      double child_a = (child_v - parent_node.v) / TIME_STEP;

      // 1. 校验车辆动力约束
      if (!IsValidMotion(child_v, child_a)) {
          continue;
      }

      // 2. 校验是否与障碍物碰撞
      STNode temp_node(child_t, child_s, child_v, child_a);
      if (IsCollision(temp_node)) {
          continue;
      }

      // 3. 计算子节点代价
      double g_cost = CalculateGCost(parent_node, temp_node);
      double h_cost = CalculateHCost(temp_node);
      double f_cost = g_cost + h_cost;

      // 4. 创建子节点，设置父节点
      STNode child_node(child_t, child_s, child_v, child_a, g_cost, h_cost,
                        const_cast<STNode*>(&parent_node));
      children.push_back(child_node);
  }

  return children;
}

// 校验速度、加速度是否符合车辆约束
bool LongitudinalAStar::IsValidMotion(double v, double a) const {
  return (v >= MIN_VELOCITY && v <= MAX_VELOCITY) &&
          (a >= MIN_ACCEL && a <= MAX_ACCEL);
}

// 检查节点是否与障碍物碰撞
bool LongitudinalAStar::IsCollision(const STNode& node) const {
  int index = static_cast<int>(node.t / kTimeResolution + 0.51);
}

// 计算g_cost：起点到当前节点的实际代价
double LongitudinalAStar::CalculateGCost(const STNode& parent, const STNode& child) const {
  double dist_cost = WEIGHT_DIST * fabs(child.s - goal_state_.target_s);
  double vel_cost = WEIGHT_VEL * fabs(child.v - goal_state_.target_v);
  double accel_cost = WEIGHT_ACCEL * fabs(child.a); // 加速度越小，舒适性越好
  double jerk_cost = WEIGHT_JERK * std::pow((child.a - parent.a)/TIME_STEP,2.0); //  jerk越小，舒适性越好
  // 父节点g_cost + 当前节点增量代价
  return parent.g_cost + dist_cost + vel_cost + accel_cost + jerk_cost;
}

double LongitudinalAStar::CalculateHCost(const STNode& node) const {
  double s_h = fabs(node.s - goal_state_.target_s);
  double v_h = fabs(node.v - goal_state_.target_v);
  return s_h + 0.5 * v_h;
}

std::vector<STNode> LongitudinalAStar::BacktrackTrajectory(STNode* goal_node) {
  std::vector<STNode> trajectory;
  STNode* current_node = goal_node;

  while (current_node != nullptr) {
      trajectory.push_back(*current_node);
      current_node = current_node->parent;
  }

  return trajectory;
}
}