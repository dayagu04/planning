#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

namespace planning {
constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double DELTA_T = 0.2;     // 时间分辨率 (秒)
constexpr double DELTA_S = 0.5;     // 位移分辨率 (米)
constexpr double MAX_T = 5.0;       // 规划时长
constexpr double MAX_S = 150.0;     // 规划距离
constexpr double MAX_ACCEL = 1.5;   // 最大加速度
constexpr double MIN_ACCEL = -2.0;  // 最大减速度
constexpr double MAX_SPEED = 20.0;  // 最大速度

// 代价权重系数
struct CostWeights {
  double obs_cost = 1e6;    // 障碍物碰撞代价
  spatial_potential = 1.0;  // 空间势能代价
  edge_cost = 10.0;         // 状态转移边代价
  speed_limit = 100.0;      // 速度限制代价
};

// ST坐标点
struct StPoint {
  double t;
  double s;

  StPoint() = default;
  StPoint(double t_, double s_) : t(t_), s(s_) {}
};

// Cost Table节点
struct CostNode {
  double total_cost = INF;    // 累计最小代价
  double obs_cost = 0.0;      // 障碍物代价
  double spatial_cost = 0.0;  // 空间代价
  int parent_index = -1;      // 父节点索引
  bool feasible = true;       // 是否可行
};

// DP状态转移边
struct Edge {
  StPoint from;
  StPoint to;
  double cost;
};

// 障碍物类
class Obstacle {
 public:
  StPoint st_start;  // 障碍物起始ST坐标
  StPoint st_end;    // 障碍物结束ST坐标

  Obstacle(double t_start, double s_start, double t_end, double s_end)
      : st_start(t_start, s_start), st_end(t_end, s_end) {}

  // 计算点(t,s)的碰撞代价
  double GetCost(const StPoint& point) const {
    if (point.t >= st_start.t && point.t <= st_end.t && point.s >= st_start.s &&
        point.s <= st_end.s) {
      return INF;  // 碰撞
    }

    // 距离代价 (简化版)
    double dist = 0.0;
    if (point.t < st_start.t) {
      dist = st_start.s - point.s;
    } else {
      dist = point.s - st_end.s;
    }
    return std::max(0.0, 1.0 / (dist + 0.1));
  }
};

class CostTable {
 private:
  size_t num_time_steps_;  // 时间步数
  size_t num_s_samples_;   // 位移采样数
  double delta_t_;
  double delta_s_;
  CostWeights weights_;

  std::vector<std::vector<CostNode>> cost_matrix_;

  // 障碍物列表
  std::vector<Obstacle> obstacles_;

 public:
  CostTable(double max_t, double max_s, double delta_t, double delta_s)
      : delta_t_(delta_t), delta_s_(delta_s) {
    num_time_steps_ = static_cast<size_t>(max_t / delta_t + 0.51) + 1;
    num_s_samples_ = static_cast<size_t>(max_s / delta_s + 0.51) + 1;
    cost_matrix_.resize(num_time_steps_, std::vector<CostNode>(num_s_samples_));
  }

  void SetObstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
  }

  double CalculateObstacleCost(const StPoint& point) const {
    double cost = 0.0;
    for (const auto& obs : obstacles_) {
      cost += obs.GetCost(point);
      if (cost >= INF) return INF;
    }
    return cost * weights_.obs_cost;
  }

  // 计算空间势能代价 (距离目标越远代价越高)
  double CalculateSpatialCost(const StPoint& point, double target_s) const {
    double dist_to_target = std::abs(target_s - point.s);
    return dist_to_target * weights_.spatial_potential;
  }

  // 计算边代价 (平滑性、加速度)
  double CalculateEdgeCost(const StPoint& from, const StPoint& to) const {
    double dt = to.t - from.t;
    double ds = to.s - from.s;

    if (dt <= 0) return INF;

    double velocity = ds / dt;
    double accel = (velocity - (from.s / from.t)) / dt;  // 简化计算

    // 速度限制代价
    double cost = 0.0;
    if (velocity > MAX_SPEED) {
      cost += (velocity - MAX_SPEED) * weights_.speed_limit;
    }

    // 加速度限制代价
    if (accel > MAX_ACCEL || accel < MIN_ACCEL) {
      cost += INF;
    }

    // 平滑性代价 (jerk)
    double jerk = (accel - 0.0) / dt;  // 假设前一个加速度为0
    cost += std::abs(jerk) * weights_.edge_cost;

    return cost;
  }

  // 检查动力学约束
  bool CheckDynamicConstraint(const StPoint& from, const StPoint& to) const {
    double dt = to.t - from.t;
    double ds = to.s - from.s;

    if (dt <= 0 || ds < 0) return false;  // 时间必须正向，位移不能倒退

    double v0 = (from.t == 0) ? 0.0 : from.s / from.t;
    double v1 = ds / dt;

    // 计算加速度范围
    double s_min = from.s + v0 * dt + 0.5 * MIN_ACCEL * dt * dt;
    double s_max = from.s + v0 * dt + 0.5 * MAX_ACCEL * dt * dt;

    return (to.s >= s_min - 1e-3 && to.s <= s_max + 1e-3);
  }

  // 获取可达的s索引范围
  void GetReachableSRange(int time_idx, int current_s_idx, int& min_s_idx,
                          int& max_s_idx) const {
    double current_t = time_idx * delta_t_;
    double current_s = current_s_idx * delta_s_;

    double dt = delta_t_;
    double v0 = (current_t == 0) ? 0.0 : current_s / current_t;

    // 根据动力学约束计算可达的s范围
    double s_min = current_s + v0 * dt + 0.5 * MIN_ACCEL * dt * dt;
    double s_max = current_s + v0 * dt + 0.5 * MAX_ACCEL * dt * dt;

    min_s_idx = std::max(0, static_cast<int>(s_min / delta_s_));
    max_s_idx = std::min(static_cast<int>(num_s_samples_ - 1),
                         static_cast<int>(s_max / delta_s_));
  }

  // DP核心计算过程
  void DynamicProgramming(double target_s) {
    // 初始化起点 (t=0, s=0)
    cost_matrix_[0][0].total_cost = 0.0;
    cost_matrix_[0][0].feasible = true;

    // 逐层计算 (按时间维度)
    for (size_t i = 0; i < num_time_steps_ - 1; ++i) {
      double from_t = i * delta_t_;

      // 遍历当前层所有可行节点
      for (size_t j = 0; j < num_s_samples_; ++j) {
        if (!cost_matrix_[i][j].feasible ||
            cost_matrix_[i][j].total_cost >= INF) {
          continue;
        }

        StPoint from_point(from_t, j * delta_s_);

        // 获取可达的s索引范围
        int min_k, max_k;
        GetReachableSRange(static_cast<int>(i), static_cast<int>(j), min_k,
                           max_k);

        // 遍历所有可能的转移
        for (int k = min_k; k <= max_k; ++k) {
          size_t next_s_idx = static_cast<size_t>(k);
          StPoint to_point((i + 1) * delta_t_, next_s_idx * delta_s_);

          // 检查动力学约束
          if (!CheckDynamicConstraint(from_point, to_point)) {
            continue;
          }

          // 计算各项代价
          double edge_cost = CalculateEdgeCost(from_point, to_point);
          double obs_cost = CalculateObstacleCost(to_point);
          double spatial_cost = CalculateSpatialCost(to_point, target_s);

          // 总代价
          double total_cost = cost_matrix_[i][j].total_cost + edge_cost +
                              obs_cost + spatial_cost;

          // 状态转移方程 (核心)
          if (total_cost < cost_matrix_[i + 1][next_s_idx].total_cost) {
            cost_matrix_[i + 1][next_s_idx].total_cost = total_cost;
            cost_matrix_[i + 1][next_s_idx].obs_cost = obs_cost;
            cost_matrix_[i + 1][next_s_idx].spatial_cost = spatial_cost;
            cost_matrix_[i + 1][next_s_idx].parent_index = static_cast<int>(j);
            cost_matrix_[i + 1][next_s_idx].feasible = true;
          }
        }
      }
    }
  }

  // 提取最优路径
  std::vector<StPoint> GetOptimalPath() const {
    std::vector<StPoint> path;

    // 在最后一层找到代价最小的节点
    double min_cost = INF;
    size_t optimal_s_idx = 0;
    size_t last_time_idx = num_time_steps_ - 1;

    for (size_t j = 0; j < num_s_samples_; ++j) {
      if (cost_matrix_[last_time_idx][j].total_cost < min_cost) {
        min_cost = cost_matrix_[last_time_idx][j].total_cost;
        optimal_s_idx = j;
      }
    }

    if (min_cost >= INF) {
      std::cerr << "No feasible path found!" << std::endl;
      return path;
    }

    // 回溯最优路径
    for (int i = static_cast<int>(last_time_idx); i >= 0; --i) {
      double t = i * delta_t_;
      double s = optimal_s_idx * delta_s_;
      path.emplace_back(t, s);

      if (i > 0) {
        optimal_s_idx = cost_matrix_[i][optimal_s_idx].parent_index;
      }
    }

    std::reverse(path.begin(), path.end());
    return path;
  }

  // 打印Cost Matrix (调试用)
  void PrintCostMatrix(int time_step) const {
    std::cout << "Cost Matrix at time step " << time_step << ":\n";
    for (size_t j = 0; j < num_s_samples_; j += 10) {
      std::cout << "s=" << j * delta_s_ << ": "
                << cost_matrix_[time_step][j].total_cost << " ";
    }
    std::cout << std::endl;
  }
};
}  // namespace planning