#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>
#include "sample_space_base.h"
#include "uniform_jerk_curve.h"
#include "ego_planning_config.h"

namespace planning {
const double TIME_STEP_NEAR = 1;  // 时间步长（分层步长）(s)
const double TIME_STEP_FAR = 1;   // 时间步长（分层步长）(s)
const double DISTANCE_STEP = 0.1;  // 距离步长（每层节点采样间隔）(m)
const double GOAL_TOLERANCE = 5.0;  // 目标距离容忍误差 (m)
const int MAX_ITERATION = 90;       // 最大迭代次数
const std::vector<int> MAX_NODES_PER_LAYER = {8, 25, 25, 20};
const double ACC_STEP = 0.37;
const double LIMIT_TIME = 4.5;    // 最小规划时域

struct STNode { 
  double t;  // 时间 (s)
  double s;  // 纵向距离 (m)
  double v;  // 速度 (m/s)
  double a;  // 加速度 (m/s²)
  double jerk;
  int depth = 0;
  double g_cost;   // 起点到当前节点的实际代价 (g(n))
  double h_cost;   // 当前节点到目标节点的预估代价 (h(n))
  double safety_cost;  // 当前节点的安全成本（不累积）
  double f_cost;   // 总代价 f(n) = g(n) + h(n) + safety_cost
  STNode* parent;  // 父节点指针，用于回溯轨迹
  double dis_to_gap_front_cost = kMaxPenalty;
  double dis_to_gap_rear_cost = kMaxPenalty;

  STNode() = default;
  STNode(double t_, double s_, double v_, double a_, double jerk_ = 0.0,
         double g_ = 0.0, double h_ = 0.0, STNode* p_ = nullptr)
      : t(t_),
        s(s_),
        v(v_),
        a(a_),
        jerk(jerk_),
        g_cost(g_),
        h_cost(h_),
        safety_cost(0.0),
        f_cost(g_ + h_),
        parent(p_) {}

  bool operator>(const STNode& other) const { return f_cost > other.f_cost; }
  bool operator<(const STNode& other) const { return f_cost < other.f_cost; }
  std::string getKey() const {
    return std::to_string(int(t * 1000)) + "_" +
           std::to_string(int(s / DISTANCE_STEP));
  }
};

struct GoalState {
  double target_s;  // 目标纵向距离 (m)
  double target_v;  // 目标速度 (m/s)
  double max_t;     // 最大规划时间 (s)
  GoalState(double s_, double v_, double t_)
      : target_s(s_), target_v(v_), max_t(t_) {}
};

class LongitudinalAStar {
 public:
  LongitudinalAStar(
      const STNode& start_node, const GoalState& goal,
      const STSampleSpaceBase* sample_space_ptr, double merge_point_s,
      const LeadingAgentInfo& leading_agent_info,
      const StateLimit& state_limit_upper, const StateLimit& state_limit_lower,
      double front_edge_to_rear_axle, double rear_edge_to_rear_axle,
      double ego_s, SampleAstarTrajConfig* config,
      std::unordered_map<int32_t, double>& agent_lateral_offset_map,
      bool is_low_speed_congestion_scene, double fartheset_merge_s);

  void PlanTrajectory();
  std::vector<STNode> GetAStarTraj() const { return astar_traj_; }
  bool IsValid() const { return valid_; }
  std::pair<double, double> GetSearchRange(const STNode& node) const;
  double CalcS(const double t) const;
  double CalcV(const double t) const;
  double CalcA(const double t) const;
  double CalcVelSafeDistance(const double ego_v, const double obj_v,
                             const double ego_a, const double obj_a,
                             bool is_front_car);
  int count_ = 0;

 private:
  struct PtrHash {
    std::size_t operator()(const std::shared_ptr<STNode>& p) const noexcept {
      return std::hash<std::string>{}(p->getKey());
    }
  };
  struct PtrEqual {
    bool operator()(const std::shared_ptr<STNode>& a,
                    const std::shared_ptr<STNode>& b) const noexcept {
      return a->getKey() == b->getKey();
    }
  };

 private:
  std::set<STNode> open_list_;
  std::unordered_map<std::string, STNode> all_list_;
  std::unordered_set<std::shared_ptr<STNode>, PtrHash, PtrEqual> closed_list_;
  std::unordered_map<int, int> layer_expanded_count_;

  STNode start_node_;
  GoalState goal_state_;
  const STSampleSpaceBase* sample_space_ptr_;
  std::vector<STNode> astar_traj_;
  bool IsGoalReached(const std::shared_ptr<STNode>& node) const;
  std::vector<STNode> GenerateChildren(std::shared_ptr<STNode>& parent_node);
  bool IsValidMotion(double v, double a) const;
  bool CollisionSafetyCheck(STNode& node) const;
  void CalculateGCost(const std::shared_ptr<STNode>& parent, STNode& child);
  void CalculateHCost(STNode& node);
  void CalculateFCost(STNode& node);
  void BacktrackTrajectory(std::shared_ptr<STNode> goal_node);
  bool CalcChildParam(const std::shared_ptr<STNode>& parent,
                      std::shared_ptr<STNode>& node) const;
  double CalcSafetyCollisionCost(double ego_speed, double object_speed,
                                 double init_distance, double vehicle_length,
                                 double left_time) const;
  bool valid_ = false;
  double merge_point_s_ = 0.0;
  LeadingAgentInfo leading_agent_info_;
  StateLimit state_limit_upper_;
  StateLimit state_limit_lower_;
  double front_edge_to_rear_axle_ = 0.0;
  double rear_edge_to_rear_axle_ = 0.0;
  double prediction_time_ = 2.0;
  double ego_s_ = 0.0;
  SampleAstarTrajConfig* config_ = nullptr;
  bool is_low_speed_congestion_scene_ = false;
  double fartheset_merge_s_ = 0.0;

  double max_velocity_;  // 最大速度 (m/s)
  double min_velocity_;   // 最小速度 (m/s)
  double max_accel_;      // 最大加速度 (m/s²)
  double min_accel_;      // 最大减速度（负表示减速）(m/s²)
  double max_jerk_;      // 最大jerk (m/s³)
  double min_jerk_;     // 最小jerk (m/s³)

  std::unordered_map<int32_t, double>& agent_lateral_offset_map_;
};
}  // namespace planning