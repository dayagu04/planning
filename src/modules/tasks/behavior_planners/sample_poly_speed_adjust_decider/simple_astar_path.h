#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <iomanip>
#include "sample_space_base.h"

namespace planning {
// 车辆动力约束配置（可根据实际车型调整）
const double MAX_VELOCITY = 30.0;       // 最大速度 (m/s)
const double MIN_VELOCITY = 0.0;        // 最小速度 (m/s)
const double MAX_ACCEL = 5.0;           // 最大加速度 (m/s²)
const double MIN_ACCEL = -8.0;          // 最大减速度（负表示减速）(m/s²)
const double TIME_STEP = 0.1;           // 时间步长（分层步长）(s)
const double DISTANCE_STEP = 1.0;       // 距离步长（每层节点采样间隔）(m)
const double GOAL_TOLERANCE = 2.0;      // 目标距离容忍误差 (m)
const double WEIGHT_DIST = 1.0;         // 距离代价权重
const double WEIGHT_VEL = 0.5;          // 速度代价权重
const double WEIGHT_ACCEL = 2.0;        // 加速度代价权重（舒适性）
const double WEIGHT_JERK = 1.0;        // jerk代价权重（舒适性）

// ST节点定义：包含纵向运动状态与A*算法所需代价
struct STNode {
    double t;          // 时间 (s)
    double s;          // 纵向距离 (m)
    double v;          // 速度 (m/s)
    double a;          // 加速度 (m/s²)
    double g_cost;     // 起点到当前节点的实际代价 (g(n))
    double h_cost;     // 当前节点到目标节点的预估代价 (h(n))
    double f_cost;     // 总代价 f(n) = g(n) + h(n)
    STNode* parent;    // 父节点指针，用于回溯轨迹

    STNode() = default;
    STNode(double t_, double s_, double v_, double a_,
           double g_ = 0.0, double h_ = 0.0, STNode* p_ = nullptr)
        : t(t_), s(s_), v(v_), a(a_), g_cost(g_), h_cost(h_),
          f_cost(g_ + h_), parent(p_) {}

    bool operator>(const STNode& other) const {
        return f_cost > other.f_cost;
    }

    // 节点唯一标识（用于去重）：t*1000 + s（放大避免精度问题）
    std::string getKey() const {
        return std::to_string(int(t * 1000)) + "_" + std::to_string(int(s * 1000));
    }
};

struct GoalState {
    double target_s;    // 目标纵向距离 (m)
    double target_v;    // 目标速度 (m/s)
    double max_t;       // 最大规划时间 (s)
    GoalState(double s_, double v_, double t_) : target_s(s_), target_v(v_), max_t(t_) {}
};

class LongitudinalAStar {
public:
    LongitudinalAStar(const STNode& start_node, const GoalState& goal,
                     STSampleSpaceBase* sample_space_ptr)
        : start_node_(start_node), goal_state_(goal), sample_space_ptr_(sample_space_ptr) {}

    std::vector<STNode> PlanTrajectory();

private:
    STNode start_node_;
    GoalState goal_state_;
    STSampleSpaceBase* sample_space_ptr_;

    bool IsGoalReached(const STNode& node) const;
    std::vector<STNode> GenerateChildren(const STNode& parent_node);
    bool IsValidMotion(double v, double a) const;
    bool IsCollision(const STNode& node) const;
    double CalculateGCost(const STNode& parent, const STNode& child) const ;
    double CalculateHCost(const STNode& node) const;
    std::vector<STNode> BacktrackTrajectory(STNode* goal_node);
};
}