#pragma once

#include "st_graph/st_boundary.h"
#include "st_graph/st_point.h"
#include "vec2d.h"
#include <unordered_map>

namespace planning {

class StSearchNode {
 public:
  StSearchNode() = default;
  StSearchNode(double s, double t, double vel, double s_step, double t_step,
               double vel_step, bool only_s_t_hash = false);
  ~StSearchNode() = default;

  int64_t id() const { return id_; }
  void set_id(int64_t id) { id_ = id; }

  int64_t parent_id() const { return parent_id_; }
  void set_parent_id(int64_t parent_id) { parent_id_ = parent_id; }

  bool is_valid() const { return is_valid_; }
  void set_is_valid(bool is_valid) { is_valid_ = is_valid; }

  double s() const { return s_; }
  void set_s(double s) { s_ = s; }

  double t() const { return t_; }
  void set_t(double t) { t_ = t; }

  double vel() const { return vel_; }
  void set_vel(double vel) { vel_ = vel; }

  double accel() const { return accel_; }
  void set_accel(double accel) { accel_ = accel; }

  double jerk() const { return jerk_; }
  void set_jerk(double jerk) { jerk_ = jerk; }

  double cost() const { return cost_; }
  void set_cost(double cost) { cost_ = cost; }

  double g_cost() const { return g_cost_; }
  void set_g_cost(double g_cost) { g_cost_ = g_cost; }

  double h_cost() const { return h_cost_; }
  void set_h_cost(double h_cost) { h_cost_ = h_cost; }

  // total_cost = g_cost + h_cost
  double TotalCost() const { return g_cost_ + h_cost_; }

  std::unordered_map<int64_t, speed::STBoundary::DecisionType> decision_table()
      const {
    return decision_table_;
  }
  void set_decision_table(
      std::unordered_map<int64_t, speed::STBoundary::DecisionType>
          decision_table) {
    decision_table_ = decision_table;
  }
  const std::unordered_map<int64_t, speed::STBoundary::DecisionType>&
  current_decision_table() const {
    return current_decision_table_;
  }
  void set_current_decision_table(
      const std::unordered_map<int64_t, speed::STBoundary::DecisionType>&
          decision_table) {
    current_decision_table_ = decision_table;
  }

  speed::STPoint upper_bound() const { return upper_bound_; }
  void set_upper_bound(speed::STPoint upper_bound) {
    upper_bound_ = upper_bound;
  }
  speed::STPoint lower_bound() const { return lower_bound_; }
  void set_lower_bound(speed::STPoint lower_bound) {
    lower_bound_ = lower_bound;
  }

 private:
  // make a hash id by step
  int64_t ComputeId(double s, double t, double vel, double s_step,
                    double t_step, double vel_step, bool only_s_t_hash = false) const;

 private:
  int64_t id_ = -1;
  int64_t parent_id_ = -1;
  bool is_valid_ = false;
  double s_ = 0.0;
  double t_ = 0.0;
  double vel_ = 0.0;
  double accel_ = 0.0;
  double jerk_ = 0.0;

  double cost_ = 0.0;  // edge cost from parent
  double g_cost_ = 0.0;
  double h_cost_ = 0.0;
  // <STBoundary_id, decision>
  std::unordered_map<int64_t, speed::STBoundary::DecisionType> decision_table_;
  std::unordered_map<int64_t, speed::STBoundary::DecisionType>
      current_decision_table_;

  speed::STPoint upper_bound_{};
  speed::STPoint lower_bound_{};
};

}  // namespace planning