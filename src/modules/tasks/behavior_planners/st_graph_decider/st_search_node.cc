#include "st_search_node.h"

#include <bitset>

#include "log.h"

namespace planning {

namespace {
constexpr int32_t kTLeftBit = 28;
constexpr int32_t kVLeftBit = 56;
constexpr int32_t kBase = 134217728;
}  // namespace

StSearchNode::StSearchNode(double s, double t, double vel, double s_step,
                           double t_step, double vel_step, bool only_s_t_hash) {
  s_ = s;
  t_ = t;
  vel_ = vel;

  id_ = ComputeId(s_, t_, vel_, s_step, t_step, vel_step, only_s_t_hash);
  LOG_DEBUG("StSearchNode constructor is called \n");
}

StSearchNode::StSearchNode(const StSearchNode& st_search_node)
    : id_(st_search_node.id()),
      parent_id_(st_search_node.parent_id()),
      is_valid_(st_search_node.is_valid()),
      s_(st_search_node.s()),
      t_(st_search_node.t()),
      vel_(st_search_node.vel()),
      accel_(st_search_node.accel()),
      jerk_(st_search_node.jerk()),
      cost_(st_search_node.cost()),
      g_cost_(st_search_node.g_cost()),
      h_cost_(st_search_node.h_cost()),
      edge_sub_cost_(st_search_node.edge_sub_cost()),
      node_sub_cost_(st_search_node.node_sub_cost()),
      decision_table_(st_search_node.decision_table()),
      current_decision_table_(st_search_node.current_decision_table()),
      upper_bound_(st_search_node.upper_bound().s(),
                   st_search_node.upper_bound().t(),
                   st_search_node.upper_bound().agent_id(),
                   st_search_node.upper_bound().boundary_id(),
                   st_search_node.upper_bound().velocity(),
                   st_search_node.upper_bound().acceleration(),
                   st_search_node.upper_bound().extreme_l()),
      lower_bound_(st_search_node.lower_bound().s(),
                   st_search_node.lower_bound().t(),
                   st_search_node.lower_bound().agent_id(),
                   st_search_node.lower_bound().boundary_id(),
                   st_search_node.lower_bound().velocity(),
                   st_search_node.lower_bound().acceleration(),
                   st_search_node.lower_bound().extreme_l()) {
  LOG_DEBUG("StSearchNode copy constructor is called \n");
}

StSearchNode::StSearchNode(StSearchNode&& st_search_node)
    : id_(st_search_node.id()),
      parent_id_(st_search_node.parent_id()),
      is_valid_(st_search_node.is_valid()),
      s_(st_search_node.s()),
      t_(st_search_node.t()),
      vel_(st_search_node.vel()),
      accel_(st_search_node.accel()),
      jerk_(st_search_node.jerk()),
      cost_(st_search_node.cost()),
      g_cost_(st_search_node.g_cost()),
      h_cost_(st_search_node.h_cost()),
      edge_sub_cost_(std::move(st_search_node.edge_sub_cost())),
      node_sub_cost_(std::move(st_search_node.node_sub_cost())),
      decision_table_(std::move(st_search_node.decision_table())),
      current_decision_table_(
          std::move(st_search_node.current_decision_table())),
      upper_bound_(st_search_node.upper_bound().s(),
                   st_search_node.upper_bound().t(),
                   st_search_node.upper_bound().agent_id(),
                   st_search_node.upper_bound().boundary_id(),
                   st_search_node.upper_bound().velocity(),
                   st_search_node.upper_bound().acceleration(),
                   st_search_node.upper_bound().extreme_l()),
      lower_bound_(st_search_node.lower_bound().s(),
                   st_search_node.lower_bound().t(),
                   st_search_node.lower_bound().agent_id(),
                   st_search_node.lower_bound().boundary_id(),
                   st_search_node.lower_bound().velocity(),
                   st_search_node.lower_bound().acceleration(),
                   st_search_node.lower_bound().extreme_l()) {
  LOG_DEBUG("StSearchNode move constructor is called \n");
}

StSearchNode& StSearchNode::operator=(const StSearchNode& st_search_node) {
  if (this == &st_search_node) {
    return *this;
  }
  id_ = st_search_node.id();
  parent_id_ = st_search_node.parent_id();
  is_valid_ = st_search_node.is_valid();
  s_ = st_search_node.s();
  t_ = st_search_node.t();
  vel_ = st_search_node.vel();
  accel_ = st_search_node.accel();
  jerk_ = st_search_node.jerk();
  cost_ = st_search_node.cost();
  g_cost_ = st_search_node.g_cost();
  h_cost_ = st_search_node.h_cost();
  edge_sub_cost_ = st_search_node.edge_sub_cost();
  node_sub_cost_ = st_search_node.node_sub_cost();
  decision_table_ = st_search_node.decision_table();
  current_decision_table_ = st_search_node.current_decision_table();
  upper_bound_.set_s(st_search_node.upper_bound().s());
  upper_bound_.set_t(st_search_node.upper_bound().t());
  upper_bound_.set_agent_id(st_search_node.upper_bound().agent_id());
  upper_bound_.set_boundary_id(st_search_node.upper_bound().boundary_id());
  upper_bound_.set_velocity(st_search_node.upper_bound().velocity());
  upper_bound_.set_acceleration(st_search_node.upper_bound().acceleration());
  upper_bound_.set_extreme_l(st_search_node.upper_bound().extreme_l());
  lower_bound_.set_s(st_search_node.lower_bound().s());
  lower_bound_.set_t(st_search_node.lower_bound().t());
  lower_bound_.set_agent_id(st_search_node.lower_bound().agent_id());
  lower_bound_.set_boundary_id(st_search_node.lower_bound().boundary_id());
  lower_bound_.set_velocity(st_search_node.lower_bound().velocity());
  lower_bound_.set_acceleration(st_search_node.lower_bound().acceleration());
  lower_bound_.set_extreme_l(st_search_node.lower_bound().extreme_l());
  LOG_DEBUG("StSearchNode copy assignment is called \n");
  return *this;
}

int64_t StSearchNode::ComputeId(double s, double t, double vel, double s_step,
                                double t_step, double vel_step,
                                bool only_s_t_hash) const {
  // [-134217728, 134217728]
  auto hash_s = int64_t(s / s_step) % kBase;
  std::bitset<64> b_s(std::abs(hash_s));
  if (s < 0) {
    b_s.set(27);
  }

  auto hash_t = int64_t(t / t_step) % kBase;
  std::bitset<64> b_t(std::abs(hash_t));
  if (t < 0) {
    b_t.set(27);
  }
  b_t <<= kTLeftBit;

  if (only_s_t_hash) {
    std::bitset<64> b_id(b_s.to_ulong() + b_t.to_ulong());
    return int64_t(b_id.to_ulong());
  } else {
    auto hash_vel = int64_t(vel / vel_step) % kBase;
    if (vel == 0.0) {
      hash_vel = kBase;
    }
    std::bitset<64> b_vel(std::abs(hash_vel));
    if (vel < 0) {
      b_vel.set(7);
    }
    b_vel <<= kVLeftBit;

    std::bitset<64> b_id(b_s.to_ulong() + b_t.to_ulong() + b_vel.to_ulong());
    return int64_t(b_id.to_ulong());
  }
}

void StSearchNode::NodeSubCostAccumulate(const StSearchNode& father_node) {
  node_sub_cost_.current_node_cost_yield_accumulated =
      edge_sub_cost().fathernode_to_childnode_cost_yield +
      father_node.node_sub_cost().current_node_cost_yield_accumulated;
  node_sub_cost_.current_node_cost_overtake_accumulated =
      edge_sub_cost().fathernode_to_childnode_edge_cost_overtake +
      father_node.node_sub_cost().current_node_cost_overtake_accumulated;
  node_sub_cost_.current_node_cost_vel_accumulated =
      edge_sub_cost().fathernode_to_childnode_edge_cost_vel +
      father_node.node_sub_cost().current_node_cost_vel_accumulated;
  node_sub_cost_.current_node_cost_accel_accumulated =
      edge_sub_cost().fathernode_to_childnode_edge_cost_accel +
      father_node.node_sub_cost().current_node_cost_accel_accumulated;
  node_sub_cost_.current_node_cost_accel_sign_changed_accumulated =
      edge_sub_cost().fathernode_to_childnode_edge_cost_accel_sign_changed +
      father_node.node_sub_cost()
          .current_node_cost_accel_sign_changed_accumulated;
  node_sub_cost_.current_node_cost_jerk_accumulated =
      edge_sub_cost().fathernode_to_childnode_edge_cost_jerk +
      father_node.node_sub_cost().current_node_cost_jerk_accumulated;
  node_sub_cost_.current_node_cost_length_accumulated =
      edge_sub_cost().fathernode_to_childnode_edge_cost_length +
      father_node.node_sub_cost().current_node_cost_length_accumulated;
}

}  // namespace planning