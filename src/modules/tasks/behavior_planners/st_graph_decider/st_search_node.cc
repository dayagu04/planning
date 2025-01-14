#include "st_search_node.h"

#include <bitset>

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