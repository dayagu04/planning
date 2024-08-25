#include "dynamic_world.h"
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>
#include "agent/agent_manager.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "log.h"
// #include "trajectory/path_point.h"
#include "trajectory/trajectory_point.h"
#include "utils/path_point.h"
#include "virtual_lane_manager.h"
// #include "utils/kd_path.h"

namespace planning {
namespace planning_data {

// void DynamicWorld::UpdateAgentManager(
//     agent::AgentManager* const agent_manager) {
//   agent_manager_ = agent_manager;
// }

const agent::AgentManager* DynamicWorld::agent_manager() const {
  return &agent_manager_;
}
agent::AgentManager* DynamicWorld::mutable_agent_manager() {
  return &agent_manager_;
}

void DynamicWorld::Reset() {
  dynamic_agent_node_table_.clear();
  assigned_dynamic_agents_.clear();
}

bool DynamicWorld::ConstructDynamicWorld() {
  Reset();
  const auto& all_current_agents = agent_manager_.GetAllCurrentAgents();
  if (all_current_agents.empty()) {
    return true;
  }

  const auto& ego_pose = session_->mutable_environmental_model()
                             ->get_ego_state_manager()
                             ->ego_pose();
  auto& lane_manager =
      *session_->mutable_environmental_model()->get_virtual_lane_manager();
  // auto& refline_manager =
  //     *session_->mutable_environmental_model()->get_reference_path_manager();

  trajectory::TrajectoryPoint ego_state{ego_pose.x, ego_pose.y, ego_pose.theta,
                                        0,          0,          0};

  // build nodes for all agents
  std::vector<int32_t> lane_ids;
  for (const auto& lane : lane_manager.get_virtual_lanes()) {
    if (lane == nullptr) {
      continue;
    }
    lane_ids.emplace_back(lane->get_virtual_id());
  }

  std::unordered_set<int32_t> lane_id_set;
  for (const auto& lane_id : lane_ids) {
    if (lane_id_set.find(lane_id) != lane_id_set.end()) {
      continue;
    }
    lane_id_set.insert(lane_id);
    auto static_lane = lane_manager.get_lane_with_virtual_id(lane_id);
    if (static_lane == nullptr) {
      continue;
    }
    auto ref_line = static_lane->get_reference_path();
    // auto ref_line = refline_manager.get_reference_path_by_lane(lane_id);
    if (ref_line == nullptr) {
      continue;
    }
    // loop through all dynamic_objects
    std::unordered_set<int32_t> agent_id_set;
    auto time_start = IflyTime::Now_ms();
    for (const auto& agent : all_current_agents) {
      if (agent == nullptr) {
        continue;
      }
      if (!(agent->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
        continue;
      }
      if (agent_id_set.find(agent->agent_id()) != agent_id_set.end()) {
        continue;
      }
      agent_id_set.insert(agent->agent_id());
      auto agent_node = std::make_unique<DynamicAgentNode>(
          agent, *static_lane, *ref_line, ego_state);
      if (agent_node == nullptr || !agent_node->is_valid()) {
        continue;
      }
      if (dynamic_agent_node_table_.find(agent_node->node_id()) !=
          dynamic_agent_node_table_.end()) {
        continue;
      }
      if (agent_node->is_cone_type()) {
        cone_nodes_.emplace_back(agent_node.get());
      }
      StoreDynamicAgentNodeByStaticLane(static_lane->get_virtual_id(),
                                        agent_node->node_id());
      auto node_id = agent_node->node_id();
      dynamic_agent_node_table_[node_id] = std::move(agent_node);
    }
    auto time_end = IflyTime::Now_ms();
    LOG_DEBUG("agent node construct cost:%f\n", time_end - time_start);
  }
  // ========= build node connections
  auto time_start = IflyTime::Now_ms();
  bool is_success = BuildConnections(&lane_manager, ego_state);
  auto time_end = IflyTime::Now_ms();
  LOG_DEBUG("agent connection cost:%f\n", time_end - time_start);
  return is_success;
}

void DynamicWorld::StoreDynamicAgentNodeByStaticLane(const int32_t& lane_id,
                                                     const int64_t& node_id) {
  if (assigned_dynamic_agents_.find(lane_id) ==
      assigned_dynamic_agents_.end()) {
    assigned_dynamic_agents_[lane_id] = {node_id};
    return;
  }
  assigned_dynamic_agents_[lane_id].emplace_back(node_id);
}

bool DynamicWorld::BuildConnections(
    const VirtualLaneManager* lane_manager,
    const trajectory::TrajectoryPoint& ego_state) {
  // sort agent_nodes on each static_lane, order: front -> rear
  auto comp = [this](const int64_t& a, const int64_t& b) {
    // the key must exist in dynamic_agent_node_table_,because we check it at
    // tmp_lane_agent_ids
    auto* ptr_a = this->dynamic_agent_node_table_.at(a).get();
    auto* ptr_b = this->dynamic_agent_node_table_.at(b).get();
    return ptr_a->node_s() > (ptr_b->node_s() + 1e-10);
  };
  for (auto& lane_agents_entry : assigned_dynamic_agents_) {
    auto& lane_agent_ids = lane_agents_entry.second;

    std::vector<int64_t> tmp_lane_agent_ids;
    if (lane_agent_ids.size() < 1) {
      return false;
    }
    tmp_lane_agent_ids.reserve(lane_agent_ids.size());
    for (const int64_t& agent_node_id : lane_agent_ids) {
      const auto iter = dynamic_agent_node_table_.find(agent_node_id);
      if (iter == dynamic_agent_node_table_.end() || iter->second == nullptr) {
        continue;
      }
      tmp_lane_agent_ids.emplace_back(agent_node_id);
    }
    std::sort(tmp_lane_agent_ids.begin(), tmp_lane_agent_ids.end(), comp);
    lane_agent_ids.swap(tmp_lane_agent_ids);
  }

  // get front/rear neightbor for each node
  for (auto& lane_agents_entry : assigned_dynamic_agents_) {
    auto& lane_agents = lane_agents_entry.second;
    if (lane_agents.size() < 2) {
      continue;
    }
    for (int i = 0; i < lane_agents.size() - 1; i++) {
      const int64_t cur_node_id = lane_agents[i];
      const int64_t next_node_id = lane_agents[i + 1];

      auto* cur_node = GetNode(cur_node_id);
      auto* next_node = GetNode(next_node_id);

      if (cur_node == nullptr || next_node == nullptr) {
        continue;
      }
      auto* mutable_cur_node = GetMutableNode(cur_node->node_id());
      auto* mutable_next_node = GetMutableNode(next_node->node_id());
      if (mutable_cur_node == nullptr || mutable_next_node == nullptr) {
        continue;
      }
      mutable_cur_node->set_rear_node_id(next_node->node_id());
      mutable_next_node->set_front_node_id(cur_node->node_id());
    }
  }

  // get ego's eight neighbor node
  BuildConnectionForEgoLane(lane_manager, ego_state);
  BuildConnectionForNeighborLane(lane_manager, ego_state, true);
  BuildConnectionForNeighborLane(lane_manager, ego_state, false);

  return true;
}

void DynamicWorld::BuildConnectionForEgoLane(
    const VirtualLaneManager* lane_manager,
    const trajectory::TrajectoryPoint& ego_state) {
  const auto& ego_lane = lane_manager->get_current_lane();
  if (ego_lane == nullptr) {
    return;
  }
  const auto& ego_ref_line = ego_lane->get_reference_path();
  if (ego_ref_line == nullptr) {
    std::vector<planning_math::PathPoint> ego_center_line_points;
    if (ego_lane->lane_points().size() < 1) {
      return;
    }
    ego_center_line_points.reserve(ego_ref_line->get_points().size());
    for (const auto& ref_line_point : ego_ref_line->get_points()) {
      planning_math::PathPoint path_point{ref_line_point.path_point.x,
                                          ref_line_point.path_point.y};
      ego_center_line_points.emplace_back(path_point);
    }
    ego_lane_coord_ =
        std::make_shared<KDPath>(std::move(ego_center_line_points));
    return;
  } else {
    ego_lane_coord_ = ego_ref_line->get_frenet_coord();
  }

  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!ego_lane_coord_->XYToSL(ego_state.x(), ego_state.y(), &ego_s, &ego_l)) {
    return;
  }
  auto ego_lane_assigned_agent_entery =
      assigned_dynamic_agents_.find(ego_lane->get_virtual_id());
  if (ego_lane_assigned_agent_entery == assigned_dynamic_agents_.end()) {
    return;
  }
  auto ego_lane_assigned_agents = ego_lane_assigned_agent_entery->second;
  double min_distance_in_front = std::numeric_limits<double>::max();
  int64_t front_agent_node_id = kInvalidId;
  double min_distance_in_rear = std::numeric_limits<double>::max();
  int64_t rear_agent_node_id = kInvalidId;
  for (const int64_t& ego_lane_agent_node_id : ego_lane_assigned_agents) {
    auto* ego_lane_agent_node = GetNode(ego_lane_agent_node_id);
    if (ego_lane_agent_node == nullptr) {
      continue;
    }
    double cur_distance = ego_lane_agent_node->node_s() - ego_s;
    if (cur_distance >= 0.0) {
      if (cur_distance < min_distance_in_front) {
        min_distance_in_front = cur_distance;
        front_agent_node_id = ego_lane_agent_node->node_id();
      }
    } else {
      if (std::fabs(cur_distance) < min_distance_in_rear) {
        min_distance_in_rear = std::fabs(cur_distance);
        rear_agent_node_id = ego_lane_agent_node->node_id();
      }
    }
  }
  ego_front_node_id_ = front_agent_node_id;
  ego_rear_node_id_ = rear_agent_node_id;
  JSON_DEBUG_VALUE("front_node_id", front_agent_node_id & 0xFFFF)
  JSON_DEBUG_VALUE("rear_node_id", rear_agent_node_id & 0xFFFF)
}

void DynamicWorld::BuildConnectionForNeighborLane(
    const VirtualLaneManager* lane_manager,
    const trajectory::TrajectoryPoint& ego_state, const bool is_left) {
  auto ego_lane = lane_manager->get_current_lane();
  if (ego_lane == nullptr) {
    return;
  }
  int32_t neighbor_lane_id;
  if (is_left) {
    if (lane_manager->get_left_lane() == nullptr) {
      return;
    }
    auto left_lane_id = lane_manager->get_left_lane()->get_virtual_id();
    // if (left_lane_id == -1) {
    //   return;
    // }
    neighbor_lane_id = left_lane_id;
  } else {
    if (lane_manager->get_right_lane() == nullptr) {
      return;
    }
    auto right_lane_id = lane_manager->get_right_lane()->get_virtual_id();
    // if (right_lane_id == -1) {
    //   return;
    // }
    neighbor_lane_id = right_lane_id;
  }
  const auto& neighbor_lane =
      lane_manager->get_lane_with_virtual_id(neighbor_lane_id);
  if (neighbor_lane == nullptr) {
    return;
  }
  double ego_s = 0.0;
  double ego_l = 0.0;

  const auto& neighbor_ref_line = neighbor_lane->get_reference_path();
  if (neighbor_ref_line == nullptr) {
    std::vector<planning_math::PathPoint> center_line_points;
    if (neighbor_ref_line->get_points().size() < 1) {
      return;
    }
    center_line_points.reserve(neighbor_ref_line->get_points().size());
    for (const auto& ref_line_point : neighbor_ref_line->get_points()) {
      planning_math::PathPoint path_point{ref_line_point.path_point.x,
                                          ref_line_point.path_point.y};
      center_line_points.emplace_back(path_point);
    }
    neighbor_lane_coord_ =
        std::make_shared<KDPath>(std::move(center_line_points));
    return;
  } else {
    neighbor_lane_coord_ = neighbor_ref_line->get_frenet_coord();
  }

  if (!neighbor_lane_coord_->XYToSL(ego_state.x(), ego_state.y(), &ego_s,
                                    &ego_l)) {
    return;
  }
  auto neighbor_lane_assigned_agent_entery =
      assigned_dynamic_agents_.find(neighbor_lane->get_virtual_id());
  if (neighbor_lane_assigned_agent_entery == assigned_dynamic_agents_.end()) {
    return;
  }
  auto neighbor_lane_assigned_agents =
      neighbor_lane_assigned_agent_entery->second;
  double min_distance = std::numeric_limits<double>::max();
  int64_t closest_agent_node_id = kInvalidId;
  for (const int64_t& neighbor_lane_agent_node_id :
       neighbor_lane_assigned_agents) {
    auto* neighbor_lane_agent_node = GetNode(neighbor_lane_agent_node_id);
    if (neighbor_lane_agent_node == nullptr) {
      continue;
    }
    double cur_distance = std::fabs(neighbor_lane_agent_node->node_s() - ego_s);
    if (cur_distance < min_distance) {
      min_distance = cur_distance;
      closest_agent_node_id = neighbor_lane_agent_node->node_id();
    }
  }
  if (closest_agent_node_id == kInvalidId) {
    return;
  }

  auto* closest_agent_node = GetNode(closest_agent_node_id);
  if (closest_agent_node == nullptr) {
    return;
  }
  int64_t neighbor_node_id = kInvalidId;
  int64_t neighbor_front_id = kInvalidId;
  int64_t neighbor_rear_id = kInvalidId;
  if (closest_agent_node->is_agent_has_overlap_with_ego()) {
    if (!closest_agent_node->is_cone_type()) {
      neighbor_node_id = closest_agent_node->node_id();
    }
    neighbor_front_id = GetNodeFrontNotConeNode(closest_agent_node);
    neighbor_rear_id = GetNodeRearNotConeNode(closest_agent_node);
  } else {
    if (closest_agent_node->node_s() > ego_s) {
      if (!closest_agent_node->is_cone_type()) {
        neighbor_front_id = closest_agent_node->node_id();
      } else {
        neighbor_front_id = GetNodeFrontNotConeNode(closest_agent_node);
      }

      neighbor_rear_id = GetNodeRearNotConeNode(closest_agent_node);
      LOG_DEBUG(
          "closest_agent_node id = %ld, node s: %.2f, is cone type %d, "
          "neighbor_front_id %ld",
          closest_agent_node->node_id(), closest_agent_node->node_s(),
          closest_agent_node->is_cone_type(), neighbor_front_id);
    } else {
      neighbor_front_id = closest_agent_node->front_node_id();
      if (!closest_agent_node->is_cone_type()) {
        neighbor_rear_id = closest_agent_node->node_id();
      } else {
        neighbor_rear_id = GetNodeRearNotConeNode(closest_agent_node);
      }
    }
  }
  LOG_DEBUG(
      "dynamic world neighbor_node_id = %ld, neighbor_front_id = %ld, "
      "neighbor_rear_id = %ld",
      neighbor_node_id, neighbor_front_id, neighbor_rear_id);

  if (is_left) {
    ego_left_node_id_ = neighbor_node_id;
    ego_left_front_node_id_ = neighbor_front_id;
    ego_left_rear_node_id_ = neighbor_rear_id;
    JSON_DEBUG_VALUE("ego_left_node", neighbor_node_id & 0xFFFF)
    JSON_DEBUG_VALUE("ego_left_front_node", neighbor_front_id & 0xFFFF)
    JSON_DEBUG_VALUE("ego_left_rear_node", neighbor_rear_id & 0xFFFF)
  } else {
    ego_right_node_id_ = neighbor_node_id;
    ego_right_front_node_id_ = neighbor_front_id;
    ego_right_rear_node_id_ = neighbor_rear_id;
    JSON_DEBUG_VALUE("ego_right_node", neighbor_node_id & 0xFFFF)
    JSON_DEBUG_VALUE("ego_right_front_node", neighbor_front_id & 0xFFFF)
    JSON_DEBUG_VALUE("ego_right_rear_node", neighbor_rear_id & 0xFFFF)
  }
}

const int64_t DynamicWorld::GetNodeFrontNotConeNode(
    const DynamicAgentNode* node) {
  const int64_t front_node_id = node->front_node_id();
  auto* front_node = GetNode(front_node_id);
  if (front_node == nullptr) {
    return kInvalidId;
  }

  while (front_node->is_cone_type()) {
    int64_t front_front_node_id = front_node->front_node_id();
    if (front_front_node_id == kInvalidId) {
      return kInvalidId;
    }
    front_node = GetNode(front_front_node_id);

    if (front_node == nullptr) {
      return kInvalidId;
    }
  }

  return front_node->node_id();
}

const int64_t DynamicWorld::GetNodeRearNotConeNode(
    const DynamicAgentNode* node) {
  const int64_t rear_node_id = node->rear_node_id();
  auto* rear_node = GetNode(rear_node_id);
  if (rear_node == nullptr) {
    return kInvalidId;
  }

  while (rear_node->is_cone_type()) {
    int64_t rear_rear_node_id = rear_node->rear_node_id();
    if (rear_rear_node_id == kInvalidId) {
      return kInvalidId;
    }
    rear_node = GetNode(rear_rear_node_id);

    if (rear_node == nullptr) {
      return kInvalidId;
    }
  }

  return rear_node->node_id();
}

const DynamicAgentNode* DynamicWorld::GetNode(const int64_t& node_id) const {
  auto iter = dynamic_agent_node_table_.find(node_id);
  return iter == dynamic_agent_node_table_.end() ? nullptr : iter->second.get();
}

DynamicAgentNode* DynamicWorld::GetMutableNode(const int64_t& node_id) const {
  auto iter = dynamic_agent_node_table_.find(node_id);
  return iter == dynamic_agent_node_table_.end() ? nullptr : iter->second.get();
}

std::vector<const DynamicAgentNode*> DynamicWorld::GetNodesByLaneId(
    const int32_t lane_id) const {
  std::vector<const DynamicAgentNode*> nodes_on_lane;
  auto iter = assigned_dynamic_agents_.find(lane_id);
  if (iter == assigned_dynamic_agents_.end()) {
    return nodes_on_lane;
  }
  const auto& node_ids = iter->second;
  for (const auto& node_id : node_ids) {
    auto* node = GetNode(node_id);
    if (node == nullptr) {
      continue;
    }
    nodes_on_lane.emplace_back(node);
  }
  return nodes_on_lane;
}

std::vector<const DynamicAgentNode*> DynamicWorld::GetConeNodes() const {
  return cone_nodes_;
}

const int64_t DynamicWorld::ego_front_node_id() const {
  return ego_front_node_id_;
}
const int64_t DynamicWorld::ego_rear_node_id() const {
  return ego_rear_node_id_;
}
const int64_t DynamicWorld::ego_left_node_id() const {
  return ego_left_node_id_;
}
const int64_t DynamicWorld::ego_right_node_id() const {
  return ego_right_node_id_;
}
const int64_t DynamicWorld::ego_left_front_node_id() const {
  return ego_left_front_node_id_;
}
const int64_t DynamicWorld::ego_right_front_node_id() const {
  return ego_right_front_node_id_;
}
const int64_t DynamicWorld::ego_left_rear_node_id() const {
  return ego_left_rear_node_id_;
}
const int64_t DynamicWorld::ego_right_rear_node_id() const {
  return ego_right_rear_node_id_;
}

}  // namespace planning_data
}  // namespace planning