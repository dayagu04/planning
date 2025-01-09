#pragma once

#include <cstddef>
#include <memory>
#include "agent/agent_manager.h"
#include "dynamic_agent_node.h"
#include "ego_state_manager.h"
#include "session.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"

namespace planning {
namespace planning_data {

class DynamicWorld {
 public:
  DynamicWorld() = delete;

  DynamicWorld(agent::AgentManager& agent_manager, framework::Session* session)
      : session_(session), agent_manager_(agent_manager) {}

  ~DynamicWorld() = default;

  void UpdateAgentManager(agent::AgentManager* const agent_manager);
  const agent::AgentManager* agent_manager() const;
  agent::AgentManager* mutable_agent_manager();

  //   bool ConstructDynamicWorld(const trajectory::TrajectoryPoint& ego_state,
  //                              const VirtualLaneManager& lane_manager);
  bool ConstructDynamicWorld();

  const DynamicAgentNode* GetNode(const int64_t& node_id) const;

  std::vector<const DynamicAgentNode*> GetNodesByLaneId(
      const int32_t lane_id) const;

  std::vector<const DynamicAgentNode*> GetConeNodes() const;

  void DebugTrajectoryForNode(const int node_id,
                              const std::string& prefix) const;

  void DebugEgoNearByAgentNodesTrajectory() const;

  const int64_t ego_front_node_id() const;
  const int64_t ego_rear_node_id() const;
  const int64_t ego_left_node_id() const;
  const int64_t ego_right_node_id() const;
  const int64_t ego_left_front_node_id() const;
  const int64_t ego_right_front_node_id() const;
  const int64_t ego_left_rear_node_id() const;
  const int64_t ego_right_rear_node_id() const;

 private:
  void Reset();

  void StoreDynamicAgentNodeByStaticLane(const int32_t& lane_id,
                                         const int64_t& node_id);

  bool BuildConnections(const VirtualLaneManager* lane_manager,
                        const trajectory::TrajectoryPoint& ego_state);

  DynamicAgentNode* GetMutableNode(const int64_t& node_id) const;

  void BuildConnectionForEgoLane(const VirtualLaneManager* lane_manager,
                                 const trajectory::TrajectoryPoint& ego_state);
  void BuildConnectionForNeighborLane(
      const VirtualLaneManager* lane_manager,
      const trajectory::TrajectoryPoint& ego_state, const bool is_left);
  const int64_t GetNodeFrontNotConeNode(const DynamicAgentNode* node);

  const int64_t GetNodeRearNotConeNode(const DynamicAgentNode* node);

  void StoreNodeInfoInJsonDebug();

 private:
  framework::Session* session_ = nullptr;
  agent::AgentManager& agent_manager_;
  std::shared_ptr<planning_math::KDPath> ego_lane_coord_;
  std::shared_ptr<planning_math::KDPath> neighbor_lane_coord_;

  // key: node_id, value: dynamic_agent_node ptr
  std::unordered_map<int64_t, std::unique_ptr<DynamicAgentNode>>
      dynamic_agent_node_table_;
  // key: lane_id, value: vector of dynamic agent node ptrs on the static_lane
  std::unordered_map<int32_t, std::vector<int64_t>> assigned_dynamic_agents_;

  // ego's eight neighbor node
  int64_t ego_front_node_id_ = kInvalidId;
  int64_t ego_rear_node_id_ = kInvalidId;
  int64_t ego_left_node_id_ = kInvalidId;
  int64_t ego_right_node_id_ = kInvalidId;
  int64_t ego_left_front_node_id_ = kInvalidId;
  int64_t ego_right_front_node_id_ = kInvalidId;
  int64_t ego_left_rear_node_id_ = kInvalidId;
  int64_t ego_right_rear_node_id_ = kInvalidId;
  std::vector<const DynamicAgentNode*> cone_nodes_;
};

}  // namespace planning_data
}  // namespace planning