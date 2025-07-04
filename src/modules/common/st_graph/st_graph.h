#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "agent/agent.h"
#include "math/box2d.h"
#include "math/geometry_object.h"
#include "math/line_segment2d.h"
#include "math/vec2d.h"
#include "planning_debug_info.pb.h"
#include "speed/st_point.h"
#include "st_boundary.h"
#include "st_graph.pb.h"
#include "st_graph/st_point.h"
#include "st_graph_input.h"

namespace planning {
namespace speed {

class STGraph {
 public:
  struct NeighborCorridorYieldInfo {
    int32_t first_yield_index = std::numeric_limits<int32_t>::max();
    int32_t first_yield_agent_id = agent::AgentDefaultInfo::kNoAgentId;
    STPoint first_yield_st_point = STPoint::HighestSTPoint();
  };
  STGraph() = default;

  ~STGraph() = default;

  /*******common functions*******/
  bool Init(const std::shared_ptr<StGraphInput>& st_graph_input);

  /*******normal st-graph functions*******/
  bool UpdateStBoundaryDecisionResults(
      const std::unordered_map<int64_t, STBoundary::DecisionType>&
          decision_table);

  /*******neighbor st-graph functions*******/

  bool InsertAgent(const agent::Agent& agent, const StBoundaryType type);

  bool UpdateNeighborAgentResults(
      const std::unordered_map<int32_t, STBoundary::DecisionType>&
          neighbor_agent_decision_table);

  bool UpdateNeighborAgentResultsForEgoMotionSimPath();

  /*******st_graph_helper functions*******/
  const std::shared_ptr<StGraphInput>& st_graph_input() const;

  const std::pair<double, double> path_range() const;

  const std::pair<double, double> time_range() const;

  const int32_t reserve_num() const;

  const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
  boundary_id_st_boundaries_map() const;

  const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
  neighbor_boundary_id_st_boundaries_map() const;

  const std::unordered_map<int32_t, std::vector<int64_t>>&
  neighbor_agent_id_st_boundaries_map() const;

  const int32_t first_neighbor_yield_agent_id() const;

  const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
  expand_boundary_id_st_boundaries_map() const;

  const std::unordered_map<int32_t, std::vector<int64_t>>&
  expand_agent_id_st_boundaries_map() const;

  const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
  close_pass_boundary_id_st_boundaries_map() const;

  const std::unordered_map<int32_t, std::vector<int64_t>>&
  close_pass_agent_id_st_boundaries_map() const;

  const std::vector<std::vector<std::pair<STPoint, STPoint>>>& st_points_table()
      const;

  const std::unordered_map<int32_t, std::vector<int64_t>>&
  agent_id_st_boundaries_map() const;

  const std::vector<std::pair<STPoint, STPoint>>& st_pass_corridor() const;

  const std::vector<std::pair<STPoint, STPoint>>& neighbor_corridor() const;
  const int32_t first_neighbor_yield_index() const;
  const int32_t first_neighbor_overtake_index() const;

  const std::vector<int32_t>& caution_yield_agent_ids() const;
  /*******st_graph_helper* functions(end)*******/
 private:
  void MakeAgentStBoundaries();

  void MakeStaticAgentStBoundary(
      const agent::Agent& agent, const StBoundaryType type,
      const bool reuse_for_close_pass = false,
      const double extra_lateral_buffer_for_close_pass = 0.0);

  void MakeDynamicAgentStBoundary(
      const agent::Agent& agent, const StBoundaryType type,
      const bool reuse_for_close_pass = false,
      const double extra_lateral_buffer_for_close_pass = 0.0);

  void MakeStPointsTable();

  void ConstructDefaultStPassCorridor();

  void ResetNeighborCorridor();

  bool CalculateStPassCorridor();
  bool CalculateNeighborCorridor();

  void BackwardExtendStBoundaries();

  void BackwardExtendSingleStBoundary(const agent::Agent& agent,
                                      const trajectory::Trajectory& trajectory,
                                      const STBoundary& st_boundary,
                                      const size_t idx);

  void AddStGraphDataToProto();

  void Reset();

 private:
  std::shared_ptr<StGraphInput> st_graph_input_;

  planning::common::StGraphData st_graph_data_pb_;

  std::unordered_map<int64_t, std::unique_ptr<STBoundary>>
      boundary_id_st_boundaries_map_;
  std::unordered_map<int32_t, std::vector<int64_t>> agent_id_st_boundaries_map_;
  std::unordered_map<int64_t, std::unique_ptr<STBoundary>>
      neighbor_boundary_id_st_boundaries_map_;
  std::unordered_map<int32_t, std::vector<int64_t>>
      neighbor_agent_id_st_boundaries_map_;
  std::unordered_map<int64_t, std::unique_ptr<STBoundary>>
      expand_boundary_id_st_boundaries_map_;
  std::unordered_map<int32_t, std::vector<int64_t>>
      expand_agent_id_st_boundaries_map_;
  // <key: time_index, value<key: boundry_id value: boundary>: boundaries>
  std::vector<std::vector<std::pair<STPoint, STPoint>>> st_points_table_;

  // st_pass_corridor: order is <upper_point, lower_point>
  std::vector<std::pair<STPoint, STPoint>> st_pass_corridor_;

  // bellow is for neighbor corridor
  std::vector<std::pair<STPoint, STPoint>> neighbor_corridor_;
  int32_t first_neighbor_yield_index_ = std::numeric_limits<int32_t>::max();
  int32_t first_neighbor_overtake_index_ = std::numeric_limits<int32_t>::max();
  int32_t first_neighbor_yield_agent_id_ = agent::AgentDefaultInfo::kNoAgentId;
  std::vector<int32_t> caution_yield_agent_ids_;
  std::vector<int32_t> ignore_agent_ids_;
  // <double, NeighborCorridorYieldInfo> : <yield_st_point.s(),
  // NeighborCorridorYieldInfo>
  std::map<double, NeighborCorridorYieldInfo> neighbor_corridor_yield_info_map_;

  // bellow is for close-pass st boundary
  std::unordered_set<int32_t> static_close_pass_candicate_agent_ids_;
  std::unordered_set<int32_t> dynamic_close_pass_candicate_agent_ids_;
  std::unordered_map<int64_t, std::unique_ptr<STBoundary>>
      close_pass_boundary_id_st_boundaries_map_;
  std::unordered_map<int32_t, std::vector<int64_t>>
      close_pass_agent_id_st_boundaries_map_;
};
}  // namespace speed
}  // namespace planning