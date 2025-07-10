/**
 * @file spatio_temporal_union_planner.h
 **/

#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "agent/agent.h"
#include "agent_node_manager.h"
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "filters.h"
#include "path_time_heuristic_optimizer.h"
#include "reference_path.h"
#include "session.h"
#include "spatio_temporal_grid_map_adapter.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "utils_math.h"
#include "virtual_lane.h"

namespace planning {
using namespace planning_math;

class SpatioTemporalPlanner : public Task {
 public:
  explicit SpatioTemporalPlanner(const EgoPlanningConfigBuilder *config_builder,
                                 framework::Session *session);
  ~SpatioTemporalPlanner() = default;

  bool Execute() override;

 private:
  void LogDebugInfo(
      const TrajectoryPoints &traj_points,
      const std::vector<AgentFrenetSpatioTemporalInFo> &agents_state);

  void UpdateIntersection();

  EgoPlanningConfig config_;

  planning::framework::Session *session_;

  SLTGridMapAdapter slt_grid_map_;

  PathTimeHeuristicOptimizer path_time_heuristic_optimizer_;
  bool ego_in_intersection_state_ = false;
  int intersection_count_ = 0;
  bool last_enable_using_st_plan_ = false;

};

}  // namespace planning
