#pragma once

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <assert.h>
#include "agent/agent.h"
#include "agent_node_manager.h"
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "ego_planning_config.h"
#include "filters.h"
#include "grid_map.h"
#include "gridded_path_time_graph.h"
#include "reference_path.h"
#include "session.h"
#include "speed_data.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "utils_math.h"
#include "virtual_lane.h"

namespace planning {
using namespace planning_math;

class PathTimeHeuristicOptimizer {
 public:
  explicit PathTimeHeuristicOptimizer(
      const EgoPlanningConfigBuilder *config_builder,
      framework::Session *session);

  ~PathTimeHeuristicOptimizer() = default;

  DpStSpeedOptimizerConfig config_;

  bool Process(
      TrajectoryPoints &traj_points,
      const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
      const std::vector<VirtualAgentSpatioTemporalInFo> &virtual_agents_info,
      const bool &last_enable_using_st_plan);

  bool GetStDpIsSuccess() const { return st_dp_is_sucess_; }

  const std::array<AABox2d, 26> &GetEgoBoxSet() { return ego_box_set_; }

  void UpdateLateralObstacleDecision(
      const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs);

 private:
  bool SearchPathTimeGraph(
      TrajectoryPoints &traj_points,
      const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
      const std::vector<VirtualAgentSpatioTemporalInFo> &virtual_agents_info,
      const bool &last_enable_using_st_plan);

  void FallbackFunction(TrajectoryPoints &traj_points);

  void GenerateEgoBoxSet(TrajectoryPoints &traj_points);

  void GetVehicleVertices(const TrajectoryPoint &traj_point,
                          std::vector<planning_math::Vec2d> &vertices);

 private:
  planning::framework::Session *session_;

  std::shared_ptr<KDPath> base_frenet_coord_;

  std::array<AABox2d, 26> ego_box_set_;

  GriddedPathTimeGraph slt_graph_;
  PlanningInitPoint planning_init_point_;

  bool st_dp_is_sucess_ = false;
};

}  // namespace planning