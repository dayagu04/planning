#pragma once

#include <memory>
#include <vector>

// #include "agent/agent.h"
// #include "agent_node_manager.h"
#include <assert.h>
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "filters.h"
#include "grid_map.h"
#include "reference_path.h"
#include "session.h"
#include "slt_graph_point.h"
#include "spatio_temporal_union_dp.h"
#include "speed_data.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "utils_math.h"
#include "virtual_lane.h"

namespace planning {

// struct SLTGraphMessage {
//   SLTGraphMessage(const uint32_t c_, const int32_t r_, const int32_t k_) :
//   c(c_), r(r_), k(k_){} uint32_t c; uint32_t r; uint32_t k;
// };
class GriddedPathTimeGraph {
 public:
  GriddedPathTimeGraph(const EgoPlanningConfigBuilder *config_builder,
                       framework::Session *session);

  ~GriddedPathTimeGraph() = default;

  bool Search(
      TrajectoryPoints &traj_points,
      const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
      const std::vector<VirtualAgentSpatioTemporalInFo> &virtual_agents_info,
      const bool &last_enable_using_st_plan);

 private:
  planning::framework::Session *session_;

  SpatioTemporalUnionDp spatio_temporal_union_plan_dp_;

  // dp configuration
  DpStSpeedOptimizerConfig gridded_path_time_graph_config_;

  DpPolyPathConfig dp_poly_path_config_;

  // speed_limit configuration
  SpeedLimitConfig speed_limit_config_;

  // 当前车道中心线kd_path
  std::shared_ptr<KDPath> current_lane_coord_;

  std::shared_ptr<ReferencePath> current_refline_;

  Point2D ego_frenet_pose_;

  // initial status
  PlanningInitPoint planning_init_point_;

  double total_length_s_ = 200.0;

  double max_acceleration_ = 0.0;

  double max_deceleration_ = 0.0;

  int half_lateral_sample_nums_ = 1;
};

}  // namespace planning
