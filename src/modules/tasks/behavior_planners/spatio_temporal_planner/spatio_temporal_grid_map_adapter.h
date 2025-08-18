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
#include "reference_path.h"
#include "session.h"
#include "slt_point.h"
#include "src/common/vec2d.h"
#include "src/modules/common/config/vehicle_param.h"
#include "src/modules/common/math/box2d.h"
#include "src/modules/tasks/behavior_planners/spatio_temporal_planner/grid_map.h"
#include "src/modules/tasks/behavior_planners/spatio_temporal_planner/ssc_map.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "utils_math.h"
#include "virtual_lane.h"

namespace planning {

struct State {
  double time_stamp{0.0};
  planning_math::Vec2d vec_position;
  double angle{0.0};
  double curvature{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double steer{0.0};
};

class SLTGridMapAdapter {
 public:
  using ObstacleMapType = uint8_t;
  using SscMapDataType = uint8_t;

  using GridMap2D = GridMapND<ObstacleMapType, 2>;

 public:
  SLTGridMapAdapter() = default;
  SLTGridMapAdapter(const EgoPlanningConfigBuilder *config_builder,
                    framework::Session *session);
  // SLTGridMapAdapter() = default;
  ~SLTGridMapAdapter() = default;

  /**
   * @brief setters
   */

  void set_initial_state(const State &state);
  /**
   * @brief getters
   */
  SscMap *p_ssc_map() const { return p_ssc_map_; }

  std::string Name();

  /**
   * @brief Initialize the planner with config path
   */
  void Init(const std::string config_path);

  /**
   * @brief Run one planning round with given states
   */
  void RunOnce();

  const std::vector<AgentFrenetSpatioTemporalInFo>
      &SurroundForwardAgentsTrajs() {
    return surround_forward_trajs_state_;
  }

  const std::vector<VirtualAgentSpatioTemporalInFo> &VirtualAgentSTInFo() {
    return virtual_agents_st_info_;
  }

  SpatioTemporalGridMap config_;
  planning::framework::Session *session_;
  std::shared_ptr<ReferencePath> reference_path_ = nullptr;
  std::vector<std::shared_ptr<agent::Agent>> static_obstacles_;
  std::vector<std::shared_ptr<agent::Agent>> dynamic_obstacles_;
  std::vector<std::shared_ptr<agent::Agent>> virtual_agents_;
  std::vector<std::shared_ptr<agent::Agent>> consider_surround_agents_;
  PlanningInitPoint planning_init_point_;
  FrenetEgoState ego_frenet_state_;

  std::shared_ptr<KDPath> current_lane_coord_;
  std::array<double, 3> ego_planning_init_s_;
  std::array<double, 2> ego_cart_point_;
  std::array<double, 2> planning_init_cart_point_;
  std::array<double, 2> planning_init_frenet_point_;

 private:
  /**
   * @brief transform all the states in a batch
   */
  void StateTransformForInputData();

  void GetVehicleVertices(const State &state,
                          std::vector<planning_math::Vec2d> *vertices);

  void GetAgentVertices(const agent::Agent &agent, const State &state,
                        std::vector<planning_math::Vec2d> *vertices);
  /**
   * @brief transform all the states using openmp
   */
  // STErrorType StateTransformUsingOpenMp(const vec_E<State>& global_state_vec,
  //                                     const vec_E<Vec2f>& global_point_vec,
  //                                     vec_E<FrenetState>* frenet_state_vec,
  //                                     vec_E<Vec2f>* fs_point_vec) const;

  void AgentVerticesTransform(
      const std::vector<planning_math::Vec2d> &global_point_vec,
      std::vector<planning_math::Vec2d> *fs_point_vec);

  void EgoVerticesTransform(
      const std::array<planning_math::Vec2d, 4> &global_point_vec,
      std::array<planning_math::Vec2d, 4> &fs_point_vec);

  void UpdateTrajectoryWithCurrentBehavior();

 private:
  double time_origin_{0.0};

  State initial_state_;
  bool has_initial_state_ = false;

  AgentFrenetSpatioTemporalInFo ego_vehicle_initial_state_;
  // std::vector<AgentFrenetSpatioTemporalInFo> surround_forward_trajs_state_;
  std::vector<AgentFrenetSpatioTemporalInFo> surround_forward_trajs_state_;

  std::vector<VirtualAgentSpatioTemporalInFo> virtual_agents_st_info_;

  double origin_lane_width_ = 3.75;

  bool use_query_lane_width_ = false;

  std::vector<std::pair<double, double>>
      current_lane_s_width_;  // <s, lane_width>

  // Map
  bool map_valid_ = false;

  SscMap *p_ssc_map_;

  double ego_front_consider_obstacle_distance_ = 50.0;
};

}  // namespace planning