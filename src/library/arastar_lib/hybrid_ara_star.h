#pragma once

#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "hybrid_ara_data.h"
#include "node3d.h"
#include "src/library/arastar_lib/cost/agent_cost.h"
#include "src/library/arastar_lib/cost/boundary_cost.h"
#include "src/library/arastar_lib/cost/center_cost.h"
#include "src/library/arastar_lib/cost/cost_manager.h"
#include "src/library/arastar_lib/cost/motion_cost.h"
#include "ifly_time.h"
#include "src/modules/context/lateral_obstacle.h"
#include "src/framework/session.h"
#include "src/modules/tasks/task.h"
#include "src/modules/common/utils/index_list.h"
#include "src/modules/context/virtual_lane_manager.h"

namespace planning {

class HybridARAStar {
 public:
  explicit HybridARAStar(framework::Session* session);
  HybridARAStar();

  virtual ~HybridARAStar() = default;

  bool Plan(ara_star::HybridARAStarResult& result);

 private:
  bool ProcessStaticAgents();
  void BuildAgentKDTree(const std::vector<const Obstacle*>& nudge_agents);
  void BuildVirturalKDTree(
      const std::vector<planning_math::LineSegment2d>& virtual_lineseg_vec);
  bool SetStartAndEndPose(const PlanningInitPoint& planning_init_point,
                          const std::shared_ptr<KDPath>& target_lane,
                          const vector<TrajectoryPoint>& plan_history_traj,
                          const double target_v);
  bool ValidityCheck(const std::shared_ptr<Node3D> node) const;
  planning::planning_math::Box2d GetBoundingBox(const double x, const double y,
                                                const double phi) const;
  double CalculateBaseHeuCost(const std::shared_ptr<Node3D> current_node) const;
  bool ImprovePath();
  void RegisterCost(ara_star::CostManager& cost_manager) const;
  std::shared_ptr<Node3D> NextNodeGenerator(
      const std::shared_ptr<Node3D> current_node,
      const size_t next_node_index);
  bool ReachDestination(const std::shared_ptr<Node3D> node) const;
  void SetMiddleFinalNode();
  bool GetResult(ara_star::HybridARAStarResult& result) const;
  void UpdateHeuristicFactor();
  void UpdateOpenSetWithHeuristicFactor();
  void Reset();
  bool Init();

 private:
  framework::Session* session_;
  HybridAraStarConfig hybrid_ara_star_conf_;
  HppGeneralLateralDeciderConfig hpp_general_lateral_decider_config_;
  VehicleParam vehicle_param_;
  size_t next_node_num_ = 0;  // should be odd, to keep 0 as going straight
  double max_front_wheel_angle_ = 0.0;
  double step_size_ = 0.0;
  double one_shot_distance_ = 0.0;
  double x_grid_resolution_ = 0.0;
  double y_grid_resolution_ = 0.0;
  double phi_grid_resolution_ = 0.0;
  double heuristic_factor_ = 5.0;
  double end_s_ = 0.0;
  double end_l_ = 0.0;
  double lane_width_ = 4.0;
  int num_node_expand_ = 0;
  bool is_in_lane_change_execution_ = false;
  uint8_t lane_change_direction_ = 0;
  double init_v_ = 0.0;
  Pose2D start_pose_;
  Pose2D end_pose_;
  double hard_safe_distance_ = 0.0;
  double soft_safe_distance_ = 0.0;
  double ego_half_width_ = 0.0;
  double ego_half_length_ = 0.0;
  double center_cost_weight_ = 0.0;
  double agent_cost_weight_ = 0.0;
  double boundary_cost_weight_ = 0.0;
  double motion_cost_weight_ = 0.0;
  double boundary_soft_extra_buffer_ = 0.0;
  double crosslinebuffer_ = 0.0;
  double obs_max_s_ = std::numeric_limits<double>::lowest();
  double obs_min_s_ = std::numeric_limits<double>::max();
  bool enable_middle_final_node_ = false;
  double l_limit_ = 0.0;
  double collision_buffer_ = 0.1;
  double front_obs_s_ = 0.0;
  double rear_obs_s_ = 0.0;
  uint64_t start_search_time_ = 0;
  planning::planning_math::Box2d ego_box_;
  double expand_num_ = 0.0;
  std::vector<double> last_result_s_;
  std::shared_ptr<ReferencePath> reference_path_ptr_ = nullptr;
  double ego_s_ = 0.0;
  double ego_l_ = 0.0;
  double last_ego_s_ = 0.0;
  double ref_init_point_x_ = 0.0;
  double ref_init_point_y_ = 0.0;
  double compensation_s_ = 0.0;

  // map bound
  std::vector<double> XYbounds_;
  std::shared_ptr<KDPath> fix_lane_ = nullptr;

  std::shared_ptr<Node3D> start_node_;
  std::shared_ptr<Node3D> end_node_;
  std::shared_ptr<Node3D>
      final_node_;  // reach destination not need fully match end node

  // bounding box for cost
  std::vector<ara_star::SLBox2d> bounding_box_vec_;
  std::shared_ptr<planning_math::AABoxKDTree2d<
      planning_math::GeometryObject>>
      agent_box_tree_;
  std::shared_ptr<planning_math::AABoxKDTree2d<
      planning_math::GeometryObject>>
      virtual_lineseg_tree_;

  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq_;
  std::unordered_map<std::string, std::shared_ptr<Node3D>> open_set_;
  std::unordered_map<std::string, std::shared_ptr<Node3D>> close_set_;
  std::unordered_map<std::string, std::shared_ptr<Node3D>>
      observed_set_;  // has alaready visited
};

}  // namespace planning
