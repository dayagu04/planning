#pragma once
#include "base_cost.h"
#include "frenet_obstacle.h"
#include "obstacle.h"
#include "src/library/arastar_lib/hybrid_ara_data.h"
#include "utils/kd_path.h"

using namespace planning::planning_math;
namespace planning {
namespace ara_star {

class AgentCost : public BaseCost {
 public:
  AgentCost(
      const double weight, const double ego_wheel_base, const double ego_length,
      const double ego_circle_radius, const std::vector<SLBox2d>& obstacles,
      const std::shared_ptr<KDPath>& ego_lane,
      const std::shared_ptr<AABoxKDTree2d<GeometryObject>>& agent_box_tree,
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const std::vector<std::shared_ptr<planning::FrenetObstacle>>&
          nudge_agents,
      const double front_edge_to_rear_axle, const double rear_edge_to_rear_axle,
      const std::pair<double, double> pass_interval, const bool left_turn,
      const bool right_turn);
  ~AgentCost() = default;

  double MakeCost(Node3D& vertex) const;

 private:
  void NormalizeCost(double& cost) const;
  bool GetAreaCost(
      const std::shared_ptr<planning::FrenetObstacle>& frenet_obstacle,
      const double ego_front_s, const double ego_front_l,
      const double ego_back_s, const double ego_back_l, double& area_cost,
      const bool left_turn, const bool right_turn) const;
  void GetDirectlyBehindCost(
      const std::shared_ptr<planning::FrenetObstacle>& frenet_obstacle,
      const double ego_front_s, const double ego_front_l,
      const double ego_back_l, double& cost, const bool left_turn,
      const bool right_turn) const;
  void GetPassIntervalCost(const double delta_s, const double ego_front_l,
                           const double ego_back_l, double& cost,
                           const bool left_turn, const bool right_turn,
                           const std::pair<double, double> pass_interval) const;

 private:
  // agent list;
  const std::vector<SLBox2d> obstacles_;
  const std::shared_ptr<KDPath> ego_lane_;
  const std::shared_ptr<AABoxKDTree2d<GeometryObject>> agent_box_tree_;
  const std::shared_ptr<ReferencePath> reference_path_ptr_;
  const std::vector<std::shared_ptr<planning::FrenetObstacle>> nudge_agents_;
  const std::pair<double, double> pass_interval_;
  const bool left_turn_;
  const bool right_turn_;

  double ego_wheel_base_ = 0.0;
  double ego_length_ = 0.0;
  double ego_half_width_ = 0.0;
  double front_edge_to_rear_axle_ = 0.0;
  double rear_edge_to_rear_axle_ = 0.0;
};

}  // namespace ara_star
}  // namespace planning