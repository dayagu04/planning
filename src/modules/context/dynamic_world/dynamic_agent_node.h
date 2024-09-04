#pragma once

#include "agent/agent.h"
#include "config/vehicle_param.h"
#include "reference_path.h"
#include "trajectory/center_line_point.h"
#include "trajectory/trajectory_point.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"

namespace planning {
namespace planning_data {

constexpr int32_t kInvalidId = -1;

class DynamicAgentNode {
 public:
  DynamicAgentNode() = default;

  DynamicAgentNode(const agent::Agent* agent, const VirtualLane& lane,
                   const ReferencePath& ref_line,
                   const trajectory::TrajectoryPoint ego_state);

  ~DynamicAgentNode() = default;

  std::pair<double, double> CalculateLRange(const double agent_match_s,
                                            const double agent_match_l,
                                            const double agent_match_theta,
                                            const double agent_theta,
                                            const double agent_length,
                                            const double agent_width);

  bool IsAgentPartiallyWithinLane(
      const std::pair<double, double>& l_min_max,
      const double agent_match_left_lane_width,
      const double agent_match_right_lane_width, const double ego_vehicle_width,
      const double in_lane_distance_to_boundary_threshold);

  bool Init();

  bool InitEgoNode(
      const int32_t& lane_id, const double ego_s,
      const trajectory::TrajectoryPoint& time_aligned_vehicle_state);

  bool GetClosestCenterLinePoint(
      const double x, const double y,
      std::vector<trajectory::CenterLinePoint>& CenterLineVec,
      trajectory::CenterLinePoint* const match_center_point,
      double* const l) const;

  std::vector<trajectory::CenterLinePoint>::const_iterator QueryLowerBound(
      const std::vector<trajectory::CenterLinePoint>& center_line_points,
      const double path_s) const;

  bool is_valid() const;
  const int64_t node_id() const;
  const int32_t node_lane_id() const;
  const int32_t node_agent_id() const;

  double node_x() const;
  double node_y() const;
  double node_theta() const;
  double node_speed() const;
  double node_accel() const;
  double node_s() const;
  double node_t() const;
  double node_length() const;
  double node_width() const;
  double node_to_ego_distance() const;
  double node_back_edge_to_ego_front_edge_distance() const;
  double node_front_edge_to_ego_back_edge_distance() const;
  const std::vector<trajectory::Trajectory>& node_trajectories() const;
  const iflyauto::ObjectType type() const;

  bool is_agent_within_lane() const;
  bool is_agent_most_within_lane() const;
  bool is_agent_against_traffic() const;
  bool is_agent_extremely_slow() const;
  bool is_agent_has_overlap_with_ego() const;
  bool is_VRU_type() const;
  bool is_cone_type() const;
  bool is_vehicle_type() const;

  const int64_t front_node_id() const;
  void set_front_node_id(const int64_t& forward_node_id);
  const int64_t rear_node_id() const;
  void set_rear_node_id(const int64_t& backward_node_id);

 private:
  const agent::Agent* agent_;
  const VehicleParam vehicle_param_;
  std::shared_ptr<KDPath> coord_;
  bool is_valid_ = false;
  int64_t node_id_ = kInvalidId;
  int32_t node_lane_id_ = kInvalidId;
  int32_t node_agent_id_ = kInvalidId;

  double node_s_ = 0.0;
  double node_t_ = 0.0;
  double node_length_ = 0.0;
  double node_to_ego_distance_ = 0.0;
  double node_front_edge_to_ego_back_edge_distance_ = 0.0;
  double node_back_edge_to_ego_front_edge_distance_ = 0.0;

  bool is_agent_within_lane_ = false;
  bool is_agent_most_within_lane_ = false;
  bool is_agent_against_traffic_ = false;
  bool is_agent_extremely_slow_ = false;
  bool is_agent_has_overlap_with_ego_ = false;

  int64_t front_node_id_ = kInvalidId;
  int64_t rear_node_id_ = kInvalidId;
};

}  // namespace planning_data
}  // namespace planning