#pragma once
#include <memory>
#include <unordered_map>
#include <vector>

#include "dynamic_world/dynamic_agent_node.h"
#include "math/aabox2d.h"
#include "sample_poly_const.h"
// #include "speed/st_boundary.h"
#include "st_graph/st_point.h"
namespace planning {
struct AgentInfo {
  int32_t id = kNoAgentId;
  double center_s;
  double half_length;
  double v;
};

struct LeadingAgentInfo : public AgentInfo {
  std::vector<std::pair<double, double>>
      prediction_path;  // first is traveled_distance second is speed
  bool prediction_path_valid = false;
};

class STSampleSpaceBase {
 public:
  STSampleSpaceBase() = default;
  STSampleSpaceBase(const double front_edge_to_rear_axle,
                    const double rear_edge_to_rear_axle);

  STSampleSpaceBase(
      const std::vector<const planning_data::DynamicAgentNode*>& target_lane_nodes,
      const double init_s, const double front_edge_to_rear_axle,
      const double rear_edge_to_rear_axle);

  void LinearExtendAgentStBoundary(const planning_data::DynamicAgentNode* agent_node);
  void ConstructStPointsTable();
  bool GetBorderByAvailable(double s, double t, planning::speed::STPoint* const lower_st_point,
                            planning::speed::STPoint* const upper_st_point);
  void Init(const std::vector<const planning_data::DynamicAgentNode*>& target_lane_nodes,
            const double init_s);
  void GetAvailableGap(const int index, double s);

  void Clear();
  void SetInitS(const double s0);

  bool IsWithinRange(double s, double start, double end) {
    return s > start && s < end;
  };
  bool IsInInterval(double s, double front_edge_offset, double rear_edge_offset,
                    const planning::speed::STPoint& interval_start,
                    const planning::speed::STPoint& interval_end) {
    double front_s = s + front_edge_offset;
    double rear_s = s - rear_edge_offset;
    return (front_s > interval_start.s() && front_s < interval_end.s()) ||
           (rear_s > interval_start.s() && rear_s < interval_end.s());
  }
  const double init_s() const;
  void SetInitVel(const double v0);

  std::unordered_map<int64_t, std::unique_ptr<AgentInfo>>&
  mutable_agent_id_veh_info() {
    return agent_id_veh_info_;
  }

  const std::unordered_map<int64_t, std::unique_ptr<AgentInfo>>&
  agent_id_veh_info() const {
    return agent_id_veh_info_;
  }

  std::vector<std::vector<std::pair<planning::speed::STPoint, planning::speed::STPoint>>>&
  mutable_st_points_table() {
    return st_points_table_;
  }

  const std::vector<std::vector<std::pair<planning::speed::STPoint, planning::speed::STPoint>>>& st_points_table()
      const {
    return st_points_table_;
  }

  std::vector<planning::speed::STPoint>& mutable_sample_points() { return sample_points_; }

  const std::vector<planning::speed::STPoint>& sample_points() const { return sample_points_; }

  std::vector<std::pair<planning::speed::STPoint, planning::speed::STPoint>>& get_gap_array() {
    return gap_array_;
  }

 private:
  std::vector<std::vector<std::pair<planning::speed::STPoint, planning::speed::STPoint>>> st_points_table_;
  std::vector<planning::speed::STPoint> sample_points_;  //(s, v)
  std::vector<std::pair<planning::speed::STPoint, planning::speed::STPoint>> gap_array_;

  std::vector<std::vector<std::pair<planning::speed::STPoint, planning::speed::STPoint>>> agents_st_point_paris_;
  std::unordered_map<int64_t, std::unique_ptr<AgentInfo>> agent_id_veh_info_;
  double init_s_{0.0};

  double front_edge_to_rear_axle_ = 3.73;
  double rear_edge_to_rear_axle_ = 1.085;
};
}  // namespace planning
