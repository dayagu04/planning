#pragma once
#include <memory>
#include <unordered_map>
#include <vector>

#include "math/aabox2d.h"
#include "sample_poly_const.h"
// #include "speed/st_boundary.h"
#include "st_graph/st_point.h"
using planning::speed::STPoint;

namespace planning {
struct AgentInfo {
  int32_t id = kNoAgentId;
  double center_s;
  double half_length;
  double v;
};

class STSampleSpaceBase {
 public:
  STSampleSpaceBase() = default;
  STSampleSpaceBase(const double front_edge_to_rear_axle,
                    const double rear_edge_to_rear_axle);

  STSampleSpaceBase(const std::vector<AgentInfo>& lane_change_veh_info,
                    const double init_s, const double front_edge_to_rear_axle,
                    const double rear_edge_to_rear_axle);

  void LinearExtendAgentStBoundary(const AgentInfo& agent);
  void ConstructStPointsTable();
  bool GetBorderByAvailable(double s, double t, STPoint* const lower_st_point,
                            STPoint* const upper_st_point);
  void Init(const std::vector<AgentInfo>& lane_change_veh_info,
            const double init_s);
  void Clear();
  void SetInitS(const double s0);

  bool IsWithinRange(double s, double start, double end) {
    return s > start && s < end;
  };
  bool IsInInterval(double s, double front_edge_offset, double rear_edge_offset,
                    const STPoint& interval_start,
                    const STPoint& interval_end) {
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

  std::vector<std::vector<std::pair<STPoint, STPoint>>>&
  mutable_st_points_table() {
    return st_points_table_;
  }

  const std::vector<std::vector<std::pair<STPoint, STPoint>>>& st_points_table()
      const {
    return st_points_table_;
  }

  std::vector<STPoint>& mutable_sample_points() { return sample_points_; }

  const std::vector<STPoint>& sample_points() const { return sample_points_; }

 private:
  std::vector<std::vector<std::pair<STPoint, STPoint>>> st_points_table_;
  std::vector<STPoint> sample_points_;  //(s, v)

  std::vector<std::vector<std::pair<STPoint, STPoint>>> agents_st_point_paris_;
  std::unordered_map<int64_t, std::unique_ptr<AgentInfo>> agent_id_veh_info_;
  double init_s_{0.0};

  double front_edge_to_rear_axle_ = 3.73;
  double rear_edge_to_rear_axle_ = 1.085;
};
}  // namespace planning
