#pragma once

#include "st_graph.h"

namespace planning {
namespace speed {

// The helper of st_graph to support read-only method.
class StGraphHelper {//返回一些st 的可行边界 可通行走廊等等
 public:
  explicit StGraphHelper(const STGraph& st_graph);
  ~StGraphHelper() = default;

  const STPoint GetSoftPassCorridorUpperBound(const double t) const;

  const std::pair<double, double> GetPathRange() const;

  const std::pair<double, double> GetTimeRange() const;

  // void AddStGraphDataToProto(
  //     proto::PlanningDebugMessage* const planning_debug_msg) const;

  bool GetBorderByStPoint(double s, double t, STPoint* const lower_st_point,
                          STPoint* const upper_st_point) const;

  bool GetNeighborAgentStBoundaries(
      const int32_t agent_id, std::vector<int64_t>* const st_boundaries) const;

  bool GetNeighborAgentStBoundary(const int64_t boundary_id,
                                  STBoundary* const st_boundary) const;

  const std::vector<std::pair<STPoint, STPoint>>& st_pass_corridor() const;

  const STPoint GetPassCorridorUpperBound(const double t) const;

  const STPoint GetPassCorridorLowerBound(const double t) const;

  bool GetFirstNeighborUpperBound(STPoint* const upper_point) const;

  bool GetFirstNeighborLowerBound(STPoint* const lower_point) const;

  const bool GetStBoundary(const int64_t boundary_id,
                           STBoundary* const st_boundary) const;

  const bool GetAgentStBoundaries(
      const int32_t agent_id, std::vector<int64_t>* const st_boundaries) const;

  const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
  GetAllStBoundaries() const;

  const std::unordered_map<int32_t, std::vector<int64_t>>&
  GetAgentIdSTBoundariesMap() const;

  const std::unordered_map<int32_t, std::vector<int64_t>>&
  GetNeighborAgentIdSTBoundariesMap() const {
    return st_graph_.neighbor_agent_id_st_boundaries_map();
  }

  void DetermineIfConeBucketCIPV(
      const std::vector<std::shared_ptr<agent::Agent>>& agents);

  const bool GetExpandStBoundary(const int64_t boundary_id,
                                 STBoundary* const st_boundary) const;

  const bool GetExpandAgentStBoundaries(
      const int32_t agent_id, std::vector<int64_t>* const st_boundaries) const;

  const std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
  close_pass_boundary_id_st_boundaries_map() const;

  const std::unordered_map<int32_t, std::vector<int64_t>>&
  close_pass_agent_id_st_boundaries_map() const;

 private:
  bool IsTimeInRange(double t) const;
  void MakeSpeedLimitedConeBucketStBoundary(
      const agent::Agent& agent,
      std::unordered_map<int64_t, std::unique_ptr<STBoundary>>&
          boundary_id_st_boundaries_map);

 private:
  const STGraph& st_graph_;
};

}  // namespace speed
}  // namespace planning