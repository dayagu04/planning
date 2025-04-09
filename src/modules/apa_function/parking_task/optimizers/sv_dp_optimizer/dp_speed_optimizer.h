#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "dp_speed_common.h"
#include "dp_speed_cost.h"
#include "geometry_math.h"
#include "parking_task.h"
#include "pose2d.h"
#include "speed/apa_speed_decision.h"
#include "src/modules/apa_function/parking_task/deciders/speed_limit_decider/speed_limit_profile.h"
#include "sv_graph_node.h"
#include "dp_speed_config.h"
#include "src/modules/common/speed/speed_data.h"
#include "dp_cost_generator.h"
#include "dp_error_code.h"
#include "optimizer_common.h"

namespace planning {

// solution for speed generation:
// 1. search in s-v graph;
// 2. search in s-t graph;
// 3. trapezoid curve;
// 4. optimization based;
class DpSpeedOptimizer : public ParkingTask {
 public:
  DpSpeedOptimizer();

  void Excute(const std::vector<pnc::geometry_lib::PathPoint>& path,
              const Pose2D& ego_pose, const double ego_v, const double ego_acc,
              const SpeedDecisions* speed_decisions,
              const SpeedLimitProfile* speed_limit_profile);

  bool Init() override;

  const SpeedData& SpeedProfile() const { return speed_data_; }

  const SVPoint GetStartSpeedPoint() const;

  const SpeedOptimizerState GetDPState() const { return solver_state_; }

 private:
  // dp搜索
  void Search();

  void GetNextNodeSpeedRange(const double node_v, SpeedBoundary* bound);

  void GetNodeSpeedRangeIndex(const SpeedBoundary* v_bound,
                              SpeedBoundaryIndex* v_index);

  void GetNodeLayerSpeedRange(const double s, SpeedBoundary* bound);

  SVGraphNode* GetNodeFromPool(const SVGridIndex& index);

  void GridIndexTransform(const SVGridIndex* index, const DpSpeedConfig* config,
                          SVPoint* point);

  const double GetSpeedByIndex(const int32_t v_index) {
    return v_index * config_.unit_v;
  }

  void GridIndexTransform(const SVPoint* point, const DpSpeedConfig* config,
                          SVGridIndex* index);

  const double GetSpeedLimitByIndex(const int32_t s_index);

  void UpdateStartNode();

  void UpdateSearchBoundary();

  void GenerateNextNode(const SVGraphNode* parent_node, const int32_t v_index,
                        SVGraphNode* child_node);

  void UpdateCostTable();

  void UpdateSpeedLimitLookUp();

  void UpdateEndNode();

  // retrive
  void RetrieveSpeedProfile(SpeedData* const speed_data);

  void CalculateTotalCost();

  void CalculateNodeCost(const SVGraphNode* parent, const int32_t s_index,
                         const int32_t v_index, SVGraphNode* child);

  // node s is over path length, should stop;
  const bool IsNodeShouldStop(SVGraphNode& node);

  // debug
  void DebugSpeedData();

  void DebugSpeedLimitLookUp() const;

  void RecordDebugInfo();

 private:
  const SpeedDecisions *speed_decisions_;
  const SpeedLimitProfile *speed_limit_profile_;
  std::vector<pnc::geometry_lib::PathPoint> path_;

  double ego_v_;
  double ego_acc_;
  double max_search_v_;

  DpSpeedConfig config_;

  double total_s_ = 0.0;
  int32_t dimension_s_ = 0;

  int32_t dimension_v_ = 0;

  // cost_table_[s][v]
  // todo: use c array
  std::vector<std::vector<SVGraphNode>> cost_table_;

  // s index对应的speed limit. 可以快速查找，已知index, 获取speed limit
  std::vector<double> speed_limit_by_index_;

  DPCostGenerator cost_generator_;

  DpSpeedErrorCode error_code_;

  SVGraphNode* start_node_;
  SVGraphNode* end_node_;

  SpeedData speed_data_;

  SpeedOptimizerState solver_state_;
};

}  // namespace planning
