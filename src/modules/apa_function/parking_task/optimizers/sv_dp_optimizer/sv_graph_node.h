#pragma once

#include <limits>
#include <vector>

#include "dp_speed_common.h"
#include "dp_speed_cost.h"

namespace planning {
namespace apa_planner {
// sv-searching point
class SVGraphNode {
 public:
  SVGraphNode() = default;

  int32_t IndexS() const;
  int32_t indexV() const;

  const SVPoint& GetSVPoint() const;
  const SVGraphNode* PrePoint() const;

  double TotalCost() const;

  void Init(const int32_t index_s, const int32_t index_v,
            const SVPoint& sv_point, const double speed_limit);

  // total cost
  void SetTotalCost(const double total_cost);

  void SetPrePoint(const SVGraphNode* pre_point);

  void SetAcc(const double acc) { sv_point_.acc = acc; }

  void SetTime(const double time) { sv_point_.t = time; }

  void SetV(const double v) { sv_point_.v = v; }

  void SetCost(const DpSpeedCost& cost) {
    cost_ = cost;
    return;
  }

  void SetZeroCost();

  const DpSpeedCost& ConstCost() { return cost_; }

  DpSpeedCost* MutableCost() { return &cost_; }

  const SVGridIndex& GetSVIndex() const { return grid_index_; }

  const bool IsZeroSpeed();

  // interpolation: include current node, not include successor node.
  void Interpolate(const double s_step, const SVGraphNode* successor,
                   std::vector<SVPoint>& speed_curve) const;

  void DebugString() const;

  const double GetSpeedLimit() const { return speed_limit_; }

  void EvaluateAcc(const SVGraphNode* parent_node, const double dist);

  // Node acc/speed should kown.
  void EvaluateTime(const SVGraphNode* parent_node, const double dist);

  void EvaluateJerk(const SVGraphNode* parent_node);

 private:
  // dp图的点
  SVPoint sv_point_;

  // cost_table_的index
  SVGridIndex grid_index_;

  const SVGraphNode* pre_point_ = nullptr;

  double speed_limit_ = 0.0;

  DpSpeedCost cost_;
};
}  // namespace apa_planner
}  // namespace planning
