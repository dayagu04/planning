#pragma once

#include <cstddef>
#include <string>
#include "astar_decider.h"
#include "hybrid_astar_common.h"
#include "node3d.h"
#include "pose2d.h"

namespace planning {

enum class PathGearSwitchNumber {
  NONE,
  ONCE,
  TWICE,
  MANY_TIMES,
};

// 推理下一次换档信息，行驶距离信息等
struct InferenceNextPathInfo {
  PathGearSwitchNumber gear_switch_number_;

  double start_point_s_;
  double end_point_s_;
  double dist_;
  size_t start_point_id_;
  size_t end_point_id_;

  AstarPathGear gear_;
};

// decider: check path will go to parking slot by single shot
class DriveDistanceDecider : public AstarDecider {
 public:
  DriveDistanceDecider() = default;

  void Process(HybridAStarResult *history_path,
               const PlanningReason plan_reason);

  void Process(const Pose2D &start, const Pose2D &end);

  const AstarPathGear GetNextPathGear();

  const double GetNextPathLength();

  const size_t GetNextPathStartPointId();

  const bool IsNextPathNoGearSwitch();

 protected:
  // use this info as an heuristic info
  InferenceNextPathInfo next_path_info_;

 private:
  std::string PathGearSwitchNumberString(
      const PathGearSwitchNumber &gear_number);
};

}  // namespace planning