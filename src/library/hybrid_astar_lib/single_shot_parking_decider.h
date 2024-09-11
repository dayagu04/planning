#pragma once

#include <cstddef>
#include <string>
#include "astar_decider.h"
#include "hybrid_astar_common.h"
#include "node3d.h"
#include "pose2d.h"

namespace planning {

enum class PathShotNumber {
  none,
  single_shot_path,
  multi_shot_path,
};

struct NextShotPathInfo {
  PathShotNumber shot_number_;

  double start_point_s_;
  double end_point_s_;
  double dist_;
  size_t start_point_id_;
  size_t end_point_id_;

  AstarPathGear gear_;
};

// decider: check path will go to parking slot by single shot
class SingleShotParkingDecider : public AstarDecider {
 public:
  SingleShotParkingDecider() = default;

  void Process(HybridAStarResult *history_path);

  void Process(const Pose2D &start, const Pose2D &end);

  const AstarPathGear GetNextShotGear();

  const double GetNextShotPathLength();

  const size_t GetNextShotStartPointId();

  const bool IsSingleShotPath();

 protected:
  // use this info as an heuristic info
  NextShotPathInfo next_shot_path_info_;

 private:
  std::string PathShotNumberString(const PathShotNumber &shot_number);
};

}  // namespace planning