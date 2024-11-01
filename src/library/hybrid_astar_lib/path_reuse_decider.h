#pragma once

#include "astar_decider.h"
#include "future_path_decider.h"
#include "hybrid_astar_common.h"

namespace planning {

// for now, only for single shot parking scenario, can reuse path.
class PathReuseDecider : public AstarDecider {
 public:
  PathReuseDecider() = default;

  void Process(const Pose2D& start, const Pose2D& end) override;

  void Process(HybridAStarResult* path, const HybridAStarResult* history_path,
               const Pose2D& current_slot_pose,
               const HistoryPathDriveInfo* next_shot_info);

  const bool IsReusePath();

 private:
  bool reuse_path_;
};

}  // namespace planning