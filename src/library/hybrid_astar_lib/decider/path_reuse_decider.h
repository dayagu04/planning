#pragma once

#include "astar_decider.h"
#include "future_path_decider.h"
#include "hybrid_astar_common.h"

namespace planning {

// todo: how to reuse path.
class PathReuseDecider : public AstarDecider {
 public:
  PathReuseDecider() = default;

  void Process(const Pose2f& start, const Pose2f& end) override;

  void Process(HybridAStarResult* path, const HybridAStarResult* history_path,
               const Pose2f& current_slot_pose);

  const bool IsReusePath();

 private:
  bool reuse_path_;
};

}  // namespace planning