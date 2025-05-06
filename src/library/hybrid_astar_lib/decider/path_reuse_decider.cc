
#include "path_reuse_decider.h"

#include "pose2d.h"
#include "transform2d.h"

namespace planning {

void PathReuseDecider::Process(HybridAStarResult* path,
                               const HybridAStarResult* history_path,
                               const Pose2D& current_slot_pose) {
  reuse_path_ = false;
  return;
}

const bool PathReuseDecider::IsReusePath() { return reuse_path_; }

void PathReuseDecider::Process(const Pose2D& start, const Pose2D& end) {
  AstarDecider::Process(start, end);

  return;
}

}  // namespace planning