
#include "path_reuse_decider.h"

#include "pose2d.h"
#include "single_shot_parking_decider.h"
#include "transform2d.h"

namespace planning {

void PathReuseDecider::Process(HybridAStarResult* path,
                               const HybridAStarResult* history_path,
                               const Pose2D& current_slot_pose,
                               const NextShotPathInfo* next_shot_info) {
  reuse_path_ = false;
  if (next_shot_info->shot_number_ != PathShotNumber::single_shot_path) {
    return;
  }

  if (history_path == nullptr) {
    return;
  }

  Transform2d history_tf;
  Transform2d cur_tf;
  history_tf.SetBasePose(history_path->base_pose);
  cur_tf.SetBasePose(current_slot_pose);

  if (next_shot_info->start_point_id_ >= history_path->x.size()) {
    return;
  }

  if (next_shot_info->dist_ < 0.5) {
    return;
  }

  // change to global
  Pose2D global;
  Pose2D local;
  path->Clear();

  size_t i;
  for (i = next_shot_info->start_point_id_; i < history_path->x.size(); i++) {
    local.x = history_path->x[i];
    local.y = history_path->y[i];
    local.theta = history_path->phi[i];

    history_tf.ULFLocalPoseToGlobal(&global, local);

    cur_tf.GlobalPoseToULFLocal(&local, global);

    path->x.emplace_back(local.x);
    path->y.emplace_back(local.y);
    path->phi.emplace_back(local.theta);
    path->gear.emplace_back(history_path->gear[i]);
    path->type.emplace_back(history_path->type[i]);
    path->accumulated_s.emplace_back(history_path->accumulated_s[i] -
                                     next_shot_info->start_point_s_);
  }

  path->base_pose = current_slot_pose;
  reuse_path_ = true;

  return;
}

const bool PathReuseDecider::IsReusePath() { return reuse_path_; }

void PathReuseDecider::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  return;
}

}  // namespace planning