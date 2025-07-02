#include "obstacle_clear_zone_decider.h"

#include "log_glog.h"
#include "src/modules/apa_function/apa_param_config.h"

namespace planning {
namespace apa_planner {
const bool ObstacleClearZoneDecider::GenerateBoundingBox(
    const std::vector<Eigen::Vector2d>& pt_vec,
    const std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr) {
  box_vec_.clear();
  if (obstacle_manager_ptr == nullptr) {
    return false;
  }
  obstacle_manager_ptr_ = obstacle_manager_ptr;
  for (const Eigen::Vector2d& pt : pt_vec) {
    box_vec_.emplace_back(GenerateBoundingBox(pt));
  }
  return true;
}

const cdl::AABB ObstacleClearZoneDecider::GenerateBoundingBox(
    const Eigen::Vector2d& pt) {
  cdl::AABB box;

  box.Reset(pt);

  box.ExtendX(0.5);
  box.ExtendY(0.8);

  const float max_x_extend = 10.0;
  const float max_y_extend = 15.0;
  const float step = 0.2;
  const float revert_gain = 2.5;

  cdl::AABB max_box;
  max_box.min_[0] = pt.x() - max_x_extend;
  max_box.max_[0] = pt.x() + max_x_extend;
  max_box.min_[1] = pt.y() - max_y_extend;
  max_box.max_[1] = pt.y() + max_y_extend;

  const size_t max_x_number = std::ceil(max_x_extend / step);
  const size_t max_y_number = std::ceil(max_y_extend / step);
  const size_t max_numer = std::max(max_x_number, max_y_number) + 1;

  bool achieve_x_max = false, achieve_x_min = false;
  bool achieve_y_max = false, achieve_y_min = false;

  for (size_t i = 0; i < max_numer; ++i) {
    // extend along the positive x axis drirection
    if (!achieve_x_max) {
      box.ExtendXupper(step);
      if (IsCollisionForBox(box) || box.max_[0] > max_box.max_[0]) {
        box.ExtendXupper(-step * revert_gain);
        achieve_x_max = true;
      }
    }

    // extend along the negative x axis drirection
    if (!achieve_x_min) {
      box.ExtendXlower(step);
      if (IsCollisionForBox(box) || box.min_[0] < max_box.min_[0]) {
        box.ExtendXlower(-step * revert_gain);
        achieve_x_min = true;
      }
    }

    // extend along the positive y axis drirection
    if (!achieve_y_max) {
      box.ExtendYupper(step);
      if (IsCollisionForBox(box) || box.max_[1] > max_box.max_[1]) {
        box.ExtendYupper(-step * revert_gain);
        achieve_y_max = true;
      }
    }

    // extend along the negative y axis drirection
    if (!achieve_y_min) {
      box.ExtendYlower(step);
      if (IsCollisionForBox(box) || box.min_[1] < max_box.min_[1]) {
        box.ExtendYlower(-step * revert_gain);
        achieve_y_min = true;
      }
    }
  }

  box.DebugString();

  return box;
}

const bool ObstacleClearZoneDecider::IsInClearZone(const Eigen::Vector2d& pt) {
  for (const cdl::AABB& box : box_vec_) {
    if (box.contain(pt)) {
      return true;
    }
  }
  return false;
}

const bool ObstacleClearZoneDecider::IsInClearZone(const cdl::AABB& box) {
  for (const cdl::AABB& temp_box : box_vec_) {
    if (temp_box.contain(box)) {
      return true;
    }
  }
  return false;
}

const bool ObstacleClearZoneDecider::IsInClearZone(
    const geometry_lib::RectangleBound& bound) {
  cdl::AABB box;
  box.min_[0] = bound.min_x;
  box.min_[1] = bound.min_y;
  box.max_[0] = bound.max_x;
  box.max_[1] = bound.max_y;
  return IsInClearZone(box);
}

const bool ObstacleClearZoneDecider::IsCollisionForBox(const cdl::AABB& box) {
  const std::unordered_map<size_t, ApaObstacle>& obs_map =
      obstacle_manager_ptr_->GetObstacles();

  // now, the obs is pt cloud
  for (const auto& obs : obs_map) {
    for (const Eigen::Vector2d& pt : obs.second.GetPtClout2dLocal()) {
      if (box.contain(pt)) {
        return true;
      }
    }
  }
  return false;
}
}  // namespace apa_planner
}  // namespace planning