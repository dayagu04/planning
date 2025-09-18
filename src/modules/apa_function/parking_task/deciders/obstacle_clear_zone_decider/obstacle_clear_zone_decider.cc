#include "obstacle_clear_zone_decider.h"

#include "log_glog.h"
#include "src/modules/apa_function/apa_param_config.h"

namespace planning {
namespace apa_planner {
const bool ObstacleClearZoneDecider::GenerateBoundingBox(
    const std::vector<Eigen::Vector2d>& pt_vec,
    const std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr) {
  box_vec_.clear();
  box_vecf_.clear();
  if (obstacle_manager_ptr == nullptr) {
    return false;
  }
  obstacle_manager_ptr_ = obstacle_manager_ptr;
  ILOG_INFO << "generate safe box";
  const float min_width = 6.6, min_length = 3.2;
  for (const Eigen::Vector2d& pt : pt_vec) {
    cdl::AABB box =
        GenerateBoundingBox(pt, 1.0, 1.0, false, false, false, false);
    if (box.long_side() > min_width && box.short_side() > min_length) {
      box_vec_.emplace_back(box);

      const Eigen::Vector2d temp_pt_right((box.min_[0] + box.max_[0]) * 0.5,
                                          box.min_[1]);

      const Eigen::Vector2d temp_pt_left((box.min_[0] + box.max_[0]) * 0.5,
                                         box.max_[1]);

      const Eigen::Vector2d temp_pt_down(box.min_[0],
                                         (box.min_[1] + box.max_[1]) * 0.5);

      const Eigen::Vector2d temp_pt_up(box.max_[0],
                                       (box.min_[1] + box.max_[1]) * 0.5);

      box = GenerateBoundingBox(temp_pt_right, 1.0, 2.0, false, false, false,
                                true);
      if (box.long_side() > min_width && box.short_side() > min_length) {
        box_vec_.emplace_back(box);
      }

      box = GenerateBoundingBox(temp_pt_left, 1.0, 2.0, false, false, true,
                                false);
      if (box.long_side() > min_width && box.short_side() > min_length) {
        box_vec_.emplace_back(box);
      }

      box = GenerateBoundingBox(temp_pt_down, 2.0, 1.0, false, true, false,
                                false);
      if (box.long_side() > min_width && box.short_side() > min_length) {
        box_vec_.emplace_back(box);
      }

      box =
          GenerateBoundingBox(temp_pt_up, 2.0, 1.0, true, false, false, false);
      if (box.long_side() > min_width && box.short_side() > min_length) {
        box_vec_.emplace_back(box);
      }
    }
  }

  cdl::AABB2f boxf;
  for (const cdl::AABB& box : box_vec_) {
    boxf.min_ << box.min_[0], box.min_[1];
    boxf.max_ << box.max_[0], box.max_[1];
    box_vecf_.emplace_back(boxf);
  }

  return true;
}

const cdl::AABB ObstacleClearZoneDecider::GenerateBoundingBox(
    const Eigen::Vector2d& pt, const float x_gain, const float y_gain,
    const bool _achieve_x_min, const bool _achieve_x_max,
    const bool _achieve_y_min, const bool _achieve_y_max) {
  cdl::AABB box;

  box.Reset(pt);
  box.expand(Eigen::Vector2d(0.05, 0.05));

  const float init_extend_x = 0.5;
  const float init_extend_y = 0.8;

  const float max_x_extend = 15.0;
  const float max_y_extend = 15.0;
  const float step = 0.2;
  const float revert_gain = 1.2;
  const float extra_buffer = 0.3;

  cdl::AABB max_box;
  max_box.min_[0] = pt.x() - max_x_extend;
  max_box.max_[0] = pt.x() + max_x_extend;
  max_box.min_[1] = pt.y() - max_y_extend;
  max_box.max_[1] = pt.y() + max_y_extend;

  const size_t max_x_number = std::ceil(max_x_extend / step);
  const size_t max_y_number = std::ceil(max_y_extend / step);
  const size_t max_numer = std::max(max_x_number, max_y_number) + 1;

  bool achieve_x_max = _achieve_x_max, achieve_x_min = _achieve_x_min;
  bool achieve_y_max = _achieve_y_max, achieve_y_min = _achieve_y_min;

  cdl::AABB temp_box;

  for (size_t i = 0; i < max_numer; ++i) {
    // extend along the positive x axis drirection
    if (!achieve_x_max) {
      box.ExtendXupper(i == 0 ? init_extend_x : step * x_gain);

      temp_box = box;
      temp_box.ExtendXupper(extra_buffer);
      if (i > 0) {
        temp_box.ExtendXlower(achieve_x_min ? 0.0 : extra_buffer);
        temp_box.ExtendYupper(achieve_y_max ? 0.0 : extra_buffer);
        temp_box.ExtendYlower(achieve_y_min ? 0.0 : extra_buffer);
      }

      if (IsCollisionForBox(temp_box) || box.max_[0] > max_box.max_[0]) {
        box.ExtendXupper(i == 0 ? -init_extend_x
                                : -step * revert_gain * x_gain);
        achieve_x_max = true;
      }
    }

    // extend along the negative x axis drirection
    if (!achieve_x_min) {
      box.ExtendXlower(i == 0 ? init_extend_x : step * x_gain);

      temp_box = box;
      temp_box.ExtendXlower(extra_buffer);
      if (i > 0) {
        temp_box.ExtendXupper(achieve_x_max ? 0.0 : extra_buffer);
        temp_box.ExtendYupper(achieve_y_max ? 0.0 : extra_buffer);
        temp_box.ExtendYlower(achieve_y_min ? 0.0 : extra_buffer);
      }

      if (IsCollisionForBox(temp_box) || box.min_[0] < max_box.min_[0]) {
        box.ExtendXlower(i == 0 ? -init_extend_x
                                : -step * revert_gain * x_gain);
        achieve_x_min = true;
      }
    }

    // extend along the positive y axis drirection
    if (!achieve_y_max) {
      box.ExtendYupper(i == 0 ? init_extend_y : step * y_gain);

      temp_box = box;
      temp_box.ExtendYupper(extra_buffer);
      if (i > 0) {
        temp_box.ExtendXupper(achieve_x_max ? 0.0 : extra_buffer);
        temp_box.ExtendXlower(achieve_x_min ? 0.0 : extra_buffer);
        temp_box.ExtendYlower(achieve_y_min ? 0.0 : extra_buffer);
      }

      if (IsCollisionForBox(temp_box) || box.max_[1] > max_box.max_[1]) {
        box.ExtendYupper(i == 0 ? -init_extend_y
                                : -step * revert_gain * y_gain);
        achieve_y_max = true;
      }
    }

    // extend along the negative y axis drirection
    if (!achieve_y_min) {
      box.ExtendYlower(i == 0 ? init_extend_y : step * y_gain);

      temp_box = box;
      temp_box.ExtendYlower(extra_buffer);
      if (i > 0) {
        temp_box.ExtendXupper(achieve_x_max ? 0.0 : extra_buffer);
        temp_box.ExtendXlower(achieve_x_min ? 0.0 : extra_buffer);
        temp_box.ExtendYupper(achieve_y_max ? 0.0 : extra_buffer);
      }

      if (IsCollisionForBox(temp_box) || box.min_[1] < max_box.min_[1]) {
        temp_box.DebugString();
        box.ExtendYlower(i == 0 ? -init_extend_y
                                : -step * revert_gain * y_gain);
        achieve_y_min = true;
      }
    }

    if (achieve_x_max && achieve_x_min && achieve_y_max && achieve_y_min) {
      break;
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

const bool ObstacleClearZoneDecider::IsInClearZone(const cdl::AABB2f& box) {
  for (const cdl::AABB2f& temp_box : box_vecf_) {
    if (temp_box.IsContain(box)) {
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