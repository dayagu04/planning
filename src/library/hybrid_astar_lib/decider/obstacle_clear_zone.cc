
#include "obstacle_clear_zone.h"

#include <algorithm>

#include "./../convex_collision_detection/aabb2d.h"
#include "collision_detect_types.h"
#include "log_glog.h"

namespace planning {

bool ObstacleClearZone::IsCollisionForBox(const cdl::AABB& box,
                                          const ParkObstacleList* obstacles) {
  cdl::AABB obs_box;
  for (size_t i = 0; i < obstacles->virtual_obs.size(); i++) {
    if (box.contain(obstacles->virtual_obs[i])) {
      return true;
    }
  }

  for (size_t i = 0; i < obstacles->point_cloud_list.size(); i++) {
    if (!box.overlap(obstacles->point_cloud_list[i].box)) {
      continue;
    }

    for (size_t j = 0; j < obstacles->point_cloud_list[i].points.size(); j++) {
      if (box.contain(obstacles->point_cloud_list[i].points[j])) {
        return true;
      }
    }
  }

  return false;
}

bool ObstacleClearZone::GenerateBoundingBox(const Pose2D& start,
                                            const ParkObstacleList* obstacles) {
  box_.Reset(cdl::Vector2r(start.x, start.y));

  box_.ExtendX(0.5);
  box_.ExtendY(0.8);

  double max_x_extend = 10;
  double max_y_extend = 15;
  double step = 0.2;

  cdl::AABB max_box;
  max_box.min_[0] = start.x - max_x_extend;
  max_box.max_[0] = start.x + max_x_extend;
  max_box.min_[1] = start.y - max_y_extend;
  max_box.max_[1] = start.y + max_y_extend;

  max_box.DebugString();

  size_t max_x_number = std::ceil(max_x_extend / step);
  size_t max_y_number = std::ceil(max_y_extend / step);
  size_t max_numer = std::max(max_x_number, max_y_number);

  bool achieve_x_max = false;
  bool achieve_x_min = false;
  bool achieve_y_max = false;
  bool achieve_y_min = false;

  cdl::AABB tmp = box_;

  for (size_t i = 0; i <= max_numer; i++) {
    if (!achieve_y_max) {
      tmp.ExtendYupper(step);

      if (IsCollisionForBox(tmp, obstacles)) {
        tmp.ExtendYupper(-step);
        achieve_y_max = true;
      }

      if (tmp.max_[1] > max_box.max_[1]) {
        tmp.ExtendYupper(-step);
        achieve_y_max = true;
      }
    }

    // ILOG_INFO << "y max " << tmp.max_[1];

    if (!achieve_y_min) {
      tmp.ExtendYlower(step);

      if (IsCollisionForBox(tmp, obstacles)) {
        tmp.ExtendYlower(-step);
        achieve_y_min = true;
      }

      if (tmp.min_[1] < max_box.min_[1]) {
        tmp.ExtendYlower(-step);
        achieve_y_min = true;
      }
    }

    // ILOG_INFO << "y min " << tmp.min_[1];

    if (!achieve_x_max) {
      tmp.ExtendXupper(step);

      if (IsCollisionForBox(tmp, obstacles)) {
        tmp.ExtendXupper(-step);
        achieve_x_max = true;
      }

      if (tmp.max_[0] > max_box.max_[0]) {
        tmp.ExtendXupper(-step);
        achieve_x_max = true;
      }
    }

    // ILOG_INFO << "x max " << tmp.max_[0];

    if (!achieve_x_min) {
      tmp.ExtendXlower(step);

      if (IsCollisionForBox(tmp, obstacles)) {
        tmp.ExtendXlower(-step);
        achieve_x_min = true;
      }

      if (tmp.min_[0] < max_box.min_[0]) {
        tmp.ExtendXlower(-step);
        achieve_x_min = true;
      }
    }

    // ILOG_INFO << "x min " << tmp.min_[0];
  }

  step = 0.1;
  tmp.ExtendYupper(step);

  if (IsCollisionForBox(tmp, obstacles)) {
    tmp.ExtendYupper(-step);
  }

  if (tmp.max_[1] > max_box.max_[1]) {
    tmp.ExtendYupper(-step);
  }

  // ILOG_INFO << "y max " << tmp.max_[1];

  tmp.ExtendYlower(step);

  if (IsCollisionForBox(tmp, obstacles)) {
    tmp.ExtendYlower(-step);
  }

  if (tmp.min_[1] < max_box.min_[1]) {
    tmp.ExtendYlower(-step);
  }

  // ILOG_INFO << "y min " << tmp.min_[1];

  tmp.ExtendXupper(step);

  if (IsCollisionForBox(tmp, obstacles)) {
    tmp.ExtendXupper(-step);
  }

  if (tmp.max_[0] > max_box.max_[0]) {
    tmp.ExtendXupper(-step);
  }

  // ILOG_INFO << "x max " << tmp.max_[0];

  tmp.ExtendXlower(step);

  if (IsCollisionForBox(tmp, obstacles)) {
    tmp.ExtendXlower(-step);
  }

  if (tmp.min_[0] < max_box.min_[0]) {
    tmp.ExtendXlower(-step);
  }

  box_ = tmp;

  box_.DebugString();

  return true;
}

const bool ObstacleClearZone::IsContain(const cdl::AABB& box) const {
  return box_.contain(box);
}

}  // namespace planning