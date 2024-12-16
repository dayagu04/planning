#include "collision_checker.h"

#include <iostream>
#include <memory>

#include "ad_common/math/math_utils.h"

namespace planning {
namespace planning_math {
CollisionChecker::CollisionChecker() {
  deviation_length_ = 0.0;
  trajectory_.clear();
  check_type_ = CheckType::TRAJECTORY;
  ego_model_ = std::make_shared<EgoModelManager>();
}

bool CollisionChecker::set_params(const double deviation_length) {
  deviation_length_ = deviation_length;
  ego_model_->set_params(deviation_length_);

  return true;
}

bool CollisionChecker::set_trajectory(
    const std::vector<PathPoint> &trajectory) {
  check_type_ = CheckType::TRAJECTORY;
  trajectory_ = trajectory;
  return true;
}

bool CollisionChecker::set_trajectory(const std::vector<Pose2D> &trajectory) {
  check_type_ = CheckType::TRAJECTORY;
  trajectory_.clear();
  for (Pose2D point_tmp : trajectory) {
    PathPoint point;
    point.set_x(point_tmp.x);
    point.set_y(point_tmp.y);
    point.set_theta(point_tmp.theta);
    trajectory_.push_back(point);
  }
  return true;
}

bool CollisionChecker::set_point(const PathPoint &point) {
  check_type_ = CheckType::POINT;
  point_ = point;
  return true;
}

bool CollisionChecker::set_point(const Pose2D &point) {
  check_type_ = CheckType::POINT;
  point_.set_x(point.x);
  point_.set_y(point.y);
  point_.set_theta(point.theta);
  return true;
}

CollisionCheckStatus CollisionChecker::collision_check(
    const planning_math::Box2d &box, const double collision_threshold,
    CollisionCheckStatus::CollisionType collision_type) {
  using namespace ad_common::math;
  planning_math::Box2d ego_box;
  CollisionCheckStatus result;
  double s = 0;
  double min_dis = 100;
  bool init = false;
  if (is_trajectory()) {
    std::vector<PathPoint>::iterator iter = trajectory_.begin();
    for (const auto &point : trajectory_) {
      s += fast_hypot(iter->x() - point.x(), iter->y() - point.y());

      ego_model_->set_model_center(point);
      ego_model_->set_origin_model();
      double temp_dis = ego_model_->get_ego_model_polygon().DistanceTo(box);
      if (temp_dis < min_dis) {
        result.s = s;
        min_dis = temp_dis;
        if (min_dis < collision_threshold) {
          result.is_collision = true;
          result.min_distance = min_dis;
          result.ego_poit.x = point.x();
          result.ego_poit.y = point.y();
          result.ego_poit.theta = point.theta();
          result.collision_type = collision_type;
          result.collision_object_position.x = box.center().x();
          result.collision_object_position.y = box.center().y();
          result.collision_object_position.theta = box.heading();
          return result;
        }
      }
      if (init) {
        iter++;
      } else {
        init = true;
      }
    }
  } else {
    ego_model_->set_model_center(point_);
    ego_model_->set_origin_model();
    double temp_dis = ego_model_->get_ego_model_polygon().DistanceTo(box);
    if (temp_dis < min_dis) {
      result.s = s;
      min_dis = temp_dis;
      if (min_dis < collision_threshold) {
        result.is_collision = true;
        result.min_distance = min_dis;
        result.ego_poit.x = point_.x();
        result.ego_poit.y = point_.y();
        result.ego_poit.theta = point_.theta();
        result.collision_type = collision_type;
        result.collision_object_position.x = box.center().x();
        result.collision_object_position.y = box.center().y();
        result.collision_object_position.theta = box.heading();
        return result;
      }
    }
  }

  result.is_collision = false;
  result.min_distance = min_dis;
  return result;
}

CollisionCheckStatus CollisionChecker::collision_check(
    const planning_math::Polygon2d &polygon, const double collision_threshold,
    CollisionCheckStatus::CollisionType collision_type) {
  using namespace ad_common::math;
  planning_math::Box2d ego_box;
  CollisionCheckStatus result;
  double s = 0;
  double min_dis = 100;
  bool init = false;
  if (is_trajectory()) {
    std::vector<PathPoint>::iterator iter = trajectory_.begin();
    for (const auto &point : trajectory_) {
      s += fast_hypot(iter->x() - point.x(), iter->y() - point.y());

      ego_model_->set_model_center(point);

      ego_model_->set_origin_model();
      double temp_dis = ego_model_->get_ego_model_polygon().DistanceTo(polygon);
      if (temp_dis < min_dis) {
        result.s = s;
        min_dis = temp_dis;
        if (min_dis < collision_threshold) {
          result.is_collision = true;
          result.min_distance = min_dis;
          result.ego_poit.x = point.x();
          result.ego_poit.y = point.y();
          result.ego_poit.theta = point.theta();
          result.collision_type = collision_type;
          result.collision_object_position.x =
              0.5 * (polygon.min_x() + polygon.max_x());
          result.collision_object_position.y =
              0.5 * (polygon.min_y() + polygon.max_y());
          return result;
        }
      }

      if (init) {
        iter++;
      } else {
        init = true;
      }
    }
  }

  else {
    ego_model_->set_model_center(point_);

    ego_model_->set_origin_model();
    double temp_dis = ego_model_->get_ego_model_polygon().DistanceTo(polygon);
    if (temp_dis < min_dis) {
      result.s = s;
      min_dis = temp_dis;
      if (min_dis < collision_threshold) {
        result.is_collision = true;
        result.min_distance = min_dis;
        result.ego_poit.x = point_.x();
        result.ego_poit.y = point_.y();
        result.ego_poit.theta = point_.theta();
        result.collision_type = collision_type;
        result.collision_object_position.x =
            0.5 * (polygon.min_x() + polygon.max_x());
        result.collision_object_position.y =
            0.5 * (polygon.min_y() + polygon.max_y());
        return result;
      }
    }
  }

  result.is_collision = false;
  result.min_distance = min_dis;
  return result;
}

CollisionCheckStatus CollisionChecker::collision_check(
    const planning_math::LineSegment2d &line, const double collision_threshold,
    CollisionCheckStatus::CollisionType collision_type) {
  using namespace ad_common::math;
  planning_math::Box2d ego_box;
  CollisionCheckStatus result;
  double s = 0;
  double min_dis = 100;
  double temp_dis = 100.0;
  bool init = false;
  if (is_trajectory()) {
    std::vector<PathPoint>::iterator iter = trajectory_.begin();
    for (const auto &point : trajectory_) {
      ego_model_->set_model_center(point);
      s += fast_hypot(iter->x() - point.x(), iter->y() - point.y());

      ego_model_->set_origin_model();
      temp_dis = ego_model_->get_ego_model_polygon().DistanceTo(line);

      if (temp_dis < min_dis) {
        result.s = s;
        min_dis = temp_dis;
        if (min_dis < collision_threshold) {
          result.is_collision = true;
          result.min_distance = min_dis;
          result.ego_poit.x = point.x();
          result.ego_poit.y = point.y();
          result.ego_poit.theta = point.theta();
          result.collision_type = collision_type;
          result.collision_object_position.x =
              0.5 * (line.start().x() + line.end().x());
          result.collision_object_position.y =
              0.5 * (line.start().y() + line.end().y());
          result.collision_object_position.theta = line.heading();
          return result;
        }
      }
      if (init) {
        iter++;
      } else {
        init = true;
      }
    }
  } else {
    ego_model_->set_model_center(point_);
    ego_model_->set_origin_model();

    min_dis =
        std::min(min_dis, ego_model_->get_ego_model_polygon().DistanceTo(line));
    if (min_dis < collision_threshold) {
      result.is_collision = true;
      result.min_distance = min_dis;
      result.ego_poit.x = point_.x();
      result.ego_poit.y = point_.y();
      result.ego_poit.theta = point_.theta();
      result.collision_type = collision_type;
      result.collision_object_position.x =
          0.5 * (line.start().x() + line.end().x());
      result.collision_object_position.y =
          0.5 * (line.start().y() + line.end().y());
      result.collision_object_position.theta = line.heading();
      return result;
    }
  }

  result.is_collision = false;
  result.min_distance = min_dis;
  return result;
}

CollisionCheckStatus CollisionChecker::collision_check(
    const planning_math::Vec2d &single_point, const double collision_threshold,
    CollisionCheckStatus::CollisionType collision_type) {
  using namespace ad_common::math;
  planning_math::Box2d ego_box;
  CollisionCheckStatus result;
  double s = 0;
  double min_dis = 100;
  double temp_dis = 100.0;
  bool init = false;
  if (is_trajectory()) {
    std::vector<PathPoint>::iterator iter = trajectory_.begin();
    for (const auto &point : trajectory_) {
      ego_model_->set_model_center(point);
      s += fast_hypot(iter->x() - point.x(), iter->y() - point.y());
      ego_model_->set_origin_model();
      temp_dis = ego_model_->get_ego_model_polygon().DistanceTo(single_point);
      if (temp_dis < min_dis) {
        result.s = s;
        min_dis = temp_dis;
        if (min_dis < collision_threshold) {
          result.is_collision = true;
          result.min_distance = min_dis;
          result.ego_poit.x = point.x();
          result.ego_poit.y = point.y();
          result.ego_poit.theta = point.theta();
          result.collision_type = collision_type;
          result.collision_object_position.x = single_point.x();
          result.collision_object_position.y = single_point.y();
          return result;
        }
      }
      if (init) {
        iter++;
      } else {
        init = true;
      }
    }
  } else {
    ego_model_->set_model_center(point_);
    ego_model_->set_origin_model();
    min_dis = std::min(
        min_dis, ego_model_->get_ego_model_polygon().DistanceTo(single_point));
    if (min_dis < collision_threshold) {
      result.is_collision = true;
      result.min_distance = min_dis;
      result.ego_poit.x = point_.x();
      result.ego_poit.y = point_.y();
      result.ego_poit.theta = point_.theta();
      result.collision_type = collision_type;
      result.collision_object_position.x = single_point.x();
      result.collision_object_position.y = single_point.y();
      return result;
    }
  }
  result.is_collision = false;
  result.min_distance = min_dis;
  return result;
}

bool CollisionChecker::is_polygon_within_range(
    const planning_math::Polygon2d &polygon, double xy_range) {
  if (xy_range <= 0) {
    return true;
  }
  // use AABB Box
  EgoModelManager ego_model;
  ego_model.set_model_center(point_);
  ego_model.set_origin_model();
  if (polygon.max_x() < ego_model.get_ego_model_polygon().min_x() - xy_range ||
      polygon.min_x() > ego_model.get_ego_model_polygon().max_x() + xy_range ||
      polygon.max_y() < ego_model.get_ego_model_polygon().min_y() - xy_range ||
      polygon.min_y() > ego_model.get_ego_model_polygon().max_y() + xy_range) {
    return false;
  } else {
    return true;
  }
}
}  // namespace planning_math
}  // namespace planning