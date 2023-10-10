#include "collision_detection.h"

#include <cstddef>
#include <utility>

#include "dubins_lib/geometry_math.h"

namespace planning {

static const std::vector<double> car_circle_x_vec = {
    1.35, 3.3, 3.3, 2.02, -0.55, -0.55, 2.02, 2.7, 1.8, 0.9, 0.0};

static const std::vector<double> car_circle_y_vec = {
    0.0, -0.55, 0.55, 0.95, 0.5, -0.5, -0.95, 0.0, 0.0, 0.0, 0.0};

static const std::vector<double> car_circle_radius_vec = {
    2.4, 0.35, 0.35, 0.18, 0.35, 0.35, 0.18, 0.95, 0.95, 0.95, 0.95};

void CollisionDetector::Init() {
  car_circle_local_vec_.clear();
  car_circle_local_vec_.reserve(car_circle_x_vec.size());
  for (size_t i = 0; i < car_circle_x_vec.size(); ++i) {
    pnc::geometry_lib::Circle car_circle_local;
    car_circle_local.center =
        Eigen::Vector2d(car_circle_x_vec[i], car_circle_y_vec[i]);

    car_circle_local.radius = car_circle_radius_vec[i];
    car_circle_local_vec_.emplace_back(std::move(car_circle_local));
  }
}
void CollisionDetector::Reset() {
  obstacle_global_vec_.clear();
  car_circle_global_vec_path_vec_.clear();
}

void CollisionDetector::GenObstacles() {
  double remain_s_uss = uss_oa_ptr_->GetRemainDist();
  if (remain_s_uss < 0.35) {
    pnc::geometry_lib::LineSegment min_dist_uss_line_local =
        uss_oa_ptr_->GetMinDistUssLine();

    pnc::geometry_lib::LineSegment min_dist_uss_line_global;
    pnc::geometry_lib::LocalToGlobalTf l2g_tf;
    double ego_theta =
        local_view_ptr_->localization_estimate.pose().euler_angles().yaw();

    Eigen::Vector2d ego_coord_global(
        local_view_ptr_->localization_estimate.pose().local_position().x(),
        local_view_ptr_->localization_estimate.pose().local_position().y());

    l2g_tf.Init(ego_coord_global, ego_theta);

    min_dist_uss_line_global.SetPoints(
        l2g_tf.GetPos(min_dist_uss_line_local.pA),
        l2g_tf.GetPos(min_dist_uss_line_local.pB));

    obstacle_global_vec_.emplace_back(std::move(min_dist_uss_line_global));
  }
}

void CollisionDetector::GenCarCircles(
    std::vector<pnc::dubins_lib::DubinsLibrary::PathPoint> &path_point_vec) {
  const auto N = path_point_vec.size();
  car_circle_global_vec_path_vec_.clear();
  car_circle_global_vec_path_vec_.reserve(N);

  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  for (size_t i = 0; i < N; ++i) {
    l2g_tf.Init(path_point_vec[i].pos, path_point_vec[i].heading);

    std::vector<pnc::geometry_lib::Circle> car_circle_global_vec;

    for (const auto &car_circle_local : car_circle_local_vec_) {
      pnc::geometry_lib::Circle car_circle_global;
      car_circle_global.center = l2g_tf.GetPos(car_circle_local.center);

      car_circle_global.radius = car_circle_local.radius;
      car_circle_global_vec.emplace_back(std::move(car_circle_global));
    }

    car_circle_global_vec_path_vec_.emplace_back(
        std::move(car_circle_global_vec));
  }
}

bool CollisionDetector::CollisionDetect() {
  if (obstacle_global_vec_.size() < 1) {
    return false;
  }

  bool collision_flag = false;

  for (const auto &car_circle_global_vec_path :
       car_circle_global_vec_path_vec_) {
    for (const auto &obstacle_global : obstacle_global_vec_) {
      for (size_t k = 0; k < car_circle_global_vec_path.size(); k++) {
        collision_flag = pnc::geometry_lib::CheckLineSegmentInCircle(
            obstacle_global, car_circle_global_vec_path[k]);

        // first large circle to avoid current obstacle
        if (k == 0 && collision_flag == false) {
          break;
        }

        // once collision to return
        if (k > 0 && collision_flag == true) {
          return true;
        }
      }
    }
  }
  return false;
}

};  // namespace planning