#include "collision_detection.h"

#include <cstddef>
#include <utility>

#include "dubins_lib/geometry_math.h"

namespace planning {

static const std::vector<double> car_circle_x_vec = {
    1.28, 3.39, 3.39, 1.98, -0.82, -0.82, 1.98, 2.8, 1.8, 0.8, -0.2};

static const std::vector<double> car_circle_y_vec = {
    0.0, 0.58, -0.58, -1.00, -0.62, 0.62, 1.00, 0.0, 0.0, 0.0, 0.0};

static const std::vector<double> car_circle_radius_vec = {
    2.6, 0.36, 0.36, 0.13, 0.38, 0.38, 0.13, 0.97, 0.97, 0.97, 0.97};

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

const std::vector<pnc::geometry_lib::Circle>
CollisionDetector::GetCarCircleByEgoCarGlobal() const {
  double ego_theta =
      local_view_ptr_->localization_estimate.pose().euler_angles().yaw();

  Eigen::Vector2d ego_coord_global(
      local_view_ptr_->localization_estimate.pose().local_position().x(),
      local_view_ptr_->localization_estimate.pose().local_position().y());

  pnc::geometry_lib::LocalToGlobalTf l2g_tf(ego_coord_global, ego_theta);

  std::vector<pnc::geometry_lib::Circle> car_circle_global_vec;
  car_circle_global_vec.clear();
  car_circle_global_vec.reserve(car_circle_local_vec_.size());
  pnc::geometry_lib::Circle car_circle_global;

  for (const auto &circle_local : car_circle_local_vec_) {
    car_circle_global.center = l2g_tf.GetPos(circle_local.center);
    car_circle_global.radius = circle_local.radius;
    car_circle_global_vec.emplace_back(car_circle_global);
  }
  return car_circle_global_vec;
}

void CollisionDetector::GenObstaclesByUssOA() {
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

void CollisionDetector::GenObstaclesSimulation(
    const std::vector<pnc::geometry_lib::LineSegment> &obstacle_line_vec) {
  obstacle_global_vec_ = obstacle_line_vec;
}

void CollisionDetector::GenCarCircles(
    const std::vector<pnc::dubins_lib::DubinsLibrary::PathPoint>
        &path_point_vec) {
  const auto N = path_point_vec.size();

  car_circle_global_vec_path_vec_.clear();
  car_circle_global_vec_path_vec_.reserve(N);

  std::vector<pnc::geometry_lib::Circle> car_circle_global_vec;
  car_circle_global_vec.clear();
  car_circle_global_vec.reserve(car_circle_local_vec_.size());
  pnc::geometry_lib::Circle car_circle_global;
  pnc::geometry_lib::LocalToGlobalTf l2g_tf;

  for (const auto &path_point : path_point_vec) {
    l2g_tf.Init(path_point.pos, path_point.heading);
    for (const auto &car_circle_local : car_circle_local_vec_) {
      car_circle_global.center = l2g_tf.GetPos(car_circle_local.center);
      car_circle_global.radius = car_circle_local.radius;
      car_circle_global_vec.emplace_back(std::move(car_circle_global));
    }
    car_circle_global_vec_path_vec_.emplace_back(
        std::move(car_circle_global_vec));
  }

  // for (size_t i = 0; i < N; ++i) {
  //   l2g_tf.Init(path_point_vec[i].pos, path_point_vec[i].heading);

  //   std::vector<pnc::geometry_lib::Circle> car_circle_global_vec;

  //   for (const auto &car_circle_local : car_circle_local_vec_) {
  //     pnc::geometry_lib::Circle car_circle_global;
  //     car_circle_global.center = l2g_tf.GetPos(car_circle_local.center);

  //     car_circle_global.radius = car_circle_local.radius;
  //     car_circle_global_vec.emplace_back(std::move(car_circle_global));
  //   }

  //   car_circle_global_vec_path_vec_.emplace_back(
  //       std::move(car_circle_global_vec));
  // }
}

bool CollisionDetector::CollisionDetect() {
  if (obstacle_global_vec_.size() < 1) {
    // std::cout << "obstacle_global_vec_ size error!" << std::endl;
    return false;
  }

  bool collision_flag = false;
  // std::cout << "\ncar_circle_global_vec_path_vec_"
  //           << car_circle_global_vec_path_vec_.size();
  size_t path_count = 0;
  for (const auto &car_circle_global_vec : car_circle_global_vec_path_vec_) {
    if (path_count++ == 0) {
      continue;
    }

    for (const auto &obstacle_global : obstacle_global_vec_) {
      for (size_t k = 0; k < car_circle_global_vec.size(); k++) {
        collision_flag = pnc::geometry_lib::CheckLineSegmentInCircle(
            obstacle_global, car_circle_global_vec[k]);
        // std::cout << " collision_flag_pb:" << k << ": " << collision_flag;
        //  first large circle to avoid current obstacle
        if (k == 0 && collision_flag == false) {
          // std::cout << i << "point first large circle is not in collision\n";
          break;
        }

        // once collision to return
        if (k > 0 && collision_flag == true) {
          // std::cout << i << " point " << k << " circle is in collision\n";
          return true;
        }
      }
    }
    // i++;
  }
  return false;
}

};  // namespace planning