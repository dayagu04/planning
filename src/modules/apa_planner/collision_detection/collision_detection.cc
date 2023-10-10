#include "collision_detection.h"

#include <cstddef>
#include <utility>

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
  car_circle_global_vec_.clear();
  uss_oa_ptr_ = nullptr;
  local_view_ptr_ = nullptr;
}

void CollisionDetector::GenObstacles() {
  double remain_s_uss = uss_oa_ptr_->GetRemainDist();
  if (remain_s_uss < 0.35) {
    pnc::geometry_lib::LineSegment min_dist_uss_line_local =
        uss_oa_ptr_->GetMinDistUssLine();

    pnc::geometry_lib::LineSegment min_dist_uss_line_global;
    PoseTf tf;
    double ego_theta =
        local_view_ptr_->localization_estimate.pose().euler_angles().yaw();

    Eigen::Vector2d ego_coord_global(
        local_view_ptr_->localization_estimate.pose().local_position().x(),
        local_view_ptr_->localization_estimate.pose().local_position().y());

    tf.SetEgoPose(ego_coord_global, ego_theta);

    min_dist_uss_line_global.pA = tf.GetGlobalPos(min_dist_uss_line_local.pA);
    min_dist_uss_line_global.pB = tf.GetGlobalPos(min_dist_uss_line_local.pB);

    obstacle_global_vec_.emplace_back(std::move(min_dist_uss_line_global));
  }
}

void CollisionDetector::GenCarCircles(
    PlanningOutput::PlanningOutput *const planning_output) {
  auto trajectory = planning_output->trajectory();
  std::vector<Eigen::Vector2d> pos_vec;
  std::vector<double> theta_vec;
  for (size_t i = 0; i < trajectory.trajectory_points_size(); i++) {
    pos_vec.emplace_back(Eigen::Vector2d(trajectory.trajectory_points(i).x(),
                                         trajectory.trajectory_points(i).y()));
    theta_vec.emplace_back(trajectory.trajectory_points(i).heading_yaw());
  }

  car_circle_global_vec_.clear();
  car_circle_global_vec_.reserve(pos_vec.size());
  for (size_t i = 0; i < pos_vec.size(); i++) {
    PoseTf tf;
    // pos_vec[i].x() = 5.0;
    // pos_vec[i].y() = 5.0;
    // theta_vec[i] = 0.5;
    tf.SetEgoPose(pos_vec[i], theta_vec[i]);
    std::vector<pnc::geometry_lib::Circle> car_circle_global_vec;

    for (size_t j = 0; j < car_circle_local_vec_.size(); j++) {
      pnc::geometry_lib::Circle car_circle_global;
      car_circle_global.center =
          tf.GetGlobalPos(car_circle_local_vec_[j].center);

      car_circle_global.radius = car_circle_local_vec_[j].radius;
      car_circle_global_vec.emplace_back(std::move(car_circle_global));

      // if (i == 0) {
      //   std::cout << j << "  local_C++:" << car_circle_local_vec_[j].center
      //             << "   global_C++:" << car_circle_global.center <<
      //             std::endl;
      // }
    }
    car_circle_global_vec_.emplace_back(std::move(car_circle_global_vec));
  }
}

bool CollisionDetector::CollisionDetect() {
  if (obstacle_global_vec_.size() < 1) {
    return false;
  }

  for (size_t i = 0; i < car_circle_global_vec_.size(); i++) {
    bool collision_flag = false;
    std::vector<pnc::geometry_lib::Circle> car_circle_global_vec =
        car_circle_global_vec_[i];

    for (size_t j = 0; j < obstacle_global_vec_.size(); j++) {
      pnc::geometry_lib::LineSegment line_obstacle = obstacle_global_vec_[j];

      for (size_t k = 0; k < car_circle_global_vec.size(); k++) {
        pnc::geometry_lib::Circle car_circle_global = car_circle_global_vec[k];
        collision_flag = pnc::geometry_lib::CheckLineSegmentInCircle(
            line_obstacle, car_circle_global);

        if (k == 0 && collision_flag == false) {
          break;
        }
        if (k > 0 && collision_flag == true) {
          return true;
        }
      }
    }
  }
  return false;
}

};  // namespace planning