#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "ilqr_core.h"
#include "spline.h"
namespace planning {

struct JointPlannerTrajectoryPoint {
  double t = 0.0;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double delta = 0.0;
  double omega = 0.0;
  double vel = 0.0;
  double acc = 0.0;
  double jerk = 0.0;
};

struct EgoTrajectory {
  std::vector<JointPlannerTrajectoryPoint> trajectory_points;

  void clear() { trajectory_points.clear(); }
  size_t size() const { return trajectory_points.size(); }
  bool empty() const { return trajectory_points.empty(); }
  void resize(size_t size) { trajectory_points.resize(size); }

  JointPlannerTrajectoryPoint& operator[](size_t index) {
    return trajectory_points[index];
  }

  const JointPlannerTrajectoryPoint& operator[](size_t index) const {
    return trajectory_points[index];
  }
};

class LatLonJointPlannerDeciderOutput {
 public:
  LatLonJointPlannerDeciderOutput() = default;
  ~LatLonJointPlannerDeciderOutput() = default;

  const EgoTrajectory& GetEgoTrajectory() const { return ego_trajectory_; }
  EgoTrajectory& GetEgoTrajectory() { return ego_trajectory_; }

  const std::vector<int32_t>& GetSelectedObstacleIds() const {
    return selected_obstacle_ids_;
  }
  std::vector<int32_t>& GetSelectedObstacleIds() {
    return selected_obstacle_ids_;
  }
  void SetSelectedObstacleIds(const std::vector<int32_t>& obstacle_ids) {
    selected_obstacle_ids_ = obstacle_ids;
  }

  void SetPlanningSuccess(bool success) { planning_success_ = success; }
  bool IsPlanningSuccess() const { return planning_success_; }

  void Clear() {
    ego_trajectory_.clear();
    selected_obstacle_ids_.clear();
    planning_success_ = false;
  }

  void BuildSplines() {
    if (ego_trajectory_.empty()) {
      return;
    }

    const size_t N = ego_trajectory_.size();

    std::vector<double> s_lat_vec_internal(N + 1);
    std::vector<double> x_vec(N + 1);
    std::vector<double> y_vec(N + 1);
    std::vector<double> theta_vec(N + 1);
    std::vector<double> delta_vec(N + 1);
    std::vector<double> omega_vec(N + 1);
    std::vector<double> t_vec_lat(N + 1);

    double s = 0.0;
    double t = 0.0;
    for (size_t i = 0; i < N; ++i) {
      x_vec[i + 1] = ego_trajectory_[i].x;
      y_vec[i + 1] = ego_trajectory_[i].y;
      theta_vec[i + 1] = ego_trajectory_[i].theta;
      delta_vec[i + 1] = ego_trajectory_[i].delta;
      omega_vec[i + 1] = ego_trajectory_[i].omega;

      if (i == 0) {
        s = 0.0;
        t = 0.0;
      } else {
        const double ds =
            std::hypot(x_vec[i + 1] - x_vec[i], y_vec[i + 1] - y_vec[i]);
        s += std::max(ds, 1e-3);
        t += 0.2;
      }
      s_lat_vec_internal[i + 1] = s;
      t_vec_lat[i + 1] = t;
    }

    std::vector<double> t_vec_lon(N);
    std::vector<double> v_vec(N);
    std::vector<double> a_vec(N);
    std::vector<double> j_vec(N);

    for (size_t i = 0; i < N; ++i) {
      t_vec_lon[i] = ego_trajectory_[i].t;
      v_vec[i] = ego_trajectory_[i].vel;
      a_vec[i] = ego_trajectory_[i].acc;
      j_vec[i] = ego_trajectory_[i].jerk;
    }

    const double appended_length = 2.5;

    const double dx = x_vec[1] - x_vec[2];
    const double dy = y_vec[1] - y_vec[2];
    const double norm = std::sqrt(dx * dx + dy * dy);
    double unit_x = 0.0, unit_y = 0.0;
    if (norm > 1e-6) {
      unit_x = dx / norm;
      unit_y = dy / norm;
    }

    s_lat_vec_internal[0] = -appended_length;
    x_vec[0] = x_vec[1] + unit_x * appended_length;
    y_vec[0] = y_vec[1] + unit_y * appended_length;
    theta_vec[0] = theta_vec[1];
    delta_vec[0] = delta_vec[1];
    omega_vec[0] = omega_vec[1];
    t_vec_lat[0] = -0.2;

    x_s_spline.set_points(s_lat_vec_internal, x_vec);
    y_s_spline.set_points(s_lat_vec_internal, y_vec);
    theta_s_spline.set_points(s_lat_vec_internal, theta_vec);
    delta_s_spline.set_points(s_lat_vec_internal, delta_vec);
    omega_s_spline.set_points(s_lat_vec_internal, omega_vec);

    const double curv_factor = 0.33;
    std::vector<double> curv_vec(N + 1);
    std::vector<double> d_curv_vec(N + 1);
    for (size_t i = 0; i < N + 1; ++i) {
      curv_vec[i] = curv_factor * delta_vec[i];
      d_curv_vec[i] = curv_factor * omega_vec[i];
    }
    curv_s_spline.set_points(s_lat_vec_internal, curv_vec);
    d_curv_s_spline.set_points(s_lat_vec_internal, d_curv_vec);

    lateral_s_t_spline.set_points(t_vec_lat, s_lat_vec_internal);
    lateral_t_s_spline.set_points(s_lat_vec_internal, t_vec_lat);

    v_t_spline.set_points(t_vec_lon, v_vec);
    a_t_spline.set_points(t_vec_lon, a_vec);
    j_t_spline.set_points(t_vec_lon, j_vec);

    s_lat_vec = s_lat_vec_internal;
    path_backward_appended_length = appended_length;
  }

  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  pnc::mathlib::spline theta_s_spline;
  pnc::mathlib::spline delta_s_spline;
  pnc::mathlib::spline omega_s_spline;
  pnc::mathlib::spline v_t_spline;
  pnc::mathlib::spline a_t_spline;
  pnc::mathlib::spline j_t_spline;
  pnc::mathlib::spline x_t_spline;
  pnc::mathlib::spline y_t_spline;
  pnc::mathlib::spline theta_t_spline;
  pnc::mathlib::spline delta_t_spline;
  pnc::mathlib::spline omega_t_spline;
  pnc::mathlib::spline lateral_x_t_spline;
  pnc::mathlib::spline lateral_y_t_spline;
  pnc::mathlib::spline lateral_theta_t_spline;
  pnc::mathlib::spline lateral_s_t_spline;
  pnc::mathlib::spline lateral_t_s_spline;
  pnc::mathlib::spline curv_s_spline;
  pnc::mathlib::spline d_curv_s_spline;
  std::vector<double> s_lat_vec;
  double path_backward_appended_length = 0.0;

 private:
  EgoTrajectory ego_trajectory_;
  std::vector<int32_t> selected_obstacle_ids_;
  bool planning_success_ = false;
};

}  // namespace planning
