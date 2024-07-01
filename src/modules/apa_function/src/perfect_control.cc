#include "perfect_control.h"

#include "geometry_math.h"
#include "math_lib.h"
#include "spline.h"
#include "spline_projection.h"
namespace planning {

static const double kVelSimulation = 1.0;
static const double kPie = 3.141592654;

void PerfectControl::Init() { state_.Reset(); }

const bool PerfectControl::Update(
    const iflyauto::PlanningOutput &planning_output, const double dt) {
  const auto path_size = planning_output.trajectory.trajectory_points_size;

  if (path_size < 2) {
    std::cout << "planning error: path_size = " << path_size << std::endl;
    return false;
  }

  std::vector<double> path_s_vec;
  std::vector<double> path_x_vec;
  std::vector<double> path_y_vec;
  std::vector<double> path_heading_vec;
  path_x_vec.reserve(path_size);
  path_y_vec.reserve(path_size);
  path_s_vec.reserve(path_size);
  path_heading_vec.reserve(path_size);

  double s = 0.0;
  double ds = 0.0;
  double angle_offset = 0.0;
  for (int i = 0; i < path_size; ++i) {
    const auto &traj_point = planning_output.trajectory.trajectory_points[i];

    path_x_vec.emplace_back(traj_point.x);
    path_y_vec.emplace_back(traj_point.y);
    path_s_vec.emplace_back(s);

    if (i <= path_size - 2) {
      const auto &traj_point_next =
          planning_output.trajectory.trajectory_points[i + 1];

      ds = std::hypot(traj_point_next.x - traj_point.x,
                      traj_point_next.y - traj_point.y);
      s += std::max(ds, 0.01);
    }

    auto heading = traj_point.heading_yaw;

    if (i > 0) {
      const auto &traj_point_last =
          planning_output.trajectory.trajectory_points[i - 1];

      const auto d_heading =
          traj_point.heading_yaw - traj_point_last.heading_yaw;

      if (d_heading > 1.5 * kPie) {
        angle_offset -= 2.0 * kPie;
      } else if (d_heading < -1.5 * kPie) {
        angle_offset += 2.0 * kPie;
      }

      heading += angle_offset;
    }

    path_heading_vec.emplace_back(heading);
  }

  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  pnc::mathlib::spline heading_s_spline;

  x_s_spline.set_points(path_s_vec, path_x_vec);
  y_s_spline.set_points(path_s_vec, path_y_vec);
  heading_s_spline.set_points(path_s_vec, path_heading_vec);

  const auto &current_pos = state_.pos;
  double path_length = path_s_vec.back();
  double s_proj = 0.0;
  bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
      0.0, path_length, s_proj, current_pos, x_s_spline, y_s_spline);

  double remain_dist = 5.01;
  if (success == true) {
    remain_dist = path_length - s_proj;
  } else {
    std::cout << "remain_dist calculation error:input is error" << std::endl;
  }

  // pnc::spline::Projection proj;

  // proj.CalProjectionPoint(x_s_spline, y_s_spline, path_s_vec.front(),
  //                         path_s_vec.back(), current_pos);

  // const auto s_proj = pnc::mathlib::Clamp(
  //     proj.GetOutput().s_proj, path_s_vec.front(), path_s_vec.back());

  // const auto remain_dist = path_s_vec.back() - s_proj;
  if (state_.static_flag) {
    if (state_.static_time > 0.6) {
      state_.static_flag = false;
    } else {
      state_.static_time += dt;
      state_.vel = 0.0;
      return true;
    }
  }

  if (fabs(remain_dist) <= 0.06) {
    state_.vel = 0.0;
    state_.static_flag = true;
  } else {
    auto s_next = s_proj + state_.vel * dt;
    state_.vel = kVelSimulation;
    state_.pos << x_s_spline(s_next), y_s_spline(s_next);

    state_.heading =
        pnc::geometry_lib::NormalizeAngle(heading_s_spline(s_next));
  }

  std::cout << "------ dynamics:" << std::endl;
  std::cout << "remain_dist = " << remain_dist << std::endl;
  std::cout << "s_proj = " << s_proj << std::endl;
  std::cout << "s_path = " << path_length << std::endl;

  return true;
}

}  // namespace planning
