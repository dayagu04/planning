#include "trajectory_generator/result_trajectory_generator.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include "environmental_model.h"

// #include "core/common/trace.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "math/math_utils.h"
#include "math_lib.h"

namespace planning {

using namespace std;
using namespace planning::planning_math;
using namespace pnc::mathlib;

ResultTrajectoryGenerator::ResultTrajectoryGenerator(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<ResultTrajectoryGeneratorConfig>();
  name_ = "ResultTrajectoryGenerator";
  Init();
}

void ResultTrajectoryGenerator::Init() {}

bool ResultTrajectoryGenerator::Execute(planning::framework::Frame *frame) {
  if (Task::Execute(frame) == false) {
    return false;
  }

  const auto &location_valid =
      frame_->session()->environmental_model().location_valid();

  bool res = false;
  if (location_valid) {
    res = TrajectoryGenerator();
  } else {
    res = RealtimeTrajectoryGenerator();
  }

  return res;
}

bool ResultTrajectoryGenerator::TrajectoryGenerator() {
  auto &ego_planning_result = pipeline_context_->planning_result;
  // Step 1) get x,y of trajectory points
  auto &traj_points = ego_planning_result.traj_points;
  // const auto &num_point = traj_points.size();
  auto &motion_planning_info = frame_->mutable_session()
                                   ->mutable_planning_context()
                                   ->mutable_planning_result()
                                   .motion_planning_info;

  pnc::mathlib::spline s_t_spline;
  pnc::mathlib::spline l_t_spline;

  pnc::mathlib::spline curvature_t_spline;
  pnc::mathlib::spline dkappa_t_spline;
  pnc::mathlib::spline ddkappa_t_spline;

  auto const N = traj_points.size();

  std::vector<double> t_vec(N);
  std::vector<double> s_vec(N);
  std::vector<double> l_vec(N);
  std::vector<double> curvature_vec(N);
  std::vector<double> dkappa_vec(N);
  std::vector<double> ddkappa_vec(N);

  for (size_t i = 0; i < traj_points.size(); i++) {
    if (config_.is_pwj_planning) {
      Point2D frenet_pt{traj_points[i].s, traj_points[i].l};
      Point2D cart_pt;
      if (!frenet_coord_->SLToXY(frenet_pt, cart_pt)) {
        LOG_ERROR("ResultTrajectoryGenerator::execute, transform failed \n");
        return false;
      }

      traj_points[i].x = cart_pt.x;
      traj_points[i].y = cart_pt.y;

      LOG_DEBUG("result traj_point s=%f, l=%f, x=%f, y=%f \n", traj_points[i].s,
                traj_points[i].l, traj_points[i].x, traj_points[i].y);
    }
    t_vec[i] = traj_points[i].t;
    s_vec[i] = traj_points[i].s;
    l_vec[i] = traj_points[i].l;
    curvature_vec[i] = traj_points[i].curvature;
    dkappa_vec[i] = traj_points[i].dkappa;
    ddkappa_vec[i] = traj_points[i].ddkappa;
  }
  JSON_DEBUG_VECTOR("traj_s_vec", s_vec, 3)
  s_t_spline.set_points(t_vec, s_vec);
  l_t_spline.set_points(t_vec, l_vec);

  curvature_t_spline.set_points(t_vec, curvature_vec);
  dkappa_t_spline.set_points(t_vec, dkappa_vec);
  ddkappa_t_spline.set_points(t_vec, ddkappa_vec);

  // Step 2) get dense trajectory points
  auto &planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  // double init_point_relative_time = planning_init_point.relative_time;

  std::vector<TrajectoryPoint> dense_traj_points;
  int dense_num_points =
      int(traj_points.back().t / config_.planning_result_delta_time) + 1;

  for (int j = 0; j < dense_num_points; ++j) {
    TrajectoryPoint traj_pt;
    double t = j * config_.planning_result_delta_time;

    // traj_pt.t = init_point_relative_time + t;
    traj_pt.t = t;
    traj_pt.s = s_t_spline(t);
    traj_pt.x = motion_planning_info.x_t_spline(t);
    traj_pt.y = motion_planning_info.y_t_spline(t);
    traj_pt.v = motion_planning_info.v_t_spline(t);
    traj_pt.a = motion_planning_info.a_t_spline(t);
    traj_pt.l = l_t_spline(t);
    traj_pt.heading_angle = motion_planning_info.theta_t_spline(t);
    traj_pt.curvature = curvature_t_spline(t);
    traj_pt.dkappa = dkappa_t_spline(t);
    traj_pt.ddkappa = ddkappa_t_spline(t);
    traj_pt.frenet_valid = true;
    dense_traj_points.push_back(std::move(traj_pt));
  }

  // Step 3) extends to max length if needed
  double desired_length = planning_init_point.frenet_state.s;

  while (dense_traj_points.back().s < desired_length) {
    auto traj_pt = dense_traj_points.back();
    traj_pt.s += 0.2;
    traj_pt.t += config_.planning_result_delta_time;

    Point2D frenet_pt{traj_pt.s, traj_pt.l};
    Point2D cart_pt;
    if (!frenet_coord_->SLToXY(frenet_pt, cart_pt)) {
      LOG_ERROR("ResultTrajectoryGenerator::execute, transform failed \n");
      return false;
    }

    traj_pt.x = cart_pt.x;
    traj_pt.y = cart_pt.y;
    dense_traj_points.emplace_back(std::move(traj_pt));
  }

  const auto N_ext = dense_traj_points.size();
  std::vector<double> traj_x_vec(N_ext);
  std::vector<double> traj_y_vec(N_ext);

  for (size_t i = 0; i < N_ext; ++i) {
    traj_x_vec[i] = dense_traj_points[i].x;
    traj_y_vec[i] = dense_traj_points[i].y;
  }

  JSON_DEBUG_VECTOR("traj_x_vec", traj_x_vec, 3)
  JSON_DEBUG_VECTOR("traj_y_vec", traj_y_vec, 3)

  ego_planning_result.traj_points = dense_traj_points;

  // record some results
  ego_planning_result.motion_planning_info = motion_planning_info;

  return true;
}

bool ResultTrajectoryGenerator::RealtimeTrajectoryGenerator() {
  bool enable_lat_traj = config_.enable_lat_traj;

  auto &ego_planning_result = pipeline_context_->planning_result;

  // Step 1) get x,y of trajectory points
  auto &traj_points = ego_planning_result.traj_points;
  auto &motion_planning_info = frame_->mutable_session()
                                   ->mutable_planning_context()
                                   ->mutable_planning_result()
                                   .motion_planning_info;

  pnc::mathlib::spline s_t_spline;
  pnc::mathlib::spline curvature_t_spline;
  pnc::mathlib::spline dkappa_t_spline;
  pnc::mathlib::spline ddkappa_t_spline;

  auto const N = traj_points.size();

  std::vector<double> t_vec(N);
  std::vector<double> s_vec(N);
  std::vector<double> curvature_vec(N);
  std::vector<double> dkappa_vec(N);
  std::vector<double> ddkappa_vec(N);

  for (size_t i = 0; i < traj_points.size(); i++) {
    t_vec[i] = traj_points[i].t;
    s_vec[i] = traj_points[i].s;
    curvature_vec[i] = traj_points[i].curvature;
    dkappa_vec[i] = traj_points[i].dkappa;
    ddkappa_vec[i] = traj_points[i].ddkappa;
  }

  s_t_spline.set_points(t_vec, s_vec);

  curvature_t_spline.set_points(t_vec, curvature_vec);
  dkappa_t_spline.set_points(t_vec, dkappa_vec);
  ddkappa_t_spline.set_points(t_vec, ddkappa_vec);

  // combination of genereted trajectory and reference without localization
  const double lat_err_norm =
      std::fabs(motion_planning_info.ref_y_t_spline(0.0));
  static const std::vector<double> lat_err_norm_tab = {0.0, 0.3, 0.5, 100.0};
  static const std::vector<double> alpha_tab = {1.0, 1.0, 0.0, 0.0};

  double alpha = 1.0;
  if (enable_lat_traj) {
    // 使能横向轨迹，采用轨迹和参考线合并的形式
    alpha = pnc::mathlib::Interp1(lat_err_norm_tab, alpha_tab, lat_err_norm);
  }

  // Step 2) get dense trajectory points
  std::vector<TrajectoryPoint> dense_traj_points;
  size_t dense_num_points =
      int(traj_points.back().t / config_.planning_result_delta_time) + 1;
  dense_traj_points.reserve(dense_num_points);

  for (int j = 0; j < dense_num_points; ++j) {
    TrajectoryPoint traj_pt;
    double t = j * config_.planning_result_delta_time;

    traj_pt.t = t;
    traj_pt.s = s_t_spline(t);

    // combination of genereted trajectory and reference considering x and y
    traj_pt.x = motion_planning_info.x_t_spline(t) * (1.0 - alpha) +
                motion_planning_info.ref_x_t_spline(t) * alpha;

    traj_pt.y = motion_planning_info.y_t_spline(t) * (1.0 - alpha) +
                motion_planning_info.ref_y_t_spline(t) * alpha;

    traj_pt.v = motion_planning_info.v_t_spline(t);
    traj_pt.a = motion_planning_info.a_t_spline(t);
    traj_pt.heading_angle = motion_planning_info.theta_t_spline(t);
    traj_pt.curvature = curvature_t_spline(t);
    traj_pt.dkappa = dkappa_t_spline(t);
    traj_pt.ddkappa = ddkappa_t_spline(t);
    traj_pt.frenet_valid = true;
    dense_traj_points.emplace_back(std::move(traj_pt));
  }

  const auto N_ext = dense_traj_points.size();
  std::vector<double> traj_x_vec(N_ext);
  std::vector<double> traj_y_vec(N_ext);

  for (size_t i = 0; i < N_ext; ++i) {
    traj_x_vec[i] = dense_traj_points[i].x;
    traj_y_vec[i] = dense_traj_points[i].y;
  }

  JSON_DEBUG_VECTOR("traj_x_vec", traj_x_vec, 3)
  JSON_DEBUG_VECTOR("traj_y_vec", traj_y_vec, 3)

  ego_planning_result.traj_points = dense_traj_points;

  // record some results
  ego_planning_result.motion_planning_info = motion_planning_info;

  return true;
}

}  // namespace planning
