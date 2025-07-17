#include "tasks/trajectory_generator/result_trajectory_generator.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include "environmental_model.h"

// #include "core/common/trace.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "math/math_utils.h"
#include "math_lib.h"
#include "planning_context.h"

namespace planning {

using namespace std;
using namespace planning::planning_math;
using namespace pnc::mathlib;

ResultTrajectoryGenerator::ResultTrajectoryGenerator(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<ResultTrajectoryGeneratorConfig>();
  name_ = "ResultTrajectoryGenerator";
  Init();
}

void ResultTrajectoryGenerator::Init() {}

bool ResultTrajectoryGenerator::Execute() {
  LOG_DEBUG("=======ResultTrajectoryGenerator======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  auto start_time = IflyTime::Now_ms();

  // const auto &location_valid =
  // session_->environmental_model().location_valid();
  bool res = false;
  res = TrajectoryGenerator();

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("TrajectoryGeneratorCostTime", end_time - start_time);
  return res;
}

bool ResultTrajectoryGenerator::TrajectoryGenerator() {
  auto &ego_planning_result =
      session_->mutable_planning_context()->mutable_planning_result();

  // Step 1) get x,y of trajectory points
  auto &traj_points = ego_planning_result.traj_points;
  ego_planning_result.raw_traj_points = traj_points;
  std::copy(traj_points.begin(), traj_points.end(),
            ego_planning_result.raw_traj_points.begin());
  // const auto &num_point = traj_points.size();
  auto &motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();

  pnc::mathlib::spline s_t_spline;
  pnc::mathlib::spline l_t_spline;

  pnc::mathlib::spline curvature_t_spline;
  pnc::mathlib::spline dkappa_t_spline;
  pnc::mathlib::spline ddkappa_t_spline;

  auto const N = traj_points.size();
  double traj_max_lat_acc = 0.0;
  double traj_max_lat_jerk = 0.0;
  double traj_max_lon_acc = 0.0;
  double traj_max_lon_jerk = 0.0;
  std::vector<double> t_vec(N);
  std::vector<double> s_vec(N);
  std::vector<double> l_vec(N);
  std::vector<double> curvature_vec(N);
  std::vector<double> dkappa_vec(N);
  std::vector<double> ddkappa_vec(N);
  std::vector<double> lat_acc_vec(N);
  std::vector<double> lat_jerk_vec(N);
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto &frenet_coord = reference_path_ptr->get_frenet_coord();
  const double tp_init_s = traj_points.front().s;
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (config_.is_pwj_planning) {
      Point2D frenet_pt{traj_points[i].s, traj_points[i].l};
      Point2D cart_pt;
      if (!frenet_coord->SLToXY(frenet_pt, cart_pt)) {
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
    double tp_curvature =
        motion_planner_output.curv_s_spline(traj_points[i].s - tp_init_s);
    curvature_vec[i] = tp_curvature;
    double tp_dcurvature =
        motion_planner_output.d_curv_s_spline(traj_points[i].s - tp_init_s);
    dkappa_vec[i] = tp_dcurvature;
    ddkappa_vec[i] = traj_points[i].ddkappa;
    double tp_delta =
        motion_planner_output.delta_s_spline(traj_points[i].s - tp_init_s);
    double tp_lat_acc =
        config_.curv_factor * traj_points[i].v * traj_points[i].v * tp_delta;
    lat_acc_vec[i] = tp_lat_acc;
    traj_max_lat_acc = std::max(std::fabs(tp_lat_acc), traj_max_lat_acc);
    double tp_omega =
        motion_planner_output.omega_s_spline(traj_points[i].s - tp_init_s);
    double tp_lat_jerk =
        config_.curv_factor * traj_points[i].v * traj_points[i].v * tp_omega;
    lat_jerk_vec[i] = tp_lat_jerk;
    traj_max_lat_jerk = std::max(std::fabs(tp_lat_jerk), traj_max_lat_jerk);
    traj_max_lon_acc = std::max(std::fabs(traj_points[i].a), traj_max_lon_acc);
    traj_max_lon_jerk = std::max(std::fabs(traj_points[i].jerk), traj_max_lon_jerk);
  }
  JSON_DEBUG_VECTOR("traj_lat_acc_vec", lat_acc_vec, 3)
  JSON_DEBUG_VECTOR("traj_lat_jerk_vec", lat_jerk_vec, 3)
  JSON_DEBUG_VECTOR("traj_s_vec", s_vec, 3)
  s_t_spline.set_points(t_vec, s_vec);
  l_t_spline.set_points(t_vec, l_vec);

  curvature_t_spline.set_points(t_vec, curvature_vec);
  dkappa_t_spline.set_points(t_vec, dkappa_vec);
  ddkappa_t_spline.set_points(t_vec, ddkappa_vec);
  double lat_jerk_thr = config_.lat_jerk_thr;
  // dynamic lat jerk thr
  if (config_.use_dynamic_lat_jerk_thr) {
    const auto &lateral_motion_planning_input =
        DebugInfoManager::GetInstance()
            .GetDebugInfoPb()
            ->lateral_motion_planning_input();
    lat_jerk_thr =
        lateral_motion_planning_input.jerk_bound();
  }
  // judge condition
  auto &ad_info =
      session_->mutable_planning_context()
              ->mutable_planning_hmi_info()
              ->ad_info;
  if ((traj_max_lat_acc > config_.lat_acc_thr) ||
      (traj_max_lat_jerk > lat_jerk_thr) ||
      (traj_max_lon_acc > config_.lon_acc_thr) ||
      (traj_max_lon_jerk > config_.lon_jerk_thr)) {
    ad_info.is_avaliable = false;
  } else {
    ad_info.is_avaliable = true;
  }
  // Step 2) get dense trajectory points

  std::vector<TrajectoryPoint> dense_traj_points;
  int dense_num_points =
      int(traj_points.back().t / config_.planning_result_delta_time) + 1;

  for (int j = 0; j < dense_num_points; ++j) {
    TrajectoryPoint traj_pt;
    double t = j * config_.planning_result_delta_time;

    // traj_pt.t = init_point_relative_time + t;
    traj_pt.t = t;
    traj_pt.s = s_t_spline(t);
    traj_pt.x = motion_planner_output.x_t_spline(t);
    traj_pt.y = motion_planner_output.y_t_spline(t);
    traj_pt.v = motion_planner_output.v_t_spline(t);
    traj_pt.a = motion_planner_output.a_t_spline(t);
    traj_pt.jerk = motion_planner_output.j_t_spline(t);
    traj_pt.l = l_t_spline(t);
    traj_pt.heading_angle = motion_planner_output.theta_t_spline(t);
    traj_pt.curvature = curvature_t_spline(t);
    traj_pt.dkappa = dkappa_t_spline(t);
    traj_pt.ddkappa = ddkappa_t_spline(t);
    traj_pt.frenet_valid = true;
    dense_traj_points.push_back(std::move(traj_pt));
  }

  // Step 3) extends to max length if needed
  // double desired_length = planning_init_point.frenet_state.s + 1.0;

  // while (dense_traj_points.back().s < desired_length) {
  //   auto traj_pt = dense_traj_points.back();
  //   traj_pt.s += 0.2;
  //   traj_pt.t += config_.planning_result_delta_time;

  //   Point2D frenet_pt{traj_pt.s, traj_pt.l};
  //   Point2D cart_pt;
  //   if (!frenet_coord->SLToXY(frenet_pt, cart_pt)) {
  //     LOG_ERROR("ResultTrajectoryGenerator::execute, transform failed \n");
  //     return false;
  //   }

  //   traj_pt.x = cart_pt.x;
  //   traj_pt.y = cart_pt.y;
  //   dense_traj_points.emplace_back(std::move(traj_pt));
  // }

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

  return true;
}

bool ResultTrajectoryGenerator::RealtimeTrajectoryGenerator() {
  bool enable_lat_traj = config_.enable_lat_traj;

  auto &ego_planning_result =
      session_->mutable_planning_context()->mutable_planning_result();

  // Step 1) get x,y of trajectory points
  auto &traj_points = ego_planning_result.traj_points;
  auto &motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();

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
      std::fabs(motion_planner_output.ref_y_t_spline(0.0));
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
    traj_pt.x = motion_planner_output.x_t_spline(t) * (1.0 - alpha) +
                motion_planner_output.ref_x_t_spline(t) * alpha;

    traj_pt.y = motion_planner_output.y_t_spline(t) * (1.0 - alpha) +
                motion_planner_output.ref_y_t_spline(t) * alpha;

    traj_pt.v = motion_planner_output.v_t_spline(t);
    traj_pt.a = motion_planner_output.a_t_spline(t);
    traj_pt.heading_angle = motion_planner_output.theta_t_spline(t);
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

  return true;
}

}  // namespace planning
