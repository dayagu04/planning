#include "tasks/trajectory_generator/result_trajectory_generator.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include "environmental_model.h"
#include "geometry_math.h"
#include "modules/tasks/task_interface/lane_change_decider_output.h"
#include "traffic_light_decision_manager.h"

// #include "core/common/trace.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "math/math_utils.h"
#include "math_lib.h"
#include "planning_context.h"
#include "vehicle_config_context.h"
namespace planning {

using namespace std;
using namespace planning::planning_math;
using namespace pnc::mathlib;
namespace {
static constexpr int kHmiSendMsgCntThreshold = 5;
}

ResultTrajectoryGenerator::ResultTrajectoryGenerator(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<ResultTrajectoryGeneratorConfig>();
  name_ = "ResultTrajectoryGenerator";
  Init();
}

void ResultTrajectoryGenerator::Init() {
  const int N = config_.trajectory_time_length / config_.planning_dt;
  t_vec_.resize(N + 1);
  s_vec_.resize(N + 1);
  l_vec_.resize(N + 1);
  curvature_vec_.resize(N + 1);
  dkappa_vec_.resize(N + 1);
  ddkappa_vec_.resize(N + 1);
}

bool ResultTrajectoryGenerator::Execute() {
  ILOG_DEBUG << "=======ResultTrajectoryGenerator=======";

  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }
  std::fill(t_vec_.begin(), t_vec_.end(), 0);
  std::fill(s_vec_.begin(), s_vec_.end(), 0);
  std::fill(l_vec_.begin(), l_vec_.end(), 0);
  std::fill(curvature_vec_.begin(), curvature_vec_.end(), 0);
  std::fill(dkappa_vec_.begin(), dkappa_vec_.end(), 0);
  std::fill(ddkappa_vec_.begin(), ddkappa_vec_.end(), 0);

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
  auto& ego_planning_result =
      session_->mutable_planning_context()->mutable_planning_result();


  // Step 1) get x,y of trajectory points
  auto& traj_points = ego_planning_result.traj_points;
  ego_planning_result.raw_traj_points = traj_points;
  std::copy(traj_points.begin(), traj_points.end(),
            ego_planning_result.raw_traj_points.begin());
  // const auto &num_point = traj_points.size();
  auto& motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();
  double curv_factor = motion_planner_output.curv_factor;
  pnc::mathlib::spline s_t_spline;
  pnc::mathlib::spline l_t_spline;

  pnc::mathlib::spline curvature_t_spline;
  pnc::mathlib::spline dkappa_t_spline;
  pnc::mathlib::spline ddkappa_t_spline;

  auto const N = traj_points.size();

  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto& frenet_coord = reference_path_ptr->get_frenet_coord();
  const double tp_init_s = traj_points.front().s;
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (config_.is_pwj_planning) {
      Point2D frenet_pt{traj_points[i].s, traj_points[i].l};
      Point2D cart_pt;
      if (!frenet_coord->SLToXY(frenet_pt, cart_pt)) {
        ILOG_ERROR << "ResultTrajectoryGenerator::execute, transform failed";
        return false;
      }

      traj_points[i].x = cart_pt.x;
      traj_points[i].y = cart_pt.y;

      ILOG_DEBUG << "result traj_point s=" << traj_points[i].s
                 << ", l=" << traj_points[i].l << ", x=" << traj_points[i].x
                 << ", y=" << traj_points[i].y;
    }
    // 默认高度类型：NORMAL，后续可在特定场景中覆盖
    traj_points[i].height_type = TrajectoryHeightType::NORMAL;
    t_vec_[i] = traj_points[i].t;
    s_vec_[i] = traj_points[i].s;
    l_vec_[i] = traj_points[i].l;
    curvature_vec_[i] = motion_planner_output.curv_s_spline(traj_points[i].s - tp_init_s);
    dkappa_vec_[i] = motion_planner_output.d_curv_s_spline(traj_points[i].s - tp_init_s);
    ddkappa_vec_[i] = traj_points[i].ddkappa;
  }
  // JSON_DEBUG_VECTOR("traj_s_vec", s_vec, 3)
  s_t_spline.set_points(t_vec_, s_vec_);
  l_t_spline.set_points(t_vec_, l_vec_);

  curvature_t_spline.set_points(t_vec_, curvature_vec_);
  dkappa_t_spline.set_points(t_vec_, dkappa_vec_);
  ddkappa_t_spline.set_points(t_vec_, ddkappa_vec_);


  // Step 2) get dense trajectory points

  // HPP：轨迹点为后轴中心，精度要求高，后轴/前轴压过减速带分开算，不合并区间
  // - 后轴压过减速带：后轴 s in [s_min, s_max]，前后各扩 margin
  // - 前轴压过减速带：前轴在 [s_min,s_max] 时后轴 s = 前轴s - wheel_base，前后各扩 margin
  static constexpr double kSpeedBumpExpandMargin = 0.1;
  std::vector<std::pair<double, double>> speed_bump_expanded_segments;
  if (session_->is_hpp_scene()) {
    const double wheel_base =
        VehicleConfigurationContext::Instance()->get_vehicle_param().wheel_base;
    const auto &segments =
        session_->planning_context().planning_result().speed_bump_path_segments;
    speed_bump_expanded_segments.reserve(segments.size() * 2);
    for (const auto &seg : segments) {
      // 后轴压过减速带：[s_min - margin, s_max + margin]
      speed_bump_expanded_segments.emplace_back(
          seg.s_min - kSpeedBumpExpandMargin,
          seg.s_max + kSpeedBumpExpandMargin);
      // 前轴压过减速带：后轴 s in [s_min - wheel_base - margin, s_max - wheel_base + margin]
      speed_bump_expanded_segments.emplace_back(
          seg.s_min - wheel_base - kSpeedBumpExpandMargin,
          seg.s_max - wheel_base + kSpeedBumpExpandMargin);
    }
  }

  std::vector<TrajectoryPoint> dense_traj_points;
  const double output_end_t =
      std::min(traj_points.back().t, config_.output_time_length);
  int dense_num_points =
      int(output_end_t / config_.planning_result_delta_time) + 1;

  auto is_in_speed_bump_range = [&speed_bump_expanded_segments](double s) {
    for (const auto &range : speed_bump_expanded_segments) {
      if (s >= range.first && s <= range.second) {
        return true;
      }
    }
    return false;
  };

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
    // traj_pt.curvature = curvature_t_spline(t);
    traj_pt.curvature = motion_planner_output.curv_s_spline(traj_pt.s - tp_init_s);
    traj_pt.dkappa = motion_planner_output.d_curv_s_spline(traj_pt.s - tp_init_s);
    // traj_pt.dkappa = dkappa_t_spline(t);
    traj_pt.ddkappa = ddkappa_t_spline(t);
    traj_pt.frenet_valid = true;
    // 默认 NORMAL；HPP 下若 s 落在减速带扩展区间内则标为 SPEED_BUMP
    traj_pt.height_type = TrajectoryHeightType::NORMAL;
    if (session_->is_hpp_scene() && is_in_speed_bump_range(traj_pt.s)) {
      traj_pt.height_type = TrajectoryHeightType::SPEED_BUMP;
    }
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
  //     return false;
  //   }

  //   traj_pt.x = cart_pt.x;
  //   traj_pt.y = cart_pt.y;
  //   dense_traj_points.emplace_back(std::move(traj_pt));
  // }

  // const auto N_ext = dense_traj_points.size();
  // std::vector<double> traj_x_vec(N_ext);
  // std::vector<double> traj_y_vec(N_ext);

  // for (size_t i = 0; i < N_ext; ++i) {
  //   traj_x_vec[i] = dense_traj_points[i].x;
  //   traj_y_vec[i] = dense_traj_points[i].y;
  // }

  // JSON_DEBUG_VECTOR("traj_x_vec", traj_x_vec, 3)
  // JSON_DEBUG_VECTOR("traj_y_vec", traj_y_vec, 3)

  ego_planning_result.traj_points = std::move(dense_traj_points);

  return true;
}

bool ResultTrajectoryGenerator::RealtimeTrajectoryGenerator() {
  bool enable_lat_traj = config_.enable_lat_traj;

  auto& ego_planning_result =
      session_->mutable_planning_context()->mutable_planning_result();

  // Step 1) get x,y of trajectory points
  auto& traj_points = ego_planning_result.traj_points;
  auto& motion_planner_output =
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
  const double output_end_t =
      std::min(traj_points.back().t, config_.output_time_length);
  size_t dense_num_points =
      int(output_end_t / config_.planning_result_delta_time) + 1;
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

  // JSON_DEBUG_VECTOR("traj_x_vec", traj_x_vec, 3)
  // JSON_DEBUG_VECTOR("traj_y_vec", traj_y_vec, 3)

  ego_planning_result.traj_points = dense_traj_points;

  return true;
}


}  // namespace planning
