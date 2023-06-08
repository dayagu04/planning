#include "tasks/trajectory_generator/result_trajectory_generator.h"

// #include "core/common/trace.h"
#include "common/define/geometry.h"
#include "common/math/math_utils.h"

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

void ResultTrajectoryGenerator::Init() {
  // auto &num_point = pipeline_context_->planning_result.traj_points.size();
  const size_t num_point = 26;
  t_vec_.resize(num_point);
  x_vec_.resize(num_point);
  y_vec_.resize(num_point);
  s_vec_.resize(num_point);
  l_vec_.resize(num_point);
  v_vec_.resize(num_point);
  a_vec_.resize(num_point);
  heading_angle_vec_.resize(num_point);
  curvature_vec_.resize(num_point);
  dkappa_vec_.resize(num_point);
  ddkappa_vec_.resize(num_point);
}

bool ResultTrajectoryGenerator::Execute(planning::framework::Frame *frame) {
  // NTRACE_CALL(7);

  if (Task::Execute(frame) == false) {
    return false;
  }

  auto &ego_planning_result = pipeline_context_->planning_result;
  // Step 1) get x,y of trajectory points
  auto &traj_points = ego_planning_result.traj_points;
  const auto &num_point = traj_points.size();
  pnc::mathlib::spline x_t_spline;
  pnc::mathlib::spline y_t_spline;
  pnc::mathlib::spline s_t_spline;
  pnc::mathlib::spline l_t_spline;
  pnc::mathlib::spline v_t_spline;
  pnc::mathlib::spline a_t_spline;
  pnc::mathlib::spline heading_angle_t_spline;
  pnc::mathlib::spline curvature_t_spline;
  pnc::mathlib::spline dkappa_t_spline;
  pnc::mathlib::spline ddkappa_t_spline;

  for (size_t i = 0; i < traj_points.size(); i++) {
    if (config_.is_pwj_planning) {
      Point2D frenet_pt{traj_points[i].s, traj_points[i].l};
      Point2D cart_pt;
      if (frenet_coord_->FrenetCoord2CartCoord(frenet_pt, cart_pt) !=
          TRANSFORM_SUCCESS) {
        LOG_ERROR("ResultTrajectoryGenerator::execute, transform failed \n");
        return false;
      }

      traj_points[i].x = cart_pt.x;
      traj_points[i].y = cart_pt.y;

      LOG_DEBUG("result traj_point s=%f, l=%f, x=%f, y=%f \n", traj_points[i].s,
                traj_points[i].l, traj_points[i].x, traj_points[i].y);
    }

    t_vec_[i] = traj_points[i].t;
    x_vec_[i] = traj_points[i].x;
    y_vec_[i] = traj_points[i].y;
    s_vec_[i] = traj_points[i].s;
    l_vec_[i] = traj_points[i].l;
    v_vec_[i] = traj_points[i].v;
    a_vec_[i] = traj_points[i].a;
    heading_angle_vec_[i] = traj_points[i].heading_angle;
    curvature_vec_[i] = traj_points[i].curvature;
    dkappa_vec_[i] = traj_points[i].dkappa;
    ddkappa_vec_[i] = traj_points[i].ddkappa;
  }

  x_t_spline.set_points(t_vec_, x_vec_);
  y_t_spline.set_points(t_vec_, y_vec_);
  s_t_spline.set_points(t_vec_, s_vec_);
  l_t_spline.set_points(t_vec_, l_vec_);
  v_t_spline.set_points(t_vec_, v_vec_);
  a_t_spline.set_points(t_vec_, a_vec_);
  heading_angle_t_spline.set_points(t_vec_, heading_angle_vec_);
  curvature_t_spline.set_points(t_vec_, curvature_vec_);
  dkappa_t_spline.set_points(t_vec_, dkappa_vec_);
  ddkappa_t_spline.set_points(t_vec_, ddkappa_vec_);
  // Step 2) get dense trajectory points
  auto &planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  double init_point_relative_time = planning_init_point.relative_time;

  std::vector<TrajectoryPoint> dense_traj_points;
  int dense_num_points =
      int(traj_points.back().t / config_.planning_result_delta_time) + 1;
  size_t i = 1;
  for (int j = 0; j < dense_num_points; ++j) {
    TrajectoryPoint traj_pt;
    double t = j * config_.planning_result_delta_time;
    traj_pt.x = x_t_spline(t);
    traj_pt.y = y_t_spline(t);

    traj_pt.t = init_point_relative_time + t;
    traj_pt.s = s_t_spline(t);
    traj_pt.x = x_t_spline(t);
    traj_pt.y = y_t_spline(t);
    traj_pt.v = v_t_spline(t);
    traj_pt.a = a_t_spline(t);
    traj_pt.l = l_t_spline(t);
    traj_pt.heading_angle = heading_angle_t_spline(t);
    traj_pt.curvature = curvature_t_spline(t);
    traj_pt.dkappa = dkappa_t_spline(t);
    traj_pt.ddkappa = ddkappa_t_spline(t);
    traj_pt.frenet_valid = true;
    dense_traj_points.push_back(std::move(traj_pt));
  }

  // Step 3) extends to max length if needed
  double desired_length =
      planning_init_point.frenet_state.s + config_.min_path_length;

  while (dense_traj_points.back().s < desired_length) {
    auto traj_pt = dense_traj_points.back();
    traj_pt.s += 0.2;
    traj_pt.t += config_.planning_result_delta_time;

    Point2D frenet_pt{traj_pt.s, traj_pt.l};
    Point2D cart_pt;
    if (frenet_coord_->FrenetCoord2CartCoord(frenet_pt, cart_pt) !=
        TRANSFORM_SUCCESS) {
      LOG_ERROR("ResultTrajectoryGenerator::execute, transform failed \n");
      return false;
    }

    traj_pt.x = cart_pt.x;
    traj_pt.y = cart_pt.y;
    dense_traj_points.push_back(std::move(traj_pt));
  }

  ego_planning_result.traj_points = dense_traj_points;

  // record some results
  ego_planning_result.traj_spline = frame_->mutable_session()
                                        ->mutable_planning_context()
                                        ->mutable_planning_result()
                                        .traj_spline;

  return true;
}

}  // namespace planning
