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

  std::vector<double> t_vec(num_point);
  std::vector<double> x_vec(num_point);
  std::vector<double> y_vec(num_point);
  std::vector<double> s_vec(num_point);
  std::vector<double> l_vec(num_point);
  std::vector<double> v_vec(num_point);
  std::vector<double> a_vec(num_point);
  std::vector<double> heading_angle_vec(num_point);
  std::vector<double> curvature_vec(num_point);
  std::vector<double> dkappa_vec(num_point);
  std::vector<double> ddkappa_vec(num_point);

  for (auto &traj_pt : traj_points) {
    if (config_.is_pwj_planning) {
      Point2D frenet_pt{traj_pt.s, traj_pt.l};
      Point2D cart_pt;
      if (frenet_coord_->FrenetCoord2CartCoord(frenet_pt, cart_pt) !=
          TRANSFORM_SUCCESS) {
        LOG_ERROR("ResultTrajectoryGenerator::execute, transform failed \n");
        return false;
      }

      traj_pt.x = cart_pt.x;
      traj_pt.y = cart_pt.y;

      LOG_DEBUG("result traj_point s=%f, l=%f, x=%f, y=%f \n", traj_pt.s,
                traj_pt.l, traj_pt.x, traj_pt.y);
    }

    t_vec.emplace_back(traj_pt.t);
    x_vec.emplace_back(traj_pt.x);
    y_vec.emplace_back(traj_pt.y);
    s_vec.emplace_back(traj_pt.s);
    l_vec.emplace_back(traj_pt.l);
    v_vec.emplace_back(traj_pt.v);
    a_vec.emplace_back(traj_pt.a);
    heading_angle_vec.emplace_back(traj_pt.heading_angle);
    curvature_vec.emplace_back(traj_pt.curvature);
    dkappa_vec.emplace_back(traj_pt.dkappa);
    ddkappa_vec.emplace_back(traj_pt.ddkappa);
  }

  x_t_spline.set_points(t_vec, x_vec);
  y_t_spline.set_points(t_vec, y_vec);
  s_t_spline.set_points(t_vec, s_vec);
  l_t_spline.set_points(t_vec, l_vec);
  v_t_spline.set_points(t_vec, v_vec);
  a_t_spline.set_points(t_vec, a_vec);
  heading_angle_t_spline.set_points(t_vec, heading_angle_vec);
  curvature_t_spline.set_points(t_vec, curvature_vec);
  dkappa_t_spline.set_points(t_vec, dkappa_vec);
  ddkappa_t_spline.set_points(t_vec, ddkappa_vec);
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
    traj_pt.s = s_t_spline(traj_pt.t);
    traj_pt.x = x_t_spline(traj_pt.t);
    traj_pt.y = y_t_spline(traj_pt.t);
    traj_pt.v = v_t_spline(traj_pt.t);
    traj_pt.a = a_t_spline(traj_pt.t);
    traj_pt.l = l_t_spline(traj_pt.t);
    traj_pt.heading_angle = heading_angle_t_spline(traj_pt.t);
    traj_pt.curvature = curvature_t_spline(traj_pt.t);
    traj_pt.dkappa = dkappa_t_spline(traj_pt.t);
    traj_pt.ddkappa = ddkappa_t_spline(traj_pt.t);
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
