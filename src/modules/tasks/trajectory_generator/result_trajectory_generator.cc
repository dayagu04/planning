#include "tasks/trajectory_generator/result_trajectory_generator.h"

// #include "core/common/trace.h"
#include "common/define/geometry.h"
#include "common/math/math_utils.h"

namespace planning {

using namespace std;
using namespace planning::planning_math;

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
  for (auto &traj_pt : traj_points) {
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
    while (i + 1 < traj_points.size() && traj_points[i].t < t) {
      ++i;
    }
    double ratio =
        (traj_points[i].t - t) / (traj_points[i].t - traj_points[i - 1].t);
    if (is_abnormal_number(ratio)) {
      LOG_ERROR("ResultTrajectoryGenerator::execute, ratio is abnormal \n");
      return false;
    }

    traj_pt.t = init_point_relative_time + t;
    traj_pt.s = Interpolate(traj_points[i - 1].s, traj_points[i].s, ratio);
    traj_pt.l = Interpolate(traj_points[i - 1].l, traj_points[i].l, ratio);
    traj_pt.v = Interpolate(traj_points[i - 1].v, traj_points[i].v, ratio);
    traj_pt.a = Interpolate(traj_points[i - 1].a, traj_points[i].a, ratio);
    traj_pt.x = Interpolate(traj_points[i - 1].x, traj_points[i].x, ratio);
    traj_pt.y = Interpolate(traj_points[i - 1].y, traj_points[i].y, ratio);
    traj_pt.heading_angle = InterpolateAngle(
        traj_points[i - 1].heading_angle, traj_points[i].heading_angle, ratio);
    traj_pt.curvature = Interpolate(traj_points[i - 1].curvature,
                                    traj_points[i].curvature, ratio);
    traj_pt.dkappa =
        Interpolate(traj_points[i - 1].dkappa, traj_points[i].dkappa, ratio);
    traj_pt.ddkappa =
        Interpolate(traj_points[i - 1].ddkappa, traj_points[i].ddkappa, ratio);
    traj_pt.frenet_valid = true;
    dense_traj_points.push_back(std::move(traj_pt));
  }

  // Step 3) extends to max length if needed
  double desired_length =
      planning_init_point.frenet_state.s + config_.min_path_length;

  // if (frame_->session()->is_parking_scene()) {
  //   auto dest = frame_->session()
  //                   ->environmental_model()
  //                   .get_virtual_lane_manager()
  //                   ->get_dest();
  //   auto destination_s = std::numeric_limits<double>::max();
  //   // auto distance_to_intersection = std::numeric_limits<double>::max();
  //   if (dest.isValid()) {
  //     auto distance_to_destination = dest.hasLanePathOffset()
  //                                        ? dest.getLanePathOffset()
  //                                        : dest.getRoadOffset();
  //     destination_s =
  //         planning_init_point.frenet_state.s + distance_to_destination;
  //   }
  // }

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

} // namespace planning
