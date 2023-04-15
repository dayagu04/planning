#include "modules/scenario/ego_planning_candidate.h"
#include <memory>
#include "modules/context/reference_path_manager.h"
#include "modules/common/math/math_utils.h"
#include "modules/tasks/task_pipeline_context.h"

namespace planning {

EgoPlanningCandidate::EgoPlanningCandidate(planning::framework::Frame *frame) {
  frame_ = frame;
  result_trajectory_type_ = ResultTrajectoryType::REFINED_TRAJ;
  auto config_builder =
      frame->session()->environmental_model().config_builder(
          frame_->session()->get_scene_type());
  config_ = config_builder->cast<EgoPlanningCandidateConfig>();
}

void EgoPlanningCandidate::set_coarse_planning_info(
    const StateTransitionContext &transition_context) {
  // Step 1) 计算state
  coarse_planning_info_.source_state = transition_context.source_state;
  coarse_planning_info_.target_state = transition_context.target_state;
  coarse_planning_info_.lane_change_request_source =
      transition_context.lane_change_request_source;
  coarse_planning_info_.source_lane_id = transition_context.source_lane_id;
  coarse_planning_info_.target_lane_id = transition_context.target_lane_id;
  coarse_planning_info_.bind_end_state = transition_context.bind_end_state;
  if (transition_context.target_state == ROAD_LC_LWAIT ||
      transition_context.target_state == ROAD_LC_RWAIT ||
      transition_context.target_state == ROAD_LC_LBACK ||
      transition_context.target_state == ROAD_LC_RBACK) {
    coarse_planning_info_.overtake_obstacles =
        transition_context.overtake_obstacles;
    coarse_planning_info_.yield_obstacles = transition_context.yield_obstacles;
  } else {
    coarse_planning_info_.overtake_obstacles.clear();
    coarse_planning_info_.yield_obstacles.clear();
  }

  // Step 2) build reference path
  frame_->mutable_session()
      ->mutable_planning_context()
      ->mutable_planning_result()
      .use_refined_reference_path = false;
  coarse_planning_info_.reference_path =
      make_map_lane_reference_path(frame_->mutable_session()
                                        ->mutable_environmental_model()
                                        ->get_reference_path_manager()
                                        .get(),
                                    coarse_planning_info_.target_lane_id);

  // Step 3) calculate trajectory points
  const auto &planning_init_point =
      coarse_planning_info_.reference_path->get_frenet_ego_state()
          .planning_init_point();
  LOG_DEBUG("planning_init_point: x = %f, y = %f,s = %f,l = %f",
        planning_init_point.x, planning_init_point.y,
        planning_init_point.frenet_state.s,
        planning_init_point.frenet_state.r);
  const auto &reference_path =
      coarse_planning_info_.reference_path->get_points();
  LOG_DEBUG("reference_path.size() = %d", reference_path.size());
  const auto &frenet_coord =
      coarse_planning_info_.reference_path->get_frenet_coord();

  TrajectoryPoint point;
  TrajectoryPoints trajectory_points;
  for (size_t i = 0; i < 21; i++) {
    point.v =
        i == 0 ? planning_init_point.v : config_.reference_point_velocity;
    point.s = planning_init_point.frenet_state.s + 0.2 * i * point.v;
    point.l = i == 0 ? planning_init_point.frenet_state.r : 0.;
    Point2D frenet_pt{point.s, point.l};
    Point2D cart_pt;
    if (frenet_coord != nullptr &&
        frenet_coord->FrenetCoord2CartCoord(frenet_pt, cart_pt) ==
            TRANSFORM_SUCCESS) {
      point.x = cart_pt.x;
      point.y = cart_pt.y;
    } else {
      LOG_DEBUG(
          "FrenetCoord2CartCoord = FAILED !!!!!!!! index: %d,  point.s : "
          "%f, point.l: %f ",
          i, point.s, point.l);
    }
    trajectory_points.emplace_back(point);
    coarse_planning_info_.trajectory_points.emplace_back(point);
  }
}

void EgoPlanningCandidate::set_last_planning_result(
    const std::shared_ptr<PlanningResult> &planning_result) {
  last_planning_result_ = planning_result;
}

bool EgoPlanningCandidate::pre_check() {
  // Step 1) check reference_path
  if (coarse_planning_info_.reference_path == nullptr) {
    LOG_DEBUG("reference_path is null");
    return false;
  }

  // Step 2) check init state when target_state is CRUISE_CHANGE
  if (coarse_planning_info_.target_state == ROAD_LC_LCHANGE ||
      coarse_planning_info_.target_state == ROAD_LC_RCHANGE) {
    auto &reference_path = coarse_planning_info_.reference_path;
    auto &frenet_ego_state = reference_path->get_frenet_ego_state();

    auto &ego_boundary = frenet_ego_state.boundary();
    std::vector<double> ego_s_list = {ego_boundary.s_start, ego_boundary.s_end,
                                      frenet_ego_state.s()};
    for (auto s : ego_s_list) {
      ReferencePathPoint refpath_pt{};
      (void)reference_path->get_reference_point_by_lon(s, refpath_pt);
      auto ego_l = frenet_ego_state.l();
      if (ego_l < -refpath_pt.distance_to_right_road_border or
          ego_l > refpath_pt.distance_to_left_road_border) {
        LOG_DEBUG("init_state error");
        return false;
      }
    }
  }

  return true;
}

void EgoPlanningCandidate::refine(
    const std::shared_ptr<TaskPipeline> &task_pipeline) {
  success_ = false;

  //  wait for task_pipeline
  bool ok = false;
  // bool ok = task_pipeline->run(*this);
  // auto task_pipeline_context = task_pipeline->get_pipeline_context();
  // status_info_ = task_pipeline_context->status_info;
  // if (ok) {
  //   planning_result_ = task_pipeline_context->planning_result;
  //   planning_result_.use_backup_cnt = 0;
  //   success_ = true;
  // } else {
  //   LOG_ERROR("task_pipepine failed");
  // }

  if (not ok and last_planning_result_ != nullptr and
      last_planning_result_->use_backup_cnt <= 5) {
    auto cur_time = frame_->mutable_session()
                        ->mutable_environmental_model()
                        ->get_ego_state_manager()
                        ->navi_timestamp();
    auto delta_time =
        cur_time - last_planning_result_->timestamp;
    if (0 < delta_time and delta_time < 1.0) {
      interpolate_with_last_trajectory_points();
      planning_result_.use_backup_cnt =
          last_planning_result_->use_backup_cnt + 1;
      success_ = true;
    }
  }

  if (success_) {
    planning_result_.timestamp = frame_->mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_ego_state_manager()
                                     ->navi_timestamp();
    planning_result_.target_lane_id = coarse_planning_info_.target_lane_id;
    planning_result_.target_scenario_state = coarse_planning_info_.target_state;
  }
}

void EgoPlanningCandidate::interpolate_with_last_trajectory_points() {
  assert(last_planning_result_ != nullptr);
  assert(last_planning_result_->target_lane_id ==
         coarse_planning_info_.target_lane_id);

  auto curr_time = frame_->mutable_session()
                       ->mutable_environmental_model()
                       ->get_ego_state_manager()
                       ->navi_timestamp();
  auto start_time = curr_time - last_planning_result_->timestamp;
  assert(start_time >= 0);

  // interpolate traj points
  // todo @xbliu config
  planning_result_.traj_points.clear();
  auto backup_num_points = 201;
  auto delta_time = 0.025;
  auto &last_traj_points = last_planning_result_->traj_points;
  assert(last_traj_points.size() >= 2);

  size_t idx = 0;
  for (int j = 0; j < backup_num_points; ++j) {
    TrajectoryPoint traj_pt;
    auto t = j * delta_time;
    auto interpolate_t = t + start_time;
    for (; idx < last_traj_points.size() - 1; idx++) {
      if (last_traj_points[idx].t <= interpolate_t and
          interpolate_t <= last_traj_points[idx + 1].t) {
        break;
      }
    }

    auto pre_idx = std::min(idx, last_traj_points.size() - 2);
    auto &pre_pt = last_traj_points[pre_idx];
    auto &next_pt = last_traj_points[pre_idx + 1];

    traj_pt.t = t;
    traj_pt.x =
        planning_math::interpolate(pre_pt.t, pre_pt.x, next_pt.t, next_pt.x, interpolate_t);
    traj_pt.y =
        planning_math::interpolate(pre_pt.t, pre_pt.y, next_pt.t, next_pt.y, interpolate_t);
    traj_pt.heading_angle =
        planning_math::InterpolateAngle(pre_pt.t, pre_pt.heading_angle, next_pt.t,
                          next_pt.heading_angle, interpolate_t);
    traj_pt.curvature = planning_math::interpolate(pre_pt.t, pre_pt.curvature, next_pt.t,
                                    next_pt.curvature, interpolate_t);
    traj_pt.v =
        planning_math::interpolate(pre_pt.t, pre_pt.v, next_pt.t, next_pt.v, interpolate_t);
    traj_pt.a =
        planning_math::interpolate(pre_pt.t, pre_pt.a, next_pt.t, next_pt.a, interpolate_t);
    traj_pt.s =
        planning_math::interpolate(pre_pt.t, pre_pt.s, next_pt.t, next_pt.s, interpolate_t);
    traj_pt.l =
        planning_math::interpolate(pre_pt.t, pre_pt.l, next_pt.t, next_pt.l, interpolate_t);
    traj_pt.frenet_valid = pre_pt.frenet_valid and next_pt.frenet_valid;
    planning_result_.traj_points.emplace_back(traj_pt);
  }
  planning_result_.raw_traj_points.clear();
}

void EgoPlanningCandidate::copy_to_planning_context() {
  assert(success_);

  // traj points and status info
  frame_->mutable_session()
      ->mutable_planning_context()
      ->mutable_planning_result() = planning_result_;
}

}  // namespace planning
