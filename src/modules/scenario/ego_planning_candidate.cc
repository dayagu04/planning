#include "ego_planning_candidate.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "debug_info_log.h"
#include "math/math_utils.h"
#include "math_lib.h"
#include "reference_path_manager.h"
#include "spline.h"
#include "spline_projection.h"
#include "task_pipeline_context.h"
namespace planning {
using namespace pnc;

EgoPlanningCandidate::EgoPlanningCandidate(planning::framework::Frame *frame) {
  frame_ = frame;
  result_trajectory_type_ = ResultTrajectoryType::REFINED_TRAJ;
  auto config_builder = frame->session()->environmental_model().config_builder(
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
      frame_->mutable_session()
          ->mutable_environmental_model()
          ->get_reference_path_manager()
          ->make_map_lane_reference_path(coarse_planning_info_.target_lane_id);

  // Step 3) calculate trajectory points
  // generate reference path
  static const double min_ego_v_cruise = 2.0;
  const auto &v_ref_cruise = std::fmax(frame_->session()
                                           ->environmental_model()
                                           .get_ego_state_manager()
                                           ->ego_v_cruise(),
                                       min_ego_v_cruise);
  // config_.reference_point_velocity ignored

  static const size_t &N = config_.num_point;
  const auto &delta_time = config_.delta_t;

  auto &cart_ref_info = coarse_planning_info_.cart_ref_info;
  const auto &frenet_coord =
      coarse_planning_info_.reference_path->get_frenet_coord();

  double s = 0.0;
  Point2D frenet_pt{s, 0.0};
  Point2D cart_pt(0.0, 0.0);

  auto point_size = coarse_planning_info_.reference_path->get_points().size();
  cart_ref_info.x_vec.resize(point_size);
  cart_ref_info.y_vec.resize(point_size);
  cart_ref_info.s_vec.resize(point_size);

  for (size_t i = 0; i < point_size; ++i) {
    cart_ref_info.x_vec[i] =
        coarse_planning_info_.reference_path->get_points().at(i).path_point.x;
    cart_ref_info.y_vec[i] =
        coarse_planning_info_.reference_path->get_points().at(i).path_point.y;
    cart_ref_info.s_vec[i] =
        coarse_planning_info_.reference_path->get_points().at(i).path_point.s;
  }

  cart_ref_info.x_s_spline.set_points(cart_ref_info.s_vec, cart_ref_info.x_vec);
  cart_ref_info.y_s_spline.set_points(cart_ref_info.s_vec, cart_ref_info.y_vec);

  JSON_DEBUG_VECTOR("raw_refline_x_vec", cart_ref_info.x_vec, 2)
  JSON_DEBUG_VECTOR("raw_refline_y_vec", cart_ref_info.y_vec, 2)

  const auto &planning_init_point =
      coarse_planning_info_.reference_path->get_frenet_ego_state()
          .planning_init_point();

  Eigen::Vector2d init_pos(planning_init_point.lat_init_state.x(),
                           planning_init_point.lat_init_state.y());
  const auto &ego_state =
      frame_->session()->environmental_model().get_ego_state_manager();

  JSON_DEBUG_VALUE("ego_pos_x", ego_state->ego_pose_raw().x)
  JSON_DEBUG_VALUE("ego_pos_y", ego_state->ego_pose_raw().y)
  JSON_DEBUG_VALUE("ego_pos_yaw", ego_state->ego_pose_raw().theta)
  JSON_DEBUG_VALUE("init_pos_x", init_pos.x())
  JSON_DEBUG_VALUE("init_pos_y", init_pos.y())

  // start from project s
  pnc::spline::Projection projection_spline;
  projection_spline.CalProjectionPoint(
      cart_ref_info.x_s_spline, cart_ref_info.y_s_spline,
      cart_ref_info.s_vec.front(), cart_ref_info.s_vec.back(), init_pos);

  double s_ref = projection_spline.GetOutput().s_proj;

  const auto &frenet_length = frenet_coord->GetLength();

  s_ref = std::min(s_ref, frenet_length * 0.95);
  const double delta_s = frenet_length - s_ref;
  const double v_cruise_scale = std::min(delta_s / (v_ref_cruise * 5.0), 1.0);
  frame_->mutable_session()->mutable_planning_context()->set_v_ref_cruise(
      v_cruise_scale * v_ref_cruise);

  coarse_planning_info_.trajectory_points.clear();
  TrajectoryPoint point;
  for (size_t i = 0; i < N; ++i) {
    // cart info
    if (s_ref < cart_ref_info.s_vec.back()) {
      point.x = cart_ref_info.x_s_spline(s_ref);
      point.y = cart_ref_info.y_s_spline(s_ref);
      point.heading_angle =
          std::atan2(cart_ref_info.y_s_spline.deriv(1, s_ref),
                     cart_ref_info.x_s_spline.deriv(1, s_ref));
    }

    // frenet info
    Point2D frenet_pt{0.0, 0.0};
    Point2D cart_pt(point.x, point.y);
    frenet_coord->CartCoord2FrenetCoord(cart_pt, frenet_pt);
    point.s = frenet_pt.x;
    point.l = frenet_pt.y;
    point.t = static_cast<double>(i) * delta_time;

    s_ref += v_cruise_scale * v_ref_cruise * delta_time;
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
  // bool ok = false;
  bool ok = task_pipeline->Run(*this);
  auto task_pipeline_context = task_pipeline->get_pipeline_context();
  // status_info_ = task_pipeline_context->status_info;
  if (ok) {
    planning_result_ = task_pipeline_context->planning_result;
    planning_result_.use_backup_cnt = 0;
    success_ = true;
  } else {
    LOG_ERROR("task_pipepine failed\n");
  }

  if (not ok and last_planning_result_ != nullptr and
      last_planning_result_->use_backup_cnt <= 5) {
    auto cur_time = frame_->mutable_session()
                        ->mutable_environmental_model()
                        ->get_ego_state_manager()
                        ->navi_timestamp();
    auto delta_time = cur_time - last_planning_result_->timestamp;
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
    traj_pt.x = planning_math::Interpolate(pre_pt.t, pre_pt.x, next_pt.t,
                                           next_pt.x, interpolate_t);
    traj_pt.y = planning_math::Interpolate(pre_pt.t, pre_pt.y, next_pt.t,
                                           next_pt.y, interpolate_t);
    traj_pt.heading_angle = planning_math::InterpolateAngle(
        pre_pt.t, pre_pt.heading_angle, next_pt.t, next_pt.heading_angle,
        interpolate_t);
    traj_pt.curvature =
        planning_math::Interpolate(pre_pt.t, pre_pt.curvature, next_pt.t,
                                   next_pt.curvature, interpolate_t);
    traj_pt.v = planning_math::Interpolate(pre_pt.t, pre_pt.v, next_pt.t,
                                           next_pt.v, interpolate_t);
    traj_pt.a = planning_math::Interpolate(pre_pt.t, pre_pt.a, next_pt.t,
                                           next_pt.a, interpolate_t);
    traj_pt.s = planning_math::Interpolate(pre_pt.t, pre_pt.s, next_pt.t,
                                           next_pt.s, interpolate_t);
    traj_pt.l = planning_math::Interpolate(pre_pt.t, pre_pt.l, next_pt.t,
                                           next_pt.l, interpolate_t);
    traj_pt.frenet_valid = pre_pt.frenet_valid and next_pt.frenet_valid;
    planning_result_.traj_points.emplace_back(traj_pt);
  }
  planning_result_.raw_traj_points.clear();
}

void EgoPlanningCandidate::copy_to_planning_context() {
  // assert(success_);

  // traj points and status info
  frame_->mutable_session()
      ->mutable_planning_context()
      ->mutable_planning_result() = planning_result_;
}

}  // namespace planning
