#include "hybrid_astar_park_planner.h"

#include <cmath>
#include <cstddef>
#include <cstdio>

#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "hybrid_astar_response.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math/vec2d.h"
#include "math_utils.h"
#include "path_safe_checker.h"
#include "point_cloud_obstacle.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "transform2d.h"
#include "utils_math.h"

namespace planning {
namespace apa_planner {

HybridAStarParkPlanner::HybridAStarParkPlanner(
    const std::shared_ptr<ApaWorld>& apa_world_ptr) {
  SetApaWorldPtr(apa_world_ptr);

  Init();
}

void HybridAStarParkPlanner::Reset() {
  frame_.Reset();
  current_path_point_global_vec_.clear();

  const ApaParameters& params = apa_param.GetParam();
  if (params.path_generator_type == ParkPathGenerationType::SEARCH_BASED) {
    // init thread first
    if (!thread_.IsInit()) {
      thread_.Init(params.rear_overhanging, params.car_length, params.car_width,
                   params.steer_ratio, params.wheel_base,
                   params.min_turn_radius,
                   (params.max_car_width - params.car_width) * 0.5);
      thread_.Start();
    } else {
      thread_.Clear();
    }
  }

  current_gear_ = AstarPathGear::parking;
  in_slot_car_adjust_count_ = 0;
  is_path_single_shot_to_goal_ = false;

  return;
}

void HybridAStarParkPlanner::Init() {
  const ApaParameters& params = apa_param.GetParam();

  // todo, system should use same vehicle parameter configuration file and
  // data structure.
  ILOG_INFO << "init astar thread";

  current_gear_ = AstarPathGear::parking;
  in_slot_car_adjust_count_ = 0;
  is_path_single_shot_to_goal_ = false;

  thread_.Init(params.rear_overhanging, params.car_length, params.car_width,
               params.steer_ratio, params.wheel_base, params.min_turn_radius,
               (params.max_car_width - params.car_width) * 0.5);

  thread_.Start();

  return;
}

const bool HybridAStarParkPlanner::CheckSegCompleted() {
  bool is_seg_complete = false;
  if (frame_.spline_success) {
    if (frame_.remain_dist < apa_param.GetParam().max_replan_remain_dist &&
        apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag &&
        frame_.current_path_length > 1e-2) {
      if (frame_.stuck_uss_time > 0.068) {
        is_seg_complete = true;
      }
    }
  }

  return is_seg_complete;
}

const bool HybridAStarParkPlanner::CheckUssStucked() {
  if (frame_.remain_dist_uss < apa_param.GetParam().max_replan_remain_dist &&
      apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag) {
    if (frame_.stuck_uss_time >
        apa_param.GetParam().deadend_uss_stuck_replan_wait_time) {
      frame_.is_replan_by_uss = true;
      return true;
    }
  }

  return false;
}

const bool HybridAStarParkPlanner::CheckReplan() {
  if (frame_.is_replan_first == true) {
    frame_.replan_reason = FIRST_PLAN;
    return true;
  }

  frame_.is_replan_by_uss = false;
  frame_.is_replan_dynamic = false;

  if (CheckSegCompleted()) {
    frame_.replan_reason = SEG_COMPLETED_PATH;
    return true;
  }

  if (CheckUssStucked()) {
    frame_.replan_reason = SEG_COMPLETED_USS;
    return true;
  }

  if (frame_.stuck_uss_time > apa_param.GetParam().stuck_replan_time) {
    // if plan once, the stuck_uss_time is clear and accumlate again
    frame_.replan_reason = STUCKED;
    return true;
  }

  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool HybridAStarParkPlanner::CheckFinished() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  const bool lon_condition =
      ego_slot_info.terminal_err.pos.x() < apa_param.GetParam().finish_lon_err;

  const double lat_offset = ego_slot_info.ego_pos_slot.y();
  const double ego_head_lat_offset =
      (ego_slot_info.ego_pos_slot + (apa_param.GetParam().wheel_base +
                                     apa_param.GetParam().front_overhanging) *
                                        ego_slot_info.ego_heading_slot_vec)
          .y();

  const bool ego_center_lat_condition =
      std::fabs(lat_offset) <= apa_param.GetParam().finish_lat_err;

  const bool ego_head_lat_condition =
      std::fabs(lat_offset) <= apa_param.GetParam().finish_lat_err_strict &&
      std::fabs(ego_head_lat_offset) <=
          apa_param.GetParam().finish_lat_err_strict;

  const bool heading_condition_1 =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      apa_param.GetParam().finish_heading_err * kDeg2Rad;

  const bool heading_condition_2 =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      (apa_param.GetParam().finish_heading_err + 1.988) * kDeg2Rad;

  const bool lat_condition =
      (ego_center_lat_condition && heading_condition_1) &&
      (ego_head_lat_condition && heading_condition_2);

  const bool static_condition =
      apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag;

  const bool remain_s_condition =
      frame_.remain_dist < apa_param.GetParam().max_replan_remain_dist;

  bool parking_finish =
      lon_condition && lat_condition && static_condition && remain_s_condition;

  if (parking_finish) {
    return true;
  }

  // stucked by directly behind uss
  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();
  const bool enter_slot_condition =
      frame_.ego_slot_info.slot_occupied_ratio >
      apa_param.GetParam().finish_uss_slot_occupied_ratio;
  const bool remain_uss_condition =
      frame_.remain_dist_uss < apa_param.GetParam().max_replan_remain_dist;
  if (uss_obstacle_avoider_ptr->CheckIsDirectlyBehindUss()) {
    parking_finish = lat_condition && static_condition &&
                     enter_slot_condition && remain_uss_condition;
  }

  if (parking_finish) {
    return true;
  }

  parking_finish = lat_condition && static_condition && enter_slot_condition &&
                   (ego_slot_info.terminal_err.pos.x() < 0.568);

  return parking_finish;
}

void HybridAStarParkPlanner::PlanCore() {
  // prepare simulation
  InitSimulation();

  // check planning status
  if (!apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan &&
      CheckPlanSkip()) {
    return;
  }

  if (CheckPaused()) {
    SetParkingStatus(PARKING_PAUSED);
    if (frame_.pause_time > apa_param.GetParam().pause_failed_time) {
      SetParkingStatus(PARKING_FAILED);
      frame_.plan_fail_reason = PAUSE_FAILED_TIME;
    }
    return;
  }

  // update remain dist
  UpdateRemainDist();

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = UPDATE_EGO_SLOT_INFO;
    return;
  }

  PathShrinkBySlotLimiter();

  PathExpansionBySlotLimiter();

  // check finish
  if (CheckFinished()) {
    SetParkingStatus(PARKING_FINISHED);
    ILOG_INFO << "park finish";
    return;
  }

  // check failed
  if (CheckStuckFailed()) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = STUCK_FAILED_TIME;
    return;
  }

  bool is_replan = CheckReplan();

  if (!CheckEgoReplanNumber(is_replan)) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = PLAN_COUNT_EXCEED_LIMIT;
    return;
  }

  bool update_thread_path = UpdateThreadPath();
  PathPlannerResult path_plan_result = PathPlannerResult::PLAN_FAILED;

  ILOG_INFO << "stuck_uss_time = " << frame_.stuck_uss_time
            << ",is_replan = " << is_replan;

  // check replan
  if (apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan || is_replan ||
      update_thread_path) {
    ILOG_INFO << "plan reason = " << GetPlanReason(frame_.replan_reason)
              << ",force replan = "
              << apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan
              << ",thread update = " << update_thread_path
              << ",is_replan = " << is_replan;

    frame_.replan_flag = true;
    path_plan_result = PlanBySearchBasedMethod();
    frame_.pathplan_result = static_cast<uint8_t>(path_plan_result);

    switch (path_plan_result) {
      case PathPlannerResult::PLAN_UPDATE:
        if (PostProcessPath()) {
          SetParkingStatus(PARKING_PLANNING);
        } else {
          SetParkingStatus(PARKING_FAILED);
        }
        break;
      case PathPlannerResult::PLAN_FAILED:
        SetParkingStatus(PARKING_FAILED);
        break;
      default:
        SetParkingStatus(PARKING_PLANNING);
        break;
    }

  } else {
    SetParkingStatus(PARKING_RUNNING);
    HybridAstarDebugInfoClear();
    ILOG_INFO << "use history path";
  }

  // DebugPathString(current_path_point_global_vec_);

  return;
}

void HybridAStarParkPlanner::Log() const {
  const EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;
  const auto& l2g_tf = ego_slot_info.l2g_tf;

  JSON_DEBUG_VALUE("correct_path_for_limiter", frame_.correct_path_for_limiter)
  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag)

  std::vector<double> slot_corner_X;
  const size_t corner_size = ego_slot_info.slot_corner.size();
  slot_corner_X.clear();
  slot_corner_X.reserve(4 * corner_size);
  std::vector<double> slot_corner_Y;
  slot_corner_Y.clear();
  slot_corner_Y.reserve(4 * corner_size);
  std::vector<Eigen::Vector2d> pt_vec;
  pt_vec.clear();
  pt_vec.reserve(corner_size);
  for (const auto& corner : ego_slot_info.slot_corner) {
    slot_corner_X.emplace_back(corner.x());
    slot_corner_Y.emplace_back(corner.y());
    pt_vec.emplace_back(corner);
  }

  if (corner_size == 4) {
    slot_corner_X.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).x());
    slot_corner_Y.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).y());
    slot_corner_X.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).x());
    slot_corner_Y.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).y());
    const Eigen::Vector2d vec_01 = (pt_vec[1] - pt_vec[0]).normalized();
    const Eigen::Vector2d vec_23 = (pt_vec[3] - pt_vec[2]).normalized();
    pt_vec[0] = pt_vec[0] + ego_slot_info.move_slot_dist * vec_01;
    pt_vec[1] = pt_vec[1] + ego_slot_info.move_slot_dist * vec_01;
    pt_vec[2] = pt_vec[2] + ego_slot_info.move_slot_dist * vec_23;
    pt_vec[3] = pt_vec[3] + ego_slot_info.move_slot_dist * vec_23;
    for (const Eigen::Vector2d& pt : pt_vec) {
      slot_corner_X.emplace_back(pt.x());
      slot_corner_Y.emplace_back(pt.y());
    }
    slot_corner_X.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).x());
    slot_corner_Y.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).y());
    slot_corner_X.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).x());
    slot_corner_Y.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).y());
  }

  JSON_DEBUG_VECTOR("slot_corner_X", slot_corner_X, 6)
  JSON_DEBUG_VECTOR("slot_corner_Y", slot_corner_Y, 6)

  std::vector<double> limiter_corner_X;
  limiter_corner_X.clear();
  limiter_corner_X.reserve(frame_.ego_slot_info.limiter_corner.size());
  std::vector<double> limiter_corner_Y;
  limiter_corner_Y.clear();
  limiter_corner_Y.reserve(frame_.ego_slot_info.limiter_corner.size());
  for (const auto& corner : frame_.ego_slot_info.limiter_corner) {
    const auto tmp_corner = l2g_tf.GetPos(corner);
    limiter_corner_X.emplace_back(tmp_corner.x());
    limiter_corner_Y.emplace_back(tmp_corner.y());
  }
  JSON_DEBUG_VECTOR("limiter_corner_X", limiter_corner_X, 2)
  JSON_DEBUG_VECTOR("limiter_corner_Y", limiter_corner_Y, 2)

  JSON_DEBUG_VALUE("terminal_error_x",
                   frame_.ego_slot_info.terminal_err.pos.x())
  JSON_DEBUG_VALUE("terminal_error_y",
                   frame_.ego_slot_info.terminal_err.pos.y())
  JSON_DEBUG_VALUE("terminal_error_heading",
                   frame_.ego_slot_info.terminal_err.heading)

  ILOG_INFO << "lon error = " << frame_.ego_slot_info.terminal_err.pos.x()
            << ",lat error=" << frame_.ego_slot_info.terminal_err.pos.y()
            << ",heading error="
            << ifly_rad2deg(ego_slot_info.terminal_err.heading);

  JSON_DEBUG_VALUE("is_replan", frame_.is_replan)
  JSON_DEBUG_VALUE("is_finished", frame_.is_finished)
  JSON_DEBUG_VALUE("is_replan_first", frame_.is_replan_first)
  JSON_DEBUG_VALUE("is_replan_by_uss", frame_.is_replan_by_uss)
  JSON_DEBUG_VALUE("current_path_length", frame_.current_path_length)
  JSON_DEBUG_VALUE("gear_change_count", frame_.gear_change_count)
  JSON_DEBUG_VALUE("path_plan_success", frame_.plan_stm.path_plan_success)
  JSON_DEBUG_VALUE("planning_status", frame_.plan_stm.planning_status)
  JSON_DEBUG_VALUE("spline_success", frame_.spline_success)
  JSON_DEBUG_VALUE("remain_dist", frame_.remain_dist)
  JSON_DEBUG_VALUE("remain_dist_col_det", frame_.remain_dist_col_det)
  JSON_DEBUG_VALUE("remain_dist_uss", frame_.remain_dist_uss)
  JSON_DEBUG_VALUE("stuck_time", frame_.stuck_time)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
  JSON_DEBUG_VALUE("plan_fail_reason", frame_.plan_fail_reason)
  JSON_DEBUG_VALUE("dynamic_replan_count", frame_.dynamic_replan_count)
  JSON_DEBUG_VALUE("ego_heading_slot", frame_.ego_slot_info.ego_heading_slot)

  JSON_DEBUG_VALUE("selected_slot_id", frame_.ego_slot_info.selected_slot_id)
  JSON_DEBUG_VALUE("slot_length", frame_.ego_slot_info.slot_length)
  JSON_DEBUG_VALUE("slot_width", frame_.ego_slot_info.slot_width)

  JSON_DEBUG_VALUE("slot_origin_pos_x",
                   frame_.ego_slot_info.slot_origin_pos.x())

  JSON_DEBUG_VALUE("slot_origin_pos_y",
                   frame_.ego_slot_info.slot_origin_pos.y())

  JSON_DEBUG_VALUE("slot_origin_heading",
                   frame_.ego_slot_info.slot_origin_heading)

  JSON_DEBUG_VALUE("slot_occupied_ratio",
                   frame_.ego_slot_info.slot_occupied_ratio)

  std::vector<double> target_ego_pos_slot = {
      frame_.ego_slot_info.target_ego_pos_slot.x(),
      frame_.ego_slot_info.target_ego_pos_slot.y()};

  JSON_DEBUG_VALUE("pathplan_result", frame_.pathplan_result)
  JSON_DEBUG_VECTOR("target_ego_pos_slot", target_ego_pos_slot, 2)

  const auto uss_info =
      apa_world_ptr_->GetUssObstacleAvoidancePtr()->GetRemainDistInfo();
  JSON_DEBUG_VALUE("uss_available", uss_info.is_available)
  JSON_DEBUG_VALUE("uss_remain_dist", uss_info.remain_dist)
  JSON_DEBUG_VALUE("uss_index", uss_info.uss_index)
  JSON_DEBUG_VALUE("uss_car_index", uss_info.car_index)

  // lateral optimization
  const auto plan_debug_info =
      apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();

  if (plan_debug_info.has_terminal_pos_error()) {
    JSON_DEBUG_VALUE("optimization_terminal_pose_error",
                     plan_debug_info.terminal_pos_error())
    JSON_DEBUG_VALUE("optimization_terminal_heading_error",
                     plan_debug_info.terminal_heading_error())
  } else {
    JSON_DEBUG_VALUE("optimization_terminal_pose_error", 0.0)
    JSON_DEBUG_VALUE("optimization_terminal_heading_error", 0.0)
  }

  return;
}

void HybridAStarParkPlanner::GenTlane() { return; }

void HybridAStarParkPlanner::GenObstacles() { return; }

const uint8_t HybridAStarParkPlanner::PathPlanOnce() { return false; }

const std::string HybridAStarParkPlanner::GetPlanReason(const uint8_t type) {
  switch (type) {
    case 1:
      return "first_replan";
    case 2:
      return "SEG_COMPLETED_PATH";
    case 3:
      return "SEG_COMPLETED_USS";
    case 4:
      return "STUCKED";
    case 5:
      return "DYNAMIC";
    case 6:
      return "SEG_COMPLETED_COL_DET";
    default:
      return "NOT_REPLAN";
  }

  return "none";
}

void HybridAStarParkPlanner::ShrinkPathByFusionObj() {
  double path_checker_start_time = IflyTime::Now_ms();
  // init
  is_ego_collision_ = false;
  is_path_collision_ = false;
  path_collision_id_ = 1000000;

  // obs generate
  ParkObstacleList obs;
  PointCloudObstacleTransform obstacle_generator;
  const LocalView* local_view = apa_world_ptr_->GetLocalViewPtr();
  const MeasurementData* measures_ptr =
      &(apa_world_ptr_->GetApaDataPtr()->measurement_data);
  Pose2D ego_pose(measures_ptr->pos[0], measures_ptr->pos[1],
                  measures_ptr->heading);

  obstacle_generator.GenerateGlobalObstacle(obs, local_view);

  PathSafeChecker path_safe_checker;
  path_safe_checker.Excute(
      current_path_point_global_vec_,
      static_cast<pnc::geometry_lib::PathSegGear>(frame_.gear_command), &obs,
      ego_pose);

  frame_.remain_dist_col_det = frame_.remain_dist;
  if (path_safe_checker.IsPathCollision()) {
    path_safe_checker.UpdatePathValidDist(current_path_point_global_vec_,
                                          ego_pose);

    double valid_dist = path_safe_checker.GetPathValidDist();
    // for stop safely, add a lon buffer
    valid_dist -= 0.1;
    valid_dist = std::max(0.0, valid_dist);

    if (frame_.remain_dist_col_det > valid_dist) {
      frame_.remain_dist_col_det = valid_dist;
    }
    is_path_collision_ = true;

    // for debug
    path_collision_id_ = path_safe_checker.GetPathCollisionID();

    ILOG_INFO << "valid dist=" << valid_dist
              << ",path_collision_id_=" << path_collision_id_;
  }

  double path_checker_end_time = IflyTime::Now_ms();
  ILOG_INFO << "path checker time ms "
            << path_checker_end_time - path_checker_start_time;

  return;
}

void HybridAStarParkPlanner::UpdateRemainDist() {
  // 1. calculate remain dist according to plan path
  frame_.remain_dist = CalRemainDistFromPath();

  // 2.calculate remain dist uss according to uss
  frame_.remain_dist_uss = CalRemainDistFromUss();

  ILOG_INFO << "remain s = " << frame_.remain_dist
            << ", uss s = " << frame_.remain_dist_uss
            << ", obs s = " << frame_.remain_dist_col_det;

  return;
  ShrinkPathByFusionObj();

  return;
}

ApaPlannerBase::PathPlannerResult
HybridAStarParkPlanner::PlanBySearchBasedMethod() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  PathPlannerResult res = ApaPlannerBase::PathPlannerResult::WAIT_PATH;

  // start
  Pose2D start;
  start.x = ego_slot_info.ego_pos_slot[0];
  start.y = ego_slot_info.ego_pos_slot[1];
  start.theta = ego_slot_info.ego_heading_slot;

  // real target pose in slot
  Pose2D real_end;
  real_end.x = ego_slot_info.target_ego_pos_slot[0];
  real_end.y = ego_slot_info.target_ego_pos_slot[1];
  real_end.theta = ego_slot_info.target_ego_heading_slot;

  // astar end, maybe different with real end.
  Pose2D end = real_end;
  end.x = real_end.x + apa_param.GetParam().vertical_slot_target_adjust_dist;

  double astar_start_time = IflyTime::Now_ms();
  // obs generate
  ParkObstacleList obs;
  PointCloudObstacleTransform obstacle_generator;

  Pose2D slot_base_pose = Pose2D(ego_slot_info.slot_origin_pos.x(),
                                 ego_slot_info.slot_origin_pos.y(),
                                 ego_slot_info.slot_origin_heading);
  const LocalView* local_view = apa_world_ptr_->GetLocalViewPtr();

  // hack: delete obstacle around ego and slot. In the future, it will be
  // retired.
  obstacle_generator.GenerateLocalObstacle(
      obs, local_view, true, ego_slot_info.slot_length,
      ego_slot_info.slot_width, slot_base_pose, start, real_end);

  double search_start_time = IflyTime::Now_ms();
  ILOG_INFO << "fusion obj time ms " << search_start_time - astar_start_time;

  // set input
  AstarRequest cur_request;
  cur_request.path_generate_method =
      planning::AstarPathGenerateType::ASTAR_SEARCHING;
  cur_request.first_action_request.has_request = false;
  cur_request.space_type = ParkSpaceType::vertical;
  cur_request.parking_task = ParkingTask::parking_in;
  cur_request.head_request = ParkingVehDirectionRequest::tail_in_first;
  cur_request.rs_request = RSPathRequestType::none;
  cur_request.timestamp_ms = astar_start_time;

  cur_request.start_ = start;
  cur_request.base_pose_ = slot_base_pose;
  cur_request.real_goal = real_end;

  cur_request.vertical_slot_target_adjust_dist_ =
      apa_param.GetParam().vertical_slot_target_adjust_dist;
  cur_request.slot_width = ego_slot_info.slot_width;
  cur_request.slot_length = ego_slot_info.slot_length;
  cur_request.history_gear = current_gear_;

  switch (frame_.replan_reason) {
    case FIRST_PLAN:
      cur_request.plan_reason = PlanningReason::FIRST_PLAN;
      break;
    case SEG_COMPLETED_PATH:
      cur_request.plan_reason = PlanningReason::PATH_COMPLETED;
      break;
    case SEG_COMPLETED_USS:
      cur_request.plan_reason = PlanningReason::PATH_STUCKED;
      break;
    case STUCKED:
      cur_request.plan_reason = PlanningReason::PATH_STUCKED;
      break;
    case DYNAMIC:
      cur_request.plan_reason = PlanningReason::ADJUST_SELF_CAR_POSE;
      break;
    case SEG_COMPLETED_COL_DET:
      cur_request.plan_reason = PlanningReason::PATH_STUCKED;
      break;
    default:
      cur_request.plan_reason = PlanningReason::NONE;
      break;
  }

  is_path_single_shot_to_goal_ = false;

  // generate request
  bool need_drive_forward = IsEgoNeedDriveForwardInSlot(
      start, ego_slot_info.slot_width, ego_slot_info.slot_length);
  if (need_drive_forward) {
    cur_request.path_generate_method =
        planning::AstarPathGenerateType::REEDS_SHEPP;

    end.y = real_end.y;
    end.x = start.x + 30.0;
    cur_request.goal_ = end;

  } else {
    // gear need be different with history in next replanning
    if (frame_.replan_reason != 1) {
      cur_request.first_action_request.has_request = true;

      switch (current_gear_) {
        case AstarPathGear::reverse:
          cur_request.first_action_request.gear_request = AstarPathGear::drive;
          break;
        case AstarPathGear::drive:
          cur_request.first_action_request.gear_request =
              AstarPathGear::reverse;
          break;
        default:
          cur_request.first_action_request.has_request = false;
          break;
      }
    }

    cur_request.goal_ = end;
  }

  // search state
  AstarResponse response;

  // check result
  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    // get output
    thread_.PublishResponse(&response);
    ILOG_INFO << "publish path";

    bool is_nice = false;

    is_nice = IsResponseNice(cur_request, response);
    if (!is_nice) {
      thread_.Clear();
      thread_state_ = RequestResponseState::NONE;
    }
  }

  // publish result
  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    Transform2d response_tf;
    response_tf.SetBasePose(response.request.base_pose_);

    // success
    if (response.first_seg_path.size() >= 5) {
      std::vector<pnc::geometry_lib::PathPoint> local_path;
      size_t i;
      for (i = 0; i < response.first_seg_path.size(); i++) {
        local_path.emplace_back(pnc::geometry_lib::PathPoint(
            Eigen::Vector2d(response.first_seg_path[i].x,
                            response.first_seg_path[i].y),
            response.first_seg_path[i].phi, response.first_seg_path[i].kappa));
      }

      // todo:
      if (frame_.is_replan_first) {
        frame_.is_replan_first = false;
      }

      double search_end_time = IflyTime::Now_ms();

      // check path is single shot to goal.
      if (response.result.gear_change_num > 0) {
        is_path_single_shot_to_goal_ = false;
      } else {
        is_path_single_shot_to_goal_ = true;
      }

      double path_dist = 0.0;
      path_dist = response.first_seg_path.back().accumulated_s;
      ILOG_INFO << "first path gear = "
                << PathGearDebugString(response.first_seg_path[0].gear)
                << ",dist = " << path_dist
                << ",gear_change_num=" << response.result.gear_change_num;

      PathOptimizationByCILRQ(path_dist, local_path, &response_tf);

      double lqr_end_time = IflyTime::Now_ms();
      ILOG_INFO << "lqr time ms " << lqr_end_time - search_end_time;

      PublishHybridAstarDebugInfo(response.result, &thread_, &response_tf);

      double publish_end_time = IflyTime::Now_ms();
      ILOG_INFO << "publish time ms " << publish_end_time - lqr_end_time;

      res = PathPlannerResult::PLAN_UPDATE;
    } else {
      res = PathPlannerResult::WAIT_PATH;

      // publish fallback path
      GenerateFallBackPath();

      ILOG_INFO << "path plan point less 5";
    }

    // update gear
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (response.first_seg_path.size() > 0) {
      if (response.first_seg_path[0].gear == AstarPathGear::drive) {
        frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
      }
      current_gear_ = response.first_seg_path[0].gear;

      thread_.Clear();
      ILOG_INFO << "clear thread";
    }

    frame_.gear_command = frame_.current_gear;
    ILOG_INFO << "first path gear = " << static_cast<int>(frame_.gear_command);
  } else if (thread_state_ == RequestResponseState::NONE ||
             thread_state_ == RequestResponseState::HAS_PUBLISHED_RESPONSE) {
    // send request
    thread_.SetRequest(obs, cur_request);
    res = PathPlannerResult::WAIT_PATH;

    // publish fallback path
    GenerateFallBackPath();

    ILOG_INFO << "set input";

    frame_.total_plan_count++;
    if (current_gear_ == AstarPathGear::reverse &&
        frame_.ego_slot_info.slot_occupied_ratio > 0.2) {
      in_slot_car_adjust_count_++;
    }

  } else if (thread_state_ == RequestResponseState::HAS_REQUEST) {
    res = PathPlannerResult::WAIT_PATH;

    // publish fallback path
    GenerateFallBackPath();
    HybridAstarDebugInfoClear();

    ILOG_INFO << "has input";
  }

  // DebugPathString(current_path_point_global_vec_);

  return res;
}

const int HybridAStarParkPlanner::PublishHybridAstarDebugInfo(
    const HybridAStarResult& result, HybridAStarThreadSolver* thread,
    Transform2d* tf) {
  if (result.x.size() < 1) {
    ILOG_INFO << "no path";
    return 0;
  }

  size_t i;
  Pose2D local_position;
  Pose2D global_position;

  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();

  debug_->mutable_refline_info()->Clear();

  for (i = 0; i < result.x.size(); i++) {
    local_position.x = result.x[i];
    local_position.y = result.y[i];
    local_position.theta = result.phi[i];

    tf->ULFLocalPoseToGlobal(&global_position, local_position);

    planning::common::TrajectoryPoint* point = debug_->add_refline_info();

    point->set_x(global_position.x);
    point->set_y(global_position.y);
    point->set_heading_angle(global_position.theta);
    point->set_s(result.accumulated_s[i]);

    // todo, add hybrid astar msg. but now reuse TrajectoryPoint.
    if (result.type[i] == AstarPathType::REEDS_SHEPP) {
      point->set_l(-1.0);
    } else {
      point->set_l(1.0);
    }
  }

  planning::common::AstarNodeList* list = debug_->mutable_node_list();
  list->Clear();

  thread->GetNodeListMessagePublish(list);
  ILOG_INFO << "list size " << list->nodes_size();

  for (int i = 0; i < list->nodes_size(); i++) {
    for (int j = 0; j < list->nodes(i).path_point_size(); j++) {
      local_position.x = list->nodes(i).path_point(j).x();
      local_position.y = list->nodes(i).path_point(j).y();

      tf->ULFLocalPoseToGlobal(&global_position, local_position);

      list->mutable_nodes(i)->mutable_path_point(j)->set_x(global_position.x);
      list->mutable_nodes(i)->mutable_path_point(j)->set_y(global_position.y);
    }
  }

  return 0;
}

const int HybridAStarParkPlanner::HybridAstarDebugInfoClear() {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();

  // debug_->mutable_refline_info()->Clear();

  planning::common::AstarNodeList* list = debug_->mutable_node_list();
  list->Clear();

  return 0;
}

const int HybridAStarParkPlanner::GenerateFallBackPath() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  if (frame_.replan_reason == 5) {
    return 0;
  }

  current_path_point_global_vec_.clear();

  pnc::geometry_lib::PathPoint global_point;
  global_point.Set(
      ego_slot_info.l2g_tf.GetPos(ego_slot_info.ego_pos_slot),
      ego_slot_info.l2g_tf.GetHeading(ego_slot_info.ego_heading_slot));
  global_point.kappa = 0.0;

  // todo, use one point
  current_path_point_global_vec_.emplace_back(global_point);

  return 0;
}

const int HybridAStarParkPlanner::PathOptimizationByCILRQ(
    const double paht_s,
    const std::vector<pnc::geometry_lib::PathPoint>& local_path,
    Transform2d* tf) {
  LocalPathToGlobal(local_path, tf);
  ILOG_INFO << "output path by coarse a star path";

  return 0;
}

const int HybridAStarParkPlanner::LocalPathToGlobal(
    const std::vector<pnc::geometry_lib::PathPoint>& local_path,
    Transform2d* tf) {
  // TODO: longitudinal path optimization
  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(local_path.size());

  pnc::geometry_lib::PathPoint global_point;
  Pose2D global;
  for (const auto& path_point : local_path) {
    tf->ULFLocalPoseToGlobal(
        &global,
        Pose2D(path_point.pos.x(), path_point.pos.y(), path_point.heading));

    global_point.Set(Eigen::Vector2d(global.x, global.y), global.theta);
    global_point.kappa = path_point.kappa;

    current_path_point_global_vec_.emplace_back(global_point);
  }

  return 0;
}

const bool HybridAStarParkPlanner::UpdateThreadPath() {
  thread_.GetThreadState(&thread_state_);

  ILOG_INFO << "thread state " << static_cast<int>(thread_state_);

  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    ILOG_INFO << "fetch path";

    return true;
  }

  return false;
}

const bool HybridAStarParkPlanner::UpdateEgoSlotInfo() {
  const auto* measures_ptr = &apa_world_ptr_->GetApaDataPtr()->measurement_data;
  const auto slot_manager_ptr_ = apa_world_ptr_->GetSlotManagerPtr();

  frame_.correct_path_for_limiter = false;
  frame_.replan_flag = false;

  EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;

  ego_slot_info.target_managed_slot =
      slot_manager_ptr_->GetEgoSlotInfo().select_slot_filter;

  const auto& slot_points =
      ego_slot_info.target_managed_slot.corner_points().corner_point();
  std::vector<Eigen::Vector2d> pt;
  pt.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    if (apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
        apa_world_ptr_->GetApaDataPtr()
                ->simu_param.target_managed_slot_x_vec.size() == 4 &&
        apa_world_ptr_->GetApaDataPtr()->simu_param.use_slot_in_bag) {
      pt[i] << apa_world_ptr_->GetApaDataPtr()
                   ->simu_param.target_managed_slot_x_vec[i],
          apa_world_ptr_->GetApaDataPtr()
              ->simu_param.target_managed_slot_y_vec[i];
    } else {
      pt[i] << slot_points[i].x(), slot_points[i].y();
    }
  }
  const Eigen::Vector2d pM01 = 0.5 * (pt[0] + pt[1]);
  const Eigen::Vector2d pM23 = 0.5 * (pt[2] + pt[3]);
  const double real_slot_length = (pM01 - pM23).norm();
  const Eigen::Vector2d t = (pt[1] - pt[0]).normalized();
  // n is vec that slot opening orientation
  const Eigen::Vector2d n = Eigen::Vector2d(t.y(), -t.x());
  // const Eigen::Vector2d n = (pM01 - pM23).normalized();
  pt[2] = pt[0] - real_slot_length * n;
  pt[3] = pt[1] - real_slot_length * n;

  ego_slot_info.slot_corner = pt;

  ego_slot_info.slot_center = (pt[0] + pt[1] + pt[2] + pt[3]) * 0.25;

  // const double virtual_slot_length =
  //     apa_param.GetParam().car_length +
  //     apa_param.GetParam().slot_compare_to_car_length;

  // const double use_slot_length =
  //     std::min(real_slot_length, virtual_slot_length);

  const double use_slot_length = real_slot_length;

  ego_slot_info.slot_origin_pos = pM01 - use_slot_length * n;
  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;
  ego_slot_info.slot_length = use_slot_length;
  ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  if (ego_slot_info.target_managed_slot.slot_type() ==
      Common::PARKING_SLOT_TYPE_SLANTING) {
    const Eigen::Vector2d origin_pt_0 =
        Eigen::Vector2d(slot_manager_ptr_->GetEgoSlotInfo()
                            .select_fusion_slot.corner_points[0]
                            .x,
                        slot_manager_ptr_->GetEgoSlotInfo()
                            .select_fusion_slot.corner_points[0]
                            .y);

    const Eigen::Vector2d origin_pt_1 =
        Eigen::Vector2d(slot_manager_ptr_->GetEgoSlotInfo()
                            .select_fusion_slot.corner_points[1]
                            .x,
                        slot_manager_ptr_->GetEgoSlotInfo()
                            .select_fusion_slot.corner_points[1]
                            .y);

    ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(origin_pt_0);
    ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(origin_pt_1);

    if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
      std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
    }

    const Eigen::Vector2d pt_01_vec = ego_slot_info.pt_1 - ego_slot_info.pt_0;

    double angle = std::fabs(pnc::geometry_lib::GetAngleFromTwoVec(
                       Eigen::Vector2d(real_slot_length, 0.0), pt_01_vec)) *
                   kRad2Deg;

    if (angle > 90.0) {
      angle = 180.0 - angle;
    }

    angle = pnc::mathlib::DoubleConstrain(angle, 10.0, 80.0);
    ego_slot_info.sin_angle = std::sin(angle * kDeg2Rad);
    ego_slot_info.origin_pt_0_heading = 90.0 - angle;
  } else {
    ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(pt[0]);
    ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(pt[1]);
    if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
      std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
    }
    ego_slot_info.sin_angle = 1.0;
    ego_slot_info.origin_pt_0_heading = 0.0;
  }

  ego_slot_info.ego_pos_slot = ego_slot_info.g2l_tf.GetPos(measures_ptr->pos);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measures_ptr->heading);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  ego_slot_info.fus_obj_valid_flag =
      slot_manager_ptr_->GetEgoSlotInfo().fus_obj_valid_flag;
  ego_slot_info.obs_pt_vec_slot.clear();
  ego_slot_info.obs_pt_vec_slot.reserve(
      slot_manager_ptr_->GetEgoSlotInfo().obs_pt_vec_slot.size());

  for (const Eigen::Vector2d& obs_pt :
       slot_manager_ptr_->GetEgoSlotInfo().obs_pt_vec_slot) {
    const Eigen::Vector2d obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
    ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
  }
  // ego_slot_info.obs_pt_vec_slot =
  //     slot_manager_ptr_->GetEgoSlotInfo().obs_pt_vec_slot;

  ego_slot_info.limiter = slot_manager_ptr_->GetEgoSlotInfo().limiter;

  if (apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
      apa_world_ptr_->GetApaDataPtr()
              ->simu_param.target_managed_limiter_x_vec.size() == 2 &&
      apa_world_ptr_->GetApaDataPtr()->simu_param.use_slot_in_bag) {
    ego_slot_info.limiter.first
        << apa_world_ptr_->GetApaDataPtr()
               ->simu_param.target_managed_limiter_x_vec[0],
        apa_world_ptr_->GetApaDataPtr()
            ->simu_param.target_managed_limiter_y_vec[0];
    ego_slot_info.limiter.first =
        ego_slot_info.g2l_tf.GetPos(ego_slot_info.limiter.first);
    ego_slot_info.limiter.second
        << apa_world_ptr_->GetApaDataPtr()
               ->simu_param.target_managed_limiter_x_vec[1],
        apa_world_ptr_->GetApaDataPtr()
            ->simu_param.target_managed_limiter_y_vec[1];
    ego_slot_info.limiter.second =
        ego_slot_info.g2l_tf.GetPos(ego_slot_info.limiter.second);
  }

  ego_slot_info.limiter_corner.clear();
  ego_slot_info.limiter_corner.reserve(2);
  ego_slot_info.limiter_corner.emplace_back(ego_slot_info.limiter.first);
  ego_slot_info.limiter_corner.emplace_back(ego_slot_info.limiter.second);

  // cal target pos
  ego_slot_info.target_ego_pos_slot
      << (ego_slot_info.limiter.first.x() + ego_slot_info.limiter.second.x()) /
             2.0,
      apa_param.GetParam().terminal_target_y;
  ego_slot_info.target_ego_heading_slot =
      apa_param.GetParam().terminal_target_heading;

  ILOG_INFO << "limiter x=" << ego_slot_info.target_ego_pos_slot[0]
            << ",y=" << ego_slot_info.target_ego_pos_slot[1];

  // cal terminal error
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      ego_slot_info.ego_heading_slot - ego_slot_info.target_ego_heading_slot);

  // cal slot occupied ratio
  if (std::fabs(ego_slot_info.terminal_err.pos.y()) <
          apa_param.GetParam().slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_slot_info.ego_heading_slot) <
          apa_param.GetParam().slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    const std::vector<double> x_bound = {
        ego_slot_info.target_ego_pos_slot.x(),
        ego_slot_info.slot_length + apa_param.GetParam().rear_overhanging};

    const std::vector<double> occupied_ratio_bound = {1.0, 0.0};
    ego_slot_info.slot_occupied_ratio = pnc::mathlib::Interp1(
        x_bound, occupied_ratio_bound, ego_slot_info.ego_pos_slot.x());
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  // calc slot side and init gear and init steer at first
  if (frame_.is_replan_first) {
    frame_.car_already_move_dist = 0.0;

    const auto pM01 =
        0.5 * (ego_slot_info.slot_corner[0] + ego_slot_info.slot_corner[1]);
    const auto pM23 =
        0.5 * (ego_slot_info.slot_corner[2] + ego_slot_info.slot_corner[3]);
    Eigen::Vector2d ego_to_slot_center_vec =
        0.5 * (pM01 + pM23) - measures_ptr->pos;

    const double cross_ego_to_slot_center =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->heading_vec,
                                                ego_to_slot_center_vec);

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(
            measures_ptr->heading_vec, ego_slot_info.slot_origin_heading_vec);

    // judge slot side via slot center and heading
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    }
  }

  // update stuck by uss time
  // 只要车静止不动，这个值一直在更新，需要检查超声波的距离？
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      measures_ptr->static_flag && !measures_ptr->brake_flag &&
      apa_world_ptr_->GetApaDataPtr()->cur_state ==
          ApaStateMachine::ACTIVE_IN) {
    frame_.stuck_uss_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_uss_time = 0.0;
  }

  // update stuck time
  // 车静止不动，这个值一直在更新
  if ((frame_.plan_stm.planning_status == PARKING_RUNNING ||
       frame_.plan_stm.planning_status == PARKING_PLANNING) &&
      measures_ptr->static_flag && !measures_ptr->brake_flag &&
      apa_world_ptr_->GetApaDataPtr()->cur_state ==
          ApaStateMachine::ACTIVE_IN) {
    frame_.stuck_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_time = 0.0;
  }

  // update pause time
  if (frame_.plan_stm.planning_status == PARKING_PAUSED) {
    frame_.pause_time += apa_param.GetParam().plan_time;
  } else {
    frame_.pause_time = 0.0;
  }

  // fix slot
  if (ego_slot_info.slot_occupied_ratio >
          apa_param.GetParam().fix_slot_occupied_ratio &&
      !frame_.is_fix_slot && measures_ptr->static_flag) {
    frame_.is_fix_slot = true;
    ego_slot_info.fix_limiter = true;
  }

  return true;
}

HybridAStarParkPlanner::~HybridAStarParkPlanner() {}

const bool HybridAStarParkPlanner::CheckStuckFailed() {
  return frame_.stuck_time > 12.0;
}

const double HybridAStarParkPlanner::CalRemainDistFromUss() {
  double remain_dist = 10.0;
  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();

  uss_obstacle_avoider_ptr->Update(&planning_output_,
                                   apa_world_ptr_->GetApaDataPtr());

  // hack: use uss circle dist. In the future, will use uss point cloud to do
  // safe check.
  const double safe_uss_remain_dist = 0.31;
  remain_dist = uss_obstacle_avoider_ptr->GetRemainDistInfo().remain_dist -
                safe_uss_remain_dist;

  return remain_dist;
}

void HybridAStarParkPlanner::PathShrinkBySlotLimiter() {
  if (current_gear_ != AstarPathGear::reverse) {
    return;
  }

  if (current_path_point_global_vec_.size() <= 1) {
    return;
  }

  EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;

  double limiter_x = ego_slot_info.target_ego_pos_slot[0];

  Eigen::Vector2d path_end_global = current_path_point_global_vec_.back().pos;
  Eigen::Vector2d point_local;
  point_local = ego_slot_info.g2l_tf.GetPos(path_end_global);

  ILOG_INFO << "x = " << limiter_x << ", end x = " << point_local[0]
            << ", y=" << point_local[1];

  if (point_local[0] >= limiter_x) {
    return;
  }

  double x_diff = std::fabs(ego_slot_info.target_ego_pos_slot[0] -
                            ego_slot_info.ego_pos_slot[0]);
  double y_diff = std::fabs(ego_slot_info.target_ego_pos_slot[1] -
                            ego_slot_info.ego_pos_slot[1]);
  if (x_diff > 1.5 || y_diff > 0.5) {
    return;
  }

  // shrink path
  size_t path_size = current_path_point_global_vec_.size();
  for (size_t i = 0; i < path_size; i++) {
    Eigen::Vector2d& point_global = current_path_point_global_vec_.back().pos;

    point_local = ego_slot_info.g2l_tf.GetPos(point_global);

    if (point_local[0] >= limiter_x) {
      break;
    }

    current_path_point_global_vec_.pop_back();
  }

  return;
}

void HybridAStarParkPlanner::PathExpansionBySlotLimiter() {
  if (current_gear_ != AstarPathGear::reverse) {
    return;
  }

  if (current_path_point_global_vec_.size() <= 2) {
    return;
  }

  if (!is_path_single_shot_to_goal_) {
    return;
  }

  EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;

  double limiter_x = ego_slot_info.target_ego_pos_slot[0];

  Eigen::Vector2d path_end_global = current_path_point_global_vec_.back().pos;
  Eigen::Vector2d point_local;
  point_local = ego_slot_info.g2l_tf.GetPos(path_end_global);

  if (point_local[0] <= (limiter_x + 0.1)) {
    return;
  }

  double ego_x_diff = std::fabs(ego_slot_info.target_ego_pos_slot[0] -
                                ego_slot_info.ego_pos_slot[0]);
  double ego_y_diff = std::fabs(ego_slot_info.target_ego_pos_slot[1] -
                            ego_slot_info.ego_pos_slot[1]);
  if (ego_x_diff > 1.5 || ego_y_diff > 0.5) {
    return;
  }

  double x_diff =
      std::fabs(point_local[0] - ego_slot_info.target_ego_pos_slot[0]);
  double y_diff =
      std::fabs(point_local[1] - ego_slot_info.target_ego_pos_slot[1]);
  double length = std::sqrt(x_diff * x_diff + y_diff * y_diff);
  length = std::min(length, 5.0);

  if (x_diff > 1.0 || y_diff > 0.5) {
    return;
  }

  ILOG_INFO << "x = " << limiter_x << ", end x = " << point_local[0]
            << ", y=" << point_local[1];

  double phi = current_path_point_global_vec_.back().heading;

  size_t path_point_size = current_path_point_global_vec_.size();

  Eigen::Vector2d the_last_but_one =
      current_path_point_global_vec_[path_point_size - 2].pos;

  Eigen::Vector2d unit_line_vec =
      Eigen::Vector2d(path_end_global[0] - the_last_but_one[0],
                      path_end_global[1] - the_last_but_one[1]);

  if (unit_line_vec.norm() < 0.01) {
    return;
  }
  unit_line_vec.normalize();

  double s = 0.1;
  double ds = 0.1;

  Eigen::Vector2d point;
  pnc::geometry_lib::PathPoint global_point;
  while (s < length) {
    point = path_end_global + s * unit_line_vec;

    global_point.Set(point, phi);
    global_point.kappa = 0.0;

    current_path_point_global_vec_.push_back(global_point);

    s += ds;
  }

  return;
}

const bool HybridAStarParkPlanner::CheckEgoReplanNumber(
    const bool is_replan) {
  if (is_replan) {
    // check total plan number
    if (frame_.total_plan_count > 30) {
      return false;
    }
  }

  // check plan number in slot
  if (is_replan && current_gear_ == AstarPathGear::reverse &&
      frame_.ego_slot_info.slot_occupied_ratio > 0.2) {
    if (in_slot_car_adjust_count_ >= 3) {
      return false;
    }
  }

  return true;
}

const bool HybridAStarParkPlanner::IsEgoNeedDriveForwardInSlot(
    const Pose2D& ego_pose, const double slot_width, const double slot_len) {
  if (current_gear_ == AstarPathGear::drive) {
    return false;
  }

  double ego_lat_offset = std::fabs(ego_pose.y);
  bool need_drive_forward = false;

  // if ego is beyond slot, can use spiral/rs/polynomial to adjust ego. Not use
  // astar because maybe switch gear too much or generate weird path.
  if (ego_pose.x >= slot_len) {
    if (ego_lat_offset < 1.0 && std::fabs(ego_pose.theta) < ifly_deg2rad(5.0)) {
      need_drive_forward = true;
    }
  } else {
    if (ego_lat_offset < 1.0 &&
        std::fabs(ego_pose.theta) < ifly_deg2rad(15.0)) {
      need_drive_forward = true;
    }
  }

  ILOG_INFO << "dist = " << ego_lat_offset
            << ",theta = " << ifly_rad2deg(ego_pose.theta);

  return need_drive_forward;
}

const double HybridAStarParkPlanner::CalRemainDistFromPath() {
  double remain_dist = 20.0;

  // no path
  size_t path_point_size = current_path_point_global_vec_.size();
  if (current_path_point_global_vec_.size() <= 1) {
    return 0.0;
  }

  const MeasurementData* measures_ptr =
      &(apa_world_ptr_->GetApaDataPtr()->measurement_data);
  Pose2D ego_pose(measures_ptr->pos[0], measures_ptr->pos[1],
                  measures_ptr->heading);

  size_t nearest_point_id = 100000;
  nearest_point_id =
      GetNearestPathPoint(current_path_point_global_vec_, ego_pose);
  if (nearest_point_id >= path_point_size) {
    return 0.0;
  }

  // calc base vector
  pnc::geometry_lib::PathPoint *base_point;
  base_point = &current_path_point_global_vec_[nearest_point_id];
  ad_common::math::Vec2d base_vector;
  if (nearest_point_id == path_point_size - 1) {
    const pnc::geometry_lib::PathPoint& point1 =
        current_path_point_global_vec_[nearest_point_id - 1];
    const pnc::geometry_lib::PathPoint& point2 =
        current_path_point_global_vec_[nearest_point_id];

    base_vector.set_x(point2.pos.x() - point1.pos.x());
    base_vector.set_y(point2.pos.y() - point1.pos.y());
  } else {
    const pnc::geometry_lib::PathPoint& point1 =
        current_path_point_global_vec_[nearest_point_id];
    const pnc::geometry_lib::PathPoint& point2 =
        current_path_point_global_vec_[nearest_point_id + 1];
    base_vector.set_x(point2.pos.x() - point1.pos.x());
    base_vector.set_y(point2.pos.y() - point1.pos.y());
  }

  if (base_vector.LengthSquare() > 1e-4) {
    base_vector.Normalize();
  } else {
    if (current_gear_ == AstarPathGear::drive) {
      base_vector.set_x(std::cos(base_point->heading));
      base_vector.set_y(std::sin(base_point->heading));
    } else {
      base_vector.set_x(std::cos(base_point->heading + M_PI));
      base_vector.set_y(std::sin(base_point->heading + M_PI));
    }
  }

  ad_common::math::Vec2d projection_vector;
  projection_vector.set_x(ego_pose.x - base_point->pos.x());
  projection_vector.set_y(ego_pose.y - base_point->pos.y());

  double horizon_dist = base_vector.InnerProd(projection_vector);

  // end point
  if (nearest_point_id == path_point_size - 1) {
    if (horizon_dist > 0.0) {
      remain_dist = 0.0;
    } else {
      remain_dist = -horizon_dist;
    }
  } else {
    // get dist to goal
    double total_dist = -horizon_dist;
    double delta_dist;
    for (size_t i = nearest_point_id; i < path_point_size - 1; i++) {
      const pnc::geometry_lib::PathPoint& point1 =
          current_path_point_global_vec_[i];
      const pnc::geometry_lib::PathPoint& point2 =
          current_path_point_global_vec_[i + 1];

      delta_dist = GetTwoPointDist(point1, point2);
      total_dist += delta_dist;
    }

    remain_dist = total_dist;
  }

  return remain_dist;
}

size_t HybridAStarParkPlanner::GetNearestPathPoint(
    const std::vector<pnc::geometry_lib::PathPoint>& path, const Pose2D& pose) {
  double nearest_dist = 10000.0;
  double dist;
  Pose2D global_pose;
  size_t nearest_id = 0;

  for (size_t i = 0; i < path.size(); i++) {
    global_pose.x = path[i].pos[0];
    global_pose.y = path[i].pos[1];
    global_pose.theta = path[i].heading;
    dist = pose.DistanceSquareTo(&global_pose);

    if (i == 0) {
      nearest_dist = dist;
      nearest_id = i;

      continue;
    }

    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_id = i;
    }
  }

  return nearest_id;
}

void HybridAStarParkPlanner::DebugPathString(
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  for (size_t i = 0; i < path.size(); i++) {
    ILOG_INFO << "i = " << i << ",x = " << path[i].pos.x()
              << ",y = " << path[i].pos.y();
  }
  return;
}

}  // namespace apa_planner
}  // namespace planning