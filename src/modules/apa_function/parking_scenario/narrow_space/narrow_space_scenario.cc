#include "narrow_space_scenario.h"

#include <cmath>
#include <cstddef>
#include <cstdio>

#include "apa_slot.h"
#include "collision_detection/path_safe_checker.h"
#include "common.pb.h"
#include "common_c.h"
#include "geometry_math.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "hybrid_astar_response.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math/vec2d.h"
#include "math_utils.h"
#include "narrow_space_decider.h"
#include "parking_scenario.h"
#include "point_cloud_obstacle.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "transform2d.h"
#include "utils_math.h"
#include "virtual_wall_decider.h"

namespace planning {
namespace apa_planner {

NarrowSpaceScenario::NarrowSpaceScenario(
    const std::shared_ptr<ApaWorld>& apa_world_ptr)
    : ParkingScenario(apa_world_ptr) {
  Init();
}

void NarrowSpaceScenario::Reset() {
  frame_.Reset();
  current_path_point_global_vec_.clear();

  // reset planning output
  memset(&planning_output_, 0, sizeof(planning_output_));

  memset(&apa_hmi_, 0, sizeof(apa_hmi_));

  const ApaParameters& params = apa_param.GetParam();
  if (params.path_generator_type == ParkPathGenerationType::SEARCH_BASED) {
    // init thread first
    if (!thread_.IsInit()) {
      thread_.Init(params.rear_overhanging, params.car_length, params.car_width,
                   params.steer_ratio, params.wheel_base,
                   params.min_turn_radius,
                   (params.max_car_width - params.car_width) * 0.5);
      thread_.Start();
    }
  }

  current_gear_ = AstarPathGear::PARKING;
  in_slot_car_adjust_count_ = 0;
  is_path_connected_to_goal_ = false;

  ParkingScenario::Reset();

  narrow_space_decider_.Clear();
  virtual_wall_decider_.Reset(Pose2D(0, 0, 0));

  return;
}

void NarrowSpaceScenario::Init() {
  const ApaParameters& params = apa_param.GetParam();

  // todo, system should use same vehicle parameter configuration file and
  // data structure.
  ILOG_INFO << "init astar thread";

  current_gear_ = AstarPathGear::PARKING;
  in_slot_car_adjust_count_ = 0;
  is_path_connected_to_goal_ = false;

  thread_.Init(params.rear_overhanging, params.car_length, params.car_width,
               params.steer_ratio, params.wheel_base, params.min_turn_radius,
               (params.max_car_width - params.car_width) * 0.5);

  thread_.Start();

  return;
}

const bool NarrowSpaceScenario::CheckFinished() {
  bool ret = false;
  if (apa_world_ptr_->GetSlotManagerPtr()
          ->ego_info_under_slot_.slot.slot_type_ == SlotType::PARALLEL) {
    ret = CheckParallelSlotFinished();
  } else {
    ret = CheckVerticalSlotFinished();
  }

  return ret;
}

const bool NarrowSpaceScenario::CheckVerticalSlotFinished() {
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;
  const ApaParameters& config = apa_param.GetParam();

  const bool lon_condition =
      ego_info.terminal_err.pos.x() < config.finish_lon_err;

  const double lat_offset = std::fabs(ego_info.cur_pose.pos.y() -
                                      thread_.GetAstarTargetPose().GetY());

  const double ego_head_lat_offset = std::fabs(
      (ego_info.cur_pose.pos + (config.wheel_base + config.front_overhanging) *
                                   ego_info.cur_pose.heading_vec)
          .y() -
      thread_.GetAstarTargetPose().GetY());

  const bool ego_center_lat_condition =
      std::fabs(lat_offset) <= apa_param.GetParam().finish_lat_err_strict;

  const bool ego_head_lat_condition =
      ego_center_lat_condition &&
      std::fabs(ego_head_lat_offset) <= apa_param.GetParam().finish_lat_err;

  const bool heading_condition_1 =
      std::fabs(ego_info.terminal_err.heading) <=
      apa_param.GetParam().finish_heading_err * kDeg2Rad;

  const bool heading_condition_2 =
      std::fabs(ego_info.terminal_err.heading) <=
      (apa_param.GetParam().finish_heading_err + 1.988) * kDeg2Rad;

  const bool lat_condition =
      (ego_center_lat_condition && heading_condition_1) &&
      (ego_head_lat_condition && heading_condition_2);

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const bool remain_s_condition =
      frame_.remain_dist_path < apa_param.GetParam().max_replan_remain_dist;

  bool parking_finish =
      lon_condition && lat_condition && static_condition && remain_s_condition;

  if (parking_finish) {
    return true;
  }

  // stucked by directly behind uss
  const std::shared_ptr<UssObstacleAvoidance>& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetCollisionDetectorInterfacePtr()
          ->GetUssObsAvoidancePtr();
  const bool enter_slot_condition =
      ego_info.slot_occupied_ratio >
      apa_param.GetParam().finish_uss_slot_occupied_ratio;
  const bool remain_uss_condition =
      frame_.remain_dist_obs < apa_param.GetParam().max_replan_remain_dist;

  parking_finish = lat_condition && static_condition && enter_slot_condition &&
                   remain_uss_condition;

  if (parking_finish) {
    return true;
  }

  parking_finish = lat_condition && static_condition && enter_slot_condition &&
                   (ego_info.terminal_err.pos.x() < 0.568);

  if (parking_finish) {
    return true;
  }

  // 车辆不压线，车头基本摆正，就认定完成泊车
  if (static_condition && remain_s_condition && lon_condition &&
      heading_condition_2) {
    Pose2D ego;
    ego.x = ego_info.cur_pose.pos[0];
    ego.y = ego_info.cur_pose.pos[1];
    ego.theta = ego_info.cur_pose.heading;
    if (!IsVehicleOverlapWithSlotLine(ego_info.slot.slot_length_,
                                      ego_info.slot.slot_width_, ego)) {
      ILOG_INFO << "vehicle is inside slot line, finish";
      return true;
    }
  }

  return false;
}

void NarrowSpaceScenario::ExcutePathPlanningTask() {
  // prepare simulation
  InitSimulation();

  // check planning status
  if (CheckPlanSkip()) {
    return;
  }

  UpdateStuckTime();

  if (CheckPaused()) {
    SetParkingStatus(PARKING_PAUSED);
    if (frame_.pause_time > apa_param.GetParam().pause_failed_time) {
      SetParkingStatus(PARKING_FAILED);
      frame_.plan_fail_reason = PAUSE_FAILED_TIME;
    }
    return;
  }

  // calculate remain dist according to plan path
  frame_.remain_dist_path = CalRemainDistFromPath();

  // calculate remain dist uss according to uss
  frame_.remain_dist_obs = CalRemainDistFromObs(0.31);

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = UPDATE_EGO_SLOT_INFO;
    return;
  }

  // todo: do not shrink in every frame
  PathShrinkBySlotLimiter();
  PathExpansionBySlotLimiter();

  // check finish
  if (CheckFinished()) {
    SetParkingStatus(PARKING_FINISHED);
    ILOG_INFO << "park finish";
    return;
  }

  // check failed
  if (CheckStuckFailed(12.0)) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = STUCK_FAILED_TIME;
    return;
  }

  bool is_replan = CheckReplan(
      apa_param.GetParam().max_replan_remain_dist, 0.068,
      apa_param.GetParam().max_replan_remain_dist,
      apa_param.GetParam().astar_config.deadend_uss_stuck_replan_wait_time,
      apa_param.GetParam().stuck_replan_time);

  if (!CheckEgoReplanNumber(is_replan)) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = PLAN_COUNT_EXCEED_LIMIT;
    return;
  }

  bool update_thread_path = UpdateThreadPath();
  PathPlannerResult path_plan_result = PathPlannerResult::PLAN_FAILED;

  ILOG_INFO << "stuck_uss_time = " << frame_.stuck_obs_time
            << " ,is_replan = " << is_replan;

  // check replan
  if (is_replan || update_thread_path) {
    ILOG_INFO << "plan reason = " << GetPlanReason(frame_.replan_reason)
              << ",force replan = " << apa_world_ptr_->GetSimuParam().force_plan
              << ",thread update = " << update_thread_path
              << ",is_replan = " << is_replan;

    frame_.replan_flag = true;
    path_plan_result = PlanBySearchBasedMethod(false);
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
        SetParkingStatus(PARKING_RUNNING);
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

void NarrowSpaceScenario::Log() const {
  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag);

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;
  const geometry_lib::LocalToGlobalTf& l2g_tf = ego_info_under_slot.l2g_tf;

  std::vector<double> slot_corner_X;
  slot_corner_X.clear();
  slot_corner_X.reserve(16);
  std::vector<double> slot_corner_Y;
  slot_corner_Y.clear();
  slot_corner_Y.reserve(16);

  const auto& origin_corner_coord_global =
      ego_info_under_slot.slot.origin_corner_coord_global_;

  std::vector<Eigen::Vector2d> pt_vec{
      origin_corner_coord_global.pt_0, origin_corner_coord_global.pt_1,
      origin_corner_coord_global.pt_2, origin_corner_coord_global.pt_3};

  for (const Eigen::Vector2d& pt : pt_vec) {
    slot_corner_X.emplace_back(pt.x());
    slot_corner_Y.emplace_back(pt.y());
  }

  slot_corner_X.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).x());
  slot_corner_Y.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).y());
  slot_corner_X.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).x());
  slot_corner_Y.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).y());

  pt_vec[0] = pt_vec[0] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_01_vec.normalized();
  pt_vec[1] = pt_vec[1] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_01_vec.normalized();
  pt_vec[2] = pt_vec[2] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_23_vec.normalized();
  pt_vec[3] = pt_vec[3] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_23_vec.normalized();

  for (const Eigen::Vector2d& pt : pt_vec) {
    slot_corner_X.emplace_back(pt.x());
    slot_corner_Y.emplace_back(pt.y());
  }

  slot_corner_X.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).x());
  slot_corner_Y.emplace_back(((pt_vec[0] + pt_vec[1]) * 0.5).y());
  slot_corner_X.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).x());
  slot_corner_Y.emplace_back(((pt_vec[2] + pt_vec[3]) * 0.5).y());

  slot_corner_X.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.target_pose.pos).x());
  slot_corner_Y.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.target_pose.pos).y());

  JSON_DEBUG_VECTOR("slot_corner_X", slot_corner_X, 3);
  JSON_DEBUG_VECTOR("slot_corner_Y", slot_corner_Y, 3);

  std::vector<double> limiter_corner_X;
  limiter_corner_X.clear();
  limiter_corner_X.reserve(3);
  std::vector<double> limiter_corner_Y;
  limiter_corner_Y.clear();
  limiter_corner_Y.reserve(3);

  limiter_corner_X.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.virtual_limiter.first).x());
  limiter_corner_X.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.virtual_limiter.second).x());
  limiter_corner_Y.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.virtual_limiter.first).y());
  limiter_corner_Y.emplace_back(
      l2g_tf.GetPos(ego_info_under_slot.virtual_limiter.second).y());

  JSON_DEBUG_VECTOR("limiter_corner_X", limiter_corner_X, 2);
  JSON_DEBUG_VECTOR("limiter_corner_Y", limiter_corner_Y, 2);

  JSON_DEBUG_VALUE("terminal_error_x",
                   ego_info_under_slot.terminal_err.pos.x());
  JSON_DEBUG_VALUE("terminal_error_y",
                   ego_info_under_slot.terminal_err.pos.y());
  JSON_DEBUG_VALUE("terminal_error_heading",
                   ego_info_under_slot.terminal_err.heading)

  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag)
  JSON_DEBUG_VALUE("is_replan_first", frame_.is_replan_first)
  JSON_DEBUG_VALUE("is_replan_by_uss", frame_.is_replan_by_obs)
  JSON_DEBUG_VALUE("current_path_length", frame_.current_path_length)
  JSON_DEBUG_VALUE("path_plan_success", frame_.plan_stm.path_plan_success)
  JSON_DEBUG_VALUE("planning_status", frame_.plan_stm.planning_status)
  JSON_DEBUG_VALUE("spline_success", frame_.spline_success)
  JSON_DEBUG_VALUE("remain_dist", frame_.remain_dist_path)
  JSON_DEBUG_VALUE("remain_dist_col_det", frame_.remain_dist_col_det)
  JSON_DEBUG_VALUE("remain_dist_uss", frame_.remain_dist_obs)
  JSON_DEBUG_VALUE("stuck_time", frame_.stuck_time)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
  JSON_DEBUG_VALUE("plan_fail_reason", frame_.plan_fail_reason)
  JSON_DEBUG_VALUE("dynamic_replan_count", frame_.dynamic_replan_count)
  JSON_DEBUG_VALUE("ego_heading_slot", ego_info_under_slot.cur_pose.heading)

  JSON_DEBUG_VALUE("selected_slot_id", ego_info_under_slot.id);
  JSON_DEBUG_VALUE("slot_length", ego_info_under_slot.slot.slot_length_);
  JSON_DEBUG_VALUE("slot_width", ego_info_under_slot.slot.slot_width_);

  JSON_DEBUG_VALUE("slot_origin_pos_x",
                   ego_info_under_slot.origin_pose_global.pos.x());

  JSON_DEBUG_VALUE("slot_origin_pos_y",
                   ego_info_under_slot.origin_pose_global.pos.y());

  JSON_DEBUG_VALUE("slot_origin_heading",
                   ego_info_under_slot.origin_pose_global.heading);

  JSON_DEBUG_VALUE("slot_occupied_ratio",
                   ego_info_under_slot.slot_occupied_ratio);

  std::vector<double> target_ego_pos_slot = {
      ego_info_under_slot.target_pose.pos.x(),
      ego_info_under_slot.target_pose.pos.y()};

  JSON_DEBUG_VALUE("pathplan_result", frame_.pathplan_result);
  JSON_DEBUG_VECTOR("target_ego_pos_slot", target_ego_pos_slot, 2);

  const UssObstacleAvoidance::RemainDistInfo uss_info =
      apa_world_ptr_->GetCollisionDetectorInterfacePtr()
          ->GetUssObsAvoidancePtr()
          ->GetRemainDistInfo();
  JSON_DEBUG_VALUE("uss_available", uss_info.is_available);
  JSON_DEBUG_VALUE("uss_remain_dist", uss_info.remain_dist);
  JSON_DEBUG_VALUE("uss_index", uss_info.uss_index);
  JSON_DEBUG_VALUE("uss_car_index", uss_info.car_index);

  // lateral optimization
  const auto plan_debug_info =
      apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();

  if (plan_debug_info.has_terminal_pos_error()) {
    JSON_DEBUG_VALUE("optimization_terminal_pose_error",
                     plan_debug_info.terminal_pos_error());
    JSON_DEBUG_VALUE("optimization_terminal_heading_error",
                     plan_debug_info.terminal_heading_error());
  } else {
    JSON_DEBUG_VALUE("optimization_terminal_pose_error", 0.0);
    JSON_DEBUG_VALUE("optimization_terminal_heading_error", 0.0);
  }

  JSON_DEBUG_VECTOR("plan_traj_x", std::vector<double>{0.0}, 3);
  JSON_DEBUG_VECTOR("plan_traj_y", std::vector<double>{0.0}, 3);
  JSON_DEBUG_VECTOR("plan_traj_heading", std::vector<double>{0.0}, 3);
  JSON_DEBUG_VECTOR("plan_traj_lat_buffer", std::vector<double>{0.0}, 3);

  return;
}

const bool NarrowSpaceScenario::GenTlane() { return true; }

const bool NarrowSpaceScenario::GenObstacles() { return true; }

const uint8_t NarrowSpaceScenario::PathPlanOnce() { return false; }

const std::string NarrowSpaceScenario::GetPlanReason(const uint8_t type) {
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

PathPlannerResult NarrowSpaceScenario::PlanBySearchBasedMethod(
    const bool is_scenario_try) {
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  PathPlannerResult res = PathPlannerResult::WAIT_PATH;

  // start
  Pose2D start;
  start.x = ego_info.cur_pose.pos[0];
  start.y = ego_info.cur_pose.pos[1];
  start.theta = ego_info.cur_pose.heading;

  // real target pose in slot
  Pose2D real_end;
  real_end.x = ego_info.target_pose.pos[0];
  real_end.y = ego_info.target_pose.pos[1];
  real_end.theta = ego_info.target_pose.heading;

  // astar end, maybe different with real end.
  Pose2D end = real_end;
  double end_straight_len;
  ParkSpaceType slot_type;
  ParkingVehDirection parking_in_type;
  ApaStateMachine fsm =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine();

  if (apa_world_ptr_->GetSlotManagerPtr()
          ->ego_info_under_slot_.slot.slot_type_ == SlotType::PARALLEL) {
    end_straight_len =
        apa_param.GetParam().astar_config.parallel_slot_end_straight_dist;
    slot_type = ParkSpaceType::PARALLEL;
  } else if (apa_world_ptr_->GetSlotManagerPtr()
                 ->ego_info_under_slot_.slot.slot_type_ == SlotType::SLANT) {
    end_straight_len =
        apa_param.GetParam().astar_config.vertical_tail_in_end_straight_dist;
    slot_type = ParkSpaceType::SLANTING;
  } else {
    if (fsm == ApaStateMachine::ACTIVE_IN_CAR_REAR ||
        fsm == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR) {
      end_straight_len =
          apa_param.GetParam().astar_config.vertical_tail_in_end_straight_dist;

      parking_in_type = ParkingVehDirection::TAIL_IN;
    } else {
      end_straight_len =
          apa_param.GetParam().astar_config.vertical_head_in_end_straight_dist;
      parking_in_type = ParkingVehDirection::HEAD_IN;
    }
    slot_type = ParkSpaceType::VERTICAL;
  }
  end.x = real_end.x + end_straight_len;

  double astar_start_time = IflyTime::Now_ms();
  Pose2D slot_base_pose = Pose2D(ego_info.origin_pose_global.pos.x(),
                                 ego_info.origin_pose_global.pos.y(),
                                 ego_info.origin_pose_global.heading);

  ParkObstacleList obs;

  // If in searching, use ego init bound;
  // If in parking, use ego position init bound by first plan;
  if (is_scenario_try || frame_.replan_reason == FIRST_PLAN) {
    virtual_wall_decider_.Init(start);
  }
  virtual_wall_decider_.Process(obs.virtual_obs, ego_info.slot.slot_width_,
                                ego_info.slot.slot_length_, start, real_end,
                                slot_type, ego_info.slot_side, parking_in_type);

  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_info.g2l_tf);

  PointCloudObstacleTransform obstacle_generator;
  obstacle_generator.GenerateLocalObstacle(
      apa_world_ptr_->GetObstacleManagerPtr(), obs);

  double search_start_time = IflyTime::Now_ms();
  ILOG_INFO << "fusion obj time ms " << search_start_time - astar_start_time;

  // set input
  AstarRequest cur_request;
  cur_request.path_generate_method =
      planning::AstarPathGenerateType::ASTAR_SEARCHING;
  cur_request.first_action_request.has_request = true;
  cur_request.first_action_request.gear_request = AstarPathGear::NONE;
  cur_request.space_type = slot_type;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT) {
    cur_request.direction_request = ParkingVehDirection::HEAD_IN;
  } else if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
                 ApaStateMachine::ACTIVE_IN_CAR_REAR ||
             apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
                 ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR) {
    cur_request.direction_request = ParkingVehDirection::TAIL_IN;
  }

  cur_request.rs_request = RSPathRequestType::none;
  cur_request.timestamp_ms = astar_start_time;
  cur_request.slot_id = ego_info.id;

  cur_request.start_ = start;
  cur_request.base_pose_ = slot_base_pose;
  cur_request.real_goal = real_end;

  cur_request.slot_width = ego_info.slot.GetWidth();
  cur_request.slot_length = ego_info.slot.GetLength();
  cur_request.history_gear = current_gear_;
  frame_.current_gear = current_gear_ == AstarPathGear::DRIVE
                            ? geometry_lib::SEG_GEAR_REVERSE
                            : geometry_lib::SEG_GEAR_INVALID;

  switch (frame_.replan_reason) {
    case FIRST_PLAN:
      cur_request.plan_reason = PlanningReason::FIRST_PLAN;
      break;
    case SEG_COMPLETED_PATH:
      cur_request.plan_reason = PlanningReason::PATH_COMPLETED;
      break;
    case SEG_COMPLETED_OBS:
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

  is_path_connected_to_goal_ = false;

  // generate request
  bool need_adjust_plan = IsEgoNeedAdjustInSlot(start, ego_info.slot.GetWidth(),
                                                ego_info.slot.GetLength());
  if (need_adjust_plan) {
    if (apa_param.GetParam().astar_config.cubic_polynomial_pose_adjustment) {
      cur_request.path_generate_method =
          planning::AstarPathGenerateType::CUBIC_POLYNOMIAL_SAMPLING;
    } else {
      cur_request.path_generate_method =
          planning::AstarPathGenerateType::REEDS_SHEPP_SAMPLING;
    }
    end.y = real_end.y;
    end.x = start.x + 30.0;
    cur_request.goal_ = end;

  } else {
    cur_request.goal_ = end;
  }

  // gear need be different with history in next replanning
  if (frame_.replan_reason != FIRST_PLAN) {
    switch (current_gear_) {
      case AstarPathGear::REVERSE:
        cur_request.first_action_request.gear_request = AstarPathGear::DRIVE;
        break;
      case AstarPathGear::DRIVE:
        cur_request.first_action_request.gear_request = AstarPathGear::REVERSE;
        break;
      default:
        break;
    }
  }

  if (is_scenario_try) {
    cur_request.path_generate_method =
        planning::AstarPathGenerateType::TRY_SEARCHING;
    cur_request.first_action_request.gear_request = AstarPathGear::NONE;
  }

  // 目前,平行车位入库使用混合A星搜索，交换起点终点
  cur_request.swap_start_goal = false;
  if (cur_request.space_type == ParkSpaceType::PARALLEL) {
    if (cur_request.path_generate_method ==
            planning::AstarPathGenerateType::ASTAR_SEARCHING ||
        cur_request.path_generate_method ==
            planning::AstarPathGenerateType::TRY_SEARCHING) {
      cur_request.swap_start_goal = true;
      ClearFirstActionReqeust(&cur_request);
    }
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
      pnc::geometry_lib::PathPoint point;

      for (i = 0; i < response.first_seg_path.size(); i++) {
        point = pnc::geometry_lib::PathPoint(
            Eigen::Vector2d(response.first_seg_path[i].x,
                            response.first_seg_path[i].y),
            response.first_seg_path[i].phi, response.first_seg_path[i].kappa);
        point.s = response.first_seg_path[i].accumulated_s;

        local_path.emplace_back(point);
      }

      // todo:
      if ((fsm == ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
           fsm == ApaStateMachine::ACTIVE_IN_CAR_REAR) &&
          frame_.is_replan_first) {
        frame_.is_replan_first = false;
      }

      double search_end_time = IflyTime::Now_ms();

      // check path is single shot to goal.
      if (response.result.gear_change_num > 0) {
        is_path_connected_to_goal_ = false;
      } else {
        is_path_connected_to_goal_ = true;
      }

      double path_dist = 0.0;
      path_dist = response.first_seg_path.back().accumulated_s;
      ILOG_INFO << "first path gear = "
                << PathGearDebugString(response.first_seg_path[0].gear)
                << ",dist = " << path_dist
                << ",gear_change_num=" << response.result.gear_change_num;

      PathOptimizationByCILRQ(local_path, &response_tf);

      double lqr_end_time = IflyTime::Now_ms();
      ILOG_INFO << "lqr time ms " << lqr_end_time - search_end_time;

      if (!is_scenario_try) {
        PublishHybridAstarDebugInfo(response.result, &thread_, &response_tf);

        double publish_end_time = IflyTime::Now_ms();
        ILOG_INFO << "publish time ms " << publish_end_time - lqr_end_time;

        frame_.total_plan_count++;
        ILOG_INFO << "frame_.total_plan_count = "
                  << static_cast<int>(frame_.total_plan_count);
      }

      res = PathPlannerResult::PLAN_UPDATE;
    } else {
      res = PathPlannerResult::PLAN_FAILED;

      // publish fallback path
      GenerateFallBackPath();

      ILOG_INFO << "path plan point less 5";
    }

    // update gear
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (response.first_seg_path.size() > 0) {
      if (response.first_seg_path[0].gear == AstarPathGear::DRIVE) {
        frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
      }

      if (fsm == ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
          fsm == ApaStateMachine::ACTIVE_IN_CAR_REAR) {
        current_gear_ = response.first_seg_path[0].gear;
      }

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

    if (ego_info.slot_occupied_ratio > 0.2) {
      in_slot_car_adjust_count_++;
      ILOG_INFO << "in_slot_car_adjust_count_ = " << in_slot_car_adjust_count_;
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

const int NarrowSpaceScenario::PublishHybridAstarDebugInfo(
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

  // do not publish it.
  if (0) {
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
  }

  return 0;
}

const int NarrowSpaceScenario::HybridAstarDebugInfoClear() {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();

  // debug_->mutable_refline_info()->Clear();

  planning::common::AstarNodeList* list = debug_->mutable_node_list();
  list->Clear();

  return 0;
}

const void NarrowSpaceScenario::GenerateFallBackPath() {
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  if (frame_.replan_reason == 5) {
    return;
  }

  const std::shared_ptr<ApaMeasureDataManager> measures_ptr =
      apa_world_ptr_->GetMeasureDataManagerPtr();
  pnc::geometry_lib::PathPoint global_point;
  global_point.Set(measures_ptr->GetPos(), measures_ptr->GetHeading());

  // todo, use one point
  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.emplace_back(global_point);

  return;
}

const int NarrowSpaceScenario::PathOptimizationByCILRQ(
    const std::vector<pnc::geometry_lib::PathPoint>& local_path,
    Transform2d* tf) {
  LocalPathToGlobal(local_path, tf);
  ILOG_INFO << "output path by coarse a star path";

  return 0;
}

const int NarrowSpaceScenario::LocalPathToGlobal(
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
    global_point.s = path_point.s;

    current_path_point_global_vec_.emplace_back(global_point);
  }

  return 0;
}

const bool NarrowSpaceScenario::UpdateThreadPath() {
  thread_.GetThreadState(&thread_state_);

  ILOG_INFO << "thread state " << static_cast<int>(thread_state_);

  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    ILOG_INFO << "fetch path";

    return true;
  }

  return false;
}

const bool NarrowSpaceScenario::UpdateEgoSlotInfo() {
  bool ret = false;
  if (apa_world_ptr_->GetSlotManagerPtr()
          ->ego_info_under_slot_.slot.slot_type_ == SlotType::PARALLEL) {
    ret = UpdateParallelSlotInfo();
  } else {
    ret = UpdateVerticalSlotInfo();
  }

  return ret;
}

const bool NarrowSpaceScenario::UpdateVerticalSlotInfo() {
  const std::shared_ptr<ApaMeasureDataManager> measures_ptr =
      apa_world_ptr_->GetMeasureDataManagerPtr();

  const ApaParameters& param = apa_param.GetParam();
  frame_.replan_flag = false;

  // 建立车位坐标系 根据23角点或者限位器角点确定规划终点位姿
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  ego_info_under_slot.origin_pose_global.heading_vec =
      ego_info_under_slot.slot.processed_corner_coord_global_.pt_23mid_01_mid
          .normalized();

  ego_info_under_slot.origin_pose_global.heading =
      std::atan2(ego_info_under_slot.origin_pose_global.heading_vec.y(),
                 ego_info_under_slot.origin_pose_global.heading_vec.x());

  ego_info_under_slot.origin_pose_global.pos =
      ego_info_under_slot.slot.processed_corner_coord_global_.pt_01_mid -
      ego_info_under_slot.slot.slot_length_ *
          ego_info_under_slot.origin_pose_global.heading_vec;

  ego_info_under_slot.g2l_tf = geometry_lib::GlobalToLocalTf(
      ego_info_under_slot.origin_pose_global.pos,
      ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.l2g_tf = geometry_lib::LocalToGlobalTf(
      ego_info_under_slot.origin_pose_global.pos,
      ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.origin_pose_local.pos = ego_info_under_slot.g2l_tf.GetPos(
      ego_info_under_slot.origin_pose_global.pos);

  ego_info_under_slot.origin_pose_local.heading =
      ego_info_under_slot.g2l_tf.GetHeading(
          ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.origin_pose_local.heading_vec =
      geometry_lib::GenHeadingVec(
          ego_info_under_slot.origin_pose_local.heading);

  ego_info_under_slot.slot.TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  ego_info_under_slot.cur_pose.pos =
      ego_info_under_slot.g2l_tf.GetPos(measures_ptr->GetPos());
  ego_info_under_slot.cur_pose.heading =
      ego_info_under_slot.g2l_tf.GetHeading(measures_ptr->GetHeading());
  ego_info_under_slot.cur_pose.heading_vec =
      geometry_lib::GenHeadingVec(ego_info_under_slot.cur_pose.heading);

  if (frame_.is_replan_first) {
    const Eigen::Vector2d ego_to_slot_center_vec =
        ego_info_under_slot.slot.origin_corner_coord_global_.pt_center -
        measures_ptr->GetPos();

    const double cross_ego_to_slot_center =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->GetHeadingVec(),
                                                ego_to_slot_center_vec);

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(
            measures_ptr->GetHeadingVec(),
            ego_info_under_slot.origin_pose_global.heading_vec);

    // 这个初始参考挡位对迭代式路径规划已经没有什么意义
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
      ego_info_under_slot.slot_side = geometry_lib::SLOT_SIDE_RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      ego_info_under_slot.slot_side = geometry_lib::SLOT_SIDE_LEFT;
    } else {
      ego_info_under_slot.slot_side = geometry_lib::SLOT_SIDE_INVALID;
    }
  }

  // 计算停车位置
  double virtual_tar_x = 0.0;
  if (ego_info_under_slot.slot.limiter_.valid) {
    // 根据原始限位器点计算停车终点
    Eigen::Vector2d pt1 = ego_info_under_slot.g2l_tf.GetPos(
        ego_info_under_slot.slot.limiter_.start_pt);
    Eigen::Vector2d pt2 = ego_info_under_slot.g2l_tf.GetPos(
        ego_info_under_slot.slot.limiter_.end_pt);

    virtual_tar_x = 0.5 * (pt1 + pt2).x() + param.limiter_move_dist;
  } else {
    // 根据后面两个角点计算停车终点
    virtual_tar_x =
        ego_info_under_slot.slot.processed_corner_coord_local_.pt_23_mid.x() +
        param.terminal_target_x;
  }

  // 如果限位器很靠后 可以结合一下前面两个车位角点信息
  virtual_tar_x = std::max(
      virtual_tar_x,
      ego_info_under_slot.slot.processed_corner_coord_local_.pt_01_mid.x() -
          param.limiter_length - param.wheel_base - param.front_overhanging);

  ego_info_under_slot.virtual_limiter.first.x() = virtual_tar_x;
  ego_info_under_slot.virtual_limiter.second.x() = virtual_tar_x;
  ego_info_under_slot.virtual_limiter.first.y() =
      0.5 * ego_info_under_slot.slot.slot_width_;
  ego_info_under_slot.virtual_limiter.second.y() =
      -0.5 * ego_info_under_slot.slot.slot_width_;

  // 后续横向终点位置会随着障碍物而进行改变
  ego_info_under_slot.target_pose.pos
      << ego_info_under_slot.virtual_limiter.first.x(),
      param.terminal_target_y;
  ego_info_under_slot.target_pose.heading = param.terminal_target_heading;
  ego_info_under_slot.target_pose.heading_vec = Eigen::Vector2d(1, 0);

  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT) {
    ego_info_under_slot.target_pose.pos[0] += param.wheel_base;
    ego_info_under_slot.target_pose.heading += M_PI;
    ego_info_under_slot.target_pose.heading_vec = Eigen::Vector2d(-1, 0);
  }

  // 终点误差
  ego_info_under_slot.terminal_err.Set(
      ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
      geometry_lib::NormalizeAngle(ego_info_under_slot.cur_pose.heading -
                                   ego_info_under_slot.target_pose.heading));

  // 固定车位,计算占库比
  if (std::fabs(ego_info_under_slot.terminal_err.pos.y()) <
          param.slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_info_under_slot.terminal_err.heading) <
          param.slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    // 车头泊入占比
    if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
            ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
        apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
            ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT) {
      const std::vector<double> x_tab = {
          ego_info_under_slot.target_pose.pos.x() - param.rear_overhanging -
              param.front_overhanging,
          ego_info_under_slot.slot.slot_length_};

      const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
      const Eigen::Vector2d front_car_pos =
          ego_info_under_slot.cur_pose.pos +
          (param.wheel_base + param.front_overhanging) *
              ego_info_under_slot.cur_pose.heading_vec;

      ego_info_under_slot.slot_occupied_ratio =
          mathlib::Interp1(x_tab, occupied_ratio_tab, front_car_pos.x());
    } else {
      // 车尾泊入占比
      const std::vector<double> x_tab = {
          ego_info_under_slot.target_pose.pos.x(),
          ego_info_under_slot.slot.slot_length_ + param.rear_overhanging};

      const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
      ego_info_under_slot.slot_occupied_ratio = mathlib::Interp1(
          x_tab, occupied_ratio_tab, ego_info_under_slot.cur_pose.pos.x());
    }

  } else {
    ego_info_under_slot.slot_occupied_ratio = 0.0;
  }
  ILOG_INFO << "slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;
  // trim or extend path according to limiter, only run once
  frame_.correct_path_for_limiter = false;
  if (((apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
            ApaStateMachine::ACTIVE_IN_CAR_REAR &&
        frame_.gear_command == geometry_lib::SEG_GEAR_REVERSE)) &&
      !ego_info_under_slot.fix_slot) {
    const geometry_lib::LineSegment limiter_line(
        ego_info_under_slot.virtual_limiter.first,
        ego_info_under_slot.virtual_limiter.second);

    const double dist_ego_limiter = geometry_lib::CalPoint2LineDist(
        ego_info_under_slot.cur_pose.pos, limiter_line);

    ILOG_INFO << "dist_ego_limiter = " << dist_ego_limiter;

    if (dist_ego_limiter < param.car_to_limiter_dis) {
      ILOG_INFO << "should correct path according limiter";
      ego_info_under_slot.fix_slot = true;
    }
  }

  // fix slot
  double fix_slot_ratio = param.fix_slot_occupied_ratio;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
      ApaStateMachine::ACTIVE_IN_CAR_FRONT) {
    fix_slot_ratio = param.headin_fix_slot_occupied_ratio;
  }

  if (ego_info_under_slot.slot_occupied_ratio > fix_slot_ratio &&
      !ego_info_under_slot.fix_slot && measures_ptr->GetStaticFlag()) {
    ego_info_under_slot.fix_slot = true;
    ILOG_INFO << "fix_slot";
  }

  return true;
}

NarrowSpaceScenario::~NarrowSpaceScenario() {}

void NarrowSpaceScenario::PathShrinkBySlotLimiter() {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_FRONT &&
      current_gear_ != AstarPathGear::DRIVE) {
    return;
  }

  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_REAR &&
      current_gear_ != AstarPathGear::REVERSE) {
    return;
  }

  if (current_path_point_global_vec_.size() <= 1) {
    return;
  }

  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  // path is not pass limiter, return
  double limiter_x = ego_info.target_pose.pos[0];
  Eigen::Vector2d path_end_global = current_path_point_global_vec_.back().pos;
  Eigen::Vector2d point_local;
  point_local = ego_info.g2l_tf.GetPos(path_end_global);

  ILOG_INFO << "target point x = " << limiter_x
            << ", path end x = " << point_local[0]
            << ", path end y=" << point_local[1];

  if (point_local[0] >= limiter_x) {
    return;
  }

  // car pose has big distance with limiter, return
  double x_diff = std::fabs(ego_info.terminal_err.pos.x());
  double y_diff = std::fabs(ego_info.terminal_err.pos.y());
  if (x_diff > 2.0 || y_diff > 2.0) {
    return;
  }

  // shrink path
  size_t path_size = current_path_point_global_vec_.size();
  for (size_t i = 0; i < path_size; i++) {
    Eigen::Vector2d& point_global = current_path_point_global_vec_.back().pos;
    point_local = ego_info.g2l_tf.GetPos(point_global);
    if (point_local[0] >= limiter_x + 0.1) {
      break;
    }

    current_path_point_global_vec_.pop_back();
  }

  return;
}

void NarrowSpaceScenario::PathExpansionBySlotLimiter() {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_FRONT &&
      current_gear_ != AstarPathGear::DRIVE) {
    return;
  }

  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_REAR &&
      current_gear_ != AstarPathGear::REVERSE) {
    return;
  }

  if (current_path_point_global_vec_.size() <= 2) {
    return;
  }

  // If path is not linked with goal, do not expand.
  if (!is_path_connected_to_goal_) {
    return;
  }

  // path is near limiter, return.
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;
  Eigen::Vector2d path_end_global = current_path_point_global_vec_.back().pos;
  Eigen::Vector2d point_local;
  point_local = ego_info.g2l_tf.GetPos(path_end_global);
  double limiter_x = ego_info.target_pose.pos.x();
  if (point_local[0] <= (limiter_x + 0.1)) {
    return;
  }

  // car pose has big distance with limiter, return
  double ego_x_diff = std::fabs(ego_info.terminal_err.pos.x());
  double ego_y_diff = std::fabs(ego_info.terminal_err.pos.y());
  if (ego_x_diff > 1.5 || ego_y_diff > 0.5) {
    return;
  }

  // path end pose has big distance with limiter, return
  double x_diff = std::fabs(point_local[0] - ego_info.target_pose.pos.x());
  double y_diff = std::fabs(point_local[1] - ego_info.target_pose.pos.y());
  if (x_diff > 1.0 || y_diff > 0.5) {
    return;
  }

  double dist_to_goal = x_diff;
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
  double phi = current_path_point_global_vec_.back().heading;
  while (s < dist_to_goal) {
    point = path_end_global + s * unit_line_vec;

    global_point.Set(point, phi);
    global_point.kappa = 0.0;

    current_path_point_global_vec_.push_back(global_point);

    s += ds;
  }

  return;
}

const bool NarrowSpaceScenario::CheckEgoReplanNumber(const bool is_replan) {
  if (is_replan) {
    // check total plan number
    if (frame_.total_plan_count >
        apa_param.GetParam().headin_max_replan_count) {
      return false;
    }
  }

  // check plan number in slot
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  const ApaStateMachine fsm =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine();

  if (is_replan && ego_info.slot_occupied_ratio > 0.2) {
    if (in_slot_car_adjust_count_ >=
        apa_param.GetParam().in_slot_car_adjust_max_count) {
      return false;
    }
  }

  return true;
}

const bool NarrowSpaceScenario::IsEgoNeedAdjustInSlot(const Pose2D& ego_pose,
                                                      const double slot_width,
                                                      const double slot_len) {
  double ego_lat_offset = std::fabs(ego_pose.y);
  bool need_adjust_plan = false;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
      ApaStateMachine::ACTIVE_IN_CAR_FRONT) {
    if (current_gear_ != AstarPathGear::DRIVE) {
      return false;
    }
    EgoInfoUnderSlot& ego_info =
        apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;
    if (ego_pose.x <= slot_len) {
      if (ego_lat_offset < 1.0 &&
          std::fabs(ego_info.terminal_err.heading) < ifly_deg2rad(15.0)) {
        need_adjust_plan = true;
      }
    } else {
      if (ego_lat_offset < 1.0 &&
          std::fabs(ego_info.terminal_err.heading) < ifly_deg2rad(20.0)) {
        need_adjust_plan = true;
      }
    }
  } else if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
             ApaStateMachine::ACTIVE_IN_CAR_REAR) {
    if (current_gear_ != AstarPathGear::REVERSE) {
      return false;
    }
    // If ego is beyond slot and ego neayby center line, use spiral,rs,or
    // polynomial curve to adjust ego pose.
    // For astar searching, ego maybe switch gear too much or generate weird
    // path, we will improve it in the future for better parking performance.
    if (ego_pose.x >= slot_len) {
      if (ego_lat_offset < apa_param.GetParam()
                               .astar_config.adjust_ego_y_thresh_outside_slot &&
          std::fabs(ego_pose.theta) < ifly_deg2rad(5.0)) {
        need_adjust_plan = true;
      }
    } else {
      if (ego_lat_offset < 1.0 &&
          std::fabs(ego_pose.theta) < ifly_deg2rad(15.0)) {
        need_adjust_plan = true;
      }
    }
  }

  ILOG_INFO << "lateral dist = " << ego_lat_offset
            << ",theta = " << ifly_rad2deg(ego_pose.theta)
            << ",need_adjust_plan = " << need_adjust_plan;

  return need_adjust_plan;
}

const double NarrowSpaceScenario::CalRemainDistFromPath() {
  double remain_dist = 20.0;

  // no path
  size_t path_point_size = current_path_point_global_vec_.size();
  if (current_path_point_global_vec_.size() <= 1) {
    return 0.0;
  }

  const auto measures_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();
  Pose2D ego_pose(measures_ptr->GetPos()[0], measures_ptr->GetPos()[1],
                  measures_ptr->GetHeading());

  size_t nearest_point_id = 100000;
  nearest_point_id =
      GetNearestPathPoint(current_path_point_global_vec_, ego_pose);
  if (nearest_point_id >= path_point_size) {
    return 0.0;
  }

  // calc base vector
  pnc::geometry_lib::PathPoint* base_point;
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
    if (current_gear_ == AstarPathGear::DRIVE) {
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

  ILOG_INFO << "remain_dist = " << remain_dist
            << "  current_path_length = " << frame_.current_path_length;

  return remain_dist;
}

size_t NarrowSpaceScenario::GetNearestPathPoint(
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

void NarrowSpaceScenario::DebugPathString(
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  for (size_t i = 0; i < path.size(); i++) {
    ILOG_INFO << "i = " << i << ",x = " << path[i].pos.x()
              << ",y = " << path[i].pos.y();
  }
  return;
}

const bool NarrowSpaceScenario::UpdateParallelSlotInfo() {
  const auto measures_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;
  const ApaParameters& param = apa_param.GetParam();

  // note: slot points' order is corrected in slot management
  if (frame_.is_replan_first) {
    Pose2D vec02;
    vec02.x = ego_info.slot.processed_corner_coord_global_.pt_2.x() -
              ego_info.slot.processed_corner_coord_global_.pt_0.x();
    vec02.y = ego_info.slot.processed_corner_coord_global_.pt_2.y() -
              ego_info.slot.processed_corner_coord_global_.pt_0.y();

    Pose2D ego_vector;
    ego_vector.x = std::cos(measures_ptr->GetHeading());
    ego_vector.y = std::sin(measures_ptr->GetHeading());

    double cross = CrossProduct(ego_vector, vec02);
    if (cross > 0) {
      ego_info.slot_side = geometry_lib::SLOT_SIDE_LEFT;
    } else if (cross < 0) {
      ego_info.slot_side = geometry_lib::SLOT_SIDE_RIGHT;
    } else {
      ILOG_ERROR << "ego is vertical";
      ego_info.slot_side = geometry_lib::SLOT_SIDE_INVALID;
    }
  }

  if (ego_info.slot_side == geometry_lib::SLOT_SIDE_RIGHT) {
    ego_info.origin_pose_global.heading_vec =
        (ego_info.slot.processed_corner_coord_global_.pt_0 -
         ego_info.slot.processed_corner_coord_global_.pt_1)
            .normalized();

    ego_info.origin_pose_global.heading =
        std::atan2(ego_info.origin_pose_global.heading_vec.y(),
                   ego_info.origin_pose_global.heading_vec.x());

    ego_info.origin_pose_global.pos =
        (ego_info.slot.processed_corner_coord_global_.pt_1 +
         ego_info.slot.processed_corner_coord_global_.pt_3) /
        2;

  } else {
    ego_info.origin_pose_global.heading_vec =
        (ego_info.slot.processed_corner_coord_global_.pt_1 -
         ego_info.slot.processed_corner_coord_global_.pt_0)
            .normalized();

    ego_info.origin_pose_global.heading =
        std::atan2(ego_info.origin_pose_global.heading_vec.y(),
                   ego_info.origin_pose_global.heading_vec.x());

    ego_info.origin_pose_global.pos =
        (ego_info.slot.processed_corner_coord_global_.pt_0 +
         ego_info.slot.processed_corner_coord_global_.pt_2) /
        2;
  }

  ego_info.g2l_tf = geometry_lib::GlobalToLocalTf(
      ego_info.origin_pose_global.pos, ego_info.origin_pose_global.heading);

  ego_info.l2g_tf = geometry_lib::LocalToGlobalTf(
      ego_info.origin_pose_global.pos, ego_info.origin_pose_global.heading);

  ego_info.origin_pose_local.pos =
      ego_info.g2l_tf.GetPos(ego_info.origin_pose_global.pos);

  ego_info.origin_pose_local.heading =
      ego_info.g2l_tf.GetHeading(ego_info.origin_pose_global.heading);

  ego_info.origin_pose_local.heading_vec =
      geometry_lib::GenHeadingVec(ego_info.origin_pose_local.heading);

  ego_info.slot.TransformCoordFromGlobalToLocal(ego_info.g2l_tf);

  ego_info.cur_pose.pos = ego_info.g2l_tf.GetPos(measures_ptr->GetPos());
  ego_info.cur_pose.heading =
      ego_info.g2l_tf.GetHeading(measures_ptr->GetHeading());
  ego_info.cur_pose.heading_vec =
      geometry_lib::GenHeadingVec(ego_info.cur_pose.heading);

  // 计算停车位置
  double virtual_tar_x = 0.0;
  if (ego_info.slot.limiter_.valid) {
    // 根据原始限位器点计算停车终点
    Eigen::Vector2d pt1 =
        ego_info.g2l_tf.GetPos(ego_info.slot.limiter_.start_pt);
    Eigen::Vector2d pt2 = ego_info.g2l_tf.GetPos(ego_info.slot.limiter_.end_pt);

    virtual_tar_x = 0.5 * (pt1 + pt2).x() + param.limiter_move_dist;
  } else {
    const double terminal_x =
        0.5 * (ego_info.slot.GetLength() - param.car_length) +
        param.rear_overhanging;
    virtual_tar_x = terminal_x;
  }

  ego_info.virtual_limiter.first.x() = virtual_tar_x;
  ego_info.virtual_limiter.second.x() = virtual_tar_x;
  ego_info.virtual_limiter.first.y() = 0.5 * ego_info.slot.slot_width_;
  ego_info.virtual_limiter.second.y() = -0.5 * ego_info.slot.slot_width_;

  // 后续横向终点位置会随着障碍物而进行改变
  ego_info.target_pose.pos << ego_info.virtual_limiter.first.x(),
      param.terminal_target_y;
  ego_info.target_pose.heading = param.terminal_target_heading;
  ego_info.target_pose.heading_vec = Eigen::Vector2d(1, 0);

  // calc terminal error once
  ego_info.terminal_err.Set(
      ego_info.cur_pose.pos - ego_info.target_pose.pos,
      ego_info.cur_pose.heading - ego_info.target_pose.heading);

  // calc slot occupied ratio
  double slot_occupied_ratio = 0.0;
  if (pnc::mathlib::IsInBound(ego_info.terminal_err.pos.x(), -3.0, 4.0)) {
    const double y_err_ratio =
        ego_info.terminal_err.pos.y() / (0.5 * ego_info.slot.GetWidth());

    if (ego_info.slot_side == geometry_lib::SLOT_SIDE_RIGHT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }

  return true;
}

const bool NarrowSpaceScenario::CheckParallelSlotFinished() {
  const ApaParameters& config = apa_param.GetParam();
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  const bool rear_axis_lon_condition =
      ego_info.terminal_err.pos.x() <
      config.astar_config.parallel_finish_lon_err;

  const double rear_axis_lat_offset = ego_info.cur_pose.pos.y();
  const double veh_head_lat_offset =
      (ego_info.cur_pose.pos + (config.wheel_base + config.front_overhanging) *
                                   ego_info.cur_pose.heading_vec)
          .y();

  const bool ego_center_lat_condition =
      std::fabs(rear_axis_lat_offset) <=
      config.astar_config.parallel_finish_center_lat_err;

  const bool ego_head_lat_condition =
      std::fabs(veh_head_lat_offset) <=
      config.astar_config.parallel_finish_head_lat_err;

  const bool heading_condition =
      std::fabs(ego_info.terminal_err.heading) <=
      config.astar_config.parallel_finish_heading_err * kDeg2Rad;

  const bool lat_condition =
      ego_center_lat_condition && ego_head_lat_condition && heading_condition;

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const bool remain_s_condition =
      frame_.remain_dist_path < config.max_replan_remain_dist;

  bool parking_finish = rear_axis_lon_condition && lat_condition &&
                        static_condition && remain_s_condition;

  return parking_finish;
}

void NarrowSpaceScenario::ScenarioTry() {
  if (!apa_param.GetParam()
           .astar_config.perpendicular_slot_auto_switch_to_astar) {
    return;
  }

  auto& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  if (ego_info_under_slot.slot.slot_type_ != SlotType::PERPENDICULAR) {
    return;
  }

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ego_info_under_slot.slot.release_info_
        .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
        SlotReleaseState::NOT_RELEASE;
    return;
  }

  narrow_space_decider_.Process(ego_info_under_slot.slot_type);

  PathPlannerResult res = PathPlannerResult::WAIT_PATH;
  bool has_response = UpdateThreadPath();

  if (narrow_space_decider_.IsNeedAstar() &&
      narrow_space_decider_.GetAstarState() == AstarSearchState::NONE) {
    narrow_space_decider_.SetAstarState(AstarSearchState::SEARCHING);
    res = PlanBySearchBasedMethod(true);
  } else if (narrow_space_decider_.IsNeedAstar() && has_response) {
    res = PlanBySearchBasedMethod(true);
  }

  if (res == PathPlannerResult::PLAN_FAILED) {
    narrow_space_decider_.SetAstarState(AstarSearchState::FAILURE);

    ego_info_under_slot.slot.release_info_
        .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
        SlotReleaseState::NOT_RELEASE;

    ILOG_INFO << "astar path try fail";

    return;
  } else if (res == PathPlannerResult::PLAN_UPDATE) {
    narrow_space_decider_.SetAstarState(AstarSearchState::SUCCESS);

    ego_info_under_slot.slot.release_info_
        .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
        SlotReleaseState::RELEASE;

    ILOG_INFO << "hybrid astar path try success";

    return;
  } else if (res == PathPlannerResult::WAIT_PATH) {
    SlotReleaseState astar_release_state =
        ego_info_under_slot.slot.release_info_
            .release_state[ASTAR_PLANNING_RELEASE];
    // 如果上一帧A星释放车位，在当前帧结果还没有出来时，使用上一帧的结果填充.
    if (astar_release_state == SlotReleaseState::RELEASE) {
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
          SlotReleaseState::RELEASE;
    }
  }

  return;
}

void NarrowSpaceScenario::ThreadClear() {
  thread_.Clear();
  return;
}

const bool NarrowSpaceScenario::IsVehicleOverlapWithSlotLine(
    const double slot_length, const double slot_width,
    const Pose2D& ego_start) {
  const apa_planner::ApaParameters& config = apa_param.GetParam();
  Polygon2D local_polygon;
  Polygon2D ego_global_polygon;

  double lat_buffer = 0.03;
  GenerateUpLeftFrameBox(&local_polygon, -config.rear_overhanging,
                         -config.car_width / 2 - lat_buffer,
                         config.car_length - config.rear_overhanging,
                         config.car_width / 2 + lat_buffer);
  ULFLocalPolygonToGlobal(&ego_global_polygon, &local_polygon, ego_start);

  // slot polygon
  Polygon2D slot_left_line;
  GenerateLineSegmentPolygon(&slot_left_line,
                             Position2D(slot_length, slot_width / 2),
                             Position2D(0, slot_width / 2));

  Polygon2D slot_right_line;
  GenerateLineSegmentPolygon(&slot_right_line,
                             Position2D(slot_length, -slot_width / 2),
                             Position2D(0, -slot_width / 2));

  bool is_collision;
  GJK2DInterface gjk;
  gjk.PolygonCollisionByCircleCheck(&is_collision, &ego_global_polygon,
                                    &slot_left_line, 0.1);
  if (is_collision) {
    ILOG_INFO << "collision";
    return true;
  }

  gjk.PolygonCollisionByCircleCheck(&is_collision, &ego_global_polygon,
                                    &slot_right_line, 0.1);
  if (is_collision) {
    ILOG_INFO << "collision";
    return true;
  }
  return false;
}

}  // namespace apa_planner
}  // namespace planning