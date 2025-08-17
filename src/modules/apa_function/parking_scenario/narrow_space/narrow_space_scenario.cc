#include "narrow_space_scenario.h"

#include <math.h>

#include <cmath>
#include <cstddef>
#include <cstdio>

#include "aabb2d.h"
#include "apa_slot.h"
#include "apa_trajectory_stitcher.h"
#include "collision_detection/path_safe_checker.h"
#include "common.pb.h"
#include "common_c.h"
#include "geometry_math.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "hybrid_astar_response.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math/math_utils.h"
#include "math/vec2d.h"
#include "math_utils.h"
#include "narrow_space_decider.h"
#include "parking_scenario.h"
#include "point_cloud_obstacle.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "spiral_typedefs.h"
#include "transform2d.h"
#include "utils_math.h"

namespace planning {
namespace apa_planner {

NarrowSpaceScenario::NarrowSpaceScenario(
    const std::shared_ptr<ApaWorld>& apa_world_ptr)
    : ParkingScenario(apa_world_ptr) {
  Init();
}

void NarrowSpaceScenario::Reset() {
  ParkingScenario::Reset();

  current_gear_ = AstarPathGear::PARKING;
  replan_number_inside_slot_ = 0;
  is_path_connected_to_goal_ = false;
  path_planning_fail_num_ = 0;
  lateral_offset_ = 0;
  lon_offset_ = 0;

  current_path_last_heading_ = 0.0;
  dynamic_flag_head_out_ = false;
  count_frame_from_last_dynamic_ = 100;

  narrow_space_decider_.Reset();
  virtual_wall_decider_.Reset(Pose2D(0, 0, 0));

  return;
}

void NarrowSpaceScenario::Init() {
  const ApaParameters& params = apa_param.GetParam();

  // todo, system should use same vehicle parameter configuration file and
  // data structure.
  ILOG_INFO << "init astar thread";

  thread_.Init(params.rear_overhanging, params.car_length, params.car_width,
               params.steer_ratio, params.wheel_base, params.min_turn_radius,
               (params.max_car_width - params.car_width) * 0.5);

  thread_.Start();
  response_.Clear();

  return;
}

const bool NarrowSpaceScenario::CheckFinished() {
  bool ret = false;

  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkOutStatus()) {
    return CheckHeadOutFinished();
  }

  if (apa_world_ptr_->GetSlotManagerPtr()
          ->GetEgoInfoUnderSlot()
          .slot.slot_type_ == SlotType::PARALLEL) {
    ret = CheckParallelSlotFinished();
  } else {
    ret = CheckVerticalSlotFinished();
  }

  return ret;
}

const bool NarrowSpaceScenario::CheckVerticalSlotFinished() {
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  const ApaParameters& config = apa_param.GetParam();

  const bool lon_condition =
      ego_info.terminal_err.pos.x() < config.finish_lon_err;

  const double lat_offset =
      std::fabs(ego_info.cur_pose.pos.y() - lateral_offset_);

  const double ego_head_lat_offset = std::fabs(
      (ego_info.cur_pose.pos + (config.wheel_base + config.front_overhanging) *
                                   ego_info.cur_pose.heading_vec)
          .y() -
      lateral_offset_);

  const bool ego_center_lat_condition =
      std::fabs(lat_offset) <= apa_param.GetParam().finish_lat_err_strict;

  const bool ego_head_lat_condition =
      std::fabs(ego_head_lat_offset) <= apa_param.GetParam().finish_lat_err;

  double heading_thresh = apa_param.GetParam().finish_heading_err;
  const ApaStateMachine fsm =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine();
  if (fsm == ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
      fsm == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT) {
    heading_thresh =
        apa_param.GetParam().astar_config.head_in_finish_heading_err;
  }
  const bool heading_condition_1 =
      std::fabs(ego_info.terminal_err.heading) <= heading_thresh * kDeg2Rad;

  const bool lat_condition =
      ego_center_lat_condition && heading_condition_1 && ego_head_lat_condition;

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
      apa_world_ptr_->GetColDetInterfacePtr()->GetUssObsAvoidancePtr();
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

  return false;
}

const bool NarrowSpaceScenario::CheckHeadOutFinished() {
  const ApaParkOutDirection& park_out_direction =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection();

  bool parking_finish = false;
  const EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  const double& target_heading_deg_head_out =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().slot.angle_;

  constexpr double kTargetHeadingThreshold = 5.0;

  const bool heading_condition_1 =
      std::fabs(ego_info.cur_pose.heading) <=
      (target_heading_deg_head_out + kTargetHeadingThreshold) *
          kDeg2Rad;  // TODU::

  const bool heading_condition_2 =
      std::fabs(ego_info.cur_pose.heading) >=
      (target_heading_deg_head_out - kTargetHeadingThreshold) * kDeg2Rad;

  const bool lat_condition = heading_condition_1 && heading_condition_2;

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const bool remain_s_condition =
      frame_.remain_dist_path < apa_param.GetParam().max_replan_remain_dist;

  const bool pos_condition =
      std::fabs(ego_info.cur_pose.pos.x() - ego_info.target_pose.pos.x()) < 0.5;

  switch (park_out_direction) {
    case ApaParkOutDirection::LEFT_FRONT:
    case ApaParkOutDirection::RIGHT_FRONT:
    case ApaParkOutDirection::LEFT_REAR:
    case ApaParkOutDirection::RIGHT_REAR:
      parking_finish = lat_condition && static_condition && remain_s_condition;
      break;
    case ApaParkOutDirection::FRONT:
    case ApaParkOutDirection::REAR:
      parking_finish =
          (remain_s_condition && static_condition && pos_condition) ||
          (ego_info.slot_occupied_ratio < 0.3 && static_condition);
      break;

    default:
      break;
  }

  if (parking_finish) {
    return true;
  }

  return parking_finish;
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
  frame_.remain_dist_obs = CalRemainDistFromObs(0.3);

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

  CheckReplanParams replan_params(
      apa_param.GetParam().max_replan_remain_dist, 0.068,
      apa_param.GetParam().max_replan_remain_dist,
      apa_param.GetParam().astar_config.deadend_uss_stuck_replan_wait_time,
      apa_param.GetParam().max_replan_remain_dist, 0.168,
      apa_param.GetParam().stuck_replan_time);

  bool is_replan = CheckReplan(replan_params);

  if (!CheckEgoReplanNumber(is_replan)) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = PLAN_COUNT_EXCEED_LIMIT;
    return;
  }

  bool update_thread_path = UpdateThreadPath();
  PathPlannerResult path_plan_result = PathPlannerResult::PLAN_FAILED;

  ILOG_INFO << "stuck_uss_time = " << frame_.stuck_obs_time
            << " ,is_replan = " << is_replan;

  if (is_replan) {
    dynamic_flag_head_out_ =
        (frame_.replan_reason == ReplanReason::DYNAMIC) ? true : false;
  }
  count_frame_from_last_dynamic_ =
      (frame_.replan_reason == ReplanReason::DYNAMIC)
          ? 0
          : count_frame_from_last_dynamic_ + 1;

  // check replan
  if (is_replan || update_thread_path) {
    ILOG_INFO << "plan reason = " << GetRePlanReasonString(frame_.replan_reason)
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
          frame_.plan_fail_reason = PATH_PLAN_FAILED;
        }
        break;
      case PathPlannerResult::PLAN_FAILED:
        if (frame_.replan_fail_time >
            apa_param.GetParam().max_replan_failed_time) {
          SetParkingStatus(PARKING_FAILED);
          frame_.plan_fail_reason = PATH_PLAN_FAILED;
        }
        break;
      default:
        SetParkingStatus(PARKING_RUNNING);
        break;
    }
  } else {
    SetParkingStatus(PARKING_RUNNING);
    HybridAstarDebugInfoClear();
    // ILOG_INFO << "use history path";
  }

  // DebugPathString(current_path_point_global_vec_);

  return;
}

void NarrowSpaceScenario::Log() const {
  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag);

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
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
                              origin_corner_coord_global.pt_01_unit_vec;
  pt_vec[1] = pt_vec[1] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_01_unit_vec;
  pt_vec[2] = pt_vec[2] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_23_unit_vec;
  pt_vec[3] = pt_vec[3] + ego_info_under_slot.move_slot_dist *
                              origin_corner_coord_global.pt_23_unit_vec;

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
  JSON_DEBUG_VALUE("current_path_length", frame_.current_path_length)
  JSON_DEBUG_VALUE("path_plan_success", frame_.plan_stm.path_plan_success)
  JSON_DEBUG_VALUE("planning_status", frame_.plan_stm.planning_status)
  JSON_DEBUG_VALUE("spline_success", frame_.spline_success)
  JSON_DEBUG_VALUE("remain_dist", frame_.remain_dist_path)
  JSON_DEBUG_VALUE("remain_dist_obs", frame_.remain_dist_obs)
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
      apa_world_ptr_->GetColDetInterfacePtr()
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

  return;
}

const bool NarrowSpaceScenario::GenTlane() { return true; }

const bool NarrowSpaceScenario::GenObstacles() { return true; }

const uint8_t NarrowSpaceScenario::PathPlanOnce() { return false; }

PathPlannerResult NarrowSpaceScenario::PlanBySearchBasedMethod(
    const bool is_scenario_try) {
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  PathPlannerResult res = PathPlannerResult::WAIT_PATH;

  // start
  Pose2D start = GenerateStitchPoint();

  // real target pose in slot
  Pose2D real_end;
  real_end.x = ego_info.target_pose.pos[0];
  real_end.y = ego_info.target_pose.pos[1];
  real_end.theta = ego_info.target_pose.heading;

  ParkSpaceType slot_type = GetSlotType(ego_info.slot.slot_type_);
  ParkingVehDirection parking_dir_type = GetDirection();
  double end_straight_len = GetStraightLength(parking_dir_type, slot_type);
  // astar end, maybe different with real end.
  Pose2D end = real_end;
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

  float passage_height =
      ego_info.slot_occupied_ratio >
              apa_param.GetParam().pose_slot_occupied_ratio_2
          ? 7.0f
          : apa_param.GetParam()
                .astar_config.vertical_slot_passage_height_bound;
  virtual_wall_decider_.Process(
      obs.virtual_obs, static_cast<float>(ego_info.slot.slot_width_),
      static_cast<float>(ego_info.slot.slot_length_), start, real_end,
      slot_type, ego_info.slot_side, parking_dir_type, passage_height);

  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_info.g2l_tf);

  PointCloudObstacleTransform obstacle_generator;
  cdl::AABB slot_box;
  if (NeedBlindZonePlanning(ego_info)) {
    slot_box = GenerateBlindZoneSlotBox(ego_info);

    obstacle_generator.GenerateLocalObstacle(
        apa_world_ptr_->GetObstacleManagerPtr(), obs, start, slot_box, true);
  } else {
    obstacle_generator.GenerateLocalObstacle(
        apa_world_ptr_->GetObstacleManagerPtr(), obs, start, slot_box, false);
  }

  // set input
  AstarRequest cur_request;

  cur_request.first_action_request.has_request = true;
  cur_request.first_action_request.gear_request = AstarPathGear::NONE;
  cur_request.space_type = slot_type;
  cur_request.direction_request = parking_dir_type;
  cur_request.rs_request = RSPathRequestType::NONE;
  cur_request.timestamp_ms = astar_start_time;
  cur_request.slot_id = ego_info.id;

  cur_request.start_ = Pose2f(start.x, start.y, start.theta);
  cur_request.base_pose_ = slot_base_pose;
  cur_request.real_goal = Pose2f(real_end.x, real_end.y, real_end.theta);

  cur_request.slot_width = ego_info.slot.GetWidth();
  cur_request.slot_length = ego_info.slot.GetLength();
  cur_request.history_gear = current_gear_;
  frame_.current_gear = GetGear(current_gear_);
  cur_request.goal_ = Pose2f(end.x, end.y, end.theta);
  FillPlanningReason(cur_request);

  // generate request
  FillPlanningMethod(is_scenario_try, cur_request);

  // gear need be different with history in next replanning
  FillGearRequest(is_scenario_try, cur_request);

  // publish result
  if (is_scenario_try) {
    res = PubResponseForScenarioTry(ego_info, cur_request, obs);
  } else {
    res = PubResponseForScenarioRunning(ego_info, cur_request, obs);
  }

  // DebugPathString(current_path_point_global_vec_);

  return res;
}

const int NarrowSpaceScenario::PublishHybridAstarDebugInfo(
    const HybridAStarResult& result, Transform2d* tf) {
  if (result.x.size() < 1) {
    ILOG_INFO << "no path";
    return 0;
  }

  Pose2D local_position;
  Pose2D global_position;

  geometry_lib::PathPoint gl_pt;
  complete_path_point_global_vec_.clear();
  complete_path_point_global_vec_.reserve(result.x.size());

  for (size_t i = 0; i < result.x.size(); i++) {
    if (apa_world_ptr_->GetStateMachineManagerPtr()->IsHeadOutStatus() &&
        i > 0) {
      if (IsNeedClipping(result, i)) {
        break;
      }
    }

    local_position.x = result.x[i];
    local_position.y = result.y[i];
    local_position.theta = result.phi[i];

    tf->ULFLocalPoseToGlobal(&global_position, local_position);

    gl_pt.pos << global_position.x, global_position.y;
    gl_pt.heading = global_position.theta;
    gl_pt.lat_buffer = 0.0;
    gl_pt.kappa = result.kappa[i];
    gl_pt.s = result.accumulated_s[i];
    gl_pt.gear = GetGear(result.gear[i]);

    if (IsSearchNode(result.type[i])) {
      gl_pt.type = 0;
    } else if (result.type[i] == AstarPathType::LINE_SEGMENT) {
      gl_pt.type = 1;
    } else {
      gl_pt.type = 2;
    }
    complete_path_point_global_vec_.emplace_back(gl_pt);
  }

  // do not publish it.
#if 0
  RecordSearchNode(tf);
#endif

  return 0;
}

void NarrowSpaceScenario::RecordSearchNode(Transform2d* tf) {
  auto& debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::AstarNodeList* list =
      debug->mutable_apa_path_debug()->mutable_astar_node_list();
  list->Clear();

  thread_.GetNodeListMessagePublish(list);
  ILOG_INFO << "list size " << list->nodes_size();

  Pose2D local_position;
  Pose2D global_position;
  for (int i = 0; i < list->nodes_size(); i++) {
    for (int j = 0; j < list->nodes(i).path_point_size(); j++) {
      local_position.x = list->nodes(i).path_point(j).x();
      local_position.y = list->nodes(i).path_point(j).y();

      tf->ULFLocalPoseToGlobal(&global_position, local_position);

      list->mutable_nodes(i)->mutable_path_point(j)->set_x(global_position.x);
      list->mutable_nodes(i)->mutable_path_point(j)->set_y(global_position.y);
    }
  }

  return;
}

const int NarrowSpaceScenario::HybridAstarDebugInfoClear() {
  auto& debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::AstarNodeList* list =
      debug->mutable_apa_path_debug()->mutable_astar_node_list();
  list->Clear();

  return 0;
}

const void NarrowSpaceScenario::GenerateFallBackPath() {
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  if (frame_.replan_reason == ReplanReason::DYNAMIC) {
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

const int NarrowSpaceScenario::PathOptimizationByCILQR(
    const std::vector<AStarPathPoint>& first_seg_path, Transform2d* tf) {
  std::vector<pnc::geometry_lib::PathPoint> local_path;
  local_path.reserve(first_seg_path.size());

  constexpr float kHeadHeadingStartDeg = 80.0f;
  constexpr float kTailHeadingStartDeg = 95.0f;
  constexpr float kHeadingEndDeg = 89.9f;
  constexpr float kHeadingDiffThresh = 1e-3f;
  constexpr float kRad2Deg = 180.0f / static_cast<float>(M_PI);

  bool heading_flag = true;
  bool sample_finish = false;

  pnc::geometry_lib::PathPoint point;

  for (size_t i = 0; i < first_seg_path.size(); ++i) {
    const AStarPathPoint& path_pt = first_seg_path[i];
    point = pnc::geometry_lib::PathPoint(Eigen::Vector2d(path_pt.x, path_pt.y),
                                         path_pt.phi, path_pt.kappa);
    point.s = path_pt.accumulated_s;
    point.gear = GetGear(path_pt.gear);

    if (apa_world_ptr_->GetStateMachineManagerPtr()->IsHeadOutStatus()) {
      const float heading_deg = std::abs(path_pt.phi * kRad2Deg);

      if (heading_deg > kHeadHeadingStartDeg && i > 0) {
        float heading_diff = path_pt.phi - first_seg_path[i - 1].phi;
        heading_flag = std::abs(heading_diff) > kHeadingDiffThresh;
      }

      if (std::abs(path_pt.phi) * kRad2Deg <= kHeadingEndDeg &&
          !sample_finish && heading_flag) {
        local_path.emplace_back(point);
      } else {
        sample_finish = true;
      }

    } else if (apa_world_ptr_->GetStateMachineManagerPtr()->IsTailOutStatus()) {
      const float heading_deg = std::abs(path_pt.phi * kRad2Deg);

      if (heading_deg < kHeadHeadingStartDeg && i > 0) {
        float heading_diff = path_pt.phi - first_seg_path[i - 1].phi;
        heading_flag = std::abs(heading_diff) > kHeadingDiffThresh;
      }

      if (heading_deg >= kHeadingEndDeg && !sample_finish && heading_flag) {
        local_path.emplace_back(point);
      } else {
        sample_finish = true;
      }
    } else {
      local_path.emplace_back(point);
    }
  }

  if (local_path.size() > 0) {
    current_path_last_heading_ = local_path.back().heading;
  }

  LocalPathToGlobal(local_path, tf);
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

  ILOG_INFO << " path point num " << current_path_point_global_vec_.size();
  return 0;
}

const bool NarrowSpaceScenario::UpdateThreadPath() {
  thread_.GetThreadState(&thread_state_);

  // ILOG_INFO << "thread state " << static_cast<int>(thread_state_);

  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    return true;
  }

  return false;
}

const bool NarrowSpaceScenario::UpdateEgoSlotInfo() {
  bool ret = false;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkOutStatus()) {
    ret = UpdateVerticalOutSlotInfo();
  } else {
    if (apa_world_ptr_->GetSlotManagerPtr()
            ->GetMutableEgoInfoUnderSlot()
            .slot.slot_type_ == SlotType::PARALLEL) {
      ret = UpdateParallelSlotInfo();
    } else {
      ret = UpdateVerticalSlotInfo();
    }
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
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  ego_info_under_slot.origin_pose_global.heading_vec =
      ego_info_under_slot.slot.processed_corner_coord_global_
          .pt_23mid_01mid_unit_vec;

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
  const ApaStateMachine fsm =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine();
  if (fsm == ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
      fsm == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT) {
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
    if (fsm == ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
        fsm == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT) {
      const std::vector<double> x_tab = {
          ego_info_under_slot.target_pose.pos.x() - param.wheel_base -
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
  // ILOG_INFO << "slot_occupied_ratio = "
  //           << ego_info_under_slot.slot_occupied_ratio;

  if (((fsm == ApaStateMachine::ACTIVE_IN_CAR_REAR &&
        frame_.gear_command == geometry_lib::SEG_GEAR_REVERSE)) &&
      !ego_info_under_slot.fix_slot) {
    Eigen::Vector2d center = (ego_info_under_slot.virtual_limiter.first +
                              ego_info_under_slot.virtual_limiter.second) /
                             2.0;
    double dist = (center - ego_info_under_slot.cur_pose.pos).norm();
    if (dist < param.car_to_limiter_dis) {
      ILOG_INFO << "should correct path according limiter";
      ego_info_under_slot.fix_slot = true;
    }
  }

  // fix slot
  double fix_slot_ratio = param.fix_slot_occupied_ratio;
  if (fsm == ApaStateMachine::ACTIVE_IN_CAR_FRONT) {
    fix_slot_ratio = param.headin_fix_slot_occupied_ratio;
  }

  if (ego_info_under_slot.slot_occupied_ratio > fix_slot_ratio &&
      !ego_info_under_slot.fix_slot && measures_ptr->GetStaticFlag()) {
    ego_info_under_slot.fix_slot = true;
    ILOG_INFO << "fix_slot";
  }

  return true;
}

const bool NarrowSpaceScenario::UpdateVerticalOutSlotInfo() {
  const std::shared_ptr<ApaMeasureDataManager> measures_ptr =
      apa_world_ptr_->GetMeasureDataManagerPtr();

  const ApaParameters& param = apa_param.GetParam();
  frame_.replan_flag = false;

  // 建立车位坐标系 根据23角点或者限位器角点确定规划终点位姿
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  ego_info_under_slot.origin_pose_global.heading_vec =
      ego_info_under_slot.slot.processed_corner_coord_global_.pt_23mid_01mid_vec
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
    if (apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection() ==
        ApaParkOutDirection::RIGHT_FRONT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (apa_world_ptr_->GetStateMachineManagerPtr()
                   ->GetParkOutDirection() == ApaParkOutDirection::LEFT_FRONT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else if (apa_world_ptr_->GetStateMachineManagerPtr()
                   ->GetParkOutDirection() == ApaParkOutDirection::FRONT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_STRAIGHT;
    }
  }

  constexpr double kInitialTargetX = 7.0;
  constexpr double kInitialTargetY = 11.0;
  constexpr double kAlternateTargetX = 8.0;
  constexpr double kAlternateTargetY = 4.0;
  constexpr double kPositionThresholdX = 7.0;
  constexpr double kHeadingThresholdRad0 = 60.0 * M_PI / 180.0;
  constexpr double kHeadingThresholdRad1 = 130.0 * M_PI / 180.0;

  const double target_heading_rad_head_out =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().slot.angle_ *
      M_PI / 180.0;

  const ApaParkOutDirection park_out_direction =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection();

  switch (park_out_direction) {
    case ApaParkOutDirection::LEFT_FRONT:
      ego_info_under_slot.target_pose.pos << kInitialTargetX, kInitialTargetY;
      ego_info_under_slot.target_pose.heading = target_heading_rad_head_out;
      ego_info_under_slot.target_pose.heading_vec = Eigen::Vector2d(0, 1);

      // 特殊位置要对目标点进行特殊调整
      if (ego_info_under_slot.cur_pose.pos.x() < kInitialTargetX &&
          std::abs(ego_info_under_slot.cur_pose.heading) >
              kHeadingThresholdRad0) {
        ego_info_under_slot.target_pose.pos << kAlternateTargetX,
            kAlternateTargetY;
      }
      break;

    case ApaParkOutDirection::RIGHT_FRONT:
      ego_info_under_slot.target_pose.pos << kInitialTargetX, -kInitialTargetY;
      ego_info_under_slot.target_pose.heading = -target_heading_rad_head_out;
      ego_info_under_slot.target_pose.heading_vec = Eigen::Vector2d(0, -1);

      // 特殊位置要对目标点进行特殊调整
      if (ego_info_under_slot.cur_pose.pos.x() < kInitialTargetX &&
          std::abs(ego_info_under_slot.cur_pose.heading) >
              kHeadingThresholdRad0) {
        ego_info_under_slot.target_pose.pos << kAlternateTargetX,
            -kAlternateTargetY;
      }
      break;

    case ApaParkOutDirection::LEFT_REAR:
      ego_info_under_slot.target_pose.pos << kAlternateTargetX,
          kAlternateTargetY;
      ego_info_under_slot.target_pose.heading = target_heading_rad_head_out;
      ego_info_under_slot.target_pose.heading_vec = Eigen::Vector2d(0, -1);

      // 特殊位置要对目标点进行特殊调整
      if (std::abs(ego_info_under_slot.cur_pose.heading) <
          kHeadingThresholdRad1) {
        ego_info_under_slot.target_pose.pos << kAlternateTargetX,
            kAlternateTargetY - 1.0;
      }
      break;

    case ApaParkOutDirection::RIGHT_REAR:
      ego_info_under_slot.target_pose.pos << kAlternateTargetX,
          -kAlternateTargetY;
      ego_info_under_slot.target_pose.heading = -target_heading_rad_head_out;
      ego_info_under_slot.target_pose.heading_vec = Eigen::Vector2d(0, -1);

      // 特殊位置要对目标点进行特殊调整
      if (std::abs(ego_info_under_slot.cur_pose.heading) <
          kHeadingThresholdRad1) {
        ego_info_under_slot.target_pose.pos << kAlternateTargetX,
            -kAlternateTargetY + 1.0;
      }
      break;

    case ApaParkOutDirection::REAR:
      ego_info_under_slot.target_pose.pos << kInitialTargetX + 1.0, 0.0;
      ego_info_under_slot.target_pose.heading = M_PI;
      ego_info_under_slot.target_pose.heading_vec = Eigen::Vector2d(0, -1);

      break;

    case ApaParkOutDirection::FRONT:
    default:
      ego_info_under_slot.target_pose.pos << kInitialTargetX - 2.5, 0.0;
      ego_info_under_slot.target_pose.heading = 0.0;
      ego_info_under_slot.target_pose.heading_vec = Eigen::Vector2d(0, 0);
      break;
  }

  // 终点误差
  ego_info_under_slot.terminal_err.Set(
      ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
      geometry_lib::NormalizeAngle(ego_info_under_slot.cur_pose.heading -
                                   ego_info_under_slot.target_pose.heading));

  // 固定车位,计算占库比
  ego_info_under_slot.slot_occupied_ratio = 0.0;

  if (std::fabs(ego_info_under_slot.cur_pose.pos.y()) <
      param.slot_occupied_ratio_max_lat_err) {
    double virtual_ego_pos_x = ego_info_under_slot.cur_pose.pos.x();
    double heading_err_slot_occupied =
        std::fabs(ego_info_under_slot.cur_pose.heading);

    if (apa_world_ptr_->GetStateMachineManagerPtr()->IsTailOutStatus()) {
      heading_err_slot_occupied =
          std::fabs(std::fabs(ego_info_under_slot.cur_pose.heading) - M_PI);
      virtual_ego_pos_x = ego_info_under_slot.cur_pose.pos.x() -
                          param.wheel_base * cos(heading_err_slot_occupied);
    }

    if (heading_err_slot_occupied <
            param.slot_occupied_ratio_max_heading_err * kDeg2Rad &&
        virtual_ego_pos_x < 6.5) {
      const double x_tab_0 = 1.0;
      const double x_tab_1 =
          ego_info_under_slot.slot.slot_length_ + param.rear_overhanging;
      const std::vector<double> x_tab = {x_tab_0, x_tab_1};
      const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
      ego_info_under_slot.slot_occupied_ratio =
          mathlib::Interp1(x_tab, occupied_ratio_tab, virtual_ego_pos_x);
    }
  }

  ego_info_under_slot.fix_slot = false;

  ILOG_INFO << "slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;

  return true;
}

NarrowSpaceScenario::~NarrowSpaceScenario() {}

void NarrowSpaceScenario::PathShrinkBySlotLimiter() {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkOutStatus()) {
    return;
  }
  const ApaStateMachine fsm =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine();
  if (fsm == ApaStateMachine::ACTIVE_IN_CAR_FRONT &&
      current_gear_ != AstarPathGear::DRIVE) {
    return;
  }

  if (fsm == ApaStateMachine::ACTIVE_IN_CAR_REAR &&
      current_gear_ != AstarPathGear::REVERSE) {
    return;
  }

  if (current_path_point_global_vec_.size() <= 1) {
    return;
  }

  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

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

  // If car pose has big distance with limiter, return;
  // If car pose is nearby of limiter (0.4 meter), do not shrink path to prevent
  // car speed change too much, return;
  double x_diff = std::fabs(ego_info.terminal_err.pos.x());
  double y_diff = std::fabs(ego_info.terminal_err.pos.y());
  if (x_diff > 2.0 || y_diff > 2.0 || x_diff < 0.4) {
    return;
  }

  // find nearby limiter point, but not cross limiter
  int path_size = current_path_point_global_vec_.size();
  int path_end_point_id = path_size;
  double dist_to_limiter = 0.0;

  for (int i = 0; i < path_size; i++) {
    const Eigen::Vector2d& point_global = current_path_point_global_vec_[i].pos;
    point_local = ego_info.g2l_tf.GetPos(point_global);
    dist_to_limiter = point_local[0] - limiter_x;
    path_end_point_id = i;
    if (dist_to_limiter < 0.0) {
      break;
    }
  }

  // delete cross limiter points
  if (path_end_point_id < path_size - 1 &&
      current_path_point_global_vec_.size() > 1) {
    const pnc::geometry_lib::PathPoint& path_end_point =
        current_path_point_global_vec_[path_end_point_id];

    point_local = ego_info.g2l_tf.GetPos(path_end_point.pos);
    dist_to_limiter = point_local[0] - limiter_x;

    // If point distance to limiter is big, add an extra point in limiter.
    if (dist_to_limiter > 0.01) {
      path_end_point_id++;

      pnc::geometry_lib::PathPoint& limit_point =
          current_path_point_global_vec_[path_end_point_id];
      // get local
      point_local = ego_info.g2l_tf.GetPos(limit_point.pos);
      point_local[0] = limiter_x;
      // change to global
      limit_point.pos = ego_info.l2g_tf.GetPos(point_local);
      limit_point.s =
          path_end_point.s + (limit_point.pos - path_end_point.pos).norm();
    }

    for (int i = path_end_point_id + 1; i < path_size; i++) {
      current_path_point_global_vec_.pop_back();
    }
  }

  return;
}

void NarrowSpaceScenario::PathExpansionBySlotLimiter() {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkOutStatus()) {
    return;
  }
  const ApaStateMachine fsm =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine();
  if (fsm == ApaStateMachine::ACTIVE_IN_CAR_FRONT &&
      current_gear_ != AstarPathGear::DRIVE) {
    return;
  }

  if (fsm == ApaStateMachine::ACTIVE_IN_CAR_REAR &&
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
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  Eigen::Vector2d path_end_global = current_path_point_global_vec_.back().pos;
  Eigen::Vector2d point_local;
  point_local = ego_info.g2l_tf.GetPos(path_end_global);
  double limiter_x = ego_info.target_pose.pos.x();
  if (point_local[0] <= (limiter_x + 0.02)) {
    return;
  }

  // car pose has big distance with limiter, return;
  // If car is nearby with limiter, such as 0.3 meter, do not extend path to
  // keep from speed change too much.
  double ego_x_diff = std::fabs(ego_info.terminal_err.pos.x());
  double ego_y_diff = std::fabs(ego_info.terminal_err.pos.y());
  if (ego_x_diff > 2.0 || ego_y_diff > 0.5 || ego_x_diff < 0.5) {
    return;
  }

  // path end pose has big distance with limiter, return
  double path_end_pt_to_limiter =
      std::fabs(point_local[0] - ego_info.target_pose.pos.x());
  double path_y_diff = std::fabs(point_local[1] - ego_info.target_pose.pos.y());
  if (path_end_pt_to_limiter > 1.5 || path_y_diff > 0.5) {
    return;
  }

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

  double s = 0.0;
  double ds = 0.1;

  Eigen::Vector2d point;
  pnc::geometry_lib::PathPoint global_point;
  double phi = current_path_point_global_vec_.back().heading;
  while (s < path_end_pt_to_limiter) {
    s += ds;
    s = std::min(s, path_end_pt_to_limiter);

    point = path_end_global + s * unit_line_vec;

    global_point.Set(point, phi);
    global_point.kappa = 0.0;

    current_path_point_global_vec_.push_back(global_point);
  }

  return;
}

const bool NarrowSpaceScenario::CheckEgoReplanNumber(const bool is_replan) {
  if (!is_replan) {
    return true;
  }

  // check total plan number
  if (frame_.total_plan_count >
      apa_param.GetParam().astar_config.max_replan_number) {
    return false;
  }

  // check plan number in slot
  const EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  if (ego_info.slot_occupied_ratio > 0.2 &&
      replan_number_inside_slot_ >=
          apa_param.GetParam().astar_config.max_replan_number_inside_slot) {
    return false;
  }

  return true;
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

  // ILOG_INFO << "remain_dist = " << remain_dist
  //           << "  current_path_length = " << frame_.current_path_length;

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

const bool NarrowSpaceScenario::UpdateParallelSlotInfo() {
  const auto measures_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
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
  const EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

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

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  if (ego_info_under_slot.slot.slot_type_ != SlotType::PERPENDICULAR &&
      ego_info_under_slot.slot.slot_type_ != SlotType::SLANT) {
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
  } else if (res == PathPlannerResult::PLAN_UPDATE) {
    narrow_space_decider_.SetAstarState(AstarSearchState::SUCCESS);

    ego_info_under_slot.slot.release_info_
        .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
        SlotReleaseState::RELEASE;

    ILOG_INFO << "hybrid astar path try success";
  } else if (res == PathPlannerResult::WAIT_PATH) {
    SlotReleaseState astar_release_state =
        ego_info_under_slot.slot.release_info_
            .release_state[ASTAR_PLANNING_RELEASE];
    // 如果上一帧A星释放车位，在当前帧结果还没有出来时，不改变上一帧结果;
    // 如果上一帧结果未知，使用计算中状态填充;
    if (astar_release_state == SlotReleaseState::UNKOWN) {
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
          SlotReleaseState::COMPUTING;
    }
  }

  TansformPreparePlanningTraj();

  return;
}

void NarrowSpaceScenario::ThreadClearState() {
  thread_.Clear();
  return;
}

const bool NarrowSpaceScenario::NeedBlindZonePlanning(
    const EgoInfoUnderSlot& ego_info) {
  if (!apa_param.GetParam().astar_config.enable_blind_zone) {
    return false;
  }

  if (path_planning_fail_num_ <= 0 || path_planning_fail_num_ >= 3) {
    return false;
  }

  double position_y_error = std::fabs(ego_info.cur_pose.pos[1]);
  double position_x_error = std::fabs(ego_info.cur_pose.pos[0]);
  if (position_y_error < ego_info.slot.slot_width_ / 2 &&
      (position_x_error > 0.0 &&
       position_x_error < ego_info.slot.slot_length_ + 1.0)) {
    return false;
  }

  return true;
}

const bool NarrowSpaceScenario::CheckDynamicUpdate() {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkOutStatus()) {
    return CheckDynamicHeadOut();
  } else {
    return ReplanBySlotRefresh();
  }
}

const bool NarrowSpaceScenario::ReplanBySlotRefresh() {
  const ApaParameters& param = apa_param.GetParam();
  const bool car_motion_flag =
      !apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  const bool car_pos_flag =
      ego_info_under_slot.cur_pose.pos.x() <
      (ego_info_under_slot.slot.GetOriginCornerCoordLocal().pt_01_mid.x() +
       3.68);

  const bool occupied_ratio_flag = (ego_info_under_slot.slot_occupied_ratio <
                                    param.pose_slot_occupied_ratio_3);

  // check path remain dist
  bool path_dist_flag = false;
  if (frame_.remain_dist_path > 1.5) {
    path_dist_flag = true;
  }

  const bool dynamic_replan_flag = car_motion_flag && car_pos_flag &&
                                   occupied_ratio_flag &&
                                   is_path_connected_to_goal_ && path_dist_flag;
  if (!dynamic_replan_flag) {
    return false;
  }

  // path end pose check
  bool lateral_offset_flag = false;
  bool theta_offset_flag = false;
  if (current_path_point_global_vec_.size() > 0) {
    double history_lat_offset = lateral_offset_;

    Eigen::Vector2d path_end_global = current_path_point_global_vec_.back().pos;
    Eigen::Vector2d point_local;
    point_local = ego_info_under_slot.g2l_tf.GetPos(path_end_global);
    double later_error = std::fabs(point_local.y() - lateral_offset_);
    if (later_error > 0.06) {
      lateral_offset_flag = true;
    }

    double path_heading = current_path_point_global_vec_.back().heading;
    double slot_heading = ego_info_under_slot.origin_pose_global.heading;
    const ApaStateMachine fsm =
        apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine();
    if (fsm == ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
        fsm == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT) {
      slot_heading -= M_PI;
    }
    double phi_error =
        ad_common::math::NormalizeAngle(path_heading - slot_heading);
    if (std::fabs(phi_error) > 0.026) {
      theta_offset_flag = true;
    }

    ILOG_INFO << "lat offset error = " << later_error
              << ", theta error = " << phi_error * kRad2Deg;
  }

  if (lateral_offset_flag || theta_offset_flag) {
    return true;
  }

  return false;
}

const bool NarrowSpaceScenario::CheckDynamicHeadOut() {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection() ==
      ApaParkOutDirection::FRONT) {
    // This direction does not require dynamic planning;
    return false;
  }
  const ApaParameters& param = apa_param.GetParam();
  const bool car_motion_flag =
      !apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  const bool car_pos_flag =
      ego_info_under_slot.cur_pose.pos.x() <
      (ego_info_under_slot.slot.GetOriginCornerCoordLocal().pt_01_mid.x() +
       3.68);

  const bool occupied_ratio_flag =
      (ego_info_under_slot.slot_occupied_ratio <
       param.pose_slot_occupied_ratio_3) &&
      (ego_info_under_slot.slot_occupied_ratio > 0.0);

  // check path remain dist
  const bool path_dist_flag = frame_.remain_dist_path > 1.5;

  const float perception_blind_spot_distance = 6.0;

  const bool current_path_length_flag =
      frame_.current_path_length > perception_blind_spot_distance;

  constexpr double kHeadingThreshold = 0.05;

  bool heading_flag =
      std::fabs(current_path_last_heading_ -
                ego_info_under_slot.target_pose.heading) < kHeadingThreshold;

  bool historical_condition = true;
  if (dynamic_flag_head_out_ && heading_flag) {
    // 如果上一次当前动态规划的heading 接近 目标heading，则无需再次重规划。
    historical_condition =
        frame_.remain_dist_path > perception_blind_spot_distance ? true : false;
  }

  bool count_frame_condition =
      count_frame_from_last_dynamic_ > 10 ? true : false;

  if (historical_condition) {
    bool dynamic_replan_flag =
        car_motion_flag && car_pos_flag && occupied_ratio_flag &&
        path_dist_flag && current_path_length_flag && count_frame_condition;

    return dynamic_replan_flag;
  } else {
    return historical_condition;
  }
}

void NarrowSpaceScenario::FillPlanningReason(AstarRequest& cur_request) {
  switch (frame_.replan_reason) {
    case ReplanReason::FIRST_PLAN:
      cur_request.plan_reason = PlanningReason::FIRST_PLAN;
      break;
    case ReplanReason::SEG_COMPLETED_PATH:
      cur_request.plan_reason = PlanningReason::PATH_COMPLETED;
      break;
    case ReplanReason::SEG_COMPLETED_OBS:
      cur_request.plan_reason = PlanningReason::PATH_STUCKED;
      break;
    case ReplanReason::STUCKED:
      cur_request.plan_reason = PlanningReason::PATH_STUCKED;
      break;
    case ReplanReason::DYNAMIC:
      cur_request.plan_reason = PlanningReason::SLOT_REFRESHED;
      break;
    case ReplanReason::SEG_COMPLETED_COL_DET:
      cur_request.plan_reason = PlanningReason::PATH_STUCKED;
      break;
    case ReplanReason::SLOT_CRUISING:
      cur_request.plan_reason = PlanningReason::SLOT_CRUISING;
      break;
    case ReplanReason::DYNAMIC_GEAR_SWITCH:
      cur_request.plan_reason = PlanningReason::DYNAMIC_GEAR_SWITCH;
      break;
    default:
      cur_request.plan_reason = PlanningReason::NONE;
      break;
  }

  return;
}

void NarrowSpaceScenario::FillGearRequest(const bool is_scenario_try,
                                          AstarRequest& cur_request) {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkOutStatus()) {
    if (frame_.replan_reason == FIRST_PLAN) {
      if (apa_world_ptr_->GetStateMachineManagerPtr()->IsHeadOutStatus()) {
        cur_request.first_action_request.gear_request = AstarPathGear::DRIVE;
      } else if (apa_world_ptr_->GetStateMachineManagerPtr()
                     ->IsTailOutStatus()) {
        cur_request.first_action_request.gear_request = AstarPathGear::REVERSE;
      }
    }
  }

  // gear need be different with history in next replanning
  if (frame_.replan_reason != ReplanReason::FIRST_PLAN &&
      frame_.replan_reason != ReplanReason::DYNAMIC) {
    switch (current_gear_) {
      case AstarPathGear::REVERSE:
        cur_request.first_action_request.gear_request = AstarPathGear::DRIVE;
        break;
      case AstarPathGear::DRIVE:
        cur_request.first_action_request.gear_request = AstarPathGear::REVERSE;
        break;
      default:
        cur_request.first_action_request.gear_request = AstarPathGear::NONE;
        break;
    }
  }

  if (frame_.replan_reason == ReplanReason::DYNAMIC) {
    cur_request.first_action_request.gear_request = current_gear_;
  }

  if (is_scenario_try) {
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
    }
  }
  if (apa_world_ptr_->GetSimuParam().enable_debug_swap_start_goal) {
    cur_request.swap_start_goal =
        apa_world_ptr_->GetSimuParam().swap_start_goal;
  }

  if (cur_request.swap_start_goal) {
    ClearFirstActionReqeust(&cur_request);
  }

  return;
}

const cdl::AABB NarrowSpaceScenario::GenerateBlindZoneSlotBox(
    const EgoInfoUnderSlot& ego_info) const {
  const SlotCoord slot_info = ego_info.slot.GetProcessedCornerCoordLocal();
  double y_buffer = 0.0;
  double y_bound = 0.0;
  if (path_planning_fail_num_ == 1) {
    y_buffer = 0.08;
  } else if (path_planning_fail_num_ == 2) {
    // obstacles invade slot too much, delete bigger range obstacle around
    // slot.
    y_buffer = 0.1;
  }

  y_bound = std::min(slot_info.pt_0.y(), slot_info.pt_2.y());
  y_bound = std::min(y_bound, -apa_param.GetParam().max_car_width / 2);

  cdl::AABB slot_box;
  slot_box.min_ = cdl::Vector2r(slot_info.pt_23_mid.x() - 0.01f,
                                static_cast<float>(y_bound - y_buffer));

  y_bound = std::max(slot_info.pt_1.y(), slot_info.pt_3.y());
  y_bound = std::max(y_bound, apa_param.GetParam().max_car_width / 2);
  slot_box.max_ = cdl::Vector2r(slot_info.pt_01_mid.x() + 4.0f,
                                static_cast<float>(y_bound + y_buffer));

  return slot_box;
}

void NarrowSpaceScenario::FillPlanningMethod(const bool is_scenario_try,
                                             AstarRequest& cur_request) {
  cur_request.path_generate_method =
      planning::AstarPathGenerateType::ASTAR_SEARCHING;

  if (frame_.replan_reason == ReplanReason::DYNAMIC) {
    switch (cur_request.direction_request) {
      case ParkingVehDirection::TAIL_IN:
        cur_request.path_generate_method =
            planning::AstarPathGenerateType::GEAR_REVERSE_SEARCHING;
        break;
      case ParkingVehDirection::HEAD_IN:
        cur_request.path_generate_method =
            planning::AstarPathGenerateType::GEAR_DRIVE_SEARCHING;
        break;
      case ParkingVehDirection::HEAD_OUT_TO_LEFT:
      case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
      case ParkingVehDirection::HEAD_OUT_TO_MIDDLE:
      case ParkingVehDirection::TAIL_OUT_TO_LEFT:
      case ParkingVehDirection::TAIL_OUT_TO_RIGHT:
      case ParkingVehDirection::TAIL_OUT_TO_MIDDLE:
        cur_request.path_generate_method =
            planning::AstarPathGenerateType::ASTAR_SEARCHING;
        break;
      default:
        cur_request.path_generate_method =
            planning::AstarPathGenerateType::ASTAR_SEARCHING;
        break;
    }
  }

  if (is_scenario_try) {
    cur_request.path_generate_method =
        planning::AstarPathGenerateType::TRY_SEARCHING;
  }

  return;
}

ParkSpaceType NarrowSpaceScenario::GetSlotType(const SlotType slot_type) {
  ParkSpaceType space_type;
  if (slot_type == SlotType::PARALLEL) {
    space_type = ParkSpaceType::PARALLEL;
  } else if (slot_type == SlotType::SLANT) {
    space_type = ParkSpaceType::SLANTING;
  } else {
    space_type = ParkSpaceType::VERTICAL;
  }

  return space_type;
}

ParkingVehDirection NarrowSpaceScenario::GetDirection() {
  const ApaStateMachine fsm =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine();

  ParkingVehDirection dir_type;

  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkOutStatus()) {
    switch (
        apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection()) {
      case ApaParkOutDirection::LEFT_FRONT:
        dir_type = ParkingVehDirection::HEAD_OUT_TO_LEFT;
        break;
      case ApaParkOutDirection::RIGHT_FRONT:
        dir_type = ParkingVehDirection::HEAD_OUT_TO_RIGHT;
        break;
      case ApaParkOutDirection::FRONT:
        dir_type = ParkingVehDirection::HEAD_OUT_TO_MIDDLE;
        break;
      case ApaParkOutDirection::LEFT_REAR:
        dir_type = ParkingVehDirection::TAIL_OUT_TO_LEFT;
        break;
      case ApaParkOutDirection::RIGHT_REAR:
        dir_type = ParkingVehDirection::TAIL_OUT_TO_RIGHT;
        break;
      case ApaParkOutDirection::REAR:
        dir_type = ParkingVehDirection::TAIL_OUT_TO_MIDDLE;
        break;
      default:
        dir_type = ParkingVehDirection::NONE;
        break;
    }
  } else {
    if (fsm == ApaStateMachine::ACTIVE_IN_CAR_REAR ||
        fsm == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR) {
      dir_type = ParkingVehDirection::TAIL_IN;
    } else {
      dir_type = ParkingVehDirection::HEAD_IN;
    }
  }

  return dir_type;
}

double NarrowSpaceScenario::GetStraightLength(const ParkingVehDirection dir,
                                              const ParkSpaceType slot_type) {
  double end_straight_len = 0.0;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkInStatus()) {
    if (slot_type == ParkSpaceType::PARALLEL) {
      end_straight_len =
          apa_param.GetParam().astar_config.parallel_slot_end_straight_dist;
    } else if (slot_type == ParkSpaceType::SLANTING) {
      end_straight_len =
          apa_param.GetParam().astar_config.vertical_tail_in_end_straight_dist;
    } else {
      if (dir == ParkingVehDirection::TAIL_IN) {
        end_straight_len = apa_param.GetParam()
                               .astar_config.vertical_tail_in_end_straight_dist;
      } else {
        end_straight_len = apa_param.GetParam()
                               .astar_config.vertical_head_in_end_straight_dist;
      }
    }
  }

  return end_straight_len;
}

Pose2D NarrowSpaceScenario::GenerateStitchPoint() {
  Pose2D start;
  EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  if (frame_.replan_reason == ReplanReason::DYNAMIC) {
    double acc = apa_world_ptr_->GetMeasureDataManagerPtr()->GetAcceleration();
    if (apa_world_ptr_->GetMeasureDataManagerPtr()->GetVel() < 0.0) {
      acc = -apa_world_ptr_->GetMeasureDataManagerPtr()->GetAcceleration();
    }
    SVPoint init_point = SVPoint(
        0, std::fabs(apa_world_ptr_->GetMeasureDataManagerPtr()->GetVel()),
        acc);

    ApaTrajectoryStitcher traj_stitcher;
    traj_stitcher.Execute(
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetPose(),
        current_path_point_global_vec_,
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetFrontWheelAngle(),
        init_point, 0.2, trajectory_,
        pnc::geometry_lib::GetGearType(frame_.current_gear),
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetGear());

    const pnc::geometry_lib::PathPoint& stitch_point =
        traj_stitcher.GetStitchPathPoint();
    Eigen::Vector2d local_pos = ego_info.g2l_tf.GetPos(stitch_point.pos);

    start = Pose2D(local_pos.x(), local_pos.y(),
                   ego_info.g2l_tf.GetHeading(stitch_point.heading));
  } else if (frame_.replan_reason == ReplanReason::DYNAMIC_GEAR_SWITCH) {
    if (!current_path_point_global_vec_.empty()) {
      const pnc::geometry_lib::PathPoint& back =
          current_path_point_global_vec_.back();

      Eigen::Vector2d local = ego_info.g2l_tf.GetPos(back.pos);
      start.x = local[0];
      start.y = local[1];
      start.theta = ego_info.g2l_tf.GetHeading(back.heading);
    } else {
      start.x = ego_info.cur_pose.pos[0];
      start.y = ego_info.cur_pose.pos[1];
      start.theta = ego_info.cur_pose.heading;
    }
  } else {
    start = Pose2D(ego_info.cur_pose.pos[0], ego_info.cur_pose.pos[1],
                   ego_info.cur_pose.heading);
  }

  return start;
}

const PathPlannerResult NarrowSpaceScenario::PubResponseForScenarioRunning(
    const EgoInfoUnderSlot& ego_info, const AstarRequest& cur_request,
    const ParkObstacleList& obs) {
  PathPlannerResult res = PathPlannerResult::WAIT_PATH;
  response_.Clear();

  // check result
  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    // get output
    thread_.PublishResponse(&response_);
    // ILOG_INFO << "publish path";

    if (!IsResponseNice(cur_request, response_)) {
      ThreadClearState();
      thread_state_ = RequestResponseState::NONE;
    }
  }

  // get first gear path length
  bool path_search_valid = true;
  if (response_.GetFirstPathLength() <
      apa_param.GetParam().astar_config.vertical_min_path_length) {
    path_search_valid = false;
  }

  // If path planning failed in slot refresh, do nothing.
  if (response_.request.plan_reason == PlanningReason::SLOT_REFRESHED &&
      !path_search_valid) {
    return PathPlannerResult::PLAN_HOLD;
  }

  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    // success
    if (path_search_valid) {
      if (frame_.is_replan_first) {
        frame_.is_replan_first = false;
      }

      // check path is single shot to goal.
      if (response_.result.gear_change_num > 0 ||
          IsSamplingBasedPlanning(response_.request.path_generate_method)) {
        is_path_connected_to_goal_ = false;
      } else {
        is_path_connected_to_goal_ = true;
      }

      ILOG_INFO << "first path gear = "
                << PathGearDebugString(response_.first_seg_path[0].gear)
                << ",gear_change_num=" << response_.result.gear_change_num;

      lateral_offset_ = response_.result.y.back();

      Transform2d response_tf;
      response_tf.SetBasePose(response_.request.base_pose_);
      PublishHybridAstarDebugInfo(response_.result, &response_tf);
      PathOptimizationByCILQR(response_.first_seg_path, &response_tf);

      if (response_.request.plan_reason != PlanningReason::SLOT_REFRESHED) {
        frame_.total_plan_count++;
      }
      path_planning_fail_num_ = 0;
      if (ego_info.slot_occupied_ratio > 0.2 &&
          response_.request.plan_reason != PlanningReason::SLOT_REFRESHED) {
        replan_number_inside_slot_++;
      }

      ILOG_INFO << "total_plan_count = "
                << static_cast<int>(frame_.total_plan_count)
                << ", replan_number_inside_slot = "
                << replan_number_inside_slot_;

      res = PathPlannerResult::PLAN_UPDATE;
    } else {
      res = PathPlannerResult::PLAN_FAILED;
      path_planning_fail_num_ += 1;
      if (path_planning_fail_num_ == 1 || path_planning_fail_num_ == 2) {
        res = PathPlannerResult::WAIT_PATH;
      } else if (response_.request.plan_reason ==
                 PlanningReason::SLOT_REFRESHED) {
        // If path planning in dynamic replan is fail, use history path.
        res = PathPlannerResult::PLAN_HOLD;
      }

      // publish fallback path
      // GenerateFallBackPath();

      ILOG_INFO << "path planning fail number = " << path_planning_fail_num_;
    }

    // if planning success, update gear
    if (path_search_valid) {
      current_gear_ = response_.first_seg_path[0].gear;
      frame_.current_gear = GetGear(current_gear_);
    }

    ThreadClearState();
    frame_.gear_command = frame_.current_gear;
    ILOG_INFO << "path gear = " << static_cast<int>(frame_.gear_command);
  } else if (thread_state_ == RequestResponseState::NONE ||
             thread_state_ == RequestResponseState::HAS_PUBLISHED_RESPONSE) {
    // send request
    thread_.SetRequest(obs, cur_request);
    res = PathPlannerResult::WAIT_PATH;

    ILOG_INFO << "set input";
  } else if (thread_state_ == RequestResponseState::HAS_REQUEST) {
    res = PathPlannerResult::WAIT_PATH;

    // publish fallback path
    // GenerateFallBackPath();
    HybridAstarDebugInfoClear();

    ILOG_INFO << "has input";
  }

  return res;
}

const PathPlannerResult NarrowSpaceScenario::PubResponseForScenarioTry(
    const EgoInfoUnderSlot& ego_info, const AstarRequest& cur_request,
    const ParkObstacleList& obs) {
  PathPlannerResult res = PathPlannerResult::WAIT_PATH;

  // check result
  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    // get output
    thread_.PublishResponse(&response_);
  }

  // request changed, reset response
  if (response_.GetFirstPathLength() >
      apa_param.GetParam().astar_config.vertical_min_path_length) {
    if (!IsResponseNice(cur_request, response_)) {
      ThreadClearState();
      response_.Clear();
      thread_state_ = RequestResponseState::NONE;
    }
  }

  // publish astar path in cruise state.
  Transform2d response_tf;
  response_tf.SetBasePose(response_.request.base_pose_);
  PublishHybridAstarDebugInfo(response_.result, &response_tf);

  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    // success
    // get first gear path length
    bool path_search_valid = true;
    if (response_.GetFirstPathLength() <
        apa_param.GetParam().astar_config.vertical_min_path_length) {
      path_search_valid = false;
    }
    if (path_search_valid) {
      res = PathPlannerResult::PLAN_UPDATE;
    } else {
      res = PathPlannerResult::PLAN_FAILED;
      // path planning failed, reset response
      response_.Clear();
    }

    ThreadClearState();
  } else if (thread_state_ == RequestResponseState::NONE ||
             thread_state_ == RequestResponseState::HAS_PUBLISHED_RESPONSE) {
    // send request
    thread_.SetRequest(obs, cur_request);

    res = PathPlannerResult::WAIT_PATH;
    ILOG_INFO << "set input";
  } else if (thread_state_ == RequestResponseState::HAS_REQUEST) {
    res = PathPlannerResult::WAIT_PATH;
    ILOG_INFO << "has input";
  }

  return res;
}

const bool NarrowSpaceScenario::IsNeedClipping(const HybridAStarResult& result,
                                               const size_t i) {
  constexpr float kHeadHeadingStartDeg = 80.0f;
  constexpr float kTailHeadingStartDeg = 95.0f;
  constexpr float kHeadingEndDeg = 89.9f;
  constexpr float kHeadingDiffThresh = 1e-3f;
  constexpr float kRad2Deg = 180.0f / static_cast<float>(M_PI);

  bool heading_flag = true;
  bool sample_finish = false;
  const float heading_deg = std::abs(result.phi[i] * kRad2Deg);

  if (heading_deg > kHeadHeadingStartDeg && i > 0) {
    float heading_diff = result.phi[i] - result.phi[i - 1];
    heading_flag = std::abs(heading_diff) > kHeadingDiffThresh;
  }

  if (std::abs(result.phi[i]) * kRad2Deg <= kHeadingEndDeg && !sample_finish &&
      heading_flag) {
    return false;
  } else {
    return true;
  }
}

const pnc::geometry_lib::PathSegGear NarrowSpaceScenario::GetGear(
    const AstarPathGear gear) {
  return (gear == AstarPathGear::DRIVE) ? pnc::geometry_lib::SEG_GEAR_DRIVE
                                        : pnc::geometry_lib::SEG_GEAR_REVERSE;
}

}  // namespace apa_planner
}  // namespace planning