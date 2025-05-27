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

  current_gear_ = AstarPathGear::PARKING;
  replan_number_inside_slot_ = 0;
  is_path_connected_to_goal_ = false;
  path_planning_fail_num_ = 0;
  lateral_offset_ = 0;
  lon_offset_ = 0;

  ParkingScenario::Reset();

  narrow_space_decider_.Reset();
  virtual_wall_decider_.Reset(Pose2D(0, 0, 0));

  return;
}

void NarrowSpaceScenario::Init() {
  const ApaParameters& params = apa_param.GetParam();

  // todo, system should use same vehicle parameter configuration file and
  // data structure.
  ILOG_INFO << "init astar thread";

  current_gear_ = AstarPathGear::PARKING;
  replan_number_inside_slot_ = 0;
  is_path_connected_to_goal_ = false;
  path_planning_fail_num_ = 0;

  thread_.Init(params.rear_overhanging, params.car_length, params.car_width,
               params.steer_ratio, params.wheel_base, params.min_turn_radius,
               (params.max_car_width - params.car_width) * 0.5);

  thread_.Start();

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

const bool NarrowSpaceScenario::CheckHeadOutFinished() {
  bool parking_finish = false;
  const EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  const bool heading_condition_1 =
      std::fabs(ego_info.cur_pose.heading) <= 95.0 * kDeg2Rad;  // TODU::

  const bool heading_condition_2 =
      std::fabs(ego_info.cur_pose.heading) >= 85.0 * kDeg2Rad;

  const bool lat_condition = heading_condition_1 && heading_condition_2;

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const bool remain_s_condition =
      frame_.remain_dist_path < apa_param.GetParam().max_replan_remain_dist;

  if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_STRAIGHT) {
    parking_finish = remain_s_condition && static_condition;
  } else {
    parking_finish = lat_condition && static_condition && remain_s_condition;
  }

  if (parking_finish) {
    return true;
  }

  // stucked by directly behind uss
  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetCollisionDetectorInterfacePtr();
  const bool enter_slot_condition =
      ego_info.slot_occupied_ratio >
      apa_param.GetParam().finish_uss_slot_occupied_ratio;
  const bool remain_uss_condition =
      frame_.remain_dist_obs < apa_param.GetParam().max_replan_remain_dist;

  parking_finish = lat_condition && static_condition && enter_slot_condition &&
                   remain_uss_condition;

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
        if (frame_.replan_fail_time >
            apa_param.GetParam().max_replan_failed_time) {
          SetParkingStatus(PARKING_FAILED);
        }
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

const PerpendicularHeadOutScenario::SlotObsType
NarrowSpaceScenario::CalSlotObsType(const Eigen::Vector2d& obs_slot) {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  // 2米2的车位重规划考虑的障碍物单侧最多入侵车位15厘米
  double dy1 = 0.15 / 1.1 * (ego_info_under_slot.slot.slot_width_ * 0.5);

  // 内外侧障碍物往远离车位的一边考虑远一些
  double dy2 = 6.68;

  // 最多高于车位3.468米的障碍物可以当做内外侧障碍物
  double dx1 = 3.468;
  // // 但是如果自车位置本身较低 那么内外侧障碍物考虑的x值也应该降低
  // dx1 = std::min(dx1, ego_info_under_slot.cur_pose.pos.x() -
  //                         apa_param.GetParam().car_width * 0.5 -
  //                         ego_info_under_slot.slot.slot_length_);
  // // 也需要有个最低考虑位置
  // dx1 = std::max(dx1, 0.368);

  // 对于5米长的车位 从车位线往内延长3.86米当做内外侧障碍物即可
  // 这个时候可以参考当做根据障碍物移动车位的标准， 再深就无需横向移动
  double dx2 = 4.86 / 5.0 * ego_info_under_slot.slot.slot_length_;

  // 对于5米长的车位 最多往后5.2米的障碍物可以在重规划的时候不考虑
  // 再往后就要考虑
  double dx3 = 5.2 / 5.0 * ego_info_under_slot.slot.slot_length_ - dx2;

  Eigen::Vector2d slot_left_pt =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_1;
  Eigen::Vector2d slot_right_pt =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_0;
  if (slot_left_pt.y() < slot_right_pt.y()) {
    std::swap(slot_left_pt, slot_right_pt);
  }

  bool is_left_side = false;
  if (ego_info_under_slot.slot_side == geometry_lib::SLOT_SIDE_LEFT) {
    is_left_side = true;
  }

  std::vector<Eigen::Vector2d> inside_area;
  std::vector<Eigen::Vector2d> outside_area;
  std::vector<Eigen::Vector2d> in_area;
  std::vector<Eigen::Vector2d> discard_area;
  inside_area.resize(4);
  outside_area.resize(4);
  in_area.resize(4);
  discard_area.resize(4);

  const Eigen::Vector2d unit_right2left_vec =
      (slot_left_pt - slot_right_pt).normalized();
  const Eigen::Vector2d unit_left2right_vec = -unit_right2left_vec;
  const Eigen::Vector2d unit_up2down_vec(-1.0, 0.0);
  const Eigen::Vector2d unit_down2up_vec = -unit_up2down_vec;

  // Firstly, the default right side is the inner side, and the left side is the
  // outer side
  Eigen::Vector2d pt;
  // cal inside area
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx1 * unit_down2up_vec;
  inside_area[0] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec + dx1 * unit_down2up_vec;
  inside_area[1] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec + dx2 * unit_up2down_vec;
  inside_area[2] = pt;
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx2 * unit_up2down_vec;
  inside_area[3] = pt;

  // cal outside area
  pt = slot_left_pt + dy2 * unit_right2left_vec + dx1 * unit_down2up_vec;
  outside_area[0] = pt;
  pt = slot_left_pt + dy1 * unit_left2right_vec + dx1 * unit_down2up_vec;
  outside_area[1] = pt;
  pt = slot_left_pt + dy1 * unit_left2right_vec + dx2 * unit_up2down_vec;
  outside_area[2] = pt;
  pt = slot_left_pt + dy2 * unit_right2left_vec + dx2 * unit_up2down_vec;
  outside_area[3] = pt;

  if (is_left_side) {
    std::swap(inside_area, outside_area);
  }

  // cal in_area
  pt = slot_left_pt + dy1 * unit_left2right_vec + dx1 * unit_down2up_vec;
  in_area[0] = pt;
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx1 * unit_down2up_vec;
  in_area[1] = pt;
  pt = slot_right_pt + dy1 * unit_right2left_vec + dx2 * unit_up2down_vec;
  in_area[2] = pt;
  pt = slot_left_pt + dy1 * unit_left2right_vec + dx2 * unit_up2down_vec;
  in_area[3] = pt;

  // cal discard area
  pt = slot_left_pt + dy2 * unit_right2left_vec + dx2 * unit_up2down_vec;
  discard_area[0] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec + dx2 * unit_up2down_vec;
  discard_area[1] = pt;
  pt = slot_right_pt + dy2 * unit_left2right_vec +
       (dx2 + dx3) * unit_up2down_vec;
  discard_area[2] = pt;
  pt =
      slot_left_pt + dy2 * unit_right2left_vec + (dx2 + dx3) * unit_up2down_vec;
  discard_area[3] = pt;

  if (geometry_lib::IsPointInPolygon(inside_area, obs_slot)) {
    return PerpendicularHeadOutScenario::SlotObsType::INSIDE_OBS;
  } else if (geometry_lib::IsPointInPolygon(outside_area, obs_slot)) {
    return PerpendicularHeadOutScenario::SlotObsType::OUTSIDE_OBS;
  } else if (geometry_lib::IsPointInPolygon(in_area, obs_slot)) {
    return PerpendicularHeadOutScenario::SlotObsType::IN_OBS;
  } else if (geometry_lib::IsPointInPolygon(discard_area, obs_slot)) {
    return PerpendicularHeadOutScenario::SlotObsType::DISCARD_OBS;
  } else {
    return PerpendicularHeadOutScenario::SlotObsType::OTHER_OBS;
  }
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
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  PathPlannerResult res = PathPlannerResult::WAIT_PATH;

  // start
  Pose2D start;
  if (frame_.replan_reason == ReplanReason::DYNAMIC) {
    ApaTrajectoryStitcher traj_stitcher;
    traj_stitcher.Process(
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetPose(),
        current_path_point_global_vec_,
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetVel(),
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetFrontWheelAngle(), 0.2);

    const pnc::geometry_lib::PathPoint& stitch_point =
        traj_stitcher.GetStitchPoint();
    Eigen::Vector2d local_pos = ego_info.g2l_tf.GetPos(stitch_point.pos);

    start = Pose2D(local_pos.x(), local_pos.y(),
                   ego_info.g2l_tf.GetHeading(stitch_point.heading));
  } else {
    start = Pose2D(ego_info.cur_pose.pos[0], ego_info.cur_pose.pos[1],
                   ego_info.cur_pose.heading);
  }

  // real target pose in slot
  Pose2D real_end;
  real_end.x = static_cast<float>(ego_info.target_pose.pos[0]);
  real_end.y = static_cast<float>(ego_info.target_pose.pos[1]);
  real_end.theta = static_cast<float>(ego_info.target_pose.heading);

  // astar end, maybe different with real end.
  Pose2D end = real_end;
  double end_straight_len;
  ParkSpaceType slot_type;
  ParkingVehDirection parking_type;
  const ApaStateMachine fsm =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine();
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkOutStatus()) {
    end.y = real_end.y;
    slot_type = ParkSpaceType::VERTICAL;
    switch (
        apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection()) {
      case ApaParkOutDirection::LEFT_FRONT:
        parking_type = ParkingVehDirection::HEAD_OUT_TO_LEFT;
        break;

      case ApaParkOutDirection::RIGHT_FRONT:
        parking_type = ParkingVehDirection::HEAD_OUT_TO_RIGHT;
        break;

      case ApaParkOutDirection::FRONT:
        parking_type = ParkingVehDirection::HEAD_OUT_TO_MIDDLE;
        break;

      default:
        parking_type = ParkingVehDirection::HEAD_OUT_TO_MIDDLE;
        break;
    }
  } else {
    if (ego_info.slot.slot_type_ == SlotType::PARALLEL) {
      end_straight_len =
          apa_param.GetParam().astar_config.parallel_slot_end_straight_dist;
      slot_type = ParkSpaceType::PARALLEL;
    } else if (ego_info.slot.slot_type_ == SlotType::SLANT) {
      end_straight_len =
          apa_param.GetParam().astar_config.vertical_tail_in_end_straight_dist;
      slot_type = ParkSpaceType::SLANTING;
    } else {
      if (fsm == ApaStateMachine::ACTIVE_IN_CAR_REAR ||
          fsm == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR) {
        end_straight_len = apa_param.GetParam()
                               .astar_config.vertical_tail_in_end_straight_dist;

        parking_type = ParkingVehDirection::TAIL_IN;
      } else {
        end_straight_len = apa_param.GetParam()
                               .astar_config.vertical_head_in_end_straight_dist;
        parking_type = ParkingVehDirection::HEAD_IN;
      }
      slot_type = ParkSpaceType::VERTICAL;
    }
    end.x = real_end.x + static_cast<float>(end_straight_len);
  }

  double astar_start_time = IflyTime::Now_ms();
  Pose2D slot_base_pose =
      Pose2D(static_cast<float>(ego_info.origin_pose_global.pos.x()),
             static_cast<float>(ego_info.origin_pose_global.pos.y()),
             static_cast<float>(ego_info.origin_pose_global.heading));

  ParkObstacleList obs;

  // If in searching, use ego init bound;
  // If in parking, use ego position init bound by first plan;
  if (is_scenario_try || frame_.replan_reason == FIRST_PLAN) {
    virtual_wall_decider_.Init(start);
  }
  virtual_wall_decider_.Process(
      obs.virtual_obs, static_cast<float>(ego_info.slot.slot_width_),
      static_cast<float>(ego_info.slot.slot_length_), start, real_end,
      slot_type, ego_info.slot_side, parking_type);

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

  double search_start_time = IflyTime::Now_ms();
  ILOG_INFO << "fusion obj time ms " << search_start_time - astar_start_time;

  // set input
  AstarRequest cur_request;
  cur_request.path_generate_method =
      planning::AstarPathGenerateType::ASTAR_SEARCHING;

  cur_request.first_action_request.has_request = true;
  cur_request.first_action_request.gear_request = AstarPathGear::NONE;
  cur_request.space_type = slot_type;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkOutStatus()) {
    if (frame_.replan_reason == FIRST_PLAN) {
      cur_request.first_action_request.gear_request = AstarPathGear::DRIVE;
    }

    switch (
        apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection()) {
      case ApaParkOutDirection::LEFT_FRONT:
        cur_request.direction_request = ParkingVehDirection::HEAD_OUT_TO_LEFT;
        break;
      case ApaParkOutDirection::FRONT:
        cur_request.direction_request = ParkingVehDirection::HEAD_OUT_TO_MIDDLE;
        break;
      case ApaParkOutDirection::RIGHT_FRONT:
        cur_request.direction_request = ParkingVehDirection::HEAD_OUT_TO_RIGHT;
        break;
      default:
        cur_request.direction_request = ParkingVehDirection::HEAD_OUT_TO_MIDDLE;
        break;
    }
  } else {
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
  }

  cur_request.rs_request = RSPathRequestType::NONE;

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
  cur_request.goal_ = end;
  FillPlanningReason(cur_request);

  is_path_connected_to_goal_ = false;

  // generate request
  if (frame_.replan_reason == ReplanReason::DYNAMIC) {
    switch (cur_request.direction_request) {
      break;
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
        cur_request.path_generate_method =
            planning::AstarPathGenerateType::ASTAR_SEARCHING;
        break;

      default:
        cur_request.path_generate_method =
            planning::AstarPathGenerateType::ASTAR_SEARCHING;
        break;
    }
  }
  // gear need be different with history in next replanning
  FillGearRequest(is_scenario_try, cur_request);

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

  // If path planning failed in slot refresh, do nothing.
  if (response.request.plan_reason == PlanningReason::SLOT_REFRESHED &&
      response.result.x.size() < 4) {
    return PathPlannerResult::PLAN_HOLD;
  }

  // publish result
  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    Transform2d response_tf;
    response_tf.SetBasePose(response.request.base_pose_);

    // success
    if (response.first_seg_path.size() >= 3) {
      if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus() &&
          frame_.is_replan_first) {
        frame_.is_replan_first = false;
      }

      // check path is single shot to goal.
      if (response.result.gear_change_num > 0 ||
          IsSamplingBasedPlanning(response.request.path_generate_method)) {
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

      lateral_offset_ = response.result.y.back();

      if (!is_scenario_try) {
        PublishHybridAstarDebugInfo(response.result, &thread_, &response_tf);

        std::vector<pnc::geometry_lib::PathPoint> local_path;
        size_t i;

        const size_t num = response.first_seg_path.size();
        ILOG_INFO << " path num " << num;

        constexpr float kHeadingStartDeg = 80.0f;
        constexpr float kHeadingEndDeg = 89.9f;
        constexpr float kHeadingDiffThresh = 1e-3f;
        constexpr float kRad2Deg = 180.0f / static_cast<float>(M_PI);

        bool heading_flag = true;
        bool sample_finish = false;

        pnc::geometry_lib::PathPoint point;

        for (int i = 0; i < num; ++i) {
          const AStarPathPoint& path_pt = response.first_seg_path[i];
          point = pnc::geometry_lib::PathPoint(
              Eigen::Vector2d(path_pt.x, path_pt.y), path_pt.phi,
              path_pt.kappa);
          point.s = path_pt.accumulated_s;

          const bool is_park_out =
              apa_world_ptr_->GetStateMachineManagerPtr()->IsParkOutStatus();

          if (is_park_out) {
            const float heading_deg = std::abs(path_pt.phi * kRad2Deg);

            if (heading_deg > kHeadingStartDeg) {
              float heading_diff =
                  path_pt.phi - response.first_seg_path[i - 1].phi;
              heading_flag = std::abs(heading_diff) > kHeadingDiffThresh;
            }

            if (std::abs(path_pt.phi) * kRad2Deg <= kHeadingEndDeg &&
                !sample_finish && heading_flag) {
              local_path.emplace_back(point);
            } else {
              sample_finish = true;
            }

          } else {
            local_path.emplace_back(point);
          }
        }

        PathOptimizationByCILRQ(local_path, &response_tf);
        ILOG_INFO << " current_path_point_global_vec num "
                  << current_path_point_global_vec_.size();

        if (response.request.plan_reason != PlanningReason::SLOT_REFRESHED) {
          frame_.total_plan_count++;
        }
        path_planning_fail_num_ = 0;
        if (ego_info.slot_occupied_ratio > 0.2 &&
            response.request.plan_reason != PlanningReason::SLOT_REFRESHED) {
          replan_number_inside_slot_++;
        }

        ILOG_INFO << "total_plan_count = "
                  << static_cast<int>(frame_.total_plan_count)
                  << ", replan_number_inside_slot = "
                  << replan_number_inside_slot_;
      }

      res = PathPlannerResult::PLAN_UPDATE;
    } else {
      res = PathPlannerResult::PLAN_FAILED;
      if (!is_scenario_try) {
        path_planning_fail_num_ += 1;
        if (path_planning_fail_num_ == 1 || path_planning_fail_num_ == 2) {
          res = PathPlannerResult::WAIT_PATH;
        } else if (response.request.plan_reason ==
                   PlanningReason::SLOT_REFRESHED) {
          // If path planning in dynamic replan is fail, use history path.
          res = PathPlannerResult::PLAN_HOLD;
        }
      }

      // publish fallback path
      // GenerateFallBackPath();

      ILOG_INFO << "path planning fail number = " << path_planning_fail_num_;
    }

    // update gear
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (response.first_seg_path.size() > 1) {
      if (response.first_seg_path[0].gear == AstarPathGear::DRIVE) {
        frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
      }

      if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus()) {
        current_gear_ = response.first_seg_path[0].gear;
      }

      thread_.Clear();
    }

    frame_.gear_command = frame_.current_gear;
    ILOG_INFO << "first path gear = " << static_cast<int>(frame_.gear_command);
  } else if (thread_state_ == RequestResponseState::NONE ||
             thread_state_ == RequestResponseState::HAS_PUBLISHED_RESPONSE) {
    // send request
    thread_.SetRequest(obs, cur_request);
    res = PathPlannerResult::WAIT_PATH;

    // publish fallback path
    // GenerateFallBackPath();

    ILOG_INFO << "set input";
  } else if (thread_state_ == RequestResponseState::HAS_REQUEST) {
    res = PathPlannerResult::WAIT_PATH;

    // publish fallback path
    // GenerateFallBackPath();
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

  std::vector<double> path_x;
  std::vector<double> path_y;
  std::vector<double> path_theta;
  std::vector<double> path_lat_buffer;

  size_t i;
  Pose2D local_position;
  Pose2D global_position;

  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();

  debug_->mutable_refline_info()->Clear();
  bool sample_finish = false;

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

    path_x.emplace_back(global_position.x);
    path_y.emplace_back(global_position.y);
    path_theta.emplace_back(global_position.theta);
    path_lat_buffer.emplace_back(0.0);
  }

  JSON_DEBUG_VECTOR("plan_traj_x", path_x, 3);
  JSON_DEBUG_VECTOR("plan_traj_y", path_y, 3);
  JSON_DEBUG_VECTOR("plan_traj_heading", path_theta, 3);
  JSON_DEBUG_VECTOR("plan_traj_lat_buffer", path_lat_buffer, 3);

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
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

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
  ILOG_INFO << "slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;
  // trim or extend path according to limiter, only run once
  frame_.correct_path_for_limiter = false;
  if (((fsm == ApaStateMachine::ACTIVE_IN_CAR_REAR &&
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
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
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
  constexpr double kInitialTargetHeading = 0.5 * M_PI;
  constexpr double kAlternateTargetX = 8.0;
  constexpr double kAlternateTargetY = 5.0;
  constexpr double kPositionThresholdX = 7.0;
  constexpr double kHeadingThresholdRad = 70.0 * M_PI / 180.0;

  const ApaParkOutDirection park_out_direction =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection();

  switch (park_out_direction) {
    case ApaParkOutDirection::LEFT_FRONT:
      ego_info_under_slot.target_pose.pos << kInitialTargetX, kInitialTargetY;
      ego_info_under_slot.target_pose.heading = kInitialTargetHeading;
      ego_info_under_slot.target_pose.heading_vec = Eigen::Vector2d(0, 1);

      // 特殊位置要对目标点进行特殊调整
      if (ego_info_under_slot.cur_pose.pos.x() < kInitialTargetX &&
          std::abs(ego_info_under_slot.cur_pose.heading) >
              kHeadingThresholdRad) {
        ego_info_under_slot.target_pose.pos << kAlternateTargetX,
            kAlternateTargetY;
      }
      break;

    case ApaParkOutDirection::RIGHT_FRONT:
      ego_info_under_slot.target_pose.pos << kInitialTargetX, -kInitialTargetY;
      ego_info_under_slot.target_pose.heading = -kInitialTargetHeading;
      ego_info_under_slot.target_pose.heading_vec = Eigen::Vector2d(0, -1);

      // 特殊位置要对目标点进行特殊调整
      if (ego_info_under_slot.cur_pose.pos.x() < kInitialTargetX &&
          std::abs(ego_info_under_slot.cur_pose.heading) >
              kHeadingThresholdRad) {
        ego_info_under_slot.target_pose.pos << kAlternateTargetX,
            -kAlternateTargetY;
      }
      break;

    case ApaParkOutDirection::FRONT:
    default:
      ego_info_under_slot.target_pose.pos << kInitialTargetX - 4, 0.0;
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
  if (std::fabs(ego_info_under_slot.cur_pose.pos.y()) <
          param.slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_info_under_slot.cur_pose.heading) <
          param.slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    const std::vector<double> x_tab = {
        ego_info_under_slot.virtual_limiter.first.x(),
        ego_info_under_slot.slot.slot_length_ + param.rear_overhanging};

    const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
    ego_info_under_slot.slot_occupied_ratio = mathlib::Interp1(
        x_tab, occupied_ratio_tab, ego_info_under_slot.cur_pose.pos.x());
  } else {
    ego_info_under_slot.slot_occupied_ratio = 0.0;
  }

  ILOG_INFO << "slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;

  return true;
}

NarrowSpaceScenario::~NarrowSpaceScenario() {}

void NarrowSpaceScenario::PathShrinkBySlotLimiter() {
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

  // car pose has big distance with limiter, return
  // If car pose is nearby of limiter (0.4 meter), do not shrink path to prevent
  // car speed change too much.
  double x_diff = std::fabs(ego_info.terminal_err.pos.x());
  double y_diff = std::fabs(ego_info.terminal_err.pos.y());
  if (x_diff > 2.0 || y_diff > 2.0 || x_diff < 0.4) {
    return;
  }

  // shrink path
  size_t path_size = current_path_point_global_vec_.size();
  for (size_t i = 0; i < path_size; i++) {
    Eigen::Vector2d& point_global = current_path_point_global_vec_.back().pos;
    point_local = ego_info.g2l_tf.GetPos(point_global);
    if (point_local[0] >= limiter_x + 1e-2) {
      break;
    }

    current_path_point_global_vec_.pop_back();
  }

  return;
}

void NarrowSpaceScenario::PathExpansionBySlotLimiter() {
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

  // If ego is around limiter（0.5 meter）, do not extend path to prevent car
  // speed change too much.
  if (dist_to_goal < 0.5) {
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
    // 如果上一帧A星释放车位，在当前帧结果还没有出来时，不改变上一帧结果;
    // 如果上一帧结果未知，使用计算中状态填充;
    if (astar_release_state == SlotReleaseState::UNKOWN) {
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
          SlotReleaseState::COMPUTING;
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
    return CheckDynamicParkingIn();
  }
}

const bool NarrowSpaceScenario::CheckDynamicParkingIn() {
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
                                    param.pose_slot_occupied_ratio_2);

  // check path remain dist
  const bool path_dist_flag = frame_.remain_dist_path > 1.5;

  const float perception_blind_spot_distance = 6.0;

  const bool current_path_length_flag =
      frame_.current_path_length > perception_blind_spot_distance;

  const bool dynamic_replan_flag = car_motion_flag && car_pos_flag &&
                                   occupied_ratio_flag && path_dist_flag &&
                                   current_path_length_flag;

  return dynamic_replan_flag;
}

void NarrowSpaceScenario::FillPlanningReason(AstarRequest& cur_request) {
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
      cur_request.plan_reason = PlanningReason::SLOT_REFRESHED;
      break;
    case SEG_COMPLETED_COL_DET:
      cur_request.plan_reason = PlanningReason::PATH_STUCKED;
      break;
    default:
      cur_request.plan_reason = PlanningReason::NONE;
      break;
  }

  return;
}

void NarrowSpaceScenario::FillGearRequest(const bool is_scenario_try,
                                          AstarRequest& cur_request) {
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

}  // namespace apa_planner
}  // namespace planning