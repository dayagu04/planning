#include "perpendicular_head_in_scenario.h"

#include <bits/stdint-uintn.h>

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <queue>

#include "apa_utils.h"
#include "apa_world.h"
#include "basic_types.pb.h"
#include "collision_detection/collision_detection.h"
#include "common.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "fem_pos_deviation_smoother_config.pb.h"
#include "func_state_machine_c.h"
#include "general_planning_context.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "log_glog.h"
#include "math_lib.h"
#include "perpendicular_head_in_path_generator.h"
#include "planning_plan_c.h"
#include "slot_management_info.pb.h"
#include "src/modules/apa_function/apa_param_config.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

namespace planning {
namespace apa_planner {

void PerpendicularHeadInScenario::Reset() {
  frame_.Reset();
  perpendicular_path_planner_.Reset();

  current_plan_path_vec_.clear();

  ParkingScenario::Reset();
}

const double PerpendicularHeadInScenario::CalRemainDistFromPath() {
  double remain_dist = 15.0;

  if (frame_.is_replan_first) {
    return remain_dist;
  }

  if (frame_.spline_success) {
    double s_proj = 0.0;
    bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
        0.0, frame_.current_path_length, s_proj,
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetPos(), frame_.x_s_spline,
        frame_.y_s_spline);

    if (success == true) {
      remain_dist = frame_.current_path_length - s_proj;

      ILOG_INFO << "remain_dist = " << remain_dist << "  s_proj = " << s_proj
                << "  current_path_length = " << frame_.current_path_length;
    } else {
      ILOG_INFO << "remain_dist calculation error:input is error";
    }
  } else {
    ILOG_INFO << "remain_dist calculation error: path spline failed!";
  }

  return remain_dist;
}

void PerpendicularHeadInScenario::ExcutePathPlanningTask() {
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

  const double safe_uss_remain_dist =
      (apa_world_ptr_->GetSlotManagerPtr()
           ->GetEgoInfoUnderSlot()
           .slot_occupied_ratio < 0.05)
          ? apa_param.GetParam().safe_uss_remain_dist_out_slot
          : apa_param.GetParam().safe_uss_remain_dist_in_slot;

  // calculate remain dist according to plan path
  frame_.remain_dist_path = CalRemainDistFromPath();

  // calculate remain dist uss according to uss
  frame_.remain_dist_obs = CalRemainDistFromObs(safe_uss_remain_dist);

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ILOG_INFO << "update ego slot info";
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = UPDATE_EGO_SLOT_INFO;
    return;
  }

  // check finish
  if (CheckFinished()) {
    ILOG_INFO << "check apa finished!";
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

  // check failed
  if (CheckStuckFailed()) {
    ILOG_INFO << "check stuck failed!";
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = STUCK_FAILED_TIME;
    return;
  }

  CheckReplanParams replan_params;
  // check replan
  if (CheckReplan(replan_params)) {
    ILOG_INFO << "replan is required!";
    frame_.replan_flag = true;
    EgoInfoUnderSlot& ego_info_under_slot =
        apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

    frame_.dynamic_plan_fail_flag = false;

    const double start_time = IflyTime::Now_ms();

    GenTlane();

    GenObstacles();

    frame_.total_plan_count++;

    uint8_t pathplan_result = PathPlannerResult::PLAN_FAILED;

    if (frame_.total_plan_count <=
        apa_param.GetParam().headin_max_replan_count) {
      pathplan_result = PathPlanOnce();
      ILOG_INFO << "generate path by geometry";
    } else {
      ILOG_INFO << "replan count is exceed max count, fail, directly quit apa";
      frame_.plan_fail_reason = PLAN_COUNT_EXCEED_LIMIT;
    }

    if (!frame_.dynamic_plan_fail_flag) {
      frame_.car_already_move_dist = 0.0;
    }

    JSON_DEBUG_VALUE("replan_count", frame_.total_plan_count)

    JSON_DEBUG_VALUE("dynamic_plan_fail_flag", frame_.dynamic_plan_fail_flag)

    ILOG_INFO << "replan_consume_time = " << IflyTime::Now_ms() - start_time
              << " ms";
    JSON_DEBUG_VALUE("replan_consume_time", IflyTime::Now_ms() - start_time)

    frame_.pathplan_result = pathplan_result;

    if (pathplan_result == PathPlannerResult::PLAN_HOLD) {
      if (PostProcessPath()) {
        SetParkingStatus(PARKING_GEARCHANGE);
        ILOG_INFO << "replan from PARKING_GEARCHANGE!";
      } else {
        SetParkingStatus(PARKING_FAILED);
        ILOG_INFO << "replan failed from PLAN_HOLD!";
      }
    } else if (pathplan_result == PathPlannerResult::PLAN_UPDATE) {
      if (PostProcessPath()) {
        SetParkingStatus(PARKING_PLANNING);
        ILOG_INFO << "replan from PARKING_PLANNING!";
      } else {
        SetParkingStatus(PARKING_FAILED);
        ILOG_INFO << "replan failed from PARKING_PLANNING!";
      }
    } else if (pathplan_result == PathPlannerResult::PLAN_FAILED) {
      SetParkingStatus(PARKING_FAILED);
    }

    ILOG_INFO << "pathplan_result = " << static_cast<int>(pathplan_result);
  } else {
    ILOG_INFO << "replan is not required!";
    SetParkingStatus(PARKING_RUNNING);
  }

  // check planning status
  ILOG_INFO << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status);
}

const bool PerpendicularHeadInScenario::UpdateEgoSlotInfo() {
  const auto measures_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();
  const ApaParameters& param = apa_param.GetParam();
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  frame_.correct_path_for_limiter = false;
  frame_.replan_flag = false;

  // update target slot info
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

  // update ego slot info
  ego_info_under_slot.cur_pose.pos =
      ego_info_under_slot.g2l_tf.GetPos(measures_ptr->GetPos());
  ego_info_under_slot.cur_pose.heading =
      ego_info_under_slot.g2l_tf.GetHeading(measures_ptr->GetHeading());
  ego_info_under_slot.cur_pose.heading_vec =
      geometry_lib::GenHeadingVec(ego_info_under_slot.cur_pose.heading);

  // update limiter info
  double virtual_tar_x = 0.0;
  if (ego_info_under_slot.slot.limiter_.valid) {
    Eigen::Vector2d pt1 = ego_info_under_slot.g2l_tf.GetPos(
        ego_info_under_slot.slot.limiter_.start_pt);
    Eigen::Vector2d pt2 = ego_info_under_slot.g2l_tf.GetPos(
        ego_info_under_slot.slot.limiter_.end_pt);

    virtual_tar_x = 0.5 * (pt1 + pt2).x() + param.limiter_move_dist;
  } else {
    virtual_tar_x =
        ego_info_under_slot.slot.processed_corner_coord_local_.pt_23_mid.x() +
        param.terminal_target_x;
  }

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

  // for simulation
  if (apa_world_ptr_->GetSimuParam().is_simulation &&
      apa_world_ptr_->GetSimuParam().target_managed_limiter_x_vec.size() == 2 &&
      apa_world_ptr_->GetSimuParam().use_slot_in_bag) {
    ego_info_under_slot.virtual_limiter.first
        << apa_world_ptr_->GetSimuParam().target_managed_limiter_x_vec[0],
        apa_world_ptr_->GetSimuParam().target_managed_limiter_y_vec[0];

    ego_info_under_slot.virtual_limiter.first =
        ego_info_under_slot.g2l_tf.GetPos(
            ego_info_under_slot.virtual_limiter.first);

    ego_info_under_slot.virtual_limiter.second
        << apa_world_ptr_->GetSimuParam().target_managed_limiter_x_vec[1],
        apa_world_ptr_->GetSimuParam().target_managed_limiter_y_vec[1];

    ego_info_under_slot.virtual_limiter.second =
        ego_info_under_slot.g2l_tf.GetPos(
            ego_info_under_slot.virtual_limiter.second);
  }

  // calculate path planner target info
  ego_info_under_slot.target_pose.pos
      << ego_info_under_slot.virtual_limiter.first.x() + param.wheel_base,
      param.terminal_target_y;

  if (ego_info_under_slot.cur_pose.heading <= 0.0) {
    ego_info_under_slot.target_pose.heading =
        param.terminal_target_heading - 180.0 * kDeg2Rad;
  } else {
    ego_info_under_slot.target_pose.heading =
        param.terminal_target_heading + 180.0 * kDeg2Rad;
  }

  // cal terminal error
  ego_info_under_slot.terminal_err.Set(
      ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
      geometry_lib::NormalizeAngle(ego_info_under_slot.cur_pose.heading -
                                   ego_info_under_slot.target_pose.heading));

  // cal slot occupied ratio
  if (std::fabs(ego_info_under_slot.terminal_err.pos.y()) <
          param.slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_info_under_slot.terminal_err.heading) <
          param.slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    ego_info_under_slot.slot_occupied_ratio =
        pnc::mathlib::Clamp(1.0 - (ego_info_under_slot.terminal_err.pos.x() /
                                   ego_info_under_slot.slot.slot_length_),
                            0.0, 1.0);
  } else {
    ego_info_under_slot.slot_occupied_ratio = 0.0;
  }

  // calc slot side and init gear and init steer first time
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

    // judge slot side via slot center and heading
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else {
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      ILOG_INFO << "calculate slot side error ";
      // return false;
    }
  }

  // trim path according to limiter
  if (frame_.gear_command == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    const geometry_lib::LineSegment limiter_line(
        ego_info_under_slot.virtual_limiter.first,
        ego_info_under_slot.virtual_limiter.second);

    const double dist_ego_limiter = geometry_lib::CalPoint2LineDist(
        ego_info_under_slot.cur_pose.pos +
            param.wheel_base * ego_info_under_slot.cur_pose.heading_vec,
        limiter_line);

    // ILOG_INFO << "wheel_base = " << param.wheel_base
    //           << " front wheel pose = "
    //           << (ego_info_under_slot.cur_pose.pos +
    //               param.wheel_base *
    //                   ego_info_under_slot.cur_pose.heading_vec)
    //                  .transpose()
    //           << " dist_ego_limiter = " << dist_ego_limiter;

    if (dist_ego_limiter < 1.2 * param.car_to_limiter_dis &&
        dist_ego_limiter > 1.1 * param.car_to_limiter_dis) {
      ILOG_INFO << "should correct path according limiter";
      PostProcessPathAccordingLimiter();
      frame_.correct_path_for_limiter = true;
    }
  }

  // fix slot
  if (ego_info_under_slot.terminal_err.heading <
          param.headin_multi_plan_min_heading_err * kDeg2Rad &&
      !ego_info_under_slot.fix_slot && measures_ptr->GetStaticFlag()) {
    ego_info_under_slot.fix_slot = true;
    ILOG_INFO << "fix slot !";
  }

  ILOG_INFO << "slot_side = "
            << static_cast<int>(ego_info_under_slot.slot_side);
  ILOG_INFO << "frame_.current_gear = "
            << static_cast<int>(frame_.current_gear);
  ILOG_INFO << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer);

  ILOG_INFO << "terminal x error= " << ego_info_under_slot.terminal_err.pos.x();
  ILOG_INFO << "terminal y error= " << ego_info_under_slot.terminal_err.pos.y();
  ILOG_INFO << "terminal heading error= "
            << ego_info_under_slot.terminal_err.heading * kRad2Deg;

  ILOG_INFO << "slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;

  ILOG_INFO << "stuck_time = " << frame_.stuck_time << " s";
  return true;
}

const bool PerpendicularHeadInScenario::GenTlane() {
  using namespace pnc::geometry_lib;
  const ApaParameters& param = apa_param.GetParam();
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  // construct tlane pq
  // left y is positive, right y is negative
  // left y should be smallest, right y should be largest
  // all x should be largest
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      left_pq_for_y(Compare(3));

  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      left_pq_for_x(Compare(0));

  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      right_pq_for_y(Compare(2));

  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      right_pq_for_x(Compare(0));

  const double mir_width = (param.max_car_width - param.car_width) * 0.5;

  CollisionDetector::ObsSlotType obs_slot_type;
  const std::pair<Eigen::Vector2d, Eigen::Vector2d> slot_pt =
      std::make_pair(ego_info_under_slot.slot.origin_corner_coord_local_.pt_1,
                     ego_info_under_slot.slot.origin_corner_coord_local_.pt_0);

  const bool is_left_side =
      (ego_info_under_slot.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT);

  double max_obs_lat_invasion_slot_dist = 0.0;

  const double mir_x = ego_info_under_slot.target_pose.pos.x();

  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  const auto& obstacles =
      apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles();

  // sift obstacles that meet requirement
  for (const auto& pair : obstacles) {
    // ILOG_INFO << "obs type = " << pair.first << " , local obstacle size = "
    //           << pair.second.GetPtClout2dLocal().size();

    for (const auto obstacle_point_slot_ : pair.second.GetPtClout2dLocal()) {
      Eigen::Vector2d obstacle_point_slot = obstacle_point_slot_;
      obs_slot_type = apa_world_ptr_->GetCollisionDetectorPtr()->GetObsSlotType(
          obstacle_point_slot, slot_pt, is_left_side, frame_.replan_flag, true);

      if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_IN_OBS &&
          !param.believe_in_fus_obs) {
        max_obs_lat_invasion_slot_dist =
            frame_.replan_flag
                ? param.max_obs_lat_invasion_slot_dist
                : param.max_obs_lat_invasion_slot_dist_dynamic_col;

        const double min_left_y = 0.5 * ego_info_under_slot.slot.slot_width_ -
                                  max_obs_lat_invasion_slot_dist;

        const double max_right_y = -0.5 * ego_info_under_slot.slot.slot_width_ +
                                   max_obs_lat_invasion_slot_dist;

        // obs is in slot, temp hack, when believe_in_fus_obs is false,
        // force move obs to out slot
        if (obstacle_point_slot.y() < min_left_y &&
            obstacle_point_slot.y() > 0.0) {
          obstacle_point_slot.y() = min_left_y;
        }
        if (obstacle_point_slot.y() > max_right_y &&
            obstacle_point_slot.y() < 0.0) {
          obstacle_point_slot.y() = max_right_y;
        }
      } else if (obs_slot_type !=
                     CollisionDetector::ObsSlotType::SLOT_INSIDE_OBS &&
                 obs_slot_type !=
                     CollisionDetector::ObsSlotType::SLOT_OUTSIDE_OBS) {
        continue;
      }

      // omit obstacle nearby lower boundary to make sure tlane width
      if ((obstacle_point_slot.x() < param.headin_tlane_obs_omit_x) &&
          (std::fabs(obstacle_point_slot.y()) <
           ego_info_under_slot.slot.slot_width_ * 0.5 +
               param.headin_virtual_obs_y_pos)) {
        continue;
      }

      // the obs lower mir can relax requirements
      if (obstacle_point_slot.x() < mir_x) {
        if (obstacle_point_slot.y() > 1e-6) {
          obstacle_point_slot.y() += mir_width;
        } else {
          obstacle_point_slot.y() -= mir_width;
        }
      }

      if (obstacle_point_slot.y() > 1e-6) {
        left_pq_for_y.emplace(std::move(obstacle_point_slot));
        left_pq_for_x.emplace(std::move(obstacle_point_slot));
      } else {
        right_pq_for_y.emplace(std::move(obstacle_point_slot));
        right_pq_for_x.emplace(std::move(obstacle_point_slot));
      }
    }
  }

  // If there are no obstacles on either side, set a virtual obstacle that is
  // farther away
  const auto pt_01_norm_vec =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_01_unit_vec;

  const auto pt_01_tan_vec = ego_info_under_slot.slot.origin_corner_coord_local_
                                 .pt_23mid_01mid_unit_vec;
  const double virtual_x =
      (ego_info_under_slot.slot.origin_corner_coord_local_.pt_01_mid -
       param.headin_virtual_obs_x_pos * pt_01_tan_vec)
          .x();

  const double virtual_left_y =
      (ego_info_under_slot.slot.origin_corner_coord_local_.pt_1 +
       param.headin_virtual_obs_y_pos * pt_01_norm_vec)
          .y();

  const double virtual_right_y =
      (ego_info_under_slot.slot.origin_corner_coord_local_.pt_0 -
       param.headin_virtual_obs_y_pos * pt_01_norm_vec)
          .y();

  ILOG_INFO << "virtual_left_y = " << virtual_left_y
            << "  virtual_right_y = " << virtual_right_y;

  bool left_empty = false;
  bool right_empty = false;

  if (left_pq_for_x.empty()) {
    ILOG_INFO << "left space is empty";
    left_empty = true;
    left_pq_for_x.emplace(Eigen::Vector2d(virtual_x, 0.0));
    left_pq_for_y.emplace(Eigen::Vector2d(0.0, virtual_left_y));
  }
  if (right_pq_for_x.empty()) {
    ILOG_INFO << "right space is empty";
    right_empty = true;
    right_pq_for_x.emplace(Eigen::Vector2d(virtual_x, 0.0));
    right_pq_for_y.emplace(Eigen::Vector2d(0.0, virtual_right_y));
  }
  // set right&left slot empty info
  frame_.is_left_empty = left_empty;
  frame_.is_right_empty = right_empty;

  const double car_half_width_with_mirror = param.max_car_width * 0.5;

  const double virtual_slot_width =
      param.max_car_width + param.slot_compare_to_car_width;

  const double real_slot_width = ego_info_under_slot.slot.slot_width_;

  ILOG_INFO << "max_car_width = " << param.max_car_width
            << "  virtual slot width = " << virtual_slot_width
            << "  real slot width = " << real_slot_width;

  double left_y = left_pq_for_y.top().y();
  double real_left_y = left_y;

  double left_x = left_pq_for_x.top().x();
  double real_left_x = left_x;

  double right_y = right_pq_for_y.top().y();
  double real_right_y = right_y;

  double right_x = right_pq_for_x.top().x();
  double real_right_x = right_x;

  // set t lane area
  const double threshold = 0.0868;
  if (param.use_fus_occ_obj) {
    left_y = std::max(left_y, car_half_width_with_mirror + threshold);
    left_y = std::min(left_y, virtual_left_y);
    left_x = std::min(
        left_x, ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.x() -
                    1.68 * threshold);

    left_x = std::max(left_x, virtual_x);

    right_y = std::min(right_y, -car_half_width_with_mirror - threshold);
    right_y = std::max(right_y, virtual_right_y);
    right_x = std::min(
        right_x, ego_info_under_slot.slot.origin_corner_coord_local_.pt_0.x() -
                     1.68 * threshold);

    right_x = std::max(right_x, virtual_x);
  }

  ILOG_INFO << "real_left_y = " << real_left_y
            << "  real_right_y = " << real_right_y;

  ILOG_INFO << "left_y = " << left_y << "  right_y = " << right_y
            << "  left_x = " << left_x << "  right_x = " << right_x;

  // target pose
  double left_dis_obs_car = 0.0;
  double right_dis_obs_car = 0.0;
  if (param.use_fus_occ_obj) {
    // use fus obj
    left_dis_obs_car = real_left_y - car_half_width_with_mirror;
    right_dis_obs_car = -car_half_width_with_mirror - real_right_y;
  } else {
    // use uss obj
    left_dis_obs_car = left_y - car_half_width_with_mirror;
    right_dis_obs_car = -car_half_width_with_mirror - right_y;
  }

  bool left_obs_meet_safe_require = false;
  bool right_obs_meet_safe_require = false;

  const double safe_threshold =
      param.car_lat_inflation_normal + param.safe_threshold;

  left_obs_meet_safe_require = left_dis_obs_car > safe_threshold ? true : false;
  right_obs_meet_safe_require =
      right_dis_obs_car > safe_threshold ? true : false;

  if (ego_info_under_slot.slot_occupied_ratio < 0.618 && frame_.replan_flag) {
    // if two side is all no safe, the slot would not release in slot managment,
    // and should not move target pose
    if (!left_obs_meet_safe_require && right_obs_meet_safe_require) {
      // left side is dangerous, should move toward right
      ego_info_under_slot.move_slot_dist = safe_threshold - left_dis_obs_car;
      ego_info_under_slot.move_slot_dist *= -1.0;
    } else if (left_obs_meet_safe_require && !right_obs_meet_safe_require) {
      // right side is dangerous, should move toward left
      ego_info_under_slot.move_slot_dist = safe_threshold - right_dis_obs_car;
    }
    ILOG_INFO << "left_dis_obs_car = " << left_dis_obs_car
              << "  right_dis_obs_car = " << right_dis_obs_car;
  }

  const bool need_move_slot =
      (pnc::mathlib::IsDoubleEqual(ego_info_under_slot.move_slot_dist, 0.0))
          ? false
          : true;

  if (need_move_slot) {
    // cal max_move_slot_dist to avoid car press line
    // no consider mirror
    const double half_car_width = param.car_width * 0.5;
    const double half_slot_width = ego_info_under_slot.slot.slot_width_ * 0.5;
    const double car2line_dist_threshold = param.car2line_dist_threshold;

    const double max_move_slot_dist =
        half_slot_width - half_car_width - car2line_dist_threshold;
    if (max_move_slot_dist > 0.0 &&
        (std::fabs(ego_info_under_slot.move_slot_dist) > max_move_slot_dist)) {
      if (ego_info_under_slot.move_slot_dist > 0.0) {
        ego_info_under_slot.move_slot_dist = max_move_slot_dist;
      }
      if (ego_info_under_slot.move_slot_dist < 0.0) {
        ego_info_under_slot.move_slot_dist = -max_move_slot_dist;
      }
    }

    // update target pos
    ego_info_under_slot.target_pose.pos +=
        ego_info_under_slot.move_slot_dist * pt_01_norm_vec;

    ego_info_under_slot.terminal_err.Set(
        ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
        geometry_lib::NormalizeAngle(ego_info_under_slot.cur_pose.heading -
                                     ego_info_under_slot.target_pose.heading));
  }

  JSON_DEBUG_VALUE("move_slot_dist", ego_info_under_slot.move_slot_dist)

  // construct slot_t_lane_, left is positive, right is negative
  const double slot_width = std::min(virtual_slot_width, real_slot_width);
  const double channel_width =
      param.channel_width / ego_info_under_slot.slot.sin_angle_;

  const double channel_length = param.channel_length;
  const double obs_length = (channel_length - slot_width) * 0.5;

  Eigen::Vector2d corner_left_slot(ego_info_under_slot.slot.slot_length_,
                                   0.5 * slot_width);

  Eigen::Vector2d corner_right_slot(ego_info_under_slot.slot.slot_length_,
                                    -0.5 * slot_width);

  ego_info_under_slot.obs_tlane.min_x =
      ego_info_under_slot.target_pose.pos.x() - param.wheel_base -
      param.rear_overhanging - param.col_obs_safe_dist_normal - 0.05;

  const auto& slot_side = ego_info_under_slot.slot_side;
  ILOG_INFO << "channel_width = " << channel_width;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    // inside is left, outside is right
    ego_info_under_slot.obs_tlane.E = corner_right_slot;
    ego_info_under_slot.obs_tlane.E.x() =
        std::min(real_right_x,
                 ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.x()) +
        param.tlane_safe_dx;

    ego_info_under_slot.obs_tlane.E.y() = std::min(
        real_right_y,
        ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.y() - 0.05);

    ego_info_under_slot.obs_tlane.B = corner_left_slot;
    ego_info_under_slot.obs_tlane.B.x() =
        std::min(real_left_x,
                 ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.x()) +
        param.tlane_safe_dx;

    ego_info_under_slot.obs_tlane.B.y() = std::max(
        real_left_y,
        ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.y() - 0.05);

    ego_info_under_slot.obs_tlane.A =
        ego_info_under_slot.obs_tlane.B + pt_01_norm_vec * obs_length;

    ego_info_under_slot.obs_tlane.C << ego_info_under_slot.obs_tlane.min_x,
        ego_info_under_slot.obs_tlane.B.y();

    ego_info_under_slot.obs_tlane.D << ego_info_under_slot.obs_tlane.min_x,
        ego_info_under_slot.obs_tlane.E.y();

    ego_info_under_slot.obs_tlane.F =
        ego_info_under_slot.obs_tlane.E - pt_01_norm_vec * obs_length;

    ego_info_under_slot.obs_tlane.G
        << ego_info_under_slot.obs_tlane.min_x + channel_width +
               ego_info_under_slot.slot.slot_length_,
        ego_info_under_slot.obs_tlane.F.y();

    ego_info_under_slot.obs_tlane.H << ego_info_under_slot.obs_tlane.G.x(),
        ego_info_under_slot.obs_tlane.A.y();

  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    // inside is right, outside is left
    ego_info_under_slot.obs_tlane.E = corner_left_slot;
    ego_info_under_slot.obs_tlane.E.x() =
        std::min(real_left_x,
                 ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.x()) +
        param.tlane_safe_dx;

    ego_info_under_slot.obs_tlane.E.y() = std::max(
        real_left_y,
        ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.y() - 0.05);

    ego_info_under_slot.obs_tlane.B = corner_right_slot;
    ego_info_under_slot.obs_tlane.B.x() =
        std::min(real_right_x,
                 ego_info_under_slot.slot.origin_corner_coord_local_.pt_0.x()) +
        param.tlane_safe_dx;

    ego_info_under_slot.obs_tlane.B.y() = std::min(
        real_right_y,
        ego_info_under_slot.slot.origin_corner_coord_local_.pt_0.y() + 0.05);

    ego_info_under_slot.obs_tlane.A =
        ego_info_under_slot.obs_tlane.B - pt_01_norm_vec * obs_length;

    ego_info_under_slot.obs_tlane.C << ego_info_under_slot.obs_tlane.min_x,
        ego_info_under_slot.obs_tlane.B.y();

    ego_info_under_slot.obs_tlane.D << ego_info_under_slot.obs_tlane.min_x,
        ego_info_under_slot.obs_tlane.E.y();

    ego_info_under_slot.obs_tlane.F =
        ego_info_under_slot.obs_tlane.E + pt_01_norm_vec * obs_length;

    ego_info_under_slot.obs_tlane.G
        << ego_info_under_slot.obs_tlane.min_x + channel_width +
               ego_info_under_slot.slot.slot_length_,
        ego_info_under_slot.obs_tlane.F.y();

    ego_info_under_slot.obs_tlane.H << ego_info_under_slot.obs_tlane.G.x(),
        ego_info_under_slot.obs_tlane.A.y();
  }

  if (need_move_slot) {
    ego_info_under_slot.obs_tlane.B.y() += ego_info_under_slot.move_slot_dist;
    ego_info_under_slot.obs_tlane.E.y() += ego_info_under_slot.move_slot_dist;
    ego_info_under_slot.obs_tlane.C.y() += ego_info_under_slot.move_slot_dist;
    ego_info_under_slot.obs_tlane.D.y() += ego_info_under_slot.move_slot_dist;
    // std::cout << "should move slot according to obs pt, move dist = "
    //           << ego_slot_info.move_slot_dist << std::endl;
  }
  return true;
}

const bool PerpendicularHeadInScenario::GenObstacles() {
  apa_world_ptr_->GetCollisionDetectorPtr()->ClearObstacles();
  // set obstacles
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  const TLane& obs_tlane = ego_info_under_slot.obs_tlane;
  const ApaParameters& param = apa_param.GetParam();

  // for simulation
  if (apa_world_ptr_->GetSimuParam().is_simulation &&
      apa_world_ptr_->GetSimuParam().use_obs_in_bag &&
      apa_world_ptr_->GetSimuParam().obs_x_vec.size() > 0) {
    std::vector<Eigen::Vector2d> obs_vec;
    obs_vec.reserve(apa_world_ptr_->GetSimuParam().obs_x_vec.size());
    Eigen::Vector2d obs;
    for (size_t i = 0; i < apa_world_ptr_->GetSimuParam().obs_x_vec.size();
         ++i) {
      obs = ego_info_under_slot.g2l_tf.GetPos(
          Eigen::Vector2d(apa_world_ptr_->GetSimuParam().obs_x_vec[i],
                          apa_world_ptr_->GetSimuParam().obs_y_vec[i]));
      obs_vec.emplace_back(obs);
    }
    apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
        obs_vec, CollisionDetector::RECORD_OBS);
    return true;
  }

  // real time obs
  geometry_lib::LineSegment tlane_line;
  std::vector<geometry_lib::LineSegment> tlane_line_vec;
  tlane_line.SetPoints(obs_tlane.A, obs_tlane.B);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.B, obs_tlane.C);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.C, obs_tlane.D);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.D, obs_tlane.E);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.E, obs_tlane.F);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.F, obs_tlane.G);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.G, obs_tlane.H);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(obs_tlane.H, obs_tlane.A);
  tlane_line_vec.emplace_back(tlane_line);

  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(188);
  std::vector<Eigen::Vector2d> point_set;
  for (const auto& line : tlane_line_vec) {
    geometry_lib::SamplePointSetInLineSeg(point_set, line,
                                          apa_param.GetParam().obstacle_ds);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }
  // post process
  std::vector<Eigen::Vector2d> tlane_obs_vec;
  tlane_obs_vec.reserve(tlane_obstacle_vec.size());
  for (const Eigen::Vector2d& obs_pos : tlane_obstacle_vec) {
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs_pos, ego_info_under_slot.cur_pose, 0.0168)) {
      tlane_obs_vec.emplace_back(obs_pos);
    }
  }
  apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
      tlane_obs_vec, CollisionDetector::TLANE_OBS);

  // add actual obs
  const std::vector<Eigen::Vector2d> tlane_vec{
      obs_tlane.A, obs_tlane.B, obs_tlane.C, obs_tlane.D,
      obs_tlane.E, obs_tlane.F, obs_tlane.G, obs_tlane.H};

  std::vector<Eigen::Vector2d> fus_obs_vec;
  const auto& obstacles =
      apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles();

  Eigen::Vector2d obs_pt_slot;
  for (const auto& pair : obstacles) {
    // ILOG_INFO << "obs type = " << pair.first << " , local obstacle size = "
    //           << pair.second.GetPtClout2dLocal().size();

    for (const auto& obs : pair.second.GetPtClout2dLocal()) {
      if (apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs, ego_info_under_slot.cur_pose, 0.0168)) {
        // when obs is in car, lose it
        if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
          return false;
        }
        continue;
      }

      // if obs is not in tlane area lose it
      if (!geometry_lib::IsPointInPolygon(tlane_vec, obs)) {
        continue;
      }
      if (obs.x() < param.headin_tlane_obs_omit_x) {
        continue;
      }
      fus_obs_vec.emplace_back(obs);
    }
  }

  apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
      fus_obs_vec, CollisionDetector::FUSION_OBS);

  return true;
}

const uint8_t PerpendicularHeadInScenario::PathPlanOnce() {
  ILOG_INFO << "-------------- PathPlanOnce --------------";
  // construct input
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  GeometryPathInput path_planner_input;
  path_planner_input.ego_info_under_slot = ego_info_under_slot;
  path_planner_input.ego_info_under_slot.slot.origin_corner_coord_local_.pt_0 =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_1;

  path_planner_input.ego_info_under_slot.slot.origin_corner_coord_local_.pt_1 =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_0;

  path_planner_input.is_complete_path =
      apa_world_ptr_->GetSimuParam().is_complete_path;
  path_planner_input.sample_ds = apa_world_ptr_->GetSimuParam().sample_ds;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;
  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_replan_second = frame_.is_replan_second;
  path_planner_input.is_replan_dynamic =
      (frame_.replan_reason == ReplanReason::DYNAMIC);

  path_planner_input.is_left_empty = frame_.is_left_empty;
  path_planner_input.is_right_empty = frame_.is_right_empty;

  if (frame_.replan_reason == ReplanReason::DYNAMIC &&
      frame_.gear_command == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    ILOG_INFO << "dynamic replan, gear should be reverse/drive";
    path_planner_input.ref_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
  }

  perpendicular_path_planner_.SetInput(path_planner_input);

  // std::cout << "input gear = " << static_cast<int>(frame_.current_gear)
  //           << " input steer = " <<
  //           static_cast<int>(frame_.current_arc_steer)
  //           << std::endl;

  // need replan all path
  const bool path_plan_success = perpendicular_path_planner_.Update(
      apa_world_ptr_->GetCollisionDetectorPtr());

  uint8_t plan_result = 0;
  if (!path_plan_success && !apa_world_ptr_->GetSimuParam().is_simulation &&
      frame_.replan_reason != ReplanReason::DYNAMIC) {
    ILOG_INFO << "path plan fail";
    plan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
    current_plan_path_vec_.clear();
    current_path_point_global_vec_.clear();
    return plan_result;
  }

  if (!path_plan_success && frame_.replan_reason == ReplanReason::DYNAMIC) {
    ILOG_INFO << "path dynamic plan fail, save last plan path.";
    plan_result = PathPlannerResult::PLAN_UPDATE;
    frame_.dynamic_plan_fail_flag = true;
    return plan_result;
  }

  plan_result = PathPlannerResult::PLAN_UPDATE;

  if (!perpendicular_path_planner_.SetCurrentPathSegIndex()) {
    ILOG_INFO << "path plan fail";
    plan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = SET_SEG_INDEX;
    return plan_result;
  }

  // perpendicular_path_planner_.PrintOutputSegmentsInfo();

  perpendicular_path_planner_.InsertLineSegAfterCurrentFollowLastPath(
      apa_param.GetParam().path_extend_distance);

  if (!perpendicular_path_planner_.CheckCurrentGearLength()) {
    ILOG_INFO << "path plan fail";
    plan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = CHECK_GEAR_LENGTH;
    return plan_result;
  }

  perpendicular_path_planner_.SampleCurrentPathSeg();
  perpendicular_path_planner_.PrintOutputSegmentsInfo();

  const auto& planner_output = perpendicular_path_planner_.GetOutput();
  current_plan_path_vec_.clear();
  current_plan_path_vec_.reserve(5);
  if (false) {
    for (size_t i = planner_output.path_seg_index.first;
         i <= planner_output.path_seg_index.second; ++i) {
      const auto& path_seg_local = planner_output.path_segment_vec[i];
      pnc::geometry_lib::PathSegment path_seg_global = path_seg_local;
      if (path_seg_local.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
        path_seg_global.line_seg.pA =
            ego_info_under_slot.l2g_tf.GetPos(path_seg_local.GetLineSeg().pA);
        path_seg_global.line_seg.pB =
            ego_info_under_slot.l2g_tf.GetPos(path_seg_local.GetLineSeg().pB);
        path_seg_global.line_seg.heading =
            ego_info_under_slot.l2g_tf.GetHeading(
                path_seg_local.GetLineSeg().heading);
      } else if (path_seg_local.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        path_seg_global.arc_seg.pA =
            ego_info_under_slot.l2g_tf.GetPos(path_seg_local.GetArcSeg().pA);
        path_seg_global.arc_seg.pB =
            ego_info_under_slot.l2g_tf.GetPos(path_seg_local.GetArcSeg().pB);
        path_seg_global.arc_seg.circle_info.center =
            ego_info_under_slot.l2g_tf.GetPos(
                path_seg_local.GetArcSeg().circle_info.center);
        path_seg_global.arc_seg.headingA =
            ego_info_under_slot.l2g_tf.GetHeading(
                path_seg_local.GetArcSeg().headingA);
        path_seg_global.arc_seg.headingB =
            ego_info_under_slot.l2g_tf.GetHeading(
                path_seg_local.GetArcSeg().headingB);
      }
      current_plan_path_vec_.emplace_back(std::move(path_seg_global));
    }
  }

  ILOG_INFO << "current gear = " << static_cast<int>(frame_.current_gear)
            << " current steer = "
            << static_cast<int>(frame_.current_arc_steer);

  // std::cout << "is_replan_first = " << frame_.is_replan_first
  //           << " is_replan_second = " << frame_.is_replan_second
  //           << " gear_shift = " << planner_output.gear_shift
  //           << " replan_reason = " << static_cast<int>(frame_.replan_reason)
  //           << std::endl;
  if (frame_.is_replan_first) {
    frame_.current_gear = path_planner_input.ref_gear;
    frame_.current_arc_steer = path_planner_input.ref_arc_steer;
  }
  const bool gear_steer_shift =
      (frame_.is_replan_first && planner_output.gear_shift) ||
      (frame_.is_replan_second && planner_output.gear_shift) ||
      (!frame_.is_replan_first && !frame_.is_replan_second &&
       frame_.replan_reason != ReplanReason::DYNAMIC);

  // std::cout << "gear shift = " << gear_steer_shift << std::endl;

  if (gear_steer_shift) {
    ILOG_INFO << "next plan should shift gear";
    // set current arc steer
    if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else {
      ILOG_INFO << "fault ref_arc_steer state!";
      return false;
    }

    // set current gear
    if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    } else if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    } else {
      ILOG_INFO << "fault ref_gear state!";
      return false;
    }

    // std::cout << "next gear = " << static_cast<int>(frame_.current_gear)
    //           << " next steer = " <<
    //           static_cast<int>(frame_.current_arc_steer)
    //           << std::endl;
  }

  if (frame_.is_replan_first) {
    frame_.is_replan_first = false;
    // if (planner_output.gear_shift) {
    //   frame_.is_replan_second = false;
    // } else {
    //   frame_.is_replan_second = true;
    // }
  } else {
    if (frame_.is_replan_second) {
      frame_.is_replan_second = false;
    }
  }

  if (!path_plan_success) {
    return plan_result;
  }

  frame_.gear_command = planner_output.current_gear;

  // lateral path optimization
  bool is_use_optimizer = true;

  // refuse optimizer
  if (planner_output.path_point_vec.size() < 3) {
    ILOG_INFO << " input size is too small";
    is_use_optimizer = false;
  } else {
    const auto path_length = (planner_output.path_point_vec.front().pos -
                              planner_output.path_point_vec.back().pos)
                                 .norm();
    if (path_length < apa_param.GetParam().min_opt_path_length) {
      ILOG_INFO << "path length is too short, optimizer is closed ";
      is_use_optimizer = false;
    }
  }

  bool cilqr_optimization_enable = true;
  bool perpendicular_optimization_enable = true;

  if (!apa_world_ptr_->GetSimuParam().is_simulation) {
    perpendicular_optimization_enable =
        apa_param.GetParam().perpendicular_lat_opt_enable;

    cilqr_optimization_enable =
        apa_param.GetParam().cilqr_path_optimization_enable;

  } else {
    perpendicular_optimization_enable =
        apa_world_ptr_->GetSimuParam().is_path_optimization;
    cilqr_optimization_enable =
        apa_world_ptr_->GetSimuParam().is_cilqr_optimization;
  }

  double lat_path_opt_cost_time_ms = 0.0;
  if (perpendicular_optimization_enable && is_use_optimizer) {
    ILOG_INFO << "------------------------ lateral path optimization "
                 "------------------------";
    ILOG_INFO << "gear_command = " << static_cast<int>(frame_.gear_command);
    ILOG_INFO << "origin path size= " << planner_output.path_point_vec.size();

    LateralPathOptimizer::Parameter param;
    param.sample_ds = apa_world_ptr_->GetSimuParam().sample_ds;
    param.q_ref_xy = apa_world_ptr_->GetSimuParam().q_ref_xy;
    param.q_ref_theta = apa_world_ptr_->GetSimuParam().q_ref_theta;
    param.q_terminal_xy = apa_world_ptr_->GetSimuParam().q_terminal_xy;
    param.q_terminal_theta = apa_world_ptr_->GetSimuParam().q_terminal_theta;
    param.q_k = apa_world_ptr_->GetSimuParam().q_k;
    param.q_u = apa_world_ptr_->GetSimuParam().q_u;
    param.q_k_bound = apa_world_ptr_->GetSimuParam().q_k_bound;
    param.q_u_bound = apa_world_ptr_->GetSimuParam().q_u_bound;

    apa_world_ptr_->GetLateralPathOptimizerPtr()->Init(
        cilqr_optimization_enable);
    apa_world_ptr_->GetLateralPathOptimizerPtr()->SetParam(param);
    auto time_start = IflyTime::Now_ms();
    apa_world_ptr_->GetLateralPathOptimizerPtr()->Update(
        planner_output.path_point_vec, frame_.gear_command);

    auto time_end = IflyTime::Now_ms();
    lat_path_opt_cost_time_ms = time_end - time_start;

    const auto& optimized_path_vec =
        apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputPathVec();
    // TODO: longitudinal path optimization
    current_path_point_global_vec_.clear();
    current_path_point_global_vec_.reserve(optimized_path_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto& path_point : optimized_path_vec) {
      global_point.Set(
          ego_info_under_slot.l2g_tf.GetPos(path_point.pos),
          ego_info_under_slot.l2g_tf.GetHeading(path_point.heading));

      current_path_point_global_vec_.emplace_back(global_point);
    }
    const auto plan_debug_info =
        apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();

    ILOG_INFO << "lat_path_opt_cost_time_ms = " << lat_path_opt_cost_time_ms;

    ILOG_INFO << "terminal point error = "
              << plan_debug_info.terminal_pos_error();

    ILOG_INFO << "terminal heading error = "
              << plan_debug_info.terminal_heading_error();
  } else {
    current_path_point_global_vec_.clear();
    current_path_point_global_vec_.reserve(
        planner_output.path_point_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto& path_point : planner_output.path_point_vec) {
      global_point.Set(
          ego_info_under_slot.l2g_tf.GetPos(path_point.pos),
          ego_info_under_slot.l2g_tf.GetHeading(path_point.heading));

      current_path_point_global_vec_.emplace_back(global_point);
    }
  }

  JSON_DEBUG_VALUE("cilqr_optimization_enable", cilqr_optimization_enable);
  JSON_DEBUG_VALUE("lat_path_opt_cost_time_ms", lat_path_opt_cost_time_ms);

  ILOG_INFO << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size();

  return plan_result;
}

const bool PerpendicularHeadInScenario::CheckFinished() {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const bool lon_condition = ego_info_under_slot.terminal_err.pos.x() <
                             apa_param.GetParam().finish_lon_err;

  const double y1 = ego_info_under_slot.terminal_err.pos.y();

  const double y2 = (ego_info_under_slot.terminal_err.pos -
                     (apa_param.GetParam().wheel_base +
                      apa_param.GetParam().front_overhanging) *
                         ego_info_under_slot.slot.processed_corner_coord_local_
                             .pt_23mid_01mid_unit_vec)
                        .y();

  const bool lat_condition_1 =
      std::fabs(y1) <= apa_param.GetParam().finish_lat_err;

  const bool lat_condition_2 =
      std::fabs(y1) <= apa_param.GetParam().finish_lat_err_strict &&
      std::fabs(y2) <= apa_param.GetParam().finish_lat_err_strict;

  const bool heading_condition_1 =
      std::fabs(ego_info_under_slot.terminal_err.heading) <=
      apa_param.GetParam().finish_heading_err * kDeg2Rad;

  const bool heading_condition_2 =
      std::fabs(ego_info_under_slot.terminal_err.heading) <=
      (apa_param.GetParam().finish_heading_err + 1.988) * kDeg2Rad;

  const bool lat_condition = (lat_condition_1 && heading_condition_1) &&
                             (lat_condition_2 && heading_condition_2);

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const bool remain_s_condition =
      frame_.remain_dist_path < apa_param.GetParam().max_replan_remain_dist;

  // ILOG_INFO << "lon_condition = " << lon_condition
  //           << " lat_condition = " << lat_condition
  //           << " static_condition = " << static_condition
  //           << " remain_s_condition = " << remain_s_condition;

  bool parking_finish =
      lon_condition && lat_condition && static_condition && remain_s_condition;

  if (parking_finish) {
    return true;
  }

  // stucked by directly front uss
  const std::shared_ptr<UssObstacleAvoidance>& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetColDetInterfacePtr()->GetUssObsAvoidancePtr();
  const bool enter_slot_condition =
      ego_info_under_slot.slot_occupied_ratio >
      apa_param.GetParam().finish_uss_slot_occupied_ratio;
  const bool remain_uss_condition =
      frame_.remain_dist_obs < apa_param.GetParam().max_replan_remain_dist;
  // TODO headin stuck by front uss for headin park
  if (uss_obstacle_avoider_ptr->CheckIsDirectlyFrontUss()) {
    parking_finish = lat_condition && static_condition &&
                     enter_slot_condition && remain_uss_condition;
  }

  if (parking_finish) {
    return true;
  }

  parking_finish = lat_condition && static_condition && enter_slot_condition &&
                   (ego_info_under_slot.terminal_err.pos.x() < 0.4001);

  return parking_finish;
}

const bool PerpendicularHeadInScenario::PostProcessPath() {
  size_t origin_trajectory_size = current_path_point_global_vec_.size();

  if (origin_trajectory_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: origin_trajectory_size = " << origin_trajectory_size;
    frame_.plan_fail_reason = POST_PROCESS_PATH_POINT_SIZE;
    return false;
  }

  const size_t trajectory_size = origin_trajectory_size + 1;

  // head
  std::vector<double> headin_x_vec;
  std::vector<double> headin_y_vec;
  std::vector<double> headin_s_vec;
  headin_x_vec.clear();
  headin_y_vec.clear();
  headin_s_vec.clear();

  headin_x_vec.resize(trajectory_size);
  headin_y_vec.resize(trajectory_size);
  headin_s_vec.resize(trajectory_size);
  // tail
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
  x_vec.clear();
  y_vec.clear();
  s_vec.clear();

  x_vec.resize(trajectory_size);
  y_vec.resize(trajectory_size);
  s_vec.resize(trajectory_size);

  double headin_s = 0.0;
  double headin_ds = 0.0;
  double s = 0.0;
  double ds = 0.0;
  for (size_t i = 0; i < origin_trajectory_size; ++i) {
    headin_x_vec[i] = current_path_point_global_vec_[i].pos.x() +
                      apa_param.GetParam().wheel_base *
                          std::cos(current_path_point_global_vec_[i].heading);

    headin_y_vec[i] = current_path_point_global_vec_[i].pos.y() +
                      apa_param.GetParam().wheel_base *
                          std::sin(current_path_point_global_vec_[i].heading);

    x_vec[i] = current_path_point_global_vec_[i].pos.x();
    y_vec[i] = current_path_point_global_vec_[i].pos.y();

    if (i == 0) {
      headin_s_vec[i] = headin_s;
      s_vec[i] = s;
    } else {
      headin_ds = std::hypot(headin_x_vec[i] - headin_x_vec[i - 1],
                             headin_y_vec[i] - headin_y_vec[i - 1]);
      headin_s += std::max(headin_ds, 1e-3);
      headin_s_vec[i] = headin_s;

      ds = std::hypot(x_vec[i] - x_vec[i - 1], y_vec[i] - y_vec[i - 1]);
      s += std::max(ds, 1e-3);
      s_vec[i] = s;
    }
  }

  frame_.current_path_length = s;
  frame_.headin_current_path_length = headin_s;
  // calculate the extended point and insert

  Eigen::Vector2d headin_end_point(headin_x_vec[origin_trajectory_size - 1],
                                   headin_y_vec[origin_trajectory_size - 1]);

  Eigen::Vector2d end_point(x_vec[origin_trajectory_size - 1],
                            y_vec[origin_trajectory_size - 1]);

  const double end_point_heading =
      current_path_point_global_vec_[origin_trajectory_size - 1].heading;

  Eigen::Vector2d headin_extended_point;
  // head in
  headin_extended_point.x() =
      headin_end_point.x() +
      frame_.path_extended_dist * std::cos(end_point_heading);

  headin_extended_point.y() =
      headin_end_point.y() +
      frame_.path_extended_dist * std::sin(end_point_heading);

  headin_x_vec[origin_trajectory_size] = headin_extended_point.x();
  headin_y_vec[origin_trajectory_size] = headin_extended_point.y();
  headin_s_vec[origin_trajectory_size] =
      headin_s_vec[origin_trajectory_size - 1] + frame_.path_extended_dist;

  frame_.headin_x_s_spline.set_points(headin_s_vec, headin_x_vec);
  frame_.headin_y_s_spline.set_points(headin_s_vec, headin_y_vec);
  // tain in
  Eigen::Vector2d extended_point;
  extended_point.x() =
      end_point.x() + frame_.path_extended_dist * std::cos(end_point_heading);

  extended_point.y() =
      end_point.y() + frame_.path_extended_dist * std::sin(end_point_heading);

  x_vec[origin_trajectory_size] = extended_point.x();
  y_vec[origin_trajectory_size] = extended_point.y();

  s_vec[origin_trajectory_size] =
      s_vec[origin_trajectory_size - 1] + frame_.path_extended_dist;

  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);
  // ILOG_INFO << << "post process spline success";
  frame_.spline_success = true;

  return true;
}

const bool PerpendicularHeadInScenario::PostProcessPathAccordingLimiter() {
  size_t origin_traj_size = current_path_point_global_vec_.size();

  if (origin_traj_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: origin_traj_size = " << origin_traj_size;
    return false;
  }

  // cal proj point, need extend, add a point
  // if need extend according limiter, need to add another point
  const size_t traj_size = origin_traj_size + 1;
  // head
  std::vector<double> headin_x_vec;
  std::vector<double> headin_y_vec;
  std::vector<double> headin_s_vec;
  headin_x_vec.clear();
  headin_y_vec.clear();
  headin_s_vec.clear();

  headin_x_vec.reserve(traj_size);
  headin_y_vec.reserve(traj_size);
  headin_s_vec.reserve(traj_size);
  // tail
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
  std::vector<double> heading_vec;
  x_vec.clear();
  y_vec.clear();
  s_vec.clear();
  heading_vec.clear();

  x_vec.reserve(traj_size);
  y_vec.reserve(traj_size);
  s_vec.reserve(traj_size);
  heading_vec.reserve(traj_size);

  bool success = false;
  double s_proj = 0.0;
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  // to avoid one of the front wheel crash to limiter when heading in
  Eigen::Vector2d limit_mid_local =
      0.5 * (ego_info_under_slot.virtual_limiter.first +
             ego_info_under_slot.virtual_limiter.second);

  const double limiter_length = (ego_info_under_slot.virtual_limiter.first -
                                 ego_info_under_slot.virtual_limiter.second)
                                    .norm();

  limit_mid_local.y() -=
      0.5 * limiter_length *
      (ego_info_under_slot.cur_pose.heading_vec.y() /
       std::fabs(ego_info_under_slot.cur_pose.heading_vec.y()));

  const Eigen::Vector2d limiter_mid =
      ego_info_under_slot.l2g_tf.GetPos(limit_mid_local);

  success = pnc::geometry_lib::CalProjFromSplineByBisection(
      0.0, frame_.headin_current_path_length, s_proj, limiter_mid,
      frame_.headin_x_s_spline, frame_.headin_y_s_spline);

  // ILOG_INFO << "limiter_mid = " << limiter_mid.transpose()
  //                              << ", limit_mid_local = "
  //                              << limit_mid_local.transpose());
  if (!success) {
    ILOG_INFO << "path is err";
    return false;
  }
  if (s_proj < apa_world_ptr_->GetSimuParam().sample_ds * 1.5) {
    ILOG_INFO << "limiter s_proj is too small";
    return false;
  }

  double headin_s = 0.0;
  double headin_ds = 0.0;
  double s = 0.0;
  double ds = 0.0;
  for (size_t i = 0; i < current_path_point_global_vec_.size(); ++i) {
    // ILOG_INFO << << i << " s = " << s << " headin_s = " << headin_s
    //           << " s_proj = " << s_proj;

    if (i > 0) {
      ds = std::hypot(current_path_point_global_vec_[i].pos.x() -
                          current_path_point_global_vec_[i - 1].pos.x(),
                      current_path_point_global_vec_[i].pos.y() -
                          current_path_point_global_vec_[i - 1].pos.y());

      s += std::max(ds, 1e-3);

      headin_ds = std::hypot(
          current_path_point_global_vec_[i].pos.x() +
              apa_param.GetParam().wheel_base *
                  std::cos(current_path_point_global_vec_[i].heading) -
              current_path_point_global_vec_[i - 1].pos.x() -
              apa_param.GetParam().wheel_base *
                  std::cos(current_path_point_global_vec_[i - 1].heading),
          current_path_point_global_vec_[i].pos.y() +
              apa_param.GetParam().wheel_base *
                  std::sin(current_path_point_global_vec_[i].heading) -
              current_path_point_global_vec_[i - 1].pos.y() -
              apa_param.GetParam().wheel_base *
                  std::sin(current_path_point_global_vec_[i - 1].heading));

      headin_s += std::max(headin_ds, 1e-3);
    }
    if (headin_s > s_proj) {
      ILOG_INFO << "path shoule be shorten because of limiter";
      // ILOG_INFO << << " first point = " << headin_x_vec.back() << " "
      //           << headin_y_vec.back() << " " << heading_vec.back() *
      //           kRad2Deg;

      if (s_proj - headin_s_vec.back() > 0.036 && frame_.spline_success) {
        heading_vec.emplace_back(heading_vec.back());

        headin_x_vec.emplace_back(frame_.headin_x_s_spline(s_proj));
        headin_y_vec.emplace_back(frame_.headin_y_s_spline(s_proj));
        headin_s_vec.emplace_back(s_proj);
        // ILOG_INFO << << " last point = " << headin_x_vec.back() << " "
        //           << headin_y_vec.back() << " "
        //           << heading_vec.back() * kRad2Deg;

        // ILOG_INFO << << " real first point = " << x_vec.back() << " "
        //           << y_vec.back() << " " << heading_vec.back() * kRad2Deg;
        x_vec.emplace_back(frame_.x_s_spline(s - headin_s + s_proj));

        y_vec.emplace_back(frame_.y_s_spline(s - headin_s + s_proj));

        s_vec.emplace_back(s - headin_s + s_proj);
        // ILOG_INFO << << " real last point = " << x_vec.back() << " "
        //           << y_vec.back() << " "
        //           << heading_vec.back() * kRad2Deg;
      }
      break;
    }
    heading_vec.emplace_back(current_path_point_global_vec_[i].heading);

    headin_x_vec.emplace_back(
        current_path_point_global_vec_[i].pos.x() +
        apa_param.GetParam().wheel_base *
            std::cos(current_path_point_global_vec_[i].heading));

    headin_y_vec.emplace_back(
        current_path_point_global_vec_[i].pos.y() +
        apa_param.GetParam().wheel_base *
            std::sin(current_path_point_global_vec_[i].heading));

    headin_s_vec.emplace_back(headin_s);

    x_vec.emplace_back(current_path_point_global_vec_[i].pos.x());
    y_vec.emplace_back(current_path_point_global_vec_[i].pos.y());
    s_vec.emplace_back(s);
  }
  // ILOG_INFO << "s_proj = " << s_proj
  //                         << " s_origin = " << frame_.current_path_length);
  frame_.current_path_length = s_vec.back();
  frame_.headin_current_path_length = headin_s_vec.back();
  // ILOG_INFO << "s_new = " << frame_.current_path_length);
  const size_t N = x_vec.size();
  if (N < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: no enough point = " << x_vec.size();
    return false;
  }

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(N);
  pnc::geometry_lib::PathPoint path_point;
  for (size_t i = 0; i < N; ++i) {
    path_point.Set(Eigen::Vector2d(x_vec[i], y_vec[i]), heading_vec[i]);
    current_path_point_global_vec_.emplace_back(path_point);
  }

  // need extend by cal proj point
  Eigen::Vector2d extended_point;
  // head
  extended_point.x() = headin_x_vec.back() +
                       frame_.path_extended_dist * std::cos(heading_vec.back());

  extended_point.y() = headin_y_vec.back() +
                       frame_.path_extended_dist * std::sin(heading_vec.back());

  headin_x_vec.emplace_back(extended_point.x());
  headin_y_vec.emplace_back(extended_point.y());
  headin_s_vec.emplace_back(frame_.headin_current_path_length +
                            frame_.path_extended_dist);

  frame_.headin_x_s_spline.set_points(headin_s_vec, headin_x_vec);
  frame_.headin_y_s_spline.set_points(headin_s_vec, headin_y_vec);
  // tail
  extended_point.x() =
      x_vec.back() + frame_.path_extended_dist * std::cos(heading_vec.back());

  extended_point.y() =
      y_vec.back() + frame_.path_extended_dist * std::sin(heading_vec.back());

  x_vec.emplace_back(extended_point.x());
  y_vec.emplace_back(extended_point.y());
  s_vec.emplace_back(frame_.current_path_length + frame_.path_extended_dist);

  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;

  return true;
}

void PerpendicularHeadInScenario::Log() const {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const geometry_lib::LocalToGlobalTf& l2g_tf = ego_info_under_slot.l2g_tf;

  const std::vector<Eigen::Vector2d>& obstacles =
      apa_world_ptr_->GetCollisionDetectorPtr()->GetObstacles();

  std::vector<double> obstaclesX;
  obstaclesX.clear();
  obstaclesX.reserve(obstacles.size());
  std::vector<double> obstaclesY;
  obstaclesY.clear();
  obstaclesY.reserve(obstacles.size());
  for (const Eigen::Vector2d& obstacle : obstacles) {
    const Eigen::Vector2d tmp_obstacle = l2g_tf.GetPos(obstacle);
    obstaclesX.emplace_back(tmp_obstacle.x());
    obstaclesY.emplace_back(tmp_obstacle.y());
  }
  const std::unordered_map<size_t, std::vector<Eigen::Vector2d>>&
      obstacles_map =
          apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap();

  for (const auto& obs_pair : obstacles_map) {
    if (obs_pair.first == CollisionDetector::RECORD_OBS) {
      continue;
    }
    for (const auto& obstacle : obs_pair.second) {
      if (obs_pair.first == CollisionDetector::FUSION_OBS &&
          !(std::fabs(obstacle.y()) < 1.568) && obstacle.x() < 5.568 &&
          obstacle.x() > 0.268) {
        continue;
      }
      const Eigen::Vector2d tmp_obstacle = l2g_tf.GetPos(obstacle);
      obstaclesX.emplace_back(tmp_obstacle.x());
      obstaclesY.emplace_back(tmp_obstacle.y());
    }
  }

  const size_t max_count = 798;
  if (obstaclesX.size() > max_count) {
    obstaclesX.resize(max_count);
    obstaclesY.resize(max_count);
  }
  JSON_DEBUG_VECTOR("obstaclesX", obstaclesX, 2)
  JSON_DEBUG_VECTOR("obstaclesY", obstaclesY, 2)

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

  JSON_DEBUG_VECTOR("slot_corner_X", slot_corner_X, 3)
  JSON_DEBUG_VECTOR("slot_corner_Y", slot_corner_Y, 3)

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

  JSON_DEBUG_VECTOR("limiter_corner_X", limiter_corner_X, 2)
  JSON_DEBUG_VECTOR("limiter_corner_Y", limiter_corner_Y, 2)

  JSON_DEBUG_VALUE("terminal_error_x", ego_info_under_slot.terminal_err.pos.x())
  JSON_DEBUG_VALUE("terminal_error_y", ego_info_under_slot.terminal_err.pos.y())
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

  JSON_DEBUG_VALUE("selected_slot_id", ego_info_under_slot.id)
  JSON_DEBUG_VALUE("slot_length", ego_info_under_slot.slot.slot_length_)
  JSON_DEBUG_VALUE("slot_width", ego_info_under_slot.slot.slot_width_)

  JSON_DEBUG_VALUE("slot_origin_pos_x",
                   ego_info_under_slot.origin_pose_global.pos.x())

  JSON_DEBUG_VALUE("slot_origin_pos_y",
                   ego_info_under_slot.origin_pose_global.pos.y())

  JSON_DEBUG_VALUE("slot_origin_heading",
                   ego_info_under_slot.origin_pose_global.heading)

  JSON_DEBUG_VALUE("slot_occupied_ratio",
                   ego_info_under_slot.slot_occupied_ratio)

  std::vector<double> target_ego_pos_slot = {
      ego_info_under_slot.target_pose.pos.x(),
      ego_info_under_slot.target_pose.pos.y()};

  JSON_DEBUG_VALUE("pathplan_result", frame_.pathplan_result)
  JSON_DEBUG_VECTOR("target_ego_pos_slot", target_ego_pos_slot, 2)

  const auto& path_plan_output = perpendicular_path_planner_.GetOutput();
  JSON_DEBUG_VALUE("path_start_seg_index",
                   path_plan_output.path_seg_index.first)
  JSON_DEBUG_VALUE("path_end_seg_index", path_plan_output.path_seg_index.second)
  JSON_DEBUG_VALUE("path_length", path_plan_output.length)

  const UssObstacleAvoidance::RemainDistInfo uss_info =
      apa_world_ptr_->GetColDetInterfacePtr()
          ->GetUssObsAvoidancePtr()
          ->GetRemainDistInfo();
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
}

}  // namespace apa_planner

}  // namespace planning