#include "parallel_park_in_scenario.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_slot_manager.h"
#include "apa_utils.h"
#include "apa_world.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "func_state_machine_c.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "log_glog.h"
#include "math_lib.h"
#include "obstacle.h"
#include "parallel_path_generator.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

namespace planning {
namespace apa_planner {
static double kInsertLineLonBuffer = 0.2;
static double kFrontDetaXMagWhenFrontVacant = 1.98;
static double kFrontMaxDetaXMagWhenFrontOccupied = 0.5;
static double kRearDetaXMagWhenFrontVacant = 0.4;
static double kRearDetaXMagWhenBothSidesVacant = 0.2;
static double kRearDetaXMagWhenFrontOccupiedRearVacant = 1.2;
static double kRearDetaXMagWhenFrontVacantRearOccupied = 0.2;
static double kRearMaxDetaXMagWhenRearOccupied = 0.5;
static double kFrontObsLineYMagIdentification = 0.6;
static double kRearObsLineYMagIdentification = 0.6;
static double kCurbInitialOffset = 0.46;
static double kCurbYMagIdentification = 0.0;
static double kMaxDistDeleteObsToEgoInSlot = 0.3;
static double kMaxDistDeleteObsToEgoOutSlot = 0.35;
static double kMinChannelYMagIdentification = 3.3;
static double kExtendLengthOutsideSlot = 0.4;
static double kDeletedObsDistOutSlot = 0.3;
static double kDeletedObsDistInSlot = 0.10;
static double kTBoundarySampleDist = 0.38;
static double kChannelSampleDist = 0.46;
static double kEnterMultiPlanSlotRatio = 0.1;
static double kEps = 1e-5;

void ParallelParkInScenario::Reset() {
  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  parallel_path_planner_.Reset();
  current_path_point_global_vec_.clear();

  memset(&apa_hmi_, 0, sizeof(apa_hmi_));
  memset(&planning_output_, 0, sizeof(planning_output_));

  ParkingScenario::Reset();
}

void ParallelParkInScenario::ExcutePathPlanningTask() {
  // init simulation
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
    }
    return;
  }

  double lat_buffer = 0.0;
  double safe_uss_remain_dist = 0.0;

  if (apa_world_ptr_->GetSlotManagerPtr()
          ->ego_info_under_slot_.slot_occupied_ratio < 0.05) {
    lat_buffer = 0.09;
    safe_uss_remain_dist = apa_param.GetParam().safe_uss_remain_dist_out_slot;

    const auto& path_planner_output = parallel_path_planner_.GetOutputPtr();
    const auto& cur_path_end_pose =
        path_planner_output
            ->path_segment_vec[path_planner_output->path_seg_index.second]
            .GetEndPose();

    // first enter slot
    if (frame_.current_gear == geometry_lib::SEG_GEAR_REVERSE &&
        std::fabs(cur_path_end_pose.pos.y()) <
            0.5 * apa_world_ptr_->GetSlotManagerPtr()
                      ->ego_info_under_slot_.slot.slot_width_) {
      lat_buffer = 0.04;
      safe_uss_remain_dist = 0.12;
    }
  } else {
    lat_buffer = 0.0;
    safe_uss_remain_dist =
        apa_param.GetParam().safe_uss_remain_dist_in_parallel_slot;
  }

  ILOG_INFO << "parallel lat_buffer = " << lat_buffer;
  ILOG_INFO << "parallel safe_uss_remain_dist = " << safe_uss_remain_dist;

  // calculate remain dist according to plan path
  frame_.remain_dist_path = CalRemainDistFromPath();

  // calculate remain dist uss according to uss
  frame_.remain_dist_obs = CalRemainDistFromObs(
      safe_uss_remain_dist, lat_buffer, 0.0, lat_buffer, true);

  ILOG_INFO << "final remain_dist_obs = " << frame_.remain_dist_obs;

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ILOG_INFO << "update ego slot info";
    SetParkingStatus(PARKING_FAILED);
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
    return;
  }

  const double max_replan_path_dist = 0.15;
  // check replan
  if (!CheckReplan(max_replan_path_dist, 0.068,
                   apa_param.GetParam().max_replan_remain_dist,
                   apa_param.GetParam().uss_stuck_replan_wait_time,
                   apa_param.GetParam().stuck_replan_time)) {
    ILOG_INFO << "replan is not required!";
    SetParkingStatus(PARKING_RUNNING);
    return;
  }

  ILOG_INFO << "replan is required!";

  // generate t-lane
  GenTlane();

  // update obstacles
  GenTBoundaryObstacles();

  // path plan
  const auto pathplan_result = PathPlanOnce();
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
  // print planning status
  // ILOG_INFO << "parking status = "
  //           << static_cast<int>(GetPlannerStates().planning_status)
  //           ;
}

const bool ParallelParkInScenario::UpdateEgoSlotInfo() {
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  auto& select_slot_global =
      ego_info_under_slot.slot.origin_corner_coord_global_;
  ILOG_INFO << " select_slot_global pt 0 = "
            << select_slot_global.pt_0.transpose();
  ILOG_INFO << " select_slot_global pt 1 = "
            << select_slot_global.pt_1.transpose();
  ILOG_INFO << " select_slot_global pt 2 = "
            << select_slot_global.pt_2.transpose();
  ILOG_INFO << " select_slot_global pt 3 = "
            << select_slot_global.pt_3.transpose();

  if (select_slot_global.pt_0 == select_slot_global.pt_1 ||
      select_slot_global.pt_0 == select_slot_global.pt_2 ||
      select_slot_global.pt_0 == select_slot_global.pt_3 ||
      select_slot_global.pt_1 == select_slot_global.pt_2 ||
      select_slot_global.pt_1 == select_slot_global.pt_3 ||
      select_slot_global.pt_2 == select_slot_global.pt_3) {
    ILOG_ERROR << "slot corner points exist same pt!";
    return false;
  }

  const auto measures_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();

  Eigen::Vector2d v_10 =
      (select_slot_global.pt_0 - select_slot_global.pt_1).normalized();

  if (v_10.dot(measures_ptr->GetHeadingVec()) < 1e-9) {
    v_10 *= -1.0;
    select_slot_global.pt_0.swap(select_slot_global.pt_1);
    select_slot_global.pt_2.swap(select_slot_global.pt_3);
  }
  select_slot_global.CalExtraCoord();
  const double heading_10 = std::atan2(v_10.y(), v_10.x());

  const pnc::geometry_lib::LineSegment line_01(
      select_slot_global.pt_1, select_slot_global.pt_0, heading_10);

  const double dist_01_2 =
      pnc::geometry_lib::CalPoint2LineDist(select_slot_global.pt_2, line_01);

  const double dist_01_3 =
      pnc::geometry_lib::CalPoint2LineDist(select_slot_global.pt_3, line_01);

  ego_info_under_slot.slot.slot_width_ = std::min(dist_01_2, dist_01_3);
  ego_info_under_slot.slot.slot_length_ =
      (select_slot_global.pt_0 - select_slot_global.pt_1).norm();
  ILOG_INFO << "slot_length_ = " << ego_info_under_slot.slot.slot_length_;
  ILOG_INFO << "slot_width = " << ego_info_under_slot.slot.slot_width_;

  // calc slot side once at first
  if (frame_.is_replan_first) {
    const auto v_23mid_to_01 =
        ego_info_under_slot.slot.origin_corner_coord_global_.pt_23mid_01mid_vec
            .normalized();

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->GetHeadingVec(),
                                                v_23mid_to_01);
    ILOG_INFO << "v_23mid_to_01 " << v_23mid_to_01.transpose();

    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_slot_heading > 1e-3) {
      t_lane_.slot_side_sgn = 1.0;
      t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;

    } else if (cross_ego_to_slot_heading < -1e-3) {
      t_lane_.slot_side_sgn = -1.0;
      t_lane_.slot_side = geometry_lib::SLOT_SIDE_LEFT;
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;

    } else {
      t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
      ILOG_INFO << "calculate parallel slot side error ";
      return false;
    }
  }
  ILOG_INFO << "slot_side = " << static_cast<int>(t_lane_.slot_side);

  const Eigen::Vector2d n =
      (select_slot_global.pt_0 - select_slot_global.pt_1).normalized();
  ILOG_INFO << "n = " << n.transpose();
  const Eigen::Vector2d t(-n.y(), n.x());
  ILOG_INFO << "t = " << t.transpose();

  ego_info_under_slot.origin_pose_global.heading = std::atan2(n.y(), n.x());
  ILOG_INFO << " ego_info_under_slot.origin_pose_global.heading = "
            << ego_info_under_slot.origin_pose_global.heading * kRad2Deg;
  ego_info_under_slot.origin_pose_global.heading_vec = n;

  const double target_y_sgn =
      (ego_info_under_slot.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT
           ? 1.0
           : -1.0);
  const double target_y = 0.5 * ego_info_under_slot.slot.GetWidth();

  ego_info_under_slot.origin_pose_global.pos =
      select_slot_global.pt_1 + target_y * target_y_sgn * t;

  ILOG_INFO << "ego_info_under_slot.origin_pose_global.pos = "
            << ego_info_under_slot.origin_pose_global.pos.transpose();

  ego_info_under_slot.g2l_tf = geometry_lib::GlobalToLocalTf(
      ego_info_under_slot.origin_pose_global.pos,
      ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.l2g_tf = geometry_lib::LocalToGlobalTf(
      ego_info_under_slot.origin_pose_global.pos,
      ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.origin_pose_local.pos << 0.0, 0.0;
  ego_info_under_slot.origin_pose_local.heading = 0.0;
  ego_info_under_slot.origin_pose_local.heading_vec << 1.0, 0.0;
  ego_info_under_slot.slot.TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  ego_info_under_slot.cur_pose.pos =
      ego_info_under_slot.g2l_tf.GetPos(measures_ptr->GetPos());
  ego_info_under_slot.cur_pose.heading =
      ego_info_under_slot.g2l_tf.GetHeading(measures_ptr->GetHeading());
  ego_info_under_slot.cur_pose.heading_vec =
      pnc::geometry_lib::GenHeadingVec(ego_info_under_slot.cur_pose.heading);

  ILOG_INFO << "ego_pos_slot = "
            << ego_info_under_slot.cur_pose.pos.transpose();
  ILOG_INFO << "ego_heading_slot (deg)= "
            << ego_info_under_slot.cur_pose.heading * kRad2Deg;

  ILOG_INFO << "slot width =" << ego_info_under_slot.slot.slot_width_;
  ILOG_INFO << "t_lane_.slot_side = " << static_cast<int>(t_lane_.slot_side);
  ILOG_INFO << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer);
  ego_info_under_slot.target_pose.pos
      << 0.5 * (ego_info_under_slot.slot.slot_length_ -
                apa_param.GetParam().car_length) +
             apa_param.GetParam().rear_overhanging,
      0.0;
  ego_info_under_slot.target_pose.heading = 0.0;
  ego_info_under_slot.target_pose.heading_vec << 1.0, 0.0;

  // calc terminal error once
  ego_info_under_slot.terminal_err.Set(
      ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
      pnc::geometry_lib::NormalizeAngle(
          ego_info_under_slot.cur_pose.heading -
          ego_info_under_slot.target_pose.heading));

  // calc slot occupied ratio
  double slot_occupied_ratio = 0.0;
  if (pnc::mathlib::IsInBound(ego_info_under_slot.terminal_err.pos.x(), -3.0,
                              4.0)) {
    const double y_err_ratio = ego_info_under_slot.terminal_err.pos.y() /
                               (0.5 * ego_info_under_slot.slot.slot_width_);

    if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  ego_info_under_slot.slot_occupied_ratio = slot_occupied_ratio;
  ILOG_INFO << "ego_slot_info.slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;
  return true;
}

const bool ParallelParkInScenario::GenTlane() {
  // Todo: generate t-lane according to nearby obstacles

  // y
  // ^_______________    left side
  // |               |
  // |->x            |
  // |               |
  //
  // y                  ego car------->>
  // ^
  // |               |
  // |->x            |
  // |_______________|   right side

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  const double slot_length = ego_info_under_slot.slot.GetLength();
  const double slot_width = ego_info_under_slot.slot.GetWidth();
  const double half_slot_width = 0.5 * slot_width;
  const double quarter_slot_width = 0.5 * half_slot_width;
  const double side_sgn = t_lane_.slot_side_sgn;
  const Eigen::Vector2d slot_center(0.5 * slot_length, 0.0);
  obs_pt_local_vec_.clear();
  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  for (const auto& pair :
       apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles()) {
    for (const auto& obs_pt_local : pair.second.GetPtClout2dLocal()) {
      if ((obs_pt_local - slot_center).norm() > 20.0) {
        continue;
      }

      // outof total box range
      if (!pnc::mathlib::IsInBound(
              obs_pt_local.x(), -5.0,
              apa_param.GetParam().parallel_channel_x_mag)) {
        // total_box_x_fail_cnt++;
        continue;
      }

      if (obs_pt_local.y() * side_sgn >
              apa_param.GetParam().parallel_channel_y_mag ||
          obs_pt_local.y() * side_sgn <
              (-0.5 * t_lane_.slot_width - apa_param.GetParam().curb_offset)) {
        // total_box_y_fail_cnt++;
        continue;
      }

      // remote front T-boundary obs
      if (obs_pt_local.x() >
              t_lane_.slot_length + kFrontDetaXMagWhenFrontVacant &&
          obs_pt_local.y() * side_sgn < -0.3 * side_sgn) {
        // front_box_fail_cnt++;
        continue;
      }

      // remote rear T-boundary obs
      if (obs_pt_local.x() < -5.0 &&
          obs_pt_local.y() * side_sgn < -0.3 * side_sgn) {
        // rear_box_fail_cnt++;
        continue;
      }

      if (apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs_pt_local, ego_info_under_slot.cur_pose,
              kDeletedObsDistOutSlot)) {
        // in_ego_cnt++;
        continue;
      }
      obs_pt_local_vec_.emplace_back(std::move(obs_pt_local));
    }
  }
  // set initial x coordination for front and rear tlane obs
  double front_min_x = slot_length + kFrontDetaXMagWhenFrontVacant;
  double rear_max_x = -kRearDetaXMagWhenFrontOccupiedRearVacant;

  // set initial y coordination for front and rear y tlane obs
  double front_parallel_line_y_limit = side_sgn * quarter_slot_width;
  double rear_parallel_line_y_limit = front_parallel_line_y_limit;

  // channel
  double channel_x_limit = apa_param.GetParam().parallel_channel_x_mag;
  double channel_y_limit =
      side_sgn * apa_param.GetParam().parallel_channel_y_mag;

  // curb
  size_t curb_count = 0;
  double curb_y_limit = -side_sgn * (half_slot_width + kCurbInitialOffset);

  for (const auto& obstacle_point_slot : obs_pt_local_vec_) {
    const bool front_obs_condition =
        pnc::mathlib::IsInBound(
            obstacle_point_slot.x(),
            slot_length - kFrontMaxDetaXMagWhenFrontOccupied,
            slot_length + kFrontDetaXMagWhenFrontVacant) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(),
            -kFrontObsLineYMagIdentification * side_sgn,
            (half_slot_width + kFrontObsLineYMagIdentification) * side_sgn);

    if (front_obs_condition) {
      front_min_x = std::min(front_min_x, obstacle_point_slot.x());

      // ILOG_INFO<<"front_obs_condition!");
    }

    const bool rear_obs_condition =
        ((pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                  -kRearDetaXMagWhenFrontOccupiedRearVacant,
                                  kRearMaxDetaXMagWhenRearOccupied)) &&
         (pnc::mathlib::IsInBound(obstacle_point_slot.y(),
                                  -side_sgn * kRearObsLineYMagIdentification,
                                  (half_slot_width + 0.1) * side_sgn)));

    if (rear_obs_condition) {
      rear_max_x = std::max(rear_max_x, obstacle_point_slot.x());
      // ILOG_INFO<<"rear_obs_condition!");
    }

    const bool curb_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), 1.0,
                                slot_length - 1.0) &&
        (obstacle_point_slot.y() * side_sgn <= -kCurbYMagIdentification);

    if (curb_condition) {
      curb_count++;
      if (side_sgn > 0.0) {
        curb_y_limit = std::max(curb_y_limit, obstacle_point_slot.y());
      } else {
        curb_y_limit = std::min(curb_y_limit, obstacle_point_slot.y());
      }

      // ILOG_INFO<<"curb condition!");
    }

    const bool front_parallel_line_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), slot_length - 0.2,
                                slot_length + 1.6) &&
        pnc::mathlib::IsInBound(obstacle_point_slot.y(), 0.0, 2.2 * side_sgn);

    if (front_parallel_line_condition) {
      front_parallel_line_y_limit =
          side_sgn > 0.0
              ? std::max(front_parallel_line_y_limit, obstacle_point_slot.y())
              : std::min(front_parallel_line_y_limit, obstacle_point_slot.y());
      // ILOG_INFO<<"front_parallel_line_y_limit condition!");
    }

    const bool rear_parallel_line_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), -2.5, 0.2) &&
        pnc::mathlib::IsInBound(obstacle_point_slot.y() * side_sgn, 0.0, 2.3);

    if (rear_parallel_line_condition) {
      rear_parallel_line_y_limit =
          side_sgn > 0.0
              ? std::max(rear_parallel_line_y_limit, obstacle_point_slot.y())
              : std::min(rear_parallel_line_y_limit, obstacle_point_slot.y());
      // ILOG_INFO<<"rear_parallel_line_y_limit condition!");
    }
  }

  bool front_vacant = false;
  bool rear_vacant = false;

  if (front_min_x >= slot_length + kFrontDetaXMagWhenFrontVacant - kEps) {
    front_vacant = true;
    ILOG_INFO << "front space empty!";
  }

  if (rear_max_x <= -kRearDetaXMagWhenFrontOccupiedRearVacant + kEps) {
    rear_vacant = true;
    ILOG_INFO << "rear space empty!";
  }
  if (front_vacant && rear_vacant) {
    front_min_x = slot_length + kFrontDetaXMagWhenFrontVacant;
    rear_max_x = -kRearDetaXMagWhenBothSidesVacant;

  } else if (front_vacant && !rear_vacant) {
    front_min_x = slot_length + kFrontDetaXMagWhenFrontVacant;
    // protection for rear x due to the low accuracy of rear uss obstacle
    // points
    rear_max_x =
        std::max(rear_max_x, -kRearDetaXMagWhenFrontVacantRearOccupied);

  } else if (!front_vacant && rear_vacant) {
    rear_max_x = -kRearDetaXMagWhenFrontOccupiedRearVacant;
  } else {
  }

  ILOG_INFO << "front_vacant = " << front_vacant;
  ILOG_INFO << "rear_vacant = " << rear_vacant;

  ILOG_INFO << "front_min_x before clamp = " << front_min_x;
  front_min_x = pnc::mathlib::Clamp(
      front_min_x, slot_length - kRearDetaXMagWhenFrontVacant,
      slot_length + kFrontDetaXMagWhenFrontVacant);
  ILOG_INFO << "front_min_x after clamp =" << front_min_x;

  const double front_y_limit = front_parallel_line_y_limit;
  ILOG_INFO << "front parallel line y =" << front_y_limit;

  ILOG_INFO << "rear_max_x before clamp = " << rear_max_x;
  rear_max_x =
      pnc::mathlib::Clamp(rear_max_x, -kRearDetaXMagWhenFrontOccupiedRearVacant,
                          kRearMaxDetaXMagWhenRearOccupied);
  if (rear_vacant &&
      ego_info_under_slot.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
    rear_max_x =
        std::min(rear_max_x, ego_info_under_slot.cur_pose.pos.x() - 1.6);
  }

  ILOG_INFO << "rear_max_x after clamp = " << rear_max_x;

  ILOG_INFO << "rear parallel line y =" << rear_parallel_line_y_limit;

  t_lane_.obs_pt_inside << front_min_x, front_y_limit;
  t_lane_.obs_pt_outside << rear_max_x, rear_parallel_line_y_limit;

  curb_y_limit = pnc::mathlib::Clamp(
      curb_y_limit, -side_sgn * (half_slot_width + kCurbInitialOffset),
      -side_sgn * (half_slot_width - kCurbInitialOffset));

  if (ego_info_under_slot.slot_occupied_ratio < 0.01) {
    curb_y_limit +=
        side_sgn * apa_param.GetParam().curb_offset_when_ego_outside_slot;
  }

  t_lane_.corner_inside_slot << slot_length, half_slot_width * side_sgn;
  t_lane_.corner_outside_slot << 0.0, half_slot_width * side_sgn;

  t_lane_.curb_y = curb_y_limit;
  t_lane_.channel_y = channel_y_limit;
  t_lane_.channel_x_limit = channel_x_limit;

  t_lane_.pt_outside = t_lane_.obs_pt_outside;
  t_lane_.pt_inside = t_lane_.obs_pt_inside;

  t_lane_.slot_length = slot_length;
  t_lane_.slot_width = slot_width;

  const double target_y_with_curb =
      curb_y_limit +
      side_sgn * (apa_param.GetParam().terminal_parallel_y_offset_with_curb +
                  0.5 * apa_param.GetParam().car_width);

  ego_info_under_slot.target_pose.pos.y() =
      (side_sgn > 0.0 ? std::max(0.0, target_y_with_curb)
                      : std::min(0.0, target_y_with_curb));

  const double target_x_in_slot_center =
      0.5 * (ego_info_under_slot.slot.slot_length_ -
             apa_param.GetParam().car_length) +
      apa_param.GetParam().rear_overhanging;

  ego_info_under_slot.target_pose.pos.x() = target_x_in_slot_center;
  t_lane_.pt_terminal_pos = ego_info_under_slot.target_pose.pos;

  ego_info_under_slot.terminal_err.Set(
      ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
      pnc::geometry_lib::NormalizeAngle(
          ego_info_under_slot.cur_pose.heading -
          ego_info_under_slot.target_pose.heading));

  ILOG_INFO << "-- t_lane --------";
  if (pnc::mathlib::IsDoubleEqual(side_sgn, 1.0)) {
    ILOG_INFO << "right slot";
  } else if (pnc::mathlib::IsDoubleEqual(side_sgn, -1.0)) {
    ILOG_INFO << "left slot!";
  }
  ILOG_INFO << "obs_pt_inside = " << t_lane_.obs_pt_inside.transpose();
  ILOG_INFO << "obs_pt_outside = " << t_lane_.obs_pt_outside.transpose();
  ILOG_INFO << "corner_inside_slot = "
            << t_lane_.corner_inside_slot.transpose();
  ILOG_INFO << "corner_outside_slot = "
            << t_lane_.corner_outside_slot.transpose();

  ILOG_INFO << "pt_outside = " << t_lane_.pt_outside.transpose();
  ILOG_INFO << "pt_inside = " << t_lane_.pt_inside.transpose();
  ILOG_INFO << "pt_terminal_pos = " << t_lane_.pt_terminal_pos.transpose();
  ILOG_INFO << "slot length =" << t_lane_.slot_length;
  ILOG_INFO << "half slot width =" << 0.5 * t_lane_.slot_width;
  ILOG_INFO << "curb y =" << t_lane_.curb_y;
  ILOG_INFO << "channel x =" << t_lane_.channel_x_limit;
  ILOG_INFO << "channel y =" << t_lane_.channel_y;
  ILOG_INFO << "------------------";

  JSON_DEBUG_VALUE("para_tlane_obs_in_x", t_lane_.obs_pt_inside.x())
  JSON_DEBUG_VALUE("para_tlane_obs_in_y", t_lane_.obs_pt_inside.y())
  JSON_DEBUG_VALUE("para_tlane_obs_out_x", t_lane_.obs_pt_outside.x())
  JSON_DEBUG_VALUE("para_tlane_obs_out_x", t_lane_.obs_pt_outside.y())

  std::vector<double> x_vec = {0.0};
  std::vector<double> y_vec = {0.0};
  std::vector<double> phi_vec = {0.0};

  JSON_DEBUG_VECTOR("col_det_path_x", x_vec, 2)
  JSON_DEBUG_VECTOR("col_det_path_y", y_vec, 2)
  JSON_DEBUG_VECTOR("col_det_path_phi", phi_vec, 2)

  return true;
}

const bool ParallelParkInScenario::GenObstacles() { return true; }

void ParallelParkInScenario::GenTBoundaryObstacles() {
  //                                                  |
  //                         c-------------D
  //                         |             |  pin
  //            A -----------B pout        E---------F
  //                         |       ego-------->     |
  //  channel pt1 ----------------------------------- channel pt2
  //                                                  |
  //                         |       ego-------->     |
  //            A -----------B pout        E---------F
  //                         |             |  pin
  //                         c-------------D

  apa_world_ptr_->GetCollisionDetectorPtr()->Reset();
  const auto& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  // set T-Boundary obstacles
  const Eigen::Vector2d B(t_lane_.obs_pt_outside.x(),
                          0.3 * t_lane_.slot_side_sgn);

  const Eigen::Vector2d A(B.x() - 3.2, B.y());
  const Eigen::Vector2d E(
      t_lane_.obs_pt_inside.x(),
      t_lane_.obs_pt_inside.y() - t_lane_.slot_side_sgn * 0.8);

  const Eigen::Vector2d C(
      B.x(), (-0.5 * t_lane_.slot_width - apa_param.GetParam().curb_offset) *
                 t_lane_.slot_side_sgn);
  const Eigen::Vector2d D(E.x(), C.y());
  const Eigen::Vector2d F(t_lane_.channel_x_limit, E.y());

  const Eigen::Vector2d channel_point_1(
      A.x(),
      apa_param.GetParam().parallel_channel_y_mag * t_lane_.slot_side_sgn);

  const Eigen::Vector2d channel_point_2(
      F.x(),
      apa_param.GetParam().parallel_channel_y_mag * t_lane_.slot_side_sgn);

  const pnc::geometry_lib::LineSegment channel_line(channel_point_1,
                                                    channel_point_2);

  // sample channel boundary line
  std::vector<Eigen::Vector2d> channel_line_obs_vec;
  std::vector<Eigen::Vector2d> filtered_channel_obs_vec;
  pnc::geometry_lib::SamplePointSetInLineSeg(channel_line_obs_vec, channel_line,
                                             kChannelSampleDist);
  for (const auto& obs : channel_line_obs_vec) {
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs, ego_info_under_slot.cur_pose, kDeletedObsDistOutSlot)) {
      filtered_channel_obs_vec.emplace_back(obs);
    }
  }

  for (const auto& obstacle_point_slot : obs_pt_local_vec_) {
    // add obs near channel
    const bool channel_y_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), channel_point_1.x(),
                                channel_point_2.x()) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(),
            kMinChannelYMagIdentification * t_lane_.slot_side_sgn,
            channel_point_1.y());

    if (channel_y_condition) {
      filtered_channel_obs_vec.emplace_back(obstacle_point_slot);

      if (pnc::mathlib::IsInBound(obstacle_point_slot.x(), t_lane_.slot_length,
                                  t_lane_.slot_length + 5.0)) {
        if (t_lane_.slot_side_sgn > 0.0) {
          t_lane_.channel_y =
              std::min(obstacle_point_slot.y(), t_lane_.channel_y);
        } else {
          t_lane_.channel_y =
              std::max(obstacle_point_slot.y(), t_lane_.channel_y);
        }
      }
    }
  }
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      filtered_channel_obs_vec, CollisionDetector::CHANNEL_OBS);

  // set tlane obs
  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;
  // set tlane parallel line obs
  tlane_line.SetPoints(A, B);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(E, F);
  tlane_line_vec.emplace_back(tlane_line);

  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> tlane_obstacle_vec;

  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line,
                                               kTBoundarySampleDist);

    for (const auto& obs : point_set) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs, ego_info_under_slot.cur_pose, kDeletedObsDistOutSlot)) {
        tlane_obstacle_vec.emplace_back(obs);
      }
    }
  }

  // tlane vertical line
  for (const auto& obstacle_point_slot : obs_pt_local_vec_) {
    const bool is_rear_tlane_line =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), A.x(),
                                t_lane_.obs_pt_outside.x()) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(), B.y(),
            kMinChannelYMagIdentification * t_lane_.slot_side_sgn);

    if (is_rear_tlane_line) {
      tlane_obstacle_vec.emplace_back(obstacle_point_slot);
      continue;
    }

    const bool is_front_tlane_line =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                t_lane_.obs_pt_inside.x() - 0.3, F.x()) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(), E.y(),
            kMinChannelYMagIdentification * t_lane_.slot_side_sgn);

    if (is_front_tlane_line) {
      tlane_obstacle_vec.emplace_back(obstacle_point_slot);
    }
  }

  // tlane
  tlane_line_vec.clear();

  const Eigen::Vector2d C_curb(C.x(), t_lane_.curb_y);
  const Eigen::Vector2d D_curb(D.x(), t_lane_.curb_y);

  tlane_line.SetPoints(B, C_curb);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(D_curb, E);
  tlane_line_vec.emplace_back(tlane_line);

  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line,
                                               kTBoundarySampleDist);

    for (const auto& obs : point_set) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs, ego_info_under_slot.cur_pose, kDeletedObsDistInSlot)) {
        tlane_obstacle_vec.emplace_back(obs);
      }
    }
  }

  for (const auto& obs_pos : obs_pt_local_vec_) {
    const bool is_tlane_obs =
        pnc::mathlib::IsInBound(obs_pos.x(), B.x(), E.x()) &&
        pnc::mathlib::IsInBound(obs_pos.y(), t_lane_.obs_pt_inside.y(),
                                C_curb.y());
    if (!is_tlane_obs) {
      continue;
    }

    if (pnc::mathlib::IsInBound(obs_pos.x(), 0.5, t_lane_.slot_length - 0.5) &&
        pnc::mathlib::IsInBound(obs_pos.y(), -0.4 * t_lane_.slot_side_sgn,
                                1.2 * t_lane_.slot_side_sgn)) {
      // obs noise in slot
      continue;
    }

    const bool is_front_tlane_obs =
        pnc::mathlib::IsInBound(obs_pos.x(), t_lane_.obs_pt_inside.x(),
                                E.x()) &&
        pnc::mathlib::IsInBound(obs_pos.y(), t_lane_.obs_pt_inside.y(),
                                D_curb.y());
    const bool is_rear_tlane_obs =
        pnc::mathlib::IsInBound(obs_pos.x(), B.x(),
                                t_lane_.obs_pt_outside.x()) &&
        pnc::mathlib::IsInBound(obs_pos.y(), 1.5 * t_lane_.slot_side_sgn,
                                C_curb.y());

    if (is_front_tlane_obs || is_rear_tlane_obs) {
      tlane_obstacle_vec.emplace_back(obs_pos);
    }
  }
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      tlane_obstacle_vec, CollisionDetector::TLANE_BOUNDARY_OBS);

  point_set.clear();
  tlane_obstacle_vec.clear();
  tlane_line.SetPoints(C_curb, D_curb);
  pnc::geometry_lib::SamplePointSetInLineSeg(point_set, tlane_line,
                                             kTBoundarySampleDist);

  for (const auto& obs : point_set) {
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs, ego_info_under_slot.cur_pose, kDeletedObsDistInSlot)) {
      tlane_obstacle_vec.emplace_back(obs);
    }
  }

  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      tlane_obstacle_vec, CollisionDetector::CURB_OBS);
}

const uint8_t ParallelParkInScenario::PathPlanOnce() {
  // construct input
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;
  GeometryPathInput path_planner_input;
  path_planner_input.tlane = t_lane_;
  path_planner_input.sample_ds = apa_world_ptr_->GetSimuParam().sample_ds;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_complete_path =
      apa_world_ptr_->GetSimuParam().is_complete_path;
  path_planner_input.ego_info_under_slot = ego_info_under_slot;

  if (frame_.is_replan_first) {
    // temprarily give driving gear
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;

    frame_.current_arc_steer =
        t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT
            ? pnc::geometry_lib::SEG_STEER_LEFT
            : pnc::geometry_lib::SEG_STEER_RIGHT;

    path_planner_input.path_planner_state =
        ParallelPathGenerator::PrepareStepPlan;
  } else if (path_planner_input.ego_info_under_slot.slot_occupied_ratio >
                 kEnterMultiPlanSlotRatio &&
             frame_.in_slot_plan_count == 0) {
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    frame_.current_arc_steer =
        t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT
            ? pnc::geometry_lib::SEG_STEER_RIGHT
            : pnc::geometry_lib::SEG_STEER_LEFT;
  }

  ILOG_INFO << "slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;
  ILOG_INFO << "frame_.in_slot_plan_count = " << frame_.in_slot_plan_count;

  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;

  ILOG_INFO << "ref gear to path planner input ="
            << static_cast<int>(path_planner_input.ref_gear);
  ILOG_INFO << "ref steer to path planner input ="
            << static_cast<int>(path_planner_input.ref_arc_steer);

  parallel_path_planner_.SetInput(path_planner_input);

  const double path_plan_start_time = IflyTime::Now_ms();

  const bool path_plan_success =
      parallel_path_planner_.Update(apa_world_ptr_->GetCollisionDetectorPtr());

  ILOG_INFO << "path planner cost time(ms) = "
            << IflyTime::Now_ms() - path_plan_start_time;
  // const auto& path_planner_output = parallel_path_planner_.GetOutput();

  uint8_t plan_result = 0;
  if (path_plan_success) {
    plan_result = PathPlannerResult::PLAN_UPDATE;
    ILOG_INFO << "path plan success!";
  } else {
    plan_result = PathPlannerResult::PLAN_FAILED;
    ILOG_INFO << "path plan fail!";
    return PathPlannerResult::PLAN_FAILED;
  }

  parallel_path_planner_.SetCurrentPathSegIndex();

  const auto path_planner_output = parallel_path_planner_.GetOutput();
  ILOG_INFO << "first seg idx = " << path_planner_output.path_seg_index.first;
  ILOG_INFO << "last seg idx = " << path_planner_output.path_seg_index.second;

  const auto& cur_path_end_pose =
      path_planner_output
          .path_segment_vec[path_planner_output.path_seg_index.second]
          .GetEndPose();

  const double x_diff_mag =
      std::fabs(cur_path_end_pose.pos.x() - t_lane_.pt_terminal_pos.x());

  const double y_diff_mag =
      std::fabs(cur_path_end_pose.pos.y() - t_lane_.pt_terminal_pos.y());

  const double heading_deg_diff_mag =
      std::fabs(cur_path_end_pose.heading * kRad2Deg);

  double current_path_length = 0.0;
  for (size_t i = path_planner_output.path_seg_index.first;
       i <= path_planner_output.path_seg_index.second; i++) {
    current_path_length += path_planner_output.path_segment_vec[i].Getlength();
  }

  // enter slot
  if (ego_info_under_slot.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
    double extend_lenth = 0.0;
    if (current_path_length < apa_param.GetParam().min_path_length) {
      extend_lenth = std::max(
          apa_param.GetParam().min_path_length - current_path_length, 0.1);

    } else {
      const double x_diff =
          std::fabs(cur_path_end_pose.pos.x() - t_lane_.pt_terminal_pos.x());

      if (heading_deg_diff_mag < 0.5 &&
          pnc::mathlib::IsInBound(x_diff, 0.1,
                                  apa_param.GetParam().min_path_length + 0.1)) {
        extend_lenth = apa_param.GetParam().min_path_length + 0.1 - x_diff;
      } else if (heading_deg_diff_mag > 10.0 &&
                 frame_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
        extend_lenth = apa_param.GetParam().min_path_length;
      } else if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
        extend_lenth = 0.0;
      }
    }
    parallel_path_planner_.InsertLineSegAfterCurrentFollowLastPath(
        extend_lenth, kInsertLineLonBuffer);
  } else {
    // outside slot normal extended length and buffer
    double lon_buffer = kInsertLineLonBuffer;
    double extend_lenth = kExtendLengthOutsideSlot;

    // ego is outof slot, current path is backward to slot
    if (path_planner_output.current_gear ==
            pnc::geometry_lib::SEG_GEAR_REVERSE &&
        y_diff_mag < t_lane_.slot_width * 0.5 &&
        mathlib::IsInBound(cur_path_end_pose.pos.x(), -1.5,
                           0.75 * t_lane_.slot_length) &&
        heading_deg_diff_mag > 5.0) {
      extend_lenth = 0.5;
      lon_buffer = 0.05;
    }

    parallel_path_planner_.InsertLineSegAfterCurrentFollowLastPath(extend_lenth,
                                                                   lon_buffer);
  }
  parallel_path_planner_.SampleCurrentPathSeg();

  frame_.total_plan_count++;
  if (ego_info_under_slot.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
    frame_.in_slot_plan_count++;
  }

  // print segment info
  // pnc::geometry_lib::PrintSegmentsVecInfo(
  //     parallel_path_planner_.GetOutput().path_segment_vec);

  // reverse info for next plan
  if (frame_.is_replan_first) {
    frame_.current_gear = pnc::geometry_lib::ReverseGear(
        parallel_path_planner_.GetOutput().gear_cmd_vec.front());

    ILOG_INFO << "next gear =" << static_cast<int>(frame_.current_gear);
  } else {
    // set current gear
    frame_.current_gear = pnc::geometry_lib::ReverseGear(frame_.current_gear);
    ILOG_INFO << "next gear =" << static_cast<int>(frame_.current_gear);

    if (!pnc::geometry_lib::IsValidGear(frame_.current_gear)) {
      ILOG_INFO << "frame_.current_gear == invalid gear!";
      return PathPlannerResult::PLAN_FAILED;
    }

    if (ego_info_under_slot.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
      // set current arc steer
      frame_.current_arc_steer =
          pnc::geometry_lib::ReverseSteer(frame_.current_arc_steer);

      if (!pnc::geometry_lib::IsValidArcSteer(frame_.current_arc_steer)) {
        ILOG_INFO << "frame_.current_arc == invalid arc!";
        return PathPlannerResult::PLAN_FAILED;
      }
    }
  }

  frame_.is_replan_first = false;

  const auto& planner_output = parallel_path_planner_.GetOutput();
  frame_.gear_command = planner_output.current_gear;

  ILOG_INFO << "start lat optimizer!";

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
  bool parallel_optimization_enable = true;
  if (!apa_world_ptr_->GetSimuParam().is_simulation) {
    parallel_optimization_enable = apa_param.GetParam().parallel_lat_opt_enable;
    cilqr_optimization_enable =
        apa_param.GetParam().cilqr_path_optimization_enable;

  } else {
    parallel_optimization_enable =
        apa_world_ptr_->GetSimuParam().is_path_optimization;
    cilqr_optimization_enable =
        apa_world_ptr_->GetSimuParam().is_cilqr_optimization;
  }

  double lat_path_opt_cost_time_ms = 0.0;
  if (parallel_optimization_enable && is_use_optimizer) {
    ILOG_INFO << "------------------------ lateral path optimization "
                 "------------------------";

    ILOG_INFO << "frame_.gear_command_= "
              << static_cast<int>(frame_.gear_command);

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

  JSON_DEBUG_VECTOR("plan_traj_x", std::vector<double>{0.0}, 3)
  JSON_DEBUG_VECTOR("plan_traj_y", std::vector<double>{0.0}, 3)
  JSON_DEBUG_VECTOR("plan_traj_heading", std::vector<double>{0.0}, 3)
  JSON_DEBUG_VECTOR("plan_traj_lat_buffer", std::vector<double>{0.0}, 3)

  JSON_DEBUG_VALUE("cilqr_optimization_enable", cilqr_optimization_enable);
  JSON_DEBUG_VALUE("lat_path_opt_cost_time_ms", lat_path_opt_cost_time_ms);

  ILOG_INFO << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size();

  return plan_result;
}

const bool ParallelParkInScenario::CheckFinished() {
  const auto& ego_slot_info =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  // const Eigen::Vector2d rear_bumper_center =
  //     ego_slot_info.ego_pos_slot - apa_param.GetParam().rear_overhanging *
  //                                      ego_slot_info.ego_heading_slot_vec;

  // const Eigen::Vector2d front_bumper_center =
  //     ego_slot_info.ego_pos_slot + (apa_param.GetParam().wheel_base +
  //                                   apa_param.GetParam().front_overhanging)
  //                                   *
  //                                      ego_slot_info.ego_heading_slot_vec;

  const bool lon_condition = std::fabs(ego_slot_info.terminal_err.pos.x()) <
                             apa_param.GetParam().finish_parallel_lon_err;

  ILOG_INFO << "terminal x error = " << ego_slot_info.terminal_err.pos.x();
  ILOG_INFO << "lon_condition = " << lon_condition;

  const bool heading_condition =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      apa_param.GetParam().finish_parallel_heading_err / 57.3;

  ILOG_INFO << "terminal heading error = "
            << ego_slot_info.terminal_err.heading * 57.3;
  ILOG_INFO << "heading_condition = " << heading_condition;

  ILOG_INFO << "lat error = " << ego_slot_info.terminal_err.pos.y();
  const bool lat_condition_1 = std::fabs(ego_slot_info.terminal_err.pos.y()) <=
                               apa_param.GetParam().finish_parallel_lat_rac_err;

  // lat condition 2, keep both outer wheels in slot
  const double side_sgn =
      t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT ? 1.0 : -1.0;

  pnc::geometry_lib::LocalToGlobalTf ego2slot;
  ego2slot.Init(ego_slot_info.cur_pose.pos, ego_slot_info.cur_pose.heading);

  const auto& front_out_wheel = ego2slot.GetPos(
      Eigen::Vector2d(apa_param.GetParam().wheel_base,
                      0.5 * apa_param.GetParam().car_width * side_sgn));

  const auto& rear_out_wheel = ego2slot.GetPos(
      Eigen::Vector2d(0.0, 0.5 * apa_param.GetParam().car_width * side_sgn));

  const double slot_outer_pt_y =
      0.5 * ego_slot_info.slot.slot_width_ * side_sgn;
  bool lat_condition_2 = false;
  if (side_sgn > 0.0) {
    const double wheel_limit_y =
        slot_outer_pt_y - apa_param.GetParam().finish_parallel_lat_err;
    lat_condition_2 = (rear_out_wheel.y() <= wheel_limit_y) &&
                      (front_out_wheel.y() <= wheel_limit_y);
  } else {
    const double wheel_limit_y =
        slot_outer_pt_y + apa_param.GetParam().finish_parallel_lat_err;
    lat_condition_2 = (rear_out_wheel.y() >= wheel_limit_y) &&
                      (front_out_wheel.y() >= wheel_limit_y);
  }
  const bool lat_condition = lat_condition_1 || lat_condition_2;

  ILOG_INFO << "terminal y error = " << ego_slot_info.terminal_err.pos.y();
  ILOG_INFO << "lat condition  = " << lat_condition;
  if (lat_condition) {
    if (lat_condition_1) {
      ILOG_INFO << "lat y err = " << ego_slot_info.terminal_err.pos.y() << " < "
                << apa_param.GetParam().finish_parallel_lat_rac_err;
    } else {
      ILOG_INFO << "ego outer wheel are both in slot!";
    }
  }

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  ILOG_INFO << "static_condition = " << static_condition;

  return lon_condition && lat_condition && heading_condition &&
         static_condition;
}

const double ParallelParkInScenario::CalcSlotOccupiedRatio(
    const Eigen::Vector2d& terminal_err, const double slot_width,
    const bool is_right_side) const {
  double slot_occupied_ratio = 0.0;
  if (pnc::mathlib::IsInBound(terminal_err.x(), -3.0, 4.0)) {
    const double y_err_ratio = terminal_err.y() / (0.5 * slot_width);

    if (is_right_side) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  return slot_occupied_ratio;
}

void ParallelParkInScenario::Log() const {
  const auto& obs_map =
      apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap();

  const auto& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  const auto& l2g_tf = ego_info_under_slot.l2g_tf;

  const auto p0_g = l2g_tf.GetPos(t_lane_.obs_pt_outside);
  const auto p1_g = l2g_tf.GetPos(t_lane_.obs_pt_inside);
  ILOG_INFO << "obs p out = " << p0_g.transpose();
  ILOG_INFO << "obs p in = " << p1_g.transpose();

  int obs_size = 0;
  for (const auto& obs_pair : obs_map) {
    if (obs_pair.first != CollisionDetector::VIRTUAL_OBS) {
      obs_size += obs_pair.second.size();
    }
  }
  ILOG_INFO << "obs_size = " << obs_size;
  const int count_unit = std::ceil(obs_size / 500.0);
  const size_t simplify_obs_num = std::ceil(obs_size / count_unit);

  size_t idx = 0;
  std::vector<double> obstaclesX;
  std::vector<double> obstaclesY;
  obstaclesX.reserve(simplify_obs_num);
  obstaclesY.reserve(simplify_obs_num);

  for (const auto& obs_pair : obs_map) {
    if (obs_pair.first == CollisionDetector::VIRTUAL_OBS) {
      continue;
    }

    for (const auto& obs : obs_pair.second) {
      if (idx % count_unit == 0) {
        const auto obs_g = l2g_tf.GetPos(obs);
        obstaclesX.emplace_back(obs_g.x());
        obstaclesY.emplace_back(obs_g.y());
      }
      idx++;
    }
  }
  ILOG_INFO << "obstaclesX simp size = " << obstaclesX.size();
  JSON_DEBUG_VECTOR("obstaclesX", obstaclesX, 2)
  JSON_DEBUG_VECTOR("obstaclesY", obstaclesY, 2)

  std::vector<double> limiter_corner_X = {-2.0, -2.0};
  std::vector<double> limiter_corner_Y = {1.2, -1.2};
  JSON_DEBUG_VECTOR("limiter_corner_X", limiter_corner_X, 2)
  JSON_DEBUG_VECTOR("limiter_corner_Y", limiter_corner_Y, 2)

  const auto& pts = ego_info_under_slot.slot.origin_corner_coord_global_;
  std::vector<double> slot_corner_X = {pts.pt_0.x(), pts.pt_1.x(), pts.pt_2.x(),
                                       pts.pt_3.x()};

  std::vector<double> slot_corner_Y = {pts.pt_0.y(), pts.pt_1.y(), pts.pt_2.y(),
                                       pts.pt_3.y()};
  JSON_DEBUG_VECTOR("slot_corner_X", slot_corner_X, 2)
  JSON_DEBUG_VECTOR("slot_corner_Y", slot_corner_Y, 2)

  JSON_DEBUG_VALUE("terminal_error_x", ego_info_under_slot.terminal_err.pos.x())
  JSON_DEBUG_VALUE("terminal_error_y", ego_info_under_slot.terminal_err.pos.y())
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
  JSON_DEBUG_VALUE("remain_dist_uss", frame_.remain_dist_obs)
  JSON_DEBUG_VALUE("stuck_time", frame_.stuck_time)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
  JSON_DEBUG_VALUE("ego_heading_slot", ego_info_under_slot.cur_pose.heading)

  JSON_DEBUG_VALUE("selected_slot_id", ego_info_under_slot.slot.id_)
  JSON_DEBUG_VALUE("slot_length", ego_info_under_slot.slot.GetLength())
  JSON_DEBUG_VALUE("slot_width", ego_info_under_slot.slot.GetWidth())

  JSON_DEBUG_VALUE("slot_origin_pos_x",
                   ego_info_under_slot.origin_pose_global.pos.x())

  JSON_DEBUG_VALUE("slot_origin_pos_y",
                   ego_info_under_slot.origin_pose_global.pos.y())

  JSON_DEBUG_VALUE("slot_origin_heading",
                   ego_info_under_slot.origin_pose_global.heading)

  JSON_DEBUG_VALUE("slot_occupied_ratio",
                   ego_info_under_slot.slot_occupied_ratio)

  const std::vector<double> target_ego_pos_slot = {
      ego_info_under_slot.target_pose.pos.x(),
      ego_info_under_slot.target_pose.pos.y()};

  JSON_DEBUG_VALUE("pathplan_result", frame_.pathplan_result)
  JSON_DEBUG_VECTOR("target_ego_pos_slot", target_ego_pos_slot, 2)

  const auto& path_plan_output = parallel_path_planner_.GetOutput();
  JSON_DEBUG_VALUE("path_start_seg_index",
                   path_plan_output.path_seg_index.first)
  JSON_DEBUG_VALUE("path_end_seg_index", path_plan_output.path_seg_index.second)
  JSON_DEBUG_VALUE("path_length", path_plan_output.length)

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
