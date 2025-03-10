#include "parallel_park_out_scenario.h"

#include <cstddef>

#include "apa_param_config.h"
#include "apa_state_machine_manager.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math_lib.h"

namespace planning {
namespace apa_planner {

static double kFrontDetaXMagWhenFrontVacant = 1.98;
static double kFrontMaxDetaXMagWhenFrontOccupied = 0.5;
static double kRearDetaXMagWhenFrontVacant = 0.4;
static double kRearDetaXMagWhenBothSidesVacant = 0.2;
static double kRearDetaXMagWhenFrontOccupiedRearVacant = 2.0;
static double kRearDetaXMagWhenFrontVacantRearOccupied = 0.2;
static double kRearMaxDetaXMagWhenRearOccupied = 0.5;

static double kFrontObsLineYMagIdentification = 0.6;
static double kRearObsLineYMagIdentification = 0.6;
static double kCurbInitialOffset = 0.46;
static double kCurbYMagIdentification = 0.0;
static double kMinChannelYMagIdentification = 3.3;

static double kTBoundarySampleDist = 0.38;
static double kChannelSampleDist = 0.46;
static double kEnterMultiPlanSlotRatio = 0.1;
static double kEps = 1e-5;
void ParallelParkOutScenario::Reset() {
  frame_.Reset();
  tlane_.Reset();
  obs_pt_local_vec_.clear();
  parallel_out_path_planner_.Reset();
  current_path_point_global_vec_.clear();

  memset(&apa_hmi_, 0, sizeof(apa_hmi_));
  memset(&planning_output_, 0, sizeof(planning_output_));

  ParkingScenario::Reset();
}

void ParallelParkOutScenario::ExcutePathPlanningTask() {
  ILOG_INFO << "---------------------parallel out ---------------------------";
  // init simulation
  InitSimulation();

  // check planning status
  if (!apa_world_ptr_->GetSimuParam().force_plan && CheckPlanSkip()) {
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
  } else {
    lat_buffer = 0.02;
    safe_uss_remain_dist =
        apa_param.GetParam().safe_uss_remain_dist_in_parallel_slot;
  }

  // update remain dist
  UpdateRemainDist(safe_uss_remain_dist, lat_buffer, 0.0);

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ILOG_INFO << "update ego slot info failed!";
    SetParkingStatus(PARKING_FAILED);
    return;
  }
  ILOG_INFO << "update ego slot info success!";

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

  // check replan
  if (CheckReplan() || apa_world_ptr_->GetSimuParam().force_plan) {
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
  } else {
    ILOG_INFO << "replan is not required!";
    SetParkingStatus(PARKING_RUNNING);
  }

  // print planning status
  ILOG_INFO << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status);
}

const bool ParallelParkOutScenario::UpdateEgoSlotInfo() {
  using namespace pnc::geometry_lib;

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

  if (frame_.is_replan_first) {
    const Eigen::Vector2d v_ego_to_01_mid =
        select_slot_global.pt_01_mid - measures_ptr->GetPos();

    const Eigen::Vector2d v_ego_heading = measures_ptr->GetHeadingVec();

    if (GetCrossFromTwoVec2d(v_ego_heading, v_ego_to_01_mid) < 1e-9) {
      select_slot_global.pt_0.swap(select_slot_global.pt_2);
      select_slot_global.pt_1.swap(select_slot_global.pt_3);
    }

    frame_.is_park_out_left =
        (apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection() ==
             ApaParkOutDirection::LEFT_FRONT ||
         apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection() ==
             ApaParkOutDirection::LEFT_REAR);
    ILOG_INFO << "park out direction = "
              << static_cast<int>(apa_world_ptr_->GetStateMachineManagerPtr()
                                      ->GetParkOutDirection());

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

    const Eigen::Vector2d n =
        (select_slot_global.pt_0 - select_slot_global.pt_1).normalized();
    const Eigen::Vector2d t(-n.y(), n.x());

    ego_info_under_slot.origin_pose_global.heading_vec = n;
    ego_info_under_slot.origin_pose_global.heading = std::atan2(n.y(), n.x());
    ILOG_INFO << " ego_info_under_slot.origin_pose_global.heading = "
              << ego_info_under_slot.origin_pose_global.heading * kRad2Deg;

    const Eigen::Vector2d v_31 =
        select_slot_global.pt_1 - select_slot_global.pt_3;

    double mov_y_sgn = 1.0;
    if (t.dot(v_31) > 1e-9) {
      mov_y_sgn = -1.0;
    }

    ego_info_under_slot.origin_pose_global.pos =
        0.5 * (select_slot_global.pt_1 + select_slot_global.pt_3);
    ILOG_INFO << "origin pos = "
              << ego_info_under_slot.origin_pose_global.pos.transpose();

    ego_info_under_slot.g2l_tf = pnc::geometry_lib::GlobalToLocalTf(
        ego_info_under_slot.origin_pose_global.pos,
        ego_info_under_slot.origin_pose_global.heading);

    ego_info_under_slot.l2g_tf = pnc::geometry_lib::LocalToGlobalTf(
        ego_info_under_slot.origin_pose_global.pos,
        ego_info_under_slot.origin_pose_global.heading);

    ego_info_under_slot.origin_pose_local.pos << 0.0, 0.0;
    ego_info_under_slot.origin_pose_local.heading = 0.0;
    ego_info_under_slot.origin_pose_local.heading_vec << 1.0, 0.0;
    ego_info_under_slot.slot.TransformCoordFromGlobalToLocal(
        ego_info_under_slot.g2l_tf);
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;

  } else {
    frame_.current_gear =
        ReverseGear(parallel_out_path_planner_.GetOutputPtr()->current_gear);
  }

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

  ego_info_under_slot.target_pose.pos << 0.0, 0.0;
  ego_info_under_slot.target_pose.heading = 0.0;

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
                               (0.4 * ego_info_under_slot.slot.slot_width_);

    if (frame_.is_park_out_left) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  ego_info_under_slot.slot_occupied_ratio = slot_occupied_ratio;
  ILOG_INFO << "ego_slot_info.slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;

  return true;
}

const bool ParallelParkOutScenario::CheckFinished() {
  const double slot_occupied_ratio =
      apa_world_ptr_->GetSlotManagerPtr()
          ->ego_info_under_slot_.slot_occupied_ratio;

  const double heading_mag_deg =
      std::fabs(apa_world_ptr_->GetSlotManagerPtr()
                    ->ego_info_under_slot_.cur_pose.heading *
                kRad2Deg);

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  ILOG_INFO << "heading_mag_deg = " << heading_mag_deg;
  ILOG_INFO << "static_condition = " << static_condition;
  ILOG_INFO << "slot_occupied_ratio = " << slot_occupied_ratio;

  return static_condition && slot_occupied_ratio < 0.1 &&
         heading_mag_deg < apa_param.GetParam().finish_parallel_out_heading_mag;
}

const bool ParallelParkOutScenario::GenTlane() {
  ILOG_INFO << "--------------- GenTlane ------------------------";
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
  const double side_sgn = frame_.is_park_out_left ? 1.0 : -1.0;
  ILOG_INFO << "frame_.is_park_out_left = " << frame_.is_park_out_left;

  const Eigen::Vector2d slot_center(0.5 * slot_length, 0.0);
  obs_pt_local_vec_.clear();
  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  for (const auto& pair :
       apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles()) {
    for (const auto& obs_pt_local : pair.second.GetPtClout2dLocal()) {
      if ((obs_pt_local - slot_center).norm() > 15.0) {
        continue;
      }

      // outof total box range
      if (!pnc::mathlib::IsInBound(
              obs_pt_local.x(), -1.5,
              apa_param.GetParam().parallel_channel_x_mag)) {
        // total_box_x_fail_cnt++;
        continue;
      }

      if (obs_pt_local.y() * side_sgn >
              apa_param.GetParam().parallel_channel_y_mag ||
          obs_pt_local.y() * side_sgn <
              (-0.5 * ego_info_under_slot.slot.slot_width_ -
               apa_param.GetParam().curb_offset)) {
        // total_box_y_fail_cnt++;
        continue;
      }

      if (apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs_pt_local, ego_info_under_slot.cur_pose, 0.3)) {
        // in_ego_cnt++;
        continue;
      }
      obs_pt_local_vec_.emplace_back((obs_pt_local));
    }
  }
  ILOG_INFO << "after obs filter";

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

  ILOG_INFO << "obs_pt_local_vec_ size =" << obs_pt_local_vec_.size();

  // filter obstacles meeting requirements that are in slot system
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
         (pnc::mathlib::IsInBound(
             obstacle_point_slot.y(),
             -side_sgn * kRearObsLineYMagIdentification,
             (half_slot_width + kRearObsLineYMagIdentification) * side_sgn)));

    if (rear_obs_condition) {
      rear_max_x = std::max(rear_max_x, obstacle_point_slot.x());
      // ILOG_INFO<<"rear_obs_condition!");
    }

    const bool curb_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                kRearDetaXMagWhenFrontVacant,
                                slot_length - kRearDetaXMagWhenFrontVacant) &&
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
    // protection for rear x due to the low accuracy of rear uss obstacle points
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

  tlane_.obs_pt_inside << front_min_x, front_y_limit;
  tlane_.obs_pt_outside << rear_max_x, rear_parallel_line_y_limit;

  curb_y_limit = pnc::mathlib::Clamp(
      curb_y_limit, -side_sgn * (half_slot_width + kCurbInitialOffset),
      -side_sgn * (half_slot_width - kCurbInitialOffset));

  if (ego_info_under_slot.slot_occupied_ratio > 0.01) {
    curb_y_limit -= side_sgn * 0.1;
  }

  tlane_.corner_inside_slot << slot_length, half_slot_width * side_sgn;
  tlane_.corner_outside_slot << 0.0, half_slot_width * side_sgn;

  tlane_.curb_y = curb_y_limit;
  tlane_.channel_y = channel_y_limit;
  tlane_.channel_x_limit = channel_x_limit;

  tlane_.pt_outside = tlane_.obs_pt_outside;
  tlane_.pt_inside = tlane_.obs_pt_inside;

  tlane_.slot_length = slot_length;
  tlane_.slot_width = slot_width;

  ILOG_INFO << "-- t_lane --------";
  if (pnc::mathlib::IsDoubleEqual(side_sgn, 1.0)) {
    ILOG_INFO << "park out left";
  } else if (pnc::mathlib::IsDoubleEqual(side_sgn, -1.0)) {
    ILOG_INFO << "park out right";
  }
  ILOG_INFO << "obs_pt_inside = " << tlane_.obs_pt_inside.transpose();
  ILOG_INFO << "obs_pt_outside = " << tlane_.obs_pt_outside.transpose();
  ILOG_INFO << "corner_inside_slot = " << tlane_.corner_inside_slot.transpose();
  ILOG_INFO << "corner_outside_slot = "
            << tlane_.corner_outside_slot.transpose();

  ILOG_INFO << "pt_outside = " << tlane_.pt_outside.transpose();
  ILOG_INFO << "pt_inside = " << tlane_.pt_inside.transpose();

  ILOG_INFO << "slot length =" << tlane_.slot_length;
  ILOG_INFO << "half slot width =" << 0.5 * tlane_.slot_width;
  ILOG_INFO << "curb y =" << tlane_.curb_y;
  ILOG_INFO << "channel x =" << tlane_.channel_x_limit;
  ILOG_INFO << "channel y =" << tlane_.channel_y;
  ILOG_INFO << "------------------";

  std::vector<double> x_vec = {0.0};
  std::vector<double> y_vec = {0.0};
  std::vector<double> phi_vec = {0.0};

  JSON_DEBUG_VECTOR("col_det_path_x", x_vec, 2)
  JSON_DEBUG_VECTOR("col_det_path_y", y_vec, 2)
  JSON_DEBUG_VECTOR("col_det_path_phi", phi_vec, 2)
  return true;
}

void ParallelParkOutScenario::GenTBoundaryObstacles() {
  //                         c-------------------------D
  //                         |                         |
  //                         |---->x                   |
  //                         |        ego-------->     |  pin
  //                         |                         |
  //            A -----------B pout                    E---------F
  //                                         |
  //                                         v
  //                                         park to right

  //  channel pt1 ------------------------------------------------- channel pt2
  //
  //                                         park to left
  //                                         ^
  //                         |               |         |
  //            A -----------B pout                    E---------F
  //                         |        ego-------->     |  pin
  //                         |---->x                   |
  //                         |                         |
  //                         c-------------------------D

  apa_world_ptr_->GetCollisionDetectorPtr()->Reset();

  const double side_sgn = (frame_.is_park_out_left ? 1.0 : -1.0);
  tlane_.slot_side_sgn = side_sgn;
  // set T-Boundary obstacles
  const Eigen::Vector2d B(tlane_.obs_pt_outside.x(),
                          0.5 * tlane_.slot_side_sgn);

  const Eigen::Vector2d A(B.x() - 3.2, B.y());
  const Eigen::Vector2d E(
      tlane_.obs_pt_inside.x(),
      tlane_.obs_pt_inside.y() - tlane_.slot_side_sgn * 0.5);

  const Eigen::Vector2d C(
      B.x(), (-0.5 * tlane_.slot_width - apa_param.GetParam().curb_offset) *
                 tlane_.slot_side_sgn);
  const Eigen::Vector2d D(E.x(), C.y());
  const Eigen::Vector2d F(tlane_.channel_x_limit, E.y());

  const Eigen::Vector2d channel_point_1(
      A.x(),
      apa_param.GetParam().parallel_channel_y_mag * tlane_.slot_side_sgn);

  const Eigen::Vector2d channel_point_2(
      F.x(),
      apa_param.GetParam().parallel_channel_y_mag * tlane_.slot_side_sgn);

  const pnc::geometry_lib::LineSegment channel_line(channel_point_1,
                                                    channel_point_2);

  // sample channel boundary line
  std::vector<Eigen::Vector2d> channel_obstacle_vec;
  pnc::geometry_lib::SamplePointSetInLineSeg(channel_obstacle_vec, channel_line,
                                             kChannelSampleDist);
  ILOG_INFO << "channel_obstacle_vec size = " << channel_obstacle_vec.size();

  for (const auto& obstacle_point_slot : obs_pt_local_vec_) {
    // add obs near channel
    const bool channel_y_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), channel_point_1.x(),
                                channel_point_2.x()) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(),
            kMinChannelYMagIdentification * tlane_.slot_side_sgn,
            channel_point_1.y());

    if (channel_y_condition) {
      channel_obstacle_vec.emplace_back(obstacle_point_slot);

      if (pnc::mathlib::IsInBound(obstacle_point_slot.x(), tlane_.slot_length,
                                  tlane_.slot_length + 3.0)) {
      }
    }
  }

  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      channel_obstacle_vec, CollisionDetector::CHANNEL_OBS);

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
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }

  // tlane vertical line
  for (const auto& obstacle_point_slot : obs_pt_local_vec_) {
    const bool is_rear_tlane_line =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), A.x(),
                                tlane_.obs_pt_outside.x()) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(), B.y(),
            kMinChannelYMagIdentification * tlane_.slot_side_sgn);

    if (is_rear_tlane_line) {
      tlane_obstacle_vec.emplace_back(obstacle_point_slot);
      continue;
    }

    const bool is_front_tlane_line =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                tlane_.obs_pt_inside.x() - 0.3, F.x()) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(), E.y(),
            kMinChannelYMagIdentification * tlane_.slot_side_sgn);

    if (is_front_tlane_line) {
      tlane_obstacle_vec.emplace_back(obstacle_point_slot);
    }
  }

  // tlane
  tlane_line_vec.clear();

  const Eigen::Vector2d C_curb(C.x(), tlane_.curb_y);
  const Eigen::Vector2d D_curb(D.x(), tlane_.curb_y);

  tlane_line.SetPoints(B, C_curb);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(D_curb, E);
  tlane_line_vec.emplace_back(tlane_line);

  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line,
                                               kTBoundarySampleDist);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }

  for (const auto& obs_pos : obs_pt_local_vec_) {
    const bool is_tlane_obs =
        pnc::mathlib::IsInBound(obs_pos.x(), B.x(), E.x()) &&
        pnc::mathlib::IsInBound(obs_pos.y(), tlane_.obs_pt_inside.y(),
                                C_curb.y());
    if (!is_tlane_obs) {
      continue;
    }

    if (pnc::mathlib::IsInBound(obs_pos.x(), 0.5, tlane_.slot_length - 0.5) &&
        pnc::mathlib::IsInBound(obs_pos.y(), -0.4 * tlane_.slot_side_sgn,
                                1.2 * tlane_.slot_side_sgn)) {
      // obs noise in slot
      continue;
    }

    const bool is_front_tlane_obs =
        pnc::mathlib::IsInBound(obs_pos.x(), tlane_.obs_pt_inside.x(), E.x()) &&
        pnc::mathlib::IsInBound(obs_pos.y(), tlane_.obs_pt_inside.y(),
                                D_curb.y());
    const bool is_rear_tlane_obs =
        pnc::mathlib::IsInBound(obs_pos.x(), B.x(),
                                tlane_.obs_pt_outside.x()) &&
        pnc::mathlib::IsInBound(obs_pos.y(), 1.5 * tlane_.slot_side_sgn,
                                C_curb.y());

    if (is_front_tlane_obs || is_rear_tlane_obs) {
      tlane_obstacle_vec.emplace_back(obs_pos);
    }
  }
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      tlane_obstacle_vec, CollisionDetector::TLANE_BOUNDARY_OBS);

  point_set.clear();
  tlane_line.SetPoints(C_curb, D_curb);
  pnc::geometry_lib::SamplePointSetInLineSeg(point_set, tlane_line,
                                             kTBoundarySampleDist);

  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      point_set, CollisionDetector::CURB_OBS);

  for (const auto& pair :
       apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap()) {
    if (pair.first == CollisionDetector::CHANNEL_OBS) {
      ILOG_INFO << "channel_obstacle_vec after foreach in obs = "
                << pair.second.size();
    }
  }
}

const uint8_t ParallelParkOutScenario::PathPlanOnce() {
  // construct input
  ParallelOutPathGenerator::Input path_planner_input;
  path_planner_input.tlane = tlane_;
  path_planner_input.sample_ds = apa_world_ptr_->GetSimuParam().sample_ds;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_complete_path =
      apa_world_ptr_->GetSimuParam().is_complete_path;

  const auto& ego_slot_info =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  path_planner_input.ego_pose = ego_slot_info.cur_pose;
  path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;

  if (frame_.is_replan_first) {
    // temprarily give driving gear
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;

    frame_.current_arc_steer = frame_.is_park_out_left
                                   ? geometry_lib::SEG_STEER_LEFT
                                   : geometry_lib::SEG_STEER_RIGHT;
  }

  ILOG_INFO << "ego_slot_info.slot_occupied_ratio = "
            << ego_slot_info.slot_occupied_ratio;

  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;

  ILOG_INFO << "ref gear to path planner input ="
            << static_cast<int>(path_planner_input.ref_gear);
  ILOG_INFO << "ref steer to path planner input ="
            << static_cast<int>(path_planner_input.ref_arc_steer);

  parallel_out_path_planner_.SetInput(path_planner_input);

  const double path_plan_start_time = IflyTime::Now_ms();

  const bool path_plan_success = parallel_out_path_planner_.Update(
      apa_world_ptr_->GetCollisionDetectorPtr());

  ILOG_INFO << "path planner cost time(ms) = "
            << IflyTime::Now_ms() - path_plan_start_time;

  uint8_t plan_result = 0;
  if (path_plan_success) {
    if (parallel_out_path_planner_.GetOutput().path_available &&
        parallel_out_path_planner_.GetOutput().path_segment_vec.size() > 0) {
      plan_result = PathPlannerResult::PLAN_UPDATE;
      ILOG_INFO << "path plan success!";
    } else {
      ILOG_INFO << "path plan success! however no path given!";
      return PathPlannerResult::PLAN_FAILED;
    }

  } else {
    plan_result = PathPlannerResult::PLAN_FAILED;
    ILOG_INFO << "path plan fail!";
    return PathPlannerResult::PLAN_FAILED;
  }

  parallel_out_path_planner_.SetCurrentPathSegIndex();

  const auto& path_planner_output = parallel_out_path_planner_.GetOutput();
  ILOG_INFO << "first seg idx = " << path_planner_output.path_seg_index.first;
  ILOG_INFO << "last seg idx = " << path_planner_output.path_seg_index.second;

  frame_.gear_command =
      path_planner_output
          .gear_cmd_vec[path_planner_output.path_seg_index.first];

  parallel_out_path_planner_.SampleCurrentPathSeg();

  if (frame_.is_replan_first) {
    frame_.is_replan_first = false;
    frame_.current_gear = pnc::geometry_lib::ReverseGear(
        path_planner_output.gear_cmd_vec.front());

    ILOG_INFO << "next gear =" << static_cast<int>(frame_.current_gear);
  } else {
    // set current gear
    frame_.current_gear = pnc::geometry_lib::ReverseGear(frame_.current_gear);
    ILOG_INFO << "next gear =" << static_cast<int>(frame_.current_gear);

    if (!pnc::geometry_lib::IsValidGear(frame_.current_gear)) {
      ILOG_INFO << "frame_.current_gear == invalid gear!";
      return PathPlannerResult::PLAN_FAILED;
    }
  }

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(
      path_planner_output.path_point_vec.size());

  pnc::geometry_lib::PathPoint global_point;
  for (const auto& path_point : path_planner_output.path_point_vec) {
    global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                     ego_slot_info.l2g_tf.GetHeading(path_point.heading));

    current_path_point_global_vec_.emplace_back(global_point);
  }

  JSON_DEBUG_VECTOR("plan_traj_x", std::vector<double>{0.0}, 3)
  JSON_DEBUG_VECTOR("plan_traj_y", std::vector<double>{0.0}, 3)
  JSON_DEBUG_VECTOR("plan_traj_heading", std::vector<double>{0.0}, 3)
  JSON_DEBUG_VECTOR("plan_traj_lat_buffer", std::vector<double>{0.0}, 3)
  return plan_result;
}

const bool ParallelParkOutScenario::CheckReplan() {
  if (frame_.is_replan_first == true ||
      apa_world_ptr_->GetSimuParam().force_plan) {
    ILOG_INFO << "first plan";
    frame_.replan_reason = FIRST_PLAN;
    return true;
  }

  if (CheckSegCompleted()) {
    ILOG_INFO << "replan by current segment completed!";
    frame_.replan_reason = SEG_COMPLETED_PATH;
    if (frame_.is_replan_by_uss) {
      frame_.replan_reason = SEG_COMPLETED_USS;
    }
    return true;
  }

  ILOG_INFO << "frame_.stuck_time = " << frame_.stuck_time;

  if (frame_.stuck_time > apa_param.GetParam().stuck_replan_time) {
    ILOG_INFO << "replan by stuck!";
    frame_.replan_reason = STUCKED;
    return true;
  }

  // Todo: maybe CheckDynamicUpdate
  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool ParallelParkOutScenario::CheckSegCompleted() {
  ILOG_INFO << "CheckSegCompleted -------------------------------";
  frame_.is_replan_by_uss = false;

  bool is_seg_complete = false;
  if (frame_.spline_success) {
    ILOG_INFO << "frame_.remain_dist_uss = " << frame_.remain_dist_uss;

    const auto min_remain_dist =
        std::min(frame_.remain_dist_uss, frame_.remain_dist);

    if (min_remain_dist < apa_param.GetParam().max_replan_remain_dist &&
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
      frame_.is_replan_by_uss = (frame_.remain_dist_uss < frame_.remain_dist);

      if (!frame_.is_replan_by_uss) {
        ILOG_INFO << "close to target!\n";
        is_seg_complete = true;
      } else {
        ILOG_INFO << "close to obstacle by uss!\n";
        // uss distance may not be accurate, so wait for a period of time
        if (frame_.stuck_time >
            apa_param.GetParam().uss_stuck_replan_wait_time) {
          is_seg_complete = true;
        }
      }
    }
  }

  return is_seg_complete;
}

void ParallelParkOutScenario::Log() const {
  const auto& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->ego_info_under_slot_;

  const auto& l2g_tf = ego_info_under_slot.l2g_tf;

  const auto p0_g = l2g_tf.GetPos(tlane_.obs_pt_outside);
  const auto p1_g = l2g_tf.GetPos(tlane_.obs_pt_inside);
  ILOG_INFO << "pt out = " << p0_g.transpose();
  ILOG_INFO << "pt in = " << p1_g.transpose();

  std::vector<double> obstaclesX;
  std::vector<double> obstaclesY;
  for (const auto& obs_pair :
       apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap()) {
    for (const auto& obstacle : obs_pair.second) {
      const auto tmp_obstacle = l2g_tf.GetPos(obstacle);
      obstaclesX.emplace_back(tmp_obstacle.x());
      obstaclesY.emplace_back(tmp_obstacle.y());
    }
  }
  JSON_DEBUG_VECTOR("obstaclesX", obstaclesX, 2)
  JSON_DEBUG_VECTOR("obstaclesY", obstaclesY, 2)

  std::vector<double> limiter_corner_X = {-2.0, -2.0};
  std::vector<double> limiter_corner_Y = {1.2, -1.2};
  JSON_DEBUG_VECTOR("limiter_corner_X", limiter_corner_X, 2)
  JSON_DEBUG_VECTOR("limiter_corner_Y", limiter_corner_Y, 2)

  const auto& pts = ego_info_under_slot.slot.origin_corner_coord_global_;
  const std::vector<double> slot_corner_X = {pts.pt_0.x(), pts.pt_1.x(),
                                             pts.pt_2.x(), pts.pt_3.x()};

  const std::vector<double> slot_corner_Y = {pts.pt_0.y(), pts.pt_1.y(),
                                             pts.pt_2.y(), pts.pt_3.y()};

  JSON_DEBUG_VECTOR("slot_corner_X", slot_corner_X, 2)
  JSON_DEBUG_VECTOR("slot_corner_Y", slot_corner_Y, 2)

  JSON_DEBUG_VALUE("terminal_error_x", ego_info_under_slot.terminal_err.pos.x())
  JSON_DEBUG_VALUE("terminal_error_y", ego_info_under_slot.terminal_err.pos.y())
  JSON_DEBUG_VALUE("terminal_error_heading",
                   ego_info_under_slot.terminal_err.heading)

  JSON_DEBUG_VALUE("is_replan_first", frame_.is_replan_first)
  JSON_DEBUG_VALUE("is_replan_by_uss", frame_.is_replan_by_uss)
  JSON_DEBUG_VALUE("current_path_length", frame_.current_path_length)
  JSON_DEBUG_VALUE("path_plan_success", frame_.plan_stm.path_plan_success)
  JSON_DEBUG_VALUE("planning_status", frame_.plan_stm.planning_status)
  JSON_DEBUG_VALUE("spline_success", frame_.spline_success)
  JSON_DEBUG_VALUE("remain_dist", frame_.remain_dist)
  JSON_DEBUG_VALUE("remain_dist_uss", frame_.remain_dist_uss)
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

  std::vector<double> target_ego_pos_slot = {
      ego_info_under_slot.target_pose.pos.x(),
      ego_info_under_slot.target_pose.pos.y()};

  JSON_DEBUG_VALUE("pathplan_result", frame_.pathplan_result)
  JSON_DEBUG_VECTOR("target_ego_pos_slot", target_ego_pos_slot, 2)

  const auto& path_plan_output = parallel_out_path_planner_.GetOutput();
  JSON_DEBUG_VALUE("path_start_seg_index",
                   path_plan_output.path_seg_index.first)
  JSON_DEBUG_VALUE("path_end_seg_index", path_plan_output.path_seg_index.second)
  JSON_DEBUG_VALUE("path_length", path_plan_output.length)

  // // lateral optimization
  // const auto plan_debug_info =
  //     apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();

  // if (plan_debug_info.has_terminal_pos_error()) {
  //   JSON_DEBUG_VALUE("optimization_terminal_pose_error",
  //                    plan_debug_info.terminal_pos_error())
  //   JSON_DEBUG_VALUE("optimization_terminal_heading_error",
  //                    plan_debug_info.terminal_heading_error())
  // } else {
  //   JSON_DEBUG_VALUE("optimization_terminal_pose_error", 0.0)
  //   JSON_DEBUG_VALUE("optimization_terminal_heading_error", 0.0)
  // }
}

const bool ParallelParkOutScenario::GenObstacles() { return true; };

}  // namespace apa_planner
}  // namespace planning
