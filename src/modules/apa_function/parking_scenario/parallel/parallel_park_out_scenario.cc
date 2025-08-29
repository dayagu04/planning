#include "parallel_park_out_scenario.h"

#include <cstddef>

#include "apa_param_config.h"
#include "apa_state_machine_manager.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math_lib.h"
#include "park_hmi_state.h"

namespace planning {
namespace apa_planner {

static double kFrontDetaXMagWhenFrontVacant = 3.0;
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
static double kDeletedObsDistOutSlot = 0.3;
static double kDeletedObsDistInSlot = 0.25;

static double kTBoundarySampleDist = 0.38;
static double kChannelSampleDist = 0.46;
static double kEnterMultiPlanSlotRatio = 0.1;
static double kEps = 1e-5;
void ParallelParkOutScenario::Reset() {
  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  parallel_out_path_planner_.Reset();

  ParkingScenario::Reset();
}

bool ParallelParkOutScenario::CheckFinishParallel() {
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  if (ego_info_under_slot.slot_occupied_ratio < 0.1 &&
      std::fabs(ego_info_under_slot.cur_pose.heading * kRad2Deg) <
          apa_param.GetParam().finish_parallel_out_heading_mag) {
    ILOG_INFO << "parallel out finish slot_occupied_ratio: "
              << ego_info_under_slot.slot_occupied_ratio;
    ILOG_INFO << "parallel out finish heading: "
              << ego_info_under_slot.cur_pose.heading * kRad2Deg;
    return true;
  }
  return false;
}

void ParallelParkOutScenario::ExcutePathPlanningTask() {
  ILOG_INFO << "---------------------parallel out ---------------------------";
  // init simulation
  InitSimulation();

  // check planning status
  if (CheckPlanSkip()) {
    return;
  }

  UpdateStuckTime();

  // calculate remain dist according to plan path
  frame_.remain_dist_path = CalRemainDistFromPath();

  // calculate remain dist uss according to uss
  frame_.remain_dist_obs = CalRealTimeBrakeDist();

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ILOG_INFO << "update ego slot info failed!";
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = UPDATE_EGO_SLOT_INFO;
    return;
  }
  ILOG_INFO << "update ego slot info success!";

  // generate t-lane
  if (!GenTlane()) {
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

  const double max_replan_path_dist = 0.15;
  const double stuck_replan_wait_time = 1.5;

  CheckReplanParams replan_params(
      max_replan_path_dist, 0.068, apa_param.GetParam().max_replan_remain_dist,
      stuck_replan_wait_time, apa_param.GetParam().max_replan_remain_dist,
      0.168, apa_param.GetParam().stuck_replan_time);

  // check replan
  if (!CheckReplan(replan_params)) {
    ILOG_INFO << "replan is not required!";
    SetParkingStatus(PARKING_RUNNING);
    return;
  }
  if (CheckFinishParallel()) {
    ILOG_INFO << "check apa parallel finished!";
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

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
      frame_.plan_fail_reason = PATH_PLAN_FAILED;
      ILOG_INFO << "replan failed from PLAN_HOLD!";
    }
  } else if (pathplan_result == PathPlannerResult::PLAN_UPDATE) {
    if (PostProcessPath()) {
      SetParkingStatus(PARKING_PLANNING);
      ILOG_INFO << "replan from PARKING_PLANNING!";
    } else {
      SetParkingStatus(PARKING_FAILED);
      frame_.plan_fail_reason = PATH_PLAN_FAILED;
      ILOG_INFO << "replan failed from PARKING_PLANNING!";
    }
  } else if (pathplan_result == PathPlannerResult::PLAN_FAILED) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = PATH_PLAN_FAILED;
  }

  ILOG_INFO << "pathplan_result = " << static_cast<int>(pathplan_result);

  // print planning status
  ILOG_INFO << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status);
}

bool ParallelParkOutScenario::ParkOutDirectionTry() {
  ILOG_INFO << "----------parallel out Scenario Try----------";
  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  // init simulation
  InitSimulation();

  // check planning status
  if (CheckPlanSkip()) {
    return false;
  }

  UpdateStuckTime();

  if (CheckPaused()) {
    return false;
  }

  // calculate remain dist according to plan path
  frame_.remain_dist_path = CalRemainDistFromPath();

  double lat_buffer = 0.0;
  double safe_uss_remain_dist = 0.0;
  CalStaticBufferInDiffSteps(lat_buffer, safe_uss_remain_dist);
  ILOG_INFO << "parallel lat_buffer = " << lat_buffer;
  ILOG_INFO << "parallel safe_uss_remain_dist = " << safe_uss_remain_dist;

  double dynaminc_lat_buffer = 0.0;
  double dynamic_lon_buffer = 0.0;
  CalDynamicBufferInDiffSteps(dynaminc_lat_buffer, dynamic_lon_buffer);

  apa_world_ptr_->GetColDetInterfacePtr()->Init(true);
  // calculate remain dist uss according to uss
  frame_.remain_dist_obs = CalRemainDistFromObs(
      safe_uss_remain_dist, lat_buffer, lat_buffer, dynamic_lon_buffer,
      dynaminc_lat_buffer, dynaminc_lat_buffer);
  ILOG_INFO << "final remain_dist_obs = " << frame_.remain_dist_obs;

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ILOG_INFO << "update ego slot info failed!";
    return false;
  }
  ILOG_INFO << "update ego slot info success!";

  // generate t-lane
  if (!GenTlane()) {
    ILOG_INFO << "GenTlane failed!";
    return false;
  }

  // check finish
  if (CheckFinished()) {
    ILOG_INFO << "check apa finished!";
    return false;
  }

  // check failed
  if (CheckStuckFailed()) {
    ILOG_INFO << "check stuck failed!";
    return false;
  }

  const double max_replan_path_dist = 0.15;
  const double stuck_replan_wait_time = 1.5;

  CheckReplanParams replan_params(
      max_replan_path_dist, 0.068, apa_param.GetParam().max_replan_remain_dist,
      stuck_replan_wait_time, apa_param.GetParam().max_replan_remain_dist,
      0.168, apa_param.GetParam().stuck_replan_time);

  // check replan
  if (!CheckReplan(replan_params)) {
    ILOG_INFO << "replan is not required!";
    return false;
  }

  // update obstacles
  GenTBoundaryObstacles();

  // path plan
  const auto pathplan_result = PathPlanOnce();
  frame_.pathplan_result = pathplan_result;
  bool success_ret = false;

  if (pathplan_result == PathPlannerResult::PLAN_HOLD) {
    if (PostProcessPath()) {
      ILOG_INFO << "replan from PARKING_GEARCHANGE!";
      success_ret = true;
    } else {
      ILOG_INFO << "replan failed from PLAN_HOLD!";
    }
  } else if (pathplan_result == PathPlannerResult::PLAN_UPDATE) {
    if (PostProcessPath()) {
      ILOG_INFO << "replan from PARKING_PLANNING!";
      success_ret = true;
    } else {
      ILOG_INFO << "replan failed from PARKING_PLANNING!";
    }
  } else if (pathplan_result == PathPlannerResult::PLAN_FAILED) {
    ILOG_INFO << "geometry path try fail";
  }
  ILOG_INFO << "pathplan_result = " << static_cast<int>(pathplan_result);

  frame_.is_replan_first = true;
  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  return success_ret;
}

void ParallelParkOutScenario::ScenarioTry() {
  //ApaParkOutDirection::RIGHT_FRONT
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  ego_info_under_slot.slot.release_info_
        .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
        SlotReleaseState::NOT_RELEASE;
  multi_parkout_direction.clear();
  multi_parkout_path_vec.clear();
  ApaParkOutDirection directions[] = {ApaParkOutDirection::LEFT_FRONT,
                                      ApaParkOutDirection::RIGHT_FRONT};
  for (auto direction : directions) {
    apa_world_ptr_->GetStateMachineManagerPtr()->SetParkOutDirection(
      direction);
    parkout_direction_ = direction;
    if(ParkOutDirectionTry()) {
      multi_parkout_direction[direction] = true;
      ego_info_under_slot.slot.release_info_
        .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
        SlotReleaseState::RELEASE;
    } else {
      multi_parkout_direction[direction] = false;
    }
    ILOG_INFO << "direction = " << static_cast<int>(direction) <<
      " multi_parkout_direction = " << multi_parkout_direction[direction];
  }
  ApaDirectionGenerator generator;
  generator.ClearRecommendationDirectionFlag(apa_hmi_);
  if (multi_parkout_direction[ApaParkOutDirection::RIGHT_FRONT]) {
    generator.SetRecommendationDirectionFlag(apa_hmi_, ParityBit);
    generator.SetRecommendationDirectionFlag(apa_hmi_, ParallelFrontRight);
    complete_path_point_global_vec_ = multi_parkout_path_vec[ApaParkOutDirection::RIGHT_FRONT];
  }
  if (multi_parkout_direction[ApaParkOutDirection::LEFT_FRONT]) {
    generator.SetRecommendationDirectionFlag(apa_hmi_, ParityBit);
    generator.SetRecommendationDirectionFlag(apa_hmi_, ParallelFrontLeft);
    complete_path_point_global_vec_ = multi_parkout_path_vec[ApaParkOutDirection::LEFT_FRONT];
  }
  TansformPreparePlanningTraj();
  ILOG_INFO << "recommendation direction = " << apa_hmi_.planning_park_dir;
  parkout_direction_ = ApaParkOutDirection::INVALID;
}

const double ParallelParkOutScenario::CalRealTimeBrakeDist() {
  double lat_buffer = 0.0;
  double safe_uss_remain_dist = 0.0;
  CalStaticBufferInDiffSteps(lat_buffer, safe_uss_remain_dist);
  ILOG_INFO << "parallel lat_buffer = " << lat_buffer;
  ILOG_INFO << "parallel safe_uss_remain_dist = " << safe_uss_remain_dist;

  double dynaminc_lat_buffer = 0.0;
  double dynamic_lon_buffer = 0.0;
  CalDynamicBufferInDiffSteps(dynaminc_lat_buffer, dynamic_lon_buffer);

  apa_world_ptr_->GetColDetInterfacePtr()->Init(true);

  const double remain_dist_obs = CalRemainDistFromObs(
      safe_uss_remain_dist, lat_buffer, lat_buffer, dynamic_lon_buffer,
      dynaminc_lat_buffer, dynaminc_lat_buffer);

  ILOG_INFO << "final remain_dist_obs = " << remain_dist_obs;

  return remain_dist_obs;
}

const bool ParallelParkOutScenario::UpdateEgoSlotInfo() {
  using namespace pnc::geometry_lib;

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

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

  const auto& limiter = ego_info_under_slot.slot.GetLimiter();
  ILOG_INFO << "limiter.valid = " << limiter.valid;
  if (limiter.valid) {
    t_lane_.limiter.valid = true;
    // transfer limiter in slot coordination
    t_lane_.limiter.start_pt =
        ego_info_under_slot.g2l_tf.GetPos(limiter.start_pt);

    t_lane_.limiter.end_pt = ego_info_under_slot.g2l_tf.GetPos(limiter.end_pt);

    ILOG_INFO << "limiter start pos = " << t_lane_.limiter.start_pt.transpose();
    ILOG_INFO << "limiter end pos = " << t_lane_.limiter.end_pt.transpose();

    const double max_limiter_x =
        std::max(t_lane_.limiter.start_pt.x(), t_lane_.limiter.end_pt.x());

    const double min_limiter_x =
        std::min(t_lane_.limiter.start_pt.x(), t_lane_.limiter.end_pt.x());

    if (max_limiter_x < 0.5 * ego_info_under_slot.slot.slot_length_) {
      ILOG_INFO << "limiter behind!";
    } else {
      ILOG_INFO << "limiter front!";
    }
  }

  return true;
}

const bool ParallelParkOutScenario::CheckFinished() {
  const double slot_occupied_ratio = apa_world_ptr_->GetSlotManagerPtr()
                                         ->GetEgoInfoUnderSlot()
                                         .slot_occupied_ratio;

  const double heading_mag_deg = std::fabs(apa_world_ptr_->GetSlotManagerPtr()
                                               ->GetEgoInfoUnderSlot()
                                               .cur_pose.heading *
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
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

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
    if (pair.second.GetObsMovementType() != ApaObsMovementType::STATIC) {
      continue;
    }

    const auto obs_scement = pair.second.GetObsScemanticType();

    const bool is_rigid = (obs_scement == ApaObsScemanticType::WALL ||
                           obs_scement == ApaObsScemanticType::COLUMN ||
                           obs_scement == ApaObsScemanticType::CAR);

    for (const auto& obs_pt_local : pair.second.GetPtClout2dLocal()) {
      if ((obs_pt_local - slot_center).norm() > 25.0) {
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
              (-0.5 * slot_width - apa_param.GetParam().curb_offset)) {
        // total_box_y_fail_cnt++;
        continue;
      }

      // remote front T-boundary obs
      if (obs_pt_local.x() > slot_length + kFrontDetaXMagWhenFrontVacant &&
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
      if (ego_info_under_slot.slot_occupied_ratio > 0.1) {
        if (mathlib::IsInBound(obs_pt_local.x(), 1.0, slot_length - 0.5) &&
            mathlib::IsInBound(obs_pt_local.y(),
                               (0.5 * slot_width - 0.5) * side_sgn,
                               (0.5 * slot_width + 0.8) * side_sgn)) {
          ILOG_WARN << "out is obs, obs_pt_local = "
                    << obs_pt_local.transpose();
          return false;
        }
      }

      if (mathlib::IsInBound(obs_pt_local.x(), 0.8, slot_length - 0.8) &&
          obs_pt_local.y() * side_sgn < -0.25 * slot_width * side_sgn &&
          obs_pt_local.y() * side_sgn >
              -(0.5 * slot_width + apa_param.GetParam().curb_offset) *
                  side_sgn &&
          is_rigid) {
        ILOG_INFO << "rigid obs = " << obs_pt_local.transpose();
        t_lane_.is_inside_rigid = true;
      }

      obs_pt_local_vec_.emplace_back(std::move(obs_pt_local));
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

  for (const auto& obstacle_point_slot : obs_pt_local_vec_) {
    const bool front_obs_condition =
        pnc::mathlib::IsInBound(
            obstacle_point_slot.x(),
            slot_length - kFrontMaxDetaXMagWhenFrontOccupied,
            slot_length + kFrontDetaXMagWhenFrontVacant) &&
        pnc::mathlib::IsInBound(
            obstacle_point_slot.y(), -0.4 * side_sgn,
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
                                slot_length + kFrontDetaXMagWhenFrontVacant) &&
        pnc::mathlib::IsInBound(obstacle_point_slot.y(), 0.0, 2.5 * side_sgn);

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

  t_lane_.obs_pt_inside << front_min_x, front_y_limit;
  t_lane_.obs_pt_outside << rear_max_x, rear_parallel_line_y_limit;

  curb_y_limit = pnc::mathlib::Clamp(
      curb_y_limit, -side_sgn * (half_slot_width + kCurbInitialOffset),
      -side_sgn * (half_slot_width - kCurbInitialOffset));

  // if (ego_info_under_slot.slot_occupied_ratio > 0.01) {
  //   curb_y_limit -= side_sgn * 0.1;
  // }

  t_lane_.corner_inside_slot << slot_length, half_slot_width * side_sgn;
  t_lane_.corner_outside_slot << 0.0, half_slot_width * side_sgn;

  t_lane_.curb_y = curb_y_limit;
  t_lane_.channel_y = channel_y_limit;
  t_lane_.channel_x_limit = channel_x_limit;

  t_lane_.pt_outside = t_lane_.obs_pt_outside;
  t_lane_.pt_inside = t_lane_.obs_pt_inside;

  t_lane_.slot_length = slot_length;
  t_lane_.slot_width = slot_width;

  ILOG_INFO << "-- t_lane --------";
  if (pnc::mathlib::IsDoubleEqual(side_sgn, 1.0)) {
    ILOG_INFO << "park out left";
  } else if (pnc::mathlib::IsDoubleEqual(side_sgn, -1.0)) {
    ILOG_INFO << "park out right";
  }
  ILOG_INFO << "obs_pt_inside = " << t_lane_.obs_pt_inside.transpose();
  ILOG_INFO << "obs_pt_outside = " << t_lane_.obs_pt_outside.transpose();
  ILOG_INFO << "corner_inside_slot = "
            << t_lane_.corner_inside_slot.transpose();
  ILOG_INFO << "corner_outside_slot = "
            << t_lane_.corner_outside_slot.transpose();

  ILOG_INFO << "pt_outside = " << t_lane_.pt_outside.transpose();
  ILOG_INFO << "pt_inside = " << t_lane_.pt_inside.transpose();

  ILOG_INFO << "slot length =" << t_lane_.slot_length;
  ILOG_INFO << "half slot width =" << 0.5 * t_lane_.slot_width;
  ILOG_INFO << "curb y =" << t_lane_.curb_y;
  ILOG_INFO << "channel x =" << t_lane_.channel_x_limit;
  ILOG_INFO << "channel y =" << t_lane_.channel_y;
  ILOG_INFO << "------------------";

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
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const double slot_side_sgn = (frame_.is_park_out_left ? 1.0 : -1.0);
  t_lane_.slot_side_sgn = slot_side_sgn;
  // set T-Boundary obstacles
  const Eigen::Vector2d B(t_lane_.obs_pt_outside.x(), 0.3 * slot_side_sgn);

  const Eigen::Vector2d A(B.x() - 3.2, B.y());

  const Eigen::Vector2d E(t_lane_.obs_pt_inside.x(),
                          t_lane_.obs_pt_inside.y() - slot_side_sgn * 0.8);

  const Eigen::Vector2d C(
      B.x(), (-0.5 * t_lane_.slot_width - apa_param.GetParam().curb_offset) *
                 slot_side_sgn);

  const Eigen::Vector2d D(E.x(), C.y());

  const Eigen::Vector2d F(t_lane_.channel_x_limit, E.y());

  const Eigen::Vector2d channel_point_1(
      A.x(), apa_param.GetParam().parallel_channel_y_mag * slot_side_sgn);

  const Eigen::Vector2d channel_point_2(
      F.x(), apa_param.GetParam().parallel_channel_y_mag * slot_side_sgn);

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

  double dist_to_cat = slot_side_sgn > 0 ? std::min(B.y(), E.y()) : std::max(B.y(), E.y());
  for (const auto& obstacle_point_slot : obs_pt_local_vec_) {
    // add obs near channel
    const bool channel_y_condition =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(), channel_point_1.x(),
                                channel_point_2.x()) &&
        pnc::mathlib::IsInBound(obstacle_point_slot.y(),
                                kMinChannelYMagIdentification * slot_side_sgn,
                                channel_point_1.y());

    if (channel_y_condition) {
      filtered_channel_obs_vec.emplace_back(obstacle_point_slot);

      if (pnc::mathlib::IsInBound(obstacle_point_slot.x(), t_lane_.slot_length,
                                  t_lane_.slot_length + 5.0)) {
        if (slot_side_sgn > 0.0) {
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
        pnc::mathlib::IsInBound(obstacle_point_slot.y(), B.y(),
                                kMinChannelYMagIdentification * slot_side_sgn);

    if (is_rear_tlane_line) {
      tlane_obstacle_vec.emplace_back(obstacle_point_slot);
      continue;
    }

    const bool is_front_tlane_line =
        pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                t_lane_.obs_pt_inside.x() - 0.3, F.x()) &&
        pnc::mathlib::IsInBound(obstacle_point_slot.y(), E.y(),
                                kMinChannelYMagIdentification * slot_side_sgn);

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
        pnc::mathlib::IsInBound(obs_pos.y(), -0.4 * slot_side_sgn,
                                1.2 * slot_side_sgn)) {
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
        pnc::mathlib::IsInBound(obs_pos.y(), 1.5 * slot_side_sgn, C_curb.y());

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

  if (t_lane_.limiter.valid) {
    double limiter_obs_x = 0.0;
    if (t_lane_.limiter.start_pt.x() < 0.5 * t_lane_.slot_length) {
      ILOG_INFO << "rear limiter";
      limiter_obs_x =
          std::max(t_lane_.limiter.start_pt.x(), t_lane_.limiter.end_pt.x());
      limiter_obs_x +=
          (apa_param.GetParam().parallel_ego_ac_x_offset_with_limiter -
           apa_param.GetParam().rear_overhanging - 0.12);
      ILOG_INFO << "limiter_obs_x = " << limiter_obs_x;
    } else {
      ILOG_INFO << "front limiter";
      limiter_obs_x =
          std::min(t_lane_.limiter.start_pt.x(), t_lane_.limiter.end_pt.x());
      limiter_obs_x +=
          (-apa_param.GetParam().parallel_ego_ac_x_offset_with_limiter -
           apa_param.GetParam().front_overhanging + 0.12);
    }

    const Eigen::Vector2d limiter_obs_start(limiter_obs_x,
                                            t_lane_.limiter.start_pt.y());
    const Eigen::Vector2d limiter_obs_end(limiter_obs_x,
                                          t_lane_.limiter.end_pt.y());

    const pnc::geometry_lib::LineSegment limiter_line(limiter_obs_start,
                                                      limiter_obs_end);
    point_set.clear();
    tlane_obstacle_vec.clear();
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, limiter_line,
                                               kTBoundarySampleDist);
    for (const auto& obs : point_set) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs, ego_info_under_slot.cur_pose, kDeletedObsDistInSlot)) {
        tlane_obstacle_vec.emplace_back(obs);
      }
    }
    apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
        tlane_obstacle_vec, CollisionDetector::LIMITER_OBS);
  }
}

const uint8_t ParallelParkOutScenario::PathPlanOnce() {
  // construct input
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  GeometryPathInput path_planner_input;
  path_planner_input.tlane = t_lane_;
  path_planner_input.sample_ds = apa_world_ptr_->GetSimuParam().sample_ds;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_complete_path =
      apa_world_ptr_->GetSimuParam().is_complete_path;
  path_planner_input.ego_info_under_slot = ego_info_under_slot;

  if (frame_.is_replan_first) {
    // temprarily give driving gear
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;

    frame_.current_arc_steer = frame_.is_park_out_left
                                   ? geometry_lib::SEG_STEER_LEFT
                                   : geometry_lib::SEG_STEER_RIGHT;
  }

  ILOG_INFO << "ego_slot_info.slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;

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
    global_point.Set(ego_info_under_slot.l2g_tf.GetPos(path_point.pos),
                     ego_info_under_slot.l2g_tf.GetHeading(path_point.heading));
    global_point.s = path_point.s;
    global_point.kappa = path_point.kappa;

    current_path_point_global_vec_.emplace_back(global_point);
  }
  if (parkout_direction_ != ApaParkOutDirection::INVALID) {
    std::vector<pnc::geometry_lib::PathPoint> cur_dir_path_point_global_vec;
    for (const auto& path_point : path_planner_output.all_gear_path_point_vec) {
      global_point.Set(ego_info_under_slot.l2g_tf.GetPos(path_point.pos),
                      ego_info_under_slot.l2g_tf.GetHeading(path_point.heading));
      global_point.lat_buffer = path_point.lat_buffer;
      global_point.s = path_point.s;
      global_point.kappa = path_point.kappa;
      global_point.gear = path_point.gear;
      cur_dir_path_point_global_vec.emplace_back(global_point);
    }
    multi_parkout_path_vec[parkout_direction_] = cur_dir_path_point_global_vec;
  }

  return plan_result;
}

const double ParallelParkOutScenario::CalcSlotOccupiedRatio(
    const pnc::geometry_lib::PathPoint start_pose) const {
  double slot_occupied_ratio = 0.0;

  if (pnc::mathlib::IsInBound(start_pose.pos.x(), t_lane_.pt_outside.x(),
                              t_lane_.pt_inside.x())) {
    const double y_err_ratio = start_pose.pos.y() / (0.5 * t_lane_.slot_width);

    if (t_lane_.slot_side_sgn) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  return slot_occupied_ratio;
}

void ParallelParkOutScenario::CalStaticBufferInDiffSteps(
    double& lat_buffer, double& safe_uss_remain_dist) const {
  const auto slot_mgr = apa_world_ptr_->GetSlotManagerPtr();
  const auto& ego_info = slot_mgr->GetEgoInfoUnderSlot();

  const bool is_ego_in_slot =
      ego_info.slot_occupied_ratio >= kEnterMultiPlanSlotRatio;
  ILOG_INFO << "is_ego_in_slot = " << is_ego_in_slot;

  // totally in slot
  if (is_ego_in_slot) {
    ILOG_INFO << " totally in slot!";
    safe_uss_remain_dist =
        apa_param.GetParam().safe_uss_remain_dist_in_parallel_slot;

    lat_buffer = t_lane_.is_inside_rigid
                     ? 0.0
                     : apa_param.GetParam().safe_lat_buffer_in_parallel_slot;
  } else {
    lat_buffer = 0.2;
    safe_uss_remain_dist = apa_param.GetParam().safe_uss_remain_dist_out_slot;
  }
}

void ParallelParkOutScenario::CalDynamicBufferInDiffSteps(
    double& dynaminc_lat_buffer, double& dynamic_lon_buffer) const {
  if (apa_world_ptr_->GetSlotManagerPtr()
          ->GetEgoInfoUnderSlot()
          .slot_occupied_ratio < kEnterMultiPlanSlotRatio) {
    dynaminc_lat_buffer = apa_param.GetParam().parallel_dynamic_lat_buffer;
    dynamic_lon_buffer = apa_param.GetParam().parallel_dynamic_lon_buffer;
  } else {
    dynaminc_lat_buffer =
        apa_param.GetParam().parallel_dynamic_lat_buffer_in_slot;
    dynamic_lon_buffer =
        apa_param.GetParam().parallel_dynamic_lon_buffer_in_slot;
  }
}

void ParallelParkOutScenario::Log() const {
  const auto& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const auto& l2g_tf = ego_info_under_slot.l2g_tf;

  const auto p0_g = l2g_tf.GetPos(t_lane_.obs_pt_outside);
  const auto p1_g = l2g_tf.GetPos(t_lane_.obs_pt_inside);
  ILOG_INFO << "obs p out = " << p0_g.transpose();
  ILOG_INFO << "obs p in = " << p1_g.transpose();

  size_t real_obs_size = 0;
  for (const auto& obs_pair :
       apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap()) {
    if (obs_pair.first == CollisionDetector::VIRTUAL_OBS) {
      continue;
    }
    real_obs_size += obs_pair.second.size();
  }

  ILOG_INFO << "obs_size = " << real_obs_size;
  const int count_unit = std::ceil(real_obs_size / 400.0);
  const size_t simplify_obs_num =
      std::ceil(real_obs_size / std::max(1, count_unit));

  std::vector<double> obstaclesX;
  std::vector<double> obstaclesY;
  obstaclesX.reserve(simplify_obs_num);
  obstaclesY.reserve(simplify_obs_num);
  for (const auto& obs_pair :
       apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap()) {
    if (obs_pair.first == CollisionDetector::VIRTUAL_OBS) {
      continue;
    }
    const auto& obs_vec = obs_pair.second;

    for (size_t i = 0; i < obs_vec.size(); i += count_unit) {
      const auto obs_g = l2g_tf.GetPos(obs_vec[i]);
      obstaclesX.emplace_back(obs_g.x());
      obstaclesY.emplace_back(obs_g.y());
    }
  }
  if (obstaclesX.empty()) {
    obstaclesX = {0.0};
    obstaclesY = {0.0};
  }

  const size_t max_count = 798;
  if (obstaclesX.size() > max_count) {
    obstaclesX.resize(max_count);
    obstaclesY.resize(max_count);
  }

  RecordDebugObstacle(obstaclesX, obstaclesY);

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
  JSON_DEBUG_VALUE("current_path_length", frame_.current_path_length)
  JSON_DEBUG_VALUE("path_plan_success", frame_.plan_stm.path_plan_success)
  JSON_DEBUG_VALUE("planning_status", frame_.plan_stm.planning_status)
  JSON_DEBUG_VALUE("spline_success", frame_.spline_success)
  JSON_DEBUG_VALUE("remain_dist", frame_.remain_dist_path)
  JSON_DEBUG_VALUE("remain_dist_obs", frame_.remain_dist_obs)
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
