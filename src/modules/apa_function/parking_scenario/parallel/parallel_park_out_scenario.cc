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
static double kMinChannelYMagIdentification = 3.5;
static double kStrictChannelYMagIdentification = 2.8;
static double kDeletedObsDistOutSlot = 0.3;
static double kDeletedObsDistInSlot = 0.25;

static double kTBoundarySampleDist = 0.38;
static double kChannelSampleDist = 0.46;
static double kEnterMultiPlanSlotRatio = 0.1;
static double kEps = 1e-5;
static double kArcEndHeadThreshold = 40.0;
void ParallelParkOutScenario::Reset() {
  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  obs_id_pt_map_.clear();
  parallel_out_path_planner_.Reset();
  is_try_tlane_ = false;
  delay_check_finish_ = false;
  is_last_pose_set_ = false;
  last_target_pose_ = pnc::geometry_lib::PathPoint();
  arc_slot_init_out_heading_ = 0.0;
  is_arc_slot_ = false;
  is_outer_arc_slot_ = false;
  path_end_heading_is_met_ = false;
  current_gear_ = AstarPathGear::PARKING;
  obs_hastar.Clear();
  virtual_wall_decider_.Reset(Pose2D(0, 0, 0));
  astar_state_ = AstarSearchState::NONE;
  strict_channel_y = 6.5;

  ParkingScenario::Reset();
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
  parkout_direction_ =
      apa_world_ptr_->GetStateMachineManagerPtr()->GetParkOutDirection();
  is_try_tlane_ = false;

  // calculate remain dist according to plan path
  frame_.remain_dist_path = CalRemainDistFromPath();

  // calculate remain dist uss according to uss
  frame_.remain_dist_obs = CalRealTimeBrakeDist();

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ILOG_INFO << "update ego slot info failed!";
    CheckEgoPoseWhenParkOutFaild(UPDATE_EGO_SLOT_INFO);
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
    CheckEgoPoseWhenParkOutFaild(STUCK_FAILED_TIME);
    return;
  }

  const double max_replan_path_dist = 0.15;
  const double stuck_replan_wait_time = 1.5;

  CheckReplanParams replan_params(
      max_replan_path_dist, 0.068, apa_param.GetParam().max_replan_remain_dist,
      stuck_replan_wait_time, apa_param.GetParam().max_replan_remain_dist,
      0.168, apa_param.GetParam().stuck_replan_time);

  bool update_thread_path = true;
  ILOG_INFO << "parallel_enable_hybrid_astar: "
            << int(apa_param.GetParam().parallel_enable_hybrid_astar);
  if (apa_param.GetParam().parallel_enable_hybrid_astar) {
    update_thread_path = UpdateThreadPath();
    ILOG_INFO << "update_thread_path" << int(update_thread_path);
    if (update_thread_path) {
      PathPlannerResult update_result = PathPlanOnceHybridAStar();
      UpdatePostProcessStatus(update_result);
      return;
    }
  }
  // check replan
  if (!CheckReplan(replan_params)) {
    bool replan_by_short_trimmed_path = false;
    ILOG_INFO << "replan is not required!";
    if (astar_state_ == AstarSearchState::NONE &&
        (apa_param.GetParam().is_trim_limter_parallel_enable ||
         !apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus())) {
      GeometryPathInput path_planner_input;
      const EgoInfoUnderSlot& ego_info_under_slot =
          apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
      path_planner_input.ego_info_under_slot = ego_info_under_slot;
      parallel_out_path_planner_.SetInput(path_planner_input);
      std::vector<geometry_lib::PathPoint> tmp_path_point_vec;
      tmp_path_point_vec = previous_current_path_point_global_vec_;
      parallel_out_path_planner_.TrimPathByLimiterPathPoint(tmp_path_point_vec,
                                                            true);
      if (tmp_path_point_vec.size() < current_path_point_global_vec_.size()) {
        current_path_point_global_vec_ = tmp_path_point_vec;
      }
      replan_by_short_trimmed_path = !PostProcessPathPara();
      // parallel_path_planner_.TrimPathByLimiterLastPathVec(false);
    }
    if (!replan_by_short_trimmed_path) {
      SetParkingStatus(PARKING_RUNNING);
    }
    if (apa_param.GetParam().parallel_enable_hybrid_astar) {
      HybridAstarDebugInfoClear();
    }
    return;
  }
  if (CheckFinished()) {
    ILOG_INFO << "check apa parallel finished!";
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

  // generate t-lane
  if (!GenTlane()) {
    CheckEgoPoseWhenParkOutFaild(NO_TARGET_POSE);
    return;
  }

  // update obstacles
  GenTBoundaryObstacles();
  previous_parallel_out_path_planner_ = parallel_out_path_planner_;

  // path plan
  const auto pathplan_result = PathPlanOnce();
  frame_.pathplan_result = pathplan_result;

  UpdatePostProcessStatus(static_cast<PathPlannerResult>(pathplan_result));

  ILOG_INFO << "pathplan_result = " << static_cast<int>(pathplan_result);

  // print planning status
  ILOG_INFO << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status);
}

void ParallelParkOutScenario::UpdatePostProcessStatus(
    PathPlannerResult pathplan_result) {
  if (pathplan_result == PathPlannerResult::PLAN_HOLD) {
    if (PostProcessPath()) {
      SetParkingStatus(PARKING_GEARCHANGE);
      delay_check_finish_ = true;
      ILOG_INFO << "replan from PARKING_GEARCHANGE!";
    } else {
      CheckEgoPoseWhenParkOutFaild(PATH_PLAN_FAILED);
      ILOG_INFO << "replan failed from PLAN_HOLD!";
    }
  } else if (pathplan_result == PathPlannerResult::PLAN_UPDATE) {
    if (PostProcessPath()) {
      SetParkingStatus(PARKING_PLANNING);
      delay_check_finish_ = true;
      ILOG_INFO << "replan from PARKING_PLANNING!";
    } else {
      CheckEgoPoseWhenParkOutFaild(PATH_PLAN_FAILED);
      ILOG_INFO << "replan failed from PARKING_PLANNING!";
    }
  } else if (pathplan_result == PathPlannerResult::PLAN_FAILED) {
    CheckEgoPoseWhenParkOutFaild(PATH_PLAN_FAILED);
  } else if (pathplan_result == PathPlannerResult::WAIT_PATH) {
    SetParkingStatus(PARKING_RUNNING);
  }
  return;
}

bool ParallelParkOutScenario::ParkOutDirectionTryHybridAStar() {
  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  is_try_tlane_ = true;

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ILOG_INFO << "update ego slot info failed!";
    return false;
  }

  PathPlannerResult res = PathPlannerResult::WAIT_PATH;
  bool update_thread_path = UpdateThreadPath();
  res = PathPlanOnceHybridAStar();

  if (res == PathPlannerResult::PLAN_FAILED) {

    ego_info_under_slot.slot.release_info_
        .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
        SlotReleaseState::NOT_RELEASE;

    ILOG_INFO << "astar path try fail";
  } else if (res == PathPlannerResult::PLAN_UPDATE) {

    ego_info_under_slot.slot.release_info_
        .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
        SlotReleaseState::RELEASE;

    ILOG_INFO << "hybrid astar path try success";
  } else if (res == PathPlannerResult::WAIT_PATH) {
    ILOG_INFO << "hybrid astar path try wait astar release = "
              << int(ego_info_under_slot.slot.release_info_
                      .release_state[ASTAR_PLANNING_RELEASE]);
    SlotReleaseState astar_release_state =
        ego_info_under_slot.slot.release_info_
            .release_state[ASTAR_PLANNING_RELEASE];
    // 如果上一帧A星释放车位，在当前帧结果还没有出来时，不改变上一帧结果;
    // 如果上一帧结果未知，使用计算中状态填充;
    if (astar_release_state == SlotReleaseState::UNKNOWN) {
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
          SlotReleaseState::COMPUTING;
    }
  }

  frame_.is_replan_first = true;
  is_try_tlane_ = false;
  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  return true;
}

bool ParallelParkOutScenario::ParkOutPlanTry() {
  ILOG_INFO << "----------parallel out Scenario Try----------";
  frame_.Reset();
  t_lane_.Reset();
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSwitchToSearch()){
    parallel_out_path_planner_.Reset();
  }
  obs_pt_local_vec_.clear();
  obs_id_pt_map_.clear();
  is_try_tlane_ = true;
  // init simulation
  InitSimulation();

  if (CheckPaused()) {
    return false;
  }

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ILOG_INFO << "update ego slot info failed!";
    return false;
  }

  // generate t-lane
  if (!GenTlane()) {
    ILOG_INFO << "GenTlane failed!";
    return false;
  }

  // update obstacles
  GenTBoundaryObstacles();

  previous_parallel_out_path_planner_ = parallel_out_path_planner_;

  // path plan
  const auto pathplan_result = PathPlanOnceGeometry();
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
  is_try_tlane_ = false;
  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  obs_id_pt_map_.clear();
  return success_ret;
}

bool ParallelParkOutScenario::ParkOutDirectionTry() {
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  ego_info_under_slot.slot.release_info_
      .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
      SlotReleaseState::NOT_RELEASE;
  ApaParkOutDirection directions[] = {ApaParkOutDirection::RIGHT_FRONT,
                                      ApaParkOutDirection::LEFT_FRONT};
  for (auto direction : directions) {
    apa_world_ptr_->GetStateMachineManagerPtr()->SetParkOutDirection(direction);
    parkout_direction_ = direction;
    if (ParkOutPlanTry()) {
      multi_parkout_direction[direction] = true;
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
          SlotReleaseState::RELEASE;
    } else {
      multi_parkout_direction[direction] = false;
    }
    ILOG_INFO << "direction = " << static_cast<int>(direction)
              << " multi_parkout_direction = "
              << multi_parkout_direction[direction];
  }
  if (ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] !=
      SlotReleaseState::RELEASE) {
    ILOG_INFO << "geometry path try fail";
    return false;
  }
  complete_path_point_global_vec_.clear();
  ApaDirectionGenerator generator;
  generator.ClearReleaseDirectionFlag(apa_hmi_);
  generator.SetReleaseDirectionFlag(apa_hmi_, ParityBit);
  generator.ClearRecommendationDirectionFlag(apa_hmi_);
  generator.SetRecommendationDirectionFlag(apa_hmi_, ParityBit);
  if (multi_parkout_direction[ApaParkOutDirection::RIGHT_FRONT]) {
    generator.SetReleaseDirectionFlag(apa_hmi_, ParallelFrontRight);
    generator.SetRecommendationDirectionFlag(apa_hmi_, ParallelFrontRight);
    complete_path_point_global_vec_ =
        multi_parkout_path_vec[ApaParkOutDirection::RIGHT_FRONT];
  }
  if (multi_parkout_direction[ApaParkOutDirection::LEFT_FRONT]) {
    generator.SetReleaseDirectionFlag(apa_hmi_, ParallelFrontLeft);
    generator.ClearRecommendationDirectionFlag(apa_hmi_);
    generator.SetRecommendationDirectionFlag(apa_hmi_, ParityBit);
    generator.SetRecommendationDirectionFlag(apa_hmi_, ParallelFrontLeft);
    complete_path_point_global_vec_ =
        multi_parkout_path_vec[ApaParkOutDirection::LEFT_FRONT];
  }

  // generator.SetReleaseDirectionFlag(apa_hmi_, ParallelFrontLeft);
  // generator.SetRecommendationDirectionFlag(apa_hmi_, ParallelFrontLeft);
  return true;
}

void ParallelParkOutScenario::ScenarioTry() {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus()) {
    return;
  }
  multi_parkout_direction.clear();
  multi_parkout_path_vec.clear();

  bool res = ParkOutDirectionTry();
  if (!res && apa_param.GetParam().parallel_enable_hybrid_astar) {
    ParkOutDirectionTryHybridAStar();
  }

  TansformPreparePlanningTraj();
  ILOG_INFO << "relaese direction = " << apa_hmi_.planning_park_dir
            << ", recommendation direction = "
            << apa_hmi_.planning_recommend_park_dir;
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
      dynaminc_lat_buffer, dynaminc_lat_buffer, false,
      apa_param.GetParam().use_obs_height_method, false);

  ILOG_INFO << "final remain_dist_obs = " << remain_dist_obs;

  return remain_dist_obs;
}

const bool ParallelParkOutScenario::UpdateEgoSlotInfo() {
  using namespace pnc::geometry_lib;

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  auto& select_slot_global =
      ego_info_under_slot.slot.origin_corner_coord_global_;
  ILOG_INFO << " select_slot_global pt 0 = " << select_slot_global.pt_0.x()
            << " " << select_slot_global.pt_0.y();
  ILOG_INFO << " select_slot_global pt 1 = " << select_slot_global.pt_1.x()
            << " " << select_slot_global.pt_1.y();
  ILOG_INFO << " select_slot_global pt 2 = " << select_slot_global.pt_2.x()
            << " " << select_slot_global.pt_2.y();
  ILOG_INFO << " select_slot_global pt 3 = " << select_slot_global.pt_3.x()
            << " " << select_slot_global.pt_3.y();

  if (select_slot_global.pt_0 == select_slot_global.pt_1 ||
      select_slot_global.pt_0 == select_slot_global.pt_2 ||
      select_slot_global.pt_0 == select_slot_global.pt_3 ||
      select_slot_global.pt_1 == select_slot_global.pt_2 ||
      select_slot_global.pt_1 == select_slot_global.pt_3 ||
      select_slot_global.pt_2 == select_slot_global.pt_3) {
    ILOG_ERROR << "slot corner points exist same pt!";
    return false;
  }
  arc_slot_init_out_heading_ = 0.0;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus() ||
      apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus()) {
    arc_slot_init_out_heading_ =
        parallel_out_path_planner_.GetArcSlotParkOutHeading();
    if (arc_slot_init_out_heading_ > M_PI ||
        arc_slot_init_out_heading_ < -M_PI) {
      arc_slot_init_out_heading_ = 0.0;
    }
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
              << ego_info_under_slot.origin_pose_global.pos.x() << " "
              << ego_info_under_slot.origin_pose_global.pos.y();

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

  ILOG_INFO << "ego_pos_slot = " << ego_info_under_slot.cur_pose.pos.x() << " "
            << ego_info_under_slot.cur_pose.pos.y();
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
  // judge arc slot
  JudgeArcSlot();
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

    ILOG_INFO << "limiter start pos = " << t_lane_.limiter.start_pt.x() << " "
              << t_lane_.limiter.start_pt.y();
    ILOG_INFO << "limiter end pos = " << t_lane_.limiter.end_pt.x() << " "
              << t_lane_.limiter.end_pt.y();

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

void ParallelParkOutScenario::CheckEgoPoseWhenParkOutFaild(
    ParkingFailReason reason) {
  ILOG_INFO << "Enter CheckEgoPoseWhenParkOutFaild!";
  if (CheckFinished()) {
    ILOG_INFO << "parallel parking finish!";
    SetParkingStatus(PARKING_FINISHED);
  } else {
    ILOG_INFO << "parallel parking failed!";
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = reason;
  }
  return;
}

const bool ParallelParkOutScenario::CheckFinished() {
  ILOG_INFO << "start CheckFinished!";
  if (frame_.is_replan_first) {
    ILOG_INFO << "before first finish, not check finish";
    return false;
  }

  if (!path_end_heading_is_met_){
    return false;
  }

  const EgoInfoUnderSlot& ego_slot_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const double side_sgn = frame_.is_park_out_left ? 1.0 : -1.0;

  pnc::geometry_lib::LocalToGlobalTf ego2slot;
  ego2slot.Init(ego_slot_info.cur_pose.pos, ego_slot_info.cur_pose.heading);

  const auto& front_in_wheel = ego2slot.GetPos(
      Eigen::Vector2d(apa_param.GetParam().wheel_base,
                      0.5 * apa_param.GetParam().car_width * (-side_sgn)));

  const auto& rear_in_wheel = ego2slot.GetPos(
      Eigen::Vector2d(0.0, 0.5 * apa_param.GetParam().car_width * (-side_sgn)));

  const double slot_outer_pt_y =
      apa_param.GetParam().finish_parallel_out_lat_wheel_y *
      ego_slot_info.slot.slot_width_ * side_sgn;
  bool lat_condition = false;



  double finish_parallel_out_heading_mag =
      apa_param.GetParam().finish_parallel_out_heading_mag;
  if (is_arc_slot_) {
    finish_parallel_out_heading_mag += 10.0;
    ILOG_INFO << "finish_parallel_out_heading_mag:"
              << finish_parallel_out_heading_mag;
  } else if (astar_state_ != AstarSearchState::NONE) {
    finish_parallel_out_heading_mag = 6.0;
    ILOG_INFO << "using astar heading:" << finish_parallel_out_heading_mag;
  }

  if (side_sgn > 0.0) {
    const double wheel_limit_y = slot_outer_pt_y;
    lat_condition = (rear_in_wheel.y() >= wheel_limit_y) &&
                    (front_in_wheel.y() >= wheel_limit_y);
    if (is_outer_arc_slot_) {
      lat_condition = (rear_in_wheel.y() >= wheel_limit_y) ||
                      ego_slot_info.cur_pose.pos.x() >
                          ego_slot_info.slot.slot_length_ - 1.0;;
    }
  } else {
    const double wheel_limit_y = slot_outer_pt_y;
    lat_condition = (rear_in_wheel.y() <= wheel_limit_y) &&
                    (front_in_wheel.y() <= wheel_limit_y);
    if (is_outer_arc_slot_) {
      lat_condition = (rear_in_wheel.y() <= wheel_limit_y) ||
                      ego_slot_info.cur_pose.pos.x() >
                          ego_slot_info.slot.slot_length_ - 1.0;
    }
  }
  ILOG_INFO << "is_outer_arc_slot_:" << is_outer_arc_slot_;
  ILOG_INFO << "is_arc_slot_:" << is_arc_slot_;
  ILOG_INFO << "front_in_wheel = " << front_in_wheel.x() << " "
            << front_in_wheel.y() << " rear_in_wheel = " << rear_in_wheel.x()
            << " " << rear_in_wheel.y();
  ILOG_INFO << "slot_outer_pt_y = " << slot_outer_pt_y;
  ILOG_INFO << "lat_condition = " << lat_condition;

  const double slot_occupied_ratio = apa_world_ptr_->GetSlotManagerPtr()
                                         ->GetEgoInfoUnderSlot()
                                         .slot_occupied_ratio;
  double heading_offset =0.0;
  if (is_last_pose_set_){
    heading_offset = last_target_pose_.heading;
  }else{
    heading_offset = arc_slot_init_out_heading_;
  }
  const double heading_mag_deg = std::fabs((apa_world_ptr_->GetSlotManagerPtr()
                                                ->GetEgoInfoUnderSlot()
                                                .cur_pose.heading -
                                            heading_offset) *
                                           kRad2Deg);

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  ILOG_INFO << "heading_mag_deg = " << heading_mag_deg;
  ILOG_INFO << "static_condition = " << static_condition;
  ILOG_INFO << "slot_occupied_ratio = " << slot_occupied_ratio;

  if (!static_condition) {
    delay_check_finish_ = false;
  }
  if (delay_check_finish_) {
    ILOG_INFO << "delay_check_finish_ = " << delay_check_finish_;
    return false;
  }

  return static_condition && slot_occupied_ratio < 0.1 &&
         heading_mag_deg < finish_parallel_out_heading_mag && lat_condition;
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
  double is_low_curb = false;
  int curb_obs_num = 0;
  int low_curb_obs_num = 0;

  const Eigen::Vector2d slot_center(0.5 * slot_length, 0.0);
  obs_pt_local_vec_.clear();
  obs_id_pt_map_.clear();
  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);
  apa_world_ptr_->GetCollisionDetectorPtr()->SetParam(
      CollisionDetector::Paramters(kDeletedObsDistOutSlot, true));

  for (const auto& pair :
       apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles()) {
    if (pair.second.GetObsMovementType() != ApaObsMovementType::STATIC) {
      continue;
    }
    bool is_limiter = false;
    if (pair.second.GetObsScemanticType() == ApaObsScemanticType::LIMITER) {
      is_limiter = true;
    }

    auto obs_scement = pair.second.GetObsScemanticType();
    const auto obs_parent_id = pair.second.GetParentId();
    ApaObsHeightType obs_height = pair.second.GetObsHeightType();

    const bool is_rigid = (obs_scement == ApaObsScemanticType::WALL ||
                           obs_scement == ApaObsScemanticType::COLUMN ||
                           obs_scement == ApaObsScemanticType::CAR);
    auto pair_obs = std::move(pair.second);
    for (auto& obs_pt_local : pair_obs.GetPtClout2dLocal()) {
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
      bool curb_obs_condition_x =
          pnc::mathlib::IsInBound(obs_pt_local.x(), -3.0, slot_length + 4);
      bool curb_obs_condition_y = pnc::mathlib::IsInBound(
          -obs_pt_local.y() * side_sgn, 0.4, 0.5 * slot_width + 2);
      if (curb_obs_condition_x && curb_obs_condition_y) {
        obs_id_pt_map_[obs_parent_id].emplace_back(obs_pt_local);
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
              obs_pt_local, ego_info_under_slot.cur_pose, 0.0168)) {
        // in_ego_cnt++;

        obs_scement = ApaObsScemanticType::CURB_INSIDE;
      }

      if (mathlib::IsInBound(obs_pt_local.x(), 0.8, slot_length - 0.8) &&
          obs_pt_local.y() * side_sgn < -0.25 * slot_width * side_sgn &&
          obs_pt_local.y() * side_sgn >
              -(0.5 * slot_width + apa_param.GetParam().curb_offset) *
                  side_sgn &&
          is_rigid) {
        ILOG_INFO << "rigid obs = " << obs_pt_local.x() << " "
                  << obs_pt_local.y();
        t_lane_.is_inside_rigid = true;
      }

      if (is_limiter) {
        auto obs_pt_local_cp = obs_pt_local;
        ILOG_INFO << "befor limiter obs = " << obs_pt_local_cp.x();
        obs_pt_local_cp.x() = obs_pt_local_cp.x() + 0.8;
        ILOG_INFO << "limiter obs = " << obs_pt_local_cp.x();
        obs_pt_local_vec_[static_cast<size_t>(obs_scement)].emplace_back(
            std::move(obs_pt_local_cp));
        continue;
      }

      if (mathlib::IsInBound(obs_pt_local.x(), 0.5, slot_length - 0.5) &&
          (obs_pt_local.y() * side_sgn <= -quarter_slot_width)) {
        curb_obs_num++;
        if ((obs_height == ApaObsHeightType::RUN_OVER ||
             obs_height == ApaObsHeightType::LOW)) {
          low_curb_obs_num++;
        }
      }

      obs_pt_local_vec_[static_cast<size_t>(obs_scement)].emplace_back(
          std::move(obs_pt_local));
    }
  }

  ILOG_INFO << "curb_obs_num = " << curb_obs_num
            << " low_curb_obs_num = " << low_curb_obs_num;
  const double low_height_curb_ratio = 0.25;
  is_low_curb =
      (double(low_curb_obs_num) > (curb_obs_num * low_height_curb_ratio))
          ? true
          : false;

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
  double curb_y_limit_all = -side_sgn * (half_slot_width + kCurbInitialOffset);

  ILOG_INFO << "obs_pt_local_vec_ size =" << obs_pt_local_vec_.size();

  for (const auto& obstacle_point_set : obs_pt_local_vec_) {
    for (const auto& obstacle_point_slot : obstacle_point_set.second) {
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

      const bool curb_condition_all =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(), 0.0, slot_length) &&
          (obstacle_point_slot.y() * side_sgn < -(slot_width * 0.5));
      if (curb_condition_all &&
          obstacle_point_set.first !=
              static_cast<size_t>(ApaObsScemanticType::CAR)) {
        if (side_sgn > 0.0) {
          curb_y_limit_all =
              std::max(curb_y_limit_all, obstacle_point_slot.y());
        } else {
          curb_y_limit_all =
              std::min(curb_y_limit_all, obstacle_point_slot.y());
        }
      }

      const bool front_parallel_line_condition =
          pnc::mathlib::IsInBound(
              obstacle_point_slot.x(), slot_length - 0.2,
              slot_length + kFrontDetaXMagWhenFrontVacant) &&
          pnc::mathlib::IsInBound(obstacle_point_slot.y(), 0.0, 2.5 * side_sgn);

      if (front_parallel_line_condition) {
        front_parallel_line_y_limit =
            side_sgn > 0.0
                ? std::max(front_parallel_line_y_limit, obstacle_point_slot.y())
                : std::min(front_parallel_line_y_limit,
                           obstacle_point_slot.y());
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

  if (rear_vacant) {
    curb_y_limit = side_sgn ? std::min(curb_y_limit_all, curb_y_limit)
                            : std::max(curb_y_limit_all, curb_y_limit);
    ILOG_INFO << "curb_y_limit =" << curb_y_limit
              << " curb_y_limit_all = " << curb_y_limit_all;
  }

  if (is_low_curb) {
    ILOG_INFO << "curb_y_limit = " << curb_y_limit;
    curb_y_limit =
        std::fabs(curb_y_limit) >
                std::fabs(half_slot_width +
                          apa_param.GetParam().curb_offset_by_height)
            ? curb_y_limit
            : -side_sgn * (half_slot_width +
                           apa_param.GetParam().curb_offset_by_height);
    ILOG_INFO << "is_low_curb update curb_y_limit = " << curb_y_limit;
  }

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
  ILOG_INFO << "obs_pt_inside = " << t_lane_.obs_pt_inside.x() << " "
            << t_lane_.obs_pt_inside.y();
  ILOG_INFO << "obs_pt_outside = " << t_lane_.obs_pt_outside.x() << " "
            << t_lane_.obs_pt_outside.y();
  ILOG_INFO << "corner_inside_slot = " << t_lane_.corner_inside_slot.x() << " "
            << t_lane_.corner_inside_slot.y();
  ILOG_INFO << "corner_outside_slot = " << t_lane_.corner_outside_slot.x()
            << " " << t_lane_.corner_outside_slot.y();

  ILOG_INFO << "pt_outside = " << t_lane_.pt_outside.x();
  ILOG_INFO << "pt_inside = " << t_lane_.pt_inside.x();

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
  ILOG_INFO << "--------------- GenTBoundaryObstacles ------------------------";
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

  for (const auto& obstacle_point_set : obs_pt_local_vec_) {
    for (const auto& obstacle_point_slot : obstacle_point_set.second) {
      // add obs near channel
      const bool channel_y_condition =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(), channel_point_1.x(),
                                  channel_point_2.x()) &&
          pnc::mathlib::IsInBound(obstacle_point_slot.y(),
                                  kMinChannelYMagIdentification * slot_side_sgn,
                                  channel_point_1.y());

      if (!channel_y_condition) {
        const bool channel_y_condition_try =
            pnc::mathlib::IsInBound(obstacle_point_slot.x(), B.x(), E.x()) &&
            pnc::mathlib::IsInBound(
                obstacle_point_slot.y(),
                t_lane_.corner_outside_slot.y() * slot_side_sgn,
                kMinChannelYMagIdentification * slot_side_sgn);
        if (channel_y_condition_try) {
          filtered_channel_obs_vec.emplace_back(obstacle_point_slot);
        }
      }
      if (channel_y_condition) {
        // ILOG_INFO << "ARC SLOT DO NOT USE OBS";
        filtered_channel_obs_vec.emplace_back(obstacle_point_slot);

        if (pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                    t_lane_.slot_length,
                                    t_lane_.slot_length + 5.0) &&
                                    !is_arc_slot_) {
          if (slot_side_sgn > 0.0) {
            t_lane_.channel_y =
                std::min(obstacle_point_slot.y(), t_lane_.channel_y);
          } else {
            t_lane_.channel_y =
                std::max(obstacle_point_slot.y(), t_lane_.channel_y);
          }
        }
      }
      const bool strict_channel_y_condition =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                  t_lane_.obs_pt_outside.x(),
                                  t_lane_.obs_pt_inside.x()) &&
          pnc::mathlib::IsInBound(
              obstacle_point_slot.y(),
              kStrictChannelYMagIdentification * slot_side_sgn,
              channel_point_1.y());
      if (strict_channel_y_condition) {
        strict_channel_y =
            std::min(std::fabs(obstacle_point_slot.y()), strict_channel_y);
      }
    }
  }
  ILOG_INFO << "strict_channel_y: " << strict_channel_y;
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      filtered_channel_obs_vec, CollisionDetector::CHANNEL_OBS);

  // set tlane obs
  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;
  ILOG_INFO << "-----------tlane AB/EF-------------";
  // set tlane parallel line obs
  tlane_line.SetPoints(A, B);
  tlane_line_vec.emplace_back(tlane_line);

  if (!is_outer_arc_slot_) {
    tlane_line.SetPoints(E, F);
    tlane_line_vec.emplace_back(tlane_line);
  }

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
  for (const auto& obstacle_point_set : obs_pt_local_vec_) {
    for (const auto& obstacle_point_slot : obstacle_point_set.second) {
      const bool is_rear_tlane_line =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(), A.x(),
                                  t_lane_.obs_pt_outside.x()) &&
          pnc::mathlib::IsInBound(
              obstacle_point_slot.y(), B.y(),
              kMinChannelYMagIdentification * slot_side_sgn);

      if (is_rear_tlane_line) {
        tlane_obstacle_vec.emplace_back(obstacle_point_slot);
        continue;
      }

      const bool is_front_tlane_line =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                  t_lane_.obs_pt_inside.x() - 0.3, F.x()) &&
          pnc::mathlib::IsInBound(
              obstacle_point_slot.y(), E.y(),
              kMinChannelYMagIdentification * slot_side_sgn);

      if (is_front_tlane_line) {
        tlane_obstacle_vec.emplace_back(obstacle_point_slot);
      }
    }
  }
  ILOG_INFO << "-----------tlane BC/DE-------------";

  // tlane
  tlane_line_vec.clear();

  const Eigen::Vector2d C_curb(C.x(), t_lane_.curb_y);
  const Eigen::Vector2d D_curb(D.x(), t_lane_.curb_y);

  tlane_line.SetPoints(B, C_curb);
  tlane_line_vec.emplace_back(tlane_line);

  if (!is_outer_arc_slot_) {
    tlane_line.SetPoints(D_curb, E);
    tlane_line_vec.emplace_back(tlane_line);
  }

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
  std::vector<Eigen::Vector2d> curb_inside_obs;
  for (const auto& obstacle_point_set : obs_pt_local_vec_) {
    const auto obs_type = obstacle_point_set.first;

    for (const auto& obs_pos : obstacle_point_set.second) {
      if(obs_type == static_cast<size_t>(ApaObsScemanticType::CURB_INSIDE)){
        curb_inside_obs.emplace_back(obs_pos);
      }


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
  }
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      curb_inside_obs, CollisionDetector::CURB_OBS);


  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      tlane_obstacle_vec, CollisionDetector::TLANE_BOUNDARY_OBS);

  ILOG_INFO << "-----------tlane CD-------------";
  point_set.clear();
  tlane_obstacle_vec.clear();
  std::vector<double> y_of_c_d_curb_y={C.y(), D.y(), t_lane_.curb_y};
  bool is_use_curb_obs = ProcessCurbPointsAndGetPoints(
      point_set, obs_id_pt_map_, C_curb, D_curb, y_of_c_d_curb_y);
  if (!is_use_curb_obs) {
    ILOG_INFO << "use origin curb obs";
    tlane_line.SetPoints(C_curb, D_curb);
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, tlane_line,
                                               kTBoundarySampleDist);
  }
  // 记录首尾 被过滤（车内） 的点的 x
  bool filter_start = false;
  double first_filtered_x = 0.0;
  double last_filtered_x = 0.0;

  for (const auto& obs : point_set) {
    if (is_use_curb_obs) {
      if (obs.x() <= C_curb.x() - 1e-5 || obs.x() > D_curb.x() + 1e-5 ||
          std::abs(obs.y()) < std::abs(t_lane_.curb_y) - 1e-2) {
        continue;
      }
    }
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs, ego_info_under_slot.cur_pose, kDeletedObsDistInSlot)) {
      tlane_obstacle_vec.emplace_back(obs);
    } else {
      // 被过滤（在车内）的点
      ILOG_INFO << "Filtered point: x=" << obs.x() << ", y=" << obs.y();

      if (!filter_start) {
        first_filtered_x = obs.x();
        last_filtered_x = obs.x();
        filter_start = true;
      } else {
        last_filtered_x = obs.x();
      }
    }
  }

  if (filter_start) {
    double xmin = std::min(first_filtered_x, last_filtered_x);
    double xmax = std::max(first_filtered_x, last_filtered_x);
    double eps = 1e-6;

    auto it =
        std::remove_if(tlane_obstacle_vec.begin(), tlane_obstacle_vec.end(),
                       [&](const Eigen::Vector2d& p) {
                         return (p.x() >= xmin - eps && p.x() <= xmax + eps);
                       });
    if (it != tlane_obstacle_vec.end()) {
      tlane_obstacle_vec.erase(it, tlane_obstacle_vec.end());
    }
    ILOG_INFO << "Pruned tlane CD samples between x=[" << xmin << ", " << xmax
              << "], remaining=" << tlane_obstacle_vec.size();
  }

  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      tlane_obstacle_vec, CollisionDetector::CURB_OBS);

  ILOG_INFO << "-----------tlane limiter-------------";
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
              obs, ego_info_under_slot.cur_pose, 0.0)) {
        tlane_obstacle_vec.emplace_back(obs);
      }
    }
    apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
        tlane_obstacle_vec, CollisionDetector::LIMITER_OBS);
  }
}

const PathPlannerResult ParallelParkOutScenario::PathPlanOnceGeometry() {
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
  bool is_in_slot = ego_info_under_slot.slot_occupied_ratio > 0.0;
  if (apa_world_ptr_->GetLocalViewPtr()
              ->function_state_machine_info.current_state ==
          iflyauto::FunctionalState::FunctionalState_PARK_PRE_ACTIVE &&
          !previous_parallel_out_path_planner_.GetOutput()
           .all_gear_path_point_vec.empty() &&
      !is_last_pose_set_) {
    last_target_pose_ = previous_parallel_out_path_planner_.GetOutput()
                            .all_gear_path_point_vec.back();
    last_target_pose_.heading = arc_slot_init_out_heading_;

    is_last_pose_set_ = true;
  }
  path_planner_input.last_target_pose_ = last_target_pose_;
  path_planner_input.is_before_running_stage =
      apa_world_ptr_->GetLocalViewPtr()
              ->function_state_machine_info.current_state ==
          iflyauto::FunctionalState::FunctionalState_PARK_PRE_ACTIVE ||
      apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus();

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

  PathPlannerResult plan_result = PathPlannerResult::PLAN_FAILED;
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
  ILOG_INFO << "gear cmd = " << static_cast<int>(frame_.gear_command);
  if (apa_param.GetParam().is_trim_limter_parallel_enable &&
      !path_planner_input.is_searching_stage) {
    // parallel_path_planner_.TrimPathByLimiterLastPathVec(true);
    parallel_out_path_planner_.TrimPathByLimiterPathPoint(
        current_path_point_global_vec_);
  }
  parallel_out_path_planner_.SampleCurrentPathSeg();
  parallel_out_path_planner_.JudgeNeedOptimize();

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
  path_end_heading_is_met_ = false;
  for (const auto& path_point : path_planner_output.path_point_vec) {
    global_point.Set(ego_info_under_slot.l2g_tf.GetPos(path_point.pos),
                     ego_info_under_slot.l2g_tf.GetHeading(path_point.heading));
    global_point.s = path_point.s;
    global_point.kappa = path_point.kappa;
    global_point.gear = frame_.current_gear;

    current_path_point_global_vec_.emplace_back(global_point);
  }
  if (!path_planner_output.path_point_vec.empty()) {
    path_end_heading_is_met_ =
        std::abs(path_planner_output.path_point_vec.back().GetTheta() -
                 last_target_pose_.heading) < pnc::mathlib::Deg2Rad(2.5);
  }
  previous_current_path_point_global_vec_.clear();
  previous_current_path_point_global_vec_ = current_path_point_global_vec_;
  if (!apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    std::vector<geometry_lib::PathPoint> tmp_path_point_vec;
    tmp_path_point_vec = current_path_point_global_vec_;
    parallel_out_path_planner_.TrimPathByLimiterPathPoint(tmp_path_point_vec,
                                                          true);
    if (tmp_path_point_vec.size() < current_path_point_global_vec_.size()) {
      current_path_point_global_vec_ = tmp_path_point_vec;
    }
  }
  if (parkout_direction_ != ApaParkOutDirection::INVALID) {
    std::vector<pnc::geometry_lib::PathPoint> cur_dir_path_point_global_vec;
    for (const auto& path_point : path_planner_output.all_gear_path_point_vec) {
      global_point.Set(
          ego_info_under_slot.l2g_tf.GetPos(path_point.pos),
          ego_info_under_slot.l2g_tf.GetHeading(path_point.heading));
      global_point.lat_buffer = path_point.lat_buffer;
      global_point.s = path_point.s;
      global_point.kappa = path_point.kappa;
      global_point.gear = path_point.gear;
      cur_dir_path_point_global_vec.emplace_back(global_point);
    }
    multi_parkout_path_vec[parkout_direction_] = cur_dir_path_point_global_vec;
  }

  complete_path_point_global_vec_.clear();
  if (multi_parkout_path_vec.find(parkout_direction_) !=
      multi_parkout_path_vec.end()) {
    complete_path_point_global_vec_ =
        multi_parkout_path_vec[parkout_direction_];
  }

  return plan_result;
}

void ParallelParkOutScenario::SetRequestForScenarioTry(
    AstarRequest& cur_request, const EgoInfoUnderSlot& ego_info) {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingOutStatus()) {
    const double slot_width = ego_info.slot.GetWidth();
    const double slot_length = ego_info.slot.GetLength();

    const double park_out_offset_y = apa_param.GetParam().car_width + 0.2;
    const double park_out_offset_x = 0.0;  // 2.49;

    cur_request.direction_request_size = 2;
    cur_request.direction_request_stack[0] =
        ParkingVehDirection::HEAD_OUT_TO_LEFT;
    // cur_request.direction_request_stack[1] =
    //     ParkingVehDirection::HEAD_OUT_TO_MIDDLE;
    cur_request.direction_request_stack[1] =
        ParkingVehDirection::HEAD_OUT_TO_RIGHT;

    // const Eigen::Vector2d temp_head_pos(base_x, 0.0);
    const Eigen::Vector2d temp_head_pos(ego_info.cur_pose.pos);
    const Eigen::Vector2d target_pos_head_left =
        temp_head_pos + Eigen::Vector2d(park_out_offset_x, park_out_offset_y);
    // temp_head_pos +
    // direction_origin_corner_23_normalized_ * head_out_offset_y;
    const Eigen::Vector2d target_pos_head_right =
        temp_head_pos + Eigen::Vector2d(park_out_offset_x, -park_out_offset_y);

    cur_request.real_goal_stack[0] =
        Pose2f(target_pos_head_left.x(), target_pos_head_left.y(), 0.0);
    cur_request.real_goal_stack[1] =
        Pose2f(target_pos_head_right.x(), target_pos_head_right.y(), 0.0);
  } else {
    cur_request.direction_request_size = 0;
  }
}

void ParallelParkOutScenario::SetReleaseDirection(
    iflyauto::APAHMIData& apa_hmi_data, const AstarRequest& cur_request) {
  ApaDirectionGenerator generator;
  generator.ClearReleaseDirectionFlag(apa_hmi_data);
  generator.ClearRecommendationDirectionFlag(apa_hmi_data);
  generator.SetReleaseDirectionFlag(apa_hmi_data, ParityBit);
  generator.SetRecommendationDirectionFlag(apa_hmi_data, ParityBit);

  ApaRecommendationDirection dir = ParityBit;

  bool is_left_dir = false;
  bool is_right_dir = false;
  for (int8_t i = 0; i < response_.feasible_directions.size(); i++) {
    if (response_.feasible_directions[i] == false) {
      continue;
    }

    apa_world_ptr_->GetSlotManagerPtr()
        ->GetMutableEgoInfoUnderSlot()
        .slot.release_info_
        .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
        SlotReleaseState::RELEASE;

    switch (cur_request.direction_request_stack[i]) {
      case ParkingVehDirection::HEAD_OUT_TO_LEFT:
        dir = ApaRecommendationDirection::ParallelFrontLeft;
        is_left_dir = true;
        break;
      case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
        dir = ApaRecommendationDirection::ParallelFrontRight;
        is_right_dir = true;
        break;
      default:
        break;
    }

    generator.SetReleaseDirectionFlag(apa_hmi_data, dir);
  }
  ApaRecommendationDirection planning_recommend_park_dir = ParityBit;
  if (is_right_dir) {
    planning_recommend_park_dir =
        ApaRecommendationDirection::ParallelFrontRight;
  }
  if (is_left_dir) {
    planning_recommend_park_dir = ApaRecommendationDirection::ParallelFrontLeft;
  }
  generator.SetRecommendationDirectionFlag(apa_hmi_data,
                                           planning_recommend_park_dir);
}

void ParallelParkOutScenario::SetTargetGroup(AstarRequest& cur_request) {
  const EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const double slot_width = ego_info.slot.GetWidth();
  const double slot_length = ego_info.slot.GetLength();

  // The center of the front slot is shifted down by half an wheelbase
  const double target_x =
      (slot_length * 1.5) - (apa_param.GetParam().wheel_base * 0.5) + 0.3;
  double target_y =
      (slot_width * 0.5) + (apa_param.GetParam().car_width * 0.5) + 0.3;

  double target_heading = 0.0;
  if (is_arc_slot_) {
    target_heading = arc_slot_init_out_heading_;
    ILOG_INFO << "update astar target heading: " << target_heading;
  }

  const double min_strict_channel_y = 4.0;
  ILOG_INFO << "strict_channel_y: " << strict_channel_y;
  if (strict_channel_y < min_strict_channel_y) {
    target_y = strict_channel_y - (apa_param.GetParam().car_width * 0.5) - 0.3;
  }

  float sign = frame_.is_park_out_left ? 1.0 : -1.0;
  size_t x_nums = 6;
  size_t y_nums = 3;
  const double step_x = 1.0;
  const double step_y = 0.3;
  const bool is_searchout_state =
      apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingOutStatus();
  if (is_searchout_state) {
    x_nums = 4;
    y_nums = 2;
  }
  for (size_t i = 0; i < x_nums; i++) {
    float x = target_x - step_x * i;
    for (size_t j = 0; j < y_nums; j++) {
      float y = (target_y + step_y * j) * sign;
      if (strict_channel_y < min_strict_channel_y) {
        y = (target_y - step_y * j) * sign;
      }
      cur_request.parallel_target_group.emplace_back(
          Pose2f(x, y, target_heading));
      if (is_searchout_state) {
        float y_flipp = y * -1.0;
        cur_request.parallel_target_group.emplace_back(
            Pose2f(x, y_flipp, target_heading));
      }
    }
  }

  // for (size_t i = 0; i < cur_request.parallel_target_group.size(); i++) {
  //   ILOG_INFO << "parallel_target_group: " << i << " "
  //             << cur_request.parallel_target_group[i].x << " "
  //             << cur_request.parallel_target_group[i].y << " heading: "
  //             << cur_request.parallel_target_group[i].GetPhi() * kRad2Deg;
  // }
  return;
}

void ParallelParkOutScenario::UpdateStartPosByInSlot(
    Pose2f& start_pos, pnc::geometry_lib::PathSegGear& last_path_gear) {
  const EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  last_path_gear = pnc::geometry_lib::PathSegGear::SEG_GEAR_INVALID;
  if (ego_info.slot_occupied_ratio < 0.1) {
    ILOG_INFO << "ego is out slot";
    return;
  }

  const auto& path_by_direction =
      parallel_out_path_planner_.GetPathByDirection();
  ApaParkOutDirection out_dir = frame_.is_park_out_left
                                    ? ApaParkOutDirection::LEFT_FRONT
                                    : ApaParkOutDirection::RIGHT_FRONT;
  ILOG_INFO << "out_dir: " << int(out_dir);
  if (path_by_direction.find(out_dir) == path_by_direction.end()) {
    ILOG_INFO << "path_by_direction not found";
    return;
  }
  const auto& path_segment_vec = path_by_direction.at(out_dir);
  if (path_segment_vec.empty()) {
    ILOG_INFO << "path_segment_vec is empty";
    return;
  }
  if (path_segment_vec.back().seg_type != pnc::geometry_lib::SEG_TYPE_ARC) {
    ILOG_INFO << "last path is not arc";
    return;
  }
  if (std::fabs(path_segment_vec.back().GetEndHeading()) >
      kArcEndHeadThreshold * kDeg2Rad) {
    ILOG_INFO << "arc end heading too large";
    return;
  }
  last_path_gear =
      pnc::geometry_lib::PathSegGear(path_segment_vec.back().seg_gear);

  ILOG_INFO << "debug last path: ";
  path_segment_vec.back().PrintInfo();
  start_pos.SetPose(path_segment_vec.back().GetArcSeg().pB.x(),
                    path_segment_vec.back().GetArcSeg().pB.y(),
                    path_segment_vec.back().GetArcSeg().headingB);
  return;
}

void ParallelParkOutScenario::UpdatePathByGeometry() {
  const EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  if (ego_info.slot_occupied_ratio < 0.1) {
    ILOG_INFO << "ego is out slot";
    return;
  }
  const auto& path_by_direction =
      parallel_out_path_planner_.GetPathByDirection();
  ApaParkOutDirection out_dir = frame_.is_park_out_left
                                    ? ApaParkOutDirection::LEFT_FRONT
                                    : ApaParkOutDirection::RIGHT_FRONT;
  ILOG_INFO << "out_dir: " << int(out_dir);
  if (path_by_direction.find(out_dir) == path_by_direction.end()) {
    ILOG_INFO << "path_by_direction not found";
    return;
  }
  const auto& path_segment_vec = path_by_direction.at(out_dir);
  if (path_segment_vec.empty()) {
    ILOG_INFO << "path_segment_vec is empty";
    return;
  }
  if (std::fabs(path_segment_vec.back().GetEndHeading()) >
      kArcEndHeadThreshold * kDeg2Rad) {
    ILOG_INFO << "arc end heading too large";
    return;
  }
  pnc::geometry_lib::PathPoint global_back_pose(
      ego_info.l2g_tf.GetPos(path_segment_vec.back().GetEndPos()),
      ego_info.l2g_tf.GetHeading(path_segment_vec.back().GetEndHeading()));
  if ((complete_path_point_global_vec_[0].pos - global_back_pose.pos).norm() >
      0.1) {
    ILOG_INFO << "cur_pose is too far from path_segment_vec[0]";
    return;
  }

  double sample_ds = 0.05;
  std::vector<pnc::geometry_lib::PathSegment> cur_gear_path_segment_vec;
  cur_gear_path_segment_vec.emplace_back(path_segment_vec[0]);
  ILOG_INFO << "path_segment_vec[" << 0
            << "].seg_gear: " << int(path_segment_vec[0].seg_gear);
  for (size_t i = 1; i < path_segment_vec.size(); ++i) {
    ILOG_INFO << "path_segment_vec[" << i
              << "].seg_gear: " << int(path_segment_vec[i].seg_gear);
    if (path_segment_vec[i].seg_gear != path_segment_vec[0].seg_gear) {
      break;
    }
    cur_gear_path_segment_vec.emplace_back(path_segment_vec[i]);
  }

  auto complete_path_sample =
      pnc::geometry_lib::SamplePathSegVec(path_segment_vec, sample_ds);

  std::vector<pnc::geometry_lib::PathPoint> complete_path;
  complete_path.reserve(complete_path_sample.size());
  complete_path.emplace_back(complete_path_sample[0]);
  for (size_t i = 1; i < complete_path_sample.size(); ++i) {
    if (pnc::geometry_lib::CheckTwoPoseIsSame(complete_path_sample[i],
                                              complete_path_sample[i - 1])) {
      continue;
    }
    complete_path.emplace_back(complete_path_sample[i]);
  }

  auto path_point_vec_sample =
      pnc::geometry_lib::SamplePathSegVec(cur_gear_path_segment_vec, sample_ds);

  std::vector<pnc::geometry_lib::PathPoint> path_point_vec;
  path_point_vec.reserve(path_point_vec_sample.size());
  path_point_vec.emplace_back(path_point_vec_sample[0]);
  for (size_t i = 1; i < path_point_vec_sample.size(); ++i) {
    if (pnc::geometry_lib::CheckTwoPoseIsSame(path_point_vec_sample[i],
                                              path_point_vec_sample[i - 1])) {
      continue;
    }
    path_point_vec.emplace_back(path_point_vec_sample[i]);
  }

  ILOG_INFO << "complete_path.size: " << complete_path.size();
  ILOG_INFO << "path_point_vec.size: " << path_point_vec.size();
  if (path_point_vec.size() < 3) {
    ILOG_INFO << "path_point_vec.size < 3";
    return;
  }

  std::vector<pnc::geometry_lib::PathPoint> all_global_point_vec;
  all_global_point_vec.reserve(complete_path.size() +
                               complete_path_point_global_vec_.size());
  for (const auto& path_point : complete_path) {
    pnc::geometry_lib::PathPoint all_global_point;
    all_global_point.Set(ego_info.l2g_tf.GetPos(path_point.pos),
                         ego_info.l2g_tf.GetHeading(path_point.heading));
    all_global_point.s = path_point.s;
    all_global_point.kappa = path_point.kappa;

    all_global_point_vec.emplace_back(all_global_point);
  }
  double last_s = complete_path[complete_path.size() - 1].s;

  for (size_t i = 1; i < complete_path_point_global_vec_.size(); ++i) {
    pnc::geometry_lib::PathPoint all_global_point;
    all_global_point = complete_path_point_global_vec_[i];
    all_global_point.s += last_s;
    all_global_point_vec.emplace_back(all_global_point);
  }

  // ILOG_INFO << "debug";
  // for (size_t i = 0; i < all_global_point_vec.size(); ++i) {
  //   all_global_point_vec[i].PrintInfo();
  // }

  complete_path_point_global_vec_.clear();
  complete_path_point_global_vec_ = all_global_point_vec;

  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    return;
  }

  std::vector<pnc::geometry_lib::PathPoint> global_point_vec;
  global_point_vec.reserve(path_point_vec.size() +
                           current_path_point_global_vec_.size());
  for (const auto& path_point : path_point_vec) {
    pnc::geometry_lib::PathPoint global_point;
    global_point.Set(ego_info.l2g_tf.GetPos(path_point.pos),
                     ego_info.l2g_tf.GetHeading(path_point.heading));
    global_point.s = path_point.s;
    global_point.kappa = path_point.kappa;

    global_point_vec.emplace_back(global_point);
  }

  Eigen::Vector2d in_slot_path_dir =
      global_point_vec[global_point_vec.size() - 1].GetPos() -
      global_point_vec[global_point_vec.size() - 2].GetPos();
  Eigen::Vector2d astart_dir = current_path_point_global_vec_[1].GetPos() -
                               current_path_point_global_vec_[0].GetPos();
  size_t start_i = 1;
  for (size_t i = 1; i < current_path_point_global_vec_.size(); ++i) {
    Eigen::Vector2d astart_dir_i =
        current_path_point_global_vec_[i].GetPos() -
        global_point_vec[global_point_vec.size() - 2].GetPos();
    double dot_AB_AC = in_slot_path_dir.dot(astart_dir_i);
    double dot_AB_AB = in_slot_path_dir.dot(in_slot_path_dir);
    double t = dot_AB_AC / dot_AB_AB;
    if (t > 1.0 + kEps) {
      start_i = i;
      break;
    }
  }
  ILOG_INFO << "in_slot_path_dir.dot(astart_dir): "
            << in_slot_path_dir.dot(astart_dir) << " start_i: " << start_i;
  if (in_slot_path_dir.dot(astart_dir) > 0) {
    last_s = path_point_vec[path_point_vec.size() - 1].s;
    for (size_t i = start_i; i < current_path_point_global_vec_.size(); ++i) {
      pnc::geometry_lib::PathPoint global_point;
      global_point = current_path_point_global_vec_[i];
      global_point.s += last_s;
      global_point_vec.emplace_back(global_point);
    }
  }

  // ILOG_INFO << "debug";
  // for (size_t i = 0; i < global_point_vec.size(); ++i) {
  //   global_point_vec[i].PrintInfo();
  // }

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_ = global_point_vec;

  current_gear_ =
      GetAstarGearFromSegGear(cur_gear_path_segment_vec[0].seg_gear);
  frame_.current_gear = GetGear(current_gear_);

  frame_.gear_command = frame_.current_gear;

  return;
}

const bool ParallelParkOutScenario::SetAstarRequest(
    AstarRequest& astar_request) {
  const EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const double slot_width = ego_info.slot.GetWidth();
  const double slot_length = ego_info.slot.GetLength();

  Pose2D start = Pose2D(ego_info.cur_pose.pos[0], ego_info.cur_pose.pos[1],
                        ego_info.cur_pose.heading);

  const double park_out_offset_y =
      frame_.is_park_out_left ? slot_width + 0.4 : -(slot_width + 0.4);
  const double park_out_offset_x = 0.0;  // 2.49;//slot_length * 0.5;
  Pose2D real_goal = Pose2D(park_out_offset_x + start.GetX(),
                            start.GetY() + park_out_offset_y, 0.0);

  Pose2D slot_base_pose = Pose2D(ego_info.origin_pose_global.pos.x(),
                                 ego_info.origin_pose_global.pos.y(),
                                 ego_info.origin_pose_global.heading);
  astar_request.first_action_request.has_request = true;
  astar_request.first_action_request.gear_request = AstarPathGear::NONE;
  astar_request.space_type = ParkSpaceType::PARALLEL_OUT;
  astar_request.direction_request = ParkingVehDirection::TAIL_IN;
  // frame_.is_park_out_left ? ParkingVehDirection::HEAD_OUT_TO_LEFT
  //                         : ParkingVehDirection::HEAD_OUT_TO_RIGHT;
  astar_request.rs_request = RSPathRequestType::NONE;
  astar_request.slot_id = ego_info.slot.GetId();
  // astar_request.path_generate_method =
  //     planning::AstarPathGenerateType::ASTAR_SEARCHING;

  astar_request.slot_width = ego_info.slot.GetWidth();
  astar_request.slot_length = ego_info.slot.GetLength();
  astar_request.base_pose = slot_base_pose;
  astar_request.history_gear = current_gear_;
  astar_request.real_goal =
      Pose2f(real_goal.GetX(), real_goal.GetY(), real_goal.GetPhi());
  astar_request.goal = astar_request.real_goal;
  SetTargetGroup(astar_request);

  Pose2f start_2f = Pose2f(start.GetX(), start.GetY(), start.GetPhi());
  // if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus()) {
  // }
  pnc::geometry_lib::PathSegGear last_path_gear =
      pnc::geometry_lib::PathSegGear::SEG_GEAR_INVALID;
  UpdateStartPosByInSlot(start_2f, last_path_gear);
  astar_request.start_pose = start_2f;
  ILOG_INFO << "start_pose: ";
  start_2f.DebugString();
  // astar_request.swap_start_goal = false;

  FillPlanningReason(astar_request);
  FillPlanningMethod(astar_request);
  FillGearRequest(astar_request, last_path_gear);
  ILOG_INFO << "first gear_request = "
            << int(astar_request.first_action_request.gear_request)
            << " history_gear = " << int(astar_request.history_gear)
            << " current_gear = " << int(current_gear_);

  SetRequestForScenarioTry(astar_request, ego_info);

  ParkingVehDirection dir_type = ParkingVehDirection::NONE;
  const double passage_height = 0.0;

  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus() ||
      frame_.replan_reason == FIRST_PLAN) {
    virtual_wall_decider_.Init(start);
  }
  // const float passage_height = SetPassageHeight(ego_info);

  ILOG_INFO << "frame_.is_park_out_left " << frame_.is_park_out_left;
  pnc::geometry_lib::SlotSide slot_side = frame_.is_park_out_left
                                              ? geometry_lib::SLOT_SIDE_RIGHT
                                              : geometry_lib::SLOT_SIDE_LEFT;
  virtual_wall_decider_.Process(obs_hastar.virtual_obs, ego_info.slot, start,
                                slot_side, dir_type, passage_height);

  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus() ||
      astar_request.plan_reason == PlanningReason::FIRST_PLAN) {
    astar_request.recommend_route_bound =
        virtual_wall_decider_.GetMaxMapBound();
  } else {
    // astar_request.recommend_route_bound = recommend_route_bound_;
    astar_request.recommend_route_bound.Combine(
        virtual_wall_decider_.GetVehBound());
  }

  PointCloudObstacleTransform obstacle_generator;
  cdl::AABB slot_box;
  obstacle_generator.GenerateLocalObstacle(
      apa_world_ptr_->GetObstacleManagerPtr(), obs_hastar, start, slot_box,
      false);

  return true;
}

const PathPlannerResult ParallelParkOutScenario::PubResponseForScenarioTry(
    const EgoInfoUnderSlot& ego_info, const ParkObstacleList& obs) {
  PathPlannerResult res = PathPlannerResult::WAIT_PATH;

  // check result
  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    // get output
    thread_.PublishResponse(&response_);
    RecordSearchTime(response_.time);

    // success, get first gear path length
    bool path_search_valid = false;
    const bool park_out_directions_valid =
        std::any_of(response_.feasible_directions.begin(),
                    response_.feasible_directions.end(),
                    [](bool feasible) { return feasible; });
    if (response_.GetFirstPathLength() > 0.25) {
      path_search_valid = true;
    }
    ILOG_INFO << "path_search_valid = " << path_search_valid
             << ",park_out_directions_valid = " << park_out_directions_valid;
    if (path_search_valid && park_out_directions_valid) {
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
    AstarRequest cur_request;
    SetAstarRequest(cur_request);
    thread_.SetRequest(obs, cur_request);

    res = PathPlannerResult::WAIT_PATH;
    ILOG_INFO << "set input";
  } else if (thread_state_ == RequestResponseState::HAS_REQUEST) {
    res = PathPlannerResult::WAIT_PATH;
    ILOG_INFO << "has input";
  }
  Transform2d response_tf;
  response_tf.SetBasePose(response_.request.base_pose);
  PublishHybridAstarCompletePathInfo(response_.result, &response_tf);
  UpdatePathByGeometry();
  SetReleaseDirection(apa_hmi_, response_.request);

  return res;
}

const PathPlannerResult ParallelParkOutScenario::PubResponseForScenarioRunning(
    const EgoInfoUnderSlot& ego_info, const ParkObstacleList& obs) {
  PathPlannerResult res = PathPlannerResult::WAIT_PATH;
  response_.Clear();

  // check result
  if (thread_state_ == RequestResponseState::HAS_RESPONSE) {
    // get output
    thread_.PublishResponse(&response_);
    RecordSearchTime(response_.time);
    RecordSearchTrajectoryInfo(response_.search_traj_info);

    // get first gear path length
    bool path_search_valid = true;
    if (response_.GetFirstPathLength() < 0.25) {
      ILOG_INFO << "path is too short";
      path_search_valid = false;
    }

    // If path planning failed in slot refresh, do nothing.
    if (response_.request.plan_reason == PlanningReason::SLOT_REFRESHED &&
        !path_search_valid) {
      return PathPlannerResult::PLAN_HOLD;
    }

    // success
    if (path_search_valid) {
      if (frame_.is_replan_first) {
        frame_.is_replan_first = false;
      }

      // if planning success, update gear
      current_gear_ = response_.first_seg_path[0].gear;
      frame_.current_gear = GetGear(current_gear_);

      frame_.gear_command = frame_.current_gear;

      ILOG_INFO << "first path gear = "
                << PathGearDebugString(response_.first_seg_path[0].gear)
                << ",gear_change_num=" << response_.result.gear_change_num;

      // lateral_offset_ = response_.result.y.back();

      path_end_heading_is_met_ = true;
      last_target_pose_.heading = 0.0;
      is_last_pose_set_ = true;

      Transform2d response_tf;
      response_tf.SetBasePose(response_.request.base_pose);
      PublishHybridAstarCompletePathInfo(response_.result, &response_tf);
      PublishHybridAstarCurrentPathInfo(response_.first_seg_path, &response_tf);
      UpdatePathByGeometry();

      if (response_.request.plan_reason != PlanningReason::SLOT_REFRESHED) {
        frame_.total_plan_count++;
      }
      // path_planning_fail_num_ = 0;
      if (ego_info.slot_occupied_ratio > 0.2 &&
          response_.request.plan_reason != PlanningReason::SLOT_REFRESHED) {
        // replan_number_inside_slot_++;
      }

      // UpdateRecommentRouteBox();

      ILOG_INFO << "total_plan_count = "
                << static_cast<int>(frame_.total_plan_count);

      res = PathPlannerResult::PLAN_UPDATE;
    } else {
      res = PathPlannerResult::PLAN_FAILED;
      if (response_.request.plan_reason == PlanningReason::SLOT_REFRESHED) {
        // If path planning in dynamic replan is fail, use history path.
        res = PathPlannerResult::PLAN_HOLD;
      }
    }

    ThreadClearState();
    // ILOG_INFO << "path gear = " << static_cast<int>(frame_.gear_command);
  } else if (thread_state_ == RequestResponseState::NONE ||
             thread_state_ == RequestResponseState::HAS_PUBLISHED_RESPONSE) {
    // send request
    AstarRequest cur_request;
    SetAstarRequest(cur_request);
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

const PathPlannerResult ParallelParkOutScenario::PathPlanOnceHybridAStar() {
  ILOG_INFO << "Enter PathPlanOnce HybridAStar";
  const EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  if (frame_.is_replan_first) {
    current_gear_ =
        GetAstarGearFromSegGear(pnc::geometry_lib::SEG_GEAR_INVALID);
  }

  if (ego_info.slot_occupied_ratio < 0.1 ||
      apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    parallel_out_path_planner_.ClearPathByDirection();
  }

  PathPlannerResult res = PathPlannerResult::WAIT_PATH;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    res = PubResponseForScenarioTry(ego_info, obs_hastar);
  } else {
    res = PubResponseForScenarioRunning(ego_info, obs_hastar);
  }

  return res;
}

const uint8_t ParallelParkOutScenario::PathPlanOnce() {
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  PathPlannerResult plan_result = PathPlannerResult::PLAN_FAILED;
  bool astar_is_replan_first = frame_.is_replan_first;
  if (ego_info_under_slot.slot_occupied_ratio > 0.1) {
    plan_result = PathPlanOnceGeometry();
    if (plan_result == PathPlannerResult::PLAN_UPDATE &&
        astar_state_ == AstarSearchState::NONE) {
      ILOG_INFO << "geometry path planning success";
      return uint8_t(plan_result);
    }
  }

  if (apa_param.GetParam().parallel_enable_hybrid_astar) {
    frame_.is_replan_first = astar_is_replan_first;
    astar_state_ = AstarSearchState::SEARCHING;
    plan_result = PathPlanOnceHybridAStar();
    if (plan_result == PathPlannerResult::PLAN_FAILED) {
      ILOG_INFO << "hybrid astar path planning fail";
      astar_state_ = AstarSearchState::FAILURE;
      return uint8_t(plan_result);
    } else if (plan_result == PathPlannerResult::PLAN_UPDATE) {
      astar_state_ = AstarSearchState::SUCCESS;
    }
  }

  return uint8_t(plan_result);
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
  ILOG_INFO << "obs p out = " << p0_g.x();
  ILOG_INFO << "obs p in = " << p1_g.x();

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
  if (!obs_hastar.virtual_obs.empty()) {
    for (size_t i = 0; i < obs_hastar.virtual_obs.size(); ++i) {
      Eigen::Vector2d obs_p = Eigen::Vector2d(obs_hastar.virtual_obs[i].x,
                                              obs_hastar.virtual_obs[i].y);
      const auto obs_g = l2g_tf.GetPos(obs_p);
      obstaclesX.emplace_back(obs_g.x());
      obstaclesY.emplace_back(obs_g.y());
    }
  } else {
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
  JSON_DEBUG_VALUE("plan_fail_reason", frame_.plan_fail_reason)

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
const bool ParallelParkOutScenario::PostProcessPathPara() {
  const size_t origin_trajectory_size = current_path_point_global_vec_.size();
  if (origin_trajectory_size < 3) {
    frame_.spline_success = false;
    ILOG_INFO << "error: origin_trajectory_size = " << origin_trajectory_size;
    frame_.plan_fail_reason = POST_PROCESS_PATH_POINT_SIZE;
    return false;
  }

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> heading_vec;
  std::vector<double> s_vec;
  std::vector<double> kappa_vec;
  x_vec.reserve(origin_trajectory_size + 1);
  y_vec.reserve(origin_trajectory_size + 1);
  heading_vec.reserve(origin_trajectory_size + 1);
  s_vec.reserve(origin_trajectory_size + 1);
  kappa_vec.reserve(origin_trajectory_size + 1);
  double s = 0.0;
  double ds = 0.0;
  for (size_t i = 0; i < origin_trajectory_size; ++i) {
    pnc::geometry_lib::PathPoint pt = current_path_point_global_vec_[i];
    if (i > 0) {
      pnc::geometry_lib::PathPoint pt_ = current_path_point_global_vec_[i - 1];
      ds = std::hypot(pt.pos.x() - pt_.pos.x(), pt.pos.y() - pt_.pos.y());
      if (ds < 1e-3) {
        continue;
      }
      s += ds;
    }
    x_vec.emplace_back(pt.pos.x());
    y_vec.emplace_back(pt.pos.y());
    heading_vec.emplace_back(pt.heading);
    s_vec.emplace_back(s);
    kappa_vec.emplace_back(pt.kappa);
  }

  size_t x_vec_size = x_vec.size();
  if (x_vec_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: x_vec_size = " << x_vec.size();
    frame_.plan_fail_reason = POST_PROCESS_PATH_POINT_SIZE;
    return false;
  }
  frame_.current_path_length = s;
  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;
  return true;
}

void ParallelParkOutScenario::JudgeArcSlot(){
  const double side_sgn = frame_.is_park_out_left ? 1.0 : -1.0;
  is_arc_slot_ =false;
  is_outer_arc_slot_ =false;
  if (std::abs(arc_slot_init_out_heading_) > pnc::mathlib::Deg2Rad(5.0) &&
      std::abs(arc_slot_init_out_heading_) < pnc::mathlib::Deg2Rad(45.0)) {
    is_arc_slot_ = true;
  }

  ILOG_INFO << "arc_slot_init_out_heading_ = " << arc_slot_init_out_heading_;
  // ILOG_INFO << "is_arc_slot_ = " << is_arc_slot_;
  ILOG_INFO << "side_sgn" << side_sgn;
  if (side_sgn < 0.0 && arc_slot_init_out_heading_ > pnc::mathlib::Deg2Rad(10.0) &&
      arc_slot_init_out_heading_ < pnc::mathlib::Deg2Rad(45.0)) {
    is_outer_arc_slot_ = true;
  }
  if (side_sgn > 0.0 && arc_slot_init_out_heading_ < - pnc::mathlib::Deg2Rad(10.0) &&
      arc_slot_init_out_heading_ > - pnc::mathlib::Deg2Rad(45.0)) {
    is_outer_arc_slot_ = true;
  }
  ILOG_INFO << "is_arc_slot_ = " << is_arc_slot_ << ", is_outer_arc_slot_ = " << is_outer_arc_slot_;
}


}  // namespace apa_planner
}  // namespace planning
