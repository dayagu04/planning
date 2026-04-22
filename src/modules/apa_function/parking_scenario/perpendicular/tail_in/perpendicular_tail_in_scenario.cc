#include "perpendicular_tail_in_scenario.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "apa_context.h"
#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_slot_manager.h"
#include "apa_state_machine_manager.h"
#include "collision_detection/collision_detection.h"
#include "collision_detection/gjk_collision_detector.h"
#include "debug_info_log.h"
#include "generate_obstacle_decider/generate_obstacle_decider.h"
#include "geometry_math.h"
#include "geometry_path_generator.h"
#include "ifly_time.h"
#include "initial_action_decider/initial_action_decider.h"
#include "log_glog.h"
#include "math_lib.h"
#include "parking_scenario.h"
#include "perpendicular_tail_in_path_generator.h"
#include "target_pose_decider/target_pose_decider.h"

namespace planning {
namespace apa_planner {

void PerpendicularTailInScenario::Reset() {
  current_plan_path_vec_.clear();
  all_plan_path_vec_.clear();
  ParkingScenario::Reset();
}

void PerpendicularTailInScenario::Clear() {
  current_plan_path_vec_.clear();
  all_plan_path_vec_.clear();
  ParkingScenario::Clear();
}

void PerpendicularTailInScenario::ScenarioTry() {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus()) {
    return;
  }

  Reset();
  frame_.replan_reason = ReplanReason::SLOT_CRUISING;

  SlotReleaseInfo& release_info = apa_world_ptr_->GetSlotManagerPtr()
                                      ->GetMutableEgoInfoUnderSlot()
                                      .slot.release_info_;
  auto& geometry_release_state =
      release_info.release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE];

  if (!UpdateEgoSlotInfo() || !GenTlane()) {
    geometry_release_state = SlotReleaseState::NOT_RELEASE;
    return;
  }

  const auto park_path_plan_type = apa_param.GetParam().park_path_plan_type;
  uint8_t path_plan_res = PathPlannerResult::PLAN_FAILED;
  switch (park_path_plan_type) {
    case ParkPathPlanType::GEOMETRY:
      path_plan_res = PathPlanOnce();
      break;
    case ParkPathPlanType::HYBRID_ASTAR:
      path_plan_res = PathPlanOnceHybridAstar();
      break;
    case ParkPathPlanType::HYBRID_ASTAR_THREAD:
      path_plan_res = PathPlanOnceHybridAstarThread();
      break;
    default:
      break;
  }

  TransformPreparePlanningTraj();
  // a redundant operation aimed at preparing for the first formal plan state
  Clear();

  switch (path_plan_res) {
    case PathPlannerResult::PLAN_UPDATE:
      geometry_release_state = SlotReleaseState::RELEASE;
      break;
    case PathPlannerResult::WAIT_PATH:
      geometry_release_state = SlotReleaseState::COMPUTING;
      break;
    case PathPlannerResult::PLAN_FAILED:
    default:
      geometry_release_state = SlotReleaseState::NOT_RELEASE;
      break;
  }

  if (park_path_plan_type != ParkPathPlanType::GEOMETRY) {
    release_info.release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
        geometry_release_state;
  }

  // fill head/tail in parking action.
  // TODO: actions need computing by algorithm.
  SetParkInFeasibleDirection();
}

void PerpendicularTailInScenario::ExcutePathPlanningTask() {
  InitSimulation();
  DecideExpandMirrorCommand();

  if (CheckPlanSkip()) {
    return;
  }

  if (!UpdateEgoSlotInfo()) {
    SetParkingStatus(PARKING_FAILED);
    return;
  }

  UpdateStuckTime();
  CalSlotJumpErr();
  UpdateRemainDist();
  DecideFoldMirrorCommand();

  if (CheckFinished()) {
    ILOG_INFO << "check apa finished!";
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

  if (CheckStuckFailed()) {
    SetParkingStatus(PARKING_FAILED);
    return;
  }

  RunPathPlanningByConfig();

  if (CheckFinished()) {
    ILOG_INFO << "check apa finished!";
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

  if (CheckFoldMirrorFailed()) {
    SetParkingStatus(PARKING_FAILED);
    return;
  }

  if (CheckGearChangeCountTooMuch(
          apa_param.GetParam().gear_change_decide_params)) {
    SetParkingStatus(PARKING_FAILED);
    return;
  }
}

void PerpendicularTailInScenario::RunPathPlanningByConfig() {
  const auto park_path_plan_type = apa_param.GetParam().park_path_plan_type;
  PrintParkPathPlanType(park_path_plan_type);
  PrintAnalyticExpansionType(apa_param.GetParam().analytic_expansion_type);
  if (park_path_plan_type == ParkPathPlanType::GEOMETRY ||
      park_path_plan_type == ParkPathPlanType::HYBRID_ASTAR) {
    PathPlan();
  } else if (park_path_plan_type == ParkPathPlanType::HYBRID_ASTAR_THREAD) {
    PathPlanByHybridAstarThread();
  }
}

const bool PerpendicularTailInScenario::UpdateEgoSlotInfo() {
  static const std::vector<double> kOccupiedRatioTab = {1.0, 0.0};
  const ApaParameters& param = apa_param.GetParam();
  const std::shared_ptr<ApaMeasureDataManager> measures_ptr =
      apa_world_ptr_->GetMeasureDataManagerPtr();
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  const auto& slot = ego_info_under_slot.slot;
  const auto& ego_pos = measures_ptr->GetPos();
  const auto ego_heading = measures_ptr->GetHeading();
  const auto& ego_heading_vec = measures_ptr->GetHeadingVec();
  const auto& g2l_tf = ego_info_under_slot.g2l_tf;

  const bool is_supported_slot = slot.slot_type_ == SlotType::SLANT ||
                                 slot.slot_type_ == SlotType::PERPENDICULAR;
  if (!is_supported_slot) {
    ILOG_INFO << "slot type is not supported, return false";
    frame_.plan_fail_reason = UPDATE_EGO_SLOT_INFO;
    return false;
  }

  auto& cur_pose = ego_info_under_slot.cur_pose;
  cur_pose.pos = g2l_tf.GetPos(ego_pos);
  cur_pose.heading = g2l_tf.GetHeading(ego_heading);
  cur_pose.heading_vec = geometry_lib::GenHeadingVec(cur_pose.heading);

  // only use for geometry path planner
  if (frame_.is_replan_first) {
    const Eigen::Vector2d ego_to_slot_center_vec =
        slot.origin_corner_coord_global_.pt_center - ego_pos;

    const double cross_ego_to_slot_center =
        pnc::geometry_lib::GetCrossFromTwoVec2d(ego_heading_vec,
                                                ego_to_slot_center_vec);

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(
            ego_heading_vec,
            ego_info_under_slot.origin_pose_global.heading_vec);

    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
      ego_info_under_slot.slot_side = geometry_lib::SLOT_SIDE_RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      ego_info_under_slot.slot_side = geometry_lib::SLOT_SIDE_LEFT;
    } else {
      ego_info_under_slot.slot_side = geometry_lib::SLOT_SIDE_INVALID;
    }
  }

  // no consider obs target pose, real-time update
  TargetPoseDeciderRequest tar_pose_decider_request(
      scenario_type_,
      apa_world_ptr_->GetStateMachineManagerPtr()->GetSlotLatPosPreference());

  TargetPoseDeciderResult res =
      apa_world_ptr_->GetParkingTaskInterfacePtr()
          ->GetTargetPoseDeciderPtr()
          ->CalcTargetPose(slot, tar_pose_decider_request);

  ego_info_under_slot.origin_target_pose = res.target_pose_local;

  const double origin_target_x = ego_info_under_slot.origin_target_pose.pos.x();
  const double half_slot_width = 0.5 * slot.slot_width_;
  ego_info_under_slot.virtual_limiter.first << origin_target_x, half_slot_width;
  ego_info_under_slot.virtual_limiter.second << origin_target_x,
      -half_slot_width;

  if (std::fabs(ego_info_under_slot.terminal_err.GetY()) <
          param.slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_info_under_slot.terminal_err.GetTheta()) <
          param.slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    double end_x = slot.slot_length_ + param.rear_overhanging;
    if (scenario_type_ == ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
      end_x = slot.slot_length_ + param.front_overhanging + param.wheel_base;
    }

    const double cur_x = cur_pose.GetX();
    const std::vector<double> x_tab = {origin_target_x, end_x};
    const std::vector<double> x_postprocess_tab = {
        ego_info_under_slot.target_pose.pos.x(), end_x};

    ego_info_under_slot.slot_occupied_ratio =
        mathlib::Interp1(x_tab, kOccupiedRatioTab, cur_x);

    ego_info_under_slot.slot_occupied_ratio_postprocess =
        mathlib::Interp1(x_postprocess_tab, kOccupiedRatioTab, cur_x);

  } else {
    ego_info_under_slot.slot_occupied_ratio = 0.0;
    ego_info_under_slot.slot_occupied_ratio_postprocess = 0.0;
  }

  PostProcessPathAccordingLimiter();

  // fix slot
  if (ego_info_under_slot.slot_occupied_ratio_postprocess >
          param.fix_slot_occupied_ratio &&
      !ego_info_under_slot.fix_slot && measures_ptr->GetStaticFlag()) {
    ego_info_under_slot.fix_slot = true;
  }

  return true;
}

const bool PerpendicularTailInScenario::CheckCanDelObsInSlot() {
  return apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag() &&
         !CheckEgoPoseInBelieveSlotArea(
             0.2, apa_param.GetParam().believe_obs_ego_area, 60.0,
             scenario_type_ ==
                 ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN);
}

const bool PerpendicularTailInScenario::CalcPtInside() {
  // construct tlane pq
  // left y is positive, right y is negative
  // left y should be smallest, right y should be largest
  // all x should be largest
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      left_pq_for_y(geometry_lib::Compare(3));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      left_pq_for_x(geometry_lib::Compare(0));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      right_pq_for_y(geometry_lib::Compare(2));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      geometry_lib::Compare>
      right_pq_for_x(geometry_lib::Compare(0));

  const auto& param = apa_param.GetParam();

  const double mir_width =
      (param.max_car_width - param.car_width) * 0.5 - 0.0168;

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  const double mir_x = ego_info_under_slot.target_pose.pos.x() +
                       param.lon_dist_mirror_to_rear_axle - 0.368;

  const auto& obstacles =
      apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles();

  Eigen::Vector2d obs_pt_slot;
  for (const auto& pair : obstacles) {
    for (const auto& obs : pair.second.GetPtClout2dLocal()) {
      obs_pt_slot = obs;
      SlotObsType obs_slot_type = CalSlotObsType(obs_pt_slot);
      if (obs_slot_type != SlotObsType::INSIDE_OBS &&
          obs_slot_type != SlotObsType::OUTSIDE_OBS) {
        continue;
      }

      // the obs lower mir can relax lat requirements
      if (obs_pt_slot.x() < mir_x) {
        if (obs_pt_slot.y() > 1e-6) {
          obs_pt_slot.y() += mir_width;
        } else {
          obs_pt_slot.y() -= mir_width;
        }
      }

      // the obs far from slot can relax lon equirements
      if (std::fabs(obs_pt_slot.y()) >
          ego_info_under_slot.slot.slot_width_ * 0.5 + 0.468) {
        obs_pt_slot.x() -= 0.268;
      }

      if (obs_pt_slot.y() > 1e-6) {
        left_pq_for_y.emplace(std::move(obs_pt_slot));
        left_pq_for_x.emplace(std::move(obs_pt_slot));
      } else {
        right_pq_for_y.emplace(std::move(obs_pt_slot));
        right_pq_for_x.emplace(std::move(obs_pt_slot));
      }
    }
  }

  apa_param.SetParam().actual_mono_plan_enable = param.mono_plan_enable;
  frame_.is_left_empty = left_pq_for_x.empty();
  frame_.is_right_empty = right_pq_for_x.empty();
  if (param.conservative_mono_enable &&
      (!frame_.is_left_empty || !frame_.is_right_empty)) {
    apa_param.SetParam().actual_mono_plan_enable = false;
  }

  const Eigen::Vector2d pt_01_unit_vec =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_01_unit_vec;

  const Eigen::Vector2d pt_01_mid =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_01_mid;

  double half_origin_slot_width =
      ego_info_under_slot.slot.origin_corner_coord_local_.pt_01_vec.norm() *
      0.5;
  half_origin_slot_width =
      std::max(half_origin_slot_width, param.max_car_width * 0.5 + 0.168);

  const Eigen::Vector2d virtual_left_obs =
      pt_01_mid -
      param.virtual_obs_left_x_pos *
          ego_info_under_slot.origin_pose_local.heading_vec +
      (half_origin_slot_width + param.virtual_obs_left_y_pos) * pt_01_unit_vec;

  const Eigen::Vector2d virtual_right_obs =
      pt_01_mid -
      param.virtual_obs_right_x_pos *
          ego_info_under_slot.origin_pose_local.heading_vec -
      (half_origin_slot_width + param.virtual_obs_right_y_pos) * pt_01_unit_vec;

  left_pq_for_y.emplace(virtual_left_obs);
  left_pq_for_x.emplace(virtual_left_obs);
  right_pq_for_y.emplace(virtual_right_obs);
  right_pq_for_x.emplace(virtual_right_obs);

  const Eigen::Vector2d left_obs(left_pq_for_x.top().x(),
                                 left_pq_for_y.top().y());
  const Eigen::Vector2d right_obs(right_pq_for_x.top().x(),
                                  right_pq_for_y.top().y());

  if (ego_info_under_slot.slot_side == geometry_lib::SLOT_SIDE_RIGHT) {
    ego_info_under_slot.pt_inside = right_obs;
    ego_info_under_slot.pt_inside.x() =
        mathlib::Constrain(
            ego_info_under_slot.pt_inside.x(),
            ego_info_under_slot.slot.origin_corner_coord_local_.pt_0.x() -
                2.168,
            ego_info_under_slot.slot.origin_corner_coord_local_.pt_0.x() +
                2.68) +
        param.tlane_safe_dx;

    ego_info_under_slot.pt_inside.y() =
        mathlib::Constrain(
            ego_info_under_slot.pt_inside.y(),
            -0.5 * ego_info_under_slot.slot.slot_width_ - 0.128,
            -0.5 * ego_info_under_slot.slot.slot_width_ + 0.068) +
        0.0268;
  } else {
    ego_info_under_slot.pt_inside = left_obs;
    ego_info_under_slot.pt_inside.x() =
        mathlib::Constrain(
            ego_info_under_slot.pt_inside.x(),
            ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.x() -
                2.168,
            ego_info_under_slot.slot.origin_corner_coord_local_.pt_1.x() +
                2.68) +
        param.tlane_safe_dx;

    ego_info_under_slot.pt_inside.y() =
        mathlib::Constrain(ego_info_under_slot.pt_inside.y(),
                           0.5 * ego_info_under_slot.slot.slot_width_ - 0.068,
                           0.5 * ego_info_under_slot.slot.slot_width_ + 0.128) -
        0.0268;
  }

  return true;
}

const bool PerpendicularTailInScenario::GenTlane() {
  // if safe target pose exist, return true, otherwise return false
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  const ApaParameters& param = apa_param.GetParam();
  const auto measure_data_manager_ptr =
      apa_world_ptr_->GetMeasureDataManagerPtr();
  const auto state_machine_manager_ptr =
      apa_world_ptr_->GetStateMachineManagerPtr();
  const auto parking_task_interface_ptr =
      apa_world_ptr_->GetParkingTaskInterfacePtr();
  const auto col_det_interface_ptr = apa_world_ptr_->GetColDetInterfacePtr();
  const auto target_pose_decider_ptr =
      parking_task_interface_ptr->GetTargetPoseDeciderPtr();
  const auto generate_obstacle_decider_ptr =
      parking_task_interface_ptr->GetGenerateObstacleDeciderPtr();

  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  generate_obstacle_decider_ptr->GenObs(
      ego_info_under_slot,
      GenerateObstacleRequest(scenario_type_, frame_.process_obs_method));

  if (param.park_path_plan_type == ParkPathPlanType::GEOMETRY) {
    CalcPtInside();
  }

  const bool is_dynamic_replan_with_high_occupied_ratio =
      frame_.replan_reason == ReplanReason::DYNAMIC &&
      ego_info_under_slot.slot_occupied_ratio > 0.6;
  const bool update_slot_move_dist =
      frame_.replan_reason != ReplanReason::NOT_REPLAN &&
      !is_dynamic_replan_with_high_occupied_ratio;

  ILOG_INFO << "  update_slot_move_dist = " << update_slot_move_dist
            << "  process_obs_method = "
            << static_cast<int>(frame_.process_obs_method);

  if (update_slot_move_dist) {
    const bool has_fold_mirror = measure_data_manager_ptr->GetFoldMirrorFlag();
    const bool has_smart_fold_mirror =
        param.smart_fold_mirror_params.has_smart_fold_mirror;
    const bool is_searching = state_machine_manager_ptr->IsSeachingStatus();
    const bool is_parking_status = state_machine_manager_ptr->IsParkingStatus();
    const bool need_try_fold_mirror_only =
        has_fold_mirror || (has_smart_fold_mirror && is_searching);
    const bool need_try_unfold_then_fold =
        !need_try_fold_mirror_only && has_smart_fold_mirror;

    const ParkingLatLonTargetPoseBuffer& target_pose_buffer =
        param.lat_lon_target_pose_buffer;
    const double min_lat_body_buf = target_pose_buffer.min_lat_body_buffer;
    const double min_lat_mirror_buf = target_pose_buffer.min_lat_mirror_buffer;
    const int buf_size = std::max(target_pose_buffer.buf_size, 1);
    const double lon_buffer = target_pose_buffer.lon_buffer;
    const auto slot_lat_pos_preference =
        state_machine_manager_ptr->GetSlotLatPosPreference();

    bool find_target_pose = false;
    const double start_time = IflyTime::Now_ms();

    const auto try_find_target_pose = [&](const bool fold_mirror_flag) {
      col_det_interface_ptr->Init(fold_mirror_flag);

      double max_lat_body_buf = target_pose_buffer.max_lat_body_buffer;
      double max_lat_mirror_buf = target_pose_buffer.max_lat_mirror_buffer;
      if (is_parking_status &&
          frame_.replan_reason != ReplanReason::FIRST_PLAN &&
          frame_.replan_reason != ReplanReason::FORCE_PLAN) {
        max_lat_body_buf = std::min(max_lat_body_buf,
                                    ego_info_under_slot.safe_lat_body_buffer);
        max_lat_mirror_buf = std::min(
            max_lat_mirror_buf, ego_info_under_slot.safe_lat_mirror_buffer);
      }

      const double body_buf_step =
          std::max((max_lat_body_buf - min_lat_body_buf) / buf_size, 5e-3);
      const double mirror_buf_step =
          std::max((max_lat_mirror_buf - min_lat_mirror_buf) / buf_size, 5e-3);

      max_lat_body_buf =
          std::max(max_lat_body_buf, min_lat_body_buf + body_buf_step + 1e-3);
      max_lat_mirror_buf = std::max(
          max_lat_mirror_buf, min_lat_mirror_buf + mirror_buf_step + 1e-3);

      ILOG_INFO << "find target pose max lat body buffer = " << max_lat_body_buf
                << " max lat mirror buffer = " << max_lat_mirror_buf;

      std::vector<double> lat_body_buffer_vec;
      std::vector<double> lat_mirror_buffer_vec;
      lat_body_buffer_vec.reserve(buf_size + 1);
      lat_mirror_buffer_vec.reserve(buf_size + 1);
      for (double lat_body_buf = max_lat_body_buf,
                  lat_mirror_buf = max_lat_mirror_buf;
           lat_body_buf > min_lat_body_buf - 1e-3 &&
           lat_mirror_buf > min_lat_mirror_buf - 1e-3;
           lat_body_buf -= body_buf_step, lat_mirror_buf -= mirror_buf_step) {
        lat_body_buffer_vec.emplace_back(lat_body_buf);
        lat_mirror_buffer_vec.emplace_back(lat_mirror_buf);
      }

      CheckEgoPoseInBelieveSlotArea(
          0.2, param.believe_obs_ego_area, 60.0,
          scenario_type_ ==
              ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN);

      const TargetPoseDeciderRequest tar_pose_decider_request(
          lat_body_buffer_vec, lat_mirror_buffer_vec, lon_buffer,
          scenario_type_, true, true, slot_lat_pos_preference, false,
          CheckEgoPoseInBelieveSlotArea(
              0.2, param.believe_obs_ego_area, 60.0,
              scenario_type_ ==
                  ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN));

      const TargetPoseDeciderResult res =
          target_pose_decider_ptr->CalcTargetPose(ego_info_under_slot.slot,
                                                  tar_pose_decider_request);
      if (res.target_pose_type == TargetPoseType::FAIL) {
        return false;
      }

      ego_info_under_slot.target_pose = res.target_pose_local;
      if (frame_.replan_reason != ReplanReason::DYNAMIC) {
        ego_info_under_slot.safe_lat_body_buffer = res.safe_lat_body_buffer;
        ego_info_under_slot.safe_lat_mirror_buffer = res.safe_lat_mirror_buffer;
      }

      ego_info_under_slot.tar_pose_result = res;
      ego_info_under_slot.lon_move_dist_every_replan = res.safe_lon_move_dist;
      ego_info_under_slot.lat_move_dist_every_replan = res.safe_lat_move_dist;
      return true;
    };

    if (need_try_fold_mirror_only) {
      find_target_pose = try_find_target_pose(true);
    } else {
      find_target_pose = try_find_target_pose(false);
      if (!find_target_pose && need_try_unfold_then_fold) {
        find_target_pose = try_find_target_pose(true);
      }
    }

    ILOG_INFO << "find target pose time cost = "
              << (IflyTime::Now_ms() - start_time);

    if (!find_target_pose) {
      ILOG_ERROR << "can not find target pose";
      return false;
    }
  } else {
    ego_info_under_slot.target_pose = ego_info_under_slot.origin_target_pose;
    ego_info_under_slot.target_pose.pos.x() +=
        ego_info_under_slot.lon_move_dist_replan_success;
    ego_info_under_slot.target_pose.pos.y() +=
        ego_info_under_slot.lat_move_dist_replan_success;
  }

  ego_info_under_slot.tar_line =
      geometry_lib::BuildLineSegByPose(ego_info_under_slot.target_pose.pos,
                                       ego_info_under_slot.target_pose.heading);

  col_det_interface_ptr->GetEDTColDetPtr()->UpdateObsClearZone(
      std::vector<Eigen::Vector2d>{ego_info_under_slot.cur_pose.pos,
                                   ego_info_under_slot.target_pose.pos});

  ILOG_INFO
      << "cur pose = " << ego_info_under_slot.cur_pose.pos.transpose() << "  "
      << ego_info_under_slot.cur_pose.heading * kRad2Deg
      << "  origin_tar_pose = "
      << ego_info_under_slot.origin_target_pose.pos.transpose() << "  "
      << ego_info_under_slot.origin_target_pose.heading * kRad2Deg
      << "  tar_pose = " << ego_info_under_slot.target_pose.pos.transpose()
      << "  " << ego_info_under_slot.target_pose.heading * kRad2Deg
      << "  terminal_err = " << ego_info_under_slot.terminal_err.pos.transpose()
      << "  " << ego_info_under_slot.terminal_err.heading * kRad2Deg
      << "  terminal_y_front_err = " << ego_info_under_slot.terminal_y_front_err
      << "  slot_occupied_ratio_postprocess = "
      << ego_info_under_slot.slot_occupied_ratio_postprocess
      << "  slot occupied ratio = " << ego_info_under_slot.slot_occupied_ratio
      << "  pt_inside = " << ego_info_under_slot.pt_inside.transpose()
      << "  stuck time(s) = " << frame_.stuck_time
      << "  stuck_obs_time(s) = " << frame_.stuck_obs_time
      << "  stuck_dynamic_obs_time(s) = " << frame_.stuck_dynamic_obs_time
      << "  stuck_by_dynamic_obs = " << frame_.stuck_by_dynamic_obs << "  "
      << "  slot side = "
      << geometry_lib::GetSlotSideString(ego_info_under_slot.slot_side);

  return true;
}

const bool PerpendicularTailInScenario::GenObstacles() { return true; }

const uint8_t PerpendicularTailInScenario::PathPlanOnce() {
  if (scenario_type_ == ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    ILOG_INFO << "PathPlanOnce not support perpendicular head in";
    frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
    return PathPlannerResult::PLAN_FAILED;
  }

  // --- cache pointers and common references ---
  const auto slot_manager_ptr = apa_world_ptr_->GetSlotManagerPtr();
  const auto col_det_interface_ptr = apa_world_ptr_->GetColDetInterfacePtr();
  const auto measure_data_manager_ptr =
      apa_world_ptr_->GetMeasureDataManagerPtr();
  const auto parking_task_interface_ptr =
      apa_world_ptr_->GetParkingTaskInterfacePtr();
  const auto& per_path_planner_ptr =
      parking_task_interface_ptr->GetPerpendicularTailInPathGeneratorPtr();

  const ApaParameters& param = apa_param.GetParam();
  const EgoInfoUnderSlot& ego_info_under_slot =
      slot_manager_ptr->GetEgoInfoUnderSlot();
  const SimulationParam& simu_param = apa_world_ptr_->GetSimuParam();

  // --- named booleans for readability ---
  const bool is_dynamic_replan =
      (frame_.replan_reason == ReplanReason::DYNAMIC);
  const bool is_searching_stage =
      apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus();
  const bool has_fold_mirror = measure_data_manager_ptr->GetFoldMirrorFlag();

  // --- build input ---
  GeometryPathInput input;
  input.ego_info_under_slot = ego_info_under_slot;
  input.is_complete_path = simu_param.is_complete_path;
  input.sample_ds = simu_param.sample_ds;
  input.ref_gear = frame_.current_gear;
  input.ref_arc_steer = frame_.current_arc_steer;
  input.is_replan_first = frame_.is_replan_first;
  input.is_replan_second = frame_.is_replan_second;
  input.is_replan_dynamic = is_dynamic_replan;
  input.is_searching_stage = is_searching_stage;
  input.force_mid_process_plan = simu_param.force_mid_process_plan;
  input.can_first_plan_again = frame_.can_first_plan_again;
  input.is_simulation = simu_param.is_simulation;
  input.is_left_empty = frame_.is_left_empty;
  input.is_right_empty = frame_.is_right_empty;

  if (simu_param.is_simulation && simu_param.ref_gear != 0) {
    input.ref_gear = simu_param.ref_gear;
  }

  // collision detection init & smart fold mirror
  if (has_fold_mirror) {
    col_det_interface_ptr->Init(true);
  } else {
    col_det_interface_ptr->Init(false);
    if (ego_info_under_slot.tar_pose_result.target_pose_type ==
        TargetPoseType::FOLD_MIRROR) {
      ILOG_INFO << "try path plan with fold mirror";
      input.enable_smart_fold_mirror = true;
    }
  }

  if (input.is_simulation) {
    apa_param.SetParam().use_average_obs_dist = simu_param.use_average_obs_dist;
  }

  if (is_dynamic_replan) {
    ILOG_INFO << "dynamic replan, gear should be reverse";
    input.ref_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
  }

  // --- splicing points for dynamic plan ---
  std::vector<geometry_lib::PathPoint> splicing_point_global_vec;
  CalcProjPtForDynamicPlan(input.ego_info_under_slot.cur_pose,
                           splicing_point_global_vec);

  // --- first path plan attempt ---
  per_path_planner_ptr->SetInput(input);
  const bool path_plan_success = per_path_planner_ptr->Update();

  // searching stage: fail immediately if plan fails
  if (is_searching_stage && !path_plan_success) {
    return PathPlannerResult::PLAN_FAILED;
  }

  // --- handle plan result by replan type ---
  if (is_dynamic_replan) {
    if (!path_plan_success) {
      frame_.dynamic_plan_fail_flag = true;
      ILOG_INFO << "path dynamic plan fail";
      frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
      return PathPlannerResult::PLAN_UPDATE;
    }
    ILOG_INFO << "path dynamic plan success";
    frame_.dynamic_plan_fail_flag = false;
    if (!CheckDynamicPlanPathOptimalByGeometryPath()) {
      frame_.dynamic_plan_path_superior = false;
      frame_.plan_fail_reason = ParkingFailReason::DYNAMIC_PATH_NOT_SUPERIOR;
      ILOG_INFO << "path dynamic plan is not superior, should not replace "
                   "last path";
      return PathPlannerResult::PLAN_UPDATE;
    }
    ILOG_INFO << "path dynamic plan is superior, should replace last path";
    frame_.dynamic_plan_path_superior = true;
  } else if (path_plan_success) {
    // static replan success
    ILOG_INFO << "path plan success";
    const bool should_reject_gear_change =
        frame_.process_obs_method == ProcessObsMethod::DO_NOTHING &&
        !frame_.is_replan_first &&
        per_path_planner_ptr->GetOutput().current_gear != frame_.current_gear &&
        CheckCanDelObsInSlot();
    if (should_reject_gear_change) {
      ILOG_INFO << "when process_obs_method is do nothing and no first "
                   "replan and can del obs in slot, plan gear should be same "
                   "with ref gear, otherwise fail";
      frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
      return PathPlannerResult::PLAN_FAILED;
    }
  } else {
    // static replan fail — try first plan again if allowed
    ILOG_INFO << "path plan fail";
    const bool cannot_retry =
        frame_.process_obs_method == ProcessObsMethod::DO_NOTHING ||
        frame_.process_obs_method == ProcessObsMethod::MOVE_OBS_OUT_SLOT ||
        !frame_.can_first_plan_again || frame_.is_replan_first;
    if (cannot_retry) {
      ILOG_INFO << "no try first path plan, directly fail";
      frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
      return PathPlannerResult::PLAN_FAILED;
    }

    ILOG_INFO << "try first path plan again";
    frame_.is_replan_first = true;
    frame_.is_replan_second = false;
    frame_.can_first_plan_again = false;
    input.is_replan_first = true;
    input.is_replan_second = false;
    input.can_first_plan_again = false;
    per_path_planner_ptr->SetInput(input);
    if (!per_path_planner_ptr->Update()) {
      ILOG_INFO << "try first path plan again also fail";
      frame_.plan_fail_reason = PATH_PLAN_FAILED;
      return PathPlannerResult::PLAN_FAILED;
    }
    // it can limit gear, but now not limit, try success possible
    if (per_path_planner_ptr->GetOutput().current_gear != frame_.current_gear) {
      // return PathPlannerResult::PLAN_FAILED;
    }
    ILOG_INFO << "try first path plan again success";
  }

  // --- post-plan validation ---
  if (!per_path_planner_ptr->SetCurrentPathSegIndex()) {
    ILOG_INFO << "path plan fail";
    frame_.plan_fail_reason = ParkingFailReason::SET_SEG_INDEX;
    return PathPlannerResult::PLAN_FAILED;
  }
  if (!per_path_planner_ptr->CheckCurrentGearLength()) {
    ILOG_INFO << "path plan fail";
    frame_.plan_fail_reason = ParkingFailReason::CHECK_GEAR_LENGTH;
    return PathPlannerResult::PLAN_FAILED;
  }

  per_path_planner_ptr->SampleCurrentPathSeg();
  per_path_planner_ptr->PrintOutputSegmentsInfo();

  // --- build output path segments ---
  const GeometryPathOutput& planner_output = per_path_planner_ptr->GetOutput();
  current_plan_path_vec_.clear();
  current_plan_path_vec_.reserve(5);
  all_plan_path_vec_.clear();
  all_plan_path_vec_.reserve(8);
  frame_.is_last_path = planner_output.is_last_path;

  for (size_t i = planner_output.path_seg_index.first;
       i < planner_output.path_segment_vec.size(); ++i) {
    geometry_lib::PathSegment path_seg_global =
        planner_output.path_segment_vec[i];
    path_seg_global.LocalToGlobal(ego_info_under_slot.l2g_tf);
    all_plan_path_vec_.emplace_back(path_seg_global);
    if (i <= planner_output.path_seg_index.second) {
      current_plan_path_vec_.emplace_back(path_seg_global);
    }
  }

  // --- update frame state ---
  frame_.current_gear = geometry_lib::ReverseGear(planner_output.current_gear);
  if (frame_.is_replan_first) {
    frame_.is_replan_first = false;
    frame_.is_replan_second = true;
  } else if (frame_.is_replan_second) {
    frame_.is_replan_second = false;
  }
  frame_.gear_command = planner_output.current_gear;
  frame_.cur_path_gear_change_count = planner_output.gear_change_count;

  // --- build output path points (local -> global) ---
  const auto& l2g_tf = ego_info_under_slot.l2g_tf;

  // helper: convert local path point to global
  const auto to_global = [&l2g_tf](const geometry_lib::PathPoint& local_pt,
                                   geometry_lib::PathPoint& out) {
    out.Set(l2g_tf.GetPos(local_pt.pos), l2g_tf.GetHeading(local_pt.heading));
    out.lat_buffer = local_pt.lat_buffer;
    out.s = local_pt.s;
    out.kappa = local_pt.kappa;
  };

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(planner_output.path_point_vec.size() +
                                         splicing_point_global_vec.size() + 18);
  complete_path_point_global_vec_.clear();
  complete_path_point_global_vec_.reserve(
      planner_output.all_gear_path_point_vec.size() +
      splicing_point_global_vec.size() + 18);

  std::vector<pnc::geometry_lib::PathPoint> path_pt_vec;
  if (LateralPathOptimize(path_pt_vec)) {
    JSON_DEBUG_VALUE("is_path_lateral_optimized", true);
  } else {
    path_pt_vec = planner_output.path_point_vec;
    JSON_DEBUG_VALUE("is_path_lateral_optimized", false);
  }

  // splicing points (already in global frame)
  for (const geometry_lib::PathPoint& pt : splicing_point_global_vec) {
    current_path_point_global_vec_.emplace_back(pt);
    complete_path_point_global_vec_.emplace_back(pt);
  }

  // current path points: local -> global
  geometry_lib::PathPoint global_point;
  for (const geometry_lib::PathPoint& path_point : path_pt_vec) {
    to_global(path_point, global_point);
    current_path_point_global_vec_.emplace_back(global_point);
  }

  // complete path points: local -> global (includes gear)
  for (const auto& path_point : planner_output.all_gear_path_point_vec) {
    to_global(path_point, global_point);
    global_point.gear = path_point.gear;
    complete_path_point_global_vec_.emplace_back(global_point);
  }

  // --- correct target pose ---
  if (!complete_path_point_global_vec_.empty()) {
    EgoInfoUnderSlot& mutable_ego_info_under_slot =
        slot_manager_ptr->GetMutableEgoInfoUnderSlot();
    geometry_lib::PathPoint real_target_pose =
        complete_path_point_global_vec_.back();
    real_target_pose.GlobalToLocal(mutable_ego_info_under_slot.g2l_tf);
    const geometry_lib::PathPoint& origin_target_pose =
        mutable_ego_info_under_slot.origin_target_pose;
    mutable_ego_info_under_slot.lat_move_dist_replan_success =
        real_target_pose.GetY() - origin_target_pose.GetY();
    mutable_ego_info_under_slot.lon_move_dist_replan_success =
        real_target_pose.GetX() - origin_target_pose.GetX();
    mutable_ego_info_under_slot.target_pose = real_target_pose;
  }

  perferred_geometry_path_vec_ = planner_output.perferred_geometry_path_vec;

  ILOG_INFO << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size()
            << "  complete_path_point_global_vec_.size() = "
            << complete_path_point_global_vec_.size();

  return PathPlannerResult::PLAN_UPDATE;
}

void PerpendicularTailInScenario::PathPlan() {
  const ApaParameters& param = apa_param.GetParam();
  CheckReplanParams replan_params;
  replan_params.use_obs_height_method = param.use_obs_height_method;
  frame_.replan_flag = CheckReplan(replan_params);
  frame_.pathplan_result = PathPlannerResult::PLAN_UPDATE;
  frame_.plan_fail_reason = ParkingFailReason::NOT_FAILED;

  if (!CheckCanDelObsInSlot()) {
    frame_.process_obs_method = ProcessObsMethod::DO_NOTHING;
  }

  const auto& simu_param = apa_world_ptr_->GetSimuParam();

  if (simu_param.is_simulation && simu_param.process_obs_method != -1) {
    frame_.process_obs_method =
        static_cast<ProcessObsMethod>(simu_param.process_obs_method);
  }

  if (!frame_.replan_flag) {
    ILOG_INFO << "replan is not required!";
    SetParkingStatus(ParkingStatus::PARKING_RUNNING);
    return;
  }

  SetParkingStatus(ParkingStatus::PARKING_PLANNING);

  const bool should_mark_parking_failed =
      frame_.replan_fail_time > param.max_replan_failed_time;

  const bool exist_target_pose = GenTlane();
  if (!exist_target_pose) {
    frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = ParkingFailReason::NO_TARGET_POSE;
    if (should_mark_parking_failed) {
      SetParkingStatus(ParkingStatus::PARKING_FAILED);
    }
    return;
  }

  ILOG_INFO << "target pose exists and replan is required!";
  const double start_time = IflyTime::Now_ms();
  frame_.pathplan_result =
      param.park_path_plan_type == ParkPathPlanType::GEOMETRY
          ? PathPlanOnce()
          : PathPlanOnceHybridAstar();
  const double replan_consume_time = IflyTime::Now_ms() - start_time;

  ILOG_INFO << "try generate path, and replan_consume_time = "
            << replan_consume_time << " ms, "
            << "  dynamic_plan_fail_flag = " << frame_.dynamic_plan_fail_flag
            << "  dynamic_plan_path_superior = "
            << frame_.dynamic_plan_path_superior;

  JSON_DEBUG_VALUE("dynamic_plan_fail_flag", frame_.dynamic_plan_fail_flag)
  JSON_DEBUG_VALUE("dynamic_plan_path_superior",
                   frame_.dynamic_plan_path_superior)
  JSON_DEBUG_VALUE("process_obs_method",
                   static_cast<int>(frame_.process_obs_method))

  TimeBenchmark::Instance().SetTime(TimeBenchmarkType::TB_APA_PATH_PLAN_TIME,
                                    replan_consume_time);

  const bool dynamic_plan_failed =
      frame_.replan_reason == ReplanReason::DYNAMIC &&
      (frame_.dynamic_plan_fail_flag || !frame_.dynamic_plan_path_superior);

  if (dynamic_plan_failed) {
    ILOG_INFO << "dynamic replan failed or path is not superior, use last path";
    return;
  }

  if (frame_.pathplan_result == PathPlannerResult::PLAN_FAILED) {
    ILOG_INFO << "static replan fail";
    if (should_mark_parking_failed) {
      SetParkingStatus(PARKING_FAILED);
    }
    SwitchProcessObsMethod();
    frame_.plan_fail_reason = PATH_PLAN_FAILED;
    return;
  }

  if (frame_.pathplan_result != PathPlannerResult::PLAN_UPDATE) {
    return;
  }

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  // path plan success
  ILOG_INFO << "path plan success, update path, record success info";
  ego_info_under_slot.lat_move_dist_replan_success =
      ego_info_under_slot.lat_move_dist_every_replan;
  ego_info_under_slot.lon_move_dist_replan_success =
      ego_info_under_slot.lon_move_dist_every_replan;
  ego_info_under_slot.replan_success_origin_target_pose =
      ego_info_under_slot.origin_target_pose;
  ego_info_under_slot.replan_success_origin_target_pose.LocalToGlobal(
      ego_info_under_slot.l2g_tf);

  frame_.process_obs_method = ProcessObsMethod::DO_NOTHING;

  if (frame_.replan_reason == ReplanReason::DYNAMIC) {
    ILOG_INFO << "dynamic replan success and path is superior, update path";
  } else {
    ILOG_INFO << "static replan success, update path";
    ego_info_under_slot.fix_limiter = false;
  }

  if (PostProcessPath()) {
    ILOG_INFO << "postprocess path success!";
    return;
  }
  ILOG_INFO << "postprocess path failed!";
  frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
  if (should_mark_parking_failed) {
    SetParkingStatus(PARKING_FAILED);
  }

  return;
}

void PerpendicularTailInScenario::GenHybridAstarConfigAndRequest(
    PlannerOpenSpaceConfig& config, HybridAStarRequest& request) {
  const ApaParameters& param = apa_param.GetParam();
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  const auto measure_data_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();
  const auto col_det_interface_ptr = apa_world_ptr_->GetColDetInterfacePtr();
  const auto& simu_param = apa_world_ptr_->GetSimuParam();
  const bool is_tail_in =
      scenario_type_ == ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN;

  // gen config
  config.InitConfig();
  // targeted customization parameters
  config.traj_kappa_change_penalty = param.traj_kappa_change_penalty;
  config.exceed_interseting_area_penalty = 1.2f * config.gear_switch_penalty;
  config.borrow_slot_penalty = 3.68f;
  config.expect_steer_penalty = 1.68f;
  config.exceed_cul_de_sac_limit_pos_penalty =
      0.68f * config.gear_switch_penalty;

  // gen request
  const bool mirror_has_folded = measure_data_ptr->GetFoldMirrorFlag();
  const bool should_try_fold_mirror =
      !mirror_has_folded &&
      ego_info_under_slot.tar_pose_result.target_pose_type ==
          TargetPoseType::FOLD_MIRROR;
  col_det_interface_ptr->Init(mirror_has_folded);
  if (should_try_fold_mirror) {
    ILOG_INFO << "try planning path with fold mirror";
  }
  request.mirror_has_folded_flag = mirror_has_folded;
  request.enable_smart_fold_mirror = should_try_fold_mirror;
  request.pre_search_mode = param.pre_search_mode;
  request.decide_cul_de_sac = param.enable_decide_cul_de_sac;
  request.enable_interesting_search_area = param.enable_interesting_search_area;
  request.process_obs_method = frame_.process_obs_method;
  request.sample_ds = config.node_path_dist_resolution;
  request.replan_reason = frame_.replan_reason;
  request.every_gear_length = 0.3f;
  request.swap_start_goal = false;
  request.scenario_type = scenario_type_;
  request.analytic_expansion_type = param.analytic_expansion_type;
  request.ego_info_under_slot = ego_info_under_slot;
  request.ref_solve_number = simu_param.ref_solve_number;
  request.max_scurve_number = 2;

  AstarPathGear ref_gear = GetAstarGearFromSegGear(frame_.current_gear);
  AstarPathSteer ref_steer =
      GetAstarSteerFromSegSteer(frame_.current_arc_steer);
  if (simu_param.ref_gear != 0) {
    ref_gear = GetAstarGearFromSegGear(simu_param.ref_gear);
  }
  if (simu_param.ref_steer != 0) {
    ref_steer = GetAstarSteerFromSegSteer(simu_param.ref_steer);
  }

  int max_gear_shift_number = 0;
  if (request.replan_reason == ReplanReason::SLOT_CRUISING) {
    max_gear_shift_number =
        param.gear_change_decide_params.all_max_gear_change_count_searching;
  } else if (request.replan_reason == ReplanReason::DYNAMIC) {
    max_gear_shift_number = 0;
    ref_gear = is_tail_in ? AstarPathGear::REVERSE : AstarPathGear::DRIVE;
  } else {
    max_gear_shift_number =
        param.gear_change_decide_params.all_max_gear_change_count_parking;
  }

  request.inital_action_request.ref_length = config.node_step + 0.01f;
  request.inital_action_request.ref_gear = ref_gear;
  request.inital_action_request.ref_steer = ref_steer;
  request.max_gear_shift_number = max_gear_shift_number;
  InitalActionDecider inital_action_decider(col_det_interface_ptr);
  inital_action_decider.Process(request.inital_action_request,
                                ego_info_under_slot.cur_pose,
                                ego_info_under_slot.tar_line, config.node_step,
                                config.node_path_dist_resolution, 3.0);

  const AstarPathGear adjust_pose_ref_gear =
      is_tail_in ? AstarPathGear::DRIVE : AstarPathGear::REVERSE;
  const double adjust_pose_front_offset =
      is_tail_in ? 2.0 : 2.0 + param.wheel_base;
  const double adjust_pose_rear_offset =
      is_tail_in ? 1.0 : 1.0 - param.wheel_base;
  request.adjust_pose =
      adjust_pose_ref_gear == ref_gear &&
      std::fabs(ego_info_under_slot.terminal_err.GetTheta()) * kRad2Deg <
          20.0 &&
      ego_info_under_slot.slot.IsPointInCustomSlot(
          ego_info_under_slot.cur_pose.pos, adjust_pose_front_offset,
          adjust_pose_rear_offset, -0.4, -0.4, true);

  ILOG_INFO << "hybrid_ref_gear = "
            << PathGearDebugString(request.inital_action_request.ref_gear)
            << " ref steer = "
            << GetPathSteerDebugString(request.inital_action_request.ref_steer)
            << " ref length = " << request.inital_action_request.ref_length
            << "  max_gear_shift_number = " << request.max_gear_shift_number
            << "  adjust_pose = " << request.adjust_pose
            << "  ref_solve_number = " << request.ref_solve_number;
}

const uint8_t PerpendicularTailInScenario::PathPlanOnceHybridAstar() {
  HybridAStarRequest request;
  PlannerOpenSpaceConfig config;
  GenHybridAstarConfigAndRequest(config, request);

  CalcProjPtForDynamicPlan(request.ego_info_under_slot.cur_pose,
                           request.splicing_pt_vec);

  const auto hybrid_astar_path_generator_ptr =
      apa_world_ptr_->GetParkingTaskInterfacePtr()
          ->GetHybridAstarPathGeneratorInterfacePtr();

  HybridAStarResult hybrid_astar_result;
  hybrid_astar_path_generator_ptr->SetRequest(request);
  hybrid_astar_path_generator_ptr->UpdateConfig(config);
  hybrid_astar_path_generator_ptr->Update();
  hybrid_astar_path_generator_ptr->GetResult(hybrid_astar_result);

  const bool path_plan_success = hybrid_astar_result.path_plan_success;
  ILOG_INFO << "hybrid astar plan success = " << path_plan_success << "  "
            << "  path_plan_consume_time_ms = "
            << hybrid_astar_result.search_consume_time_ms;

  JSON_DEBUG_VALUE("search_consume_time",
                   hybrid_astar_result.search_consume_time_ms)

  TimeBenchmark::Instance().SetTime(TimeBenchmarkType::TB_APA_ASTAR,
                                    hybrid_astar_result.search_consume_time_ms);

  JSON_DEBUG_VALUE("solve_number", hybrid_astar_result.solve_number)
  JSON_DEBUG_VALUE("search_node_num", hybrid_astar_result.search_node_num)

  if (request.replan_reason == ReplanReason::SLOT_CRUISING) {
    if (!path_plan_success) {
      return PathPlannerResult::PLAN_FAILED;
    }
  }

  HybridAstarResponse response;
  response.result = hybrid_astar_result;
  response.request = request;

  if (frame_.replan_reason == ReplanReason::DYNAMIC) {
    // dynamic replan
    if (!path_plan_success) {
      frame_.dynamic_plan_fail_flag = true;
      ILOG_INFO << "path dynamic plan fail";
      frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
      return PathPlannerResult::PLAN_UPDATE;
    }

    frame_.dynamic_plan_fail_flag = false;
    ILOG_INFO << "path dynamic plan success";
    if (!CheckDynamicPlanPathOptimalByHybridAstarPath(response)) {
      frame_.dynamic_plan_path_superior = false;
      frame_.plan_fail_reason = ParkingFailReason::DYNAMIC_PATH_NOT_SUPERIOR;
      ILOG_INFO << "path dynamic plan is not superior, should not replace "
                   "last path";
      return PathPlannerResult::PLAN_UPDATE;
    }

    frame_.dynamic_plan_path_superior = true;
    ILOG_INFO << "path dynamic plan is superior, should replace last path";
  } else {
    // static replan
    if (!path_plan_success) {
      ILOG_INFO << "path static plan fail";
      frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
      return PathPlannerResult::PLAN_FAILED;
    }
    ILOG_INFO << "path static plan success";
  }

  if (frame_.is_replan_first) {
    frame_.is_replan_first = false;
  }

  FillPathPointGlobalFromHybridPath(response);

  return PathPlannerResult::PLAN_UPDATE;
}

const uint8_t PerpendicularTailInScenario::PathPlanOnceHybridAstarThread() {
  const auto path_generator_thread_ptr =
      apa_world_ptr_->GetParkingTaskInterfacePtr()->GetPathGeneratorThreadPtr();

  frame_.has_response = UpdateThreadPath();

  // gen request every frame, but not set every frame
  PathGenThreadRequest request;
  request.col_det_interface_ptr = apa_world_ptr_->GetColDetInterfacePtr();
  GenHybridAstarConfigAndRequest(request.config, request.hybrid_astar_request);

  if (frame_.has_response) {
    ILOG_INFO << "path gen thread has response, update response";
    path_generator_thread_ptr->PublishResponseData(hybrid_astar_response_);
    TimeBenchmark::Instance().SetTime(
        TimeBenchmarkType::TB_APA_ASTAR,
        hybrid_astar_response_.result.search_consume_time_ms);
  }

  if (!CheckResponseReasonable(request.hybrid_astar_request,
                               hybrid_astar_response_)) {
    ILOG_INFO << "response not reasonable";
    hybrid_astar_response_.Clear();
  }

  // Once there is no result in the current frame, reuse the response from the
  // previous frame to fill the path
  FillPathPointGlobalFromHybridPath(hybrid_astar_response_);

  if (frame_.path_gen_request_response_state ==
      PathGenRequestResponseState::HAS_RESPONSE) {
    if (!hybrid_astar_response_.result.path_plan_success) {
      return PathPlannerResult::PLAN_FAILED;
    } else {
      return PathPlannerResult::PLAN_UPDATE;
    }
  }

  if (frame_.path_gen_request_response_state !=
      PathGenRequestResponseState::HAS_REQUEST) {
    ILOG_INFO << "path generator thread has not request, can set request";
    path_generator_thread_ptr->SetRequest(request);
  }

  const SlotReleaseState last_release_state =
      apa_world_ptr_->GetSlotManagerPtr()
          ->GetEgoInfoUnderSlot()
          .slot.release_info_
          .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE];

  switch (last_release_state) {
    case SlotReleaseState::RELEASE:
      ILOG_INFO << "last result is release, the frame set release";
      return PathPlannerResult::PLAN_UPDATE;
    case SlotReleaseState::UNKNOWN:
    case SlotReleaseState::COMPUTING:
      ILOG_INFO << "last result is "
                << (last_release_state == SlotReleaseState::UNKNOWN
                        ? "unknown"
                        : "computing")
                << ", the frame set computing";
      return PathPlannerResult::WAIT_PATH;
    default:
      ILOG_INFO << "last result is not release, the frame set not release";
      return PathPlannerResult::PLAN_FAILED;
  }

  return PathPlannerResult::PLAN_FAILED;
}

void PerpendicularTailInScenario::PathPlanByHybridAstarThread() {
  const ApaParameters& param = apa_param.GetParam();

  CheckReplanParams replan_params;
  replan_params.use_obs_height_method = param.use_obs_height_method;
  frame_.replan_flag = CheckReplan(replan_params);
  frame_.has_response = UpdateThreadPath();
  frame_.pathplan_result = PathPlannerResult::PLAN_UPDATE;
  frame_.plan_fail_reason = ParkingFailReason::NOT_FAILED;

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  if (!CheckCanDelObsInSlot()) {
    frame_.process_obs_method = ProcessObsMethod::DO_NOTHING;
  }

  const auto& simu_param = apa_world_ptr_->GetSimuParam();
  if (simu_param.is_simulation && simu_param.process_obs_method != -1) {
    frame_.process_obs_method =
        static_cast<ProcessObsMethod>(simu_param.process_obs_method);
  }

  if (frame_.is_replan_first) {
    current_path_point_global_vec_.clear();
    complete_path_point_global_vec_.clear();
  }

  const std::shared_ptr<PathGeneratorThread>& path_generator_thread_ptr =
      apa_world_ptr_->GetParkingTaskInterfacePtr()->GetPathGeneratorThreadPtr();

  switch (frame_.path_gen_request_response_state) {
    case PathGenRequestResponseState::HAS_REQUEST:
      ILOG_INFO << "thread request response state is has_request, should wait "
                   "for response, no need to trigger plan";
      frame_.replan_flag = false;
      break;
    case PathGenRequestResponseState::HAS_RESPONSE:
      ILOG_INFO << "thread request response state is has_response, should get "
                   "path from thread, no need to trigger plan";
      frame_.replan_flag = false;
      break;
    case PathGenRequestResponseState::HAS_PUBLISHED_RESPONSE:
      ILOG_INFO << "thread request response state is has_published_response, "
                   "use last path and can trigger plan";
      break;
    default:
      ILOG_INFO << "thread request response state is NONE, can trigger plan";
      break;
  }

  SetParkingStatus(PARKING_RUNNING);

  const bool exist_target_pose = GenTlane();
  if (!frame_.replan_flag) {
    frame_.replan_reason = NOT_REPLAN;
  }

  const bool has_thread_response = frame_.path_gen_request_response_state ==
                                   PathGenRequestResponseState::HAS_RESPONSE;
  const bool should_mark_parking_failed =
      frame_.replan_fail_time > param.max_replan_failed_time;
  if (has_thread_response) {
    constexpr double kUsableSearchTimeMs = 1900.0;

    HybridAstarResponse response;
    path_generator_thread_ptr->PublishResponseData(response);
    const HybridAStarRequest& last_request = response.request;
    const EgoInfoUnderSlot& last_ego_info_under_slot =
        last_request.ego_info_under_slot;
    const bool is_dynamic_replan =
        IsReplanReasonDynamic(last_request.replan_reason);
    const bool should_mark_parking_failed =
        frame_.replan_fail_time > param.max_replan_failed_time;

    if (last_request.replan_reason == ReplanReason::SLOT_CRUISING) {
      if (!response.result.path_plan_success) {
        ILOG_INFO << "this response path is search path and path plan failed";
        frame_.plan_fail_reason = LOSS_SEARCH_PATH;
        return;
      }
      if (response.result.search_consume_time_ms < kUsableSearchTimeMs) {
        ILOG_INFO << "this response path is search path and consume time < "
                  << kUsableSearchTimeMs << "ms, need loss and replan";
        frame_.plan_fail_reason = LOSS_SEARCH_PATH;
        return;
      }
      ILOG_INFO << "this response path is search path and consume time > "
                << kUsableSearchTimeMs
                << "ms, decide it can directly use the path";
    }

    ILOG_INFO << "hybrid astar plan success = "
              << response.result.path_plan_success << "  "
              << "  path_plan_consume_time_ms = "
              << response.result.search_consume_time_ms;

    JSON_DEBUG_VALUE("process_obs_method",
                     static_cast<int>(last_request.process_obs_method))

    JSON_DEBUG_VALUE("search_consume_time",
                     response.result.search_consume_time_ms)

    TimeBenchmark::Instance().SetTime(TimeBenchmarkType::TB_APA_ASTAR,
                                      response.result.search_consume_time_ms);

    JSON_DEBUG_VALUE("solve_number", response.result.solve_number)
    JSON_DEBUG_VALUE("search_node_num", response.result.search_node_num)

    JSON_DEBUG_VALUE("last_replan_reason", last_request.replan_reason)

    bool success = true;
    if (!response.result.path_plan_success) {
      frame_.plan_fail_reason = PATH_PLAN_FAILED;
      success = false;
      ILOG_INFO << "response path plan failed";
    } else if (last_ego_info_under_slot.id != ego_info_under_slot.id) {
      frame_.plan_fail_reason = SLOT_ID_CHANGED;
      success = false;
      ILOG_INFO << "response slot id changed, current id = "
                << ego_info_under_slot.id
                << "  response id = " << last_ego_info_under_slot.id;
    } else if (last_ego_info_under_slot.slot_type !=
               ego_info_under_slot.slot_type) {
      frame_.plan_fail_reason = SLOT_TYPE_CHANGED;
      success = false;
      ILOG_INFO << "response slot type changed, current type = "
                << GetSlotTypeString(ego_info_under_slot.slot_type)
                << "  response type = "
                << GetSlotTypeString(last_ego_info_under_slot.slot_type);
    }

    if (last_request.replan_reason == ReplanReason::DYNAMIC) {
      if (!success) {
        ILOG_INFO << "dynamic replan response invalid, reason = "
                  << static_cast<int>(frame_.plan_fail_reason);
      } else if (!CheckDynamicPlanPathOptimalByHybridAstarPath(response)) {
        frame_.plan_fail_reason = DYNAMIC_PATH_NOT_SUPERIOR;
        success = false;
        ILOG_INFO << "dynamic replan path is not superior";
      } else {
        ILOG_INFO
            << "dynamic replan success and path is superior, use this path";
      }
    }

    if (!success && !is_dynamic_replan) {
      ILOG_ERROR << "static replan failed, reason = "
                 << static_cast<int>(frame_.plan_fail_reason);
      frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
      if (should_mark_parking_failed) {
        SetParkingStatus(PARKING_FAILED);
      }
      SwitchProcessObsMethod();
      if (last_request.replan_reason == FIRST_PLAN) {
        frame_.is_replan_first = true;
      }
      return;
    }

    if (!success) {
      ILOG_INFO << "discard invalid response, reason = "
                << static_cast<int>(frame_.plan_fail_reason);
      path_generator_thread_ptr->Reset();
      return;
    }

    ego_info_under_slot.lat_move_dist_replan_success =
        last_ego_info_under_slot.lat_move_dist_every_replan;
    ego_info_under_slot.lon_move_dist_replan_success =
        last_ego_info_under_slot.lon_move_dist_every_replan;
    ego_info_under_slot.replan_success_origin_target_pose =
        last_ego_info_under_slot.origin_target_pose;
    ego_info_under_slot.replan_success_origin_target_pose.LocalToGlobal(
        last_ego_info_under_slot.l2g_tf);

    if (is_dynamic_replan) {
      ILOG_INFO << "dynamic replan success, update path";
    } else {
      ILOG_INFO << "static replan success, update path";
      ego_info_under_slot.fix_limiter = false;
    }

    ILOG_INFO << "response is valid, use new path";
    frame_.process_obs_method = ProcessObsMethod::DO_NOTHING;
    frame_.is_replan_first = false;
    FillPathPointGlobalFromHybridPath(response);

    SetParkingStatus(PARKING_PLANNING);
    if (PostProcessPath()) {
      ILOG_INFO << "postprocess path success!";
    } else {
      frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
      if (should_mark_parking_failed) {
        SetParkingStatus(PARKING_FAILED);
      }
      ILOG_INFO << "postprocess path failed!";
    }
    return;
  }

  if (!frame_.replan_flag) {
    return;
  }

  SetParkingStatus(PARKING_PLANNING);
  if (!exist_target_pose) {
    frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = ParkingFailReason::NO_TARGET_POSE;
    if (should_mark_parking_failed) {
      SetParkingStatus(PARKING_FAILED);
    }
    SwitchProcessObsMethod();
    return;
  }

  ILOG_INFO << "replan is required, set request to path generate thread!";
  PathGenThreadRequest request;
  request.col_det_interface_ptr = apa_world_ptr_->GetColDetInterfacePtr();

  GenHybridAstarConfigAndRequest(request.config, request.hybrid_astar_request);

  CalcProjPtForDynamicPlan(
      request.hybrid_astar_request.ego_info_under_slot.cur_pose,
      request.hybrid_astar_request.splicing_pt_vec);

  path_generator_thread_ptr->SetRequest(request);
  if (frame_.is_replan_first) {
    frame_.is_replan_first = false;
  }
}

void PerpendicularTailInScenario::CalcProjPtForDynamicPlan(
    geometry_lib::PathPoint& proj_pt,
    std::vector<geometry_lib::PathPoint>& splicing_pt_vec) {
  splicing_pt_vec.clear();

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  const ApaParameters& param = apa_param.GetParam();
  const auto measure_data_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();
  const auto predict_path_manager_ptr =
      apa_world_ptr_->GetPredictPathManagerPtr();

  constexpr double kRemainDistBuffer = 0.68;
  constexpr double kMinProjPointCountFactor = 2.0;
  const std::vector<double> proj_dist_breakpoints{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

  proj_pt = ego_info_under_slot.cur_pose;

  if (frame_.replan_reason == ReplanReason::DYNAMIC_GEAR_SWITCH) {
    proj_pt = GetCurrentPathTerminal(true);
    return;
  }

  if (frame_.replan_reason != ReplanReason::DYNAMIC &&
      frame_.replan_reason != ReplanReason::PATH_DANGEROUS) {
    return;
  }

  ILOG_INFO << "decide projection point for dynamic path plan";

  if (predict_path_manager_ptr->GetControlErrBig()) {
    ILOG_INFO << "control err is big, use ego pose for dynamic path plan";
    return;
  }

  const double remain_dist =
      std::min({frame_.remain_dist_path, frame_.remain_dist_obs,
                frame_.remain_dist_slot_jump, frame_.remain_dist_by_od,
                frame_.remain_dist_col_det});
  const double delta_t =
      (param.max_dynamic_plan_proj_dt - param.min_dynamic_plan_proj_dt) /
      (proj_dist_breakpoints.size() - 1);
  std::vector<double> proj_dt_table(proj_dist_breakpoints.size(),
                                    param.min_dynamic_plan_proj_dt);
  for (size_t i = 0; i < proj_dt_table.size(); ++i) {
    proj_dt_table[i] += delta_t * i;
  }
  const double dt =
      mathlib::Interp1(proj_dist_breakpoints, proj_dt_table, remain_dist);
  const double proj_dist = std::fabs(measure_data_ptr->GetVel()) * dt;

  ILOG_INFO << "dynamic plan projection: proj_dist = " << proj_dist
            << " dt = " << dt << " delta_t = " << delta_t
            << " remain_dist = " << remain_dist;

  JSON_DEBUG_VALUE("dynamic_plan_predict_dt", dt)
  JSON_DEBUG_VALUE("dynamic_plan_predict_ds", proj_dist)

  if (proj_dist > remain_dist - kRemainDistBuffer) {
    return;
  }

  const auto& path_point_global_vec = current_path_point_global_vec_;
  const size_t pt_number = path_point_global_vec.size();
  if (pt_number < 2) {
    return;
  }

  const double ds = path_point_global_vec[pt_number - 1].s -
                    path_point_global_vec[pt_number - 2].s;
  if (proj_dist < ds * kMinProjPointCountFactor) {
    return;
  }

  const double ego_s_proj =
      frame_.current_path_length - frame_.remain_dist_path;
  const double fur_s_proj = ego_s_proj + proj_dist;

  geometry_lib::PathPoint fur_proj_pt;
  size_t ego_proj_index = 0;
  size_t fur_proj_index = 0;
  bool ego_proj_found = false;
  bool fur_proj_found = false;
  for (size_t i = 0; i < pt_number; ++i) {
    const geometry_lib::PathPoint& pt = path_point_global_vec[i];
    if (!ego_proj_found && pt.s > ego_s_proj) {
      ego_proj_found = true;
      ego_proj_index = i;
    }
    if (!fur_proj_found && pt.s > fur_s_proj) {
      fur_proj_found = true;
      fur_proj_index = i;
      fur_proj_pt = pt;
    }
    if (ego_proj_found && fur_proj_found) {
      break;
    }
  }

  if (!ego_proj_found || !fur_proj_found || fur_proj_index <= ego_proj_index) {
    ILOG_INFO << "failed to find valid projection points on current path";
    return;
  }

  proj_pt.pos = ego_info_under_slot.g2l_tf.GetPos(fur_proj_pt.pos);
  proj_pt.heading = ego_info_under_slot.g2l_tf.GetHeading(fur_proj_pt.heading);

  splicing_pt_vec.reserve(fur_proj_index - ego_proj_index);
  for (size_t i = ego_proj_index + 1; i < fur_proj_index; ++i) {
    splicing_pt_vec.emplace_back(path_point_global_vec[i]);
  }

  fur_proj_pt.PrintInfo();
}

size_t PerpendicularTailInScenario::GetHybridSegmentPointSize(
    const HybridAStarResult& result, size_t index) const {
  return std::min(
      {result.x_vec_vec[index].size(), result.y_vec_vec[index].size(),
       result.phi_vec_vec[index].size(), result.kappa_vec_vec[index].size(),
       result.accumulated_s_vec_vec[index].size(),
       result.type_vec_vec[index].size()});
}

void PerpendicularTailInScenario::WriteHybridSegmentFromPathPoints(
    const std::vector<geometry_lib::PathPoint>& path_points,
    HybridAStarResult* result, size_t index) const {
  if (result == nullptr) {
    return;
  }

  result->x_vec_vec[index].resize(path_points.size());
  result->y_vec_vec[index].resize(path_points.size());
  result->phi_vec_vec[index].resize(path_points.size());
  result->kappa_vec_vec[index].resize(path_points.size());
  result->accumulated_s_vec_vec[index].resize(path_points.size());
  result->type_vec_vec[index].resize(path_points.size());
  for (size_t i = 0; i < path_points.size(); ++i) {
    const auto& pt = path_points[i];
    result->x_vec_vec[index][i] = pt.pos.x();
    result->y_vec_vec[index][i] = pt.pos.y();
    result->phi_vec_vec[index][i] = pt.heading;
    result->kappa_vec_vec[index][i] = pt.kappa;
    result->accumulated_s_vec_vec[index][i] = pt.s;
    result->type_vec_vec[index][i] = static_cast<AstarPathType>(pt.type);
  }
}

std::vector<geometry_lib::PathPoint>
PerpendicularTailInScenario::BuildHybridSegmentPathPoints(
    const HybridAStarResult& result, size_t index) const {
  const auto& x_vec = result.x_vec_vec[index];
  const auto& y_vec = result.y_vec_vec[index];
  const auto& phi_vec = result.phi_vec_vec[index];
  const auto& kappa_vec = result.kappa_vec_vec[index];
  const auto& s_vec = result.accumulated_s_vec_vec[index];
  const auto& type_vec = result.type_vec_vec[index];
  const auto seg_gear = GetSegGearFromAstarGear(result.gear_vec[index]);
  const size_t pt_size = GetHybridSegmentPointSize(result, index);

  std::vector<geometry_lib::PathPoint> path_points;
  path_points.reserve(pt_size);
  for (size_t i = 0; i < pt_size; ++i) {
    geometry_lib::PathPoint pt;
    pt.pos << x_vec[i], y_vec[i];
    pt.heading = phi_vec[i];
    pt.kappa = kappa_vec[i];
    pt.s = s_vec[i];
    pt.type = static_cast<int>(type_vec[i]);
    pt.gear = seg_gear;
    path_points.emplace_back(pt);
  }
  return path_points;
}

void PerpendicularTailInScenario::AppendHybridSegmentGlobalPathPoints(
    const HybridAStarResult& result, size_t index,
    const geometry_lib::LocalToGlobalTf& l2g_tf,
    std::vector<geometry_lib::PathPoint>* current_path_points,
    std::vector<geometry_lib::PathPoint>* complete_path_points) const {
  if (current_path_points == nullptr || complete_path_points == nullptr) {
    return;
  }

  const auto path_points = BuildHybridSegmentPathPoints(result, index);
  for (const auto& path_point : path_points) {
    geometry_lib::PathPoint global_path_point = path_point;
    global_path_point.pos = l2g_tf.GetPos(path_point.pos);
    global_path_point.heading = l2g_tf.GetHeading(path_point.heading);
    if (index == 0) {
      current_path_points->emplace_back(global_path_point);
    }
    complete_path_points->emplace_back(global_path_point);
  }
}

void PerpendicularTailInScenario::OptimizeHybridFirstSegment(
    HybridAStarResult* result) {
  // if (result == nullptr || result->gear_vec.empty()) {
  //   return;
  // }

  // std::vector<geometry_lib::PathPoint> raw_pts_vec =
  //     BuildHybridSegmentPathPoints(*result, 0);

  // std::vector<geometry_lib::PathPoint> optimized_pts_vec;
  // PathOptimizeParams optimize_params;
  // optimize_params.enable_optimize =
  //     apa_param.GetParam().perpendicular_lat_opt_enable;
  // optimize_params.use_obs_height_method =
  //     apa_param.GetParam().use_obs_height_method;
  // optimize_params.base_on_slot = true;
  // if (!PathOptimize(raw_pts_vec, optimized_pts_vec, optimize_params)) {
  //   return;
  // }

  // WriteHybridSegmentFromPathPoints(optimized_pts_vec, result, 0);
  return;
}

void PerpendicularTailInScenario::FillPathPointGlobalFromHybridPath(
    const HybridAstarResponse& response) {
  const HybridAStarResult& result = response.result;
  const HybridAStarRequest& request = response.request;
  const std::vector<geometry_lib::PathPoint>& splicing_pt_vec =
      request.splicing_pt_vec;

  hybrid_astar_response_ = response;

  frame_.is_last_path = (result.gear_change_num < 1);

  frame_.cur_path_gear_change_count = result.gear_change_num;

  frame_.gear_command = GetSegGearFromAstarGear(result.cur_gear);
  frame_.current_gear = geometry_lib::ReverseGear(frame_.gear_command);
  frame_.current_arc_steer =
      geometry_lib::ReverseSteer(GetSegSteerFromAstarSteer(result.cur_steer));

  ILOG_INFO << "path gear change count = " << result.gear_change_num
            << " path cur gear = " << PathGearDebugString(result.cur_gear);

  current_path_point_global_vec_.clear();
  complete_path_point_global_vec_.clear();

  if (result.x_vec_vec.empty()) {
    ILOG_INFO << "hybrid path is empty, skip filling global path points";
    return;
  }

  const size_t gear_size = std::min(
      {result.gear_vec.size(), result.x_vec_vec.size(), result.y_vec_vec.size(),
       result.phi_vec_vec.size(), result.kappa_vec_vec.size(),
       result.type_vec_vec.size(), result.accumulated_s_vec_vec.size()});
  if (gear_size == 0) {
    ILOG_INFO << "hybrid path is empty, skip filling global path points";
    return;
  }

  size_t complete_path_point_count = splicing_pt_vec.size();
  for (size_t i = 0; i < gear_size; ++i) {
    complete_path_point_count += GetHybridSegmentPointSize(result, i);
  }

  // OptimizeHybridFirstSegment(&result);

  const size_t first_seg_size = GetHybridSegmentPointSize(result, 0);

  current_path_point_global_vec_.reserve(first_seg_size +
                                         splicing_pt_vec.size() + 18);
  complete_path_point_global_vec_.reserve(complete_path_point_count + 18);

  current_path_point_global_vec_.insert(current_path_point_global_vec_.end(),
                                        splicing_pt_vec.begin(),
                                        splicing_pt_vec.end());
  complete_path_point_global_vec_.insert(complete_path_point_global_vec_.end(),
                                         splicing_pt_vec.begin(),
                                         splicing_pt_vec.end());

  const geometry_lib::LocalToGlobalTf& l2g_tf =
      request.ego_info_under_slot.l2g_tf;
  for (size_t i = 0; i < gear_size; ++i) {
    AppendHybridSegmentGlobalPathPoints(result, i, l2g_tf,
                                        &current_path_point_global_vec_,
                                        &complete_path_point_global_vec_);
  }

  if (!complete_path_point_global_vec_.empty()) {
    EgoInfoUnderSlot& mutable_ego_info_under_slot =
        apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
    const geometry_lib::PathPoint& origin_target_pose =
        mutable_ego_info_under_slot.origin_target_pose;

    geometry_lib::PathPoint real_target_pose =
        complete_path_point_global_vec_.back();
    real_target_pose.GlobalToLocal(mutable_ego_info_under_slot.g2l_tf);

    mutable_ego_info_under_slot.lat_move_dist_replan_success =
        real_target_pose.GetY() - origin_target_pose.GetY();
    mutable_ego_info_under_slot.lon_move_dist_replan_success =
        real_target_pose.GetX() - origin_target_pose.GetX();
    mutable_ego_info_under_slot.target_pose = real_target_pose;
  }

  ILOG_INFO << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size()
            << "  complete_path_point_global_vec_.size() = "
            << complete_path_point_global_vec_.size();
}

void PerpendicularTailInScenario::SwitchProcessObsMethod() {
  switch (frame_.process_obs_method) {
    case ProcessObsMethod::DO_NOTHING:
      frame_.process_obs_method = ProcessObsMethod::MOVE_OBS_OUT_SLOT;
      break;
    case ProcessObsMethod::MOVE_OBS_OUT_SLOT:
      frame_.process_obs_method = ProcessObsMethod::MOVE_OBS_OUT_CAR_SAFE_POS;
      break;
    case ProcessObsMethod::MOVE_OBS_OUT_CAR_SAFE_POS:
    default:
      frame_.process_obs_method = ProcessObsMethod::DO_NOTHING;
      break;
  }
}

const bool PerpendicularTailInScenario::CheckFinished() {
  const ApaParameters& param = apa_param.GetParam();
  const CheckFinishParams& finish_params = param.check_finish_params;

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  const auto& cur_pose = ego_info_under_slot.cur_pose;
  const auto& target_pose = ego_info_under_slot.target_pose;
  const auto front_pose = GetCarFrontPoseFromCarPose(cur_pose);

  ego_info_under_slot.terminal_err.Set(
      cur_pose.pos - target_pose.pos,
      geometry_lib::NormalizeAngle(cur_pose.heading - target_pose.heading));

  ego_info_under_slot.terminal_y_front_err =
      front_pose.GetY() - target_pose.GetY();

  const double lon_err = ego_info_under_slot.terminal_err.GetX();
  const double lat_err = ego_info_under_slot.terminal_err.GetY();
  const double front_lat_err = ego_info_under_slot.terminal_y_front_err;
  const double heading_err =
      ego_info_under_slot.terminal_err.GetTheta() * kRad2Deg;

  JSON_DEBUG_VALUE("terminal_error_x", lon_err)
  JSON_DEBUG_VALUE("terminal_error_y", lat_err)
  JSON_DEBUG_VALUE("terminal_error_y_front", front_lat_err)
  JSON_DEBUG_VALUE("terminal_error_heading", heading_err * kDeg2Rad)
  ILOG_INFO << "terminal_error_x = " << lon_err
            << "  terminal_error_y = " << lat_err
            << "  front_lat_err = " << front_lat_err
            << "  heading_err = " << heading_err;

  const CarSlotRelationship car_slot_relationship =
      CalCarSlotRelationship(cur_pose);

  const bool auto_fold_mirror =
      param.smart_fold_mirror_params.has_smart_fold_mirror &&
      frame_.mirror_command == MirrorCommand::FOLD &&
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetFoldMirrorFlag();

  ILOG_INFO << "check finish ship = "
            << static_cast<int>(car_slot_relationship);

  if (car_slot_relationship == CarSlotRelationship::TOUCHING &&
      !auto_fold_mirror) {
    ILOG_INFO << "car press line, not allow finish";
    return false;
  }

  const bool is_space_slot =
      (ego_info_under_slot.slot.slot_source_type_ == SlotSourceType::USS);

  const double gain = (auto_fold_mirror || is_space_slot) ? 3.0 : 1.0;

  const double finish_lon_err = finish_params.lon_err;

  const double finish_lat_err =
      (car_slot_relationship == CarSlotRelationship::IDEAL
           ? finish_params.lat_err
           : finish_params.lat_err_strict) *
      gain;

  const double finish_heading_err =
      (car_slot_relationship == CarSlotRelationship::IDEAL
           ? finish_params.heading_err
           : finish_params.heading_err_strict) *
      gain;

  const bool lon_condition = std::fabs(lon_err) < finish_lon_err;

  const bool lat_condition = std::fabs(lat_err) < finish_lat_err &&
                             std::fabs(front_lat_err) < finish_lat_err;

  const bool heading_condition = std::fabs(heading_err) < finish_heading_err;

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  // first replan not check remain_s_condition
  const bool remain_s_condition =
      frame_.remain_dist_path < finish_params.max_remain_path_dist ||
      frame_.is_replan_first;

  const bool basic_finish = lon_condition && lat_condition &&
                            heading_condition && static_condition &&
                            remain_s_condition;

  ILOG_INFO << "finish condition, lon = " << lon_condition
            << "  lat = " << lat_condition
            << "  heading = " << heading_condition
            << "  static = " << static_condition
            << "  remain_s = " << remain_s_condition
            << "  finish_lon_err = " << finish_lon_err
            << "  finish_lat_err = " << finish_lat_err
            << "  finish_heading_err = " << finish_heading_err;

  if (basic_finish) {
    return true;
  }

  // stucked by directly behind obs
  const bool enter_slot_condition = ego_info_under_slot.slot_occupied_ratio >
                                    finish_params.obs_stuck_slot_occupied_ratio;
  const bool remain_obs_condition =
      frame_.remain_dist_obs < finish_params.max_remain_obs_dist;

  const bool has_fold_mirror =
      apa_world_ptr_->GetColDetInterfacePtr()->GetFoldMirrorFlag();

  if (!has_fold_mirror) {
    apa_world_ptr_->GetColDetInterfacePtr()->Init(true);
  }

  GJKColDetRequest gjk_col_det_request(true, param.uss_config.use_uss_pt_cloud);
  const auto has_obstacle_at_pose = [&](const geometry_lib::PathPoint& pose) {
    return apa_world_ptr_->GetColDetInterfacePtr()
        ->GetGJKColDetPtr()
        ->Update(std::vector<geometry_lib::PathPoint>{pose},
                 ColDetBuffer(0.0, 0.0), gjk_col_det_request)
        .col_flag;
  };

  bool target_pose_blocked = has_obstacle_at_pose(target_pose);

  ILOG_INFO << "target_pose_blocked = " << target_pose_blocked;

  if (!target_pose_blocked) {
    double move_back_dist = 0.168;
    if (lon_err < 0.368) {
      // if only can move 0.1m, no move, finish
      move_back_dist = param.lat_lon_speed_buffer.lon_buffer + 0.1;
    }

    const geometry_lib::PathPoint uss_pose{
        target_pose.pos - move_back_dist * Eigen::Vector2d(1.0, 0.0),
        target_pose.heading};

    target_pose_blocked = has_obstacle_at_pose(uss_pose);

    ILOG_INFO << "after_move_back_target_pose_blocked = " << target_pose_blocked
              << "  move_back_dist = " << move_back_dist;
  }

  if (!has_fold_mirror) {
    apa_world_ptr_->GetColDetInterfacePtr()->Init(false);
  }

  // If there are indeed obstacles near the target pose, finishing is allowed
  // because reaching the exact target pose is no longer feasible.
  const bool obs_stuck_finish = lat_condition && heading_condition &&
                                static_condition && enter_slot_condition &&
                                remain_obs_condition && target_pose_blocked;

  ILOG_INFO << "obs stuck finish condition, lat = " << lat_condition
            << "  heading = " << heading_condition
            << "  static = " << static_condition
            << "  enter_slot = " << enter_slot_condition
            << "  remain_obs = " << remain_obs_condition
            << "  target_pose_blocked = " << target_pose_blocked;

  return obs_stuck_finish;
}

const bool PerpendicularTailInScenario::PostProcessPathAccordingLimiter() {
  frame_.correct_path_for_limiter = false;

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  const ApaParameters& param = apa_param.GetParam();

  size_t origin_traj_size = current_path_point_global_vec_.size();

  if (frame_.mirror_command == MirrorCommand::FOLD &&
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetFoldMirrorFlag() &&
      ego_info_under_slot.cur_pose.GetX() -
              ego_info_under_slot.origin_target_pose.GetX() >
          0.068) {
    ILOG_INFO << "mirror has folded, try to change path according limiter";
    ego_info_under_slot.fix_limiter = false;
  }

  if (frame_.gear_command != geometry_lib::SEG_GEAR_REVERSE &&
          !frame_.is_last_path ||
      !frame_.spline_success || origin_traj_size < 2) {
    return false;
  }

  const geometry_lib::LineSegment limiter_line(
      Eigen::Vector2d(ego_info_under_slot.origin_target_pose.pos.x(),
                      0.5 * ego_info_under_slot.slot.slot_width_),
      Eigen::Vector2d(ego_info_under_slot.origin_target_pose.pos.x(),
                      -0.5 * ego_info_under_slot.slot.slot_width_));

  const double dist_ego_limiter = geometry_lib::CalPoint2LineDist(
      ego_info_under_slot.cur_pose.pos, limiter_line);

  const double diff = ego_info_under_slot.slot_occupied_ratio_postprocess -
                      ego_info_under_slot.slot_occupied_ratio;

  double delta_dis = 0.0;
  if (diff < 0.06) {
    delta_dis = 0.0;
  } else if (diff < 0.12) {
    delta_dis = 0.43;
  } else if (diff < 0.24) {
    delta_dis = 0.86;
  } else if (diff < 0.36) {
    delta_dis = 1.72;
  } else {
    delta_dis = 2.58;
  }

  const double car_to_limiter_dis = param.car_to_limiter_dis + delta_dis;

  ILOG_INFO << "dist_ego_limiter = " << dist_ego_limiter
            << "  car_to_limiter_dis = " << car_to_limiter_dis;

  if (!(dist_ego_limiter < car_to_limiter_dis &&
        std::fabs(ego_info_under_slot.terminal_err.heading) * kRad2Deg <
            param.check_finish_params.heading_err * 2.0 &&
        std::fabs(ego_info_under_slot.terminal_err.pos.y()) <
            param.check_finish_params.lat_err * 2.0)) {
    return false;
  }

  ILOG_INFO << "try extend or trim path by limiter";

  std::vector<double> x_vec, y_vec, s_vec, heading_vec;
  x_vec.reserve(origin_traj_size + 20);
  y_vec.reserve(origin_traj_size + 20);
  s_vec.reserve(origin_traj_size + 20);
  heading_vec.reserve(origin_traj_size + 20);

  const auto& l2g_tf = ego_info_under_slot.l2g_tf;
  const auto& g2l_tf = ego_info_under_slot.g2l_tf;
  const Eigen::Vector2d limiter_mid =
      l2g_tf.GetPos(ego_info_under_slot.origin_target_pose.pos);

  geometry_lib::PathPoint pt = current_path_point_global_vec_.back();

  // If the target pose is very close to the previously planned
  // endpoint, there is no need to run the following steps
  if ((limiter_mid - pt.pos).norm() < 0.026) {
    return false;
  }

  if (ego_info_under_slot.fix_limiter &&
      (limiter_mid - pt.pos).norm() >
          param.check_finish_params.lon_err - 0.026) {
    ego_info_under_slot.fix_limiter = false;
  }

  if (ego_info_under_slot.fix_limiter &&
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag() &&
      dist_ego_limiter > param.check_finish_params.lon_err - 0.026) {
    ego_info_under_slot.fix_limiter = false;
  }

  if (ego_info_under_slot.fix_limiter) {
    return false;
  }

  const double max_extend_dist = 2.5;

  double s_proj = 0.0;
  bool success = geometry_lib::CalProjFromSplineByBisection(
      0.0, frame_.current_path_length + max_extend_dist, s_proj, limiter_mid,
      frame_.x_s_spline, frame_.y_s_spline);

  if (!success) {
    ILOG_INFO << "path is err";
    return false;
  }

  if (s_proj < 0.068) {
    ILOG_INFO << "limiter s_proj is too small";
    return false;
  }

  double ds = 0.0, s = 0.0;
  for (size_t i = 0; i < origin_traj_size; ++i) {
    if (i > 0) {
      ds = std::hypot(current_path_point_global_vec_[i].GetX() -
                          current_path_point_global_vec_[i - 1].GetX(),
                      current_path_point_global_vec_[i].GetY() -
                          current_path_point_global_vec_[i - 1].GetY());
      s += std::max(ds, 1e-3);
    }

    if (s > s_proj) {
      ILOG_INFO << "path shoule be shorten because of limiter";
      if (s_proj - s_vec.back() > 0.036) {
        x_vec.emplace_back(frame_.x_s_spline(s_proj));
        y_vec.emplace_back(frame_.y_s_spline(s_proj));
        heading_vec.emplace_back(heading_vec.back());
        s_vec.emplace_back(s_proj);
      }
      break;
    }

    x_vec.emplace_back(current_path_point_global_vec_[i].GetX());
    y_vec.emplace_back(current_path_point_global_vec_[i].GetY());
    heading_vec.emplace_back(current_path_point_global_vec_[i].GetTheta());
    s_vec.emplace_back(s);
  }

  if (s < s_proj) {
    ILOG_INFO << "path shoule be extended because of limiter";
    if (std::fabs(g2l_tf.GetHeading(pt.GetTheta())) * kRad2Deg >
        param.check_finish_params.heading_err_strict) {
      ILOG_INFO << "ego heading is big, not allow extend path";
      return false;
    }

    double extend_length = s_proj - s;
    double temp_s = 0.0, temp_ds = 0.05;
    pt.s = temp_s;
    // reverse gear
    const Eigen::Vector2d unit_heading_vec =
        geometry_lib::GenHeadingVec(pt.GetTheta()) * (-1.0);
    std::vector<geometry_lib::PathPoint> extend_pt_vec{pt};
    do {
      temp_s += temp_ds;
      if (temp_s >= extend_length) {
        temp_ds = std::max(extend_length - extend_pt_vec.back().s, 0.01);
        temp_s = extend_length;
      }
      pt.pos += unit_heading_vec * temp_ds;
      pt.s += temp_ds;
      extend_pt_vec.emplace_back(pt);
    } while (temp_s < extend_length - 1e-5);

    ILOG_INFO << "extend path according limiter with fold mirror = "
              << apa_world_ptr_->GetColDetInterfacePtr()->GetFoldMirrorFlag();

    double body_lat_buffer = param.lat_lon_speed_buffer.stop_body_lat_buffer;
    double mirror_lat_buffer =
        param.lat_lon_speed_buffer.stop_mirror_lat_buffer;
    double lon_buffer = param.lat_lon_speed_buffer.lon_buffer;

    if (param.park_path_plan_type == ParkPathPlanType::GEOMETRY) {
      body_lat_buffer = param.stop_lat_inflation;
      mirror_lat_buffer = param.stop_lat_inflation;
      lon_buffer = param.col_obs_safe_dist_normal;
    }

    body_lat_buffer += 0.02;
    mirror_lat_buffer += 0.02;

    const ColResult col_res =
        apa_world_ptr_->GetColDetInterfacePtr()->GetGJKColDetPtr()->Update(
            extend_pt_vec,
            ColDetBuffer(lon_buffer, body_lat_buffer, mirror_lat_buffer),
            GJKColDetRequest(false));

    if (col_res.remain_dist < 0.02) {
      ILOG_INFO << "consider obs extend_length is small, not allow extend "
                   "length, value = "
                << col_res.remain_dist;
      return false;
    }

    ILOG_INFO << "origin extend_length = " << extend_length
              << "  consider obs extend_length = " << col_res.remain_dist;

    extend_length = col_res.remain_dist;

    const double total_length = s + extend_length;
    while (s < total_length - 1e-2) {
      s += temp_ds;
      if (s > total_length) {
        s = total_length;
      }
      x_vec.emplace_back(frame_.x_s_spline(s));
      y_vec.emplace_back(frame_.y_s_spline(s));
      heading_vec.emplace_back(heading_vec.back());
      s_vec.emplace_back(s);
    }
  }

  const size_t N = x_vec.size();
  if (N < 2) {
    return false;
  }

  // need extend by cal proj point
  Eigen::Vector2d extended_point;
  success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
      Eigen::Vector2d(x_vec[N - 2], y_vec[N - 2]),
      Eigen::Vector2d(x_vec[N - 1], y_vec[N - 1]), extended_point,
      frame_.path_extended_dist);

  if (!success) {
    return false;
  }

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(N);
  pnc::geometry_lib::PathPoint path_point;
  for (size_t i = 0; i < N; ++i) {
    path_point.Set(Eigen::Vector2d(x_vec[i], y_vec[i]), heading_vec[i]);
    path_point.s = s_vec[i];
    current_path_point_global_vec_.emplace_back(path_point);
  }
  complete_path_point_global_vec_ = current_path_point_global_vec_;

  frame_.current_path_length = s_vec.back();

  x_vec.emplace_back(extended_point.x());
  y_vec.emplace_back(extended_point.y());
  heading_vec.emplace_back(heading_vec.back());
  s_vec.emplace_back(frame_.current_path_length + frame_.path_extended_dist);

  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;

  ego_info_under_slot.fix_limiter = true;
  frame_.correct_path_for_limiter = true;

  if (complete_path_point_global_vec_.size() > 0) {
    geometry_lib::PathPoint end_pose = complete_path_point_global_vec_.back();
    end_pose.GlobalToLocal(ego_info_under_slot.g2l_tf);
    ego_info_under_slot.target_pose.pos.x() = end_pose.pos.x();
    ego_info_under_slot.target_pose.PrintInfo();
    ego_info_under_slot.lon_move_dist_every_replan =
        ego_info_under_slot.target_pose.pos.x() -
        ego_info_under_slot.origin_target_pose.pos.x();
    ego_info_under_slot.lon_move_dist_replan_success =
        ego_info_under_slot.lon_move_dist_every_replan;
  }
  ILOG_INFO << "adjust path according to limiter complete";

  return true;
}

const double PerpendicularTailInScenario::CalRealTimeBrakeDist() {
  const double start_time = IflyTime::Now_ms();
  const auto measure_data_manager_ptr =
      apa_world_ptr_->GetMeasureDataManagerPtr();
  const auto col_det_interface_ptr = apa_world_ptr_->GetColDetInterfacePtr();
  const auto predict_path_manager_ptr =
      apa_world_ptr_->GetPredictPathManagerPtr();
  col_det_interface_ptr->Init(measure_data_manager_ptr->GetFoldMirrorFlag());

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  const geometry_lib::PathPoint& cur_pose = ego_info_under_slot.cur_pose;

  const double termial_err_y = ego_info_under_slot.terminal_err.GetY();
  const double termial_err_heading =
      ego_info_under_slot.terminal_err.GetTheta() * kRad2Deg;

  const ApaParameters& param = apa_param.GetParam();
  const ParkingLatLonSpeedBuffer& speed_buffer = param.lat_lon_speed_buffer;
  const bool need_extra_reverse_lon_buffer =
      frame_.gear_command == geometry_lib::SEG_GEAR_REVERSE &&
      (!frame_.is_last_path || frame_.slot_jump_big_flag ||
       std::fabs(termial_err_y) > 0.08 ||
       std::fabs(termial_err_heading) > 2.068);

  const bool use_geometry_plan_method =
      param.park_path_plan_type == ParkPathPlanType::GEOMETRY;

  double lon_buffer;
  if (use_geometry_plan_method) {
    lon_buffer = (ego_info_under_slot.slot_occupied_ratio < 0.05)
                     ? param.safe_uss_remain_dist_out_slot
                     : param.safe_uss_remain_dist_in_slot;

    geometry_lib::GeometryPath geometry_path_brake(all_plan_path_vec_);
    if (geometry_path_brake.gear_change_count > 0 &&
        geometry_path_brake.gear_index_vec.size() > 0) {
      const size_t start_index = geometry_path_brake.gear_index_vec[0];
      size_t end_index = geometry_path_brake.path_count - 1;
      if (geometry_path_brake.gear_index_vec.size() > 1) {
        end_index = geometry_path_brake.gear_index_vec[1] - 1;
      }
      std::vector<geometry_lib::PathSegment> seg_vec;
      seg_vec.reserve(end_index - start_index + 1);
      for (size_t i = start_index; i <= end_index; ++i) {
        seg_vec.emplace_back(geometry_path_brake.path_segment_vec[i]);
      }
      geometry_path_brake.SetPath(seg_vec);
      if (geometry_path_brake.collide_flag &&
          geometry_path_brake.total_length < 0.76) {
        lon_buffer = param.limited_safe_uss_remain_dist;
        ILOG_INFO
            << "next path is col and length is short, so this path need be "
               "more radical";
      }
    }

    if (need_extra_reverse_lon_buffer) {
      lon_buffer += 0.05;
    }
  } else {
    // use hybrid a star
    lon_buffer = speed_buffer.lon_buffer;
    const HybridAStarResult& res = hybrid_astar_response_.result;
    if (res.length_vec.size() > 1 && res.length_vec[0] < 0.68 &&
        res.length_vec[1] < 0.68) {
      lon_buffer = speed_buffer.extreme_case_lon_buffer;
      ILOG_INFO << "cur gear length and next gear length are both short, so "
                   "cur gear path need be more radical";
    }

    if (need_extra_reverse_lon_buffer) {
      lon_buffer += speed_buffer.extra_reverse_gear_lon_buffer;
    }
  }

  bool increase_lat_err_flag = false;
  if (frame_.mirror_command == MirrorCommand::NONE) {
    const bool control_err_big = predict_path_manager_ptr->GetControlErrBig();
    const bool in_slot_jump_big =
        ego_info_under_slot.slot_occupied_ratio > 0.168 &&
        frame_.slot_jump_big_flag;

    const bool geometry_need_increase =
        frame_.gear_command == geometry_lib::SEG_GEAR_DRIVE &&
        ego_info_under_slot.cur_pose.GetX() >
            ego_info_under_slot.slot.GetOriginCornerCoordLocal().pt_01_mid.x() +
                2.168;

    const bool hybrid_need_increase =
        ego_info_under_slot.slot_occupied_ratio < 1e-4;

    if (control_err_big || in_slot_jump_big) {
      increase_lat_err_flag = true;
    } else if (use_geometry_plan_method) {
      increase_lat_err_flag = geometry_need_increase;
    } else {
      increase_lat_err_flag = hybrid_need_increase;
    }
  }

  double stop_body_lat_inflation = param.stop_lat_inflation;
  double stop_mirror_lat_inflation = param.stop_lat_inflation;
  double stop_lon_dist = param.stop_lon_dist;

  double heavy_brake_body_lat_inflation = param.heavy_brake_lat_inflation;
  double heavy_brake_mirror_lat_inflation = param.heavy_brake_lat_inflation;
  double heavy_brake_lon_dist = param.heavy_brake_lon_dist;

  double moderate_brake_body_lat_inflation = param.moderate_brake_lat_inflation;
  double moderate_brake_mirror_lat_inflation =
      param.moderate_brake_lat_inflation;
  double moderate_brake_lon_dist = param.moderate_brake_lon_dist;

  double slight_brake_body_lat_inflation = param.slight_brake_lat_inflation;
  double slight_brake_mirror_lat_inflation = param.slight_brake_lat_inflation;
  double slight_brake_lon_dist = param.slight_brake_lon_dist;

  if (!use_geometry_plan_method) {
    stop_body_lat_inflation = speed_buffer.stop_body_lat_buffer;
    stop_mirror_lat_inflation = speed_buffer.stop_mirror_lat_buffer;
    stop_lon_dist = speed_buffer.stop_min_lon_dist;

    heavy_brake_body_lat_inflation = speed_buffer.low_speed_body_lat_buffer;
    heavy_brake_mirror_lat_inflation = speed_buffer.low_speed_mirror_lat_buffer;
    heavy_brake_lon_dist = speed_buffer.low_speed_min_lon_dist;

    moderate_brake_body_lat_inflation = speed_buffer.mid_speed_body_lat_buffer;
    moderate_brake_mirror_lat_inflation =
        speed_buffer.mid_speed_mirror_lat_buffer;
    moderate_brake_lon_dist = speed_buffer.mid_speed_min_lon_dist;

    slight_brake_body_lat_inflation = speed_buffer.high_speed_body_lat_buffer;
    slight_brake_mirror_lat_inflation =
        speed_buffer.high_speed_mirror_lat_buffer;
    slight_brake_lon_dist = speed_buffer.high_speed_min_lon_dist;
  }

  // adopting a graded lat buffer real-time braking
  std::vector<RealTimeBrakeInfo> real_time_brake_info_vec = {
      RealTimeBrakeInfo(
          RealTimeBrakeType::STOP, stop_lon_dist,
          ColDetBuffer(lon_buffer, stop_body_lat_inflation,
                       stop_mirror_lat_inflation),
          ColDetBuffer(speed_buffer.dynamic_lon_buffer,
                       speed_buffer.dynamic_stop_body_lat_buffer,
                       speed_buffer.dynamic_stop_mirror_lat_buffer)),
      RealTimeBrakeInfo(
          RealTimeBrakeType::HEAVY_BRAKE, heavy_brake_lon_dist,
          ColDetBuffer(lon_buffer, heavy_brake_body_lat_inflation,
                       heavy_brake_mirror_lat_inflation),
          ColDetBuffer(speed_buffer.dynamic_lon_buffer,
                       speed_buffer.dynamic_low_speed_body_lat_buffer,
                       speed_buffer.dynamic_low_speed_mirror_lat_buffer)),
      RealTimeBrakeInfo(
          RealTimeBrakeType::MODERATE_BRAKE, moderate_brake_lon_dist,
          ColDetBuffer(lon_buffer, moderate_brake_body_lat_inflation,
                       moderate_brake_mirror_lat_inflation),
          ColDetBuffer(speed_buffer.dynamic_lon_buffer,
                       speed_buffer.dynamic_mid_speed_body_lat_buffer,
                       speed_buffer.dynamic_mid_speed_mirror_lat_buffer)),
      RealTimeBrakeInfo(
          RealTimeBrakeType::SLIGHT_BRAKE, slight_brake_lon_dist,
          ColDetBuffer(lon_buffer, slight_brake_body_lat_inflation,
                       slight_brake_mirror_lat_inflation),
          ColDetBuffer(speed_buffer.dynamic_lon_buffer,
                       speed_buffer.dynamic_high_speed_body_lat_buffer,
                       speed_buffer.dynamic_high_speed_mirror_lat_buffer))};

  bool special_stop_flag = increase_lat_err_flag;
  double special_stop_body_lat_buffer =
      speed_buffer.special_stop_body_lat_buffer;
  double special_stop_mirror_lat_buffer =
      speed_buffer.special_stop_mirror_lat_buffer;
  double special_stop_lon_dist = speed_buffer.special_stop_min_lon_dist;
  double special_stop_lon_buffer = lon_buffer;
  if (!special_stop_flag && measure_data_manager_ptr->GetStaticFlag()) {
    const double traveled_dist =
        frame_.current_path_length - frame_.remain_dist_path;
    const bool new_plan_path = traveled_dist < 0.168;
    if (new_plan_path) {
      // leave inital place logic to let car move
      if (speed_buffer.enable_leave_initial_place &&
          traveled_dist < speed_buffer.leave_initial_place_dist) {
        special_stop_flag = true;
        special_stop_body_lat_buffer =
            speed_buffer.leave_initial_place_body_lat_buffer;
        special_stop_mirror_lat_buffer =
            speed_buffer.leave_initial_place_mirror_lat_buffer;
        special_stop_lon_dist = speed_buffer.leave_initial_place_min_lon_dist;
        special_stop_lon_buffer = speed_buffer.leave_initial_place_lon_buffer;
      }
    } else {
      // keep stuck place logic to let cat stuck and replan reverse path
      if (speed_buffer.enable_keep_stuck_place &&
          !frame_.stuck_by_dynamic_obs && frame_.stuck_obs_time > 0.0) {
        special_stop_flag = true;
        special_stop_body_lat_buffer =
            speed_buffer.keep_stuck_place_body_lat_buffer;
        special_stop_mirror_lat_buffer =
            speed_buffer.keep_stuck_place_mirror_lat_buffer;
        special_stop_lon_dist = speed_buffer.keep_stuck_place_min_lon_dist;
        special_stop_lon_buffer = speed_buffer.keep_stuck_place_lon_buffer;
      }
    }
  }

  if (special_stop_flag) {
    real_time_brake_info_vec[0] = RealTimeBrakeInfo(
        RealTimeBrakeType::STOP, special_stop_lon_dist,
        ColDetBuffer(special_stop_lon_buffer, special_stop_body_lat_buffer,
                     special_stop_mirror_lat_buffer),
        ColDetBuffer(speed_buffer.dynamic_lon_buffer,
                     speed_buffer.dynamic_stop_body_lat_buffer,
                     speed_buffer.dynamic_stop_mirror_lat_buffer));
  }

  double safe_remain_dist = std::numeric_limits<double>::infinity();
  for (const auto& real_time_brake_info : real_time_brake_info_vec) {
    double remain_dist = CalRemainDistFromObs(
        real_time_brake_info.static_col_det_buffer,
        real_time_brake_info.dynamic_col_det_buffer, false,
        param.use_obs_height_method);
    remain_dist = std::max(remain_dist, real_time_brake_info.min_lon_dist);
    safe_remain_dist = std::min(safe_remain_dist, remain_dist);
  }

  const double stop_body_lat_buffer =
      real_time_brake_info_vec[0].static_col_det_buffer.body_lat_buffer;

  JSON_DEBUG_VALUE("car_real_time_col_lat_buffer", stop_body_lat_buffer)

  ILOG_INFO << "real time brake safe_remain_dist = " << safe_remain_dist
            << "  lon_buffer = " << lon_buffer << "  lat_buffer = "
            << real_time_brake_info_vec[0].static_col_det_buffer.body_lat_buffer
            << "  increase_lat_err_flag = " << increase_lat_err_flag;

  ILOG_INFO << "real time brake time cost = "
            << (IflyTime::Now_ms() - start_time);

  return safe_remain_dist;
}

void PerpendicularTailInScenario::CalSlotJumpErr() {
  if (frame_.is_replan_first) {
    return;
  }
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const ApaParameters& param = apa_param.GetParam();

  const geometry_lib::PathPoint& real_time_pose =
      ego_info_under_slot.origin_target_pose;

  const geometry_lib::PathPoint front_real_time_pose =
      GetCarFrontPoseFromCarPose(real_time_pose);

  geometry_lib::PathPoint last_time_pose =
      ego_info_under_slot.replan_success_origin_target_pose;

  last_time_pose.GlobalToLocal(ego_info_under_slot.g2l_tf);
  const geometry_lib::PathPoint front_last_time_pose =
      GetCarFrontPoseFromCarPose(last_time_pose);

  const double rear_lat_err =
      std::fabs(real_time_pose.GetY() - last_time_pose.GetY());

  const double front_lat_err =
      std::fabs(front_real_time_pose.GetY() - front_last_time_pose.GetY());
  const double lat_err = std::max(rear_lat_err, front_lat_err);
  const double lon_err =
      std::fabs(real_time_pose.GetX() - last_time_pose.GetX());

  const double heading_err =
      std::fabs(geometry_lib::NormalizeAngle(real_time_pose.heading -
                                             last_time_pose.heading)) *
      kRad2Deg;

  frame_.slot_jump_lat_err = lat_err;
  frame_.slot_jump_lon_err = lon_err;
  frame_.slot_jump_heading_err = heading_err;

  const geometry_lib::PathPoint& terminal_err =
      ego_info_under_slot.terminal_err;

  double terminal_error_y = 16.8;
  if (ego_info_under_slot.slot_occupied_ratio > 0.168 &&
      std::fabs(terminal_err.heading) * kRad2Deg < 6.8) {
    terminal_error_y = std::fabs(terminal_err.GetY());
  }

  const double scale_factor =
      ego_info_under_slot.slot.slot_source_type_ == SlotSourceType::USS ? 3.0
                                                                        : 1.0;

  const bool lat_jump_big = std::min(lat_err, terminal_error_y) >
                            param.slot_jump_lat_big_err * scale_factor;
  const bool heading_jump_big =
      heading_err > param.slot_jump_heading_big_err * scale_factor;
  frame_.slot_jump_big_flag = lat_jump_big || heading_jump_big;

  ILOG_INFO << "lat_err = " << lat_err << "  lon_err = " << lon_err
            << "  heading_err = " << heading_err
            << "  slot_jump_big_flag = " << frame_.slot_jump_big_flag;

  JSON_DEBUG_VALUE("slot_lat_err", lat_err)
  JSON_DEBUG_VALUE("slot_heading_err", heading_err)
  JSON_DEBUG_VALUE("slot_lon_err", lon_err)
  JSON_DEBUG_VALUE("slot_jump_big_flag", frame_.slot_jump_big_flag)
}

void PerpendicularTailInScenario::UpdateRemainDist() {
  frame_.remain_dist_path = CalRemainDistFromPath();
  frame_.remain_dist_obs = CalRealTimeBrakeDist();
  frame_.remain_dist_slot_jump = CalRemainDistBySlotJump();
  frame_.remain_dist_col_det = CalRemainDistFromPlanPathDangerous(
      0.0, 0.0, 0.0, apa_param.GetParam().use_obs_height_method);
}

const double PerpendicularTailInScenario::CalRemainDistBySlotJump() {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const uint8_t ref_gear =
      scenario_type_ == ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN
          ? geometry_lib::SEG_GEAR_REVERSE
          : geometry_lib::SEG_GEAR_DRIVE;

  const bool should_skip_slot_jump_stop =
      !frame_.is_last_path || !frame_.slot_jump_big_flag ||
      frame_.mirror_command != MirrorCommand::NONE ||
      frame_.gear_command != ref_gear ||
      current_path_point_global_vec_.empty() ||
      ego_info_under_slot.slot_occupied_ratio < 0.168 ||
      (ego_info_under_slot.slot_occupied_ratio > 0.708 &&
       !frame_.ego_should_stop_by_slot_jump);

  if (should_skip_slot_jump_stop) {
    frame_.car_already_move_dist = 0.0;
    frame_.ego_should_stop_by_slot_jump = false;
    return 5.01;
  }

  const double car_already_move_dist_last =
      frame_.current_path_length - frame_.remain_dist_path_last;

  const double car_already_move_dist =
      frame_.current_path_length - frame_.remain_dist_path;

  if (!frame_.ego_should_stop_by_slot_jump) {
    double lon_stop_dist = 0.0, heading_stop_dist = 0.0;
    const double x_err = 1.68, heading_err = 12.68 * kDeg2Rad;
    const double target_x = ego_info_under_slot.target_pose.GetX();
    const double target_heading = ego_info_under_slot.target_pose.heading;
    geometry_lib::PathPoint path_point_local;
    for (const auto& path_point_global : current_path_point_global_vec_) {
      path_point_local.pos =
          ego_info_under_slot.g2l_tf.GetPos(path_point_global.pos);

      path_point_local.heading =
          ego_info_under_slot.g2l_tf.GetHeading(path_point_global.heading);

      if (path_point_local.GetX() - target_x > x_err) {
        lon_stop_dist = path_point_global.s;
      }

      if (std::fabs(geometry_lib::AngleSubtraction(
              path_point_local.heading, target_heading)) > heading_err) {
        heading_stop_dist = path_point_global.s;
      }
    }

    ILOG_INFO << "lon_stop_dist = " << lon_stop_dist
              << "  heading_stop_dist = " << heading_stop_dist;

    const double remain_lon_stop_dist = lon_stop_dist - car_already_move_dist;
    const double remain_heading_stop_dist =
        heading_stop_dist - car_already_move_dist;

    frame_.ego_should_stop_dist_by_slot_jump = std::max(
        std::min(remain_lon_stop_dist, remain_heading_stop_dist), 1.268);
  }

  const double remain_dist_slot_jump =
      frame_.ego_should_stop_dist_by_slot_jump - frame_.car_already_move_dist;

  frame_.car_already_move_dist +=
      std::fabs(car_already_move_dist - car_already_move_dist_last);

  ILOG_INFO
      << "should stop because of the slot jump much, remain_dist_slot_jump = "
      << remain_dist_slot_jump
      << "  ego_stop_dist = " << frame_.ego_should_stop_dist_by_slot_jump
      << "  car_already_move_dist = " << car_already_move_dist
      << "  car_already_move_dist_last = " << car_already_move_dist_last
      << "  frame_.car_already_move_dist = " << frame_.car_already_move_dist;

  frame_.ego_should_stop_by_slot_jump = true;

  return remain_dist_slot_jump;
}

const bool PerpendicularTailInScenario::PostProcessPathAccordingRemainDist(
    const double remain_dist) {
  size_t origin_traj_size = current_path_point_global_vec_.size();

  if (origin_traj_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: origin_traj_size = " << origin_traj_size;
    return false;
  }

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
  std::vector<double> heading_vec;
  x_vec.clear();
  y_vec.clear();
  s_vec.clear();
  heading_vec.clear();

  x_vec.reserve(origin_traj_size);
  y_vec.reserve(origin_traj_size);
  s_vec.reserve(origin_traj_size);
  heading_vec.reserve(origin_traj_size);

  double ds = 0.0;
  double s = 0.0;
  for (size_t i = 0; i < origin_traj_size; ++i) {
    if (i > 0) {
      ds = std::hypot(current_path_point_global_vec_[i].pos.x() -
                          current_path_point_global_vec_[i - 1].pos.x(),
                      current_path_point_global_vec_[i].pos.y() -
                          current_path_point_global_vec_[i - 1].pos.y());
      s += std::max(ds, 1e-3);
    }
    if (s > remain_dist) {
      ILOG_INFO << "path shoule be shorten";
      if (remain_dist - s_vec.back() > 0.036 && frame_.spline_success) {
        x_vec.emplace_back(frame_.x_s_spline(remain_dist));
        y_vec.emplace_back(frame_.y_s_spline(remain_dist));
        heading_vec.emplace_back(heading_vec.back());
        s_vec.emplace_back(remain_dist);
      }
      break;
    }
    x_vec.emplace_back(current_path_point_global_vec_[i].pos.x());
    y_vec.emplace_back(current_path_point_global_vec_[i].pos.y());
    heading_vec.emplace_back(current_path_point_global_vec_[i].heading);
    s_vec.emplace_back(s);
  }

  frame_.current_path_length = s_vec.back();
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
  bool success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
      Eigen::Vector2d(x_vec[N - 2], y_vec[N - 2]),
      Eigen::Vector2d(x_vec[N - 1], y_vec[N - 1]), extended_point,
      frame_.path_extended_dist);

  if (!success) {
    frame_.spline_success = false;
    ILOG_INFO << "limit need extend fit line by spline error!";
    return false;
  }

  x_vec.emplace_back(extended_point.x());
  y_vec.emplace_back(extended_point.y());
  heading_vec.emplace_back(heading_vec.back());
  s_vec.emplace_back(frame_.current_path_length + frame_.path_extended_dist);

  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;

  return true;
}

const bool
PerpendicularTailInScenario::CheckDynamicPlanPathOptimalByGeometryPath() {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  geometry_lib::GeometryPath old_geometry_path(all_plan_path_vec_);
  old_geometry_path.GlobalToLocal(ego_info_under_slot.g2l_tf);

  const geometry_lib::GeometryPath new_geometry_path(
      apa_world_ptr_->GetParkingTaskInterfacePtr()
          ->GetPerpendicularTailInPathGeneratorPtr()
          ->GetOutput()
          .path_segment_vec);

  if (complete_path_point_global_vec_.empty()) {
    return true;
  }

  if (new_geometry_path.cur_gear != geometry_lib::SEG_GEAR_REVERSE ||
      new_geometry_path.gear_change_count > 0) {
    return false;
  }

  double final_line_length = 0.0;
  for (int i = new_geometry_path.path_segment_vec.size() - 1; i >= 0; i--) {
    const geometry_lib::PathSegment& seg =
        new_geometry_path.path_segment_vec[i];
    if (seg.seg_steer == geometry_lib::SEG_STEER_STRAIGHT) {
      final_line_length += seg.GetLength();
    } else {
      break;
    }
  }

  double first_pt_kappa = 0.0;
  std::vector<geometry_lib::PathPoint> s_turn_path;
  s_turn_path.reserve(35);
  if (new_geometry_path.IsHasSTurnPath()) {
    const std::vector<geometry_lib::PathSegment>& segments =
        new_geometry_path.path_segment_vec;

    std::vector<geometry_lib::PathSegment> s_turn_segs;

    int last_added_idx = -1;
    for (size_t i = 0; i < new_geometry_path.path_count - 1; ++i) {
      if (i == 0) {
        if (segments[i].seg_steer == geometry_lib::SEG_STEER_STRAIGHT) {
          first_pt_kappa = 0.0;
        } else if (segments[i].seg_steer == geometry_lib::SEG_STEER_LEFT) {
          first_pt_kappa = 1.0 / std::max(segments[i].GetRadius(), 0.01);
        } else if (segments[i].seg_steer == geometry_lib::SEG_STEER_RIGHT) {
          first_pt_kappa = -1.0 / std::max(segments[i].GetRadius(), 0.01);
        }
      }
      if (IsSTrunPath(segments[i], segments[i + 1])) {
        if (last_added_idx != static_cast<int>(i)) {
          s_turn_segs.emplace_back(segments[i]);
        }
        s_turn_segs.emplace_back(segments[i + 1]);
        last_added_idx = static_cast<int>(i + 1);
      }
    }

    s_turn_path = geometry_lib::SamplePathSegVec(s_turn_segs, 0.1);
  }

  return CheckDynamicPlanPathOptimal(
      old_geometry_path.gear_change_count, new_geometry_path.gear_change_count,
      final_line_length, first_pt_kappa, s_turn_path,
      old_geometry_path.end_pose, new_geometry_path.end_pose,
      ego_info_under_slot.target_pose);
}

const bool
PerpendicularTailInScenario::CheckDynamicPlanPathOptimalByHybridAstarPath(
    const HybridAstarResponse& response) {
  const HybridAStarResult& res = response.result;
  const HybridAStarRequest& req = response.request;
  const EgoInfoUnderSlot& ego_info_under_slot = req.ego_info_under_slot;

  if (complete_path_point_global_vec_.empty()) {
    return true;
  }

  const AstarPathGear expected_gear =
      (scenario_type_ == ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN)
          ? AstarPathGear::REVERSE
          : AstarPathGear::DRIVE;
  const bool has_valid_plan = res.path_plan_success;
  const bool has_no_gear_change = res.gear_change_num == 0;
  const bool has_valid_path =
      !res.kappa_vec_vec.empty() && !res.kappa_vec_vec.back().empty();
  const bool gear_matches = res.cur_gear == expected_gear;
  if (!has_valid_plan || !has_no_gear_change || !has_valid_path ||
      !gear_matches) {
    return false;
  }

  const auto& x_vec = res.x_vec_vec.back();
  const auto& y_vec = res.y_vec_vec.back();
  const auto& phi_vec = res.phi_vec_vec.back();
  const auto& kappa_vec = res.kappa_vec_vec.back();

  const double first_pt_kappa = kappa_vec.front();

  std::vector<pnc::geometry_lib::PathPoint> s_turn_path;
  float final_line_length = 0.0;
  ExtractSTurnAndStraight(x_vec, y_vec, phi_vec, kappa_vec, req.sample_ds,
                          s_turn_path, final_line_length);

  geometry_lib::PathPoint new_tar_pose;
  new_tar_pose.pos << x_vec.back(), y_vec.back();
  new_tar_pose.heading = phi_vec.back();

  geometry_lib::PathPoint old_tar_pose = complete_path_point_global_vec_.back();
  old_tar_pose.GlobalToLocal(ego_info_under_slot.g2l_tf);

  return CheckDynamicPlanPathOptimal(
      hybrid_astar_response_.result.gear_change_num, res.gear_change_num,
      final_line_length, first_pt_kappa, s_turn_path, old_tar_pose,
      new_tar_pose, ego_info_under_slot.target_pose);
}

const bool PerpendicularTailInScenario::CheckDynamicPlanPathOptimal(
    const size_t old_path_gear_change_count,
    const size_t new_path_gear_change_count,
    const double new_path_final_line_length, const double first_pt_kappa,
    const std::vector<geometry_lib::PathPoint>& s_turn_path,
    const geometry_lib::PathPoint& old_tar_pose,
    const geometry_lib::PathPoint& new_tar_pose,
    const geometry_lib::PathPoint& real_tar_pose) {
  const ApaParameters& param = apa_param.GetParam();
  const CheckFinishParams& finish_params = param.check_finish_params;

  constexpr double kMinFinalLineLength = 0.268;
  constexpr double kExtraSteerAllowanceDeg = 1.68;
  constexpr double kSTurnColDetStep = 0.2;
  constexpr double kReachTargetLonBuffer = 0.068;
  constexpr double kLatAllowErr = 0.04;
  constexpr double kFrontLatAllowErr = 0.06;
  constexpr double kHalfGain = 0.5;
  const std::vector<double> lat_err_breakpoints{0.01, 0.03, 0.05,
                                                0.07, 0.09, 0.10};
  const std::vector<double> min_line_length_table{1.3, 1.1, 0.9,
                                                  0.7, 0.5, 0.368};

  if (new_path_final_line_length < kMinFinalLineLength) {
    ILOG_INFO << "new path final line length is too small";
    return false;
  }

  const std::vector<double> sturn_lat_err_breakpoints{0.05, 0.10, 0.15, 0.20};
  const std::vector<double> sturn_min_line_length_table{0.3, 0.7, 1.1, 1.5};

  const bool new_path_has_sturn = !s_turn_path.empty();
  if (old_path_gear_change_count > 0 && !new_path_has_sturn) {
    ILOG_INFO << "old path has gear change while new path has no s turn";
    return true;
  }

  if (new_path_has_sturn) {
    const double cur_steer =
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetSteerWheelAngle() *
        kRad2Deg;
    const double new_steer = std::atan(param.wheel_base * first_pt_kappa) *
                             param.steer_ratio * kRad2Deg;
    const double steer_diff = std::fabs(cur_steer - new_steer);
    const double max_allowed_steer_diff =
        param.max_steer_angle_deg + kExtraSteerAllowanceDeg;
    if (steer_diff > max_allowed_steer_diff) {
      ILOG_INFO << "new path steer diff is too large for s turn";
      return false;
    }
  }

  struct PathPoseError {
    double lat_err = 0.0;
    double front_lat_err = 0.0;
    double lon_err = 0.0;
    double theta_err = 0.0;
  };

  const auto calc_path_pose_error =
      [&](const geometry_lib::PathPoint& tar_pose) -> PathPoseError {
    const geometry_lib::PathPoint front_pose =
        GetCarFrontPoseFromCarPose(tar_pose);

    PathPoseError path_pose_error;
    path_pose_error.lat_err = tar_pose.GetY() - real_tar_pose.GetY();
    path_pose_error.front_lat_err = front_pose.GetY() - real_tar_pose.GetY();
    path_pose_error.lon_err = tar_pose.GetX() - real_tar_pose.GetX();
    path_pose_error.theta_err =
        (tar_pose.GetTheta() - real_tar_pose.GetTheta()) * kRad2Deg;
    return path_pose_error;
  };

  const PathPoseError old_err = calc_path_pose_error(old_tar_pose);
  const PathPoseError new_err = calc_path_pose_error(new_tar_pose);

  ILOG_INFO << "old_lat_err = " << old_err.lat_err
            << "  old_front_lat_err = " << old_err.front_lat_err
            << "  old_lon_err = " << old_err.lon_err
            << "  old_theta_err = " << old_err.theta_err
            << "  new_lat_err = " << new_err.lat_err
            << "  new_front_lat_err = " << new_err.front_lat_err
            << "  new_lon_err = " << new_err.lon_err
            << "  new_theta_err = " << new_err.theta_err;

  if (new_path_has_sturn) {
    const double old_max_lat_err =
        std::max(std::fabs(old_err.lat_err), std::fabs(old_err.front_lat_err));
    const double min_sturn_final_line_length =
        mathlib::Interp1(sturn_lat_err_breakpoints, sturn_min_line_length_table,
                         old_max_lat_err);
    if (new_path_final_line_length < min_sturn_final_line_length) {
      ILOG_INFO
          << "old path lat err is larger while new path final line length "
             "is relatively small";
      return false;
    }

    const auto& gjk_col_det_ptr =
        apa_world_ptr_->GetColDetInterfacePtr()->GetGJKColDetPtr();
    GJKColDetRequest gjk_col_det_req(
        true, param.uss_config.use_uss_pt_cloud, CarBodyType::NORMAL,
        ApaObsMovementType::ALL, param.use_obs_height_method, true);
    if (gjk_col_det_ptr
            ->Update(s_turn_path, ColDetBuffer(0.0, kSTurnColDetStep),
                     gjk_col_det_req)
            .col_flag) {
      ILOG_INFO << "s turn path is not safe enough";
      return false;
    }
  }

  if (old_err.lon_err > param.car_to_limiter_dis - kReachTargetLonBuffer &&
      geometry_lib::IsTwoNumerEqual(new_err.lon_err, 0.0)) {
    ILOG_INFO << "old path does not reach target while new path reaches target";
    return true;
  }

  const auto check_path_meet_finish = [&](const CarSlotRelationship ship,
                                          const PathPoseError& path_err) {
    if (ship == CarSlotRelationship::TOUCHING) {
      return false;
    }
    const double finish_lat_err =
        (ship == CarSlotRelationship::IDEAL ? finish_params.lat_err
                                            : finish_params.lat_err_strict);
    const double finish_heading_err =
        (ship == CarSlotRelationship::IDEAL ? finish_params.heading_err
                                            : finish_params.heading_err_strict);
    return std::fabs(path_err.lat_err) < finish_lat_err &&
           std::fabs(path_err.front_lat_err) < finish_lat_err &&
           std::fabs(path_err.theta_err) < finish_heading_err;
  };

  const CarSlotRelationship old_ship = CalCarSlotRelationship(old_tar_pose);
  const CarSlotRelationship new_ship = CalCarSlotRelationship(new_tar_pose);
  const bool old_path_meet_finish = check_path_meet_finish(old_ship, old_err);
  const bool new_path_meet_finish = check_path_meet_finish(new_ship, new_err);

  ILOG_INFO << "check dynamic path optimal, old_ship = "
            << static_cast<int>(old_ship)
            << "  new_ship = " << static_cast<int>(new_ship)
            << "  old_path_meet_finish = " << old_path_meet_finish
            << "  new_path_meet_finish = " << new_path_meet_finish;

  if (!old_path_meet_finish && new_path_meet_finish) {
    return true;
  }

  if (old_path_meet_finish && !new_path_meet_finish) {
    return false;
  }

  const double old_max_lat_err =
      std::max(std::fabs(old_err.lat_err), std::fabs(old_err.front_lat_err));
  const double min_line_length = mathlib::Interp1(
      lat_err_breakpoints, min_line_length_table, old_max_lat_err);
  if (new_path_final_line_length < min_line_length) {
    ILOG_INFO << "new path final line length is too short for current error";
    return false;
  }

  const double old_lat_err_abs = std::fabs(old_err.lat_err);
  const double old_front_lat_err_abs = std::fabs(old_err.front_lat_err);
  const double new_lat_err_abs = std::fabs(new_err.lat_err);
  const double new_front_lat_err_abs = std::fabs(new_err.front_lat_err);
  const bool new_path_worse_on_both_lat_metrics =
      new_lat_err_abs > old_lat_err_abs &&
      new_front_lat_err_abs > old_front_lat_err_abs;
  if (new_path_worse_on_both_lat_metrics) {
    ILOG_INFO << "new path is worse on both lat metrics";
    return false;
  }

  const bool new_path_clearly_better =
      old_lat_err_abs > new_lat_err_abs + kLatAllowErr ||
      old_front_lat_err_abs > new_front_lat_err_abs + kFrontLatAllowErr;
  if (new_path_clearly_better) {
    ILOG_INFO << "new path is clearly better on lat metrics";
    return true;
  }

  const bool lat_err_is_close =
      old_lat_err_abs < new_lat_err_abs + kLatAllowErr;
  const bool front_lat_err_is_close =
      old_front_lat_err_abs < new_front_lat_err_abs + kFrontLatAllowErr;
  if (lat_err_is_close && front_lat_err_is_close) {
    if (new_path_has_sturn) {
      ILOG_INFO
          << "lat metrics are close, prefer old path without extra s turn";
      return false;
    }

    const bool new_path_slightly_better_without_sturn =
        old_lat_err_abs > new_lat_err_abs + kLatAllowErr * kHalfGain ||
        old_front_lat_err_abs >
            new_front_lat_err_abs + kFrontLatAllowErr * kHalfGain;
    if (new_path_slightly_better_without_sturn) {
      ILOG_INFO << "lat metrics are close and new path is slightly better";
    }
    return new_path_slightly_better_without_sturn;
  }

  return false;
}

const bool PerpendicularTailInScenario::LateralPathOptimize(
    std::vector<geometry_lib::PathPoint>& optimal_path_vec) {
  const GeometryPathOutput& path_plan_output =
      apa_world_ptr_->GetParkingTaskInterfacePtr()
          ->GetPerpendicularTailInPathGeneratorPtr()
          ->GetOutput();
  const double sample_ds = path_plan_output.actual_ds;
  const std::vector<pnc::geometry_lib::PathPoint>& pt_vec =
      path_plan_output.path_point_vec;
  const double cur_gear_length = path_plan_output.cur_gear_length;
  const SimulationParam& simu_param = apa_world_ptr_->GetSimuParam();
  if (pt_vec.size() < 3 ||
      cur_gear_length < apa_param.GetParam().min_opt_path_length) {
    return false;
  }
  bool cilqr_optimization_enable = true;
  bool perpendicular_optimization_enable = true;
  if (!simu_param.is_simulation) {
    perpendicular_optimization_enable =
        apa_param.GetParam().perpendicular_lat_opt_enable;

    cilqr_optimization_enable =
        apa_param.GetParam().cilqr_path_optimization_enable;
  } else {
    perpendicular_optimization_enable = simu_param.is_path_optimization;
    cilqr_optimization_enable = simu_param.is_cilqr_optimization;
  }

  if (!perpendicular_optimization_enable) {
    return false;
  }

  LateralPathOptimizer::Parameter param;
  param.sample_ds = 0.01;
  if (simu_param.is_simulation) {
    param.q_ref_xy = simu_param.q_ref_xy;
    param.q_ref_theta = simu_param.q_ref_theta;
    param.q_terminal_xy = simu_param.q_terminal_xy;
    param.q_terminal_theta = simu_param.q_terminal_theta;
    param.q_k = simu_param.q_k;
    param.q_u = simu_param.q_u;
    param.q_k_bound = simu_param.q_k_bound;
    param.q_u_bound = simu_param.q_u_bound;
  } else {
    param.q_ref_xy = 50.0;
    param.q_ref_theta = 50.0;
    param.q_terminal_xy = 5000.0;
    param.q_terminal_theta = 168000.0;
    param.q_k = 5.0;
    param.q_u = 5.0;
    param.q_k_bound = 100.0;
    param.q_u_bound = 100.0;
  }

  const double start_time = IflyTime::Now_ms();

  apa_world_ptr_->GetLateralPathOptimizerPtr()->Init(cilqr_optimization_enable);
  apa_world_ptr_->GetLateralPathOptimizerPtr()->SetParam(param);
  apa_world_ptr_->GetLateralPathOptimizerPtr()->Update(pt_vec,
                                                       frame_.gear_command);

  ILOG_INFO << "lat_path_opt_cost_time_ms = " << IflyTime::Now_ms() - start_time
            << " ms";

  const std::vector<pnc::geometry_lib::PathPoint>& origin_optimized_path_vec =
      apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOriginOutputPathVec();

  if (origin_optimized_path_vec.size() < 3) {
    ILOG_INFO << "origin_optimized_path_vec.size() < 3";
    return false;
  }

  const size_t max_pt_number =
      PLANNING_TRAJ_POINTS_MAX_NUM - APA_COMPARE_PLANNING_TRAJ_POINTS_MAX_NUM;
  optimal_path_vec.clear();
  optimal_path_vec.reserve(max_pt_number + 2);
  if (origin_optimized_path_vec.size() <= max_pt_number) {
    optimal_path_vec = origin_optimized_path_vec;
  } else {
    const double length = origin_optimized_path_vec.back().s;
    const double resampled_ds = length / max_pt_number;
    std::vector<double> x_vec;
    std::vector<double> y_vec;
    std::vector<double> s_vec;
    std::vector<double> heading_vec;
    std::vector<double> kappa_vec;
    x_vec.reserve(origin_optimized_path_vec.size());
    y_vec.reserve(origin_optimized_path_vec.size());
    s_vec.reserve(origin_optimized_path_vec.size());
    heading_vec.reserve(origin_optimized_path_vec.size());
    kappa_vec.reserve(origin_optimized_path_vec.size());
    for (const auto& pt : origin_optimized_path_vec) {
      x_vec.emplace_back(pt.pos.x());
      y_vec.emplace_back(pt.pos.y());
      s_vec.emplace_back(pt.s);
      heading_vec.emplace_back(pt.heading);
      kappa_vec.emplace_back(pt.kappa);
    }
    mathlib::spline x_s_spline;
    mathlib::spline y_s_spline;
    mathlib::spline heading_s_spline;
    mathlib::spline kappa_s_spline;
    x_s_spline.set_points(s_vec, x_vec);
    y_s_spline.set_points(s_vec, y_vec);
    heading_s_spline.set_points(s_vec, heading_vec);
    kappa_s_spline.set_points(s_vec, kappa_vec);
    geometry_lib::PathPoint tmp_pt;
    double ds = 0.0;
    for (size_t i = 1; i < max_pt_number; ++i) {
      tmp_pt.pos << x_s_spline(ds), y_s_spline(ds);
      tmp_pt.heading = heading_s_spline(ds);
      tmp_pt.s = ds;
      tmp_pt.kappa = kappa_s_spline(ds);
      ds += resampled_ds;
      if (optimal_path_vec.size() > 0) {
        const double dist_err =
            (tmp_pt.pos - optimal_path_vec.back().pos).norm();
        const double heading_err =
            geometry_lib::NormalizeAngle(tmp_pt.heading -
                                         optimal_path_vec.back().heading) *
            kRad2Deg;
        if (dist_err < 0.01 || std::fabs(heading_err) > 36.8) {
          continue;
        }
      }
      optimal_path_vec.emplace_back(tmp_pt);
    }

    if (optimal_path_vec.size() > 0) {
      tmp_pt = optimal_path_vec.back();
      if (s_vec.back() - optimal_path_vec.back().s > 1e-2) {
        tmp_pt.pos << x_s_spline(s_vec.back()), y_s_spline(s_vec.back());
        tmp_pt.heading = heading_s_spline(s_vec.back());
        tmp_pt.s = s_vec.back();
        tmp_pt.kappa = kappa_s_spline(s_vec.back());
        optimal_path_vec.emplace_back(tmp_pt);
      }
    }
  }

  if (optimal_path_vec.size() < 3) {
    ILOG_INFO << "optimized_path_vec.size() < 3";
    return false;
  }

  if (std::fabs(cur_gear_length - optimal_path_vec.back().s) > 5e-2) {
    ILOG_INFO << "length is not the same, cur_gear_length = " << cur_gear_length
              << "  optimized_path length = " << optimal_path_vec.back().s;
    return false;
  }

  if (!pnc::geometry_lib::CheckTwoPoseIsSame(
          optimal_path_vec.front(), pt_vec.front(), 0.02, 0.3 / 57.3) ||
      !pnc::geometry_lib::CheckTwoPoseIsSame(optimal_path_vec.back(),
                                             pt_vec.back(), 0.02, 0.3 / 57.3)) {
    ILOG_INFO << "start or end pose is not same";
    return false;
  }

  if (apa_world_ptr_->GetColDetInterfacePtr()
          ->GetGJKColDetPtr()
          ->Update(optimal_path_vec, ColDetBuffer(0.0, 0.08),
                   GJKColDetRequest())
          .col_flag) {
    ILOG_INFO << "the optimal path is col";
    return false;
  }

  ILOG_INFO << "optimal path success, pos_err = "
            << (pt_vec.back().pos - optimal_path_vec.back().pos).norm()
            << "  heading_err = "
            << (pt_vec.back().heading - optimal_path_vec.back().heading) *
                   kRad2Deg;

  return true;
}

const PerpendicularTailInScenario::SlotObsType
PerpendicularTailInScenario::CalSlotObsType(const Eigen::Vector2d& obs_slot) {
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const std::vector<double> slot_width_tab{2.2, 2.4, 2.6, 2.8, 3.0, 3.2};
  const std::vector<double> invasion_dist_tab{0.20, 0.25, 0.30, 0.35, 0.40};
  const double dy1 = mathlib::Interp1(slot_width_tab, invasion_dist_tab,
                                      ego_info_under_slot.slot.slot_width_);

  const double dy2 = 1.68;

  double dx1 = 3.468;
  dx1 = std::min(dx1, ego_info_under_slot.cur_pose.pos.x() -
                          apa_param.GetParam().car_width * 0.5 -
                          ego_info_under_slot.slot.slot_length_);

  dx1 = std::max(dx1, 0.368);

  double dx2 = 4.86 / 5.0 * ego_info_under_slot.slot.slot_length_;

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

  // Firstly, the default right side is the inner side, and the left side is
  // the outer side
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
    return SlotObsType::INSIDE_OBS;
  } else if (geometry_lib::IsPointInPolygon(outside_area, obs_slot)) {
    return SlotObsType::OUTSIDE_OBS;
  } else if (geometry_lib::IsPointInPolygon(in_area, obs_slot)) {
    return SlotObsType::IN_OBS;
  } else if (geometry_lib::IsPointInPolygon(discard_area, obs_slot)) {
    return SlotObsType::DISCARD_OBS;
  } else {
    return SlotObsType::OTHER_OBS;
  }
}

const bool PerpendicularTailInScenario::CheckDynamicUpdate() {
  const ApaParameters& param = apa_param.GetParam();
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const bool is_tail_in =
      scenario_type_ == ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN;

  const uint8_t expected_gear = is_tail_in ? geometry_lib::SEG_GEAR_REVERSE
                                           : geometry_lib::SEG_GEAR_DRIVE;

  if (frame_.mirror_command == MirrorCommand::FOLD ||
      frame_.gear_command != expected_gear ||
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag() ||
      ego_info_under_slot.confidence != 1) {
    frame_.dynamic_plan_time = 0.0;
    return false;
  }

  constexpr double kTailInMaxEgoXOffset = 2.68;
  constexpr double kMaxHeadingErrDeg = 60.0;
  constexpr double kObsRemainDistFactor = 2.5;

  const double max_ego_x_offset = is_tail_in
                                      ? kTailInMaxEgoXOffset
                                      : kTailInMaxEgoXOffset + param.wheel_base;
  const double ego_x = ego_info_under_slot.cur_pose.GetX();
  const double slot_mid_x =
      ego_info_under_slot.slot.GetOriginCornerCoordLocal().pt_01_mid.x();
  const double heading_err_deg =
      std::fabs(ego_info_under_slot.terminal_err.GetTheta()) * kRad2Deg;
  const bool ego_pose_out_of_range = ego_x >= slot_mid_x + max_ego_x_offset ||
                                     heading_err_deg >= kMaxHeadingErrDeg;
  if (ego_pose_out_of_range || ego_info_under_slot.slot_occupied_ratio >=
                                   param.pose_slot_occupied_ratio_3) {
    frame_.dynamic_plan_time = 0.0;
    return false;
  }

  const bool path_remain_dist_too_short =
      frame_.remain_dist_path <=
      param.gear_switch_config.dist_thresh_for_gear_switch_point;
  const bool obs_remain_dist_too_short =
      frame_.remain_dist_obs <= kObsRemainDistFactor * param.min_drive_dist;
  if (path_remain_dist_too_short || obs_remain_dist_too_short) {
    frame_.dynamic_plan_time = 0.0;
    return false;
  }

  frame_.dynamic_plan_time += param.plan_time;
  if (frame_.dynamic_plan_time <= param.dynamic_plan_interval_time) {
    return false;
  }

  frame_.dynamic_plan_time = 0.0;
  return true;
}

const bool PerpendicularTailInScenario::CheckPathDangerous() {
  const ApaParameters& param = apa_param.GetParam();
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  if (!param.enable_path_dangerous_replan) {
    return false;
  }

  if (param.park_path_plan_type == ParkPathPlanType::GEOMETRY) {
    return false;
  }

  if (frame_.mirror_command == MirrorCommand::FOLD) {
    return false;
  }

  if (apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
    return false;
  }

  if (frame_.remain_dist_col_det > frame_.current_path_length - 0.1) {
    return false;
  }

  if (std::min({frame_.remain_dist_path, frame_.remain_dist_obs,
                frame_.remain_dist_slot_jump}) < 3.86) {
    return false;
  }

  if (ego_info_under_slot.slot_occupied_ratio > 0.0) {
    return false;
  }

  return true;
}

const CarSlotRelationship PerpendicularTailInScenario::CalCarSlotRelationship(
    const geometry_lib::PathPoint& cur_pose) {
  const ApaParameters& params = apa_param.GetParam();
  const CheckFinishParams& finish_params = params.check_finish_params;
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const double heading_err =
      ego_info_under_slot.terminal_err.GetTheta() * kRad2Deg;

  if (std::fabs(heading_err) > finish_params.heading_err) {
    return CarSlotRelationship::TOUCHING;
  }

  const geometry_lib::PathPoint front_pose =
      GetCarFrontPoseFromCarPose(cur_pose);

  const std::vector<double> car_border_ys = {
      cur_pose.GetY() + 0.5 * params.car_width,
      cur_pose.GetY() - 0.5 * params.car_width,
      front_pose.GetY() + 0.5 * params.car_width,
      front_pose.GetY() - 0.5 * params.car_width};

  const double half_slot_width = 0.5 * ego_info_under_slot.slot.slot_width_;

  double min_car2line_dist = finish_params.min_car2line_dist;
  const double dw = half_slot_width - 0.5 * params.car_width;
  if (dw < 0.0) {
    min_car2line_dist += dw;
  } else if (dw < 0.03) {
    min_car2line_dist -= dw;
  }

  for (const double y : car_border_ys) {
    if (half_slot_width - std::fabs(y) < min_car2line_dist) {
      return CarSlotRelationship::TOUCHING;
    }
  }

  for (const double y : car_border_ys) {
    if (half_slot_width - std::fabs(y) < finish_params.max_car2line_dist) {
      return CarSlotRelationship::MARGINAL;
    }
  }

  return CarSlotRelationship::IDEAL;
}

void PerpendicularTailInScenario::DecideFoldMirrorCommand() {
  const ApaParameters& param = apa_param.GetParam();
  const SmartFoldMirrorParams& smart_fold_mirror_params =
      param.smart_fold_mirror_params;

  if (!smart_fold_mirror_params.has_smart_fold_mirror) {
    ILOG_INFO << "decide fold mirror, not enable smart fold mirror";
    return;
  }

  if (frame_.mirror_command != MirrorCommand::NONE) {
    ILOG_INFO << "decide fold mirror, mirror command is not none";
    return;
  }

  if (!frame_.is_last_path) {
    ILOG_INFO << "decide fold mirror, not last path";
    return;
  }

  const auto measure_data_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();
  if (measure_data_ptr->GetFoldMirrorFlag() ||
      measure_data_ptr->GetStaticFlag() || measure_data_ptr->GetBrakeFlag()) {
    ILOG_INFO
        << "decide fold mirror, fold mirror or static or brake flag is true";
    return;
  }

  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const bool is_geometry_plan =
      param.park_path_plan_type == ParkPathPlanType::GEOMETRY;

  constexpr double kExtraSafeBuffer = 0.015;
  constexpr double kMinConsumeDist = 0.068;
  constexpr double kPredictTrajReserve = 0.1;
  constexpr double kLatErrRelax = 0.0168;
  constexpr double kHeadingErrRelax = 0.168;

  const Eigen::Vector2d mirror_pos =
      ego_info_under_slot.cur_pose.pos +
      param.lon_dist_mirror_to_rear_axle *
          ego_info_under_slot.cur_pose.heading_vec;

  const Eigen::Vector2d& slot_pt_01_mid =
      ego_info_under_slot.slot.processed_corner_coord_local_.pt_01_mid;

  const double min_x = slot_pt_01_mid.x() +
                       smart_fold_mirror_params.x_down_offset -
                       smart_fold_mirror_params.x_redunant;

  const double max_x = slot_pt_01_mid.x() +
                       smart_fold_mirror_params.x_up_offset -
                       smart_fold_mirror_params.x_redunant;

  if (!mathlib::IsInBound(mirror_pos.x(), min_x, max_x)) {
    ILOG_INFO << "decide fold mirror, mirror is not in x range, x = "
              << mirror_pos.x() << "  min_x = " << min_x
              << "  max_x = " << max_x;
    return;
  }

  const geometry_lib::PathPoint& terminal_err =
      ego_info_under_slot.terminal_err;
  const double terminal_y_err = std::fabs(terminal_err.GetY());
  const double terminal_heading_err =
      std::fabs(terminal_err.GetTheta()) * kRad2Deg;

  if (terminal_y_err > smart_fold_mirror_params.y_offset ||
      terminal_heading_err > smart_fold_mirror_params.heading_offset) {
    ILOG_INFO
        << "decide fold mirror, terminal err is out of y/heading range, y = "
        << terminal_y_err << "  heading = " << terminal_heading_err
        << "  y_offset = " << smart_fold_mirror_params.y_offset
        << "  heading_offset = " << smart_fold_mirror_params.heading_offset;
    return;
  }

  geometry_lib::PathPoint old_tar_pose = complete_path_point_global_vec_.back();
  old_tar_pose.GlobalToLocal(ego_info_under_slot.g2l_tf);
  const geometry_lib::PathPoint& real_tar_pose =
      ego_info_under_slot.target_pose;
  const double target_pose_lat_err =
      std::fabs(old_tar_pose.GetY() - real_tar_pose.GetY());

  const double target_pose_heading_err =
      std::fabs(old_tar_pose.GetTheta() - real_tar_pose.GetTheta()) * kRad2Deg;

  const double target_pose_lat_threshold =
      param.check_finish_params.lat_err_strict - kLatErrRelax;

  const double target_pose_heading_threshold =
      param.check_finish_params.heading_err_strict - kHeadingErrRelax;

  if (target_pose_lat_err > target_pose_lat_threshold ||
      target_pose_heading_err > target_pose_heading_threshold) {
    ILOG_INFO << "decide fold mirror, target pose diff is out of range, lat = "
              << target_pose_lat_err
              << "  heading = " << target_pose_heading_err
              << "  lat_threshold = " << target_pose_lat_threshold
              << "  heading_threshold = " << target_pose_heading_threshold;
    return;
  }

  const double predict_traj_s =
      apa_world_ptr_->GetPredictPathManagerPtr()->GetPredictTrajS();

  const double vel = std::max(float(std::fabs(measure_data_ptr->GetVel())),
                              smart_fold_mirror_params.min_vel);

  const double remain_x_to_target = ego_info_under_slot.cur_pose.GetX() -
                                    ego_info_under_slot.target_pose.GetX() +
                                    kMinConsumeDist;

  const double dynamic_lon_buffer =
      param.lat_lon_speed_buffer.dynamic_lon_buffer + kExtraSafeBuffer;

  const double dynamic_stop_body_lat_buffer =
      param.lat_lon_speed_buffer.dynamic_stop_body_lat_buffer +
      kExtraSafeBuffer;

  const double base_stop_mirror_lat_buffer =
      (is_geometry_plan ? param.stop_lat_inflation
                        : param.lat_lon_speed_buffer.stop_mirror_lat_buffer) +
      kExtraSafeBuffer;

  const double folding_mirror_consume_dist = std::max(
      std::min({vel * smart_fold_mirror_params.consume_time,
                predict_traj_s - kPredictTrajReserve, remain_x_to_target}),
      kMinConsumeDist);

  if (CalRemainDistFromObs(
          ColDetBuffer(folding_mirror_consume_dist, base_stop_mirror_lat_buffer,
                       base_stop_mirror_lat_buffer),
          ColDetBuffer(dynamic_lon_buffer, dynamic_stop_body_lat_buffer,
                       dynamic_stop_body_lat_buffer),
          true, param.use_obs_height_method) < 0.0) {
    ILOG_INFO << "decide fold mirror, mirror is not safe when folding mirror, "
                 "should not fold mirror";
    return;
  }

  const double fold_mirror_reduce_width =
      0.5 * (param.max_car_width - param.fold_mirror_max_car_width);

  const double folded_mirror_lat_buffer =
      base_stop_mirror_lat_buffer - fold_mirror_reduce_width;

  const double folded_mirror_consume_dist = std::max(
      std::min({vel * (smart_fold_mirror_params.consume_time +
                       smart_fold_mirror_params.reaction_time),
                predict_traj_s - kPredictTrajReserve, remain_x_to_target}),
      kMinConsumeDist);

  if (CalRemainDistFromObs(
          ColDetBuffer(folded_mirror_consume_dist, folded_mirror_lat_buffer,
                       folded_mirror_lat_buffer),
          ColDetBuffer(dynamic_lon_buffer, dynamic_stop_body_lat_buffer,
                       dynamic_stop_body_lat_buffer),
          true, param.use_obs_height_method) < 0.0) {
    ILOG_INFO << "decide fold mirror, mirror is not safe even folded mirror, "
                 "should not fold mirror";
    return;
  }

  const double stop_body_lat_buffer =
      (is_geometry_plan ? param.stop_lat_inflation
                        : param.lat_lon_speed_buffer.stop_body_lat_buffer) +
      kExtraSafeBuffer;

  const double stop_body_lon_buffer =
      (is_geometry_plan ? param.safe_uss_remain_dist_in_slot
                        : param.lat_lon_speed_buffer.lon_buffer) +
      kExtraSafeBuffer;

  const double stop_body_safe_dist_threshold =
      frame_.remain_dist_path - param.check_finish_params.lon_err -
      kMinConsumeDist;

  if (CalRemainDistFromObs(
          ColDetBuffer(stop_body_lon_buffer, stop_body_lat_buffer,
                       folded_mirror_lat_buffer),
          ColDetBuffer(dynamic_lon_buffer, dynamic_stop_body_lat_buffer,
                       dynamic_stop_body_lat_buffer),
          false, param.use_obs_height_method) < stop_body_safe_dist_threshold) {
    ILOG_INFO << "decide fold mirror, mirror is not safe even folded mirror, "
                 "should not fold mirror";
    return;
  }

  const double min_safe_obs2mirror_dist = smart_fold_mirror_params.lat_buffer;
  if (CalRemainDistFromObs(
          ColDetBuffer(folded_mirror_consume_dist, min_safe_obs2mirror_dist,
                       min_safe_obs2mirror_dist),
          ColDetBuffer(dynamic_lon_buffer, dynamic_stop_body_lat_buffer,
                       dynamic_stop_body_lat_buffer),
          true, param.use_obs_height_method) < 0.0) {
    ILOG_INFO << "decide fold mirror, need send fold mirror msg";
    frame_.mirror_command = MirrorCommand::FOLD;
    return;
  }

  ILOG_INFO << "decide fold mirror, should not send fold mirror msg";

  return;
}

void PerpendicularTailInScenario::Log() const { return; }

}  // namespace apa_planner
}  // namespace planning