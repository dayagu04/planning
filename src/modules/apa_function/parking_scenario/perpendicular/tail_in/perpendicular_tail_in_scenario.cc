#include "perpendicular_tail_in_scenario.h"

#include <bits/stdint-uintn.h>

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <queue>

#include "Eigen/src/Core/util/Constants.h"
#include "apa_data.h"
#include "apa_param_config.h"
#include "apa_state_machine_manager.h"
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
#include "perpendicular_tail_in_path_generator.h"
#include "planning_plan_c.h"
#include "slot_management_info.pb.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

namespace planning {
namespace apa_planner {
void PerpendicularTailInScenario::Reset() {
  frame_.Reset();
  slot_t_lane_.Reset();
  obstacle_t_lane_.Reset();
  perpendicular_path_planner_.Reset();
  current_path_point_global_vec_.clear();
  current_plan_path_vec_.clear();

  pt_center_replan_.setZero();
  pt_center_heading_replan_ = 0.0;
  pt_center_replan_jump_dist_ = 0.0;
  pt_center_replan_jump_heading_ = 0.0;

  // reset planning output
  memset(&planning_output_, 0, sizeof(planning_output_));

  memset(&apa_hmi_, 0, sizeof(apa_hmi_));

  ParkingScenario::Reset();
}

void PerpendicularTailInScenario::ExcutePathPlanningTask() {
  // prepare simulation
  InitSimulation();

  // check planning status
  if (CheckPlanSkip()) {
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

  const double safe_uss_remain_dist =
      (frame_.ego_slot_info.slot_occupied_ratio < 0.05)
          ? apa_param.GetParam().safe_uss_remain_dist_out_slot
          : apa_param.GetParam().safe_uss_remain_dist_in_slot;

  // update remain dist
  UpdateRemainDist(safe_uss_remain_dist);

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

  // check replan
  if (CheckReplan()) {
    ILOG_INFO << "replan is required!";

    frame_.replan_flag = true;
    pt_center_replan_ = frame_.ego_slot_info.slot_center;
    pt_center_heading_replan_ = frame_.ego_slot_info.slot_origin_heading;
    frame_.dynamic_plan_fail_flag = false;

    const double start_time = IflyTime::Now_ms();

    GenTlane();

    GenObstacles();

    if (frame_.replan_reason != FORCE_PLAN && frame_.replan_reason != DYNAMIC) {
      frame_.total_plan_count++;
    }

    uint8_t pathplan_result = PathPlannerResult::PLAN_FAILED;

    if (frame_.total_plan_count <= apa_param.GetParam().max_replan_count) {
      if (apa_param.GetParam().new_itervative_solution) {
        pathplan_result = NewPathPlanOnce();
      } else {
        pathplan_result = PathPlanOnce();
      }
      ILOG_INFO << "generate path by geometry";
    } else {
      ILOG_INFO << "replan count is exceed max count, fail, directly quit apa";
      frame_.plan_fail_reason = PLAN_COUNT_EXCEED_LIMIT;
    }

    if (frame_.dynamic_plan_fail_flag ||
        pathplan_result == PathPlannerResult::PLAN_FAILED) {
      EgoSlotInfo& ego_slot = frame_.ego_slot_info;
      ego_slot.move_slot_dist = frame_.ego_slot_info.last_move_slot_dist;

      ego_slot.terminal_err.Set(
          ego_slot.ego_pos_slot -
              Eigen::Vector2d(
                  ego_slot.target_ego_pos_slot.x(),
                  ego_slot.target_ego_pos_slot.y() + ego_slot.move_slot_dist),
          ego_slot.ego_heading_slot - ego_slot.target_ego_heading_slot);
    } else {
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
      if (frame_.dynamic_plan_fail_flag) {
        SetParkingStatus(PARKING_PLANNING);
        ILOG_INFO << "replan from PARKING_PLANNING!";
      } else {
        if (PostProcessPath()) {
          SetParkingStatus(PARKING_PLANNING);
          ILOG_INFO << "replan from PARKING_PLANNING!";
        } else {
          SetParkingStatus(PARKING_FAILED);
          ILOG_INFO << "replan failed from PARKING_PLANNING!";
        }
      }
    } else if (pathplan_result == PathPlannerResult::PLAN_FAILED) {
      SetParkingStatus(PARKING_FAILED);
    }

    ILOG_INFO << "pathplan_result = " << static_cast<int>(pathplan_result);
  } else {
    ILOG_INFO << "replan is not required!";
    SetParkingStatus(PARKING_RUNNING);
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

  // check planning status
  ILOG_INFO << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status);
}

const bool PerpendicularTailInScenario::UpdateEgoSlotInfo() {
  const auto measures_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();
  const auto slot_manager_ptr_ = apa_world_ptr_->GetSlotManagerPtr();

  frame_.correct_path_for_limiter = false;
  frame_.replan_flag = false;

  EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;

  if (!frame_.is_fix_slot) {
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
    Eigen::Vector2d n = Eigen::Vector2d(t.y(), -t.x());
    n = (pM01 - pM23).normalized();
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
  }

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(measures_ptr->GetPos());

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measures_ptr->GetHeading());

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  ego_slot_info.fus_obj_valid_flag =
      slot_manager_ptr_->GetEgoSlotInfo().fus_obj_valid_flag;

  UpdateObstacleLocal();

  if (!ego_slot_info.fix_limiter) {
    ego_slot_info.limiter = slot_manager_ptr_->GetEgoSlotInfo().limiter;
  }

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

  // cal terminal error
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      ego_slot_info.ego_heading_slot - ego_slot_info.target_ego_heading_slot);

  // cal slot occupied ratio
  if (std::fabs(ego_slot_info.terminal_err.pos.y()) <
          apa_param.GetParam().slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_slot_info.ego_heading_slot) <
          apa_param.GetParam().slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    const std::vector<double> x_tab = {
        ego_slot_info.target_ego_pos_slot.x(),
        ego_slot_info.slot_length + apa_param.GetParam().rear_overhanging};

    const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
    ego_slot_info.slot_occupied_ratio = pnc::mathlib::Interp1(
        x_tab, occupied_ratio_tab, ego_slot_info.ego_pos_slot.x());
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  // calc slot side and init gear and init steer at first
  if (frame_.is_replan_first) {
    pt_center_replan_ = ego_slot_info.slot_center;
    pt_center_heading_replan_ = ego_slot_info.slot_origin_heading;
    frame_.car_already_move_dist = 0.0;

    const auto pM01 =
        0.5 * (ego_slot_info.slot_corner[0] + ego_slot_info.slot_corner[1]);
    const auto pM23 =
        0.5 * (ego_slot_info.slot_corner[2] + ego_slot_info.slot_corner[3]);
    Eigen::Vector2d ego_to_slot_center_vec =
        0.5 * (pM01 + pM23) - measures_ptr->GetPos();

    const double cross_ego_to_slot_center =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->GetHeadingVec(),
                                                ego_to_slot_center_vec);

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(
            measures_ptr->GetHeadingVec(),
            ego_slot_info.slot_origin_heading_vec);

    // judge slot side via slot center and heading
    // frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
      slot_t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      slot_t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else {
      slot_t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      ILOG_INFO << "calculate slot side error ";
      // return false;
    }
  }

  // construct real time obs
  GenTlane();
  GenObstacles();

  // real time dynamic col det
  frame_.remain_dist_col_det = 3.0;
  if (!apa_world_ptr_->GetApaDataPtr()->simu_param.sim_to_target &&
      !current_plan_path_vec_.empty()) {
    const double start_time = IflyTime::Now_ms();

    const double car_already_move_dist =
        frame_.current_path_length - frame_.remain_dist;

    // distance of vehicle motion in adjacent frames
    const double dmove_dist =
        car_already_move_dist - frame_.car_already_move_dist;

    ILOG_INFO << "car real move dist = " << car_already_move_dist
              << "  last frame car_already_move_dist = "
              << frame_.car_already_move_dist
              << "  this frame car_already_move_dist = " << dmove_dist;

    frame_.car_already_move_dist = car_already_move_dist;

    // trim path real time according to dmove_dist
    double length = 0.0;
    std::vector<size_t> trim_id_vec;
    std::vector<size_t> lose_id_vec;
    for (size_t i = 0; i < current_plan_path_vec_.size(); ++i) {
      const pnc::geometry_lib::PathSegment& path_seg_global =
          current_plan_path_vec_[i];
      length += path_seg_global.Getlength();
      if (dmove_dist < length - 1e-3) {
        trim_id_vec.emplace_back(i);
        break;
      } else if (dmove_dist > length - 1e-3 && dmove_dist < length + 1e-3) {
        lose_id_vec.emplace_back(i);
        break;
      } else {
        lose_id_vec.emplace_back(i);
      }
    }

    // first lose and then trim
    while (!lose_id_vec.empty()) {
      current_plan_path_vec_.erase(current_plan_path_vec_.begin() +
                                   lose_id_vec.front());
      lose_id_vec.erase(lose_id_vec.begin());
      for (size_t i = 0; i < lose_id_vec.size(); ++i) {
        if (lose_id_vec[i] > 0) {
          lose_id_vec[i]--;
        }
      }
      for (size_t i = 0; i < trim_id_vec.size(); ++i) {
        if (trim_id_vec[i] > 0) {
          trim_id_vec[i]--;
        }
      }
    }
    while (!trim_id_vec.empty()) {
      const size_t trim_id = trim_id_vec.front();
      trim_id_vec.erase(trim_id_vec.begin());
      for (size_t i = 0; i < trim_id_vec.size(); ++i) {
        if (trim_id_vec[i] > 0) {
          trim_id_vec[i]--;
        }
      }
      for (size_t i = 0; i < current_plan_path_vec_.size(); ++i) {
        if (i == trim_id) {
          const double save_path = length - dmove_dist;
          pnc::geometry_lib::CompletePathSeg(current_plan_path_vec_[i],
                                             save_path, false);
        }
      }
    }

    // only for debug
    std::vector<double> x_vec;
    std::vector<double> y_vec;
    std::vector<double> phi_vec;
    std::vector<pnc::geometry_lib::PathPoint> pt_vec;
    length = 0.0;
    for (const pnc::geometry_lib::PathSegment& path_seg_global :
         current_plan_path_vec_) {
      length += path_seg_global.Getlength();
      std::vector<pnc::geometry_lib::PathPoint> temp_pt_vec;
      pnc::geometry_lib::SamplePointSetInPathSeg(temp_pt_vec, path_seg_global,
                                                 0.02);
      pt_vec.insert(pt_vec.end(), temp_pt_vec.begin(), temp_pt_vec.end());
    }
    ILOG_INFO << "current_plan_path_vec_ length  = " << length;

    for (const pnc::geometry_lib::PathPoint& pt : pt_vec) {
      x_vec.emplace_back(pt.pos.x());
      y_vec.emplace_back(pt.pos.y());
      phi_vec.emplace_back(pt.heading);
    }

    JSON_DEBUG_VECTOR("col_det_path_x", x_vec, 3)
    JSON_DEBUG_VECTOR("col_det_path_y", y_vec, 3)
    JSON_DEBUG_VECTOR("col_det_path_phi", phi_vec, 3)

    // the dist that car can move
    double car_safe_move_dist = 0.0;
    // col det to current plan path
    // the start pt is ego pose
    for (const pnc::geometry_lib::PathSegment& path_seg_global :
         current_plan_path_vec_) {
      // this path is global, need to transform to local to col det
      pnc::geometry_lib::PathSegment path_seg_local = path_seg_global;
      path_seg_local.GlobalToLocal(ego_slot_info.g2l_tf);

      CollisionDetector::CollisionResult col_res =
          apa_world_ptr_->GetCollisionDetectorPtr()->UpdateByObsMap(
              path_seg_local,
              apa_param.GetParam().car_lat_inflation_dynamic_col,
              apa_param.GetParam().col_obs_safe_dist_strict);

      if (col_res.collision_flag) {
        // the path would col by obs
        car_safe_move_dist += col_res.remain_dist;
        break;
      } else {
        // the path would not col by obs
        car_safe_move_dist += col_res.remain_car_dist;
      }
    }

    if (apa_param.GetParam().dynamic_col_det_enable) {
      frame_.remain_dist_col_det = car_safe_move_dist;

      frame_.remain_dist_col_det =
          std::min(frame_.remain_dist_col_det, frame_.remain_dist);
    }

    ILOG_INFO << "remain_dist_col_det = " << frame_.remain_dist_col_det;

    ILOG_INFO << "dynamic_col_det_consume_time = "
              << IflyTime::Now_ms() - start_time << " ms";
    JSON_DEBUG_VALUE("dynamic_col_det_consume_time",
                     IflyTime::Now_ms() - start_time)
  }

  // trim or extend path according to limiter, only run once
  if (frame_.gear_command == pnc::geometry_lib::SEG_GEAR_REVERSE &&
      !ego_slot_info.fix_limiter) {
    const pnc::geometry_lib::LineSegment limiter_line(
        ego_slot_info.limiter.first, ego_slot_info.limiter.second);

    const double dist_ego_limiter = pnc::geometry_lib::CalPoint2LineDist(
        ego_slot_info.ego_pos_slot, limiter_line);

    ILOG_INFO << "dist_ego_limiter = " << dist_ego_limiter;

    if (dist_ego_limiter < apa_param.GetParam().car_to_limiter_dis) {
      ILOG_INFO << "should correct path according limiter";
      ego_slot_info.fix_limiter = true;
      frame_.is_fix_slot = true;
      PostProcessPathAccordingLimiter();
      frame_.correct_path_for_limiter = true;
    }
  }

  // update stuck by uss time
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      measures_ptr->GetStaticFlag() && !measures_ptr->GetBrakeFlag() &&
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_REAR) {
    frame_.stuck_uss_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_uss_time = 0.0;
  }

  // update stuck time
  if ((frame_.plan_stm.planning_status == PARKING_RUNNING ||
       frame_.plan_stm.planning_status == PARKING_PLANNING) &&
      measures_ptr->GetStaticFlag() && !measures_ptr->GetBrakeFlag() &&
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_REAR) {
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
      !frame_.is_fix_slot && measures_ptr->GetStaticFlag()) {
    frame_.is_fix_slot = true;
    ego_slot_info.fix_limiter = true;
  }

  ILOG_INFO << "slot_side = " << static_cast<int>(slot_t_lane_.slot_side);
  ILOG_INFO << "frame_.current_gear = "
            << static_cast<int>(frame_.current_gear);
  ILOG_INFO << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer);

  ILOG_INFO << "  ego_pos_slot = " << ego_slot_info.ego_pos_slot.transpose()
            << "  ego_heading_slot = "
            << ego_slot_info.ego_heading_slot * kRad2Deg;

  ILOG_INFO << "ego_slot_info.limiter.first = "
            << ego_slot_info.limiter.first.transpose()
            << "  ego_slot_info.limiter.second = "
            << ego_slot_info.limiter.second.transpose();

  ILOG_INFO << "target_ego_pos_slot = "
            << ego_slot_info.target_ego_pos_slot.transpose()
            << "  target_ego_heading_slot = "
            << ego_slot_info.target_ego_heading_slot * kRad2Deg;

  ILOG_INFO << "terminal x error= " << ego_slot_info.terminal_err.pos.x();
  ILOG_INFO << "terminal y error= " << ego_slot_info.terminal_err.pos.y();
  ILOG_INFO << "terminal heading error= "
            << ego_slot_info.terminal_err.heading * kRad2Deg;

  ILOG_INFO << "slot_occupied_ratio = " << ego_slot_info.slot_occupied_ratio;

  ILOG_INFO << "stuck_time = " << frame_.stuck_time << " s";

  return true;
}

void PerpendicularTailInScenario::GenTlane() {
  using namespace pnc::geometry_lib;
  EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;

  const Eigen::Vector2d pt_01_norm_vec =
      (ego_slot_info.pt_1 - ego_slot_info.pt_0).normalized();
  const Eigen::Vector2d pt_10_norm_vec =
      (ego_slot_info.pt_0 - ego_slot_info.pt_1).normalized();
  const Eigen::Vector2d pt_01_norm_up_vec(pt_01_norm_vec.y(),
                                          -pt_01_norm_vec.x());
  const Eigen::Vector2d pt_01_norm_down_vec(-pt_01_norm_vec.y(),
                                            pt_01_norm_vec.x());

  const double x_max =
      ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5 +
       apa_param.GetParam().obs_consider_long_threshold * pt_01_norm_up_vec)
          .x();

  const double y_max = std::fabs(
      (ego_slot_info.pt_1 +
       apa_param.GetParam().obs_consider_lat_threshold * pt_01_norm_vec)
          .y());

  // construct channel width and length
  if (!ego_slot_info.fus_obj_valid_flag) {
    // use uss obs
    std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
        channel_width_pq_for_x(Compare(1));
    for (const auto& obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
      if (obstacle_point_slot.x() < x_max ||
          std::fabs(obstacle_point_slot.y()) > y_max) {
        continue;
      }
      channel_width_pq_for_x.emplace(obstacle_point_slot);
    }

    if (channel_width_pq_for_x.empty()) {
      channel_width_pq_for_x.emplace(Eigen::Vector2d(
          apa_param.GetParam().channel_width +
              ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5).x(),
          0.0));
    }

    const double channel_width =
        channel_width_pq_for_x.top().x() -
        ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5).x();

    ego_slot_info.channel_width = pnc::mathlib::DoubleConstrain(
        channel_width, apa_param.GetParam().min_channel_width,
        apa_param.GetParam().channel_width);
  } else {
    // use fus obs
    ego_slot_info.channel_width =
        apa_world_ptr_->GetCollisionDetectorPtr()->GetCarMaxX(
            pnc::geometry_lib::PathPoint(ego_slot_info.ego_pos_slot,
                                         ego_slot_info.ego_heading_slot)) +
        3.168 - std::max(ego_slot_info.pt_0.x(), ego_slot_info.pt_1.x());
    ego_slot_info.channel_width = mathlib::DoubleConstrain(
        ego_slot_info.channel_width, apa_param.GetParam().channel_width, 12.0);
  }

  ILOG_INFO << "channel_width = " << ego_slot_info.channel_width;

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

  // only hack for obs is not accurate
  // const double y_min = ego_slot_info.slot_width * 0.5 - 0.068;
  // const double x_min = ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5 +
  //                       4.28 * pt_01_norm_down_vec)
  //                          .x();

  CollisionDetector::ObsSlotType obs_slot_type;
  const std::pair<Eigen::Vector2d, Eigen::Vector2d> slot_pt =
      std::make_pair(ego_slot_info.pt_1, ego_slot_info.pt_0);
  const bool is_left_side =
      (slot_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT);
  double max_obs_lat_invasion_slot_dist = 0.0;
  const double mir_width =
      (apa_param.GetParam().max_car_width - apa_param.GetParam().car_width) *
      0.5;

  const double mir_x = ego_slot_info.target_ego_pos_slot.x() +
                       apa_param.GetParam().lon_dist_mirror_to_rear_axle -
                       0.368;
  // sift obstacles that meet requirement
  for (Eigen::Vector2d obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
    // if (std::fabs(obstacle_point_slot.x()) > x_max ||
    //     obstacle_point_slot.x() < x_min ||
    //     std::fabs(obstacle_point_slot.y()) > y_max ||
    //     std::fabs(obstacle_point_slot.y()) < y_min) {
    //   continue;
    // }
    obs_slot_type = apa_world_ptr_->GetCollisionDetectorPtr()->GetObsSlotType(
        obstacle_point_slot, slot_pt, is_left_side, frame_.replan_flag);

    if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_IN_OBS &&
        !apa_param.GetParam().believe_in_fus_obs) {
      max_obs_lat_invasion_slot_dist =
          frame_.replan_flag
              ? apa_param.GetParam().max_obs_lat_invasion_slot_dist
              : apa_param.GetParam().max_obs_lat_invasion_slot_dist_dynamic_col;
      const double min_left_y =
          0.5 * ego_slot_info.slot_width - max_obs_lat_invasion_slot_dist;
      const double max_right_y =
          -0.5 * ego_slot_info.slot_width + max_obs_lat_invasion_slot_dist;
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
    // the obs lower mir can relax requirements
    if (obstacle_point_slot.x() < mir_x) {
      if (obstacle_point_slot.y() > 1e-6) {
        obstacle_point_slot.y() += mir_width;
      } else {
        obstacle_point_slot.y() -= mir_width;
      }
    }
    // the obs far from slot can relax requirements
    if (std::fabs(obstacle_point_slot.y()) >
        ego_slot_info.slot_width * 0.5 + 0.468) {
      obstacle_point_slot.x() -= 0.268;
    }
    if (obstacle_point_slot.y() > 1e-6) {
      left_pq_for_y.emplace(std::move(obstacle_point_slot));
      left_pq_for_x.emplace(std::move(obstacle_point_slot));
    } else {
      right_pq_for_y.emplace(std::move(obstacle_point_slot));
      right_pq_for_x.emplace(std::move(obstacle_point_slot));
    }
  }

  apa_param.SetPram().actual_mono_plan_enable =
      apa_param.GetParam().mono_plan_enable;
  if (apa_param.GetParam().conservative_mono_enable) {
    if (!left_pq_for_x.empty() || !right_pq_for_x.empty()) {
      apa_param.SetPram().actual_mono_plan_enable = false;
    }
  }

  // If there are no obstacles on either side, set up a virtual obstacle that is
  // farther away
  const double virtual_x =
      ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5 +
       apa_param.GetParam().virtual_obs_x_pos * pt_01_norm_down_vec)
          .x();

  const double virtual_left_y =
      (ego_slot_info.pt_1 +
       apa_param.GetParam().virtual_obs_y_pos * pt_01_norm_vec)
          .y();
  const double virtual_right_y =
      (ego_slot_info.pt_0 +
       apa_param.GetParam().virtual_obs_y_pos * pt_10_norm_vec)
          .y();

  bool left_empty = false;
  bool right_empty = false;

  if (left_pq_for_x.empty()) {
    ILOG_INFO << "left space is empty";
    left_empty = true;
    frame_.is_left_empty = true;
    left_pq_for_x.emplace(Eigen::Vector2d(virtual_x, 0.0));
    left_pq_for_y.emplace(Eigen::Vector2d(0.0, virtual_left_y));
  }
  if (right_pq_for_x.empty()) {
    ILOG_INFO << "right space is empty";
    right_empty = true;
    frame_.is_right_empty = true;
    right_pq_for_x.emplace(Eigen::Vector2d(virtual_x, 0.0));
    right_pq_for_y.emplace(Eigen::Vector2d(0.0, virtual_right_y));
  }

  const double car_half_width_with_mirror =
      apa_param.GetParam().max_car_width * 0.5;

  const double virtual_slot_width =
      apa_param.GetParam().max_car_width +
      apa_param.GetParam().slot_compare_to_car_width;

  const double real_slot_width = ego_slot_info.slot_width;

  ILOG_INFO << "max_car_width = " << apa_param.GetParam().max_car_width
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
  const double threshold = 0.6868;
  if (ego_slot_info.fus_obj_valid_flag) {
    left_y = std::max(left_y, car_half_width_with_mirror + threshold);
    left_y = std::min(left_y, virtual_left_y);
    left_x = std::min(left_x, ego_slot_info.pt_1.x() - 1.68 * threshold);
    left_x = std::max(left_x, virtual_x);
    right_y = std::min(right_y, -car_half_width_with_mirror - threshold);
    right_y = std::max(right_y, virtual_right_y);
    right_x = std::min(right_x, ego_slot_info.pt_0.x() - 1.68 * threshold);
    right_x = std::max(right_x, virtual_x);
  }

  ILOG_INFO << "real_left_y = " << real_left_y
            << "  real_right_y = " << real_right_y;

  ILOG_INFO << "left_y = " << left_y << "  right_y = " << right_y
            << "  left_x = " << left_x << "  right_x = " << right_x;

  // todo: consider actual obs pos to let slot release or not release or move
  // target pose
  double left_dis_obs_car = 0.0;
  double right_dis_obs_car = 0.0;
  if (ego_slot_info.fus_obj_valid_flag) {
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

  const double safe_threshold = apa_param.GetParam().car_lat_inflation_normal +
                                apa_param.GetParam().safe_threshold;

  const double min_safe_threshold =
      apa_param.GetParam().car_lat_inflation_normal + 0.016;

  left_obs_meet_safe_require = left_dis_obs_car > safe_threshold ? true : false;
  right_obs_meet_safe_require =
      right_dis_obs_car > safe_threshold ? true : false;

  ego_slot_info.last_move_slot_dist = ego_slot_info.move_slot_dist;
  if (frame_.replan_flag) {
    // if two side is all no safe, the slot would not release in slot managment,
    // and should not move target pose
    if (!left_obs_meet_safe_require && right_obs_meet_safe_require) {
      // left side is dangerous, should move toward right
      ego_slot_info.move_slot_dist =
          -1.0 * std::min(right_dis_obs_car - min_safe_threshold,
                          safe_threshold - left_dis_obs_car);
    } else if (left_obs_meet_safe_require && !right_obs_meet_safe_require) {
      // right side is dangerous, should move toward left
      ego_slot_info.move_slot_dist =
          std::min(left_dis_obs_car - min_safe_threshold,
                   safe_threshold - right_dis_obs_car);
    } else if (left_obs_meet_safe_require && right_obs_meet_safe_require) {
      ego_slot_info.move_slot_dist = 0.0;
    } else if (!left_obs_meet_safe_require && !right_obs_meet_safe_require) {
      const double mid_dist = (left_dis_obs_car + right_dis_obs_car) * 0.5;
      if (mid_dist < left_dis_obs_car) {
        ego_slot_info.move_slot_dist = left_dis_obs_car - mid_dist;
      } else if (mid_dist < right_dis_obs_car) {
        ego_slot_info.move_slot_dist = mid_dist - right_dis_obs_car;
      }
    }
    ILOG_INFO << "left_dis_obs_car = " << left_dis_obs_car
              << "  right_dis_obs_car = " << right_dis_obs_car;
  }

  const bool need_move_slot =
      (pnc::mathlib::IsDoubleEqual(ego_slot_info.move_slot_dist, 0.0)) ? false
                                                                       : true;

  if (need_move_slot) {
    // cal max_move_slot_dist to avoid car press line
    // no consider mirror
    const double half_car_width = apa_param.GetParam().car_width * 0.5;
    const double half_slot_width = ego_slot_info.slot_width * 0.5;
    const double car2line_dist_threshold =
        apa_param.GetParam().car2line_dist_threshold;

    // first sure if car is parked in the center, does it meet the slot line
    // distance requirement
    const double max_move_slot_dist =
        half_slot_width - half_car_width - car2line_dist_threshold;
    if (max_move_slot_dist > 0.0 &&
        (std::fabs(ego_slot_info.move_slot_dist) > max_move_slot_dist)) {
      if (ego_slot_info.move_slot_dist > 0.0) {
        ego_slot_info.move_slot_dist = max_move_slot_dist;
      }
      if (ego_slot_info.move_slot_dist < 0.0) {
        ego_slot_info.move_slot_dist = -max_move_slot_dist;
      }
    }
  }

  JSON_DEBUG_VALUE("move_slot_dist", ego_slot_info.move_slot_dist)

  // construct slot_t_lane_, left is positive, right is negative
  const double slot_width = std::min(virtual_slot_width, real_slot_width);

  Eigen::Vector2d corner_left_slot(ego_slot_info.slot_length, 0.5 * slot_width);

  Eigen::Vector2d corner_right_slot(ego_slot_info.slot_length,
                                    -0.5 * slot_width);

  const auto& slot_side = slot_t_lane_.slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    // inside is right, outside is left
    slot_t_lane_.corner_outside_slot = corner_left_slot;
    slot_t_lane_.corner_inside_slot = corner_right_slot;
    slot_t_lane_.pt_outside = corner_left_slot;
    slot_t_lane_.pt_inside = corner_right_slot;
    slot_t_lane_.pt_inside.x() =
        std::min(real_right_x, ego_slot_info.pt_0.x() + 2.68) +
        apa_param.GetParam().tlane_safe_dx;
    slot_t_lane_.pt_inside.y() =
        pnc::mathlib::Constrain(real_right_y, corner_right_slot.y() - 0.128,
                                corner_right_slot.y() + 0.068);
    // slot_t_lane_.pt_inside.y() =
    //     std::min(real_right_y, ego_slot_info.pt_0.y() + 0.05);
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    // outside is right, inside is left
    slot_t_lane_.corner_outside_slot = corner_right_slot;
    slot_t_lane_.corner_inside_slot = corner_left_slot;
    slot_t_lane_.pt_outside = corner_right_slot;
    slot_t_lane_.pt_inside = corner_left_slot;
    slot_t_lane_.pt_inside.x() =
        std::min(real_left_x, ego_slot_info.pt_1.x() + 2.68) +
        apa_param.GetParam().tlane_safe_dx;
    slot_t_lane_.pt_inside.y() =
        pnc::mathlib::Constrain(real_left_y, corner_left_slot.y() - 0.068,
                                corner_left_slot.y() + 0.128);
    // slot_t_lane_.pt_inside.y() =
    //     std::max(real_left_y, ego_slot_info.pt_1.y() - 0.05);
  }

  slot_t_lane_.pt_terminal_pos << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  slot_t_lane_.pt_terminal_heading = ego_slot_info.target_ego_heading_slot;

  if (need_move_slot) {
    slot_t_lane_.pt_terminal_pos.y() += ego_slot_info.move_slot_dist;
    slot_t_lane_.pt_inside.y() += ego_slot_info.move_slot_dist;
    slot_t_lane_.pt_outside.y() += ego_slot_info.move_slot_dist;
    ILOG_INFO << "should move slot according to obs pt, move dist = "
              << ego_slot_info.move_slot_dist;

    ego_slot_info.terminal_err.Set(
        ego_slot_info.ego_pos_slot - slot_t_lane_.pt_terminal_pos,
        ego_slot_info.ego_heading_slot - slot_t_lane_.pt_terminal_heading);
  }

  slot_t_lane_.pt_lower_boundry_pos = slot_t_lane_.pt_terminal_pos;
  slot_t_lane_.pt_lower_boundry_pos.x() =
      slot_t_lane_.pt_lower_boundry_pos.x() -
      apa_param.GetParam().rear_overhanging -
      apa_param.GetParam().col_obs_safe_dist_normal - 0.168;

  // construct obstacle_t_lane_
  // for onstacle_t_lane    right is inside, left is outside
  obstacle_t_lane_.slot_side = slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    obstacle_t_lane_.pt_inside.x() = right_x + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_inside.y() = right_y;
    obstacle_t_lane_.pt_outside.x() = left_x + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_outside.y() = left_y;
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    obstacle_t_lane_.pt_inside.x() = left_x + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_inside.y() = left_y;
    obstacle_t_lane_.pt_outside.x() =
        right_x + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_outside.y() = right_y;
  }

  obstacle_t_lane_.pt_terminal_pos = slot_t_lane_.pt_terminal_pos;
  obstacle_t_lane_.pt_terminal_heading = slot_t_lane_.pt_terminal_heading;
  obstacle_t_lane_.pt_lower_boundry_pos = slot_t_lane_.pt_lower_boundry_pos;

  // tmp method, obstacle is temporarily unavailable, force lift slot_t_lane
  // pt_inside and pt_outside
  if (apa_param.GetParam().force_both_side_occupied) {
    slot_t_lane_.pt_inside.x() = corner_right_slot.x();
    slot_t_lane_.pt_outside.x() = corner_left_slot.x();

    slot_t_lane_.pt_inside +=
        Eigen::Vector2d(apa_param.GetParam().occupied_pt_inside_dx,
                        apa_param.GetParam().occupied_pt_inside_dy);

    slot_t_lane_.pt_outside +=
        Eigen::Vector2d(apa_param.GetParam().occupied_pt_outside_dx,
                        apa_param.GetParam().occupied_pt_outside_dy);
  }

  ILOG_INFO << "-- slot_t_lane_ --";
  ILOG_INFO << "pt_outside = " << slot_t_lane_.pt_outside.transpose();
  ILOG_INFO << "pt_inside = " << slot_t_lane_.pt_inside.transpose();
  ILOG_INFO << "pt_terminal_pos = " << slot_t_lane_.pt_terminal_pos.transpose();
  ILOG_INFO << "pt_terminal_heading = " << slot_t_lane_.pt_terminal_heading;
  ILOG_INFO << "pt_lower_boundry_pos = "
            << slot_t_lane_.pt_lower_boundry_pos.transpose();

  ILOG_INFO << "-- obstacle_t_lane_ --";
  ILOG_INFO << "pt_outside = " << obstacle_t_lane_.pt_outside.transpose();
  ILOG_INFO << "pt_inside = " << obstacle_t_lane_.pt_inside.transpose();
  ILOG_INFO << "pt_terminal_pos = "
            << obstacle_t_lane_.pt_terminal_pos.transpose();
  ILOG_INFO << "pt_terminal_heading = " << obstacle_t_lane_.pt_terminal_heading;
  ILOG_INFO << "pt_lower_boundry_pos = "
            << obstacle_t_lane_.pt_lower_boundry_pos.transpose();
}

void PerpendicularTailInScenario::GenObstacles() {
  apa_world_ptr_->GetCollisionDetectorPtr()->ClearObstacles();
  // set obstacles
  double channel_width = frame_.ego_slot_info.channel_width;
  double channel_length = apa_param.GetParam().channel_length;

  if (apa_param.GetParam().force_both_side_occupied) {
    obstacle_t_lane_ = slot_t_lane_;
  }

  if (frame_.is_replan) {
    apa_world_ptr_->GetCollisionDetectorPtr()->SetParam(
        CollisionDetector::Paramters(
            apa_param.GetParam().car_lat_inflation_normal));
  } else {
    apa_world_ptr_->GetCollisionDetectorPtr()->SetParam(
        CollisionDetector::Paramters(
            apa_param.GetParam().car_lat_inflation_dynamic_col));
  }

  // add tlane obstacle
  //  B is always outside
  int slot_side = 1;
  bool is_left_side = false;
  if (obstacle_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    slot_side = -1;
    is_left_side = false;
  } else if (obstacle_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    slot_side = 1;
    is_left_side = true;
  }
  Eigen::Vector2d B(obstacle_t_lane_.pt_outside);
  const EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;
  const Eigen::Vector2d pt_01_vec = ego_slot_info.pt_1 - ego_slot_info.pt_0;
  const Eigen::Vector2d pt_01_norm_vec = pt_01_vec.normalized();
  const double obs_length = (channel_length - pt_01_vec.norm()) * 0.5;
  Eigen::Vector2d A = B - slot_side * pt_01_norm_vec * obs_length;

  Eigen::Vector2d C(obstacle_t_lane_.pt_lower_boundry_pos);
  C.y() = B.y();

  Eigen::Vector2d E(obstacle_t_lane_.pt_inside);
  Eigen::Vector2d D(obstacle_t_lane_.pt_lower_boundry_pos);
  D.y() = E.y();

  Eigen::Vector2d F = E + slot_side * pt_01_norm_vec * obs_length;

  // add channel obstacle
  const double pt_01_x = ((ego_slot_info.pt_0 + ego_slot_info.pt_1) * 0.5).x();
  const double top_x = pt_01_x + channel_width / ego_slot_info.sin_angle;
  Eigen::Vector2d channel_point_1 =
      Eigen::Vector2d(top_x, 0.0) -
      slot_side * pt_01_norm_vec * channel_length * 0.5;
  Eigen::Vector2d channel_point_2 =
      Eigen::Vector2d(top_x, 0.0) +
      slot_side * pt_01_norm_vec * channel_length * 0.5;

  Eigen::Vector2d channel_point_3;
  pnc::geometry_lib::LineSegment channel_line;
  std::vector<pnc::geometry_lib::LineSegment> channel_line_vec;
  channel_point_3 = F;
  channel_point_2.y() = channel_point_3.y();
  channel_line.SetPoints(channel_point_2, channel_point_3);
  channel_line_vec.emplace_back(channel_line);
  channel_point_3 = A;
  channel_point_1.y() = channel_point_3.y();
  channel_line.SetPoints(channel_point_1, channel_point_3);
  channel_line_vec.emplace_back(channel_line);
  channel_line.SetPoints(channel_point_1, channel_point_2);
  channel_line_vec.emplace_back(channel_line);

  const double ds = apa_param.GetParam().obstacle_ds;
  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> channel_obstacle_vec;
  channel_obstacle_vec.clear();
  channel_obstacle_vec.reserve(68);
  for (const auto& line : channel_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    channel_obstacle_vec.insert(channel_obstacle_vec.end(), point_set.begin(),
                                point_set.end());
  }

  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      channel_obstacle_vec, CollisionDetector::CHANNEL_OBS);
  // apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(channel_obstacle_vec);

  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;
  tlane_line.SetPoints(A, B);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(B, C);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(C, D);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(D, E);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(E, F);
  tlane_line_vec.emplace_back(tlane_line);

  // tmp method, should modify
  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(88);
  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }
  pnc::geometry_lib::PathPoint ego_pose;
  ego_pose.Set(frame_.ego_slot_info.ego_pos_slot,
               frame_.ego_slot_info.ego_heading_slot);

  const double bound_threshold = 0.68;

  if (apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
      apa_world_ptr_->GetApaDataPtr()->simu_param.use_obs_in_bag &&
      apa_world_ptr_->GetApaDataPtr()->simu_param.obs_x_vec.size() > 0) {
    std::vector<Eigen::Vector2d> obs_vec;
    obs_vec.reserve(
        apa_world_ptr_->GetApaDataPtr()->simu_param.obs_x_vec.size());
    Eigen::Vector2d obs;
    for (size_t i = 0;
         i < apa_world_ptr_->GetApaDataPtr()->simu_param.obs_x_vec.size();
         ++i) {
      obs = ego_slot_info.g2l_tf.GetPos(Eigen::Vector2d(
          apa_world_ptr_->GetApaDataPtr()->simu_param.obs_x_vec[i],
          apa_world_ptr_->GetApaDataPtr()->simu_param.obs_y_vec[i]));
      obs_vec.emplace_back(obs);
    }
    apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
        obs_vec, CollisionDetector::RECORD_OBS);

    OccupancyGridBound bound(
        std::min(C.x(), D.x()) - bound_threshold,
        std::min({channel_point_1.y(), channel_point_2.y(), A.y(), F.y()}) -
            bound_threshold,
        std::max(channel_point_1.x(), channel_point_2.x()) + bound_threshold,
        std::max({channel_point_1.y(), channel_point_2.y(), A.y(), F.y()}) +
            bound_threshold);
    bound.PrintInfo();
    apa_world_ptr_->GetCollisionDetectorPtr()->TransObsMapToOccupancyGridMap(
        bound);
    return;
  }

  if (!ego_slot_info.fus_obj_valid_flag) {
    // when no fus obj, temp hack, only increase plan success ratio
    double safe_dist = apa_param.GetParam().max_obs2car_dist_out_slot;
    if (frame_.ego_slot_info.slot_occupied_ratio >
            apa_param.GetParam().max_obs2car_dist_slot_occupied_ratio &&
        std::fabs(frame_.ego_slot_info.terminal_err.heading) * kRad2Deg <
            36.6) {
      safe_dist = apa_param.GetParam().max_obs2car_dist_in_slot;
    }
    for (const Eigen::Vector2d& obs_pos : tlane_obstacle_vec) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs_pos, ego_pose, safe_dist)) {
        apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
            obs_pos, CollisionDetector::TLANE_OBS);
      }
    }
  }

  else {
    const double safe_dist = 0.0268;
    // add virtual tlane obs
    std::vector<Eigen::Vector2d> tlane_obs_vec;
    tlane_obs_vec.reserve(tlane_obstacle_vec.size());
    for (const Eigen::Vector2d& obs_pos : tlane_obstacle_vec) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs_pos, ego_pose, safe_dist)) {
        tlane_obs_vec.emplace_back(obs_pos);
      }
    }
    apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
        tlane_obs_vec, CollisionDetector::TLANE_OBS);

    // add actual fus obs
    Eigen::Vector2d pt_left = obstacle_t_lane_.pt_outside;
    Eigen::Vector2d pt_right = obstacle_t_lane_.pt_inside;
    if (obstacle_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      std::swap(pt_left, pt_right);
    }
    double max_obs_lat_invasion_slot_dist = 0.0;
    std::vector<Eigen::Vector2d> fus_obs_vec;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> slot_pt =
        std::make_pair(ego_slot_info.pt_1, ego_slot_info.pt_0);
    CollisionDetector::ObsSlotType obs_slot_type;
    std::vector<Eigen::Vector2d> tlane_vec;
    tlane_vec.emplace_back(A);
    tlane_vec.emplace_back(B);
    tlane_vec.emplace_back(C);
    tlane_vec.emplace_back(D);
    tlane_vec.emplace_back(E);
    tlane_vec.emplace_back(F);
    tlane_vec.emplace_back(channel_point_2);
    tlane_vec.emplace_back(channel_point_1);

    for (Eigen::Vector2d obs_pos : ego_slot_info.obs_pt_vec_slot) {
      obs_slot_type = apa_world_ptr_->GetCollisionDetectorPtr()->GetObsSlotType(
          obs_pos, slot_pt, is_left_side, frame_.is_replan);

      if (apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs_pos, ego_pose, safe_dist)) {
        // temp hack, when obs is in car, lose it, only increase plan success
        // ratio, To Do, when obs change accurately, should not del any obs
        // ILOG_INFO <<"obs is in car, lost it, obs = "
        //             << ego_slot_info.l2g_tf.GetPos(obs_pos).transpose())
        continue;
      }

      if (!apa_param.GetParam().believe_in_fus_obs) {
        if (obs_slot_type ==
                CollisionDetector::ObsSlotType::SLOT_ENTRANCE_OBS &&
            frame_.replan_flag) {
          // obs is slot entrance, when replan, no conside it, but when dynamic
          // move and col det, conside it
          continue;
        }

        if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_IN_OBS) {
          max_obs_lat_invasion_slot_dist =
              frame_.replan_flag
                  ? apa_param.GetParam().max_obs_lat_invasion_slot_dist
                  : apa_param.GetParam()
                        .max_obs_lat_invasion_slot_dist_dynamic_col;
          const double min_left_y =
              0.5 * ego_slot_info.slot_width - max_obs_lat_invasion_slot_dist;
          const double max_right_y =
              -0.5 * ego_slot_info.slot_width + max_obs_lat_invasion_slot_dist;
          // obs is in slot, temp hack, when believe_in_fus_obs is false,
          // force move obs to out slot
          if (obs_pos.y() < min_left_y && obs_pos.y() > 0.0) {
            obs_pos.y() = min_left_y;
          }
          if (obs_pos.y() > max_right_y && obs_pos.y() < 0.0) {
            obs_pos.y() = max_right_y;
          }
        }

        if (obs_slot_type ==
            CollisionDetector::ObsSlotType::SLOT_DIRECTLY_BEHIND_OBS) {
          if (frame_.replan_flag) {
            // when replan, temp del obs directly behind slot
            continue;
          }
          // obs is directly behind slot, temp hack, when believe_in_fus_obs is
          // false, force move obs to out slot
          // if (obs_pos.y() > 0.0) {
          //   obs_pos.y() =
          //       0.5 * ego_slot_info.slot_width -
          //       max_obs_lat_invasion_slot_dist;
          // } else {
          //   obs_pos.y() = -0.5 * ego_slot_info.slot_width +
          //                 max_obs_lat_invasion_slot_dist;
          // }
        }
      }

      // if obs is not in tlane area lose it
      if (!pnc::geometry_lib::IsPointInPolygon(tlane_vec, obs_pos)) {
        continue;
      }

      fus_obs_vec.emplace_back(std::move(obs_pos));
    }

    apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
        fus_obs_vec, CollisionDetector::FUSION_OBS);
  }
  OccupancyGridBound bound(
      std::min(C.x(), D.x()) - bound_threshold,
      std::min({channel_point_1.y(), channel_point_2.y(), A.y(), F.y()}) -
          bound_threshold,
      std::max(channel_point_1.x(), channel_point_2.x()) + bound_threshold,
      std::max({channel_point_1.y(), channel_point_2.y(), A.y(), F.y()}) +
          bound_threshold);

  bound.PrintInfo();

  apa_world_ptr_->GetCollisionDetectorPtr()->TransObsMapToOccupancyGridMap(
      bound);
}

const uint8_t PerpendicularTailInScenario::PathPlanOnce() {
  ILOG_INFO << "-------------- PathPlanOnce --------------";
  // construct input
  const auto& ego_slot_info = frame_.ego_slot_info;
  PerpendicularTailInPathGenerator::Input path_planner_input;
  path_planner_input.pt_0 = ego_slot_info.pt_0;
  path_planner_input.pt_1 = ego_slot_info.pt_1;
  path_planner_input.sin_angle = ego_slot_info.sin_angle;
  path_planner_input.origin_pt_0_heading = ego_slot_info.origin_pt_0_heading;
  path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  path_planner_input.tlane = slot_t_lane_;
  path_planner_input.is_complete_path =
      apa_world_ptr_->GetApaDataPtr()->simu_param.is_complete_path;
  path_planner_input.sample_ds =
      apa_world_ptr_->GetApaDataPtr()->simu_param.sample_ds;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;
  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_replan_second = frame_.is_replan_second;
  path_planner_input.is_replan_dynamic = frame_.is_replan_dynamic;
  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);
  path_planner_input.slot2global_tf = ego_slot_info.l2g_tf;
  path_planner_input.global2slot_tf = ego_slot_info.g2l_tf;

  if (frame_.replan_reason == DYNAMIC &&
      frame_.gear_command == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    ILOG_INFO << "dynamic replan, gear should be reverse";
    path_planner_input.ref_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
  }

  perpendicular_path_planner_.SetInput(path_planner_input);

  // need replan all path
  const bool path_plan_success =
      perpendicular_path_planner_.GeometryPathGenerator::Update(
          apa_world_ptr_->GetCollisionDetectorPtr());

  uint8_t plan_result = 0;
  if (!path_plan_success &&
      !apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
      !frame_.is_replan_dynamic) {
    ILOG_INFO << "path plan fail";
    plan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = PATH_PLAN_FAILED;
    current_plan_path_vec_.clear();
    current_path_point_global_vec_.clear();
    return plan_result;
  }

  if (!path_plan_success && frame_.is_replan_dynamic) {
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

  for (size_t i = planner_output.path_seg_index.first;
       i <= planner_output.path_seg_index.second; ++i) {
    const auto& path_seg_local = planner_output.path_segment_vec[i];
    pnc::geometry_lib::PathSegment path_seg_global = path_seg_local;
    if (path_seg_local.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      path_seg_global.line_seg.pA =
          ego_slot_info.l2g_tf.GetPos(path_seg_local.GetLineSeg().pA);
      path_seg_global.line_seg.pB =
          ego_slot_info.l2g_tf.GetPos(path_seg_local.GetLineSeg().pB);
      path_seg_global.line_seg.heading =
          ego_slot_info.l2g_tf.GetHeading(path_seg_local.GetLineSeg().heading);
    } else if (path_seg_local.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      path_seg_global.arc_seg.pA =
          ego_slot_info.l2g_tf.GetPos(path_seg_local.GetArcSeg().pA);
      path_seg_global.arc_seg.pB =
          ego_slot_info.l2g_tf.GetPos(path_seg_local.GetArcSeg().pB);
      path_seg_global.arc_seg.circle_info.center = ego_slot_info.l2g_tf.GetPos(
          path_seg_local.GetArcSeg().circle_info.center);
      path_seg_global.arc_seg.headingA =
          ego_slot_info.l2g_tf.GetHeading(path_seg_local.GetArcSeg().headingA);
      path_seg_global.arc_seg.headingB =
          ego_slot_info.l2g_tf.GetHeading(path_seg_local.GetArcSeg().headingB);
    }
    current_plan_path_vec_.emplace_back(std::move(path_seg_global));
  }

  const bool gear_steer_shift =
      (frame_.is_replan_first && planner_output.gear_shift) ||
      (frame_.is_replan_second && planner_output.gear_shift) ||
      (!frame_.is_replan_first && !frame_.is_replan_second &&
       frame_.replan_reason != DYNAMIC);

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
  }

  if (frame_.is_replan_first) {
    frame_.is_replan_first = false;
    if (planner_output.gear_shift) {
      frame_.is_replan_second = false;
    } else {
      frame_.is_replan_second = true;
    }
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

  if (!apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation) {
    perpendicular_optimization_enable =
        apa_param.GetParam().perpendicular_lat_opt_enable;

    cilqr_optimization_enable =
        apa_param.GetParam().cilqr_path_optimization_enable;

  } else {
    perpendicular_optimization_enable =
        apa_world_ptr_->GetApaDataPtr()->simu_param.is_path_optimization;
    cilqr_optimization_enable =
        apa_world_ptr_->GetApaDataPtr()->simu_param.is_cilqr_optimization;
  }

  double lat_path_opt_cost_time_ms = 0.0;
  if (perpendicular_optimization_enable && is_use_optimizer) {
    ILOG_INFO << "------------------------ lateral path optimization "
                 "------------------------";
    ILOG_INFO << "frame_.gear_command= "
              << static_cast<int>(frame_.gear_command);
    ILOG_INFO << "origin path size= " << planner_output.path_point_vec.size();

    LateralPathOptimizer::Parameter param;
    param.sample_ds = apa_world_ptr_->GetApaDataPtr()->simu_param.sample_ds;
    param.q_ref_xy = apa_world_ptr_->GetApaDataPtr()->simu_param.q_ref_xy;
    param.q_ref_theta = apa_world_ptr_->GetApaDataPtr()->simu_param.q_ref_theta;
    param.q_terminal_xy =
        apa_world_ptr_->GetApaDataPtr()->simu_param.q_terminal_xy;
    param.q_terminal_theta =
        apa_world_ptr_->GetApaDataPtr()->simu_param.q_terminal_theta;
    param.q_k = apa_world_ptr_->GetApaDataPtr()->simu_param.q_k;
    param.q_u = apa_world_ptr_->GetApaDataPtr()->simu_param.q_u;
    param.q_k_bound = apa_world_ptr_->GetApaDataPtr()->simu_param.q_k_bound;
    param.q_u_bound = apa_world_ptr_->GetApaDataPtr()->simu_param.q_u_bound;

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
      global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                       ego_slot_info.l2g_tf.GetHeading(path_point.heading));

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
      global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                       ego_slot_info.l2g_tf.GetHeading(path_point.heading));

      current_path_point_global_vec_.emplace_back(global_point);
    }
  }

  JSON_DEBUG_VALUE("cilqr_optimization_enable", cilqr_optimization_enable);
  JSON_DEBUG_VALUE("lat_path_opt_cost_time_ms", lat_path_opt_cost_time_ms);

  ILOG_INFO << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size();

  return plan_result;
}

const uint8_t PerpendicularTailInScenario::NewPathPlanOnce() {
  ILOG_INFO << "-------------- PathPlanOnce --------------";
  // construct input
  const auto& ego_slot_info = frame_.ego_slot_info;
  PerpendicularTailInPathGenerator::Input path_planner_input;
  path_planner_input.pt_0 = ego_slot_info.pt_0;
  path_planner_input.pt_1 = ego_slot_info.pt_1;
  path_planner_input.sin_angle = ego_slot_info.sin_angle;
  path_planner_input.origin_pt_0_heading = ego_slot_info.origin_pt_0_heading;

  path_planner_input.tlane = slot_t_lane_;
  path_planner_input.is_complete_path =
      apa_world_ptr_->GetApaDataPtr()->simu_param.is_complete_path;
  path_planner_input.sample_ds =
      apa_world_ptr_->GetApaDataPtr()->simu_param.sample_ds;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;
  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_replan_second = frame_.is_replan_second;
  path_planner_input.is_replan_dynamic = frame_.is_replan_dynamic;
  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);

  path_planner_input.is_searching_stage = true;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus()) {
    path_planner_input.is_searching_stage = false;
  }

  if (frame_.replan_reason == DYNAMIC &&
      frame_.gear_command == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    ILOG_INFO << "dynamic replan, gear should be reverse";
    path_planner_input.ref_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
  }

  perpendicular_path_planner_.SetInput(path_planner_input);

  // need replan all path
  const bool path_plan_success =
      perpendicular_path_planner_.GeometryPathGenerator ::Update(
          apa_world_ptr_->GetCollisionDetectorPtr());

  if (path_planner_input.is_searching_stage) {
    if (path_plan_success) {
      return PathPlannerResult::PLAN_UPDATE;
    } else {
      return PathPlannerResult::PLAN_FAILED;
    }
  }

  uint8_t plan_result = 0;
  if (!path_plan_success &&
      !apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
      !frame_.is_replan_dynamic) {
    ILOG_INFO << "path plan fail";
    plan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = PATH_PLAN_FAILED;
    current_plan_path_vec_.clear();
    current_path_point_global_vec_.clear();
    return plan_result;
  }

  if (frame_.is_replan_dynamic) {
    if (path_plan_success && CheckDynamicPlanPathOptimal()) {
      ILOG_INFO << "path dynamic plan success and is superior, replace last "
                   "plan path.";
    } else {
      ILOG_INFO << "path dynamic plan fail or is not superior, continue use "
                   "last plan path.";
      frame_.dynamic_plan_fail_flag = true;
      return PathPlannerResult::PLAN_UPDATE;
    }
  }

  plan_result = PathPlannerResult::PLAN_UPDATE;

  if (!perpendicular_path_planner_.SetCurrentPathSegIndex()) {
    ILOG_INFO << "path plan fail";
    plan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = SET_SEG_INDEX;
    return plan_result;
  }

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
  all_plan_path_vec_.clear();
  all_plan_path_vec_.reserve(8);

  pnc::geometry_lib::PathSegment path_seg_global;
  for (size_t i = planner_output.path_seg_index.first;
       i < planner_output.path_segment_vec.size(); ++i) {
    path_seg_global = planner_output.path_segment_vec[i];
    path_seg_global.LocalToGlobal(ego_slot_info.l2g_tf);
    all_plan_path_vec_.emplace_back(path_seg_global);
    if (i <= planner_output.path_seg_index.second) {
      current_plan_path_vec_.emplace_back(path_seg_global);
    }
  }

  frame_.current_gear =
      pnc::geometry_lib::ReverseGear(planner_output.current_gear);

  if (frame_.is_replan_first) {
    frame_.is_replan_first = false;
    frame_.is_replan_second = true;
  } else {
    if (frame_.is_replan_second) {
      frame_.is_replan_second = false;
    }
  }

  if (!path_plan_success) {
    return plan_result;
  }

  frame_.gear_command = planner_output.current_gear;

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(planner_output.path_point_vec.size() +
                                         18);

  std::vector<pnc::geometry_lib::PathPoint> path_pt_vec;
  if (!LateralPathOptimize(path_pt_vec)) {
    path_pt_vec = planner_output.path_point_vec;
    JSON_DEBUG_VALUE("is_path_lateral_optimized", false);
  } else {
    JSON_DEBUG_VALUE("is_path_lateral_optimized", true);
  }

  pnc::geometry_lib::PathPoint global_point;
  for (const auto& path_point : path_pt_vec) {
    global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                     ego_slot_info.l2g_tf.GetHeading(path_point.heading));

    current_path_point_global_vec_.emplace_back(global_point);
  }

  // record all_gear_path_segment_vec
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> heading_vec;
  std::vector<double> lat_buffer_vec;
  const size_t interval_number = 2;
  x_vec.reserve(planner_output.all_gear_path_point_vec.size() /
                    interval_number +
                interval_number);
  y_vec.reserve(planner_output.all_gear_path_point_vec.size() /
                    interval_number +
                interval_number);
  heading_vec.reserve(planner_output.all_gear_path_point_vec.size() /
                          interval_number +
                      interval_number);
  lat_buffer_vec.reserve(planner_output.all_gear_path_point_vec.size() /
                             interval_number +
                         interval_number);
  for (size_t i = 0; i < planner_output.all_gear_path_point_vec.size(); ++i) {
    if (i % interval_number == 0) {
      global_point.Set(ego_slot_info.l2g_tf.GetPos(
                           planner_output.all_gear_path_point_vec[i].pos),
                       ego_slot_info.l2g_tf.GetHeading(
                           planner_output.all_gear_path_point_vec[i].heading));
      x_vec.emplace_back(global_point.pos.x());
      y_vec.emplace_back(global_point.pos.y());
      heading_vec.emplace_back(global_point.heading);
      lat_buffer_vec.emplace_back(
          planner_output.all_gear_path_point_vec[i].lat_buffer);
    }
  }

  JSON_DEBUG_VECTOR("plan_traj_x", x_vec, 3)
  JSON_DEBUG_VECTOR("plan_traj_y", y_vec, 3)
  JSON_DEBUG_VECTOR("plan_traj_heading", heading_vec, 3)
  JSON_DEBUG_VECTOR("plan_traj_lat_buffer", lat_buffer_vec, 3)

  ILOG_INFO << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size();

  return plan_result;
}

const bool PerpendicularTailInScenario::LateralPathOptimize(
    std::vector<geometry_lib::PathPoint>& optimal_path_vec) {
  const GeometryPathGenerator::Output& path_plan_output =
      perpendicular_path_planner_.GetOutput();
  const double sample_ds = path_plan_output.actual_ds;
  const std::vector<pnc::geometry_lib::PathPoint>& pt_vec =
      path_plan_output.path_point_vec;
  const double cur_gear_length = path_plan_output.cur_gear_length;
  const SimulationParam& simu_param =
      apa_world_ptr_->GetApaDataPtr()->simu_param;
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
  if (apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation) {
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
      PLANNING_TRAJ_POINTS_NUM - APA_COMPARE_PLANNING_TRAJ_POINTS_NUM;
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
    x_vec.reserve(origin_optimized_path_vec.size());
    y_vec.reserve(origin_optimized_path_vec.size());
    s_vec.reserve(origin_optimized_path_vec.size());
    heading_vec.reserve(origin_optimized_path_vec.size());
    for (const auto& pt : origin_optimized_path_vec) {
      x_vec.emplace_back(pt.pos.x());
      y_vec.emplace_back(pt.pos.y());
      s_vec.emplace_back(pt.s);
      heading_vec.emplace_back(pt.heading);
    }
    mathlib::spline x_s_spline;
    mathlib::spline y_s_spline;
    mathlib::spline heading_s_spline;
    x_s_spline.set_points(s_vec, x_vec);
    y_s_spline.set_points(s_vec, y_vec);
    heading_s_spline.set_points(s_vec, heading_vec);
    geometry_lib::PathPoint tmp_pt;
    double ds = 0.0;
    for (size_t i = 1; i < max_pt_number; ++i) {
      tmp_pt.pos << x_s_spline(ds), y_s_spline(ds);
      tmp_pt.heading = heading_s_spline(ds);
      tmp_pt.s = ds;
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

    // 可能没采样到终点
    if (optimal_path_vec.size() > 0) {
      tmp_pt = optimal_path_vec.back();
      if (s_vec.back() - optimal_path_vec.back().s > 1e-2) {
        tmp_pt.pos << x_s_spline(s_vec.back()), y_s_spline(s_vec.back());
        tmp_pt.heading = heading_s_spline(s_vec.back());
        tmp_pt.s = s_vec.back();
        optimal_path_vec.emplace_back(tmp_pt);
      }
    }
  }

  if (optimal_path_vec.size() < 3) {
    ILOG_INFO << "optimized_path_vec.size() < 3";
    return false;
  }

  // 检查路径长度是否有较大变化
  if (std::fabs(cur_gear_length - optimal_path_vec.back().s) > 5e-2) {
    ILOG_INFO << "length is not the same, cur_gear_length = " << cur_gear_length
              << "  optimized_path length = " << optimal_path_vec.back().s;
    return false;
  }

  // 检查起点终点是否一致
  if (!pnc::geometry_lib::CheckTwoPoseIsSame(
          optimal_path_vec.front(), pt_vec.front(), 0.02, 0.3 / 57.3) ||
      !pnc::geometry_lib::CheckTwoPoseIsSame(optimal_path_vec.back(),
                                             pt_vec.back(), 0.02, 0.3 / 57.3)) {
    ILOG_INFO << "start or end pose is not same";
    return false;
  }

  // 检查优化后的路径是否很奇怪

  // 检查是否碰撞
  const auto col_res =
      apa_world_ptr_->GetCollisionDetectorPtr()->UpdateByObsMap(
          optimal_path_vec, 0.08, 0.0);
  if (col_res.remain_dist < optimal_path_vec.back().s - 2e-2) {
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

const bool PerpendicularTailInScenario::CheckDynamicPlanPathOptimal() {
  // 拿到之前的路径 并转换到如今的车位坐标系下
  pnc::geometry_lib::GeometryPath geometry_path_bef(all_plan_path_vec_);
  geometry_path_bef.GlobalToLocal(frame_.ego_slot_info.g2l_tf);

  // 拿到现在的路径
  const pnc::geometry_lib::GeometryPath geometry_path_now(
      perpendicular_path_planner_.GetOutput().path_segment_vec);

  // 拿到之前的规划终点
  const pnc::geometry_lib::PathPoint tar_pose_bef = geometry_path_bef.end_pose;

  // 拿到现在的规划终点
  const pnc::geometry_lib::PathPoint tar_pose_now(
      slot_t_lane_.pt_terminal_pos, slot_t_lane_.pt_terminal_heading);

  if (geometry_path_now.gear_change_count > 0) {
    return false;
  }

  if (geometry_path_bef.gear_change_count > 0) {
    return true;
  }

  // 如果现在路径和当前路径方向并不相反
  if (!pnc::geometry_lib::IsOppositeSteer(geometry_path_bef.cur_steer,
                                          geometry_path_now.cur_steer)) {
    return true;
  }

  // 如果现在的路径没有S弯 不甩头
  if (!geometry_path_now.IsHasSTurnPath()) {
    return true;
  }

  // 如果有S弯甩头 比较一下 误差较大影响finish时才调
  if (std::fabs(tar_pose_now.pos.y() - tar_pose_bef.pos.y()) >
          apa_param.GetParam().finish_lat_err * 0.5 ||
      std::fabs(pnc::geometry_lib::AngleSubtraction(tar_pose_now.heading,
                                                    tar_pose_bef.heading)) >
          apa_param.GetParam().finish_heading_err * 0.5) {
    return true;
  }

  return false;
}

const bool PerpendicularTailInScenario::CheckSegCompleted() {
  bool is_seg_complete = false;
  if (frame_.spline_success) {
    if (frame_.remain_dist < apa_param.GetParam().max_replan_remain_dist &&
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
      ILOG_INFO << "close to target, need wait a certain time!";
      if (frame_.stuck_uss_time > 0.068) {
        ILOG_INFO << "wait a certain time, start plan";
        is_seg_complete = true;
      }
    }
  }

  return is_seg_complete;
}

const bool PerpendicularTailInScenario::CheckUssStucked() {
  if (frame_.remain_dist_uss < apa_param.GetParam().max_replan_remain_dist &&
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
    ILOG_INFO << "close to obstacle by uss!, need wait a certain time!";
    if (frame_.stuck_uss_time >
        apa_param.GetParam().uss_stuck_replan_wait_time) {
      ILOG_INFO << "wait a certain time, start plan";
      frame_.is_replan_by_uss = true;
      return true;
    }
  }

  return false;
}

const bool PerpendicularTailInScenario::CheckColDetStucked() {
  if (frame_.remain_dist_col_det <
          apa_param.GetParam().max_replan_remain_dist &&
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
    ILOG_INFO << "close to obstacle by col det!, need wait a certain time!";
    if (frame_.stuck_uss_time >
        apa_param.GetParam().uss_stuck_replan_wait_time) {
      ILOG_INFO << "wait a certain time, start plan";
      return true;
    }
  }
  return false;
}

const bool PerpendicularTailInScenario::CheckDynamicUpdate() {
  double heading_err = apa_param.GetParam().max_heading_err_3;
  if (frame_.is_left_empty && frame_.is_right_empty) {
    heading_err = apa_param.GetParam().pose_heading_err;
  }
  // const bool dynamic_update_flag =
  //     frame_.ego_slot_info.slot_occupied_ratio >
  //         apa_param.GetParam().pose_slot_occupied_ratio &&
  //     frame_.ego_slot_info.slot_occupied_ratio <
  //         apa_param.GetParam().pose_slot_occupied_ratio_3 &&
  //     std::fabs(frame_.ego_slot_info.terminal_err.heading) <
  //         heading_err * kDeg2Rad &&
  //     frame_.gear_command == pnc::geometry_lib::SEG_GEAR_REVERSE &&
  //     !apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  const bool dynamic_update_flag =
      frame_.gear_command == pnc::geometry_lib::SEG_GEAR_REVERSE &&
      !apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag() &&
      frame_.ego_slot_info.slot_occupied_ratio <
          apa_param.GetParam().pose_slot_occupied_ratio_3;

  if (dynamic_update_flag) {
    frame_.dynamic_plan_time += apa_param.GetParam().plan_time;
  } else {
    frame_.dynamic_plan_time = 0.0;
  }

  if (frame_.dynamic_plan_time >
      apa_param.GetParam().dynamic_plan_interval_time) {
    frame_.is_replan_dynamic = true;
    frame_.dynamic_plan_time = 0.0;
  }

  return frame_.is_replan_dynamic;
}

const bool PerpendicularTailInScenario::CheckReplan() {
  pt_center_replan_jump_dist_ =
      (pt_center_replan_ - frame_.ego_slot_info.slot_center).norm();
  pt_center_replan_jump_heading_ =
      std::fabs(pnc::geometry_lib::NormalizeAngle(
          pt_center_heading_replan_ -
          frame_.ego_slot_info.slot_origin_heading)) *
      kRad2Deg;
  ILOG_INFO << "replan slot_jump_dist = " << pt_center_replan_jump_dist_;
  ILOG_INFO << "replan slot_jump_heading = " << pt_center_replan_jump_heading_;

  if (frame_.is_replan_first == true) {
    ILOG_INFO << "first plan";
    frame_.replan_reason = FIRST_PLAN;
    return true;
  }

  frame_.is_replan_by_uss = false;
  frame_.is_replan_dynamic = false;

  if (apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan) {
    ILOG_INFO << "force plan";
    frame_.replan_reason = FORCE_PLAN;
    return true;
  }

  if (CheckSegCompleted()) {
    ILOG_INFO << "replan by current segment completed!";
    frame_.replan_reason = SEG_COMPLETED_PATH;
    return true;
  }

  if (CheckUssStucked()) {
    ILOG_INFO << "replan by uss stucked!";
    frame_.replan_reason = SEG_COMPLETED_USS;
    return true;
  }

  if (CheckColDetStucked()) {
    ILOG_INFO << "replan by col det stucked!";
    frame_.replan_reason = SEG_COMPLETED_COL_DET;
    return true;
  }

  if (frame_.stuck_uss_time > apa_param.GetParam().stuck_replan_time) {
    // if plan once, the stuck_uss_time is clear and accumlate again
    ILOG_INFO << "replan by stuck!";
    frame_.replan_reason = STUCKED;
    return true;
  }

  if (!apa_world_ptr_->GetApaDataPtr()->simu_param.sim_to_target &&
      CheckDynamicUpdate()) {
    ILOG_INFO << "replan by dynamic!";
    frame_.replan_reason = DYNAMIC;
    return true;
  }

  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool PerpendicularTailInScenario::CheckFinished() {
  const auto& terminal_err_pose = frame_.ego_slot_info.terminal_err;

  const bool lon_condition =
      terminal_err_pose.pos.x() < apa_param.GetParam().finish_lon_err;

  const double y1 = terminal_err_pose.pos.y();
  const double y2 =
      (terminal_err_pose.pos + (apa_param.GetParam().wheel_base +
                                apa_param.GetParam().front_overhanging) *
                                   frame_.ego_slot_info.ego_heading_slot_vec)
          .y();

  const bool lat_condition_1 =
      std::fabs(y1) <= apa_param.GetParam().finish_lat_err;

  const bool lat_condition_2 =
      std::fabs(y1) <= apa_param.GetParam().finish_lat_err_strict &&
      std::fabs(y2) <= apa_param.GetParam().finish_lat_err_strict;

  const bool heading_condition_1 =
      std::fabs(terminal_err_pose.heading) <=
      apa_param.GetParam().finish_heading_err * kDeg2Rad;

  const bool heading_condition_2 =
      std::fabs(terminal_err_pose.heading) <=
      (apa_param.GetParam().finish_heading_err + 1.988) * kDeg2Rad;

  const bool lat_condition = (lat_condition_1 && heading_condition_1) &&
                             (lat_condition_2 && heading_condition_2);

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

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

  // stucked by dynamic col det
  const bool remain_dist_col_det_condition =
      frame_.remain_dist_col_det < apa_param.GetParam().max_replan_remain_dist;

  parking_finish = lat_condition && static_condition && enter_slot_condition &&
                   remain_dist_col_det_condition &&
                   (terminal_err_pose.pos.x() < 0.568);

  return parking_finish;
}

const bool PerpendicularTailInScenario::PostProcessPathAccordingLimiter() {
  size_t origin_traj_size = current_path_point_global_vec_.size();

  if (origin_traj_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: origin_traj_size = " << origin_traj_size;
    return false;
  }

  // cal proj point, need extend, add a point
  // if need extend according limiter, need to add another point
  const size_t traj_size = origin_traj_size + 2;

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

  const Eigen::Vector2d limiter_mid =
      (frame_.ego_slot_info.l2g_tf.GetPos(frame_.ego_slot_info.limiter.first) +
       frame_.ego_slot_info.l2g_tf.GetPos(
           frame_.ego_slot_info.limiter.second)) /
      2.0;

  double s_proj = 0.0;
  bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
      0.0, frame_.current_path_length + frame_.path_extended_dist, s_proj,
      limiter_mid, frame_.x_s_spline, frame_.y_s_spline);
  if (!success) {
    ILOG_INFO << "path is err";
    return false;
  }
  if (s_proj < apa_world_ptr_->GetApaDataPtr()->simu_param.sample_ds * 1.5) {
    ILOG_INFO << "limiter s_proj is too small";
    return false;
  }
  double ds = 0.0;
  double s = 0.0;
  for (size_t i = 0; i < current_path_point_global_vec_.size(); ++i) {
    if (i > 0) {
      ds = std::hypot(current_path_point_global_vec_[i].pos.x() -
                          current_path_point_global_vec_[i - 1].pos.x(),
                      current_path_point_global_vec_[i].pos.y() -
                          current_path_point_global_vec_[i - 1].pos.y());
      s += std::max(ds, 1e-3);
    }
    if (s > s_proj) {
      ILOG_INFO << "path shoule be shorten because of limiter";
      if (s_proj - s_vec.back() > 0.036 && frame_.spline_success) {
        x_vec.emplace_back(frame_.x_s_spline(s_proj));
        y_vec.emplace_back(frame_.y_s_spline(s_proj));
        heading_vec.emplace_back(heading_vec.back());
        s_vec.emplace_back(s_proj);
      }
      break;
    }
    x_vec.emplace_back(current_path_point_global_vec_[i].pos.x());
    y_vec.emplace_back(current_path_point_global_vec_[i].pos.y());
    heading_vec.emplace_back(current_path_point_global_vec_[i].heading);
    s_vec.emplace_back(s);
  }
  if (s < s_proj) {
    ILOG_INFO << "path shoule be extended because of limiter";
    using namespace pnc::geometry_lib;
    double init_length = 0.0;
    double extend_length = s_proj - s;
    if (current_plan_path_vec_.size() > 0 &&
        std::fabs(current_plan_path_vec_.back().GetEndHeading()) * kRad2Deg <
            1.08) {
      PathSegment& path_seg_global = current_plan_path_vec_.back();
      init_length = path_seg_global.Getlength();
      if (path_seg_global.seg_type == SEG_TYPE_LINE) {
        CompleteLineInfo(path_seg_global.line_seg, init_length + extend_length);
      } else if (path_seg_global.seg_type == SEG_TYPE_ARC) {
        CompleteArcInfo(path_seg_global.arc_seg, init_length + extend_length,
                        path_seg_global.arc_seg.is_anti_clockwise);
      }
      ILOG_INFO << "init_length = " << init_length
                << "  extend_length = " << extend_length
                << "  cur_path_length = " << path_seg_global.Getlength()
                << "  s_proj = " << s_proj << "  s = " << s;

      // this path is global, need to transform to local to col det
      CollisionDetector::CollisionResult col_res;
      pnc::geometry_lib::PathSegment path_seg_local = path_seg_global;
      const EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;
      if (path_seg_global.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
        path_seg_local.line_seg.pA =
            ego_slot_info.g2l_tf.GetPos(path_seg_global.line_seg.pA);
        path_seg_local.line_seg.pB =
            ego_slot_info.g2l_tf.GetPos(path_seg_global.line_seg.pB);
        path_seg_local.line_seg.heading =
            ego_slot_info.g2l_tf.GetHeading(path_seg_global.line_seg.heading);

        col_res = apa_world_ptr_->GetCollisionDetectorPtr()->UpdateByObsMap(
            path_seg_local.line_seg, path_seg_local.line_seg.heading);
      } else if (path_seg_global.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        path_seg_local.arc_seg.pA =
            ego_slot_info.g2l_tf.GetPos(path_seg_global.GetArcSeg().pA);
        path_seg_local.arc_seg.pB =
            ego_slot_info.g2l_tf.GetPos(path_seg_global.GetArcSeg().pB);
        path_seg_local.arc_seg.circle_info.center = ego_slot_info.g2l_tf.GetPos(
            path_seg_global.GetArcSeg().circle_info.center);
        path_seg_local.arc_seg.headingA = ego_slot_info.g2l_tf.GetHeading(
            path_seg_global.GetArcSeg().headingA);
        path_seg_local.arc_seg.headingB = ego_slot_info.g2l_tf.GetHeading(
            path_seg_global.GetArcSeg().headingB);

        col_res = apa_world_ptr_->GetCollisionDetectorPtr()->UpdateByObsMap(
            path_seg_local.arc_seg, path_seg_local.arc_seg.headingA);
      }

      const double remain_dist =
          std::max(init_length,
                   std::min(col_res.remain_obstacle_dist -
                                apa_param.GetParam().col_obs_safe_dist_normal,
                            col_res.remain_car_dist));

      extend_length = remain_dist - init_length;

      pnc::geometry_lib::CompletePathSeg(path_seg_global, remain_dist);

      ILOG_INFO << "length = " << path_seg_global.Getlength()
                << "  remain_dist = " << remain_dist
                << "  extend_length = " << extend_length;
    }
    const double total_length = s + extend_length;
    while (s <= total_length) {
      if (x_vec.size() > PLANNING_TRAJ_POINTS_NUM - 1) {
        break;
      }
      s += apa_world_ptr_->GetApaDataPtr()->simu_param.sample_ds;
      x_vec.emplace_back(frame_.x_s_spline(s));
      y_vec.emplace_back(frame_.y_s_spline(s));
      heading_vec.emplace_back(heading_vec.back());
      s_vec.emplace_back(s);
    }
    // x_vec.emplace_back(frame_.x_s_spline(s_proj));
    // y_vec.emplace_back(frame_.y_s_spline(s_proj));
    // heading_vec.emplace_back(heading_vec.back());
    // s_vec.emplace_back(s_proj);
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
  success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
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

const bool PerpendicularTailInScenario::PostProcessPathAccordingObs(
    const double car_remain_dist) {
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
    if (s > car_remain_dist) {
      ILOG_INFO << "path shoule be shorten because of obs";
      if (car_remain_dist - s_vec.back() > 0.036 && frame_.spline_success) {
        x_vec.emplace_back(frame_.x_s_spline(car_remain_dist));
        y_vec.emplace_back(frame_.y_s_spline(car_remain_dist));
        heading_vec.emplace_back(heading_vec.back());
        s_vec.emplace_back(car_remain_dist);
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

void PerpendicularTailInScenario::Log() const {
  const EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;
  const auto& l2g_tf = ego_slot_info.l2g_tf;
  const auto p0_g = l2g_tf.GetPos(slot_t_lane_.pt_outside);
  const auto p1_g = l2g_tf.GetPos(slot_t_lane_.pt_inside);
  const auto pt_g = l2g_tf.GetPos(slot_t_lane_.pt_terminal_pos);

  JSON_DEBUG_VALUE("tlane_p0_x", p0_g.x())
  JSON_DEBUG_VALUE("tlane_p0_y", p0_g.y())
  JSON_DEBUG_VALUE("tlane_p1_x", p1_g.x())
  JSON_DEBUG_VALUE("tlane_p1_y", p1_g.y())
  JSON_DEBUG_VALUE("tlane_pt_x", pt_g.x())
  JSON_DEBUG_VALUE("tlane_pt_y", pt_g.y())
  JSON_DEBUG_VALUE("slot_side", slot_t_lane_.slot_side)

  JSON_DEBUG_VALUE("correct_path_for_limiter", frame_.correct_path_for_limiter)
  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag)
  JSON_DEBUG_VALUE("slot_replan_jump_dist", pt_center_replan_jump_dist_)
  JSON_DEBUG_VALUE("slot_replan_jump_heading", pt_center_replan_jump_heading_)

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
          !(std::fabs(obstacle.y()) < 1.168 && obstacle.x() < 5.068 &&
            obstacle.x() > -0.468)) {
        continue;
      }
      const Eigen::Vector2d tmp_obstacle = l2g_tf.GetPos(obstacle);
      obstaclesX.emplace_back(tmp_obstacle.x());
      obstaclesY.emplace_back(tmp_obstacle.y());
    }
  }

  JSON_DEBUG_VECTOR("obstaclesX", obstaclesX, 2)
  JSON_DEBUG_VECTOR("obstaclesY", obstaclesY, 2)

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
  slot_corner_X.emplace_back(l2g_tf.GetPos(slot_t_lane_.pt_terminal_pos).x());
  slot_corner_Y.emplace_back(l2g_tf.GetPos(slot_t_lane_.pt_terminal_pos).y());

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

  const auto& path_plan_output = perpendicular_path_planner_.GetOutput();
  JSON_DEBUG_VALUE("path_start_seg_index",
                   path_plan_output.path_seg_index.first)
  JSON_DEBUG_VALUE("path_end_seg_index", path_plan_output.path_seg_index.second)
  JSON_DEBUG_VALUE("path_length", path_plan_output.length)

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
}

void PerpendicularTailInScenario::ScenarioTry() {
  Reset();

  std::shared_ptr<SlotManager> slot_manager =
      apa_world_ptr_->GetSlotManagerPtr();

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    ILOG_INFO << "update ego slot info fail";
    slot_manager->SlotReleaseByScenarioTry(
        false, SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE);

    return;
  }

  // todo: use prepare plan to replace PathPlan.
  GenTlane();
  GenObstacles();

  uint8_t result = NewPathPlanOnce();
  if (result != PathPlannerResult::PLAN_UPDATE) {
    slot_manager->SlotReleaseByScenarioTry(
        false, SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE);

    ILOG_INFO << "geometry path fail";
    return;
  } else {
    slot_manager->SlotReleaseByScenarioTry(
        true, SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE);

    ILOG_INFO << "geometry path success";
  }

  return;
}

}  // namespace apa_planner
}  // namespace planning
