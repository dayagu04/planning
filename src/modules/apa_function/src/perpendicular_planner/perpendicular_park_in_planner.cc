#include "perpendicular_park_in_planner.h"

#include <bits/stdint-uintn.h>

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <queue>

#include "Eigen/src/Core/util/Constants.h"
#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_utils.h"
#include "apa_world.h"
#include "basic_types.pb.h"
#include "collision_detection.h"
#include "common.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "fem_pos_deviation_smoother_config.pb.h"
#include "func_state_machine.pb.h"
#include "general_planning_context.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "perpendicular_path_planner.h"
#include "slot_management_info.pb.h"

namespace planning {
namespace apa_planner {
void PerpendicularInPlanner::Reset() {
  frame_.Reset();
  slot_t_lane_.Reset();
  perpendicular_path_planner_.Reset();
}

void PerpendicularInPlanner::SetParkingStatus(uint8_t status) {
  if (status == PARKING_IDLE || status == PARKING_FAILED) {
    frame_.plan_stm.path_plan_success = false;
  } else if (status == PARKING_RUNNING || status == PARKING_GEARCHANGE ||
             status == PARKING_PLANNING || status == PARKING_FINISHED ||
             status == PARKING_PAUSED) {
    frame_.plan_stm.path_plan_success = true;
  }

  frame_.plan_stm.planning_status = status;
}

void PerpendicularInPlanner::Update() {
  // run plan core
  PlanCore();

  // generate planning output
  GenPlanningOutput();

  // log json debug
  Log();
}

void PerpendicularInPlanner::PlanCore() {
  // prepare simulation
  InitSimulation();

  // if (!simu_param_.force_plan) {
  //   return;
  // }
  // std::cout << "PerpendicularInPlanner\n";

  // check planning status
  if (!simu_param_.force_plan && CheckPlanSkip()) {
    return;
  }

  if (CheckPaused()) {
    SetParkingStatus(PARKING_PAUSED);
    if (frame_.pause_time > apa_param.GetParam().pause_failed_time) {
      SetParkingStatus(PARKING_FAILED);
    }
    return;
  }

  // update remain dist
  UpdateRemainDist();

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    std::cout << "update ego slot info" << std::endl;
    SetParkingStatus(PARKING_FAILED);
    return;
  }

  // check finish
  if (CheckFinished()) {
    std::cout << "check apa finished!" << std::endl;
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

  // check failed
  if (CheckStuckFailed()) {
    std::cout << "check stuck failed!" << std::endl;
    SetParkingStatus(PARKING_FAILED);
    return;
  }

  // check replan
  if (simu_param_.force_plan || CheckReplan()) {
    std::cout << "replan is required!" << std::endl;

    // UpdateSlotRealtime();
    frame_.ego_slot_info.first_fix_limiter = true;

    GenTlane();

    GenObstacles();

    // path plan
    const auto pathplan_result = PathPlanOnce();
    frame_.pathplan_result = pathplan_result;

    if (pathplan_result == PathPlannerResult::PLAN_HOLD) {
      if (PostProcessPath()) {
        SetParkingStatus(PARKING_GEARCHANGE);
        std::cout << "replan from PARKING_GEARCHANGE!" << std::endl;
      } else {
        SetParkingStatus(PARKING_FAILED);
        std::cout << "replan failed from PLAN_HOLD!" << std::endl;
      }
    } else if (pathplan_result == PathPlannerResult::PLAN_UPDATE) {
      if (PostProcessPath()) {
        SetParkingStatus(PARKING_PLANNING);
        std::cout << "replan from PARKING_PLANNING!" << std::endl;
      } else {
        SetParkingStatus(PARKING_FAILED);
        std::cout << "replan failed from PARKING_PLANNING!" << std::endl;
      }
    } else if (pathplan_result == PathPlannerResult::PLAN_FAILED) {
      SetParkingStatus(PARKING_FAILED);
    }

    std::cout << "pathplan_result = " << static_cast<int>(pathplan_result)
              << std::endl;
  } else {
    std::cout << "replan is not required!" << std::endl;
    SetParkingStatus(PARKING_RUNNING);
  }

  // check finish
  if (CheckFinished()) {
    std::cout << "check apa finished!" << std::endl;
    SetParkingStatus(PARKING_FINISHED);
    return;
  }

  // check failed
  if (CheckStuckFailed()) {
    std::cout << "check stuck failed!" << std::endl;
    SetParkingStatus(PARKING_FAILED);
    return;
  }

  // check planning status
  std::cout << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status)
            << std::endl;
}
const bool PerpendicularInPlanner::UpdateEgoSlotInfo() {
  const auto measures_ptr = apa_world_ptr_->GetMeasurementsPtr();
  const auto slot_manager_ptr_ = apa_world_ptr_->GetSlotManagerPtr();

  EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;

  if (!frame_.is_fix_slot) {
    ego_slot_info.target_managed_slot =
        slot_manager_ptr_->GetEgoSlotInfo().select_slot_filter;

    const auto& slot_points =
        ego_slot_info.target_managed_slot.corner_points().corner_point();
    std::vector<Eigen::Vector2d> pt;
    pt.resize(4);
    for (size_t i = 0; i < 4; ++i) {
      if (is_simulation_ && simu_param_.target_managed_slot_x_vec.size() == 4) {
        pt[i] << simu_param_.target_managed_slot_x_vec[i],
            simu_param_.target_managed_slot_y_vec[i];
      } else {
        pt[i] << slot_points[i].x(), slot_points[i].y();
      }
    }
    const auto pM01 = 0.5 * (pt[0] + pt[1]);
    const auto pM23 = 0.5 * (pt[2] + pt[3]);
    const double real_slot_length = (pM01 - pM23).norm();
    const auto t = (pt[1] - pt[0]).normalized();
    // const auto n = (pM01 - pM23).normalized();
    const auto n = Eigen::Vector2d(t.y(), -t.x());
    pt[2] = pt[0] - real_slot_length * n;
    pt[3] = pt[1] - real_slot_length * n;

    ego_slot_info.slot_corner = pt;

    const double virtual_slot_length =
        apa_param.GetParam().car_length +
        apa_param.GetParam().slot_compare_to_car_length;

    ego_slot_info.slot_origin_pos = pM01 - virtual_slot_length * n;
    ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
    ego_slot_info.slot_origin_heading_vec = n;
    ego_slot_info.slot_length = virtual_slot_length;
    ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

    ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                              ego_slot_info.slot_origin_heading);

    ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                              ego_slot_info.slot_origin_heading);

    if (ego_slot_info.target_managed_slot.slot_type() ==
        Common::PARKING_SLOT_TYPE_SLANTING) {
      const Eigen::Vector2d origin_pt_0 =
          Eigen::Vector2d(slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points(0)
                              .x(),
                          slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points(0)
                              .y());

      const Eigen::Vector2d origin_pt_1 =
          Eigen::Vector2d(slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points(1)
                              .x(),
                          slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points(1)
                              .y());

      ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(origin_pt_0);
      ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(origin_pt_1);

      if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
        std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
      }

      const Eigen::Vector2d pt_01_vec = ego_slot_info.pt_1 - ego_slot_info.pt_0;

      double angle =
          std::fabs(pnc::geometry_lib::GetAngleFromTwoVec(
              Eigen::Vector2d(virtual_slot_length, 0.0), pt_01_vec)) *
          57.3;

      if (angle > 90.0) {
        angle = 180.0 - angle;
      }

      angle = pnc::mathlib::DoubleConstrain(angle, 10.0, 80.0);
      ego_slot_info.sin_angle = std::sin(angle / 57.3);
      ego_slot_info.origin_pt_0_heading = 90.0 - angle;
    } else {
      ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(pt[0]);
      ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(pt[1]);
      ego_slot_info.sin_angle = 1.0;
      ego_slot_info.origin_pt_0_heading = 0.0;
    }
  }

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(measures_ptr->pos_ego);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measures_ptr->heading_ego);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  ego_slot_info.obs_pt_vec_slot.clear();
  ego_slot_info.obs_pt_vec_slot =
      slot_manager_ptr_->GetEgoSlotInfo().obs_pt_vec_slot;

  ego_slot_info.limiter = slot_manager_ptr_->GetEgoSlotInfo().limiter;
  if (is_simulation_ && simu_param_.target_managed_limiter_x_vec.size() == 2) {
    ego_slot_info.limiter.first << simu_param_.target_managed_limiter_x_vec[0],
        simu_param_.target_managed_limiter_y_vec[0];
    ego_slot_info.limiter.first =
        ego_slot_info.g2l_tf.GetPos(ego_slot_info.limiter.first);
    ego_slot_info.limiter.second << simu_param_.target_managed_limiter_x_vec[1],
        simu_param_.target_managed_limiter_y_vec[1];
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
          apa_param.GetParam().slot_occupied_ratio_max_heading_err / 57.3) {
    ego_slot_info.slot_occupied_ratio = pnc::mathlib::Clamp(
        1.0 - (ego_slot_info.terminal_err.pos.x() / ego_slot_info.slot_length),
        0.0, 1.0);
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  // trim path according to limiter
  if (!frame_.is_replan_first &&
      (perpendicular_path_planner_.GetOutput().is_last_path ||
       perpendicular_path_planner_.GetOutput().path_segment_vec.empty() ||
       true) &&
      gear_command_ == pnc::geometry_lib::SEG_GEAR_REVERSE &&
      ego_slot_info.first_fix_limiter) {
    const pnc::geometry_lib::LineSegment limiter_line(
        ego_slot_info.limiter.first, ego_slot_info.limiter.second);

    const auto dist_ego_limiter = pnc::geometry_lib::CalPoint2LineDist(
        ego_slot_info.ego_pos_slot, limiter_line);

    std::cout << "dist_ego_limiter = " << dist_ego_limiter << std::endl;

    if (dist_ego_limiter < apa_param.GetParam().car_to_limiter_dis) {
      std::cout << "should trim path according limiter\n";
      ego_slot_info.first_fix_limiter = false;
      PostProcessPathAccordingLimiter();
    }
  }

  if (frame_.is_replan_first) {
    pt_center_ = (ego_slot_info.slot_corner[0] + ego_slot_info.slot_corner[1] +
                  ego_slot_info.slot_corner[2] + ego_slot_info.slot_corner[3]) /
                 4.0;
  }

  // trim path according to slot when 1R
  if ((perpendicular_path_planner_.GetOutput().is_first_reverse_path ||
       apa_param.GetParam().dynamic_col_det_enable) &&
      !first_reverse_path_vec_.empty()) {
    const double dist =
        ((ego_slot_info.slot_corner[0] + ego_slot_info.slot_corner[1] +
          ego_slot_info.slot_corner[2] + ego_slot_info.slot_corner[3]) /
             4.0 -
         pt_center_)
            .norm();
    std::cout << "slot jump dist = " << dist << std::endl;

    CollisionDetector::Paramters params;
    double safe_dist = apa_param.GetParam().safe_dist_for_trim_path;
    if (perpendicular_path_planner_.GetOutput().is_first_reverse_path) {
      params.lat_inflation +=
          apa_param.GetParam().car_lat_inflation_for_trim_path;
      safe_dist += 0.1;
    } else {
      params.lat_inflation +=
          apa_param.GetParam().car_lat_inflation_for_trim_path / 3.0;
    }

    apa_world_ptr_->GetCollisionDetectorPtr()->SetParam(params);
    const double min_remain_dist = 0.016;
    if (dist > apa_param.GetParam().slot_max_jump_dist &&
        frame_.remain_dist > min_remain_dist) {
      // record pt_center_
      pt_center_ =
          (ego_slot_info.slot_corner[0] + ego_slot_info.slot_corner[1] +
           ego_slot_info.slot_corner[2] + ego_slot_info.slot_corner[3]) /
          4.0;
      // construct obs
      GenTlane();
      GenObstacles();
      bool need_trim_path = false;
      double car_remain_dist = 0.0;
      for (const auto& path_seg_global : first_reverse_path_vec_) {
        auto path_seg_local = path_seg_global;
        pnc::geometry_lib::PrintSegmentInfo(path_seg_global);
        CollisionDetector::CollisionResult col_res;
        if (path_seg_global.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
          path_seg_local.line_seg.pA =
              ego_slot_info.g2l_tf.GetPos(path_seg_global.line_seg.pA);
          path_seg_local.line_seg.pB =
              ego_slot_info.g2l_tf.GetPos(path_seg_global.line_seg.pB);
          path_seg_local.line_seg.heading =
              ego_slot_info.g2l_tf.GetHeading(path_seg_global.line_seg.heading);
          pnc::geometry_lib::PrintSegmentInfo(path_seg_local);
          col_res = apa_world_ptr_->GetCollisionDetectorPtr()->UpdateByObsMap(
              path_seg_local.line_seg, path_seg_local.line_seg.heading);
        } else if (path_seg_global.seg_type ==
                   pnc::geometry_lib::SEG_TYPE_ARC) {
          path_seg_local.arc_seg.pA =
              ego_slot_info.g2l_tf.GetPos(path_seg_global.GetArcSeg().pA);
          path_seg_local.arc_seg.pB =
              ego_slot_info.g2l_tf.GetPos(path_seg_global.GetArcSeg().pB);
          path_seg_local.arc_seg.circle_info.center =
              ego_slot_info.g2l_tf.GetPos(
                  path_seg_global.GetArcSeg().circle_info.center);
          path_seg_local.arc_seg.headingA = ego_slot_info.g2l_tf.GetHeading(
              path_seg_global.GetArcSeg().headingA);
          path_seg_local.arc_seg.headingB = ego_slot_info.g2l_tf.GetHeading(
              path_seg_global.GetArcSeg().headingB);
          pnc::geometry_lib::PrintSegmentInfo(path_seg_local);
          col_res = apa_world_ptr_->GetCollisionDetectorPtr()->UpdateByObsMap(
              path_seg_local.arc_seg, path_seg_local.arc_seg.headingA);
        }
        std::cout
            << "remain_obstacle_dist = " << col_res.remain_obstacle_dist
            << "  remain_car_dist = " << col_res.remain_car_dist
            << "  collision_point_local"
            << col_res.collision_point_global.transpose()
            << "  collision_point_global = "
            << ego_slot_info.l2g_tf.GetPos(col_res.collision_point_global)
                   .transpose()
            << "  pt_outside = "
            << ego_slot_info.l2g_tf.GetPos(slot_t_lane_.pt_outside).transpose()
            << "  car_line_order = " << col_res.car_line_order << std::endl;

        if (perpendicular_path_planner_.GetOutput().is_first_reverse_path) {
          if ((slot_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT &&
               col_res.collision_point_global.y() > 0.0) ||
              (slot_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT &&
               col_res.collision_point_global.y() < 0.0)) {
            std::cout
                << "no consider inner stuck in dynamic trim path when 1R\n";
            break;
          }
        }
        if (col_res.remain_obstacle_dist - safe_dist <
            col_res.remain_car_dist) {
          // this path is dangerous, should trim path length, and lose
          // subsequent path
          need_trim_path = true;
          car_remain_dist += col_res.remain_obstacle_dist - safe_dist;
          break;
        }
        car_remain_dist += col_res.remain_car_dist;
      }
      car_remain_dist =
          std::max(apa_param.GetParam().min_gear_path_length, car_remain_dist);
      std::cout << "car_remain_dist = " << car_remain_dist << std::endl;
      if (need_trim_path) {
        PostProcessPathAccordingObs(car_remain_dist);
      }
    }
    params.Reset();
    apa_world_ptr_->GetCollisionDetectorPtr()->SetParam(params);
  }

  // update stuck uss time
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag &&
      apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) {
    frame_.stuck_uss_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_uss_time = 0.0;
  }

  // update stuck time
  if ((frame_.plan_stm.planning_status == PARKING_RUNNING ||
       frame_.plan_stm.planning_status == PARKING_PLANNING) &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag &&
      apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) {
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
      !frame_.is_fix_slot &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag) {
    // apa_world_ptr_->GetSlotManagerPtr()->SetRealtime();
    frame_.is_fix_slot = true;
  }

  // calc slot side and init gear and init steer at first
  if (frame_.is_replan_first == true) {
    const auto pM01 =
        0.5 * (ego_slot_info.slot_corner[0] + ego_slot_info.slot_corner[1]);
    const auto pM23 =
        0.5 * (ego_slot_info.slot_corner[2] + ego_slot_info.slot_corner[3]);
    Eigen::Vector2d ego_to_slot_center_vec =
        0.5 * (pM01 + pM23) - measures_ptr->pos_ego;

    const double cross_ego_to_slot_center =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->heading_ego_vec,
                                                ego_to_slot_center_vec);

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(
            measures_ptr->heading_ego_vec,
            ego_slot_info.slot_origin_heading_vec);

    // judge slot side via slot center and heading
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
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
      std::cout << "calculate slot side error " << std::endl;
      // return false;
    }
  }

  std::cout << "slot_side = " << static_cast<int>(slot_t_lane_.slot_side)
            << std::endl;
  std::cout << "frame_.current_gear = " << static_cast<int>(frame_.current_gear)
            << std::endl;
  std::cout << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer) << std::endl;

  std::cout << "ego_pos = " << measures_ptr->pos_ego.transpose()
            << "  ego_heading = " << measures_ptr->heading_ego * 57.3
            << "  ego_pos_slot = " << ego_slot_info.ego_pos_slot.transpose()
            << "  ego_heading_slot = " << ego_slot_info.ego_heading_slot * 57.3
            << std::endl;

  std::cout << "ego_slot_info.limiter.first = "
            << ego_slot_info.limiter.first.transpose()
            << "  ego_slot_info.limiter.second = "
            << ego_slot_info.limiter.second.transpose() << std::endl;

  std::cout << "target_ego_pos_slot = "
            << ego_slot_info.target_ego_pos_slot.transpose()
            << "  target_ego_heading_slot = "
            << ego_slot_info.target_ego_heading_slot * 57.3 << std::endl;

  std::cout << "terminal x error= " << ego_slot_info.terminal_err.pos.x()
            << std::endl;
  std::cout << "terminal y error= " << ego_slot_info.terminal_err.pos.y()
            << std::endl;
  std::cout << "terminal heading error= "
            << ego_slot_info.terminal_err.heading * 57.3 << std::endl;

  std::cout << "slot_occupied_ratio = " << ego_slot_info.slot_occupied_ratio
            << std::endl;

  std::cout << "vel_ego = " << measures_ptr->vel_ego << std::endl;
  std::cout << "stuck_time = " << frame_.stuck_time << " s\n";
  std::cout << "static_flag = "
            << apa_world_ptr_->GetMeasurementsPtr()->static_flag << "\n";

  return true;
}

void PerpendicularInPlanner::GenTlane() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  const auto lambda_func_1 = [](const Eigen::Vector2d& a,
                                const Eigen::Vector2d& b) {
    return a.y() > b.y();  // the top element is smallest
  };
  const auto lambda_func_2 = [](const Eigen::Vector2d& a,
                                const Eigen::Vector2d& b) {
    return a.y() < b.y();  // the top element is largest
  };

  const auto lambda_func_3 = [](const Eigen::Vector2d& a,
                                const Eigen::Vector2d& b) {
    return a.x() < b.x();  // the top element is largest
  };

  // left y is positive, right y is negative
  // left y should be smallest, right y should be largest
  // all x should be largest
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      decltype(lambda_func_1)>
      left_que_for_y(lambda_func_1);

  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      decltype(lambda_func_3)>
      left_que_for_x(lambda_func_3);

  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      decltype(lambda_func_2)>
      right_que_for_y(lambda_func_2);

  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>,
                      decltype(lambda_func_3)>
      right_que_for_x(lambda_func_3);

  const double x_max = ((ego_slot_info.pt_0 + ego_slot_info.pt_1) * 0.5).x() +
                       apa_param.GetParam().obs_consider_long_threshold /
                           ego_slot_info.sin_angle;

  const double y_max =
      ((ego_slot_info.slot_width / ego_slot_info.sin_angle) * 0.5 +
       apa_param.GetParam().obs_consider_lat_threshold) *
      ego_slot_info.sin_angle;

  // sift obstacles that meet requirement
  for (const auto& obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
    if (std::fabs(obstacle_point_slot.x()) > x_max ||
        std::fabs(obstacle_point_slot.y()) > y_max) {
      continue;
    }
    if (obstacle_point_slot.y() > 1e-6) {
      left_que_for_y.emplace(obstacle_point_slot);
      left_que_for_x.emplace(obstacle_point_slot);
    } else {
      right_que_for_y.emplace(obstacle_point_slot);
      right_que_for_x.emplace(obstacle_point_slot);
    }
  }

  if (apa_param.GetParam().conservative_mono_enable) {
    if (!left_que_for_x.empty() || !right_que_for_x.empty()) {
      apa_param.SetPram().mono_plan_enable = false;
    }
  }

  // If there are no obstacles on either side, set up a virtual obstacle that is
  // farther away
  if (left_que_for_x.empty()) {
    std::cout << "left space is empty\n";
    left_que_for_x.emplace(
        Eigen::Vector2d(apa_param.GetParam().virtual_obs_x_pos, 0.0));
    left_que_for_y.emplace(
        Eigen::Vector2d(0.0, apa_param.GetParam().virtual_obs_y_pos));
  }
  if (right_que_for_x.empty()) {
    std::cout << "right space is empty\n";
    right_que_for_x.emplace(
        Eigen::Vector2d(apa_param.GetParam().virtual_obs_x_pos, 0.0));
    right_que_for_y.emplace(
        Eigen::Vector2d(0.0, -apa_param.GetParam().virtual_obs_y_pos));
  }

  const double car_width_include_mirror =
      apa_param.GetParam().car_width + 2.0 * apa_param.GetParam().mirror_width;
  const double car_y_right_include_mirror = -car_width_include_mirror * 0.5;
  const double car_y_left_include_mirror = car_width_include_mirror * 0.5;

  const double virtual_slot_width =
      car_width_include_mirror + apa_param.GetParam().slot_compare_to_car_width;

  const double real_slot_width = ego_slot_info.slot_width;

  std::cout << "car_width_include_mirror = " << car_width_include_mirror
            << "  virtual slot width = " << virtual_slot_width
            << "  real slot width = " << real_slot_width << std::endl;

  const double safe_threshold = apa_param.GetParam().safe_threshold;

  double left_y = left_que_for_y.top().y();

  double right_y = right_que_for_y.top().y();

  if (apa_param.GetParam().tmp_no_consider_obs_dy) {
    left_y = real_slot_width * 0.5 + apa_param.GetParam().tmp_virtual_obs_dy;

    right_y = -real_slot_width * 0.5 - apa_param.GetParam().tmp_virtual_obs_dy;
  }

  std::cout << "left_y = " << left_y << "  right_y = " << right_y << std::endl;

  const double left_dis_obs_car = left_y - car_y_left_include_mirror;

  const double right_dis_obs_car = car_y_right_include_mirror - right_y;

  std::cout << "left_dis_obs_car = " << left_dis_obs_car
            << "  right_dis_obs_car = " << right_dis_obs_car << std::endl;

  bool left_obs_meet_safe_require = false;
  bool right_obs_meet_safe_require = false;

  left_obs_meet_safe_require = left_dis_obs_car > safe_threshold ? true : false;
  right_obs_meet_safe_require =
      right_dis_obs_car > safe_threshold ? true : false;

  bool need_move_slot = false;
  double move_slot_dist = 0.0;

  if (!left_obs_meet_safe_require && right_obs_meet_safe_require) {
    // left side is dangerous, should move toward right
    need_move_slot = true;
    move_slot_dist = safe_threshold - left_dis_obs_car;
    move_slot_dist *= -1.0;
  } else if (left_obs_meet_safe_require && !right_obs_meet_safe_require) {
    // right side is dangerous, should move toward left
    need_move_slot = true;
    move_slot_dist = safe_threshold - right_dis_obs_car;
  }

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
        right_que_for_x.top().x() + apa_param.GetParam().tlane_safe_dx;
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    // outside is right, inside is left
    slot_t_lane_.corner_outside_slot = corner_right_slot;
    slot_t_lane_.corner_inside_slot = corner_left_slot;
    slot_t_lane_.pt_outside = corner_right_slot;
    slot_t_lane_.pt_inside = corner_left_slot;
    slot_t_lane_.pt_inside.x() =
        left_que_for_x.top().x() + apa_param.GetParam().tlane_safe_dx;
  }

  slot_t_lane_.pt_terminal_pos << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  slot_t_lane_.pt_terminal_heading = ego_slot_info.target_ego_heading_slot;

  if (need_move_slot) {
    slot_t_lane_.pt_terminal_pos.y() += move_slot_dist;
    slot_t_lane_.pt_inside.y() += move_slot_dist;
    slot_t_lane_.pt_outside.y() += move_slot_dist;
    std::cout << "should move slot according to obs pt\n";
  }

  slot_t_lane_.pt_lower_boundry_pos = slot_t_lane_.pt_terminal_pos;
  slot_t_lane_.pt_lower_boundry_pos.x() =
      slot_t_lane_.pt_lower_boundry_pos.x() -
      apa_param.GetParam().rear_overhanging -
      apa_param.GetParam().col_obs_safe_dist - 0.05;

  // construct obstacle_t_lane_
  // for onstacle_t_lane    right is inside, left is outside
  obstacle_t_lane_.slot_side = slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    obstacle_t_lane_.pt_inside.x() =
        right_que_for_x.top().x() + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_inside.y() = right_y;
    obstacle_t_lane_.pt_outside.x() =
        left_que_for_x.top().x() + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_outside.y() = left_y;
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    obstacle_t_lane_.pt_inside.x() =
        left_que_for_x.top().x() + apa_param.GetParam().obs_safe_dx;
    obstacle_t_lane_.pt_inside.y() = left_y;
    obstacle_t_lane_.pt_outside.x() =
        right_que_for_x.top().x() + apa_param.GetParam().obs_safe_dx;
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

  std::cout << "-- slot_t_lane_ --" << std::endl;
  std::cout << "pt_outside = " << slot_t_lane_.pt_outside.transpose()
            << std::endl;
  std::cout << "pt_inside = " << slot_t_lane_.pt_inside.transpose()
            << std::endl;
  std::cout << "pt_terminal_pos = " << slot_t_lane_.pt_terminal_pos.transpose()
            << std::endl;
  std::cout << "pt_terminal_heading = " << slot_t_lane_.pt_terminal_heading
            << std::endl;
  std::cout << "pt_lower_boundry_pos = "
            << slot_t_lane_.pt_lower_boundry_pos.transpose() << std::endl;

  std::cout << "-- obstacle_t_lane_ --" << std::endl;
  std::cout << "pt_outside = " << obstacle_t_lane_.pt_outside.transpose()
            << std::endl;
  std::cout << "pt_inside = " << obstacle_t_lane_.pt_inside.transpose()
            << std::endl;
  std::cout << "pt_terminal_pos = "
            << obstacle_t_lane_.pt_terminal_pos.transpose() << std::endl;
  std::cout << "pt_terminal_heading = " << obstacle_t_lane_.pt_terminal_heading
            << std::endl;
  std::cout << "pt_lower_boundry_pos = "
            << obstacle_t_lane_.pt_lower_boundry_pos.transpose() << std::endl;
}

void PerpendicularInPlanner::GenObstacles() {
  apa_world_ptr_->GetCollisionDetectorPtr()->ClearObstacles();
  // set obstacles
  double channel_width = apa_param.GetParam().channel_width;
  double channel_length = apa_param.GetParam().channel_length;

  if (apa_param.GetParam().force_both_side_occupied) {
    obstacle_t_lane_ = slot_t_lane_;
  }

  // add tlane obstacle
  //  B is always outside
  int slot_side = 1;
  if (obstacle_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    slot_side = -1;
  } else if (obstacle_t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    slot_side = 1;
  }
  Eigen::Vector2d B(obstacle_t_lane_.pt_outside);
  const auto& ego_slot_info = frame_.ego_slot_info;
  const auto pt_01_vec = ego_slot_info.pt_1 - ego_slot_info.pt_0;
  const auto obs_length = (channel_length - pt_01_vec.norm()) * 0.5;
  Eigen::Vector2d A = B - slot_side * pt_01_vec.normalized() * obs_length;

  Eigen::Vector2d C(obstacle_t_lane_.pt_lower_boundry_pos);
  C.y() = B.y();

  Eigen::Vector2d E(obstacle_t_lane_.pt_inside);
  Eigen::Vector2d D(obstacle_t_lane_.pt_lower_boundry_pos);
  D.y() = E.y();

  Eigen::Vector2d F = E + slot_side * pt_01_vec.normalized() * obs_length;

  // add channel obstacle
  const double pt_01_x = ((ego_slot_info.pt_0 + ego_slot_info.pt_1) * 0.5).x();
  const double top_x = pt_01_x + channel_width / ego_slot_info.sin_angle;
  Eigen::Vector2d channel_point_1 =
      Eigen::Vector2d(top_x, 0.0) -
      slot_side * pt_01_vec.normalized() * channel_length * 0.5;
  Eigen::Vector2d channel_point_2 =
      Eigen::Vector2d(top_x, 0.0) +
      slot_side * pt_01_vec.normalized() * channel_length * 0.5;

  Eigen::Vector2d channel_point_3;
  pnc::geometry_lib::LineSegment channel_line;
  std::vector<pnc::geometry_lib::LineSegment> channel_line_vec;
  channel_line.SetPoints(channel_point_1, channel_point_2);
  channel_line_vec.emplace_back(channel_line);
  channel_point_3 = F;
  channel_line.SetPoints(channel_point_2, channel_point_3);
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
  if (frame_.ego_slot_info.slot_occupied_ratio < 0.001 &&
      std::fabs(frame_.ego_slot_info.ego_heading_slot) * 57.3 > 48.68 &&
      false) {
  } else {
    tlane_line.SetPoints(D, E);
    tlane_line_vec.emplace_back(tlane_line);
    tlane_line.SetPoints(E, F);
    tlane_line_vec.emplace_back(tlane_line);
  }

  // std::cout << "A = " << A.transpose() << "  B = " << B.transpose()
  //           << "  C = " << C.transpose() << "  D = " << D.transpose()
  //           << "  E = " << E.transpose() << "  F = " << F.transpose()
  //           << "  channel1 = " << channel_point_1.transpose()
  //           << "  channel2 = " << channel_point_2.transpose()
  //           << "  channel3 = " << channel_point_3.transpose() << std::endl;

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
  double safe_dist = apa_param.GetParam().max_obs2car_dist_out_slot;
  if (frame_.ego_slot_info.slot_occupied_ratio >
          apa_param.GetParam().max_obs2car_dist_slot_occupied_ratio &&
      std::fabs(frame_.ego_slot_info.terminal_err.heading) * 57.3 < 36.6) {
    safe_dist = apa_param.GetParam().max_obs2car_dist_in_slot;
  }
  for (const auto& obs_pos : tlane_obstacle_vec) {
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs_pos, ego_pose, safe_dist)) {
      // apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(obs_pos);
      apa_world_ptr_->GetCollisionDetectorPtr()->AddObstacles(
          obs_pos, CollisionDetector::TLANE_OBS);
    }
  }
}

const uint8_t PerpendicularInPlanner::PathPlanOnce() {
  std::cout << "-------------- PathPlanOnce --------------" << std::endl;
  // construct input
  const auto& ego_slot_info = frame_.ego_slot_info;
  PerpendicularPathPlanner::Input path_planner_input;
  path_planner_input.pt_0 = ego_slot_info.pt_0;
  path_planner_input.pt_1 = ego_slot_info.pt_1;
  path_planner_input.sin_angle = ego_slot_info.sin_angle;
  path_planner_input.origin_pt_0_heading = ego_slot_info.origin_pt_0_heading;
  path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  path_planner_input.tlane = slot_t_lane_;
  path_planner_input.is_complete_path = simu_param_.is_complete_path;
  path_planner_input.sample_ds = simu_param_.sample_ds;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;
  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_replan_second = frame_.is_replan_second;
  path_planner_input.is_replan_dynamic = frame_.is_replan_dynamic;
  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);

  if (frame_.replan_reason == DYNAMIC &&
      gear_command_ == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    std::cout << "dynamic replan, gear should be reverse\n";
    path_planner_input.ref_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
  }

  perpendicular_path_planner_.SetInput(path_planner_input);

  // need replan all path
  const bool path_plan_success = perpendicular_path_planner_.Update(
      apa_world_ptr_->GetCollisionDetectorPtr());

  uint8_t plan_result = 0;
  if (!path_plan_success && !is_simulation_ && !frame_.is_replan_dynamic) {
    std::cout << "path plan fail\n";
    plan_result = PathPlannerResult::PLAN_FAILED;
    return plan_result;
  }

  if (!path_plan_success && frame_.is_replan_dynamic) {
    std::cout << "path dynamic plan fail, save last plan path.\n";
    plan_result = PathPlannerResult::PLAN_UPDATE;
    return plan_result;
  }

  plan_result = PathPlannerResult::PLAN_UPDATE;

  if (!perpendicular_path_planner_.SetCurrentPathSegIndex()) {
    std::cout << "path plan fail\n";
    plan_result = PathPlannerResult::PLAN_FAILED;
    return plan_result;
  }

  perpendicular_path_planner_.SetLineSegmentHeading();

  perpendicular_path_planner_.InsertLineSegAfterCurrentFollowLastPath(
      apa_param.GetParam().path_extend_distance);

  perpendicular_path_planner_.SampleCurrentPathSeg();

  perpendicular_path_planner_.PrintOutputSegmentsInfo();

  const auto& planner_output = perpendicular_path_planner_.GetOutput();

  first_reverse_path_vec_.clear();
  first_reverse_path_vec_.reserve(5);
  if (planner_output.is_first_reverse_path ||
      apa_param.GetParam().dynamic_col_det_enable) {
    for (size_t i = planner_output.path_seg_index.first;
         i <= planner_output.path_seg_index.second; ++i) {
      const auto& path_seg_local = planner_output.path_segment_vec[i];
      if (path_seg_local.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
        pnc::geometry_lib::PathSegment path_seg_global = path_seg_local;
        if (path_seg_local.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
          path_seg_global.line_seg.pA =
              ego_slot_info.l2g_tf.GetPos(path_seg_local.GetLineSeg().pA);
          path_seg_global.line_seg.pB =
              ego_slot_info.l2g_tf.GetPos(path_seg_local.GetLineSeg().pB);
          path_seg_global.line_seg.heading = ego_slot_info.l2g_tf.GetHeading(
              path_seg_local.GetLineSeg().heading);
        } else if (path_seg_local.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
          path_seg_global.arc_seg.pA =
              ego_slot_info.l2g_tf.GetPos(path_seg_local.GetArcSeg().pA);
          path_seg_global.arc_seg.pB =
              ego_slot_info.l2g_tf.GetPos(path_seg_local.GetArcSeg().pB);
          path_seg_global.arc_seg.circle_info.center =
              ego_slot_info.l2g_tf.GetPos(
                  path_seg_local.GetArcSeg().circle_info.center);
          path_seg_global.arc_seg.headingA = ego_slot_info.l2g_tf.GetHeading(
              path_seg_local.GetArcSeg().headingA);
          path_seg_global.arc_seg.headingB = ego_slot_info.l2g_tf.GetHeading(
              path_seg_local.GetArcSeg().headingB);
        }
        // pnc::geometry_lib::PrintSegmentInfo(path_seg_local);
        // pnc::geometry_lib::PrintSegmentInfo(path_seg_global);
        first_reverse_path_vec_.emplace_back(std::move(path_seg_global));
      }
    }
  }

  const bool gear_steer_shift =
      (frame_.is_replan_first && planner_output.gear_shift) ||
      (frame_.is_replan_second && planner_output.gear_shift) ||
      (!frame_.is_replan_first && !frame_.is_replan_second &&
       frame_.replan_reason != DYNAMIC);

  if (gear_steer_shift) {
    std::cout << "next plan should shift gear\n";
    // set current arc steer
    if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else if (frame_.current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else {
      std::cout << "fault ref_arc_steer state!" << std::endl;
      return false;
    }

    // set current gear
    if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    } else if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    } else {
      std::cout << "fault ref_gear state!" << std::endl;
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

  gear_command_ = planner_output.current_gear;

  // lateral path optimization
  bool is_use_optimizer = true;

  // refuse optimizer
  if (planner_output.path_point_vec.size() < 3) {
    std::cout << " input size is too small" << std::endl;
    is_use_optimizer = false;
  } else {
    const auto path_length = (planner_output.path_point_vec.front().pos -
                              planner_output.path_point_vec.back().pos)
                                 .norm();
    if (path_length < apa_param.GetParam().min_opt_path_length) {
      std::cout << "path length is too short, optimizer is closed "
                << std::endl;
      is_use_optimizer = false;
    }
  }

  bool cilqr_optimization_enable = true;
  bool perpendicular_optimization_enable = true;

  if (!is_simulation_) {
    perpendicular_optimization_enable =
        apa_param.GetParam().perpendicular_lat_opt_enable;

    cilqr_optimization_enable =
        apa_param.GetParam().cilqr_path_optimization_enable;

  } else {
    perpendicular_optimization_enable = simu_param_.is_path_optimization;
    cilqr_optimization_enable = simu_param_.is_cilqr_optimization;
  }

  double lat_path_opt_cost_time_ms = 0.0;
  if (perpendicular_optimization_enable && is_use_optimizer) {
    std::cout << "------------------------ lateral path optimization "
                 "------------------------"
              << std::endl;
    std::cout << "gear_command_= " << static_cast<int>(gear_command_)
              << std::endl;
    std::cout << "origin path size= " << planner_output.path_point_vec.size()
              << std::endl;

    LateralPathOptimizer::Parameter param;
    param.sample_ds = simu_param_.sample_ds;
    param.q_ref_xy = simu_param_.q_ref_xy;
    param.q_ref_theta = simu_param_.q_ref_theta;
    param.q_terminal_xy = simu_param_.q_terminal_xy;
    param.q_terminal_theta = simu_param_.q_terminal_theta;
    param.q_k = simu_param_.q_k;
    param.q_u = simu_param_.q_u;
    param.q_k_bound = simu_param_.q_k_bound;
    param.q_u_bound = simu_param_.q_u_bound;

    lateral_path_optimizer_ptr_->Init(cilqr_optimization_enable);
    lateral_path_optimizer_ptr_->SetParam(param);
    auto time_start = IflyTime::Now_ms();
    lateral_path_optimizer_ptr_->Update(planner_output.path_point_vec,
                                        gear_command_);

    auto time_end = IflyTime::Now_ms();
    lat_path_opt_cost_time_ms = time_end - time_start;

    const auto& optimized_path_vec =
        lateral_path_optimizer_ptr_->GetOutputPathVec();
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
        lateral_path_optimizer_ptr_->GetOutputDebugInfo();

    std::cout << "lat_path_opt_cost_time_ms = " << lat_path_opt_cost_time_ms
              << std::endl;

    std::cout << "terminal point error = "
              << plan_debug_info.terminal_pos_error() << std::endl;

    std::cout << "terminal heading error = "
              << plan_debug_info.terminal_heading_error() << std::endl;
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

  std::cout << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size() << std::endl;

  return plan_result;
}

const bool PerpendicularInPlanner::CheckSegCompleted() {
  bool is_seg_complete = false;
  if (frame_.spline_success) {
    const auto min_remain_dist =
        std::min(frame_.remain_dist_uss, frame_.remain_dist);

    if (min_remain_dist < apa_param.GetParam().max_replan_remain_dist &&
        apa_world_ptr_->GetMeasurementsPtr()->static_flag) {
      frame_.is_replan_by_uss = (frame_.remain_dist_uss < frame_.remain_dist);

      if (!frame_.is_replan_by_uss) {
        std::cout << "close to target!\n";
        is_seg_complete = true;
      } else {
        std::cout << "close to obstacle by uss!\n";
        // uss distance may not be accurate, so wait for a period of time
        if (frame_.stuck_uss_time >
            apa_param.GetParam().uss_stuck_replan_wait_time) {
          is_seg_complete = true;
        }
      }
    }
  }

  return is_seg_complete;
}

void PerpendicularInPlanner::UpdateSlotRealtime() {
  bool update_slot = false;

  const size_t replan_reaon = frame_.replan_reason;
  if (replan_reaon == FIRST_PLAN || replan_reaon == DYNAMIC) {
    update_slot = true;
  }

  if (replan_reaon == SEG_COMPLETED_USS) {
    if (frame_.ego_slot_info.slot_occupied_ratio >
        apa_param.GetParam().uss_slot_occupied_ratio) {
      update_slot = true;
    }
  }

  if (replan_reaon == SEG_COMPLETED_PATH) {
    if (std::fabs(frame_.ego_slot_info.ego_heading_slot) <
            apa_param.GetParam().path_heading_slot / 57.3 &&
        frame_.ego_slot_info.terminal_err.pos.norm() <
            apa_param.GetParam().path_pos_err) {
      update_slot = true;
    }
  }

  if (update_slot && frame_.need_update_slot) {
    std::cout << " real time update slot\n";
    apa_world_ptr_->GetSlotManagerPtr()->SetRealtime();
    apa_world_ptr_->Update();
    UpdateEgoSlotInfo();
  }

  if (frame_.ego_slot_info.slot_occupied_ratio >
      apa_param.GetParam().last_update_slot_occupied_ratio) {
    frame_.need_update_slot = false;
  }
}

const bool PerpendicularInPlanner::CheckDynamicUpdate() {
  bool update_flag = false;
  // first update, conditions are relatively relaxed
  if (frame_.dynamic_replan_count == 0) {
    bool condition_1 = std::fabs(frame_.ego_slot_info.terminal_err.pos.y()) <
                           apa_param.GetParam().pose_y_err &&
                       std::fabs(frame_.ego_slot_info.terminal_err.heading) <
                           apa_param.GetParam().pose_heading_err / 57.3;

    bool condition_2 = frame_.ego_slot_info.slot_occupied_ratio >
                       apa_param.GetParam().pose_slot_occupied_ratio;

    if (condition_1 && condition_2) {
      frame_.dynamic_replan_count++;
      update_flag = true;
    }
  }

  // second update
  if (frame_.dynamic_replan_count == 1) {
    bool condition_1 = std::fabs(frame_.ego_slot_info.terminal_err.pos.y()) >
                           apa_param.GetParam().max_y_err_2 ||
                       std::fabs(frame_.ego_slot_info.terminal_err.heading) >
                           apa_param.GetParam().max_heading_err_2 / 57.3;

    bool condition_2 = frame_.ego_slot_info.slot_occupied_ratio >
                       apa_param.GetParam().pose_slot_occupied_ratio_2;

    bool condition_3 =
        frame_.remain_dist > apa_param.GetParam().pose_min_remain_dis;

    if (condition_1 && condition_2 && condition_3) {
      frame_.dynamic_replan_count++;
      update_flag = true;
    }
  }

  // third update
  if (frame_.dynamic_replan_count == 2) {
    bool condition_1 = std::fabs(frame_.ego_slot_info.terminal_err.pos.y()) >
                           apa_param.GetParam().max_y_err_3 ||
                       std::fabs(frame_.ego_slot_info.terminal_err.heading) >
                           apa_param.GetParam().max_heading_err_3 / 57.3;

    bool condition_2 = frame_.ego_slot_info.slot_occupied_ratio >
                       apa_param.GetParam().pose_slot_occupied_ratio_3;

    bool condition_3 =
        frame_.remain_dist > apa_param.GetParam().pose_min_remain_dis;

    if (condition_1 && condition_2 && condition_3) {
      frame_.dynamic_replan_count++;
      update_flag = true;
    }
  }

  std::cout << "dynamic_replan_count = "
            << static_cast<int>(frame_.dynamic_replan_count) << std::endl;

  update_flag =
      (update_flag && gear_command_ == pnc::geometry_lib::SEG_GEAR_REVERSE &&
       !apa_world_ptr_->GetMeasurementsPtr()->static_flag);

  frame_.is_replan_dynamic = update_flag;

  return update_flag;
}

const bool PerpendicularInPlanner::CheckReplan() {
  if (frame_.is_replan_first == true) {
    std::cout << "first plan" << std::endl;
    frame_.replan_reason = FIRST_PLAN;
    return true;
  }

  frame_.is_replan_by_uss = false;
  frame_.is_replan_dynamic = false;

  if (CheckSegCompleted()) {
    DEBUG_PRINT("replan by current segment completed!");
    frame_.replan_reason = SEG_COMPLETED_PATH;
    if (frame_.is_replan_by_uss) {
      frame_.replan_reason = SEG_COMPLETED_USS;
    }
    return true;
  }

  if (frame_.stuck_uss_time > apa_param.GetParam().stuck_replan_time) {
    DEBUG_PRINT("replan by stuck!");
    frame_.replan_reason = STUCKED;
    return true;
  }

  if (CheckDynamicUpdate()) {
    std::cout << "replan by dynamic!" << std::endl;
    frame_.replan_reason = DYNAMIC;
    return true;
  }

  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool PerpendicularInPlanner::CheckStuckFailed() {
  return frame_.stuck_time > apa_param.GetParam().stuck_failed_time;
}

const bool PerpendicularInPlanner::CheckFinished() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  const bool lon_condition =
      ego_slot_info.terminal_err.pos.x() < apa_param.GetParam().finish_lon_err;

  const double y1 = ego_slot_info.ego_pos_slot.y();
  const double y2 =
      (ego_slot_info.ego_pos_slot + (apa_param.GetParam().wheel_base +
                                     apa_param.GetParam().front_overhanging) *
                                        ego_slot_info.ego_heading_slot_vec)
          .y();

  const bool lat_condition_1 =
      std::fabs(y1) <= apa_param.GetParam().finish_lat_err;

  const bool lat_condition_2 =
      std::fabs(y1) <= apa_param.GetParam().finish_lat_err_strict &&
      std::fabs(y2) <= apa_param.GetParam().finish_lat_err_strict;

  const bool heading_condition_1 =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      apa_param.GetParam().finish_heading_err / 57.3;

  const bool heading_condition_2 =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      (apa_param.GetParam().finish_heading_err + 1.988) / 57.3;

  const bool lat_condition = (lat_condition_1 && heading_condition_1) &&
                             (lat_condition_2 && heading_condition_2);

  const bool static_condition =
      apa_world_ptr_->GetMeasurementsPtr()->static_flag;

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

  return parking_finish;
}

void PerpendicularInPlanner::GenPlanningOutput() {
  pnc::geometry_lib::PathPoint current_ego_pose(
      apa_world_ptr_->GetMeasurementsPtr()->pos_ego,
      apa_world_ptr_->GetMeasurementsPtr()->heading_ego);

  if (frame_.plan_stm.planning_status == PARKING_FINISHED) {
    SetFinishedPlanningOutput(planning_output_, current_ego_pose);
  } else if (frame_.plan_stm.planning_status == PARKING_FAILED) {
    SetFailedPlanningOutput(planning_output_, current_ego_pose);
  } else if (frame_.plan_stm.planning_status == PARKING_PLANNING ||
             frame_.plan_stm.planning_status == PARKING_GEARCHANGE ||
             frame_.plan_stm.planning_status == PARKING_RUNNING ||
             frame_.plan_stm.planning_status == PARKING_PAUSED) {
    GenPlanningPath();
  } else if (frame_.plan_stm.planning_status == PARKING_IDLE) {
    SetIdlePlanningOutput(planning_output_, current_ego_pose);
  }

  // std::cout << "planning_output:" << planning_output_.DebugString()
  //           << std::endl;
}

void PerpendicularInPlanner::GenPlanningPath() {
  planning_output_.Clear();
  planning_output_.mutable_planning_status()->set_apa_planning_status(
      ::PlanningOutput::ApaPlanningStatus::IN_PROGRESS);

  auto trajectory = planning_output_.mutable_trajectory();
  trajectory->set_available(true);

  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);

  for (const auto& global_point : current_path_point_global_vec_) {
    auto trajectory_point = trajectory->add_trajectory_points();
    trajectory_point->set_x(global_point.pos.x());
    trajectory_point->set_y(global_point.pos.y());
    trajectory_point->set_heading_yaw(global_point.heading);
    trajectory_point->set_v(0.5);
  }

  // set target velocity to control as a limit
  const std::vector<double> ratio_tab = {0.0, 0.4, 0.8, 1.0};
  const std::vector<double> vel_limit_tab = {apa_param.GetParam().max_velocity,
                                             apa_param.GetParam().max_velocity,
                                             0.45, 0.35};
  const double vel_limit = pnc::mathlib::Interp1(
      ratio_tab, vel_limit_tab, frame_.ego_slot_info.slot_occupied_ratio);

  planning_output_.mutable_trajectory()
      ->mutable_target_reference()
      ->set_target_velocity(vel_limit);

  // send uss remain dist to control
  planning_output_.mutable_trajectory()
      ->mutable_trajectory_points(0)
      ->set_distance(frame_.remain_dist_uss);

  // send slot occupation ratio to control
  planning_output_.mutable_trajectory()
      ->mutable_trajectory_points(1)
      ->set_distance(frame_.ego_slot_info.slot_occupied_ratio);

  // send slot type to control
  planning_output_.mutable_trajectory()
      ->mutable_trajectory_points(2)
      ->set_distance(static_cast<double>(
          Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL));

  // set plan gear cmd
  auto gear_command = planning_output_.mutable_gear_command();
  gear_command->set_available(true);

  if (gear_command_ == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    gear_command->set_gear_command_value(
        Common::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE);
  } else {
    gear_command->set_gear_command_value(
        Common::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE);
  }
}

void PerpendicularInPlanner::InitSimulation() {
  if (is_simulation_ && simu_param_.is_reset) {
    Reset();
  }
}

const bool PerpendicularInPlanner::CheckPlanSkip() const {
  if (frame_.plan_stm.planning_status == PARKING_FINISHED ||
      frame_.plan_stm.planning_status == PARKING_FAILED) {
    std::cout << "plan has been finished or failed, need reset" << std::endl;

    if (!is_simulation_) {
      apa_world_ptr_->GetSlotManagerPtr()->Reset();
    }

    return true;
  } else {
    return false;
  }
}

const bool PerpendicularInPlanner::CheckPaused() {
  if (apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_SUSPEND_ACTIVATE ||
      apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_SUSPEND_CLOSE) {
    return true;
  } else {
    return false;
  }
}

const double PerpendicularInPlanner::CalRemainDistFromPath() {
  double remain_dist = 5.01;

  if (frame_.is_replan_first) {
    return remain_dist;
  }

  if (frame_.spline_success) {
    double s_proj = 0.0;
    bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
        0.0, frame_.current_path_length + frame_.path_extended_dist, s_proj,
        apa_world_ptr_->GetMeasurementsPtr()->pos_ego, frame_.x_s_spline,
        frame_.y_s_spline);

    if (success == true) {
      remain_dist = frame_.current_path_length - s_proj;

      std::cout << "remain_dist = " << remain_dist << "  s_proj = " << s_proj
                << "  current_path_length = " << frame_.current_path_length
                << std::endl;
    } else {
      std::cout << "remain_dist calculation error:input is error" << std::endl;
    }
  } else {
    std::cout << "remain_dist calculation error: path spline failed!"
              << std::endl;
  }

  return remain_dist;
}

const double PerpendicularInPlanner::CalRemainDistFromUss() {
  double remain_dist = 5.01;

  // if (frame_.is_replan_first) {
  //   return remain_dist;
  // }
  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();

  uss_obstacle_avoider_ptr->Update(&planning_output_,
                                   apa_world_ptr_->GetLocalViewPtr());

  const double safe_uss_remain_dist =
      (frame_.ego_slot_info.slot_occupied_ratio < 0.05)
          ? apa_param.GetParam().safe_uss_remain_dist_out_slot
          : apa_param.GetParam().safe_uss_remain_dist_in_slot;

  remain_dist = uss_obstacle_avoider_ptr->GetRemainDistInfo().remain_dist -
                safe_uss_remain_dist;

  std::cout << "uss remain dist = " << remain_dist << std::endl;

  // remain_dist = 5.01;

  return remain_dist;
}

void PerpendicularInPlanner::UpdateRemainDist() {
  // 1. calculate remain dist according to plan path
  frame_.remain_dist = CalRemainDistFromPath();

  // 2.calculate remain dist uss according to uss
  frame_.remain_dist_uss = CalRemainDistFromUss();

  return;
}

const bool PerpendicularInPlanner::PostProcessPathAccordingLimiter() {
  size_t origin_traj_size = current_path_point_global_vec_.size();

  if (origin_traj_size < 2) {
    frame_.spline_success = false;
    std::cout << "error: origin_traj_size = " << origin_traj_size << std::endl;
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
    std::cout << "path is err\n";
    return false;
  }
  if (s_proj < simu_param_.sample_ds * 1.5) {
    std::cout << "limiter s_proj is too small\n";
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
      std::cout << "path shoule be shorten because of limiter\n";
      break;
    }
    x_vec.emplace_back(current_path_point_global_vec_[i].pos.x());
    y_vec.emplace_back(current_path_point_global_vec_[i].pos.y());
    heading_vec.emplace_back(current_path_point_global_vec_[i].heading);
    s_vec.emplace_back(s);
  }
  if (s < s_proj) {
    std::cout << "path shoule be extended because of limiter\n";
    while (s <= s_proj) {
      s += simu_param_.sample_ds;
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
    std::cout << "error: no enough point = " << x_vec.size() << std::endl;
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
    std::cout << "limit need extend fit line by spline error!" << std::endl;
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

const bool PerpendicularInPlanner::PostProcessPathAccordingObs(
    const double& car_remain_dist) {
  size_t origin_traj_size = current_path_point_global_vec_.size();

  if (origin_traj_size < 2) {
    frame_.spline_success = false;
    std::cout << "error: origin_traj_size = " << origin_traj_size << std::endl;
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
      std::cout << "path shoule be shorten because of obs\n";
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
    std::cout << "error: no enough point = " << x_vec.size() << std::endl;
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
    std::cout << "limit need extend fit line by spline error!" << std::endl;
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

const bool PerpendicularInPlanner::PostProcessPath() {
  size_t origin_trajectory_size = current_path_point_global_vec_.size();

  if (origin_trajectory_size < 2) {
    frame_.spline_success = false;
    std::cout << "error: origin_trajectory_size = " << origin_trajectory_size
              << std::endl;
    return false;
  }

  const size_t trajectory_size = origin_trajectory_size + 1;

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
  x_vec.clear();
  y_vec.clear();
  s_vec.clear();

  x_vec.resize(trajectory_size);
  y_vec.resize(trajectory_size);
  s_vec.resize(trajectory_size);

  double s = 0.0;
  double ds = 0.0;
  for (size_t i = 0; i < origin_trajectory_size; ++i) {
    x_vec[i] = current_path_point_global_vec_[i].pos.x();
    y_vec[i] = current_path_point_global_vec_[i].pos.y();
    if (i == 0) {
      s_vec[i] = s;
    } else {
      ds = std::hypot(x_vec[i] - x_vec[i - 1], y_vec[i] - y_vec[i - 1]);
      s += std::max(ds, 1e-3);
      s_vec[i] = s;
    }
  }

  frame_.current_path_length = s;

  // calculate the extended point and insert
  Eigen::Vector2d start_point(x_vec[origin_trajectory_size - 2],
                              y_vec[origin_trajectory_size - 2]);

  Eigen::Vector2d end_point(x_vec[origin_trajectory_size - 1],
                            y_vec[origin_trajectory_size - 1]);

  Eigen::Vector2d extended_point;

  bool success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
      start_point, end_point, extended_point, frame_.path_extended_dist);

  if (success == false) {
    frame_.spline_success = false;
    std::cout << "fit line by spline error!" << std::endl;
    return false;
  }

  x_vec[origin_trajectory_size] = extended_point.x();
  y_vec[origin_trajectory_size] = extended_point.y();

  s_vec[origin_trajectory_size] =
      s_vec[origin_trajectory_size - 1] + frame_.path_extended_dist;

  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;

  return true;
}

void PerpendicularInPlanner::Log() const {
  const auto& l2g_tf = frame_.ego_slot_info.l2g_tf;
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

  const std::vector<Eigen::Vector2d>& obstacles =
      apa_world_ptr_->GetCollisionDetectorPtr()->GetObstacles();

  std::vector<double> obstaclesX;
  obstaclesX.clear();
  obstaclesX.reserve(obstacles.size());
  std::vector<double> obstaclesY;
  obstaclesY.clear();
  obstaclesY.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) {
    const auto tmp_obstacle = l2g_tf.GetPos(obstacle);
    obstaclesX.emplace_back(tmp_obstacle.x());
    obstaclesY.emplace_back(tmp_obstacle.y());
  }

  const std::unordered_map<size_t, std::vector<Eigen::Vector2d>>&
      obstacles_map =
          apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap();

  for (const auto& obs_pair : obstacles_map) {
    for (const auto& obstacle : obs_pair.second) {
      const auto tmp_obstacle = l2g_tf.GetPos(obstacle);
      obstaclesX.emplace_back(tmp_obstacle.x());
      obstaclesY.emplace_back(tmp_obstacle.y());
    }
  }

  JSON_DEBUG_VECTOR("obstaclesX", obstaclesX, 2)
  JSON_DEBUG_VECTOR("obstaclesY", obstaclesY, 2)

  std::vector<double> slot_corner_X;
  slot_corner_X.clear();
  slot_corner_X.reserve(frame_.ego_slot_info.slot_corner.size());
  std::vector<double> slot_corner_Y;
  slot_corner_Y.clear();
  slot_corner_Y.reserve(frame_.ego_slot_info.slot_corner.size());
  for (const auto& corner : frame_.ego_slot_info.slot_corner) {
    slot_corner_X.emplace_back(corner.x());
    slot_corner_Y.emplace_back(corner.y());
  }

  JSON_DEBUG_VECTOR("slot_corner_X", slot_corner_X, 2)
  JSON_DEBUG_VECTOR("slot_corner_Y", slot_corner_Y, 2)

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
  JSON_DEBUG_VALUE("remain_dist_uss", frame_.remain_dist_uss)
  JSON_DEBUG_VALUE("stuck_time", frame_.stuck_time)
  JSON_DEBUG_VALUE(
      "car_static_timer_by_pos",
      apa_world_ptr_->GetMeasurementsPtr()->car_static_timer_by_pos)
  JSON_DEBUG_VALUE(
      "car_static_timer_by_vel",
      apa_world_ptr_->GetMeasurementsPtr()->car_static_timer_by_vel)
  JSON_DEBUG_VALUE("static_flag",
                   apa_world_ptr_->GetMeasurementsPtr()->static_flag)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
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
      lateral_path_optimizer_ptr_->GetOutputDebugInfo();

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
