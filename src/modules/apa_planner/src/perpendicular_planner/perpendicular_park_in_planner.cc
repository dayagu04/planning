#include "perpendicular_park_in_planner.h"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_utils.h"
#include "apa_world.h"
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "fem_pos_deviation_smoother_config.pb.h"
#include "func_state_machine.pb.h"
#include "general_planning_context.h"
#include "geometry_math.h"
#include "local_view.h"
#include "perpendicular_path_planner.h"
#include "slot_management_info.pb.h"

namespace planning {
namespace apa_planner {
static const bool kForceBothOccupied = true;

void PerpendicularInPlanner::Reset() {
  frame_.Reset();
  t_lane_.Reset();
}

void PerpendicularInPlanner::SetParkingStatus(uint8_t status) {
  if (status == PARKING_IDLE || status == PARKING_FAILED) {
    frame_.plan_stm.path_plan_success = false;
  } else if (status == PARKING_RUNNING || status == PARKING_GEARCHANGE ||
             status == PARKING_PLANNING || status == PARKING_FINISHED) {
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

  // check planning status
  if (CheckPlanSkip() && simu_param_.force_plan == false) {
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
  if (CheckReplan() || simu_param_.force_plan) {
    std::cout << "replan is required!" << std::endl;

    UpdateSlotRealtime();
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

  // check planning status
  std::cout << "parking status = "
            << static_cast<int>(GetPlannerStates().planning_status)
            << std::endl;
}

const bool PerpendicularInPlanner::UpdateEgoSlotInfo() {
  const auto measures_ptr = apa_world_ptr_->GetMeasurementsPtr();

  auto& ego_slot_info = frame_.ego_slot_info;

  ego_slot_info.target_managed_slot.CopyFrom(measures_ptr->target_managed_slot);

  const auto& slot_points =
      ego_slot_info.target_managed_slot.corner_points().corner_point();

  std::vector<Eigen::Vector2d> pt;
  pt.resize(slot_points.size());

  for (int i = 0; i < slot_points.size(); i++) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
  }

  const auto pM01 = 0.5 * (pt[0] + pt[1]);
  const auto pM23 = 0.5 * (pt[2] + pt[3]);
  const auto n = (pM01 - pM23).normalized();

  ego_slot_info.slot_origin_pos =
      pM01 - apa_param.GetParam().normal_slot_length * n;
  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;

  // calc slot side once at first
  if (frame_.is_replan_first == true) {
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
      t_lane_.slot_side = SLOT_SIDE_RIGHT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      t_lane_.slot_side = SLOT_SIDE_LEFT;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else {
      t_lane_.slot_side = SLOT_SIDE_INVALID;
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      std::cout << "calculate slot side error " << std::endl;
      // return false;
    }
  }

  std::cout << "t_lane_.slot_side = " << static_cast<int>(t_lane_.slot_side)
            << std::endl;

  std::cout << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer) << std::endl;

  ego_slot_info.slot_length =
      apa_param.GetParam().normal_slot_length;
  ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(measures_ptr->pos_ego);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measures_ptr->heading_ego);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  if (frame_.is_replan_first) {
    ego_slot_info.limiter.first = Eigen::Vector2d(
        apa_param.GetParam().terminal_target_x, 0.5 * ego_slot_info.slot_width);
    ego_slot_info.limiter.second =
        Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                        -0.5 * ego_slot_info.slot_width);
  }

  if (perpendicular_path_planner_.GetOutput().is_last_path &&
      ego_slot_info.first_fix_limiter &&
      gear_command_ == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    std::pair<Eigen::Vector2d, Eigen::Vector2d> tmp_limiter =
        ego_slot_info.limiter;

    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter;
    const auto is_have_limiter =
        apa_world_ptr_->GetSlotManagerPtr()->GetSelectedLimiter(limiter);

    if (is_have_limiter) {
      // using limiters as stop reference
      const auto limit_left = ego_slot_info.g2l_tf.GetPos(limiter.first);
      const auto limit_right = ego_slot_info.g2l_tf.GetPos(limiter.second);
      tmp_limiter.first.x() = limit_left.x();
      tmp_limiter.second.x() = limit_right.x();
    } else {
      // using corner_2_3 as stop reference
      const auto point_2 = ego_slot_info.g2l_tf.GetPos(pt[2]);
      const auto point_3 = ego_slot_info.g2l_tf.GetPos(pt[3]);
      tmp_limiter.first.x() =
          point_2.x() + apa_param.GetParam().terminal_target_x;
      tmp_limiter.second.x() =
          point_3.x() + apa_param.GetParam().terminal_target_x;
    }
    std::cout << "tmp_limiter.first = " << tmp_limiter.first.transpose()
              << "  tmp_limiter.second = " << tmp_limiter.second.transpose()
              << "   have limiter = " << is_have_limiter << std::endl;
    const auto dist_ego_limiter =
        ((tmp_limiter.first + tmp_limiter.second) / 2.0 -
         ego_slot_info.ego_pos_slot)
            .norm();
    const double distance = 1.0;
    // todo: 1.0 can change, if limiter is accurate, should increase distance
    if (dist_ego_limiter < distance) {
      ego_slot_info.limiter = tmp_limiter;
      ego_slot_info.first_fix_limiter = false;
      PostProcessPathAccordingLimiter();
    }
  }

  // calc terminal pos
  ego_slot_info.target_ego_pos_slot
      << (ego_slot_info.limiter.first.x() + ego_slot_info.limiter.second.x()) /
             2.0,
      apa_param.GetParam().terminal_target_y;
  ego_slot_info.target_ego_heading_slot = 0.0;

  // calc terminal error once
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      ego_slot_info.ego_heading_slot - ego_slot_info.target_ego_heading_slot);

  std::cout << "-- ego_slot:" << std::endl;
  std::cout << "ego_pos_slot = " << ego_slot_info.ego_pos_slot.transpose()
            << std::endl;

  std::cout << "ego_heading_slot = " << ego_slot_info.ego_heading_slot * 57.3
            << std::endl;

  std::cout << "slot_side = " << static_cast<int>(t_lane_.slot_side)
            << std::endl;
  std::cout << "vel_ego = " << measures_ptr->vel_ego << std::endl;

  // calc slot occupied ratio
  // ego_slot_info.slot_occupied_ratio =
  //     apa_world_ptr_->GetSlotManagerPtr()->GetOccupiedRatio();

  if (std::fabs(ego_slot_info.terminal_err.pos.y()) < 0.9 &&
      std::fabs(ego_slot_info.ego_heading_slot) < 75.0 / 57.3) {
    ego_slot_info.slot_occupied_ratio =
        pnc::mathlib::Clamp(1.0 - (ego_slot_info.terminal_err.pos.x() /
                                   apa_param.GetParam().normal_slot_length),
                            0.0, 1.0);
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  std::cout << "ego_slot_info.slot_occupied_ratio = "
            << ego_slot_info.slot_occupied_ratio << std::endl;

  // update stuck time
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag &&
      apa_world_ptr_->GetMeasurementsPtr()->current_state ==
          FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) {
    frame_.stuck_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_time = 0.0;
  }

  return true;
}

void PerpendicularInPlanner::GenTlane() {
  const auto measure = apa_world_ptr_->GetMeasurementsPtr();
  const auto& ego_slot_info = frame_.ego_slot_info;
  const auto& target_corner_pts =
      ego_slot_info.target_managed_slot.corner_points();

  Eigen::Vector2d target_corner0(target_corner_pts.corner_point(0).x(),
                                 target_corner_pts.corner_point(0).y());

  Eigen::Vector2d target_corner1(target_corner_pts.corner_point(1).x(),
                                 target_corner_pts.corner_point(1).y());

  bool corner_0_side_occupied = true;
  bool corner_1_side_occupied = true;
  size_t nearby_slot_nums = 0;

  const auto& slot_info_vec =
      apa_world_ptr_->GetSlotManagerPtr()->GetOutput().slot_info_vec();

  // Check if there are slot on both sides of the selected slot
  // if so, determine if they are occupied
  for (const auto& managed_slot : slot_info_vec) {
    Eigen::Vector2d managed_corner0(
        managed_slot.corner_points().corner_point(0).x(),
        managed_slot.corner_points().corner_point(0).y());

    Eigen::Vector2d managed_corner1(
        managed_slot.corner_points().corner_point(1).x(),
        managed_slot.corner_points().corner_point(1).y());

    if ((target_corner0 - managed_corner1).norm() <
        apa_param.GetParam().nearby_slot_corner_dist) {
      nearby_slot_nums++;
      corner_0_side_occupied = managed_slot.is_occupied();
    }

    if ((target_corner1 - managed_corner0).norm() <
        apa_param.GetParam().nearby_slot_corner_dist) {
      nearby_slot_nums++;
      corner_1_side_occupied = managed_slot.is_occupied();
    }
    if (nearby_slot_nums == 2) {
      break;
    }
  }

  if (kForceBothOccupied) {
    corner_1_side_occupied = true;
    corner_0_side_occupied = true;
  }

  std::cout << "corner 0 side slot occupied = " << corner_0_side_occupied
            << std::endl;

  std::cout << "corner 1 side slot occupied = " << corner_1_side_occupied
            << std::endl;

  // order of parking slot on the left side is inconsistent with the right side
  // 0 2 is on the right side, 1 3 is on the left side, left +, right -
  // 0 1 is outter, 2 3 is inner
  const auto& slot_side = t_lane_.slot_side;

  Eigen::Vector2d corner0_slot(ego_slot_info.slot_length,
                               -0.5 * ego_slot_info.slot_width);

  Eigen::Vector2d corner1_slot(ego_slot_info.slot_length,
                               0.5 * ego_slot_info.slot_width);

  if (slot_side == SLOT_SIDE_RIGHT) {
    // right corner is inside, left corner is outside
    if (corner_0_side_occupied) {
      // right side slot is occupied
      t_lane_.pt_inside =
          corner0_slot +
          Eigen::Vector2d(apa_param.GetParam().occupied_p1_x_diff,
                          apa_param.GetParam().occupied_p1_y_diff);
    } else {
      // right side slot is free
      t_lane_.pt_inside =
          corner0_slot +
          Eigen::Vector2d(-apa_param.GetParam().vacant_p1_x_diff,
                          -apa_param.GetParam().vacant_p1_y_diff);
    }
    if (corner_1_side_occupied) {
      // left side slot is occupied
      t_lane_.pt_outside =
          corner1_slot +
          Eigen::Vector2d(apa_param.GetParam().occupied_p0_x_diff,
                          -apa_param.GetParam().occupied_p0_y_diff);
    } else {
      // left side slot is free
      t_lane_.pt_outside =
          corner1_slot + Eigen::Vector2d(-apa_param.GetParam().vacant_p0_x_diff,
                                         apa_param.GetParam().vacant_p0_y_diff);
    }
  } else if (slot_side == SLOT_SIDE_LEFT) {
    // left corner is inside, right corner is outside
    if (corner_0_side_occupied) {
      // right side slot is occupied
      t_lane_.pt_outside =
          corner0_slot +
          Eigen::Vector2d(apa_param.GetParam().occupied_p0_x_diff,
                          apa_param.GetParam().occupied_p0_y_diff);
    } else {
      // right side slot is free
      t_lane_.pt_outside =
          corner0_slot +
          Eigen::Vector2d(-apa_param.GetParam().vacant_p0_x_diff,
                          -apa_param.GetParam().vacant_p0_y_diff);
    }
    if (corner_1_side_occupied) {
      // left side slot is occupied
      t_lane_.pt_inside =
          corner1_slot +
          Eigen::Vector2d(apa_param.GetParam().occupied_p1_x_diff,
                          -apa_param.GetParam().occupied_p1_y_diff);
    } else {
      // left side slot is free
      t_lane_.pt_inside =
          corner1_slot + Eigen::Vector2d(-apa_param.GetParam().vacant_p1_x_diff,
                                         apa_param.GetParam().vacant_p1_y_diff);
    }
  }

  t_lane_.channel_x = apa_param.GetParam().normal_slot_length +
                      apa_param.GetParam().channel_width;

  t_lane_.pt_terminal << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  std::cout << "-- t_lane --" << std::endl;
  std::cout << "pt_outside = " << t_lane_.pt_outside.transpose() << std::endl;
  std::cout << "pt_inside = " << t_lane_.pt_inside.transpose() << std::endl;
  std::cout << "pt_terminal = " << t_lane_.pt_terminal.transpose() << std::endl;
}

void PerpendicularInPlanner::GenObstacles() {
  // set obstacles
  double channel_width = 10.5;
  double channel_length = 10.5;
  // double slot_length = input_.tlane.slot_length;
  if (t_lane_.pt_inside.y() < t_lane_.pt_outside.y()) {
    // right side slot
    channel_length = -channel_length;
  }
  double p0_x = t_lane_.pt_outside.x();
  double p0_y = t_lane_.pt_outside.y();
  double p1_x = t_lane_.pt_inside.x();
  double p1_y = t_lane_.pt_inside.y();

  Eigen::Vector2d A(channel_width, p0_y);
  Eigen::Vector2d B(channel_width, p0_y + channel_length);

  // Eigen::Vector2d C(p1_x, p0_y + channel_length);
  // Eigen::Vector2d D(p1_x, p1_y);
  // Eigen::Vector2d E(p0_x - slot_length, p1_y);
  // Eigen::Vector2d F(p0_x - slot_length, p0_y);
  // Eigen::Vector2d G(p0_x, p0_y);

  std::vector<pnc::geometry_lib::LineSegment> line_vec;
  pnc::geometry_lib::LineSegment line;
  line.SetPoints(A, B);
  line_vec.emplace_back(line);

  // line.SetPoints(B, C);
  // line_vec.emplace_back(line);

  // line.SetPoints(C, D);
  // line_vec.emplace_back(line);

  // line.SetPoints(D, E);
  // line_vec.emplace_back(line);

  // line.SetPoints(D, E);
  // line_vec.emplace_back(line);

  // line.SetPoints(F, G);
  // line_vec.emplace_back(line);

  double ds = 0.5;
  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> obstacle_vec;
  for (const auto& line : line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    obstacle_vec.reserve(obstacle_vec.size() + point_set.size());
    obstacle_vec.insert(obstacle_vec.end(), point_set.begin(), point_set.end());
  }

  if ((t_lane_.pt_outside - frame_.ego_slot_info.ego_pos_slot).norm() > 2.2) {
    obstacle_vec.emplace_back(t_lane_.pt_outside);
  }

  // if ((t_lane_.pt_inside - frame_.ego_slot_info.ego_pos_slot).norm() > 2.2) {
  //   obstacle_vec.emplace_back(t_lane_.pt_inside);
  // }
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(obstacle_vec);
}

const uint8_t PerpendicularInPlanner::PathPlanOnce() {
  // construct input
  const auto& ego_slot_info = frame_.ego_slot_info;
  PerpendicularPathPlanner::Input path_planner_input;
  path_planner_input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  path_planner_input.tlane = t_lane_;
  path_planner_input.is_complete_path = simu_param_.is_complete_path;
  path_planner_input.sample_ds = simu_param_.sample_ds;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;
  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                  ego_slot_info.ego_heading_slot);

  perpendicular_path_planner_.SetInput(path_planner_input);

  // need replan all path
  const bool path_plan_success = perpendicular_path_planner_.Update(
      apa_world_ptr_->GetCollisionDetectorPtr());

  // set current arc_steer
  if (!frame_.is_replan_first) {
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

  frame_.is_replan_first = false;

  // TODO: check if need update
  uint8_t plan_result = 0;
  if (path_plan_success) {
    plan_result = PathPlannerResult::PLAN_UPDATE;
    std::cout << "path plan success\n";
  } else {
    plan_result = PathPlannerResult::PLAN_FAILED;
    std::cout << "path plan fail\n";
    return PathPlannerResult::PLAN_FAILED;
  }

  perpendicular_path_planner_.SetCurrentPathSegIndex();

  perpendicular_path_planner_.SetLineSegmentHeading();

  perpendicular_path_planner_.InsertLineSegAfterCurrentFollowLastPath(0.3);

  perpendicular_path_planner_.SampleCurrentPathSeg();

  // print segment info
  perpendicular_path_planner_.PrintOutputSegmentsInfo();

  const auto& planner_output = perpendicular_path_planner_.GetOutput();
  gear_command_ = planner_output.current_gear;

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(planner_output.path_point_vec.size());

  pnc::geometry_lib::PathPoint global_point;
  for (const auto& path_point : planner_output.path_point_vec) {
    global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                     ego_slot_info.l2g_tf.GetHeading(path_point.heading));

    current_path_point_global_vec_.emplace_back(global_point);
  }

  std::cout << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size() << std::endl;

  return plan_result;
}

const bool PerpendicularInPlanner::CheckSegCompleted() {
  frame_.is_replan_by_uss = false;

  bool is_seg_complete = false;
  if (frame_.spline_success) {
    const auto min_remain_dist =
        std::min(frame_.remain_dist_uss, frame_.remain_dist);

    if (min_remain_dist <
            apa_param.GetParam().max_replan_remain_dist &&
        apa_world_ptr_->GetMeasurementsPtr()->static_flag) {
      frame_.is_replan_by_uss = (frame_.remain_dist_uss < frame_.remain_dist);

      if (!frame_.is_replan_by_uss) {
        std::cout << "close to target!\n";
        is_seg_complete = true;
      } else {
        std::cout << "close to obstacle by uss!\n";
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

void PerpendicularInPlanner::UpdateSlotRealtime() {
  bool update_slot = false;
  if (frame_.is_replan_by_uss &&
      frame_.ego_slot_info.slot_occupied_ratio > 0.25) {
    update_slot = true;
    std::cout << "update slot by uss!" << std::endl;
  }

  if (std::fabs(frame_.ego_slot_info.ego_heading_slot) < 55.0 / 57.3 &&
      frame_.ego_slot_info.ego_pos_slot.norm() < 7.5) {
    update_slot = true;
    std::cout << "update slot by pose!" << std::endl;
  }

  if (update_slot) {
    apa_world_ptr_->GetSlotManagerPtr()->SetRealtime();
    apa_world_ptr_->Update();
    UpdateEgoSlotInfo();
  }
}

const bool PerpendicularInPlanner::CheckReplan() {
  if (frame_.is_replan_first == true) {
    std::cout << "first plan" << std::endl;
    frame_.replan_reason = FIRST_PLAN;
    return true;
  }

  if (CheckSegCompleted()) {
    std::cout << "replan by current segment completed!" << std::endl;
    frame_.replan_reason = SEG_COMPLETED;
    return true;
  }

  if (frame_.stuck_time >
      apa_param.GetParam().stuck_replan_time) {
    std::cout << "replan by stuck!" << std::endl;
    frame_.replan_reason = STUCKED;
    return true;
  }

  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool PerpendicularInPlanner::CheckStuckFailed() {
  return frame_.stuck_time >
         apa_param.GetParam().stuck_failed_time;
}

const bool PerpendicularInPlanner::CheckFinished() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  const bool parking_success =
      ego_slot_info.terminal_err.pos.x() <
          apa_param.GetParam().max_finish_lon_offset &&
      std::fabs(ego_slot_info.terminal_err.pos.y()) <=
          apa_param.GetParam().max_finish_lat_offset &&
      std::fabs(ego_slot_info.terminal_err.heading) <=
          apa_param.GetParam().max_finish_heading_offset_deg /
              57.3 &&
      apa_world_ptr_->GetMeasurementsPtr()->static_flag;

  std::cout << "terminal x error= " << ego_slot_info.terminal_err.pos.x()
            << std::endl;
  std::cout << "terminal y error= " << ego_slot_info.terminal_err.pos.y()
            << std::endl;
  std::cout << "terminal heading error= "
            << ego_slot_info.terminal_err.heading * 57.3 << std::endl;

  return parking_success;
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
             frame_.plan_stm.planning_status == PARKING_RUNNING) {
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
  const std::vector<double> vel_limit_tab = {
      apa_param.GetParam().max_velocity,
      apa_param.GetParam().max_velocity, 0.45, 0.35};
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
  if (is_simulation_ && simu_param_.force_plan && simu_param_.is_reset) {
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

  double safe_uss_remain_dist =
      apa_param.GetParam().safe_uss_remain_dist;
  if (frame_.ego_slot_info.slot_occupied_ratio < 0.05) {
    safe_uss_remain_dist = 0.6;
  }

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

  const auto limiter_left =
      frame_.ego_slot_info.l2g_tf.GetPos(frame_.ego_slot_info.limiter.first);

  const auto limiter_right =
      frame_.ego_slot_info.l2g_tf.GetPos(frame_.ego_slot_info.limiter.second);

  const auto limiter = limiter_right - limiter_left;
  const auto path_point_first =
      Eigen::Vector2d(current_path_point_global_vec_.front().pos.x(),
                      current_path_point_global_vec_.front().pos.y());

  const auto first_point_limiter = path_point_first - limiter_left;
  double first_point_direction =
      pnc::geometry_lib::GetCrossFromTwoVec2d(first_point_limiter, limiter);

  std::cout << "limiter_mid = " << limiter_mid.transpose() << std::endl;
  double s = 0.0;
  double ds = 0.0;
  double dis = 0.0;
  size_t i = 0;
  for (i = 0; i < origin_traj_size; ++i) {
    const double x = current_path_point_global_vec_[i].pos.x();
    const double y = current_path_point_global_vec_[i].pos.y();
    Eigen::Vector2d path_point;
    path_point << x, y;

    const auto point_limiter = path_point - limiter_left;
    double point_direction =
        pnc::geometry_lib::GetCrossFromTwoVec2d(point_limiter, limiter);

    dis = (path_point - limiter_mid).norm();
    if (point_direction * first_point_direction <= 1e-8 ||
        dis < apa_param.GetParam().terminal_target_x_to_limiter) {
      // need shorten
      std::cout << "path shoule be shorten because of limiter\n";
      std::cout << "path_point = " << path_point.transpose() << std::endl;
      std::cout << "current dist = " << dis << std::endl;
      break;
    }
    x_vec.emplace_back(x);
    y_vec.emplace_back(y);
    heading_vec.emplace_back(current_path_point_global_vec_[i].heading);

    if (i == 0) {
      s_vec.emplace_back(s);
    } else {
      ds = std::hypot(x_vec[i] - x_vec[i - 1], y_vec[i] - y_vec[i - 1]);
      s += std::max(ds, 1e-3);
      s_vec.emplace_back(s);
    }
  }

  if (i < 2) {
    frame_.spline_success = false;
    std::cout << "error: no enough point = " << i << std::endl;
    return false;
  }

  if (dis > apa_param.GetParam().terminal_target_x_to_limiter) {
    // need extend by limiter
    std::cout << "path shoule be extended because of limiter\n";
    double extend_dis =
        dis - apa_param.GetParam().terminal_target_x_to_limiter;

    Eigen::Vector2d extended_point;
    bool success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
        Eigen::Vector2d(x_vec[i - 2], y_vec[i - 2]),
        Eigen::Vector2d(x_vec[i - 1], y_vec[i - 1]), extended_point,
        extend_dis);
    if (success == false) {
      frame_.spline_success = false;
      std::cout << "extend point err, set spline error!\n";
      return false;
    }

    x_vec.emplace_back(extended_point.x());
    y_vec.emplace_back(extended_point.y());
    heading_vec.emplace_back(heading_vec.back());
    s += extend_dis;
    s_vec.emplace_back(s);
    i++;
  }

  frame_.current_path_length = s;

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(i);
  pnc::geometry_lib::PathPoint path_point;
  for (size_t j = 0; j < x_vec.size(); ++j) {
    path_point.Set(Eigen::Vector2d(x_vec[j], y_vec[j]), heading_vec[j]);
    current_path_point_global_vec_.emplace_back(path_point);
  }

  // need extend by cal proj point
  Eigen::Vector2d extended_point;
  bool success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
      Eigen::Vector2d(x_vec[i - 2], y_vec[i - 2]),
      Eigen::Vector2d(x_vec[i - 1], y_vec[i - 1]), extended_point,
      frame_.path_extended_dist);

  if (success == false) {
    frame_.spline_success = false;
    std::cout << "limit need extend fit line by spline error!" << std::endl;
    return false;
  }

  x_vec.emplace_back(extended_point.x());
  y_vec.emplace_back(extended_point.y());
  heading_vec.emplace_back(heading_vec.back());
  s += frame_.path_extended_dist;
  s_vec.emplace_back(s);

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
  const auto p0_g = l2g_tf.GetPos(t_lane_.pt_outside);
  const auto p1_g = l2g_tf.GetPos(t_lane_.pt_inside);
  const auto pt_g = l2g_tf.GetPos(t_lane_.pt_terminal);

  JSON_DEBUG_VALUE("tlane_p0_x", p0_g.x())
  JSON_DEBUG_VALUE("tlane_p0_y", p0_g.y())
  JSON_DEBUG_VALUE("tlane_p1_x", p1_g.x())
  JSON_DEBUG_VALUE("tlane_p1_y", p1_g.y())
  JSON_DEBUG_VALUE("tlane_pt_x", pt_g.x())
  JSON_DEBUG_VALUE("tlane_pt_y", pt_g.y())
  JSON_DEBUG_VALUE("channel_x", t_lane_.channel_x)
  JSON_DEBUG_VALUE("slot_side", t_lane_.slot_side)

  std::vector<Eigen::Vector2d> obstacles =
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

  JSON_DEBUG_VECTOR("obstaclesX", obstaclesX, 2)
  JSON_DEBUG_VECTOR("obstaclesY", obstaclesY, 2)

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
      "standstill_timer_by_pos",
      apa_world_ptr_->GetMeasurementsPtr()->standstill_timer_by_pos)
  JSON_DEBUG_VALUE(
      "standstill_timer_by_vel",
      apa_world_ptr_->GetMeasurementsPtr()->standstill_timer_by_vel)
  JSON_DEBUG_VALUE("static_flag",
                   apa_world_ptr_->GetMeasurementsPtr()->static_flag)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
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
}

}  // namespace apa_planner
}  // namespace planning
