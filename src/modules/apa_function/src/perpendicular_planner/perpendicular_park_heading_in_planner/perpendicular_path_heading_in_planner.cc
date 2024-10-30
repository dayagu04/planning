#include "perpendicular_path_heading_in_planner.h"

#include <bits/stdint-uintn.h>
#include <google/protobuf/message.h>
#include <math.h>
#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <utility>
#include <vector>

#include "apa_param_setting.h"
#include "apa_world.h"
#include "collision_detection.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math_lib.h"
#include "planning_plan_c.h"
#include "utils/cartesian_coordinate_system.h"

namespace planning {
namespace apa_planner {

static const size_t kMaxPerpenParkInSegmentNums = 15;
static const size_t kReservedOutputPathPointSize = 750;
static const int kMultiPlanMaxPathNumsInSlot = 1;
static const int kLineArcPlanMaxPathNumsInSlot = 2;
static const size_t kAdjustPlanMaxPathNumsInSlot = 3;
static const double kMinSingleGearPathLength = 0.4;

void PerpendicularPathHeadingInPlanner::Reset() {
  output_.Reset();
  output_.path_segment_vec.reserve(kMaxPerpenParkInSegmentNums);
  output_.gear_cmd_vec.reserve(kMaxPerpenParkInSegmentNums);

  calc_params_.Reset();
}

void PerpendicularPathHeadingInPlanner::PreprocessForSimu() {
  // calc_params_.Reset();
  calc_params_.multi_plan = false;
  calc_params_.directly_use_ego_pose = false;
  calc_params_.turn_radius = apa_param.GetParam().min_turn_radius;
  calc_params_.can_insert_line = true;
  calc_params_.use_line_arc = false;
  calc_params_.is_outside_occupied = false;
  calc_params_.is_inside_occupied = false;
  calc_params_.use_mono_circle_headin = false;
  // if channel width is over 8m;set true
  calc_params_.verify_mono_circle_headin = true;
  calc_params_.use_adjust = false;

  // for simulation
  if (input_.tlane.pt_inside.y() > input_.tlane.pt_outside.y()) {
    calc_params_.is_left_side = false;
    calc_params_.slot_side_sgn = 1.0;
  } else {
    calc_params_.is_left_side = true;
    calc_params_.slot_side_sgn = -1.0;
  }

  // reset output
  output_.Reset();

  // TODO: make sure inside and outside if is occuppied
  if (std::fabs(input_.pt_1.x() - input_.tlane.corner_outside_slot.x()) < 0.1) {
    calc_params_.is_outside_occupied = true;
  }

  if (std::fabs(input_.pt_0.x() - input_.tlane.corner_inside_slot.x()) < 0.1) {
    calc_params_.is_inside_occupied = true;
  }

  // std::cout << " input_.pt_0 = " << input_.pt_0.transpose()
  //           << " input_.tlane.corner_inside_slot = "
  //           << input_.tlane.corner_inside_slot.transpose() << std::endl;

  // std::cout << "is_outside_occupied = " << calc_params_.is_outside_occupied
  //           << " is_inside_occupied = " << calc_params_.is_inside_occupied
  //           << std::endl;

  // target line
  calc_params_.target_line = pnc::geometry_lib::BuildLineSegByPose(
      input_.tlane.pt_terminal_pos, input_.tlane.pt_terminal_heading);
}

void PerpendicularPathHeadingInPlanner::Preprocess() {
  // calc_params_.Reset();
  calc_params_.multi_plan = false;
  calc_params_.directly_use_ego_pose = false;
  calc_params_.turn_radius = 1.0 * apa_param.GetParam().min_turn_radius;
  calc_params_.can_insert_line = true;
  calc_params_.use_line_arc = false;
  calc_params_.is_outside_occupied = false;
  calc_params_.is_inside_occupied = false;
  calc_params_.use_mono_circle_headin = false;
  calc_params_.verify_mono_circle_headin = true;
  calc_params_.use_adjust = false;

  // calculate slot side by Tlane
  if (input_.tlane.pt_inside.y() < input_.tlane.pt_outside.y()) {
    calc_params_.is_left_side = true;
    calc_params_.slot_side_sgn = -1.0;
    calc_params_.is_outside_occupied = !input_.is_left_empty;
    calc_params_.is_inside_occupied = !input_.is_right_empty;
  } else {
    calc_params_.is_left_side = false;
    calc_params_.slot_side_sgn = 1.0;
    calc_params_.is_outside_occupied = !input_.is_right_empty;
    calc_params_.is_inside_occupied = !input_.is_left_empty;
  }
  // std::cout << "is_outside_occupied = " << calc_params_.is_outside_occupied
  //           << " is_inside_occupied = " << calc_params_.is_inside_occupied
  //           << std::endl;

  // target line
  calc_params_.target_line = pnc::geometry_lib::BuildLineSegByPose(
      input_.tlane.pt_terminal_pos, input_.tlane.pt_terminal_heading);
}

const bool PerpendicularPathHeadingInPlanner::Update(
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  collision_detector_ptr_ = collision_detector_ptr;
  return Update();
}

const bool PerpendicularPathHeadingInPlanner::Update() {
  // preprocess
  Preprocess();

  // reset output
  output_.Reset();

  // prepare plan for the first time
  if (input_.is_replan_first) {
    calc_params_.first_multi_plan = true;
    if (PreparePlan()) {
      if (calc_params_.directly_use_ego_pose) {
        ILOG_INFO << "ego pose is close to safe_circle_tang_pt, directly use"
                     "ego pose to multi plan, no second prepare!";
        calc_params_.should_prepare_second = false;
        output_.gear_shift = true;
      } else if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
        ILOG_INFO << "first prepare success, the gear is reverse, not need "
                     "multi plan, need second prepare!";
        calc_params_.should_prepare_second = true;
        // TODO consider prepare_second flag if needed
        output_.gear_shift = false;
        input_.ref_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
        input_.ref_arc_steer =
            input_.ref_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT
                ? pnc::geometry_lib::SEG_STEER_LEFT
                : pnc::geometry_lib::SEG_STEER_RIGHT;
        // return true;
      } else if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
        ILOG_INFO << "first prepare success, the gear is drive, need multi "
                     "plan, need second prepare!";
        // TODO consider debug_print
        calc_params_.should_prepare_second = true;
        output_.gear_shift = true;
        return true;
      } else {
        ILOG_INFO << "except err";
        return false;
      }
    } else {
      ILOG_INFO << "prepare plan fail, quit plan!";
      return false;
    }
  }

  // second plan
  if (input_.is_replan_second) {
    if (calc_params_.should_prepare_second) {
      calc_params_.should_prepare_second = false;
      ILOG_INFO << "should try second prepare!";
      if (PreparePlanSecond()) {
        if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
          ILOG_INFO << "second prepare gear is reverse, need multi plan!";
          return true;
        } else if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
          ILOG_INFO << "second prepare gear is drive, need multi plan!";
        } else {
          ILOG_INFO << "except err";
          return false;
        }
      } else {
        ILOG_INFO << "second prepare failed, use ego pose to multi plan!";
        output_.gear_shift =
            (calc_params_.prepare_point_condition == 0) ? false : true;
        ILOG_INFO << "calc_params_.prepare_point_condition = "
                  << calc_params_.prepare_point_condition;
        ILOG_INFO << "output_.gear_shift = " << output_.gear_shift;
      }
    } else {
      ILOG_INFO
          << "no need second prepare, directly use ego pose to multi plan";
      output_.gear_shift = false;
    }
  }
  // multi step
  if (MultiPlan()) {
    // calc_params_.first_multi_plan = false;
    ILOG_INFO << "multi plan success!";
  }
  if (CheckReachTargetPose()) {
    ILOG_INFO << "multi plan is already to target pos!";
    return true;
  }

  // line arc
  if (MultiLineArcPlan()) {
    ILOG_INFO << "multi line arc plan success!";
  }

  if (output_.linearc_reach_target_pose) {
    ILOG_INFO << "multi line arc plan is already to target pos!";
    return true;
  }

  // adjust step
  if (AdjustPlan()) {
    ILOG_INFO << "adjust plan success!";
    return true;
  }

  if (output_.path_segment_vec.size() == 0) {
    ILOG_INFO << "no path, plan fail";
    output_.Reset();
    return false;
  } else {
    ILOG_INFO
        << "there are path, through no plan to target pose, but let car go.";
    return true;
  }
}

// prepare first plan
const bool PerpendicularPathHeadingInPlanner::PreparePlan() {
  std::vector<double> x_offset_vec;
  std::vector<double> heading_offset_vec;

  const double pt_01_x = ((input_.pt_0 + input_.pt_1) * 0.5).x();
  const double x_min =
      pt_01_x + apa_param.GetParam().headin_prepare_line_min_x_offset_slot /
                    input_.sin_angle;

  const double x_max =
      pt_01_x +
      std::min(apa_param.GetParam().headin_prepare_line_max_x_offset_slot,
               input_.channel_width - 0.8) /
          input_.sin_angle;

  double x_offset = x_max;
  while (x_offset >= x_min) {
    x_offset_vec.emplace_back(x_offset);
    x_offset -= apa_param.GetParam().prepare_line_dx_offset_slot;
  }

  double heading_offset =
      apa_param.GetParam().headin_prepare_line_max_heading_offset_slot_deg *
      kDeg2Rad;

  while (heading_offset >=
         apa_param.GetParam().headin_prepare_line_min_heading_offset_slot_deg *
             kDeg2Rad) {
    heading_offset_vec.emplace_back(heading_offset);
    heading_offset -=
        apa_param.GetParam().prepare_line_dheading_offset_slot_deg * kDeg2Rad;
  }

  bool flag = false;
  bool prepare_success = false;

  // judge if drive gear is allowed
  CollisionDetector::Paramters param;
  const double drive_allowed_path_length =
      (input_.pt_1.y() - calc_params_.slot_side_sgn * 1.0) -
      input_.ego_pose.pos.y();
  ILOG_INFO << "drive_allowed_path_length = " << drive_allowed_path_length;
  pnc::geometry_lib::LineSegment line(
      input_.ego_pose.pos,
      input_.ego_pose.pos +
          drive_allowed_path_length *
              Eigen::Vector2d(std::fabs(std::cos(input_.ego_pose.heading)),
                              std::fabs(std::sin(input_.ego_pose.heading))),
      input_.ego_pose.heading);

  param.lat_inflation = apa_param.GetParam().car_lat_inflation_strict;
  collision_detector_ptr_->SetParam(param);
  CollisionDetector::CollisionResult col_res;
  col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
  if (!col_res.collision_flag) {
    calc_params_.prepare_point_condition = 1;  // drive
  }
  for (size_t i = 0; i < x_offset_vec.size() && !flag; ++i) {
    const double& x_offset = x_offset_vec[i];
    for (size_t j = 0; j < heading_offset_vec.size() && !flag; ++j) {
      const double& heading_offset = heading_offset_vec[j];
      if (PreparePlanOnce(x_offset, heading_offset, calc_params_.turn_radius)) {
        prepare_success = true;
        ILOG_INFO << "x_offset = " << x_offset
                  << "  heading_offset = " << heading_offset * kRad2Deg;
      } else {
        // decide if can directly use ego pose to plan
        if (calc_params_.cal_tang_pt_success) {
          const double dist =
              (input_.ego_pose.pos - calc_params_.safe_circle_tang_pt.pos)
                  .norm();
          const double heading_err =
              std::fabs(input_.ego_pose.heading -
                        calc_params_.safe_circle_tang_pt.heading);
          if (dist <
                  apa_param.GetParam().prepare_directly_use_tangent_pos_err &&
              heading_err < apa_param.GetParam()
                                    .prepare_directly_use_tangent_heading_err *
                                kDeg2Rad) {
            calc_params_.directly_use_ego_pose = true;
          }
        }
      }
      flag = prepare_success || calc_params_.directly_use_ego_pose;
    }
    calc_params_.verify_mono_circle_headin = true;
  }

  calc_params_.pt_inside = input_.tlane.pt_inside;

  if (!flag) {
    ILOG_INFO << "prepare first fail";
    return false;
  }

  // if success, directly quit
  if (prepare_success) {
    // gen prepare path
    GenPathOutputByDubins();
    if (output_.length < kMinSingleGearPathLength - 0.016) {
      output_.Reset();
      return false;
    }
    ILOG_INFO << "first try dubins success";
    input_.ego_pose = calc_params_.safe_circle_tang_pt;
  }

  if (calc_params_.directly_use_ego_pose) {
    // directly use ego pose to multi plan
    ILOG_INFO << "directly use ego pose to multi plan";
  }
  calc_params_.first_path_gear = output_.current_gear;
  return true;
}

void PerpendicularPathHeadingInPlanner::CalMonoSafeCircle() {
  if (!calc_params_.use_mono_circle_headin &&
      !calc_params_.is_inside_occupied) {
    calc_params_.target_line_y_offset_sign = 1.0;
    // std::cout << "inside translation\n";
  }
  if (!calc_params_.use_mono_circle_headin &&
      !calc_params_.is_outside_occupied) {
    calc_params_.target_line_y_offset_sign = -1.0;
    // std::cout << "outside translation\n";
  }
  calc_params_.mono_safe_circle.center.y() =
      calc_params_.target_line.pA.y() +
      (calc_params_.turn_radius +
       calc_params_.target_line_y_offset_sign *
           apa_param.GetParam().headin_max_pt_inside_drop_dy) *
          calc_params_.slot_side_sgn;

  calc_params_.mono_safe_circle.radius = calc_params_.turn_radius;

  auto pt_inside = input_.tlane.pt_inside;

  pt_inside.x() =
      std::max(pt_inside.x(),
               input_.tlane.corner_inside_slot.x() -
                   apa_param.GetParam().max_pt_inside_drop_dx_mono_headin);

  pt_inside.y() =
      pt_inside.y() + calc_params_.slot_side_sgn *
                          calc_params_.target_line_y_offset_sign *
                          apa_param.GetParam().headin_max_pt_inside_drop_dy;

  ILOG_INFO << "pt inside = " << pt_inside.transpose();
  // ATTENTION: when inside is empty it can work
  double delta_x = -0.3;
  const double delta_radius = calc_params_.turn_radius -
                              apa_param.GetParam().car_width * 0.5 -
                              apa_param.GetParam().car_lat_inflation_normal;
  const double delta_y =
      calc_params_.mono_safe_circle.center.y() - pt_inside.y();

  if (delta_radius > std::fabs(delta_y)) {
    delta_x = std::sqrt(std::pow(delta_radius, 2) - std::pow(delta_y, 2));
  }
  ILOG_INFO << "delta_x = " << delta_x;

  calc_params_.mono_safe_circle.center.x() = pt_inside.x() - delta_x;

  ILOG_INFO << "mono safe circle info: center = "
            << calc_params_.mono_safe_circle.center.transpose()
            << "  radius = " << calc_params_.mono_safe_circle.radius;
  // ILOG_INFO << "mono safe circle info: center = "
  //             << calc_params_.mono_safe_circle.center.transpose()
  //             << "   radius = " << calc_params_.mono_safe_circle.radius);
}

const bool PerpendicularPathHeadingInPlanner::CheckMonoIsFeasible() {
  const double dist = CalPoint2LineDist(calc_params_.mono_safe_circle.center,
                                        calc_params_.prepare_line);

  ILOG_INFO << "dist  = " << dist;
  if (dist >= calc_params_.mono_safe_circle.radius) {
    // ILOG_INFO << "prepare_line is tangential or disjoint from mono safe "
    //              "circle, mono is feasible!";
    return true;
  } else {
    // ILOG_INFO << "prepare_line intersects circle, mono is not feasible";
    return false;
  }
}

const bool PerpendicularPathHeadingInPlanner::MonoPreparePlan(
    Eigen::Vector2d& tag_point) {
  CalMonoSafeCircle();

  if (CheckMonoIsFeasible() == false) {
    // ILOG_INFO << "cal monostep safe circle fail!";
    return false;
  }

  // calc actual circle corresponding to mono safe circle
  pnc::geometry_lib::Circle autual_circle;
  autual_circle = calc_params_.mono_safe_circle;

  const Eigen::Vector2d point_start = calc_params_.prepare_line.pA;

  // the length from point_start to point_tangent
  const double length =
      (autual_circle.center.y() -
       autual_circle.radius * calc_params_.pre_line_normal_vec.y() -
       point_start.y()) /
      calc_params_.pre_line_tangent_vec.y();

  tag_point = point_start + length * calc_params_.pre_line_tangent_vec;
  // ILOG_INFO << "point_tangent = " << tag_point.transpose());
  return true;
}

const bool PerpendicularPathHeadingInPlanner::CalMultiSafeCircle() {
  auto pt_inside = input_.tlane.pt_inside;
  pt_inside.x() =
      std::max(pt_inside.x(),
               input_.tlane.corner_inside_slot.x() -
                   apa_param.GetParam().max_pt_inside_drop_dx_multi_headin);

  pnc::geometry_lib::Circle circle_p1;
  circle_p1.center = pt_inside;

  circle_p1.radius = calc_params_.turn_radius -
                     0.5 * apa_param.GetParam().car_width -
                     apa_param.GetParam().car_lat_inflation_strict;

  // move down the start line
  const Eigen::Vector2d pt_s =
      calc_params_.prepare_line.pA +
      calc_params_.pre_line_normal_vec * calc_params_.turn_radius;

  pnc::geometry_lib::LineSegment line_sa(
      pt_s, pt_s + calc_params_.pre_line_tangent_vec);

  // calc cross points of line and circle
  std::vector<Eigen::Vector2d> cross_points;
  const size_t cross_pt_nums =
      pnc::geometry_lib::CalcCrossPointsOfLineAndCircle(line_sa, circle_p1,
                                                        cross_points);

  if (cross_pt_nums != 2) {
    return false;
  }

  auto& multi_safe_circle = calc_params_.multi_safe_circle;

  if (cross_points[0].y() * calc_params_.slot_side_sgn >
      cross_points[1].y() * calc_params_.slot_side_sgn) {
    multi_safe_circle.center = cross_points[0];
  } else {
    multi_safe_circle.center = cross_points[1];
  }

  multi_safe_circle.radius = calc_params_.turn_radius;

  // ILOG_INFO << "multi safe circle info: center = " <<
  // multi_safe_circle.center
  //           << "  radius = " << multi_safe_circle.radius);
  ILOG_INFO << "multi safe circle info: center = " << multi_safe_circle.center
            << "  radius = " << multi_safe_circle.radius;

  return true;
}

const bool PerpendicularPathHeadingInPlanner::MultiPreparePlan(
    Eigen::Vector2d& tag_point) {
  if (!CalMultiSafeCircle()) {
    // ILOG_INFO << "cal multistep safe circle fail!";
    return false;
  }
  tag_point =
      calc_params_.multi_safe_circle.center -
      calc_params_.pre_line_normal_vec * calc_params_.multi_safe_circle.radius;

  // ILOG_INFO << "point_tangent = " << tag_point.transpose();
  return true;
}

const bool PerpendicularPathHeadingInPlanner::ComputePreparePointSecond(
    pnc::dubins_lib::DubinsLibrary::Input& input,
    const pnc::geometry_lib::PathPoint& target_pose) {
  ILOG_INFO << "new target point before multiplan, need second plan";
  pnc::geometry_lib::PathPoint first_prepare_point;

  if (calc_params_.prepare_point_condition == 0) {
    const double theta_delta = input_.ego_pose.heading - target_pose.heading;
    first_prepare_point.pos.x() =
        input_.ego_pose.pos.x() +
        calc_params_.turn_radius * (1.0 - std::cos(theta_delta));

    first_prepare_point.pos.y() =
        input_.ego_pose.pos.y() +
        calc_params_.turn_radius * std::sin(theta_delta);

    first_prepare_point.heading = target_pose.heading;

    first_prepare_point.pos -=
        1.0 * pnc::geometry_lib::GenHeadingVec(first_prepare_point.heading);
    if (calc_params_.is_inside_occupied) {
      first_prepare_point.pos.x() +=
          calc_params_.turn_radius * (1.0 - std::cos(theta_delta));

      first_prepare_point.pos.y() +=
          calc_params_.turn_radius * std::sin(theta_delta);

      first_prepare_point.heading = input_.ego_pose.heading;
    }
  } else if (calc_params_.prepare_point_condition == 1) {
    first_prepare_point = input_.ego_pose;
    double move_straight_length = 0.0;
    if (calc_params_.is_outside_occupied) {
      move_straight_length = -input_.ego_pose.pos.y() -
                             calc_params_.slot_side_sgn *
                                 std::max(0.2 * calc_params_.turn_radius, 2.88);
    } else {
      move_straight_length = -input_.ego_pose.pos.y() -
                             calc_params_.slot_side_sgn *
                                 std::max(0.2 * calc_params_.turn_radius, 2.08);
    }
    if (std::fabs(move_straight_length) < kMinSingleGearPathLength) {
      move_straight_length =
          kMinSingleGearPathLength *
          (move_straight_length / std::fabs(move_straight_length));
    }
    ILOG_INFO << "move_straight_length = " << move_straight_length;
    first_prepare_point.pos.x() +=
        move_straight_length * std::fabs(std::cos(input_.ego_pose.heading));

    first_prepare_point.pos.y() +=
        move_straight_length * std::fabs(std::sin(input_.ego_pose.heading));

    const std::vector<double> heading_tab = {0.0, 3.8, 6.8, 8.8, 9.8};
    const std::vector<double> length_tab = {0.0, 1.0, 2.0, 3.8, 5.0};
    const double heading_err = pnc::mathlib::Interp1(
        length_tab, heading_tab,
        pnc::mathlib::Limit(std::fabs(move_straight_length), 5.0));
    ILOG_INFO << "prepare point heading change = " << heading_err;
    if (first_prepare_point.heading < 0.0) {
      first_prepare_point.heading -= heading_err * kDeg2Rad;
    } else {
      first_prepare_point.heading += heading_err * kDeg2Rad;
    }

  } else {
    first_prepare_point.pos.x() =
        0.3 * input_.ego_pose.pos.x() + 0.7 * target_pose.pos.x();

    first_prepare_point.pos.y() =
        input_.ego_pose.pos.y() -
        6.0 * pnc::geometry_lib::GenHeadingVec(input_.ego_pose.heading).y();

    first_prepare_point.heading =
        0.8 * target_pose.heading + 0.2 * input_.ego_pose.heading;
  }

  ILOG_INFO << "first_prepare_point = " << first_prepare_point.pos.transpose()
            << " heading = " << first_prepare_point.heading * kRad2Deg;

  input.Set(input_.ego_pose.pos, first_prepare_point.pos,
            input_.ego_pose.heading, first_prepare_point.heading);

  dubins_planner_.SetRadius(calc_params_.turn_radius);
  dubins_planner_.SetInput(input);
  bool prepare_success =
      dubins_planner_.OneStepDubinsUpdateByVer(kMinSingleGearPathLength);
  ILOG_INFO << "prepare_success = " << prepare_success;
  calc_params_.second_prepareplan_input = first_prepare_point;
  ILOG_INFO << "new dubins success = " << prepare_success;
  return prepare_success;
}

const bool PerpendicularPathHeadingInPlanner::PreparePlanOnce(
    const double& x_offset, const double& heading_offset,
    const double& radius) {
  const double start_heading =
      calc_params_.slot_side_sgn *
      ((90.0 - input_.origin_pt_0_heading) * kDeg2Rad - heading_offset);

  pnc::geometry_lib::PathPoint start_pose;
  start_pose.Set(Eigen::Vector2d(x_offset, 0.0), start_heading);

  // cal pre line tangent vec and normal vec
  const Eigen::Vector2d line_tangent_vec =
      pnc::geometry_lib::GenHeadingVec(start_pose.heading);

  Eigen::Vector2d line_normal_vec(line_tangent_vec.y(), -line_tangent_vec.x());

  // sure line_normal_vec towards downward along the x axis.
  if (line_normal_vec.x() > 0.0) {
    line_normal_vec = -1.0 * line_normal_vec;
  }

  // gen prepare line
  calc_params_.prepare_line =
      pnc::geometry_lib::BuildLineSegByPose(start_pose.pos, start_pose.heading);

  ILOG_INFO << "prepareline = " << start_pose.pos.transpose()
            << " heading = " << start_heading * kRad2Deg;

  calc_params_.pre_line_tangent_vec = line_tangent_vec;
  calc_params_.pre_line_normal_vec = line_normal_vec;

  // find a target point on prepare line
  pnc::geometry_lib::PathPoint target_pose;
  // TODO: should change heading
  if (start_heading >= 0) {
    target_pose.heading = start_heading - 180.0 * kDeg2Rad;
  } else {
    target_pose.heading = start_heading + 180.0 * kDeg2Rad;
  }

  pnc::dubins_lib::DubinsLibrary::Input input;
  input.radius = radius;

  calc_params_.cal_tang_pt_success = false;
  calc_params_.use_mono_tang = false;
  calc_params_.use_multi_tang = false;
  calc_params_.use_mono_circle_headin = false;
  bool prepare_success = false;

  // to verify if can one arc headin
  if (!calc_params_.verify_mono_circle_headin) {
    pnc::geometry_lib::Arc arc;
    const double heading_error =
        target_pose.heading - calc_params_.target_line.heading;

    const double length_offset =
        std::fabs(radius * std::tan(0.5 * heading_error));

    arc.pA = calc_params_.prepare_line.pA +
             length_offset * calc_params_.pre_line_tangent_vec;

    arc.headingA = target_pose.heading;
    arc.circle_info.center = arc.pA + radius * calc_params_.pre_line_normal_vec;
    arc.circle_info.radius = radius;
    arc.length = radius * std::fabs(heading_error);
    arc.is_anti_clockwise = calc_params_.is_left_side ? true : false;
    pnc::geometry_lib::CompleteArcInfo(arc, arc.length, arc.is_anti_clockwise);

    CollisionDetector::Paramters param;
    param.lat_inflation = apa_param.GetParam().car_lat_inflation_strict;
    collision_detector_ptr_->SetParam(param);
    CollisionDetector::CollisionResult col_res;
    col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);

    if (!col_res.collision_flag) {
      ILOG_INFO << "--------use_mono_circle_headin----------";
      std::cout << "circle center = " << arc.circle_info.center.transpose()
                << " length_offset = " << length_offset
                << " arc.pa = " << arc.pA.transpose()
                << " arc.headingA = " << arc.headingA * kRad2Deg
                << " arc.pb = " << arc.pB.transpose()
                << " arc.headingb = " << arc.headingB * kRad2Deg << std::endl;
      // const double remain_car_dist = col_res.remain_car_dist;
      // const double remain_obs_dist = col_res.remain_obstacle_dist;
      // const double safe_remain_dist =
      //     std::min(remain_car_dist, remain_obs_dist - 0.35);

      // ILOG_INFO << "remain_car_dist = "
      //             << remain_car_dist
      //             << "  remain_obs_dist = " << remain_obs_dist
      //             << "  safe_remain_dist = " << safe_remain_dist);
      target_pose.pos = arc.pA;
      target_pose.heading = arc.headingA;
      calc_params_.use_mono_circle_headin = true;
      calc_params_.verify_mono_circle_headin = true;
    }
  }

  while (!prepare_success) {
    // first use mono prepare to find target point
    if (calc_params_.use_mono_circle_headin) {
      input.Set(input_.ego_pose.pos, target_pose.pos, input_.ego_pose.heading,
                target_pose.heading);

      dubins_planner_.SetInput(input);
      prepare_success =
          dubins_planner_.OneStepDubinsUpdateByVer(kMinSingleGearPathLength);

      calc_params_.cal_tang_pt_success = true;
      calc_params_.safe_circle_tang_pt = target_pose;
      // std::cout << "prepare_success = " << prepare_success
      //           << " safe_circle_tang_pt = " << target_pose.pos.transpose()
      //           << " heading = " << target_pose.heading * kRad2Deg <<
      //           std::endl;
    }
    if (prepare_success) {
      ILOG_INFO << "use_mono_circle_headin successful!";
      calc_params_.use_mono_tang = true;
      break;
    }
    // inside is vacant or outside is vacant
    if (!calc_params_.is_inside_occupied || !calc_params_.is_outside_occupied) {
      if (MonoPreparePlan(target_pose.pos)) {
        ILOG_INFO << "ego_pose = " << input_.ego_pose.pos.transpose()
                  << " heading = " << input_.ego_pose.heading * kRad2Deg;
        ILOG_INFO << "point_tangent = " << target_pose.pos.transpose()
                  << " heading = " << target_pose.heading * kRad2Deg;

        calc_params_.cal_tang_pt_success = true;
        calc_params_.safe_circle_tang_pt = target_pose;
        // TODO: to find a suitable current pose by dubins
        input.Set(input_.ego_pose.pos, target_pose.pos, input_.ego_pose.heading,
                  target_pose.heading);

        dubins_planner_.SetInput(input);
        prepare_success =
            dubins_planner_.OneStepDubinsUpdateByVer(kMinSingleGearPathLength);

        if (!prepare_success &&
            (target_pose.pos.x() - input_.pt_0.x() < input_.channel_width)) {
          prepare_success = ComputePreparePointSecond(input, target_pose);
          if (prepare_success) {
            calc_params_.safe_circle_tang_pt =
                calc_params_.second_prepareplan_input;
          }
        }
      }
    }
    ILOG_INFO << "mono prepare success = " << prepare_success;
    if (prepare_success) {
      // ILOG_INFO << "use mono prepare to find target point successful!";
      calc_params_.use_mono_tang = true;
      break;
    }
    // inside and outside are occupied
    // if mono prepare fail, use multi prepare to find target point
    if (MultiPreparePlan(target_pose.pos)) {
      ILOG_INFO << "ego_pose = " << input_.ego_pose.pos.transpose()
                << " heading = " << input_.ego_pose.heading * kRad2Deg;
      ILOG_INFO << "point_tangent = " << target_pose.pos.transpose()
                << " heading = " << target_pose.heading * kRad2Deg;

      calc_params_.cal_tang_pt_success = true;
      calc_params_.safe_circle_tang_pt = target_pose;
      // TODO: to find a suitable current pose by dubins
      input.Set(input_.ego_pose.pos, target_pose.pos, input_.ego_pose.heading,
                target_pose.heading);

      dubins_planner_.SetInput(input);
      prepare_success =
          dubins_planner_.OneStepDubinsUpdateByVer(kMinSingleGearPathLength);

      if (!prepare_success &&
          (target_pose.pos.x() - input_.pt_0.x() < input_.channel_width)) {
        prepare_success = ComputePreparePointSecond(input, target_pose);
        if (prepare_success) {
          calc_params_.safe_circle_tang_pt =
              calc_params_.second_prepareplan_input;
        }
      }
    }
    ILOG_INFO << "multi prepare success = " << prepare_success;
    if (prepare_success) {
      // ILOG_INFO << "use multi prepare to find target point successful!";
      calc_params_.use_multi_tang = true;
      break;
    }
    // force quit
    break;
  }

  if (prepare_success) {
    // should col det
    std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
    path_seg_vec.clear();
    path_seg_vec.reserve(3);
    pnc::geometry_lib::PathSegment path_seg;
    // set arc AB
    if (dubins_planner_.GetOutput().gear_cmd_vec[0] !=
        pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
      path_seg.arc_seg = dubins_planner_.GetOutput().arc_AB;
      path_seg_vec.emplace_back(path_seg);
    }
    // set line BC
    if (dubins_planner_.GetOutput().gear_cmd_vec[1] !=
        pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;
      path_seg.line_seg = dubins_planner_.GetOutput().line_BC;
      path_seg_vec.emplace_back(path_seg);
    }
    // set arc CD
    if (dubins_planner_.GetOutput().gear_cmd_vec[2] !=
        pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
      path_seg.arc_seg = dubins_planner_.GetOutput().arc_CD;
      path_seg_vec.emplace_back(path_seg);
    }

    for (pnc::geometry_lib::PathSegment& path_seg : path_seg_vec) {
      CollisionDetector::CollisionResult col_res;
      if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
        auto& line = path_seg.line_seg;
        col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
      } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        auto& arc = path_seg.arc_seg;
        col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
      }
      const double safe_remain_dist =
          std::min(col_res.remain_car_dist,
                   col_res.remain_obstacle_dist -
                       apa_param.GetParam().col_obs_safe_dist_normal);
      if (col_res.remain_car_dist - 1e-3 > safe_remain_dist ||
          safe_remain_dist < 1e-5) {
        ILOG_INFO << "remain_car_dist = " << col_res.remain_car_dist
                  << " safe_remain_dist = " << safe_remain_dist;
        prepare_success = false;
        break;
      }
    }
  }
  return prepare_success;
}

// prepare second plan
const bool PerpendicularPathHeadingInPlanner::DubinsPlan(
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const double turn_radius,
    const double min_length, const bool need_col_det,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  pnc::dubins_lib::DubinsLibrary::Input input;
  input.radius = turn_radius;
  input.Set(start_pose.pos, target_pose.pos, start_pose.heading,
            target_pose.heading);

  dubins_planner_.SetInput(input);
  bool success = dubins_planner_.OneStepDubinsUpdateByVer(min_length);
  ILOG_INFO << "dubins plan = " << success;
  if (success) {
    path_seg_vec.clear();
    path_seg_vec.reserve(3);
    pnc::geometry_lib::PathSegment path_seg;
    // set arc AB
    if (dubins_planner_.GetOutput().gear_cmd_vec[0] !=
        pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_gear = dubins_planner_.GetOutput().gear_cmd_vec[0];
      path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
      path_seg.arc_seg = dubins_planner_.GetOutput().arc_AB;
      path_seg_vec.emplace_back(path_seg);
    }
    // set line BC
    if (dubins_planner_.GetOutput().gear_cmd_vec[1] !=
        pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_gear = dubins_planner_.GetOutput().gear_cmd_vec[1];
      path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;
      path_seg.line_seg = dubins_planner_.GetOutput().line_BC;
      path_seg_vec.emplace_back(path_seg);
    }
    // set arc CD
    if (dubins_planner_.GetOutput().gear_cmd_vec[2] !=
        pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_gear = dubins_planner_.GetOutput().gear_cmd_vec[2];
      path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
      path_seg.arc_seg = dubins_planner_.GetOutput().arc_CD;
      path_seg_vec.emplace_back(path_seg);
    }
  }

  if (need_col_det) {
    for (pnc::geometry_lib::PathSegment& path_seg : path_seg_vec) {
      CollisionDetector::CollisionResult col_res;
      if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
        auto& line = path_seg.line_seg;
        col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
      } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        auto& arc = path_seg.arc_seg;
        col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
      }
      const double safe_remain_dist =
          std::min(col_res.remain_car_dist,
                   col_res.remain_obstacle_dist -
                       apa_param.GetParam().col_obs_safe_dist_normal);
      if (col_res.remain_car_dist - 1e-3 > safe_remain_dist ||
          safe_remain_dist < 1e-5) {
        success = false;
        break;
      }
    }
  }

  return success;
}

const bool PerpendicularPathHeadingInPlanner::PreparePlanSecond() {
  const double dist =
      (input_.ego_pose.pos - calc_params_.safe_circle_tang_pt.pos).norm();
  const double heading_err = std::fabs(
      input_.ego_pose.heading - calc_params_.safe_circle_tang_pt.heading);

  if (dist < apa_param.GetParam().prepare_directly_use_tangent_pos_err &&
      heading_err <
          apa_param.GetParam().prepare_directly_use_tangent_heading_err *
              kDeg2Rad) {
    ILOG_INFO << "use ego pose to multi plan when prepare second";
    return false;
  }

  double min_dubins_path = kMinSingleGearPathLength;
  if (calc_params_.first_path_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    // current dubins gear must be reverse, and the path is continue to multi
    // or adjust plan so no length need for it
    min_dubins_path = 0.086;
  }

  ILOG_INFO << "second prepareplan input = " << input_.ego_pose.pos.transpose()
            << ", " << input_.ego_pose.heading * kRad2Deg << ", "
            << calc_params_.safe_circle_tang_pt.pos.transpose() << ", "
            << calc_params_.safe_circle_tang_pt.heading * kRad2Deg;

  pnc::geometry_lib::PathPoint new_tang_pt;
  new_tang_pt = calc_params_.safe_circle_tang_pt;
  // new_tang_pt.pos.y() += calc_params_.target_line_y_offset_sign *
  //                        apa_param.GetParam().headin_max_pt_inside_drop_dy *
  //                        calc_params_.slot_side_sgn;
  // try dubins no col det, use dynamic col det to avoid col
  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
  const bool dubins_success =
      DubinsPlan(input_.ego_pose, new_tang_pt, calc_params_.turn_radius,
                 min_dubins_path, false, path_seg_vec) &&
      (path_seg_vec.size() > 0) &&
      (dubins_planner_.GetOutput().current_gear_cmd !=
       calc_params_.first_path_gear);

  if (dubins_success) {
    output_.path_available = true;
    for (const pnc::geometry_lib::PathSegment& path_seg : path_seg_vec) {
      output_.path_segment_vec.emplace_back(path_seg);
      output_.length += path_seg.Getlength();
      output_.gear_cmd_vec.emplace_back(path_seg.seg_gear);
      output_.steer_vec.emplace_back(path_seg.seg_steer);
    }
    output_.current_gear = output_.gear_cmd_vec.front();
    input_.ego_pose = path_seg_vec.back().GetEndPose();
  }
  ILOG_INFO << "second prepareplan success  = " << dubins_success;
  calc_params_.second_prepareplan_success = dubins_success;
  return dubins_success;
}

// multiplan
const bool PerpendicularPathHeadingInPlanner::CheckMultiPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose,
    const double& slot_occupied_ratio) {
  const double heading_error = pnc::geometry_lib::NormalizeAngle(
      current_pose.heading - calc_params_.target_line.heading);

  if ((std::fabs(heading_error) <=
       apa_param.GetParam().headin_multi_plan_min_heading_err * kDeg2Rad) ||
      slot_occupied_ratio >=
          apa_param.GetParam().headin_multi_plan_max_occupied_ratio) {
    ILOG_INFO << "pose err is relatively small, multi plan is not suitable";
    return false;
  }
  return true;
}

const bool PerpendicularPathHeadingInPlanner::CheckReachTargetPose(
    const pnc::geometry_lib::PathPoint& current_pose) {
  if ((current_pose.pos - calc_params_.target_line.pA).norm() <=
          apa_param.GetParam().target_pos_err &&
      std::fabs(pnc::geometry_lib::NormalizeAngle(
          current_pose.heading - input_.tlane.pt_terminal_heading)) <=
          apa_param.GetParam().target_heading_err * kDeg2Rad) {
    return true;
  }
  return false;
}

const bool PerpendicularPathHeadingInPlanner::OneLinePlan(
    pnc::geometry_lib::LineSegment& line,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear) {
  ILOG_INFO << "--- try one line plan ---";
  pnc::geometry_lib::PathPoint pose;
  pose.Set(line.pA, line.heading);
  if (pnc::geometry_lib::IsPoseOnLine(
          pose, calc_params_.target_line,
          apa_param.GetParam().target_pos_err - 1e-6,
          apa_param.GetParam().target_heading_err * kDeg2Rad - 1e-6)) {
    ILOG_INFO << "pose is on line, success";
    line.pB.x() = calc_params_.target_line.pA.x();
    line.pB.y() = line.pA.y();
    line.length = (line.pB - line.pA).norm();
    line.heading = calc_params_.target_line.heading;
    const uint8_t seg_gear = pnc::geometry_lib::CalLineSegGear(line);
    if (seg_gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
        seg_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) {
      ILOG_INFO << "the line gear is invalid";
      return false;
    }
    pnc::geometry_lib::PathSegment line_seg(seg_gear, line);
    path_seg_vec.emplace_back(line_seg);
    return true;
  } else {
    ILOG_INFO << "pose is not on line, fail";
    return false;
  }
}

const bool PerpendicularPathHeadingInPlanner::CheckArcOrLineAvailable(
    const pnc::geometry_lib::Arc& arc) {
  const pnc::geometry_lib::PathPoint pose1(arc.pA, arc.headingA);
  const pnc::geometry_lib::PathPoint pose2(arc.pB, arc.headingB);
  if (pnc::geometry_lib::CheckTwoPoseIsSame(
          pose1, pose2, apa_param.GetParam().static_pos_eps,
          apa_param.GetParam().static_heading_eps * kDeg2Rad)) {
    ILOG_INFO << "arc.pA = " << arc.pA.transpose()
              << "  arc.headingA = " << arc.headingA * kRad2Deg
              << "  arc.pB = " << arc.pB.transpose()
              << "  arc.headingB = " << arc.headingB * kRad2Deg;

    return false;
  }
  return true;
}

const bool PerpendicularPathHeadingInPlanner::TwoArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear, const uint8_t current_arc_steer) {
  pnc::geometry_lib::Arc arc2;
  // calc_params_.target_line.pA.y() =
  // apa_param.GetParam().headin_max_pt_inside_drop_dy;
  // calc_params_.target_line.pB.y() =
  // apa_param.GetParam().headin_max_pt_inside_drop_dy;
  if (!CalTwoArcWithLine(arc, arc2, calc_params_.target_line)) {
    ILOG_INFO << "TwoArcPlan fail 0";
    return false;
  }

  if (CheckArcOrLineAvailable(arc)) {
    uint8_t steer_1 = pnc::geometry_lib::CalArcSteer(arc);
    uint8_t gear_1 = pnc::geometry_lib::CalArcGear(arc);
    if (steer_1 == current_arc_steer && gear_1 == current_gear) {
      pnc::geometry_lib::PathSegment arc_seg1(steer_1, gear_1, arc);
      path_seg_vec.emplace_back(arc_seg1);
    }
  } else {
    ILOG_INFO << "TwoArcPlan fail 1";
    return false;
  }

  if (CheckArcOrLineAvailable(arc2)) {
    uint8_t next_arc_steer =
        (current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT)
            ? pnc::geometry_lib::SEG_STEER_RIGHT
            : pnc::geometry_lib::SEG_STEER_LEFT;

    uint8_t next_gear = (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE)
                            ? pnc::geometry_lib::SEG_GEAR_REVERSE
                            : pnc::geometry_lib::SEG_GEAR_DRIVE;

    uint8_t steer_2 = pnc::geometry_lib::CalArcSteer(arc2);
    uint8_t gear_2 = pnc::geometry_lib::CalArcGear(arc2);
    if (steer_2 == next_arc_steer && gear_2 == next_gear) {
      pnc::geometry_lib::PathSegment arc_seg2(steer_2, gear_2, arc2);
      path_seg_vec.emplace_back(arc_seg2);
    }
  }
  return true;
}

const bool PerpendicularPathHeadingInPlanner::OneArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear, const uint8_t current_arc_steer) {
  if (!calc_params_.use_mono_circle_headin) {
    auto target_line_tmp = calc_params_.target_line;
    if (calc_params_.is_inside_occupied) {
      target_line_tmp.pA.y() +=
          0.4 * calc_params_.slot_side_sgn *
          calc_params_.target_line_y_offset_sign *
          apa_param.GetParam().headin_max_pt_inside_drop_dy;

      target_line_tmp.pB.y() +=
          0.4 * calc_params_.slot_side_sgn *
          calc_params_.target_line_y_offset_sign *
          apa_param.GetParam().headin_max_pt_inside_drop_dy;

    } else if (calc_params_.is_outside_occupied) {
      target_line_tmp.pA.y() +=
          0.5 * calc_params_.slot_side_sgn *
          calc_params_.target_line_y_offset_sign *
          apa_param.GetParam().headin_max_pt_inside_drop_dy;

      target_line_tmp.pB.y() +=
          0.5 * calc_params_.slot_side_sgn *
          calc_params_.target_line_y_offset_sign *
          apa_param.GetParam().headin_max_pt_inside_drop_dy;
    }

    if (!pnc::geometry_lib::CalOneArcWithLine(
            arc, target_line_tmp, apa_param.GetParam().target_radius_err)) {
      ILOG_INFO << "OneArcPlan fail 0";
      return false;
    }
  } else {
    if (!pnc::geometry_lib::CalOneArcWithLine(
            arc, calc_params_.target_line,
            apa_param.GetParam().target_radius_err)) {
      ILOG_INFO << "OneArcPlan fail 0";
      return false;
    }
  }

  if (CheckArcOrLineAvailable(arc)) {
    uint8_t steer = pnc::geometry_lib::CalArcSteer(arc);
    uint8_t gear = pnc::geometry_lib::CalArcGear(arc);
    if (steer == current_arc_steer && gear == current_gear) {
      // TODO cut the path avoid totally into the slot
      // if (!calc_params_.use_mono_circle_headin &&
      // calc_params_.is_inside_occupied &&
      //     (arc.pB.x() < 0.5 * input_.pt_0.x())) {
      //   double heading_low = arc.headingA;
      //   double heading_high = arc.headingB;
      //   auto pos_tmp = arc.pB;
      //   double heading_tmp = arc.headingB;
      //   const auto OA = arc.pA - arc.circle_info.center;
      //   while (pos_tmp.x() < input_.pt_0.x()) {
      //     heading_tmp = 0.5 * (heading_low + heading_high);
      //     const double heading_error = heading_tmp - heading_low;
      //     pos_tmp = arc.circle_info.center +
      //               pnc::geometry_lib::GetRotm2dFromTheta(heading_error) *
      //               OA;

      //     if (pos_tmp.x() < input_.pt_0.x()) {
      //       heading_high = heading_tmp;
      //     }
      //   }
      //   arc.pB = pos_tmp;
      //   arc.headingB = heading_tmp;
      //   arc.length =
      //       arc.circle_info.radius * std::fabs(arc.headingB - arc.headingA);

      //   std::cout << "cut one arc path" << std::endl;
      // }
      pnc::geometry_lib::PathSegment arc_seg(steer, gear, arc);
      path_seg_vec.emplace_back(arc_seg);
      return true;
    }
  }

  ILOG_INFO << "OneArcPlan fail 1";
  return false;
}

const bool PerpendicularPathHeadingInPlanner::CheckArcOrLineAvailable(
    const pnc::geometry_lib::LineSegment& line) {
  const pnc::geometry_lib::PathPoint pose1(line.pA, line.heading);
  const pnc::geometry_lib::PathPoint pose2(line.pB, line.heading);
  if (pnc::geometry_lib::CheckTwoPoseIsSame(
          pose1, pose2, apa_param.GetParam().static_pos_eps,
          apa_param.GetParam().static_heading_eps * kDeg2Rad)) {
    return false;
  }
  return true;
}

const bool PerpendicularPathHeadingInPlanner::LineArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear, const uint8_t current_arc_steer) {
  pnc::geometry_lib::LineSegment line_seg1;
  line_seg1 = pnc::geometry_lib::BuildLineSegByPose(arc.pA, arc.headingA);

  pnc::geometry_lib::LineSegment line_seg2;
  line_seg2 = calc_params_.target_line;
  if (!calc_params_.use_mono_circle_headin) {
    if (calc_params_.is_inside_occupied) {
      line_seg2.pA.y() = line_seg2.pA.y() +
                         0.4 * calc_params_.slot_side_sgn *
                             calc_params_.target_line_y_offset_sign *
                             apa_param.GetParam().headin_max_pt_inside_drop_dy;

      line_seg2.pB.y() = line_seg2.pB.y() +
                         0.4 * calc_params_.slot_side_sgn *
                             calc_params_.target_line_y_offset_sign *
                             apa_param.GetParam().headin_max_pt_inside_drop_dy;
    } else if (calc_params_.is_outside_occupied) {
      line_seg2.pA.y() = line_seg2.pA.y() +
                         0.5 * calc_params_.slot_side_sgn *
                             calc_params_.target_line_y_offset_sign *
                             apa_param.GetParam().headin_max_pt_inside_drop_dy;

      line_seg2.pB.y() = line_seg2.pB.y() +
                         0.5 * calc_params_.slot_side_sgn *
                             calc_params_.target_line_y_offset_sign *
                             apa_param.GetParam().headin_max_pt_inside_drop_dy;
    }
  }

  std::vector<Eigen::Vector2d> centers;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> tangent_ptss;
  if (!pnc::geometry_lib::CalCommonTangentCircleOfTwoLine(
          line_seg1, line_seg2, arc.circle_info.radius, centers,
          tangent_ptss)) {
    ILOG_INFO << "LineArcPlan fail 0";
    return false;
  }

  bool line_arc_success = false;
  // check which center and tang pt is suitable
  for (size_t i = 0; i < centers.size(); ++i) {
    path_seg_vec.clear();
    path_seg_vec.reserve(2);

    bool line_available = false;
    pnc::geometry_lib::LineSegment tmp_line(line_seg1.pA, tangent_ptss[i].first,
                                            line_seg1.heading);
    if (CheckArcOrLineAvailable(tmp_line)) {
      if (pnc::geometry_lib::CalLineSegGear(tmp_line) == current_gear) {
        pnc::geometry_lib::PathSegment line_seg(current_gear, tmp_line);
        path_seg_vec.emplace_back(line_seg);
        line_available = true;
      }
    }

    bool arc_available = false;
    pnc::geometry_lib::Arc tmp_arc;
    tmp_arc.circle_info.center = centers[i];
    tmp_arc.pA = tangent_ptss[i].first;
    tmp_arc.headingA = line_seg1.heading;
    tmp_arc.pB = tangent_ptss[i].second;
    if (CheckArcOrLineAvailable(tmp_arc) &&
        pnc::geometry_lib::CompleteArcInfo(tmp_arc)) {
      double head_err = std::fabs(pnc::geometry_lib::NormalizeAngle(
          tmp_arc.headingB - calc_params_.target_line.heading));
      const uint8_t tmp_arc_steer = pnc::geometry_lib::CalArcSteer(tmp_arc);
      const uint8_t tmp_gear = pnc::geometry_lib::CalArcGear(tmp_arc);
      if (tmp_arc_steer == current_arc_steer && tmp_gear == current_gear &&
          head_err <= apa_param.GetParam().static_heading_eps * kDeg2Rad) {
        pnc::geometry_lib::PathSegment arc_seg(tmp_arc_steer, tmp_gear,
                                               tmp_arc);
        path_seg_vec.emplace_back(arc_seg);
        arc_available = true;
      }
    }

    if (path_seg_vec.size() == 2) {
      line_arc_success = true;
      break;
    }

    if (path_seg_vec.size() == 1 && line_available) {
      double dist = pnc::geometry_lib::CalPoint2LineDist(
          tmp_line.pB, calc_params_.target_line);
      double head_err = std::fabs(pnc::geometry_lib::NormalizeAngle(
          tmp_line.heading - calc_params_.target_line.heading));
      if (dist <= apa_param.GetParam().target_pos_err - 1e-6 &&
          head_err <=
              apa_param.GetParam().target_heading_err * kDeg2Rad - 1e-6) {
        line_arc_success = true;
        break;
      }
    }

    if (path_seg_vec.size() == 1 && arc_available) {
      const double dist = (arc.pA - tmp_arc.pA).norm();
      const double heading_err = std::fabs(arc.headingA - tmp_arc.headingA);
      if (dist < apa_param.GetParam().target_pos_err - 1e-6 &&
          heading_err <
              apa_param.GetParam().target_heading_err * kDeg2Rad - 1e-6) {
        line_arc_success = true;
        break;
      }
    }
  }
  if (!line_arc_success) {
    path_seg_vec.clear();
  }
  if (path_seg_vec.empty()) {
    ILOG_INFO << "LineArcPlan fail 1";
    return false;
  }
  return true;
}

const bool PerpendicularPathHeadingInPlanner::CalSinglePathInMulti(
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const uint8_t current_arc_steer,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const double turn_radius, size_t i) {
  ILOG_INFO << "-----CalSinglePathInMulti-----";
  ILOG_INFO << "current_arc_steer = " << static_cast<int>(current_arc_steer)
            << ",  current_gear = " << static_cast<int>(current_gear)
            << ",  current_pos = " << current_pose.pos.transpose()
            << ",  current_heading = " << current_pose.heading * kRad2Deg;

  const double lat_err = apa_param.GetParam().target_pos_err;
  const double heading_err = apa_param.GetParam().target_heading_err;
  if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    apa_param.SetPram().target_pos_err = lat_err * 0.5;
    apa_param.SetPram().target_heading_err = heading_err * 0.5;
  }

  auto temp_pose = current_pose;
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(4);

  // first: try one line plan to target pose
  if (!CheckReachTargetPose(temp_pose)) {
    // try line
    pnc::geometry_lib::LineSegment line_seg;
    line_seg.pA = temp_pose.pos;
    line_seg.heading = temp_pose.heading;
    if (OneLinePlan(line_seg, tmp_path_seg_vec, input_.ref_gear)) {
      temp_pose.pos = line_seg.pB;
      temp_pose.heading = line_seg.heading;
      ILOG_INFO << "OneLinePlan success";
    } else {
      ILOG_INFO << "OneLinePlan fail";
    }
  }

  // second: try two arc(0), one arc(1) or line arc(2) plan to target line
  using namespace pnc::geometry_lib;
  PathPlanType play_type = PLAN_TYPE_INVALID;
  if (!CheckReachTargetPose(temp_pose)) {
    const double current_turn_radius = turn_radius;
    const Eigen::Vector2d current_tang_vec =
        pnc::geometry_lib::GetUnitTangVecByHeading(temp_pose.heading);

    Eigen::Vector2d current_norm_vec;
    if (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
      current_norm_vec << current_tang_vec.y(), -current_tang_vec.x();
    } else if (current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
      current_norm_vec << -current_tang_vec.y(), current_tang_vec.x();
    }

    const Eigen::Vector2d current_turn_center =
        temp_pose.pos + current_norm_vec * current_turn_radius;

    pnc::geometry_lib::Arc current_arc;
    current_arc.circle_info.center = current_turn_center;
    current_arc.circle_info.radius = current_turn_radius;
    current_arc.pA = temp_pose.pos;
    current_arc.headingA = pnc::geometry_lib::NormalizeAngle(temp_pose.heading);

    // cal the dist from current_turn_center to target line
    ILOG_INFO << "current_turn_center = " << current_turn_center.transpose();
    double dist = pnc::geometry_lib::CalPoint2LineDist(
        current_turn_center, calc_params_.target_line);
    ILOG_INFO << "origin dist = " << dist;
    if (!calc_params_.use_mono_circle_headin &&
        (!calc_params_.second_prepareplan_success &&
         calc_params_.use_mono_tang && calc_params_.is_inside_occupied)) {
      dist += apa_param.GetParam().headin_max_pt_inside_drop_dy;
    }

    ILOG_INFO << "final dist = " << dist;
    // two arc
    if (dist < current_turn_radius - apa_param.GetParam().target_radius_err) {
      ILOG_INFO << "center to line dist = " << dist << ",  try TwoArcPlan";
      play_type = PLAN_TYPE_TWO_ARC;
      if (TwoArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                     current_arc_steer)) {
        ILOG_INFO << "TwoArcPlan success";
      } else {
        ILOG_INFO << "TwoArcPlan fail";
      }
      // one arc
    } else if ((dist >=
                current_turn_radius - apa_param.GetParam().target_radius_err) &&
               (dist <=
                current_turn_radius + apa_param.GetParam().target_radius_err)) {
      ILOG_INFO << "center to line dist = " << dist << ",  try OneArcPlan";
      play_type = PLAN_TYPE_ONE_ARC;
      if (OneArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                     current_arc_steer)) {
        ILOG_INFO << "OneArcPlan success";
      } else {
        ILOG_INFO << "OneArcPlan fail";
      }
      // line arc
    } else if (dist >
               current_turn_radius + apa_param.GetParam().target_radius_err) {
      ILOG_INFO << "center to line dist = " << dist << ",  try LineArcPlan";
      play_type = PLAN_TYPE_LINE_ARC;
      if (LineArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                      current_arc_steer)) {
        ILOG_INFO << "LineArcPlan success";
      } else {
        ILOG_INFO << "LineArcPlan fail";
      }
    }
    if (!tmp_path_seg_vec.empty()) {
      const auto& path_seg = tmp_path_seg_vec.back();
      if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
        temp_pose.pos = path_seg.GetLineSeg().pB;
        temp_pose.heading = path_seg.GetLineSeg().heading;
      } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        temp_pose.pos = path_seg.GetArcSeg().pB;
        temp_pose.heading = path_seg.GetArcSeg().headingB;
      }
    }
  }

  // third: try one line plan to target pose
  if (!CheckReachTargetPose(temp_pose)) {
    // try line
    pnc::geometry_lib::LineSegment line_seg;
    line_seg.pA = temp_pose.pos;
    line_seg.heading = temp_pose.heading;
    if (OneLinePlan(line_seg, tmp_path_seg_vec, input_.ref_gear)) {
      temp_pose.pos = line_seg.pB;
      temp_pose.heading = line_seg.heading;
      ILOG_INFO << "OneLinePlan success";
    } else {
      ILOG_INFO << "OneLinePlan fail";
    }
  }

  // exit multiplan, continue new multi line arc plan
  if (play_type == PLAN_TYPE_TWO_ARC && tmp_path_seg_vec.size() <= 0) {
    calc_params_.use_line_arc = true;
    ILOG_INFO << "two arc fail";
    return false;
  }

  // exit multiplan, continue adjust plan
  // if (type > 0 && !calc_params_.use_mono_circle_headin) {
  //   calc_params_.use_adjust = true;
  // }

  // avoid line arc length too length which let car go too far
  if (play_type == PLAN_TYPE_LINE_ARC &&
      current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    const double channel_width =
        apa_param.GetParam().line_arc_obs_channel_width;
    const double channel_length =
        apa_param.GetParam().line_arc_obs_channel_length;

    const double pt_01_x = ((input_.pt_0 + input_.pt_1) * 0.5).x();
    const double top_x = pt_01_x + channel_width / input_.sin_angle;

    Eigen::Vector2d channel_point_1 =
        Eigen::Vector2d(top_x, 0.0) +
        (input_.pt_0 - input_.pt_1).normalized() * channel_length * 0.5;
    Eigen::Vector2d channel_point_2 =
        Eigen::Vector2d(top_x, 0.0) -
        (input_.pt_0 - input_.pt_1).normalized() * channel_length * 0.5;

    pnc::geometry_lib::LineSegment channel_line;
    channel_line.SetPoints(channel_point_1, channel_point_2);
    std::vector<Eigen::Vector2d> point_set;
    pnc::geometry_lib::SamplePointSetInLineSeg(
        point_set, channel_line, apa_param.GetParam().obstacle_ds);
    collision_detector_ptr_->AddObstacles(point_set,
                                          CollisionDetector::LINEARC_OBS);
    ILOG_INFO << "add linearc obs";
  } else {
    collision_detector_ptr_->DeleteObstacles(CollisionDetector::LINEARC_OBS);
  }

  apa_param.SetPram().target_pos_err = lat_err;
  apa_param.SetPram().target_heading_err = heading_err;

  // collison detection
  for (pnc::geometry_lib::PathSegment& tmp_path_seg : tmp_path_seg_vec) {
    double safe_dist = apa_param.GetParam().col_obs_safe_dist_normal;
    CollisionDetector::Paramters params;
    params.lat_inflation = apa_param.GetParam().car_lat_inflation_normal;
    if (calc_params_.first_multi_plan &&
        (play_type == PLAN_TYPE_TWO_ARC || play_type == PLAN_TYPE_ONE_ARC ||
         play_type == PLAN_TYPE_LINE_ARC) &&
        i == 0 && tmp_path_seg.seg_gear == current_gear) {
      // when 1R and two arc, should far from inside obs
      params.lat_inflation = apa_param.GetParam().car_lat_inflation_strict;
      safe_dist = apa_param.GetParam().col_obs_safe_dist_strict;
    }
    collision_detector_ptr_->SetParam(params);
    // PrintSegmentInfo(tmp_path_seg);
    const PathColDetRes path_col_det_res =
        TrimPathByCollisionDetection(tmp_path_seg, safe_dist);

    if (path_col_det_res == PathColDetRes::NORMAL) {
      path_seg_vec.emplace_back(tmp_path_seg);
    } else if (path_col_det_res == PathColDetRes::SHORTEN) {
      path_seg_vec.emplace_back(tmp_path_seg);
      break;
    } else if (path_col_det_res == PathColDetRes::INVALID) {
      break;
    } else if (path_col_det_res == PathColDetRes::SINGLE_PLAN_AGAIN) {
      calc_params_.single_plan_again = true;
      return false;
    } else if (path_col_det_res == PathColDetRes::COMPLETE_PLAN_AGAIN) {
      calc_params_.complete_plan_again = true;
      return false;
    }
  }

  collision_detector_ptr_->DeleteObstacles(CollisionDetector::LINEARC_OBS);
  // postprocess path
  if (!path_seg_vec.empty()) {
    uint8_t current_gear = path_seg_vec.front().seg_gear;
    double length = 0.0;
    bool collision_flag = false;
    for (const auto& path_seg : path_seg_vec) {
      // PrintSegmentInfo(path_seg);
      if (path_seg.seg_gear == current_gear) {
        length += path_seg.Getlength();
        collision_flag = path_seg.collision_flag;
      }
    }
    if (length < apa_param.GetParam().min_gear_path_length && collision_flag) {
      ILOG_INFO << "this gear path is too small, lose it";
      path_seg_vec.clear();
    }
    ILOG_INFO << "CalSinglePathInMulti success";
    return true;
  } else {
    ILOG_INFO << "CalSinglePathInMulti fail";
    return false;
  }
}

const bool PerpendicularPathHeadingInPlanner::MultiPlan() {
  ILOG_INFO << "-----multiplan2-----";
  // set init state
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;

  // check pose and slot_occupied_ratio, if error is small, exit multiplan
  if (!CheckMultiPlanSuitable(current_pose, input_.slot_occupied_ratio)) {
    return false;
  }

  // check gear and steer, if input error, break
  if ((current_gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
       current_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) ||
      (current_arc_steer != pnc::geometry_lib::SEG_STEER_LEFT &&
       current_arc_steer != pnc::geometry_lib::SEG_STEER_RIGHT)) {
    ILOG_INFO << "ref_gear or ref_arc_steer error";
    return false;
  }

  ILOG_INFO << "try multi plan to target point";
  Output multi_output;
  bool success = false;
  calc_params_.multi_plan = true;
  double turn_radius =
      calc_params_.turn_radius + apa_param.GetParam().radius_add;

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  for (size_t i = 0; i < kMultiPlanMaxPathNumsInSlot; i++) {
    ILOG_INFO << "-- No." << i << "  multi-plan --";
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);
    if (CalSinglePathInMulti(current_pose, current_gear, current_arc_steer,
                             tmp_path_seg_vec, turn_radius, i)) {
      multi_output.path_available = true;
      success = true;
    }

    if (tmp_path_seg_vec.size() > 0) {
      for (const auto& tmp_path_seg : tmp_path_seg_vec) {
        multi_output.path_segment_vec.emplace_back(tmp_path_seg);
        multi_output.length += tmp_path_seg.Getlength();
        multi_output.gear_cmd_vec.emplace_back(tmp_path_seg.seg_gear);
        multi_output.steer_vec.emplace_back(tmp_path_seg.seg_steer);
      }
      const auto& last_segment = tmp_path_seg_vec.back();
      pnc::geometry_lib::PathPoint last_pose;
      if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        last_pose.Set(last_segment.GetArcSeg().pB,
                      last_segment.GetArcSeg().headingB);

      } else if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
        last_pose.Set(last_segment.GetLineSeg().pB,
                      last_segment.GetLineSeg().heading);
      }
      current_pose = last_pose;
      current_gear =
          (last_segment.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE)
              ? pnc::geometry_lib::SEG_GEAR_DRIVE
              : pnc::geometry_lib::SEG_GEAR_REVERSE;

      if (last_segment.seg_steer == pnc::geometry_lib::SEG_STEER_STRAIGHT) {
        current_arc_steer =
            (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT)
                ? pnc::geometry_lib::SEG_STEER_LEFT
                : pnc::geometry_lib::SEG_STEER_RIGHT;
      } else {
        current_arc_steer =
            (last_segment.seg_steer == pnc::geometry_lib::SEG_STEER_RIGHT)
                ? pnc::geometry_lib::SEG_STEER_LEFT
                : pnc::geometry_lib::SEG_STEER_RIGHT;
      }

    } else {
      current_gear = (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE)
                         ? pnc::geometry_lib::SEG_GEAR_DRIVE
                         : pnc::geometry_lib::SEG_GEAR_REVERSE;

      current_arc_steer =
          (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT)
              ? pnc::geometry_lib::SEG_STEER_LEFT
              : pnc::geometry_lib::SEG_STEER_RIGHT;

      break;
    }

    if (calc_params_.use_line_arc) {
      std::cout << "use line arc, break\n";
      break;
    }

    if (CheckReachTargetPose(current_pose)) {
      break;
    }
  }

  if (multi_output.path_segment_vec.size() > 0 &&
      multi_output.gear_cmd_vec.front() != input_.ref_gear) {
    multi_output.Reset();
    success = false;
  }

  if (multi_output.path_segment_vec.size() > 0) {
    success = true;
    output_.path_available = true;
    output_.length += multi_output.length;
    output_.path_segment_vec.insert(output_.path_segment_vec.end(),
                                    multi_output.path_segment_vec.begin(),
                                    multi_output.path_segment_vec.end());
    output_.gear_cmd_vec.insert(output_.gear_cmd_vec.end(),
                                multi_output.gear_cmd_vec.begin(),
                                multi_output.gear_cmd_vec.end());
    output_.steer_vec.insert(output_.steer_vec.end(),
                             multi_output.steer_vec.begin(),
                             multi_output.steer_vec.end());
  }

  if (!success) {
    ILOG_INFO << "multi plan failed!";
  }
  return success;
}

// multi line arc plan
const bool PerpendicularPathHeadingInPlanner::CheckAdjustPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose) {
  const auto heading_error = std::fabs(pnc::geometry_lib::NormalizeAngle(
      current_pose.heading - calc_params_.target_line.heading));
  if (calc_params_.is_inside_occupied && calc_params_.is_outside_occupied) {
    if (heading_error <=
        apa_param.GetParam().headin_adjust_plan_max_heading1_err * kDeg2Rad) {
      apa_param.SetPram().headin_sturn_steer_ratio_dist =
          std::fabs(current_pose.pos.y()) + 1e-6;
      return true;
    }
    ILOG_INFO << "both occupied: current pose is not suitable for adjust plan";
    return false;
  } else {
    if (heading_error <=
            apa_param.GetParam().headin_adjust_plan_max_heading1_err *
                kDeg2Rad ||
        (heading_error <=
             apa_param.GetParam().headin_adjust_plan_max_heading2_err *
                 kDeg2Rad &&
         ((std::fabs(current_pose.pos.x()) <=
           calc_params_.target_line.pA.x() +
               apa_param.GetParam().headin_adjust_plan_max_lon_err) ||
          (std::fabs(current_pose.pos.y()) <=
           apa_param.GetParam().headin_sturn_steer_ratio_dist)))) {
      return true;
    }
    ILOG_INFO << "current pose is not suitable for adjust plan";
    return false;
  }
}

const bool PerpendicularPathHeadingInPlanner::MultiLineArcPlan() {
  // set init state
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;
  if (output_.path_segment_vec.size() > 0 && output_.gear_cmd_vec.size() > 0) {
    const auto& last_seg = output_.path_segment_vec.back();
    if (last_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      current_pose.Set(last_seg.GetLineSeg().pB, last_seg.GetLineSeg().heading);
    } else if (last_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      current_pose.Set(last_seg.GetArcSeg().pB, last_seg.GetArcSeg().headingB);
    }

    current_gear =
        (output_.gear_cmd_vec.back() == pnc::geometry_lib::SEG_GEAR_REVERSE)
            ? pnc::geometry_lib::SEG_GEAR_DRIVE
            : pnc::geometry_lib::SEG_GEAR_REVERSE;

    current_arc_steer =
        (output_.steer_vec.back() == pnc::geometry_lib::SEG_STEER_RIGHT)
            ? pnc::geometry_lib::SEG_STEER_LEFT
            : pnc::geometry_lib::SEG_STEER_RIGHT;

    ILOG_INFO << "try line arc plan to target point";
  }

  // check gear and steer
  if ((current_gear != pnc::geometry_lib::SEG_GEAR_REVERSE &&
       current_gear != pnc::geometry_lib::SEG_GEAR_DRIVE) ||
      (current_arc_steer != pnc::geometry_lib::SEG_STEER_LEFT &&
       current_arc_steer != pnc::geometry_lib::SEG_STEER_RIGHT)) {
    ILOG_INFO << "ref_gear or ref_arc_steer error";
    return false;
  }

  if (CheckAdjustPlanSuitable(current_pose)) {
    ILOG_INFO << "swtich to adjust plan";
    return false;
  }
  // TODO: judge two arc if available
  Output plan_output;
  double reverse_line_length = 0.2;
  Eigen::Vector2d intersection;
  pnc::geometry_lib::LineSegment current_line_seg;
  double heading_error = 0.0;
  pnc::geometry_lib::Arc arc;
  bool drive_arc_first = false;

  if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    drive_arc_first = true;
  }

  for (size_t i = 0; i < kLineArcPlanMaxPathNumsInSlot; i++) {
    ILOG_INFO << "--- No." << i << " MultiLineArcPlan ---";
    ILOG_INFO << "current_arc_steer = " << static_cast<int>(current_arc_steer)
              << ",  current_gear = " << static_cast<int>(current_gear)
              << ",  current_pos = " << current_pose.pos.transpose()
              << ",  current_heading = " << current_pose.heading * kRad2Deg;

    // 0. prepare
    Eigen::Vector2d ego_heading_vec =
        pnc::geometry_lib::GenHeadingVec(current_pose.heading);

    heading_error = current_pose.heading - calc_params_.target_line.heading;
    bool use_reverse_line = false;
    bool use_reverse_arc = false;
    double radius_adjust = 0.0;

    // 1.1 if first gear is drive
    if (drive_arc_first) {
      current_line_seg.pA = current_pose.pos;
      current_line_seg.pB = current_pose.pos + ego_heading_vec;
      current_line_seg.heading = current_pose.heading;
      radius_adjust = calc_params_.turn_radius;
      if (pnc::geometry_lib::GetIntersectionFromTwoLine(
              intersection, current_line_seg, calc_params_.target_line)) {
        const double length = (intersection - current_pose.pos).norm();
        const double length_min =
            std::fabs(calc_params_.turn_radius * std::tan(0.5 * heading_error));

        const double length_offset = length - length_min;
        if (length_offset > 0.0) {
          // radius_adjust = calc_params_.turn_radius;
          radius_adjust = (0.02 * length_min + 0.98 * length) /
                          std::fabs(std::tan(0.5 * heading_error));
        }
      }
      Eigen::Vector2d line_norm_vec(ego_heading_vec.y(), -ego_heading_vec.x());
      if (line_norm_vec.x() > 0.0) {
        line_norm_vec = -1.0 * line_norm_vec;
      }

      Eigen::Vector2d circle_center =
          current_pose.pos + radius_adjust * line_norm_vec;

      arc.pA = current_pose.pos;
      arc.headingA = current_pose.heading;
      arc.circle_info.center = circle_center;
      arc.circle_info.radius = radius_adjust;
      arc.length = radius_adjust * std::fabs(heading_error);
      arc.is_anti_clockwise = calc_params_.is_left_side ? true : false;
      pnc::geometry_lib::CompleteArcInfo(arc, arc.length,
                                         arc.is_anti_clockwise);

      pnc::geometry_lib::PathSegment arc_seg_0(current_arc_steer, current_gear,
                                               arc);

      // ILOG_INFO << "arc0 center = " << arc.circle_info.center.transpose()
      //                              << " arc0 end = " << arc.pB.transpose());
      // collision detection
      double safe_dist = apa_param.GetParam().col_obs_safe_dist_strict;
      const PathColDetRes path_col_det_res =
          TrimPathByCollisionDetection(arc_seg_0, safe_dist);
      if (path_col_det_res == PathColDetRes::INVALID) {
        ILOG_INFO << "drive arc path col invalid";
      } else {
        ILOG_INFO << "arc0 end pos = " << arc_seg_0.GetEndPos().transpose()
                  << " arc0 end heading = "
                  << arc_seg_0.GetEndHeading() * kRad2Deg;

        plan_output.gear_cmd_vec.emplace_back(arc_seg_0.seg_gear);
        plan_output.steer_vec.emplace_back(arc_seg_0.seg_steer);
        plan_output.path_segment_vec.emplace_back(arc_seg_0);
        plan_output.length += arc_seg_0.Getlength();
        current_pose = plan_output.path_segment_vec.back().GetEndPose();
        if (CheckAdjustPlanSuitable(current_pose)) {
          ILOG_INFO << "swtich to adjust plan";
          break;
        }
      }

      current_gear = (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE)
                         ? pnc::geometry_lib::SEG_GEAR_DRIVE
                         : pnc::geometry_lib::SEG_GEAR_REVERSE;

      current_arc_steer =
          (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT)
              ? pnc::geometry_lib::SEG_STEER_LEFT
              : pnc::geometry_lib::SEG_STEER_RIGHT;
      drive_arc_first = false;
    }

    // 1.2 verify reverse arc if possible
    // compute a suitable reverse_degree
    // method 1
    // const double real_headin_reverse_deg =
    //     apa_param.GetParam().headin_reverse_deg;
    // method 2
    const std::vector<double> y_tab = {0.0, 0.5, 0.8, 1.2, 1.68};
    const std::vector<double> reverse_degree_tab = {
        2.5 * apa_param.GetParam().headin_reverse_deg,
        2.0 * apa_param.GetParam().headin_reverse_deg,
        1.8 * apa_param.GetParam().headin_reverse_deg,
        1.5 * apa_param.GetParam().headin_reverse_deg,
        apa_param.GetParam().headin_reverse_deg};
    const double real_headin_reverse_deg = pnc::mathlib::Interp1(
        y_tab, reverse_degree_tab, std::fabs(current_pose.pos.y()));

    ego_heading_vec = pnc::geometry_lib::GenHeadingVec(current_pose.heading);

    Eigen::Vector2d line_norm_vec(ego_heading_vec.y(), -ego_heading_vec.x());
    if (line_norm_vec.x() < 0.0) {
      line_norm_vec = -1.0 * line_norm_vec;
    }
    Eigen::Vector2d circle_center =
        current_pose.pos + calc_params_.turn_radius * line_norm_vec;

    pnc::geometry_lib::Arc arc;
    arc.pA = current_pose.pos;
    arc.headingA = current_pose.heading;
    arc.circle_info.center = circle_center;
    arc.circle_info.radius = calc_params_.turn_radius;
    arc.length =
        arc.circle_info.radius *
        std::min(std::fabs(heading_error), real_headin_reverse_deg * kDeg2Rad);

    arc.is_anti_clockwise = calc_params_.is_left_side ? true : false;
    pnc::geometry_lib::CompleteArcInfo(arc, arc.length, arc.is_anti_clockwise);
    pnc::geometry_lib::PathSegment arc_seg(current_arc_steer, current_gear,
                                           arc);

    // ILOG_INFO << "arc center = " << arc.circle_info.center.transpose()
    //                             << " arc end = " << arc.pB.transpose());

    // collision detection
    double safe_dist = apa_param.GetParam().col_obs_safe_dist_normal;
    const PathColDetRes path_col_det_res =
        TrimPathByCollisionDetection(arc_seg, safe_dist);

    if (path_col_det_res == PathColDetRes::INVALID) {
      ILOG_INFO << "path col invalid";
    } else {
      use_reverse_arc = true;
    }

    if (use_reverse_arc) {
      ILOG_INFO << "arc end pos = " << arc_seg.GetEndPos().transpose()
                << " arc end heading = " << arc_seg.GetEndHeading() * kRad2Deg;

      plan_output.gear_cmd_vec.emplace_back(arc_seg.seg_gear);
      plan_output.steer_vec.emplace_back(arc_seg.seg_steer);
      plan_output.path_segment_vec.emplace_back(arc_seg);
      plan_output.length += arc_seg.Getlength();
      current_pose = plan_output.path_segment_vec.back().GetEndPose();
    }

    if (CheckAdjustPlanSuitable(current_pose)) {
      ILOG_INFO << "swtich to adjust plan";
      break;
    }

    // 2. verify adjust radius if possible(variable radius)
    ego_heading_vec = pnc::geometry_lib::GenHeadingVec(current_pose.heading);
    current_line_seg.pA = current_pose.pos;
    current_line_seg.pB = current_pose.pos + ego_heading_vec;
    current_line_seg.heading = current_pose.heading;
    if (pnc::geometry_lib::GetIntersectionFromTwoLine(
            intersection, current_line_seg, calc_params_.target_line)) {
      const double length = (intersection - current_pose.pos).norm();
      const double length_min =
          std::fabs(calc_params_.turn_radius * std::tan(0.5 * heading_error));

      // std::cout << "length_min = " << length_min
      //           << " length_actual = " << length << std::endl;

      const double length_offset = length - length_min;
      if (length_offset > 0.0) {
        // radius_adjust = calc_params_.turn_radius;
        radius_adjust = (0.02 * length_min + 0.98 * length) /
                        std::fabs(std::tan(0.5 * heading_error));
      } else {
        use_reverse_line = true;
        reverse_line_length = std::fabs(length_offset) > reverse_line_length
                                  ? std::fabs(length_offset)
                                  : reverse_line_length;

        radius_adjust = calc_params_.turn_radius;
      }
    }
    // ILOG_INFO << "radius_adjust = " << radius_adjust );
    pnc::geometry_lib::PathPoint target_pose = current_pose;
    if (use_reverse_line) {
      // reverse line seg
      target_pose.pos =
          current_pose.pos - reverse_line_length * ego_heading_vec;
      pnc::geometry_lib::LineSegment line(current_pose.pos, target_pose.pos,
                                          current_pose.heading);

      CollisionDetector::Paramters params;
      params.lat_inflation = apa_param.GetParam().car_lat_inflation_normal;
      collision_detector_ptr_->SetParam(params);
      CollisionDetector::CollisionResult col_res =
          collision_detector_ptr_->UpdateByObsMap(line, line.heading);

      const double safe_remain_dist =
          std::min(col_res.remain_car_dist,
                   col_res.remain_obstacle_dist -
                       apa_param.GetParam().col_obs_safe_dist_normal);

      if (col_res.remain_car_dist - 1e-3 > safe_remain_dist ||
          safe_remain_dist < 1e-5) {
        ILOG_INFO << "line will collide, fail and quit line arc plan";
        return false;
      }

      plan_output.path_available = true;
      pnc::geometry_lib::PathSegment line_seg(current_gear, line);

      plan_output.gear_cmd_vec.emplace_back(line_seg.seg_gear);
      plan_output.steer_vec.emplace_back(line_seg.seg_steer);
      plan_output.path_segment_vec.emplace_back(line_seg);
      plan_output.length += line_seg.Getlength();
    }

    // arc seg
    current_pose = target_pose;
    ego_heading_vec = pnc::geometry_lib::GenHeadingVec(current_pose.heading);
    heading_error = current_pose.heading - calc_params_.target_line.heading;
    line_norm_vec(ego_heading_vec.y(), -ego_heading_vec.x());
    if (line_norm_vec.x() > 0.0) {
      line_norm_vec = -1.0 * line_norm_vec;
    }
    circle_center = current_pose.pos + radius_adjust * line_norm_vec;
    arc.pA = current_pose.pos;
    arc.headingA = current_pose.heading;
    arc.circle_info.center = circle_center;
    arc.circle_info.radius = radius_adjust;
    arc.length = radius_adjust * std::fabs(heading_error);
    arc.is_anti_clockwise = calc_params_.is_left_side ? true : false;
    pnc::geometry_lib::CompleteArcInfo(arc, arc.length, arc.is_anti_clockwise);
    // TODO:cut the path if end pos is deeply into slot
    // bool trim_condition =
    //     (arc.pB.x() < 0.8 * input_.pt_0.x()) &&
    //     (std::fabs(arc.pB.y() - calc_params_.target_line.pA.y()) > 0.1);

    // if (calc_params_.is_inside_occupied && trim_condition) {
    //   double heading_low = arc.headingA;
    //   double heading_high = arc.headingB;
    //   auto pos_tmp = arc.pB;
    //   double heading_tmp = arc.headingB;
    //   const auto OA = arc.pA - arc.circle_info.center;
    //   while (pos_tmp.x() < input_.pt_0.x()) {
    //     heading_tmp = 0.5 * (heading_low + heading_high);
    //     const double heading_error = heading_tmp - heading_low;
    //     pos_tmp = arc.circle_info.center +
    //               pnc::geometry_lib::GetRotm2dFromTheta(heading_error) *
    //               OA;

    //     if (pos_tmp.x() < input_.pt_0.x()) {
    //       heading_high = heading_tmp;
    //     }
    //   }
    //   arc.pB = pos_tmp;
    //   arc.headingB = heading_tmp;
    //   arc.length =
    //       arc.circle_info.radius * std::fabs(arc.headingB - arc.headingA);

    //   std::cout << "cut arc end pos = " << arc.pB.transpose()
    //             << " cut arc end heading = " << arc.headingB * 57.3
    //             << std::endl;
    //   std::cout << "cut multi line arc path" << std::endl;
    // }
    const auto reverse_arc_steer =
        (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT)
            ? pnc::geometry_lib::SEG_STEER_LEFT
            : pnc::geometry_lib::SEG_STEER_RIGHT;

    const auto reverse_gear =
        (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE)
            ? pnc::geometry_lib::SEG_GEAR_DRIVE
            : pnc::geometry_lib::SEG_GEAR_REVERSE;

    pnc::geometry_lib::PathSegment arc_seg_2(reverse_arc_steer, reverse_gear,
                                             arc);

    // ILOG_INFO << "arc2 center = " << arc.circle_info.center.transpose()
    //                              << " arc2 end = " << arc.pB.transpose());
    // collision detection
    double safe_dist_2 = apa_param.GetParam().col_obs_safe_dist_normal;
    const PathColDetRes path_col_det_res_2 =
        TrimPathByCollisionDetection(arc_seg_2, safe_dist_2);
    if (path_col_det_res_2 == PathColDetRes::INVALID) {
      ILOG_INFO << "path col invalid";
      return false;
    }

    ILOG_INFO << "arc2 end pos = " << arc_seg_2.GetEndPos().transpose()
              << " arc2 end heading = " << arc_seg_2.GetEndHeading() * kRad2Deg;

    plan_output.gear_cmd_vec.emplace_back(arc_seg_2.seg_gear);
    plan_output.steer_vec.emplace_back(arc_seg_2.seg_steer);
    plan_output.path_segment_vec.emplace_back(arc_seg_2);
    plan_output.length += arc_seg_2.Getlength();
    current_pose = plan_output.path_segment_vec.back().GetEndPose();

    // add line seg if pose is on target_line
    if (pnc::geometry_lib::IsPoseOnLine(
            current_pose, calc_params_.target_line,
            apa_param.GetParam().target_pos_err - 1e-6,
            apa_param.GetParam().target_heading_err * kDeg2Rad - 1e-6)) {
      ILOG_INFO << "pose is on line, success";
      pnc::geometry_lib::LineSegment line;
      line.pA = current_pose.pos;
      line.pB.x() = calc_params_.target_line.pA.x();
      line.pB.y() = line.pA.y();
      line.length = (line.pB - line.pA).norm();
      line.heading = calc_params_.target_line.heading;
      pnc::geometry_lib::PathSegment line_seg(pnc::geometry_lib::SEG_GEAR_DRIVE,
                                              line);

      plan_output.gear_cmd_vec.emplace_back(line_seg.seg_gear);
      plan_output.steer_vec.emplace_back(line_seg.seg_steer);
      plan_output.path_segment_vec.emplace_back(line_seg);
      plan_output.length += line_seg.Getlength();
    }
    current_pose = plan_output.path_segment_vec.back().GetEndPose();

    if (CheckReachTargetPose(current_pose)) {
      output_.linearc_reach_target_pose = true;
      break;
    } else {
      output_.linearc_reach_target_pose = false;
    }

    if (CheckAdjustPlanSuitable(current_pose)) {
      ILOG_INFO << "swtich to adjust plan";
      break;
    }
  }

  if (plan_output.path_segment_vec.size() > 0) {
    output_.path_available = true;
    output_.length += plan_output.length;
    output_.path_segment_vec.insert(output_.path_segment_vec.end(),
                                    plan_output.path_segment_vec.begin(),
                                    plan_output.path_segment_vec.end());

    output_.gear_cmd_vec.insert(output_.gear_cmd_vec.end(),
                                plan_output.gear_cmd_vec.begin(),
                                plan_output.gear_cmd_vec.end());

    output_.steer_vec.insert(output_.steer_vec.end(),
                             plan_output.steer_vec.begin(),
                             plan_output.steer_vec.end());

    ILOG_INFO << "multi line arc plan success!";
    return true;
  }
  ILOG_INFO << "multi line arc plan failed!";
  return false;
}

// adjust plan
const bool PerpendicularPathHeadingInPlanner::OneArcPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  ILOG_INFO << "try one arc plan";
  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;
  if (pnc::geometry_lib::IsHeadingEqual(arc.headingA,
                                        calc_params_.target_line.heading) ||
      pnc::mathlib::IsDoubleEqual(arc.pA.y(),
                                  calc_params_.target_line.pA.y())) {
    ILOG_INFO << "current heading is equal to target heading or current y is "
                 "equal to "
                 "target y, no need to one arc plan";
    return false;
  }
  bool success = pnc::geometry_lib::CalOneArcWithLineAndGear(
      arc, calc_params_.target_line, current_gear);

  success = success && CheckArcOrLineAvailable(arc);

  if (success) {
    // check radius and gear can or not meet needs
    ILOG_INFO << "cal radius = " << arc.circle_info.radius;
    const auto steer = pnc::geometry_lib::CalArcSteer(arc);
    const auto gear = pnc::geometry_lib::CalArcGear(arc);
    success = (arc.circle_info.radius >= calc_params_.turn_radius - 1e-3 &&
               arc.circle_info.radius <=
                   apa_param.GetParam().max_one_step_arc_radius + 1e-3) &&
              (gear == current_gear);
    if (success) {
      ILOG_INFO << "one arc plan success";
      // ILOG_INFO << "start: coord = " << arc.pA.transpose()
      //           << "  heading = " << arc.headingA * kRad2Deg
      //           << "  end: coord = " << arc.pB.transpose()
      //           << "  heading = " << arc.headingB * kRad2Deg);
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }
  if (!success) {
    ILOG_INFO << "one arc plan fail";
  }
  return success;
}

const bool PerpendicularPathHeadingInPlanner::LineArcPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const double turn_radius) {
  ILOG_INFO << "try line arc plan";

  if (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE &&
      current_pose.pos.y() * current_pose.heading > 0.0) {
  } else if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE &&
             current_pose.pos.y() * current_pose.heading < 0.0) {
    ILOG_INFO << "should no line arc in this case";
    return false;
  } else {
    ILOG_INFO << "should no line arc in this case";
    return false;
  }

  pnc::geometry_lib::LineSegment line_seg1;
  line_seg1 = pnc::geometry_lib::BuildLineSegByPose(current_pose.pos,
                                                    current_pose.heading);

  pnc::geometry_lib::LineSegment line_seg2;
  line_seg2 = calc_params_.target_line;

  std::vector<Eigen::Vector2d> centers;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> tangent_ptss;
  if (!pnc::geometry_lib::CalCommonTangentCircleOfTwoLine(
          line_seg1, line_seg2, turn_radius, centers, tangent_ptss)) {
    ILOG_INFO << "LineArcPlan fail 0";
    return false;
  }

  bool line_arc_success = false;
  // check which center and tang pt is suitable
  for (size_t i = 0; i < centers.size(); ++i) {
    path_seg_vec.clear();
    path_seg_vec.reserve(2);

    bool line_available = false;
    pnc::geometry_lib::LineSegment tmp_line(line_seg1.pA, tangent_ptss[i].first,
                                            line_seg1.heading);
    if (CheckArcOrLineAvailable(tmp_line)) {
      if (pnc::geometry_lib::CalLineSegGear(tmp_line) == current_gear) {
        pnc::geometry_lib::PathSegment line_seg(current_gear, tmp_line);
        path_seg_vec.emplace_back(line_seg);
        line_available = true;
      }
    }

    bool arc_available = false;
    pnc::geometry_lib::Arc tmp_arc;
    tmp_arc.circle_info.center = centers[i];
    tmp_arc.pA = tangent_ptss[i].first;
    tmp_arc.headingA = line_seg1.heading;
    tmp_arc.pB = tangent_ptss[i].second;
    if (CheckArcOrLineAvailable(tmp_arc) &&
        pnc::geometry_lib::CompleteArcInfo(tmp_arc)) {
      double head_err = std::fabs(pnc::geometry_lib::NormalizeAngle(
          tmp_arc.headingB - calc_params_.target_line.heading));
      const uint8_t tmp_arc_steer = pnc::geometry_lib::CalArcSteer(tmp_arc);
      const uint8_t tmp_gear = pnc::geometry_lib::CalArcGear(tmp_arc);
      if (tmp_gear == current_gear &&
          head_err <= apa_param.GetParam().static_heading_eps * kDeg2Rad) {
        pnc::geometry_lib::PathSegment arc_seg(tmp_arc_steer, tmp_gear,
                                               tmp_arc);
        path_seg_vec.emplace_back(arc_seg);
        arc_available = true;
      }
    }

    if (path_seg_vec.size() == 2) {
      line_arc_success = true;
      break;
    }

    if (path_seg_vec.size() == 1 && line_available) {
      line_arc_success = false;
      break;
    }

    if (path_seg_vec.size() == 1 && arc_available) {
      const double dist = (current_pose.pos - tmp_arc.pA).norm();
      const double heading_err =
          std::fabs(current_pose.heading - tmp_arc.headingA);
      if (dist < apa_param.GetParam().target_pos_err - 1e-6 &&
          heading_err <
              apa_param.GetParam().target_heading_err * kDeg2Rad - 1e-6) {
        line_arc_success = true;
        break;
      }
    }
  }

  if (!line_arc_success) {
    path_seg_vec.clear();
  }

  if (path_seg_vec.empty()) {
    ILOG_INFO << "LineArcPlan fail 1";
    return false;
  }
  ILOG_INFO << "line arc success";
  return true;
}

const bool PerpendicularPathHeadingInPlanner::AlignBodyPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  ILOG_INFO << "try align body plan";
  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;
  arc.circle_info.radius = calc_params_.turn_radius;
  // check if it is necessary to align body
  if (pnc::geometry_lib::IsHeadingEqual(arc.headingA,
                                        calc_params_.target_line.heading)) {
    ILOG_INFO << "body already align";
    return false;
  }
  bool success = pnc::geometry_lib::CalOneArcWithTargetHeading(
      arc, current_gear, calc_params_.target_line.heading);

  success = success && CheckArcOrLineAvailable(arc);

  if (success) {
    // check gear can or not meet needs
    const auto steer = pnc::geometry_lib::CalArcSteer(arc);
    const auto gear = pnc::geometry_lib::CalArcGear(arc);
    success = (gear == current_gear);
    // ILOG_INFO << "steer = " << static_cast<int>(steer));
    // ILOG_INFO << "start: coord = " << arc.pA.transpose()
    //           << "  heading = " << arc.headingA * kRad2Deg
    //           << "  end: coord = " << arc.pB.transpose()
    //           << "  heading = " << arc.headingB * kRad2Deg);
    if (success) {
      ILOG_INFO << "align body plan success";
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }
  if (!success) {
    ILOG_INFO << "align body plan fail";
  }
  return success;
}

const bool PerpendicularPathHeadingInPlanner::STurnParallelPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const double steer_change_ratio1,
    const double steer_change_radius) {
  ILOG_INFO << "try s turn parallel plan";
  pnc::geometry_lib::Arc arc_s_1;
  arc_s_1.headingA = current_pose.heading;
  arc_s_1.pA = current_pose.pos;
  // check if it is possible to take S turn to target line
  if (std::fabs(pnc::geometry_lib::NormalizeAngle(
          arc_s_1.headingA - calc_params_.target_line.heading)) >
      apa_param.GetParam().static_heading_eps * kDeg2Rad) {
    ILOG_INFO << "body no align";
    return false;
  }
  arc_s_1.headingA = calc_params_.target_line.heading;
  if (std::fabs(arc_s_1.pA.y() - calc_params_.target_line.pA.y()) <
      apa_param.GetParam().static_pos_eps) {
    ILOG_INFO << "current pos is already on target line, no need to "
                 "STurnParallelPlan";
    return true;
  }

  // double slot_occupied_ratio = CalOccupiedRatio(current_pose);
  // // ILOG_INFO << "slot_occupied_ratio = " << slot_occupied_ratio);
  // const std::vector<double> ratio_tab = {0.0, 0.2, 0.5, 0.8, 1.0};
  // const double radius_change = apa_param.GetParam().headin_max_radius_in_slot
  // -
  //                              apa_param.GetParam().headin_min_radius_out_slot;
  // if (radius_change < 1e-8) {
  //   ILOG_INFO << "radius setting is err";
  //   return false;
  // }
  // const std::vector<double> radius_tab = {
  //     apa_param.GetParam().headin_min_radius_out_slot,
  //     apa_param.GetParam().headin_min_radius_out_slot + radius_change * 0.44,
  //     apa_param.GetParam().headin_min_radius_out_slot + radius_change * 0.66,
  //     apa_param.GetParam().headin_min_radius_out_slot + radius_change * 0.88,
  //     apa_param.GetParam().headin_min_radius_out_slot + radius_change};

  // double real_steer_change_radius = steer_change_radius;
  // real_steer_change_radius =
  //     pnc::mathlib::Interp1(ratio_tab, radius_tab, slot_occupied_ratio);

  // new method
  double y_error = pnc::mathlib::Limit(std::fabs(current_pose.pos.y()), 1.0);
  const std::vector<double> y_error_tab = {1.0, 0.8, 0.5, 0.2, 0.0};
  const double radius_change = apa_param.GetParam().headin_max_radius_in_slot -
                               apa_param.GetParam().headin_min_radius_out_slot;
  if (radius_change < 1e-8) {
    ILOG_INFO << "radius setting is err";
    return false;
  }

  const std::vector<double> radius_tab = {
      apa_param.GetParam().headin_min_radius_out_slot,
      apa_param.GetParam().headin_min_radius_out_slot + radius_change * 0.66,
      apa_param.GetParam().headin_min_radius_out_slot + radius_change * 0.77,
      apa_param.GetParam().headin_min_radius_out_slot + radius_change * 0.88,
      apa_param.GetParam().headin_min_radius_out_slot + radius_change};

  const auto steer_change_ratio =
      pnc::mathlib::Clamp(steer_change_ratio1, 0.1, 1.0);

  double real_steer_change_radius = steer_change_radius;
  if (y_error <= apa_param.GetParam().headin_sturn_steer_ratio_dist &&
      current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    real_steer_change_radius =
        0.25 *
        (std::pow(y_error, 2) +
         std::pow(current_pose.pos.x() - calc_params_.target_line.pA.x(), 2)) /
        y_error;
  } else {
    real_steer_change_radius =
        pnc::mathlib::Interp1(y_error_tab, radius_tab, y_error);
  }

  ILOG_INFO << "real_steer_change_radius = " << real_steer_change_radius;

  arc_s_1.circle_info.radius = real_steer_change_radius;

  // cal current line
  const auto current_line =
      pnc::geometry_lib::BuildLineSegByPose(arc_s_1.pA, arc_s_1.headingA);

  // cal target line according to steer_change_ratio
  // steer_change_ratio = 0.0 -> target_line = current_line
  // steer_change_ratio = 1.0 -> target_line = calc_params_.target_line
  pnc::geometry_lib::LineSegment target_line;
  target_line.heading = calc_params_.target_line.heading;
  target_line.SetPoints((1.0 - steer_change_ratio) * current_line.pA +
                            steer_change_ratio * calc_params_.target_line.pA,
                        (1.0 - steer_change_ratio) * current_line.pB +
                            steer_change_ratio * calc_params_.target_line.pB);

  pnc::geometry_lib::Arc arc_s_2;
  arc_s_2.circle_info.radius = real_steer_change_radius;
  arc_s_2.pB = target_line.pA;
  arc_s_2.headingB = target_line.heading;

  bool success = pnc::geometry_lib::CalTwoArcWithSameHeading(arc_s_1, arc_s_2,
                                                             current_gear);
  if (success) {
    // check gear and steer can or not meet needs
    bool arc_s_1_available = false;
    if (CheckArcOrLineAvailable(arc_s_1)) {
      const uint8_t steer_1 = pnc::geometry_lib::CalArcSteer(arc_s_1);
      const uint8_t gear_1 = pnc::geometry_lib::CalArcGear(arc_s_1);
      if (gear_1 == current_gear) {
        // ILOG_INFO << "steer_1 = " << static_cast<int>(steer_1));
        path_seg_vec.emplace_back(
            pnc::geometry_lib::PathSegment(steer_1, gear_1, arc_s_1));
        path_seg_vec.back().plan_type = pnc::geometry_lib::PLAN_TYPE_S_TURN;
        arc_s_1_available = true;
      }
    }

    bool arc_s_2_available = false;
    if (CheckArcOrLineAvailable(arc_s_2)) {
      const uint8_t steer_2 = pnc::geometry_lib::CalArcSteer(arc_s_2);
      const uint8_t gear_2 = pnc::geometry_lib::CalArcGear(arc_s_2);
      if (gear_2 == current_gear) {
        // ILOG_INFO << "steer_2 = " << static_cast<int>(steer_2));
        path_seg_vec.emplace_back(
            pnc::geometry_lib::PathSegment(steer_2, gear_2, arc_s_2));
        path_seg_vec.back().plan_type = pnc::geometry_lib::PLAN_TYPE_S_TURN;
        arc_s_2_available = true;
      }
    }

    if (path_seg_vec.empty()) {
      success = false;
    }

    if (path_seg_vec.size() == 1 && arc_s_1_available) {
      double dist = pnc::geometry_lib::CalPoint2LineDist(
          arc_s_1.pB, calc_params_.target_line);
      double head_err = std::fabs(pnc::geometry_lib::NormalizeAngle(
          arc_s_1.headingB - calc_params_.target_line.heading));
      if (dist <= apa_param.GetParam().static_pos_eps - 1e-6 &&
          head_err <=
              apa_param.GetParam().static_heading_eps * kDeg2Rad - 1e-6) {
      } else {
        path_seg_vec.clear();
        success = false;
      }
    }

    if (path_seg_vec.size() == 1 && arc_s_2_available) {
      const double dist = (arc_s_2.pA - current_pose.pos).norm();
      const double heading_err =
          std::fabs(arc_s_2.headingA - current_pose.heading);
      if (dist < apa_param.GetParam().target_pos_err - 1e-6 &&
          heading_err <
              apa_param.GetParam().target_heading_err * kDeg2Rad - 1e-6) {
        return true;
      } else {
        path_seg_vec.clear();
        return false;
      }
    }
  }

  if (!success) {
    ILOG_INFO << "s turn parallel plan fail";
  } else {
    ILOG_INFO << "s turn parallel plan success";
  }

  return success;
}

const bool PerpendicularPathHeadingInPlanner::CalSinglePathInAdjust(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t& current_gear, const double& steer_change_ratio,
    const double& steer_change_radius, const size_t& i) {
  ILOG_INFO << "-----CalSinglePathInAdjust-----";
  ILOG_INFO << "current_gear = " << static_cast<int>(current_gear)
            << ",  current_pos = " << current_pose.pos.transpose()
            << ",  current_heading = " << current_pose.heading * kRad2Deg;

  const double lat_err = apa_param.GetParam().target_pos_err;
  const double heading_err = apa_param.GetParam().target_heading_err;
  if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    apa_param.SetPram().target_pos_err = lat_err * 0.5;
    apa_param.SetPram().target_heading_err = heading_err * 0.5;
  }

  auto temp_pose = current_pose;
  temp_pose.heading = pnc::geometry_lib::NormalizeAngle(current_pose.heading);
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(5);
  // first: try one line plan to target pose
  if (!CheckReachTargetPose(temp_pose)) {
    // try line
    pnc::geometry_lib::LineSegment line_seg;
    line_seg.pA = temp_pose.pos;
    line_seg.heading = temp_pose.heading;
    if (OneLinePlan(line_seg, tmp_path_seg_vec, input_.ref_gear)) {
      temp_pose.pos = line_seg.pB;
      temp_pose.heading = line_seg.heading;
      ILOG_INFO << "OneLinePlan success";
    } else {
      ILOG_INFO << "OneLinePlan fail";
    }
  }
  bool line_arc_success = false;
  // second: try two arc(0), one arc(1) or line arc(2) plan to target line
  if (!CheckReachTargetPose(temp_pose)) {
    auto tmp_current_pose = temp_pose;
    // first try one arc to target line
    bool success = OneArcPlan(tmp_path_seg_vec, tmp_current_pose, current_gear);
    // if try one arc fail, second try align the ego body and then try go take
    // S-turn to target line
    if (success) {
      tmp_current_pose.Set(tmp_path_seg_vec.back().GetArcSeg().pB,
                           tmp_path_seg_vec.back().GetArcSeg().headingB);
    } else {
      // try line arc
      success = LineArcPlan(tmp_path_seg_vec, tmp_current_pose, current_gear,
                            calc_params_.turn_radius);
      if (success) {
        line_arc_success = true;
        tmp_current_pose.Set(tmp_path_seg_vec.back().GetEndPos(),
                             tmp_path_seg_vec.back().GetEndHeading());
      }

      success = AlignBodyPlan(tmp_path_seg_vec, tmp_current_pose, current_gear);
      // if align success
      if (success) {
        tmp_current_pose.Set(tmp_path_seg_vec.back().GetArcSeg().pB,
                             tmp_path_seg_vec.back().GetArcSeg().headingB);
      }
      // try one line plan
      pnc::geometry_lib::LineSegment line_seg;
      line_seg.pA = tmp_current_pose.pos;
      line_seg.heading = tmp_current_pose.heading;
      success = OneLinePlan(line_seg, tmp_path_seg_vec, input_.ref_gear);
      if (success) {
        tmp_current_pose.Set(line_seg.pB, line_seg.heading);
      } else {
        success =
            STurnParallelPlan(tmp_path_seg_vec, tmp_current_pose, current_gear,
                              steer_change_ratio, steer_change_radius);
      }
    }
    if (!success) {
      tmp_path_seg_vec.clear();
    }
    if (!tmp_path_seg_vec.empty()) {
      const auto& path_seg = tmp_path_seg_vec.back();
      if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
        temp_pose.pos = path_seg.GetLineSeg().pB;
        temp_pose.heading = path_seg.GetLineSeg().heading;
      } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        temp_pose.pos = path_seg.GetArcSeg().pB;
        temp_pose.heading = path_seg.GetArcSeg().headingB;
      }
    }
  }

  // third: try one line plan to target pose
  if (!CheckReachTargetPose(temp_pose)) {
    // try line
    pnc::geometry_lib::LineSegment line_seg;
    line_seg.pA = temp_pose.pos;
    line_seg.heading = temp_pose.heading;
    if (OneLinePlan(line_seg, tmp_path_seg_vec, input_.ref_gear)) {
      temp_pose.pos = line_seg.pB;
      temp_pose.heading = line_seg.heading;
      ILOG_INFO << "OneLinePlan success";
    } else {
      ILOG_INFO << "OneLinePlan fail";
    }
  }

  // avoid line arc length too length which let car go too far
  if (line_arc_success && current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    const double channel_width =
        (input_.slot_occupied_ratio < 0.368)
            ? apa_param.GetParam().channel_width
            : apa_param.GetParam().line_arc_obs_channel_width;

    const double channel_length =
        apa_param.GetParam().line_arc_obs_channel_length;

    const double pt_01_x = ((input_.pt_0 + input_.pt_1) * 0.5).x();
    const double top_x = pt_01_x + channel_width / input_.sin_angle;

    Eigen::Vector2d channel_point_1 =
        Eigen::Vector2d(top_x, 0.0) +
        (input_.pt_0 - input_.pt_1).normalized() * channel_length * 0.5;
    Eigen::Vector2d channel_point_2 =
        Eigen::Vector2d(top_x, 0.0) -
        (input_.pt_0 - input_.pt_1).normalized() * channel_length * 0.5;

    pnc::geometry_lib::LineSegment channel_line;
    channel_line.SetPoints(channel_point_1, channel_point_2);
    std::vector<Eigen::Vector2d> point_set;
    pnc::geometry_lib::SamplePointSetInLineSeg(
        point_set, channel_line, apa_param.GetParam().obstacle_ds);
    collision_detector_ptr_->AddObstacles(point_set,
                                          CollisionDetector::LINEARC_OBS);
    ILOG_INFO << "add linearc obs";
  } else {
    collision_detector_ptr_->DeleteObstacles(CollisionDetector::LINEARC_OBS);
  }

  apa_param.SetPram().target_pos_err = lat_err;
  apa_param.SetPram().target_heading_err = heading_err;

  // collision detect
  bool only_s_turn = false;
  for (size_t j = 0; j < tmp_path_seg_vec.size(); ++j) {
    pnc::geometry_lib::PathSegment& tmp_path_seg = tmp_path_seg_vec[j];
    tmp_path_seg.collision_flag = false;
    if (j == 0 &&
        tmp_path_seg.plan_type == pnc::geometry_lib::PLAN_TYPE_S_TURN) {
      only_s_turn = true;
    }
    // PrintSegmentInfo(tmp_path_seg);
    const PathColDetRes path_col_det_res =
        TrimPathByCollisionDetection(tmp_path_seg);
    if (output_.multi_reach_target_pose &&
        path_col_det_res != PathColDetRes::NORMAL) {
      // this case should no col, otherwise lose all path
      path_seg_vec.clear();
      break;
    }
    if (path_col_det_res == PathColDetRes::NORMAL) {
      path_seg_vec.emplace_back(tmp_path_seg);
    } else if (path_col_det_res == PathColDetRes::SHORTEN) {
      // when gear is drive, if there is no only s turn, then when s_turn col,
      // lose all s turn path
      if (only_s_turn) {
        path_seg_vec.emplace_back(tmp_path_seg);
      } else {
        if (tmp_path_seg.plan_type == pnc::geometry_lib::PLAN_TYPE_S_TURN &&
            j > 0) {
          ILOG_INFO << "s turn col, lose all s turn path.";
          if (tmp_path_seg_vec[j - 1].plan_type ==
              pnc::geometry_lib::PLAN_TYPE_S_TURN) {
            path_seg_vec.pop_back();
          }
        } else {
          path_seg_vec.emplace_back(tmp_path_seg);
        }
      }
      break;
    } else if (path_col_det_res == PathColDetRes::INVALID) {
      break;
    }
  }

  collision_detector_ptr_->DeleteObstacles(CollisionDetector::LINEARC_OBS);

  // postprocess path
  if (!path_seg_vec.empty()) {
    uint8_t current_gear = path_seg_vec.front().seg_gear;
    double length = 0.0;
    bool collision_flag = false;
    for (const auto& path_seg : path_seg_vec) {
      // PrintSegmentInfo(path_seg);
      if (path_seg.seg_gear == current_gear) {
        length += path_seg.Getlength();
        collision_flag = path_seg.collision_flag;
      }
    }
    if (length < apa_param.GetParam().min_gear_path_length && collision_flag) {
      ILOG_INFO << "this gear path is too small, lose it";
      path_seg_vec.clear();
    }
    ILOG_INFO << "CalSinglePathInAdjust success";
    return true;
  } else {
    ILOG_INFO << "CalSinglePathInAdjust fail";
    return false;
  }
}

const bool PerpendicularPathHeadingInPlanner::AdjustPlan() {
  ILOG_INFO << "-----adjust plan-----";
  // set init state
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;
  size_t fail_count = 0;
  if (output_.path_segment_vec.size() > 0 && output_.gear_cmd_vec.size() > 0) {
    const auto& last_seg = output_.path_segment_vec.back();
    if (last_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      current_pose.Set(last_seg.GetLineSeg().pB, last_seg.GetLineSeg().heading);
    } else if (last_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      current_pose.Set(last_seg.GetArcSeg().pB, last_seg.GetArcSeg().headingB);
    }

    current_gear =
        (output_.gear_cmd_vec.back() == pnc::geometry_lib::SEG_GEAR_REVERSE)
            ? pnc::geometry_lib::SEG_GEAR_DRIVE
            : pnc::geometry_lib::SEG_GEAR_REVERSE;

    current_arc_steer =
        (output_.steer_vec.back() == pnc::geometry_lib::SEG_STEER_RIGHT)
            ? pnc::geometry_lib::SEG_STEER_LEFT
            : pnc::geometry_lib::SEG_STEER_RIGHT;

    ILOG_INFO << "continue to plan after multi";
  }

  // check pose, if error is large, adjust is not suitable
  if (!CheckAdjustPlanSuitable(current_pose)) {
    return false;
  }

  // check gear and steer
  if ((current_gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
       current_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) ||
      (current_arc_steer != pnc::geometry_lib::SEG_STEER_LEFT &&
       current_arc_steer != pnc::geometry_lib::SEG_STEER_RIGHT)) {
    ILOG_INFO << "ref_gear or ref_arc_steer error";
    return false;
  }

  ILOG_INFO << "try adjust plan to target point";
  bool success = false;
  calc_params_.multi_plan = false;
  // double steer_change_ratio = 1.0;
  double steer_change_radius = apa_param.GetParam().headin_max_radius_in_slot;

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  for (size_t i = 0; i < kAdjustPlanMaxPathNumsInSlot; ++i) {
    ILOG_INFO << "-------- No." << i << " in adjust-plan--------";
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);

    double steer_change_ratio =
        1.0 / (std::floor(std::fabs(current_pose.pos.y()) /
                          apa_param.GetParam().headin_sturn_steer_ratio_dist) +
               1.0);

    ILOG_INFO << "steer_change_ratio = " << steer_change_ratio;
    if (CalSinglePathInAdjust(tmp_path_seg_vec, current_pose, current_gear,
                              steer_change_ratio, steer_change_radius, i)) {
      ILOG_INFO << "single path of adjust plan success!";
      output_.path_available = true;
      success = true;
    } else {
      ILOG_INFO << "single path of adjust plan failed!";
      fail_count += 1;
      if (fail_count == 1) {
      } else {
        calc_params_.adjust_fail_count += 1;
        ILOG_INFO << "adjust_fail_count = " << calc_params_.adjust_fail_count;
        if (!output_.multi_reach_target_pose &&
            calc_params_.adjust_fail_count > 4) {
          output_.Reset();
          success = false;
        }
        break;
      }
    }

    if (tmp_path_seg_vec.size() > 0) {
      for (const auto& tmp_path_seg : tmp_path_seg_vec) {
        output_.path_segment_vec.emplace_back(tmp_path_seg);
        output_.length += tmp_path_seg.Getlength();
        output_.gear_cmd_vec.emplace_back(tmp_path_seg.seg_gear);
        output_.steer_vec.emplace_back(tmp_path_seg.seg_steer);
      }
      const auto& last_segment = tmp_path_seg_vec.back();
      pnc::geometry_lib::PathPoint last_pose;
      if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        last_pose.Set(last_segment.GetArcSeg().pB,
                      last_segment.GetArcSeg().headingB);
      } else if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
        last_pose.Set(last_segment.GetLineSeg().pB,
                      last_segment.GetLineSeg().heading);
      }
      current_pose = last_pose;

      current_gear =
          (last_segment.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE)
              ? pnc::geometry_lib::SEG_GEAR_DRIVE
              : pnc::geometry_lib::SEG_GEAR_REVERSE;

      current_arc_steer =
          (last_segment.seg_steer == pnc::geometry_lib::SEG_STEER_RIGHT)
              ? pnc::geometry_lib::SEG_STEER_LEFT
              : pnc::geometry_lib::SEG_STEER_RIGHT;
    } else {
      current_gear = (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE)
                         ? pnc::geometry_lib::SEG_GEAR_DRIVE
                         : pnc::geometry_lib::SEG_GEAR_REVERSE;

      current_arc_steer =
          (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT)
              ? pnc::geometry_lib::SEG_STEER_LEFT
              : pnc::geometry_lib::SEG_STEER_RIGHT;
    }

    if (CheckReachTargetPose(current_pose)) {
      if (output_.path_segment_vec.size() > 0) {
        const auto& last_segment = output_.path_segment_vec.back();
        if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_LINE &&
            last_segment.seg_gear != pnc::geometry_lib::SEG_GEAR_DRIVE) {
          ILOG_INFO << "last line is not drive, should lose";
          output_.length -= last_segment.Getlength();
          output_.path_segment_vec.pop_back();
          output_.gear_cmd_vec.pop_back();
          output_.steer_vec.pop_back();
        }
      }
      break;
    }
  }

  if (output_.path_segment_vec.size() == 0) {
    success = false;
    output_.path_available = false;
  }

  CollisionDetector::Paramters params;
  collision_detector_ptr_->SetParam(params);

  if (!success) {
    ILOG_INFO << "adjust plan failed!";
    return false;
  }

  if (output_.path_segment_vec.size() > 0 && input_.is_replan_dynamic) {
    uint8_t ref_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    // The final reverse gear path endpoint heading and y must meet the
    // requirements, otherwise this dynamic planning will be considered a
    // failure, if meet, only the front reverse gear path is retained, and the
    // forward gear path is not required
    // for dynamic plan, all seg gear should be reverse
    for (const auto& path_seg : output_.path_segment_vec) {
      uint8_t gear = path_seg.seg_gear;
      if (gear != ref_gear) {
        // dynamic plan fail
        output_.Reset();
        return false;
      }
    }
    const auto& path_seg = output_.path_segment_vec.back();
    pnc::geometry_lib::PathPoint temp_pose;
    if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      temp_pose.pos = path_seg.GetLineSeg().pB;
      temp_pose.heading = path_seg.GetLineSeg().heading;
    } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      temp_pose.pos = path_seg.GetArcSeg().pB;
      temp_pose.heading = path_seg.GetArcSeg().headingB;
    }

    if (!CheckReachTargetPose(temp_pose)) {
      // dynamic plan fail
      output_.Reset();
      return false;
    }
    ILOG_INFO << "dynamic plan successful";
    return true;
  }
  return success;
}

// collision detection
const PerpendicularPathHeadingInPlanner::PathColDetRes
PerpendicularPathHeadingInPlanner::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg) {
  return TrimPathByCollisionDetection(
      path_seg, apa_param.GetParam().col_obs_safe_dist_normal);
}

const PerpendicularPathHeadingInPlanner::PathColDetRes
PerpendicularPathHeadingInPlanner::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg, const double safe_dist) {
  // ILOG_INFO << "--- collision detection ---";
  CollisionDetector::CollisionResult col_res;
  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    auto& line = path_seg.line_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    auto& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
  } else {
    ILOG_INFO << "no support the seg type";
    return PerpendicularPathHeadingInPlanner::PathColDetRes::INVALID;
  }

  const double remain_car_dist = col_res.remain_car_dist;
  const double remain_obs_dist = col_res.remain_obstacle_dist;
  const double safe_remain_dist =
      std::min(remain_car_dist, remain_obs_dist - safe_dist);

  ILOG_INFO << "remain_car_dist = " << remain_car_dist
            << "  remain_obs_dist = " << remain_obs_dist
            << "  safe_remain_dist = " << safe_remain_dist;

  if (safe_remain_dist < 1e-5) {
    ILOG_INFO << "safe_remain_dist is samller than 0.0, the path do not meet"
                 "requirements";
    ILOG_INFO << "col_pt_ego_global = " << col_res.col_pt_ego_global.transpose()
              << "  obs_pt_global = " << col_res.col_pt_obs_global.transpose()
              << "  car_line_order = " << col_res.car_line_order
              << "  obs_type = " << static_cast<int>(col_res.obs_type);

    // if 1R col by channel obs, even if safe_remain_dist is small. also plan
    // again
    // TODO: if headin need this part??
    // if (calc_params_.multi_plan && calc_params_.first_multi_plan &&
    //     path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE &&
    //     path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
    //     col_res.col_pt_ego_local.x() > 1.68 &&
    //     ((calc_params_.is_left_side && col_res.col_pt_ego_local.y() < 0.0) ||
    //      (!calc_params_.is_left_side &&
    //       col_res.col_pt_ego_local.y() > 0.0)) &&
    //     safe_remain_dist < 2.68) {
    //   return PathColDetRes::COMPLETE_PLAN_AGAIN;
    // }

    return PathColDetRes::INVALID;
  }

  if (remain_car_dist > safe_remain_dist + 1e-3) {
    // ILOG_INFO <<
    //     "the path will collide, the length need shorten to "
    //     "safe_remain_dist";
    ILOG_INFO << "col_pt_ego_global = " << col_res.col_pt_ego_global.transpose()
              << "  col_pt_ego_local = " << col_res.col_pt_ego_local.transpose()
              << "  obs_pt_global = " << col_res.col_pt_obs_global.transpose()
              << "  car_line_order = " << col_res.car_line_order
              << "  obs_type = " << static_cast<int>(col_res.obs_type);

    path_seg.collision_flag = true;

    // std::pair<Eigen::Vector2d, Eigen::Vector2d> slot_pt =
    //     std::make_pair(input_.pt_1, input_.pt_0);

    // CollisionDetector::ObsSlotType obs_slot_type =
    //     collision_detector_ptr_->GetObsSlotType(
    //         col_res.col_pt_obs_global, slot_pt, 5.0,
    //         calc_params_.is_left_side);

    // bool need_plan_again = false;

    // if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_INSIDE_OBS &&
    //     col_res.col_pt_obs_global.x() > input_.tlane.pt_inside.x() - 0.368) {
    //   need_plan_again = true;
    // }

    // if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_IN_OBS &&
    //     col_res.col_pt_obs_global.x() > input_.tlane.pt_inside.x() - 0.368 &&
    //     ((calc_params_.is_left_side && col_res.col_pt_obs_global.y() > 0.998)
    //     ||
    //      (!calc_params_.is_left_side &&
    //       col_res.col_pt_obs_global.y() < -0.998))) {
    //   need_plan_again = true;
    // }

    // // if 1R col by slot inner obs, plan again
    // if (need_plan_again && calc_params_.multi_plan &&
    //     path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    //   if (calc_params_.first_multi_plan) {
    //     return PathColDetRes::COMPLETE_PLAN_AGAIN;
    //   } else if (path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    //     return PathColDetRes::SINGLE_PLAN_AGAIN;
    //   }
    // }

    // // if 1R col by channel obs, plan again
    // if (calc_params_.multi_plan && calc_params_.first_multi_plan &&
    //     path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE &&
    //     path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
    //     col_res.col_pt_ego_local.x() > 1.68 &&
    //     ((calc_params_.is_left_side && col_res.col_pt_ego_local.y() < 0.0) ||
    //      (!calc_params_.is_left_side && col_res.col_pt_ego_local.y() > 0.0))
    //      &&
    //     safe_remain_dist < 2.68) {
    //   return PathColDetRes::COMPLETE_PLAN_AGAIN;
    // }

    // otherwise shorten the path to make sure safe, and lost
    if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      auto& line = path_seg.line_seg;
      if (!pnc::geometry_lib::CompleteLineInfo(line, safe_remain_dist)) {
        return PathColDetRes::INVALID;
      }
    } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      auto& arc = path_seg.arc_seg;
      if (!pnc::geometry_lib::CompleteArcInfo(arc, safe_remain_dist,
                                              arc.is_anti_clockwise)) {
        return PathColDetRes::INVALID;
      }
    }

    return PathColDetRes::SHORTEN;
  } else {
    // ILOG_INFO << "the path will not collide";
    path_seg.collision_flag = false;
    return PathColDetRes::NORMAL;
  }
}

// path postprocess
void PerpendicularPathHeadingInPlanner::InsertLineSegAfterCurrentFollowLastPath(
    double extend_distance) {
  if ((input_.is_replan_first || input_.is_replan_second) &&
      !output_.gear_shift) {
    return;
  }
  if (pnc::mathlib::IsDoubleEqual(extend_distance, 0.0)) {
    return;
  }

  if (output_.is_last_path == true &&
      output_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    ILOG_INFO << "path is last and gear is drive, not extend path";
    return;
  }

  if (output_.path_segment_vec.size() < 1) {
    return;
  }

  auto& path_seg = output_.path_segment_vec[output_.path_seg_index.second];

  double length = 0.0;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
    length += output_.path_segment_vec[i].Getlength();
  }

  if (!calc_params_.can_insert_line &&
      path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    return;
  }

  int insert_case = -1;
  // TODO: according to heading error not pos.heading
  // insert case = -1 : line seg length is ok and heading error is small, no
  // need insert;
  // insert case = 0 : small heading error & opposite gear, insert
  // line to far away from target
  // insert case = 1 : seg length is short &
  // heading error is big, insert line/arc length
  // insert case = 2 : arc seg,
  // heading error is big & length is ok, need insert
  const auto heading_error =
      std::fabs(path_seg.GetEndHeading() - calc_params_.target_line.heading) *
      kRad2Deg;

  if (pnc::mathlib::IsInBound(heading_error, -0.208, 0.208)) {
    insert_case = 0;
  } else {
    if (length > apa_param.GetParam().min_one_step_path_length - 0.016) {
      if (heading_error >
              apa_param.GetParam().headin_extend_line_min_heading_err &&
          path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        insert_case = 2;
      } else {
        ILOG_INFO << "no need insert line";
        return;
      }
    } else {
      insert_case = 1;
    }
  }

  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
      extend_distance < 0.0) {
    ILOG_INFO << "arc can not shorten";
    return;
  }

  if (extend_distance > 0.0) {
    pnc::geometry_lib::PathSegment new_line;
    new_line.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;
    new_line.seg_gear = output_.current_gear;
    new_line.seg_steer = pnc::geometry_lib::SEG_STEER_STRAIGHT;

    if (insert_case == 0) {
      double min_path_length = apa_param.GetParam().min_one_step_path_length;
      if (input_.slot_occupied_ratio > 0.086 &&
          output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
        const std::vector<double> lat_tab = {0.0, 0.05, 0.1, 0.15, 0.20};
        const std::vector<double> path_length_tab = {
            0.668 * min_path_length, 1.268 * min_path_length,
            1.868 * min_path_length, 2.468 * min_path_length,
            3.068 * min_path_length};

        min_path_length = pnc::mathlib::Interp1(
            lat_tab, path_length_tab, std::fabs(path_seg.GetEndPos().y()));

        const std::vector<double> max_x_tab = {5.086, 5.286, 5.486, 5.686,
                                               5.886};

        const double max_x = pnc::mathlib::Interp1(
            lat_tab, max_x_tab, std::fabs(path_seg.GetEndPos().y()));

        const double init_x =
            output_.path_segment_vec[output_.path_seg_index.first]
                .GetStartPos()
                .x();

        min_path_length = std::min(min_path_length, max_x - init_x);
      }

      if (length + extend_distance < min_path_length) {
        extend_distance = min_path_length - path_seg.Getlength();
      }
    } else if (insert_case == 1) {
      extend_distance = apa_param.GetParam().min_one_step_path_length - length;
    } else if (insert_case == 2) {
      extend_distance = apa_param.GetParam().insert_line_after_arc;
    }
    ILOG_INFO << "insert_case = " << insert_case;
    ILOG_INFO << "extend_distance = " << extend_distance;

    if (extend_distance < 0.02168) {
      return;
    }

    new_line.line_seg.length = extend_distance;
    if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      new_line.line_seg.pA = path_seg.line_seg.pB;
      new_line.line_seg.heading = path_seg.line_seg.heading;

    } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      new_line.line_seg.pA = path_seg.arc_seg.pB;
      new_line.line_seg.heading = path_seg.arc_seg.headingB;
    }

    Eigen::Vector2d unit_tangent = Eigen::Vector2d(
        cos(new_line.line_seg.heading), sin(new_line.line_seg.heading));

    if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
      unit_tangent *= -1.0;
    }

    Eigen::Vector2d new_line_vector = extend_distance * unit_tangent;
    new_line.line_seg.pB = new_line_vector + new_line.line_seg.pA;

    calc_params_.multi_plan = false;
    const PathColDetRes path_col_res = TrimPathByCollisionDetection(
        new_line, apa_param.GetParam().col_obs_safe_dist_strict);

    if (new_line.Getlength() < 0.02168) {
      return;
    }

    if (path_col_res == PathColDetRes::NORMAL ||
        path_col_res == PathColDetRes::SHORTEN) {
      output_.path_segment_vec.insert(
          output_.path_segment_vec.begin() + output_.path_seg_index.second + 1,
          new_line);

      output_.gear_cmd_vec.insert(
          output_.gear_cmd_vec.begin() + output_.path_seg_index.second + 1,
          output_.current_gear);

      output_.steer_vec.insert(
          output_.steer_vec.begin() + output_.path_seg_index.second + 1,
          pnc::geometry_lib::SEG_STEER_STRAIGHT);

      output_.path_seg_index.second += 1;

      ILOG_INFO << "insert line segment successful";
    } else {
      ILOG_INFO << "can not inset line segment";
    }
  }

  else {
    const auto path_seg_length = path_seg.Getlength();
    const double min_path_seg_length = 0.2;
    if (path_seg_length < min_path_seg_length) {
      return;
    }
    if (path_seg_length + extend_distance < min_path_seg_length) {
      extend_distance = min_path_seg_length - path_seg_length;
    }

    const auto path_seg_extended_length = path_seg_length + extend_distance;

    path_seg.line_seg.length = path_seg_extended_length;
    Eigen::Vector2d AB = path_seg.line_seg.pB - path_seg.line_seg.pA;
    Eigen::Vector2d AB_unit = AB.normalized();
    AB = path_seg_extended_length * AB_unit;
    path_seg.line_seg.pB = AB + path_seg.line_seg.pA;
  }
}

const bool PerpendicularPathHeadingInPlanner::GenPathOutputByDubins() {
  const auto& dubins_output = dubins_planner_.GetOutput();
  output_.path_available = true;
  output_.length += dubins_output.length;
  output_.gear_change_count += dubins_output.gear_change_count;

  // set arc AB
  if (dubins_output.gear_cmd_vec[0] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_AB = pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalArcSteer(dubins_output.arc_AB),
        dubins_output.current_gear_cmd, dubins_output.arc_AB);

    output_.steer_vec.emplace_back(
        pnc::geometry_lib::CalArcSteer(dubins_output.arc_AB));
    output_.gear_cmd_vec.emplace_back(dubins_output.current_gear_cmd);
    output_.path_segment_vec.emplace_back(seg_AB);
  }

  // seg line BC
  if (dubins_output.gear_cmd_vec[1] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_BC = pnc::geometry_lib::PathSegment(dubins_output.current_gear_cmd,
                                                 dubins_output.line_BC);

    output_.steer_vec.emplace_back(pnc::geometry_lib::SEG_STEER_STRAIGHT);
    output_.gear_cmd_vec.emplace_back(dubins_output.current_gear_cmd);
    output_.path_segment_vec.emplace_back(seg_BC);
  }

  // set arc CD
  if (dubins_output.gear_cmd_vec[2] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_CD = pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalArcSteer(dubins_output.arc_CD),
        dubins_output.current_gear_cmd, dubins_output.arc_CD);

    output_.steer_vec.emplace_back(
        pnc::geometry_lib::CalArcSteer(dubins_output.arc_CD));
    output_.gear_cmd_vec.emplace_back(dubins_output.current_gear_cmd);
    output_.path_segment_vec.emplace_back(seg_CD);
  }

  // ILOG_INFO << "prepare plan output:";
  // for (size_t i = 0; i < output_.path_segment_vec.size(); ++i) {
  //   auto& path_seg = output_.path_segment_vec[i];
  //   ILOG_INFO << i << "th path seg info:";
  //   ILOG_INFO << "type = " << static_cast<int>(path_seg.seg_type)
  //                         << "  length = " << path_seg.Getlength()
  //                         << "  gear = " <<
  //                         static_cast<int>(path_seg.seg_gear)
  //                         << "  steer = "
  //                         << static_cast<int>(path_seg.seg_steer));
  //   if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
  //     ILOG_INFO << "pA = " << path_seg.line_seg.pA.transpose() << "  pB = "
  //                         << path_seg.line_seg.pB.transpose() << "  heading
  //                         =
  //                         "
  //                         << path_seg.line_seg.heading * kRad2Deg);
  //   } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
  //     ILOG_INFO << "pA = " << path_seg.arc_seg.pA.transpose()
  //                         << "  pB = " << path_seg.arc_seg.pB.transpose()
  //                         << "  headingA = " << path_seg.arc_seg.headingA
  //                         * kRad2Deg
  //                         << "  headingB = " << path_seg.arc_seg.headingB
  //                         * kRad2Deg
  //                         << "  radius = "
  //                         << path_seg.arc_seg.circle_info.radius
  //                         << "  center = "
  //                         <<
  //                         path_seg.arc_seg.circle_info.center.transpose());
  //   }
  // }

  if (output_.gear_cmd_vec.size() > 0) {
    output_.current_gear = output_.gear_cmd_vec.front();
  }

  return output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE;
}

// some utils
const bool PerpendicularPathHeadingInPlanner::CheckReachTargetPose() {
  if (output_.path_segment_vec.empty()) {
    return CheckReachTargetPose(input_.ego_pose);
  }
  const auto& last_segment = output_.path_segment_vec.back();
  pnc::geometry_lib::PathPoint last_pose;
  if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    last_pose.Set(last_segment.GetArcSeg().pB,
                  last_segment.GetArcSeg().headingB);
  } else if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    last_pose.Set(last_segment.GetLineSeg().pB,
                  last_segment.GetLineSeg().heading);
  }

  return CheckReachTargetPose(last_pose);
}

const double PerpendicularPathHeadingInPlanner::CalOccupiedRatio(
    const pnc::geometry_lib::PathPoint& current_pose) {
  pnc::geometry_lib::PathPoint terminal_err;
  terminal_err.Set(
      current_pose.pos - calc_params_.target_line.pA,
      std::fabs(pnc::geometry_lib::NormalizeAngle(
          current_pose.heading - input_.tlane.pt_terminal_heading)));
  double slot_occupied_ratio;
  if (std::fabs(terminal_err.pos.y()) <
          apa_param.GetParam().slot_occupied_ratio_max_lat_err &&
      std::fabs(terminal_err.heading) <
          apa_param.GetParam().slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    slot_occupied_ratio = pnc::mathlib::Clamp(
        1.0 - (terminal_err.pos.x() / apa_param.GetParam().normal_slot_length),
        0.0, 1.0);
  } else {
    slot_occupied_ratio = 0.0;
  }
  return slot_occupied_ratio;
}

void PerpendicularPathHeadingInPlanner::PrintOutputSegmentsInfo() const {
  ILOG_INFO << "-------------- OutputSegmentsInfo --------------";
  const size_t N = std::min(2, int(output_.path_segment_vec.size()));
  for (size_t i = 0; i < N; i++) {
    const auto& current_seg = output_.path_segment_vec[i];

    if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      const auto& line_seg = current_seg.line_seg;

      ILOG_INFO << "Segment [" << i << "] "
                << " LINE_SEGMENT "
                << " length= " << line_seg.length;

      ILOG_INFO << "seg_gear: " << static_cast<int>(current_seg.seg_gear);

      ILOG_INFO << "seg_steer: " << static_cast<int>(current_seg.seg_steer);

      ILOG_INFO << "start_pos: " << line_seg.pA.transpose();
      ILOG_INFO << "start_heading: " << line_seg.heading * kRad2Deg;
      ILOG_INFO << "end_pos: " << line_seg.pB.transpose() << "";
    } else {
      const auto& arc_seg = current_seg.arc_seg;

      ILOG_INFO << "Segment [" << i << "] "
                << "ARC_SEGMENT "
                << "length= " << arc_seg.length;

      ILOG_INFO << "seg_gear: " << static_cast<int>(current_seg.seg_gear);

      ILOG_INFO << "seg_steer: " << static_cast<int>(current_seg.seg_steer);

      ILOG_INFO << "start_pos: " << arc_seg.pA.transpose();
      ILOG_INFO << "start_heading: " << arc_seg.headingA * kRad2Deg;
      ILOG_INFO << "end_pos: " << arc_seg.pB.transpose();
      ILOG_INFO << "end_heading: " << arc_seg.headingB * kRad2Deg;
      ILOG_INFO << "center: " << arc_seg.circle_info.center.transpose()
                << "  radius = " << arc_seg.circle_info.radius << "";
    }
  }
}

// for simulation
const bool PerpendicularPathHeadingInPlanner::PreparePlanPybind() {
  return PreparePlan();
}

const bool PerpendicularPathHeadingInPlanner::PreparePlanSecondPybind() {
  return PreparePlanSecond();
}

const bool PerpendicularPathHeadingInPlanner::GenPathOutputByDubinsPybind() {
  input_.ego_pose.pos = dubins_planner_.GetInput().p2;
  input_.ego_pose.heading = dubins_planner_.GetInput().heading2;
  return GenPathOutputByDubins();
}

const bool PerpendicularPathHeadingInPlanner::MultiPlanPybind() {
  return MultiPlan();
}

const bool PerpendicularPathHeadingInPlanner::MultiLineArcPlanPybind() {
  return MultiLineArcPlan();
}

const bool PerpendicularPathHeadingInPlanner::AdjustPlanPybind() {
  return AdjustPlan();
}

const PerpendicularPathHeadingInPlanner::PlannerParams&
PerpendicularPathHeadingInPlanner::GetCalcParams() {
  return calc_params_;
}

const bool PerpendicularPathHeadingInPlanner::CheckReachTargetPosePybind() {
  return CheckReachTargetPose();
}
}  // namespace apa_planner
}  // namespace planning
