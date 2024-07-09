#include "perpendicular_path_planner.h"

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
#include "apa_plan_base.h"
#include "apa_world.h"
#include "collision_detection.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "math_lib.h"
#include "planning_plan_c.h"
namespace planning {
namespace apa_planner {

static const size_t kMaxPerpenParkInSegmentNums = 15;
static const size_t kReservedOutputPathPointSize = 750;
static const int kMultiPlanMaxPathNumsInSlot = 5;
static const size_t kAdjustPlanMaxPathNumsInSlot = 5;
static const double kMinSingleGearPathLength = 0.4;

void PerpendicularPathPlanner::Reset() {
  output_.Reset();
  output_.path_segment_vec.reserve(kMaxPerpenParkInSegmentNums);
  output_.gear_cmd_vec.reserve(kMaxPerpenParkInSegmentNums);

  calc_params_.Reset();
}

void PerpendicularPathPlanner::Preprocess() {
  // calc_params_.Reset();
  calc_params_.stuck_by_inside = false;
  calc_params_.multi_plan = false;
  calc_params_.directly_use_ego_pose = false;
  calc_params_.turn_radius = 1.0 * apa_param.GetParam().min_turn_radius;
  calc_params_.can_insert_line = true;

  // calc slot side by Tlane
  if (input_.tlane.pt_inside.y() > input_.tlane.pt_outside.y()) {
    calc_params_.is_left_side = true;
    calc_params_.slot_side_sgn = 1.0;
  } else {
    calc_params_.is_left_side = false;
    calc_params_.slot_side_sgn = -1.0;
  }

  // reset output
  output_.Reset();

  // target line
  calc_params_.target_line = pnc::geometry_lib::BuildLineSegByPose(
      input_.tlane.pt_terminal_pos, input_.tlane.pt_terminal_heading);
}

bool PerpendicularPathPlanner::Update() {
  DEBUG_PRINT("-------- path planner --------");

  // preprocess
  Preprocess();

  // reset output
  output_.Reset();

  // prepare plan, only for first plan
  if (input_.is_replan_first) {
    calc_params_.first_multi_plan = true;
    if (PreparePlan()) {
      if (calc_params_.directly_use_ego_pose) {
        DEBUG_PRINT(
            "ego pose is close to safe_circle_tang_pt, directly use"
            "ego pose to multi plan, no second prepare!");
        calc_params_.should_prepare_second = false;
        output_.gear_shift = true;
      } else if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
        DEBUG_PRINT(
            "first prepare success, the gear is reverse, use "
            "safe_circle_tang_pt to multi plan, no second prepare!");
        calc_params_.should_prepare_second = false;
        output_.gear_shift = true;
      } else if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
        DEBUG_PRINT(
            "first prepare success, the gear is drive, no need multi "
            "plan, need second prepare!");
        calc_params_.should_prepare_second = true;
        output_.gear_shift = false;
        return true;
      } else {
        DEBUG_PRINT("except err");
        return false;
      }
    } else {
      DEBUG_PRINT("prepare plan fail, quit plan!");
      return false;
    }
  }

  if (input_.is_replan_second) {
    if (calc_params_.should_prepare_second) {
      DEBUG_PRINT("should try second prepare!");
      if (PreparePlanSecond()) {
        if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
          DEBUG_PRINT("second prepare gear is reverse, need multi plan!");
          output_.gear_shift = true;
        } else if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
          DEBUG_PRINT("second prepare gear is drive, no need multi plan!");
          return true;
        } else {
          DEBUG_PRINT("except err");
          return false;
        }
      } else {
        DEBUG_PRINT("directly use ego pose to multi plan!");
        output_.gear_shift = true;
      }
    } else {
      DEBUG_PRINT(
          "no need second prepare, directly use ego pose to multi plan");
      output_.gear_shift = true;
    }
    calc_params_.should_prepare_second = false;
  }

  // multi step
  if (MultiPlan()) {
    if (calc_params_.first_multi_plan) {
      output_.is_first_reverse_path = true;
    }
    calc_params_.first_multi_plan = false;
    DEBUG_PRINT("multi plan success!");
  }

  if (CheckReachTargetPose()) {
    DEBUG_PRINT("multi plan is already to target pos!");
    return true;
  }

  // adjust step
  if (AdjustPlan()) {
    DEBUG_PRINT("adjust plan success!");
    return true;
  }
  if (output_.multi_reach_target_pose) {
    DEBUG_PRINT("multi plan is already to target pos!");
    return true;
  }
  if (output_.path_segment_vec.size() == 0) {
    DEBUG_PRINT("no path, plan fail");
    output_.Reset();
    return false;
  } else {
    DEBUG_PRINT(
        "there are path, through no plan to target pose, but let car go.");
    return true;
  }
}

bool PerpendicularPathPlanner::Update(
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  collision_detector_ptr_ = collision_detector_ptr;
  return Update();
}

bool PerpendicularPathPlanner::UpdateByPrePlan() {
  Preprocess();

  return PreparePlan() || calc_params_.directly_use_ego_pose;
}

// prepare plan start
const bool PerpendicularPathPlanner::PreparePlan() {
  std::vector<double> x_offset_vec;
  std::vector<double> heading_offset_vec;

  const double pt_01_x = ((input_.pt_0 + input_.pt_1) * 0.5).x();
  const double x_min =
      pt_01_x +
      apa_param.GetParam().prepare_line_min_x_offset_slot / input_.sin_angle;
  const double x_max =
      pt_01_x +
      apa_param.GetParam().prepare_line_max_x_offset_slot / input_.sin_angle;

  double x_offset = x_min;
  while (x_offset < x_max) {
    x_offset_vec.emplace_back(x_offset);
    x_offset += apa_param.GetParam().prepare_line_dx_offset_slot;
  }

  double heading_offset =
      apa_param.GetParam().prepare_line_max_heading_offset_slot_deg / 57.3;
  while (heading_offset >=
         apa_param.GetParam().prepare_line_min_heading_offset_slot_deg / 57.3) {
    heading_offset_vec.emplace_back(heading_offset);
    heading_offset -=
        apa_param.GetParam().prepare_line_dheading_offset_slot_deg / 57.3;
  }

  bool flag = false;
  bool prepare_success = false;

  for (size_t i = 0; i < heading_offset_vec.size() && !flag; ++i) {
    const double& heading_offset = heading_offset_vec[i];
    for (size_t j = 0; j < x_offset_vec.size() && !flag; ++j) {
      const double& x_offset = x_offset_vec[j];
      if (PreparePlanOnce(x_offset, heading_offset, calc_params_.turn_radius)) {
        prepare_success = true;
        DEBUG_PRINT("  x_offset = " << x_offset << "  heading_offset = "
                                    << heading_offset * 57.3);
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
                                    .prepare_directly_use_tangent_heading_err /
                                57.3) {
            calc_params_.directly_use_ego_pose = true;
          }
        }
      }
      flag = prepare_success || calc_params_.directly_use_ego_pose;
    }
  }

  calc_params_.pt_inside = input_.tlane.pt_inside;

  if (!flag) {
    DEBUG_PRINT("prepare first fail");
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
    DEBUG_PRINT("first try dubins success");
    input_.ego_pose = calc_params_.safe_circle_tang_pt;
  }

  if (calc_params_.directly_use_ego_pose) {
    // directly use ego pose to multi plan
    DEBUG_PRINT("directly use ego pose to multi plan");
  }

  calc_params_.first_path_gear = output_.current_gear;

  return true;
}

const bool PerpendicularPathPlanner::PreparePlanOnce(
    const double& x_offset, const double& heading_offset,
    const double& radius) {
  const double start_heading =
      calc_params_.slot_side_sgn *
      ((90.0 - input_.origin_pt_0_heading) / 57.3 - heading_offset);

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

  calc_params_.pre_line_tangent_vec = line_tangent_vec;
  calc_params_.pre_line_normal_vec = line_normal_vec;

  // find a target point on prepare line
  pnc::geometry_lib::PathPoint target_pose;
  target_pose.heading = calc_params_.prepare_line.heading;

  pnc::dubins_lib::DubinsLibrary::Input input;
  input.radius = radius;

  calc_params_.cal_tang_pt_success = false;
  calc_params_.use_mono_tang = false;
  calc_params_.use_multi_tang = false;
  bool prepare_success = false;
  while (!prepare_success) {
    // first use mono prepare to find target point
    if (apa_param.GetParam().mono_plan_enable &&
        MonoPreparePlan(target_pose.pos)) {
      input.Set(input_.ego_pose.pos, target_pose.pos, input_.ego_pose.heading,
                target_pose.heading);
      dubins_planner_.SetInput(input);
      prepare_success =
          dubins_planner_.OneStepDubinsUpdateByVer(kMinSingleGearPathLength);
      calc_params_.cal_tang_pt_success = true;
      calc_params_.safe_circle_tang_pt = target_pose;
    }
    if (prepare_success) {
      // DEBUG_PRINT("use mono prepare to find target point successful!");
      calc_params_.use_mono_tang = true;
      break;
    }
    // if mono prepare fail, use multi prepare to find target point
    if (MultiPreparePlan(target_pose.pos)) {
      input.Set(input_.ego_pose.pos, target_pose.pos, input_.ego_pose.heading,
                target_pose.heading);
      dubins_planner_.SetInput(input);
      prepare_success =
          dubins_planner_.OneStepDubinsUpdateByVer(kMinSingleGearPathLength);
      calc_params_.cal_tang_pt_success = true;
      calc_params_.safe_circle_tang_pt = target_pose;
    }
    if (prepare_success) {
      // DEBUG_PRINT("use multi prepare to find target point successful!");
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
    // set virtual arc DE that make sure multi plan success
    bool use_virtual_arc = false;
    if (dubins_planner_.GetOutput().current_gear_cmd ==
        pnc::geometry_lib::SEG_GEAR_DRIVE) {
      use_virtual_arc = true;
      pnc::geometry_lib::Arc arc_DE;
      arc_DE.pA = target_pose.pos;
      arc_DE.headingA = target_pose.heading;
      arc_DE.length = 3.068;
      // cal pre line tangent vec and normal vec
      const Eigen::Vector2d line_tangent_vec =
          pnc::geometry_lib::GenHeadingVec(target_pose.heading);
      Eigen::Vector2d line_normal_vec(line_tangent_vec.y(),
                                      -line_tangent_vec.x());
      // sure line_normal_vec towards downward along the x axis.
      if (line_normal_vec.x() > 0.0) {
        line_normal_vec = -1.0 * line_normal_vec;
      }
      arc_DE.circle_info.radius = calc_params_.turn_radius;
      arc_DE.circle_info.center =
          arc_DE.pA + arc_DE.circle_info.radius * line_normal_vec;
      arc_DE.is_anti_clockwise = calc_params_.is_left_side ? false : true;
      pnc::geometry_lib::CompleteArcInfo(arc_DE, arc_DE.length,
                                         arc_DE.is_anti_clockwise);
      path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
      path_seg.arc_seg = arc_DE;
      path_seg_vec.emplace_back(path_seg);
    }
    if (use_virtual_arc) {
      CollisionDetector::Paramters param;
      param.lat_inflation = apa_param.GetParam().car_lat_inflation_strict;
      collision_detector_ptr_->SetParam(param);
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
        prepare_success = false;
        break;
      }
    }
    if (use_virtual_arc) {
      CollisionDetector::Paramters param;
      collision_detector_ptr_->SetParam(param);
    }
  }
  if (prepare_success) {
    DEBUG_PRINT("use_mono_tang = " << calc_params_.use_mono_tang
                                   << "  use_multi_tang = "
                                   << calc_params_.use_multi_tang);
  }
  return prepare_success;
}

const bool PerpendicularPathPlanner::PreparePlanSecond() {
  // try directly multi
  double dist =
      (input_.ego_pose.pos - calc_params_.safe_circle_tang_pt.pos).norm();
  double heading_err = std::fabs(input_.ego_pose.heading -
                                 calc_params_.safe_circle_tang_pt.heading);

  if (dist < apa_param.GetParam().prepare_directly_use_tangent_pos_err &&
      heading_err <
          apa_param.GetParam().prepare_directly_use_tangent_heading_err /
              57.3) {
    DEBUG_PRINT("use ego pose to multi plan when prepare second");
    return false;
  }

  double min_dubins_path = kMinSingleGearPathLength;
  if (calc_params_.first_path_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    // current dubins gear must be reverse, and the path is continue to multi or
    // adjust plan so no length need for it
    min_dubins_path = 0.086;
  }

  // try dubins
  bool dubins_success = false;
  pnc::dubins_lib::DubinsLibrary::Input input;
  input.Set(input_.ego_pose.pos, calc_params_.safe_circle_tang_pt.pos,
            input_.ego_pose.heading, calc_params_.safe_circle_tang_pt.heading);
  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;

  input.radius = calc_params_.turn_radius;
  dubins_planner_.SetInput(input);
  path_seg_vec.clear();
  path_seg_vec.reserve(3);

  if (dubins_planner_.OneStepDubinsUpdateByVer(min_dubins_path) &&
      dubins_planner_.GetOutput().current_gear_cmd !=
          calc_params_.first_path_gear) {
    // should col det
    std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);
    pnc::geometry_lib::PathSegment path_seg;
    // set arc AB
    if (dubins_planner_.GetOutput().gear_cmd_vec[0] !=
        pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
      path_seg.arc_seg = dubins_planner_.GetOutput().arc_AB;
      path_seg.seg_gear = dubins_planner_.GetOutput().gear_cmd_vec[0];
      path_seg.seg_steer = pnc::geometry_lib::CalArcSteer(path_seg.arc_seg);
      tmp_path_seg_vec.emplace_back(path_seg);
    }
    // set line BC
    if (dubins_planner_.GetOutput().gear_cmd_vec[1] !=
        pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;
      path_seg.line_seg = dubins_planner_.GetOutput().line_BC;
      path_seg.seg_gear = dubins_planner_.GetOutput().gear_cmd_vec[1];
      path_seg.seg_steer = pnc::geometry_lib::SEG_STEER_STRAIGHT;
      tmp_path_seg_vec.emplace_back(path_seg);
    }
    // set arc CD
    if (dubins_planner_.GetOutput().gear_cmd_vec[2] !=
        pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
      path_seg.arc_seg = dubins_planner_.GetOutput().arc_CD;
      path_seg.seg_gear = dubins_planner_.GetOutput().gear_cmd_vec[2];
      path_seg.seg_steer = pnc::geometry_lib::CalArcSteer(path_seg.arc_seg);
      tmp_path_seg_vec.emplace_back(path_seg);
    }
    // collision detect
    for (pnc::geometry_lib::PathSegment& tmp_path_seg : tmp_path_seg_vec) {
      const uint8_t path_col_det_res =
          TrimPathByCollisionDetection(tmp_path_seg);
      if (path_col_det_res == PATH_COL_NORMAL) {
        path_seg_vec.emplace_back(tmp_path_seg);
      } else if (path_col_det_res == PATH_COL_SHORTEN) {
        path_seg_vec.emplace_back(tmp_path_seg);
        break;
      } else if (path_col_det_res == PATH_COL_INVALID) {
        break;
      }
    }
    if (path_seg_vec.size() > 0) {
      dubins_success = true;
    }
  }

  if (dubins_success) {
    output_.path_available = true;
    output_.current_gear = dubins_planner_.GetOutput().current_gear_cmd;
    for (const pnc::geometry_lib::PathSegment& path_seg : path_seg_vec) {
      output_.path_segment_vec.emplace_back(path_seg);
      output_.length += path_seg.Getlength();
      output_.gear_cmd_vec.emplace_back(path_seg.seg_gear);
      output_.steer_vec.emplace_back(path_seg.seg_steer);
    }
    input_.ego_pose = path_seg_vec.back().GetEndPose();
  }
  return dubins_success;
}

void PerpendicularPathPlanner::CalMonoSafeCircle() {
  calc_params_.mono_safe_circle.center.y() =
      calc_params_.target_line.pA.y() +
      calc_params_.turn_radius * calc_params_.slot_side_sgn;

  calc_params_.mono_safe_circle.radius = calc_params_.turn_radius;

  auto pt_inside = input_.tlane.pt_inside;

  pt_inside.x() = std::max(pt_inside.x(),
                           input_.tlane.corner_inside_slot.x() -
                               apa_param.GetParam().max_pt_inside_drop_dx_mono);

  const double deta_x = std::sqrt(
      std::pow(
          (calc_params_.turn_radius - apa_param.GetParam().car_width * 0.5 -
           apa_param.GetParam().car_lat_inflation_normal - 0.0268),
          2) -
      std::pow((calc_params_.mono_safe_circle.center.y() - pt_inside.y()), 2));

  calc_params_.mono_safe_circle.center.x() = pt_inside.x() - deta_x;

  // DEBUG_PRINT("mono safe circle info: center = "
  //           << calc_params_.mono_safe_circle.center.transpose()
  //           << "   radius = " << calc_params_.mono_safe_circle.radius
  //          );
}

const bool PerpendicularPathPlanner::CheckMonoIsFeasible() {
  const double dist = CalPoint2LineDist(calc_params_.mono_safe_circle.center,
                                        calc_params_.prepare_line);
  if (dist >= calc_params_.mono_safe_circle.radius) {
    // DEBUG_PRINT("prepare_line is tangential or disjoint from mono safe "
    //              "circle, mono is feasible!");
    return true;
  } else {
    // DEBUG_PRINT("prepare_line intersects circle, mono is not feasible");
    return false;
  }
}

const bool PerpendicularPathPlanner::MonoPreparePlan(
    Eigen::Vector2d& tag_point) {
  CalMonoSafeCircle();

  if (CheckMonoIsFeasible() == false) {
    // DEBUG_PRINT("cal monostep safe circle fail!");
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
  // DEBUG_PRINT("point_tangent = " << tag_point.transpose());
  return true;
}

bool PerpendicularPathPlanner::CalMultiSafeCircle() {
  auto pt_inside = input_.tlane.pt_inside;
  pt_inside.x() = std::max(
      pt_inside.x(), input_.tlane.corner_inside_slot.x() -
                         apa_param.GetParam().max_pt_inside_drop_dx_multi);

  pnc::geometry_lib::Circle circle_p1;
  circle_p1.center = pt_inside;
  circle_p1.radius = calc_params_.turn_radius -
                     0.5 * apa_param.GetParam().car_width -
                     apa_param.GetParam().car_lat_inflation_normal - 0.0268;

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

  // DEBUG_PRINT("multi safa circle info: center = " <<
  // multi_safe_circle.center
  //           << "  radius = " << multi_safe_circle.radius);

  return true;
}

const bool PerpendicularPathPlanner::MultiPreparePlan(
    Eigen::Vector2d& tag_point) {
  if (CalMultiSafeCircle() == false) {
    // DEBUG_PRINT("cal multistep safe circle fail!");
    return false;
  }

  tag_point =
      calc_params_.multi_safe_circle.center -
      calc_params_.pre_line_normal_vec * calc_params_.multi_safe_circle.radius;

  // DEBUG_PRINT("point_tangent = " << tag_point.transpose());
  return true;
}

const bool PerpendicularPathPlanner::GenPathOutputByDubins() {
  const auto& dubins_output = dubins_planner_.GetOutput();
  output_.path_available = true;
  output_.length = dubins_output.length;
  output_.gear_change_count = dubins_output.gear_change_count;
  output_.current_gear = dubins_output.current_gear_cmd;

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

  // DEBUG_PRINT("prepare plan output:");
  // for (size_t i = 0; i < output_.path_segment_vec.size(); ++i) {
  //   auto& path_seg = output_.path_segment_vec[i];
  //   DEBUG_PRINT(i << "th path seg info:");
  //   DEBUG_PRINT("type = " << static_cast<int>(path_seg.seg_type)
  //                         << "  length = " << path_seg.Getlength()
  //                         << "  gear = " <<
  //                         static_cast<int>(path_seg.seg_gear)
  //                         << "  steer = "
  //                         << static_cast<int>(path_seg.seg_steer));
  //   if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
  //     DEBUG_PRINT("pA = " << path_seg.line_seg.pA.transpose() << "  pB = "
  //                         << path_seg.line_seg.pB.transpose() << "  heading =
  //                         "
  //                         << path_seg.line_seg.heading * 57.3);
  //   } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
  //     DEBUG_PRINT("pA = " << path_seg.arc_seg.pA.transpose()
  //                         << "  pB = " << path_seg.arc_seg.pB.transpose()
  //                         << "  headingA = " << path_seg.arc_seg.headingA
  //                         * 57.3
  //                         << "  headingB = " << path_seg.arc_seg.headingB
  //                         * 57.3
  //                         << "  radius = "
  //                         << path_seg.arc_seg.circle_info.radius
  //                         << "  center = "
  //                         <<
  //                         path_seg.arc_seg.circle_info.center.transpose());
  //   }
  // }

  return output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE;
}
// prepare plan end

// checkout multi suitable
const bool PerpendicularPathPlanner::CheckMultiPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose,
    const double& slot_occupied_ratio) {
  if ((std::fabs(current_pose.pos.y()) <=
           apa_param.GetParam().multi_plan_min_lat_err &&
       std::fabs(current_pose.heading) <=
           apa_param.GetParam().multi_plan_min_heading_err / 57.3) ||
      slot_occupied_ratio >=
          apa_param.GetParam().multi_plan_max_occupied_ratio) {
    DEBUG_PRINT("pose err is relatively small, multi plan is not suitable");
    return false;
  }
  return true;
}

// multi plan start
const bool PerpendicularPathPlanner::MultiPlan() {
  printf("-----multi plan-----\n");
  // set init state
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;

  // check pose and slot_occupied_ratio, if error is small, multi isnot
  // suitable
  if (!CheckMultiPlanSuitable(current_pose, input_.slot_occupied_ratio)) {
    return false;
  }

  // check gear and steer
  if ((current_gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
       current_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) ||
      (current_arc_steer != pnc::geometry_lib::SEG_STEER_LEFT &&
       current_arc_steer != pnc::geometry_lib::SEG_STEER_RIGHT)) {
    DEBUG_PRINT("ref_gear or ref_arc_steer error");
    return false;
  }

  Output multi_out_put;
  DEBUG_PRINT("try multi plan to target point");
  bool success = false;
  calc_params_.multi_plan = true;
  double turn_radius = calc_params_.turn_radius;
  size_t stuck_by_inside_count = 0;
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  for (int i = 0; i < kMultiPlanMaxPathNumsInSlot; ++i) {
    DEBUG_PRINT("-- No." << i << "  multi-plan --");
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);
    if (CalSinglePathInMulti(current_pose, current_gear, current_arc_steer,
                             tmp_path_seg_vec, turn_radius, i)) {
      multi_out_put.path_available = true;
      success = true;
    } else {
      DEBUG_PRINT("single path of multi-plan failed!");
      if (calc_params_.stuck_by_inside) {
        i = -1;
        stuck_by_inside_count++;
        if (calc_params_.first_multi_plan ||
            current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
          DEBUG_PRINT(
              "reverse path stuck by inside, use reverse line and arc "
              "to far from inside.");
          if (stuck_by_inside_count > 10) {
            std::cout << "try reverse line and arc enough, but also failed\n";
            success = false;
            break;
          }
          if (calc_params_.first_multi_plan) {
            DEBUG_PRINT("first multi plan, clear all path.");
            multi_out_put.Reset();
          }
          double compensate_line_length = 0.15;
          if (multi_out_put.path_segment_vec.empty()) {
            current_pose = input_.ego_pose;
            current_gear = input_.ref_gear;
            current_arc_steer = input_.ref_arc_steer;
            compensate_line_length *= stuck_by_inside_count;
          } else {
            current_pose = multi_out_put.path_segment_vec.back().GetEndPose();
          }

          pnc::geometry_lib::PathPoint target_pose = current_pose;

          Eigen::Vector2d ego_heading_vec =
              pnc::geometry_lib::GenHeadingVec(current_pose.heading);

          target_pose.pos =
              current_pose.pos - compensate_line_length * ego_heading_vec;

          pnc::geometry_lib::LineSegment line(current_pose.pos, target_pose.pos,
                                              current_pose.heading);

          CollisionDetector::CollisionResult col_res =
              collision_detector_ptr_->UpdateByObsMap(line, line.heading);

          const double safe_remain_dist =
              std::min(col_res.remain_car_dist,
                       col_res.remain_obstacle_dist -
                           apa_param.GetParam().col_obs_safe_dist_normal);

          if (col_res.remain_car_dist - 1e-3 > safe_remain_dist ||
              safe_remain_dist < 1e-5) {
            DEBUG_PRINT("line will collide, fail and quit multi plan");
            success = false;
            break;
          }

          current_pose = target_pose;

          multi_out_put.path_available = true;
          pnc::geometry_lib::PathSegment line_seg(
              pnc::geometry_lib::SEG_GEAR_REVERSE, line);
          multi_out_put.gear_cmd_vec.emplace_back(line_seg.seg_gear);
          multi_out_put.steer_vec.emplace_back(line_seg.seg_steer);
          multi_out_put.path_segment_vec.emplace_back(line_seg);
          multi_out_put.length += line_seg.Getlength();

          continue;
        } else if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
          DEBUG_PRINT("save current path and continue to plan.");
        }
      } else {
        // Determine if there are paths that have been stored. If not, it
        // will fail. If there are, save paths and use adjust to
        // continue planning
        if (multi_out_put.path_segment_vec.size() > 0) {
          multi_out_put.path_available = true;
          success = true;
        } else {
          multi_out_put.path_available = false;
          success = false;
        }
        break;
      }
    }

    if (tmp_path_seg_vec.size() > 0) {
      for (const auto& tmp_path_seg : tmp_path_seg_vec) {
        multi_out_put.path_segment_vec.emplace_back(tmp_path_seg);
        multi_out_put.length += tmp_path_seg.Getlength();
        multi_out_put.gear_cmd_vec.emplace_back(tmp_path_seg.seg_gear);
        multi_out_put.steer_vec.emplace_back(tmp_path_seg.seg_steer);
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
      break;
    }
  }

  // if multi path current gear is not correct, lost all path and use adjust to
  // plan
  if (multi_out_put.path_segment_vec.size() > 0 &&
      multi_out_put.gear_cmd_vec.front() != input_.ref_gear) {
    multi_out_put.Reset();
    success = false;
  }

  if (multi_out_put.path_segment_vec.size() > 0) {
    success = true;

    if (multi_out_put.gear_cmd_vec.size() > 0 &&
        multi_out_put.gear_cmd_vec[0] == pnc::geometry_lib::SEG_GEAR_REVERSE) {
      int path_num_gear = 0;
      for (size_t i = 0; i < multi_out_put.gear_cmd_vec.size(); ++i) {
        if (multi_out_put.gear_cmd_vec[i] == multi_out_put.gear_cmd_vec[0]) {
          path_num_gear++;
        } else {
          break;
        }
      }

      if (CheckReachTargetPose(
              multi_out_put.path_segment_vec.back().GetEndPose())) {
        DEBUG_PRINT(
            "multi already plan to target pos, look it can lose drive path and "
            "continue to use reverse gear adjust plan to reduce change count!");
        output_.multi_reach_target_pose = true;
      } else {
        DEBUG_PRINT(
            "multi can not plan to target pos, look it can lose drive path and "
            "continue to use reverse gear adjust plan to reduce change count!");
        output_.multi_reach_target_pose = false;
      }
      // only to onvenient for simulation use
      bool continue_to_adjust = true;
      if (input_.is_simulation) {
        continue_to_adjust = !output_.multi_reach_target_pose;
      }
      if (continue_to_adjust &&
          path_num_gear < static_cast<int>(multi_out_put.gear_cmd_vec.size()) &&
          CheckAdjustPlanSuitable(
              multi_out_put.path_segment_vec[path_num_gear - 1].GetEndPose())) {
        DEBUG_PRINT("lose drive path, continue use reverse gear adjust plan");
        for (int i = static_cast<int>(multi_out_put.gear_cmd_vec.size() - 1);
             i >= path_num_gear; --i) {
          multi_out_put.length -= multi_out_put.path_segment_vec[i].Getlength();
          multi_out_put.gear_cmd_vec.pop_back();
          multi_out_put.path_segment_vec.pop_back();
          multi_out_put.steer_vec.pop_back();
        }
      }
    }

    output_.path_available = true;
    output_.length += multi_out_put.length;
    output_.path_segment_vec.insert(output_.path_segment_vec.end(),
                                    multi_out_put.path_segment_vec.begin(),
                                    multi_out_put.path_segment_vec.end());
    output_.gear_cmd_vec.insert(output_.gear_cmd_vec.end(),
                                multi_out_put.gear_cmd_vec.begin(),
                                multi_out_put.gear_cmd_vec.end());
    output_.steer_vec.insert(output_.steer_vec.end(),
                             multi_out_put.steer_vec.begin(),
                             multi_out_put.steer_vec.end());
  }

  CollisionDetector::Paramters params;
  collision_detector_ptr_->SetParam(params);

  if (!success) {
    DEBUG_PRINT("multi plan failed!");
  }
  return success;
}

const bool PerpendicularPathPlanner::CalSinglePathInMulti(
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const uint8_t current_arc_steer,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const double turn_radius, size_t i) {
  DEBUG_PRINT("-----CalSinglePathInMulti-----");
  DEBUG_PRINT("current_arc_steer = "
              << static_cast<int>(current_arc_steer)
              << ",  current_gear = " << static_cast<int>(current_gear)
              << ",  current_pos = " << current_pose.pos.transpose()
              << ",  current_heading = " << current_pose.heading * 57.3);

  const double lat_err = apa_param.GetParam().target_pos_err;
  const double heading_err = apa_param.GetParam().target_heading_err;
  if (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
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
      DEBUG_PRINT("OneLinePlan success");
    } else {
      DEBUG_PRINT("OneLinePlan fail");
    }
  }

  // second: try two arc(0), one arc(1) or line arc(2) plan to target line
  int type = -1;
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
    const double dist = pnc::geometry_lib::CalPoint2LineDist(
        current_turn_center, calc_params_.target_line);

    // two arc
    if (dist < current_turn_radius - apa_param.GetParam().target_radius_err) {
      DEBUG_PRINT("center to line dist = " << dist << ",  try TwoArcPlan");
      if (TwoArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                     current_arc_steer)) {
        DEBUG_PRINT("TwoArcPlan success");
        type = 0;
      } else {
        DEBUG_PRINT("TwoArcPlan fail");
      }
    }
    // one arc
    else if ((dist >=
              current_turn_radius - apa_param.GetParam().target_radius_err) &&
             (dist <=
              current_turn_radius + apa_param.GetParam().target_radius_err)) {
      DEBUG_PRINT("center to line dist = " << dist << ",  try OneArcPlan");
      if (OneArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                     current_arc_steer)) {
        DEBUG_PRINT("OneArcPlan success");
        type = 1;
      } else {
        DEBUG_PRINT("OneArcPlan fail");
      }
    }
    // lin arc
    else if (dist >
             current_turn_radius + apa_param.GetParam().target_radius_err) {
      if (LineArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                      current_arc_steer)) {
        DEBUG_PRINT("LineArcPlan success");
        type = 2;
      } else {
        DEBUG_PRINT("LineArcPlan fail");
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
      DEBUG_PRINT("OneLinePlan success");
    } else {
      DEBUG_PRINT("OneLinePlan fail");
    }
  }

  // avoid line arc length too length whicl let car go too far
  if (type == 2 && current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
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
    DEBUG_PRINT("add linearc obs");
  } else {
    collision_detector_ptr_->DeleteObstacles(CollisionDetector::LINEARC_OBS);
  }

  apa_param.SetPram().target_pos_err = lat_err;
  apa_param.SetPram().target_heading_err = heading_err;

  for (pnc::geometry_lib::PathSegment& tmp_path_seg : tmp_path_seg_vec) {
    double safe_dist = apa_param.GetParam().col_obs_safe_dist_normal;
    CollisionDetector::Paramters params;
    params.lat_inflation = apa_param.GetParam().car_lat_inflation_normal;
    if (calc_params_.first_multi_plan && type == 0 && i == 0 &&
        tmp_path_seg.seg_gear == current_gear) {
      // when 1R and two arc, should far from inside obs
      params.lat_inflation = apa_param.GetParam().car_lat_inflation_strict;
      safe_dist = apa_param.GetParam().col_obs_safe_dist_strict;
    }
    collision_detector_ptr_->SetParam(params);
    // PrintSegmentInfo(tmp_path_seg);
    const uint8_t path_col_det_res =
        TrimPathByCollisionDetection(tmp_path_seg, safe_dist);
    if (path_col_det_res == PATH_COL_NORMAL) {
      path_seg_vec.emplace_back(tmp_path_seg);
    } else if (path_col_det_res == PATH_COL_SHORTEN) {
      path_seg_vec.emplace_back(tmp_path_seg);
      break;
    } else if (path_col_det_res == PATH_COL_INVALID) {
      break;
    } else if (path_col_det_res == PATH_COL_INSIDE_STUCK) {
      if (i == 0 || calc_params_.first_multi_plan) {
        // when stuck by inside pos, can use line and arc to avoid it
        calc_params_.stuck_by_inside = true;
        return false;
      }
      path_seg_vec.emplace_back(tmp_path_seg);
      break;
    }
  }

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
      DEBUG_PRINT("this gear path is too small, lose it");
      path_seg_vec.clear();
    }
    DEBUG_PRINT("CalSinglePathInMulti success");
    return true;
  } else {
    DEBUG_PRINT("CalSinglePathInMulti fail");
    return false;
  }
}

const bool PerpendicularPathPlanner::OneArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear, const uint8_t current_arc_steer) {
  if (!pnc::geometry_lib::CalOneArcWithLine(
          arc, calc_params_.target_line,
          apa_param.GetParam().target_radius_err)) {
    DEBUG_PRINT("OneArcPlan fail 0");
    return false;
  }

  if (CheckArcOrLineAvailable(arc)) {
    uint8_t steer = pnc::geometry_lib::CalArcSteer(arc);
    uint8_t gear = pnc::geometry_lib::CalArcGear(arc);
    if (steer == current_arc_steer && gear == current_gear) {
      pnc::geometry_lib::PathSegment arc_seg(steer, gear, arc);
      path_seg_vec.emplace_back(arc_seg);
      return true;
    }
  }

  DEBUG_PRINT("OneArcPlan fail 1");
  return false;
}

const bool PerpendicularPathPlanner::TwoArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear, const uint8_t current_arc_steer) {
  pnc::geometry_lib::Arc arc2;
  if (!CalTwoArcWithLine(arc, arc2, calc_params_.target_line)) {
    DEBUG_PRINT("TwoArcPlan fail 0");
    return false;
  }

  bool arc_available = false;
  if (CheckArcOrLineAvailable(arc)) {
    uint8_t steer_1 = pnc::geometry_lib::CalArcSteer(arc);
    uint8_t gear_1 = pnc::geometry_lib::CalArcGear(arc);
    if (steer_1 == current_arc_steer && gear_1 == current_gear) {
      pnc::geometry_lib::PathSegment arc_seg1(steer_1, gear_1, arc);
      path_seg_vec.emplace_back(arc_seg1);
      arc_available = true;
    }
  }

  bool arc2_available = false;
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
      arc2_available = true;
    }
  }

  if (path_seg_vec.size() == 1 && arc_available) {
    const double dist =
        pnc::geometry_lib::CalPoint2LineDist(arc.pB, calc_params_.target_line);
    const double heading_err = std::fabs(pnc::geometry_lib::NormalizeAngle(
        arc.headingB - calc_params_.target_line.heading));
    if (dist < apa_param.GetParam().target_pos_err - 1e-6 &&
        heading_err < apa_param.GetParam().target_heading_err / 57.3 - 1e-6) {
      return true;
    } else {
      path_seg_vec.clear();
      return false;
    }
  }

  if (path_seg_vec.size() == 1 && arc2_available) {
    const double dist = (arc.pA - arc2.pA).norm();
    const double heading_err = std::fabs(arc.headingA - arc2.headingA);
    if (dist < apa_param.GetParam().target_pos_err - 1e-6 &&
        heading_err < apa_param.GetParam().target_heading_err / 57.3 - 1e-6) {
      return true;
    } else {
      path_seg_vec.clear();
      return false;
    }
  }

  if (path_seg_vec.empty()) {
    DEBUG_PRINT("TwoArcPlan fail 1");
    return false;
  }

  if (path_seg_vec.size() == 2 &&
      path_seg_vec[0].seg_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    pnc::geometry_lib::Arc virtual_arc1 = path_seg_vec[0].arc_seg;
    pnc::geometry_lib::Arc virtual_arc2 = path_seg_vec[1].arc_seg;
    const uint8_t arc2_steer = path_seg_vec[1].seg_steer;
    double tmp_arc_length = virtual_arc1.length;
    double trim_path_step = 0.108;
    Eigen::Vector2d t_vec;
    Eigen::Vector2d n_vec;
    bool no_use_arc2_flag = false;
    CollisionDetector::CollisionResult col_res;
    while (true) {
      if (tmp_arc_length < apa_param.GetParam().min_one_step_path_length ||
          virtual_arc1.pA.y() * virtual_arc1.pB.y() > 0.0) {
        break;
      }
      pnc::geometry_lib::CompleteArcInfo(virtual_arc1, tmp_arc_length,
                                         virtual_arc1.is_anti_clockwise);
      // constuct virtual_arc2
      virtual_arc2.pA = virtual_arc1.pB;
      virtual_arc2.headingA = virtual_arc1.headingB;
      virtual_arc2.length = 1.68;
      t_vec = pnc::geometry_lib::GenHeadingVec(virtual_arc2.headingA);
      if (arc2_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
        n_vec << t_vec.y(), -t_vec.x();
      } else {
        n_vec << -t_vec.y(), t_vec.x();
      }
      virtual_arc2.circle_info.center =
          virtual_arc2.pA + virtual_arc2.circle_info.radius * n_vec;
      pnc::geometry_lib::CompleteArcInfo(virtual_arc2, virtual_arc2.length,
                                         virtual_arc2.is_anti_clockwise);
      col_res = collision_detector_ptr_->UpdateByObsMap(virtual_arc2,
                                                        virtual_arc2.headingA);
      if (std::min(col_res.remain_car_dist,
                   col_res.remain_obstacle_dist -
                       apa_param.GetParam().col_obs_safe_dist_normal) < 1e-5) {
        tmp_arc_length -= trim_path_step;
        no_use_arc2_flag = true;
      } else {
        break;
      }
    }
    if (no_use_arc2_flag) {
      path_seg_vec.pop_back();
      DEBUG_PRINT("init arc length = " << path_seg_vec[0].arc_seg.length);
      path_seg_vec[0].arc_seg = virtual_arc1;
      calc_params_.can_insert_line = false;
    }
  }

  return true;
}

const bool PerpendicularPathPlanner::LineArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear, const uint8_t current_arc_steer) {
  pnc::geometry_lib::LineSegment line_seg1;
  line_seg1 = pnc::geometry_lib::BuildLineSegByPose(arc.pA, arc.headingA);

  pnc::geometry_lib::LineSegment line_seg2;
  line_seg2 = calc_params_.target_line;

  std::vector<Eigen::Vector2d> centers;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> tangent_ptss;
  if (!pnc::geometry_lib::CalCommonTangentCircleOfTwoLine(
          line_seg1, line_seg2, arc.circle_info.radius, centers,
          tangent_ptss)) {
    DEBUG_PRINT("LineArcPlan fail 0");
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
          head_err <= apa_param.GetParam().static_heading_eps / 57.3) {
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
          head_err <= apa_param.GetParam().target_heading_err / 57.3 - 1e-6) {
        line_arc_success = true;
        break;
      }
    }

    if (path_seg_vec.size() == 1 && arc_available) {
      const double dist = (arc.pA - tmp_arc.pA).norm();
      const double heading_err = std::fabs(arc.headingA - tmp_arc.headingA);
      if (dist < apa_param.GetParam().target_pos_err - 1e-6 &&
          heading_err < apa_param.GetParam().target_heading_err / 57.3 - 1e-6) {
        line_arc_success = true;
        break;
      }
    }
  }
  if (!line_arc_success) {
    path_seg_vec.clear();
  }
  if (path_seg_vec.empty()) {
    DEBUG_PRINT("LineArcPlan fail 1");
    return false;
  }
  return true;
}

const bool PerpendicularPathPlanner::OneLinePlan(
    pnc::geometry_lib::LineSegment& line,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear) {
  DEBUG_PRINT("--- try one line plan ---");
  pnc::geometry_lib::PathPoint pose;
  pose.Set(line.pA, line.heading);
  if (pnc::geometry_lib::IsPoseOnLine(
          pose, calc_params_.target_line,
          apa_param.GetParam().target_pos_err - 1e-6,
          apa_param.GetParam().target_heading_err / 57.3 - 1e-6)) {
    DEBUG_PRINT("pose is on line, success");
    line.pB.x() = calc_params_.target_line.pA.x();
    line.pB.y() = line.pA.y();
    line.length = (line.pB - line.pA).norm();
    line.heading = calc_params_.target_line.heading;
    const uint8_t seg_gear = pnc::geometry_lib::CalLineSegGear(line);
    if (seg_gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
        seg_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) {
      DEBUG_PRINT("the line gear is invalid");
      return false;
    }
    pnc::geometry_lib::PathSegment line_seg(seg_gear, line);
    path_seg_vec.emplace_back(line_seg);
    return true;
  } else {
    DEBUG_PRINT("pose is not on line, fail");
    return false;
  }
}
// multi plan end

// adjust plan start
// checkout adjust plan suitable
const bool PerpendicularPathPlanner::CheckAdjustPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose,
    const double slot_occupied_ratio) {
  if (std::fabs(current_pose.heading) <=
          apa_param.GetParam().adjust_plan_max_heading1_err / 57.3 ||
      (std::fabs(current_pose.heading) <=
           apa_param.GetParam().adjust_plan_max_heading2_err / 57.3 &&
       std::fabs(current_pose.pos.y()) <=
           apa_param.GetParam().adjust_plan_max_lat_err)) {
    return true;
  }
  DEBUG_PRINT("current pose is not suitable for adjust plan");
  return false;
}

const bool PerpendicularPathPlanner::AdjustPlan() {
  printf("-----adjust plan-----\n");
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
    current_gear = output_.gear_cmd_vec.back();
    current_arc_steer = output_.steer_vec.back();
    current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    DEBUG_PRINT("continue to plan after multi");
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
    DEBUG_PRINT("ref_gear or ref_arc_steer error");
    return false;
  }

  DEBUG_PRINT("try adjust plan to target point");
  bool success = false;
  calc_params_.multi_plan = false;
  double steer_change_ratio = 1.0;
  double steer_change_radius = apa_param.GetParam().max_radius_in_slot;

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  for (size_t i = 0; i < kAdjustPlanMaxPathNumsInSlot; ++i) {
    DEBUG_PRINT("-------- No." << i << " in adjust-plan--------");
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);
    if (CalSinglePathInAdjust(tmp_path_seg_vec, current_pose, current_gear,
                              steer_change_ratio, steer_change_radius, i)) {
      DEBUG_PRINT("single path of adjust plan success!");
      output_.path_available = true;
      success = true;
    } else {
      DEBUG_PRINT("single path of adjust plan failed!");
      fail_count += 1;
      if (fail_count == 1) {
      } else {
        calc_params_.adjust_fail_count += 1;
        DEBUG_PRINT("adjust_fail_count = " << calc_params_.adjust_fail_count);
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
            last_segment.seg_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) {
          DEBUG_PRINT("last line is not reverse, should lose");
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
    DEBUG_PRINT("adjust plan failed!");
    return false;
  }

  if (output_.path_segment_vec.size() > 0 && input_.is_replan_dynamic) {
    uint8_t ref_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
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
    DEBUG_PRINT("dynamic plan successful");
    return true;
  }
  return success;
}

const bool PerpendicularPathPlanner::OneArcPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  DEBUG_PRINT("try one arc plan");
  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;
  if (pnc::mathlib::IsDoubleEqual(arc.headingA,
                                  calc_params_.target_line.heading) ||
      pnc::mathlib::IsDoubleEqual(arc.pA.y(),
                                  calc_params_.target_line.pA.y())) {
    DEBUG_PRINT(
        "current heading is equal to target heading or current y is "
        "equal to "
        "target y, no need to one arc plan");
    return false;
  }
  bool success = pnc::geometry_lib::CalOneArcWithLineAndGear(
      arc, calc_params_.target_line, current_gear);

  success = success && CheckArcOrLineAvailable(arc);

  if (success) {
    // check radius and gear can or not meet needs
    DEBUG_PRINT("cal radius = " << arc.circle_info.radius);
    const auto steer = pnc::geometry_lib::CalArcSteer(arc);
    const auto gear = pnc::geometry_lib::CalArcGear(arc);
    success = (arc.circle_info.radius >= calc_params_.turn_radius - 1e-3 &&
               arc.circle_info.radius <=
                   apa_param.GetParam().max_one_step_arc_radius + 1e-3) &&
              (gear == current_gear);
    if (success) {
      DEBUG_PRINT("one arc plan success");
      // DEBUG_PRINT("start: coord = " << arc.pA.transpose()
      //           << "  heading = " << arc.headingA * 57.3
      //           << "  end: coord = " << arc.pB.transpose()
      //           << "  heading = " << arc.headingB * 57.3);
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }
  if (!success) {
    DEBUG_PRINT("one arc plan fail");
  }
  return success;
}

const bool PerpendicularPathPlanner::LineArcPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const double turn_radius) {
  DEBUG_PRINT("try line arc plan");

  if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE &&
      current_pose.pos.y() * current_pose.heading < 0.0) {
  } else if (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE &&
             current_pose.pos.y() * current_pose.heading > 0.0) {
    DEBUG_PRINT("should no line arc in this case");
    return false;
  } else {
    DEBUG_PRINT("should no line arc in this case");
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
    DEBUG_PRINT("LineArcPlan fail 0");
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
          head_err <= apa_param.GetParam().static_heading_eps / 57.3) {
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
          head_err <= apa_param.GetParam().target_heading_err / 57.3 - 1e-6) {
        line_arc_success = true;
        break;
      }
    }

    if (path_seg_vec.size() == 1 && arc_available) {
      const double dist = (current_pose.pos - tmp_arc.pA).norm();
      const double heading_err =
          std::fabs(current_pose.heading - tmp_arc.headingA);
      if (dist < apa_param.GetParam().target_pos_err - 1e-6 &&
          heading_err < apa_param.GetParam().target_heading_err / 57.3 - 1e-6) {
        line_arc_success = true;
        break;
      }
    }
  }

  if (!line_arc_success) {
    path_seg_vec.clear();
  }

  if (path_seg_vec.empty()) {
    DEBUG_PRINT("LineArcPlan fail 1");
    return false;
  }
  DEBUG_PRINT("line arc success");
  return true;
}

const bool PerpendicularPathPlanner::AlignBodyPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  DEBUG_PRINT("try align body plan");
  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;
  arc.circle_info.radius = calc_params_.turn_radius;
  // check if it is necessary to align body
  if (pnc::mathlib::IsDoubleEqual(arc.headingA,
                                  calc_params_.target_line.heading)) {
    DEBUG_PRINT("body already align");
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
    // DEBUG_PRINT("steer = " << static_cast<int>(steer));
    // DEBUG_PRINT("start: coord = " << arc.pA.transpose()
    //           << "  heading = " << arc.headingA * 57.3
    //           << "  end: coord = " << arc.pB.transpose()
    //           << "  heading = " << arc.headingB * 57.3);
    if (success) {
      DEBUG_PRINT("align body plan success");
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }
  if (!success) {
    DEBUG_PRINT("align body plan fail");
  }
  return success;
}

const bool PerpendicularPathPlanner::STurnParallelPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const double steer_change_ratio1,
    const double steer_change_radius) {
  DEBUG_PRINT("try s turn parallel plan");
  pnc::geometry_lib::Arc arc_s_1;
  arc_s_1.headingA = current_pose.heading;
  arc_s_1.pA = current_pose.pos;
  // check if it is possible to take S turn to target line
  if (std::fabs(arc_s_1.headingA - calc_params_.target_line.heading) >
      apa_param.GetParam().static_heading_eps / 57.3) {
    DEBUG_PRINT("body no align");
    return false;
  }
  arc_s_1.headingA = calc_params_.target_line.heading;
  // if (pnc::mathlib::IsDoubleEqual(arc_s_1.pA.y(),
  //                                 calc_params_.target_line.pA.y())) {
  //   DEBUG_PRINT("current pos is already on target line, no need to "
  //                "STurnParallelPlan");
  //   return true;
  // }
  if (std::fabs(arc_s_1.pA.y() - calc_params_.target_line.pA.y()) <
      apa_param.GetParam().static_pos_eps) {
    DEBUG_PRINT(
        "current pos is already on target line, no need to "
        "STurnParallelPlan");
    return true;
  }

  double slot_occupied_ratio = CalOccupiedRatio(current_pose);

  const std::vector<double> ratio_tab = {0.0, 0.2, 0.5, 0.8, 1.0};
  const double radius_change = apa_param.GetParam().max_radius_in_slot -
                               apa_param.GetParam().min_radius_out_slot;
  if (radius_change < 1e-8) {
    DEBUG_PRINT("radius setting is err");
    return false;
  }
  const std::vector<double> radius_tab = {
      apa_param.GetParam().min_radius_out_slot,
      apa_param.GetParam().min_radius_out_slot + radius_change * 0.22,
      apa_param.GetParam().min_radius_out_slot + radius_change * 0.55,
      apa_param.GetParam().min_radius_out_slot + radius_change * 0.88,
      apa_param.GetParam().min_radius_out_slot + radius_change};

  double real_steer_change_radius = steer_change_radius;
  real_steer_change_radius =
      pnc::mathlib::Interp1(ratio_tab, radius_tab, slot_occupied_ratio);

  // DEBUG_PRINT("real_steer_change_radius = " << real_steer_change_radius
  //          );

  arc_s_1.circle_info.radius = real_steer_change_radius;

  const auto steer_change_ratio =
      pnc::mathlib::Clamp(steer_change_ratio1, 0.1, 1.0);

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
        // DEBUG_PRINT("steer_1 = " << static_cast<int>(steer_1));
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
        // DEBUG_PRINT("steer_2 = " << static_cast<int>(steer_2));
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
          head_err <= apa_param.GetParam().static_heading_eps / 57.3 - 1e-6) {
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
          heading_err < apa_param.GetParam().target_heading_err / 57.3 - 1e-6) {
        return true;
      } else {
        path_seg_vec.clear();
        return false;
      }
    }
  }

  if (!success) {
    DEBUG_PRINT("s turn parallel plan fail");
  } else {
    DEBUG_PRINT("s turn parallel plan success");
  }

  return success;
}

const bool PerpendicularPathPlanner::CalSinglePathInAdjust(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t& current_gear, const double& steer_change_ratio,
    const double& steer_change_radius, const size_t& i) {
  DEBUG_PRINT("-----CalSinglePathInAdjust-----");
  DEBUG_PRINT("current_gear = "
              << static_cast<int>(current_gear)
              << ",  current_pos = " << current_pose.pos.transpose()
              << ",  current_heading = " << current_pose.heading * 57.3);

  const double lat_err = apa_param.GetParam().target_pos_err;
  const double heading_err = apa_param.GetParam().target_heading_err;
  if (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    apa_param.SetPram().target_pos_err = lat_err * 0.5;
    apa_param.SetPram().target_heading_err = heading_err * 0.5;
  }

  auto temp_pose = current_pose;
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
      DEBUG_PRINT("OneLinePlan success");
    } else {
      DEBUG_PRINT("OneLinePlan fail");
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
      DEBUG_PRINT("OneLinePlan success");
    } else {
      DEBUG_PRINT("OneLinePlan fail");
    }
  }

  // avoid line arc length too length whicl let car go too far
  if (line_arc_success && current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
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
    DEBUG_PRINT("add linearc obs");
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
    const uint8_t path_col_det_res = TrimPathByCollisionDetection(tmp_path_seg);
    if (output_.multi_reach_target_pose &&
        path_col_det_res != PATH_COL_NORMAL) {
      // this case should no col, otherwise lose all path
      path_seg_vec.clear();
      break;
    }
    if (path_col_det_res == PATH_COL_NORMAL) {
      path_seg_vec.emplace_back(tmp_path_seg);
    } else if (path_col_det_res == PATH_COL_SHORTEN) {
      // when gear is drive, if there is no only s turn, then when s_turn col,
      // lose all s turn path
      if (only_s_turn) {
        path_seg_vec.emplace_back(tmp_path_seg);
      } else {
        if (tmp_path_seg.plan_type == pnc::geometry_lib::PLAN_TYPE_S_TURN &&
            j > 0) {
          DEBUG_PRINT("s turn col, lose all s turn path.");
          if (tmp_path_seg_vec[j - 1].plan_type ==
              pnc::geometry_lib::PLAN_TYPE_S_TURN) {
            path_seg_vec.pop_back();
          }
        } else {
          path_seg_vec.emplace_back(tmp_path_seg);
        }
      }
      break;
    } else if (path_col_det_res == PATH_COL_INVALID) {
      break;
    } else if (path_col_det_res == PATH_COL_INSIDE_STUCK) {
      if (i == 0) {
        // when stuck by inside pos, can use more big radius to avoid it
        calc_params_.stuck_by_inside = true;
        return false;
      }
      path_seg_vec.emplace_back(tmp_path_seg);
      break;
    }
  }

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
      DEBUG_PRINT("this gear path is too small, lose it");
      path_seg_vec.clear();
    }
    DEBUG_PRINT("CalSinglePathInAdjust success");
    return true;
  } else {
    DEBUG_PRINT("CalSinglePathInAdjust fail");
    return false;
  }
}
// adjust plan end

// sample path start
const bool PerpendicularPathPlanner::SetCurrentPathSegIndex() {
  if (!output_.path_available) {
    return false;
  }

  if (output_.gear_cmd_vec.empty() || output_.path_segment_vec.empty() ||
      output_.steer_vec.empty()) {
    DEBUG_PRINT("no path can get");
    return false;
  }

  const size_t N = output_.path_segment_vec.size();
  if (output_.gear_cmd_vec.size() != N || output_.steer_vec.size() != N) {
    DEBUG_PRINT("size is not equal");
    return false;
  }

  // Now, the is_first_path is always true, and the first index is always 0
  if (output_.is_first_path == true) {
    // first get path segment from path_segment_vec, first and second index is
    // 0 at the moment
    output_.is_first_path = false;
  } else {
    if (output_.path_seg_index.second == N - 1) {
      DEBUG_PRINT("no more path can get");
      return false;
    }
    output_.path_seg_index.first = output_.path_seg_index.second + 1;
  }

  if (output_.path_seg_index.first == N - 1) {
    output_.path_seg_index.second = output_.path_seg_index.first;
  }

  if (output_.path_seg_index.first >= N) {
    DEBUG_PRINT("first index is err");
    return false;
  }
  output_.current_gear = output_.gear_cmd_vec[output_.path_seg_index.first];

  for (size_t i = output_.path_seg_index.first + 1; i < N; ++i) {
    // gear change, break
    if (output_.gear_cmd_vec[i] != output_.current_gear) {
      output_.path_seg_index.second = i - 1;
      break;
    }
    // gear always no change
    if (i == N - 1) {
      output_.path_seg_index.second = i;
    }
  }

  if (output_.path_seg_index.second >= N) {
    DEBUG_PRINT("second index is err");
    return false;
  }

  const int first = output_.path_seg_index.first;
  const int second = output_.path_seg_index.second;
  if (first < 0 || second < 0 || first > second) {
    DEBUG_PRINT("first and second index is err");
    return false;
  }
  for (int i = second; i >= first; --i) {
    if (output_.path_segment_vec[i].seg_type ==
        pnc::geometry_lib::SEG_TYPE_ARC) {
      output_.current_arc_steer = output_.steer_vec[i];
      break;
    }
  }

  if (output_.path_seg_index.second == N - 1) {
    DEBUG_PRINT("current path is final path");
    output_.is_last_path = true;
  }

  // DEBUG_PRINT("before insert");
  // DEBUG_PRINT("output_.segment_type_vec = [  ";
  // for (size_t i = 0; i < output_.path_segment_vec.size(); ++i) {
  //   DEBUG_PRINT(static_cast<int>(output_.path_segment_vec[i].seg_type) <<
  //   "
  //   ";
  // }
  // DEBUG_PRINT("]\noutput_.steer_cmd_vec = [  ";
  // for (size_t i = 0; i < output_.steer_vec.size(); ++i) {
  //   DEBUG_PRINT(static_cast<int>(output_.steer_vec[i]) << "  ";
  // }
  // DEBUG_PRINT("]\noutput_.gear_cmd_vec = [  ";
  // for (size_t i = 0; i < output_.gear_cmd_vec.size(); ++i) {
  //   DEBUG_PRINT(static_cast<int>(output_.gear_cmd_vec[i]) << "  ";
  // }
  // DEBUG_PRINT("]\n current_gear = " <<
  // static_cast<int>(output_.current_gear)
  //           << "   current_arc_steer = "
  //           << static_cast<int>(output_.current_arc_steer));
  // DEBUG_PRINT("current send path: first index = "
  //           << static_cast<int>(output_.path_seg_index.first)
  //           << "   second index = "
  //           << static_cast<int>(output_.path_seg_index.second) <<
  //           std::endl;

  return true;
}

void PerpendicularPathPlanner::SetLineSegmentHeading() {
  const size_t N = output_.path_segment_vec.size();
  if (N < 1) {
    return;
  }

  // if first path is line segment, use car heading
  // line segment do not change heading, use car heading or arc segment
  // heading
  double current_heading = 0.0;
  for (size_t i = 0; i < N; ++i) {
    auto& current_path_seg = output_.path_segment_vec[i];
    auto last_path_seg = output_.path_segment_vec[i];
    if (i > 0) {
      last_path_seg = output_.path_segment_vec[i - 1];
    }
    if (i == 0) {
      current_heading = input_.ego_pose.heading;
    } else if (last_path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      current_heading = last_path_seg.arc_seg.headingB;
    }
    if (current_path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      current_path_seg.line_seg.heading = current_heading;
    }
  }
}

void PerpendicularPathPlanner::InsertLineSegAfterCurrentFollowLastPath(
    double extend_distance) {
  if ((input_.is_replan_first || input_.is_replan_second) &&
      !output_.gear_shift) {
    return;
  }
  if (pnc::mathlib::IsDoubleEqual(extend_distance, 0.0)) {
    return;
  }
  if (output_.is_last_path == true &&
      output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    DEBUG_PRINT("path is last and gear is reverse, not extend path");
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
      path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    return;
  }

  int insert_case = -1;
  if (pnc::mathlib::IsInBound(path_seg.GetEndHeading() * 57.3, -0.208, 0.208)) {
    insert_case = 0;
  } else {
    if (length > apa_param.GetParam().min_one_step_path_length - 0.016) {
      if (std::fabs(path_seg.GetEndHeading() * 57.3) >
              apa_param.GetParam().multi_plan_min_heading_err &&
          path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        insert_case = 2;
      } else {
        DEBUG_PRINT("no need insert line");
        return;
      }
    } else {
      insert_case = 1;
    }
  }

  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
      extend_distance < 0.0) {
    DEBUG_PRINT("arc can not shorten");
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
          output_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
        const std::vector<double> lat_tab = {0.0, 0.05, 0.1, 0.15, 0.20};
        const std::vector<double> path_length_tab = {
            1.668 * min_path_length, 3.168 * min_path_length,
            3.868 * min_path_length, 4.268 * min_path_length,
            4.868 * min_path_length};

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
    DEBUG_PRINT("extend_distance = " << extend_distance);

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

    const uint8_t path_col_res = TrimPathByCollisionDetection(
        new_line, apa_param.GetParam().col_obs_safe_dist_strict);

    if (new_line.Getlength() < 0.02168) {
      return;
    }

    if (path_col_res == PATH_COL_NORMAL || path_col_res == PATH_COL_SHORTEN) {
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

      DEBUG_PRINT("insert line segment successful");
    } else {
      DEBUG_PRINT("can not inset line segment");
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

const bool PerpendicularPathPlanner::CheckCurrentGearLength() {
  if (output_.path_segment_vec.size() < 1) {
    return false;
  }
  double length = 0.0;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
    length += output_.path_segment_vec[i].Getlength();
  }

  return (length > kMinSingleGearPathLength) ? true : false;
}

void PerpendicularPathPlanner::ExtendCurrentFollowLastPath(
    double extend_distance) {
  if (pnc::mathlib::IsDoubleEqual(extend_distance, 0.0)) {
    return;
  }
  if (output_.is_last_path == true) {
    DEBUG_PRINT("is last path, not extend path");
    return;
  }
  if (output_.path_segment_vec.size() < 1) {
    return;
  }

  auto& path_seg = output_.path_segment_vec[output_.path_seg_index.second];
  const auto path_seg_length = path_seg.Getlength();
  const double min_path_seg_length = 0.2;
  if (extend_distance < 0.0) {
    if (path_seg_length < min_path_seg_length) {
      return;
    }
    if (path_seg_length + extend_distance < min_path_seg_length) {
      extend_distance = min_path_seg_length - path_seg_length;
    }
  }
  const auto path_seg_extended_length = path_seg_length + extend_distance;
  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    path_seg.line_seg.length = path_seg_extended_length;
    Eigen::Vector2d AB = path_seg.line_seg.pB - path_seg.line_seg.pA;
    Eigen::Vector2d AB_unit = AB.normalized();
    AB = path_seg_extended_length * AB_unit;
    path_seg.line_seg.pB = AB + path_seg.line_seg.pA;
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    path_seg.arc_seg.length = path_seg_extended_length;
    double theta =
        path_seg_extended_length / path_seg.arc_seg.circle_info.radius;
    const Eigen::Vector2d OA =
        path_seg.arc_seg.pA - path_seg.arc_seg.circle_info.center;
    if (path_seg.arc_seg.is_anti_clockwise == false) {
      theta = -theta;
    }
    const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(theta);
    const auto OB = rot_m * OA;
    path_seg.arc_seg.pB = OB + path_seg.arc_seg.circle_info.center;
    path_seg.arc_seg.headingB =
        pnc::geometry_lib::NormalizeAngle(path_seg.arc_seg.headingA + theta);
  }
  if (extend_distance > 0.0) {
    DEBUG_PRINT("--- extend distance collision --- ");
    TrimPathByCollisionDetection(path_seg);
  }
}

const bool PerpendicularPathPlanner::SampleCurrentPathSeg() {
  if (!output_.path_available) {
    return false;
  }

  if (output_.path_segment_vec.empty()) {
    return false;
  }

  if (input_.is_complete_path == true) {
    // for simulation
    output_.path_seg_index.first = 0;
    output_.path_seg_index.second = output_.gear_cmd_vec.size() - 1;
  }

  output_.path_point_vec.clear();
  output_.path_point_vec.reserve(kReservedOutputPathPointSize);

  double length = 0.0;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
    length += output_.path_segment_vec[i].Getlength();
  }
  size_t N = std::ceil(length / input_.sample_ds);
  double sample_ds = input_.sample_ds;
  const size_t max_seg_count = 7;
  if (N >= PLANNING_TRAJ_POINTS_NUM - APA_COMPARE_PLANNING_TRAJ_POINTS_NUM -
               max_seg_count) {
    N = PLANNING_TRAJ_POINTS_NUM - APA_COMPARE_PLANNING_TRAJ_POINTS_NUM -
        max_seg_count;
    sample_ds = length / static_cast<double>(N);
  }

  pnc::geometry_lib::PathPoint path_point;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
    const auto& current_seg = output_.path_segment_vec[i];

    if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      SampleLineSegment(current_seg.line_seg, sample_ds);
    } else {
      SampleArcSegment(current_seg.arc_seg, sample_ds);
    }
    if (i < output_.path_seg_index.second) {
      output_.path_point_vec.pop_back();
    }
  }

  JSON_DEBUG_VALUE("current_gear_length", length);
  JSON_DEBUG_VALUE("current_gear_pt_size", output_.path_point_vec.size());
  JSON_DEBUG_VALUE("sample_ds", sample_ds);

  return true;
}

void PerpendicularPathPlanner::SampleLineSegment(
    const pnc::geometry_lib::LineSegment& cur_line_seg, const double ds) {
  if (cur_line_seg.is_ignored) {
    return;
  }

  pnc::geometry_lib::PathPoint path_point;

  // get first point
  path_point.Set(cur_line_seg.pA, cur_line_seg.heading);
  output_.path_point_vec.emplace_back(path_point);

  if (cur_line_seg.length > ds) {
    auto pn = cur_line_seg.pA;
    const Eigen::Vector2d diff_vec =
        (cur_line_seg.pB - cur_line_seg.pA).normalized() * ds;

    double s = ds;
    while (s < cur_line_seg.length) {
      pn += diff_vec;
      path_point.Set(pn, cur_line_seg.heading);
      output_.path_point_vec.emplace_back(path_point);
      s += ds;
    }
  }
  // check the dist of the last point and end point
  const double dist =
      (output_.path_point_vec.back().pos - cur_line_seg.pB).norm();
  if (dist > 1e-2) {
    // get end point
    path_point.Set(cur_line_seg.pB, cur_line_seg.heading);
    output_.path_point_vec.emplace_back(path_point);
  }
}

void PerpendicularPathPlanner::SampleArcSegment(
    const pnc::geometry_lib::Arc& current_arc_seg, const double ds) {
  if (current_arc_seg.is_ignored) {
    return;
  }

  pnc::geometry_lib::PathPoint path_point;

  // get first point
  path_point.Set(current_arc_seg.pA,
                 pnc::geometry_lib::NormalizeAngle(current_arc_seg.headingA));
  output_.path_point_vec.emplace_back(path_point);

  if (current_arc_seg.length > ds) {
    const auto& pO = current_arc_seg.circle_info.center;
    double theta = current_arc_seg.headingA;
    const double dtheta = ds / current_arc_seg.circle_info.radius *
                          (current_arc_seg.is_anti_clockwise ? 1.0 : -1.0);

    const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(dtheta);
    Eigen::Vector2d pn = current_arc_seg.pA;

    Eigen::Vector2d v_n =
        current_arc_seg.pA - current_arc_seg.circle_info.center;

    double s = ds;
    while (s < current_arc_seg.length) {
      v_n = rot_m * v_n;
      pn = pO + v_n;
      s += ds;
      theta += dtheta;

      path_point.Set(pn, pnc::geometry_lib::NormalizeAngle(theta));
      output_.path_point_vec.emplace_back(path_point);
    }
  }

  // check the dist of the last point and end point
  const double dist =
      (output_.path_point_vec.back().pos - current_arc_seg.pB).norm();
  if (dist > 1e-2) {
    // get end point
    path_point.Set(current_arc_seg.pB, current_arc_seg.headingB);
    output_.path_point_vec.emplace_back(path_point);
  }
}
// sample path end

// collision detect start
const uint8_t PerpendicularPathPlanner::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg) {
  return TrimPathByCollisionDetection(
      path_seg, apa_param.GetParam().col_obs_safe_dist_normal);
}

const uint8_t PerpendicularPathPlanner::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg, const double safe_dist) {
  // DEBUG_PRINT("--- collision detection ---");
  CollisionDetector::CollisionResult col_res;
  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    auto& line = path_seg.line_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    auto& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
  } else {
    DEBUG_PRINT("no support the seg type");
    return PATH_COL_INVALID;
  }

  const double remain_car_dist = col_res.remain_car_dist;
  const double remain_obs_dist = col_res.remain_obstacle_dist;
  const double safe_remain_dist =
      std::min(remain_car_dist, remain_obs_dist - safe_dist);

  DEBUG_PRINT("remain_car_dist = "
              << remain_car_dist << "  remain_obs_dist = " << remain_obs_dist
              << "  safe_remain_dist = " << safe_remain_dist);

  if (safe_remain_dist < 1e-5) {
    DEBUG_PRINT(
        "safe_remain_dist is samller than 0.0, the path donot meet "
        "requirements");
    DEBUG_PRINT("collision_point = "
                << col_res.collision_point.transpose() << "  obs_pt_global = "
                << col_res.collision_point_global.transpose()
                << "  car_line_order = " << col_res.car_line_order);
    return PATH_COL_INVALID;
  }

  if (remain_car_dist > safe_remain_dist + 1e-3) {
    // DEBUG_PRINT(
    //     "the path will collide, the length need shorten to "
    //     "safe_remain_dist");
    DEBUG_PRINT("collision_point = "
                << col_res.collision_point.transpose() << "  obs_pt_global = "
                << col_res.collision_point_global.transpose()
                << "  car_line_order = " << col_res.car_line_order);
    if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      auto& line = path_seg.line_seg;
      if (!pnc::geometry_lib::CompleteLineInfo(line, safe_remain_dist)) {
        return PATH_COL_INVALID;
      }
    } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      auto& arc = path_seg.arc_seg;
      if (!pnc::geometry_lib::CompleteArcInfo(arc, safe_remain_dist,
                                              arc.is_anti_clockwise)) {
        return PATH_COL_INVALID;
      }
    }
    path_seg.collision_flag = true;
    if (calc_params_.multi_plan &&
        (col_res.collision_point_global.x() >
             input_.tlane.pt_inside.x() - 0.368 &&
         col_res.collision_point_global.x() <
             (input_.pt_0.x() + input_.pt_1.x()) * 0.5 + 2.68) &&
        (calc_params_.first_multi_plan ||
         path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE)) {
      if ((calc_params_.is_left_side &&
           col_res.collision_point_global.y() > 0.0) ||
          (!calc_params_.is_left_side &&
           col_res.collision_point_global.y() < 0.0)) {
        return PATH_COL_INSIDE_STUCK;
      }
    }

    return PATH_COL_SHORTEN;
  } else {
    // DEBUG_PRINT("the path will not collide");
    path_seg.collision_flag = false;
    return PATH_COL_NORMAL;
  }
}
// collision detect end

void PerpendicularPathPlanner::PrintSegmentInfo(
    const pnc::geometry_lib::PathSegment& seg) const {
  DEBUG_PRINT("----");
  DEBUG_PRINT("seg_gear: " << static_cast<int>(seg.seg_gear));

  DEBUG_PRINT("seg_steer: " << static_cast<int>(seg.seg_steer));
  DEBUG_PRINT("seg_type: " << static_cast<int>(seg.seg_type));
  DEBUG_PRINT("length: " << seg.Getlength());

  if (seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    DEBUG_PRINT("start_pos: " << seg.GetLineSeg().pA.transpose());
    DEBUG_PRINT("start_heading: " << seg.GetLineSeg().heading * 57.3);
    DEBUG_PRINT("end_pos: " << seg.GetLineSeg().pB.transpose());
    DEBUG_PRINT("end_heading: " << seg.GetLineSeg().heading * 57.3);
  } else {
    DEBUG_PRINT("start_pos: " << seg.GetArcSeg().pA.transpose());
    DEBUG_PRINT("start_heading: " << seg.GetArcSeg().headingA * 57.3);
    DEBUG_PRINT("end_pos: " << seg.GetArcSeg().pB.transpose());
    DEBUG_PRINT("end_heading: " << seg.GetArcSeg().headingB * 57.3);
  }
}

void PerpendicularPathPlanner::PrintOutputSegmentsInfo() const {
  DEBUG_PRINT("-------------- OutputSegmentsInfo --------------");
  const size_t N = std::min(2, int(output_.path_segment_vec.size()));
  for (size_t i = 0; i < N; i++) {
    const auto& current_seg = output_.path_segment_vec[i];

    if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      const auto& line_seg = current_seg.line_seg;

      DEBUG_PRINT("Segment [" << i << "] "
                              << " LINE_SEGMENT "
                              << " length= " << line_seg.length);

      DEBUG_PRINT("seg_gear: " << static_cast<int>(current_seg.seg_gear));

      DEBUG_PRINT("seg_steer: " << static_cast<int>(current_seg.seg_steer));

      DEBUG_PRINT("start_pos: " << line_seg.pA.transpose());
      DEBUG_PRINT("start_heading: " << line_seg.heading * 57.3);
      DEBUG_PRINT("end_pos: " << line_seg.pB.transpose() << "");
    } else {
      const auto& arc_seg = current_seg.arc_seg;

      DEBUG_PRINT("Segment [" << i << "] "
                              << "ARC_SEGMENT "
                              << "length= " << arc_seg.length);

      DEBUG_PRINT("seg_gear: " << static_cast<int>(current_seg.seg_gear));

      DEBUG_PRINT("seg_steer: " << static_cast<int>(current_seg.seg_steer));

      DEBUG_PRINT("start_pos: " << arc_seg.pA.transpose());
      DEBUG_PRINT("start_heading: " << arc_seg.headingA * 57.3);
      DEBUG_PRINT("end_pos: " << arc_seg.pB.transpose());
      DEBUG_PRINT("end_heading: " << arc_seg.headingB * 57.3);
      DEBUG_PRINT("center: " << arc_seg.circle_info.center.transpose()
                             << "  radius = " << arc_seg.circle_info.radius
                             << "");
    }
  }
}

const std::vector<double> PerpendicularPathPlanner::GetPathEle(
    size_t index) const {
  std::vector<double> out_vec;
  const auto& N = output_.path_point_vec.size();
  out_vec.reserve(N);

  for (size_t i = 0; i < N; ++i) {
    if (index == 0) {
      out_vec.emplace_back(output_.path_point_vec[i].pos.x());
    } else if (index == 1) {
      out_vec.emplace_back(output_.path_point_vec[i].pos.y());
    } else if (index == 2) {
      out_vec.emplace_back(output_.path_point_vec[i].heading);
    }
  }

  return out_vec;
}

const std::vector<double> PerpendicularPathPlanner::GetMinSafeCircle() const {
  return std::vector<double>{calc_params_.mono_safe_circle.center.x(),
                             calc_params_.mono_safe_circle.center.y(),
                             calc_params_.mono_safe_circle.radius};
}

const bool PerpendicularPathPlanner::IsRightCircle(
    const pnc::geometry_lib::PathPoint& ego_pose,
    const Eigen::Vector2d& center) const {
  return IsRightCircle(ego_pose.pos, ego_pose.heading, center);
}

const bool PerpendicularPathPlanner::IsRightCircle(
    const Eigen::Vector2d& ego_pos, const double ego_heading,
    const Eigen::Vector2d& center) const {
  const Eigen::Vector2d center_to_ego_vec = ego_pos - center;

  const Eigen::Vector2d ego_heading_vec(std::cos(ego_heading),
                                        std::sin(ego_heading));

  return (pnc::geometry_lib::GetCrossFromTwoVec2d(ego_heading_vec,
                                                  center_to_ego_vec) > 0.0);
}

const bool PerpendicularPathPlanner::CheckTwoPoseInCircle(
    const Eigen::Vector2d& ego_pos0, const double ego_heading0,
    const Eigen::Vector2d& ego_pos1, const double ego_heading1,
    const Eigen::Vector2d& center) const {
  return IsRightCircle(ego_pos0, ego_heading0, center) &&
         IsRightCircle(ego_pos1, ego_heading1, center);
}

const bool PerpendicularPathPlanner::CheckArcOrLineAvailable(
    const pnc::geometry_lib::Arc& arc) {
  const pnc::geometry_lib::PathPoint pose1(arc.pA, arc.headingA);
  const pnc::geometry_lib::PathPoint pose2(arc.pB, arc.headingB);
  if (pnc::geometry_lib::CheckTwoPoseIsSame(
          pose1, pose2, apa_param.GetParam().static_pos_eps,
          apa_param.GetParam().static_heading_eps / 57.3)) {
    DEBUG_PRINT("arc.pA = " << arc.pA.transpose()
                            << "  arc.headingA = " << arc.headingA * 57.3
                            << "  arc.pB = " << arc.pB.transpose()
                            << "  arc.headingB = " << arc.headingB * 57.3);

    return false;
  }
  return true;
}

const bool PerpendicularPathPlanner::CheckArcOrLineAvailable(
    const pnc::geometry_lib::LineSegment& line) {
  const pnc::geometry_lib::PathPoint pose1(line.pA, line.heading);
  const pnc::geometry_lib::PathPoint pose2(line.pB, line.heading);
  if (pnc::geometry_lib::CheckTwoPoseIsSame(
          pose1, pose2, apa_param.GetParam().static_pos_eps,
          apa_param.GetParam().static_heading_eps / 57.3)) {
    return false;
  }
  return true;
}

const bool PerpendicularPathPlanner::CheckPathIsNormal(
    const pnc::geometry_lib::PathSegment& path_seg) {
  double x_start;
  double x_end;
  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    x_start = path_seg.GetArcSeg().pA.x();
    x_end = path_seg.GetArcSeg().pB.x();
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    x_start = path_seg.GetLineSeg().pA.x();
    x_end = path_seg.GetLineSeg().pB.x();
  } else {
    return false;
  }
  double x_target = calc_params_.target_line.pA.x();
  if (x_start < x_target + 0.168 && x_end < x_target &&
      path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    return false;
  }
  return true;
}

const bool PerpendicularPathPlanner::CheckReachTargetPose(
    const pnc::geometry_lib::PathPoint& current_pose) {
  if ((current_pose.pos - calc_params_.target_line.pA).norm() <=
          apa_param.GetParam().target_pos_err &&
      std::fabs(pnc::geometry_lib::NormalizeAngle(
          current_pose.heading - input_.tlane.pt_terminal_heading)) <=
          apa_param.GetParam().target_heading_err / 57.3) {
    return true;
  }
  return false;
}

const bool PerpendicularPathPlanner::CheckReachTargetPose() {
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

const double PerpendicularPathPlanner::CalOccupiedRatio(
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
          apa_param.GetParam().slot_occupied_ratio_max_heading_err / 57.3) {
    slot_occupied_ratio = pnc::mathlib::Clamp(
        1.0 - (terminal_err.pos.x() / apa_param.GetParam().normal_slot_length),
        0.0, 1.0);
  } else {
    slot_occupied_ratio = 0.0;
  }
  return slot_occupied_ratio;
}

// for simulation
const bool PerpendicularPathPlanner::PreparePlanPb() { return PreparePlan(); }

const bool PerpendicularPathPlanner::PreparePlanSecondPb() {
  return PreparePlanSecond();
}

const bool PerpendicularPathPlanner::GenPathOutputByDubinsPb() {
  input_.ego_pose.pos = dubins_planner_.GetInput().p2;
  input_.ego_pose.heading = dubins_planner_.GetInput().heading2;
  return GenPathOutputByDubins();
}

const bool PerpendicularPathPlanner::MultiPlanPb() { return MultiPlan(); }

const bool PerpendicularPathPlanner::AdjustPlanPb() { return AdjustPlan(); }

const PerpendicularPathPlanner::PlannerParams&
PerpendicularPathPlanner::GetCalcParams() {
  return calc_params_;
}

const bool PerpendicularPathPlanner::CheckReachTargetPosePb() {
  return CheckReachTargetPose();
}

const bool PerpendicularPathPlanner::UpdatePb(
    const Input& input,
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  input_ = input;
  collision_detector_ptr_ = collision_detector_ptr;
  Preprocess();

  if (CheckReachTargetPose()) {
    DEBUG_PRINT("init pose is already at target pose");
    return true;
  }

  if (PreparePlan()) {
    calc_params_.adjust_fail_count = 0;
    calc_params_.first_multi_plan = true;
    DEBUG_PRINT("first prepare plan success");
  } else {
    DEBUG_PRINT("first prepare plan fail, quit");
    return false;
  }
  if (CheckReachTargetPose()) {
    DEBUG_PRINT("first prepare plan to target pose");
    return true;
  }

  if (PreparePlanSecond()) {
    DEBUG_PRINT("second prepare plan success");
  } else {
    DEBUG_PRINT("second prepare  plan fail");
  }
  if (CheckReachTargetPose()) {
    DEBUG_PRINT("second prepare plan to target pose");
    return true;
  }

  if (MultiPlan()) {
    DEBUG_PRINT("multi plan success");
  }

  if (CheckReachTargetPose()) {
    DEBUG_PRINT("multi plan to target pose");
    return true;
  }

  if (AdjustPlan()) {
    DEBUG_PRINT("adjust plan success");
    return true;
  }
  if (CheckReachTargetPose()) {
    DEBUG_PRINT("adjust plan to target pose");
    return true;
  }

  return false;
}

}  // namespace apa_planner
}  // namespace planning