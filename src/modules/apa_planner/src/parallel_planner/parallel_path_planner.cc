#include "parallel_path_planner.h"

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

#include "apa_plan_base.h"
#include "apa_world.h"
#include "collision_detection.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "math_lib.h"

namespace planning {
namespace apa_planner {
static const double kPie = 3.1415926;
static const double kRadiusEps = 0.01;
static const double kPosEps = 0.01;
static const double kMinLineStepLength = 0.3;
static const double kMinTurnRadius = 5.5;
static const double kMaxOneStepArcRadius = 8.5;
static const double kMaxRadiusInSlot = 12.66;
static const double kMaxIgnoredLength = 0.05;
static const double kMaxYCoordSeenAsLine = 0.03;
static const double kMaxHeadingSeenAsLine = 3.0 / 57.3;
static const double kFrontOverhanging = 0.924;
static const double kRearOverhanging = 0.94;
static const double kWheelBase = 2.7;
static const double kVehicleWidth = 1.89;
static const double kPrepareLineEgoSlotX = 6.95;
static const double kPrepareLineEgoSlotY = 3.0;
static const double kPrepareLineEgoSlotHeadingOffset = 0.0 / 57.3;
static const double kMinOneStepPathLength = 0.6;
static const size_t kMaxPerpenParkInSegmentNums = 15;
static const size_t kReservedOutputPathPointSize = 750;
static const size_t kMaxPathNumsInSlot = 6;
static const bool kP0P1CollisionDetectEnable = true;
static const bool kMonoPlanEnable = true;

static const std::vector<double> kPrepareTargetLineDeltaHeadingTab = {
    10.0 / 57.3, 12.0 / 57.3, 8.0 / 57.3};

static const std::vector<Eigen::Vector2d> kPrepareTargetLinePosTab = {
    Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(0.0, 0.0)};

void ParallelPathPlanner::Reset() {
  output_.Reset();
  output_.path_segment_vec.reserve(kMaxPerpenParkInSegmentNums);
  output_.gear_cmd_vec.reserve(kMaxPerpenParkInSegmentNums);

  calc_params_.Reset();
}

void ParallelPathPlanner::Preprocess() {
  calc_params_.Reset();

  // calc slot side by Tlane
  if (input_.tlane.p1.x() > input_.tlane.p0.x()) {
    calc_params_.is_right_side = true;
    calc_params_.slot_side_sgn = 1.0;
    calc_params_.target_heading = 0.0;
  } else {
    calc_params_.is_right_side = false;
    calc_params_.slot_side_sgn = -1.0;
    calc_params_.target_heading = kPie;
  }

  // target line
  const Eigen::Vector2d pt2 =
      input_.tlane.pt + Eigen::Vector2d(calc_params_.slot_side_sgn, 0.0);
  calc_params_.target_line.SetPoints(input_.tlane.pt, pt2);
  calc_params_.target_line.heading =
      (calc_params_.slot_side_sgn > 0.0 ? 0.0 : kPie);

  const double dist_front_bumper_to_rac = kFrontOverhanging + kWheelBase;
  const double half_width = 0.5 * kVehicleWidth;

  calc_params_.dist_r_corner_to_rac = std::hypot(half_width, kRearOverhanging);
  calc_params_.dist_f_corner_to_rac =
      std::hypot(half_width, dist_front_bumper_to_rac);

  calc_params_.fl_corner_vec << dist_front_bumper_to_rac, half_width;
  calc_params_.fr_corner_vec << dist_front_bumper_to_rac, -half_width;
  calc_params_.rl_corner_vec << -kRearOverhanging, half_width;
  calc_params_.rl_corner_vec << -kRearOverhanging, -half_width;

  calc_params_.min_fo_radius =
      std::hypot(kMinTurnRadius + half_width, dist_front_bumper_to_rac);

  calc_params_.min_fi_radius =
      std::hypot(kMinTurnRadius - half_width, dist_front_bumper_to_rac);
}

const bool ParallelPathPlanner::GenPathOutputByDubins() {
  // output_.Reset();
  const auto& dubins_output = dubins_planner_.GetOutput();
  output_.path_available = true;
  output_.length = dubins_output.length;
  output_.gear_change_count = dubins_output.gear_change_count;
  output_.current_gear = dubins_output.current_gear_cmd;

  // set arc AB
  if (dubins_output.gear_cmd_vec[0] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_AB =
        PathSegment(CalArcSteer(dubins_output.arc_AB),
                    dubins_output.current_gear_cmd, dubins_output.arc_AB);

    output_.steer_vec.emplace_back(CalArcSteer(dubins_output.arc_AB));
    output_.gear_cmd_vec.emplace_back(dubins_output.current_gear_cmd);
    output_.path_segment_vec.emplace_back(seg_AB);
  }

  // seg line BC
  if (dubins_output.gear_cmd_vec[1] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_BC =
        PathSegment(dubins_output.current_gear_cmd, dubins_output.line_BC);

    output_.steer_vec.emplace_back(pnc::geometry_lib::SEG_STEER_STRAIGHT);
    output_.gear_cmd_vec.emplace_back(dubins_output.current_gear_cmd);
    output_.path_segment_vec.emplace_back(seg_BC);
  }

  // set arc CD
  if (dubins_output.gear_cmd_vec[2] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_CD =
        PathSegment(CalArcSteer(dubins_output.arc_CD),
                    dubins_output.current_gear_cmd, dubins_output.arc_CD);

    output_.steer_vec.emplace_back(CalArcSteer(dubins_output.arc_CD));
    output_.gear_cmd_vec.emplace_back(dubins_output.current_gear_cmd);
    output_.path_segment_vec.emplace_back(seg_CD);
  }

  std::cout << "prepare plan output:\n";
  for (size_t i = 0; i < output_.path_segment_vec.size(); ++i) {
    auto& path_seg = output_.path_segment_vec[i];
    std::cout << i << "th path seg info:\n";
    std::cout << "type = " << static_cast<int>(path_seg.seg_type)
              << "  length = " << path_seg.Getlength()
              << "  direction = " << static_cast<int>(path_seg.seg_direction)
              << "  steer = " << static_cast<int>(path_seg.seg_steer)
              << std::endl;
    if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      std::cout << "pA = " << path_seg.line_seg.pA.transpose()
                << "  pB = " << path_seg.line_seg.pB.transpose()
                << "  heading = " << path_seg.line_seg.heading * 57.3
                << std::endl;
    } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      std::cout << "pA = " << path_seg.arc_seg.pA.transpose()
                << "  pB = " << path_seg.arc_seg.pB.transpose()
                << "  headingA = " << path_seg.arc_seg.headingA * 57.3
                << "  headingB = " << path_seg.arc_seg.headingB * 57.3
                << "  radius = " << path_seg.arc_seg.circle_info.radius
                << "  center = "
                << path_seg.arc_seg.circle_info.center.transpose() << std::endl;
    }
  }

  const bool res =
      (output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE);

  if (res) {
    const auto& dubins_input = dubins_planner_.GetInput();
    input_.ego_pose.Set(dubins_input.p2, dubins_input.heading2);
  }

  return res;
}

const bool ParallelPathPlanner::PreparePlanOnce(const double y_offset,
                                                const double heading_offset) {
  double start_heading =
      (calc_params_.slot_side_sgn > 0.0 ? heading_offset
                                        : kPie - heading_offset);
  std::cout << "calc_params_.slot_side_sgn: " << calc_params_.slot_side_sgn
            << std::endl;

  pnc::geometry_lib::PathPoint start_point;

  start_point.Set(Eigen::Vector2d(input_.tlane.p1.x(), y_offset),
                  start_heading);

  // cal pre line tangent vec and normal vec
  const Eigen::Vector2d line_tangent_vec =
      pnc::geometry_lib::GenHeadingVec(start_point.heading);

  // gen prepare line
  calc_params_.prepare_line = pnc::geometry_lib::LineSegment(
      start_point.pos, start_point.pos + line_tangent_vec);

  calc_params_.prepare_line.heading = start_point.heading;

  calc_params_.pre_line_tangent_vec = line_tangent_vec;

  // find a target point on prepare line
  pnc::geometry_lib::PathPoint target_point;
  target_point.heading = calc_params_.prepare_line.heading;
  target_point.pos.y() = start_point.pos.y();
  // target_point.pos.x() = input_.tlane.pt.x() + 1.5 * kMinTurnRadius * 1.05;
  target_point.pos.x() = 7.0 * calc_params_.slot_side_sgn;

  pnc::dubins_lib::DubinsLibrary::Input input;
  input.radius = kMinTurnRadius * 1.05;

  input.Set(input_.ego_pose.pos, target_point.pos, input_.ego_pose.heading,
            target_point.heading);

  std::cout << "prepare start_pos: " << input.p1.transpose()
            << ", heading: " << input.heading1 * 57.3
            << "end_pos: " << input.p2.transpose()
            << ", heading: " << input.heading2 * 57.3 << std::endl;

  dubins_planner_.SetInput(input);

  const bool prepare_success = dubins_planner_.OneStepDubinsUpdate();
  if (prepare_success) {
    std::cout << "prepare to tangent point successful" << std::endl;
    calc_params_.safe_circle_key_pt = target_point.pos;
    calc_params_.safe_circle_key_heading = target_point.heading;
  } else {
    std::cout << "prepare to tangent pt failed!" << std::endl;
  }
  return prepare_success;
}

const bool ParallelPathPlanner::PreparePlan() {
  std::vector<double> y_offset_vec;
  std::vector<double> heading_offset_vec;

  double y_offset = 2.8;
  while (y_offset < kPrepareLineEgoSlotY + 1.5) {
    y_offset_vec.emplace_back(y_offset);
    y_offset += 0.1;
  }

  heading_offset_vec.emplace_back(kPrepareLineEgoSlotHeadingOffset);

  for (const auto& heading_offset : heading_offset_vec) {
    for (const auto& y_offset : y_offset_vec) {
      if (PreparePlanOnce(y_offset, heading_offset)) {
        std::cout << "y_offset = " << y_offset << std::endl;
        std::cout << "heading_offset = " << heading_offset * 57.3 << std::endl;
        return true;
      }
    }
  }

  return false;
}

bool ParallelPathPlanner::Update() {
  std::cout << "----------------------------------------- path "
               "planner:---------------------------------------"
            << std::endl;
  // preprocess
  Preprocess();

  // reset output
  output_.Reset();

  // prepare plan, only for first plan
  if (input_.is_replan_first) {
    if (!PreparePlan()) {
      std::cout << "prepare plan failed!" << std::endl;
      return false;
    }
    if (GenPathOutputByDubins() == false) {
      std::cout << "prepare gear is drive, no need multi plan" << std::endl;
      return true;
    }
    std::cout << "prepare gear is reverse, need mono plan" << std::endl;
  }

  // // mono step
  // if (MonoStepPlan()) {
  //   std::cout << "mono step plan success!" << std::endl;
  //   return true;
  // } else {
  //   std::cout << "mono step plan failed!" << std::endl;
  // }

  // adjust step
  if (AdjustStepPlan()) {
    std::cout << "adjust step plan success!" << std::endl;
    return true;
  } else {
    std::cout << "adjust step plan failed!" << std::endl;
  }

  return false;
}

const bool ParallelPathPlanner::MonoStepPlan() {
  pnc::dubins_lib::DubinsLibrary::Input park_step_input;
  park_step_input.radius = kMinTurnRadius;

  park_step_input.Set(input_.ego_pose.pos, input_.tlane.pt,
                      input_.ego_pose.heading, calc_params_.target_heading);

  dubins_planner_.SetInput(park_step_input);
  const bool success = dubins_planner_.OneStepDubinsUpdate();
  if (!success) {
    return false;
  }

  // output_.Reset();

  const auto& dubins_output = dubins_planner_.GetOutput();
  output_.path_available = true;
  output_.length = dubins_output.length;
  output_.gear_change_count = dubins_output.gear_change_count;
  output_.current_gear = dubins_output.current_gear_cmd;

  // set arc AB
  if (dubins_output.gear_cmd_vec[0] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_AB =
        PathSegment(CalArcSteer(dubins_output.arc_AB),
                    dubins_output.current_gear_cmd, dubins_output.arc_AB);

    output_.steer_vec.emplace_back(CalArcSteer(dubins_output.arc_AB));
    output_.gear_cmd_vec.emplace_back(dubins_output.current_gear_cmd);
    output_.path_segment_vec.emplace_back(seg_AB);
  }

  // seg line BC
  if (dubins_output.gear_cmd_vec[1] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    const auto seg_BC =
        PathSegment(dubins_output.current_gear_cmd, dubins_output.line_BC);

    output_.steer_vec.emplace_back(pnc::geometry_lib::SEG_STEER_STRAIGHT);
    output_.gear_cmd_vec.emplace_back(dubins_output.current_gear_cmd);
    output_.path_segment_vec.emplace_back(seg_BC);
  }

  // set arc CD
  if (dubins_output.gear_cmd_vec[2] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_CD =
        PathSegment(CalArcSteer(dubins_output.arc_CD),
                    dubins_output.current_gear_cmd, dubins_output.arc_CD);

    output_.steer_vec.emplace_back(CalArcSteer(dubins_output.arc_CD));
    output_.gear_cmd_vec.emplace_back(dubins_output.current_gear_cmd);
    output_.path_segment_vec.emplace_back(seg_CD);
  }

  std::cout << "parking in slot output:\n";
  for (size_t i = 0; i < output_.path_segment_vec.size(); ++i) {
    auto& path_seg = output_.path_segment_vec[i];
    std::cout << i << "th path seg info:\n";
    std::cout << "type = " << static_cast<int>(path_seg.seg_type)
              << "  length = " << path_seg.Getlength()
              << "  direction = " << static_cast<int>(path_seg.seg_direction)
              << "  steer = " << static_cast<int>(path_seg.seg_steer)
              << std::endl;
    if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      std::cout << "pA = " << path_seg.line_seg.pA.transpose()
                << "  pB = " << path_seg.line_seg.pB.transpose()
                << "  heading = " << path_seg.line_seg.heading * 57.3
                << std::endl;
    } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      std::cout << "pA = " << path_seg.arc_seg.pA.transpose()
                << "  pB = " << path_seg.arc_seg.pB.transpose()
                << "  headingA = " << path_seg.arc_seg.headingA * 57.3
                << "  headingB = " << path_seg.arc_seg.headingB * 57.3
                << "  radius = " << path_seg.arc_seg.circle_info.radius
                << "  center = "
                << path_seg.arc_seg.circle_info.center.transpose() << std::endl;
    }
  }

  return true;
}

const bool ParallelPathPlanner::AdjustStepPlan() {
  bool success = AdjustStepPlanOnce(1.0, kMinTurnRadius);

  if (success &&
      output_.gear_cmd_vec.back() == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    return true;
  }

  if (success &&
      output_.gear_cmd_vec.back() == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    if (output_.length <= 5.0) {
      return true;
    } else {
      output_.Reset();
      success = AdjustStepPlanOnce(0.35, kMinTurnRadius);
      if (success && output_.length <= 5.0) {
        return true;
      }
    }
  }

  return false;
}

const bool ParallelPathPlanner::AdjustStepPlanOnce(
    const double shift_scale, const double min_shift_radius) {
  CollisionDetector::CollisionResult collision_result;
  bool success = false;

  PathSegment line_seg;
  success =
      OneLineGeometryPlan(input_.ego_pose, calc_params_.target_line, line_seg);
  if (success) {
    output_.path_available = true;
    output_.path_segment_vec.emplace_back(line_seg);
    output_.gear_cmd_vec.emplace_back(line_seg.seg_direction);
    output_.steer_vec.emplace_back(pnc::geometry_lib::SEG_STEER_STRAIGHT);
    output_.length += line_seg.Getlength();
    std::cout << "one line success!" << std::endl;
    return true;
  }

  const bool is_advance =
      (input_.ref_gear == pnc::geometry_lib::SEG_GEAR_DRIVE);
  std::cout << "is_advance = " << is_advance << std::endl;

  const auto fix_shift_scale = pnc::mathlib::Clamp(shift_scale, 0.1, 1.0);
  // try one step arc
  pnc::geometry_lib::Arc adjust_arc;
  if (fix_shift_scale > 0.99) {
    success = pnc::geometry_lib::OneStepArcTargetLineByGear(
        adjust_arc, input_.ego_pose.pos, input_.ego_pose.heading, is_advance,
        calc_params_.target_line);

    success =
        success && (adjust_arc.circle_info.radius >= kMinTurnRadius &&
                    adjust_arc.circle_info.radius <= kMaxOneStepArcRadius);

    std::cout << "adjust_arc.circle_info.radius = "
              << adjust_arc.circle_info.radius << std::endl;
  }

  if (success) {
    const auto steer = CalArcSteer(adjust_arc);
    const auto gear = CalArcGear(adjust_arc);
    output_.path_segment_vec.emplace_back(PathSegment(steer, gear, adjust_arc));

    output_.steer_vec.emplace_back(steer);
    output_.gear_cmd_vec.emplace_back(gear);
    output_.length += adjust_arc.length;

    std::cout << "OneStepArcTargetLine success!" << std::endl;
  } else {
    // if not success, try two step arc
    std::cout << "OneStepArcTargetLine failed!" << std::endl;
    std::cout << "is_advance = " << is_advance << std::endl;

    // align the ego body
    pnc::geometry_lib::Arc adjust_arc_head;

    pnc::geometry_lib::OneStepArcTargetHeading(
        adjust_arc_head, input_.ego_pose.pos, input_.ego_pose.heading,
        calc_params_.target_line.heading, kMinTurnRadius, is_advance);

    const auto target_heading_line = ConstructEgoHeadingLine(
        adjust_arc_head.pB, calc_params_.target_line.heading);

    auto target_line = calc_params_.target_line;

    if (fix_shift_scale < 0.99) {
      target_line.SetPoints(
          fix_shift_scale * target_heading_line.pA +
              (1.0 - fix_shift_scale) * calc_params_.target_line.pA,
          fix_shift_scale * target_heading_line.pA +
              (1.0 - fix_shift_scale) * calc_params_.target_line.pB);
    }

    const auto steer = CalArcSteer(adjust_arc_head);
    const auto gear = CalArcGear(adjust_arc_head);

    output_.path_segment_vec.emplace_back(
        PathSegment(steer, gear, adjust_arc_head));

    output_.gear_cmd_vec.emplace_back(gear);
    output_.steer_vec.emplace_back(steer);
    output_.length += adjust_arc_head.length;

    std::pair<pnc::geometry_lib::Arc, pnc::geometry_lib::Arc> arc_pair;

    std::cout << "target_line_pA = " << target_line.pA.transpose() << std::endl;
    std::cout << "target_line_pB = " << target_line.pB.transpose() << std::endl;

    success = pnc::geometry_lib::OneStepParallelShift(
        arc_pair, adjust_arc_head.pB, adjust_arc_head.headingB, target_line,
        min_shift_radius, is_advance);

    if (!success) {
      std::cout << "OneStepParallelShift failed!" << std::endl;
      return false;
    } else {
      std::cout << "OneStepParallelShift successful!" << std::endl;
    }

    output_.path_segment_vec.emplace_back(
        PathSegment(CalArcSteer(arc_pair.first), CalArcGear(arc_pair.first),
                    arc_pair.first));

    output_.gear_cmd_vec.emplace_back(CalArcGear(arc_pair.first));
    output_.steer_vec.emplace_back(CalArcSteer(arc_pair.first));

    output_.length += arc_pair.first.length;

    output_.path_segment_vec.emplace_back(
        PathSegment(CalArcSteer(arc_pair.second), CalArcGear(arc_pair.second),
                    arc_pair.second));

    output_.gear_cmd_vec.emplace_back(CalArcGear(arc_pair.second));
    output_.steer_vec.emplace_back(CalArcSteer(arc_pair.second));

    output_.length += arc_pair.second.length;
  }

  pnc::geometry_lib::LineSegment last_line_seg;
  last_line_seg.SetPoints(output_.path_segment_vec.back().GetArcSeg().pB,
                          input_.tlane.pt);
  last_line_seg.heading = output_.path_segment_vec.back().GetArcSeg().headingB;
  last_line_seg.is_ignored = false;

  const uint8_t last_gear =
      last_line_seg.pA.x() * calc_params_.slot_side_sgn <
              last_line_seg.pB.x() * calc_params_.slot_side_sgn
          ? pnc::geometry_lib::SEG_GEAR_DRIVE
          : pnc::geometry_lib::SEG_GEAR_REVERSE;

  output_.path_available = true;

  // exit when gear changed
  if (last_gear != input_.ref_gear) {
    return true;
  }

  output_.path_segment_vec.emplace_back(PathSegment(last_gear, last_line_seg));

  output_.gear_cmd_vec.emplace_back(last_gear);
  output_.steer_vec.emplace_back(pnc::geometry_lib::SEG_STEER_STRAIGHT);
  output_.length += last_line_seg.length;

  return true;
}

const uint8_t ParallelPathPlanner::CalArcGear(
    const pnc::geometry_lib::Arc& arc) const {
  const Eigen::Vector2d v_ab = arc.pB - arc.pA;

  const Eigen::Vector2d v_heading_a(std::cos(arc.headingA),
                                    std::sin(arc.headingA));

  return (v_ab.dot(v_heading_a) > 0.0) ? pnc::geometry_lib::SEG_GEAR_DRIVE
                                       : pnc::geometry_lib::SEG_GEAR_REVERSE;
}

const uint8_t ParallelPathPlanner::CalArcSteer(
    const pnc::geometry_lib::Arc& arc) const {
  const Eigen::Vector2d v_heading_a(std::cos(arc.headingA),
                                    std::sin(arc.headingA));

  const Eigen::Vector2d v_oa = arc.pA - arc.circle_info.center;

  return (pnc::geometry_lib::GetCrossFromTwoVec2d(v_oa, v_heading_a) > 0.0)
             ? pnc::geometry_lib::SEG_STEER_LEFT
             : pnc::geometry_lib::SEG_STEER_RIGHT;
}

const bool ParallelPathPlanner::MultiStepPlan() {
  // output_.Reset();

  if (std::fabs(input_.ego_pose.pos.y()) < 0.5 &&
      std::fabs(input_.ego_pose.heading) < 12.0 / 57.3) {
    std::cout << "skip multi-step plan, try adjust plan first!" << std::endl;
    return false;
  }

  uint8_t current_direction = 0;
  uint8_t current_arc_steer = 0;
  pnc::geometry_lib::PathPoint current_pose;

  // set ref gear
  current_direction = input_.ref_gear;
  // set ref steer
  current_arc_steer = input_.ref_arc_steer;

  std::cout << "current_direction = " << static_cast<int>(current_direction)
            << std::endl;

  std::cout << "current_arc_steer = " << static_cast<int>(current_arc_steer)
            << std::endl;

  // set plan init state
  current_pose = input_.ego_pose;

  // step 1 multi-step search
  // firstly go backward along p1, then turn to the other side and go
  // forward.

  bool success = false;
  std::vector<PathSegment> tmp_segment_vec;
  for (size_t i = 0; i < kMaxPathNumsInSlot; i++) {
    std::cout << "\n-------- No." << i << " step in multi-plan----------------"
              << std::endl;

    std::cout << "current steer: " << static_cast<int>(current_arc_steer)
              << ", direction: " << static_cast<int>(current_direction)
              << std::endl;

    const bool single_path_success = CalOnePathInMultiStep(
        current_pose, current_direction, current_arc_steer, tmp_segment_vec);

    if (single_path_success) {
      output_.path_available = true;
      success = true;
    } else {
      std::cout << "single path of multi-plan failed!" << std::endl;
      output_.path_available = false;
      success = false;
      break;
    }

    for (const auto& seg : tmp_segment_vec) {
      output_.path_segment_vec.emplace_back(seg);
      output_.length += seg.Getlength();
      output_.gear_cmd_vec.emplace_back(seg.seg_direction);
      output_.steer_vec.emplace_back(current_arc_steer);

      // PrintSegmentInfo(seg);
    }

    std::cout << "input_.tlane.slot_side = "
              << static_cast<int>(input_.tlane.slot_side) << std::endl;

    const auto& last_segment = output_.path_segment_vec.back();

    pnc::geometry_lib::PathPoint last_pose;
    if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      last_pose.Set(last_segment.GetArcSeg().pB,
                    last_segment.GetArcSeg().headingB);
    } else if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      last_pose.Set(last_segment.GetLineSeg().pB,
                    last_segment.GetLineSeg().heading);
    }

    current_direction =
        (current_direction == pnc::geometry_lib::SEG_GEAR_REVERSE
             ? pnc::geometry_lib::SEG_GEAR_DRIVE
             : pnc::geometry_lib::SEG_GEAR_REVERSE);
    current_arc_steer = (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT
                             ? pnc::geometry_lib::SEG_STEER_LEFT
                             : pnc::geometry_lib::SEG_STEER_RIGHT);

    // std::cout << "current pose: " << current_pose.pos.transpose()
    //           << "heading: " << current_pose.heading * 57.3 << std::endl;
    // std::cout << "end pose: " << current_pose.pos.transpose()
    //           << "end heading: " << current_pose.heading * 57.3 <<
    //           std::endl;
    // std::cout << "next steer: " << static_cast<int>(current_arc_steer)
    //           << ", next direction: " <<
    //           static_cast<int>(current_direction)
    //           << std::endl;

    current_pose = last_pose;

    if ((last_pose.pos - input_.tlane.pt).norm() <= kPosEps) {
      std::cout << "already plan to target pos!" << std::endl;
      break;
    } else {
      // try line
      PathSegment last_line_seg;
      const bool last_Line_success = OneLineGeometryPlan(
          current_pose, calc_params_.target_line, last_line_seg);
      if (last_Line_success) {
        output_.path_segment_vec.emplace_back(last_line_seg);
        output_.gear_cmd_vec.emplace_back(last_line_seg.seg_direction);
        output_.steer_vec.emplace_back(pnc::geometry_lib::SEG_STEER_STRAIGHT);
        output_.length += last_line_seg.Getlength();
        output_.path_available = true;
        break;
      }
    }
  }

  return success;
}

void ParallelPathPlanner::GenPrepareTargetLine(
    pnc::geometry_lib::LineSegment& line_seg, const double delta_heading,
    const Eigen::Vector2d& pos) {
  // pA is tag point
}

const bool ParallelPathPlanner::CalOnePathInMultiStep(
    const pnc::geometry_lib::PathPoint& current_pose, const uint8_t direction,
    const uint8_t steer, std::vector<PathSegment>& segment_vec) {
  const double radius = kMinTurnRadius;
  segment_vec.clear();
  segment_vec.reserve(2);

  Eigen::Vector2d current_center = CalEgoTurningCenter(
      current_pose.pos, current_pose.heading, radius, steer);

  const double dist = pnc::geometry_lib::CalPoint2LineDist(
      current_center, calc_params_.target_line);

  std::cout << "each step in multi-plan: current circle to target line dist = "
            << dist << std::endl;

  std::cout << "current_pose.pos = " << current_pose.pos.transpose()
            << std::endl;

  std::cout << "current_pose.heading = " << current_pose.heading << std::endl;
  std::cout << "steer = " << static_cast<int>(steer) << std::endl;

  bool cal_success = false;
  std::vector<PathSegment> filtered_segs;
  pnc::geometry_lib::PathPoint limit_pose;
  PathSegment opt_arc_segment;
  std::vector<PathSegment> current_path_seg_vec;

  if (dist >= radius + kRadiusEps && false) {
    cal_success = LineArcLinePlan(current_pose, direction, steer, radius,
                                  current_path_seg_vec);
    if (cal_success) {
      std::cout << "dist is more than radius, try line-arc-line success!"
                << std::endl;
    } else {
      std::cout << "dist is more than radius, try line-arc-line failed!"
                << std::endl;
    }
    // use line arc line
  } else if (dist > radius - kRadiusEps) {
    cal_success = OneArcGeometryPlan(current_pose, calc_params_.target_line,
                                     radius, steer, opt_arc_segment);

    if (cal_success) {
      std::cout << "dist is equal to radius plan success!" << std::endl;
      segment_vec.emplace_back(opt_arc_segment);
    } else {
      std::cout << "dist is equal to radius plan failed!" << std::endl;
    }

  } else {
    cal_success = InverseTwoArcPlan(current_pose, direction, steer, radius,
                                    opt_arc_segment);
    if (cal_success) {
      std::cout << "first step of inverse two arc plan success!" << std::endl;
      segment_vec.emplace_back(opt_arc_segment);
    } else {
      std::cout << "first step of inverse two arc plan failed!" << std::endl;
    }
  }

  return cal_success;
}

const bool ParallelPathPlanner::LineArcLinePlan(
    const pnc::geometry_lib::PathPoint& start_pose, const uint8_t direction,
    const uint8_t steer, const double radius,
    std::vector<PathSegment>& line_arc_line_segments) const {
  const auto& current_line =
      ConstructEgoHeadingLine(start_pose.pos, start_pose.heading);

  pnc::geometry_lib::PathPoint target_pose = {input_.tlane.pt,
                                              calc_params_.target_line.heading};

  std::vector<Eigen::Vector2d> possible_centers;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> possible_tangent_pts;

  const auto& target_line = calc_params_.target_line;

  const bool cal_res = pnc::geometry_lib::CalTangentCirclesOfTwoLines(
      current_line, target_line, radius, possible_centers,
      possible_tangent_pts);

  if (!cal_res) {
    std::cout << "current line is parallel to target line, line-arc-line failed"
              << std::endl;
    return false;
  }

  // select correct circle
  size_t correct_pt_idx = 0;
  const bool select_circle_success = GetCorrectCircleIndexofLineArcLine(
      start_pose.heading, target_pose.heading, steer, possible_centers,
      possible_tangent_pts, correct_pt_idx);

  if (!select_circle_success) {
    std::cout << "no tangent circle satisfies the requirement of circle heading"
              << std::endl;
    return false;
  }

  // check dirve direction
  pnc::geometry_lib::Circle circle_step = {possible_centers[correct_pt_idx],
                                           radius};
  const bool res = SelectLineArcLineByDir(
      start_pose, target_pose, possible_tangent_pts[correct_pt_idx],
      circle_step, direction, line_arc_line_segments);

  if (!res) {
    std::cout << "dirve dir is different!" << std::endl;
  } else {
    std::cout << "line arc line success" << std::endl;
  }

  for (size_t i = 0; i < line_arc_line_segments.size(); ++i) {
    std::cout << i << "th path seg = "
              << static_cast<int>(line_arc_line_segments[i].seg_type)
              << std::endl;
  }

  return res;
}

const bool ParallelPathPlanner::GetCorrectCircleIndexofLineArcLine(
    const double start_heading, const double target_heading,
    const uint8_t steer, const std::vector<Eigen::Vector2d>& possible_centers,
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>&
        possible_tangent_pts,
    size_t& circle_index) const {
  std::cout << "possible_tangent_pts.size: " << possible_tangent_pts.size()
            << std::endl;

  std::cout << "desired steer :" << static_cast<int>(steer) << std::endl;

  size_t correct_pt_idx = 0;
  for (; correct_pt_idx < possible_centers.size(); correct_pt_idx++) {
    std::cout << "center: " << possible_centers[correct_pt_idx].transpose()
              << std::endl;

    const uint8_t tangent_pt1_steer =
        IsRightCircle(possible_tangent_pts[correct_pt_idx].first, start_heading,
                      possible_centers[correct_pt_idx])
            ? pnc::geometry_lib::SEG_STEER_RIGHT
            : pnc::geometry_lib::SEG_STEER_LEFT;

    // std::cout << "tangent_pt1: "
    //           << possible_tangent_pts[correct_pt_idx].first.transpose()
    //           << "heaing:" << start_heading * 57.3
    //           << "steer: " << static_cast<int>(tangent_pt1_steer) <<
    //           std::endl;

    const uint8_t tangent_pt2_steer =
        IsRightCircle(possible_tangent_pts[correct_pt_idx].second,
                      target_heading, possible_centers[correct_pt_idx])
            ? pnc::geometry_lib::SEG_STEER_RIGHT
            : pnc::geometry_lib::SEG_STEER_LEFT;

    // std::cout << "tangent_pt2: "
    //           << possible_tangent_pts[correct_pt_idx].second.transpose()
    //           << "heaing:" << target_heading * 57.3
    //           << "steer: " << static_cast<int>(tangent_pt2_steer) << "\n"
    //           << std::endl;

    if (steer == tangent_pt1_steer && steer == tangent_pt2_steer) {
      break;
    }
  }

  if (correct_pt_idx == possible_centers.size()) {
    std::cout << "check tangent pt direction failed" << std::endl;
    return false;
  } else {
    circle_index = correct_pt_idx;
    return true;
  }
}

const bool ParallelPathPlanner::SelectLineArcLineByDir(
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::PathPoint& target_pose,
    const std::pair<Eigen::Vector2d, Eigen::Vector2d>& tangent_pts,
    const pnc::geometry_lib::Circle& circle, const uint8_t ideal_dir,
    std::vector<PathSegment>& line_arc_line_segments) const {
  // first line step check
  const Eigen::Vector2d start_to_tangent_vec =
      tangent_pts.first - start_pose.pos;

  const Eigen::Vector2d start_heading_vec(std::cos(start_pose.heading),
                                          std::sin(start_pose.heading));

  const uint8_t first_line_dir =
      start_to_tangent_vec.dot(start_heading_vec) > 0.0
          ? pnc::geometry_lib::SEG_GEAR_DRIVE
          : pnc::geometry_lib::SEG_GEAR_REVERSE;

  if (first_line_dir != ideal_dir &&
      start_to_tangent_vec.norm() > kMaxIgnoredLength) {
    std::cout << "first line step direction is different than ideal one"
              << std::endl;
    return false;
  }

  // last line step check
  const Eigen::Vector2d tangent_to_target_vec =
      target_pose.pos - tangent_pts.second;

  const Eigen::Vector2d target_heading_vec(std::cos(target_pose.heading),
                                           std::sin(target_pose.heading));

  const uint8_t last_line_dir =
      tangent_to_target_vec.dot(target_heading_vec) > 0.0
          ? pnc::geometry_lib::SEG_GEAR_DRIVE
          : pnc::geometry_lib::SEG_GEAR_REVERSE;

  if (tangent_pts.second.x() < target_pose.pos.x() - kMaxIgnoredLength) {
    std::cout << "last line step direction is different than ideal one"
              << std::endl;
    return false;
  }

  // fill first line segment output
  pnc::geometry_lib::LineSegment first_line_seg(start_pose.pos,
                                                tangent_pts.first);
  first_line_seg.heading = start_pose.heading;
  first_line_seg.length = start_to_tangent_vec.norm();
  first_line_seg.is_ignored = false;
  if (first_line_dir == ideal_dir) {
    line_arc_line_segments.emplace_back(
        PathSegment(first_line_dir, first_line_seg));
  }

  // fill second arc segment output
  pnc::geometry_lib::Arc second_arc_seg;
  second_arc_seg.circle_info = circle;
  second_arc_seg.pA = tangent_pts.first;
  second_arc_seg.headingA = first_line_seg.heading;
  second_arc_seg.pB = tangent_pts.second;
  second_arc_seg.headingB = target_pose.heading;
  second_arc_seg.is_ignored = false;

  const auto theta_arc = pnc::geometry_lib::GetAngleFromTwoVec(
      second_arc_seg.pA - second_arc_seg.circle_info.center,
      second_arc_seg.pB - second_arc_seg.circle_info.center);

  second_arc_seg.is_anti_clockwise = (theta_arc > 0.0);
  second_arc_seg.length =
      std::fabs(theta_arc) * second_arc_seg.circle_info.radius;

  // fill last line segment output
  pnc::geometry_lib::LineSegment last_line_seg(tangent_pts.second,
                                               target_pose.pos);
  last_line_seg.heading = target_pose.heading;
  last_line_seg.length = tangent_to_target_vec.norm();
  last_line_seg.is_ignored = false;
  if (last_line_dir == ideal_dir) {
    line_arc_line_segments.emplace_back(
        PathSegment(last_line_dir, last_line_seg));
  } else {
    if (last_line_seg.length > kMaxIgnoredLength) {
      line_arc_line_segments.emplace_back(
          PathSegment(last_line_dir, last_line_seg));
    }
  }

  return true;
}

const bool ParallelPathPlanner::InverseTwoArcPlan(
    const pnc::geometry_lib::PathPoint& current_pose, const uint8_t direction,
    const uint8_t steer, const double radius, PathSegment& opt_arc_segment) {
  pnc::geometry_lib::Circle current_circle;
  current_circle.radius = radius;
  current_circle.center = CalEgoTurningCenter(
      current_pose.pos, current_pose.heading, radius, steer);

  std::vector<PathSegment> current_path_seg_vec;
  const bool success = CalInverseTwoArcGeometry(current_pose, direction, steer,
                                                current_path_seg_vec);

  std::cout << "current_path_seg_vec:" << std::endl;
  for (const auto& seg : current_path_seg_vec) {
    std::cout << "===========" << std::endl;
    PrintSegmentInfo(seg);
  }

  if (!success) {
    return false;
  }

  auto arc = current_path_seg_vec.front().arc_seg;

  if (CalArcGear(arc) != direction) {
    return false;
  }

  if (kP0P1CollisionDetectEnable) {
    std::cout << "--- collision" << std::endl;
    const auto result = collision_detector_ptr_->Update(arc, arc.headingA);
    const auto& remain_car_dist = result.remain_car_dist;
    const auto& remain_obstacle_dist = result.remain_obstacle_dist;

    const auto safe_remain_dist =
        std::min(remain_car_dist, remain_obstacle_dist - 0.3);

    std::cout << "remain_obstacle_dist = " << remain_obstacle_dist << std::endl;
    std::cout << "remain_car_dist = " << remain_car_dist << std::endl;
    std::cout << "safe_remain_dist of p0 = " << safe_remain_dist << std::endl;

    if (safe_remain_dist < 0.0) {
      std::cout << "safe_remain_dist is too small" << std::endl;
      return false;
    }

    const double sign = arc.is_anti_clockwise ? 1.0 : -1.0;
    const double d_theta = (safe_remain_dist) / arc.circle_info.radius * sign;
    const auto rotm = pnc::geometry_lib::GetRotm2dFromTheta(d_theta);
    const auto v_OA = arc.pA - arc.circle_info.center;
    const auto v_OC = rotm * v_OA;
    const auto pC = arc.circle_info.center + v_OC;

    pnc::geometry_lib::PathPoint limit_pose;
    limit_pose.Set(pC,
                   pnc::geometry_lib::NormalizeAngle(d_theta + arc.headingA));
    arc.pB = limit_pose.pos;
    arc.headingB = limit_pose.heading;
    arc.length = safe_remain_dist;
  }

  opt_arc_segment = PathSegment(steer, direction, arc);

  return true;
}

const bool ParallelPathPlanner::CalInverseTwoArcGeometry(
    const pnc::geometry_lib::PathPoint& start_pose, const uint8_t direction,
    const uint8_t steer, std::vector<PathSegment>& inverse_two_segmemts,
    DebugInfo& debuginfo) const {
  const double radius = kMinTurnRadius;
  double start_dir_sgn =
      (direction == pnc::geometry_lib::SEG_GEAR_DRIVE ? 1.0 : -1.0);

  if (steer == pnc::geometry_lib::SEG_STEER_STRAIGHT) {
    std::cout << "steer input error" << std::endl;
    return false;
  }

  pnc::geometry_lib::Circle start_circle;
  start_circle.radius = radius;
  start_circle.center = CalEgoTurningCenter(start_pose.pos, start_pose.heading,
                                            start_circle.radius, steer);

  std::cout << "start_circle.center = " << start_circle.center.transpose()
            << std::endl;

  Eigen::Vector2d target_line =
      (calc_params_.target_line.pB - calc_params_.target_line.pA).normalized();
  target_line *= target_line.x() > 0.0 ? 1.0 : -1.0;

  std::cout << "target_line = " << target_line.transpose() << std::endl;

  // start cal target center
  Eigen::Vector2d target_line_n(-target_line.y(), target_line.x());

  const Eigen::Vector2d start_center_to_target_vec =
      calc_params_.target_line.pA - start_circle.center;

  if (target_line_n.dot(start_center_to_target_vec) < 0.0) {
    target_line_n *= -1.0;
  }

  std::cout << "target_line_n = " << target_line_n.transpose() << std::endl;

  pnc::geometry_lib::LineSegment end_circle_center_line(
      calc_params_.target_line.pA + target_line_n * radius,
      calc_params_.target_line.pB + target_line_n * radius);

  pnc::geometry_lib::Circle tmp_circle = {start_circle.center, radius * 2.0};

  std::vector<Eigen::Vector2d> possible_end_centers;
  pnc::geometry_lib::CalcCrossPointsOfLineAndCircle(
      end_circle_center_line, tmp_circle, possible_end_centers);
  if (possible_end_centers.size() != 2) {
    return false;
  }

  Eigen::Vector2d end_center =
      (possible_end_centers.front().x() * start_dir_sgn <
               possible_end_centers.back().x() * start_dir_sgn
           ? possible_end_centers.front()
           : possible_end_centers.back());

  const Eigen::Vector2d center_vec =
      (start_circle.center - end_center).normalized();

  pnc::geometry_lib::Arc start_arc;
  start_arc.is_ignored = false;
  start_arc.circle_info = start_circle;
  start_arc.pA = start_pose.pos;
  start_arc.headingA = start_pose.heading;
  start_arc.pB = end_center + center_vec * radius;

  debuginfo.tag_point = start_arc.pB;
  const auto v_OA = start_arc.pA - start_circle.center;
  const auto v_OB = start_arc.pB - start_circle.center;
  const auto d_theta = pnc::geometry_lib::GetAngleFromTwoVec(v_OA, v_OB);

  start_arc.headingB =
      pnc::geometry_lib::NormalizeAngle(start_arc.headingA + d_theta);

  debuginfo.headingB = start_arc.headingB;

  const Eigen::Vector2d o1a_vec = start_arc.pA - start_arc.circle_info.center;
  const Eigen::Vector2d o1b_vec = start_arc.pB - start_arc.circle_info.center;
  const auto theta1 = pnc::geometry_lib::GetAngleFromTwoVec(o1a_vec, o1b_vec);
  start_arc.is_anti_clockwise = (theta1 > 0.0);
  start_arc.length = std::fabs(theta1) * radius;

  inverse_two_segmemts.emplace_back(PathSegment(steer, direction, start_arc));

  // second inverse arc
  const uint8_t end_steer = (steer == pnc::geometry_lib::SEG_STEER_RIGHT
                                 ? pnc::geometry_lib::SEG_STEER_LEFT
                                 : pnc::geometry_lib::SEG_STEER_RIGHT);
  const uint8_t end_direction = (direction == pnc::geometry_lib::SEG_GEAR_DRIVE
                                     ? pnc::geometry_lib::SEG_GEAR_REVERSE
                                     : pnc::geometry_lib::SEG_GEAR_DRIVE);

  pnc::geometry_lib::Arc end_arc;
  end_arc.is_ignored = false;
  end_arc.circle_info.center = end_center;
  end_arc.circle_info.radius = radius;
  end_arc.pA = start_arc.pB;
  end_arc.headingA = start_arc.headingB;

  end_arc.pB = end_arc.circle_info.center - target_line_n * radius;
  end_arc.headingB = std::atan2(target_line.y(), target_line.x());

  const Eigen::Vector2d o2a_vec = end_arc.pA - end_arc.circle_info.center;
  const Eigen::Vector2d o2b_vec = end_arc.pB - end_arc.circle_info.center;
  const auto theta2 = pnc::geometry_lib::GetAngleFromTwoVec(o2a_vec, o2b_vec);
  end_arc.is_anti_clockwise = (theta2 > 0.0);
  end_arc.length = std::fabs(theta2) * radius;

  inverse_two_segmemts.emplace_back(
      PathSegment(end_steer, end_direction, end_arc));

  return true;
}

const bool ParallelPathPlanner::CalInverseTwoArcGeometry(
    const pnc::geometry_lib::PathPoint& start_pose, const uint8_t direction,
    const uint8_t steer, std::vector<PathSegment>& inverse_two_segmemts) const {
  const double radius = kMinTurnRadius;
  double start_dir_sgn =
      (direction == pnc::geometry_lib::SEG_GEAR_DRIVE ? 1.0 : -1.0);

  if (steer == pnc::geometry_lib::SEG_STEER_STRAIGHT) {
    std::cout << "steer input error" << std::endl;
    return false;
  }

  pnc::geometry_lib::Circle start_circle;
  start_circle.radius = radius;
  start_circle.center = CalEgoTurningCenter(start_pose.pos, start_pose.heading,
                                            start_circle.radius, steer);

  std::cout << "start_circle.center = " << start_circle.center.transpose()
            << std::endl;

  Eigen::Vector2d target_line =
      (calc_params_.target_line.pB - calc_params_.target_line.pA).normalized();
  target_line *= target_line.x() > 0.0 ? 1.0 : -1.0;

  std::cout << "target_line = " << target_line.transpose() << std::endl;

  // start cal target center
  Eigen::Vector2d target_line_n(-target_line.y(), target_line.x());

  const Eigen::Vector2d start_center_to_target_vec =
      calc_params_.target_line.pA - start_circle.center;

  if (target_line_n.dot(start_center_to_target_vec) < 0.0) {
    target_line_n *= -1.0;
  }

  std::cout << "target_line_n = " << target_line_n.transpose() << std::endl;

  pnc::geometry_lib::LineSegment end_circle_center_line(
      calc_params_.target_line.pA + target_line_n * radius,
      calc_params_.target_line.pB + target_line_n * radius);

  pnc::geometry_lib::Circle tmp_circle = {start_circle.center, radius * 2.0};

  std::vector<Eigen::Vector2d> possible_end_centers;

  pnc::geometry_lib::CalcCrossPointsOfLineAndCircle(
      end_circle_center_line, tmp_circle, possible_end_centers);

  if (possible_end_centers.size() != 2) {
    std::cout << "possible_end_centers.size() = " << possible_end_centers.size()
              << std::endl;
    return false;
  }

  Eigen::Vector2d end_center =
      (possible_end_centers.front().x() * start_dir_sgn <
               possible_end_centers.back().x() * start_dir_sgn
           ? possible_end_centers.front()
           : possible_end_centers.back());

  const Eigen::Vector2d center_vec =
      (start_circle.center - end_center).normalized();

  pnc::geometry_lib::Arc start_arc;
  start_arc.is_ignored = false;
  start_arc.circle_info = start_circle;
  start_arc.pA = start_pose.pos;
  start_arc.headingA = start_pose.heading;
  start_arc.pB = end_center + center_vec * radius;

  const auto v_OA = start_arc.pA - start_circle.center;
  const auto v_OB = start_arc.pB - start_circle.center;
  const auto d_theta = pnc::geometry_lib::GetAngleFromTwoVec(v_OA, v_OB);

  start_arc.headingB =
      pnc::geometry_lib::NormalizeAngle(start_arc.headingA + d_theta);

  const Eigen::Vector2d o1a_vec = start_arc.pA - start_arc.circle_info.center;
  const Eigen::Vector2d o1b_vec = start_arc.pB - start_arc.circle_info.center;
  const auto theta1 = pnc::geometry_lib::GetAngleFromTwoVec(o1a_vec, o1b_vec);
  start_arc.is_anti_clockwise = (theta1 > 0.0);
  start_arc.length = std::fabs(theta1) * radius;

  inverse_two_segmemts.emplace_back(PathSegment(steer, direction, start_arc));

  // second inverse arc
  const uint8_t end_steer = (steer == pnc::geometry_lib::SEG_STEER_RIGHT
                                 ? pnc::geometry_lib::SEG_STEER_LEFT
                                 : pnc::geometry_lib::SEG_STEER_RIGHT);
  const uint8_t end_direction = (direction == pnc::geometry_lib::SEG_GEAR_DRIVE
                                     ? pnc::geometry_lib::SEG_GEAR_REVERSE
                                     : pnc::geometry_lib::SEG_GEAR_DRIVE);

  pnc::geometry_lib::Arc end_arc;
  end_arc.is_ignored = false;
  end_arc.circle_info.center = end_center;
  end_arc.circle_info.radius = radius;
  end_arc.pA = start_arc.pB;
  end_arc.headingA = start_arc.headingB;

  end_arc.pB = end_arc.circle_info.center - target_line_n * radius;
  end_arc.headingB = std::atan2(target_line.y(), target_line.x());

  const Eigen::Vector2d o2a_vec = end_arc.pA - end_arc.circle_info.center;
  const Eigen::Vector2d o2b_vec = end_arc.pB - end_arc.circle_info.center;
  const auto theta2 = pnc::geometry_lib::GetAngleFromTwoVec(o2a_vec, o2b_vec);
  end_arc.is_anti_clockwise = (theta2 > 0.0);
  end_arc.length = std::fabs(theta2) * radius;

  inverse_two_segmemts.emplace_back(
      PathSegment(end_steer, end_direction, end_arc));

  return true;
}

const bool ParallelPathPlanner::UpdateMultiStepMinSafeCircle() {
  // cal mulit-step safe circle
  const auto prepare_target_heading_vec =
      pnc::geometry_lib::GenHeadingVec(calc_params_.prepare_line.heading);

  Eigen::Vector2d ego_n_vec(prepare_target_heading_vec.y(),
                            -prepare_target_heading_vec.x());

  if (ego_n_vec.x() > 0.0) {
    ego_n_vec = -1.0 * ego_n_vec;
  }
  const auto ego_n_down_vec = ego_n_vec;

  // move down the start line
  const Eigen::Vector2d pt_s =
      calc_params_.prepare_line.pA + ego_n_down_vec * kMinTurnRadius;

  std::cout << "ego n vec " << ego_n_down_vec.transpose() << std::endl;

  pnc::geometry_lib::LineSegment line_sa(pt_s,
                                         pt_s + prepare_target_heading_vec);

  pnc::geometry_lib::Circle circle_p1 = {input_.tlane.p1,
                                         kMinTurnRadius - 0.5 * kVehicleWidth};

  // calc cross points of line and circle
  std::vector<Eigen::Vector2d> cross_points;
  const size_t cross_pt_nums =
      pnc::geometry_lib::CalcCrossPointsOfLineAndCircle(line_sa, circle_p1,
                                                        cross_points);
  if (cross_pt_nums != 2) {
    return false;
  }

  calc_params_.multi_safe_circle.center =
      (cross_points[0].y() * calc_params_.slot_side_sgn <
               cross_points[1].y() * calc_params_.slot_side_sgn
           ? cross_points[0]
           : cross_points[1]);

  std::cout << "multi-step safe circle"
            << calc_params_.multi_safe_circle.center.transpose() << std::endl;

  calc_params_.multi_safe_circle.radius = kMinTurnRadius;

  calc_params_.safe_circle_key_pt =
      calc_params_.multi_safe_circle.center - ego_n_down_vec * kMinTurnRadius;

  calc_params_.safe_circle_key_heading = input_.ego_pose.heading;

  std::cout << "safe circle key point: "
            << calc_params_.safe_circle_key_pt.transpose()
            << " , heading: " << calc_params_.safe_circle_key_heading
            << std::endl;

  return true;
}

void ParallelPathPlanner::CalMonoSafeCircle() {
  calc_params_.mono_safe_circle.center.y() =
      input_.tlane.pt.y() - kMinTurnRadius * calc_params_.slot_side_sgn;

  calc_params_.mono_safe_circle.radius = kMinTurnRadius;

  const double deta_x = std::sqrt(
      std::pow((kMinTurnRadius - kVehicleWidth * 0.5), 2) -
      std::pow((calc_params_.mono_safe_circle.center.y() - input_.tlane.p1.y()),
               2));

  calc_params_.mono_safe_circle.center.x() = input_.tlane.p1.x() - deta_x;

  // std::cout << "mono safe circle info: center = "
  //           << calc_params_.mono_safe_circle.center.transpose()
  //           << "   radius = " << calc_params_.mono_safe_circle.radius
  //           << std::endl;
}

const bool ParallelPathPlanner::CheckMonoIsFeasible() {
  const double dist = CalPoint2LineDist(calc_params_.mono_safe_circle.center,
                                        calc_params_.prepare_line);
  if (dist >= calc_params_.mono_safe_circle.radius) {
    // std::cout << "prepare_line is tangential or disjoint from mono safe "
    //              "circle, mono is feasible!\n";
    return true;
  } else {
    // std::cout << "prepare_line intersects circle, mono is not
    // feasible\n";
    return false;
  }
}

const bool ParallelPathPlanner::MonoPreparePlan(Eigen::Vector2d& tag_point) {
  CalMonoSafeCircle();

  if (CheckMonoIsFeasible() == false) {
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
  // std::cout << "point_tangent = " << tag_point.transpose() << std::endl;
  return true;
}

bool ParallelPathPlanner::CalMultiSafeCircle() {
  pnc::geometry_lib::Circle circle_p1;
  circle_p1.center = input_.tlane.p1;
  circle_p1.radius = kMinTurnRadius - 0.5 * kVehicleWidth;

  // move down the start line
  const Eigen::Vector2d pt_s =
      calc_params_.prepare_line.pA +
      calc_params_.pre_line_normal_vec * kMinTurnRadius;

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

  if (cross_points[0].y() * calc_params_.slot_side_sgn <
      cross_points[1].y() * calc_params_.slot_side_sgn) {
    multi_safe_circle.center = cross_points[0];
  } else {
    multi_safe_circle.center = cross_points[1];
  }

  multi_safe_circle.radius = kMinTurnRadius;

  // std::cout << "multi safa circle info: center = " <<
  // multi_safe_circle.center
  //           << "  radius = " << multi_safe_circle.radius << std::endl;

  return true;
}

const bool ParallelPathPlanner::MultiPreparePlan(Eigen::Vector2d& tag_point) {
  if (CalMultiSafeCircle() == false) {
    std::cout << "cal multistep safe circle fail!" << std::endl;
    return false;
  }

  tag_point =
      calc_params_.multi_safe_circle.center -
      calc_params_.pre_line_normal_vec * calc_params_.multi_safe_circle.radius;

  // std::cout << "point_tangent = " << tag_point.transpose() << std::endl;
  return true;
}

const bool ParallelPathPlanner::CheckMonoStepInSlot() {
  CalcMonoStepMinSafetyCircle();

  std::cout << "mono_safe_circle: "
            << calc_params_.mono_safe_circle.center.transpose()
            << " radius: " << calc_params_.mono_safe_circle.radius << std::endl;

  const pnc::geometry_lib::LineSegment& current_line =
      calc_params_.prepare_line;

  const double dist =
      CalPoint2LineDist(calc_params_.mono_safe_circle.center, current_line);

  std::cout << "dist of current line to circle center: " << dist << std::endl;
  std::cout << "circle radius: " << kMinTurnRadius << std::endl;

  if (dist >= kMinTurnRadius) {
    return true;
  } else {
    return false;
  }
}

const bool ParallelPathPlanner::IsSeenAsLine(
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::PathPoint& target_pose) const {
  return std::fabs(current_pose.pos.y() - target_pose.pos.y()) <=
             kMaxYCoordSeenAsLine &&
         std::fabs(pnc::geometry_lib::NormalizeAngle(current_pose.heading -
                                                     target_pose.heading)) <=
             kMaxHeadingSeenAsLine;
}

const bool ParallelPathPlanner::OneLineGeometryPlan(
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::LineSegment& target_line,
    PathSegment& line_seg) const {
  const bool is_line_success = IsSeenAsLine(
      current_pose,
      pnc::geometry_lib::PathPoint(target_line.pA, target_line.heading));

  if (is_line_success) {
    std::cout << "last line plan success!" << std::endl;
    pnc::geometry_lib::LineSegment line_step(current_pose.pos, target_line.pA);

    line_step.heading = current_pose.heading;
    line_step.length = (line_step.pA - line_step.pB).norm();

    line_step.is_ignored =
        line_step.length >= kMinLineStepLength ? false : true;

    if (!line_step.is_ignored) {
      uint8_t line_step_dir =
          line_step.pA.x() * calc_params_.slot_side_sgn >
                  line_step.pB.x() * calc_params_.slot_side_sgn
              ? pnc::geometry_lib::SEG_GEAR_REVERSE
              : pnc::geometry_lib::SEG_GEAR_DRIVE;

      line_seg = PathSegment(line_step_dir, line_step);
    }
  } else {
    std::cout << "last line plan failed " << std::endl;
  }

  return is_line_success;
}

const bool ParallelPathPlanner::OneArcGeometryPlan(
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::LineSegment& target_line, const double radius,
    const uint8_t steer, PathSegment& arc_segment) const {
  Eigen::Vector2d ego_heading_vec(std::cos(current_pose.heading),
                                  std::sin(current_pose.heading));

  Eigen::Vector2d ego_n_vec(ego_heading_vec.y(), -ego_heading_vec.x());
  const double steer_sgn =
      (steer == pnc::geometry_lib::SEG_STEER_RIGHT ? 1.0 : -1.0);
  ego_n_vec *= steer_sgn;

  Eigen::Vector2d center = current_pose.pos + ego_n_vec * radius;

  const double dist = pnc::geometry_lib::CalPoint2LineDist(center, target_line);

  if (std::fabs(dist - radius) > kRadiusEps) {
    std::cout << "dist is more than radius, one arc failed! " << dist
              << std::endl;

    return false;
  }

  pnc::geometry_lib::Arc arc;
  arc.circle_info.center = center;
  arc.circle_info.radius = radius;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;

  const Eigen::Vector2d center_to_target_vec = target_line.pA - center;
  const Eigen::Vector2d target_t_vec =
      (target_line.pB - target_line.pA).normalized();

  Eigen::Vector2d target_n_vec(target_t_vec.y(), -target_t_vec.x());

  if (target_n_vec.dot(center_to_target_vec) < 0.0) {
    target_n_vec *= -1.0;
  }

  arc.pB = target_n_vec * radius + center;

  std::cout << "\n*****One arc cal: " << std::endl;
  std::cout << "center " << center.transpose() << std::endl;
  std::cout << "target n vec " << target_n_vec.transpose() << std::endl;
  std::cout << "arc:pB " << arc.pB.transpose() << std::endl;
  arc.headingB = target_line.heading;
  const double theta =
      pnc::geometry_lib::GetAngleFromTwoVec(arc.pA - center, arc.pB - center);
  arc.is_anti_clockwise = (theta > 0.0);
  arc.length = std::fabs(theta) * radius;

  Eigen::Vector2d ab_vec = arc.pB - arc.pA;
  const uint8_t direction = ab_vec.dot(ego_heading_vec) < 0.0
                                ? pnc::geometry_lib::SEG_GEAR_REVERSE
                                : pnc::geometry_lib::SEG_GEAR_DRIVE;

  arc_segment = PathSegment(steer, direction, arc);
  std::cout << "one arc plan success" << std::endl;
  return true;
}

void ParallelPathPlanner::CalcMonoStepMinSafetyCircle() {
  // calc min safe circle
  calc_params_.mono_safe_circle.center.y() =
      input_.tlane.pt.y() - kMinTurnRadius * calc_params_.slot_side_sgn;

  calc_params_.mono_safe_circle.radius = kMinTurnRadius;

  const double deta_x = std::sqrt(
      std::pow((kMinTurnRadius - kVehicleWidth * 0.5), 2) -
      std::pow((calc_params_.mono_safe_circle.center.y() - input_.tlane.p1.y()),
               2));
  std::cout << "deta_x = " << deta_x << std::endl;

  calc_params_.mono_safe_circle.center.x() = input_.tlane.p1.x() - deta_x;
}

const bool ParallelPathPlanner::SetCurrentPathSegIndex() {
  if (!output_.path_available) {
    return false;
  }

  if (output_.gear_cmd_vec.size() < 1 || output_.path_segment_vec.size() < 1) {
    std::cout << "no path can get" << std::endl;
    return false;
  }

  if (output_.is_first_path == true) {
    // first get path segment from path_segment_vec, first and second index
    // is 0 at the moment
    output_.is_first_path = false;
  } else {
    if (output_.path_seg_index.second == output_.gear_cmd_vec.size() - 1) {
      std::cout << "no more path can get" << std::endl;
      return false;
    }
    output_.path_seg_index.first = output_.path_seg_index.second + 1;
  }

  if (output_.path_seg_index.first == output_.gear_cmd_vec.size() - 1) {
    output_.path_seg_index.second = output_.path_seg_index.first;
  }

  output_.current_gear = output_.gear_cmd_vec[output_.path_seg_index.first];

  for (size_t i = output_.path_seg_index.first + 1;
       i < output_.gear_cmd_vec.size(); ++i) {
    // gear change, break
    if (output_.gear_cmd_vec[i] != output_.current_gear) {
      output_.path_seg_index.second = i - 1;
      break;
    }
    // gear always no change
    if (i == output_.gear_cmd_vec.size() - 1) {
      output_.path_seg_index.second = i;
    }
  }

  for (size_t i = output_.path_seg_index.second;
       i >= output_.path_seg_index.first; --i) {
    if (output_.path_segment_vec[i].seg_type ==
        pnc::geometry_lib::SEG_TYPE_ARC) {
      output_.current_arc_steer = output_.steer_vec[i];
      break;
    }
  }

  if (output_.path_seg_index.second == output_.gear_cmd_vec.size() - 1) {
    std::cout << "current path is final path\n";
    output_.is_last_path = true;
  }

  // std::cout << "before insert\n";
  // std::cout << "output_.segment_type_vec = [  ";
  // for (size_t i = 0; i < output_.path_segment_vec.size(); ++i) {
  //   std::cout << static_cast<int>(output_.path_segment_vec[i].seg_type) << "
  //   ";
  // }
  // std::cout << "]\noutput_.steer_cmd_vec = [  ";
  // for (size_t i = 0; i < output_.steer_vec.size(); ++i) {
  //   std::cout << static_cast<int>(output_.steer_vec[i]) << "  ";
  // }
  // std::cout << "]\noutput_.gear_cmd_vec = [  ";
  // for (size_t i = 0; i < output_.gear_cmd_vec.size(); ++i) {
  //   std::cout << static_cast<int>(output_.gear_cmd_vec[i]) << "  ";
  // }
  // std::cout << "]\n current_gear = " <<
  // static_cast<int>(output_.current_gear)
  //           << "   current_arc_steer = "
  //           << static_cast<int>(output_.current_arc_steer) << std::endl;
  // std::cout << "current send path: first index = "
  //           << static_cast<int>(output_.path_seg_index.first)
  //           << "   second index = "
  //           << static_cast<int>(output_.path_seg_index.second) << std::endl;

  return true;
}

void ParallelPathPlanner::SetLineSegmentHeading() {
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

void ParallelPathPlanner::InsertLineSegAfterCurrentFollowLastPath(
    double extend_distance) {
  if (pnc::mathlib::IsDoubleEqual(extend_distance, 0.0)) {
    return;
  }
  if (output_.is_last_path == true) {
    std::cout << "is last path, not extend path\n";
    return;
  }

  if (output_.path_segment_vec.size() < 1) {
    return;
  }

  auto& path_seg = output_.path_segment_vec[output_.path_seg_index.second];

  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
      extend_distance < 0.0) {
    std::cout << "arc can not shorten\n";
    return;
  }

  if (extend_distance > 0.0) {
    PathSegment new_line;
    new_line.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;

    if (path_seg.Getlength() + extend_distance < kMinOneStepPathLength) {
      extend_distance = kMinOneStepPathLength - path_seg.Getlength();
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

    CollisionDetector::CollisionResult collision_result;
    collision_result = collision_detector_ptr_->Update(
        new_line.line_seg, new_line.line_seg.heading);

    const auto& remain_car_dist = collision_result.remain_car_dist;
    const auto& remain_obstacle_dist = collision_result.remain_obstacle_dist;

    const auto safe_remain_dist =
        std::min(remain_car_dist, remain_obstacle_dist - 0.3);
    // std::cout << "remain_car_dist = " << remain_car_dist
    //           << "   remain_obstacle_dist = " << remain_obstacle_dist
    //           << "  safe_remain_dist = " << safe_remain_dist <<
    //           std::endl;
    if (safe_remain_dist > 0.0) {
      if (pnc::mathlib::IsDoubleEqual(safe_remain_dist, remain_car_dist) ==
          false) {
        new_line.line_seg.length = safe_remain_dist;
        Eigen::Vector2d AB = new_line.line_seg.pB - new_line.line_seg.pA;
        Eigen::Vector2d AB_unit = AB.normalized();
        AB = safe_remain_dist * AB_unit;
        new_line.line_seg.pB = AB + new_line.line_seg.pA;
      }

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

      std::cout << "inset line segment successful\n";

      // std::cout << "output_.segment_type_vec = [  ";
      // for (size_t i = 0; i < output_.path_segment_vec.size(); ++i) {
      //   std::cout <<
      //   static_cast<int>(output_.path_segment_vec[i].seg_type)
      //             << "  ";
      // }
      // std::cout << "]\noutput_.steer_cmd_vec = [  ";
      // for (size_t i = 0; i < output_.steer_vec.size(); ++i) {
      //   std::cout << static_cast<int>(output_.steer_vec[i]) << "  ";
      // }
      // std::cout << "]\noutput_.gear_cmd_vec = [  ";
      // for (size_t i = 0; i < output_.gear_cmd_vec.size(); ++i) {
      //   std::cout << static_cast<int>(output_.gear_cmd_vec[i]) << "  ";
      // }
      // std::cout << "]\n current_gear = "
      //           << static_cast<int>(output_.current_gear)
      //           << "   current_arc_steer = "
      //           << static_cast<int>(output_.current_arc_steer) <<
      //           std::endl;
      // std::cout << "current send path: first index = "
      //           << static_cast<int>(output_.path_seg_index.first)
      //           << "   second index = "
      //           << static_cast<int>(output_.path_seg_index.second) <<
      //           std::endl;

    } else {
      std::cout << "safe_remain_dist < 0.0, can not inset line segment\n";
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

void ParallelPathPlanner::ExtendCurrentFollowLastPath(double extend_distance) {
  if (pnc::mathlib::IsDoubleEqual(extend_distance, 0.0)) {
    return;
  }
  if (output_.is_last_path == true) {
    std::cout << "is last path, not extend path\n";
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
    std::cout << "--- extend distance collision --- " << std::endl;
    CollisionDetector::CollisionResult collision_result;
    if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      collision_result = collision_detector_ptr_->Update(
          path_seg.line_seg, path_seg.line_seg.heading);
    } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      collision_result = collision_detector_ptr_->Update(
          path_seg.arc_seg, path_seg.arc_seg.headingA);
    }
    const auto& remain_car_dist = collision_result.remain_car_dist;
    const auto& remain_obstacle_dist = collision_result.remain_obstacle_dist;
    const auto safe_remain_dist =
        std::min(remain_car_dist, remain_obstacle_dist - 0.3);
    // std::cout << "remain_car_dist = " << remain_car_dist
    //           << "   remain_obstacle_dist = " << remain_obstacle_dist
    //           << "  safe_remain_dist = " << safe_remain_dist <<
    //           std::endl;
    if (pnc::mathlib::IsDoubleEqual(safe_remain_dist, remain_car_dist) ||
        safe_remain_dist < 0.0) {
      return;
    }
    if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      path_seg.line_seg.length = safe_remain_dist;
      Eigen::Vector2d AB = path_seg.line_seg.pB - path_seg.line_seg.pA;
      Eigen::Vector2d AB_unit = AB.normalized();
      AB = safe_remain_dist * AB_unit;
      path_seg.line_seg.pB = AB + path_seg.line_seg.pA;
    } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      path_seg.arc_seg.length = safe_remain_dist;
      double theta = safe_remain_dist / path_seg.arc_seg.circle_info.radius;
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
  }
}

const bool ParallelPathPlanner::SampleCurrentPathSeg() {
  if (!output_.path_available) {
    return false;
  }

  if (input_.is_complete_path == true) {
    // for simulation
    output_.path_seg_index.first = 0;
    output_.path_seg_index.second = output_.gear_cmd_vec.size() - 1;
  }

  output_.path_point_vec.clear();
  output_.path_point_vec.reserve(kReservedOutputPathPointSize);

  pnc::geometry_lib::PathPoint path_point;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
    const auto& current_seg = output_.path_segment_vec[i];

    if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      SampleLineSegment(current_seg.line_seg, input_.sample_ds);
    } else {
      SampleArcSegment(current_seg.arc_seg, input_.sample_ds);
    }
    if (i < output_.path_seg_index.second) {
      output_.path_point_vec.pop_back();
    }
  }

  return true;
}

void ParallelPathPlanner::SampleLineSegment(
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

  // get last point
  path_point.Set(cur_line_seg.pB, cur_line_seg.heading);
  output_.path_point_vec.emplace_back(path_point);
}

void ParallelPathPlanner::SampleArcSegment(
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

  // get last point
  path_point.Set(current_arc_seg.pB,
                 pnc::geometry_lib::NormalizeAngle(current_arc_seg.headingB));

  output_.path_point_vec.emplace_back(path_point);
}

void ParallelPathPlanner::PrintSegmentInfo(const PathSegment& seg) const {
  std::cout << "----" << std::endl;
  std::cout << "seg_direction: " << static_cast<int>(seg.seg_direction)
            << std::endl;

  std::cout << "seg_steer: " << static_cast<int>(seg.seg_steer) << std::endl;
  std::cout << "seg_type: " << static_cast<int>(seg.seg_type) << std::endl;
  std::cout << "length: " << seg.Getlength() << std::endl;

  if (seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    std::cout << "start_pos: " << seg.GetLineSeg().pA.transpose() << std::endl;
    std::cout << "start_heading: " << seg.GetLineSeg().heading * 57.3
              << std::endl;
    std::cout << "end_pos: " << seg.GetLineSeg().pB.transpose() << std::endl;
    std::cout << "end_heading: " << seg.GetLineSeg().heading * 57.3
              << std::endl;
  } else {
    std::cout << "start_pos: " << seg.GetArcSeg().pA.transpose() << std::endl;
    std::cout << "start_heading: " << seg.GetArcSeg().headingA * 57.3
              << std::endl;
    std::cout << "end_pos: " << seg.GetArcSeg().pB.transpose() << std::endl;
    std::cout << "end_heading: " << seg.GetArcSeg().headingB * 57.3
              << std::endl;
  }
}

void ParallelPathPlanner::PrintOutputSegmentsInfo() const {
  std::cout << "-------------- OutputSegmentsInfo --------------" << std::endl;
  for (size_t i = 0; i < output_.path_segment_vec.size(); i++) {
    const auto& current_seg = output_.path_segment_vec[i];

    if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      const auto& line_seg = current_seg.line_seg;

      std::cout << "Segment [" << i << "] "
                << " LINE_SEGMENT "
                << " length= " << line_seg.length << std::endl;

      std::cout << "seg_direction: "
                << static_cast<int>(current_seg.seg_direction) << std::endl;

      std::cout << "seg_steer: " << static_cast<int>(current_seg.seg_steer)
                << std::endl;

      std::cout << "start_pos: " << line_seg.pA.transpose() << std::endl;
      std::cout << "start_heading: " << line_seg.heading << std::endl;
      std::cout << "end_pos: " << line_seg.pB.transpose() << std::endl;
    } else {
      const auto& arc_seg = current_seg.arc_seg;

      std::cout << "Segment [" << i << "] "
                << "ARC_SEGMENT "
                << "length= " << arc_seg.length << std::endl;

      std::cout << "seg_direction: "
                << static_cast<int>(current_seg.seg_direction) << std::endl;

      std::cout << "seg_steer: " << static_cast<int>(current_seg.seg_steer)
                << std::endl;

      std::cout << "start_pos: " << arc_seg.pA.transpose() << std::endl;
      std::cout << "start_heading: " << arc_seg.headingA << std::endl;
      std::cout << "end_pos: " << arc_seg.pB.transpose() << std::endl;
      std::cout << "end_heading: " << arc_seg.headingB << std::endl;
      std::cout << "center: " << arc_seg.circle_info.center.transpose()
                << "radius = " << arc_seg.circle_info.radius << std::endl;
    }
  }
}

const std::vector<double> ParallelPathPlanner::GetPathEle(size_t index) const {
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

const std::vector<double> ParallelPathPlanner::GetMinSafeCircle() const {
  return std::vector<double>{calc_params_.mono_safe_circle.center.x(),
                             calc_params_.mono_safe_circle.center.y(),
                             calc_params_.mono_safe_circle.radius};
}

const bool ParallelPathPlanner::CheckArcCollided(
    const pnc::geometry_lib::Arc& arc_step) const {
  pnc::geometry_lib::LocalToGlobalTf ego2slot_tf;

  ego2slot_tf.Init(arc_step.pA, arc_step.headingA);
  Eigen::Vector2d fl_corner_pA = ego2slot_tf.GetPos(calc_params_.fl_corner_vec);
  Eigen::Vector2d fr_corner_pA = ego2slot_tf.GetPos(calc_params_.fr_corner_vec);

  ego2slot_tf.Init(arc_step.pB, arc_step.headingB);
  Eigen::Vector2d fl_corner_pB = ego2slot_tf.GetPos(calc_params_.fl_corner_vec);
  Eigen::Vector2d fr_corner_pB = ego2slot_tf.GetPos(calc_params_.fr_corner_vec);

  pnc::geometry_lib::Arc fl_arc;
  fl_arc.pA = fl_corner_pA;
  fl_arc.pB = fl_corner_pB;
  fl_arc.circle_info.center = arc_step.circle_info.center;
  fl_arc.circle_info.radius = (fl_corner_pA - fl_arc.circle_info.center).norm();

  pnc::geometry_lib::Arc fr_arc;
  fr_arc.pA = fr_corner_pA;
  fr_arc.pB = fr_corner_pB;
  fr_arc.circle_info.center = arc_step.circle_info.center;
  fr_arc.circle_info.radius = (fr_corner_pA - fr_arc.circle_info.center).norm();

  pnc::geometry_lib::LineSegment channel_line(
      Eigen::Vector2d(input_.tlane.channel_x, -10.0),
      Eigen::Vector2d(input_.tlane.channel_x, 10.0));

  Eigen::Vector2d intersection;
  return pnc::geometry_lib::GetArcLineIntersection(intersection, fl_arc,
                                                   channel_line) ||
         pnc::geometry_lib::GetArcLineIntersection(intersection, fr_arc,
                                                   channel_line);
}

const Eigen::Vector2d ParallelPathPlanner::CalEgoTurningCenter(
    const Eigen::Vector2d& ego_pos, const double ego_heading,
    const double radius, const uint8_t steer) const {
  Eigen::Vector2d ego_heading_vec(std::cos(ego_heading), std::sin(ego_heading));

  Eigen::Vector2d ego_n_vec(-ego_heading_vec.y(), ego_heading_vec.x());
  if (steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
    ego_n_vec *= -1.0;
  }
  return ego_pos + ego_n_vec * radius;
}

const bool ParallelPathPlanner::IsRightCircle(
    const pnc::geometry_lib::PathPoint& ego_pose,
    const Eigen::Vector2d& center) const {
  return IsRightCircle(ego_pose.pos, ego_pose.heading, center);
}

const bool ParallelPathPlanner::IsRightCircle(
    const Eigen::Vector2d& ego_pos, const double ego_heading,
    const Eigen::Vector2d& center) const {
  const Eigen::Vector2d center_to_ego_vec = ego_pos - center;

  const Eigen::Vector2d ego_heading_vec(std::cos(ego_heading),
                                        std::sin(ego_heading));

  return pnc::geometry_lib::GetCrossFromTwoVec2d(ego_heading_vec,
                                                 center_to_ego_vec);
}

const bool ParallelPathPlanner::CheckTwoPoseInCircle(
    const Eigen::Vector2d& ego_pos0, const double ego_heading0,
    const Eigen::Vector2d& ego_pos1, const double ego_heading1,
    const Eigen::Vector2d& center) const {
  return IsRightCircle(ego_pos0, ego_heading0, center) &&
         IsRightCircle(ego_pos1, ego_heading1, center);
}

pnc::geometry_lib::LineSegment ParallelPathPlanner::ConstructEgoHeadingLine(
    const Eigen::Vector2d& ego_pos, const double ego_heading) const {
  const Eigen::Vector2d next_pos =
      ego_pos + Eigen::Vector2d(std::cos(ego_heading), std::sin(ego_heading));

  return pnc::geometry_lib::LineSegment(ego_pos, next_pos);
}

}  // namespace apa_planner
}  // namespace planning