#include "perpendicular_path_planner.h"

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
#include "dubins_lib.h"
#include "geometry_math.h"
#include "math_lib.h"
namespace planning {
namespace apa_planner {
static const double kMaxYCoordSeenAsLine = 0.03;
static const double kMaxHeadingSeenAsLine = 3.0 / 57.3;
static const size_t kMaxPerpenParkInSegmentNums = 15;
static const size_t kReservedOutputPathPointSize = 750;
static const size_t kMaxPathNumsInSlot = 6;
static const bool kP0P1CollisionDetectEnable = true;
static const bool kMonoPlanEnable = false;

static const std::vector<double> kPrepareTargetLineDeltaHeadingTab = {
    10.0 / 57.3, 12.0 / 57.3, 8.0 / 57.3};

static const std::vector<Eigen::Vector2d> kPrepareTargetLinePosTab = {
    Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(0.0, 0.0)};

void PerpendicularPathPlanner::Reset() {
  output_.Reset();
  output_.path_segment_vec.reserve(kMaxPerpenParkInSegmentNums);
  output_.gear_cmd_vec.reserve(kMaxPerpenParkInSegmentNums);

  calc_params_.Reset();
}

void PerpendicularPathPlanner::Preprocess() {
  calc_params_.Reset();

  // calc slot side by Tlane
  if (input_.tlane.pt_inside.y() > input_.tlane.pt_outside.y()) {
    calc_params_.is_left_side = true;
    calc_params_.slot_side_sgn = 1.0;
  } else {
    calc_params_.is_left_side = false;
    calc_params_.slot_side_sgn = -1.0;
  }

  // target line
  calc_params_.target_line =
      pnc::geometry_lib::BuildLineSegByPose(input_.tlane.pt_terminal, 0.0);
}

bool PerpendicularPathPlanner::Update() {
  std::cout << "-------- path planner --------" << std::endl;

  // preprocess
  Preprocess();

  // reset output
  output_.Reset();

  // prepare plan, only for first plan
  if (input_.is_replan_first) {
    if (PreparePlan() == false) {
      std::cout << "prepare plan failed!" << std::endl;
      return false;
    }
    if (GenPathOutputByDubins() == false) {
      std::cout << "prepare gear is drive, no need multi plan" << std::endl;
      return true;
    }
    std::cout << "prepare gear is reverse, need multi plan" << std::endl;
  }

  // multi step
  if (MultiPlan()) {
    std::cout << "multi plan success!" << std::endl;
    return true;
  }

  // adjust step
  if (AdjustPlan()) {
    std::cout << "adjust plan success!" << std::endl;
    return true;
  }

  return false;
}

bool PerpendicularPathPlanner::Update(
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  collision_detector_ptr_ = collision_detector_ptr;
  return Update();
}

// prepare plan start
const bool PerpendicularPathPlanner::PreparePlan() {
  std::vector<double> x_offset_vec;
  std::vector<double> heading_offset_vec;

  double x_offset = apa_param.GetParam().prepare_line_x_offset_slot;
  while (x_offset < apa_param.GetParam().prepare_line_x_offset_slot + 2.5) {
    x_offset_vec.emplace_back(x_offset);
    x_offset += 0.1;
  }

  double heading_offset =
      apa_param.GetParam().prepare_line_heading_offset_slot_deg / 57.3;
  while (heading_offset >= 0.0) {
    heading_offset_vec.emplace_back(heading_offset);
    heading_offset -= 1.0 / 57.3;
  }

  for (const auto& heading_offset : heading_offset_vec) {
    for (const auto& x_offset : x_offset_vec) {
      if (PreparePlanOnce(x_offset, heading_offset)) {
        std::cout << "x_offset = " << x_offset << std::endl;
        std::cout << "heading_offset = " << heading_offset * 57.3 << std::endl;
        return true;
      }
    }
  }

  return false;
}

const bool PerpendicularPathPlanner::PreparePlanOnce(
    const double x_offset, const double heading_offset) {
  double start_heading =
      calc_params_.slot_side_sgn * (90.0 / 57.3 - heading_offset);

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
  input.radius = apa_param.GetParam().min_turn_radius * 1.05;

  bool prepare_success = false;
  // first use mono prepare to find target point
  if (MonoPreparePlan(target_pose.pos) && kMonoPlanEnable) {
    input.Set(input_.ego_pose.pos, target_pose.pos, input_.ego_pose.heading,
              target_pose.heading);
    dubins_planner_.SetInput(input);
    prepare_success = dubins_planner_.OneStepDubinsUpdate();
  }

  if (prepare_success) {
    std::cout << "use mono prepare to find target point successful!\n";
    return true;
  }
  // std::cout << "use mono prepare to find target point failed!\n";

  // if mono prepare fail, use multi prepare to find target point
  if (MultiPreparePlan(target_pose.pos)) {
    input.Set(input_.ego_pose.pos, target_pose.pos, input_.ego_pose.heading,
              target_pose.heading);
    dubins_planner_.SetInput(input);
    prepare_success = dubins_planner_.OneStepDubinsUpdate();
  }

  if (prepare_success) {
    std::cout << "prepare find target point multi successful\n";
    return true;
  }
  // std::cout << "prepare find target point multi failed\n";

  return false;
}

void PerpendicularPathPlanner::CalMonoSafeCircle() {
  calc_params_.mono_safe_circle.center.y() =
      input_.tlane.pt_terminal.y() +
      apa_param.GetParam().min_turn_radius * calc_params_.slot_side_sgn;

  calc_params_.mono_safe_circle.radius = apa_param.GetParam().min_turn_radius;

  const double deta_x =
      std::sqrt(std::pow((apa_param.GetParam().min_turn_radius -
                          apa_param.GetParam().car_width * 0.5),
                         2) -
                std::pow((calc_params_.mono_safe_circle.center.y() -
                          input_.tlane.pt_inside.y()),
                         2));

  calc_params_.mono_safe_circle.center.x() =
      input_.tlane.pt_inside.x() - deta_x;

  // std::cout << "mono safe circle info: center = "
  //           << calc_params_.mono_safe_circle.center.transpose()
  //           << "   radius = " << calc_params_.mono_safe_circle.radius
  //           << std::endl;
}

const bool PerpendicularPathPlanner::CheckMonoIsFeasible() {
  const double dist = CalPoint2LineDist(calc_params_.mono_safe_circle.center,
                                        calc_params_.prepare_line);
  if (dist >= calc_params_.mono_safe_circle.radius) {
    // std::cout << "prepare_line is tangential or disjoint from mono safe "
    //              "circle, mono is feasible!\n";
    return true;
  } else {
    // std::cout << "prepare_line intersects circle, mono is not feasible\n";
    return false;
  }
}

const bool PerpendicularPathPlanner::MonoPreparePlan(
    Eigen::Vector2d& tag_point) {
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

bool PerpendicularPathPlanner::CalMultiSafeCircle() {
  pnc::geometry_lib::Circle circle_p1;
  circle_p1.center = input_.tlane.pt_inside;
  circle_p1.radius = apa_param.GetParam().min_turn_radius -
                     0.5 * apa_param.GetParam().car_width;

  // move down the start line
  const Eigen::Vector2d pt_s =
      calc_params_.prepare_line.pA +
      calc_params_.pre_line_normal_vec * apa_param.GetParam().min_turn_radius;

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

  multi_safe_circle.radius = apa_param.GetParam().min_turn_radius;

  // std::cout << "multi safa circle info: center = " <<
  // multi_safe_circle.center
  //           << "  radius = " << multi_safe_circle.radius << std::endl;

  return true;
}

const bool PerpendicularPathPlanner::MultiPreparePlan(
    Eigen::Vector2d& tag_point) {
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

  std::cout << "prepare plan output:\n";
  for (size_t i = 0; i < output_.path_segment_vec.size(); ++i) {
    auto& path_seg = output_.path_segment_vec[i];
    std::cout << i << "th path seg info:\n";
    std::cout << "type = " << static_cast<int>(path_seg.seg_type)
              << "  length = " << path_seg.Getlength()
              << "  gear = " << static_cast<int>(path_seg.seg_gear)
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
// prepare plan end

// multi plan start
const bool PerpendicularPathPlanner::MultiPlan() {
  std::cout << "-----multi plan-----\n";
  // set init state
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;

  // check pose and slot_occupied_ratio, if error is small, multi isnot suitable
  if ((std::fabs(current_pose.pos.y()) <= 0.5 &&
       std::fabs(current_pose.heading) <= 12.0 / 57.3) ||
      input_.slot_occupied_ratio >= 0.8) {
    std::cout << "pose err is relatively small, skip multi plan, directly try "
                 "adjust plan\n";
    return false;
  }

  // check gear and steer
  if ((current_gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
       current_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) ||
      (current_arc_steer != pnc::geometry_lib::SEG_STEER_LEFT &&
       current_arc_steer != pnc::geometry_lib::SEG_STEER_RIGHT)) {
    std::cout << "ref_gear or ref_arc_steer error\n";
    return false;
  }

  std::cout << "try multi plan to target point\n";
  bool success = false;
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  for (size_t i = 0; i < kMaxPathNumsInSlot; ++i) {
    std::cout << "-------- No." << i << " in multi-plan--------\n";
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);
    if (CalSinglePathInMulti(current_pose, current_gear, current_arc_steer,
                             tmp_path_seg_vec)) {
      output_.path_available = true;
      success = true;
    } else {
      std::cout << "single path of multi-plan failed!\n\n";
      output_.Reset();
      success = false;
      break;
    }

    for (const auto& tmp_path_seg : tmp_path_seg_vec) {
      output_.path_segment_vec.emplace_back(tmp_path_seg);
      output_.length += tmp_path_seg.Getlength();
      output_.gear_cmd_vec.emplace_back(tmp_path_seg.seg_gear);
      output_.steer_vec.emplace_back(tmp_path_seg.seg_steer);
    }

    if (output_.path_segment_vec.size() > 0) {
      const auto& last_segment = output_.path_segment_vec.back();
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

    if ((current_pose.pos - input_.tlane.pt_terminal).norm() <=
            apa_param.GetParam().static_pos_eps &&
        std::fabs(current_pose.heading - calc_params_.target_line.heading) <=
            apa_param.GetParam().static_heading_eps / 57.3) {
      std::cout << "already plan to target pos!\n\n";
      break;
    }
  }
  if (!success) {
    std::cout << "multi plan failed!" << std::endl;
  }
  return success;
}

const bool PerpendicularPathPlanner::CalSinglePathInMulti(
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const uint8_t current_arc_steer,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  std::cout << "-----CalSinglePathInMulti-----\n";
  std::cout << "current_arc_steer = " << static_cast<int>(current_arc_steer)
            << ",  current_gear = " << static_cast<int>(current_gear)
            << ",  current_pos = " << current_pose.pos.transpose()
            << ",  current_heading = " << current_pose.heading * 57.3
            << std::endl;

  const double current_turn_radius = 1.0 * apa_param.GetParam().min_turn_radius;

  const Eigen::Vector2d current_tang_vec =
      pnc::geometry_lib::GetUnitTangVecByHeading(current_pose.heading);

  Eigen::Vector2d current_norm_vec;
  if (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
    current_norm_vec << current_tang_vec.y(), -current_tang_vec.x();
  } else if (current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
    current_norm_vec << -current_tang_vec.y(), current_tang_vec.x();
  }

  const Eigen::Vector2d current_turn_center =
      current_pose.pos + current_norm_vec * current_turn_radius;

  pnc::geometry_lib::Arc current_arc;
  current_arc.circle_info.center = current_turn_center;
  current_arc.circle_info.radius = current_turn_radius;
  current_arc.pA = current_pose.pos;
  current_arc.headingA =
      pnc::geometry_lib::NormalizeAngle(current_pose.heading);

  // cal the dist from current_turn_center to target line
  const double dist = pnc::geometry_lib::CalPoint2LineDist(
      current_turn_center, calc_params_.target_line);

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(2);
  bool success = false;
  // determine the relationship between the turning circle of the car and
  // the target line
  if ((dist <= current_turn_radius + apa_param.GetParam().radius_eps) &&
      (dist >= current_turn_radius - apa_param.GetParam().radius_eps)) {
    // circle and line are tangent, try first
    std::cout << "center to line dist = " << dist << ",  try OneArcPlan\n";
    if (OneArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                   current_arc_steer)) {
      std::cout << "OneArcPlan success\n";
      success = true;
    } else {
      std::cout << "OneArcPlan fail\n";
      success = false;
    }
  } else if (dist < current_turn_radius - apa_param.GetParam().radius_eps) {
    // circle and line are intersected, try second
    std::cout << "center to line dist = " << dist << ",  try TwoArcPlan\n";
    if (TwoArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                   current_arc_steer)) {
      std::cout << "TwoArcPlan success\n";
      success = true;
    } else {
      std::cout << "TwoArcPlan fail\n";
      success = false;
    }
  } else if (dist > current_turn_radius + apa_param.GetParam().radius_eps) {
    // circle and line are disjoint, try last
    std::cout << "center to line dist = " << dist << ",  try LineArcPlan\n";
    if (LineArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                    current_arc_steer)) {
      std::cout << "LineArcPlan success\n";
      success = true;
    } else {
      std::cout << "LineArcPlan fail\n";
      success = false;
    }
  }

  if (!success) {
    tmp_path_seg_vec.clear();
  }

  // try one line plan
  pnc::geometry_lib::PathPoint last_pose;
  if (tmp_path_seg_vec.size() > 0) {
    const auto& last_segment = tmp_path_seg_vec.back();
    last_pose.Set(last_segment.GetArcSeg().pB,
                  last_segment.GetArcSeg().headingB);
    std::cout << "last path pose to one plan\n";
  } else {
    last_pose = current_pose;
    std::cout << "current pose to one plan\n";
  }
  if ((last_pose.pos - input_.tlane.pt_terminal).norm() <=
          apa_param.GetParam().static_pos_eps &&
      std::fabs(last_pose.heading - calc_params_.target_line.heading) <=
          apa_param.GetParam().static_heading_eps / 57.3) {
    std::cout << "already plan to target pos, no need to one line plan!\n";
  } else {
    // try line
    pnc::geometry_lib::LineSegment last_line;
    last_line.pA = last_pose.pos;
    last_line.heading = last_pose.heading;
    if (OneLinePlan(last_line, tmp_path_seg_vec, input_.ref_gear)) {
      std::cout << "OneLinePlan success\n";
    } else {
      std::cout << "OneLinePlan fail\n";
    }
  }

  std::cout << "tmp_path_seg_vec:" << std::endl;
  for (const auto& tmp_path_seg : tmp_path_seg_vec) {
    PrintSegmentInfo(tmp_path_seg);
  }

  for (auto& tmp_path_seg : tmp_path_seg_vec) {
    const uint8_t path_col_det_res = TrimPathByCollisionDetection(tmp_path_seg);

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
    std::cout << "CalSinglePathInMulti success\n";
    std::cout << "cur_path_seg_vec:\n";
    double length = 0.0;
    for (const auto& path_seg : path_seg_vec) {
      PrintSegmentInfo(path_seg);
      length += path_seg.Getlength();
    }
    if (length < 0.25) {
      std::cout << "this gear path is too small, lose it\n";
      path_seg_vec.clear();
    }
    return true;
  } else {
    std::cout << "CalSinglePathInMulti fail\n";
    return false;
  }
}

const bool PerpendicularPathPlanner::OneArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear, const uint8_t current_arc_steer) {
  if (!CalOneArcWithLine(arc, calc_params_.target_line,
                         apa_param.GetParam().radius_eps)) {
    std::cout << "OneArcPlan fail 0\n";
    return false;
  }
  uint8_t steer = pnc::geometry_lib::CalArcSteer(arc);
  uint8_t gear = pnc::geometry_lib::CalArcGear(arc);
  if (steer != current_arc_steer || gear != current_gear) {
    std::cout << "OneArcPlan fail 1\n";
    return false;
  }
  pnc::geometry_lib::PathSegment arc_seg(steer, gear, arc);
  path_seg_vec.emplace_back(arc_seg);
  return true;
}

const bool PerpendicularPathPlanner::TwoArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear, const uint8_t current_arc_steer) {
  pnc::geometry_lib::Arc arc2;
  if (!CalTwoArcWithLine(arc, arc2, calc_params_.target_line)) {
    std::cout << "TwoArcPlan fail 0\n";
    return false;
  }

  uint8_t next_arc_steer;
  uint8_t next_gear;

  next_arc_steer = (current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT)
                       ? pnc::geometry_lib::SEG_STEER_RIGHT
                       : pnc::geometry_lib::SEG_STEER_LEFT;

  next_gear = (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE)
                  ? pnc::geometry_lib::SEG_GEAR_REVERSE
                  : pnc::geometry_lib::SEG_GEAR_DRIVE;

  uint8_t steer_1 = pnc::geometry_lib::CalArcSteer(arc);
  uint8_t gear_1 = pnc::geometry_lib::CalArcGear(arc);
  uint8_t steer_2 = pnc::geometry_lib::CalArcSteer(arc2);
  uint8_t gear_2 = pnc::geometry_lib::CalArcGear(arc2);

  if (steer_1 != current_arc_steer || gear_1 != current_gear ||
      steer_2 != next_arc_steer || gear_2 != next_gear) {
    std::cout << "TwoArcPlan fail 1\n";
    return false;
  }

  pnc::geometry_lib::PathSegment arc_seg1(steer_1, gear_1, arc);
  path_seg_vec.emplace_back(arc_seg1);
  pnc::geometry_lib::PathSegment arc_seg2(steer_2, gear_2, arc2);
  path_seg_vec.emplace_back(arc_seg2);

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
    std::cout << "LineArcPlan fail 0\n";
    return false;
  }

  // check which center and tang pt is suitable
  for (size_t i = 0; i < centers.size(); ++i) {
    pnc::geometry_lib::Arc tmp_arc;
    tmp_arc.pA = tangent_ptss[i].first;
    tmp_arc.headingA = line_seg1.heading;
    tmp_arc.pB = tangent_ptss[i].second;
    const auto rot_angle = pnc::geometry_lib::GetAngleFromTwoVec(
        tangent_ptss[i].first - centers[i],
        tangent_ptss[i].second - centers[i]);

    tmp_arc.headingB = tmp_arc.headingA + rot_angle;
    if (!pnc::mathlib::IsDoubleEqual(tmp_arc.headingB, line_seg2.heading)) {
      continue;
    }
    tmp_arc.circle_info.center = centers[i];
    tmp_arc.circle_info.radius = arc.circle_info.radius;

    const uint8_t tmp_arc_steer = pnc::geometry_lib::CalArcSteer(tmp_arc);
    const uint8_t tmp_gear = pnc::geometry_lib::CalArcGear(tmp_arc);
    if (tmp_arc_steer != current_arc_steer || tmp_gear != current_gear ||
        !pnc::geometry_lib::CompleteArcInfo(tmp_arc)) {
      continue;
    }

    pnc::geometry_lib::LineSegment line(line_seg1.pA, tangent_ptss[i].first,
                                        line_seg1.heading);
    if (pnc::geometry_lib::CalLineSegGear(line) != current_gear) {
      std::cout << "line seg gear is error, line seg gear = "
                << static_cast<int>(pnc::geometry_lib::CalLineSegGear(line))
                << std::endl;
      continue;
    }
    pnc::geometry_lib::PathSegment line_seg(current_gear, line);
    path_seg_vec.emplace_back(line_seg);

    pnc::geometry_lib::PathSegment arc_seg(tmp_arc_steer, tmp_gear, tmp_arc);
    path_seg_vec.emplace_back(arc_seg);

    return true;
  }
  std::cout << "LineArcPlan fail 1\n";
  return false;
}

const bool PerpendicularPathPlanner::OneLinePlan(
    pnc::geometry_lib::LineSegment& line,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear) {
  std::cout << "--- try one line plan ---\n";
  pnc::geometry_lib::PathPoint pose;
  pose.Set(line.pA, line.heading);
  if (pnc::geometry_lib::IsPoseOnLine(pose, calc_params_.target_line,
                                      kMaxYCoordSeenAsLine,
                                      kMaxHeadingSeenAsLine)) {
    std::cout << "pose is on line, success\n";
    line.pB = calc_params_.target_line.pA;
    line.length = (line.pB - line.pA).norm();
    pnc::geometry_lib::PathSegment line_seg;
    if (line.length > apa_param.GetParam().static_pos_eps) {
      const uint8_t seg_gear = pnc::geometry_lib::CalLineSegGear(line);
      if (seg_gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
          seg_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) {
        std::cout << "the line gear is invalid\n";
        return false;
      }
      pnc::geometry_lib::PathSegment line_seg(seg_gear, line);
      path_seg_vec.emplace_back(line_seg);
      return true;
    }
    std::cout << "already plan to target pos\n";
    return false;
  } else {
    std::cout << "pose is not on line, fail\n";
    return false;
  }
}
// multi plan end

// adjust plan start
const bool PerpendicularPathPlanner::AdjustPlan() {
  std::cout << "-----adjust plan-----\n";
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
    current_gear = output_.gear_cmd_vec.back();
    std::cout << "continue to plan after multi\n";
  }

  // check pose, if error is large, adjust is not suitable
  if ((std::fabs(current_pose.pos.y()) >= 1.2 &&
       std::fabs(current_pose.heading) >= 50.0 / 57.3) ||
      std::fabs(current_pose.heading) >= 70.0 / 57.3) {
    std::cout << "pose err is relatively large, skip adjust plan, plan fail\n";
    return false;
  }

  // check gear and steer
  if ((current_gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
       current_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) ||
      (current_arc_steer != pnc::geometry_lib::SEG_STEER_LEFT &&
       current_arc_steer != pnc::geometry_lib::SEG_STEER_RIGHT)) {
    std::cout << "ref_gear or ref_arc_steer error\n";
    return false;
  }

  std::cout << "try adjust plan to target point\n";
  bool success = false;
  double steer_change_ratio = 1.0;
  double steer_change_radius = apa_param.GetParam().max_radius_in_slot;
  // set steer_change_radius according to slot_occupied_ratio
  // const std::vector<double> ratio_tab = {0.0, 0.2, 0.5, 0.8, 1.0};
  // const double radius_change = kMaxRadiusInSlot - kMinTurnRadius;
  // if (radius_change < 1e-8) {
  //   std::cout << "radius setting is err\n";
  //   return false;
  // }
  // const std::vector<double> radius_tab = {
  //     kMinTurnRadius, kMinTurnRadius + radius_change * 0.22,
  //     kMinTurnRadius + radius_change * 0.55,
  //     kMinTurnRadius + radius_change * 0.88, kMinTurnRadius + radius_change};

  // steer_change_radius =
  //     pnc::mathlib::Interp1(ratio_tab, radius_tab,
  //     input_.slot_occupied_ratio);

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  for (size_t i = 0; i < kMaxPathNumsInSlot; ++i) {
    std::cout << "-------- No." << i << " in adjust-plan--------\n";
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);
    if (CalSinglePathInAdjust(tmp_path_seg_vec, current_pose, current_gear,
                              steer_change_ratio, steer_change_radius)) {
      output_.path_available = true;
      success = true;
    } else {
      std::cout << "single path of adjust plan failed!\n\n";
      output_.Reset();
      success = false;
      break;
    }

    for (const auto& tmp_path_seg : tmp_path_seg_vec) {
      output_.path_segment_vec.emplace_back(tmp_path_seg);
      output_.length += tmp_path_seg.Getlength();
      output_.gear_cmd_vec.emplace_back(tmp_path_seg.seg_gear);
      output_.steer_vec.emplace_back(tmp_path_seg.seg_steer);
    }

    if (output_.path_segment_vec.size() > 0) {
      const auto& last_segment = output_.path_segment_vec.back();
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

    if ((current_pose.pos - input_.tlane.pt_terminal).norm() <=
            apa_param.GetParam().static_pos_eps &&
        std::fabs(current_pose.heading - calc_params_.target_line.heading) <=
            apa_param.GetParam().static_heading_eps / 57.3) {
      std::cout << "already plan to target pos!\n";
      if (output_.path_segment_vec.size() > 0) {
        const auto& last_segment = output_.path_segment_vec.back();
        if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_LINE &&
            last_segment.seg_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) {
          std::cout << "last line is not reverse, should lose\n";
          output_.length -= last_segment.Getlength();
          output_.path_segment_vec.pop_back();
          output_.gear_cmd_vec.pop_back();
          output_.steer_vec.pop_back();
        }
      }
      break;
    }
  }

  if (!success) {
    std::cout << "adjust plan failed!" << std::endl;
  }

  return success;
}

const bool PerpendicularPathPlanner::OneArcPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  std::cout << "try one arc plan\n";
  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;
  if (pnc::mathlib::IsDoubleEqual(arc.headingA,
                                  calc_params_.target_line.heading) ||
      pnc::mathlib::IsDoubleEqual(arc.pA.y(),
                                  calc_params_.target_line.pA.y())) {
    std::cout << "current heading is equal to target heading or current y is "
                 "equal to "
                 "target y, no need to one arc plan\n";
    return false;
  }
  bool success = pnc::geometry_lib::CalOneArcWithLineAndGear(
      arc, calc_params_.target_line, current_gear);

  if (success) {
    // check radius and gear can or not meet needs
    std::cout << "cal radius = " << arc.circle_info.radius << std::endl;
    const auto steer = pnc::geometry_lib::CalArcSteer(arc);
    const auto gear = pnc::geometry_lib::CalArcGear(arc);
    success = (arc.circle_info.radius >=
                   apa_param.GetParam().min_turn_radius - 1e-3 &&
               arc.circle_info.radius <=
                   apa_param.GetParam().max_one_step_arc_radius + 1e-3) &&
              (gear == current_gear);
    if (success) {
      std::cout << "one arc plan success\n";
      // std::cout << "start: coord = " << arc.pA.transpose()
      //           << "  heading = " << arc.headingA * 57.3
      //           << "  end: coord = " << arc.pB.transpose()
      //           << "  heading = " << arc.headingB * 57.3 << std::endl;
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }
  if (!success) {
    std::cout << "one arc plan fail\n";
  }
  return success;
}

const bool PerpendicularPathPlanner::AlignBodyPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  std::cout << "try align body plan\n";
  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;
  arc.circle_info.radius = apa_param.GetParam().min_turn_radius;
  // check if it is necessary to align body
  if (pnc::mathlib::IsDoubleEqual(arc.headingA,
                                  calc_params_.target_line.heading)) {
    std::cout << "body already align\n";
    return false;
  }
  bool success = pnc::geometry_lib::CalOneArcWithTargetHeading(
      arc, current_gear, calc_params_.target_line.heading);

  if (success) {
    // check gear can or not meet needs
    const auto steer = pnc::geometry_lib::CalArcSteer(arc);
    const auto gear = pnc::geometry_lib::CalArcGear(arc);
    success = (gear == current_gear);
    std::cout << "steer = " << static_cast<int>(steer) << std::endl;
    // std::cout << "start: coord = " << arc.pA.transpose()
    //           << "  heading = " << arc.headingA * 57.3
    //           << "  end: coord = " << arc.pB.transpose()
    //           << "  heading = " << arc.headingB * 57.3 << std::endl;
    if (success) {
      std::cout << "align body plan success\n";
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }
  if (!success) {
    std::cout << "align body plan fail\n";
  }
  return success;
}

const bool PerpendicularPathPlanner::STurnParallelPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const double steer_change_ratio1,
    const double steer_change_radius) {
  std::cout << "try s turn parallel plan\n";
  pnc::geometry_lib::Arc arc_s_1;
  arc_s_1.headingA = current_pose.heading;
  arc_s_1.pA = current_pose.pos;
  // check if it is possible to take S turn to target line
  if (!pnc::mathlib::IsDoubleEqual(arc_s_1.headingA,
                                   calc_params_.target_line.heading)) {
    std::cout << "body no align\n";
    return false;
  }
  if (pnc::mathlib::IsDoubleEqual(arc_s_1.pA.y(),
                                  calc_params_.target_line.pA.y())) {
    std::cout << "current pos is already on target line, no need to "
                 "STurnParallelPlan\n";
    return true;
  }

  pnc::geometry_lib::PathPoint terminal_err;
  terminal_err.Set(current_pose.pos - input_.tlane.pt_terminal,
                   current_pose.heading - 0.0);
  double slot_occupied_ratio;
  if (std::fabs(terminal_err.pos.y()) < 0.9 &&
      std::fabs(terminal_err.heading) < 75.0 / 57.3) {
    slot_occupied_ratio = pnc::mathlib::Clamp(
        1.0 - (terminal_err.pos.x() / apa_param.GetParam().normal_slot_length),
        0.0, 1.0);
  } else {
    slot_occupied_ratio = 0.0;
  }

  const std::vector<double> ratio_tab = {0.0, 0.2, 0.5, 0.8, 1.0};
  const double radius_change = apa_param.GetParam().max_radius_in_slot -
                               apa_param.GetParam().min_radius_out_slot;
  if (radius_change < 1e-8) {
    std::cout << "radius setting is err\n";
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

  std::cout << "real_steer_change_radius = " << real_steer_change_radius
            << std::endl;

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
    const auto steer_1 = pnc::geometry_lib::CalArcSteer(arc_s_1);
    const auto gear_1 = pnc::geometry_lib::CalArcGear(arc_s_1);
    const auto steer_2 = pnc::geometry_lib::CalArcSteer(arc_s_2);
    const auto gear_2 = pnc::geometry_lib::CalArcGear(arc_s_2);
    std::cout << "steer_1 = " << static_cast<int>(steer_1)
              << "  steer_2 = " << static_cast<int>(steer_2) << std::endl;
    success = (gear_1 == current_gear && gear_2 == current_gear &&
               steer_1 != steer_2);
    // std::cout << "start: coord = " << arc_s_1.pA.transpose()
    //           << "  heading= " << arc_s_1.headingA * 57.3
    //           << "  mid: coord = " << arc_s_1.pB.transpose()
    //           << "  heading = " << arc_s_1.headingB * 57.3
    //           << "  end: coord = " << arc_s_2.pB.transpose()
    //           << "  heading = " << arc_s_2.headingB * 57.3 << std::endl;
    if (success) {
      std::cout << "s turn parallel plan success\n";
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer_1, gear_1, arc_s_1));
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer_2, gear_2, arc_s_2));
    }
  }
  if (!success) {
    std::cout << "s turn parallel plan fail\n";
  }

  return success;
}

const bool PerpendicularPathPlanner::CalSinglePathInAdjust(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const double steer_change_ratio,
    const double steer_change_radius) {
  std::cout << "-----CalSinglePathInAdjust-----\n";
  std::cout << "current_gear = " << static_cast<int>(current_gear)
            << ",  current_pos = " << current_pose.pos.transpose()
            << ",  current_heading = " << current_pose.heading * 57.3
            << std::endl;

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(5);
  // first try one arc to target line
  bool success = OneArcPlan(tmp_path_seg_vec, current_pose, current_gear);

  // if try one arc fail, second try align the ego body and then try go take
  // S-turn to target line
  if (!success) {
    success = AlignBodyPlan(tmp_path_seg_vec, current_pose, current_gear);
    pnc::geometry_lib::PathPoint tmp_current_pose = current_pose;
    if (success) {
      tmp_current_pose.Set(tmp_path_seg_vec.back().GetArcSeg().pB,
                           tmp_path_seg_vec.back().GetArcSeg().headingB);
    }

    success =
        STurnParallelPlan(tmp_path_seg_vec, tmp_current_pose, current_gear,
                          steer_change_ratio, steer_change_radius);
  }

  if (!success) {
    tmp_path_seg_vec.clear();
  }

  // try one line plan
  pnc::geometry_lib::PathPoint last_pose;
  if (tmp_path_seg_vec.size() > 0) {
    const auto& last_segment = tmp_path_seg_vec.back();
    last_pose.Set(last_segment.GetArcSeg().pB,
                  last_segment.GetArcSeg().headingB);
    std::cout << "last path pose to one plan\n";
  } else {
    last_pose = current_pose;
    std::cout << "current pose to one plan\n";
  }
  if ((last_pose.pos - input_.tlane.pt_terminal).norm() <=
          apa_param.GetParam().static_pos_eps &&
      std::fabs(last_pose.heading - calc_params_.target_line.heading) <=
          apa_param.GetParam().static_heading_eps / 57.3) {
    std::cout << "already plan to target pos, no need to one line plan!\n";
  } else {
    // try line
    pnc::geometry_lib::LineSegment last_line;
    last_line.pA = last_pose.pos;
    last_line.heading = last_pose.heading;
    if (OneLinePlan(last_line, tmp_path_seg_vec, current_gear)) {
      std::cout << "OneLinePlan success\n";
    } else {
      std::cout << "OneLinePlan fail\n";
    }
  }

  // std::cout << "tmp_path_seg_vec:" << std::endl;
  // for (const auto& tmp_path_seg : tmp_path_seg_vec) {
  //   PrintSegmentInfo(tmp_path_seg);
  // }

  // collision detection
  for (auto& tmp_path_seg : tmp_path_seg_vec) {
    const uint8_t path_col_det_res = TrimPathByCollisionDetection(tmp_path_seg);

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
    std::cout << "CalSinglePathInAdjust success\n";
    std::cout << "cur_path_seg_vec:\n";
    double length = 0.0;
    for (const auto& path_seg : path_seg_vec) {
      PrintSegmentInfo(path_seg);
      length += path_seg.Getlength();
    }
    if (length < 0.25) {
      std::cout << "this gear path is too small, lose it\n";
      path_seg_vec.clear();
    }
    return true;
  } else {
    std::cout << "CalSinglePathInAdjust fail\n";
    return false;
  }

  // path_seg_vec = tmp_path_seg_vec;

  // if (path_seg_vec.size() > 0) {
  //   std::cout << "CalSinglePathInAdjust success\n";
  //   return true;
  // } else {
  //   std::cout << "CalSinglePathInAdjust fail\n";
  //   return false;
  // }
}
// adjust plan end

// sample path start
const bool PerpendicularPathPlanner::SetCurrentPathSegIndex() {
  if (!output_.path_available) {
    return false;
  }

  if (output_.gear_cmd_vec.size() < 1 || output_.path_segment_vec.size() < 1) {
    std::cout << "no path can get" << std::endl;
    return false;
  }

  if (output_.is_first_path == true) {
    // first get path segment from path_segment_vec, first and second index is 0
    // at the moment
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
    pnc::geometry_lib::PathSegment new_line;
    new_line.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;

    if (path_seg.Getlength() + extend_distance <
        apa_param.GetParam().min_one_step_path_length) {
      extend_distance =
          apa_param.GetParam().min_one_step_path_length - path_seg.Getlength();
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

    const uint8_t path_col_res = TrimPathByCollisionDetection(new_line);

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

      std::cout << "insert line segment successful\n";
    } else {
      std::cout << "can not inset line segment\n";
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

void PerpendicularPathPlanner::ExtendCurrentFollowLastPath(
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
    TrimPathByCollisionDetection(path_seg);
  }
}

const bool PerpendicularPathPlanner::SampleCurrentPathSeg() {
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

  // get last point
  path_point.Set(cur_line_seg.pB, cur_line_seg.heading);
  output_.path_point_vec.emplace_back(path_point);
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

  // get last point
  path_point.Set(current_arc_seg.pB,
                 pnc::geometry_lib::NormalizeAngle(current_arc_seg.headingB));

  output_.path_point_vec.emplace_back(path_point);
}
// sample path end

// collision detect start
const uint8_t PerpendicularPathPlanner::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg) {
  std::cout << "--- collision detection ---" << std::endl;
  CollisionDetector::CollisionResult col_res;
  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    auto& line = path_seg.line_seg;
    col_res = collision_detector_ptr_->Update(line, line.heading);
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    auto& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->Update(arc, arc.headingA);
  } else {
    std::cout << "no support the seg type\n";
    return PATH_COL_INVALID;
  }

  const double remain_car_dist = col_res.remain_car_dist;
  const double remain_obs_dist = col_res.remain_obstacle_dist;
  const double safe_remain_dist =
      std::min(remain_car_dist, remain_obs_dist - 0.3);

  std::cout << "  remain_car_dist = " << remain_car_dist
            << "  remain_obs_dist = " << remain_obs_dist
            << "  safe_remain_dist = " << safe_remain_dist << std::endl;
  if (safe_remain_dist < 0.0) {
    std::cout << "safe_remain_dist is samller than 0.0, the path donot meet "
                 "requirements\n";
    return PATH_COL_INVALID;
  }

  if (remain_car_dist > safe_remain_dist) {
    std::cout << "the path will collide, the length need shorten to "
                 "safe_remain_dist\n";
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
    return PATH_COL_SHORTEN;
  } else {
    std::cout << "the path will not collide\n";
    return PATH_COL_NORMAL;
  }
}
// collision detect end

void PerpendicularPathPlanner::PrintSegmentInfo(
    const pnc::geometry_lib::PathSegment& seg) const {
  std::cout << "----" << std::endl;
  std::cout << "seg_gear: " << static_cast<int>(seg.seg_gear) << std::endl;

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

void PerpendicularPathPlanner::PrintOutputSegmentsInfo() const {
  std::cout << "-------------- OutputSegmentsInfo --------------" << std::endl;
  for (size_t i = 0; i < output_.path_segment_vec.size(); i++) {
    const auto& current_seg = output_.path_segment_vec[i];

    if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      const auto& line_seg = current_seg.line_seg;

      std::cout << "Segment [" << i << "] "
                << " LINE_SEGMENT "
                << " length= " << line_seg.length << std::endl;

      std::cout << "seg_gear: " << static_cast<int>(current_seg.seg_gear)
                << std::endl;

      std::cout << "seg_steer: " << static_cast<int>(current_seg.seg_steer)
                << std::endl;

      std::cout << "start_pos: " << line_seg.pA.transpose() << std::endl;
      std::cout << "start_heading: " << line_seg.heading << std::endl;
      std::cout << "end_pos: " << line_seg.pB.transpose() << "\n\n";
    } else {
      const auto& arc_seg = current_seg.arc_seg;

      std::cout << "Segment [" << i << "] "
                << "ARC_SEGMENT "
                << "length= " << arc_seg.length << std::endl;

      std::cout << "seg_gear: " << static_cast<int>(current_seg.seg_gear)
                << std::endl;

      std::cout << "seg_steer: " << static_cast<int>(current_seg.seg_steer)
                << std::endl;

      std::cout << "start_pos: " << arc_seg.pA.transpose() << std::endl;
      std::cout << "start_heading: " << arc_seg.headingA << std::endl;
      std::cout << "end_pos: " << arc_seg.pB.transpose() << std::endl;
      std::cout << "end_heading: " << arc_seg.headingB << std::endl;
      std::cout << "center: " << arc_seg.circle_info.center.transpose()
                << "  radius = " << arc_seg.circle_info.radius << "\n\n";
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

}  // namespace apa_planner
}  // namespace planning