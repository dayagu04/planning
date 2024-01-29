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

#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_world.h"
#include "collision_detection.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "math_lib.h"

namespace planning {
namespace apa_planner {

static const size_t kMaxParallelParkInSegmentNums = 15;
static const size_t kReservedOutputPathPointSize = 750;
static const size_t kMaxPathNumsInSlot = 6;

static const std::vector<double> kPrepareTargetLineDeltaHeadingTab = {
    10.0 / 57.3, 12.0 / 57.3, 8.0 / 57.3};

static const std::vector<Eigen::Vector2d> kPrepareTargetLinePosTab = {
    Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(0.0, 0.0)};

void ParallelPathPlanner::Reset() {
  output_.Reset();
  calc_params_.Reset();
  output_.steer_vec.reserve(kMaxParallelParkInSegmentNums);
  output_.gear_cmd_vec.reserve(kMaxParallelParkInSegmentNums);
  output_.path_segment_vec.reserve(kMaxParallelParkInSegmentNums);
}

void ParallelPathPlanner::Preprocess() {
  calc_params_.Reset();

  calc_params_.is_left_side =
      (input_.tlane.slot_side == ApaPlannerBase::SLOT_SIDE_LEFT);

  calc_params_.slot_side_sgn = calc_params_.is_left_side ? -1.0 : 1.0;

  // target line
  calc_params_.target_line =
      pnc::geometry_lib::BuildLineSegByPose(input_.tlane.pt_terminal_pos, 0.0);
}

const bool ParallelPathPlanner::Update() {
  std::cout << "----------------------------------------- path "
               "planner:---------------------------------------"
            << std::endl;
  // preprocess
  Preprocess();
  // reset output
  output_.Reset();

  if (!CalMinSafeCircle()) {
    std::cout << "inverse plan in slot failed!" << std::endl;
    return false;
  }
  // prepare plan, only for first plan which is transferred to search from
  // target to start pose.
  if (input_.is_replan_first) {
    if (!PreparePlan()) {
      std::cout << "prepare plan failed!" << std::endl;
      return false;
    }
    GenPathOutputByDubins();
    return true;
  }

  // normal plan, if ego car is outside of slot, use inverse plan, or plan from
  // current pose to target pose
  if (input_.ego_pose.pos.y() * calc_params_.slot_side_sgn >=
      input_.tlane.pt_inside.y() * calc_params_.slot_side_sgn) {
    if (ProcessPlan()) {
      std::cout << "process plan success!" << std::endl;
      return true;
    } else {
      std::cout << "process plan failed!" << std::endl;
    }
  }

  // adjust step
  if (AdjustPlan()) {
    std::cout << "adjust step plan success!" << std::endl;
    return true;
  } else {
    std::cout << "adjust step plan failed!" << std::endl;
  }

  return false;
}

const bool ParallelPathPlanner::Update(
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  collision_detector_ptr_ = collision_detector_ptr;
  return Update();
}

const bool ParallelPathPlanner::PreparePlan() {
  std::vector<double> x_offset_vec;
  std::vector<double> y_offset_vec;

  double x_offset = input_.tlane.pt_terminal_pos.x() + 5.0;
  while (x_offset < input_.tlane.pt_terminal_pos.x() + 10.0) {
    x_offset_vec.emplace_back(x_offset);
    x_offset += 0.1;
  }

  double y_offset =
      input_.tlane.pt_inside.y() + calc_params_.slot_side_sgn * 1.8;

  const double y_offset_limit_abs =
      std::fabs(input_.tlane.pt_inside.y() + calc_params_.slot_side_sgn * 4.0);

  const double deta_y = calc_params_.slot_side_sgn * 0.1;
  while (std::fabs(y_offset) < y_offset_limit_abs) {
    y_offset_vec.emplace_back(y_offset);
    y_offset += deta_y;
  }

  for (const auto& x_coord : x_offset_vec) {
    for (const auto& y_coord : y_offset_vec) {
      if (PreparePlanOnce(x_coord, y_coord)) {
        // std::cout << "input_.tlane.pt_inside.y() ="
        //           << input_.tlane.pt_inside.y() << std::endl;
        // std::cout << "calc_params_.slot_side_sgn ="
        //           << calc_params_.slot_side_sgn << std::endl;
        // std::cout << "prepare x coord = " << x_coord << std::endl;
        // std::cout << "prepare y coord = " << y_coord << std::endl;

        return true;
      }
    }
  }

  return false;
}

const bool ParallelPathPlanner::PreparePlanOnce(const double x_coord,
                                                const double y_coord) {
  pnc::geometry_lib::PathPoint prepare_target_pose(
      Eigen::Vector2d(x_coord, y_coord), 0.0);

  if (!TripleStepPlan(prepare_target_pose, calc_params_.safe_circle_target_pose,
                      apa_param.GetParam().min_turn_radius)) {
    return false;
  }

  prepare_target_pose.pos.x() += 0.3;
  // use dubins plan from input.cur_pose to prepare pt
  pnc::dubins_lib::DubinsLibrary::Input prepare_step_input;
  prepare_step_input.radius = apa_param.GetParam().min_turn_radius;

  prepare_step_input.Set(input_.ego_pose.pos, prepare_target_pose.pos,
                         input_.ego_pose.heading, prepare_target_pose.heading);

  dubins_planner_.SetInput(prepare_step_input);
  const bool res = dubins_planner_.OneStepDubinsUpdate();

  return res;
}

const bool ParallelPathPlanner::GenPathOutputByDubins() {
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

  const bool res =
      (output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE);

  return res;
}

const bool ParallelPathPlanner::ProcessPlan() {
  bool res = false;
  std::cout << "safe circle target pos in process plan="
            << calc_params_.safe_circle_target_pose.pos.transpose()
            << std::endl;

  pnc::geometry_lib::PathPoint alternative_target_pose =
      calc_params_.safe_circle_target_pose;

  std::vector<double> deta_x_vec = {0, -0.25, -0.3};
  for (double deta_x = deta_x_vec.back(); deta_x > -1.1; deta_x -= 0.1) {
    deta_x_vec.emplace_back(deta_x);
  }

  for (size_t i = 0; i < deta_x_vec.size(); i++) {
    alternative_target_pose.pos.x() += deta_x_vec[i];

    // use TripleStepPlan out of slot, use multiPlan in slot
    res = TripleStepPlan(input_.ego_pose, alternative_target_pose,
                         apa_param.GetParam().min_turn_radius);
    if (res) {
      GenPathOutputByDubins();
      break;
    }
  }
  return res;
}

const bool ParallelPathPlanner::CalMinSafeCircle() {
  // search from target pose to current pose
  pnc::geometry_lib::PathPoint start_pose;
  start_pose.Set(input_.tlane.pt_terminal_pos, 0.0);

  // firstly backward to limit pose of t-lane
  pnc::geometry_lib::PathPoint current_pose = start_pose;

  const double radius = apa_param.GetParam().min_turn_radius;

  const uint8_t dirve_step_steer =
      (calc_params_.is_left_side ? pnc::geometry_lib::SEG_STEER_RIGHT
                                 : pnc::geometry_lib::SEG_STEER_LEFT);
  bool success = false;
  std::vector<pnc::geometry_lib::PathSegment> path_vec;

  for (size_t i = 0; i < kMaxPathNumsInSlot / 2; i++) {
    // Todo: finish collision detection with pt inside
    if (CheckParkOutWithCollisionFree(current_pose, radius)) {
      calc_params_.safe_circle_target_pose = current_pose;
      calc_params_.safe_circle.radius = radius;
      calc_params_.safe_circle.center =
          CalEgoTurningCenter(current_pose, radius, dirve_step_steer);
      std::cout << "safe circle target pt =" << current_pose.pos.transpose()
                << std::endl;
      success = true;
      break;
    } else {
      // // collided with pt_inside
      // InverseTwoArcInSlot(tmp_arc_vec, current_pose, dirve_step_steer);
    }
  }
  return success;
}

const bool ParallelPathPlanner::CheckParkOutWithCollisionFree(
    const pnc::geometry_lib::PathPoint& current_pose,
    const double radius) const {
  return true;
}

const bool ParallelPathPlanner::TripleStepPlan(
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const double radius) {
  pnc::dubins_lib::DubinsLibrary::Input dubins_input;
  dubins_input.radius = radius;
  dubins_input.Set(current_pose.pos, target_pose.pos, current_pose.heading,
                   target_pose.heading);
  dubins_planner_.SetInput(dubins_input);

  for (size_t i = 0; i < pnc::dubins_lib::DubinsLibrary::CASE_COUNT; ++i) {
    const uint8_t dubins_type =
        (calc_params_.is_left_side ? pnc::dubins_lib::DubinsLibrary::L_S_R
                                   : pnc::dubins_lib::DubinsLibrary::R_S_L);

    if (dubins_planner_.Solve(dubins_type, i)) {
      if (dubins_planner_.GetOutput().gear_change_count == 0) {
        // std::cout << "dubins success!" << std::endl;
        return true;
      }
    }
  }

  return false;
}

const bool ParallelPathPlanner::MultiPlan() { return true; }

// adjust plan start
const bool ParallelPathPlanner::AdjustPlan() {
  std::cout << "-----adjust plan-----\n";
  // set init state
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;

  // check pose, if error is large, adjust is not suitable
  if ((std::fabs(current_pose.pos.y()) >= 1.0 ||
       std::fabs(pnc::geometry_lib::NormalizeAngle(current_pose.heading)) >=
           45.0 / 57.3)) {
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
  double steer_change_radius = apa_param.GetParam().min_radius_out_slot;

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  for (size_t i = 0; i < 1; ++i) {
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

    if ((current_pose.pos - input_.tlane.pt_terminal_pos).norm() <=
            apa_param.GetParam().static_pos_eps &&
        std::fabs(current_pose.heading - calc_params_.target_line.heading) <=
            apa_param.GetParam().static_heading_eps / 57.3) {
      std::cout << "already plan to target pos!\n\n";
      break;
    }
  }

  if (!success) {
    std::cout << "adjust plan failed!" << std::endl;
  }

  return success;
}

const bool ParallelPathPlanner::OneArcPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  std::cout << "try one arc plan\n";
  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;
  bool success = pnc::geometry_lib::CalOneArcWithLineAndGear(
      arc, calc_params_.target_line, current_gear);

  if (success) {
    // check radius and gear can or not meet needs
    std::cout << "cal radius = " << arc.circle_info.radius << std::endl;
    const auto steer = pnc::geometry_lib::CalArcSteer(arc);
    const auto gear = pnc::geometry_lib::CalArcGear(arc);
    success = (arc.circle_info.radius >= apa_param.GetParam().min_turn_radius &&
               arc.circle_info.radius <=
                   apa_param.GetParam().max_one_step_arc_radius) &&
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

const bool ParallelPathPlanner::AlignBodyPlan(
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

const bool ParallelPathPlanner::STurnParallelPlan(
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

  pnc::geometry_lib::PathPoint terminal_err;
  terminal_err.Set(current_pose.pos - input_.tlane.pt_terminal_pos,
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
                               apa_param.GetParam().min_turn_radius;
  if (radius_change < 1e-8) {
    std::cout << "radius setting is err\n";
    return false;
  }
  const std::vector<double> radius_tab = {
      apa_param.GetParam().min_turn_radius,
      apa_param.GetParam().min_turn_radius + radius_change * 0.22,
      apa_param.GetParam().min_turn_radius + radius_change * 0.55,
      apa_param.GetParam().min_turn_radius + radius_change * 0.88,
      apa_param.GetParam().min_turn_radius + radius_change};

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

const bool ParallelPathPlanner::CalSinglePathInAdjust(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const double steer_change_ratio,
    const double steer_change_radius) {
  std::cout << "-----CalSinglePathInAdjust-----\n";
  std::cout << "current_gear = " << static_cast<int>(current_gear)
            << ",  current_pos = " << current_pose.pos.transpose()
            << ",  current_heading = " << current_pose.heading * 57.3
            << std::endl;
  std::cout << "target_line pa =" << calc_params_.target_line.pA.transpose()
            << " " << calc_params_.target_line.heading * 57.3 << std::endl;

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
      // PrintSegmentInfo(tmp_path_seg_vec.front());
      tmp_current_pose.Set(tmp_path_seg_vec.back().GetArcSeg().pB,
                           tmp_path_seg_vec.back().GetArcSeg().headingB);
    }

    success =
        STurnParallelPlan(tmp_path_seg_vec, tmp_current_pose, current_gear,
                          steer_change_ratio, steer_change_radius);
    // std::cout << "S turn arc1 " << std::endl;
    // PrintSegmentInfo(tmp_path_seg_vec[1]);
    // std::cout << "S turn arc2 " << std::endl;
    // PrintSegmentInfo(tmp_path_seg_vec[2]);
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
    std::cout << "last pose: " << last_pose.pos.transpose() << ", "
              << last_pose.heading * 57.3 << std::endl;
  } else {
    last_pose = current_pose;
    std::cout << "current pose to one plan\n";
  }
  if (IsLocatedOnTarget(last_pose)) {
    std::cout << "already plan to target pos, no need to one line plan!\n";
  } else {
    // try line
    pnc::geometry_lib::LineSegment last_line;
    last_line.pA = last_pose.pos;
    last_line.heading = last_pose.heading;
    if (OneLinePlan(last_line, tmp_path_seg_vec, current_gear)) {
      std::cout << "last line seg: " << std::endl;
      std::cout << tmp_path_seg_vec.front().line_seg.pA.transpose()
                << std::endl;
      std::cout << tmp_path_seg_vec.front().line_seg.pB.transpose()
                << std::endl;
      std::cout << "OneLinePlan success\n";
    } else {
      std::cout << "OneLinePlan fail\n";
    }
  }

  // std::cout << "tmp_path_seg_vec:" << std::endl;
  // for (const auto& tmp_path_seg : tmp_path_seg_vec) {
  //   PrintSegmentInfo(tmp_path_seg);
  // }

  // // collision detection
  // for (auto& tmp_path_seg : tmp_path_seg_vec) {
  //   const uint8_t path_col_det_res =
  //   TrimPathByCollisionDetection(tmp_path_seg);

  //   if (path_col_det_res == PATH_COL_NORMAL) {
  //     path_seg_vec.emplace_back(tmp_path_seg);
  //   } else if (path_col_det_res == PATH_COL_SHORTEN) {
  //     path_seg_vec.emplace_back(tmp_path_seg);
  //     std::cout << "cut path!" << std::endl;
  //     break;
  //   } else if (path_col_det_res == PATH_COL_INVALID) {
  //     break;
  //   }
  // }

  for (auto& tmp_path_seg : tmp_path_seg_vec) {
    path_seg_vec.emplace_back(tmp_path_seg);
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

const uint8_t ParallelPathPlanner::TrimPathByCollisionDetection(
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

const bool ParallelPathPlanner::OneLinePlan(
    pnc::geometry_lib::LineSegment& line,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear) {
  std::cout << "--- try one line plan ---\n";

  pnc::geometry_lib::PathPoint pose;
  pose.Set(line.pA, line.heading);

  std::cout << "last pose deg" << pose.pos.transpose() << ", "
            << pose.heading * 57.3 << std::endl;

  std::cout << "target line" << calc_params_.target_line.pA.transpose()
            << calc_params_.target_line.heading * 57.3 << std::endl;

  if (pnc::geometry_lib::IsPoseOnLine(
          pose, calc_params_.target_line, apa_param.GetParam().static_pos_eps,
          apa_param.GetParam().static_heading_eps / 57.3)) {
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
  //   std::cout << static_cast<int>(output_.path_segment_vec[i].seg_type) <<
  //   "
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
  //           << static_cast<int>(output_.path_seg_index.second) <<
  //           std::endl;

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

void ParallelPathPlanner::PrintSegmentInfo(
    const pnc::geometry_lib::PathSegment& seg) const {
  std::cout << "----" << std::endl;
  std::cout << "seg_gear: " << static_cast<int>(seg.seg_gear) << std::endl;

  std::cout << "seg_steer: " << static_cast<int>(seg.seg_steer) << std::endl;
  std::cout << "seg_type: " << static_cast<int>(seg.seg_type) << std::endl;
  std::cout << "length: " << seg.Getlength() << std::endl;

  if (seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    std::cout << "start_pos: " << seg.GetLineSeg().pA.transpose() << std::endl;
    std::cout << "start_heading deg: " << seg.GetLineSeg().heading * 57.3
              << std::endl;
    std::cout << "end_pos: " << seg.GetLineSeg().pB.transpose() << std::endl;
    std::cout << "end_heading deg: " << seg.GetLineSeg().heading * 57.3
              << std::endl;
  } else {
    std::cout << "start_pos: " << seg.GetArcSeg().pA.transpose() << std::endl;
    std::cout << "start_heading deg: " << seg.GetArcSeg().headingA * 57.3
              << std::endl;
    std::cout << "end_pos: " << seg.GetArcSeg().pB.transpose() << std::endl;
    std::cout << "end_heading deg: " << seg.GetArcSeg().headingB * 57.3
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

      std::cout << "seg_gear: " << static_cast<int>(current_seg.seg_gear)
                << std::endl;

      std::cout << "seg_steer: " << static_cast<int>(current_seg.seg_steer)
                << std::endl;

      std::cout << "start_pos: " << line_seg.pA.transpose() << std::endl;
      std::cout << "start_heading deg: " << line_seg.heading * 57.3
                << std::endl;
      std::cout << "end_pos: " << line_seg.pB.transpose() << std::endl;
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
      std::cout << "start_heading deg: " << arc_seg.headingA * 57.3
                << std::endl;
      std::cout << "end_pos: " << arc_seg.pB.transpose() << std::endl;
      std::cout << "end_heading deg: " << arc_seg.headingB * 57.3 << std::endl;
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
  return std::vector<double>{calc_params_.safe_circle.center.x(),
                             calc_params_.safe_circle.center.y(),
                             calc_params_.safe_circle.radius};
}

const Eigen::Vector2d ParallelPathPlanner::CalEgoTurningCenter(
    const pnc::geometry_lib::PathPoint& ego_pose, const double radius,
    const uint8_t steer) const {
  Eigen::Vector2d ego_heading_vec(std::cos(ego_pose.heading),
                                  std::sin(ego_pose.heading));

  Eigen::Vector2d ego_n_vec(-ego_heading_vec.y(), ego_heading_vec.x());
  if (steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
    ego_n_vec *= -1.0;
  }
  return ego_pose.pos + ego_n_vec * radius;
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

const bool ParallelPathPlanner::IsLocatedOnTarget(
    const pnc::geometry_lib::PathPoint& current_pose) const {
  const double lon_error_abs =
      std::fabs(current_pose.pos.x() - input_.tlane.pt_terminal_pos.x());

  const double lat_error_abs =
      std::fabs(current_pose.pos.y() - input_.tlane.pt_terminal_pos.y());

  const double heading_error_abs = std::fabs(pnc::geometry_lib::NormalizeAngle(
      current_pose.heading - calc_params_.target_line.heading));

  return (lon_error_abs <= apa_param.GetParam().finish_lon_err &&
          lat_error_abs <= apa_param.GetParam().finish_lat_err &&
          heading_error_abs <= apa_param.GetParam().finish_heading_err / 57.3);
}
}  // namespace apa_planner
}  // namespace planning