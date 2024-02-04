#include "parallel_path_planner.h"

#include <google/protobuf/message.h>
#include <math.h>
#include <sys/types.h>

#include <algorithm>
#include <chrono>
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

namespace planning {
namespace apa_planner {

static const double kMaxParkOutFirstArcHeading = 66.0;
static const double kMaxParkOutLineLength = 6.0;
static const double kSearchOutArcSampleDistance = 0.2;
static const double kSearchOutLineSampleDistance = 0.3;
static const double kSearchOutTargetHeading = 6.6;

static const double kColBuffer = 0.3;

static const size_t kMaxParallelParkInSegmentNums = 15;
static const size_t kReservedOutputPathPointSize = 750;
static const size_t kMaxPathNumsInSlot = 5;
static const size_t kMaxMultiStepNums = 3;
static const size_t kMaxParallelShiftNums = 6;

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
      (input_.tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT);

  calc_params_.slot_side_sgn = calc_params_.is_left_side ? -1.0 : 1.0;

  // target line
  calc_params_.target_line =
      pnc::geometry_lib::BuildLineSegByPose(input_.tlane.pt_terminal_pos, 0.0);
}

const bool ParallelPathPlanner::Update() {
  std::cout << "-----------------------------------------parallel path "
               "planner:---------------------------------------"
            << std::endl;
  // preprocess
  Preprocess();

  // reset output
  output_.Reset();

  if (CheckEgoPoseCollided(input_.ego_pose)) {
    DEBUG_PRINT("start pose collided!");
    return false;
  }
  DEBUG_PRINT("start pose safe!");

  // judge if ego is out of slot
  const bool ego_in_slot = CheckEgoInSlot();
  std::cout << "is ego in slot? " << ego_in_slot << std::endl;

  if (!ego_in_slot) {
    if (!CalMinSafeCircle()) {
      std::cout << "inverse plan in slot failed!" << std::endl;
      return false;
    }
    std::cout << "CalMinSafeCircle success!" << std::endl;

    // prepare plan, only for first plan which is transferred to search from
    // target to start pose.
    if (input_.is_replan_first) {
      if (PreparePlan()) {
        // PrintOutputSegmentsInfo();
        std::cout << "prepare plan success!" << std::endl;
        return true;
      }
      std::cout << "prepare plan false!" << std::endl;
    } else {
      if (BackwardNormalPlan()) {
        std::cout << "BackwardNormalPlan success!" << std::endl;
        return true;
      }

      if (PlanFromEgoToParkOutRootPose(false)) {
        std::cout << "backward plan success!" << std::endl;
        return true;
      } else {
        std::cout << "backward plan failed!" << std::endl;
      }
    }
  } else {
    // ego is in slot, search from ego pose to target pose, or just
    // correct heading
    if (MultiPlan()) {
      std::cout << "MultiPlan  success!" << std::endl;
      return true;
    } else {
      std::cout << "MultiPlan  failed!" << std::endl;
    }

    // parallel adjust step
    if (ParallelAdjustPlan()) {
      std::cout << "parallel adjust step plan success!" << std::endl;
      // PrintOutputSegmentsInfo();
      return true;
    } else {
      std::cout << "parallel adjust step plan failed!" << std::endl;
    }

    // adjust step
    if (AdjustPlan()) {
      std::cout << "adjust step plan success!" << std::endl;
      return true;
    } else {
      std::cout << "adjust step plan failed!" << std::endl;
    }
  }
  output_.Reset();
  std::cout << "plan failed!" << std::endl;
  return false;
}

const bool ParallelPathPlanner::Update(
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  const auto time0 = std::chrono::high_resolution_clock::now();
  collision_detector_ptr_ = collision_detector_ptr;
  const bool success = Update();

  const auto time1 = std::chrono::high_resolution_clock::now();
  const auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(time1 - time0)
          .count();
  std::cout << "parallel cost time(ms) = " << duration << "\n" << std::endl;
  return success;
}

const bool ParallelPathPlanner::PreparePlan() {
  // first for both side vancant senario
  if (calc_params_.reverse_arc_vec.size() == 1) {
    if (MonoStepPlan()) {
      // PrintOutputSegmentsInfo();
      std::cout << "mono step plan success!" << std::endl;
      return true;
    }
  }
  std::cout << "mono step plan failed! start foreach" << std::endl;
  // search out
  bool is_prepare_step = true;
  if (PlanFromEgoToParkOutRootPose(is_prepare_step)) {
    std::cout << "foreach success!" << std::endl;
    return true;
  }
  // PrintOutputSegmentsInfo();
  return false;
}

const bool ParallelPathPlanner::PlanFromEgoToParkOutRootPose(
    const bool is_prepare_step) {
  std::vector<pnc::geometry_lib::PathSegment> res;
  res.clear();
  res.reserve(4);

  // construct arc length vec
  std::vector<double> first_arc_length_vec;
  first_arc_length_vec.clear();
  first_arc_length_vec.reserve(50);
  auto first_arc = calc_params_.reverse_arc_vec.back();

  if (!ConstructParkOutArcLengthVec(first_arc_length_vec, first_arc)) {
    DEBUG_PRINT("construct arc length vec failed!");
    return false;
  }
  std::cout << "first arc range = " << first_arc_length_vec.front() << ", "
            << first_arc_length_vec.back() << std::endl;

  const double park_out_target_heading =
      kSearchOutTargetHeading / 57.3 * calc_params_.slot_side_sgn;

  for (const auto& arc_length : first_arc_length_vec) {
    if (!pnc::geometry_lib::CompleteArcInfo(first_arc, arc_length,
                                            first_arc.is_anti_clockwise)) {
      std::cout << "CompleteArcInfo error" << std::endl;
      break;
    }
    std::cout << "first_arc heading =" << first_arc.headingB * 57.3
              << std::endl;

    // if (first_arc.headingB * calc_params_.slot_side_sgn >=
    //     kMaxParkOutFirstArcHeading / 57.3 * calc_params_.slot_side_sgn) {
    //   std::cout << "arc heading too large!" << std::endl;
    //   break;
    // }

    // construct line length vec
    std::vector<double> line_length_vec;
    line_length_vec.clear();
    line_length_vec.reserve(50);

    pnc::geometry_lib::LineSegment line(
        first_arc.pB,
        first_arc.pB + kMaxParkOutLineLength *
                           Eigen::Vector2d(std::cos(first_arc.headingB),
                                           std::sin(first_arc.headingB)),
        first_arc.headingB);

    if (!ConstructParkOutLineLengthVec(line_length_vec, line)) {
      DEBUG_PRINT("construct line length vec failed!");
      continue;
    }
    std::cout << "line range = " << line_length_vec.front() << ", "
              << line_length_vec.back() << std::endl;

    for (const auto& line_length : line_length_vec) {
      if (!pnc::geometry_lib::CompleteLineInfo(line, line_length)) {
        break;
      }

      if (CalcParkOutPath(res, first_arc, line, park_out_target_heading)) {
        if (PlanFromEgoToParkOutEndPose(res.front().GetStartPos(),
                                        res.front().GetStartHeading(),
                                        is_prepare_step)) {
          AddPathSegToOutPut(res);
          std::cout << "arc_length =" << arc_length
                    << ", line_length =" << arc_length << std::endl;
          return true;
        }
      }
    }
  }

  return false;
}

const bool ParallelPathPlanner::MonoStepPlan() {
  if (calc_params_.reverse_arc_vec.size() != 1) {
    DEBUG_PRINT("shouldn't enter mono prepare plan!");
    return false;
  }

  // first try plan to ego line from target
  auto ego_unit = pnc::geometry_lib::BuildLineSegByPose(
      input_.ego_pose.pos, input_.ego_pose.heading);
  pnc::geometry_lib::LineSegment ego_line = ego_unit;

  bool success = false;
  bool first_line_prolong = false;
  pnc::geometry_lib::Arc arc_1;
  pnc::geometry_lib::Arc arc_2;
  arc_1.circle_info.radius = apa_param.GetParam().min_turn_radius;
  const uint8_t arc_1_steer =
      (calc_params_.is_left_side ? pnc::geometry_lib::SEG_STEER_RIGHT
                                 : pnc::geometry_lib::SEG_STEER_LEFT);

  for (const auto& target_pose : calc_params_.valid_target_pt_vec) {
    arc_1.pA = target_pose.pos;
    arc_1.headingA = target_pose.heading;
    arc_1.circle_info.center = CalEgoTurningCenter(
        target_pose, apa_param.GetParam().min_turn_radius, arc_1_steer);

    if (!pnc::geometry_lib::CalTwoSameGearArcWithLine(
            arc_1, arc_2, ego_unit, pnc::geometry_lib::SEG_GEAR_DRIVE)) {
      continue;
    }

    ego_line.heading = input_.ego_pose.heading;
    ego_line.SetPoints(input_.ego_pose.pos, arc_2.pB);
    const auto ego_line_gear = pnc::geometry_lib::CalLineSegGear(ego_line);

    // make sure first line step length are more than min_leng during dirve gear
    // but not too long
    if (input_.is_replan_first) {
      const double min_line_length = apa_param.GetParam().min_line_length + 0.2;
      if (ego_line_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
        if (ego_line.length < min_line_length) {
          const Eigen::Vector2d pt_B =
              ego_line.pA +
              min_line_length * Eigen::Vector2d(std::cos(ego_line.heading),
                                                std::sin(ego_line.heading));
          ego_line.SetPoints(ego_line.pA, pt_B);
          first_line_prolong = true;
        }

        if ((ego_line.pB.x() > 8.0)) {
          first_line_prolong = false;
          continue;
        }
      }
    } else {
      // SEG_GEAR_DRIVE is not allowed in normal backward plan
      if (ego_line_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
        continue;
      }
    }

    // make sure last line step length are more than min_leng during dirve gear
    const pnc::geometry_lib::PathPoint last_line_pose_A(arc_1.pA,
                                                        arc_1.headingA);
    const pnc::geometry_lib::PathPoint last_line_pose_B(
        input_.tlane.pt_terminal_pos, calc_params_.target_line.heading);

    const pnc::geometry_lib::LineSegment last_line_seg(
        last_line_pose_A.pos, last_line_pose_B.pos, last_line_pose_A.heading);

    if (!CheckSamePose(last_line_pose_A, last_line_pose_B)) {
      if (pnc::geometry_lib::CalLineSegGear(last_line_seg) ==
              pnc::geometry_lib::SEG_GEAR_DRIVE &&
          last_line_seg.length <= apa_param.GetParam().min_line_length + 0.2) {
        continue;
      }
    }

    // check collision of first line and two arc,(last line is calculated in
    // inverse loop)
    auto col_res = collision_detector_ptr_->Update(arc_1, arc_1.headingA);
    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist) {
      continue;
    }

    col_res = collision_detector_ptr_->Update(arc_2, arc_2.headingA);
    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist) {
      continue;
    }

    col_res = collision_detector_ptr_->Update(ego_line, ego_line.heading);
    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist) {
      continue;
    }

    success = true;
    break;
  }

  if (success) {
    pnc::geometry_lib::PathSegment arc_seg_1(pnc::geometry_lib::SEG_STEER_LEFT,
                                             pnc::geometry_lib::SEG_GEAR_DRIVE,
                                             arc_1);

    pnc::geometry_lib::PathSegment arc_seg_2(pnc::geometry_lib::SEG_STEER_RIGHT,
                                             pnc::geometry_lib::SEG_GEAR_DRIVE,
                                             arc_2);
    std::cout << "before reverse----------------" << std::endl;
    PrintSegmentInfo(arc_seg_1);
    PrintSegmentInfo(arc_seg_2);

    const pnc::geometry_lib::PathPoint last_line_poseA(arc_1.pA,
                                                       arc_1.headingA);
    const pnc::geometry_lib::PathPoint last_line_poseB(
        input_.tlane.pt_terminal_pos, calc_params_.target_line.heading);

    if (pnc::geometry_lib::ReverseArcSegInfo(arc_seg_1) &&
        pnc::geometry_lib::ReverseArcSegInfo(arc_seg_2)) {
      output_.path_available = true;

      if (!pnc::mathlib::IsDoubleEqual(ego_line.length, 0.0)) {
        AddPathSegToOutPut(pnc::geometry_lib::PathSegment(
            pnc::geometry_lib::CalLineSegGear(ego_line), ego_line));
      }

      if (first_line_prolong) {
        pnc::geometry_lib::LineSegment tmp_line(
            ego_line.pB, arc_seg_2.GetStartPos(), ego_line.heading);

        AddPathSegToOutPut(pnc::geometry_lib::PathSegment(
            pnc::geometry_lib::SEG_GEAR_REVERSE, tmp_line));
      }

      AddPathSegToOutPut(arc_seg_2);
      AddPathSegToOutPut(arc_seg_1);

      if (!CheckSamePose(last_line_poseA, last_line_poseB)) {
        pnc::geometry_lib::LineSegment last_line(
            last_line_poseA.pos, last_line_poseB.pos, last_line_poseA.heading);
        AddPathSegToOutPut(pnc::geometry_lib::PathSegment(
            pnc::geometry_lib::CalLineSegGear(last_line), last_line));
      }

      DEBUG_PRINT(
          "calc prepare mono step from target pose vec to ego pose success!");
      return true;
    }
  }

  return false;
}

const bool ParallelPathPlanner::ConstructParkOutArcLengthVec(
    std::vector<double>& first_arc_length_vec, pnc::geometry_lib::Arc& arc) {
  // calc limit pose
  pnc::geometry_lib::Arc tmp_arc = arc;
  const double length_lim = 9.0;

  if (!CompleteArcInfo(tmp_arc, length_lim, arc.is_anti_clockwise)) {
    return false;
  }

  pnc::geometry_lib::PathSegment arc_seg(
      pnc::geometry_lib::CalArcSteer(tmp_arc),
      pnc::geometry_lib::CalArcGear(tmp_arc), tmp_arc);

  const auto col_res = TrimPathByCollisionDetection(arc_seg);
  DEBUG_PRINT("col_res =" << static_cast<int>(col_res));

  if (col_res == PATH_COL_INVALID) {
    DEBUG_PRINT("calc park out first arc col failed");
    return false;
  }

  DEBUG_PRINT("max practical arc length =" << arc_seg.Getlength());

  double d_heading = kSearchOutArcSampleDistance / arc.circle_info.radius;
  d_heading *= (arc.is_anti_clockwise ? 1.0 : -1.0);

  double arc_length = std::fabs(pnc::geometry_lib::NormalizeAngle(
                          calc_params_.park_out_pose.heading -
                          calc_params_.safe_circle_root_pose.heading)) *
                      arc.circle_info.radius;

  double heading = calc_params_.park_out_pose.heading;
  while (std::fabs(heading) < kMaxParkOutFirstArcHeading / 57.3) {
    if (arc_length > 4.0 || arc_length > arc_seg.Getlength()) {
      break;
    }
    first_arc_length_vec.emplace_back(arc_length);
    heading += d_heading;
    arc_length += kSearchOutArcSampleDistance;
  }
  arc = tmp_arc;
  return true;
}

const bool ParallelPathPlanner::ConstructParkOutLineLengthVec(
    std::vector<double>& line_length_vec,
    pnc::geometry_lib::LineSegment& line) {
  line_length_vec.clear();
  line_length_vec.reserve(50);
  // std::cout << "----------------line vec ---------------" << std::endl;
  double line_length = 0.1;

  Eigen::Vector2d line_limit_pos =
      line.pA + kMaxParkOutLineLength * Eigen::Vector2d(std::cos(line.heading),
                                                        std::sin(line.heading));

  line.SetPoints(line.pA, line_limit_pos);

  pnc::geometry_lib::PathSegment line_seg(pnc::geometry_lib::SEG_GEAR_DRIVE,
                                          line);

  const auto col_res = TrimPathByCollisionDetection(line_seg);

  if (col_res == PATH_COL_INVALID) {
    DEBUG_PRINT("calc park out line col failed");
    return false;
  }

  while (line_length < line_seg.Getlength()) {
    line_length_vec.emplace_back(line_length);
    line_length += kSearchOutLineSampleDistance;
  }
  return true;
}

const bool ParallelPathPlanner::PlanFromEgoToParkOutEndPose(
    const Eigen::Vector2d& park_out_end_pos, const double park_out_end_heading,
    const bool is_prepare_step) {
  const pnc::geometry_lib::PathPoint prepare_pose(park_out_end_pos,
                                                  park_out_end_heading);
  const auto prepare_line = pnc::geometry_lib::BuildLineSegByPose(
      prepare_pose.pos, prepare_pose.heading);

  // std::cout << "prepare pose =" << prepare_pose.pos.transpose() << " "
  //           << prepare_pose.heading * 57.3 << std::endl;

  if (OneStepDubinsPlan(input_.ego_pose, prepare_pose,
                        apa_param.GetParam().min_turn_radius)) {
    GenPathOutputByDubins();
    std::cout << "one step dubins success!" << std::endl;
    return true;
  } else {
    // DEBUG_PRINT("one step dubins failed");
  }

  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
  path_seg_vec.clear();
  path_seg_vec.reserve(4);
  if (is_prepare_step) {
    if (input_.ego_pose.pos.x() <= 8.5) {
      if (TwoSameGearArcPlanToLine(path_seg_vec, input_.ego_pose, prepare_line,
                                   pnc::geometry_lib::SEG_GEAR_DRIVE)) {
        AddPathSegToOutPut(path_seg_vec);
        DEBUG_PRINT("prepare plan: ego to prepare pose success!");
        return true;
      }
    } else {
      if (TwoSameGearArcPlanToLine(path_seg_vec, input_.ego_pose, prepare_line,
                                   pnc::geometry_lib::SEG_GEAR_REVERSE) &&
          path_seg_vec.back().seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
        AddPathSegToOutPut(path_seg_vec);
        DEBUG_PRINT("prepare plan: ego to prepare pose success!");
        return true;
      }
    }
  } else {
    if (TwoSameGearArcPlanToLine(path_seg_vec, input_.ego_pose, prepare_line,
                                 pnc::geometry_lib::SEG_GEAR_REVERSE) &&
        path_seg_vec.back().seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
      AddPathSegToOutPut(path_seg_vec);
      DEBUG_PRINT("prepare plan: ego to prepare pose success!");
      return true;
    }
  }

  // success = RSCurvePlan(input_.ego_pose, prepare_pose,
  //                       apa_param.GetParam().min_turn_radius);

  return false;
}

const bool ParallelPathPlanner::BackwardNormalPlan() {
  // first for both side vancant senario
  if (calc_params_.reverse_arc_vec.size() == 1) {
    if (MonoStepPlan()) {
      // PrintOutputSegmentsInfo();
      return true;
    }
  }

  if (OneStepDubinsPlan(input_.ego_pose, calc_params_.safe_circle_root_pose,
                        apa_param.GetParam().min_turn_radius)) {
    DEBUG_PRINT(
        "triple step plan from ego pose to safe circle root pose success!");
    GenPathOutputByDubins();
    return true;
  }

  if (OneStepDubinsPlan(input_.ego_pose, calc_params_.park_out_pose,
                        apa_param.GetParam().min_turn_radius)) {
    DEBUG_PRINT("triple step plan from ego pose to park out pose success!");
    GenPathOutputByDubins();
    AddLastArc();
    return true;
  }

  // start backward three step plan
  for (const auto& target_pose : calc_params_.valid_target_pt_vec) {
    if (OneStepDubinsPlan(input_.ego_pose, target_pose,
                          apa_param.GetParam().min_turn_radius)) {
      DEBUG_PRINT("plan from ego pose to valid target pose success!"
                  << " valid pos= " << target_pose.pos.transpose());
      GenPathOutputByDubins();
      // PrintOutputSegmentsInfo();
      return true;
    }
  }

  return false;
}

const bool ParallelPathPlanner::OneStepDubinsPlan(
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const double radius) {
  pnc::dubins_lib::DubinsLibrary::Input dubins_input;

  dubins_input.radius = radius;
  dubins_input.Set(start_pose.pos, target_pose.pos, start_pose.heading,
                   target_pose.heading);

  dubins_planner_.SetInput(dubins_input);

  bool success = dubins_planner_.OneStepDubinsUpdate();

  if (success) {
    success = !IsDubinsCollided();
  }

  return success;
}

const bool ParallelPathPlanner::CalcParkOutPath(
    std::vector<pnc::geometry_lib::PathSegment>& reversed_park_out_path,
    const pnc::geometry_lib::Arc& first_arc,
    const pnc::geometry_lib::LineSegment& line,
    const double park_out_target_heading) {
  reversed_park_out_path.clear();
  reversed_park_out_path.reserve(3);

  if (first_arc.length < 0.0 || line.length < 0.0) {
    // std::cout << "input length is less than 0.0!" << std::endl;
    return false;
  }

  const pnc::geometry_lib::PathPoint last_arc_start(line.pB, line.heading);

  // correct heading
  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
  if (!AlignBodyPlan(path_seg_vec, last_arc_start, park_out_target_heading,
                     pnc::geometry_lib::SEG_GEAR_DRIVE)) {
    // std::cout << "align ego body failed!" << std::endl;
    return false;
  }

  const auto& last_arc = path_seg_vec.front().GetArcSeg();
  // std::cout << "last_arc.pB: x y :" << last_arc.pB.transpose() << std::endl;
  if (last_arc.pB.x() < calc_params_.safe_circle_root_pose.pos.x() + 3.0) {
    // std::cout << "x not enough!" << std::endl;
    return false;
  }
  if (last_arc.pB.y() * calc_params_.slot_side_sgn <=
      input_.tlane.pt_inside.y() * calc_params_.slot_side_sgn +
          0.4 * apa_param.GetParam().car_width) {
    return false;
  }

  auto col_res = collision_detector_ptr_->Update(last_arc, last_arc.headingA);
  if (col_res.collision_flag) {
    // std::cout << "line collided!" << std::endl;
    return false;
  }

  std::vector<pnc::geometry_lib::PathSegment> tmp_park_out_path;
  calc_params_.reverse_arc_vec.back() = first_arc;
  tmp_park_out_path.emplace_back(pnc::geometry_lib::CalArcSteer(first_arc),
                                 pnc::geometry_lib::CalArcGear(first_arc),
                                 first_arc);

  tmp_park_out_path.emplace_back(pnc::geometry_lib::SEG_GEAR_DRIVE, line);

  tmp_park_out_path.emplace_back(pnc::geometry_lib::CalArcSteer(last_arc),
                                 pnc::geometry_lib::CalArcGear(last_arc),
                                 last_arc);

  // output is auto filled in dubins_planner_
  for (auto& path_seg : tmp_park_out_path) {
    // std::cout << "path_seg.Getlength()" << path_seg.Getlength() << std::endl;
    if (!pnc::geometry_lib::ReversePathSegInfo(path_seg)) {
      std::cout << "reverse park out path seg error!" << std::endl;
      return false;
    }
  }

  reversed_park_out_path.insert(reversed_park_out_path.end(),
                                tmp_park_out_path.rbegin(),
                                tmp_park_out_path.rend());
  return true;
}

void ParallelPathPlanner::GenPathOutputByDubins() {
  const auto& dubins_output = dubins_planner_.GetOutput();
  output_.path_available = true;
  output_.length = dubins_output.length;
  output_.gear_change_count = dubins_output.gear_change_count;
  output_.current_gear = dubins_output.current_gear_cmd;

  // set arc AB
  if (dubins_output.gear_cmd_vec[0] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_AB = pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalArcSteer(dubins_output.arc_AB),
        pnc::geometry_lib::CalArcGear(dubins_output.arc_AB),
        dubins_output.arc_AB);

    output_.steer_vec.emplace_back(
        pnc::geometry_lib::CalArcSteer(dubins_output.arc_AB));

    output_.gear_cmd_vec.emplace_back(
        pnc::geometry_lib::CalArcGear(dubins_output.arc_AB));

    output_.path_segment_vec.emplace_back(seg_AB);
  }

  // seg line BC
  if (dubins_output.gear_cmd_vec[1] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_BC = pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalLineSegGear(dubins_output.line_BC),
        dubins_output.line_BC);

    output_.steer_vec.emplace_back(pnc::geometry_lib::SEG_STEER_STRAIGHT);
    output_.gear_cmd_vec.emplace_back(
        pnc::geometry_lib::CalLineSegGear(dubins_output.line_BC));
    output_.path_segment_vec.emplace_back(seg_BC);
  }

  // set arc CD
  if (dubins_output.gear_cmd_vec[2] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    auto seg_CD = pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalArcSteer(dubins_output.arc_CD),
        pnc::geometry_lib::CalArcGear(dubins_output.arc_CD),
        dubins_output.arc_CD);

    output_.steer_vec.emplace_back(
        pnc::geometry_lib::CalArcSteer(dubins_output.arc_CD));
    output_.gear_cmd_vec.emplace_back(
        pnc::geometry_lib::CalArcGear(dubins_output.arc_CD));
    output_.path_segment_vec.emplace_back(seg_CD);
  }
}

void ParallelPathPlanner::AddLastArc() {
  const auto& last_reversed_arc = calc_params_.reverse_arc_vec.back();

  pnc::geometry_lib::PathSegment last_path_seg(
      pnc::geometry_lib::CalArcSteer(last_reversed_arc),
      pnc::geometry_lib::CalArcGear(last_reversed_arc), last_reversed_arc);

  if (!pnc::geometry_lib::ReversePathSegInfo(last_path_seg)) {
    return;
  }

  output_.length += last_path_seg.Getlength();
  output_.steer_vec.emplace_back(last_path_seg.seg_steer);
  output_.gear_cmd_vec.emplace_back(last_path_seg.seg_gear);
  output_.path_segment_vec.emplace_back(last_path_seg);
}

void ParallelPathPlanner::GenTriplePath() {
  if (output_.gear_cmd_vec.back() != pnc::geometry_lib::SEG_GEAR_REVERSE) {
    output_.gear_change_count++;
  }

  output_.path_segment_vec.insert(output_.path_segment_vec.end(),
                                  calc_params_.triple_step_path.begin(),
                                  calc_params_.triple_step_path.end());

  for (const auto& path_seg : calc_params_.triple_step_path) {
    output_.length += path_seg.Getlength();
    output_.gear_cmd_vec.emplace_back(path_seg.seg_gear);
    output_.steer_vec.emplace_back(path_seg.seg_steer);
  }
}

const bool ParallelPathPlanner::IsDubinsCollided() {
  bool is_collided = false;
  bool is_arc = false;
  pnc::geometry_lib::Arc arc;
  pnc::geometry_lib::LineSegment line;
  CollisionDetector::CollisionResult col_res;
  for (size_t i = 0; i < 3; i++) {
    if (i == 0) {
      arc = dubins_planner_.GetOutput().arc_AB;
      is_arc = true;
    } else if (i == 1) {
      line = dubins_planner_.GetOutput().line_BC;
      is_arc = false;
    } else if (i == 2) {
      arc = dubins_planner_.GetOutput().arc_CD;
      is_arc = true;
    }

    if (is_arc && !arc.is_ignored) {
      col_res = collision_detector_ptr_->Update(arc, arc.headingA);
    } else if (!is_arc && !line.is_ignored) {
      col_res = collision_detector_ptr_->Update(line, line.heading);
    }

    if (col_res.collision_flag) {
      is_collided = true;
      break;
    }
  }
  return is_collided;
}

void ParallelPathPlanner::AddPathSegToOutPut(
    const std::vector<pnc::geometry_lib::PathSegment>& path_seg) {
  output_.path_available = true;

  for (const auto& seg : path_seg) {
    if (output_.gear_cmd_vec.size() > 0) {
      if (output_.gear_cmd_vec.back() != seg.seg_gear) {
        output_.gear_change_count++;
      }
    }
    output_.path_segment_vec.emplace_back(seg);
    output_.length += seg.Getlength();
    output_.gear_cmd_vec.emplace_back(seg.seg_gear);
    output_.steer_vec.emplace_back(seg.seg_steer);
  }
}

const bool ParallelPathPlanner::CheckEgoInSlot() const {
  // Todo: use slot occupied ratio
  return ((input_.ego_pose.pos.y() * calc_params_.slot_side_sgn < 1.4) &&
          (input_.ego_pose.pos.x() > input_.tlane.pt_outside.x() - 2.0) &&
          (input_.ego_pose.pos.x() < input_.tlane.pt_inside.x() + 2.0));
}

const bool ParallelPathPlanner::CalMinSafeCircle() {
  // search from target pose to current pose
  pnc::geometry_lib::PathPoint target_pose(input_.tlane.pt_terminal_pos,
                                           calc_params_.target_line.heading);

  std::vector<pnc::geometry_lib::PathSegment> search_out_res;

  bool success = InverseSearchLoopInSlot(
      target_pose, calc_params_.safe_circle_root_pose, search_out_res);
  std::cout << "park out root pt (deg) ="
            << calc_params_.safe_circle_root_pose.pos.transpose() << " , "
            << calc_params_.safe_circle_root_pose.heading * 57.3 << std::endl;

  // calc pose with ego had just cross pt_inside'y, which is used to connect the
  // prepare point
  if (success) {
    CalcParkOutPose();
    std::cout << "park out pose ="
              << calc_params_.reverse_arc_vec.back().pB.transpose() << " "
              << calc_params_.reverse_arc_vec.back().headingB * 57.3
              << std::endl;
  }

  return success;
}

const bool ParallelPathPlanner::CalcParkOutPose() {
  // the park out pt is the pose where ego front vertex has just crossed
  // pt_inside y coordination

  if (calc_params_.reverse_arc_vec.size() == 0) {
    std::cout << "has no inverse info!" << std::endl;
    return false;
  }

  auto& front_arc = calc_params_.reverse_arc_vec.back();

  const pnc::geometry_lib::PathPoint start_pose(front_arc.pA,
                                                front_arc.headingA);

  if (!CheckSamePose(start_pose, calc_params_.safe_circle_root_pose)) {
    std::cout << "error: start pose diffs " << std::endl;
    return false;
  }

  // make sure which vertex is front outside one of ego car
  const size_t front_vertex_idx = calc_params_.is_left_side ? 0 : 5;

  const Eigen::Vector2d v_vertex(
      apa_param.GetParam().car_vertex_x_vec[front_vertex_idx],
      apa_param.GetParam().car_vertex_y_vec[front_vertex_idx]);

  const double d_theta_sgn = (front_arc.is_anti_clockwise ? 1.0 : -1.0);

  const double d_theta =
      d_theta_sgn * input_.sample_ds / front_arc.circle_info.radius;

  bool success = false;
  pnc::geometry_lib::LocalToGlobalTf ego2slot;
  auto park_out_pose = start_pose;
  const Eigen::Matrix2d rot_m = pnc::geometry_lib::GetRotm2dFromTheta(d_theta);

  const double max_park_out_heading =
      kMaxParkOutFirstArcHeading / 57.3 * calc_params_.slot_side_sgn;

  while (park_out_pose.heading * calc_params_.slot_side_sgn <=
         max_park_out_heading) {
    ego2slot.Init(park_out_pose.pos, park_out_pose.heading);
    const Eigen::Vector2d vertex_slot = ego2slot.GetPos(v_vertex);

    if (vertex_slot.y() * calc_params_.slot_side_sgn >=
        input_.tlane.pt_inside.y() * calc_params_.slot_side_sgn) {
      success = true;
      break;
    }

    const Eigen::Vector2d v_oa =
        park_out_pose.pos - front_arc.circle_info.center;

    park_out_pose.pos = rot_m * v_oa + front_arc.circle_info.center;
    park_out_pose.heading += d_theta;
  }

  if (success) {
    front_arc.pB = park_out_pose.pos;
    front_arc.headingB = park_out_pose.heading;
    front_arc.length = front_arc.circle_info.radius *
                       std::fabs(front_arc.headingB - front_arc.headingA);

    calc_params_.park_out_pose = park_out_pose;
  }

  return success;
}

const bool ParallelPathPlanner::InverseSearchLoopInSlot(
    const pnc::geometry_lib::PathPoint& terminal_pose,
    pnc::geometry_lib::PathPoint& park_out_pose,
    std::vector<pnc::geometry_lib::PathSegment>& search_out_res) {
  calc_params_.reverse_arc_vec.clear();
  const double radius = apa_param.GetParam().min_turn_radius;

  const uint8_t forward_steer =
      (calc_params_.is_left_side ? pnc::geometry_lib::SEG_STEER_RIGHT
                                 : pnc::geometry_lib::SEG_STEER_LEFT);

  // calc backward limit
  pnc::geometry_lib::LineSegment first_line_step;
  first_line_step.pA = terminal_pose.pos;
  first_line_step.heading = terminal_pose.heading;
  if (!CalcLineStepLimitPose(first_line_step,
                             pnc::geometry_lib::SEG_GEAR_REVERSE)) {
    // std::cout << "CalcLineStepLimitPose error!" << std::endl;
    return false;
  }

  const pnc::geometry_lib::PathPoint backward_line_limit_pose(
      first_line_step.pB, first_line_step.heading);

  if (!CheckSamePose(backward_line_limit_pose, terminal_pose)) {
    std::cout << "need backward step!\n" << std::endl;
    search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::SEG_GEAR_REVERSE,
        pnc::geometry_lib::LineSegment(terminal_pose.pos,
                                       backward_line_limit_pose.pos,
                                       terminal_pose.heading)));
  }

  // get valid target pose
  std::vector<double> valid_target_x_vec = pnc::geometry_lib::Linspace(
      terminal_pose.pos.x(), backward_line_limit_pose.pos.x(), 0.3);

  pnc::geometry_lib::PathPoint valid_pose = terminal_pose;
  for (const auto& x : valid_target_x_vec) {
    valid_pose.pos.x() = x;
    calc_params_.valid_target_pt_vec.emplace_back(valid_pose);
  }

  // check if ego is able to park out in start pose
  bool is_dirve_out_safe = false;
  pnc::geometry_lib::Arc forward_arc;
  forward_arc.pA = terminal_pose.pos;
  forward_arc.headingA = terminal_pose.heading;
  forward_arc.circle_info.radius = radius;

  if (!CalcArcStepLimitPose(forward_arc, is_dirve_out_safe,
                            pnc::geometry_lib::SEG_GEAR_DRIVE, forward_steer)) {
    // std::cout << "calc forward arc limit error!" << std::endl;
    return false;
  }

  if (is_dirve_out_safe) {
    park_out_pose = terminal_pose;
    calc_params_.reverse_arc_vec.emplace_back(forward_arc);
    std::cout << "ego can park out at first!" << std::endl;
    return true;
  }

  // std::cout << "ego can't park out at first!\n" << std::endl;

  // std::cout << "-------------- start loop -----------------------" <<
  // std::endl;
  bool loop_success = false;
  pnc::geometry_lib::PathPoint start_pose_in_loop = backward_line_limit_pose;
  const auto backward_steer = pnc::geometry_lib::ReverseSteer(forward_steer);
  for (size_t i = 0; i < kMaxPathNumsInSlot; i += 2) {
    // start calc forward limit pose
    // std::cout << "---------- No. " << i << "loop search ---------------"
    //           << std::endl;
    // std::cout << "------ forward step --------" << std::endl;
    // std::cout << "start pose in loop = " <<
    // start_pose_in_loop.pos.transpose()
    //           << "  " << start_pose_in_loop.heading * 57.3 << std::endl;

    pnc::geometry_lib::Arc forward_arc;
    forward_arc.pA = start_pose_in_loop.pos;
    forward_arc.headingA = start_pose_in_loop.heading;
    forward_arc.circle_info.radius = radius;

    if (!CalcArcStepLimitPose(forward_arc, is_dirve_out_safe,
                              pnc::geometry_lib::SEG_GEAR_DRIVE,
                              forward_steer)) {
      // std::cout << "calc forward arc limit error!" << std::endl;
      break;
    }
    // std::cout << "calculated forward limit pose = "
    //           << forward_arc.pB.transpose() << "  "
    //           << forward_arc.headingB * 57.3 << std::endl;
    // std::cout << "arc length = " << forward_arc.length << std::endl;

    pnc::geometry_lib::PathPoint forward_limit_pose;
    forward_limit_pose.Set(forward_arc.pB, forward_arc.headingB);
    if (CheckSamePose(start_pose_in_loop, forward_limit_pose)) {
      // std::cout << "can't dirve in forward loop!" << std::endl;
      break;
    }
    calc_params_.reverse_arc_vec.emplace_back(forward_arc);

    // ego can park out in forward step, return true
    if (is_dirve_out_safe) {
      loop_success = true;
      park_out_pose = start_pose_in_loop;
      std::cout << "find park out pt!" << std::endl;
      break;
    }

    // std::cout << "forward limit pose is valid\n" << std::endl;

    // std::cout << "------ backward step --------" << std::endl;
    // start calc backward limit pose
    pnc::geometry_lib::PathPoint backward_limit_pose;
    pnc::geometry_lib::Arc backward_arc;
    backward_arc.pA = forward_arc.pB;
    backward_arc.headingA = forward_arc.headingB;
    backward_arc.circle_info.radius = radius;
    // std::cout << "backward_steer " << backward_steer << std::endl;

    if (!CalcArcStepLimitPose(backward_arc, is_dirve_out_safe,
                              pnc::geometry_lib::SEG_GEAR_REVERSE,
                              backward_steer)) {
      // std::cout << "calc backward arc limit error!" << std::endl;
      break;
    }

    backward_limit_pose.Set(backward_arc.pB, backward_arc.headingB);
    if (CheckSamePose(backward_limit_pose, forward_limit_pose)) {
      // std::cout << "can't reverse in backward loop!" << std::endl;
      break;
    }
    // std::cout << "calculated backward limit pose = "
    //           << backward_arc.pB.transpose() << "  "
    //           << backward_arc.headingB * 57.3 << std::endl;
    // std::cout << "arc length = " << backward_arc.length << std::endl;

    calc_params_.reverse_arc_vec.emplace_back(backward_arc);
    start_pose_in_loop = backward_limit_pose;
  }
  return loop_success;
}

const bool ParallelPathPlanner::CalcLineStepLimitPose(
    pnc::geometry_lib::LineSegment& line, const uint8_t gear) {
  // start pose should be given in line
  pnc::geometry_lib::PathPoint start_pose(line.pA, line.heading);

  double dirve_sgn = 1.0;
  if (gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    dirve_sgn = 1.0;
  } else if (gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    dirve_sgn = -1.0;
  } else {
    std::cout << "fault gear type!" << std::endl;
    return false;
  }

  const double line_dist_limit = 3.5;

  const Eigen::Vector2d rough_limit_pt =
      start_pose.pos + line_dist_limit * dirve_sgn *
                           Eigen::Vector2d(std::cos(start_pose.heading),
                                           std::sin(start_pose.heading));

  pnc::geometry_lib::PathSegment line_path(
      gear, pnc::geometry_lib::LineSegment(start_pose.pos, rough_limit_pt,
                                           start_pose.heading));

  const auto& col_res = TrimPathByCollisionDetection(line_path);

  line.is_ignored = true;
  // check if ego can park out at limit pose
  if (col_res == PATH_COL_NORMAL || col_res == PATH_COL_SHORTEN) {
    if (line_path.GetLineSeg().length >= apa_param.GetParam().min_line_length) {
      line.pB = line_path.GetLineSeg().pB;
      line.length = (line.pB - line.pA).norm();
      line.is_ignored = false;
    }
  }

  if (line.is_ignored) {
    line.pB = line.pA;
    line.length = 0.0;
  }
  return true;
}

const bool ParallelPathPlanner::CalcArcStepLimitPose(
    pnc::geometry_lib::Arc& arc, bool& is_drive_out_safe, const uint8_t gear,
    const uint8_t steer) {
  // start pose and radius should be given in arc

  if (arc.circle_info.radius <
      apa_param.GetParam().min_turn_radius - apa_param.GetParam().radius_eps) {
    std::cout << "radius fault!" << std::endl;
    return false;
  }

  if (gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
      gear != pnc::geometry_lib::SEG_GEAR_REVERSE) {
    std::cout << "arc fault gear type!" << std::endl;
    return false;
  }

  if (steer != pnc::geometry_lib::SEG_STEER_RIGHT &&
      steer != pnc::geometry_lib::SEG_STEER_LEFT) {
    std::cout << "arc fault steer type!" << std::endl;
    return false;
  }

  is_drive_out_safe = false;
  pnc::geometry_lib::PathPoint start_pose(arc.pA, arc.headingA);

  arc.circle_info.center =
      CalEgoTurningCenter(start_pose, arc.circle_info.radius, steer);

  if (!pnc::geometry_lib::CalcArcDirection(arc.is_anti_clockwise, gear,
                                           steer)) {
    std::cout << "arc or steer error!" << std::endl;
    return false;
  }

  const double arc_length_limit = 8.0;
  arc.length = arc_length_limit;
  const double theta_diff_sgn = (arc.is_anti_clockwise ? 1.0 : -1.0);

  const double theta_diff =
      arc_length_limit / arc.circle_info.radius * theta_diff_sgn;

  arc.headingB = arc.headingA + theta_diff;

  const Eigen::Vector2d v_oa = arc.pA - arc.circle_info.center;
  const Eigen::Matrix2d rot_m =
      pnc::geometry_lib::GetRotm2dFromTheta(theta_diff);
  arc.pB = rot_m * v_oa + arc.circle_info.center;

  Eigen::Vector2d forward_col_pt;
  pnc::geometry_lib::PathSegment arc_path(steer, gear, arc);
  const auto& col_res = TrimPathByCollisionDetection(arc_path, forward_col_pt);
  std::cout << "forward_col_pt =" << forward_col_pt.transpose() << std::endl;

  arc.is_ignored = true;
  // check if ego can park out at limit pose
  if (col_res == PATH_COL_NORMAL || col_res == PATH_COL_SHORTEN) {
    if (arc_path.GetArcSeg().length >= apa_param.GetParam().min_line_length) {
      arc.is_ignored = false;
      arc.pB = arc_path.GetArcSeg().pB;
      arc.headingB = arc_path.GetArcSeg().headingB;

      const double trim_theta_diff =
          pnc::geometry_lib::NormalizeAngle(arc.headingB - arc.headingA);

      arc.length = std::fabs(trim_theta_diff) * arc.circle_info.radius;
    }
  }

  if (gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    if (col_res == PATH_COL_NORMAL) {
      // path is long enough, can park out
      is_drive_out_safe = true;
    } else if (col_res == PATH_COL_SHORTEN) {
      const bool tlane_col_x_condition = pnc::mathlib::IsInBound(
          forward_col_pt.x(),
          input_.tlane.pt_inside.x() + apa_param.GetParam().static_pos_eps,
          input_.tlane.pt_outside.x() - apa_param.GetParam().static_pos_eps);

      std::cout << "calc_params_.slot_side_sgn =" << calc_params_.slot_side_sgn
                << std::endl;

      const bool tlane_col_y_condition =
          forward_col_pt.y() * calc_params_.slot_side_sgn <=
          (input_.tlane.pt_inside.y() + apa_param.GetParam().static_pos_eps) *
              calc_params_.slot_side_sgn;

      if (tlane_col_x_condition && tlane_col_y_condition) {
        is_drive_out_safe = false;
      } else {
        is_drive_out_safe = true;
      }
      std::cout << "is_drive_out_safe =" << is_drive_out_safe << std::endl;
    }
  }

  if (arc.is_ignored) {
    arc.pB = arc.pA;
    arc.headingB = arc.headingA;
    arc.length = 0.0;
  }

  return true;
}

const bool ParallelPathPlanner::TwoSameGearArcPlanToLine(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::LineSegment& target_line, const uint8_t gear) {
  path_seg_vec.clear();
  path_seg_vec.reserve(4);

  if (gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
      gear != pnc::geometry_lib::SEG_GEAR_REVERSE) {
    return false;
  }
  const auto v_start =
      pnc::geometry_lib::GetUnitTangVecByHeading(start_pose.heading);

  const auto v_target =
      pnc::geometry_lib::GetUnitTangVecByHeading(target_line.heading);

  if (std::fabs(v_start.dot(v_target)) > 1.414) {
    std::cout << "heading diff is too more than 45 deg!" << std::endl;
    return false;
  }

  pnc::geometry_lib::Arc arc_1;
  pnc::geometry_lib::Arc arc_2;
  arc_1.pA = start_pose.pos;
  arc_1.headingA = start_pose.heading;
  arc_1.circle_info.radius = apa_param.GetParam().min_turn_radius;

  const uint8_t arc_1_steer = (pnc::geometry_lib::IsPointOnLeftSideOfLineSeg(
                                   start_pose.pos, target_line)
                                   ? pnc::geometry_lib::SEG_STEER_RIGHT
                                   : pnc::geometry_lib::SEG_STEER_LEFT);

  arc_1.circle_info.center = CalEgoTurningCenter(
      start_pose, apa_param.GetParam().min_turn_radius, arc_1_steer);

  auto tmp_target_line = target_line;
  if (!pnc::geometry_lib::CalTwoSameGearArcWithLine(arc_1, arc_2,
                                                    tmp_target_line, gear)) {
    return false;
  }

  auto col_res = collision_detector_ptr_->Update(arc_1, arc_1.headingA);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist) {
    return false;
  }
  col_res = collision_detector_ptr_->Update(arc_2, arc_2.headingA);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist) {
    return false;
  }

  pnc::geometry_lib::LineSegment last_line(arc_2.pB, target_line.pA,
                                           arc_2.headingB);
  auto last_line_gear = pnc::geometry_lib::CalLineSegGear(last_line);
  col_res = collision_detector_ptr_->Update(last_line, last_line.heading);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist) {
    return false;
  }

  bool is_prolonged = false;
  auto prolonged_line = last_line;
  if (last_line_gear != gear && last_line.length < 0.3) {
    const double gear_sgn =
        (gear == pnc::geometry_lib::SEG_GEAR_DRIVE ? 1.0 : -1.0);

    const auto v_line_heading =
        pnc::geometry_lib::GetUnitTangVecByHeading(arc_2.headingB);

    const auto prolonged_pt = arc_2.pB + 0.3 * v_line_heading * gear_sgn;
    prolonged_line.SetPoints(arc_2.pB, prolonged_pt);

    col_res =
        collision_detector_ptr_->Update(prolonged_line, prolonged_line.heading);
    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist) {
      return false;
    }
    is_prolonged = true;
    last_line.SetPoints(prolonged_line.pB, target_line.pA);
  }

  path_seg_vec.emplace_back(
      pnc::geometry_lib::PathSegment(arc_1_steer, gear, arc_1));

  path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
      pnc::geometry_lib::CalArcSteer(arc_2), gear, arc_2));

  if (is_prolonged) {
    path_seg_vec.emplace_back(
        pnc::geometry_lib::PathSegment(gear, prolonged_line));
  }

  if (last_line.length > 0.05) {
    path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalLineSegGear(last_line), last_line));
  }

  return true;
}

const bool ParallelPathPlanner::RSCurvePlan(
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const double radius) {
  pnc::dubins_lib::DubinsLibrary::Input dubins_input;
  dubins_input.Set(current_pose.pos, target_pose.pos, current_pose.heading,
                   target_pose.heading);
  dubins_input.radius = radius;

  dubins_planner_.SetInput(dubins_input);

  // try dubins method
  for (size_t i = 0; i < pnc::dubins_lib::DubinsLibrary::CASE_COUNT; ++i) {
    for (size_t j = 0; j < pnc::dubins_lib::DubinsLibrary::DUBINS_TYPE_COUNT;
         ++j) {
      if (dubins_planner_.Solve(j, i)) {
        if (dubins_planner_.GetOutput().gear_change_count < 2 &&
            dubins_planner_.GetOutput().gear_cmd_vec.back() ==
                pnc::geometry_lib::SEG_GEAR_REVERSE) {
          if (!IsDubinsCollided()) {
            // std::cout << "dubins success!" << std::endl;
            return true;
          }
        }
      }
    }
  }
  // std::cout << "dubins failed!" << std::endl;
  // try line arc method
  for (size_t i = 0; i < pnc::dubins_lib::DubinsLibrary::LINEARC_TYPE_COUNT;
       ++i) {
    if (dubins_planner_.Solve(i)) {
      if (dubins_planner_.GetOutput().gear_change_count < 2 &&
          dubins_planner_.GetOutput().line_arc_radius >= radius &&
          dubins_planner_.GetOutput().line_arc_radius <= 18.0 &&
          dubins_planner_.GetOutput().gear_cmd_vec.back() ==
              pnc::geometry_lib::SEG_GEAR_REVERSE) {
        if (!IsDubinsCollided()) {
          // std::cout << "line arc success!" << std::endl;
          return true;
        }
      }
    }
  }

  return false;
}

// multi plan start
const bool ParallelPathPlanner::MultiPlan() {
  std::cout << "-----multi plan-----\n";
  // set init state
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;

  // check pose and slot_occupied_ratio, if error is small, multi isnot suitable
  if (!CheckMultiPlanSuitable(current_pose)) {
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
  for (size_t i = 0; i < kMaxMultiStepNums; ++i) {
    std::cout << "-------- No." << i << " in multi-plan--------\n";
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);
    success = CalSinglePathInMulti(current_pose, current_gear,
                                   current_arc_steer, tmp_path_seg_vec);
    if (!success) {
      std::cout << "single path of multi-plan failed!\n\n";
      // output_.Reset();
      break;
    }
    output_.path_available = true;
    AddPathSegToOutPut(tmp_path_seg_vec);

    if (output_.path_segment_vec.size() > 0) {
      const auto& last_segment = output_.path_segment_vec.back();

      pnc::geometry_lib::PathPoint last_pose(last_segment.GetEndPos(),
                                             last_segment.GetEndHeading());

      current_pose = last_pose;
      current_gear = pnc::geometry_lib::ReverseGear(last_segment.seg_gear);

      current_arc_steer =
          pnc::geometry_lib::ReverseSteer(last_segment.seg_steer);
    } else {
      current_gear = pnc::geometry_lib::ReverseGear(current_gear);
      current_arc_steer = pnc::geometry_lib::ReverseSteer(current_arc_steer);
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
    std::cout << "multi plan failed!" << std::endl;
  }
  return success;
}

// checkout multi suitable
const bool ParallelPathPlanner::CheckMultiPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose) const {
  if (std::fabs(current_pose.heading) <=
      apa_param.GetParam().multi_plan_min_heading_err / 57.3) {
    return false;
  }
  return true;
}

const bool ParallelPathPlanner::CalSinglePathInMulti(
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
  if ((last_pose.pos - input_.tlane.pt_terminal_pos).norm() <=
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

  int i = -1;
  uint8_t path_col_det_res = PATH_COL_COUNT;
  for (auto& tmp_path_seg : tmp_path_seg_vec) {
    i++;
    path_col_det_res = TrimPathByCollisionDetection(tmp_path_seg);

    if (path_col_det_res == PATH_COL_NORMAL) {
      path_seg_vec.emplace_back(tmp_path_seg);
      DEBUG_PRINT("No. " << i << " normal!");
    } else if (path_col_det_res == PATH_COL_SHORTEN) {
      path_seg_vec.emplace_back(tmp_path_seg);
      DEBUG_PRINT(
          "No. "
          << i << " seg "
          << "cut partial of multi single path due to collision, end point ="
          << tmp_path_seg.GetEndPos().transpose()
          << ", heading =" << tmp_path_seg.GetEndHeading() * 57.3);
      break;
    } else if (path_col_det_res == PATH_COL_INVALID) {
      DEBUG_PRINT("path col invalid!");
      break;
    }
  }
  if (path_col_det_res == PATH_COL_NORMAL) {
    DEBUG_PRINT("single path normal, collision free!");
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

// adjust plan start
const bool ParallelPathPlanner::AdjustPlan() {
  DEBUG_PRINT("-----adjust plan-----");
  // set init state
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;

  if (output_.path_segment_vec.size() > 0 && output_.gear_cmd_vec.size() > 0) {
    const auto& last_seg = output_.path_segment_vec.back();
    current_pose.Set(last_seg.GetEndPos(), last_seg.GetEndHeading());
    current_gear = pnc::geometry_lib::ReverseGear(last_seg.seg_gear);
    current_arc_steer = pnc::geometry_lib::ReverseSteer(last_seg.seg_steer);
    std::cout << "continue to plan after multi\n";
  }

  DEBUG_PRINT("adjust plan input gear =" << static_cast<int>(current_gear));

  DEBUG_PRINT(
      "adjust plan input steer =" << static_cast<int>(current_arc_steer));

  std::cout << "current pose =" << current_pose.pos.transpose() << ", "
            << current_pose.heading * 57.3 << std::endl;

  // check pose, if error is large, adjust is not suitable
  // if (!CheckAdjustPlanSuitable(current_pose)) {
  //   DEBUG_PRINT("pose err is relatively large, skip adjust plan, plan fail");
  //   return false;
  // }

  // check gear and steer
  if (!pnc::geometry_lib::IsValidGear(current_gear) ||
      !pnc::geometry_lib::IsValidArcSteer(current_arc_steer)) {
    DEBUG_PRINT("ref_gear or ref_arc_steer error");
    return false;
  }

  // DEBUG_PRINT("try adjust plan to target point");
  bool success = false;
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;

  for (size_t i = 0; i < kMaxPathNumsInSlot; ++i) {
    DEBUG_PRINT("-------- No." << i << " in adjust-plan--------");
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);

    if (!CalSinglePathInAdjust(tmp_path_seg_vec, current_pose, current_gear,
                               1.0, apa_param.GetParam().min_turn_radius)) {
      DEBUG_PRINT("single path of adjust plan failed!");
      success = false;
      output_.Reset();
      break;
    }
    success = true;
    output_.path_available = true;

    if (tmp_path_seg_vec.size() > 0) {
      AddPathSegVecToOutput(tmp_path_seg_vec);
      const auto& last_segment = output_.path_segment_vec.back();

      current_pose.Set(last_segment.GetEndPos(), last_segment.GetEndHeading());
      current_gear = pnc::geometry_lib::ReverseGear(last_segment.seg_gear);
      current_arc_steer =
          pnc::geometry_lib::ReverseSteer(last_segment.seg_steer);
    } else {
      current_gear = pnc::geometry_lib::ReverseGear(current_gear);
      current_arc_steer = pnc::geometry_lib::ReverseSteer(current_arc_steer);
    }

    pnc::geometry_lib::PathPoint target_pose(input_.tlane.pt_terminal_pos,
                                             calc_params_.target_line.heading);

    if (CheckSamePose(current_pose, target_pose)) {
      break;
    }
  }

  if (!success) {
    std::cout << "adjust plan failed!" << std::endl;
  }

  return success;
}

// adjust plan start
const bool ParallelPathPlanner::ParallelAdjustPlan() {
  DEBUG_PRINT("-----prallel adjust plan-----");
  // set init state
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;

  if (output_.path_segment_vec.size() > 0 && output_.gear_cmd_vec.size() > 0) {
    const auto& last_seg = output_.path_segment_vec.back();
    current_pose.Set(last_seg.GetEndPos(), last_seg.GetEndHeading());
    current_gear = pnc::geometry_lib::ReverseGear(last_seg.seg_gear);
    current_arc_steer = pnc::geometry_lib::ReverseSteer(last_seg.seg_steer);
    std::cout << "continue to plan after multi\n";
  }

  std::cout << "current pose =" << current_pose.pos.transpose() << ", "
            << current_pose.heading * 57.3 << std::endl;

  // check pose, if error is large, adjust is not suitable
  // if (!CheckAdjustPlanSuitable(current_pose)) {
  //   DEBUG_PRINT("pose err is relatively large, skip adjust plan, plan fail");
  //   return false;
  // }

  // check gear and steer
  if (!pnc::geometry_lib::IsValidGear(current_gear) ||
      !pnc::geometry_lib::IsValidArcSteer(current_arc_steer)) {
    DEBUG_PRINT("ref_gear or ref_arc_steer error");
    return false;
  }

  std::cout << "input gear =" << static_cast<int>(current_gear) << std::endl;

  // DEBUG_PRINT("try adjust plan to target point");
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(5);
  // first try one arc
  bool success = OneArcPlan(tmp_path_seg_vec, current_pose, current_gear);
  if (success) {
    if (!CheckPathSegCollided(tmp_path_seg_vec.back())) {
      std::cout << "only one arc success!" << std::endl;
      AddPathSegVecToOutput(tmp_path_seg_vec);
      return true;
    }
  }
  success = false;
  DEBUG_PRINT("One arc failed!");

  // secondly check if should align body
  if (!pnc::mathlib::IsDoubleEqual(current_pose.heading,
                                   calc_params_.target_line.heading)) {
    // need align body
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(5);
    if (AlignBodyPlan(tmp_path_seg_vec, current_pose, current_gear)) {
      if (!CheckPathSegCollided(tmp_path_seg_vec.back())) {
        std::cout << "align body success!" << std::endl;
        AddPathSegVecToOutput(tmp_path_seg_vec);
        success = true;
        current_pose.Set(tmp_path_seg_vec.back().GetEndPos(),
                         tmp_path_seg_vec.back().GetEndHeading());
      }
    }
    if (!success) {
      DEBUG_PRINT("align body failed!");
      return false;
    }
  }

  DEBUG_PRINT("body already align");
  bool loop_success = false;
  std::vector<pnc::geometry_lib::PathSegment> parallel_shift_path_vec;
  parallel_shift_path_vec.clear();
  parallel_shift_path_vec.reserve(7);
  for (size_t i = 0; i < kMaxParallelShiftNums; ++i) {
    DEBUG_PRINT("-------- No." << i << " in paralle adjust-plan--------");

    bool s_turn_success = false;
    std::vector<pnc::geometry_lib::PathSegment> s_turn_vec;
    double ratio = 1.0;
    for (; ratio > 0.1; ratio -= 0.2) {
      s_turn_vec.clear();
      s_turn_vec.reserve(2);
      if (STurnParallelPlan(s_turn_vec, current_pose, calc_params_.target_line,
                            current_gear, ratio,
                            apa_param.GetParam().min_turn_radius)) {
        if (!CheckPathSegVecCollided(s_turn_vec, 0.1)) {
          s_turn_success = true;
          parallel_shift_path_vec.insert(parallel_shift_path_vec.end(),
                                         s_turn_vec.begin(), s_turn_vec.end());
          break;
        }
      }
    }
    // Todo: need reverse gear and check s turn again
    if (!s_turn_success) {
      loop_success = false;
      DEBUG_PRINT("shift plan fail with current gear");
      break;
    }

    // sturn success
    if (ratio == 1.0) {
      loop_success = true;
      DEBUG_PRINT("shit loop success, try last line!");
      break;
    }
    // ratio is less than 1.0
    current_gear = pnc::geometry_lib::ReverseGear(current_gear);
    current_pose.Set(s_turn_vec.back().GetEndPos(),
                     s_turn_vec.back().GetEndHeading());
    current_arc_steer =
        pnc::geometry_lib::ReverseSteer(s_turn_vec.back().seg_steer);
  }

  if (loop_success) {
    pnc::geometry_lib::PathPoint loop_end_pose(
        parallel_shift_path_vec.back().GetEndPos(),
        parallel_shift_path_vec.back().GetEndHeading());

    pnc::geometry_lib::PathPoint target_pose(input_.tlane.pt_terminal_pos,
                                             calc_params_.target_line.heading);
    if (!CheckSamePose(current_pose, target_pose)) {
      pnc::geometry_lib::LineSegment last_line;
      last_line.pA = loop_end_pose.pos;
      last_line.heading = loop_end_pose.heading;

      if (OneLinePlan(last_line, parallel_shift_path_vec, input_.ref_gear)) {
        std::cout << "OneLinePlan success\n";
      } else {
        loop_success = false;
        std::cout << "OneLinePlan fail\n";
      }
    }
  }

  if (loop_success) {
    AddPathSegVecToOutput(parallel_shift_path_vec);
  }
  return loop_success;
}

const bool ParallelPathPlanner::CheckAdjustPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose) const {
  return (std::fabs(current_pose.heading) <=
              apa_param.GetParam().adjust_plan_max_heading1_err / 57.3 ||
          (std::fabs(current_pose.heading) <=
               apa_param.GetParam().adjust_plan_max_heading2_err / 57.3 &&
           std::fabs(current_pose.pos.y()) <=
               apa_param.GetParam().adjust_plan_max_lat_err));
}

const bool ParallelPathPlanner::OneArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear, const uint8_t current_arc_steer) {
  if (!CalOneArcWithLine(arc, calc_params_.target_line,
                         apa_param.GetParam().radius_eps)) {
    DEBUG_PRINT("OneArcPlan fail 0");
    return false;
  }

  uint8_t steer = pnc::geometry_lib::CalArcSteer(arc);
  uint8_t gear = pnc::geometry_lib::CalArcGear(arc);
  if (steer != current_arc_steer || gear != current_gear) {
    DEBUG_PRINT("OneArcPlan fail 1");
    return false;
  }
  pnc::geometry_lib::PathSegment arc_seg(steer, gear, arc);
  path_seg_vec.emplace_back(arc_seg);
  return true;
}

const bool ParallelPathPlanner::OneArcPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  // DEBUG_PRINT("try one arc plan");

  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;

  if (pnc::mathlib::IsDoubleEqual(arc.headingA,
                                  calc_params_.target_line.heading) ||
      pnc::mathlib::IsDoubleEqual(arc.pA.y(),
                                  calc_params_.target_line.pA.y())) {
    // DEBUG_PRINT(
    //     "current heading is equal to target heading or current y is equal to
    //     " "target y, no need to one arc plan");
    return false;
  }

  bool success = pnc::geometry_lib::CalOneArcWithLineAndGear(
      arc, calc_params_.target_line, current_gear);

  if (success) {
    // check radius and gear can or not meet needs
    // DEBUG_PRINT("cal radius = " << arc.circle_info.radius);
    const auto steer = pnc::geometry_lib::CalArcSteer(arc);
    const auto gear = pnc::geometry_lib::CalArcGear(arc);

    success = (arc.circle_info.radius >=
                   apa_param.GetParam().min_turn_radius - 1e-3 &&
               arc.circle_info.radius <=
                   apa_param.GetParam().max_one_step_arc_radius + 1e-3) &&
              (gear == current_gear);

    if (success) {
      DEBUG_PRINT("one arc plan success");

      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }

  if (!success) {
    // DEBUG_PRINT("one arc plan fail");
  }

  return success;
}

const bool ParallelPathPlanner::TwoArcPlan(
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

  next_arc_steer = pnc::geometry_lib::ReverseSteer(current_arc_steer);
  next_gear = pnc::geometry_lib::ReverseGear(current_gear);

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

const bool ParallelPathPlanner::LineArcPlan(
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

const bool ParallelPathPlanner::AlignBodyPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  return AlignBodyPlan(path_seg_vec, current_pose,
                       calc_params_.target_line.heading, current_gear);
}

const bool ParallelPathPlanner::AlignBodyPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const double target_heading, const uint8_t current_gear) {
  // DEBUG_PRINT("try align body plan");

  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;
  arc.circle_info.radius = apa_param.GetParam().min_turn_radius;

  // check if it is necessary to align body
  if (pnc::mathlib::IsDoubleEqual(arc.headingA, target_heading)) {
    // DEBUG_PRINT("body already align");
    return false;
  }

  bool success = pnc::geometry_lib::CalOneArcWithTargetHeading(
      arc, current_gear, target_heading);

  if (success) {
    // check if gear satisfies needs
    const auto steer = pnc::geometry_lib::CalArcSteer(arc);
    const auto gear = pnc::geometry_lib::CalArcGear(arc);
    success = (gear == current_gear);

    if (success) {
      // DEBUG_PRINT("align body plan success");

      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }

  if (!success) {
    // DEBUG_PRINT("align body plan fail");
  }
  return success;
}

const bool ParallelPathPlanner::STurnParallelPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::LineSegment& target_line,
    const uint8_t current_gear, const double steer_change_ratio1,
    const double radius) {
  // steer_change_ratio = 0.0 -> target_line == current_line
  // steer_change_ratio = 1.0 -> target_line == calc_params_.target_line
  // DEBUG_PRINT("try s turn parallel plan");

  pnc::geometry_lib::Arc arc_s_1;
  arc_s_1.pA = current_pose.pos;
  arc_s_1.headingA = current_pose.heading;

  // check if it is possible to take S turn to target line
  if (!pnc::mathlib::IsDoubleEqual(arc_s_1.headingA, target_line.heading)) {
    // DEBUG_PRINT("body no align");
    return false;
  }

  arc_s_1.circle_info.radius = radius;

  const auto steer_change_ratio =
      pnc::mathlib::Clamp(steer_change_ratio1, 0.1, 1.0);

  // cal current line
  const auto current_line =
      pnc::geometry_lib::BuildLineSegByPose(arc_s_1.pA, arc_s_1.headingA);

  // cal target line according to steer_change_ratio
  // steer_change_ratio = 0.0 -> target_line = current_line
  // steer_change_ratio = 1.0 -> target_line = calc_params_.target_line
  pnc::geometry_lib::LineSegment tmp_target_line;
  tmp_target_line.heading = target_line.heading;

  tmp_target_line.SetPoints((1.0 - steer_change_ratio) * current_line.pA +
                                steer_change_ratio * target_line.pA,
                            (1.0 - steer_change_ratio) * current_line.pB +
                                steer_change_ratio * target_line.pB);

  pnc::geometry_lib::Arc arc_s_2;
  arc_s_2.circle_info.radius = radius;
  arc_s_2.pB = tmp_target_line.pA;
  arc_s_2.headingB = tmp_target_line.heading;

  bool success = pnc::geometry_lib::CalTwoArcWithSameHeading(arc_s_1, arc_s_2,
                                                             current_gear);
  if (success) {
    // check gear and steer can or not meet needs
    const auto steer_1 = pnc::geometry_lib::CalArcSteer(arc_s_1);
    const auto gear_1 = pnc::geometry_lib::CalArcGear(arc_s_1);
    const auto steer_2 = pnc::geometry_lib::CalArcSteer(arc_s_2);
    const auto gear_2 = pnc::geometry_lib::CalArcGear(arc_s_2);

    success = (gear_1 == current_gear && gear_2 == current_gear &&
               steer_1 != steer_2);

    if (success) {
      DEBUG_PRINT("s turn parallel plan success!");

      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer_1, gear_1, arc_s_1));

      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer_2, gear_2, arc_s_2));
    }
  }

  if (!success) {
    DEBUG_PRINT("s turn parallel plan fail!");
  }
  return success;
}

const bool ParallelPathPlanner::CalSinglePathInAdjust(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const double steer_change_ratio,
    const double radius) {
  DEBUG_PRINT("-----CalSinglePathInAdjust-----");
  // DEBUG_PRINT("current_gear = "
  //             << static_cast<int>(current_gear)
  //             << ",  current_pos = " << current_pose.pos.transpose()
  //             << ",  current_heading = " << current_pose.heading * 57.3);

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(5);
  // first try one arc to target line
  bool success = OneArcPlan(tmp_path_seg_vec, current_pose, current_gear);
  if (success) {
    std::cout << "one arc plan success!" << std::endl;
  }

  // if try one arc fail, second try align the ego body and then try go take
  // S-turn to target line
  if (!success) {
    success = AlignBodyPlan(tmp_path_seg_vec, current_pose, current_gear);

    pnc::geometry_lib::PathPoint tmp_current_pose = current_pose;
    if (success) {
      tmp_current_pose.Set(tmp_path_seg_vec.back().GetArcSeg().pB,
                           tmp_path_seg_vec.back().GetArcSeg().headingB);
    }

    success = STurnParallelPlan(tmp_path_seg_vec, tmp_current_pose,
                                calc_params_.target_line, current_gear,
                                steer_change_ratio, radius);
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
    // DEBUG_PRINT("last path pose to one plan");
  } else {
    last_pose = current_pose;
    // DEBUG_PRINT("current pose to one plan");
  }
  if ((last_pose.pos - input_.tlane.pt_terminal_pos).norm() <=
          apa_param.GetParam().static_pos_eps &&
      std::fabs(last_pose.heading - calc_params_.target_line.heading) <=
          apa_param.GetParam().static_heading_eps / 57.3) {
    DEBUG_PRINT("already plan to target pos, no need to one line plan!");
  } else {
    // try line
    pnc::geometry_lib::LineSegment last_line;
    last_line.pA = last_pose.pos;
    last_line.heading = last_pose.heading;
    if (OneLinePlan(last_line, tmp_path_seg_vec, current_gear)) {
      DEBUG_PRINT("OneLinePlan success!");
    } else {
      DEBUG_PRINT("OneLinePlan fail!");
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
    DEBUG_PRINT("CalSinglePathInAdjust success!");
    DEBUG_PRINT("cur_path_seg_vec:");
    double length = 0.0;
    for (const auto& path_seg : path_seg_vec) {
      PrintSegmentInfo(path_seg);
      length += path_seg.Getlength();
    }
    if (length < 0.25) {
      DEBUG_PRINT("this gear path is too small, lose it");
      path_seg_vec.clear();
    }
    return true;
  } else {
    DEBUG_PRINT("CalSinglePathInAdjust fail");
    return false;
  }

  // path_seg_vec = tmp_path_seg_vec;

  // if (path_seg_vec.size() > 0) {
  //   DEBUG_PRINT("CalSinglePathInAdjust success");
  //   return true;
  // } else {
  //   DEBUG_PRINT("CalSinglePathInAdjust fail");
  //   return false;
  // }
}
// adjust plan end

const uint8_t ParallelPathPlanner::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg) {
  // std::cout << "--- collision detection ---" << std::endl;
  CollisionDetector::CollisionResult col_res;
  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    auto& line = path_seg.line_seg;
    col_res = collision_detector_ptr_->Update(line, line.heading);
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    auto& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->Update(arc, arc.headingA);
  } else {
    // std::cout << "no support the seg type\n";
    return PATH_COL_INVALID;
  }

  const double remain_car_dist = col_res.remain_car_dist;
  const double remain_obs_dist = col_res.remain_obstacle_dist;
  const double safe_remain_dist =
      std::min(remain_car_dist, remain_obs_dist - 0.1);

  // std::cout << "  remain_car_dist = " << remain_car_dist
  //           << "  remain_obs_dist = " << remain_obs_dist
  //           << "  safe_remain_dist = " << safe_remain_dist << std::endl;

  if (safe_remain_dist < 0.0) {
    // std::cout << "the distance between obstacle and ego is smaller than "
    //              "min_safe_distance, collided! "
    //           << std::endl;
    return PATH_COL_INVALID;
  }

  if (remain_car_dist <= safe_remain_dist) {
    // std::cout << "the path will not collide\n";
    return PATH_COL_NORMAL;
  }

  // std::cout << "the path will collide, need to be shorten to
  // safe_remain_dist"
  //           << std::endl;
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
}
// collision detect end

const uint8_t ParallelPathPlanner::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg, Eigen::Vector2d& collision_pt) {
  // std::cout << "--- collision detection ---" << std::endl;
  collision_pt.setZero();
  CollisionDetector::CollisionResult col_res;

  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    auto& line = path_seg.line_seg;
    col_res = collision_detector_ptr_->Update(line, line.heading);
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    auto& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->Update(arc, arc.headingA);
  } else {
    // std::cout << "no support the seg type\n";
    return PATH_COL_INVALID;
  }

  const double remain_car_dist = col_res.remain_car_dist;
  const double remain_obs_dist = col_res.remain_obstacle_dist;
  const double safe_remain_dist =
      std::min(remain_car_dist, remain_obs_dist - 0.3);

  // std::cout << "  remain_car_dist = " << remain_car_dist
  //           << "  remain_obs_dist = " << remain_obs_dist
  //           << "  safe_remain_dist = " << safe_remain_dist << std::endl;

  if (safe_remain_dist < 0.0) {
    // std::cout << "the distance between obstacle and ego is smaller than "
    //              "min_safe_distance, collided! "
    //           << std::endl;
    return PATH_COL_INVALID;
  }

  if (remain_car_dist <= safe_remain_dist) {
    // std::cout << "the path will not collide\n";
    return PATH_COL_NORMAL;
  }

  // std::cout << "the path will collide, need to be shorten to
  // safe_remain_dist"
  //           << std::endl;
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
  collision_pt = col_res.collision_point_global;
  return PATH_COL_SHORTEN;
}
// collision detect end

const bool ParallelPathPlanner::CheckPathSegCollided(
    const pnc::geometry_lib::PathSegment& path_seg, const double buffer) const {
  CollisionDetector::CollisionResult col_res;
  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    auto& line = path_seg.line_seg;
    col_res = collision_detector_ptr_->Update(line, line.heading);
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    auto& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->Update(arc, arc.headingA);
  } else {
    // std::cout << "no support the seg type\n";
    return true;
  }

  const double remain_car_dist = col_res.remain_car_dist;
  const double remain_obs_dist = col_res.remain_obstacle_dist;
  const double safe_remain_dist =
      std::min(remain_car_dist, remain_obs_dist - buffer);

  if (safe_remain_dist <= 0.0) {
    return true;
  }

  if (remain_car_dist > safe_remain_dist) {
    return true;
  }

  return false;
}

const bool ParallelPathPlanner::CheckPathSegVecCollided(
    const std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const double buffer) const {
  for (const auto& path_seg : path_seg_vec) {
    if (CheckPathSegCollided(path_seg, buffer)) {
      return true;
    }
  }
  return false;
}

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

const bool ParallelPathPlanner::CheckSamePose(
    const pnc::geometry_lib::PathPoint& pose1,
    const pnc::geometry_lib::PathPoint& pose2) const {
  return (
      (pose1.pos - pose2.pos).norm() <= apa_param.GetParam().static_pos_eps &&
      std::fabs(
          pnc::geometry_lib::NormalizeAngle(pose1.heading - pose2.heading)) <=
          apa_param.GetParam().static_heading_eps / 57.3);
}

const bool ParallelPathPlanner::CheckEgoPoseCollided(
    pnc::geometry_lib::PathPoint ego_pose) const {
  const Eigen::Vector2d end_pos =
      ego_pose.pos + 0.01 * Eigen::Vector2d(std::cos(ego_pose.heading),
                                            std::sin(ego_pose.heading));

  pnc::geometry_lib::LineSegment line(ego_pose.pos, end_pos, ego_pose.heading);

  const auto& col_res = collision_detector_ptr_->Update(line, line.heading);

  return (col_res.collision_flag ||
          col_res.remain_car_dist > col_res.remain_obstacle_dist);
}

void ParallelPathPlanner::AddPathSegToOutPut(
    const pnc::geometry_lib::PathSegment& path_seg) {
  output_.path_available = true;
  output_.path_segment_vec.emplace_back(path_seg);
  output_.length += path_seg.Getlength();
  output_.gear_cmd_vec.emplace_back(path_seg.seg_gear);
  output_.steer_vec.emplace_back(path_seg.seg_steer);
}

void ParallelPathPlanner::AddPathSegVecToOutput(
    const std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  for (const auto& path_seg : path_seg_vec) {
    AddPathSegToOutPut(path_seg);
  }
}

}  // namespace apa_planner
}  // namespace planning