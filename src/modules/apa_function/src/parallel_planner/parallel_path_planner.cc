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
static const double kMaxParkOutRootHeading = 25.0;

static const double kColBufferTrippleStep = 0.2;
static const double kColBufferInSlot = 0.3;
static const double kSmallColBufferInSlot = 0.1;
static const double kColBufferOutSlot = 0.5;

static const double kLatColBufferOutSlot = 0.25;
static const double kLatColBufferInSlot = 0.0;

static const size_t kMaxParallelParkInSegmentNums = 15;
static const size_t kMaxPathNumsInSlot = 5;
static const size_t kMaxMultiStepNums = 5;
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

  input_.ego_pose.heading =
      pnc::geometry_lib::NormalizeAngle(input_.ego_pose.heading);

  calc_params_.is_left_side =
      (input_.tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT);

  calc_params_.slot_side_sgn = input_.tlane.slot_side_sgn;

  const double target_heading = 0.0;
  calc_params_.target_pose.Set(input_.tlane.pt_terminal_pos, target_heading);
  // target line
  calc_params_.target_line = pnc::geometry_lib::BuildLineSegByPose(
      input_.tlane.pt_terminal_pos, target_heading);

  CollisionDetector::Paramters params;
  params.lat_inflation = input_.slot_occupied_ratio > 0.3 ? kLatColBufferOutSlot
                                                          : kLatColBufferInSlot;
  collision_detector_ptr_->SetParam(params);
}

const bool ParallelPathPlanner::Update() {
  std::cout << "-----------------------------------------parallel path "
               "planner:---------------------------------------"
            << std::endl;
  // preprocess
  Preprocess();

  // reset output
  output_.Reset();

  // judge if ego is out of slot
  if (!CheckEgoInSlot()) {
    DEBUG_PRINT("ego is out of slot");
    if (MonoStepPlanWithShift()) {
      std::cout << "MonoStepPlanWithShift success" << std::endl;
    } else {
      if (!CalMinSafeCircle()) {
        std::cout << "inverse plan in slot failed!" << std::endl;
        return false;
      }
      std::cout << "CalMinSafeCircle success!" << std::endl;
    }

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
    DEBUG_PRINT("ego is in slot");
    CollisionDetector::Paramters param;
    param.lat_inflation = 0.0;
    collision_detector_ptr_->SetParam(param);
    // ego is in slot, search from ego pose to target pose, or just
    // correct heading
    if (MultiPlan()) {
      // pnc::geometry_lib::PrintSegmentsVecInfo(output_.path_segment_vec);
      std::cout << "MultiPlan success!" << std::endl;
      return true;
    } else {
      std::cout << "MultiPlan  failed!" << std::endl;
      if (MultiAlignBody()) {
        std::cout << "Multi align body success!" << std::endl;
        return true;
      } else {
        std::cout << "Multi align body failed!" << std::endl;
      };
    }

    // parallel adjust step
    if (ParallelAdjustPlan()) {
      // pnc::geometry_lib::PrintSegmentsVecInfo(output_.path_segment_vec);
      std::cout << "parallel adjust step plan success!" << std::endl;
      // PrintOutputSegmentsInfo();
      return true;
    } else {
      std::cout << "parallel adjust step plan failed!" << std::endl;
    }

    // // if heading diff is small use CSCS curve, or use multi
    // if (CSCSAdjustPlan()) {
    //   // pnc::geometry_lib::PrintSegmentsVecInfo(output_.path_segment_vec);
    //   DEBUG_PRINT("CSCSAdjustPlan success!");
    //   return true;
    // } else {
    //   DEBUG_PRINT("CSCSAdjustPlan failed!");
    // }
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
  JSON_DEBUG_VALUE("path_plan_time_ms", duration);
  return success;
}

const bool ParallelPathPlanner::PreparePlan() {
  // if ego is front enough, first try backward plan.
  if (input_.ego_pose.pos.x() >= 5.0) {
    if (BackwardNormalPlan()) {
      std::cout << "backward plan success in preparing step!" << std::endl;
      return true;
    }
  }
  std::cout << "backward failed in preparing step! start park out foreach"
            << std::endl;
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
  auto first_arc = calc_params_.park_out_path_in_slot.back().GetArcSeg();

  if (!ConstructParkOutArcLengthVec(first_arc_length_vec, first_arc)) {
    DEBUG_PRINT("construct arc length vec failed!");
    return false;
  }
  std::cout << "first arc range = " << first_arc_length_vec.front() << ", "
            << first_arc_length_vec.back() << std::endl;

  const double park_out_target_heading =
      apa_param.GetParam().parallel_search_out_heading / 57.3 *
      calc_params_.slot_side_sgn;

  for (const auto& arc_length : first_arc_length_vec) {
    if (!pnc::geometry_lib::CompleteArcInfo(first_arc, arc_length,
                                            first_arc.is_anti_clockwise)) {
      std::cout << "CompleteArcInfo error" << std::endl;
      break;
    }
    // std::cout << "first_arc heading =" << first_arc.headingB * 57.3
    //           << std::endl;

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

    const double lat_dis =
        pnc::geometry_lib::CalPoint2LineDist(input_.tlane.obs_pt_inside, line);
    if (lat_dis - 0.5 * apa_param.GetParam().car_width <
        apa_param.GetParam().parallel_ego_side_to_obs_in_buffer) {
      continue;
    }

    if (!ConstructParkOutLineLengthVec(line_length_vec, line)) {
      DEBUG_PRINT("construct line length vec failed!");
      continue;
    }

    // std::cout << "line range = " << line_length_vec.front() << ", "
    //           << line_length_vec.back() << std::endl;

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

// park out from target pose with two arc to ego line.
const bool ParallelPathPlanner::MonoStepPlan() {
  if (calc_params_.park_out_path_in_slot.size() != 1) {
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

    if (!CheckParkOutCornerSafeWithObsPin(arc_1)) {
      continue;
    }

    // make sure first line step length are more than min_leng during dirve gear
    // but not too long
    if (input_.is_replan_first) {
      if (ego_line_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
        // avoid driving along the line for long distance when ego heading is
        // large
        if (std::fabs(input_.ego_pose.heading * 57.3) > 10.0 &&
            ego_line.length > 0.66) {
          continue;
        }
        // prolong the path during preparation, or will not calc success
        // during normal backward step if ego stops early
        first_line_prolong = true;
        const Eigen::Vector2d v_ego =
            pnc::geometry_lib::GenHeadingVec(ego_line.heading);

        ego_line.SetPoints(
            ego_line.pA,
            ego_line.pB + apa_param.GetParam().min_line_length * v_ego);
        // if ((ego_line.pB.x() > 8.0)) {
        //   first_line_prolong = false;
        //   continue;
        // }
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
          last_line_seg.length <= apa_param.GetParam().min_line_length) {
        continue;
      }
    }

    // first check if front corner collided with obs_pin
    // if (!CheckParkOutCornerSafeWithObsPin(arc_1)) {
    //   continue;
    // }

    // check collision of first line and two arc,(last line is calculated in
    // inverse loop)
    auto col_res = collision_detector_ptr_->Update(arc_1, arc_1.headingA);
    if (col_res.collision_flag ||
        col_res.remain_car_dist >
            col_res.remain_obstacle_dist - kColBufferTrippleStep) {
      continue;
    }

    col_res = collision_detector_ptr_->Update(arc_2, arc_2.headingA);
    if (col_res.collision_flag ||
        col_res.remain_car_dist >
            col_res.remain_obstacle_dist - kColBufferTrippleStep) {
      continue;
    }

    col_res = collision_detector_ptr_->Update(ego_line, ego_line.heading);
    if (col_res.collision_flag ||
        col_res.remain_car_dist >
            col_res.remain_obstacle_dist - kColBufferTrippleStep) {
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
    pnc::geometry_lib::PrintSegmentInfo(arc_seg_1);
    pnc::geometry_lib::PrintSegmentInfo(arc_seg_2);

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

      if (!CheckSamePose(last_line_poseA, last_line_poseB) &&
          pnc::mathlib::IsDoubleEqual(last_line_poseA.pos.y(),
                                      last_line_poseB.pos.y())) {
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

const bool ParallelPathPlanner::MonoStepPlanWithShift() {
  const double deta_y = 0.05 * calc_params_.slot_side_sgn;
  const double terminal_y = input_.tlane.pt_terminal_pos.y();

  std::vector<double> target_y_vec;
  target_y_vec.clear();
  target_y_vec.reserve(10);

  for (double y_offset = 0.0; std::fabs(y_offset) < 0.1; y_offset += deta_y) {
    target_y_vec.emplace_back(y_offset + terminal_y);
  }

  bool success = false;
  bool is_dirve_out_safe = false;
  pnc::geometry_lib::PathPoint tmp_pose(input_.tlane.pt_terminal_pos,
                                        calc_params_.target_line.heading);

  size_t safe_y_cnt = 0;
  for (const auto& target_y : target_y_vec) {
    tmp_pose.pos.y() = target_y;
    if (MonoStepPlanOnceWithShift(is_dirve_out_safe, tmp_pose)) {
      if (is_dirve_out_safe) {
        success = true;
        safe_y_cnt++;
        DEBUG_PRINT("successful target_y =" << target_y);
        if (safe_y_cnt == 1) {
          calc_params_.safe_circle_root_pose = tmp_pose;
          // calc_params_.park_out_pose = tmp_pose;
          break;
        }
      }
    }
  }

  return success;
}

const bool ParallelPathPlanner::MonoStepPlanOnceWithShift(
    bool& is_drive_out_safe, pnc::geometry_lib::PathPoint& target_pose) {
  // calc backward limit
  pnc::geometry_lib::LineSegment backward_line;
  backward_line.pA = target_pose.pos;
  backward_line.heading = target_pose.heading;
  std::cout << "MonoStepPlanOnceWithShift target pose"
            << target_pose.pos.transpose() << std::endl;

  if (!CalcLineStepLimitPose(backward_line,
                             pnc::geometry_lib::SEG_GEAR_REVERSE)) {
    std::cout << "CalcLineStepLimitPose error!" << std::endl;
    return false;
  }

  // check if ego is able to park out at backward limit pose
  pnc::geometry_lib::Arc forward_arc;
  forward_arc.pA = backward_line.pB;
  forward_arc.headingA = backward_line.heading;
  forward_arc.circle_info.radius = apa_param.GetParam().min_turn_radius;

  const auto forward_steer = calc_params_.slot_side_sgn
                                 ? pnc::geometry_lib::SEG_STEER_LEFT
                                 : pnc::geometry_lib::SEG_STEER_RIGHT;

  if (!CalcArcStepLimitPose(forward_arc, is_drive_out_safe,
                            pnc::geometry_lib::SEG_GEAR_DRIVE, forward_steer,
                            kColBufferInSlot)) {
    std::cout << "calc forward arc limit error!" << std::endl;
    return false;
  }

  if (!CheckParkOutCornerSafeWithObsPin(forward_arc)) {
    is_drive_out_safe = false;
    DEBUG_PRINT("first arc collided with obs Pin!");
    return true;
  }

  if (is_drive_out_safe) {
    pnc::geometry_lib::PathSegment forward_seg(
        forward_steer, pnc::geometry_lib::SEG_GEAR_DRIVE, forward_arc);
    calc_params_.park_out_path_in_slot.clear();
    calc_params_.park_out_path_in_slot.emplace_back(std::move(forward_seg));

    target_pose.pos = forward_arc.pA;

    calc_params_.valid_target_pt_vec.clear();
    calc_params_.valid_target_pt_vec.emplace_back(target_pose);

    // find all pt which can park out along the y_offset line
    bool is_safe = false;
    for (double x_offset = 0.05; x_offset < 0.5; x_offset += 0.05) {
      forward_arc.pA.x() = target_pose.pos.x() + x_offset;
      if (!CalcArcStepLimitPose(forward_arc, is_safe,
                                pnc::geometry_lib::SEG_GEAR_DRIVE,
                                forward_steer, kColBufferInSlot)) {
        std::cout << "calc forward arc limit error!" << std::endl;
        break;
      }
      if (!is_safe) {
        break;
      }
      calc_params_.valid_target_pt_vec.emplace_back(
          pnc::geometry_lib::PathPoint(forward_arc.pA, forward_arc.headingA));
    }

    auto compare = [](const pnc::geometry_lib::PathPoint& pose1,
                      const pnc::geometry_lib::PathPoint& pose2) {
      return pose1.pos.x() > pose2.pos.x();
    };

    if (calc_params_.valid_target_pt_vec.size() > 1) {
      std::sort(calc_params_.valid_target_pt_vec.begin(),
                calc_params_.valid_target_pt_vec.end(), compare);
      std::cout << "target pt vec: " << std::endl;
      for (const auto& pt : calc_params_.valid_target_pt_vec) {
        std::cout << pt.pos.x() << ", ";
      }
      std::cout << std::endl;
    }
  }
  return true;
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

  const auto col_res =
      TrimPathByCollisionDetection(arc_seg, kColBufferTrippleStep);
  DEBUG_PRINT("col_res =" << static_cast<int>(col_res));

  if (col_res == PATH_COL_INVALID) {
    DEBUG_PRINT("calc park out first arc col failed");
    return false;
  }

  DEBUG_PRINT("max practical arc length =" << arc_seg.Getlength());

  double d_heading = kSearchOutArcSampleDistance / arc.circle_info.radius;
  d_heading *= (arc.is_anti_clockwise ? 1.0 : -1.0);

  double arc_length = 0.2;
  double heading = arc.headingA;
  while (std::fabs(heading) < kMaxParkOutFirstArcHeading / 57.3) {
    if (arc_length > 7.0 || arc_length > arc_seg.Getlength()) {
      // DEBUG_PRINT(
      //     "length over given value or max safe length!  current heading ="
      //     << heading * 57.3);
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

  const auto col_res =
      TrimPathByCollisionDetection(line_seg, kColBufferTrippleStep);

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
                        apa_param.GetParam().min_radius_out_slot,
                        kColBufferOutSlot)) {
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
    if (TwoSameGearArcPlanToLine(path_seg_vec, input_.ego_pose, prepare_line,
                                 pnc::geometry_lib::SEG_GEAR_DRIVE,
                                 kColBufferOutSlot)) {
      AddPathSegToOutPut(path_seg_vec);
      DEBUG_PRINT("prepare plan: ego to prepare pose success with dirve gear!");
      return true;
    }
    if (TwoSameGearArcPlanToLine(path_seg_vec, input_.ego_pose, prepare_line,
                                 pnc::geometry_lib::SEG_GEAR_REVERSE,
                                 kColBufferTrippleStep) &&
        path_seg_vec.back().seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
      AddPathSegToOutPut(path_seg_vec);
      DEBUG_PRINT(
          "prepare plan: ego to prepare pose success with reverse gear!");
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

  // success = RSCurvePlan(input_.ego_pose, prepare_pose,
  //                       apa_param.GetParam().min_turn_radius);

  return false;
}

const bool ParallelPathPlanner::BackwardNormalPlan() {
  if (MonoStepPlan()) {
    // PrintOutputSegmentsInfo();
    return true;
  }

  const std::vector<double> radius_vec = {
      11.0, 10.0, 9.0, 7.0, 6.0, apa_param.GetParam().min_turn_radius};

  for (const auto& radius : radius_vec) {
    if (OneStepDubinsPlan(input_.ego_pose, calc_params_.safe_circle_root_pose,
                          radius, kColBufferTrippleStep)) {
      DEBUG_PRINT("triple step from ego to circle root pose success! radius = "
                  << radius);
      GenPathOutputByDubins();
      return true;
    }
    if (OneStepDubinsPlan(input_.ego_pose, calc_params_.park_out_pose, radius,
                          kColBufferTrippleStep)) {
      DEBUG_PRINT(
          "triple step from ego to park out pose success! radius = " << radius);
      GenPathOutputByDubins();
      AddLastArc();
      return true;
    }

    // start backward three step plan
    for (const auto& target_pose : calc_params_.valid_target_pt_vec) {
      if (OneStepDubinsPlan(input_.ego_pose, target_pose, radius,
                            kColBufferTrippleStep)) {
        DEBUG_PRINT("plan from ego pose to valid target pose success!"
                    << " valid pos= " << target_pose.pos.transpose()
                    << ", radius =" << radius);
        GenPathOutputByDubins();
        // PrintOutputSegmentsInfo();
        return true;
      }
    }
  }

  return false;
}

const bool ParallelPathPlanner::OneStepDubinsPlan(
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const double radius,
    const double buffer) {
  pnc::dubins_lib::DubinsLibrary::Input dubins_input;

  dubins_input.radius = radius;
  dubins_input.Set(start_pose.pos, target_pose.pos, start_pose.heading,
                   target_pose.heading);

  dubins_planner_.SetInput(dubins_input);

  bool success = dubins_planner_.OneStepDubinsUpdate();

  if (success) {
    success = !IsDubinsCollided(buffer);
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
      input_.tlane.obs_pt_inside.y() * calc_params_.slot_side_sgn +
          0.4 * apa_param.GetParam().car_width) {
    return false;
  }

  const auto last_line =
      pnc::geometry_lib::BuildLineSegByPose(last_arc.pB, last_arc.headingB);

  const double lat_dist = pnc::geometry_lib::CalPoint2LineDist(
                              input_.tlane.obs_pt_inside, last_line) -
                          0.5 * apa_param.GetParam().car_width;
  if (lat_dist < apa_param.GetParam().parallel_ego_side_to_obs_in_buffer) {
    return false;
  }

  auto col_res = collision_detector_ptr_->Update(last_arc, last_arc.headingA);
  if (col_res.collision_flag ||
      col_res.remain_car_dist >
          col_res.remain_obstacle_dist - kColBufferTrippleStep) {
    // std::cout << "line collided!" << std::endl;
    return false;
  }

  std::vector<pnc::geometry_lib::PathSegment> tmp_park_out_path;

  pnc::geometry_lib::PathSegment first_arc_seg(
      pnc::geometry_lib::CalArcSteer(first_arc),
      pnc::geometry_lib::CalArcGear(first_arc), first_arc);
  calc_params_.park_out_path_in_slot.back() = first_arc_seg;

  tmp_park_out_path.emplace_back(std::move(first_arc_seg));

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
  pnc::geometry_lib::PathSegment last_path_seg =
      calc_params_.park_out_path_in_slot.back();

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

const bool ParallelPathPlanner::IsDubinsCollided(const double buffer) {
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

    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
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
          (input_.ego_pose.pos.x() > input_.tlane.obs_pt_outside.x() - 2.0) &&
          (input_.ego_pose.pos.x() < input_.tlane.obs_pt_inside.x() + 2.0));
}

// search from the inside parking space to outside
const bool ParallelPathPlanner::CalMinSafeCircle() {
  std::vector<pnc::geometry_lib::PathSegment> search_out_res;
  bool success =
      InverseSearchLoopInSlot(search_out_res, calc_params_.target_pose);

  if (success) {
    ReduceRootPoseHeadingInSlot(search_out_res);

    // calc pose with ego had just cross pt_inside'y, which is used to connect
    // the prepare point
    CalcParkOutPose(search_out_res.back());

    calc_params_.park_out_path_in_slot = search_out_res;
    calc_params_.safe_circle_root_pose = search_out_res.back().GetStartPose();
    calc_params_.park_out_pose = search_out_res.back().GetEndPose();
    pnc::geometry_lib::PrintPose("corrected multi root pose",
                                 calc_params_.safe_circle_root_pose);
    pnc::geometry_lib::PrintPose("corrected park out pose",
                                 calc_params_.park_out_pose);
  }
  return success;
}

const bool ParallelPathPlanner::ReduceRootPoseHeadingInSlot(
    std::vector<pnc::geometry_lib::PathSegment>& search_out_res) {
  const size_t step_length = search_out_res.size();
  if (step_length == 0) {
    DEBUG_PRINT("found no parkout path!");
    return false;
  }

  DEBUG_PRINT("root pose heading =" << search_out_res.back().GetStartHeading() *
                                           57.3);

  if (std::fabs(search_out_res.back().GetStartHeading()) <
      kMaxParkOutRootHeading / 57.3) {
    DEBUG_PRINT("park out heading is small enough!");
    return true;
  }

  const auto& last_ori_backward_path = search_out_res[step_length - 2];

  if (std::fabs(last_ori_backward_path.GetStartHeading()) <=
      kMaxParkOutRootHeading) {
    // use line step or curves with large radius, instead of mininimal radius,
    // to reduce parking out heading

    // firstly, contrust backward line which is long enougn.
    pnc::geometry_lib::LineSegment backward_line;
    backward_line.pA = last_ori_backward_path.GetStartPos();
    backward_line.heading = last_ori_backward_path.GetStartHeading();

    // calc backward limit pose
    if (!CalcLineStepLimitPose(backward_line,
                               pnc::geometry_lib::SEG_GEAR_REVERSE)) {
      return false;
    }

    bool is_dirve_out_safe = false;
    pnc::geometry_lib::Arc forward_arc;
    forward_arc.pA = backward_line.pB;
    forward_arc.headingA = backward_line.heading;
    forward_arc.circle_info.radius = apa_param.GetParam().min_turn_radius;
    const uint8_t forward_steer = search_out_res.back().seg_steer;

    if (!CalcArcStepLimitPose(forward_arc, is_dirve_out_safe,
                              pnc::geometry_lib::SEG_GEAR_DRIVE, forward_steer,
                              kColBufferInSlot)) {
      return false;
    }

    if (!is_dirve_out_safe) {
      DEBUG_PRINT("current step is not able to park out!");
      return false;
    }

    // last two path should be replaced,
    search_out_res[step_length - 2] = pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::SEG_GEAR_REVERSE, backward_line);

    search_out_res[step_length - 1] = pnc::geometry_lib::PathSegment(
        forward_steer, pnc::geometry_lib::SEG_GEAR_DRIVE, forward_arc);
    DEBUG_PRINT("replace last two steps success!");
  }
  pnc::geometry_lib::PrintPose("opt root pose",
                               search_out_res.back().GetStartPose());

  return true;
}

const bool ParallelPathPlanner::CalcParkOutPose(
    pnc::geometry_lib::PathSegment& park_out_seg) {
  // the park out pt is the pose where ego front vertex has just crossed
  // pt_inside y coordination

  auto& front_arc = park_out_seg.arc_seg;

  const pnc::geometry_lib::PathPoint start_pose(front_arc.pA,
                                                front_arc.headingA);

  // make sure which vertex is front outside one of ego car
  const size_t front_vertex_idx = calc_params_.is_left_side ? 0 : 3;

  const Eigen::Vector2d v_vertex(
      apa_param.GetParam().car_vertex_x_vec[front_vertex_idx],
      apa_param.GetParam().car_vertex_y_vec[front_vertex_idx]);

  const double sample_ds = 0.2;
  const double d_theta_sgn = (front_arc.is_anti_clockwise ? 1.0 : -1.0);
  const double d_theta = d_theta_sgn * sample_ds / front_arc.circle_info.radius;
  const Eigen::Matrix2d rot_m = pnc::geometry_lib::GetRotm2dFromTheta(d_theta);

  bool success = false;
  pnc::geometry_lib::LocalToGlobalTf ego2slot;
  auto park_out_pose = start_pose;
  const double max_park_out_heading = kMaxParkOutFirstArcHeading / 57.3;

  while (std::fabs(park_out_pose.heading) <= max_park_out_heading) {
    ego2slot.Init(park_out_pose.pos, park_out_pose.heading);
    const Eigen::Vector2d vertex_slot = ego2slot.GetPos(v_vertex);

    if (vertex_slot.y() * calc_params_.slot_side_sgn >=
        input_.tlane.obs_pt_inside.y() * calc_params_.slot_side_sgn) {
      success = true;
      break;
    }
    const auto v_oa = park_out_pose.pos - front_arc.circle_info.center;
    park_out_pose.pos = rot_m * v_oa + front_arc.circle_info.center;
    park_out_pose.heading += d_theta;
  }

  if (success) {
    front_arc.pB = park_out_pose.pos;
    front_arc.headingB = park_out_pose.heading;
    front_arc.length = std::fabs(pnc::geometry_lib::NormalizeAngle(
                           front_arc.headingB - front_arc.headingA)) *
                       front_arc.circle_info.radius;
  }
  return success;
}

const bool ParallelPathPlanner::InverseSearchLoopInSlot(
    std::vector<pnc::geometry_lib::PathSegment>& search_out_res,
    const pnc::geometry_lib::PathPoint& terminal_pose) {
  DEBUG_PRINT("enter InverseSearchLoopInSlot---------");
  search_out_res.clear();
  search_out_res.reserve(10);
  const double radius = apa_param.GetParam().min_turn_radius;

  // calc backward limit
  pnc::geometry_lib::LineSegment first_line_step;
  first_line_step.pA = terminal_pose.pos;
  first_line_step.heading = terminal_pose.heading;
  if (!CalcLineStepLimitPose(first_line_step,
                             pnc::geometry_lib::SEG_GEAR_REVERSE)) {
    std::cout << "CalcLineStepLimitPose error!" << std::endl;
    return false;
  }

  const pnc::geometry_lib::PathPoint back_line_limit(first_line_step.pB,
                                                     first_line_step.heading);

  if (!CheckSamePose(back_line_limit, terminal_pose)) {
    DEBUG_PRINT("need backward step, pose =" << back_line_limit.pos.x() << ", "
                                             << back_line_limit.pos.y() << ", "
                                             << back_line_limit.heading * 57.3);

    search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::SEG_GEAR_REVERSE, first_line_step));
  } else {
    DEBUG_PRINT("first backward step collided!");
  }

  // get valid target pose
  std::vector<double> valid_target_x_vec = pnc::geometry_lib::Linspace(
      terminal_pose.pos.x(), back_line_limit.pos.x(), 0.2);

  pnc::geometry_lib::PathPoint valid_pose = terminal_pose;
  for (const auto& x : valid_target_x_vec) {
    valid_pose.pos.x() = x;

    // Todo(Gdbai2): change to pose to be suitable for multi search
    calc_params_.valid_target_pt_vec.emplace_back(valid_pose);
  }

  // check if ego is able to park out at start pose
  bool is_dirve_out_safe = false;
  pnc::geometry_lib::Arc forward_arc;
  forward_arc.pA = terminal_pose.pos;
  forward_arc.headingA = terminal_pose.heading;
  forward_arc.circle_info.radius = radius;

  const uint8_t forward_steer =
      (calc_params_.is_left_side ? pnc::geometry_lib::SEG_STEER_RIGHT
                                 : pnc::geometry_lib::SEG_STEER_LEFT);

  if (!CalcArcStepLimitPose(forward_arc, is_dirve_out_safe,
                            pnc::geometry_lib::SEG_GEAR_DRIVE, forward_steer,
                            kColBufferInSlot)) {
    std::cout << "calc forward arc limit error!" << std::endl;
    return false;
  }
  DEBUG_PRINT("radius =" << forward_arc.circle_info.radius);

  if (is_dirve_out_safe) {
    search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
        forward_steer, pnc::geometry_lib::SEG_GEAR_DRIVE, forward_arc));
    std::cout << "ego can park out at first!" << std::endl;
    return true;
  }
  std::cout << "ego can't park out at first!" << std::endl;

  std::cout << "-------------- start loop -----------------------" << std::endl;
  bool loop_success = false;
  auto start_pose_in_loop = back_line_limit;
  const auto backward_steer = pnc::geometry_lib::ReverseSteer(forward_steer);
  for (size_t i = 0; i < kMaxPathNumsInSlot; i += 2) {
    // start calc forward limit pose
    DEBUG_PRINT("---------- No. " << i << "loop search ---------------");
    DEBUG_PRINT("------ forward step --------");
    PrintPose("start pose in loop", start_pose_in_loop);

    pnc::geometry_lib::Arc forward_arc;
    forward_arc.pA = start_pose_in_loop.pos;
    forward_arc.headingA = start_pose_in_loop.heading;
    forward_arc.circle_info.radius = radius;
    if (!CalcArcStepLimitPose(forward_arc, is_dirve_out_safe,
                              pnc::geometry_lib::SEG_GEAR_DRIVE, forward_steer,
                              kColBufferInSlot)) {
      std::cout << "calc forward arc limit error!" << std::endl;
      break;
    }
    pnc::geometry_lib::PrintPose("forward limit pose", forward_arc.pB,
                                 forward_arc.headingB);
    DEBUG_PRINT("arc length = " << forward_arc.length);

    pnc::geometry_lib::PathPoint forward_limit_pose;
    forward_limit_pose.Set(forward_arc.pB, forward_arc.headingB);
    if (CheckSamePose(start_pose_in_loop, forward_limit_pose)) {
      DEBUG_PRINT("can't dirve in forward loop!");
      break;
    }

    search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
        forward_steer, pnc::geometry_lib::SEG_GEAR_DRIVE, forward_arc));

    // ego can park out in forward step, return true
    if (is_dirve_out_safe) {
      loop_success = true;
      DEBUG_PRINT("find park out pose!");
      break;
    }

    DEBUG_PRINT("forward limit pose is valid\n");
    DEBUG_PRINT("------ backward step --------");

    pnc::geometry_lib::Arc backward_arc;
    backward_arc.pA = forward_arc.pB;
    backward_arc.headingA = forward_arc.headingB;
    backward_arc.circle_info.radius = radius;

    if (!CalcArcStepLimitPose(backward_arc, is_dirve_out_safe,
                              pnc::geometry_lib::SEG_GEAR_REVERSE,
                              backward_steer)) {
      // std::cout << "calc backward arc limit error!" << std::endl;
      break;
    }

    // start calc backward limit pose
    pnc::geometry_lib::PathPoint backward_limit_pose(backward_arc.pB,
                                                     backward_arc.headingB);

    if (CheckSamePose(backward_limit_pose, forward_limit_pose)) {
      DEBUG_PRINT("can't reverse in backward loop!");
      break;
    }
    std::cout << "backward limit pose = ";
    pnc::geometry_lib::PrintPose(backward_arc.pB, backward_arc.headingB);
    DEBUG_PRINT("arc length = " << backward_arc.length);

    search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
        backward_steer, pnc::geometry_lib::SEG_GEAR_REVERSE, backward_arc));

    start_pose_in_loop = backward_limit_pose;
  }

  if (!loop_success) {
    search_out_res.clear();
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

  const double line_dist_limit = 5.5;

  const Eigen::Vector2d rough_limit_pt =
      start_pose.pos + line_dist_limit * dirve_sgn *
                           Eigen::Vector2d(std::cos(start_pose.heading),
                                           std::sin(start_pose.heading));

  pnc::geometry_lib::PathSegment line_path(
      gear, pnc::geometry_lib::LineSegment(start_pose.pos, rough_limit_pt,
                                           start_pose.heading));

  const auto& col_res =
      TrimPathByCollisionDetection(line_path, kColBufferInSlot);

  line.is_ignored = true;
  // check if ego can park out at limit pose
  if (col_res == PATH_COL_NORMAL || col_res == PATH_COL_SHORTEN) {
    if (line_path.GetLineSeg().length >= 0.1) {
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
    const uint8_t steer, const double buffer) {
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
  if (!pnc::geometry_lib::CompleteArcInfo(arc, arc_length_limit,
                                          arc.is_anti_clockwise)) {
    return false;
  }

  Eigen::Vector2d forward_col_pt;
  pnc::geometry_lib::PathSegment arc_path(steer, gear, arc);
  const auto& col_res =
      TrimPathByCollisionDetection(arc_path, forward_col_pt, buffer);
  // std::cout << "forward_col_pt =" << forward_col_pt.transpose() << std::endl;

  arc.is_ignored = true;

  // get updated arc info of collision free
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

  // check if ego can park out at limit pose
  if (gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    if (col_res == PATH_COL_NORMAL) {
      // path is long enough, can park out
      is_drive_out_safe = true;
    } else if (col_res == PATH_COL_SHORTEN) {
      const bool tlane_col_x_condition = pnc::mathlib::IsInBound(
          forward_col_pt.x(),
          input_.tlane.obs_pt_inside.x() + apa_param.GetParam().static_pos_eps,
          input_.tlane.obs_pt_outside.x() -
              apa_param.GetParam().static_pos_eps);

      // std::cout << "calc_params_.slot_side_sgn =" <<
      // calc_params_.slot_side_sgn
      //           << std::endl;

      const bool tlane_col_y_condition =
          forward_col_pt.y() * calc_params_.slot_side_sgn <=
          input_.tlane.obs_pt_inside.y() * calc_params_.slot_side_sgn +
              apa_param.GetParam().static_pos_eps;

      if (tlane_col_x_condition && tlane_col_y_condition) {
        is_drive_out_safe = false;
      } else {
        is_drive_out_safe = true;
      }

      DEBUG_PRINT("is_drive_out_safe ="
                  << is_drive_out_safe
                  << ",  check pose =" << arc.pA.transpose()
                  << "  heading =" << arc.headingA * 57.3);
    }
  }

  if (arc.is_ignored) {
    arc.pB = arc.pA;
    arc.headingB = arc.headingA;
    arc.length = 0.0;
  }

  return true;
}

const bool ParallelPathPlanner::CheckParkOutCornerSafeWithObsPin(
    const pnc::geometry_lib::Arc& first_arc) const {
  const double rac_radius = first_arc.circle_info.radius;
  const Eigen::Vector2d center_in_ego(0.0, -rac_radius);

  const Eigen::Vector2d ego_fl_corner_0(
      apa_param.GetParam().car_vertex_x_vec[0],
      apa_param.GetParam().car_vertex_y_vec[0]);

  const Eigen::Vector2d ego_fl_corner_1(
      apa_param.GetParam().car_vertex_x_vec[1],
      apa_param.GetParam().car_vertex_y_vec[1]);

  const Eigen::Vector2d ego_fl_corner_middle =
      0.5 * (ego_fl_corner_0 + ego_fl_corner_1);

  const double corner_0_radius = (ego_fl_corner_0 - center_in_ego).norm();
  const double corner_1_radius = (ego_fl_corner_1 - center_in_ego).norm();
  const double corner_middle_radius =
      (ego_fl_corner_middle - center_in_ego).norm();

  double max_corner_radius = std::max(corner_0_radius, corner_1_radius);
  max_corner_radius = std::max(max_corner_radius, corner_middle_radius);

  const double center_to_obs_in =
      (first_arc.circle_info.center - input_.tlane.obs_pt_inside).norm();

  DEBUG_PRINT("center_to_obs_in = " << center_to_obs_in);
  DEBUG_PRINT("max_corner_radius = " << max_corner_radius);

  return (center_to_obs_in >=
          max_corner_radius +
              apa_param.GetParam().parallel_ego_front_corner_to_obs_in_buffer);
}

const bool ParallelPathPlanner::TwoSameGearArcPlanToLine(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::LineSegment& target_line, const uint8_t gear,
    const double buffer) {
  path_seg_vec.clear();
  path_seg_vec.reserve(4);

  if (!pnc::geometry_lib::IsValidGear(gear)) {
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
      col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
    return false;
  }
  col_res = collision_detector_ptr_->Update(arc_2, arc_2.headingA);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
    return false;
  }

  pnc::geometry_lib::LineSegment last_line(arc_2.pB, target_line.pA,
                                           arc_2.headingB);
  auto last_line_gear = pnc::geometry_lib::CalLineSegGear(last_line);
  col_res = collision_detector_ptr_->Update(last_line, last_line.heading);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
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
        col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
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
  DEBUG_PRINT("multi-plan ref gear =" << static_cast<int>(current_gear));
  DEBUG_PRINT("multi-plan ref arc = " << static_cast<int>(current_arc_steer));

  // check pose and slot_occupied_ratio, if error is small, multi isn't suitable
  if (!CheckMultiPlanSuitable(current_pose)) {
    std::cout << "pose err is relatively small, skip multi plan, directly try "
                 "adjust plan\n";
    return false;
  }
  // Todo: remove ref arc, gear is enough.
  // check gear
  if (!pnc::geometry_lib::IsValidGear(current_gear)) {
    std::cout << "ref_gear error\n";
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

    current_gear = pnc::geometry_lib::ReverseGear(current_gear);
    current_arc_steer = pnc::geometry_lib::ReverseSteer(current_arc_steer);
    current_pose = tmp_path_seg_vec.back().GetEndPose();

    if (IsOnTarget(current_pose)) {
      std::cout << "already plan to target pos!\n\n";
      break;
    }
  }
  DEBUG_PRINT("multi seg size =" << output_.path_segment_vec.size());
  return success;
}

// checkout multi suitable
const bool ParallelPathPlanner::CheckMultiPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose) const {
  // apa_param.GetParam().multi_plan_min_heading_err
  if (std::fabs(current_pose.heading) <= 5.0 / 57.3) {
    return false;
  }
  return true;
}

const bool ParallelPathPlanner::CalSinglePathInMulti(
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const uint8_t current_arc_steer,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  DEBUG_PRINT("-----CalSinglePathInMulti-----");
  DEBUG_PRINT("current_arc_steer = "
              << static_cast<int>(current_arc_steer)
              << ",  current_gear = " << static_cast<int>(current_gear)
              << ",  current_pos = " << current_pose.pos.transpose()
              << ",  current_heading = " << current_pose.heading * 57.3);

  path_seg_vec.clear();
  path_seg_vec.reserve(3);
  const double min_radius = apa_param.GetParam().min_turn_radius;

  const Eigen::Vector2d current_turn_center =
      CalEgoTurningCenter(current_pose, min_radius, current_arc_steer);

  pnc::geometry_lib::Arc current_arc;
  current_arc.pA = current_pose.pos;
  current_arc.headingA =
      pnc::geometry_lib::NormalizeAngle(current_pose.heading);

  current_arc.circle_info.radius = min_radius;
  current_arc.circle_info.center = current_turn_center;

  // cal the dist from current_turn_center to target line
  const double dist = pnc::geometry_lib::CalPoint2LineDist(
      current_turn_center, calc_params_.target_line);

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(4);
  bool success = false;
  MultiPlanMethod multi_plan_method = MultiPlanMethod::MultiPlanMethodCount;
  // determine the relationship between the turning circle of the car and
  // the target line
  if ((dist <=
       min_radius + apa_param.GetParam().parallel_multi_plan_radius_eps) &&
      (dist >=
       min_radius - apa_param.GetParam().parallel_multi_plan_radius_eps)) {
    // circle and line are tangent, try first
    std::cout << "center to line dist = " << dist << ",  try OneArcPlan\n";
    if (OneArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                   current_arc_steer)) {
      // std::cout << "OneArcPlan success\n";
      success = true;
      multi_plan_method = MultiPlanMethod::OneArcMultiPlan;
    } else {
      // std::cout << "OneArcPlan fail\n";
      success = false;
    }
  } else if (dist <
             min_radius - apa_param.GetParam().parallel_multi_plan_radius_eps) {
    // circle and line are intersected, try second
    std::cout << "center to line dist = " << dist << ",  try TwoArcPlan\n";
    if (TwoArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                   current_arc_steer)) {
      // if (tmp_path_seg_vec.front().Getlength() < 0.06 ||
      //     tmp_path_seg_vec.back().Getlength() < 0.06) {
      //   std::vector<pnc::geometry_lib::PathSegment> line_path_seg_vec;

      //   if (InsertTwoLinePathBetweenPathSeg(
      //           line_path_seg_vec, tmp_path_seg_vec.front().GetEndPose(),
      //           tmp_path_seg_vec.front().seg_gear, 0.2)) {
      //     tmp_path_seg_vec.insert(tmp_path_seg_vec.begin() + 1,
      //                             line_path_seg_vec.begin(),
      //                             line_path_seg_vec.end());
      //   }
      // }
      std::cout << "TwoArcPlan success\n";
      success = true;
      multi_plan_method = MultiPlanMethod::TwoArcMultiPlan;
    } else {
      std::cout << "TwoArcPlan fail\n";
      success = false;
    }
  } else if (dist >
             min_radius + apa_param.GetParam().parallel_multi_plan_radius_eps) {
    // circle and line are disjoint, try last
    std::cout << "center to line dist = " << dist << ",  try LineArcPlan\n";
    if (LineArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                    current_arc_steer)) {
      std::cout << "LineArcPlan success\n";
      success = true;
      multi_plan_method = MultiPlanMethod::LineArcMultiPlan;
    } else {
      std::cout << "LineArcPlan fail\n";
      success = false;
    }
  }

  if (!success) {
    tmp_path_seg_vec.clear();
  }

  // try one line plan
  pnc::geometry_lib::PathPoint last_pose = current_pose;
  if (tmp_path_seg_vec.size() > 0) {
    const auto& last_segment = tmp_path_seg_vec.back();
    last_pose.Set(last_segment.GetArcSeg().pB,
                  last_segment.GetArcSeg().headingB);
    // std::cout << "last path pose to one plan\n";
  }

  pnc::geometry_lib::LineSegment last_line;
  last_line.pA = last_pose.pos;
  last_line.heading = last_pose.heading;
  if (OneLinePlanAlongEgoHeading(last_line, calc_params_.target_pose)) {
    tmp_path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalLineSegGear(last_line), last_line));
  }

  uint8_t col_res = PATH_COL_COUNT;
  for (size_t i = 0; i < tmp_path_seg_vec.size(); i++) {
    auto& tmp_path_seg = tmp_path_seg_vec[i];
    col_res = TrimPathByCollisionDetection(tmp_path_seg, kColBufferInSlot);

    if (col_res == PATH_COL_NORMAL) {
      path_seg_vec.emplace_back(tmp_path_seg);
      DEBUG_PRINT("No. " << i << " normal!");
    } else if (col_res == PATH_COL_SHORTEN) {
      path_seg_vec.emplace_back(tmp_path_seg);
      DEBUG_PRINT("No. " << i << " cut, due to collision, end point ="
                         << tmp_path_seg.GetEndPos().transpose()
                         << ", heading ="
                         << tmp_path_seg.GetEndHeading() * 57.3);
      if (multi_plan_method == MultiPlanMethod::LineArcMultiPlan && i == 0) {
        DEBUG_PRINT("line-arc cut at line, quit multi-plan");
        return false;
      }
      break;
    } else if (col_res == PATH_COL_INVALID) {
      DEBUG_PRINT("path col at start pose, invalid!");
      break;
    }
  }

  if (col_res == PATH_COL_INVALID) {
    DEBUG_PRINT("start loose buffer to 0.1 in slot!");
    for (size_t i = 0; i < tmp_path_seg_vec.size(); i++) {
      auto& tmp_path_seg = tmp_path_seg_vec[i];
      col_res =
          TrimPathByCollisionDetection(tmp_path_seg, kSmallColBufferInSlot);

      if (col_res == PATH_COL_NORMAL) {
        path_seg_vec.emplace_back(tmp_path_seg);
        DEBUG_PRINT("No. " << i << " normal!");
      } else if (col_res == PATH_COL_SHORTEN) {
        path_seg_vec.emplace_back(tmp_path_seg);
        DEBUG_PRINT("No. " << i << " cut, due to collision, end point ="
                           << tmp_path_seg.GetEndPos().transpose()
                           << ", heading ="
                           << tmp_path_seg.GetEndHeading() * 57.3);
        if (multi_plan_method == MultiPlanMethod::LineArcMultiPlan && i == 0) {
          DEBUG_PRINT("line-arc cut at line, quit multi-plan");
          return false;
        }
        break;
      } else if (col_res == PATH_COL_INVALID) {
        DEBUG_PRINT("path col at start pose, invalid!");
        break;
      }
    }
  }

  success = false;
  if (path_seg_vec.size() > 0) {
    DEBUG_PRINT("CalSinglePathInMulti success, single path in multi:");
    double first_path_length = 0.0;

    for (const auto& path_seg : path_seg_vec) {
      pnc::geometry_lib::PrintSegmentInfo(path_seg);
      if (path_seg.seg_gear == path_seg_vec.front().seg_gear) {
        first_path_length += path_seg.Getlength();
      }
    }

    if (first_path_length < 0.06) {
      success = false;
      path_seg_vec.clear();
      DEBUG_PRINT("path is too short, length = " << first_path_length);
    } else {
      success = true;
    }
  } else {
    DEBUG_PRINT("CalSinglePathInMulti fail");
    success = false;
  }
  return success;
}

const bool ParallelPathPlanner::MultiAlignBody() {
  std::cout << "-----MultiAlignBodyPlan-----\n";
  // set init state
  uint8_t current_gear = input_.ref_gear;
  DEBUG_PRINT("ref gear = " << static_cast<int>(current_gear));

  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  pnc::geometry_lib::PrintPose("start pose", current_pose);

  if (!pnc::geometry_lib::IsValidGear(current_gear)) {
    DEBUG_PRINT("ref_gear error!");
    return false;
  }

  if (input_.slot_occupied_ratio < 0.3) {
    DEBUG_PRINT("not in slot!");
    return false;
  }

  // check pose and slot_occupied_ratio, if error is small, multi isn't suitable
  if (std::fabs(current_pose.heading * 57.3) <=
      apa_param.GetParam().finish_parallel_heading_err) {
    DEBUG_PRINT("body already aligned!");
    return false;
  }

  std::vector<pnc::geometry_lib::PathSegment> single_aligned_path;
  std::vector<pnc::geometry_lib::PathSegment> path_res;
  path_res.clear();
  path_res.reserve(kMaxMultiStepNums);

  bool success = false;
  while (std::fabs(current_pose.heading * 57.3) >
         apa_param.GetParam().finish_parallel_heading_err) {
    single_aligned_path.clear();
    single_aligned_path.reserve(1);

    if (AlignBodyPlan(single_aligned_path, current_pose,
                      calc_params_.target_pose.heading, current_gear)) {
      auto col_res =
          TrimPathByCollisionDetection(single_aligned_path.back(), 0.2);

      if (col_res == PATH_COL_SHORTEN) {
        success = true;
        path_res.emplace_back(single_aligned_path.back());

      } else if (col_res == PATH_COL_NORMAL) {
        success = true;
        path_res.emplace_back(single_aligned_path.back());
        break;
      } else if (col_res == PATH_COL_INVALID) {
        success = false;
      }

      if (!success) {
        DEBUG_PRINT("normal buffer failed");
        col_res = TrimPathByCollisionDetection(single_aligned_path.back(), 0.1);

        if (col_res == PATH_COL_SHORTEN) {
          success = true;
          path_res.emplace_back(single_aligned_path.back());

        } else if (col_res == PATH_COL_NORMAL) {
          success = true;
          path_res.emplace_back(single_aligned_path.back());
          break;
        } else if (col_res == PATH_COL_INVALID) {
          success = false;
          break;
        }
      }
    }
    current_pose = single_aligned_path.back().GetEndPose();
    current_gear = pnc::geometry_lib::ReverseGear(current_gear);
  }

  if (success) {
    pnc::geometry_lib::LineSegment last_line;
    const Eigen::Vector2d last_pt_start = path_res.back().GetEndPos();
    const Eigen::Vector2d last_pt_end(input_.tlane.pt_terminal_pos.x(),
                                      last_pt_start.y());
    last_line.SetPoints(last_pt_start, last_pt_end);
    const pnc::geometry_lib::PathSegment last_line_path(
        pnc::geometry_lib::CalLineSegGear(last_line), last_line);
    if ((last_line_path.seg_gear == path_res.back().seg_gear) ||
        (last_line_path.seg_gear != path_res.back().seg_gear &&
         last_line_path.GetLineSeg().length >=
             apa_param.GetParam().min_line_length)) {
      path_res.emplace_back(last_line_path);
    }

    output_.Reset();
    AddPathSegToOutPut(path_res);
  } else {
    DEBUG_PRINT("small buffer failed");
  }

  return success;
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
  input_.ego_pose.heading =
      pnc::geometry_lib::NormalizeAngle(input_.ego_pose.heading);

  auto current_gear = input_.ref_gear;
  auto current_pose = input_.ego_pose;
  pnc::geometry_lib::PrintPose("input pose =", input_.ego_pose);

  // dirve backward directly if can park near target pose
  pnc::geometry_lib::LineSegment first_line;
  first_line.pA = input_.ego_pose.pos;
  first_line.heading = input_.ego_pose.heading;

  if (OneLinePlanInCSCS(first_line, calc_params_.target_pose)) {
    DEBUG_PRINT("firstly calc line success");
    AddPathSegToOutPut(pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalLineSegGear(first_line), first_line));
    return true;
  }

  if (output_.path_segment_vec.size() > 0) {
    const auto& last_seg = output_.path_segment_vec.back();
    current_pose.Set(last_seg.GetEndPos(), last_seg.GetEndHeading());
    current_gear = pnc::geometry_lib::ReverseGear(last_seg.seg_gear);
    std::cout << "continue to plan after multi\n";
  }

  std::cout << "current pose =" << current_pose.pos.transpose() << ", "
            << current_pose.heading * 57.3 << std::endl;

  // check gear and steer
  if (!pnc::geometry_lib::IsValidGear(current_gear)) {
    DEBUG_PRINT("ref_gear or ref_arc_steer error");
    return false;
  }

  std::cout << "input gear =" << static_cast<int>(current_gear) << std::endl;

  // DEBUG_PRINT("try adjust plan to target point");
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(5);
  // 1. try one arc
  //  Todo: target line should changed to be in range of +- 0.03cm
  bool success = OneArcPlan(tmp_path_seg_vec, current_pose, current_gear);

  if (success) {
    if (!CheckPathSegCollided(tmp_path_seg_vec.back(), kColBufferInSlot)) {
      std::cout << "only one arc success!" << std::endl;
      AddPathSegVecToOutput(tmp_path_seg_vec);
      return true;
    }
  }
  success = false;
  DEBUG_PRINT("One arc failed!");

  // 2. check if should align body
  if (!pnc::mathlib::IsDoubleEqual(current_pose.heading,
                                   calc_params_.target_line.heading)) {
    // need align body
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(5);
    if (AlignBodyPlan(tmp_path_seg_vec, current_pose, current_gear)) {
      if (!CheckPathSegCollided(tmp_path_seg_vec.back())) {
        success = true;
        AddPathSegToOutPut(tmp_path_seg_vec.back());
        pnc::geometry_lib::PrintPose("align body success! start pose",
                                     current_pose);
        current_pose = tmp_path_seg_vec.back().GetEndPose();
        pnc::geometry_lib::PrintPose("align body success! end pose",
                                     current_pose);
      }
    }
    if (!success) {
      DEBUG_PRINT("align body failed!");
      return false;
    }
  }

  DEBUG_PRINT("body already align");
  if (IsOnTargetLine(current_pose)) {
    pnc::geometry_lib::LineSegment last_line;
    last_line.pA = current_pose.pos;
    last_line.heading = current_pose.heading;

    if (CheckLonToTarget(current_pose)) {
      return true;
    }

    if (OneLinePlanAlongEgoHeading(last_line, calc_params_.target_pose)) {
      AddPathSegToOutPut(pnc::geometry_lib::PathSegment(
          pnc::geometry_lib::CalLineSegGear(last_line), last_line));
      return true;
    }
  }

  bool loop_success = false;
  std::vector<pnc::geometry_lib::PathSegment> parallel_shift_path_vec;
  parallel_shift_path_vec.clear();
  parallel_shift_path_vec.reserve(12);
  for (size_t i = 0; i < kMaxParallelShiftNums; ++i) {
    DEBUG_PRINT("-------- No." << i << " in paralle adjust-plan--------");

    bool s_turn_success = false;
    std::vector<pnc::geometry_lib::PathSegment> s_turn_vec;
    double ratio = 1.0;
    for (; ratio > 0.1; ratio -= 0.1) {
      s_turn_vec.clear();
      s_turn_vec.reserve(2);
      if (STurnParallelPlan(s_turn_vec, current_pose, calc_params_.target_line,
                            current_gear, ratio,
                            apa_param.GetParam().min_turn_radius)) {
        if (!CheckPathSegVecCollided(s_turn_vec, kColBufferInSlot)) {
          s_turn_success = true;
          parallel_shift_path_vec.insert(parallel_shift_path_vec.end(),
                                         s_turn_vec.begin(), s_turn_vec.end());
          break;
        }
      }
    }
    // Todo: need reverse gear and check s turn again
    if (!s_turn_success) {
      current_gear = pnc::geometry_lib::ReverseGear(current_gear);
      ratio = 1.0;
      for (; ratio > 0.1; ratio -= 0.1) {
        s_turn_vec.clear();
        s_turn_vec.reserve(2);
        if (STurnParallelPlan(s_turn_vec, current_pose,
                              calc_params_.target_line, current_gear, ratio,
                              apa_param.GetParam().min_turn_radius)) {
          if (!CheckPathSegVecCollided(s_turn_vec, kColBufferInSlot)) {
            s_turn_success = true;
            parallel_shift_path_vec.insert(parallel_shift_path_vec.end(),
                                           s_turn_vec.begin(),
                                           s_turn_vec.end());
            break;
          }
        }
      }
      if (!s_turn_success) {
        loop_success = false;
        DEBUG_PRINT("shift plan fail with current gear");
        break;
      }
    }

    // sturn success
    if (ratio == 1.0) {
      loop_success = true;
      DEBUG_PRINT("shift loop success, try last line!");
      break;
    } else if (IsOnTargetLine(s_turn_vec.back().GetEndPose())) {
      loop_success = true;
      DEBUG_PRINT("close to target line, try last line!");
      break;
    }
    // ratio is less than 1.0
    current_gear = pnc::geometry_lib::ReverseGear(current_gear);
    current_pose.Set(s_turn_vec.back().GetEndPos(),
                     s_turn_vec.back().GetEndHeading());
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

      if (OneLinePlanAlongEgoHeading(last_line, target_pose)) {
        DEBUG_PRINT("calc line success");
        if (!last_line.is_ignored) {
          parallel_shift_path_vec.emplace_back(pnc::geometry_lib::PathSegment(
              pnc::geometry_lib::CalLineSegGear(last_line), last_line));
          DEBUG_PRINT("last line exist");
        }
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

const bool ParallelPathPlanner::CSCSAdjustPlan() {
  DEBUG_PRINT("-----CSCS AdjustPlan-----");
  // set init state
  pnc::geometry_lib::PathPoint start_pose = input_.ego_pose;
  const uint8_t start_gear = input_.ref_gear;

  const auto& terminal_line = calc_params_.target_line;

  pnc::geometry_lib::LineSegment line;
  line.pA = start_pose.pos;
  line.heading = start_pose.heading;
  if (OneLinePlanInCSCS(line, calc_params_.target_pose)) {
    AddPathSegToOutPut(pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalLineSegGear(line), line));
    return true;
  }

  const uint8_t start_steer = (pnc::geometry_lib::IsPointOnLeftSideOfLineSeg(
                                   start_pose.pos, terminal_line)
                                   ? pnc::geometry_lib::SEG_STEER_RIGHT
                                   : pnc::geometry_lib::SEG_STEER_LEFT);
  bool success = false;

  // const bool is_small_heading =
  //     std::fabs(pnc::geometry_lib::NormalizeAngle(
  //         start_pose.heading - terminal_line.heading)) < 9.0 / 57.3;
  const bool is_reverse = (start_gear == pnc::geometry_lib::SEG_GEAR_REVERSE);

  if (is_reverse) {
    DEBUG_PRINT("reverse");
    // check if line step is enough

    // try cscs
    double shift_ratio = 1.0;
    for (; shift_ratio > 0.05; shift_ratio -= 0.1) {
      std::vector<pnc::geometry_lib::PathPoint> target_tan_pose_vec;
      if (SameSteerCSCToLine(target_tan_pose_vec, start_steer, start_gear,
                             start_pose, terminal_line, shift_ratio)) {
        GenPathOutputByDubins();

        if (pnc::mathlib::IsDoubleEqual(shift_ratio, 1.0)) {
          pnc::geometry_lib::LineSegment last_line(
              dubins_planner_.GetOutput().arc_CD.pB, terminal_line.pA,
              terminal_line.heading);
          // Todo: extend line if last line is too short and the gear is
          // different from second to last
          AddPathSegToOutPut(pnc::geometry_lib::PathSegment(
              pnc::geometry_lib::CalLineSegGear(last_line), last_line));
        }
        success = true;
        break;
      }
    }
  } else {
    DEBUG_PRINT("dirve step!");
    // drive
    std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;

    if (CSCSInclinedStepPlan(path_seg_vec, start_pose, terminal_line,
                             start_gear)) {
      success = true;
      AddPathSegVecToOutput(path_seg_vec);
    }
  }

  return success;
}

const bool ParallelPathPlanner::SameSteerCSCToLine(
    std::vector<pnc::geometry_lib::PathPoint>& target_tan_pose_vec,
    const uint8_t start_steer, const uint8_t start_gear,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::LineSegment& terminal_line,
    const double shift_ratio) {
  // target_tan_pose_vec is filled with x coord from small to large
  if (!CalcLineDirAllValidPose(target_tan_pose_vec, start_pose, terminal_line,
                               shift_ratio)) {
    target_tan_pose_vec.clear();
    DEBUG_PRINT("calc line two limit error!");
    return false;
  }
  // PrintPose("line bottom limit", target_tan_pose_vec.front());
  // PrintPose("line top limit", target_tan_pose_vec.back());

  const double start_dir_sgn =
      (start_gear == pnc::geometry_lib::SEG_GEAR_DRIVE ? 1.0 : -1.0);

  bool success = false;
  for (const auto& target_pose : target_tan_pose_vec) {
    // first select target_pose x coord which is more than start pose when
    // gear is drive, vise versa.
    if (target_pose.pos.x() * start_dir_sgn <
        start_pose.pos.x() * start_dir_sgn) {
      // DEBUG_PRINT("x not suitable! " << start_pose.pos.x());
      continue;
    }

    // use two step LSL or RSR to target line
    if (!SameSteerCSCPlan(start_pose, target_pose, start_steer, start_gear)) {
      // DEBUG_PRINT("CSCS failed!");
      continue;
    }
    success = true;
    break;
  }
  return success;
}

const bool ParallelPathPlanner::CalcLineDirAllValidPose(
    std::vector<pnc::geometry_lib::PathPoint>& target_tan_pose_vec,
    const pnc::geometry_lib::PathPoint start_pose,
    const pnc::geometry_lib::LineSegment& terminal_line,
    const double shift_ratio) {
  Eigen::Vector2d line_norm_vec = Eigen::Vector2d::Zero();

  // line_norm_vec starts from point to line
  if (!pnc::geometry_lib::CalLineUnitNormVecByPos(start_pose.pos, terminal_line,
                                                  line_norm_vec)) {
    DEBUG_PRINT("calc norm unit failed!");
    return false;
  }

  const double total_lat_dist =
      pnc::geometry_lib::CalPoint2LineDist(start_pose.pos, terminal_line);

  const double shift_lat_dist = shift_ratio * total_lat_dist;

  pnc::geometry_lib::LineSegment target_line =
      pnc::geometry_lib::BuildLineSegByPose(
          start_pose.pos + shift_lat_dist * line_norm_vec,
          terminal_line.heading);

  bool is_success = CalcLineDirAllValidPose(target_tan_pose_vec, target_line);

  if (is_success) {
    // DEBUG_PRINT("total lat dist =" << total_lat_dist);
    // DEBUG_PRINT("shift lat dist =" << shift_lat_dist);
  }

  return is_success;
}

const bool ParallelPathPlanner::CalcLineDirAllValidPose(
    std::vector<pnc::geometry_lib::PathPoint>& target_tan_pose_vec,
    const pnc::geometry_lib::LineSegment& target_line) {
  target_tan_pose_vec.clear();
  target_tan_pose_vec.reserve(100);

  // first search in drive gear
  pnc::geometry_lib::LineSegment forward_line_seg;
  forward_line_seg.pA = target_line.pA;
  forward_line_seg.heading = target_line.heading;

  if (!CalcLineStepLimitPose(forward_line_seg,
                             pnc::geometry_lib::SEG_GEAR_DRIVE)) {
    DEBUG_PRINT("calc forward limit pose error");
    return false;
  }

  pnc::geometry_lib::LineSegment backward_line_seg;
  backward_line_seg.pA = forward_line_seg.pB;
  backward_line_seg.heading = forward_line_seg.heading;

  if (!CalcLineStepLimitPose(backward_line_seg,
                             pnc::geometry_lib::SEG_GEAR_REVERSE)) {
    DEBUG_PRINT("calc backward limit pose error");
    return false;
  }

  if ((backward_line_seg.pA - backward_line_seg.pB).norm() < 0.3) {
    DEBUG_PRINT("two limit pose too close along the line!");
    return false;
  }

  std::vector<Eigen::Vector2d> pt_vec = pnc::geometry_lib::LinSpace(
      backward_line_seg.pB, backward_line_seg.pA, 0.05);

  pnc::geometry_lib::PathPoint tmp_pose;
  for (const auto& pt : pt_vec) {
    tmp_pose.Set(pt, backward_line_seg.heading);
    target_tan_pose_vec.emplace_back(tmp_pose);
  }

  return true;
}

const bool ParallelPathPlanner::SameSteerCSCPlan(
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const uint8_t start_steer,
    const uint8_t start_gear) {
  if ((!pnc::geometry_lib::IsValidGear(start_gear)) ||
      (!pnc::geometry_lib::IsValidArcSteer(start_steer))) {
    DEBUG_PRINT("gear or steer input error!");
    return false;
  }

  pnc::dubins_lib::DubinsLibrary::Input dubins_input;
  dubins_input.radius = apa_param.GetParam().min_turn_radius;
  dubins_input.Set(start_pose.pos, target_pose.pos, start_pose.heading,
                   target_pose.heading);
  dubins_planner_.SetInput(dubins_input);

  uint8_t dubins_type =
      pnc::dubins_lib::DubinsLibrary::DubinsType::DUBINS_TYPE_COUNT;
  if (start_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
    dubins_type = pnc::dubins_lib::DubinsLibrary::DubinsType::L_S_L;
  } else if (start_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
    dubins_type = pnc::dubins_lib::DubinsLibrary::DubinsType::R_S_R;
  } else {
    DEBUG_PRINT("steer input error!");
    return false;
  }

  bool success = false;
  for (uint8_t case_type = pnc::dubins_lib::DubinsLibrary::CASE_A;
       case_type < pnc::dubins_lib::DubinsLibrary::CASE_COUNT; case_type++) {
    if (!dubins_planner_.Solve(dubins_type, case_type)) {
      // DEBUG_PRINT("dubins solve failed!");
      continue;
    }

    const auto& dubins_output = dubins_planner_.GetOutput();

    // check heading
    const double theta_diff_AB_abs =
        std::fabs(pnc::geometry_lib::NormalizeAngle(
            dubins_output.arc_AB.headingA - dubins_output.arc_AB.headingB));

    if (theta_diff_AB_abs >= 35.0 / 57.3) {
      // DEBUG_PRINT("theta diff is more than 40 deg");
      continue;
    }

    if (theta_diff_AB_abs <= 10.0 / 57.3) {
      // DEBUG_PRINT("theta diff is less than 10 deg");
      continue;
    }

    // check gear of all steps. the first two should be same, the last different
    if (!dubins_output.arc_AB.is_ignored &&
        pnc::geometry_lib::CalArcGear(dubins_output.arc_AB) != start_gear) {
      // DEBUG_PRINT("ab gear error!");
      continue;
    }

    if (!dubins_output.line_BC.is_ignored &&
        pnc::geometry_lib::CalLineSegGear(dubins_output.line_BC) !=
            start_gear) {
      // DEBUG_PRINT("bc gear error!");
      continue;
    }

    if (!dubins_output.arc_CD.is_ignored &&
        pnc::geometry_lib::CalArcGear(dubins_output.arc_CD) == start_gear) {
      // DEBUG_PRINT("cd gear error!");
      continue;
    }

    if (IsDubinsCollided(kColBufferInSlot)) {
      // DEBUG_PRINT("collided!");
      continue;
    }
    success = true;
    break;
  }

  return success;
}

const bool ParallelPathPlanner::CSCSInclinedStepPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::LineSegment& target_line,
    const uint8_t current_gear) {
  const pnc::geometry_lib::PathPoint target_pose(target_line.pA,
                                                 target_line.heading);

  DEBUG_PRINT("-----CSCSInclinedStepPlan-----");
  DEBUG_PRINT("current_gear = " << static_cast<int>(current_gear));
  pnc::geometry_lib::PrintPose("current_pose", current_pose);
  pnc::geometry_lib::PrintPose("target_pose", target_pose);

  // calc ref steer according to ego heading and gear
  const uint8_t current_steer =
      (((current_pose.heading * 57.3 < 0.0 &&
         current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) ||
        (current_pose.heading * 57.3 > 0.0 &&
         current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE))
           ? pnc::geometry_lib::SEG_STEER_RIGHT
           : pnc::geometry_lib::SEG_STEER_LEFT);

  pnc::geometry_lib::Arc current_arc;
  current_arc.pA = current_pose.pos;
  current_arc.headingA =
      pnc::geometry_lib::NormalizeAngle(current_pose.heading);

  const double min_radius = apa_param.GetParam().min_turn_radius;
  current_arc.circle_info.radius = min_radius;
  current_arc.circle_info.center =
      CalEgoTurningCenter(current_pose, min_radius, current_steer);

  // cal the dist from current_turn_center to target line
  const double dist = pnc::geometry_lib::CalPoint2LineDist(
      current_arc.circle_info.center, target_line);

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(2);
  bool success = false;
  // determine the relationship between the turning circle of the car and
  // the target line
  DEBUG_PRINT("actual dist =" << dist << ", min_radius =" << min_radius);
  if ((dist <= min_radius + apa_param.GetParam().radius_eps) &&
      (dist >= min_radius - apa_param.GetParam().radius_eps)) {
    DEBUG_PRINT("tangent, try one arc");
    // circle and line are tangent, try first
    success =
        OneArcPlan(current_arc, tmp_path_seg_vec, current_gear, current_steer);

    if (success) {
      DEBUG_PRINT("OneArcPlan success");
    } else {
      DEBUG_PRINT("OneArcPlan fail");
    }
  } else if (dist > min_radius + apa_param.GetParam().radius_eps) {
    // circle and line are disjoint, try last
    DEBUG_PRINT("disjoint, try LineArcPlan");
    success =
        LineArcPlan(current_arc, tmp_path_seg_vec, current_gear, current_steer);

    if (success) {
      DEBUG_PRINT("LineArcPlan success");
    } else {
      DEBUG_PRINT("LineArcPlan fail");
    }
  } else if (dist < min_radius - apa_param.GetParam().radius_eps) {
    DEBUG_PRINT("intersected, correct heading and move to limit along line");

    success = pnc::geometry_lib::CalOneArcWithTargetHeading(
        current_arc, current_gear, target_line.heading);
    if (success) {
      // const double dir_sgn =
      //     (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE ? 1.0 : -1.0);
      // // temporarily give a line which is long enough
      // const double len = 6.6;
      // const auto v_ego_heading =
      //     pnc::geometry_lib::GenHeadingVec(current_arc.headingB);

      // const auto tmp_end_pos = current_arc.pB + dir_sgn * len *
      // v_ego_heading;

      // pnc::geometry_lib::LineSegment last_line(current_arc.pB, tmp_end_pos,
      //                                          current_arc.headingB);

      tmp_path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
          pnc::geometry_lib::CalArcSteer(current_arc),
          pnc::geometry_lib::CalArcGear(current_arc), current_arc));

      // tmp_path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
      //     pnc::geometry_lib::CalLineSegGear(last_line), last_line));
    }

    if (success) {
      DEBUG_PRINT("correct heading and move to limit along line success");
    } else {
      DEBUG_PRINT("correct heading and move to limit along line fail");
    }
  }

  if (!success) {
    tmp_path_seg_vec.clear();
  }

  // try one line plan
  pnc::geometry_lib::PathPoint last_pose = current_pose;
  if (tmp_path_seg_vec.size() > 0) {
    const auto& last_segment = tmp_path_seg_vec.back();
    last_pose.Set(last_segment.GetArcSeg().pB,
                  last_segment.GetArcSeg().headingB);
  }

  if (!CheckSamePose(last_pose, target_pose)) {
    // try line
    pnc::geometry_lib::LineSegment last_line;
    last_line.pA = last_pose.pos;
    last_line.heading = last_pose.heading;
    if (!OneLinePlan(last_line, target_pose)) {
      // line failed, just move to the limit in slot with same gear for next
      // cscs plan
      std::cout << "extend along line to limit" << std::endl;
      const double limit_len = 6.0;

      const double vel_heading =
          (tmp_path_seg_vec.back().seg_gear == pnc::geometry_lib::SEG_GEAR_DRIVE
               ? last_line.heading
               : pnc::geometry_lib::NormalizeAngle(last_line.heading + M_PI));

      pnc::geometry_lib::CompleteLineInfo(last_line, limit_len, vel_heading);
    }
    std::cout << "set last line --========!" << std::endl;
    tmp_path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalLineSegGear(last_line), last_line));
  } else {
    DEBUG_PRINT("already plan to target pos, no need to one line plan!");
  }

  // std::cout << "rougn tmp_path_seg_vec without col: --------" << std::endl;
  // pnc::geometry_lib::PrintSegmentsVecInfo(tmp_path_seg_vec);

  int i = -1;
  uint8_t path_col_det_res = PATH_COL_COUNT;
  for (auto& tmp_path_seg : tmp_path_seg_vec) {
    i++;
    path_col_det_res =
        TrimPathByCollisionDetection(tmp_path_seg, kColBufferInSlot);

    if (path_col_det_res == PATH_COL_NORMAL) {
      path_seg_vec.emplace_back(tmp_path_seg);
      DEBUG_PRINT("No. " << i << " normal!");
    } else if (path_col_det_res == PATH_COL_INVALID) {
      DEBUG_PRINT("path col invalid!");
      break;
    } else if (path_col_det_res == PATH_COL_SHORTEN) {
      path_seg_vec.emplace_back(tmp_path_seg);
      DEBUG_PRINT(
          "No. "
          << i << " seg "
          << "cut partial of cscs single path due to collision, end point ="
          << tmp_path_seg.GetEndPos().transpose()
          << ", heading =" << tmp_path_seg.GetEndHeading() * 57.3);
      break;
    }
  }

  if (path_col_det_res == PATH_COL_NORMAL) {
    DEBUG_PRINT("single path normal, collision free!");
  }

  if (path_seg_vec.size() > 0) {
    DEBUG_PRINT("inclined step in cscs plan success -------");
    DEBUG_PRINT("path after trim: ");
    double length = 0.0;
    // pnc::geometry_lib::PrintSegmentsVecInfo(path_seg_vec);
    // cacl length of first path with same gear
    for (const auto& path_seg : path_seg_vec) {
      if (path_seg.seg_gear == path_seg_vec.front().seg_gear) {
        length += path_seg.Getlength();
      } else {
        break;
      }
    }

    if (length < 0.1) {
      DEBUG_PRINT("this gear path is too small, lose it");
      path_seg_vec.clear();
      return false;
    }
    DEBUG_PRINT("length is long enough");
    return true;
  }
  return false;
}

const bool ParallelPathPlanner::OneArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear, const uint8_t current_arc_steer) {
  if (!CalOneArcWithLine(arc, calc_params_.target_line,
                         apa_param.GetParam().parallel_multi_plan_radius_eps)) {
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
    //     "current heading is equal to target heading or current y is equal
    //     to " "target y, no need to one arc plan");
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

const bool ParallelPathPlanner::OneArcPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::LineSegment& target_line,
    const uint8_t current_gear) {
  // DEBUG_PRINT("try one arc plan");

  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;

  if (pnc::mathlib::IsDoubleEqual(arc.headingA, target_line.heading) ||
      pnc::mathlib::IsDoubleEqual(arc.pA.y(), target_line.pA.y())) {
    DEBUG_PRINT("small heading or y coord diff, quit one arc plan");
    return false;
  }

  bool success = pnc::geometry_lib::CalOneArcWithLineAndGear(arc, target_line,
                                                             current_gear);
  if (success) {
    // check radius and gear can or not meet needs
    // DEBUG_PRINT("cal radius = " << arc.circle_info.radius);
    const auto steer = pnc::geometry_lib::CalArcSteer(arc);
    const auto gear = pnc::geometry_lib::CalArcGear(arc);
    const auto arc_radius = arc.circle_info.radius;

    success =
        (arc_radius >= apa_param.GetParam().min_turn_radius - 1e-3 &&
         arc_radius <= apa_param.GetParam().max_one_step_arc_radius + 1e-3) &&
        (gear == current_gear);

    if (success) {
      DEBUG_PRINT("one arc plan success!");
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
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

  if (steer_1 != current_arc_steer || gear_1 != current_gear) {
    std::cout << "TwoArcPlan arc1 fail\n";
    return false;
  }
  if (steer_2 != next_arc_steer || gear_2 != next_gear) {
    std::cout << "TwoArcPlan arc2 fail\n";
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
  //   pnc::geometry_lib::PrintSegmentInfo(tmp_path_seg);
  // }

  // collision detection
  for (auto& tmp_path_seg : tmp_path_seg_vec) {
    const uint8_t path_col_det_res =
        TrimPathByCollisionDetection(tmp_path_seg, kColBufferInSlot);

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
      pnc::geometry_lib::PrintSegmentInfo(path_seg);
      length += path_seg.Getlength();
    }
    if (length < 0.1) {
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
    pnc::geometry_lib::PathSegment& path_seg, const double buffer) {
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
      std::min(remain_car_dist, remain_obs_dist - buffer);

  // std::cout << "  remain_car_dist = " << remain_car_dist
  //           << "  remain_obs_dist = " << remain_obs_dist
  //           << "  safe_remain_dist = " << safe_remain_dist << std::endl;

  if (safe_remain_dist < 0.0) {
    // std::cout << "the distance between obstacle and ego is smaller than "
    //    "min_safe_distance, collided! "
    // << std::endl;
    std::cout << col_res.collision_point.transpose() << std::endl;
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
    pnc::geometry_lib::PathSegment& path_seg, Eigen::Vector2d& collision_pt,
    const double buffer) {
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
      std::min(remain_car_dist, remain_obs_dist - buffer);

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

    if (line.length > 0.02) {
      // if (line.length > apa_param.GetParam().static_pos_eps) {
      const uint8_t seg_gear = pnc::geometry_lib::CalLineSegGear(line);

      if (pnc::geometry_lib::IsValidGear(seg_gear)) {
        std::cout << "the line gear is invalid\n";
        return false;
      }
      pnc::geometry_lib::PathSegment line_seg(seg_gear, line);
      path_seg_vec.emplace_back(line_seg);
      return true;
    } else {
      std::cout << "already plan to target pos\n";
      return true;
    }

  } else {
    std::cout << "pose is not on line, fail\n";
    return false;
  }
}

const bool ParallelPathPlanner::OneLinePlan(
    pnc::geometry_lib::LineSegment& line,
    const pnc::geometry_lib::PathPoint& target_pose) const {
  std::cout << "--- try one line plan ---\n";

  const pnc::geometry_lib::PathPoint start_pose(line.pA, line.heading);

  std::cout << "line start pose =" << start_pose.pos.transpose()
            << ", heading(deg) =" << start_pose.heading * 57.3 << std::endl;

  auto target_line = pnc::geometry_lib::BuildLineSegByPose(target_pose.pos,
                                                           target_pose.heading);
  std::cout << "target line pA =" << target_line.pA.transpose()
            << "heading(deg) =" << target_line.heading * 57.3 << std::endl;

  if (!pnc::geometry_lib::IsPoseOnLine(
          start_pose, target_line, apa_param.GetParam().static_pos_eps,
          apa_param.GetParam().static_heading_eps / 57.3)) {
    std::cout << "pose is not on line, fail\n";
    return false;
  }
  DEBUG_PRINT("pose is on line, success");

  line.SetPoints(start_pose.pos, target_line.pA);

  if (line.length > 0.02) {
    const uint8_t seg_gear = pnc::geometry_lib::CalLineSegGear(line);
    if (!pnc::geometry_lib::IsValidGear(seg_gear)) {
      std::cout << "the line gear is invalid\n";
      return false;
    }
    DEBUG_PRINT("line plan to target pos success");
  } else {
    line.is_ignored = true;
    DEBUG_PRINT("already is on target pos");
  }
  return true;
}

const bool ParallelPathPlanner::OneLinePlanInCSCS(
    pnc::geometry_lib::LineSegment& line,
    const pnc::geometry_lib::PathPoint& target_pose) const {
  const pnc::geometry_lib::PathPoint start_pose(line.pA, line.heading);

  auto target_line = pnc::geometry_lib::BuildLineSegByPose(target_pose.pos,
                                                           target_pose.heading);
  // std::cout << "target line pA =" << target_line.pA.transpose()
  //           << "heading(deg) =" << target_line.heading * 57.3 << std::endl;

  const double lat_dif_mag = std::fabs(start_pose.pos.y() - target_line.pA.y());
  const double heading_dif_mag = std::fabs(pnc::geometry_lib::NormalizeAngle(
      start_pose.heading - target_pose.heading));

  if (lat_dif_mag > apa_param.GetParam().finish_parallel_lat_rac_err ||
      heading_dif_mag >
          apa_param.GetParam().finish_parallel_heading_err / 57.3) {
    // std::cout << "pose is not on line, fail\n";
    return false;
  }

  const double fixed_y_coor = 0.5 * (start_pose.pos.y() + target_line.pA.y());
  const Eigen::Vector2d fixed_pa(start_pose.pos.x(), fixed_y_coor);
  const Eigen::Vector2d fixed_pb(target_line.pA.x(), fixed_y_coor);

  line.SetPoints(fixed_pa, fixed_pb);
  line.heading = target_pose.heading;

  if (line.length > 0.02) {
    const uint8_t seg_gear = pnc::geometry_lib::CalLineSegGear(line);
    if (!pnc::geometry_lib::IsValidGear(seg_gear)) {
      // std::cout << "the line gear is invalid\n";
      return false;
    }
    // DEBUG_PRINT("line plan to target pos success");
  } else {
    line.is_ignored = true;
    // DEBUG_PRINT("already is on target pos");
  }
  return true;
}

const bool ParallelPathPlanner::OneLinePlanAlongEgoHeading(
    pnc::geometry_lib::LineSegment& line,
    const pnc::geometry_lib::PathPoint& target_pose) {
  std::cout << "--- try OneLinePlanAlongEgoHeading ---\n";

  const pnc::geometry_lib::PathPoint start_pose(line.pA, line.heading);

  const Eigen::Vector2d v_start_heading(std::cos(start_pose.heading),
                                        std::sin(start_pose.heading));

  if (pnc::mathlib::IsDoubleEqual(v_start_heading.x(), 0.0)) {
    DEBUG_PRINT("ego heading is not possible equal to 90 deg, calc failed!");
    return false;
  }

  // firstly extend ego line to the same x coordination as target pose
  double len = (target_pose.pos.x() - start_pose.pos.x()) / v_start_heading.x();

  const pnc::geometry_lib::PathPoint fixed_target_pose(
      start_pose.pos + len * v_start_heading, start_pose.heading);

  DEBUG_PRINT("fixed target pose =" << fixed_target_pose.pos.transpose()
                                    << " ,heading ="
                                    << fixed_target_pose.heading * 57.3);

  if (!IsOnTargetLine(fixed_target_pose)) {
    std::cout << "pose is not on line, fail\n";
    return false;
  }
  DEBUG_PRINT("pose is on line, success");

  line.SetPoints(start_pose.pos, fixed_target_pose.pos);
  if (line.length > 0.06) {
    const uint8_t seg_gear = pnc::geometry_lib::CalLineSegGear(line);
    if (!pnc::geometry_lib::IsValidGear(seg_gear)) {
      std::cout << "the line gear is invalid\n";
      return false;
    }
    DEBUG_PRINT("line plan to target pos success");
  } else {
    DEBUG_PRINT("path is too short");
    return false;
  }
  return true;
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

  const int first = output_.path_seg_index.first;
  const int second = output_.path_seg_index.second;
  if (first < 0 || second < 0 || first > second) {
    std::cout << "first and second index is err\n";
    return false;
  }
  for (int i = second; i >= first; --i) {
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
  // if (output_.is_last_path == true) {
  //   std::cout << "is last path, not extend path\n";
  //   return;
  // }


  if (output_.path_segment_vec.size() < 1) {
    return;
  }

  auto& path_seg = output_.path_segment_vec[output_.path_seg_index.second];

  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
      extend_distance < 0.0) {
    std::cout << "arc can not shorten\n";
    return;
  }

  double path_len = 0.0;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; i++) {
    path_len += output_.path_segment_vec[i].Getlength();
  }

  if (extend_distance > 0.0) {
    pnc::geometry_lib::PathSegment new_line;
    new_line.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;

    if (path_len + extend_distance < apa_param.GetParam().min_path_length) {
      extend_distance =
          apa_param.GetParam().min_path_length - path_seg.Getlength();
    }
    if (output_.is_last_path &&
        path_len < apa_param.GetParam().min_path_length) {
      // incase control stops too early
      extend_distance +=
          2.0 * apa_param.GetParam().min_path_length - path_seg.Getlength();
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
        std::min(remain_car_dist, remain_obstacle_dist - kColBufferInSlot);
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

const bool ParallelPathPlanner::InsertTwoLinePathBetweenPathSeg(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& pose, const uint8_t gear,
    const double extend_distance) {
  path_seg_vec.clear();
  path_seg_vec.reserve(2);

  if (!pnc::geometry_lib::IsValidGear(gear)) {
    return false;
  }

  const Eigen::Vector2d v_heading(std::cos(pose.heading),
                                  std::sin(pose.heading));
  const Eigen::Vector2d line_end = pose.pos + v_heading * extend_distance;

  pnc::geometry_lib::LineSegment line(pose.pos, line_end, pose.heading);
  pnc::geometry_lib::PathSegment line_path(gear, line);
  path_seg_vec.emplace_back(line_path);

  if (!pnc::geometry_lib::ReversePathSegInfo(line_path)) {
    return false;
  }
  path_seg_vec.emplace_back(line_path);
  return true;
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
  output_.path_point_vec.reserve(PLANNING_TRAJ_POINTS_NUM);

  double first_path_length = 0.0;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; i++) {
    first_path_length += output_.path_segment_vec[i].Getlength();
  }

  const size_t max_points_size =
      PLANNING_TRAJ_POINTS_NUM - APA_COMPARE_PLANNING_TRAJ_POINTS_NUM;
  const double min_ds =
      first_path_length / static_cast<double>(max_points_size);

  const double actual_sample_ds = std::max(input_.sample_ds, min_ds);

  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
    const auto& current_seg = output_.path_segment_vec[i];

    if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      SampleLineSegment(current_seg.line_seg, actual_sample_ds);
    } else {
      SampleArcSegment(current_seg.arc_seg, actual_sample_ds);
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

  if (!pnc::mathlib::IsDoubleEqual(path_point.pos.x(),
                                   output_.path_point_vec.back().pos.x()) ||
      !pnc::mathlib::IsDoubleEqual(path_point.pos.y(),
                                   output_.path_point_vec.back().pos.y())) {
    output_.path_point_vec.emplace_back(path_point);
  }
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

const bool ParallelPathPlanner::IsOnTarget(
    const pnc::geometry_lib::PathPoint& current_pose) const {
  const auto& target_pose = calc_params_.target_pose;

  const double lon_err_mag =
      std::fabs(current_pose.pos.x() - target_pose.pos.x());

  const double lat_err_mag =
      std::fabs(current_pose.pos.y() - target_pose.pos.y());

  const double heading_err_mag = std::fabs(pnc::geometry_lib::NormalizeAngle(
      current_pose.heading - target_pose.heading));

  return (lon_err_mag <= apa_param.GetParam().finish_parallel_lon_err &&
          lat_err_mag <= apa_param.GetParam().finish_parallel_lat_err &&
          heading_err_mag <=
              apa_param.GetParam().finish_parallel_heading_err / 57.3);
}

const bool ParallelPathPlanner::CheckLonToTarget(
    const pnc::geometry_lib::PathPoint& current_pose) const {
  const Eigen::Vector2d v_heading =
      pnc::geometry_lib::GetUnitTangVecByHeading(current_pose.heading);

  const Eigen::Vector2d rear_bumper_center =
      current_pose.pos - apa_param.GetParam().rear_overhanging * v_heading;

  const Eigen::Vector2d front_bumper_center =
      current_pose.pos + v_heading * (apa_param.GetParam().wheel_base +
                                      apa_param.GetParam().front_overhanging);

  const bool lon_condition =
      rear_bumper_center.x() >= apa_param.GetParam().finish_parallel_lon_err &&
      front_bumper_center.x() <=
          input_.tlane.slot_length -
              apa_param.GetParam().finish_parallel_lon_err;
  return lon_condition;
}

const bool ParallelPathPlanner::IsOnTargetLine(
    const pnc::geometry_lib::PathPoint& current_pose) {
  const auto& target_pose = calc_params_.target_pose;

  const bool heading_condition =
      (std::fabs(pnc::geometry_lib::NormalizeAngle(current_pose.heading -
                                                   target_pose.heading)) <=
       apa_param.GetParam().finish_parallel_heading_err / 57.3);

  const double lat_condition_1 =
      std::fabs(current_pose.pos.y() - target_pose.pos.y()) <=
      apa_param.GetParam().finish_parallel_lat_rac_err;
  // lat condition 2, keep both outer wheels in slot

  const double side_sgn = calc_params_.slot_side_sgn;
  pnc::geometry_lib::LocalToGlobalTf ego2slot;
  ego2slot.Init(current_pose.pos, current_pose.heading);

  const auto& front_out_wheel = ego2slot.GetPos(
      Eigen::Vector2d(apa_param.GetParam().wheel_base,
                      0.5 * apa_param.GetParam().car_width * side_sgn));

  const auto& rear_out_wheel = ego2slot.GetPos(
      Eigen::Vector2d(0.0, 0.5 * apa_param.GetParam().car_width * side_sgn));

  const double slot_outer_pt_y = 0.5 * input_.tlane.slot_width * side_sgn;
  bool lat_condition_2 = false;
  if (side_sgn > 0.0) {
    const double wheel_limit_y =
        slot_outer_pt_y - apa_param.GetParam().finish_parallel_lat_err;
    lat_condition_2 = (rear_out_wheel.y() <= wheel_limit_y) &&
                      (front_out_wheel.y() <= wheel_limit_y);
  } else {
    const double wheel_limit_y =
        slot_outer_pt_y + apa_param.GetParam().finish_parallel_lat_err;
    lat_condition_2 = (rear_out_wheel.y() >= wheel_limit_y) &&
                      (front_out_wheel.y() >= wheel_limit_y);
  }
  const bool lat_condition = lat_condition_1 || lat_condition_2;

  return (lat_condition && heading_condition);
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