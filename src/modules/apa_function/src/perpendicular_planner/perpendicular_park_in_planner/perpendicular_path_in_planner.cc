#include "perpendicular_path_in_planner.h"

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
#include <limits>
#include <utility>
#include <vector>

#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_world.h"
#include "collision_detection.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "math_lib.h"
#include "planning_plan_c.h"
namespace planning {
namespace apa_planner {

static const size_t kMaxPerpenParkInSegmentNums = 15;
static const size_t kReservedOutputPathPointSize = 750;
static const int kMultiPlanMaxPathNumsInSlot = 5;
static const size_t kAdjustPlanMaxPathNumsInSlot = 5;
static const double kMinSingleGearPathLength = 0.35;
static const double kMinSinglePlanPathLength = 0.2;

const ApaParameters& g_params = apa_param.GetParam();

void PerpendicularPathInPlanner::Reset() {
  output_.Reset();
  output_.path_segment_vec.reserve(kMaxPerpenParkInSegmentNums);
  output_.gear_cmd_vec.reserve(kMaxPerpenParkInSegmentNums);

  calc_params_.Reset();
}

void PerpendicularPathInPlanner::Preprocess() {
  // calc_params_.Reset();
  calc_params_.complete_plan_again = false;
  calc_params_.single_plan_again = false;
  calc_params_.multi_plan = false;
  calc_params_.directly_use_ego_pose = false;
  calc_params_.turn_radius = 1.0 * apa_param.GetParam().min_turn_radius;
  calc_params_.can_insert_line = true;
  calc_params_.is_searching_stage = false;

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

const bool PerpendicularPathInPlanner::Update() {
  DEBUG_PRINT("--------perpendicular path planner --------");

  // preprocess
  Preprocess();

  // reset output
  output_.Reset();

  return UpdatePath();

  // prepare plan, only for first plan
  if (input_.is_replan_first) {
    calc_params_.first_multi_plan = true;
    if (PreparePlan()) {
      if (calc_params_.pre_plan_case == PrePlanCase::EGO_POSE) {
        DEBUG_PRINT("use ego pose to pre plan");
        if (calc_params_.directly_use_ego_pose) {
          DEBUG_PRINT(
              "ego pose is close to safe_circle_tang_pt, directly use"
              "ego pose to multi plan, no second prepare!");
          calc_params_.should_prepare_second = false;
          output_.gear_shift = true;
        } else if (output_.current_gear ==
                   pnc::geometry_lib::SEG_GEAR_REVERSE) {
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
      } else if (calc_params_.pre_plan_case == PrePlanCase::FIRST_MID_POINT) {
        DEBUG_PRINT("use first mid pt to pre plan");
        calc_params_.should_prepare_second = true;
        output_.gear_shift = false;
        if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
          DEBUG_PRINT(
              "first prepare success, the gear is reverse, and then "
              "should use mid pt to dubins to safe_circle_tang_pt");
        } else if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
          DEBUG_PRINT(
              "first prepare success, the gear is drive, and no need multi "
              "plan, need second prepare!");
        } else {
          DEBUG_PRINT("except err");
          return false;
        }
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
          output_.gear_shift = false;
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

const bool PerpendicularPathInPlanner::UpdatePath() {
  // prepare plan, only for first plan
  if (input_.is_replan_first) {
    calc_params_.first_multi_plan = true;
    if (PreparePathPlan()) {
      if (calc_params_.pre_plan_case == PrePlanCase::EGO_POSE) {
        if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
          DEBUG_PRINT(
              "first ego pose prepare success, the gear is drive, no need "
              "multi "
              "plan, need second prepare!");
          calc_params_.should_prepare_second = true;
          output_.gear_shift = false;
          return true;
        } else if (output_.current_gear ==
                   pnc::geometry_lib::SEG_GEAR_REVERSE) {
          DEBUG_PRINT(
              "first ego pose prepare success, the gear is reverse, use "
              "safe_circle_tang_pt to multi plan, no second prepare!");
          calc_params_.should_prepare_second = false;
          output_.gear_shift = true;
        } else {
          DEBUG_PRINT("except err");
          return false;
        }
      } else if (calc_params_.pre_plan_case == PrePlanCase::MID_POINT) {
        DEBUG_PRINT(
            "first mid prepare success, the gear is reverse, no need multi "
            "plan, need second prepare!");
        calc_params_.should_prepare_second = true;
        output_.gear_shift = false;
        return true;
      } else {
        DEBUG_PRINT("except err");
        return false;
      }
    } else {
      DEBUG_PRINT("first prepare path plan fail, quit plan");
      return false;
    }
  }

  if (input_.is_replan_second) {
    if (calc_params_.should_prepare_second) {
      DEBUG_PRINT("should try second prepare!");
      if (PreparePathPlanSecond()) {
        if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
          DEBUG_PRINT("second prepare gear is reverse, need multi plan!");
          output_.gear_shift = true;
        } else if (output_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
          DEBUG_PRINT("second prepare gear is drive, no need multi plan!");
          output_.gear_shift = false;
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

const bool PerpendicularPathInPlanner::UpdateByPrePlan() {
  Preprocess();
  calc_params_.is_searching_stage = true;

  return PreparePathPlan();

  return PreparePlan() || calc_params_.directly_use_ego_pose;
}

// prepare plan start
const bool PerpendicularPathInPlanner::PreparePlan() {
  std::cout << "enter prepare plan\n";
  // using namespace only to reduce unvalid code
  using namespace pnc::geometry_lib;
  const ApaParameters& params = apa_param.GetParam();
  const double pre_start_time = IflyTime::Now_ms();
  const double pt_01_x = ((input_.pt_0 + input_.pt_1) * 0.5).x();
  const Eigen::Vector2d vec_01 = (input_.pt_1 - input_.pt_0).normalized();
  const Eigen::Vector2d vec_01_norm_up(vec_01.y(), -vec_01.x());

  const Eigen::Vector2d ego_heading_vec =
      GenHeadingVec(input_.ego_pose.heading);

  const Eigen::Vector2d rear_axle_to_mirror_vec =
      params.lon_dist_mirror_to_rear_axle * ego_heading_vec;

  const Eigen::Vector2d mirror_pos =
      input_.ego_pose.pos + rear_axle_to_mirror_vec;

  LineSegment ego_line(input_.ego_pose.pos, mirror_pos,
                       input_.ego_pose.heading);

  // prepare plan set bigger lat_inflation to lat 1R more safe
  CollisionDetector::Paramters param;
  param.lat_inflation = apa_param.GetParam().car_lat_inflation_strict + 0.068;
  collision_detector_ptr_->SetParam(param);

  // if the safe circle tang pt is close to the ego pose, use ego to multi plan
  const double direct_use_ego_dist_err =
      apa_param.GetParam().prepare_directly_use_tangent_pos_err;

  const double direct_use_ego_heading_err =
      apa_param.GetParam().prepare_directly_use_tangent_heading_err * kDeg2Rad;

  // 1 indicate left side, -1 indicate right side
  const double slot_side_sgn = calc_params_.slot_side_sgn;

  Tlane& tlane = input_.tlane;
  const Eigen::Vector2d init_pt_inside = tlane.pt_inside;
  const double sample_dy = 0.25;
  double max_dy = 1.25;
  const double y_limit = init_pt_inside.y() - slot_side_sgn * sample_dy * 1.0;
  const double idle_x_offset = 1.668;
  const double inside_x =
      std::max(tlane.corner_inside_slot.x(), tlane.pt_inside.x());

  const std::vector<double> dy_tab = {0.0, 0.25, 0.5};
  const std::vector<double> max_heading_tab = {
      apa_param.GetParam().prepare_line_max_heading_offset_slot_deg, 42.8,
      53.8};
  const std::vector<double> min_heading_tab = {
      apa_param.GetParam().prepare_line_min_heading_offset_slot_deg, 24.8,
      35.8};
  const double dheading = 3.08;
  const double max_heading = 90.0 - input_.origin_pt_0_heading;

  const std::vector<double> max_x_tab = {3.0, 3.0, 3.01};
  const std::vector<double> min_x_tab = {2.0, 2.2, 2.3};
  const double dx = 0.35;

  std::vector<double> x_offset_vec;
  std::vector<double> heading_offset_vec;

  // set some flag
  const bool use_dubins = true;
  bool plan_success = false;
  bool prepare_success = false;
  calc_params_.directly_use_ego_pose = false;

  const double max_allow_time =
      apa_param.GetParam().prepare_single_max_allow_time;

  Eigen::Vector2d P;

  const int try_count = apa_param.GetParam().prepare_max_try_count;
  int i = 0;
  while (!plan_success && i < try_count) {
    switch (i) {
      case 0:
        max_dy = 0.168;
        P = input_.ego_pose.pos;
        break;
      case 1:
        max_dy = 0.568;
        P = calc_params_.is_left_side ? input_.pt_0 : input_.pt_1;
        P.y() -= 0.168 * slot_side_sgn;
        // if (mirror_pos.y() * slot_side_sgn < P.y() * slot_side_sgn && false)
        // {
        //   DEBUG_PRINT("ego mirror is before " << i
        //                                       << " th P, no consider and
        //                                       try");
        //   i++;
        //   continue;
        // }
        break;
      default:
        break;
    }

    // make meet min length require
    if (mirror_pos.y() * slot_side_sgn > P.y() * slot_side_sgn &&
        std::fabs(mirror_pos.y() - P.y()) < kMinSingleGearPathLength) {
      P.y() =
          mirror_pos.y() - slot_side_sgn * (kMinSingleGearPathLength + 0.168);
    }

    // ego pose to mid point and then to safe circle tange pt
    if (i > 0) {
      pnc::geometry_lib::PathPoint mid_point;

      // first try dubins from ego to mid point
      bool dubins_success = false;
      if (!dubins_success && use_dubins) {
        mid_point.heading = calc_params_.slot_side_sgn *
                            (90.0 - input_.origin_pt_0_heading) * kDeg2Rad;
        const Eigen::Vector2d B = P + idle_x_offset * vec_01_norm_up;
        mid_point.pos = B - calc_params_.slot_side_sgn *
                                params.lon_dist_mirror_to_rear_axle * vec_01;

        std::cout << "mid_point pos = " << mid_point.pos.transpose()
                  << "  mid_point heading = " << mid_point.heading * kRad2Deg
                  << "\n";
        std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
        if (DubinsPlan(input_.ego_pose, mid_point, calc_params_.turn_radius,
                       kMinSingleGearPathLength, true, path_seg_vec)) {
          std::cout << "dubins can move to " << i << " mid_point\n";
          dubins_success = true;
          GenPathOutputByDubins();
        } else {
          std::cout << "dubins can not move to " << i
                    << "  mid_point, then try line move to mid point\n";
        }
      }

      // then try line from ego to mid point
      if (!dubins_success) {
        // cal first mid point again, the car will move to it along line
        LineSegment mirror_line(
            P, P + 1.68 * vec_01_norm_up,
            -calc_params_.slot_side_sgn * input_.origin_pt_0_heading);

        Eigen::Vector2d target_mirror_pos;
        if (!GetIntersectionFromTwoLine(target_mirror_pos, ego_line,
                                        mirror_line)) {
          DEBUG_PRINT("line can not move to mid_point");
          i++;
          continue;
        }

        mid_point.pos = target_mirror_pos - rear_axle_to_mirror_vec;
        mid_point.heading = input_.ego_pose.heading;

        DEBUG_PRINT("mid_point pos = " << mid_point.pos.transpose()
                                       << "  mid_point heading = "
                                       << mid_point.heading * kRad2Deg);

        LineSegment move_line(input_.ego_pose.pos, mid_point.pos,
                              input_.ego_pose.heading);

        const uint8_t gear = CalLineSegGear(move_line);

        if (!IsValidGear(gear)) {
          DEBUG_PRINT("line gear is not valid from " << i << " mid_point");
          i++;
          continue;
        }

        PathSegment line_seg(gear, move_line);
        if (TrimPathByCollisionDetection(line_seg) == PathColDetRes::INVALID) {
          DEBUG_PRINT("line col det in not valid for  " << i << " mid_point");
          i++;
          continue;
        }

        DEBUG_PRINT("use line from ego pose to  " << i << " mid_point");

        output_.path_available = true;
        output_.path_segment_vec.emplace_back(line_seg);
        output_.steer_vec.emplace_back(SEG_STEER_STRAIGHT);
        output_.gear_cmd_vec.emplace_back(gear);
        output_.length += line_seg.Getlength();
        output_.current_gear = gear;
      }

      if (output_.path_segment_vec.empty()) {
        DEBUG_PRINT("from ego pose go to  " << i << " mid point fail");
        i++;
        continue;
      } else {
        DEBUG_PRINT("from ego go to  " << i
                                       << " mid point success, update ego");
        input_.ego_pose = output_.path_segment_vec.back().GetEndPose();
      }
    }

    double pre_once_time = IflyTime::Now_ms();
    for (double dy = 0.0; dy < max_dy + 0.01 && !plan_success;
         dy += sample_dy) {
      // change inside y
      tlane.pt_inside.y() =
          init_pt_inside.y() - calc_params_.slot_side_sgn * dy;
      // if y exceed limit, use inside x
      if (tlane.pt_inside.y() > y_limit + 0.01) {
        tlane.pt_inside.x() = inside_x;
      } else {
        tlane.pt_inside.x() = init_pt_inside.x();
      }
      heading_offset_vec.clear();
      for (double heading_offset =
               pnc::mathlib::Interp1(dy_tab, max_heading_tab, dy);
           heading_offset >= pnc::mathlib::Interp1(dy_tab, min_heading_tab, dy);
           heading_offset -= dheading) {
        heading_offset_vec.emplace_back(std::min(heading_offset, max_heading) *
                                        kDeg2Rad);
      }

      x_offset_vec.clear();
      for (double x_offset = pnc::mathlib::Interp1(dy_tab, min_x_tab, dy);
           x_offset <= pnc::mathlib::Interp1(dy_tab, max_x_tab, dy);
           x_offset += dx) {
        x_offset_vec.emplace_back(x_offset / input_.sin_angle + pt_01_x);
      }

      for (size_t j = 0; j < heading_offset_vec.size() && !plan_success; ++j) {
        for (size_t k = 0; k < x_offset_vec.size() && !plan_success; ++k) {
          if (IflyTime::Now_ms() - pre_once_time > max_allow_time) {
            j = 1000;
            k = 1000;
            DEBUG_PRINT(i << "th try consume "
                          << IflyTime::Now_ms() - pre_once_time
                          << " ms, try time is out, fail, try last dy");
            pre_once_time = IflyTime::Now_ms();
            continue;
          }
          if (PreparePlanOnce(x_offset_vec[k], heading_offset_vec[j],
                              calc_params_.turn_radius)) {
            prepare_success = true;
            DEBUG_PRINT("x_offset = "
                        << (x_offset_vec[k] - pt_01_x) * input_.sin_angle
                        << "  heading_offset = "
                        << heading_offset_vec[j] * kRad2Deg);
          } else if (i == 0 && calc_params_.cal_tang_pt_success &&
                     CheckTwoPoseIsSame(
                         input_.ego_pose, calc_params_.safe_circle_tang_pt,
                         direct_use_ego_dist_err, direct_use_ego_heading_err)) {
            calc_params_.directly_use_ego_pose = true;
            DEBUG_PRINT("directly use ego pose!");
          }
          plan_success = prepare_success || calc_params_.directly_use_ego_pose;
          if (plan_success) {
            DEBUG_PRINT("  tlane.pt_inside = " << tlane.pt_inside.transpose()
                                               << "  dy = " << dy);
          }
        }
      }
    }

    DEBUG_PRINT(i << "th try consume " << IflyTime::Now_ms() - pre_once_time
                  << " ms");

    // recover pt inside
    tlane.pt_inside = init_pt_inside;

    // check path length
    if (i > 0 && output_.length < kMinSingleGearPathLength &&
        (calc_params_.directly_use_ego_pose ||
         (prepare_success &&
          output_.current_gear !=
              dubins_planner_.GetOutput().current_gear_cmd))) {
      plan_success = false;
      calc_params_.directly_use_ego_pose = false;
      prepare_success = false;
    }

    if (!plan_success) {
      if (i == 0) {
        DEBUG_PRINT("try ego pose to safe point fail");
      } else {
        DEBUG_PRINT(
            "try " << i << " mid point to safe point fail, recover ego pose");
        input_.ego_pose = output_.path_segment_vec.front().GetStartPose();
        output_.Reset();
      }
    } else {
      switch (i) {
        case 0:
          calc_params_.pre_plan_case = PrePlanCase::EGO_POSE;
          DEBUG_PRINT("try ego pose to safe point success");
          break;
        case 1:
          calc_params_.pre_plan_case = PrePlanCase::FIRST_MID_POINT;
          DEBUG_PRINT("try first_mid_point to safe point success");
          break;
        default:
          break;
      }

      if (prepare_success && (dubins_planner_.GetOutput().current_gear_cmd ==
                                  output_.current_gear ||
                              i == 0)) {
        // dubins path transform to output path
        GenPathOutputByDubins();
        // update ego pose
        input_.ego_pose = output_.path_segment_vec.back().GetEndPose();
      } else if (calc_params_.directly_use_ego_pose) {
        // directly use ego pose to multi plan, no need to update ego pose
      }
    }

    i++;
  }

  param.Reset();
  collision_detector_ptr_->SetParam(param);

  calc_params_.first_path_gear = output_.current_gear;

  std::cout << "pre plan success = " << plan_success << "  prepare time cost "
            << IflyTime::Now_ms() - pre_start_time << " ms" << std::endl;

  std::cout << "leave prepare plan\n";

  JSON_DEBUG_VALUE("mono_plan", calc_params_.use_mono_tang)
  JSON_DEBUG_VALUE("multi_plan", calc_params_.use_multi_tang)

  return plan_success;
}

const bool PerpendicularPathInPlanner::PreparePlanOnce(
    const double x_offset, const double heading_offset, const double radius) {
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
  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
  while (!prepare_success) {
    // first use mono prepare to find target point
    if (apa_param.GetParam().actual_mono_plan_enable &&
        MonoPreparePlan(target_pose.pos)) {
      prepare_success =
          DubinsPlan(input_.ego_pose, target_pose, radius,
                     kMinSingleGearPathLength, true, path_seg_vec);
      calc_params_.cal_tang_pt_success = true;
      calc_params_.safe_circle_tang_pt = target_pose;
    }
    if (prepare_success) {
      calc_params_.use_mono_tang = true;
      break;
    }
    // if mono prepare fail, use multi prepare to find target point
    if (MultiPreparePlan(target_pose.pos)) {
      prepare_success =
          DubinsPlan(input_.ego_pose, target_pose, radius,
                     kMinSingleGearPathLength, true, path_seg_vec);
      calc_params_.cal_tang_pt_success = true;
      calc_params_.safe_circle_tang_pt = target_pose;
    }
    if (prepare_success) {
      calc_params_.use_multi_tang = true;
      break;
    }
    // force quit
    break;
  }

  if (prepare_success && (dubins_planner_.GetOutput().current_gear_cmd ==
                          pnc::geometry_lib::SEG_GEAR_DRIVE)) {
    // set virtual arc DE to do col det that make sure multi plan success
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

    CollisionDetector::CollisionResult col_res =
        collision_detector_ptr_->UpdateByObsMap(arc_DE, arc_DE.headingA);

    const double safe_remain_dist =
        std::min(col_res.remain_car_dist,
                 col_res.remain_obstacle_dist -
                     apa_param.GetParam().col_obs_safe_dist_normal);
    if ((col_res.remain_car_dist - 1e-3 > safe_remain_dist &&
         col_res.col_pt_ego_local.x() > 1.68) ||
        safe_remain_dist < 1e-5) {
      prepare_success = false;
    }
  }
  if (prepare_success) {
    std::cout << "prepare_success!!! use_mono_tang = "
              << calc_params_.use_mono_tang
              << "  use_multi_tang = " << calc_params_.use_multi_tang << "\n";
  }
  return prepare_success;
}

const bool PerpendicularPathInPlanner::CalTurnAroundPose() {
  // using namespace only to reduce unvalid code
  using namespace pnc::geometry_lib;
  const double slot_side_sgn = calc_params_.slot_side_sgn;
  bool success = false;
  int i = 0;
  const pnc::geometry_lib::PathPoint init_pose = input_.ego_pose;
  pnc::geometry_lib::PathPoint target_pose = input_.ego_pose;
  const int try_count = 20;
  const Eigen::Vector2d ego_heading_vec =
      GenHeadingVec(input_.ego_pose.heading);

  CollisionDetector::CollisionResult last_col_res;
  double length = 0.0;
  const double min_lat_err = -0.5;
  const double max_lat_err = 0.5;
  const double lat_err = (calc_params_.is_left_side) ? 0.5 : -0.5;
  while (!success && i < try_count) {
    i++;
    Arc arc;
    arc.pA = target_pose.pos;
    arc.headingA = target_pose.heading;
    arc.headingB = slot_side_sgn * 6.68 * kDeg2Rad;
    arc.circle_info.radius = calc_params_.turn_radius;
    const uint8_t arc_steer = SEG_STEER_LEFT;
    CompleteArcInfo(arc, arc_steer);

    PathSegment arc_seg;
    arc_seg.seg_type = SEG_TYPE_ARC;
    arc_seg.arc_seg = arc;
    arc_seg.seg_gear = SEG_GEAR_DRIVE;
    arc_seg.seg_steer = arc_steer;

    CollisionDetector::CollisionResult col_res;
    PathColDetRes col_det_res = TrimPathByCollisionDetection(arc_seg, &col_res);
    std::cout << "col_pt_ego_local = " << col_res.col_pt_ego_local.transpose()
              << "  last_col_pt_ego_local = "
              << last_col_res.col_pt_ego_local.transpose() << std::endl;

    std::cout << "init_pose = " << init_pose.pos.transpose()
              << "  target_pose = " << target_pose.pos.transpose()
              << "  length = " << length << std::endl;

    if (col_det_res == PathColDetRes::NORMAL) {
      std::cout << "this arc is safe, no collision\n";
      if (pnc::mathlib::IsInBound(arc_seg.GetEndPos().y(), min_lat_err,
                                  max_lat_err)) {
        std::cout << "this arc end pt y lat err is smaller than lat err, meet "
                     "needs, quit\n";
        output_.path_available = true;
        output_.path_segment_vec.emplace_back(arc_seg);
        output_.steer_vec.emplace_back(SEG_STEER_STRAIGHT);
        output_.gear_cmd_vec.emplace_back(arc_steer);
        output_.length += arc_seg.Getlength();
        output_.current_gear = arc_seg.seg_gear;
        success = true;
        break;
      }
      output_.Reset();
      int side = 1;
      uint8_t gear = SEG_GEAR_DRIVE;
      const double y = arc_seg.GetEndPos().y();
      if (slot_side_sgn * y > slot_side_sgn * lat_err) {
        std::cout
            << "this arc end pt y lat err is bigger than lat err, not meet "
               "needs, use line backward and then continue use arc\n";
        side = -1;
        gear = SEG_GEAR_REVERSE;
      } else if (slot_side_sgn * y < slot_side_sgn * -1.0 * lat_err) {
        std::cout
            << "this arc end pt y lat err is smaller than lat err, not meet "
               "needs, use line backward and then continue use arc\n";
        side = 1;
        gear = SEG_GEAR_DRIVE;
      }
      length = side * 0.268 * i;
      target_pose.pos = init_pose.pos + length * ego_heading_vec;
      LineSegment line(init_pose.pos, target_pose.pos, init_pose.heading);
      PathSegment line_seg(gear, line);
      output_.path_available = true;
      output_.path_segment_vec.emplace_back(line_seg);
      output_.steer_vec.emplace_back(SEG_STEER_STRAIGHT);
      output_.gear_cmd_vec.emplace_back(gear);
      output_.length += line_seg.Getlength();
      output_.current_gear = gear;
    }

    if (col_det_res == PathColDetRes::SHORTEN) {
      if (i > 1) {
        if (col_res.col_pt_ego_local.y() * last_col_res.col_pt_ego_local.y() <
            0.0) {
          std::cout << "this col res and last col res is not same dir, break\n";
          output_.path_available = true;
          output_.path_segment_vec.emplace_back(arc_seg);
          output_.steer_vec.emplace_back(SEG_STEER_STRAIGHT);
          output_.gear_cmd_vec.emplace_back(arc_steer);
          output_.length += arc_seg.Getlength();
          output_.current_gear = arc_seg.seg_gear;
          success = true;
          break;
        }
      }
      output_.Reset();
      int side = 1;
      uint8_t gear = SEG_GEAR_DRIVE;
      if (slot_side_sgn * col_res.col_pt_ego_local.y() < 0.0) {
        std::cout << "this arc col channel width, use line forward and then "
                     "continue use arc\n";
        side = 1;
        gear = SEG_GEAR_DRIVE;
      } else {
        std::cout << "this arc col channel length, use line backward and then "
                     "continue use arc\n";
        side = -1;
        gear = SEG_GEAR_REVERSE;
      }
      length = side * 0.268 * i;
      target_pose.pos = init_pose.pos + length * ego_heading_vec;
      LineSegment line(init_pose.pos, target_pose.pos, init_pose.heading);
      PathSegment line_seg(gear, line);
      output_.path_available = true;
      output_.path_segment_vec.emplace_back(line_seg);
      output_.steer_vec.emplace_back(SEG_STEER_STRAIGHT);
      output_.gear_cmd_vec.emplace_back(gear);
      output_.length += line_seg.Getlength();
      output_.current_gear = gear;
    }

    last_col_res = col_res;
  }

  if (output_.path_segment_vec.size() == 2) {
    if (output_.gear_cmd_vec[0] != output_.gear_cmd_vec[1] &&
        output_.path_segment_vec[0].Getlength() < kMinSingleGearPathLength) {
      output_.length -= output_.path_segment_vec[0].Getlength();
      output_.path_segment_vec.erase(output_.path_segment_vec.begin());
      output_.gear_cmd_vec.erase(output_.gear_cmd_vec.begin());
      output_.steer_vec.erase(output_.steer_vec.begin());
    }
  }

  if (output_.path_segment_vec.size() == 2) {
    output_.current_gear = output_.gear_cmd_vec.front();
  }

  // del arc, and then continue use turn around to plan
  if (output_.path_segment_vec.size() == 1 &&
      output_.path_segment_vec[0].seg_type == SEG_TYPE_ARC) {
    output_.Reset();
  }
  if (output_.path_segment_vec.size() == 2 &&
      output_.path_segment_vec[1].seg_type == SEG_TYPE_ARC) {
    output_.length -= output_.path_segment_vec[1].Getlength();
    output_.path_segment_vec.pop_back();
    output_.gear_cmd_vec.pop_back();
    output_.steer_vec.pop_back();
  }

  return success;
}

const bool PerpendicularPathInPlanner::TurnAround() { return true; }

const bool PerpendicularPathInPlanner::DubinsPlan(
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

const bool PerpendicularPathInPlanner::PreparePathPlan() {
  std::cout << "enter prepare plan\n";

  if (collision_detector_ptr_ == nullptr) {
    std::cout << "collision_detector_ptr_ is nullptr\n";
    return false;
  }

  // first check ego pose if collision
  if (!calc_params_.is_searching_stage) {
    collision_detector_ptr_->SetParam(CollisionDetector::Paramters(
        g_params.car_lat_inflation_strict + 0.068));
    if (collision_detector_ptr_->IsObstacleInCar(input_.ego_pose)) {
      std::cout << "ego pose has obs, force quit PreparePathPlan, fail\n";
      return false;
    }
  }

  const double pre_start_time = IflyTime::Now_ms();
  const double slot_side_sgn = calc_params_.slot_side_sgn;
  bool find_mid_pt = false;
  std::vector<geometry_lib::GeometryPath> geometry_path_vec;
  if (!PrepareSinglePathPlan(input_.ego_pose, geometry_path_vec)) {
    output_.Reset();
    geometry_lib::PathPoint tar_pose;
    double x = std::max(input_.ego_pose.pos.x(), 6.68);
    x = std::min(x, 10.68);
    const std::vector<Eigen::Vector2d> pos_vec = {
        Eigen::Vector2d(x, -1.968 * slot_side_sgn),
        Eigen::Vector2d(x, -3.368 * slot_side_sgn),
        Eigen::Vector2d(x, -4.868 * slot_side_sgn)};

    const std::vector<double> head_vec = {
        (90.0 * slot_side_sgn - slot_side_sgn * 1.68) * kDeg2Rad,
        (90.0 * slot_side_sgn - slot_side_sgn * 2.68) * kDeg2Rad,
        (90.0 * slot_side_sgn - slot_side_sgn * 4.68) * kDeg2Rad,
        (90.0 * slot_side_sgn - slot_side_sgn * 6.68) * kDeg2Rad,
        (90.0 * slot_side_sgn - slot_side_sgn * 8.68) * kDeg2Rad};

    geometry_lib::GeometryPath geometry_path;
    for (size_t i = 0; i < head_vec.size() && !find_mid_pt; ++i) {
      const double head = head_vec[i];
      for (size_t j = 0; j < pos_vec.size() && !find_mid_pt; ++j) {
        const Eigen::Vector2d& pos = pos_vec[j];
        tar_pose.Set(pos, head);
        DubinsPlanResult result =
            DubinsPathPlan(input_.ego_pose, tar_pose, calc_params_.turn_radius,
                           kMinSingleGearPathLength, true, geometry_path);
        if (result == DubinsPlanResult::SUCCESS &&
            geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE &&
            PrepareSinglePathPlan(tar_pose, geometry_path_vec)) {
          for (const auto& path_seg : geometry_path.path_segment_vec) {
            output_.path_available = true;
            output_.path_segment_vec.emplace_back(path_seg);
            output_.steer_vec.emplace_back(path_seg.seg_steer);
            output_.gear_cmd_vec.emplace_back(path_seg.seg_gear);
            output_.length += path_seg.Getlength();
          }
          find_mid_pt = true;
        }
      }
    }
  }

  if (geometry_path_vec.size() > 0) {
    // if is in searching stage, directly quit
    if (calc_params_.is_searching_stage) {
      return true;
    }

    if (!find_mid_pt) {
      calc_params_.pre_plan_case = PrePlanCase::EGO_POSE;
    } else {
      calc_params_.pre_plan_case = PrePlanCase::MID_POINT;
    }

    // if is in parking stage, choose a better path
    // How to choose a better path, temporarily reverse, shorter and one step
    // path
    int better_index = -1;
    const double gear_cost = 10.0;
    const double length_cost = 1.0;
    const double two_step_cost = 30.0;
    const double one_step_col_cost = 51.0;
    double min_cost = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < geometry_path_vec.size(); ++i) {
      const auto& path_seg_vec = geometry_path_vec[i].path_segment_vec;
      double cost = 0.0;
      for (size_t j = 0; j < path_seg_vec.size(); ++j) {
        if (find_mid_pt &&
            path_seg_vec[0].seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
          cost = std::numeric_limits<double>::infinity();
          break;
        }

        if (j == 0 &&
            path_seg_vec[0].seg_gear == geometry_lib::SEG_GEAR_DRIVE) {
          cost += gear_cost;
        }
        cost += path_seg_vec[j].Getlength() * length_cost;

        if (j == path_seg_vec.size() - 1) {
          geometry_lib::PathPoint end_pose = path_seg_vec[j].GetEndPose();
          const uint8_t steer = (calc_params_.is_left_side)
                                    ? geometry_lib::SEG_STEER_LEFT
                                    : geometry_lib::SEG_STEER_RIGHT;

          const uint8_t gear = geometry_lib::SEG_GEAR_REVERSE;

          const Eigen::Vector2d current_tang_vec =
              geometry_lib::GenHeadingVec(end_pose.heading);

          Eigen::Vector2d current_norm_vec(current_tang_vec.y(),
                                           -current_tang_vec.x());
          if (current_norm_vec.x() > 0.0) {
            current_norm_vec *= -1.0;
          }

          const Eigen::Vector2d current_turn_center =
              end_pose.pos + current_norm_vec * calc_params_.turn_radius;

          const double dist = geometry_lib::CalPoint2LineDist(
              current_turn_center, calc_params_.target_line);

          if (dist > calc_params_.turn_radius - g_params.target_radius_err) {
            if (!g_params.actual_mono_plan_enable) {
              cost += one_step_col_cost;
              continue;
            }
            if (dist > calc_params_.turn_radius + g_params.target_radius_err) {
              double sin_theta = std::fabs(sin(end_pose.heading));
              sin_theta = std::max(sin_theta, 0.0001);
              const double length =
                  (dist - calc_params_.turn_radius) / sin_theta;
              end_pose.pos =
                  end_pose.pos -
                  length * geometry_lib::GenHeadingVec(end_pose.heading);
            }
            const double heading_diff = std::fabs(end_pose.heading);
            const double arc_length = heading_diff * calc_params_.turn_radius;
            geometry_lib::PathSegment arc_seg;
            geometry_lib::CalArcFromPt(gear, steer, arc_length,
                                       calc_params_.turn_radius, end_pose,
                                       arc_seg);
            if (!IsGeometryPathSafe(geometry_lib::GeometryPath(arc_seg),
                                    g_params.car_lat_inflation_strict,
                                    g_params.col_obs_safe_dist_strict)) {
              cost += one_step_col_cost;
            }
          } else {
            cost += two_step_cost;
          }
        }
      }
      if (cost < min_cost) {
        better_index = static_cast<int>(i);
        min_cost = cost;
      }
    }

    if (better_index == -1) {
      output_.Reset();

      return false;
    }

    if (!find_mid_pt) {
      for (const auto& path_seg :
           geometry_path_vec[better_index].path_segment_vec) {
        output_.path_available = true;
        output_.path_segment_vec.emplace_back(path_seg);
        output_.steer_vec.emplace_back(path_seg.seg_steer);
        output_.gear_cmd_vec.emplace_back(path_seg.seg_gear);
        output_.length += path_seg.Getlength();
      }
    }

    // save safe_circle_tang_pt
    calc_params_.safe_circle_tang_pt = geometry_path_vec[better_index].end_pose;

    std::cout << "prepare path plan consume time = "
              << IflyTime::Now_ms() - pre_start_time << "ms\n";

    if (output_.path_segment_vec.size() > 0) {
      output_.current_gear = output_.gear_cmd_vec.front();
      input_.ego_pose = output_.path_segment_vec.back().GetEndPose();
      calc_params_.first_path_gear = output_.current_gear;
      std::cout << "first prepare pos = " << input_.ego_pose.pos.transpose()
                << "  heading = " << input_.ego_pose.heading * kRad2Deg
                << "  path length = " << output_.length << std::endl;
      return true;
    }
  }

  output_.Reset();

  return false;
}

const bool PerpendicularPathInPlanner::PrepareSinglePathPlan(
    const pnc::geometry_lib::PathPoint& cur_pose,
    std::vector<geometry_lib::GeometryPath>& geometry_path_vec) {
  std::cout << "enter single prepare plan\n";
  const double pre_start_time = IflyTime::Now_ms();

  const double slot_side_sgn = calc_params_.slot_side_sgn;
  const double slot_angle = input_.origin_pt_0_heading;  // 0 30 45
  const double sin_slot_angle = input_.sin_angle;  // sin(90) sin(60) sin(45)
  const double slot_x = ((input_.pt_0 + input_.pt_1) * 0.5).x();

  std::vector<double> x_offset_vec;
  std::vector<double> heading_offset_vec;
  x_offset_vec.reserve(5);
  heading_offset_vec.reserve(40);

  double max_heading = std::min(
      90.0 - slot_angle, g_params.prepare_line_max_heading_offset_slot_deg);
  max_heading = slot_side_sgn * (90.0 - slot_angle - max_heading) * kDeg2Rad;

  double min_heading = g_params.prepare_line_min_heading_offset_slot_deg;
  min_heading = slot_side_sgn * (90.0 - slot_angle - min_heading) * kDeg2Rad;

  const double dheading =
      g_params.prepare_line_dheading_offset_slot_deg * kDeg2Rad;

  double heading = max_heading;
  while (slot_side_sgn * heading < slot_side_sgn * min_heading) {
    heading_offset_vec.emplace_back(heading);
    heading += slot_side_sgn * dheading;
  }

  const double max_x =
      g_params.prepare_line_max_x_offset_slot / sin_slot_angle + slot_x;
  const double min_x =
      g_params.prepare_line_min_x_offset_slot / sin_slot_angle + slot_x;
  const double dx = g_params.prepare_line_dx_offset_slot / sin_slot_angle;
  double x = min_x;
  while (x < max_x) {
    x_offset_vec.emplace_back(x);
    x += dx;
  }

  std::vector<geometry_lib::LineSegment> prepare_line_vec;
  prepare_line_vec.reserve(heading_offset_vec.size() * x_offset_vec.size() + 1);
  for (const double heading : heading_offset_vec) {
    for (const double x : x_offset_vec) {
      prepare_line_vec.emplace_back(
          geometry_lib::BuildLineSegByPose(Eigen::Vector2d(x, 0.0), heading));
    }
  }
  prepare_line_vec.emplace_back(
      geometry_lib::BuildLineSegByPose(cur_pose.pos, cur_pose.heading));

  std::cout << "prepare_line_vec size = " << prepare_line_vec.size()
            << std::endl;

  const double virtual_1r_arc_length = 3.08;
  const uint8_t count = 7;
  const double ds = 0.5;
  const uint8_t gear = geometry_lib::SEG_GEAR_REVERSE;
  const uint8_t steer = (calc_params_.is_left_side)
                            ? geometry_lib::SEG_STEER_LEFT
                            : geometry_lib::SEG_STEER_RIGHT;

  std::vector<std::vector<std::vector<geometry_lib::PathPoint>>> tang_pose_vec;
  tang_pose_vec.clear();
  tang_pose_vec.reserve(prepare_line_vec.size() * count * count * 2);

  int number = 0;
  for (const geometry_lib::LineSegment& line : prepare_line_vec) {
    // cal pre line tangent vec and normal vec
    const Eigen::Vector2d line_tangent_vec =
        geometry_lib::GenHeadingVec(line.heading);

    Eigen::Vector2d line_normal_vec(line_tangent_vec.y(),
                                    -line_tangent_vec.x());

    // sure line_normal_vec towards downward along the x axis.
    if (line_normal_vec.x() > 0.0) {
      line_normal_vec = -1.0 * line_normal_vec;
    }

    calc_params_.prepare_line = line;

    calc_params_.pre_line_tangent_vec = line_tangent_vec;
    calc_params_.pre_line_normal_vec = line_normal_vec;

    bool cal_tang_pt_success = false;

    for (uint8_t i = 0; i < 2; ++i) {
      geometry_lib::PathPoint pose;

      pose.heading = line.heading;
      if (i == 0 && apa_param.GetParam().actual_mono_plan_enable &&
          MonoPreparePlan(pose.pos)) {
        cal_tang_pt_success = true;
      }
      if (i == 1 && MultiPreparePlan(pose.pos)) {
        cal_tang_pt_success = true;
      }

      geometry_lib::PathSegment arc_seg;
      if (cal_tang_pt_success) {
        geometry_lib::CalArcFromPt(gear, steer, virtual_1r_arc_length,
                                   calc_params_.turn_radius, pose, arc_seg);
      }

      std::vector<std::vector<geometry_lib::PathPoint>> inner_tang_pose_vec;
      inner_tang_pose_vec.clear();
      inner_tang_pose_vec.reserve(count + 1);

      for (uint8_t j = 0; j < count && cal_tang_pt_success; ++j) {
        std::vector<geometry_lib::PathPoint> inner_inner_tang_pose_vec;
        inner_inner_tang_pose_vec.clear();
        inner_inner_tang_pose_vec.reserve(count + 1);
        geometry_lib::CalPtFromPathSeg(pose, arc_seg, j * ds);
        geometry_lib::PathPoint temp_pose = pose;
        const Eigen::Vector2d heading_vec =
            geometry_lib::GenHeadingVec(pose.heading);
        for (uint8_t k = 0; k < count; ++k) {
          temp_pose.pos = pose.pos + ds * k * heading_vec;
          inner_inner_tang_pose_vec.emplace_back(temp_pose);
          number++;
        }
        inner_tang_pose_vec.emplace_back(inner_inner_tang_pose_vec);
      }

      if (cal_tang_pt_success) {
        tang_pose_vec.emplace_back(inner_tang_pose_vec);
      }
    }
  }

  std::cout << "link point size = " << number << std::endl;
  std::cout << "cal tang pose consume time = "
            << IflyTime::Now_ms() - pre_start_time << "ms\n";

  bool exceed_time_flag = false;
  double max_allow_time = g_params.prepare_single_max_allow_time;
  if (!calc_params_.is_searching_stage) {
    max_allow_time = 999.9;
  }
  geometry_path_vec.clear();
  geometry_path_vec.reserve(number);
  geometry_lib::GeometryPath geometry_path;
  geometry_lib::PathSegment arc_seg;
  DubinsPlanResult result;
  bool find_all_result = true;
  for (size_t i = 0;
       i < tang_pose_vec.size() && find_all_result && !exceed_time_flag; ++i) {
    bool reverse_1arc_safe = false;
    const auto& inner_tang_pose_vec = tang_pose_vec[i];
    for (size_t j = 0;
         j < inner_tang_pose_vec.size() && find_all_result && !exceed_time_flag;
         ++j) {
      const auto& inner_inner_tang_pose_vec = inner_tang_pose_vec[j];
      if (inner_inner_tang_pose_vec.size() < 1) {
        break;
      }

      // when the 1r corresponding to tangpt is safe, all subsequent tangpt
      // are safe
      if (!reverse_1arc_safe) {
        // construct 1r arc
        geometry_lib::CalArcFromPt(gear, steer, virtual_1r_arc_length - j * ds,
                                   calc_params_.turn_radius,
                                   inner_inner_tang_pose_vec[0], arc_seg);
        // check 1r is safe
        if (IsGeometryPathSafe(geometry_lib::GeometryPath(arc_seg),
                               g_params.car_lat_inflation_strict + 0.068,
                               g_params.col_obs_safe_dist_strict + 0.068)) {
          reverse_1arc_safe = true;
        } else {
          continue;
        }
      }

      for (size_t k = 0; k < inner_inner_tang_pose_vec.size() &&
                         find_all_result && !exceed_time_flag;
           ++k) {
        if (IflyTime::Now_ms() - pre_start_time > max_allow_time) {
          exceed_time_flag = true;
          DEBUG_PRINT("try time is out, quit find result.");
          break;
        }
        const auto& tang_pose = inner_inner_tang_pose_vec[k];
        result = DubinsPathPlan(cur_pose, tang_pose, calc_params_.turn_radius,
                                kMinSingleGearPathLength, true, geometry_path);

        if (result == DubinsPlanResult::SUCCESS) {
          geometry_path_vec.emplace_back(geometry_path);
          if (calc_params_.is_searching_stage) {
            find_all_result = false;
          }
          break;
        }

        if (result == DubinsPlanResult::PATH_COLLISION) {
          break;
        }

        if (result == DubinsPlanResult::NO_PATH) {
        }
      }
    }
  }

  std::cout << "there arc " << geometry_path_vec.size()
            << " tangpt can be target\n";

  std::cout << "prepare single path plan consume time = "
            << IflyTime::Now_ms() - pre_start_time << "ms\n";

  return geometry_path_vec.size() > 0;
}

const bool PerpendicularPathInPlanner::PreparePathPlanSecond() {
  using namespace pnc;

  double min_length = 0.0;
  uint8_t ref_gear = geometry_lib::SEG_GEAR_INVALID;

  if (calc_params_.pre_plan_case == PrePlanCase::EGO_POSE &&
      calc_params_.first_path_gear == geometry_lib::SEG_GEAR_DRIVE) {
    min_length = 0.0268;
    ref_gear = geometry_lib::SEG_GEAR_REVERSE;
  } else if (calc_params_.pre_plan_case == PrePlanCase::MID_POINT &&
             calc_params_.first_path_gear == geometry_lib::SEG_GEAR_REVERSE) {
    min_length = kMinSingleGearPathLength;
    ref_gear = geometry_lib::SEG_GEAR_DRIVE;
  } else {
    return false;
  }

  geometry_lib::GeometryPath geometry_path;
  DubinsPlanResult result = DubinsPathPlan(
      input_.ego_pose, calc_params_.safe_circle_tang_pt,
      calc_params_.turn_radius, min_length, false, geometry_path);

  if (result == DubinsPlanResult::SUCCESS && geometry_path.path_count > 0 &&
      geometry_path.cur_gear == ref_gear) {
    output_.path_available = true;
    for (const pnc::geometry_lib::PathSegment& path_seg :
         geometry_path.path_segment_vec) {
      output_.path_segment_vec.emplace_back(path_seg);
      output_.length += path_seg.Getlength();
      output_.gear_cmd_vec.emplace_back(path_seg.seg_gear);
      output_.steer_vec.emplace_back(path_seg.seg_steer);
    }
    output_.current_gear = ref_gear;
    input_.ego_pose = geometry_path.end_pose;
    std::cout << "second prepare, from first prepare pos to safe circle tange "
                 "success\n";
    return true;
  } else {
    std::cout
        << "second prepare, from first prepare pos to safe circle tange fail\n";
    return false;
  }
}

const PerpendicularPathInPlanner::DubinsPlanResult
PerpendicularPathInPlanner::DubinsPathPlan(
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const double turn_radius,
    const double min_length, const bool need_col_det,
    geometry_lib::GeometryPath& geometry_path) {
  dubins_lib::DubinsLibrary::Input input(start_pose.pos, target_pose.pos,
                                         start_pose.heading,
                                         target_pose.heading, turn_radius);

  dubins_planner_.SetInput(input);

  if (!dubins_planner_.OneStepDubinsUpdateByVer(min_length)) {
    return DubinsPlanResult::NO_PATH;
  }

  std::vector<geometry_lib::PathSegment> path_seg_vec;
  path_seg_vec.reserve(3);

  pnc::geometry_lib::PathSegment path_seg;
  // set arc AB
  if (dubins_planner_.GetOutput().gear_cmd_vec[0] !=
      pnc::geometry_lib::SEG_GEAR_INVALID) {
    path_seg.seg_gear = dubins_planner_.GetOutput().gear_cmd_vec[0];
    path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
    path_seg.arc_seg = dubins_planner_.GetOutput().arc_AB;
    path_seg.seg_steer = geometry_lib::CalArcSteer(path_seg.arc_seg);
    path_seg_vec.emplace_back(path_seg);
  }
  // set line BC
  if (dubins_planner_.GetOutput().gear_cmd_vec[1] !=
      pnc::geometry_lib::SEG_GEAR_INVALID) {
    path_seg.seg_gear = dubins_planner_.GetOutput().gear_cmd_vec[1];
    path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;
    path_seg.line_seg = dubins_planner_.GetOutput().line_BC;
    path_seg.seg_steer = geometry_lib::SEG_STEER_STRAIGHT;
    path_seg_vec.emplace_back(path_seg);
  }
  // set arc CD
  if (dubins_planner_.GetOutput().gear_cmd_vec[2] !=
      pnc::geometry_lib::SEG_GEAR_INVALID) {
    path_seg.seg_gear = dubins_planner_.GetOutput().gear_cmd_vec[2];
    path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
    path_seg.arc_seg = dubins_planner_.GetOutput().arc_CD;
    path_seg.seg_steer = geometry_lib::CalArcSteer(path_seg.arc_seg);
    path_seg_vec.emplace_back(path_seg);
  }

  geometry_path.SetPath(path_seg_vec);

  if (need_col_det &&
      !IsGeometryPathSafe(geometry_path,
                          g_params.car_lat_inflation_strict + 0.068,
                          g_params.col_obs_safe_dist_strict + 0.068)) {
    geometry_path.Reset();
    return DubinsPlanResult::PATH_COLLISION;
  }

  return DubinsPlanResult::SUCCESS;
}

const bool PerpendicularPathInPlanner::IsPathSafe(
    const pnc::geometry_lib::PathSegment& path_seg, const double lat_inflation,
    const double lon_safe_dist) {
  using namespace pnc;
  if (path_seg.seg_type != geometry_lib::SEG_TYPE_LINE &&
      path_seg.seg_type != geometry_lib::SEG_TYPE_ARC) {
    return false;
  }

  const double origin_lat_inflation =
      collision_detector_ptr_->GetParam().lat_inflation;
  bool update_lat_inflation = false;
  CollisionDetector::Paramters params;
  if (!mathlib::IsDoubleEqual(origin_lat_inflation, lat_inflation)) {
    update_lat_inflation = true;
    params.lat_inflation = lat_inflation;
    collision_detector_ptr_->SetParam(params);
  }

  bool path_safe = true;
  if (true && collision_detector_ptr_->IsObstacleInPath(path_seg, 0.0, false)) {
    path_safe = false;
  } else {
    CollisionDetector::CollisionResult col_res;

    if (path_seg.seg_type == geometry_lib::SEG_TYPE_LINE) {
      const geometry_lib::LineSegment& line = path_seg.line_seg;
      col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
    } else {
      const geometry_lib::Arc& arc = path_seg.arc_seg;
      col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
    }

    if (col_res.remain_car_dist + lon_safe_dist >
        col_res.remain_obstacle_dist) {
      path_safe = false;
    }
  }

  if (update_lat_inflation) {
    // revert params
    params.lat_inflation = origin_lat_inflation;
    collision_detector_ptr_->SetParam(params);
  }

  return path_safe;
}

const bool PerpendicularPathInPlanner::IsGeometryPathSafe(
    const geometry_lib::GeometryPath& geometry_path, const double lat_inflation,
    const double lon_safe_dist) {
  return !collision_detector_ptr_
              ->UpdateByEDT(geometry_path, lat_inflation, lon_safe_dist)
              .collision_flag;
}

const PerpendicularPathInPlanner::PathColDetRes
PerpendicularPathInPlanner::TrimPathByObs(
    pnc::geometry_lib::PathSegment& path_seg, const double lat_inflation,
    const double lon_safe_dist) {
  using namespace pnc;
  if (path_seg.seg_type != geometry_lib::SEG_TYPE_LINE &&
      path_seg.seg_type != geometry_lib::SEG_TYPE_ARC) {
    return PathColDetRes::INVALID;
  }

  const double origin_lat_inflation =
      collision_detector_ptr_->GetParam().lat_inflation;
  bool update_lat_inflation = false;
  CollisionDetector::Paramters params;
  if (!mathlib::IsDoubleEqual(origin_lat_inflation, lat_inflation)) {
    update_lat_inflation = true;
    params.lat_inflation = lat_inflation;
    collision_detector_ptr_->SetParam(params);
  }

  CollisionDetector::CollisionResult col_res;

  if (path_seg.seg_type == geometry_lib::SEG_TYPE_LINE) {
    const geometry_lib::LineSegment& line = path_seg.line_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
  } else {
    const geometry_lib::Arc& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
  }

  if (col_res.remain_car_dist + lon_safe_dist > col_res.remain_obstacle_dist) {
  }

  if (update_lat_inflation) {
    // revert params
    params.lat_inflation = origin_lat_inflation;
    collision_detector_ptr_->SetParam(params);
  }

  return PathColDetRes::INVALID;
}

const bool PerpendicularPathInPlanner::PreparePlanSecond() {
  const double dist =
      (input_.ego_pose.pos - calc_params_.safe_circle_tang_pt.pos).norm();
  const double heading_err = std::fabs(
      input_.ego_pose.heading - calc_params_.safe_circle_tang_pt.heading);

  if (dist < apa_param.GetParam().prepare_directly_use_tangent_pos_err &&
      heading_err <
          apa_param.GetParam().prepare_directly_use_tangent_heading_err *
              kDeg2Rad) {
    DEBUG_PRINT("use ego pose to multi plan when prepare second");
    return false;
  }

  double min_dubins_path = kMinSingleGearPathLength;
  if (calc_params_.first_path_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    // this case current dubins gear must be reverse, and the path is continue
    // to multi or adjust plan so no length need for it
    min_dubins_path = 0.086;
  }

  // try dubins no col det, use dynamic col det to avoid col
  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
  const bool dubins_success =
      DubinsPlan(input_.ego_pose, calc_params_.safe_circle_tang_pt,
                 calc_params_.turn_radius, min_dubins_path, false,
                 path_seg_vec) &&
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
  return dubins_success;
}

void PerpendicularPathInPlanner::CalMonoSafeCircle() {
  calc_params_.mono_safe_circle.center.y() =
      calc_params_.target_line.pA.y() +
      calc_params_.turn_radius * calc_params_.slot_side_sgn;

  calc_params_.mono_safe_circle.radius = calc_params_.turn_radius;

  Eigen::Vector2d pt_inside = input_.tlane.pt_inside;

  pt_inside.x() = std::max(pt_inside.x(),
                           input_.tlane.corner_inside_slot.x() -
                               apa_param.GetParam().max_pt_inside_drop_dx_mono);

  const double deta_x = std::sqrt(
      std::pow(
          (calc_params_.turn_radius - apa_param.GetParam().car_width * 0.5 -
           apa_param.GetParam().car_lat_inflation_strict),
          2) -
      std::pow((calc_params_.mono_safe_circle.center.y() - pt_inside.y()), 2));

  calc_params_.mono_safe_circle.center.x() = pt_inside.x() - deta_x;

  // DEBUG_PRINT("mono safe circle info: center = "
  //           << calc_params_.mono_safe_circle.center.transpose()
  //           << "   radius = " << calc_params_.mono_safe_circle.radius
  //          );
}

const bool PerpendicularPathInPlanner::CheckMonoIsFeasible() {
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

const bool PerpendicularPathInPlanner::MonoPreparePlan(
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

bool PerpendicularPathInPlanner::CalMultiSafeCircle() {
  auto pt_inside = input_.tlane.pt_inside;
  pt_inside.x() = std::max(
      pt_inside.x(), input_.tlane.corner_inside_slot.x() -
                         apa_param.GetParam().max_pt_inside_drop_dx_multi);

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

  // DEBUG_PRINT("multi safa circle info: center = " <<
  // multi_safe_circle.center
  //           << "  radius = " << multi_safe_circle.radius);

  return true;
}

const bool PerpendicularPathInPlanner::MultiPreparePlan(
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

const bool PerpendicularPathInPlanner::GenPathOutputByDubins() {
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
  //                         << path_seg.line_seg.pB.transpose() << "  heading
  //                         =
  //                         "
  //                         << path_seg.line_seg.heading * kRad2Deg);
  //   } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
  //     DEBUG_PRINT("pA = " << path_seg.arc_seg.pA.transpose()
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
// prepare plan end

const bool PerpendicularPathInPlanner::OneArcPathPlan(
    const pnc::geometry_lib::PathPoint& pose,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  DEBUG_PRINT("try one arc path plan");
  using namespace pnc;
  if (mathlib::IsDoubleEqual(pose.heading, calc_params_.target_line.heading) ||
      mathlib::IsDoubleEqual(pose.pos.y(), calc_params_.target_line.pA.y())) {
    DEBUG_PRINT(
        "current heading is equal to target heading or current y is "
        "equal to "
        "target y, no need to one arc plan");
    return false;
  }

  const auto& params = apa_param.GetParam();
  const std::vector<uint8_t> gear_vec = {geometry_lib::SEG_GEAR_DRIVE,
                                         geometry_lib::SEG_GEAR_REVERSE};
  geometry_lib::Arc arc;
  arc.pA = pose.pos;
  arc.headingA = pose.heading;
  for (const uint8_t temp_gear : gear_vec) {
    bool success = pnc::geometry_lib::CalOneArcWithLineAndGear(
        arc, calc_params_.target_line, temp_gear);

    success = success &&
              (arc.length < arc.circle_info.radius * (178.68) * kDeg2Rad) &&
              (arc.length > 0.0168);

    if (!success) {
      continue;
    }

    const uint8_t steer = geometry_lib::CalArcSteer(arc);
    const uint8_t gear = geometry_lib::CalArcGear(arc);

    const double radius = arc.circle_info.radius;
    success = (gear == temp_gear) &&
              (radius > calc_params_.turn_radius - params.target_radius_err &&
               radius < params.max_one_step_arc_radius);

    if (!success) {
      continue;
    }

    geometry_lib::PathSegment arc_seg(steer, gear, arc);

    // do col det
    if (IsPathSafe(arc_seg, params.car_lat_inflation_normal,
                   params.col_obs_safe_dist_normal)) {
      path_seg_vec.emplace_back(arc_seg);
    }
  }

  return false;
}

const bool PerpendicularPathInPlanner::TwoArcPathPlan(
    const pnc::geometry_lib::PathPoint& pose,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  DEBUG_PRINT("try two reverse arc path plan");
  using namespace pnc;

  // const auto& params = apa_param.GetParam();
  const std::vector<uint8_t> gear_vec = {geometry_lib::SEG_GEAR_DRIVE,
                                         geometry_lib::SEG_GEAR_REVERSE};
  const std::vector<uint8_t> steer_vec = {geometry_lib::SEG_STEER_LEFT,
                                          geometry_lib::SEG_STEER_RIGHT};
  geometry_lib::Arc arc1;
  geometry_lib::Arc arc2;
  arc1.pA = pose.pos;
  arc1.headingA = pose.heading;
  arc1.circle_info.radius = calc_params_.turn_radius;
  const Eigen::Vector2d tang_vec = geometry_lib::GenHeadingVec(pose.heading);
  Eigen::Vector2d norm_vec;
  for (const uint8_t temp_steer : steer_vec) {
    if (temp_steer == geometry_lib::SEG_STEER_RIGHT) {
      norm_vec << tang_vec.y(), -tang_vec.x();
    } else {
      norm_vec << -tang_vec.y(), tang_vec.x();
    }
    arc1.circle_info.center = pose.pos + calc_params_.turn_radius * norm_vec;

    for (const uint8_t temp_gear : gear_vec) {
      bool success = CalTwoArcWithLine(arc1, arc2, calc_params_.target_line);

      success = success &&
                (arc1.length < arc1.circle_info.radius * (178.68) * kDeg2Rad) &&
                (arc1.length > 0.0168) &&
                (arc2.length < arc2.circle_info.radius * (178.68) * kDeg2Rad) &&
                (arc2.length > 0.0168);

      if (!success) {
        continue;
      }

      const uint8_t steer1 = geometry_lib::CalArcSteer(arc1);
      const uint8_t gear1 = geometry_lib::CalArcGear(arc1);
      const uint8_t steer2 = geometry_lib::CalArcSteer(arc2);
      const uint8_t gear2 = geometry_lib::CalArcGear(arc2);

      success = (steer1 == temp_steer && gear1 == temp_gear) &&
                (steer2 == geometry_lib::ReverseSteer(temp_steer) &&
                 gear2 == geometry_lib::ReverseGear(temp_gear));

      if (!success) {
        continue;
      }

      geometry_lib::PathSegment arc_seg1(steer1, gear1, arc1);
      geometry_lib::PathSegment arc_seg2(steer2, gear2, arc2);
    }
  }
  return false;
}

// checkout multi suitable
const bool PerpendicularPathInPlanner::CheckMultiPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose,
    const double& slot_occupied_ratio) {
  if ((std::fabs(current_pose.pos.y()) <=
           apa_param.GetParam().multi_plan_min_lat_err &&
       std::fabs(current_pose.heading) <=
           apa_param.GetParam().multi_plan_min_heading_err * kDeg2Rad) ||
      slot_occupied_ratio >=
          apa_param.GetParam().multi_plan_max_occupied_ratio) {
    DEBUG_PRINT("pose err is relatively small, multi plan is not suitable");
    return false;
  }
  return true;
}

// multi plan start
const bool PerpendicularPathInPlanner::MultiPlan() {
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
  size_t plan_again_count = 0;
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

      if (calc_params_.single_plan_again || calc_params_.complete_plan_again) {
        i = -1;
        plan_again_count++;
        DEBUG_PRINT(
            "reverse path stuck by inside or channel, need use reverse line "
            "and arc to far from inside or channel.");

        if (plan_again_count > 10) {
          std::cout << "try reverse line and arc enough, but also failed\n";
          success = false;
          break;
        }

        if (calc_params_.complete_plan_again) {
          DEBUG_PRINT(
              "clear all multi output path, and complete plan again from "
              "current ego pose.");
          multi_out_put.Reset();
        } else if (calc_params_.single_plan_again) {
          DEBUG_PRINT("single plan again from last plan end pose.");
        }

        double compensate_line_length = 0.268;
        if (multi_out_put.path_segment_vec.empty()) {
          current_pose = input_.ego_pose;
          current_gear = input_.ref_gear;
          current_arc_steer = input_.ref_arc_steer;
          compensate_line_length *= plan_again_count;
        } else {
          current_pose = multi_out_put.path_segment_vec.back().GetEndPose();
        }
        DEBUG_PRINT("compensate_line_length = " << compensate_line_length);

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

  // if multi path current gear is not correct, lost all path and use adjust
  // to plan if (multi_out_put.path_segment_vec.size() > 0 &&
  //     multi_out_put.gear_cmd_vec.front() != input_.ref_gear) {
  //   multi_out_put.Reset();
  //   success = false;
  // }

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
            "multi already plan to target pos, look it can lose drive path "
            "and "
            "continue to use reverse gear adjust plan to reduce change "
            "count!");
        output_.multi_reach_target_pose = true;
      } else {
        DEBUG_PRINT(
            "multi can not plan to target pos, look it can lose drive path "
            "and "
            "continue to use reverse gear adjust plan to reduce change "
            "count!");
        output_.multi_reach_target_pose = false;
      }
      // only to onvenient for simulation use
      bool continue_to_adjust = true;
      if (input_.is_simulation) {
        continue_to_adjust = !output_.multi_reach_target_pose;
      }
      if (!apa_param.GetParam().actual_mono_plan_enable &&
          calc_params_.first_multi_plan) {
        continue_to_adjust = false;
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

const bool PerpendicularPathInPlanner::CalSinglePathInMulti(
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const uint8_t current_arc_steer,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const double turn_radius, size_t i) {
  DEBUG_PRINT("-----CalSinglePathInMulti-----");
  DEBUG_PRINT("current_arc_steer = "
              << static_cast<int>(current_arc_steer)
              << ",  current_gear = " << static_cast<int>(current_gear)
              << ",  current_pos = " << current_pose.pos.transpose()
              << ",  current_heading = " << current_pose.heading * kRad2Deg);

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

  // second: try two arc, one arc or line arc plan to target line
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
    const double dist = pnc::geometry_lib::CalPoint2LineDist(
        current_turn_center, calc_params_.target_line);

    // two arc
    if (dist < current_turn_radius - apa_param.GetParam().target_radius_err) {
      DEBUG_PRINT("center to line dist = " << dist << ",  try TwoArcPlan");
      if (TwoArcPlan(current_arc, tmp_path_seg_vec, current_gear,
                     current_arc_steer)) {
        DEBUG_PRINT("TwoArcPlan success");
        play_type = PLAN_TYPE_TWO_ARC;
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
        play_type = PLAN_TYPE_ONE_ARC;
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
        play_type = PLAN_TYPE_LINE_ARC;
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
  if (play_type == PLAN_TYPE_LINE_ARC &&
      current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    double channel_width =
        collision_detector_ptr_->GetCarMaxX(input_.ego_pose) + 3.168 -
        std::max(input_.pt_0.x(), input_.pt_1.x());

    const double channel_para_width =
        (input_.slot_occupied_ratio < 0.368)
            ? apa_param.GetParam().channel_width
            : apa_param.GetParam().line_arc_obs_channel_width;

    channel_width = std::max(channel_width, channel_para_width);

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

  calc_params_.single_plan_again = false;
  calc_params_.complete_plan_again = false;

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

    DEBUG_PRINT("lat_lat_inflation = " << params.lat_inflation);

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

const bool PerpendicularPathInPlanner::OneArcPlan(
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

const bool PerpendicularPathInPlanner::TwoArcPlan(
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
        heading_err <
            apa_param.GetParam().target_heading_err * kDeg2Rad - 1e-6) {
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
        heading_err <
            apa_param.GetParam().target_heading_err * kDeg2Rad - 1e-6) {
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

  if (path_seg_vec.size() == 2) {
    // only save one gear path
    path_seg_vec.pop_back();
  }

  return true;
}

const bool PerpendicularPathInPlanner::LineArcPlan(
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
    DEBUG_PRINT("LineArcPlan fail 1");
    return false;
  }
  return true;
}

const bool PerpendicularPathInPlanner::OneLinePlan(
    pnc::geometry_lib::LineSegment& line,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear) {
  DEBUG_PRINT("--- try one line plan ---");
  pnc::geometry_lib::PathPoint pose;
  pose.Set(line.pA, line.heading);
  if (pnc::geometry_lib::IsPoseOnLine(
          pose, calc_params_.target_line,
          apa_param.GetParam().target_pos_err - 1e-6,
          apa_param.GetParam().target_heading_err * kDeg2Rad - 1e-6)) {
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

    if (seg_gear == geometry_lib::SEG_GEAR_REVERSE &&
        !input_.is_replan_dynamic) {
      geometry_lib::PathSegment temp_line_seg = line_seg;
      collision_detector_ptr_->SetParam(
          CollisionDetector::Paramters(g_params.car_lat_inflation_normal));
      PathColDetRes col_res = TrimPathByCollisionDetection(temp_line_seg);
      if (col_res == PathColDetRes::INVALID ||
          col_res == PathColDetRes::SHORTEN) {
        if (temp_line_seg.Getlength() < kMinSingleGearPathLength + 1e-3) {
          temp_line_seg.seg_gear = geometry_lib::SEG_GEAR_DRIVE;
          temp_line_seg.line_seg.length = kMinSingleGearPathLength + 1e-3;
          temp_line_seg.line_seg.pB =
              temp_line_seg.line_seg.pA +
              temp_line_seg.line_seg.length *
                  geometry_lib::GenHeadingVec(temp_line_seg.GetEndHeading());
          col_res = TrimPathByCollisionDetection(temp_line_seg);
          if (col_res == PathColDetRes::NORMAL) {
            line_seg = temp_line_seg;
          }
        }
      }
    }

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
const bool PerpendicularPathInPlanner::CheckAdjustPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose,
    const double slot_occupied_ratio) {
  if (std::fabs(current_pose.heading) <=
          apa_param.GetParam().adjust_plan_max_heading1_err * kDeg2Rad ||
      (std::fabs(current_pose.heading) <=
           apa_param.GetParam().adjust_plan_max_heading2_err * kDeg2Rad &&
       std::fabs(current_pose.pos.y()) <=
           apa_param.GetParam().adjust_plan_max_lat_err)) {
    return true;
  }
  DEBUG_PRINT("current pose is not suitable for adjust plan");
  return false;
}

const bool PerpendicularPathInPlanner::AdjustPlan() {
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
            calc_params_.adjust_fail_count > 10000) {
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
    const pnc::geometry_lib::PathSegment& path_seg =
        output_.path_segment_vec.back();

    // only the last seg is line and length meet length require, the control
    // would track it
    pnc::geometry_lib::PathPoint temp_pose;
    if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE &&
        path_seg.Getlength() > 0.368) {
      temp_pose.pos = path_seg.GetLineSeg().pB;
      temp_pose.heading = path_seg.GetLineSeg().heading;
    } else {
      // dynamic plan fail
      output_.Reset();
      return false;
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

const bool PerpendicularPathInPlanner::OneArcPlan(
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
      //           << "  heading = " << arc.headingA * kRad2Deg
      //           << "  end: coord = " << arc.pB.transpose()
      //           << "  heading = " << arc.headingB * kRad2Deg);
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }
  if (!success) {
    DEBUG_PRINT("one arc plan fail");
  }
  return success;
}

const bool PerpendicularPathInPlanner::LineArcPlan(
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
    DEBUG_PRINT("LineArcPlan fail 1");
    return false;
  }
  DEBUG_PRINT("line arc success");
  return true;
}

const bool PerpendicularPathInPlanner::AlignBodyPlan(
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
    //           << "  heading = " << arc.headingA * kRad2Deg
    //           << "  end: coord = " << arc.pB.transpose()
    //           << "  heading = " << arc.headingB * kRad2Deg);
    if (success) {
      DEBUG_PRINT("align body plan success");
      pnc::geometry_lib::PathSegment path_seg(steer, gear, arc);
      path_seg.plan_type = pnc::geometry_lib::PLAN_TYPE_ALIGN_BODY;
      path_seg_vec.emplace_back(std::move(path_seg));
    }
  }
  if (!success) {
    DEBUG_PRINT("align body plan fail");
  }
  return success;
}

const bool PerpendicularPathInPlanner::STurnParallelPlan(
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
      apa_param.GetParam().static_heading_eps * kDeg2Rad) {
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
    DEBUG_PRINT("s turn parallel plan fail");
  } else {
    DEBUG_PRINT("s turn parallel plan success");
  }

  return success;
}

const bool PerpendicularPathInPlanner::CalSinglePathInAdjust(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t& current_gear, const double& steer_change_ratio,
    const double& steer_change_radius, const size_t& i) {
  DEBUG_PRINT("-----CalSinglePathInAdjust-----");
  DEBUG_PRINT("current_gear = "
              << static_cast<int>(current_gear)
              << ",  current_pos = " << current_pose.pos.transpose()
              << ",  current_heading = " << current_pose.heading * kRad2Deg);

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
    // use fus obs
    double channel_width =
        collision_detector_ptr_->GetCarMaxX(input_.ego_pose) + 3.168 -
        std::max(input_.pt_0.x(), input_.pt_1.x());

    const double channel_para_width =
        (input_.slot_occupied_ratio < 0.368)
            ? apa_param.GetParam().channel_width
            : apa_param.GetParam().line_arc_obs_channel_width;

    channel_width = std::max(channel_width, channel_para_width);

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
        if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE &&
            input_.slot_occupied_ratio > 0.48) {
          path_seg_vec.clear();
          pnc::geometry_lib::LineSegment line;
          line.pA = current_pose.pos;
          line.heading = current_pose.heading;
          const std::vector<double> ratio_tab = {0.48, 0.68, 0.88, 0.98};
          const std::vector<double> length_tab = {0.68, 1.78, 2.78, 3.48};

          const double length = pnc::mathlib::Interp1(
              ratio_tab, length_tab, input_.slot_occupied_ratio);
          line.pB =
              line.pA + length * pnc::geometry_lib::GenHeadingVec(line.heading);
          line.length = (line.pB - line.pA).norm();
          pnc::geometry_lib::PathSegment line_seg(current_gear, line);
          path_seg_vec.emplace_back(std::move(line_seg));
        } else {
          path_seg_vec.emplace_back(tmp_path_seg);
        }
      } else {
        if (tmp_path_seg.plan_type == pnc::geometry_lib::PLAN_TYPE_S_TURN &&
            j > 0) {
          DEBUG_PRINT("s turn col, lose all s turn path.");
          if (tmp_path_seg_vec[j - 1].plan_type ==
              pnc::geometry_lib::PLAN_TYPE_S_TURN) {
            path_seg_vec.pop_back();
          }
          if (path_seg_vec.size() == 1) {
            const pnc::geometry_lib::PathSegment& path_seg =
                path_seg_vec.back();
            if (path_seg.plan_type == pnc::geometry_lib::PLAN_TYPE_ALIGN_BODY &&
                current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE &&
                input_.slot_occupied_ratio > 0.48) {
              pnc::geometry_lib::LineSegment line;
              line.pA = path_seg.GetEndPos();
              line.heading = path_seg.GetEndHeading();
              const std::vector<double> ratio_tab = {0.48, 0.68, 0.88, 0.98};
              const std::vector<double> length_tab = {0.68, 1.78, 2.78, 3.48};

              const double length =
                  pnc::mathlib::Interp1(ratio_tab, length_tab,
                                        input_.slot_occupied_ratio) -
                  path_seg.Getlength();
              line.pB = line.pA +
                        length * pnc::geometry_lib::GenHeadingVec(line.heading);
              line.length = (line.pB - line.pA).norm();
              pnc::geometry_lib::PathSegment line_seg(current_gear, line);
              path_seg_vec.emplace_back(std::move(line_seg));
            }
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

void PerpendicularPathInPlanner::InsertLineSegAfterCurrentFollowLastPath(
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
  if (pnc::mathlib::IsInBound(path_seg.GetEndHeading() * kRad2Deg, -0.208,
                              0.208)) {
    insert_case = 0;
  } else {
    if (length > apa_param.GetParam().min_one_step_path_length - 0.016) {
      if (std::fabs(path_seg.GetEndHeading() * kRad2Deg) >
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
            2.068 * min_path_length, 3.368 * min_path_length,
            3.968 * min_path_length, 4.368 * min_path_length,
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
      if (std::fabs(path_seg.GetEndHeading() * kRad2Deg) >
              apa_param.GetParam().multi_plan_min_heading_err &&
          path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
          path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
        extend_distance = std::max(extend_distance,
                                   apa_param.GetParam().insert_line_after_arc);
      }
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

const bool PerpendicularPathInPlanner::CheckCurrentGearLength() {
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

void PerpendicularPathInPlanner::ExtendCurrentFollowLastPath(
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

// collision detect start
const PerpendicularPathInPlanner::PathColDetRes
PerpendicularPathInPlanner::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg,
    CollisionDetector::CollisionResult* pcol_res) {
  return TrimPathByCollisionDetection(
      path_seg, apa_param.GetParam().col_obs_safe_dist_normal, pcol_res);
}

const PerpendicularPathInPlanner::PathColDetRes
PerpendicularPathInPlanner::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg, const double safe_dist,
    CollisionDetector::CollisionResult* pcol_res) {
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
    return PathColDetRes::INVALID;
  }

  if (pcol_res != nullptr) {
    *pcol_res = col_res;
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
    DEBUG_PRINT("col_pt_ego_global = "
                << col_res.col_pt_ego_global.transpose()
                << "  obs_pt_global = " << col_res.col_pt_obs_global.transpose()
                << "  car_line_order = " << col_res.car_line_order
                << "  obs_type = " << static_cast<int>(col_res.obs_type));

    // if 1R col by channel obs, even if safe_remain_dist is small. also plan
    // again
    if (calc_params_.multi_plan && calc_params_.first_multi_plan &&
        path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE &&
        path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
        col_res.col_pt_ego_local.x() > 1.68 &&
        ((calc_params_.is_left_side && col_res.col_pt_ego_local.y() < 0.0) ||
         (!calc_params_.is_left_side && col_res.col_pt_ego_local.y() > 0.0)) &&
        safe_remain_dist < 2.68) {
      return PathColDetRes::COMPLETE_PLAN_AGAIN;
    }

    return PathColDetRes::INVALID;
  }

  if (remain_car_dist > safe_remain_dist + 1e-3) {
    // DEBUG_PRINT(
    //     "the path will collide, the length need shorten to "
    //     "safe_remain_dist");
    path_seg.collision_flag = true;

    std::pair<Eigen::Vector2d, Eigen::Vector2d> slot_pt =
        std::make_pair(input_.pt_1, input_.pt_0);

    CollisionDetector::ObsSlotType obs_slot_type =
        collision_detector_ptr_->GetObsSlotType(
            col_res.col_pt_obs_global, slot_pt, calc_params_.is_left_side,
            true);

    DEBUG_PRINT("col_pt_ego_global = "
                << col_res.col_pt_ego_global.transpose()
                << "  col_pt_ego_local = "
                << col_res.col_pt_ego_local.transpose()
                << "  obs_pt_global = " << col_res.col_pt_obs_global.transpose()
                << "  car_line_order = " << col_res.car_line_order
                << "  obs_type = " << static_cast<int>(col_res.obs_type)
                << "  obs_slot_type = " << static_cast<int>(obs_slot_type));

    bool need_plan_again = false;
    if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_INSIDE_OBS &&
        col_res.col_pt_obs_global.x() > input_.tlane.pt_inside.x() - 0.368) {
      need_plan_again = true;
    }

    if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_IN_OBS &&
        col_res.col_pt_obs_global.x() > input_.tlane.pt_inside.x() - 0.368 &&
        ((calc_params_.is_left_side && col_res.col_pt_obs_global.y() > 0.998) ||
         (!calc_params_.is_left_side &&
          col_res.col_pt_obs_global.y() < -0.998))) {
      need_plan_again = true;
    }

    // if 1R col by slot inner obs, plan again
    if (need_plan_again && calc_params_.multi_plan &&
        path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      if (calc_params_.first_multi_plan) {
        return PathColDetRes::COMPLETE_PLAN_AGAIN;
      } else if (path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
        return PathColDetRes::SINGLE_PLAN_AGAIN;
      }
    }

    // if 1R col by channel obs, plan again
    if (calc_params_.multi_plan && calc_params_.first_multi_plan &&
        path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_REVERSE &&
        path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
        col_res.col_pt_ego_local.x() > 1.68 &&
        ((calc_params_.is_left_side && col_res.col_pt_ego_local.y() < 0.0) ||
         (!calc_params_.is_left_side && col_res.col_pt_ego_local.y() > 0.0)) &&
        safe_remain_dist < 2.68) {
      return PathColDetRes::COMPLETE_PLAN_AGAIN;
    }

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
    // DEBUG_PRINT("the path will not collide");
    path_seg.collision_flag = false;
    return PathColDetRes::NORMAL;
  }
}
// collision detect end

const std::vector<double> PerpendicularPathInPlanner::GetMinSafeCircle() const {
  return std::vector<double>{calc_params_.mono_safe_circle.center.x(),
                             calc_params_.mono_safe_circle.center.y(),
                             calc_params_.mono_safe_circle.radius};
}

const bool PerpendicularPathInPlanner::IsRightCircle(
    const pnc::geometry_lib::PathPoint& ego_pose,
    const Eigen::Vector2d& center) const {
  return IsRightCircle(ego_pose.pos, ego_pose.heading, center);
}

const bool PerpendicularPathInPlanner::IsRightCircle(
    const Eigen::Vector2d& ego_pos, const double ego_heading,
    const Eigen::Vector2d& center) const {
  const Eigen::Vector2d center_to_ego_vec = ego_pos - center;

  const Eigen::Vector2d ego_heading_vec(std::cos(ego_heading),
                                        std::sin(ego_heading));

  return (pnc::geometry_lib::GetCrossFromTwoVec2d(ego_heading_vec,
                                                  center_to_ego_vec) > 0.0);
}

const bool PerpendicularPathInPlanner::CheckTwoPoseInCircle(
    const Eigen::Vector2d& ego_pos0, const double ego_heading0,
    const Eigen::Vector2d& ego_pos1, const double ego_heading1,
    const Eigen::Vector2d& center) const {
  return IsRightCircle(ego_pos0, ego_heading0, center) &&
         IsRightCircle(ego_pos1, ego_heading1, center);
}

const bool PerpendicularPathInPlanner::CheckArcOrLineAvailable(
    const pnc::geometry_lib::Arc& arc) {
  const pnc::geometry_lib::PathPoint pose1(arc.pA, arc.headingA);
  const pnc::geometry_lib::PathPoint pose2(arc.pB, arc.headingB);
  if (pnc::geometry_lib::CheckTwoPoseIsSame(
          pose1, pose2, apa_param.GetParam().static_pos_eps,
          apa_param.GetParam().static_heading_eps * kDeg2Rad)) {
    DEBUG_PRINT("arc.pA = " << arc.pA.transpose()
                            << "  arc.headingA = " << arc.headingA * kRad2Deg
                            << "  arc.pB = " << arc.pB.transpose()
                            << "  arc.headingB = " << arc.headingB * kRad2Deg);

    return false;
  }
  return true;
}

const bool PerpendicularPathInPlanner::CheckArcOrLineAvailable(
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

const bool PerpendicularPathInPlanner::CheckPathIsNormal(
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

const bool PerpendicularPathInPlanner::CheckReachTargetPose(
    const pnc::geometry_lib::PathPoint& current_pose) {
  const double lon_err = current_pose.pos.x() - calc_params_.target_line.pA.x();
  const double lat_err =
      std::fabs(current_pose.pos.y() - calc_params_.target_line.pA.y());
  const double heading_err =
      std::fabs(pnc::geometry_lib::NormalizeAngle(
          current_pose.heading - input_.tlane.pt_terminal_heading)) *
      kRad2Deg;

  if (lon_err < 0.268 && lat_err < 0.0308 && heading_err < 0.268) {
    return true;
  }
  return false;
}

const bool PerpendicularPathInPlanner::CheckReachTargetPose() {
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

const double PerpendicularPathInPlanner::CalOccupiedRatio(
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

// for simulation
const bool PerpendicularPathInPlanner::PreparePlanPb() { return PreparePlan(); }

const bool PerpendicularPathInPlanner::PreparePlanSecondPb() {
  return PreparePlanSecond();
}

const bool PerpendicularPathInPlanner::GenPathOutputByDubinsPb() {
  input_.ego_pose.pos = dubins_planner_.GetInput().p2;
  input_.ego_pose.heading = dubins_planner_.GetInput().heading2;
  return GenPathOutputByDubins();
}

const bool PerpendicularPathInPlanner::MultiPlanPb() { return MultiPlan(); }

const bool PerpendicularPathInPlanner::AdjustPlanPb() { return AdjustPlan(); }

const PerpendicularPathInPlanner::PlannerParams&
PerpendicularPathInPlanner::GetCalcParams() {
  return calc_params_;
}

const bool PerpendicularPathInPlanner::CheckReachTargetPosePb() {
  return CheckReachTargetPose();
}

const bool PerpendicularPathInPlanner::UpdatePb(
    const Input& input,
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  input_ = input;
  collision_detector_ptr_ = collision_detector_ptr;
  Preprocess();
  calc_params_.is_searching_stage = false;
  input_.is_simulation = true;
  apa_param.SetPram().actual_mono_plan_enable = true;

  if (CheckReachTargetPose()) {
    DEBUG_PRINT("init pose is already at target pose");
    return true;
  }

  if (PreparePathPlan()) {
    calc_params_.adjust_fail_count = 0;
    calc_params_.first_multi_plan = true;
    DEBUG_PRINT("first prepare plan success");
  } else {
    DEBUG_PRINT("first prepare plan fail, quit");
    // return CalTurnAroundPose();
    return false;
  }
  if (CheckReachTargetPose()) {
    DEBUG_PRINT("first prepare plan to target pose");
    return true;
  }

  if (PreparePathPlanSecond()) {
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