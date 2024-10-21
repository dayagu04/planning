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
#include <iterator>
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

namespace planning {
namespace apa_planner {

static const double kMaxParkOutFirstArcHeading = 66.0;
static const double kMaxParkOutRootHeading = 25.0;

static const double kLonBufferTrippleStep = 0.2;
// static const double kLatBufferTrippleStep = 0.05;
static const double kColBufferInSlot = 0.2;
static const double kColBufferOutSlot = 0.5;
static const double kColLargeLatBufferOutSlot = 0.56;
static const double kColSmallLatBufferOutSlot = 0.1;
static const double kSmallColBufferInSlot = 0.1;

static const size_t kMaxParallelParkInSegmentNums = 15;
static const size_t kMaxPathNumsInSlot = 5;
static const size_t kMaxMultiStepNums = 8;
static const size_t kMaxParallelShiftNums = 6;

static const double kChannelYMoveDist = 0.15;
static const double kCornerSafeBufferWithChannel = 0.15;
static const double kMaxHeadingFirstStepForwardLine = 5.0;
static const double kMaxFirstStepForwardInclinedLineLength = 1.36;
static const double kVirtualObsDetaXMag = 0.1;
static const double kVirtualObsDetaYMag = 0.2;
static const double kMinTlaneAddedLength = 0.8;
static const double kNarrowChannelLastArcCrossLength = 1.38;

static const double kLineStepLength = 0.16;
static const double k1dExtendLength = 0.36;

static const size_t kInvalidInteger = 666;

void ParallelPathPlanner::Reset() {
  output_.Reset();
  calc_params_.Reset();
  output_.steer_vec.reserve(kMaxParallelParkInSegmentNums);
  output_.gear_cmd_vec.reserve(kMaxParallelParkInSegmentNums);
  output_.path_segment_vec.reserve(kMaxParallelParkInSegmentNums);
}

void ParallelPathPlanner::Preprocess() {
  DEBUG_PRINT("channel_y in path planner = " << input_.tlane.channel_y);
  calc_params_.Reset();
  debug_info_.debug_arc_vec.clear();
  output_.Reset();

  input_.ego_pose.heading =
      pnc::geometry_lib::NormalizeAngle(input_.ego_pose.heading);

  calc_params_.is_left_side =
      (input_.tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT);

  calc_params_.slot_side_sgn =
      (input_.tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) ? -1.0
                                                                    : 1.0;

  const double target_heading = 0.0;
  calc_params_.target_pose.Set(input_.tlane.pt_terminal_pos, target_heading);
  // target line
  calc_params_.target_line = pnc::geometry_lib::BuildLineSegByPose(
      input_.tlane.pt_terminal_pos, target_heading);

  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.0));

  ExpandPInObstacles();
  MoveChannelObstacles();

  CalcEgoParams();

  if (input_.slot_occupied_ratio < 0.01) {
    calc_params_.lat_outside_slot_buffer_vec = GetMinDistOfEgoToObs();
  }
}

void ParallelPathPlanner::ExpandPInObstacles() {
  const auto& obs_map = collision_detector_ptr_->GetObstaclesMap();

  DEBUG_PRINT("obs pin " << input_.tlane.obs_pt_inside.transpose());

  const Eigen::Vector2d coord_diff(
      -kVirtualObsDetaXMag, kVirtualObsDetaYMag * input_.tlane.slot_side_sgn);

  std::vector<size_t> obs_type_vec = {CollisionDetector::TLANE_OBS,
                                      CollisionDetector::TLANE_BOUNDARY_OBS};

  for (const auto obs_type : obs_type_vec) {
    const auto obs_pair = obs_map.find(obs_type);

    if (obs_pair == obs_map.end()) {
      continue;
    }

    // get nearby obstalces of pin
    for (const auto& tlane_obs_pt : obs_pair->second) {
      if (pnc::geometry_lib::CalTwoPointDistSquare(
              tlane_obs_pt, input_.tlane.obs_pt_inside) <= 0.25) {
        calc_params_.front_corner_obs_vec.emplace_back(tlane_obs_pt +
                                                       coord_diff);
      }
    }
  }

  // DEBUG_PRINT(
  //     "virtual tlane obs size = " <<
  //     calc_params_.front_corner_obs_vec.size());

  // for (const auto& obs_pt : calc_params_.front_corner_obs_vec) {
  //   DEBUG_PRINT(obs_pt.transpose());
  // }

  const Eigen::Vector2d channel_mov_vec(
      0.0, -calc_params_.slot_side_sgn * kChannelYMoveDist);
  // channel obs
  const auto channel_obs_pair = obs_map.find(CollisionDetector::CHANNEL_OBS);
  if (channel_obs_pair != obs_map.end()) {
    calc_params_.channel_obs_vec = channel_obs_pair->second;
    for (const auto& channel_obs_pt : channel_obs_pair->second) {
      calc_params_.virtual_channel_obs_vec.emplace_back(channel_obs_pt +
                                                        channel_mov_vec);
    }
  }
}

void ParallelPathPlanner::AddPInVirtualObstacles() {
  collision_detector_ptr_->AddObstacles(calc_params_.front_corner_obs_vec,
                                        CollisionDetector::VIRTUAL_OBS);
}

void ParallelPathPlanner::DeletePInVirtualObstacles() {
  collision_detector_ptr_->DeleteGivenTypeObstacles(
      CollisionDetector::VIRTUAL_OBS);
}

void ParallelPathPlanner::MoveChannelObstacles() {
  collision_detector_ptr_->DeleteGivenTypeObstacles(
      CollisionDetector::CHANNEL_OBS);

  collision_detector_ptr_->AddObstacles(
      calc_params_.virtual_channel_obs_vec,
      CollisionDetector::CollisionDetector::CHANNEL_OBS);
}

void ParallelPathPlanner::RecorverChannelObstacles() {
  collision_detector_ptr_->DeleteGivenTypeObstacles(
      CollisionDetector::CHANNEL_OBS);

  collision_detector_ptr_->AddObstacles(
      calc_params_.channel_obs_vec,
      CollisionDetector::CollisionDetector::CHANNEL_OBS);
}

const bool ParallelPathPlanner::Update() {
  std::cout << "-----------------------------------------parallel path "
               "planner:---------------------------------------"
            << std::endl;
  // preprocess
  Preprocess();

  if (!CheckTlaneAvailable()) {
    DEBUG_PRINT("tlane_too short!");
    return false;
  }

  const double start_time = IflyTime::Now_ms();

  // judge if ego is out of slot
  if (!CheckEgoInSlot()) {
    DEBUG_PRINT("ego is out of slot");
    AddPInVirtualObstacles();

    // if (MonoStepPlanWithShift()) {
    //   DEBUG_PRINT("MonoStepPlanWithShift success");
    // } else
    if (CalMinSafeCircle()) {
      DEBUG_PRINT("CalMinSafeCircle success!");
    } else {
      DEBUG_PRINT("calc safe circle failed!");
      return false;
    }

    const double start_plan_time = IflyTime::Now_ms();
    DEBUG_PRINT("calc safe circle cost time(ms) = " << start_plan_time -
                                                           start_time);

    const bool success = OutsideSlotPlan();
    DeletePInVirtualObstacles();
    const double outside_plan_end_time = IflyTime::Now_ms();
    DEBUG_PRINT("OutsideSlotPlan cost time(ms) = " << outside_plan_end_time -
                                                          start_plan_time);
    if (success) {
      DEBUG_PRINT("OutsideSlotPlan success!");
      if (calc_params_.park_out_path_in_slot.size() > 1) {
        ReversePathSegVec(calc_params_.park_out_path_in_slot);
        AddPathSegToOutPut(calc_params_.park_out_path_in_slot);
      }

      if (output_.path_segment_vec.size() > 0) {
        const auto& end_pose = output_.path_segment_vec.back().GetEndPose();
        if (pnc::mathlib::IsDoubleEqual(end_pose.heading * 57.3, 0.0) &&
            (!pnc::mathlib::IsDoubleEqual(end_pose.pos.x(),
                                          input_.tlane.pt_terminal_pos.x()))) {
          const Eigen::Vector2d fixed_target_pos(
              input_.tlane.pt_terminal_pos.x(), end_pose.pos.y());

          const pnc::geometry_lib::LineSegment last_line(
              end_pose.pos, fixed_target_pos, end_pose.heading);

          const pnc::geometry_lib::PathSegment last_path_seg(
              pnc::geometry_lib::CalLineSegGear(last_line), last_line);
          AddPathSegToOutPut(last_path_seg);
        }
      }
      return true;
    } else {
      DEBUG_PRINT("OutsideSlotPlan failed!");
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
  }
  output_.Reset();
  std::cout << "plan failed!" << std::endl;
  return false;
}

const bool ParallelPathPlanner::CheckTlaneAvailable() const {
  const double tlane_length =
      input_.tlane.obs_pt_inside.x() - input_.tlane.obs_pt_outside.x();

  const double min_released_slot_length =
      apa_param.GetParam().car_length + kMinTlaneAddedLength;

  DEBUG_PRINT("tlane_length = " << tlane_length);
  // DEBUG_PRINT("car length = " << apa_param.GetParam().car_length);
  DEBUG_PRINT("min_released_slot_length = " << min_released_slot_length);

  return tlane_length >= min_released_slot_length;
}

const bool ParallelPathPlanner::Update(
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  const auto time0 = std::chrono::high_resolution_clock::now();
  collision_detector_ptr_ = collision_detector_ptr;
  const bool success = Update();
  RecorverChannelObstacles();

  const auto time1 = std::chrono::high_resolution_clock::now();
  const auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(time1 - time0)
          .count();
  std::cout << "parallel cost time(ms) = " << duration << "\n" << std::endl;
  JSON_DEBUG_VALUE("path_plan_time_ms", duration);
  return success;
}

// park out from target pose with two arc to ego line.
const bool ParallelPathPlanner::PlanFromTargetToLine(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose) {
  // DEBUG_PRINT("------PlanFromTargetToLine-------");

  auto ego_line_unit =
      pnc::geometry_lib::BuildLineSegByPose(start_pose.pos, start_pose.heading);
  auto ego_line = ego_line_unit;
  bool success = false;
  pnc::geometry_lib::Arc arc_1;
  pnc::geometry_lib::Arc arc_2;
  arc_1.circle_info.radius = apa_param.GetParam().min_turn_radius;
  const uint8_t arc_1_steer =
      (calc_params_.is_left_side ? pnc::geometry_lib::SEG_STEER_RIGHT
                                 : pnc::geometry_lib::SEG_STEER_LEFT);

  // DEBUG_PRINT(
  //     "valid_target_pt_vec size = " <<
  //     calc_params_.valid_target_pt_vec.size());

  bool is_narrow_channel_activated = false;
  std::vector<pnc::geometry_lib::PathSegment> narrow_path_seg_vec;
  for (const auto& target_pose : calc_params_.valid_target_pt_vec) {
    pnc::geometry_lib::PrintPose("current target", target_pose);
    arc_1.pA = target_pose.pos;
    arc_1.headingA = target_pose.heading;
    arc_1.circle_info.center = CalEgoTurningCenter(
        target_pose, apa_param.GetParam().min_turn_radius, arc_1_steer);

    if (!pnc::geometry_lib::CalTwoSameGearArcWithLine(
            arc_1, arc_2, ego_line_unit, pnc::geometry_lib::SEG_GEAR_DRIVE)) {
      // DEBUG_PRINT("CalTwoArcWithLine fail!");
      continue;
    }

    // if (!CheckParkOutCornerSafeWithObsPin(arc_1)) {
    //   DEBUG_PRINT("front inner corner collided!");
    //   continue;
    // }

    // last line step length is more than min_leng during dirve gear in mono
    // step
    if (calc_params_.valid_target_pt_vec.size() > 1) {
      const pnc::geometry_lib::PathPoint last_pA(arc_1.pA, arc_1.headingA);

      const pnc::geometry_lib::PathPoint last_pB(
          input_.tlane.pt_terminal_pos, calc_params_.target_line.heading);

      const pnc::geometry_lib::LineSegment last_line_seg(
          last_pA.pos, last_pB.pos, last_pA.heading);
      const uint8_t last_line_gear =
          pnc::geometry_lib::CalLineSegGear(last_line_seg);

      if (!CheckSamePose(last_pA, last_pB) &&
          last_line_gear == pnc::geometry_lib::SEG_GEAR_DRIVE &&
          last_line_seg.length < apa_param.GetParam().min_line_length) {
        continue;
      }
    }

    ego_line.heading = start_pose.heading;
    ego_line.SetPoints(start_pose.pos, arc_2.pB);
    const auto ego_line_gear = pnc::geometry_lib::CalLineSegGear(ego_line);
    if (ego_line_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      // prolong the path during preparation, or will not calc success
      // during normal backward step if ego stops early
      const auto v_ego = pnc::geometry_lib::GenHeadingVec(ego_line.heading);
      ego_line.SetPoints(
          start_pose.pos,
          start_pose.pos + (ego_line.length + k1dExtendLength) * v_ego);

      // make sure first line step length are more than min_leng during dirve
      // gear but not too long
      if (input_.is_replan_first) {
        const double heading_mag_deg = std::fabs(start_pose.heading) * 57.3;
        if (heading_mag_deg > kMaxHeadingFirstStepForwardLine &&
            ego_line.length > kMaxFirstStepForwardInclinedLineLength) {
          continue;
        }
      }
    }

    double left_buf = 0.0;
    double right_buf = 0.0;
    if (input_.tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
      left_buf = kColSmallLatBufferOutSlot;
      right_buf = 0.0;
    } else {
      left_buf = 0.0;
      right_buf = kColSmallLatBufferOutSlot;
    }
    collision_detector_ptr_->SetParam(
        CollisionDetector::Paramters(left_buf, right_buf));

    auto col_res =
        collision_detector_ptr_->UpdateByObsMap(ego_line, ego_line.heading);
    if (col_res.collision_flag ||
        col_res.remain_car_dist >
            col_res.remain_obstacle_dist - kColBufferOutSlot) {
      // DEBUG_PRINT("ego line collided!");
      continue;
    }

    col_res = collision_detector_ptr_->Update(arc_1, arc_1.headingA);
    if (col_res.collision_flag ||
        col_res.remain_car_dist >
            col_res.remain_obstacle_dist - kLonBufferTrippleStep) {
      // DEBUG_PRINT("arc 1 collided!");
      continue;
    }
    // if (!CheckParkOutCornerSafeWithObsPin(arc_1)) {
    //   DEBUG_PRINT("arc1 collided!");
    //   continue;
    // }

    col_res = collision_detector_ptr_->UpdateByObsMap(arc_2, arc_2.headingA);
    if (col_res.collision_flag ||
        col_res.remain_car_dist >
            col_res.remain_obstacle_dist - kLonBufferTrippleStep) {
      // DEBUG_PRINT("arc 2 collided!");
      is_narrow_channel_activated = true;
      if (!PlanFromTargetToLineInNarrowChannel(narrow_path_seg_vec, arc_1,
                                               arc_2)) {
        // DEBUG_PRINT("narrow channel plan failed");
        continue;
      } else {
        // DEBUG_PRINT("narrow channel plan success!");
        // DEBUG_PRINT(
        //     "narrow_path_seg_vec.size() = " << narrow_path_seg_vec.size());
      }
    }
    success = true;
    break;
  }

  if (!success) {
    // DEBUG_PRINT("arc2 collided! && narrow plan failed!");
    return false;
  }

  if (!is_narrow_channel_activated) {
    pnc::geometry_lib::PathSegment arc_seg_1(pnc::geometry_lib::SEG_STEER_LEFT,
                                             pnc::geometry_lib::SEG_GEAR_DRIVE,
                                             arc_1);

    pnc::geometry_lib::PathSegment arc_seg_2(pnc::geometry_lib::SEG_STEER_RIGHT,
                                             pnc::geometry_lib::SEG_GEAR_DRIVE,
                                             arc_2);

    pnc::geometry_lib::ReverseArcSegInfo(arc_seg_1);
    pnc::geometry_lib::ReverseArcSegInfo(arc_seg_2);

    if (!pnc::mathlib::IsDoubleEqual(ego_line.length, 0.0)) {
      path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
          pnc::geometry_lib::CalLineSegGear(ego_line), ego_line));
    }
    pnc::geometry_lib::LineSegment tmp_line(
        ego_line.pB, arc_seg_2.GetStartPos(), ego_line.heading);

    path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::SEG_GEAR_REVERSE, tmp_line));

    path_seg_vec.emplace_back(arc_seg_2);
    path_seg_vec.emplace_back(arc_seg_1);
  } else {
    pnc::geometry_lib::LineSegment ego_line_in_channel;
    ego_line_in_channel.heading = start_pose.heading;
    ego_line_in_channel.SetPoints(start_pose.pos,
                                  narrow_path_seg_vec.back().GetEndPos());
    // DEBUG_PRINT("narrow_path_seg_vec.back().GetEndPos() = "
    //             << narrow_path_seg_vec.back().GetEndPos());

    if (!pnc::mathlib::IsDoubleEqual(ego_line.length, 0.0)) {
      // DEBUG_PRINT("ego line len not == 0.0");
      const auto ego_line_gear =
          pnc::geometry_lib::CalLineSegGear(ego_line_in_channel);
      // DEBUG_PRINT("ego_line_gear = " << static_cast<int>(ego_line_gear));

      if (ego_line_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
        const auto v_extend =
            (ego_line_in_channel.pB - ego_line_in_channel.pA).normalized();

        ego_line_in_channel.SetPoints(
            ego_line_in_channel.pA,
            ego_line_in_channel.pB + k1dExtendLength * v_extend);
      }
      path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
          pnc::geometry_lib::CalLineSegGear(ego_line_in_channel),
          ego_line_in_channel));

      const pnc::geometry_lib::PathPoint extended_pose(
          ego_line_in_channel.pB, ego_line_in_channel.heading);

      if (!CheckSamePose(extended_pose,
                         narrow_path_seg_vec.back().GetEndPose())) {
        const pnc::geometry_lib::LineSegment back_line(
            extended_pose.pos, narrow_path_seg_vec.back().GetEndPose().pos,
            extended_pose.heading);

        const pnc::geometry_lib::PathSegment back_line_seg(
            pnc::geometry_lib::SEG_GEAR_REVERSE, back_line);

        path_seg_vec.emplace_back(std::move(back_line_seg));
      }

      for (auto& narrow_path_seg : narrow_path_seg_vec) {
        if (!pnc::geometry_lib::ReversePathSegInfo(narrow_path_seg)) {
          DEBUG_PRINT("reverse single path seg failed!");
        }
      }

      std::reverse(narrow_path_seg_vec.begin(), narrow_path_seg_vec.end());

      for (const auto& path_seg : narrow_path_seg_vec) {
        path_seg_vec.emplace_back(path_seg);
      }
    }

    if (pnc::mathlib::IsDoubleEqual(path_seg_vec.back().GetEndHeading(),
                                    calc_params_.target_pose.heading) &&
        !pnc::mathlib::IsDoubleEqual(path_seg_vec.back().GetEndPos().x(),
                                     input_.tlane.pt_terminal_pos.x())) {
      const Eigen::Vector2d fixed_target_pos(
          input_.tlane.pt_terminal_pos.x(),
          path_seg_vec.back().GetEndPos().y());

      const pnc::geometry_lib::LineSegment last_line(
          path_seg_vec.back().GetEndPos(), fixed_target_pos,
          path_seg_vec.back().GetEndHeading());

      path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
          pnc::geometry_lib::CalLineSegGear(last_line), last_line));
    }
  }
  return success;
}

const bool ParallelPathPlanner::PlanFromTargetToLineInNarrowChannel(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::Arc& arc1, const pnc::geometry_lib::Arc& arc_2) {
  path_seg_vec.clear();
  path_seg_vec.reserve(10);
  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.1));

  // DEBUG_PRINT(
  //     "------------------------  PlanFromTargetToLineInNarrowChannel "
  //     "----------------------");

  // DEBUG_PRINT("slot_side_sgn = " << calc_params_.slot_side_sgn);
  // DEBUG_PRINT("arc_2 center = " << arc_2.circle_info.center.transpose()
  //                               << ", radius = " <<
  //                               arc_2.circle_info.radius);

  // debug for corner arcs
  // debug_info_.debug_arc_vec.emplace_back(arc_2);
  // auto tmp_corner_arc = arc_2;
  // tmp_corner_arc.circle_info.radius =
  //     calc_params_.min_outer_front_corner_radius;
  // debug_info_.debug_arc_vec.emplace_back(tmp_corner_arc);
  // tmp_corner_arc.circle_info.radius = apa_param.GetParam().min_turn_radius -
  //                                     0.5 * apa_param.GetParam().car_width;
  // debug_info_.debug_arc_vec.emplace_back(tmp_corner_arc);

  const pnc::geometry_lib::PathPoint start_pose(arc1.pA, arc1.headingA);

  Eigen::Vector2d new_center = Eigen::Vector2d::Zero();
  // select new center according to obs instead of channel width
  // DEBUG_PRINT("current channel y = " << input_.tlane.channel_y);

  new_center.y() = input_.tlane.channel_y -
                   calc_params_.slot_side_sgn *
                       (kCornerSafeBufferWithChannel + kChannelYMoveDist +
                        calc_params_.min_outer_front_corner_radius);

  const double radius_square =
      std::pow(arc1.circle_info.radius + arc_2.circle_info.radius, 2.0);

  const double dy_square =
      std::pow(arc1.circle_info.center.y() - new_center.y(), 2.0);

  const double dx_square = radius_square - dy_square;
  if (dx_square < 0.0) {
    // DEBUG_PRINT("geometry dy^2 < 0.0!");
    return false;
  }

  new_center.x() = arc1.circle_info.center.x() + std::sqrt(dx_square);
  // DEBUG_PRINT("new center " << new_center.transpose());

  // for debug
  debug_info_.debug_arc_vec.emplace_back(arc1);
  pnc::geometry_lib::Arc new_arc2 = arc_2;
  new_arc2.circle_info.center = new_center;
  debug_info_.debug_arc_vec.emplace_back(new_arc2);
  new_arc2.circle_info.radius = calc_params_.min_outer_front_corner_radius;
  debug_info_.debug_arc_vec.emplace_back(new_arc2);

  const double corner_theta =
      std::atan(std::fabs(calc_params_.v_ego_farest_front_corner.x()) /
                (apa_param.GetParam().min_turn_radius +
                 std::fabs(calc_params_.v_ego_farest_front_corner.y()))) *
      calc_params_.slot_side_sgn;
  // DEBUG_PRINT("corner_theta deg = " << corner_theta * 57.3);

  const double travel_theta = -calc_params_.slot_side_sgn *
                              kNarrowChannelLastArcCrossLength /
                              arc_2.circle_info.radius;
  // DEBUG_PRINT("travel_theta deg = " << travel_theta * 57.3);
  const double d_theta =
      pnc::geometry_lib::NormalizeAngle(corner_theta + travel_theta);
  // DEBUG_PRINT("d_theta = " << d_theta * 57.3);
  const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(d_theta);

  const Eigen::Vector2d v_vertical(0.0, calc_params_.slot_side_sgn);

  const auto pos_B =
      (rot_m * v_vertical) * apa_param.GetParam().min_turn_radius + new_center;

  // DEBUG_PRINT("v vertical = " << v_vertical.transpose());
  // DEBUG_PRINT("(rot_m * v_vertical) = " << (rot_m * v_vertical).transpose());
  // DEBUG_PRINT("(rot_m * v_vertical) * apa_param.GetParam().min_turn_radius =
  // "
  //             << (rot_m * v_vertical) *
  //             apa_param.GetParam().min_turn_radius);
  // DEBUG_PRINT("new center " << new_center.transpose());

  const double heading_B = pnc::geometry_lib::NormalizeAngle(d_theta);

  const pnc::geometry_lib::PathPoint tmp_target_pose(pos_B, heading_B);
  // pnc::geometry_lib::PrintPose("narrow channel arc end pose",
  // tmp_target_pose);

  std::vector<pnc::geometry_lib::PathSegment> first_three_steps;

  pnc::dubins_lib::DubinsLibrary::Input dubins_input;
  dubins_input.radius = apa_param.GetParam().min_turn_radius;
  dubins_input.Set(start_pose.pos, tmp_target_pose.pos, start_pose.heading,
                   tmp_target_pose.heading);

  dubins_planner_.SetInput(dubins_input);
  const uint8_t dubins_type =
      calc_params_.slot_side_sgn > 0.0
          ? pnc::dubins_lib::DubinsLibrary::DubinsType::L_S_R
          : pnc::dubins_lib::DubinsLibrary::DubinsType::R_S_L;

  bool success = false;
  // try dubins method
  for (size_t i = 0; i < pnc::dubins_lib::DubinsLibrary::CASE_COUNT; ++i) {
    if (dubins_planner_.Solve(dubins_type, i)) {
      if (output_.gear_change_count == 0) {
        // std::cout << "dubins success!" << std::endl;
        success = true;
        break;
      }
    }
  }  // dubins loop
  // debug_info_.debug_arc_vec.emplace_back(dubins_planner_.GetOutput().arc_AB);
  // debug_info_.debug_arc_vec.emplace_back(dubins_planner_.GetOutput().arc_CD);
  // auto cd_corner_arc = dubins_planner_.GetOutput().arc_CD;
  // cd_corner_arc.circle_info.radius =
  // calc_params_.min_outer_front_corner_radius;
  // debug_info_.debug_arc_vec.emplace_back(cd_corner_arc);
  if (!success) {
    // DEBUG_PRINT("tripple step dubins failed!");
    return false;
  }

  // DEBUG_PRINT("tripple step dubins success!");
  // debug_info_.debug_arc_vec.emplace_back(dubins_planner_.GetOutput().arc_CD);
  success = true;
  GetPathSegVecByDubins(first_three_steps);

  const uint8_t gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
  const uint8_t steer = calc_params_.slot_side_sgn > 0.0
                            ? pnc::geometry_lib::SEG_STEER_RIGHT
                            : pnc::geometry_lib::SEG_STEER_LEFT;
  const pnc::geometry_lib::PathPoint target_pose(arc_2.pB, arc_2.headingB);
  auto current_pose = first_three_steps.back().GetEndPose();
  for (size_t i = 0; i < 3; i++) {
    std::vector<pnc::geometry_lib::PathSegment> tmp_line_arc_seg_vec;
    if (!CalSinglePathInNarrowChannel(tmp_line_arc_seg_vec, current_pose,
                                      target_pose, gear, steer)) {
      DEBUG_PRINT("CalSinglePathInNarrowChannel failed!");
      success = false;
      break;
    }

    if (tmp_line_arc_seg_vec.back().seg_type ==
        pnc::geometry_lib::SEG_TYPE_LINE) {
      tmp_line_arc_seg_vec.pop_back();
    }

    for (const auto& path_seg : tmp_line_arc_seg_vec) {
      first_three_steps.emplace_back(path_seg);
    }
    current_pose = first_three_steps.back().GetEndPose();

    if (pnc::mathlib::IsDoubleEqual(tmp_line_arc_seg_vec.back().GetEndHeading(),
                                    target_pose.heading)) {
      DEBUG_PRINT("already plan to prepare line");
      success = true;
      break;
    }
  }

  if (success) {
    path_seg_vec = first_three_steps;
    // DEBUG_PRINT("path_seg_vec size = " << path_seg_vec.size());
    //   for (const auto& path_seg : path_seg_vec) {
    //     if (path_seg.seg_type ==
    //     pnc::geometry_lib::PathSegType::SEG_TYPE_LINE) {
    //       continue;
    //     }
    //     debug_info_.debug_arc_vec.emplace_back(path_seg.GetArcSeg());
    //   }
  }

  return success;
}

const bool ParallelPathPlanner::CalSinglePathInNarrowChannel(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const uint8_t current_gear,
    const uint8_t current_arc_steer) {
  // DEBUG_PRINT("----- " << __func__ << " -----");
  // DEBUG_PRINT("current_arc_steer = "
  //             << static_cast<int>(current_arc_steer)
  //             << ",  current_gear = " << static_cast<int>(current_gear)
  //             << ",  current_pos = " << current_pose.pos.transpose()
  //             << ",  current_heading = " << current_pose.heading * 57.3);

  path_seg_vec.clear();
  path_seg_vec.reserve(3);

  pnc::geometry_lib::Arc current_arc;
  current_arc.pA = current_pose.pos;
  current_arc.headingA =
      pnc::geometry_lib::NormalizeAngle(current_pose.heading);

  const double min_radius = apa_param.GetParam().min_turn_radius;
  current_arc.circle_info.radius = min_radius;
  current_arc.circle_info.center =
      CalEgoTurningCenter(current_pose, min_radius, current_arc_steer);

  // debug_info_.debug_arc_vec.emplace_back(current_arc);

  auto target_line = pnc::geometry_lib::BuildLineSegByPose(target_pose.pos,
                                                           target_pose.heading);

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  if (!LineArcPlan(current_arc, tmp_path_seg_vec, target_line, current_gear,
                   current_arc_steer)) {
    DEBUG_PRINT("LineArcPlan fail");
    return false;
  }

  uint8_t col_res = PATH_COL_COUNT;

  size_t path_idx = 0;
  for (; path_idx < tmp_path_seg_vec.size(); path_idx++) {
    auto& tmp_path_seg = tmp_path_seg_vec[path_idx];
    col_res = TrimPathByCollisionDetection(tmp_path_seg, kColBufferOutSlot);

    if (col_res == PATH_COL_NORMAL) {
      DEBUG_PRINT("No. " << path_idx << " normal!");
    } else if (col_res == PATH_COL_SHORTEN) {
      break;
    } else if (col_res == PATH_COL_INVALID) {
      DEBUG_PRINT("path col at start pose, invalid!");
      break;
    }
  }
  bool success = false;
  if (col_res == PATH_COL_SHORTEN) {
    auto ego_line = pnc::geometry_lib::BuildLineSegByPose(current_pose.pos,
                                                          current_pose.heading);

    ego_line.pB = 2.0 * ego_line.pB - ego_line.pA;
    ego_line.length = (ego_line.pB - ego_line.pA).norm();
    pnc::geometry_lib::PathSegment ego_line_seg(current_gear, ego_line);
    col_res = TrimPathByCollisionDetection(ego_line_seg, kColBufferOutSlot);
    path_seg_vec.emplace_back(ego_line_seg);

    pnc::geometry_lib::Arc next_arc;
    next_arc.pA = ego_line_seg.GetEndPos();
    next_arc.headingA = ego_line_seg.GetEndHeading();
    next_arc.circle_info.radius = min_radius;
    next_arc.circle_info.center = CalEgoTurningCenter(
        ego_line_seg.GetEndPose(), min_radius, current_arc_steer);

    bool is_anti_clockwise =
        (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE &&
         current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) ||
        (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE &&
         current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT);

    if (!pnc::geometry_lib::CompleteArcInfo(next_arc, 0.6, is_anti_clockwise)) {
      return false;
    }
    path_seg_vec.emplace_back(pnc::geometry_lib::PathSegment(
        current_arc_steer, current_gear, next_arc));

    success = true;
  } else if (col_res == PATH_COL_NORMAL) {
    success = true;
    path_seg_vec = tmp_path_seg_vec;
  } else if (col_res == PATH_COL_INVALID) {
    success = false;
  }

  return success;
}

const bool ParallelPathPlanner::MonoStepPlanWithShift() {
  std::vector<double> target_y_vec;
  target_y_vec.clear();
  target_y_vec.reserve(10);
  target_y_vec.emplace_back(input_.tlane.pt_terminal_pos.y());

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
        if (safe_y_cnt == 2) {
          calc_params_.safe_circle_root_pose = tmp_pose;
        }
      }
    }
  }

  return success;
}

const bool ParallelPathPlanner::MonoStepPlanOnceWithShift(
    bool& is_drive_out_safe, const pnc::geometry_lib::PathPoint& target_pose) {
  // calc backward limit
  pnc::geometry_lib::LineSegment backward_line;
  backward_line.pA = target_pose.pos;
  backward_line.heading = target_pose.heading;

  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.15));

  if (!CalcLineStepLimitPose(backward_line, pnc::geometry_lib::SEG_GEAR_REVERSE,
                             0.4)) {
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

  if (!CalcArcStepLimitPose(forward_arc, pnc::geometry_lib::SEG_GEAR_DRIVE,
                            forward_steer, kColBufferInSlot)) {
    std::cout << "calc forward arc limit error!" << std::endl;
    return false;
  }

  if (!CheckParkOutCornerSafeWithObsPin(forward_arc)) {
    DEBUG_PRINT("park out failed in backward line limit!");
    return false;
  }

  pnc::geometry_lib::PathSegment forward_seg(
      forward_steer, pnc::geometry_lib::SEG_GEAR_DRIVE, forward_arc);
  calc_params_.park_out_path_in_slot.clear();
  calc_params_.park_out_path_in_slot.emplace_back(std::move(forward_seg));

  calc_params_.valid_target_pt_vec.clear();
  calc_params_.valid_target_pt_vec.emplace_back(
      pnc::geometry_lib::PathPoint(forward_arc.pA, forward_arc.headingA));

  // find all pt which can park out along the y_offset line
  const double length = std::fabs(target_pose.pos.x() - forward_arc.pA.x());
  const double step = length / 3.0;
  const auto back_line_limit = forward_arc.pA;

  for (double x_offset = step; x_offset < length + 0.3; x_offset += step) {
    forward_arc.pA.x() = back_line_limit.x() + x_offset;
    if (!CalcArcStepLimitPose(forward_arc, pnc::geometry_lib::SEG_GEAR_DRIVE,
                              forward_steer, kColBufferInSlot)) {
      std::cout << "calc forward arc limit error!" << std::endl;
      break;
    }
    if (!CheckParkOutCornerSafeWithObsPin(forward_arc)) {
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

  return true;
}

const bool ParallelPathPlanner::BackwardNormalPlan() {
  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
  path_seg_vec.reserve(10);

  if (OneStepDubinsTryInTripplePlan(path_seg_vec, input_.ego_pose)) {
    AddPathSegToOutPut(path_seg_vec);
    DEBUG_PRINT("ego_pose OneStepDubinsTryInTripplePlan success!");
    return true;
  }

  if (PlanFromTargetToLine(path_seg_vec, input_.ego_pose)) {
    AddPathSegToOutPut(path_seg_vec);
    DEBUG_PRINT("ego_pose PlanFromTargetToLine success!");
    return true;
  }

  return false;
}

const bool ParallelPathPlanner::BackwardNormalPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose) {
  path_seg_vec.clear();
  path_seg_vec.reserve(10);

  if (OneStepDubinsTryInTripplePlan(path_seg_vec, start_pose)) {
    DEBUG_PRINT("OneStepDubinsTryInTripplePlan success!");
    return true;
  }

  if (PlanFromTargetToLine(path_seg_vec, start_pose)) {
    DEBUG_PRINT("PlanFromTargetToLine success!");
    return true;
  }

  return false;
}

const bool ParallelPathPlanner::OneStepDubinsTryInTripplePlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose) {
  path_seg_vec.clear();
  path_seg_vec.reserve(5);

  std::vector<double> radius_vec = {apa_param.GetParam().min_turn_radius + 0.5,
                                    apa_param.GetParam().min_turn_radius};
  if (input_.ego_pose.pos.x() > input_.tlane.slot_length + 1.0) {
    radius_vec.insert(radius_vec.begin(),
                      apa_param.GetParam().min_turn_radius + 1.5,
                      apa_param.GetParam().min_turn_radius + 1.0);
  }

  DEBUG_PRINT("terminal pos = " << input_.tlane.pt_terminal_pos.transpose());

  DEBUG_PRINT("target pose ---------------");
  for (const auto& target_pt : calc_params_.valid_target_pt_vec) {
    DEBUG_PRINT("target_pt_x =" << target_pt.pos.transpose());
  }

  for (const auto& radius : radius_vec) {
    // start backward three step plan
    for (const auto& target_pose : calc_params_.valid_target_pt_vec) {
      if (!OneStepDubinsPlan(start_pose, target_pose, radius,
                             kLonBufferTrippleStep)) {
        continue;
      }

      const auto line_seg = dubins_planner_.GetOutput().line_BC;

      if (!line_seg.is_ignored &&
          pnc::geometry_lib::CalPoint2LineDist(input_.tlane.obs_pt_inside,
                                               line_seg) <
              apa_param.GetParam().parallel_ego_side_to_obs_in_buffer) {
        continue;
      }

      DEBUG_PRINT("plan from ego pose to valid target pose success!"
                  << " valid pos= " << target_pose.pos.transpose()
                  << ", radius =" << radius);

      GetPathSegVecByDubins(path_seg_vec);
      return true;
    }

    if (calc_params_.valid_target_pt_vec.size() > 1 ||
        !OneStepDubinsPlan(start_pose, calc_params_.park_out_pose, radius,
                           kLonBufferTrippleStep)) {
      continue;
    }

    const auto line_seg = dubins_planner_.GetOutput().line_BC;

    if (!line_seg.is_ignored &&
        pnc::geometry_lib::CalPoint2LineDist(input_.tlane.obs_pt_inside,
                                             line_seg) -
                0.5 * apa_param.GetParam().car_width <
            apa_param.GetParam().parallel_ego_side_to_obs_in_buffer) {
      continue;
    }

    DEBUG_PRINT(
        "triple step from ego to park out pose success! radius = " << radius);

    auto last_path_seg = calc_params_.park_out_path_in_slot.back();

    if (!pnc::geometry_lib::ReversePathSegInfo(last_path_seg)) {
      continue;
    }

    GetPathSegVecByDubins(path_seg_vec);
    path_seg_vec.emplace_back(last_path_seg);

    return true;
  }

  return false;
}

const bool ParallelPathPlanner::OutsideSlotPlan() {
  DEBUG_PRINT(
      "---------------------------------- outside slot plan "
      "----------------------------------");

  std::vector<GeometryPath> geo_path_vec;
  debug_info_.debug_all_path_vec.clear();

  if (input_.ego_pose.pos.x() > input_.tlane.slot_length - 1.5) {
    GeometryPath ego_line_geo_path;
    std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
    collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.1));
    if (BackwardNormalPlan(tmp_path_seg_vec, input_.ego_pose)) {
      AssempleGeometryPath(ego_line_geo_path, tmp_path_seg_vec);

      DEBUG_PRINT("first try ego line plan success!");
      if (ego_line_geo_path.gear_change_count == 0) {
        debug_info_.debug_all_path_vec.emplace_back(ego_line_geo_path);
        AddPathSegToOutPut(ego_line_geo_path.path_segment_vec);
        DEBUG_PRINT("no need change gear in preparing step!");
        return true;
      }
      geo_path_vec.emplace_back(ego_line_geo_path);
    }
  }

  // Todo: connect two line vec method
  std::vector<double> preparing_line_y_vec;
  GenPreparingLineVec(preparing_line_y_vec);
  GenAlignedPreparingLine(preparing_line_y_vec, input_.ego_pose);

  std::sort(preparing_line_y_vec.begin(), preparing_line_y_vec.end(),
            [sgn = input_.tlane.slot_side_sgn](int a, int b) {
              return a * sgn < b * sgn;
            });

  DEBUG_PRINT("preparing_line_y_vec size= " << preparing_line_y_vec.size());
  for (const auto y : preparing_line_y_vec) {
    std::cout << y << ", ";
  }
  std::cout << std::endl;

  size_t success_cnt = 0;
  for (const auto& prepare_y : preparing_line_y_vec) {
    DEBUG_PRINT("side dist to tlane p in = "
                << std::fabs(prepare_y - input_.tlane.obs_pt_inside.y()));

    const Eigen::Vector2d prepare_pos(input_.tlane.pt_inside.x(), prepare_y);
    pnc::geometry_lib::PathPoint prepare_pose(prepare_pos, 0.0);
    std::vector<pnc::geometry_lib::PathSegment> inversed_park_out_path;
    if (!PlanFromTargetToLine(inversed_park_out_path, prepare_pose)) {
      DEBUG_PRINT(" PlanFromTargetToLine failed!");
      continue;
    }

    // DEBUG_PRINT(
    //     "inversed_park_out_path size =" << inversed_park_out_path.size());
    if (inversed_park_out_path.size() == 0) {
      continue;
    }

    while (inversed_park_out_path.front().seg_type ==
           pnc::geometry_lib::SEG_TYPE_LINE) {
      inversed_park_out_path.erase(inversed_park_out_path.begin());
    }

    prepare_pose = inversed_park_out_path.front().GetStartPose();
    const auto prepare_line = pnc::geometry_lib::BuildLineSegByPose(
        prepare_pose.pos, prepare_pose.heading);

    std::vector<pnc::geometry_lib::PathSegment> prepare_seg_vec;
    if (!PlanToPreparingLine(prepare_seg_vec, input_.ego_pose, prepare_line)) {
      // DEBUG_PRINT("PlanToPreparingLine fail!");
      continue;
    }

    // DEBUG_PRINT("prepare_seg_vec size == " << prepare_seg_vec.size());
    if (prepare_seg_vec.size() == 0) {
      // DEBUG_PRINT("prepare_seg_vec size == 0");
      continue;
    }

    prepare_seg_vec.insert(prepare_seg_vec.end(),
                           inversed_park_out_path.begin(),
                           inversed_park_out_path.end());
    GeometryPath tmp_geo_path;
    if (!AssempleGeometryPath(tmp_geo_path, prepare_seg_vec)) {
      DEBUG_PRINT("AssempleGeometryPath failed!");
      continue;
    }

    // DEBUG_PRINT("calc success!");
    geo_path_vec.emplace_back(tmp_geo_path);
    success_cnt++;
    if (success_cnt > 4) {
      break;
    }
  }

  DEBUG_PRINT("path size = " << geo_path_vec.size());
  if (geo_path_vec.size() == 0) {
    DEBUG_PRINT("none path calculated!");
    return false;
  }

  size_t best_path_idx = std::numeric_limits<size_t>::max();
  if (!SelectBestPathOutsideSlot(geo_path_vec, best_path_idx)) {
    DEBUG_PRINT("SelectBestPathOutsideSlot failed!");
    return false;
  }
  AddPathSegToOutPut(geo_path_vec[best_path_idx].path_segment_vec);
  debug_info_.debug_all_path_vec = std::move(geo_path_vec);

  return true;
}

const bool ParallelPathPlanner::PlanToPreparingLine(
    std::vector<pnc::geometry_lib::PathSegment>& ego_to_prepare_seg_vec,
    const pnc::geometry_lib::PathPoint& ego_pose,
    const pnc::geometry_lib::LineSegment& prepare_line) {
  using namespace pnc::geometry_lib;
  ego_to_prepare_seg_vec.clear();
  ego_to_prepare_seg_vec.reserve(6);
  // search min dist of ego corner to obs, which is used the minimal value with
  // 0.36 as buffer
  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(
      calc_params_.lat_outside_slot_buffer_vec[0],
      calc_params_.lat_outside_slot_buffer_vec[1], false));

  pnc::geometry_lib::PathSegment line_seg;
  if (OneLinePlan(line_seg, ego_pose, prepare_line)) {
    ego_to_prepare_seg_vec.emplace_back(line_seg);
    return true;
  }

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  std::vector<std::vector<pnc::geometry_lib::PathSegment>> path_vec;
  std::vector<std::vector<pnc::geometry_lib::PathSegment>> tmp_path_vec;
  const uint8_t ref_gear = geometry_lib::SEG_GEAR_INVALID;
  const uint8_t ref_steer = geometry_lib::SEG_STEER_INVALID;
  const double ref_radius = apa_param.GetParam().min_turn_radius + 0.3;

  if (!TwoArcPath(path_vec, ego_pose, prepare_line, ref_gear, ref_radius,
                  kColBufferOutSlot)) {
    if (!LineArcPlan(path_vec, ego_pose, prepare_line, ref_gear, ref_steer,
                     ref_radius, kColBufferOutSlot)) {
    }
  }
  pnc::dubins_lib::DubinsLibrary::Input dubins_input;
  dubins_input.radius = ref_radius;
  dubins_input.Set(ego_pose.pos, prepare_line.pA, ego_pose.heading,
                   prepare_line.heading);
  dubins_planner_.SetInput(dubins_input);

  // for (size_t i = 0; i < pnc::dubins_lib::DubinsLibrary::LINEARC_TYPE_COUNT;
  //      ++i) {
  //   if (!dubins_planner_.Solve(i)) {
  //     // DEBUG_PRINT("solve line arc failed!");
  //     continue;
  //   }

  //   if (!pnc::mathlib::IsInBound(
  //           dubins_planner_.GetOutput().line_arc_radius,
  //           apa_param.GetParam().min_turn_radius - 1e-6, 35.0)) {
  //     DEBUG_PRINT("arc not in bound! << "
  //                 << dubins_planner_.GetOutput().line_arc_radius);
  //     continue;
  //   }

  //   if (IsDubinsCollided(kColBufferOutSlot)) {
  //     DEBUG_PRINT("line arc collided!");
  //     continue;
  //   }

  //   GetPathSegVecByDubins(tmp_path_seg_vec);
  //   path_vec.emplace_back(tmp_path_seg_vec);

  // }  // linearc loop

  // try dubins method
  for (size_t i = 0; i < pnc::dubins_lib::DubinsLibrary::CASE_COUNT; ++i) {
    for (size_t j = 0; j < pnc::dubins_lib::DubinsLibrary::DUBINS_TYPE_COUNT;
         ++j) {
      if (!dubins_planner_.Solve(j, i)) {
        continue;
      }
      if (IsDubinsCollided(kColBufferOutSlot)) {
        continue;
      }
      GetPathSegVecByDubins(tmp_path_seg_vec);
      path_vec.emplace_back(tmp_path_seg_vec);
    }
  }  // dubins loop

  std::vector<GeometryPath> geo_path_vec;
  DEBUG_PRINT("path_vec size = " << path_vec.size());
  for (const auto& path : path_vec) {
    GeometryPath tmp_geo_path;
    if (AssempleGeometryPath(tmp_geo_path, path)) {
      geo_path_vec.emplace_back(tmp_geo_path);
    }
  }

  DEBUG_PRINT("geo_path_vec size = " << geo_path_vec.size());
  if (geo_path_vec.size() == 0) {
    return false;
  }

  size_t best_path_idx = std::numeric_limits<size_t>::max();
  if (!SelectBestPathOutsideSlot(geo_path_vec, best_path_idx)) {
    return false;
  }

  ego_to_prepare_seg_vec = geo_path_vec[best_path_idx].path_segment_vec;
  return true;
}

const std::vector<double> ParallelPathPlanner::GetMinDistOfEgoToObs() {
  const auto& car_vertex_x_vec = apa_param.GetParam().car_vertex_x_vec;
  const auto& car_vertex_y_vec = apa_param.GetParam().car_vertex_y_vec;

  const std::vector<Eigen::Vector2d> left_corner = {
      Eigen::Vector2d(car_vertex_x_vec[0], car_vertex_y_vec[0]),
      Eigen::Vector2d(car_vertex_x_vec[15], car_vertex_y_vec[15])};

  const std::vector<Eigen::Vector2d> right_corner = {
      Eigen::Vector2d(car_vertex_x_vec[5], car_vertex_y_vec[5]),
      Eigen::Vector2d(car_vertex_x_vec[10], car_vertex_y_vec[10])};

  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(input_.ego_pose.pos, input_.ego_pose.heading);

  std::vector<double> min_real_dist_vec = {2.0, 2.0};
  for (const auto& pt_pair : collision_detector_ptr_->GetObstaclesMap()) {
    const bool is_left_corner =
        (pt_pair.first == CollisionDetector::CHANNEL_OBS &&
         input_.tlane.slot_side_sgn > 0.0) ||
        (pt_pair.first != CollisionDetector::CHANNEL_OBS &&
         input_.tlane.slot_side_sgn < 0.0);
    const auto& corners = is_left_corner ? left_corner : right_corner;
    pnc::geometry_lib::LineSegment car_line(l2g_tf.GetPos(corners[0]),
                                            l2g_tf.GetPos(corners[1]));

    for (const auto& obs_pt : pt_pair.second) {
      const double real_dist =
          pnc::geometry_lib::CalPoint2LineSegDist(obs_pt, car_line);

      if (is_left_corner) {
        min_real_dist_vec[0] = std::min(min_real_dist_vec[0], real_dist);
      } else {
        min_real_dist_vec[1] = std::min(min_real_dist_vec[1], real_dist);
      }
    }
  }

  std::vector<double> min_buffer_vec = {kColLargeLatBufferOutSlot,
                                        kColLargeLatBufferOutSlot};
  for (size_t i = 0; i < min_real_dist_vec.size(); i++) {
    if (min_real_dist_vec[i] < kColLargeLatBufferOutSlot) {
      min_buffer_vec[i] =
          mathlib::Clamp(min_real_dist_vec[i] - 0.4, kColSmallLatBufferOutSlot,
                         kColLargeLatBufferOutSlot);
    } else if (min_real_dist_vec[i] < kColLargeLatBufferOutSlot + 0.3) {
      min_buffer_vec[i] =
          mathlib::Clamp(min_real_dist_vec[i] - 0.3, kColSmallLatBufferOutSlot,
                         kColLargeLatBufferOutSlot);
    }
  }

  DEBUG_PRINT("left lat buffer = " << min_buffer_vec[0]);
  DEBUG_PRINT("right lat buffer = " << min_buffer_vec[1]);
  return min_buffer_vec;
}

const bool ParallelPathPlanner::GenAlignedPreparingLine(
    std::vector<double>& preparing_y_vec,
    const pnc::geometry_lib::PathPoint& ego_pose) {
  if (std::fabs(input_.ego_pose.heading) / 57.3 < 1.0) {
    if (input_.ego_pose.pos.x() > input_.tlane.slot_length ||
        std::fabs(input_.ego_pose.pos.y() - input_.tlane.obs_pt_inside.y()) >
            0.4) {
      preparing_y_vec.emplace_back(ego_pose.pos.y());
    }

    return true;
  }

  const double rac_tlane_bound =
      input_.tlane.obs_pt_inside.y() +
      calc_params_.slot_side_sgn * (0.5 * apa_param.GetParam().car_width);

  const double rac_channel_bound =
      input_.tlane.channel_y -
      calc_params_.slot_side_sgn * (0.5 * apa_param.GetParam().car_width);

  std::vector<uint8_t> ref_gear_vec = {pnc::geometry_lib::SEG_GEAR_REVERSE,
                                       pnc::geometry_lib::SEG_GEAR_DRIVE};

  for (const auto& ref_gear : ref_gear_vec) {
    DEBUG_PRINT("ref gear = " << static_cast<int>(ref_gear));

    std::vector<pnc::geometry_lib::PathSegment> aligned_path_seg_vec;

    collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.0));

    if (!AlignBodyPlan(aligned_path_seg_vec, ego_pose, 0.0, ref_gear)) {
      // DEBUG_PRINT("aligned plan failed!");
      continue;
    }

    const auto& aligned_pos = aligned_path_seg_vec.front().GetEndPos();

    if (!pnc::mathlib::IsInBound(aligned_pos.y(), rac_tlane_bound,
                                 rac_channel_bound)) {
      // DEBUG_PRINT("aligned_y = " << aligned_y << ", over bound!");
      continue;
    }

    if (aligned_pos.x() < input_.tlane.slot_length &&
        std::fabs(input_.ego_pose.pos.y() - input_.tlane.obs_pt_inside.y()) <
            0.4) {
      continue;
    }

    // if (CheckPathSegVecCollided(aligned_path_seg_vec, 0.3)) {
    //   DEBUG_PRINT("aligned plan collided!");
    //   continue;
    // }
    preparing_y_vec.emplace_back(aligned_pos.y());
  }

  return true;
}

const bool ParallelPathPlanner::GenPreparingLineVec(
    std::vector<double>& preparing_y_vec) {
  preparing_y_vec.clear();

  const double half_slot_width = 0.5 * input_.tlane.slot_width;

  const double tlane_outer_y =
      std::fabs(input_.tlane.obs_pt_inside.y()) > half_slot_width
          ? input_.tlane.obs_pt_inside.y()
          : half_slot_width * calc_params_.slot_side_sgn;

  const double rac_tlane_bound =
      tlane_outer_y +
      calc_params_.slot_side_sgn * (0.5 * apa_param.GetParam().car_width + 0.4);

  const double rac_channel_bound =
      input_.tlane.channel_y -
      calc_params_.slot_side_sgn * (0.5 * apa_param.GetParam().car_width + 0.4);

  if (calc_params_.slot_side_sgn * (rac_channel_bound - rac_tlane_bound) <
      0.0) {
    return false;
  }

  double dy = 0.3;
  const double start_y = rac_tlane_bound + 0.2 * calc_params_.slot_side_sgn;
  double end_y = (1.2 + 0.5 * apa_param.GetParam().car_width + 1.8) *
                 calc_params_.slot_side_sgn;

  if (std::fabs(end_y) > rac_channel_bound) {
    end_y = rac_channel_bound;
  }

  preparing_y_vec = pnc::geometry_lib::Linspace(start_y, end_y, dy);
  preparing_y_vec.emplace_back(rac_tlane_bound);
  preparing_y_vec.emplace_back(rac_tlane_bound +
                               0.1 * calc_params_.slot_side_sgn);

  return true;
}

const bool ParallelPathPlanner::SelectBestPathOutsideSlot(
    const std::vector<GeometryPath>& path_vec, size_t& best_path_idx) {
  best_path_idx = std::numeric_limits<size_t>::max();

  if (path_vec.size() == 0) {
    DEBUG_PRINT("no path in path pool!");
    return false;
  }

  // for min gear
  std::vector<std::size_t> index_vec;
  size_t min_gear_shift_cnt = std::numeric_limits<size_t>::max();
  for (size_t i = 0; i < path_vec.size(); i++) {
    if (path_vec[i].gear_change_count < min_gear_shift_cnt) {
      min_gear_shift_cnt = path_vec[i].gear_change_count;
      index_vec.clear();
      index_vec.emplace_back(i);
    } else if (path_vec[i].gear_change_count == min_gear_shift_cnt) {
      index_vec.emplace_back(i);
    }
  }

  if (index_vec.size() == 0) {
    DEBUG_PRINT("calc min gear shift cnt error!");
  }
  DEBUG_PRINT("min_gear_shift_cnt = " << min_gear_shift_cnt);

  // for min length
  double min_length = std::numeric_limits<double>::max();
  for (const auto idx : index_vec) {
    const auto& path = path_vec[idx];
    DEBUG_PRINT("path length = " << path.length);
    if (path.length < min_length) {
      min_length = path.length;
      best_path_idx = idx;
    }
  }

  if (best_path_idx >= path_vec.size()) {
    return false;
  }
  return true;
}

const bool ParallelPathPlanner::AssempleGeometryPath(
    GeometryPath& geometry_path,
    const std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  geometry_path.Reset();

  if (path_seg_vec.size() == 0) {
    return false;
  }

  geometry_path.path_segment_vec = path_seg_vec;

  for (size_t i = 0; i < path_seg_vec.size(); i++) {
    geometry_path.length += path_seg_vec[i].Getlength();
    geometry_path.gear_cmd_vec.emplace_back(path_seg_vec[i].seg_gear);
    if (i > 0 && path_seg_vec[i].seg_gear != path_seg_vec[i - 1].seg_gear) {
      geometry_path.gear_change_count++;
    }
  }

  for (const auto& path_seg : path_seg_vec) {
    if (path_seg.seg_gear == path_seg_vec.front().seg_gear) {
      geometry_path.first_path_length += path_seg.Getlength();
    } else {
      break;
    }
  }
  return true;
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
  // std::cout << "last_arc.pB: x y :" << last_arc.pB.transpose() <<
  // std::endl;
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

  auto col_res =
      collision_detector_ptr_->UpdateByObsMap(last_arc, last_arc.headingA);
  if (col_res.collision_flag ||
      col_res.remain_car_dist >
          col_res.remain_obstacle_dist - kLonBufferTrippleStep) {
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
    // std::cout << "path_seg.Getlength()" << path_seg.Getlength() <<
    // std::endl;
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

void ParallelPathPlanner::GetPathSegVecByDubins(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  path_seg_vec.clear();
  path_seg_vec.reserve(3);

  const auto& dubins_output = dubins_planner_.GetOutput();

  if (dubins_output.gear_cmd_vec[0] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    pnc::geometry_lib::PathSegment seg_AB(
        pnc::geometry_lib::CalArcSteer(dubins_output.arc_AB),
        pnc::geometry_lib::CalArcGear(dubins_output.arc_AB),
        dubins_output.arc_AB);

    path_seg_vec.emplace_back(std::move(seg_AB));
  }

  // seg line BC
  if (dubins_output.gear_cmd_vec[1] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    pnc::geometry_lib::PathSegment seg_BC(
        pnc::geometry_lib::CalLineSegGear(dubins_output.line_BC),
        dubins_output.line_BC);

    path_seg_vec.emplace_back(std::move(seg_BC));
  }

  // set arc CD
  if (dubins_output.gear_cmd_vec[2] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    pnc::geometry_lib::PathSegment seg_CD(
        pnc::geometry_lib::CalArcSteer(dubins_output.arc_CD),
        pnc::geometry_lib::CalArcGear(dubins_output.arc_CD),
        dubins_output.arc_CD);

    path_seg_vec.emplace_back(std::move(seg_CD));
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
      col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
    } else if (!is_arc && !line.is_ignored) {
      col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
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
  return input_.slot_occupied_ratio > 0.0;
}

// search from the inside parking space to outside
const bool ParallelPathPlanner::CalMinSafeCircle() {
  const auto time0 = IflyTime::Now_ms();

  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.1, false));

  std::vector<pnc::geometry_lib::PathSegment> tra_search_out_res;
  const bool success_tra =
      InverseSearchLoopInSlot(tra_search_out_res, calc_params_.target_pose);
  debug_info_.tra_search_out_res = tra_search_out_res;
  const auto time1 = IflyTime::Now_ms();

  std::vector<pnc::geometry_lib::PathSegment> adv_search_out_res;
  const bool success_adv = AdvancedInversedTrialsInSlot(
      adv_search_out_res, calc_params_.target_pose);
  const auto time2 = IflyTime::Now_ms();

  DEBUG_PRINT("Traditional vs Advanced Method");
  // heading
  double tra_heading_deg = 0.0;
  double opt_heading_deg = 0.0;

  double tra_back_x = 999.9;
  double adv_back_x = 999.9;

  size_t tra_shifting_cnt = 0;
  size_t adv_shifting_cnt = 0;

  if (tra_search_out_res.size() > 0) {
    tra_heading_deg = tra_search_out_res.back().GetStartHeading() * 57.3;

    tra_back_x = tra_search_out_res.front().GetEndPos().x();

    tra_shifting_cnt = CalPathGearChangeCounts(tra_search_out_res);
  }

  if (adv_search_out_res.size() > 0) {
    opt_heading_deg = adv_search_out_res.back().GetStartHeading() * 57.3;

    adv_back_x = adv_search_out_res.front().GetEndPos().x();
    if (adv_search_out_res.size() >= 2) {
      if (adv_search_out_res[0].seg_gear == adv_search_out_res[1].seg_gear &&
          adv_search_out_res[0].seg_gear ==
              pnc::geometry_lib::SEG_GEAR_REVERSE) {
        adv_back_x = adv_search_out_res[1].GetEndPos().x();
      }
    }

    adv_shifting_cnt = CalPathGearChangeCounts(adv_search_out_res);
  }

  DEBUG_PRINT("park out heading(deg) = " << tra_heading_deg << " vs "
                                         << opt_heading_deg);

  DEBUG_PRINT("first backward limit x(m) = " << tra_back_x << " vs "
                                             << adv_back_x);

  DEBUG_PRINT("gear change cnt " << tra_shifting_cnt << " vs "
                                 << adv_shifting_cnt);

  DEBUG_PRINT("Time cost (ms) =" << time1 - time0 << " vs " << time2 - time1);

  DEBUG_PRINT("apa_param.GetParam().is_parallel_advanced_method = "
              << apa_param.GetParam().is_parallel_advanced_method);

  std::vector<pnc::geometry_lib::PathSegment> search_out_res;

  if (!apa_param.GetParam().is_parallel_advanced_method) {
    if (success_tra) {
      search_out_res = tra_search_out_res;
    } else {
      return false;
    }
  }

  if (apa_param.GetParam().is_parallel_advanced_method) {
    if (success_adv) {
      search_out_res = adv_search_out_res;
    } else {
      return false;
    }
  }

  if (search_out_res.size() == 0) {
    DEBUG_PRINT("search_out_res size = 0");
    return false;
  }

  ReduceRootPoseHeadingInSlot(search_out_res);

  // calc pose with ego had just cross pt_inside'y, which is used to
  // connect  the prepare point

  CalcParkOutPose(search_out_res.back());
  calc_params_.park_out_path_in_slot = search_out_res;
  calc_params_.park_out_pose = search_out_res.back().GetEndPose();
  calc_params_.safe_circle_root_pose = search_out_res.back().GetStartPose();
  calc_params_.valid_target_pt_vec.emplace_back(
      calc_params_.safe_circle_root_pose);
  pnc::geometry_lib::PrintPose("corrected multi root pose",
                               calc_params_.safe_circle_root_pose);
  pnc::geometry_lib::PrintPose("corrected park out pose",
                               calc_params_.park_out_pose);

  return true;
}

const bool ParallelPathPlanner::ReduceRootPoseHeadingInSlot(
    std::vector<pnc::geometry_lib::PathSegment>& search_out_res) {
  const size_t step_length = search_out_res.size();
  if (step_length == 0) {
    DEBUG_PRINT("found no parkout path!");
    return false;
  }

  if (std::fabs(search_out_res.back().GetStartHeading()) <
      kMaxParkOutRootHeading / 57.3) {
    DEBUG_PRINT(
        "park out heading is small enough, no need to correct heading!");
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

    pnc::geometry_lib::Arc forward_arc;
    forward_arc.pA = backward_line.pB;
    forward_arc.headingA = backward_line.heading;
    forward_arc.circle_info.radius = apa_param.GetParam().min_turn_radius;
    const uint8_t forward_steer = search_out_res.back().seg_steer;

    if (!CalcArcStepLimitPose(forward_arc, pnc::geometry_lib::SEG_GEAR_DRIVE,
                              forward_steer, kColBufferInSlot)) {
      return false;
    }

    if (!CheckParkOutCornerSafeWithObsPin(forward_arc)) {
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

const bool ParallelPathPlanner::ReversePathSegVec(
    std::vector<pnc::geometry_lib::PathSegment>& park_out_res) {
  if (park_out_res.size() <= 1) {
    return false;
  }
  park_out_res.pop_back();
  for (auto& path_seg : park_out_res) {
    pnc::geometry_lib::ReversePathSegInfo(path_seg);
  }
  std::reverse(park_out_res.begin(), park_out_res.end());
  return true;
}

const bool ParallelPathPlanner::InverseSearchLoopInSlot(
    std::vector<pnc::geometry_lib::PathSegment>& search_out_res,
    const pnc::geometry_lib::PathPoint& terminal_pose) {
  search_out_res.clear();
  search_out_res.reserve(10);
  const double radius = apa_param.GetParam().min_turn_radius;

  pnc::geometry_lib::Arc forward_arc;
  forward_arc.pA = terminal_pose.pos;
  forward_arc.headingA = terminal_pose.heading;
  forward_arc.circle_info.radius = radius;

  const uint8_t forward_steer =
      (calc_params_.is_left_side ? pnc::geometry_lib::SEG_STEER_RIGHT
                                 : pnc::geometry_lib::SEG_STEER_LEFT);

  if (CalcArcStepLimitPose(forward_arc, pnc::geometry_lib::SEG_GEAR_DRIVE,
                           forward_steer, kColBufferInSlot)) {
    if (CheckParkOutCornerSafeWithObsPin(forward_arc)) {
      search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
          forward_steer, pnc::geometry_lib::SEG_GEAR_DRIVE, forward_arc));
      std::cout << "ego can park out at first!" << std::endl;
      return true;
    }
  }

  // calc backward limit
  pnc::geometry_lib::LineSegment first_line_step;
  first_line_step.pA = terminal_pose.pos;
  first_line_step.heading = terminal_pose.heading;
  if (!CalcLineStepLimitPose(first_line_step,
                             pnc::geometry_lib::SEG_GEAR_REVERSE,
                             kColBufferInSlot)) {
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
  }

  bool loop_success = false;
  auto start_pose_in_loop = back_line_limit;
  const auto backward_steer = pnc::geometry_lib::ReverseSteer(forward_steer);
  for (size_t i = 0; i < kMaxPathNumsInSlot; i += 2) {
    pnc::geometry_lib::Arc forward_arc;
    forward_arc.pA = start_pose_in_loop.pos;
    forward_arc.headingA = start_pose_in_loop.heading;
    forward_arc.circle_info.radius = radius;

    if (!CalcArcStepLimitPose(forward_arc, pnc::geometry_lib::SEG_GEAR_DRIVE,
                              forward_steer, kColBufferInSlot)) {
      std::cout << "calc forward arc limit error!" << std::endl;
      break;
    }

    search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
        forward_steer, pnc::geometry_lib::SEG_GEAR_DRIVE, forward_arc));

    // ego can park out in forward step, return true
    if (CheckParkOutCornerSafeWithObsPin(forward_arc)) {
      loop_success = true;
      DEBUG_PRINT("find park out pose!");
      break;
    }

    // DEBUG_PRINT("forward limit pose is valid\n");
    // DEBUG_PRINT("------ backward step --------");

    pnc::geometry_lib::Arc backward_arc;
    backward_arc.pA = forward_arc.pB;
    backward_arc.headingA = forward_arc.headingB;
    backward_arc.circle_info.radius = radius;

    if (!CalcArcStepLimitPose(backward_arc, pnc::geometry_lib::SEG_GEAR_REVERSE,
                              backward_steer, kColBufferInSlot)) {
      std::cout << "calc backward arc limit error!" << std::endl;
      break;
    }

    // start calc backward limit pose
    pnc::geometry_lib::PathPoint backward_limit_pose(backward_arc.pB,
                                                     backward_arc.headingB);

    // std::cout << "backward limit pose = ";
    // pnc::geometry_lib::PrintPose(backward_arc.pB, backward_arc.headingB);
    // DEBUG_PRINT("arc length = " << backward_arc.length);

    search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
        backward_steer, pnc::geometry_lib::SEG_GEAR_REVERSE, backward_arc));

    start_pose_in_loop = backward_limit_pose;
  }

  if (!loop_success) {
    search_out_res.clear();
  }

  return loop_success;
}

const bool ParallelPathPlanner::AdvancedInversedTrialsInSlot(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& target_pose) {
  DEBUG_PRINT("---------AdvancedInversedTrialsInSlot --------------");
  using namespace pnc::geometry_lib;

  std::vector<Eigen::Vector2d> line_step_vec;
  if (!GenLineStepValidEnd(line_step_vec, target_pose)) {
    DEBUG_PRINT("cal first step failed!");
    return false;
  }

  std::vector<GeometryPath> total_path_vec;
  total_path_vec.reserve(line_step_vec.size() * 2);

  pnc::geometry_lib::PathPoint start_pose(line_step_vec.front(),
                                          target_pose.heading);
  uint total_cnt = 0;
  uint success_cnt = 0;
  uint dirve_success_cnt = 0;
  uint reverse_success_cnt = 0;

  uint rear_limit_fail_cnt = 0;
  uint front_limit_fail_cnt = 0;
  uint calc_fail_cnt = 0;
  std::vector<double> success_x_vec;
  std::vector<uint8_t> gear_vec = {SEG_GEAR_REVERSE, SEG_GEAR_DRIVE};

  size_t debug_path_idx = 0;
  // size_t cal_cnt = 0;
  for (const auto& gear : gear_vec) {
    for (const auto& start_pos : line_step_vec) {
      total_cnt++;
      // cal_cnt++;
      // if (cal_cnt > 1) {
      //   break;
      // }

      if ((start_pos - line_step_vec.front()).norm() <= 0.1 &&
          gear == SEG_GEAR_REVERSE) {
        rear_limit_fail_cnt++;
        continue;
      }

      if ((start_pos - line_step_vec.back()).norm() <= 0.1 &&
          gear == SEG_GEAR_REVERSE) {
        front_limit_fail_cnt++;
        continue;
      }

      start_pose.pos = start_pos;
      std::vector<PathSegment> path_seg_vec;
      if (!InversedTrialsByGivenGear(path_seg_vec, start_pose, gear)) {
        // DEBUG_PRINT("InversedTrialsByGivenGear failed with x = "
        //             << start_pose.pos.x()
        //             << "gear = " << static_cast<int>(gear));
        calc_fail_cnt++;
        continue;
      }

      const LineSegment first_line(target_pose.pos, start_pose.pos,
                                   target_pose.heading);

      double first_path_length = first_line.length;
      success_x_vec.emplace_back(start_pos.x());
      if (first_path_length > 1e-5) {
        pnc::geometry_lib::PathSegment first_line_path_seg(
            CalLineSegGear(first_line), first_line);

        path_seg_vec.insert(path_seg_vec.begin(), first_line_path_seg);

        if (mathlib::IsDoubleEqual(start_pos.x(), line_step_vec.front().x())) {
          DEBUG_PRINT("calc back limit path success! " << start_pos.x());
          debug_path_idx = total_path_vec.size();
        }
      }

      GeometryPath geo_path;
      AssempleGeometryPath(geo_path, path_seg_vec);
      if (geo_path.first_path_length < 0.2) {
        DEBUG_PRINT("first path length is too short!");
        continue;
      }
      DEBUG_PRINT("geo_path gear change cnt = " << geo_path.gear_change_count);
      total_path_vec.emplace_back(geo_path);
      success_cnt++;
      gear == SEG_GEAR_DRIVE ? ++dirve_success_cnt : ++reverse_success_cnt;
    }
    // if (cal_cnt > 1) {
    //   break;
    // }
  }

  DEBUG_PRINT("total_cnt = " << total_cnt);
  DEBUG_PRINT("success_cnt = " << success_cnt);
  DEBUG_PRINT("dirve_success_cnt = " << dirve_success_cnt);
  DEBUG_PRINT("reverse_success_cnt = " << reverse_success_cnt);
  DEBUG_PRINT("rear_limit_fail_cnt = " << rear_limit_fail_cnt);
  DEBUG_PRINT("front_limit_fail_cnt = " << front_limit_fail_cnt);
  DEBUG_PRINT("calc fail cnt = " << calc_fail_cnt);
  DEBUG_PRINT("success x vec -------");
  for (const auto x : success_x_vec) {
    std::cout << x << ", ";
  }
  std::cout << std::endl;

  std::vector<size_t> shifting_vec;
  std::vector<double> length_vec;
  size_t min_shifting_cnt = 10;

  for (auto& path_info : total_path_vec) {
    const auto& path = path_info.path_segment_vec;

    path_info.park_out_heading_deg =
        std::fabs(path.back().GetStartHeading() * 57.3);

    min_shifting_cnt = std::min(min_shifting_cnt, path_info.gear_change_count);
  }

  std::vector<size_t> min_gear_idx_vec;
  for (size_t i = 0; i < total_path_vec.size(); i++) {
    if (total_path_vec[i].gear_change_count == min_shifting_cnt) {
      min_gear_idx_vec.emplace_back(i);
    }
  }
  DEBUG_PRINT("path size =  " << min_gear_idx_vec.size()
                              << " after gear filtered!");

  size_t best_idx = kInvalidInteger;
  double min_heading = 66.6;
  bool success = false;

  for (size_t i = 0; i < min_gear_idx_vec.size(); i++) {
    size_t idx = min_gear_idx_vec[i];
    if (total_path_vec[idx].park_out_heading_deg < min_heading) {
      success = true;
      min_heading = total_path_vec[idx].park_out_heading_deg;
      best_idx = idx;
    }
  }

  if (success) {
    path_seg_vec = total_path_vec[best_idx].path_segment_vec;

    // // for path debug
    // path_seg_vec = total_path_vec[debug_path_idx].path_segment_vec;
  }

  return success;
}

const size_t ParallelPathPlanner::CalPathGearChangeCounts(
    const std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  size_t shifting_cnt = 0;
  for (size_t j = 0; j < path_seg_vec.size(); j++) {
    if (j > 0) {
      if (path_seg_vec[j].seg_gear != path_seg_vec[j - 1].seg_gear) {
        shifting_cnt++;
      }
    }
  }
  return shifting_cnt;
}

const bool ParallelPathPlanner::GenLineStepValidEnd(
    std::vector<Eigen::Vector2d>& line_end_vec,
    const pnc::geometry_lib::PathPoint& target_pose) {
  using namespace pnc::geometry_lib;

  line_end_vec.clear();
  line_end_vec.reserve(10);

  const auto v_heading = GetUnitTangVecByHeading(target_pose.heading);
  PrintPose("terminal pose", target_pose);

  std::vector<Eigen::Vector2d> line_step_vec;
  line_step_vec.emplace_back(target_pose.pos);

  std::vector<uint8_t> gear_vec = {SEG_GEAR_REVERSE, SEG_GEAR_DRIVE};

  pnc::geometry_lib::Arc forward_arc;
  forward_arc.pA = target_pose.pos;
  forward_arc.headingA = target_pose.heading;
  forward_arc.circle_info.radius = apa_param.GetParam().min_turn_radius;

  const uint8_t forward_steer =
      (calc_params_.is_left_side ? pnc::geometry_lib::SEG_STEER_RIGHT
                                 : pnc::geometry_lib::SEG_STEER_LEFT);

  if (CalcArcStepLimitPose(forward_arc, pnc::geometry_lib::SEG_GEAR_DRIVE,
                           forward_steer, kColBufferInSlot)) {
    if (CheckParkOutCornerSafeWithObsPin(forward_arc)) {
      gear_vec.erase(gear_vec.begin());
      DEBUG_PRINT("ego can park out at first, no need use backward loop!");
    }
  }

  auto first_line = GetEgoHeadingLine(target_pose.pos, target_pose.heading);
  for (const auto& gear : gear_vec) {
    if (!CalcLineStepLimitPose(first_line, gear, kColBufferInSlot)) {
      DEBUG_PRINT("gear = " << gear << " failed!");
      continue;
    }

    if (CheckSamePos(first_line.pB, target_pose.pos)) {
      DEBUG_PRINT("first_line limit == target pose failed!");
      continue;
    }

    double line_length = (first_line.pB - target_pose.pos).norm();
    if (line_length < 0.2) {
      DEBUG_PRINT("line_length" << line_length
                                << "smaller than min line length! gear == "
                                << static_cast<int>(gear));
      continue;
    }

    int step_size = static_cast<int>(line_length / kLineStepLength);
    int max_size = static_cast<int>(line_length / 0.1);
    if (gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      line_length = std::min(line_length, 0.6);
      step_size = mathlib::Clamp(step_size, 2, 3);
      step_size = std::min(step_size, max_size);

    } else {
      line_length = std::min(line_length, 0.8);
      step_size = mathlib::Clamp(step_size, 2, 4);
      step_size = std::min(step_size, max_size);
    }

    const double step = line_length / step_size;
    double length_diff = step;
    const double dir_sgn = (gear == SEG_GEAR_DRIVE ? 1.0 : -1.0);

    while (length_diff <= line_length) {
      const auto line_end_pos =
          target_pose.pos + dir_sgn * length_diff * v_heading;

      line_step_vec.emplace_back(std::move(line_end_pos));
      length_diff += step;
    }

    if (std::fabs(line_step_vec.back().x() - first_line.pB.x()) < 0.36 * step) {
      line_step_vec.pop_back();
    }
    line_step_vec.emplace_back(first_line.pB);
  }

  bool success = false;
  if (line_step_vec.size() != 0) {
    success = true;
    auto comp = [](const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
      return v1.x() < v2.x();
    };
    std::sort(line_step_vec.begin(), line_step_vec.end(), comp);
    line_end_vec = std::move(line_step_vec);

    // debug for line step pos
    DEBUG_PRINT("line step pos ");
    for (const auto& pos : line_end_vec) {
      std::cout << pos.x() << "  ";
    }
    std::cout << std::endl;
  }

  return success;
}

const bool ParallelPathPlanner::InversedTrialsByGivenGear(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const uint8_t current_gear) {
  using namespace pnc::geometry_lib;
  // DEBUG_PRINT("InversedTrialsByGivenGear, start_pos"
  //             << start_pose.pos.transpose());
  uint8_t ref_gear = current_gear;
  uint8_t ref_steer = SEG_STEER_RIGHT;
  if ((ref_gear == SEG_GEAR_DRIVE && input_.tlane.slot_side_sgn > 0.0) ||
      (ref_gear == SEG_GEAR_REVERSE && input_.tlane.slot_side_sgn < 0.0)) {
    ref_steer = SEG_STEER_LEFT;
  }

  // DEBUG_PRINT("slot side sgn = " << input_.tlane.slot_side_sgn);
  // DEBUG_PRINT("current_gear = " << static_cast<int>(ref_gear));
  // DEBUG_PRINT("current_steer = " << static_cast<int>(ref_steer));

  // check if ego is able to park out at start pose
  pnc::geometry_lib::Arc arc;
  arc.pA = start_pose.pos;
  arc.headingA = start_pose.heading;
  arc.circle_info.radius = apa_param.GetParam().min_turn_radius;

  std::vector<pnc::geometry_lib::PathSegment> search_out_res;
  search_out_res.reserve(kMaxParallelParkInSegmentNums);

  bool success = false;
  for (size_t i = 0; i <= kMaxPathNumsInSlot + 1; i += 1) {
    if (!CalcArcStepLimitPose(arc, ref_gear, ref_steer, kColBufferInSlot)) {
      // DEBUG_PRINT("calc arc limit error!");
      break;
    }

    search_out_res.emplace_back(
        pnc::geometry_lib::PathSegment(ref_steer, ref_gear, arc));

    if (ref_gear == SEG_GEAR_DRIVE && CheckParkOutCornerSafeWithObsPin(arc)) {
      // DEBUG_PRINT("is_dirve_out_safe success");
      success = true;
      break;
    }

    // reverse for next loop
    arc.pA = arc.pB;
    arc.headingA = arc.headingB;
    arc.circle_info.radius = apa_param.GetParam().min_turn_radius;
    ref_gear = ReverseGear(ref_gear);
    ref_steer = ReverseSteer(ref_steer);
  }

  if (success) {
    path_seg_vec = search_out_res;
  }

  return success;
}

const bool ParallelPathPlanner::CalcLineStepLimitPose(
    pnc::geometry_lib::LineSegment& line, const uint8_t gear,
    const double buffer) {
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

  const double line_dist_limit = input_.slot_occupied_ratio > 0.0 ? 1.5 : 4.0;

  const Eigen::Vector2d rough_limit_pt =
      start_pose.pos + line_dist_limit * dirve_sgn *
                           Eigen::Vector2d(std::cos(start_pose.heading),
                                           std::sin(start_pose.heading));

  pnc::geometry_lib::PathSegment line_path(
      gear, pnc::geometry_lib::LineSegment(start_pose.pos, rough_limit_pt,
                                           start_pose.heading));

  const auto& col_res = TrimPathByCollisionDetection(line_path, buffer);

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
  return (!line.is_ignored);
}

const bool ParallelPathPlanner::CalcArcStepLimitPose(
    pnc::geometry_lib::Arc& arc, const uint8_t gear, const uint8_t steer,
    const double buffer) {
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

  pnc::geometry_lib::PathPoint start_pose(arc.pA, arc.headingA);

  arc.circle_info.center =
      CalEgoTurningCenter(start_pose, arc.circle_info.radius, steer);

  if (!pnc::geometry_lib::CalcArcDirection(arc.is_anti_clockwise, gear,
                                           steer)) {
    std::cout << "arc or steer error!" << std::endl;
    return false;
  }

  const double arc_length_limit = input_.slot_occupied_ratio > 0.0 ? 3.5 : 4.5;
  if (!pnc::geometry_lib::CompleteArcInfo(arc, arc_length_limit,
                                          arc.is_anti_clockwise)) {
    return false;
  }

  pnc::geometry_lib::PathSegment arc_path(steer, gear, arc);
  const auto& col_res = TrimPathByCollisionDetection(arc_path, buffer);
  // std::cout << "forward_col_pt =" << forward_col_pt.transpose() <<
  // std::endl;

  arc.is_ignored = true;
  // get updated arc info of collision free
  if (col_res == PATH_COL_NORMAL || col_res == PATH_COL_SHORTEN) {
    if (arc_path.GetArcSeg().length >= 0.1) {
      arc.is_ignored = false;
      arc.pB = arc_path.GetArcSeg().pB;
      arc.headingB = arc_path.GetArcSeg().headingB;

      const double trim_theta_diff =
          pnc::geometry_lib::NormalizeAngle(arc.headingB - arc.headingA);

      arc.length = std::fabs(trim_theta_diff) * arc.circle_info.radius;
    }
  }

  if (arc.is_ignored) {
    arc.pB = arc.pA;
    arc.headingB = arc.headingA;
    arc.length = 0.0;
  }

  return (!arc.is_ignored);
}

const bool ParallelPathPlanner::CheckParkOutCornerSafeWithObsPin(
    const pnc::geometry_lib::Arc& first_arc) const {
  double center_to_obs_in = 100.0;

  // DEBUG_PRINT("center_to_obs_in = " << center_to_obs_in);
  // DEBUG_PRINT(
  //     "max_corner_radius = " <<
  //     calc_params_.min_outer_front_corner_radius);
  for (const auto& virtual_obs_pt : calc_params_.front_corner_obs_vec) {
    center_to_obs_in =
        std::min(center_to_obs_in,
                 (virtual_obs_pt - first_arc.circle_info.center).norm());
  }
  // DEBUG_PRINT("actual corner safe dist = "
  //             << center_to_obs_in -
  //             calc_params_.min_outer_front_corner_radius);

  return (center_to_obs_in >=
          calc_params_.min_outer_front_corner_radius +
              apa_param.GetParam().parallel_ego_front_corner_to_obs_in_buffer);
}

const bool ParallelPathPlanner::TwoSameGearArcPlanToLine(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::LineSegment& target_line, const uint8_t gear,
    const double buffer) {
  return TwoSameGearArcPlanToLine(path_seg_vec, start_pose, target_line, gear,
                                  apa_param.GetParam().min_turn_radius, buffer);
}

const bool ParallelPathPlanner::TwoSameGearArcPlanToLine(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::LineSegment& target_line, const uint8_t gear,
    const double radius, const double buffer) {
  path_seg_vec.clear();
  path_seg_vec.reserve(4);

  if (!pnc::geometry_lib::IsValidGear(gear)) {
    DEBUG_PRINT("gear invalid!");
    return false;
  }

  pnc::geometry_lib::Arc arc_1;
  pnc::geometry_lib::Arc arc_2;
  arc_1.pA = start_pose.pos;
  arc_1.headingA = start_pose.heading;
  arc_1.circle_info.radius = radius;

  DEBUG_PRINT("target line =" << target_line.pA.transpose()
                              << ", heading deg = "
                              << target_line.heading * 57.3);

  const uint8_t arc_1_steer = (pnc::geometry_lib::IsPointOnLeftSideOfLineSeg(
                                   start_pose.pos, target_line)
                                   ? pnc::geometry_lib::SEG_STEER_RIGHT
                                   : pnc::geometry_lib::SEG_STEER_LEFT);

  arc_1.circle_info.center =
      CalEgoTurningCenter(start_pose, radius, arc_1_steer);

  auto tmp_target_line = target_line;
  if (!pnc::geometry_lib::CalTwoSameGearArcWithLine(arc_1, arc_2,
                                                    tmp_target_line, gear)) {
    DEBUG_PRINT("CalTwoSameGearArcWithLine failed!");
    return false;
  }

  auto col_res = collision_detector_ptr_->UpdateByObsMap(arc_1, arc_1.headingA);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
    DEBUG_PRINT("col pt = " << col_res.col_pt_obs_global.transpose());
    // debug_info_.debug_arc_vec.emplace_back(arc_1);
    DEBUG_PRINT("TwoSameGearArcPlanToLine arc1 collided!");
    return false;
  }
  col_res = collision_detector_ptr_->UpdateByObsMap(arc_2, arc_2.headingA);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
    DEBUG_PRINT("arc2 collided!");
    return false;
  }

  pnc::geometry_lib::LineSegment last_line(arc_2.pB, target_line.pA,
                                           arc_2.headingB);
  auto last_line_gear = pnc::geometry_lib::CalLineSegGear(last_line);
  col_res =
      collision_detector_ptr_->UpdateByObsMap(last_line, last_line.heading);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
    DEBUG_PRINT("arc2 end to line key pt collided!");
    return false;
  }

  bool is_prolonged = false;
  auto prolonged_line = last_line;
  if (last_line_gear != gear) {
    const double gear_sgn =
        (gear == pnc::geometry_lib::SEG_GEAR_DRIVE ? 1.0 : -1.0);

    const auto v_line_heading =
        pnc::geometry_lib::GetUnitTangVecByHeading(arc_2.headingB);

    const auto prolonged_pt =
        arc_2.pB + k1dExtendLength * v_line_heading * gear_sgn;
    prolonged_line.SetPoints(arc_2.pB, prolonged_pt);

    col_res = collision_detector_ptr_->UpdateByObsMap(prolonged_line,
                                                      prolonged_line.heading);
    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
      DEBUG_PRINT("arc 2 extend line collided!");
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

  // check pose and slot_occupied_ratio, if error is small, multi isn't
  // suitable
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
    success =
        CalSinglePathInMulti(current_pose, calc_params_.target_pose,
                             current_gear, current_arc_steer, tmp_path_seg_vec);
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
    const pnc::geometry_lib::PathPoint& target_pose, const uint8_t current_gear,
    const uint8_t current_arc_steer,
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

  auto target_line = pnc::geometry_lib::BuildLineSegByPose(target_pose.pos,
                                                           target_pose.heading);

  // cal the dist from current_turn_center to target line
  const double dist =
      pnc::geometry_lib::CalPoint2LineDist(current_turn_center, target_line);

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
    if (OneArcPlan(current_arc, tmp_path_seg_vec, target_line, current_gear,
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
    if (TwoArcPlan(current_arc, tmp_path_seg_vec, target_line, current_gear,
                   current_arc_steer)) {
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
    if (LineArcPlan(current_arc, tmp_path_seg_vec, target_line, current_gear,
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
  if (OneLinePlanAlongEgoHeading(last_line, target_pose)) {
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

  // check pose and slot_occupied_ratio, if error is small, multi isn't
  // suitable
  if (std::fabs(current_pose.heading * 57.3) <=
      apa_param.GetParam().finish_parallel_heading_err) {
    DEBUG_PRINT("body already aligned!");
    return false;
  }

  std::vector<pnc::geometry_lib::PathSegment> single_aligned_path;
  std::vector<pnc::geometry_lib::PathSegment> path_res;
  path_res.reserve(kMaxMultiStepNums);

  bool success = false;
  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.0));
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
  //   DEBUG_PRINT("pose err is relatively large, skip adjust plan, plan
  //   fail"); return false;
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

  if (OneLinePlan(first_line, calc_params_.target_pose)) {
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

const bool ParallelPathPlanner::OneArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    pnc::geometry_lib::LineSegment& target_line, const uint8_t current_gear,
    const uint8_t current_arc_steer) {
  if (!CalOneArcWithLine(arc, target_line,
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
    pnc::geometry_lib::LineSegment& target_line, const uint8_t current_gear) {
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
    pnc::geometry_lib::LineSegment& target_line, const uint8_t current_gear,
    const uint8_t current_arc_steer) {
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

const bool ParallelPathPlanner::TwoArcPath(
    std::vector<std::vector<pnc::geometry_lib::PathSegment>>& path_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::LineSegment& target_line, const uint8_t ref_gear,
    const double radius, const double lon_buffer) {
  // if ref_gear == invalid or cound  that mins ref_gear is not limited!

  using namespace pnc::geometry_lib;

  path_vec.clear();

  std::vector<std::pair<Arc, Arc>> arc_pair_vec;
  auto tmp_target_line = target_line;
  bool success = CalTwoArcWithLine(start_pose, tmp_target_line, radius, radius,
                                   arc_pair_vec);

  if (!success) {
    DEBUG_PRINT("CalTwoArcWithLine failed!");
    return false;
  }

  for (const auto& arc_pair : arc_pair_vec) {
    std::vector<PathSegment> path_seg_vec;
    path_seg_vec.reserve(3);

    Arc arc1 = arc_pair.first;
    Arc arc2 = arc_pair.second;

    if (!mathlib::IsDoubleEqual(arc2.headingB * 57.3,
                                tmp_target_line.heading * 57.3)) {
      continue;
    }

    const PathSegment path_seg_1(CalArcSteer(arc1), CalArcGear(arc1), arc1);

    if (IsValidGear(ref_gear) && path_seg_1.seg_gear != ref_gear) {
      continue;
    }

    auto col_res = collision_detector_ptr_->UpdateByObsMap(arc1, arc1.headingA);
    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist - lon_buffer) {
      continue;
    }

    col_res = collision_detector_ptr_->UpdateByObsMap(arc2, arc2.headingA);
    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist - lon_buffer) {
      continue;
    }

    LineSegment preparing_line(arc2.pB, target_line.pA, arc2.headingB);
    col_res = col_res = collision_detector_ptr_->UpdateByObsMap(
        preparing_line, preparing_line.heading);

    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist - lon_buffer) {
      continue;
    }

    path_seg_vec.emplace_back(
        PathSegment(CalArcSteer(arc1), CalArcGear(arc1), arc1));

    path_seg_vec.emplace_back(
        PathSegment(CalArcSteer(arc2), CalArcGear(arc2), arc2));

    path_seg_vec.emplace_back(
        PathSegment(CalLineSegGear(preparing_line), preparing_line));
    path_vec.emplace_back(path_seg_vec);
    success = true;
  }
  DEBUG_PRINT("path_vec size = " << path_vec.size());

  return success;
}

const bool ParallelPathPlanner::LineArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    pnc::geometry_lib::LineSegment& target_line, const uint8_t current_gear,
    const uint8_t current_arc_steer) {
  pnc::geometry_lib::LineSegment line_seg1;
  line_seg1 = pnc::geometry_lib::BuildLineSegByPose(arc.pA, arc.headingA);

  pnc::geometry_lib::LineSegment line_seg2;
  line_seg2 = target_line;

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
    tmp_arc.circle_info.center = centers[i];
    tmp_arc.circle_info.radius = arc.circle_info.radius;

    if (!geometry_lib::CompleteArcInfo(tmp_arc)) {
      continue;
    }

    if (!pnc::mathlib::IsDoubleEqual(tmp_arc.headingB * 57.3,
                                     line_seg2.heading * 57.3)) {
      continue;
    }

    const uint8_t tmp_arc_steer = pnc::geometry_lib::CalArcSteer(tmp_arc);
    if (geometry_lib::IsValidArcSteer(current_arc_steer) &&
        tmp_arc_steer != current_arc_steer) {
      continue;
    }

    const uint8_t tmp_gear = pnc::geometry_lib::CalArcGear(tmp_arc);
    if (geometry_lib::IsValidGear(current_gear) && tmp_gear != current_gear) {
      continue;
    }

    pnc::geometry_lib::LineSegment line(line_seg1.pA, tangent_ptss[i].first,
                                        line_seg1.heading);
    if (geometry_lib::IsValidGear(current_gear) &&
        pnc::geometry_lib::CalLineSegGear(line) != current_gear) {
      DEBUG_PRINT("line pA =" << line.pA.transpose());
      DEBUG_PRINT("line pB =" << line.pB.transpose());
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

const bool ParallelPathPlanner::LineArcPlan(
    std::vector<std::vector<geometry_lib::PathSegment>>& path_vec,
    const pnc::geometry_lib::PathPoint& ego_pose,
    const pnc::geometry_lib::LineSegment& target_line, const uint8_t ref_gear,
    const uint8_t ref_steer, const double radius, const double lon_buffer) {
  using namespace pnc::geometry_lib;

  auto first_line = BuildLineSegByPose(ego_pose.pos, ego_pose.heading);
  auto last_line = target_line;

  std::vector<Eigen::Vector2d> centers;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> tangent_ptss;
  if (!CalCommonTangentCircleOfTwoLine(first_line, last_line, radius, centers,
                                       tangent_ptss)) {
    std::cout << "LineArcPlan fail 0\n";
    return false;
  }

  bool success = false;
  // check which center and tang pt is suitable
  for (size_t i = 0; i < centers.size(); ++i) {
    Arc tmp_arc;
    tmp_arc.pA = tangent_ptss[i].first;
    tmp_arc.headingA = first_line.heading;
    tmp_arc.pB = tangent_ptss[i].second;
    tmp_arc.circle_info.center = centers[i];
    tmp_arc.circle_info.radius = radius;

    if (!CompleteArcInfo(tmp_arc)) {
      continue;
    }

    if (!pnc::mathlib::IsDoubleEqual(tmp_arc.headingB * 57.3,
                                     last_line.heading * 57.3)) {
      continue;
    }

    const PathSegment arc_seg(CalArcSteer(tmp_arc), CalArcGear(tmp_arc),
                              tmp_arc);

    if (IsValidArcSteer(ref_steer) && arc_seg.seg_steer != ref_steer) {
      continue;
    }

    first_line.SetPoints(first_line.pA, tangent_ptss[i].first);
    const PathSegment first_line_seg(CalLineSegGear(first_line), first_line);
    if (IsValidGear(ref_gear) && first_line_seg.seg_gear != ref_gear) {
      continue;
    }

    const LineSegment last_line(tmp_arc.pB, target_line.pA, tmp_arc.headingB);
    const PathSegment last_line_seg(CalLineSegGear(last_line), last_line);

    auto col_res =
        collision_detector_ptr_->UpdateByObsMap(first_line, first_line.heading);
    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist - lon_buffer) {
      continue;
    }

    col_res =
        collision_detector_ptr_->UpdateByObsMap(tmp_arc, tmp_arc.headingA);
    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist - lon_buffer) {
      continue;
    }

    col_res =
        collision_detector_ptr_->UpdateByObsMap(last_line, last_line.heading);
    if (col_res.collision_flag ||
        col_res.remain_car_dist > col_res.remain_obstacle_dist - lon_buffer) {
      continue;
    }

    const std::vector<PathSegment> path_seg_vec = {first_line_seg, arc_seg,
                                                   last_line_seg};
    path_vec.emplace_back(std::move(path_seg_vec));
    success = true;
  }

  return success;
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

  if (!pnc::geometry_lib::CalOneArcWithTargetHeading(arc, current_gear,
                                                     target_heading)) {
    return false;
  }

  // check if gear satisfies needs
  const auto steer = pnc::geometry_lib::CalArcSteer(arc);
  const auto gear = pnc::geometry_lib::CalArcGear(arc);
  if (gear != current_gear) {
    return false;
  }
  pnc::geometry_lib::PathSegment aligned_arc_seg(steer, gear, arc);
  path_seg_vec.emplace_back(aligned_arc_seg);
  return true;
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
    col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    auto& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
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
    std::cout << col_res.col_pt_ego_global.transpose() << std::endl;
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

const bool ParallelPathPlanner::CheckPathSegCollided(
    const pnc::geometry_lib::PathSegment& path_seg, const double buffer) const {
  CollisionDetector::CollisionResult col_res;
  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    auto& line = path_seg.line_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    auto& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
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
  // std::cout << "--- try one line plan ---\n";

  const pnc::geometry_lib::PathPoint start_pose(line.pA, line.heading);

  // std::cout << "line start pose =" << start_pose.pos.transpose()
  //           << ", heading(deg) =" << start_pose.heading * 57.3 << std::endl;

  auto target_line = pnc::geometry_lib::BuildLineSegByPose(target_pose.pos,
                                                           target_pose.heading);
  // std::cout << "target line pA =" << target_line.pA.transpose()
  //           << "heading(deg) =" << target_line.heading * 57.3 << std::endl;

  if (!pnc::geometry_lib::IsPoseOnLine(
          start_pose, target_line, apa_param.GetParam().static_pos_eps,
          apa_param.GetParam().static_heading_eps / 57.3)) {
    // std::cout << "pose is not on line, fail\n";
    return false;
  }
  // DEBUG_PRINT("pose is on line, success");

  line.SetPoints(start_pose.pos, target_line.pA);

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

const bool ParallelPathPlanner::OneLinePlan(
    pnc::geometry_lib::PathSegment& line_seg,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::LineSegment& target_line,
    const double lon_buffer) const {
  const pnc::geometry_lib::PathPoint target_pose(target_line.pA,
                                                 target_line.heading);
  auto line =
      pnc::geometry_lib::BuildLineSegByPose(start_pose.pos, start_pose.heading);

  if (!OneLinePlan(line, target_pose)) {
    return false;
  }

  const auto col_res =
      collision_detector_ptr_->UpdateByObsMap(line, line.heading);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist - lon_buffer) {
    return false;
  }

  line_seg = pnc::geometry_lib::PathSegment(
      pnc::geometry_lib::CalLineSegGear(line), line);
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

    if (path_len < apa_param.GetParam().min_path_length) {
      extend_distance = apa_param.GetParam().min_path_length;
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
    collision_result = collision_detector_ptr_->UpdateByObsMap(
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
      collision_result = collision_detector_ptr_->UpdateByObsMap(
          path_seg.line_seg, path_seg.line_seg.heading);
    } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      collision_result = collision_detector_ptr_->UpdateByObsMap(
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

const bool ParallelPathPlanner::CheckSamePos(
    const Eigen::Vector2d& pos0, const Eigen::Vector2d& pos1) const {
  return ((pos0 - pos1).norm() <= apa_param.GetParam().static_pos_eps);
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

void ParallelPathPlanner::CalcEgoParams() {
  std::vector<Eigen::Vector2d> front_corner_vec;
  front_corner_vec.reserve(5);

  front_corner_vec.emplace_back(
      Eigen::Vector2d(apa_param.GetParam().car_vertex_x_vec[0],
                      apa_param.GetParam().car_vertex_y_vec[0]));

  front_corner_vec.emplace_back(
      Eigen::Vector2d(apa_param.GetParam().car_vertex_x_vec[1],
                      apa_param.GetParam().car_vertex_y_vec[1]));

  front_corner_vec.emplace_back(
      Eigen::Vector2d(apa_param.GetParam().car_vertex_x_vec[2],
                      apa_param.GetParam().car_vertex_y_vec[2]));

  front_corner_vec.emplace_back(0.5 *
                                (front_corner_vec[0] + front_corner_vec[1]));

  front_corner_vec.emplace_back(0.5 *
                                (front_corner_vec[1] + front_corner_vec[2]));

  const Eigen::Vector2d center(0.0, -apa_param.GetParam().min_turn_radius);

  calc_params_.v_ego_farest_front_corner = front_corner_vec.front();
  calc_params_.min_outer_front_corner_radius =
      (front_corner_vec.front() - center).norm();
  // find max radius of front corner
  for (size_t i = 1; i < front_corner_vec.size(); i++) {
    const double corner_radius = (front_corner_vec[i] - center).norm();

    if (calc_params_.min_outer_front_corner_radius < corner_radius) {
      calc_params_.min_outer_front_corner_radius = corner_radius;
      calc_params_.v_ego_farest_front_corner = front_corner_vec[i];
    }
  }

  calc_params_.min_outer_front_corner_deta_y =
      calc_params_.min_outer_front_corner_radius -
      apa_param.GetParam().min_turn_radius -
      0.5 * apa_param.GetParam().car_width;
  DEBUG_PRINT("min_outer_front_corner_radius = "
              << calc_params_.min_outer_front_corner_radius);
  // DEBUG_PRINT("min_turn_radiu = " << apa_param.GetParam().min_turn_radius);
  // DEBUG_PRINT("half car width = " << 0.5 * apa_param.GetParam().car_width);
  DEBUG_PRINT("car length in apa_param = " << apa_param.GetParam().car_length);
}

}  // namespace apa_planner
}  // namespace planning