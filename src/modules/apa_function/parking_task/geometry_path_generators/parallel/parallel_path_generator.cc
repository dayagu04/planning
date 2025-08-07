#include "parallel_path_generator.h"

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
// #include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

#include "apa_param_config.h"
#include "apa_world.h"
#include "collision_detection/collision_detection.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math_lib.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

namespace planning {
namespace apa_planner {

// for park out
static const double kMaxParkOutFirstArcHeading = 66.0;
static const double kMaxParkOutRootHeading = 25.0;

static const double kLonBufferTrippleStep = 0.2;   // tripple step
static const double kColBufferInSlot = 0.25;       // in slot
static const double kColSmallBufferInSlot = 0.16;  // in slot

static const double kColBufferOutSlot = 0.3;           // out slot
static const double kColLargeLatBufferOutSlot = 0.3;   // out slot
static const double kColSmallLatBufferOutSlot = 0.15;  // outslot

static const size_t kMaxParallelParkInSegmentNums = 15;
static const size_t kMaxPathNumsInSlot = 8;
static const size_t kMaxMultiStepNums = 8;
static const size_t kMaxParallelShiftNums = 6;

static const double kChannelYMoveDist = 0.15;
static const double kCornerSafeBufferWithChannel = 0.15;
static const double kMaxHeadingFirstStepForwardLine = 5.0;
static const double kMaxFirstStepForwardInclinedLineLength = 1.56;
static const double kVirtualObsDetaXMag = 0.1;
static const double kVirtualObsDetaYMag = 0.2;
static const double kMinTlaneAddedLength = 0.65;
static const double kNarrowChannelLastArcCrossLength = 1.38;

static const double kLineStepLength = 0.16;
static const double k1dExtendLength = 0.36;

static const size_t kInvalidInteger = 666;

static const double kMinXInTBoundary = -1.0;
static const double kMaxDriveLengthInclineInSlot = 0.6;
static const double kMinInclineHeadingDeg = 12.0;

static const double kMaxSearchArcLength = 1.5;
static const double kMinSearchArcStep = 0.2;
static const int kMaxSearchArcStepNum = 3;

static const double kSamplingLineStep = 0.3;
static const double kSamplingArcAngleStep = 3.0 * kDeg2Rad;
static const int kMaxPathSize = 5;
static const double kGearSwitchPenalty = 7.0;
static const double kGearFirstPenalty = 0.0;
static const double kPathLengthPenalty = 1.0;
static const double kPathLengthFristPenalty = 0.0;
static const double kPathLengthShortPenalty = 0.0;
static const double kParkingHeadingDegPenalty = 0.0;
static const double kMaxXPenalty = 0.8;
static const double kMaxYPenalty = 2.0;

void ParallelPathGenerator::Reset() {
  output_.Reset();
  calc_params_.Reset();
  output_.steer_vec.reserve(kMaxParallelParkInSegmentNums);
  output_.gear_cmd_vec.reserve(kMaxParallelParkInSegmentNums);
  output_.path_segment_vec.reserve(kMaxParallelParkInSegmentNums);
}

void ParallelPathGenerator::Preprocess() {
  pnc::geometry_lib::PrintPose("start pose",
                               input_.ego_info_under_slot.cur_pose);
  output_.Reset();
  debug_info_.Reset();
  calc_params_.Reset();

  input_.ego_info_under_slot.cur_pose.heading =
      pnc::geometry_lib::NormalizeAngle(
          input_.ego_info_under_slot.cur_pose.heading);

  calc_params_.is_left_side =
      (input_.tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT);
  calc_params_.slot_side_sgn = calc_params_.is_left_side ? -1.0 : 1.0;

  const double target_heading = 0.0;
  calc_params_.target_pose.Set(input_.tlane.pt_terminal_pos, target_heading);
  calc_params_.target_line = pnc::geometry_lib::BuildLineSegByPose(
      input_.tlane.pt_terminal_pos, target_heading);

  //  buffer init
  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.1, false));

  ExpandObstacles();

  MoveChannelObstacles();

  CalcEgoParams();

  if (input_.ego_info_under_slot.slot_occupied_ratio < 0.01) {
    calc_params_.lat_outside_slot_buffer = CalcBufferViaDistOfEgoToObs();
  }

  if (input_.tlane.pt_inside.x() - input_.tlane.pt_outside.x() >= 6.2) {
    calc_params_.lon_buffer_rev_trials = kColBufferInSlot;
  } else {
    calc_params_.lon_buffer_rev_trials = kColSmallBufferInSlot;
  }

  calc_params_.weights = {kGearSwitchPenalty,
                          kGearFirstPenalty,
                          kPathLengthPenalty,
                          kPathLengthFristPenalty,
                          kPathLengthShortPenalty,
                          kParkingHeadingDegPenalty,
                          kMaxXPenalty,
                          kMaxYPenalty};
}

void ParallelPathGenerator::ExpandObstacles() {
  const auto& obs_map = collision_detector_ptr_->GetObstaclesMap();

  const Eigen::Vector2d coord_diff(
      -kVirtualObsDetaXMag, kVirtualObsDetaYMag * input_.tlane.slot_side_sgn);

  std::vector<size_t> obs_type_vec = {CollisionDetector::TLANE_OBS,
                                      CollisionDetector::TLANE_BOUNDARY_OBS};

  calc_params_.front_corner_obs_vec.clear();
  for (const auto obs_type : obs_type_vec) {
    const auto obs_it = obs_map.find(obs_type);
    if (obs_it == obs_map.end()) continue;

    // get nearby obstalces of pin
    for (const auto& tlane_obs_pt : obs_it->second) {
      if (pnc::geometry_lib::CalTwoPointDistSquare(
              tlane_obs_pt, input_.tlane.obs_pt_inside) <= 1) {
        calc_params_.front_corner_obs_vec.emplace_back(tlane_obs_pt +
                                                       coord_diff);
        ILOG_INFO << "VIRTUAL OBS = "
                  << (tlane_obs_pt + coord_diff).transpose();
      }
    }
  }

  // channel obs expand
  const auto channel_obs_iter = obs_map.find(CollisionDetector::CHANNEL_OBS);
  if (channel_obs_iter != obs_map.end()) {
    const Eigen::Vector2d channel_mov_vec(
        0.0, -calc_params_.slot_side_sgn * kChannelYMoveDist);

    calc_params_.channel_obs_vec = channel_obs_iter->second;

    for (const auto& channel_obs_pt : channel_obs_iter->second) {
      calc_params_.virtual_channel_obs_vec.emplace_back(channel_obs_pt +
                                                        channel_mov_vec);
    }
  }
}

void ParallelPathGenerator::AddPInVirtualObstacles() {
  collision_detector_ptr_->AddObstacles(calc_params_.front_corner_obs_vec,
                                        CollisionDetector::VIRTUAL_OBS);
}

void ParallelPathGenerator::DeletePInVirtualObstacles() {
  collision_detector_ptr_->DeleteGivenTypeObstacles(
      CollisionDetector::VIRTUAL_OBS);
}

void ParallelPathGenerator::MoveChannelObstacles() {
  collision_detector_ptr_->DeleteGivenTypeObstacles(
      CollisionDetector::CHANNEL_OBS);

  collision_detector_ptr_->AddObstacles(
      calc_params_.virtual_channel_obs_vec,
      CollisionDetector::CollisionDetector::CHANNEL_OBS);
}

void ParallelPathGenerator::RecorverChannelObstacles() {
  collision_detector_ptr_->DeleteGivenTypeObstacles(
      CollisionDetector::CHANNEL_OBS);

  collision_detector_ptr_->AddObstacles(calc_params_.channel_obs_vec,
                                        CollisionDetector::CHANNEL_OBS);
}

const std::vector<Eigen::Vector2d> ParallelPathGenerator::GetVirtualObs() {
  std::vector<Eigen::Vector2d> obs_vec;
  obs_vec = calc_params_.virtual_channel_obs_vec;
  for (const auto& obs_pt : calc_params_.front_corner_obs_vec) {
    obs_vec.emplace_back(obs_pt);
  }
  return obs_vec;
}

const bool ParallelPathGenerator::Update() {
  ILOG_INFO << "-----------------------------------------parallel path "
               "planner:---------------------------------------";
  // preprocess
  Preprocess();

  if (!CheckTlaneAvailable()) {
    ILOG_INFO << "tlane_too short!";
    return false;
  }

  const double start_time = IflyTime::Now_ms();

  // judge if ego is out of slot
  if (!CheckEgoInSlot()) {
    ILOG_INFO << "ego is out of slot";
    AddPInVirtualObstacles();

    if (!CalMinSafeCircle()) {
      ILOG_INFO << "calc safe circle failed!";
      return false;
    }
    ILOG_INFO << "CalMinSafeCircle success!";
    const double safe_circle_end_time = IflyTime::Now_ms();
    ILOG_INFO << "calc safe circle cost time(ms) = "
              << safe_circle_end_time - start_time;

    bool success = OutsideSlotPlan();
    if (!success) {
      ILOG_INFO << "OutsideSlotPlan failed!";
      const double search_start_time = IflyTime::Now_ms();
      success = GenerateCandidatePath();
      const double search_end_time = IflyTime::Now_ms();
      ILOG_INFO << "search cost time(ms) = "
                << search_end_time - search_start_time;
    }

    if (success) {
      ILOG_INFO << "calc_params_.park_out_path_in_slot.size() = "
                << calc_params_.park_out_path_in_slot.size();
      if (calc_params_.park_out_path_in_slot.size() > 1 &&
          std::fabs(
              calc_params_.park_out_path_in_slot.back().GetStartHeading() *
              kRad2Deg) > 0.2) {
        calc_params_.park_out_path_in_slot.pop_back();

        ReversePathSegVec(calc_params_.park_out_path_in_slot);
        AddPathSegToOutPut(calc_params_.park_out_path_in_slot);
      }

      pnc::geometry_lib::PathSegment last_path_seg;
      if (AddLastLine(last_path_seg)) {
        ILOG_INFO << "AddLastLine!";
        AddPathSegToOutPut(last_path_seg);
      }
      ILOG_INFO << "don't AddLastLine!";
      DeletePInVirtualObstacles();
      return success;
    }

  } else {
    ILOG_INFO << "ego is in slot";
    collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.0, false));
    // ego is in slot, search from ego pose to target pose, or just
    // correct heading
    if (MultiPlan()) {
      // pnc::geometry_lib::PrintSegmentsVecInfo(output_.path_segment_vec);
      ILOG_INFO << "MultiPlan success!";
      return true;
    } else {
      ILOG_INFO << "MultiPlan  failed!";
      if (MultiAlignBody()) {
        ILOG_INFO << "Multi align body success!";
        return true;
      } else {
        ILOG_INFO << "Multi align body failed!";
      };
    }

    // parallel adjust step
    if (ParallelAdjustPlan()) {
      // pnc::geometry_lib::PrintSegmentsVecInfo(output_.path_segment_vec);
      ILOG_INFO << "parallel adjust step plan success!";
      // PrintOutputSegmentsInfo();
      return true;
    } else {
      ILOG_INFO << "parallel adjust step plan failed!";
    }
  }
  output_.Reset();
  ILOG_INFO << "plan failed!";
  return false;
}

const bool ParallelPathGenerator::CheckTlaneAvailable() const {
  const double tlane_length =
      input_.tlane.obs_pt_inside.x() - input_.tlane.obs_pt_outside.x();

  const double min_released_slot_length =
      apa_param.GetParam().car_length + kMinTlaneAddedLength;

  ILOG_INFO << "tlane_length = " << tlane_length;
  // ILOG_INFO <<"car length = " << apa_param.GetParam().car_length);
  ILOG_INFO << "min_released_slot_length = " << min_released_slot_length;

  return tlane_length >= min_released_slot_length;
}

const bool ParallelPathGenerator::Update(
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  const auto time0 = std::chrono::high_resolution_clock::now();
  collision_detector_ptr_ = collision_detector_ptr;
  const bool success = Update();
  RecorverChannelObstacles();
  pnc::geometry_lib::PrintSegmentsVecInfo(output_.path_segment_vec);
  const auto time1 = std::chrono::high_resolution_clock::now();
  const auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(time1 - time0)
          .count();
  JSON_DEBUG_VALUE("path_plan_time_ms", duration);
  ILOG_INFO << "parallel cost time(ms) = " << duration;
  return success;
}

// park out from target pose with two arc to ego line.
const bool ParallelPathGenerator::PlanFromTargetToLine(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose, const bool is_park_out) {
  // ILOG_INFO <<"------PlanFromTargetToLine-------");
  using namespace pnc::mathlib;
  using namespace pnc::geometry_lib;

  Arc arc_1;
  Arc diverse_arc_2;
  Arc narrow_arc_2;

  bool success = false;
  bool diverse_radius_success = false;
  bool narrow_channel_success = false;

  auto ego_line_unit = BuildLineSegByPose(start_pose.pos, start_pose.heading);
  auto ego_line = ego_line_unit;

  const double min_turn_radius = apa_param.GetParam().min_turn_radius;

  arc_1.circle_info.radius = min_turn_radius;
  const uint8_t arc_1_steer =
      (calc_params_.is_left_side ? SEG_STEER_RIGHT : SEG_STEER_LEFT);

  const std::vector<double> arc2_radius_vec = {
      min_turn_radius,     min_turn_radius + 1,  min_turn_radius + 2,
      min_turn_radius + 3, min_turn_radius + 4,  min_turn_radius + 5,
      min_turn_radius + 8, min_turn_radius + 10, min_turn_radius + 12};

  std::vector<PathSegment> narrow_path_seg_vec;
  for (const auto& target_pose : calc_params_.valid_target_pt_vec) {
    arc_1.pA = target_pose.pos;
    arc_1.headingA = target_pose.heading;
    arc_1.circle_info.center =
        CalEgoTurningCenter(target_pose, min_turn_radius, arc_1_steer);

    // last line step length should be more than min_leng with dirve gear
    if (calc_params_.valid_target_pt_vec.size() > 1) {
      const pnc::geometry_lib::PathPoint last_pA(arc_1.pA, arc_1.headingA);

      const pnc::geometry_lib::PathPoint last_pB(
          input_.tlane.pt_terminal_pos, calc_params_.target_line.heading);

      const LineSegment last_line(last_pA.pos, last_pB.pos, last_pA.heading);
      const uint8_t last_line_gear = CalLineSegGear(last_line);

      if (!CheckSamePose(last_pA, last_pB) &&
          last_line_gear == SEG_GEAR_DRIVE &&
          last_line.length < apa_param.GetParam().min_path_length) {
        continue;
      }
    }

    bool is_arc_2_set = false;
    size_t i = 0;
    for (; i < arc2_radius_vec.size(); i++) {
      const auto& arc2_radius = arc2_radius_vec[i];
      diverse_arc_2.circle_info.radius = arc2_radius;

      if (!CalTwoSameGearArcWithLine(arc_1, diverse_arc_2, ego_line_unit,
                                     SEG_GEAR_DRIVE, true)) {
        continue;
      }

      if (i == 0) {
        narrow_arc_2 = diverse_arc_2;
        is_arc_2_set = true;
      }

      ego_line.heading = start_pose.heading;
      ego_line.SetPoints(start_pose.pos, diverse_arc_2.pB);
      if (!CheckEgoLine(ego_line)) {
        // ILOG_INFO << "CheckEgoLine failed!";
        continue;
      }

      auto col_res = collision_detector_ptr_->UpdateByObsMap(
          diverse_arc_2, diverse_arc_2.headingA);
      if (col_res.collision_flag ||
          col_res.remain_car_dist >
              col_res.remain_obstacle_dist - kLonBufferTrippleStep) {
        // ILOG_INFO << "arc2 collided!";
        continue;
      }
      ILOG_INFO << "arc2_radius = " << arc2_radius << "calc success!";
      success = true;
      diverse_radius_success = true;
      break;
    }

    if (is_arc_2_set && i > 2) {
      if (PlanFromTargetToLineInNarrowChannel(narrow_path_seg_vec, arc_1,
                                              narrow_arc_2)) {
        success = true;
        narrow_channel_success = true;
      }
    }

    if (success) {
      break;
    }
  }

  if (!success) {
    // ILOG_INFO <<"arc2 collided! && narrow plan failed!");
    return false;
  }
  bool is_narrow_channel = false;

  ILOG_INFO << "diverse_radius_success = " << diverse_radius_success;
  ILOG_INFO << "narrow_channel_success = " << narrow_channel_success;

  if (diverse_radius_success && !narrow_channel_success) {
    is_narrow_channel = false;
  } else if (!diverse_radius_success && narrow_channel_success) {
    is_narrow_channel = true;
  } else if (diverse_radius_success && narrow_channel_success) {
    is_narrow_channel =
        narrow_path_seg_vec.back().GetEndPos().x() < diverse_arc_2.pB.x();
    ILOG_INFO << "narrow_path_seg_vec.back().GetEndPos().x() = "
              << narrow_path_seg_vec.back().GetEndPos().x();
    ILOG_INFO << "diverse_arc_2.pB.x() = " << diverse_arc_2.pB.x();
  }

  Arc arc_2;
  if (is_narrow_channel) {
    ego_line.heading = start_pose.heading;
    ego_line.SetPoints(start_pose.pos, narrow_path_seg_vec.back().GetEndPos());
    arc_2 = narrow_arc_2;
  } else {
    arc_2 = diverse_arc_2;
  }

  const double ego_length = ego_line.length;
  const auto ego_gear = CalLineSegGear(ego_line);
  const bool is_ego_line_needed =
      (ego_gear == SEG_GEAR_DRIVE &&
       ego_length > apa_param.GetParam().min_path_length) ||
      (ego_gear == SEG_GEAR_REVERSE && ego_length > 0.01);

  if (is_ego_line_needed) {
    path_seg_vec.emplace_back(PathSegment(ego_gear, ego_line));
  }

  if (is_narrow_channel && ReversePathSegVecInfo(narrow_path_seg_vec)) {
    for (const auto& path_seg : narrow_path_seg_vec) {
      path_seg_vec.emplace_back(path_seg);
    }

  } else {
    PathSegment arc_seg_1(SEG_STEER_LEFT, SEG_GEAR_DRIVE, arc_1);
    PathSegment arc_seg_2(SEG_STEER_RIGHT, SEG_GEAR_DRIVE, arc_2);
    ReverseArcSegInfo(arc_seg_1);
    ReverseArcSegInfo(arc_seg_2);

    path_seg_vec.emplace_back(arc_seg_2);
    path_seg_vec.emplace_back(arc_seg_1);
  }

  const auto path_end_pose = path_seg_vec.back().GetEndPose();

  if (!is_park_out &&
      IsDoubleEqual(path_end_pose.heading, calc_params_.target_pose.heading) &&
      !IsDoubleEqual(path_end_pose.pos.x(), input_.tlane.pt_terminal_pos.x())) {
    const Eigen::Vector2d fixed_target_pos(input_.tlane.pt_terminal_pos.x(),
                                           path_end_pose.pos.y());

    const LineSegment last_line(path_end_pose.pos, fixed_target_pos,
                                path_end_pose.heading);

    path_seg_vec.emplace_back(
        PathSegment(CalLineSegGear(last_line), last_line));
  }

  return success;
}

const bool ParallelPathGenerator::CheckEgoLine(
    pnc::geometry_lib::LineSegment& ego_line) {
  using namespace pnc::geometry_lib;

  const auto ego_line_gear = CalLineSegGear(ego_line);
  if (ego_line_gear == SEG_GEAR_DRIVE) {
    const double heading_mag_deg = std::fabs(ego_line.heading) * kRad2Deg;
    if (heading_mag_deg > kMaxHeadingFirstStepForwardLine &&
        ego_line.length > kMaxFirstStepForwardInclinedLineLength) {
      return false;
    }
  }

  // collision_detector_ptr_->SetParam(
  //     CollisionDetector::Paramters(kColSmallLatBufferOutSlot, false));

  auto col_res =
      collision_detector_ptr_->UpdateByObsMap(ego_line, ego_line.heading);
  if (col_res.collision_flag ||
      col_res.remain_car_dist >
          col_res.remain_obstacle_dist - kColBufferOutSlot) {
    // ILOG_INFO << "ego line collided!";
    return false;
  }

  return true;
}

const bool ParallelPathGenerator::PlanFromTargetToLineInNarrowChannel(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::Arc& arc1, const pnc::geometry_lib::Arc& arc_2) {
  path_seg_vec.clear();
  path_seg_vec.reserve(10);
  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.1, false));

  // ILOG_INFO <<
  //     "------------------------  PlanFromTargetToLineInNarrowChannel "
  //     "----------------------");

  // ILOG_INFO <<"slot_side_sgn = " << calc_params_.slot_side_sgn);
  // ILOG_INFO <<"arc_2 center = " << arc_2.circle_info.center.transpose()
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
  // ILOG_INFO <<"current channel y = " << input_.tlane.channel_y);

  const auto& channel_obs_vec = collision_detector_ptr_->GetObstaclesMap().at(
      CollisionDetector::CHANNEL_OBS);

  const double radius_square =
      std::pow(arc1.circle_info.radius + arc_2.circle_info.radius, 2.0);

  const double safe_radius_square = std::pow(
      calc_params_.min_outer_front_corner_radius + kChannelYMoveDist, 2.0);

  const std::vector<double> y_diff_vec = {-0.1, -0.2, -0.3, -0.4, -0.5};
  bool arc2_success = false;
  for (const auto& y_diff : y_diff_vec) {
    new_center.y() =
        arc_2.circle_info.center.y() + y_diff * calc_params_.slot_side_sgn;

    const double dy_square =
        std::pow(arc1.circle_info.center.y() - new_center.y(), 2.0);

    const double dx_square = radius_square - dy_square;
    if (dx_square < 0.0) {
      // ILOG_INFO <<"geometry dy^2 < 0.0!");
      return false;
    }

    new_center.x() = arc1.circle_info.center.x() + std::sqrt(dx_square);

    for (size_t j = 0; j < channel_obs_vec.size(); j++) {
      const auto& obs_pt = channel_obs_vec[j];

      const Eigen::Vector2d v_diff = new_center - obs_pt;
      const double dist_square =
          v_diff.x() * v_diff.x() + v_diff.y() * v_diff.y();
      if (dist_square < safe_radius_square) {
        break;
      }

      if (j == channel_obs_vec.size() - 1) {
        arc2_success = true;
        break;
      }
    }

    if (arc2_success) {
      break;
    }
  }

  if (!arc2_success) {
    return false;
  }

  // ILOG_INFO <<"new center " << new_center.transpose());

  // for debug
  // debug_info_.debug_arc_vec.emplace_back(arc1);
  // pnc::geometry_lib::Arc new_arc2 = arc_2;
  // new_arc2.circle_info.center = new_center;
  // debug_info_.debug_arc_vec.emplace_back(new_arc2);
  // new_arc2.circle_info.radius = calc_params_.min_outer_front_corner_radius;
  // debug_info_.debug_arc_vec.emplace_back(new_arc2);

  const double corner_theta =
      std::atan(std::fabs(calc_params_.v_ego_farest_front_corner.x()) /
                (apa_param.GetParam().min_turn_radius +
                 std::fabs(calc_params_.v_ego_farest_front_corner.y()))) *
      calc_params_.slot_side_sgn;
  // ILOG_INFO <<"corner_theta deg = " << corner_theta * kRad2Deg);

  const double travel_theta = -calc_params_.slot_side_sgn *
                              kNarrowChannelLastArcCrossLength /
                              arc_2.circle_info.radius;
  // ILOG_INFO <<"travel_theta deg = " << travel_theta * kRad2Deg);
  const double d_theta =
      pnc::geometry_lib::NormalizeAngle(corner_theta + travel_theta);
  // ILOG_INFO <<"d_theta = " << d_theta * kRad2Deg);
  const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(d_theta);

  const Eigen::Vector2d v_vertical(0.0, calc_params_.slot_side_sgn);

  const auto pos_B =
      (rot_m * v_vertical) * apa_param.GetParam().min_turn_radius + new_center;

  // ILOG_INFO <<"v vertical = " << v_vertical.transpose());
  // ILOG_INFO <<"(rot_m * v_vertical) = " << (rot_m * v_vertical).transpose());
  // ILOG_INFO <<"(rot_m * v_vertical) * apa_param.GetParam().min_turn_radius =
  // "
  //             << (rot_m * v_vertical) *
  //             apa_param.GetParam().min_turn_radius);
  // ILOG_INFO <<"new center " << new_center.transpose());

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
        // ILOG_INFO << "dubins success!" ;
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
    // ILOG_INFO <<"tripple step dubins failed!");
    return false;
  }

  // ILOG_INFO <<"tripple step dubins success!");
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
      ILOG_INFO << "CalSinglePathInNarrowChannel failed!";
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
      ILOG_INFO << "already plan to prepare line";
      success = true;
      break;
    }
  }

  if (success) {
    path_seg_vec = first_three_steps;
    // ILOG_INFO <<"path_seg_vec size = " << path_seg_vec.size());
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

const bool ParallelPathGenerator::CalSinglePathInNarrowChannel(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const uint8_t current_gear,
    const uint8_t current_arc_steer) {
  // ILOG_INFO <<"----- " << __func__ << " -----");
  // ILOG_INFO <<"current_arc_steer = "
  //             << static_cast<int>(current_arc_steer)
  //             << ",  current_gear = " << static_cast<int>(current_gear)
  //             << ",  current_pos = " << current_pose.pos.transpose()
  //             << ",  current_heading = " << current_pose.heading * kRad2Deg);

  path_seg_vec.clear();
  path_seg_vec.reserve(3);

  pnc::geometry_lib::Arc current_arc;
  current_arc.pA = current_pose.pos;
  current_arc.headingA =
      pnc::geometry_lib::NormalizeAngle(current_pose.heading);

  const double min_radius = apa_param.GetParam().min_turn_radius + 0.5;
  current_arc.circle_info.radius = min_radius;
  current_arc.circle_info.center =
      CalEgoTurningCenter(current_pose, min_radius, current_arc_steer);

  // debug_info_.debug_arc_vec.emplace_back(current_arc);

  auto target_line = pnc::geometry_lib::BuildLineSegByPose(target_pose.pos,
                                                           target_pose.heading);

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  if (!LineArcPlan(current_arc, tmp_path_seg_vec, target_line, current_gear,
                   current_arc_steer)) {
    ILOG_INFO << "LineArcPlan fail";
    return false;
  }

  uint8_t col_res = PATH_COL_COUNT;

  size_t path_idx = 0;
  for (; path_idx < tmp_path_seg_vec.size(); path_idx++) {
    auto& tmp_path_seg = tmp_path_seg_vec[path_idx];
    col_res = TrimPathByCollisionDetection(tmp_path_seg, kColBufferOutSlot);

    if (col_res == PATH_COL_NORMAL) {
      ILOG_INFO << "No. " << path_idx << " normal!";
    } else if (col_res == PATH_COL_SHORTEN) {
      break;
    } else if (col_res == PATH_COL_INVALID) {
      ILOG_INFO << "path col at start pose, invalid!";
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

const bool ParallelPathGenerator::BackwardNormalPlan() {
  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
  path_seg_vec.reserve(10);

  if (OneStepDubinsTryInTripplePlan(path_seg_vec,
                                    input_.ego_info_under_slot.cur_pose)) {
    AddPathSegToOutPut(path_seg_vec);
    ILOG_INFO << "ego_pose OneStepDubinsTryInTripplePlan success!";
    return true;
  }

  if (PlanFromTargetToLine(path_seg_vec, input_.ego_info_under_slot.cur_pose)) {
    AddPathSegToOutPut(path_seg_vec);
    ILOG_INFO << "ego_pose PlanFromTargetToLine success!";
    return true;
  }

  return false;
}

const bool ParallelPathGenerator::BackwardNormalPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose) {
  path_seg_vec.clear();
  path_seg_vec.reserve(10);

  if (OneStepDubinsTryInTripplePlan(path_seg_vec, start_pose)) {
    ILOG_INFO << "OneStepDubinsTryInTripplePlan success!";
    return true;
  }

  if (PlanFromTargetToLine(path_seg_vec, start_pose)) {
    ILOG_INFO << "PlanFromTargetToLine success!";
    return true;
  }

  return false;
}

const bool ParallelPathGenerator::OneStepDubinsTryInTripplePlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose) {
  path_seg_vec.clear();
  path_seg_vec.reserve(5);

  std::vector<double> radius_vec = {apa_param.GetParam().min_turn_radius + 0.5,
                                    apa_param.GetParam().min_turn_radius};
  if (input_.ego_info_under_slot.cur_pose.pos.x() >
      input_.tlane.slot_length + 1.0) {
    radius_vec.insert(radius_vec.begin(),
                      apa_param.GetParam().min_turn_radius + 1.5,
                      apa_param.GetParam().min_turn_radius + 1.0);
  }

  ILOG_INFO << "terminal pos = " << input_.tlane.pt_terminal_pos.transpose();

  ILOG_INFO << "target pose ---------------";
  for (const auto& target_pt : calc_params_.valid_target_pt_vec) {
    ILOG_INFO << "target_pt_x =" << target_pt.pos.transpose();
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

      ILOG_INFO << "plan from ego pose to valid target pose success!"
                << " valid pos= " << target_pose.pos.transpose()
                << ", radius =" << radius;

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

    ILOG_INFO << "triple step from ego to park out pose success! radius = "
              << radius;

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

const bool ParallelPathGenerator::OutsideSlotPlan() {
  ILOG_INFO << "---------------------------------- outside slot plan "
               "----------------------------------";

  output_.Reset();
  std::vector<GeometryPath> geo_path_vec;
  debug_info_.debug_all_path_vec.clear();

  if (input_.ego_info_under_slot.cur_pose.pos.x() >
      input_.tlane.slot_length - 2.0) {
    GeometryPath ego_line_geo_path;
    std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
    collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.1, false));
    if (BackwardNormalPlan(tmp_path_seg_vec,
                           input_.ego_info_under_slot.cur_pose)) {
      AssempleGeometryPath(ego_line_geo_path, tmp_path_seg_vec);

      ILOG_INFO << "first try ego line plan success!";
      if (ego_line_geo_path.gear_change_count == 0) {
        debug_info_.debug_all_path_vec.emplace_back(ego_line_geo_path);
        AddPathSegToOutPut(ego_line_geo_path.path_segment_vec);
        ILOG_INFO << "ego line path vec -----------------------------";
        geometry_lib::PrintSegmentsVecInfo(ego_line_geo_path.path_segment_vec);
        ILOG_INFO << "no need change gear in preparing step!";
        return true;
      }
      geo_path_vec.emplace_back(ego_line_geo_path);
    }
  }

  // Todo: connect two line vec method
  std::vector<pnc::geometry_lib::PathPoint> preparing_pose_vec;
  GenAlignedPreparingLine(preparing_pose_vec,
                          input_.ego_info_under_slot.cur_pose);
  const double aligned_size = preparing_pose_vec.size();
  ILOG_INFO << "aligned_size = " << aligned_size;

  GenParallelPreparingLineVec(preparing_pose_vec);
  const double parallel_line_size = preparing_pose_vec.size();
  ILOG_INFO << "parallel preparing line size= " << parallel_line_size;

  GenTiltedPreparingLine(preparing_pose_vec);
  ILOG_INFO << "total preparing line size= " << preparing_pose_vec.size();
  const double tiled_line_size = preparing_pose_vec.size();

  bool ret = GenTiltedPreparingLine2ShortChannel(preparing_pose_vec);
  if (!ret) {
    ILOG_WARN << "GenTiltedPreparingLine2ShortChannel failed!";
  }
  ILOG_INFO << "total2 preparing line size= " << preparing_pose_vec.size();
  const double big_ang_tiled_line_size = preparing_pose_vec.size();
  for ( int k = tiled_line_size; k < big_ang_tiled_line_size; k++) {
    pnc::geometry_lib::LineSegment line_seg(preparing_pose_vec[k].pos,
      preparing_pose_vec[k].heading, 1, 1);
    debug_info_.debug_line_vec.emplace_back(line_seg);
  }
  ILOG_INFO << "big_ang_tiled_line_size = " << debug_info_.debug_line_vec.size();

  size_t aligned_success_cnt = 0;
  size_t total_success_cnt = 0;
  size_t parallel_success_cnt = 0;
  size_t tiled_success_cnt = 0;
  size_t big_ang_tiled_success_cnt = 0;
  std::vector<std::pair<int, pnc::geometry_lib::PathPoint>> target2line;
  std::vector<pnc::geometry_lib::PathPoint> target2prepare_line;
  std::vector<std::pair<int, pnc::geometry_lib::PathPoint>> car2line;
  std::unordered_map<double, int> tiled_success_cnt_map;
  bool short_channel_flag = calc_params_.slot_side_sgn *
      (input_.tlane.obs_pt_inside.y() - input_.tlane.corner_inside_slot.y()) > 0.5;
  for (size_t i = 0; i < preparing_pose_vec.size(); i++) {
    ILOG_INFO << "No. " << i;
    geometry_lib::PrintPose("prepare pose", preparing_pose_vec[i]);
    if ((i >= aligned_size && i < parallel_line_size && parallel_success_cnt > 1)||
    (parallel_line_size <= i && i < tiled_line_size && tiled_success_cnt > 1)) {
      continue;
    }
    if ((i >= tiled_line_size && aligned_success_cnt + parallel_success_cnt + tiled_success_cnt > 0)
        && !short_channel_flag) {
      ILOG_INFO << "commonly prepare success, so skip!";
      break;
    }
    if (tiled_success_cnt_map[preparing_pose_vec[i].heading] >= 1 ||
       (tiled_line_size <= i && i < big_ang_tiled_line_size && big_ang_tiled_success_cnt > 2)) {
      continue;
    }
    collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.1, false));

    std::vector<pnc::geometry_lib::PathSegment> inversed_park_out_path;
    if (!PlanFromTargetToLine(inversed_park_out_path, preparing_pose_vec[i])) {
      ILOG_INFO << " PlanFromTargetToLine failed!";
      continue;
    }

    if (inversed_park_out_path.size() == 0) {
      ILOG_INFO << "inversed_park_out_path size ="
                << inversed_park_out_path.size();
      continue;
    }

    if (inversed_park_out_path.front().seg_type ==
        pnc::geometry_lib::SEG_TYPE_LINE) {
      inversed_park_out_path.erase(inversed_park_out_path.begin());
    }

    const auto prepare_pose = inversed_park_out_path.front().GetStartPose();
    const auto prepare_line = pnc::geometry_lib::BuildLineSegByPose(
        prepare_pose.pos, prepare_pose.heading);
    target2line.emplace_back(std::make_pair(i, preparing_pose_vec[i]));
    target2prepare_line.emplace_back(prepare_pose);

    // search min dist of ego corner to obs, which is used the minimal value
    // with
    // 0.36 as buffer
    const double min_lat_buffer = calc_params_.lat_outside_slot_buffer;
    ILOG_INFO << "min_lat_buffer = " << min_lat_buffer;

    collision_detector_ptr_->SetParam(
        CollisionDetector::Paramters(min_lat_buffer, false));
    pnc::geometry_lib::PrintPose("input_.ego_info_under_slot.cur_pose",
                                 input_.ego_info_under_slot.cur_pose);
    std::vector<pnc::geometry_lib::PathSegment> prepare_seg_vec;
    if (!PlanToPreparingLine(prepare_seg_vec,
                             input_.ego_info_under_slot.cur_pose,
                             prepare_line)) {
      ILOG_INFO << "PlanToPreparingLine fail!";
      continue;
    }

    // ILOG_INFO <<"prepare_seg_vec size == " << prepare_seg_vec.size());
    if (prepare_seg_vec.size() == 0) {
      ILOG_INFO << "prepare_seg_vec size == 0";
      continue;
    }

    prepare_seg_vec.insert(prepare_seg_vec.end(),
                           inversed_park_out_path.begin(),
                           inversed_park_out_path.end());

    GeometryPath tmp_geo_path;
    if (!AssempleGeometryPath(tmp_geo_path, prepare_seg_vec)) {
      ILOG_INFO << "AssempleGeometryPath failed!";
      continue;
    }

    if (i < aligned_size) {
      aligned_success_cnt++;
    } else if (i < parallel_line_size) {
      parallel_success_cnt++;
    } else if (i < tiled_line_size) {
      tiled_success_cnt++;
    } else if (i < big_ang_tiled_line_size) {
      big_ang_tiled_success_cnt++;
    }
    ILOG_INFO << "calc success!";
    total_success_cnt++;
    tiled_success_cnt_map[preparing_pose_vec[i].heading]++;
    car2line.emplace_back(std::make_pair(i, preparing_pose_vec[i]));
    geo_path_vec.emplace_back(tmp_geo_path);
  }
  ILOG_INFO << "aligned_success_cnt = " << aligned_success_cnt;
  ILOG_INFO << "parallel_success_cnt = " << parallel_success_cnt;
  ILOG_INFO << "tiled_success_cnt = " << tiled_success_cnt;
  ILOG_INFO << "big_ang_tiled_success_cnt = " << big_ang_tiled_success_cnt;

  ILOG_INFO << "path size = " << geo_path_vec.size();
  if (geo_path_vec.size() == 0) {
    ILOG_INFO << "none path calculated!";
    return false;
  }

  size_t best_path_idx = std::numeric_limits<size_t>::max();
  if (!SelectBestPathOutsideSlot(geo_path_vec, best_path_idx)) {
    ILOG_INFO << "SelectBestPathOutsideSlot failed!";
    return false;
  }
  ILOG_INFO << "best_path_idx = " << best_path_idx;
  AddPathSegToOutPut(geo_path_vec[best_path_idx].path_segment_vec);
  debug_info_.debug_all_path_vec = geo_path_vec;

  const auto end_pose =
      geo_path_vec[best_path_idx].path_segment_vec.back().GetEndPose();

  calc_params_.park_out_path_in_slot.clear();
  for (size_t i = 0; i < calc_params_.valid_target_pt_vec.size(); i++) {
    if (CheckSamePose(end_pose, calc_params_.valid_target_pt_vec[i])) {
      calc_params_.park_out_path_in_slot =
          calc_params_.inversed_path_vec_in_slot[i].path_segment_vec;
    }
  }
  return true;
}

const bool ParallelPathGenerator::PlanToPreparingLine(
    std::vector<pnc::geometry_lib::PathSegment>& ego_to_prepare_seg_vec,
    const pnc::geometry_lib::PathPoint& ego_pose,
    const pnc::geometry_lib::LineSegment& prepare_line) {
  using namespace pnc::geometry_lib;
  ego_to_prepare_seg_vec.clear();
  ego_to_prepare_seg_vec.reserve(6);

  const auto& apa_params = apa_param.GetParam();

  // 1.5r
  const double max_front_wheel_angle =
      atan(apa_params.wheel_base / apa_params.min_turn_radius);

  const double small_front_wheel_angle = max_front_wheel_angle / 3.0;
  const double mid_front_wheel_angle = max_front_wheel_angle * 2.0 / 3.0;

  const double mid_radius =
      apa_params.wheel_base / tan(mid_front_wheel_angle);  // 1r

  const double large_radius =
      apa_params.wheel_base / tan(small_front_wheel_angle);

  const std::vector<double> radius_vec = {apa_params.min_turn_radius + 0.3,
                                          mid_radius, large_radius};

  pnc::geometry_lib::PrintPose("ego_pose", ego_pose);

  pnc::geometry_lib::PathSegment line_seg;
  if (OneLinePlan(line_seg, ego_pose, prepare_line)) {
    ego_to_prepare_seg_vec.emplace_back(line_seg);
    return true;
  }

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  std::vector<std::vector<pnc::geometry_lib::PathSegment>> path_vec;

  const uint8_t ref_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
  const uint8_t ref_steer = pnc::geometry_lib::SEG_STEER_INVALID;
  const double ref_radius = radius_vec[0];

  bool success = false;
  for (const double radius : radius_vec) {
    if (LineArcPlan(path_vec, ego_pose, prepare_line, ref_gear, ref_steer,
                    radius, kColBufferOutSlot)) {
      success = true;
      ILOG_INFO << "LineArcPlan success! size = " << path_vec.size();
      break;
    }
  }

  for (const double radius : radius_vec) {
    if (TwoArcPath(path_vec, ego_pose, prepare_line, ref_gear, radius,
                   kColBufferOutSlot)) {
      success = true;
      ILOG_INFO << "two arc success, total path size = " << path_vec.size();
      break;
    }
  }

  const pnc::geometry_lib::PathPoint target_pose(prepare_line.pA,
                                                 prepare_line.heading);
  std::vector<std::vector<pnc::geometry_lib::PathSegment>> tmp_path_vec;
  if (DubinsPlan(tmp_path_vec, ego_pose, target_pose, ref_radius,
                 kColBufferOutSlot, pnc::geometry_lib::SEG_STEER_INVALID,
                 false)) {
    for (const auto& path : tmp_path_vec) {
      path_vec.emplace_back(path);
    }
  }

  if (path_vec.size() == 0) {
    // search based
    if (SearchToTargetLine(tmp_path_vec, ego_pose, prepare_line, ref_radius,
                           kColBufferOutSlot)) {
      ILOG_INFO << "SearchToTargetLine SUCCESS!";
      ILOG_INFO << "SearchToTargetLine size = " << tmp_path_vec.size();
      for (const auto& path : tmp_path_vec) {
        path_vec.emplace_back(path);
      }
    } else {
      ILOG_INFO << "SearchToTargetLine failed!";
    }
  }

  ILOG_INFO << "path_vec size = " << path_vec.size();
  if (path_vec.size() == 0) {
    return false;
  }

  std::vector<GeometryPath> geo_path_vec;
  AssempleGeometryPathVec(geo_path_vec, path_vec);

  // ILOG_INFO <<"geo_path_vec size = " << geo_path_vec.size());
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

const double ParallelPathGenerator::CalcBufferViaDistOfEgoToObs() {
  const auto& car_vertex_x_vec = apa_param.GetParam().car_vertex_x_vec;
  const auto& car_vertex_y_vec = apa_param.GetParam().car_vertex_y_vec;

  const std::vector<Eigen::Vector2d> left_corner = {
      Eigen::Vector2d(car_vertex_x_vec[0], car_vertex_y_vec[0]),
      Eigen::Vector2d(car_vertex_x_vec[15], car_vertex_y_vec[15])};

  const std::vector<Eigen::Vector2d> right_corner = {
      Eigen::Vector2d(car_vertex_x_vec[5], car_vertex_y_vec[5]),
      Eigen::Vector2d(car_vertex_x_vec[10], car_vertex_y_vec[10])};

  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(input_.ego_info_under_slot.cur_pose.pos,
              input_.ego_info_under_slot.cur_pose.heading);

  double min_real_dist = 2.0;
  for (const auto& pt_pair : collision_detector_ptr_->GetObstaclesMap()) {
    const bool is_left_corner =
        (pt_pair.first == CollisionDetector::CHANNEL_OBS &&
         input_.tlane.slot_side_sgn > 0.0) ||
        (pt_pair.first != CollisionDetector::CHANNEL_OBS &&
         input_.tlane.slot_side_sgn < 0.0);
    const auto& corners = is_left_corner ? left_corner : right_corner;
    const pnc::geometry_lib::LineSegment car_line(l2g_tf.GetPos(corners[0]),
                                                  l2g_tf.GetPos(corners[1]));

    for (const auto& obs_pt : pt_pair.second) {
      const double real_dist =
          pnc::geometry_lib::CalPoint2LineSegDist(obs_pt, car_line);

      min_real_dist = std::min(min_real_dist, real_dist);
    }
  }

  double min_buffer =
      pnc::mathlib::Clamp(min_real_dist - 0.3, kColSmallLatBufferOutSlot,
                          kColLargeLatBufferOutSlot);

  ILOG_INFO << "min_real_dist = " << min_real_dist;
  ILOG_INFO << "min_buffer = " << min_buffer;
  return min_buffer;
}

const bool ParallelPathGenerator::GenAlignedPreparingLine(
    std::vector<pnc::geometry_lib::PathPoint>& preparing_pose_vec,
    const pnc::geometry_lib::PathPoint& ego_pose) {
  if (std::fabs(input_.ego_info_under_slot.cur_pose.heading) * kRad2Deg < 1.0) {
    if (input_.ego_info_under_slot.cur_pose.pos.x() >
            input_.tlane.slot_length ||
        std::fabs(input_.ego_info_under_slot.cur_pose.pos.y() -
                  input_.tlane.obs_pt_inside.y()) > 0.4) {
      preparing_pose_vec.emplace_back(ego_pose);
    }
    return true;
  }

  std::vector<uint8_t> ref_gear_vec = {pnc::geometry_lib::SEG_GEAR_REVERSE,
                                       pnc::geometry_lib::SEG_GEAR_DRIVE};

  for (const auto& ref_gear : ref_gear_vec) {
    ILOG_INFO << "ref gear = " << static_cast<int>(ref_gear);

    std::vector<pnc::geometry_lib::PathSegment> aligned_path_seg_vec;

    collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.0));

    if (!AlignBodyPlan(aligned_path_seg_vec, ego_pose, 0.0, ref_gear)) {
      // ILOG_INFO <<"aligned plan failed!");
      continue;
    }

    const auto& aligned_pos = aligned_path_seg_vec.front().GetEndPos();

    // if (CheckPathSegVecCollided(aligned_path_seg_vec, 0.2)) {
    //   ILOG_INFO <<"aligned plan collided!");
    //   continue;
    // }

    const pnc::geometry_lib::PathPoint aligned_pose(
        Eigen::Vector2d(input_.tlane.pt_inside.x(), aligned_pos.y()), 0.0);

    preparing_pose_vec.emplace_back(aligned_pose);
  }

  return true;
}

const bool ParallelPathGenerator::GenParallelPreparingLineVec(
    std::vector<pnc::geometry_lib::PathPoint>& preparing_pose_vec,
    const bool is_ref_slot_line) {
  const double half_slot_width = 0.5 * input_.tlane.slot_width;
  const double slot_side_sgn = calc_params_.slot_side_sgn;

  double pin_y = input_.tlane.obs_pt_inside.y();

  double tlane_outer_y =
      std::fabs(pin_y) > half_slot_width - 1e-5
          ? pin_y
          : (half_slot_width + std::fabs(pin_y)) * 0.5 * slot_side_sgn;

  double rac_tlane_bound =
      tlane_outer_y +
      slot_side_sgn * (0.5 * apa_param.GetParam().car_width + 0.3);

  if (is_ref_slot_line) {
    tlane_outer_y = std::max(input_.tlane.obs_pt_inside.y() * slot_side_sgn,
                             half_slot_width) *
                    slot_side_sgn;
    rac_tlane_bound =
        tlane_outer_y +
        slot_side_sgn * (0.5 * apa_param.GetParam().car_width + 0.6);
  }

  double rac_channel_bound =
      input_.tlane.channel_y -
      slot_side_sgn * (0.5 * apa_param.GetParam().car_width + 0.3);

  if (slot_side_sgn * (rac_channel_bound - rac_tlane_bound) < 0.0) {
    ILOG_INFO << "rac_channel_bound - rac_tlane_bound "
              << rac_channel_bound - rac_tlane_bound;
    ILOG_INFO << "SGN DIFFERENT!";
    return false;
  }

  if (rac_channel_bound * slot_side_sgn > 10) {
    rac_channel_bound = 10.0 * slot_side_sgn;
  }

  const double y_bound = std::fabs(rac_channel_bound - rac_tlane_bound);

  const double channel_width =
      std::fabs(input_.tlane.channel_y) - half_slot_width;

  double dy = channel_width > 4.0 ? 0.1 : 0.05;

  int nums = static_cast<int>(y_bound / dy);
  nums = pnc::mathlib::Clamp(nums, 5, 16);
  dy = y_bound / nums;

  pnc::geometry_lib::PathPoint prepare_pose(input_.tlane.pt_inside, 0.0);
  prepare_pose.pos.y() = rac_tlane_bound;
  preparing_pose_vec.emplace_back(prepare_pose);

  const auto y_vec =
      pnc::geometry_lib::Linspace(rac_tlane_bound, rac_channel_bound, dy);

  for (const auto y : y_vec) {
    prepare_pose.pos.y() = y;
    preparing_pose_vec.emplace_back(prepare_pose);
  }

  return true;
}

const bool ParallelPathGenerator::GenTiltedPreparingLine(
    std::vector<pnc::geometry_lib::PathPoint>& preparing_pose_vec) {
  const double slot_side_sgn = calc_params_.slot_side_sgn;
  const double half_car_width = 0.5 * apa_param.GetParam().car_width;

  const double channel_width =
      std::fabs(input_.tlane.channel_y) - 0.5 * input_.tlane.slot_width;

  std::vector<double> heading_vec = {7.5, 10.0, 12.5};

  if (channel_width < 3.6) {
    heading_vec.emplace_back(25.0);
    heading_vec.emplace_back(30.0);

    heading_vec.emplace_back(35.0);

    heading_vec.emplace_back(40.0);

    heading_vec.emplace_back(45.0);
  }

  const double step = 0.2;
  size_t max_num = 10;

  for (const auto& heading_deg : heading_vec) {
    const double heading_rad = heading_deg * kDeg2Rad;

    const auto v_preparing_line_heading =
        pnc::geometry_lib::GenHeadingVec(heading_rad);

    const Eigen::Vector2d v_preparing_line_norm(-v_preparing_line_heading.y(),
                                                v_preparing_line_heading.x());
    for (size_t i = 0; i < max_num; i++) {
      auto start_pos = input_.tlane.pt_inside +
                       v_preparing_line_norm * input_.tlane.slot_side_sgn *
                           (0.3 + i * step + half_car_width);

      if (std::fabs(start_pos.y()) + 1 + 0.5 * apa_param.GetParam().car_width >
          std::fabs(input_.tlane.channel_y)) {
        break;
      }

      preparing_pose_vec.emplace_back(
          pnc::geometry_lib::PathPoint(start_pos, heading_rad));
    }
  }

  return true;
}

const bool ParallelPathGenerator::GenTiltedPreparingLine2ShortChannel(
    std::vector<pnc::geometry_lib::PathPoint>& preparing_pose_vec) {
  const double slot_side_sgn = calc_params_.slot_side_sgn;
  const double half_car_width = 0.5 * apa_param.GetParam().car_width;

  const double channel_width =
      std::fabs(input_.tlane.channel_y) - 0.5 * input_.tlane.slot_width;

  std::vector<double> heading_vec = {47.5, 50.0, 60.0, 40.0, 55.0};

  const double step = 0.2;
  size_t max_num = 10;

  for (const auto& heading_deg : heading_vec) {
    const double heading_rad = heading_deg * kDeg2Rad;
    const auto v_preparing_line_heading =
        pnc::geometry_lib::GenHeadingVec(heading_rad);

    const Eigen::Vector2d v_preparing_line_norm(-v_preparing_line_heading.y(),
                                                v_preparing_line_heading.x());

    Eigen::Vector2d tp(0.0, 0.0);
    for (int k = 0; k < calc_params_.inversed_path_vec_in_slot.size(); k++) {
      std::vector<Eigen::Vector2d> tangent_points;
      bool ret = pnc::geometry_lib::CalTangentLineFromHeadingAndArc(heading_rad,
        calc_params_.inversed_path_vec_in_slot[k].path_segment_vec.back().GetArcSeg(),
        tangent_points);
      if (!ret) {
        ILOG_INFO << "CalTangentLineFromHeadingAndArc failed!";
        continue;
      }
      tp = tangent_points[0];
      Eigen::Vector2d x_tmp(-1.0, 0.0);
      Eigen::Vector2d y_tmp(0.0, 1.0);
      ILOG_INFO << "tangent_points_0: " << tangent_points[0].transpose();
      for (size_t i = 0; i < max_num; i++) {
        auto start_pos = tp + x_tmp * input_.tlane.slot_side_sgn * i * step + y_tmp * 2;

        if (std::fabs(start_pos.x()) - 2.0 <
            std::fabs(input_.tlane.obs_pt_outside.x())) {
          break;
        }

        preparing_pose_vec.emplace_back(
            pnc::geometry_lib::PathPoint(start_pos, heading_rad));
      }
    }
  }

  return preparing_pose_vec.empty() ? false : true;
}

const bool ParallelPathGenerator::SelectBestPathOutsideSlot(
    const std::vector<GeometryPath>& path_vec, size_t& best_path_idx) {
  best_path_idx = std::numeric_limits<size_t>::max();
  if (path_vec.empty()) {
    ILOG_INFO << "no path in path pool!";
    return false;
  }

  double min_cost = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path_vec.size(); i++) {
    if (path_vec[i].cost_total < min_cost) {
      min_cost = path_vec[i].cost_total;
      best_path_idx = i;
    }
  }

  return true;
}

const bool ParallelPathGenerator::AssempleGeometryPath(
    GeometryPath& geometry_path,
    const std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) const {
  geometry_path.Reset();

  if (path_seg_vec.size() == 0) {
    return false;
  }
  geometry_path.path_segment_vec = path_seg_vec;

  double max_x = 0.0;
  double max_y_mag = 0.0;
  double min_heading = 0.0;

  for (size_t i = 0; i < path_seg_vec.size(); i++) {
    const auto& current_path_seg = path_seg_vec[i];
    const auto current_gear = current_path_seg.seg_gear;

    geometry_path.length += current_path_seg.Getlength();
    geometry_path.gear_cmd_vec.emplace_back(current_path_seg.seg_gear);

    if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      max_x = std::max(max_x, current_path_seg.GetEndPos().x());
      max_y_mag =
          std::max(max_y_mag, std::fabs(current_path_seg.GetEndPos().y()));
    }

    if (i > 0 && path_seg_vec[i].seg_gear != path_seg_vec[i - 1].seg_gear) {
      geometry_path.gear_change_count++;

      const auto& start_pose = path_seg_vec[i].GetStartPose();
      const bool is_overslot = start_pose.pos.x() > input_.tlane.slot_length;
      if (is_overslot) {
        geometry_path.preparing_line_heading_deg =
            start_pose.heading * kRad2Deg;
      }
    }
  }

  double current_length = 0.0f;
  uint8_t current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
  for (size_t i = 0; i < geometry_path.path_segment_vec.size(); ++i) {
    const auto& seg = geometry_path.path_segment_vec[i];
    if (i == 0) {
      current_gear = seg.seg_gear;
      current_length = seg.Getlength();
    } else {
      if (seg.seg_gear == current_gear) {
        current_length += seg.Getlength();
      } else {
        geometry_path.path_length_vec.emplace_back(current_length);
        current_gear = seg.seg_gear;
        current_length = seg.Getlength();
      }
    }
  }
  if (!geometry_path.path_segment_vec.empty()) {
    geometry_path.path_length_vec.emplace_back(current_length);
  }

  geometry_path.first_path_length = geometry_path.path_length_vec.front();

  double min_path_length = 100.0;
  for (const double& path_length : geometry_path.path_length_vec) {
    if (min_path_length < path_length) {
      min_path_length = path_length;
    }
  }
  geometry_path.shortest_path_length = min_path_length;

  geometry_path.cost[COST_WEIGHT_GEAR_SWITCH] = geometry_path.gear_change_count;
  geometry_path.cost[COST_WEIGHT_GEAR_FIRST] = 0.0;
  geometry_path.cost[COST_WEIGHT_LENGTH] = geometry_path.length;
  geometry_path.cost[COST_WEIGHT_FIRST_PATH_LENGTH] =
      geometry_path.first_path_length;

  if (geometry_path.shortest_path_length < 0.3) {
    geometry_path.cost[COST_WEIGHT_SHORT_DIST] = 1;
  }

  // is heading neg
  if (geometry_path.preparing_line_heading_deg * calc_params_.slot_side_sgn <
      -1e-3) {
    geometry_path.cost[COST_WEIGHT_ANGLE] =
        std::fabs(geometry_path.preparing_line_heading_deg);
  } else {
    geometry_path.cost[COST_WEIGHT_ANGLE] = 0.0;
  }

  geometry_path.cost[COST_WEIGHT_DIST_MAX_X] = max_x - input_.tlane.slot_length;

  geometry_path.cost[COST_WEIGHT_DIST_MAX_Y] = std::min(max_y_mag - 3.5, 0.0);
  geometry_path.cost_total = 0.0;
  for (size_t i = 0; i < COST_WEIGHT::COST_WEIGHT_MAX; i++) {
    geometry_path.cost_total += geometry_path.cost[i] * calc_params_.weights[i];
  }

  return true;
}

const bool ParallelPathGenerator::AssempleGeometryPathVec(
    std::vector<GeometryPath>& geometry_path_vec,
    const std::vector<std::vector<pnc::geometry_lib::PathSegment>>& path_vec)
    const {
  bool success = true;
  GeometryPath tmp_geo_path;
  for (const auto& path_seg : path_vec) {
    if (!AssempleGeometryPath(tmp_geo_path, path_seg)) {
      success = false;
      break;
    }
    geometry_path_vec.emplace_back(tmp_geo_path);
  }

  return success;
}

const uint8_t ParallelPathGenerator::GetMinShiftTimeInGeometryPathVec(
    const std::vector<GeometryPath>& geometry_path_vec) {
  uint8_t min_nums = 100;
  for (const auto& path : geometry_path_vec) {
    if (min_nums < path.gear_change_count) {
      min_nums = path.gear_change_count;
    }
  }
  return min_nums;
}

const bool ParallelPathGenerator::OneStepDubinsPlan(
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

const bool ParallelPathGenerator::DubinsPlan(
    std::vector<std::vector<pnc::geometry_lib::PathSegment>>& path_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const double radius,
    const double buffer, const uint8_t ref_steer, const bool force_steer_diff) {
  using namespace pnc::dubins_lib;

  path_vec.clear();
  bool success = false;

  DubinsLibrary::Input dubins_input;
  dubins_input.radius = radius;
  dubins_input.Set(start_pose.pos, target_pose.pos, start_pose.heading,
                   target_pose.heading);
  dubins_planner_.SetInput(dubins_input);

  std::vector<uint8_t> type_vec;
  switch (ref_steer) {
    case pnc::geometry_lib::SEG_STEER_RIGHT:
      type_vec = {DubinsLibrary::R_S_L};
      if (!force_steer_diff) {
        type_vec.emplace_back(DubinsLibrary::R_S_R);
      }
      break;

    case pnc::geometry_lib::SEG_STEER_LEFT:
      type_vec = {DubinsLibrary::L_S_R};
      if (!force_steer_diff) {
        type_vec.emplace_back(DubinsLibrary::L_S_L);
      }
      break;
    default:
      type_vec = {DubinsLibrary::L_S_R, DubinsLibrary::R_S_L};
      if (!force_steer_diff) {
        type_vec.emplace_back(DubinsLibrary::L_S_L);
        type_vec.emplace_back(DubinsLibrary::R_S_R);
      }
      break;
  }

  // try dubins method
  for (size_t i = 0; i < pnc::dubins_lib::DubinsLibrary::CASE_COUNT; ++i) {
    for (const auto& j : type_vec) {
      if (!dubins_planner_.Solve(j, i)) {
        continue;
      }
      if (IsDubinsCollided(kColBufferOutSlot)) {
        continue;
      }
      std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
      GetPathSegVecByDubins(path_seg_vec);
      path_vec.emplace_back(path_seg_vec);
      success = true;
    }
  }  // dubins loop

  return success;
}

const bool ParallelPathGenerator::CalcParkOutPath(
    std::vector<pnc::geometry_lib::PathSegment>& reversed_park_out_path,
    const pnc::geometry_lib::Arc& first_arc,
    const pnc::geometry_lib::LineSegment& line,
    const double park_out_target_heading) {
  reversed_park_out_path.clear();
  reversed_park_out_path.reserve(3);

  if (first_arc.length < 0.0 || line.length < 0.0) {
    // ILOG_INFO << "input length is less than 0.0!" ;
    return false;
  }

  const pnc::geometry_lib::PathPoint last_arc_start(line.pB, line.heading);

  // correct heading
  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
  if (!AlignBodyPlan(path_seg_vec, last_arc_start, park_out_target_heading,
                     pnc::geometry_lib::SEG_GEAR_DRIVE)) {
    // ILOG_INFO << "align ego body failed!" ;
    return false;
  }

  const auto& last_arc = path_seg_vec.front().GetArcSeg();
  // ILOG_INFO << "last_arc.pB: x y :" << last_arc.pB.transpose() <<
  // std::endl;
  if (last_arc.pB.x() < calc_params_.safe_circle_root_pose.pos.x() + 3.0) {
    // ILOG_INFO << "x not enough!" ;
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
    // ILOG_INFO << "line collided!" ;
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
    // ILOG_INFO << "path_seg.Getlength()" << path_seg.Getlength() <<
    // std::endl;
    if (!pnc::geometry_lib::ReversePathSegInfo(path_seg)) {
      ILOG_INFO << "reverse park out path seg error!";
      return false;
    }
  }

  reversed_park_out_path.insert(reversed_park_out_path.end(),
                                tmp_park_out_path.rbegin(),
                                tmp_park_out_path.rend());
  return true;
}

void ParallelPathGenerator::GenPathOutputByDubins() {
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

void ParallelPathGenerator::GetPathSegVecByDubins(
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

void ParallelPathGenerator::AddLastArc() {
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

const bool ParallelPathGenerator::IsDubinsCollided(const double buffer) {
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

void ParallelPathGenerator::AddPathSegToOutPut(
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

const bool ParallelPathGenerator::CheckEgoInSlot() const {
  // Todo: use slot occupied ratio
  return input_.ego_info_under_slot.slot_occupied_ratio > 0.0;
}

// search from the inside parking space to outside
const bool ParallelPathGenerator::CalMinSafeCircle() {
  const double lat_buffer = input_.tlane.is_inside_rigid ? 0.15 : 0.025;
  ILOG_INFO << "is rigid body in side of slot = "
            << input_.tlane.is_inside_rigid;
  ILOG_INFO << "lat buffer in reversed trials = " << lat_buffer;

  collision_detector_ptr_->SetParam(
      CollisionDetector::Paramters(lat_buffer, false));

  std::vector<pnc::geometry_lib::PathSegment> tra_search_out_res;
  const bool success_tra =
      InverseSearchLoopInSlot(tra_search_out_res, calc_params_.target_pose);
  debug_info_.tra_search_out_res = tra_search_out_res;

  std::vector<pnc::geometry_lib::PathSegment> adv_search_out_res;
  const bool success_adv = AdvancedInversedTrialsInSlot(
      adv_search_out_res, calc_params_.target_pose);

  std::vector<pnc::geometry_lib::PathSegment> search_out_res;
  if (apa_param.GetParam().is_parallel_advanced_method) {
    if (success_adv) {
      search_out_res = adv_search_out_res;
    } else {
      return false;
    }
  } else {
    if (success_tra) {
      search_out_res = tra_search_out_res;
    } else {
      return false;
    }
  }

  if (search_out_res.size() == 0) {
    ILOG_INFO << "search_out_res size = 0";
    return false;
  }

  ReduceRootPoseHeadingInSlot(search_out_res);

  // calc pose with ego had just cross pt_inside'y, which is used to
  // connect the prepare point

  auto& search_out_seg = search_out_res.back();

  CalcParkOutPose(search_out_seg);
  calc_params_.park_out_pose = search_out_seg.GetEndPose();
  calc_params_.safe_circle_root_pose = search_out_seg.GetStartPose();

  calc_params_.park_out_path_in_slot = search_out_res;
  // calc_params_.valid_target_pt_vec.emplace_back(
  //     calc_params_.safe_circle_root_pose);

  pnc::geometry_lib::PrintPose("corrected park out pose",
                               calc_params_.park_out_pose);
  return true;
}

const bool ParallelPathGenerator::ReduceRootPoseHeadingInSlot(
    std::vector<pnc::geometry_lib::PathSegment>& search_out_res) {
  const size_t step_length = search_out_res.size();
  if (step_length == 0) {
    ILOG_INFO << "found no parkout path!";
    return false;
  }

  if (std::fabs(search_out_res.back().GetStartHeading()) <
      kMaxParkOutRootHeading * kDeg2Rad) {
    ILOG_INFO
        << "park out heading is small enough, no need to correct heading!";
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
                              forward_steer,
                              calc_params_.lon_buffer_rev_trials)) {
      return false;
    }

    if (!CheckParkOutCornerSafeWithObsPin(forward_arc)) {
      ILOG_INFO << "current step is not able to park out!";
      return false;
    }

    // last two path should be replaced,
    search_out_res[step_length - 2] = pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::SEG_GEAR_REVERSE, backward_line);

    search_out_res[step_length - 1] = pnc::geometry_lib::PathSegment(
        forward_steer, pnc::geometry_lib::SEG_GEAR_DRIVE, forward_arc);
    ILOG_INFO << "replace last two steps success!";
  }
  pnc::geometry_lib::PrintPose("opt root pose",
                               search_out_res.back().GetStartPose());

  return true;
}

const bool ParallelPathGenerator::CalcParkOutPose(
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
  const double max_park_out_heading = kMaxParkOutFirstArcHeading * kDeg2Rad;

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

const bool ParallelPathGenerator::ReversePathSegVec(
    std::vector<pnc::geometry_lib::PathSegment>& park_out_res) {
  if (park_out_res.size() < 1) {
    return false;
  }

  for (auto& path_seg : park_out_res) {
    pnc::geometry_lib::ReversePathSegInfo(path_seg);
  }
  std::reverse(park_out_res.begin(), park_out_res.end());
  return true;
}

const bool ParallelPathGenerator::InverseSearchLoopInSlot(
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
                           forward_steer, calc_params_.lon_buffer_rev_trials)) {
    if (CheckParkOutCornerSafeWithObsPin(forward_arc)) {
      search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
          forward_steer, pnc::geometry_lib::SEG_GEAR_DRIVE, forward_arc));
      ILOG_INFO << "ego can park out at first!";
      return true;
    }
  }

  // calc backward limit
  pnc::geometry_lib::LineSegment first_line_step;
  first_line_step.pA = terminal_pose.pos;
  first_line_step.heading = terminal_pose.heading;
  if (!CalcLineStepLimitPose(first_line_step,
                             pnc::geometry_lib::SEG_GEAR_REVERSE,
                             calc_params_.lon_buffer_rev_trials)) {
    ILOG_INFO << "CalcLineStepLimitPose error!";
    return false;
  }

  const pnc::geometry_lib::PathPoint back_line_limit(first_line_step.pB,
                                                     first_line_step.heading);

  if (!CheckSamePose(back_line_limit, terminal_pose)) {
    ILOG_INFO << "need backward step, pose =" << back_line_limit.pos.x() << ", "
              << back_line_limit.pos.y() << ", "
              << back_line_limit.heading * kRad2Deg;

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
                              forward_steer,
                              calc_params_.lon_buffer_rev_trials)) {
      ILOG_INFO << "calc forward arc limit error!";
      break;
    }

    search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
        forward_steer, pnc::geometry_lib::SEG_GEAR_DRIVE, forward_arc));

    // ego can park out in forward step, return true
    if (CheckParkOutCornerSafeWithObsPin(forward_arc)) {
      loop_success = true;
      ILOG_INFO << "find park out pose!";
      break;
    }

    ILOG_INFO << "forward limit pose is valid\n";
    ILOG_INFO << "------ backward step --------";

    pnc::geometry_lib::Arc backward_arc;
    backward_arc.pA = forward_arc.pB;
    backward_arc.headingA = forward_arc.headingB;
    backward_arc.circle_info.radius = radius;

    if (!CalcArcStepLimitPose(backward_arc, pnc::geometry_lib::SEG_GEAR_REVERSE,
                              backward_steer,
                              calc_params_.lon_buffer_rev_trials)) {
      ILOG_INFO << "calc backward arc limit error!";
      break;
    }

    // start calc backward limit pose
    pnc::geometry_lib::PathPoint backward_limit_pose(backward_arc.pB,
                                                     backward_arc.headingB);

    // ILOG_INFO << "backward limit pose = ";
    // pnc::geometry_lib::PrintPose(backward_arc.pB, backward_arc.headingB);
    // ILOG_INFO <<"arc length = " << backward_arc.length);

    search_out_res.emplace_back(pnc::geometry_lib::PathSegment(
        backward_steer, pnc::geometry_lib::SEG_GEAR_REVERSE, backward_arc));

    start_pose_in_loop = backward_limit_pose;
  }

  if (!loop_success) {
    search_out_res.clear();
  }

  return loop_success;
}

const bool ParallelPathGenerator::SortPathByGearShiftHeadingAndLength(
    const std::vector<GeometryPath>& total_path_vec,
    std::vector<GeometryPath>& sorted_path_vec) {
  if (total_path_vec.empty()) {
    ILOG_INFO << "input path vector is empty!";
    return false;
  }

  // 复制路径向量用于排序
  sorted_path_vec = total_path_vec;

  // 为每个路径计算停车时的航向角（度）
  for (auto& path : sorted_path_vec) {
    path.park_out_heading_deg =
        std::fabs(path.path_segment_vec.back().GetStartHeading() * kRad2Deg);
  }

  // 按照以下优先级排序：
  // 1. 换挡次数少的优先
  // 2. 停车时航向角小的优先（相差1度以内视为相同优先级）
  // 3. 路径总长度短的优先
  std::sort(sorted_path_vec.begin(), sorted_path_vec.end(),
            [](const GeometryPath& path_a, const GeometryPath& path_b) {
              // 首先比较换挡次数
              if (path_a.gear_change_count != path_b.gear_change_count) {
                return path_a.gear_change_count < path_b.gear_change_count;
              }

              // 换挡次数相同时，比较停车时的航向角（允许1度的误差）
              const double heading_diff = std::fabs(
                  path_a.park_out_heading_deg - path_b.park_out_heading_deg);
              if (heading_diff > 1.0) {
                return path_a.park_out_heading_deg <
                       path_b.park_out_heading_deg;
              }

              // 航向角相近时，比较路径总长度
              return path_a.length < path_b.length;
            });

  ILOG_INFO << "sort paths completed, total path count: "
            << sorted_path_vec.size();

  // 为了降低运算，此处选择最优和最靠后的路径，来解决短通道无法行驶到最优路径的问题。
  std::vector<GeometryPath> selected_path_vec;
  if (sorted_path_vec.size() > 1) {
    const double base_x =
        sorted_path_vec.front().path_segment_vec.back().GetStartPos().x();
    double min_x = base_x;
    size_t min_x_idx = 0;

    for (size_t i = 1; i < sorted_path_vec.size(); ++i) {
      const double curr_x =
          sorted_path_vec[i].path_segment_vec.back().GetStartPos().x();
      if (curr_x < min_x) {
        min_x = curr_x;
        min_x_idx = i;
      }
    }

    const double kMaxHeadingDeg = 65.0;
    int max_heading_id = 0;
    double base_heading = sorted_path_vec.front().park_out_heading_deg;
    for (int i = 1; i < sorted_path_vec.size(); ++i) {
      double cur_heading = sorted_path_vec[i].park_out_heading_deg;
      if (cur_heading > base_heading && cur_heading < kMaxHeadingDeg) {
        max_heading_id = i;
      }
    }

    constexpr double kXDiffThreshold = 0.4;
    if ((base_x - min_x) > kXDiffThreshold) {
      selected_path_vec = {sorted_path_vec.front(), sorted_path_vec[min_x_idx]};
    } else {
      selected_path_vec = {sorted_path_vec.front()};
    }
    if (sorted_path_vec[max_heading_id].park_out_heading_deg - sorted_path_vec.front().park_out_heading_deg > 1.0) {
      selected_path_vec.emplace_back(sorted_path_vec[max_heading_id]);
    }
  }
  // std::cout << std::endl;
  // std::cout << "gear    = ";
  // for (const auto& path : sorted_path_vec) {
  //   std::cout << std::setw(10) << path.gear_change_count;
  // }
  // std::cout << std::endl;

  debug_info_.debug_inslot_path_vec = selected_path_vec;
  sorted_path_vec = selected_path_vec;
  // std::cout << "heading = ";
  // for (const auto& path : sorted_path_vec) {
  //   std::cout << std::setw(10) << path.park_out_heading_deg;
  // }
  // std::cout << std::endl;

  // std::cout << "x       = ";
  // for (const auto& path : sorted_path_vec) {
  //   std::cout << std::setw(10)
  //             << path.path_segment_vec.back().GetStartPose().pos.x();
  // }
  // std::cout << std::endl;

  // std::cout << "y       = ";
  // for (const auto& path : sorted_path_vec) {
  //   std::cout << std::setw(10)
  //             << path.path_segment_vec.back().GetStartPose().pos.y();
  // }
  // std::cout << std::endl;
  // std::cout << "heading = ";
  // for (const auto& path : selected_path_vec) {
  //   std::cout << std::setw(10) << path.park_out_heading_deg;
  // }
  // std::cout << std::endl;
  return true;
}

const bool ParallelPathGenerator::AdvancedInversedTrialsInSlot(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& target_pose) {
  ILOG_INFO << "---------AdvancedInversedTrialsInSlot --------------";
  ILOG_INFO << "calc_params_.lon_buffer_rev_trials = "
            << calc_params_.lon_buffer_rev_trials;
  using namespace pnc::geometry_lib;

  std::vector<Eigen::Vector2d> line_step_vec;
  if (!GenLineStepValidEnd(line_step_vec, target_pose)) {
    ILOG_INFO << "cal first step failed!";
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
        // ILOG_INFO <<"InversedTrialsByGivenGear failed with x = "
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
          // ILOG_INFO <<"calc back limit path success! " << start_pos.x());
          debug_path_idx = total_path_vec.size();
        }
      }

      GeometryPath geo_path;
      AssempleGeometryPath(geo_path, path_seg_vec);
      // if (geo_path.first_path_length < 0.2) {
      //   // ILOG_INFO <<"first path length is too short!");
      //   continue;
      // }
      // ILOG_INFO <<"geo_path gear change cnt = " <<
      // geo_path.gear_change_count);
      total_path_vec.emplace_back(geo_path);
      success_cnt++;
      gear == SEG_GEAR_DRIVE ? ++dirve_success_cnt : ++reverse_success_cnt;
    }
    // if (cal_cnt > 1) {
    //   break;
    // }
  }

  ILOG_INFO << "total_cnt = " << total_cnt;
  ILOG_INFO << "success_cnt = " << success_cnt;
  ILOG_INFO << "dirve_success_cnt = " << dirve_success_cnt;
  ILOG_INFO << "reverse_success_cnt = " << reverse_success_cnt;
  // ILOG_INFO <<"rear_limit_fail_cnt = " << rear_limit_fail_cnt);
  // ILOG_INFO <<"front_limit_fail_cnt = " << front_limit_fail_cnt);
  ILOG_INFO << "calc fail cnt = " << calc_fail_cnt;
  // ILOG_INFO <<"success x vec -------");

  if (success_x_vec.size() == 0) {
    ILOG_INFO << "no path success!";
    return false;
  }

  if (!SortPathByGearShiftHeadingAndLength(
          total_path_vec, calc_params_.inversed_path_vec_in_slot)) {
    ILOG_INFO << "sort failed!";
    return false;
  }

  path_seg_vec =
      calc_params_.inversed_path_vec_in_slot.front().path_segment_vec;

  for (const auto& path : calc_params_.inversed_path_vec_in_slot) {
    calc_params_.valid_target_pt_vec.emplace_back(
        path.path_segment_vec.back().GetStartPose());
  }

  return true;
}

const size_t ParallelPathGenerator::CalPathGearChangeCounts(
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

const bool ParallelPathGenerator::GenLineStepValidEnd(
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
                           forward_steer, calc_params_.lon_buffer_rev_trials)) {
    if (CheckParkOutCornerSafeWithObsPin(forward_arc)) {
      gear_vec.erase(gear_vec.begin());
      ILOG_INFO << "ego can park out at first, no need use backward loop!";
    }
  }

  auto first_line = GetEgoHeadingLine(target_pose.pos, target_pose.heading);
  for (const auto& gear : gear_vec) {
    if (!CalcLineStepLimitPose(first_line, gear,
                               calc_params_.lon_buffer_rev_trials)) {
      ILOG_INFO << "gear = " << gear << " failed!";
      continue;
    }

    if (CheckSamePos(first_line.pB, target_pose.pos)) {
      ILOG_INFO << "first_line limit == target pose failed!";
      continue;
    }

    ILOG_INFO << "Line limit = " << first_line.pB.transpose();

    double line_length = (first_line.pB - target_pose.pos).norm();
    if (line_length < 0.1) {
      ILOG_INFO << "line_length" << line_length
                << "smaller than min line length! gear == "
                << static_cast<int>(gear);
      continue;
    }

    int step_size = static_cast<int>(line_length / kLineStepLength);
    int max_size = static_cast<int>(line_length / 0.1);
    if (gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      line_length = std::min(line_length, 0.6);
      step_size = mathlib::Clamp(step_size, 2, 3);
      step_size = std::min(step_size, max_size);

    } else {
      line_length = std::min(line_length, 1.0);
      step_size = mathlib::Clamp(step_size, 2, 3);
      step_size = std::min(step_size, max_size);
    }

    const double step = line_length / step_size;
    // double length_diff = step;
    const double dir_sgn = (gear == SEG_GEAR_DRIVE ? 1.0 : -1.0);

    for (size_t i = 1; i <= step_size; i++) {
      const Eigen::Vector2d line_end_pos =
          target_pose.pos + dir_sgn * i * step * v_heading;
      line_step_vec.emplace_back(std::move(line_end_pos));
    }
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
    ILOG_INFO << "line step pos ";
    for (const auto& pos : line_end_vec) {
      ILOG_INFO << pos.x() << "  ";
    }
    ILOG_INFO;
  }

  return success;
}

const bool ParallelPathGenerator::InversedTrialsByGivenGear(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const uint8_t current_gear) {
  using namespace pnc::geometry_lib;
  ILOG_INFO << "InversedTrialsByGivenGear, start_pos"
            << start_pose.pos.transpose();
  uint8_t ref_gear = current_gear;
  uint8_t ref_steer = SEG_STEER_RIGHT;
  if ((ref_gear == SEG_GEAR_DRIVE && input_.tlane.slot_side_sgn > 0.0) ||
      (ref_gear == SEG_GEAR_REVERSE && input_.tlane.slot_side_sgn < 0.0)) {
    ref_steer = SEG_STEER_LEFT;
  }

  // ILOG_INFO << "slot side sgn = " << input_.tlane.slot_side_sgn;
  // ILOG_INFO << "current_gear = " << static_cast<int>(ref_gear);
  // ILOG_INFO << "current_steer = " << static_cast<int>(ref_steer);

  // check if ego is able to park out at start pose
  pnc::geometry_lib::Arc arc;
  arc.pA = start_pose.pos;
  arc.headingA = start_pose.heading;
  arc.circle_info.radius = apa_param.GetParam().min_turn_radius;

  std::vector<pnc::geometry_lib::PathSegment> search_out_res;
  search_out_res.reserve(kMaxParallelParkInSegmentNums);

  bool success = false;
  for (size_t i = 0; i <= kMaxPathNumsInSlot + 1; i += 1) {
    if (!CalcArcStepLimitPose(arc, ref_gear, ref_steer,
                              calc_params_.lon_buffer_rev_trials)) {
      ILOG_INFO << "calc arc limit error!";
      break;
    }

    search_out_res.emplace_back(
        pnc::geometry_lib::PathSegment(ref_steer, ref_gear, arc));

    if (ref_gear == SEG_GEAR_DRIVE && CheckParkOutCornerSafeWithObsPin(arc)) {
      ILOG_INFO << "is_dirve_out_safe success";
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
    ILOG_INFO << "success !";
    path_seg_vec = search_out_res;
  } else {
    ILOG_INFO << "failed !";
  }

  return success;
}

const bool ParallelPathGenerator::CalcLineStepLimitPose(
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
    ILOG_INFO << "fault gear type!";
    return false;
  }

  const double line_dist_limit =
      input_.ego_info_under_slot.slot_occupied_ratio > 0.0 ? 1.5 : 4.0;

  const Eigen::Vector2d rough_limit_pt =
      start_pose.pos + line_dist_limit * dirve_sgn *
                           Eigen::Vector2d(std::cos(start_pose.heading),
                                           std::sin(start_pose.heading));

  pnc::geometry_lib::PathSegment line_path(
      gear, pnc::geometry_lib::LineSegment(start_pose.pos, rough_limit_pt,
                                           start_pose.heading));

  const auto& col_res = TrimPathByCollisionDetection(line_path, buffer);
  if (col_res == PATH_COL_INVALID) {
    ILOG_INFO << "line collided at start pos!";
    return false;
  }

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

const bool ParallelPathGenerator::CalcArcStepLimitPose(
    pnc::geometry_lib::Arc& arc, const uint8_t gear, const uint8_t steer,
    const double buffer) {
  // start pose and radius should be given in arc

  if (arc.circle_info.radius <
      apa_param.GetParam().min_turn_radius - apa_param.GetParam().radius_eps) {
    ILOG_INFO << "radius fault!";
    return false;
  }

  if (gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
      gear != pnc::geometry_lib::SEG_GEAR_REVERSE) {
    ILOG_INFO << "arc fault gear type!";
    return false;
  }

  if (steer != pnc::geometry_lib::SEG_STEER_RIGHT &&
      steer != pnc::geometry_lib::SEG_STEER_LEFT) {
    ILOG_INFO << "arc fault steer type!";
    return false;
  }

  pnc::geometry_lib::PathPoint start_pose(arc.pA, arc.headingA);

  arc.circle_info.center =
      CalEgoTurningCenter(start_pose, arc.circle_info.radius, steer);

  if (!pnc::geometry_lib::CalcArcDirection(arc.is_anti_clockwise, gear,
                                           steer)) {
    ILOG_INFO << "arc or steer error!";
    return false;
  }

  const double arc_length_limit = 6.0;
  if (!pnc::geometry_lib::CompleteArcInfo(arc, arc_length_limit,
                                          arc.is_anti_clockwise)) {
    // ILOG_INFO << "CompleteArcInfo error!";
    return false;
  }

  pnc::geometry_lib::PathSegment arc_path(steer, gear, arc);
  const auto& col_res = TrimPathByCollisionDetection(arc_path, buffer);
  // ILOG_INFO << "forward_col_pt =" << forward_col_pt.transpose() <<
  // std::endl;

  arc.is_ignored = true;
  // get updated arc info of collision free
  if (col_res == PATH_COL_NORMAL || col_res == PATH_COL_SHORTEN) {
    // ILOG_INFO << "arc_path.GetArcSeg().length = "
    //           << arc_path.GetArcSeg().length;
    if (arc_path.GetArcSeg().length >= 0.05) {
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
  // ILOG_INFO << "arc.is_ignored = " << arc.is_ignored;
  return (!arc.is_ignored);
}

const bool ParallelPathGenerator::CheckParkOutCornerSafeWithObsPin(
    const pnc::geometry_lib::Arc& first_arc) const {
  double center_to_obs_in = 100.0;

  // ILOG_INFO <<"center_to_obs_in = " << center_to_obs_in);
  // ILOG_INFO <<
  //     "max_corner_radius = " <<
  //     calc_params_.min_outer_front_corner_radius);

  double virtual_obs_y_lim = input_.tlane.pt_inside.y();

  for (const auto& virtual_obs_pt : calc_params_.front_corner_obs_vec) {
    center_to_obs_in =
        std::min(center_to_obs_in,
                 (virtual_obs_pt - first_arc.circle_info.center).norm());

    if (calc_params_.is_left_side) {
      virtual_obs_y_lim = std::min(virtual_obs_y_lim, virtual_obs_pt.y());
    } else {
      virtual_obs_y_lim = std::max(virtual_obs_y_lim, virtual_obs_pt.y());
    }
  }
  // ILOG_INFO << "corner remain dist = "
  //           << center_to_obs_in -
  //           calc_params_.min_outer_front_corner_radius;

  const bool corner_safe =
      center_to_obs_in >=
      calc_params_.min_outer_front_corner_radius +
          apa_param.GetParam().parallel_ego_front_corner_to_obs_in_buffer;

  // ILOG_INFO << "corner_safe = " << corner_safe;

  pnc::geometry_lib::LocalToGlobalTf l2g_tf(first_arc.pB, first_arc.headingB);

  auto park_out_corner = calc_params_.v_ego_farest_front_corner;
  park_out_corner.y() *= -calc_params_.slot_side_sgn;

  park_out_corner = l2g_tf.GetPos(park_out_corner);

  const bool is_corner_out =
      park_out_corner.y() * calc_params_.slot_side_sgn >
              virtual_obs_y_lim * calc_params_.slot_side_sgn
          ? true
          : false;

  // ILOG_INFO << "park_out_corner.y() = " << park_out_corner.y()
  //           << "   virtual_obs_y_lim = " << virtual_obs_y_lim;
  // ILOG_INFO << "is_corner_out = " << is_corner_out;

  return corner_safe && is_corner_out;
}

const bool ParallelPathGenerator::TwoSameGearArcPlanToLine(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::LineSegment& target_line, const uint8_t gear,
    const double buffer) {
  return TwoSameGearArcPlanToLine(path_seg_vec, start_pose, target_line, gear,
                                  apa_param.GetParam().min_turn_radius, buffer);
}

const bool ParallelPathGenerator::TwoSameGearArcPlanToLine(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::LineSegment& target_line, const uint8_t gear,
    const double radius, const double buffer) {
  path_seg_vec.clear();
  path_seg_vec.reserve(4);

  if (!pnc::geometry_lib::IsValidGear(gear)) {
    ILOG_INFO << "gear invalid!";
    return false;
  }

  pnc::geometry_lib::Arc arc_1;
  pnc::geometry_lib::Arc arc_2;
  arc_1.pA = start_pose.pos;
  arc_1.headingA = start_pose.heading;
  arc_1.circle_info.radius = radius;

  ILOG_INFO << "target line =" << target_line.pA.transpose()
            << ", heading deg = " << target_line.heading * kRad2Deg;

  const uint8_t arc_1_steer = (pnc::geometry_lib::IsPointOnLeftSideOfLineSeg(
                                   start_pose.pos, target_line)
                                   ? pnc::geometry_lib::SEG_STEER_RIGHT
                                   : pnc::geometry_lib::SEG_STEER_LEFT);

  arc_1.circle_info.center =
      CalEgoTurningCenter(start_pose, radius, arc_1_steer);

  auto tmp_target_line = target_line;
  if (!pnc::geometry_lib::CalTwoSameGearArcWithLine(arc_1, arc_2,
                                                    tmp_target_line, gear)) {
    ILOG_INFO << "CalTwoSameGearArcWithLine failed!";
    return false;
  }

  auto col_res = collision_detector_ptr_->UpdateByObsMap(arc_1, arc_1.headingA);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
    ILOG_INFO << "col pt = " << col_res.col_pt_obs_global.transpose();
    // debug_info_.debug_arc_vec.emplace_back(arc_1);
    ILOG_INFO << "TwoSameGearArcPlanToLine arc1 collided!";
    return false;
  }
  col_res = collision_detector_ptr_->UpdateByObsMap(arc_2, arc_2.headingA);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
    ILOG_INFO << "arc2 collided!";
    return false;
  }

  pnc::geometry_lib::LineSegment last_line(arc_2.pB, target_line.pA,
                                           arc_2.headingB);
  auto last_line_gear = pnc::geometry_lib::CalLineSegGear(last_line);
  col_res =
      collision_detector_ptr_->UpdateByObsMap(last_line, last_line.heading);
  if (col_res.collision_flag ||
      col_res.remain_car_dist > col_res.remain_obstacle_dist - buffer) {
    ILOG_INFO << "arc2 end to line key pt collided!";
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
      ILOG_INFO << "arc 2 extend line collided!";
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

const bool ParallelPathGenerator::RSCurvePlan(
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
            // ILOG_INFO << "dubins success!" ;
            return true;
          }
        }
      }
    }
  }
  // ILOG_INFO << "dubins failed!" ;
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
          // ILOG_INFO << "line arc success!" ;
          return true;
        }
      }
    }
  }

  return false;
}

// multi plan start
const bool ParallelPathGenerator::MultiPlan() {
  ILOG_INFO << "-----multi plan-----\n";
  // set init state
  pnc::geometry_lib::PathPoint current_pose =
      input_.ego_info_under_slot.cur_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;
  ILOG_INFO << "multi-plan ref gear =" << static_cast<int>(current_gear);
  ILOG_INFO << "multi-plan ref arc = " << static_cast<int>(current_arc_steer);

  // check pose and ego_info_under_slot.slot_occupied_ratio, if error is
  // small, multi isn't suitable
  if (!CheckMultiPlanSuitable(current_pose)) {
    ILOG_INFO << "pose err is relatively small, skip multi plan, directly try "
                 "adjust plan\n";
    return false;
  }
  // Todo: remove ref arc, gear is enough.
  // check gear
  if (!pnc::geometry_lib::IsValidGear(current_gear)) {
    ILOG_INFO << "ref_gear error\n";
    return false;
  }

  ILOG_INFO << "try multi plan to target point\n";
  bool success = false;
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  for (size_t i = 0; i < kMaxMultiStepNums; ++i) {
    ILOG_INFO << "-------- No." << i << " in multi-plan--------\n";

    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);
    success =
        CalSinglePathInMulti(current_pose, calc_params_.target_pose,
                             current_gear, current_arc_steer, tmp_path_seg_vec);
    if (!success) {
      ILOG_INFO << "single path of multi-plan failed!\n\n";
      // output_.Reset();
      break;
    }
    output_.path_available = true;
    AddPathSegToOutPut(tmp_path_seg_vec);

    current_gear = pnc::geometry_lib::ReverseGear(current_gear);
    current_arc_steer = pnc::geometry_lib::ReverseSteer(current_arc_steer);
    current_pose = tmp_path_seg_vec.back().GetEndPose();

    if (IsOnTarget(current_pose)) {
      ILOG_INFO << "already plan to target pos!\n\n";
      break;
    }
  }
  ILOG_INFO << "multi seg size =" << output_.path_segment_vec.size();
  return success;
}

// checkout multi suitable
const bool ParallelPathGenerator::CheckMultiPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose) const {
  // apa_param.GetParam().multi_plan_min_heading_err
  if (std::fabs(current_pose.heading) <= 5.0 * kDeg2Rad) {
    return false;
  }
  return true;
}

const bool ParallelPathGenerator::CalSinglePathInMulti(
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const uint8_t current_gear,
    const uint8_t current_arc_steer,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  ILOG_INFO << "-----CalSinglePathInMulti-----";
  ILOG_INFO << "current_arc_steer = " << static_cast<int>(current_arc_steer)
            << ",  current_gear = " << static_cast<int>(current_gear)
            << ",  current_pos = " << current_pose.pos.transpose()
            << ",  current_heading = " << current_pose.heading * kRad2Deg;

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
    ILOG_INFO << "center to line dist = " << dist << ",  try OneArcPlan\n";
    if (OneArcPlan(current_arc, tmp_path_seg_vec, target_line, current_gear,
                   current_arc_steer)) {
      // ILOG_INFO << "OneArcPlan success\n";
      success = true;
      multi_plan_method = MultiPlanMethod::OneArcMultiPlan;
    } else {
      // ILOG_INFO << "OneArcPlan fail\n";
      success = false;
    }
  } else if (dist <
             min_radius - apa_param.GetParam().parallel_multi_plan_radius_eps) {
    // circle and line are intersected, try second
    ILOG_INFO << "center to line dist = " << dist << ",  try TwoArcPlan\n";
    if (TwoArcPlan(current_arc, tmp_path_seg_vec, target_line, current_gear,
                   current_arc_steer)) {
      ILOG_INFO << "TwoArcPlan success\n";
      success = true;
      multi_plan_method = MultiPlanMethod::TwoArcMultiPlan;
    } else {
      ILOG_INFO << "TwoArcPlan fail\n";
      success = false;
    }
  } else if (dist >
             min_radius + apa_param.GetParam().parallel_multi_plan_radius_eps) {
    // circle and line are disjoint, try last
    ILOG_INFO << "center to line dist = " << dist << ",  try LineArcPlan\n";
    if (LineArcPlan(current_arc, tmp_path_seg_vec, target_line, current_gear,
                    current_arc_steer)) {
      ILOG_INFO << "LineArcPlan success\n";
      success = true;
      multi_plan_method = MultiPlanMethod::LineArcMultiPlan;
    } else {
      ILOG_INFO << "LineArcPlan fail\n";
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
    // ILOG_INFO << "last path pose to one plan\n";
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
    col_res = TrimPathByCollisionDetection(tmp_path_seg,
                                           calc_params_.lon_buffer_rev_trials);

    if (col_res == PATH_COL_NORMAL) {
      path_seg_vec.emplace_back(tmp_path_seg);
      ILOG_INFO << "No. " << i << " normal!";
    } else if (col_res == PATH_COL_SHORTEN) {
      path_seg_vec.emplace_back(tmp_path_seg);
      ILOG_INFO << "No. " << i << " cut, due to collision, end point ="
                << tmp_path_seg.GetEndPos().transpose()
                << ", heading =" << tmp_path_seg.GetEndHeading() * kRad2Deg;
      if (multi_plan_method == MultiPlanMethod::LineArcMultiPlan && i == 0) {
        ILOG_INFO << "line-arc cut at line, quit multi-plan";
        return false;
      }
      break;
    } else if (col_res == PATH_COL_INVALID) {
      ILOG_INFO << "path col at start pose, invalid!";
      break;
    }
  }

  if (col_res == PATH_COL_INVALID) {
    ILOG_INFO << "start loose buffer to 0.15 in slot!";
    for (size_t i = 0; i < tmp_path_seg_vec.size(); i++) {
      auto& tmp_path_seg = tmp_path_seg_vec[i];
      col_res =
          TrimPathByCollisionDetection(tmp_path_seg, kColSmallBufferInSlot);

      if (col_res == PATH_COL_NORMAL) {
        path_seg_vec.emplace_back(tmp_path_seg);
        ILOG_INFO << "No. " << i << " normal!";
      } else if (col_res == PATH_COL_SHORTEN) {
        path_seg_vec.emplace_back(tmp_path_seg);
        ILOG_INFO << "No. " << i << " cut, due to collision, end point ="
                  << tmp_path_seg.GetEndPos().transpose()
                  << ", heading =" << tmp_path_seg.GetEndHeading() * kRad2Deg;
        if (multi_plan_method == MultiPlanMethod::LineArcMultiPlan && i == 0) {
          ILOG_INFO << "line-arc cut at line, quit multi-plan";
          return false;
        }
        break;
      } else if (col_res == PATH_COL_INVALID) {
        ILOG_INFO << "path col at start pose, invalid!";
        break;
      }
    }
  }

  success = false;
  if (path_seg_vec.size() > 0) {
    ILOG_INFO << "CalSinglePathInMulti success, single path in multi:";
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
      ILOG_INFO << "path is too short, length = " << first_path_length;
    } else {
      success = true;
    }
  } else {
    ILOG_INFO << "CalSinglePathInMulti fail";
    success = false;
  }
  return success;
}

const bool ParallelPathGenerator::MultiAlignBody() {
  ILOG_INFO << "-----MultiAlignBodyPlan-----";

  // 初始化状态
  uint8_t current_gear = input_.ref_gear;
  ILOG_INFO << "ref gear = " << static_cast<int>(current_gear);

  pnc::geometry_lib::PathPoint current_pose =
      input_.ego_info_under_slot.cur_pose;
  pnc::geometry_lib::PrintPose("start pose", current_pose);

  if (!pnc::geometry_lib::IsValidGear(current_gear)) {
    ILOG_ERROR << "ref_gear error!";
    return false;
  }

  if (input_.ego_info_under_slot.slot_occupied_ratio < 0.3) {
    ILOG_INFO << "not in slot!";
    return false;
  }

  std::vector<pnc::geometry_lib::PathSegment> single_aligned_path;
  std::vector<pnc::geometry_lib::PathSegment> path_res;
  path_res.reserve(kMaxMultiStepNums);

  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.0, false));

  bool success = false;
  const int max_iter = 20;
  int i = 0;

  while (std::fabs(current_pose.heading * kRad2Deg) > 1.0 && i++ < max_iter) {
    ILOG_INFO << "-------No. " << i;
    single_aligned_path.clear();
    single_aligned_path.reserve(1);

    const double heading_deg = std::fabs(current_pose.heading * kRad2Deg);
    double max_length = (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE &&
                         heading_deg > kMinInclineHeadingDeg)
                            ? kMaxDriveLengthInclineInSlot
                            : 10.0;

    if (!AlignBodyPlan(single_aligned_path, current_pose,
                       calc_params_.target_pose.heading, current_gear,
                       max_length)) {
      ILOG_ERROR << "AlignBodyPlan failed, exiting";
      break;
    }

    // 两次 buffer 尝试
    auto col_res =
        TrimPathByCollisionDetection(single_aligned_path.back(), 0.15);
    if (col_res == PATH_COL_INVALID) {
      ILOG_WARN << "first buffer failed, trying fallback buffer";
      col_res = TrimPathByCollisionDetection(single_aligned_path.back(), 0.1);
    }

    if (col_res == PATH_COL_SHORTEN || col_res == PATH_COL_NORMAL) {
      path_res.emplace_back(single_aligned_path.back());
      current_pose = single_aligned_path.back().GetEndPose();
      current_gear = pnc::geometry_lib::ReverseGear(current_gear);
      success = true;
    } else {
      ILOG_ERROR << "Collision check failed twice, exiting";
      success = false;
      break;
    }
  }

  if (success && !path_res.empty()) {
    // 添加最终 heading 对齐 segment（横向拉直）
    Eigen::Vector2d last_pt_start = path_res.back().GetEndPos();
    Eigen::Vector2d last_pt_end(calc_params_.target_pose.pos.x(),
                                last_pt_start.y());
    pnc::geometry_lib::LineSegment last_line(last_pt_start, last_pt_end,
                                             calc_params_.target_pose.heading);
    pnc::geometry_lib::PathSegment last_line_path(
        pnc::geometry_lib::CalLineSegGear(last_line), last_line);

    if (last_line_path.seg_gear == path_res.back().seg_gear ||
        (last_line_path.seg_gear != path_res.back().seg_gear &&
         last_line_path.GetLineSeg().length >=
             apa_param.GetParam().min_line_length)) {
      path_res.emplace_back(last_line_path);
    }

    output_.Reset();
    AddPathSegToOutPut(path_res);
  } else {
    ILOG_INFO << "MultiAlign failed or no valid path generated.";
  }

  return success;
}

// adjust plan start
const bool ParallelPathGenerator::AdjustPlan() {
  ILOG_INFO << "-----adjust plan-----";
  // set init state
  pnc::geometry_lib::PathPoint current_pose =
      input_.ego_info_under_slot.cur_pose;
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;

  if (output_.path_segment_vec.size() > 0 && output_.gear_cmd_vec.size() > 0) {
    const auto& last_seg = output_.path_segment_vec.back();
    current_pose.Set(last_seg.GetEndPos(), last_seg.GetEndHeading());
    current_gear = pnc::geometry_lib::ReverseGear(last_seg.seg_gear);
    current_arc_steer = pnc::geometry_lib::ReverseSteer(last_seg.seg_steer);
    ILOG_INFO << "continue to plan after multi\n";
  }

  ILOG_INFO << "adjust plan input gear =" << static_cast<int>(current_gear);

  ILOG_INFO << "adjust plan input steer ="
            << static_cast<int>(current_arc_steer);

  ILOG_INFO << "current pose =" << current_pose.pos.transpose() << ", "
            << current_pose.heading * kRad2Deg;

  // check pose, if error is large, adjust is not suitable
  // if (!CheckAdjustPlanSuitable(current_pose)) {
  //   ILOG_INFO <<"pose err is relatively large, skip adjust plan, plan
  //   fail"); return false;
  // }

  // check gear and steer
  if (!pnc::geometry_lib::IsValidGear(current_gear) ||
      !pnc::geometry_lib::IsValidArcSteer(current_arc_steer)) {
    ILOG_INFO << "ref_gear or ref_arc_steer error";
    return false;
  }

  // ILOG_INFO <<"try adjust plan to target point");
  bool success = false;
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;

  for (size_t i = 0; i < kMaxPathNumsInSlot; ++i) {
    ILOG_INFO << "-------- No." << i << " in adjust-plan--------";
    tmp_path_seg_vec.clear();
    tmp_path_seg_vec.reserve(3);

    if (!CalSinglePathInAdjust(tmp_path_seg_vec, current_pose, current_gear,
                               1.0, apa_param.GetParam().min_turn_radius)) {
      ILOG_INFO << "single path of adjust plan failed!";
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
    ILOG_INFO << "adjust plan failed!";
  }

  return success;
}

// adjust plan start
const bool ParallelPathGenerator::ParallelAdjustPlan() {
  ILOG_INFO << "-----prallel adjust plan-----";
  // set init state
  input_.ego_info_under_slot.cur_pose.heading =
      pnc::geometry_lib::NormalizeAngle(
          input_.ego_info_under_slot.cur_pose.heading);

  auto current_gear = input_.ref_gear;
  auto current_pose = input_.ego_info_under_slot.cur_pose;
  pnc::geometry_lib::PrintPose("input pose =",
                               input_.ego_info_under_slot.cur_pose);

  // dirve backward directly if can park near target pose
  pnc::geometry_lib::LineSegment first_line;
  first_line.pA = input_.ego_info_under_slot.cur_pose.pos;
  first_line.heading = input_.ego_info_under_slot.cur_pose.heading;

  if (OneLinePlan(first_line, calc_params_.target_pose)) {
    ILOG_INFO << "firstly calc line success";
    AddPathSegToOutPut(pnc::geometry_lib::PathSegment(
        pnc::geometry_lib::CalLineSegGear(first_line), first_line));
    return true;
  }

  if (output_.path_segment_vec.size() > 0) {
    const auto& last_seg = output_.path_segment_vec.back();
    current_pose.Set(last_seg.GetEndPos(), last_seg.GetEndHeading());
    current_gear = pnc::geometry_lib::ReverseGear(last_seg.seg_gear);
    ILOG_INFO << "continue to plan after multi\n";
  }

  ILOG_INFO << "current pose =" << current_pose.pos.transpose() << ", "
            << current_pose.heading * kRad2Deg;

  // check gear and steer
  if (!pnc::geometry_lib::IsValidGear(current_gear)) {
    ILOG_INFO << "ref_gear or ref_arc_steer error";
    return false;
  }

  ILOG_INFO << "input gear =" << static_cast<int>(current_gear);

  // ILOG_INFO <<"try adjust plan to target point");
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.reserve(5);
  // 1. try one arc
  //  Todo: target line should changed to be in range of +- 0.03cm
  bool success = OneArcPlan(tmp_path_seg_vec, current_pose, current_gear);

  if (success) {
    if (!CheckPathSegCollided(tmp_path_seg_vec.back(),
                              calc_params_.lon_buffer_rev_trials)) {
      ILOG_INFO << "only one arc success!";
      AddPathSegVecToOutput(tmp_path_seg_vec);
      return true;
    }
  }
  success = false;
  ILOG_INFO << "One arc failed!";

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
      ILOG_INFO << "align body failed!";
      return false;
    }
  }

  ILOG_INFO << "body already align";
  if (IsOnTargetLine(current_pose)) {
    pnc::geometry_lib::LineSegment last_line;
    last_line.pA = current_pose.pos;
    last_line.heading = current_pose.heading;

    // if (CheckLonToTarget(current_pose)) {
    //   return true;
    // }

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
    ILOG_INFO << "-------- No." << i << " in paralle adjust-plan--------";

    bool s_turn_success = false;
    std::vector<pnc::geometry_lib::PathSegment> s_turn_vec;
    double ratio = 1.0;
    for (; ratio > 0.1; ratio -= 0.1) {
      s_turn_vec.clear();
      s_turn_vec.reserve(2);
      if (STurnParallelPlan(s_turn_vec, current_pose, calc_params_.target_line,
                            current_gear, ratio,
                            apa_param.GetParam().min_turn_radius)) {
        if (!CheckPathSegVecCollided(s_turn_vec,
                                     calc_params_.lon_buffer_rev_trials)) {
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
          if (!CheckPathSegVecCollided(s_turn_vec,
                                       calc_params_.lon_buffer_rev_trials)) {
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
        ILOG_INFO << "shift plan fail with current gear";
        break;
      }
    }

    // sturn success
    if (ratio == 1.0) {
      loop_success = true;
      ILOG_INFO << "shift loop success, try last line!";
      break;
    } else if (IsOnTargetLine(s_turn_vec.back().GetEndPose())) {
      loop_success = true;
      ILOG_INFO << "close to target line, try last line!";
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
        ILOG_INFO << "calc line success";
        if (!last_line.is_ignored) {
          parallel_shift_path_vec.emplace_back(pnc::geometry_lib::PathSegment(
              pnc::geometry_lib::CalLineSegGear(last_line), last_line));
          ILOG_INFO << "last line exist";
        }
      } else {
        loop_success = false;
        ILOG_INFO << "OneLinePlan fail\n";
      }
    }
  }

  if (loop_success) {
    AddPathSegVecToOutput(parallel_shift_path_vec);
  }
  return loop_success;
}

const bool ParallelPathGenerator::CheckAdjustPlanSuitable(
    const pnc::geometry_lib::PathPoint& current_pose) const {
  return (std::fabs(current_pose.heading) <=
              apa_param.GetParam().adjust_plan_max_heading1_err * kDeg2Rad ||
          (std::fabs(current_pose.heading) <=
               apa_param.GetParam().adjust_plan_max_heading2_err * kDeg2Rad &&
           std::fabs(current_pose.pos.y()) <=
               apa_param.GetParam().adjust_plan_max_lat_err));
}

const bool ParallelPathGenerator::CalcLineDirAllValidPose(
    std::vector<pnc::geometry_lib::PathPoint>& target_tan_pose_vec,
    const pnc::geometry_lib::PathPoint start_pose,
    const pnc::geometry_lib::LineSegment& terminal_line,
    const double shift_ratio) {
  Eigen::Vector2d line_norm_vec = Eigen::Vector2d::Zero();

  // line_norm_vec starts from point to line
  if (!pnc::geometry_lib::CalLineUnitNormVecByPos(start_pose.pos, terminal_line,
                                                  line_norm_vec)) {
    ILOG_INFO << "calc norm unit failed!";
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
    // ILOG_INFO <<"total lat dist =" << total_lat_dist);
    // ILOG_INFO <<"shift lat dist =" << shift_lat_dist);
  }

  return is_success;
}

const bool ParallelPathGenerator::SearchToTargetLine(
    std::vector<std::vector<geometry_lib::PathSegment>>& path_vec,
    const pnc::geometry_lib::PathPoint& ego_pose,
    const pnc::geometry_lib::LineSegment& prepare_line, const double radius,
    const double lon_buffer) {
  using namespace pnc::geometry_lib;
  ILOG_INFO << "------------- SearchToTargetLine -----------------------";
  pnc::geometry_lib::PrintPose("ego pose", ego_pose);

  bool success = false;

  // std::vector<uint8_t> gear_vec = {SEG_GEAR_DRIVE, SEG_GEAR_REVERSE};
  // std::vector<uint8_t> steer_vec = {SEG_STEER_LEFT, SEG_STEER_RIGHT,
  //                                   SEG_STEER_STRAIGHT};

  std::vector<uint8_t> gear_vec = {SEG_GEAR_DRIVE, SEG_GEAR_REVERSE};

  std::vector<uint8_t> steer_vec;
  if (std::fabs(ego_pose.heading) * kRad2Deg < 5.0) {
    steer_vec = {SEG_STEER_STRAIGHT, SEG_STEER_STRAIGHT};
  } else {
    if (ego_pose.heading * kRad2Deg > 1e-5) {
      steer_vec = {SEG_STEER_RIGHT, SEG_STEER_LEFT};
    } else {
      steer_vec = {SEG_STEER_LEFT, SEG_STEER_RIGHT};
    }
  }

  if (steer_vec.size() == 0) {
    ILOG_INFO << "steer_vec.size() == 0";
    return false;
  }

  const Eigen::Vector2d ego_heading = GetUnitTangVecByHeading(ego_pose.heading);

  for (size_t i = 0; i < gear_vec.size(); i++) {
    const uint8_t gear = gear_vec[i];
    const uint8_t steer = steer_vec[i];

    if (steer == SEG_STEER_STRAIGHT) {
      ILOG_INFO << "-------------------- LINE LOGIC ----------------------";
      auto first_line = BuildLineSegByPose(ego_pose.pos, ego_pose.heading);

      if (!CalcLineStepLimitPose(first_line, gear, kColLargeLatBufferOutSlot)) {
        ILOG_INFO << "CalcLineStepLimitPose FAILED";
        continue;
      }

      ILOG_INFO << "first_line length limit = " << first_line.length;

      const double max_first_line_len = std::min(first_line.length, 1.5);
      ILOG_INFO << "max_first_line_len = " << max_first_line_len;

      const double gear_sgn = gear == SEG_GEAR_DRIVE ? 1.0 : -1.0;
      pnc::geometry_lib::PathPoint ego_transition_pose = ego_pose;

      const size_t step_num = 3;
      const double step = max_first_line_len / static_cast<double>(step_num);

      for (size_t i = 1; i < step_num + 1; i++) {
        ego_transition_pose.pos =
            ego_pose.pos + i * step * gear_sgn * ego_heading;

        // PrintPose("ego_transition_pose", ego_transition_pose);

        std::vector<std::vector<PathSegment>> tmp_path_vec;
        if (!TwoArcPath(tmp_path_vec, ego_transition_pose, prepare_line,
                        SEG_GEAR_INVALID, radius, kColBufferOutSlot)) {
          continue;
        }
        // ILOG_INFO << "TwoArcPath tmp_path_vec size = " <<
        // tmp_path_vec.size();
        success = true;
        first_line.SetPoints(ego_pose.pos, ego_transition_pose.pos);
        const PathSegment first_line_path_seg(gear, first_line);

        for (auto& path : tmp_path_vec) {
          std::vector<PathSegment> tmp_path = {first_line_path_seg};
          tmp_path.insert(tmp_path.end(), path.begin(), path.end());
          path_vec.emplace_back(tmp_path);
        }
      }

    } else {
      // ILOG_INFO << " --------------- ARC LOGIC ----------------------";
      PrintGear("gear = ", gear);
      std::vector<PathSegment> aligned_path_seg_vec;
      if (!AlignBodyPlan(aligned_path_seg_vec, ego_pose, prepare_line.heading,
                         gear)) {
        ILOG_INFO << "AlignBodyPlan failed!";
        continue;
      }

      if (aligned_path_seg_vec.size() == 0) {
        ILOG_INFO << "aligned_path_seg_vec size == 0!";
        continue;
      }

      auto& aligned_seg = aligned_path_seg_vec.back();
      if (TrimPathByCollisionDetection(aligned_seg, kColBufferOutSlot) ==
          PATH_COL_INVALID) {
        // ILOG_INFO << "TrimPathByCollisionDetection failed!";
        continue;
      }

      if (aligned_seg.Getlength() < 0.15) {
        // ILOG_INFO << "max avaliable path length < 0.15m";
        continue;
      }

      const double max_search_lenth =
          std::min(kMaxSearchArcLength, aligned_seg.Getlength());

      const int max_step_num =
          std::max(static_cast<int>(max_search_lenth / kMinSearchArcStep), 1);

      const int step_num = std::min(kMaxSearchArcStepNum, max_step_num);
      const double step = max_search_lenth / step_num;

      const uint8_t ref_gear = geometry_lib::SEG_GEAR_INVALID;
      const double ref_radius = apa_param.GetParam().min_turn_radius + 0.3;

      for (size_t i = 1; i < step_num + 1; i++) {
        Arc tmp_arc = aligned_seg.GetArcSeg();

        if (!CompleteArcInfo(tmp_arc, i * step, tmp_arc.is_anti_clockwise)) {
          continue;
        }

        const pnc::geometry_lib::PathPoint ego_transition_pose(
            tmp_arc.pB, tmp_arc.headingB);

        std::vector<std::vector<PathSegment>> tmp_path_vec;
        if (!TwoArcPath(tmp_path_vec, ego_transition_pose, prepare_line,
                        ref_gear, ref_radius, kColBufferOutSlot)) {
          continue;
        }

        const PathSegment aligned_path_seg(CalArcSteer(tmp_arc),
                                           CalArcGear(tmp_arc), tmp_arc);

        for (const auto& path_seg : tmp_path_vec) {
          if (path_seg.front().seg_steer == aligned_path_seg.seg_steer) {
            continue;
          }

          success = true;
          std::vector<PathSegment> path = {aligned_path_seg};
          path.insert(path.end(), path_seg.begin(), path_seg.end());
          path_vec.emplace_back(path);
        }

        // ILOG_INFO <<"two arc path = " << path_vec.size());
      }
    }
  }
  // ILOG_INFO << "path_vec size = " << path_vec.size();

  return success;
}

const bool ParallelPathGenerator::CalcLineDirAllValidPose(
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
    ILOG_INFO << "calc forward limit pose error";
    return false;
  }

  pnc::geometry_lib::LineSegment backward_line_seg;
  backward_line_seg.pA = forward_line_seg.pB;
  backward_line_seg.heading = forward_line_seg.heading;

  if (!CalcLineStepLimitPose(backward_line_seg,
                             pnc::geometry_lib::SEG_GEAR_REVERSE)) {
    ILOG_INFO << "calc backward limit pose error";
    return false;
  }

  if ((backward_line_seg.pA - backward_line_seg.pB).norm() < 0.3) {
    ILOG_INFO << "two limit pose too close along the line!";
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

const bool ParallelPathGenerator::GenerateCandidatePath() {
  return StartNodeGenerator();
}

const bool ParallelPathGenerator::StartNodeGenerator() {
  const int depth_max = 4;
  const float search_dist_max = 2.0f;
  const float srarch_angle_max = M_PI / 3.0;

  ILOG_INFO << "calc_params_.valid_target_pt_vec size = "
            << calc_params_.valid_target_pt_vec.size();

  int start_nodes_size = 0;
  pnc::geometry_lib::PathSegSteer current_steer =
      pnc::geometry_lib::SEG_STEER_STRAIGHT;

  auto& node_space = calc_params_.ego_start_pose_space;

  node_space.clear();
  node_space.reserve(500);
  input_.ego_info_under_slot.cur_pose.id = 0;
  input_.ego_info_under_slot.cur_pose.depth = 0;
  input_.ego_info_under_slot.cur_pose.parent_id = -1;
  node_space.emplace_back(input_.ego_info_under_slot.cur_pose);

  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.3, false));

  const bool is_drive_line_success =
      SearchLineNode(node_space, input_.ego_info_under_slot.cur_pose,
                     search_dist_max, pnc::geometry_lib::SEG_GEAR_DRIVE);

  const bool is_reverse_line_success =
      SearchLineNode(node_space, input_.ego_info_under_slot.cur_pose,
                     search_dist_max, pnc::geometry_lib::SEG_GEAR_REVERSE);
  node_space.front().depth = 1;

  size_t current_idx = 0;
  const auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<std::vector<pnc::geometry_lib::PathSegment>> path_vec;
  while (current_idx < node_space.size()) {
    const auto time1 = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                              time1 - start_time)
                              .count();
    if (duration > 200.0) {
      break;
    }

    if (node_space.size() > 500) {
      ILOG_INFO << "start_nodes_size override! " << node_space.size();
      break;
    }

    const auto& parent_pose = node_space[current_idx];
    current_idx++;

    if (parent_pose.depth >= depth_max) {
      ILOG_INFO << "current_depth override! " << parent_pose.depth
                << " >= " << depth_max;
      break;
    }

    if (pnc::geometry_lib::NormalizeAngle(parent_pose.heading) > M_PI_2 ||
        pnc::geometry_lib::NormalizeAngle(parent_pose.heading) < -M_PI_2) {
      continue;
    }

    ILOG_INFO << " ";
    ILOG_INFO << "currently in depth = " << parent_pose.depth;
    pnc::geometry_lib::PrintPose("parent_pose", parent_pose);

    bool success = SearchToTargetLineVecV2(
        path_vec, parent_pose, apa_param.GetParam().min_turn_radius + 0.3,
        kColBufferOutSlot);
    if (!success) {
      success = SearchToRootPoseVec(path_vec, parent_pose,
                                    apa_param.GetParam().min_turn_radius + 0.3,
                                    kColBufferOutSlot);
    }

    if (path_vec.size() > kMaxPathSize) {
      ILOG_INFO << "path_vec big enough!" << path_vec.size();
      break;
    }

    if (parent_pose.depth < 2) {
      ILOG_INFO << "";
      ILOG_INFO << "No.1 DRIVE LEFT ARC";
      SearchArcNode(node_space, parent_pose, srarch_angle_max,
                    pnc::geometry_lib::SEG_GEAR_DRIVE,
                    pnc::geometry_lib::SEG_STEER_LEFT);

      ILOG_INFO << "";
      ILOG_INFO << "No.2 DRIVE RIGHT ARC";
      SearchArcNode(node_space, parent_pose, srarch_angle_max,
                    pnc::geometry_lib::SEG_GEAR_DRIVE,
                    pnc::geometry_lib::SEG_STEER_RIGHT);

      ILOG_INFO << "";
      ILOG_INFO << "No.3 REVERSE LEFT ARC";
      SearchArcNode(node_space, parent_pose, srarch_angle_max,
                    pnc::geometry_lib::SEG_GEAR_REVERSE,
                    pnc::geometry_lib::SEG_STEER_LEFT);
      ILOG_INFO << "";
      ILOG_INFO << "No.4 REVERSE RIGHT ARC";
      SearchArcNode(node_space, parent_pose, srarch_angle_max,
                    pnc::geometry_lib::SEG_GEAR_REVERSE,
                    pnc::geometry_lib::SEG_STEER_RIGHT);
      ILOG_INFO << "";
    } else if (std::fabs(parent_pose.kappa) > 1e-5) {
      const auto ref_steer = parent_pose.kappa > 0.0
                                 ? pnc::geometry_lib::SEG_STEER_RIGHT
                                 : pnc::geometry_lib::SEG_STEER_LEFT;
      const auto ref_gear = parent_pose.is_drive
                                ? pnc::geometry_lib::SEG_GEAR_REVERSE
                                : pnc::geometry_lib::SEG_GEAR_DRIVE;
      SearchArcNode(node_space, parent_pose, srarch_angle_max, ref_gear,
                    ref_steer);
      SearchArcNode(node_space, parent_pose, srarch_angle_max, ref_gear,
                    ref_steer);
    }
  }

  ILOG_INFO << "node_space size = " << node_space.size();
  debug_info_.point_vec = node_space;
  if (node_space.empty()) {
    return false;
  }

  ILOG_INFO << "current_idx = " << current_idx;

  if (path_vec.size() < kMaxPathSize) {
    for (size_t i = current_idx + 1; i < node_space.size(); i++) {
      bool success = SearchToTargetLineVecV2(
          path_vec, node_space[i], apa_param.GetParam().min_turn_radius + 0.3,
          kColBufferOutSlot);

      if (!success) {
        success = SearchToRootPoseVec(
            path_vec, node_space[i], apa_param.GetParam().min_turn_radius + 0.3,
            kColBufferOutSlot);
      }

      const auto time1 = std::chrono::high_resolution_clock::now();
      const auto duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(time1 -
                                                                start_time)
              .count();
      if (duration > 200.0) {
        break;
      }
      if (path_vec.size() > kMaxPathSize) {
        break;
      }
    }
  }
  ILOG_INFO << "FINAL  path_vec size = " << path_vec.size();

  if (path_vec.size() == 0) {
    ILOG_INFO << "FINAL  path_vec size = " << path_vec.size();
    return false;
  }

  ILOG_INFO
      << "\n\n\n----------------------------------------------------------";
  // for (size_t mm = 0; mm < path_vec.size(); mm++) {
  //   ILOG_INFO << "\n";
  //   ILOG_INFO "no. " << mm << " path ---------------";
  //   pnc::geometry_lib::PrintSegmentsVecInfo(path_vec[mm]);
  // }

  std::vector<GeometryPath> geo_path_vec;
  if (!AssempleGeometryPathVec(geo_path_vec, path_vec)) {
    return false;
  }

  size_t best_path_idx = std::numeric_limits<size_t>::max();
  if (!SelectBestPathOutsideSlot(geo_path_vec, best_path_idx)) {
    return false;
  }

  ILOG_INFO << "best idx = " << best_path_idx;

  AddPathSegToOutPut(geo_path_vec[best_path_idx].path_segment_vec);
  debug_info_.debug_all_path_vec = geo_path_vec;

  const auto end_pose =
      geo_path_vec[best_path_idx].path_segment_vec.back().GetEndPose();

  calc_params_.park_out_path_in_slot.clear();
  for (size_t i = 0; i < calc_params_.valid_target_pt_vec.size(); i++) {
    if (CheckSamePose(end_pose, calc_params_.valid_target_pt_vec[i])) {
      calc_params_.park_out_path_in_slot =
          calc_params_.inversed_path_vec_in_slot[i].path_segment_vec;
    }
  }

  return true;
}

const bool ParallelPathGenerator::SearchLineNode(
    std::vector<pnc::geometry_lib::PathPoint>& node_space,
    const pnc::geometry_lib::PathPoint& start_pose,
    const double max_search_dist, const uint8_t ref_gear) {
  ILOG_INFO << " ";
  ILOG_INFO << " -----start line search ------";

  if (ref_gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
      ref_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) {
    ILOG_INFO << "gear error in searchLineNode!";
    return false;
  }
  pnc::geometry_lib::PrintGear("ref gear = ", ref_gear);

  double max_search_dist_mag = std::fabs(max_search_dist);

  pnc::geometry_lib::PathPoint child_pose;
  child_pose.id = -1;
  child_pose.parent_id = -1;

  const double sig =
      (ref_gear == pnc::geometry_lib::SEG_GEAR_DRIVE ? 1.0 : -1.0);
  const bool is_drive = (ref_gear == pnc::geometry_lib::SEG_GEAR_DRIVE);

  pnc::geometry_lib::LineSegment line(start_pose.pos, start_pose.heading,
                                      max_search_dist_mag, ref_gear);

  pnc::geometry_lib::PathSegment line_seg(ref_gear, line);

  const auto col_res =
      TrimPathByCollisionDetection(line_seg, kColBufferOutSlot);
  if (col_res == PATH_COL_INVALID) {
    pnc::geometry_lib::PrintGear("collided at start pose with gear == ",
                                 ref_gear);
    return false;
  }

  max_search_dist_mag = std::min(max_search_dist_mag, line_seg.Getlength());
  max_search_dist_mag = std::min(max_search_dist_mag, 2.5);
  ILOG_INFO << "max_search_dist_mag after trim = " << max_search_dist_mag;
  if (kSamplingLineStep > max_search_dist_mag) {
    ILOG_INFO << "trimed path too short!";
    return false;
  }

  const Eigen::Vector2d heading_vec(std::cos(start_pose.heading),
                                    std::sin(start_pose.heading));

  int seg_num = static_cast<int>(max_search_dist_mag / kSamplingLineStep);
  if (seg_num < 1) {
    return false;
  } else if (seg_num > 2) {
    seg_num = 2;
  }

  const double step = max_search_dist_mag / seg_num;

  child_pose.kappa = 0;
  child_pose.is_drive = is_drive;
  child_pose.depth = start_pose.depth + 1;
  child_pose.parent_id = start_pose.id;
  child_pose.heading = start_pose.heading;
  child_pose.heading_vec = start_pose.heading_vec;

  for (int i = 1; i <= seg_num; i++) {
    const double offset = step * i;
    child_pose.s = offset;
    child_pose.pos = start_pose.pos + heading_vec * offset * sig;
    child_pose.id = node_space.size();
    node_space.emplace_back(child_pose);
  }
  ILOG_INFO << "after line search, node size = " << node_space.size();

  return true;
}

const bool ParallelPathGenerator::SearchArcNode(
    std::vector<pnc::geometry_lib::PathPoint>& node_space,
    const pnc::geometry_lib::PathPoint& start_pose,
    const double max_search_angle, const uint8_t ref_gear,
    const uint8_t ref_steer, const double ref_radius) {
  ILOG_INFO << " ";
  // ILOG_INFO << "----- SearchArcNode ------";

  pnc::geometry_lib::Arc arc;
  arc.pA = start_pose.pos;
  arc.headingA = start_pose.heading;
  arc.circle_info.radius = ref_radius;
  if (!pnc::geometry_lib::CalcArcDirection(arc.is_anti_clockwise, ref_gear,
                                           ref_steer)) {
    return false;
  }

  // ILOG_INFO << "parent pose = " << arc.pA.transpose()
  //           << ", heading(deg) = " << arc.headingA * kRad2Deg;

  pnc::geometry_lib::PrintGear("current search gear = ", ref_gear);
  pnc::geometry_lib::PrintSteer("current search steer = ", ref_steer);

  // ILOG_INFO << "arc.is_anti_clockwise = " << arc.is_anti_clockwise;

  double max_search_angle_rad_mag = std::fabs(max_search_angle);
  if (!CompleteArcInfo(arc, max_search_angle_rad_mag * ref_radius, ref_steer)) {
    return false;
  }

  // ILOG_INFO << "arc pb = " << arc.pB.transpose()
  //           << ", heading(deg) = " << arc.headingB * kRad2Deg;
  // ILOG_INFO << "arc length =" << arc.length;

  pnc::geometry_lib::PathSegment arc_seg(ref_steer, ref_gear, arc);
  const auto col_res = TrimPathByCollisionDetection(arc_seg, kColBufferOutSlot);
  if (col_res == PATH_COL_INVALID) {
    ILOG_INFO << "colided at start pose!";
    return false;
  }

  // ILOG_INFO << "max available length = " << arc_seg.Getlength();

  max_search_angle_rad_mag =
      std::min(max_search_angle_rad_mag,
               arc_seg.Getlength() / arc_seg.GetArcSeg().circle_info.radius);

  ILOG_INFO << "max available angle = " << max_search_angle_rad_mag * kRad2Deg;

  if (max_search_angle_rad_mag < kSamplingArcAngleStep) {
    // ILOG_INFO << "max available angle too small";
    return false;
  }

  int seg_num =
      static_cast<int>(max_search_angle_rad_mag / kSamplingArcAngleStep);
  if (seg_num < 1) {
    return false;
  } else if (seg_num > 2) {
    seg_num = 2;
  }

  double d_angle = max_search_angle_rad_mag / seg_num;
  const double ds = d_angle * ref_radius;
  const double steer_sgn =
      (ref_steer == pnc::geometry_lib::SEG_STEER_LEFT ? 1.0 : -1.0);
  const double kappa = steer_sgn / ref_radius;

  pnc::geometry_lib::PathPoint child_pose;
  child_pose.kappa = kappa;
  child_pose.parent_id = start_pose.id;
  child_pose.pos = start_pose.pos;
  child_pose.heading = start_pose.heading;
  child_pose.depth = start_pose.depth + 1;

  const double heading_sgn = arc.is_anti_clockwise ? 1.0 : -1.0;
  const bool is_drive = ref_gear == pnc::geometry_lib::SEG_GEAR_DRIVE;

  Eigen::Vector2d v_n = arc.pA - arc.circle_info.center;
  const auto rot_m =
      pnc::geometry_lib::GetRotm2dFromTheta(heading_sgn * d_angle);

  for (int i = 1; i <= seg_num; i++) {
    const double angle_offset = i * d_angle;

    v_n = rot_m * v_n;
    child_pose.pos = arc.circle_info.center + v_n;

    child_pose.s = angle_offset * ref_radius;
    child_pose.is_drive = is_drive;

    child_pose.heading = pnc::geometry_lib::NormalizeAngle(
        child_pose.heading + d_angle * heading_sgn);

    child_pose.heading_vec << std::cos(child_pose.heading),
        std::sin(child_pose.heading);

    child_pose.id = node_space.size();
    node_space.emplace_back(child_pose);
  }

  // ILOG_INFO << "after arc search, node size = " << node_space.size();

  return true;
}

const bool ParallelPathGenerator::SearchToRootPoseVec(
    std::vector<std::vector<pnc::geometry_lib::PathSegment>>& path_vec,
    const pnc::geometry_lib::PathPoint& start_pose, const double radius,
    const double lon_buffer) {
  bool success = false;
  const auto ref_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
  const auto ref_steer = pnc::geometry_lib::SEG_STEER_INVALID;

  for (const auto& target_pose : calc_params_.valid_target_pt_vec) {
    const auto target_line =
        geometry_lib::BuildLineSegByPose(target_pose.pos, target_pose.heading);

    std::vector<std::vector<pnc::geometry_lib::PathSegment>> tmp_path_vec;

    success = LineArcPlan(tmp_path_vec, start_pose, target_line, ref_gear,
                          ref_steer, radius, lon_buffer);
    if (!success) {
      // ILOG_INFO << "LineArcPlan failed, try TwoArcPath!";
      tmp_path_vec.clear();
      success = TwoArcPath(tmp_path_vec, start_pose, target_line, ref_gear,
                           radius, lon_buffer, ref_steer);
      if (success) {
        // ILOG_INFO << "TwoArcPath success!";
      } else {
        // ILOG_INFO << "TwoArcPath failed!";
      }
    } else {
      // ILOG_INFO << "LineArcPlan success!";
    }

    if (!success) {
      // ILOG_INFO << "LineArcPlan && TwoArcPath both failed!";
      tmp_path_vec.clear();
      success = DubinsPlan(tmp_path_vec, start_pose, target_pose, radius,
                           lon_buffer, ref_steer, true);
      if (success) {
        // ILOG_INFO << "DubinsPlan success!";
      } else {
        // ILOG_INFO << "DubinsPlan failed!";
      }
    }

    if (success) {
      path_vec.insert(path_vec.end(), tmp_path_vec.begin(), tmp_path_vec.end());
      break;
    }
  }

  return success;
}

const bool ParallelPathGenerator::SearchToTargetLineVecV2(
    std::vector<std::vector<pnc::geometry_lib::PathSegment>>& path_vec,
    const pnc::geometry_lib::PathPoint& start_pose, const double radius,
    const double lon_buffer) {
  for (const auto& inv_park_out_path : calc_params_.suc_rev_parking_out_vec) {
    ILOG_INFO << "\n";
    const auto& preparation_pose = inv_park_out_path.front().GetStartPose();
    pnc::geometry_lib::PrintPose("preparation pose", preparation_pose);

    const auto preparation_line = pnc::geometry_lib::BuildLineSegByPose(
        preparation_pose.pos, preparation_pose.heading);

    std::vector<std::vector<pnc::geometry_lib::PathSegment>> tmp_path_vec;
    if (!SearchToTargetLineV2(tmp_path_vec, start_pose, preparation_line,
                              radius, lon_buffer)) {
      continue;
    }

    for (auto tmp_path_seg : tmp_path_vec) {
      for (const auto& inv_path_seg : inv_park_out_path) {
        tmp_path_seg.emplace_back(inv_path_seg);
      }

      path_vec.emplace_back(tmp_path_seg);
    }
  }
  return true;
}

const bool ParallelPathGenerator::SearchToTargetLineV2(
    std::vector<std::vector<pnc::geometry_lib::PathSegment>>& path_vec,
    const pnc::geometry_lib::PathPoint& ego_pose,
    const pnc::geometry_lib::LineSegment& prepare_line, const double radius,
    const double lon_buffer) {
  // ILOG_INFO << "";
  // ILOG_INFO << "----------- SearchToTargetLineV2 ---------------------";
  // pnc::geometry_lib::PrintPose("ego transit pose", ego_pose);
  // ILOG_INFO << "preparation line  pA = " << prepare_line.pA.transpose()
  //           << ", heading(deg) = " << prepare_line.heading * kRad2Deg;

  bool success = false;
  const pnc::geometry_lib::PathPoint preparation_pose(prepare_line.pA,
                                                      prepare_line.heading);
  // Line
  success = false;
  std::vector<std::vector<pnc::geometry_lib::PathSegment>> tmp_path_vec;
  if (std::fabs(ego_pose.kappa) <= 1e-5) {
    ILOG_INFO << "ego last step is line!";
    tmp_path_vec.clear();
    success =
        TwoArcPath(tmp_path_vec, ego_pose, prepare_line,
                   pnc::geometry_lib::SEG_GEAR_INVALID, radius, lon_buffer);

    if (!success) {
      // ILOG_INFO << "TwoArcPath failed, try dubins!";
      tmp_path_vec.clear();
      success =
          DubinsPlan(tmp_path_vec, ego_pose, preparation_pose, radius,
                     lon_buffer, pnc::geometry_lib::SEG_STEER_INVALID, true);
      if (success) {
        // ILOG_INFO << "DubinsPlan success!";
      } else {
        // ILOG_INFO << "DubinsPlan failed!";
      }
    } else {
      // ILOG_INFO << "TwoArcPath success!";
    }
  } else {
    ILOG_INFO << "ego last step is arc!";

    const auto ref_steer = ego_pose.kappa > 0.0
                               ? pnc::geometry_lib::SEG_STEER_RIGHT
                               : pnc::geometry_lib::SEG_STEER_LEFT;
    const auto ref_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    pnc::geometry_lib::PrintSteer("this time steer", ref_steer);
    pnc::geometry_lib::PrintGear("this time gear", ref_gear);

    success = LineArcPlan(tmp_path_vec, ego_pose, prepare_line, ref_gear,
                          ref_steer, radius, lon_buffer);
    if (!success) {
      // ILOG_INFO << "LineArcPlan failed, try TwoArcPath!";
      tmp_path_vec.clear();
      success = TwoArcPath(tmp_path_vec, ego_pose, prepare_line, ref_gear,
                           radius, lon_buffer, ref_steer);
      if (success) {
        // ILOG_INFO << "TwoArcPath success!";
      } else {
        // ILOG_INFO << "TwoArcPath failed!";
      }
    } else {
      // ILOG_INFO << "LineArcPlan success!";
    }

    if (!success) {
      // ILOG_INFO << "LineArcPlan && TwoArcPath both failed!";
      tmp_path_vec.clear();
      success = DubinsPlan(tmp_path_vec, ego_pose, preparation_pose, radius,
                           lon_buffer, ref_steer, true);
      if (success) {
        // ILOG_INFO << "DubinsPlan success!";
      } else {
        // ILOG_INFO << "DubinsPlan failed!";
      }
    }
  }
  // ILOG_INFO "\n";
  // ILOG_INFO << "transit pose to preparation pose path size = "
  //           << tmp_path_vec.size();

  if (success) {
    ILOG_INFO << "";
    ILOG_INFO << "backtracking!";

    auto& node_space = calc_params_.ego_start_pose_space;

    int parent_id = ego_pose.id;
    std::vector<int> reversed_id_vec;
    while (parent_id != -1) {
      ILOG_INFO << " id " << parent_id;
      pnc::geometry_lib::PrintPose("parent pose", node_space[parent_id]);
      reversed_id_vec.emplace_back(parent_id);

      parent_id = node_space[parent_id].parent_id;
    }
    ILOG_INFO << "point_vec size = " << reversed_id_vec.size();

    std::vector<pnc::geometry_lib::PathSegment> search_start_path_vec;
    std::reverse(reversed_id_vec.begin(), reversed_id_vec.end());

    ILOG_INFO << "\nid from path size";
    for (const auto& id : reversed_id_vec) {
      ILOG_INFO << "id = " << id;
    }

    for (size_t i = 1; i < reversed_id_vec.size(); i++) {
      const auto& pA = node_space[reversed_id_vec[i - 1]];
      const auto& pB = node_space[reversed_id_vec[i]];

      if (std::fabs(pB.kappa) < 1e-5) {
        ILOG_INFO << "\nline!";

        pnc::geometry_lib::LineSegment line(pA.pos, pB.pos, pB.heading);
        const auto gear = pB.is_drive ? pnc::geometry_lib::SEG_GEAR_DRIVE
                                      : pnc::geometry_lib::SEG_GEAR_REVERSE;
        search_start_path_vec.emplace_back(
            pnc::geometry_lib::PathSegment(gear, line));

        ILOG_INFO << "pa = " << line.pA.transpose()
                  << ", pB = " << line.pB.transpose()
                  << "heading (deg)= " << pB.heading * kRad2Deg
                  << "gear = " << static_cast<int>(gear);
      } else {
        ILOG_INFO << "\narc!";

        const uint8_t gear = pB.is_drive ? pnc::geometry_lib::SEG_GEAR_DRIVE
                                         : pnc::geometry_lib::SEG_GEAR_REVERSE;
        const uint8_t steer = pB.kappa > 0.0
                                  ? pnc::geometry_lib::SEG_STEER_LEFT
                                  : pnc::geometry_lib::SEG_STEER_RIGHT;
        pnc::geometry_lib::PrintGear("gear", gear);
        pnc::geometry_lib::PrintSteer("steer", steer);

        pnc::geometry_lib::Arc arc;
        arc.pA = pA.pos;
        arc.headingA = pA.heading;
        arc.pB = pB.pos;
        arc.headingB = pB.heading;

        arc.circle_info.radius = 1.0 / std::fabs(pB.kappa);
        arc.circle_info.center = pnc::geometry_lib::CalArcCenter(
            arc.pB, arc.headingB, arc.circle_info.radius, steer);

        arc.length = pB.s;
        pnc::geometry_lib::CalcArcDirection(arc.is_anti_clockwise, gear, steer);
        const pnc::geometry_lib::PathSegment arc_seg(steer, gear, arc);

        search_start_path_vec.emplace_back(arc_seg);
        pnc::geometry_lib::PrintSegmentInfo(arc_seg);
      }
    }
    ILOG_INFO << "backtracking search_start_path_vec size = "
              << search_start_path_vec.size();

    ILOG_INFO << "\n";
    ILOG_INFO << "\n";
    ILOG_INFO << "\n";
    ILOG_INFO << "successed transit pose to preparation line path size = "
              << tmp_path_vec.size();

    for (size_t k = 0; k < tmp_path_vec.size(); k++) {
      pnc::geometry_lib::PrintSegmentsVecInfo(tmp_path_vec[k]);
    }

    for (const auto path : tmp_path_vec) {
      auto total_path = search_start_path_vec;
      total_path.insert(total_path.end(), path.begin(), path.end());
      path_vec.emplace_back(total_path);
    }
  }

  return success;
}

const bool ParallelPathGenerator::OneArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    pnc::geometry_lib::LineSegment& target_line, const uint8_t current_gear,
    const uint8_t current_arc_steer) {
  if (!CalOneArcWithLine(arc, target_line,
                         apa_param.GetParam().parallel_multi_plan_radius_eps)) {
    ILOG_INFO << "OneArcPlan fail 0";
    return false;
  }

  uint8_t steer = pnc::geometry_lib::CalArcSteer(arc);
  uint8_t gear = pnc::geometry_lib::CalArcGear(arc);
  if (steer != current_arc_steer || gear != current_gear) {
    ILOG_INFO << "OneArcPlan fail 1";
    return false;
  }
  pnc::geometry_lib::PathSegment arc_seg(steer, gear, arc);
  path_seg_vec.emplace_back(arc_seg);
  return true;
}

const bool ParallelPathGenerator::OneArcPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  // ILOG_INFO <<"try one arc plan");

  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;

  if (pnc::mathlib::IsDoubleEqual(arc.headingA,
                                  calc_params_.target_line.heading) ||
      pnc::mathlib::IsDoubleEqual(arc.pA.y(),
                                  calc_params_.target_line.pA.y())) {
    // ILOG_INFO <<
    //     "current heading is equal to target heading or current y is equal
    //     to " "target y, no need to one arc plan");
    return false;
  }

  bool success = pnc::geometry_lib::CalOneArcWithLineAndGear(
      arc, calc_params_.target_line, current_gear);

  if (success) {
    success = pnc::mathlib::IsInBound(
        arc.circle_info.radius, apa_param.GetParam().min_turn_radius - 1e-3,
        apa_param.GetParam().min_turn_radius + 5.0);

    if (success) {
      ILOG_INFO << "one arc plan success";
      const auto steer = pnc::geometry_lib::CalArcSteer(arc);
      const auto gear = pnc::geometry_lib::CalArcGear(arc);
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }

  return success;
}

const bool ParallelPathGenerator::OneArcPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    pnc::geometry_lib::LineSegment& target_line, const uint8_t current_gear) {
  // ILOG_INFO <<"try one arc plan");

  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;

  if (pnc::mathlib::IsDoubleEqual(arc.headingA, target_line.heading) ||
      pnc::mathlib::IsDoubleEqual(arc.pA.y(), target_line.pA.y())) {
    ILOG_INFO << "small heading or y coord diff, quit one arc plan";
    return false;
  }

  bool success = pnc::geometry_lib::CalOneArcWithLineAndGear(arc, target_line,
                                                             current_gear);
  if (success) {
    // check radius and gear can or not meet needs
    // ILOG_INFO <<"cal radius = " << arc.circle_info.radius);
    const auto steer = pnc::geometry_lib::CalArcSteer(arc);
    const auto gear = pnc::geometry_lib::CalArcGear(arc);
    const auto arc_radius = arc.circle_info.radius;

    success =
        (arc_radius >= apa_param.GetParam().min_turn_radius - 1e-3 &&
         arc_radius <= apa_param.GetParam().max_one_step_arc_radius + 1e-3) &&
        (gear == current_gear);

    if (success) {
      ILOG_INFO << "one arc plan success!";
      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer, gear, arc));
    }
  }
  return success;
}

const bool ParallelPathGenerator::TwoArcPlan(
    pnc::geometry_lib::Arc& arc,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    pnc::geometry_lib::LineSegment& target_line, const uint8_t current_gear,
    const uint8_t current_arc_steer) {
  pnc::geometry_lib::Arc arc2;
  if (!CalTwoArcWithLine(arc, arc2, calc_params_.target_line)) {
    ILOG_INFO << "TwoArcPlan fail 0\n";
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
    ILOG_INFO << "TwoArcPlan arc1 fail\n";
    return false;
  }
  if (steer_2 != next_arc_steer || gear_2 != next_gear) {
    ILOG_INFO << "TwoArcPlan arc2 fail\n";
    return false;
  }

  pnc::geometry_lib::PathSegment arc_seg1(steer_1, gear_1, arc);
  path_seg_vec.emplace_back(arc_seg1);
  pnc::geometry_lib::PathSegment arc_seg2(steer_2, gear_2, arc2);
  path_seg_vec.emplace_back(arc_seg2);

  return true;
}

const bool ParallelPathGenerator::TwoArcPath(
    std::vector<std::vector<pnc::geometry_lib::PathSegment>>& path_vec,
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::LineSegment& target_line, const uint8_t ref_gear,
    const double radius, const double lon_buffer, const uint8_t ref_steer) {
  // if ref_gear == invalid or cound  that mins ref_gear is not limited!

  using namespace pnc::geometry_lib;
  std::vector<std::pair<Arc, Arc>> arc_pair_vec;
  auto tmp_target_line = target_line;
  if (!CalTwoArcWithLine(start_pose, tmp_target_line, radius, radius,
                         arc_pair_vec)) {
    ILOG_INFO << "CalTwoArcWithLine failed!";
    return false;
  }

  bool success = false;
  for (const auto& arc_pair : arc_pair_vec) {
    std::vector<PathSegment> path_seg_vec;
    path_seg_vec.reserve(3);

    const Arc arc1 = arc_pair.first;
    const Arc arc2 = arc_pair.second;

    if (arc1.pB.x() < kMinXInTBoundary ||
        arc1.pB.x() < start_pose.pos.x() - 4.0 ||
        arc2.pB.x() < kMinXInTBoundary ||
        arc2.pB.x() < start_pose.pos.x() - 4.0) {
      continue;
    }

    if (!pnc::mathlib::IsDoubleEqual(arc2.headingB * kRad2Deg,
                                     tmp_target_line.heading * kRad2Deg)) {
      continue;
    }

    const PathSegment path_seg_1(CalArcSteer(arc1), CalArcGear(arc1), arc1);

    if (IsValidArcSteer(ref_steer) && path_seg_1.seg_steer != ref_steer) {
      continue;
    }

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

  // ILOG_INFO << "path_vec size in two arc  = " << path_vec.size();
  return success;
}

const bool ParallelPathGenerator::LineArcPlan(
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
    ILOG_INFO << "LineArcPlan fail 0\n";
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

    if (!pnc::mathlib::IsDoubleEqual(tmp_arc.headingB * kRad2Deg,
                                     line_seg2.heading * kRad2Deg)) {
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
      // ILOG_INFO <<"line pA =" << line.pA.transpose());
      // ILOG_INFO <<"line pB =" << line.pB.transpose());
      // ILOG_INFO << "line seg gear is error, line seg gear = "
      //           <<
      //           static_cast<int>(pnc::geometry_lib::CalLineSegGear(line))
      //           ;
      continue;
    }
    pnc::geometry_lib::PathSegment line_seg(current_gear, line);
    path_seg_vec.emplace_back(line_seg);

    pnc::geometry_lib::PathSegment arc_seg(tmp_arc_steer, tmp_gear, tmp_arc);
    path_seg_vec.emplace_back(arc_seg);

    return true;
  }
  ILOG_INFO << "LineArcPlan fail 1\n";
  return false;
}

const bool ParallelPathGenerator::LineArcPlan(
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
    ILOG_INFO << "LineArcPlan fail 0\n";
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

    if (tmp_arc.pA.x() < kMinXInTBoundary ||
        tmp_arc.pB.x() < kMinXInTBoundary ||
        tmp_arc.pA.x() < ego_pose.pos.x() - 2.5 ||
        tmp_arc.pB.x() < ego_pose.pos.x() - 2.5) {
      continue;
    }

    if (!CompleteArcInfo(tmp_arc)) {
      continue;
    }

    if (!pnc::mathlib::IsDoubleEqual(tmp_arc.headingB * kRad2Deg,
                                     last_line.heading * kRad2Deg)) {
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

const bool ParallelPathGenerator::AlignBodyPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear) {
  return AlignBodyPlan(path_seg_vec, current_pose,
                       calc_params_.target_line.heading, current_gear);
}

const bool ParallelPathGenerator::AlignBodyPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const double target_heading, const uint8_t current_gear,
    const double length_limit) {
  // ILOG_INFO <<"try align body plan");

  pnc::geometry_lib::Arc arc;
  arc.pA = current_pose.pos;
  arc.headingA = current_pose.heading;
  arc.circle_info.radius = apa_param.GetParam().min_turn_radius;

  // check if it is necessary to align body
  if (pnc::mathlib::IsDoubleEqual(arc.headingA, target_heading)) {
    ILOG_INFO << "body already align";
    return false;
  }

  if (!pnc::geometry_lib::CalOneArcWithTargetHeading(arc, current_gear,
                                                     target_heading)) {
    ILOG_INFO << "CalOneArcWithTargetHeading failed!";
    return false;
  }

  // check if gear satisfies needs
  const auto steer = pnc::geometry_lib::CalArcSteer(arc);
  const auto gear = pnc::geometry_lib::CalArcGear(arc);
  if (gear != current_gear) {
    ILOG_INFO << "gears are different!";
    return false;
  }

  if (arc.length > length_limit) {
    pnc::geometry_lib::CompleteArcInfo(arc, length_limit,
                                       arc.is_anti_clockwise);
  }

  pnc::geometry_lib::PathSegment aligned_arc_seg(steer, gear, arc);
  path_seg_vec.emplace_back(aligned_arc_seg);
  return true;
}

const bool ParallelPathGenerator::STurnParallelPlan(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const pnc::geometry_lib::LineSegment& target_line,
    const uint8_t current_gear, const double steer_change_ratio1,
    const double radius) {
  // steer_change_ratio = 0.0 -> target_line == current_line
  // steer_change_ratio = 1.0 -> target_line == calc_params_.target_line
  // ILOG_INFO <<"try s turn parallel plan");

  pnc::geometry_lib::Arc arc_s_1;
  arc_s_1.pA = current_pose.pos;
  arc_s_1.headingA = current_pose.heading;

  // check if it is possible to take S turn to target line
  if (!pnc::mathlib::IsDoubleEqual(arc_s_1.headingA, target_line.heading)) {
    // ILOG_INFO <<"body no align");
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
      ILOG_INFO << "s turn parallel plan success!";

      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer_1, gear_1, arc_s_1));

      path_seg_vec.emplace_back(
          pnc::geometry_lib::PathSegment(steer_2, gear_2, arc_s_2));
    }
  }

  if (!success) {
    ILOG_INFO << "s turn parallel plan fail!";
  }
  return success;
}

const bool ParallelPathGenerator::CalSinglePathInAdjust(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const double steer_change_ratio,
    const double radius) {
  ILOG_INFO << "-----CalSinglePathInAdjust-----";
  // ILOG_INFO <<"current_gear = "
  //             << static_cast<int>(current_gear)
  //             << ",  current_pos = " << current_pose.pos.transpose()
  //             << ",  current_heading = " << current_pose.heading *
  //             kRad2Deg);

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(5);
  // first try one arc to target line
  bool success = OneArcPlan(tmp_path_seg_vec, current_pose, current_gear);
  if (success) {
    ILOG_INFO << "one arc plan success!";
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
    // ILOG_INFO <<"last path pose to one plan");
  } else {
    last_pose = current_pose;
    // ILOG_INFO <<"current pose to one plan");
  }
  if ((last_pose.pos - input_.tlane.pt_terminal_pos).norm() <=
          apa_param.GetParam().static_pos_eps &&
      std::fabs(last_pose.heading - calc_params_.target_line.heading) <=
          apa_param.GetParam().static_heading_eps * kDeg2Rad) {
    ILOG_INFO << "already plan to target pos, no need to one line plan!";
  } else {
    // try line
    pnc::geometry_lib::LineSegment last_line;
    last_line.pA = last_pose.pos;
    last_line.heading = last_pose.heading;
    if (OneLinePlan(last_line, tmp_path_seg_vec, current_gear)) {
      ILOG_INFO << "OneLinePlan success!";
    } else {
      ILOG_INFO << "OneLinePlan fail!";
    }
  }

  // ILOG_INFO << "tmp_path_seg_vec:" ;
  // for (const auto& tmp_path_seg : tmp_path_seg_vec) {
  //   pnc::geometry_lib::PrintSegmentInfo(tmp_path_seg);
  // }

  // collision detection
  for (auto& tmp_path_seg : tmp_path_seg_vec) {
    const uint8_t path_col_det_res = TrimPathByCollisionDetection(
        tmp_path_seg, calc_params_.lon_buffer_rev_trials);

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
    ILOG_INFO << "CalSinglePathInAdjust success!";
    ILOG_INFO << "cur_path_seg_vec:";
    double length = 0.0;
    for (const auto& path_seg : path_seg_vec) {
      pnc::geometry_lib::PrintSegmentInfo(path_seg);
      length += path_seg.Getlength();
    }
    if (length < 0.1) {
      ILOG_INFO << "this gear path is too small, lose it";
      path_seg_vec.clear();
    }
    return true;
  } else {
    ILOG_INFO << "CalSinglePathInAdjust fail";
    return false;
  }

  // path_seg_vec = tmp_path_seg_vec;

  // if (path_seg_vec.size() > 0) {
  //   ILOG_INFO <<"CalSinglePathInAdjust success");
  //   return true;
  // } else {
  //   ILOG_INFO <<"CalSinglePathInAdjust fail");
  //   return false;
  // }
}
// adjust plan end

const uint8_t ParallelPathGenerator::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg, const double buffer) {
  // ILOG_INFO << "--- collision detection ---" ;
  CollisionDetector::CollisionResult col_res;
  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    auto& line = path_seg.line_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    auto& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
  } else {
    // ILOG_INFO << "no support the seg type\n";
    return PATH_COL_INVALID;
  }

  const double remain_car_dist = col_res.remain_car_dist;
  const double remain_obs_dist = col_res.remain_obstacle_dist;
  const double safe_remain_dist =
      std::min(remain_car_dist, remain_obs_dist - buffer);

  // ILOG_INFO << "remain_car_dist = " << remain_car_dist;
  // ILOG_INFO << "remain_obs_dist = " << remain_obs_dist;
  // ILOG_INFO << "buffer = " << buffer;

  // ILOG_INFO << "ego local col pt = " <<
  // col_res.col_pt_ego_local.transpose(); ILOG_INFO << "col_pt_obs in slot =
  // " << col_res.col_pt_obs_global.transpose();

  if (safe_remain_dist < 0.0) {
    // ILOG_INFO << "the distance between obstacle and ego is smaller than "
    //              "min_safe_distance, collided! ";

    return PATH_COL_INVALID;
  }

  if (remain_car_dist <= safe_remain_dist) {
    // ILOG_INFO << "the path will not collide\n";
    return PATH_COL_NORMAL;
  }

  // ILOG_INFO << "the path will collide, need to be shorten to
  // safe_remain_dist"
  //           ;
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

const bool ParallelPathGenerator::CheckPathSegCollided(
    const pnc::geometry_lib::PathSegment& path_seg, const double buffer) const {
  CollisionDetector::CollisionResult col_res;
  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    auto& line = path_seg.line_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(line, line.heading);
  } else if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    auto& arc = path_seg.arc_seg;
    col_res = collision_detector_ptr_->UpdateByObsMap(arc, arc.headingA);
  } else {
    // ILOG_INFO << "no support the seg type\n";
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

const bool ParallelPathGenerator::CheckPathSegVecCollided(
    const std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const double buffer) const {
  for (const auto& path_seg : path_seg_vec) {
    if (CheckPathSegCollided(path_seg, buffer)) {
      return true;
    }
  }
  return false;
}

const bool ParallelPathGenerator::OneLinePlan(
    pnc::geometry_lib::LineSegment& line,
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const uint8_t current_gear) {
  ILOG_INFO << "--- try one line plan ---\n";

  pnc::geometry_lib::PathPoint pose;
  pose.Set(line.pA, line.heading);

  ILOG_INFO << "last pose deg" << pose.pos.transpose() << ", "
            << pose.heading * kRad2Deg;

  ILOG_INFO << "target line" << calc_params_.target_line.pA.transpose()
            << calc_params_.target_line.heading * kRad2Deg;

  if (pnc::geometry_lib::IsPoseOnLine(
          pose, calc_params_.target_line, apa_param.GetParam().static_pos_eps,
          apa_param.GetParam().static_heading_eps * kDeg2Rad)) {
    ILOG_INFO << "pose is on line, success\n";

    line.pB = calc_params_.target_line.pA;
    line.length = (line.pB - line.pA).norm();
    pnc::geometry_lib::PathSegment line_seg;

    if (line.length > 0.02) {
      // if (line.length > apa_param.GetParam().static_pos_eps) {
      const uint8_t seg_gear = pnc::geometry_lib::CalLineSegGear(line);

      if (pnc::geometry_lib::IsValidGear(seg_gear)) {
        ILOG_INFO << "the line gear is invalid\n";
        return false;
      }
      pnc::geometry_lib::PathSegment line_seg(seg_gear, line);
      path_seg_vec.emplace_back(line_seg);
      return true;
    } else {
      ILOG_INFO << "already plan to target pos\n";
      return true;
    }

  } else {
    ILOG_INFO << "pose is not on line, fail\n";
    return false;
  }
}

const bool ParallelPathGenerator::OneLinePlan(
    pnc::geometry_lib::LineSegment& line,
    const pnc::geometry_lib::PathPoint& target_pose) const {
  const pnc::geometry_lib::PathPoint start_pose(line.pA, line.heading);

  auto target_line = pnc::geometry_lib::BuildLineSegByPose(target_pose.pos,
                                                           target_pose.heading);
  if (!pnc::geometry_lib::IsPoseOnLine(
          start_pose, target_line, apa_param.GetParam().static_pos_eps,
          apa_param.GetParam().static_heading_eps * kDeg2Rad)) {
    // ILOG_INFO << "pose is not on line, fail\n";
    return false;
  }
  // ILOG_INFO <<"pose is on line, success");

  line.SetPoints(start_pose.pos, target_line.pA);

  if (line.length > 0.02) {
    const uint8_t seg_gear = pnc::geometry_lib::CalLineSegGear(line);
    if (!pnc::geometry_lib::IsValidGear(seg_gear)) {
      // ILOG_INFO << "the line gear is invalid\n";
      return false;
    }
    // ILOG_INFO <<"line plan to target pos success");
  } else {
    line.is_ignored = true;
    // ILOG_INFO <<"already is on target pos");
  }
  return true;
}

const bool ParallelPathGenerator::OneLinePlan(
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

const bool ParallelPathGenerator::OneLinePlanAlongEgoHeading(
    pnc::geometry_lib::LineSegment& line,
    const pnc::geometry_lib::PathPoint& target_pose) {
  ILOG_INFO << "--- try OneLinePlanAlongEgoHeading ---\n";

  const pnc::geometry_lib::PathPoint start_pose(line.pA, line.heading);

  const Eigen::Vector2d v_start_heading(std::cos(start_pose.heading),
                                        std::sin(start_pose.heading));

  if (pnc::mathlib::IsDoubleEqual(v_start_heading.x(), 0.0)) {
    ILOG_INFO << "ego heading is not possible equal to 90 deg, calc failed!";
    return false;
  }

  // firstly extend ego line to the same x coordination as target pose
  double len = (target_pose.pos.x() - start_pose.pos.x()) / v_start_heading.x();

  const pnc::geometry_lib::PathPoint fixed_target_pose(
      start_pose.pos + len * v_start_heading, start_pose.heading);

  ILOG_INFO << "fixed target pose =" << fixed_target_pose.pos.transpose()
            << " ,heading =" << fixed_target_pose.heading * kRad2Deg;

  if (!IsOnTargetLine(fixed_target_pose)) {
    ILOG_INFO << "pose is not on line, fail\n";
    return false;
  }
  ILOG_INFO << "pose is on line, success";

  line.SetPoints(start_pose.pos, fixed_target_pose.pos);
  if (line.length > 0.06) {
    const uint8_t seg_gear = pnc::geometry_lib::CalLineSegGear(line);
    if (!pnc::geometry_lib::IsValidGear(seg_gear)) {
      ILOG_INFO << "the line gear is invalid\n";
      return false;
    }
    ILOG_INFO << "line plan to target pos success";
  } else {
    ILOG_INFO << "path is too short";
    return false;
  }
  return true;
}

void ParallelPathGenerator::InsertLineSegAfterCurrentFollowLastPath(
    double extend_distance, double lon_buffer) {
  if (pnc::mathlib::IsDoubleEqual(extend_distance, 0.0)) {
    return;
  }

  if (output_.path_segment_vec.size() < 1) {
    return;
  }

  double current_path_len = 0.0;
  for (size_t i = output_.path_seg_index.first;
       i < output_.path_seg_index.second; i++) {
    current_path_len += output_.path_segment_vec[i].Getlength();
  }

  const pnc::geometry_lib::PathPoint current_path_end =
      output_.path_segment_vec[output_.path_seg_index.second].GetEndPose();
  pnc::geometry_lib::PrintPose("end pose = ", current_path_end);

  const bool is_last_path =
      std::fabs(current_path_end.pos.x() - calc_params_.target_pose.pos.x()) <
          1e-2 &&
      std::fabs(current_path_end.heading * kRad2Deg) < 1.0;
  ILOG_INFO << "is_last_path = " << is_last_path;

  if (is_last_path &&
      current_path_len >= apa_param.GetParam().min_path_length) {
    return;
  }

  // if (output_.is_last_path == true) {
  //   ILOG_INFO << "is last path, not extend path\n";
  //   return;
  // }

  auto& path_seg = output_.path_segment_vec[output_.path_seg_index.second];

  if (path_seg.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
      extend_distance < 0.0) {
    ILOG_INFO << "arc can not shorten\n";
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
      if (is_last_path) {
        extend_distance = std::max(
            0.1, 0.1 + 2.0 * apa_param.GetParam().min_path_length - path_len);
      } else {
        extend_distance = std::max(
            extend_distance, apa_param.GetParam().min_path_length - path_len);
      }
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
        std::min(remain_car_dist, remain_obstacle_dist - lon_buffer);
    // ILOG_INFO << "remain_car_dist = " << remain_car_dist
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

      ILOG_INFO << "inset line segment successful, extending length = "
                << extend_distance;

    } else {
      ILOG_INFO << "safe_remain_dist < 0.0, can not inset line segment";
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

void ParallelPathGenerator::ExtendCurrentFollowLastPath(
    double extend_distance) {
  if (pnc::mathlib::IsDoubleEqual(extend_distance, 0.0)) {
    return;
  }
  if (output_.is_last_path == true) {
    ILOG_INFO << "is last path, not extend path\n";
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
    ILOG_INFO << "--- extend distance collision --- ";
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
    // ILOG_INFO << "remain_car_dist = " << remain_car_dist
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

const bool ParallelPathGenerator::InsertTwoLinePathBetweenPathSeg(
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

const Eigen::Vector2d ParallelPathGenerator::CalEgoTurningCenter(
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

const bool ParallelPathGenerator::IsOnTarget(
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
              apa_param.GetParam().finish_parallel_heading_err * kDeg2Rad);
}

const bool ParallelPathGenerator::CheckLonToTarget(
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

const bool ParallelPathGenerator::IsOnTargetLine(
    const pnc::geometry_lib::PathPoint& current_pose) {
  const auto& target_pose = calc_params_.target_pose;

  const bool heading_condition =
      (std::fabs(pnc::geometry_lib::NormalizeAngle(current_pose.heading -
                                                   target_pose.heading)) <=
       apa_param.GetParam().finish_parallel_heading_err * kDeg2Rad);

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

const bool ParallelPathGenerator::CheckSamePose(
    const pnc::geometry_lib::PathPoint& pose1,
    const pnc::geometry_lib::PathPoint& pose2) const {
  return (
      (pose1.pos - pose2.pos).norm() <= apa_param.GetParam().static_pos_eps &&
      std::fabs(
          pnc::geometry_lib::NormalizeAngle(pose1.heading - pose2.heading)) <=
          apa_param.GetParam().static_heading_eps * kDeg2Rad);
}

const bool ParallelPathGenerator::CheckSamePos(
    const Eigen::Vector2d& pos0, const Eigen::Vector2d& pos1) const {
  return ((pos0 - pos1).norm() <= apa_param.GetParam().static_pos_eps);
}

void ParallelPathGenerator::AddPathSegToOutPut(
    const pnc::geometry_lib::PathSegment& path_seg) {
  output_.path_available = true;
  output_.path_segment_vec.emplace_back(path_seg);
  output_.length += path_seg.Getlength();
  output_.gear_cmd_vec.emplace_back(path_seg.seg_gear);
  output_.steer_vec.emplace_back(path_seg.seg_steer);
}

void ParallelPathGenerator::AddPathSegVecToOutput(
    const std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec) {
  for (const auto& path_seg : path_seg_vec) {
    AddPathSegToOutPut(path_seg);
  }
}

void ParallelPathGenerator::CalcEgoParams() {
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
  ILOG_INFO << "min_outer_front_corner_radius = "
            << calc_params_.min_outer_front_corner_radius;

  ILOG_INFO << " calc_params_.min_outer_front_corner_deta_y = "
            << calc_params_.min_outer_front_corner_deta_y;
}

const bool ParallelPathGenerator::AddLastLine(
    pnc::geometry_lib::PathSegment& last_path_seg) {
  if (output_.path_segment_vec.size() > 0) {
    const auto& end_pose = output_.path_segment_vec.back().GetEndPose();
    pnc::geometry_lib::PrintPose("end_pose", end_pose);

    const bool is_heading_small =
        pnc::mathlib::IsDoubleEqual(end_pose.heading * kRad2Deg, 0.0);
    ILOG_INFO << "is_heading_small = " << is_heading_small;

    ILOG_INFO << "input_.tlane.pt_terminal_pos.x() = "
              << input_.tlane.pt_terminal_pos.x();

    const bool is_x_diff_small = pnc::mathlib::IsDoubleEqual(
        end_pose.pos.x(), input_.tlane.pt_terminal_pos.x());

    ILOG_INFO << "is_x_diff_small = " << is_x_diff_small;

    if (is_heading_small && !is_x_diff_small) {
      const Eigen::Vector2d fixed_target_pos(input_.tlane.pt_terminal_pos.x(),
                                             end_pose.pos.y());

      const pnc::geometry_lib::LineSegment last_line(
          end_pose.pos, fixed_target_pos, end_pose.heading);

      const pnc::geometry_lib::PathSegment tmp(
          pnc::geometry_lib::CalLineSegGear(last_line), last_line);
      last_path_seg = tmp;
      return true;
    }
  }

  return false;
}

}  // namespace apa_planner
}  // namespace planning