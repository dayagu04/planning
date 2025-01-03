#include "perpendicular_head_out_path_generator.h"

#include <math.h>

#include <cmath>
#include <cstddef>

#include "debug_info_log.h"
#include "geometry_math.h"

namespace planning {
namespace apa_planner {

void PerpendicularPathOutPlanner::Reset() { return; }

const bool PerpendicularPathOutPlanner::Update() {
  ILOG_INFO << "--------perpendicular path planner --------";

  // preprocess
  Preprocess();

  // reset output
  output_.Reset();

  if (input_.ego_pose.pos.x() < 3.0) {
    if (PreparePlan()) {
      ILOG_INFO << "parking out prepare plan success";
    } else {
      ILOG_INFO << " prepare plan fail , enter s t turn paralle plan";
      if (STurnParallelPlan()) {
        ILOG_INFO << "parking out s t turn paralle plan success";
      } else {
        ILOG_INFO << "frist plan fail";
        return false;
      };
    }
  }

  if (CheckReachTargetPose()) {
    ILOG_INFO << "parking out prepare plan to target pose";
    return true;
  }

  if (AdjustPlan()) {
    ILOG_INFO << "parking out adjust plan success";
    return true;
  }

  return false;
}

const bool PerpendicularPathOutPlanner::UpdatePyband(
    const Input& input,
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  ILOG_INFO << "--------perpendicular path planner pyband --------";

  input_ = input;
  collision_detector_ptr_ = collision_detector_ptr;

  // preprocess
  Preprocess();

  // // reset output
  // output_.Reset();

  if (PreparePlan()) {
    ILOG_INFO << "parking out prepare plan success";
  } else {
    ILOG_INFO << " prepare plan fail , enter s t turn paralle plan";
    if (STurnParallelPlan()) {
      ILOG_INFO << "parking out s t turn paralle plan success";
    } else {
      ILOG_INFO << "frist plan fail";
      return false;
    };
  }

  if (CheckReachTargetPose()) {
    ILOG_INFO << "parking out prepare plan to target pose";
    return true;
  }

  if (AdjustPlan()) {
    ILOG_INFO << "parking out adjust plan success";
    return true;
  }
  return false;
}

const bool PerpendicularPathOutPlanner::UpdateByPrePlan() { return false; };

void PerpendicularPathOutPlanner::Preprocess() {
  // calc_params_.Reset();
  calc_params_.complete_plan_again = false;
  calc_params_.single_plan_again = false;
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

const bool PerpendicularPathOutPlanner::PreparePlan() {
  uint8_t current_gear = input_.ref_gear;
  uint8_t current_arc_steer = input_.ref_arc_steer;
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;
  std::vector<double> x_offset_vec;
  const double slant_angle_rad = (input_.origin_pt_0_heading) * kDeg2Rad;
  const double cos_slant_angle = cos(input_.origin_pt_0_heading * kDeg2Rad);
  double x_min = 2.8;
  double x_max = 4.8;
  const double max_plan_num = 20;

  if (input_.origin_pt_0_heading != 0) {
    x_min = 4.0 / cos_slant_angle;
    x_max = 5.0 / cos_slant_angle;
  }

  double x_offset = 0.0;
  if (std::abs(current_pose.pos.x()) < x_min) {
    x_offset = x_min - current_pose.pos.x();
  }
  const double sampling_step = 0.15;
  // const size_t x_offset_vec_num = std::ceil((x_max - x_min) / sampling_step);
  // for (size_t i = 0; i < x_offset_vec_num; i++) {
  //   x_offset_vec.emplace_back(x_offset);
  //   x_offset += sampling_step;
  // }

  bool flag = false;
  bool prepare_success = false;
  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
  path_seg_vec.reserve(2);
  if (current_arc_steer == pnc::geometry_lib::SEG_STEER_STRAIGHT) {
    if (PreparePlanOnce(path_seg_vec, 3.0, calc_params_.turn_radius,
                        current_gear, current_arc_steer, current_pose)) {
      flag = true;
    }
  } else {
    for (size_t j = 0; j < max_plan_num && !flag; ++j) {
      ILOG_INFO << "x_offset = " << x_offset
                << ", (slot) ego x  = " << current_pose.pos.x();
      path_seg_vec.clear();
      if (PreparePlanOnce(path_seg_vec, x_offset, calc_params_.turn_radius,
                          current_gear, current_arc_steer, current_pose)) {
        prepare_success = true;
      }
      x_offset += sampling_step;
      flag = prepare_success;
    }
  }

  if (path_seg_vec.size() > 0) {
    output_.path_available = true;
    for (const auto& tmp_path_seg : path_seg_vec) {
      output_.path_segment_vec.emplace_back(tmp_path_seg);
      output_.length += tmp_path_seg.Getlength();
      output_.gear_cmd_vec.emplace_back(tmp_path_seg.seg_gear);
      output_.steer_vec.emplace_back(tmp_path_seg.seg_steer);
    }
    const auto& last_segment = path_seg_vec.back();
    pnc::geometry_lib::PathPoint last_pose;
    if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      last_pose.Set(last_segment.GetArcSeg().pB,
                    last_segment.GetArcSeg().headingB);
    } else if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      last_pose.Set(last_segment.GetLineSeg().pB,
                    last_segment.GetLineSeg().heading);
      ILOG_INFO << "last_segment not is arc path, current_arc_steer is : "
                << static_cast<int>(current_arc_steer);
    }
    current_pose = last_pose;
  }

  calc_params_.pt_inside = input_.tlane.pt_inside;

  if (!flag) {
    ILOG_INFO << "prepare first fail";
    output_.Reset();
    path_seg_vec.clear();
    return false;
  }

  calc_params_.first_path_gear = output_.current_gear;

  return true;
}

const bool PerpendicularPathOutPlanner::PreparePlanOnce(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const double& x_offset, const double& radius, const uint8_t current_gear,
    const uint8_t current_arc_steer,
    pnc::geometry_lib::PathPoint current_pose) {
  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(2);
  const double start_heading = current_pose.heading;
  const double current_turn_radius = radius;
  pnc::geometry_lib::PathPoint start_pose;
  Eigen::Vector2d ego_heading_vec =
      pnc::geometry_lib::GenHeadingVec(current_pose.heading);

  Eigen::Vector2d start_pos = current_pose.pos + x_offset * ego_heading_vec;
  start_pose.Set(start_pos,
                 start_heading);  // TODO::adjust plan

  pnc::geometry_lib::LineSegment tmp_line;
  tmp_line.Reset();
  tmp_line.pA = current_pose.pos;
  tmp_line.pB = start_pose.pos;
  tmp_line.heading = start_pose.heading;
  tmp_line.length = (tmp_line.pB - tmp_line.pA).norm();
  if (CheckArcOrLineAvailable(tmp_line)) {
    if (pnc::geometry_lib::CalLineSegGear(tmp_line) == current_gear) {
      pnc::geometry_lib::PathSegment line_seg(current_gear, tmp_line);
      line_seg.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;
      tmp_path_seg_vec.emplace_back(line_seg);
    }
  }

  if (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT ||
      current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
    // cal pre line tangent vec and normal vec
    const Eigen::Vector2d line_tangent_vec =
        pnc::geometry_lib::GenHeadingVec(start_pose.heading);

    Eigen::Vector2d line_normal_vec;
    if (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
      line_normal_vec << line_tangent_vec.y(), -line_tangent_vec.x();
    } else if (current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
      line_normal_vec << -line_tangent_vec.y(), line_tangent_vec.x();
    }

    // sure line_normal_vec towards downward along the x axis.
    // if (line_normal_vec.x() > 0.0) {
    //   line_normal_vec = -1.0 * line_normal_vec;
    // }

    // gen prepare line
    calc_params_.prepare_line = pnc::geometry_lib::BuildLineSegByPose(
        start_pose.pos, start_pose.heading);

    calc_params_.pre_line_tangent_vec = line_tangent_vec;
    calc_params_.pre_line_normal_vec = line_normal_vec;

    // gen one arc
    const Eigen::Vector2d current_turn_center =
        start_pose.pos + line_normal_vec * current_turn_radius;
    pnc::geometry_lib::Arc current_arc;
    current_arc.Reset();
    current_arc.circle_info.center = current_turn_center;
    current_arc.circle_info.radius = current_turn_radius;
    current_arc.pA = start_pose.pos;
    current_arc.headingA =
        pnc::geometry_lib::NormalizeAngle(start_pose.heading);

    // current_arc.pB << current_turn_center.x() + current_turn_radius,
    //     current_turn_center.y();

    current_arc.pB << current_turn_center.x() +
                          current_turn_radius *
                              sin((90 - input_.origin_pt_0_heading) * kDeg2Rad),
        current_turn_center.y() +
            current_turn_radius *
                cos((90 - input_.origin_pt_0_heading) * kDeg2Rad);

    if (CheckArcOrLineAvailable(current_arc) &&
        pnc::geometry_lib::CompleteArcInfo(current_arc)) {
      const uint8_t tmp_arc_steer = pnc::geometry_lib::CalArcSteer(current_arc);
      const uint8_t tmp_gear = pnc::geometry_lib::CalArcGear(current_arc);
      if (tmp_arc_steer == current_arc_steer && tmp_gear == current_gear) {
        pnc::geometry_lib::PathSegment arc_seg(tmp_arc_steer, tmp_gear,
                                               current_arc);
        tmp_path_seg_vec.emplace_back(arc_seg);
      }
    }
  }

  // param.lat_inflation = apa_param.GetParam().car_lat_inflation_strict;
  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.08));
  bool prepare_success = true;
  // collision detect
  for (pnc::geometry_lib::PathSegment& tmp_path_seg : tmp_path_seg_vec) {
    if (&tmp_path_seg == &tmp_path_seg_vec.back()) {
      collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.20));
    }
    const PathColDetRes path_col_det_res =
        TrimPathByCollisionDetection(tmp_path_seg, 0.08);
    // path_seg_vec.emplace_back(tmp_path_seg);
    if (path_col_det_res == PathColDetRes::NORMAL) {
      path_seg_vec.emplace_back(tmp_path_seg);
    } else if (path_col_det_res == PathColDetRes::SHORTEN) {
      path_seg_vec.emplace_back(tmp_path_seg);
    } else if (path_col_det_res == PathColDetRes::INVALID) {
      prepare_success = false;
      ILOG_INFO << " Path Col Det Res::INVALID ";
      break;
    } else if (path_col_det_res == PathColDetRes::INSIDE_STUCK) {
      prepare_success = false;
      ILOG_INFO << " Path Col Det Res::INSIDE_STUCK ";
      break;
    }
  }

  return prepare_success;
}

const bool PerpendicularPathOutPlanner::AdjustPlan() {
  ILOG_INFO << "-----prking out adjust plan-----";
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
    current_arc_steer = output_.steer_vec.back();
    ILOG_INFO << "continue to plan after prepareplan";
  }

  // check pose, if error is large, adjust is not suitable
  // if (!CheckAdjustPlanSuitable(current_pose)) {
  //   return false;
  // }

  // check gear and steer
  if ((current_gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
       current_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) ||
      (current_arc_steer != pnc::geometry_lib::SEG_STEER_LEFT &&
       current_arc_steer != pnc::geometry_lib::SEG_STEER_RIGHT)) {
    ILOG_INFO << "ref_gear or ref_arc_steer error";
    return false;
  }

  ILOG_INFO << "try adjust plan to target point";
  bool success = true;

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.clear();
  tmp_path_seg_vec.reserve(2);
  bool is_finish = false;
  size_t plan_num = 0;

  while (!is_finish) {
    const double safe_dist_r = 0.8;
    const double safe_dist_d = 0.268;

    // set current arc steer
    if (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
      current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else if (current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
      current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else {
      ILOG_INFO << "fault ref_arc_steer state!";
      return false;
    }

    // set current gear
    if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    } else if (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
      current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    } else {
      ILOG_INFO << "fault ref_gear state!";
      return false;
    }
    if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      AdjustPlanOnce(tmp_path_seg_vec, current_pose, safe_dist_d,
                     current_arc_steer, current_gear);
    } else if (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
      AdjustPlanOnce(tmp_path_seg_vec, current_pose, safe_dist_r,
                     current_arc_steer, current_gear);
    }

    double finish_heading_deg = current_pose.heading * kRad2Deg;

    if (std::fabs(finish_heading_deg) < 110 &&
        std::fabs(finish_heading_deg) > 80) {
      is_finish = true;
    }

    plan_num++;
    if (plan_num > 5) {
      is_finish = true;
    }
  }

  if (tmp_path_seg_vec.size() > 0) {
    output_.path_available = true;
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
  }

  // if (CheckReachTargetPose(current_pose)) {
  //   if (output_.path_segment_vec.size() > 0) {
  //     const auto& last_segment = output_.path_segment_vec.back();
  //     if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_ARC &&
  //         last_segment.seg_gear != pnc::geometry_lib::SEG_GEAR_DRIVE) {
  //       ILOG_INFO << "last line is not SEG_GEAR_DRIVE, should lose";
  //       output_.length -= last_segment.Getlength();
  //       output_.path_segment_vec.pop_back();
  //       output_.gear_cmd_vec.pop_back();
  //       output_.steer_vec.pop_back();
  //     }
  //   }
  // }

  if (output_.path_segment_vec.size() == 0) {
    success = false;
    output_.path_available = false;
  }

  // CollisionDetector::Paramters params;
  // collision_detector_ptr_->SetParam(params);

  if (!success) {
    ILOG_INFO << "adjust plan failed!";
    return false;
  }

  return success;
}

const bool PerpendicularPathOutPlanner::AdjustPlanOnce(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    pnc::geometry_lib::PathPoint& current_pose, const double safe_dist,
    const uint8_t current_arc_steer, const uint8_t current_gear) {
  const double current_turn_radius = calc_params_.turn_radius;
  const Eigen::Vector2d line_tangent_vec =
      pnc::geometry_lib::GenHeadingVec(current_pose.heading);
  if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE &&
      current_pose.pos.x() < 5.2) {
    const double move_lon =
        6.5 - current_pose.pos.x();  // Two meters higher than the slot
    const double need_move_length = move_lon / cos(current_pose.heading);
    pnc::geometry_lib::PathPoint tmp_pose;
    Eigen::Vector2d tmp_pos =
        current_pose.pos + need_move_length * line_tangent_vec;
    tmp_pose.Set(tmp_pos,
                 current_pose.heading);  // TODO::adjust plan

    pnc::geometry_lib::LineSegment tmp_line;
    tmp_line.Reset();
    tmp_line.pA = current_pose.pos;
    tmp_line.pB = tmp_pose.pos;
    tmp_line.heading = tmp_pose.heading;
    if (CheckArcOrLineAvailable(tmp_line)) {
      if (pnc::geometry_lib::CalLineSegGear(tmp_line) == current_gear) {
        pnc::geometry_lib::PathSegment line_seg(current_gear, tmp_line);
        line_seg.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;
        path_seg_vec.emplace_back(line_seg);
      }
    }
    current_pose.Set(tmp_line.pB, tmp_line.heading);
  }

  Eigen::Vector2d line_normal_vec;
  if (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
    line_normal_vec << line_tangent_vec.y(), -line_tangent_vec.x();
  } else if (current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
    line_normal_vec << -line_tangent_vec.y(), line_tangent_vec.x();
  }
  // gen arc
  const Eigen::Vector2d current_turn_center =
      current_pose.pos + line_normal_vec * current_turn_radius;
  pnc::geometry_lib::Arc current_arc;
  current_arc.Reset();
  current_arc.circle_info.center = current_turn_center;
  current_arc.circle_info.radius = current_turn_radius;
  current_arc.pA = current_pose.pos;
  current_arc.headingA =
      pnc::geometry_lib::NormalizeAngle(current_pose.heading);

  if (current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    current_arc.pB << current_turn_center.x() -
                          current_turn_radius *
                              sin((90 - input_.origin_pt_0_heading) * kDeg2Rad),
        current_turn_center.y() -
            current_turn_radius *
                cos((90 - input_.origin_pt_0_heading) * kDeg2Rad);
  } else if (current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    current_arc.pB << current_turn_center.x() +
                          current_turn_radius *
                              sin((90 - input_.origin_pt_0_heading) * kDeg2Rad),
        current_turn_center.y() +
            current_turn_radius *
                cos((90 - input_.origin_pt_0_heading) * kDeg2Rad);
  }

  if (CheckArcOrLineAvailable(current_arc) &&
      pnc::geometry_lib::CompleteArcInfo(current_arc)) {
    const uint8_t tmp_arc_steer = pnc::geometry_lib::CalArcSteer(current_arc);
    const uint8_t tmp_gear = pnc::geometry_lib::CalArcGear(current_arc);

    if (tmp_arc_steer == current_arc_steer && tmp_gear == current_gear) {
      pnc::geometry_lib::PathSegment arc_seg(tmp_arc_steer, tmp_gear,
                                             current_arc);
      const PathColDetRes arc_path_col_det_res =
          TrimPathByCollisionDetection(arc_seg, safe_dist);
      if (arc_path_col_det_res == PathColDetRes::INVALID ||
          arc_path_col_det_res == PathColDetRes::INSIDE_STUCK) {
        ILOG_INFO << "arc path is invalid, adjust plan failed";
        return false;
      }
      path_seg_vec.emplace_back(arc_seg);
      current_pose.Reset();
      current_pose.Set(arc_seg.arc_seg.pB, arc_seg.arc_seg.headingB);
    }
  }
  return true;
}

const bool PerpendicularPathOutPlanner::STurnParallelPlan() {
  ILOG_INFO << "-----prking out s turn paralle plan-----";
  uint8_t current_gear = input_.ref_gear;
  pnc::geometry_lib::PathPoint current_pose = input_.ego_pose;

  double steer_change_radius = apa_param.GetParam().max_radius_in_slot;
  std::vector<double> offset_y_vec;
  offset_y_vec.clear();
  const double y_min = -0.12;
  const double y_max = 0.12;
  double offset_y = y_min;
  for (size_t i = 0; i < std::ceil((y_max - y_min) / 0.01); i++) {
    offset_y_vec.emplace_back(offset_y);
    offset_y += 0.01;
  }

  bool flag = false;
  bool prepare_success = false;
  std::vector<pnc::geometry_lib::PathSegment> path_seg_vec;
  path_seg_vec.reserve(3);

  for (size_t j = 0; j < offset_y_vec.size() && !flag; ++j) {
    const double& offset_y = offset_y_vec[j];
    path_seg_vec.clear();
    if (STurnParallelPlanOnce(path_seg_vec, offset_y, calc_params_.turn_radius,
                              current_pose, current_gear, input_.ref_arc_steer,
                              steer_change_radius)) {
      prepare_success = true;
      ILOG_INFO << "y_offset = " << offset_y;
    }
    flag = prepare_success;
  }

  if (path_seg_vec.size() > 0) {
    output_.path_available = true;
    for (const auto& tmp_path_seg : path_seg_vec) {
      output_.path_segment_vec.emplace_back(tmp_path_seg);
      output_.length += tmp_path_seg.Getlength();
      output_.gear_cmd_vec.emplace_back(tmp_path_seg.seg_gear);
      output_.steer_vec.emplace_back(tmp_path_seg.seg_steer);
    }
    const auto& last_segment = path_seg_vec.back();
    pnc::geometry_lib::PathPoint last_pose;
    if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
      last_pose.Set(last_segment.GetArcSeg().pB,
                    last_segment.GetArcSeg().headingB);
    } else if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      last_pose.Set(last_segment.GetLineSeg().pB,
                    last_segment.GetLineSeg().heading);
      ILOG_INFO << "last_segment not is arc path, current_arc_steer is : "
                << static_cast<int>(input_.ref_arc_steer);
    }
    current_pose = last_pose;
  }
  if (!flag) {
    ILOG_INFO << "s turn paralle plan first fail";
    return false;
  }

  calc_params_.first_path_gear = output_.current_gear;

  return true;
}

const bool PerpendicularPathOutPlanner::STurnParallelPlanOnce(
    std::vector<pnc::geometry_lib::PathSegment>& path_seg_vec,
    const double offset_y, const double radius,
    const pnc::geometry_lib::PathPoint& current_pose,
    const uint8_t current_gear, const uint8_t current_arc_steer,
    const double steer_change_radius) {
  pnc::geometry_lib::Arc arc_s_1;
  arc_s_1.headingA = current_pose.heading;
  arc_s_1.pA = current_pose.pos;
  // check if it is possible to take S turn to target line

  // if (std::fabs(arc_s_1.headingA - calc_params_.target_line.heading) >
  //     apa_param.GetParam().static_heading_eps * kDeg2Rad) {
  //   ILOG_INFO << "body no align";
  //   return false;
  // }
  arc_s_1.headingA = calc_params_.target_line.heading;

  // if (std::fabs(arc_s_1.pA.y() - calc_params_.target_line.pA.y()) <
  //     apa_param.GetParam().static_pos_eps) {
  //   ILOG_INFO << "current pos is already on target line, no need to "
  //                "STurnParallelPlan";
  //   return true;
  // }

  double slot_occupied_ratio = CalOccupiedRatio(current_pose);

  const std::vector<double> ratio_tab = {0.0, 0.2, 0.5, 0.8, 1.0};
  const double radius_change = apa_param.GetParam().max_radius_in_slot -
                               apa_param.GetParam().min_radius_out_slot;
  if (radius_change < 1e-8) {
    ILOG_INFO << "radius setting is err";
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

  // ILOG_INFO << "real_steer_change_radius = " << real_steer_change_radius
  //          );

  arc_s_1.circle_info.radius = real_steer_change_radius;

  Eigen::Vector2d s_end_point;
  s_end_point.x() = current_pose.pos.x();
  // s_end_point.y() = (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT)
  //                       ? offset_y
  //                       : -offset_y;
  s_end_point.y() = offset_y;

  pnc::geometry_lib::Arc arc_s_2;
  arc_s_2.circle_info.radius = real_steer_change_radius;
  arc_s_2.pB = s_end_point;
  arc_s_2.headingB = 0.0;

  bool success = pnc::geometry_lib::CalTwoArcWithSameHeading(arc_s_1, arc_s_2,
                                                             current_gear);

  std::vector<pnc::geometry_lib::PathSegment> tmp_path_seg_vec;
  tmp_path_seg_vec.reserve(3);
  if (success) {
    // check gear and steer can or not meet needs
    bool arc_s_1_available = false;
    if (CheckArcOrLineAvailable(arc_s_1)) {
      const uint8_t steer_1 = pnc::geometry_lib::CalArcSteer(arc_s_1);
      const uint8_t gear_1 = pnc::geometry_lib::CalArcGear(arc_s_1);
      if (gear_1 == current_gear) {
        // ILOG_INFO << "steer_1 = " << static_cast<int>(steer_1));
        tmp_path_seg_vec.emplace_back(
            pnc::geometry_lib::PathSegment(steer_1, gear_1, arc_s_1));
        tmp_path_seg_vec.back().plan_type = pnc::geometry_lib::PLAN_TYPE_S_TURN;
        arc_s_1_available = true;
      }
    }

    bool arc_s_2_available = false;
    if (CheckArcOrLineAvailable(arc_s_2)) {
      const uint8_t steer_2 = pnc::geometry_lib::CalArcSteer(arc_s_2);
      const uint8_t gear_2 = pnc::geometry_lib::CalArcGear(arc_s_2);
      if (gear_2 == current_gear) {
        // ILOG_INFO << "steer_2 = " << static_cast<int>(steer_2));
        tmp_path_seg_vec.emplace_back(
            pnc::geometry_lib::PathSegment(steer_2, gear_2, arc_s_2));
        tmp_path_seg_vec.back().plan_type = pnc::geometry_lib::PLAN_TYPE_S_TURN;
        arc_s_2_available = true;
      }
    }

    if (tmp_path_seg_vec.empty()) {
      success = false;
    }

    if (tmp_path_seg_vec.size() == 1 && arc_s_1_available) {
      double dist = pnc::geometry_lib::CalPoint2LineDist(
          arc_s_1.pB, calc_params_.target_line);
      double head_err = std::fabs(pnc::geometry_lib::NormalizeAngle(
          arc_s_1.headingB - calc_params_.target_line.heading));
      if (dist <= apa_param.GetParam().static_pos_eps - 1e-6 &&
          head_err <=
              apa_param.GetParam().static_heading_eps * kDeg2Rad - 1e-6) {
      } else {
        tmp_path_seg_vec.clear();
        success = false;
      }
    }

    if (tmp_path_seg_vec.size() == 1 && arc_s_2_available) {
      const double dist = (arc_s_2.pA - current_pose.pos).norm();
      const double heading_err =
          std::fabs(arc_s_2.headingA - current_pose.heading);
      if (dist < apa_param.GetParam().target_pos_err - 1e-6 &&
          heading_err <
              apa_param.GetParam().target_heading_err * kDeg2Rad - 1e-6) {
        return true;
      } else {
        tmp_path_seg_vec.clear();
        return false;
      }
    }
  }
  pnc::geometry_lib::PathPoint next_seg_start_pose;
  next_seg_start_pose.Set(tmp_path_seg_vec.back().arc_seg.pB,
                          tmp_path_seg_vec.back().arc_seg.headingB);
  const double min_lon_turn_offset = 4.2;

  if (next_seg_start_pose.pos.x() < min_lon_turn_offset) {
    pnc::geometry_lib::PathPoint tmp_end_pose;
    Eigen::Vector2d tmp_pos;
    tmp_pos << min_lon_turn_offset, next_seg_start_pose.pos.y();
    tmp_end_pose.Set(tmp_pos,
                     next_seg_start_pose.heading);  // TODO::adjust plan

    pnc::geometry_lib::LineSegment tmp_line;
    tmp_line.Reset();
    tmp_line.pA = next_seg_start_pose.pos;
    tmp_line.pB = tmp_end_pose.pos;
    tmp_line.heading = tmp_end_pose.heading;

    if (CheckArcOrLineAvailable(tmp_line)) {
      if (pnc::geometry_lib::CalLineSegGear(tmp_line) == current_gear) {
        pnc::geometry_lib::PathSegment line_seg(current_gear, tmp_line);
        line_seg.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;
        tmp_path_seg_vec.emplace_back(line_seg);
        next_seg_start_pose.Set(line_seg.GetEndPose().pos,
                                line_seg.GetEndHeading());
      }
    }
  }

  if (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT ||
      current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
    // // cal pre line tangent vec and normal vec
    // pnc::geometry_lib::PathPoint s_path_last_pose;
    // s_path_last_pose.Set(tmp_path_seg_vec.back().arc_seg.pB,
    //                      tmp_path_seg_vec.back().arc_seg.headingB);
    const Eigen::Vector2d line_tangent_vec =
        pnc::geometry_lib::GenHeadingVec(next_seg_start_pose.heading);

    Eigen::Vector2d line_normal_vec;
    if (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
      line_normal_vec << line_tangent_vec.y(), -line_tangent_vec.x();
    } else if (current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
      line_normal_vec << -line_tangent_vec.y(), line_tangent_vec.x();
    }

    // gen one arc
    const double current_turn_radius = radius;
    const Eigen::Vector2d current_turn_center =
        next_seg_start_pose.pos + line_normal_vec * current_turn_radius;
    pnc::geometry_lib::Arc current_arc;
    current_arc.Reset();
    current_arc.circle_info.center = current_turn_center;
    current_arc.circle_info.radius = current_turn_radius;
    current_arc.pA = next_seg_start_pose.pos;
    current_arc.headingA =
        pnc::geometry_lib::NormalizeAngle(next_seg_start_pose.heading);

    current_arc.pB << current_turn_center.x() +
                          current_turn_radius *
                              sin((90 - input_.origin_pt_0_heading) * kDeg2Rad),
        current_turn_center.y() +
            current_turn_radius *
                cos((90 - input_.origin_pt_0_heading) * kDeg2Rad);

    if (CheckArcOrLineAvailable(current_arc) &&
        pnc::geometry_lib::CompleteArcInfo(current_arc)) {
      const uint8_t tmp_arc_steer = pnc::geometry_lib::CalArcSteer(current_arc);
      const uint8_t tmp_gear = pnc::geometry_lib::CalArcGear(current_arc);
      if (tmp_arc_steer == current_arc_steer && tmp_gear == current_gear) {
        pnc::geometry_lib::PathSegment arc_seg(tmp_arc_steer, tmp_gear,
                                               current_arc);
        tmp_path_seg_vec.emplace_back(arc_seg);
      }
    }
  }

  // collision detect
  collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.08));
  for (pnc::geometry_lib::PathSegment& tmp_path_seg : tmp_path_seg_vec) {
    if (&tmp_path_seg == &tmp_path_seg_vec.back()) {
      collision_detector_ptr_->SetParam(CollisionDetector::Paramters(0.20));
    }
    const PathColDetRes path_col_det_res =
        TrimPathByCollisionDetection(tmp_path_seg, 0.08);
    // path_seg_vec.emplace_back(tmp_path_seg);
    if (path_col_det_res == PathColDetRes::NORMAL) {
      path_seg_vec.emplace_back(tmp_path_seg);
    } else if (path_col_det_res == PathColDetRes::SHORTEN) {
      path_seg_vec.emplace_back(tmp_path_seg);
    } else if (path_col_det_res == PathColDetRes::INVALID) {
      success = false;
      ILOG_INFO << " Path Col Det Res::INVALID ";
      break;
    } else if (path_col_det_res == PathColDetRes::INSIDE_STUCK) {
      success = false;
      ILOG_INFO << " Path Col Det Res::INSIDE_STUCK ";
      break;
    }
  }

  if (!success) {
  } else {
    ILOG_INFO << "s turn parallel plan success";
  }

  return success;
}

const double PerpendicularPathOutPlanner::CalOccupiedRatio(
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

const bool PerpendicularPathOutPlanner::CheckReachTargetPose() {
  if (input_.ref_arc_steer == pnc::geometry_lib::SEG_STEER_STRAIGHT) {
    return true;
  }

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

const bool PerpendicularPathOutPlanner::CheckReachTargetPose(
    const pnc::geometry_lib::PathPoint& current_pose) {
  if (std::fabs(pnc::geometry_lib::NormalizeAngle(current_pose.heading)) <=
          (apa_param.GetParam().perpendicular_park_out_max_target_heading -
           input_.origin_pt_0_heading) *
              kDeg2Rad &&
      std::fabs(pnc::geometry_lib::NormalizeAngle(current_pose.heading)) >=
          (apa_param.GetParam().perpendicular_park_out_min_target_heading -
           input_.origin_pt_0_heading) *
              kDeg2Rad) {
    return true;
  }
  return false;
}

const bool PerpendicularPathOutPlanner::CheckArcOrLineAvailable(
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

const bool PerpendicularPathOutPlanner::CheckArcOrLineAvailable(
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

// collision detect start
const PerpendicularPathOutPlanner::PathColDetRes
PerpendicularPathOutPlanner::TrimPathByCollisionDetection(
    pnc::geometry_lib::PathSegment& path_seg) {
  return TrimPathByCollisionDetection(
      path_seg, apa_param.GetParam().col_obs_safe_dist_normal);
}

const PerpendicularPathOutPlanner::PathColDetRes
PerpendicularPathOutPlanner::TrimPathByCollisionDetection(
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
    return PathColDetRes::INVALID;
  }

  const double remain_car_dist = col_res.remain_car_dist;
  const double remain_obs_dist = col_res.remain_obstacle_dist;
  const double safe_remain_dist =
      std::min(remain_car_dist, remain_obs_dist - safe_dist);

  // ILOG_INFO << "remain_car_dist = "
  //             << remain_car_dist << "  remain_obs_dist = " << remain_obs_dist
  //             << "  safe_remain_dist = " << safe_remain_dist);

  if (safe_remain_dist < 1e-5) {
    ILOG_INFO << "safe_remain_dist is samller than 0.0, the path donot meet "
                 "requirements";

    return PathColDetRes::INVALID;
  }

  if (remain_car_dist > safe_remain_dist + 1e-3) {
    if (col_res.col_pt_obs_global.transpose().x() < 6 &&
        path_seg.seg_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
      // ILOG_INFO << "**** collision_point = "
      //             << col_res.col_pt_ego_local.transpose()
      //             << "  obs_pt_global = "
      //             << col_res.col_pt_ego_global.transpose();
      return PathColDetRes::INSIDE_STUCK;
    }

    // ILOG_INFO << "collision_point = "
    //             << col_res.col_pt_ego_local.transpose()
    //             << "  obs_pt_global = " <<
    //             col_res.col_pt_ego_global.transpose()
    //             << "  car_line_order = " << col_res.car_line_order);

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
    path_seg.collision_flag = true;

    return PathColDetRes::SHORTEN;
  } else {
    // ILOG_INFO << "the path will not collide";
    path_seg.collision_flag = false;
    return PathColDetRes::NORMAL;
  }
}

const PerpendicularPathOutPlanner::PlannerParams&
PerpendicularPathOutPlanner::GetCalcParams() {
  return calc_params_;
}

}  // namespace apa_planner
}  // namespace planning
