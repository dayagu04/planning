#include "apa_path_planner.h"

#include <vector>

#include "debug_info_log.h"
#include "geometry_math.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

const bool ApaPathPlanner::Update(
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  collision_detector_ptr_ = collision_detector_ptr;
  return Update();
}

const bool ApaPathPlanner::SetCurrentPathSegIndex() {
  if (!output_.path_available) {
    return false;
  }

  if (output_.gear_cmd_vec.empty() || output_.path_segment_vec.empty() ||
      output_.steer_vec.empty()) {
    ILOG_INFO << "no path can get";
    return false;
  }

  const size_t N = output_.path_segment_vec.size();
  if (output_.gear_cmd_vec.size() != N || output_.steer_vec.size() != N) {
    ILOG_INFO << "size is not equal";
    return false;
  }

  // Now, the is_first_path is always true, and the first index is always 0
  if (output_.is_first_path == true) {
    // first get path segment from path_segment_vec, first and second index is
    // 0 at the moment
    output_.is_first_path = false;
  } else {
    if (output_.path_seg_index.second == N - 1) {
      ILOG_INFO << "no more path can get";
      return false;
    }
    output_.path_seg_index.first = output_.path_seg_index.second + 1;
  }

  if (output_.path_seg_index.first == N - 1) {
    output_.path_seg_index.second = output_.path_seg_index.first;
  }

  if (output_.path_seg_index.first >= N) {
    ILOG_INFO << "first index is err";
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
    ILOG_INFO << "second index is err";
    return false;
  }

  const int first = output_.path_seg_index.first;
  const int second = output_.path_seg_index.second;
  if (first < 0 || second < 0 || first > second) {
    ILOG_INFO << "first and second index is err";
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
    ILOG_INFO << "current path is final path";
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

void ApaPathPlanner::SetLineSegmentHeading() {
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

const bool ApaPathPlanner::CheckCurrentGearLength() {
  if (output_.path_segment_vec.size() < 1) {
    return false;
  }
  double length = 0.0;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
    length += output_.path_segment_vec[i].Getlength();
  }

  return (length > 0.3) ? true : false;
}

const bool ApaPathPlanner::SamplePathSeg(
    std::vector<pnc::geometry_lib::PathPoint>& path_point_vec,
    const std::vector<pnc::geometry_lib::PathSegment>& path_segment_vec) {
  if (path_segment_vec.empty()) {
    return false;
  }

  path_point_vec.clear();
  path_point_vec.reserve(PLANNING_TRAJ_POINTS_NUM);

  double length = 0.0;
  for (size_t i = 0; i <= path_segment_vec.size(); ++i) {
    length += path_segment_vec[i].Getlength();
  }
  size_t N = std::ceil(length / input_.sample_ds);
  double sample_ds = input_.sample_ds;
  const size_t max_seg_count = 7;
  if (N >= PLANNING_TRAJ_POINTS_NUM - 26 - max_seg_count) {
    N = PLANNING_TRAJ_POINTS_NUM - 26 - max_seg_count;
    sample_ds = length / static_cast<double>(N);
  }

  pnc::geometry_lib::PathPoint path_point;
  for (size_t i = 0; i < path_segment_vec.size(); ++i) {
    const auto& current_seg = path_segment_vec[i];
    std::vector<pnc::geometry_lib::PathPoint> tmp_path_point_vec;
    if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      SampleLineSegment(tmp_path_point_vec, current_seg.line_seg, sample_ds);
    } else {
      SampleArcSegment(tmp_path_point_vec, current_seg.arc_seg, sample_ds);
    }
    path_point_vec.insert(path_point_vec.end(), tmp_path_point_vec.begin(),
                          tmp_path_point_vec.end());
    if (i < path_segment_vec.size() - 1) {
      path_point_vec.pop_back();
    }
  }

  return true;
}

const bool ApaPathPlanner::SampleCurrentPathSeg() {
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
  output_.path_point_vec.reserve(PLANNING_TRAJ_POINTS_NUM);

  double length = 0.0;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
    length += output_.path_segment_vec[i].Getlength();
  }
  size_t N = std::ceil(length / input_.sample_ds);
  double sample_ds = input_.sample_ds;
  const size_t max_seg_count = 7;
  if (N >= PLANNING_TRAJ_POINTS_NUM - 26 - max_seg_count) {
    N = PLANNING_TRAJ_POINTS_NUM - 26 - max_seg_count;
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

void ApaPathPlanner::PrintOutputSegmentsInfo() const {
  ILOG_INFO << "-------------- OutputSegmentsInfo --------------";
  const size_t N =
      std::min(output_.path_seg_index.second - output_.path_seg_index.first + 1,
               output_.path_segment_vec.size());

  for (size_t i = 0; i < N; i++) {
    const auto& current_seg = output_.path_segment_vec[i];

    if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      const auto& line_seg = current_seg.line_seg;

      ILOG_INFO << "Segment [" << i << "] "
                << " LINE_SEGMENT "
                << " length= " << line_seg.length;

      ILOG_INFO << "seg_gear: " << static_cast<int>(current_seg.seg_gear);

      ILOG_INFO << "seg_steer: " << static_cast<int>(current_seg.seg_steer);

      ILOG_INFO << "start_pos: " << line_seg.pA.transpose();
      ILOG_INFO << "start_heading: " << line_seg.heading * kRad2Deg;
      ILOG_INFO << "end_pos: " << line_seg.pB.transpose() << "";
    } else {
      const auto& arc_seg = current_seg.arc_seg;

      ILOG_INFO << "Segment [" << i << "] "
                << "ARC_SEGMENT "
                << "length= " << arc_seg.length;

      ILOG_INFO << "seg_gear: " << static_cast<int>(current_seg.seg_gear);

      ILOG_INFO << "seg_steer: " << static_cast<int>(current_seg.seg_steer);

      ILOG_INFO << "start_pos: " << arc_seg.pA.transpose();
      ILOG_INFO << "start_heading: " << arc_seg.headingA * kRad2Deg;
      ILOG_INFO << "end_pos: " << arc_seg.pB.transpose();
      ILOG_INFO << "end_heading: " << arc_seg.headingB * kRad2Deg;
      ILOG_INFO << "center: " << arc_seg.circle_info.center.transpose()
                << "  radius = " << arc_seg.circle_info.radius << "";
    }
  }
}

void ApaPathPlanner::PrintSegmentInfo(
    const pnc::geometry_lib::PathSegment& seg) const {
  ILOG_INFO << "----";
  ILOG_INFO << "seg_gear: " << static_cast<int>(seg.seg_gear);

  ILOG_INFO << "seg_steer: " << static_cast<int>(seg.seg_steer);
  ILOG_INFO << "seg_type: " << static_cast<int>(seg.seg_type);
  ILOG_INFO << "length: " << seg.Getlength();

  if (seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    ILOG_INFO << "start_pos: " << seg.GetLineSeg().pA.transpose();
    ILOG_INFO << "start_heading: " << seg.GetLineSeg().heading * kRad2Deg;
    ILOG_INFO << "end_pos: " << seg.GetLineSeg().pB.transpose();
    ILOG_INFO << "end_heading: " << seg.GetLineSeg().heading * kRad2Deg;
  } else {
    ILOG_INFO << "start_pos: " << seg.GetArcSeg().pA.transpose();
    ILOG_INFO << "start_heading: " << seg.GetArcSeg().headingA * kRad2Deg;
    ILOG_INFO << "end_pos: " << seg.GetArcSeg().pB.transpose();
    ILOG_INFO << "end_heading: " << seg.GetArcSeg().headingB * kRad2Deg;
  }
}

void ApaPathPlanner::SampleLineSegment(
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

void ApaPathPlanner::SampleLineSegment(
    std::vector<pnc::geometry_lib::PathPoint>& path_point_vec,
    const pnc::geometry_lib::LineSegment& cur_line_seg, const double ds) {
  path_point_vec.clear();
  path_point_vec.reserve(50);
  if (cur_line_seg.is_ignored) {
    return;
  }

  // get first point
  pnc::geometry_lib::PathPoint path_point;
  path_point.Set(cur_line_seg.pA, cur_line_seg.heading);
  path_point_vec.emplace_back(path_point);

  if (cur_line_seg.length > ds) {
    auto pn = cur_line_seg.pA;
    const Eigen::Vector2d diff_vec =
        (cur_line_seg.pB - cur_line_seg.pA).normalized() * ds;

    double s = ds;
    while (s < cur_line_seg.length) {
      pn += diff_vec;
      path_point.Set(pn, cur_line_seg.heading);
      path_point_vec.emplace_back(path_point);
      s += ds;
    }
  }
  // check the dist of the last point and end point
  const double dist = (path_point_vec.back().pos - cur_line_seg.pB).norm();
  if (dist > 1e-2) {
    // get end point
    path_point.Set(cur_line_seg.pB, cur_line_seg.heading);
    path_point_vec.emplace_back(path_point);
  }
}

void ApaPathPlanner::SampleArcSegment(const pnc::geometry_lib::Arc& cur_arc_seg,
                                      const double ds) {
  if (cur_arc_seg.is_ignored) {
    return;
  }

  pnc::geometry_lib::PathPoint path_point;

  // get first point
  path_point.Set(cur_arc_seg.pA,
                 pnc::geometry_lib::NormalizeAngle(cur_arc_seg.headingA));
  output_.path_point_vec.emplace_back(path_point);

  if (cur_arc_seg.length > ds) {
    const auto& pO = cur_arc_seg.circle_info.center;
    double theta = cur_arc_seg.headingA;
    const double dtheta = ds / cur_arc_seg.circle_info.radius *
                          (cur_arc_seg.is_anti_clockwise ? 1.0 : -1.0);

    const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(dtheta);
    Eigen::Vector2d pn = cur_arc_seg.pA;

    Eigen::Vector2d v_n = cur_arc_seg.pA - cur_arc_seg.circle_info.center;

    double s = ds;
    while (s < cur_arc_seg.length) {
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
      (output_.path_point_vec.back().pos - cur_arc_seg.pB).norm();
  if (dist > 1e-2) {
    // get end point
    path_point.Set(cur_arc_seg.pB, cur_arc_seg.headingB);
    output_.path_point_vec.emplace_back(path_point);
  }
}

void ApaPathPlanner::SampleArcSegment(
    std::vector<pnc::geometry_lib::PathPoint>& path_point_vec,
    const pnc::geometry_lib::Arc& cur_arc_seg, const double ds) {
  path_point_vec.clear();
  path_point_vec.reserve(50);
  if (cur_arc_seg.is_ignored) {
    return;
  }

  pnc::geometry_lib::PathPoint path_point;

  // get first point
  path_point.Set(cur_arc_seg.pA,
                 pnc::geometry_lib::NormalizeAngle(cur_arc_seg.headingA));
  path_point_vec.emplace_back(path_point);

  if (cur_arc_seg.length > ds) {
    const auto& pO = cur_arc_seg.circle_info.center;
    double theta = cur_arc_seg.headingA;
    const double dtheta = ds / cur_arc_seg.circle_info.radius *
                          (cur_arc_seg.is_anti_clockwise ? 1.0 : -1.0);

    const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(dtheta);
    Eigen::Vector2d pn = cur_arc_seg.pA;

    Eigen::Vector2d v_n = cur_arc_seg.pA - cur_arc_seg.circle_info.center;

    double s = ds;
    while (s < cur_arc_seg.length) {
      v_n = rot_m * v_n;
      pn = pO + v_n;
      s += ds;
      theta += dtheta;

      path_point.Set(pn, pnc::geometry_lib::NormalizeAngle(theta));
      path_point_vec.emplace_back(path_point);
    }
  }

  // check the dist of the last point and end point
  const double dist = (path_point_vec.back().pos - cur_arc_seg.pB).norm();
  if (dist > 1e-2) {
    // get end point
    path_point.Set(cur_arc_seg.pB, cur_arc_seg.headingB);
    path_point_vec.emplace_back(path_point);
  }
}

const std::vector<double> ApaPathPlanner::GetPathEle(size_t index) const {
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

}  // namespace apa_planner
}  // namespace planning