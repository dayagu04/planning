#include "geometry_path_generator.h"

#include <vector>

#include "debug_info_log.h"
#include "geometry_math.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

const bool GeometryPathGenerator::Update(
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  collision_detector_ptr_ = collision_detector_ptr;
  return Update();
}

const bool GeometryPathGenerator::SetCurrentPathSegIndex() {
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

  return true;
}

const bool GeometryPathGenerator::CheckCurrentGearLength() {
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

const bool GeometryPathGenerator::SampleCurrentPathSeg() {
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

  std::vector<pnc::geometry_lib::PathSegment> cur_gear_path_segment_vec;
  double length = 0.0;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
    length += output_.path_segment_vec[i].Getlength();
    cur_gear_path_segment_vec.emplace_back(output_.path_segment_vec[i]);
  }
  size_t N = std::ceil(length / input_.sample_ds);
  double sample_ds = input_.sample_ds;
  const size_t max_seg_count = 7;
  if (N >= PLANNING_TRAJ_POINTS_NUM - 26 - max_seg_count) {
    N = PLANNING_TRAJ_POINTS_NUM - 26 - max_seg_count;
    sample_ds = length / static_cast<double>(N);
  }
  output_.actual_ds = sample_ds;
  output_.cur_gear_length = length;

  // for (size_t i = output_.path_seg_index.first;
  //      i <= output_.path_seg_index.second; ++i) {
  //   const auto& current_seg = output_.path_segment_vec[i];

  //   if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
  //     SampleLineSegment(current_seg.line_seg, sample_ds);
  //   } else {
  //     SampleArcSegment(current_seg.arc_seg, sample_ds);
  //   }
  //   if (i < output_.path_seg_index.second) {
  //     output_.path_point_vec.pop_back();
  //   }
  // }

  output_.all_gear_path_point_vec =
      pnc::geometry_lib::SamplePathSegVec(output_.path_segment_vec, sample_ds);

  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
  }
  output_.path_point_vec =
      pnc::geometry_lib::SamplePathSegVec(cur_gear_path_segment_vec, sample_ds);

  JSON_DEBUG_VALUE("current_gear_length", length);
  JSON_DEBUG_VALUE("current_gear_pt_size", output_.path_point_vec.size());
  JSON_DEBUG_VALUE("sample_ds", sample_ds);

  return true;
}

void GeometryPathGenerator::PrintOutputSegmentsInfo() const {
  ILOG_INFO << "-------------- OutputSegmentsInfo --------------";
  for (size_t i = 0; i < output_.path_segment_vec.size(); i++) {
    ILOG_INFO << "Segment [" << i << "]";
    output_.path_segment_vec[i].PrintInfo();
  }
}

const std::vector<double> GeometryPathGenerator::GetPathEle(
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

}  // namespace apa_planner
}  // namespace planning