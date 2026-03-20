#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ad_common/hdmap/hdmap_utils.h"
#include "ad_common/math/aaboxkdtree2d.h"
#include "ad_common/math/line_segment2d.h"
#include "ad_common/math/vec2d.h"
#include "ehr.pb.h"

namespace ad_common {
namespace hdmap {

class LaneBoundaryInfo;

using LaneBoundarySegmentBox =
    ObjectWithAABox<LaneBoundaryInfo, ad_common::math::LineSegment2d>;
using LaneBoundarySegmentKDTree =
    ad_common::math::AABoxKDTree2d<LaneBoundarySegmentBox>;
using LaneBoundaryInfoConstPtr = std::shared_ptr<const LaneBoundaryInfo>;
using LaneBoundaryInfoTable =
    std::unordered_map<uint64_t, std::shared_ptr<LaneBoundaryInfo>>;

// TODO(xjli32): LaneBoundaryInfo and LaneInfo should inherit from the same
// class
class LaneBoundaryInfo {
 public:
  explicit LaneBoundaryInfo(const ::Map::LaneBoundary &lane_boundary);

  const uint64_t id() const { return lane_boundary_.boundary_id(); }

  const bool is_valid() const { return is_valid_; }

  const ::Map::LaneBoundary &lane_boundary() const { return lane_boundary_; }

  const std::vector<ad_common::math::Vec2d> &points() const { return points_; }

  const std::vector<ad_common::math::LineSegment2d> &segments() const {
    return segments_;
  }
  const std::vector<double> &accumulate_s() const { return accumulated_s_; }

  double total_length() const { return total_length_; }

  double DistanceTo(const ad_common::math::Vec2d &point) const;
  double DistanceTo(const ad_common::math::Vec2d &point,
                    ad_common::math::Vec2d *map_point, double *s_offset,
                    int *s_offset_index) const;
  ad_common::math::Vec2d GetNearestPoint(const ad_common::math::Vec2d &point,
                                         double *distance) const;

 private:
  void Init();
  void GetBoundaryPoints(const ::Map::LaneBoundary &boundary,
                         std::vector<ad_common::math::Vec2d> *points) const;
  void CreateKDTree();

 private:
  const ::Map::LaneBoundary &lane_boundary_;
  std::vector<ad_common::math::Vec2d> points_;
  std::vector<ad_common::math::LineSegment2d> segments_;
  std::vector<double> accumulated_s_;
  double total_length_ = 0.0;

  std::vector<LaneBoundarySegmentBox> segment_box_list_;
  std::unique_ptr<LaneBoundarySegmentKDTree> lane_boundary_segment_kdtree_;

  bool is_valid_ = true;
};

}  // namespace hdmap
}  // namespace ad_common
