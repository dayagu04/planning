#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ad_common/hdmap/hdmap_lane_boundary_info.h"
#include "ad_common/hdmap/hdmap_utils.h"
#include "ad_common/math/aaboxkdtree2d.h"
#include "ad_common/math/box2d.h"
#include "ad_common/math/line_segment2d.h"
#include "ad_common/math/vec2d.h"
#include "ehr.pb.h"

namespace ad_common {
namespace hdmap {

class LaneInfo;

using LaneSegmentBox =
    ObjectWithAABox<LaneInfo, ad_common::math::LineSegment2d>;
using LaneSegmentKDTree = ad_common::math::AABoxKDTree2d<LaneSegmentBox>;
using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;

// TODO(xjli32): LaneBoundaryInfo and LaneInfo should inherit from the same
// class
class LaneInfo {
 public:
  struct BoundarySegment {
    double start_s = 0.0;
    double end_s = 0.0;
    std::vector<::Map::BoundaryAttributes::Type> types;
  };

 public:
  LaneInfo(const ::Map::LaneData &lane,
           const LaneBoundaryInfoTable &lane_boundary_table,
           const LaneBoundaryInfoTable &road_boundary_table);

  const uint64_t id() const { return lane_.lane_id(); }
  const uint64_t lane_group_id() const { return lane_.lane_group_id(); }
  const ::Map::LaneData &lane() const { return lane_; }
  const std::vector<ad_common::math::Vec2d> &points() const { return points_; }
  const std::vector<ad_common::math::Vec2d> &unit_directions() const {
    return unit_directions_;
  }
  const bool is_valid() const { return is_valid_; }

  std::vector<::Map::BoundaryAttributes::Type> GetLeftLaneBoundaryTypes(
      const double s) const;

  std::vector<::Map::BoundaryAttributes::Type> GetRightLaneBoundaryTypes(
      const double s) const;

  std::vector<::Map::BoundaryAttributes::Type> GetLeftRoadBoundaryTypes(
      const double s) const;

  std::vector<::Map::BoundaryAttributes::Type> GetRightRoadBoundaryTypes(
      const double s) const;

  ad_common::math::Vec2d GetPoint(const double s) const;
  double GetHeading(const double s) const;
  double GetCurvature(const double s) const;
  const std::vector<double> &headings() const { return headings_; }
  const std::vector<ad_common::math::LineSegment2d> &segments() const {
    return segments_;
  }
  const std::vector<double> &accumulate_s() const { return accumulated_s_; }
  double total_length() const { return total_length_; }
  const std::vector<double> &left_widths() const { return left_widths_; }
  const std::vector<double> &right_widths() const { return right_widths_; }
  void GetWidth(const double s, double *left_width, double *right_width) const;
  double GetWidth(const double s) const;
  double GetEffectiveWidth(const double s) const;

  const std::vector<double> &left_road_widths() const {
    return left_road_width_;
  }
  const std::vector<double> &right_road_widths() const {
    return right_road_width_;
  }
  void GetRoadWidth(const double s, double *left_width,
                    double *right_width) const;
  double GetRoadWidth(const double s) const;

  double max_speed_limit() const { return lane_.max_speed_limit(); }
  double min_speed_limit() const { return lane_.min_speed_limit(); }

  bool IsOnLane(const ad_common::math::Vec2d &point) const;
  bool IsOnLane(const ad_common::math::Box2d &box) const;

  ad_common::math::Vec2d GetSmoothPoint(double s) const;
  double DistanceTo(const ad_common::math::Vec2d &point) const;
  double DistanceTo(const ad_common::math::Vec2d &point,
                    ad_common::math::Vec2d *map_point, double *s_offset,
                    int *s_offset_index) const;
  ad_common::math::Vec2d GetNearestPoint(const ad_common::math::Vec2d &point,
                                         double *distance) const;
  // TODO(xjli32): reduce computation complexity, avoid use this function if
  // possible
  bool GetProjection(const ad_common::math::Vec2d &point, double *accumulate_s,
                     double *lateral) const;

 private:
  void GetLaneCentralPoints(const ::Map::LaneData &lane,
                            std::vector<ad_common::math::Vec2d> *points);
  void Init();
  double GetWidth(const std::vector<double> &samples, const double s) const;
  void GetWidths();
  void GetBoundarySegments();
  void GetBoundarySegments(const ::Map::LaneBoundary &boundary,
                           std::vector<BoundarySegment> *boundary_segments);
  void CreateKDTree();
  std::vector<::Map::BoundaryAttributes::Type> GetBoundaryTypes(
      const std::vector<BoundarySegment> &boundary_segments,
      const double s) const;
  static bool BoundarySegmentComparator(const BoundarySegment &boundary_segment,
                                        const double s) {
    return boundary_segment.end_s < s;
  }

 private:
  const ::Map::LaneData &lane_;
  const LaneBoundaryInfoTable &lane_boundary_table_;
  const LaneBoundaryInfoTable &road_boundary_table_;
  std::vector<ad_common::math::Vec2d> points_;
  std::vector<ad_common::math::Vec2d> unit_directions_;
  std::vector<double> headings_;
  std::vector<ad_common::math::LineSegment2d> segments_;
  std::vector<double> accumulated_s_;
  double total_length_ = 0.0;
  std::vector<double> left_widths_;
  std::vector<double> right_widths_;

  std::vector<double> left_road_width_;
  std::vector<double> right_road_width_;

  std::vector<LaneSegmentBox> segment_box_list_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

  std::vector<BoundarySegment> left_lane_boundary_segments_;
  std::vector<BoundarySegment> right_lane_boundary_segments_;
  std::vector<BoundarySegment> left_road_boundary_segments_;
  std::vector<BoundarySegment> right_road_boundary_segments_;

  bool is_valid_ = true;
};

}  // namespace hdmap
}  // namespace ad_common
