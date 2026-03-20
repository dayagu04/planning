#include "ad_common/hdmap/hdmap_lane_boundary_info.h"

#include <algorithm>
#include <limits>

#include "ad_common/hdmap/hdmap_impl.h"
#include "ad_common/hdmap/hdmap_utils.h"
#include "ad_common/math/aabox2d.h"
#include "ad_common/math/linear_interpolation.h"
#include "ad_common/math/math_utils.h"

namespace ad_common {
namespace hdmap {
using ad_common::math::Vec2d;
using ::Map::LaneBoundary;
namespace {
// Minimum distance to remove duplicated points.
constexpr double kDuplicatedPointsEpsilon = 1e-7;
constexpr double kDuplicatedPointsEpsilonSq =
    kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
}  // namespace

LaneBoundaryInfo::LaneBoundaryInfo(const LaneBoundary &lane_boundary)
    : lane_boundary_(lane_boundary) {
  Init();
}

void LaneBoundaryInfo::GetBoundaryPoints(const LaneBoundary &boundary,
                                         std::vector<Vec2d> *points) const {
  points->clear();
  for (const auto &boundary_attribute : boundary.boundary_attributes()) {
    for (const auto &point : boundary_attribute.points()) {
      if (points->empty()) {
        points->emplace_back(point.x(), point.y());
      } else {
        const double dx = point.x() - points->back().x();
        const double dy = point.y() - points->back().y();
        const double distance_square = dx * dx + dy * dy;
        if (distance_square > kDuplicatedPointsEpsilonSq) {
          points->emplace_back(point.x(), point.y());
        }
      }
    }
  }
}

void LaneBoundaryInfo::Init() {
  is_valid_ = true;
  GetBoundaryPoints(lane_boundary_, &points_);
  if (points_.size() < 2) {
    is_valid_ = false;
    return;
  }
  // assert(points_.size() >= 2U);
  segments_.clear();
  segments_.reserve(points_.size() - 1);
  accumulated_s_.clear();
  accumulated_s_.reserve(points_.size());

  double s = 0;
  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    segments_.emplace_back(points_[i], points_[i + 1]);
    accumulated_s_.push_back(s);
    s += segments_.back().length();
  }
  if (segments_.empty()) {
    is_valid_ = false;
    return;
  }
  // assert(!segments_.empty());

  accumulated_s_.push_back(s);
  total_length_ = s;

  CreateKDTree();
}

double LaneBoundaryInfo::DistanceTo(const Vec2d &point) const {
  const auto segment_box =
      lane_boundary_segment_kdtree_->GetNearestObject(point);
  if (segment_box == nullptr) {
    return 0.0;
  }
  return segment_box->DistanceTo(point);
}

double LaneBoundaryInfo::DistanceTo(const Vec2d &point, Vec2d *map_point,
                                    double *s_offset,
                                    int *s_offset_index) const {
  if (map_point == nullptr || s_offset == nullptr ||
      s_offset_index == nullptr) {
    return 0.0;
  }

  const auto segment_box =
      lane_boundary_segment_kdtree_->GetNearestObject(point);
  if (segment_box == nullptr) {
    return 0.0;
  }
  int index = segment_box->id();
  double distance = segments_[index].DistanceTo(point, map_point);
  *s_offset_index = index;
  *s_offset =
      accumulated_s_[index] + segments_[index].start().DistanceTo(*map_point);
  return distance;
}

Vec2d LaneBoundaryInfo::GetNearestPoint(const Vec2d &point,
                                        double *distance) const {
  Vec2d empty_point;
  if (distance == nullptr) {
    return empty_point;
  }

  const auto segment_box =
      lane_boundary_segment_kdtree_->GetNearestObject(point);
  if (segment_box == nullptr) {
    return empty_point;
  }
  int index = segment_box->id();
  Vec2d nearest_point;
  *distance = segments_[index].DistanceTo(point, &nearest_point);

  return nearest_point;
}

void LaneBoundaryInfo::CreateKDTree() {
  ad_common::math::AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;

  segment_box_list_.clear();
  for (size_t id = 0; id < segments_.size(); ++id) {
    const auto &segment = segments_[id];
    segment_box_list_.emplace_back(
        ad_common::math::AABox2d(segment.start(), segment.end()), this,
        &segment, id);
  }
  lane_boundary_segment_kdtree_ = 
        std::make_unique<LaneBoundarySegmentKDTree>(segment_box_list_, params);
}

}  // namespace hdmap
}  // namespace ad_common
