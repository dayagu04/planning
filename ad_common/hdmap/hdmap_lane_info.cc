#include "ad_common/hdmap/hdmap_lane_info.h"

#include <algorithm>
#include <limits>

#include "ad_common/hdmap/hdmap_impl.h"
#include "ad_common/hdmap/hdmap_utils.h"
#include "ad_common/math/aabox2d.h"
#include "ad_common/math/linear_interpolation.h"
#include "ad_common/math/math_utils.h"

namespace ad_common {
namespace hdmap {
using ad_common::math::Box2d;
using ad_common::math::Vec2d;
using ::Map::LaneBoundary;
using ::Map::LaneData;
namespace {
// Minimum distance to remove duplicated points.
constexpr double kDuplicatedPointsEpsilon = 1e-7;
constexpr double kDuplicatedPointsEpsilonSq =
    kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
// Margin for comparation
constexpr double kEpsilon = 0.001;
constexpr double kMathEpsilon = 1e-10;
//constexpr double kDefaultWidth = 3.75 * 0.5;
//TODO:fengwang31 temp modify , for test
constexpr double kDefaultWidth = 6.0 * 0.5;
constexpr double kDefaultLength = 10.0;
}  // namespace

LaneInfo::LaneInfo(const LaneData &lane,
                   const LaneBoundaryInfoTable &lane_boundary_table,
                   const LaneBoundaryInfoTable &road_boundary_table)
    : lane_(lane),
      lane_boundary_table_(lane_boundary_table),
      road_boundary_table_(road_boundary_table) {
  Init();
}

void LaneInfo::GetLaneCentralPoints(const LaneData &lane,
                                    std::vector<Vec2d> *points) {
  if (points == nullptr) {
    is_valid_ = false;
    return;
  }
  // assert(points != nullptr);
  points->clear();

  points->reserve(lane.points_on_central_line_size());
  for (const auto &point : lane.points_on_central_line()) {
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

void LaneInfo::Init() {
  is_valid_ = true;
  GetLaneCentralPoints(lane_, &points_);
  if (points_.size() < 2) {
    is_valid_ = false;
    return;
  }
  // assert(points_.size() >= 2U);
  segments_.clear();
  segments_.reserve(points_.size() - 1);
  accumulated_s_.clear();
  accumulated_s_.reserve(points_.size());
  unit_directions_.clear();
  unit_directions_.reserve(points_.size());
  headings_.clear();
  headings_.reserve(points_.size());

  double s = 0;
  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    segments_.emplace_back(points_[i], points_[i + 1]);
    accumulated_s_.push_back(s);
    unit_directions_.push_back(segments_.back().unit_direction());
    s += segments_.back().length();
  }
  if (segments_.empty()) {
    is_valid_ = false;
    return;
  }
  // assert(!segments_.empty());

  accumulated_s_.push_back(s);
  total_length_ = s;
  if (unit_directions_.empty()) {
    is_valid_ = false;
    return;
  }
  // assert(!unit_directions_.empty());
  unit_directions_.push_back(unit_directions_.back());
  for (const auto &direction : unit_directions_) {
    headings_.push_back(direction.Angle());
  }

  CreateKDTree();

  GetWidths();

  GetBoundarySegments();
}

void LaneInfo::GetWidths() {
  // left lane width
  left_widths_.resize(points_.size(), kDefaultWidth);
  auto it_left_lane_boundary =
      lane_boundary_table_.find(lane_.left_lane_boundary_id());
  if (it_left_lane_boundary != lane_boundary_table_.end()) {
    for (size_t i = 0; i < points_.size(); ++i) {
      left_widths_[i] = it_left_lane_boundary->second->DistanceTo(points_[i]);
    }
  }

  // right lane width
  right_widths_.resize(points_.size(), kDefaultWidth);
  auto it_right_lane_boundary =
      lane_boundary_table_.find(lane_.right_lane_boundary_id());
  if (it_right_lane_boundary != lane_boundary_table_.end()) {
    for (size_t i = 0; i < points_.size(); ++i) {
      right_widths_[i] = it_right_lane_boundary->second->DistanceTo(points_[i]);
    }
  }

  // left road width
  left_road_width_.resize(points_.size(), kDefaultWidth);
  auto it_left_road_boundary =
      road_boundary_table_.find(lane_.left_road_boundary_id());
  if (it_left_road_boundary != road_boundary_table_.end()) {
    for (size_t i = 0; i < points_.size(); ++i) {
      left_road_width_[i] =
          it_left_road_boundary->second->DistanceTo(points_[i]);
    }
  }

  // right road width
  right_road_width_.resize(points_.size(), kDefaultWidth);
  auto it_right_road_boundary =
      road_boundary_table_.find(lane_.right_road_boundary_id());
  if (it_right_road_boundary != road_boundary_table_.end()) {
    for (size_t i = 0; i < points_.size(); ++i) {
      right_road_width_[i] =
          it_right_road_boundary->second->DistanceTo(points_[i]);
    }
  }
}

void LaneInfo::GetBoundarySegments() {
  // left lane boundary segment
  auto it_left_lane_boundary =
      lane_boundary_table_.find(lane_.left_lane_boundary_id());
  if (it_left_lane_boundary != lane_boundary_table_.end()) {
    GetBoundarySegments(it_left_lane_boundary->second->lane_boundary(),
                        &left_lane_boundary_segments_);
  }

  // right lane boundary segment
  auto it_right_lane_boundary =
      lane_boundary_table_.find(lane_.right_lane_boundary_id());
  if (it_right_lane_boundary != lane_boundary_table_.end()) {
    GetBoundarySegments(it_right_lane_boundary->second->lane_boundary(),
                        &right_lane_boundary_segments_);
  }

  // left road boundary segment
  auto it_left_road_boundary =
      road_boundary_table_.find(lane_.left_road_boundary_id());
  if (it_left_road_boundary != road_boundary_table_.end()) {
    GetBoundarySegments(it_left_road_boundary->second->lane_boundary(),
                        &left_road_boundary_segments_);
  }

  // right road boundary segment
  auto it_right_road_boundary =
      road_boundary_table_.find(lane_.right_road_boundary_id());
  if (it_right_road_boundary != road_boundary_table_.end()) {
    GetBoundarySegments(it_right_road_boundary->second->lane_boundary(),
                        &right_road_boundary_segments_);
  }
}

void LaneInfo::GetBoundarySegments(
    const LaneBoundary &boundary,
    std::vector<BoundarySegment> *boundary_segments) {
  boundary_segments->clear();
  boundary_segments->reserve(boundary.boundary_attributes_size());
  for (const auto &boundary_attribute : boundary.boundary_attributes()) {
    if (boundary_attribute.points_size() < 2) {
      continue;
    }
    const Vec2d end_point(boundary_attribute.points().rbegin()->x(),
                          boundary_attribute.points().rbegin()->y());
    BoundarySegment boundary_segment;
    if (boundary_segments->empty()) {
      boundary_segment.start_s = boundary_segments->back().end_s;
    }
    Vec2d map_point;
    int index = 0;
    DistanceTo(end_point, &map_point, &boundary_segment.end_s, &index);
    boundary_segment.types.reserve(boundary_attribute.types_size());
    for (int i = 0; i < boundary_attribute.types_size(); ++i) {
      boundary_segment.types.push_back(boundary_attribute.types(i));
    }
    boundary_segments->emplace_back(std::move(boundary_segment));
  }
}

std::vector<::Map::BoundaryAttributes::Type> LaneInfo::GetLeftLaneBoundaryTypes(
    const double s) const {
  return GetBoundaryTypes(left_lane_boundary_segments_, s);
}

std::vector<::Map::BoundaryAttributes::Type>
LaneInfo::GetRightLaneBoundaryTypes(const double s) const {
  return GetBoundaryTypes(right_lane_boundary_segments_, s);
}

std::vector<::Map::BoundaryAttributes::Type> LaneInfo::GetLeftRoadBoundaryTypes(
    const double s) const {
  return GetBoundaryTypes(left_road_boundary_segments_, s);
}

std::vector<::Map::BoundaryAttributes::Type>
LaneInfo::GetRightRoadBoundaryTypes(const double s) const {
  return GetBoundaryTypes(right_road_boundary_segments_, s);
}

std::vector<::Map::BoundaryAttributes::Type> LaneInfo::GetBoundaryTypes(
    const std::vector<BoundarySegment> &boundary_segments,
    const double s) const {
  if (accumulated_s_.empty() || boundary_segments.empty()) {
    return {::Map::BoundaryAttributes::UNKNOWN};
  }

  if (s + kEpsilon < accumulated_s_.front()) {
    return boundary_segments.front().types;
  }
  if (s - kEpsilon > accumulated_s_.back()) {
    return boundary_segments.back().types;
  }

  auto iter =
      std::lower_bound(boundary_segments.begin(), boundary_segments.end(), s,
                       BoundarySegmentComparator);
  if (iter == boundary_segments.end()) {
    return boundary_segments.back().types;
  }
  int index = static_cast<int>(std::distance(boundary_segments.begin(), iter));
  return boundary_segments[index].types;
}

Vec2d LaneInfo::GetPoint(const double s) const {
  if (accumulated_s_.empty()) {
    return {0.0, 0.0};
  }

  if (s + kEpsilon < accumulated_s_.front()) {
    return points_.front();
  }
  if (s - kEpsilon > accumulated_s_.back()) {
    return points_.back();
  }

  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0 || *iter - s <= kMathEpsilon) {
    return points_[index];
  }
  const double ratio = (s - accumulated_s_[index - 1]) /
                       (accumulated_s_[index] - accumulated_s_[index - 1]);
  const double x =
      points_[index - 1].x() * (1.0 - ratio) + points_[index].x() * ratio;
  const double y =
      points_[index - 1].y() * (1.0 - ratio) + points_[index].y() * ratio;
  return {x, y};
}

double LaneInfo::GetHeading(const double s) const {
  if (accumulated_s_.empty()) {
    return 0.0;
  }

  if (s + kEpsilon < accumulated_s_.front()) {
    return headings_.front();
  }
  if (s - kEpsilon > accumulated_s_.back()) {
    return headings_.back();
  }

  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0 || *iter - s <= kMathEpsilon) {
    return headings_[index];
  }
  return ad_common::math::slerp(headings_[index - 1], accumulated_s_[index - 1],
                               headings_[index], accumulated_s_[index], s);
}

double LaneInfo::GetCurvature(const double s) const {
  if (points_.size() < 2U) {
    return 0.0;
  }

  if (s + kEpsilon < accumulated_s_.front()) {
    return 0.0;
  }
  if (s > accumulated_s_.back() + kEpsilon) {
    return 0.0;
  }

  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  if (iter == accumulated_s_.end()) {
    return 0.0;
  }
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0) {
    return 0.0;
  }
  return (headings_[index] - headings_[index - 1]) /
         (accumulated_s_[index] - accumulated_s_[index - 1] + kEpsilon);
}

void LaneInfo::GetWidth(const double s, double *left_width,
                        double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidth(left_widths_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidth(right_widths_, s);
  }
}

double LaneInfo::GetWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(s, &left_width, &right_width);
  return left_width + right_width;
}

double LaneInfo::GetEffectiveWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(s, &left_width, &right_width);
  return 2 * std::min(left_width, right_width);
}

void LaneInfo::GetRoadWidth(const double s, double *left_width,
                            double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidth(left_road_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidth(right_road_width_, s);
  }
}

double LaneInfo::GetRoadWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetRoadWidth(s, &left_width, &right_width);
  return left_width + right_width;
}

double LaneInfo::GetWidth(const std::vector<double> &samples,
                          const double s) const {
  if (accumulated_s_.empty()) {
    return 0.0;
  }

  if (s + kEpsilon < accumulated_s_.front()) {
    return samples.front();
  }
  if (s - kEpsilon > accumulated_s_.back()) {
    return samples.back();
  }

  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0 || *iter - s <= kMathEpsilon) {
    return samples[index];
  }
  return ad_common::math::lerp(samples[index - 1], accumulated_s_[index - 1],
                               samples[index], accumulated_s_[index], s);
}

bool LaneInfo::IsOnLane(const Vec2d &point) const {
  double accumulate_s = 0.0;
  double lateral = 0.0;
  if (!GetProjection(point, &accumulate_s, &lateral)) {
    return false;
  }

  if (accumulate_s > (total_length() + kDefaultLength) ||
      (accumulate_s + kEpsilon) < 0.0) {
    return false;
  }

  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(accumulate_s, &left_width, &right_width);
  if (lateral < left_width && lateral > -right_width) {
    return true;
  }
  return false;
}

bool LaneInfo::IsOnLane(const Box2d &box) const {
  std::vector<Vec2d> corners;
  box.GetAllCorners(&corners);
  for (const auto &corner : corners) {
    if (!IsOnLane(corner)) {
      return false;
    }
  }
  return true;
}

Vec2d LaneInfo::GetSmoothPoint(double s) const {
  Vec2d point;
  if (points_.size() < 2) {
    return point;
  }
  if (s <= 0.0) {
    return points_[0];
  }

  if (s >= total_length()) {
    return points_.back();
  }

  const auto low_itr =
      std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  if (low_itr == accumulated_s_.end()) {
    return point;
  }
  size_t index = low_itr - accumulated_s_.begin();
  double delta_s = *low_itr - s;
  if (delta_s < kMathEpsilon) {
    return points_[index];
  }

  auto smooth_point = points_[index] - unit_directions_[index - 1] * delta_s;

  return smooth_point;
}

double LaneInfo::DistanceTo(const Vec2d &point) const {
  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  if (segment_box == nullptr) {
    return 0.0;
  }
  return segment_box->DistanceTo(point);
}

double LaneInfo::DistanceTo(const Vec2d &point, Vec2d *map_point,
                            double *s_offset, int *s_offset_index) const {
  if (map_point == nullptr || s_offset == nullptr ||
      s_offset_index == nullptr) {
    return 0.0;
  }

  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
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

Vec2d LaneInfo::GetNearestPoint(const Vec2d &point, double *distance) const {
  Vec2d empty_point;
  if (distance == nullptr) {
    return empty_point;
  }

  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  if (segment_box == nullptr) {
    return empty_point;
  }
  int index = segment_box->id();
  Vec2d nearest_point;
  *distance = segments_[index].DistanceTo(point, &nearest_point);

  return nearest_point;
}

bool LaneInfo::GetProjection(const Vec2d &point, double *accumulate_s,
                             double *lateral) const {
  if (accumulate_s == nullptr || lateral == nullptr || segments_.empty()) {
    return false;
  }
  double min_dist = std::numeric_limits<double>::infinity();
  int seg_num = static_cast<int>(segments_.size());
  int min_index = 0;
  for (int i = 0; i < seg_num; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < min_dist) {
      min_index = i;
      min_dist = distance;
    }
  }
  min_dist = std::sqrt(min_dist);
  const auto &nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else if (min_index == seg_num - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
  }
  return true;
}

void LaneInfo::CreateKDTree() {
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
  lane_segment_kdtree_ = std::make_unique<LaneSegmentKDTree>(segment_box_list_, params);
}

}  // namespace hdmap
}  // namespace ad_common
