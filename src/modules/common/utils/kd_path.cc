
#include "kd_path.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

#include "define/geometry.h"
#include "utils/path_point.h"

namespace planning {
namespace planning_math {

// KDPath::KDPath(std::vector<double>& x_vec, std::vector<double>& y_vec)
//     : x_vec_(x_vec), y_vec_(y_vec) {
//   const bool need_reset_s = true;
//   bool res = InitData(need_reset_s);
//   if (!res) {
//     // std::cout << "Path::Path() Init data fail"
//     //           << "\n";
//     throw res;
//   }
// };

KDPath::KDPath(std::vector<PathPoint>&& path_points)
    : path_points_(std::move(path_points)) {
  const bool need_reset_s = true;
  bool res = InitData(need_reset_s);
  if (!res) {
    throw res;
  }
}

KDPath::KDPath(std::vector<PathPoint>&& path_points, const bool need_reset_s)
    : path_points_(std::move(path_points)) {
  bool res = InitData(need_reset_s);
  if (!res) {
    throw res;
  }
}

KDPath::KDPath(std::vector<PathPoint>&& path_points, const bool need_reset_s,
               const double start_ref)
    : path_points_(std::move(path_points)) {
  bool res = InitData(need_reset_s, true, start_ref);
  if (!res) {
    // std::cout << "Path::Path() Init data fail"
    //           << "\n";
    throw res;
  }
}

bool KDPath::InitData(const bool need_reset_s, bool head_mono,
                      double start_ref) {
  if (path_points_.size() < 2) {
    return false;
  }
  if (need_reset_s) {
    KDPath::ResetCurveCalculate(&path_points_);
  }
  length_ = path_points_.back().s() - path_points_.front().s();
  min_x_ = std::numeric_limits<double>::max();
  min_y_ = std::numeric_limits<double>::max();
  max_x_ = -std::numeric_limits<double>::max();
  max_y_ = -std::numeric_limits<double>::max();
  line_segments_.reserve(path_points_.size() - 1);
  for (int i = 0; i + 1 < path_points_.size(); ++i) {
    LineSegment2d line(path_points_.at(i), path_points_.at(i + 1));
    min_x_ = std::fmin(min_x_, line.min_x());
    min_y_ = std::fmin(min_y_, line.min_y());
    max_x_ = std::fmax(max_x_, line.max_x());
    max_y_ = std::fmax(max_y_, line.max_y());
    line_segments_.emplace_back(std::move(line));
  }
  // set linesegment heading monotonic
  if (head_mono) {
    MonoHeading(start_ref);
  }
  BuildKDTree();
  return true;
}

double KDPath::Length() const { return length_; }

bool KDPath::KdtreeValid() const { return kdtree_valid_; }

const std::vector<PathPoint>& KDPath::path_points() const {
  return path_points_;
}

bool KDPath::BuildKDTree() {
  if (path_points_.size() < 2) {
    return false;
  }

  AABoxKDTreeParams kdtree_params;
  kdtree_params.max_depth = 2;
  kdtree_params.max_leaf_dimension = 25;
  kdtree_params.max_leaf_size = 20;

  objects_.reserve(line_segments_.size());
  for (int i = 0; i < line_segments_.size(); i++) {
    objects_.emplace_back(line_segments_[i], i);
  }
  kd_tree_ =
      std::make_unique<AABoxKDTree2d<GeometryObject>>(objects_, kdtree_params);
  kdtree_valid_ = true;

  return true;
}

PathPoint KDPath::GetPathPointByS(const double path_s) const {
  auto it_lower = QueryLowerBound(path_points_, path_s);
  if (it_lower == path_points_.begin()) {
    it_lower += 1;
  }
  if (it_lower == path_points_.end()) {
    it_lower -= 1;
  }
  PathPoint point =
      (it_lower - 1)->GetInterpolateByLinearApproximation(*(it_lower), path_s);
  return point;
}

std::vector<PathPoint>::const_iterator KDPath::QueryLowerBound(
    const std::vector<PathPoint>& path_points, const double path_s) const {
  auto func = [](const PathPoint& tp, const double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(path_points.begin(), path_points.end(), path_s, func);
}

double KDPath::DistanceTo(const Vec2d& point) const {
  double distance = std::numeric_limits<double>::max();
  const auto* nearest_object = kd_tree_->GetNearestObject(point);
  if (nearest_object == nullptr) {
    return distance;
  }
  const auto& line_object = nearest_object->line_segment();
  return line_object->DistanceTo(point);
}

double KDPath::DistanceSquareTo(const Vec2d& point) const {
  double distance = std::numeric_limits<double>::max();
  const auto* nearest_object = kd_tree_->GetNearestObject(point);
  if (nearest_object == nullptr) {
    return distance;
  }
  const auto& line_object = nearest_object->line_segment();
  return line_object->DistanceSquareTo(point);
}

bool KDPath::IsPointIn(const Vec2d& point) const {
  const auto* nearest_object = kd_tree_->GetNearestObject(point);
  if (nearest_object == nullptr) {
    return false;
  }
  const auto& line_object = nearest_object->line_segment();
  return line_object->IsPointIn(point);
}

bool KDPath::XYToSL(const double x, const double y, double* const s,
                    double* const l) const {
  if (path_points_.empty()) {
    return false;
  }

  const auto* nearest_object = kd_tree_->GetNearestObject({x, y});
  if (nearest_object == nullptr) {
    return false;
  }

  const auto& line_object = nearest_object->line_segment();
  double base_s = path_points_.at(nearest_object->index()).s();
  Project(*line_object, x, y, base_s, s, l);
  return true;
}

bool KDPath::SLToXY(const double s, const double l, double* const x,
                    double* const y) const {
  if (path_points_.empty()) {
    return false;
  }

  const auto point = GetPathPointByS(s);
  const double normal_heading = point.theta() + M_PI / 2;
  // auto it = std::lower_bound(path_points_.begin(), path_points_.end(), s,
  //                            [](const PathPoint& point, double s) { return
  //                            point.s() < s; });

  // PathPoint p1;
  // PathPoint p2;
  // if (it == path_points_.end()) {
  //   p1 = *(std::prev(it, 2));
  //   p2 = *(std::prev(it, 1));
  // } else if (it == path_points_.begin()) {
  //   p1 = *it;
  //   p2 = *(std::next(it));
  // } else {
  //   p1 = *(std::prev(it));
  //   p2 = *it;
  // }

  // double segment_ratio = (s - p1.s()) / (p2.s() - p1.s());
  // double dx = p2.x() - p1.x();
  // double dy = p2.y() - p1.y();

  // double heading = std::atan2(dy, dx);
  // *x = p1.x() + std::cos(heading) * segment_ratio * std::hypotf(dx, dy);
  // *y = p1.y() + std::sin(heading) * segment_ratio * std::hypotf(dx, dy);

  // double normal_heading = heading + M_PI_2;
  *x = point.x() + l * std::cos(normal_heading);
  *y = point.y() + l * std::sin(normal_heading);

  return true;
}

bool KDPath::XYToSL(const Point2D& cart_point, Point2D& frenet_point) {
  if (path_points_.empty()) {
    LOG_DEBUG("path_points_ is empty");
    return false;
  }

  const auto* nearest_object =
      kd_tree_->GetNearestObject({cart_point.x, cart_point.y});
  if (nearest_object == nullptr) {
    LOG_DEBUG("nearest_object is nullptr \n");
    return false;
  }

  const auto& line_object = nearest_object->line_segment();
  double base_s = path_points_.at(nearest_object->index()).s();
  Project(*line_object, cart_point.x, cart_point.y, base_s, &frenet_point.x,
          &frenet_point.y);

  if (frenet_point.x < 0 || frenet_point.x > length_) {
    LOG_DEBUG("s is not within the valid range \n");
    return false;
  }

  return true;
}

KDPathStatus KDPath::XYPointToSLPoint(const Point2D& cart_point,
                                      Point2D& frenet_point) {
  if (path_points_.empty()) {
    LOG_DEBUG("path_points_ is empty \n");
    return ERROR;
  }

  const auto* nearest_object =
      kd_tree_->GetNearestObject({cart_point.x, cart_point.y});
  if (nearest_object == nullptr) {
    LOG_DEBUG("nearest_object is nullptr \n");
    return ERROR;
  }

  const auto& line_object = nearest_object->line_segment();
  double base_s = path_points_.at(nearest_object->index()).s();
  Project(*line_object, cart_point.x, cart_point.y, base_s, &frenet_point.x,
          &frenet_point.y);
  if (frenet_point.x < 0) {
    return FALL;
  } else if (frenet_point.x > length_) {
    return EXCEED;
  }

  return NORMAL;
}
bool KDPath::SLToXY(const Point2D& frenet_point, Point2D& cart_point) {
  if (path_points_.empty()) {
    LOG_DEBUG("path_points_ is empty");
    return false;
  }

  if (frenet_point.x < 0 || frenet_point.x > length_) {
    LOG_DEBUG("s is not within the valid range");
    return false;
  }

  const auto point = GetPathPointByS(frenet_point.x);
  const double normal_heading = point.theta() + M_PI / 2;
  cart_point.x = point.x() + frenet_point.y * std::cos(normal_heading);
  cart_point.y = point.y() + frenet_point.y * std::sin(normal_heading);

  return true;
}

bool KDPath::GetKappaByS(const double s, double* const kappa) const {
  if (path_points_.empty()) {
    return false;
  }

  const auto point = GetPathPointByS(s);
  *kappa = point.kappa();
  return true;
}

double KDPath::GetPathCurveHeading(double s) const {
  // assert(s <= length_ && s >= 0);
  if (s >= length_) {
    LOG_DEBUG(
        "s is not within the valid range, theta is taken from the last point");
  } else if (s < 0) {
    LOG_DEBUG(
        "s is not within the valid range, theta is taken from the first point");
  }
  return GetPathPointByS(s).theta();
}

const LineSegment2d* KDPath::GetNearestLineSegmentByXY(const double x,
                                                       const double y) const {
  const auto* nearest_object = kd_tree_->GetNearestObject({x, y});
  if (nearest_object == nullptr) {
    return nullptr;
  }

  return nearest_object->line_segment();
}

const GeometryObject* KDPath::GetNearestObjectByXY(const double x,
                                                   const double y) const {
  return kd_tree_->GetNearestObject({x, y});
}

bool KDPath::NeighborXYToSL(const double x, const double y, const double theta,
                            const double radius, const double theta_tol,
                            double* const s, double* const l) const {
  return true;
}

void KDPath::Project(const LineSegment2d& line, const double x, const double y,
                     const double base_s, double* const s,
                     double* const l) const {
  double local_s = line.ProjectOntoUnit({x, y});
  double local_l = line.ProductOntoUnit({x, y});
  *s = base_s + local_s;
  *l = local_l;
  return;
}

const std::vector<const LineSegment2d*> KDPath::GetLineSegments(
    const Vec2d& point, const double radius) const {
  const auto objects = kd_tree_->GetObjects(point, radius);
  std::vector<const LineSegment2d*> res;
  res.reserve(objects.size());
  for (const auto* object : objects) {
    if (object == nullptr) {
      continue;
    }
    res.emplace_back(object->line_segment());
  }
  return res;
}

const LineSegment2d* KDPath::GetNearestLineSegment(const Vec2d& point) const {
  const auto* nearest_object = kd_tree_->GetNearestObject(point);
  if (nearest_object == nullptr) {
    return nullptr;
  }
  return nearest_object->line_segment();
}

inline double KDPath::PerpendicularDistance(const PathPoint& line_pt_start,
                                            const PathPoint& line_pt_end,
                                            const PathPoint& cur_pt) {
  double a = line_pt_start.y() - line_pt_end.y();
  double b = line_pt_end.x() - line_pt_start.x();
  double c =
      line_pt_start.x() * line_pt_end.y() - line_pt_start.y() * line_pt_end.x();

  double denominator = std::sqrt(a * a + b * b);
  double perpendicular_distance = 0.0;

  if (denominator < KD_EPSILON) {
    perpendicular_distance = KD_MAX;
  } else {
    perpendicular_distance =
        std::abs((a * cur_pt.x() + b * cur_pt.y() + c) / denominator);
  }

  return perpendicular_distance;
}

void KDPath::DouglasPeuckerRecursion(const std::vector<PathPoint>& path_points,
                                     const int first_pt_index,
                                     const int last_pt_index,
                                     const double max_tolerance,
                                     std::vector<bool>* index_flags) {
  if (index_flags == nullptr) {
    return;
  }
  double max_distance = 0.0;
  int farthest_pt_index = 0;

  const auto& point1 = path_points.at(first_pt_index);
  const auto& point2 = path_points.at(last_pt_index);

  for (int32_t index = first_pt_index + 1; index < last_pt_index; ++index) {
    double distance =
        PerpendicularDistance(point1, point2, path_points.at(index));
    if (distance > max_distance) {
      max_distance = distance;
      farthest_pt_index = index;
    }
  }

  if (max_distance > max_tolerance && farthest_pt_index >= 0) {
    index_flags->at(farthest_pt_index) = true;
    DouglasPeuckerRecursion(path_points, first_pt_index, farthest_pt_index,
                            max_tolerance, index_flags);
    DouglasPeuckerRecursion(path_points, farthest_pt_index, last_pt_index,
                            max_tolerance, index_flags);
  }
}

std::vector<PathPoint> KDPath::DouglasPeuckerCondense(
    const std::vector<PathPoint>& path_points, const double max_tolerance,
    const double skip_dist) {
  std::vector<PathPoint> path_points_condense{};
  std::vector<bool> index_flags;
  if (path_points.empty() || max_tolerance < KD_EPSILON) {
    return path_points_condense;
  }

  int total_path_points_num = static_cast<int32_t>(path_points.size());
  index_flags.resize(total_path_points_num, false);
  // skip some distance start
  int first_pt_idx = 0;
  double s = 0.0;
  for (int i = 0; i < total_path_points_num; i++) {
    index_flags[i] = true;
    first_pt_idx = i;
    int pre = std::max(0, i - 1);
    double dist = std::hypot(path_points[i].x() - path_points[pre].x(),
                             path_points[i].y() - path_points[pre].y());
    s += dist;
    if (s >= skip_dist) {
      break;
    }
  }

  DouglasPeuckerRecursion(path_points, first_pt_idx, total_path_points_num - 1,
                          max_tolerance, &index_flags);

  path_points_condense.push_back(path_points.front());
  for (int32_t index = 1; index + 1 < total_path_points_num; ++index) {
    if (index_flags[index]) {
      path_points_condense.push_back(path_points.at(index));
    }
  }
  path_points_condense.push_back(path_points.back());

  return path_points_condense;
}

bool KDPath::CartStateToFrenetState(const CartesianState& cart_state,
                                    FrenetState& frenet_state) {
  Point2D cart_coord;
  cart_coord.x = cart_state.x;
  cart_coord.y = cart_state.y;
  Point2D frenet_coord;
  if (!XYToSL(cart_coord, frenet_coord)) {
    return false;
  }
  frenet_state.s = frenet_coord.x;
  frenet_state.r = frenet_coord.y;

  const auto point = GetPathPointByS(frenet_state.s);

  double ref_curve = point.kappa();
  double yaw_diff = cart_state.yaw - ref_curve;

  double dr_ds = (1 - ref_curve * frenet_state.r) * tan(yaw_diff);
  frenet_state.dr_ds = dr_ds;
  frenet_state.dr = cart_state.speed * sin(yaw_diff);

  double dcurve = point.dkappa();
  double term_a = (dcurve * frenet_state.r + ref_curve * dr_ds);
  double term_b =
      (cart_state.curvature * (1 - ref_curve * frenet_state.r) / cos(yaw_diff) -
       ref_curve);
  double term_c = (1 - ref_curve * frenet_state.r);
  double ddr_dsds = -term_a * tan(yaw_diff) +
                    term_b * term_c / (cos(yaw_diff) * cos(yaw_diff));
  frenet_state.ddr_dsds = ddr_dsds;
  frenet_state.ds =
      cart_state.speed * cos(yaw_diff) / (1 - ref_curve * frenet_state.r);
  frenet_state.dds =
      (cart_state.acceleration -
       pow(frenet_state.ds, 2) * (term_b * term_c * tan(yaw_diff) - term_a) /
           cos(yaw_diff)) /
      (term_c / cos(yaw_diff));
  // Get ddr
  frenet_state.ddr =
      frenet_state.dds * dr_ds + pow(frenet_state.ds, 2) * ddr_dsds;
  return true;
}

}  // namespace planning_math
}  // namespace planning
