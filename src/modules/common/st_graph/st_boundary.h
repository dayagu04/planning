#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "math/box2d.h"
#include "math/math_utils.h"
#include "math/polygon2d.h"
#include "math/math_utils.h"
#include "st_point.h"

namespace planning {
namespace speed {

enum class StBoundaryType {
  NORMAL = 0,
  NEIGHBOR,
  EXPAND,
};

class STBoundary : public planning_math::Polygon2d {
public:
  STBoundary() = default;

  explicit STBoundary(const std::vector<std::pair<STPoint, STPoint>>& st_point_pairs);

  explicit STBoundary(const planning_math::Box2d& box) = delete;

  explicit STBoundary(std::vector<planning_math::Vec2d> points) = delete;

  static STBoundary CreateInstance(const std::vector<STPoint>& lower_points,
                                   const std::vector<STPoint>& upper_points);

  ~STBoundary() = default;

  bool IsEmpty() const;

  bool GetBoundarySRange(const double curr_time, double* const s_lower,
                         double* const s_upper) const;

  bool GetBoundaryBounds(const double curr_time, STPoint* const lower_point,
                         STPoint* const upper_point) const;

  enum class DecisionType {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
    NEIGHBOR_YIELD,
    NEIGHBOR_OVERTAKE,
    CAUTION_YIELD,
  };

  DecisionType decision_type() const;

  int64_t id() const;

  void set_id(const int64_t id);

  void set_decision_type(const DecisionType& boundary_type);

  void SetCharacteristicLength(const double characteristic_length);

  double min_s() const;

  double min_t() const;

  double max_s() const;

  double max_t() const;

  const std::vector<STPoint>& upper_points() const;

  const std::vector<STPoint>& lower_points() const;

  void clear();

  bool IsPointInBoundary(const STPoint& st_point) const;

  STBoundary ExpandByDs(const double ds) const;

  STBoundary ExpandByDt(const double dt) const;

  bool CutOffByTimeRange(const double start_time, const double end_time);

  const STPoint& upper_left_point() const;

  const STPoint& upper_right_point() const;

  const STPoint& bottom_left_point() const;

  const STPoint& bottom_right_point() const;

  const std::vector<std::pair<double, double>>& invalid_time_sections() const;

  void set_upper_left_point(const STPoint& st_point);

  void set_upper_right_point(const STPoint& st_point);

  void set_bottom_left_point(const STPoint& st_point);

  void set_bottom_right_point(const STPoint& st_point);

  void InsertInvalidTimeSection(const std::pair<double, double>& invalid_time_section);

  double max_delta_s() const;

private:
  bool IsValid(const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;

  bool IsPointNearToLinesegment(const planning_math::LineSegment2d& seg,
                                const planning_math::Vec2d& point, const double max_dist);

  bool GetIndexRange(const double t, size_t* const left, size_t* const right) const;

  bool IsTimeInvalid(const double t) const;

private:
  DecisionType decision_type_ = DecisionType::UNKNOWN;

  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;

  int64_t id_;
  double characteristic_length_ = 1.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;

  std::vector<std::pair<double, double>> invalid_time_sections_;

  double max_delta_s_ = std::numeric_limits<double>::lowest();
};

} // namespace speed
} // namespace planning