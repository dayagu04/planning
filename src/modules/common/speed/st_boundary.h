#ifndef MODULES_PLANNING_OPTIMIZERS_STBOUNDARY_H_
#define MODULES_PLANNING_OPTIMIZERS_STBOUNDARY_H_

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "math/math_utils.h"
#include "speed/st_point.h"
#include "src/modules/common/math/box2d.h"
#include "src/modules/common/math/line_segment2d.h"
#include "src/common/vec2d.h"

namespace planning {

class STBoundary {
 public:
  STBoundary() = default;

  explicit STBoundary(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs);

  explicit STBoundary(const planning_math::Box2d& box) = delete;

  explicit STBoundary(std::vector<planning_math::Vec2d> points) = delete;

  static STBoundary CreateInstance(const std::vector<STPoint>& lower_points,
                                   const std::vector<STPoint>& upper_points);

  static std::unique_ptr<STBoundary> CreateInstance(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs);

  ~STBoundary() = default;

  bool IsEmpty() const { return lower_points_.empty(); }

  bool IsPointInBoundary(const STPoint& st_point) const;

  bool GetTimestampBeginWithS(double lower_s, double& t) const;

  STPoint upper_left_point() const;
  STPoint upper_right_point() const;

  STPoint bottom_left_point() const;
  STPoint bottom_right_point() const;

  void set_upper_left_point(STPoint st_point);
  void set_upper_right_point(STPoint st_point);
  void set_bottom_left_point(STPoint st_point);
  void set_bottom_right_point(STPoint st_point);

  STBoundary ExpandByS(const double s) const;
  STBoundary ExpandByT(const double t) const;
  STBoundary ShiftByS(const double s) const;

  STBoundary ShrinkByDs(const double ds, const double begin_t) const;

  size_t id() const;

  double characteristic_length() const;

  void set_id(const size_t id);

  void SetCharacteristicLength(const double characteristic_length);

  double min_s() const;
  double min_t() const;
  double max_s() const;
  double max_t() const;

  const std::vector<STPoint>& upper_points() const { return upper_points_; }
  const std::vector<STPoint>& lower_points() const { return lower_points_; }

 private:
  bool IsValid(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;

  bool IsPointNear(const planning_math::LineSegment2d& seg,
                   const planning_math::Vec2d& point, const double max_dist);

  bool GetIndexRange(const std::vector<STPoint>& points, const double t,
                     size_t* left, size_t* right) const;

 private:
  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;

  size_t id_;
  double characteristic_length_ = 1.0;

  // TODO: fill it
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  // TODO: fill it
  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;
};

}  // namespace planning

#endif /* MODULES_PLANNING_OPTIMIZERS_STBOUNDARY_H_ */