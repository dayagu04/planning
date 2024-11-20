#pragma once

#include <cstdint>
#include <string>

#include "vec2d.h"

namespace planning {
namespace speed {

constexpr int32_t kNoAgentId = -1;

class STPoint : public planning_math::Vec2d {
 public:
  STPoint() = default;
  STPoint(const double s, const double t);
  STPoint(const double s, const double t, const int32_t agent_id,
          const int64_t boundary_id, const double velocity,
          const double acceleration);
  STPoint(const double s, const double t, const int32_t agent_id,
          const int64_t boundary_id, const double velocity,
          const double acceleration, const double extreme_l);
  explicit STPoint(const planning_math::Vec2d& point);

  static STPoint HighestSTPoint();
  static STPoint LowestSTPoint();

  ~STPoint() = default;

  double x() const = delete;
  double y() const = delete;

  double s() const;
  void set_s(const double s);

  double t() const;
  void set_t(const double t);

  int32_t agent_id() const;
  void set_agent_id(const int32_t agent_id);

  int64_t boundary_id() const;
  void set_boundary_id(const int64_t boundary_id);

  double velocity() const;
  void set_velocity(const double velocity);

  double acceleration() const;
  void set_acceleration(const double acceleration);

  bool valid() const;
  void set_valid(const bool valid);

  double extreme_l() const;
  void set_extreme_l(const double extreme_l);

 private:
  int32_t agent_id_ = kNoAgentId;
  int64_t boundary_id_ = kNoAgentId;
  double velocity_ = 0.0;
  double acceleration_ = 0.0;
  bool valid_ = true;

  // save min_l in LowerPoints and max_l in UpperPoints
  double extreme_l_ = 0.0;
};

}  // namespace speed
}  // namespace planning