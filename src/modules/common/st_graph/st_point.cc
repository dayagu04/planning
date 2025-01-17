#include "st_point.h"

#include <limits>

namespace planning {
namespace speed {

STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const double s, const double t, const int32_t agent_id,
                 const int64_t boundary_id, const double velocity,
                 const double acceleration)
    : Vec2d(t, s),
      agent_id_(agent_id),
      boundary_id_(boundary_id),
      velocity_(velocity),
      acceleration_(acceleration) {}

STPoint::STPoint(const double s, const double t, const int32_t agent_id,
                 const int64_t boundary_id, const double velocity,
                 const double acceleration, const double extreme_l)
    : STPoint(s, t, agent_id, boundary_id, velocity, acceleration) {
  extreme_l_ = extreme_l;
}

STPoint::STPoint(const planning_math::Vec2d& vec2d_point)
    : Vec2d(vec2d_point) {}

STPoint STPoint::HighestSTPoint() {
  STPoint pt;
  pt.set_agent_id(kNoAgentId);
  pt.set_s(std::numeric_limits<double>::max());
  return pt;
}

STPoint STPoint::LowestSTPoint() {
  STPoint pt;
  pt.set_agent_id(kNoAgentId);
  pt.set_s(std::numeric_limits<double>::lowest());
  return pt;
}

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { y_ = s; }

void STPoint::set_t(const double t) { x_ = t; }

int32_t STPoint::agent_id() const { return agent_id_; }

void STPoint::set_agent_id(const int32_t agent_id) { agent_id_ = agent_id; }

int64_t STPoint::boundary_id() const { return boundary_id_; }

void STPoint::set_boundary_id(const int64_t boundary_id) {
  boundary_id_ = boundary_id;
}

double STPoint::velocity() const { return velocity_; }

void STPoint::set_velocity(const double velocity) { velocity_ = velocity; }

double STPoint::acceleration() const { return acceleration_; }

void STPoint::set_acceleration(const double acceleration) {
  acceleration_ = acceleration;
}

bool STPoint::valid() const { return valid_; }

void STPoint::set_valid(const bool valid) { valid_ = valid; }

double STPoint::extreme_l() const { return extreme_l_; }

void STPoint::set_extreme_l(const double extreme_l) { extreme_l_ = extreme_l; }

void STPoint::set_info(const double s, const double t, const double velocity,
                       const int32_t agent_id, const int64_t boundary_id) {
  y_ = s;
  x_ = t;
  velocity_ = velocity;
  agent_id_ = agent_id;
  boundary_id_ = boundary_id;
}
}  // namespace speed
}  // namespace planning