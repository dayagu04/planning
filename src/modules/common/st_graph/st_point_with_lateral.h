#pragma once

#include "st_point.h"
#include "agent/agent.h"

namespace planning {
namespace speed {

class STPointWithLateral : public STPoint {
 public:
  STPointWithLateral() = default;
  STPointWithLateral(const double s, const double t);
  STPointWithLateral(const double s, const double t, const int32_t agent_id,
                     const int64_t boundary_id, const double velocity,
                     const double acceleration);
  STPointWithLateral(const double s, const double t, const int32_t agent_id,
                     const int64_t boundary_id, const double velocity,
                     const double acceleration, const double vehicle_length,
                     const double l);

  ~STPointWithLateral() = default;

  agent::AgentType type() const { return type_; }
  void set_type(agent::AgentType type) { type_ = type; }

  double l() const { return l_; }
  void set_l(const double l) { l_ = l; }

  double vehicle_length() const { return vehicle_length_; }
  void set_vehicle_length(const double vehicle_length) {
    vehicle_length_ = vehicle_length;
  }

 private:
  double l_ = 0.0;
  double vehicle_length_ = 0.0;
  agent::AgentType type_;
};

}  // namespace speed
}  // namespace planning
