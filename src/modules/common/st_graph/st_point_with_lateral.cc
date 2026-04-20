#include "st_point_with_lateral.h"

namespace planning {
namespace speed {

STPointWithLateral::STPointWithLateral(const double s, const double t)
    : STPoint(s, t) {}

STPointWithLateral::STPointWithLateral(const double s, const double t,
                                       const int32_t agent_id,
                                       const int64_t boundary_id,
                                       const double velocity,
                                       const double acceleration)
    : STPoint(s, t, agent_id, boundary_id, velocity, acceleration) {}

STPointWithLateral::STPointWithLateral(const double s, const double t,
                                       const int32_t agent_id,
                                       const int64_t boundary_id,
                                       const double velocity,
                                       const double acceleration,
                                       const double vehicle_length,
                                       const double l)
    : STPoint(s, t, agent_id, boundary_id, velocity, acceleration),
      vehicle_length_(vehicle_length),
      l_(l) {}

}  // namespace speed
}  // namespace planning
