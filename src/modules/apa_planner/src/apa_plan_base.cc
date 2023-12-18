#include "apa_plan_base.h"

namespace planning {
namespace apa_planner {

void ApaPlannerBase::Init() {
  collision_detector_ptr_ = std::make_shared<CollisionDetector>();
}

}  // namespace apa_planner
}  // namespace planning
