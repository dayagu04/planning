#include "apa_plan_base.h"

#include <memory>

#include "lateral_path_optimizer.h"

namespace planning {
namespace apa_planner {

void ApaPlannerBase::Init() {
  lateral_path_optimizer_ptr_ = std::make_shared<LateralPathOptimizer>();
  lateral_path_optimizer_ptr_->Init();
}

}  // namespace apa_planner
}  // namespace planning
