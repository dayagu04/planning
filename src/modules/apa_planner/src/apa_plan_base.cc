#include "apa_plan_base.h"

#include <memory>

#include "lateral_path_optimizer.h"

namespace planning {
namespace apa_planner {

void ApaPlannerBase::Init(const bool c_ilqr_enable) {
  lateral_path_optimizer_ptr_ = std::make_shared<LateralPathOptimizer>();
  lateral_path_optimizer_ptr_->Init(c_ilqr_enable);
}

}  // namespace apa_planner
}  // namespace planning
