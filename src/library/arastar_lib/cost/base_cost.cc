#include "base_cost.h"

namespace planning {
namespace ara_star {

BaseCost::BaseCost(const CostType type, const double weight) : type_(type), weight_(weight){};

} // namespace ara_star

} // namespace cp_planning
