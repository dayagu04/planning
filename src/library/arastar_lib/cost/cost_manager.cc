#include "cost_manager.h"

namespace planning {
namespace ara_star {
namespace {
constexpr double kEpsilon = 1.0e-6;
} // namespace

void CostManager::AddCost(const std::shared_ptr<BaseCost> cost_ptr) {
  if (cost_ptr->GetCostWeight() > kEpsilon) {
    cost_ptrs_.push_back(cost_ptr);
  }
}

double CostManager::ComputeCost(Node3D& vertex) const {
  double total_cost = 0.0;
  for (auto cost_ptr : cost_ptrs_){
    double cost = cost_ptr->MakeCost(vertex);
    total_cost += cost_ptr->GetCostWeight()*cost;
  }
  return total_cost;
}

} // namespace ara_star
} // namespace planning