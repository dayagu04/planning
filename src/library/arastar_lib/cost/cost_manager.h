#pragma once

#include "base_cost.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace planning {
namespace ara_star {

class CostManager {
public:
  CostManager() = default;
  ~CostManager() = default;

  void AddCost(const std::shared_ptr<BaseCost> cost_ptr);

  double ComputeCost(Node3D& vertex) const;

private:
  std::vector<std::shared_ptr<BaseCost>> cost_ptrs_;
};

} // namespace ara_star
} // namespace planning