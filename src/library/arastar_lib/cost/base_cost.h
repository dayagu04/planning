#pragma once
#include <iostream>
#include <string>
#include "src/library/arastar_lib/node3d.h"

namespace planning {
namespace ara_star {

class BaseCost {
public:
  enum class CostType { UNKNOWN = 0, AGENT = 1, CENTER, BOUNDARY, MOTION };
  BaseCost(const CostType type, const double weight);
  virtual ~BaseCost() = default;
  virtual double MakeCost(Node3D& vertex) const = 0;
  CostType GetCostType() const {
    return type_;
  }
  double GetCostWeight() const {
    return weight_;
  }

private:
  CostType type_;
  double weight_;
};

} // namespace ara_star
} // namespace planning
