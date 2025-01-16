#pragma once
#include "sample_poly_const.h"
namespace planning {

class CurveCost {
 public:
  CurveCost() = default;
  explicit CurveCost(const double weight) : weight_(weight){};
  virtual ~CurveCost() = default;
  void SetWeight(const double weight) { weight_ = weight; };
  const double cost() const { return cost_; };

 protected:
  double cost_ = 0.0;
  double weight_ = 0.0;
};
}  // namespace planning
