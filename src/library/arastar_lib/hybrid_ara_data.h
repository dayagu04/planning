#pragma once
#include <cstdint>
#include <vector>

#include "src/modules/common/math/box2d.h"
namespace planning {

namespace ara_star {

struct HybridARAStarResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> s;
  std::vector<double> l;

  void Clear() {
    x.clear();
    y.clear();
    phi.clear();
    s.clear();
    l.clear();
  }

  bool Valid() const {
    if (x.empty() || y.empty() || phi.empty() || s.empty() || l.empty()) {
      return false;
    }
    if (!(x.size() == y.size() && y.size() == phi.size() &&
          phi.size() == s.size() && s.size() == l.size())) {
      return false;
    }
    if (x.size() < 3) {
      return false;
    }
    return true;
  }
};

struct SLBox2d {
  bool nudge_less = false;
  double max_s = -1000.0;
  double max_l = -1000.0;
  double min_s = 1000.0;
  double min_l = 1000.0;
  double s = 0.0;
  planning::planning_math::Box2d box;
  uint16_t id;
};

}  // namespace ara_star

}  // namespace planning
