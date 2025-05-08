  #pragma once
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "config/message_type.h"
#include "ego_planning_config.h"
namespace planning {
  struct StaticObstacleInfo{
    int32 id =  0;
    double s_start;
    double s_end;
    double l_start;
    double l_end;
  };
  struct EgoInfo{
    double ego_s;
    double ego_l;
    double ego_v;
    double v_cruise;
  };
class ComparableCost {
 public:
  ComparableCost() = default;
  ComparableCost(const bool has_collision, const bool out_boundary,
                 const double safety_cost, const double smooth_cost, const double stitch_cost)
      : has_collision_(has_collision),
        out_boundary_(out_boundary),
        safety_cost_(safety_cost),
        smooth_cost_(smooth_cost),
        stitch_cost_ (stitch_cost){}
  ComparableCost(const ComparableCost &) = default;

  int CompareTo(const ComparableCost &another) const {
    double kEpsilion = 1e-12;
    if ((has_collision_ || out_boundary_) &&
        !(another.has_collision_ || another.out_boundary_)) {
      return 1;
    } else if (!(has_collision_ || out_boundary_) &&
               (another.has_collision_ || another.out_boundary_)) {
      return -1;
    } else if (std::fabs(safety_cost_ + smooth_cost_ + stitch_cost_ - another.safety_cost_ -
                             another.smooth_cost_ - another.stitch_cost_) <
                         kEpsilion) {
      return 0;
    } else if (safety_cost_ + smooth_cost_ + stitch_cost_>
               another.safety_cost_ + another.smooth_cost_ + another.stitch_cost_) {
      return 1;
    } else {
      return -1;
    }
  }

  bool has_collision_ = false;
  bool out_boundary_ = false;
  double safety_cost_ = 0.0;
  double smooth_cost_ = 0.0;
  double stitch_cost_ = 0.0;
  ComparableCost &operator+(const ComparableCost &another) {
    has_collision_ = has_collision_ || another.has_collision_;
    out_boundary_ = out_boundary_ || another.out_boundary_;
    safety_cost_ = safety_cost_ + another.safety_cost_;
    smooth_cost_ = smooth_cost_ + another.smooth_cost_;
    stitch_cost_ = stitch_cost_ + another.stitch_cost_;
    return *this;
  }
  ComparableCost &operator+=(const ComparableCost &another) {
    has_collision_ = has_collision_ || another.has_collision_;
    out_boundary_ = out_boundary_ || another.out_boundary_;
    safety_cost_ = safety_cost_ + another.safety_cost_;
    smooth_cost_ = smooth_cost_ + another.smooth_cost_;
    stitch_cost_ = stitch_cost_ + another.stitch_cost_;
    return *this;
  }
  bool operator>(const ComparableCost &another) const {
    return this->CompareTo(another) > 0;
  }
  bool operator>=(const ComparableCost &another) const {
    return this->CompareTo(another) >= 0;
  }
  bool operator<(const ComparableCost &another) const {
    return this->CompareTo(another) < 0;
  }
  bool operator<=(const ComparableCost &another) const {
    return this->CompareTo(another) <= 0;
  }
};


}