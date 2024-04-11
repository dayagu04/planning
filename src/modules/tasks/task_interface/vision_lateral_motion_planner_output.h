#pragma once
#include <array>
namespace planning {

struct VisionLateralMotionPlannerOutput {
  std::array<double, 4> c_poly;
  std::array<double, 4> d_poly;
  double lat_offset;
};

}  // namespace planning