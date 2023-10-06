#ifndef __DUBINS_LIB_H__
#define __DUBINS_LIB_H__

#include <sys/types.h>

#include <array>
#include <cstdint>
#include <utility>

#include "Eigen/Core"
#include "geometry_math.h"

namespace pnc {

namespace dubins_lib {
class DubinsLibrary {
 public:
  enum DubinsType {
    L_S_R,
    R_S_L,
    L_S_L,
    R_S_R,
    DUBINS_TYPE_COUNT,
  };

  enum CaseType {
    CASE_A,
    CASE_B,
    CASE_COUNT,
  };
  struct Input {
    Eigen::Vector2d p1 = Eigen::Vector2d::Zero();
    Eigen::Vector2d p2 = Eigen::Vector2d::Zero();
    double heading1 = 0.0;
    double heading2 = 0.0;
    double radius = 0.0;
  };

  struct Output {
    geometry_lib::Arc arc_AB;
    geometry_lib::LineSegment line_BC;
    geometry_lib::Arc arc_CD;
  };

  struct DubinsResult {
    geometry_lib::Circle c1;
    geometry_lib::Circle c2;
    geometry_lib::TangentOutput tangent_result;
  };

 public:
  void SetInput(Input& input) { input_ = input; }

  const bool DubinsCalculate(DubinsLibrary::DubinsResult& result,
                             const uint8_t dubins_type);

  bool Solve(uint8_t dubins_type, uint8_t case_type);
  void Sampling(double ds);
  const Output GetOutput() const { return output_; }

 private:
  Input input_;
  Output output_;
  std::array<Output, DUBINS_TYPE_COUNT * CASE_COUNT> output_arr;
  uint8_t dubins_type_ = 0;
};
}  // namespace dubins_lib
}  // namespace pnc

#endif