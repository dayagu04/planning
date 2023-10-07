#include "dubins_lib.h"

#include <math.h>

#include <cstddef>
#include <iostream>

#include "geometry_math.h"

using namespace pnc::geometry_lib;

namespace pnc {
namespace dubins_lib {

const bool DubinsLibrary::DubinsCalculate(DubinsLibrary::DubinsResult& result,
                                          const uint8_t dubins_type) {
  // 1: start
  // 2: target
  double lambda1 = 1.0;
  double lambda2 = -1.0;

  if (dubins_type == L_S_R) {
    lambda1 = 1.0;
    lambda2 = -1.0;
    std::cout << "L_S_R" << std::endl;
  } else if (dubins_type == R_S_L) {
    lambda1 = -1.0;
    lambda2 = 1.0;
    std::cout << "R_S_L" << std::endl;
  } else if (dubins_type == L_S_L) {
    lambda1 = 1.0;
    lambda2 = 1.0;
    std::cout << "L_S_L" << std::endl;
  } else if (dubins_type == R_S_R) {
    lambda1 = -1.0;
    lambda2 = -1.0;
    std::cout << "R_S_R" << std::endl;
  }

  // calculate c1 center: start pose
  Circle c1;
  Eigen::Vector2d n1(-lambda1 * sin(input_.heading1),
                     lambda1 * cos(input_.heading1));

  c1.center = input_.p1 + input_.radius * n1;
  c1.radius = input_.radius;

  // calculate c2 center: target pose
  Circle c2;
  Eigen::Vector2d n2(-lambda2 * sin(input_.heading2),
                     lambda2 * cos(input_.heading2));

  c2.center = input_.p2 + input_.radius * n2;
  c2.radius = input_.radius;

  // calculate tagent points
  TangentOutput tangent_result;
  const bool flag = CalInnerTangentPointsOfEqualCircles(tangent_result, c1, c2);

  if (!flag) {
    std::cout << "no tangent points between these two circles!" << std::endl;
  }

  result.tangent_result = tangent_result;
  result.c1 = c1;
  result.c2 = c2;

  return flag;
}

bool DubinsLibrary::Solve(uint8_t dubins_type, uint8_t case_type) {
  dubins_type_ = dubins_type;

  DubinsResult dubins_result;
  const auto flag = DubinsCalculate(dubins_result, dubins_type);

  // assemble output info
  auto target_points = dubins_result.tangent_result.tagent_points_b;
  if (case_type == CASE_A) {
    target_points = dubins_result.tangent_result.tagent_points_a;
  }

  SetOutputByCaseType(output_, dubins_result, case_type);

  return flag;
}

void DubinsLibrary::SetOutputByCaseType(Output& output,
                                        DubinsLibrary::DubinsResult& result,
                                        const uint8_t case_type) {
  auto target_points = result.tangent_result.tagent_points_b;
  if (case_type == CASE_A) {
    target_points = result.tangent_result.tagent_points_a;
  }

  output.arc_AB.circle_info = result.c1;
  output.arc_AB.pA = input_.p1;
  output.arc_AB.pB = target_points.first;

  output.line_BC.pA = output.arc_AB.pB;
  output.line_BC.pB = target_points.second;

  output.arc_CD.circle_info = result.c2;
  output.arc_CD.pA = output.line_BC.pB;
  output.arc_CD.pB = input_.p2;
}

bool DubinsLibrary::SolveAll() {
  auto flag = true;

  DubinsResult dubins_result;
  Output output;

  for (size_t i = 0; i < DUBINS_TYPE_COUNT; ++i) {
    flag = flag && DubinsCalculate(dubins_result, i);
    // set case a
    SetOutputByCaseType(output, dubins_result, CASE_A);
    output_arr[2 * i] = output;

    // set case b
    SetOutputByCaseType(output, dubins_result, CASE_B);
    output_arr[2 * i + 1] = output;
  }

  return flag;
}

}  // namespace dubins_lib
}  // namespace pnc