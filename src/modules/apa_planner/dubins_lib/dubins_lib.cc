#include "dubins_lib.h"

#include <math.h>

#include <iostream>

#include "geometry_math.h"

using namespace pnc::geometry_lib;

namespace pnc {
namespace dubins_lib {
bool DubinsLibrary::Solve(uint8_t dubins_type, uint8_t case_type) {
  dubins_type_ = dubins_type;

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
  TangentOutput output;
  const bool flag = CalInnerTangentPointsOfEqualCircles(output, c1, c2);

  if (!flag) {
    std::cout << "no tangent points between these two circles!" << std::endl;
  }

  // assemble output info
  output_.tangent_result = output;

  // consider p1, TA1, TB1
  auto target_points = output.tagent_points_b;
  if (case_type == CASE_A) {
    target_points = output.tagent_points_a;
  }

  output_.arc_AB.circle_info = c1;
  output_.arc_AB.pA = input_.p1;
  output_.arc_AB.pB = target_points.first;

  output_.line_BC.pA = output_.arc_AB.pB;
  output_.line_BC.pB = target_points.second;

  output_.arc_CD.circle_info = c2;
  output_.arc_CD.pA = output_.line_BC.pB;
  output_.arc_CD.pB = input_.p2;

  return flag;
}

}  // namespace dubins_lib
}  // namespace pnc