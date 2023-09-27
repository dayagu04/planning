#include "dubins_lib.h"

#include <math.h>

#include <iostream>

#include "geometry_math.h"

using namespace pnc::geometry_lib;

namespace pnc {
namespace dubins_lib {
bool DubinsLibrary::Solve(uint8_t dubins_type, uint8_t case_type) {
  dubins_type_ = dubins_type;

  double lambda_start = 1.0;
  double lambda_target = -1.0;

  if (dubins_type == L_S_R) {
    lambda_start = 1.0;
    lambda_target = -1.0;
    std::cout << "L_S_R" << std::endl;
  } else if (dubins_type == R_S_L) {
    lambda_start = -1.0;
    lambda_target = 1.0;
    std::cout << "R_S_L" << std::endl;
  }

  // calculate c1 center: start pose
  Circle c1;
  Eigen::Vector2d t_start(cos(input_.heading_1), sin(input_.heading_1));

  Eigen::Vector2d n_start(-lambda_start * t_start.y(),
                          lambda_start * t_start.x());

  c1.center = input_.P_1 + input_.radius * n_start;
  c1.radius = input_.radius;

  // calculate c2 center: target pose
  Circle c2;
  Eigen::Vector2d t_target(cos(input_.heading_2), sin(input_.heading_2));

  Eigen::Vector2d n_target(-lambda_target * t_target.y(),
                           lambda_target * t_target.x());

  c2.center = input_.P_2 + input_.radius * n_target;
  c2.radius = input_.radius;

  // calculate tagent points
  TangentOutput output;
  const bool flag = CalInnerTangentPointsOfEqualCircles(output, c1, c2);

  if (!flag) {
    std::cout << "no tangent points between these two circles!" << std::endl;
  }

  // assemble output info
  output_.tangent_result = output;

  // consider P_1, TA1, TB1
  auto target_points = output.tagent_points_b;
  if (case_type == CASE_A) {
    target_points = output.tagent_points_a;
  }

  output_.arc_AB.circle_info = c1;
  output_.arc_AB.pA = input_.P_1;
  output_.arc_AB.pB = target_points.first;

  output_.line_BC.pA = output_.arc_AB.pB;
  output_.line_BC.pB = target_points.second;

  output_.arc_CD.circle_info = c2;
  output_.arc_CD.pA = output_.line_BC.pB;
  output_.arc_CD.pB = input_.P_2;

  return flag;
}

}  // namespace dubins_lib
}  // namespace pnc