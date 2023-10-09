#include "dubins_lib.h"

#include <math.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include "geometry_math.h"
#include "transform_lib.h"

using namespace pnc::geometry_lib;

namespace pnc {
namespace dubins_lib {

const bool DubinsLibrary::LineArcCalculate(
    DubinsLibrary::GeometryResult& result, const uint8_t line_arc_type) {
  result.is_line_arc = true;
  result.line_arc_type = line_arc_type;

  const auto& p1 = input_.p1;
  const auto& p2 = input_.p2;

  const auto t1 = Eigen::Vector2d(cos(input_.heading1), sin(input_.heading1));
  const auto t2 = Eigen::Vector2d(cos(input_.heading2), sin(input_.heading2));

  // 1: start
  // 2: target
  Eigen::Vector2d arc_point;
  Eigen::Vector2d n1;
  Eigen::Vector2d n2;

  LineSegment line_segment_t;
  LineSegment line_segment_n;

  if (line_arc_type == L_S) {
    arc_point = p1;
    n1 << -t1.y(), t1.x();
    n2 << -t2.y(), t2.x();
    line_segment_t.SetPoints(p2, p2 - t2);
    line_segment_n.SetPoints(p2, p2 - n2);
  } else if (line_arc_type == R_S) {
    arc_point = p1;
    n1 << t1.y(), -t1.x();
    n2 << t2.y(), -t2.x();
    line_segment_t.SetPoints(p2, p2 - t2);
    line_segment_n.SetPoints(p2, p2 - n2);
  } else if (line_arc_type == S_L) {
    arc_point = p2;
    n1 << -t1.y(), t1.x();
    n2 << -t2.y(), t2.x();
    line_segment_t.SetPoints(p1, p1 - t1);
    line_segment_n.SetPoints(p1, p1 - n1);
  } else if (line_arc_type == S_R) {
    arc_point = p2;
    n1 << t1.y(), -t1.x();
    n2 << t2.y(), -t2.x();
    line_segment_t.SetPoints(p1, p1 - t1);
    line_segment_n.SetPoints(p1, p1 - n1);
  }

  const auto d_t = CalPoint2LineDist(arc_point, line_segment_t);
  const auto d_n = CalPoint2LineDist(arc_point, line_segment_n);

  const auto cos_dtheta = t1.dot(t2);
  const auto sin_dtheta = sqrt(1.0 - cos_dtheta * cos_dtheta);

  const auto r = d_t / (1.0 - cos_dtheta);
  const auto l = d_n - r * sin_dtheta;

  // std::cout << "d_t = " << d_t << std::endl;
  // std::cout << "d_n = " << d_n << std::endl;
  // std::cout << "cos_dtheta = " << cos_dtheta << std::endl;
  // std::cout << "r = " << r << std::endl;
  // std::cout << "l = " << l << std::endl;

  if (line_arc_type == L_S || line_arc_type == R_S) {
    result.c1.radius = r;
    result.c1.center = p1 + n1 * r;
    result.c2 = result.c1;
    result.tangent_result.cross_point = result.c1.center - n2 * r;
  } else if (line_arc_type == S_L || line_arc_type == S_R) {
    result.c2.radius = r;
    result.c2.center = p2 + n2 * r;
    result.c1 = result.c2;
    result.tangent_result.cross_point = result.c2.center - n1 * r;
  }

  result.n1 = n1;
  result.n2 = n2;
  result.t1 = t1;
  result.t2 = t2;

  result.l = l;
  result.r = r;

  return true;
}

const bool DubinsLibrary::DubinsCalculate(DubinsLibrary::GeometryResult& result,
                                          const uint8_t dubins_type) {
  // 1: start
  // 2: target
  result.is_line_arc = false;

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

  result.dubins_type = dubins_type;

  // calculate c1 center: start pose
  Circle c1;
  Eigen::Vector2d t1(cos(input_.heading1), sin(input_.heading1));
  Eigen::Vector2d n1(-lambda1 * t1.y(), lambda1 * t1.x());

  c1.center = input_.p1 + input_.radius * n1;
  c1.radius = input_.radius;

  // calculate c2 center: target pose
  Circle c2;
  Eigen::Vector2d t2(cos(input_.heading2), sin(input_.heading2));
  Eigen::Vector2d n2(-lambda2 * t2.y(), lambda2 * t2.x());

  c2.center = input_.p2 + input_.radius * n2;
  c2.radius = input_.radius;

  // calculate tagent points
  TangentOutput tangent_result;
  const bool flag = CalTangentPointsOfEqualCircles(tangent_result, c1, c2);

  if (!flag) {
    std::cout << "no tangent points between these two circles!" << std::endl;
  }

  result.tangent_result = tangent_result;
  result.c1 = c1;
  result.c2 = c2;

  result.t1 = t1;
  result.t2 = t2;

  result.n1 = n1;
  result.n2 = n2;

  result.r = input_.radius;

  return flag;
}

bool DubinsLibrary::Solve(uint8_t line_arc_type) {
  line_arc_type_ = line_arc_type;
  GeometryResult line_arc_result;
  const auto flag = LineArcCalculate(line_arc_result, line_arc_type);

  SetOutputByLineArcType(output_, line_arc_result, line_arc_type);
  GenDubinsOutput(output_, line_arc_result);

  return flag;
}

bool DubinsLibrary::Solve(uint8_t dubins_type, uint8_t case_type) {
  dubins_type_ = dubins_type;

  GeometryResult dubins_result;
  const auto flag = DubinsCalculate(dubins_result, dubins_type);

  // assemble output info
  auto target_points = dubins_result.tangent_result.tagent_points_b;
  if (case_type == CASE_A) {
    target_points = dubins_result.tangent_result.tagent_points_a;
  }

  SetOutputByCaseType(output_, dubins_result, case_type);
  GenDubinsOutput(output_, dubins_result);

  return flag;
}

void DubinsLibrary::SetOutputByCaseType(
    Output& output, const DubinsLibrary::GeometryResult& result,
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

void DubinsLibrary::SetOutputByLineArcType(
    Output& output, const DubinsLibrary::GeometryResult& result,
    const uint8_t line_arc_type) {
  output.arc_AB.circle_info = result.c1;
  output.arc_CD.circle_info = result.c2;

  if (line_arc_type == L_S || line_arc_type == R_S) {
    output.arc_AB.pA = input_.p1;
    output.arc_AB.pB = result.tangent_result.cross_point;

    output.line_BC.pA = output.arc_AB.pB;
    output.line_BC.pB = input_.p2;

    output.arc_CD.pA = input_.p2;
    output.arc_CD.pB = input_.p2;
  } else if (line_arc_type == S_L || line_arc_type == S_R) {
    output.arc_AB.pA = input_.p1;
    output.arc_AB.pB = input_.p1;

    output.line_BC.pA = input_.p1;
    output.line_BC.pB = result.tangent_result.cross_point;

    output.arc_CD.pA = output.line_BC.pB;
    output.arc_CD.pB = input_.p2;
  }
}

const double DubinsLibrary::GetThetaBC() const {
  const auto t1 = Eigen::Vector2d(cos(input_.heading1), sin(input_.heading1));

  // rotation from O1A to O1B, then O2C to O2D, AB does not change heading
  const auto v_O1A = output_.arc_AB.pA - output_.arc_AB.circle_info.center;
  const auto v_O1B = output_.arc_AB.pB - output_.arc_AB.circle_info.center;

  const auto rotm_AB = geometry_lib::GetRotm2dFromTwoVec(v_O1A, v_O1B);

  // car heading vector after two rotations
  const auto t1_out = rotm_AB * t1;

  return atan2(t1_out.y(), t1_out.x());
}

const double DubinsLibrary::GetThetaD() const {
  const auto theta_BC = GetThetaBC();
  const auto t1_after_AB = Eigen::Vector2d(cos(theta_BC), sin(theta_BC));

  const auto v_O2C = output_.arc_CD.pA - output_.arc_CD.circle_info.center;
  const auto v_O2D = output_.arc_CD.pB - output_.arc_CD.circle_info.center;

  // car heading vector after two rotations
  const auto rotm_CD = geometry_lib::GetRotm2dFromTwoVec(v_O2C, v_O2D);
  const auto t1_out = rotm_CD * t1_after_AB;

  return atan2(t1_out.y(), t1_out.x());
}

void DubinsLibrary::GenDubinsOutput(
    Output& output, const DubinsLibrary::GeometryResult& result) {
  if (result.is_line_arc && result.r < 0.99 * input_.radius) {
    output.path_available = false;
    return;
  }

  // rotation from O1A to O1B, then O2C to O2D, AB does not change heading
  const auto v_O1A = output.arc_AB.pA - output.arc_AB.circle_info.center;
  const auto v_O1B = output.arc_AB.pB - output.arc_AB.circle_info.center;

  const auto v_O2C = output.arc_CD.pA - output.arc_CD.circle_info.center;
  const auto v_O2D = output.arc_CD.pB - output.arc_CD.circle_info.center;

  const auto rotm_AB = geometry_lib::GetRotm2dFromTwoVec(v_O1A, v_O1B);
  const auto rotm_CD = geometry_lib::GetRotm2dFromTwoVec(v_O2C, v_O2D);

  // car heading vector after two rotations
  const auto t1_out = rotm_CD * rotm_AB * result.t1;

  // 1. reversed heading should be kicked out
  // 2. too long arc length should be kicked out: angle > 120.0 deg
  const auto radius = result.c1.radius;

  const auto cos_120deg = -0.5;

  // note that norm of O1A, O1B, O2C, O2D, are all equal to radius
  const auto cos_AO1B = v_O1A.dot(v_O1B) / (radius * radius);
  const auto cos_CO2D = v_O2C.dot(v_O2D) / (radius * radius);

  const auto res = t1_out.dot(result.t2);

  // std::cout << "res = " << res << std::endl;

  if (res < -0.9 || cos_AO1B < cos_120deg || cos_CO2D < cos_120deg) {
    output.path_available = false;
    return;
  } else {
    output.path_available = true;

    // calculate arc length
    const auto theta1 = transform::GetAngleFromTwoVec(v_O1A, v_O1B);
    output.arc_AB.length = theta1 * radius;

    output.line_BC.length = (output.line_BC.pA - output.line_BC.pB).norm();

    const auto theta2 = transform::GetAngleFromTwoVec(v_O2C, v_O2D);
    output.arc_CD.length = theta2 * radius;

    output.length =
        output.arc_AB.length + output.line_BC.length + output.arc_CD.length;

    // calculate gear info
    output.gear_cmd_vec.clear();
    output.gear_cmd_vec.reserve(3);
    output.gear_change_count = 0;

    uint8_t current_gear = 0;
    // 1. AB arc
    // check if AB arc is too short
    if (output.arc_AB.length > 2.5e-2) {
      if (v_O1B.dot(result.t1) < 0.0) {
        current_gear = REVERSE;
      } else {
        current_gear = NORMAL;
      }
    } else {
      current_gear = EMPTY;
    }
    output.gear_cmd_vec.emplace_back(current_gear);
    uint8_t last_gear = current_gear;

    const auto t1_after_AB = rotm_AB * result.t1;

    // 2. BC line segment
    if (output.line_BC.length > 2.5e-2) {
      const auto v_BC = output.line_BC.pB - output.line_BC.pA;
      if (t1_after_AB.dot(v_BC) < 0.0) {
        current_gear = REVERSE;
      } else {
        current_gear = NORMAL;
      }
    } else {
      current_gear = EMPTY;
    }
    output.gear_cmd_vec.emplace_back(current_gear);
    if (last_gear != current_gear && last_gear != EMPTY) {
      output.gear_change_count++;
    }
    last_gear = current_gear;

    // 3. CD arc
    // check if CD arc is too short
    if (output.arc_CD.length > 2.5e-2) {
      if (v_O2D.dot(t1_after_AB) < 0.0) {
        current_gear = REVERSE;
      } else {
        current_gear = NORMAL;
      }
    } else {
      current_gear = EMPTY;
    }

    output.gear_cmd_vec.emplace_back(current_gear);
    if (last_gear != current_gear && last_gear != EMPTY) {
      output.gear_change_count++;
    }
    last_gear = current_gear;
  }

  if (result.is_line_arc && output.gear_change_count > 0) {
    output.path_available = false;
    return;
  }
}

}  // namespace dubins_lib
}  // namespace pnc