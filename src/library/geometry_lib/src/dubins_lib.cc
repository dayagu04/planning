#include "dubins_lib.h"

#include <math.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <utility>

#include "geometry_math.h"
#include "math_lib.h"
#include "transform_lib.h"

using namespace pnc::geometry_lib;

namespace pnc {
namespace dubins_lib {

static const double dist_tol = 0.05;

void DubinsLibrary::LineArcCalculate(DubinsLibrary::GeometryResult& result,
                                     const uint8_t line_arc_type) {
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

  const auto cos_dtheta = std::min(t1.dot(t2), 0.999);
  const auto sin_dtheta = sqrt(1.0 - cos_dtheta * cos_dtheta);

  const auto r = d_t / (1.0 - cos_dtheta);
  const auto l = d_n - r * sin_dtheta;

  // std::cout << "t1 = " << t1.transpose() << std::endl;
  // std::cout << "t2 = " << t2.transpose() << std::endl;
  // std::cout << "d_n = " << d_n << std::endl;
  // std::cout << "cos_dtheta = " << cos_dtheta << std::endl;
  // std::cout << "r = " << r << std::endl;
  // std::cout << "l = " << l << std::endl;

  if (line_arc_type == S_L || line_arc_type == S_R) {
    result.c2.radius = r;
    result.c2.center = p2 + n2 * r;
    result.c1 = result.c2;
    result.tangent_result.cross_point = result.c2.center - n1 * r;
  } else if (line_arc_type == L_S || line_arc_type == R_S) {
    result.c1.radius = r;
    result.c1.center = p1 + n1 * r;
    result.c2 = result.c1;
    result.tangent_result.cross_point = result.c1.center - n2 * r;
  }

  // std::cout << "result.tangent_result.cross_point = "
  //           << result.tangent_result.cross_point.transpose() << std::endl;

  result.n1 = n1;
  result.n2 = n2;
  result.t1 = t1;
  result.t2 = t2;

  result.l = l;
  result.r = r;
}

void DubinsLibrary::DubinsCalculate(DubinsLibrary::GeometryResult& result,
                                    const uint8_t dubins_type) {
  // 1: start
  // 2: target
  result.is_line_arc = false;

  double lambda1 = 1.0;
  double lambda2 = -1.0;

  if (dubins_type == L_S_R) {
    lambda1 = 1.0;
    lambda2 = -1.0;
    // std::cout << "L_S_R" << std::endl;
  } else if (dubins_type == R_S_L) {
    lambda1 = -1.0;
    lambda2 = 1.0;
    // std::cout << "R_S_L" << std::endl;
  } else if (dubins_type == L_S_L) {
    lambda1 = 1.0;
    lambda2 = 1.0;
    // std::cout << "L_S_L" << std::endl;
  } else if (dubins_type == R_S_R) {
    lambda1 = -1.0;
    lambda2 = -1.0;
    // std::cout << "R_S_R" << std::endl;
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
  CalTangentPointsOfEqualCircles(tangent_result, c1, c2);

  result.tangent_result = tangent_result;
  result.c1 = c1;
  result.c2 = c2;

  result.t1 = t1;
  result.t2 = t2;

  result.n1 = n1;
  result.n2 = n2;

  result.r = input_.radius;
}

// solve by line arc
bool DubinsLibrary::Solve(uint8_t line_arc_type) {
  line_arc_type_ = line_arc_type;
  GeometryResult line_arc_result;
  LineArcCalculate(line_arc_result, line_arc_type);

  SetOutputByLineArcType(output_, line_arc_result, line_arc_type);
  const auto flag = GenLineArcOutput(output_, line_arc_result);

  return flag;
}

// solve by dubins
bool DubinsLibrary::Solve(uint8_t dubins_type, uint8_t case_type) {
  dubins_type_ = dubins_type;

  GeometryResult dubins_result;
  DubinsCalculate(dubins_result, dubins_type);

  // assemble output info
  auto target_points = dubins_result.tangent_result.tagent_points_b;
  if (case_type == CASE_A) {
    target_points = dubins_result.tangent_result.tagent_points_a;
  }

  SetOutputByCaseType(output_, dubins_result, case_type);
  const auto flag = GenDubinsOutput(output_, dubins_result);

  return flag;
}

void DubinsLibrary::SetOutputByCaseType(
    Output& output, const DubinsLibrary::GeometryResult& result,
    const uint8_t case_type) {
  auto target_points = result.tangent_result.tagent_points_b;
  if (case_type == CASE_A) {
    target_points = result.tangent_result.tagent_points_a;
  }
  output.case_type = case_type;
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
  output.line_arc_type = line_arc_type;

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
  const auto& v_O1A = output_.arc_AB.pA - output_.arc_AB.circle_info.center;
  const auto& v_O1B = output_.arc_AB.pB - output_.arc_AB.circle_info.center;

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

const bool DubinsLibrary::GenLineArcOutput(
    Output& output, const DubinsLibrary::GeometryResult& result) {
  output.is_line_arc = true;
  const auto v_O1A = output.arc_AB.pA - output.arc_AB.circle_info.center;
  const auto v_O1B = output.arc_AB.pB - output.arc_AB.circle_info.center;

  const auto v_O2C = output.arc_CD.pA - output.arc_CD.circle_info.center;
  const auto v_O2D = output.arc_CD.pB - output.arc_CD.circle_info.center;

  const auto rotm_AB = geometry_lib::GetRotm2dFromTwoVec(v_O1A, v_O1B);
  const auto rotm_CD = geometry_lib::GetRotm2dFromTwoVec(v_O2C, v_O2D);

  const auto theta2 = geometry_lib::GetAngleFromTwoVec(v_O2C, v_O2D);

  if (result.line_arc_type == S_L || result.line_arc_type == S_R) {
    // arc AB
    output.arc_AB.length = 0.0;
    output.arc_AB.headingA = input_.heading1;
    output.arc_AB.headingB = input_.heading1;

    // line BC
    const auto v_BC = output.line_BC.pB - output.line_BC.pA;
    const auto v_BC_norm = v_BC.norm();
    if (v_BC_norm > dist_tol &&
        std::fabs(v_BC.dot(result.t1) / v_BC_norm) < 0.999962) {
      // std::cout << "BC wrong!" << std::endl;
      output.path_available = false;
      return false;
    }
    output.line_BC.length = (output.line_BC.pA - output.line_BC.pB).norm();
    output.line_BC.heading = output.arc_AB.headingB;

    // arc CD
    output.arc_CD.length = std::fabs(theta2) * result.r;
    const auto theta2 = geometry_lib::GetAngleFromTwoVec(v_O2C, v_O2D);
    const auto t_AB_out = rotm_AB * result.t1;
    const auto t2_out = rotm_CD * t_AB_out;
    output.arc_CD.headingA = output.arc_AB.headingB;
    output.arc_CD.headingB = atan2(t2_out.y(), t2_out.x());
    output.arc_CD.is_anti_clockwise = (theta2 > 0.0);
  } else if (result.line_arc_type == L_S || result.line_arc_type == R_S) {
    // arc AB
    const auto theta1 = geometry_lib::GetAngleFromTwoVec(v_O1A, v_O1B);
    output.arc_AB.length = std::fabs(theta1) * result.r;
    output.arc_AB.headingA = input_.heading1;
    output.dtheta_arc_AB = theta1;
    const auto t_AB_out = rotm_AB * result.t1;
    output.arc_AB.headingB = atan2(t_AB_out.y(), t_AB_out.x());
    output.arc_AB.is_anti_clockwise = (theta1 > 0.0);

    // line BC
    const auto v_BC = output.line_BC.pB - output.line_BC.pA;
    const auto v_BC_norm = v_BC.norm();
    if (v_BC_norm > dist_tol &&
        std::fabs(v_BC.dot(result.t2) / v_BC_norm) < 0.999962) {
      // std::cout << "BC wrong!" << std::endl;
      output.path_available = false;
      return false;
    }
    output.line_BC.length = (output.line_BC.pA - output.line_BC.pB).norm();
    output.line_BC.heading = output.arc_AB.headingB;

    // arc CD
    output.arc_CD.length = 0.0;
    output.arc_CD.headingA = input_.heading2;
    output.arc_CD.headingB = input_.heading2;
  }

  output.path_available = true;
  output.line_arc_radius = result.r;

  // std::cout << "output.pA = " << output.arc_AB.pA.transpose() << std::endl;
  // std::cout << "output.pB = " << output.arc_AB.pB.transpose() << std::endl;
  // std::cout << "output.pC = " << output.arc_CD.pA.transpose() << std::endl;
  // std::cout << "output.pD = " << output.arc_CD.pB.transpose() << std::endl;

  // std::cout << " output.arc_AB.length =" << output.arc_AB.length <<
  // std::endl; std::cout << " output.line_BC.length =" << output.line_BC.length
  // << std::endl; std::cout << " output.arc_CD.length =" <<
  // output.arc_CD.length << std::endl;

  output.length =
      output.arc_AB.length + output.line_BC.length + output.arc_CD.length;
  if (output.length < dist_tol) {
    output.path_available = false;
    return false;
  }

  // calculate gear info
  output.gear_cmd_vec.clear();
  output.gear_cmd_vec.reserve(3);
  output.gear_change_count = 0;
  output.gear_change_index = 18;  // just for fun: 18

  uint8_t current_gear = 0;
  output.current_gear_cmd = EMPTY;

  // set some type info
  output.dubins_type = result.dubins_type;

  // 1. AB arc
  // check if AB arc is too short
  if (output.arc_AB.length > dist_tol) {
    if (v_O1B.dot(result.t1) < 0.0) {
      current_gear = REVERSE;
    } else {
      current_gear = NORMAL;
    }
    output.arc_AB.is_ignored = false;
  } else {
    current_gear = EMPTY;
    output.arc_AB.length = 0.0;
    output.arc_AB.is_ignored = true;
  }

  output.gear_cmd_vec.emplace_back(current_gear);
  uint8_t last_gear = current_gear;

  const auto t1_after_AB = rotm_AB * result.t1;

  if (output.current_gear_cmd == EMPTY) {
    output.current_gear_cmd = current_gear;
  }

  // 2. BC line segment
  if (output.line_BC.length > dist_tol) {
    const auto v_BC = output.line_BC.pB - output.line_BC.pA;
    if (t1_after_AB.dot(v_BC) < 0.0) {
      current_gear = REVERSE;
    } else {
      current_gear = NORMAL;
    }
    output.line_BC.is_ignored = false;
  } else {
    current_gear = EMPTY;
    output.line_BC.length = 0.0;
    output.line_BC.is_ignored = true;
  }
  output.gear_cmd_vec.emplace_back(current_gear);
  if (last_gear != EMPTY && current_gear != EMPTY &&
      last_gear != current_gear) {
    if (output.gear_change_count == 0) {
      output.gear_change_index = 1;
    }
    output.gear_change_count++;
  }

  last_gear = current_gear;

  if (output.current_gear_cmd == EMPTY) {
    output.current_gear_cmd = current_gear;
  }

  // 3. CD arc
  // check if CD arc is too short
  if (output.arc_CD.length > dist_tol) {
    if (v_O2D.dot(t1_after_AB) < 0.0) {
      current_gear = REVERSE;
    } else {
      current_gear = NORMAL;
    }
    output.arc_CD.is_ignored = false;
  } else {
    current_gear = EMPTY;
    output.arc_CD.length = 0.0;
    output.arc_CD.is_ignored = true;
  }

  output.gear_cmd_vec.emplace_back(current_gear);

  bool gear_change_flag = false;
  if (last_gear != EMPTY) {
    if (current_gear != EMPTY && last_gear != current_gear) {
      gear_change_flag = true;
    }
  } else {
    if (current_gear != EMPTY && output.gear_cmd_vec[0] != current_gear &&
        output.gear_cmd_vec[0] != EMPTY) {
      gear_change_flag = true;
    }
  }

  if (gear_change_flag) {
    if (output.gear_change_count == 0) {
      output.gear_change_index = 2;
    }
    output.gear_change_count++;
  }

  last_gear = current_gear;

  if (output.current_gear_cmd == EMPTY) {
    output.current_gear_cmd = current_gear;
  }
  return true;
}

const bool DubinsLibrary::GenDubinsOutput(
    Output& output, const DubinsLibrary::GeometryResult& result) {
  output.is_line_arc = false;
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
    return false;
  } else {
    output.path_available = true;

    // calculate length and heading
    // arc AB
    const auto theta1 = geometry_lib::GetAngleFromTwoVec(v_O1A, v_O1B);
    output.arc_AB.length = std::fabs(theta1) * radius;
    output.arc_AB.headingA = input_.heading1;
    output.dtheta_arc_AB = theta1;

    const auto t_AB_out = rotm_AB * result.t1;
    output.arc_AB.headingB = atan2(t_AB_out.y(), t_AB_out.x());

    output.arc_AB.is_anti_clockwise = (theta1 > 0.0);

    // line BC
    output.line_BC.length = (output.line_BC.pA - output.line_BC.pB).norm();
    output.line_BC.heading = output.arc_AB.headingB;

    // arc CD
    const auto theta2 = geometry_lib::GetAngleFromTwoVec(v_O2C, v_O2D);
    output.arc_CD.length = std::fabs(theta2) * radius;
    const auto t2_out = rotm_CD * t_AB_out;
    output.arc_CD.headingA = output.arc_AB.headingB;
    output.arc_CD.headingB = atan2(t2_out.y(), t2_out.x());
    output.arc_CD.is_anti_clockwise = (theta2 > 0.0);

    output.length =
        output.arc_AB.length + output.line_BC.length + output.arc_CD.length;

    // calculate gear info
    output.gear_cmd_vec.clear();
    output.gear_cmd_vec.reserve(3);
    output.gear_change_count = 0;
    output.gear_change_index = 18;  // just for fun: 18

    uint8_t current_gear = 0;
    output.current_gear_cmd = EMPTY;

    // set some type info
    output.dubins_type = result.dubins_type;

    // 1. AB arc
    // check if AB arc is too short
    if (output.arc_AB.length > dist_tol) {
      if (v_O1B.dot(result.t1) < 0.0) {
        current_gear = REVERSE;
      } else {
        current_gear = NORMAL;
      }
      output.arc_AB.is_ignored = false;
    } else {
      current_gear = EMPTY;
      output.arc_AB.length = 0.0;
      output.arc_AB.is_ignored = true;
    }

    output.gear_cmd_vec.emplace_back(current_gear);
    uint8_t last_gear = current_gear;

    const auto t1_after_AB = rotm_AB * result.t1;

    if (output.current_gear_cmd == EMPTY) {
      output.current_gear_cmd = current_gear;
    }

    // 2. BC line segment
    if (output.line_BC.length > dist_tol) {
      const auto v_BC = output.line_BC.pB - output.line_BC.pA;
      if (t1_after_AB.dot(v_BC) < 0.0) {
        current_gear = REVERSE;
      } else {
        current_gear = NORMAL;
      }
      output.line_BC.is_ignored = false;
    } else {
      current_gear = EMPTY;
      output.line_BC.length = 0.0;
      output.line_BC.is_ignored = true;
    }
    output.gear_cmd_vec.emplace_back(current_gear);
    if (last_gear != EMPTY && current_gear != EMPTY &&
        last_gear != current_gear) {
      if (output.gear_change_count == 0) {
        output.gear_change_index = 1;
      }
      output.gear_change_count++;

      output.current_length = output.arc_AB.length;
    } else {
      output.current_length = output.arc_AB.length + output.line_BC.length;
    }

    last_gear = current_gear;

    if (output.current_gear_cmd == EMPTY) {
      output.current_gear_cmd = current_gear;
    }

    // 3. CD arc
    // check if CD arc is too short
    if (output.arc_CD.length > dist_tol) {
      if (v_O2D.dot(t1_after_AB) < 0.0) {
        current_gear = REVERSE;
      } else {
        current_gear = NORMAL;
      }
      output.arc_CD.is_ignored = false;
    } else {
      current_gear = EMPTY;
      output.arc_CD.length = 0.0;
      output.arc_CD.is_ignored = true;
    }

    output.gear_cmd_vec.emplace_back(current_gear);

    bool gear_change_flag = false;
    if (last_gear != EMPTY) {
      if (current_gear != EMPTY && last_gear != current_gear) {
        gear_change_flag = true;
      }
    } else {
      if (current_gear != EMPTY && output.gear_cmd_vec[0] != current_gear &&
          output.gear_cmd_vec[0] != EMPTY) {
        gear_change_flag = true;
      }
    }

    if (gear_change_flag) {
      if (output.gear_change_count == 0) {
        output.gear_change_index = 2;
      }
      output.gear_change_count++;
    } else {
      if (output.gear_change_count == 0) {
        output.current_length =
            output.arc_AB.length + output.line_BC.length + output.arc_CD.length;
      }
    }

    last_gear = current_gear;

    if (output.current_gear_cmd == EMPTY) {
      output.current_gear_cmd = current_gear;
    }
  }

  return true;
}

void DubinsLibrary::GetSampling(Output& output, const double ds,
                                const bool is_complete_path) {
  if (!output.path_available) {
    return;
  }

  const size_t N = std::ceil(output.length / ds) + 10;

  output.path_point_vec.clear();
  output.path_point_vec.reserve(N);

  const auto& arc_AB_length = output.arc_AB.length;
  const auto& line_BC_length = output.line_BC.length;
  const auto& arc_CD_length = output.arc_CD.length;
  const auto& gear_change_index = output.gear_change_index;

  geometry_lib::PathPoint path_point;

  bool set_pB = false;
  bool set_pC = false;
  // set pA always
  path_point.Set(output.arc_AB.pA, output.arc_AB.headingA);
  output.path_point_vec.emplace_back(path_point);
  // sampling arc AB first
  if (!output.arc_AB.is_ignored) {
    // sample rest of arc AB (pB is not included)
    const auto& pO1 = output.arc_AB.circle_info.center;

    const double dtheta1 = ds / output.arc_AB.circle_info.radius *
                           (output.arc_AB.is_anti_clockwise ? 1.0 : -1.0);

    const auto rot_m1 = GetRotm2dFromTheta(dtheta1);
    double s = ds;
    // v_O1A
    Eigen::Vector2d v_n = output.arc_AB.pA - output.arc_AB.circle_info.center;
    double theta = output.arc_AB.headingA;
    Eigen::Vector2d pn;

    if (arc_AB_length > ds) {
      while (s < arc_AB_length) {
        v_n = rot_m1 * v_n;
        pn = pO1 + v_n;
        s += ds;
        theta += dtheta1;

        path_point.Set(pn, NormalizeAngle(theta));
        output.path_point_vec.emplace_back(path_point);
      }
    }

    // set pB
    if (line_BC_length > ds) {
      path_point.Set(output.arc_AB.pB, output.arc_AB.headingB);
      output.path_point_vec.emplace_back(path_point);
      set_pB = true;
    }
  }

  output.path_seg_count = 1;

  if (!is_complete_path && gear_change_index == 1) {
    // std::cout << "AB in Path!" << std::endl;
    if (!set_pB) {
      // set pB if not
      path_point.Set(output.arc_AB.pB, output.arc_AB.headingB);
      output.path_point_vec.emplace_back(path_point);
    }
    return;
  }

  // sample rest of line BC (pC is not included)
  if (!output.line_BC.is_ignored) {
    double s = ds;
    auto pn = output.line_BC.pA;
    auto theta = output.arc_AB.headingB;

    const auto diff_vec =
        (output.line_BC.pB - output.line_BC.pA).normalized() * ds;
    if (line_BC_length > ds) {
      while (s < line_BC_length) {
        pn += diff_vec;
        s += ds;
        path_point.Set(pn, theta);
        output.path_point_vec.emplace_back(path_point);
      }
    }

    // set pC
    if (!set_pB) {
      // set pC if pB not
      set_pC = true;
      path_point.Set(output.arc_CD.pA, output.arc_CD.headingA);
      output.path_point_vec.emplace_back(path_point);
    }
  }
  output.path_seg_count = 2;

  if (!is_complete_path && gear_change_index == 2) {
    // std::cout << "AB & BC in Path!" << std::endl;
    if (!set_pC) {
      // set pC if not
      path_point.Set(output.arc_CD.pA, output.arc_CD.headingA);
      output.path_point_vec.emplace_back(path_point);
    }
    return;
  }

  if (!output.arc_CD.is_ignored) {
    // sample rest of arc CD (pD is not included)
    auto pn = output.arc_AB.pA;
    auto theta = output.arc_CD.headingA;

    const auto& pO2 = output.arc_CD.circle_info.center;

    const double dtheta2 = ds / output.arc_CD.circle_info.radius *
                           (output.arc_CD.is_anti_clockwise ? 1.0 : -1.0);

    const auto rot_m2 = GetRotm2dFromTheta(dtheta2);
    // v_O2C
    Eigen::Vector2d v_n = output.arc_CD.pA - output.arc_CD.circle_info.center;
    auto s = ds;
    if (arc_CD_length > ds) {
      while (s < arc_CD_length) {
        v_n = rot_m2 * v_n;
        pn = pO2 + v_n;
        s += ds;
        theta += dtheta2;

        path_point.Set(pn, NormalizeAngle(theta));
        output.path_point_vec.emplace_back(path_point);
      }
    }
  }

  // set pD always
  path_point.Set(output.arc_CD.pB, output.arc_CD.headingB);
  output.path_point_vec.emplace_back(path_point);

  // std::cout << "AB & BC & CD, all in Path!" << std::endl;
  output.path_seg_count = 3;
}

void DubinsLibrary::Sampling(const double ds, const bool is_complete_path) {
  GetSampling(output_, ds, is_complete_path);
}

void DubinsLibrary::Extend(const double extend_s) {
  if (extend_s < 0.0) {
    return;
  }

  const auto& end_heading = output_.path_point_vec.back().heading;
  double direction_scale = 1.0;

  if (output_.current_gear_cmd == REVERSE) {
    direction_scale = -1.0;
  }

  const Eigen::Vector2d diff_vec_t =
      direction_scale *
      Eigen::Vector2d(std::cos(end_heading), std::sin(end_heading));

  auto pn = output_.path_point_vec.back().pos + diff_vec_t * extend_s;

  geometry_lib::PathPoint path_point;
  path_point.Set(pn, end_heading);
  output_.path_point_vec.emplace_back(path_point);
}

void DubinsLibrary::GetTransform(
    std::vector<geometry_lib::PathPoint>& path_point_vec,
    const geometry_lib::LocalToGlobalTf& l2g_tf) {
  for (auto& path_point : path_point_vec) {
    const Eigen::Vector2d pos_g = l2g_tf.GetPos(path_point.pos);
    const double heading_g = l2g_tf.GetHeading(path_point.heading);

    path_point.Set(std::move(pos_g), heading_g);
  }
}

void DubinsLibrary::Transform(const geometry_lib::LocalToGlobalTf& l2g_tf) {
  GetTransform(output_.path_point_vec, l2g_tf);
}

const std::vector<double> DubinsLibrary::GetPathEle(size_t index) const {
  std::vector<double> out_vec;
  const auto& N = output_.path_point_vec.size();
  out_vec.reserve(3);

  for (size_t i = 0; i < N; ++i) {
    if (index == 0) {
      out_vec.emplace_back(output_.path_point_vec[i].pos.x());
    } else if (index == 1) {
      out_vec.emplace_back(output_.path_point_vec[i].pos.y());
    } else if (index == 2) {
      out_vec.emplace_back(output_.path_point_vec[i].heading);
    }
  }

  return out_vec;
}

void DubinsLibrary::PrintOutput() const {
  std::cout << "------------- dubins result ---" << std::endl;
  const auto& output = GetOutput();
  std::cout << "path_available = " << output.path_available << std::endl;
  std::cout << "pA = " << output.arc_AB.pA.transpose() << std::endl;
  std::cout << "pB = " << output.arc_AB.pB.transpose() << std::endl;
  std::cout << "pC = " << output.arc_CD.pA.transpose() << std::endl;
  std::cout << "pD = " << output.arc_CD.pB.transpose() << std::endl;

  std::cout << "target_pos = " << GetOutput().arc_CD.pB.transpose()
            << ", target_heading_deg = " << GetOutput().arc_CD.headingB * 57.3
            << std::endl;

  std::cout << "ego_pos = " << GetOutput().arc_AB.pA.transpose() << std::endl;
  std::cout << "ego_heading_deg = " << GetOutput().arc_AB.headingB * 57.3
            << std::endl;

  std::cout << "path length = " << output.length << std::endl;

  std::cout << "AB_length = " << output.arc_AB.length
            << ", BC_length = " << output.line_BC.length
            << ", CD_length = " << output.arc_CD.length << std::endl;

  std::cout << "line_arc_radius = " << output.line_arc_radius << std::endl;

  const auto tmp_gear_cmd_vec = Eigen::Vector3d(
      output.gear_cmd_vec[0], output.gear_cmd_vec[1], output.gear_cmd_vec[2]);

  std::cout << "gear_cmd_vec = " << tmp_gear_cmd_vec.transpose() << std::endl;

  std::cout << "gear_change_count = "
            << static_cast<int>(output.gear_change_count) << std::endl;

  std::cout << "is_line_arc = " << output.is_line_arc << std::endl;

  std::cout << "dubins_type = " << static_cast<int>(output.dubins_type)
            << std::endl;
  std::cout << "case_type = " << static_cast<int>(output.case_type)
            << std::endl;

  std::cout << "line_arc_type = " << static_cast<int>(output.line_arc_type)
            << std::endl;
}

const bool DubinsLibrary::OneStepDubinsUpdate() {
  // try line arc method
  for (size_t i = 0; i < DubinsLibrary::LINEARC_TYPE_COUNT; ++i) {
    if (Solve(i)) {
      if (output_.gear_change_count == 0 &&
          output_.line_arc_radius >= input_.radius) {
        std::cout << "line arc success!" << std::endl;
        return true;
      }
    }

  }  // linearc loop

  // try dubins method
  for (size_t i = 0; i < DubinsLibrary::CASE_COUNT; ++i) {
    for (size_t j = 0; j < DubinsLibrary::DUBINS_TYPE_COUNT; ++j) {
      if (Solve(j, i)) {
        if (output_.gear_change_count == 0) {
          std::cout << "dubins success!" << std::endl;
          return true;
        }
      }
    }
  }  // dubins loop
  // std::cout << "dubins failed!" << std::endl;

  return false;
}

}  // namespace dubins_lib
}  // namespace pnc