#pragma once

#include <limits>

#include "common/vehicle_param_helper.h"

namespace planning {

struct PlanningPoint {
  PlanningPoint() = default;
  PlanningPoint(const double _x, const double _y, const double _theta)
      : x(_x), y(_y), theta(_theta) {}
  double x = 0;
  double y = 0;
  double theta = 0;
};

struct DiagonalSegmentsInfo {
  PlanningPoint opt_point_a;
  PlanningPoint opt_point_b;
  PlanningPoint opt_point_c;
  PlanningPoint opt_point_d;
  PlanningPoint opt_point_e;
  PlanningPoint opt_point_f;

  double turn_radius_min = 5.5;
  double opt_radius_bc = turn_radius_min;
  double opt_radius_cd = turn_radius_min;
  double opt_radius_de = turn_radius_min;
  double opt_radius_ef = turn_radius_min;

  double total_cost = std::numeric_limits<double>::infinity();
};

struct ParallelSegmentsInfo {
  PlanningPoint opt_point_a;
  PlanningPoint opt_point_b;
  PlanningPoint opt_point_c;
  PlanningPoint opt_point_d;
  PlanningPoint opt_point_e;
  PlanningPoint opt_point_f;
  PlanningPoint opt_point_g;
  PlanningPoint opt_point_h;
  PlanningPoint opt_point_i;

  double turn_radius_min = 5.5;
  double opt_radius_bc = turn_radius_min;
  double opt_radius_cd = turn_radius_min;
  double opt_radius_ef = turn_radius_min;
  double opt_radius_fg = turn_radius_min;
  double opt_radius_fh = turn_radius_min;
  double opt_radius_hi = turn_radius_min;

  double total_cost = std::numeric_limits<double>::infinity();
};

}  // namespace planning