#pragma once

#include <limits>

#include "apa_planner/common/vehicle_param_helper.h"

namespace planning {

struct PlanningPoint {
  PlanningPoint() = default;
  PlanningPoint(const double _x, const double _y, const double _theta) : x(_x), y(_y), theta(_theta) {}
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

  double opt_radius_bc = VehicleParamHelper::Instance()->GetParam().min_turn_radius();
  double opt_radius_cd = VehicleParamHelper::Instance()->GetParam().min_turn_radius();
  double opt_radius_de = VehicleParamHelper::Instance()->GetParam().min_turn_radius();
  double opt_radius_ef = VehicleParamHelper::Instance()->GetParam().min_turn_radius();

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

  double opt_radius_bc = VehicleParamHelper::Instance()->GetParam().min_turn_radius();
  double opt_radius_cd = VehicleParamHelper::Instance()->GetParam().min_turn_radius();
  double opt_radius_ef = VehicleParamHelper::Instance()->GetParam().min_turn_radius();
  double opt_radius_fg = VehicleParamHelper::Instance()->GetParam().min_turn_radius();
  double opt_radius_fh = VehicleParamHelper::Instance()->GetParam().min_turn_radius();
  double opt_radius_hi = VehicleParamHelper::Instance()->GetParam().min_turn_radius();

  double total_cost = std::numeric_limits<double>::infinity();
};

}  // namespace planning