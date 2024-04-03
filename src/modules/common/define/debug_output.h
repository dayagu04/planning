#pragma once

namespace planning {

struct Polyline {
  // y = ax^3 + bx^2 + cx +d
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double d = 0.0;
  double vel = 10.0;
  double t = 5.0;  // 5.0s
  double dt = 0.1;
  double corri_l = 0.5;  // corridor size = 0.5m
};

}  // namespace planning