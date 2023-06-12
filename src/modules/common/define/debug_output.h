#pragma once
#include <cstdint>
#include <string>

#include "common.pb.h"
#include "planning_debug_info.pb.h"
#include "planning_plan.pb.h"

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

struct DebugOutput {
  // 自车位置
  Common::Point2d ego_position;
  // 横向信息
  Polyline left_lane;
  Polyline centre_lane;
  Polyline right_lane;
  Polyline target_lane;
  Polyline fix_lane;
  // 纵向信息
  Common::Point2d velocity;
  PlanningOutput::AccelerationRange acceleration;
  planning::common::PlanningDebugInfo data;
};
}  // namespace planning