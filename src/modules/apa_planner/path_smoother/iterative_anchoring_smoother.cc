/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#include "apa_planner/path_smoother/iterative_anchoring_smoother.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>

#include "apa_planner/common/apa_cos_sin.h"
#include "apa_planner/common/apa_utils.h"
#include "apa_planner/common/geometry_planning_io.h"
#include "apa_planner/common/planning_log_helper.h"
#include "common/math/discrete_points_math.h"
#include "common/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/utils/file.h"
#include "ifly_time.h"

namespace planning {

using ::PlanningOutput::PlanningOutput;
using planning_math::FemPosDeviationSmoother;
using planning_math::LineSegment2d;
using planning_math::Polygon2d;
using planning_math::Vec2d;

IterativeAnchoringSmoother::IterativeAnchoringSmoother() {
  const std::string config_file =
      "/asw/planning/res/conf/iterative_anchoring_smoother_config.pb.txt";
  common::util::GetProtoFromFile(config_file, &iterative_anchoring_config_);
}

bool IterativeAnchoringSmoother::Smooth(
      const std::vector<LineSegment2d>& obstacles,
      PlanningOutput *const planning_output) {
  size_t traj_point_num = planning_output->trajectory().trajectory_points_size();
  if (traj_point_num < 4) {
    PLANNING_LOG << "reference points size smaller than four, smoother early "
        "returned" << std::endl;
    return false;
  }

  const uint64_t start_timestamp = IflyTime::Now_ms();

  obstacles_linesegments_vec_ = obstacles;

  const auto& traj_points = planning_output->trajectory().trajectory_points();
  motion_sign_ = traj_points[0].v() > 0.0 ? 1.0 : -1.0;

  // Interpolate the traj
  DiscretizedPath warm_start_path;
  double accumulated_s = 0.0;
  Vec2d last_path_point(traj_points[0].x(), traj_points[0].y());
  for (size_t i = 0; i < traj_point_num; ++i) {
    Vec2d cur_path_point(traj_points[i].x(), traj_points[i].y());
    accumulated_s += cur_path_point.DistanceTo(last_path_point);
    PathPoint path_point;
    path_point.set_x(traj_points[i].x());
    path_point.set_y(traj_points[i].y());
    path_point.set_theta(traj_points[i].heading_yaw());
    path_point.set_s(accumulated_s);
    warm_start_path.emplace_back(std::move(path_point));
    last_path_point = cur_path_point;
  }

  std::vector<std::pair<double, double>> interpolated_warm_start_point2ds;
  const double path_length = warm_start_path.Length();
  const double delta_s = path_length / (traj_point_num - 1);
  interpolated_warm_start_point2ds.reserve(traj_point_num);

  for (int i = 0; i < traj_point_num; ++i) {
    const double s = delta_s * i;
    const auto point2d = warm_start_path.Evaluate(s);
    interpolated_warm_start_point2ds.emplace_back(point2d.x, point2d.y);
  }

  // Reset path profile by discrete point heading and curvature estimation
  DiscretizedPath interpolated_warm_start_path;
  if (!SetPathProfile(*planning_output, interpolated_warm_start_point2ds,
      &interpolated_warm_start_path)) {
    PLANNING_LOG << "Set path profile fails" << std::endl;
    return false;
  }

  // Generate feasible bounds for each path point
  std::vector<double> bounds;
  if (!GenerateInitialBounds(interpolated_warm_start_path, &bounds)) {
    PLANNING_LOG << "Generate initial bounds failed" << std::endl;
    return false;
  }

  // Check initial path collision avoidance, if it fails, smoother assumption
  // fails. Try reanchoring
  if (CheckCollisionAvoidance(interpolated_warm_start_path)) {
    PLANNING_LOG << "Interpolated input path points colliding with obstacle"
        << std::endl;
    return false;
  }

  const uint64_t path_smooth_start_timestamp = IflyTime::Now_ms();
  // Smooth path to have smoothed x, y, phi, kappa and s
  DiscretizedPath smoothed_path_points;
  if (!SmoothPath(*planning_output, interpolated_warm_start_path, bounds,
      &smoothed_path_points)) {
    return false;
  }

  const uint64_t path_smooth_end_timestamp = IflyTime::Now_ms();
  const uint64_t path_smooth_diff =
      path_smooth_end_timestamp - path_smooth_start_timestamp;
  PLANNING_LOG << "iterative anchoring path smoother time: "
      << path_smooth_diff << " ms." << std::endl;

  UpdatePlanningOutput(smoothed_path_points, planning_output);

  const uint64_t end_timestamp = IflyTime::Now_ms();
  const uint64_t diff = end_timestamp - start_timestamp;
  PLANNING_LOG << "iterative anchoring smoother total time: "
      << diff << " ms." << std::endl;

  return true;
}

bool IterativeAnchoringSmoother::GenerateInitialBounds(
    const DiscretizedPath& path_points, std::vector<double>* initial_bounds) {
  // CHECK_NOTNULL(initial_bounds);
  initial_bounds->clear();
  const double default_bound = iterative_anchoring_config_.default_bound();
  initial_bounds->resize(path_points.size(), default_bound);
  return true;
}

bool IterativeAnchoringSmoother::SmoothPath(
    const PlanningOutput& planning_output,
    const DiscretizedPath& raw_path_points, const std::vector<double>& bounds,
    DiscretizedPath* smoothed_path_points) {
  std::vector<std::pair<double, double>> raw_point2d;
  const size_t point_num = raw_path_points.size();
  raw_point2d.reserve(point_num);
  for (const auto& path_point : raw_path_points) {
    raw_point2d.emplace_back(path_point.x, path_point.y);
  }
  std::vector<double> flexible_bounds(bounds);

  FemPosDeviationSmoother fem_pos_smoother(
      iterative_anchoring_config_.fem_pos_deviation_smoother_config());

  const size_t max_iteration_num = 50;

  std::vector<std::pair<double, double>> smoothed_point2d;
  smoothed_point2d.reserve(point_num);
  bool is_collision = true;
  size_t counter = 0;

  while (is_collision) {
    if (counter > max_iteration_num) {
      PLANNING_LOG << "Maximum iteration reached, path smoother early stops"
          << std::endl;
      return true;
    }

    AdjustPathBounds(&flexible_bounds);

    std::vector<double> opt_x;
    std::vector<double> opt_y;
    if (!fem_pos_smoother.Solve(raw_point2d, flexible_bounds, &opt_x, &opt_y)) {
      PLANNING_LOG << "Smoothing path fails" << std::endl;
      return false;
    }

    if (opt_x.size() < 2 || opt_y.size() < 2) {
      PLANNING_LOG << "Return by fem_pos_smoother is wrong. Size smaller than 2 "
          << std::endl;
      return false;
    }

    smoothed_point2d.clear();
    for (size_t i = 0; i < point_num; ++i) {
      smoothed_point2d.emplace_back(opt_x[i], opt_y[i]);
    }

    if (!SetPathProfile(planning_output, smoothed_point2d,
        smoothed_path_points)) {
      PLANNING_LOG << "Set path profile fails" << std::endl;
      return false;
    }

    is_collision = CheckCollisionAvoidance(*smoothed_path_points);

    PLANNING_LOG << "loop iteration number is " << counter << std::endl;
    ++counter;
  }

  return true;
}

bool IterativeAnchoringSmoother::CheckCollisionAvoidance(
    const DiscretizedPath& path_points) {
  const size_t path_points_size = path_points.size();
  const double front_buffer =
      motion_sign_ > 0.0 ? iterative_anchoring_config_.lon_buffer() : 0.0;
  const double rear_buffer =
      motion_sign_ < 0.0 ? iterative_anchoring_config_.lon_buffer() : 0.0;
  const double lat_buffer = iterative_anchoring_config_.lat_buffer();

  PlanningPoint point_0(
      path_points[0].x, path_points[0].y, path_points[0].theta);
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
        point_0, front_buffer, rear_buffer, lat_buffer));
  for (size_t i = 0; i < path_points_size; ++i) {
    PlanningPoint point_i(
        path_points[i].x, path_points[i].y, path_points[i].theta);
    Polygon2d ego_polygon(init_ego_polygon);
    const double rotate_angle = point_i.theta - point_0.theta;
    ego_polygon.RotateAndTranslate(Vec2d(point_0.x, point_0.y),
      apa_sin(rotate_angle), apa_cos(rotate_angle),
      Vec2d(point_i.x - point_0.x, point_i.y - point_0.y));
    for (const auto& obstacle_linesegment : obstacles_linesegments_vec_) {
      if (ego_polygon.HasOverlap(obstacle_linesegment)) {
        PLANNING_LOG << "point at " << i << " collied with LineSegment"
            << std::endl;
        return true;
      }
    }
  }

  return false;
}

void IterativeAnchoringSmoother::AdjustPathBounds(
    std::vector<double>* bounds) {
  const double collision_decrease_ratio =
      iterative_anchoring_config_.collision_decrease_ratio();

  for (auto& bound : *bounds) {
    bound *= collision_decrease_ratio;
  }

  // Anchor the end points to enforce the initial end end heading continuity and
  // zero kappa
  bounds->at(0) = 0.0;
  bounds->at(1) = 0.0;
  bounds->at(bounds->size() - 1) = 0.0;
  bounds->at(bounds->size() - 2) = 0.0;
}

bool IterativeAnchoringSmoother::SetPathProfile(
    const PlanningOutput& planning_output,
    const std::vector<std::pair<double, double>>& point2d,
    DiscretizedPath* raw_path_points) {
  const size_t points_size = point2d.size();
  raw_path_points->clear();
  raw_path_points->reserve(points_size);
  // Compute path profile
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> accumulated_s;
  if (!ComputePathProfile(planning_output,
      point2d, &headings, &accumulated_s, &kappas)) {
    return false;
  }

  // Load into path point
  for (size_t i = 0; i < points_size; ++i) {
    PathPoint path_point;
    path_point.set_x(point2d[i].first);
    path_point.set_y(point2d[i].second);
    path_point.set_theta(headings[i]);
    path_point.set_s(accumulated_s[i]);
    path_point.set_kappa(kappas[i]);
    raw_path_points->push_back(std::move(path_point));
  }
  return true;
}

bool IterativeAnchoringSmoother::ComputePathProfile(
    const PlanningOutput& planning_output,
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<double>* headings, std::vector<double>* accumulated_s,
    std::vector<double>* kappas) const {
  headings->clear(); // heading means orientation of the car
  accumulated_s->clear();
  kappas->clear();

  const int points_size = xy_points.size();
  if (points_size < 4) {
    return false;
  }

  headings->reserve(points_size);
  accumulated_s->reserve(points_size);
  kappas->reserve(points_size);

  const auto& traj_pts = planning_output.trajectory().trajectory_points();
  const int end_index = points_size - 1;
  headings->push_back(traj_pts[0].heading_yaw());
  for (int i = 1; i < end_index; ++i) {
    const double x_delta = (xy_points[i].first - xy_points[i - 1].first);
    const double y_delta = (xy_points[i].second - xy_points[i - 1].second);
    const double raw_heading = std::atan2(y_delta, x_delta);
    const double heading = motion_sign_ > 0.0 ? raw_heading :
        planning_math::NormalizeAngle(raw_heading + M_PI);
    headings->push_back(heading);
  }
  headings->push_back(traj_pts[end_index].heading_yaw());

  double distance = 0.0;
  accumulated_s->push_back(distance);
  double fx = xy_points[0].first;
  double fy = xy_points[0].second;
  double nx = 0.0;
  double ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = xy_points[i].first;
    ny = xy_points[i].second;
    const double end_segment_s = std::hypot(fx - nx, fy - ny);
    accumulated_s->push_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  kappas->push_back(traj_pts[0].curvature());
  kappas->push_back(traj_pts[1].curvature());
  for (int i = 2; i < points_size - 2; ++i) {
    const double heading_diff = (*headings)[i + 1] - (*headings)[i - 1];
    const double abs_kappa = std::fabs(
        heading_diff / ((*accumulated_s)[i + 1] - (*accumulated_s)[i - 1]));
    // kappa > 0 for turn left
    if (heading_diff * motion_sign_ >= 0.0) {
      kappas->push_back(abs_kappa);
    } else {
      kappas->push_back(-abs_kappa);
    }
  }
  kappas->push_back(traj_pts[end_index - 1].curvature());
  kappas->push_back(traj_pts[end_index].curvature());

  return true;
}

bool IterativeAnchoringSmoother::UpdatePlanningOutput(
    const DiscretizedPath& raw_path_points,
    PlanningOutput* const planning_output) const {
  const size_t points_size = raw_path_points.size();
  auto trajectory = planning_output->mutable_trajectory();

  for (size_t i = 0; i < points_size; ++i) {
    auto trajectory_point =
        planning_output->mutable_trajectory()->mutable_trajectory_points(i);
    trajectory_point->set_x(raw_path_points[i].x);
    trajectory_point->set_y(raw_path_points[i].y);
    trajectory_point->set_heading_yaw(raw_path_points[i].theta);
    trajectory_point->set_curvature(raw_path_points[i].kappa);
    trajectory_point->set_distance(raw_path_points[i].s);
  }
  return true;
}

}  // namespace planning
