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

#pragma once

#include <utility>
#include <vector>

#include "iterative_anchoring_config.pb.h"
#include "planning_plan.pb.h"

#include "vec2d.h"
#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/path/discretized_path.h"

namespace planning {

class IterativeAnchoringSmoother {
 public:
  IterativeAnchoringSmoother();

  ~IterativeAnchoringSmoother() = default;

  bool Smooth(const std::vector<planning_math::LineSegment2d>& obstacles,
      ::PlanningOutput::PlanningOutput *const planning_output);

 private:
  bool GenerateInitialBounds(const DiscretizedPath& path_points,
                             std::vector<double>* initial_bounds);

  bool SmoothPath(const ::PlanningOutput::PlanningOutput& planning_output,
      const DiscretizedPath& raw_path_points, const std::vector<double>& bounds,
      DiscretizedPath* smoothed_path_points);

  bool CheckCollisionAvoidance(const DiscretizedPath& path_points);

  void AdjustPathBounds(std::vector<double>* bounds);

  bool SetPathProfile(const ::PlanningOutput::PlanningOutput& planning_output,
      const std::vector<std::pair<double, double>>& point2d,
      DiscretizedPath* raw_path_points);

  bool ComputePathProfile(
      const ::PlanningOutput::PlanningOutput& planning_output,
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings, std::vector<double>* accumulated_s,
      std::vector<double>* kappas) const;

  bool UpdatePlanningOutput(const DiscretizedPath& raw_path_points,
      ::PlanningOutput::PlanningOutput* const planning_output) const;

 private:
  std::vector<planning_math::LineSegment2d> obstacles_linesegments_vec_;

  std::vector<size_t> input_colliding_point_index_;

  bool enforce_initial_kappa_ = true;

  IterativeAnchoringConfig iterative_anchoring_config_;

  double motion_sign_ = 0.0; // 1.0 means move forward, -1.0 means move backward
};

}  // namespace planning
