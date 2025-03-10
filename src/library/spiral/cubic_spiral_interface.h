#pragma once

#include "spiral_path.h"

namespace planning {

class CubicSpiralInterface {
 public:
  CubicSpiralInterface() = default;

  /**
   *  \brief Cubic spiral path generation function based on gradient descent
   *         method.
   *
   *         Given the start state and goal state, generate a cubic spiral path
   *         solution in knot vector form that connects these two states based
   *         on the gradient descent method.
   *
   *         This is a convenient function wrapper for above four cases.
   *
   *  \param[out]  solution_usable Whether the solution is usable
   *  \param[out]          motions The solution represented by motions.
   *  \param[in]             start Start state in global frame.
   *  \param[in]              goal Goal state in global frame.
   *  \param[in]       step_length The sampling step length along the spiral
   *path \param[in] constrain_start_k Whether to constrained the curvature of
   *start \param[in]  constrain_goal_k Whether to constrained the curvature of
   *goal \return 1 -- Success; -0 -- Error
   **/
  const bool CubicSpiralStatesSolve(bool *solution_usable,
                                    std::vector<spiral_path_point_t> &states,
                                    const spiral_path_point_t *start,
                                    const spiral_path_point_t *goal,
                                    double step_length, bool constrain_start_k,
                                    bool constrain_goal_k);

  /**
   *  \brief Cubic spiral path generation function based on
   *         CubicSpiralStrictSolve
   *
   *         Given the start state and goal state, generate a cubic spiral path
   *         solution in knot vector form that connects these two states based
   *         on the gradient descent method.
   *
   *
   *  \param[out]  solution_usable Whether the solution is usable
   *  \param[out]          states The solution represented by states.
   *  \param[in]             start Start state in global frame.
   *  \param[in]              goal Goal state in global frame.
   *  \param[in]       step_length The sampling step length along the spiral
   *goal \return 1 -- Success; -0 -- Error
   **/
  const bool GenerateCubicSpiralPathByStrictSolve(
      solution_cubic_t *solution, std::vector<spiral_path_point_t> &states,
      const spiral_path_point_t *start, const spiral_path_point_t *goal,
      const double step_length);
};

}  // namespace planning