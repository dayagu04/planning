#pragma once

namespace planning {

typedef struct SpiralPathPoint {
  float x;
  float y;
  float theta;
  float kappa;           /* curvature of state */
} spiral_path_point_t;

typedef struct KnotCubic {
  float p0; /* curvature at s = 0 */
  float p1; /* curvature at s = 1/3 * sf */
  float p2; /* curvature at s = 2/3 * sf */
  float p3; /* curvature at s = sf */
  float sf; /* the total arc-length of a cubic spiral path */
} knot_cubic_t;

typedef enum SolveStatus {
  SOLVE_STATUS_MAX_ITERATION = 0,
  SOLVE_STATUS_CONVERGE = 1
} solve_status_t;

typedef struct SolutionCubic {
  /* the knot parameter representation of the solution in the frame of the
     base_state */
  knot_cubic_t knots;
  spiral_path_point_t base_state; /* the base state in global frame */
  solve_status_t solve_status;    /* the solve status of the gradient descent */
} solution_cubic_t;

typedef struct CoefficientCubic {
  /**
   * \kappa = a + b*s + c*s^2 +d*s^3 , s \in [0, sf]
   * */
  float a;
  float b;
  float c;
  float d;
  float sf;
} coefficient_cubic_t;

}  // namespace planning
