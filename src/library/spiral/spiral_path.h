#ifndef _SPIRAL_PATH_H__
#define _SPIRAL_PATH_H__

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <vector>

#include "math_matrix.h"
#include "spiral_typedefs.h"
namespace planning {
#define MAX_SPIRAL_PATH_POINT_NUM (400)

/**
 *  \brief convert the global_state to a local state in the frame of the
 *         base_state. Can't use cvt_pose_global_to_local() since the curvture
 *         information is lost. cvt_state_global_to_local() is too heavy. Many
 *         other unrelated fields need to be copied over. This function will be
 *         used frequently.
 *
 *  Note: this function need to be moved to uos_core or uos_base. Put here
 *  temporally for quick implementation.

 *  \param[out]  local_state_target  State in a local frame
 *  \param[in]  global_state_target  State in global frame
 *  \param[in] base_state_reference  The state of the reference frame in global
 *                                   frame
 *  \return 0 -- Success; -1 -- Error
 **/
const bool CvtCompactStateGlobalToLocal(
    spiral_path_point_t *local_state_target,
    const spiral_path_point_t *global_state_target,
    const spiral_path_point_t *base_state_reference);

/**
 *  \brief Convert the local state in the frame of the base_state to a global
 *         state
 *
 *  \param[out] global_state_target  State in a global frame
 *  \param[in]   local_state_target  State in the local frame that the origin is
 *                                   base_state_reference
 *  \param[in] base_state_reference  The state of the reference frame in global
 *                                   frame
 *  \return 0 -- Success; -1 -- Error
 **/
const bool CvtCompactStateLocalToGlobal(
    spiral_path_point_t *global_state_target,
    const spiral_path_point_t *local_state_target,
    const spiral_path_point_t *base_state_reference);

/**
 *  \brief knots vector to coefficient vector for the cubic spiral
 *         \kappa = a + bs + cs^2 +ds^3,  s \in (0, sf]
 *
 *  \param[out] coefficients The coefficients of the cubic spiral,a,b,c,d,sf
 *  \param[in]         knots The knots vector represented by curvatures and the
 *                           final arc-lengh.
 *  \return 0 -- Success; -1 -- Error
 **/
const bool CubicKnotsToCoefficients(coefficient_cubic_t *coefficients,
                                    const knot_cubic_t *knots);

/**
 *  \brief Calculate the theta according to a given coefficient vector and s for
 *         a cubic spiral, where s \in (0, sf].
 *
 *  \param[out]       theta The heading of the vehicle
 *  \param[in] coefficients The coeffcient representation of a cubic spiral.
 *  \param[in]            s The arc-length of a point along the cubic spiral.
 * */
const bool FThetaCubicByS(double *theta,
                          const coefficient_cubic_t *coefficients, double s);

/**
 *  \brief Calculate the curvature according to a given coefficient vector and s
 *         for a cubic spiral, where s \in (0, sf]
 *
 *  \param[out]       kappa The curvature at s position along the spiral curve.
 *  \param[in] coefficients The coefficients representation of the cubic sprial
 *                          curve.
 *  \param[in]            s The arc-length of a given point on the curve.
 **/
const bool FKappaCubicS(double *kappa, const coefficient_cubic_t *coefficients,
                        double s);

/**
 *  \brief Generate (x, y, theta, kappa) vector according to the coefficient
 *         representation of a cubic spiral and step_length by using iterative
 *         trapezoidal method.
 *
 *  \param[out]     states The states vector
 *  \param[in]      coeffs The coefficients representation of a cubic sprial
 *                         curve.
 *  \param[in]       start The global state where the cubic spiral starts.
 *  \param[in] step_length The sampling step length along the curve.
 **/
const bool SampleCubicSpiralStatesByCoef(
    std::vector<spiral_path_point_t> &states, const coefficient_cubic_t *coeffs,
    const spiral_path_point_t *start, double step_length);

/**
 *  \brief Generate (x, y, theta, kappa) state vector according to the knots
 *         representation of a cubic spiral and step_length by using iterative
 *         trapezoidal method.
 *
 *  \param[out]     states The states vector
 *  \param[in]    solution The solution of a cubic sprial curve in knots form.
 *  \param[in] step_length The sampling step length along the curve.
 **/
const bool SampleCubicSpiralStatesBySol(
    std::vector<spiral_path_point_t> &states, const solution_cubic_t *solution,
    double step_length);

/**
 *  \brief Generate a motion list according to the coefficient representation of
 *         a cubic spiral and step_length by using iterative trapezoidal method.
 *
 *  \param[out]    motions The motion list.
 *  \param[in]      coeffs The coefficients representation of a cubic sprial
 *                         curve.
 *  \param[in]       start The global state where the cubic spiral starts.
 *  \param[in] step_length The sampling step length along the curve.
 **/
// const bool SampleCubicSpiralMotionsByCoef(UOSSteerMotionList *motions,
//                                           const coefficient_cubic_t *coeffs,
//                                           const spiral_path_point_t *start,
//                                           double step_length);
// /**
//  *  \brief Generate a motion list according to the coefficient representation
//  of
//  *         a cubic spiral and step_length by using iterative trapezoidal
//  method.
//  *
//  *  \param[out]    motions The motion list.
//  *  \param[in]      coeffs The coefficients representation of a cubic sprial
//  *                         curve.
//  *  \param[in]       start The global state where the cubic spiral starts.
//  *  \param[in] step_length The sampling step length along the curve.
//  **/
// const bool SampleCubicSpiralMotionsBySol(UOSSteerMotionList *motions,
//                                          const solution_cubic_t *solution,
//                                          double step_length);

/**
 *  \brief Given the shift distance and the original coefficient representation
 *         calculate the new parameteric representation of the cubic spiral.
 *
 *         kappa = a + b*s + c*s^2 + ds^3
 *
 *  \param[out]             a The coefficients representation of a cubic sprial.
 *  \param[out]             b The coefficients representation of a cubic sprial.
 *  \param[out]             c The coefficients representation of a cubic sprial.
 *  \param[out]             d The coefficients representation of a cubic sprial.
 *  \param[in] shift_distance The shifted distance in arclength i
 *  \param[in]         coeffs The original coefficients of the sprial.
 **/
const bool TransformCubicSpiralCoeffs(double *a, double *b, double *c,
                                      double *d, double shift_distance,
                                      const coefficient_cubic_t *coeffs);

/**
 *  \brief Calculate the lower and upper bounds of curvatures according to
 *         vehecle paramters.
 *
 *  \param[out] lower_bound The min signed curvature, negtive value
 *  \param[out] upper_bound The max signed curvature, positive value
 *  \return 0 -- Success; -1 -- Error
 **/
const bool GetCurvatureBounds(double *lower_bound, double *upper_bound);
/**
 *  \brief Cubic spiral path generation function based on gradient descent
 *         method.
 *
 *         Given the start state and goal state, generate a cubic spiral path
 *         solution in knot vector form that connects these two states based on
 *         the gradient descent method.
 *         (x_start, y_start, theta_start, kappa_start) and
 *         (x_goal, y_goal, theta_goal, kappa_goal) are fully constrained.
 *
 *  \param[out] solution  A spiral path solution in knot vector form.
 *  \param[in]     start  Start state in global frame.
 *  \param[in]      goal  Goal state in global frame.
 *  \return 0 -- Success; -1 -- Error
 **/
const bool CubicSpiralStrictSolve(solution_cubic_t *solution,
                                  const spiral_path_point_t *start,
                                  const spiral_path_point_t *goal);

/**
 *  \brief Cubic spiral path generation function based on gradient descent
 *         method.
 *
 *         Given the start state and goal state, generate a cubic spiral path
 *         solution in knot vector form that connects these two states based on
 *         the gradient descent method.
 *         (x_start, y_start, theta_start, kappa_start) and
 *         (x_goal, y_goal, theta_goal) are constrained. The curvature of the
 *         final state is free.
 *
 *  \param[out]  solution  A spiral path solution in knot vector form.
 *  \param[in]      start  Start state in global frame.
 *  \param[in]       goal  Goal state in global frame.
 *  \return 0 -- Success; -1 -- Error
 **/
const bool CubicSpiralEndkFreeSolve(solution_cubic_t *solution,
                                    const spiral_path_point_t *start,
                                    const spiral_path_point_t *goal);

/**
 *  \brief Cubic spiral path generation function based on gradient descent
 *         method.
 *
 *         Given the start state and goal state, generate a cubic spiral path
 *         solution in knot vector form that connects these two states based on
 *         the gradient descent method.
 *         (x_start, y_start, theta_start) and
 *         (x_goal, y_goal, theta_goal, kappa_goal) are constrained.
 *         The curvature of the start state is free.
 *
 *  \param[out]  solution  A spiral path solution in knot vector form.
 *  \param[in]      start  Start state in global frame.
 *  \param[in]       goal  Goal state in global frame.
 *  \return 0 -- Success; -1 -- Error
 **/
const bool CubicSpiralStartkFreeSolve(solution_cubic_t *solution,
                                      const spiral_path_point_t *start,
                                      const spiral_path_point_t *goal);

/**
 *  \brief Cubic spiral path generation function based on gradient descent
 *         method.
 *
 *         Given the start state and goal state, generate a cubic spiral path
 *         solution in knot vector form that connects these two states based on
 *         the gradient descent method.
 *         (x_start, y_start, theta_start) and
 *         (x_goal, y_goal, theta_goal) are constrained. The curvature of the
 *         start and goal states are free.
 *
 *  Note: Since this case is an under-constrained condition, the curve itself
 *has freedoms to deform according to initial guess. The generated curve is not
 *as flat as the first three cases. The smoothness need to be taken into
 *consideration to improve the quality of the solution.
 *
 *  \param[out] solution  A spiral path solution in knot vector form.
 *  \param[in]     start  Start state in global frame.
 *  \param[in]      goal  Goal state in global frame.
 *  \return 0 -- Success; -1 -- Error
 **/
const bool CubicSpiralBothkFreeSolve(solution_cubic_t *solution,
                                     const spiral_path_point_t *start,
                                     const spiral_path_point_t *goal);

// /**
//  *  \brief Cubic spiral path generation function based on gradient descent
//  *         method.
//  *
//  *         Given the start state and goal state, generate a cubic spiral path
//  *         solution in knot vector form that connects these two states based
//  on
//  *         the gradient descent method.
//  *
//  *         This is a convenient function wrapper for above four cases.
//  *
//  *  \param[out]  solution_usable Whether the solution is usable
//  *  \param[out]          motions The solution represented by motions.
//  *  \param[in]             start Start state in global frame.
//  *  \param[in]              goal Goal state in global frame.
//  *  \param[in]       step_length The sampling step length along the spiral
//  path
//  *  \param[in] constrain_start_k Whether to constrained the curvature of
//  start
//  *  \param[in]  constrain_goal_k Whether to constrained the curvature of goal
//  *  \return 1 -- Success; -0 -- Error
//  **/
// const bool CubicSpiralSolve(bool *solution_usable,
//                             uos_steer_motion_list_t *motions,
//                             const spiral_path_point_t *start,
//                             const spiral_path_point_t *goal, double
//                             step_length, bool constrain_start_k, bool
//                             constrain_goal_k);

inline double ThetaDiff(double theta1, double theta2) {
  double diff = theta1 - theta2;
  if (diff > M_PI) diff -= 2 * M_PI;
  if (diff < -M_PI) diff += 2 * M_PI;
  return diff;
}

inline double UnifyTheta(double theta, double pi) {
  double diff = fmod(theta, 2 * pi);
  if (diff > pi) diff -= 2 * pi;
  if (diff < -pi) diff += 2 * pi;
  return diff;
}

inline bool Fgreater(double a, double b) { return a > b; }

inline bool Fless(double a, double b, double epsilon = 1e-6) {
  return (a - b) < -epsilon;
}
}  // namespace planning

#endif /* !_SPIRAL_PATH_H__ */
