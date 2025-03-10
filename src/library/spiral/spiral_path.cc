#include "spiral_path.h"

#include <cstddef>
#include <iostream>

#include "debug_info_log.h"
#include "log_glog.h"
namespace planning {
static const size_t max_iterations = 100;
static const double newton_tolerance = 1e-3;

const bool CvtCompactStateGlobalToLocal(
    spiral_path_point_t *local_state_target,
    const spiral_path_point_t *global_state_target,
    const spiral_path_point_t *base_state_reference) {
  double dx, dy, theta;

  if (local_state_target == NULL || global_state_target == NULL ||
      base_state_reference == NULL) {
    ILOG_ERROR
        << "local_state_target, global_state_target, and base_state_reference "
           "should not be NULL!";
  }

  dx = global_state_target->x - base_state_reference->x;
  dy = global_state_target->y - base_state_reference->y;
  theta = base_state_reference->theta;
  local_state_target->x = dx * cos(theta) + dy * sin(theta);
  local_state_target->y = dy * cos(theta) - dx * sin(theta);
  /* limit the theta to [-pi , pi] */
  local_state_target->theta = ThetaDiff(global_state_target->theta, theta);
  local_state_target->kappa = global_state_target->kappa;

  return true;
}

const bool CvtCompactStateLocalToGlobal(
    spiral_path_point_t *global_state_target,
    const spiral_path_point_t *local_state_target,
    const spiral_path_point_t *base_state_reference) {
  double theta;

  if (global_state_target == NULL || local_state_target == NULL ||
      base_state_reference == NULL) {
    ILOG_ERROR << "input pointers should not be NULL !";
  }

  theta = base_state_reference->theta;
  global_state_target->x = base_state_reference->x +
                           local_state_target->x * cos(theta) -
                           local_state_target->y * sin(theta);
  global_state_target->y = base_state_reference->y +
                           local_state_target->x * sin(theta) +
                           local_state_target->y * cos(theta);
  /* limit the theta to [-pi , pi] */
  global_state_target->theta =
      UnifyTheta(theta + local_state_target->theta, M_PI);
  global_state_target->kappa = local_state_target->kappa;

  return true;
}

const bool CubicKnotsToCoefficients(coefficient_cubic_t *coefficients,
                                    const knot_cubic_t *knots) {
  if (coefficients == NULL || knots == NULL) {
    ILOG_ERROR << "coefficients and knots should not be NULL!";
    return false;
  }

  if (!Fgreater(knots->sf, 0)) {
    // ILOG_ERROR << "final path length sf should be not less than 0!";
    return false;
  }

  coefficients->a = knots->p0;
  coefficients->sf = knots->sf;
  coefficients->b =
      (18 * knots->p1 + 2 * knots->p3 - 11 * knots->p0 - 9 * knots->p2) /
      (2 * knots->sf);
  coefficients->c =
      (18 * knots->p0 - 45 * knots->p1 + 36 * knots->p2 - 9 * knots->p3) /
      (2 * pow(knots->sf, 2));
  coefficients->d =
      (27 * knots->p1 - 27 * knots->p2 + 9 * knots->p3 - 9 * knots->p0) /
      (2 * pow(knots->sf, 3));

  return true;
}

const bool FThetaCubicByS(double *theta,
                          const coefficient_cubic_t *coefficients, double s) {
  double s2, s3, s4;

  if (coefficients == NULL || theta == NULL) {
    ILOG_ERROR << "coefficients and theta should not be NULL!";
    return false;
  }

  if (Fless(s, 0)) {
    ILOG_ERROR << "arc-length s should not be less than zero!";
    return false;
  }

  s2 = s * s;
  s3 = s2 * s;
  s4 = s3 * s;
  (*theta) = coefficients->a * s + coefficients->b * s2 / 2 +
             coefficients->c * s3 / 3 + coefficients->d * s4 / 4;

  return true;
}

const bool FKappaCubicS(double *kappa, const coefficient_cubic_t *coefficients,
                        double s) {
  double s2, s3;

  if (coefficients == NULL || kappa == NULL) {
    ILOG_ERROR << "coefficients and kappa should not be NULL!";
    return false;
  }

  if (Fless(s, 0)) {
    ILOG_ERROR << "arc-length s should not be less than zero!";
    return false;
  }

  s2 = s * s;
  s3 = s2 * s;
  (*kappa) = coefficients->a + coefficients->b * s + coefficients->c * s2 +
             coefficients->d * s3;

  return true;
}

// const bool SampleCubicSpiralMotionsByCoef(UOSSteerMotionList *motions,
//                                           const coefficient_cubic_t *coeffs,
//                                           const spiral_path_point_t *start,
//                                           double step_length) {
//   /* The max size for points is
//   MAX_SPIRAL_PATH_POINT_NUM x UOS_MAX_STEER_MOTION_NUM */
//   int32_t max_point_num, total_point_number = 0;
//   spiral_path_point_t local_state, global_state, key_state;
//   double delta_x = 0, delta_y = 0, delta_x1 = 0, delta_y1 = 0, theta = 0;
//   double theta1 = 0, s = 0, last_key_s = 0;
//   int32_t i, m = 0, n = 0;
//   bool ret = false;
//   double curvature_lower_bound, curvature_upper_bound;

//   if (motions == NULL || coeffs == NULL || start == NULL) {
//     ILOG_ERROR << "Input pointers should not be NULL!";
//     return false;
//   }

//   if (!Fgreater(step_length, 0.0)) {
//     ILOG_ERROR << "step length should be greater than zero";
//     return false;
//   }

//   total_point_number = (int32_t)(ceil(coeffs->sf / step_length)) + 1;
//   ILOG_INFO << "coeffs->sf " << coeffs->sf;
//   ILOG_INFO << "total_point_number " << total_point_number;
//   max_point_num = (MAX_SPIRAL_PATH_POINT_NUM - 1) * UOS_MAX_STEER_MOTION_NUM +
//   1;

//   if (total_point_number > max_point_num) {
//     ILOG_ERROR << "The sample number is greater than the motion size!";
//     return false;
//   }

//   /* get the curvature boundaries according to vehicle parameters */
//   if (!GetCurvatureBounds(&curvature_lower_bound, &curvature_upper_bound)) {
//     ILOG_ERROR << "GetCurvatureBounds_config() failed!";
//     return false;
//   }

//   motions->size = (int32_t)(ceil((double)total_point_number /
//                                  (double)MAX_SPIRAL_PATH_POINT_NUM));

//   if (total_point_number <= 1) {
//     ILOG_ERROR << "total_point_number should be greater than 1!";
//     return false;
//   }

//   /* the first key state is the start state */
//   key_state = *start;
//   for (size_t i = 1; i < total_point_number; i++) {
//     s = i * coeffs->sf / (total_point_number - 1);

//     if (!FThetaCubicByS(&theta, coeffs, s)) {
//       ILOG_ERROR << "Theta calculation of the spiral path failed!";
//       return false;
//     }

//     if (1 == i) {
//       /* cos(0) = 1.0,  sin(0) =0 */
//       delta_x = (cos(theta) + 1.0) / (2 * i);
//       delta_y = (sin(theta) + 0.0) / (2 * i);
//     } else {
//       delta_x = (cos(theta) + cos(theta1)) / (2 * i) + delta_x1 * (i - 1) /
//       i; delta_y = (sin(theta) + sin(theta1)) / (2 * i) + delta_y1 * (i - 1)
//       / i;
//     }
//     /* state in the frame of the start state */
//     local_state.x = delta_x * s;
//     local_state.y = delta_y * s;
//     local_state.theta = theta;

//     if (!FKappaCubicS(&(local_state.kappa), coeffs, s)) {
//       ILOG_ERROR << "kappa calculation failed!";
//       return false;
//     }

//     if (Fless(local_state.kappa, curvature_lower_bound) ||
//         Fgreater(local_state.kappa, curvature_upper_bound)) {
//       ILOG_ERROR << "the curvature of the solution is out of bounds!";
//       return false;
//     }

//     /* convert the local state to a global state */
//     if (!CvtCompactStateLocalToGlobal(&global_state, &local_state, start)) {
//       ILOG_ERROR << "transformation failed!";
//       return false;
//     }
//     /* segment index */
//     m = floor(i / (MAX_SPIRAL_PATH_POINT_NUM - 1));
//     ILOG_INFO << " i " << i;
//     /* check whether this is a start of a new segment */
//     n = i % (MAX_SPIRAL_PATH_POINT_NUM - 1);
//     /* this is a key state coupled with a control defined by coeffcients */
//     /* fill the UOSSteerMotionList */
//     if (0 == n && m > 0 && i < (total_point_number - 1)) {
//       motions->motions[m - 1].state = key_state;
//       motions->motions[m - 1].control.order = 3; /* cubic */
//       ret = TransformCubicSpiralCoeffs(
//           &(motions->motions[m - 1].control.coeffs[0]),
//           &(motions->motions[m - 1].control.coeffs[1]),
//           &(motions->motions[m - 1].control.coeffs[2]),
//           &(motions->motions[m - 1].control.coeffs[3]), last_key_s, coeffs);

//       if (!ret) {
//         ILOG_ERROR << "TransformCubicSpiralCoeffs() failed!";
//         return false;
//       }
//       motions->motions[m - 1].control.length = s - last_key_s;
//       key_state = global_state;
//       last_key_s = s;
//     }

//     /* the last segment */
//     if (i == (total_point_number - 1)) {
//       if (0 == n) {
//         motions->motions[m - 1].state = key_state;
//         motions->motions[m - 1].control.order = 3; /* cubic */
//         ret = TransformCubicSpiralCoeffs(
//             &(motions->motions[m - 1].control.coeffs[0]),
//             &(motions->motions[m - 1].control.coeffs[1]),
//             &(motions->motions[m - 1].control.coeffs[2]),
//             &(motions->motions[m - 1].control.coeffs[3]), last_key_s,
//             coeffs);
//         if (!ret) {
//           ILOG_ERROR << "TransformCubicSpiralCoeffs() failed!";
//           return false;
//         }
//         motions->motions[m - 1].control.length = s - last_key_s;
//       } else {
//         motions->motions[m].state = key_state;
//         motions->motions[m].control.order = 3; /* cubic */
//         ret = TransformCubicSpiralCoeffs(
//             &(motions->motions[m].control.coeffs[0]),
//             &(motions->motions[m].control.coeffs[1]),
//             &(motions->motions[m].control.coeffs[2]),
//             &(motions->motions[m].control.coeffs[3]), last_key_s, coeffs);
//         if (!ret) {
//           ILOG_ERROR << "TransformCubicSpiralCoeffs() failed!";
//           return false;
//         }
//         motions->motions[m].control.length = s - last_key_s;
//       }
//     }

//     delta_x1 = delta_x;
//     delta_y1 = delta_y;
//     theta1 = theta;
//   }

//   return true;
// }

// const bool SampleCubicSpiralMotionsBySol(UOSSteerMotionList *motions,
//                                          const solution_cubic_t *solution,
//                                          double step_length) {
//   bool ret = false;
//   coefficient_cubic_t coeffs;

//   if (motions == NULL || solution == NULL) {
//     ILOG_ERROR << "input pointers should not be NULL!";
//     return false;
//   }

//   if (!Fgreater(step_length, 0)) {
//     ILOG_ERROR << "step_length should not be less than zero!";
//     return false;
//   }

//   if (!CubicKnotsToCoefficients(&coeffs, &(solution->knots))) {
//     ILOG_ERROR << "CubicKnotsToCoefficients() failed!";
//     return false;
//   }

//   if (!SampleCubicSpiralMotionsByCoef(motions, &coeffs,
//   &(solution->base_state),
//                                       step_length)) {
//     ILOG_ERROR << "SampleCubicSpiralMotionsByCoef() failed!";
//     return false;
//   }

//   return true;
// }

const bool SampleCubicSpiralStatesByCoef(
    std::vector<spiral_path_point_t> &states, const coefficient_cubic_t *coeffs,
    const spiral_path_point_t *start, double step_length) {
  int32_t total_point_number;
  spiral_path_point_t local_state, global_state;
  double delta_x = 0, delta_y = 0, delta_x1 = 0, delta_y1 = 0, theta = 0;
  double theta1 = 0, s = 0;
  int32_t i;
  bool ret = false;
  double curvature_lower_bound, curvature_upper_bound;

  if (coeffs == NULL || start == NULL) {
    ILOG_ERROR << "coeffs and start should not be NULL!";
    return false;
  }

  if (!Fgreater(step_length, 0)) {
    // ILOG_ERROR << "step_length should not be less than zero!";
    return false;
  }

  if (!GetCurvatureBounds(&curvature_lower_bound, &curvature_upper_bound)) {
    return false;
  }

  total_point_number = (int32_t)(floor(coeffs->sf / step_length));

  states.clear();
  states.shrink_to_fit();
  states.reserve(total_point_number);
  /* push the first element */
  states.emplace_back(*start);
  for (i = 1; i <= total_point_number; i++) {
    s = i * coeffs->sf / total_point_number;

    if (!FThetaCubicByS(&theta, coeffs, s)) {
      // ILOG_ERROR << "theta calculation of the spiral path failed!";
      return false;
    }

    if (1 == i) {
      delta_x = (cos(theta) + cos(0)) / (2 * i);
      delta_y = (sin(theta) + sin(0)) / (2 * i);
    } else {
      delta_x = (cos(theta) + cos(theta1)) / (2 * i) + delta_x1 * (i - 1) / i;
      delta_y = (sin(theta) + sin(theta1)) / (2 * i) + delta_y1 * (i - 1) / i;
    }
    /* state in the frame of the start state */
    local_state.x = delta_x * s;
    local_state.y = delta_y * s;
    local_state.theta = theta;

    if (!FKappaCubicS(&(local_state.kappa), coeffs, s)) {
      // ILOG_ERROR << "curvature calculation failed!";
      return false;
    }

    if (Fless(local_state.kappa, curvature_lower_bound) ||
        Fgreater(local_state.kappa, curvature_upper_bound)) {
      // ILOG_ERROR << "the curvature of the solution is out of bounds!";
      return false;
    }

    /* convert the local state to a global state */
    CvtCompactStateLocalToGlobal(&global_state, &local_state, start);
    states.emplace_back(global_state);
    delta_x1 = delta_x;
    delta_y1 = delta_y;
    theta1 = theta;
  }

  return true;
}

const bool SampleCubicSpiralStatesBySol(
    std::vector<spiral_path_point_t> &states, const solution_cubic_t *solution,
    double step_length) {
  bool ret = false;
  coefficient_cubic_t coeffs;

  if (!CubicKnotsToCoefficients(&coeffs, &(solution->knots))) {
    return false;
  }

  if (!Fgreater(step_length, 0)) {
    // ILOG_ERROR << "step_length should not be less than zero!";
    return false;
  }

  if (!SampleCubicSpiralStatesByCoef(states, &coeffs, &(solution->base_state),
                                     step_length)) {
    return false;
  }

  return true;
}

const bool TransformCubicSpiralCoeffs(double *a, double *b, double *c,
                                      double *d, double shift_distance,
                                      const coefficient_cubic_t *coeffs) {
  if (coeffs == NULL || a == NULL || b == NULL || c == NULL || d == NULL) {
    ILOG_ERROR << "a, b, c, d and coeffs can not be NULL!";
    return false;
  }
  /* new a */
  *a = coeffs->a + coeffs->b * shift_distance +
       coeffs->c * shift_distance * shift_distance +
       coeffs->d * pow(shift_distance, 3);
  /* new b */
  *b = coeffs->b + 2 * coeffs->c * shift_distance +
       3 * coeffs->d * shift_distance * shift_distance;
  /* new c */
  *c = coeffs->c + 3 * coeffs->d * shift_distance;
  /* new d */
  *d = coeffs->d;

  return true;
}

extern const bool GetCurvatureBounds(double *lower_bound, double *upper_bound) {
  double curvature = 0.2;
  // vehicle_params_t *veh_params = get_vehicle_params();

  // CHECK_LOG_RET(UOS_MOD_PLANNER_BASE,
  //         lower_bound != NULL &&  upper_bound != NULL,
  //         "input pointer can not be null!\n", false);
  // CHECK_LOG_RET(UOS_MOD_PLANNER_BASE,
  //         Fgreater(veh_params->lfr_m, 0),
  //         "the wheelbase should be greater than zero!\n", false);
  // curvature =
  //         uos_tan(uos_deg2rad(veh_params->steering_angle_limit)) /
  //         veh_params->lfr_m;
  (*lower_bound) = -curvature;
  (*upper_bound) = curvature;

  return true;
}

const bool CubicSpiralStrictSolve(solution_cubic_t *solution,
                                  const spiral_path_point_t *start,
                                  const spiral_path_point_t *goal) {
  bool ret = false;
  // planner_base_config_t *config;
  double diff;
  int32_t i;
  spiral_path_point_t local_goal;
  /*  matrix variables for gradient descent  */
  Eigen::Matrix<double, 3, 1> state_final;
  /*  Jacobian matrix for the newton method */
  Eigen::Matrix<double, 3, 3> jacobi;
  /*  difference between the final goal state and intermediate state */
  Eigen::Matrix<double, 3, 1> delta_state;
  /*  parameter difference that represents the update step */
  Eigen::Matrix<double, 3, 1> delta_p;
  /*  the intermediate state  */
  Eigen::Matrix<double, 3, 1> state_guess;
  knot_cubic_t guess;

  if (solution == NULL || start == NULL || goal == NULL) {
    ILOG_ERROR << "input pointers should not be NULL!";
    return false;
  }

  /* limit the delta theta_g to [-pi, pi] */
  if (!CvtCompactStateGlobalToLocal(&local_goal, goal, start)) {
    // ILOG_ERROR << "input pointers should not be NULL!";
    return false;
  }

  guess.p0 = start->kappa;
  guess.p1 = 0.0;
  guess.p2 = 0.0;
  guess.p3 = local_goal.kappa;
  guess.sf = (local_goal.theta * local_goal.theta / 5.0 + 1.0) *
             sqrt(local_goal.x * local_goal.x + local_goal.y * local_goal.y);

  /*  the final goal state in local frame */
  state_final << local_goal.x, local_goal.y, local_goal.theta;

  solution->solve_status = SOLVE_STATUS_MAX_ITERATION;
  for (size_t i = 0; i < max_iterations; ++i) {
    /* intermediate states */
    state_guess << fx_cubic(guess), fy_cubic(guess), ftheta_cubic(guess);
    /* jacobian */
    jacobi(0, 0) = dx_cubic_dp1(guess);
    jacobi(0, 1) = dx_cubic_dp2(guess);
    jacobi(0, 2) = dx_cubic_dp4(guess);
    jacobi(1, 0) = dy_cubic_dp1(guess);
    jacobi(1, 1) = dy_cubic_dp2(guess);
    jacobi(1, 2) = dy_cubic_dp4(guess);
    jacobi(2, 0) = dtheta_cubic_dp1(guess);
    jacobi(2, 1) = dtheta_cubic_dp2(guess);
    jacobi(2, 2) = dtheta_cubic_dp4(guess);

    delta_state = state_final - state_guess;
    diff = fabs(delta_state(0)) + fabs(delta_state(1)) + fabs(delta_state(2));
    if (Fless(diff, newton_tolerance)) {
      solution->solve_status = SOLVE_STATUS_CONVERGE;
      break;
    }
    /* solve the equation by lu decomposition */
    delta_p = jacobi.lu().solve(delta_state);
    /* check whether there is a NAN value in parameter veoctor */
    if ((delta_p(0) != delta_p(0)) || (delta_p(1) != delta_p(1)) ||
        (delta_p(2) != delta_p(2))) {
      ILOG_INFO << "delta_p of cubic spirals has a NAN value at least!";
      return false;
    }

    /* update parameter guess by adding the delta p */
    guess.p1 += delta_p(0);
    guess.p2 += delta_p(1);
    guess.sf += delta_p(2);
  } /* for max_iterations loop */
  solution->base_state = *start;
  solution->knots = guess;
  // ILOG_INFO << " StrictSolve real_max_iterations " << i;

  return true;
}

const bool CubicSpiralEndkFreeSolve(solution_cubic_t *solution,
                                    const spiral_path_point_t *start,
                                    const spiral_path_point_t *goal) {
  bool ret = false;
  ;
  // planner_base_config_t *config;
  double diff;
  int32_t i;
  spiral_path_point_t local_goal;
  /*  matrix variables for gradient descent  */
  Eigen::Matrix<double, 3, 1> state_final;
  /*  Jacobian matrix for the newton method */
  Eigen::Matrix<double, 3, 4> jacobi;
  /*  difference between the final goal state and intermediate state */
  Eigen::Matrix<double, 3, 1> delta_state;
  /*  parameter difference that represents the update step */
  Eigen::Matrix<double, 4, 1> delta_p;
  /*  the intermediate state  */
  Eigen::Matrix<double, 3, 1> state_guess;
  /* svd setup */
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      3, 4, Eigen::ComputeThinU | Eigen::ComputeThinV);

  if (solution == NULL || start == NULL || goal == NULL) {
    ILOG_ERROR << "input pointers should not be NULL!";
    return false;
  }

  /* limit the delta theta_g to [-pi, pi] */
  if (!CvtCompactStateGlobalToLocal(&local_goal, goal, start)) {
    // ILOG_ERROR << "transformation failed!";
    return false;
  }

  knot_cubic_t guess;
  guess.p0 = start->kappa;
  guess.p1 = 0.0;
  guess.p2 = 0.0;
  guess.p3 = 0.0;
  guess.sf = (local_goal.theta * local_goal.theta / 5.0 + 1.0) *
             sqrt(local_goal.x * local_goal.x + local_goal.y * local_goal.y);

  /*  the final goal state in local frame */
  state_final << local_goal.x, local_goal.y, local_goal.theta;

  solution->solve_status = SOLVE_STATUS_MAX_ITERATION;
  for (i = 0; i < max_iterations; ++i) {
    /* intermediate states */
    state_guess << fx_cubic(guess), fy_cubic(guess), ftheta_cubic(guess);
    /* jacobian */
    jacobi(0, 0) = dx_cubic_dp1(guess);
    jacobi(0, 1) = dx_cubic_dp2(guess);
    jacobi(0, 2) = dx_cubic_dp3(guess);
    jacobi(0, 3) = dx_cubic_dp4(guess);

    jacobi(1, 0) = dy_cubic_dp1(guess);
    jacobi(1, 1) = dy_cubic_dp2(guess);
    jacobi(1, 2) = dy_cubic_dp3(guess);
    jacobi(1, 3) = dy_cubic_dp4(guess);

    jacobi(2, 0) = dtheta_cubic_dp1(guess);
    jacobi(2, 1) = dtheta_cubic_dp2(guess);
    jacobi(2, 2) = dtheta_cubic_dp3(guess);
    jacobi(2, 3) = dtheta_cubic_dp4(guess);

    delta_state = state_final - state_guess;
    diff = fabs(delta_state(0)) + fabs(delta_state(1)) + fabs(delta_state(2));
    if (Fless(diff, newton_tolerance)) {
      solution->solve_status = SOLVE_STATUS_CONVERGE;
      break;
    }

    /* solve the equation by svd decomposition */
    svd.compute(jacobi);
    delta_p = svd.solve(delta_state);

    /* check whether there is a NAN value in parameter veoctor */
    if ((delta_p(0) != delta_p(0)) || (delta_p(1) != delta_p(1)) ||
        (delta_p(2) != delta_p(2)) || (delta_p(3) != delta_p(3))) {
      ILOG_INFO << "delta_p of cubic spirals has a NAN value at least!";
      return false;
    }

    /* update parameter guess by adding the delta p */
    guess.p1 += delta_p(0);
    guess.p2 += delta_p(1);
    guess.p3 += delta_p(2);
    guess.sf += delta_p(3);
  } /* for max_iterations loop */
  solution->base_state = *start;
  solution->knots = guess;

  // ILOG_INFO << " EndkFreeSolve real_max_iterations " << i;

  return true;
}

const bool CubicSpiralStartkFreeSolve(solution_cubic_t *solution,
                                      const spiral_path_point_t *start,
                                      const spiral_path_point_t *goal) {
  bool ret = false;
  ;
  // planner_base_config_t *config;
  double diff;
  int32_t i;
  spiral_path_point_t local_goal;
  /*  matrix variables for gradient descent  */
  Eigen::Matrix<double, 3, 1> state_final;
  /*  Jacobian matrix for the newton method */
  Eigen::Matrix<double, 3, 4> jacobi;
  /*  difference between the final goal state and intermediate state */
  Eigen::Matrix<double, 3, 1> delta_state;
  /*  parameter difference that represents the update step */
  Eigen::Matrix<double, 4, 1> delta_p;
  /*  the intermediate state  */
  Eigen::Matrix<double, 3, 1> state_guess;
  /* svd setup */
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      3, 4, Eigen::ComputeThinU | Eigen::ComputeThinV);
  knot_cubic_t guess;

  if (solution == NULL || start == NULL || goal == NULL) {
    ILOG_INFO << "input pointers should not be NULL!";
    return false;
  }

  /* limit the delta theta_g to [-pi, pi] */
  if (!CvtCompactStateGlobalToLocal(&local_goal, goal, start)) {
    // ILOG_INFO << "transformation failed!";
    return false;
  }

  guess.p0 = 0.0;
  guess.p1 = 0.0;
  guess.p2 = 0.0;
  guess.p3 = goal->kappa;
  guess.sf = (local_goal.theta * local_goal.theta / 5.0 + 1.0) *
             sqrt(local_goal.x * local_goal.x + local_goal.y * local_goal.y);

  /*  the final goal state in local frame */
  state_final << local_goal.x, local_goal.y, local_goal.theta;
  solution->solve_status = SOLVE_STATUS_MAX_ITERATION;
  for (i = 0; i < max_iterations; ++i) {
    /* intermediate states */
    state_guess << fx_cubic(guess), fy_cubic(guess), ftheta_cubic(guess);
    /* jacobian */
    jacobi(0, 0) = dx_cubic_dp0(guess);
    jacobi(0, 1) = dx_cubic_dp1(guess);
    jacobi(0, 2) = dx_cubic_dp2(guess);
    jacobi(0, 3) = dx_cubic_dp4(guess);

    jacobi(1, 0) = dy_cubic_dp0(guess);
    jacobi(1, 1) = dy_cubic_dp1(guess);
    jacobi(1, 2) = dy_cubic_dp2(guess);
    jacobi(1, 3) = dy_cubic_dp4(guess);

    jacobi(2, 0) = dtheta_cubic_dp0(guess);
    jacobi(2, 1) = dtheta_cubic_dp1(guess);
    jacobi(2, 2) = dtheta_cubic_dp2(guess);
    jacobi(2, 3) = dtheta_cubic_dp4(guess);

    delta_state = state_final - state_guess;
    diff = fabs(delta_state(0)) + fabs(delta_state(1)) + fabs(delta_state(2));
    if (Fless(diff, newton_tolerance)) {
      solution->solve_status = SOLVE_STATUS_CONVERGE;
      break;
    }

    /* solve the equation by svd decomposition */
    svd.compute(jacobi);
    delta_p = svd.solve(delta_state);

    /* check whether there is a NAN value in parameter veoctor */
    if ((delta_p(0) != delta_p(0)) || (delta_p(1) != delta_p(1)) ||
        (delta_p(2) != delta_p(2)) || (delta_p(3) != delta_p(3))) {
      ILOG_INFO << "delta_p of cubic spirals has a NAN value at least!";
      return false;
    }

    /* update parameter guess by adding the delta p */
    guess.p0 += delta_p(0);
    guess.p1 += delta_p(1);
    guess.p2 += delta_p(2);
    guess.sf += delta_p(3);
  } /* for max_iterations loop */
  solution->base_state = *start;
  solution->knots = guess;
  // ILOG_INFO << " StartkFreeSolve real_max_iterations " << i;

  return true;
}

const bool CubicSpiralBothkFreeSolve(solution_cubic_t *solution,
                                     const spiral_path_point_t *start,
                                     const spiral_path_point_t *goal) {
  bool ret = false;
  ;
  // planner_base_config_t *config;
  double diff;
  int32_t i;
  spiral_path_point_t local_goal;
  /*  matrix variables for gradient descent  */
  Eigen::Matrix<double, 3, 1> state_final;
  /*  Jacobian matrix for the newton method */
  Eigen::Matrix<double, 3, 5> jacobi;
  /*  difference between the final goal state and intermediate state */
  Eigen::Matrix<double, 3, 1> delta_state;
  /*  parameter difference that represents the update step */
  Eigen::Matrix<double, 5, 1> delta_p;
  /*  the intermediate state  */
  Eigen::Matrix<double, 3, 1> state_guess;
  /* svd setup */
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      3, 5, Eigen::ComputeThinU | Eigen::ComputeThinV);
  knot_cubic_t guess;

  if (solution == NULL || start == NULL || goal == NULL) {
    ILOG_INFO << "input pointers should not be NULL!";
    return false;
  }

  /* limit the delta theta_g to [-pi, pi] */
  if (!CvtCompactStateGlobalToLocal(&local_goal, goal, start)) {
    // ILOG_INFO << "transformation failed!";
    return false;
  }

  guess.p0 = 0.0;
  guess.p1 = 0.0;
  guess.p2 = 0.0;
  guess.p3 = 0.0;
  guess.sf = (local_goal.theta * local_goal.theta / 5.0 + 1.0) *
             sqrt(local_goal.x * local_goal.x + local_goal.y * local_goal.y);

  /*  the final goal state in local frame */
  state_final << local_goal.x, local_goal.y, local_goal.theta;
  solution->solve_status = SOLVE_STATUS_MAX_ITERATION;
  for (i = 0; i < max_iterations; ++i) {
    /* intermediate states */
    state_guess << fx_cubic(guess), fy_cubic(guess), ftheta_cubic(guess);
    /* jacobian */
    jacobi(0, 0) = dx_cubic_dp0(guess);
    jacobi(0, 1) = dx_cubic_dp1(guess);
    jacobi(0, 2) = dx_cubic_dp2(guess);
    jacobi(0, 3) = dx_cubic_dp3(guess);
    jacobi(0, 4) = dx_cubic_dp4(guess);

    jacobi(1, 0) = dy_cubic_dp0(guess);
    jacobi(1, 1) = dy_cubic_dp1(guess);
    jacobi(1, 2) = dy_cubic_dp2(guess);
    jacobi(1, 3) = dy_cubic_dp3(guess);
    jacobi(1, 4) = dy_cubic_dp4(guess);

    jacobi(2, 0) = dtheta_cubic_dp0(guess);
    jacobi(2, 1) = dtheta_cubic_dp1(guess);
    jacobi(2, 2) = dtheta_cubic_dp2(guess);
    jacobi(2, 3) = dtheta_cubic_dp3(guess);
    jacobi(2, 4) = dtheta_cubic_dp4(guess);

    delta_state = state_final - state_guess;
    diff = fabs(delta_state(0)) + fabs(delta_state(1)) + fabs(delta_state(2));
    if (Fless(diff, newton_tolerance)) {
      solution->solve_status = SOLVE_STATUS_CONVERGE;
      break;
    }

    /* solve the equation by svd decomposition */
    svd.compute(jacobi);
    delta_p = svd.solve(delta_state);

    /* check whether there is a NAN value in parameter veoctor */
    if ((delta_p(0) != delta_p(0)) || (delta_p(1) != delta_p(1)) ||
        (delta_p(2) != delta_p(2)) || (delta_p(3) != delta_p(3)) ||
        (delta_p(4) != delta_p(4))) {
      ILOG_INFO << "delta_p of cubic spirals has a NAN value at least!";
      return false;
    }

    /* update parameter guess by adding the delta p */
    guess.p0 += delta_p(0);
    guess.p1 += delta_p(1);
    guess.p2 += delta_p(2);
    guess.p3 += delta_p(3);
    guess.sf += delta_p(4);
  } /* for max_iterations loop */
  solution->base_state = *start;
  solution->knots = guess;

  // ILOG_INFO << " BothkFreeSolve real_max_iterations " << i;

  return true;
}

// const bool CubicSpiralSolve(bool *solution_usable,
//                             uos_steer_motion_list_t *motions,
//                             const spiral_path_point_t *start,
//                             const spiral_path_point_t *goal, double
//                             step_length, bool constrain_start_k, bool
//                             constrain_goal_k) {
//   bool ret = false;
//   solution_cubic_t sol;

//   if (solution_usable == NULL || motions == NULL || start == NULL ||
//       goal == NULL) {
//     ILOG_INFO << "input pointers should not be NULL!";
//     return false;
//   }

//   if (constrain_start_k && constrain_goal_k) {
//     ret = CubicSpiralStrictSolve(&sol, start, goal);
//   } else if (!constrain_start_k && constrain_goal_k) {
//     ret = CubicSpiralStartkFreeSolve(&sol, start, goal);
//   } else if (constrain_start_k && !constrain_goal_k) {
//     ret = CubicSpiralEndkFreeSolve(&sol, start, goal);
//   } else /* both end free */
//   {
//     ret = CubicSpiralBothkFreeSolve(&sol, start, goal);
//   }

//   if (!ret) {
//     ILOG_INFO << "cubic spiral solve failed !";
//     return false;
//   }

//   *solution_usable = (bool)(sol.solve_status);
//   if (*solution_usable) /* usable */
//   {
//     if (!SampleCubicSpiralMotionsBySol(motions, &sol, step_length)) {
//       ILOG_INFO << "cubic spiral sampling failed !";
//       return false;
//     }
//   }
//   return true;
// }
}  // namespace planning