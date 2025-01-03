#ifndef _MATH_MATRIX_
#define _MATH_MATRIX_

#include "spiral_typedefs.h"
namespace planning {
/**  p0 = \kappa(0)
 *   p1 = \kappa(1/3*sf)
 *   p2 = \kappa(2/3*sf)
 *   p3 = \kappa(sf)
 *   p4 = sf
 *   where the \kappa is the curvature function about s, sf is the total
 *   arc-length of the spiral curve. p1, p2, p3 are curvatures.
 *   Gradient matrix of final states that are represented by parameter names
 *
 *   Gradient  =      dx/dp1       dx/dp2       dx/dp3        dx/dp4
 *                    dy/dp1       dy/dp2       dy/dp3        dy/dp4
 *                dtheta/dp1   dtheta/dp2   dtheta/dp3    dtheta/dp4
 *                dkappa/dp1   dkappa/dp2   dkappa/dp3    dkappa/dp4
 *
 *   dkappa/dp1 = dkappa/dp2 = dkappa/dp4 = 0, dkappa/dp3 = 1
 * */

double fx_cubic(knot_cubic_t param);
double fy_cubic(knot_cubic_t param);
double ftheta_cubic(knot_cubic_t param);

double dx_cubic_dp0(knot_cubic_t param);
double dx_cubic_dp1(knot_cubic_t param);
double dx_cubic_dp2(knot_cubic_t param);
double dx_cubic_dp3(knot_cubic_t param);
double dx_cubic_dp4(knot_cubic_t param);

double dy_cubic_dp0(knot_cubic_t param);
double dy_cubic_dp1(knot_cubic_t param);
double dy_cubic_dp2(knot_cubic_t param);
double dy_cubic_dp3(knot_cubic_t param);
double dy_cubic_dp4(knot_cubic_t param);

double dtheta_cubic_dp0(knot_cubic_t param);
double dtheta_cubic_dp1(knot_cubic_t param);
double dtheta_cubic_dp2(knot_cubic_t param);
double dtheta_cubic_dp3(knot_cubic_t param);
double dtheta_cubic_dp4(knot_cubic_t param);

/**
 *   Hessain matrix of final states that are represented by parameter names
 *
 *   Hessian  =
 *   (df(.))^2/dp1dp1 (df(.))^2/dp1dp2  (df(.))^2/dp1dp3  (df(.))^2/dp1dp4
 *   (df(.))^2/dp2dp1 (df(.))^2/dp2dp2  (df(.))^2/dp2dp3  (df(.))^2/dp2dp4
 *   (df(.))^2/dp3dp1 (df(.))^2/dp3dp2  (df(.))^2/dp3dp3  (df(.))^2/dp3dp4
 *   (df(.))^2/dp4dp1 (df(.))^2/dp4dp2  (df(.))^2/dp4dp3  (df(.))^2/dp4dp4
 *
 *   In the following implemention, the entry number
 *   (for example, 21 -> (df(x))^2 / dp2dp1) of the matrix is used instead of
 *   parameter names.
 *
 *  Hessian =    11                   12               13                   14
 *               21                   22               23                   24
 *               31                   32               33                   34
 *               41                   42               43                   44
 *
 *   Hessain matrix itself is symmetric, that is,
 *   (df(x))^2 / dp2dp1 = (df(x))^2 / dp1dp2
 * */
double dx_cubic_d11(knot_cubic_t param);
double dx_cubic_d21(knot_cubic_t param);
double dx_cubic_d22(knot_cubic_t param);
double dx_cubic_d31(knot_cubic_t param);
double dx_cubic_d32(knot_cubic_t param);
double dx_cubic_d33(knot_cubic_t param);
double dx_cubic_d41(knot_cubic_t param);
double dx_cubic_d42(knot_cubic_t param);
double dx_cubic_d43(knot_cubic_t param);
double dx_cubic_d44(knot_cubic_t param);

double dy_cubic_d11(knot_cubic_t param);
double dy_cubic_d21(knot_cubic_t param);
double dy_cubic_d22(knot_cubic_t param);
double dy_cubic_d31(knot_cubic_t param);
double dy_cubic_d32(knot_cubic_t param);
double dy_cubic_d33(knot_cubic_t param);
double dy_cubic_d41(knot_cubic_t param);
double dy_cubic_d42(knot_cubic_t param);
double dy_cubic_d43(knot_cubic_t param);
double dy_cubic_d44(knot_cubic_t param);

double dtheta_cubic_d11(knot_cubic_t param);
double dtheta_cubic_d21(knot_cubic_t param);
double dtheta_cubic_d22(knot_cubic_t param);
double dtheta_cubic_d31(knot_cubic_t param);
double dtheta_cubic_d32(knot_cubic_t param);
double dtheta_cubic_d33(knot_cubic_t param);
double dtheta_cubic_d41(knot_cubic_t param);
double dtheta_cubic_d42(knot_cubic_t param);
double dtheta_cubic_d43(knot_cubic_t param);
double dtheta_cubic_d44(knot_cubic_t param);
}  // namespace planning
#endif /* !_MATH_MATRIX_ */
