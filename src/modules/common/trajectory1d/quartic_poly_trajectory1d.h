/** quartic_polynomial.h
 *
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University
 * of Singapore
 *
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 *
 * Defination of Quartic Polynomial
 */

#ifndef QUARTIC_POLYNOMIAL_H_
#define QUARTIC_POLYNOMIAL_H_

#include <vector>

#include "Eigen/Dense"

namespace planning {
struct QuarticPolyState {
  double p;
  double v;
  double a;

  double T;
};

class QuarticPolynomial {
 public:
  // Constructor
  QuarticPolynomial(const std::vector<double>& start,
                    const std::vector<double>& end, double T);
  QuarticPolynomial(const QuarticPolyState& start,
                    const QuarticPolyState& end);  // p, v, a, T
  QuarticPolynomial() = default;

  // Destructor
  virtual ~QuarticPolynomial(){};

  // calculate the s/d coordinate of a point
  double CalculatePoint(const double t) const;

  double CalculateFirstDerivative(const double t) const;

  double CalculateSecondDerivative(const double t) const;

  double CalculateThirdDerivative(const double t) const;

  void FindFirstDerivativeExtrema(const double te);

  void FindSecondDerivativeExtrema(const double te);

  void FindThirdDerivativeExtrema(const double te);

  std::vector<double> SolveQuadratic(double a, double b, double c) const;

  const std::pair<double, double> vel_extrema() const {
    return first_derv_extrema_;
  }
  const std::pair<double, double> acc_extrema() const {
    return second_derv_extrema_;
  }
  const std::pair<double, double> jerk_extrema() const {
    return third_derv_extrema_;
  }

  const std::vector<double>& coefficients() const { return coefficients_; };
  const double T() const { return T_; };

 private:
  std::vector<double> coefficients_;
  std::pair<double, double> first_derv_extrema_;   // <vel_max, vel_min>
  std::pair<double, double> second_derv_extrema_;  // <a_max, a_min>
  std::pair<double, double> third_derv_extrema_;   // <j_max, j_min>

  double T_ = 5.0;
};

}  // namespace planning

#endif  // QUARTIC_POLYNOMIAL_H_