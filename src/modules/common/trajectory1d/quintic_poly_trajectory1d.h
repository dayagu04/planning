#ifndef QUINTIC_POLYNOMIAL_H_
#define QUINTIC_POLYNOMIAL_H_

#include <vector>

#include "Eigen/Dense"

namespace planning {
struct QuinticPolyState {
  double p;
  double v;
  double a;

  double T;
};

class QuinticPolynomial {
 public:
  // Constructor
  QuinticPolynomial(const std::vector<double>& start,
                    const std::vector<double>& end, double T);
  QuinticPolynomial(const QuinticPolyState& start,
                    const QuinticPolyState& end);  // p, v, a, T
  QuinticPolynomial() = default;

  // Destructor
  virtual ~QuinticPolynomial(){};

  // calculate the s/d coordinate of a point
  double CalculatePoint(const double t) const;

  double CalculateFirstDerivative(const double t) const;

  double CalculateSecondDerivative(const double t) const;

  double CalculateThirdDerivative(const double t) const;

  std::pair<double, double> FindFirstDerivativeExtrema(const double te);

  std::pair<double, double> FindSecondDerivativeExtrema(const double te);

  std::pair<double, double> FindThirdDerivativeExtrema(const double te);

  std::vector<double> SolveCubic(double a, double b, double c, double d) const;

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