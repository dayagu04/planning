/***************************************************************
 * @file       math_lib.h
 * @brief      for math lib design
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-17-2021
 **************************************************************/
#include "math_lib.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include "Eigen/Geometry"
namespace pnc {
namespace mathlib {
static const double kPie = 3.141592653589793;

double DoubleConstrain(double u, double min, double max) {
  if (u < min) {
    return min;
  } else if (u > max) {
    return max;
  } else {
    return u;
  }
}

double DoubleLimit(double u, double limit) {
  if (u < -limit) {
    return -limit;
  } else if (u > limit) {
    return limit;
  } else {
    return u;
  }
}

double Rad2Deg(double u) { return (57.295779513082323 * u); }

double Deg2Rad(double u) { return (0.017453292519943 * u); }

double DeadzoneTypeI(double d0, double k, double u) {
  double out = u;

  d0 = std::max(d0, 0.0);
  k = std::max(k, 1.2);

  if (fabs(d0) > 0.0000001) {
    if (fabs(u) <= d0) {
      out = 0.0;
    } else if (IsInBound(u, d0, k * d0 / (k - 1.0))) {
      out = k * (u - d0);
    } else if (IsInBound(u, -k * d0 / (k - 1.0), -d0)) {
      out = k * (u + d0);
    }
  }

  return out;
}

bool IsDoubleEqual(double a, double b) {
  if (fabs(a - b) < 1E-8) {
    return true;
  } else {
    return false;
  }
}

bool IsDoubleEqual(double a, double b, double eps) {
  if (fabs(a - b) < eps) {
    return true;
  } else {
    return false;
  }
}

bool PolynomialFitting(size_t order, std::vector<double> &coef_vec, std::vector<double> &x_vec,
                       std::vector<double> &y_vec) {
  if (order < 1 || x_vec.size() < order + 1 || y_vec.size() < order + 1 || x_vec.size() != y_vec.size()) {
    return false;
  }

  coef_vec.reserve(order + 1);
  coef_vec.clear();

  // map sample data from STL vector to eigen vector
  Eigen::Map<Eigen::VectorXd> sampleX(x_vec.data(), x_vec.size());
  Eigen::Map<Eigen::VectorXd> sampleY(y_vec.data(), y_vec.size());

  Eigen::MatrixXd mtxVandermonde(x_vec.size(),
                                 order + 1);  // Vandermonde matrix of X-axis coordinate vector of sample data
  Eigen::VectorXd colVandermonde = sampleX;   // Vandermonde column

  // construct Vandermonde matrix column by column
  for (size_t i = 0; i < order + 1; ++i) {
    if (0 == i) {
      mtxVandermonde.col(0) = Eigen::VectorXd::Constant(x_vec.size(), 1, 1);
      continue;
    }
    if (1 == i) {
      mtxVandermonde.col(1) = colVandermonde;
      continue;
    }
    colVandermonde = colVandermonde.array() * sampleX.array();
    mtxVandermonde.col(i) = colVandermonde;
  }

  // calculate coefficients vector of fitted polynomial
  Eigen::VectorXd result =
      (mtxVandermonde.transpose() * mtxVandermonde).inverse() * (mtxVandermonde.transpose()) * sampleY;

  // Eigen::VectorXd::Map(&coef_vec[0], result.size()) = result;
  for (size_t i = 0; i < order + 1; ++i) {
    coef_vec.push_back(result[order - i]);
  }

  return true;
}

double GetCurvFactor(double c1, double c2, double vel) {
  double curv_factor;
  curv_factor = c1 / (1 + c2 * vel * vel);
  curv_factor = mathlib::Clamp(curv_factor, 0.15, 0.5);
  return curv_factor;
}

double DeltaAngleFix(double delta_angle) {
  double out = delta_angle;
  for (int i = 0; i < 5; ++i) {
    if ((out >= -kPie) && (out <= kPie)) {
      break;
    }
    if (out < -kPie) {
      out += 2.0 * kPie;
    } else if (out > kPie) {
      out -= 2.0 * kPie;
    }
  }

  out = Clamp(out, -kPie, kPie);
  return out;
}

}  // namespace mathlib

}  // namespace pnc
