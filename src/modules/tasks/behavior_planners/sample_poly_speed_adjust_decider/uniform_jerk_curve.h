#pragma once
#include "sample_poly_const.h"
#include "sample_poly_curve.h"
#include "vector"
#include "array"

namespace planning {
struct UniformJerkCurveCoffi {
  std::vector<std::array<double, 4>> cofficients;
  std::vector<double> time_points;
  void clear(){
    cofficients.clear();
    time_points.clear();
  }
};

class UniformJerkCurve : public SamplePolyCurve {
 public:
  UniformJerkCurve() = default;
  UniformJerkCurve(const double s0, const double v0, const double a0,
                   const double jerk, const SamplePolyCurveConfig& config);
  ~UniformJerkCurve() = default;

  double CalcS(const double t) const override;
  double CalcV(const double t) const override;
  double CalcAcc(const double t) const override;
  double CalcJerk(const double t) const override;

  UniformJerkCurveCoffi GetCoffi() const { return cofficients_; }
  void BuildZeroJerkTraj();
  void BuildUniformJerkTraj(const double jerk);

 private:
  double s0_ = 0.0;
  double v0_ = 0.0;
  double a0_ = 0.0;
  double jerk_limit_upper_ = 0.0;
  double jerk_limit_lower_ = 0.0;
  double acc_limit_upper_ = 0.0;
  double acc_limit_lower_ = 0.0;
  double vel_limit_upper_ = 0.0;
  double vel_limit_lower_ = 0.0;
  UniformJerkCurveCoffi cofficients_;
};
}  // namespace planning