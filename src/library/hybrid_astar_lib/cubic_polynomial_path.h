#pragma once

#include <cstddef>
#include <vector>

#include "hybrid_astar_common.h"

namespace planning {

class CubicPathInterface {
 public:
  CubicPathInterface() = default;

  void Init();

  std::vector<double> GeneratePolynomialCoefficients(
      const Pose2D& start_point, const Pose2D& target_point);

  void GeneratePolynomialPath(std::vector<AStarPathPoint>& path,
                              const std::vector<double>& coefficients,
                              const double step, const Pose2D& start_point,
                              const Pose2D& target_point);

  std::vector<double> GetThetaVec();
  std::vector<double> GetCurvatureVec();

  const double GetMinCurvatureRadius() const;
  const bool ArePosesEqual(const Pose2D& p1, const Pose2D& p2, double epsilon = 1e-9);

 private:
  std::vector<double> theta_vec_;
  std::vector<double> curvature_vec_;
  size_t max_plan_num_ = 25;
  double max_curvature_;
};
}  // namespace planning