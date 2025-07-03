#pragma once

#include <cstddef>
#include <vector>

#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"

namespace planning {

class CubicPathInterface {
 public:
  CubicPathInterface() = default;

  void Init();

  std::vector<float> GeneratePolynomialCoefficients(const Pose2f& start_point,
                                                    const Pose2f& target_point);

  void GeneratePolynomialPath(std::vector<AStarPathPoint>& path,
                              const std::vector<float>& coefficients,
                              const float step, const Pose2f& start_point,
                              const Pose2f& target_point);

  std::vector<float> GetThetaVec();
  std::vector<float> GetCurvatureVec();

  const float GetMinCurvatureRadius() const;
  const bool ArePosesEqual(const Pose2f& p1, const Pose2f& p2,
                           float epsilon = 1e-9);

 private:
  std::vector<float> theta_vec_;
  std::vector<float> curvature_vec_;
  // size_t max_plan_num_ = 25;
  float max_curvature_;
};
}  // namespace planning