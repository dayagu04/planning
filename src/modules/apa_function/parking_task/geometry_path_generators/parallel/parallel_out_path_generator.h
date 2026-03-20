#pragma once

#include "parallel_path_generator.h"
namespace planning {
namespace apa_planner {

class ParallelOutPathGenerator : public ParallelPathGenerator {
 public:
  void Reset() override;
  virtual const bool Update() override;
  virtual const bool Update(const std::shared_ptr<CollisionDetector>
                                &collision_detector_ptr) override;

  const std::unordered_map<ApaParkOutDirection,
                           std::vector<pnc::geometry_lib::PathSegment>>&
  GetPathByDirection() const {
    return parkout_path_by_direction_;
  }

  void ClearPathByDirection() { parkout_path_by_direction_.clear(); }

 protected:
  virtual void Preprocess() override;

  const bool GenParallelPreparingLineVecOut(
      std::vector<pnc::geometry_lib::PathPoint>& preparing_pose_vec);

  const int SelectParkOutPathVec(
      const std::vector<std::vector<pnc::geometry_lib::PathSegment>>&
          park_out_path_vec);

  std::unordered_map<ApaParkOutDirection,
                     std::vector<pnc::geometry_lib::PathSegment>>
      parkout_path_by_direction_;
};

}  // namespace apa_planner
}  // namespace planning