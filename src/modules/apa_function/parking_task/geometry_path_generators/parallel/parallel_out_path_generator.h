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
  const double GetArcSlotParkOutHeading(){
    return arc_slot_init_out_heading_;
  };

 protected:
  virtual void Preprocess() override;

  const bool GenParallelPreparingLineVecOut(
      std::vector<pnc::geometry_lib::PathPoint>& preparing_pose_vec);

  const int SelectParkOutPathVec(
      const std::vector<std::vector<pnc::geometry_lib::PathSegment>>&
          park_out_path_vec);
  double arc_slot_init_out_heading_= 0.0;
};

}  // namespace apa_planner
}  // namespace planning