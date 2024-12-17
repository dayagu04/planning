#ifndef __PARALLEL_OUT_PATH_GENERATOR__
#define __PARALLEL_OUT_PATH_GENERATOR__

#include "parallel_path_generator.h"
namespace planning {
namespace apa_planner {

class ParallelOutPathGenerator : public ParallelPathGenerator {
 public:
  void Reset() override;
  virtual const bool Update() override;
  virtual const bool Update(const std::shared_ptr<CollisionDetector>
                                &collision_detector_ptr) override;

 protected:
  virtual void Preprocess() override;
};

}  // namespace apa_planner
}  // namespace planning

#endif