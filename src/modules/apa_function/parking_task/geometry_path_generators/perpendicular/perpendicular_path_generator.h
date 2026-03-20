#ifndef __PERPENDICULAR_PATH_PLANNER_H__
#define __PERPENDICULAR_PATH_PLANNER_H__

#include "geometry_path_generator.h"

namespace planning {
namespace apa_planner {

class PerpendicularPathGenerator : public GeometryPathGenerator {
 public:
  virtual void Reset() override;
  virtual const bool Update() override;

  virtual const bool UpdateByPrePlan();

 protected:
  virtual void Preprocess() override;
};

}  // namespace apa_planner

}  // namespace planning

#endif