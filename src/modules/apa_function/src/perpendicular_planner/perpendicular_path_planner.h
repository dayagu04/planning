#ifndef __PERPENDICULAR_PATH_PLANNER_H__
#define __PERPENDICULAR_PATH_PLANNER_H__

#include "apa_path_planner.h"

namespace planning {
namespace apa_planner {

class PerpendicularPathPlanner : public ApaPathPlanner {
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