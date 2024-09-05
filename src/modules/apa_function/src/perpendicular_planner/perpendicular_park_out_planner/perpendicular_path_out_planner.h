#ifndef __PERPENDICULAR_PATH_OUT_PLANNER_H__
#define __PERPENDICULAR_PATH_OUT_PLANNER_H__

#include "perpendicular_path_planner.h"

namespace planning {
namespace apa_planner {

class PerpendicularPathOutPlanner : public PerpendicularPathPlanner {
 public:
  virtual void Reset() override;
  virtual const bool Update() override;

  virtual const bool UpdateByPrePlan() override;

 private:
  virtual void Preprocess() override;
};

}  // namespace apa_planner
}  // namespace planning

#endif