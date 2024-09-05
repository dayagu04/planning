#ifndef __PERPENDICULAR_PATH_HEADING_IN_PLANNER_H__
#define __PERPENDICULAR_PATH_HEADING_IN_PLANNER_H__

#include "perpendicular_path_planner.h"

namespace planning {
namespace apa_planner {

class PerpendicularPathHeadingInPlanner : public PerpendicularPathPlanner {
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