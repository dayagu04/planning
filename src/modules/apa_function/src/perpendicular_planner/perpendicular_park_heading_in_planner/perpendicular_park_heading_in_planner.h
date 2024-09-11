#ifndef __PERPENDICULAR_PARK_HEADING_IN_PLANNER_H__
#define __PERPENDICULAR_PARK_HEADING_IN_PLANNER_H__

#include "perpendicular_park_planner.h"

#include "perpendicular_path_heading_in_planner.h"

namespace planning {
namespace apa_planner {

class PerpendicularParkHeadingInPlanner : public PerpendicularParkPlanner {
 public:
  PerpendicularParkHeadingInPlanner() = default;
  PerpendicularParkHeadingInPlanner(
      const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }
  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); }
};

}  // namespace apa_planner
}  // namespace planning

#endif
