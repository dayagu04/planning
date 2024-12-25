#pragma once
#include <memory>

#include "apa_measure_data_manager.h"
#include "geometry_math.h"
#include "local_view.h"
#include "planning_plan_c.h"

namespace planning {
namespace apa_planner {
class ApaPredictPathManager final {
 public:
  ApaPredictPathManager() {}
  ~ApaPredictPathManager() {}

  void Update(const LocalView* local_view_ptr,
              const iflyauto::PlanningOutput* planning_output,
              const std::shared_ptr<ApaMeasureDataManager> measure_data_ptr);

  void Reset() {
    predict_pt_vec_.clear();
    gear_ = pnc::geometry_lib::PathSegGear::SEG_GEAR_INVALID;
  }

  const std::vector<pnc::geometry_lib::PathPoint>& GetPredictPath() const {
    return predict_pt_vec_;
  }

  const pnc::geometry_lib::PathSegGear GetPathGear() const { return gear_; }

 private:
  std::vector<pnc::geometry_lib::PathPoint> predict_pt_vec_;
  pnc::geometry_lib::PathSegGear gear_{
      pnc::geometry_lib::PathSegGear::SEG_GEAR_INVALID};
};
}  // namespace apa_planner
}  // namespace planning
