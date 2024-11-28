#pragma once
#include "geometry_math.h"
#include "local_view.h"

namespace planning {
namespace apa_planner {
class ApaPredictPathManager final {
 public:
  ApaPredictPathManager() {}
  ~ApaPredictPathManager() {}

  void Update(const LocalView* local_view_ptr);

  void Reset() { predict_pt_vec_.clear(); }

  const std::vector<pnc::geometry_lib::PathPoint> GetPredictPath() const {
    return predict_pt_vec_;
  }

 private:
  std::vector<pnc::geometry_lib::PathPoint> predict_pt_vec_;
};
}  // namespace apa_planner
}  // namespace planning
