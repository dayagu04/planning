#ifndef MODULES_PLANNING_OPTIMIZERS_SL_POLYGON_SEQ_H_
#define MODULES_PLANNING_OPTIMIZERS_SL_POLYGON_SEQ_H_

#include <string>
#include <vector>

#include "math/polygon2d.h"
#include "math/math_utils.h"

namespace planning {

using PolygonWithT = std::pair<double, planning_math::Polygon2d>;
class SLPolygonSeq : public std::vector<PolygonWithT> {
 public:
  SLPolygonSeq() = default;

  virtual ~SLPolygonSeq() = default;

  explicit SLPolygonSeq(std::vector<PolygonWithT> sl_polygon_points);

  void SetTimeStep(double time_step);

  bool EvaluateByTime(const double t, PolygonWithT* const polygon_t) const;

  void set_invalid_time_sections(const std::vector<std::pair<double, double>> & secs);

  double TotalTime() const;

 private:
  double time_step_;
  bool is_uniform_time_step_{false};
  std::vector<std::pair<double, double>> invalid_time_sections_;
  static planning_math::IntervalMethodSolution<double> interval_methods_;
};

}  //  namespace planning

#endif /* MODULES_PLANNING_OPTIMIZERS_SL_POLYGON_SEQ_H_ */
