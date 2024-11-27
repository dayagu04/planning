#include "dv_spline.h"

#include <algorithm>
#include <vector>

#include "st_spline.h"

namespace planning {

void DvSpline::sort_by_d() {
  auto comp = [](const DvPoint& p1, const DvPoint& p2) { return p1.d < p2.d; };
  std::sort(dv_points_.begin(), dv_points_.end(), comp);
}

}  // namespace planning