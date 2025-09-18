#pragma once
#include <vector>

#include "config/basic_type.h"
#include "spatio_temporal_union_plan.pb.h"
#include "utils/kd_path.h"

namespace planning {

struct SpatioTemporalUnionPlanOutput {
  bool st_dp_is_sucess = 1;
  double cost_time = 4;
  bool enable_using_st_plan = 5;
};

}  // namespace planning