#pragma once

#include <map>
#include <utility>

#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "lane_reference_path.h"
#include "session.h"
#include "src/modules/common/math/filter/mean_filter.h"

namespace planning {

enum class ReferencePathType {
  MAP_LANE,
  REFINED_LANE,
  LANE_CHANGE,
  HISTORY_LINE
};
using ReferencePathKeyType = std::pair<ReferencePathType, int>;

class ReferencePathManager {
 public:
  ReferencePathManager(planning::framework::Session* session);

  virtual ~ReferencePathManager();

  // map lane reference
  std::shared_ptr<ReferencePath> get_reference_path_by_lane(
      int virtual_lane_id, bool create_if_not_exist = true);
  std::shared_ptr<ReferencePath> get_reference_path_by_current_lane();

  // update
  bool update();
  std::shared_ptr<ReferencePath> make_map_lane_reference_path(
      int virtual_lane_id);

  std::shared_ptr<planning::planning_math::MeanFilter> &MutableSmoothBoundFilter() {
    return smooth_bound_filter_;
  }

  const bool GetIsSwitchRefPath() const {
    return is_switch_ref_path_;
  }

 private:
  bool GetReferencePathByConstructionScene();

 private:
  planning::framework::Session* session_;
  std::map<ReferencePathKeyType, std::shared_ptr<ReferencePath>>
      reference_paths_;
  // smooth
  int last_current_lane_virtual_id_ = INT_MAX;
  std::shared_ptr<planning::planning_math::MeanFilter> smooth_bound_filter_ = nullptr;
  bool is_switch_ref_path_ = false;
  ReferencePathSource current_ref_path_source_ = ReferencePathSource::FUSION_ROAD;
};

}  // namespace planning
