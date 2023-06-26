#pragma once

#include <map>
#include <utility>

#include "config/basic_type.h"
#include "lane_reference_path.h"
#include "session.h"

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
  void update();
  std::shared_ptr<ReferencePath> make_map_lane_reference_path(
      int virtual_lane_id);

 private:
  planning::framework::Session* session_;
  std::map<ReferencePathKeyType, std::shared_ptr<ReferencePath>>
      reference_paths_;
};

}  // namespace planning
