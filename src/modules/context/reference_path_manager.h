#pragma once


#include <map>
#include <utility>

#include "framework/session.h"
#include "modules/common/config/basic_type.h"
#include "modules/context/lane_reference_path.h"

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

 private:
  planning::framework::Session* session_;
  std::map<ReferencePathKeyType, std::shared_ptr<ReferencePath>>
      reference_paths_;
  std::map<int, std::shared_ptr<LaneReferencePath>>
      lane_reference_paths_;
  // std::map<int, std::shared_ptr<LaneReferencePath>>
  //     lane_reference_paths_;
  // std::map<int, std::shared_ptr<LaneReferencePath>>
  //     lane_reference_paths_;
  // std::map<int, std::shared_ptr<LaneReferencePath>>
  //     lane_reference_paths_;
  // std::map<int, std::shared_ptr<ReferencePath>>
  //     reference_paths_;
};

std::shared_ptr<ReferencePath> make_map_lane_reference_path(
    ReferencePathManager* reference_path_manager, int virtual_lane_id);

}

