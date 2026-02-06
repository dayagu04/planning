#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>
#include "road_type_utils.h"

namespace planning {

bool RoadTypeUtils::RoadLatTypeAnalysis(
    const ReferencePathPoints& refer_path_points,
    planning_math::ConstKDPathPtr kd_path,
    RoadTypeStoragePtr road_type_storage) {
  return true;
}

bool RoadTypeUtils::RoadWidthTypeAnalysis(
    const ReferencePathPoints& refer_path_points,
    planning_math::ConstKDPathPtr kd_path,
    RoadTypeStoragePtr road_type_storage) {
  return true;
}

bool RoadTypeUtils::RoadHeightTypeAnalysis(
    const ReferencePathPoints& refer_path_points,
    planning_math::ConstKDPathPtr kd_path,
    RoadTypeStoragePtr road_type_storage) {
  return true;
}
}  // namespace planning