#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include "road_type_storage.h"
#include "utils/kd_path.h"

namespace planning {

class RoadTypeUtils {
 public:
  RoadTypeUtils() = delete;
  ~RoadTypeUtils() = delete;

  static bool RoadLatTypeAnalysis(const ReferencePathPoints& refer_path_points,
                                  planning_math::ConstKDPathPtr kd_path,
                                  RoadTypeStoragePtr road_type_storage);

  static bool RoadHeightTypeAnalysis(
      const ReferencePathPoints& refer_path_points,
      planning_math::ConstKDPathPtr kd_path,
      RoadTypeStoragePtr road_type_storage);

  static bool RoadWidthTypeAnalysis(
      const ReferencePathPoints& refer_path_points,
      planning_math::ConstKDPathPtr kd_path,
      RoadTypeStoragePtr road_type_storage);

 private:
/*
  void CalcRoadWidthType();

  void CalcRoadHeightType();

*/

};
}  // namespace planning