#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include "static_analysis_storage.h"
#include "utils/kd_path.h"
#include "modules/context/reference_path.h"

namespace planning {

class StaticAnalysisUtils {
 public:
  StaticAnalysisUtils() = delete;
  ~StaticAnalysisUtils() = delete;

  static bool RoadTypeAnalysis(
      const ReferencePathPoints& refer_path_points,
      planning_math::ConstKDPathPtr kd_path,
      StaticAnalysisStoragePtr static_analysis_storage);

  static bool PassageTypeAnalysis(
      const ReferencePathPoints& refer_path_points,
      planning_math::ConstKDPathPtr kd_path,
      StaticAnalysisStoragePtr static_analysis_storage);

  static bool ElemTypeAnalysis(
      ConstReferencePathPtr refer_path_ptr,
      const ReferencePathPoints& refer_path_points,
      planning_math::ConstKDPathPtr kd_path,
      StaticAnalysisStoragePtr static_analysis_storage);

 private:

};
}  // namespace planning