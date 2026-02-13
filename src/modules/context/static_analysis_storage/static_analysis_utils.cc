#include <google/protobuf/message.h>
#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>
#include "static_analysis_utils.h"

namespace planning {

struct LatTypeRangeInfo {
public:
  LatTypeRangeInfo(size_t index = 0) {
    idx = index;
  }
public:
  size_t idx;
  size_t s_idx;
  size_t e_idx;
  double max_kappa;
  CRoadType type;
};

bool GenerateTurnRanges(const ReferencePathPoints& refer_path_points,
                        std::vector<LatTypeRangeInfo>& turn_range_infos) {
  turn_range_infos.clear();
  constexpr double kPeakKappaThr = 0.1;
  constexpr double kTurnKappaThr = 0.04;

  // S1: check peak
  for (size_t i = 1; i < refer_path_points.size() - 1; ++i) {
    const auto prev_kappa = std::fabs(refer_path_points[i - 1].path_point.kappa());
    const auto curr_kappa = std::fabs(refer_path_points[i].path_point.kappa());
    const auto next_kappa = std::fabs(refer_path_points[i + 1].path_point.kappa());
    if (curr_kappa > kPeakKappaThr && curr_kappa > prev_kappa &&
        curr_kappa > next_kappa) {
      turn_range_infos.push_back(LatTypeRangeInfo(i));
    }
  }

  // S2: generate range
  for (auto& peak_kappa_range_info : turn_range_infos) {
    int index = peak_kappa_range_info.idx;
    int start_idx = index - 1, end_idx = index + 1;
    while (start_idx >= 0 &&
           std::fabs(refer_path_points[start_idx].path_point.kappa()) >
               kTurnKappaThr) {
      --start_idx;
    }
    while (end_idx <= refer_path_points.size() - 1 &&
           std::fabs(refer_path_points[end_idx].path_point.kappa()) >
               kTurnKappaThr) {
      ++end_idx;
    }
    peak_kappa_range_info.s_idx = start_idx + 1;
    peak_kappa_range_info.e_idx = end_idx - 1;
    peak_kappa_range_info.max_kappa =
        refer_path_points[index].path_point.kappa();
  }

  // S3: sort and merge duplicated range
  sort(turn_range_infos.begin(), turn_range_infos.end(),
       [](const LatTypeRangeInfo& a, const LatTypeRangeInfo& b) {
         return a.s_idx < b.s_idx;
       });
  size_t cur_idx = 1, res_idx = 0;
  for (; cur_idx < turn_range_infos.size(); ++cur_idx) {
    const auto& cur_info = turn_range_infos[cur_idx];
    const auto& res_info = turn_range_infos[res_idx];
    if (cur_info.s_idx > res_info.e_idx) {
      turn_range_infos[++res_idx] = cur_info;
    } else {
      if(std::fabs(cur_info.max_kappa) > std::fabs(res_info.max_kappa)) {
        turn_range_infos[res_idx].max_kappa = cur_info.max_kappa;
      }
      turn_range_infos[res_idx].e_idx = cur_info.e_idx;
    }
  }
  turn_range_infos.resize(res_idx + 1);

  // S4: check turn type
  auto is_curve_turn = [&refer_path_points](const LatTypeRangeInfo& turn_range_info) {
    return false;
  };
  auto is_uturn = [&refer_path_points](const LatTypeRangeInfo& turn_range_info) {
    return false;
  };
  constexpr double kAngleToRadianRation = 180 / M_PI;
  for (auto& turn_range_info : turn_range_infos) {
    const auto& s_point = refer_path_points[turn_range_info.s_idx].path_point;
    const auto& e_point = refer_path_points[turn_range_info.e_idx].path_point;
    // delta_heading range: 0° ~ 180°
    const double delta_heading =
        std::fabs(ad_common::math::AngleDiff(e_point.theta(), s_point.theta()));
    if(is_curve_turn(turn_range_info)) {
      turn_range_info.type = CRoadType::CurveTurn;
    } else if(is_uturn(turn_range_info)) {
      turn_range_info.type = CRoadType::UTurn;
    } else {
      if( delta_heading > 120 / kAngleToRadianRation) { // 120° ~ 180°
        turn_range_info.type = CRoadType::SharpTurn;
      } else if(delta_heading < 60 / kAngleToRadianRation) { // 0° ~ 60°
        turn_range_info.type = CRoadType::WideTurn;
      } else {  // 60° ~ 120°
        turn_range_info.type = CRoadType::NormalTurn;
      }
    }
  }
  return true;
}

bool GenerateStraightRanges(
    const ReferencePathPoints& refer_path_points,
    const std::vector<LatTypeRangeInfo>& turn_range_infos,
    std::vector<LatTypeRangeInfo>& straight_range_infos) {
  size_t s_idx = 0;
  for (const auto& turn_range_info : turn_range_infos) {
    if(turn_range_info.s_idx > s_idx) {
      straight_range_infos.push_back(LatTypeRangeInfo(s_idx));
      straight_range_infos.back().s_idx = s_idx;
      straight_range_infos.back().e_idx = turn_range_info.s_idx;
      straight_range_infos.back().type = CRoadType::NormalStraight;
      s_idx = turn_range_info.e_idx;
    }
  }
  if (!turn_range_infos.empty() &&
      turn_range_infos.back().e_idx < refer_path_points.size() - 1) {
    if (refer_path_points.size() - 1 > turn_range_infos.back().e_idx) {
      straight_range_infos.push_back(LatTypeRangeInfo(s_idx));
      straight_range_infos.back().s_idx = turn_range_infos.back().e_idx;
      straight_range_infos.back().e_idx = refer_path_points.size() - 1;
      straight_range_infos.back().type = CRoadType::NormalStraight;
    }
  }
  return true;
}

bool StaticAnalysisUtils::RoadTypeAnalysis(
    const ReferencePathPoints& refer_path_points,
    planning_math::ConstKDPathPtr kd_path,
    StaticAnalysisStoragePtr static_analysis_storage) {
  size_t num = refer_path_points.size();

  // S1：check kappa peak value and index
  std::vector<LatTypeRangeInfo> turn_range_infos;
  GenerateTurnRanges(refer_path_points, turn_range_infos);

  // S2: determine turn type and range

  // S3: merge small range

  // S4: generate straight range
  std::vector<LatTypeRangeInfo> straight_range_infos;
  GenerateStraightRanges(refer_path_points, turn_range_infos,
                         straight_range_infos);

  // S5: generate road lat type result

  std::vector<LatTypeRangeInfo> final_range_info = turn_range_infos;
  final_range_info.insert(final_range_info.end(), straight_range_infos.begin(),
                          straight_range_infos.end());

  std::unordered_map<CRoadType, SRangeList> lat_type_to_s_range_list;
  for (const auto& range_info : final_range_info) {
    auto& s_range_list = lat_type_to_s_range_list[range_info.type];
    s_range_list.push_back(
        std::make_pair(refer_path_points[range_info.s_idx].path_point.s(),
                       refer_path_points[range_info.e_idx].path_point.s()));
  }
  for (auto& item : lat_type_to_s_range_list) {
    sort(item.second.begin(), item.second.end(),
         [](const auto& a, const auto& b) { return a.first < b.first; });

    static_analysis_storage->SetTypeList(item.first, item.second);
  }
#ifdef ENABLE_IDX_RANGE_LIST_STORAGE
  std::unordered_map<CRoadType, IdxRangeList> lat_type_to_idx_range_list;
  for (const auto& range_info : final_range_info) {
    auto& idx_range_list = lat_type_to_idx_range_list[range_info.type];
    idx_range_list.push_back(
        std::make_pair(range_info.s_idx, range_info.e_idx));
  }
  for (auto& item : lat_type_to_idx_range_list) {
    sort(item.second.begin(), item.second.end(),
         [](const auto& a, const auto& b) { return a.first < b.first; });

    static_analysis_storage->SetTypeList(item.first, item.second);
  }
#endif
  return true;
}

bool StaticAnalysisUtils::PassageTypeAnalysis(
    const ReferencePathPoints& refer_path_points,
    planning_math::ConstKDPathPtr kd_path,
    StaticAnalysisStoragePtr static_analisys_storage) {
  return true;
}

bool StaticAnalysisUtils::ElemTypeAnalysis(
    const ReferencePathPoints& refer_path_points,
    planning_math::ConstKDPathPtr kd_path,
    StaticAnalysisStoragePtr static_analysis_storage) {
  return true;
}

}  // namespace planning