#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include "static_analysis_storage.h"

namespace planning {

std::vector<CRoadType> StaticAnalysisStorage::turn_type_list = {
    CRoadType::NormalTurn, CRoadType::SharpTurn, CRoadType::WideTurn,
    CRoadType::UTurn, CRoadType::CurveTurn};
std::vector<CElemType> StaticAnalysisStorage::ramp_type_list = {
    CElemType::UpRampRoad, CElemType::DownRampRoad,
    CElemType::UnknownRampRoad};

void StaticAnalysisStorage::Clear() {
  road_type_storage_.clear();
  passage_type_storage_.clear();
  elem_type_storage_.clear();
}

template <>
void StaticAnalysisStorage::SetTypeList(CRoadType type,
                                  const SRangeList& s_range_list) {
  road_type_storage_[type].s_range_list = s_range_list;
}

template <>
void StaticAnalysisStorage::SetTypeList(CPassageType type,
                                  const SRangeList& s_range_list) {
  passage_type_storage_[type].s_range_list = s_range_list;
}

template <>
void StaticAnalysisStorage::SetTypeList(CElemType type,
                                  const SRangeList& s_range_list) {
  elem_type_storage_[type].s_range_list = s_range_list;
}

#ifdef ENABLE_IDX_RANGE_LIST_STORAGE
template <>
void StaticAnalysisStorage::SetTypeList(CRoadType type,
                                        const IdxRangeList& idx_range_list) {
  road_type_storage_[type].idx_range_list = idx_range_list;
}

template <>
void StaticAnalysisStorage::SetTypeList(CPassageType type,
                                        const IdxRangeList& idx_range_list) {
  passage_type_storage_[type].idx_range_list = idx_range_list;
}

template <>
void StaticAnalysisStorage::SetTypeList(CElemType type,
                                        const IdxRangeList& idx_range_list) {
  elem_type_storage_[type].idx_range_list = idx_range_list;
}
#endif
ResultTypeInfo StaticAnalysisStorage::GetTypeInfo(const double cur_s) const {
  ResultTypeInfo road_type_info;
  road_type_info.road_type = GetTypeInfo(road_type_storage_, cur_s);
  road_type_info.passage_type = GetTypeInfo(passage_type_storage_, cur_s);
  road_type_info.elem_types = GetTypeInfos(elem_type_storage_, cur_s);
  return road_type_info;
}

SRangeList StaticAnalysisStorage::GetSRangeList(const QueryTypeInfo& road_type) const {
  std::vector<SRangeList> source_range_lists;
  if (road_type.road_type == CRoadType::Turn) {
    SRangeList turn_range_list;
    for(const auto& turn_type : turn_type_list) {
      if (road_type_storage_.find(turn_type) != road_type_storage_.end()) {
        const auto& s_range_list = road_type_storage_.at(turn_type).s_range_list;
        turn_range_list.insert(turn_range_list.end(), s_range_list.begin(),
                               s_range_list.end());
      }
    }
    source_range_lists.push_back(CalcIntervalUnion(turn_range_list));
  } else {
    if (road_type.road_type != CRoadType::Ignore) {
      if (road_type_storage_.find(road_type.road_type) !=
          road_type_storage_.end()) {
        source_range_lists.push_back(
            road_type_storage_.at(road_type.road_type).s_range_list);
      } else {
        source_range_lists.emplace_back();
      }
    }
  }

  if (road_type.passage_type != CPassageType::Ignore) {
    if (passage_type_storage_.find(road_type.passage_type) != passage_type_storage_.end()) {
      source_range_lists.push_back(
          passage_type_storage_.at(road_type.passage_type).s_range_list);
    } else {
      source_range_lists.emplace_back();
    }
  }

  if (road_type.elem_type == CElemType::RampRoad) {
    SRangeList ramp_range_list;
    for(const auto& ramp_type : ramp_type_list) {
      if (elem_type_storage_.find(ramp_type) != elem_type_storage_.end()) {
        const auto& s_range_list = elem_type_storage_.at(ramp_type).s_range_list;
        ramp_range_list.insert(ramp_range_list.end(), s_range_list.begin(),
                               s_range_list.end());
      }
    }
    source_range_lists.push_back(CalcIntervalUnion(ramp_range_list));
  } else {
    if (road_type.elem_type != CElemType::Ignore) {
      if (elem_type_storage_.find(road_type.elem_type) !=
          elem_type_storage_.end()) {
        source_range_lists.push_back(
            elem_type_storage_.at(road_type.elem_type).s_range_list);
      } else {
        source_range_lists.emplace_back();
      }
    }
  }

  auto res_range_list = CalcIntervalIntersection(source_range_lists);
  std::sort(res_range_list.begin(), res_range_list.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  return res_range_list;
}

std::pair<double, double> StaticAnalysisStorage::GetFrontSRange(
    const QueryTypeInfo& type_list, const double cur_s) const {
  const auto& target_s_range_list = GetSRangeList(type_list);

  if (target_s_range_list.empty()) {
    return std::pair<double, double>(0.0, -1.0);
  }
  for (const auto& range : target_s_range_list) {
    if (range.second >= cur_s) {
      return range;
    }
  }
  return std::pair<double, double>(0.0, -1.0);
}

std::pair<double, double> StaticAnalysisStorage::GetBackSRange(
    const QueryTypeInfo& type_list, const double cur_s) const {
  const auto& target_s_range_list = GetSRangeList(type_list);
  if (target_s_range_list.empty()) {
    return std::pair<double, double>(0.0, -1.0);
  }
  for (int i = target_s_range_list.size() - 1; i >= 0; --i) {
    const auto& range = target_s_range_list[i];
    if (range.first <= cur_s) {
      return range;
    }
  }
  return std::pair<double, double>(0.0, -1.0);
}

SRangeList StaticAnalysisStorage::GetUnionSRangeList(
    const std::vector<QueryTypeInfo>& type_lists) const {
  SRangeList source_range_list;
  for (const auto& type_list : type_lists) {
    const auto& cur_type_range_list = GetSRangeList(type_list);
    source_range_list.insert(source_range_list.begin(),
                             cur_type_range_list.begin(),
                             cur_type_range_list.end());
  }
  auto res_range_list = CalcIntervalUnion(source_range_list);
  std::sort(res_range_list.begin(), res_range_list.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  return res_range_list;
}

std::pair<double, double> StaticAnalysisStorage::GetBackUnionSRange(
    const std::vector<QueryTypeInfo>& type_list, const double cur_s) const {
  const auto& target_s_range_list = GetUnionSRangeList(type_list);
  if (target_s_range_list.empty()) {
    return std::pair<double, double>(0.0, -1.0);
  }
  for (int i = target_s_range_list.size() - 1; i >= 0; --i) {
    const auto& range = target_s_range_list[i];
    if (range.first <= cur_s) {
      return range;
    }
  }
  return std::pair<double, double>(0.0, -1.0);
}

std::pair<double, double> StaticAnalysisStorage::GetFrontUnionSRange(
    const std::vector<QueryTypeInfo>& type_list, const double cur_s) const {
  const auto& target_s_range_list = GetUnionSRangeList(type_list);
  if (target_s_range_list.empty()) {
    return std::pair<double, double>(0.0, -1.0);
  }
  for (const auto& range : target_s_range_list) {
    if (range.second >= cur_s) {
      return range;
    }
  }
  return std::pair<double, double>(0.0, -1.0);
}

bool StaticAnalysisStorage::SerializeToDebugInfo(
    planning_math::ConstKDPathPtr kd_path,
    common::StaticAnalysisResult& static_analysis_result) const {
  static_analysis_result.Clear();
  const auto& path_points = kd_path->path_points();

  auto serialize_storage_item = [&](const auto& storage_item, auto* result_item) {
#ifdef ENABLE_IDX_RANGE_LIST_STORAGE
    for(const auto& range : storage_item.idx_range_list) {
      for(int i = range.first; i <= range.second; ++i) {
        auto* point = result_item->add_points();
        point->set_x(path_points[i].x());
        point->set_y(path_points[i].y());
      }
    }
#else
    for(const auto& range : storage_item.s_range_list) {
      int start_idx = kd_path->GetPathPointIdxByS(range.first);
      int end_idx = kd_path->GetPathPointIdxByS(range.second);
      for(int i = start_idx; i <= end_idx; ++i) {
        auto* point = result_item->add_points();
        point->set_x(path_points[i].x());
        point->set_y(path_points[i].y());
      }
    }
#endif
  };

  for(const auto& item : road_type_storage_) {
    auto* road_type_item = static_analysis_result.add_road_types();
    road_type_item->set_type(static_cast<common::RoadTypeItem_RoadTypeEnum>(item.first));
    serialize_storage_item(item.second, road_type_item);
  }

  for(const auto& item : passage_type_storage_) {
    auto* passage_type_item = static_analysis_result.add_passage_types();
    passage_type_item->set_type(static_cast<common::PassageTypeItem_PassageTypeEnum>(item.first));
    serialize_storage_item(item.second, passage_type_item);
  }

  for(const auto& item : elem_type_storage_) {
    auto* elem_type_item = static_analysis_result.add_elem_types();
    elem_type_item->set_type(static_cast<common::ElemTypeItem_ElemTypeEnum>(item.first));
    serialize_storage_item(item.second, elem_type_item);
  }
  return true;
}

/******************** private funcion **************** */
template <typename T>
std::vector<std::pair<T, T>> StaticAnalysisStorage::CalcIntervalIntersection(
    std::vector<std::vector<std::pair<T, T>>>& range_lists) const {
  if (range_lists.empty()) {
    return {};
  }
  for (const auto& range_list : range_lists) {
    for (const auto& range : range_list) {
      CHECK_LE(range.first, range.second);
    }
  }

  std::vector<std::pair<T, T>> res_range_list = range_lists.front();
  auto compare_less = [](const std::pair<T, T>& interval_lhs, const std::pair<T, T>& interval_rhs) {
    if (interval_lhs.first != interval_rhs.first) {
      return interval_lhs.first < interval_rhs.first;
    } else {
      return interval_lhs.second < interval_rhs.second;
    }
  };
  std::sort(res_range_list.begin(), res_range_list.end(), compare_less);
  for (auto ite = range_lists.begin() + 1; ite != range_lists.end(); ++ite) {
    auto& next_range_list = *ite;
    std::sort(next_range_list.begin(), next_range_list.end(), compare_less);
    size_t first_range_index = 0;
    size_t second_range_index = 0;
    std::vector<std::pair<T, T>> tmp_range_list;
    while (first_range_index < res_range_list.size() &&
           second_range_index < next_range_list.size()) {
      if (res_range_list[first_range_index].second < next_range_list[second_range_index].first) {
        ++first_range_index;
        continue;
      }
      if (next_range_list[second_range_index].second < res_range_list[first_range_index].first) {
        ++second_range_index;
        continue;
      }
      const T interval_start = std::max(res_range_list[first_range_index].first,
                                        next_range_list[second_range_index].first);
      const T interval_end = std::min(res_range_list[first_range_index].second,
                                      next_range_list[second_range_index].second);
      tmp_range_list.push_back({interval_start, interval_end});
      if (res_range_list[first_range_index].second < next_range_list[second_range_index].second) {
        ++first_range_index;
      } else {
        ++second_range_index;
      }
    }
    res_range_list.swap(tmp_range_list);
  }
  return res_range_list;
}

template <typename T>
std::vector<std::pair<T, T>> StaticAnalysisStorage::CalcIntervalUnion(
    std::vector<std::pair<T, T>>& range_lists) const {
  if (range_lists.size() == 0) {
    return {};
  }

  for (const auto& range : range_lists) {
    CHECK_LE(range.first, range.second);
  }

  std::sort(range_lists.begin(), range_lists.end(),
            [](const std::pair<T, T>& interval_lhs, const std::pair<T, T>& interval_rhs) {
              return interval_lhs.first <= interval_rhs.first;
            });
  std::vector<std::pair<T, T>> combined_intervals;
  for (size_t i = 0; i < range_lists.size(); ++i) {
    T left = range_lists[i].first;
    T right = range_lists[i].second;
    if (combined_intervals.empty() || combined_intervals.back().second <= left) {
      combined_intervals.emplace_back(std::make_pair(left, right));
    } else {
      combined_intervals.back().second = std::fmax(combined_intervals.back().second, right);
    }
  }
  return combined_intervals;
}

template <typename T1>
T1 StaticAnalysisStorage::GetTypeInfo(
    const std::unordered_map<T1, StorageItem>& type_2_storage_item,
    const double s) const {
  constexpr double kEpsilon = 0.001;
  for (const auto& item : type_2_storage_item) {
    const auto& s_range_list = item.second.s_range_list;
    if (s_range_list.empty()) {
      continue;
    }
    for (const auto& interval : s_range_list) {
      if (s > interval.first - kEpsilon && s < interval.second + kEpsilon) {
        return item.first;
      }
    }
  }
  return T1::Unknown;
}

template <typename T1>
std::vector<T1> StaticAnalysisStorage::GetTypeInfos(
    const std::unordered_map<T1, StorageItem>& type_2_storage_item,
    const double s) const {
  constexpr double kEpsilon = 0.001;
  std::vector<T1> res;
  for (const auto& item : type_2_storage_item) {
    const auto& s_range_list = item.second.s_range_list;
    if (s_range_list.empty()) {
      continue;
    }
    for (const auto& interval : s_range_list) {
      if (s > interval.first - kEpsilon && s < interval.second + kEpsilon) {
        res.push_back(item.first);
      }
    }
  }
  return res;
}
}  // namespace planning