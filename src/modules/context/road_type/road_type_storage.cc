#include <cstddef>
#include <cstdint>
#include <functional>
#include "road_type_storage.h"

namespace planning {

std::vector<RLatType> RoadTypeStorage::turn_type_list = {
    RLatType::NormalTurn, RLatType::SharpTurn, RLatType::WideTurn,
    RLatType::UTurn, RLatType::CurveTurn};
std::vector<RHeightType> RoadTypeStorage::ramp_type_list = {
    RHeightType::UpRampRoad, RHeightType::DownRampRoad,
    RHeightType::UnknownRampRoad};

void RoadTypeStorage::Clear() {
  lat_type_storage_.clear();
  width_type_storage_.clear();
  height_type_storage_.clear();
}

template <typename T>
void RoadTypeStorage::SetTypeList(const RTypeEnum type_enum, T type,
                 const SRangeList& s_range_list) {
  switch (type_enum) {
    case RTypeEnum::RLatType:
      lat_type_storage_[type].s_range_list = s_range_list;
      break;
    case RTypeEnum::RWidthType:
      width_type_storage_[type].s_range_list = s_range_list;
      break;
    case RTypeEnum::RHeightType:
      height_type_storage_[type].s_range_list = s_range_list;
      break;
    default:
      break;
  }
}

RoadTypeInfo RoadTypeStorage::GetRoadTypeInfo(const double cur_s) const {
  RoadTypeInfo road_type_info;
  road_type_info.lat_type = GetRoadType(lat_type_storage_, cur_s);
  road_type_info.width_type = GetRoadType(width_type_storage_, cur_s);
  road_type_info.height_type = GetRoadType(height_type_storage_, cur_s);
  return road_type_info;
}

SRangeList RoadTypeStorage::GetSRangeList(const RoadTypeInfo& road_type) const {
  std::vector<SRangeList> source_range_lists;
  if (road_type.lat_type == RLatType::Turn) {
    SRangeList turn_range_list;
    for(const auto& turn_type : turn_type_list) {
      if (lat_type_storage_.find(turn_type) != lat_type_storage_.end()) {
        const auto& s_range_list = lat_type_storage_.at(turn_type).s_range_list;
        turn_range_list.insert(turn_range_list.end(), s_range_list.begin(),
                               s_range_list.end());
      }
    }
    source_range_lists.push_back(CalcIntervalUnion(turn_range_list));
  } else {
    if (road_type.lat_type != RLatType::Ignore) {
      if (lat_type_storage_.find(road_type.lat_type) !=
          lat_type_storage_.end()) {
        source_range_lists.push_back(
            lat_type_storage_.at(road_type.lat_type).s_range_list);
      } else {
        source_range_lists.emplace_back();
      }
    }
  }

  if (road_type.width_type != RWidthType::Ignore) {
    if (width_type_storage_.find(road_type.width_type) != width_type_storage_.end()) {
      source_range_lists.push_back(
          width_type_storage_.at(road_type.width_type).s_range_list);
    } else {
      source_range_lists.emplace_back();
    }
  }

  if (road_type.height_type == RHeightType::RampRoad) {
    SRangeList ramp_range_list;
    for(const auto& ramp_type : ramp_type_list) {
      if (height_type_storage_.find(ramp_type) != height_type_storage_.end()) {
        const auto& s_range_list = height_type_storage_.at(ramp_type).s_range_list;
        ramp_range_list.insert(ramp_range_list.end(), s_range_list.begin(),
                               s_range_list.end());
      }
    }
    source_range_lists.push_back(CalcIntervalUnion(ramp_range_list));
  } else {
    if (road_type.height_type != RHeightType::Ignore) {
      if (height_type_storage_.find(road_type.height_type) !=
          height_type_storage_.end()) {
        source_range_lists.push_back(
            height_type_storage_.at(road_type.height_type).s_range_list);
      } else {
        source_range_lists.emplace_back();
      }
    }
  }

  return CalcIntervalIntersection(source_range_lists);
}

std::pair<double, double> RoadTypeStorage::GetFrontSRangeByRoadType(
    const RoadTypeInfo& type_list, const double cur_s) const {
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

std::pair<double, double> RoadTypeStorage::GetBackSRangeByRoadType(
    const RoadTypeInfo& type_list, const double cur_s) const {
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

SRangeList RoadTypeStorage::GetUnionSRangeList(
    const std::vector<RoadTypeInfo>& type_lists) const {
  SRangeList source_range_list;
  for (const auto& type_list : type_lists) {
    const auto& cur_type_range_list = GetSRangeList(type_list);
    source_range_list.insert(source_range_list.begin(),
                             cur_type_range_list.begin(),
                             cur_type_range_list.end());
  }
  return CalcIntervalUnion(source_range_list);
}

std::pair<double, double> RoadTypeStorage::GetBackUnionSRangeByRoadType(
    const std::vector<RoadTypeInfo>& type_list, const double cur_s) const {
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

std::pair<double, double> RoadTypeStorage::GetFrontUnionSRangeByRoadType(
    const std::vector<RoadTypeInfo>& type_list, const double cur_s) const {
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

/******************** private funcion **************** */
template <typename T>
std::vector<std::pair<T, T>> RoadTypeStorage::CalcIntervalIntersection(
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
std::vector<std::pair<T, T>> RoadTypeStorage::CalcIntervalUnion(
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
T1 RoadTypeStorage::GetRoadType(
    const std::unordered_map<T1, RTypeStorageItem>& type_2_storage_item,
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
}  // namespace planning