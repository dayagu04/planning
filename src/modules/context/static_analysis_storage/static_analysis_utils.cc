#include "static_analysis_utils.h"

#include <google/protobuf/message.h>

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

#include "reference_path.h"
#include "static_analysis_storage/static_analysis_storage.h"

namespace planning {

struct LatTypeRangeInfo {
 public:
  LatTypeRangeInfo(size_t index = 0) { idx = index; }

 public:
  size_t idx;
  size_t s_idx;
  size_t e_idx;
  double max_kappa;
  CRoadType type;
};

struct PassageTypeRangeInfo {
 public:
  PassageTypeRangeInfo(size_t index = 0) { idx = index; }

 public:
  size_t idx;
  size_t s_idx;
  size_t e_idx;
  CPassageType type;
};

bool GenerateLatTypeRanges(
    const ReferencePathPoints& refer_path_points,
    std::vector<LatTypeRangeInfo>& turn_range_infos,
    std::vector<LatTypeRangeInfo>& straight_range_infos) {
  turn_range_infos.clear();
  straight_range_infos.clear();
  const size_t path_size = refer_path_points.size();
  if (path_size < 3) {
    return false;
  }
  // s1: generate turn_range_infos
  constexpr double kPeakKappaThr = 0.05;
  constexpr double kTurnKappaThr = 0.01;
  // s1-1: check peak
  for (size_t i = 1; i < path_size - 1; ++i) {
    const auto prev_kappa =
        std::fabs(refer_path_points[i - 1].path_point.kappa());
    const auto curr_kappa = std::fabs(refer_path_points[i].path_point.kappa());
    const auto next_kappa =
        std::fabs(refer_path_points[i + 1].path_point.kappa());
    if (curr_kappa > kPeakKappaThr && curr_kappa > prev_kappa &&
        curr_kappa > next_kappa) {
      turn_range_infos.push_back(LatTypeRangeInfo(i));
    }
  }
  // s1-2: generate range, ensure at least one point before/after peak
  for (auto& peak_kappa_range_info : turn_range_infos) {
    const int index = static_cast<int>(peak_kappa_range_info.idx);
    int start_idx = index - 1;
    int end_idx = index + 1;
    while (start_idx >= 0 &&
           std::fabs(refer_path_points[start_idx].path_point.kappa()) >
               kTurnKappaThr) {
      --start_idx;
    }
    while (end_idx < static_cast<int>(path_size) &&
           std::fabs(refer_path_points[end_idx].path_point.kappa()) >
               kTurnKappaThr) {
      ++end_idx;
    }
    int s_idx = start_idx + 1;
    int e_idx = end_idx - 1;
    // 保证至少两个点
    if (s_idx > index || e_idx < index) {
      continue;
    }
    peak_kappa_range_info.s_idx = static_cast<size_t>(s_idx);
    peak_kappa_range_info.e_idx = static_cast<size_t>(e_idx);
    peak_kappa_range_info.max_kappa =
        refer_path_points[index].path_point.kappa();
  }
  // 过滤掉未成功生成范围的峰值
  turn_range_infos.erase(
      std::remove_if(turn_range_infos.begin(), turn_range_infos.end(),
                     [](const LatTypeRangeInfo& info) {
                       return info.s_idx >= info.e_idx;
                     }),
      turn_range_infos.end());
  if (!turn_range_infos.empty()) {
    // s1-3: sort and merge duplicated range
    std::sort(turn_range_infos.begin(), turn_range_infos.end(),
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
        if (std::fabs(cur_info.max_kappa) > std::fabs(res_info.max_kappa)) {
          turn_range_infos[res_idx].max_kappa = cur_info.max_kappa;
        }
        turn_range_infos[res_idx].e_idx = cur_info.e_idx;
      }
    }
    turn_range_infos.resize(res_idx + 1);
  }
  // s2: generate straight_range_infos
  if (turn_range_infos.empty()) {
    straight_range_infos.push_back(LatTypeRangeInfo(0));
    straight_range_infos.back().s_idx = 0;
    straight_range_infos.back().e_idx = refer_path_points.size() - 1;
    straight_range_infos.back().type = CRoadType::NormalStraight;
  } else {
    size_t s_idx = 0;
    for (const auto& turn_range_info : turn_range_infos) {
      if (turn_range_info.s_idx > s_idx) {
        straight_range_infos.push_back(LatTypeRangeInfo(s_idx));
        straight_range_infos.back().s_idx = s_idx;
        straight_range_infos.back().e_idx = turn_range_info.s_idx;
        straight_range_infos.back().type = CRoadType::NormalStraight;
        s_idx = turn_range_info.e_idx;
      }
    }
    if (turn_range_infos.back().e_idx < path_size - 1) {
      straight_range_infos.push_back(LatTypeRangeInfo(s_idx));
      straight_range_infos.back().s_idx = turn_range_infos.back().e_idx;
      straight_range_infos.back().e_idx = path_size - 1;
      straight_range_infos.back().type = CRoadType::NormalStraight;
    }
  }

  // s3: merge short straight_range_infos into turn_range_infos
  constexpr double kShortStraightLenThr = 2.0;
  for (int i = 0; i < static_cast<int>(straight_range_infos.size()); ++i) {
    const size_t str_s = straight_range_infos[i].s_idx;
    const size_t str_e = straight_range_infos[i].e_idx;
    double str_len = refer_path_points[str_e].path_point.s() -
                     refer_path_points[str_s].path_point.s();
    if (str_len >= kShortStraightLenThr) {
      continue;
    }

    int prev_turn_idx = -1;
    int next_turn_idx = -1;
    for (int j = 0; j < static_cast<int>(turn_range_infos.size()); ++j) {
      if (turn_range_infos[j].e_idx == str_s) prev_turn_idx = j;
      if (turn_range_infos[j].s_idx == str_e) next_turn_idx = j;
    }

    // 将前后 turn_range_infos 和 straight_range_infos 段合成一段
    if (prev_turn_idx != -1 && next_turn_idx != -1) {
      turn_range_infos[prev_turn_idx].e_idx =
          turn_range_infos[next_turn_idx].e_idx;
      if (std::fabs(turn_range_infos[next_turn_idx].max_kappa) >
          std::fabs(turn_range_infos[prev_turn_idx].max_kappa)) {
        turn_range_infos[prev_turn_idx].max_kappa =
            turn_range_infos[next_turn_idx].max_kappa;
      }
      turn_range_infos.erase(turn_range_infos.begin() + next_turn_idx);
      if (next_turn_idx < prev_turn_idx) --prev_turn_idx;
    } else if (prev_turn_idx != -1) {
      turn_range_infos[prev_turn_idx].e_idx = str_e;
    } else if (next_turn_idx != -1) {
      turn_range_infos[next_turn_idx].s_idx = str_s;
    } else {
      continue;
    }

    straight_range_infos.erase(straight_range_infos.begin() + i);
    --i;
  }
  return true;
}

bool DetermineTurnTypeAndRange(
    const ReferencePathPoints& refer_path_points,
    std::vector<LatTypeRangeInfo>& turn_range_infos) {
  constexpr double kDegToRad = M_PI / 180.0;


  const auto get_delta_heading =
      [&refer_path_points](const LatTypeRangeInfo& info) {
        if (info.e_idx <= info.s_idx ||
            info.e_idx >= refer_path_points.size()) {
          return 0.0;
        }
        double sum_abs_delta = 0.0;
        double prev_theta = refer_path_points[info.s_idx].path_point.theta();
        for (size_t i = info.s_idx + 1; i <= info.e_idx; ++i) {
          const double curr_theta = refer_path_points[i].path_point.theta();
          sum_abs_delta +=
              std::fabs(ad_common::math::AngleDiff(prev_theta, curr_theta));
          prev_theta = curr_theta;
        }
        return sum_abs_delta;
      };

   // 螺旋弯：大转角 + 长弧长
  auto is_curve_turn = [&get_delta_heading,&refer_path_points](
                           const LatTypeRangeInfo& turn_range_info) {
    const double s_start =
        refer_path_points[turn_range_info.s_idx].path_point.theta();

    constexpr double kCurveTurnAngleMax = 200.0 * kDegToRad;
    constexpr double kCurveTurnLengthMax = 16.0;
    double seg_len = refer_path_points[turn_range_info.e_idx].path_point.s() -
                     refer_path_points[turn_range_info.s_idx].path_point.s();
    return (get_delta_heading(turn_range_info) > kCurveTurnAngleMax) && (seg_len > kCurveTurnLengthMax);
  };

  // U型弯：转角 > 160°
  auto is_u_turn = [&get_delta_heading,
                    kDegToRad](const LatTypeRangeInfo& info) {
    return get_delta_heading(info) > 160.0 * kDegToRad;
  };
  // 钝角弯：100° < 转角 < 160°
  auto is_wide_turn = [&get_delta_heading,
                       kDegToRad](const LatTypeRangeInfo& info) {
    double d = get_delta_heading(info);
    return d > 100.0 * kDegToRad && d < 160.0 * kDegToRad;
  };
  // 直角：70° < 转角 < 100°
  auto is_normal_turn = [&get_delta_heading,
                         kDegToRad](const LatTypeRangeInfo& info) {
    double d = get_delta_heading(info);
    return d > 70.0 * kDegToRad && d < 100.0 * kDegToRad;
  };
  // 锐角弯：20° < 转角 < 70°
  auto is_sharp_turn = [&get_delta_heading,
                        kDegToRad](const LatTypeRangeInfo& info) {
    double d = get_delta_heading(info);
    return d > 20.0 * kDegToRad && d < 70.0 * kDegToRad;
  };
  // 绕障：转角 < 10°
  auto is_nudge_turn = [&get_delta_heading,
                        kDegToRad](const LatTypeRangeInfo& info) {
    return get_delta_heading(info) < 10.0 * kDegToRad;
  };
  // S弯：弧长长且曲率正负异号
  auto is_s_turn = [&refer_path_points](
                       const LatTypeRangeInfo& turn_range_info) {
    constexpr double kPeakKappaThr = 0.1;
    constexpr double kSCurveMinLength = 16.0;
    double kappa_min = 10.0;
    double kappa_max = -10.0;
    for (int j = turn_range_info.s_idx; j <= turn_range_info.e_idx; ++j) {
      double k = refer_path_points[j].path_point.kappa();
      if (k > kappa_max) kappa_max = k;
      if (k < kappa_min) kappa_min = k;
    }
    double seg_len = refer_path_points[turn_range_info.e_idx].path_point.s() -
                     refer_path_points[turn_range_info.s_idx].path_point.s();
    return seg_len > kSCurveMinLength && kappa_max * kappa_min < -0.001 &&
           kappa_max > kPeakKappaThr && kappa_min < -kPeakKappaThr;
  };

  for (auto& turn_range_info : turn_range_infos) {
    if (is_curve_turn(turn_range_info)) {
      turn_range_info.type = CRoadType::CurveTurn;
    } else if (is_u_turn(turn_range_info)) {
      turn_range_info.type = CRoadType::UTurn;
    } else if (is_wide_turn(turn_range_info)) {
      turn_range_info.type = CRoadType::WideTurn;
    } else if (is_normal_turn(turn_range_info)) {
      turn_range_info.type = CRoadType::NormalTurn;
    } else if (is_s_turn(turn_range_info)) {
      turn_range_info.type = CRoadType::SCurveStaight;
    } else if (is_sharp_turn(turn_range_info)) {
      turn_range_info.type = CRoadType::SharpTurn;
    } else if (is_nudge_turn(turn_range_info)) {
      turn_range_info.type = CRoadType::NudgeStraight;
    } else {
      turn_range_info.type = CRoadType::Unknown;
    }
  }
  return true;
}

bool StaticAnalysisUtils::RoadTypeAnalysis(
    const ReferencePathPoints& refer_path_points,
    planning_math::ConstKDPathPtr kd_path,
    StaticAnalysisStoragePtr static_analysis_storage) {
  (void)kd_path;
  if (!static_analysis_storage) return false;
  if (refer_path_points.empty()) return true;

  // S1：check kappa peak value and index

  // S2: generate straight and  range
  std::vector<LatTypeRangeInfo> turn_range_infos;
  std::vector<LatTypeRangeInfo> straight_range_infos;
  GenerateLatTypeRanges(refer_path_points, turn_range_infos,
                        straight_range_infos);

  // s4:determine turn type and range
  DetermineTurnTypeAndRange(refer_path_points, turn_range_infos);

  // S5: generate road lat type result
  std::vector<LatTypeRangeInfo> final_range_info;
  final_range_info.reserve(turn_range_infos.size() +
                           straight_range_infos.size());
  final_range_info.insert(final_range_info.end(), turn_range_infos.begin(),
                          turn_range_infos.end());
  final_range_info.insert(final_range_info.end(), straight_range_infos.begin(),
                          straight_range_infos.end());

  std::unordered_map<CRoadType, SRangeList> lat_type_to_s_range_list;
  for (const auto& range_info : final_range_info) {
    if (range_info.s_idx >= refer_path_points.size() ||
        range_info.e_idx >= refer_path_points.size() ||
        range_info.s_idx > range_info.e_idx) {
      continue;
    }
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

bool GenerateWidthRanges(
    const ReferencePathPoints& refer_path_points,
    std::vector<PassageTypeRangeInfo>& wide_passage_range_infos,
    std::vector<PassageTypeRangeInfo>& normal_passage_range_infos,
    std::vector<PassageTypeRangeInfo>& narrow_passage_range_infos) {
  const size_t refer_path_points_num = refer_path_points.size();
  if (refer_path_points_num == 0) return true;
  wide_passage_range_infos.clear();
  normal_passage_range_infos.clear();
  narrow_passage_range_infos.clear();
  constexpr double kWideStartThr = 6.0;
  constexpr double kWideEndThr = 5.5;

  // 从第一点开始遍历，lane_width > 6 为 wide 起点，lane_width < 5.5 为 wide
  // 终点，仅保留长度 > 1.0
  constexpr double kWideMinLenThr = 1.0;
  bool in_wide = false;
  size_t wide_s_idx = 0;
  for (size_t i = 0; i < refer_path_points.size(); ++i) {
    const auto curr_width = std::fabs(refer_path_points[i].drivable_width);
    if (in_wide) {
      if (curr_width < kWideEndThr) {
        size_t e_idx = i > 0 ? i - 1 : 0;
        double len = refer_path_points[e_idx].path_point.s() -
                     refer_path_points[wide_s_idx].path_point.s();
        if (len > kWideMinLenThr) {
          PassageTypeRangeInfo info(wide_s_idx);
          info.s_idx = wide_s_idx;
          info.e_idx = e_idx;
          info.type = CPassageType::WidedPassage;
          wide_passage_range_infos.push_back(info);
        }
        in_wide = false;
      }
    } else {
      if (curr_width > kWideStartThr) {
        in_wide = true;
        wide_s_idx = i;
      }
    }
  }
  if (in_wide && !refer_path_points.empty()) {
    size_t e_idx = refer_path_points.size() - 1;
    double len = refer_path_points[e_idx].path_point.s() -
                 refer_path_points[wide_s_idx].path_point.s();
    if (len > kWideMinLenThr) {
      PassageTypeRangeInfo info(wide_s_idx);
      info.s_idx = wide_s_idx;
      info.e_idx = e_idx;
      info.type = CPassageType::WidedPassage;
      wide_passage_range_infos.push_back(info);
    }
  }

  constexpr double kNarrowStartThr = 3.0;
  constexpr double kNarrowEndThr = 4.0;
  constexpr double kNarrowMinLenThr = 1.0;
  // 从第一点开始遍历，lane_width < 3 为 narrow 起点，lane_width > 4 为 narrow
  // 终点
  bool in_narrow = false;
  size_t narrow_s_idx = 0;
  for (size_t i = 0; i < refer_path_points.size(); ++i) {
    const auto curr_width = std::fabs(refer_path_points[i].drivable_width);
    if (in_narrow) {
      if (curr_width > kNarrowEndThr) {
        size_t e_idx = i > 0 ? i - 1 : 0;
        double len = refer_path_points[e_idx].path_point.s() -
                     refer_path_points[narrow_s_idx].path_point.s();
        if (len > kNarrowMinLenThr) {
          PassageTypeRangeInfo info(narrow_s_idx);
          info.s_idx = narrow_s_idx;
          info.e_idx = e_idx;
          info.type = CPassageType::NarrowPassage;
          narrow_passage_range_infos.push_back(info);
        }
        in_narrow = false;
      }
    } else {
      if (curr_width < kNarrowStartThr) {
        in_narrow = true;
        narrow_s_idx = i;
      }
    }
  }
  if (in_narrow && !refer_path_points.empty()) {
    size_t e_idx = refer_path_points.size() - 1;
    double len = refer_path_points[e_idx].path_point.s() -
                 refer_path_points[narrow_s_idx].path_point.s();
    if (len > kNarrowMinLenThr) {
      PassageTypeRangeInfo info(narrow_s_idx);
      info.s_idx = narrow_s_idx;
      info.e_idx = e_idx;
      info.type = CPassageType::NarrowPassage;
      narrow_passage_range_infos.push_back(info);
    }
  }

  // refer_path_points 未被 wide/narrow 覆盖的区间存入
  // normal_passage_range_infos
  std::vector<std::pair<size_t, size_t>> ranges_covered;
  for (const auto& r : wide_passage_range_infos) {
    ranges_covered.push_back({r.s_idx, r.e_idx});
  }
  for (const auto& r : narrow_passage_range_infos) {
    ranges_covered.push_back({r.s_idx, r.e_idx});
  }
  std::sort(ranges_covered.begin(), ranges_covered.end());
  std::vector<std::pair<size_t, size_t>> ranges_covered_merged;
  for (const auto& range_covered : ranges_covered) {
    if (ranges_covered_merged.empty() ||
        range_covered.first > ranges_covered_merged.back().second + 1) {
      ranges_covered_merged.push_back(range_covered);
    } else if (range_covered.second > ranges_covered_merged.back().second) {
      ranges_covered_merged.back().second = range_covered.second;
    }
  }

  if (ranges_covered_merged.empty()) {
    PassageTypeRangeInfo info(0);
    info.s_idx = 0;
    info.e_idx = refer_path_points_num - 1;
    info.type = CPassageType::NormalPassage;
    normal_passage_range_infos.push_back(info);
  } else {
    size_t last_end = 0;
    for (const auto& covered_merged : ranges_covered_merged) {
      if (covered_merged.first > last_end) {
        PassageTypeRangeInfo info(last_end);
        info.s_idx = last_end;
        info.e_idx = covered_merged.first - 1;
        info.type = CPassageType::NormalPassage;
        normal_passage_range_infos.push_back(info);
      }
      if (covered_merged.second + 1 > last_end) {
        last_end = covered_merged.second + 1;
      }
    }
    if (last_end < refer_path_points_num) {
      PassageTypeRangeInfo info(last_end);
      info.s_idx = last_end;
      info.e_idx = refer_path_points_num - 1;
      info.type = CPassageType::NormalPassage;
      normal_passage_range_infos.push_back(info);
    }
  }

  // 长度小于 1.0 的 normal 段：类型一致则三段合成一段，否则融入前后较长的 range
  constexpr double kShortNormalLenThr = 1.0;
  for (int i = 0; i < static_cast<int>(normal_passage_range_infos.size());
       ++i) {
    const auto& norm = normal_passage_range_infos[i];
    double len = refer_path_points[norm.e_idx].path_point.s() -
                 refer_path_points[norm.s_idx].path_point.s();
    if (len >= kShortNormalLenThr) continue;

    int prev_wide_idx = -1, prev_narrow_idx = -1;
    int next_wide_idx = -1, next_narrow_idx = -1;
    if (norm.s_idx > 0) {
      size_t prev_e = norm.s_idx - 1;
      for (int j = 0; j < static_cast<int>(wide_passage_range_infos.size());
           ++j) {
        if (wide_passage_range_infos[j].e_idx == prev_e) prev_wide_idx = j;
      }
      for (int j = 0; j < static_cast<int>(narrow_passage_range_infos.size());
           ++j) {
        if (narrow_passage_range_infos[j].e_idx == prev_e) prev_narrow_idx = j;
      }
    }
    if (norm.e_idx + 1 < refer_path_points_num) {
      size_t next_s = norm.e_idx + 1;
      for (int j = 0; j < static_cast<int>(wide_passage_range_infos.size());
           ++j) {
        if (wide_passage_range_infos[j].s_idx == next_s) next_wide_idx = j;
      }
      for (int j = 0; j < static_cast<int>(narrow_passage_range_infos.size());
           ++j) {
        if (narrow_passage_range_infos[j].s_idx == next_s) next_narrow_idx = j;
      }
    }

    bool prev_wide = (prev_wide_idx >= 0), prev_narrow = (prev_narrow_idx >= 0);
    bool next_wide = (next_wide_idx >= 0), next_narrow = (next_narrow_idx >= 0);

    if (prev_wide && next_wide) {
      wide_passage_range_infos[prev_wide_idx].e_idx =
          wide_passage_range_infos[next_wide_idx].e_idx;
      wide_passage_range_infos.erase(wide_passage_range_infos.begin() +
                                     next_wide_idx);
      if (next_wide_idx < prev_wide_idx) --prev_wide_idx;
    } else if (prev_narrow && next_narrow) {
      narrow_passage_range_infos[prev_narrow_idx].e_idx =
          narrow_passage_range_infos[next_narrow_idx].e_idx;
      narrow_passage_range_infos.erase(narrow_passage_range_infos.begin() +
                                       next_narrow_idx);
      if (next_narrow_idx < prev_narrow_idx) --prev_narrow_idx;
    } else if (prev_wide || next_wide || prev_narrow || next_narrow) {
      double prev_len = -1.0, next_len = -1.0;
      if (prev_wide) {
        prev_len =
            refer_path_points[wide_passage_range_infos[prev_wide_idx].e_idx]
                .path_point.s() -
            refer_path_points[wide_passage_range_infos[prev_wide_idx].s_idx]
                .path_point.s();
      } else if (prev_narrow) {
        prev_len =
            refer_path_points[narrow_passage_range_infos[prev_narrow_idx].e_idx]
                .path_point.s() -
            refer_path_points[narrow_passage_range_infos[prev_narrow_idx].s_idx]
                .path_point.s();
      }
      if (next_wide) {
        next_len =
            refer_path_points[wide_passage_range_infos[next_wide_idx].e_idx]
                .path_point.s() -
            refer_path_points[wide_passage_range_infos[next_wide_idx].s_idx]
                .path_point.s();
      } else if (next_narrow) {
        next_len =
            refer_path_points[narrow_passage_range_infos[next_narrow_idx].e_idx]
                .path_point.s() -
            refer_path_points[narrow_passage_range_infos[next_narrow_idx].s_idx]
                .path_point.s();
      }
      if (prev_len >= next_len && (prev_wide || prev_narrow)) {
        if (prev_wide) {
          wide_passage_range_infos[prev_wide_idx].e_idx = norm.e_idx;
        } else {
          narrow_passage_range_infos[prev_narrow_idx].e_idx = norm.e_idx;
        }
      } else if (next_wide || next_narrow) {
        if (next_wide) {
          wide_passage_range_infos[next_wide_idx].s_idx = norm.s_idx;
        } else {
          narrow_passage_range_infos[next_narrow_idx].s_idx = norm.s_idx;
        }
      }
    } else {
      continue;
    }
    normal_passage_range_infos.erase(normal_passage_range_infos.begin() + i);
    --i;
  }
  return true;
}

bool DetermineWidthTypeAndRange(
    const ReferencePathPoints& refer_path_points,
    std::vector<PassageTypeRangeInfo>& wide_passage_range_infos,
    std::vector<PassageTypeRangeInfo>& normal_passage_range_infos,
    std::vector<PassageTypeRangeInfo>& narrow_passage_range_infos) {
  for (auto& wide_passage_range_info : wide_passage_range_infos) {
    if (refer_path_points[wide_passage_range_info.e_idx].path_point.s() -
            refer_path_points[wide_passage_range_info.s_idx].path_point.s() >
        10.0) {
      wide_passage_range_info.type = CPassageType::WidedPassage;
    } else {
      wide_passage_range_info.type = CPassageType::ShortWidenPassage;
    }
  }

  for (auto& normal_passage_range_info : normal_passage_range_infos) {
    normal_passage_range_info.type = CPassageType::NormalPassage;
  }

  for (auto& narrow_range_info : narrow_passage_range_infos) {
    narrow_range_info.type = CPassageType::NarrowPassage;
  }
  return true;
}

bool StaticAnalysisUtils::PassageTypeAnalysis(
    const ReferencePathPoints& refer_path_points,
    planning_math::ConstKDPathPtr kd_path,
    StaticAnalysisStoragePtr static_analysis_storage) {
  (void)kd_path;
  if (!static_analysis_storage) return false;
  if (refer_path_points.empty()) return true;
  // S1：divid refer_path_points to
  // normal_passage_range_infos---wide_passage_range_infos---narrowarrow_range_infos
  std::vector<PassageTypeRangeInfo> wide_passage_range_infos;
  std::vector<PassageTypeRangeInfo> normal_passage_range_infos;
  std::vector<PassageTypeRangeInfo> narrow_passage_range_infos;
  GenerateWidthRanges(refer_path_points, wide_passage_range_infos,
                      normal_passage_range_infos, narrow_passage_range_infos);

  // s2: determine wide type and range
  DetermineWidthTypeAndRange(refer_path_points, wide_passage_range_infos,
                             normal_passage_range_infos,
                             narrow_passage_range_infos);

  // S3: generate road passage type result
  std::vector<PassageTypeRangeInfo> final_range_info;
  final_range_info.reserve(wide_passage_range_infos.size() +
                           normal_passage_range_infos.size() +
                           narrow_passage_range_infos.size());
  final_range_info.insert(final_range_info.end(),
                          wide_passage_range_infos.begin(),
                          wide_passage_range_infos.end());
  final_range_info.insert(final_range_info.end(),
                          normal_passage_range_infos.begin(),
                          normal_passage_range_infos.end());
  final_range_info.insert(final_range_info.end(),
                          narrow_passage_range_infos.begin(),
                          narrow_passage_range_infos.end());

  std::unordered_map<CPassageType, SRangeList> passage_type_to_s_range_list;
  for (const auto& range_info : final_range_info) {
    if (range_info.s_idx >= refer_path_points.size() ||
        range_info.e_idx >= refer_path_points.size() ||
        range_info.s_idx > range_info.e_idx) {
      continue;
    }
    auto& s_range_list = passage_type_to_s_range_list[range_info.type];
    s_range_list.push_back(
        std::make_pair(refer_path_points[range_info.s_idx].path_point.s(),
                       refer_path_points[range_info.e_idx].path_point.s()));
  }
  for (auto& item : passage_type_to_s_range_list) {
    sort(item.second.begin(), item.second.end(),
         [](const auto& a, const auto& b) { return a.first < b.first; });

    static_analysis_storage->SetTypeList(item.first, item.second);
  }

#ifdef ENABLE_IDX_RANGE_LIST_STORAGE
  std::unordered_map<CPassageType, IdxRangeList> passage_type_to_idx_range_list;
  for (const auto& range_info : final_range_info) {
    auto& idx_range_list = passage_type_to_idx_range_list[range_info.type];
    idx_range_list.push_back(
        std::make_pair(range_info.s_idx, range_info.e_idx));
  }
  for (auto& item : passage_type_to_idx_range_list) {
    sort(item.second.begin(), item.second.end(),
         [](const auto& a, const auto& b) { return a.first < b.first; });

    static_analysis_storage->SetTypeList(item.first, item.second);
  }
#endif
  return true;
}

bool GenerateObstacleRangeList(
    const std::vector<FrenetObstaclePtr>& frenet_obstacles,
    const ReferencePathPoints& refer_path_points,
    planning_math::ConstKDPathPtr kd_path, const double lat_valid_thr,
    const double lon_expand_thr, const double lon_merge_thr,
    const CElemType elem_type,
    StaticAnalysisStoragePtr static_analysis_storage) {
  const double refer_start_s = refer_path_points.front().path_point.s();
  const double refer_end_s = refer_path_points.back().path_point.s();
  SRangeList s_range_list;
  for (const auto& obj : frenet_obstacles) {
    const auto& frenet_boundary = obj->frenet_obstacle_boundary();
    if (frenet_boundary.s_start > refer_end_s ||
        frenet_boundary.s_end < refer_start_s) {
      continue;
    }
    if (frenet_boundary.l_start > lat_valid_thr ||
        frenet_boundary.l_end < -lat_valid_thr) {
      continue;
    }
    s_range_list.push_back(
        std::make_pair(frenet_boundary.s_start - lon_expand_thr,
                       frenet_boundary.s_end + lon_expand_thr));
  }
  if (s_range_list.empty() == false) {
    // 排序合并重复
    sort(s_range_list.begin(), s_range_list.end(),
         [](const auto& a, const auto& b) { return a.first < b.first; });
    size_t res_idx = 0;
    for (size_t curr_idx = 1; curr_idx < s_range_list.size(); ++curr_idx) {
      if (s_range_list[curr_idx].first <=
          s_range_list[res_idx].second + lon_merge_thr) {
        s_range_list[res_idx].second = s_range_list[curr_idx].second;
      } else {
        s_range_list[++res_idx] = s_range_list[curr_idx];
      }
    }
    s_range_list.resize(res_idx + 1);
  }
  static_analysis_storage->SetTypeList(elem_type, s_range_list);
#ifdef ENABLE_IDX_RANGE_LIST_STORAGE
  IdxRangeList idx_range_list;
  for (const auto& s_range : s_range_list) {
    idx_range_list.push_back(
        std::make_pair(kd_path->GetPathPointIdxByS(s_range.first),
                       kd_path->GetPathPointIdxByS(s_range.second)));
  }
  static_analysis_storage->SetTypeList(elem_type, idx_range_list);
#endif
  return true;
}

bool StaticAnalysisUtils::ElemTypeAnalysis(
    ConstReferencePathPtr refer_path_ptr,
    const ReferencePathPoints& refer_path_points,
    planning_math::ConstKDPathPtr kd_path,
    StaticAnalysisStoragePtr static_analysis_storage) {
  if (refer_path_points.empty()) {
    return true;
  }
  // S1: 坡道区域
  std::unordered_map<CElemType, IdxRangeList> ramp_idx_range_list;
  std::unordered_map<CElemType, SRangeList> ramp_s_range_list;
  CElemType last_ramp_type = CElemType::Unknown;
  std::pair<double, double> last_ramp_s_range;
  std::pair<size_t, size_t> last_ramp_idx_range;
  for (size_t i = 0; i < refer_path_points.size(); ++i) {
    if (refer_path_points[i].is_ramp) {
      CElemType curr_ramp = refer_path_points[i].ramp_slope > 0.0
                                ? CElemType::UpRampRoad
                                : CElemType::DownRampRoad;
      if (curr_ramp != last_ramp_type) {
        if (last_ramp_type == CElemType::UpRampRoad ||
            last_ramp_type == CElemType::DownRampRoad) {
          ramp_idx_range_list[last_ramp_type].push_back(last_ramp_idx_range);
          ramp_s_range_list[last_ramp_type].push_back(last_ramp_s_range);
        }
        last_ramp_type = curr_ramp;
        last_ramp_s_range = std::make_pair(refer_path_points[i].path_point.s(),
                                           refer_path_points[i].path_point.s());
        last_ramp_idx_range = std::make_pair(i, i);
      } else {
        last_ramp_s_range.second = refer_path_points[i].path_point.s();
        last_ramp_idx_range.second = i;
      }
    } else {
      last_ramp_type = CElemType::Unknown;
    }
  }
  if (last_ramp_type == CElemType::UpRampRoad ||
      last_ramp_type == CElemType::DownRampRoad) {
    ramp_idx_range_list[last_ramp_type].push_back(last_ramp_idx_range);
    ramp_s_range_list[last_ramp_type].push_back(last_ramp_s_range);
  }
  for (const auto& elem : ramp_s_range_list) {
    static_analysis_storage->SetTypeList(elem.first, elem.second);
#ifdef ENABLE_IDX_RANGE_LIST_STORAGE
    if (ramp_idx_range_list.find(elem.first) != ramp_idx_range_list.end()) {
      static_analysis_storage->SetTypeList(elem.first,
                                           ramp_idx_range_list.at(elem.first));
    }
#endif
  }

  // S2：减速带区域
  const auto& speed_bump_frenet_obstacles =
      refer_path_ptr->get_speed_bump_obstacles();
  constexpr double kValidSpeedBumpLatThr = 2.0;
  constexpr double kSpeedBumpLonExpandThr = 1.0;
  constexpr double kSpeedBumpLonMergeThr = 1.0;
  GenerateObstacleRangeList(speed_bump_frenet_obstacles, refer_path_points,
                            kd_path, kValidSpeedBumpLatThr,
                            kSpeedBumpLonExpandThr, kSpeedBumpLonMergeThr,
                            CElemType::SpeedBumpRoad, static_analysis_storage);

  // S3：路口区域
  const auto& semantic_sign_frenet_obstacles =
      refer_path_ptr->get_semantic_sign_obstacles();
  std::vector<FrenetObstaclePtr> intersection_frenet_obstacles;
  for (const auto& frenet_obstacle : semantic_sign_frenet_obstacles) {
    // if (frenet_obstacle->obstacle()->intersection_type() !=
    //     iflyauto::IntersectionType::INTERSECTION_UNKNOWN) {
    //   intersection_frenet_obstacles.push_back(frenet_obstacle);
    // }
    intersection_frenet_obstacles.push_back(frenet_obstacle);
  }
  constexpr double kValidIntersectionLatThr = 2.0;
  constexpr double kIntersectionLonExpandThr = 1.0;
  constexpr double kIntersectionLonMergeThr = 1.0;
  GenerateObstacleRangeList(intersection_frenet_obstacles, refer_path_points,
                            kd_path, kValidIntersectionLatThr,
                            kIntersectionLonExpandThr, kIntersectionLonMergeThr,
                            CElemType::IntersectionRoad,
                            static_analysis_storage);

  // S4：闸机区域
  const auto& turnstile_frenet_obstacles =
      refer_path_ptr->get_turnstile_obstacles();
  constexpr double kValidTurnstileLatThr = 2.0;
  constexpr double kTurnstileLonExpandThr = 1.0;
  constexpr double kTurnstileLonMergeThr = 1.0;
  GenerateObstacleRangeList(turnstile_frenet_obstacles, refer_path_points,
                            kd_path, kValidTurnstileLatThr,
                            kTurnstileLonExpandThr, kTurnstileLonMergeThr,
                            CElemType::TurnStileRoad, static_analysis_storage);
  return true;
}

}  // namespace planning