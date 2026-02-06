#pragma once

#include <functional>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/context/lane_reference_path.h"

namespace planning {

enum class RLatType {
  Unknown = 0,      // 不确定类型
  NornalStraight = 1,     // 正常直道
  NudgeStraight = 2,      // 绕障直道
  SCurveStaight = 3,      // S型直道

  NormalTurn = 10,   // 直角弯
  SharpTurn = 11,    // 锐角弯
  WideTurn = 12,     // 钝角弯
  UTurn = 13,        // 掉头弯
  CurveTurn = 14,    // 圆弧弯（比如扎道、螺旋坡道）

  Ignore = 30,       // 任意类型道路（该类型用于检索，类型判断中不会使用）
  Turn = 31          // 所有类型弯道（该类型用于检索，类型判断中不会使用）
};

enum class RWidthType {
  Unknown = 0,        // 不确定类型
  NormalPassage = 1,        // 正常宽度
  NarrowPassage = 2,        // 狭窄道路
  WidedPassage = 2,         // 宽敞道路
  ShortWidenPassage = 3,    // 短暂拓宽道路

  Ignore = 30               // 任意类型道路（该类型用于检索，类型判断中不会使用）
};

enum class RHeightType {
  Unknown = 0,            // 不确定类型
  NormalRoad = 1,         // 平坦路段
  UpRampRoad = 2,         // 上坡
  DownRampRoad = 3,       // 下坡
  UnknownRampRoad = 4,    // 不确定的坡道
  SpeedBumpRoad = 5,      // 减速带

  Ignore = 30,            // 任意类型道路（该类型用于检索，类型判断中不会使用）
  RampRoad = 31,          // 任意类型坡道（该类型用于检索，类型判断中不会使用）
};

enum class RTypeEnum {
  RLatType = 0,
  RWidthType = 1,
  RHeightType = 2
};

using SRangeList = std::vector<std::pair<double, double>>;
using IdxRangeList = std::vector<std::pair<size_t, size_t>>;
struct RTypeStorageItem {
  SRangeList s_range_list;
#ifdef ENABLE_IDX_RANGE_LIST
  IdxRanIdxRangeList idx_range_list;
#endif
};

using RoadLatTypeStorage = std::unordered_map<RLatType, RTypeStorageItem>;
using RoadWidthTypeStorage = std::unordered_map<RWidthType, RTypeStorageItem>;
using RoadHeightTypeStorage = std::unordered_map<RHeightType, RTypeStorageItem>;

/*某个位置的道路类型结果，用于检索，不用于存储*/
struct RoadTypeInfo {
 public:
  RoadTypeInfo(const RLatType& l_type = RLatType::Ignore,
               const RWidthType& w_type = RWidthType::Ignore,
               const RHeightType& h_type = RHeightType::Ignore) {
    lat_type = l_type;
    width_type = w_type;
    height_type = h_type;
  }

 public:
  RLatType lat_type = RLatType::Unknown;
  RWidthType width_type = RWidthType::Unknown;
  RHeightType height_type = RHeightType::Unknown;
};

class RoadTypeStorage {
 public:
  RoadTypeStorage() = default;

  void Clear();

#ifdef ENABLE_IDX_RANGE_LIST
  template <typename T>
  void SetTypeList(const RTypeEnum type_enum, T type, const SRangeList& s_range_list,
                   const IdxRangeList& idx_range_list);

  template <typename T>
  void SetTypeList(const RTypeEnum type_enum, T type, const IdxRangeList& idx_range_list);
#endif

  template <typename T>
  void SetTypeList(const RTypeEnum type_enum, T type, const SRangeList& s_range_list);

  /********************************** 检索接口 ******************************/
  /**
   * @brief given s, return road type at given s
   */
  RoadTypeInfo GetRoadTypeInfo(const double cur_s) const;

  /**
   * @brief Get the Intersection S Range List which satisfy road_type
   */
  SRangeList GetSRangeList(const RoadTypeInfo& road_type) const;

  /**
   * @brief Get the First Intersection S Range in front of cur_s
   */
  std::pair<double, double> GetFrontSRangeByRoadType(
      const RoadTypeInfo& road_type, const double cur_s) const;

  /**
   * @brief Get the First Intersection S Range back of cur_s
   */
  std::pair<double, double> GetBackSRangeByRoadType(
      const RoadTypeInfo& road_type, const double cur_s) const;

  /**
   * @brief Get the Union S Range List which satisfy road_types
   * @return SRangeList
   */
  SRangeList GetUnionSRangeList(const std::vector<RoadTypeInfo>& road_types) const;

  /**
   * @brief Get the First Intersection Idx Range in front of cur_s
   */
  std::pair<double, double> GetFrontUnionSRangeByRoadType(
      const std::vector<RoadTypeInfo>& road_types, const double cur_s) const;

  /**
   * @brief Get the First Intersection Idx Range back of cur_index
   */
  std::pair<double, double> GetBackUnionSRangeByRoadType(
      const std::vector<RoadTypeInfo>& road_types, const double cur_s) const;


#ifdef ENABLE_IDX_RANGE_LIST
  RoadTypeInfo GetRoadTypeInfo(const size_t cur_idx) const;

  IdxRangeList GetIdxRangeList(const RoadTypeInfo& road_type) const;

  std::pair<size_t, size_t> GetFrontIdxRangeByRoadType(
      const RoadTypeInfo& road_type, const size_t cur_index) const;

  std::pair<size_t, size_t> GetBackIdxRangeByRoadType(
      const RoadTypeInfo& road_type, const size_t cur_index) const;

  IdxRangeList GetUnionIdxRangeList(
      const std::vector<RoadTypeInfo>& road_types) const;

  std::pair<size_t, size_t> GetFrontUnionIdxRangeByRoadType(
      const std::vector<RoadTypeInfo>& road_types,
      const size_t cur_index) const;

  std::pair<size_t, size_t> GetBackUnionIdxRangeByRoadType(
      const std::vector<RoadTypeInfo>& road_types,
      const size_t cur_index) const;
#endif

private:
 template <typename T>
 std::vector<std::pair<T, T>> CalcIntervalIntersection(
     std::vector<std::vector<std::pair<T, T>>>& range_lists) const;

 template <typename T>
 std::vector<std::pair<T, T>> CalcIntervalUnion(
     std::vector<std::pair<T, T>>& range_lists) const;

#ifdef ENABLE_IDX_RANGE_LIST
 template <typename T1>
 T1 GetRoadType(
     const std::unordered_map<T1, RTypeStorageItem>& road_type_2_range_list,
     const size_t pos) const;
#endif

 template <typename T1>
 T1 GetRoadType(
     const std::unordered_map<T1, RTypeStorageItem>& type_2_storage_item,
     const double s) const;

private:
  ConstReferencePathPtr ref_path_ptr = nullptr;

  RoadLatTypeStorage lat_type_storage_;
  RoadWidthTypeStorage width_type_storage_;
  RoadHeightTypeStorage height_type_storage_;

private:
  static std::vector<RLatType> turn_type_list;
  static std::vector<RHeightType> ramp_type_list;
};
} // namespace planning
