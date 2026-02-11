#pragma once

#include <functional>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
#include <cmath>

#include "log_glog.h"

namespace planning {

// 道路趋势类型，每个位置类型唯一
enum class CRoadType {
  Unknown = 0,         // 不确定类型
  NornalStraight = 1,  // 正常直道
  NudgeStraight = 2,   // 绕障直道
  SCurveStaight = 3,   // S型直道

  NormalTurn = 10,  // 直角弯
  SharpTurn = 11,   // 锐角弯
  WideTurn = 12,    // 钝角弯
  UTurn = 13,       // 掉头弯
  CurveTurn = 14,   // 圆弧弯（比如扎道、螺旋坡道）

  Ignore = 30,  // 任意类型道路（该类型用于检索，类型判断中不会使用）
  Turn = 31  // 所有类型弯道（该类型用于检索，类型判断中不会使用）
};

// 通道宽度类型，每个位置唯一
enum class CPassageType {
  Unknown = 0,            // 不确定类型
  NormalPassage = 1,      // 正常宽度
  NarrowPassage = 2,      // 狭窄道路
  WidedPassage = 2,       // 宽敞道路
  ShortWidenPassage = 3,  // 短暂拓宽道路

  Ignore = 30  // 任意类型道路（该类型用于检索，类型判断中不会使用）
};

// 道路元素类型，每个位置可能不唯一
enum class CElemType {
  Unknown = 0,           // 不确定类型
  NormalRoad = 1,        // 平坦路段
  UpRampRoad = 2,        // 上坡
  DownRampRoad = 3,      // 下坡
  UnknownRampRoad = 4,   // 不确定的坡道
  SpeedBumpRoad = 5,     // 减速带
  IntersectionRoad = 6,  // 路口

  Ignore = 30,  // 任意类型道路（该类型用于检索，类型判断中不会使用）
  RampRoad = 31,  // 任意类型坡道（该类型用于检索，类型判断中不会使用）
};

using SRangeList = std::vector<std::pair<double, double>>;
using IdxRangeList = std::vector<std::pair<size_t, size_t>>;
struct StorageItem {
  SRangeList s_range_list;
#ifdef ENABLE_IDX_RANGE_LIST
  IdxRanIdxRangeList idx_range_list;
#endif
};

using RoadTypeStorage = std::unordered_map<CRoadType, StorageItem>;
using PassageTypeStorage = std::unordered_map<CPassageType, StorageItem>;
using ElemTypeStorage = std::unordered_map<CElemType, StorageItem>;

/*某个位置的道路类型结果，用于检索输入，不用于存储*/
struct QueryTypeInfo {
 public:
  QueryTypeInfo(const CRoadType& r_type = CRoadType::Ignore,
                const CPassageType& p_type = CPassageType::Ignore,
                const CElemType& e_type = CElemType::Ignore) {
    road_type = r_type;
    passage_type = p_type;
    elem_type = e_type;
  }

 public:
  CRoadType road_type = CRoadType::Unknown;
  CPassageType passage_type = CPassageType::Unknown;
  CElemType elem_type = CElemType::Unknown;
};

/*某个位置的道路类型结果，用于检索返回，不用于存储*/
struct ResultTypeInfo {
 public:
  ResultTypeInfo(const CRoadType& r_type = CRoadType::Ignore,
                 const CPassageType& p_type = CPassageType::Ignore,
                 const std::vector<CElemType>& e_types = {}) {
    road_type = r_type;
    passage_type = p_type;
    elem_types = e_types;
  }

 public:
  CRoadType road_type = CRoadType::Unknown;
  CPassageType passage_type = CPassageType::Unknown;
  std::vector<CElemType> elem_types;
};

class StaticAnalysisStorage {
 public:
  StaticAnalysisStorage() = default;

  void Clear();

#ifdef ENABLE_IDX_RANGE_LIST
  template <typename T>
  void SetTypeList(T type, const SRangeList& s_range_list,
                   const IdxRangeList& idx_range_list);

  template <typename T>
  void SetTypeList(T type, const IdxRangeList& idx_range_list);
#endif

  template <typename T>
  void SetTypeList(T type, const SRangeList& s_range_list);

  /********************************** 检索接口 ******************************/
  /**
   * @brief given s, return road type at given s
   */
  ResultTypeInfo GetTypeInfo(const double cur_s) const;

  /**
   * @brief Get the Intersection S Range List which satisfy road_type
   */
  SRangeList GetSRangeList(const QueryTypeInfo& road_type) const;

  /**
   * @brief Get the First Intersection S Range in front of cur_s
   */
  std::pair<double, double> GetFrontSRange(
      const QueryTypeInfo& road_type, const double cur_s) const;

  /**
   * @brief Get the First Intersection S Range back of cur_s
   */
  std::pair<double, double> GetBackSRange(
      const QueryTypeInfo& road_type, const double cur_s) const;

  /**
   * @brief Get the Union S Range List which satisfy road_types
   * @return SRangeList
   */
  SRangeList GetUnionSRangeList(
      const std::vector<QueryTypeInfo>& road_types) const;

  /**
   * @brief Get the First Intersection Idx Range in front of cur_s
   */
  std::pair<double, double> GetFrontUnionSRange(
      const std::vector<QueryTypeInfo>& road_types, const double cur_s) const;

  /**
   * @brief Get the First Intersection Idx Range back of cur_index
   */
  std::pair<double, double> GetBackUnionSRange(
      const std::vector<QueryTypeInfo>& road_types, const double cur_s) const;

#ifdef ENABLE_IDX_RANGE_LIST
  ResultTypeInfo GetTypeInfo(const size_t cur_idx) const;

  IdxRangeList GetIdxRangeList(const QueryTypeInfo& road_type) const;

  std::pair<size_t, size_t> GetFrontIdxRange(
      const QueryTypeInfo& road_type, const size_t cur_index) const;

  std::pair<size_t, size_t> GetBackIdxRange(
      const QueryTypeInfo& road_type, const size_t cur_index) const;

  IdxRangeList GetUnionIdxRangeList(
      const std::vector<QueryTypeInfo>& road_types) const;

  std::pair<size_t, size_t> GetFrontUnionIdxRange(
      const std::vector<QueryTypeInfo>& road_types,
      const size_t cur_index) const;

  std::pair<size_t, size_t> GetBackUnionIdxRange(
      const std::vector<QueryTypeInfo>& road_types,
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
  T1 GetTypeInfo(const std::unordered_map<T1, StorageItem>& type_2_storage_item,
                 const double s) const;

  template <typename T1>
  std::vector<T1> GetTypeInfos(const std::unordered_map<T1, StorageItem>& type_2_storage_item,
                 const double s) const;

 private:
  RoadTypeStorage road_type_storage_;
  PassageTypeStorage passage_type_storage_;
  ElemTypeStorage elem_type_storage_;

 private:
  static std::vector<CRoadType> turn_type_list;
  static std::vector<CElemType> ramp_type_list;
};

using StaticAnalysisStoragePtr = std::shared_ptr<StaticAnalysisStorage>;
using ConstStaticAnalysisStoragePtr =
    std::shared_ptr<const StaticAnalysisStorage>;
}  // namespace planning
