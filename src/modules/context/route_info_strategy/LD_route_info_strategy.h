#pragma once
# include <iostream>
#include "local_view.h"
#include "route_info_strategy.h"
// #include "hdmap/hdmap.h"
// #include "sdmap/sdmap.h"
// #include "sdpromap/sdpromap.h"
#include "lane_topo_graph.h"

namespace planning{
class LDRouteInfoStrategy : public RouteInfoStrategy{
public:

LDRouteInfoStrategy(const MLCDeciderConfig* config_builder,
                   const planning::framework::Session* session);

void Update(RouteInfoOutput& route_info_output) override;

void CalculateMLCDecider(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    RouteInfoOutput& route_info_output) override;

bool get_sdpromap_valid() override;

const ad_common::sdpromap::SDProMap& get_sdpro_map() override;
const iflymapdata::sdpro::LinkInfo_Link* get_current_link() override;

//todo(ldh): 其他virutal函数。
protected:
  bool UpdateLDMap();

  bool CalculateRouteInfo();

  bool CalculateCurrentLink();

  bool IsInExpressWay();

  struct EgoPositionResult {
    bool is_between_links;          // 横向在两条link之间
    bool is_left_of_link1;           // 在link1左侧
    bool is_right_of_link1;          // 在link1右侧
    bool is_left_of_link2;           // 在link2左侧
    bool is_right_of_link2;          // 在link2右侧
    double dist_to_link1;            // 到link1的距离（线段距离）
    double dist_to_link2;            // 到link2的距离（线段距离）
    double min_dist_to_link1;        // 到link1所有形点的最小距离
    double min_dist_to_link2;        // 到link2所有形点的最小距离
    int min_dist_idx_link1;          // link1上最近形点的索引
    int min_dist_idx_link2;          // link2上最近形点的索引
    double proj_ratio_link1;         // 自车在link1上的投影比例
    double proj_ratio_link2;         // 自车在link2上的投影比例
  };

  bool CheckEgoPositionRelativeTwoLinks(
      const iflymapdata::sdpro::LinkInfo_Link* link1,
      const iflymapdata::sdpro::LinkInfo_Link* link2,
      EgoPositionResult& result) const;

  bool IsNearingSplit();

 bool IsNearingRamp();

 bool IsNearingMerge();

 bool IsTwoSplitClose();

 void CalculateMergeInfo();
 void CalculateSplitInfo();
 void CalculateRampInfo();

 bool IsIgnoreMerge(const std::pair<const iflymapdata::sdpro::LinkInfo_Link*,
                                    double>& merge_info) const;

 bool IsValidInputLanes(
     const iflymapdata::sdpro::LinkInfo_Link* link,
     const std::vector<iflymapdata::sdpro::Lane>& start_lane_vec);

 void MLCSceneTypeDecider();

 bool CalculateFeasibleLaneGraph(
     TopoLinkGraph& feasible_lane_graph,
     const std::vector<iflymapdata::sdpro::Lane>& start_lane_vec,
     const iflymapdata::sdpro::LinkInfo_Link& target_link);

  bool CalculateExtenedFeasibleLane(TopoLinkGraph& feasible_lane_graph);

 bool SortLaneBaseSeq(std::vector<iflymapdata::sdpro::Lane>& start_lane_vec);

 void UpdateLCNumTask(
     const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
     const TopoLinkGraph& feasible_lane_graph);

 bool CalculateFrontTargetLinkBaseFixDis(
     iflymapdata::sdpro::LinkInfo_Link* target_link,
     std::vector<iflymapdata::sdpro::Lane>& start_lane_vec,
     const iflymapdata::sdpro::LinkInfo_Link* cur_link,
     const MLCSceneType scene);

 bool CalculateMergePreFeasibleLane(
     std::vector<iflymapdata::sdpro::Lane>& feasible_lane_vec,
     const TopoLinkGraph& feasible_lane_graph,
     const iflymapdata::sdpro::LinkInfo_Link* merge_link);

 bool IsEmergencyLane(const iflymapdata::sdpro::Lane* lane_info) const;
 bool IsAccelerateLane(const iflymapdata::sdpro::Lane* lane_info) const;
 bool IsDecelerateLane(const iflymapdata::sdpro::Lane* lane_info) const;
 bool IsEntryLane(const iflymapdata::sdpro::Lane* lane_info) const;
 bool IsDiversionLane(const iflymapdata::sdpro::Lane* lane_info) const;
 bool IsExitLane(const iflymapdata::sdpro::Lane* lane_info) const;
 bool IsMergeLane(const iflymapdata::sdpro::Lane* lane_info) const;
 bool HasLaneId(const std::vector<iflymapdata::sdpro::Lane>& lane_vec,
              uint64 target_id) const;

 bool IsInvalidLane(const iflymapdata::sdpro::Lane* lane_info) const;

 bool CalculateFeasibleLaneInRampScene(TopoLinkGraph& feasible_lane_graph, TopoLinkGraph& feasible_lane_graph_after_topo_change_vec);
 bool CalculateFeasibleLaneInMergeScene(TopoLinkGraph& feasible_lane_graph, TopoLinkGraph& feasible_lane_graph_after_topo_change_vec);
 bool CalculateFeasibleLaneInNormalScene(TopoLinkGraph& feasible_lane_graph);
 void ProcessLaneDistance(
    const std::shared_ptr<VirtualLane>& relative_id_lane,
    const std::unordered_map<int, double>& feasible_lane_distance);
void ProcessLaneMapMergePoint(
    const std::shared_ptr<VirtualLane>& relative_id_lane,
    const std::unordered_map<int, MapMergePointInfo>& map_merge_point_info);

 void CaculateDistanceToRoadEnd(
     const iflymapdata::sdpro::LinkInfo_Link* segment, const double nearest_s);
 void CaculateDistanceToTollStation(
     const iflymapdata::sdpro::LinkInfo_Link* segment, const double nearest_s);
 bool IsLaneSuccessorInPlannedRoute(const iflymapdata::sdpro::Lane* lane_info);
 bool IsLaneSuccessorIsMergeLane(const iflymapdata::sdpro::Lane* lane_info) const;
 const iflymapdata::sdpro::LinkInfo_Link* FindFrontValidRampSplitLink() const;
 void CalculateAvoidMergeFeasibleLane(TopoLinkGraph& feasible_lane_graph);
 void Erase1Split2FeasibleLane(TopoLinkGraph& feasible_lane_graph);
 std::vector<TopoLane> CalculateMaxDistanceLanes(const TopoLinkGraph& feasible_lane_graph) const;
 const iflymapdata::sdpro::Lane* IsEntryLanePresentOnEitherSideOfSuccessorLane(
     const iflymapdata::sdpro::Lane* cur_link_lane_info);
  // pair<left_lane, right_lane>
  std::pair<const iflymapdata::sdpro::Lane*, const iflymapdata::sdpro::Lane*>
  FindLaneLeftRightNeighbors(const iflymapdata::sdpro::Lane* target_lane);
  bool IsInvalidLaneMergeLaneOppositeSide(
      const iflymapdata::sdpro::Lane* merge_lane);
  bool IsInvalidNonDrivingLane(const iflymapdata::sdpro::Lane* lane);
  MergeLaneType CalculateMergeLaneType(const iflymapdata::sdpro::Lane* merge_lane);
  void HandleMainLinkPreLane(
      const iflymapdata::sdpro::Lane* pre_lane,
      std::vector<iflymapdata::sdpro::Lane>& next_lane_vec);
  void HandleOtherMergeLinkPreLane(
      const TopoLane& topo_lane,
      const iflymapdata::sdpro::LinkInfo_Link* next_pre_link,
      const iflymapdata::sdpro::LinkInfo_Link* current_link,
      const iflymapdata::sdpro::Lane* pre_lane,
      std::vector<iflymapdata::sdpro::Lane>& next_lane_vec);
  iflymapdata::sdpro::Lane FindMatchingPreLaneInMainLink(
      const TopoLane& topo_lane,
      const iflymapdata::sdpro::LinkInfo_Link* next_pre_link);
  size_t GetTargetRampIndex();
  bool HasValidMergeBeforeRamp(const double ramp_dis);
  bool IsDistanceToRampWithinThreshold(const double dis_to_ramp);
  bool IsMergePriorToRamp(const double dis_to_ramp);
  void UpdateSceneInfo(
      const iflymapdata::sdpro::LinkInfo_Link& front_first_link,
      const double dis_to_front_first_link,
      const iflymapdata::sdpro::LinkInfo_Link& target_link);

  void EraseFeasibleLaneIfNeeded(
      uint64_t lane_id, const iflymapdata::sdpro::LinkInfo_Link* split_next_link,
      TopoLinkGraph& feasible_lane_graph);
  std::pair<FPPoint, FPPoint> CalculateSplitExchangeAreaFP(const iflymapdata::sdpro::LinkInfo_Link* split_link, const SplitDirection& split_dir);
  std::pair<FPPoint, FPPoint> CalculateMergeExchangeAreaFP(const iflymapdata::sdpro::LinkInfo_Link* merge_link, const SplitDirection& merge_dir);
  std::tuple<size_t, size_t> CountAccAndEntryLanes(const iflymapdata::sdpro::LinkInfo_Link* link) const;
  void CalculateFrontMergePointInfo(double search_dis);
  double CalculateDisToLastLinkSplitPoint(const iflymapdata::sdpro::LinkInfo_Link* cur_link) const;
  double CalculateDisToLastLinkMergePoint(const iflymapdata::sdpro::LinkInfo_Link* cur_link) const;
  bool IsSucMergeLink(const iflymapdata::sdpro::LinkInfo_Link* link_info) const;
  bool CalculateSplitLinkExitLane(
      const iflymapdata::sdpro::LinkInfo_Link* split_link,
      const iflymapdata::sdpro::LinkInfo_Link* out_link,
      std::vector<iflymapdata::sdpro::Lane>& exit_lane_vec) const;
  bool IsNeedFilterSplit(const iflymapdata::sdpro::Lane* lane) const;
  void IsPreLaneOnOtherLink(TopoLinkGraph& before_split_feasible_lane_graph,
                            std::unordered_map<int, int>& order_diff);
  bool IsTheLaneOnSide(const iflymapdata::sdpro::Lane* lane,
                       bool& is_leftest, bool& is_rightest) const;
  bool CalculateDistanceToCertainLink(
      const iflymapdata::sdpro::LinkInfo_Link* target_link,
      const iflymapdata::sdpro::LinkInfo_Link* current_link,
      double& link_distance);
  void CalculateFeasibleLaneByMergePoint(TopoLinkGraph& feasible_lane_graph);
  bool CalculateLinkLaneNum(const iflymapdata::sdpro::LinkInfo_Link* link,
                            int& lane_num);
  double CalculateLSLLengthBetweenLanes(
      const iflymapdata::sdpro::LinkInfo_Link* link, uint32_t from_seq,
      uint32_t to_seq);

  bool IsMissedNaviRoute() const;

  ad_common::sdpromap::SDProMap ld_map_;
  const LocalView* local_view_ = nullptr;
  bool ldmap_valid_{false};
  uint64_t ld_map_info_updated_timestamp_ = 0;
  //  MLCDeciderRouteInfo mlc_decider_route_info_;
  const iflymapdata::sdpro::LinkInfo_Link* current_link_ = nullptr;
  double ego_on_cur_link_s_ = 0;
  double ego_on_cur_link_l_ = 0;
  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
      merge_info_vec_;
  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
      split_info_vec_;
  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
      ramp_info_vec_;
  MLCDeciderSceneTypeInfo mlc_decider_scene_type_info_;
  std::vector<TopoLane> avoid_link_merge_lane_id_vec_;

};
}