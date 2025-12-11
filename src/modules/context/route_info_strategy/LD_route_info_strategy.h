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

//todo(ldh): 其他virutal函数。
protected:
 bool UpdateLDMap();

 bool CalculateRouteInfo();

 bool CalculateCurrentLink();

 bool IsInExpressWay();

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

 bool CalculateFeasibleLaneInRampScene(TopoLinkGraph& feasible_lane_graph);
 bool CalculateFeasibleLaneInMergeScene(TopoLinkGraph& feasible_lane_graph);
 bool CalculateFeasibleLaneInNormalScene(TopoLinkGraph& feasible_lane_graph);
 void ProcessLaneDistance(
    const std::shared_ptr<VirtualLane>& relative_id_lane,
    const std::unordered_map<int, double>& feasible_lane_distance);

 void CaculateDistanceToRoadEnd(
     const iflymapdata::sdpro::LinkInfo_Link* segment, const double nearest_s);
 void CaculateDistanceToTollStation(
     const iflymapdata::sdpro::LinkInfo_Link* segment, const double nearest_s);
 bool IsLaneSuccessorInPlannedRoute(const iflymapdata::sdpro::Lane* lane_info);
 bool IsLaneSuccessorIsMergeLane(const iflymapdata::sdpro::Lane* lane_info);
 const iflymapdata::sdpro::LinkInfo_Link* FindFrontValidRampSplitLink() const;
 void CalculateAvoidMergeFeasibleLane(TopoLinkGraph& feasible_lane_graph);
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
  MLCDeciderSceneInfoBaseBaidu mlc_decider_info_base_baidu_;

};
}