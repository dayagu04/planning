#pragma once
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include "local_view.h"
#include "route_info_strategy.h"
// #include "hdmap/hdmap.h"
// #include "sdmap/sdmap.h"
// #include "sdpromap/sdpromap.h"
#include "lane_topo_graph.h"
#include "modules/context/route_info_strategy/centerline_link_analyzer.h"

namespace planning {

// Single link node in the virtual lane topology graph.
// Covers begin_dist..end_dist meters ahead of ego (negative = behind).
// Lanes in this link are split into:
//   - recommended_orders: upstream-computed optimal path (from
//   feasible_lane_graph)
//   - on_route_orders: lane successor leads to route (may include
//   non-recommended)
//   - off_route_orders: lane successor leaves the route
// Orders are 1-based from right, with DiversionLane excluded.
struct LaneTopoNode {
  uint64_t link_id = 0;
  double begin_dist = 0.0;  // relative to ego longitudinal position
  double end_dist = 0.0;
  int lane_num = 0;  // effective lanes (no DiversionLane)
  std::vector<int>
      recommended_orders;  // upstream recommended path (FeasibleOrder)
  std::vector<int> on_route_orders;   // orders whose successor stays on route
  std::vector<int> off_route_orders;  // orders whose successor leaves route

  std::vector<int>
      successor_indices;  // indices into VirtualLaneTopoGraph::nodes
  std::vector<int> predecessor_indices;
};

// Complete lane topology graph covering -50m to +150m around ego.
struct VirtualLaneTopoGraph {
  std::vector<LaneTopoNode> nodes;
  int ego_node_index = -1;  // index of the node containing ego's current link

  // Return all node indices that cover the given longitudinal distance.
  // Multiple nodes may cover the same distance at a split/merge point.
  std::vector<size_t> GetNodeIndicesAtDistance(double dist) const {
    std::vector<size_t> result;
    for (size_t i = 0; i < nodes.size(); ++i) {
      const auto& node = nodes[i];
      if (dist >= node.begin_dist && dist < node.end_dist) {
        result.push_back(i);
      }
    }
    return result;
  }

  // Return ego node index, or -1 if invalid
  int GetEgoNodeIndex() const { return ego_node_index; }

  const std::shared_ptr<LaneTopoNode> GetEgoNode() const {
    if (ego_node_index >= 0 &&
        ego_node_index < static_cast<int>(nodes.size())) {
      return std::make_shared<LaneTopoNode>(nodes[ego_node_index]);
    }
    return nullptr;
  }
};

// Topological path representing a possible route from ego position forward.
struct TopoPath {
  // Nodes along this path (shared_ptr into VirtualLaneTopoGraph::nodes)
  std::vector<std::shared_ptr<LaneTopoNode>> nodes_ptr;

  // At each transition between nodes[i] and nodes[i+1], which lane orders
  // in nodes[i] connect to nodes[i+1]?
  // Size = nodes_ptr.size() - 1.
  // Example: Link1 has 4 lanes, lanes 1-2 go to Link2, lanes 3-4 go to Link3.
  //   Path(Link1→Link2): exit_orders[0] = {1, 2}
  //   Path(Link1→Link3): exit_orders[0] = {3, 4}
  // This tells us exactly which orders in the parent link feed into this path.
  std::vector<std::vector<int>> exit_orders;

  const std::shared_ptr<LaneTopoNode> GetNodeAtDistance(double dist) const {
    for (std::shared_ptr<LaneTopoNode> const node : nodes_ptr) {
      if (dist >= node->begin_dist && dist < node->end_dist) {
        return node;
      }
    }
    return nullptr;
  }
};

class LDRouteInfoStrategy : public RouteInfoStrategy {
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

  // 包装函数：判断传入lane是否在route
  // link上，结果写入lane的route_on_link_status属性
  void UpdateLaneIsOnRouteLinkStatus(
      const std::shared_ptr<VirtualLane>& lane,
      const iflymapdata::sdpro::LinkInfo_Link* split_link) const;

  // 对已判断为ON_ROUTE的lane，计算其在split_next_link上从左向右的序号
  // 若多条lane序号冲突，利用车道线点横向位置结合relative_id消歧
  void UpdateLanesOrderOnSplitNextLink(
      const std::shared_ptr<VirtualLane>& cur_lane,
      const std::shared_ptr<VirtualLane>& left_lane,
      const std::shared_ptr<VirtualLane>& right_lane,
      const iflymapdata::sdpro::LinkInfo_Link* split_link) const;

  // todo(ldh): 其他virutal函数。
 protected:
  bool UpdateLDMap();

  bool CalculateRouteInfo();

  /**
   * @brief Update feasible lane graph from MLCSceneTypeDecider results.
   * @return false if feasible lane graph computation fails.
   */
  bool UpdateFeasibleLaneGraph();

  /**
   * @brief Calculate route cost for each VirtualLane.
   * @param relative_id_lanes VirtualLanes sorted by order_id.
   * @return Per-lane cost vector (lower cost = more likely on route).
   */
  VirtualLanesRouteCost GetVirtualLaneCostOnRoute(
      std::vector<std::shared_ptr<VirtualLane>> const& relative_id_lanes)
      override;

  bool CalculateCurrentLink();

  bool IsInExpressWay();

  struct EgoPositionResult {
    bool is_between_links;     // 横向在两条link之间
    bool is_left_of_link1;     // 在link1左侧
    bool is_right_of_link1;    // 在link1右侧
    bool is_left_of_link2;     // 在link2左侧
    bool is_right_of_link2;    // 在link2右侧
    double dist_to_link1;      // 到link1的距离（线段距离）
    double dist_to_link2;      // 到link2的距离（线段距离）
    double min_dist_to_link1;  // 到link1所有形点的最小距离
    double min_dist_to_link2;  // 到link2所有形点的最小距离
    int min_dist_idx_link1;    // link1上最近形点的索引
    int min_dist_idx_link2;    // link2上最近形点的索引
    double proj_ratio_link1;   // 自车在link1上的投影比例
    double proj_ratio_link2;   // 自车在link2上的投影比例
  };

  bool CheckEgoPositionRelativeTwoLinks(
      const iflymapdata::sdpro::LinkInfo_Link* link1,
      const iflymapdata::sdpro::LinkInfo_Link* link2,
      EgoPositionResult& result) const;
  // 检查中心线与两条link的位置关系
  bool CheckCenterlineRelativeTwoLinks(
      const std::shared_ptr<VirtualLane>& current_lane,
      const iflymapdata::sdpro::LinkInfo_Link* link1,
      const iflymapdata::sdpro::LinkInfo_Link* link2,
      context::CenterlineCheckResult& result) const;

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

  /**
   * @brief Compute per-VirtualLane route cost using topo graph matching.
   *
   * Builds VirtualLaneTopoGraph, extracts topological paths, and scores
   * each VirtualLane against every path to determine on/off-route probability.
   * @param relative_id_lanes VirtualLanes sorted by order_id.
   * @param feasible_lane_graph Upstream feasible lane graph.
   * @return Per-lane cost vector (lower cost = more likely on route).
   */
  std::vector<VirtualLaneRouteCost> CalculateVirtualLaneRouteCost(
      const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
      const TopoLinkGraph& feasible_lane_graph);

  /**
   * @brief Build lane topology graph covering [-50m, +150m] around ego.
   *
   * Forward BFS through successor links and backward BFS through predecessors.
   * Each node classifies lanes as on_route/off_route and stores recommended
   * orders.
   * @param feasible_lane_graph Source of route link IDs and feasible orders.
   * @param[out] topo_graph Output topology graph.
   * @return false if ego is off-route (wrong_route scenario).
   */
  bool BuildVirtualLaneTopoGraph(const TopoLinkGraph& feasible_lane_graph,
                                 VirtualLaneTopoGraph& topo_graph);

  /**
   * @brief Create a LaneTopoNode for a single link.
   *
   * Classifies each lane as on_route or off_route based on successor link
   * membership. DiversionLanes are excluded; remaining lane orders are adjusted
   * accordingly.
   * @param link Map link pointer.
   * @param begin_dist Longitudinal start distance relative to ego.
   * @param end_dist Longitudinal end distance relative to ego.
   * @param route_link_ids Set of link IDs on the navigation route.
   * @param feasible_orders_by_link Recommended orders per link from upstream.
   * @return Populated LaneTopoNode.
   */
  LaneTopoNode CreateNodeFromLink(
      const iflymapdata::sdpro::LinkInfo_Link* link, double begin_dist,
      double end_dist, const std::unordered_set<uint64_t>& route_link_ids,
      const std::unordered_map<uint64_t, std::vector<int>>&
          feasible_orders_by_link);

  /**
   * @brief Extract all possible topological paths via DFS from ego node.
   *
   * For each node transition, computes exit_orders (which lane orders in the
   * parent node connect to the child node via lane-level successor).
   * @param topo_graph Input topology graph.
   * @param max_distance Maximum forward distance to traverse (meters).
   * @return Vector of TopoPath, one per reachable route branch.
   */
  std::vector<TopoPath> ExtractTopoPaths(const VirtualLaneTopoGraph& topo_graph,
                                         double max_distance = 100.0);

  /**
   * @brief Score how well a VirtualLane matches a topological path.
   *
   * Combines three signals: lane-count matching, fork boundary detection
   * (left/right drop to 0), and exit-order matching at split points.
   * @param virtual_lane Perception VirtualLane to evaluate.
   * @param topo_path Candidate topological path.
   * @param sample_step Longitudinal sampling interval (meters).
   * @param max_distance Maximum forward distance to sample (meters).
   * @return Weighted average match score in roughly [-1, 1].
   */
  double CalculateMatchScore(const std::shared_ptr<VirtualLane>& virtual_lane,
                             const TopoPath& topo_path,
                             double sample_step = 5.0,
                             double max_distance = 100.0);

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

  bool CalculateFeasibleLaneInRampScene(
      TopoLinkGraph& feasible_lane_graph,
      TopoLinkGraph& feasible_lane_graph_after_topo_change_vec);
  bool CalculateFeasibleLaneInMergeScene(
      TopoLinkGraph& feasible_lane_graph,
      TopoLinkGraph& feasible_lane_graph_after_topo_change_vec);
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
  void CaculateDistanceToNOAEnd(
      const iflymapdata::sdpro::LinkInfo_Link* segment, const double nearest_s);
  bool IsLaneSuccessorInPlannedRoute(const iflymapdata::sdpro::Lane* lane_info);
  bool IsLaneSuccessorIsMergeLane(
      const iflymapdata::sdpro::Lane* lane_info) const;
  const iflymapdata::sdpro::LinkInfo_Link* FindFrontValidRampSplitLink() const;
  void CalculateAvoidMergeFeasibleLane(TopoLinkGraph& feasible_lane_graph);
  void Erase1Split2FeasibleLane(TopoLinkGraph& feasible_lane_graph);
  std::vector<TopoLane> CalculateMaxDistanceLanes(
      const TopoLinkGraph& feasible_lane_graph) const;
  const iflymapdata::sdpro::Lane* IsEntryLanePresentOnEitherSideOfSuccessorLane(
      const iflymapdata::sdpro::Lane* cur_link_lane_info);
  // pair<left_lane, right_lane>
  std::pair<const iflymapdata::sdpro::Lane*, const iflymapdata::sdpro::Lane*>
  FindLaneLeftRightNeighbors(const iflymapdata::sdpro::Lane* target_lane);
  bool IsInvalidLaneMergeLaneOppositeSide(
      const iflymapdata::sdpro::Lane* merge_lane);
  bool IsInvalidNonDrivingLane(const iflymapdata::sdpro::Lane* lane);
  MergeLaneType CalculateMergeLaneType(
      const iflymapdata::sdpro::Lane* merge_lane);
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
      uint64_t lane_id,
      const iflymapdata::sdpro::LinkInfo_Link* split_next_link,
      TopoLinkGraph& feasible_lane_graph);
  std::pair<FPPoint, FPPoint> CalculateSplitExchangeAreaFP(
      const iflymapdata::sdpro::LinkInfo_Link* split_link,
      const SplitDirection& split_dir);
  std::pair<FPPoint, FPPoint> CalculateMergeExchangeAreaFP(
      const iflymapdata::sdpro::LinkInfo_Link* merge_link,
      const SplitDirection& merge_dir);
  std::tuple<size_t, size_t> CountAccAndEntryLanes(
      const iflymapdata::sdpro::LinkInfo_Link* link) const;
  void CalculateFrontMergePointInfo(double search_dis);
  double CalculateDisToLastLinkSplitPoint(
      const iflymapdata::sdpro::LinkInfo_Link* cur_link) const;
  double CalculateDisToLastLinkMergePoint(
      const iflymapdata::sdpro::LinkInfo_Link* cur_link) const;
  bool IsSucMergeLink(const iflymapdata::sdpro::LinkInfo_Link* link_info) const;
  bool CalculateSplitLinkExitLane(
      const iflymapdata::sdpro::LinkInfo_Link* split_link,
      const iflymapdata::sdpro::LinkInfo_Link* out_link,
      std::vector<iflymapdata::sdpro::Lane>& exit_lane_vec) const;
  bool IsNeedFilterSplit(const iflymapdata::sdpro::Lane* lane) const;
  void IsPreLaneOnOtherLink(TopoLinkGraph& before_split_feasible_lane_graph,
                            std::unordered_map<int, int>& order_diff);
  bool IsTheLaneOnSide(const iflymapdata::sdpro::Lane* lane, bool& is_leftest,
                       bool& is_rightest) const;
  bool CalculateDistanceToCertainLink(
      const iflymapdata::sdpro::LinkInfo_Link* target_link,
      const iflymapdata::sdpro::LinkInfo_Link* current_link,
      double& link_distance);
  void CalculateFeasibleLaneByMergePoint(TopoLinkGraph& feasible_lane_graph);
  void ProcessEraseFeasibleLaneForSplitScene(
      TopoLinkGraph& feasible_lane_graph);
  bool CalculateLinkLaneNum(const iflymapdata::sdpro::LinkInfo_Link* link,
                            int& lane_num);
  double CalculateLSLLengthBetweenLanes(
      const iflymapdata::sdpro::LinkInfo_Link* link, uint32_t from_seq,
      uint32_t to_seq);

  bool IsMissedNaviRoute() const;

  bool IsCurrentLaneOnRouteLink(const TopoLinkGraph& feasible_lane_graph) const;

  // 收集 link 上有效（非 emergency/diversion）车道，按 sequence 从大到小排序
  // 返回 vector<pair<sequence, lane_id>>，下标+1即为从左向右序号
  std::vector<std::pair<int, uint64_t>> BuildSeqLaneIds(
      const iflymapdata::sdpro::LinkInfo_Link* link) const;

  // 在已排序的 seq_lane_ids 中查找 sequence
  // 对应的从左向右序号（1=最左），未找到返回 -1
  static int SeqToOrder(
      const std::vector<std::pair<int, uint64_t>>& seq_lane_ids, int sequence);
  // 根据link的形点判断，两个后继link中哪个与输入link的横向距离更小
  // 返回横向距离更小的后继link，如果输入无效则返回nullptr
  const iflymapdata::sdpro::LinkInfo_Link*
  GetCloserSuccessorLinkByLateralDistance(
      const iflymapdata::sdpro::LinkInfo_Link* input_link,
      const iflymapdata::sdpro::LinkInfo_Link* successor_link1,
      const iflymapdata::sdpro::LinkInfo_Link* successor_link2) const;

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

  TopoLinkGraph feasible_lane_graph_;

  // 中心线与Link关系分析器
  context::CenterlineLinkAnalyzer centerline_link_analyzer_;
};
}  // namespace planning
