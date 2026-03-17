#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "aabb2d.h"
#include "apa_slot_manager.h"
#include "common_math.h"
#include "compact_node_pool.h"
#include "curve_node.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "grid_search.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_context.h"
#include "hybrid_astar_request.h"
#include "link_pose_line.h"
#include "link_pt_line.h"
#include "node3d.h"
#include "node_delete_decider/node_delete_decider.h"
#include "obstacle_clear_zone_decider/obstacle_clear_zone_decider.h"
#include "parking_task.h"
#include "pose2d.h"
#include "reeds_shepp_interface.h"
#include "rs_expansion_decider.h"
#include "rs_path_interpolate.h"
#include "vecf32.h"

namespace planning {
namespace apa_planner {

extern const float kMaxTurnRadius;

using namespace pnc;

class HybridAStarPathGenerator : public ParkingTask {
 public:
  void SetRequest(const HybridAStarRequest& request) { request_ = request; }

  void UpdateConfig(const PlannerOpenSpaceConfig& config);

  virtual void SetCollisionDetectorIntefacePtr(
      const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr)
      override;

  virtual const bool Init() override;

  const HybridAStarResult& GetResult() { return result_; }

  const std::vector<DebugAstarSearchPoint>& GetChildNodeForDebug();
  const std::vector<Eigen::Vector2d>& GetQueuePathForDebug();
  const std::vector<Eigen::Vector2d>& GetDeleteQueuePathForDebug();
  const std::vector<std::vector<Eigen::Vector2d>>& GetSearchNodeListMessage();
  const std::vector<std::vector<Eigen::Vector2d>>& GetCurveNodeListMessage();
  const std::vector<CurvePath>& GetAllSuccessCurvePathForDebug();
  const std::vector<geometry_lib::PathPoint>
  GetAllSuccessCurvePathFirstGearSwitchPoseForDebug();
  const cdl::AABB& GetIntersetingAreaForDebug();

 protected:
  struct SearchPhaseTimeCost {
    double decide_cul_de_sac_consume_time_ms = 0.0;
    double pre_search_consume_time_ms = 0.0;
    double formal_search_consume_time_ms = 0.0;
  };

  struct SearchConfigSnapshot {
    AnalyticExpansionType analytic_expansion_type;
    bool swap_start_goal;
    float traj_kappa_change_penalty;
  };

  enum class PreSearchPhaseOutcome {
    CONTINUE_FORMAL_SEARCH,
    RETURN_SUCCESS,
    RETURN_FAILURE,
  };

  void InitNodePool();
  void ResetNodePool();
  void GenerateCarMotionVec();
  const CarMotion GenerateCarMotion(const float radius,
                                    const AstarPathGear gear);

  virtual void UpdatePoseBoundary() {}
  void ResetSearchState();
  const geometry_lib::PathPoint& GetStartPoseForCurrentSearch() const;
  const geometry_lib::PathPoint& GetEndPoseForCurrentSearch() const;
  const bool InitStartAndEndNodes();
  const NodeDeleteInput BuildNodeDeleteInput(
      const PathColDetBuffer& path_col_det_buffer) const;
  const bool ValidateStartAndEndNodes();
  void PopulateChildNodeState(Node3d* new_node, Node3d* current_node,
                              const CarMotion& car_motion) const;
  const bool CheckOutOfGridBound(const NodeGridIndex& id) const;
  const bool CheckOutOfPoseBound(const Pose2D& pose) const;
  void InitSearchPhaseTimeCost();
  void SyncSearchPhaseTimeCostToResult();
  const bool FinalizeUpdate(const bool search_success);
  void LogUpdateSummary() const;
  virtual const bool RunFormalSearch(const SearchConfigSnapshot& snapshot);
  PathColDetBuffer BuildFormalSearchPathColDetBuffer() const;
  virtual const std::string GetScenarioPrefix() const;
  virtual const bool UpdateOnce(
      const PathColDetBuffer& path_col_det_buffer) = 0;

  void GenerateNextNode(Node3d* new_node, Node3d* parent_node,
                        const CarMotion& car_motion);

  const NodePath GetNodePathByCarMotion(const Pose2f& pose,
                                        const CarMotion& car_motion);

  virtual void CalcNodeGCost(Node3d* current_node, Node3d* next_node);

  virtual void CalcNodeHCost(
      Node3d* current_node, Node3d* next_node,
      const AnalyticExpansionRequest& analytic_expansion_request);

  const bool AnalyticExpansion(const AnalyticExpansionRequest& request);

  const bool AnalyticExpansionByRS(
      Node3d* current_node, CurveNode* curve_node_to_goal,
      const float rs_radius, const bool need_rs_dense_point = false,
      const bool need_anchor_point = false,
      const RSPathRequestType rs_reuest =
          RSPathRequestType::GEAR_SWITCH_LESS_THAN_TWICE);

  const bool CalcRSPathToGoal(Node3d* current_node,
                              const bool need_rs_dense_point,
                              const bool need_anchor_point,
                              const RSPathRequestType rs_request,
                              const float rs_radius,
                              const bool cal_h_cost = false);

  const float GenerateHeuristicCostByRsPath(Node3d* next_node,
                                            NodeHeuristicCost* cost);

  const bool AnalyticExpansionByLPL(
      Node3d* current_node, CurveNode* curve_node_to_goal,
      const link_pt_line::LinkPtLineInput<float>& input);

  const bool CalcLPLPathToGoal(
      Node3d* current_node, const link_pt_line::LinkPtLineInput<float>& input,
      const bool cal_h_cost = false);

  const float GenerateHeuristicCostByLPLPath(
      Node3d* next_node, const link_pt_line::LinkPtLineInput<float>& input,
      NodeHeuristicCost* cost);

  const float CalcCurveNodeGCostToParentNode(Node3d* current_node,
                                             CurveNode* curve_node);

  const bool CheckNodeShouldDelete(const NodeDeleteRequest& request);

  virtual void ChooseBestCurveNode(
      const std::vector<CurveNode>& curve_node_to_goal_vec,
      const AnalyticExpansionType analytic_expansion_type,
      const bool consider_obs_dist, const PathColDetBuffer& safe_buffer,
      CurveNode& best_curve_node_to_goal);

  const bool BackwardPassByCurveNode(const CurveNode* curve_node_to_goal);

  void CalcObsDistRelativeSlot(
      const CurveNode& curve_node,
      ObsToPathDistRelativeSlot& obs_dist_relative_slot);

  virtual const float CalcGearChangePoseCost(
      const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
      const float gear_switch_penalty, const float length_penalty);

  void DebugNodePath(const NodePath& path,
                     const AstarPathGear gear = AstarPathGear::NONE);

  void DebugCurvePath(const CurvePath& path);

 protected:
  PlannerOpenSpaceConfig config_;

  RSExpansionDecider rs_expansion_decider_;
  RSPathInterface rs_path_interface_;
  RSPath rs_path_;

  link_pt_line::LinkPtLinePath<float> lpl_path_;
  link_pt_line::LinkPtLine<float> lpl_interface_;

  static CompactNodePool node_pool_;
  static GridSearch grid_search_;

  // key is cost, value is node ptr
  std::multimap<float, Node3d*> open_pq_;

  // open set + close set
  std::unordered_map<size_t, Node3d*> node_set_;

  HybridAStarRequest request_;

  MapBound search_map_boundary_;
  NodeGridIndex search_map_grid_boundary_;

  float min_radius_;
  float max_front_wheel_angle_;
  float max_kappa_;
  float max_kappa_change_;

  std::vector<CarMotion> car_motion_vec;

  HybridAStarResult result_;
  SearchPhaseTimeCost search_phase_time_cost_;

  // astar start, end
  Node3d* start_node_;
  Node3d* end_node_;

  std::vector<DebugAstarSearchPoint> child_node_debug_;
  std::vector<Eigen::Vector2d> queue_path_debug_;
  std::vector<Eigen::Vector2d> delete_queue_path_debug_;
  std::vector<std::vector<Eigen::Vector2d>> all_search_node_list_;
  std::vector<std::vector<Eigen::Vector2d>> all_curve_node_list_;
  std::vector<CurvePath> all_success_curve_path_debug_;
  std::vector<common_math::PathPt<float>>
      all_success_path_first_gear_switch_pose_debug_;

  cdl::AABB interesting_area_;
  CulDeSacInfo cul_de_sac_info_;

  NodeDeleteDecider node_delete_decider_;

  // for debug
  double collision_check_time_ms_;
  double rs_interpolate_time_ms_;
  double rs_time_ms_;
  int rs_try_num_;
  double lpl_interpolate_time_ms_;
  double lpl_time_ms_;
  int lpl_try_num_;
  double set_curve_path_time_ = 0.0;
  double check_node_should_del_time_ = 0.0;
  double heuristic_time_;
  double generate_next_node_time_ = 0.0;
};
}  // namespace apa_planner
}  // namespace planning