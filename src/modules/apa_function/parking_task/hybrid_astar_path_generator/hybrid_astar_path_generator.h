#pragma once
#include <string>
#include <vector>

#include "aabb2d.h"
#include "common_math.h"
#include "compact_node_pool.h"
#include "curve_node.h"
#include "geometry_math.h"
#include "grid_search.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_context.h"
#include "link_pt_line.h"
#include "node3d.h"
#include "node_delete_decider/node_delete_decider.h"
#include "obstacle_clear_zone_decider/obstacle_clear_zone_decider.h"
#include "parking_task.h"
#include "pose2d.h"
#include "reeds_shepp_interface.h"
#include "rs_path_interpolate.h"
#include "rs_path_request.h"

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
      override final;

  virtual const bool Init() override final;

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

  struct SearchLoopStats {
    size_t curve_path_success_num = 0;
    size_t explored_curve_path_num = 0;
    size_t explored_node_num = 0;
    size_t gen_child_node_num = 0;
    size_t gen_child_node_num_success = 0;
    size_t memory_usage = 0;
    size_t max_memory_usage = static_cast<size_t>(46.8 * 1024 * 1024);
    size_t curve_node_memory_usage = 16000;
  };

  struct SearchBudget {
    size_t find_success_curve_min_count = 0;
    double find_success_curve_max_time = 5000.0;
  };

  struct DebugTimeProfile {
    double collision_check_time_ms = 0.0;
    double rs_interpolate_time_ms = 0.0;
    double rs_time_ms = 0.0;
    int rs_try_num = 0;
    double lpl_interpolate_time_ms = 0.0;
    double lpl_time_ms = 0.0;
    int lpl_try_num = 0;
    double set_curve_path_time_ms = 0.0;
    double check_node_should_del_time_ms = 0.0;
    double heuristic_time_ms = 0.0;
    double generate_next_node_time_ms = 0.0;
  };

  struct CurveNodeScoreParam {
    float gear_change_penalty;
    float length_penalty;
    float unsuitable_last_line_length_penalty;
    float kappa_change_penalty;
    float last_path_kappa_change_penalty;
    float lat_err_penalty;
    float heading_err_penalty;
  };

  enum class PreSearchPhaseOutcome {
    CONTINUE_FORMAL_SEARCH,
    RETURN_SUCCESS,
    RETURN_FAILURE,
  };

  const bool UpdateOnce(const PathColDetBuffer& path_col_det_buffer);

  const bool FindOrAllocateSearchNode(size_t new_node_global_id,
                                      AstarNodeVisitedType& vis_type,
                                      Node3d*& next_node_in_pool) const;

  void InitNodePool();
  void ResetNodePool();
  void GenerateCarMotionVec();
  void ResetSearchState();
  const geometry_lib::PathPoint& GetStartPoseForCurrentSearch() const;
  const geometry_lib::PathPoint& GetEndPoseForCurrentSearch() const;
  const bool InitStartAndEndNodes();
  void GenerateDpMapForCurrentSearch();
  void SeedStartNodeIntoSearch();
  const bool PrepareCurrentNodeForExpansion(Node3d*& current_node);
  const bool ShouldStopCurveSearch(double search_start_time) const;
  const bool ValidateStartAndEndNodes();
  const bool CheckOutOfGridBound(const NodeGridIndex& id) const;
  const bool CheckOutOfPoseBound(const Pose2D& pose) const;
  void InitSearchPhaseTimeCost();
  void SyncSearchPhaseTimeCostToResult();
  const bool FinalizeUpdate(const bool search_success);
  void LogUpdateSummary() const;
  PathColDetBuffer BuildFormalSearchPathColDetBuffer() const;
  const std::string GetScenarioPrefix() const;
  const bool AnalyticExpansion(const AnalyticExpansionRequest& request);
  const bool CheckNodeShouldDelete(const NodeDeleteRequest& request);
  void DebugCurvePath(const CurvePath& path);
  void GenDecelerPointss();
  void UpdateInterestingAreaCache();

  const CarMotion GenerateCarMotion(const float radius,
                                    const AstarPathGear gear);

  void ActivateSearchNodeInOpenSet(const Node3d& new_node,
                                   Node3d* next_node_in_pool);

  void TryRelaxAnalyticExpansionConstraintsForDifficultScenario(
      size_t success_curve_count, size_t node_pool_size_threshold,
      AnalyticExpansionRequest& analytic_expansion_request);

  const bool HandleSuccessfulCurvePath(
      const AnalyticExpansionRequest& analytic_expansion_request,
      CurveNode& curve_node_to_goal,
      std::vector<CurveNode>& curve_node_to_goal_vec);

  void LogUpdateOnceSummary(
      double search_start_time,
      const std::vector<CurveNode>& curve_node_to_goal_vec) const;

  void SetLplInputPoseFromNode(
      const Node3d* node,
      link_pt_line::LinkPtLineInput<float>* lpl_input) const;

  void ConfigNodeDeleteInput(const PathColDetBuffer& path_col_det_buffer,
                             NodeDeleteInput* node_delete_input) const;

  void ConfigureNodeDeleteRequestForChildNode(
      Node3d& new_node, Node3d* current_node, const CarMotion& car_motion,
      AstarPathGear default_gear_request,
      NodeDeleteRequest& node_del_request) const;

  AstarPathGear DetermineChildNodeGearRequest(
      const Node3d& current_node, AstarPathGear default_gear_request) const;

  void PopulateChildNodeState(Node3d* new_node, Node3d* current_node,
                              const CarMotion& car_motion) const;

  void GenerateNextNode(Node3d* new_node, Node3d* parent_node,
                        const CarMotion& car_motion);

  const NodePath GetNodePathByCarMotion(const Pose2f& pose,
                                        const CarMotion& car_motion);

  const bool AnalyticExpansionByRS(Node3d* current_node,
                                   CurveNode* curve_node_to_goal,
                                   const RSInput* input);

  const bool CalcRSPathToGoal(Node3d* current_node, const RSInput* input,
                              const bool cal_h_cost = false);

  const bool AnalyticExpansionByLPL(
      Node3d* current_node, CurveNode* curve_node_to_goal,
      const link_pt_line::LinkPtLineInput<float>* input);

  const bool CalcLPLPathToGoal(
      Node3d* current_node, const link_pt_line::LinkPtLineInput<float>* input,
      const bool cal_h_cost = false);

  const float GenerateHeuristicCost(
      Node3d* next_node,
      const AnalyticExpansionRequest& analytic_expansion_request);

  const float CalcCurveNodeGCostToParentNode(Node3d* current_node,
                                             CurveNode* curve_node);

  const bool BackwardPassByCurveNode(const CurveNode* curve_node_to_goal);

  void CalcObsDistRelativeSlot(
      const CurveNode& curve_node,
      ObsToPathDistRelativeSlot& obs_dist_relative_slot);

  bool UpdateObsDistRelativeSlot(
      CurveNode* curve_node, ObsToPathDistRelativeSlot* obs_dist_relative_slot);

  void DebugNodePath(const NodePath& path,
                     const AstarPathGear gear = AstarPathGear::NONE);

  void ChooseBestCurveNode(std::vector<CurveNode>& curve_node_to_goal_vec,
                           CurveNode& best_curve_node_to_goal);

  virtual void UpdatePoseBoundary() = 0;
  virtual void ConfigureSearchBudget() = 0;

  virtual void ConfigureBaseAnalyticExpansionRequest(
      AnalyticExpansionRequest& analytic_expansion_request,
      link_pt_line::LinkPtLineInput<float>* lpl_input, RSInput* rs_input) const;

  virtual void ConfigureAnalyticExpansionRequestForCurrentNode(
      Node3d* current_node, CurveNode* curve_node_to_goal,
      AnalyticExpansionRequest& analytic_expansion_request) const;

  virtual void ConfigureAnalyticExpansionRequestForNewNode(
      const Node3d& new_node,
      AnalyticExpansionRequest& analytic_expansion_request) const;

  virtual void UpdateCulDeSacLimitByNewNode(Node3d& new_node);

  virtual void CalcNodeGCost(Node3d* current_node, Node3d* next_node);

  virtual void CalcNodeHCost(
      Node3d* next_node,
      const AnalyticExpansionRequest& analytic_expansion_request);

  virtual CurveNodeScoreParam BuildCurveNodeScoreParam() const;

  virtual void FillCurveNodeBaseCost(const CurveNode& curve_node,
                                     const CurveNodeScoreParam& score_param,
                                     PathCompareCost& cost) const;

  virtual void FillCurveNodeObsDistCost(CurveNode& curve_node,
                                        const CurveNodeScoreParam& score_param,
                                        PathCompareCost& cost);

  virtual void FillCurveNodeGearSwitchCost(
      const CurveNode& curve_node, const CurveNodeScoreParam& score_param,
      PathCompareCost& cost);

  virtual const bool RunFormalSearch(const SearchConfigSnapshot& snapshot);

 protected:
  PlannerOpenSpaceConfig config_;

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
  bool has_interesting_area_ = false;
  float interesting_area_min_x_ = 0.0f;
  float interesting_area_max_x_ = 0.0f;
  float interesting_area_min_y_ = 0.0f;
  float interesting_area_max_y_ = 0.0f;
  CulDeSacInfo cul_de_sac_info_;
  SearchPhaseTimeCost search_phase_time_cost_;
  SearchLoopStats search_loop_stats_;
  SearchBudget search_budget_;
  DebugTimeProfile debug_time_profile_;

  NodeDeleteDecider node_delete_decider_;
  ObstacleClearZoneDecider obstacle_clear_zone_decider_;
};
}  // namespace apa_planner
}  // namespace planning