#pragma once

#include "apa_context.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_config.h"
#include "hybrid_astar_context.h"
#include "tail_in/hybrid_astar_perpendicular_tail_in_path_generator.h"
namespace planning {
namespace apa_planner {

class HybridAstarPathGeneratorInterface final {
 public:
  HybridAstarPathGeneratorInterface();
  HybridAstarPathGeneratorInterface(
      const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr);
  ~HybridAstarPathGeneratorInterface();

  void Init();

  void Reset();

  const bool Update();

  void SetRequest(const HybridAStarRequest& request);

  void SetColDetIntefacePtr(
      const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr);

  void UpdateConfig(const PlannerOpenSpaceConfig& config);

  const bool GetResult(HybridAStarResult& result);

  const bool GetChildNodeForDebug(
      std::vector<DebugAstarSearchPoint>& child_node_debug);

  const bool GetQueuePathForDebug(std::vector<Eigen::Vector2d>& queue_path);

  const bool GetDeleteQueuePathForDebug(
      std::vector<Eigen::Vector2d>& del_queue_path);

  const bool GetSearchNodeListMessage(
      std::vector<std::vector<Eigen::Vector2d>>& search_node_list);

  const bool GetCurveNodeListMessage(
      std::vector<std::vector<Eigen::Vector2d>>& curve_node_list);

  const bool GetAllSuccessCurvePathForDebug(
      std::vector<CurvePath>& all_success_curve_path_debug);

  const bool GetAllSuccessCurvePathFirstGearSwitchPoseForDebug(
      std::vector<geometry_lib::PathPoint>&
          all_success_curve_path_first_gear_switch_pose_debug);

  const bool GetPreSearchABBoxForDebug(cdl::AABB& pre_search_abbox);

 private:
  std::shared_ptr<HybridAStarPerpendicularTailInPathGenerator>
      hybrid_astar_perpendicular_tail_in_path_generator_ptr_;

  ParkingScenarioType scenario_type_ = ParkingScenarioType::SCENARIO_UNKNOWN;

  bool init_flag_ = false;
  bool has_constructed_flag_ = false;
};
}  // namespace apa_planner
}  // namespace planning