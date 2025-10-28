
#pragma once

#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "lateral_obstacle.h"
#include "math/linear_interpolation.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "quintic_poly_path.h"
// #include "scenario_state_machine.h"
#include "modules/common/define/geometry.h"
#include "modules/tasks/behavior_planners/lane_change_decider/lane_change_requests/lane_change_lane_manager.h"
#include "modules/tasks/task.h"
#include "modules/tasks/task_interface/construction_scene_decider_output.h"
#include "spline_projection.h"
#include "task_basic_types.h"
#include "utils/hysteresis_decision.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"

namespace planning {

using namespace planning_math;

class ConstructionSceneDecider : public Task {
 public:
  ConstructionSceneDecider(const EgoPlanningConfigBuilder* config_builder,
                           framework::Session* session);

  virtual ~ConstructionSceneDecider() = default;

  bool Execute() override;

  bool ExecuteTest(bool pipeline_test);

  bool InitInfo();

  void UnitTest();

 private:
  void UpdateConstructionAgentClusters();

  void GetOriginLaneWidthByConstructionAgent(
      const std::shared_ptr<VirtualLane> ego_lane,
      const double construction_agent_s, const double construction_agent_l,
      bool is_left, double* dist);

  bool ConstructionAgentDistance(const ConstructionAgentPoint& a,
                                 const ConstructionAgentPoint& b, double eps_s,
                                 double eps_l);

  void ExpandCluster(ConstructionAgentPoints& construction_agent_points,
                     int index, int c, double eps_s, double eps_l, int minPts);

  void DbScan(ConstructionAgentPoints& construction_agent_points, double eps_s,
              double eps_l, int minPts);

  double CalcClusterToBoundaryDist(const ConstructionAgentPoints& points,
                                   RequestType direction);

  double QueryLaneWidth(
      const double s0,
      const std::vector<std::pair<double, double>>& lane_s_width);

  bool IsConstructionAgent(iflyauto::ObjectType type);

  void IdentifyConstructionScene();

  void IsExistConstructionArea();

  void JudgeConstructionIntrusionLevel();

  bool CheckLaneAvailable(
      const std::shared_ptr<VirtualLane> seach_lane,
      bool is_left, bool is_right);

  void UpdateDriveArea();

  std::pair<bool, int> CalIntersectionRefAndCone(
      std::shared_ptr<planning_math::KDPath> lane_frenet_coord,
      const std::vector<Point2d>& ref_points,
      const std::vector<Point2d>& cone_points);

  void UpdateResult(
      const std::map<int, std::map<int, std::vector<int>>>& results);

  void GenerateConstructionSceneOutput();

  void SaveLatDebugInfo();

 private:
  ConstructionSceneDeciderConfig config_;
  std::shared_ptr<planning_math::KDPath> base_frenet_coord_;
  std::shared_ptr<ReferencePath> left_reference_path_ = nullptr;
  std::shared_ptr<ReferencePath> right_reference_path_ = nullptr;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;
  std::shared_ptr<LaneTracksManager> lane_tracks_manager_ = nullptr;
  ConstructionAgentPoints construction_agent_points_;
  std::map<int, ConstructionAgentClusterArea>
      construction_agent_cluster_attribute_map_;
  std::map<int, ad_common::math::Polygon2d> out_cluster_;
  std::vector<int32_t> construction_agent_cluster_size_;
  ConstructionAgentPoints construction_agent_cluster_;
  std::vector<std::pair<double, double>> origin_lane_s_width_;
  bool is_construction_agent_cluster_success_ = false;
  std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr_;
  int origin_lane_virtual_id_;
  bool is_exist_construction_area_ = false;
  bool is_pass_construction_area_ = false;
};

}  // namespace planning
