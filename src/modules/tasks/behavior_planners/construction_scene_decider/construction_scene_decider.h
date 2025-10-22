
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
  explicit ConstructionSceneDecider(
      const EgoPlanningConfigBuilder* config_builder,
      framework::Session* session);

  virtual ~ConstructionSceneDecider() = default;

  bool Execute() override;

  bool ExecuteTest(bool pipeline_test);

  bool InitInfo();
  void UnitTest();

 private:
  void UpdateConstructionAgentClusters();

  void SetLaneChangeRequestByConstructionAgent();

  void GetTargetLaneWidthByConstructionAgent(
      const std::vector<std::pair<double, double>> lane_s_width,
      const std::shared_ptr<VirtualLane> target_lane,
      const double construction_agent_s, const double construction_agent_l,
      bool is_left, double* dist);

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

  void ConstructionAgentDir();

  bool ConstructionAgentDirection(RequestType& direction);

  bool CheckEgoLaneAvailable(bool is_left);

  bool CheckTargetLaneAvailable(bool is_left,
                                const std::shared_ptr<VirtualLane> lane);

  double ConstructionAgentSpearmanRankCorrelation(
      const ConstructionAgentPoints points);

  double ConstructionAgentComputeSlope(ConstructionAgentPoints points);

  std::vector<double> ConstructionAgentRankify(std::vector<double>& arr);

  bool ConstructionAgentMean(const ConstructionAgentPoints& points,
                             double& s_mean, double& l_mean);

  bool ConstructionAgentStddev(const ConstructionAgentPoints& points,
                               double s_mean, double l_mean, double& s_stddev,
                               double& l_stddev);

  bool ConstructionAgentStandardize(ConstructionAgentPoints& points);

  double QueryLaneWidth(
      const double s0,
      const std::vector<std::pair<double, double>>& lane_s_width);

  bool IsConstructionAgent(iflyauto::ObjectType type);

  void IdentifyConstructionScene();

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
  PlanningInitPoint planning_init_point_;
  std::shared_ptr<ReferencePath> left_reference_path_ = nullptr;
  std::shared_ptr<ReferencePath> right_reference_path_ = nullptr;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;
  std::shared_ptr<LaneTracksManager> lane_tracks_manager_ = nullptr;
  bool is_construction_agent_lane_change_situation_ = false;
  int construction_agent_alc_trigger_counter_;
  RequestType construction_agent_lane_change_direction_ = NO_CHANGE;
  ConstructionAgentPoints construction_agent_points_;
  std::map<int, ConstructionAgentClusterArea>
      construction_agent_cluster_attribute_map_;

  std::map<int, ad_common::math::Polygon2d> out_cluster_;
  std::vector<int32_t> construction_agent_cluster_size_;
  ConstructionAgentPoints construction_agent_cluster_;
  std::vector<std::pair<double, double>> left_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>>
      right_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>> origin_lane_s_width_;
  int right_lane_nums_ = 0;
  int left_lane_nums_ = 0;
  // bool use_query_lane_width_ = false;
  bool is_construction_agent_cluster_success_ = false;
  std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr_;
  int origin_lane_virtual_id_;
  bool is_exist_construction_area_ = false;
  bool is_pass_construction_area_ = false;
};

}  // namespace planning
