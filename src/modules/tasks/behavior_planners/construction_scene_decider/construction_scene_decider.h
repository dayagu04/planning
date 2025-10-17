
#pragma once

#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "lateral_obstacle.h"
#include "math/linear_interpolation.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "quintic_poly_path.h"
// #include "scenario_state_machine.h"
#include "common/define/geometry.h"
#include "spline_projection.h"
#include "task_basic_types.h"
#include "tasks/task.h"
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
  enum class Direction { DEFAULT, LEFT, RIGHT, LON, UNSURE };
  struct ConstructionAgentPoint {
    // 施工障碍物数据结构体
    double x, y;
    double car_x, car_y;
    double s, l;
    double left_dist, right_dist;
    int32_t id;
    int cluster;
    bool visited;

    // Default constructor
    ConstructionAgentPoint()
        : x(0.0),
          y(0.0),
          car_x(0.0),
          car_y(0.0),
          s(0.0),
          l(0.0),
          left_dist(0.0),
          right_dist(0.0),
          id(0),
          cluster(-1),
          visited(false) {}
    // Parameterized constructor
    ConstructionAgentPoint(int32_t id, double x, double y, double car_x,
                           double car_y, double s, double l, double left_dist,
                           double right_dist)
        : x(x),
          y(y),
          car_x(car_x),
          car_y(car_y),
          s(s),
          l(l),
          left_dist(left_dist),
          right_dist(right_dist),
          id(id),
          cluster(-1),
          visited(false) {}
  };
  using ConstructionAgentPoints = std::vector<ConstructionAgentPoint>;

  struct ConstructionAgentClusterArea {
    ConstructionAgentPoints points;
    Direction direction = Direction :: DEFAULT;
  };

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

  void ExpandCluster(
      ConstructionAgentPoints& construction_agent_points, int index,
      int c, double eps_s, double eps_l, int minPts);

  void DbScan(ConstructionAgentPoints& construction_agent_points,
              double eps_s, double eps_l, int minPts);

  double CalcClusterToBoundaryDist(
      const ConstructionAgentPoints& points, RequestType direction);

  void ConstructionAgentDir();

  bool ConstructionAgentDirection(RequestType& direction);

  bool CheckEgoLaneAvailable(bool is_left);

  bool CheckTargetLaneAvailable(bool is_left,
                                const std::shared_ptr<VirtualLane> lane);

  double ConstructionAgentSpearmanRankCorrelation(
      const ConstructionAgentPoints points);

  double ConstructionAgentComputeSlope(
      ConstructionAgentPoints points);

  std::vector<double> ConstructionAgentRankify(std::vector<double>& arr);

  bool ConstructionAgentMean(const ConstructionAgentPoints& points,
                             double& s_mean, double& l_mean);

  bool ConstructionAgentStddev(
      const ConstructionAgentPoints& points, double s_mean,
      double l_mean, double& s_stddev, double& l_stddev);

  bool ConstructionAgentStandardize(
      ConstructionAgentPoints& points);

  double QueryLaneWidth(
      const double s0,
      const std::vector<std::pair<double, double>>& lane_s_width);

  bool IsConstructionAgent(iflyauto::ObjectType type);

  void UpdateDriveArea();

  std::pair<bool, int> CalIntersectionRefAndCone(
      std::shared_ptr<planning_math::KDPath> lane_frenet_coord,
      const std::vector<Point2d>& ref_points,
      const std::vector<Point2d>& cone_points);

  void UpdateResult(
    const std::map<int, std::map<int, std::vector<int>>>& results);

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
      construction_agent_cluster_attribute_set_;

  std::map<int, ad_common::math::Polygon2d> out_cluster_;
  std::vector<int32_t> construction_agent_cluster_size_;
  ConstructionAgentPoints construction_agent_cluster_;
  std::vector<std::pair<double, double>> left_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>>
      right_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>> origin_lane_s_width_;
  int right_lane_nums_ = 0;
  int left_lane_nums_ = 0;
  bool is_construction_agent_cluster_success_ = false;
  // bool use_query_lane_width_ = false;
};

}  // namespace planning
