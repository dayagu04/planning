
#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "./../../modules/common/config/vehicle_param.h"
#include "./../../modules/context/vehicle_config_context.h"
#include "./../convex_collision_detection/aabb2d.h"
#include "./../convex_collision_detection/gjk2d_interface.h"
#include "./../occupancy_grid_map/euler_distance_transform.h"
#include "./../occupancy_grid_map/point_cloud_obstacle.h"
#include "./../reeds_shepp/reeds_shepp_interface.h"
#include "ad_common/math/line_segment2d.h"
#include "compact_node_pool.h"
#include "cubic_polynomial_path.h"
#include "dynamic_programing_cost.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_config.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "node_shrink_decider.h"
#include "obstacle_clear_zone.h"
#include "park_reference_line.h"
#include "planning_debug_info.pb.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "rs_expansion_decider.h"
#include "rs_path_interpolate.h"

namespace planning {

class HybridAStar {
 public:
  HybridAStar() = default;

  explicit HybridAStar(const PlannerOpenSpaceConfig& open_space_conf,
                       const VehicleParam& veh_param);

  virtual ~HybridAStar() = default;

  void Init();

  void UpdateCarBoxBySafeBuffer(const double lat_buffer_outside,
                                const double lat_buffer_inside,
                                const double lon_buffer);

  int UpdateConfig(const PlannerOpenSpaceConfig& open_space_conf);

  void UpdateConfig(const AstarRequest& request);

  /**
   * start: astar start
   * end: astar end, maybe different from real goal in slot.
   */
  bool AstarSearch(const Pose2D& start, const Pose2D& end,
                   const MapBound& XYbounds, const ParkObstacleList& obstacles,
                   const AstarRequest& request,
                   const ObstacleClearZone* clear_zone,
                   HybridAStarResult* result, EulerDistanceTransform* edt,
                   ParkReferenceLine* ref_line);

  // use rs path sampling to link start point and end point.
  // 库内向前揉库使用.
  // todo: 向前揉库，向后揉库统一起来.
  bool PlanByRSPathSampling(
      HybridAStarResult* result, const Pose2D& start, const Pose2D& end,
      const double lon_min_sampling_length, const MapBound& XYbounds,
      const ParkObstacleList& obstacles, const AstarRequest& request,
      EulerDistanceTransform* edt, const ObstacleClearZone* clear_zone,
      ParkReferenceLine* ref_line);

  // use cubic path sampling to link start point and end point.
  bool SamplingByCubicPolyForVerticalSlot(
      HybridAStarResult* result, const Pose2D& start, const Pose2D& target,
      const double lon_min_sampling_length, const MapBound& XYbounds,
      const ParkObstacleList& obstacles, const AstarRequest& request,
      EulerDistanceTransform* edt, const ObstacleClearZone* clear_zone,
      ParkReferenceLine* ref_line);

  // use cubic spiral path sampling to link start point and end point.
  bool SamplingByCubicSpiralForVerticalSlot(
      HybridAStarResult* result, const Pose2D& start, const Pose2D& target,
      const double lon_min_sampling_length, const MapBound& XYbounds,
      const ParkObstacleList& obstacles, const AstarRequest& request,
      EulerDistanceTransform* edt, const ObstacleClearZone* clear_zone,
      ParkReferenceLine* ref_line);

  void GetRSPathForDebug(std::vector<double>& x, std::vector<double>& y,
                         std::vector<double>& phi);

  // for debug
  void DebugRSPath(const RSPath* reeds_shepp_path);

  // for debug
  const std::vector<DebugAstarSearchPoint>& GetChildNodeForDebug();

  // for debug
  const std::vector<ad_common::math::Vec2d>& GetQueuePathForDebug();
  const std::vector<ad_common::math::Vec2d>& GetDelQueuePathForDebug();
  // for debug
  const std::vector<RSPath>& GetRSPathHeuristic();

  // for debug
  void GetNodeListMessage(planning::common::AstarNodeList* list);

  // for debug
  void GetNodeListMessage(std::vector<std::vector<Eigen::Vector2d>>& list);

  const ParkReferenceLine* GetConstRefLine() const;

  void Clear();

  void CopyFallbackPath(HybridAStarResult* path);

  // search single gear path by gear reverse searching.
  // todo: gear drive searching.
  void GearRerversePathAttempt(
      const MapBound& XYbounds, const ParkObstacleList& obstacles,
      const AstarRequest& request, const ObstacleClearZone* clear_zone,
      const Pose2D& start, const Pose2D& target, HybridAStarResult* result,
      EulerDistanceTransform* edt, ParkReferenceLine* ref_line);

  // If search success, return a path linked wtih goal point;
  // If search fail, return a path nearby goal point;
  void GearDrivePathAttempt(
      const MapBound& XYbounds, const ParkObstacleList& obstacles,
      const AstarRequest& request, const ObstacleClearZone* clear_zone,
      const Pose2D& start, const Pose2D& target, HybridAStarResult* result,
      EulerDistanceTransform* edt, ParkReferenceLine* ref_line);

  // for debug
  void DebugPathString(const HybridAStarResult* result) const;

  // use cubic path sampling to link start point and end point.
  bool SamplingByCubicPolyForParallelSlot(
      HybridAStarResult* result, const Pose2D& start, const Pose2D& end,
      const double lon_min_sampling_length, const MapBound& XYbounds,
      const ParkObstacleList& obstacles, const AstarRequest& request,
      EulerDistanceTransform* edt, const ObstacleClearZone* clear_zone,
      ParkReferenceLine* ref_line);

  // debug
  FootPrintCircleModel* GetSlotOutsideCircleFootPrint();

 private:
  // todo: select dubins/rs path by request gear to accelerate computation.
  bool AnalyticExpansionByRS(Node3d* current_node,
                             const PathGearRequest gear_request_info,
                             Node3d* rs_node_to_goal);

  bool SamplingByQunticPolynomial(Node3d* current_node,
                                  std::vector<AStarPathPoint>& path,
                                  Node3d* polynomial_node,
                                  PolynomialPathErrorCode* fail_type);

  // 向后揉库使用
  bool SamplingByRSPath(const PathGearRequest gear_request,
                        Node3d* current_node, Node3d* polynomial_node);

  // check collision and validity
  bool ValidityCheckByConvex(Node3d* node);

  // check collision and validity
  const bool ValidityCheckByEDT(Node3d* node);

  // check Reeds Shepp path collision and validity
  bool RSPathCollisionCheck(const RSPath* reeds_shepp_to_end,
                            Node3d* rs_node_to_goal);

  void CalculateNodeFCost(Node3d* current_node, Node3d* next_node);

  void CalculateNodeGCost(Node3d* current_node, Node3d* next_node);

  void CalculateNodeHeuristicCost(Node3d* father_node, Node3d* next_node);

  void GetSingleShotNodeHeuCost(const Node3d* father_node, Node3d* next_node);

  double CalcGCostToParentNode(Node3d* current_node, Node3d* next_node);

  double CalcRSGCostToParentNode(Node3d* current_node, Node3d* rs_node,
                                 const RSPath* rs_path);

  void GetSingleShotNodeGCost(Node3d* current_node, Node3d* next_node);

  // holonomic: freedom is equal with controllable variables
  // e.g. car freedom is x,y,theta, but controllable variables are lon speed and
  // wheel. so car is nonholonomic.
  // for human, freedom is x,y,theta, controllable variables are x speed, y
  // speed, rotate wheel, so human is holonomic.
  double ObstacleHeuristicWithHolonomic(Node3d* next_node);

  double GenerateHeuristicCost(Node3d* next_node);

  double GenerateRefLineHeuristicCost(Node3d* next_node,
                                      const double dist_to_go);

  double GenerateHeuristicCostByRsPath(Node3d* next_node,
                                       NodeHeuristicCost* cost);

  const bool BackwardPassByRSPath(HybridAStarResult* result,
                                  Node3d* best_rs_node, const RSPath* rs_path);

  void ResetNodePool();

  void NodePoolInit();

  bool NodeInSearchBound(Node3d* node);

  bool NodeInSearchBound(const NodeGridIndex& id);

  // todo: refact
  const NodeShrinkType NextNodeGenerator(
      Node3d* new_node, Node3d* parent_node, size_t next_node_index,
      const PathGearRequest gear_request_info);

  bool IsAllPathSegmentLongEnough(const RSPath* reeds_shepp_to_end,
                                  const double father_node_dist);

  bool IsRsPathFirstSegmentLongEnough(const RSPath* reeds_shepp_to_end,
                                      const double father_node_dist);

  bool RsLastSegmentSatisfyRequest(const RSPath* reeds_shepp_to_end);

  bool CheckRSPathGear(const RSPath* reeds_shepp_to_end,
                       const PathGearRequest gear_request_info);

  bool IsRSPathSafeByConvexHull(const RSPath* reeds_shepp_path, Node3d* node);

  const bool IsRSPathSafeByEDT(const RSPath* reeds_shepp_path, Node3d* node);

  const bool IsPolynomialPathSafeByEDT(const std::vector<AStarPathPoint>& path,
                                       Node3d* node);

  void LinkRsToAstarEndPoint(HybridAStarResult* result,
                             const Pose2D& astar_end);

  void DebugEDTCheck(HybridAStarResult* path);

  Polygon2D* GetVehPolygon(const AstarPathGear& gear);

  // radius:left is positve
  void KineticsModel(const Pose2D* old_pose, const double radius, Pose2D* pose,
                     const bool is_forward);

  // radius:left is positve
  // arc: is always positive
  void GetPathByBicycleModel(NodePath* path, const double arc,
                             const double radius, const bool is_forward);

  void GetPathByLine(NodePath* path, const double arc, const bool is_forward);

  // if left, radius is positive
  void GetPathByCircle(NodePath* path, const double arc, const double radius,
                       const bool is_forward);

  // dist_to_start: if forward, dist_to_start is positive
  int GetStraightLinePoint(Pose2D* goal_state, const Pose2D* start_state,
                           const double dist_to_start,
                           const Pose2D* unit_vector);

  // radius: if left turn, radius is positive
  int GetVehCircleByPose(VehicleCircle* veh_circle, const Pose2D* pose,
                         const double radius, const AstarPathGear gear);

  // arc is positive.
  // inverse_radius is positive
  int InterpolateByArcOffset(Pose2D* pose, const VehicleCircle* veh_circle,
                             const Pose2D* start_pose, const double arc,
                             const double inverse_radius);

  void UpdatePoseByPathPointInterval(const Pose2D* old_pose,
                                     const double radius, const double interval,
                                     Pose2D* pose, const bool is_forward);

  void UpdatePoseBySamplingNumber(const Pose2D* old_pose, const double radius,
                                  const int number, Pose2D* pose,
                                  const bool is_forward);

  size_t GetPathCollisionIndex(HybridAStarResult* result);

  const bool IsPointBeyondBound(const double x, const double y) const;

  bool CalcRSPathToGoal(Node3d* current_node, const bool need_rs_dense_point,
                        const bool need_anchor_point,
                        const RSPathRequestType rs_request,
                        const double rs_radius);

  double CalcSafeDistCost(Node3d* node);

  // for debug
  void DebugObstacleString() const;

  // for debug
  void DebugLineSegment(const ad_common::math::LineSegment2d& line) const;

  void RSPathCandidateByRadius(HybridAStarResult* result, const Pose2D& start,
                               const Pose2D& end,
                               const double lon_min_sampling_length,
                               const double radius);

  size_t GetPathCollisionIDByEDT(HybridAStarResult* result);

  void GetQunticPolynomialPath(std::vector<AStarPathPoint>& path,
                               const Pose2D& start, const double start_kappa,
                               const Pose2D& end);

  const bool GetCubicSpiralPath(std::vector<AStarPathPoint>& path,
                                const Pose2D& start, const Pose2D& end,
                                const AstarPathGear ref_gear);

  const bool BackwardPassByPolynomialPath(
      HybridAStarResult* result, Node3d* poly_node,
      const std::vector<AStarPathPoint>& poly_path);

  void DebugPolynomialPath(const std::vector<AStarPathPoint>& poly_path);

  size_t GetPathCollisionIDByEDT(const std::vector<AStarPathPoint>& poly_path);

  void ReversePathBySwapStartGoal(HybridAStarResult* result);

  const bool IsPolygonCollision(const Polygon2D* polygon);

  const bool IsFootPrintCollision(const Transform2d& tf);

  FootPrintCircleModel* GetCircleFootPrintModel(const Pose2D& pose);

  const bool IsExpectedGearForRsPath(const RSPath &path);

  // copy path from rs path
  void PathTransformByRSPath(const RSPath& rs_path, HybridAStarResult* result);

  const bool BackwardPassByNode(HybridAStarResult* result, Node3d* end_node);

  const bool BestNodeIsNice(const Node3d *node);

 private:
  PlannerOpenSpaceConfig config_;
  VehicleParam vehicle_param_;
  double car_half_width_;
  double min_radius_;
  double inv_radius_;

  // todo, width = vehicle width + mirror width + safe width, bounding box
  Polygon2D veh_box_gear_none_;
  Polygon2D veh_box_gear_drive_;
  Polygon2D veh_box_gear_reverse_;
  // convex hull for accurate car
  PolygonFootPrint cvx_hull_foot_print_;

  size_t next_node_num_ = 0;
  // front wheel angle, [-pi, +pi]
  // left is positive
  AstarSamplingAngle next_node_angles_;

  // child node shrink related
  NodeShrinkDecider node_shrink_decider_;

  //  front wheel angle, not steering wheel angle
  double max_steer_angle_ = 0.0;

  double node_path_dist_resolution_ = 0.0;
  double kinetics_model_step_ = 0.05;
  double xy_grid_resolution_ = 0.0;
  double phi_grid_resolution_ = 0.0;

  // search bound
  // todo: map bound is large, unify map bound and search bound.
  // search bound is small. (0-40 meter)
  // PER_DIMENSION_MAX_NODE, data bound(bit:0-9) is more large.
  size_t max_x_search_size_;
  size_t max_y_search_size_;
  size_t max_theta_search_size_;

  // xmin, xmax, ymin, ymax
  MapBound XYbounds_;

  // astar start, end
  Node3d* start_node_;
  Node3d* astar_end_node_;

  const ParkObstacleList* obstacles_;
  // if search node in aabb, no need to check collision;
  const ObstacleClearZone* clear_zone_;

  EulerDistanceTransform* edt_;

  CompactNodePool node_pool_;

  // std::priority_queue<QueuePoint, std::vector<QueuePoint>, QueueCompare>
  //     open_pq_;
  std::multimap<double, Node3d*> open_pq_;

  // open set + close set
  std::unordered_map<size_t, Node3d*> node_set_;

  // rs related
  RSExpansionDecider rs_expansion_decider_;
  RSPathInterface rs_path_interface_;
  RSPath rs_path_;
  CubicPathInterface cubic_path_interface_;

  std::unique_ptr<GridSearch> dp_heuristic_generator_;

  ParkReferenceLine* ref_line_;

  AstarRequest request_;

  GJK2DInterface gjk_interface_;

  HybridAStarResult fallback_path_;

  // 用于区分库内库外
  cdl::AABB slot_box_;

  // 根据车辆位置，使用不同buffer
  HierarchyBufferCircleFootPrint hierachy_circle_model_;

  // just for debug, display all result in hmi/plot
  std::vector<DebugAstarSearchPoint> child_node_debug_;
  std::vector<ad_common::math::Vec2d> queue_path_debug_;
  std::vector<ad_common::math::Vec2d> delete_queue_path_debug_;
  std::vector<RSPath> rs_path_h_cost_debug_;

  // for debug
  double collision_check_time_ms_;
  double rs_interpolate_time_ms_;
  double rs_time_ms_;
  double heuristic_time_;
};

}  // namespace planning
