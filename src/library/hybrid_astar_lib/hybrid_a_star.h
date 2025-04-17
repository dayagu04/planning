
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
#include "cost/dynamic_programing_cost.h"
#include "curve/cubic_polynomial_path.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_config.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "node_collision_detect.h"
#include "node_shrink_decider.h"
#include "obstacle_clear_zone.h"
#include "park_reference_line.h"
#include "planning_debug_info.pb.h"
#include "polygon_base.h"
#include "polynomial_curve_sampling.h"
#include "pose2d.h"
#include "rs_expansion_decider.h"
#include "rs_path_interpolate.h"
#include "rs_sampling.h"
#include "spiral_sampling.h"
#include "vecf32.h"

namespace planning {

class HybridAStar {
 public:
  HybridAStar() = default;

  explicit HybridAStar(const PlannerOpenSpaceConfig& open_space_conf,
                       const VehicleParam& veh_param,
                       const ParkObstacleList* obstacles,
                       EulerDistanceTransform* edt,
                       const ObstacleClearZone* clear_zone,
                       ParkReferenceLine* ref_line,
                       GridSearch *dp_map);

  virtual ~HybridAStar() = default;

  void Init();

  void UpdateCarBoxBySafeBuffer(const float lat_buffer_outside,
                                const float lat_buffer_inside,
                                const float lon_buffer);

  void SetRequest(const AstarRequest& request);

  /**
   * start: astar start
   * end: astar end, maybe different from real goal in slot.
   */
  bool AstarSearch(const Pose2D& start, const Pose2D& end,
                   const MapBound& XYbounds, HybridAStarResult* result);

  void GetRSPathForDebug(std::vector<float>& x, std::vector<float>& y,
                         std::vector<float>& phi);

  // for debug
  const std::vector<DebugAstarSearchPoint>& GetChildNodeForDebug();

  // for debug
  const std::vector<Vec2df32>& GetQueuePathForDebug();
  const std::vector<Vec2df32>& GetDelQueuePathForDebug();
  // for debug
  const std::vector<RSPath>& GetRSPathHeuristic();

  // for debug
  void GetNodeListMessage(planning::common::AstarNodeList* list);

  // for debug
  void GetNodeListMessage(std::vector<std::vector<Eigen::Vector2f>>& list);

  const ParkReferenceLine* GetConstRefLine() const;

  void Clear();

  void CopyFallbackPath(HybridAStarResult* path);

  // If search success, return a path linked wtih goal point;
  // todo: unify this API with AstarSearch().
  void OneShotPathAttempt(const MapBound& XYbounds, const Pose2D& start,
                          const Pose2D& target, HybridAStarResult* result);

  // debug
  FootPrintCircleModel* GetSlotOutsideCircleFootPrint();

  // todo: move sampling based method out of astar class, we can move it to
  // interface.
  bool SamplingByCubicPolyForParallelSlot(HybridAStarResult* result,
                                          const Pose2D& start,
                                          const Pose2D& end,
                                          const float lon_min_sampling_length) {
    return polynomial_sampling_->SamplingByCubicPolyForParallelSlot(
        result, start, end, lon_min_sampling_length);
  }

  // use rs path sampling to link start point and end point.
  bool PlanByRSPathSampling(HybridAStarResult* result, const Pose2D& start,
                            const Pose2D& end,
                            const float lon_min_sampling_length) {
    return rs_sampling_->PlanByRSPathSampling(result, start, end,
                                              lon_min_sampling_length);
  }

  bool SamplingByCubicSpiralForVerticalSlot(
      HybridAStarResult* result, const Pose2D& start, const Pose2D& target,
      const float lon_min_sampling_length) {
    return spiral_sampling_->SamplingByCubicSpiralForVerticalSlot(
        result, start, target, lon_min_sampling_length);
  }

 private:
  // todo: select dubins/rs path by request gear to accelerate computation.
  bool AnalyticExpansionByRS(Node3d* current_node,
                             const AstarPathGear gear_request_info,
                             Node3d* rs_node_to_goal);

  void CalculateNodeFCost(Node3d* current_node, Node3d* next_node);

  void CalculateNodeGCost(Node3d* current_node, Node3d* next_node);

  void CalculateNodeHeuristicCost(Node3d* father_node, Node3d* next_node);

  void GetSingleShotNodeHeuCost(const Node3d* father_node, Node3d* next_node);

  float CalcGCostToParentNode(Node3d* current_node, Node3d* next_node);

  float CalcRSGCostToParentNode(Node3d* current_node, Node3d* rs_node,
                                const RSPath* rs_path);

  void GetSingleShotNodeGCost(Node3d* current_node, Node3d* next_node);

  // holonomic: freedom is equal with controllable variables
  // e.g. car freedom is x,y,theta, but controllable variables are lon speed and
  // wheel. so car is nonholonomic.
  // for human, freedom is x,y,theta, controllable variables are x speed, y
  // speed, rotate wheel, so human is holonomic.
  float ObstacleHeuristicWithHolonomic(Node3d* next_node);

  float GenerateHeuristicCost(Node3d* next_node);

  float GenerateRefLineHeuristicCost(Node3d* next_node, const float dist_to_go);

  float GenerateHeuristicCostByRsPath(Node3d* next_node,
                                      NodeHeuristicCost* cost);

  void ResetNodePool();

  void NodePoolInit();

  bool NodeInSearchBound(Node3d* node);

  bool NodeInSearchBound(const NodeGridIndex& id);

  // do searching in normal pipline, no gear request
  const NodeShrinkType NextNodeGenerator(Node3d* new_node, Node3d* parent_node,
                                         size_t next_node_index,
                                         const AstarPathGear gear_request_info);

  bool IsAllPathSegmentLongEnough(const RSPath* reeds_shepp_to_end,
                                  const float father_node_dist);

  bool IsRsPathFirstSegmentLongEnough(const RSPath* reeds_shepp_to_end,
                                      const float father_node_dist);

  bool RsLastSegmentSatisfyRequest(const RSPath* reeds_shepp_to_end);

  bool CheckRSPathGear(const RSPath* reeds_shepp_to_end,
                       const AstarPathGear gear_request_info);

  // radius:left is positve
  void KineticsModel(const Pose2D* old_pose, const float radius, Pose2D* pose,
                     const bool is_forward);

  // radius:left is positve
  // arc: is always positive
  void GetPathByBicycleModel(NodePath* path, const float arc,
                             const float radius, const bool is_forward);

  void GetPathByLine(NodePath* path, const float arc, const bool is_forward);

  // if left, radius is positive
  void GetPathByCircle(NodePath* path, const float arc, const float radius,
                       const bool is_forward);

  // dist_to_start: if forward, dist_to_start is positive
  int GetStraightLinePoint(Pose2D* goal_state, const Pose2D* start_state,
                           const float dist_to_start,
                           const Pose2D* unit_vector);

  // radius: if left turn, radius is positive
  int GetVehCircleByPose(VehicleCircle* veh_circle, const Pose2D* pose,
                         const float radius, const AstarPathGear gear);

  // arc is positive.
  // inverse_radius is positive
  int InterpolateByArcOffset(Pose2D* pose, const VehicleCircle* veh_circle,
                             const Pose2D* start_pose, const float arc,
                             const float inverse_radius);

  void UpdatePoseByPathPointInterval(const Pose2D* old_pose, const float radius,
                                     const float interval, Pose2D* pose,
                                     const bool is_forward);

  void UpdatePoseBySamplingNumber(const Pose2D* old_pose, const float radius,
                                  const int number, Pose2D* pose,
                                  const bool is_forward);

  bool CalcRSPathToGoal(Node3d* current_node, const bool need_rs_dense_point,
                        const bool need_anchor_point,
                        const RSPathRequestType rs_request,
                        const float rs_radius);

  float CalcSafeDistCost(Node3d* node);

  // for debug
  void DebugLineSegment(const ad_common::math::LineSegment2d& line) const;

  void ReversePathBySwapStartGoal(HybridAStarResult* result);

  // todo: unify different path
  const bool BackwardPassByNode(HybridAStarResult* result, Node3d* best_node,
                                const RSPath* rs_path,
                                const std::vector<AStarPathPoint>& poly_path);

  const bool BestNodeIsNice(const Node3d* node);

  void CopyNodePath(const Node3d* node, HybridAStarResult* astar_path);

  void UpdatePathS(HybridAStarResult* path);

  // If expect gear is drive, check ego pose.
  const bool IsNeedGearDriveSearch(const Pose2D& start);

  // If expect gear is reverse, check ego pose.
  const bool IsNeedGearReverseSearch(const Pose2D& start);

  void DebugNodeList(const std::vector<Node3d*>& node_list);

 private:
  PlannerOpenSpaceConfig config_;
  VehicleParam vehicle_param_;
  float car_half_width_;
  float min_radius_;
  float inv_radius_;

  size_t next_node_num_ = 0;
  // front wheel angle, [-pi, +pi]
  // left is positive
  AstarSamplingAngle next_node_angles_;

  // child node shrink related
  NodeShrinkDecider node_shrink_decider_;

  //  front wheel angle, not steering wheel angle
  float max_steer_angle_ = 0.0;

  float node_path_dist_resolution_ = 0.0;
  float kinetics_model_step_ = 0.05;
  float xy_grid_resolution_ = 0.0;
  float phi_grid_resolution_ = 0.0;

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
  std::multimap<float, Node3d*> open_pq_;

  // open set + close set
  std::unordered_map<size_t, Node3d*> node_set_;

  // rs related
  RSExpansionDecider rs_expansion_decider_;
  RSPathInterface rs_path_interface_;
  RSPath rs_path_;

  GridSearch* dp_heuristic_generator_;

  ParkReferenceLine* ref_line_;

  AstarRequest request_;

  std::shared_ptr<NodeCollisionDetect> collision_detect_;

  HybridAStarResult fallback_path_;

  std::shared_ptr<PolynomialCurveSampling> polynomial_sampling_;
  std::shared_ptr<RSSampling> rs_sampling_;
  std::shared_ptr<SpiralSampling> spiral_sampling_;

  // just for debug, display all result in hmi/plot
  std::vector<DebugAstarSearchPoint> child_node_debug_;
  std::vector<Vec2df32> queue_path_debug_;
  std::vector<Vec2df32> delete_queue_path_debug_;
  std::vector<RSPath> rs_path_h_cost_debug_;

  // for debug
  double collision_check_time_ms_;
  double rs_interpolate_time_ms_;
  double rs_time_ms_;
  double heuristic_time_;
};

}  // namespace planning
