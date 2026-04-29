#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "hierarchy_occupancy_grid_map.h"
#include "hybrid_a_star.h"
#include "node3d.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "src/library/convex_collision_detection/gjk2d_interface.h"
#include "src/library/occupancy_grid_map/point_cloud_obstacle.h"
#include "target_pose_regulator.h"

namespace planning {

class HybridAStarInterface {
 public:
  HybridAStarInterface();

  ~HybridAStarInterface();

  // todo: need to unify same vehicle chassis params for on lane driving and
  // parking
  int Init(const VehicleParam& veh_param);

  // for now, use slot coordinate. you can call this API in one thread.
  void GeneratePath(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                    const AstarRequest& request);

  const AstarSearchState GetFullLengthPath(HybridAStarResult* result);

  AstarRequest* GetMutableRequest() { return &request_; }

  const AstarRequest& GetConstRequest() const { return request_; }

  static void GetRSPathInFullPath(std::vector<float>& x, std::vector<float>& y,
                                  std::vector<float>& phi,
                                  const HybridAStarResult& result);

  const HybridAStarResult* GetConstFullLengthPath() const;

  const bool GetFirstSegmentPath(std::vector<AStarPathPoint>& result);

  static const AstarSearchState TransformFirstSegmentPath(
      std::vector<AStarPathPoint>& result, const HybridAStarResult& full_path,
      const Pose2f& start);

  const int GetFallBackPath(std::vector<AStarPathPoint>& result);

  const int GetFallBackPath(HybridAStarResult* result);

  const std::array<bool, 6> GetFeasibleDirections() {
    return stable_feasible_directions_;
  };

  const ParkObstacleList& GetConstObstacles() const;

  ParkObstacleList& GetMutableObstacleList();

  const Pose2f GetAstarTargetPose() const { return target_regulator_goal_; }

  // multi-thread, input
  void UpdateInput(const ParkObstacleList& obs_list,
                   const AstarRequest& request);

  // multi-thread, output
  void UpdateOutput();

  HierarchyEulerDistanceTransform* GetMutableHierarchyEDT() {
    return &hierarchy_edt_;
  }
  const HierarchyEulerDistanceTransform* GetHierarchyEulerDistanceTransform()
      const {
    return &hierarchy_edt_;
  }

  // for debug
  void GetRSPathHeuristic(std::vector<std::vector<Vec2f>>& path_list);

  // for debug
  const std::vector<DebugAstarSearchPoint>& GetChildNodeForDebug();

  // for debug
  const std::vector<Vec2f>& GetPriorQueueNode();
  const std::vector<Vec2f>& GetDelNodeQueueNode();
  // for debug,retired
  void GetNodeListMessage(planning::common::AstarNodeList* list);

  // for debug
  void GetNodeListMessage(std::vector<std::vector<Eigen::Vector2d>>& list);

  // for debug
  void GetRSPathForDebug(std::vector<float>& x, std::vector<float>& y,
                         std::vector<float>& phi);

  const ParkReferenceLine& GetConstRefLine() const;

  // for debug
  void GetPolynomialPathForDebug(std::vector<float>& x, std::vector<float>& y,
                                 std::vector<float>& phi);

  // for debug
  MultiHeightFootPrintView* GetCircleFootPrint(
      const HierarchySafeBuffer buffer);

  const SearchTimeBenchmark& GetTimeBenchmark() const {
    return time_benchmark_;
  }

  const SearchTrajectoryInfo& GetSearchTrajInfo() const {
    return search_traj_info_;
  }

  const std::array<HybridAStarResult, 9>& GetTarjCandidates() const {
    return traj_candidates_;
  }

  // for debug
  void GetRoundRobinTarget(std::vector<Pose2f>& candidates);

 private:
  int UpdateEDT(const bool use_height_info = false);

  void PathClear();

  void UpdateGridMapBound();

  void UpdateEDTBasePose(Pose2f& ogm_base_pose);

  const Pose2f& GetStartPoint();

  const Pose2f& GetGoalPoint();

  const bool IsEgoOverlapWithSlot();

  // 基于采样的揉库API, will be retired.
  void PathSamplingForScenarioRunning();

  void PathSearchForScenarioTry(TargetPoseRegulator& regulator);

  // 基于搜索的路径生成API
  void ParkInPathSearchForScenarioRunning(TargetPoseRegulator& regulator,
                                          const float ego_obs_dist,
                                          const bool is_ego_overlap_with_slot);
  void ParkOutPathSearchForScenarioRunning(TargetPoseRegulator& regulator,
                                           const float ego_obs_dist,
                                           const bool is_ego_overlap_with_slot);

  // todo: move it to safe buffer decider
  // return safe buffer when path is insidet slot.
  // Need to consider:
  // 1. distance from ego to obstacle;
  // 2. distance from target pose to obstacle;
  const float GetLatBufferForInsideSlot(const float target_obs_dist,
                                        const float ego_obs_dist,
                                        const bool is_ego_overlap_with_slot);

  void PathCandidateCompare(const int gear_change_num_buffer);
  void PrependPolynomialPathToHybridAStar(
      const std::vector<AStarPathPoint>& polynomial_path,
      HybridAStarResult* hybrid_path);

  void GenerateRefLine();
  void ParkingDirectionAttempt(const float& advised_lat_buffer_inside);
  const float GenLatBufferForCandidatePose();
  void DebugSearchTraj(const size_t path_index);
  const bool ShouldStopSearchEarly(double& search_time,
                                   const size_t path_index);

 private:
  // read vehicle param from file
  VehicleParam vehicle_param_;
  PlannerOpenSpaceConfig config_;
  // xmin, xmax, ymin, ymax
  MapBound map_bounds_;

  std::shared_ptr<HybridAStar> hybrid_astar_;
  std::shared_ptr<GridSearch> dp_heuristic_generator_;
  // path = astar node path + rs path.
  HybridAStarResult* best_traj_;
  std::array<HybridAStarResult, 9> traj_candidates_;

  size_t traj_candidates_size_;

  Pose2f ego_state_;
  // 对于垂直、平行车位，goal_state位于车位中心线上.
  // 位姿调节器依赖这个pose重新计算搜索目标点 target_regulator_goal.
  Pose2f goal_state_;
  // 目标调节器会计算一个合适的目标
  Pose2f target_regulator_goal_;

  AstarSearchState search_state_;

  AstarRequest request_;

  ParkObstacleList obs_;

  ObstacleClearZone clear_zone_;

  HierarchyOccupancyGridMap ogm_;
  HierarchyEulerDistanceTransform hierarchy_edt_;
  ParkReferenceLine ref_line_;

  int gear_switch_number_scenario_try_;

  // Stable version of feasible_directions_ exposed to HMI.
  // Only flips after kFeasibleConfirmThresh consecutive successes or
  // kInfeasibleConfirmThresh consecutive failures, preventing flickering.
  std::array<bool, 6> stable_feasible_directions_{};
  std::array<int8_t, 6> feasible_confirm_count_{};
  static constexpr int8_t kFeasibleConfirmThresh = 2;
  static constexpr int8_t kInfeasibleConfirmThresh = -3;

  // for debug
  SearchTimeBenchmark time_benchmark_;
  SearchTrajectoryInfo search_traj_info_;
};

}  // namespace planning