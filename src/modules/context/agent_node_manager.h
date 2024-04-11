
#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

// #include "common/speed/st_boundary.h"
#include "config/basic_type.h"
#include "frenet_obstacle.h"
#include "obstacle_manager.h"
#include "session.h"
#include "speed/st_point.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
using namespace planning;

namespace planning {
enum MonotonicStatus { NONMONOTIONIC = 0, INCREASE, DECREASE };
class AgentNodeManager {
 public:
  AgentNodeManager(){};
  AgentNodeManager(std::vector<std::pair<double, double>> origin_refline_points,
                   std::vector<std::pair<double, double>> target_refline_points,
                   std::unordered_map<int, Obstacle> map_gs_care_obstacles,
                   std::vector<int> map_target_lane_obstacles,
                   std::vector<int> map_origin_lane_obstacles, int target_state)
      : map_gs_care_obstacles_(map_gs_care_obstacles),
        map_target_lane_obstacles_(map_target_lane_obstacles),
        map_origin_lane_obstacles_(map_origin_lane_obstacles),
        target_state_(target_state) {
    for (auto i = 0; i < origin_refline_points.size(); i++) {
      x_vec_origin_lane_.emplace_back(origin_refline_points[i].first);
      y_vec_origin_lane_.emplace_back(origin_refline_points[i].second);
    }

    for (auto i = 0; i < target_refline_points.size(); i++) {
      x_vec_target_lane_.emplace_back(target_refline_points[i].first);
      y_vec_target_lane_.emplace_back(target_refline_points[i].second);
    }

    Update();
  };
  ~AgentNodeManager() = default;

  bool init();
  bool clear();
  bool Update();
  bool Process();

  void PybindHandle();

  const std::unordered_map<int, Obstacle> &map_gs_care_obstacles() const {
    return map_gs_care_obstacles_;
  };

  const std::unordered_map<int64_t, ObstaclePredicatedInfo> &
  agent_node_origin_lane_map() const {
    return agent_node_origin_lane_map_;
  };

  const std::unordered_map<int64_t, ObstaclePredicatedInfo> &
  agent_node_target_lane_map() const {
    return agent_node_target_lane_map_;
  };

  const std::vector<int64_t> &ids_sorted_target_lane() const {
    return ids_sorted_target_lane_;
  };

  const std::vector<int64_t> &ids_sorted_origin_lane() const {
    return ids_sorted_origin_lane_;
  };

  void set_input_info(
      std::shared_ptr<KDPath> origin_coord,
      std::shared_ptr<KDPath> target_coord, const int request,
      const std::vector<int> &ids_obstacle_in_origin_lane,
      const std::vector<int> &ids_obstacle_in_target_lane,
      const std::unordered_map<int, Obstacle> &gs_care_obstacles);

  //------------------- simulate ----------------------

  void RefineObjInitState(const int64_t &obj_id, const double &obj_add_vel,
                          const double &obj_add_s);
  bool InferObjTrajByLane(
      ObstaclePredicatedInfo &obstacle_predicated_info, const Obstacle &obj,
      const int obj_in_which_lane = 0);  // 0 --origin lane; 1-- target lane

  bool InferObjTrajByKDPath(ObstaclePredicatedInfo &obstacle_predicated_info,
                            const Obstacle &obj, const int obj_in_target_lane);

 public:
  bool HandleAllObjPredInfo();

  bool HandleAllObjNormalInfo();

  bool InferAllObjPredInfoByKDPath();
  bool InferAllObjNormalInfoByKDPath();

  // -------------------new code -----------------------
  int FindNearestIndex(const std::vector<double> &inputVector,
                       const int &monotonic_status, double target);

  std::pair<std::vector<std::vector<double>>,
            std::vector<std::pair<size_t, size_t>>>
  PartitionIntoMonotonicSectionsWithIndices(
      const std::vector<double> &lane_input);

  std::vector<std::vector<double>> DivideLaneIntoMonoIntervals(
      const std::vector<double> &lane_input);

  std::vector<std::pair<size_t, size_t>> GetLaneMonoIntervalIndex(
      std::vector<std::vector<double>> &lane_mono_intervals);

  int FindSectionIndex(const std::vector<std::pair<size_t, size_t>> &indices,
                       size_t indexToFind);

  bool InferConstSpeedObstacleTraj(
      const Obstacle &obj, const MonotonicStatus mono_status,
      const double delta_y, const double traj_length,
      const std::vector<double> &x_values, const std::vector<double> &y_values,
      const std::vector<double> &s_vec, const int obj_in_which_lane,
      ObstaclePredicatedInfo &obj_predicted_points);

  double Distance(const Point2D &a, const Point2D &b);

  Point2D FindClosestPointOnCurve(const Point2D &point,
                                  const std::vector<Point2D> &curvePoints);
  // ----------------------------------------------------

  void StitchLaneValidInfo(const int &monotonic_status,
                           const double obj_traj_length,
                           std::vector<double> &filtered_x_values,
                           std::vector<double> &filtered_y_values,
                           std::vector<double> &s_vec);

  void PrepareLaneCubicInfo();
  void PrePareLaneCubicInfo(std::vector<double> &x_target_vec,
                            std::vector<double> &y_target_vec,
                            std::vector<double> &x_origin_vec,
                            std::vector<double> &y_origin_vec);
  MonotonicStatus IsMonotonic(const std::vector<double> &x_values);
  MonotonicStatus IsMonotonic(
      std::vector<std::vector<double>> &divide_lane_intervals);
  void ConstructDefaultStPassCorridor();

  void ConstructGaps();

  void SortNodeByS();

  std::unordered_map<int64_t, ObstaclePredicatedInfo>
      agent_node_origin_lane_map_;
  std::unordered_map<int64_t, ObstaclePredicatedInfo>
      agent_node_target_lane_map_;

  // std::vector<int64_t> ids_sorted_target_lane_;
  // std::vector<int64_t> ids_sorted_origin_lane_;

  // pnc::mathlib::spline cart_spline_target_lane_;
  // pnc::mathlib::spline cart_spline_origin_lane_;

  std::vector<double> x_vec_target_lane_;
  std::vector<double> y_vec_target_lane_;
  std::vector<double> x_vec_origin_lane_;
  std::vector<double> y_vec_origin_lane_;

  std::shared_ptr<planning_math::KDPath> frenet_coord_;
  int target_state_{0};  // 0 -- no change; 1--left change; 2--right change
  std::unordered_map<int, Obstacle> map_gs_care_obstacles_;

  std::vector<int> map_target_lane_obstacles_;
  std::vector<int> map_origin_lane_obstacles_;
  std::vector<int64_t> ids_sorted_target_lane_;  // sorted id
  std::vector<int64_t> ids_sorted_origin_lane_;
  // std::vector<Gap> gap_list_;
};

}  // namespace planning