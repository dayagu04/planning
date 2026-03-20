/**
 * @file: st_graph_data.h
 * @brief: data with map info and obstacle info
 **/
#ifndef MODULES_PLANNING_OPTIMIZERSTGRAPH_DATA_H_
#define MODULES_PLANNING_OPTIMIZERSTGRAPH_DATA_H_

#include <vector>

#include "config/message_type.h"
#include "speed/speed_limit.h"
#include "speed/st_boundary.h"

namespace planning {

enum class ObstacleTag : int {
  TRAVERSE = 0,
  CONVERGE = 1,
  MERGE = 2,
  FRONT = 3,
  BACK = 4,
  UNKNOWN = -1
};

// a struct for obstacles
struct Barrier {
  int id = -1;
  ObjectType type = ObjectType::NOT_KNOW;
  double s_min = 200;
  double s_max = 200;
  double vel = 120 / 3.6;  // velocity
  double r_frenet = 0.0;
  double r_close = 100.0;
  double yaw_relative_frenet = 0.0;
  double dr_close = 0.0;
  bool isLatNudge = false;
  double sigma_x = 0.0;
  double sigma_y = 0.0;
};

class StGraphData {
 public:
  using BarrierPair = std::pair<Barrier, Barrier>;
  using BarrierList = std::vector<Barrier>;
  using BarrierArray = std::vector<std::vector<Barrier>>;
  using BarrierPairList = std::vector<BarrierPair>;
  using DoublePair = std::pair<double, double>;
  using DoublePairList = std::vector<DoublePair>;
  using ObstacleTagList = std::vector<ObstacleTag>;
  using IntObstacleTypeUmap = std::unordered_map<int, ObstacleType>;

 public:
  StGraphData() = default;

  void LoadData(const std::vector<const STBoundary*>& st_boundaries,
                const double min_s_on_st_boundaries,
                const PncTrajectoryPoint& init_point,
                const SpeedLimit& speed_limit, const double path_data_length,
                const double total_time_by_conf);

  void load_output_data(
      const BarrierPairList& lane_keep_sv, const BarrierArray& normal_sv,
      const BarrierList& rear_sv, const BarrierList& yield_sv,
      const BarrierList& lane_keep_potential_sv,
      const ObstacleTagList& obstacle_tag_list,
      const IntObstacleTypeUmap& obstacle_first_time_state_map,
      bool dynamic_prebrake_signal, bool static_prebrake_signal,
      bool is_prebrake_target_transverse);

  bool is_initialized() const { return init_; }

  void resetBoundary(const std::vector<const STBoundary*>& st_boundaries);

  const std::vector<const STBoundary*>& st_boundaries() const;

  double min_s_on_st_boundaries() const;

  const PncTrajectoryPoint& init_point() const;

  const SpeedLimit& speed_limit() const;

  double path_length() const;

  double total_time_by_conf() const;

  // interface for current st graph data
  const BarrierPairList& get_lane_keep_sv() const;

  const BarrierArray& get_normal_sv() const;

  const BarrierList& get_rear_sv() const;

  const BarrierList& get_yield_sv() const;

  const BarrierList& get_potential_sv() const;

  const ObstacleTagList& get_obstacle_tag() const;

  const IntObstacleTypeUmap& get_obstacle_first_time_state_map() const;

  bool get_prebrake_signal_from_static() const;

  bool get_prebrake_signal_for_transverse() const;

  bool get_prebrake_signal_from_dynamic() const;

 private:
  bool init_ = false;
  std::vector<const STBoundary*> st_boundaries_;
  double min_s_on_st_boundaries_ = 0.0;
  PncTrajectoryPoint init_point_;
  SpeedLimit speed_limit_;
  double path_data_length_ = 0.0;
  // double path_length_by_conf_ = 0.0;
  double total_time_by_conf_ = 0.0;

  BarrierPairList lane_keep_sv_;
  BarrierArray normal_sv_;
  BarrierList rear_sv_;
  BarrierList yield_sv_;
  BarrierList lane_keep_potential_sv_;
  ObstacleTagList obstacle_tag_list_;
  IntObstacleTypeUmap obstacle_first_time_state_map_;
  bool dynamic_prebrake_signal_;
  bool static_prebrake_signal_;
  bool is_prebrake_target_transverse_;
};

}  // namespace planning

#endif /* MODULES_PLANNING_OPTIMIZERSTGRAPH_DATA_H_ */