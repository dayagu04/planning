#ifndef MSQUARE_REAL_TIME_DECISION_PLANNING_PLANNER_LANE_CHANGE_DECIDER_H_
#define MSQUARE_REAL_TIME_DECISION_PLANNING_PLANNER_LANE_CHANGE_DECIDER_H_

#include "environmental_model.h"
#include "fusion_road.pb.h"
#include "lateral_obstacle.h"
#include "real_time_lon_behavior_planner.pb.h"
#include "session.h"
#include "tasks/task_interface/vision_longitudinal_behavior_planner_output.h"

namespace planning {

struct RTLaneChangeParams {
  double most_front_car_dist;
  double most_rear_car_dist;
  double cost_minus;
  double v_rel_bufer;
};

// class RealTimeLaneChangeDecider : public Decider {
class RealTimeLaneChangeDecider {
 public:
  // explicit RealTimeLaneChangeDecider(const TaskConfig &config);
  explicit RealTimeLaneChangeDecider(planning::common::LaneChangeInfo lc_info);

  std::pair<int, int> get_target_gap() { return target_gap_; }
  std::vector<GapInfo> get_gap_list() { return gap_list_; }
  double get_lc_safe_dist(double buffer, double t_gap, double v_ego_p) {
    return buffer + v_ego_p * t_gap;
  }
  double get_target_gap_cost() { return target_gap_cost_; }

  void update_lc_info(planning::common::LaneChangeInfo *new_lc_info) {
    lc_info_.Swap(new_lc_info);
  }

  void feed_config_and_target_cars(
      bool is_merging_, RTLaneChangeParams params, double dis_to_change_point,
      std::vector<const planning::common::TrackedObjectInfo *> &target_cars,
      const planning::common::TrackedObjectInfo &lead_one, double v_ego);

  TargetObstacle nearest_rear_car_track() { return nearest_rear_car_track_; };

  bool process();

 private:
  bool is_on_target(const FrenetObstacleBoundary &sl_boundary);
  GapInfo check_gap_valid(const TargetObstacle &rear_car,
                          const TargetObstacle &front_car);
  static bool compare_distance_asc(const TargetObstacle &obs1,
                                   const TargetObstacle &obs2);
  double calc_lane_width(const double &s,
                         const std::vector<RefPointFrenet> &ref_line);
  double clip(const double x, const double lo, const double hi);
  double calc_time_for_lane_change(TargetObstacle base_car,
                                   TargetObstacle front_car, GapInfo gap_info,
                                   const double safety_distance,
                                   const double max_v);
  double calc_desired_distance(const double v_lead, const double v_ego);
  double calc_desired_speed(const double d_rel, const double d_des,
                            const double v_lead);
  static bool compare_cost_asc(const GapInfo &gap1, const GapInfo &gap2);
  double interp(double x, const std::vector<double> &xp,
                const std::vector<double> &fp);

  const std::vector<double> _T_GAP_VEGO_BP{5.0, 15.0, 30.0};
  const std::vector<double> _T_GAP_VEGO_V{1.35, 1.55, 2.0};
  const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
  const std::vector<double> _L_SLOPE_V{0.35, 0.08};
  const std::vector<double> _P_SLOPE_BP{0., 40.0};
  const std::vector<double> _P_SLOPE_V{0.8, 0.2};

  bool called_in_state_machine_{false};
  TargetObstacle lead_car_;
  std::vector<TargetObstacle> obstacle_on_target_;
  std::pair<int, int> target_gap_;
  double target_gap_cost_{std::numeric_limits<double>::max()};
  int lane_change_direction_;
  double v_limit_;
  const double min_l_threshold_ = 1.3;
  double max_l_threshold_ = 5.0;
  TargetObstacle most_front_car_ = {-1, 100.0, 10.0};
  TargetObstacle most_rear_car_ = {-2, -110.0, -10.0};
  std::vector<GapInfo> gap_list_;
  std::vector<RefPointFrenet> cur_lane_, target_lane_;
  double dis_to_change_point_;
  int lc_map_decision_;
  FusionRoad::LaneType current_lane_type_;

  int target_lane_id_;
  int current_lane_id_;
  int leader_car_id_;

  std::vector<const planning::common::TrackedObjectInfo *> target_cars_;
  const planning::common::TrackedObjectInfo *lead_one_ = nullptr;
  TargetObstacle nearest_rear_car_track_{-1, DBL_MAX, DBL_MAX};
  double v_ego_ = 0.0;
  bool is_merging_ = false;

  RTLaneChangeParams params_;
  planning::common::LaneChangeInfo lc_info_;
};

}  // namespace planning

#endif
