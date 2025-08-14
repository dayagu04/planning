#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include "agent_decision.h"
#include "math/box2d.h"
#include "prediction_object.h"
#include "trajectory/trajectory.h"

namespace planning {
namespace agent {

// 与iflyauto::ObjectType对齐
enum class AgentType {
  UNKNOWN = 0,                // 未知障碍物
  UNKNOWN_MOVABLE = 1,        // 未知可移动障碍物
  UNKNOWN_IMMOVABLE = 2,      // 未知不可移动障碍物
  COUPE = 3,                  // 轿车
  MINIBUS = 4,                // 面包车
  VAN = 5,                    // 厢式轿车
  BUS = 6,                    // 大型客车
  TRUCK = 7,                  // 卡车
  TRAILER = 8,                // 拖车
  BICYCLE = 9,                // 自行车
  MOTORCYCLE = 10,            // 摩托车
  TRICYCLE = 11,              // 三轮车
  PEDESTRIAN = 12,            // 行人
  ANIMAL = 13,                // 动物
  TRAFFIC_CONE = 14,          // 交通锥
  TRAFFIC_BARREL = 15,        // 隔离墩
  TRAFFIC_TEM_SIGN = 16,      // 临时指示牌
  FENCE = 17,                 // 栅栏
  CYCLE_RIDING = 18,          // 人骑着自行车
  MOTORCYCLE_RIDING = 19,     // 人骑着摩托车
  TRICYCLE_RIDING = 20,       // 人骑着三轮车
  WATER_SAFETY_BARRIER = 21,  // 水马
  CTASH_BARREL = 22,          // 防撞桶
  WARNING_TRIANGLE = 23,      // 三角牌
  PARKING_GATE = 24,          // 停车杆
  POLE = 25,                  // 杆
  SPEED_BUMP = 26,            // 减速带
  PARKING_SPOT_LOCK = 27,     // 地锁
  SQUARE_COLUMN = 28,         // 方柱
  CYLINDER = 29,              // 圆柱
  SUV = 30,                   // SUV车辆
  VIRTUAL = 31,               // 虚拟障碍物
};

struct AgentStInfo {
  bool is_in_st_graph_ = false;
  int32_t st_graph_count_ = 0;
  double lateral_buffer_ = 0.0;
  double min_t_ = -1.0;
};

struct AgentDefaultInfo {
  /***common id info***/
  const static int32_t kNoAgentId = -1;
  // Note: agent virtual id = xxId_Base + num(which maybe a relative agent id)
  // RADS virtual agent id base
  const static int32_t kRadsVirtualAgentId_Base = 200000;

  /***specific id which is defined by user***/
  const static int32_t kRadsStopDestinationVirtualAgentId =
      kRadsVirtualAgentId_Base + 1;
};

struct TmpPathPoint {
  double x, y, theta;
};

class Agent {
 public:
  Agent() = default;
  Agent(const Agent& agent);
  Agent(const PredictionObject& prediction_object, bool is_static,
        double start_relative_timestamp);

  const int32_t agent_id() const;
  void set_agent_id(const int32_t agent_id);

  const double length() const;
  void set_length(const double length);

  const double width() const;
  void set_width(const double width);

  const double height() const;
  void set_height(const double height);

  const double x() const;
  void set_x(const double x);

  const double y() const;
  void set_y(const double y);

  const double theta() const;
  void set_theta(const double theta);

  const double speed() const;
  void set_speed(const double speed);

  const double speed_fusion() const;
  void set_speed_fusion(const double speed_fusion);

  const double accel() const;
  void set_accel(const double accel);

  const double accel_fusion() const;
  void set_accel_fusion(const double accel_fusion);

  const AgentType type() const;
  void set_type(const AgentType type);

  // void set_agent_light_state_info(const AgentLightStateInfo&
  // agent_light_state_info); const AgentLightStateInfo&
  // agent_light_state_info() const;

  const unsigned int fusion_source() const;
  void set_fusion_source(const unsigned int fusion_source);

  bool is_vehicle_type() const;

  bool is_static() const;
  void set_is_static(const bool is_static);

  const planning_math::Box2d& box() const;
  void set_box(const planning_math::Box2d& box);

  const std::vector<trajectory::Trajectory>& trajectories() const;
  void set_trajectories(
      const std::vector<trajectory::Trajectory>& trajectories);
  void add_trajectory(const trajectory::Trajectory& trajectory);

  const std::vector<trajectory::Trajectory>& trajectories_used_by_st_graph()
      const;
  void set_trajectories_used_by_st_graph(
      const std::vector<trajectory::Trajectory>& trajectories_used_by_st_graph);
  void add_trajectories_used_by_st_graph(
      const trajectory::Trajectory& trajectory);

  const AgentDecision& agent_decision() const;
  AgentDecision* const mutable_agent_decision();

  const bool b_backup_freemove() const;
  void set_b_backup_freemove(const bool b_backup_freemove);

  const bool is_prediction_cutin() const;
  void set_is_prediction_cutin(const bool is_prediction_cutin);

  const bool is_cutin() const;
  void set_is_cutin(const bool is_cutin);

  const bool is_cutout() const;
  void set_is_cutout(const bool is_cutout);

  const bool is_rule_base_cutin() const;
  void set_is_rule_base_cutin(const bool is_rule_base_cutin);

  const double prediction_cutin_score() const;
  void set_prediction_cutin_score(const double prediction_cutin_score);

  const bool is_crossing() const;
  void set_is_crossing(const bool is_crossing);

  const double timestamp_s() const;
  void set_timestamp_s(const double timestamp_s);

  const uint64_t timestamp_us() const;
  void set_timestamp_us(const uint64_t timestamp_us);

  const bool need_speed_limit() const;
  void set_need_speed_limit(const bool need_speed_limit);

  const bool is_cone_bucket_cipv() const;
  void set_is_cone_bucket_cipv(const bool is_cone_bucket_cipv);

  const std::pair<double, double> time_range() const;
  void set_time_range(const std::pair<double, double> time_range);
  const bool is_time_range_valid() const;

  const AgentStInfo& agent_st_info() const;
  AgentStInfo* mutable_agent_st_info();

  const bool is_reverse() const;
  void set_is_reverse(const bool is_reverse);

  const bool is_reverse_in_large_curv() const;
  void set_is_reverse_in_large_curv(const bool is_reverse_in_large_curv);

  const bool is_far_in_large_curv() const;
  void set_is_far_in_large_curv(const bool is_far_in_large_curv);

  const bool is_reverse_cutin() const;
  void set_is_reverse_cutin(const bool is_reverse);

  const bool has_low_spd_unstable_trajectory() const;
  void set_has_low_spd_unstable_trajectory(
      const bool has_low_spd_unstable_trajectory);

  const bool is_vru() const;
  void set_is_vru(const bool is_vru);

  const bool is_sod() const;
  void set_is_sod(const bool is_sod);

  const bool need_backward_extend() const;
  void set_need_backward_extend(const bool need_backward_extend);

  const bool is_cut_out_for_lane_change() const;
  void set_is_cut_out_for_lane_change(const bool is_cut_out_for_lane_change);

  const bool is_tfl_virtual_obs() const;
  void set_is_tfl_virtual_obs(bool is_tfl_virtual_obs);

  const bool is_stop_destination_virtual_obs() const;
  void set_is_stop_destination_virtual_obs(
      bool is_stop_destination_virtual_obs);
  const bool is_lane_borrow_virtual_obs() const;
  void set_is_lane_borrow_virtual_obs(
      bool is_lane_borrow_virtual_obs);

  const double d_path() const;
  void set_d_path(double d_path);

  const double d_rel() const;
  void set_d_rel(double d_rel);

  ~Agent() = default;

 private:
  void RecalculateLowSpeedTrajectories();

 private:
  int32_t agent_id_ = -1;

  AgentType type_ = AgentType::UNKNOWN;

  double length_ = 0.0;
  double width_ = 0.0;
  double height_ = 0.0;

  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  double speed_ = 0.0;
  double speed_fusion_ = 0.0;
  double accel_ = 0.0;
  double accel_fusion_ = 0.0;
  planning_math::Box2d box_;

  double d_path_ = std::numeric_limits<double>::max();
  double d_rel_ = std::numeric_limits<double>::max();

  std::vector<trajectory::Trajectory> trajectories_;
  std::vector<trajectory::Trajectory> trajectories_used_by_st_graph_;

  int32_t top_probability_index_ = 0;
  double top_probability_ = 0.0;

  bool b_backup_freemove_ = false;

  // prediction cut in of current frame
  bool is_prediction_cutin_ = false;

  // rule base cut in of current frame
  bool is_rule_base_cutin_ = false;

  // steady cut in flag(count for 3 frames)
  bool is_cutin_ = false;
  double prediction_cutin_score_ = 0.0;

  bool is_cutout_ = false;

  bool is_crossing_ = false;

  double timestamp_s_ = 0.0;
  uint64_t timestamp_us_ = 0;
  AgentDecision agent_decision_;
  // AgentLightStateInfo agent_light_state_info_;

  // speed limit for cone bucket
  bool need_speed_limit_ = false;

  bool is_cone_bucket_cipv_ = false;
  bool is_static_ = false;
  // info for st graph
  std::pair<double, double> time_range_ = {-1.0, -1.0};

  AgentStInfo agent_st_info_;

  bool is_reverse_ = false;
  bool has_low_spd_unstable_trajectory_ = false;
  bool is_vru_ = false;
  bool is_sod_ = false;
  bool need_backward_extend_ = false;
  bool is_far_in_large_curv_ = false;
  bool is_reverse_in_large_curv_ = false;
  bool is_reverse_cutin_ = false;

  bool is_cut_out_for_lane_change_ = false;

  bool is_tfl_virtual_obs_ = false;
  bool is_stop_destination_virtual_obs_ = false;
  bool is_lane_borrow_virtual_obs_ = false;
  unsigned int fusion_source_;
};

}  // namespace agent
}  // namespace planning