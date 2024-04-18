#include "agent.h"
#include <cstddef>
#include "common.h"
#include "config/basic_type.h"
#include "log.h"
#include "math/box2d.h"
#include "math/linear_interpolation.h"

namespace planning {
namespace agent {

namespace {

constexpr double kLowSpeedAgentSpeedThrMps = 5.0;
constexpr double kTimeResolution = 0.1;
constexpr double kMathEpsilon = 1e-10;
constexpr double kDecayJerkMps3 = -0.2;
constexpr double kPredictionHorizon = 5.0;
}  // namespace

Agent::Agent(const PredictionObject& prediction_object, bool is_static,
             double start_relative_timestamp)
    : agent_id_(prediction_object.id),
      box_({prediction_object.position_x, prediction_object.position_y},
           prediction_object.yaw, prediction_object.length,
           prediction_object.width),
      is_static_(is_static) {
  x_ = prediction_object.position_x;
  y_ = prediction_object.position_y;
  // theta定义
  theta_ = prediction_object.relative_theta;
  speed_ = prediction_object.speed;
  accel_ = prediction_object.acc;
  length_ = prediction_object.length;
  width_ = prediction_object.width;
  type_ = prediction_object.type;
  fusion_source_ = prediction_object.fusion_source;
  timestamp_us_ = prediction_object.timestamp_us;
  timestamp_s_ = prediction_object.timestamp_us / 1000000.0;

  if (prediction_object.trajectory_array.empty()) {
    return;
  }
  // 轨迹默认选第一条
  auto& prediction_trajectory =
      prediction_object.trajectory_array[0].trajectory;
  if (prediction_trajectory.empty()) {
    return;
  }
  // 当前默认trajectories_中只存一条轨迹
  trajectories_.clear();
  trajectory::Trajectory trajectory;
  double cumulative_s = 0.0;
  for (size_t i = 0; i < prediction_trajectory.size(); ++i) {
    auto& traj_point = prediction_trajectory[i];
    trajectory::TrajectoryPoint tp;
    tp.set_vel(traj_point.speed);
    tp.set_acc(0.0);
    tp.set_x(traj_point.x);
    tp.set_y(traj_point.y);
    // TODO：绝对时间 or 相对时间?
    tp.set_absolute_time(traj_point.relative_time);
    tp.set_theta(traj_point.yaw);
    tp.set_s(cumulative_s);
    // tp.set_kappa(const double kappa)

    if (i >= 1) {
      cumulative_s += planning_math::fast_hypot(trajectory[i - 1].x() - tp.x(),
                                                trajectory[i - 1].y() - tp.y());
    }
    trajectory.emplace_back(tp);
  }
  trajectories_.emplace_back(trajectory);
}

const int32_t Agent::agent_id() const { return agent_id_; }
void Agent::set_agent_id(const int32_t agent_id) { agent_id_ = agent_id; }

const double Agent::length() const { return length_; }
void Agent::set_length(const double length) { length_ = length; }

const double Agent::width() const { return width_; }
void Agent::set_width(const double width) { width_ = width; }

const double Agent::height() const { return height_; }
void Agent::set_height(const double height) { height_ = height; }

const double Agent::x() const { return x_; }
void Agent::set_x(const double x) { x_ = x; }

const double Agent::y() const { return y_; }
void Agent::set_y(const double y) { y_ = y; }

const double Agent::theta() const { return theta_; }
void Agent::set_theta(const double theta) { theta_ = theta; }

const double Agent::speed() const { return speed_; }
void Agent::set_speed(const double speed) { speed_ = speed; }

const double Agent::accel() const { return accel_; }
void Agent::set_accel(const double accel) { accel_ = accel; }

const planning_math::Box2d& Agent::box() const { return box_; }
void Agent::set_box(const planning_math::Box2d& box) { box_ = box; }

const unsigned int Agent::fusion_source() const { return fusion_source_; };
void Agent::set_fusion_source(const unsigned int fusion_source) {
  fusion_source_ = fusion_source;
};

const std::vector<trajectory::Trajectory>& Agent::trajectories() const {
  return trajectories_;
}
void Agent::set_trajectories(
    const std::vector<trajectory::Trajectory>& trajectories) {
  trajectories_ = trajectories;

  if (speed_ < kLowSpeedAgentSpeedThrMps && !is_reverse_ && !is_vru_) {
    RecalculateLowSpeedTrajectories();
  }
}
void Agent::add_trajectory(const trajectory::Trajectory& trajectory) {
  trajectories_.emplace_back(trajectory);
}

// const std::vector<trajectory::Trajectory>&
// Agent::trajectories_used_by_st_graph() const {
//   return trajectories_used_by_st_graph_;
// }
// void Agent::set_trajectories_used_by_st_graph(
//     const std::vector<trajectory::Trajectory>& trajectories_used_by_st_graph)
//     {
//   trajectories_used_by_st_graph_ = trajectories_used_by_st_graph;
// }
// void Agent::add_trajectories_used_by_st_graph(const trajectory::Trajectory&
// trajectory) {
//   trajectories_used_by_st_graph_.emplace_back(trajectory);
// }

const AgentDecision& Agent::agent_decision() const { return agent_decision_; }
AgentDecision* const Agent::mutable_agent_decision() {
  return &agent_decision_;
}

const Common::ObjectType Agent::type() const { return type_; }
void Agent::set_type(const Common::ObjectType type) { type_ = type; }

bool Agent::is_vehicle_type() const {
  return type_ == Common::ObjectType::OBJECT_TYPE_BUS ||
         type_ == Common::ObjectType::OBJECT_TYPE_COUPE ||
         type_ == Common::ObjectType::OBJECT_TYPE_MINIBUS ||
         type_ == Common::ObjectType::OBJECT_TYPE_VAN ||
         type_ == Common::ObjectType::OBJECT_TYPE_TRAILER ||
         type_ == Common::ObjectType::OBJECT_TYPE_TRUCK;
}

bool Agent::is_static() const { return is_static_; }
void Agent::set_is_static(const bool is_static) { is_static_ = is_static; }

void Agent::set_b_backup_freemove(const bool b_backup_freemove) {
  b_backup_freemove_ = b_backup_freemove;
}

const bool Agent::is_prediction_cutin() const { return is_prediction_cutin_; }
void Agent::set_is_prediction_cutin(const bool is_prediction_cutin) {
  is_prediction_cutin_ = is_prediction_cutin;
}

const bool Agent::is_cutin() const { return is_cutin_; }
void Agent::set_is_cutin(const bool is_cutin) { is_cutin_ = is_cutin; }

const bool Agent::is_rule_base_cutin() const { return is_rule_base_cutin_; }
void Agent::set_is_rule_base_cutin(const bool is_rule_base_cutin) {
  is_rule_base_cutin_ = is_rule_base_cutin;
}

const double Agent::prediction_cutin_score() const {
  return prediction_cutin_score_;
}
void Agent::set_prediction_cutin_score(const double prediction_cutin_score) {
  prediction_cutin_score_ = prediction_cutin_score;
}

const double Agent::timestamp_s() const { return timestamp_s_; }
void Agent::set_timestamp_s(const double timestamp_s) {
  timestamp_s_ = timestamp_s;
}

const uint64_t Agent::timestamp_us() const { return timestamp_us_; }
void Agent::set_timestamp_us(const uint64_t timestamp_us) {
  timestamp_us_ = timestamp_us;
}

const bool Agent::need_speed_limit() const { return need_speed_limit_; }
void Agent::set_need_speed_limit(const bool need_speed_limit) {
  need_speed_limit_ = need_speed_limit;
}

const bool Agent::is_cone_bucket_cipv() const { return is_cone_bucket_cipv_; }
void Agent::set_is_cone_bucket_cipv(const bool is_cone_bucket_cipv) {
  is_cone_bucket_cipv_ = is_cone_bucket_cipv;
}

const std::pair<double, double> Agent::time_range() const {
  return time_range_;
}
void Agent::set_time_range(const std::pair<double, double> time_range) {
  time_range_ = time_range;
}
const bool Agent::is_time_range_valid() const {
  return (time_range_.first > 0.0) && (time_range_.second > 0.0);
}

const AgentStInfo& Agent::agent_st_info() const { return agent_st_info_; }
AgentStInfo* Agent::mutable_agent_st_info() { return &agent_st_info_; }

const bool Agent::is_reverse() const { return is_reverse_; }

void Agent::set_is_reverse(const bool is_reverse) { is_reverse_ = is_reverse; }

const bool Agent::has_low_spd_unstable_trajectory() const {
  return has_low_spd_unstable_trajectory_;
}

void Agent::set_has_low_spd_unstable_trajectory(
    const bool has_low_spd_unstable_trajectory) {
  has_low_spd_unstable_trajectory_ = has_low_spd_unstable_trajectory;
}

const bool Agent::is_vru() const { return is_vru_; }

void Agent::set_is_vru(const bool is_vru) { is_vru_ = is_vru; }

const bool Agent::is_sod() const { return is_sod_; }

void Agent::set_is_sod(const bool is_sod) { is_sod_ = is_sod; }

const bool Agent::need_backward_extend() const { return need_backward_extend_; }

void Agent::set_need_backward_extend(const bool need_backward_extend) {
  need_backward_extend_ = need_backward_extend;
}

const bool Agent::is_cut_out_for_lane_change() const {
  return is_cut_out_for_lane_change_;
}

void Agent::set_is_cut_out_for_lane_change(
    const bool is_cut_out_for_lane_change) {
  is_cut_out_for_lane_change_ = is_cut_out_for_lane_change;
}

const bool Agent::is_tfl_virtual_obs() const { return is_tfl_virtual_obs_; }

void Agent::set_is_tfl_virtual_obs(bool is_tfl_virtual_obs) {
  is_tfl_virtual_obs_ = is_tfl_virtual_obs;
}

void Agent::RecalculateLowSpeedTrajectories() {
  const double init_accel = accel_;
  const double init_speed = std::fmax(speed_, 0.0);
  std::vector<double> processed_acc;
  std::vector<double> processed_speed;
  std::vector<double> processed_lon_position;
  processed_lon_position.reserve(51);
  processed_speed.reserve(51);
  processed_acc.reserve(51);

  // calculate low speed agent lon prediction info
  for (double relative_time = 0.0; relative_time < kPredictionHorizon;
       relative_time += kTimeResolution) {
    if (init_accel + kMathEpsilon > 0) {
      // acceling obj's accel is assumed to decay to zero to avoid over-reaction
      double accel_to_zero_time = -init_accel / kDecayJerkMps3;
      if (relative_time < accel_to_zero_time) {
        const double t2 = relative_time * relative_time;
        const double t3 = t2 * relative_time;
        processed_acc.emplace_back(init_accel + kDecayJerkMps3 * relative_time);
        processed_speed.emplace_back(init_speed + init_accel * relative_time +
                                     kDecayJerkMps3 * t2 * 0.5);
        processed_lon_position.emplace_back(init_speed * relative_time +
                                            0.5 * init_accel * t2 +
                                            0.167 * kDecayJerkMps3 * t3);
      } else {
        const double t2 = accel_to_zero_time * accel_to_zero_time;
        const double t3 = t2 * accel_to_zero_time;
        processed_acc.emplace_back(0.0);
        processed_speed.emplace_back(init_speed +
                                     init_accel * accel_to_zero_time +
                                     kDecayJerkMps3 * t2 * 0.5);
        processed_lon_position.emplace_back(
            init_speed * accel_to_zero_time + 0.5 * init_accel * t2 +
            0.167 * kDecayJerkMps3 * t3 +
            processed_speed.back() * (relative_time - accel_to_zero_time));
      }
    } else {
      // deceling obj is assume to stop with const accel to avoid late braking
      if (init_accel < 0.0 && relative_time > (-init_speed / init_accel)) {
        processed_lon_position.emplace_back(init_speed * init_speed * 0.5 /
                                            std::fabs(init_accel));
        processed_speed.emplace_back(0.0);
        processed_acc.emplace_back(init_accel);
        break;
      } else {
        processed_acc.emplace_back(init_accel);
        processed_speed.emplace_back(init_speed + init_accel * relative_time);
        processed_lon_position.emplace_back(init_speed * relative_time +
                                            0.5 * init_accel * relative_time *
                                                relative_time);
      }
    }
  }

  if (processed_lon_position.empty() || processed_speed.empty() ||
      processed_acc.empty()) {
    return;
  }
  // assign speed to predicted path
  for (const auto& trajectory : trajectories_) {
    if (trajectory.empty()) {
      continue;
    }
    const double trajectory_len = trajectory.back().s();
    const trajectory::TrajectoryPoint last_trajectory_point = trajectory.back();
    const double last_x = last_trajectory_point.x();
    const double last_y = last_trajectory_point.y();
    const double last_theta = last_trajectory_point.theta();
    const double last_t = last_trajectory_point.absolute_time();
    const double start_tmie = trajectory.front().absolute_time();
    trajectory::Trajectory processed_trajectory;
    for (int i = 0; i < processed_speed.size(); ++i) {
      if (processed_trajectory.empty() ||
          processed_lon_position[i] < trajectory_len) {
        // query nearest trajectory_point and interpolate
        const int end_idx =
            trajectory.QueryLowerBoundPointByS(processed_lon_position[i]);
        const int start_idx = std::max(end_idx - 1, 0);
        auto trajectory_point =
            start_idx == end_idx
                ? trajectory[start_idx]
                : trajectory[start_idx].InterpolateTrajectoryPointByS(
                      trajectory[end_idx], processed_lon_position[i]);
        if (!processed_trajectory.empty() &&
            trajectory_point.absolute_time() -
                    processed_trajectory.back().absolute_time() <
                kMathEpsilon) {
          double target_time =
              processed_trajectory.back().absolute_time() + kTimeResolution;
          trajectory_point.set_absolute_time(target_time);
        }
        trajectory_point.set_s(processed_lon_position[i]);
        trajectory_point.set_vel(processed_speed[i]);
        trajectory_point.set_acc(processed_acc[i]);
        processed_trajectory.emplace_back(trajectory_point);
      } else {
        const double ds = processed_lon_position[i] - trajectory_len;
        double target_time =
            processed_trajectory.back().absolute_time() + kTimeResolution;
        if (processed_trajectory.back().vel() > kMathEpsilon) {
          double dt = ds / processed_trajectory.back().vel();
          target_time = std::fmin(
              target_time, processed_trajectory.back().absolute_time() + dt);
        }
        double x = last_x + ds * std::cos(last_theta);
        double y = last_y + ds * std::sin(last_theta);
        double s = processed_lon_position[i];
        processed_trajectory.emplace_back(x, y, last_theta, processed_speed[i],
                                          processed_acc[i], target_time, 0.0,
                                          0.0, processed_lon_position[i], 0.0);
      }
    }
    // trajectories_used_by_st_graph_.emplace_back(processed_trajectory);
  }
}

}  // namespace agent
}  // namespace planning