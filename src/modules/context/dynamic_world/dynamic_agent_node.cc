#include "dynamic_agent_node.h"
#include <memory>
#include <utility>
#include <vector>
#include "ifly_time.h"
#include "reference_path.h"
#include "trajectory/center_line_point.h"
// #include "trajectory/path_point.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"

namespace planning {
namespace planning_data {

namespace {

constexpr double kMaxThresholdForInLaneCheck = 0.2;
constexpr double kMinSafeDistance = 0.5;
constexpr double kHeadingDiffForAgainstTrafficRad = 1.0472;
constexpr double kMaxSpeedValueForExtremelySlowMps = 0.5;
constexpr double kMaxAccelValueForExtremelySlowMps2 = 0.5;
constexpr double kDefaultLaneWidth = 3.75;
constexpr double kMaxDistanceBeforeStart = 30.0;
constexpr double kMaxDistanceAfterEnd = 30.0;

void GetApproximatedCornerSlCoordinates(
    const double center_s, const double center_l, const double delta_theta,
    const double half_length, const double half_width,
    std::array<std::pair<double, double>, 4>* const ptr_sl_corners) {
  const double cos_delta_theta = std::cos(delta_theta);
  const double sin_delta_theta = std::sin(delta_theta);

  const double a = cos_delta_theta * half_length;
  const double b = -sin_delta_theta * half_width;
  const double c = sin_delta_theta * half_length;
  const double d = cos_delta_theta * half_width;
  ptr_sl_corners->at(0) = {center_s + a + b, center_l + c + d};
  ptr_sl_corners->at(1) = {center_s - a + b, center_l - c + d};
  ptr_sl_corners->at(2) = {center_s - a - b, center_l - c - d};
  ptr_sl_corners->at(3) = {center_s + a - b, center_l + c - d};
}
}  // namespace

DynamicAgentNode::DynamicAgentNode(const agent::Agent* agent,
                                   const VirtualLane& lane,
                                   const ReferencePath& ref_line,
                                   const trajectory::TrajectoryPoint ego_state)
    : agent_(agent) {
  double kEgoWidth = vehicle_param_.width;
  double kEgoFrontEdgeToCenterDistance = vehicle_param_.front_edge_to_center;
  double kEgoBackEdgeToCenterDistance = vehicle_param_.back_edge_to_center;
  is_valid_ = false;
  if (agent == nullptr) {
    return;
  }
  is_agent_within_lane_ = false;
  is_agent_most_within_lane_ = false;
  is_agent_against_traffic_ = false;
  is_agent_extremely_slow_ = false;
  node_lane_id_ = lane.get_virtual_id();
  node_agent_id_ = agent->agent_id();

  // whether is necessary to get node_id?
  node_id_ = (int64_t)(int64_t(node_lane_id_) << 16 | node_agent_id_);
  // node_id_ = agent->agent_id();

  // Get closest matched reference point
  // TODO: Add frenet coord construct, when it is nullptr
  if (ref_line.get_frenet_coord() == nullptr) {
    std::vector<planning_math::PathPoint> ref_line_points;
    if (ref_line.get_points().size() < 1) {
      return;
    }
    ref_line_points.reserve(ref_line.get_points().size());
    for (const auto& ref_line_point : ref_line.get_points()) {
      planning_math::PathPoint path_point{ref_line_point.path_point.x,
                                          ref_line_point.path_point.y};
      ref_line_points.emplace_back(path_point);
    }
    coord_ = std::make_shared<KDPath>(std::move(ref_line_points));
  } else {
    coord_ = ref_line.get_frenet_coord();
  }

  std::vector<trajectory::CenterLinePoint> center_line_points;
  if (lane.lane_points().size() < 1) {
    return;
  }
  center_line_points.reserve(lane.lane_points().size());
  for (const auto& ref_line_point : lane.lane_points()) {
    trajectory::CenterLinePoint center_line_point{
        ref_line_point.local_point.x,
        ref_line_point.local_point.y,
        ref_line_point.s,
        0,
        ref_line_point.local_heading,
        0,
        0,
        0,
        ref_line_point.distance_to_left_lane_border,
        ref_line_point.distance_to_right_lane_border};
    center_line_points.emplace_back(center_line_point);
  }

  trajectory::CenterLinePoint agent_matched_point;
  double agent_match_l_value = 0.0;
  if (!GetClosestCenterLinePoint(agent->x(), agent->y(), center_line_points,
                                 &agent_matched_point, &agent_match_l_value)) {
    return;
  }

  // pair<min_l, max_l>
  auto agent_l_min_max = CalculateLRange(
      agent_matched_point.s(), agent_match_l_value, agent_matched_point.theta(),
      agent->theta(), agent->length(), agent->width());

  double left_width, right_width;
  if (!agent_matched_point.get_left_lane_width(&left_width)) {
    left_width = kDefaultLaneWidth / 2.0;
  }

  if (!agent_matched_point.get_right_lane_width(&right_width)) {
    right_width = kDefaultLaneWidth / 2.0;
  }

  is_agent_within_lane_ =
      IsAgentPartiallyWithinLane(agent_l_min_max, left_width, right_width,
                                 kEgoWidth, kMaxThresholdForInLaneCheck);
  const double lane_width_at_point =
      std::fabs(left_width) + std::fabs(right_width);
  is_agent_most_within_lane_ =
      is_agent_within_lane_ &&
      (std::fabs(agent_match_l_value) < lane_width_at_point / 2.0);

  // delta theta > pi/3
  is_agent_against_traffic_ =
      std::abs(planning_math::AngleDiff(agent_matched_point.theta(),
                                        agent->theta())) >
      kHeadingDiffForAgainstTrafficRad;

  is_agent_extremely_slow_ =
      agent->speed() < kMaxSpeedValueForExtremelySlowMps &&
      agent->accel() < kMaxAccelValueForExtremelySlowMps2;

  // get ego matched point on static_lane
  trajectory::CenterLinePoint ego_matched_point;
  double ego_match_l_value = 0.0;
  if (!GetClosestCenterLinePoint(ego_state.x(), ego_state.y(),
                                 center_line_points, &ego_matched_point,
                                 &ego_match_l_value)) {
    return;
  }
  node_s_ = agent_matched_point.s();
  node_length_ = agent->length();
  node_to_ego_distance_ = agent_matched_point.s() - ego_matched_point.s();
  bool is_agent_in_front = false;
  bool is_agent_in_rear = false;
  if (node_to_ego_distance_ >= 0.0) {
    node_back_edge_to_ego_front_edge_distance_ = node_to_ego_distance_ -
                                                 kEgoFrontEdgeToCenterDistance -
                                                 agent->length() * 0.5;
    if (node_back_edge_to_ego_front_edge_distance_ >= 0.0) {
      is_agent_in_front = true;
    }
  } else {
    node_to_ego_distance_ = -node_to_ego_distance_;
    node_front_edge_to_ego_back_edge_distance_ = node_to_ego_distance_ -
                                                 kEgoBackEdgeToCenterDistance -
                                                 agent->length() * 0.5;
    if (node_front_edge_to_ego_back_edge_distance_ >= 0.0) {
      is_agent_in_rear = true;
    }
  }
  is_agent_has_overlap_with_ego_ = !(is_agent_in_front || is_agent_in_rear);

  bool is_last_prediction_point_within_lane = false;
  trajectory::CenterLinePoint last_prediction_point_matched_point;
  double last_prediction_point_l_value = 0.0;
  for (size_t trajectory_index = 0;
       trajectory_index < agent->trajectories().size(); ++trajectory_index) {
    const auto& trajectory = agent->trajectories().at(trajectory_index);
    const auto& last_point = trajectory.back();
    if (!GetClosestCenterLinePoint(last_point.x(), last_point.y(),
                                   center_line_points,
                                   &last_prediction_point_matched_point,
                                   &last_prediction_point_l_value)) {
      continue;
    }
    const std::pair<double, double> last_prediction_point_l_min_max =
        CalculateLRange(last_prediction_point_matched_point.s(),
                        last_prediction_point_l_value,
                        last_prediction_point_matched_point.theta(),
                        agent->theta(), agent->length(), agent->width());
    double left_width, right_width;
    if (!last_prediction_point_matched_point.get_left_lane_width(&left_width)) {
      left_width = kDefaultLaneWidth / 2.0;
    }

    if (!last_prediction_point_matched_point.get_right_lane_width(
            &right_width)) {
      right_width = kDefaultLaneWidth / 2.0;
    }
    is_last_prediction_point_within_lane = IsAgentPartiallyWithinLane(
        last_prediction_point_l_min_max, left_width, right_width, kEgoWidth,
        kMaxThresholdForInLaneCheck);
    if (is_last_prediction_point_within_lane) {
      break;
    }
  }
  // only agent match with lane are valid
  is_valid_ = (is_agent_within_lane_ || is_last_prediction_point_within_lane ||
               agent_match_l_value * last_prediction_point_l_value < 0);
}

std::pair<double, double> DynamicAgentNode::CalculateLRange(
    const double agent_match_s, const double agent_match_l,
    const double agent_match_theta, const double agent_theta,
    const double agent_length, const double agent_width) {
  const double half_length = 0.5 * agent_length;
  const double half_width = 0.5 * agent_width;
  const double delta_theta = agent_theta - agent_match_theta;

  std::array<std::pair<double, double>, 4> sl_corners;
  GetApproximatedCornerSlCoordinates(agent_match_s, agent_match_l, delta_theta,
                                     half_length, half_width, &sl_corners);

  double l_min = std::numeric_limits<double>::max();
  double l_max = std::numeric_limits<double>::lowest();
  for (const auto& sl_corner : sl_corners) {
    l_min = std::fmin(l_min, sl_corner.second);
    l_max = std::fmax(l_max, sl_corner.second);
  }
  return {l_min, l_max};
}

bool DynamicAgentNode::IsAgentPartiallyWithinLane(
    const std::pair<double, double>& l_min_max,
    const double agent_match_left_lane_width,
    const double agent_match_right_lane_width, const double ego_vehicle_width,
    const double in_lane_distance_to_boundary_threshold) {
  const double half_width = ego_vehicle_width * 0.5 + kMinSafeDistance;
  const bool is_agent_on_left_outside =
      l_min_max.first > std::min(std::fabs(agent_match_left_lane_width) -
                                     in_lane_distance_to_boundary_threshold,
                                 half_width);
  const bool is_agent_on_right_outside =
      l_min_max.second < -(std::min(std::fabs(agent_match_right_lane_width) -
                                        in_lane_distance_to_boundary_threshold,
                                    half_width));
  const bool is_within_lane =
      !(is_agent_on_left_outside || is_agent_on_right_outside);
  return is_within_lane;
}

bool DynamicAgentNode::GetClosestCenterLinePoint(
    const double x, const double y,
    std::vector<trajectory::CenterLinePoint>& center_line_point_Vec,
    trajectory::CenterLinePoint* const match_center_point,
    double* const l) const {
  if (match_center_point == nullptr || l == nullptr) {
    return false;
  }
  double match_s = 0.0;
  double match_l = 0.0;
  if (!coord_->XYToSL(x, y, &match_s, &match_l)) {
    return false;
  }
  if (match_s < center_line_point_Vec.front().s() - kMaxDistanceBeforeStart ||
      match_s > center_line_point_Vec.back().s() - kMaxDistanceAfterEnd) {
    return false;
  }
  *l = match_l;
  // Get center line point by match_s
  auto it_lower = QueryLowerBound(center_line_point_Vec, match_s);
  if (it_lower == center_line_point_Vec.begin()) {
    it_lower += 1;
  }
  if (it_lower == center_line_point_Vec.end()) {
    it_lower -= 1;
  }
  *match_center_point = (it_lower - 1)->GetInterpolateByS(*(it_lower), match_s);
  return true;
}

std::vector<trajectory::CenterLinePoint>::const_iterator
DynamicAgentNode::QueryLowerBound(
    const std::vector<trajectory::CenterLinePoint>& center_line_points,
    const double path_s) const {
  auto func = [](const trajectory::CenterLinePoint& tp, const double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(center_line_points.begin(), center_line_points.end(),
                          path_s, func);
}

bool DynamicAgentNode::is_valid() const { return is_valid_; }
const int64_t DynamicAgentNode::node_id() const { return node_id_; }
const int32_t DynamicAgentNode::node_lane_id() const { return node_lane_id_; }
const int32_t DynamicAgentNode::node_agent_id() const { return node_agent_id_; }

// NOTICE: always make sure node is valid before call this part of interfaces
// otherwise the return value might not be correct
double DynamicAgentNode::node_x() const {
  if (agent_ == nullptr) {
    return 0.0;
  }
  return agent_->x();
}
double DynamicAgentNode::node_y() const {
  if (agent_ == nullptr) {
    return 0.0;
  }
  return agent_->y();
}
double DynamicAgentNode::node_theta() const {
  if (agent_ == nullptr) {
    return 0.0;
  }
  return agent_->theta();
}
double DynamicAgentNode::node_width() const {
  if (agent_ == nullptr) {
    return 0.0;
  }
  return agent_->width();
}
double DynamicAgentNode::node_speed() const {
  if (agent_ == nullptr) {
    return 0.0;
  }
  return agent_->speed();
}
double DynamicAgentNode::node_accel() const {
  if (agent_ == nullptr) {
    return 0.0;
  }
  return agent_->accel();
}

double DynamicAgentNode::node_s() const { return node_s_; }
double DynamicAgentNode::node_t() const { return node_t_; }
double DynamicAgentNode::node_length() const { return node_length_; }
double DynamicAgentNode::node_to_ego_distance() const {
  return node_to_ego_distance_;
}
double DynamicAgentNode::node_back_edge_to_ego_front_edge_distance() const {
  return node_back_edge_to_ego_front_edge_distance_;
}
double DynamicAgentNode::node_front_edge_to_ego_back_edge_distance() const {
  return node_front_edge_to_ego_back_edge_distance_;
}

const std::vector<trajectory::Trajectory>& DynamicAgentNode::node_trajectories()
    const {
  return agent_->trajectories();
}

const iflyauto::ObjectType DynamicAgentNode::type() const {
  return agent_->type();
}

bool DynamicAgentNode::is_agent_within_lane() const {
  return is_agent_within_lane_;
}
bool DynamicAgentNode::is_agent_most_within_lane() const {
  return is_agent_most_within_lane_;
}
bool DynamicAgentNode::is_agent_against_traffic() const {
  return is_agent_against_traffic_;
}
bool DynamicAgentNode::is_agent_extremely_slow() const {
  return is_agent_extremely_slow_;
}
bool DynamicAgentNode::is_agent_has_overlap_with_ego() const {
  return is_agent_has_overlap_with_ego_;
}

const int64_t DynamicAgentNode::front_node_id() const { return front_node_id_; }
void DynamicAgentNode::set_front_node_id(const int64_t& front_node_id) {
  front_node_id_ = front_node_id;
}
const int64_t DynamicAgentNode::rear_node_id() const { return rear_node_id_; }
void DynamicAgentNode::set_rear_node_id(const int64_t& rear_node_id) {
  rear_node_id_ = rear_node_id;
}

bool DynamicAgentNode::is_VRU_type() const { return agent_->is_vru(); }

bool DynamicAgentNode::is_cone_type() const { return agent_->is_sod(); }

bool DynamicAgentNode::is_vehicle_type() const {
  return agent_->is_vehicle_type();
}

}  // namespace planning_data
}  // namespace planning