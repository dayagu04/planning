#include "agent_node_manager.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "config/basic_type.h"
#include "define/geometry.h"
#include "environmental_model.h"
#include "obstacle.h"
#include "reference_path.h"
#include "utils/frenet_coordinate_system.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"

namespace planning {
using namespace planning::planning_math;
constexpr double Eps = 1e-3;
constexpr double LaneCheckBuffer = 0.55;
constexpr double delta_t = 0.1;

bool AgentNodeManager::init() {
  agent_node_origin_lane_map_.clear();
  agent_node_target_lane_map_.clear();
  ids_sorted_target_lane_.clear();
  ids_sorted_origin_lane_.clear();
  map_gs_care_obstacles_.clear();
  x_vec_origin_lane_.clear();
  y_vec_origin_lane_.clear();
  x_vec_target_lane_.clear();
  y_vec_target_lane_.clear();
  map_gs_care_obstacles_.clear();
  map_target_lane_obstacles_.clear();
  map_origin_lane_obstacles_.clear();
  return true;
}

void AgentNodeManager::set_input_info(
    std::shared_ptr<KDPath> origin_coord, std::shared_ptr<KDPath> target_coord,
    const int request, const std::vector<int> &ids_obstacle_in_origin_lane,
    const std::vector<int> &ids_obstacle_in_target_lane,
    const std::unordered_map<int, Obstacle> &gs_care_obstacles) {
  if (!init()) {
    return;
  }
  target_state_ = request;
  // frenet_coord_ = origin_coord;
  frenet_coord_ = target_coord;

  map_gs_care_obstacles_.reserve(gs_care_obstacles.size());
  for (const auto &pair : gs_care_obstacles) {
    map_gs_care_obstacles_.emplace(pair.first, pair.second);
  }
  map_target_lane_obstacles_.reserve(ids_obstacle_in_target_lane.size());
  std::copy(ids_obstacle_in_target_lane.begin(),
            ids_obstacle_in_target_lane.end(),
            std::back_inserter(map_target_lane_obstacles_));
  map_origin_lane_obstacles_.reserve(ids_obstacle_in_origin_lane.size());
  std::copy(ids_obstacle_in_origin_lane.begin(),
            ids_obstacle_in_origin_lane.end(),
            std::back_inserter(map_origin_lane_obstacles_));

  x_vec_origin_lane_.reserve(origin_coord->path_points().size());
  y_vec_origin_lane_.reserve(origin_coord->path_points().size());
  for (auto i = 0; i < origin_coord->path_points().size(); i++) {
    x_vec_origin_lane_.emplace_back(origin_coord->path_points()[i].x());
    y_vec_origin_lane_.emplace_back(origin_coord->path_points()[i].y());
  }
  x_vec_target_lane_.reserve(target_coord->path_points().size());
  y_vec_target_lane_.reserve(target_coord->path_points().size());
  for (auto i = 0; i < target_coord->path_points().size(); i++) {
    x_vec_target_lane_.emplace_back(target_coord->path_points()[i].x());
    y_vec_target_lane_.emplace_back(target_coord->path_points()[i].y());
  }

  return;
}

int AgentNodeManager::FindNearestIndex(const std::vector<double> &input_vector,
                                       const int &monotonic_status,
                                       double target) {
  int low = 0;
  int high = input_vector.size() - 1;
  int nearestIndex = -1;
  double minDiff = std::numeric_limits<double>::max();

  while (low <= high) {
    int mid = low + (high - low) / 2;
    double diff = input_vector[mid] - target;
    if (monotonic_status == MonotonicStatus::INCREASE) {
      if (diff >= 0) {
        if (diff < minDiff) {
          minDiff = diff;
          nearestIndex = mid;
        }
        high = mid - 1;
      } else {
        low = mid + 1;
      }
    } else {
      if (diff <= 0) {
        if (-diff < minDiff) {
          minDiff = -diff;
          nearestIndex = mid;
        }
        high = mid - 1;
      } else {
        low = mid + 1;
      }
    }
  }

  return nearestIndex;
}

std::vector<std::vector<double>> AgentNodeManager::DivideLaneIntoMonoIntervals(
    const std::vector<double> &lane_input) {
  std::vector<std::vector<double>> intervals;

  if (lane_input.empty()) {
    return intervals;
  }
  std::vector<double> increasing_interval;
  std::vector<double> decreasing_interval;

  for (size_t i = 1; i < lane_input.size(); ++i) {
    if (lane_input[i] >= lane_input[i - 1]) {
      if (i == 1) {
        increasing_interval.emplace_back(lane_input[0]);
      }
      increasing_interval.push_back(lane_input[i]);
      if (!decreasing_interval.empty()) {
        intervals.push_back(decreasing_interval);
        decreasing_interval.clear();
      }
    } else {
      if (i == 1) {
        decreasing_interval.emplace_back(lane_input[0]);
      }
      decreasing_interval.push_back(lane_input[i]);
      if (!increasing_interval.empty()) {
        intervals.push_back(increasing_interval);
        increasing_interval.clear();
      }
    }
  }

  if (!increasing_interval.empty()) {
    intervals.push_back(increasing_interval);
  }

  if (!decreasing_interval.empty()) {
    intervals.push_back(decreasing_interval);
  }

  return intervals;
}

int AgentNodeManager::FindSectionIndex(
    const std::vector<std::pair<size_t, size_t>> &indices, size_t target_idx) {
  for (size_t i = 0; i < indices.size(); ++i) {
    if (target_idx >= indices[i].first && target_idx <= indices[i].second) {
      return i;
    }
  }
  return -1;  // Index not found in any section
}

bool AgentNodeManager::InferObjTrajByLane(
    ObstaclePredicatedInfo &obstacle_predicated_info, const Obstacle &obj,
    const int obj_in_which_lane) {
  double obj_x_0 = obj.x_center();
  double obj_y_0 = obj.y_center();

  const std::vector<double> &lane_x_vec =
      obj_in_which_lane == 0 ? x_vec_origin_lane_ : x_vec_target_lane_;
  const std::vector<double> &lane_y_vec =
      obj_in_which_lane == 0 ? y_vec_origin_lane_ : y_vec_target_lane_;

  std::vector<Point2D> lane_points;
  lane_points.resize(lane_x_vec.size());
  for (size_t i = 0; i < lane_x_vec.size(); ++i) {
    lane_points[i] = {lane_x_vec[i], lane_y_vec[i]};
  }
  bool infer_ok{true};
  // Point2D obj_position = {obj_x_0, obj_y_0}; //TODO:@liuhao why delete

  // Point2D projection_point = FindClosestPointOnCurve(obj_position,
  // lane_points);

  // if (projection_point.x < start_x_value || projection_point.x >
  // back_x_value) {
  //   return;
  // }

  std::vector<std::vector<double>> lane_mono_intervals =
      DivideLaneIntoMonoIntervals(lane_x_vec);

  std::vector<std::pair<size_t, size_t>> lane_mono_interval_index =
      GetLaneMonoIntervalIndex(lane_mono_intervals);

  MonotonicStatus mono_status = IsMonotonic(lane_mono_intervals);

  auto objx0_index = FindNearestIndex(lane_x_vec, mono_status, obj_x_0);

  auto sections_index = FindSectionIndex(lane_mono_interval_index, objx0_index);

  pnc::mathlib::spline cart_spline_line_info;
  std::vector<double> reverse_lane_x_vec;
  std::vector<double> reverse_lane_y_vec;

  if (mono_status == MonotonicStatus::INCREASE) {
    cart_spline_line_info.set_points(lane_x_vec, lane_y_vec);

  } else if (mono_status == MonotonicStatus::DECREASE) {
    reverse_lane_x_vec = lane_x_vec;
    reverse_lane_y_vec = lane_y_vec;

    std::reverse(reverse_lane_x_vec.begin(), reverse_lane_x_vec.end());
    std::reverse(reverse_lane_y_vec.begin(), reverse_lane_y_vec.end());
    cart_spline_line_info.set_points(reverse_lane_x_vec, reverse_lane_y_vec);

  } else if (mono_status == MonotonicStatus::NONMONOTIONIC &&
             sections_index != -1) {
    std::vector<double> section_y_values(  // TODO@liuhao: consider the short
                                           // length of the corresponding part
        lane_y_vec.begin() + lane_mono_interval_index[sections_index].first,
        lane_y_vec.begin() + lane_mono_interval_index[sections_index].second +
            1);
    auto section_x_values = lane_mono_intervals[sections_index];
    if (section_x_values[0] > section_x_values[1]) {
      std::reverse(section_x_values.begin(), section_x_values.end());
      std::reverse(section_y_values.begin(), section_y_values.end());
    }
    cart_spline_line_info.set_points(section_x_values, section_y_values);
  } else {
    ILOG_ERROR << "MonotonicStatus bug, id is" << obj.id();
    return false;
  }

  double lane_intersection_point_y = cart_spline_line_info(obj_x_0);
  double delta_y = lane_intersection_point_y - obj_y_0;

  std::vector<double> lane_predict_points_y;
  lane_predict_points_y.reserve(lane_x_vec.size());
  std::transform(lane_y_vec.begin(), lane_y_vec.end(),
                 std::back_inserter(lane_predict_points_y),
                 [=](double y) { return y - delta_y; });

  double obj_traj_length =
      obj.velocity() * delta_t * (prediction_traj_points_size_ - 1);

  std::vector<double> s_vec;
  if (mono_status == MonotonicStatus::NONMONOTIONIC) {
    std::vector<double> non_mono_x_value(
        lane_x_vec.begin() + lane_mono_interval_index[sections_index].first,
        lane_x_vec.begin() + lane_mono_interval_index[sections_index].second);
    std::vector<double> non_mono_y_value(
        lane_y_vec.begin() + lane_mono_interval_index[sections_index].first,
        lane_y_vec.begin() + lane_mono_interval_index[sections_index].second);
    StitchLaneValidInfo(mono_status, obj_traj_length, non_mono_x_value,
                        non_mono_y_value, s_vec);
    infer_ok = InferConstSpeedObstacleTraj(
        obj, mono_status, delta_y, obj_traj_length, non_mono_x_value,
        non_mono_y_value, s_vec, obj_in_which_lane, obstacle_predicated_info);

  } else {
    std::vector<double> filtered_x_value(
        lane_x_vec.begin() + objx0_index,
        lane_x_vec.end());  // TODO: reversed x indexs
    std::vector<double> filtered_y_value(
        lane_predict_points_y.begin() + objx0_index,
        lane_predict_points_y.end());
    StitchLaneValidInfo(mono_status, obj_traj_length, filtered_x_value,
                        filtered_y_value, s_vec);
    infer_ok = InferConstSpeedObstacleTraj(
        obj, mono_status, delta_y, obj_traj_length, filtered_x_value,
        filtered_y_value, s_vec, obj_in_which_lane, obstacle_predicated_info);
  }

  return infer_ok;
}

bool AgentNodeManager::InferObjTrajByKDPath(
    ObstaclePredicatedInfo &obstacle_predicated_info, const Obstacle &obj,
    const int obj_in_target_lane) {
  const double init_s = obstacle_predicated_info.cur_s;
  bool use_cur_s = false;

  if (init_s < -50 || init_s > 180.) {
    use_cur_s = true;
  }

  auto &obj_pred_traj = obstacle_predicated_info.obstacle_pred_info;
  obj_pred_traj.resize(prediction_traj_points_size_);

  // for (auto i = 0; i < prediction_traj_points_size_ + 1; i++) {
  //   obj_pred_traj[i].x =
  //       obstacle_predicated_info.origin_x;  // x y is keep origin
  //   obj_pred_traj[i].y = obstacle_predicated_info.origin_y;

  //   obj_pred_traj[i].s =
  //       use_cur_s ? obstacle_predicated_info.cur_s
  //                 : std::fmin(obstacle_predicated_info.cur_s +
  //                                 (i - 1) * obstacle_predicated_info.raw_vel
  //                                 *
  //                                     delta_t,
  //                             frenet_coord_->Length());
  //   obj_pred_traj[i].l = obstacle_predicated_info.cur_l;

  //   // reserve, the xy can be convert by kdpath
  //   obj_pred_traj[i].heading_angle = 0.;
  // }
  obj_pred_traj[0].x = obstacle_predicated_info.origin_x;
  obj_pred_traj[0].y = obstacle_predicated_info.origin_y;
  obj_pred_traj[0].heading_angle =
      obstacle_predicated_info.origin_heading_angle;
  obj_pred_traj[0].s = obstacle_predicated_info.cur_s;
  obj_pred_traj[0].l = obstacle_predicated_info.cur_l;
  obstacle_predicated_info.x_vec.emplace_back(obj_pred_traj[0].x);
  obstacle_predicated_info.y_vec.emplace_back(obj_pred_traj[0].y);
  obstacle_predicated_info.heading_angle_vec.emplace_back(
      obj_pred_traj[0].heading_angle);
  for (auto i = 1; i < prediction_traj_points_size_; i++) {
    obj_pred_traj[i].s =
        use_cur_s ? obstacle_predicated_info.cur_s
                  : std::fmin(obstacle_predicated_info.cur_s +
                                  (i - 1) * obstacle_predicated_info.raw_vel *
                                      delta_t,
                              frenet_coord_->Length());
    obj_pred_traj[i].l = obstacle_predicated_info.cur_l;
    Point2D frenet_point{obj_pred_traj[i].s, obj_pred_traj[i].l};
    Point2D cart_point{0.0, 0.0};
    frenet_coord_->SLToXY(frenet_point, cart_point);
    obj_pred_traj[i].x = cart_point.x;
    obj_pred_traj[i].y = cart_point.y;
    // reserve, the xy can be convert by kdpath
    obj_pred_traj[i].heading_angle =
        obstacle_predicated_info.origin_heading_angle;
    obstacle_predicated_info.x_vec.emplace_back(obj_pred_traj[i].x);
    obstacle_predicated_info.y_vec.emplace_back(obj_pred_traj[i].y);
    obstacle_predicated_info.heading_angle_vec.emplace_back(
        obj_pred_traj[i].heading_angle);
  }

  return true;
}

std::vector<std::pair<size_t, size_t>>
AgentNodeManager::GetLaneMonoIntervalIndex(
    std::vector<std::vector<double>> &lane_mono_intervals) {
  size_t start_idx = 0;
  std::vector<std::pair<size_t, size_t>> intervals_index;
  intervals_index.reserve(lane_mono_intervals.size());

  for (auto i = 0; i < lane_mono_intervals.size(); i++) {
    intervals_index.emplace_back(std::make_pair(
        start_idx, start_idx + lane_mono_intervals[i].size() - 1));
    start_idx = intervals_index[i].second + 1;
  }
  return intervals_index;
}

void AgentNodeManager::StitchLaneValidInfo(
    const int &monotonic_status, const double obj_traj_length,
    std::vector<double> &filtered_x_values,
    std::vector<double> &filtered_y_values, std::vector<double> &s_vec) {
  s_vec.reserve(filtered_x_values.size());
  s_vec.emplace_back(0.);

  for (int i = 1; i < filtered_x_values.size(); i++) {
    double s_vec_tem =
        s_vec[i - 1] +
        std::fmax(std::hypot(filtered_x_values[i] - filtered_x_values[i - 1],
                             filtered_y_values[i] - filtered_y_values[i - 1]),
                  Eps);
    s_vec.emplace_back(s_vec_tem);
  }

  if (s_vec.back() < obj_traj_length + Eps) {
    double last_x = filtered_x_values.back();
    double second_last_x = filtered_x_values[filtered_x_values.size() - 2];
    double diff_x = last_x - second_last_x;

    Eigen::Vector2d compensate_vector(
        diff_x, filtered_y_values.back() -
                    filtered_y_values[filtered_y_values.size() - 2]);
    Eigen::Vector2d normlized_vec = compensate_vector.normalized();
    double angle = std::atan2(normlized_vec.y(), normlized_vec.x());
    double length_end = obj_traj_length - s_vec.back() + Eps;

    double filtered_x_values_end =
        filtered_x_values.back() + length_end * std::cos(angle);
    double filtered_y_values_end =
        filtered_y_values.back() + length_end * std::sin(angle);

    filtered_x_values.emplace_back(filtered_x_values_end);
    filtered_y_values.emplace_back(filtered_y_values_end);
    double compensated_ds =
        std::fmax(std::hypot(filtered_x_values_end - last_x,
                             filtered_y_values_end - filtered_y_values.back()),
                  Eps);
    s_vec.emplace_back(compensated_ds + s_vec.back());
    return;
  }
}

bool AgentNodeManager::InferConstSpeedObstacleTraj(
    const Obstacle &obj, const MonotonicStatus mono_status,
    const double delta_y, const double traj_length,
    const std::vector<double> &x_values, const std::vector<double> &y_values,
    const std::vector<double> &s_vec, const int obj_in_which_lane,
    ObstaclePredicatedInfo &obj_predicted_points) {
  if (x_values.size() < 3) {
    ILOG_ERROR << "lane x vec num is little, id is" << obj.id();
    return false;  // NOTE: core dump时 注意检查
  }
  // spline
  pnc::mathlib::spline x_s_spline;
  x_s_spline.set_points(s_vec, x_values);
  pnc::mathlib::spline y_s_spline;
  y_s_spline.set_points(s_vec, y_values);

  double sn = 0.;
  auto &obj_pred_traj = obj_predicted_points.obstacle_pred_info;
  obj_pred_traj.resize(prediction_traj_points_size_);

  Point2D obj_cart_position{obj.x_center(), obj.y_center()};
  Point2D obj_frenet_position{
      300, obj_in_which_lane == 0 ? 0. : (target_state_ == 1 ? 3. : -3.)};
  if (!frenet_coord_->XYToSL(obj_cart_position, obj_frenet_position)) {
  }

  for (auto i = 0; i < prediction_traj_points_size_;
       i++) {  // only update pred x, y, heading
    obj_pred_traj[i].x = x_s_spline(sn);
    obj_pred_traj[i].y = y_s_spline(sn);
    obj_pred_traj[i].heading_angle =
        std::atan2(y_s_spline.deriv(1, sn), x_s_spline.deriv(1, sn));
    double delta_s = obj.velocity() * delta_t;
    sn += delta_s;

    Point2D cart_point{obj_pred_traj[i].x, obj_pred_traj[i].y};
    Point2D frenet_point{100, 0.};
    auto status = frenet_coord_->XYPointToSLPoint(cart_point, frenet_point);
    if (status == KDPathStatus::FALL) {
      frenet_point.x = -100.;

    } else if (status == KDPathStatus::EXCEED) {
      frenet_point.x = 200.;
    }

    obj_pred_traj[i].s = frenet_point.x;
    obj_pred_traj[i].l = frenet_point.y;
  }
  return true;
}

MonotonicStatus AgentNodeManager::IsMonotonic(
    std::vector<std::vector<double>> &divide_lane_intervals) {
  if (divide_lane_intervals.size() > 1) {
    return MonotonicStatus::NONMONOTIONIC;

  } else {
    const auto &mono_interval = divide_lane_intervals[0];
    if (mono_interval[1] > mono_interval[0]) {
      return MonotonicStatus::INCREASE;
    } else {
      return MonotonicStatus::DECREASE;
    }
  }
}

double AgentNodeManager::Distance(const Point2D &a, const Point2D &b) {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

// Point2D AgentNodeManager::FindClosestPointOnCurve(
//     const Point2D &point, const std::vector<Point2D> &curve_points) {
//   Point2D closest_point = curve_points[0];
//   double minDistance = Distance(point, closestPoint);

//   for (const Point2D &curvePoint : curvePoints) {
//     double currentDistance = Distance(point, curvePoint);
//     if (currentDistance < minDistance) {
//       minDistance = currentDistance;
//       closestPoint = curvePoint;
//     }
//   }

//   return closestPoint;
// }

bool AgentNodeManager::HandleAllObjPredInfo() {
  // Recheck unreasonable objs
  for (auto &obj : map_origin_lane_obstacles_) {  // finally, need to check
                                                  // relative x,y to check
    auto iter = map_gs_care_obstacles_.find(obj);
    if (iter != map_gs_care_obstacles_.end()) {
      if (iter->second.y_relative_center() - iter->second.width() * 0.5 >
              1.75 + LaneCheckBuffer ||
          iter->second.y_relative_center() + iter->second.width() * 0.5 <
              -1.75 + LaneCheckBuffer) {
        map_origin_lane_obstacles_.erase(
            std::remove(map_origin_lane_obstacles_.begin(),
                        map_origin_lane_obstacles_.end(), obj),
            map_origin_lane_obstacles_.end());
      }
    }
  }
  if (target_state_ == 1) {
    for (auto &obj : map_target_lane_obstacles_) {  // finally, need to check
                                                    // relative x,y to check
      auto iter = map_gs_care_obstacles_.find(obj);
      if (iter != map_gs_care_obstacles_.end()) {
        if (iter->second.y_relative_center() - iter->second.width() * 0.5 >
                5.25 + LaneCheckBuffer ||
            iter->second.y_relative_center() + iter->second.width() * 0.5 <
                1.75 - LaneCheckBuffer) {
          map_target_lane_obstacles_.erase(
              std::remove(map_target_lane_obstacles_.begin(),
                          map_target_lane_obstacles_.end(), obj),
              map_target_lane_obstacles_.end());
        }
      }
    }
  } else if (target_state_ == 2) {
    for (auto &obj : map_target_lane_obstacles_) {  // finally, need to check
                                                    // relative x,y to check
      auto iter = map_gs_care_obstacles_.find(obj);
      if (iter != map_gs_care_obstacles_.end()) {
        if (iter->second.y_relative_center() - iter->second.width() * 0.5 >
                -1.75 + LaneCheckBuffer ||
            iter->second.y_relative_center() + iter->second.width() * 0.5 <
                -5.75 - LaneCheckBuffer) {
          map_target_lane_obstacles_.erase(
              std::remove(map_target_lane_obstacles_.begin(),
                          map_target_lane_obstacles_.end(), obj),
              map_target_lane_obstacles_.end());
        }
      }
    }
  }

  if (!map_origin_lane_obstacles_.empty()) {
    bool infer_ok{false};
    for (auto &agent : map_origin_lane_obstacles_) {
      ObstaclePredicatedInfo obj_infos;
      auto iter = map_gs_care_obstacles_.find(agent);
      if (iter != map_gs_care_obstacles_.end()) {
        infer_ok = InferObjTrajByLane(obj_infos, iter->second, true);
        if (infer_ok) {
          agent_node_origin_lane_map_.insert(std::make_pair(agent, obj_infos));
        }
      }
    }
  };
  if (!map_target_lane_obstacles_.empty()) {
    bool infer_ok{false};
    for (auto &agent : map_target_lane_obstacles_) {
      ObstaclePredicatedInfo obj_infos;
      auto iter = map_gs_care_obstacles_.find(agent);
      if (iter != map_gs_care_obstacles_.end()) {
        infer_ok = InferObjTrajByLane(obj_infos, iter->second, false);
        if (infer_ok) {
          agent_node_target_lane_map_.insert(std::make_pair(agent, obj_infos));
        }
      }
    }
  };

  return true;
}

bool AgentNodeManager::HandleAllObjNormalInfo() {
  // Here the obj predinfo is handled!
  ids_sorted_target_lane_.reserve(agent_node_target_lane_map_.size());
  ids_sorted_origin_lane_.reserve(agent_node_origin_lane_map_.size());

  // set vector to store ids ,and then delete it
  std::vector<int64_t> origin_delete_ids;
  std::vector<int64_t> target_delete_ids;
  for (auto &agent : agent_node_origin_lane_map_) {
    ObstaclePredicatedInfo &obj_predicated_info = agent.second;
    auto iter = map_gs_care_obstacles_.find(agent.first);
    if (iter != map_gs_care_obstacles_.end()) {
      Point2D obj_frenet_p{-50., 0.};
      Point2D obj_cart_p{iter->second.x_center(), iter->second.y_center()};
      auto status = frenet_coord_->XYPointToSLPoint(obj_cart_p, obj_frenet_p);
      if (status == KDPathStatus::FALL) {
        obj_frenet_p.x = -100.;

      } else if (status == KDPathStatus::EXCEED) {
        obj_frenet_p.x = 200.;
      }
      obj_predicated_info.origin_x = iter->second.x_center();
      obj_predicated_info.origin_y = iter->second.y_center();
      obj_predicated_info.raw_vel = iter->second.velocity();
      obj_predicated_info.cur_s = obj_frenet_p.x;
      obj_predicated_info.cur_l = obj_frenet_p.y;
      obj_predicated_info.length = iter->second.length();
      obj_predicated_info.width = iter->second.width();
      ids_sorted_origin_lane_.emplace_back(agent.first);
    } else {
      origin_delete_ids.emplace_back(agent.first);
    }
  }

  for (auto &agent : agent_node_target_lane_map_) {
    ObstaclePredicatedInfo &obj_predicated_info = agent.second;
    auto iter = map_gs_care_obstacles_.find(agent.first);
    if (iter != map_gs_care_obstacles_.end()) {
      Point2D obj_frenet_p{-50., 0.};
      Point2D obj_cart_p{iter->second.x_center(), iter->second.y_center()};
      auto status = frenet_coord_->XYPointToSLPoint(obj_cart_p, obj_frenet_p);
      if (status == KDPathStatus::FALL) {
        obj_frenet_p.x = -100.;

      } else if (status == KDPathStatus::EXCEED) {
        obj_frenet_p.x = 200.;
      }

      obj_predicated_info.origin_x = iter->second.x_center();
      obj_predicated_info.origin_y = iter->second.y_center();
      obj_predicated_info.raw_vel = iter->second.velocity();
      obj_predicated_info.cur_s = obj_frenet_p.x;
      obj_predicated_info.cur_l = obj_frenet_p.y;
      obj_predicated_info.length = iter->second.length();
      obj_predicated_info.width = iter->second.width();
      ids_sorted_target_lane_.emplace_back(agent.first);
    } else {
      target_delete_ids.emplace_back(agent.first);
    }
  }

  for (auto id : origin_delete_ids) {
    agent_node_origin_lane_map_.erase(id);
  }

  for (auto id : target_delete_ids) {
    agent_node_target_lane_map_.erase(id);
  }

  return true;
}

void AgentNodeManager::SortNodeByS() {
  std::sort(ids_sorted_origin_lane_.begin(), ids_sorted_origin_lane_.end(),
            [&](int64_t a, int64_t b) {
              return agent_node_origin_lane_map_.at(a).cur_s <
                     agent_node_origin_lane_map_.at(b).cur_s;
            });

  std::sort(ids_sorted_target_lane_.begin(), ids_sorted_target_lane_.end(),
            [&](int64_t a, int64_t b) {
              return agent_node_target_lane_map_.at(a).cur_s <
                     agent_node_target_lane_map_.at(b).cur_s;
            });
  return;
}

bool AgentNodeManager::InferAllObjPredInfoByKDPath() { return true; };

bool AgentNodeManager::InferAllObjNormalInfoByKDPath() {
  // Recheck unreasonable objs
  for (auto &obj : map_origin_lane_obstacles_) {  // finally, need to check
                                                  // relative x,y to check
    auto iter = map_gs_care_obstacles_.find(obj);
    if (iter != map_gs_care_obstacles_.end()) {
      if (iter->second.y_relative_center() - iter->second.width() * 0.5 >
              1.75 + LaneCheckBuffer ||
          iter->second.y_relative_center() + iter->second.width() * 0.5 <
              -1.75 + LaneCheckBuffer) {
        map_origin_lane_obstacles_.erase(
            std::remove(map_origin_lane_obstacles_.begin(),
                        map_origin_lane_obstacles_.end(), obj),
            map_origin_lane_obstacles_.end());
      }
    }
  }
  if (target_state_ == 1) {
    for (auto &obj : map_target_lane_obstacles_) {  // finally, need to check
                                                    // relative x,y to check
      auto iter = map_gs_care_obstacles_.find(obj);
      if (iter != map_gs_care_obstacles_.end()) {
        if (iter->second.y_relative_center() - iter->second.width() * 0.5 >
                5.25 + LaneCheckBuffer ||
            iter->second.y_relative_center() + iter->second.width() * 0.5 <
                1.75 - LaneCheckBuffer) {
          map_target_lane_obstacles_.erase(
              std::remove(map_target_lane_obstacles_.begin(),
                          map_target_lane_obstacles_.end(), obj),
              map_target_lane_obstacles_.end());
        }
      }
    }
  } else if (target_state_ == 2) {
    for (auto &obj : map_target_lane_obstacles_) {  // finally, need to check
                                                    // relative x,y to check
      auto iter = map_gs_care_obstacles_.find(obj);
      if (iter != map_gs_care_obstacles_.end()) {
        if (iter->second.y_relative_center() - iter->second.width() * 0.5 >
                -1.75 + LaneCheckBuffer ||
            iter->second.y_relative_center() + iter->second.width() * 0.5 <
                -5.75 - LaneCheckBuffer) {
          map_target_lane_obstacles_.erase(
              std::remove(map_target_lane_obstacles_.begin(),
                          map_target_lane_obstacles_.end(), obj),
              map_target_lane_obstacles_.end());
        }
      }
    }
  }

  std::vector<int64_t> origin_delete_ids;
  std::vector<int64_t> target_delete_ids;

  for (auto &agent_id : map_target_lane_obstacles_) {
    ObstaclePredicatedInfo obj_predicated_info;
    auto iter = map_gs_care_obstacles_.find(agent_id);
    if (iter != map_gs_care_obstacles_.end()) {
      Point2D obj_frenet_p{-50.0, 0.};
      Point2D obj_cart_p{iter->second.x_center(), iter->second.y_center()};
      auto status = frenet_coord_->XYPointToSLPoint(obj_cart_p, obj_frenet_p);
      if (status == KDPathStatus::FALL) {
        obj_frenet_p.x = -100.0;
        obj_frenet_p.y = 0.0;
      } else if (status == KDPathStatus::EXCEED) {
        obj_frenet_p.x = 200.0;
        obj_frenet_p.y = 0.0;
      }

      obj_predicated_info.origin_x = iter->second.x_center();
      obj_predicated_info.origin_y = iter->second.y_center();
      obj_predicated_info.raw_vel = iter->second.velocity();
      obj_predicated_info.cur_s = obj_frenet_p.x;
      obj_predicated_info.cur_l = obj_frenet_p.y;
      obj_predicated_info.length = iter->second.length();
      obj_predicated_info.width = iter->second.width();
      obj_predicated_info.origin_heading_angle = iter->second.heading_angle();

      bool infer_ok =
          InferObjTrajByKDPath(obj_predicated_info, iter->second, false);
      if (infer_ok) {
        agent_node_origin_lane_map_.insert(
            std::make_pair(agent_id, obj_predicated_info));
      }
      ids_sorted_origin_lane_.emplace_back(agent_id);
    }
  };

  for (auto &agent_id : map_target_lane_obstacles_) {
    ObstaclePredicatedInfo obj_predicated_info;
    auto iter = map_gs_care_obstacles_.find(agent_id);
    if (iter != map_gs_care_obstacles_.end()) {
      Point2D obj_frenet_p{-50., 0.};
      Point2D obj_cart_p{iter->second.x_center(), iter->second.y_center()};
      auto status = frenet_coord_->XYPointToSLPoint(obj_cart_p, obj_frenet_p);
      if (status == KDPathStatus::FALL) {
        obj_frenet_p.x = -100.0;
        obj_frenet_p.y = 0.0;
      } else if (status == KDPathStatus::EXCEED) {
        obj_frenet_p.x = 200.0;
        obj_frenet_p.y = 0.0;
      }

      obj_predicated_info.origin_x = iter->second.x_center();
      obj_predicated_info.origin_y = iter->second.y_center();
      obj_predicated_info.raw_vel = iter->second.velocity();
      obj_predicated_info.origin_heading_angle = iter->second.heading_angle();
      obj_predicated_info.cur_s = obj_frenet_p.x;
      obj_predicated_info.cur_l = obj_frenet_p.y;
      obj_predicated_info.length = iter->second.length();
      obj_predicated_info.width = iter->second.width();
      ids_sorted_target_lane_.emplace_back(agent_id);

      auto iter = map_gs_care_obstacles_.find(agent_id);
      if (iter != map_gs_care_obstacles_.end()) {
        bool infer_ok =
            InferObjTrajByKDPath(obj_predicated_info, iter->second, true);
        if (infer_ok) {
          agent_node_target_lane_map_.insert(
              std::make_pair(agent_id, obj_predicated_info));
        }
        ids_sorted_target_lane_.emplace_back(agent_id);
      }
    }
  }
  return true;
}

// pybind debug
bool AgentNodeManager::Update() {
  // if (!HandleAllObjPredInfo()) {
  // }

  // if (!HandleAllObjNormalInfo()) {
  // }

  InferAllObjNormalInfoByKDPath();
  SortNodeByS();
  return true;
}

void AgentNodeManager::RefineObjInitState(const int64_t &obj_id,
                                          const double &obj_add_vel,
                                          const double &obj_add_s) {
  auto iter = map_gs_care_obstacles_.find(obj_id);
  ObstaclePredicatedInfo obj_infos;
  if (iter != map_gs_care_obstacles_.end()) {
    iter->second.set_v(iter->second.velocity() + obj_add_vel);
    InferObjTrajByLane(obj_infos, iter->second, 1);  // default target lane obj
    // update frenet

    auto node_iter = agent_node_target_lane_map_.find(obj_id);
    if (node_iter != agent_node_target_lane_map_.end()) {
      Point2D refine_update_pos;
      if (!frenet_coord_->XYToSL(
              Point2D{iter->second.x_center(), iter->second.y_center()},
              refine_update_pos)) {
        refine_update_pos.x = node_iter->second.cur_s;
        refine_update_pos.y = node_iter->second.cur_l;
        ILOG_ERROR << "Refine obj state cart -> frenet false";
      }
      obj_infos.cur_s = refine_update_pos.x;
      obj_infos.cur_l = refine_update_pos.y;
      obj_infos.origin_x = refine_update_pos.x;
      obj_infos.origin_y = refine_update_pos.y;
      obj_infos.raw_vel = iter->second.velocity();  // updated done!
      obj_infos.length = iter->second.length();
      obj_infos.width = iter->second.width();
    }
  }

  agent_node_target_lane_map_.erase(obj_id);
  agent_node_target_lane_map_.insert(std::make_pair(obj_id, obj_infos));
  // map_gs_care_obstacles_.erase(obj_id);
  // map_gs_care_obstacles_.insert(std::make_pair(obj_id, tmp_obstacle));
  return;
}
}
