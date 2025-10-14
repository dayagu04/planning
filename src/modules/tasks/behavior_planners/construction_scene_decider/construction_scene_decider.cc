#include "construction_scene_decider.h"

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "agent/agent.h"
#include "agent_node_manager.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "frenet_ego_state.h"
#include "log.h"
#include "utils/hysteresis_decision.h"
#include "utils/kd_path.h"
#include "vehicle_config_context.h"
#include "virtual_lane_manager.h"
#include "agent/agent.h"
#include "utils/hysteresis_decision.h"
#include "src/framework/session.h"

namespace planning {

namespace {
constexpr double kLongClusterCoeff = 3.2;
constexpr double kLatClusterThre = 0.6;
constexpr double kLatPassThre = 0.75;
constexpr double kLatPassThreBuffer = 0.35;
constexpr uint32_t kConeAlcCountThre = 3;
constexpr int kConeAlcCountLowerThre = 0;
constexpr double kLongClusterTimeGap = 4.0;
constexpr double kDefaultLaneWidth = 3.75;
constexpr double kMinDefaultLaneWidth = 2.65;
constexpr uint32_t kConeDirecSize = 5;
constexpr double kConeDirecThre = 0.5;
constexpr double kConeSlopeThre = 1;
constexpr int kInvalidAgentId = -1;

}  // namespace

using namespace planning_math;
using namespace pnc::spline;

ConstructionSceneDecider::ConstructionSceneDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<ConstructionSceneDeciderConfig>();
  name_ = "ConstructionSceneDecider";
}

bool ConstructionSceneDecider::InitInfo() {
  return true;
}

bool ConstructionSceneDecider::Execute() {
  ILOG_INFO << "=======ConstructionSceneDecider=======";
  auto start_time = IflyTime::Now_ms();
  if (!InitInfo()) {
    return false;
  };
  auto end_time = IflyTime::Now_ms();

  // 施工障碍物聚类
  UpdateConstructionAgentClusters();



  JSON_DEBUG_VALUE("ConstructionSceneDeciderCostTime", end_time - start_time);
  return true;
}


void ConstructionSceneDecider::UpdateConstructionAgentClusters() {
  // 施工障碍物聚类
  // 自车状态
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto planning_init_point = ego_state->planning_init_point();
  double ego_fx = std::cos(ego_state->ego_pose_raw().theta);
  double ego_fy = std::sin(ego_state->ego_pose_raw().theta);

  const auto virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const int current_lane_virtual_id =
      virtual_lane_mgr->current_lane_virtual_id();
  const auto origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(current_lane_virtual_id, false);
  const auto base_lane =
      virtual_lane_mgr->get_lane_with_virtual_id(current_lane_virtual_id);

  const auto &frenet_obstacles_map = origin_refline->get_obstacles_map();
  base_frenet_coord_ = origin_refline->get_frenet_coord();
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point.lat_init_state.x(),
                         planning_init_point.lat_init_state.y()};
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    // 有待商榷
    ILOG_DEBUG << "fail to get ego position on base lane";
    is_construction_agent_lane_change_situation_ = false;
    return;
  }

  const double ego_rear_edge = vehicle_param.rear_edge_to_rear_axle;
  double eps_x = vehicle_param.length * kLongClusterCoeff;
  double eps_y = vehicle_param.width + kLatClusterThre;
  int cone_nums_of_front_objects = 0; // 前方锥桶数量
  int minPts = 1; // 最小聚类簇个数
  construction_agent_points_.clear();
  construction_agent_cluster_attribute_set_.clear();

  lateral_obstacle_ = session_->environmental_model().get_lateral_obstacle();
  const auto& tracks_map = lateral_obstacle_->tracks_map();
  const auto& front_obstacles_array =
      lateral_obstacle_->front_tracks();
  Transform2d ego_base;
  auto base_pose = Pose2D(planning_init_point.lat_init_state.x(), planning_init_point.lat_init_state.y(),
                          planning_init_point.lat_init_state.theta());
  ego_base.SetBasePose(base_pose);
  for (const auto front_obstacle : front_obstacles_array) {
    int obstacle_id = front_obstacle->id();
    auto front_vehicle_iter = tracks_map.find(obstacle_id);
    if (front_vehicle_iter != tracks_map.end()) {
      if (obstacle_id == kInvalidAgentId) {
        continue;
      }
      if (IsConstructionAgent(front_vehicle_iter->second->type())) {
        // 目前仅针对锥桶一个类型，后续扩展这里可以用子函数筛选
        // IsConstructionAgent 是否为施工类型障碍物
        if (front_vehicle_iter->second->d_s_rel() < -ego_rear_edge ||
            front_vehicle_iter->second->d_s_rel() >
                base_frenet_coord_->Length() - ego_frenet_point.x) {
          continue;
        }
        cone_nums_of_front_objects++;
        Point2D obs_cart_point{0.0, 0.0};
        Point2D obs_frenet_point;
        obs_cart_point.x = front_vehicle_iter->second->obstacle()->x_center();
        obs_cart_point.y = front_vehicle_iter->second->obstacle()->y_center();
        if (frenet_obstacles_map.find(obstacle_id) == frenet_obstacles_map.end() ||
            !frenet_obstacles_map.at(obstacle_id)->b_frenet_valid()) {
          continue;
        }
        double cone_s = front_vehicle_iter->second->frenet_s();
        double cone_l = front_vehicle_iter->second->frenet_l();
        Pose2D obs_car_point;
        ego_base.GlobalPointToULFLocal(&obs_car_point,
                                       Pose2D(obs_cart_point.x, obs_cart_point.y, 0));
        double dist_to_left_boundary = kDefaultLaneWidth;
        GetOriginLaneWidthByConstructionAgent(base_lane, cone_s, cone_l, true,
                                              &dist_to_left_boundary);
        double dist_to_right_boundary = kDefaultLaneWidth;
        GetOriginLaneWidthByConstructionAgent(base_lane, cone_s, cone_l, false,
                                              &dist_to_right_boundary);
        auto point = ConstructionAgentPoint(front_vehicle_iter->first, obs_cart_point.x,
                               obs_cart_point.y, obs_car_point.x, obs_car_point.y,
                               cone_s, cone_l, dist_to_left_boundary, dist_to_right_boundary);
        construction_agent_points_.push_back(point);
      }
    } else {
      continue;
    }
  }
  JSON_DEBUG_VALUE("cone_nums_of_front_objects", cone_nums_of_front_objects);

  if (construction_agent_points_.empty()) {
    // if no cones found, counter--
    ILOG_DEBUG << "no construction agent found!!!";
    construction_agent_alc_trigger_counter_ =
        std::max(construction_agent_alc_trigger_counter_ - 1, kConeAlcCountLowerThre);
    is_construction_agent_lane_change_situation_ = false;
    return;
  } else {
    // 检测类型为cone的障碍物赋予cluster属性
    // 按照自车系下的坐标点聚类
    DbScan(construction_agent_points_, eps_x, eps_y, minPts);
  }

  construction_agent_cluster_size_.clear();
  construction_agent_cluster_.clear();
  for (const auto& p : construction_agent_points_) {
    // 构建相同cluster属性所包含cones的map
    construction_agent_cluster_attribute_set_[p.cluster].push_back(p);
  }

  // 保存聚类的障碍物信息
  std::vector<double> construction_agent_cluster_attribute_ids;
  std::vector<double> construction_agent_clusters;
  std::vector<double> construction_agent_clusters_length;
  for (const auto& cluster_attribute_iter : construction_agent_cluster_attribute_set_) {
    const std::vector<ConstructionAgentPoint>& points = cluster_attribute_iter.second;
    for (const auto& p : points) {
      construction_agent_cluster_attribute_ids.emplace_back(p.id);
    }
    construction_agent_clusters.emplace_back(cluster_attribute_iter.first);
    construction_agent_clusters_length.emplace_back(points.size());
  }
  JSON_DEBUG_VECTOR("construction_agent_clusters", construction_agent_clusters, 0);
  JSON_DEBUG_VECTOR("construction_agent_clusters_length", construction_agent_clusters_length, 0);
  JSON_DEBUG_VECTOR("construction_agent_cluster_attribute_ids", construction_agent_cluster_attribute_ids, 0);




  bool did_break = false;
  for (const auto& cluster_attribute_iter : construction_agent_cluster_attribute_set_) {
    int cluster = cluster_attribute_iter.first;
    const std::vector<ConstructionAgentPoint>& points = cluster_attribute_iter.second;
    double min_left_l, min_right_l, pass_threshold_left, pass_threshold_right;
    min_left_l = CalcClusterToBoundaryDist(points, LEFT_CHANGE);
    min_right_l = CalcClusterToBoundaryDist(points, RIGHT_CHANGE);

    pass_threshold_left =
        vehicle_param.width + kLatPassThre + kLatPassThreBuffer;
    pass_threshold_right =
        vehicle_param.width + kLatPassThre + kLatPassThreBuffer;
    ILOG_DEBUG << "min_left_l is:" << min_left_l
               << ", min_right_l is: is:" << min_right_l
               << ", pass_threshold_left is:" << pass_threshold_left
               << ", pass_threshold_right is:" << pass_threshold_right;
    // judge if to trigger cone lc
    if (min_left_l < pass_threshold_left &&
        min_right_l < pass_threshold_right) {
      construction_agent_alc_trigger_counter_++;
      ILOG_DEBUG << "trigger_counter is " << construction_agent_alc_trigger_counter_ << ", cluster is " << cluster;
      if (construction_agent_alc_trigger_counter_ >= kConeAlcCountThre) {
        is_construction_agent_lane_change_situation_ = true;
        return;
      }
      did_break = true;
      break;
    }
  }

  // if all clusters is far away from cernter line, counter--
  if (!did_break) {
    construction_agent_alc_trigger_counter_ =
        std::max(construction_agent_alc_trigger_counter_ - 1, kConeAlcCountLowerThre);
    ILOG_DEBUG << "trigger_counter is " << construction_agent_alc_trigger_counter_;
    if (construction_agent_alc_trigger_counter_ < kConeAlcCountThre) {
      is_construction_agent_lane_change_situation_ = false;
    }
  }
  // if all cone l is larger than threshold, then no need to lane change
  return;
}

void ConstructionSceneDecider::DbScan(
    std::vector<ConstructionAgentPoint>& cone_points,
    double eps_x, double eps_y, int minPts) {

  int c = 0;  // cluster index
  for (size_t index = 0; index < cone_points.size(); ++index) {
    if (!cone_points[index].visited) {
      // 根据锥桶间的距离聚类
      ExpandCluster(cone_points, index, c, eps_x, eps_y, minPts);
      c++;
    }
  }
}

bool ConstructionSceneDecider::ConstructionAgentDistance(
    const ConstructionAgentPoint& a, const ConstructionAgentPoint& b,
    double eps_x, double eps_y) {
  return std::abs(a.car_x - b.car_x) < eps_x && std::abs(a.car_y - b.car_y) < eps_y;
}

void ConstructionSceneDecider::ExpandCluster(
    std::vector<ConstructionAgentPoint>& cone_points, int index,
    int c, double eps_x, double eps_y, int minPts) {

  std::vector<int> neighborPts;

  for (size_t i = 0; i < cone_points.size(); ++i) {
    if (ConstructionAgentDistance(cone_points[index], cone_points[i], eps_x, eps_y)) {
      neighborPts.push_back(i);
    }
  }

  if (neighborPts.size() < minPts) {
    // The point is noise
    cone_points[index].cluster = -1;
    return;
  }

  // Assign the cluster to initial point
  cone_points[index].visited = true;
  cone_points[index].cluster = c;

  // Check all neighbours for being part of the cluster
  for (auto& neighborPt : neighborPts) {
    ConstructionAgentPoint& p_neighbor = cone_points[neighborPt];
    if (!p_neighbor.visited) {
      // Recursively expand the cluster
      ExpandCluster(cone_points, neighborPt, c, eps_x, eps_y, minPts);
    }
  }
}

double ConstructionSceneDecider::CalcClusterToBoundaryDist(
    const std::vector<ConstructionAgentPoint>& points, RequestType direction) {
  double left_l = std::abs(points[0].left_dist);
  double right_l = std::abs(points[0].right_dist);
  for (const auto& p : points) {
    left_l = std::min(std::abs(p.left_dist), left_l);
    right_l = std::min(std::abs(p.right_dist), right_l);
  }
  if (direction == LEFT_CHANGE) {
    return left_l;
  } else if (direction == RIGHT_CHANGE) {
    return right_l;
  } else {
    return std::max(left_l, right_l);
  }
}

bool ConstructionSceneDecider::IsConstructionAgent(iflyauto::ObjectType type) {
  // 目前仅针对锥桶一个类型，后续扩展可以在这里添加
  return (type == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_CONE);
}

void ConstructionSceneDecider::GetOriginLaneWidthByConstructionAgent(
    const std::shared_ptr<VirtualLane> base_lane, const double cone_s,
    const double cone_l, bool is_left, double* dist) {
  if (base_lane == nullptr) {
    is_construction_agent_lane_change_situation_ = false;
    return;
  }
  const auto origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(base_lane->get_virtual_id(), false);
  double origin_lane_width = kDefaultLaneWidth;
  if (origin_refline != nullptr) {
    origin_lane_s_width_.clear();
    origin_lane_s_width_.reserve(origin_refline->get_points().size());
    for (auto i = 0; i < origin_refline->get_points().size(); i++) {
      const ReferencePathPoint& ref_path_point =
          origin_refline->get_points()[i];
      origin_lane_s_width_.emplace_back(std::make_pair(
          ref_path_point.path_point.s(), ref_path_point.lane_width));
    }
    origin_lane_width = QueryLaneWidth(cone_s, origin_lane_s_width_);
  }
  if (is_left) {
    double left_width = 0.5 * origin_lane_width;
    if (cone_l > left_width) {
      *dist = kDefaultLaneWidth;
    } else {
      *dist = left_width - cone_l;
    }
  } else {
    double right_width = 0.5 * origin_lane_width;
    if (cone_l < -right_width) {
      *dist = kDefaultLaneWidth;
    } else {
      *dist = right_width + cone_l;
    }
  }
}

double ConstructionSceneDecider::QueryLaneWidth(
    const double s0,
    const std::vector<std::pair<double, double>>& lane_s_width) {
  auto comp = [](const std::pair<double, double>& s_width, const double s) {
    return s_width.first < s;
  };
  double lane_width;
  const auto& first_pair_on_lane =
      std::lower_bound(lane_s_width.begin(), lane_s_width.end(), s0, comp);

  if (first_pair_on_lane == lane_s_width.begin()) {
    lane_width = lane_s_width.front().second;
  } else if (first_pair_on_lane == lane_s_width.end()) {
    lane_width = lane_s_width.back().second;
  } else {
    lane_width = planning_math::lerp(
        (first_pair_on_lane - 1)->second, (first_pair_on_lane - 1)->first,
        first_pair_on_lane->second, first_pair_on_lane->first, s0);
  }
  return std::fmax(lane_width, 2.8);
}

}

