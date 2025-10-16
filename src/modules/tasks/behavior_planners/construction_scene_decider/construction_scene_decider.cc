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
#include "src/framework/session.h"
#include "utils.h"
#include "utils/hysteresis_decision.h"
#include "utils/kd_path.h"
#include "vehicle_config_context.h"
#include "virtual_lane_manager.h"
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
constexpr double kCareLongDistance = 130;

}  // namespace

using namespace planning_math;
using namespace pnc::spline;

ConstructionSceneDecider::ConstructionSceneDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<ConstructionSceneDeciderConfig>();
  name_ = "ConstructionSceneDecider";
}

bool ConstructionSceneDecider::InitInfo() {
  lateral_obstacle_ =
      session_->environmental_model().get_lateral_obstacle();
  lane_tracks_manager_ =
      session_->environmental_model().get_lane_tracks_manager();
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr =
        session_->mutable_environmental_model()->get_virtual_lane_manager();
  lane_change_lane_mgr_ =
      std::make_shared<LaneChangeLaneManager>(virtual_lane_mgr, session_);
  const int current_lane_virtual_id =
      virtual_lane_mgr->current_lane_virtual_id();
  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
  } else {
    origin_lane_virtual_id_ = current_lane_virtual_id;
  }
  is_construction_agent_cluster_success_ = false;
  construction_agent_points_.clear();
  construction_agent_cluster_attribute_set_.clear();
  construction_agent_cluster_size_.clear();
  construction_agent_cluster_.clear();
  return true;
}

bool ConstructionSceneDecider::Execute() {
  ILOG_INFO << "=======ConstructionSceneDecider=======";
  auto start_time = IflyTime::Now_ms();
  if (!InitInfo()) {
    return false;
  }

  // 施工障碍物聚类
  UpdateConstructionAgentClusters();

  UpdateDriveArea();


  // 依据聚类结果判定施工区域场景
  IdentifyConstructionScene();

  SaveLatDebugInfo();

  auto end_time = IflyTime::Now_ms();
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
  const auto origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(origin_lane_virtual_id_, false);
  const auto base_lane =
      virtual_lane_mgr->get_lane_with_virtual_id(origin_lane_virtual_id_);

  const auto& frenet_obstacles_map = origin_refline->get_obstacles_map();
  base_frenet_coord_ = origin_refline->get_frenet_coord();
  Point2D ego_cart_point{planning_init_point.lat_init_state.x(),
                         planning_init_point.lat_init_state.y()};
  // Point2D ego_frenet_point;
  // if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
  //   // 有待商榷
  //   return;
  // }

  const double ego_rear_edge = vehicle_param.rear_edge_to_rear_axle;
  const double ego_length = vehicle_param.length;
  double eps_x = vehicle_param.length * kLongClusterCoeff;
  double eps_y = vehicle_param.width + kLatClusterThre;
  int construction_agent_nums_of_front_objects = 0; // 前方锥桶数量
  int minPts = 1; // 最小聚类簇个数
  const auto& tracks_map = lateral_obstacle_->tracks_map();
  const auto& front_obstacles_array = lateral_obstacle_->front_tracks();
  const auto& side_obstacles_array = lateral_obstacle_->side_tracks();
  std::vector<std::shared_ptr<FrenetObstacle>> combined_obstacles;
  combined_obstacles.reserve(front_obstacles_array.size() +
                             side_obstacles_array.size());
  combined_obstacles.insert(combined_obstacles.end(),
                            front_obstacles_array.begin(),
                            front_obstacles_array.end());
  combined_obstacles.insert(combined_obstacles.end(),
                            side_obstacles_array.begin(),
                            side_obstacles_array.end());
  Transform2d ego_base;
  auto base_pose = Pose2D(planning_init_point.lat_init_state.x(),
                          planning_init_point.lat_init_state.y(),
                          planning_init_point.lat_init_state.theta());
  ego_base.SetBasePose(base_pose);
  for (const auto obstacle : combined_obstacles) {
    int obstacle_id = obstacle->id();
    auto vehicle_iter = tracks_map.find(obstacle_id);
    if (vehicle_iter != tracks_map.end()) {
      if (obstacle_id == kInvalidAgentId) {
        continue;
      }
      if (IsConstructionAgent(vehicle_iter->second->type())) {
        // 目前仅针对锥桶一个类型，后续扩展这里可以用子函数筛选
        // IsConstructionAgent 是否为施工类型障碍物
        if (vehicle_iter->second->d_s_rel() < - ego_length ||
            vehicle_iter->second->d_s_rel() > kCareLongDistance) {
          continue;
        }
        construction_agent_nums_of_front_objects++;
        Point2D obs_cart_point{0.0, 0.0};
        Point2D obs_frenet_point;
        obs_cart_point.x = vehicle_iter->second->obstacle()->x_center();
        obs_cart_point.y = vehicle_iter->second->obstacle()->y_center();
        if (frenet_obstacles_map.find(obstacle_id) ==
                frenet_obstacles_map.end() ||
            !frenet_obstacles_map.at(obstacle_id)->b_frenet_valid()) {
          continue;
        }
        double construction_agent_s = vehicle_iter->second->frenet_s();
        double construction_agent_l = vehicle_iter->second->frenet_l();
        Pose2D obs_car_point;
        ego_base.GlobalPointToULFLocal(
            &obs_car_point, Pose2D(obs_cart_point.x, obs_cart_point.y, 0));
        double dist_to_left_boundary = kDefaultLaneWidth;
        GetOriginLaneWidthByConstructionAgent(base_lane, construction_agent_s, construction_agent_l, true,
                                              &dist_to_left_boundary);
        double dist_to_right_boundary = kDefaultLaneWidth;
        GetOriginLaneWidthByConstructionAgent(base_lane, construction_agent_s, construction_agent_l, false,
                                              &dist_to_right_boundary);
        auto point = ConstructionAgentPoint(vehicle_iter->first, obs_cart_point.x,
                               obs_cart_point.y, obs_car_point.x, obs_car_point.y,
                               construction_agent_s, construction_agent_l, dist_to_left_boundary, dist_to_right_boundary);
        construction_agent_points_.push_back(point);
      }
    } else {
      continue;
    }
  }
  JSON_DEBUG_VALUE("construction_agent_nums_of_front_objects", construction_agent_nums_of_front_objects);

  if (!construction_agent_points_.empty()) {
    // 检测类型为cone的障碍物赋予cluster属性
    // 按照自车系下的坐标点聚类
    DbScan(construction_agent_points_, eps_x, eps_y, minPts);
  } else {
    return;
  }

  for (const auto& p : construction_agent_points_) {
    // 构建相同cluster属性所包含施工障碍物的map
    construction_agent_cluster_attribute_set_[p.cluster].points.push_back(p);
  }

  is_construction_agent_cluster_success_ = true;

  for (auto& [cluster_id, cluster_area] :
       construction_agent_cluster_attribute_set_) {
    std::sort(
        cluster_area.points.begin(), cluster_area.points.end(),
        [](const ConstructionAgentPoint& a, const ConstructionAgentPoint& b) {
          return a.car_x < b.car_x;
        });
  }
}

void ConstructionSceneDecider::DbScan(ConstructionAgentPoints& cone_points,
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
  return std::abs(a.car_x - b.car_x) < eps_x &&
         std::abs(a.car_y - b.car_y) < eps_y;
}

void ConstructionSceneDecider::ExpandCluster(
    ConstructionAgentPoints& cone_points, int index, int c, double eps_x,
    double eps_y, int minPts) {
  std::vector<int> neighborPts;

  for (size_t i = 0; i < cone_points.size(); ++i) {
    if (ConstructionAgentDistance(cone_points[index], cone_points[i], eps_x,
                                  eps_y)) {
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
    const ConstructionAgentPoints& points, RequestType direction) {
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
  return (type == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_CONE ||
          type == iflyauto::ObjectType::OBJECT_TYPE_WATER_SAFETY_BARRIER ||
          type == iflyauto::ObjectType::OBJECT_TYPE_CTASH_BARREL);
}

void ConstructionSceneDecider::GetOriginLaneWidthByConstructionAgent(
    const std::shared_ptr<VirtualLane> base_lane, const double construction_agent_s,
    const double construction_agent_l, bool is_left, double* dist) {
  if (base_lane == nullptr) {
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
    origin_lane_width = QueryLaneWidth(construction_agent_s, origin_lane_s_width_);
  }
  if (is_left) {
    double left_width = 0.5 * origin_lane_width;
    if (construction_agent_l > left_width) {
      *dist = kDefaultLaneWidth;
    } else {
      *dist = left_width - construction_agent_l;
    }
  } else {
    double right_width = 0.5 * origin_lane_width;
    if (construction_agent_l < -right_width) {
      *dist = kDefaultLaneWidth;
    } else {
      *dist = right_width + construction_agent_l;
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

void ConstructionSceneDecider::IdentifyConstructionScene() {

  // 判定是否能够触发变道

}

void ConstructionSceneDecider::UpdateDriveArea() {
  if (construction_agent_cluster_attribute_set_.size() <= 0) {
    return;
  }

  const std::vector<std::shared_ptr<VirtualLane>> lanes =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_virtual_lanes();

  std::map<int, std::map<int, std::vector<int>>> results;
  for (const auto& lane : lanes) {
    if (lane == nullptr) {
      continue;
    }
    const auto& lane_frenet_coord = lane->get_lane_frenet_coord();
    if (lane_frenet_coord == nullptr) {
      continue;
    }

    ILOG_DEBUG << "lane id:" << lane->get_virtual_id();
    const auto& lane_points = lane->lane_points();
    std::vector<Point2d> ref_points;
    ref_points.reserve(lane_points.size());
    for (const auto& lane_point : lane_points) {
      ref_points.emplace_back(std::move(
          Point2d(lane_point.local_point.x, lane_point.local_point.y)));
    }

    for (const auto construction_agent_cluster_iter :
         construction_agent_cluster_attribute_set_) {
      if (construction_agent_cluster_iter.second.points.size() <= 1) {
        continue;
      }

      // Attention!!! 存在重复计算
      std::vector<Point2d> cone_points;
      cone_points.reserve(construction_agent_cluster_iter.second.points.size());
      for (const auto& agent_clusters :
           construction_agent_cluster_iter.second.points) {
        cone_points.emplace_back(
            std::move(Point2d(agent_clusters.x, agent_clusters.y)));
      }

      const auto result =
          CalIntersectionRefAndCone(lane_frenet_coord, ref_points, cone_points);

      results[construction_agent_cluster_iter.first][result.second]
          .emplace_back(lane->get_virtual_id());
      ILOG_DEBUG << "result: "
                 << "lane id:" << lane->get_virtual_id() << "   ***  "
                 << "cone cluster id" << construction_agent_cluster_iter.first
                 << "intersection :" << result.second;
    }
  }

  UpdateResult(results);
}

std::pair<bool, int> ConstructionSceneDecider::CalIntersectionRefAndCone(
    const std::shared_ptr<planning_math::KDPath> lane_frenet_coord,
    const std::vector<Point2d>& ref_points,
    const std::vector<Point2d>& cone_points) {
  if (!lane_frenet_coord) {
    return {false, -1};
  }

  if (cone_points.size() < 1) {
    return {false, -1};
  }

  if (construction_scene_utils::polylinesIntersect(ref_points, cone_points)) {
    return {true, 0};
  }

  Point2D frenet_point;
  if (!lane_frenet_coord->XYToSL(Point2D(cone_points[0].x, cone_points[0].y),
                                 frenet_point)) {
    return {false, -1};
  }

  if (frenet_point.y < 0) {
    return {false, 1};
  } else if (frenet_point.y > 0) {
    return {false, 2};
  } else {
    return {false, 0};
  }
}

void ConstructionSceneDecider::UpdateResult(
    const std::map<int, std::map<int, std::vector<int>>>& results) {
  for (auto result : results) {
    if (result.second.count(1) && result.second.count(2)) {
      construction_agent_cluster_attribute_set_[result.first].direction =
          Direction::UNSURE;
      std::cout << "abnormal ref" << std::endl;
    } else if (result.second.count(1)) {
      construction_agent_cluster_attribute_set_[result.first].direction =
          Direction::LEFT;
    } else if (result.second.count(2)) {
      construction_agent_cluster_attribute_set_[result.first].direction =
          Direction::RIGHT;
    }

    if (result.second.count(-1)) {
      construction_agent_cluster_attribute_set_[result.first].direction =
          Direction::UNSURE;
      std::cout << "colinear_or_facing ref" << std::endl;
    }
  }
}

void ConstructionSceneDecider::SaveLatDebugInfo() {
  // 保存聚类的障碍物信息
  JSON_DEBUG_VALUE("is_construction_agent_cluster_success",
                    is_construction_agent_cluster_success_);
  if (is_construction_agent_cluster_success_) {
    std::vector<double> construction_agent_cluster_attribute_ids;
    std::vector<double> construction_agent_clusters;
    std::vector<double> construction_agent_clusters_length;
    std::vector<double> construction_agent_clusters_driection;
    for (const auto& cluster_attribute_iter :
        construction_agent_cluster_attribute_set_) {
      const ConstructionAgentPoints& points =
          cluster_attribute_iter.second.points;
      for (const auto& p : points) {
        construction_agent_cluster_attribute_ids.emplace_back(p.id);
      }
      construction_agent_clusters.emplace_back(cluster_attribute_iter.first);
      construction_agent_clusters_length.emplace_back(points.size());
      construction_agent_clusters_driection.emplace_back(static_cast<uint32_t>(
          cluster_attribute_iter.second.direction));
    }
    JSON_DEBUG_VECTOR("construction_agent_clusters", construction_agent_clusters,
                      0);
    JSON_DEBUG_VECTOR("construction_agent_clusters_length",
                      construction_agent_clusters_length, 0);
    JSON_DEBUG_VECTOR("construction_agent_cluster_attribute_ids",
                      construction_agent_cluster_attribute_ids, 0);
    JSON_DEBUG_VECTOR("construction_agent_clusters_driection",
                      construction_agent_clusters_driection, 0);
  }
}

}  // namespace planning
