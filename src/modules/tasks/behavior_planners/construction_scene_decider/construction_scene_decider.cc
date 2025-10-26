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
constexpr double kLongClusterCoeff = 5.0;
constexpr double kLatClusterThre = 0.3;
constexpr double kLatPassThre = 0.8;
constexpr double kLatPassThreBuffer = 0.35;
constexpr double kConeCrossingLaneLineBuffer = 0.15;
constexpr uint32_t kConeAlcCountThre = 4;
constexpr uint32_t kConeAlcMaxCountThre = 8;
constexpr uint32_t kConeAlcHystereticCount = 1;
constexpr int kConeAlcCountLowerThre = 0;
constexpr double kLongClusterTimeGap = 4.0;
constexpr double kDefaultLaneWidth = 4.5;
constexpr double kMinDefaultLaneWidth = 2.65;
constexpr uint32_t kConeDirecSize = 5;
constexpr double kConeDirecThre = 0.5;
constexpr double kConeSlopeThre = 1;
constexpr int kInvalidAgentId = -1;
constexpr double kCareLongDistance = 130;
constexpr double kConstructionBucketSpacingThreshold = 8; // 国标是5m
constexpr double kNumSatisfySonstructionAgent = 3;
constexpr double kLaneAvailableLatPassThre = 0.5; // 锥桶侵入车道的距离阈值
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
  auto& construction_scene_decider_output =
      session_->mutable_planning_context()
          ->mutable_construction_scene_decider_output();
  construction_scene_decider_output.Clear();
  lateral_obstacle_ = session_->environmental_model().get_lateral_obstacle();
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
  construction_agent_cluster_attribute_map_.clear();
  construction_agent_cluster_size_.clear();
  construction_agent_cluster_.clear();
  return true;
}

bool ConstructionSceneDecider::Execute() {
  ILOG_INFO << "=======ConstructionSceneDecider=======";
  auto start_time = IflyTime::Now_ms();
  if (!InitInfo()) {
    is_exist_construction_area_ = false;
    is_pass_construction_area_ = false;
    return false;
  }

  // 施工障碍物聚类
  UpdateConstructionAgentClusters();

  UpdateDriveArea();

  // 依据聚类结果判定施工区域场景
  IdentifyConstructionScene();

  GenerateConstructionSceneOutput();

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
  int construction_agent_nums_of_front_objects = 0;  // 前方锥桶数量
  int minPts = 1;                                    // 最小聚类簇个数
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
        if (vehicle_iter->second->d_s_rel() < -ego_length ||
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
        GetOriginLaneWidthByConstructionAgent(base_lane, construction_agent_s,
                                              construction_agent_l, true,
                                              &dist_to_left_boundary);
        double dist_to_right_boundary = kDefaultLaneWidth;
        GetOriginLaneWidthByConstructionAgent(base_lane, construction_agent_s,
                                              construction_agent_l, false,
                                              &dist_to_right_boundary);
        auto point = ConstructionAgentPoint(
            vehicle_iter->first, obs_cart_point.x, obs_cart_point.y,
            obs_car_point.x, obs_car_point.y, construction_agent_s,
            construction_agent_l, dist_to_left_boundary,
            dist_to_right_boundary);
        construction_agent_points_.push_back(point);
      }
    } else {
      continue;
    }
  }
  JSON_DEBUG_VALUE("construction_agent_nums_of_front_objects",
                   construction_agent_nums_of_front_objects);

  if (!construction_agent_points_.empty()) {
    // 检测类型为cone的障碍物赋予cluster属性
    // 按照自车系下的坐标点聚类
    DbScan(construction_agent_points_, eps_x, eps_y, minPts);
  } else {
    return;
  }

  for (const auto& p : construction_agent_points_) {
    // 构建相同cluster属性所包含施工障碍物的map
    construction_agent_cluster_attribute_map_[p.cluster].points.push_back(p);
  }

  is_construction_agent_cluster_success_ = true;

  for (auto & [ cluster_id, cluster_area ] :
       construction_agent_cluster_attribute_map_) {
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
    const std::shared_ptr<VirtualLane> base_lane,
    const double construction_agent_s, const double construction_agent_l,
    bool is_left, double* dist) {
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
    origin_lane_width =
        QueryLaneWidth(construction_agent_s, origin_lane_s_width_);
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
  // 判定是否存在施工区域
  IsExistConstructionArea();

  // 判定施工区域的倾入程度，并给出变道及其方向、走通行空间的标志位
  JudgeConstructionIntrusionLevel();
}

void ConstructionSceneDecider::IsExistConstructionArea() {
  // 判定是否存在施工区域以及是否正在经过施工区域
  // 计算纵向平均间距
  auto calc_avg_spacing = [](const std::vector<double>& xs) -> double {
    if (xs.size() < 2) return 100; // 无法计算
    double sum = 0.0;
    for (size_t i = 1; i < xs.size(); ++i) {
      sum += std::abs(xs[i] - xs[i - 1]);
    }
    return sum / (xs.size() - 1);
  };
  VehicleParam vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto ego_rear_axis_to_front_edge =
      vehicle_param.front_edge_to_rear_axle;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  bool is_valid_construction_area = false;
  // 遍历所有聚类的子集，如果簇所包含的施工区域障碍物较少，则非有效
  // 避免误识别施工区域
  bool is_pass_construction_area = false;
  // 是否正在经过施工区域
  // 通过自车系判断是否正在通过施工区域
  int construction_agents_low_num = 5;  // 需要结合实际调整
  // 国标要求3~5个锥桶
  double pass_buffer = kConstructionBucketSpacingThreshold;
  if (is_exist_construction_area_) {
    construction_agents_low_num = 3;
  }
  if (is_pass_construction_area_) {
    // 如果3s之内前方还是存在施工区域，则还是处于施工路段，避免自车先加速后加速
    pass_buffer = std::fmax(3 * pass_buffer, 3 * ego_state->ego_v());
  }
  int total_construction_agents = 0;
  int left_construction_agents = 0;
  int right_construction_agents = 0;
  double left_avg_spacing = 100;
  double right_avg_spacing = 100;
  std::vector<double> left_xs;
  std::vector<double> right_xs;
  left_xs.reserve(10);
  right_xs.reserve(10);
  for (const auto& [cluster_id, construction_agent_cluster_iter] :
       construction_agent_cluster_attribute_map_) {
    const auto& points = construction_agent_cluster_iter.points;
    if (points.empty()) {
      continue;
    }
    total_construction_agents += points.size();
    // 判断簇的左右方向
    for (const auto& p : points) {
      if (p.car_y >= 0.0) {
        left_construction_agents++;
        left_xs.push_back(p.car_x);
      } else {
        right_construction_agents++;
        right_xs.push_back(p.car_x);
      }
    }
    if (points.size() >= construction_agents_low_num) {
      is_valid_construction_area = true;
    }
    double min_x = points.front().car_x;
    // double max_x = points.back().car_x;
    // 如果自车位置在该簇范围内，则认为正在经过施工区域
    if (ego_rear_axis_to_front_edge + pass_buffer >= min_x) {
      is_pass_construction_area = true;
    }
    if (points.size() < 2) {
      continue;
    }
    // 计算相邻锥桶的纵向间距均值
    double total_spacing = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
      total_spacing += std::fabs(points[i].car_x - points[i - 1].car_x);
    }
    double avg_spacing = total_spacing / (points.size() - 1);
    if (avg_spacing <= kConstructionBucketSpacingThreshold) {
      is_valid_construction_area = true;
    }
  }

  // 按纵向排序
  std::sort(left_xs.begin(), left_xs.end());
  std::sort(right_xs.begin(), right_xs.end());
  left_avg_spacing = calc_avg_spacing(left_xs);
  right_avg_spacing = calc_avg_spacing(right_xs);
  if (total_construction_agents >= 2 * construction_agents_low_num ||
      left_construction_agents >= construction_agents_low_num ||
      right_construction_agents >= construction_agents_low_num ||
      (left_construction_agents > 2 &&
      left_avg_spacing <= kConstructionBucketSpacingThreshold) ||
      (right_construction_agents > 2 &&
      right_avg_spacing <= kConstructionBucketSpacingThreshold)) {
    // 避免锥桶摆的较开，未聚成一类
    is_valid_construction_area = true;
  }
  if (!is_valid_construction_area) {
    is_pass_construction_area = false;
  }

  is_exist_construction_area_ = is_valid_construction_area;
  is_pass_construction_area_ = is_pass_construction_area;
  JSON_DEBUG_VALUE("is_exist_construction_area", is_exist_construction_area_);
  JSON_DEBUG_VALUE("is_pass_construction_area", is_pass_construction_area_);
}

void ConstructionSceneDecider::JudgeConstructionIntrusionLevel() {
  auto& construction_scene_decider_output =
      session_->mutable_planning_context()
          ->mutable_construction_scene_decider_output();
  ConstructionIntrusionLevel construction_intrusion_level = ConstructionIntrusionLevel :: NONE;
  std::vector<int> available_virtual_lane_ids;
  available_virtual_lane_ids.reserve(5);
  const auto virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto& current_lane = virtual_lane_mgr->get_current_lane();
  const auto& current_virtual_lane_id = current_lane->get_virtual_id();
  const auto left_left_virtual_lane_id = current_virtual_lane_id - 2;
  const auto left_virtual_lane_id = current_virtual_lane_id - 1;
  const auto right_right_virtual_lane_id = current_virtual_lane_id + 2;
  const auto right_virtual_lane_id = current_virtual_lane_id + 1;
  bool is_current_lane_available = true;
  bool is_left_left_lane_available = true;
  bool is_left_lane_available = true;
  bool is_right_right_lane_available = true;
  bool is_right_lane_available = true;
  const auto& left_left_lane = virtual_lane_mgr->get_lane_with_virtual_id(left_left_virtual_lane_id);
  const auto& left_lane = virtual_lane_mgr->get_lane_with_virtual_id(left_virtual_lane_id);
  const auto& right_right_lane = virtual_lane_mgr->get_lane_with_virtual_id(right_right_virtual_lane_id);
  const auto& right_lane = virtual_lane_mgr->get_lane_with_virtual_id(right_virtual_lane_id);
  is_current_lane_available = CheckLaneAvailable(current_lane, false, false);
  is_left_left_lane_available = CheckLaneAvailable(left_left_lane, true, false);
  is_left_lane_available = CheckLaneAvailable(left_lane, true, false);
  is_right_right_lane_available = CheckLaneAvailable(right_right_lane, false, true);
  is_right_lane_available = CheckLaneAvailable(right_lane, false, true);
  if (is_current_lane_available) {
    available_virtual_lane_ids.emplace_back(current_virtual_lane_id);
  }
  if (is_left_left_lane_available) {
    available_virtual_lane_ids.emplace_back(left_left_virtual_lane_id);
  }
  if (is_left_lane_available) {
    available_virtual_lane_ids.emplace_back(left_virtual_lane_id);
  }
  if (is_right_right_lane_available) {
    available_virtual_lane_ids.emplace_back(right_right_virtual_lane_id);
  }
  if (is_right_lane_available) {
    available_virtual_lane_ids.emplace_back(right_virtual_lane_id);
  }
  // 判定车道是否是真实的

  // 如果存在施工区域，判定上游给的参考车道是否能够通行
  // 如果一条都不能满足通行，则走通行空间的生成（完全占道）
  // 否则走车道内或者变道（轻微占道和中等变道）
  if (!is_exist_construction_area_) {
    // 不存在施工区域
    construction_intrusion_level =
        ConstructionIntrusionLevel :: NONE;
  } else if (!is_current_lane_available && !is_right_lane_available &&
             !is_left_lane_available && !is_left_left_lane_available &&
             !is_right_right_lane_available) {
    // 可能需要做一下滞回操作
    construction_intrusion_level =
        ConstructionIntrusionLevel :: HIGH;
  } else {
    construction_intrusion_level =
        ConstructionIntrusionLevel :: MEDIUM;
  }
  construction_scene_decider_output.construction_intrusion_level =
      construction_intrusion_level;
  construction_scene_decider_output.is_current_lane_available =
      is_current_lane_available;
  construction_scene_decider_output.is_left_left_lane_available =
      is_left_left_lane_available;
  construction_scene_decider_output.is_left_lane_available =
      is_left_lane_available;
  construction_scene_decider_output.is_right_right_lane_available =
      is_right_right_lane_available;
  construction_scene_decider_output.is_right_lane_available =
      is_right_lane_available;
  construction_scene_decider_output.available_virtual_lane_ids =
      available_virtual_lane_ids;
}

bool ConstructionSceneDecider::CheckLaneAvailable(
    const std::shared_ptr<VirtualLane> seach_lane,
    bool is_left, bool is_right) {
  if (seach_lane == nullptr) {
    ILOG_DEBUG << "seach fail: seach lane is nullptr";
    return false;
  }
  if (!is_exist_construction_area_) {
    // 不存在施工区域
    return true;
  }
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto& current_lane = virtual_lane_mgr->get_current_lane();
  const auto& current_virtual_lane_id = current_lane->get_virtual_id();
  const bool is_current_lane = current_virtual_lane_id == seach_lane->get_relative_id();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& planning_init_point = ego_state->planning_init_point();
  const auto ego_v = ego_state->ego_v();
  const auto care_ego_time = 3; // 后续需要考虑自车通行速度
  const auto care_length = care_ego_time * ego_v;
  std::shared_ptr<ReferencePath> target_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(seach_lane->get_virtual_id(), false);
  if (target_refline == nullptr) {
    ILOG_DEBUG << "target_refline is nullptr";
    return false;
  }
  std::shared_ptr<planning_math::KDPath> target_lane_frenet_coord =
      target_refline->get_frenet_coord();
  if (target_lane_frenet_coord == nullptr) {
    ILOG_DEBUG << "target_lane_frenet_coord is nullptr";
    return false;
  }
  Point2D ego_cart_point{planning_init_point.lat_init_state.x(),
                         planning_init_point.lat_init_state.y()};
  Point2D ego_frenet_point;
  if (!target_lane_frenet_coord->XYToSL(ego_cart_point, ego_frenet_point)) {
    return false;
  }
  double lane_width = 3.5;
  double half_lane_width = lane_width * 0.5;
  int num_satisfy_construction_agent = 0;
  double pass_thre = vehicle_param.width + kLatPassThre;

  for (const auto & [ cluster_id, cluster_area ] :
       construction_agent_cluster_attribute_map_) {
    const auto& points = cluster_area.points;
    for (const auto& p : points) {
      Point2D agent_cart_point{p.x, p.y};
      Point2D agent_frenet_point;
      if (!target_lane_frenet_coord->XYToSL(agent_cart_point, agent_frenet_point)) {
        continue;
      }
      lane_width = seach_lane->width_by_s(agent_frenet_point.x);
      half_lane_width = lane_width * 0.5;
      // 判定自车3s内自车横向是否可达
      bool is_lat_available = true;
      if (agent_frenet_point.x < care_length + ego_frenet_point.x &&
          std::fabs(ego_frenet_point.y) > std::fabs(agent_frenet_point.y) &&
          (ego_frenet_point.y * agent_frenet_point.y > 0)) {
        is_lat_available = false;
      }
      if (!is_lat_available) {
        return false;
      }
      if (half_lane_width - std::fabs(agent_frenet_point.y) >
          kLaneAvailableLatPassThre) {
          // 需要考虑距离自车远近的影响
        num_satisfy_construction_agent++;
      }
    }
  }
  if (num_satisfy_construction_agent >= kNumSatisfySonstructionAgent) {
    // 满足一定数量侵占当车道，则该车道不可用
    return false;
  } else {
    return true;
  }
  return false;
}

void ConstructionSceneDecider::UpdateDriveArea() {
  if (construction_agent_cluster_attribute_map_.size() <= 0) {
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
         construction_agent_cluster_attribute_map_) {
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
      construction_agent_cluster_attribute_map_[result.first].direction =
          ConstructionDirection::UNSURE;
      std::cout << "abnormal ref" << std::endl;
    } else if (result.second.count(1)) {
      construction_agent_cluster_attribute_map_[result.first].direction =
          ConstructionDirection::LEFT;
    } else if (result.second.count(2)) {
      construction_agent_cluster_attribute_map_[result.first].direction =
          ConstructionDirection::RIGHT;
    }

    if (result.second.count(-1)) {
      construction_agent_cluster_attribute_map_[result.first].direction =
          ConstructionDirection::UNSURE;
      std::cout << "colinear_or_facing ref" << std::endl;
    }
  }
}

void ConstructionSceneDecider::GenerateConstructionSceneOutput() {
  auto& construction_scene_decider_output =
      session_->mutable_planning_context()
          ->mutable_construction_scene_decider_output();
  construction_scene_decider_output.is_exist_construction_area =
      is_exist_construction_area_;
  construction_scene_decider_output.construction_agent_cluster_attribute_map =
      construction_agent_cluster_attribute_map_;
  construction_scene_decider_output.is_pass_construction_area =
      is_pass_construction_area_;
}

void ConstructionSceneDecider::SaveLatDebugInfo() {
  // 保存信息
  const auto& construction_scene_decider_output =
      session_->planning_context()
          .construction_scene_decider_output();
  JSON_DEBUG_VALUE("is_current_lane_available",
                    construction_scene_decider_output.is_current_lane_available);
  JSON_DEBUG_VALUE("is_right_lane_available",
                    construction_scene_decider_output.is_right_lane_available);
  JSON_DEBUG_VALUE("is_left_lane_available",
                    construction_scene_decider_output.is_left_lane_available);
  JSON_DEBUG_VALUE("is_left_left_lane_available",
                    construction_scene_decider_output.is_left_left_lane_available);
  JSON_DEBUG_VALUE("is_right_right_lane_available",
                    construction_scene_decider_output.is_right_right_lane_available);
  JSON_DEBUG_VALUE("construction_intrusion_level",
                    static_cast<uint32_t>(construction_scene_decider_output
                    .construction_intrusion_level));
  // 转成 double 类型
  std::vector<double> available_virtual_lane_ids_double;
  available_virtual_lane_ids_double.reserve(construction_scene_decider_output
      .available_virtual_lane_ids.size());
  std::transform(construction_scene_decider_output.available_virtual_lane_ids.begin(),
                 construction_scene_decider_output.available_virtual_lane_ids.end(),
                 std::back_inserter(available_virtual_lane_ids_double),
                 [](int id) { return static_cast<double>(id); });
  JSON_DEBUG_VECTOR("construction_available_virtual_lane_ids",
                    available_virtual_lane_ids_double, 0);
  JSON_DEBUG_VALUE("is_construction_agent_cluster_success",
                   is_construction_agent_cluster_success_);
  if (is_construction_agent_cluster_success_) {
    std::vector<double> construction_agent_cluster_attribute_ids;
    std::vector<double> construction_agent_clusters;
    std::vector<double> construction_agent_clusters_length;
    std::vector<double> construction_agent_clusters_driection;
    for (const auto& cluster_attribute_iter :
         construction_agent_cluster_attribute_map_) {
      const ConstructionAgentPoints& points =
          cluster_attribute_iter.second.points;
      for (const auto& p : points) {
        construction_agent_cluster_attribute_ids.emplace_back(p.id);
      }
      construction_agent_clusters.emplace_back(cluster_attribute_iter.first);
      construction_agent_clusters_length.emplace_back(points.size());
      construction_agent_clusters_driection.emplace_back(
          static_cast<uint32_t>(cluster_attribute_iter.second.direction));
    }
    JSON_DEBUG_VECTOR("construction_agent_clusters",
                      construction_agent_clusters, 0);
    JSON_DEBUG_VECTOR("construction_agent_clusters_length",
                      construction_agent_clusters_length, 0);
    JSON_DEBUG_VECTOR("construction_agent_cluster_attribute_ids",
                      construction_agent_cluster_attribute_ids, 0);
    JSON_DEBUG_VECTOR("construction_agent_clusters_driection",
                      construction_agent_clusters_driection, 0);
  }
}

}  // namespace planning
