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
#include "general_lateral_decider_utils.h"
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

  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  auto start_time = IflyTime::Now_ms();

  if (!InitInfo()) {
    return false;
  };

  auto end_time = IflyTime::Now_ms();
  // 施工障碍物聚类

  JSON_DEBUG_VALUE("ConstructionSceneDeciderCostTime", end_time - start_time);

  return true;
}



void ConstructionSceneDecider::UpdateConstructionAgentClusters() {
  // 施工障碍物聚类
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double ego_fx = std::cos(ego_state->ego_pose_raw().theta);
  double ego_fy = std::sin(ego_state->ego_pose_raw().theta);

  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(origin_lane_virtual_id_, false);

  const auto &frenet_obstacles_map = origin_refline->get_obstacles_map();
  base_frenet_coord_ = origin_refline->get_frenet_coord();
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    ILOG_DEBUG << "fail to get ego position on base lane";
    // is_construction_agent_lane_change_situation_ = false;
    return;
  }

  const double ego_rear_edge = vehicle_param.rear_edge_to_rear_axle;
  double eps_s = vehicle_param.length * kLongClusterCoeff;
  double eps_l = vehicle_param.width + kLatClusterThre;
  int cone_nums_of_front_objects = 0;
  int minPts = 1;
  construction_agent_points_.clear();
  construction_agent_cluster_attribute_set_.clear();

  const auto& tracks_map = lateral_obstacle_->tracks_map();
  const auto& front_obstacles_array =
      lateral_obstacle_->front_tracks();
  for (const auto front_obstacle : front_obstacles_array) {
    int obstacle_id = front_obstacle->id();
    auto front_vehicle_iter = tracks_map.find(obstacle_id);
    Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
    if (front_vehicle_iter != tracks_map.end()) {
      if (obstacle_id == kInvalidAgentId) {
        continue;
      }
      if (front_vehicle_iter->second->type() ==
          iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
        if (front_vehicle_iter->second->d_s_rel() < -ego_rear_edge ||
            front_vehicle_iter->second->d_s_rel() >
                base_frenet_coord_->Length() - ego_frenet_point.x) {
          continue;
        }
        cone_nums_of_front_objects++;
        Point2D obs_cart_point{0.0, 0.0};
        Point2D obs_frenet_point;
        obs_cart_point.x = ego_cart_point.x +
                           front_vehicle_iter->second->obstacle()->x_center() * ego_fx -
                           front_vehicle_iter->second->obstacle()->y_center() * ego_fy;
        obs_cart_point.y = ego_cart_point.y +
                           front_vehicle_iter->second->obstacle()->x_center() * ego_fy +
                           front_vehicle_iter->second->obstacle()->y_center() * ego_fx;
        if (frenet_obstacles_map.find(obstacle_id) == frenet_obstacles_map.end() || !frenet_obstacles_map.at(obstacle_id)->b_frenet_valid()) {
          continue;
        }
        double cone_s = front_vehicle_iter->second->frenet_s();
        double cone_l = front_vehicle_iter->second->frenet_l();
        double dist_to_left_boundary;
        if (!GetOriginLaneWidthByCone(base_lane, cone_s, cone_l, true,
                                      &dist_to_left_boundary)) {
          is_construction_agent_lane_change_situation_ = false;
          return;
        }
        double dist_to_right_boundary;
        if (!GetOriginLaneWidthByCone(base_lane, cone_s, cone_l, false,
                                      &dist_to_right_boundary)) {
          is_construction_agent_lane_change_situation_ = false;
          return;
        }
        auto point = ConstructionAgentPoint(front_vehicle_iter->first, obs_cart_point.x,
                               obs_cart_point.y, cone_s, cone_l,
                               dist_to_left_boundary, dist_to_right_boundary);
        construction_agent_points_.push_back(point);
      }
    } else {
      continue;
    }
  }
  JSON_DEBUG_VALUE("cone_nums_of_front_objects", cone_nums_of_front_objects);

  if (construction_agent_points_.empty()) {
    // if no cones found, counter--
    ILOG_DEBUG << "no cone found!!!";
    cone_alc_trigger_counter_ =
        std::max(cone_alc_trigger_counter_ - 1, kConeAlcCountLowerThre);
    is_construction_agent_lane_change_situation_ = false;
    return;
  } else {
    // 检测类型为cone的障碍物赋予cluster属性
    DbScan(construction_agent_points_, eps_s, eps_l, minPts);
  }

  construction_agent_cluster_size_.clear();
  construction_agent_cluster_.clear();
  bool did_break = false;
  for (const auto& p : construction_agent_points_) {
    // 构建相同cluster属性所包含cones的map
    construction_agent_cluster_attribute_set_[p.cluster].push_back(p);
  }
  for (const auto& cluster_attribute_iter : construction_agent_cluster_attribute_set_) {
    int cluster = cluster_attribute_iter.first;
    const std::vector<ConePoint>& points = cluster_attribute_iter.second;
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
      cone_alc_trigger_counter_++;
      ILOG_DEBUG << "trigger_counter is " << cone_alc_trigger_counter_ << ", cluster is " << cluster;
      if (cone_alc_trigger_counter_ >= kConeAlcCountThre) {
        is_construction_agent_lane_change_situation_ = true;
        return;
      }
      did_break = true;
      break;
    }
  }
  // if all clusters is far away from cernter line, counter--
  if (!did_break) {
    cone_alc_trigger_counter_ =
        std::max(cone_alc_trigger_counter_ - 1, kConeAlcCountLowerThre);
    ILOG_DEBUG << "trigger_counter is " << cone_alc_trigger_counter_;
    if (cone_alc_trigger_counter_ < kConeAlcCountThre) {
      is_construction_agent_lane_change_situation_ = false;
    }
  }
  // if all cone l is larger than threshold, then no need to lane change
  return;
}


}

