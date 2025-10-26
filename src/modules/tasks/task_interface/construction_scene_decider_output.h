#pragma once
#include <string>
#include <utility>
#include <vector>

#include "config/basic_type.h"
#include "define/geometry.h"
#include "lateral_motion_planner.pb.h"
#include "task_basic_types.h"

namespace planning {

enum class ConstructionDirection { DEFAULT, LEFT, RIGHT, LON, UNSURE };

enum class ConstructionIntrusionLevel {
  NONE = 0,   // 无施工影响
  LOW,        // 轻微侵入（远离车道边缘或短暂施工）
  MEDIUM,     // 中度侵入（占据部分车道或影响路径）
  HIGH        // 严重侵入（明显占道或阻塞车道）
};

struct ConstructionAgentPoint {
  // 施工障碍物数据结构体
  double x, y;
  double car_x, car_y;
  double s, l;
  double left_dist, right_dist;
  int32_t id;
  int cluster;
  bool visited;

  // Default constructor
  ConstructionAgentPoint()
      : x(0.0),  // 世界系
        y(0.0),
        car_x(0.0),  // 自车系
        car_y(0.0),
        s(0.0),  // 暂不用，可根据自己需求转化
        l(0.0),
        left_dist(0.0),
        right_dist(0.0),
        id(0),
        cluster(-1),
        visited(false) {}
  // Parameterized constructor
  ConstructionAgentPoint(int32_t id, double x, double y, double car_x,
                         double car_y, double s, double l, double left_dist,
                         double right_dist)
      : x(x),
        y(y),
        car_x(car_x),
        car_y(car_y),
        s(s),
        l(l),
        left_dist(left_dist),
        right_dist(right_dist),
        id(id),
        cluster(-1),
        visited(false) {}
};
using ConstructionAgentPoints = std::vector<ConstructionAgentPoint>;

struct ConstructionAgentClusterArea {
  ConstructionAgentPoints points;
  ConstructionDirection direction = ConstructionDirection :: DEFAULT;
};

struct ConstructionSceneDeciderOutput {
  std::map<int, ConstructionAgentClusterArea>
      construction_agent_cluster_attribute_map;  // 施工区域聚类结果
  bool is_exist_construction_area = false;       // 是否存在施工区域
  bool is_pass_construction_area = false;  // 是否正在经过施工区域（自车状态）
  ConstructionIntrusionLevel construction_intrusion_level = ConstructionIntrusionLevel :: NONE;
  bool is_current_lane_available = true;
  bool is_left_left_lane_available = true;
  bool is_left_lane_available = true;
  bool is_right_right_lane_available = true;
  bool is_right_lane_available = true;
  std::vector<int> available_virtual_lane_ids;
  void Clear() {
    construction_agent_cluster_attribute_map.clear();
    is_exist_construction_area = false;
    is_pass_construction_area = false;
    construction_intrusion_level = ConstructionIntrusionLevel :: NONE;
    is_current_lane_available = true;
    is_left_left_lane_available = true;
    is_left_lane_available = true;
    is_right_right_lane_available = true;
    is_right_lane_available = true;
    available_virtual_lane_ids.clear();
  }
};
}  // namespace planning