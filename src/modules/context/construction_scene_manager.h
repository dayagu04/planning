#pragma once

#include <map>
#include <utility>

#include "config/basic_type.h"
#include "session.h"
#include "virtual_lane.h"

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
  double length, width;

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
                         double right_dist, double length, double width)
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
        length(length),
        width(width),
        visited(false) {}
};
using ConstructionAgentPoints = std::vector<ConstructionAgentPoint>;

struct ConstructionAgentClusterArea {
  ConstructionAgentPoints points;
  ConstructionDirection direction = ConstructionDirection :: DEFAULT;
};

struct RoadBoundaryCluster {
  std::vector<Point2d> points;
  ConstructionDirection direction = ConstructionDirection :: DEFAULT;
};

struct ConstructionSceneOutput {
  std::map<int, ConstructionAgentClusterArea>
      construction_agent_cluster_attribute_map;  // 施工区域聚类结果
  std::map<int, RoadBoundaryCluster> road_boundaries_clusters_map; // 路沿决策结果
  bool is_exist_construction_area = false;       // 是否存在施工区域
  bool is_pass_construction_area = false;  // 是否正在经过施工区域（自车状态）
  ConstructionIntrusionLevel construction_intrusion_level = ConstructionIntrusionLevel :: NONE;
  // 车道是否被堵塞
  bool is_current_lane_blocked = false;
  bool is_left_left_lane_blocked = false;
  bool is_left_lane_blocked = false;
  bool is_right_right_lane_blocked = false;
  bool is_right_lane_blocked = false;
  std::vector<int> blocked_virtual_lane_ids;
  // 车道是否可达
  bool is_current_lane_available = true;
  bool is_left_left_lane_available = true;
  bool is_left_lane_available = true;
  bool is_right_right_lane_available = true;
  bool is_right_lane_available = true;
  std::vector<int> available_virtual_lane_ids;
  // 是否启用通行空间
  bool enable_construction_passage = false;
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
    is_current_lane_blocked = false;
    is_left_left_lane_blocked = false;
    is_left_lane_blocked = false;
    is_right_right_lane_blocked = false;
    is_right_lane_blocked = false;
    blocked_virtual_lane_ids.clear();
    enable_construction_passage = false;
  }
};

class ConstructionSceneManager {
 public:
  ConstructionSceneManager(planning::framework::Session* session);

  virtual ~ConstructionSceneManager();

  bool update();

  bool InitInfo();

  void UpdateConstructionAgentClusters();

  void GetOriginLaneWidthByConstructionAgent(
      const std::shared_ptr<VirtualLane> ego_lane,
      const double construction_agent_s, const double construction_agent_l,
      bool is_left, double* dist);

  bool ConstructionAgentDistance(const ConstructionAgentPoint& a,
                                 const ConstructionAgentPoint& b, double eps_s,
                                 double eps_l);

  void ExpandCluster(ConstructionAgentPoints& construction_agent_points,
                     int index, int c, double eps_s, double eps_l, int minPts);

  void DbScan(ConstructionAgentPoints& construction_agent_points, double eps_s,
              double eps_l, int minPts);

  double CalcClusterToBoundaryDist(const ConstructionAgentPoints& points,
                                   RequestType direction);

  double QueryLaneWidth(
      const double s0,
      const std::vector<std::pair<double, double>>& lane_s_width);

  bool IsConstructionAgent(iflyauto::ObjectType type);

  void IdentifyConstructionScene();

  void IsExistConstructionArea();

  void JudgeConstructionIntrusionLevel();

  void CheckLaneAvailableAndBlocked(
      const std::shared_ptr<VirtualLane> seach_lane,
      bool is_left, bool is_right,
      bool &is_available, bool &is_blocked);

  void UpdateDriveArea();

  std::pair<bool, int> CalIntersectionRefAndObstacle(
      std::shared_ptr<planning_math::KDPath> lane_frenet_coord,
      const std::vector<Point2d>& ref_points,
      const std::vector<Point2d>& obstacle_points);

  void UpdateResult(
      const std::map<int, std::map<int, std::vector<int>>>& results,
      const std::map<int, std::map<int, std::vector<int>>>&
          road_boundary_results);

  void RoadBoundaryPreProcess();

  void GenerateConstructionSceneOutput();

  void SaveLatDebugInfo();

  const ConstructionSceneOutput &get_construction_scene_output()
      const {
    return construction_scene_output_;
  }

 private:
  planning::framework::Session* session_;
  ConstructionAgentPoints construction_agent_points_;
  ConstructionAgentPoints construction_agent_cluster_;
  std::vector<std::pair<double, double>> origin_lane_s_width_;
  bool is_construction_agent_cluster_success_ = false;
  int no_construction_area_counter_ = std::numeric_limits<int>::max() / 2;
  // hysteresis for is_exist_construction_area_
  std::map<int, ConstructionAgentClusterArea>
      construction_agent_cluster_attribute_map_;
  std::map<int, RoadBoundaryCluster> road_boundaries_clusters_map_;
  bool is_exist_construction_area_ = false;
  bool is_pass_construction_area_ = false;
  ConstructionSceneOutput construction_scene_output_;
  bool enable_construction_passage_ = false;
  bool is_lane_blocked_ = false;
};

}  // namespace planning
