#pragma once
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace planning {

enum class LatObstacleDecisionType { LEFT, RIGHT, IGNORE };

enum class LonObstacleDecisionType { OVERTAKE, YIELD, IGNORE };

enum class LatIngoreType { IGNORE_BOTH, IGNORE_LEFT, IGNORE_RIGHT, IGNORE_NONE };

enum class ObsRelPosType { FRONT, CUTIN, CROSSING, ADJACENT, REAR, UNDEFINED };

struct Bound {
  double lower{std::numeric_limits<double>::min()};
  double upper{std::numeric_limits<double>::max()};
};
using Bounds = std::vector<Bound>;

struct BoundInfo {
  int id = 0;
  std::string type = "default";
};

struct WeightedBound {
  double lower;
  double upper;
  double weight;
  BoundInfo bound_info;
};
using WeightedBounds = std::vector<WeightedBound>;

struct WeightedLonLeadBound {
  double s_lead;
  double v_lead;
  double t_lead;
  double t_ego;
  double weight;
};
using WeightedLonLeadBounds = std::vector<WeightedLonLeadBound>;

// dynamic obstacle
struct DecisionTimePosition {
  double t = 0;
  double s = 0;
  double l = 0;
};

struct ObstaclePositionDecision {
  DecisionTimePosition tp;

  LatObstacleDecisionType lat_decision;
  WeightedBounds lat_bounds;

  LonObstacleDecisionType lon_decision;
  WeightedBounds lon_bounds;
  WeightedLonLeadBounds lon_lead_bounds;
};

struct ObstacleDecision {
  int id_ = 0;
  std::vector<ObstaclePositionDecision> position_decisions;
  ObsRelPosType rel_pos_type;
};
using ObstacleDecisions = std::map<int, ObstacleDecision>;

struct ObstacleExtendDecision {
  bool extended = false;
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double obstacle_length;
  double obstacle_width;
  double obstacle_type;
  double obstacle_safe_center_l;
  double obstacle_velocity;
  WeightedBounds obstacle_normal_lat_bounds;
};

struct ObstaclePotentialDecision {
  int id = 0;
  ObstacleExtendDecision extend_decisions;
};
using ObstaclePotentialDecisions = std::map<int, ObstaclePotentialDecision>;

// map obstacle
struct MapObstaclePositionDecision {
  DecisionTimePosition tp;

  WeightedBounds lat_bounds;
};
using MapObstacleDecision = std::vector<MapObstaclePositionDecision>;

struct LonObstalceYieldInfo {
  int yield_id{0};
  double yield_upper{0};
  double yield_buff{0};
};

struct LonObstacleOverlapInfo {
  double s{0};
  double t{0};
};

struct LonRefPath {
  std::vector<double> t_list;
  std::vector<std::pair<double, double>> s_refs;  // <offset, weight>
  std::vector<std::pair<double, double>> ds_refs;

  std::vector<WeightedBounds> bounds;
  std::vector<WeightedLonLeadBounds> lon_lead_bounds;
  std::unordered_map<int, std::vector<LonObstacleOverlapInfo>> lon_obstacle_overlap_info;
  std::vector<LonObstalceYieldInfo> lon_obstacle_yield_info;

  Bounds lon_bound_v;
  Bounds lon_bound_a;
  Bounds lon_bound_jerk;
};

struct LatLonEgoState {
  double s_start;
  double delta_s;
  double velocity;
  double frenet_l;
  double frenet_dl;
  double frenet_ddl;
};

struct PiecewiseJerkPoint {
  double s;
  double l;
  double dl;
  double ddl;
};

struct LatRefPath {
  std::vector<double> s_list;
  std::vector<std::pair<double, double>> enu_path_refline;  // <x, y>
  std::vector<std::pair<double, double>> path_refline;      // <offset, weight>
  std::vector<std::pair<double, double>> dl_refs;           // <offset, weight>

  std::vector<WeightedBounds> bounds;

  Bounds lateral_bound_dd;
  double lateral_bound_ddd;
};

struct TrafficLightDecision {
  double stop_distance = 500.0;
  double velocity_limit = 150.0 / 3.6;
  bool stop_flag = false;
};

struct PassableAreaInfo {
  double width;
  BoundInfo left_bound_info;
  BoundInfo right_bound_info;
};

struct LateralAvdCarsInfo {
  std::array<std::vector<double>, 2> avd_car_past;
};

struct PlanningInfo {
  //  DecisionTimePositions decision_time_positions;
  ObstacleDecisions obstacle_decisions;
  MapObstacleDecision map_obstacle_decision;
  LateralAvdCarsInfo lateral_avd_cars_info;

  std::vector<LatRefPath> lat_ref_paths;
  LatLonEgoState ego_state;
  LonRefPath lon_ref_path;
  std::vector<PassableAreaInfo> passable_area_info;

  TrafficLightDecision traffic_light_decision;
};

}  // namespace planning
