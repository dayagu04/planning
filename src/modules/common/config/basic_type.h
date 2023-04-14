#pragma once

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "framework/planning_def.h"
#include "modules/common/utils/cartesian_coordinate_system.h"
#include "modules/common/utils/frenet_coordinate_system.h"

#define DEFAULT_LANE_WIDTH (3.8)

namespace planning {

enum RequestType { NO_CHANGE, LEFT_CHANGE, RIGHT_CHANGE };

enum LooseBoundType { NONE_SIDE, LEFT_SIDE, RIGHT_SIDE, BOTH_SIDE };

enum LeverStatus {
  LEVER_STATE_OFF,
  LEVER_STATE_LEFT,
  LEVER_STATE_RIGHT,
  LEVER_STATE_RESERVED1,
  LEVER_STATE_RESERVED2
};

enum LaneBoundaryType {
  MARKING_UNKNOWN = 0,                  // 未知线型
  MARKING_DASHED = 1,                   // 虚线
  MARKING_SOLID = 2,                    // 实线
  MARKING_SHORT_DASHED = 3,             // 短虚线
  MARKING_DOUBLE_DASHED = 4,            // 双虚线
  MARKING_DOUBLE_SOLID = 5,             // 双实线
  MARKING_LEFT_DASHED_RIGHT_SOLID = 6,  // 左虚右实线
  MARKING_LEFT_SOLID_RIGHT_DASHED = 7   // 左实右虚线
};

enum RequestSource {
  NO_REQUEST,
  INT_REQUEST,
  MAP_REQUEST,
  ACT_REQUEST,
  ROUTE_REQUEST
};

struct VirtualPoint2D {
  double x = 0.0;
  double y = 0.0;

  VirtualPoint2D() = default;
  VirtualPoint2D(double xx, double yy) : x(xx), y(yy) {}
};

struct VirtualPoint3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  VirtualPoint3D() = default;
  VirtualPoint3D(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
};

struct TrajectoryPoint {
  // enu
  double x = 0;
  double y = 0;
  double heading_angle = 0;
  double curvature = 0;
  double dkappa = 0;
  double ddkappa = 0;
  double t = 0;
  double v = 0;
  double a = 0;

  // frenet
  double s = 0;
  double l = 0;
  bool frenet_valid = false;
};
using TrajectoryPoints = std::vector<TrajectoryPoint>;

enum ScenarioStateEnum {
  ROAD_NONE = 0,
  ROAD_LC_LWAIT,
  ROAD_LC_RWAIT,
  ROAD_LC_LCHANGE,
  ROAD_LC_RCHANGE,
  ROAD_LC_LBACK,
  ROAD_LC_RBACK,

  ROAD_LB_LBORROW,
  ROAD_LB_RBORROW,
  ROAD_LB_LBACK,
  ROAD_LB_RBACK,
  ROAD_LB_LRETURN,
  ROAD_LB_RRETURN,
  ROAD_LB_LSUSPEND,
  ROAD_LB_RSUSPEND,

  INTER_GS_NONE,
  INTER_GS_LC_LWAIT,
  INTER_GS_LC_RWAIT,
  INTER_GS_LC_LCHANGE,
  INTER_GS_LC_RCHANGE,
  INTER_GS_LC_LBACK,
  INTER_GS_LC_RBACK,

  INTER_GS_LB_LBORROW,
  INTER_GS_LB_RBORROW,
  INTER_GS_LB_LBACK,
  INTER_GS_LB_RBACK,
  INTER_GS_LB_LRETURN,
  INTER_GS_LB_RRETURN,
  INTER_GS_LB_LSUSPEND,
  INTER_GS_LB_RSUSPEND,

  INTER_TR_NONE,
  INTER_TR_LC_LWAIT,
  INTER_TR_LC_RWAIT,
  INTER_TR_LC_LCHANGE,
  INTER_TR_LC_RCHANGE,
  INTER_TR_LC_LBACK,
  INTER_TR_LC_RBACK,

  INTER_TR_LB_LBORROW,
  INTER_TR_LB_RBORROW,
  INTER_TR_LB_LBACK,
  INTER_TR_LB_RBACK,
  INTER_TR_LB_LRETURN,
  INTER_TR_LB_RRETURN,
  INTER_TR_LB_LSUSPEND,
  INTER_TR_LB_RSUSPEND,

  INTER_TL_NONE,
  INTER_TL_LC_LWAIT,
  INTER_TL_LC_RWAIT,
  INTER_TL_LC_LCHANGE,
  INTER_TL_LC_RCHANGE,
  INTER_TL_LC_LBACK,
  INTER_TL_LC_RBACK,

  INTER_TL_LB_LBORROW,
  INTER_TL_LB_RBORROW,
  INTER_TL_LB_LBACK,
  INTER_TL_LB_RBACK,
  INTER_TL_LB_LRETURN,
  INTER_TL_LB_RRETURN,
  INTER_TL_LB_LSUSPEND,
  INTER_TL_LB_RSUSPEND,

  INTER_UT_NONE
};

enum ScenarioEnum { SCENARIO_CRUISE = 0, SCENARIO_LOW_SPEED };

enum FaultDiagnosisType {
  OFF_ROUTE = 0,
  OFF_MAP,
  LATERAL_DIFF,
  LINE_PRESSING_DRIVING,
  BORDER_PRESSING_DRIVING,
  LOCALIZATION_FAULT,
  CONTROL_FAULT
};

struct EgoPredictionTrajectory {
  TrajectoryPoints trajectory;
  double logit = 0;
  std::vector<std::uint64_t> track_ids;
};
using EgoPredictionTrajectorys = std::vector<EgoPredictionTrajectory>;

struct EgoPredictionObject {
  double timestamp_us = 0;
  double timestamp = 0;
  bool ego_prediction_valid = false;
  std::vector<EgoPredictionTrajectory> trajectory_array;
};

enum ResultTrajectoryType { REFERENCE_PATH = 0, RAW_TRAJ, REFINED_TRAJ };

struct AccSafetyInfo {
  bool need_takeover = false;
};
struct ObstacleInformation {
  int obstacle_id{-1};
  int obstacle_type{-1};
  double obstacle_s{-1.0};
  double obstacle_v{-1.0};
  double obstacle_length{0.0};
  double duration{-1.0};
};

struct LeadoneInfo {
  bool has_leadone{false};
  ObstacleInformation leadone_information;
};

struct CutinInfo {
  bool has_cutin{false};
  std::vector<ObstacleInformation> cutin_information;
};

struct CIPVInfo {
  bool has_CIPV{false};
  std::vector<ObstacleInformation> CIPV_information;
};

struct LonDecisionInfo {
  LeadoneInfo leadone_info;
  CutinInfo cutin_info;
  CIPVInfo CIPV_info;
  AccSafetyInfo acc_safety_info;
  bool nearby_obstacle{false};
  double map_velocity_limit{0.0};
};

struct CurvatureInfo {
  double max_curvature = 0.0;
  double curv_velocity_limit = 120.0 / 3.6;
};

struct StartStopInfo {
  bool is_stop{false};
  bool is_start{false};
  bool enable_stop{false};
};

struct LatDecisionInfo {
  int direction = 0;
  int id = 0;
  int type = 0;
};
using LatDecisionInfos = std::unordered_map<int, LatDecisionInfo>;

struct FaultDiagnosisInfo {
  bool able_to_auto{true};
  bool pre_able_to_auto{false};
  int fault_type = -1;
  double pre_x{0.0};
  double pre_y{0.0};
  bool localization_fault_state{false};
  double localization_fault_duration{0.0};
  bool localization_fault_disable{false};
};

struct AccInfo {
  bool enable_jerk_up{false};
  bool enable_jerk_down{false};
  bool enable_v_cost{false};
  bool enable_v_limit{false};
  double s_ref{1000.0};
  double s_bound{1000.0};
  double v_ref{1000.0};
  double v_bound{1000.0};
};

struct AdaptiveCruiseControlInfo {
  bool navi_speed_flag{false};
  bool navi_time_distance_flag{false};
  AccInfo navi_speed_control_info;
  AccInfo navi_time_distance_info;
};

struct PlanningResult {
  int target_lane_id;
  ScenarioStateEnum target_scenario_state = ROAD_NONE;
  TrajectoryPoints raw_traj_points;
  TrajectoryPoints traj_points;
  RequestType turn_signal = NO_CHANGE;
  CurvatureInfo curvature_info;
  int use_backup_cnt = 0;
  double timestamp = 0.0;
  bool use_refined_reference_path = false;
  int contingency_trigger_index = 0;
  std::string extra_json;
};

struct PlanningInitPoint {
  double x;
  double y;
  double heading_angle;
  double curvature;
  double dkappa;
  double v;
  double a;
  double jerk;
  double relative_time;
  FrenetState frenet_state;
};

class ReferencePath;
struct CoarsePlanningInfo {
  ScenarioStateEnum source_state;
  ScenarioStateEnum target_state;
  RequestSource lane_change_request_source;
  int source_lane_id;
  int target_lane_id;
  bool bind_end_state;
  std::shared_ptr<ReferencePath> reference_path;
  TrajectoryPoints trajectory_points;
  // overtake_obstacles and yield_obstacles are used only under wait state
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
};

struct FrenetBoundary {
  double s_start;
  double s_end;
  double l_start;
  double l_end;
};

struct FrenetBoundaryCorners {
  double s_front_left;
  double l_front_left;
  double s_front_right;
  double l_front_right;
  double s_rear_left;
  double l_rear_left;
  double s_rear_right;
  double l_rear_right;
};

struct FrenetObstacleBoundary {
  double s_start{std::numeric_limits<double>::max()};
  double s_end{std::numeric_limits<double>::lowest()};
  double l_start{std::numeric_limits<double>::max()};
  double l_end{std::numeric_limits<double>::lowest()};
};

enum CurrentState {
  INIT = 0,
  LANE_KEEPING,
  APPROACH_STOPLINE_SLOW,
  PASS_INTERSECTION,
  RED_LIGHT_STOP,
  COVER_LIGHT,
  INTO_WAIT_ZONE
};

struct PointLLH {
    double Longitude;    // Longitude in degrees, ranging from -180 to 180,(度)
    double Latitude;    // Latitude in degrees, ranging from -90 to 90,(度)
    double height; // WGS-84 ellipsoid height in meters,(米)
};

struct EulerAngle {
  double yaw;
  double pitch;
  double roll;
};

typedef enum {
  BOTH_AVAILABLE = 0,
  LEFT_AVAILABLE,
  RIGHT_AVAILABLE,
  BOTH_MISSING
} LaneStatusEx;

typedef enum {
  LEFT = 0,
  RIGHT
} LineDirection;

enum FusionSource {
  CAMERA_ONLY = 1,
  RADAR_ONLY,
  FUSION
};

enum ObstacleIntentType {
  COMMON = 0,
  CUT_IN = 1
};
}  // namespace planning
