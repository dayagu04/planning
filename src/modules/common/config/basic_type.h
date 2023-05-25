#pragma once

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "common/utils/cartesian_coordinate_system.h"
#include "common/utils/frenet_coordinate_system.h"
#include "lateral_motion_planner.pb.h"
#include "longitudinal_motion_planner.pb.h"
#include "mjson/mjson.hpp"
#include "planning_def.h"
#include "spline.h"

#define DEFAULT_LANE_WIDTH (3.8)

namespace planning {

enum RequestType { NO_CHANGE, LEFT_CHANGE, RIGHT_CHANGE };

enum LooseBoundType { NONE_SIDE, LEFT_SIDE, RIGHT_SIDE, BOTH_SIDE };

// 转向灯拨杆
enum LeverStatus {
  LEVER_STATE_OFF,
  LEVER_STATE_LEFT,
  LEVER_STATE_RIGHT,
  LEVER_STATE_RESERVED1,
  LEVER_STATE_RESERVED2
};

enum LaneBoundaryType {
  MARKING_UNKNOWN = 0,                 // 未知线型
  MARKING_DASHED = 1,                  // 虚线
  MARKING_SOLID = 2,                   // 实线
  MARKING_SHORT_DASHED = 3,            // 短虚线
  MARKING_DOUBLE_DASHED = 4,           // 双虚线
  MARKING_DOUBLE_SOLID = 5,            // 双实线
  MARKING_LEFT_DASHED_RIGHT_SOLID = 6, // 左虚右实线
  MARKING_LEFT_SOLID_RIGHT_DASHED = 7  // 左实右虚线
};
enum MSDLaneType {
  MSD_LANE_TYPE_UNKNOWN = 0,
  MSD_LANE_TYPE_NORMAL = 1,
  MSD_LANE_TYPE_VIRTUAL = 2,
  MSD_LANE_TYPE_PARKING = 3,
  MSD_LANE_TYPE_ACCELERATE = 4,
  MSD_LANE_TYPE_DECELERATE = 5,
  MSD_LANE_TYPE_BUS = 6,
  MSD_LANE_TYPE_EMERGENCY = 7,
  MSD_LANE_TYPE_ACCELERATE_DECELERATE = 8,
  MSD_LANE_TYPE_LEFT_TURN_WAITTING_AREA = 9,
  MSD_LANE_TYPE_NON_MOTOR = 10,
  MSD_LANE_TYPE_RAMP = 11,
};

typedef enum {
  UNKNOWN_POS = -100,
  LEFT_LEFT_POS = -2,
  LEFT_POS = -1,
  CURR_POS = 0,
  RIGHT_POS = 1,
  RIGHT_RIGHT_POS = 2
} LanePosition;
enum RequestSource {
  NO_REQUEST,
  INT_REQUEST,
  MAP_REQUEST,
  ACT_REQUEST,
  ROUTE_REQUEST
};
struct PointLLH {
  double Longitude; // Longitude in degrees, ranging from -180 to 180,(度)
  double Latitude;  // Latitude in degrees, ranging from -90 to 90,(度)
  double height;    // WGS-84 ellipsoid height in meters,(米)
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

enum FusionSource { CAMERA_ONLY = 1, RADAR_ONLY, FUSION };
typedef enum { LEFT = 0, RIGHT } LineDirection;
struct TrackInfo {
  TrackInfo() {}

  TrackInfo(int id, double drel, double vrel)
      : track_id(id), d_rel(drel), v_rel(vrel) {}

  TrackInfo(const TrackInfo &track_info) {
    track_id = track_info.track_id;
    d_rel = track_info.d_rel;
    v_rel = track_info.v_rel;
  }

  TrackInfo &operator=(const TrackInfo &track_info) {
    track_id = track_info.track_id;
    d_rel = track_info.d_rel;
    v_rel = track_info.v_rel;
    return *this;
  }

  void set_value(int id, double drel, double vrel) {
    track_id = id;
    d_rel = drel;
    v_rel = vrel;
  }

  void reset() {
    track_id = -10000;
    d_rel = 0.0;
    v_rel = 0.0;
  }

  int track_id = -10000;
  double d_rel = 0.0;
  double v_rel = 0.0;
};

struct LatBehaviorInfo {
  std::array<std::vector<double>, 2> avd_car_past;
  std::array<std::vector<double>, 2> avd_sp_car_past;
  bool flag_avd;
  double dist_rblane;
};

enum ObstacleIntentType { COMMON = 0, CUT_IN = 1 };
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

struct TrajectorySpline {
  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  pnc::mathlib::spline theta_s_spline;
  pnc::mathlib::spline delta_s_spline;
  pnc::mathlib::spline omega_s_spline;

  pnc::mathlib::spline s_t_spline;
  pnc::mathlib::spline v_t_spline;
  pnc::mathlib::spline a_t_spline;
  pnc::mathlib::spline j_t_spline;
};

struct PlanningResult {
  bool init_flag = false;
  int target_lane_id;
  ScenarioStateEnum target_scenario_state = ROAD_NONE;
  TrajectoryPoints raw_traj_points;
  TrajectoryPoints traj_points;
  TrajectorySpline traj_spline;
  RequestType turn_signal = NO_CHANGE;
  CurvatureInfo curvature_info;
  int use_backup_cnt = 0;
  double timestamp = 0.0;
  bool use_refined_reference_path = false;
  int contingency_trigger_index = 0;
  // std::string extra_json;
  mjson::Json extra_json = mjson::Json(mjson::Json::object());
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

  planning::common::LateralInitState lat_init_state;
  LongitudinalMotionPlanning::InitState lon_init_state;
};

class ReferencePath;

struct CarReferenceInfo {
  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
};
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
  CarReferenceInfo cart_ref_info;
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

typedef enum { NORMAL_ROAD = 1, INTERSECT = 2 } AlgorithmScene;
typedef enum {
  LANE_CHANGE_LEFT = 1,
  LANE_CHANGE_RIGHT = 2,
  LANE_BORROW_LEFT = 4,
  LANE_BORROW_RIGHT = 8,
  INTERSECT_GO_STRAIGHT = 16,
  INTERSECT_TURN_LEFT = 32,
  INTERSECT_TURN_RIGHT = 64,
  INTERSECT_U_TURN = 128,
  LANE_BORROW_IN_NON_MOTORIZED_LANE = 256
} AlgorithmAction;
typedef enum {
  LANE_CHANGE_WAITING = 1,
  LANE_CHANGEING = 2,
  LANE_CHANGE_BACK = 4,
  LANE_BORROWING = 8,
  LANE_BORROW_BACK = 16,
  LANE_BORROW_RETURN = 32,
  LANE_BORROW_SUSPEND = 64
} AlgorithmStatus;
struct LatBehaviorStateMachineOutput {
  int scenario;
  int curr_state;
  int fix_lane_virtual_id;
  int origin_lane_virtual_id;
  int target_lane_virtual_id;
  std::string state_name;
  std::string lc_back_reason = "none";
  std::string lc_invalid_reason = "none";

  int turn_light;
  int map_turn_light;

  bool accident_back;
  bool accident_ahead;
  bool close_to_accident;
  bool should_premove;
  bool should_suspend;
  bool must_change_lane;
  bool behavior_suspend;        // lateral suspend
  std::vector<int> suspend_obs; // lateral suspend

  bool lc_pause;
  int lc_pause_id;
  double tr_pause_l;
  double tr_pause_s;

  bool disable_l;
  bool disable_r;
  bool enable_l;
  bool enable_r;

  bool need_clear_lb_car;
  std::unordered_map<int, int> front_tracks_slow_cnt;
  std::unordered_map<int, int> avd_track_cnt;
  bool enable_lb;
  int enable_id;

  int lc_request;
  int lc_request_source;
  int lc_turn_light;
  std::string act_request_source;

  int lb_request;
  int lb_turn_light;

  bool premovel;
  bool premover;
  double premove_dist;

  bool left_is_faster;
  bool right_is_faster;

  bool neg_left_lb_car;
  bool neg_right_lb_car;
  bool neg_left_alc_car;
  bool neg_right_alc_car;

  std::vector<int> left_lb_car;
  std::vector<int> left_alc_car;
  std::vector<int> right_lb_car;
  std::vector<int> right_alc_car;

  bool enable_alc_car_protection{false};
  std::vector<int> alc_cars_;
  std::vector<TrackInfo> near_cars_target;
  std::vector<TrackInfo> near_cars_origin;
  TrackInfo lc_invalid_track;
  TrackInfo lc_back_track;

  bool is_lc_valid;
  int lc_valid_cnt;
  int lc_back_cnt;
  std::string lc_back_invalid_reason;
};

enum class LatObstacleType { LANE, ROAD, CAR };

struct LatDeciderOutput {
  planning::common::LateralInitState init_state;

  std::vector<std::pair<double, double>> enu_ref_path; // <x, y>
  std::vector<std::pair<double, double>>
      last_enu_ref_path; // pass it by planning context
  std::vector<std::pair<Point2D, Point2D>> path_bounds; // <lower ,upper >
  std::vector<std::pair<Point2D, Point2D>> safe_bounds; // <lower ,upper >
  std::vector<double> enu_ref_theta;
  std::vector<double> last_enu_ref_theta;
  double v_cruise;
};

struct LateralMotionPlanningOutput {
  std::vector<double> time_vec;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> theta_vec;
  std::vector<double> delta_vec;
  std::vector<double> omega_vec;
  std::vector<double> omega_dot_vec;
  std::vector<double> acc_vec;
  std::vector<double> jerk_vec;
  // const ilqr_solver::iLqr::iLqrSolverInfo *solver_info_ptr;
  // ilqr_solver::iLqr::iLqrSolverInfo solver_info;  // to be removed
};

} // namespace planning
