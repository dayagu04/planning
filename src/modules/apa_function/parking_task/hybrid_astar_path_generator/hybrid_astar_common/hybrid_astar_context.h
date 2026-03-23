#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "apa_context.h"
#include "apa_slot_manager.h"
#include "collision_detector_interface.h"
#include "common_math.h"
#include "curve_node.h"
#include "geometry_math.h"
#include "hybrid_astar_common.h"
#include "link_pose_line.h"
#include "link_pt_line.h"
#include "node3d.h"
#include "rs_path_request.h"
namespace planning {
namespace apa_planner {

#define SLANT_SLOT_EXTEND_BOUND (2.45f)

struct InitalActionRequest {
  AstarPathGear ref_gear = AstarPathGear::NONE;
  AstarPathSteer ref_steer = AstarPathSteer::NONE;
  float ref_length = 0.0;
  // first gear is drive and the min ref length
  float ref_drive_length = 0.0;
  // first gear is reverse and the min ref length
  float ref_reverse_length = 0.0;
};

enum class SearchMode {
  FORMAL,
  PRE_SEARCH,
  DECIDE_CUL_DE_SAC,
};

// include input and some request
struct HybridAStarRequest {
  std::vector<geometry_lib::PathPoint> splicing_pt_vec;
  std::vector<geometry_lib::PathPoint> last_complete_pt_vec;
  std::vector<geometry_lib::PathPoint> ref_path_pt_vec;
  EgoInfoUnderSlot ego_info_under_slot;
  uint8_t replan_reason;
  ParkingScenarioType scenario_type = ParkingScenarioType::SCENARIO_UNKNOWN;
  AnalyticExpansionType analytic_expansion_type =
      AnalyticExpansionType::LINK_POSE_LINE;
  int max_gear_shift_number = 20;
  int max_scurve_number = 5;
  float every_gear_length = 0.0;
  float sample_ds = 0.1;
  bool swap_start_goal = false;

  SearchMode search_mode = SearchMode::FORMAL;

  int pre_search_mode = 1;
  bool decide_cul_de_sac = true;
  bool enable_interesting_search_area = true;

  bool mirror_has_folded_flag = false;
  bool enable_smart_fold_mirror = false;
  bool adjust_pose = false;
  int ref_solve_number = 100000;
  ProcessObsMethod process_obs_method = ProcessObsMethod::DO_NOTHING;
  InitalActionRequest inital_action_request;

  void Clear() {
    splicing_pt_vec.clear();
    last_complete_pt_vec.clear();
    ref_path_pt_vec.clear();
    ego_info_under_slot.Reset();
  }
};

struct HybridAstarResponse {
  // todo, should check response is whether an expected result for request
  HybridAStarRequest request;

  // local frame
  HybridAStarResult result;
  std::vector<AStarPathPoint> cur_gear_path;

  // if published path steering wheel change too much, true.
  // left turn->right turn: true
  // left turn->straight: true
  // Subsequently, this value can be used to determine whether to optimize the
  // path
  // bool kappa_change_too_much;

  void Clear() {
    result.Clear();
    cur_gear_path.clear();
    request.Clear();
  }
};

struct CarMotion {
  float radius;
  float kappa;
  float front_wheel_angle;
  float traj_length;
  AstarPathGear gear;
  AstarPathSegType type;
};

struct AnalyticExpansionRequest {
  AnalyticExpansionType type;

  Node3d* current_node;
  CurveNode* curve_node_to_goal;

  link_pt_line::LinkPtLineInput<float>* lpl_input;

  float rs_radius;
  bool need_rs_dense_point = false;
  bool need_anchor_point = false;
  RSPathRequestType rs_request = RSPathRequestType::GEAR_SWITCH_LESS_THAN_TWICE;

  AnalyticExpansionRequest() = default;
  ~AnalyticExpansionRequest() = default;
};

enum class GradeColDetBufferType : uint8_t {
  STRAIGHT_PATH_IN_SLOT_BUFFER = 0,
  STRAIGHT_PATH_ENTRANCE_SLOT_BUFFER = 1,
  STRAIGHT_PATH_OUT_SLOT_BUFFER = 2,
  TURN_PATH_IN_SLOT_BUFFER = 3,
  TURN_PATH_ENTRANCE_SLOT_BUFFER = 4,
  TURN_PATH_OUT_SLOT_BUFFER = 5,
  MAX_NUMBER = 6,
};

struct GradeBufferPathPts {
  GradeColDetBufferType type;
  std::vector<common_math::PathPt<float>> pts;

  GradeBufferPathPts() = default;
  GradeBufferPathPts(GradeColDetBufferType _type,
                     std::vector<common_math::PathPt<float>> _pts)
      : type(_type), pts(_pts) {
  }
  ~GradeBufferPathPts() = default;
};

struct PathColDetBuffer {
  float out_slot_body_lat_buffer;
  float out_slot_mirror_lat_buffer;
  float out_slot_extra_turn_lat_buffer;
  float entrance_slot_body_lat_buffer;
  float entrance_slot_mirror_lat_buffer;
  float entrance_slot_extra_turn_lat_buffer;
  float in_slot_body_lat_buffer;
  float in_slot_mirror_lat_buffer;
  float in_slot_extra_turn_lat_buffer;
  float body_lat_buffer;
  float mirror_lat_buffer;
  float extra_turn_lat_buffer;
  float lon_buffer;
  bool need_distinguish_outinslot = true;

  PathColDetBuffer() = default;
  PathColDetBuffer(float _lat_buffer_outslot,
                   float _extra_turn_lat_buffer_out_slot,
                   float _lat_buffer_inslot,
                   float _extra_turn_lat_buffer_in_slot, float _lat_buffer,
                   float _extra_turn_lat_buffer, float _lon_buffer,
                   bool _need_distinguish_outinslot = true)
      : out_slot_body_lat_buffer(_lat_buffer_outslot),
        out_slot_extra_turn_lat_buffer(_extra_turn_lat_buffer_out_slot),
        in_slot_body_lat_buffer(_lat_buffer_inslot),
        in_slot_extra_turn_lat_buffer(_extra_turn_lat_buffer_in_slot),
        body_lat_buffer(_lat_buffer),
        extra_turn_lat_buffer(_extra_turn_lat_buffer),
        lon_buffer(_lon_buffer),
        need_distinguish_outinslot(_need_distinguish_outinslot) {}
  ~PathColDetBuffer() = default;
};

struct PathCompareCost {
  float length_cost = 0.0f;
  float lat_err_cost = 0.0f;
  float heading_err_cost = 0.0f;
  float gear_change_cost = 0.0f;
  float kappa_change_cost = 0.0f;
  float unsuitable_last_line_length_cost = 0.0f;
  float unsuitable_single_gear_line_length_cost = 0.0f;
  float obs_dist_cost = 0.0f;
  float obs_dist = 0.0f;
  float cur_gear_switch_pose_cost = 0.0f;
  float next_gear_switch_pose_cost = 0.0f;
  float unexpect_gear_cost = 0.0f;
  float unexpect_steer_cost = 0.0f;
  float total_cost = 0.0f;

  PathCompareCost() = default;
  PathCompareCost(const PathCompareCost&) = default;
  PathCompareCost& operator=(const PathCompareCost&) = default;

  void Reset() {
    length_cost = 0.0f;
    lat_err_cost = 0.0f;
    heading_err_cost = 0.0f;
    gear_change_cost = 0.0f;
    kappa_change_cost = 0.0f;
    unsuitable_last_line_length_cost = 0.0f;
    unsuitable_single_gear_line_length_cost = 0.0f;
    obs_dist_cost = 0.0f;
    obs_dist = 0.0f;
    cur_gear_switch_pose_cost = 0.0f;
    next_gear_switch_pose_cost = 0.0f;
    unexpect_gear_cost = 0.0f;
    unexpect_steer_cost = 0.0f;
    total_cost = 0.0f;
  }

  bool operator<(const PathCompareCost& other) const {
    return total_cost < other.total_cost;
  }

  const float GetTotalCost();

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log)
        << "PathCompareCost:  total_cost = " << total_cost
        << ", length_cost = " << length_cost
        << ", gear_change_cost = " << gear_change_cost
        << ", kappa_change_cost = " << kappa_change_cost
        << ", last_line_cost = " << unsuitable_last_line_length_cost
        << ", obs_dist_cost = " << obs_dist_cost << ", obs_dist = " << obs_dist
        << ", cur_gear_switch_pose_cost = " << cur_gear_switch_pose_cost
        << ", next_gear_switch_pose_cost = " << next_gear_switch_pose_cost;
  }

  ~PathCompareCost() = default;
};

// For vertical slot, there is a distinction between left and right, while for
// parallel slot, the difference lies in front and back
enum class CulDeSacType : uint8_t {
  NON_CUL_DE_SAC = 0,
  LEFT = 1,
  RIGHT = 2,
  FRONT = 3,
  REAR = 4,
};

struct CulDeSacInfo {
  bool is_cul_de_sac = false;
  CulDeSacType type = CulDeSacType::NON_CUL_DE_SAC;
  float limit_x = 0.0f;
  float limit_y = 0.0f;
  float limit_phi = 0.0f;

  CulDeSacInfo() = default;
  ~CulDeSacInfo() = default;

  void Reset() {
    is_cul_de_sac = false;
    type = CulDeSacType::NON_CUL_DE_SAC;
    limit_x = 0.0f;
    limit_y = 0.0f;
    limit_phi = 0.0f;
  }

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log)
        << "CulDeSacInfo: is_cul_de_sac = " << is_cul_de_sac
        << ", type = " << static_cast<int>(type) << ", limit_x = " << limit_x
        << ", limit_y = " << limit_y << ", limit_phi = " << limit_phi;
  }
};

const std::string GetSearchModeString(const SearchMode& search_mode);

}  // namespace apa_planner
}  // namespace planning