#include "hybrid_astar_perpendicular_tail_in_path_generator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <vector>

#include "aabb2d.h"
#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_slot_manager.h"
#include "base_collision_detector.h"
#include "common_math.h"
#include "common_platform_type_soc.h"
#include "curve_node.h"
#include "geometry_math.h"
#include "grid_search.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_context.h"
#include "ifly_time.h"
#include "link_pose_line.h"
#include "link_pt_line.h"
#include "log_glog.h"
#include "node3d.h"
#include "parking_scenario.h"
#include "path_compare_decider/path_compare_decider.h"

namespace planning {
namespace apa_planner {

#define DEBUG_PARENT_NODE (0)
#define DEBUG_CHILD_NODE (0)
#define PLOT_CHILD_NODE (0)
#define PLOT_DELETE_NODE (0)
#define PLOT_SEARCH_SEQUENCE (0)
#define PLOT_ALL_SUCCESS_CURVE_PATH (0)
#define PLOT_ALL_SUCCESS_CURVE_PATH_FIRST_GEAR_SWITCH_POSE (0)
#define PLOT_ALL_BEST_CURVE_PATH (1)
#define PLOT_ALL_BEST_CURVE_PATH_FIRST_GEAR_SWITCH_POSE (0)
#define DEBUG_PARENT_NODE_MAX_NUM (10)
#define DEBUG_CHILD_NODE_MAX_NUM (50000)
#define DEBUG_NODE_GEAR_SWITCH_NUMBER (30)
#define ENABLE_OBS_DIST_G_COST (0)
#define ENABLE_HEADING_DIFF_G_COST (0)
#define DEBUG_SUCCESS_CURVE_PATH_INFO (0)
#define DEBUG_ALL_BEST_CURVE_PATH_INFO (0)
#define USE_PATH_COMPARE (0)

const bool ComparePath(const PathCompareCost& cost1,
                       const PathCompareCost& cost2) {
  return cost1.total_cost < cost2.total_cost;
}

void HybridAStarPerpendicularTailInPathGenerator::UpdatePoseBoundary() {
  const float bound = request_.ego_info_under_slot.slot_type == SlotType::SLANT
                          ? SLANT_SLOT_EXTEND_BOUND
                          : 0.0f;
  search_map_boundary_ = MapBound(-2.0f - bound, 20.0f + bound, -20.0f - bound,
                                  20.0f + bound, -M_PIf32, M_PIf32);

  search_map_grid_boundary_.x =
      std::ceil((search_map_boundary_.x_max - search_map_boundary_.x_min) *
                config_.xy_grid_resolution_inv);
  search_map_grid_boundary_.y =
      std::ceil((search_map_boundary_.y_max - search_map_boundary_.y_min) *
                config_.xy_grid_resolution_inv);
  search_map_grid_boundary_.phi =
      std::ceil((search_map_boundary_.phi_max - search_map_boundary_.phi_min) *
                config_.phi_grid_resolution_inv);
}

void HybridAStarPerpendicularTailInPathGenerator::CalcNodeGCost(
    Node3d* current_node, Node3d* next_node) {
  if (current_node == nullptr || next_node == nullptr) {
    ILOG_ERROR << "current_node or next_node is nullptr when CalcNodeGCost";
    return;
  }

  float length_cost = 0.0f, heading_cost = 0.0f, gear_change_cost = 0.0f,
        kappa_cost = 0.0f, kappa_change_cost = 0.0f, safe_dist_cost = 0.0f,
        exceed_interseting_area_cost = 0.0f,
        exceed_cul_de_sac_limit_pos_cost = 0.0f, borrow_slot_cost = 0.0f;

  // length cost
  length_cost = next_node->GetNodePathDistance() * config_.traj_forward_penalty;

  // gear change cost
  if (current_node->IsPathGearChange(next_node->GetGearType())) {
    gear_change_cost = config_.gear_switch_penalty;
  }

  // steer cost and steer change cost
  if (current_node->GetPathType() != AstarPathType::START_NODE) {
    // start node the steer can be any value
    kappa_cost = config_.traj_kappa_penalty * std::fabs(next_node->GetKappa());
    if (!current_node->IsPathGearChange(next_node->GetGearType())) {
      kappa_change_cost =
          config_.traj_kappa_change_penalty *
          std::fabs(next_node->GetKappa() - current_node->GetKappa());
    }
  }

  // expect ref gear cost
  if (current_node->GetPathType() == AstarPathType::START_NODE) {
    if (next_node->IsPathGearChange(request_.inital_action_request.ref_gear)) {
      gear_change_cost += config_.expect_gear_penalty;
    }
    if ((next_node->GetKappa() > 0.001f &&
         request_.inital_action_request.ref_steer == AstarPathSteer::RIGHT) ||
        (next_node->GetKappa() < -0.001f &&
         request_.inital_action_request.ref_steer == AstarPathSteer::LEFT)) {
      kappa_cost += config_.expect_steer_penalty;
    }
  }

  // expect first gear length cost
  if (current_node->GetGearSwitchNum() == 0 &&
      current_node->IsPathGearChange(next_node->GetGearType()) &&
      current_node->GetDistToStart() <
          request_.inital_action_request.ref_length) {
    length_cost += config_.expect_dist_penalty;
  }

  // safe dist cost
  // weight: 15
  // [0-0.15], cost: 1000;
  // [0.15-0.5],cost: (1/dist -2) * weight;
  // [0.5-1000], cost:0;
#if ENABLE_OBS_DIST_G_COST
  const float dist = next_node->GetDistToObs();
  const float weight = 10.0f;

  if (dist > 0.4f) {
    safe_dist_cost = 0.0;
  } else if (dist > 0.15f) {
    safe_dist_cost = (1.0f / dist - 2.5f) * weight;
  } else {
    safe_dist_cost = 100.0;
  }
#endif

// heading cost
#if ENABLE_HEADING_DIFF_G_COST
  heading_cost =
      std::fabs(next_node->GetPhi()) * config_.ref_line_heading_penalty;
#endif

  if (request_.search_mode == SearchMode::FORMAL) {
    if (interesting_area_.width() > 0.01 &&
        !interesting_area_.contain(next_node->GetPose())) {
      exceed_interseting_area_cost = config_.exceed_interseting_area_penalty;
    }

    if (cul_de_sac_info_.is_cul_de_sac) {
      if (cul_de_sac_info_.type == CulDeSacType::RIGHT &&
          next_node->GetPhi() < cul_de_sac_info_.limit_phi) {
        if ((next_node->GetGearType() == AstarPathGear::DRIVE &&
             next_node->GetKappa() < -0.001f) ||
            (next_node->GetGearType() == AstarPathGear::REVERSE &&
             next_node->GetKappa() > 0.001f)) {
          exceed_cul_de_sac_limit_pos_cost =
              config_.exceed_cul_de_sac_limit_pos_penalty;
        }
      } else if (cul_de_sac_info_.type == CulDeSacType::LEFT &&
                 next_node->GetPhi() > cul_de_sac_info_.limit_phi) {
        if ((next_node->GetGearType() == AstarPathGear::DRIVE &&
             next_node->GetKappa() > 0.001f) ||
            (next_node->GetGearType() == AstarPathGear::REVERSE &&
             next_node->GetKappa() < -0.001f)) {
          exceed_cul_de_sac_limit_pos_cost =
              config_.exceed_cul_de_sac_limit_pos_penalty;
        }
      }
    }
  }

  if (fabs(next_node->GetPhi()) * common_math::kRad2DegF > 76.0f &&
      (next_node->GetX() - 0.5 * apa_param.GetParam().car_width) <
          float(request_.ego_info_under_slot.slot.processed_corner_coord_local_
                    .pt_01_mid.x()) -
              0.5) {
    borrow_slot_cost = config_.borrow_slot_penalty;
  }

  next_node->SetGCost(current_node->GetGCost() + length_cost +
                      gear_change_cost + kappa_cost + kappa_change_cost +
                      safe_dist_cost + heading_cost +
                      exceed_interseting_area_cost +
                      exceed_cul_de_sac_limit_pos_cost + borrow_slot_cost);
}

void HybridAStarPerpendicularTailInPathGenerator::CalcNodeHCost(
    Node3d* current_node, Node3d* next_node,
    const AnalyticExpansionRequest& analytic_expansion_request) {
  if (current_node == nullptr || next_node == nullptr) {
    ILOG_ERROR << "current_node or next_node is nullptr when CalcNodeHCost";
    return;
  }

  NodeHeuristicCost cost;

  float dp_cost = 0.0, curve_path_cost = 0.0, euler_dist_cost = 0.0,
        heading_cost = 0.0;

  dp_cost = grid_search_.CheckDpMap(next_node->GetX(), next_node->GetY()) *
            config_.traj_forward_penalty;

  heading_cost = std::fabs(next_node->GetPhiErr(end_node_)) * 2.0 + dp_cost;

  euler_dist_cost = next_node->GetEulerDist(end_node_);

  if (analytic_expansion_request.type ==
      AnalyticExpansionType::LINK_POSE_LINE) {
    curve_path_cost = GenerateHeuristicCostByLPLPath(
        next_node, *analytic_expansion_request.lpl_input, &cost);
  } else if (analytic_expansion_request.type ==
             AnalyticExpansionType::REEDS_SHEEP) {
    curve_path_cost = GenerateHeuristicCostByRsPath(next_node, &cost);
  }

  next_node->SetHeuCost(
      std::max({dp_cost, curve_path_cost, euler_dist_cost, heading_cost}));
}

const bool HybridAStarPerpendicularTailInPathGenerator::Update() {
  ILOG_INFO << "hybrid astar perpendicular tail in update begin";

  const ApaParameters& param = apa_param.GetParam();
  const EgoInfoUnderSlot& ego_info_under_slot = request_.ego_info_under_slot;
  const ParkingLatLonPathBuffer& lat_lon_path_buffer =
      param.lat_lon_path_buffer;

  const float bound = request_.ego_info_under_slot.slot_type == SlotType::SLANT
                          ? SLANT_SLOT_EXTEND_BOUND
                          : 0.0f;

  double search_start_time = IflyTime::Now_ms();

  const geometry_lib::PathPoint& cur_pose = ego_info_under_slot.cur_pose;
  const AnalyticExpansionType type = request_.analytic_expansion_type;
  const bool swap_start_goal = request_.swap_start_goal;
  const float traj_kappa_change_penalty = config_.traj_kappa_change_penalty;

  const Eigen::Vector2d& interesting_pos_slot =
      ego_info_under_slot.slot.processed_corner_coord_local_.pt_01_mid;

  interesting_area_.Reset(interesting_pos_slot);
  interesting_area_.ExtendXupper(6.6 + bound);
  interesting_area_.ExtendXlower(interesting_pos_slot.x() -
                                 ego_info_under_slot.target_pose.GetX() +
                                 bound);
  interesting_area_.ExtendY(9.6 + bound);
  const double extra_val = 0.368;
  interesting_area_.max_[0] =
      std::max(interesting_area_.max_[0],
               ego_info_under_slot.cur_pose.GetX() + extra_val);
  interesting_area_.max_[1] =
      std::max(interesting_area_.max_[1],
               ego_info_under_slot.cur_pose.GetY() + extra_val);
  interesting_area_.min_[0] =
      std::min(interesting_area_.min_[0],
               ego_info_under_slot.cur_pose.GetX() - extra_val);
  interesting_area_.min_[1] =
      std::min(interesting_area_.min_[1],
               ego_info_under_slot.cur_pose.GetY() - extra_val);

  PathColDetBuffer pre_path_col_det_buffer;
  pre_path_col_det_buffer.need_distinguish_outinslot = false;
  pre_path_col_det_buffer.lon_buffer = param.lat_lon_path_buffer.lon_buffer;
  pre_path_col_det_buffer.body_lat_buffer =
      param.lat_lon_path_buffer.body_lat_buffer;
  pre_path_col_det_buffer.mirror_lat_buffer =
      param.lat_lon_path_buffer.mirror_lat_buffer;
  pre_path_col_det_buffer.extra_turn_lat_buffer =
      param.lat_lon_path_buffer.extra_lat_buffer;

  request_.analytic_expansion_type = AnalyticExpansionType::REEDS_SHEEP;
  config_.traj_kappa_change_penalty = 0.0;

  cul_de_sac_info_.Reset();
  do {
    if (!request_.decide_cul_de_sac) {
      break;
    }
    if (ego_info_under_slot.slot_type != SlotType::PERPENDICULAR) {
      break;
    }
    if (ego_info_under_slot.slot_occupied_ratio > 1e-3) {
      break;
    }
    if (std::fabs(cur_pose.GetTheta()) * kRad2Deg < 36) {
      break;
    }
    if (cur_pose.GetY() * cur_pose.GetTheta() > 0.0) {
      break;
    }
    geometry_lib::PathPoint end_pose;
    const float decide_cul_de_sac_y = 7.5f;
    if (cur_pose.GetY() > 0.0) {
      end_pose.SetY(-decide_cul_de_sac_y - bound);
      end_pose.SetTheta(cur_pose.GetTheta());
      end_pose.SetDir(cur_pose.GetTheta());
      cul_de_sac_info_.type = CulDeSacType::RIGHT;
    } else {
      end_pose.SetY(decide_cul_de_sac_y + bound);
      end_pose.SetTheta(cur_pose.GetTheta());
      end_pose.SetDir(cur_pose.GetTheta());
      cul_de_sac_info_.type = CulDeSacType::LEFT;
    }
    const std::vector<double> x_vec{7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0};
    bool find_safe_pos = false;
    for (const double x : x_vec) {
      end_pose.SetX(x);
      if (!col_det_interface_ptr_->GetEDTColDetPtr()
               ->Update(std::vector<geometry_lib::PathPoint>{end_pose},
                        param.lat_lon_path_buffer.body_lat_buffer, 0.0)
               .col_flag) {
        find_safe_pos = true;
        break;
      }
    }

    if (!find_safe_pos) {
      break;
    }

    const geometry_lib::PathPoint real_target_pose =
        ego_info_under_slot.target_pose;
    request_.ego_info_under_slot.target_pose = end_pose;
    end_pose.PrintInfo();

    request_.search_mode = SearchMode::DECIDE_CUL_DE_SAC;
    if (!UpdateOnce(pre_path_col_det_buffer)) {
      ILOG_ERROR << "this is cul-de-sac";
      cul_de_sac_info_.is_cul_de_sac = true;
      if (cul_de_sac_info_.type == CulDeSacType::RIGHT) {
        cul_de_sac_info_.limit_phi = -6.8f * common_math::kDeg2RadF;
        interesting_area_.min_[1] = cul_de_sac_info_.limit_y -
                                    param.wheel_base - param.front_overhanging +
                                    0.5 * param.max_car_width;
      } else if (cul_de_sac_info_.type == CulDeSacType::LEFT) {
        cul_de_sac_info_.limit_phi = 6.8f * common_math::kDeg2RadF;
        interesting_area_.max_[1] = cul_de_sac_info_.limit_y +
                                    param.wheel_base + param.front_overhanging -
                                    0.5 * param.max_car_width;
      }
    } else {
      ILOG_INFO << "this is not cul-de-sac";
    }
    request_.ego_info_under_slot.target_pose = real_target_pose;
  } while (false);

  do {
    if (request_.pre_search_mode == 0) {
      break;
    }
    if (cul_de_sac_info_.is_cul_de_sac) {
      break;
    }

    request_.search_mode = SearchMode::PRE_SEARCH;
    request_.swap_start_goal = true;

    const bool pre_search_success = UpdateOnce(pre_path_col_det_buffer);
    if (request_.pre_search_mode == 2) {
      ILOG_INFO << "only pre search, quit";
      result_.search_consume_time_ms = IflyTime::Now_ms() - search_start_time;
      return pre_search_success;
    }

    if (!pre_search_success) {
      ILOG_ERROR << "pre search failed";
      break;
    } else {
      ILOG_INFO << "pre search success";
      float x_min = std::numeric_limits<float>::max();
      float x_max = std::numeric_limits<float>::lowest();
      float y_min = std::numeric_limits<float>::max();
      float y_max = std::numeric_limits<float>::lowest();
      for (size_t i = 0; i < result_.x_vec_vec.size(); ++i) {
        const std::vector<float>& x_vec = result_.x_vec_vec[i];
        const std::vector<float>& y_vec = result_.y_vec_vec[i];
        for (size_t j = 0; j < x_vec.size(); ++j) {
          const float x = x_vec[j];
          const float y = y_vec[j];
          x_min = std::min(x_min, x);
          x_max = std::max(x_max, x);
          y_min = std::min(y_min, y);
          y_max = std::max(y_max, y);
        }
      }

      const float x_extend = 0.86f, y_extend = 1.86f;
      x_min -= x_extend, x_max += x_extend, y_min -= y_extend,
          y_max += y_extend;
      y_min = std::min(y_min, -3.6f);
      y_max = std::max(y_max, 3.6f);
      cdl::AABB pre_search_box;
      pre_search_box.min_ << x_min - bound, y_min - bound;
      pre_search_box.max_ << x_max + bound, y_max + bound;
      interesting_area_.combine(pre_search_box);
    }

  } while (false);

  ILOG_INFO << "interseting_area";
  interesting_area_.DebugString();

  request_.search_mode = SearchMode::FORMAL;
  request_.analytic_expansion_type = type;
  request_.swap_start_goal = swap_start_goal;
  config_.traj_kappa_change_penalty = traj_kappa_change_penalty;

  search_start_time = IflyTime::Now_ms();
  PathColDetBuffer path_col_det_buffer;
  path_col_det_buffer.need_distinguish_outinslot = true;
  path_col_det_buffer.lon_buffer = lat_lon_path_buffer.lon_buffer;
  path_col_det_buffer.out_slot_body_lat_buffer =
      lat_lon_path_buffer.out_slot_body_lat_buffer;
  path_col_det_buffer.out_slot_mirror_lat_buffer =
      lat_lon_path_buffer.out_slot_mirror_lat_buffer;
  path_col_det_buffer.out_slot_extra_turn_lat_buffer =
      lat_lon_path_buffer.out_slot_extra_turn_lat_buffer;
  path_col_det_buffer.entrance_slot_body_lat_buffer =
      lat_lon_path_buffer.entrance_slot_body_lat_buffer;
  path_col_det_buffer.entrance_slot_mirror_lat_buffer =
      lat_lon_path_buffer.entrance_slot_mirror_lat_buffer;
  path_col_det_buffer.entrance_slot_extra_turn_lat_buffer =
      lat_lon_path_buffer.entrance_slot_extra_turn_lat_buffer;
  path_col_det_buffer.in_slot_body_lat_buffer =
      lat_lon_path_buffer.in_slot_body_lat_buffer;
  path_col_det_buffer.in_slot_mirror_lat_buffer =
      lat_lon_path_buffer.in_slot_mirror_lat_buffer;
  path_col_det_buffer.in_slot_extra_turn_lat_buffer =
      lat_lon_path_buffer.in_slot_extra_turn_lat_buffer;

  const bool formal_search_success = UpdateOnce(path_col_det_buffer);

  result_.search_consume_time_ms = IflyTime::Now_ms() - search_start_time;

  result_.search_node_num = node_set_.size();

  ILOG_INFO << "hybrid astar perpendicular tail in update end, time cost(ms): "
            << result_.search_consume_time_ms
            << ", node num: " << result_.search_node_num
            << ", solve number: " << result_.solve_number;

  return formal_search_success;
}

const bool HybridAStarPerpendicularTailInPathGenerator::UpdateOnce(
    const PathColDetBuffer& path_col_det_buffer) {
  const double start_time = IflyTime::Now_ms();
  const ApaParameters& param = apa_param.GetParam();

  ResetNodePool();
  open_pq_.clear();
  node_set_.clear();
  result_.Clear();
  child_node_debug_.clear();
  queue_path_debug_.clear();
  delete_queue_path_debug_.clear();
  all_success_curve_path_debug_.clear();
  all_success_path_first_gear_switch_pose_debug_.clear();
  all_search_node_list_.clear();
  all_curve_node_list_.clear();
  rs_path_h_cost_debug_.clear();
  collision_check_time_ms_ = 0.0;
  rs_try_num_ = 0;
  rs_interpolate_time_ms_ = 0.0;
  rs_time_ms_ = 0.0;
  lpl_try_num_ = 0;
  lpl_interpolate_time_ms_ = 0.0;
  lpl_time_ms_ = 0.0;
  set_curve_path_time_ = 0.0;
  check_node_should_del_time_ = 0.0;
  generate_next_node_time_ = 0.0;

  ILOG_INFO << "hybrid astar perpendicular tail in update once begin";

  UpdatePoseBoundary();

  // NodeGridIndex grid_index;
  // Node3d::CoordinateToGridIndex(
  //     search_map_boundary_.x_max, search_map_boundary_.y_max,
  //     search_map_boundary_.phi_max, &grid_index, search_map_boundary_,
  //     config_);
  // if (!CheckOutOfGridBound(grid_index)) {
  //   ILOG_ERROR << "search boundary out of map boundary" << grid_index.x <<
  //   "
  //   "
  //              << grid_index.y << " " << grid_index.phi;

  //   result_.fail_type = AstarFailType::OUT_OF_BOUND;
  //   return false;
  // }

  // alloc start node
  start_node_ = node_pool_.AllocateNode();
  if (start_node_ == nullptr) {
    ILOG_ERROR << "start node nullptr";
    result_.fail_type = AstarFailType::ALLOCATE_NODE_FAIL;
    return false;
  }

  geometry_lib::PathPoint start_pose = request_.ego_info_under_slot.cur_pose;

  if (request_.swap_start_goal) {
    start_pose = request_.ego_info_under_slot.target_pose;
  }

  start_node_->Set(
      NodePath(start_pose.pos.x(), start_pose.pos.y(), start_pose.heading),
      search_map_boundary_, config_, 0.0);

  start_node_->SetPathType(AstarPathType::START_NODE);
  start_node_->DebugString();
  // alloc end node
  end_node_ = node_pool_.AllocateNode();
  if (end_node_ == nullptr) {
    ILOG_ERROR << "end node nullptr";
    result_.fail_type = AstarFailType::ALLOCATE_NODE_FAIL;
    return false;
  }
  geometry_lib::PathPoint end_pose = request_.ego_info_under_slot.target_pose;
  if (request_.swap_start_goal) {
    end_pose = request_.ego_info_under_slot.cur_pose;
  }
  end_node_->Set(NodePath(end_pose.pos.x(), end_pose.pos.y(), end_pose.heading),
                 search_map_boundary_, config_, 0.0);

  end_node_->SetPathType(AstarPathType::END_NODE);
  end_node_->DebugString();

  NodeDeleteInput node_del_input;
  node_del_input.ego_info_under_slot = request_.ego_info_under_slot;
  node_del_input.map_bound = search_map_boundary_;
  node_del_input.config = config_;
  node_del_input.map_grid_bound = search_map_grid_boundary_;
  node_del_input.scenario_type = request_.scenario_type;
  node_del_input.start_id = start_node_->GetGlobalID();
  node_del_input.max_gear_shift_number = request_.max_gear_shift_number;
  node_del_input.path_col_det_buffer = path_col_det_buffer;
  node_del_input.sample_ds = request_.sample_ds;
  node_del_input.swap_start_goal = request_.swap_start_goal;
  if (request_.search_mode == SearchMode::FORMAL) {
    node_del_input.need_cal_obs_dist = true;
    node_del_input.enable_smart_fold_mirror = request_.enable_smart_fold_mirror;
  } else {
    node_del_input.need_cal_obs_dist = false;
    node_del_input.enable_smart_fold_mirror = false;
  }

  node_delete_decider_.Process(node_del_input);

  NodeDeleteRequest node_del_request;

  // check start node
  node_del_request.current_node = start_node_;
  if (CheckNodeShouldDelete(node_del_request)) {
    ILOG_ERROR << "start node is deleted, reason = "
               << GetNodeDeleteReasonString(
                      node_delete_decider_.GetNodeDeleteReason());
    result_.fail_type = AstarFailType::START_COLLISION;
    return false;
  }

  // check end node
  node_del_request.current_node = end_node_;
  if (CheckNodeShouldDelete(node_del_request)) {
    ILOG_ERROR << "end node is deleted, reason = "
               << GetNodeDeleteReasonString(
                      node_delete_decider_.GetNodeDeleteReason());
    result_.fail_type = AstarFailType::GOAL_COLLISION;
    return false;
  }

  // generate astar dist
  const double dp_start_time = IflyTime::Now_ms();
  grid_search_.GenerateDpMap(end_pose.pos.x(), end_pose.pos.y(),
                             search_map_boundary_);
  ILOG_INFO << "generate dp map consume time = "
            << IflyTime::Now_ms() - dp_start_time << "  ms";

  // load open pq and node set
  start_node_->SetMultiMapIter(
      open_pq_.insert(std::make_pair(0.0, start_node_)));

  node_set_.emplace(start_node_->GetGlobalID(), start_node_);

  Node3d *current_node = nullptr, *next_node_in_pool = nullptr;
  Node3d new_node;
  CurveNode curve_node_to_goal, best_curve_node_to_goal;
  best_curve_node_to_goal.SetFCost(kMaxNodeCost);

  size_t curve_path_success_num = 0, explored_curve_path_num = 0;
  size_t explored_node_num = 0;
  size_t gen_child_node_num = 0, gen_child_node_num_success = 0;

  NodeDeleteReason node_del_reason;

  AstarPathGear gear_request = AstarPathGear::NONE;

  AstarNodeVisitedType vis_type = AstarNodeVisitedType::NOT_VISITED;

  PathCompareDecider path_compare_decider;

  ILOG_INFO << "before main cycle, consume time = "
            << IflyTime::Now_ms() - start_time << "  ms";

  const double search_start_time = IflyTime::Now_ms();
  double search_continue_time = 0.0;

#if USE_LINK_PT_LINE
  ILOG_INFO << "USE LINK PT LINE";
  link_pt_line::LinkPtLineInput<float> lpl_input;
  lpl_input.ref_line.Set(
      common_math::Pos<float>(request_.ego_info_under_slot.tar_line.pA.x(),
                              request_.ego_info_under_slot.tar_line.pA.y()),
      1.0f, float(request_.ego_info_under_slot.tar_line.heading));
#else
  ILOG_INFO << "USE LINK POSE LINE";
  LinkPoseLineInput lpl_input;
  lpl_input.ref_line = request_.ego_info_under_slot.tar_line;
#endif

  lpl_input.min_radius = min_radius_ + 0.068;
  lpl_input.bigger_radius_asssign = min_radius_ + 2.0;
  lpl_input.bigger_radius_no_asssign = min_radius_ + 1.0;
  lpl_input.theta_err = 0.68;
  lpl_input.link_line_start_pt = true;
  lpl_input.reverse_last_line_min_length = 0.45;
  lpl_input.drive_last_line_min_length = 1.68;

  AnalyticExpansionRequest analytic_expansion_request;
  analytic_expansion_request.type = request_.analytic_expansion_type;
  analytic_expansion_request.rs_radius = min_radius_ + 0.2;
  analytic_expansion_request.lpl_input = &lpl_input;

  double find_success_curve_max_time;
  size_t find_success_curve_min_count;
  if (request_.search_mode == SearchMode::DECIDE_CUL_DE_SAC) {
    find_success_curve_min_count = 0;
    find_success_curve_max_time = 10;
    config_.max_search_time_ms = 26;
  } else if (request_.search_mode == SearchMode::PRE_SEARCH) {
    find_success_curve_min_count = 0;
    find_success_curve_max_time = 68;
    config_.max_search_time_ms = 86;
  } else {
    if (request_.replan_reason == ReplanReason::SLOT_CRUISING) {
      find_success_curve_min_count = 0;
      find_success_curve_max_time = 68;
      config_.max_search_time_ms = 9800;
    } else if (request_.replan_reason == ReplanReason::DYNAMIC) {
      // dynamic plan
      find_success_curve_min_count = 18;
      find_success_curve_max_time = 68;
      config_.max_search_time_ms = 100;
    } else if (request_.replan_reason == ReplanReason::PATH_DANGEROUS) {
      find_success_curve_min_count = 68;
      find_success_curve_max_time = param.max_dynamic_plan_proj_dt * 1000 * 0.8;
      config_.max_search_time_ms = param.max_dynamic_plan_proj_dt * 1000;
    } else {
      // static plan
      find_success_curve_min_count = std::min(request_.ref_solve_number, 1080);
      find_success_curve_max_time = 1800;
      config_.max_search_time_ms = 10000;
    }
  }

  ILOG_INFO << "find_success_curve_min_count = " << find_success_curve_min_count
            << " find_success_curve_max_time = " << find_success_curve_max_time
            << " config_.max_search_time_ms = " << config_.max_search_time_ms;

  const size_t mb_to_bytes = 1024 * 1024;
  const size_t max_memory_usage = 46.8 * mb_to_bytes;
  const size_t curve_node_memory_usage = 16000;
  size_t memory_usage = 0.0;

  std::vector<CurveNode> curve_node_to_goal_vec;

  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node
    current_node = open_pq_.begin()->second;
    open_pq_.erase(open_pq_.begin());
    if (current_node == nullptr) {
      ILOG_INFO << "pq is null node";
      continue;
    }

    explored_node_num++;

    current_node->SetVisitedType(AstarNodeVisitedType::IN_CLOSE);

#if DEBUG_PARENT_NODE
    if (explored_node_num < DEBUG_PARENT_NODE_MAX_NUM) {
      ILOG_INFO << "============== main cycle =========== ";
      ILOG_INFO << "explored_node_num " << explored_node_num
                << " open set size for now " << open_pq_.size();
      current_node->DebugString();
    }
#endif

#if PLOT_SEARCH_SEQUENCE
    queue_path_debug_.emplace_back(
        Eigen::Vector2d(current_node->GetX(), current_node->GetY()));
#endif

    search_continue_time = IflyTime::Now_ms() - search_start_time;
    if (search_continue_time > config_.max_search_time_ms) {
      ILOG_INFO << "time out and search_continue_time(ms) = "
                << search_continue_time;
      break;
    }

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so,
    // search ends.
    if (analytic_expansion_request.type ==
        AnalyticExpansionType::LINK_POSE_LINE) {
#if USE_LINK_PT_LINE
      lpl_input.ref_last_line_gear = AstarPathGear::REVERSE;
#else
      lpl_input.ref_last_line_gear = geometry_lib::SEG_GEAR_REVERSE;
#endif
      lpl_input.pose.SetPos(current_node->GetX(), current_node->GetY());
      lpl_input.pose.SetTheta(current_node->GetPhi());
      lpl_input.pose.SetDir(current_node->GetPhi());
      lpl_input.sturn_radius = lpl_input.min_radius * 1.5;
      lpl_input.lat_err = 0.03;
      lpl_input.has_length_require = true;
      lpl_input.use_bigger_radius = true;
    }

    if (node_pool_.PoolSize() > 16800 && curve_node_to_goal_vec.size() < 1) {
      lpl_input.use_bigger_radius = false;
      config_.traj_kappa_change_penalty = 0.0;
    }

    analytic_expansion_request.current_node = current_node;
    analytic_expansion_request.curve_node_to_goal = &curve_node_to_goal;
    if (AnalyticExpansion(analytic_expansion_request)) {
#if PLOT_ALL_SUCCESS_CURVE_PATH
      all_success_curve_path_debug_.emplace_back(
          curve_node_to_goal.GetCurvePath());
#endif

#if PLOT_ALL_SUCCESS_CURVE_PATH_FIRST_GEAR_SWITCH_POSE
      all_success_path_first_gear_switch_pose_debug_.emplace_back(
          curve_node_to_goal.GetGearSwitchPose());
#endif

      curve_path_success_num++;
#if DEBUG_SUCCESS_CURVE_PATH_INFO
      ILOG_INFO << "find new curve path to target pose, success num = "
                << curve_path_success_num
                << "  search time = " << search_continue_time << "ms";
#endif

#if PLOT_CHILD_NODE
      // if node is gear switch point, plot it also.
      if (curve_node_to_goal.GearSwitchNode() != nullptr) {
        child_node_debug_.emplace_back(DebugAstarSearchPoint(
            curve_node_to_goal.GearSwitchNode()->GetX(),
            curve_node_to_goal.GearSwitchNode()->GetY(), true, true));
      }
#endif

#if USE_PATH_COMPARE
      if (path_compare_decider.Compare(&request_, &best_curve_node_to_goal,
                                       &curve_node_to_goal)) {
        best_curve_node_to_goal = curve_node_to_goal;
#if DEBUG_SUCCESS_CURVE_PATH_INFO
        ILOG_INFO << "find new best curve path to target pose, the gear "
                     "switch num = "
                  << best_curve_node_to_goal.GetGearSwitchNum();
#endif

        if (analytic_expansion_request.type ==
            AnalyticExpansionType::LINK_POSE_LINE) {
          lpl_path_.path_pt_vec_vec.clear();
          best_curve_node_to_goal.SetLPLPath(lpl_path_);
        }

        curve_node_to_goal_vec.emplace_back(best_curve_node_to_goal);
        memory_usage += curve_node_memory_usage;
      }
#else
      if (analytic_expansion_request.type ==
          AnalyticExpansionType::LINK_POSE_LINE) {
        lpl_path_.ptss.clear();
        curve_node_to_goal.SetLPLPath(lpl_path_);
      }

      curve_node_to_goal_vec.emplace_back(curve_node_to_goal);
      memory_usage += curve_node_memory_usage;
#endif

      if (memory_usage > max_memory_usage) {
        ILOG_INFO << "curve_node_to_goal_vec occupied memory is enough";
        break;
      }

      if (curve_node_to_goal.GetGearSwitchNum() < 1 &&
          curve_node_to_goal.GetLatErr() < 0.02) {
        ILOG_INFO << "find a path that can no shift gear to enter slot";
        break;
      }
    }

    if (curve_path_success_num > find_success_curve_min_count ||
        (curve_path_success_num > 0 &&
         search_continue_time > find_success_curve_max_time)) {
      ILOG_INFO << "curve_path_success_num or search continue time is enough";
      break;
    }

    explored_curve_path_num++;

    for (const CarMotion& car_motion : car_motion_vec) {
      GenerateNextNode(&new_node, current_node, car_motion);
      gen_child_node_num++;

#if DEBUG_CHILD_NODE
      if (gen_child_node_num < DEBUG_CHILD_NODE_MAX_NUM) {
        ILOG_INFO << "~~~~~~~~~~ child node cycle ~~~~~~~~~";
        ILOG_INFO << "open set size " << open_pq_.size()
                  << ", gear change num:" << current_node->GetGearSwitchNum();
        new_node.DebugString();
      }
#endif

      node_del_request.current_node = &new_node;
      node_del_request.parent_node = current_node;
      node_del_request.curve_node = nullptr;
      node_del_request.old_node = nullptr;
      node_del_request.cur_gear = car_motion.gear;

      // When the cur node is the start_node and the path does not allow gear
      // shifting, gear_request needs to be set as the reference gear to
      // restrict the start_node only expanding along the reference gear
      if (current_node->GetPathType() == AstarPathType::START_NODE &&
          request_.max_gear_shift_number == 0 &&
          request_.search_mode == SearchMode::FORMAL) {
        node_del_request.gear_request = request_.inital_action_request.ref_gear;
      } else if (request_.search_mode == SearchMode::DECIDE_CUL_DE_SAC) {
        node_del_request.gear_request = AstarPathGear::DRIVE;
      } else if (request_.search_mode == SearchMode::PRE_SEARCH &&
                 current_node->GetDistToStart() < 3.8f) {
        node_del_request.gear_request = AstarPathGear::DRIVE;
      } else {
        node_del_request.gear_request = gear_request;
      }

      node_del_request.explored_node_num = explored_node_num + 3;
      new_node.SetPathType(AstarPathType::NODE_SEARCHING);
      new_node.SetGearType(car_motion.gear);
      // new_node.SetSteer(car_motion.front_wheel_angle);
      // new_node.SetRadius(car_motion.radius);
      new_node.SetKappa(car_motion.kappa);
      new_node.SetPre(current_node);
      // dist_to_start is the path distance from start_node to the cur_node
      new_node.SetDistToStart(new_node.GetNodePathDistance() +
                              current_node->GetDistToStart());
      // GearSwitchNode only record first gear switch node
      // NextGearSwitchNode only record second gear switch node
      // SingleGearLength resprent every_single_gear_complete_length
      // SingleGearMinLength only resprent min single_gear_complete_length
      if (current_node->IsPathGearChange(car_motion.gear)) {
        new_node.SetGearSwitchNum(current_node->GetGearSwitchNum() + 1);
        if (new_node.GetGearSwitchNum() == 1) {
          new_node.SetGearSwitchNode(current_node);
          new_node.SetNextGearSwitchNode(nullptr);
        } else if (new_node.GetGearSwitchNum() == 2) {
          new_node.SetGearSwitchNode(current_node->GearSwitchNode());
          new_node.SetNextGearSwitchNode(current_node);
        } else {
          new_node.SetGearSwitchNode(current_node->GearSwitchNode());
          new_node.SetNextGearSwitchNode(current_node->NextGearSwitchNode());
        }
        new_node.SetScurveNum(current_node->GetScurveNum());
        new_node.SetSingleGearLength(new_node.GetNodePathDistance());
        new_node.SetSingleGearMinLength(
            std::min(new_node.GetSingleGearLength(),
                     current_node->GetSingleGearMinLength()));
      } else {
        new_node.SetGearSwitchNum(current_node->GetGearSwitchNum());
        new_node.SetGearSwitchNode(current_node->GearSwitchNode());
        new_node.SetNextGearSwitchNode(current_node->NextGearSwitchNode());
        if (IsSteerOpposite(current_node->GetKappa(), new_node.GetKappa())) {
          new_node.SetScurveNum(current_node->GetScurveNum() + 1);
        } else {
          new_node.SetScurveNum(current_node->GetScurveNum());
        }
        new_node.SetSingleGearLength(new_node.GetNodePathDistance() +
                                     current_node->GetSingleGearLength());
        if (new_node.GetGearSwitchNum() == 0) {
          new_node.SetSingleGearMinLength(new_node.GetSingleGearLength());
        } else {
          new_node.SetSingleGearMinLength(
              std::min(new_node.GetSingleGearLength(),
                       current_node->GetSingleGearMinLength()));
        }
      }

      if (CheckNodeShouldDelete(node_del_request)) {
        node_del_reason = node_delete_decider_.GetNodeDeleteReason();
        // PrintNodeDeleteReason(node_del_reason, true);

#if PLOT_DELETE_NODE
        if (node_del_reason != NodeDeleteReason::PARENT_NODE) {
          delete_queue_path_debug_.emplace_back(
              Eigen::Vector2d(new_node.GetX(), new_node.GetY()));
        }
#endif

        if (node_del_reason == NodeDeleteReason::COLLISION) {
#if PLOT_CHILD_NODE
          // if node is unsafe, plot it also.
          int gear_switch_num = new_node.GetGearSwitchNum();
          if (gen_child_node_num < DEBUG_CHILD_NODE_MAX_NUM &&
              gear_switch_num <= DEBUG_NODE_GEAR_SWITCH_NUMBER) {
            child_node_debug_.emplace_back(
                DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), false));
          }
#endif
        }
        continue;
      }

      if (request_.search_mode == SearchMode::DECIDE_CUL_DE_SAC) {
        if (cul_de_sac_info_.type == CulDeSacType::RIGHT) {
          cul_de_sac_info_.limit_y =
              std::min(cul_de_sac_info_.limit_y, new_node.GetY());
        } else if (cul_de_sac_info_.type == CulDeSacType::LEFT) {
          cul_de_sac_info_.limit_y =
              std::max(cul_de_sac_info_.limit_y, new_node.GetY());
        }
      }

      gen_child_node_num_success++;

      if (node_set_.find(new_node.GetGlobalID()) == node_set_.end()) {
        // if the new node id is not in node set, directly allow node
        next_node_in_pool = node_pool_.AllocateNode();
        vis_type = AstarNodeVisitedType::NOT_VISITED;
      } else {
        // if the new node id is already in node set, get node from pool first
        next_node_in_pool = node_set_[new_node.GetGlobalID()];
        vis_type = next_node_in_pool->GetVisitedType();
      }

      // check null, it node pool is already full
      if (next_node_in_pool == nullptr) {
        // ILOG_INFO << "node size = " << node_pool_.PoolSize()
        //           << "  node_set_.find(new_node.GetGlobalID()) == "
        //              "node_set_.end() = "
        //           << (node_set_.find(new_node.GetGlobalID()) ==
        //               node_set_.end());
        // ILOG_INFO << " next node is null";
        continue;
      }

      // update lpl input pose, and make it easier to successfully link
      if (analytic_expansion_request.type ==
          AnalyticExpansionType::LINK_POSE_LINE) {
#if USE_LINK_PT_LINE
        lpl_input.ref_last_line_gear = AstarPathGear::NONE;
#else
        lpl_input.ref_last_line_gear = geometry_lib::SEG_GEAR_INVALID;
#endif
        lpl_input.pose.SetPos(new_node.GetX(), new_node.GetY());
        lpl_input.pose.SetTheta(new_node.GetPhi());
        lpl_input.pose.SetDir(new_node.GetPhi());
        lpl_input.sturn_radius = lpl_input.min_radius;
        lpl_input.has_length_require = false;
        lpl_input.lat_err = 0.009;
        lpl_input.use_bigger_radius = false;
      }

      CalcNodeGCost(current_node, &new_node);

#if DEBUG_CHILD_NODE
      if (gen_child_node_num < DEBUG_CHILD_NODE_MAX_NUM) {
        ILOG_INFO << "  vis type " << static_cast<int>(vis_type)
                  << " new node g " << new_node.GetGCost() << " pool node g "
                  << next_node_in_pool->GetGCost()
                  << " ,safe dist: " << new_node.GetDistToObs();
      }
#endif

      if (vis_type == AstarNodeVisitedType::NOT_VISITED) {
        CalcNodeHCost(current_node, &new_node, analytic_expansion_request);

        *next_node_in_pool = new_node;
        next_node_in_pool->SetFCost();
        next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

        next_node_in_pool->SetMultiMapIter(open_pq_.insert(
            std::make_pair(next_node_in_pool->GetFCost(), next_node_in_pool)));

        node_set_.emplace(next_node_in_pool->GetGlobalID(), next_node_in_pool);

#if DEBUG_CHILD_NODE
        if (gen_child_node_num < DEBUG_CHILD_NODE_MAX_NUM) {
          ILOG_INFO << "old node visit type is NOT_VISITED";
          next_node_in_pool->DebugCost();
        }
#endif
      }

      else if (vis_type == AstarNodeVisitedType::IN_OPEN) {
        // in open set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost()) {
          CalcNodeHCost(current_node, &new_node, analytic_expansion_request);

          open_pq_.erase(next_node_in_pool->GetMultiMapIter());

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_CHILD_NODE
          if (gen_child_node_num < DEBUG_CHILD_NODE_MAX_NUM) {
            ILOG_INFO << "node has in open";
            next_node_in_pool->DebugCost();
          }
#endif
        }
      }

      else if (vis_type == AstarNodeVisitedType::IN_CLOSE) {
        // in close set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost() + 1e-1) {
          node_del_request.old_node = next_node_in_pool;
          if (CheckNodeShouldDelete(node_del_request)) {
#if PLOT_DELETE_NODE
            delete_queue_path_debug_.emplace_back(
                Eigen::Vector2d(new_node.GetX(), new_node.GetY()));
#endif
            continue;
          }

          CalcNodeHCost(current_node, &new_node, analytic_expansion_request);

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_CHILD_NODE
          if (gen_child_node_num < DEBUG_CHILD_NODE_MAX_NUM) {
            ILOG_INFO << "node has in close";
            next_node_in_pool->DebugCost();
          }
#endif

#if PLOT_CHILD_NODE
          int gear_switch_num = new_node.GetGearSwitchNum();
          if (gen_child_node_num < DEBUG_CHILD_NODE_MAX_NUM &&
              gear_switch_num <= DEBUG_NODE_GEAR_SWITCH_NUMBER) {
            child_node_debug_.emplace_back(
                DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), true));
          }
#endif
        }
      }
    }
  }

  ILOG_INFO << "node in pool num = " << node_pool_.PoolSize()
            << " curve_path_success_num = " << curve_path_success_num
            << "  curve_node_to_goal_vec size = "
            << curve_node_to_goal_vec.size()
            << "  curve_node_to_goal_vec memory_usage = "
            << memory_usage / static_cast<double>(mb_to_bytes) << "MB";

  ILOG_INFO << "hybrid astar main cycle search time = "
            << IflyTime::Now_ms() - search_start_time << " ms"
            << " generate_node_time = " << generate_next_node_time_ << " ms"
            << " rs_interpolate_time_ms_ = " << rs_interpolate_time_ms_ << " ms"
            << " rs_time_ms_ = " << rs_time_ms_ << " ms"
            << "  rs_try_num_ = " << rs_try_num_ << " single_rs_time_ms = "
            << rs_time_ms_ / (rs_try_num_ == 0 ? 1 : rs_try_num_) << " ms"
            << " lpl_sample_time = " << lpl_interpolate_time_ms_ << " ms"
            << " lpl_time_ms_ = " << lpl_time_ms_ << " ms"
            << "  lpl_try_num_ = " << lpl_try_num_ << " single_lpl_time_ms = "
            << lpl_time_ms_ / (lpl_try_num_ == 0 ? 1 : lpl_try_num_) << " ms"
            << " set_curve_path_time_ = " << set_curve_path_time_ << " ms"
            << " check_node_time = " << check_node_should_del_time_ << " ms";

  ILOG_INFO << "hybrid astar perpendicular tail in update once end"
            << " slot id = " << request_.ego_info_under_slot.id
            << " slot type = "
            << GetSlotTypeString(request_.ego_info_under_slot.slot_type)
            << "  search_mode = " << GetSearchModeString(request_.search_mode);

  if (curve_node_to_goal_vec.empty()) {
    ILOG_ERROR << "there are no path";
    return false;
  }

  if (request_.search_mode == SearchMode::FORMAL) {
    ChooseBestCurveNode(curve_node_to_goal_vec, analytic_expansion_request.type,
                        node_del_input.need_cal_obs_dist, path_col_det_buffer,
                        best_curve_node_to_goal);
  } else {
    best_curve_node_to_goal = curve_node_to_goal_vec[0];
  }

  return BackwardPassByCurveNode(&best_curve_node_to_goal);
}

void HybridAStarPerpendicularTailInPathGenerator::ChooseBestCurveNode(
    const std::vector<CurveNode>& curve_node_to_goal_vec,
    const AnalyticExpansionType analytic_expansion_type,
    const bool consider_obs_dist, const PathColDetBuffer& safe_buffer,
    CurveNode& best_curve_node_to_goal) {
  const float gear_change_penalty = 10.0f;
  const float length_penalty = 1.0f;
  const float unsuitable_last_line_length_penalty = 1.68f;
  const float kappa_change_penalty = 1.5f * length_penalty / max_kappa_change_;
  const float lat_err_penalty = 16.8f;
  const float heading_err_penalty = 0.0f * common_math::kRad2DegF;

  result_.solve_number = curve_node_to_goal_vec.size();

  // std::map<PathCompareCost, size_t, decltype(ComparePath)*> cost_index_map(
  //     ComparePath);
  std::map<PathCompareCost, size_t> cost_index_map;

  for (size_t i = 0; i < curve_node_to_goal_vec.size(); ++i) {
    PathCompareCost cost;

    ObsToPathDistRelativeSlot obs_dist_relative_slot;
    const CurveNode& temp_node = curve_node_to_goal_vec[i];

    if (IsGearDifferent(request_.inital_action_request.ref_gear,
                        temp_node.GetCurGear())) {
      cost.unexpect_gear_cost = gear_change_penalty;
    }

    if (IsGearSame(request_.inital_action_request.ref_gear,
                   temp_node.GetCurGear()) &&
        IsSteerOpposite(request_.inital_action_request.ref_steer,
                        temp_node.GetCurKappa())) {
      cost.unexpect_steer_cost = kappa_change_penalty;
    }

    cost.gear_change_cost = gear_change_penalty * temp_node.GetGearSwitchNum();

    cost.length_cost = length_penalty * temp_node.GetDistToStart();

    cost.lat_err_cost = temp_node.GetLatErr() * lat_err_penalty;
    cost.heading_err_cost = temp_node.GetThetaErr() * heading_err_penalty;

    if (analytic_expansion_type == AnalyticExpansionType::LINK_POSE_LINE) {
      const auto& lpl_path = temp_node.GetLPLPath();
      if (lpl_path.seg_num < 1) {
        continue;
      }

      if (temp_node.GetPreNode() != nullptr &&
          temp_node.GetPreNode()->GetPathType() ==
              AstarPathType::NODE_SEARCHING) {
        Node3d* pre_node = temp_node.GetPreNode();
        if (!IsGearDifferent(pre_node->GetGearType(), lpl_path.gears[0])) {
          cost.kappa_change_cost +=
              kappa_change_penalty *
              std::fabs(lpl_path.kappas[0] - pre_node->GetKappa());
        }
      }

      cost.kappa_change_cost += lpl_path.kappa_change * kappa_change_penalty;

      const auto last_steer = lpl_path.last_steer;

      const auto last_line_length = lpl_path.last_line_length;

      if (last_line_length < 2e-2f) {
        cost.unsuitable_last_line_length_cost = gear_change_penalty + 1.0f;
      } else {
        const float unsuitable_last_line_length =
            (last_line_length < 1.86f)   ? (1.86f - last_line_length)
            : (last_line_length > 3.68f) ? (last_line_length - 3.68f)
                                         : 0.0f;
        cost.unsuitable_last_line_length_cost =
            unsuitable_last_line_length * unsuitable_last_line_length_penalty;
      }

      // todo: 单段路径长度如果过短是不是也可以施加惩罚
    }

    if (consider_obs_dist) {
      CalcObsDistRelativeSlot(temp_node, obs_dist_relative_slot);
      const float out_slot_straight_diff =
          obs_dist_relative_slot.obs_dist_out_slot_straight -
          safe_buffer.out_slot_body_lat_buffer;

      const float out_slot_turn_diff =
          obs_dist_relative_slot.obs_dist_out_slot_turn -
          safe_buffer.out_slot_body_lat_buffer -
          safe_buffer.out_slot_extra_turn_lat_buffer;

      const float out_slot_diff =
          std::max(std::min(out_slot_straight_diff, out_slot_turn_diff), 0.05f);

      cost.obs_dist =
          std::min(obs_dist_relative_slot.obs_dist_out_slot_straight,
                   obs_dist_relative_slot.obs_dist_out_slot_turn);

      cost.obs_dist_cost = NodeDeleteDecider::CalcObsDistCost(
          out_slot_diff * 100.0f, gear_change_penalty, length_penalty, 10.0f,
          20.0f, 40.0f);
    }

    if (temp_node.GearSwitchNode() != nullptr) {
      const auto& gear_switch_pose = temp_node.GetGearSwitchPose();
      const AstarPathGear cur_gear = temp_node.GetCurGear();

      cost.cur_gear_switch_pose_cost = CalcGearChangePoseCost(
          gear_switch_pose, cur_gear, gear_change_penalty, length_penalty);

      if (std::fabs(temp_node.GearSwitchNode()->GetKappa()) > 0.001f) {
        cost.cur_gear_switch_pose_cost += 0.6f * length_penalty;
      }

      if (request_.adjust_pose) {
        if (cur_gear == AstarPathGear::DRIVE) {
          cost.cur_gear_switch_pose_cost +=
              (request_.inital_action_request.ref_drive_length -
               temp_node.GetCurGearLength()) *
              length_penalty;
        } else {
          cost.cur_gear_switch_pose_cost +=
              (request_.inital_action_request.ref_reverse_length -
               temp_node.GetCurGearLength()) *
              length_penalty;
        }
      }

      if (temp_node.NextGearSwitchNode() != nullptr) {
        const auto& next_gear_switch_pose = temp_node.GetNextGearSwitchPose();
        const AstarPathGear next_gear = ReversePathGear(cur_gear);
        cost.next_gear_switch_pose_cost =
            CalcGearChangePoseCost(next_gear_switch_pose, next_gear,
                                   gear_change_penalty, length_penalty);

        if (next_gear_switch_pose.GetX() < 6.0 &&
            std::fabs(next_gear_switch_pose.GetY()) < 0.3 &&
            std::fabs(next_gear_switch_pose.GetTheta()) *
                    common_math::kRad2DegF <
                30.0f &&
            next_gear_switch_pose.GetY() * next_gear_switch_pose.GetTheta() <
                0.0f) {
        }
      }
    }

    if (request_.adjust_pose) {
      cost.kappa_change_cost = 0.0;
    }

    cost.GetTotalCost();
    cost_index_map.emplace(cost, i);
  }

  if (cost_index_map.size() > 0) {
    best_curve_node_to_goal =
        curve_node_to_goal_vec[cost_index_map.begin()->second];
  }

#if PLOT_ALL_BEST_CURVE_PATH
  all_success_curve_path_debug_.clear();
  all_success_path_first_gear_switch_pose_debug_.clear();
  for (const auto& pair : cost_index_map) {
    if (all_success_curve_path_debug_.size() > 20) {
      break;
    }
    pair.first.PrintInfo();

    curve_node_to_goal_vec[pair.second].GetGearSwitchPose().PrintInfo();
    curve_node_to_goal_vec[pair.second].GetNextGearSwitchPose().PrintInfo();

    // curve_node_to_goal_vec[pair.second].GetLPLPath().PrintInfo();

    all_success_curve_path_debug_.emplace_back(
        curve_node_to_goal_vec[pair.second].GetCurvePath());
    all_success_path_first_gear_switch_pose_debug_.emplace_back(
        curve_node_to_goal_vec[pair.second].GetGearSwitchPose());
  }
#endif
}

const float HybridAStarPerpendicularTailInPathGenerator::CalcGearChangePoseCost(
#if USE_LINK_PT_LINE
    const common_math::PathPt<float>& gear_switch_pose,
#else
    const geometry_lib::PathPoint& gear_switch_pose,
#endif
    AstarPathGear gear, const float gear_switch_penalty,
    const float length_penalty) {

  const auto& target_pose = request_.ego_info_under_slot.target_pose;

#if USE_LINK_PT_LINE
  link_pt_line::LinkPtLineInput<float> lpl_input;
  lpl_input.ref_line.Set(
      common_math::Pos<float>(request_.ego_info_under_slot.tar_line.pA.x(),
                              request_.ego_info_under_slot.tar_line.pA.y()),
      1.0f, float(request_.ego_info_under_slot.tar_line.heading));
#else
  ILOG_INFO << "USE LINK POSE LINE";
  LinkPoseLineInput lpl_input;
  lpl_input.ref_line = request_.ego_info_under_slot.tar_line;
#endif

  lpl_input.min_radius = min_radius_ + 0.068;
  lpl_input.bigger_radius_asssign = min_radius_ + 2.0;
  lpl_input.bigger_radius_no_asssign = min_radius_ + 1.0;
  lpl_input.theta_err = 0.68;
  lpl_input.link_line_start_pt = true;
  lpl_input.reverse_last_line_min_length = 0.45;
  lpl_input.drive_last_line_min_length = 1.68;

#if USE_LINK_PT_LINE
  lpl_input.ref_last_line_gear = AstarPathGear::REVERSE;
#else
  lpl_input.ref_last_line_gear = geometry_lib::SEG_GEAR_REVERSE;
#endif
  lpl_input.pose.SetPos(gear_switch_pose.GetX(), gear_switch_pose.GetY());
  lpl_input.pose.SetTheta(gear_switch_pose.GetTheta());
  lpl_input.pose.SetDir(gear_switch_pose.GetTheta());
  lpl_input.sturn_radius = lpl_input.min_radius * 1.5;
  lpl_input.lat_err = 0.03;
  lpl_input.has_length_require = true;
  lpl_input.use_bigger_radius = true;

  float max_heu_dist = 2.0f * gear_switch_penalty, heu_dist = 0.0f;
  if (lpl_interface_.CalLPLPath(lpl_input)) {
    const auto& lpl_output = lpl_interface_.GetLPLOutput();
    const float gear_change_cost = 10.0f, length_cost = 1.0f;
    const float kappa_change_cost = (2.5f * length_cost / max_kappa_change_);
    float min_cost = std::numeric_limits<float>::infinity();
    uint8_t min_index = 0;
    for (uint8_t i = 0; i < lpl_output.lpl_path_num; i++) {
      float cost = 0.0;
      const auto& lpl_path = lpl_output.lpl_paths[i];
      cost = gear_change_cost * lpl_path.gear_change_num +
             length_cost * lpl_path.total_length +
             kappa_change_cost * lpl_path.kappa_change;
      if (IsGearDifferent(ReversePathGear(gear), lpl_path.cur_gear)) {
        cost += 6.0f * gear_change_cost;
      }
      if (cost < min_cost) {
        min_index = i;
        min_cost = cost;
      }
    }

    lpl_path_ = lpl_output.lpl_paths[min_index];

    const uint8_t gear_change_count = lpl_path_.gear_change_num;
    const float path_length = lpl_path_.total_length;
    const float kappa_change = lpl_path_.kappa_change;

    Eigen::Vector2d heu_pos(target_pose.GetX(), target_pose.GetY());
    if (gear == AstarPathGear::DRIVE) {
      heu_pos.x() = target_pose.GetX() + 2.2;
    } else {
      heu_pos.x() = target_pose.GetX() + 3.8;
    }

    const float heu_pt_dist = PathCompareDecider::CalcHeuristicPointDistance(
        gear_switch_pose.GetPos(), gear_switch_pose.GetTheta(), heu_pos);

    heu_dist = 1.0f * path_length + 10.0f * heu_pt_dist;

    heu_dist = std::min(heu_dist, max_heu_dist);
  } else {
    heu_dist = max_heu_dist;
  }

  float w1 = 0.95f, w2 = 0.01f, w3 = 0.0f;
  if (request_.adjust_pose) {
    w2 = 0.5f;
    w3 = 1.0f;
  }

  float gear_switch_pose_cost =
      length_penalty * heu_dist * w1 +
      std::fabs(gear_switch_pose.GetTheta()) * common_math::kRad2DegF * w2 +
      std::fabs(gear_switch_pose.GetY()) * w3;

  if (col_det_interface_ptr_->GetEDTColDetPtr()
          ->Update
#if USE_LINK_PT_LINE
      (std::vector<common_math::PathPt<float>>{gear_switch_pose}, 0.4f, 0.1f,
       0.1f, gear)
#else
      (std::vector<geometry_lib::PathPoint>{gear_switch_pose}, 0.1, 0.4, false,
       0.5, true, 0.1, temp_node.GetCurGear())
#endif
          .col_flag) {
    gear_switch_pose_cost += 0.5f * length_penalty;
  }

  if (common_math::CalTwoPosDist(gear_switch_pose.pos,
                                 request_.ego_info_under_slot.slot
                                     .processed_corner_coord_local_.pt_center) >
          9.6f ||
      gear_switch_pose.GetX() > target_pose.GetX() + 10.68f ||
      gear_switch_pose.GetX() < target_pose.GetX() + 0.68f) {
    gear_switch_pose_cost += gear_switch_penalty;
  }

  if (gear == AstarPathGear::DRIVE) {
    const float idex_x = target_pose.GetX() + 3.0f;
    if (gear_switch_pose.GetX() < idex_x) {
      gear_switch_pose_cost += (idex_x - gear_switch_pose.GetX()) * 2.0f;
    }
  }

  return gear_switch_pose_cost;
}

}  // namespace apa_planner
}  // namespace planning