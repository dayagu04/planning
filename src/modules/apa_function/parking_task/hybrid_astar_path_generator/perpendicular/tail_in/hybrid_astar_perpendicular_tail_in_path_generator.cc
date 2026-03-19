#include "hybrid_astar_perpendicular_tail_in_path_generator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
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
#include "hybrid_astar_path_generator.h"
#include "ifly_time.h"
#include "link_pose_line.h"
#include "link_pt_line.h"
#include "log_glog.h"
#include "node3d.h"
#include "parking_scenario.h"
#include "path_compare_decider/path_compare_decider.h"

namespace planning {
namespace apa_planner {

#define PLOT_ALL_BEST_CURVE_PATH (0)

void HybridAStarPerpendicularTailInPathGenerator::CalcNodeGCost(
    Node3d* current_node, Node3d* next_node) {
  HybridAStarPathGenerator::CalcNodeGCost(current_node, next_node);
  // float exceed_interseting_area_cost = 0.0f,
  //       exceed_cul_de_sac_limit_pos_cost = 0.0f, borrow_slot_cost = 0.0f;

  // if (request_.search_mode == SearchMode::FORMAL) {
  //   if (interesting_area_.width() > 0.01 &&
  //       !interesting_area_.contain(next_node->GetPose())) {
  //     exceed_interseting_area_cost = config_.exceed_interseting_area_penalty;
  //   }

  //   if (cul_de_sac_info_.is_cul_de_sac) {
  //     if (cul_de_sac_info_.type == CulDeSacType::RIGHT &&
  //         next_node->GetPhi() < cul_de_sac_info_.limit_phi) {
  //       if ((next_node->GetGearType() == AstarPathGear::DRIVE &&
  //            next_node->GetKappa() < -0.001f) ||
  //           (next_node->GetGearType() == AstarPathGear::REVERSE &&
  //            next_node->GetKappa() > 0.001f)) {
  //         exceed_cul_de_sac_limit_pos_cost =
  //             config_.exceed_cul_de_sac_limit_pos_penalty;
  //       }
  //     } else if (cul_de_sac_info_.type == CulDeSacType::LEFT &&
  //                next_node->GetPhi() > cul_de_sac_info_.limit_phi) {
  //       if ((next_node->GetGearType() == AstarPathGear::DRIVE &&
  //            next_node->GetKappa() > 0.001f) ||
  //           (next_node->GetGearType() == AstarPathGear::REVERSE &&
  //            next_node->GetKappa() < -0.001f)) {
  //         exceed_cul_de_sac_limit_pos_cost =
  //             config_.exceed_cul_de_sac_limit_pos_penalty;
  //       }
  //     }
  //   }
  // }

  // if ((next_node->GetX() - 0.5 * apa_param.GetParam().car_width) <
  //     float(request_.ego_info_under_slot.slot.processed_corner_coord_local_
  //               .pt_01_mid.x()) -
  //         0.5f) {
  //   const bool case1 =
  //       request_.scenario_type ==
  //           ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN &&
  //       fabs(next_node->GetPhi()) * common_math::kRad2DegF > 76.0f;
  //   const bool case2 =
  //       request_.scenario_type ==
  //           ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN &&
  //       fabs(next_node->GetPhi()) * common_math::kRad2DegF < 132.0f;
  //   if (case1 || case2) {
  //     borrow_slot_cost = config_.borrow_slot_penalty;
  //   }
  // }

  // next_node->AddGCost(exceed_interseting_area_cost +
  //                     exceed_cul_de_sac_limit_pos_cost + borrow_slot_cost);
}

HybridAStarPerpendicularTailInPathGenerator::SearchConfigSnapshot
HybridAStarPerpendicularTailInPathGenerator::BuildSearchConfigSnapshot() const {
  SearchConfigSnapshot snapshot;
  snapshot.analytic_expansion_type = request_.analytic_expansion_type;
  snapshot.swap_start_goal = request_.swap_start_goal;
  snapshot.traj_kappa_change_penalty = config_.traj_kappa_change_penalty;
  return snapshot;
}

HybridAStarPerpendicularTailInPathGenerator::SearchConfigSnapshot
HybridAStarPerpendicularTailInPathGenerator::PrepareSearchPhases() {
  InitSearchPhaseTimeCost();
  InitInterestingArea();
  const SearchConfigSnapshot snapshot = BuildSearchConfigSnapshot();
  ModifyPreSearchConfig();
  return snapshot;
}

void HybridAStarPerpendicularTailInPathGenerator::ModifyPreSearchConfig() {
  request_.analytic_expansion_type = AnalyticExpansionType::REEDS_SHEEP;
  config_.traj_kappa_change_penalty = 0.0;
}

HybridAStarPerpendicularTailInPathGenerator::PreSearchPhaseOutcome
HybridAStarPerpendicularTailInPathGenerator::HandlePreSearchPhase() {
  ILOG_INFO << "handle pre search phase";
  const PathColDetBuffer pre_path_col_det_buffer =
      BuildPreSearchPathColDetBuffer();

  const double decide_cul_de_sac_start_time = IflyTime::Now_ms();
  TryDecideCulDeSac(pre_path_col_det_buffer);
  search_phase_time_cost_.decide_cul_de_sac_consume_time_ms =
      IflyTime::Now_ms() - decide_cul_de_sac_start_time;

  if (request_.pre_search_mode == 0 || cul_de_sac_info_.is_cul_de_sac) {
    ILOG_INFO << "skip pre search, continue formal search";
    return PreSearchPhaseOutcome::CONTINUE_FORMAL_SEARCH;
  }

  const double pre_search_start_time = IflyTime::Now_ms();
  const bool pre_search_success = RunPreSearch(pre_path_col_det_buffer);
  search_phase_time_cost_.pre_search_consume_time_ms =
      IflyTime::Now_ms() - pre_search_start_time;

  ILOG_INFO << "pre search success = " << pre_search_success;

  if (request_.pre_search_mode == 2) {
    ILOG_INFO << "only pre search, quit";
    return pre_search_success ? PreSearchPhaseOutcome::RETURN_SUCCESS
                              : PreSearchPhaseOutcome::RETURN_FAILURE;
  }

  return PreSearchPhaseOutcome::CONTINUE_FORMAL_SEARCH;
}

void HybridAStarPerpendicularTailInPathGenerator::RestoreFormalSearchConfig(
    const SearchConfigSnapshot& snapshot) {
  request_.search_mode = SearchMode::FORMAL;
  request_.analytic_expansion_type = snapshot.analytic_expansion_type;
  request_.swap_start_goal = snapshot.swap_start_goal;
  config_.traj_kappa_change_penalty = snapshot.traj_kappa_change_penalty;
}

const bool HybridAStarPerpendicularTailInPathGenerator::RunFormalSearch(
    const SearchConfigSnapshot& snapshot) {
  RestoreFormalSearchConfig(snapshot);
  ILOG_INFO << "interseting_area";
  interesting_area_.DebugString();

  return HybridAStarPathGenerator::RunFormalSearch(snapshot);
}

void HybridAStarPerpendicularTailInPathGenerator::InitInterestingArea() {
  const EgoInfoUnderSlot& ego_info_under_slot = request_.ego_info_under_slot;
  const float bound = ego_info_under_slot.slot_type == SlotType::SLANT
                          ? SLANT_SLOT_EXTEND_BOUND
                          : 0.0f;

  const Eigen::Vector2d& interesting_pos_slot =
      ego_info_under_slot.slot.processed_corner_coord_local_.pt_01_mid;

  interesting_area_.Reset(interesting_pos_slot);
  if (request_.scenario_type ==
      ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
    interesting_area_.ExtendXupper(6.6 + bound);
    interesting_area_.ExtendY(9.6 + bound);
  } else if (request_.scenario_type ==
             ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    interesting_area_.ExtendXupper(9.6 + bound);
    interesting_area_.ExtendY(5.6 + bound);
  }

  interesting_area_.ExtendXlower(interesting_pos_slot.x() -
                                 ego_info_under_slot.target_pose.GetX() +
                                 bound);

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
}

PathColDetBuffer
HybridAStarPerpendicularTailInPathGenerator::BuildPreSearchPathColDetBuffer()
    const {
  const ParkingLatLonPathBuffer& lat_lon_path_buffer =
      apa_param.GetParam().lat_lon_path_buffer;

  PathColDetBuffer path_col_det_buffer;
  path_col_det_buffer.need_distinguish_outinslot = false;
  path_col_det_buffer.lon_buffer = lat_lon_path_buffer.lon_buffer;
  path_col_det_buffer.body_lat_buffer = lat_lon_path_buffer.body_lat_buffer;
  path_col_det_buffer.mirror_lat_buffer = lat_lon_path_buffer.mirror_lat_buffer;
  path_col_det_buffer.extra_turn_lat_buffer =
      lat_lon_path_buffer.extra_lat_buffer;
  return path_col_det_buffer;
}

void HybridAStarPerpendicularTailInPathGenerator::TryDecideCulDeSac(
    const PathColDetBuffer& pre_path_col_det_buffer) {
  ILOG_INFO << "try to decide the scenario if cul_de_sac";
  const ApaParameters& param = apa_param.GetParam();
  const EgoInfoUnderSlot& ego_info_under_slot = request_.ego_info_under_slot;
  const geometry_lib::PathPoint& cur_pose = ego_info_under_slot.cur_pose;
  const float bound = ego_info_under_slot.slot_type == SlotType::SLANT
                          ? SLANT_SLOT_EXTEND_BOUND
                          : 0.0f;

  cul_de_sac_info_.Reset();
  do {
    if (request_.scenario_type ==
        ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
      break;
    }
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
    if (cur_pose.GetY() * cur_pose.GetTheta() > 0.0 &&
        std::fabs(cur_pose.GetY()) > 2.168) {
      break;
    }

    geometry_lib::PathPoint end_pose;
    const float decide_cul_de_sac_y = 7.5f;
    if (cur_pose.GetTheta() < -0.001f) {
      end_pose.SetY(-decide_cul_de_sac_y - bound);
      end_pose.SetTheta(cur_pose.GetTheta());
      end_pose.SetDir(cur_pose.GetTheta());
      cul_de_sac_info_.type = CulDeSacType::RIGHT;
    } else if (cur_pose.GetTheta() > 0.001f) {
      end_pose.SetY(decide_cul_de_sac_y + bound);
      end_pose.SetTheta(cur_pose.GetTheta());
      end_pose.SetDir(cur_pose.GetTheta());
      cul_de_sac_info_.type = CulDeSacType::LEFT;
    } else {
      break;
    }

    constexpr double kCandidateXs[] = {7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0};
    bool find_safe_pos = false;
    std::vector<geometry_lib::PathPoint> collision_check_path(1);
    for (const double x : kCandidateXs) {
      end_pose.SetX(x);
      collision_check_path[0] = end_pose;
      if (!col_det_interface_ptr_->GetEDTColDetPtr()
               ->Update(collision_check_path,
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
}

const bool HybridAStarPerpendicularTailInPathGenerator::RunPreSearch(
    const PathColDetBuffer& pre_path_col_det_buffer) {
  request_.search_mode = SearchMode::PRE_SEARCH;
  request_.swap_start_goal = true;

  const bool pre_search_success = UpdateOnce(pre_path_col_det_buffer);
  if (!pre_search_success) {
    return false;
  }

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

  const float bound = request_.ego_info_under_slot.slot_type == SlotType::SLANT
                          ? SLANT_SLOT_EXTEND_BOUND
                          : 0.0f;
  const float x_extend = 0.86f, y_extend = 1.86f;
  x_min -= x_extend;
  x_max += x_extend;
  y_min -= y_extend;
  y_max += y_extend;
  y_min = std::min(y_min, -3.6f);
  y_max = std::max(y_max, 3.6f);

  cdl::AABB pre_search_box;
  pre_search_box.min_ << x_min - bound, y_min - bound;
  pre_search_box.max_ << x_max + bound, y_max + bound;
  interesting_area_.combine(pre_search_box);
  return true;
}

void HybridAStarPerpendicularTailInPathGenerator::ConfigureSearchBudget() {
  const ApaParameters& param = apa_param.GetParam();

  if (request_.search_mode == SearchMode::DECIDE_CUL_DE_SAC) {
    search_budget_.find_success_curve_min_count = 0;
    search_budget_.find_success_curve_max_time = 10;
    config_.max_search_time_ms = 26;
  } else if (request_.search_mode == SearchMode::PRE_SEARCH) {
    search_budget_.find_success_curve_min_count = 0;
    search_budget_.find_success_curve_max_time = 68;
    config_.max_search_time_ms = 86;
  } else {
    if (request_.replan_reason == ReplanReason::SLOT_CRUISING) {
      search_budget_.find_success_curve_min_count = 0;
      search_budget_.find_success_curve_max_time = 68;
      config_.max_search_time_ms = 9800;
    } else if (request_.replan_reason == ReplanReason::DYNAMIC) {
      search_budget_.find_success_curve_min_count = 18;
      search_budget_.find_success_curve_max_time = 68;
      config_.max_search_time_ms = 100;
    } else if (request_.replan_reason == ReplanReason::PATH_DANGEROUS) {
      search_budget_.find_success_curve_min_count = 68;
      search_budget_.find_success_curve_max_time =
          param.max_dynamic_plan_proj_dt * 1000 * 0.8;
      config_.max_search_time_ms = param.max_dynamic_plan_proj_dt * 1000;
    } else if (request_.replan_reason == ReplanReason::DYNAMIC_GEAR_SWITCH) {
      const DynamicGearSwitchConfig& gear_switch_param =
          param.gear_switch_config;
      search_budget_.find_success_curve_min_count =
          std::min(request_.ref_solve_number, 1080);
      search_budget_.find_success_curve_max_time =
          (gear_switch_param.dist_thresh_for_gear_switch_point /
           gear_switch_param.vel_thresh_for_gear_switch_point) *
          1000;
      config_.max_search_time_ms =
          search_budget_.find_success_curve_max_time + 100;
    } else {
      search_budget_.find_success_curve_min_count =
          std::min(request_.ref_solve_number, 1080);
      search_budget_.find_success_curve_max_time = 1800;
      config_.max_search_time_ms = 10000;
    }
  }

  ILOG_INFO << "find_success_curve_min_count = "
            << search_budget_.find_success_curve_min_count
            << " find_success_curve_max_time = "
            << search_budget_.find_success_curve_max_time
            << " config_.max_search_time_ms = " << config_.max_search_time_ms;
}

const bool HybridAStarPerpendicularTailInPathGenerator::Update() {
  const SearchConfigSnapshot search_config_snapshot = PrepareSearchPhases();

  switch (HandlePreSearchPhase()) {
    case PreSearchPhaseOutcome::RETURN_SUCCESS:
      return FinalizeUpdate(true);
    case PreSearchPhaseOutcome::RETURN_FAILURE:
      return FinalizeUpdate(false);
    case PreSearchPhaseOutcome::CONTINUE_FORMAL_SEARCH:
      return FinalizeUpdate(RunFormalSearch(search_config_snapshot));
    default:
      return false;
  };
}

void HybridAStarPerpendicularTailInPathGenerator::ChooseBestCurveNode(
    const std::vector<CurveNode>& curve_node_to_goal_vec,
    CurveNode& best_curve_node_to_goal) {
  const float gear_change_penalty = 10.0f;
  const float length_penalty = 1.0f;
  const float unsuitable_last_line_length_penalty = 1.68f;
  const float kappa_change_penalty = 1.5f * length_penalty / max_kappa_change_;
  const float last_path_kappa_change_penalty =
      1.0f * gear_change_penalty / max_kappa_change_;
  const float lat_err_penalty = 16.8f;
  const float heading_err_penalty = 0.0f * common_math::kRad2DegF;

  result_.solve_number = curve_node_to_goal_vec.size();

  const float cur_theta = request_.ego_info_under_slot.cur_pose.GetTheta();
  const float end_theta = end_node_->GetPhi();
  const float cur_theta_err =
      std::fabs(geometry_lib::NormalizeAngle(cur_theta - end_theta)) *
      common_math::kRad2DegF;

  bool has_best_curve_node = false;
  PathCompareCost best_cost;
#if PLOT_ALL_BEST_CURVE_PATH
  std::vector<std::pair<PathCompareCost, size_t>> ranked_costs;
  ranked_costs.reserve(curve_node_to_goal_vec.size());
#endif

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

    if (request_.analytic_expansion_type ==
        AnalyticExpansionType::LINK_POSE_LINE) {
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

      cost.kappa_change_cost +=
          temp_node.GetLastPathKappaChange() * last_path_kappa_change_penalty;

      const auto last_line_length = lpl_path.last_line_length;

      if (last_line_length < 2e-2f) {
        cost.unsuitable_last_line_length_cost = gear_change_penalty + 1.0f;
      } else {
        float min_last_line_length = 1.86f;
        float max_last_line_length = 3.68f;
        if (request_.scenario_type ==
            ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
          min_last_line_length = 3.0f;
          max_last_line_length = 7.0f;
        }
        const float unsuitable_last_line_length =
            (last_line_length < min_last_line_length)
                ? (min_last_line_length - last_line_length)
            : (last_line_length > max_last_line_length)
                ? (last_line_length - max_last_line_length)
                : 0.0f;
        cost.unsuitable_last_line_length_cost =
            unsuitable_last_line_length * unsuitable_last_line_length_penalty;
      }

      // todo: 单段路径长度如果过短是不是也可以施加惩罚
    }

    if (true) {
      CalcObsDistRelativeSlot(temp_node, obs_dist_relative_slot);
      cost.obs_dist =
          std::min(obs_dist_relative_slot.obs_dist_out_slot_straight,
                   obs_dist_relative_slot.obs_dist_out_slot_turn);

      cost.obs_dist_cost = NodeDeleteDecider::CalcObsDistCost(
          cost.obs_dist * 100.0f, gear_change_penalty, length_penalty, 50.0f,
          70.0f, 100.0f);
    }

    if (temp_node.GearSwitchNode() != nullptr) {
      const auto& gear_switch_pose = temp_node.GetGearSwitchPose();
      const AstarPathGear cur_gear = temp_node.GetCurGear();

      cost.cur_gear_switch_pose_cost = CalcGearChangePoseCost(
          gear_switch_pose, cur_gear, gear_change_penalty, length_penalty);

      const float gear_change_theta = gear_switch_pose.GetTheta();

      const float gear_change_theta_err =
          std::fabs(
              geometry_lib::NormalizeAngle(gear_change_theta - end_theta)) *
          common_math::kRad2DegF;

      if (cur_theta_err > 6.8f && cur_theta * gear_change_theta < -0.0001f) {
        const float theta_err = std::fabs(geometry_lib::NormalizeAngle(
                                    cur_theta - gear_change_theta)) *
                                common_math::kRad2DegF;
        cost.cur_gear_switch_pose_cost += theta_err * length_penalty * 0.368f;
      }

      if (gear_change_theta_err > cur_theta_err) {
        cost.cur_gear_switch_pose_cost +=
            (gear_change_theta_err - cur_theta_err) * 1.68f * length_penalty;
      }

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
#if PLOT_ALL_BEST_CURVE_PATH
    ranked_costs.emplace_back(cost, i);
#endif
    if (!has_best_curve_node || cost < best_cost) {
      best_cost = cost;
      best_curve_node_to_goal = temp_node;
      has_best_curve_node = true;
    }
  }

#if PLOT_ALL_BEST_CURVE_PATH
  std::sort(
      ranked_costs.begin(), ranked_costs.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
  all_success_curve_path_debug_.clear();
  all_success_path_first_gear_switch_pose_debug_.clear();
  for (const auto& pair : ranked_costs) {
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
    const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
    const float gear_switch_penalty, const float length_penalty) {
  const auto& target_pose = request_.ego_info_under_slot.target_pose;

  link_pt_line::LinkPtLineInput<float> lpl_input;
  lpl_input.ref_line.Set(
      common_math::Pos<float>(request_.ego_info_under_slot.tar_line.pA.x(),
                              request_.ego_info_under_slot.tar_line.pA.y()),
      1.0f, float(request_.ego_info_under_slot.tar_line.heading));

  lpl_input.min_radius = min_radius_ + 0.068;
  lpl_input.bigger_radius_asssign = min_radius_ + 2.0;
  lpl_input.bigger_radius_no_asssign = min_radius_ + 1.0;
  lpl_input.theta_err = 0.68;
  lpl_input.link_line_start_pt = true;
  lpl_input.reverse_last_line_min_length = 0.45;
  lpl_input.drive_last_line_min_length = 1.68;

  lpl_input.ref_last_line_gear = AstarPathGear::REVERSE;
  lpl_input.pose.SetPos(gear_switch_pose.GetX(), gear_switch_pose.GetY());
  lpl_input.pose.SetTheta(gear_switch_pose.GetTheta());
  lpl_input.pose.SetDir(gear_switch_pose.GetTheta());
  lpl_input.sturn_radius = lpl_input.min_radius * 1.5;
  lpl_input.lat_err = 0.03;
  lpl_input.has_length_require = true;
  lpl_input.use_bigger_radius = true;

  if (request_.scenario_type ==
      ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    std::swap(lpl_input.reverse_last_line_min_length,
              lpl_input.drive_last_line_min_length);
    lpl_input.ref_last_line_gear = AstarPathGear::DRIVE;
  }

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
    double drive_heu_pos_x = target_pose.GetX() + 2.2;
    double reverse_heu_pos_x = target_pose.GetX() + 3.8;
    if (request_.scenario_type ==
        ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
      std::swap(drive_heu_pos_x, reverse_heu_pos_x);
    }
    if (gear == AstarPathGear::DRIVE) {
      heu_pos.x() = drive_heu_pos_x;
    } else {
      heu_pos.x() = reverse_heu_pos_x;
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
          ->Update(std::vector<common_math::PathPt<float>>{gear_switch_pose},
                   0.4f, 0.1f, 0.1f, gear)
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

  float drive_idel_x = target_pose.GetX() + 4.2f;
  float reverse_idel_x = target_pose.GetX() + 3.0f;

  float min_drive_x = target_pose.GetX() + 2.5f;
  float min_reverse_x = target_pose.GetX() + 1.5f;

  if (request_.scenario_type ==
      ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    std::swap(drive_idel_x, reverse_idel_x);
    std::swap(min_drive_x, min_reverse_x);
  }

  const float idel_x =
      gear == AstarPathGear::DRIVE ? drive_idel_x : reverse_idel_x;

  const float min_x =
      gear == AstarPathGear::DRIVE ? min_drive_x : min_reverse_x;

  if (gear_switch_pose.GetX() < min_x) {
    gear_switch_pose_cost += 2.0f * gear_switch_penalty;
  } else if (gear_switch_pose.GetX() < idel_x) {
    gear_switch_pose_cost += (idel_x - gear_switch_pose.GetX()) * 2.0f;
  }

  return gear_switch_pose_cost;
}

}  // namespace apa_planner
}  // namespace planning