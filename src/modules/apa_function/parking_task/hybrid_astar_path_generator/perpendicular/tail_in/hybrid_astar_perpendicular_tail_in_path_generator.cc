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

namespace {
link_pt_line::LinkPtLineInput<float> BuildGearSwitchPoseLPLInput(
    const HybridAStarRequest& request, const float min_radius,
    const common_math::PathPt<float>& gear_switch_pose) {
  link_pt_line::LinkPtLineInput<float> lpl_input;
  lpl_input.ref_line.Set(
      common_math::Pos<float>(request.ego_info_under_slot.tar_line.pA.x(),
                              request.ego_info_under_slot.tar_line.pA.y()),
      1.0f, float(request.ego_info_under_slot.tar_line.heading));

  lpl_input.min_radius = min_radius + 0.068f;
  lpl_input.bigger_radius_asssign = min_radius + 2.0f;
  lpl_input.bigger_radius_no_asssign = min_radius + 1.0f;
  lpl_input.theta_err = 0.68f;
  lpl_input.link_line_start_pt = true;
  lpl_input.reverse_last_line_min_length = 0.45f;
  lpl_input.drive_last_line_min_length = 1.68f;
  lpl_input.ref_last_line_gear = AstarPathGear::REVERSE;
  lpl_input.pose.SetPos(gear_switch_pose.GetX(), gear_switch_pose.GetY());
  lpl_input.pose.SetTheta(gear_switch_pose.GetTheta());
  lpl_input.pose.SetDir(gear_switch_pose.GetTheta());
  lpl_input.sturn_radius = lpl_input.min_radius * 1.5f;
  lpl_input.lat_err = 0.03f;
  lpl_input.has_length_require = true;
  lpl_input.use_bigger_radius = true;

  if (request.scenario_type ==
      ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    std::swap(lpl_input.reverse_last_line_min_length,
              lpl_input.drive_last_line_min_length);
    lpl_input.ref_last_line_gear = AstarPathGear::DRIVE;
  }

  return lpl_input;
}
}  // namespace

void HybridAStarPerpendicularTailInPathGenerator::CalcNodeGCost(
    Node3d* current_node, Node3d* next_node) {
  HybridAStarPathGenerator::CalcNodeGCost(current_node, next_node);
  float exceed_interseting_area_cost = 0.0f,
        exceed_cul_de_sac_limit_pos_cost = 0.0f, borrow_slot_cost = 0.0f;

  if (request_.search_mode == SearchMode::FORMAL) {
    if (has_interesting_area_) {
      const float next_x = next_node->GetX();
      const float next_y = next_node->GetY();
      if (next_x < interesting_area_min_x_ ||
          next_x > interesting_area_max_x_ ||
          next_y < interesting_area_min_y_ ||
          next_y > interesting_area_max_y_) {
        exceed_interseting_area_cost = config_.exceed_interseting_area_penalty;
      }
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

  if ((next_node->GetX() - 0.5 * apa_param.GetParam().car_width) <
      float(request_.ego_info_under_slot.slot.processed_corner_coord_local_
                .pt_01_mid.x()) -
          0.5f) {
    const bool case1 =
        request_.scenario_type ==
            ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN &&
        fabs(next_node->GetPhi()) * common_math::kRad2DegF > 76.0f;
    const bool case2 =
        request_.scenario_type ==
            ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN &&
        fabs(next_node->GetPhi()) * common_math::kRad2DegF < 132.0f;
    if (case1 || case2) {
      borrow_slot_cost = config_.borrow_slot_penalty;
    }
  }

  next_node->AddGCost(exceed_interseting_area_cost +
                      exceed_cul_de_sac_limit_pos_cost + borrow_slot_cost);
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
  UpdateInterestingAreaCache();
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
      UpdateInterestingAreaCache();
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
  UpdateInterestingAreaCache();
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
      request_.recaluclate_obs_dist = false;
      return FinalizeUpdate(RunFormalSearch(search_config_snapshot));
    default:
      return false;
  };
}

HybridAStarPerpendicularTailInPathGenerator::CurveNodeScoreParam
HybridAStarPerpendicularTailInPathGenerator::BuildCurveNodeScoreParam() const {
  CurveNodeScoreParam score_param;
  score_param.gear_change_penalty = config_.gear_switch_penalty;
  score_param.length_penalty = config_.traj_forward_penalty;
  score_param.unsuitable_last_line_length_penalty = 1.68f;
  score_param.kappa_change_penalty =
      1.5f * score_param.length_penalty / max_kappa_change_;
  score_param.last_path_kappa_change_penalty =
      1.0f * score_param.gear_change_penalty / max_kappa_change_;
  score_param.lat_err_penalty = 16.8f;
  score_param.heading_err_penalty = 0.0f * common_math::kRad2DegF;
  return score_param;
}

void HybridAStarPerpendicularTailInPathGenerator::FillCurveNodeBaseCost(
    const CurveNode& curve_node, const CurveNodeScoreParam& score_param,
    PathCompareCost& cost) const {
  if (IsGearDifferent(request_.inital_action_request.ref_gear,
                      curve_node.GetCurGear())) {
    cost.unexpect_gear_cost = score_param.gear_change_penalty;
  }

  if (IsGearSame(request_.inital_action_request.ref_gear,
                 curve_node.GetCurGear()) &&
      IsSteerOpposite(request_.inital_action_request.ref_steer,
                      curve_node.GetCurKappa())) {
    cost.unexpect_steer_cost = score_param.kappa_change_penalty;
  }

  cost.gear_change_cost =
      score_param.gear_change_penalty * curve_node.GetGearSwitchNum();
  cost.length_cost = score_param.length_penalty * curve_node.GetDistToStart();
  cost.lat_err_cost = curve_node.GetLatErr() * score_param.lat_err_penalty;
  cost.heading_err_cost =
      curve_node.GetThetaErr() * score_param.heading_err_penalty;

  const CurvePath& curve_path = curve_node.GetCurvePath();
  if (curve_path.segment_size <= 0) {
    return;
  }

  const Node3d* pre_node = curve_node.GetPreNode();
  if (pre_node != nullptr &&
      pre_node->GetPathType() == AstarPathType::NODE_SEARCHING &&
      !IsGearDifferent(pre_node->GetGearType(), curve_path.gears[0])) {
    cost.kappa_change_cost +=
        score_param.kappa_change_penalty *
        std::fabs(curve_path.kappas[0] - pre_node->GetKappa());
  }

  cost.kappa_change_cost +=
      curve_path.kappa_change * score_param.kappa_change_penalty;
  cost.kappa_change_cost += curve_node.GetLastPathKappaChange() *
                            score_param.last_path_kappa_change_penalty;

  const float last_line_length =
      curve_path.steers[curve_path.segment_size - 1] == AstarPathSteer::STRAIGHT
          ? curve_path.dists[curve_path.segment_size - 1]
          : 0.0f;
  if (last_line_length < 2e-2f) {
    cost.unsuitable_last_line_length_cost =
        score_param.gear_change_penalty + 1.0f;
    return;
  }

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
      unsuitable_last_line_length *
      score_param.unsuitable_last_line_length_penalty;

  if (request_.adjust_pose) {
    cost.kappa_change_cost = 0.0;
  }
}

void HybridAStarPerpendicularTailInPathGenerator::FillCurveNodeObsDistCost(
    CurveNode& curve_node, const CurveNodeScoreParam& score_param,
    PathCompareCost& cost) {
  ObsToPathDistRelativeSlot obs_dist_relative_slot;
  if (request_.recaluclate_obs_dist) {
    UpdateObsDistRelativeSlot(&curve_node, &obs_dist_relative_slot);
  } else {
    CalcObsDistRelativeSlot(curve_node, obs_dist_relative_slot);
  }

  cost.obs_dist = std::min(obs_dist_relative_slot.obs_dist_out_slot_straight,
                           obs_dist_relative_slot.obs_dist_out_slot_turn);
  cost.obs_dist_cost = NodeDeleteDecider::CalcObsDistCost(
      cost.obs_dist * 100.0f, score_param.gear_change_penalty,
      score_param.length_penalty, 50.0f, 70.0f, 100.0f);
}

void HybridAStarPerpendicularTailInPathGenerator::FillCurveNodeGearSwitchCost(
    const CurveNode& curve_node, const CurveNodeScoreParam& score_param,
    PathCompareCost& cost) {
  if (curve_node.GearSwitchNode() == nullptr) {
    return;
  }

  const auto& gear_switch_pose = curve_node.GetGearSwitchPose();
  const AstarPathGear cur_gear = curve_node.GetCurGear();

  const float cur_theta = request_.ego_info_under_slot.cur_pose.GetTheta();
  const float end_theta = end_node_->GetPhi();
  const float cur_theta_err =
      std::fabs(geometry_lib::NormalizeAngle(cur_theta - end_theta)) *
      common_math::kRad2DegF;

  cost.cur_gear_switch_pose_cost = CalcGearChangePoseCost(
      gear_switch_pose, cur_gear, score_param.gear_change_penalty,
      score_param.length_penalty);

  const float gear_change_theta = gear_switch_pose.GetTheta();
  const float gear_change_theta_err =
      std::fabs(geometry_lib::NormalizeAngle(gear_change_theta - end_theta)) *
      common_math::kRad2DegF;

  if (cur_theta_err > 6.8f && cur_theta * gear_change_theta < -0.0001f) {
    const float theta_err =
        std::fabs(geometry_lib::NormalizeAngle(cur_theta - gear_change_theta)) *
        common_math::kRad2DegF;
    cost.cur_gear_switch_pose_cost +=
        theta_err * score_param.length_penalty * 0.368f;
  }

  if (gear_change_theta_err > cur_theta_err) {
    cost.cur_gear_switch_pose_cost += (gear_change_theta_err - cur_theta_err) *
                                      1.68f * score_param.length_penalty;
  }

  if (std::fabs(curve_node.GearSwitchNode()->GetKappa()) > 0.001f) {
    cost.cur_gear_switch_pose_cost += 0.6f * score_param.length_penalty;
  }

  if (request_.adjust_pose) {
    if (cur_gear == AstarPathGear::DRIVE) {
      cost.cur_gear_switch_pose_cost +=
          (request_.inital_action_request.ref_drive_length -
           curve_node.GetCurGearLength()) *
          score_param.length_penalty;
    } else {
      cost.cur_gear_switch_pose_cost +=
          (request_.inital_action_request.ref_reverse_length -
           curve_node.GetCurGearLength()) *
          score_param.length_penalty;
    }
  }

  if (curve_node.NextGearSwitchNode() != nullptr) {
    const auto& next_gear_switch_pose = curve_node.GetNextGearSwitchPose();
    const AstarPathGear next_gear = ReversePathGear(cur_gear);
    cost.next_gear_switch_pose_cost = CalcGearChangePoseCost(
        next_gear_switch_pose, next_gear, score_param.gear_change_penalty,
        score_param.length_penalty);
  }
}

const float
HybridAStarPerpendicularTailInPathGenerator::CalcGearSwitchPoseHeuDist(
    const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
    const float gear_switch_penalty) {
  const auto& target_pose = request_.ego_info_under_slot.target_pose;
  const float max_heu_dist = 2.0f * gear_switch_penalty;

  auto lpl_input =
      BuildGearSwitchPoseLPLInput(request_, min_radius_, gear_switch_pose);
  if (!lpl_interface_.CalLPLPath(lpl_input)) {
    return max_heu_dist;
  }

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

  const float path_length = lpl_path_.total_length;

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

  float heu_dist = 1.0f * path_length + 10.0f * heu_pt_dist;
  heu_dist = std::min(heu_dist, max_heu_dist);
  return heu_dist;
}

const float
HybridAStarPerpendicularTailInPathGenerator::CalcGearSwitchPoseBaseCost(
    const common_math::PathPt<float>& gear_switch_pose, const float heu_dist,
    const float length_penalty) const {
  float w1 = 0.95f, w2 = 0.01f, w3 = 0.0f;
  if (request_.adjust_pose) {
    w2 = 0.5f;
    w3 = 1.0f;
  }

  return length_penalty * heu_dist * w1 +
         std::fabs(gear_switch_pose.GetTheta()) * common_math::kRad2DegF * w2 +
         std::fabs(gear_switch_pose.GetY()) * w3;
}

const float
HybridAStarPerpendicularTailInPathGenerator::CalcGearSwitchPoseCollisionCost(
    const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
    const float length_penalty) const {
  if (col_det_interface_ptr_->GetEDTColDetPtr()
          ->Update(std::vector<common_math::PathPt<float>>{gear_switch_pose},
                   0.4f, 0.1f, 0.1f, gear)
          .col_flag) {
    return 0.5f * length_penalty;
  }
  return 0.0f;
}

const float
HybridAStarPerpendicularTailInPathGenerator::CalcGearSwitchPoseRangeCost(
    const common_math::PathPt<float>& gear_switch_pose,
    const float gear_switch_penalty) const {
  const auto& target_pose = request_.ego_info_under_slot.target_pose;
  if (common_math::CalTwoPosDist(gear_switch_pose.pos,
                                 request_.ego_info_under_slot.slot
                                     .processed_corner_coord_local_.pt_center) >
          9.6f ||
      gear_switch_pose.GetX() > target_pose.GetX() + 10.68f ||
      gear_switch_pose.GetX() < target_pose.GetX() + 0.68f) {
    return gear_switch_penalty;
  }
  return 0.0f;
}

const float
HybridAStarPerpendicularTailInPathGenerator::CalcGearSwitchPosePreferXCost(
    const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
    const float gear_switch_penalty) const {
  const auto& target_pose = request_.ego_info_under_slot.target_pose;

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
    return 2.0f * gear_switch_penalty;
  } else if (gear_switch_pose.GetX() < idel_x) {
    return (idel_x - gear_switch_pose.GetX()) * 2.0f;
  }

  return 0.0f;
}

const float HybridAStarPerpendicularTailInPathGenerator::CalcGearChangePoseCost(
    const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
    const float gear_switch_penalty, const float length_penalty) {
  const float heu_dist =
      CalcGearSwitchPoseHeuDist(gear_switch_pose, gear, gear_switch_penalty);

  float gear_switch_pose_cost =
      CalcGearSwitchPoseBaseCost(gear_switch_pose, heu_dist, length_penalty);
  gear_switch_pose_cost +=
      CalcGearSwitchPoseCollisionCost(gear_switch_pose, gear, length_penalty);
  gear_switch_pose_cost +=
      CalcGearSwitchPoseRangeCost(gear_switch_pose, gear_switch_penalty);
  gear_switch_pose_cost += CalcGearSwitchPosePreferXCost(gear_switch_pose, gear,
                                                         gear_switch_penalty);

  return gear_switch_pose_cost;
}

}  // namespace apa_planner
}  // namespace planning