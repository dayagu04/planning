#include "lane_change_decider.h"

#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <type_traits>

#include "../../common/planning_gflags.h"
#include "behavior_planners/lane_change_decider/lane_change_state_machine_manager.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "ego_state_manager.h"
#include "lateral_behavior_object_selector.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "reference_path.h"
#include "reference_path_manager.h"
#include "spline_projection.h"
#include "tasks/behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"
#include "tracked_object.h"
#include "utils/pose2d_utils.h"
#include "vehicle_config_context.h"

namespace planning {

namespace {
constexpr double kEps = 1e-6;
}

LaneChangeDecider::LaneChangeDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "LaneChangeDecider";
  config_ = config_builder->cast<ScenarioStateMachineConfig>();
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr =
      // session_->mutable_planning_context()->virtual_lane_manager();
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  lc_lane_mgr_ =
      std::make_shared<LaneChangeLaneManager>(virtual_lane_mgr, session_);
  lc_req_mgr_ = std::make_shared<LaneChangeRequestManager>(
      session_, config_builder, virtual_lane_mgr, lc_lane_mgr_);

  object_selector_ = std::make_shared<ObjectSelector>(config_builder, session_);

  lc_sm_mgr_ = std::make_shared<LaneChangeStateMachineManager>(
      config_builder, session_, lc_req_mgr_, lc_lane_mgr_);

  Init();
}

void LaneChangeDecider::Init() { lc_sm_mgr_->ResetStateMachine(); }

bool LaneChangeDecider::Execute() {
  const bool active = session_->environmental_model().GetVehicleDbwStatus();
  const auto &function_info = session_->environmental_model().function_info();
  const int current_lc_status =
      session_->planning_context().lane_change_decider_output().curr_state;

  get_lane_change_lane_manager()->upload_fix_lane_virtual_id();
  UpdateFixLaneVirtualId();

  if (!session_->planning_context().last_planning_success()) {
    lc_sm_mgr_->ResetStateMachine();
  }

  update_scenario();  // cruise or low speed or ...

  if (!active ||
      function_info.function_mode() == common::DrivingFunctionInfo::ACC) {
    lc_sm_mgr_->ResetStateMachine();
  }

  // if (!UpdateObjectSelector(active)) {
  //   return false;
  // }

  if (active) {
    LOG_DEBUG("[scenario_state_machine] active\n");
    if (scenario_ == SCENARIO_CRUISE) {
      // update lc_req_mgr_
      if (!session_->is_hpp_scene()) {
        if (!lc_req_mgr_->Update(object_selector_, current_lc_status,
                                 session_->environmental_model().IsOnRoute())) {
          return false;
        }
      }
      lc_sm_mgr_->Update();
    } else {
      return false;
    }
  } else {
    LOG_DEBUG("[scenario_state_machine] not active\n");
    if (scenario_ == SCENARIO_CRUISE) {
      lc_sm_mgr_->Update();
    } else {
      return false;
    }
  }
  if (!CheckEgoPosition()) {
    return false;
  }
  return true;
}

void LaneChangeDecider::update_scenario() { scenario_ = SCENARIO_CRUISE; }

bool LaneChangeDecider::CheckEgoPosition() const {
  // Step 1) check reference_path
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;

  if (coarse_planning_info.reference_path == nullptr) {
    LOG_DEBUG("reference_path is null");
    return false;
  }

  // Step 2) check Init state when target_state is CRUISE_CHANGE
  if (coarse_planning_info.target_state == kLaneChangeExecution ||
      coarse_planning_info.target_state == kLaneChangeComplete) {
    auto &reference_path = coarse_planning_info.reference_path;
    auto &frenet_ego_state = reference_path->get_frenet_ego_state();

    auto &ego_boundary = frenet_ego_state.boundary();
    std::vector<double> ego_s_list = {ego_boundary.s_start, ego_boundary.s_end,
                                      frenet_ego_state.s()};
    for (auto s : ego_s_list) {
      ReferencePathPoint refpath_pt{};
      (void)reference_path->get_reference_point_by_lon(s, refpath_pt);
      auto ego_l = frenet_ego_state.l();
      if (ego_l < -refpath_pt.distance_to_right_road_border or
          ego_l > refpath_pt.distance_to_left_road_border) {
        LOG_DEBUG("init_state error");
        return false;
      }
    }
  }

  return true;
}

bool LaneChangeDecider::UpdateObjectSelector(bool active) {
#ifndef CHERY_T26
  if (!session_->is_hpp_scene()) {
    if (active && !object_selector_->update(session_->planning_context()
                                                .lane_change_decider_output()
                                                .curr_state,
                                            session_->planning_context()
                                                .lane_change_decider_output()
                                                .start_move_dist_lane,
                                            false, 80., false, false, false,
                                            false, false, -1)) {
      LOG_DEBUG("object_selector_update fail\n");
      return false;
    } else {
      if (!active) {
        object_selector_->left_alc_car().clear();
        object_selector_->right_alc_car().clear();
        object_selector_->left_alc_car_cnt().clear();
        object_selector_->right_alc_car_cnt().clear();
        LOG_DEBUG("object_selector_ cleared\n");
      } else {
        LOG_DEBUG("object_selector_updated \n");
      }
    }
    LOG_DEBUG("object_selector_update end\n");
  }
#endif

  return true;
}

void LaneChangeDecider::UpdateFixLaneVirtualId() {
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();

  auto &lane_change_decider_output = session_->mutable_planning_context()
                                         ->mutable_lane_change_decider_output();

  int fix_lane_virtual_id = lane_change_decider_output.fix_lane_virtual_id;
  int current_lane_virtual_id = session_->environmental_model()
                                    .get_virtual_lane_manager()
                                    ->current_lane_virtual_id();
  auto fix_reference_path = reference_path_mgr->get_reference_path_by_lane(
      fix_lane_virtual_id, false);
  if (fix_reference_path == nullptr) {
    lc_lane_mgr_->reset_lc_lanes();
    lane_change_decider_output.fix_lane_virtual_id = current_lane_virtual_id;
    lane_change_decider_output.origin_lane_virtual_id = current_lane_virtual_id;
    lane_change_decider_output.target_lane_virtual_id = current_lane_virtual_id;
  }
}
}  // namespace planning
