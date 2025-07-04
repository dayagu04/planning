#include "mrc_brake_decider.h"
#include "func_state_machine_c.h"
#include "planning_context.h"
#include "environmental_model.h"
#include "agent/agent.h"

namespace planning {

MRCBrakeDecider::MRCBrakeDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session),
      config_(config_builder->cast<MRCBrakeDeciderConfig>()) {
  name_ = "MRCBrakeDecider";
}

bool MRCBrakeDecider::Execute() {
  if (!PreCheck()) {
    LOG_ERROR("PreCheck failed\n");
    return false;
  }
  const auto &state_machine =
      session_->environmental_model().get_local_view().function_state_machine_info;
  auto mrc_output = session_->mutable_planning_context()
      ->mutable_mrc_brake_decider_output();

  //if (state_machine.current_state == iflyauto::FunctionalState_MRC) {
  if (iflyauto::FunctionalState_MRC == iflyauto::FunctionalState_MRC) {
    return MRCBrakeProcess();
  }
  has_set_virtual_obs_ = false;
  mrc_output->SetMRCVirtualObsFlag(has_set_virtual_obs_);
  return true;

}

bool MRCBrakeDecider::MRCBrakeProcess() {
  const auto &environmental_model = session_->environmental_model();
  double dis_to_stopline = environmental_model.get_virtual_lane_manager()
                               ->GetEgoDistanceToStopline();

  if (has_set_virtual_obs_ == false) {
    mrc_brake_curv_ = GenerateMRCBrakeCurve();
    double s_mrc_brake_stop = mrc_brake_curv_.Evaluate(0, 30.0);
    double virtual_obs_dis = 0.0;

    if (s_mrc_brake_stop > dis_to_stopline) {
    // brake stop before stopline
      virtual_obs_dis = dis_to_stopline + 3.5;
    } else {
      virtual_obs_dis = s_mrc_brake_stop + 3.5;
    }
    mrc_agent_.set_agent_id(500000);
    mrc_agent_.set_type(agent::AgentType::VIRTUAL);
    mrc_agent_.set_is_tfl_virtual_obs(false);
    const auto &reference_path_ptr = session_->planning_context()
                                  .lane_change_decider_output()
                                  .coarse_planning_info.reference_path;
    if (reference_path_ptr == nullptr) {
      return true;
    }
    const auto &frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
    double ego_start_s = frenet_ego_state.s();

    ReferencePathPoint refpath_pt;
    double obs_set_dis = std::min(reference_path_ptr->get_points().back().path_point.s() - 0.5, ego_start_s + virtual_obs_dis);
    if (reference_path_ptr->get_reference_point_by_lon(
      obs_set_dis, refpath_pt)) {
      mrc_agent_.set_x(refpath_pt.path_point.x());  //几何中心
      mrc_agent_.set_y(refpath_pt.path_point.y());
    } else {
      return true;
    }

    mrc_agent_.set_length(5.0);
    mrc_agent_.set_width(2.0);
    mrc_agent_.set_fusion_source(1);
    mrc_agent_.set_is_static(true);

    mrc_agent_.set_speed(0.0);
    mrc_agent_.set_theta(0.0);
    mrc_agent_.set_accel(0.0);
    mrc_agent_.set_time_range({0.0, 5.0});

    planning::planning_math::Box2d box(
        planning::planning_math::Vec2d(mrc_agent_.x(), mrc_agent_.y()),
        mrc_agent_.theta(), mrc_agent_.length(), mrc_agent_.width());

    mrc_agent_.set_box(box);
    mrc_agent_.set_timestamp_s(0.0);
    mrc_agent_.set_timestamp_us(0.0);
    has_set_virtual_obs_ = true;
  }

  auto *agent_manager = session_->environmental_model()
                            .get_dynamic_world()
                            ->mutable_agent_manager();
  std::unordered_map<int32_t, planning::agent::Agent> agent_table;
  agent_table.insert({mrc_agent_.agent_id(), mrc_agent_});
  agent_manager->Append(agent_table);

  auto mrc_output = session_->mutable_planning_context()
      ->mutable_mrc_brake_decider_output();

  mrc_output->SetMRCVirtualObsFlag(has_set_virtual_obs_);
  return true;
}

void MRCBrakeDecider::SaveToSession() {

}

SecondOrderTimeOptimalTrajectory MRCBrakeDecider::GenerateMRCBrakeCurve()
    const {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& init_point = ego_state_manager->planning_init_point();
  const auto& v_ego = ego_state_manager->ego_v();

  LonState init_state;
  init_state.p = 0;
  init_state.v = init_point.v;
  init_state.a = init_point.a;

  StateLimit state_limit;
  double acc_lower_bound = config_.mrc_brake_deceleration;
  if (init_state.a < config_.mrc_brake_deceleration) {
    acc_lower_bound = init_state.a;
  }

  const double jerk_lower_bound = -2.0;

  state_limit.a_max = 1.0;
  state_limit.a_min = acc_lower_bound;
  state_limit.j_max = 2.0;
  state_limit.j_min = jerk_lower_bound;
  return SecondOrderTimeOptimalTrajectory(init_state, state_limit);
}

}
