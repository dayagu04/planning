#pragma once

#include "base_convert.h"
#include "planning_debug_info.pb.h"

template <class T1>
void convert(T1 &&msg, std::shared_ptr<planning::common::PlanningDebugInfo> &obj, ConvertTypeInfo type) {
  if (type == ConvertTypeInfo::TO_ROS) {
    //convert for planning_debug_info from latest proto to 2.3.6 ros is not supported.
    return;

  } else if (type == ConvertTypeInfo::TO_PROTO) {

    obj->set_timestamp(msg.timestamp);
    obj->set_data_json(msg.data_json);
    // long_ref_path
    planning::common::LonRefPath *long_ref_path = obj->mutable_long_ref_path();
    for (size_t i = 0; i < msg.long_ref_path.t_list.size(); i++) {
        long_ref_path->add_t_list(msg.long_ref_path.t_list[i]);
    }
    for (size_t i = 0; i < msg.long_ref_path.s_refs.size(); i++) {
        planning::common::DoublePair *s_refs = long_ref_path->add_s_refs();
        s_refs->set_first(msg.long_ref_path.s_refs[i].first);
        s_refs->set_second(msg.long_ref_path.s_refs[i].second);
    }
    for (size_t i = 0; i < msg.long_ref_path.ds_refs.size(); i++) {
        planning::common::DoublePair *ds_refs = long_ref_path->add_ds_refs();
        ds_refs->set_first(msg.long_ref_path.ds_refs[i].first);
        ds_refs->set_second(msg.long_ref_path.ds_refs[i].second);
    }
    for (size_t i = 0; i < msg.long_ref_path.bounds.size(); i++) {
        planning::common::WeightedBounds *bounds = long_ref_path->add_bounds();
        for (size_t j = 0; j < msg.long_ref_path.bounds[i].bound.size(); j++) {
        planning::common::WeightedBound *bound = bounds->add_bound();
        bound->set_lower(msg.long_ref_path.bounds[i].bound[j].lower);
        bound->set_upper(msg.long_ref_path.bounds[i].bound[j].upper);
        bound->set_weight(msg.long_ref_path.bounds[i].bound[j].weight);
        planning::common::BoundInfo *bound_info = bound->mutable_bound_info();
        bound_info->set_id(msg.long_ref_path.bounds[i].bound[j].bound_info.id);
        bound_info->set_type(msg.long_ref_path.bounds[i].bound[j].bound_info.type);
        }
    }

    for (size_t i = 0; i < msg.long_ref_path.soft_bounds.size(); i++) {
        planning::common::WeightedBounds *soft_bounds = long_ref_path->add_soft_bounds();
        for (size_t j = 0; j < msg.long_ref_path.soft_bounds[i].bound.size(); j++) {
        planning::common::WeightedBound *bound = soft_bounds->add_bound();
        bound->set_lower(msg.long_ref_path.soft_bounds[i].bound[j].lower);
        bound->set_upper(msg.long_ref_path.soft_bounds[i].bound[j].upper);
        bound->set_weight(msg.long_ref_path.soft_bounds[i].bound[j].weight);
        planning::common::BoundInfo *bound_info = bound->mutable_bound_info();
        bound_info->set_id(msg.long_ref_path.soft_bounds[i].bound[j].bound_info.id);
        bound_info->set_type(msg.long_ref_path.soft_bounds[i].bound[j].bound_info.type);
        }
    }

    for (size_t i = 0; i < msg.long_ref_path.lon_lead_bounds.size(); i++) {
        planning::common::WeightedLonLeadBounds *lon_lead_bounds = long_ref_path->add_lon_lead_bounds();
        for (size_t j = 0; j < msg.long_ref_path.lon_lead_bounds[i].bound.size(); j++) {
        planning::common::WeightedLonLeadBound *bound = lon_lead_bounds->add_bound();
        bound->set_s_lead(msg.long_ref_path.lon_lead_bounds[i].bound[j].s_lead);
        bound->set_v_lead(msg.long_ref_path.lon_lead_bounds[i].bound[j].v_lead);
        bound->set_t_lead(msg.long_ref_path.lon_lead_bounds[i].bound[j].t_lead);
        bound->set_t_ego(msg.long_ref_path.lon_lead_bounds[i].bound[j].t_ego);
        bound->set_weight(msg.long_ref_path.lon_lead_bounds[i].bound[j].weight);
        }
    }

    planning::common::SVBoundary *lon_sv_boundary = long_ref_path->mutable_lon_sv_boundary();
    lon_sv_boundary->set_boundary_type(planning::common::SVBoundary_SVBoundaryType(msg.long_ref_path.lon_sv_boundary.boundary_type));
    for (size_t i = 0; i < msg.long_ref_path.lon_sv_boundary.sv_bounds.size(); i++) {
        planning::common::SVBound *sv_bounds = lon_sv_boundary->add_sv_bounds();
        sv_bounds->set_s(msg.long_ref_path.lon_sv_boundary.sv_bounds[i].s);
        planning::common::Bound *v_bound = sv_bounds->mutable_v_bound();
        v_bound->set_lower(msg.long_ref_path.lon_sv_boundary.sv_bounds[i].v_bound.lower);
        v_bound->set_upper(msg.long_ref_path.lon_sv_boundary.sv_bounds[i].v_bound.upper); 
    }
    
    planning::common::Bounds *lon_bound_v = long_ref_path->mutable_lon_bound_v();
    for (size_t i = 0; i < msg.long_ref_path.lon_bound_v.bound.size(); i++) {
        planning::common::Bound *bound = lon_bound_v->add_bound();
        bound->set_lower(msg.long_ref_path.lon_bound_v.bound[i].lower);
        bound->set_upper(msg.long_ref_path.lon_bound_v.bound[i].upper);
    }
    planning::common::Bounds *lon_bound_a = long_ref_path->mutable_lon_bound_a();
    for (size_t i = 0; i < msg.long_ref_path.lon_bound_a.bound.size(); i++) {
        planning::common::Bound *bound = lon_bound_a->add_bound();
        bound->set_lower(msg.long_ref_path.lon_bound_a.bound[i].lower);
        bound->set_upper(msg.long_ref_path.lon_bound_a.bound[i].upper);
    }
    planning::common::Bounds *lon_bound_jerk = long_ref_path->mutable_lon_bound_jerk();
    for (size_t i = 0; i < msg.long_ref_path.lon_bound_jerk.bound.size(); i++) {
        planning::common::Bound *bound = lon_bound_jerk->add_bound();
        bound->set_lower(msg.long_ref_path.lon_bound_jerk.bound[i].lower);
        bound->set_upper(msg.long_ref_path.lon_bound_jerk.bound[i].upper);
    }
    // lateral_motion_planning_input
    planning::common::LateralPlanningInput *lateral_motion_planning_input = obj->mutable_lateral_motion_planning_input();
    planning::common::LateralInitState *lat_init_state = lateral_motion_planning_input->mutable_init_state();
    lat_init_state->set_x(msg.lateral_motion_planning_input.init_state.x);
    lat_init_state->set_y(msg.lateral_motion_planning_input.init_state.y);
    lat_init_state->set_theta(msg.lateral_motion_planning_input.init_state.theta);
    lat_init_state->set_delta(msg.lateral_motion_planning_input.init_state.delta);
    lat_init_state->set_omega(msg.lateral_motion_planning_input.init_state.omega);
    lat_init_state->set_curv(msg.lateral_motion_planning_input.init_state.curv);
    lat_init_state->set_d_curv(msg.lateral_motion_planning_input.init_state.d_curv);
    for (size_t i = 0; i < msg.lateral_motion_planning_input.ref_x_vec.size(); i++) {
        lateral_motion_planning_input->add_ref_x_vec(msg.lateral_motion_planning_input.ref_x_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.ref_y_vec.size(); i++) {
        lateral_motion_planning_input->add_ref_y_vec(msg.lateral_motion_planning_input.ref_y_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.ref_theta_vec.size(); i++) {
        lateral_motion_planning_input->add_ref_theta_vec(msg.lateral_motion_planning_input.ref_theta_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.last_x_vec.size(); i++) {
        lateral_motion_planning_input->add_last_x_vec(msg.lateral_motion_planning_input.last_x_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.last_y_vec.size(); i++) {
        lateral_motion_planning_input->add_last_y_vec(msg.lateral_motion_planning_input.last_y_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.last_theta_vec.size(); i++) {
        lateral_motion_planning_input->add_last_theta_vec(msg.lateral_motion_planning_input.last_theta_vec[i]);
    }
    //
    lateral_motion_planning_input->set_ref_vel(msg.lateral_motion_planning_input.ref_vel);
    lateral_motion_planning_input->set_curv_factor(msg.lateral_motion_planning_input.curv_factor);
    //
    for (size_t i = 0; i < msg.lateral_motion_planning_input.soft_upper_bound_x0_vec.size(); i++) {
        lateral_motion_planning_input->add_soft_upper_bound_x0_vec(
            msg.lateral_motion_planning_input.soft_upper_bound_x0_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.soft_upper_bound_y0_vec.size(); i++) {
        lateral_motion_planning_input->add_soft_upper_bound_y0_vec(
            msg.lateral_motion_planning_input.soft_upper_bound_y0_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.soft_upper_bound_x1_vec.size(); i++) {
        lateral_motion_planning_input->add_soft_upper_bound_x1_vec(
            msg.lateral_motion_planning_input.soft_upper_bound_x1_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.soft_upper_bound_y1_vec.size(); i++) {
        lateral_motion_planning_input->add_soft_upper_bound_y1_vec(
            msg.lateral_motion_planning_input.soft_upper_bound_y1_vec[i]);
    }
    //
    for (size_t i = 0; i < msg.lateral_motion_planning_input.soft_lower_bound_x0_vec.size(); i++) {
        lateral_motion_planning_input->add_soft_lower_bound_x0_vec(
            msg.lateral_motion_planning_input.soft_lower_bound_x0_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.soft_lower_bound_y0_vec.size(); i++) {
        lateral_motion_planning_input->add_soft_lower_bound_y0_vec(
            msg.lateral_motion_planning_input.soft_lower_bound_y0_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.soft_lower_bound_x1_vec.size(); i++) {
        lateral_motion_planning_input->add_soft_lower_bound_x1_vec(
            msg.lateral_motion_planning_input.soft_lower_bound_x1_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.soft_lower_bound_y1_vec.size(); i++) {
        lateral_motion_planning_input->add_soft_lower_bound_y1_vec(
            msg.lateral_motion_planning_input.soft_lower_bound_y1_vec[i]);
    }
    //
    for (size_t i = 0; i < msg.lateral_motion_planning_input.hard_upper_bound_x0_vec.size(); i++) {
        lateral_motion_planning_input->add_hard_upper_bound_x0_vec(
            msg.lateral_motion_planning_input.hard_upper_bound_x0_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.hard_upper_bound_y0_vec.size(); i++) {
        lateral_motion_planning_input->add_hard_upper_bound_y0_vec(
            msg.lateral_motion_planning_input.hard_upper_bound_y0_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.hard_upper_bound_x1_vec.size(); i++) {
        lateral_motion_planning_input->add_hard_upper_bound_x1_vec(
            msg.lateral_motion_planning_input.hard_upper_bound_x1_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.hard_upper_bound_y1_vec.size(); i++) {
        lateral_motion_planning_input->add_hard_upper_bound_y1_vec(
            msg.lateral_motion_planning_input.hard_upper_bound_y1_vec[i]);
    }
    //
    for (size_t i = 0; i < msg.lateral_motion_planning_input.hard_lower_bound_x0_vec.size(); i++) {
        lateral_motion_planning_input->add_hard_lower_bound_x0_vec(
            msg.lateral_motion_planning_input.hard_lower_bound_x0_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.hard_lower_bound_y0_vec.size(); i++) {
        lateral_motion_planning_input->add_hard_lower_bound_y0_vec(
            msg.lateral_motion_planning_input.hard_lower_bound_y0_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.hard_lower_bound_x1_vec.size(); i++) {
        lateral_motion_planning_input->add_hard_lower_bound_x1_vec(
            msg.lateral_motion_planning_input.hard_lower_bound_x1_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_input.hard_lower_bound_y1_vec.size(); i++) {
        lateral_motion_planning_input->add_hard_lower_bound_y1_vec(
            msg.lateral_motion_planning_input.hard_lower_bound_y1_vec[i]);
    }
    lateral_motion_planning_input->set_acc_bound(msg.lateral_motion_planning_input.acc_bound);
    lateral_motion_planning_input->set_jerk_bound(msg.lateral_motion_planning_input.jerk_bound);
    lateral_motion_planning_input->set_q_ref_x(msg.lateral_motion_planning_input.q_ref_x);
    lateral_motion_planning_input->set_q_ref_y(msg.lateral_motion_planning_input.q_ref_y);
    lateral_motion_planning_input->set_q_ref_theta(msg.lateral_motion_planning_input.q_ref_theta);
    lateral_motion_planning_input->set_q_continuity(msg.lateral_motion_planning_input.q_continuity);
    lateral_motion_planning_input->set_q_acc(msg.lateral_motion_planning_input.q_acc);
    lateral_motion_planning_input->set_q_jerk(msg.lateral_motion_planning_input.q_jerk);
    lateral_motion_planning_input->set_q_snap(msg.lateral_motion_planning_input.q_snap);
    lateral_motion_planning_input->set_q_acc_bound(msg.lateral_motion_planning_input.q_acc_bound);
    lateral_motion_planning_input->set_q_jerk_bound(msg.lateral_motion_planning_input.q_jerk_bound);
    lateral_motion_planning_input->set_q_soft_corridor(msg.lateral_motion_planning_input.q_soft_corridor);
    lateral_motion_planning_input->set_q_hard_corridor(msg.lateral_motion_planning_input.q_hard_corridor);
    for (size_t i = 0; i < msg.lateral_motion_planning_input.control_vec.size(); i++) {
        lateral_motion_planning_input->add_control_vec(msg.lateral_motion_planning_input.control_vec[i]);
    }
    lateral_motion_planning_input->set_complete_follow(msg.lateral_motion_planning_input.complete_follow);
    lateral_motion_planning_input->set_motion_plan_concerned_index(
        msg.lateral_motion_planning_input.motion_plan_concerned_index);
    // lateral_motion_planning_output
    planning::common::LateralPlanningOutput *lateral_motion_planning_output =
        obj->mutable_lateral_motion_planning_output();
    for (size_t i = 0; i < msg.lateral_motion_planning_output.time_vec.size(); i++) {
        lateral_motion_planning_output->add_time_vec(msg.lateral_motion_planning_output.time_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_output.x_vec.size(); i++) {
        lateral_motion_planning_output->add_x_vec(msg.lateral_motion_planning_output.x_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_output.y_vec.size(); i++) {
        lateral_motion_planning_output->add_y_vec(msg.lateral_motion_planning_output.y_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_output.theta_vec.size(); i++) {
        lateral_motion_planning_output->add_theta_vec(msg.lateral_motion_planning_output.theta_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_output.delta_vec.size(); i++) {
        lateral_motion_planning_output->add_delta_vec(msg.lateral_motion_planning_output.delta_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_output.omega_vec.size(); i++) {
        lateral_motion_planning_output->add_omega_vec(msg.lateral_motion_planning_output.omega_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_output.omega_dot_vec.size(); i++) {
        lateral_motion_planning_output->add_omega_dot_vec(msg.lateral_motion_planning_output.omega_dot_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_output.acc_vec.size(); i++) {
        lateral_motion_planning_output->add_acc_vec(msg.lateral_motion_planning_output.acc_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_motion_planning_output.jerk_vec.size(); i++) {
        lateral_motion_planning_output->add_jerk_vec(msg.lateral_motion_planning_output.jerk_vec[i]);
    }
    planning::common::LateralSolverInfo *solver_info = lateral_motion_planning_output->mutable_solver_info();
    solver_info->set_solver_condition(msg.lateral_motion_planning_output.solver_info.solver_condition);
    solver_info->set_cost_size(msg.lateral_motion_planning_output.solver_info.cost_size);
    solver_info->set_iter_count(msg.lateral_motion_planning_output.solver_info.iter_count);
    solver_info->set_init_cost(msg.lateral_motion_planning_output.solver_info.init_cost);
    for (size_t i = 0; i < msg.lateral_motion_planning_output.solver_info.iter_info.size(); i++) {
        planning::common::SolverIterationInfo *iter_info = solver_info->add_iter_info();
        iter_info->set_linesearch_success(msg.lateral_motion_planning_output.solver_info.iter_info[i].linesearch_success);
        iter_info->set_backward_pass_count(
            msg.lateral_motion_planning_output.solver_info.iter_info[i].backward_pass_count);
        iter_info->set_lambda(msg.lateral_motion_planning_output.solver_info.iter_info[i].lambda);
        iter_info->set_cost(msg.lateral_motion_planning_output.solver_info.iter_info[i].cost);
        iter_info->set_dcost(msg.lateral_motion_planning_output.solver_info.iter_info[i].dcost);
        iter_info->set_expect(msg.lateral_motion_planning_output.solver_info.iter_info[i].expect);
        iter_info->set_du_norm(msg.lateral_motion_planning_output.solver_info.iter_info[i].du_norm);
    }
    // longitudinal_motion_planning_input
    planning::common::LongitudinalPlanningInput *longitudinal_motion_planning_input =
        obj->mutable_longitudinal_motion_planning_input();
    planning::common::LongitudinalInitState *lon_init_state = longitudinal_motion_planning_input->mutable_init_state();
    lon_init_state->set_s(msg.longitudinal_motion_planning_input.init_state.s);
    lon_init_state->set_v(msg.longitudinal_motion_planning_input.init_state.v);
    lon_init_state->set_a(msg.longitudinal_motion_planning_input.init_state.a);
    lon_init_state->set_j(msg.longitudinal_motion_planning_input.init_state.j);
    //
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.ref_pos_vec.size(); i++) {
        longitudinal_motion_planning_input->add_ref_pos_vec(msg.longitudinal_motion_planning_input.ref_pos_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.ref_vel_vec.size(); i++) {
        longitudinal_motion_planning_input->add_ref_vel_vec(msg.longitudinal_motion_planning_input.ref_vel_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.soft_pos_max_vec.size(); i++) {
        longitudinal_motion_planning_input->add_soft_pos_max_vec(
            msg.longitudinal_motion_planning_input.soft_pos_max_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.soft_pos_min_vec.size(); i++) {
        longitudinal_motion_planning_input->add_soft_pos_min_vec(
            msg.longitudinal_motion_planning_input.soft_pos_min_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.hard_pos_max_vec.size(); i++) {
        longitudinal_motion_planning_input->add_hard_pos_max_vec(
            msg.longitudinal_motion_planning_input.hard_pos_max_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.hard_pos_min_vec.size(); i++) {
        longitudinal_motion_planning_input->add_hard_pos_min_vec(
            msg.longitudinal_motion_planning_input.hard_pos_min_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.vel_max_vec.size(); i++) {
        longitudinal_motion_planning_input->add_vel_max_vec(msg.longitudinal_motion_planning_input.vel_max_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.vel_min_vec.size(); i++) {
        longitudinal_motion_planning_input->add_vel_min_vec(msg.longitudinal_motion_planning_input.vel_min_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.acc_max_vec.size(); i++) {
        longitudinal_motion_planning_input->add_acc_max_vec(msg.longitudinal_motion_planning_input.acc_max_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.acc_min_vec.size(); i++) {
        longitudinal_motion_planning_input->add_acc_min_vec(msg.longitudinal_motion_planning_input.acc_min_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.jerk_max_vec.size(); i++) {
        longitudinal_motion_planning_input->add_jerk_max_vec(msg.longitudinal_motion_planning_input.jerk_max_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_input.jerk_min_vec.size(); i++) {
        longitudinal_motion_planning_input->add_jerk_min_vec(msg.longitudinal_motion_planning_input.jerk_min_vec[i]);
    }
    //
    longitudinal_motion_planning_input->set_s_stop(msg.longitudinal_motion_planning_input.s_stop);
    longitudinal_motion_planning_input->set_q_ref_pos(msg.longitudinal_motion_planning_input.q_ref_pos);
    longitudinal_motion_planning_input->set_q_ref_vel(msg.longitudinal_motion_planning_input.q_ref_vel);
    longitudinal_motion_planning_input->set_q_acc(msg.longitudinal_motion_planning_input.q_acc);
    longitudinal_motion_planning_input->set_q_jerk(msg.longitudinal_motion_planning_input.q_jerk);
    longitudinal_motion_planning_input->set_q_snap(msg.longitudinal_motion_planning_input.q_snap);
    longitudinal_motion_planning_input->set_q_soft_pos_bound(msg.longitudinal_motion_planning_input.q_soft_pos_bound);
    longitudinal_motion_planning_input->set_q_hard_pos_bound(msg.longitudinal_motion_planning_input.q_hard_pos_bound);
    longitudinal_motion_planning_input->set_q_vel_bound(msg.longitudinal_motion_planning_input.q_vel_bound);
    longitudinal_motion_planning_input->set_q_acc_bound(msg.longitudinal_motion_planning_input.q_acc_bound);
    longitudinal_motion_planning_input->set_q_jerk_bound(msg.longitudinal_motion_planning_input.q_jerk_bound);
    longitudinal_motion_planning_input->set_q_stop_s(msg.longitudinal_motion_planning_input.q_stop_s);
    // longitudinal_motion_planning_output
    planning::common::LongitudinalPlanningOutput *longitudinal_motion_planning_output =
        obj->mutable_longitudinal_motion_planning_output();
    for (size_t i = 0; i < msg.longitudinal_motion_planning_output.time_vec.size(); i++) {
        longitudinal_motion_planning_output->add_time_vec(msg.longitudinal_motion_planning_output.time_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_output.pos_vec.size(); i++) {
        longitudinal_motion_planning_output->add_pos_vec(msg.longitudinal_motion_planning_output.pos_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_output.vel_vec.size(); i++) {
        longitudinal_motion_planning_output->add_vel_vec(msg.longitudinal_motion_planning_output.vel_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_output.acc_vec.size(); i++) {
        longitudinal_motion_planning_output->add_acc_vec(msg.longitudinal_motion_planning_output.acc_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_output.jerk_vec.size(); i++) {
        longitudinal_motion_planning_output->add_jerk_vec(msg.longitudinal_motion_planning_output.jerk_vec[i]);
    }
    for (size_t i = 0; i < msg.longitudinal_motion_planning_output.snap_vec.size(); i++) {
        longitudinal_motion_planning_output->add_snap_vec(msg.longitudinal_motion_planning_output.snap_vec[i]);
    }
    // lat_behavior_common
    planning::common::LatBehaviorCommon *lat_behavior_common = obj->mutable_lat_behavior_common();
    lat_behavior_common->set_fix_lane_virtual_id(msg.lat_behavior_common.fix_lane_virtual_id);
    lat_behavior_common->set_origin_lane_virtual_id(msg.lat_behavior_common.origin_lane_virtual_id);
    lat_behavior_common->set_target_lane_virtual_id(msg.lat_behavior_common.target_lane_virtual_id);
    lat_behavior_common->set_is_lc_valid(msg.lat_behavior_common.is_lc_valid);
    lat_behavior_common->set_lc_valid_cnt(msg.lat_behavior_common.lc_valid_cnt);
    lat_behavior_common->set_lc_invalid_obj_id(msg.lat_behavior_common.lc_invalid_obj_id);
    lat_behavior_common->set_lc_invalid_reason(msg.lat_behavior_common.lc_invalid_reason);
    lat_behavior_common->set_lc_valid_back(msg.lat_behavior_common.lc_valid_back);
    lat_behavior_common->set_lc_back_obj_id(msg.lat_behavior_common.lc_back_obj_id);
    lat_behavior_common->set_lc_back_cnt(msg.lat_behavior_common.lc_back_cnt);
    lat_behavior_common->set_lc_back_reason(msg.lat_behavior_common.lc_back_reason);
    for (size_t i = 0; i < msg.lat_behavior_common.near_car_ids_origin.size(); i++) {
        lat_behavior_common->add_near_car_ids_origin(msg.lat_behavior_common.near_car_ids_origin[i]);
    }
    for (size_t i = 0; i < msg.lat_behavior_common.near_car_ids_target.size(); i++) {
        lat_behavior_common->add_near_car_ids_target(msg.lat_behavior_common.near_car_ids_target[i]);
    }
    lat_behavior_common->set_v_relative_left_lane(msg.lat_behavior_common.v_relative_left_lane);
    lat_behavior_common->set_is_faster_left_lane(msg.lat_behavior_common.is_faster_left_lane);
    lat_behavior_common->set_faster_left_lane_cnt(msg.lat_behavior_common.faster_left_lane_cnt);
    lat_behavior_common->set_v_relative_right_lane(msg.lat_behavior_common.v_relative_right_lane);
    lat_behavior_common->set_is_faster_right_lane(msg.lat_behavior_common.is_faster_right_lane);
    lat_behavior_common->set_faster_right_lane_cnt(msg.lat_behavior_common.faster_right_lane_cnt);
    for (size_t i = 0; i < msg.lat_behavior_common.left_alc_car_ids.size(); i++) {
        lat_behavior_common->add_left_alc_car_ids(msg.lat_behavior_common.left_alc_car_ids[i]);
    }
    for (size_t i = 0; i < msg.lat_behavior_common.right_alc_car_ids.size(); i++) {
        lat_behavior_common->add_right_alc_car_ids(msg.lat_behavior_common.right_alc_car_ids[i]);
    }
    lat_behavior_common->set_is_forbid_left_alc_car(msg.lat_behavior_common.is_forbid_left_alc_car);
    lat_behavior_common->set_is_forbid_right_alc_car(msg.lat_behavior_common.is_forbid_right_alc_car);
    lat_behavior_common->set_is_side_borrow_bicycle_lane(msg.lat_behavior_common.is_side_borrow_bicycle_lane);
    lat_behavior_common->set_is_side_borrow_lane(msg.lat_behavior_common.is_side_borrow_lane);
    lat_behavior_common->set_has_origin_lane(msg.lat_behavior_common.has_origin_lane);
    lat_behavior_common->set_has_target_lane(msg.lat_behavior_common.has_target_lane);
    lat_behavior_common->set_enable_left_lc(msg.lat_behavior_common.enable_left_lc);
    lat_behavior_common->set_enable_right_lc(msg.lat_behavior_common.enable_right_lc);
    lat_behavior_common->set_turn_light(msg.lat_behavior_common.turn_light);
    lat_behavior_common->set_map_turn_light(msg.lat_behavior_common.map_turn_light);
    lat_behavior_common->set_lc_request(msg.lat_behavior_common.lc_request);
    lat_behavior_common->set_lc_request_source(msg.lat_behavior_common.lc_request_source);
    lat_behavior_common->set_lc_turn_light(msg.lat_behavior_common.lc_turn_light);
    lat_behavior_common->set_act_request_source(msg.lat_behavior_common.act_request_source);
    lat_behavior_common->set_lc_back_invalid_reason(msg.lat_behavior_common.lc_back_invalid_reason);
    lat_behavior_common->set_current_state(msg.lat_behavior_common.current_state);
    // vo_lat_behavior_plan
    planning::common::VOLatBehaviorPlan *vo_lat_behavior_plan = obj->mutable_vo_lat_behavior_plan();
    for (size_t i = 0; i < msg.vo_lat_behavior_plan.avoid_car_ids.size(); i++) {
        vo_lat_behavior_plan->add_avoid_car_ids(msg.vo_lat_behavior_plan.avoid_car_ids[i]);
    }
    for (size_t i = 0; i < msg.vo_lat_behavior_plan.avoid_car_allow_max_opposite_offset.size(); i++) {
        vo_lat_behavior_plan->add_avoid_car_allow_max_opposite_offset(
            msg.vo_lat_behavior_plan.avoid_car_allow_max_opposite_offset[i]);
    }
    // vo_lat_motion_plan
    planning::common::VOLatMotionPlan *vo_lat_motion_plan = obj->mutable_vo_lat_motion_plan();
    for (size_t i = 0; i < msg.vo_lat_motion_plan.basic_dpoly.size(); i++) {
        vo_lat_motion_plan->add_basic_dpoly(msg.vo_lat_motion_plan.basic_dpoly[i]);
    }
    vo_lat_motion_plan->set_premove_dpoly_c0(msg.vo_lat_motion_plan.premove_dpoly_c0);
    vo_lat_motion_plan->set_avoid_dpoly_c0(msg.vo_lat_motion_plan.avoid_dpoly_c0);
    // environment_model_info
    planning::common::EnvironmentModelInfo *environment_model_info = obj->mutable_environment_model_info();
    environment_model_info->set_currrent_lane_vitual_id(msg.environment_model_info.currrent_lane_vitual_id);
    environment_model_info->set_ego_s(msg.environment_model_info.ego_s);
    environment_model_info->set_ego_l(msg.environment_model_info.ego_l);
    for (size_t i = 0; i < msg.environment_model_info.obstacle.size(); i++) {
        planning::common::Obstacle *obstacle = environment_model_info->add_obstacle();
        obstacle->set_id(msg.environment_model_info.obstacle[i].id);
        obstacle->set_type(msg.environment_model_info.obstacle[i].type);
        obstacle->set_s(msg.environment_model_info.obstacle[i].s);
        obstacle->set_l(msg.environment_model_info.obstacle[i].l);
        obstacle->set_s_to_ego(msg.environment_model_info.obstacle[i].s_to_ego);
        obstacle->set_max_l_to_ref(msg.environment_model_info.obstacle[i].max_l_to_ref);
        obstacle->set_min_l_to_ref(msg.environment_model_info.obstacle[i].min_l_to_ref);
        obstacle->set_nearest_l_to_desire_path(msg.environment_model_info.obstacle[i].nearest_l_to_desire_path);
        obstacle->set_nearest_l_to_ego(msg.environment_model_info.obstacle[i].nearest_l_to_ego);
        obstacle->set_vs_lat_relative(msg.environment_model_info.obstacle[i].vs_lat_relative);
        obstacle->set_vs_lon_relative(msg.environment_model_info.obstacle[i].vs_lon_relative);
        obstacle->set_vs_lon(msg.environment_model_info.obstacle[i].vs_lon);
        obstacle->set_min_l(msg.environment_model_info.obstacle[i].min_l);
        obstacle->set_s_with_min_l(msg.environment_model_info.obstacle[i].s_with_min_l);
        obstacle->set_max_l(msg.environment_model_info.obstacle[i].max_l);
        obstacle->set_s_with_max_l(msg.environment_model_info.obstacle[i].s_with_max_l);
        obstacle->set_nearest_y_to_desired_path(msg.environment_model_info.obstacle[i].nearest_y_to_desired_path);
        obstacle->set_is_accident_car(msg.environment_model_info.obstacle[i].is_accident_car);
        obstacle->set_is_accident_cnt(msg.environment_model_info.obstacle[i].is_accident_cnt);
        obstacle->set_is_avoid_car(msg.environment_model_info.obstacle[i].is_avoid_car);
        obstacle->set_is_lane_lead_obstacle(msg.environment_model_info.obstacle[i].is_lane_lead_obstacle);
        obstacle->set_current_lead_obstacle_to_ego(msg.environment_model_info.obstacle[i].current_lead_obstacle_to_ego);
    }
    // input_topic_timestamp
    planning::common::TopicTimeList *input_topic_timestamp = obj->mutable_input_topic_timestamp();
    input_topic_timestamp->set_fusion_object(msg.input_topic_timestamp.fusion_object);
    input_topic_timestamp->set_fusion_road(msg.input_topic_timestamp.fusion_road);
    input_topic_timestamp->set_localization_estimate(msg.input_topic_timestamp.localization_estimate);
    input_topic_timestamp->set_prediction(msg.input_topic_timestamp.prediction);
    input_topic_timestamp->set_vehicle_service(msg.input_topic_timestamp.vehicle_service);
    input_topic_timestamp->set_radar_perception(msg.input_topic_timestamp.radar_perception);
    input_topic_timestamp->set_control_output(msg.input_topic_timestamp.control_output);
    input_topic_timestamp->set_hmi(msg.input_topic_timestamp.hmi);
    input_topic_timestamp->set_parking_fusion(msg.input_topic_timestamp.parking_fusion);
    input_topic_timestamp->set_function_state_machine(msg.input_topic_timestamp.function_state_machine);
    input_topic_timestamp->set_map(msg.input_topic_timestamp.map);
    input_topic_timestamp->set_ehr_parking_map(msg.input_topic_timestamp.ehr_parking_map);
    input_topic_timestamp->set_ground_line(msg.input_topic_timestamp.ground_line);
    input_topic_timestamp->set_localization(msg.input_topic_timestamp.localization);
    // input_topic_latency
    planning::common::TopicTimeList *input_topic_latency = obj->mutable_input_topic_latency();
    input_topic_latency->set_fusion_object(msg.input_topic_latency.fusion_object);
    input_topic_latency->set_fusion_road(msg.input_topic_latency.fusion_road);
    input_topic_latency->set_localization_estimate(msg.input_topic_latency.localization_estimate);
    input_topic_latency->set_prediction(msg.input_topic_latency.prediction);
    input_topic_latency->set_vehicle_service(msg.input_topic_latency.vehicle_service);
    input_topic_latency->set_radar_perception(msg.input_topic_latency.radar_perception);
    input_topic_latency->set_control_output(msg.input_topic_latency.control_output);
    input_topic_latency->set_hmi(msg.input_topic_latency.hmi);
    input_topic_latency->set_parking_fusion(msg.input_topic_latency.parking_fusion);
    input_topic_latency->set_function_state_machine(msg.input_topic_latency.function_state_machine);
    input_topic_latency->set_map(msg.input_topic_latency.map);
    input_topic_latency->set_ehr_parking_map(msg.input_topic_latency.ehr_parking_map);
    input_topic_latency->set_ground_line(msg.input_topic_latency.ground_line);
    input_topic_latency->set_localization(msg.input_topic_latency.localization);
    // frame_info
    planning::common::FrameInfo *frame_info = obj->mutable_frame_info();
    frame_info->set_frame_num(msg.frame_info.frame_num);
    frame_info->set_version(msg.frame_info.version);
    frame_info->set_scene_type(msg.frame_info.scene_type);
    frame_info->set_frame_duration_ms(msg.frame_info.frame_duration_ms);
    frame_info->set_planning_succ(msg.frame_info.planning_succ);
    // road_fusion_latency
    planning::common::ImageLatency *road_fusion_latency = obj->mutable_road_fusion_latency();
    road_fusion_latency->set_image_com_latency_ms(msg.road_fusion_latency.image_com_latency_ms);
    road_fusion_latency->set_perception_latency_ms(msg.road_fusion_latency.perception_latency_ms);
    road_fusion_latency->set_perception_com_latency_ms(msg.road_fusion_latency.perception_com_latency_ms);
    road_fusion_latency->set_fusion_lantency_ms(msg.road_fusion_latency.fusion_lantency_ms);
    road_fusion_latency->set_fusion_com_lantency_ms(msg.road_fusion_latency.fusion_com_lantency_ms);
    // obstacle_fusion_latency
    planning::common::ImageLatency *obstacle_fusion_latency = obj->mutable_obstacle_fusion_latency();
    obstacle_fusion_latency->set_image_com_latency_ms(msg.obstacle_fusion_latency.image_com_latency_ms);
    obstacle_fusion_latency->set_perception_latency_ms(msg.obstacle_fusion_latency.perception_latency_ms);
    obstacle_fusion_latency->set_perception_com_latency_ms(msg.obstacle_fusion_latency.perception_com_latency_ms);
    obstacle_fusion_latency->set_fusion_lantency_ms(msg.obstacle_fusion_latency.fusion_lantency_ms);
    obstacle_fusion_latency->set_fusion_com_lantency_ms(msg.obstacle_fusion_latency.fusion_com_lantency_ms);
    // location_latency
    planning::common::LocationLatency *location_latency = obj->mutable_location_latency();
    location_latency->set_sensor_mcu_latency_ms(msg.location_latency.sensor_mcu_latency_ms);
    location_latency->set_sensor_soc_latency_ms(msg.location_latency.sensor_soc_latency_ms);
    location_latency->set_sensor_com_latency_ms(msg.location_latency.sensor_com_latency_ms);
    location_latency->set_location_latency(msg.location_latency.location_latency);
    //
    obj->set_data_json(msg.data_json);
    // slot_management_info
    planning::common::SlotManagementInfo *slot_management_info = obj->mutable_slot_management_info();
    for (size_t i = 0; i < msg.slot_management_info.slot_info_vec.size(); i++) {
        planning::common::SlotInfo *slot_info_vec = slot_management_info->add_slot_info_vec();
        slot_info_vec->set_id(msg.slot_management_info.slot_info_vec[i].id);
        //
        planning::common::CornerPoints *corner_points = slot_info_vec->mutable_corner_points();
        for (size_t j = 0; j < msg.slot_management_info.slot_info_vec[i].corner_points.corner_point.size(); j++) {
            planning::common::Point2d *corner_point = corner_points->add_corner_point();
            corner_point->set_x(msg.slot_management_info.slot_info_vec[i].corner_points.corner_point[j].x);
            corner_point->set_y(msg.slot_management_info.slot_info_vec[i].corner_points.corner_point[j].y);
        }
        //
        planning::common::Point2d *center = slot_info_vec->mutable_center();
        center->set_x(msg.slot_management_info.slot_info_vec[i].center.x);
        center->set_y(msg.slot_management_info.slot_info_vec[i].center.y);
        slot_info_vec->set_is_release(msg.slot_management_info.slot_info_vec[i].is_release);
        slot_info_vec->set_is_occupied(msg.slot_management_info.slot_info_vec[i].is_occupied);
        slot_info_vec->set_slot_type(msg.slot_management_info.slot_info_vec[i].slot_type);
    }

    for (size_t i = 0; i < msg.slot_management_info.limiter_points.size(); i++) {
        planning::common::Point2d *limiter_points = slot_management_info->add_limiter_points();
        limiter_points->set_x(msg.slot_management_info.limiter_points[i].x);
        limiter_points->set_y(msg.slot_management_info.limiter_points[i].x);
    }

    for (size_t i = 0; i < msg.slot_management_info.limiter_points.size(); i++) {
        planning::common::Point2d *limiter_points = slot_management_info->add_limiter_points();
        limiter_points->set_x(msg.slot_management_info.limiter_points[i].x);
        limiter_points->set_y(msg.slot_management_info.limiter_points[i].x);
    }

    for (size_t i = 0; i < msg.slot_management_info.point_obstacle_vec.size(); i++) {
        planning::common::Point2d *point_obstacle_vec = slot_management_info->add_point_obstacle_vec();
        point_obstacle_vec->set_x(msg.slot_management_info.point_obstacle_vec[i].x);
        point_obstacle_vec->set_y(msg.slot_management_info.point_obstacle_vec[i].x);
    }
    slot_management_info->set_selected_id(msg.slot_management_info.selected_id);
    slot_management_info->set_selected_slot_side(msg.slot_management_info.selected_slot_side);

    //real_time_lon_behavior_planning_input
    planning::common::RealTimeLonBehaviorInput *real_time_lon_behavior_planning_input = obj->mutable_real_time_lon_behavior_planning_input();
    planning::common::EgoInfo *ego_info = real_time_lon_behavior_planning_input->mutable_ego_info();
    ego_info->set_ego_v(msg.real_time_lon_behavior_planning_input.ego_info.ego_v);
    ego_info->set_ego_cruise(msg.real_time_lon_behavior_planning_input.ego_info.ego_cruise);
    ego_info->set_ego_acc(msg.real_time_lon_behavior_planning_input.ego_info.ego_acc);
    ego_info->set_ego_steer_angle(msg.real_time_lon_behavior_planning_input.ego_info.ego_steer_angle);

    planning::common::LatOutputInfo *lat_output = real_time_lon_behavior_planning_input->mutable_lat_output();
    lat_output->set_lc_request(msg.real_time_lon_behavior_planning_input.lat_output.lc_request);
    lat_output->set_lc_status(msg.real_time_lon_behavior_planning_input.lat_output.lc_status);
    lat_output->set_close_to_accident(msg.real_time_lon_behavior_planning_input.lat_output.close_to_accident);
    for (size_t i = 0; i < msg.real_time_lon_behavior_planning_input.lat_output.d_poly_vec.size(); i++) {
        lat_output->add_d_poly_vec(msg.real_time_lon_behavior_planning_input.lat_output.d_poly_vec[i]);
    }

    planning::common::LatObsInfo *lat_obs_info = real_time_lon_behavior_planning_input->mutable_lat_obs_info();
    planning::common::TrackedObjectInfo *lead_one = lat_obs_info->mutable_lead_one();
    lead_one->set_track_id(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.track_id);
    lead_one->set_type(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.type);
    lead_one->set_fusion_source(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.fusion_source);
    lead_one->set_v_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.v_lead);
    lead_one->set_v_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.v_rel);
    lead_one->set_a_lead_k(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.a_lead_k);
    lead_one->set_d_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.d_rel);
    lead_one->set_d_path(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.d_path);
    lead_one->set_d_path_self(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.d_path_self);
    lead_one->set_v_lat(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.v_lat);
    lead_one->set_is_accident_car(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.is_accident_car);
    lead_one->set_is_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.is_lead);
    lead_one->set_is_temp_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.is_temp_lead);
    lead_one->set_cutinp(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.cutinp);
    lead_one->set_y_min(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.y_min);
    lead_one->set_y_x0(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.y_x0);
    lead_one->set_location_head(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.location_head);
    lead_one->set_location_tail(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.location_tail);
    lead_one->set_vy_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.vy_rel);
    lead_one->set_y_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_one.y_rel);

    planning::common::TrackedObjectInfo *lead_two = lat_obs_info->mutable_lead_two();
    lead_two->set_track_id(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.track_id);
    lead_two->set_type(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.type);
    lead_two->set_fusion_source(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.fusion_source);
    lead_two->set_v_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.v_lead);
    lead_two->set_v_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.v_rel);
    lead_two->set_a_lead_k(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.a_lead_k);
    lead_two->set_d_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.d_rel);
    lead_two->set_d_path(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.d_path);
    lead_two->set_d_path_self(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.d_path_self);
    lead_two->set_v_lat(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.v_lat);
    lead_two->set_is_accident_car(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.is_accident_car);
    lead_two->set_is_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.is_lead);
    lead_two->set_is_temp_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.is_temp_lead);
    lead_two->set_cutinp(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.cutinp);
    lead_two->set_y_min(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.y_min);
    lead_two->set_y_x0(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.y_x0);
    lead_two->set_location_head(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.location_head);
    lead_two->set_location_tail(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.location_tail);
    lead_two->set_vy_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.vy_rel);
    lead_two->set_y_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.lead_two.y_rel);
    
    planning::common::TrackedObjectInfo *temp_lead_one = lat_obs_info->mutable_temp_lead_one();
    temp_lead_one->set_track_id(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.track_id);
    temp_lead_one->set_type(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.type);
    temp_lead_one->set_fusion_source(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.fusion_source);
    temp_lead_one->set_v_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.v_lead);
    temp_lead_one->set_v_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.v_rel);
    temp_lead_one->set_a_lead_k(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.a_lead_k);
    temp_lead_one->set_d_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.d_rel);
    temp_lead_one->set_d_path(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.d_path);
    temp_lead_one->set_d_path_self(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.d_path_self);
    temp_lead_one->set_v_lat(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.v_lat);
    temp_lead_one->set_is_accident_car(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.is_accident_car);
    temp_lead_one->set_is_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.is_lead);
    temp_lead_one->set_is_temp_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.is_temp_lead);
    temp_lead_one->set_cutinp(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.cutinp);
    temp_lead_one->set_y_min(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.y_min);
    temp_lead_one->set_y_x0(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.y_x0);
    temp_lead_one->set_location_head(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.location_head);
    temp_lead_one->set_location_tail(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.location_tail);
    temp_lead_one->set_vy_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.vy_rel);
    temp_lead_one->set_y_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_one.y_rel);

    planning::common::TrackedObjectInfo *temp_lead_two = lat_obs_info->mutable_temp_lead_two();
    temp_lead_two->set_track_id(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.track_id);
    temp_lead_two->set_type(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.type);
    temp_lead_two->set_fusion_source(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.fusion_source);
    temp_lead_two->set_v_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.v_lead);
    temp_lead_two->set_v_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.v_rel);
    temp_lead_two->set_a_lead_k(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.a_lead_k);
    temp_lead_two->set_d_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.d_rel);
    temp_lead_two->set_d_path(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.d_path);
    temp_lead_two->set_d_path_self(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.d_path_self);
    temp_lead_two->set_v_lat(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.v_lat);
    temp_lead_two->set_is_accident_car(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.is_accident_car);
    temp_lead_two->set_is_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.is_lead);
    temp_lead_two->set_is_temp_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.is_temp_lead);
    temp_lead_two->set_cutinp(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.cutinp);
    temp_lead_two->set_y_min(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.y_min);
    temp_lead_two->set_y_x0(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.y_x0);
    temp_lead_two->set_location_head(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.location_head);
    temp_lead_two->set_location_tail(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.location_tail);
    temp_lead_two->set_vy_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.vy_rel);
    temp_lead_two->set_y_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.temp_lead_two.y_rel);

    for (size_t i = 0; i < msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks.size(); i++) {
        planning::common::TrackedObjectInfo *front_tracks = lat_obs_info->add_front_tracks();
        front_tracks->set_track_id(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].track_id);
        front_tracks->set_type(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].type);
        front_tracks->set_fusion_source(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].fusion_source);
        front_tracks->set_v_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].v_lead);
        front_tracks->set_v_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].v_rel);
        front_tracks->set_a_lead_k(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].a_lead_k);
        front_tracks->set_d_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].d_rel);
        front_tracks->set_d_path(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].d_path);
        front_tracks->set_d_path_self(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].d_path_self);
        front_tracks->set_v_lat(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].v_lat);
        front_tracks->set_is_accident_car(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].is_accident_car);
        front_tracks->set_is_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].is_lead);
        front_tracks->set_is_temp_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].is_temp_lead);
        front_tracks->set_cutinp(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].cutinp);
        front_tracks->set_y_min(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].y_min);
        front_tracks->set_y_x0(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].y_x0);
        front_tracks->set_location_head(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].location_head);
        front_tracks->set_location_tail(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].location_tail);
        front_tracks->set_vy_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].vy_rel);
        front_tracks->set_y_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.front_tracks[i].y_rel);  
    }

    for (size_t i = 0; i < msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks.size(); i++) {
        planning::common::TrackedObjectInfo *side_tracks = lat_obs_info->add_side_tracks();
        side_tracks->set_track_id(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].track_id);
        side_tracks->set_type(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].type);
        side_tracks->set_fusion_source(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].fusion_source);
        side_tracks->set_v_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].v_lead);
        side_tracks->set_v_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].v_rel);
        side_tracks->set_a_lead_k(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].a_lead_k);
        side_tracks->set_d_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].d_rel);
        side_tracks->set_d_path(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].d_path);
        side_tracks->set_d_path_self(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].d_path_self);
        side_tracks->set_v_lat(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].v_lat);
        side_tracks->set_is_accident_car(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].is_accident_car);
        side_tracks->set_is_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].is_lead);
        side_tracks->set_is_temp_lead(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].is_temp_lead);
        side_tracks->set_cutinp(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].cutinp);
        side_tracks->set_y_min(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].y_min);
        side_tracks->set_y_x0(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].y_x0);
        side_tracks->set_location_head(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].location_head);
        side_tracks->set_location_tail(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].location_tail);
        side_tracks->set_vy_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].vy_rel);
        side_tracks->set_y_rel(msg.real_time_lon_behavior_planning_input.lat_obs_info.side_tracks[i].y_rel);  
    }

    planning::common::LaneChangeInfo *lc_info = real_time_lon_behavior_planning_input->mutable_lc_info();
    lc_info->set_has_target_lane(msg.real_time_lon_behavior_planning_input.lc_info.has_target_lane);
    lc_info->set_tlane_virtual_id(msg.real_time_lon_behavior_planning_input.lc_info.tlane_virtual_id);
    lc_info->set_lc_map_decision(msg.real_time_lon_behavior_planning_input.lc_info.lc_map_decision);
    lc_info->set_lc_end_dis(msg.real_time_lon_behavior_planning_input.lc_info.lc_end_dis);
    lc_info->set_v_limit(msg.real_time_lon_behavior_planning_input.lc_info.v_limit);
    lc_info->set_current_lane_type(msg.real_time_lon_behavior_planning_input.lc_info.current_lane_type);
    lc_info->set_target_gap_obs_first(msg.real_time_lon_behavior_planning_input.lc_info.target_gap_obs_first);
    lc_info->set_target_gap_obs_second(msg.real_time_lon_behavior_planning_input.lc_info.target_gap_obs_second);
    for (size_t i = 0; i < msg.real_time_lon_behavior_planning_input.lc_info.lc_cars.size(); i++) {
        planning::common::TrackedObjectInfo *lc_cars = lc_info->add_lc_cars();
        lc_cars->set_track_id(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].track_id);
        lc_cars->set_type(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].type);
        lc_cars->set_fusion_source(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].fusion_source);
        lc_cars->set_v_lead(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].v_lead);
        lc_cars->set_v_rel(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].v_rel);
        lc_cars->set_a_lead_k(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].a_lead_k);
        lc_cars->set_d_rel(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].d_rel);
        lc_cars->set_d_path(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].d_path);
        lc_cars->set_d_path_self(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].d_path_self);
        lc_cars->set_v_lat(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].v_lat);
        lc_cars->set_is_accident_car(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].is_accident_car);
        lc_cars->set_is_lead(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].is_lead);
        lc_cars->set_is_temp_lead(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].is_temp_lead);
        lc_cars->set_cutinp(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].cutinp);
        lc_cars->set_y_min(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].y_min);
        lc_cars->set_y_x0(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].y_x0);
        lc_cars->set_location_head(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].location_head);
        lc_cars->set_location_tail(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].location_tail);
        lc_cars->set_vy_rel(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].vy_rel);
        lc_cars->set_y_rel(msg.real_time_lon_behavior_planning_input.lc_info.lc_cars[i].y_rel);  
    }
    
    planning::common::StartStopInfo *start_stop_info = real_time_lon_behavior_planning_input->mutable_start_stop_info();
    start_stop_info->set_is_stop(msg.real_time_lon_behavior_planning_input.start_stop_info.is_stop);
    start_stop_info->set_is_start(msg.real_time_lon_behavior_planning_input.start_stop_info.is_start);
    start_stop_info->set_enable_stop(msg.real_time_lon_behavior_planning_input.start_stop_info.enable_stop);
    start_stop_info->set_state(planning::common::StartStopInfo_StateType(msg.real_time_lon_behavior_planning_input.start_stop_info.state));
    start_stop_info->set_stop_distance_of_leadone(msg.real_time_lon_behavior_planning_input.start_stop_info.stop_distance_of_leadone);

    real_time_lon_behavior_planning_input->set_dbw_status(msg.real_time_lon_behavior_planning_input.dbw_status);

    planning::common::LonDecisionInfo *lon_decision_info = real_time_lon_behavior_planning_input->mutable_lon_decision_info();
    planning::common::LeadoneInfo *leadone_info = lon_decision_info->mutable_leadone_info();
    leadone_info->set_has_leadone(msg.real_time_lon_behavior_planning_input.lon_decision_info.leadone_info.has_leadone);
    planning::common::ObstacleInformation *leadone_information = leadone_info->mutable_leadone_information();
    leadone_information->set_obstacle_id(msg.real_time_lon_behavior_planning_input.lon_decision_info.leadone_info.leadone_information.obstacle_id);
    leadone_information->set_obstacle_type(msg.real_time_lon_behavior_planning_input.lon_decision_info.leadone_info.leadone_information.obstacle_type);
    leadone_information->set_obstacle_s(msg.real_time_lon_behavior_planning_input.lon_decision_info.leadone_info.leadone_information.obstacle_s);
    leadone_information->set_obstacle_v(msg.real_time_lon_behavior_planning_input.lon_decision_info.leadone_info.leadone_information.obstacle_v);
    leadone_information->set_obstacle_length(msg.real_time_lon_behavior_planning_input.lon_decision_info.leadone_info.leadone_information.obstacle_length);
    leadone_information->set_duration(msg.real_time_lon_behavior_planning_input.lon_decision_info.leadone_info.leadone_information.duration);
    leadone_information->set_desired_distance(msg.real_time_lon_behavior_planning_input.lon_decision_info.leadone_info.leadone_information.desired_distance);

    planning::common::LeadoneInfo *temp_leadone_info = lon_decision_info->mutable_temp_leadone_info();
    temp_leadone_info->set_has_leadone(msg.real_time_lon_behavior_planning_input.lon_decision_info.temp_leadone_info.has_leadone);
    planning::common::ObstacleInformation *leadone_information1 = temp_leadone_info->mutable_leadone_information();
    leadone_information1->set_obstacle_id(msg.real_time_lon_behavior_planning_input.lon_decision_info.temp_leadone_info.leadone_information.obstacle_id);
    leadone_information1->set_obstacle_type(msg.real_time_lon_behavior_planning_input.lon_decision_info.temp_leadone_info.leadone_information.obstacle_type);
    leadone_information1->set_obstacle_s(msg.real_time_lon_behavior_planning_input.lon_decision_info.temp_leadone_info.leadone_information.obstacle_s);
    leadone_information1->set_obstacle_v(msg.real_time_lon_behavior_planning_input.lon_decision_info.temp_leadone_info.leadone_information.obstacle_v);
    leadone_information1->set_obstacle_length(msg.real_time_lon_behavior_planning_input.lon_decision_info.temp_leadone_info.leadone_information.obstacle_length);
    leadone_information1->set_duration(msg.real_time_lon_behavior_planning_input.lon_decision_info.temp_leadone_info.leadone_information.duration);
    leadone_information1->set_desired_distance(msg.real_time_lon_behavior_planning_input.lon_decision_info.temp_leadone_info.leadone_information.desired_distance);

    planning::common::CutinInfo *cutin_info = lon_decision_info->mutable_cutin_info();
    cutin_info->set_has_cutin(msg.real_time_lon_behavior_planning_input.lon_decision_info.cutin_info.has_cutin);
    for (size_t i = 0; i < msg.real_time_lon_behavior_planning_input.lon_decision_info.cutin_info.cutin_information.size(); i++)
    {
        planning::common::ObstacleInformation *cutin_information = cutin_info->add_cutin_information();
        cutin_information->set_obstacle_id(msg.real_time_lon_behavior_planning_input.lon_decision_info.cutin_info.cutin_information[i].obstacle_id);
        cutin_information->set_obstacle_type(msg.real_time_lon_behavior_planning_input.lon_decision_info.cutin_info.cutin_information[i].obstacle_type);
        cutin_information->set_obstacle_s(msg.real_time_lon_behavior_planning_input.lon_decision_info.cutin_info.cutin_information[i].obstacle_s);
        cutin_information->set_obstacle_v(msg.real_time_lon_behavior_planning_input.lon_decision_info.cutin_info.cutin_information[i].obstacle_v);
        cutin_information->set_obstacle_length(msg.real_time_lon_behavior_planning_input.lon_decision_info.cutin_info.cutin_information[i].obstacle_length);
        cutin_information->set_duration(msg.real_time_lon_behavior_planning_input.lon_decision_info.cutin_info.cutin_information[i].duration);
        cutin_information->set_desired_distance(msg.real_time_lon_behavior_planning_input.lon_decision_info.cutin_info.cutin_information[i].desired_distance);
    }

    planning::common::CIPVInfo *cipv_info = lon_decision_info->mutable_cipv_info();
    cipv_info->set_has_cipv(msg.real_time_lon_behavior_planning_input.lon_decision_info.cipv_info.has_CIPV);
    for (size_t i = 0; i < msg.real_time_lon_behavior_planning_input.lon_decision_info.cipv_info.cipv_information.size(); i++)
    {
        planning::common::ObstacleInformation *cipv_information = cipv_info->add_cipv_information();
        cipv_information->set_obstacle_id(msg.real_time_lon_behavior_planning_input.lon_decision_info.cipv_info.cipv_information[i].obstacle_id);
        cipv_information->set_obstacle_type(msg.real_time_lon_behavior_planning_input.lon_decision_info.cipv_info.cipv_information[i].obstacle_type);
        cipv_information->set_obstacle_s(msg.real_time_lon_behavior_planning_input.lon_decision_info.cipv_info.cipv_information[i].obstacle_s);
        cipv_information->set_obstacle_v(msg.real_time_lon_behavior_planning_input.lon_decision_info.cipv_info.cipv_information[i].obstacle_v);
        cipv_information->set_obstacle_length(msg.real_time_lon_behavior_planning_input.lon_decision_info.cipv_info.cipv_information[i].obstacle_length);
        cipv_information->set_duration(msg.real_time_lon_behavior_planning_input.lon_decision_info.cipv_info.cipv_information[i].duration);
        cipv_information->set_desired_distance(msg.real_time_lon_behavior_planning_input.lon_decision_info.cipv_info.cipv_information[i].desired_distance);
    }

    planning::common::AccSafetyInfo *acc_safety_info = lon_decision_info->mutable_acc_safety_info();
    acc_safety_info->set_need_takeover(msg.real_time_lon_behavior_planning_input.lon_decision_info.acc_safety_info.need_takeover);

    lon_decision_info->set_nearby_obstacle(msg.real_time_lon_behavior_planning_input.lon_decision_info.nearby_obstacle);
    lon_decision_info->set_map_velocity_limit(msg.real_time_lon_behavior_planning_input.lon_decision_info.map_velocity_limit);
    lon_decision_info->set_v_cruise(msg.real_time_lon_behavior_planning_input.lon_decision_info.v_cruise);

    planning::common::DrivingFunctionInfo *function_info = real_time_lon_behavior_planning_input->mutable_function_info();
    function_info->set_function_mode(planning::common::DrivingFunctionInfo_DrivingFunctionMode(msg.real_time_lon_behavior_planning_input.function_info.function_mode));
    function_info->set_function_state(planning::common::DrivingFunctionInfo_DrivingFunctionstate(msg.real_time_lon_behavior_planning_input.function_info.function_state));

    real_time_lon_behavior_planning_input->set_is_on_ramp(msg.real_time_lon_behavior_planning_input.is_on_ramp);
    real_time_lon_behavior_planning_input->set_dis_to_ramp(msg.real_time_lon_behavior_planning_input.dis_to_ramp);
    real_time_lon_behavior_planning_input->set_dis_to_merge(msg.real_time_lon_behavior_planning_input.dis_to_merge);

    for (size_t i = 0; i < msg.real_time_lon_behavior_planning_input.ref_path_points.size(); i++) {
        planning::common::ReferencePathPoint *ref_path_points = real_time_lon_behavior_planning_input->add_ref_path_points();
        planning::common::PathPoint *path_point = ref_path_points->mutable_path_point();
        path_point->set_x(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.x);
        path_point->set_y(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.y);
        path_point->set_z(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.z);
        path_point->set_theta(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.theta);
        path_point->set_kappa(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.kappa);
        path_point->set_s(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.s);
        path_point->set_dkappa(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.dkappa);
        path_point->set_ddkappa(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.ddkappa);
        path_point->set_lane_id(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.lane_id);
        path_point->set_x_derivative(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.x_derivative);
        path_point->set_y_derivative(msg.real_time_lon_behavior_planning_input.ref_path_points[i].path_point.y_derivative);
        
        ref_path_points->set_distance_to_left_road_border(msg.real_time_lon_behavior_planning_input.ref_path_points[i].distance_to_left_road_border);
        ref_path_points->set_distance_to_right_road_border(msg.real_time_lon_behavior_planning_input.ref_path_points[i].distance_to_right_road_border);
        ref_path_points->set_distance_to_left_lane_border(msg.real_time_lon_behavior_planning_input.ref_path_points[i].distance_to_left_lane_border);
        ref_path_points->set_distance_to_right_lane_border(msg.real_time_lon_behavior_planning_input.ref_path_points[i].distance_to_right_lane_border);
        ref_path_points->set_lane_width(msg.real_time_lon_behavior_planning_input.ref_path_points[i].lane_width);
        ref_path_points->set_max_velocity(msg.real_time_lon_behavior_planning_input.ref_path_points[i].max_velocity);
        ref_path_points->set_min_velocity(msg.real_time_lon_behavior_planning_input.ref_path_points[i].min_velocity);
        ref_path_points->set_is_in_intersection(msg.real_time_lon_behavior_planning_input.ref_path_points[i].is_in_intersection);

    
    }
    //LateralBehaviorDebugInfo lateral_behavior_debug_info
    planning::common::LateralBehaviorDebugInfo *lateral_behavior_debug_info = obj->mutable_lateral_behavior_debug_info();
    for (size_t i = 0; i < msg.lateral_behavior_debug_info.bound_s_vec.size(); i++) {
        lateral_behavior_debug_info->add_bound_s_vec(msg.lateral_behavior_debug_info.bound_s_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_behavior_debug_info.soft_upper_bound_info_vec.size(); i++) {
        planning::common::WeightedBound *soft_upper_bound_info_vec = lateral_behavior_debug_info->add_soft_upper_bound_info_vec();
        soft_upper_bound_info_vec->set_lower(msg.lateral_behavior_debug_info.soft_upper_bound_info_vec[i].lower);
        soft_upper_bound_info_vec->set_upper(msg.lateral_behavior_debug_info.soft_upper_bound_info_vec[i].upper);
        soft_upper_bound_info_vec->set_weight(msg.lateral_behavior_debug_info.soft_upper_bound_info_vec[i].weight);
        planning::common::BoundInfo *bound_info = soft_upper_bound_info_vec->mutable_bound_info();
        bound_info->set_id(msg.lateral_behavior_debug_info.soft_upper_bound_info_vec[i].bound_info.id);
        bound_info->set_type(msg.lateral_behavior_debug_info.soft_upper_bound_info_vec[i].bound_info.type);
    }
    for (size_t i = 0; i < msg.lateral_behavior_debug_info.soft_lower_bound_info_vec.size(); i++) {
        planning::common::WeightedBound *soft_lower_bound_info_vec = lateral_behavior_debug_info->add_soft_lower_bound_info_vec();
        soft_lower_bound_info_vec->set_lower(msg.lateral_behavior_debug_info.soft_lower_bound_info_vec[i].lower);
        soft_lower_bound_info_vec->set_upper(msg.lateral_behavior_debug_info.soft_lower_bound_info_vec[i].upper);
        soft_lower_bound_info_vec->set_weight(msg.lateral_behavior_debug_info.soft_lower_bound_info_vec[i].weight);
        planning::common::BoundInfo *bound_info = soft_lower_bound_info_vec->mutable_bound_info();
        bound_info->set_id(msg.lateral_behavior_debug_info.soft_lower_bound_info_vec[i].bound_info.id);
        bound_info->set_type(msg.lateral_behavior_debug_info.soft_lower_bound_info_vec[i].bound_info.type);
    }
    for (size_t i = 0; i < msg.lateral_behavior_debug_info.hard_upper_bound_info_vec.size(); i++) {
        planning::common::WeightedBound *hard_upper_bound_info_vec = lateral_behavior_debug_info->add_hard_upper_bound_info_vec();
        hard_upper_bound_info_vec->set_lower(msg.lateral_behavior_debug_info.hard_upper_bound_info_vec[i].lower);
        hard_upper_bound_info_vec->set_upper(msg.lateral_behavior_debug_info.hard_upper_bound_info_vec[i].upper);
        hard_upper_bound_info_vec->set_weight(msg.lateral_behavior_debug_info.hard_upper_bound_info_vec[i].weight);
        planning::common::BoundInfo *bound_info = hard_upper_bound_info_vec->mutable_bound_info();
        bound_info->set_id(msg.lateral_behavior_debug_info.hard_upper_bound_info_vec[i].bound_info.id);
        bound_info->set_type(msg.lateral_behavior_debug_info.hard_upper_bound_info_vec[i].bound_info.type);
    }
    for (size_t i = 0; i < msg.lateral_behavior_debug_info.hard_lower_bound_info_vec.size(); i++) {
        planning::common::WeightedBound *hard_lower_bound_info_vec = lateral_behavior_debug_info->add_hard_lower_bound_info_vec();
        hard_lower_bound_info_vec->set_lower(msg.lateral_behavior_debug_info.hard_lower_bound_info_vec[i].lower);
        hard_lower_bound_info_vec->set_upper(msg.lateral_behavior_debug_info.hard_lower_bound_info_vec[i].upper);
        hard_lower_bound_info_vec->set_weight(msg.lateral_behavior_debug_info.hard_lower_bound_info_vec[i].weight);
        planning::common::BoundInfo *bound_info = hard_lower_bound_info_vec->mutable_bound_info();
        bound_info->set_id(msg.lateral_behavior_debug_info.hard_lower_bound_info_vec[i].bound_info.id);
        bound_info->set_type(msg.lateral_behavior_debug_info.hard_lower_bound_info_vec[i].bound_info.type);
    }

    //LateralPathOptimizerInput lateral_path_optimizer_input
    planning::common::LateralPathOptimizerInput *lateral_path_optimizer_input = obj->mutable_lateral_path_optimizer_input();
    planning::common::LateralPathOptimizerInitState *init_state = lateral_path_optimizer_input->mutable_init_state();
    init_state->set_x(msg.lateral_path_optimizer_input.init_state.x);
    init_state->set_y(msg.lateral_path_optimizer_input.init_state.y);
    init_state->set_theta(msg.lateral_path_optimizer_input.init_state.theta);
    init_state->set_k(msg.lateral_path_optimizer_input.init_state.k);

    for (size_t i = 0; i < msg.lateral_path_optimizer_input.ref_x_vec.size(); i++) {
        lateral_path_optimizer_input->add_ref_x_vec(msg.lateral_path_optimizer_input.ref_x_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_input.ref_y_vec.size(); i++) {
        lateral_path_optimizer_input->add_ref_y_vec(msg.lateral_path_optimizer_input.ref_y_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_input.ref_theta_vec.size(); i++) {
        lateral_path_optimizer_input->add_ref_theta_vec(msg.lateral_path_optimizer_input.ref_theta_vec[i]);
    }

    lateral_path_optimizer_input->set_last_theta(msg.lateral_path_optimizer_input.last_theta);
    lateral_path_optimizer_input->set_last_y(msg.lateral_path_optimizer_input.last_y);
    lateral_path_optimizer_input->set_last_x(msg.lateral_path_optimizer_input.last_x);

    for (size_t i = 0; i < msg.lateral_path_optimizer_input.k_max_vec.size(); i++) {
        lateral_path_optimizer_input->add_k_max_vec(msg.lateral_path_optimizer_input.k_max_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_input.k_min_vec.size(); i++) {
        lateral_path_optimizer_input->add_k_min_vec(msg.lateral_path_optimizer_input.k_min_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_input.u_min_vec.size(); i++) {
        lateral_path_optimizer_input->add_u_min_vec(msg.lateral_path_optimizer_input.u_min_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_input.u_max_vec.size(); i++) {
        lateral_path_optimizer_input->add_u_max_vec(msg.lateral_path_optimizer_input.u_max_vec[i]);
    }

    lateral_path_optimizer_input->set_r_min(msg.lateral_path_optimizer_input.r_min);
    lateral_path_optimizer_input->set_curv_factor(msg.lateral_path_optimizer_input.curv_factor);
    lateral_path_optimizer_input->set_delta_max(msg.lateral_path_optimizer_input.delta_max);
    lateral_path_optimizer_input->set_ref_vel(msg.lateral_path_optimizer_input.ref_vel);
    lateral_path_optimizer_input->set_q_ref_x(msg.lateral_path_optimizer_input.q_ref_x);
    lateral_path_optimizer_input->set_q_ref_theta(msg.lateral_path_optimizer_input.q_ref_theta);
    lateral_path_optimizer_input->set_q_terminal_theta(msg.lateral_path_optimizer_input.q_terminal_theta);
    lateral_path_optimizer_input->set_q_terminal_x(msg.lateral_path_optimizer_input.q_terminal_x);
    lateral_path_optimizer_input->set_q_terminal_y(msg.lateral_path_optimizer_input.q_terminal_y);
    lateral_path_optimizer_input->set_q_k(msg.lateral_path_optimizer_input.q_k);
    lateral_path_optimizer_input->set_q_u(msg.lateral_path_optimizer_input.q_u);
    lateral_path_optimizer_input->set_q_k_bound(msg.lateral_path_optimizer_input.q_k_bound);
    lateral_path_optimizer_input->set_q_u_bound(msg.lateral_path_optimizer_input.q_u_bound);

    for (size_t i = 0; i < msg.lateral_path_optimizer_input.control_vec.size(); i++) {
        lateral_path_optimizer_input->add_control_vec(msg.lateral_path_optimizer_input.control_vec[i]);
    }

    lateral_path_optimizer_input->set_ref_s(msg.lateral_path_optimizer_input.ref_s);

    for (size_t i = 0; i < msg.lateral_path_optimizer_input.last_x_vec.size(); i++) {
        lateral_path_optimizer_input->add_last_x_vec(msg.lateral_path_optimizer_input.last_x_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_input.last_y_vec.size(); i++) {
        lateral_path_optimizer_input->add_last_y_vec(msg.lateral_path_optimizer_input.last_y_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_input.last_theta_vec.size(); i++) {
        lateral_path_optimizer_input->add_last_theta_vec(msg.lateral_path_optimizer_input.last_theta_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_input.last_k_vec.size(); i++) {
        lateral_path_optimizer_input->add_last_k_vec(msg.lateral_path_optimizer_input.last_k_vec[i]);
    }

    //LateralPathOptimizerOutput lateral_path_optimizer_output
    planning::common::LateralPathOptimizerOutput *lateral_path_optimizer_output = obj->mutable_lateral_path_optimizer_output();
    for (size_t i = 0; i < msg.lateral_path_optimizer_output.s_vec.size(); i++) {
        lateral_path_optimizer_output->add_s_vec(msg.lateral_path_optimizer_output.s_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_output.x_vec.size(); i++) {
        lateral_path_optimizer_output->add_x_vec(msg.lateral_path_optimizer_output.x_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_output.y_vec.size(); i++) {
        lateral_path_optimizer_output->add_y_vec(msg.lateral_path_optimizer_output.y_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_output.theta_vec.size(); i++) {
        lateral_path_optimizer_output->add_theta_vec(msg.lateral_path_optimizer_output.theta_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_output.k_vec.size(); i++) {
        lateral_path_optimizer_output->add_k_vec(msg.lateral_path_optimizer_output.k_vec[i]);
    }
    for (size_t i = 0; i < msg.lateral_path_optimizer_output.u_vec.size(); i++) {
        lateral_path_optimizer_output->add_u_vec(msg.lateral_path_optimizer_output.u_vec[i]);
    }
    planning::common::PathOptimizerSolverInfo *solver_info1 = lateral_path_optimizer_output->mutable_solver_info();
    solver_info1->set_solver_condition(msg.lateral_path_optimizer_output.solver_info.solver_condition);
    solver_info1->set_cost_size(msg.lateral_path_optimizer_output.solver_info.cost_size);
    solver_info1->set_iter_count(msg.lateral_path_optimizer_output.solver_info.iter_count);
    solver_info1->set_init_cost(msg.lateral_path_optimizer_output.solver_info.init_cost);
    for (size_t i = 0; i < msg.lateral_path_optimizer_output.solver_info.iter_info.size(); i++) {
        planning::common::PathOptimizerSolverIterationInfo *iter_info = solver_info1->add_iter_info();
        iter_info->set_linesearch_success(msg.lateral_path_optimizer_output.solver_info.iter_info[i].linesearch_success);
        iter_info->set_backward_pass_count(msg.lateral_path_optimizer_output.solver_info.iter_info[i].backward_pass_count);
        iter_info->set_lambda(msg.lateral_path_optimizer_output.solver_info.iter_info[i].lambda);
        iter_info->set_cost(msg.lateral_path_optimizer_output.solver_info.iter_info[i].cost);
        iter_info->set_dcost(msg.lateral_path_optimizer_output.solver_info.iter_info[i].dcost);
        iter_info->set_expect(msg.lateral_path_optimizer_output.solver_info.iter_info[i].expect);
        iter_info->set_du_norm(msg.lateral_path_optimizer_output.solver_info.iter_info[i].du_norm);
    }
    
    //GapSelectorInput gap_selector_input
    planning::common::GapSelectorInput *gap_selector_input = obj->mutable_gap_selector_input();
    for (size_t i = 0; i < msg.gap_selector_input.origin_refline_points.size(); i++) {
        planning::common::Point2d *origin_refline_points = gap_selector_input->add_origin_refline_points();
        origin_refline_points->set_x(msg.gap_selector_input.origin_refline_points[i].x);
        origin_refline_points->set_y(msg.gap_selector_input.origin_refline_points[i].y);
    }
    for (size_t i = 0; i < msg.gap_selector_input.target_refline_points.size(); i++) {
        planning::common::Point2d *target_refline_points = gap_selector_input->add_origin_refline_points();
        target_refline_points->set_x(msg.gap_selector_input.target_refline_points[i].x);
        target_refline_points->set_y(msg.gap_selector_input.target_refline_points[i].y);
    }
    planning::common::GapSelectorInfo *gap_selector_info = gap_selector_input->mutable_gap_selector_info();
    gap_selector_info->set_lane_cross(msg.gap_selector_input.gap_selector_info.lane_cross);
    gap_selector_info->set_lc_triggered(msg.gap_selector_input.gap_selector_info.lc_triggered);
    gap_selector_info->set_lb_triggered(msg.gap_selector_input.gap_selector_info.lb_triggered);
    gap_selector_info->set_lc_in(msg.gap_selector_input.gap_selector_info.lc_in);
    gap_selector_info->set_lb_in(msg.gap_selector_input.gap_selector_info.lb_in);
    gap_selector_info->set_lc_pass_time(msg.gap_selector_input.gap_selector_info.lc_pass_time);
    gap_selector_info->set_lc_wait_time(msg.gap_selector_input.gap_selector_info.lc_wait_time);
    gap_selector_info->set_path_requintic(msg.gap_selector_input.gap_selector_info.path_requintic);
    gap_selector_info->set_lc_cancel(msg.gap_selector_input.gap_selector_info.lc_cancel);
    gap_selector_info->set_gs_skip(msg.gap_selector_input.gap_selector_info.gs_skip);
    gap_selector_info->set_lc_request(msg.gap_selector_input.gap_selector_info.lc_request);

    planning::common::GapSelectorPathSpline *last_gap_selector_path_spline = 
        gap_selector_info->mutable_last_gap_selector_path_spline();
    for (size_t i = 0; i < msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.x_vector_spline.size(); i++) {
        last_gap_selector_path_spline->add_x_vector_spline(
        msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.x_vector_spline[i]);
    }
    for (size_t i = 0; i < msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.y_vector_spline.size(); i++) {
        last_gap_selector_path_spline->add_y_vector_spline(
        msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.y_vector_spline[i]);
    }
    for (size_t i = 0; i < msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.s_vector_spline.size(); i++) {
        last_gap_selector_path_spline->add_s_vector_spline(
        msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.s_vector_spline[i]);
    }
    planning::common::LonState *start_state = last_gap_selector_path_spline->mutable_start_state();
    start_state->set_p(msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.start_state.p);
    start_state->set_v(msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.start_state.v);
    start_state->set_a(msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.start_state.a);
    start_state->set_j(msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.start_state.j);
    start_state->set_t(msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.start_state.t);
    planning::common::Point2d *start_cart_point = last_gap_selector_path_spline->mutable_start_cart_point();
    start_cart_point->set_x(msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.start_cart_point.x);
    start_cart_point->set_y(msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.start_cart_point.y);
    last_gap_selector_path_spline->set_path_spline_status(
        msg.gap_selector_input.gap_selector_info.last_gap_selector_path_spline.path_spline_status);
    for (size_t i = 0; i < msg.gap_selector_input.gap_selector_info.lc_request_buffer.size(); i++) {
        gap_selector_info->add_lc_request_buffer(
        msg.gap_selector_input.gap_selector_info.lc_request_buffer[i]);
    }
    for (size_t i = 0; i < msg.gap_selector_input.gap_selector_info.ego_l_buffer.size(); i++) {
        gap_selector_info->add_ego_l_buffer(
        msg.gap_selector_input.gap_selector_info.ego_l_buffer[i]);
    }
    for (size_t i = 0; i < msg.gap_selector_input.trajectory_points.size(); i++) {
        planning::common::TrajectoryPoint *target_refline_points = gap_selector_input->add_trajectory_points();
        target_refline_points->set_x(msg.gap_selector_input.trajectory_points[i].x);
        target_refline_points->set_y(msg.gap_selector_input.trajectory_points[i].y);
        target_refline_points->set_heading_angle(msg.gap_selector_input.trajectory_points[i].heading_angle);
        target_refline_points->set_curvature(msg.gap_selector_input.trajectory_points[i].curvature);
        target_refline_points->set_t(msg.gap_selector_input.trajectory_points[i].t);
        target_refline_points->set_v(msg.gap_selector_input.trajectory_points[i].v);
        target_refline_points->set_a(msg.gap_selector_input.trajectory_points[i].a);
        target_refline_points->set_s(msg.gap_selector_input.trajectory_points[i].s);
        target_refline_points->set_l(msg.gap_selector_input.trajectory_points[i].l);
        target_refline_points->set_frenet_valid(msg.gap_selector_input.trajectory_points[i].frenet_valid);
    }
    gap_selector_input->set_request(msg.gap_selector_input.request);
    for (size_t i = 0; i < msg.gap_selector_input.gs_care_objs.size(); i++) {
        planning::common::PredictionObject *gs_care_objs = gap_selector_input->add_gs_care_objs();
        gs_care_objs->set_id(msg.gap_selector_input.gs_care_objs[i].id);
        gs_care_objs->set_type((planning::common::PredictionObject::ObjectType)msg.gap_selector_input.gs_care_objs[i].type);
        gs_care_objs->set_fusion_source(msg.gap_selector_input.gs_care_objs[i].fusion_source);
        gs_care_objs->set_timestamp_us(msg.gap_selector_input.gs_care_objs[i].timestamp_us);
        gs_care_objs->set_delay_time(msg.gap_selector_input.gs_care_objs[i].delay_time);
        gs_care_objs->set_intention((planning::common::PredictionObject::ObstacleIntentType)msg.gap_selector_input.gs_care_objs[i].intention);
        gs_care_objs->set_b_backup_freemove(msg.gap_selector_input.gs_care_objs[i].b_backup_freemove);
        gs_care_objs->set_cutin_score(msg.gap_selector_input.gs_care_objs[i].cutin_score);
        gs_care_objs->set_position_x(msg.gap_selector_input.gs_care_objs[i].position_x);
        gs_care_objs->set_position_y(msg.gap_selector_input.gs_care_objs[i].position_y);
        gs_care_objs->set_length(msg.gap_selector_input.gs_care_objs[i].length);
        gs_care_objs->set_width(msg.gap_selector_input.gs_care_objs[i].width);
        gs_care_objs->set_speed(msg.gap_selector_input.gs_care_objs[i].speed);
        gs_care_objs->set_theta(msg.gap_selector_input.gs_care_objs[i].theta);
        gs_care_objs->set_acc(msg.gap_selector_input.gs_care_objs[i].acc);
        gs_care_objs->set_relative_position_x(msg.gap_selector_input.gs_care_objs[i].relative_position_x);
        gs_care_objs->set_relative_position_y(msg.gap_selector_input.gs_care_objs[i].relative_position_y);
        gs_care_objs->set_relative_speed_x(msg.gap_selector_input.gs_care_objs[i].relative_speed_x);
        gs_care_objs->set_relative_speed_y(msg.gap_selector_input.gs_care_objs[i].relative_speed_y);
        gs_care_objs->set_relative_acceleration_x(msg.gap_selector_input.gs_care_objs[i].relative_acceleration_x);
        gs_care_objs->set_relative_acceleration_y(msg.gap_selector_input.gs_care_objs[i].relative_acceleration_y);
        gs_care_objs->set_acceleration_relative_to_ground_x(msg.gap_selector_input.gs_care_objs[i].acceleration_relative_to_ground_x);
        gs_care_objs->set_acceleration_relative_to_ground_y(msg.gap_selector_input.gs_care_objs[i].acceleration_relative_to_ground_y);
        gs_care_objs->set_relative_theta(msg.gap_selector_input.gs_care_objs[i].relative_theta);
    }
    for (size_t i = 0; i < msg.gap_selector_input.target_lane_obstacle_ids.size(); i++) {
        gap_selector_input->add_target_lane_obstacle_ids(msg.gap_selector_input.target_lane_obstacle_ids[i]);
    }
    for (size_t i = 0; i < msg.gap_selector_input.origin_lane_obstacles_ids.size(); i++) {
        gap_selector_input->add_origin_lane_obstacles_ids(msg.gap_selector_input.origin_lane_obstacles_ids[i]);
    }
    gap_selector_input->set_cruise_vel(msg.gap_selector_input.cruise_vel);
    planning::common::PlanningInitPoint *planning_init_state = gap_selector_input->mutable_planning_init_state();
    planning_init_state->set_x(msg.gap_selector_input.planning_init_state.x);
    planning_init_state->set_y(msg.gap_selector_input.planning_init_state.y);
    planning_init_state->set_heading_angle(msg.gap_selector_input.planning_init_state.heading_angle);
    planning_init_state->set_v(msg.gap_selector_input.planning_init_state.v);
    planning_init_state->set_a(msg.gap_selector_input.planning_init_state.a);
    planning_init_state->set_curvature(msg.gap_selector_input.planning_init_state.curvature);
    planning_init_state->set_relative_time(msg.gap_selector_input.planning_init_state.relative_time);
    planning::common::FrenetState *frenet_state = planning_init_state->mutable_frenet_state();
    frenet_state->set_s(msg.gap_selector_input.planning_init_state.frenet_state.s);
    frenet_state->set_r(msg.gap_selector_input.planning_init_state.frenet_state.r);
    for (size_t i = 0; i < msg.gap_selector_input.origin_lane_s_width.size(); i++) {
        planning::common::Point2d *origin_lane_s_width = gap_selector_input->add_origin_refline_points();
        origin_lane_s_width->set_x(msg.gap_selector_input.origin_lane_s_width[i].x);
        origin_lane_s_width->set_y(msg.gap_selector_input.origin_lane_s_width[i].y);
    }
    for (size_t i = 0; i < msg.gap_selector_input.target_lane_s_width.size(); i++) {
        planning::common::Point2d *target_lane_s_width = gap_selector_input->add_origin_refline_points();
        target_lane_s_width->set_x(msg.gap_selector_input.target_lane_s_width[i].x);
        target_lane_s_width->set_y(msg.gap_selector_input.target_lane_s_width[i].y);
    }
    gap_selector_input->set_origin_lane_id(msg.gap_selector_input.origin_lane_id);
    gap_selector_input->set_target_lane_id(msg.gap_selector_input.target_lane_id);
    gap_selector_input->set_current_lane_id(msg.gap_selector_input.current_lane_id);
    gap_selector_input->set_ego_l_cur_lane(msg.gap_selector_input.ego_l_cur_lane);

    for (size_t i = 0; i < msg.gap_selector_input.current_refline_points.size(); i++) {
        planning::common::Point2d *current_refline_points = gap_selector_input->add_origin_refline_points();
        current_refline_points->set_x(msg.gap_selector_input.current_refline_points[i].x);
        current_refline_points->set_y(msg.gap_selector_input.current_refline_points[i].y);
    }

    //GapSelectorReplayInfo gap_selector_replay_info
    planning::common::GapSelectorReplayInfo *gap_selector_replay_info = obj->mutable_gap_selector_replay_info();
    gap_selector_replay_info->set_origin_lane_id(msg.gap_selector_replay_info.origin_lane_id);
    gap_selector_replay_info->set_current_lane_id(msg.gap_selector_replay_info.current_lane_id);
    gap_selector_replay_info->set_target_lane_id(msg.gap_selector_replay_info.target_lane_id);
    //GapSelectorPathSpline gap_selector_path_spline
    planning::common::GapSelectorPathSpline *gap_selector_path_spline = gap_selector_replay_info->mutable_gap_selector_path_spline();
    for (size_t i = 0; i < msg.gap_selector_replay_info.gap_selector_path_spline.x_vector_spline.size(); i++) {
        gap_selector_path_spline->add_x_vector_spline(msg.gap_selector_replay_info.gap_selector_path_spline.x_vector_spline[i]);
    }
    for (size_t i = 0; i < msg.gap_selector_replay_info.gap_selector_path_spline.y_vector_spline.size(); i++) {
        gap_selector_path_spline->add_y_vector_spline(msg.gap_selector_replay_info.gap_selector_path_spline.y_vector_spline[i]);
    }
    for (size_t i = 0; i < msg.gap_selector_replay_info.gap_selector_path_spline.s_vector_spline.size(); i++) {
        gap_selector_path_spline->add_s_vector_spline(msg.gap_selector_replay_info.gap_selector_path_spline.s_vector_spline[i]);
    }
    {
    planning::common::LonState *start_state = gap_selector_path_spline->mutable_start_state();
    start_state->set_p(msg.gap_selector_replay_info.gap_selector_path_spline.start_state.p);
    start_state->set_v(msg.gap_selector_replay_info.gap_selector_path_spline.start_state.v);
    start_state->set_a(msg.gap_selector_replay_info.gap_selector_path_spline.start_state.a);
    start_state->set_j(msg.gap_selector_replay_info.gap_selector_path_spline.start_state.j);
    start_state->set_t(msg.gap_selector_replay_info.gap_selector_path_spline.start_state.t);
    }
    {
    planning::common::Point2d *start_cart_point = gap_selector_path_spline->mutable_start_cart_point();
    start_cart_point->set_x(msg.gap_selector_replay_info.gap_selector_path_spline.start_cart_point.x);
    start_cart_point->set_y(msg.gap_selector_replay_info.gap_selector_path_spline.start_cart_point.y);
    }
    gap_selector_path_spline->set_path_spline_status(msg.gap_selector_replay_info.gap_selector_path_spline.path_spline_status);
    for (size_t i = 0; i < msg.gap_selector_replay_info.gs_traj_points.size(); i++) {
        planning::common::TrajectoryPoint *gs_traj_points = gap_selector_replay_info->add_gs_traj_points();
        gs_traj_points->set_x(msg.gap_selector_replay_info.gs_traj_points[i].x);
        gs_traj_points->set_y(msg.gap_selector_replay_info.gs_traj_points[i].y);
        gs_traj_points->set_heading_angle(msg.gap_selector_replay_info.gs_traj_points[i].heading_angle);
        gs_traj_points->set_curvature(msg.gap_selector_replay_info.gs_traj_points[i].curvature);
        gs_traj_points->set_t(msg.gap_selector_replay_info.gs_traj_points[i].t);
        gs_traj_points->set_v(msg.gap_selector_replay_info.gs_traj_points[i].v);
        gs_traj_points->set_a(msg.gap_selector_replay_info.gs_traj_points[i].a);
        gs_traj_points->set_s(msg.gap_selector_replay_info.gs_traj_points[i].s);
        gs_traj_points->set_l(msg.gap_selector_replay_info.gs_traj_points[i].l);
        gs_traj_points->set_frenet_valid(msg.gap_selector_replay_info.gs_traj_points[i].frenet_valid);
    }
    gap_selector_replay_info->set_lane_cross(msg.gap_selector_replay_info.lane_cross);
    gap_selector_replay_info->set_lc_triggered(msg.gap_selector_replay_info.lc_triggered);
    gap_selector_replay_info->set_lb_triggered(msg.gap_selector_replay_info.lb_triggered);
    gap_selector_replay_info->set_lc_in(msg.gap_selector_replay_info.lc_in);
    gap_selector_replay_info->set_lb_in(msg.gap_selector_replay_info.lb_in);
    gap_selector_replay_info->set_gs_skip(msg.gap_selector_replay_info.gs_skip);
    gap_selector_replay_info->set_lc_cancel(msg.gap_selector_replay_info.lc_cancel);
    gap_selector_replay_info->set_lc_request(msg.gap_selector_replay_info.lc_request);
    gap_selector_replay_info->set_lc_pass_time(msg.gap_selector_replay_info.lc_pass_time);
    gap_selector_replay_info->set_lc_wait_time(msg.gap_selector_replay_info.lc_wait_time);
    for (size_t i = 0; i < msg.gap_selector_replay_info.lc_request_buffer.size(); i++) {
        gap_selector_replay_info->add_lc_request_buffer(msg.gap_selector_replay_info.lc_request_buffer[i]);
    }
    for (size_t i = 0; i < msg.gap_selector_replay_info.ego_l_buffer.size(); i++) {
        gap_selector_replay_info->add_ego_l_buffer(msg.gap_selector_replay_info.ego_l_buffer[i]);
    }
    gap_selector_replay_info->set_path_requintic(msg.gap_selector_replay_info.path_requintic);
    planning::common::Point2d *cross_line_point_global = gap_selector_replay_info->mutable_cross_line_point_global();
    cross_line_point_global->set_x(msg.gap_selector_replay_info.cross_line_point_global.x);
    cross_line_point_global->set_y(msg.gap_selector_replay_info.cross_line_point_global.y);
    for (size_t i = 0; i < msg.gap_selector_replay_info.gap_list.size(); i++) {
        planning::common::Gap *gap_list = gap_selector_replay_info->add_gap_list();
        gap_list->set_front_agent_id(msg.gap_selector_replay_info.gap_list[i].front_agent_id);
        gap_list->set_rear_agent_id(msg.gap_selector_replay_info.gap_list[i].rear_agent_id);
    }
    for (size_t i = 0; i < msg.gap_selector_replay_info.obstacle_predicate_points_front_gap_car.size(); i++) {
        planning::common::ObstaclePredicatedPoint *obstacle_predicate_points_front_gap_car =
        gap_selector_replay_info->add_obstacle_predicate_points_front_gap_car();
        obstacle_predicate_points_front_gap_car->set_x(
        msg.gap_selector_replay_info.obstacle_predicate_points_front_gap_car[i].x);
        obstacle_predicate_points_front_gap_car->set_y(
        msg.gap_selector_replay_info.obstacle_predicate_points_front_gap_car[i].y);
        obstacle_predicate_points_front_gap_car->set_heading_angle(
        msg.gap_selector_replay_info.obstacle_predicate_points_front_gap_car[i].heading_angle);
        obstacle_predicate_points_front_gap_car->set_t(
        msg.gap_selector_replay_info.obstacle_predicate_points_front_gap_car[i].t);
        obstacle_predicate_points_front_gap_car->set_s(
        msg.gap_selector_replay_info.obstacle_predicate_points_front_gap_car[i].s);
        obstacle_predicate_points_front_gap_car->set_l(
        msg.gap_selector_replay_info.obstacle_predicate_points_front_gap_car[i].l);
    }
    for (size_t i = 0; i < msg.gap_selector_replay_info.obstacle_predicate_points_rear_gap_car.size(); i++) {
        planning::common::ObstaclePredicatedPoint *obstacle_predicate_points_rear_gap_car =
        gap_selector_replay_info->add_obstacle_predicate_points_rear_gap_car();
        obstacle_predicate_points_rear_gap_car->set_x(
        msg.gap_selector_replay_info.obstacle_predicate_points_rear_gap_car[i].x);
        obstacle_predicate_points_rear_gap_car->set_y(
        msg.gap_selector_replay_info.obstacle_predicate_points_rear_gap_car[i].y);
        obstacle_predicate_points_rear_gap_car->set_heading_angle(
        msg.gap_selector_replay_info.obstacle_predicate_points_rear_gap_car[i].heading_angle);
        obstacle_predicate_points_rear_gap_car->set_t(
        msg.gap_selector_replay_info.obstacle_predicate_points_rear_gap_car[i].t);
        obstacle_predicate_points_rear_gap_car->set_s(
        msg.gap_selector_replay_info.obstacle_predicate_points_rear_gap_car[i].s);
        obstacle_predicate_points_rear_gap_car->set_l(
        msg.gap_selector_replay_info.obstacle_predicate_points_rear_gap_car[i].l);
    }
    for (size_t i = 0; i < msg.gap_selector_replay_info.ego_time_optimal.size(); i++) {
        planning::common::ObstaclePredicatedPoint *ego_time_optimal =
        gap_selector_replay_info->add_ego_time_optimal();
        ego_time_optimal->set_x(
        msg.gap_selector_replay_info.ego_time_optimal[i].x);
        ego_time_optimal->set_y(
        msg.gap_selector_replay_info.ego_time_optimal[i].y);
        ego_time_optimal->set_heading_angle(
        msg.gap_selector_replay_info.ego_time_optimal[i].heading_angle);
        ego_time_optimal->set_t(
        msg.gap_selector_replay_info.ego_time_optimal[i].t);
        ego_time_optimal->set_s(
        msg.gap_selector_replay_info.ego_time_optimal[i].s);
        ego_time_optimal->set_l(
        msg.gap_selector_replay_info.ego_time_optimal[i].l);
    }
    planning::common::Gap *nearby_gap = gap_selector_replay_info->mutable_nearby_gap();
    nearby_gap->set_front_agent_id(msg.gap_selector_replay_info.nearby_gap.front_agent_id);
    nearby_gap->set_rear_agent_id(msg.gap_selector_replay_info.nearby_gap.rear_agent_id);
    gap_selector_replay_info->set_gap_selector_status(msg.gap_selector_replay_info.gap_selector_status);
    planning::common::Point2d *quintic_p0 = gap_selector_replay_info->mutable_quintic_p0();
    quintic_p0->set_x(msg.gap_selector_replay_info.quintic_p0.x);
    quintic_p0->set_y(msg.gap_selector_replay_info.quintic_p0.y);
    planning::common::Point2d *quintic_pe = gap_selector_replay_info->mutable_quintic_pe();
    quintic_pe->set_x(msg.gap_selector_replay_info.quintic_pe.x);
    quintic_pe->set_y(msg.gap_selector_replay_info.quintic_pe.y);
    planning::common::Point2d *quintic_stitched_p = gap_selector_replay_info->mutable_quintic_stitched_p();
    quintic_stitched_p->set_x(msg.gap_selector_replay_info.quintic_stitched_p.x);
    quintic_stitched_p->set_y(msg.gap_selector_replay_info.quintic_stitched_p.y);

    //TrajectoryPoint refline_info
    //planning::common::TrajectoryPoint *refline_info = obj->mutable_refline_info();
    for (size_t i = 0; i < msg.refline_info.size(); i++) {
        planning::common::TrajectoryPoint *refline_info = obj->add_refline_info();
        refline_info->set_x(msg.refline_info[i].x);
        refline_info->set_y(msg.refline_info[i].y);
        refline_info->set_heading_angle(msg.refline_info[i].heading_angle);
        refline_info->set_curvature(msg.refline_info[i].curvature);
        refline_info->set_t(msg.refline_info[i].t);
        refline_info->set_v(msg.refline_info[i].v);
        refline_info->set_a(msg.refline_info[i].a);
        refline_info->set_s(msg.refline_info[i].s);
        refline_info->set_l(msg.refline_info[i].l);
        refline_info->set_frenet_valid(msg.refline_info[i].frenet_valid);
    }

    //PlanReferenceLine[] generated_refline_info
    for (size_t i = 0; i < msg.generated_refline_info.size(); i++) {
        planning::common::ReferenceLine *generated_refline_info = obj->add_generated_refline_info();
        for (size_t j = 0; j < msg.generated_refline_info[i].virtual_lane_refline_points.size(); j++) {
        planning::common::ReferencePoint *virtual_lane_refline_points = generated_refline_info->add_virtual_lane_refline_points();
        virtual_lane_refline_points->set_track_id(msg.generated_refline_info[i].virtual_lane_refline_points[j].track_id);
        planning::common::Point2d *car_point = virtual_lane_refline_points->mutable_car_point();
        car_point->set_x(msg.generated_refline_info[i].virtual_lane_refline_points[j].car_point.x);
        car_point->set_y(msg.generated_refline_info[i].virtual_lane_refline_points[j].car_point.y);

        planning::common::Point3d *enu_point = virtual_lane_refline_points->mutable_enu_point();
        enu_point->set_x(msg.generated_refline_info[i].virtual_lane_refline_points[j].enu_point.x);
        enu_point->set_y(msg.generated_refline_info[i].virtual_lane_refline_points[j].enu_point.y);
        enu_point->set_z(msg.generated_refline_info[i].virtual_lane_refline_points[j].enu_point.z);

        planning::common::Point3d *local_point = virtual_lane_refline_points->mutable_local_point();
        local_point->set_x(msg.generated_refline_info[i].virtual_lane_refline_points[j].local_point.x);
        local_point->set_y(msg.generated_refline_info[i].virtual_lane_refline_points[j].local_point.y);
        local_point->set_z(msg.generated_refline_info[i].virtual_lane_refline_points[j].local_point.z);

        virtual_lane_refline_points->set_curvature(msg.generated_refline_info[i].virtual_lane_refline_points[j].curvature);
        virtual_lane_refline_points->set_car_heading(msg.generated_refline_info[i].virtual_lane_refline_points[j].car_heading);
        virtual_lane_refline_points->set_enu_heading(msg.generated_refline_info[i].virtual_lane_refline_points[j].enu_heading);
        virtual_lane_refline_points->set_local_heading(msg.generated_refline_info[i].virtual_lane_refline_points[j].local_heading);
        virtual_lane_refline_points->set_distance_to_left_road_border(
            msg.generated_refline_info[i].virtual_lane_refline_points[j].distance_to_left_road_border);
        virtual_lane_refline_points->set_distance_to_right_road_border(
            msg.generated_refline_info[i].virtual_lane_refline_points[j].distance_to_right_road_border);
        virtual_lane_refline_points->set_distance_to_left_lane_border(
            msg.generated_refline_info[i].virtual_lane_refline_points[j].distance_to_left_lane_border);
        virtual_lane_refline_points->set_distance_to_right_lane_border(
            msg.generated_refline_info[i].virtual_lane_refline_points[j].distance_to_right_lane_border);
        virtual_lane_refline_points->set_lane_width(msg.generated_refline_info[i].virtual_lane_refline_points[j].lane_width);
        virtual_lane_refline_points->set_speed_limit_max(msg.generated_refline_info[i].virtual_lane_refline_points[j].speed_limit_max);
        virtual_lane_refline_points->set_speed_limit_min(msg.generated_refline_info[i].virtual_lane_refline_points[j].speed_limit_min);
        virtual_lane_refline_points->set_left_road_border_type(planning::common::LineType(
            msg.generated_refline_info[i].virtual_lane_refline_points[j].left_road_border_type));
        virtual_lane_refline_points->set_right_road_border_type(planning::common::LineType(msg.generated_refline_info[i].virtual_lane_refline_points[j].right_road_border_type));
        virtual_lane_refline_points->set_left_lane_border_type(planning::common::LineType(
            msg.generated_refline_info[i].virtual_lane_refline_points[j].left_lane_border_type));
        virtual_lane_refline_points->set_right_lane_border_type(planning::common::LineType(
            msg.generated_refline_info[i].virtual_lane_refline_points[j].right_lane_border_type));
        virtual_lane_refline_points->set_is_in_intersection(msg.generated_refline_info[i].virtual_lane_refline_points[j].is_in_intersection);
        virtual_lane_refline_points->set_lane_type(planning::common::LaneType(msg.generated_refline_info[i].virtual_lane_refline_points[j].lane_type));
        virtual_lane_refline_points->set_s(msg.generated_refline_info[i].virtual_lane_refline_points[j].s);
        }
        for (size_t j = 0; j < msg.generated_refline_info[i].poly_coefficient_car.size(); j++) {
        generated_refline_info->add_poly_coefficient_car(msg.generated_refline_info[i].poly_coefficient_car[j]);
        }
    }
  }
}