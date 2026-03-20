#pragma once

#include "base_convert.h"
#include "control_debug_info.pb.h"

template <class T1>
void convert(T1 &&msg, std::shared_ptr<ControlDebugInfo::ControlDebugInfo> &obj, ConvertTypeInfo type) {
  if (type == ConvertTypeInfo::TO_ROS) {
    //convert for control_debug_info from latest proto to 2.3.6 ros is not supported.
    return;

  } else if (type == ConvertTypeInfo::TO_PROTO) {
    //
    obj->set_timestamp(msg.timestamp);
    obj->set_extra_json(msg.extra_json);
    // input_topic_timestamp_list
    ControlDebugInfo::InputTopicStampList *input_topic_timestamp_list = obj->mutable_input_topic_timestamp_list();
    // input_topic_timestamp_list->set_vehicle_service(msg.input_topic_timestamp_list.vehicle_service);
    // input_topic_timestamp_list->set_localization(msg.input_topic_timestamp_list.localization);
    // input_topic_timestamp_list->set_planning_output(msg.input_topic_timestamp_list.planning_output);
    // input_topic_timestamp_list->set_func_state_machine(msg.input_topic_timestamp_list.func_state_machine);
    input_topic_timestamp_list->set_topic1_timestamp(0);
    input_topic_timestamp_list->set_topic2_timestamp(0);
    // lat_mpc_input
    ControlDebugInfo::LateralMpcInput *lat_mpc_input = obj->mutable_lat_mpc_input();
    lat_mpc_input->set_q_y(msg.lat_mpc_input.q_y);
    lat_mpc_input->set_q_phi(msg.lat_mpc_input.q_phi);
    lat_mpc_input->set_q_omega(msg.lat_mpc_input.q_omega);
    lat_mpc_input->set_q_phi_wn(msg.lat_mpc_input.q_phi_WN);
    lat_mpc_input->set_q_y_wn(msg.lat_mpc_input.q_y_WN);
    lat_mpc_input->set_omega_limit(msg.lat_mpc_input.omega_limit);
    lat_mpc_input->set_delta_limit(msg.lat_mpc_input.delta_limit);
    //
    ControlDebugInfo::LateralMpcState *lat_init_state = lat_mpc_input->mutable_init_state();
    lat_init_state->set_dx(msg.lat_mpc_input.init_state.dx);
    lat_init_state->set_dy(msg.lat_mpc_input.init_state.dy);
    lat_init_state->set_dphi(msg.lat_mpc_input.init_state.dphi);
    lat_init_state->set_delta(msg.lat_mpc_input.init_state.delta);
    for (size_t i = 0; i < msg.lat_mpc_input.curv_factor_vec.size(); i++) {
      lat_mpc_input->add_curv_factor_vec(msg.lat_mpc_input.curv_factor_vec[i]);
    }
    for (size_t i = 0; i < msg.lat_mpc_input.dx_ref_mpc_vec.size(); i++) {
      lat_mpc_input->add_dx_ref_mpc_vec(msg.lat_mpc_input.dx_ref_mpc_vec[i]);
    }
    for (size_t i = 0; i < msg.lat_mpc_input.dy_ref_mpc_vec.size(); i++) {
      lat_mpc_input->add_dy_ref_mpc_vec(msg.lat_mpc_input.dy_ref_mpc_vec[i]);
    }
    for (size_t i = 0; i < msg.lat_mpc_input.dphi_ref_mpc_vec.size(); i++) {
      lat_mpc_input->add_dphi_ref_mpc_vec(msg.lat_mpc_input.dphi_ref_mpc_vec[i]);
    }
    for (size_t i = 0; i < msg.lat_mpc_input.vel_lat_ctrl_mpc_vec.size(); i++) {
      lat_mpc_input->add_vel_lat_ctrl_mpc_vec(msg.lat_mpc_input.vel_lat_ctrl_mpc_vec[i]);
    }
    // lat_mpc_output
    ControlDebugInfo::LateralMpcOutput *lat_mpc_output = obj->mutable_lat_mpc_output();
    for (size_t i = 0; i < msg.lat_mpc_output.time_mpc_vec.size(); i++) {
      lat_mpc_output->add_time_mpc_vec(msg.lat_mpc_output.time_mpc_vec[i]);
    }
    for (size_t i = 0; i < msg.lat_mpc_output.dx_mpc_vec.size(); i++) {
      lat_mpc_output->add_dx_mpc_vec(msg.lat_mpc_output.dx_mpc_vec[i]);
    }
    for (size_t i = 0; i < msg.lat_mpc_output.dy_mpc_vec.size(); i++) {
      lat_mpc_output->add_dy_mpc_vec(msg.lat_mpc_output.dy_mpc_vec[i]);
    }
    for (size_t i = 0; i < msg.lat_mpc_output.dphi_mpc_vec.size(); i++) {
      lat_mpc_output->add_dphi_mpc_vec(msg.lat_mpc_output.dphi_mpc_vec[i]);
    }
    for (size_t i = 0; i < msg.lat_mpc_output.delta_mpc_vec.size(); i++) {
      lat_mpc_output->add_delta_mpc_vec(msg.lat_mpc_output.delta_mpc_vec[i]);
    }
    for (size_t i = 0; i < msg.lat_mpc_output.omega_mpc_vec.size(); i++) {
      lat_mpc_output->add_omega_mpc_vec(msg.lat_mpc_output.omega_mpc_vec[i]);
    }
    lat_mpc_output->set_status(msg.lat_mpc_output.status);
    lat_mpc_output->set_iteration(msg.lat_mpc_output.iteration);
    // // lon_mpc_input
    // ControlDebugInfo::LonMpcInput *lon_mpc_input = obj->mutable_lon_mpc_input();
    // lon_mpc_input->set_q_ref_s(msg.lon_mpc_input.q_ref_s);
    // lon_mpc_input->set_q_ref_v(msg.lon_mpc_input.q_ref_v);
    // lon_mpc_input->set_q_a(msg.lon_mpc_input.q_a);
    // lon_mpc_input->set_q_jerk(msg.lon_mpc_input.q_jerk);
    // lon_mpc_input->set_q_a_bound(msg.lon_mpc_input.q_a_bound);
    // lon_mpc_input->set_q_jerk_bound(msg.lon_mpc_input.q_jerk_bound);
    // lon_mpc_input->set_q_v_non_negative(msg.lon_mpc_input.q_v_non_negative);
    // lon_mpc_input->set_v_min(msg.lon_mpc_input.v_min);
    // lon_mpc_input->set_v_max(msg.lon_mpc_input.v_max);
    // lon_mpc_input->set_a_min(msg.lon_mpc_input.a_min);
    // lon_mpc_input->set_a_max(msg.lon_mpc_input.a_max);
    // lon_mpc_input->set_jerk_min(msg.lon_mpc_input.jerk_min);
    // lon_mpc_input->set_jerk_max(msg.lon_mpc_input.jerk_max);
    // //
    // ControlDebugInfo::LonMpcState *lon_init_state = lon_mpc_input->mutable_init_state();
    // lon_init_state->set_s(msg.lon_mpc_input.init_state.s);
    // lon_init_state->set_v(msg.lon_mpc_input.init_state.v);
    // lon_init_state->set_a(msg.lon_mpc_input.init_state.a);
    // for (size_t i = 0; i < msg.lon_mpc_input.s_ref_mpc_vec.size(); i++) {
    //   lon_mpc_input->add_s_ref_mpc_vec(msg.lon_mpc_input.s_ref_mpc_vec[i]);
    // }
    // for (size_t i = 0; i < msg.lon_mpc_input.v_ref_mpc_vec.size(); i++) {
    //   lon_mpc_input->add_v_ref_mpc_vec(msg.lon_mpc_input.v_ref_mpc_vec[i]);
    // }
    // // lon_mpc_output
    // ControlDebugInfo::LonMpcOutput *lon_mpc_output = obj->mutable_lon_mpc_output();
    // for (size_t i = 0; i < msg.lon_mpc_output.time_mpc_vec.size(); i++) {
    //   lon_mpc_output->add_time_mpc_vec(msg.lon_mpc_output.time_mpc_vec[i]);
    // }
    // for (size_t i = 0; i < msg.lon_mpc_output.s_mpc_vec.size(); i++) {
    //   lon_mpc_output->add_s_mpc_vec(msg.lon_mpc_output.s_mpc_vec[i]);
    // }
    // for (size_t i = 0; i < msg.lon_mpc_output.v_mpc_vec.size(); i++) {
    //   lon_mpc_output->add_v_mpc_vec(msg.lon_mpc_output.v_mpc_vec[i]);
    // }
    // for (size_t i = 0; i < msg.lon_mpc_output.a_mpc_vec.size(); i++) {
    //   lon_mpc_output->add_a_mpc_vec(msg.lon_mpc_output.a_mpc_vec[i]);
    // }
    // for (size_t i = 0; i < msg.lon_mpc_output.jerk_mpc_vec.size(); i++) {
    //   lon_mpc_output->add_jerk_mpc_vec(msg.lon_mpc_output.jerk_mpc_vec[i]);
    // }
    // lon_mpc_output->set_status(msg.lon_mpc_output.status);
    // lon_mpc_output->set_iteration(msg.lon_mpc_output.iteration);
  }
}

// void LocalizationDebugInfoConvert(std::shared_ptr<LocalizationDebugInfo::LocalizationDebugInfo> &obj,
//                                   proto_msgs::LocalizationDebugInfo::ConstPtr msg) {
//   obj->set_timestamp(msg.timestamp);
//   obj->set_extra_json(msg.extra_json);
//   obj->set_version(msg.version);
// }

// void PredictionDebugInfoConvert(std::shared_ptr<PredictionDebug::PredictionDebugInfo> &obj,
//                                 proto_msgs::PredictionDebugInfo::ConstPtr msg) {
//   obj->set_fusion_object_stamp(msg.fusion_object_stamp);
//   obj->set_localization_stamp(msg.localization_stamp);
//   obj->set_prediction_result_stamp(msg.prediction_result_stamp);
//   obj->set_fusion_objects_num(msg.fusion_objects_num);
//   obj->set_fusion_objects_input_num(msg.fusion_objects_input_num);
//   obj->set_object_num_output(msg.object_num_output);
//   obj->set_process_cost_time(msg.process_cost_time);
// }
