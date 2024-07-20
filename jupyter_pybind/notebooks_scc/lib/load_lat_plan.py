from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *
from lib.load_ros_bag import is_match_planning, is_bag_main

import numpy as np
import time
import ipywidgets
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from IPython.core.display import display, HTML
from bokeh.models import Label
import ipywidgets as widgets
from IPython.display import display
from ipywidgets import Button, HBox
from IPython.display import clear_output
import time
import threading
import ipywidgets
from collections import namedtuple
from functools import  partial
from bokeh.models import ColumnDataSource
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, HoverTool

car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()

def update_lat_plan_data(fig7, bag_loader, bag_time, local_view_data, lat_plan_data, g_is_display_enu = False):
  road_msg = find_nearest(bag_loader.road_msg, bag_time)
  vs_msg = find_nearest(bag_loader.vs_msg, bag_time)
  loc_msg = local_view_data['data_msg']['loc_msg']
  plan_msg = local_view_data['data_msg']['plan_msg']
  plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
  plan_debug_json_msg = local_view_data['data_msg']['plan_debug_json_msg']
  ctrl_msg = local_view_data['data_msg']['ctrl_msg']

  input_topic_timestamp = plan_debug_msg.input_topic_timestamp
  fusion_road_timestamp = input_topic_timestamp.fusion_road
  if is_bag_main:
    localization_timestamp = input_topic_timestamp.localization_estimate
  else:
    localization_timestamp = input_topic_timestamp.localization

  if is_match_planning:
    road_msg_tmp = find(bag_loader.road_msg, fusion_road_timestamp)
    if road_msg_tmp != None:
      road_msg = road_msg_tmp
    else:
      print("find road_msg error! use nearest road_msg!")
    loc_msg_tmp = find(bag_loader.loc_msg, localization_timestamp)
    if loc_msg_tmp != None:
      loc_msg = loc_msg_tmp
    else:
      print("find loc_msg error! use nearest loc_msg!")

  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = loc_msg.position.position_boot.x
    cur_pos_yn = loc_msg.position.position_boot.y
    cur_yaw = loc_msg.orientation.euler_boot.yaw
    planning_json = plan_debug_json_msg
    planning_debug = plan_debug_msg

    debug1, debug2 = load_lat_common(planning_debug, planning_json)
    print(debug2)
    ego_xn, ego_yn = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.loc_msg['data'])):
      pos_xn_i = bag_loader.loc_msg['data'][i].position.position_boot.x
      pos_yn_i = bag_loader.loc_msg['data'][i].position.position_boot.y

      ego_xn.append(pos_xn_i)
      ego_yn.append(pos_yn_i)

    lat_plan_data['data_ego'].data.update({
      'ego_xn': ego_xn,
      'ego_yn': ego_yn,
    })
    if g_is_display_enu:
      car_xn = []
      car_yn = []
      for i in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], cur_pos_xn, cur_pos_yn, cur_yaw)
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)

      lat_plan_data['data_car'].data.update({
        'car_xn': car_xn,
        'car_yn': car_yn,
      })
    else:
      lat_plan_data['data_car'].data.update({
        'car_xn': car_xb,
        'car_yn': car_yb,
      })

    if not g_is_display_enu:
      car_xn = []
      car_yn = []
      for i in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], cur_pos_xn, cur_pos_yn, cur_yaw)
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)

      lat_plan_data['data_car'].data.update({
        'car_xn2': car_xn,
        'car_yn2': car_yn,
      })
      lat_plan_data['data_ego_pos_point'].data.update({
        'ego_pos_point_x': [cur_pos_xn],
        'ego_pos_point_y': [cur_pos_yn],
        'ego_pos_point_theta': [cur_yaw],
      })
    else:
      lat_plan_data['data_car'].data.update({
        'car_xn2': car_xb,
        'car_yn2': car_yb,
      })
      lat_plan_data['data_ego_pos_point'].data.update({
        'ego_pos_point_x': [0],
        'ego_pos_point_y': [0],
        'ego_pos_point_theta': [0],
      })

    try:
      json_pos_x = planning_json['ego_pos_x']
      json_pos_y = planning_json['ego_pos_y']
      json_yaw = planning_json['ego_pos_yaw']
      coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
    except:
      coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

  if bag_loader.plan_debug_msg['enable'] == True:
    lat_motion_plan_input = plan_debug_msg.lateral_motion_planning_input

    lat_init_state = lat_motion_plan_input.init_state
    init_state_x = lat_init_state.x
    init_state_y = lat_init_state.y
    init_state_theta = lat_init_state.theta
    init_state_delta = lat_init_state.delta
    lon_init_state = plan_debug_msg.longitudinal_motion_planning_input.init_state
    init_state_s = lon_init_state.s
    init_state_v = lon_init_state.v
    init_state_a = lon_init_state.a
    replan_status = plan_debug_json_msg["replan_status"]
    if g_is_display_enu:
      init_state_x, init_state_y = coord_tf.global_to_local([init_state_x], [init_state_y])
      init_state_theta -= loc_msg.orientation.euler_boot.yaw

    lat_plan_data['data_init_pos_point'].data.update({
      'init_pos_point_y': [init_state_y],
      'init_pos_point_x': [init_state_x],
      'init_pos_point_theta': [init_state_theta],
      'init_state_x': [lat_init_state.x],
      'init_state_y': [lat_init_state.y],
      'init_state_theta': [lat_init_state.theta],
      'init_state_delta': [init_state_delta],
      'init_state_s': [init_state_s],
      'init_state_v': [init_state_v],
      'init_state_a': [init_state_a],
      'replan_status': [replan_status],
    })

    if g_is_display_enu:
      ref_x, ref_y = lat_motion_plan_input.ref_x_vec, lat_motion_plan_input.ref_y_vec

      soft_upper_bound_x0_vec, soft_upper_bound_y0_vec = lat_motion_plan_input.soft_upper_bound_x0_vec, \
        lat_motion_plan_input.soft_upper_bound_y0_vec

      soft_lower_bound_x0_vec, soft_lower_bound_y0_vec = lat_motion_plan_input.soft_lower_bound_x0_vec, \
        lat_motion_plan_input.soft_lower_bound_y0_vec

      soft_upper_bound_x1_vec, soft_upper_bound_y1_vec = lat_motion_plan_input.soft_upper_bound_x1_vec, \
        lat_motion_plan_input.soft_upper_bound_y1_vec

      soft_lower_bound_x1_vec, soft_lower_bound_y1_vec = lat_motion_plan_input.soft_lower_bound_x1_vec, \
        lat_motion_plan_input.soft_lower_bound_y1_vec

      hard_upper_bound_x0_vec, hard_upper_bound_y0_vec = lat_motion_plan_input.hard_upper_bound_x0_vec, \
        lat_motion_plan_input.hard_upper_bound_y0_vec

      hard_lower_bound_x0_vec, hard_lower_bound_y0_vec = lat_motion_plan_input.hard_lower_bound_x0_vec, \
        lat_motion_plan_input.hard_lower_bound_y0_vec

      hard_upper_bound_x1_vec, hard_upper_bound_y1_vec = lat_motion_plan_input.hard_upper_bound_x1_vec, \
        lat_motion_plan_input.hard_upper_bound_y1_vec

      hard_lower_bound_x1_vec, hard_lower_bound_y1_vec = lat_motion_plan_input.hard_lower_bound_x1_vec, \
        lat_motion_plan_input.hard_lower_bound_y1_vec

      if len(soft_upper_bound_x0_vec) > 1:
        soft_upper_bound_x0_vec[len(soft_upper_bound_x0_vec) - 1] = soft_upper_bound_x1_vec[len(soft_upper_bound_x1_vec) - 1]
        soft_upper_bound_y0_vec[len(soft_upper_bound_y0_vec) - 1] = soft_upper_bound_y1_vec[len(soft_upper_bound_y1_vec) - 1]
        soft_lower_bound_x0_vec[len(soft_lower_bound_x0_vec) - 1] = soft_lower_bound_x1_vec[len(soft_lower_bound_x1_vec) - 1]
        soft_lower_bound_y0_vec[len(soft_lower_bound_y0_vec) - 1] = soft_lower_bound_y1_vec[len(soft_lower_bound_y1_vec) - 1]
        hard_upper_bound_x0_vec[len(hard_upper_bound_x0_vec) - 1] = hard_upper_bound_x1_vec[len(hard_upper_bound_x1_vec) - 1]
        hard_upper_bound_y0_vec[len(hard_upper_bound_y0_vec) - 1] = hard_upper_bound_y1_vec[len(hard_upper_bound_y1_vec) - 1]
        hard_lower_bound_x0_vec[len(hard_lower_bound_x0_vec) - 1] = hard_lower_bound_x1_vec[len(hard_lower_bound_x1_vec) - 1]
        hard_lower_bound_y0_vec[len(hard_lower_bound_y0_vec) - 1] = hard_lower_bound_y1_vec[len(hard_lower_bound_y1_vec) - 1]
    else:
      ref_x, ref_y = coord_tf.global_to_local(lat_motion_plan_input.ref_x_vec, lat_motion_plan_input.ref_y_vec)

      soft_upper_bound_x0_vec, soft_upper_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.soft_upper_bound_x0_vec, \
        lat_motion_plan_input.soft_upper_bound_y0_vec)

      soft_lower_bound_x0_vec, soft_lower_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.soft_lower_bound_x0_vec, \
        lat_motion_plan_input.soft_lower_bound_y0_vec)

      soft_upper_bound_x1_vec, soft_upper_bound_y1_vec = coord_tf.global_to_local(lat_motion_plan_input.soft_upper_bound_x1_vec, \
        lat_motion_plan_input.soft_upper_bound_y1_vec)

      soft_lower_bound_x1_vec, soft_lower_bound_y1_vec = coord_tf.global_to_local(lat_motion_plan_input.soft_lower_bound_x1_vec, \
        lat_motion_plan_input.soft_lower_bound_y1_vec)

      hard_upper_bound_x0_vec, hard_upper_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.hard_upper_bound_x0_vec, \
        lat_motion_plan_input.hard_upper_bound_y0_vec)

      hard_lower_bound_x0_vec, hard_lower_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.hard_lower_bound_x0_vec, \
        lat_motion_plan_input.hard_lower_bound_y0_vec)

      hard_upper_bound_x1_vec, hard_upper_bound_y1_vec = coord_tf.global_to_local(lat_motion_plan_input.hard_upper_bound_x1_vec, \
        lat_motion_plan_input.hard_upper_bound_y1_vec)

      hard_lower_bound_x1_vec, hard_lower_bound_y1_vec = coord_tf.global_to_local(lat_motion_plan_input.hard_lower_bound_x1_vec, \
        lat_motion_plan_input.hard_lower_bound_y1_vec)

      if len(soft_upper_bound_x0_vec) > 1:
        soft_upper_bound_x0_vec[len(soft_upper_bound_x0_vec) - 1] = soft_upper_bound_x1_vec[len(soft_upper_bound_x1_vec) - 1]
        soft_upper_bound_y0_vec[len(soft_upper_bound_y0_vec) - 1] = soft_upper_bound_y1_vec[len(soft_upper_bound_y1_vec) - 1]
        soft_lower_bound_x0_vec[len(soft_lower_bound_x0_vec) - 1] = soft_lower_bound_x1_vec[len(soft_lower_bound_x1_vec) - 1]
        soft_lower_bound_y0_vec[len(soft_lower_bound_y0_vec) - 1] = soft_lower_bound_y1_vec[len(soft_lower_bound_y1_vec) - 1]
        hard_upper_bound_x0_vec[len(hard_upper_bound_x0_vec) - 1] = hard_upper_bound_x1_vec[len(hard_upper_bound_x1_vec) - 1]
        hard_upper_bound_y0_vec[len(hard_upper_bound_y0_vec) - 1] = hard_upper_bound_y1_vec[len(hard_upper_bound_y1_vec) - 1]
        hard_lower_bound_x0_vec[len(hard_lower_bound_x0_vec) - 1] = hard_lower_bound_x1_vec[len(hard_lower_bound_x1_vec) - 1]
        hard_lower_bound_y0_vec[len(hard_lower_bound_y0_vec) - 1] = hard_lower_bound_y1_vec[len(hard_lower_bound_y1_vec) - 1]

    if len(soft_upper_bound_x0_vec) == 0 or plan_msg.trajectory.target_reference.lateral_maneuver_gear == 2:
      soft_upper_bound_x0_vec = ref_x
      soft_upper_bound_y0_vec = ref_y
      soft_lower_bound_x0_vec = ref_x
      soft_lower_bound_y0_vec = ref_y
      hard_upper_bound_x0_vec = ref_x
      hard_upper_bound_y0_vec = ref_y
      hard_lower_bound_x0_vec = ref_x
      hard_lower_bound_y0_vec = ref_y

    bound_t_vec = []
    bound_s_vec = []
    soft_upper_bound_vec = []
    soft_lower_bound_vec = []
    hard_upper_bound_vec = []
    hard_lower_bound_vec = []
    soft_upper_bound_id_vec = []
    soft_lower_bound_id_vec = []
    hard_upper_bound_id_vec = []
    hard_lower_bound_id_vec = []
    soft_upper_bound_type_vec = []
    soft_lower_bound_type_vec = []
    hard_upper_bound_type_vec = []
    hard_lower_bound_type_vec = []
    try:
      lat_behavior_debug_info = plan_debug_msg.lateral_behavior_debug_info
      for i in range(len(lat_behavior_debug_info.bound_s_vec)):
        bound_t_vec.append(round(i * 0.2, 2))
        bound_s_vec.append(round(lat_behavior_debug_info.bound_s_vec[i], 3))
        soft_upper_bound_vec.append(round(lat_behavior_debug_info.soft_upper_bound_info_vec[i].upper, 3))
        soft_lower_bound_vec.append(round(lat_behavior_debug_info.soft_lower_bound_info_vec[i].lower, 3))
        hard_upper_bound_vec.append(round(lat_behavior_debug_info.hard_upper_bound_info_vec[i].upper, 3))
        hard_lower_bound_vec.append(round(lat_behavior_debug_info.hard_lower_bound_info_vec[i].lower, 3))
        soft_upper_bound_id_vec.append(lat_behavior_debug_info.soft_upper_bound_info_vec[i].bound_info.id)
        soft_lower_bound_id_vec.append(lat_behavior_debug_info.soft_lower_bound_info_vec[i].bound_info.id)
        hard_upper_bound_id_vec.append(lat_behavior_debug_info.hard_upper_bound_info_vec[i].bound_info.id)
        hard_lower_bound_id_vec.append(lat_behavior_debug_info.hard_lower_bound_info_vec[i].bound_info.id)
        soft_upper_bound_type_vec.append(lat_behavior_debug_info.soft_upper_bound_info_vec[i].bound_info.type)
        soft_lower_bound_type_vec.append(lat_behavior_debug_info.soft_lower_bound_info_vec[i].bound_info.type)
        hard_upper_bound_type_vec.append(lat_behavior_debug_info.hard_upper_bound_info_vec[i].bound_info.type)
        hard_lower_bound_type_vec.append(lat_behavior_debug_info.hard_lower_bound_info_vec[i].bound_info.type)
    except:
      for i in range(len(soft_upper_bound_x0_vec)):
        bound_t_vec.append(round(i * 0.2, 2))
        bound_s_vec.append(-100)
        soft_upper_bound_vec.append(-100)
        soft_lower_bound_vec.append(-100)
        hard_upper_bound_vec.append(-100)
        hard_lower_bound_vec.append(-100)
        soft_upper_bound_id_vec.append(-100)
        soft_lower_bound_id_vec.append(-100)
        hard_upper_bound_id_vec.append(-100)
        hard_lower_bound_id_vec.append(-100)
        soft_upper_bound_type_vec.append(-100)
        soft_lower_bound_type_vec.append(-100)
        hard_upper_bound_type_vec.append(-100)
        hard_lower_bound_type_vec.append(-100)
      print("no lateral_behavior_debug_info!")

    lat_plan_data['data_lat_motion_plan_input'].data.update({
      'ref_x': ref_x,
      'ref_y': ref_y,
      'ref_xn': lat_motion_plan_input.ref_x_vec,
      'ref_yn': lat_motion_plan_input.ref_y_vec,
      'soft_upper_bound_x0_vec': soft_upper_bound_x0_vec,
      'soft_upper_bound_y0_vec': soft_upper_bound_y0_vec,
      'soft_lower_bound_x0_vec': soft_lower_bound_x0_vec,
      'soft_lower_bound_y0_vec': soft_lower_bound_y0_vec,

      'hard_upper_bound_x0_vec': hard_upper_bound_x0_vec,
      'hard_upper_bound_y0_vec': hard_upper_bound_y0_vec,
      'hard_lower_bound_x0_vec': hard_lower_bound_x0_vec,
      'hard_lower_bound_y0_vec': hard_lower_bound_y0_vec,

      'bound_t_vec': bound_t_vec,
      'bound_s_vec': bound_s_vec,
      'soft_upper_bound_vec': soft_upper_bound_vec,
      'soft_lower_bound_vec': soft_lower_bound_vec,
      'hard_upper_bound_vec': hard_upper_bound_vec,
      'hard_lower_bound_vec': hard_lower_bound_vec,
      'soft_upper_bound_id_vec': soft_upper_bound_id_vec,
      'soft_lower_bound_id_vec': soft_lower_bound_id_vec,
      'hard_upper_bound_id_vec': hard_upper_bound_id_vec,
      'hard_lower_bound_id_vec': hard_lower_bound_id_vec,
      'soft_upper_bound_type_vec': soft_upper_bound_type_vec,
      'soft_lower_bound_type_vec': soft_lower_bound_type_vec,
      'hard_upper_bound_type_vec': hard_upper_bound_type_vec,
      'hard_lower_bound_type_vec': hard_lower_bound_type_vec,
    })

    if g_is_display_enu:
      raw_refline_x, raw_refline_y = planning_json['raw_refline_x_vec'], \
        planning_json['raw_refline_y_vec']
    else:
      raw_refline_x, raw_refline_y = coord_tf.global_to_local(planning_json['raw_refline_x_vec'], \
        planning_json['raw_refline_y_vec'])

    lat_plan_data['data_refline'].data.update({
      'raw_refline_x': raw_refline_x,
      'raw_refline_y': raw_refline_y,
    })

    lat_motion_plan_output = plan_debug_msg.lateral_motion_planning_output
    if g_is_display_enu:
      x_vec, y_vec = lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec
    else:
      x_vec, y_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)
    time_vec = lat_motion_plan_output.time_vec

    ref_x_vec = []
    ref_y_vec = []
    ref_theta_deg_vec = []
    theta_deg_vec = []
    steer_deg_vec = []
    steer_dot_deg_vec =[]
    acc_upper_bound = []
    acc_lower_bound = []
    jerk_upper_bound = []
    jerk_lower_bound = []
    steer_deg_upper_bound = []
    steer_deg_lower_bound = []
    steer_dot_deg_upper_bound = []
    steer_dot_deg_lower_bound = []

    delta_bound = 360.0 / 13.0 / 57.3
    omega_bound = 240.0 / 13.0 / 57.3
    try:
      delta_bound = min(delta_bound, lat_motion_plan_input.acc_bound / (lat_motion_plan_input.curv_factor * vs_msg.vehicle_speed * vs_msg.vehicle_speed))
      omega_bound = min(omega_bound, lat_motion_plan_input.jerk_bound / (lat_motion_plan_input.curv_factor * vs_msg.vehicle_speed * vs_msg.vehicle_speed))
    except:
      print("no lat_motion_plan_input!!")

    for i in range(len(time_vec)):
      ref_x_vec.append(lat_motion_plan_input.ref_x_vec[i])
      ref_y_vec.append(lat_motion_plan_input.ref_y_vec[i])
      ref_theta_deg_vec.append(lat_motion_plan_input.ref_theta_vec[i] * 57.3)
      theta_deg_vec.append(lat_motion_plan_output.theta_vec[i] * 57.3)
      steer_deg_vec.append(lat_motion_plan_output.delta_vec[i] * 57.3 * 13.0)
      steer_dot_deg_vec.append(lat_motion_plan_output.omega_vec[i] * 57.3 * 13.0)
      acc_upper_bound.append(lat_motion_plan_input.acc_bound)
      acc_lower_bound.append(-lat_motion_plan_input.acc_bound)
      jerk_upper_bound.append(lat_motion_plan_input.jerk_bound)
      jerk_lower_bound.append(-lat_motion_plan_input.jerk_bound)
      steer_deg_upper_bound.append((delta_bound * 57.3 * 13.0))
      steer_deg_lower_bound.append(-(delta_bound * 57.3 * 13.0))
      steer_dot_deg_upper_bound.append((omega_bound * 57.3 * 13.0))
      steer_dot_deg_lower_bound.append(-(omega_bound * 57.3 * 13.0))

    if not g_is_display_enu:
      ref_x_vec, ref_y_vec = coord_tf.global_to_local(ref_x_vec, ref_y_vec)

    acc_vec = lat_motion_plan_output.acc_vec
    jerk_vec = lat_motion_plan_output.jerk_vec

    lat_plan_data['data_lat_motion_plan_output'].data.update({
      'time_vec': time_vec,
      'ref_x_vec': ref_x_vec,
      'x_vec': x_vec,
      'ref_y_vec': ref_y_vec,
      'y_vec': y_vec,
      'xn_vec': lat_motion_plan_output.x_vec,
      'yn_vec': lat_motion_plan_output.y_vec,
      'ref_theta_deg_vec': ref_theta_deg_vec,
      'theta_deg_vec': theta_deg_vec,
      'steer_deg_vec': steer_deg_vec,
      'steer_dot_deg_vec': steer_dot_deg_vec,
      'acc_vec': acc_vec,
      'jerk_vec': jerk_vec,
      'acc_upper_bound': acc_upper_bound,
      'acc_lower_bound': acc_lower_bound,
      'jerk_upper_bound': jerk_upper_bound,
      'jerk_lower_bound': jerk_lower_bound,
      'steer_deg_upper_bound': steer_deg_upper_bound,
      'steer_deg_lower_bound': steer_deg_lower_bound,
      'steer_dot_deg_upper_bound': steer_dot_deg_upper_bound,
      'steer_dot_deg_lower_bound': steer_dot_deg_lower_bound,
    })

    # assembled_delta = []
    # assembled_omega = []
    # for i in range(len(planning_json['assembled_delta'])):
    #   assembled_delta.append(planning_json['assembled_delta'][i] * 57.3 * 15.7)
    #   assembled_omega.append(planning_json['assembled_omega'][i] * 57.3 * 15.7)

    print("dbw_status = ", planning_json['dbw_status'])
    print("replan_status = ", planning_json['replan_status'])
    print("lat_err = ", planning_json['lat_err'])
    print("theta_err = ", planning_json['theta_err'])
    print("lon_err = ", planning_json['lon_err'])
    print("dist_err = ", planning_json['dist_err'])
    print("solver_condition = ", planning_json['solver_condition'])
    print("iLqr_lat_update_time = ", planning_json['iLqr_lat_update_time'], " ms")

  if bag_loader.plan_msg['enable'] == True:
    trajectory = plan_msg.trajectory
    if trajectory.trajectory_type == 0: # 实时轨迹
      try:
        planning_polynomial = trajectory.target_reference.polynomial
        plan_traj_x, plan_traj_y = gen_line(planning_polynomial[3],planning_polynomial[2], planning_polynomial[1], planning_polynomial[0], 0, 50)
        if not g_is_display_enu:
          plan_traj_x, plan_traj_y = coord_tf.local_to_global(plan_traj_x, plan_traj_y)
      except:
        plan_traj_x, plan_traj_y = [], []
    else:
      plan_x = []
      plan_y = []
      for i in range(len(trajectory.trajectory_points)):
        plan_x.append(trajectory.trajectory_points[i].x)
        plan_y.append(trajectory.trajectory_points[i].y)

      if not g_is_display_enu:
        plan_traj_x, plan_traj_y = planning_json['traj_x_vec'], planning_json['traj_y_vec']
      else:
        plan_traj_x, plan_traj_y = coord_tf.global_to_local(planning_json['traj_x_vec'], planning_json['traj_y_vec'])

      lat_plan_data['data_planning_n'].data.update({
        'plan_traj_xn':planning_json['traj_x_vec'],
        'plan_traj_yn':planning_json['traj_y_vec'],
      })

    lat_plan_data['data_planning'].data.update({
      'plan_traj_y' : plan_traj_y,
      'plan_traj_x' : plan_traj_x,
    })

  # 加载车道线信息
  if plan_msg.trajectory.trajectory_type == 0: # 实时轨迹
    is_enu_to_car = False
  else:
    is_enu_to_car = True

  not_g_is_display_enu = g_is_display_enu
  if g_is_display_enu :
    not_g_is_display_enu = False
  else:
    not_g_is_display_enu = True
  if bag_loader.road_msg['enable'] == True:
    print("fusion road local point valid: ", road_msg.local_point_valid)
    # load lane info
    try:
      line_info_list = load_lane_lines(road_msg, is_enu_to_car, loc_msg, not_g_is_display_enu)
    except:
      print("vis road_msg error")

    # update lane info
    data_lane_dict = {
      0:lat_plan_data['data_lane_0'],
      1:lat_plan_data['data_lane_1'],
      2:lat_plan_data['data_lane_2'],
      3:lat_plan_data['data_lane_3'],
      4:lat_plan_data['data_lane_4'],
      5:lat_plan_data['data_lane_5'],
      6:lat_plan_data['data_lane_6'],
      7:lat_plan_data['data_lane_7'],
      8:lat_plan_data['data_lane_8'],
      9:lat_plan_data['data_lane_9'],
    }
    data_center_line_dict = {
      0:lat_plan_data['data_center_line_0'],
      1:lat_plan_data['data_center_line_1'],
      2:lat_plan_data['data_center_line_2'],
      3:lat_plan_data['data_center_line_3'],
      4:lat_plan_data['data_center_line_4'],
    }

    for i in range(10):
      try:
        if line_info_list[i]['type'] == ['dashed']:
          fig7.renderers[0 + i].glyph.line_dash = 'dashed'
        else:
          fig7.renderers[0 + i].glyph.line_dash = 'solid'
        data_lane = data_lane_dict[i]
        data_lane.data.update({
          'line_{}_x'.format(i): line_info_list[i]['line_x_vec'],
          'line_{}_y'.format(i): line_info_list[i]['line_y_vec'],
        })
      except:
        # print('error')
        pass

    center_line_list = load_lane_center_lines(road_msg, is_enu_to_car, loc_msg, not_g_is_display_enu)
    # print(center_line_list)

    for i in range(5):
      # try:
        # if (trajectory.trajectory_type == 0) or (trajectory.trajectory_type == 1 and trajectory.target_reference.lateral_maneuver_gear == 2) :
        data_center_line = data_center_line_dict[i]
        data_center_line.data.update({
          'center_line_{}_x'.format(i): center_line_list[i]['line_x_vec'],
          'center_line_{}_y'.format(i): center_line_list[i]['line_y_vec'],
        })

  # load control
  if bag_loader.ctrl_msg['enable'] == True:
    mpc_dx, mpc_dy, mpc_dtheta = generate_control(ctrl_msg, loc_msg, not g_is_display_enu)
    lat_plan_data['data_control'].data.update({
        'mpc_dx' : mpc_dx,
        'mpc_dy' : mpc_dy,
    })

def load_lateral_offset(bag_loader):
  data_fig = ColumnDataSource(data ={
    'lateral_offset_1': [],
    'lateral_offset_2': [],
    'avoid_ways': [],
    'frame_num_y': [],
  })

  if bag_loader.plan_debug_msg['enable'] == True:
      lateral_offsets = []
      smooth_lateral_offsets = []
      frame_nums = []
      avoid_ways = []
      for i, plan_json_debug in enumerate(bag_loader.plan_debug_msg['json']):
        plan_debug_msg = bag_loader.plan_debug_msg['data'][i]
        frame_nums.append(plan_debug_msg.frame_info.frame_num)
        lateral_offsets.append(plan_json_debug['lat_offset'])
        smooth_lateral_offsets.append(plan_json_debug['smooth_lateral_offset'])
        avoid_ways.append(plan_json_debug['avoid_way'] * 0.1)
      frame_num_0 = frame_nums[0]
      frame_nums = [frame_num - frame_num_0 for frame_num in frame_nums]
  fig = bkp.figure(x_axis_label='frame_num', y_axis_label='lat_offset',x_range = [frame_nums[0], frame_nums[-1]], width=800, height=200)
  data_fig.data.update({
    'lateral_offset_1':lateral_offsets,
    'lateral_offset_2':smooth_lateral_offsets,
    "avoid_ways":avoid_ways,
    'frame_num_y':frame_nums,
  })
  f1 = fig.line('frame_num_y', 'lateral_offset_1', source = data_fig, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lateral_offset')
  fig.line('frame_num_y', 'lateral_offset_2', source = data_fig, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'smooth_lateral_offset')
  fig.line('frame_num_y', 'avoid_ways', source = data_fig, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'avoid_way')

  hover1_1 = HoverTool(renderers=[f1], tooltips=[('frame_num', '@frame_num_y'), ('lateral_offset', '@lateral_offset_1'), ('smooth_lateral_offset', '@lateral_offset_2'), ('avoid_way', '@avoid_ways')], mode='vline')
  fig.add_tools(hover1_1)
  fig.legend.click_policy = 'hide'
  fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
  return fig
def load_lat_plan_figure(fig1):
  data_refline = ColumnDataSource(data = {'raw_refline_x':[],
                                          'raw_refline_y':[],})

  data_lat_motion_plan_input = ColumnDataSource(data = {'ref_x':[],
                                                        'ref_y':[],
                                                        'ref_xn':[],
                                                        'ref_yn':[],
                                                        'soft_upper_bound_x0_vec':[],
                                                        'soft_upper_bound_y0_vec':[],
                                                        'soft_lower_bound_x0_vec':[],
                                                        'soft_lower_bound_y0_vec':[],
                                                        'hard_upper_bound_x0_vec':[],
                                                        'hard_upper_bound_y0_vec':[],
                                                        'hard_lower_bound_x0_vec':[],
                                                        'hard_lower_bound_y0_vec':[],
                                                        'bound_t_vec':[],
                                                        'bound_s_vec':[],
                                                        'soft_upper_bound_vec':[],
                                                        'soft_lower_bound_vec':[],
                                                        'hard_upper_bound_vec':[],
                                                        'hard_lower_bound_vec':[],
                                                        'soft_upper_bound_id_vec':[],
                                                        'soft_lower_bound_id_vec':[],
                                                        'hard_upper_bound_id_vec':[],
                                                        'hard_lower_bound_id_vec':[],
                                                        'soft_upper_bound_type_vec':[],
                                                        'soft_lower_bound_type_vec':[],
                                                        'hard_upper_bound_type_vec':[],
                                                        'hard_lower_bound_type_vec':[],
                                                        })

  data_lat_motion_plan_output = ColumnDataSource(data = {'time_vec':[],
                                                         'ref_x_vec':[],
                                                         'x_vec':[],
                                                         'ref_y_vec':[],
                                                         'y_vec':[],
                                                         'xn_vec':[],
                                                         'yn_vec':[],
                                                         'ref_theta_deg_vec':[],
                                                         'theta_deg_vec':[],
                                                         'steer_deg_vec':[],
                                                         'steer_dot_deg_vec':[],
                                                         'acc_vec':[],
                                                         'jerk_vec':[],
                                                         'acc_upper_bound':[],
                                                         'acc_lower_bound':[],
                                                         'jerk_upper_bound':[],
                                                         'jerk_lower_bound':[],
                                                         'steer_deg_upper_bound': [],
                                                         'steer_deg_lower_bound': [],
                                                         'steer_dot_deg_upper_bound': [],
                                                         'steer_dot_deg_lower_bound': [],
                                                        })

  data_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                           'plan_traj_x':[],
                                           })

  data_planning_n = ColumnDataSource(data = {'plan_traj_xn':[],
                                           'plan_traj_yn':[],})

  data_ego = ColumnDataSource(data = {'ego_xn':[],
                                      'ego_yn':[],})
  data_car = ColumnDataSource(data = {'car_xn':[],
                                      'car_yn':[],
                                      'car_xn2':[],
                                      'car_yn2':[],})

  data_ego_pos_point = ColumnDataSource(data = {'ego_pos_point_y':[],
                                                'ego_pos_point_x':[],
                                                'ego_pos_point_theta':[]})
  data_init_pos_point = ColumnDataSource(data = {'init_pos_point_y':[],
                                                 'init_pos_point_x':[],
                                                 'init_pos_point_theta':[],
                                                 'init_state_x':[],
                                                 'init_state_y':[],
                                                 'init_state_theta':[],
                                                 'init_state_delta':[],
                                                 'init_state_s':[],
                                                 'init_state_v':[],
                                                 'init_state_a':[],
                                                 'replan_status':[]})

  data_control = ColumnDataSource(data = {'mpc_dx':[],
                                          'mpc_dy':[],})

  data_lane_0 = ColumnDataSource(data = {'line_0_y':[], 'line_0_x':[]})
  data_lane_1 = ColumnDataSource(data = {'line_1_y':[], 'line_1_x':[]})
  data_lane_2 = ColumnDataSource(data = {'line_2_y':[], 'line_2_x':[]})
  data_lane_3 = ColumnDataSource(data = {'line_3_y':[], 'line_3_x':[]})
  data_lane_4 = ColumnDataSource(data = {'line_4_y':[], 'line_4_x':[]})
  data_lane_5 = ColumnDataSource(data = {'line_5_y':[], 'line_5_x':[]})
  data_lane_6 = ColumnDataSource(data = {'line_6_y':[], 'line_6_x':[]})
  data_lane_7 = ColumnDataSource(data = {'line_7_y':[], 'line_7_x':[]})
  data_lane_8 = ColumnDataSource(data = {'line_8_y':[], 'line_8_x':[]})
  data_lane_9 = ColumnDataSource(data = {'line_9_y':[], 'line_9_x':[]})
  data_center_line_0 = ColumnDataSource(data = {'center_line_0_y':[], 'center_line_0_x':[]})
  data_center_line_1 = ColumnDataSource(data = {'center_line_1_y':[], 'center_line_1_x':[]})
  data_center_line_2 = ColumnDataSource(data = {'center_line_2_y':[], 'center_line_2_x':[]})
  data_center_line_3 = ColumnDataSource(data = {'center_line_3_y':[], 'center_line_3_x':[]})
  data_center_line_4 = ColumnDataSource(data = {'center_line_4_y':[], 'center_line_4_x':[]})
  lat_plan_data = {'data_lat_motion_plan_input':data_lat_motion_plan_input,
                   'data_lat_motion_plan_output':data_lat_motion_plan_output,
                   'data_refline':data_refline,
                   'data_planning':data_planning,
                   'data_planning_n': data_planning_n,
                   'data_control': data_control,
                   'data_ego': data_ego,
                   'data_car': data_car,
                   'data_ego_pos_point': data_ego_pos_point, \
                   'data_init_pos_point': data_init_pos_point, \
                   'data_lane_0':data_lane_0, \
                   'data_lane_1':data_lane_1, \
                   'data_lane_2':data_lane_2, \
                   'data_lane_3':data_lane_3, \
                   'data_lane_4':data_lane_4, \
                   'data_lane_5':data_lane_5, \
                   'data_lane_6':data_lane_6, \
                   'data_lane_7':data_lane_7, \
                   'data_lane_8':data_lane_8, \
                   'data_lane_9':data_lane_9, \
                   'data_center_line_0':data_center_line_0, \
                   'data_center_line_1':data_center_line_1, \
                   'data_center_line_2':data_center_line_2, \
                   'data_center_line_3':data_center_line_3, \
                   'data_center_line_4':data_center_line_4, \
  }


  # motion planning
  fig1.line('ref_y', 'ref_x', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path', visible=True)
  fig1.line('soft_upper_bound_y0_vec', 'soft_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = "darkorange", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft upper bound')
  fig1.line('soft_lower_bound_y0_vec', 'soft_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = "darkorange", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft lower bound')
  fig1.line('hard_upper_bound_y0_vec', 'hard_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = 'maroon', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard upper bound')
  fig1.line('hard_lower_bound_y0_vec', 'hard_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = 'maroon', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard lower bound')
  fig1.line('raw_refline_y', 'raw_refline_x', source = data_refline, line_width = 3, line_color = 'blue', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline', visible=False)
  # fig1.line('y_vec', 'x_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'red', line_dash = 'dashed', line_alpha = 0.4, legend_label = 'plan path')
  # fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan debug', visible=False)

  fig1.circle('soft_upper_bound_y0_vec','soft_upper_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "darkorange", line_alpha = 0.7, fill_color = 'gold',fill_alpha = 0.7, legend_label = 'soft upper bound')
  fig1.circle('soft_lower_bound_y0_vec','soft_lower_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "darkorange", line_alpha = 0.7, fill_color = 'gold',fill_alpha = 0.7, legend_label = 'soft lower bound')
  fig1.circle('hard_upper_bound_y0_vec','hard_upper_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "maroon", line_alpha = 0.5, fill_color = 'red',fill_alpha = 0.5, legend_label = 'hard upper bound')
  fig1.circle('hard_lower_bound_y0_vec','hard_lower_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "maroon", line_alpha = 0.5, fill_color = 'red',fill_alpha = 0.5, legend_label = 'hard lower bound')

  fig2 = bkp.figure(x_axis_label='time', y_axis_label='theta',x_range = [-0.1, 6.0], width=800, height=160)
  fig3 = bkp.figure(x_axis_label='time', y_axis_label='lat acc',x_range = fig2.x_range, width=800, height=160)
  fig4 = bkp.figure(x_axis_label='time', y_axis_label='lat jerk',x_range = fig2.x_range, width=800, height=160)
  fig5 = bkp.figure(x_axis_label='time', y_axis_label='steer',x_range = fig2.x_range, width=800, height=160)
  fig6 = bkp.figure(x_axis_label='time', y_axis_label='steer dot',x_range = fig2.x_range, width=800, height=160)

  fig7 = bkp.figure(x_axis_label='y', y_axis_label='x', width=800, height=800, match_aspect = True, aspect_scale=1)
  fig7.x_range.flipped = True

  fig8 = bkp.figure(x_axis_label='time', y_axis_label='x',x_range = fig2.x_range, width=800, height=160)
  fig9 = bkp.figure(x_axis_label='time', y_axis_label='y',x_range = fig2.x_range, width=800, height=160)

  fig7.line('line_0_y', 'line_0_x', source = data_lane_0, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_1_y', 'line_1_x', source = data_lane_1, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_2_y', 'line_2_x', source = data_lane_2, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_3_y', 'line_3_x', source = data_lane_3, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_4_y', 'line_4_x', source = data_lane_4, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_5_y', 'line_5_x', source = data_lane_5, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_6_y', 'line_6_x', source = data_lane_6, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_7_y', 'line_7_x', source = data_lane_7, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_8_y', 'line_8_x', source = data_lane_8, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_9_y', 'line_9_x', source = data_lane_9, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.patch('car_yn2', 'car_xn2', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
  fig7.circle('ego_pos_point_y', 'ego_pos_point_x', source = data_ego_pos_point, radius = 0.1, line_width = 2,  line_color = 'purple', line_alpha = 1, fill_alpha = 1, legend_label = 'ego_pos_point')
  fig7.circle('init_pos_point_y', 'init_pos_point_x', source = data_init_pos_point, radius = 0.1, line_width = 2,  line_color = 'black', line_alpha = 1, fill_color = "deepskyblue", fill_alpha = 1, legend_label = 'init_state')
  fig7.line('center_line_0_y', 'center_line_0_x', source = data_center_line_0, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig7.line('center_line_1_y', 'center_line_1_x', source = data_center_line_1, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig7.line('center_line_2_y', 'center_line_2_x', source = data_center_line_2, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig7.line('center_line_3_y', 'center_line_3_x', source = data_center_line_3, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig7.line('center_line_4_y', 'center_line_4_x', source = data_center_line_4, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig7.line('ego_yn', 'ego_xn', source = data_ego, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig7.line('ref_yn', 'ref_xn', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path')
  fig7.line('yn_vec', 'xn_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'red', line_dash = 'dashed', line_alpha = 0.4, legend_label = 'plan path')
  fig7.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan')
  fig7.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 5, line_color = 'green', line_dash = 'dashed', line_alpha = 0.8, legend_label = 'ctrl_traj')

  f2 = fig2.line('time_vec', 'ref_theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'ref_theta')
  fig2.line('time_vec', 'theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'theta')
  f3 = fig3.line('time_vec', 'acc_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat acc')
  fig3.line('time_vec', 'acc_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'lat acc corridor')
  fig3.line('time_vec', 'acc_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'lat acc corridor')
  fig3.triangle ('time_vec', 'acc_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat acc corridor')
  fig3.inverted_triangle ('time_vec', 'acc_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat acc corridor')
  f4 = fig4.line('time_vec', 'jerk_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat jerk')
  fig4.line('time_vec', 'jerk_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'lat jerk corridor')
  fig4.line('time_vec', 'jerk_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'lat jerk corridor')
  fig4.triangle ('time_vec', 'jerk_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat jerk corridor')
  fig4.inverted_triangle ('time_vec', 'jerk_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat jerk corridor')
  f5 = fig5.line('time_vec', 'steer_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer deg')
  fig5.line('time_vec', 'steer_deg_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'steer deg corridor')
  fig5.line('time_vec', 'steer_deg_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'steer deg corridor')
  fig5.triangle ('time_vec', 'steer_deg_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer deg corridor')
  fig5.inverted_triangle ('time_vec', 'steer_deg_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer deg corridor')
  f6 = fig6.line('time_vec', 'steer_dot_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer dot deg')
  fig6.line('time_vec', 'steer_dot_deg_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'steer dot deg corridor')
  fig6.line('time_vec', 'steer_dot_deg_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'steer dot deg corridor')
  fig6.triangle ('time_vec', 'steer_dot_deg_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer dot deg corridor')
  fig6.inverted_triangle ('time_vec', 'steer_dot_deg_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer dot deg corridor')

  f8 = fig8.line('time_vec', 'ref_x_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'ref_x')
  fig8.line('time_vec', 'x_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'x')
  f9 = fig9.line('time_vec', 'ref_y_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'ref_y')
  fig9.line('time_vec', 'y_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'y')

  hover1_1 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 4]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @soft_upper_bound_vec)'),
                                                                                      ('obstacle id', '@soft_upper_bound_id_vec'), ('type', '@soft_upper_bound_type_vec')])
  hover1_2 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 3]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @soft_lower_bound_vec)'),
                                                                                      ('obstacle id', '@soft_lower_bound_id_vec'), ('type', '@soft_lower_bound_type_vec')])
  hover1_3 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 2]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @hard_upper_bound_vec)'),
                                                                                      ('obstacle id', '@hard_upper_bound_id_vec'), ('type', '@hard_upper_bound_type_vec')])
  hover1_4 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 1]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @hard_lower_bound_vec)'),
                                                                                      ('obstacle id', '@hard_lower_bound_id_vec'), ('type', '@hard_lower_bound_type_vec')])
  hover2 = HoverTool(renderers=[f2], tooltips=[('time', '@time_vec'), ('ref_theta', '@ref_theta_deg_vec'), ('theta', '@theta_deg_vec')], mode='vline')
  hover3 = HoverTool(renderers=[f3], tooltips=[('time', '@time_vec'), ('acc', '@acc_vec'), ('|acc bound|', '@acc_upper_bound')], mode='vline')
  hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time_vec'), ('jerk', '@jerk_vec'), ('|jerk bound|', '@jerk_upper_bound')], mode='vline')
  hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('steer', '@steer_deg_vec'), ('|steer deg bound|', '@steer_deg_upper_bound')], mode='vline')
  hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time_vec'), ('steer dot', '@steer_dot_deg_vec'), ('|steer dot deg bound|', '@steer_dot_deg_upper_bound')], mode='vline')
  hover8 = HoverTool(renderers=[f8], tooltips=[('time', '@time_vec'), ('ref_x', '@ref_x_vec'), ('x', '@x_vec')], mode='vline')
  hover9 = HoverTool(renderers=[f9], tooltips=[('time', '@time_vec'), ('ref_y', '@ref_y_vec'), ('y', '@y_vec')], mode='vline')

  fig1.add_tools(hover1_1)
  fig1.add_tools(hover1_2)
  fig1.add_tools(hover1_3)
  fig1.add_tools(hover1_4)
  fig2.add_tools(hover2)
  fig3.add_tools(hover3)
  fig4.add_tools(hover4)
  fig5.add_tools(hover5)
  fig6.add_tools(hover6)
  fig8.add_tools(hover8)
  fig9.add_tools(hover9)

  fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
  fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
  fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
  fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
  fig6.toolbar.active_scroll = fig6.select_one(WheelZoomTool)
  fig7.toolbar.active_scroll = fig7.select_one(WheelZoomTool)
  fig8.toolbar.active_scroll = fig8.select_one(WheelZoomTool)
  fig9.toolbar.active_scroll = fig9.select_one(WheelZoomTool)

  fig2.legend.click_policy = 'hide'
  fig3.legend.click_policy = 'hide'
  fig4.legend.click_policy = 'hide'
  fig5.legend.click_policy = 'hide'
  fig6.legend.click_policy = 'hide'
  fig7.legend.click_policy = 'hide'
  fig8.legend.click_policy = 'hide'
  fig9.legend.click_policy = 'hide'

  return fig1, fig2, fig3, fig4, fig5, fig6, fig7, fig8, fig9, lat_plan_data