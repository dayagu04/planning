from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *
import lib.load_global_var as global_var

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

def update_spatio_temporal_union_plan_data(bag_loader, bag_time, local_view_data, spatio_temporal_union_plan_data):
  g_is_display_enu = global_var.get_value('g_is_display_enu')
  is_match_planning = global_var.get_value('is_match_planning')
  is_bag_main = global_var.get_value('is_bag_main')
  is_new_loc = global_var.get_value('is_new_loc')
  is_enu_to_car = global_var.get_value('is_enu_to_car')
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
    load_time_cost(planning_debug, planning_json)
    print(debug2)
    ego_xn, ego_yn = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.loc_msg['data'])):
      pos_xn_i = bag_loader.loc_msg['data'][i].position.position_boot.x
      pos_yn_i = bag_loader.loc_msg['data'][i].position.position_boot.y

      ego_xn.append(pos_xn_i)
      ego_yn.append(pos_yn_i)

    spatio_temporal_union_plan_data['data_ego'].data.update({
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

      spatio_temporal_union_plan_data['data_car'].data.update({
        'car_xn': car_xn,
        'car_yn': car_yn,
      })
    else:
      spatio_temporal_union_plan_data['data_car'].data.update({
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

      spatio_temporal_union_plan_data['data_car'].data.update({
        'car_xn2': car_xn,
        'car_yn2': car_yn,
      })
      spatio_temporal_union_plan_data['data_ego_pos_point'].data.update({
        'ego_pos_point_x': [cur_pos_xn],
        'ego_pos_point_y': [cur_pos_yn],
        'ego_pos_point_theta': [cur_yaw],
      })
    else:
      spatio_temporal_union_plan_data['data_car'].data.update({
        'car_xn2': car_xb,
        'car_yn2': car_yb,
      })
      spatio_temporal_union_plan_data['data_ego_pos_point'].data.update({
        'ego_pos_point_x': [0],
        'ego_pos_point_y': [0],
        'ego_pos_point_theta': [0],
      })

    # try:
    #   json_pos_x = planning_json['ego_pos_x']
    #   json_pos_y = planning_json['ego_pos_y']
    #   json_yaw = planning_json['ego_pos_yaw']
    #   coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
    # except:
    coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)
    print("load cur_pos_xn:", cur_pos_xn)
    print("load cur_pos_yn:", cur_pos_yn)
    print("load cur_yaw:", cur_yaw)

  if bag_loader.plan_debug_msg['enable'] == True:
    spatio_temporal_union_plan = plan_debug_msg.spatio_temporal_union_plan
    origin_refline_points = spatio_temporal_union_plan.origin_refline_points
    trajectory_points = spatio_temporal_union_plan.trajectory_points
    # print("spatio_temporal_union_plan:", spatio_temporal_union_plan)
    refline_x, refline_y = [], []
    traj_x, traj_y = [], []
    s_vec, l_vec, t_vec, v_vec = [], [], [], []
    for trajectory_point in trajectory_points:
      s_vec.append(trajectory_point.s)
      l_vec.append(trajectory_point.l)
      t_vec.append(trajectory_point.t)
      v_vec.append(trajectory_point.v)
    if g_is_display_enu:
      for trajectory_point in trajectory_points:
        traj_x.append(trajectory_point.x)
        traj_y.append(trajectory_point.y)
      for origin_refline_point in origin_refline_points:
        refline_x.append(origin_refline_point.x)
        refline_y.append(origin_refline_point.y)
    else:
      for trajectory_point in trajectory_points:
        traj_x_local, traj_y_local = coord_tf.global_to_local([trajectory_point.x], [trajectory_point.y])
        traj_x.append(traj_x_local[0])
        traj_y.append(traj_y_local[0])
      for origin_refline_point in origin_refline_points:
        refline_x_local, refline_y_local = coord_tf.global_to_local([origin_refline_point.x], [origin_refline_point.y])
        refline_x.append(refline_x_local)
        refline_y.append(refline_y_local)
    print("traj_x:", traj_x)
    print("traj_y:", traj_y)
    print("refline_x:", refline_x)
    print("refline_y:", refline_y)
    spatio_temporal_union_plan_data['data_spatio_temporal'].data.update({
      'refline_x': refline_x,
      'refline_y': refline_y,
    })

    spatio_temporal_union_plan_data['data_spatio_temporal_trajs'].data.update({
      'traj_x': traj_x,
      'traj_y': traj_y,
    })

    spatio_temporal_union_plan_data['data_spatio_temporal_slt_data'].data.update({
      's_vec': s_vec,
      'l_vec': l_vec,
      't_vec': t_vec,
      'v_vec': v_vec,
    })

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

    spatio_temporal_union_plan_data['data_init_pos_point'].data.update({
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
    else:
      ref_x, ref_y = coord_tf.global_to_local(lat_motion_plan_input.ref_x_vec, lat_motion_plan_input.ref_y_vec)

    spatio_temporal_union_plan_data['data_lat_motion_plan_input'].data.update({
      'ref_x': ref_x,
      'ref_y': ref_y,
      'ref_xn': lat_motion_plan_input.ref_x_vec,
      'ref_yn': lat_motion_plan_input.ref_y_vec,
    })

    if g_is_display_enu:
      raw_refline_x, raw_refline_y = planning_json['raw_refline_x_vec'], \
        planning_json['raw_refline_y_vec']
    else:
      raw_refline_x, raw_refline_y = coord_tf.global_to_local(planning_json['raw_refline_x_vec'], \
        planning_json['raw_refline_y_vec'])

    spatio_temporal_union_plan_data['data_refline'].data.update({
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

    spatio_temporal_union_plan_data['data_lat_motion_plan_output'].data.update({
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
    print("is_static_avoid_scene = ", planning_json['is_static_avoid_scene'])
    # print("min cost = ", lat_motion_plan_output.solver_info.iter_info[max(lat_motion_plan_output.solver_info.iter_count - 1, 0)].cost)

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

      spatio_temporal_union_plan_data['data_planning_n'].data.update({
        'plan_traj_xn':planning_json['traj_x_vec'],
        'plan_traj_yn':planning_json['traj_y_vec'],
      })

    spatio_temporal_union_plan_data['data_planning'].data.update({
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

  # load control
  if bag_loader.ctrl_msg['enable'] == True:
    mpc_dx, mpc_dy, mpc_dtheta = generate_control(ctrl_msg, loc_msg, not g_is_display_enu)
    spatio_temporal_union_plan_data['data_control'].data.update({
        'mpc_dx' : mpc_dx,
        'mpc_dy' : mpc_dy,
    })


def load_spatio_temporal_union_dp_result(bag_loader):
  data_fig = ColumnDataSource(data ={
    'st_dp_is_sucess': [],
    'frame_num_y': [],
  })

  if bag_loader.plan_debug_msg['enable'] == True:
    st_dp_is_sucess_vec = []
    frame_nums = []

    for i, plan_debug_msg in enumerate(bag_loader.plan_debug_msg['data']):
      plan_debug_msg = bag_loader.plan_debug_msg['data'][i]
      frame_nums.append(plan_debug_msg.frame_info.frame_num)
      st_dp_is_sucess_vec.append(plan_debug_msg.spatio_temporal_union_plan.st_dp_is_sucess)
    frame_num_0 = frame_nums[0]
    frame_nums = [frame_num - frame_num_0 for frame_num in frame_nums]
  fig = bkp.figure(x_axis_label='frame_num', y_axis_label='st_dp_is_sucess',x_range = [frame_nums[0], frame_nums[-1]], width=800, height=200)
  data_fig.data.update({
    'st_dp_is_sucess':st_dp_is_sucess_vec,
    'frame_num_y':frame_nums,
  })
  f1 = fig.line('frame_num_y', 'st_dp_is_sucess', source = data_fig, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'st_dp_is_sucess')

  hover1_1 = HoverTool(renderers=[f1], tooltips=[('frame_num', '@frame_num_y'), ('st_dp_is_sucess', '@st_dp_is_sucess')], mode='vline')
  fig.add_tools(hover1_1)
  fig.legend.click_policy = 'hide'
  fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
  return fig

def load_spatio_temporal_union_cost_time(bag_loader):
  data_fig = ColumnDataSource(data ={
    'cost_time': [],
    'frame_num_y': [],
  })

  average_cost_time = 0.0
  iter_count = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    cost_times = []
    frame_nums = []

    for i, plan_debug_msg in enumerate(bag_loader.plan_debug_msg['data']):
      plan_debug_msg = bag_loader.plan_debug_msg['data'][i]
      frame_nums.append(plan_debug_msg.frame_info.frame_num)
      try:
        cost_times.append(plan_debug_msg.spatio_temporal_union_plan.cost_time)
        if plan_debug_msg.spatio_temporal_union_plan.st_dp_is_sucess:
          average_cost_time += plan_debug_msg.spatio_temporal_union_plan.cost_time
          iter_count += 1
      except:
        print("no spatio_temporal_union_plan!!")
    frame_num_0 = frame_nums[0]
    frame_nums = [frame_num - frame_num_0 for frame_num in frame_nums]
  fig = bkp.figure(x_axis_label='frame_num', y_axis_label='cost_time',x_range = [frame_nums[0], frame_nums[-1]], width=800, height=200)
  data_fig.data.update({
    'cost_time':cost_times,
    'frame_num_y':frame_nums,
  })
  f1 = fig.line('frame_num_y', 'cost_time', source = data_fig, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'cost_time')

  hover1_1 = HoverTool(renderers=[f1], tooltips=[('frame_num', '@frame_num_y'), ('cost_time', '@cost_times')], mode='vline')
  fig.add_tools(hover1_1)
  fig.legend.click_policy = 'hide'
  fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
  if iter_count != 0:
    average_cost_time = average_cost_time / iter_count
  return fig, average_cost_time

def load_spatio_temporal_union_st_data(bag_loader, spatio_temporal_union_plan_data, local_view_data):
  # data_lon_motion_plan = ColumnDataSource(data = {'t_long_vec': [],
  #                                                 's_plan_vec': [],
  #                                                                 })
  # t_long_vec = []
  # for item in (local_view_data['data_msg']['plan_msg'].trajectory.trajectory_points):
  #   t_long_vec.append(item.t)
  # s_plan_vec =  []
  # for item in (local_view_data['data_msg']['plan_msg'].trajectory.trajectory_points):
  #   s_plan_vec.append(item.distance)
  # data_lon_motion_plan[t_long_vec].data.update({
  #   't_long_vec': t_long_vec,
  # })
  # data_lon_motion_plan[s_plan_vec].data.update({
  #   's_plan_vec': s_plan_vec,
  # })

  hover = HoverTool(tooltips = [
     ('index','$index'),
     ('id_low','@obs_low_id'),
     ('id_high','@obs_high_id'),
     ('low_type','@obs_low_type'),
     ('high_type','@obs_high_type')
  ])
  # fig S-T
  fig = bkp.figure(x_axis_label='t', y_axis_label='s', x_range = [-0.1, 7.0], width=600, height=400, tools=[hover,'pan,wheel_zoom,box_zoom,reset'], match_aspect = True, aspect_scale=1)
  fig.line('t_vec', 's_vec', source = spatio_temporal_union_plan_data["data_spatio_temporal_slt_data"], line_width = 2.5, line_color = 'red', line_dash = 'dashed', legend_label = 's_ref')
  fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
  fig.legend.click_policy = 'hide'
  return fig

def load_spatio_temporal_union_sl_data(bag_loader, spatio_temporal_union_plan_data, local_view_data):

  hover = HoverTool(tooltips = [
     ('index','$index'),
     ('id_low','@obs_low_id'),
     ('id_high','@obs_high_id'),
     ('low_type','@obs_low_type'),
     ('high_type','@obs_high_type')
  ])
  # fig S-T
  fig = bkp.figure(x_axis_label='s', y_axis_label='l', x_range = [-10, 200.0], width=600, height=400, tools=[hover,'pan,wheel_zoom,box_zoom,reset'], match_aspect = True, aspect_scale=1)
  fig.line('s_vec', 'l_vec', source = spatio_temporal_union_plan_data["data_spatio_temporal_slt_data"], line_width = 2.5, line_color = 'blue', line_dash = 'dashed', legend_label = 's_l')
  fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
  fig.legend.click_policy = 'hide'
  return fig

def load_spatio_temporal_union_vt_data(bag_loader, spatio_temporal_union_plan_data, local_view_data):

  hover = HoverTool(tooltips = [
     ('index','$index'),
     ('id_low','@obs_low_id'),
     ('id_high','@obs_high_id'),
     ('low_type','@obs_low_type'),
     ('high_type','@obs_high_type')
  ])
  # fig V-T
  fig = bkp.figure(x_axis_label='t', y_axis_label='v', x_range = [-0.1, 7.0], width=600, height=400, tools=[hover,'pan,wheel_zoom,box_zoom,reset'], match_aspect = True, aspect_scale=1)
  fig.line('t_vec', 'v_vec', source = spatio_temporal_union_plan_data["data_spatio_temporal_slt_data"], line_width = 2.5, line_color = 'green', line_dash = 'dashed', legend_label = 'v_ref')
  fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
  fig.legend.click_policy = 'hide'
  return fig

def load_spatio_temporal_union_plan_figure(fig1):
  data_refline = ColumnDataSource(data = {'raw_refline_x':[],
                                          'raw_refline_y':[],})

  data_lat_motion_plan_input = ColumnDataSource(data = {'ref_x':[],
                                                        'ref_y':[],
                                                        'ref_xn':[],
                                                        'ref_yn':[],
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

  data_spatio_temporal = ColumnDataSource(data = {'refline_x':[],
                                         'refline_y':[],})

  data_spatio_temporal_back = ColumnDataSource(data = {'refline_x':[],
                                         'refline_y':[],})

  data_spatio_temporal_trajs = ColumnDataSource(data = {'traj_x':[],
                                         'traj_y':[],})

  data_spatio_temporal_trajs_back = ColumnDataSource(data = {'traj_x':[],
                                         'traj_y':[],
                                         'time_vec':[],})
  data_spatio_temporal_slt_data = ColumnDataSource(data = {'s_vec':[],
                                        'l_vec':[],
                                        't_vec':[],})

  spatio_temporal_union_plan_data = {'data_lat_motion_plan_input':data_lat_motion_plan_input,
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
                   'data_spatio_temporal':data_spatio_temporal, \
                   'data_spatio_temporal_trajs':data_spatio_temporal_trajs, \
                   'data_spatio_temporal_back':data_spatio_temporal_back, \
                   'data_spatio_temporal_trajs_back':data_spatio_temporal_trajs_back, \
                   'data_spatio_temporal_slt_data':data_spatio_temporal_slt_data, \
  }


  # motion planning
  fig1.line('refline_y', 'refline_x', source = data_spatio_temporal, line_width = 5, line_color = 'green', line_dash = 'solid', line_alpha = 0.35, legend_label = 'origin ref', visible=True)
  fig1.line('refline_y', 'refline_x', source = data_spatio_temporal_back, line_width = 5, line_color = 'green', line_dash = 'solid', line_alpha = 0.35, legend_label = 'origin ref back', visible=True)
  fig1.line('traj_y', 'traj_x', source = data_spatio_temporal_trajs, line_width = 5, line_color = 'black', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path', visible=True)
  fig1.line('traj_y', 'traj_x', source = data_spatio_temporal_trajs_back, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.5, legend_label = 'ref path back', visible=True)
  fig1.line('raw_refline_y', 'raw_refline_x', source = data_refline, line_width = 3, line_color = 'blue', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline', visible=False)
  # fig1.line('y_vec', 'x_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'red', line_dash = 'dashed', line_alpha = 0.4, legend_label = 'plan path')
  # fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan debug', visible=False)
  fig_spatio = fig1.circle('traj_y', 'traj_x', source = data_spatio_temporal_trajs, radius = 0.3, line_width = 1,  line_color = 'black', line_alpha = 1, fill_alpha = 0, legend_label = 'spatio_temporal_plan_point')
  fig_spatio_back = fig1.circle('traj_y', 'traj_x', source = data_spatio_temporal_trajs_back, radius = 0.3, line_width = 1,  line_color = 'red', line_alpha = 1, fill_alpha = 0, legend_label = 'spatio_temporal_plan_back')

  hover1_1 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 4]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @soft_upper_bound_vec)'),
                                                                                      ('obstacle id', '@soft_upper_bound_id_vec'), ('type', '@soft_upper_bound_type_vec')])
  hover1_2 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 3]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @soft_lower_bound_vec)'),
                                                                                      ('obstacle id', '@soft_lower_bound_id_vec'), ('type', '@soft_lower_bound_type_vec')])
  hover1_3 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 2]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @hard_upper_bound_vec)'),
                                                                                      ('obstacle id', '@hard_upper_bound_id_vec'), ('type', '@hard_upper_bound_type_vec')])
  hover1_4 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 1]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @hard_lower_bound_vec)'),
                                                                                      ('obstacle id', '@hard_lower_bound_id_vec'), ('type', '@hard_lower_bound_type_vec')])
  hover1_5 = HoverTool(renderers=[fig_spatio], tooltips=[('index', '$index')])
  hover1_6 = HoverTool(renderers=[fig_spatio_back], tooltips=[('index', '$index')])


  fig1.add_tools(hover1_1)
  fig1.add_tools(hover1_2)
  fig1.add_tools(hover1_3)
  fig1.add_tools(hover1_4)
  fig1.add_tools(hover1_5)
  fig1.add_tools(hover1_6)

  return fig1, spatio_temporal_union_plan_data