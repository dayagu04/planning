from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *
import lib.load_global_var as global_var

import numpy as np
import re
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

def load_ara_obstacles_params(local_view_data):
  obs_info_all = {
    'obstacles_x_rel': [],
    'obstacles_y_rel': [],
    'obstacles_x': [],
    'obstacles_y': [],
    'pose_x': [],
    'pose_y': [],
    'obstacles_label': [],
  }

  plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
  for i in range(len(plan_debug_msg.hybrid_ara_info.ara_obstacles)):
    # print("plan_debug_msg.ara_obstacles[i].x_relative_center", plan_debug_msg.ara_obstacles[i].x_relative_center)
    # print("plan_debug_msg.ara_obstacles[i].y_relative_center",  plan_debug_msg.ara_obstacles[i].y_relative_center)
    # print("plan_debug_msg.ara_obstacles[i].x_center",  plan_debug_msg.ara_obstacles[i].x_center)
    # print("plan_debug_msg.ara_obstacles[i].y_center",  plan_debug_msg.ara_obstacles[i].y_center)
    # print('---------------------------------------------------------------------------------')
    long_pos_rel = plan_debug_msg.hybrid_ara_info.ara_obstacles[i].x_relative_center
    lat_pos_rel = plan_debug_msg.hybrid_ara_info.ara_obstacles[i].y_relative_center
    theta = plan_debug_msg.hybrid_ara_info.ara_obstacles[i].relative_heading_angle
    if theta == 255:
      theta = 0
    half_width = plan_debug_msg.hybrid_ara_info.ara_obstacles[i].width / 2
    half_length = plan_debug_msg.hybrid_ara_info.ara_obstacles[i].length / 2
    # if half_width == 0 or half_length == 0:
    #   continue
    cos_heading = math.cos(theta)
    sin_heading = math.sin(theta)
    dx1 = cos_heading * half_length
    dy1 = sin_heading * half_length
    dx2 = sin_heading * half_width
    dy2 = -cos_heading * half_width

    obs_x_rel = [long_pos_rel + dx1 + dx2,
              long_pos_rel + dx1 - dx2,
              long_pos_rel- dx1 - dx2,
              long_pos_rel - dx1 + dx2,
              long_pos_rel + dx1 + dx2]
    obs_y_rel = [lat_pos_rel + dy1 + dy2,
              lat_pos_rel + dy1 - dy2,
              lat_pos_rel - dy1 - dy2,
              lat_pos_rel - dy1 + dy2,
              lat_pos_rel + dy1 + dy2]

    # 绝对坐标系下的数据
    long_pos = plan_debug_msg.hybrid_ara_info.ara_obstacles[i].x_center
    lat_pos = plan_debug_msg.hybrid_ara_info.ara_obstacles[i].y_center
    theta = plan_debug_msg.hybrid_ara_info.ara_obstacles[i].heading_angle
    cos_heading = math.cos(theta)
    sin_heading = math.sin(theta)
    dx1 = cos_heading * half_length
    dy1 = sin_heading * half_length
    dx2 = sin_heading * half_width
    dy2 = -cos_heading * half_width
    obs_x = [long_pos + dx1 + dx2,
              long_pos + dx1 - dx2,
              long_pos - dx1 - dx2,
              long_pos - dx1 + dx2,
              long_pos + dx1 + dx2]
    obs_y = [lat_pos + dy1 + dy2,
              lat_pos + dy1 - dy2,
              lat_pos - dy1 - dy2,
              lat_pos - dy1 + dy2,
              lat_pos + dy1 + dy2]

    obs_info_all['obstacles_x_rel'].append(obs_x_rel)
    obs_info_all['obstacles_y_rel'].append(obs_y_rel)
    obs_info_all['obstacles_x'].append(obs_x)
    obs_info_all['obstacles_y'].append(obs_y)
    obs_info_all['pose_x'].append([long_pos])
    obs_info_all['pose_y'].append([lat_pos])
    obs_info_all['obstacles_label'].append(str(plan_debug_msg.hybrid_ara_info.ara_obstacles[i].id))

  return obs_info_all

def update_hybrid_ara_path_data(fig2, bag_loader, bag_time, local_view_data, hybrid_ara_path_data, expand_step, expand_open_list_node_idx, g_is_display_enu = False):
  # get param
  g_is_display_enu = global_var.get_value('g_is_display_enu')
  is_match_planning = global_var.get_value('is_match_planning')
  is_bag_main = global_var.get_value('is_bag_main')
  is_new_loc = global_var.get_value('is_new_loc')
  is_enu_to_car = global_var.get_value('is_enu_to_car')
  is_vis_map = global_var.get_value('is_vis_map')
  is_vis_sdmap = global_var.get_value('is_vis_sdmap')
  # get msg
  road_msg = find_nearest(bag_loader.road_msg, bag_time)
  vs_msg = find_nearest(bag_loader.vs_msg, bag_time)
  loc_msg = local_view_data['data_msg']['loc_msg']
  plan_msg = local_view_data['data_msg']['plan_msg']
  plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
  plan_debug_json_msg = local_view_data['data_msg']['plan_debug_json_msg']
  ctrl_msg = local_view_data['data_msg']['ctrl_msg']
  print("expand_num_vec: ", plan_debug_json_msg['expand_num_vec'])

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

  expand_nodes_index_number = re.findall(r'\d+', expand_open_list_node_idx)
  expand_nodes_index = [int(expand_node_index) for expand_node_index in expand_nodes_index_number]
  print("expand_nodes_index: ", expand_nodes_index)
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

    hybrid_ara_path_data['data_ego'].data.update({
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

      hybrid_ara_path_data['data_car'].data.update({
        'car_xn': car_xn,
        'car_yn': car_yn,
      })
    else:
      hybrid_ara_path_data['data_car'].data.update({
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

      hybrid_ara_path_data['data_car'].data.update({
        'car_xn2': car_xn,
        'car_yn2': car_yn,
      })
      hybrid_ara_path_data['data_ego_pos_point'].data.update({
        'ego_pos_point_x': [cur_pos_xn],
        'ego_pos_point_y': [cur_pos_yn],
        'ego_pos_point_theta': [cur_yaw],
      })
    else:
      hybrid_ara_path_data['data_car'].data.update({
        'car_xn2': car_xb,
        'car_yn2': car_yb,
      })
      hybrid_ara_path_data['data_ego_pos_point'].data.update({
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
    # hybrid_ara_path
    hybrid_ara_path = plan_debug_msg.hybrid_ara_info.hybrid_ara_path
    if g_is_display_enu:
      path_x_vec, path_y_vec = hybrid_ara_path.x, hybrid_ara_path.y
      path_xn_vec, path_yn_vec = coord_tf.global_to_local(hybrid_ara_path.x, hybrid_ara_path.y)
    else:
      path_x_vec, path_y_vec = coord_tf.global_to_local(hybrid_ara_path.x, hybrid_ara_path.y)
      path_xn_vec, path_yn_vec = hybrid_ara_path.x, hybrid_ara_path.y

    hybrid_ara_path_data['data_arastar'].data.update({
      'x_vec': path_x_vec,
      'y_vec': path_y_vec,
      'phi_vec': hybrid_ara_path.phi,
      'xn_vec': path_xn_vec,
      'yn_vec': path_yn_vec,
    })

    # hybrid_ara_path_cost
    hybrid_ara_path_cost = plan_debug_msg.hybrid_ara_info.hybrid_ara_path_cost
    if g_is_display_enu:
      path_x_vec, path_y_vec = hybrid_ara_path_cost.x, hybrid_ara_path_cost.y
      path_xn_vec, path_yn_vec = coord_tf.global_to_local(hybrid_ara_path_cost.x, hybrid_ara_path_cost.y)
      agent_cost_vec, boundary_cost_vec, center_cost_vec, motion_cost_vec = hybrid_ara_path_cost.agent_cost, hybrid_ara_path_cost.boundary_cost, hybrid_ara_path_cost.center_cost, hybrid_ara_path_cost.motion_cost
    else:
      path_x_vec, path_y_vec = coord_tf.global_to_local(hybrid_ara_path_cost.x, hybrid_ara_path_cost.y)
      path_xn_vec, path_yn_vec = hybrid_ara_path_cost.x, hybrid_ara_path_cost.y
      agent_cost_vec, boundary_cost_vec, center_cost_vec, motion_cost_vec = hybrid_ara_path_cost.agent_cost, hybrid_ara_path_cost.boundary_cost, hybrid_ara_path_cost.center_cost, hybrid_ara_path_cost.motion_cost

    hybrid_ara_path_data['data_arastar_cost'].data.update({
      'x_vec': path_x_vec,
      'y_vec': path_y_vec,
      'phi_vec': hybrid_ara_path_cost.phi,
      'xn_vec': path_xn_vec,
      'yn_vec': path_yn_vec,
      'agent_cost_vec': agent_cost_vec,
      'boundary_cost_vec': boundary_cost_vec,
      'center_cost_vec': center_cost_vec,
      'motion_cost_vec': motion_cost_vec,
    })

    # hybrid_ara_expand
    hybrid_ara_expand = plan_debug_msg.hybrid_ara_info.hybrid_ara_expand
    expand_cur_node_x_vec = []
    expand_cur_node_y_vec = []
    expand_cur_node_phi_vec = []
    expand_cur_node_agent_cost_vec = []
    expand_cur_node_boundary_cost_vec = []
    expand_cur_node_center_cost_vec = []
    expand_cur_node_motion_cost_vec = []
    expand_cur_node_heuristic_cost_vec = []
    expand_cur_node_total_cost_vec = []

    expand_openlist_vec = []
    expand_openlist_node_x_vec = []
    expand_openlist_node_y_vec = []
    expand_openlist_node_phi_vec = []
    expand_openlist_agent_cost_vec = []
    expand_openlist_boundary_cost_vec = []
    expand_openlist_center_cost_vec = []
    expand_openlist_motion_cost_vec = []
    expand_openlist_heuristic_cost_vec = []
    expand_openlist_total_cost_vec = []

    select_openlist_node_x_vec = []
    select_openlist_node_y_vec = []
    select_openlist_node_phi_vec = []
    select_openlist_agent_cost_vec = []
    select_openlist_boundary_cost_vec = []
    select_openlist_center_cost_vec = []
    select_openlist_motion_cost_vec = []
    select_openlist_heuristic_cost_vec = []
    select_openlist_total_cost_vec = []

    if expand_step > 0 and len(hybrid_ara_expand.hybrid_ara_expand) > 1:
      for i in range(min(expand_step, len(hybrid_ara_expand.hybrid_ara_expand))):
        expand_cur_node_x_vec.append(hybrid_ara_expand.hybrid_ara_expand[i].current_node.x)
        expand_cur_node_y_vec.append(hybrid_ara_expand.hybrid_ara_expand[i].current_node.y)
        expand_cur_node_phi_vec.append(hybrid_ara_expand.hybrid_ara_expand[i].current_node.phi)
        expand_cur_node_agent_cost_vec.append(hybrid_ara_expand.hybrid_ara_expand[i].current_node.agent_cost)
        expand_cur_node_boundary_cost_vec.append(hybrid_ara_expand.hybrid_ara_expand[i].current_node.boundary_cost)
        expand_cur_node_center_cost_vec.append(hybrid_ara_expand.hybrid_ara_expand[i].current_node.center_cost)
        expand_cur_node_motion_cost_vec.append(hybrid_ara_expand.hybrid_ara_expand[i].current_node.motion_cost)
        expand_cur_node_heuristic_cost_vec.append(hybrid_ara_expand.hybrid_ara_expand[i].current_node.heuristic_cost)
        expand_cur_node_total_cost_vec.append(hybrid_ara_expand.hybrid_ara_expand[i].current_node.total_cost)
        expand_openlist_vec.append(hybrid_ara_expand.hybrid_ara_expand[i].open_list)

      # expand_line_x_vec = []
      # expand_line_y_vec = []
      # for i in range(len(expand_openlist_vec)):
      expand_index = max(0, len(expand_openlist_vec) - 1)
      for j in range(len(expand_openlist_vec[expand_index])):
        expand_openlist_node_x_vec.append(expand_openlist_vec[expand_index][j].x)
        expand_openlist_node_y_vec.append(expand_openlist_vec[expand_index][j].y)
        expand_openlist_node_phi_vec.append(expand_openlist_vec[expand_index][j].phi)
        expand_openlist_agent_cost_vec.append(expand_openlist_vec[expand_index][j].agent_cost)
        expand_openlist_boundary_cost_vec.append(expand_openlist_vec[expand_index][j].boundary_cost)
        expand_openlist_center_cost_vec.append(expand_openlist_vec[expand_index][j].center_cost)
        expand_openlist_motion_cost_vec.append(expand_openlist_vec[expand_index][j].motion_cost)
        expand_openlist_heuristic_cost_vec.append(expand_openlist_vec[expand_index][j].heuristic_cost)
        expand_openlist_total_cost_vec.append(expand_openlist_vec[expand_index][j].total_cost)

        if j in expand_nodes_index:
          select_openlist_node_x_vec.append(expand_openlist_vec[expand_index][j].x)
          select_openlist_node_y_vec.append(expand_openlist_vec[expand_index][j].y)
          select_openlist_node_phi_vec.append(expand_openlist_vec[expand_index][j].phi)
          select_openlist_agent_cost_vec.append(expand_openlist_vec[expand_index][j].agent_cost)
          select_openlist_boundary_cost_vec.append(expand_openlist_vec[expand_index][j].boundary_cost)
          select_openlist_center_cost_vec.append(expand_openlist_vec[expand_index][j].center_cost)
          select_openlist_motion_cost_vec.append(expand_openlist_vec[expand_index][j].motion_cost)
          select_openlist_heuristic_cost_vec.append(expand_openlist_vec[expand_index][j].heuristic_cost)
          select_openlist_total_cost_vec.append(expand_openlist_vec[expand_index][j].total_cost)
        # start_node_x, start_node_y = expand_cur_node_x_vec[i], expand_cur_node_y_vec[i]
        # end_node_x, end_node_y = expand_openlist_vec[i][j].x, expand_openlist_vec[i][j].y
        # if not g_is_display_enu:
        #   start_node_x, start_node_y = coord_tf.global_to_local([start_node_x], [start_node_y])
        #   end_node_x, end_node_y = coord_tf.global_to_local([end_node_x], [end_node_y])
        # expand_line_x_vec.append([start_node_x[0], end_node_x[0]])
        # expand_line_y_vec.append([start_node_y[0], end_node_y[0]])

    if not g_is_display_enu:
      expand_cur_node_x_vec, expand_cur_node_y_vec = coord_tf.global_to_local(expand_cur_node_x_vec, expand_cur_node_y_vec)
      expand_openlist_node_x_vec, expand_openlist_node_y_vec = coord_tf.global_to_local(expand_openlist_node_x_vec, expand_openlist_node_y_vec)
      cur_yaw = loc_msg.orientation.euler_boot.yaw
      cur_node_phi_local = []
      for i in range(len(expand_cur_node_phi_vec)):
        cur_node_phi_local.append(expand_cur_node_phi_vec[i] - cur_yaw)
      expand_cur_node_phi_vec = cur_node_phi_local
      openlist_node_phi_local = []
      for i in range(len(expand_openlist_node_phi_vec)):
        openlist_node_phi_local.append(expand_openlist_node_phi_vec[i] - cur_yaw)
      expand_openlist_node_phi_vec = openlist_node_phi_local

    car_y_ratio = 0.001
    cur_node_car_x_vec = []
    cur_node_car_y_vec = []
    for i in range(len(expand_cur_node_x_vec)):
      cur_node_car_x = []
      cur_node_car_y = []
      for j in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[j], car_yb[j] * car_y_ratio, expand_cur_node_x_vec[i], expand_cur_node_y_vec[i], expand_cur_node_phi_vec[i])
        cur_node_car_x.append(tmp_x)
        cur_node_car_y.append(tmp_y)
      cur_node_car_x_vec.append(cur_node_car_x)
      cur_node_car_y_vec.append(cur_node_car_y)

    openlist_node_car_x_vec = []
    openlist_node_car_y_vec = []
    for i in range(len(expand_openlist_node_x_vec)):
      openlist_node_car_x = []
      openlist_node_car_y = []
      for j in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[j], car_yb[j] * car_y_ratio, expand_openlist_node_x_vec[i], expand_openlist_node_y_vec[i], expand_openlist_node_phi_vec[i])
        openlist_node_car_x.append(tmp_x)
        openlist_node_car_y.append(tmp_y)
      openlist_node_car_x_vec.append(openlist_node_car_x)
      openlist_node_car_y_vec.append(openlist_node_car_y)

    select_openlist_node_car_x_vec = []
    select_openlist_node_car_y_vec = []
    for i in range(len(select_openlist_node_x_vec)):
      select_openlist_node_car_x = []
      select_openlist_node_car_y = []
      for j in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[j], car_yb[j] * car_y_ratio, select_openlist_node_x_vec[i], select_openlist_node_y_vec[i], select_openlist_node_phi_vec[i])
        select_openlist_node_car_x.append(tmp_x)
        select_openlist_node_car_y.append(tmp_y)
      select_openlist_node_car_x_vec.append(select_openlist_node_car_x)
      select_openlist_node_car_y_vec.append(select_openlist_node_car_y)

    hybrid_ara_path_data['data_ara_expand_cur_node'].data.update({
      'x_vec': expand_cur_node_x_vec,
      'y_vec': expand_cur_node_y_vec,
      'phi_vec': expand_cur_node_phi_vec,
      'agent_cost_vec': expand_cur_node_agent_cost_vec,
      'boundary_cost_vec': expand_cur_node_boundary_cost_vec,
      'center_cost_vec': expand_cur_node_center_cost_vec,
      'motion_cost_vec': expand_cur_node_motion_cost_vec,
      'heuristic_cost_vec': expand_cur_node_heuristic_cost_vec,
      'total_cost_vec': expand_cur_node_total_cost_vec,
    })

    hybrid_ara_path_data['data_ara_expand_open_list_node'].data.update({
      'x_vec': expand_openlist_node_x_vec,
      'y_vec': expand_openlist_node_y_vec,
      'phi_vec': expand_openlist_node_phi_vec,
      'agent_cost_vec': expand_openlist_agent_cost_vec,
      'boundary_cost_vec': expand_openlist_boundary_cost_vec,
      'center_cost_vec': expand_openlist_center_cost_vec,
      'motion_cost_vec': expand_openlist_motion_cost_vec,
      'heuristic_cost_vec': expand_openlist_heuristic_cost_vec,
      'total_cost_vec': expand_openlist_total_cost_vec,
    })

    hybrid_ara_path_data['data_ara_expand_cur_node_car'].data.update({
      'patch_x_vec': cur_node_car_x_vec,
      'patch_y_vec': cur_node_car_y_vec,
    })

    hybrid_ara_path_data['data_ara_expand_open_list_node_car'].data.update({
      'patch_x_vec': openlist_node_car_x_vec,
      'patch_y_vec': openlist_node_car_y_vec,
    })

    hybrid_ara_path_data['data_ara_select_open_list_node_car'].data.update({
      'patch_x_vec': select_openlist_node_car_x_vec,
      'patch_y_vec': select_openlist_node_car_y_vec,
    })

    # hybrid_ara_path_data['data_expand_line'].data.update({
    #   'line_x_vec': expand_line_x_vec,
    #   'line_y_vec': expand_line_y_vec,
    # })

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

    hybrid_ara_path_data['data_init_pos_point'].data.update({
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

    hybrid_ara_path_data['data_lat_motion_plan_input'].data.update({
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

    hybrid_ara_path_data['data_refline'].data.update({
      'raw_refline_x': raw_refline_x,
      'raw_refline_y': raw_refline_y,
    })

    lat_motion_plan_output = plan_debug_msg.lateral_motion_planning_output
    if g_is_display_enu:
      x_vec, y_vec = lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec
    else:
      x_vec, y_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)

    hybrid_ara_path_data['data_lat_motion_plan_output'].data.update({
      'xn_vec': lat_motion_plan_output.x_vec,
      'yn_vec': lat_motion_plan_output.y_vec,
    })

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

      hybrid_ara_path_data['data_planning_n'].data.update({
        'plan_traj_xn':planning_json['traj_x_vec'],
        'plan_traj_yn':planning_json['traj_y_vec'],
      })

    hybrid_ara_path_data['data_planning'].data.update({
      'plan_traj_y' : plan_traj_y,
      'plan_traj_x' : plan_traj_x,
    })

  # 加载车道线信息
  if plan_msg.trajectory.trajectory_type == 0: # 实时轨迹
    is_enu_to_car = False

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
      0:hybrid_ara_path_data['data_lane_0'],
      1:hybrid_ara_path_data['data_lane_1'],
      2:hybrid_ara_path_data['data_lane_2'],
      3:hybrid_ara_path_data['data_lane_3'],
      4:hybrid_ara_path_data['data_lane_4'],
      5:hybrid_ara_path_data['data_lane_5'],
      6:hybrid_ara_path_data['data_lane_6'],
      7:hybrid_ara_path_data['data_lane_7'],
      8:hybrid_ara_path_data['data_lane_8'],
      9:hybrid_ara_path_data['data_lane_9'],
    }
    data_center_line_dict = {
      0:hybrid_ara_path_data['data_center_line_0'],
      1:hybrid_ara_path_data['data_center_line_1'],
      2:hybrid_ara_path_data['data_center_line_2'],
      3:hybrid_ara_path_data['data_center_line_3'],
      4:hybrid_ara_path_data['data_center_line_4'],
    }

    for i in range(10):
      try:
        if line_info_list[i]['type'] == ['dashed']:
          fig2.renderers[0 + i].glyph.line_dash = 'dashed'
        else:
          fig2.renderers[0 + i].glyph.line_dash = 'solid'
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
    hybrid_ara_path_data['data_control'].data.update({
        'mpc_dx' : mpc_dx,
        'mpc_dy' : mpc_dy,
    })

  obs_info_all = load_ara_obstacles_params(local_view_data)

  hybrid_ara_path_data['ara_obs_data'].data.update({
    'obstacles_x_rel': obs_info_all['obstacles_x_rel'],
    'obstacles_y_rel':  obs_info_all['obstacles_y_rel'],
    'obstacles_x':  obs_info_all['obstacles_x'],
    'obstacles_y':  obs_info_all['obstacles_y'],
    'pose_x':  obs_info_all['pose_x'],
    'pose_y':  obs_info_all['pose_y'],
    'obstacles_label':  obs_info_all['obstacles_label'],
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

def load_hybrid_ara_path_figure(fig1):
  data_arastar = ColumnDataSource(data = {'x_vec':[],
                                          'y_vec':[],
                                          'phi_vec':[],
                                          'xn_vec':[],
                                          'yn_vec':[]})

  data_arastar_cost = ColumnDataSource(data = {'x_vec':[],
                                          'y_vec':[],
                                          'phi_vec':[],
                                          'xn_vec':[],
                                          'yn_vec':[],
                                          'agent_cost_vec':[],
                                          'boundary_cost_vec':[],
                                          'center_cost_vec':[],
                                          'motion_cost_vec':[]})

  data_ara_expand_cur_node = ColumnDataSource(data = {'x_vec':[],
                                                      'y_vec':[],
                                                      'phi_vec':[],
                                                      'agent_cost_vec':[],
                                                      'boundary_cost_vec':[],
                                                      'center_cost_vec':[],
                                                      'motion_cost_vec':[],
                                                      'heuristic_cost_vec':[],
                                                      'total_cost_vec':[]})

  data_ara_expand_open_list_node = ColumnDataSource(data = {'x_vec':[],
                                                            'y_vec':[],
                                                            'phi_vec':[],
                                                            'agent_cost_vec':[],
                                                            'boundary_cost_vec':[],
                                                            'center_cost_vec':[],
                                                            'motion_cost_vec':[],
                                                            'heuristic_cost_vec':[],
                                                            'total_cost_vec':[]})

  data_expand_line = ColumnDataSource(data = {'line_x_vec':[],
                                              'line_y_vec':[]})

  data_ara_expand_cur_node_car = ColumnDataSource(data = {'patch_x_vec':[],
                                                            'patch_y_vec':[]})

  data_ara_expand_open_list_node_car = ColumnDataSource(data = {'patch_x_vec':[],
                                                                  'patch_y_vec':[]})

  data_ara_select_open_list_node_car = ColumnDataSource(data = {'patch_x_vec':[],
                                                                  'patch_y_vec':[]})

  data_refline = ColumnDataSource(data = {'raw_refline_x':[],
                                          'raw_refline_y':[],})

  data_lat_motion_plan_input = ColumnDataSource(data = {'ref_x':[],
                                                        'ref_y':[],
                                                        'ref_xn':[],
                                                        'ref_yn':[],
                                                        })

  data_lat_motion_plan_output = ColumnDataSource(data = {'xn_vec':[],
                                                         'yn_vec':[],
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
  ara_obs_data = ColumnDataSource(data = {'obstacles_x_rel': [],
    'obstacles_y_rel': [],
    'obstacles_x': [],
    'obstacles_y': [],
    'pose_x': [],
    'pose_y': [],
    'obstacles_label': [],})
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
  hybrid_ara_path_data = {'data_arastar': data_arastar,
                          'data_arastar_cost': data_arastar_cost,
                          'data_ara_expand_cur_node': data_ara_expand_cur_node,
                          'data_ara_expand_open_list_node': data_ara_expand_open_list_node,
                          'data_expand_line': data_expand_line,
                          'data_ara_expand_cur_node_car': data_ara_expand_cur_node_car,
                          'data_ara_expand_open_list_node_car': data_ara_expand_open_list_node_car,
                          'data_ara_select_open_list_node_car': data_ara_select_open_list_node_car,
                          'data_lat_motion_plan_input':data_lat_motion_plan_input,
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
                          'ara_obs_data': ara_obs_data, \
  }


  # hybrid ara path
  fig1.line('raw_refline_y', 'raw_refline_x', source = data_refline, line_width = 3, line_color = 'blue', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline', visible=False)
  fig1.line('ref_y', 'ref_x', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path')

  fig1.line('y_vec', 'x_vec', source = data_arastar, line_width = 5, line_color = 'purple', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hybrid ara path', visible=True)
  f1 = fig1.circle('y_vec','x_vec', source = data_arastar, size = 6, line_width = 4, line_color = "darkorange", line_alpha = 0.7, fill_color = 'gold',fill_alpha = 0.7, legend_label = 'hybrid ara path')
  hover1 = HoverTool(renderers=[f1], tooltips=[('index', '$index'), ('x', '@x_vec'), ('y', '@y_vec'), ('phi', '@phi_vec')])
  fig1.add_tools(hover1)

  fig1.line('y_vec','x_vec', source = data_arastar_cost, line_width = 4, line_color = "orange", line_dash = 'solid', line_alpha = 0.7, legend_label = 'hybrid ara path cost', visible=True)
  f2 = fig1.circle('y_vec','x_vec', source = data_arastar_cost, size = 6, line_width = 4, line_color = "red", line_alpha = 0.7, fill_color = 'gold',fill_alpha = 0.7, legend_label = 'hybrid ara path cost')
  hover2 = HoverTool(renderers=[f2], tooltips=[('index', '$index'), ('x', '@x_vec'), ('y', '@y_vec'), ('phi', '@phi_vec'), ('agent_cost', '@agent_cost_vec'), ('boundary_cost', '@boundary_cost_vec'), ('center_cost', '@center_cost_vec'), ('motion_cost', '@motion_cost_vec')])
  fig1.add_tools(hover2)

  # fig1.multi_line('line_y_vec', 'line_x_vec', source = data_expand_line, line_width = 2, line_color = 'black', line_dash = 'dotted', legend_label = 'hybrid ara expand')
  fig1.patches('patch_y_vec', 'patch_x_vec', source = data_ara_expand_cur_node_car, fill_color = "red", fill_alpha = 0.5, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'hybrid ara expand')
  fig1.patches('patch_y_vec', 'patch_x_vec', source = data_ara_expand_open_list_node_car, fill_color = "grey", fill_alpha = 0.5, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'hybrid ara expand pose', visible=False)
  fig1.patches('patch_y_vec', 'patch_x_vec', source = data_ara_select_open_list_node_car, fill_color = "grey", fill_alpha = 0.5, line_color = "black", line_alpha = 0.3, line_width = 1)
  f3 = fig1.circle('y_vec','x_vec', source = data_ara_expand_cur_node, size = 8, line_width = 4, line_color = "red", line_alpha = 0.7, fill_color = 'red',fill_alpha = 0.7, legend_label = 'hybrid ara expand')
  hover3 = HoverTool(renderers=[f3], tooltips=[('index', '$index'), ('x', '@x_vec'), ('y', '@y_vec'), ('phi', '@phi_vec'), ('agent_cost', '@agent_cost_vec{0.0000}'), ('boundary_cost', '@boundary_cost_vec{0.0000}'), ('center_cost', '@center_cost_vec{0.0000}'), ('motion_cost', '@motion_cost_vec{0.0000}'), ('heuristic_cost_vec', '@heuristic_cost_vec{0.0000}'), ('total_cost_vec', '@total_cost_vec{0.0000}')])
  fig1.add_tools(hover3)
  f4 = fig1.circle('y_vec','x_vec', source = data_ara_expand_open_list_node, size = 8, line_width = 4, line_color = "grey", line_alpha = 0.7, fill_color = 'grey',fill_alpha = 0.7, legend_label = 'hybrid ara expand')
  hover4 = HoverTool(renderers=[f4], tooltips=[('index', '$index'), ('x', '@x_vec'), ('y', '@y_vec'), ('phi', '@phi_vec'), ('agent_cost', '@agent_cost_vec{0.0000}'), ('boundary_cost', '@boundary_cost_vec{0.0000}'), ('center_cost', '@center_cost_vec{0.0000}'), ('motion_cost', '@motion_cost_vec{0.0000}'), ('heuristic_cost_vec', '@heuristic_cost_vec{0.0000}'), ('total_cost_vec', '@total_cost_vec{0.0000}')])
  fig1.add_tools(hover4)
  fig1.patches('obstacles_y', 'obstacles_x', source = ara_obs_data, fill_color = "green", line_color = "black", line_width = 1, fill_alpha = 0.4, legend_label = 'ara obj')
  fig1.text('pose_y', 'pose_x', text = 'obstacles_label', source = ara_obs_data, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'ara_obs_info')

  # g
  fig2 = bkp.figure(x_axis_label='y', y_axis_label='x', width=800, height=800, match_aspect = True, aspect_scale=1)
  fig2.x_range.flipped = True
  fig2.line('line_0_y', 'line_0_x', source = data_lane_0, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig2.line('line_1_y', 'line_1_x', source = data_lane_1, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig2.line('line_2_y', 'line_2_x', source = data_lane_2, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig2.line('line_3_y', 'line_3_x', source = data_lane_3, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig2.line('line_4_y', 'line_4_x', source = data_lane_4, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig2.line('line_5_y', 'line_5_x', source = data_lane_5, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig2.line('line_6_y', 'line_6_x', source = data_lane_6, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig2.line('line_7_y', 'line_7_x', source = data_lane_7, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig2.line('line_8_y', 'line_8_x', source = data_lane_8, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig2.line('line_9_y', 'line_9_x', source = data_lane_9, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig2.patch('car_yn2', 'car_xn2', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
  fig2.circle('ego_pos_point_y', 'ego_pos_point_x', source = data_ego_pos_point, radius = 0.1, line_width = 2,  line_color = 'purple', line_alpha = 1, fill_alpha = 1, legend_label = 'ego_pos_point')
  fig2.circle('init_pos_point_y', 'init_pos_point_x', source = data_init_pos_point, radius = 0.1, line_width = 2,  line_color = 'black', line_alpha = 1, fill_color = "deepskyblue", fill_alpha = 1, legend_label = 'init_state')
  fig2.line('center_line_0_y', 'center_line_0_x', source = data_center_line_0, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig2.line('center_line_1_y', 'center_line_1_x', source = data_center_line_1, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig2.line('center_line_2_y', 'center_line_2_x', source = data_center_line_2, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig2.line('center_line_3_y', 'center_line_3_x', source = data_center_line_3, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig2.line('center_line_4_y', 'center_line_4_x', source = data_center_line_4, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig2.line('ego_yn', 'ego_xn', source = data_ego, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig2.line('yn_vec', 'xn_vec', source = data_arastar, line_width = 5, line_color = 'green', line_dash = 'solid', line_alpha = 0.6, legend_label = 'hybrid ara path')
  fig2.line('ref_yn', 'ref_xn', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path')
  fig2.line('yn_vec', 'xn_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'red', line_dash = 'dashed', line_alpha = 0.4, legend_label = 'plan path')
  fig2.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan')
  fig2.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 5, line_color = 'green', line_dash = 'dashed', line_alpha = 0.8, legend_label = 'ctrl_traj')

  # phi

  fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)

  fig2.legend.click_policy = 'hide'

  return fig1, fig2, hybrid_ara_path_data