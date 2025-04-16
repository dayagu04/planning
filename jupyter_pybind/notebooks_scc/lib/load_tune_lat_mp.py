import lib.load_global_var as global_var
from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *

import numpy as np
import time
import ipywidgets
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from IPython.core.display import display, HTML
from bokeh.models import Label, DataTable, TableColumn
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

def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

# def update_tune_lat_plan_data(bag_loader, bag_time, local_view_data, lat_plan_data, upper_safe_bound, lower_safe_bound, g_is_display_enu = False):
def update_tune_lat_plan_data(fig7, bag_loader, bag_time, next_bag_time, local_view_data, lat_plan_data, tuned_ref_xy, upper_safe_bound, lower_safe_bound, upper_hard_bound, lower_hard_bound,
                              safe_ub_start_idx, safe_ub_end_idx, safe_lb_start_idx, safe_lb_end_idx, hard_ub_start_idx, hard_ub_end_idx, hard_lb_start_idx, hard_lb_end_idx, g_is_display_enu = False):
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
  # vs_msg = find_nearest(bag_loader.vs_msg, bag_time)
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
    loc_msg_tmp = find(bag_loader.loc_msg, localization_timestamp)
    if loc_msg_tmp != None:
      loc_msg = loc_msg_tmp
  try:
    next_plan_debug_msg = find_nearest(bag_loader.plan_debug_msg, next_bag_time)
    next_road_msg = find_nearest(bag_loader.road_msg, next_bag_time)
    next_input_topic_timestamp = next_plan_debug_msg.input_topic_timestamp
    next_fusion_road_timestamp = next_input_topic_timestamp.fusion_road
    if is_match_planning:
      next_road_msg_tmp = find(bag_loader.road_msg, next_fusion_road_timestamp)
      if next_road_msg_tmp != None:
        next_road_msg = next_road_msg_tmp
  except:
    print("no next msg!")

  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = loc_msg.position.position_boot.x
    cur_pos_yn = loc_msg.position.position_boot.y
    cur_yaw = loc_msg.orientation.euler_boot.yaw
    planning_json = plan_debug_json_msg

    planning_debug = plan_debug_msg

    debug1, debug2 = load_lat_common(planning_debug, planning_json)
    print(debug2)

    ego_xn_temp, ego_yn_temp = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.loc_msg['data'])):
      pos_xn_i = bag_loader.loc_msg['data'][i].position.position_boot.x
      pos_yn_i = bag_loader.loc_msg['data'][i].position.position_boot.y

      ego_xn_temp.append(pos_xn_i)
      ego_yn_temp.append(pos_yn_i)

    if g_is_display_enu:
      ego_xn, ego_yn = coord_tf.global_to_local(ego_xn_temp, ego_yn_temp)
    else:
      ego_xn, ego_yn = ego_xn_temp, ego_yn_temp

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
    else:
      lat_plan_data['data_car'].data.update({
        'car_xn2': car_xb,
        'car_yn2': car_yb,
      })

    # try:
    #   json_pos_x = planning_json['ego_pos_x']
    #   json_pos_y = planning_json['ego_pos_y']
    #   json_yaw = planning_json['ego_pos_yaw']
    #   coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
    # except:
    coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

  if bag_loader.plan_debug_msg['enable'] == True:
    lat_motion_plan_input = plan_debug_msg.lateral_motion_planning_input

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

    if(len(soft_upper_bound_x0_vec) > 1):
      soft_upper_bound_x0_vec[len(soft_upper_bound_x0_vec) - 1] = soft_upper_bound_x1_vec[len(soft_upper_bound_x1_vec) - 1]
      soft_upper_bound_y0_vec[len(soft_upper_bound_y0_vec) - 1] = soft_upper_bound_y1_vec[len(soft_upper_bound_y1_vec) - 1]
      soft_lower_bound_x0_vec[len(soft_lower_bound_x0_vec) - 1] = soft_lower_bound_x1_vec[len(soft_lower_bound_x1_vec) - 1]
      soft_lower_bound_y0_vec[len(soft_lower_bound_y0_vec) - 1] = soft_lower_bound_y1_vec[len(soft_lower_bound_y1_vec) - 1]
      hard_upper_bound_x0_vec[len(hard_upper_bound_x0_vec) - 1] = hard_upper_bound_x1_vec[len(hard_upper_bound_x1_vec) - 1]
      hard_upper_bound_y0_vec[len(hard_upper_bound_y0_vec) - 1] = hard_upper_bound_y1_vec[len(hard_upper_bound_y1_vec) - 1]
      hard_lower_bound_x0_vec[len(hard_lower_bound_x0_vec) - 1] = hard_lower_bound_x1_vec[len(hard_lower_bound_x1_vec) - 1]
      hard_lower_bound_y0_vec[len(hard_lower_bound_y0_vec) - 1] = hard_lower_bound_y1_vec[len(hard_lower_bound_y1_vec) - 1]

    ref_x, ref_y = lat_motion_plan_input.ref_x_vec, lat_motion_plan_input.ref_y_vec
    ref_xn, ref_yn = lat_motion_plan_input.ref_x_vec, lat_motion_plan_input.ref_y_vec
    tmp_ref_x = []
    tmp_ref_y = []
    for i in range(len(ref_x)):
      ref_unit_vector = [soft_upper_bound_x0_vec[i] - soft_lower_bound_x0_vec[i],
                         soft_upper_bound_y0_vec[i] - soft_lower_bound_y0_vec[i]]
      ref_unit_vector = list(normalize_vector(ref_unit_vector))
      tmp_ref_x.append(ref_x[i] + ref_unit_vector[0] * tuned_ref_xy)
      tmp_ref_y.append(ref_y[i] + ref_unit_vector[1] * tuned_ref_xy)

    if g_is_display_enu:
      ref_x, ref_y = tmp_ref_x, tmp_ref_y
      ref_xn, ref_yn = coord_tf.global_to_local(tmp_ref_x, tmp_ref_y)
    else:
      ref_x, ref_y = coord_tf.global_to_local(tmp_ref_x, tmp_ref_y)
      ref_xn, ref_yn = tmp_ref_x, tmp_ref_y

    # tune the path bound and soft bound
    tmp_soft_upper_bound_x0_vec = []
    tmp_soft_upper_bound_y0_vec = []

    tmp_soft_lower_bound_x0_vec = []
    tmp_soft_lower_bound_y0_vec = []

    tmp_hard_upper_bound_x0_vec = []
    tmp_hard_upper_bound_y0_vec = []

    tmp_hard_lower_bound_x0_vec = []
    tmp_hard_lower_bound_y0_vec = []

    if (safe_ub_start_idx < 0):
      for i in range(len(soft_upper_bound_x0_vec)):
        upper_unit_vector = [soft_upper_bound_x0_vec[i] - soft_lower_bound_x0_vec[i],
                             soft_upper_bound_y0_vec[i] - soft_lower_bound_y0_vec[i]]
        upper_unit_vector = list(normalize_vector(upper_unit_vector))
        tmp_soft_upper_bound_x0_vec.append(soft_upper_bound_x0_vec[i] + upper_unit_vector[0] * upper_safe_bound)
        tmp_soft_upper_bound_y0_vec.append(soft_upper_bound_y0_vec[i] + upper_unit_vector[1] * upper_safe_bound)
    elif (len(soft_upper_bound_x0_vec) > 0):
      for j in range(0, safe_ub_start_idx):
        tmp_soft_upper_bound_x0_vec.append(soft_upper_bound_x0_vec[j])
        tmp_soft_upper_bound_y0_vec.append(soft_upper_bound_y0_vec[j])

      for i in range(safe_ub_start_idx, safe_ub_end_idx):
        upper_unit_vector = [soft_upper_bound_x0_vec[i] - soft_lower_bound_x0_vec[i],
                             soft_upper_bound_y0_vec[i] - soft_lower_bound_y0_vec[i]]
        upper_unit_vector = list(normalize_vector(upper_unit_vector))
        tmp_soft_upper_bound_x0_vec.append(soft_upper_bound_x0_vec[i] + upper_unit_vector[0]*upper_safe_bound)
        tmp_soft_upper_bound_y0_vec.append(soft_upper_bound_y0_vec[i] + upper_unit_vector[1]*upper_safe_bound)

      for m in range(safe_ub_end_idx, len(soft_upper_bound_x0_vec)):
        upper_unit_vector = [soft_upper_bound_x0_vec[m] - soft_lower_bound_x0_vec[m],
                             soft_upper_bound_y0_vec[m] - soft_lower_bound_y0_vec[m]]
        upper_unit_vector = list(normalize_vector(upper_unit_vector))
        tmp_soft_upper_bound_x0_vec.append(soft_upper_bound_x0_vec[m])
        tmp_soft_upper_bound_y0_vec.append(soft_upper_bound_y0_vec[m])

    if (safe_lb_start_idx < 0):
      for i in range(len(soft_lower_bound_x0_vec)):
        lower_unit_vector = [soft_lower_bound_x0_vec[i] - soft_upper_bound_x0_vec[i],
                             soft_lower_bound_y0_vec[i] - soft_upper_bound_y0_vec[i]]
        lower_unit_vector = list(normalize_vector(lower_unit_vector))
        tmp_soft_lower_bound_x0_vec.append(soft_lower_bound_x0_vec[i] + lower_unit_vector[0]*lower_safe_bound)
        tmp_soft_lower_bound_y0_vec.append(soft_lower_bound_y0_vec[i] + lower_unit_vector[1]*lower_safe_bound)
    elif (len(soft_lower_bound_x0_vec) > 0):
      for j in range(0, safe_lb_start_idx):
        tmp_soft_lower_bound_x0_vec.append(soft_lower_bound_x0_vec[j])
        tmp_soft_lower_bound_y0_vec.append(soft_lower_bound_y0_vec[j])

      for i in range(safe_lb_start_idx, safe_lb_end_idx):
        lower_unit_vector = [soft_lower_bound_x0_vec[i] - soft_upper_bound_x0_vec[i],
                             soft_lower_bound_y0_vec[i] - soft_upper_bound_y0_vec[i]]
        lower_unit_vector = list(normalize_vector(lower_unit_vector))
        tmp_soft_lower_bound_x0_vec.append(soft_lower_bound_x0_vec[i] + lower_unit_vector[0]*lower_safe_bound)
        tmp_soft_lower_bound_y0_vec.append(soft_lower_bound_y0_vec[i] + lower_unit_vector[1]*lower_safe_bound)

      for m in range(safe_lb_end_idx, len(soft_lower_bound_x0_vec)):
        lower_unit_vector = [soft_lower_bound_x0_vec[m] - soft_upper_bound_x0_vec[m],
                             soft_lower_bound_y0_vec[m] - soft_upper_bound_y0_vec[m]]
        lower_unit_vector = list(normalize_vector(lower_unit_vector))
        tmp_soft_lower_bound_x0_vec.append(soft_lower_bound_x0_vec[m])
        tmp_soft_lower_bound_y0_vec.append(soft_lower_bound_y0_vec[m])

    if (hard_ub_start_idx < 0):
      for i in range(len(hard_upper_bound_x0_vec)):
        upper_unit_vector = [hard_upper_bound_x0_vec[i] - hard_lower_bound_x0_vec[i],
                             hard_upper_bound_y0_vec[i] - hard_lower_bound_y0_vec[i]]
        upper_unit_vector = list(normalize_vector(upper_unit_vector))
        tmp_hard_upper_bound_x0_vec.append(hard_upper_bound_x0_vec[i] + upper_unit_vector[0]*upper_hard_bound)
        tmp_hard_upper_bound_y0_vec.append(hard_upper_bound_y0_vec[i] + upper_unit_vector[1]*upper_hard_bound)
    elif (len(hard_upper_bound_x0_vec) > 0):
      for j in range(0, hard_ub_start_idx):
        tmp_hard_upper_bound_x0_vec.append(hard_upper_bound_x0_vec[j])
        tmp_hard_upper_bound_y0_vec.append(hard_upper_bound_y0_vec[j])

      for i in range(hard_ub_start_idx, hard_ub_end_idx):
        upper_unit_vector = [hard_upper_bound_x0_vec[i] - hard_lower_bound_x0_vec[i],
                             hard_upper_bound_y0_vec[i] - hard_lower_bound_y0_vec[i]]
        upper_unit_vector = list(normalize_vector(upper_unit_vector))
        tmp_hard_upper_bound_x0_vec.append(hard_upper_bound_x0_vec[i] + upper_unit_vector[0]*upper_hard_bound)
        tmp_hard_upper_bound_y0_vec.append(hard_upper_bound_y0_vec[i] + upper_unit_vector[1]*upper_hard_bound)

      for m in range(hard_ub_end_idx, len(hard_upper_bound_x0_vec)):
        upper_unit_vector = [hard_upper_bound_x0_vec[m] - hard_lower_bound_x0_vec[m],
                             hard_upper_bound_y0_vec[m] - hard_lower_bound_y0_vec[m]]
        upper_unit_vector = list(normalize_vector(upper_unit_vector))
        tmp_hard_upper_bound_x0_vec.append(hard_upper_bound_x0_vec[m])
        tmp_hard_upper_bound_y0_vec.append(hard_upper_bound_y0_vec[m])

    if (hard_lb_start_idx < 0):
      for i in range(len(hard_lower_bound_x0_vec)):
        lower_unit_vector = [hard_lower_bound_x0_vec[i] - hard_upper_bound_x0_vec[i],
                             hard_lower_bound_y0_vec[i] - hard_upper_bound_y0_vec[i]]
        lower_unit_vector = list(normalize_vector(lower_unit_vector))
        tmp_hard_lower_bound_x0_vec.append(hard_lower_bound_x0_vec[i] + lower_unit_vector[0]*lower_hard_bound)
        tmp_hard_lower_bound_y0_vec.append(hard_lower_bound_y0_vec[i] + lower_unit_vector[1]*lower_hard_bound)
    elif (len(hard_lower_bound_x0_vec) > 0):
      for j in range(0, hard_lb_start_idx):
        tmp_hard_lower_bound_x0_vec.append(hard_lower_bound_x0_vec[j])
        tmp_hard_lower_bound_y0_vec.append(hard_lower_bound_y0_vec[j])

      for i in range(hard_lb_start_idx, hard_lb_end_idx):
        lower_unit_vector = [hard_lower_bound_x0_vec[i] - hard_upper_bound_x0_vec[i],
                             hard_lower_bound_y0_vec[i] - hard_upper_bound_y0_vec[i]]
        lower_unit_vector = list(normalize_vector(lower_unit_vector))
        tmp_hard_lower_bound_x0_vec.append(hard_lower_bound_x0_vec[i] + lower_unit_vector[0]*lower_hard_bound)
        tmp_hard_lower_bound_y0_vec.append(hard_lower_bound_y0_vec[i] + lower_unit_vector[1]*lower_hard_bound)

      for m in range(hard_lb_end_idx, len(hard_lower_bound_x0_vec)):
        lower_unit_vector = [hard_lower_bound_x0_vec[m] - hard_upper_bound_x0_vec[m],
                             hard_lower_bound_y0_vec[m] - hard_upper_bound_y0_vec[m]]
        lower_unit_vector = list(normalize_vector(lower_unit_vector))
        tmp_hard_lower_bound_x0_vec.append(hard_lower_bound_x0_vec[m])
        tmp_hard_lower_bound_y0_vec.append(hard_lower_bound_y0_vec[m])

    last_x_vec = []
    last_y_vec = []
    if g_is_display_enu:
      try:
        soft_upper_bound_x0_vec, soft_upper_bound_y0_vec = tmp_soft_upper_bound_x0_vec, tmp_soft_upper_bound_y0_vec
        soft_lower_bound_x0_vec, soft_lower_bound_y0_vec = tmp_soft_lower_bound_x0_vec, tmp_soft_lower_bound_y0_vec
        hard_upper_bound_x0_vec, hard_upper_bound_y0_vec = tmp_hard_upper_bound_x0_vec, tmp_hard_upper_bound_y0_vec
        hard_lower_bound_x0_vec, hard_lower_bound_y0_vec = tmp_hard_lower_bound_x0_vec, tmp_hard_lower_bound_y0_vec
      except:
        print("tuned bound error! plot origin bound!")

      try:
        last_x_vec, last_y_vec = lat_motion_plan_input.last_x_vec, lat_motion_plan_input.last_y_vec
      except:
        print("last traj error!")

    else:
      try:
        soft_upper_bound_x0_vec, soft_upper_bound_y0_vec = coord_tf.global_to_local(tmp_soft_upper_bound_x0_vec,
                                                                                    tmp_soft_upper_bound_y0_vec)

        soft_lower_bound_x0_vec, soft_lower_bound_y0_vec = coord_tf.global_to_local(tmp_soft_lower_bound_x0_vec,
                                                                                    tmp_soft_lower_bound_y0_vec)

        hard_upper_bound_x0_vec, hard_upper_bound_y0_vec = coord_tf.global_to_local(tmp_hard_upper_bound_x0_vec,
                                                                                    tmp_hard_upper_bound_y0_vec)

        hard_lower_bound_x0_vec, hard_lower_bound_y0_vec = coord_tf.global_to_local(tmp_hard_lower_bound_x0_vec,
                                                                                    tmp_hard_lower_bound_y0_vec)
      except:
        soft_upper_bound_x0_vec, soft_upper_bound_y0_vec = coord_tf.global_to_local(soft_upper_bound_x0_vec,
                                                                                    soft_upper_bound_y0_vec)

        soft_lower_bound_x0_vec, soft_lower_bound_y0_vec = coord_tf.global_to_local(soft_lower_bound_x0_vec,
                                                                                    soft_lower_bound_y0_vec)

        hard_upper_bound_x0_vec, hard_upper_bound_y0_vec = coord_tf.global_to_local(hard_upper_bound_x0_vec,
                                                                                    hard_upper_bound_y0_vec)

        hard_lower_bound_x0_vec, hard_lower_bound_y0_vec = coord_tf.global_to_local(hard_lower_bound_x0_vec,
                                                                                    hard_lower_bound_y0_vec)
      try:
        last_x_vec, last_y_vec = coord_tf.global_to_local(lat_motion_plan_input.last_x_vec, lat_motion_plan_input.last_y_vec)
      except:
        print("last traj error!")

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
      'ref_xn': ref_xn,
      'ref_yn': ref_yn,
      'last_x_vec': last_x_vec,
      'last_y_vec': last_y_vec,

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
      raw_refline_x, raw_refline_y = planning_json['raw_refline_x_vec'], planning_json['raw_refline_y_vec']
      raw_refline_xn, raw_refline_yn = coord_tf.global_to_local(planning_json['raw_refline_x_vec'], \
        planning_json['raw_refline_y_vec'])
    else:
      raw_refline_x, raw_refline_y = coord_tf.global_to_local(planning_json['raw_refline_x_vec'], \
        planning_json['raw_refline_y_vec'])
      raw_refline_xn, raw_refline_yn = planning_json['raw_refline_x_vec'], planning_json['raw_refline_y_vec']

    lat_plan_data['data_refline'].data.update({
      'raw_refline_x': raw_refline_x,
      'raw_refline_y': raw_refline_y,
      'raw_refline_xn': raw_refline_xn,
      'raw_refline_yn': raw_refline_yn,
    })

    lat_motion_plan_output = plan_debug_msg.lateral_motion_planning_output
    # if g_is_display_enu:
    #   x_vec, y_vec = lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec
    # else:
    #   x_vec, y_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)
    time_vec = lat_motion_plan_output.time_vec

    ref_theta_deg_vec = []
    theta_deg_vec = []
    last_theta_deg_vec = []
    steer_deg_vec = []
    steer_dot_deg_vec = []

    for i in range(len(time_vec)):
      ref_theta_deg_vec.append(lat_motion_plan_input.ref_theta_vec[i] * 57.3)
      theta_deg_vec.append(lat_motion_plan_output.theta_vec[i] * 57.3)
      last_theta_deg_vec.append(lat_motion_plan_input.last_theta_vec[i] * 57.3)
      steer_deg_vec.append(lat_motion_plan_output.delta_vec[i] * 57.3 * 13.0)
      steer_dot_deg_vec.append(lat_motion_plan_output.omega_vec[i] * 57.3 * 13.0)

    next_ref_theta_deg_vec = []
    try:
      for i in range(len(time_vec)):
        next_ref_theta_deg_vec.append(next_plan_debug_msg.lateral_motion_planning_input.ref_theta_vec[i] * 57.3)
    except:
      print("no next_plan_debug_msg")

    acc_vec = lat_motion_plan_output.acc_vec
    jerk_vec = lat_motion_plan_output.jerk_vec

    xn_vec, yn_vec = lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec
    x_vec, y_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)
    if g_is_display_enu:
      xn_vec, yn_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)
      x_vec, y_vec = lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec

    lat_plan_data['data_lat_motion_plan_output'].data.update({
      'time_vec': time_vec,
      'x_vec': x_vec,
      'y_vec': y_vec,
      'xn_vec': xn_vec,
      'yn_vec': yn_vec,
      'ref_theta_deg_vec': ref_theta_deg_vec,
      'next_ref_theta_deg_vec': next_ref_theta_deg_vec,
      'theta_deg_vec': theta_deg_vec,
      'last_theta_deg_vec': last_theta_deg_vec,
      'steer_deg_vec': steer_deg_vec,
      'steer_dot_deg_vec': steer_dot_deg_vec,
      'acc_vec': acc_vec,
      'jerk_vec': jerk_vec,
    })

    # assembled_delta = []
    # assembled_omega = []
    # for i in range(len(planning_json['assembled_delta'])):
    #   assembled_delta.append(planning_json['assembled_delta'][i] * 57.3 * 15.7)
    #   assembled_omega.append(planning_json['assembled_omega'][i] * 57.3 * 15.7)
    # print("dbw_status = ", planning_json['dbw_status'])
    # print("replan_status = ", planning_json['replan_status'])
    # print("lat_err = ", planning_json['lat_err'])
    # print("theta_err = ", planning_json['theta_err'])
    # print("lon_err = ", planning_json['lon_err'])
    # print("dist_err = ", planning_json['dist_err'])
    print("solver_condition = ", lat_motion_plan_output.solver_info.solver_condition)
    print("iLqr_lat_update_time = ", planning_json['iLqr_lat_update_time'], " ms")
    if len(lat_motion_plan_output.solver_info.iter_info) > 0:
      print("min cost = ", lat_motion_plan_output.solver_info.iter_info[max(lat_motion_plan_output.solver_info.iter_count - 1, 0)].cost)

  if bag_loader.plan_msg['enable'] == True:
    trajectory = plan_msg.trajectory
    if trajectory.trajectory_type == 0: # 实时轨迹
      try:
        planning_polynomial = trajectory.target_reference.polynomial
        plan_traj_x, plan_traj_y = gen_line(planning_polynomial[3],planning_polynomial[2], planning_polynomial[1], planning_polynomial[0], 0, 50)
      except:
        plan_traj_x, plan_traj_y = [], []
    else:
      plan_x = []
      plan_y = []
      for i in range(len(trajectory.trajectory_points)):
        plan_x.append(trajectory.trajectory_points[i].x)
        plan_y.append(trajectory.trajectory_points[i].y)

      if g_is_display_enu:
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

    # step 3: 加载车道线信息
    if plan_msg.trajectory.trajectory_type == 0: # 实时轨迹
      is_enu_to_car = False

  not_g_is_display_enu = g_is_display_enu
  if g_is_display_enu :
    not_g_is_display_enu = False
  else:
    not_g_is_display_enu = True
  if bag_loader.road_msg['enable'] == True:
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
      10:lat_plan_data['data_lane_10'],
      11:lat_plan_data['data_lane_11'],
      12:lat_plan_data['data_lane_12'],
      13:lat_plan_data['data_lane_13'],
      14:lat_plan_data['data_lane_14'],
      15:lat_plan_data['data_lane_15'],
      16:lat_plan_data['data_lane_16'],
      17:lat_plan_data['data_lane_17'],
      18:lat_plan_data['data_lane_18'],
      19:lat_plan_data['data_lane_19'],
    }
    data_center_line_dict = {
      0:lat_plan_data['data_center_line_0'],
      1:lat_plan_data['data_center_line_1'],
      2:lat_plan_data['data_center_line_2'],
      3:lat_plan_data['data_center_line_3'],
      4:lat_plan_data['data_center_line_4'],
      5:lat_plan_data['data_center_line_5'],
      6:lat_plan_data['data_center_line_6'],
      7:lat_plan_data['data_center_line_7'],
      8:lat_plan_data['data_center_line_8'],
      9:lat_plan_data['data_center_line_9'],
    }

    for i in range(20):
      try:
        if line_info_list[i]['type_vec'][0] == ['dashed']:
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

    for i in range(10):
      # try:
        # if (trajectory.trajectory_type == 0) or (trajectory.trajectory_type == 1 and trajectory.target_reference.lateral_maneuver_gear == 2) :
        data_center_line = data_center_line_dict[i]
        data_center_line.data.update({
          'center_line_{}_x'.format(i): center_line_list[i]['line_x_vec'],
          'center_line_{}_y'.format(i): center_line_list[i]['line_y_vec'],
        })
        if center_line_list[i]['relative_id'][0] == 0:
          lat_plan_data['data_center_line_curvature'].data.update({
          'center_line_s' :  center_line_list[i]['line_s_vec'],
          'center_line_curvature' :  center_line_list[i]['curvature_vec'],
          'center_line_d_poly_curvature' :  center_line_list[i]['d_poly_curvature_vec'],
          'center_line_confidence' :  [confidence * 1000.0 for confidence in center_line_list[i]['confidence_vec']],
        })

  try:
    # load lane info
    try:
      next_line_info_list = load_lane_lines(next_road_msg, is_enu_to_car, loc_msg, not_g_is_display_enu)
    except:
      print("vis road_msg error")

    # update lane info
    next_data_lane_dict = {
      0:lat_plan_data['data_lane_20'],
      1:lat_plan_data['data_lane_21'],
      2:lat_plan_data['data_lane_22'],
      3:lat_plan_data['data_lane_23'],
      4:lat_plan_data['data_lane_24'],
      5:lat_plan_data['data_lane_25'],
      6:lat_plan_data['data_lane_26'],
      7:lat_plan_data['data_lane_27'],
      8:lat_plan_data['data_lane_28'],
      9:lat_plan_data['data_lane_29'],
      10:lat_plan_data['data_lane_30'],
      11:lat_plan_data['data_lane_31'],
      12:lat_plan_data['data_lane_32'],
      13:lat_plan_data['data_lane_33'],
      14:lat_plan_data['data_lane_34'],
      15:lat_plan_data['data_lane_35'],
      16:lat_plan_data['data_lane_36'],
      17:lat_plan_data['data_lane_37'],
      18:lat_plan_data['data_lane_38'],
      19:lat_plan_data['data_lane_39'],
    }
    next_data_center_line_dict = {
      0:lat_plan_data['data_center_line_10'],
      1:lat_plan_data['data_center_line_11'],
      2:lat_plan_data['data_center_line_12'],
      3:lat_plan_data['data_center_line_13'],
      4:lat_plan_data['data_center_line_14'],
      5:lat_plan_data['data_center_line_15'],
      6:lat_plan_data['data_center_line_16'],
      7:lat_plan_data['data_center_line_17'],
      8:lat_plan_data['data_center_line_18'],
      9:lat_plan_data['data_center_line_19'],
    }

    for i in range(20):
      try:
        if next_line_info_list[i]['type_vec'][0] == ['dashed']:
          fig7.renderers[20 + i].glyph.line_dash = 'dashed'
        else:
          fig7.renderers[20 + i].glyph.line_dash = 'solid'
        next_data_lane = next_data_lane_dict[i]
        next_data_lane.data.update({
          'line_{}_x'.format(20 + i): next_line_info_list[i]['line_x_vec'],
          'line_{}_y'.format(20 + i): next_line_info_list[i]['line_y_vec'],
        })
      except:
        pass

    next_center_line_list = load_lane_center_lines(next_road_msg, is_enu_to_car, loc_msg, not_g_is_display_enu)

    for i in range(10):
        next_data_center_line = next_data_center_line_dict[i]
        next_data_center_line.data.update({
          'center_line_{}_x'.format(10 + i): next_center_line_list[i]['line_x_vec'],
          'center_line_{}_y'.format(10 + i): next_center_line_list[i]['line_y_vec'],
        })
  except:
    print("no next road!")

  # load control
  if bag_loader.ctrl_msg['enable'] == True:
    mpc_dx, mpc_dy, mpc_dtheta = generate_control(ctrl_msg, loc_msg, not g_is_display_enu)
    lat_plan_data['data_control'].data.update({
        'mpc_dx' : mpc_dx,
        'mpc_dy' : mpc_dy,
    })

def load_lat_plan_figure(fig1):
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
  data_lane_10 = ColumnDataSource(data = {'line_10_y':[], 'line_10_x':[]})
  data_lane_11 = ColumnDataSource(data = {'line_11_y':[], 'line_11_x':[]})
  data_lane_12 = ColumnDataSource(data = {'line_12_y':[], 'line_12_x':[]})
  data_lane_13 = ColumnDataSource(data = {'line_13_y':[], 'line_13_x':[]})
  data_lane_14 = ColumnDataSource(data = {'line_14_y':[], 'line_14_x':[]})
  data_lane_15 = ColumnDataSource(data = {'line_15_y':[], 'line_15_x':[]})
  data_lane_16 = ColumnDataSource(data = {'line_16_y':[], 'line_16_x':[]})
  data_lane_17 = ColumnDataSource(data = {'line_17_y':[], 'line_17_x':[]})
  data_lane_18 = ColumnDataSource(data = {'line_18_y':[], 'line_18_x':[]})
  data_lane_19 = ColumnDataSource(data = {'line_19_y':[], 'line_19_x':[]})

  data_lane_20 = ColumnDataSource(data = {'line_20_y':[], 'line_20_x':[]})
  data_lane_21 = ColumnDataSource(data = {'line_21_y':[], 'line_21_x':[]})
  data_lane_22 = ColumnDataSource(data = {'line_22_y':[], 'line_22_x':[]})
  data_lane_23 = ColumnDataSource(data = {'line_23_y':[], 'line_23_x':[]})
  data_lane_24 = ColumnDataSource(data = {'line_24_y':[], 'line_24_x':[]})
  data_lane_25 = ColumnDataSource(data = {'line_25_y':[], 'line_25_x':[]})
  data_lane_26 = ColumnDataSource(data = {'line_26_y':[], 'line_26_x':[]})
  data_lane_27 = ColumnDataSource(data = {'line_27_y':[], 'line_27_x':[]})
  data_lane_28 = ColumnDataSource(data = {'line_28_y':[], 'line_28_x':[]})
  data_lane_29 = ColumnDataSource(data = {'line_29_y':[], 'line_29_x':[]})
  data_lane_30 = ColumnDataSource(data = {'line_30_y':[], 'line_30_x':[]})
  data_lane_31 = ColumnDataSource(data = {'line_31_y':[], 'line_31_x':[]})
  data_lane_32 = ColumnDataSource(data = {'line_32_y':[], 'line_32_x':[]})
  data_lane_33 = ColumnDataSource(data = {'line_33_y':[], 'line_33_x':[]})
  data_lane_34 = ColumnDataSource(data = {'line_34_y':[], 'line_34_x':[]})
  data_lane_35 = ColumnDataSource(data = {'line_35_y':[], 'line_35_x':[]})
  data_lane_36 = ColumnDataSource(data = {'line_36_y':[], 'line_36_x':[]})
  data_lane_37 = ColumnDataSource(data = {'line_37_y':[], 'line_37_x':[]})
  data_lane_38 = ColumnDataSource(data = {'line_38_y':[], 'line_38_x':[]})
  data_lane_39 = ColumnDataSource(data = {'line_39_y':[], 'line_39_x':[]})

  data_center_line_0 = ColumnDataSource(data = {'center_line_0_y':[], 'center_line_0_x':[]})
  data_center_line_1 = ColumnDataSource(data = {'center_line_1_y':[], 'center_line_1_x':[]})
  data_center_line_2 = ColumnDataSource(data = {'center_line_2_y':[], 'center_line_2_x':[]})
  data_center_line_3 = ColumnDataSource(data = {'center_line_3_y':[], 'center_line_3_x':[]})
  data_center_line_4 = ColumnDataSource(data = {'center_line_4_y':[], 'center_line_4_x':[]})
  data_center_line_5 = ColumnDataSource(data = {'center_line_5_y':[], 'center_line_5_x':[]})
  data_center_line_6 = ColumnDataSource(data = {'center_line_6_y':[], 'center_line_6_x':[]})
  data_center_line_7 = ColumnDataSource(data = {'center_line_7_y':[], 'center_line_7_x':[]})
  data_center_line_8 = ColumnDataSource(data = {'center_line_8_y':[], 'center_line_8_x':[]})
  data_center_line_9 = ColumnDataSource(data = {'center_line_9_y':[], 'center_line_9_x':[]})

  data_center_line_10 = ColumnDataSource(data = {'center_line_10_y':[], 'center_line_10_x':[]})
  data_center_line_11 = ColumnDataSource(data = {'center_line_11_y':[], 'center_line_11_x':[]})
  data_center_line_12 = ColumnDataSource(data = {'center_line_12_y':[], 'center_line_12_x':[]})
  data_center_line_13 = ColumnDataSource(data = {'center_line_13_y':[], 'center_line_13_x':[]})
  data_center_line_14 = ColumnDataSource(data = {'center_line_14_y':[], 'center_line_14_x':[]})
  data_center_line_15 = ColumnDataSource(data = {'center_line_15_y':[], 'center_line_15_x':[]})
  data_center_line_16 = ColumnDataSource(data = {'center_line_16_y':[], 'center_line_16_x':[]})
  data_center_line_17 = ColumnDataSource(data = {'center_line_17_y':[], 'center_line_17_x':[]})
  data_center_line_18 = ColumnDataSource(data = {'center_line_18_y':[], 'center_line_18_x':[]})
  data_center_line_19 = ColumnDataSource(data = {'center_line_19_y':[], 'center_line_19_x':[]})

  data_center_line_curvature = ColumnDataSource(data = {'center_line_s':[],
                                                        'center_line_curvature':[],
                                                        'center_line_d_poly_curvature':[],
                                                        'center_line_confidence':[], })

  data_refline = ColumnDataSource(data = {'raw_refline_x':[],
                                          'raw_refline_y':[],
                                          'raw_refline_xn':[],
                                          'raw_refline_yn':[],})

  data_control = ColumnDataSource(data = {'mpc_dx':[],
                                          'mpc_dy':[],})

  data_lat_motion_plan_input = ColumnDataSource(data = {'ref_x':[],
                                                        'ref_y':[],
                                                        'ref_xn':[],
                                                        'ref_yn':[],
                                                        'last_x_vec': [],
                                                        'last_y_vec': [],
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
                                                         'x_vec':[],
                                                         'y_vec':[],
                                                         'xn_vec':[],
                                                         'yn_vec':[],
                                                         'ref_theta_deg_vec':[],
                                                         'next_ref_theta_deg_vec':[],
                                                         'theta_deg_vec':[],
                                                         'last_theta_deg_vec':[],
                                                         'steer_deg_vec':[],
                                                         'steer_dot_deg_vec':[],
                                                         'acc_vec':[],
                                                         'jerk_vec':[],
                                                         'x_vec_t':[],
                                                         'y_vec_t':[],
                                                         'xn_vec_t':[],
                                                         'yn_vec_t':[],
                                                         'ref_theta_deg_vec_t':[],
                                                         'theta_deg_vec_t':[],
                                                         'steer_deg_vec_t':[],
                                                         'steer_dot_deg_vec_t':[],
                                                         'acc_vec_t':[],
                                                         'jerk_vec_t':[],
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
  lat_plan_data = {'data_lat_motion_plan_input':data_lat_motion_plan_input,
                   'data_lat_motion_plan_output':data_lat_motion_plan_output,
                   'data_refline':data_refline,
                   'data_planning':data_planning,
                   'data_planning_n': data_planning_n,
                   'data_ego': data_ego,
                   'data_car': data_car,
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
                   'data_lane_10':data_lane_10, \
                   'data_lane_11':data_lane_11, \
                   'data_lane_12':data_lane_12, \
                   'data_lane_13':data_lane_13, \
                   'data_lane_14':data_lane_14, \
                   'data_lane_15':data_lane_15, \
                   'data_lane_16':data_lane_16, \
                   'data_lane_17':data_lane_17, \
                   'data_lane_18':data_lane_18, \
                   'data_lane_19':data_lane_19, \
                   'data_lane_20':data_lane_20, \
                   'data_lane_21':data_lane_21, \
                   'data_lane_22':data_lane_22, \
                   'data_lane_23':data_lane_23, \
                   'data_lane_24':data_lane_24, \
                   'data_lane_25':data_lane_25, \
                   'data_lane_26':data_lane_26, \
                   'data_lane_27':data_lane_27, \
                   'data_lane_28':data_lane_28, \
                   'data_lane_29':data_lane_29, \
                   'data_lane_30':data_lane_30, \
                   'data_lane_31':data_lane_31, \
                   'data_lane_32':data_lane_32, \
                   'data_lane_33':data_lane_33, \
                   'data_lane_34':data_lane_34, \
                   'data_lane_35':data_lane_35, \
                   'data_lane_36':data_lane_36, \
                   'data_lane_37':data_lane_37, \
                   'data_lane_38':data_lane_38, \
                   'data_lane_39':data_lane_39, \
                   'data_center_line_0':data_center_line_0, \
                   'data_center_line_1':data_center_line_1, \
                   'data_center_line_2':data_center_line_2, \
                   'data_center_line_3':data_center_line_3, \
                   'data_center_line_4':data_center_line_4, \
                   'data_center_line_5':data_center_line_5, \
                   'data_center_line_6':data_center_line_6, \
                   'data_center_line_7':data_center_line_7, \
                   'data_center_line_8':data_center_line_8, \
                   'data_center_line_9':data_center_line_9, \
                   'data_center_line_10':data_center_line_10, \
                   'data_center_line_11':data_center_line_11, \
                   'data_center_line_12':data_center_line_12, \
                   'data_center_line_13':data_center_line_13, \
                   'data_center_line_14':data_center_line_14, \
                   'data_center_line_15':data_center_line_15, \
                   'data_center_line_16':data_center_line_16, \
                   'data_center_line_17':data_center_line_17, \
                   'data_center_line_18':data_center_line_18, \
                   'data_center_line_19':data_center_line_19, \
                   'data_center_line_curvature':data_center_line_curvature, \
                   'data_control':data_control
  }


  # motion planning
  fig1.line('ref_y', 'ref_x', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path', visible=True)
  fig1.line('soft_upper_bound_y0_vec', 'soft_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = "darkorange", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft upper bound')
  fig1.line('soft_lower_bound_y0_vec', 'soft_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = "darkorange", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft lower bound')
  fig1.line('hard_upper_bound_y0_vec', 'hard_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = 'maroon', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard upper bound')
  fig1.line('hard_lower_bound_y0_vec', 'hard_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = 'maroon', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard lower bound')
  fig1.line('raw_refline_y', 'raw_refline_x', source = data_refline, line_width = 3, line_color = 'blue', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline', visible=False)
  fig1.circle('y_vec', 'x_vec', source = data_lat_motion_plan_output, size = 6, line_width = 5, line_color = 'red', line_alpha = 0.4, fill_color = 'green', fill_alpha = 1.0, legend_label = 'plan path')
  fig1.line('y_vec_t', 'x_vec_t', source = data_lat_motion_plan_output, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.4, legend_label = 'tuned plan path')
  # fig1.line('comb_y_vec', 'comb_x_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'green', line_dash = 'solid', line_alpha = 0.7, legend_label = 'combined path')
  # fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan debug', visible=False)
  fig_soft_ubound = fig1.circle('soft_upper_bound_y0_vec','soft_upper_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "darkorange", line_alpha = 0.7, fill_color = 'gold',fill_alpha = 1.0, legend_label = 'soft upper bound')
  fig_soft_lbound = fig1.circle('soft_lower_bound_y0_vec','soft_lower_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "darkorange", line_alpha = 0.7, fill_color = 'gold',fill_alpha = 1.0, legend_label = 'soft lower bound')
  fig_hard_ubound = fig1.circle('hard_upper_bound_y0_vec','hard_upper_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "maroon", line_alpha = 0.35, fill_color = 'red',fill_alpha = 1.0, legend_label = 'hard upper bound')
  fig_hard_lbound = fig1.circle('hard_lower_bound_y0_vec','hard_lower_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "maroon", line_alpha = 0.35, fill_color = 'red',fill_alpha = 1.0, legend_label = 'hard lower bound')
  fig1.line('last_y_vec', 'last_x_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = 'brown', line_dash = 'solid', line_alpha = 0.35, legend_label = 'last path', visible=False)

  columns = [
        TableColumn(field="bound_t_vec", title="t"),
        TableColumn(field="bound_s_vec", title="s"),
        TableColumn(field="hard_upper_bound_vec", title="hard_upper"),
        TableColumn(field="soft_upper_bound_vec", title="soft_upper"),
        TableColumn(field="soft_lower_bound_vec", title="soft_lower"),
        TableColumn(field="hard_lower_bound_vec", title="hard_lower"),
      ]
  tab1 = DataTable(source = data_lat_motion_plan_input, columns = columns, width = 600, height = 400)

  fig2 = bkp.figure(x_axis_label='time', y_axis_label='theta',x_range = [-0.1, 5.2], width=600, height=180)
  fig3 = bkp.figure(x_axis_label='time', y_axis_label='lat acc',x_range = fig2.x_range, width=600, height=160)
  fig4 = bkp.figure(x_axis_label='time', y_axis_label='lat jerk',x_range = fig2.x_range, width=600, height=160)
  fig5 = bkp.figure(x_axis_label='time', y_axis_label='steer',x_range = fig2.x_range, width=600, height=160)
  fig6 = bkp.figure(x_axis_label='time', y_axis_label='steer dot',x_range = fig2.x_range, width=600, height=160)

  fig7 = bkp.figure(x_axis_label='y', y_axis_label='x', width=800, height=600, match_aspect = True, aspect_scale=1)
  fig7.x_range.flipped = True
  # fig7.x_range.flipped = True
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
  fig7.line('line_10_y', 'line_10_x', source = data_lane_10, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_11_y', 'line_11_x', source = data_lane_11, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_12_y', 'line_12_x', source = data_lane_12, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_13_y', 'line_13_x', source = data_lane_13, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_14_y', 'line_14_x', source = data_lane_14, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_15_y', 'line_15_x', source = data_lane_15, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_16_y', 'line_16_x', source = data_lane_16, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_17_y', 'line_17_x', source = data_lane_17, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_18_y', 'line_18_x', source = data_lane_18, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_19_y', 'line_19_x', source = data_lane_19, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig7.line('line_20_y', 'line_20_x', source = data_lane_20, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_21_y', 'line_21_x', source = data_lane_21, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_22_y', 'line_22_x', source = data_lane_22, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_23_y', 'line_23_x', source = data_lane_23, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_24_y', 'line_24_x', source = data_lane_24, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_25_y', 'line_25_x', source = data_lane_25, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_26_y', 'line_26_x', source = data_lane_26, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_27_y', 'line_27_x', source = data_lane_27, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_28_y', 'line_28_x', source = data_lane_28, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_29_y', 'line_29_x', source = data_lane_29, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_30_y', 'line_30_x', source = data_lane_30, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_31_y', 'line_31_x', source = data_lane_31, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_32_y', 'line_32_x', source = data_lane_32, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_33_y', 'line_33_x', source = data_lane_33, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_34_y', 'line_34_x', source = data_lane_34, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_35_y', 'line_35_x', source = data_lane_35, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_36_y', 'line_36_x', source = data_lane_36, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_37_y', 'line_37_x', source = data_lane_37, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_38_y', 'line_38_x', source = data_lane_38, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.line('line_39_y', 'line_39_x', source = data_lane_39, line_width = 1.5, line_color = 'gray', line_dash = 'dashed', legend_label = 'next_lane')
  fig7.patch('car_yn2', 'car_xn2', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
  fig7.line('center_line_0_y', 'center_line_0_x', source = data_center_line_0, line_width = 2, line_color = 'blue', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line')
  fig7.line('center_line_1_y', 'center_line_1_x', source = data_center_line_1, line_width = 2, line_color = 'blue', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line')
  fig7.line('center_line_2_y', 'center_line_2_x', source = data_center_line_2, line_width = 2, line_color = 'blue', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line')
  fig7.line('center_line_3_y', 'center_line_3_x', source = data_center_line_3, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig7.line('center_line_4_y', 'center_line_4_x', source = data_center_line_4, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig7.line('center_line_10_y', 'center_line_10_x', source = data_center_line_10, line_width = 2, line_color = 'brown', line_dash = 'dotted', line_alpha = 1, legend_label = 'next_center_line')
  fig7.line('center_line_11_y', 'center_line_11_x', source = data_center_line_11, line_width = 2, line_color = 'brown', line_dash = 'dotted', line_alpha = 1, legend_label = 'next_center_line')
  fig7.line('center_line_12_y', 'center_line_12_x', source = data_center_line_12, line_width = 2, line_color = 'brown', line_dash = 'dotted', line_alpha = 1, legend_label = 'next_center_line')
  fig7.line('center_line_13_y', 'center_line_13_x', source = data_center_line_13, line_width = 1, line_color = 'brown', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'next_center_line')
  fig7.line('center_line_14_y', 'center_line_14_x', source = data_center_line_14, line_width = 1, line_color = 'brown', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'next_center_line')
  fig7.line('ref_yn', 'ref_xn', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path')
  fig7.line('ego_yn', 'ego_xn', source = data_ego, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig7.line('yn_vec', 'xn_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'red', line_dash = 'dashed', line_alpha = 0.4, legend_label = 'plan path')
  fig7.line('yn_vec_t', 'xn_vec_t', source = data_lat_motion_plan_output, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.4, legend_label = 'tuned plan path')
  fig7.line('raw_refline_yn', 'raw_refline_xn', source = data_refline, line_width = 3, line_color = 'green', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline', visible=True)
  fig7.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 5, line_color = 'green', line_dash = 'dashed', line_alpha = 0.8, legend_label = 'ctrl_traj')

  f2 = fig2.line('time_vec', 'ref_theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'ref_theta')
  fig2.line('time_vec', 'theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin theta')
  fig2.line('time_vec', 'theta_deg_vec_t', source = data_lat_motion_plan_output, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'tuned theta')
  fig2.line('time_vec', 'next_ref_theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'orange', line_dash = 'dashed', legend_label = 'next ref_theta')
  fig2.line('time_vec', 'last_theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'brown', line_dash = 'solid', legend_label = 'last traj theta', visible=False)

  f3 = fig3.line('time_vec', 'acc_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin lat acc')
  fig3.line('time_vec', 'acc_vec_t', source = data_lat_motion_plan_output, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'tuned lat acc')
  fig3.line('time_vec', 'acc_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat acc corridor')
  fig3.line('time_vec', 'acc_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat acc corridor')
  fig3.triangle ('time_vec', 'acc_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat acc corridor')
  fig3.inverted_triangle ('time_vec', 'acc_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat acc corridor')

  f4 = fig4.line('time_vec', 'jerk_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin lat jerk')
  fig4.line('time_vec', 'jerk_vec_t', source = data_lat_motion_plan_output, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'tuned lat jerk')
  fig4.line('time_vec', 'jerk_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat jerk corridor')
  fig4.line('time_vec', 'jerk_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat jerk corridor')
  fig4.triangle ('time_vec', 'jerk_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat jerk corridor')
  fig4.inverted_triangle ('time_vec', 'jerk_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat jerk corridor')

  f5 = fig5.line('time_vec', 'steer_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin steer deg')
  fig5.line('time_vec', 'steer_deg_vec_t', source = data_lat_motion_plan_output, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'tuned steer deg')
  fig5.line('time_vec', 'steer_deg_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer deg corridor')
  fig5.line('time_vec', 'steer_deg_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer deg corridor')
  fig5.triangle ('time_vec', 'steer_deg_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer deg corridor')
  fig5.inverted_triangle ('time_vec', 'steer_deg_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer deg corridor')

  f6 = fig6.line('time_vec', 'steer_dot_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin steer dot deg')
  fig6.line('time_vec', 'steer_dot_deg_vec_t', source = data_lat_motion_plan_output, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'tuned steer dot deg')
  fig6.line('time_vec', 'steer_dot_deg_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer dot deg corridor')
  fig6.line('time_vec', 'steer_dot_deg_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer dot deg corridor')
  fig6.triangle ('time_vec', 'steer_dot_deg_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer dot deg corridor')
  fig6.inverted_triangle ('time_vec', 'steer_dot_deg_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer dot deg corridor')

  hover1_1 = HoverTool(renderers=[fig_soft_ubound], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @soft_upper_bound_vec)'),
                                                              ('obstacle id', '@soft_upper_bound_id_vec'), ('type', '@soft_upper_bound_type_vec')])
  hover1_2 = HoverTool(renderers=[fig_soft_lbound], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @soft_lower_bound_vec)'),
                                                              ('obstacle id', '@soft_lower_bound_id_vec'), ('type', '@soft_lower_bound_type_vec')])
  hover1_3 = HoverTool(renderers=[fig_hard_ubound], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @hard_upper_bound_vec)'),
                                                              ('obstacle id', '@hard_upper_bound_id_vec'), ('type', '@hard_upper_bound_type_vec')])
  hover1_4 = HoverTool(renderers=[fig_hard_lbound], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @hard_lower_bound_vec)'),
                                                              ('obstacle id', '@hard_lower_bound_id_vec'), ('type', '@hard_lower_bound_type_vec')])
  hover2 = HoverTool(renderers=[f2], tooltips=[('time', '@time_vec'), ('ref_theta', '@ref_theta_deg_vec'), ('origin theta', '@theta_deg_vec'), ('tuned theta', '@theta_deg_vec_t'), ('next_ref_theta', '@next_ref_theta_deg_vec'), ('last_traj_theta', '@last_theta_deg_vec')], mode='vline')
  hover3 = HoverTool(renderers=[f3], tooltips=[('time', '@time_vec'), ('origin acc', '@acc_vec'), ('tuned acc', '@acc_vec_t'), ('|acc bound|', '@acc_upper_bound')], mode='vline')
  hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time_vec'), ('origin jerk', '@jerk_vec'), ('tuned jerk', '@jerk_vec_t'), ('|jerk bound|', '@jerk_upper_bound')], mode='vline')
  hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('origin steer', '@steer_deg_vec'), ('tuned steer', '@steer_deg_vec_t'), ('|steer deg bound|', '@steer_deg_upper_bound')], mode='vline')
  hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time_vec'), ('origin steer dot', '@steer_dot_deg_vec'), ('tuned steer dot', '@steer_dot_deg_vec_t'), ('|steer dot deg bound|', '@steer_dot_deg_upper_bound')], mode='vline')

  fig1.add_tools(hover1_1)
  fig1.add_tools(hover1_2)
  fig1.add_tools(hover1_3)
  fig1.add_tools(hover1_4)
  fig2.add_tools(hover2)
  fig3.add_tools(hover3)
  fig4.add_tools(hover4)
  fig5.add_tools(hover5)
  fig6.add_tools(hover6)


  fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
  fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
  fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
  fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
  fig6.toolbar.active_scroll = fig6.select_one(WheelZoomTool)
  fig7.toolbar.active_scroll = fig7.select_one(WheelZoomTool)

  fig2.legend.click_policy = 'hide'
  fig3.legend.click_policy = 'hide'
  fig4.legend.click_policy = 'hide'
  fig5.legend.click_policy = 'hide'
  fig6.legend.click_policy = 'hide'
  fig7.legend.click_policy = 'hide'

  return fig1, fig2, fig3, fig4, fig5, fig6, fig7, lat_plan_data
