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
from cyber_record.record import Record

car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()

def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

# def update_tune_lat_plan_data(bag_loader, bag_time, local_view_data, lat_plan_data, upper_safe_bound, lower_safe_bound, g_is_display_enu = False):
def update_tune_lat_plan_data(bag_loader, bag_time, local_view_data, lat_plan_data, upper_safe_bound, lower_safe_bound, upper_hard_bound, lower_hard_bound, 
                              safe_ub_start_idx, safe_ub_end_idx, safe_lb_start_idx, safe_lb_end_idx, hard_ub_start_idx, hard_ub_end_idx, hard_lb_start_idx, hard_lb_end_idx, g_is_display_enu = False):
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']
  road_msg_idx = local_view_data['data_index']['road_msg_idx']
  fus_msg_idx = local_view_data['data_index']['fus_msg_idx']
  vs_msg_idx = local_view_data['data_index']['vs_msg_idx']
  plan_msg_idx = local_view_data['data_index']['plan_msg_idx']
  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
  pred_msg_idx = local_view_data['data_index']['pred_msg_idx']

  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw
    planning_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]

    planning_debug = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx]
    
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

    try:
      json_pos_x = planning_json['ego_pos_x']
      json_pos_y = planning_json['ego_pos_y']
      json_yaw = planning_json['ego_pos_yaw']
      coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
    except:
      coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

  if bag_loader.plan_debug_msg['enable'] == True:
    lat_motion_plan_input = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input
    if g_is_display_enu:
      ref_x, ref_y = lat_motion_plan_input.ref_x_vec, lat_motion_plan_input.ref_y_vec
    else:
      ref_x, ref_y = coord_tf.global_to_local(lat_motion_plan_input.ref_x_vec, \
        lat_motion_plan_input.ref_y_vec)
    
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
    
    if g_is_display_enu:
      try:
        soft_upper_bound_x0_vec, soft_upper_bound_y0_vec = tmp_soft_upper_bound_x0_vec, tmp_soft_upper_bound_y0_vec
        soft_lower_bound_x0_vec, soft_lower_bound_y0_vec = tmp_soft_lower_bound_x0_vec, tmp_soft_lower_bound_y0_vec
        hard_upper_bound_x0_vec, hard_upper_bound_y0_vec = tmp_hard_upper_bound_x0_vec, tmp_hard_upper_bound_y0_vec
        hard_lower_bound_x0_vec, hard_lower_bound_y0_vec = tmp_hard_lower_bound_x0_vec, tmp_hard_lower_bound_y0_vec
      except:
        print("tuned bound error! plot origin bound!")
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

    if len(soft_upper_bound_x0_vec) == 0 or bag_loader.plan_msg['data'][plan_msg_idx].trajectory.target_reference.lateral_maneuver_gear == 2:
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
      lat_behavior_debug_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_behavior_debug_info
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
      raw_refline_x, raw_refline_y = planning_json['raw_refline_x_vec'], planning_json['raw_refline_y_vec']
    else:
      raw_refline_x, raw_refline_y = coord_tf.global_to_local(planning_json['raw_refline_x_vec'], \
        planning_json['raw_refline_y_vec'])

    lat_plan_data['data_refline'].data.update({
      'raw_refline_x': raw_refline_x,
      'raw_refline_y': raw_refline_y,
      'raw_refline_xn': planning_json['raw_refline_x_vec'],
      'raw_refline_yn': planning_json['raw_refline_y_vec'],
    })

    lat_motion_plan_output = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_output
    # if g_is_display_enu:
    #   x_vec, y_vec = lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec
    # else:
    #   x_vec, y_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)
    time_vec = lat_motion_plan_output.time_vec

    ref_theta_deg_vec = []
    theta_deg_vec = []
    steer_deg_vec = []
    steer_dot_deg_vec =[]

    for i in range(len(time_vec)):
      ref_theta_deg_vec.append(lat_motion_plan_input.ref_theta_vec[i] * 57.3)
      theta_deg_vec.append(lat_motion_plan_output.theta_vec[i] * 57.3)
      steer_deg_vec.append(lat_motion_plan_output.delta_vec[i] * 57.3 * 15.7)
      steer_dot_deg_vec.append(lat_motion_plan_output.omega_vec[i] * 57.3 * 15.7)

    acc_vec = lat_motion_plan_output.acc_vec
    jerk_vec = lat_motion_plan_output.jerk_vec

    lat_plan_data['data_lat_motion_plan_output'].data.update({
      'time_vec': time_vec,
      'xn_vec': lat_motion_plan_output.x_vec,
      'yn_vec': lat_motion_plan_output.y_vec,
      'ref_theta_deg_vec': ref_theta_deg_vec,
      'theta_deg_vec': theta_deg_vec,
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

  if bag_loader.plan_msg['enable'] == True:
    trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
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


def load_lat_plan_figure(fig1):
  data_refline = ColumnDataSource(data = {'raw_refline_x':[],
                                          'raw_refline_y':[],
                                          'raw_refline_xn':[],
                                          'raw_refline_yn':[],})

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
                                                         'xn_vec':[],
                                                         'yn_vec':[],
                                                         'ref_theta_deg_vec':[],
                                                         'theta_deg_vec':[],
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
                                      'car_yn':[],})
  lat_plan_data = {'data_lat_motion_plan_input':data_lat_motion_plan_input,
                   'data_lat_motion_plan_output':data_lat_motion_plan_output,
                   'data_refline':data_refline,
                   'data_planning':data_planning,
                   'data_planning_n': data_planning_n,
                   'data_ego': data_ego,
                   'data_car': data_car,
  }


  # motion planning
  fig1.line('ref_y', 'ref_x', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path', visible=True)
  fig1.line('soft_upper_bound_y0_vec', 'soft_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = "darkorange", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft upper bound')
  fig1.line('soft_lower_bound_y0_vec', 'soft_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = "darkorange", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft lower bound')
  fig1.line('hard_upper_bound_y0_vec', 'hard_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = 'maroon', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard upper bound')
  fig1.line('hard_lower_bound_y0_vec', 'hard_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 4, line_color = 'maroon', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard lower bound')
  fig1.line('raw_refline_y', 'raw_refline_x', source = data_refline, line_width = 3, line_color = 'blue', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline', visible=False)
  # fig1.line('y_vec', 'x_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'red', line_dash = 'dashed', line_alpha = 0.4, legend_label = 'plan path')
  fig1.line('y_vec_t', 'x_vec_t', source = data_lat_motion_plan_output, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.4, legend_label = 'tuned plan path')
  # fig1.line('comb_y_vec', 'comb_x_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'green', line_dash = 'solid', line_alpha = 0.7, legend_label = 'combined path')
  # fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan debug', visible=False)
  fig1.circle('soft_upper_bound_y0_vec','soft_upper_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "darkorange", line_alpha = 0.7, fill_color = 'gold',fill_alpha = 1.0, legend_label = 'soft upper bound')
  fig1.circle('soft_lower_bound_y0_vec','soft_lower_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "darkorange", line_alpha = 0.7, fill_color = 'gold',fill_alpha = 1.0, legend_label = 'soft lower bound')
  fig1.circle('hard_upper_bound_y0_vec','hard_upper_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "maroon", line_alpha = 0.35, fill_color = 'red',fill_alpha = 1.0, legend_label = 'hard upper bound')
  fig1.circle('hard_lower_bound_y0_vec','hard_lower_bound_x0_vec', source = data_lat_motion_plan_input, size = 6, line_width = 4, line_color = "maroon", line_alpha = 0.35, fill_color = 'red',fill_alpha = 1.0, legend_label = 'hard lower bound')

  columns = [
        TableColumn(field="bound_t_vec", title="t"),
        TableColumn(field="bound_s_vec", title="s"),
        TableColumn(field="hard_upper_bound_vec", title="hard_upper"),
        TableColumn(field="soft_upper_bound_vec", title="soft_upper"),
        TableColumn(field="soft_lower_bound_vec", title="soft_lower"),
        TableColumn(field="hard_lower_bound_vec", title="hard_lower"),
      ]
  tab1 = DataTable(source = data_lat_motion_plan_input, columns = columns, width = 600, height = 400)

  fig2 = bkp.figure(x_axis_label='time', y_axis_label='theta',x_range = [-0.1, 5.2], width=600, height=160)
  fig3 = bkp.figure(x_axis_label='time', y_axis_label='lat acc',x_range = fig2.x_range, width=600, height=160)
  fig4 = bkp.figure(x_axis_label='time', y_axis_label='lat jerk',x_range = fig2.x_range, width=600, height=160)
  fig5 = bkp.figure(x_axis_label='time', y_axis_label='steer',x_range = fig2.x_range, width=600, height=160)
  fig6 = bkp.figure(x_axis_label='time', y_axis_label='steer dot',x_range = fig2.x_range, width=600, height=160)

  fig7 = bkp.figure(x_axis_label='x', y_axis_label='y', width=800, height=600, match_aspect = True, aspect_scale=1)
  # fig7.x_range.flipped = True

  fig7.line('ref_xn', 'ref_yn', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path')
  fig7.line('ego_xn', 'ego_yn', source = data_ego, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig7.patch('car_xn', 'car_yn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
  fig7.line('xn_vec', 'yn_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'red', line_dash = 'dashed', line_alpha = 0.4, legend_label = 'plan path')
  fig7.line('xn_vec_t', 'yn_vec_t', source = data_lat_motion_plan_output, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.4, legend_label = 'tuned plan path')
  fig7.line('raw_refline_xn', 'raw_refline_yn', source = data_refline, line_width = 3, line_color = 'green', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline', visible=True)

  f2 = fig2.line('time_vec', 'ref_theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'ref_theta')
  fig2.line('time_vec', 'theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin theta')
  fig2.line('time_vec', 'theta_deg_vec_t', source = data_lat_motion_plan_output, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'tuned theta')

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

  hover1_1 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 4]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @soft_upper_bound_vec)'), 
                                                                                    ('obstacle id', '@soft_upper_bound_id_vec'), ('type', '@soft_upper_bound_type_vec')])
  hover1_2 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 3]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @soft_lower_bound_vec)'), 
                                                                                    ('obstacle id', '@soft_lower_bound_id_vec'), ('type', '@soft_lower_bound_type_vec')])  
  hover1_3 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 2]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @hard_upper_bound_vec)'), 
                                                                                    ('obstacle id', '@hard_upper_bound_id_vec'), ('type', '@hard_upper_bound_type_vec')])
  hover1_4 = HoverTool(renderers=[fig1.renderers[len(fig1.renderers) - 1]], tooltips=[('index', '$index'), ('t', '@bound_t_vec'), ('(s,l)', '(@bound_s_vec, @hard_lower_bound_vec)'), 
                                                                                     ('obstacle id', '@hard_lower_bound_id_vec'), ('type', '@hard_lower_bound_type_vec')])
  hover2 = HoverTool(renderers=[f2], tooltips=[('time', '@time_vec'), ('ref_theta', '@ref_theta_deg_vec'), ('origin theta', '@theta_deg_vec'), ('tuned theta', '@theta_deg_vec_t')], mode='vline')
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
