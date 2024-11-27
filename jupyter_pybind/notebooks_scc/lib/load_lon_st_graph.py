from lib.load_rotate import *
import numpy as np
import time
import ipywidgets
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from IPython.core.display import display, HTML
from bokeh.models import Label, LabelSet, DataTable, DateFormatter, TableColumn, Panel, Tabs
import ipywidgets as widgets
from IPython.display import display
from ipywidgets import Button, HBox
from IPython.display import clear_output
import time
import threading
import ipywidgets
from collections import namedtuple
from functools import  partial
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, HoverTool
from bokeh.plotting import ColumnDataSource
from lib.load_json import *
from lib.load_struct import *

coord_tf = coord_transformer()

def load_st_label(st_graph_data):
  st_label = []
  st_boundary_list = st_graph_data.st_boundaries
  st_boundary_size = len(st_boundary_list)
  for i in range(20):
    if i < st_boundary_size:
      st_boundary = st_boundary_list[i]
      st_label_info = {'center_point_s':[],
                       'center_point_t':[],
                       'agent_id':[]}

      lower_point_s_vec = []
      lower_point_t_vec = []
      upper_point_s_vec = []
      upper_point_t_vec = []
      for point in st_boundary.lower_points:
        lower_point_s_vec.append(point.s)
        lower_point_t_vec.append(point.t)
        agent_id = [str(point.agent_id)]
      for point in st_boundary.upper_points:
        upper_point_s_vec.append(point.s)
        upper_point_t_vec.append(point.t)

      center_point_s = [((lower_point_s_vec[0]+lower_point_s_vec[-1])/2 +
                         (upper_point_s_vec[0]+upper_point_s_vec[-1])/2)/2]
      center_point_t = [((lower_point_t_vec[0]+lower_point_t_vec[-1])/2 +
                         (upper_point_t_vec[0]+upper_point_t_vec[-1])/2)/2]

      st_label_info['center_point_s'] = center_point_s
      st_label_info['center_point_t'] = center_point_t
      st_label_info['agent_id'] = agent_id

      st_label.append(st_label_info)
    else:
      st_label_info = {'center_point_s':[],
                       'center_point_t':[],
                       'agent_id':[]}

      st_label_info['center_point_s'] = [0]
      st_label_info['center_point_t'] = [0]
      st_label_info['agent_id'] = ['']

      st_label.append(st_label_info)

  return st_label

def load_st_polygen_points(st_graph_data):
  st_info = []
  st_boundary_list = st_graph_data.st_boundaries
  st_boundary_size = len(st_boundary_list)
  vec_size = int(st_graph_data.end_time // 0.2) + 1
  default_lower_points_s_vec = [0 for _ in range(vec_size)]
  default_lower_points_t_vec = [0 for _ in range(vec_size)]
  default_upper_points_s_vec = [0 for _ in range(vec_size)]
  default_upper_points_t_vec = [0 for _ in range(vec_size)]
  default_agent_id_vec = [-1] * vec_size
  for i in range(20):
    if i < st_boundary_size:
      st_boundary = st_boundary_list[i]
      st_polygen_line_info = {'agent_id_vec':[],
                              'polygen_lower_points_s_vec':[],
                              'polygen_lower_points_t_vec':[],
                              'polygen_upper_points_s_vec':[],
                              'polygen_upper_points_t_vec':[],}

      # st_polygen_line_info['boundary_id'] = st_boundary.boundary_id
      lower_point_s_vec = []
      lower_point_t_vec = []
      upper_point_s_vec = []
      upper_point_t_vec = []
      agent_id_vec = []
      for point in st_boundary.lower_points:
        lower_point_s_vec.append(point.s)
        lower_point_t_vec.append(point.t)
      for point in st_boundary.upper_points:
        upper_point_s_vec.append(point.s)
        upper_point_t_vec.append(point.t)
        agent_id_vec.append(point.agent_id)

      st_polygen_line_info['agent_id_vec'] = agent_id_vec
      st_polygen_line_info['polygen_lower_points_s_vec'] = lower_point_s_vec
      st_polygen_line_info['polygen_lower_points_t_vec'] = lower_point_t_vec
      st_polygen_line_info['polygen_upper_points_s_vec'] = upper_point_s_vec
      st_polygen_line_info['polygen_upper_points_t_vec'] = upper_point_t_vec
      st_info.append(st_polygen_line_info)
    else:
      st_polygen_line_info = {'agent_id_vec':[],
                              'polygen_lower_points_s_vec':[],
                              'polygen_lower_points_t_vec':[],
                              'polygen_upper_points_s_vec':[],
                              'polygen_upper_points_t_vec':[]}
      st_polygen_line_info['agent_id_vec'] = default_agent_id_vec
      st_polygen_line_info['polygen_lower_points_s_vec'] = default_lower_points_s_vec
      st_polygen_line_info['polygen_lower_points_t_vec'] = default_lower_points_t_vec
      st_polygen_line_info['polygen_upper_points_s_vec'] = default_upper_points_s_vec
      st_polygen_line_info['polygen_upper_points_t_vec'] = default_upper_points_t_vec
      st_info.append(st_polygen_line_info)

  return st_info

def load_st_polygen_lower_upper_point(st_graph_data):
  st_info = []
  st_boundary_list = st_graph_data.st_boundaries
  st_boundary_size = len(st_boundary_list)
  for i in range(20):
    if i < st_boundary_size:
      st_boundary = st_boundary_list[i]
      st_point_info = {'left_point_s':[],
                       'left_point_t':[],
                       'right_point_s':[],
                       'right_point_t':[]}

      lower_point_s_vec = []
      lower_point_t_vec = []
      upper_point_s_vec = []
      upper_point_t_vec = []
      for point in st_boundary.lower_points:
        lower_point_s_vec.append(point.s)
        lower_point_t_vec.append(point.t)
      for point in st_boundary.upper_points:
        upper_point_s_vec.append(point.s)
        upper_point_t_vec.append(point.t)

      left_point_s = [lower_point_s_vec[0], upper_point_s_vec[0]]
      left_point_t = [lower_point_t_vec[0], upper_point_t_vec[0]]
      right_point_s = [lower_point_s_vec[-1], upper_point_s_vec[-1]]
      right_point_t = [lower_point_t_vec[-1], upper_point_t_vec[-1]]

      st_point_info['left_point_s'] = left_point_s
      st_point_info['left_point_t'] = left_point_t
      st_point_info['right_point_s'] = right_point_s
      st_point_info['right_point_t'] = right_point_t
      st_info.append(st_point_info)
    else:
      st_point_info = {'left_point_s':[],
                       'left_point_t':[],
                       'right_point_s':[],
                       'right_point_t':[]}

      st_point_info['left_point_s'] = [0.0, 0.0]
      st_point_info['left_point_t'] = [0.0, 0.0]
      st_point_info['right_point_s'] = [0.0, 0.0]
      st_point_info['right_point_t'] = [0.0, 0.0]
      st_info.append(st_point_info)

  return st_info

def update_lon_plan_data(bag_loader, bag_time, local_view_data, lon_plan_data):
  planning_json_value_list = ['VisionLonBehavior_a_target_high', 'VisionLonBehavior_a_target_low', \
                              "VisionLateralBehaviorPlannerCost", "VisionLateralMotionPlannerCost","VisionLongitudinalBehaviorPlannerCost", \
                              "EnvironmentalModelManagerCost", "GeneralPlannerModuleCostTime", 'construct_st_graph_cost', 'st_graph_searcher_cost', \
                              'v_limit_road', 'v_limit_in_turns','v_target', 'v_ego', \
                              'lead_one_id','cipv_id_st', 'lead_one_dis', 'lead_one_vel', "v_target_lead_one", 'soft_brake_distance_lead',\
                              'lead_two_id', 'lead_two_dis', 'lead_two_vel', "v_target_lead_two", \
                              'temp_lead_one_id', 'temp_lead_one_dis', 'temp_lead_one_vel', "v_target_temp_lead_one", \
                              'temp_lead_two_id', 'temp_lead_two_dis', 'temp_lead_two_vel', "v_target_temp_lead_two", \
                              'potential_cutin_track_id', 'v_target_potential_cutin', "v_target_cutin", "road_radius", \
                              'new_cutin_id', 'new_cutin_id_count', "CIPV_id",\
                              'stop_start_state', 'v_target_start_stop', 'STANDSTILL', 'jlt_status_farslow', \
                              "dis_to_ramp", "v_target_ramp", "narrow_agent_id", "narrow_agent_v_limit",\
                              'gap_v_limit_lc', \
                              "fast_lead_id", "slow_lead_id", "fast_car_cut_in_id", "slow_car_cut_in_id", \
                              "dynamic_world_cost", "front_node_id", "rear_node_id", \
                              "ego_left_node", "ego_left_front_node", "ego_left_rear_node", \
                              "ego_right_node", "ego_right_front_node", "ego_right_rear_node", \
                              "RealTime_desired_distance_rss", "RealTime_desired_distance_calibrate", \
                              'LateralMotionCostTime', 'RealTimeLateralBehaviorCostTime', 'TrajectoryGeneratorCostTime', \
                              "SccLonBehaviorCostTime", "SccLonMotionCostTime"]
  new_cutin_list = ['new_cutin_id', 'new_cutin_id_count']

  plan_debug_info = local_view_data['data_msg']['plan_debug_msg']
  plan_debug_json_info = local_view_data['data_msg']['plan_debug_json_msg']

  # load new st boundaries
  # print(plan_debug_info.st_graph_data.st_boundaries)
  st_info_all = load_st_polygen_points(plan_debug_info.st_graph_data)
  st_point_all = load_st_polygen_lower_upper_point(plan_debug_info.st_graph_data)
  st_label_all = load_st_label(plan_debug_info.st_graph_data)

  data_st_boundaries = {i: lon_plan_data[f'st_boundary_{i}'] for i in range(20)}
  data_st_points = {i: lon_plan_data[f'st_point_{i}'] for i in range(20)}
  data_st_labels = {i: lon_plan_data[f'st_label_{i}'] for i in range(20)}

  for i in range(20):
    data_st_boundary = data_st_boundaries[i]
    data_st_boundary.data.update({
            'lower_points_{}_s_vec'.format(i): st_info_all[i]['polygen_lower_points_s_vec'],
            'lower_points_{}_t_vec'.format(i): st_info_all[i]['polygen_lower_points_t_vec'],
            'upper_points_{}_s_vec'.format(i): st_info_all[i]['polygen_upper_points_s_vec'],
            'upper_points_{}_t_vec'.format(i): st_info_all[i]['polygen_upper_points_t_vec'],
            'agent_{}_id_vec'.format(i): st_info_all[i]['agent_id_vec'],
    })

  for i in range(20):
    data_st_point = data_st_points[i]
    data_st_point.data.update({
       'left_point_{}_s'.format(i): st_point_all[i]['left_point_s'],
       'left_point_{}_t'.format(i): st_point_all[i]['left_point_t'],
       'right_point_{}_s'.format(i): st_point_all[i]['right_point_s'],
       'right_point_{}_t'.format(i): st_point_all[i]['right_point_t'],
    })

  for i in range(20):
    data_st_label = data_st_labels[i]
    data_st_label.data.update({
      'center_point_{}_s'.format(i): st_label_all[i]['center_point_s'],
      'center_point_{}_t'.format(i): st_label_all[i]['center_point_t'],
      'agent_{}_id'.format(i): st_label_all[i]['agent_id'],
    })

  t_search_vec = []
  s_search_vec = []

  for item in (plan_debug_info.st_graph_searcher.st_search_path):
    t_search_vec.append(item.t)
  for item in (plan_debug_info.st_graph_searcher.st_search_path):
    s_search_vec.append(item.s)

  lon_plan_data['data_st_searcher'].data.update({
    't_search': t_search_vec,
    's_search': s_search_vec,
  })

  # provide visual tools to target maker
  ## final target
  t_final_target_vec = []
  s_final_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.final_target.final_target_s_ref):
    t_final_target_vec.append(item.t)
  for item in (plan_debug_info.lon_target_s_ref.final_target.final_target_s_ref):
    s_final_target_vec.append(item.s)

  ## cruise target
  t_cruise_target_vec = []
  s_cruise_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.cruise_target.cruise_target_s_ref):
    t_cruise_target_vec.append(item.t)
  for item in (plan_debug_info.lon_target_s_ref.cruise_target.cruise_target_s_ref):
    s_cruise_target_vec.append(item.s)

  ## follow target
  t_follow_target_vec = []
  s_follow_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.follow_target.follow_target_s_ref):
    t_follow_target_vec.append(item.t)
  for item in (plan_debug_info.lon_target_s_ref.follow_target.follow_target_s_ref):
    s_follow_target_vec.append(item.s)

  lon_plan_data['data_target'].data.update({
    't_final_target': t_final_target_vec,
    's_final_target': s_final_target_vec,
    't_follow_target': t_follow_target_vec,
    's_follow_target': s_follow_target_vec,
    't_cruise_target': t_cruise_target_vec,
    's_cruise_target': s_cruise_target_vec
  })

  # behavior planning
  t_vec = list(plan_debug_info.long_ref_path.t_list)

  t_long_vec = []
  for item in (local_view_data['data_msg']['plan_msg'].trajectory.trajectory_points):
    t_long_vec.append(item.t)
  s_plan_vec =  []
  for item in (local_view_data['data_msg']['plan_msg'].trajectory.trajectory_points):
    s_plan_vec.append(item.distance)
  v_plan_vec =  []
  for item in (local_view_data['data_msg']['plan_msg'].trajectory.trajectory_points):
     v_plan_vec.append(item.v)
  s_ref_vec = []
  for item in (plan_debug_info.long_ref_path.s_refs):
    s_ref_vec.append(item.first)
  s_soft_upper_bound_vec = []
  s_soft_lower_bound_vec = []
  try:
    for idx in range(len(plan_debug_info.long_ref_path.soft_bounds)):
      high_bound = plan_debug_info.long_ref_path.soft_bounds[idx].bound[0].upper
      low_bound = plan_debug_info.long_ref_path.soft_bounds[idx].bound[0].lower
      try:
        for one_soft_bound in plan_debug_info.long_ref_path.soft_bounds[idx].bound:
           high_bound = min(high_bound, one_soft_bound.upper)
           low_bound = max(low_bound, one_soft_bound.lower)
        s_soft_upper_bound_vec.append(high_bound)
        s_soft_lower_bound_vec.append(low_bound)
      except:
        print("the s_soft_upper_bound_vec size: ",len(s_soft_upper_bound_vec))
  except:
        print("there is no long_ref_path.soft_bounds")

  obs_low_vec = []
  obs_high_vec = []
  obs_low_id_vec = []
  obs_high_id_vec = []
  obs_low_type_vec = []
  obs_high_type_vec = []
  obs_st_dict = {'-1':dict({'t':[], 's':[]})}
  for idx in range(len(plan_debug_info.long_ref_path.bounds)):
     low_bound = plan_debug_info.long_ref_path.bounds[idx].bound[0].lower
     high_bound = plan_debug_info.long_ref_path.bounds[idx].bound[0].upper
     low_bound_type = plan_debug_info.long_ref_path.bounds[idx].bound[0].bound_info.type
     high_bound_type = plan_debug_info.long_ref_path.bounds[idx].bound[0].bound_info.type
     low_bound_id = plan_debug_info.long_ref_path.bounds[idx].bound[0].bound_info.id
     high_bound_id = plan_debug_info.long_ref_path.bounds[idx].bound[0].bound_info.id
     for one_bound in plan_debug_info.long_ref_path.bounds[idx].bound:
        if one_bound.lower > low_bound:
           low_bound = one_bound.lower
           low_bound_type = one_bound.bound_info.type
           low_bound_id = one_bound.bound_info.id
        if one_bound.upper < high_bound:
           high_bound = one_bound.upper
           high_bound_type = one_bound.bound_info.type
           high_bound_id = one_bound.bound_info.id
        if one_bound.bound_info.type == 'obstacle':
           if(str(one_bound.bound_info.id) in obs_st_dict.keys()):
              obs_st_dict[str(one_bound.bound_info.id)]['t'].append(t_vec[idx])
              obs_st_dict[str(one_bound.bound_info.id)]['s'].append(one_bound.upper)
           else:
              ons_obs_st = dict({'t':[], 's':[]})
              ons_obs_st['t'].append(t_vec[idx])
              ons_obs_st['s'].append(one_bound.upper)
              obs_st_dict[str(one_bound.bound_info.id)] = ons_obs_st
     obs_low_vec.append(low_bound)
     obs_high_vec.append(high_bound)
     if low_bound_type == 'default':
        low_bound_id = -1
     if high_bound_type == 'default':
        high_bound_id = -1
     obs_low_id_vec.append(low_bound_id)
     obs_high_id_vec.append(high_bound_id)
     obs_low_type_vec.append(low_bound_type)
     obs_high_type_vec.append(high_bound_type)

  v_ref_vec = []
  for item in (plan_debug_info.long_ref_path.ds_refs):
     v_ref_vec.append(item.first)

  v_bound_low_vec = []
  try:
    for item in (plan_debug_info.long_ref_path.lon_bound_v.bound):
      v_bound_low_vec.append(item.lower)
    v_bound_high_vec = []
    for item in (plan_debug_info.long_ref_path.lon_bound_v.bound):
      v_bound_high_vec.append(item.upper)
  except:
    print("there is no lon_ref_path.lon_bound_v.bound")

  # get sv_bound:
  sv_bound_s_vec = []
  sv_bound_v_vec = []
  try:
    for item in (plan_debug_info.long_ref_path.lon_sv_boundary.sv_bounds):
      sv_bound_s_vec.append(item.s)
      sv_bound_v_vec.append(item.v_bound.upper)
  except:
    print("there is no lon_ref_path.lon_sv_boundary.sv_bounds")

  a_bound_low_vec = []
  for item in (plan_debug_info.long_ref_path.lon_bound_a.bound):
     a_bound_low_vec.append(item.lower)

  a_bound_high_vec = []
  for item in (plan_debug_info.long_ref_path.lon_bound_a.bound):
     a_bound_high_vec.append(item.upper)

  vision_lon_attr_vec = []
  for ind in range(len(planning_json_value_list)):
     vision_lon_attr_vec.append(plan_debug_json_info[planning_json_value_list[ind]])

  cutin_attr_vec = []
  for ind in range(len(new_cutin_list)):
     cutin_attr_vec.append(plan_debug_json_info[new_cutin_list[ind]])

  v_limit_vec = plan_debug_json_info['limit_v_type']
  print('v_limit_vec', v_limit_vec)
  lon_plan_data['data_st'].data.update({
    't': t_vec,
    's': s_ref_vec,
    's_soft_ub': s_soft_upper_bound_vec,
    's_soft_lb': s_soft_lower_bound_vec,
    'obs_low': obs_low_vec,
    'obs_high': obs_high_vec,
    'obs_low_id': obs_low_id_vec,
    'obs_high_id': obs_high_id_vec,
    'obs_low_type': obs_low_type_vec,
    'obs_high_type': obs_high_type_vec,
    'v_limit_type': v_limit_vec
  })

  #lon_plan_data['data_obs_st'].clear()
  #print(obs_st_dict)
  for obs_id in lon_plan_data['data_obs_st'].keys():
     """ if(obs_id not in lon_plan_data['data_obs_st'].keys()):
        one_obs_st_cds = ColumnDataSource(data = {'obs_t':[], 'obs_s':[]})
        lon_plan_data['data_obs_st'][obs_id] = one_obs_st_cds
        lon_plan_data['data_obs_st'][obs_id].data.update({
           'obs_t':obs_st_dict[obs_id]['t'],
           'obs_s':obs_st_dict[obs_id]['s']
        }) """
     if obs_id in obs_st_dict.keys():
       lon_plan_data['data_obs_st'][obs_id].data.update({
           'obs_t':obs_st_dict[obs_id]['t'],
           'obs_s':obs_st_dict[obs_id]['s'],
           'obs_high_id':[obs_id] * len(obs_st_dict[obs_id]['s']),
           'obs_high_type':['obstacle'] * len(obs_st_dict[obs_id]['s'])
        })
     else:
        lon_plan_data['data_obs_st'][obs_id].data.update({
           'obs_t':[],
           'obs_s':[],
           'obs_high_id':[],
           'obs_high_type':[]
        })

  #print(lon_plan_data['data_obs_st'].values())
  #for item in lon_plan_data['data_obs_st'].values():
  #   print(item.data)

  lon_plan_data['data_st_plan'].data.update({
    't_long': t_long_vec,
    's_plan': s_plan_vec,
    'v_plan': v_plan_vec
  })

  lon_plan_data['data_sv'].data.update({
    's_ref': s_ref_vec,
    'v_ref': v_ref_vec,
    'v_low': v_bound_low_vec,
    'v_high': v_bound_high_vec,
    # 'sv_bound_s': sv_bound_s_vec,
    # 'sv_bound_v': sv_bound_v_vec,
  })

  lon_plan_data['data_text'].data.update({
    'VisionLonAttr': planning_json_value_list,
    'VisionLonVal': vision_lon_attr_vec
  })

  lon_plan_data['data_cutin'].data.update({
    'cutinAttr': new_cutin_list,
    'cutinVal': cutin_attr_vec
  })

  # motion planning
  lon_motion_plan_input = plan_debug_info.longitudinal_motion_planning_input
  lon_motion_plan_output = plan_debug_info.longitudinal_motion_planning_output

  init_state = lon_motion_plan_input.init_state
  ref_pos_vec = lon_motion_plan_input.ref_pos_vec
  ref_vel_vec = lon_motion_plan_input.ref_vel_vec
  soft_pos_max_vec = lon_motion_plan_input.soft_pos_max_vec
  soft_pos_min_vec = lon_motion_plan_input.soft_pos_min_vec
  vel_max_vec = lon_motion_plan_input.vel_max_vec
  vel_min_vec = lon_motion_plan_input.vel_min_vec
  acc_max_vec = lon_motion_plan_input.acc_max_vec
  acc_min_vec = lon_motion_plan_input.acc_min_vec
  jerk_max_vec = lon_motion_plan_input.jerk_max_vec
  jerk_min_vec = lon_motion_plan_input.jerk_min_vec
  s_stop = lon_motion_plan_input.s_stop

  # time_vec = []
  # for i in range(len(ref_pos_vec)):
  #   time_vec.append(i * 0.2)

  time_vec = lon_motion_plan_output.time_vec
  pos_vec = lon_motion_plan_output.pos_vec
  vel_vec = lon_motion_plan_output.vel_vec
  acc_vec = lon_motion_plan_output.acc_vec
  jerk_vec = lon_motion_plan_output.jerk_vec

#   print("lon_motion_plan_output:=", lon_motion_plan_output)

  lon_plan_data['data_lon_motion_plan'].data.update({
    'time_vec': time_vec,
    'ref_pos_vec_origin': s_ref_vec,
    'ref_pos_vec': ref_pos_vec,
    'ref_vel_vec': ref_vel_vec,
    'soft_pos_max_vec': soft_pos_max_vec,
    'soft_pos_min_vec': soft_pos_min_vec,
    'vel_max_vec': vel_max_vec,
    'vel_min_vec': vel_min_vec,
    'acc_max_vec': acc_max_vec,
    'acc_min_vec': acc_min_vec,
    'jerk_max_vec': jerk_max_vec,
    'jerk_min_vec': jerk_min_vec,
    'pos_vec': pos_vec,
    'vel_vec': vel_vec,
    'acc_vec': acc_vec,
    'jerk_vec': jerk_vec,
  })

  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = local_view_data['data_msg']['loc_msg'].position.position_boot.x
    cur_pos_yn = local_view_data['data_msg']['loc_msg'].position.position_boot.y
    cur_yaw = local_view_data['data_msg']['loc_msg'].orientation.euler_boot.yaw
    planning_json = local_view_data['data_msg']['plan_debug_json_msg']

    print("dbw_status = ", planning_json['dbw_status'])
    print("replan_status = ", planning_json['replan_status'])
    print("lat_err = ", planning_json['lat_err'])
    print("theta_err = ", planning_json['theta_err'])
    print("lon_err = ", planning_json['lon_err'])
    print("dist_err = ", planning_json['dist_err'])

    try:
      json_pos_x = planning_json['ego_pos_x']
      json_pos_y = planning_json['ego_pos_y']
      json_yaw = planning_json['ego_pos_yaw']
      coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
    except:
      coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

   #  coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

  if bag_loader.plan_msg['enable'] == True:
    trajectory = local_view_data['data_msg']['plan_msg'].trajectory
    try:
      planning_polynomial = trajectory.target_reference.polynomial
      plan_traj_x, plan_traj_y = gen_line(planning_polynomial[3],planning_polynomial[2], planning_polynomial[1], planning_polynomial[0], 0, 50)

    except:
      plan_x = []
      plan_y = []
      for i in range(len(trajectory.trajectory_points)):
        plan_x.append(trajectory.trajectory_points[i].x)
        plan_y.append(trajectory.trajectory_points[i].y)

      # plan_traj_x, plan_traj_y = coord_tf.global_to_local(plan_x, plan_y)
      plan_traj_x, plan_traj_y = coord_tf.global_to_local(planning_json['traj_x_vec'], planning_json['traj_y_vec'])

    lon_plan_data['data_planning'].data.update({
      'plan_traj_y' : plan_traj_y,
      'plan_traj_x' : plan_traj_x,
      })

def update_lon_ref_path(lon_ref_path, lon_plan_data):
  # behavior planning
  t_vec = list(lon_ref_path.t_list)

  s_ref_vec = []
  for item in (lon_ref_path.s_refs):
    s_ref_vec.append(item.first)
  s_soft_upper_bound_vec = []
  s_soft_lower_bound_vec = []
  try:
    for idx in range(len(lon_ref_path.soft_bounds)):
      high_bound = lon_ref_path.soft_bounds[idx].bound[0].upper
      low_bound = lon_ref_path.soft_bounds[idx].bound[0].lower
      try:
        for one_soft_bound in lon_ref_path.soft_bounds[idx].bound:
           high_bound = min(high_bound, one_soft_bound.upper)
           low_bound = max(low_bound, one_soft_bound.lower)
        s_soft_upper_bound_vec.append(high_bound)
        s_soft_lower_bound_vec.append(low_bound)
      except:
        print("the s_soft_upper_bound_vec size: ",len(s_soft_upper_bound_vec))
  except:
        print("there is no lon_ref_path.soft_bounds")

  obs_low_vec = []
  obs_high_vec = []
  obs_low_id_vec = []
  obs_high_id_vec = []
  obs_low_type_vec = []
  obs_high_type_vec = []
  obs_st_dict = {'-1':dict({'t':[], 's':[]})}
  for idx in range(len(lon_ref_path.bounds)):
     low_bound = lon_ref_path.bounds[idx].bound[0].lower
     high_bound = lon_ref_path.bounds[idx].bound[0].upper
     low_bound_type = lon_ref_path.bounds[idx].bound[0].bound_info.type
     high_bound_type = lon_ref_path.bounds[idx].bound[0].bound_info.type
     low_bound_id = lon_ref_path.bounds[idx].bound[0].bound_info.id
     high_bound_id = lon_ref_path.bounds[idx].bound[0].bound_info.id
     for one_bound in lon_ref_path.bounds[idx].bound:
        if one_bound.lower > low_bound:
           low_bound = one_bound.lower
           low_bound_type = one_bound.bound_info.type
           low_bound_id = one_bound.bound_info.id
        if one_bound.upper < high_bound:
           high_bound = one_bound.upper
           high_bound_type = one_bound.bound_info.type
           high_bound_id = one_bound.bound_info.id
        if one_bound.bound_info.type == 'obstacle':
           if(str(one_bound.bound_info.id) in obs_st_dict.keys()):
              obs_st_dict[str(one_bound.bound_info.id)]['t'].append(t_vec[idx])
              obs_st_dict[str(one_bound.bound_info.id)]['s'].append(one_bound.upper)
           else:
              ons_obs_st = dict({'t':[], 's':[]})
              ons_obs_st['t'].append(t_vec[idx])
              ons_obs_st['s'].append(one_bound.upper)
              obs_st_dict[str(one_bound.bound_info.id)] = ons_obs_st
     obs_low_vec.append(low_bound)
     obs_high_vec.append(high_bound)
     if low_bound_type == 'default':
        low_bound_id = -1
     if high_bound_type == 'default':
        high_bound_id = -1
     obs_low_id_vec.append(low_bound_id)
     obs_high_id_vec.append(high_bound_id)
     obs_low_type_vec.append(low_bound_type)
     obs_high_type_vec.append(high_bound_type)

  v_ref_vec = []
  for item in (lon_ref_path.ds_refs):
     v_ref_vec.append(item.first)

  v_bound_low_vec = []
  for item in (lon_ref_path.lon_bound_v.bound):
     v_bound_low_vec.append(item.lower)

  v_bound_high_vec = []
  for item in (lon_ref_path.lon_bound_v.bound):
     v_bound_high_vec.append(item.upper)

  # get sv_bound:
  sv_bound_s_vec = []
  sv_bound_v_vec = []
  for item in (lon_ref_path.lon_sv_boundary.sv_bounds):
    sv_bound_s_vec.append(item.s)
    sv_bound_v_vec.append(item.v_bound.upper)

  a_bound_low_vec = []
  for item in (lon_ref_path.lon_bound_a.bound):
     a_bound_low_vec.append(item.lower)

  a_bound_high_vec = []
  for item in (lon_ref_path.lon_bound_a.bound):
     a_bound_high_vec.append(item.upper)

  lon_plan_data['data_st'].data.update({
    't': t_vec,
    's': s_ref_vec,
    's_soft_ub': s_soft_upper_bound_vec,
    's_soft_lb': s_soft_lower_bound_vec,
    'obs_low': obs_low_vec,
    'obs_high': obs_high_vec,
    'obs_low_id': obs_low_id_vec,
    'obs_high_id': obs_high_id_vec,
    'obs_low_type': obs_low_type_vec,
    'obs_high_type': obs_high_type_vec
  })

  #lon_plan_data['data_obs_st'].clear()
  #print(obs_st_dict)
  for obs_id in lon_plan_data['data_obs_st'].keys():
     """ if(obs_id not in lon_plan_data['data_obs_st'].keys()):
        one_obs_st_cds = ColumnDataSource(data = {'obs_t':[], 'obs_s':[]})
        lon_plan_data['data_obs_st'][obs_id] = one_obs_st_cds
        lon_plan_data['data_obs_st'][obs_id].data.update({
           'obs_t':obs_st_dict[obs_id]['t'],
           'obs_s':obs_st_dict[obs_id]['s']
        }) """
     if obs_id in obs_st_dict.keys():
       lon_plan_data['data_obs_st'][obs_id].data.update({
           'obs_t':obs_st_dict[obs_id]['t'],
           'obs_s':obs_st_dict[obs_id]['s'],
           'obs_high_id':[obs_id] * len(obs_st_dict[obs_id]['s']),
           'obs_high_type':['obstacle'] * len(obs_st_dict[obs_id]['s'])
        })
     else:
        lon_plan_data['data_obs_st'][obs_id].data.update({
           'obs_t':[],
           'obs_s':[],
           'obs_high_id':[],
           'obs_high_type':[]
        })

  #print(lon_plan_data['data_obs_st'].values())
  #for item in lon_plan_data['data_obs_st'].values():
  #   print(item.data)

  lon_plan_data['data_sv'].data.update({
    's_ref': s_ref_vec,
    'v_ref': v_ref_vec,
    'v_low': v_bound_low_vec,
    'v_high': v_bound_high_vec,
    # 'sv_bound_s': sv_bound_s_vec,
    # 'sv_bound_v': sv_bound_v_vec,
  })

def load_lon_global_figure(bag_loader):
  #real time global figure data process
  velocity_fig = bkp.figure(title='车速',x_axis_label='time/s',
                y_axis_label='velocity/(m/s)',width=600,height=300)

  ego_velocity_vec = []
  target_velocity_vec = []
  ref_velocity_vec = []
  leadone_velocity_vec = []
  leadtwo_velocity_vec = []
  t_plan_vec = bag_loader.plan_debug_msg['t']
  t_loc_vec = bag_loader.loc_msg['t']
  t_vehicle_service_vec = bag_loader.vs_msg['t']
  for ind in range(len(bag_loader.plan_debug_msg['json'])):
    target_velocity_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['v_target'], 2))
    # ref_velocity_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['RealTime_v_ref'], 2))
    leadone_velocity_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lead_one_vel'], 2))
    leadtwo_velocity_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lead_two_vel'], 2))

  for ind in range(len(bag_loader.vs_msg['data'])):
    ego_velocity_vec.append(round(bag_loader.vs_msg['data'][ind].vehicle_speed, 2))

  velocity_fig.line(t_plan_vec, target_velocity_vec, line_width=1,
                              legend_label='target_velocity', color="green")
  # velocity_fig.line(t_plan_vec, ref_velocity_vec, line_width=1,
  #                             legend_label='ref_velocity', color="gray")
  velocity_fig.line(t_vehicle_service_vec, ego_velocity_vec, line_width=1,
                                legend_label='ego_velocity',color="blue")
  velocity_fig.line(t_plan_vec, leadone_velocity_vec, line_width=1,
                              legend_label='leadone_velocity', color="red")
  velocity_fig.line(t_plan_vec, leadtwo_velocity_vec, line_width=1,
                                legend_label='leadtwo_velocity',color="orange")
  velocity_fig.legend.click_policy = 'hide'


  acc_fig = bkp.figure(title='加速度',x_axis_label='time/s',
                y_axis_label='acc/(m/s2)',width=600,height=300)

  ego_acc_vec = []
  acc_min_vec = []
  acc_max_vec = []
  lead_one_acc = []

  t_vs_vec = bag_loader.vs_msg['t']
  for ind in range(len(bag_loader.plan_debug_msg['json'])):
    acc_min_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['acc_target_low'], 2))
    acc_max_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['acc_target_high'], 2))
    lead_one_acc.append(round(bag_loader.plan_debug_msg['json'][ind]['acc_cipv'], 2))
  for ind in range(len(bag_loader.vs_msg['data'])):
    ego_acc_vec.append(round(bag_loader.vs_msg['data'][ind].long_acceleration, 2))

  acc_fig.line(t_plan_vec, acc_min_vec, line_width=1,
                              legend_label='acc_min', color="brown")
  acc_fig.line(t_vs_vec, ego_acc_vec, line_width=1,
                                legend_label='ego_acc',color="blue")
  acc_fig.line(t_plan_vec, acc_max_vec, line_width=1,
                              legend_label='acc_max', color="red")
  acc_fig.line(t_plan_vec, lead_one_acc, line_width=2,
                              legend_label='lead_one_acc', color="green")
  acc_fig.legend.click_policy = 'hide'

  lead_fig = bkp.figure(title='lead_car_distance',x_axis_label='time/s',
                y_axis_label='distance/(m)',width=600,height=300)
  # 各阶段耗时
  cost_time_fig = bkp.figure(title='耗时',x_axis_label='time/s',
                  y_axis_label='time cost/(ms)',width=600,height=300)

  lead_one_dis_vec = []
  lead_two_dis_vec = []
  temp_lead_one_dis_vec = []
  temp_lead_two_dis_vec = []
  desired_distance_rss_vec = []
  desired_distance_calibrate_vec = []
  soft_brake_distance_lead_vec = []
  SccLonBehaviorCostTime_vec = []
  SccLonMotionCostTime_vec = []
  SccLateralMotionCostTime_vec = []
  SccLateralBehaviorCostTime_vec = []
  EnvironmentalModelManagerCost_vec = []
  GeneralPlannerModuleCostTime_vec = []
  DynamicWorldAverageCostTime_vec = []

  for ind in range(len(bag_loader.plan_debug_msg['json'])):
    lead_one_dis_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lead_one_dis'], 2))
    lead_two_dis_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lead_two_dis'], 2))
    temp_lead_one_dis_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['temp_lead_one_dis'], 2))
    temp_lead_two_dis_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['temp_lead_two_dis'], 2))
    desired_distance_rss_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['RealTime_desired_distance_rss'], 2))
    desired_distance_calibrate_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['RealTime_desired_distance_calibrate'], 2))
    soft_brake_distance_lead_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['soft_brake_distance_lead'], 2))
    SccLonBehaviorCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['SccLonBehaviorCostTime'], 2))
    SccLonMotionCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['SccLonMotionCostTime'], 2))
    SccLateralMotionCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['LateralMotionCostTime'], 2))
    SccLateralBehaviorCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['RealTimeLateralBehaviorCostTime'], 2))
    EnvironmentalModelManagerCost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['EnvironmentalModelManagerCost'], 2))
    GeneralPlannerModuleCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['GeneralPlannerModuleCostTime'], 2))
    DynamicWorldAverageCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['dynamic_world_cost'], 2))

  lead_fig.line(t_plan_vec, lead_one_dis_vec, line_width=1, legend_label='lead_one_dis', color="red")
  lead_fig.line(t_plan_vec, lead_two_dis_vec, line_width=1, legend_label='lead_two_dis', color="green")
  lead_fig.line(t_plan_vec, temp_lead_one_dis_vec, line_width=1, legend_label='temp_lead_one_dis', color="blue")
  lead_fig.line(t_plan_vec, temp_lead_two_dis_vec, line_width=1, legend_label='temp_lead_two_dis', color="purple")
  lead_fig.line(t_plan_vec, desired_distance_rss_vec, line_width=1, legend_label='distance_rss', color="yellow")
  lead_fig.line(t_plan_vec, desired_distance_calibrate_vec, line_width=1, legend_label='distance_cali', color="orange")
  lead_fig.line(t_plan_vec, soft_brake_distance_lead_vec, line_width=2, legend_label='soft_brake_distance', color="black")
  lead_fig.legend.click_policy = 'hide'

  cost_time_fig.line(t_plan_vec, SccLonBehaviorCostTime_vec, line_width=1, legend_label='LonBehaviorCostTime', color="red")
  cost_time_fig.line(t_plan_vec, SccLonMotionCostTime_vec, line_width=1, legend_label='LonMotionCostTime_vec', color="blue")
  cost_time_fig.line(t_plan_vec, SccLateralMotionCostTime_vec, line_width=1, legend_label='LatMotionCostTime_vec', color="orange")
  cost_time_fig.line(t_plan_vec, EnvironmentalModelManagerCost_vec, line_width=1, legend_label='EnvironmentalCostTime_vec', color="yellow")
  cost_time_fig.line(t_plan_vec, GeneralPlannerModuleCostTime_vec, line_width=1, legend_label='GeneralPlannerModuleCostTime', color="purple")
  cost_time_fig.legend.click_policy = 'hide'

  cutin_fig = bkp.figure(title='速度',x_axis_label='time/s', y_axis_label='velocity/(m/s)',width=600,height=300)

  limit_cutin_vel_vec = []
  potential_cutin_speed_vec = []

  for ind in range(len(bag_loader.plan_debug_msg['json'])):
    limit_cutin_vel_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['v_target_cutin'], 2))
    potential_cutin_speed_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['v_target_potential_cutin'], 2))

  cutin_fig.line(t_plan_vec, limit_cutin_vel_vec, line_width=1,
                                  legend_label='Pre-deceleration cutin',color="blue")
  cutin_fig.line(t_plan_vec, potential_cutin_speed_vec, line_width=1,
                                legend_label='Speed regulation cutin', color="red")
  cutin_fig.legend.click_policy = 'hide'
  # cutin_fig.line(t_plan_vec, cutin_status_vec, line_width=1,
  #                               legend_label='cutin_status', color="black")

  LonBahaviorAverageCostTime = sum(SccLonBehaviorCostTime_vec) / len(t_plan_vec)
  LonMotionAverageCostTime = sum(SccLonMotionCostTime_vec) / len(t_plan_vec)
  LateralBahaviorAverageCostTime = sum(SccLateralBehaviorCostTime_vec) / len(t_plan_vec)
  LateralMotionAverageCostTime = sum(SccLateralMotionCostTime_vec) / len(t_plan_vec)
  EnvironmentalAverageCostTime = sum(EnvironmentalModelManagerCost_vec) / len(t_plan_vec)
  GeneralPlannerAverageCostTime = sum(GeneralPlannerModuleCostTime_vec) / len(t_plan_vec)
  DynamicWorldAverageCostTime = sum(DynamicWorldAverageCostTime_vec) / len(t_plan_vec)
  print('lat_bahavior_average_cost', LateralBahaviorAverageCostTime)
  print('lat_motion_average_cost', LateralMotionAverageCostTime)
  print('lon_bahavior_average_cost', LonBahaviorAverageCostTime)
  print('lon_motion_average_cost', LonMotionAverageCostTime)
  print('Environmental_average_cost', EnvironmentalAverageCostTime)
  print('GeneralPlanner_average_cost', GeneralPlannerAverageCostTime)
  print('dynamic_world_average_cost', DynamicWorldAverageCostTime)

  #get longtime obstacle id list in st-graph
  obs_st_ids = []
  for ind in range(len(bag_loader.plan_debug_msg['data'])):
    for item in bag_loader.plan_debug_msg['data'][ind].long_ref_path.bounds:
      for one_bound in item.bound:
          if(one_bound.bound_info.type == 'obstacle' and one_bound.bound_info.id > 0 and one_bound.bound_info.id not in obs_st_ids):
            obs_st_ids.append(one_bound.bound_info.id)

  # multipule states info plot
  fsm_state_command = ColumnDataSource(data ={
  'time': [],
  'fsm_cur_state':[],
  })

  plan_debug_multi = ColumnDataSource(data ={
  'time': [],
  'replan_status':[],
  })

  t_soc_state = []
  fsm_cur_state = []

  soc_state_info = bag_loader.soc_state_msg['data']
  soc_state_t = bag_loader.soc_state_msg['t']
  for i in range(len(soc_state_info)):
     t_soc_state.append(soc_state_t[i])
     fsm_cur_state.append(soc_state_info[i].current_state)
  if len(t_soc_state) > 0:
    x_range = [t_soc_state[0], t_soc_state[-1]]
    x_value = t_soc_state[10]
  else:
    x_range = [0, 1]
    x_value = 0

  fsm_state_command.data.update({
    'time': t_soc_state,
    'fsm_cur_state': fsm_cur_state,
  })

  replan_status = []
  lon_err = []
  t_plan_debug = []

  plan_debug_info = bag_loader.plan_debug_msg['json']
  for i in range(len(plan_debug_info)):
     t_plan_debug.append(bag_loader.plan_debug_msg['t'][i])
     replan_status.append(plan_debug_info[i]['replan_status'])
     lon_err.append(plan_debug_info[i]['lon_err'])

  plan_debug_multi.data.update({
    'time': t_plan_debug,
    'replan_status': replan_status,
    'lon_err': lon_err,
  })


  fig_fsm_state = bkp.figure(x_axis_label='time', y_axis_label='fsm state',x_range = x_range, width=600, height=300)
  f_fsm_state = fig_fsm_state.line('time', 'fsm_cur_state', source = fsm_state_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'fsm_cur_state')
  fig_fsm_state.text(x=x_value, y=12, text=['ACC_ACTIVE:5'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=8, text=['SCC_STANDBY:7'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=10, text=['SCC_ACTIVE:8'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=2, text=['SCC_OVERRIDE:9'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=6, text=['NOA_STANDBY:10'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=4, text=['NOA_ACTIVE:11'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=0, text=['NOA_OVERRIDE:12'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')

  fig_replan_status = bkp.figure(x_axis_label='time', y_axis_label='plan debug multi',x_range = [t_plan_debug[0], t_plan_debug[-1]], width=600, height=300)
  f_replan_status = fig_replan_status.line('time', 'replan_status', source = plan_debug_multi, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'replan_status')
  fig_replan_status.line('time', 'lon_err', source = plan_debug_multi, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'lon_station_err')
  # fig_replan_status.line('time', 'location_latency', source = plan_debug_multi, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'location_latency')
  # 在fig_replan_status画的图中添加相关标签注释：图的左侧竖直排列添加如下所有文字注释
  # LAT_POSITION_REPLAN:1,LAT_ANGLE_REPLAN:2,lON_POSITION_REPLAN:4,LON_TINY_SPEED_REPLAN:8,FUNCTION_REQUEST_REPLAN:16,LAT_LON_REST:32
  fig_replan_status.text(x=t_plan_debug[10], y=12, text=['LAT_POSITION_REPLAN:1'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_replan_status.text(x=t_plan_debug[10], y=10, text=['LAT_ANGLE_REPLAN:2'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_replan_status.text(x=t_plan_debug[10], y=8, text=['LON_POSITION_REPLAN:4'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_replan_status.text(x=t_plan_debug[10], y=6, text=['LON_TINY_SPEED_REPLAN:8'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_replan_status.text(x=t_plan_debug[10], y=4, text=['LAT_LON_REPLAN:16'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_replan_status.text(x=t_plan_debug[10], y=2, text=['LAT_REPLAN:32'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_replan_status.text(x=t_plan_debug[10], y=0, text=['LAT_LON_REST:64'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')

  hover_fsm_state = HoverTool(renderers=[f_fsm_state], tooltips=[('time', '@time'), ('fsm_cur_state', '@fsm_cur_state')], mode='vline')
  fig_fsm_state.add_tools(hover_fsm_state)
  fig_fsm_state.toolbar.active_scroll = fig_fsm_state.select_one(WheelZoomTool)
  fig_fsm_state.legend.click_policy = 'hide'

  hover_replan_status = HoverTool(renderers=[f_replan_status], tooltips=[('time', '@time'), ('replan_status', '@replan_status'), ('lon_station_err', '@lon_station_err')], mode='vline')
  fig_replan_status.add_tools(hover_replan_status)
  fig_replan_status.toolbar.active_scroll = fig_replan_status.select_one(WheelZoomTool)
  fig_replan_status.legend.click_policy = 'hide'


  topic_latency_fig = bkp.figure(title='各topic延时',x_axis_label='time/s',
                y_axis_label='time/(ms)',width=600,height=300)

  t_plan_vec = bag_loader.plan_debug_msg['t']

  fusion_object_latency=[]
  fusion_road_latency=[]
  vehicle_service_latency=[]
  control_output_latency=[]
  hmi_latency=[]
  function_state_machine_latency=[]
  localization_latency=[]
  localization_estimate_latency=[]

  for ind in range(len(bag_loader.plan_debug_msg['data'])):
    fusion_object_latency.append(round(bag_loader.plan_debug_msg['data'][ind].input_topic_latency.fusion_object, 2))
    fusion_road_latency.append(round(bag_loader.plan_debug_msg['data'][ind].input_topic_latency.fusion_road, 2))
    vehicle_service_latency.append(round(bag_loader.plan_debug_msg['data'][ind].input_topic_latency.vehicle_service, 2))
    control_output_latency.append(round(bag_loader.plan_debug_msg['data'][ind].input_topic_latency.control_output, 2))
    hmi_latency.append(round(bag_loader.plan_debug_msg['data'][ind].input_topic_latency.hmi, 2))
    function_state_machine_latency.append(round(bag_loader.plan_debug_msg['data'][ind].input_topic_latency.function_state_machine, 2))
    localization_latency.append(round(bag_loader.plan_debug_msg['data'][ind].input_topic_latency.localization, 2))
    localization_estimate_latency.append(round(bag_loader.plan_debug_msg['data'][ind].input_topic_latency.localization_estimate, 2))

  topic_latency_fig.line(t_plan_vec, fusion_object_latency, line_width=1,
                              legend_label='fusion_object', color="green")
  topic_latency_fig.line(t_plan_vec, fusion_road_latency, line_width=1,
                                legend_label='fusion_road',color="blue")
  topic_latency_fig.line(t_plan_vec, vehicle_service_latency, line_width=1,
                             legend_label='vehicle_service', color="red")
  topic_latency_fig.line(t_plan_vec, control_output_latency, line_width=1,
                               legend_label='control_output',color="purple")
  topic_latency_fig.line(t_plan_vec, hmi_latency, line_width=1,
                               legend_label='hmi_latency',color="brown")
  topic_latency_fig.line(t_plan_vec, function_state_machine_latency, line_width=1,
                               legend_label='function_state_machine',color="yellow")
  topic_latency_fig.line(t_plan_vec, localization_latency, line_width=1,
                               legend_label='localization',color="orange")
  topic_latency_fig.line(t_plan_vec, localization_estimate_latency, line_width=1,
                               legend_label='localization_estimate',color="black")
  topic_latency_fig.legend.click_policy = 'hide'


  return velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status,topic_latency_fig

def load_lon_plan_figure(fig1, velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status,topic_latency_fig):
  data_st = ColumnDataSource(data = {'t':[], 's':[], 's_soft_ub':[], 's_soft_lb':[], 'obs_low':[], 'obs_high':[], 'obs_low_id':[], 'obs_high_id':[], 'obs_low_type':[], 'obs_high_type':[]})
  data_st_plan = ColumnDataSource(data = {'t_long':[], 's_plan':[], 'v_plan':[]})
  data_sv = ColumnDataSource(data = {'s_ref':[], 'v_ref':[], 'v_low':[], 'v_high':[]}) # , 'sv_bound_s':[], 'sv_bound_v':[]
  data_tv = ColumnDataSource(data = {'t':[], 'vel':[]})
  data_ta = ColumnDataSource(data = {'t':[], 'acc':[]})
  data_tj = ColumnDataSource(data = {'t':[], 'jerk':[]})
  data_text = ColumnDataSource(data = {'VisionLonAttr':[], 'VisionLonVal':[]})
  data_cutin = ColumnDataSource(data = {'cutinAttr':[], 'cutinVal':[]})
  data_st_searcher = ColumnDataSource(data = {'t_search':[], 's_search':[]})
  data_target = ColumnDataSource(data = {'t_final_target':[], 's_final_target':[], 't_cruise_target':[], 's_cruise_target':[], 't_follow_target':[], 's_follow_target':[]})
  #obstacles st data, key is id, value is time and s list
  data_obs_st = {}
  for it in obs_st_ids:
     one_cds = ColumnDataSource(data = {'obs_t':[], 'obs_s':[], 'obs_high_id':[], 'obs_high_type':[]})
     data_obs_st[str(it)] = one_cds

  data_lon_motion_plan = ColumnDataSource(data = {'time_vec': [],
                                                  'ref_pos_vec_origin': [],
                                                  'ref_pos_vec':[],
                                                  'ref_vel_vec':[],
                                                  'soft_pos_max_vec':[],
                                                  'soft_pos_min_vec':[],
                                                  'vel_max_vec':[],
                                                  'vel_min_vec':[],
                                                  'acc_max_vec':[],
                                                  'acc_min_vec':[],
                                                  'jerk_max_vec':[],
                                                  'jerk_min_vec':[],
                                                  'pos_vec':[],
                                                  'vel_vec':[],
                                                  'acc_vec':[],
                                                  'jerk_vec':[],
                                                        })

  data_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                    'plan_traj_x':[],})

  lon_plan_data = {'data_st':data_st, \
                   'data_st_plan':data_st_plan, \
                   'data_text':data_text, \
                   'data_cutin':data_cutin, \
                   'data_sv':data_sv, \
                   'data_tv':data_tv, \
                   'data_ta':data_ta, \
                   'data_tj':data_tj, \
                   'data_obs_st':data_obs_st, \
                   'data_lon_motion_plan': data_lon_motion_plan, \
                   'data_planning':data_planning, \
                   'data_st_searcher':data_st_searcher, \
                   'data_target': data_target, \
  }

  for i in range(20):
    data1 = {
      f'agent_{i}_id_vec': [],
      f'lower_points_{i}_s_vec': [],
      f'lower_points_{i}_t_vec': [],
      f'upper_points_{i}_s_vec': [],
      f'upper_points_{i}_t_vec': []
    }
    data2 = {
      f'left_point_{i}_s': [],
      f'left_point_{i}_t': [],
      f'right_point_{i}_s': [],
      f'right_point_{i}_t': []
    }
    data3 = {
      f'center_point_{i}_s': [],
      f'center_point_{i}_t': [],
      f'agent_{i}_id': [],
    }
    lon_plan_data[f'st_boundary_{i}'] = ColumnDataSource(data=data1)
    lon_plan_data[f'st_point_{i}'] = ColumnDataSource(data=data2)
    lon_plan_data[f'st_label_{i}'] = ColumnDataSource(data=data3)

  boundaries = [lon_plan_data[f'st_boundary_{i}'] for i in range(0, 20)]
  st_points = [lon_plan_data[f'st_point_{i}'] for i in range(0, 20)]
  st_labels = [lon_plan_data[f'st_label_{i}'] for i in range(0, 20)]

  columns = [
        TableColumn(field="VisionLonAttr", title="VisionLonAttr"),
        TableColumn(field="VisionLonVal", title="VisionLonVal"),
    ]
  cutin_colums = [
      TableColumn(field="cutinAttr", title="cutinAttr"),
      TableColumn(field="cutinVal", title="cutinVal")
  ]
  hover = HoverTool(tooltips = [
     ('index','$index'),
     ('id_low','@obs_low_id'),
     ('id_high','@obs_high_id'),
     ('low_type','@obs_low_type'),
     ('high_type','@obs_high_type'),
     ('v_limit_type','@v_limit_type')
  ])
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan debug', visible=False)

  # fig2 S-T
  fig2 = bkp.figure(x_axis_label='t', y_axis_label='s', x_range = [-0.1, 7.0], width=600, height=400, tools=[hover,'pan,wheel_zoom,box_zoom,reset'], match_aspect = True, aspect_scale=1)
  # fig3 S-V
  # fig3 = bkp.figure(x_axis_label='s', y_axis_label='v', width=600, height=400, match_aspect = True, aspect_scale=1)
  fig3 = bkp.figure(x_axis_label='t', y_axis_label='s', x_range = [-0.1, 7.0], width=600, height=400, match_aspect = True, aspect_scale=1)
  # fig4 s-t
  fig4 = bkp.figure(x_axis_label='time', y_axis_label='pos',x_range = [-0.1, 6.5], width=600, height=200)
  # fig5 v-t
  fig5 = bkp.figure(x_axis_label='time', y_axis_label='vel',x_range = fig4.x_range, width=600, height=200)
  # fig6 a-t
  fig6 = bkp.figure(x_axis_label='time', y_axis_label='acc',x_range = fig5.x_range, width=600, height=200)
  # fig7 j-t
  fig7 = bkp.figure(x_axis_label='time', y_axis_label='jerk',x_range = fig6.x_range, width=600, height=200)

  f2 = fig2.line('t', 's', source = data_st, line_width = 2, line_color = 'green', line_dash = 'dashed', legend_label = 'origin s_ref')
  fig2.line('t', 's_soft_ub', source = data_st, line_width = 3, line_color = 'yellow', line_dash = 'solid', legend_label = 's_soft_ub')
  fig2.line('t', 's_soft_lb', source = data_st, line_width = 3, line_color = '#FFA500', line_dash = 'solid', legend_label = 's_soft_lb')
  fig2.line('time_vec', 'ref_pos_vec', source = data_lon_motion_plan, line_width = 2.5, line_color = 'red', line_dash = 'dashed', legend_label = 's_ref')
  #fig2.line('t_long', 's_plan', source = data_st_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 's_plan')
  fig2.line('time_vec', 'pos_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 's_plan')
  fig2.line('t', 'obs_low', source = data_st, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'obs_lb')
  fig2.line('t', 'obs_high', source = data_st, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'obs_ub')
  fig2.triangle('t', 'obs_low', source = data_st, size = 10, fill_color='grey', line_color='grey', alpha = 0.7, legend_label = 'obs_lb_point')
  fig2.inverted_triangle ('t', 'obs_high', source = data_st, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'obs_ub_point')

  # 添加障碍物s-t bound
  for obs_id in data_obs_st.keys():
     if(obs_id != ''):
        fig2.line('obs_t', 'obs_s', source = data_obs_st[obs_id], line_width = 2, line_color = 'firebrick', line_dash = 'solid')

  #label_low_id = LabelSet(x='t', y='obs_low', text='obs_low_id', x_offset=2, y_offset=2, source=data_st)
  #fig2.add_layout(label_low_id)
  #label_high_id = LabelSet(x='t', y='obs_high', text='obs_high_id', x_offset=2, y_offset=2, source=data_st)
  #fig2.add_layout(label_high_id)

  # f3 = fig3.line('s_ref', 'v_ref', source = data_sv, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'v_ref')
  # # fig3.line('sv_bound_s', 'sv_bound_v', source = data_sv, line_width = 2, line_color = 'purple', line_dash = 'solid', legend_label = 'sv_bound_v_upper')
  # fig3.line('pos_vec', 'vel_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'v_plan')
  # fig3.line('s_ref', 'v_low', source = data_sv, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'v_lb')
  # fig3.line('s_ref', 'v_high', source = data_sv, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'v_ub')
  # fig3.triangle('s_ref', 'v_low', source = data_sv, size = 10, fill_color='grey', line_color='grey', alpha = 0.7, legend_label = 'obs_lb_point')
  # fig3.inverted_triangle ('s_ref', 'v_high', source = data_sv, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'obs_ub_point')
  for i, source in enumerate(boundaries):
    lower_t = f'lower_points_{i}_t_vec'
    lower_s = f'lower_points_{i}_s_vec'
    upper_t = f'upper_points_{i}_t_vec'
    upper_s = f'upper_points_{i}_s_vec'

    fig3.line(lower_t, lower_s, source=source, line_width=2, line_color='red', line_dash='solid', legend_label='st_boundary')
    fig3.line(upper_t, upper_s, source=source, line_width=2, line_color='red', line_dash='solid', legend_label='st_boundary')

  for i, source in enumerate(st_points):
    left_t = f'left_point_{i}_t'
    left_s = f'left_point_{i}_s'
    right_t = f'right_point_{i}_t'
    right_s = f'right_point_{i}_s'

    fig3.line(left_t, left_s, source=source, line_width=2, line_color='red', line_dash='solid', legend_label='st_boundary')
    fig3.line(right_t, right_s, source=source, line_width=2, line_color='red', line_dash='solid', legend_label='st_boundary')

  for i, source in enumerate(st_labels):
    center_point_s = f'center_point_{i}_s'
    center_point_t = f'center_point_{i}_t'
    agent_id = f'agent_{i}_id'

    fig3.text(center_point_t, center_point_s, text = agent_id ,source = source, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'st_boundary')

  fig3.line('t_search', 's_search', source = data_st_searcher, line_width = 3.0, line_color = 'green', line_dash = 'solid', legend_label = 's_search_path')
  fig3.line('t_final_target', 's_final_target', source = data_target, line_width = 3.0, line_color = 'blue', line_dash = 'solid', legend_label = 's_final_target')
  fig3.circle('t_final_target', 's_final_target', source=data_target, size=5, color='brown', legend_label='s_final_target')
  fig3.line('t_follow_target', 's_follow_target', source = data_target, line_width = 3.0, line_color = 'red', line_dash = 'solid', legend_label = 's_follow_target')
  fig3.line('t_cruise_target', 's_cruise_target', source = data_target, line_width = 3.0, line_color = 'grey', line_dash = 'solid', legend_label = 's_cruise_target')

  # pos
  f4 = fig4.line('time_vec', 'ref_pos_vec', source = data_lon_motion_plan, line_width = 2.5, line_color = 'red', line_dash = 'dashed', legend_label = 's_ref')
  fig4.line('time_vec', 'ref_pos_vec_origin', source = data_lon_motion_plan, line_width = 2, line_color = 'green', line_dash = 'dashed', legend_label = 'origin s_ref')
  fig4.line('time_vec', 'pos_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 's_plan')

  fig4.line('time_vec', 'soft_pos_min_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 's_lb')
  fig4.triangle ('time_vec', 'soft_pos_min_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 's_lb')
  fig4.line('time_vec', 'soft_pos_max_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 's_ub')
  fig4.inverted_triangle ('time_vec', 'soft_pos_max_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 's_ub')

  # vel
  f5 = fig5.line('time_vec', 'ref_vel_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'v_ref')
  fig5.line('time_vec', 'vel_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'v_plan')

  fig5.line('time_vec', 'vel_min_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'v_lb')
  fig5.triangle ('time_vec', 'vel_min_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'v_lb')
  fig5.line('time_vec', 'vel_max_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'v_ub')
  fig5.inverted_triangle ('time_vec', 'vel_max_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'v_ub')

  # acc
  f6 = fig6.line('time_vec', 'acc_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'a_plan')
  fig6.line('time_vec', 'acc_min_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'a_lb')
  fig6.triangle ('time_vec', 'acc_min_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'a_lb')
  fig6.line('time_vec', 'acc_max_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'a_ub')
  fig6.inverted_triangle ('time_vec', 'acc_max_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'a_ub')

  # jerk
  f7 = fig7.line('time_vec', 'jerk_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'j_plan')
  fig7.line('time_vec', 'jerk_min_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'j_lb')
  fig7.triangle ('time_vec', 'jerk_min_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'j_lb')
  fig7.line('time_vec', 'jerk_max_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'j_ub')
  fig7.inverted_triangle ('time_vec', 'jerk_max_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'j_ub')

  hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time_vec'), ('s_lb', '@soft_pos_min_vec'), ('origin s_ref', '@ref_pos_vec_origin'), ('s_ref', '@ref_pos_vec'), ('s_plan', '@pos_vec'), ('s_ub', '@soft_pos_max_vec')], mode='vline')
  hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('v_lb', '@vel_min_vec'), ('v_ref', '@ref_vel_vec'), ('v_plan', '@vel_vec'), ('v_ub', '@vel_max_vec')], mode='vline')
  hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time_vec'), ('a_lb', '@acc_min_vec'), ('a_plan', '@acc_vec'), ('a_ub', '@acc_max_vec')], mode='vline')
  hover7 = HoverTool(renderers=[f7], tooltips=[('time', '@time_vec'), ('j_lb', '@jerk_min_vec'), ('j_plan', '@jerk_vec'), ('j_ub', '@jerk_max_vec')], mode='vline')

  fig4.add_tools(hover4)
  fig5.add_tools(hover5)
  fig6.add_tools(hover6)
  fig7.add_tools(hover7)

  fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
  fig2.legend.click_policy = 'hide'

  fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
  fig3.legend.click_policy = 'hide'

  fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
  fig4.legend.click_policy = 'hide'

  fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
  fig5.legend.click_policy = 'hide'

  fig6.toolbar.active_scroll = fig6.select_one(WheelZoomTool)
  fig6.legend.click_policy = 'hide'

  fig7.toolbar.active_scroll = fig7.select_one(WheelZoomTool)
  fig7.legend.click_policy = 'hide'

  pan1 = Panel(child=row(column(fig2, fig3), column(fig4, fig5, fig6, fig7)), title="Longtime")

  tab1 = DataTable(source=data_text, columns=columns, width=500, height=800)

  pan2 = Panel(child=row(column(tab1), column(velocity_fig, acc_fig, lead_fig, fig_fsm_state), column(cost_time_fig, cutin_fig, fig_replan_status,topic_latency_fig)), title="Realtime")

  pans = Tabs(tabs=[ pan1, pan2 ])

  return pans, lon_plan_data