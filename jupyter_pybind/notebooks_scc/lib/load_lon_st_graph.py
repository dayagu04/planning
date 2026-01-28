from lib.load_rotate import *
import numpy as np
import time
import warnings
import logging

warnings.filterwarnings('ignore', category=UserWarning, module='bokeh')
logging.getLogger('bokeh').setLevel(logging.ERROR)

import ipywidgets
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from IPython.core.display import display, HTML
from bokeh.models import Label, LabelSet, DataTable, DateFormatter, TableColumn, Panel, Tabs, HTMLTemplateFormatter
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

# 枚举值映射
lane_change_status_dict = {
    0: "kLaneKeeping",
    1: "kLaneChangePropose",
    2: "kLaneChangeExecution",
    3: "kLaneChangeComplete",
    4: "kLaneChangeCancel",
    5: "kLaneChangeHold"
}

start_stop_status_dict = {
    0: "CRUISE",
    1: "STOP",
    2: "START"
}

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

def load_processed_trajectory(st_graph_data):
  agents_trajectories_info = []
  agents_trajectories_list = st_graph_data.agents_trajectories
  agents_size = len(agents_trajectories_list)

  default_x_vec = []
  default_y_vec = []

  for i in range(20):
    if i < agents_size:
      agent_trajectories_obj = agents_trajectories_list[i]
      x_vec = []
      y_vec = []

      for trajectory in agent_trajectories_obj.agent_trajectories:
          for point in trajectory.agent_trajectory:
              x_vec.append(point.x)
              y_vec.append(point.y)

      agent_trajectory_info = {
          'x_vec': x_vec,
          'y_vec': y_vec
      }

      agents_trajectories_info.append(agent_trajectory_info)
    else:
      agent_trajectory_info = {
          'x_vec': default_x_vec.copy(),
          'y_vec': default_y_vec.copy()
      }
      agents_trajectories_info.append(agent_trajectory_info)
  return agents_trajectories_info

def update_lon_plan_data(bag_loader, bag_time, local_view_data, lon_plan_data):
  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = local_view_data['data_msg']['loc_msg'].position.position_boot.x
    cur_pos_yn = local_view_data['data_msg']['loc_msg'].position.position_boot.y
    cur_yaw = local_view_data['data_msg']['loc_msg'].orientation.euler_boot.yaw
    planning_json = local_view_data['data_msg']['plan_debug_json_msg']

    try:
      json_pos_x = planning_json['ego_pos_x']
      json_pos_y = planning_json['ego_pos_y']
      json_yaw = planning_json['ego_pos_yaw']
      coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
    except:
      coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

  planning_json_value_list = ["EnvironmentalModelManagerCost", "GeneralPlannerModuleCostTime", \
                              'construct_st_graph_cost', 'st_graph_searcher_cost', \
                              'LateralMotionCostTime', 'TrajectoryGeneratorCostTime', "SccLonMotionCostTime", \
                              'last_intersection_state', 'current_intersection_state', 'distance_to_stopline', 'distance_to_crosswalk', 'traffic_status_straight', \
                              'cipv_id_st', 'road_curvature_radius', "planning_fault_code", "intersection_pass_sts", "tla_reminder_state",\
                              'new_cutin_id', 'new_cutin_id_count', \
                              "new_cutout_id", "new_cutout_id_count", \
                              'lateral_avoid_ids', 'avoid_agent_id', 'avoid_agent_v_limit', \
                              "lane_borrow_agent_id", "lane_borrow_agent_v_limit", \
                              "lon_danger_agent_ids", "v_cruise_limit","v_target_decider","v_target_type_code","construction_manual_intervention_detected",\
                              "v_target_construction","v_target_near_construction","dis_to_construction","is_exist_construction_area","is_pass_construction_area","construction_lat_dist_flag",\
                              "construction_strong_deceleration_mode", "construction_strong_mode_reason","construction_strong_mode_frame_count","agents_headway_id", "agents_headway_value",\
                              "has_target_follow_curve", "has_stable_follow_target", "has_farslow_follow_target", \
                              "closest_agent_id", "min_urgent_dist", "min_more_urgent_dist", "v_target_for_dangerous_obs", "dangerous_obs_id",\
                              "lat_path_length", "extend_path_length", "comfort_jerk_min_vec", "comfort_v_target_vec",'road_radius_origin',\
                              'v_limit_in_turns','v_limit_road','v_limit_steering','road_radius','is_s_bend',\
                              'dist_to_max_curv','is_sharp_curve','is_sharp_curve_by_decel','sharp_curve_frame_count','required_deceleration','v_limit_map_sharp_curve','ramp_curv_dist_to_max_curv','ramp_curv_min_radius','is_map_sharp_curve',\
                              "dynamic_world_cost", "front_node_id", "rear_node_id", "avg_radius_0_80m","use_avg_radius_for_ewma",\
                              "ego_left_node", "ego_left_front_node", "ego_left_rear_node", \
                              "ego_right_node", "ego_right_front_node", "ego_right_rear_node", \
                              "RealTime_desired_distance_rss", "RealTime_desired_distance_calibrate", \
                              'LateralMotionCostTime', 'RealTimeLateralBehaviorCostTime', 'TrajectoryGeneratorCostTime', \
                              "road_boundary_regular_v_limit_set", "road_boundary_regular_v_limit", "road_boundary_regular_trigger_distance", "road_boundary_regular_required_decel", "road_boundary_regular_manual_intervention_detected", "road_boundary_regular_manual_intervention_reset_count",\
                              "road_boundary_regular_is_left_triggered", "road_boundary_regular_is_right_triggered", "road_boundary_regular_left_range_idx", "road_boundary_regular_right_range_idx", "road_boundary_regular_cooldown_count", \
                              "road_boundary_strictest_applied", "road_boundary_strictest_v_limit", "road_boundary_strictest_trigger_distance", "road_boundary_strictest_required_decel","road_boundary_strictest_cooldown_count",\
                              "curve_road_boundary_road_radius", "curve_road_boundary_left_gear", "curve_road_boundary_right_gear", "curve_road_boundary_v_limit", "curve_road_boundary_trigger_distance", \
                              "lateral_acceleration_limit_triggered", "lateral_acceleration_limit_v_limit","lateral_acceleration_limit_trigger_distance",\
                              "road_boundary_collision_v_limit", "road_boundary_collision_trigger_distance", "road_boundary_collision_valid", "road_boundary_collision_distance",\
                              "s_curve_road_boundary_triggered", "s_curve_road_boundary_v_limit", "s_curve_road_boundary_trigger_distance", "s_curve_min_radius", "s_curve_first_direction", \
                              "SccLonBehaviorCostTime", "SccLonMotionCostTime"]
  st_search_value_list = ["joint_danger_agent_ids", "joint_emergency_agent_ids", "rule_base_cutin_agent_ids", "upper_bound_agent_ids", "comfort_follow_agent_ids", "lon_emergency_stop", "is_confluence_area", "joint_lead_one_id", "joint_key_agent_ids", "cross_vru_agent_ids", "parallel_longitudinal_avoid_active", "parallel_target_agent_id", "is_parallel_overtake", "is_parallel_yield", "is_lead_and_target_is_truck",
                          "parallel_decider_state", "parallel_running_frames", "parallel_cooldown_frames", "parallel_lateral_distance", 'start_stop_status', "stand_wait",'cipv_relative_s', 'cipv_relative_s_prev', "cipv_stop_distance", "cipv_vel_frenet", 
                          "soft_bound_distance", "cruise_speed", "limit_speed", 'st_graph_searcher_cost', 'search_succeed', 'search_style','expanded_nodes_size', 'history_cur_nodes_size', 'open_set_empty',
                          'gear_command','cipv_id_st', 'cipv_id_hmi', 'time_headway_level','THW', "cipv_vel_fusion", 'cipv_acc', 'cipv_acc_fusion', "cipv_theta", "cipv_theta_fusion",
                          "traffic_light_can_pass","lane_change_status","gap_lon_decision_update","gap_front_agent_id","gap_rear_agent_id",
                          "coarse_planning_info_ref_pnts_size","coarse_planning_info_ref_line_s","raw_virtual_lane_pnts_size","raw_virtual_lane_s",
                          "ignore_gap_rear_agent","rear_agent_ttc_to_ego"]

  # st_search_value_list += ['cipv_id_hmi',"lon_decision_to_invade",'invade_neighbor_front_agent_id',"lon_decision_to_invade_ego_motion_sim_path",
                          # "invade_neighbor_front_agent_id_ego_motion_sim_path",'ego_ttc_to_front_invade_agent','ego_ttc_to_front_invade_agent_ego_sim_path','invade_neighbor_decision','invade_neighbor_decision_ego_motion_sim_path']

  new_cutin_list = ['new_cutin_id', 'new_cutin_id_count']

  plan_debug_info = local_view_data['data_msg']['plan_debug_msg']
  plan_debug_json_info = local_view_data['data_msg']['plan_debug_json_msg']

  # load new st boundaries
  # print(plan_debug_info.st_graph_data.st_boundaries)
  st_info_all = load_st_polygen_points(plan_debug_info.st_graph_data)
  st_point_all = load_st_polygen_lower_upper_point(plan_debug_info.st_graph_data)
  st_label_all = load_st_label(plan_debug_info.st_graph_data)
  processed_trajectory_all = load_processed_trajectory(plan_debug_info.st_graph_data)

  data_st_boundaries = {i: lon_plan_data[f'st_boundary_{i}'] for i in range(20)}
  data_st_points = {i: lon_plan_data[f'st_point_{i}'] for i in range(20)}
  data_st_labels = {i: lon_plan_data[f'st_label_{i}'] for i in range(20)}
  data_processed_trajectories = {i: lon_plan_data[f'processed_trajectory_{i}'] for i in range(20)}

  for i in range(20):
    data_processed_trajectory = data_processed_trajectories[i]
    x_vec, y_vec = coord_tf.global_to_local(processed_trajectory_all[i]['x_vec'], processed_trajectory_all[i]['y_vec'])
    data_processed_trajectory.data.update({
      'x_vec_{}'.format(i): x_vec,
      'y_vec_{}'.format(i): y_vec,
    })

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
  vel_search_vec = []
  acc_search_vec = []
  jerk_search_vec = []
  expanded_nodes_s_vec = []
  expanded_nodes_t_vec = []
  history_cur_nodes_s_vec = []
  history_cur_nodes_t_vec = []

  st_path_final_nodes_time_vec = []
  st_path_final_nodes_cost_yield_vec = []
  st_path_final_nodes_cost_overtake_vec = []
  st_path_final_nodes_cost_vel_vec = []
  st_path_final_nodes_cost_accel_vec = []
  st_path_final_nodes_cost_accel_sign_changed_vec = []
  st_path_final_nodes_cost_jerk_vec = []
  st_path_final_nodes_cost_length_vec = []
  st_path_final_nodes_total_cost_vec = []
  st_path_final_nodes_g_cost_vec = []
  st_path_final_nodes_h_cost_vec = []
  for item in (plan_debug_info.st_graph_searcher.st_search_path):
    t_search_vec.append(item.t)
  for item in (plan_debug_info.st_graph_searcher.st_search_path):
    s_search_vec.append(item.s)
    vel_search_vec.append(item.vel)
    acc_search_vec.append(item.acc)
    jerk_search_vec.append(item.jerk)
  expanded_nodes_t_vec = plan_debug_json_info['expanded_nodes_t_vec']
  expanded_nodes_s_vec = plan_debug_json_info['expanded_nodes_s_vec']
  history_cur_nodes_t_vec = plan_debug_json_info['history_cur_nodes_t_vec']
  history_cur_nodes_s_vec = plan_debug_json_info['history_cur_nodes_s_vec']


  st_path_final_nodes_time_vec = plan_debug_json_info['st_path_final_nodes_time_vec']
  st_path_final_nodes_cost_yield_vec = plan_debug_json_info['st_path_final_nodes_cost_yield_vec']
  st_path_final_nodes_cost_overtake_vec = plan_debug_json_info['st_path_final_nodes_cost_overtake_vec']
  st_path_final_nodes_cost_vel_vec = plan_debug_json_info['st_path_final_nodes_cost_vel_vec']
  st_path_final_nodes_cost_accel_vec = plan_debug_json_info['st_path_final_nodes_cost_accel_vec']
  st_path_final_nodes_cost_accel_sign_changed_vec = plan_debug_json_info['st_path_final_nodes_cost_accel_sign_changed_vec']
  st_path_final_nodes_cost_jerk_vec = plan_debug_json_info['st_path_final_nodes_cost_jerk_vec']
  st_path_final_nodes_cost_length_vec = plan_debug_json_info['st_path_final_nodes_cost_length_vec']
  st_path_final_nodes_total_cost_vec = plan_debug_json_info['st_path_final_nodes_total_cost_vec']
  st_path_final_nodes_g_cost_vec = plan_debug_json_info['st_path_final_nodes_g_cost_vec']
  st_path_final_nodes_h_cost_vec = plan_debug_json_info['st_path_final_nodes_h_cost_vec']


  lon_plan_data['data_st_searcher'].data.update({
    't_search': t_search_vec,
    's_search': s_search_vec,
    'vel_search': vel_search_vec,
    'acc_search': acc_search_vec,
    'jerk_search': jerk_search_vec,
  })

  lon_plan_data['data_st_search_nodes'].data.update({
    'expanded_nodes_t': expanded_nodes_t_vec,
    'expanded_nodes_s': expanded_nodes_s_vec,
  })

  lon_plan_data['data_st_search_history_cur_nodes'].data.update({
    'history_cur_nodes_t': history_cur_nodes_t_vec,
    'history_cur_nodes_s': history_cur_nodes_s_vec,
  })

  lon_plan_data['data_st_search_path_final_nodes_cost'].data.update({
    'st_path_final_nodes_time': st_path_final_nodes_time_vec,
    'st_path_final_nodes_cost_yield': st_path_final_nodes_cost_yield_vec,
    'st_path_final_nodes_cost_overtake': st_path_final_nodes_cost_overtake_vec,
    'st_path_final_nodes_cost_vel': st_path_final_nodes_cost_vel_vec,
    'st_path_final_nodes_cost_accel': st_path_final_nodes_cost_accel_vec,
    'st_path_final_nodes_cost_accel_sign_changed': st_path_final_nodes_cost_accel_sign_changed_vec,
    'st_path_final_nodes_cost_jerk': st_path_final_nodes_cost_jerk_vec,
    'st_path_final_nodes_cost_length': st_path_final_nodes_cost_length_vec,
    'st_path_final_nodes_total_cost': st_path_final_nodes_total_cost_vec,
    'st_path_final_nodes_g_cost': st_path_final_nodes_g_cost_vec,
    'st_path_final_nodes_h_cost': st_path_final_nodes_h_cost_vec,
  })

  # provide visual tools to target maker
  ## final target
  t_final_target_vec = []
  v_final_target_vec = []
  s_final_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.final_target.final_target_s_ref):
    t_final_target_vec.append(item.t)
    s_final_target_vec.append(item.s)
    v_final_target_vec.append(item.v)

  ## cruise target
  t_cruise_target_vec = []
  s_cruise_target_vec = []
  v_cruise_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.cruise_target.cruise_target_s_ref):
    t_cruise_target_vec.append(item.t)
    s_cruise_target_vec.append(item.s)
    v_cruise_target_vec.append(item.v)

  ## neighbor target
  t_neighbor_target_vec = []
  s_neighbor_target_vec = []
  v_neighbor_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.neighbor_target.neighbor_target_s_ref):
    t_neighbor_target_vec.append(item.t)
    s_neighbor_target_vec.append(item.s)
    v_neighbor_target_vec.append(item.v)

  lon_plan_data['data_target_s_neighbor'].data.update({
    't_neighbor_target': t_neighbor_target_vec,
    'v_neighbor_target': v_neighbor_target_vec,
    's_neighbor_target': s_neighbor_target_vec,
  })

  ## follow target
  t_follow_target_vec = []
  s_follow_target_vec = []
  v_follow_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.follow_target.follow_target_s_ref):
    t_follow_target_vec.append(item.t)
    s_follow_target_vec.append(item.s)
    v_follow_target_vec.append(item.v)

  ## overtake target
  t_overtake_target_vec = []
  s_overtake_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.overtake_target.overtake_target_s_ref):
    t_overtake_target_vec.append(item.t)
    s_overtake_target_vec.append(item.s)
  lon_plan_data['data_target_s_overtake'].data.update({
    't_overtake_target': t_overtake_target_vec,
    's_overtake_target': s_overtake_target_vec})

  ## caution target
  t_caution_target_vec = []
  s_caution_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.caution_target.caution_target_s_ref):
    t_caution_target_vec.append(item.t)
    s_caution_target_vec.append(item.s)
  lon_plan_data['data_target_s_caution'].data.update({
    't_caution_target': t_caution_target_vec,
    's_caution_target': s_caution_target_vec})

  ## max decel s curve
  t_max_decel_vec = []
  s_max_decel_vec = []
  v_max_decel_vec = []
  a_max_decel_vec = []
  j_max_decel_vec = []
  for item in (plan_debug_info.lon_target_s_ref.max_decel_target.max_decel_s_ref):
    t_max_decel_vec.append(item.t)
    s_max_decel_vec.append(item.s)
    v_max_decel_vec.append(item.v)
    a_max_decel_vec.append(item.a)
    j_max_decel_vec.append(item.j)
  lon_plan_data['data_target_s_max_decel'].data.update({
    't_max_decel': t_max_decel_vec,
    's_max_decel': s_max_decel_vec,
    'v_max_decel': v_max_decel_vec,
    'a_max_decel': a_max_decel_vec,
    'j_max_decel': j_max_decel_vec})

  ## comfort target
  t_comfort_target_vec = []
  s_comfort_target_vec = []
  v_comfort_target_vec = []
  a_comfort_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.comfort_target.comfort_target_s_ref):
    t_comfort_target_vec.append(item.t)
    s_comfort_target_vec.append(item.s)
    v_comfort_target_vec.append(item.v)
    a_comfort_target_vec.append(item.a)

  ## upper bound velocities from comfort target
  t_upper_bound_vec = []
  v_upper_bound_vec = []
  a_upper_bound_vec = []
  s_upper_bound_vec = []
  agent_id_upper_bound_vec = []
  for item in (plan_debug_info.lon_target_s_ref.comfort_target.upper_bound_infos):
    t_upper_bound_vec.append(item.t)
    v_upper_bound_vec.append(item.v)
    a_upper_bound_vec.append(item.a)
    s_upper_bound_vec.append(item.s)
    agent_id_upper_bound_vec.append(item.agent_id)

  lon_plan_data['data_target_s_comfort'].data.update({
    't_comfort_target': t_comfort_target_vec,
    'v_comfort_target': v_comfort_target_vec,
    's_comfort_target': s_comfort_target_vec,
    'a_comfort_target': a_comfort_target_vec,
    't_upper_bound': t_upper_bound_vec,
    'v_upper_bound': v_upper_bound_vec,
    'a_upper_bound': a_upper_bound_vec,
    's_upper_bound': s_upper_bound_vec,
    'agent_id_upper_bound': agent_id_upper_bound_vec})

  ## cross vru target
  t_cross_vru_target_vec = []
  s_cross_vru_target_vec = []
  v_cross_vru_target_vec = []
  for item in (plan_debug_info.lon_target_s_ref.cross_vru_target.cross_vru_target_s_ref):
    t_cross_vru_target_vec.append(item.t)
    s_cross_vru_target_vec.append(item.s)
    v_cross_vru_target_vec.append(item.v)
  lon_plan_data['data_target_s_cross_vru'].data.update({
    't_cross_vru_target': t_cross_vru_target_vec,
    'v_cross_vru_target': v_cross_vru_target_vec,
    's_cross_vru_target': s_cross_vru_target_vec})

  lon_plan_data['data_target'].data.update({
    't_final_target': t_final_target_vec,
    's_final_target': s_final_target_vec,
    'v_final_target': v_final_target_vec,
    't_follow_target': t_follow_target_vec,
    's_follow_target': s_follow_target_vec,
    'v_follow_target': v_follow_target_vec,
    't_cruise_target': t_cruise_target_vec,
    's_cruise_target': s_cruise_target_vec,
    'v_cruise_target': v_cruise_target_vec
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

  s_speed_adjust_target_vec = []
  if hasattr(plan_debug_info.long_ref_path, 's_speed_adjust_target'):
    s_speed_adjust_target_vec = list(plan_debug_info.long_ref_path.s_speed_adjust_target)
  
  s_lane_change_target_vec = []
  if hasattr(plan_debug_info.long_ref_path, 's_lane_change_target'):
    s_lane_change_target_vec = list(plan_debug_info.long_ref_path.s_lane_change_target)

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
     val = plan_debug_json_info[planning_json_value_list[ind]]
     # 转换枚举值为可读的字符串
     if planning_json_value_list[ind] == 'lane_change_status' and isinstance(val, int):
       val = lane_change_status_dict.get(val, str(val))
     elif planning_json_value_list[ind] == 'start_stop_status' and isinstance(val, int):
       val = start_stop_status_dict.get(val, str(val))
     vision_lon_attr_vec.append(val)

  st_search_attr_vec = []
  for ind in range(len(st_search_value_list)):
    val = plan_debug_json_info[st_search_value_list[ind]]
    # 转换枚举值为可读的字符串
    if st_search_value_list[ind] == 'lane_change_status' and isinstance(val, int):
      val = lane_change_status_dict.get(val, str(val))
    elif st_search_value_list[ind] == 'start_stop_status' and isinstance(val, int):
      val = start_stop_status_dict.get(val, str(val))
    st_search_attr_vec.append(val)

  cutin_attr_vec = []
  for ind in range(len(new_cutin_list)):
     cutin_attr_vec.append(plan_debug_json_info[new_cutin_list[ind]])

  lon_plan_data['data_st'].data.update({
    't': t_vec,
    's': s_ref_vec,
    's_speed_adjust_target': s_speed_adjust_target_vec,
    's_lane_change_target': s_lane_change_target_vec,
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

  lon_plan_data['data_st_search_text'].data.update({
    'StSearchAttr': st_search_value_list,
    'StSearchVal': st_search_attr_vec,
  })

  lon_plan_data['data_cutin'].data.update({
    'cutinAttr': new_cutin_list,
    'cutinVal': cutin_attr_vec
  })

  try:
    # print("obstacle st info: ",plan_debug_info.st_search_decider_info.obstacle_st_infos)
    for idx in range(len(plan_debug_info.st_search_decider_info.obstacle_st_infos)):
      print("idx: ",idx)
      obj_s_upper_vec =[]
      obj_s_lower_vec =[]
      obj_t_vec =[]
      obj_s_upper_vec = list(plan_debug_info.st_search_decider_info.obstacle_st_infos[idx].s_vec_upper)
      obj_s_lower_vec = list(plan_debug_info.st_search_decider_info.obstacle_st_infos[idx].s_vec_lower)
      obj_t_vec = list(plan_debug_info.st_search_decider_info.obstacle_st_infos[idx].t_vec)
      print("s_upper: ", obj_s_upper_vec)
      print("s_lower: ", obj_s_lower_vec)
      try:
        lon_plan_data['data_search_obj_{}'.format(idx)].data.update({'t': obj_t_vec,'s_upper':obj_s_upper_vec,'s_lower':obj_s_lower_vec})
      except:
        print("update speed search st failed, obj idx:", idx)
  except:
    print("update speed search obj failed!")

  # motion planning
  lon_motion_plan_input = plan_debug_info.longitudinal_motion_planning_input
  lon_motion_plan_output = plan_debug_info.longitudinal_motion_planning_output

  init_state = lon_motion_plan_input.init_state
  ref_pos_vec = lon_motion_plan_input.ref_pos_vec
  ref_vel_vec = lon_motion_plan_input.ref_vel_vec
  hard_pos_max_vec = lon_motion_plan_input.hard_pos_max_vec
  hard_pos_min_vec = lon_motion_plan_input.hard_pos_min_vec
  soft_pos_max_vec = lon_motion_plan_input.soft_pos_max_vec
  soft_pos_min_vec = lon_motion_plan_input.soft_pos_min_vec
  vel_max_vec = lon_motion_plan_input.vel_max_vec
  vel_min_vec = lon_motion_plan_input.vel_min_vec
  acc_max_vec = lon_motion_plan_input.acc_max_vec
  acc_min_vec = lon_motion_plan_input.acc_min_vec
  jerk_max_vec = lon_motion_plan_input.jerk_max_vec
  jerk_min_vec = lon_motion_plan_input.jerk_min_vec
  s_stop = lon_motion_plan_input.s_stop

  comfort_jerk_min_vec = []
  comfort_v_target_vec = []
  try:
    comfort_jerk_min_vec = plan_debug_json_info['comfort_jerk_min_vec']
    comfort_v_target_vec = plan_debug_json_info['comfort_v_target_vec']
  except Exception as e:
    print(f"Error getting comfort data: {e}")
    time_length = len(time_vec)
    comfort_jerk_min_vec = [0.0] * time_length
    comfort_v_target_vec = [0.0] * time_length

  # time_vec = []
  # for i in range(len(ref_pos_vec)):
  #   time_vec.append(i * 0.2)

  time_vec = lon_motion_plan_output.time_vec
  pos_vec = lon_motion_plan_output.pos_vec
  vel_vec = lon_motion_plan_output.vel_vec
  acc_vec = lon_motion_plan_output.acc_vec
  jerk_vec = lon_motion_plan_output.jerk_vec

  weight_maker_replay_info = plan_debug_info.weight_maker.weight_maker_replay_info
  # print(weight_maker_replay_info)
  s_weight_vec = []
  for item in (weight_maker_replay_info.target_point):
    s_weight_vec.append(item.s_weight)
  print(s_weight_vec)

  # print("lon_motion_plan_output:=", lon_motion_plan_output)
  motion_solver_info = lon_motion_plan_output.solver_info
  iter_count = motion_solver_info.iter_count
  cost_size = motion_solver_info.cost_size
  cost_vec = motion_solver_info.cost_vec
  lists = [cost_vec[i * cost_size : (i + 1) * cost_size] for i in range(iter_count)]
  cost_list = ["ReferenceCost", "LonAccCost", "LonJerkCost", "LonSoftPosBoundCost", "LonHardPosBoundCost", \
               "LonVelBoundCost", "LonAccBoundCost", "LonJerkBoundCost", "NonNegativeVelCost", "LonPosSafeCost", "LonEmergencyStopCost"]
  print(cost_list)
  for i, sub_list in enumerate(lists):
    if i == 0:
      print(f"Cost init: {sub_list}")
    elif i == len(lists) - 1:
      print(f"Cost {i}: {sub_list}")
      break

  lon_plan_data['data_lon_motion_plan'].data.update({
    'time_vec': time_vec,
    'ref_pos_vec_origin': s_ref_vec,
    'ref_pos_vec': ref_pos_vec,
    'ref_vel_vec': ref_vel_vec,
    'hard_pos_max_vec': hard_pos_max_vec,
    'hard_pos_min_vec': hard_pos_min_vec,
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
    'comfort_jerk_min_vec': comfort_jerk_min_vec,
    'comfort_v_target_vec': comfort_v_target_vec,
  })

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

      plan_traj_x, plan_traj_y = coord_tf.global_to_local(plan_x, plan_y)

    lon_plan_data['data_planning'].data.update({
      'plan_traj_y' : plan_traj_y,
      'plan_traj_x' : plan_traj_x,
      })

  # Extract and update lat-lon joint planner optimized trajectory
  plan_debug = local_view_data['data_msg']['plan_debug_msg']
  jp_out = plan_debug.joint_motion_planning_output
  if len(jp_out.x_vec) > 0 and len(jp_out.y_vec) > 0:
    lat_lon_opt_x = list(jp_out.x_vec)
    lat_lon_opt_y = list(jp_out.y_vec)
    lat_lon_opt_x_local, lat_lon_opt_y_local = coord_tf.global_to_local(lat_lon_opt_x, lat_lon_opt_y)
    
    lon_plan_data['data_lat_lon_opt_trajectory'].data.update({
      'lat_lon_opt_traj_y': lat_lon_opt_y_local,
      'lat_lon_opt_traj_x': lat_lon_opt_x_local,
    })
  else:
    lon_plan_data['data_lat_lon_opt_trajectory'].data.update({
      'lat_lon_opt_traj_y': [],
      'lat_lon_opt_traj_x': [],
    })

  # Extract and update key obstacles reference trajectories
  jp_input = plan_debug.joint_motion_planning_input
  num_obs = 0
  if hasattr(jp_input, 'obs_ref_trajectory') and len(jp_input.obs_ref_trajectory) > 0:
    num_obs = min(10, len(jp_input.obs_ref_trajectory))
    for i in range(num_obs):
      obs_traj = jp_input.obs_ref_trajectory[i]
      if len(obs_traj.ref_x_vec) > 0 and len(obs_traj.ref_y_vec) > 0:
        obs_x = list(obs_traj.ref_x_vec)
        obs_y = list(obs_traj.ref_y_vec)
        obs_x_local, obs_y_local = coord_tf.global_to_local(obs_x, obs_y)
        
        lon_plan_data[f'data_key_obs_traj_{i}'].data.update({
          'obs_traj_x': obs_x_local,
          'obs_traj_y': obs_y_local,
          'agent_id': [obs_traj.obs_id] * len(obs_x_local),
        })
      else:
        lon_plan_data[f'data_key_obs_traj_{i}'].data.update({
          'obs_traj_x': [],
          'obs_traj_y': [],
          'agent_id': [],
        })
  
  # Clear remaining obstacle trajectories (indices num_obs to 9)
  for i in range(num_obs, 10):
    lon_plan_data[f'data_key_obs_traj_{i}'].data.update({
      'obs_traj_x': [],
      'obs_traj_y': [],
      'agent_id': [],
    })

  # Extract and update joint motion planner output (s, v, a, j)
  if (len(jp_out.time_vec) > 0 and len(jp_out.s_vec) > 0 and
      len(jp_out.vel_vec) > 0 and len(jp_out.acc_vec) > 0 and
      len(jp_out.jerk_vec) > 0):
    lon_plan_data['data_joint_motion_plan'].data.update({
      'time_vec': list(jp_out.time_vec),
      's_vec': list(jp_out.s_vec),
      'vel_vec': list(jp_out.vel_vec),
      'acc_vec': list(jp_out.acc_vec),
      'jerk_vec': list(jp_out.jerk_vec),
    })
  else:
    lon_plan_data['data_joint_motion_plan'].data.update({
      'time_vec': [],
      's_vec': [],
      'vel_vec': [],
      'acc_vec': [],
      'jerk_vec': [],
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

  # 更新 ST 图数据源
  lon_plan_data['data_st'].data.update({
    't': t_vec,
    's': s_ref_vec,
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
  cipv_vel_vec = []
  cipv_vel_fusion_vec = []
  t_plan_vec = bag_loader.plan_debug_msg['t']
  t_loc_vec = bag_loader.loc_msg['t']
  t_vehicle_service_vec = bag_loader.vs_msg['t']
  curve_limit_velocity_vec = []
  speed_decider_limit_velocity_vec = []
  # road_boundary_regular_v_limit_vec = []
  # road_boundary_strictest_v_limit_vec = []
  # for ind in range(len(bag_loader.plan_debug_msg['json'])):
    # target_velocity_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['v_target'], 2))
    # ref_velocity_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['RealTime_v_ref'], 2))
    # leadone_velocity_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lead_one_vel'], 2))
    # leadtwo_velocity_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lead_two_vel'], 2))

  for ind in range(len(bag_loader.vs_msg['data'])):
    ego_velocity_vec.append(round(bag_loader.vs_msg['data'][ind].vehicle_speed, 2))

  for ind in range(len(bag_loader.plan_debug_msg['json'])):
    cipv_vel = round(bag_loader.plan_debug_msg['json'][ind]['cipv_vel_frenet'], 2)
    if cipv_vel > 40.0:
      cipv_vel = 0.0
    cipv_vel_vec.append(cipv_vel)
    cipv_vel_fusion_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['cipv_vel_fusion'], 2))
    curve_limit_velocity_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['v_limit_in_turns'], 2))
    speed_decider_limit_velocity_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['v_target_decider'], 2))
    #road_boundary_regular_v_limit_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['road_boundary_regular_v_limit'], 2))
    #road_boundary_strictest_v_limit_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['road_boundary_strictest_v_limit'], 2))

  velocity_fig.line(t_plan_vec, cipv_vel_vec, line_width=1,
                              legend_label='cipv_vel', color="green")
  velocity_fig.line(t_plan_vec, cipv_vel_fusion_vec, line_width=1,
                              legend_label='cipv_vel_fusion', color="red")
  velocity_fig.line(t_vehicle_service_vec, ego_velocity_vec, line_width=1,
                                legend_label='ego_velocity',color="blue")
  velocity_fig.line(t_plan_vec, curve_limit_velocity_vec, line_width=1,
                    legend_label='v_limit_in_turns', color="teal")  # 点状虚线，青绿色
  velocity_fig.line(t_plan_vec, speed_decider_limit_velocity_vec, line_width=1,
                    legend_label='speed_decider_limit_velocity', color="orange") 
  # velocity_fig.line(t_plan_vec, road_boundary_regular_v_limit_vec, line_width=1,
  #                     legend_label='boundary_regular_v', color="purple")
  # velocity_fig.line(t_plan_vec, road_boundary_strictest_v_limit_vec, line_width=1,
  #                     legend_label='boundary_strictest_v', color="brown")
  # velocity_fig.line(t_plan_vec, leadone_velocity_vec, line_width=1,
  #                             legend_label='leadone_velocity', color="red")
  # velocity_fig.line(t_plan_vec, leadtwo_velocity_vec, line_width=1,
  #                               legend_label='leadtwo_velocity',color="orange")
  velocity_fig.legend.click_policy = 'hide'


  acc_fig = bkp.figure(title='加速度',x_axis_label='time/s',
                y_axis_label='acc/(m/s2)',width=600,height=300)

  ego_acc_vec = []
  acc_min_vec = []
  acc_max_vec = []
  cipv_acc = []
  cipv_acc_fusion = []
  cipv_theta = []
  cipv_theta_fusion = []

  t_vs_vec = bag_loader.vs_msg['t']
  for ind in range(len(bag_loader.plan_debug_msg['json'])):
  #   acc_min_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['acc_target_low'], 2))
  #   acc_max_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['acc_target_high'], 2))
      cipv_acc.append(round(bag_loader.plan_debug_msg['json'][ind]['cipv_acc'], 2))
      cipv_acc_fusion.append(round(bag_loader.plan_debug_msg['json'][ind]['cipv_acc_fusion'], 2))
      cipv_theta.append(round(bag_loader.plan_debug_msg['json'][ind]['cipv_theta'], 2))
      cipv_theta_fusion.append(round(bag_loader.plan_debug_msg['json'][ind]['cipv_theta_fusion'], 2))
  for ind in range(len(bag_loader.vs_msg['data'])):
    ego_acc_vec.append(round(bag_loader.vs_msg['data'][ind].long_acceleration, 2))

  # acc_fig.line(t_plan_vec, acc_min_vec, line_width=1,
  #                             legend_label='acc_min', color="brown")
  acc_fig.line(t_vs_vec, ego_acc_vec, line_width=1,
                                legend_label='ego_acc',color="blue")
  # acc_fig.line(t_plan_vec, acc_max_vec, line_width=1,
  #                             legend_label='acc_max', color="red")
  acc_fig.line(t_plan_vec, cipv_acc, line_width=2,
                              legend_label='cipv_acc', color="green")
  acc_fig.line(t_plan_vec, cipv_acc_fusion, line_width=2,
                              legend_label='cipv_acc_fusion', color="red")
  acc_fig.line(t_plan_vec, cipv_theta, line_width=2,
                              legend_label='cipv_theta', color="pink")
  acc_fig.line(t_plan_vec, cipv_theta_fusion, line_width=2,
                              legend_label='cipv_theta_fusion', color="cyan")
  acc_fig.legend.click_policy = 'hide'

  ego_jerk_vec = []
  for ind in range(len(bag_loader.plan_debug_msg['json'])):
    ego_jerk_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['ego_jerk'], 2))

  jerk_fig = bkp.figure(title='加加速度',x_axis_label='time/s',
                y_axis_label='jerk/(m/s3)',width=600,height=300)
  jerk_fig.line(t_plan_vec, ego_jerk_vec, line_width=1, legend_label='ego_jerk', color="blue")
  jerk_fig.legend.click_policy = 'hide'

  # 各阶段耗时
  cost_time_fig = bkp.figure(title='耗时',x_axis_label='time/s',
                  y_axis_label='time cost/(ms)',width=600,height=300)

  plan_debug_Astar = ColumnDataSource(data ={'t_plan_debug': [],'st_graph_searcher_cost': []})

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
  st_graph_searcher_cost_vec = []

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
    st_graph_searcher_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['st_graph_searcher_cost'], 2))

  t_plan_debug = bag_loader.plan_debug_msg['t']
  plan_debug_Astar.data.update({
  't_plan_debug': t_plan_debug,
  'st_graph_searcher_cost': st_graph_searcher_cost_vec,
  })

  cost_time_fig = bkp.figure(title='耗时',x_axis_label='time/s',
                  y_axis_label='time cost/(ms)', x_range = [t_plan_debug[0], t_plan_debug[-1]], width=600,height=300)
  cost_time_fig.line(t_plan_vec, SccLonBehaviorCostTime_vec, line_width=1, legend_label='LonBehaviorCostTime', color="red")
  cost_time_fig.line(t_plan_vec, SccLonMotionCostTime_vec, line_width=1, legend_label='LonMotionCostTime_vec', color="blue")
  cost_time_fig.line(t_plan_vec, SccLateralMotionCostTime_vec, line_width=1, legend_label='LatMotionCostTime_vec', color="orange")
  cost_time_fig.line(t_plan_vec, EnvironmentalModelManagerCost_vec, line_width=1, legend_label='EnvironmentalCostTime_vec', color="yellow")
  cost_time_fig.line(t_plan_vec, GeneralPlannerModuleCostTime_vec, line_width=1, legend_label='GeneralPlannerModuleCostTime', color="purple")
  f_cost_time = cost_time_fig.line("t_plan_debug", "st_graph_searcher_cost", source = plan_debug_Astar, line_width=1.6, legend_label='st_graph_searcher_cost', color="green")
  cost_time_fig.legend.click_policy = 'hide'

  hover_cost_time = HoverTool(renderers=[f_cost_time], tooltips=[('t_plan_debug','@t_plan_debug'),('st_graph_searcher_cost', '@st_graph_searcher_cost')], mode='vline')
  cost_time_fig.add_tools(hover_cost_time)
  cost_time_fig.toolbar.active_scroll = cost_time_fig.select_one(WheelZoomTool)


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

  # LonBahaviorAverageCostTime = sum(SccLonBehaviorCostTime_vec) / len(t_plan_vec)
  LonMotionAverageCostTime = sum(SccLonMotionCostTime_vec) / len(t_plan_vec)
  # LateralBahaviorAverageCostTime = sum(SccLateralBehaviorCostTime_vec) / len(t_plan_vec)
  LateralMotionAverageCostTime = sum(SccLateralMotionCostTime_vec) / len(t_plan_vec)
  EnvironmentalAverageCostTime = sum(EnvironmentalModelManagerCost_vec) / len(t_plan_vec)
  # GeneralPlannerAverageCostTime = sum(GeneralPlannerModuleCostTime_vec) / len(t_plan_vec)
  DynamicWorldAverageCostTime = sum(DynamicWorldAverageCostTime_vec) / len(t_plan_vec)

  lat_lon_joint_cost_vec = []
  for ind in range(len(bag_loader.plan_debug_msg['json'])):
    lat_lon_joint_cost = round(bag_loader.plan_debug_msg['json'][ind].get('LatLonJointPlannerDeciderTime', 0.0), 2)
    lat_lon_joint_cost_vec.append(lat_lon_joint_cost)
  
  # Print average cost times for lateral and longitudinal planners
  print('lat_motion_average_cost', LateralMotionAverageCostTime)
  print('lon_motion_average_cost', LonMotionAverageCostTime)
  if lat_lon_joint_cost_vec:
    lat_lon_joint_average_cost = sum(lat_lon_joint_cost_vec) / len(lat_lon_joint_cost_vec)
    print('lat_lon_motion_average_cost: ', lat_lon_joint_average_cost)
  print('Environmental_average_cost', EnvironmentalAverageCostTime)
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
  fig_fsm_state.text(x=x_value, y=24, text=['MANUAL:0'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=22, text=['ERROR:1'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=20, text=['MRC:2'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=18, text=['DRIVING_PASSIVE(行车抑制):3'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=16, text=['ACC_OVERRIDE:6'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
  fig_fsm_state.text(x=x_value, y=14, text=['ACC_STANDBY:4'], text_align='left', text_baseline='middle', text_font_size='9pt', text_color='black')
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
  prediction_latency = []
  vehicle_service_latency=[]
  control_output_latency=[]
  hmi_latency=[]
  function_state_machine_latency=[]
  localization_latency=[]
  localization_estimate_latency=[]

  for ind in range(len(bag_loader.plan_debug_msg['data'])):
    fusion_object_latency.append(round(bag_loader.plan_debug_msg['data'][ind].input_topic_latency.fusion_object, 2))
    prediction_latency.append(round(bag_loader.plan_debug_msg['data'][ind].input_topic_latency.prediction, 2))
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
  topic_latency_fig.line(t_plan_vec, prediction_latency, line_width=1, line_dash = 'dashed',
                              legend_label='prediction', color="blue")
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


  return velocity_fig, acc_fig, jerk_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status,topic_latency_fig

def load_lon_plan_figure(fig1, velocity_fig, acc_fig, jerk_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status,topic_latency_fig):
  data_st = ColumnDataSource(data = {
    't':[], 's':[], 's_speed_adjust_target':[], 's_lane_change_target':[]
  })
  data_st_plan = ColumnDataSource(data = {'t_long':[], 's_plan':[], 'v_plan':[]})
  data_sv = ColumnDataSource(data = {'s_ref':[], 'v_ref':[], 'v_low':[], 'v_high':[]}) # , 'sv_bound_s':[], 'sv_bound_v':[]
  data_tv = ColumnDataSource(data = {'t':[], 'vel':[]})
  data_ta = ColumnDataSource(data = {'t':[], 'acc':[]})
  data_tj = ColumnDataSource(data = {'t':[], 'jerk':[]})
  data_text = ColumnDataSource(data = {'VisionLonAttr':[], 'VisionLonVal':[]})
  data_st_search_text = ColumnDataSource(data = {'StSearchAttr':[], 'StSearchVal': []})
  data_cutin = ColumnDataSource(data = {'cutinAttr':[], 'cutinVal':[]})
  data_st_searcher = ColumnDataSource(data = {'t_search':[], 's_search':[], 'vel_search':[], 'acc_search':[], 'jerk_search':[]})
  data_st_search_nodes = ColumnDataSource(data = {'expanded_nodes_t':[], 'expanded_nodes_s':[]})
  data_st_search_history_cur_nodes = ColumnDataSource(data = {'history_cur_nodes_t':[], 'history_cur_nodes_s':[]})
  data_st_search_path_final_nodes_cost = ColumnDataSource(data = {
                                                              'st_path_final_nodes_time': [],
                                                              'st_path_final_nodes_cost_yield':[],
                                                              'st_path_final_nodes_cost_overtake' : [],
                                                              'st_path_final_nodes_cost_vel' : [],
                                                              'st_path_final_nodes_cost_accel' : [],
                                                              'st_path_final_nodes_cost_accel_sign_changed' : [],
                                                              'st_path_final_nodes_cost_jerk' : [],
                                                              'st_path_final_nodes_cost_length' : [],
                                                              'st_path_final_nodes_total_cost' : [],
                                                              'st_path_final_nodes_g_cost' : [],
                                                              'st_path_final_nodes_h_cost' : [],
                                                              })
  data_target = ColumnDataSource(data = {'t_final_target':[], 's_final_target':[], 'v_final_target':[],
                                         't_cruise_target':[], 's_cruise_target':[], 'v_cruise_target':[],
                                         't_follow_target':[], 's_follow_target':[], 'v_follow_target':[]})
  data_target_s_neighbor = ColumnDataSource(data = {'t_neighbor_target':[], 's_neighbor_target':[]})
  data_target_s_overtake = ColumnDataSource(data = {'t_overtake_target':[], 's_overtake_target':[]})
  data_target_s_caution = ColumnDataSource(data = {'t_caution_target':[], 's_caution_target':[]})
  data_target_s_max_decel = ColumnDataSource(data = {'t_max_decel':[], 's_max_decel':[], 'v_max_decel':[], 'a_max_decel':[], 'j_max_decel':[]})
  data_target_s_comfort = ColumnDataSource(data = {'t_comfort_target':[], 's_comfort_target':[], 'v_comfort_target':[], 'a_comfort_target':[],
                                                  't_upper_bound':[], 'v_upper_bound':[], 'a_upper_bound':[], 's_upper_bound':[], 'agent_id_upper_bound':[]})
  data_target_s_cross_vru = ColumnDataSource(data = {'t_cross_vru_target':[], 's_cross_vru_target':[]})
  #obstacles st data, key is id, value is time and s list
  data_obs_st = {}
  for it in obs_st_ids:
     one_cds = ColumnDataSource(data = {'obs_t':[], 'obs_s':[], 'obs_high_id':[], 'obs_high_type':[]})
     data_obs_st[str(it)] = one_cds

  data_lon_motion_plan = ColumnDataSource(data = {'time_vec': [],
                                                  'ref_pos_vec_origin': [],
                                                  'ref_pos_vec':[],
                                                  'ref_vel_vec':[],
                                                  'hard_pos_max_vec': [],
                                                  'hard_pos_min_vec': [],
                                                  'soft_pos_max_vec':[],
                                                  'soft_pos_min_vec':[],
                                                  'vel_max_vec':[],
                                                  'vel_min_vec':[],
                                                  'acc_max_vec':[],
                                                  'acc_min_vec':[],
                                                  'jerk_max_vec':[],
                                                  'jerk_min_vec':[],
                                                  'pos_vec': [],
                                                  'vel_vec': [],
                                                  'acc_vec': [],
                                                  'jerk_vec': [],
                                                        })

  data_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                    'plan_traj_x':[],})

  # Lat-Lon joint planner optimized trajectory
  data_lat_lon_opt_trajectory = ColumnDataSource(data = {'lat_lon_opt_traj_y':[],
                                                         'lat_lon_opt_traj_x':[],})

  # Lat-Lon joint motion planner output (s, v, a, j)
  data_joint_motion_plan = ColumnDataSource(data = {'time_vec': [],
                                                    's_vec': [],
                                                    'vel_vec': [],
                                                    'acc_vec': [],
                                                    'jerk_vec': [],
                                                    })

  # Key obstacles reference trajectories (up to 10 obstacles)
  data_key_obs_trajectories = {}
  for i in range(10):
    data_key_obs_trajectories[i] = ColumnDataSource(data = {'obs_traj_x': [],
                                                             'obs_traj_y': [],
                                                             'agent_id': [],
                                                             })

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
                   'data_lat_lon_opt_trajectory': data_lat_lon_opt_trajectory, \
                   'data_joint_motion_plan': data_joint_motion_plan, \
                   'data_st_searcher':data_st_searcher, \
                   'data_st_search_nodes' : data_st_search_nodes, \
                   'data_st_search_history_cur_nodes' : data_st_search_history_cur_nodes, \
                   'data_st_search_path_final_nodes_cost' : data_st_search_path_final_nodes_cost, \
                   'data_target': data_target, \
                   'data_target_s_neighbor': data_target_s_neighbor, \
                   'data_target_s_overtake': data_target_s_overtake, \
                   'data_target_s_caution': data_target_s_caution, \
                   'data_target_s_max_decel': data_target_s_max_decel, \
                   'data_target_s_comfort': data_target_s_comfort, \
                   'data_target_s_cross_vru': data_target_s_cross_vru, \
                   'data_st_search_text' : data_st_search_text, \
  }
  
  # Add key obstacles trajectory data sources
  for i in range(10):
    lon_plan_data[f'data_key_obs_traj_{i}'] = data_key_obs_trajectories[i]

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
    data4 = {
      f'x_vec_{i}': [],
      f'y_vec_{i}': [],
    }
    lon_plan_data[f'st_boundary_{i}'] = ColumnDataSource(data=data1)
    lon_plan_data[f'st_point_{i}'] = ColumnDataSource(data=data2)
    lon_plan_data[f'st_label_{i}'] = ColumnDataSource(data=data3)
    lon_plan_data[f'processed_trajectory_{i}'] = ColumnDataSource(data=data4)


  boundaries = [lon_plan_data[f'st_boundary_{i}'] for i in range(0, 20)]
  st_points = [lon_plan_data[f'st_point_{i}'] for i in range(0, 20)]
  st_labels = [lon_plan_data[f'st_label_{i}'] for i in range(0, 20)]
  processed_trajectories = [lon_plan_data[f'processed_trajectory_{i}'] for i in range(0, 20)]

  columns = [
        TableColumn(field="VisionLonAttr", title="VisionLonAttr", width=300, formatter=HTMLTemplateFormatter(template='<div style="font-size: 14px;"><%= value %></div>')),
        TableColumn(field="VisionLonVal", title="VisionLonVal", width=200, formatter=HTMLTemplateFormatter(template='<div style="font-size: 14px;"><%= value %></div>')),
    ]
  st_search_columns = [
        TableColumn(field="StSearchAttr", title="StSearchAttr", width=200, formatter=HTMLTemplateFormatter(template='<div style="font-size: 14px;"><%= value %></div>')),
        TableColumn(field="StSearchVal", title="StSearchVal", width=200, formatter=HTMLTemplateFormatter(template='<div style="font-size: 14px;"><%= value %></div>')),
  ]
  cutin_colums = [
      TableColumn(field="cutinAttr", title="cutinAttr", width=200, formatter=HTMLTemplateFormatter(template='<div style="font-size: 14px;"><%= value %></div>')),
      TableColumn(field="cutinVal", title="cutinVal", width=200, formatter=HTMLTemplateFormatter(template='<div style="font-size: 14px;"><%= value %></div>'))
  ]
  # 悬停工具配置，使用 data_lon_motion_plan 数据源中的字段
  hover = HoverTool(tooltips = [
     ('index','$index'),
     ('time','@time_vec'),
     ('s_ref','@ref_pos_vec'),
     ('s_plan','@pos_vec'),
     ('s_soft_ub','@soft_pos_max_vec'),
     ('s_soft_lb','@soft_pos_min_vec'),
     ('s_hard_ub','@hard_pos_max_vec'),
     ('s_hard_lb','@hard_pos_min_vec')
  ])

  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan debug', visible=False)
  
  # Lat-Lon joint planner optimized trajectory
  fig1.line('lat_lon_opt_traj_y', 'lat_lon_opt_traj_x', source = data_lat_lon_opt_trajectory, line_width = 5, line_color = 'green', line_dash = 'solid', line_alpha = 0.8, legend_label = 'lat_lon_opt_trajectory', visible=False)
  
  # Key obstacles reference trajectories
  obstacle_colors = ['orange', 'cyan', 'magenta', 'lime', 'pink', 'brown', 'purple', 'yellow', 'gold', 'coral']
  for i in range(10):
    source = data_key_obs_trajectories[i]
    color = obstacle_colors[i % len(obstacle_colors)]
    fig1.line('obs_traj_y', 'obs_traj_x', source=source, line_width=6, line_color=color, line_dash='solid', line_alpha=1.0, legend_label='key_obs_ref_traj', visible=False)
    fig1.circle('obs_traj_y', 'obs_traj_x', source=source, size=6, color=color, alpha=0.9, legend_label='key_obs_ref_traj', visible=False)
  
  for i, source in enumerate(processed_trajectories):
    x_vec = f'x_vec_{i}'
    y_vec = f'y_vec_{i}'

    fig1.line(y_vec, x_vec, source = source, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.6, legend_label = 'processed agent traj', visible=False)
    fig1.circle(y_vec, x_vec, source=source, size=3, color='yellow', legend_label='processed agent traj', visible=False)

  # fig2 S-T
  fig2 = bkp.figure(x_axis_label='t', y_axis_label='s', x_range = [-0.1, 7.0], width=600, height=450, tools=[hover,'pan,wheel_zoom,box_zoom,reset'], match_aspect = True, aspect_scale=1)
  # fig3 S-V
  # fig3 = bkp.figure(x_axis_label='s', y_axis_label='v', width=600, height=400, match_aspect = True, aspect_scale=1)
  fig3 = bkp.figure(x_axis_label='t', y_axis_label='s', x_range = [-0.1, 7.0], width=600, height=450, match_aspect = True, aspect_scale=1)
  # fig5 v-t
  fig5 = bkp.figure(x_axis_label='time', y_axis_label='vel',x_range = [-0.1, 6.5], width=600, height=300)
  # fig6 a-t
  fig6 = bkp.figure(x_axis_label='time', y_axis_label='acc',x_range = fig5.x_range, width=600, height=300)
  # fig7 j-t
  fig7 = bkp.figure(x_axis_label='time', y_axis_label='jerk',x_range = fig6.x_range, width=600, height=300)

  fig2.line('time_vec', 'ref_pos_vec', source = data_lon_motion_plan, line_width = 2.5, line_color = 'red', line_dash = 'dashed', legend_label = 's_ref')
  #fig2.line('t_long', 's_plan', source = data_st_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 's_plan')
  fig2.line('time_vec', 'pos_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 's_plan')
  # Plot joint motion planner s on fig2
  fig2.line('time_vec', 's_vec', source = data_joint_motion_plan, line_width = 3, line_color = 'darkblue', line_dash = 'solid', legend_label = 's_joint_opt', visible=False)
  fig2.line('time_vec', 'soft_pos_min_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'magenta', line_dash = 'solid', legend_label = 's_soft_lb', visible=False)
  fig2.line('time_vec', 'soft_pos_max_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'magenta', line_dash = 'solid', legend_label = 's_soft_ub')
  fig2.triangle('time_vec', 'soft_pos_min_vec', source = data_lon_motion_plan, size = 10, fill_color='magenta', line_color='magenta', alpha = 0.7, legend_label = 's_soft_lb_point', visible=False)
  fig2.triangle('time_vec', 'soft_pos_max_vec', source = data_lon_motion_plan, size = 10, fill_color='magenta', line_color='magenta', alpha = 0.5, legend_label = 's_soft_ub_point')

  fig2.line('time_vec', 'hard_pos_min_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'obs_lb')
  fig2.line('time_vec', 'hard_pos_max_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'obs_ub')
  fig2.triangle('time_vec', 'hard_pos_min_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.7, legend_label = 'obs_lb_point')
  fig2.inverted_triangle ('time_vec', 'hard_pos_max_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'obs_ub_point')

  # 添加障碍物s-t bound
  for obs_id in data_obs_st.keys():
     if(obs_id != ''):
        fig2.line('obs_t', 'obs_s', source = data_obs_st[obs_id], line_width = 2, line_color = 'firebrick', line_dash = 'solid')

  # 添加目标轨迹
  fig2.line('t_follow_target', 's_follow_target', source = data_target, line_width = 3, line_color = 'darkred', line_dash = 'solid', legend_label = 's_follow_target')
  fig2.line('t_comfort_target', 's_comfort_target', source = data_target_s_comfort, line_width = 3, line_color = 'orange', line_dash = 'solid', legend_label = 's_comfort_target')
  fig2.line('t_upper_bound', 's_upper_bound', source = data_target_s_comfort, line_width = 2, line_color = 'purple', line_dash = 'dotted', legend_label = 's_upper_bound')
  fig2.circle('t_upper_bound', 's_upper_bound', source = data_target_s_comfort, size = 4, color = 'purple', alpha = 0.6, legend_label = 's_upper_bound')
  fig2.line('t', 's_speed_adjust_target', source = data_st, line_width = 3, line_color = 'cyan', line_dash = 'dashdot', legend_label = 's_speed_adjust_target', visible=False)
  fig2.line('t', 's_lane_change_target', source = data_st, line_width = 3, line_color = 'lime', line_dash = 'dashdot', legend_label = 's_lane_change_target', visible=False)

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

    fig3.line(lower_t, lower_s, source=source, line_width=2, line_color='grey', line_dash='solid', legend_label='st_boundary')
    fig3.circle(lower_t, lower_s, source=source, size=3, color='grey', legend_label='st_boundary')
    fig3.line(upper_t, upper_s, source=source, line_width=2, line_color='grey', line_dash='solid', legend_label='st_boundary')
    fig3.circle(upper_t, upper_s, source=source, size=3, color='grey', legend_label='st_boundary')

  for i, source in enumerate(st_points):
    left_t = f'left_point_{i}_t'
    left_s = f'left_point_{i}_s'
    right_t = f'right_point_{i}_t'
    right_s = f'right_point_{i}_s'

    fig3.line(left_t, left_s, source=source, line_width=2, line_color='grey', line_dash='solid', legend_label='st_boundary')
    fig3.line(right_t, right_s, source=source, line_width=2, line_color='grey', line_dash='solid', legend_label='st_boundary')

  for i, source in enumerate(st_labels):
    center_point_s = f'center_point_{i}_s'
    center_point_t = f'center_point_{i}_t'
    agent_id = f'agent_{i}_id'

    fig3.text(center_point_t, center_point_s, text = agent_id ,source = source, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'st_boundary')

  f3 = fig3.line('t_search', 's_search', source = data_st_searcher, line_width = 3.0, line_color = 'green', line_dash = 'solid', legend_label = 's_search_path', visible=False)
  # Plot joint motion planner s on fig3
  fig3.line('time_vec', 's_vec', source = data_joint_motion_plan, line_width = 3, line_color = 'darkblue', line_dash = 'solid', legend_label = 's_joint_opt', visible=False)
  fig3.circle('expanded_nodes_t', 'expanded_nodes_s', source=data_st_search_nodes, size=4, color='purple', legend_label='expanded_nodes', visible=False)
  fig3.circle('history_cur_nodes_t', 'history_cur_nodes_s', source=data_st_search_history_cur_nodes, size=10, color='orange', alpha=0.4, legend_label='history_cur_nodes', visible=False)
  fig3.line('t_final_target', 's_final_target', source = data_target, line_width = 3, line_color = 'blue', alpha = 1, line_dash = 'solid', legend_label = 's_final_target')
  fig3.circle('t_final_target', 's_final_target', source=data_target, size=6, color='brown', legend_label='s_final_target')
  fig3.line('t_follow_target', 's_follow_target', source = data_target, line_width = 3.0, line_color = 'red', line_dash = 'solid', legend_label = 's_follow_target')
  fig3.line('t_cruise_target', 's_cruise_target', source = data_target, line_width = 3.0, line_color = 'grey', line_dash = 'solid', legend_label = 's_cruise_target')
  fig3.line('t_overtake_target', 's_overtake_target', source = data_target_s_overtake, line_width = 3.0, line_color = 'black', line_dash = 'solid', legend_label = 's_overtake_target', visible=False)
  fig3.line('t_caution_target', 's_caution_target', source = data_target_s_caution, line_width = 3.0, line_color = 'yellow', line_dash = 'solid', legend_label = 's_caution_target', visible=False)
  fig3.line('t_neighbor_target', 's_neighbor_target', source = data_target_s_neighbor, line_width = 3.0, line_color = 'cyan', line_dash = 'solid', legend_label = 's_neighbor_target')
  fig3.line('t_max_decel', 's_max_decel', source = data_target_s_max_decel, line_width = 3.0, line_color = 'magenta', line_dash = 'solid', legend_label = 's_max_decel', visible=False)
  fig3.line('t_comfort_target', 's_comfort_target', source = data_target_s_comfort, line_width = 3.0, line_color = 'orange', line_dash = 'solid', legend_label = 's_comfort_target')
  fig3.line('t_cross_vru_target', 's_cross_vru_target', source = data_target_s_cross_vru, line_width = 3.0, line_color = 'darkviolet', line_dash = 'solid', legend_label = 's_cross_vru_target')
  fig3.line('t', 's_speed_adjust_target', source = data_st, line_width = 3, line_color = 'cyan', line_dash = 'dashdot', legend_label = 's_speed_adjust_target', visible=False)
  fig3.line('t', 's_lane_change_target', source = data_st, line_width = 3, line_color = 'lime', line_dash = 'dashdot', legend_label = 's_lane_change_target', visible=False)

  fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
  fig3.legend.click_policy = 'hide'

  # fig8 bar chart
  fig8 = bkp.figure(x_axis_label='time', y_axis_label='cost', width=600, height=400, title="Cost Over Time")

  fig8.vbar(x='st_path_final_nodes_time', top='st_path_final_nodes_cost_yield', source=data_st_search_path_final_nodes_cost, width=0.2, alpha = 0.5, color="blue", legend_label="Cost Yield")
  fig8.vbar(x='st_path_final_nodes_time', top='st_path_final_nodes_cost_overtake', source=data_st_search_path_final_nodes_cost, width=0.2, alpha = 0.5, color="red", legend_label="Cost Overtake")
  fig8.vbar(x='st_path_final_nodes_time', top='st_path_final_nodes_cost_vel', source=data_st_search_path_final_nodes_cost, width=0.2, alpha = 0.5, color="green", legend_label="Cost Velocity")
  fig8.vbar(x='st_path_final_nodes_time', top='st_path_final_nodes_cost_accel', source=data_st_search_path_final_nodes_cost, width=0.2, alpha = 0.5, color="orange", legend_label="Cost Acceleration")
  fig8.vbar(x='st_path_final_nodes_time', top='st_path_final_nodes_cost_accel_sign_changed', source=data_st_search_path_final_nodes_cost, width=0.2, alpha = 0.5, color="purple", legend_label="Cost Accel Sign Change")
  fig8.vbar(x='st_path_final_nodes_time', top='st_path_final_nodes_cost_jerk', source=data_st_search_path_final_nodes_cost, width=0.2, alpha = 0.5, color="brown", legend_label="Cost Jerk")
  fig8.vbar(x='st_path_final_nodes_time', top='st_path_final_nodes_cost_length', source=data_st_search_path_final_nodes_cost, width=0.2, alpha = 0.5, color="pink", legend_label="Cost Length")
  fig8.vbar(x='st_path_final_nodes_time', top='st_path_final_nodes_total_cost', source=data_st_search_path_final_nodes_cost, width=0.2, alpha = 0.5 ,color="cyan", legend_label="Total Cost")
  fig8.vbar(x='st_path_final_nodes_time', top='st_path_final_nodes_g_cost', source=data_st_search_path_final_nodes_cost, width=0.2, alpha = 0.5, color="magenta", legend_label="G Cost")
  fig8.vbar(x='st_path_final_nodes_time', top='st_path_final_nodes_h_cost', source=data_st_search_path_final_nodes_cost, width=0.2, alpha = 0.5, color="grey", legend_label="H Cost")

  hover8 = HoverTool(tooltips=[('time', '@st_path_final_nodes_time'),
                               ('Cost Yield', '@st_path_final_nodes_cost_yield'),
                               ('Cost Overtake', '@st_path_final_nodes_cost_overtake'),
                               ('Cost Velocity', '@st_path_final_nodes_cost_vel'),
                               ('Cost Acceleration', '@st_path_final_nodes_cost_accel'),
                               ('Cost Accel Sign Change', '@st_path_final_nodes_cost_accel_sign_changed'),
                               ('Cost Jerk', '@st_path_final_nodes_cost_jerk'),
                               ('Cost Length', '@st_path_final_nodes_cost_length'),
                               ('Total Cost', '@st_path_final_nodes_total_cost'),
                               ('G Cost', '@st_path_final_nodes_g_cost'),
                               ('H Cost', '@st_path_final_nodes_h_cost')])
  fig8.add_tools(hover8)

  fig8.toolbar.active_scroll = fig8.select_one(WheelZoomTool)
  fig8.legend.click_policy = 'hide'


  # vel
  f5 = fig5.line('time_vec', 'ref_vel_vec', source = data_lon_motion_plan, line_width = 3, line_color = 'green', line_dash = 'solid', legend_label = 'v_ref')
  fig5.line('time_vec', 'vel_vec', source = data_lon_motion_plan, line_width = 3, line_color = 'blue', line_dash = 'solid', legend_label = 'v_plan')
  fig5.line('t_follow_target', 'v_follow_target', source = data_target, line_width = 2, line_color = 'red', line_dash = 'dashed', legend_label = 'v_follow_target')
  fig5.line('t_cruise_target', 'v_cruise_target', source = data_target, line_width = 2, line_color = 'purple', line_dash = 'dashed', legend_label = 'v_cruise_target')
  fig5.line('t_comfort_target', 'v_comfort_target', source = data_target_s_comfort, line_width = 2, line_color = 'orange', line_dash = 'dashed', legend_label = 'v_comfort_target')
  fig5.line('t_upper_bound', 'v_upper_bound', source = data_target_s_comfort, line_width = 2, line_color = 'magenta', line_dash = 'dotted', legend_label = 'v_upper_bound')
  fig5.circle('t_upper_bound', 'v_upper_bound', source = data_target_s_comfort, size = 4, color = 'magenta', alpha = 0.6, legend_label = 'v_upper_bound')
  fig5.line('time_vec', 'comfort_v_target_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'darkgreen', line_dash = 'dashdot', legend_label = 'v_comfort_target')
  fig5.line('t_max_decel', 'v_max_decel', source = data_target_s_max_decel, line_width = 3, line_color = 'magenta', line_dash = 'solid', legend_label = 'v_max_decel', visible=False)
  # Plot joint motion planner v on fig5
  fig5.line('time_vec', 'vel_vec', source = data_joint_motion_plan, line_width = 3, line_color = 'navy', line_dash = 'solid', legend_label = 'v_joint_opt', visible=False)

  # acc
  f6 = fig6.line('time_vec', 'acc_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'a_plan')
  fig6.line('t_comfort_target', 'a_comfort_target', source = data_target_s_comfort, line_width = 2, line_color = 'orange', line_dash = 'dashed', legend_label = 'a_comfort_target')
  fig6.line('t_upper_bound', 'a_upper_bound', source = data_target_s_comfort, line_width = 2, line_color = 'magenta', line_dash = 'dotted', legend_label = 'a_upper_bound')
  fig6.circle('t_upper_bound', 'a_upper_bound', source = data_target_s_comfort, size = 4, color = 'magenta', alpha = 0.6, legend_label = 'a_upper_bound')
  fig6.line('time_vec', 'acc_min_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'a_lb')
  fig6.triangle ('time_vec', 'acc_min_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'a_lb')
  fig6.line('time_vec', 'acc_max_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'a_ub')
  fig6.inverted_triangle ('time_vec', 'acc_max_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'a_ub')
  fig6.line('t_max_decel', 'a_max_decel', source = data_target_s_max_decel, line_width = 3, line_color = 'magenta', line_dash = 'solid', legend_label = 'a_max_decel', visible=False)
  # Plot joint motion planner a on fig6
  fig6.line('time_vec', 'acc_vec', source = data_joint_motion_plan, line_width = 3, line_color = 'darkred', line_dash = 'solid', legend_label = 'a_joint_opt', visible=False)

  # jerk
  f7 = fig7.line('time_vec', 'jerk_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'j_plan')
  fig7.line('time_vec', 'jerk_min_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'j_lb')
  fig7.triangle ('time_vec', 'jerk_min_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'j_lb')
  fig7.line('time_vec', 'jerk_max_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'j_ub')
  fig7.inverted_triangle ('time_vec', 'jerk_max_vec', source = data_lon_motion_plan, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'j_ub')
  fig7.line('time_vec', 'comfort_jerk_min_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'darkblue', line_dash = 'dashdot', legend_label = 'j_comfort_lb')
  fig7.line('t_max_decel', 'j_max_decel', source = data_target_s_max_decel, line_width = 3, line_color = 'magenta', line_dash = 'solid', legend_label = 'j_max_decel', visible=False)
  # Plot joint motion planner j on fig7
  fig7.line('time_vec', 'jerk_vec', source = data_joint_motion_plan, line_width = 3, line_color = 'darkviolet', line_dash = 'solid', legend_label = 'j_joint_opt', visible=False)

  hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('v_lb', '@vel_min_vec'), ('v_ref', '@ref_vel_vec'), ('v_plan', '@vel_vec'), ('v_ub', '@vel_max_vec')], mode='vline')
  hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time_vec'), ('a_lb', '@acc_min_vec'), ('a_plan', '@acc_vec'), ('a_ub', '@acc_max_vec')], mode='vline')
  hover7 = HoverTool(renderers=[f7], tooltips=[('time', '@time_vec'), ('j_lb', '@jerk_min_vec'), ('j_plan', '@jerk_vec'), ('j_ub', '@jerk_max_vec')], mode='vline')

  fig5.add_tools(hover5)
  fig6.add_tools(hover6)
  fig7.add_tools(hover7)

  fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
  fig2.legend.click_policy = 'hide'

  fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
  fig3.legend.click_policy = 'hide'

  fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
  fig5.legend.click_policy = 'hide'

  fig6.toolbar.active_scroll = fig6.select_one(WheelZoomTool)
  fig6.legend.click_policy = 'hide'

  fig7.toolbar.active_scroll = fig7.select_one(WheelZoomTool)
  fig7.legend.click_policy = 'hide'

  tab_st_search = DataTable(source=data_st_search_text, columns=st_search_columns, width=600, height=400)

  pan1 = Panel(child=row(column(fig2, fig3, tab_st_search), column(fig5, fig6, fig7, fig8)), title="Longtime")

  tab1 = DataTable(source=data_text, columns=columns, width=600, height=1200)

  pan2 = Panel(child=row(column(tab1), column(velocity_fig, acc_fig, jerk_fig, fig_fsm_state), column(cost_time_fig, cutin_fig, fig_replan_status,topic_latency_fig)), title="Realtime")

  pans = Tabs(tabs=[ pan1, pan2 ])

  return pans, lon_plan_data