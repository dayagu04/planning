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

coord_tf = coord_transformer()

def update_tune_ref_path_data(bag_loader, bag_time, local_view_data, ref_path_data, g_is_display_enu = False):
  # get param
  g_is_display_enu = global_var.get_value('g_is_display_enu')
  is_match_planning = global_var.get_value('is_match_planning')
  is_bag_main = global_var.get_value('is_bag_main')
  is_new_loc = global_var.get_value('is_new_loc')
  is_enu_to_car = global_var.get_value('is_enu_to_car')
  is_vis_map = global_var.get_value('is_vis_map')
  is_vis_sdmap = global_var.get_value('is_vis_sdmap')
  car_type = global_var.get_value('car_type')
  car_xb, car_yb = load_car_params_patch(car_type)
  # get msg
  road_msg = find_nearest(bag_loader.road_msg, bag_time)
  loc_msg = local_view_data['data_msg']['loc_msg']
  plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
  plan_debug_json_msg = local_view_data['data_msg']['plan_debug_json_msg']

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

  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = loc_msg.position.position_boot.x
    cur_pos_yn = loc_msg.position.position_boot.y
    cur_yaw = loc_msg.orientation.euler_boot.yaw
    coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

  if bag_loader.plan_debug_msg['enable'] == True:
    raw_refline_xn, raw_refline_yn = plan_debug_json_msg['raw_refline_x_vec'], plan_debug_json_msg['raw_refline_y_vec']
    raw_refline_x, raw_refline_y = coord_tf.global_to_local(plan_debug_json_msg['raw_refline_x_vec'], plan_debug_json_msg['raw_refline_y_vec'])
    ref_path_data['data_refline'].data.update({
      'raw_refline_x': raw_refline_x,
      'raw_refline_y': raw_refline_y,
      'raw_refline_xn': raw_refline_xn,
      'raw_refline_yn': raw_refline_yn,
    })

    reference_path_smooth_info = plan_debug_msg.reference_path_smooth_info
    print("is_smooth_success: ", reference_path_smooth_info.is_smooth_success)
    print("smooth_refpath_points_cost: ",plan_debug_json_msg["smooth_refpath_points_cost"])
    raw_ref_path_x = []
    raw_ref_path_y = []
    raw_ref_path_xn = []
    raw_ref_path_yn = []
    raw_ref_path_x, raw_ref_path_y = coord_tf.global_to_local(reference_path_smooth_info.raw_x_vec, reference_path_smooth_info.raw_y_vec)
    raw_ref_path_xn, raw_ref_path_yn = reference_path_smooth_info.raw_x_vec, reference_path_smooth_info.raw_y_vec
    ref_path_data['data_raw_ref_path'].data.update({
      'raw_ref_path_x': raw_ref_path_x,
      'raw_ref_path_y': raw_ref_path_y,
      'raw_ref_path_xn': raw_ref_path_xn,
      'raw_ref_path_yn': raw_ref_path_yn,
    })

    smooth_ref_path_x = []
    smooth_ref_path_y = []
    smooth_ref_path_xn = []
    smooth_ref_path_yn = []
    smooth_ref_path_s = reference_path_smooth_info.smooth_s_vec
    smooth_ref_path_x, smooth_ref_path_y = coord_tf.global_to_local(reference_path_smooth_info.smooth_x_vec, reference_path_smooth_info.smooth_y_vec)
    smooth_ref_path_xn, smooth_ref_path_yn = reference_path_smooth_info.smooth_x_vec, reference_path_smooth_info.smooth_y_vec
    ref_path_data['data_smooth_ref_path'].data.update({
      'smooth_ref_path_x': smooth_ref_path_x,
      'smooth_ref_path_y': smooth_ref_path_y,
      'smooth_ref_path_xn': smooth_ref_path_xn,
      'smooth_ref_path_yn': smooth_ref_path_yn,
      'smooth_ref_path_s': smooth_ref_path_s,
    })

    lat_motion_plan_input = plan_debug_msg.lateral_motion_planning_input
    ref_x = []
    ref_y = []
    last_x_vec = []
    last_y_vec = []
    if g_is_display_enu:
      ref_x, ref_y = lat_motion_plan_input.ref_x_vec, lat_motion_plan_input.ref_y_vec
      last_x_vec, last_y_vec = lat_motion_plan_input.last_x_vec, lat_motion_plan_input.last_y_vec
    else:
      ref_x, ref_y = coord_tf.global_to_local(lat_motion_plan_input.ref_x_vec, lat_motion_plan_input.ref_y_vec)
      last_x_vec, last_y_vec = coord_tf.global_to_local(lat_motion_plan_input.last_x_vec, lat_motion_plan_input.last_y_vec)

    ref_path_data['data_lat_motion_plan_input'].data.update({
      'ref_x': ref_x,
      'ref_y': ref_y,
      'last_x_vec': last_x_vec,
      'last_y_vec': last_y_vec,
    })
    lat_motion_plan_output = plan_debug_msg.lateral_motion_planning_output
    time_vec = lat_motion_plan_output.time_vec
    ref_theta_deg_vec = []
    theta_deg_vec = []
    last_theta_deg_vec = []
    steer_deg_vec = []
    steer_dot_deg_vec = []
    acc_upper_bound = []
    acc_lower_bound = []
    jerk_upper_bound = []
    jerk_lower_bound = []
    steer_deg_upper_bound = []
    steer_deg_lower_bound = []
    steer_dot_deg_upper_bound = []
    steer_dot_deg_lower_bound = []
    for i in range(len(time_vec)):
      ref_theta_deg_vec.append(lat_motion_plan_input.ref_theta_vec[i] * 57.3)
      theta_deg_vec.append(lat_motion_plan_output.theta_vec[i] * 57.3)
      last_theta_deg_vec.append(lat_motion_plan_input.last_theta_vec[i] * 57.3)
      steer_deg_vec.append(lat_motion_plan_output.delta_vec[i] * 57.3 * 15.88)
      steer_dot_deg_vec.append(lat_motion_plan_output.omega_vec[i] * 57.3 * 15.88)
      acc_upper_bound.append(lat_motion_plan_input.acc_bound)
      acc_lower_bound.append(-lat_motion_plan_input.acc_bound)
      jerk_upper_bound.append(lat_motion_plan_input.jerk_bound)
      jerk_lower_bound.append(-lat_motion_plan_input.jerk_bound)
      try:
        ref_vel = lat_motion_plan_input.ref_vel
        delta_bound = lat_motion_plan_input.acc_bound / (lat_motion_plan_input.curv_factor * ref_vel * ref_vel)
        omega_bound = lat_motion_plan_input.jerk_bound / (lat_motion_plan_input.curv_factor * ref_vel * ref_vel)
        steer_dot_deg_upper_bound.append((omega_bound * 57.3 * 15.88))
        steer_dot_deg_lower_bound.append(-(omega_bound * 57.3 * 15.88))
        steer_deg_upper_bound.append((delta_bound * 57.3 * 15.88))
        steer_deg_lower_bound.append(-(delta_bound * 57.3 * 15.88))
      except:
        steer_dot_deg_upper_bound.append((400))
        steer_dot_deg_lower_bound.append(-(400))
        steer_deg_upper_bound.append((180.0))
        steer_deg_lower_bound.append(-(180.0))

    acc_vec = lat_motion_plan_output.acc_vec
    jerk_vec = lat_motion_plan_output.jerk_vec

    xn_vec, yn_vec = lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec
    x_vec, y_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)
    if g_is_display_enu:
      xn_vec, yn_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)
      x_vec, y_vec = lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec

    ref_path_data['data_lat_motion_plan_output'].data.update({
      'time_vec': time_vec,
      'x_vec': x_vec,
      'y_vec': y_vec,
      'xn_vec': xn_vec,
      'yn_vec': yn_vec,
      'ref_theta_deg_vec': ref_theta_deg_vec,
      'theta_deg_vec': theta_deg_vec,
      'last_theta_deg_vec': last_theta_deg_vec,
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

  ref_path_data['data_fus_refline'].data.update({
    'fus_refline_x': [],
    'fus_refline_y': [],
    'fus_refline_xn': [],
    'fus_refline_yn': [],
  })
  if bag_loader.road_msg['enable'] == True:
    reference_line_msg = road_msg.reference_line_msg
    reference_line_msg_size = road_msg.reference_line_msg_size
    for i in range(reference_line_msg_size):
      lane = reference_line_msg[i]
      relative_id = lane.relative_id
      if relative_id == 0:
        virtual_lane_refline_points = lane.lane_reference_line.virtual_lane_refline_points
        virtual_lane_refline_points_size = lane.lane_reference_line.virtual_lane_refline_points_size
        line_xn_vec = [virtual_lane_refline_points[j].local_point.x for j in range(virtual_lane_refline_points_size)]
        line_yn_vec = [virtual_lane_refline_points[j].local_point.y for j in range(virtual_lane_refline_points_size)]
        line_x_vec, line_y_vec = coord_tf.global_to_local(line_xn_vec, line_yn_vec)
        ref_path_data['data_fus_refline'].data.update({
          'fus_refline_x': line_x_vec,
          'fus_refline_y': line_y_vec,
          'fus_refline_xn': line_xn_vec,
          'fus_refline_yn': line_yn_vec,
        })

  return coord_tf

def load_smooth_time_cost(bag_loader):
  data_fig = ColumnDataSource(data ={
    'frame_num': [],
    'time_cost': [],
    'result': [],
    'cost_mean': [],
    'cost_p95': [],
    'cost_3sigma': []
  })

  frame_nums = []
  time_costs = []
  results = []
  x_start = 0
  x_end = 0
  cost_mean = 0
  cost_var = 0
  cost_3sigma = []
  cost_p95 = []
  if bag_loader.plan_debug_msg['enable'] == True:
    for i, plan_json_debug in enumerate(bag_loader.plan_debug_msg['json']):
      plan_debug_msg = bag_loader.plan_debug_msg['data'][i]
      frame_nums.append(plan_debug_msg.frame_info.frame_num)
      time_costs.append(0.001 * plan_json_debug['smooth_refpath_points_cost'])
      results.append(plan_debug_msg.reference_path_smooth_info.is_smooth_success)
    frame_num_0 = frame_nums[0]
    frame_nums = [frame_num - frame_num_0 for frame_num in frame_nums]
    x_start = frame_nums[0]
    x_end = frame_nums[-1]
    count = len(time_costs)
    cost_median = np.median(time_costs)
    mean = np.mean(time_costs)
    cost_mean = [mean] * count
    std = np.std(time_costs)
    cost_3sigma = [mean + 3 * std] * count
    cost_var = np.var(time_costs),
    cost_p95 = [np.percentile(time_costs, 95)] * count

  fig = bkp.figure(x_axis_label='frame_num', y_axis_label='time_cost / ms', x_range = [x_start, x_end], width=800, height=200)
  data_fig.data.update({
    'frame_num': frame_nums,
    'time_cost': time_costs,
    'result': results,
    'cost_mean': cost_mean,
    'cost_p95': cost_p95,
    'cost_3sigma': cost_3sigma
  })
  fig_t = fig.line('frame_num', 'time_cost', source = data_fig, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'time cost')
  fig.line('frame_num', 'result', source = data_fig, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'result')
  fig.line('frame_num', 'cost_mean', source = data_fig, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'cost mean')
  fig.line('frame_num', 'cost_p95', source = data_fig, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'cost 95%')
  fig.inverted_triangle('frame_num', 'cost_3sigma', source = data_fig, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'cost 3sigma')

  hover = HoverTool(renderers=[fig_t], tooltips=[('frame_num', '@frame_num'), ('time_cost', '@time_cost')], mode='vline')
  fig.add_tools(hover)
  fig.legend.click_policy = 'hide'
  fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
  return fig

def load_ref_path_figure(fig1):
  data_fus_refline = ColumnDataSource(data = {'fus_refline_x':[],
                                              'fus_refline_y':[],
                                              'fus_refline_xn':[],
                                              'fus_refline_yn':[]})

  data_raw_ref_path = ColumnDataSource(data = {'raw_ref_path_x':[],
                                               'raw_ref_path_y':[],
                                               'raw_ref_path_xn':[],
                                               'raw_ref_path_yn':[]})

  data_smooth_ref_path = ColumnDataSource(data = {'smooth_ref_path_x':[],
                                                  'smooth_ref_path_y':[],
                                                  'smooth_ref_path_xn':[],
                                                  'smooth_ref_path_yn':[],
                                                  'smooth_ref_path_s':[]})

  data_tune_smooth_ref_path = ColumnDataSource(data = {'smooth_ref_path_x':[],
                                                       'smooth_ref_path_y':[],
                                                       'smooth_ref_path_xn':[],
                                                       'smooth_ref_path_yn':[]})

  data_lat_motion_plan_input = ColumnDataSource(data = {'ref_x':[],
                                                        'ref_y':[],
                                                        'last_x_vec': [],
                                                        'last_y_vec': [],
                                                        })

  data_lat_motion_plan_output = ColumnDataSource(data = {'time_vec':[],
                                                         'x_vec':[],
                                                         'y_vec':[],
                                                         'xn_vec':[],
                                                         'yn_vec':[],
                                                         'ref_theta_deg_vec':[],
                                                         'theta_deg_vec':[],
                                                         'last_theta_deg_vec':[],
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
                                                         'steer_dot_deg_lower_bound': []
                                                        })

  data_refline = ColumnDataSource(data = {'raw_refline_x':[],
                                          'raw_refline_y':[],
                                          'raw_refline_xn':[],
                                          'raw_refline_yn':[]})

  ref_path_data = {'data_fus_refline':data_fus_refline,
                   'data_raw_ref_path':data_raw_ref_path,
                   'data_smooth_ref_path':data_smooth_ref_path,
                   'data_tune_smooth_ref_path': data_tune_smooth_ref_path,
                   'data_lat_motion_plan_input':data_lat_motion_plan_input,
                   'data_lat_motion_plan_output':data_lat_motion_plan_output,
                   'data_refline':data_refline
  }

  fig1.line('ref_y', 'ref_x', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path', visible=True)
  fig1.line('last_y_vec', 'last_x_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = 'brown', line_dash = 'solid', line_alpha = 0.35, legend_label = 'last path', visible=False)
  fig1.line('fus_refline_y', 'fus_refline_x', source = data_fus_refline, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.35, legend_label = 'fus refline', visible=True)
  fig1.circle('fus_refline_y', 'fus_refline_x', source = data_fus_refline, size = 6, line_width = 5, line_color = 'blue', line_alpha = 0.4, fill_color = 'blue', fill_alpha = 1.0, legend_label = 'fus refline', visible=True)
  fig1.line('raw_ref_path_y', 'raw_ref_path_x', source = data_raw_ref_path, line_width = 5, line_color = 'yellow', line_dash = 'solid', line_alpha = 0.35, legend_label = 'raw refline', visible=True)
  fig1.circle('raw_ref_path_y', 'raw_ref_path_x', source = data_raw_ref_path, size = 6, line_width = 5, line_color = 'yellow', line_alpha = 0.4, fill_color = 'yellow', fill_alpha = 1.0, legend_label = 'raw refline', visible=True)
  fig1.line('smooth_ref_path_y', 'smooth_ref_path_x', source = data_smooth_ref_path, line_width = 5, line_color = 'green', line_dash = 'solid', line_alpha = 0.35, legend_label = 'smooth refline', visible=True)
  fig1.circle('smooth_ref_path_y', 'smooth_ref_path_x', source = data_smooth_ref_path, size = 6, line_width = 5, line_color = 'green', line_alpha = 0.4, fill_color = 'green', fill_alpha = 1.0, legend_label = 'smooth refline', visible=True)
  fig1.line('smooth_ref_path_y', 'smooth_ref_path_x', source = data_tune_smooth_ref_path, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'tune smooth', visible=True)
  fig1.circle('smooth_ref_path_y', 'smooth_ref_path_x', source = data_tune_smooth_ref_path, size = 6, line_width = 5, line_color = 'red', line_alpha = 0.4, fill_color = 'red', fill_alpha = 1.0, legend_label = 'tune smooth', visible=True)
  fig1.line('raw_refline_y', 'raw_refline_x', source = data_refline, line_width = 3, line_color = 'blue', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'plan refline', visible=False)

  fig2 = bkp.figure(x_axis_label='time', y_axis_label='theta',x_range = [-0.1, 5.2], width=600, height=180)
  fig3 = bkp.figure(x_axis_label='time', y_axis_label='lat acc',x_range = fig2.x_range, width=600, height=160)
  fig4 = bkp.figure(x_axis_label='time', y_axis_label='lat jerk',x_range = fig2.x_range, width=600, height=160)
  fig5 = bkp.figure(x_axis_label='time', y_axis_label='steer',x_range = fig2.x_range, width=600, height=160)
  fig6 = bkp.figure(x_axis_label='time', y_axis_label='steer dot',x_range = fig2.x_range, width=600, height=160)

  f2 = fig2.line('time_vec', 'ref_theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'ref_theta')
  fig2.line('time_vec', 'theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin theta')
  fig2.line('time_vec', 'last_theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'brown', line_dash = 'solid', legend_label = 'last traj theta', visible=False)

  f3 = fig3.line('time_vec', 'acc_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin lat acc')
  fig3.line('time_vec', 'acc_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat acc corridor')
  fig3.line('time_vec', 'acc_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat acc corridor')
  fig3.triangle ('time_vec', 'acc_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat acc corridor')
  fig3.inverted_triangle ('time_vec', 'acc_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat acc corridor')

  f4 = fig4.line('time_vec', 'jerk_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin lat jerk')
  fig4.line('time_vec', 'jerk_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat jerk corridor')
  fig4.line('time_vec', 'jerk_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat jerk corridor')
  fig4.triangle ('time_vec', 'jerk_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat jerk corridor')
  fig4.inverted_triangle ('time_vec', 'jerk_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'lat jerk corridor')

  f5 = fig5.line('time_vec', 'steer_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin steer deg')
  fig5.line('time_vec', 'steer_deg_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer deg corridor')
  fig5.line('time_vec', 'steer_deg_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer deg corridor')
  fig5.triangle ('time_vec', 'steer_deg_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer deg corridor')
  fig5.inverted_triangle ('time_vec', 'steer_deg_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer deg corridor')

  f6 = fig6.line('time_vec', 'steer_dot_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'origin steer dot deg')
  fig6.line('time_vec', 'steer_dot_deg_upper_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer dot deg corridor')
  fig6.line('time_vec', 'steer_dot_deg_lower_bound', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer dot deg corridor')
  fig6.triangle ('time_vec', 'steer_dot_deg_lower_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer dot deg corridor')
  fig6.inverted_triangle ('time_vec', 'steer_dot_deg_upper_bound', source = data_lat_motion_plan_output, size = 5, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'steer dot deg corridor')

  hover2 = HoverTool(renderers=[f2], tooltips=[('time', '@time_vec'), ('ref_theta', '@ref_theta_deg_vec'), ('origin theta', '@theta_deg_vec'), ('tuned theta', '@theta_deg_vec_t'), ('next_ref_theta', '@next_ref_theta_deg_vec'), ('last_traj_theta', '@last_theta_deg_vec')], mode='vline')
  hover3 = HoverTool(renderers=[f3], tooltips=[('time', '@time_vec'), ('origin acc', '@acc_vec'), ('tuned acc', '@acc_vec_t'), ('|acc bound|', '@acc_upper_bound')], mode='vline')
  hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time_vec'), ('origin jerk', '@jerk_vec'), ('tuned jerk', '@jerk_vec_t'), ('|jerk bound|', '@jerk_upper_bound')], mode='vline')
  hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('origin steer', '@steer_deg_vec'), ('tuned steer', '@steer_deg_vec_t'), ('|steer deg bound|', '@steer_deg_upper_bound'), ('expected steer', '@expected_steer_deg_vec')], mode='vline')
  hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time_vec'), ('origin steer dot', '@steer_dot_deg_vec'), ('tuned steer dot', '@steer_dot_deg_vec_t'), ('|steer dot deg bound|', '@steer_dot_deg_upper_bound')], mode='vline')

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

  fig2.legend.click_policy = 'hide'
  fig3.legend.click_policy = 'hide'
  fig4.legend.click_policy = 'hide'
  fig5.legend.click_policy = 'hide'
  fig6.legend.click_policy = 'hide'
  return fig1, fig2, fig3, fig4, fig5, fig6, ref_path_data
