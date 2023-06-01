from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *

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
from cyber_record.record import Record

car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()

def update_lat_plan_data(bag_loader, bag_time, local_view_data, lat_plan_data):

  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']
  road_msg_idx = local_view_data['data_index']['road_msg_idx']
  fus_msg_idx = local_view_data['data_index']['fus_msg_idx']
  vs_msg_idx = local_view_data['data_index']['vs_msg_idx']
  plan_msg_idx = local_view_data['data_index']['plan_msg_idx']
  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
  pred_msg_idx = local_view_data['data_index']['pred_msg_idx']

  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].pose.local_position.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].pose.local_position.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].pose.euler_angles.yaw
    planning_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]

    try:
      json_pos_x = planning_json['ego_pos_x']
      json_pos_y = planning_json['ego_pos_y']
      json_yaw = planning_json['ego_pos_yaw']
      coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
    except:
      coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

  if bag_loader.plan_debug_msg['enable'] == True:
    lat_motion_plan_input = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input

    ref_x, ref_y = coord_tf.global_to_local(lat_motion_plan_input.ref_x_vec, \
      lat_motion_plan_input.ref_y_vec)

    soft_upper_bound_x0_vec, soft_upper_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.soft_upper_bound_x0_vec, \
      lat_motion_plan_input.soft_upper_bound_y0_vec)

    soft_lower_bound_x0_vec, soft_lower_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.soft_lower_bound_x0_vec, \
      lat_motion_plan_input.soft_lower_bound_y0_vec)

    hard_upper_bound_x0_vec, hard_upper_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.hard_upper_bound_x0_vec, \
      lat_motion_plan_input.hard_upper_bound_y0_vec)

    hard_lower_bound_x0_vec, hard_lower_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.hard_lower_bound_x0_vec, \
      lat_motion_plan_input.hard_lower_bound_y0_vec)

    lat_plan_data['data_lat_motion_plan_input'].data.update({
      'ref_x': ref_x,
      'ref_y': ref_y,

      'soft_upper_bound_x0_vec': soft_upper_bound_x0_vec,
      'soft_upper_bound_y0_vec': soft_upper_bound_y0_vec,
      'soft_lower_bound_x0_vec': soft_lower_bound_x0_vec,
      'soft_lower_bound_y0_vec': soft_lower_bound_y0_vec,

      'hard_upper_bound_x0_vec': hard_upper_bound_x0_vec,
      'hard_upper_bound_y0_vec': hard_upper_bound_y0_vec,
      'hard_lower_bound_x0_vec': hard_lower_bound_x0_vec,
      'hard_lower_bound_y0_vec': hard_lower_bound_y0_vec,
    })

    raw_refline_x, raw_refline_y = coord_tf.global_to_local(planning_json['raw_refline_x_vec'], \
      planning_json['raw_refline_y_vec'])

    lat_plan_data['data_refline'].data.update({
      'raw_refline_x': raw_refline_x,
      'raw_refline_y': raw_refline_y,
    })

    lat_motion_plan_output = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_output

    x_vec, y_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)
    time_vec = lat_motion_plan_output.time_vec

    ref_theta_deg_vec = []
    theta_deg_vec = []
    steer_deg_vec = []
    steer_dot_deg_vec =[]

    for i in range(len(time_vec)):
      ref_theta_deg_vec.append(lat_motion_plan_input.ref_theta_vec[i] * 57.3)
      theta_deg_vec.append(lat_motion_plan_output.theta_vec[i] * 57.3)
      steer_deg_vec.append(lat_motion_plan_output.delta_vec[i] * 57.3)
      steer_dot_deg_vec.append(lat_motion_plan_output.omega_vec[i] * 57.3)

    acc_vec = lat_motion_plan_output.acc_vec
    jerk_vec = lat_motion_plan_output.jerk_vec

    lat_plan_data['data_lat_motion_plan_output'].data.update({
      'time_vec': time_vec,
      'x_vec': x_vec,
      'y_vec': y_vec,
      'ref_theta_deg_vec': ref_theta_deg_vec,
      'theta_deg_vec': theta_deg_vec,
      'steer_deg_vec': steer_deg_vec,
      'steer_dot_deg_vec': steer_dot_deg_vec,
      'acc_vec': acc_vec,
      'jerk_vec': jerk_vec,
    })

  if bag_loader.plan_msg['enable'] == True:
    trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
    if trajectory.trajectory_type == 0:
      planning_polynomial = trajectory.target_reference.polynomial
      plan_traj_x, plan_traj_y = gen_line(planning_polynomial[3],planning_polynomial[2], planning_polynomial[1], planning_polynomial[0], 0, 50)

    else:
      plan_x = []
      plan_y = []
      for i in range(len(trajectory.trajectory_points)):
        plan_x.append(trajectory.trajectory_points[i].x)
        plan_y.append(trajectory.trajectory_points[i].y)

      plan_traj_x, plan_traj_y = coord_tf.global_to_local(plan_x, plan_y)

    lat_plan_data['data_planning'].data.update({
      'plan_traj_y' : plan_traj_y,
      'plan_traj_x' : plan_traj_x,
      })

    # print("init_state:", lat_motion_plan_input.init_state)
    # print("delta_vec:", lat_motion_plan_output.delta_vec)


def load_lat_plan_figure(fig1):
  data_refline = ColumnDataSource(data = {'raw_refline_x':[],
                                          'raw_refline_y':[],})

  data_lat_motion_plan_input = ColumnDataSource(data = {'ref_x':[],
                                                        'ref_y':[],
                                                        'soft_upper_bound_x0_vec':[],
                                                        'soft_upper_bound_y0_vec':[],
                                                        'soft_lower_bound_x0_vec':[],
                                                        'soft_lower_bound_y0_vec':[],
                                                        'hard_upper_bound_x0_vec':[],
                                                        'hard_upper_bound_y0_vec':[],
                                                        'hard_lower_bound_x0_vec':[],
                                                        'hard_lower_bound_y0_vec':[],
                                                        })

  data_lat_motion_plan_output = ColumnDataSource(data = {'time_vec':[],
                                                         'x_vec':[],
                                                         'y_vec':[],
                                                         'ref_theta_deg_vec':[],
                                                         'theta_deg_vec':[],
                                                         'steer_deg_vec':[],
                                                         'steer_dot_deg_vec':[],
                                                         'acc_vec':[],
                                                         'jerk_vec':[]
                                                        })
  data_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                    'plan_traj_x':[],})

  lat_plan_data = {'data_lat_motion_plan_input':data_lat_motion_plan_input,
                   'data_lat_motion_plan_output':data_lat_motion_plan_output,
                   'data_refline':data_refline,
                   'data_planning':data_planning,
  }



  # motion planning
  fig1.line('ref_y', 'ref_x', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path', visible=False)
  fig1.line('soft_upper_bound_y0_vec', 'soft_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = "#90EE90", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft bound', visible=False)
  fig1.line('soft_lower_bound_y0_vec', 'soft_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = "#90EE90", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft bound', visible=False)
  fig1.line('hard_upper_bound_y0_vec', 'hard_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = 'black', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard bound', visible=False)
  fig1.line('hard_lower_bound_y0_vec', 'hard_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = 'black', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard bound', visible=False)
  fig1.line('raw_refline_y', 'raw_refline_x', source = data_refline, line_width = 3, line_color = 'blue', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline', visible=False)
  fig1.line('y_vec', 'x_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'red', line_dash = 'dashed', line_alpha = 0.4, legend_label = 'plan path')

  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan_debug')

  fig2 = bkp.figure(x_axis_label='time', y_axis_label='theta',x_range = [-0.1, 5.2], width=600, height=160)
  fig3 = bkp.figure(x_axis_label='time', y_axis_label='lat acc',x_range = fig2.x_range, width=600, height=160)
  fig4 = bkp.figure(x_axis_label='time', y_axis_label='lat jerk',x_range = fig2.x_range, width=600, height=160)
  fig5 = bkp.figure(x_axis_label='time', y_axis_label='steer',x_range = fig2.x_range, width=600, height=160)
  fig6 = bkp.figure(x_axis_label='time', y_axis_label='steer dot',x_range = fig2.x_range, width=600, height=160)

  f2 = fig2.line('time_vec', 'ref_theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'ref_theta')
  fig2.line('time_vec', 'theta_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'theta')
  f3 = fig3.line('time_vec', 'acc_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat acc')
  f4 = fig4.line('time_vec', 'jerk_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat jerk')
  f5 = fig5.line('time_vec', 'steer_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer deg')
  f6 = fig6.line('time_vec', 'steer_dot_deg_vec', source = data_lat_motion_plan_output, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer dot deg')


  hover2 = HoverTool(renderers=[f2], tooltips=[('time', '@time_vec'), ('ref_theta', '@ref_theta_deg_vec'), ('theta', '@theta_deg_vec')], mode='vline')
  hover3 = HoverTool(renderers=[f3], tooltips=[('time', '@time_vec'), ('acc', '@acc_vec')], mode='vline')
  hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time_vec'), ('jerk', '@jerk_vec')], mode='vline')
  hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('steer', '@steer_deg_vec')], mode='vline')
  hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time_vec'), ('steer dot', '@steer_dot_deg_vec')], mode='vline')

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

  return fig1, fig2, fig3, fig4, fig5, fig6, lat_plan_data