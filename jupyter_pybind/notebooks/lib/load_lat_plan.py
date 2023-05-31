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

def update_lat_plan_data(fig1, bag_loader, bag_time, local_view_data, lat_plan_data):

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

    try:
      json_pos_x = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['ego_pos_x']
      json_pos_y = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['ego_pos_y']
      json_yaw = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['ego_pos_yaw']
      coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
    except:
      coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

  if bag_loader.plan_debug_msg['enable'] == True:
    ref_x, ref_y = coord_tf.global_to_local(bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.ref_x_vec, \
      bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.ref_y_vec)

    soft_upper_bound_x0_vec, soft_upper_bound_y0_vec = coord_tf.global_to_local(bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.soft_upper_bound_x0_vec, \
      bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.soft_upper_bound_y0_vec)

    soft_lower_bound_x0_vec, soft_lower_bound_y0_vec = coord_tf.global_to_local(bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.soft_lower_bound_x0_vec, \
      bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.soft_lower_bound_y0_vec)

    hard_upper_bound_x0_vec, hard_upper_bound_y0_vec = coord_tf.global_to_local(bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.hard_upper_bound_x0_vec, \
      bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.hard_upper_bound_y0_vec)

    hard_lower_bound_x0_vec, hard_lower_bound_y0_vec = coord_tf.global_to_local(bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.hard_lower_bound_x0_vec, \
      bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.hard_lower_bound_y0_vec)

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

    raw_refline_x, raw_refline_y = coord_tf.global_to_local(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['raw_refline_x_vec'], \
      bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['raw_refline_y_vec'])

    lat_plan_data['data_refline'].data.update({
      'raw_refline_x': raw_refline_x,
      'raw_refline_y': raw_refline_y,
    })


    x_vec, y_vec = coord_tf.global_to_local(bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_output.x_vec, \
        bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_output.y_vec)

    lat_plan_data['data_lat_motion_plan_output'].data.update({
      'x_vec': x_vec,
      'y_vec': y_vec,
    })

    # print('init_flag = ', bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['init_flag'])
    # print('replan_status = ', bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['replan_flag'])

  return local_view_data

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

  data_lat_motion_plan_output = ColumnDataSource(data = {'x_vec':[],
                                                         'y_vec':[],
                                                        # 'theta_vec':[],
                                                        # 'soft_upper_bound_y0_vec':[],
                                                        # 'delta_vec':[],
                                                        # 'omega_vec':[],
                                                        # 'omega_dot_vec':[],
                                                        # 'acc_vec':[],
                                                        # 'jerk_vec':[]
                                                        })

  lat_plan_data = {  'data_lat_motion_plan_input':data_lat_motion_plan_input, \
                     'data_lat_motion_plan_output':data_lat_motion_plan_output, \
                     'data_refline':data_refline, \
                     }

  # motion planning
  fig1.line('ref_y', 'ref_x', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path', visible=False)
  fig1.line('soft_upper_bound_y0_vec', 'soft_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = "#90EE90", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft bound', visible=False)
  fig1.line('soft_lower_bound_y0_vec', 'soft_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = "#90EE90", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft bound', visible=False)
  fig1.line('hard_upper_bound_y0_vec', 'hard_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = 'black', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard bound', visible=False)
  fig1.line('hard_lower_bound_y0_vec', 'hard_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = 'black', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard bound', visible=False)
  fig1.line('raw_refline_y', 'raw_refline_x', source = data_refline, line_width = 3, line_color = 'blue', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline', visible=False)
  fig1.line('y_vec', 'x_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.4, legend_label = 'plan path')

  return fig1, lat_plan_data