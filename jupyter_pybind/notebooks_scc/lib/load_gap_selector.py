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

def load_local_lane_lines(lanes):
  line_info_list = []

  for i in range(10):
    lane_info_l = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
    if i< len(lanes):
      lane = lanes[i]
      left_line = lane.left_lane_boundary
      enu_points = left_line.enu_points
      line_x = []
      line_y = []
      for index in range(enu_points.num):
        line_x.append(enu_points.points[index].x)
        line_y.append(enu_points.points[index].y)
      lane_info_l['line_x_vec'] = line_x
      lane_info_l['line_y_vec'] = line_y
      lane_info_l['type'] = left_line.segment[0].type

      line_info_list.append(lane_info_l)

      lane_info_r = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
      right_line = lane.right_lane_boundary
      enu_points = right_line.enu_points
      line_x = []
      line_y = []
      for index in range(enu_points.num):
        line_x.append(enu_points.points[index].x)
        line_y.append(enu_points.points[index].y)

      lane_info_r['line_x_vec'] = line_x
      lane_info_r['line_y_vec'] = line_y
      lane_info_r['type'] = right_line.segment[0].type
      line_info_list.append(lane_info_r)
    else:
      line_x = []
      line_y = []
      # line_x, line_y = gen_line(0,0,0,0,0,0)
      lane_info_l['line_x_vec'] = line_x
      lane_info_l['line_y_vec'] = line_y
      lane_info_l['type'] = []
      line_info_list.append(lane_info_l)

  return line_info_list

def update_lat_plan_data_with_gap_selector(bag_loader, bag_time, local_view_data, lat_plan_data, coord_info):

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

    ego_xn, ego_yn = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.loc_msg['data'])):
      pos_xn_i = bag_loader.loc_msg['data'][i].pose.local_position.x
      pos_yn_i = bag_loader.loc_msg['data'][i].pose.local_position.y

      ego_xn.append(pos_xn_i)
      ego_yn.append(pos_yn_i)

    lat_plan_data['data_ego'].data.update({
      'ego_xn': ego_xn,
      'ego_yn': ego_yn,
    })

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

    try:
      if(use_close_loop > 0.5):
        cur_pos_xn = coord_info[0]
        cur_pos_yn = coord_info[1]
        cur_yaw = coord_info[2]
      else:
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
    })

    # raw_refline_x, raw_refline_y = coord_tf.global_to_local(planning_json['raw_refline_x_vec'], \
    #   planning_json['raw_refline_y_vec'])

    # lat_plan_data['data_refline'].data.update({
    #   'raw_refline_x': raw_refline_x,
    #   'raw_refline_y': raw_refline_y,
    # })

    lat_motion_plan_output = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_output # 加载 plan_debug_msg中的信息

    x_vec, y_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)  # 将全局坐标系下的点 转移到 局部系
    time_vec = lat_motion_plan_output.time_vec

    ref_theta_deg_vec = []
    theta_deg_vec = []
    steer_deg_vec = []
    steer_dot_deg_vec =[]

    for i in range(len(time_vec)):  # 获取需要展示的planning 数据
      ref_theta_deg_vec.append(lat_motion_plan_input.ref_theta_vec[i] * 57.3)
      theta_deg_vec.append(lat_motion_plan_output.theta_vec[i] * 57.3)
      steer_deg_vec.append(lat_motion_plan_output.delta_vec[i] * 57.3 * 15.7)
      steer_dot_deg_vec.append(lat_motion_plan_output.omega_vec[i] * 57.3 * 15.7)

    acc_vec = lat_motion_plan_output.acc_vec
    jerk_vec = lat_motion_plan_output.jerk_vec

    lat_plan_data['data_lat_motion_plan_output'].data.update({ #更新 lat_plan_data中的数据
      'time_vec': time_vec,
      'x_vec': x_vec,
      'y_vec': y_vec,
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

    print("dbw_status = ", planning_json['dbw_status'])
    print("replan_status = ", planning_json['replan_status'])
    print("lat_err = ", planning_json['lat_err'])
    print("lon_err = ", planning_json['lon_err'])
    print("dist_err = ", planning_json['dist_err'])
    print("solver_condition = ", planning_json['solver_condition'])

  if bag_loader.plan_msg['enable'] == True:
    trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
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
      lat_plan_data['data_planning_n'].data.update({
        'plan_traj_xn':plan_x,
        'plan_traj_yn':plan_y,
       })

    lat_plan_data['data_planning'].data.update({
      'plan_traj_y' : plan_traj_y,
      'plan_traj_x' : plan_traj_x,
    })


  ### step 3: 加载车道线信息
  if bag_loader.road_msg['enable'] == True:
    # load lane info
    line_info_list = load_local_lane_lines(bag_loader.road_msg['data'][road_msg_idx].lanes)
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
        data_lane = data_lane_dict[i]
        data_lane.data.update({
          'line_{}_x'.format(i): line_info_list[i]['line_x_vec'],
          'line_{}_y'.format(i): line_info_list[i]['line_y_vec'],
        })
      except:
        print('error')
        pass

    center_line_list = load_lane_center_lines(bag_loader.road_msg['data'][road_msg_idx].lanes)
    # print(center_line_list)
    for i in range(5):
        data_center_line = data_center_line_dict[i]
        data_center_line.data.update({
          'center_line_{}_x'.format(i): center_line_list[i]['line_x_vec'],
          'center_line_{}_y'.format(i): center_line_list[i]['line_y_vec'],
        })
  # 加载prediction_msg
  if bag_loader.prediction_msg['enable'] == True:
    try:
      prediction_info = bag_loader.prediction_msg['data'][pred_msg_idx]
      # 定位的选择需要修改
      localization_info = bag_loader.loc_msg['data'][loc_msg_idx]
      prediction_objects, trajectory_info = load_prediction_objects(prediction_info.prediction_obstacle_list, localization_info)
      trajectory_info_x = trajectory_info['x']
      trajectory_info_y = trajectory_info['y']


    except:
      pass

  #------------- add raw sample path traj points -------------------------
  try:
    if bag_loader.plan_msg['sample_flag'] == True:
      x_vec=[]
      y_vec=[]
      for i in range(len(planning_json['quintic_spline_points_1_x'])):
        x_tmp, y_tmp = coord_tf.global_to_local(planning_json['quintic_spline_points_1_x'][i], planning_json['quintic_spline_points_1_y'][i])
        x_vec.append(x_tmp)
        y_vec.append(y_tmp)
      lat_plan_data['data_raw_sample_traj_0'].data.update({
          'x_vec':x_vec,
          'y_vec':y_vec,
      })
      for i in range(len(planning_json['quintic_spline_points_2_x'])):
        x_tmp, y_tmp = coord_tf.global_to_local(planning_json['quintic_spline_points_2_x'][i], planning_json['quintic_spline_points_2_y'][i])
        x_vec.append(x_tmp)
        y_vec.append(y_tmp)
      lat_plan_data['data_raw_sample_traj_1'].data.update({
          'x_vec':x_vec,
          'y_vec':y_vec,
      })
      for i in range(len(planning_json['quintic_spline_points_3_x'])):
        x_tmp, y_tmp = coord_tf.global_to_local(planning_json['quintic_spline_points_3_x'][i], planning_json['quintic_spline_points_3_y'][i])
        x_vec.append(x_tmp)
        y_vec.append(y_tmp)
      lat_plan_data['data_raw_sample_traj_2'].data.update({
          'x_vec':x_vec,
          'y_vec':y_vec,
      })
  except:
    pass

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
                                                        })

  data_lat_motion_plan_output = ColumnDataSource(data = {'time_vec':[],
                                                         'x_vec':[],
                                                         'y_vec':[],
                                                         'xn_vec':[],
                                                         'yn_vec':[],
                                                         'ref_theta_deg_vec':[],
                                                         'theta_deg_vec':[],
                                                         'steer_deg_vec':[],
                                                         'steer_dot_deg_vec':[],
                                                         'acc_vec':[],
                                                         'jerk_vec':[]
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


  data_sample_candidate_0 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  data_sample_candidate_1 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  data_sample_candidate_2 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  data_sample_candidate_3 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  data_sample_candidate_4 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  data_sample_candidate_5 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  data_raw_sample_traj_0 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  data_raw_sample_traj_1 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  data_raw_sample_traj_2 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  gs_traj_points = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  gs_pre_trajs = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  # data_raw_sample_traj_1 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  # data_raw_sample_traj_2 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

  data_gap_selector_vt = ColumnDataSource(data = {'t_vec':[], 'v_vec':[]})
  data_gap_selector_at = ColumnDataSource(data = {'t_vec':[], 'a_vec':[]})

  lat_plan_data = {'data_lat_motion_plan_input':data_lat_motion_plan_input,
                   'data_lat_motion_plan_output':data_lat_motion_plan_output,
                   'data_refline':data_refline,
                   'data_planning':data_planning,
                   'data_planning_n': data_planning_n,
                   'data_ego': data_ego,
                   'data_car': data_car,

                   'data_raw_sample_traj_0': data_raw_sample_traj_0, \
                   'data_raw_sample_traj_1': data_raw_sample_traj_1, \
                   'data_raw_sample_traj_2': data_raw_sample_traj_2, \
                   'gs_traj_points': gs_traj_points, \
                   'gs_pre_trajs':gs_pre_trajs,\
                  #  'data_raw_sample_traj_1': data_raw_sample_traj_1, \
                  #  'data_raw_sample_traj_2': data_raw_sample_traj_2, \
                  'data_gap_selector_vt':data_gap_selector_vt,\
                  'data_gap_selector_at':data_gap_selector_at,\

  }

  # motion planning
  #fig1.line('ref_y', 'ref_x', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path', visible=False)
  #fig1.line('soft_upper_bound_y0_vec', 'soft_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = "#90EE90", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft bound', visible=False)
  #fig1.line('soft_lower_bound_y0_vec', 'soft_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = "#90EE90", line_dash = 'solid', line_alpha = 0.7, legend_label = 'soft bound', visible=False)
  #fig1.line('hard_upper_bound_y0_vec', 'hard_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = 'black', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard bound', visible=False)
  #fig1.line('hard_lower_bound_y0_vec', 'hard_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = 'black', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard bound', visible=False)
  #fig1.line('raw_refline_y', 'raw_refline_x', source = data_refline, line_width = 3, line_color = 'blue', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline', visible=False)
  #fig1.line('y_vec', 'x_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'red', line_dash = 'dashed', line_alpha = 0.4, legend_label = 'plan path')

  #fig1.multi_line('y_vec', 'x_vec', source = gs_pre_trajs, line_width = 5, line_color = 'purple', line_dash = 'dashed', line_alpha = 0.4, legend_label = 'gs_pre_trajs')

  x_vec = [1, 2, 3]
  y_vec = [1, 2, 3]

  # try:
  #   fig1.line('y_vec', 'x_vec', source = data_raw_sample_traj_0, line_width = 5, line_color = 'yellow', line_dash = 'dashed', line_alpha = 0.9, legend_label = 'data_raw_sample_traj_0', visible=False)
  # except:
  #   pass

  # try:
  #   fig1.line('y_vec', 'x_vec', source = data_raw_sample_traj_1, line_width = 5, line_color = 'yellow', line_dash = 'dashed', line_alpha = 0.6, legend_label = 'data_raw_sample_traj_1', visible=False)
  # except:
  #   pass

  # try:
  #   fig1.line('y_vec', 'x_vec', source = data_raw_sample_traj_2, line_width = 5, line_color = 'yellow', line_dash = 'dashed', line_alpha = 0.3, legend_label = 'data_raw_sample_traj_2', visible=False)
  # except:
  #   pass

  # try:
  #   fig1.circle('y_vec', 'x_vec', source = gs_traj_points, color="blue", legend_label = 'gs_traj_points' )
  # except:
  #  pass

  fig2 = bkp.figure(x_axis_label='time', y_axis_label='theta',x_range = [-0.1, 5.2], width=600, height=160)
  fig3 = bkp.figure(x_axis_label='time', y_axis_label='lat acc',x_range = fig2.x_range, width=600, height=160)
  fig4 = bkp.figure(x_axis_label='time', y_axis_label='lat jerk',x_range = fig2.x_range, width=600, height=160)
  fig5 = bkp.figure(x_axis_label='time', y_axis_label='steer',x_range = fig2.x_range, width=600, height=160)
  fig6 = bkp.figure(x_axis_label='time', y_axis_label='steer dot',x_range = fig2.x_range, width=600, height=160)

  fig10 = bkp.figure(x_axis_label='t', y_axis_label='a',x_range = fig2.x_range, width=600, height=180)
  try:
    fig10.circle('t_vec', 'a_vec', source = data_gap_selector_at, color="blue", legend_label = 'data_gap_selector_at' )
  except:
    pass

  fig11 = bkp.figure(x_axis_label='t', y_axis_label='a',x_range = fig2.x_range, width=600, height=180)
  try:
    fig11.circle('t_vec', 'v_vec', source = data_gap_selector_vt, color="blue", legend_label = 'data_gap_selector_vt' )
  except:
    pass

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

  fig10.toolbar.active_scroll = fig10.select_one(WheelZoomTool)
  fig11.toolbar.active_scroll = fig11.select_one(WheelZoomTool)

  fig2.legend.click_policy = 'hide'
  fig3.legend.click_policy = 'hide'
  fig4.legend.click_policy = 'hide'
  fig5.legend.click_policy = 'hide'
  fig6.legend.click_policy = 'hide'

  fig10.legend.click_policy = 'hide'
  fig11.legend.click_policy = 'hide'

  return fig1, fig2, fig3, fig4, fig5, fig6, fig10, fig11, lat_plan_data