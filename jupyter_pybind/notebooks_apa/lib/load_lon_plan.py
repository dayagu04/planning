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
from cyber_record.record import Record
from lib.load_json import *
from lib.load_struct import *
from jupyter_pybind.python_proto import planning_debug_info_pb2

coord_tf = coord_transformer()

def update_lon_plan_offline_data(bag_loader, bag_time, local_view_data, lon_plan_data):
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']
  plan_msg_idx = local_view_data['data_index']['plan_msg_idx']
  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']

  planning_json_value_list = []

  plan_debug_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx]
  plan_debug_json_info = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]
  # behavior planning
  t_vec = list(plan_debug_info.long_ref_path.t_list)

  t_long_vec = []
  for item in (bag_loader.plan_msg['data'][plan_msg_idx].trajectory.trajectory_points):
    t_long_vec.append(item.t)
  s_plan_vec =  []
  for item in (bag_loader.plan_msg['data'][plan_msg_idx].trajectory.trajectory_points):
    s_plan_vec.append(item.distance)
  v_plan_vec =  []
  for item in (bag_loader.plan_msg['data'][plan_msg_idx].trajectory.trajectory_points):
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
  for i in range(len(planning_json_value_list)):
     vision_lon_attr_vec.append(plan_debug_json_info[planning_json_value_list[i]])

  # lon_plan_data['offline_data_st'].data.update({
  #   't': t_vec,
  #   's': s_ref_vec,
  #   's_soft_ub': s_soft_upper_bound_vec,
  #   's_soft_lb': s_soft_lower_bound_vec,
  #   'obs_low': obs_low_vec,
  #   'obs_high': obs_high_vec,
  #   'obs_low_id': obs_low_id_vec,
  #   'obs_high_id': obs_high_id_vec,
  #   'obs_low_type': obs_low_type_vec,
  #   'obs_high_type': obs_high_type_vec
  # })

  lon_plan_data['offline_st_curve'].data.update({
    't_long': t_long_vec,
    's_plan': s_plan_vec,
    'v_plan': v_plan_vec
  })

  lon_plan_data['offline_sv_curve'].data.update({
    's_ref': s_ref_vec,
    'v_ref': v_ref_vec,
    'sv_bound_s': sv_bound_s_vec,
    'sv_bound_v': sv_bound_v_vec,
  })

  lon_plan_data['data_text'].data.update({
    'VisionLonAttr': planning_json_value_list,
    'VisionLonVal': vision_lon_attr_vec
  })

  lon_plan_data['data_lon_motion_plan'].data.update({
    'time_vec': [],
    'ref_pos_vec_origin': [],
    'ref_pos_vec': [],
    'ref_vel_vec': [],
    'soft_pos_max_vec': [],
    'soft_pos_min_vec': [],
    'vel_max_vec': [],
    'vel_min_vec': [],
    'acc_max_vec': [],
    'acc_min_vec': [],
    'jerk_max_vec': [],
    'jerk_min_vec': [],
    'pos_vec': [],
    'vel_vec': [],
    'acc_vec': [],
    'jerk_vec': [],
  })

  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw
    planning_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]

    try:
      json_pos_x = planning_json['ego_pos_x']
      json_pos_y = planning_json['ego_pos_y']
      json_yaw = planning_json['ego_pos_yaw']
      coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
    except:
      coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

   #  coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

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

      # plan_traj_x, plan_traj_y = coord_tf.global_to_local(plan_x, plan_y)
      plan_traj_x, plan_traj_y = coord_tf.global_to_local(planning_json['traj_x_vec'], planning_json['traj_y_vec'])

    lon_plan_data['data_planning'].data.update({
      'plan_traj_y' : plan_traj_y,
      'plan_traj_x' : plan_traj_x,
      })

def update_lon_plan_online_data(dp_speed_constraints,qp_speed_constraints, ref_cruise_speed,dp_speed,qp_speed,lon_plan_data):

  # get dp sv_bound:
  sv_bound_s_vec = []
  sv_bound_v_vec = []
  obs_dist = []
  acc_upper = []
  acc_lower = []
  jerk_upper = []
  jerk_lower = []
  ref_v = []

  for i in range(len(dp_speed_constraints)):
     sv_bound_s_vec.append(dp_speed_constraints[i][0])
     sv_bound_v_vec.append(dp_speed_constraints[i][2])
     obs_dist.append(dp_speed_constraints[i][1])

     acc_upper.append(dp_speed_constraints[i][3])
     acc_lower.append(dp_speed_constraints[i][4])

     jerk_upper.append(dp_speed_constraints[i][5])
     jerk_lower.append(dp_speed_constraints[i][6])

     ref_v.append(ref_cruise_speed)

     print('s', dp_speed_constraints[i][0], 'obs dist',
           dp_speed_constraints[i][1], 'v upper', dp_speed_constraints[i][2],
           'ref_v', ref_v[i], 'acc upper', dp_speed_constraints[i][3],
           'jerk upper', dp_speed_constraints[i][5])

  lon_plan_data['data_s_vref'].data.update({
    's_ref': sv_bound_s_vec,
    'v_ref': ref_v,
  })

  lon_plan_data['dp_sv_bound'].data.update({
    'dp_sv_bound_s': sv_bound_s_vec,
    'dp_sv_bound_v': sv_bound_v_vec,
  })

  lon_plan_data['online_data_sobs'].data.update({
    'online_s': sv_bound_s_vec,
    'online_obs_dist': obs_dist,
  })

  lon_plan_data['data_s_acc_upper'].data.update({
    's': sv_bound_s_vec,
    'acc': acc_upper,
  })

  lon_plan_data['data_s_acc_lower'].data.update({
    's': sv_bound_s_vec,
    'acc': acc_lower,
  })

  # jerk
  lon_plan_data['data_s_jerk_upper'].data.update({
    's': sv_bound_s_vec,
    'jerk': jerk_upper,
  })
  lon_plan_data['data_s_jerk_lower'].data.update({
    's': sv_bound_s_vec,
    'jerk': jerk_lower,
  })

  # plot dp optimization data
  s =[]
  t =[]
  v =[]
  acc =[]
  jerk =[]
  for i in range(len(dp_speed)):
    s.append(dp_speed[i][0])
    t.append(dp_speed[i][1])
    v.append(dp_speed[i][2])

    acc.append(dp_speed[i][3])
    jerk.append(dp_speed[i][4])

  lon_plan_data['dp_data_sv'].data.update({
    'dp_sv_s': s,
    'dp_sv_v': v,
  })

  lon_plan_data['dp_data_s_acc'].data.update({
    's': s,
    'acc': acc,
  })

  lon_plan_data['dp_data_s_jerk'].data.update({
    's': s,
    'jerk': jerk,
  })

  lon_plan_data['dp_st_data'].data.update({
    't': t,
    's': s,
  })

  # get qp constraints
  sv_bound_s_vec = []
  sv_bound_v_vec = []
  for i in range(len(qp_speed_constraints)):
     sv_bound_s_vec.append(qp_speed_constraints[i][0])
     sv_bound_v_vec.append(qp_speed_constraints[i][1])

  lon_plan_data['qp_sv_bound'].data.update({
    's': sv_bound_s_vec,
    'v': sv_bound_v_vec,
  })

  # plot qp optimization data
  s =[]
  t =[]
  v =[]
  acc =[]
  jerk =[]
  for i in range(len(qp_speed)):
    s.append(qp_speed[i][0])
    t.append(qp_speed[i][1])
    v.append(qp_speed[i][2])

    acc.append(qp_speed[i][3])
    jerk.append(qp_speed[i][4])

  lon_plan_data['qp_data_sv'].data.update({
    'qp_sv_s': s,
    'qp_sv_v': v,
  })

  lon_plan_data['qp_data_s_acc'].data.update({
    's': s,
    'acc': acc,
  })

  lon_plan_data['qp_data_s_jerk'].data.update({
    's': s,
    'jerk': jerk,
  })

  lon_plan_data['qp_st_data'].data.update({
    't': t,
    's': s,
  })


# plot 离线所有帧数据
def load_lon_global_data_figure(bag_loader):
  #real time global figure data process
  velocity_fig = bkp.figure(title='车速',x_axis_label='time/s',
                y_axis_label='velocity/(m/s)',width=600,height=300)

  ego_velocity_vec = []
  target_velocity_vec = []
  # ref_velocity_vec = []
  leadone_velocity_vec = []
  leadtwo_velocity_vec = []
  t_plan_vec = bag_loader.plan_debug_msg['t']
  t_loc_vec = bag_loader.loc_msg['t']
  t_vehicle_service_vec = bag_loader.vs_msg['t']

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

  acc_fig = bkp.figure(title='加速度',x_axis_label='time/s',
                y_axis_label='acc/(m/s2)',width=600,height=300)

  ego_acc_vec = []
  acc_min_vec = []
  acc_max_vec = []

  t_vs_vec = bag_loader.vs_msg['t']

  acc_fig.line(t_plan_vec, acc_min_vec, line_width=1,
                              legend_label='acc_min', color="brown")
  acc_fig.line(t_vs_vec, ego_acc_vec, line_width=1,
                                legend_label='ego_acc',color="blue")
  acc_fig.line(t_plan_vec, acc_max_vec, line_width=1,
                              legend_label='acc_max', color="red")

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
  RealTimeLonBehaviorCostTime_vec = []
  RealTimeLonMotionCostTime_vec = []
  RealTimeLateralMotionCostTime_vec = []
  EnvironmentalModelManagerCost_vec = []
  GeneralPlannerModuleCostTime_vec = []

  lead_fig.line(t_plan_vec, desired_distance_rss_vec, line_width=1, legend_label='distance_rss', color="yellow")
  lead_fig.line(t_plan_vec, desired_distance_calibrate_vec, line_width=1, legend_label='distance_cali', color="orange")

  cost_time_fig.line(t_plan_vec, RealTimeLonBehaviorCostTime_vec, line_width=1, legend_label='LonBehaviorCostTime', color="red")
  cost_time_fig.line(t_plan_vec, RealTimeLonMotionCostTime_vec, line_width=1, legend_label='LonMotionCostTime', color="blue")
  cost_time_fig.line(t_plan_vec, RealTimeLateralMotionCostTime_vec, line_width=1, legend_label='LatMotionCostTime', color="orange")
  cost_time_fig.line(t_plan_vec, EnvironmentalModelManagerCost_vec, line_width=1, legend_label='EnvironmentalCostTime', color="yellow")
  cost_time_fig.line(t_plan_vec, GeneralPlannerModuleCostTime_vec, line_width=1, legend_label='GeneralPlannerModuleCostTime', color="purple")

  cutin_fig = bkp.figure(title='速度',x_axis_label='time/s', y_axis_label='velocity/(m/s)',width=600,height=300)

  limit_cutin_vel_vec = []
  potential_cutin_speed_vec = []
  cutin_status_vec = []

  cutin_fig.line(t_plan_vec, limit_cutin_vel_vec, line_width=1,
                                  legend_label='Pre-deceleration cutin',color="blue")
  cutin_fig.line(t_plan_vec, potential_cutin_speed_vec, line_width=1,
                                legend_label='Speed regulation cutin', color="red")

  return velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig

# offline data + online data
def create_lon_plan_figure(fig1, velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig):

  # online data
  data_s_vref = ColumnDataSource(data = {'s_ref':[], 'v_ref':[]})
  dp_data_s_acc = ColumnDataSource(data = {'s':[], 'acc':[]})
  qp_data_s_acc = ColumnDataSource(data = {'s':[], 'acc':[]})
  dp_st_data = ColumnDataSource(data = {'t':[], 's':[]})
  qp_st_data = ColumnDataSource(data = {'t':[], 's':[]})

  data_s_acc_upper = ColumnDataSource(data = {'s':[], 'acc':[]})
  data_s_acc_lower = ColumnDataSource(data = {'s':[], 'acc':[]})

  dp_data_s_jerk = ColumnDataSource(data = {'s':[], 'jerk':[]})
  qp_data_s_jerk = ColumnDataSource(data = {'s':[], 'jerk':[]})
  data_s_jerk_upper = ColumnDataSource(data = {'s':[], 'jerk':[]})
  data_s_jerk_lower = ColumnDataSource(data = {'s':[], 'jerk':[]})
  data_text = ColumnDataSource(data = {'VisionLonAttr':[], 'VisionLonVal':[]})

  dp_data_sv = ColumnDataSource(data = {'dp_sv_s':[], 'dp_sv_v':[]})
  qp_data_sv = ColumnDataSource(data = {'qp_sv_s':[], 'qp_sv_v':[]})
  dp_data_sv_bound = ColumnDataSource(data = {'dp_sv_bound_s':[], 'dp_sv_bound_v':[]})
  qp_data_sv_bound = ColumnDataSource(data = {'s':[], 'v':[]})
  online_data_sobs = ColumnDataSource(data = {'online_s':[], 'online_obs_dist':[]})

  # offline data
  offline_data_st = ColumnDataSource(data = {'t':[], 's':[], 's_soft_ub':[], 's_soft_lb':[], 'obs_low':[], 'obs_high':[], 'obs_low_id':[], 'obs_high_id':[], 'obs_low_type':[], 'obs_high_type':[]})
  offline_st_curve = ColumnDataSource(data = {'t_long':[], 's_plan':[], 'v_plan':[]})
  offline_sv_curve = ColumnDataSource(data = {'s_ref':[], 'v_ref':[], 'sv_bound_s':[], 'sv_bound_v':[]})
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

  lon_plan_data = {'data_text':data_text, \
                   'data_s_vref':data_s_vref, \
                   'dp_data_sv':dp_data_sv, \
                   'qp_data_sv':qp_data_sv, \
                   'dp_sv_bound':dp_data_sv_bound, \
                   'qp_sv_bound':qp_data_sv_bound, \
                   'dp_data_s_acc':dp_data_s_acc, \
                   'qp_data_s_acc':qp_data_s_acc, \
                   'data_s_acc_upper':data_s_acc_upper, \
                   'data_s_acc_lower':data_s_acc_lower, \
                   'dp_data_s_jerk':dp_data_s_jerk, \
                   'qp_data_s_jerk':qp_data_s_jerk, \
                   'data_s_jerk_upper':data_s_jerk_upper, \
                   'data_s_jerk_lower':data_s_jerk_lower, \
                   'data_lon_motion_plan': data_lon_motion_plan, \
                   'data_planning':data_planning,
                   'online_data_sobs':online_data_sobs,
                   'dp_st_data':dp_st_data,
                   'qp_st_data':qp_st_data,
                   'offline_data_st':offline_data_st,
                   'offline_st_curve':offline_st_curve,
                   'offline_sv_curve':offline_sv_curve,
  }

  columns = [
        TableColumn(field="VisionLonAttr", title="VisionLonAttr"),
        TableColumn(field="VisionLonVal", title="VisionLonVal"),
    ]
  hover = HoverTool(tooltips = [
     ('index','$index'),
     ('id_low','@obs_low_id'),
     ('id_high','@obs_high_id'),
     ('low_type','@obs_low_type'),
     ('high_type','@obs_high_type'),
  ])

  # fig2 S-T
  fig_s_time = bkp.figure(x_axis_label='t', y_axis_label='s',x_range = [-0.1, 10.0], y_range = [-0.1, 10.0], width=400, height=400, match_aspect = True, aspect_scale=1)

  # fig3 S-V
  fig_sv = bkp.figure(x_axis_label='s', y_axis_label='v',x_range = [-0.1, 6.0], y_range = [-0.1, 1.5], width=400, height=400, match_aspect = True, aspect_scale=1)
  # fig5 s-dist to obs
  fig_sobs = bkp.figure(x_axis_label='s', y_axis_label='dist',x_range = [-0.1, 6], y_range = [-0.1, 20.0], width=400, height=300)
  # fig6 a-s
  fig_as = bkp.figure(x_axis_label='s', y_axis_label='acc',x_range = [-0.1, 6], width=400, height=300)
  # fig7 j-s
  fig_js = bkp.figure(x_axis_label='s', y_axis_label='jerk',x_range = fig_as.x_range, width=400, height=300)

  # plot
  f3 = fig_sv.line('s_ref', 'v_ref', source = data_s_vref, line_width = 2, line_color = 'green', line_dash = 'solid', legend_label = 'v_ref')
  fig_sv.line('dp_sv_s', 'dp_sv_v', source = dp_data_sv, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'dp_sv')
  fig_sv.line('qp_sv_s', 'qp_sv_v', source = qp_data_sv, line_width = 2, line_color = 'purple', line_dash = 'solid', legend_label = 'qp_sv')
  fig_sv.line('dp_sv_bound_s', 'dp_sv_bound_v', source = dp_data_sv_bound, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'dp v bound')
  fig_sv.line('s', 'v', source = qp_data_sv_bound, line_width = 2, line_color = 'orange', line_dash = 'solid', legend_label = 'qp v bound')

  f2 = fig_s_time.line('t', 's', source = dp_st_data, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'dp_st')
  f2 = fig_s_time.line('t', 's', source = qp_st_data, line_width = 2, line_color = 'purple', line_dash = 'solid', legend_label = 'qp_st')

  # obs dist
  f5 = fig_sobs.line('online_s', 'online_obs_dist', source = online_data_sobs, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'obs dist')

  # acc
  f6 = fig_as.line('s', 'acc', source = data_s_acc_upper, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'acc_upper')
  f6 = fig_as.line('s', 'acc', source = data_s_acc_lower, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'acc_lower')
  f6 = fig_as.line('s', 'acc', source = dp_data_s_acc, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'dp_acc')
  f6 = fig_as.line('s', 'acc', source = qp_data_s_acc, line_width = 2, line_color = 'purple', line_dash = 'solid', legend_label = 'qp_acc')

  # jerk
  f7 = fig_js.line('s', 'jerk', source = data_s_jerk_upper, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'jerk_upper')
  f7 = fig_js.line('s', 'jerk', source=data_s_jerk_lower, line_width=2,line_color='red', line_dash='solid', legend_label='jerk_lower')

  f7 = fig_js.line('s', 'jerk', source = dp_data_s_jerk, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'dp_jerk')
  f7 = fig_js.line('s', 'jerk', source =qp_data_s_jerk, line_width = 2, line_color = 'purple', line_dash = 'solid', legend_label = 'qp_jerk')


  # offline data
  # f2 = fig_s_time.line('t', 's', source = offline_data_st, line_width = 2, line_color = 'green', line_dash = 'dashed', legend_label = 'origin s_ref')
  # fig_s_time.line('t', 's_soft_ub', source = offline_data_st, line_width = 3, line_color = 'yellow', line_dash = 'solid', legend_label = 's_soft_ub')
  # fig_s_time.line('t', 's_soft_lb', source = offline_data_st, line_width = 3, line_color = '#FFA500', line_dash = 'solid', legend_label = 's_soft_lb')
  # fig_s_time.line('time_vec', 'ref_pos_vec', source = data_lon_motion_plan, line_width = 2.5, line_color = 'red', line_dash = 'dashed', legend_label = 's_ref')
  # fig_s_time.line('time_vec', 'pos_vec', source = data_lon_motion_plan, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 's_plan')
  # fig_s_time.line('t', 'obs_low', source = offline_data_st, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'obs_lb')
  # fig_s_time.line('t', 'obs_high', source = offline_data_st, line_width = 2, line_color = 'grey', line_dash = 'solid', legend_label = 'obs_ub')
  # fig_s_time.triangle('t', 'obs_low', source = offline_data_st, size = 10, fill_color='grey', line_color='grey', alpha = 0.7, legend_label = 'obs_lb_point')
  # fig_s_time.inverted_triangle ('t', 'obs_high', source = offline_data_st, size = 10, fill_color='grey', line_color='grey', alpha = 0.5, legend_label = 'obs_ub_point')

  f3 = fig_sv.line('s_ref', 'v_ref', source = offline_sv_curve, line_width = 2, line_color = 'green', line_dash = 'dashed', legend_label = 'v_ref')
  fig_sv.line('sv_bound_s', 'sv_bound_v', source = offline_sv_curve, line_width = 2, line_color = 'orange', line_dash = 'solid', legend_label = 'offline_v_upper')

  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan debug', visible=False)

  hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('v_lb', '@vel_min_vec'), ('v_ref', '@ref_vel_vec'), ('v_plan', '@vel_vec'), ('v_ub', '@vel_max_vec')], mode='vline')
  hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time_vec'), ('a_lb', '@acc_min_vec'), ('a_plan', '@acc_vec'), ('a_ub', '@acc_max_vec')], mode='vline')
  hover7 = HoverTool(renderers=[f7], tooltips=[('time', '@time_vec'), ('j_lb', '@jerk_min_vec'), ('j_plan', '@jerk_vec'), ('j_ub', '@jerk_max_vec')], mode='vline')

  fig_sobs.add_tools(hover5)
  fig_as.add_tools(hover6)
  fig_js.add_tools(hover7)

  fig_sv.toolbar.active_scroll = fig_sv.select_one(WheelZoomTool)
  fig_sv.legend.click_policy = 'hide'

  fig_sobs.toolbar.active_scroll = fig_sobs.select_one(WheelZoomTool)
  fig_sobs.legend.click_policy = 'hide'

  fig_as.toolbar.active_scroll = fig_as.select_one(WheelZoomTool)
  fig_as.legend.click_policy = 'hide'

  fig_js.toolbar.active_scroll = fig_js.select_one(WheelZoomTool)
  fig_js.legend.click_policy = 'hide'

  fig_s_time.toolbar.active_scroll = fig_s_time.select_one(WheelZoomTool)
  fig_s_time.legend.click_policy = 'hide'

  pan1 = Panel(child=row(column(fig_sv, fig_s_time), column(fig_sobs, fig_as, fig_js)), title="online simulation")

  tab1 = DataTable(source=data_text, columns=columns, width=500, height=200)
  pan2 = Panel(child=row(tab1, column(velocity_fig, acc_fig, lead_fig), column(cost_time_fig, cutin_fig)), title="Real car data curve")

  pans = Tabs(tabs=[ pan1, pan2 ])

  return pans, lon_plan_data

# online data
def create_online_lon_plan_figure(fig1):
  data_s_vref = ColumnDataSource(data = {'s_ref':[], 'v_ref':[]})
  dp_data_s_acc = ColumnDataSource(data = {'s':[], 'acc':[]})
  qp_data_s_acc = ColumnDataSource(data = {'s':[], 'acc':[]})
  dp_st_data = ColumnDataSource(data = {'t':[], 's':[]})
  qp_st_data = ColumnDataSource(data = {'t':[], 's':[]})

  data_s_acc_upper = ColumnDataSource(data = {'s':[], 'acc':[]})
  data_s_acc_lower = ColumnDataSource(data = {'s':[], 'acc':[]})

  dp_data_s_jerk = ColumnDataSource(data = {'s':[], 'jerk':[]})
  qp_data_s_jerk = ColumnDataSource(data = {'s':[], 'jerk':[]})
  data_s_jerk_upper = ColumnDataSource(data = {'s':[], 'jerk':[]})
  data_s_jerk_lower = ColumnDataSource(data = {'s':[], 'jerk':[]})
  data_text = ColumnDataSource(data = {'VisionLonAttr':[], 'VisionLonVal':[]})

  dp_data_sv = ColumnDataSource(data = {'dp_sv_s':[], 'dp_sv_v':[]})
  qp_data_sv = ColumnDataSource(data = {'qp_sv_s':[], 'qp_sv_v':[]})
  dp_data_sv_bound = ColumnDataSource(data = {'dp_sv_bound_s':[], 'dp_sv_bound_v':[]})
  qp_data_sv_bound = ColumnDataSource(data = {'s':[], 'v':[]})
  online_data_sobs = ColumnDataSource(data = {'online_s':[], 'online_obs_dist':[]})


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

  lon_plan_data = {'data_text':data_text, \
                   'data_s_vref':data_s_vref, \
                   'dp_data_sv':dp_data_sv, \
                   'qp_data_sv':qp_data_sv, \
                   'dp_sv_bound':dp_data_sv_bound, \
                   'qp_sv_bound':qp_data_sv_bound, \
                   'dp_data_s_acc':dp_data_s_acc, \
                   'qp_data_s_acc':qp_data_s_acc, \
                   'data_s_acc_upper':data_s_acc_upper, \
                   'data_s_acc_lower':data_s_acc_lower, \
                   'dp_data_s_jerk':dp_data_s_jerk, \
                   'qp_data_s_jerk':qp_data_s_jerk, \
                   'data_s_jerk_upper':data_s_jerk_upper, \
                   'data_s_jerk_lower':data_s_jerk_lower, \
                   'data_lon_motion_plan': data_lon_motion_plan, \
                   'data_planning':data_planning,
                   'online_data_sobs':online_data_sobs,
                   'dp_st_data':dp_st_data,
                   'qp_st_data':qp_st_data,
  }

  columns = [
        TableColumn(field="VisionLonAttr", title="VisionLonAttr"),
        TableColumn(field="VisionLonVal", title="VisionLonVal"),
    ]
  hover = HoverTool(tooltips = [
     ('index','$index'),
     ('id_low','@obs_low_id'),
     ('id_high','@obs_high_id'),
     ('low_type','@obs_low_type'),
     ('high_type','@obs_high_type'),
  ])

  # fig2 S-T
  fig_s_time = bkp.figure(x_axis_label='t', y_axis_label='s',x_range = [-0.1, 10.0], y_range = [-0.1, 10.0], width=400, height=200, match_aspect = True, aspect_scale=1)

  # fig3 S-V
  fig_sv = bkp.figure(x_axis_label='s', y_axis_label='v',x_range = [-0.1, 6.0], y_range = [-0.1, 1.5], width=400, height=400, match_aspect = True, aspect_scale=1)
  # fig5 s-dist to obs
  fig_sobs = bkp.figure(x_axis_label='s', y_axis_label='dist',x_range = [-0.1, 6], y_range = [-0.1, 20.0], width=400, height=200)
  # fig6 a-s
  fig_as = bkp.figure(x_axis_label='s', y_axis_label='acc',x_range = [-0.1, 6], width=400, height=200)
  # fig7 j-s
  fig_js = bkp.figure(x_axis_label='s', y_axis_label='jerk',x_range = [-0.1, 6], width=400, height=200)

  # plot
  f3 = fig_sv.line('s_ref', 'v_ref', source = data_s_vref, line_width = 2, line_color = 'green', line_dash = 'solid', legend_label = 'v_ref')
  fig_sv.line('dp_sv_s', 'dp_sv_v', source = dp_data_sv, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'dp_sv')
  fig_sv.line('qp_sv_s', 'qp_sv_v', source = qp_data_sv, line_width = 2, line_color = 'purple', line_dash = 'solid', legend_label = 'qp_sv')
  fig_sv.line('dp_sv_bound_s', 'dp_sv_bound_v', source = dp_data_sv_bound, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'dp v bound')
  fig_sv.line('s', 'v', source = qp_data_sv_bound, line_width = 2, line_color = 'orange', line_dash = 'solid', legend_label = 'qp v bound')

  f2 = fig_s_time.line('t', 's', source = dp_st_data, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'dp_st')
  f2 = fig_s_time.line('t', 's', source = qp_st_data, line_width = 2, line_color = 'purple', line_dash = 'solid', legend_label = 'qp_st')

  # obs dist
  f5 = fig_sobs.line('online_s', 'online_obs_dist', source = online_data_sobs, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'obs dist')

  # acc
  f6 = fig_as.line('s', 'acc', source = dp_data_s_acc, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'dp_acc')
  f6 = fig_as.line('s', 'acc', source = qp_data_s_acc, line_width = 2, line_color = 'purple', line_dash = 'solid', legend_label = 'qp_acc')
  f6 = fig_as.line('s', 'acc', source = data_s_acc_upper, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'acc_upper')
  f6 = fig_as.line('s', 'acc', source = data_s_acc_lower, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'acc_lower')

  # jerk
  f7 = fig_js.line('s', 'jerk', source = dp_data_s_jerk, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'dp_jerk')
  f7 = fig_js.line('s', 'jerk', source =qp_data_s_jerk, line_width = 2, line_color = 'purple', line_dash = 'solid', legend_label = 'qp_jerk')
  f7 = fig_js.line('s', 'jerk', source = data_s_jerk_upper, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'jerk_upper')
  f7 = fig_js.line('s', 'jerk', source=data_s_jerk_lower, line_width=2,
                   line_color='red', line_dash='solid', legend_label='jerk_lower')

  hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('v_lb', '@vel_min_vec'), ('v_ref', '@ref_vel_vec'), ('v_plan', '@vel_vec'), ('v_ub', '@vel_max_vec')], mode='vline')
  hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time_vec'), ('a_lb', '@acc_min_vec'), ('a_plan', '@acc_vec'), ('a_ub', '@acc_max_vec')], mode='vline')
  hover7 = HoverTool(renderers=[f7], tooltips=[('time', '@time_vec'), ('j_lb', '@jerk_min_vec'), ('j_plan', '@jerk_vec'), ('j_ub', '@jerk_max_vec')], mode='vline')

  fig_sobs.add_tools(hover5)
  fig_as.add_tools(hover6)
  fig_js.add_tools(hover7)

  fig_sv.toolbar.active_scroll = fig_sv.select_one(WheelZoomTool)
  fig_sv.legend.click_policy = 'hide'

  fig_sobs.toolbar.active_scroll = fig_sobs.select_one(WheelZoomTool)
  fig_sobs.legend.click_policy = 'hide'

  fig_as.toolbar.active_scroll = fig_as.select_one(WheelZoomTool)
  fig_as.legend.click_policy = 'hide'

  fig_js.toolbar.active_scroll = fig_js.select_one(WheelZoomTool)
  fig_js.legend.click_policy = 'hide'

  fig_s_time.toolbar.active_scroll = fig_s_time.select_one(WheelZoomTool)
  fig_s_time.legend.click_policy = 'hide'

  pan1 = Panel(child=row(column(fig_sv,fig_s_time), column(fig_sobs, fig_as, fig_js)), title="online simulation")

  tab1 = DataTable(source=data_text, columns=columns, width=500, height=800)

  pans = Tabs(tabs=[ pan1])

  return pans, lon_plan_data



