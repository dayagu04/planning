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
class LoadCyberbag:
  def __init__(self, path) -> None:
    self.bag = Record(path)
    # loclization msg
    self.loc_msg = {'t':[], 'data':[], 'enable':[]}

    # road msg
    self.road_msg = {'t':[], 'data':[], 'enable':[]}

    # fusion object msg
    self.fus_msg = {'t':[], 'data':[], 'enable':[]}

    # vehicle service msg
    self.vs_msg = {'t':[], 'data':[], 'enable':[]}
    # car pos in local coordinates

    # planning msg
    self.plan_msg = {'t':[], 'data':[], 'enable':[]}

    # planning debug msg
    self.plan_debug_msg = {'t':[], 'data':[], 'json':[], 'enable':[]}

  def load_all_data(self):
    max_time = 0.0
    # load localization msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/localization/ego_pose"):
        # load timestamp
        self.loc_msg['t'].append(msg.header.timestamp / 1e6)
        self.loc_msg['data'].append(msg)
      self.loc_msg['t'] = [tmp - self.loc_msg['t'][0]  for tmp in self.loc_msg['t']]
      self.loc_msg['enable'] = True
      max_time = max(max_time, self.loc_msg['t'][-1])
    except:
      self.loc_msg['enable'] = False
      print('missing /iflytek/localization/ego_pose !!!')

    # load road_fusion msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/road_fusion"):
        self.road_msg['t'].append(msg.header.timestamp / 1e6)
        self.road_msg['data'].append(msg)
      self.road_msg['t'] = [tmp - self.road_msg['t'][0]  for tmp in self.road_msg['t']]
      self.road_msg['enable'] = True
    except:
      self.road_msg['enable'] = False
      print('missing /iflytek/fusion/road_fusion topic !!!')

    # load fusion objects msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/objects"):
        self.fus_msg['t'].append(msg.header.timestamp / 1e6)
        self.fus_msg['data'].append(msg)
      self.fus_msg['t'] = [tmp - self.fus_msg['t'][0]  for tmp in self.fus_msg['t']]
      self.fus_msg['enable'] = True
      if len(self.fus_msg['t']) > 0:
        self.fus_msg['enable'] = True
      else:
        self.fus_msg['enable'] = False
    except:
      self.fus_msg['enable'] = False
      print('missing /iflytek/fusion/objects !!!')

    # load vehicle service msg
    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/vehicle_service"):
        self.vs_msg['t'].append(msg.header.timestamp / 1e6)
        self.vs_msg['data'].append(msg)
      self.vs_msg['t'] = [tmp - self.vs_msg['t'][0]  for tmp in self.vs_msg['t']]
      self.vs_msg['enable'] = True
      # max_time = max(max_time, self.vs_msg['t'][-1])
    except:
      self.vs_msg['enable'] = False
      print("missing /iflytek/vehicle_service !!!")

    # load planning msg

    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/planning/plan"):
        self.plan_msg['t'].append(msg.meta.header.timestamp / 1e6)
        self.plan_msg['data'].append(msg)
      self.plan_msg['t'] = [tmp - self.plan_msg['t'][0]  for tmp in self.plan_msg['t']]
      if len(self.plan_msg['t']) > 0:
        self.plan_msg['enable'] = True
      else:
        self.plan_msg['enable'] = False
    except:
      self.plan_msg['enable'] = False

    # load planning debug msg
    for topic, msg, t in self.bag.read_messages("/iflytek/planning/debug_info"):
      self.plan_debug_msg['t'].append(msg.timestamp / 1e6)
      self.plan_debug_msg['data'].append(msg)
      try:
        json_struct = json.loads(msg.data_json, strict = False)
        json_data = {}
        LoadScalar(json_data, json_struct, "ego_pos_x")
        LoadScalar(json_data, json_struct, "ego_pos_y")
        LoadVector(json_data, json_struct, "raw_refline_x_vec")
        LoadVector(json_data, json_struct, "raw_refline_y_vec")

        self.plan_debug_msg['json'].append(json_data)
      except json.decoder.JSONDecodeError as jserr:
        print('except',jserr)

    self.plan_debug_msg['t'] = [tmp - self.plan_debug_msg['t'][0]  for tmp in self.plan_debug_msg['t']]
    self.plan_debug_msg['enable'] = True

    if len(self.plan_debug_msg['t']) > 0:
      self.plan_debug_msg['enable'] = True
    else:
      self.plan_debug_msg['enable'] = False
    # except:
    #   self.plan_debug_msg['enable'] = False
    #   print("missing /iflytek/planning/debug_info !!!")


    return max_time

def update_local_view_data(fig1, bag_loader, bag_time, local_view_data):

  ### step 1: 时间戳对齐
  loc_msg_idx = 0
  if bag_loader.loc_msg['enable'] == True:
    while bag_loader.loc_msg['t'][loc_msg_idx] <= bag_time and loc_msg_idx < (len(bag_loader.loc_msg['t'])-2):
        loc_msg_idx = loc_msg_idx + 1

  road_msg_idx = 0
  if bag_loader.road_msg['enable'] == True:
    while bag_loader.road_msg['t'][road_msg_idx] <= bag_time and road_msg_idx < (len(bag_loader.road_msg['t'])-2):
        road_msg_idx = road_msg_idx + 1

  fus_msg_idx = 0
  if bag_loader.fus_msg['enable'] == True:
    while bag_loader.fus_msg['t'][fus_msg_idx] <= bag_time and fus_msg_idx < (len(bag_loader.fus_msg['t'])-2):
        fus_msg_idx = fus_msg_idx + 1

  vs_msg_idx = 0
  if bag_loader.vs_msg['enable'] == True:
    while bag_loader.vs_msg['t'][vs_msg_idx] <= bag_time and vs_msg_idx < (len(bag_loader.vs_msg['t'])-2):
        vs_msg_idx = vs_msg_idx + 1

  plan_msg_idx = 0
  if bag_loader.plan_msg['enable'] == True:
    while bag_loader.plan_msg['t'][plan_msg_idx] <= bag_time and plan_msg_idx < (len(bag_loader.plan_msg['t'])-2):
        plan_msg_idx = plan_msg_idx + 1

  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx = plan_debug_msg_idx + 1

  ### step 2: 加载定位信息
  cur_pos_xn0 = 0
  cur_pos_yn0 = 0
  cur_pos_xn = 0
  cur_pos_yn = 0
  cur_yaw = 0
  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn0 = cur_pos_xn = bag_loader.loc_msg['data'][0].pose.local_position.x
    cur_pos_yn0 = cur_pos_yn = bag_loader.loc_msg['data'][0].pose.local_position.y
    # ego pos in local and global coordinates
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].pose.local_position.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].pose.local_position.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].pose.euler_angles.yaw

    coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

    ego_xb, ego_yb = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.loc_msg['data'])):
      pos_xn_i = bag_loader.loc_msg['data'][i].pose.local_position.x
      pos_yn_i = bag_loader.loc_msg['data'][i].pose.local_position.y

      ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

      ego_xb.append(ego_local_x)
      ego_yb.append(ego_local_y)

    local_view_data['data_ego'].data.update({
      'ego_xb': ego_xb,
      'ego_yb': ego_yb,
    })

    local_view_data['data_car'].data.update({
      'car_xb': car_xb,
      'car_yb': car_yb,
    })

    try:
      vel_ego =  bag_loader.loc_msg['data'][loc_msg_idx].linear_velocity_from_wheel
    except:
      vel_ego = bag_loader.vs_msg['data'][vs_msg_idx].vehicle_speed

    text_xn = cur_pos_xn - cur_pos_xn0 - 2.0
    text_yn = cur_pos_yn - cur_pos_yn0 + 2.0

    local_view_data['data_text'].data.update({
      'vel_ego_text': ['v={:.2f}'.format(round(vel_ego, 2))],
      'text_xn': [text_xn],
      'text_yn': [text_yn],
    })

  ### step 3: 加载车道线信息
  if bag_loader.road_msg['enable'] == True:
    # load lane info
    line_info_list = load_lane_lines(bag_loader.road_msg['data'][road_msg_idx].lanes)

    #
    # update lane info
    data_lane_dict = {
      0:local_view_data['data_lane_0'],
      1:local_view_data['data_lane_1'],
      2:local_view_data['data_lane_2'],
      3:local_view_data['data_lane_3'],
      4:local_view_data['data_lane_4'],
      5:local_view_data['data_lane_5'],
      6:local_view_data['data_lane_6'],
      7:local_view_data['data_lane_7'],
      8:local_view_data['data_lane_8'],
      9:local_view_data['data_lane_9'],
    }
    data_center_line_dict = {
      0:local_view_data['data_center_line_0'],
      1:local_view_data['data_center_line_1'],
      2:local_view_data['data_center_line_2'],
      3:local_view_data['data_center_line_3'],
      4:local_view_data['data_center_line_4'],
    }

    for i in range(10):
      try:
        if line_info_list[i]['type'] == 0 or \
          line_info_list[i]['type'] == 1 or \
          line_info_list[i]['type'] == 3 or \
          line_info_list[i]['type'] == 4:
          fig1.renderers[3 + i].glyph.line_dash = 'dashed'
        else:
          fig1.renderers[3 + i].glyph.line_dash = 'solid'
        data_lane = data_lane_dict[i]
        data_lane.data.update({
          'line_{}_x'.format(i): line_info_list[i]['line_x_vec'],
          'line_{}_y'.format(i): line_info_list[i]['line_y_vec'],
        })
      except:
        pass

    center_line_list = load_lane_center_lines(bag_loader.road_msg['data'][road_msg_idx].lanes)
    for i in range(5):
      try:
        if 0:
          data_center_line = data_center_line_dict[i]
          data_center_line.data.update({
            'center_line_{}_x'.format(i): center_line_list[i]['line_x_vec'],
            'center_line_{}_y'.format(i): center_line_list[i]['line_y_vec'],
          })
        else:
          data_center_line = data_center_line_dict[i]
          line_x_rel = []
          line_y_rel = []
          for index in range(len(center_line_list[i]['line_x_vec'])):
            pos_xn_i, pos_yn_i = center_line_list[i]['line_x_vec'][index], center_line_list[i]['line_y_vec'][index]
            # print(pos_xn_i, pos_yn_i)
            ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)
            line_x_rel.append(ego_local_x)
            line_y_rel.append(ego_local_y)
          center_line_list[i]['line_x_vec'] = line_x_rel
          center_line_list[i]['line_y_vec'] = line_y_rel
          data_center_line.data.update({
            'center_line_{}_x'.format(i): center_line_list[i]['line_x_vec'],
            'center_line_{}_y'.format(i): center_line_list[i]['line_y_vec'],
          })
      except:
        pass

  ### step 4: 加载障碍物信息
  # load fus_obj
  if bag_loader.fus_msg['enable'] == True:
    fusion_objects = bag_loader.fus_msg['data'][fus_msg_idx].fusion_object
    obstacles_info_all = load_obstacle_params(fusion_objects)
    # 加载自车坐标系下的数据
    if 1:
      for key in obstacles_info_all:
        obstacles_info = obstacles_info_all[key]
        if key == 1:
          local_view_data['data_fus_obj'].data.update({
            'obstacles_x_rel': obstacles_info['obstacles_x_rel'],
            'obstacles_y_rel': obstacles_info['obstacles_y_rel'],
            'pos_x_rel' : obstacles_info['pos_x_rel'],
            'pos_y_rel' : obstacles_info['pos_y_rel'],
            'obstacles_x': obstacles_info['obstacles_x'],
            'obstacles_y': obstacles_info['obstacles_y'],
            'pos_x' : obstacles_info['pos_x'],
            'pos_y' : obstacles_info['pos_y'],
            'obs_label' : obstacles_info['obs_label'],
          })
        else :
          local_view_data['data_snrd_obj'].data.update({
            'obstacles_x_rel': obstacles_info['obstacles_x_rel'],
            'obstacles_y_rel': obstacles_info['obstacles_y_rel'],
            'pos_x_rel' : obstacles_info['pos_x_rel'],
            'pos_y_rel' : obstacles_info['pos_y_rel'],
            'obstacles_x': obstacles_info['obstacles_x'],
            'obstacles_y': obstacles_info['obstacles_y'],
            'pos_x' : obstacles_info['pos_x'],
            'pos_y' : obstacles_info['pos_y'],
            'obs_label' : obstacles_info['obs_label'],
          })
    else :
      for key in obstacles_info_all:
        obstacles_info = obstacles_info_all[key]
        poses_x = obstacles_info['pos_x']
        poses_y = obstacles_info['pos_y']
        poses_x_rel = []
        poses_y_rel = []
        # print(poses_x)
        # print(cur_pos_xn, cur_pos_yn, cur_yaw)
        for index in range(len(poses_x)):
          pos_xn_i, pos_yn_i = poses_x[index], poses_y[index]
          ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)
          poses_x_rel.append(ego_local_x)
          poses_y_rel.append(ego_local_y)
        obstacles_info['pos_x_rel'] = poses_x_rel
        obstacles_info['pos_y_rel'] = poses_y_rel
        if key == 1:
          local_view_data['data_fus_obj'].data.update({
            'obstacles_x_rel': obstacles_info['obstacles_x_rel'],
            'obstacles_y_rel': obstacles_info['obstacles_y_rel'],
            'pos_x_rel' : obstacles_info['pos_x_rel'],
            'pos_y_rel' : obstacles_info['pos_y_rel'],
            'obstacles_x': obstacles_info['obstacles_x'],
            'obstacles_y': obstacles_info['obstacles_y'],
            'pos_x' : obstacles_info['pos_x'],
            'pos_y' : obstacles_info['pos_y'],
            'obs_label' : obstacles_info['obs_label'],
          })
        else :
          local_view_data['data_snrd_obj'].data.update({
            'obstacles_x_rel': obstacles_info['obstacles_x_rel'],
            'obstacles_y_rel': obstacles_info['obstacles_y_rel'],
            'pos_x_rel' : obstacles_info['pos_x_rel'],
            'pos_y_rel' : obstacles_info['pos_y_rel'],
            'obstacles_x': obstacles_info['obstacles_x'],
            'obstacles_y': obstacles_info['obstacles_y'],
            'pos_x' : obstacles_info['pos_x'],
            'pos_y' : obstacles_info['pos_y'],
            'obs_label' : obstacles_info['obs_label'],
          })

  ### step 3: 加载planning轨迹信息
  try:
    if bag_loader.plan_msg['enable'] == True:
      trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
      if trajectory.trajectory_type == 0:
        planning_polynomial = trajectory.target_reference.polynomial
        planning_x, planning_y = gen_line(planning_polynomial[3],planning_polynomial[2], planning_polynomial[1], planning_polynomial[0], 0, 50)
        local_view_data['data_planning_trajectory'].data.update({
          'planning_y' : planning_y,
          'planning_x' : planning_x,
        })
  except:
    pass

  # FBI WARNING
  json_pos_x = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['ego_pos_x']
  json_pos_y = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['ego_pos_y']
  coord_tf.set_info( json_pos_x, json_pos_y, cur_yaw)

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

  local_view_data['data_lat_motion_plan_input'].data.update({
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

  local_view_data['data_refline'].data.update({
    'raw_refline_x': raw_refline_x,
    'raw_refline_y': raw_refline_y,
  })


  x_vec, y_vec = coord_tf.global_to_local(bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_output.x_vec, \
      bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_output.y_vec)

  local_view_data['data_lat_motion_plan_output'].data.update({
    'x_vec': x_vec,
    'y_vec': y_vec,
  })


  return local_view_data

def load_local_view_figure():
  data_car = ColumnDataSource(data = {'car_yb':[], 'car_xb':[]})
  data_ego = ColumnDataSource(data = {'ego_yb':[], 'ego_xb':[]})
  data_text = ColumnDataSource(data = {'vel_ego_text':[]})
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
  data_fus_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obstacles_y_rel':[], 'obstacles_x_rel':[],
                                        'pos_y_rel':[], 'pos_x_rel':[],
                                        'obs_label':[]})
  data_snrd_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obstacles_y_rel':[], 'obstacles_x_rel':[],
                                        'pos_y_rel':[], 'pos_x_rel':[],
                                        'obs_label':[]})

  planning_data = ColumnDataSource(data = {'planning_y':[],
                                           'planning_x':[],})

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

  planning_data = ColumnDataSource(data = {'planning_y':[],
                                      'planning_x':[],})

  local_view_data = {'data_car':data_car, \
                     'data_ego':data_ego, \
                     'data_text':data_text, \
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
                     'data_fus_obj':data_fus_obj, \
                     'data_snrd_obj':data_snrd_obj, \
                     'data_planning_trajectory':planning_data,\
                     'data_lat_motion_plan_input':data_lat_motion_plan_input, \
                     'data_lat_motion_plan_output':data_lat_motion_plan_output, \
                     'data_refline':data_refline, \
                     }
  ### figures config

  fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=500, height=800, match_aspect = True, aspect_scale=1)
  fig1.x_range.flipped = True
  # figure plot
  f1 = fig1.patch('car_yb', 'car_xb', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
  fig1.line('ego_yb', 'ego_xb', source = data_ego, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig1.text(0.0, -2.0, text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt", legend_label = 'car')
  fig1.line('line_0_y', 'line_0_x', source = data_lane_0, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_1_y', 'line_1_x', source = data_lane_1, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_2_y', 'line_2_x', source = data_lane_2, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_3_y', 'line_3_x', source = data_lane_3, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_4_y', 'line_4_x', source = data_lane_4, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_5_y', 'line_5_x', source = data_lane_5, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_6_y', 'line_6_x', source = data_lane_6, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_7_y', 'line_7_x', source = data_lane_7, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_8_y', 'line_8_x', source = data_lane_8, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_9_y', 'line_9_x', source = data_lane_9, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')

  fig1.line('center_line_0_y', 'center_line_0_x', source = data_center_line_0, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig1.line('center_line_1_y', 'center_line_1_x', source = data_center_line_1, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig1.line('center_line_2_y', 'center_line_2_x', source = data_center_line_2, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig1.line('center_line_3_y', 'center_line_3_x', source = data_center_line_3, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig1.line('center_line_4_y', 'center_line_4_x', source = data_center_line_4, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')

  fig1.patches('obstacles_y_rel', 'obstacles_x_rel', source = data_fus_obj, fill_color = "gray", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'obj')
  fig1.patches('obstacles_y_rel', 'obstacles_x_rel', source = data_snrd_obj, fill_color = "black", line_color = "black", line_width = 1, fill_alpha = 0.5, legend_label = 'snrd')
  fig1.text('pos_y_rel', 'pos_x_rel', text = 'obs_label' ,source = data_fus_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'fusion_info')
  fig1.text('pos_y_rel', 'pos_x_rel', text = 'obs_label' ,source = data_snrd_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'snrd_info')
  fig1.line('planning_y', 'planning_x', source = planning_data, line_width = 3, line_color = 'pink', line_dash = 'solid', legend_label = 'planning_trajectory')

  # motion planning
  fig1.line('ref_y', 'ref_x', source = data_lat_motion_plan_input, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref path')
  fig1.line('soft_upper_bound_y0_vec', 'soft_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = "#FFA500", line_dash = 'solid', line_alpha = 0.35, legend_label = 'soft bound')
  fig1.line('soft_lower_bound_y0_vec', 'soft_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = "#FFA500", line_dash = 'solid', line_alpha = 0.35, legend_label = 'soft bound')
  fig1.line('hard_upper_bound_y0_vec', 'hard_upper_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = 'black', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard bound')
  fig1.line('hard_lower_bound_y0_vec', 'hard_lower_bound_x0_vec', source = data_lat_motion_plan_input, line_width = 5, line_color = 'black', line_dash = 'solid', line_alpha = 0.35, legend_label = 'hard bound')
  fig1.line('raw_refline_y', 'raw_refline_x', source = data_refline, line_width = 3, line_color = 'blue', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'raw refline')
  fig1.line('y_vec', 'x_vec', source = data_lat_motion_plan_output, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.4, legend_label = 'plan path')

  # toolbar
  fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

  # legend
  fig1.legend.click_policy = 'hide'
  return fig1, local_view_data