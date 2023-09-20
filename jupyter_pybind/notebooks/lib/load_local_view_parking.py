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
from bokeh.models import WheelZoomTool, HoverTool, TapTool, CustomJS
from cyber_record.record import Record
from google.protobuf.json_format import MessageToJson

car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()
max_slot_num = 20
class LoadCyberbag:
  def __init__(self, path, parking_flag = False) -> None:
    self.bag_path = path
    self.bag = Record(path)
    # loclization msg
    self.loc_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # fusion object msg
    self.fus_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # vehicle service msg
    self.vs_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}
    # car pos in local coordinates

    # planning msg
    self.plan_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # planning debug msg
    self.plan_debug_msg = {'abs_t':[], 't':[], 'data':[], 'json':[], 'enable':[]}

    # control msg
    self.ctrl_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # control debug msg
    self.ctrl_debug_msg = {'abs_t':[], 't':[], 'data':[], 'json':[], 'enable':[]}

    # parking fusion msg
    self.fus_parking_msg = {'abs_t':[], 't':[], 'data':[], 'json':[], 'enable':[]}

    # visual slot msg
    self.vis_parking_msg = {'abs_t':[], 't':[], 'data':[], 'json':[], 'enable':[]}

    # soc state machine
    self.soc_state_msg = {'abs_t':[], 't':[], 'data':[], 'json':[], 'enable':[]}

    self.parking_flag = parking_flag

    self.max_time = 0
    # time offset
    t0 = 0

  def load_all_data(self):
    max_time = 0.0
    # load localization msg
    try:
      loc_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/localization/ego_pose"):
        loc_msg_dict[msg.header.timestamp / 1e6] = msg
      loc_msg_dict = {key: val for key, val in sorted(loc_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in loc_msg_dict.items():
        self.loc_msg['t'].append(t)
        self.loc_msg['abs_t'].append(t)
        self.loc_msg['data'].append(msg)
      t0 = self.loc_msg['t'][0]
      self.loc_msg['t'] = [tmp - t0  for tmp in self.loc_msg['t']]
      max_time = max(max_time, self.loc_msg['t'][-1])
      print('loc_msg time:',self.loc_msg['t'][-1])
      if len(self.loc_msg['t']) > 0:
        self.loc_msg['enable'] = True
      else:
        self.loc_msg['enable'] = False
    except:
      self.loc_msg['enable'] = False
      print('missing /iflytek/localization/ego_pose !!!')

    # load fusion objects msg
    if self.parking_flag == False:
      try:
        fus_msg_dict = {}
        for topic, msg, t in self.bag.read_messages("/iflytek/fusion/objects"):
          fus_msg_dict[msg.header.timestamp / 1e6] = msg
        fus_msg_dict = {key: val for key, val in sorted(fus_msg_dict.items(), key = lambda ele: ele[0])}
        for t, msg in fus_msg_dict.items():
          self.fus_msg['t'].append(t)
          self.fus_msg['abs_t'].append(t)
          self.fus_msg['data'].append(msg)
        self.fus_msg['t'] = [tmp - t0  for tmp in self.fus_msg['t']]
        print('fus_msg time:',self.fus_msg['t'][-1])
        if len(self.fus_msg['t']) > 0:
          self.fus_msg['enable'] = True
        else:
          self.fus_msg['enable'] = False
      except:
        self.fus_msg['enable'] = False
        print('missing /iflytek/fusion/objects !!!')

    # load vehicle service msg
    try:
      vs_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/vehicle_service"):
        vs_msg_dict[msg.header.timestamp / 1e6] = msg
      vs_msg_dict = {key: val for key, val in sorted(vs_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in vs_msg_dict.items():
        self.vs_msg['t'].append(t)
        self.vs_msg['abs_t'].append(t)
        self.vs_msg['data'].append(msg)
      self.vs_msg['t'] = [tmp - t0  for tmp in self.vs_msg['t']]
      self.vs_msg['enable'] = True
      print('vs time:',self.vs_msg['t'][-1])
      max_time = max(max_time, self.vs_msg['t'][-1])
      if len(self.vs_msg['t']) > 0:
        self.vs_msg['enable'] = True
      else:
        self.vs_msg['enable'] = False
    except:
      self.vs_msg['enable'] = False
      print("missing /iflytek/vehicle_service !!!")

    # load planning msg
    try:
      plan_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/planning/plan"):
        plan_msg_dict[msg.meta.header.timestamp / 1e6] = msg
      plan_msg_dict = {key: val for key, val in sorted(plan_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in plan_msg_dict.items():
        self.plan_msg['t'].append(t)
        self.plan_msg['abs_t'].append(t)
        self.plan_msg['data'].append(msg)


      t0_plan = self.plan_msg['t'][0]
      self.plan_msg['t'] = [tmp - t0_plan  for tmp in self.plan_msg['t']]
      max_time = max(max_time, self.plan_msg['t'][-1])
      print('plan_msg time:',self.plan_msg['t'][-1])
      if len(self.plan_msg['t']) > 0:
        self.plan_msg['enable'] = True
      else:
        self.plan_msg['enable'] = False
    except:
      self.plan_msg['enable'] = False
      print("missing /iflytek/planning/plan !!!")

    # load planning debug msg
    try:
      json_value_list = ["replan_status", "ego_pos_x"]

      json_vector_list = ["raw_refline_x_vec", "raw_refline_y_vec", "assembled_delta", "assembled_omega", "traj_x_vec", "traj_y_vec"]

      plan_debug_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/planning/debug_info"):
        plan_debug_msg_dict[msg.timestamp / 1e6] = msg
      plan_debug_msg_dict = {key: val for key, val in sorted(plan_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in plan_debug_msg_dict.items():
        self.plan_debug_msg['t'].append(t)
        self.plan_debug_msg['abs_t'].append(t)
        self.plan_debug_msg['data'].append(msg)
        try:
          json_struct = json.loads(msg.data_json, strict = False)
          json_data = {}
          LoadScalarList(json_data, json_value_list, json_struct)
          LoadVectorList(json_data, json_vector_list, json_struct)

          self.plan_debug_msg['json'].append(json_data)
        except json.decoder.JSONDecodeError as jserr:
          print('except',jserr)

      t0_plan_debug = self.plan_debug_msg['t'][0]
      self.plan_debug_msg['t'] = [tmp - t0_plan_debug  for tmp in self.plan_debug_msg['t']]
      max_time = max(max_time, self.plan_debug_msg['t'][-1])
      print('plan_debug_msg time:',self.plan_debug_msg['t'][-1])
      if len(self.plan_debug_msg['t']) > 0:
        self.plan_debug_msg['enable'] = True
      else:
        self.plan_debug_msg['enable'] = False
    except:
      self.plan_debug_msg['enable'] = False
      print("missing /iflytek/planning/debug_info !!!")


    # load control msg
    try:
      ctrl_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/control/control_command"):
        ctrl_msg_dict[msg.header.timestamp / 1e6] = msg
      ctrl_msg_dict = {key: val for key, val in sorted(ctrl_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ctrl_msg_dict.items():
        self.ctrl_msg['t'].append(t)
        self.ctrl_msg['abs_t'].append(t)
        self.ctrl_msg['data'].append(msg)
      self.ctrl_msg['t'] = [tmp - self.ctrl_msg['t'][0]  for tmp in self.ctrl_msg['t']]
      max_time = max(max_time, self.ctrl_msg['t'][-1])
      print('ctrl_msg time:',self.ctrl_msg['t'][-1])
      if len(self.ctrl_msg['t']) > 0:
        self.ctrl_msg['enable'] = True
      else:
        self.ctrl_msg['enable'] = False
    except:
      self.ctrl_msg['enable'] = False
      print("missing /iflytek/control/control_command !!!")


    # load control debug msg
    try:
      json_value_list = ["steer_angle_cmd"]

      json_vector_list = ["dx_ref_mpc_vec", "dy_ref_mpc_vec", "dphi_ref_mpc_vec", "dx_mpc_vec", "dy_mpc_vec", "delta_mpc_vec", "dphi_mpc_vec"]

      ctrl_debug_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/control/debug_info"):
        ctrl_debug_msg_dict[msg.timestamp / 1e6] = msg
      ctrl_debug_msg_dict = {key: val for key, val in sorted(ctrl_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ctrl_debug_msg_dict.items():
        self.ctrl_debug_msg['t'].append(t)
        self.ctrl_debug_msg['abs_t'].append(t)
        self.ctrl_debug_msg['data'].append(msg)
        try:
          json_struct = json.loads(msg.extra_json, strict = False)
          json_data = {}
          LoadScalarList(json_data, json_value_list, json_struct)
          LoadVectorList(json_data, json_vector_list, json_struct)

          self.ctrl_debug_msg['json'].append(json_data)
        except json.decoder.JSONDecodeError as jserr:
          print('except',jserr)

      self.ctrl_debug_msg['t'] = [tmp - self.ctrl_debug_msg['t'][0]  for tmp in self.ctrl_debug_msg['t']]
      max_time = max(max_time, self.ctrl_debug_msg['t'][-1])
      print('ctrl_debug_msg time:',self.ctrl_debug_msg['t'][-1])
      if len(self.ctrl_debug_msg['t']) > 0:
        self.ctrl_debug_msg['enable'] = True
      else:
        self.ctrl_debug_msg['enable'] = False
    except:
      self.ctrl_debug_msg['enable'] = False
      print("missing /iflytek/control/debug_info !!!")

    # load parking fusion msg
    try:
      fus_parking_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/parking_slot"):
        fus_parking_msg_dict[msg.header.timestamp / 1e6] = msg
      fus_parking_msg_dict = {key: val for key, val in sorted(fus_parking_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in fus_parking_msg_dict.items():
        self.fus_parking_msg['t'].append(t)
        self.fus_parking_msg['abs_t'].append(t)
        self.fus_parking_msg['data'].append(msg)
      if (abs(self.fus_parking_msg['t'][0]) < 0.0001):
        self.fus_parking_msg['t'] = [tmp - self.fus_parking_msg['t'][1]  for tmp in self.fus_parking_msg['t']]
      else:
        self.fus_parking_msg['t'] = [tmp - self.fus_parking_msg['t'][0]  for tmp in self.fus_parking_msg['t']]
      max_time = max(max_time, self.fus_parking_msg['t'][-1])
      print('fus_parking_msg time:',self.fus_parking_msg['t'][-1])
      if len(self.fus_parking_msg['t']) > 0:
        self.fus_parking_msg['enable'] = True
      else:
        self.fus_parking_msg['enable'] = False
    except:
      self.fus_parking_msg['enable'] = False
      print('missing /iflytek/fusion/parking_slot !!!')


    # load visual parking msg
    try:
      vis_parking_msg_dict = {}
      # origin visual parking slot proto
      # for topic, msg, t in self.bag.read_messages("/parking_slot"):
      # new visula parking slot proto
      for topic, msg, t in self.bag.read_messages("/iflytek/camera_perception/parking_slot_list"):
        vis_parking_msg_dict[msg.header.timestamp / 1e6] = msg
      vis_parking_msg_dict = {key: val for key, val in sorted(vis_parking_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in vis_parking_msg_dict.items():
        self.vis_parking_msg['t'].append(t)
        self.vis_parking_msg['abs_t'].append(t)
        self.vis_parking_msg['data'].append(msg)
      self.vis_parking_msg['t'] = [tmp - self.vis_parking_msg['t'][0]  for tmp in self.vis_parking_msg['t']]
      max_time = max(max_time, self.vis_parking_msg['t'][-1])
      print('vis_parking_msg time:',self.vis_parking_msg['t'][-1])
      if len(self.vis_parking_msg['t']) > 0:
        self.vis_parking_msg['enable'] = True
      else:
        self.vis_parking_msg['enable'] = False
    except:
      self.vis_parking_msg['enable'] = False
      print('missing /parking_slot !!!')


    # load state machine msg
    try:
      soc_state_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/system_state/soc_state"):
        soc_state_msg_dict[msg.header.timestamp / 1e6] = msg
      soc_state_msg_dict = {key: val for key, val in sorted(soc_state_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in soc_state_msg_dict.items():
        self.soc_state_msg['t'].append(t)
        self.soc_state_msg['abs_t'].append(t)
        self.soc_state_msg['data'].append(msg)
      self.soc_state_msg['t'] = [tmp - self.soc_state_msg['t'][0]  for tmp in self.soc_state_msg['t']]
      max_time = max(max_time, self.soc_state_msg['t'][-1])
      print('soc_state_msg time:',self.soc_state_msg['t'][-1])
      if len(self.soc_state_msg['t']) > 0:
        self.soc_state_msg['enable'] = True
      else:
        self.soc_state_msg['enable'] = False
    except:
      self.soc_state_msg['enable'] = False
      print('missing /iflytek/system_state/soc_state !!!')

    self.max_time = max_time
    return max_time

def update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data):

  ### step 1: 时间戳对齐
  loc_msg_idx = 0
  if bag_loader.loc_msg['enable'] == True:
    while bag_loader.loc_msg['t'][loc_msg_idx] <= bag_time and loc_msg_idx < (len(bag_loader.loc_msg['t'])-1):
        loc_msg_idx = loc_msg_idx + 1
  local_view_data['data_index']['loc_msg_idx'] = loc_msg_idx

  fus_msg_idx = 0
  if bag_loader.fus_msg['enable'] == True:
    while bag_loader.fus_msg['t'][fus_msg_idx] <= bag_time and fus_msg_idx < (len(bag_loader.fus_msg['t'])-1):
        fus_msg_idx = fus_msg_idx + 1
  local_view_data['data_index']['fus_msg_idx'] = fus_msg_idx

  fus_parking_msg_idx = 0
  if bag_loader.fus_parking_msg['enable'] == True:
    while bag_loader.fus_parking_msg['t'][fus_parking_msg_idx] <= bag_time and fus_parking_msg_idx < (len(bag_loader.fus_parking_msg['t'])-1):
      fus_parking_msg_idx = fus_parking_msg_idx + 1
  local_view_data['data_index']['fus_parking_msg_idx'] = fus_parking_msg_idx

  vis_parking_msg_idx = 0
  if bag_loader.vis_parking_msg['enable'] == True:
    while bag_loader.vis_parking_msg['t'][vis_parking_msg_idx] <= bag_time and vis_parking_msg_idx < (len(bag_loader.vis_parking_msg['t'])-1):
      vis_parking_msg_idx = vis_parking_msg_idx + 1
  local_view_data['data_index']['vis_parking_msg_idx'] = vis_parking_msg_idx

  vs_msg_idx = 0
  if bag_loader.vs_msg['enable'] == True:
    while bag_loader.vs_msg['t'][vs_msg_idx] <= bag_time and vs_msg_idx < (len(bag_loader.vs_msg['t'])-1):
        vs_msg_idx = vs_msg_idx + 1
  local_view_data['data_index']['vs_msg_idx'] = vs_msg_idx

  plan_msg_idx = 0
  if bag_loader.plan_msg['enable'] == True:
    while bag_loader.plan_msg['t'][plan_msg_idx] <= bag_time and plan_msg_idx < (len(bag_loader.plan_msg['t'])-1):
        plan_msg_idx = plan_msg_idx + 1
  local_view_data['data_index']['plan_msg_idx'] = plan_msg_idx

  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-1):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
  local_view_data['data_index']['plan_debug_msg_idx'] = plan_debug_msg_idx

  ctrl_msg_idx = 0
  if bag_loader.ctrl_msg['enable'] == True:
    while bag_loader.ctrl_msg['t'][ctrl_msg_idx] <= bag_time and ctrl_msg_idx < (len(bag_loader.ctrl_msg['t'])-1):
        ctrl_msg_idx = ctrl_msg_idx + 1
  local_view_data['data_index']['ctrl_msg_idx'] = ctrl_msg_idx

  ctrl_debug_msg_idx = 0
  if bag_loader.ctrl_debug_msg['enable'] == True:
    while bag_loader.ctrl_debug_msg['t'][ctrl_debug_msg_idx] <= bag_time and ctrl_debug_msg_idx < (len(bag_loader.ctrl_debug_msg['t'])-1):
        ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
  local_view_data['data_index']['ctrl_debug_msg_idx'] = ctrl_debug_msg_idx

  soc_state_msg_idx = 0
  if bag_loader.soc_state_msg['enable'] == True:
    while bag_loader.soc_state_msg['t'][soc_state_msg_idx] <= bag_time and soc_state_msg_idx < (len(bag_loader.soc_state_msg['t'])-1):
        soc_state_msg_idx = soc_state_msg_idx + 1
  local_view_data['data_index']['soc_state_msg_idx'] = soc_state_msg_idx

  ### step 2: 加载定位信息
  cur_pos_xn0 = 0
  cur_pos_yn0 = 0
  cur_pos_xn = 0
  cur_pos_yn = 0
  cur_yaw = 0

  if bag_loader.loc_msg['enable'] == True:

    # ego pos in local and global coordinates
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].pose.local_position.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].pose.local_position.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].pose.euler_angles.yaw

    # coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)

    ego_xn, ego_yn = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.loc_msg['data'])):
      pos_xn_i = bag_loader.loc_msg['data'][i].pose.local_position.x
      pos_yn_i = bag_loader.loc_msg['data'][i].pose.local_position.y
      ego_xn.append(pos_xn_i - cur_pos_xn0)
      ego_yn.append(pos_yn_i - cur_pos_yn0)

    local_view_data['data_ego'].data.update({
      'ego_xn': ego_xn,
      'ego_yn': ego_yn,
    })

    # car pos in global coordinates
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], cur_pos_xn, cur_pos_yn, cur_yaw)
        car_xn.append(tmp_x - cur_pos_xn0)
        car_yn.append(tmp_y - cur_pos_yn0)

    local_view_data['data_car'].data.update({
      'car_xn': car_xn,
      'car_yn': car_yn,
    })

    local_view_data['data_current_pos'].data.update({
      'current_pos_x': [cur_pos_xn - cur_pos_xn0],
      'current_pos_y': [cur_pos_yn - cur_pos_yn0],
    })

    vel_ego =  bag_loader.loc_msg['data'][loc_msg_idx].pose.linear_velocity_from_wheel

    if bag_loader.vs_msg['enable'] == True:
      steer_deg = bag_loader.vs_msg['data'][vs_msg_idx].steering_wheel_angle * 57.3
    else:
      steer_deg = 0.0

    local_view_data['data_text'].data.update({
      'vel_ego_text': ['v = {:.2f} m/s\nsteer = {:.1f} deg'.format(round(vel_ego, 2), round(steer_deg, 1))],
    })

  ### step 3: 加载planning轨迹信息
  if bag_loader.plan_msg['enable'] == True:
    trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
    plan_x = []
    plan_y = []

    for i in range(len(trajectory.trajectory_points)):
      plan_x.append(trajectory.trajectory_points[i].x - cur_pos_xn0)
      plan_y.append(trajectory.trajectory_points[i].y - cur_pos_yn0)

    local_view_data['data_planning'].data.update({
        'plan_traj_y' : plan_y,
        'plan_traj_x' : plan_x,
    })

  # load control
  if bag_loader.ctrl_msg['enable'] == True:
    control_result_points = bag_loader.ctrl_msg['data'][ctrl_msg_idx].control_trajectory.control_result_points
    mpc_dx = []
    mpc_dy = []
    for i in range(len(control_result_points)):
      mpc_dx.append(control_result_points[i].x)
      mpc_dy.append(control_result_points[i].y)

    local_view_data['data_control'].data.update({
        'mpc_dx' : mpc_dx,
        'mpc_dy' : mpc_dy,
    })

  # load fusion slot
  if bag_loader.fus_parking_msg['enable'] == True:
    parking_fusion_slot_lists = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx].parking_fusion_slot_lists
    select_slot_id = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx].select_slot_id
    # clear data
    local_view_data['data_selected_slot'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_fusion_parking'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_fusion_parking_id'].data.update({'id':[], 'id_text_x':[], 'id_text_y':[],})
    slots_x_vec = []
    slots_y_vec = []
    id_vec = []
    id_text_x_vec = []
    id_text_y_vec = []
    for j in range(len(parking_fusion_slot_lists)):
      slot = parking_fusion_slot_lists[j]
      single_slot_x_vec = []
      single_slot_y_vec = []
      # attention: fusion slots are based on odom system, visual slots are based on vehicle system
      # 1. update slots corner points
      for k in range(len(slot.corner_points)):
        corner_x_global = slot.corner_points[k].x
        corner_y_global = slot.corner_points[k].y
        single_slot_x_vec.append(corner_x_global - cur_pos_xn0)
        single_slot_y_vec.append(corner_y_global - cur_pos_yn0)
      slot_plot_x_vec = [single_slot_x_vec[0],single_slot_x_vec[2],single_slot_x_vec[3],single_slot_x_vec[1]]
      slot_plot_y_vec = [single_slot_y_vec[0],single_slot_y_vec[2],single_slot_y_vec[3],single_slot_y_vec[1]]
      slots_x_vec.append(slot_plot_x_vec)
      slots_y_vec.append(slot_plot_y_vec)
      # 1.2 update slots limiter points in same slot_plot_vec
      single_limiter_x_vec = []
      single_limiter_y_vec = []
      if len(slot.limiter_position)!= 0:
        single_limiter_x_vec.append(slot.limiter_position[0].x - cur_pos_xn0)
        single_limiter_x_vec.append(slot.limiter_position[1].x - cur_pos_xn0)
        single_limiter_y_vec.append(slot.limiter_position[0].y - cur_pos_yn0)
        single_limiter_y_vec.append(slot.limiter_position[1].y - cur_pos_yn0)
      slots_x_vec.append(single_limiter_x_vec)
      slots_y_vec.append(single_limiter_y_vec)

      # 2. update selected fusion slot
      if select_slot_id == slot.id:
        selected_slot_with_limiter_x_vec = []
        selected_slot_with_limiter_y_vec = []
        selected_slot_with_limiter_x_vec.append(slot_plot_x_vec)
        selected_slot_with_limiter_x_vec.append(single_limiter_x_vec)

        selected_slot_with_limiter_y_vec.append(slot_plot_y_vec)
        selected_slot_with_limiter_y_vec.append(single_limiter_y_vec)

        local_view_data['data_selected_slot'].data.update({
          'corner_point_x': selected_slot_with_limiter_x_vec,
          'corner_point_y': selected_slot_with_limiter_y_vec,
          })
      # 3. update slot ids' text with their position
      id_vec.append(slot.id)
      id_text_x_vec.append((slot_plot_x_vec[0] + slot_plot_x_vec[1] + slot_plot_x_vec[2] + slot_plot_x_vec[3]) * 0.25)
      id_text_y_vec.append((slot_plot_y_vec[0] + slot_plot_y_vec[1] + slot_plot_y_vec[2] + slot_plot_y_vec[3]) * 0.25)

    local_view_data['data_fusion_parking'].data.update({'corner_point_x': slots_x_vec, 'corner_point_y': slots_y_vec,})
    local_view_data['data_fusion_parking_id'].data.update({'id':id_vec,'id_text_x':id_text_x_vec,'id_text_y':id_text_y_vec,})

    parking_fusion_slot_lists = bag_loader.fus_parking_msg['data'][-1].parking_fusion_slot_lists
    select_slot_id = bag_loader.fus_parking_msg['data'][-1].select_slot_id
    slots_x_vec = []
    slots_y_vec = []

    for j in range(len(parking_fusion_slot_lists)):
      slot = parking_fusion_slot_lists[j]
      single_slot_x_vec = []
      single_slot_y_vec = []
      # attention: fusion slots are based on odom system, visual slots are based on vehicle system
      # 1. update slots corner points
      for k in range(len(slot.corner_points)):
        corner_x_global = slot.corner_points[k].x
        corner_y_global = slot.corner_points[k].y
        single_slot_x_vec.append(corner_x_global - cur_pos_xn0)
        single_slot_y_vec.append(corner_y_global - cur_pos_yn0)
      slot_plot_x_vec = [single_slot_x_vec[0],single_slot_x_vec[2],single_slot_x_vec[3],single_slot_x_vec[1]]
      slot_plot_y_vec = [single_slot_y_vec[0],single_slot_y_vec[2],single_slot_y_vec[3],single_slot_y_vec[1]]

      # 2. update selected fusion slot
      if select_slot_id == slot.id:
        local_view_data['data_final_slot'].data.update({'corner_point_x': slot_plot_x_vec,
                                                        'corner_point_y': slot_plot_y_vec,})
        break
  # load visual slot
  if bag_loader.vis_parking_msg['enable'] == True:
    parking_slot = bag_loader.vis_parking_msg['data'][vis_parking_msg_idx].parking_slot
    local_view_data['data_vision_parking'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    slots_x_vec = []
    slots_y_vec = []
    limiter_x_vec = []
    limiter_y_vec = []

    # attention: vision slots and limiters are based on vehicle system, needed to be transferred into global system
    # 1. updatge slot points
    for j in range(len(parking_slot)):
      slot = parking_slot[j]
      single_slot_x_vec = []
      single_slot_y_vec = []
      slot_plot_x_vec = []
      slot_plot_y_vec = []
      for k in range(len(slot.corner_points)):
        corner_x_local = slot.corner_points[k].x
        corner_y_local = slot.corner_points[k].y
        corner_x_global, corner_y_global = local2global(corner_x_local, corner_y_local, cur_pos_xn, cur_pos_yn, cur_yaw)
        single_slot_x_vec.append(corner_x_global - cur_pos_xn0)
        single_slot_y_vec.append(corner_y_global - cur_pos_yn0)
      slot_plot_x_vec = [single_slot_x_vec[0],single_slot_x_vec[2],single_slot_x_vec[3],single_slot_x_vec[1]]
      slot_plot_y_vec = [single_slot_y_vec[0],single_slot_y_vec[2],single_slot_y_vec[3],single_slot_y_vec[1]]
      slots_x_vec.append(slot_plot_x_vec)
      slots_y_vec.append(slot_plot_y_vec)
    # 2. update limiters
    vision_slot_limiter = bag_loader.vis_parking_msg['data'][vis_parking_msg_idx].vision_slot_limiter
    for j in range(len(vision_slot_limiter)):
      limiter = vision_slot_limiter[j]
      global_limiter_x0, global_limiter_y0 = local2global(limiter.limiter_points[0].x, limiter.limiter_points[0].y, cur_pos_xn, cur_pos_yn, cur_yaw)
      global_limiter_x1, global_limiter_y1 = local2global(limiter.limiter_points[1].x, limiter.limiter_points[1].y, cur_pos_xn, cur_pos_yn, cur_yaw)
      slots_x_vec.append([global_limiter_x0 - cur_pos_xn0, global_limiter_x1 - cur_pos_xn0])
      slots_y_vec.append([global_limiter_y0 - cur_pos_yn0, global_limiter_y1 - cur_pos_yn0])
    local_view_data['data_vision_parking'].data.update({
      'corner_point_x': slots_x_vec,
      'corner_point_y': slots_y_vec,
    })

    # car pos in global coordinates
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], cur_pos_xn, cur_pos_yn, cur_yaw)
        car_xn.append(tmp_x - cur_pos_xn0)
        car_yn.append(tmp_y - cur_pos_yn0)

    local_view_data['data_car'].data.update({
      'car_xn': car_xn,
      'car_yn': car_yn,
    })
  return local_view_data


def load_local_view_figure_parking():
  data_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
  data_current_pos = ColumnDataSource(data = {'current_pos_y':[], 'current_pos_x':[]})
  data_ego = ColumnDataSource(data = {'ego_yn':[], 'ego_xn':[]})
  data_text = ColumnDataSource(data = {'vel_ego_text':[]})

  data_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_control = ColumnDataSource(data = {'mpc_dx':[],
                                          'mpc_dy':[],})
  data_fusion_parking = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
  data_vision_parking = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
  data_selected_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_final_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_fusion_parking_id = ColumnDataSource(data = {'id':[], 'id_text_x':[], 'id_text_y':[]})
  data_index = {'loc_msg_idx': 0,
                'road_msg_idx': 0,
                'fus_msg_idx': 0,
                'fus_parking_msg_idx': 0,
                'vis_parking_msg_idx': 0,
                'vs_msg_idx': 0,
                'plan_msg_idx': 0,
                'plan_debug_msg_idx': 0,
                'ctrl_msg_idx': 0,
                'ctrl_debug_msg_idx': 0,
                'soc_state_msg_idx': 0,
               }

  local_view_data = {'data_car':data_car, \
                     'data_current_pos': data_current_pos, \
                     'data_ego':data_ego, \
                     'data_text':data_text, \
                     'data_planning':data_planning,\
                     'data_control':data_control,\
                     'data_fusion_parking':data_fusion_parking, \
                     'data_vision_parking':data_vision_parking, \
                     'data_selected_slot':data_selected_slot, \
                     'data_final_slot':data_final_slot, \
                     'data_fusion_parking_id':data_fusion_parking_id, \
                     'data_index': data_index, \
                     }
  ### figures config

  fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=640, match_aspect = True, aspect_scale=1)
  fig1.x_range.flipped = True
  # figure plot
  f1 = fig1.patch('car_yn', 'car_xn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
  fig1.circle('current_pos_y','current_pos_x', source = data_current_pos, size=5, color='orange')
  fig1.line('ego_yn', 'ego_xn', source = data_ego, line_width = 1.5, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig1.text(0.0, -2.0, text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt", legend_label = 'car')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 2.5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan')
  fig1.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 4.0, line_color = 'red', line_dash = 'solid', line_alpha = 0.8, legend_label = 'mpc')

  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_vision_parking, line_width = 3, line_color = 'lightgrey', line_dash = 'solid',legend_label = 'vision_parking_slot')
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_fusion_parking, line_width = 2, line_color = 'red', line_dash = 'solid',legend_label = 'fusion_parking_slot')
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_selected_slot, line_width = 3, line_color = 'green', line_dash = 'solid',legend_label = 'selected_parking_slot')
  fig1.line('corner_point_y', 'corner_point_x', source = data_final_slot, line_width = 3, line_color = '#A52A2A', line_dash = 'dashed',legend_label = 'final_parking_slot')

  fig1.text(x = 'id_text_y', y = 'id_text_x', text = 'id', source = data_fusion_parking_id, text_color='red', text_align='center', text_font_size='10pt',legend_label = 'fusion_parking_slot_id')
  # toolbar
  fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

  # legend
  fig1.legend.click_policy = 'hide'
  return fig1, local_view_data
