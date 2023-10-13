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
from bokeh.models import DataTable, DateFormatter, TableColumn
from bokeh.models import TextInput
from cyber_record.record import Record
from google.protobuf.json_format import MessageToJson
from lib.local_view_lib import *

plan_debug_ts = []
plan_debug_timestamps = []
fusion_object_timestamps = []
fusion_road_timestamps = []
localization_timestamps = []
prediction_timestamps = []
vehicle_service_timestamps = []
control_output_timestamps = []
slot_timestamps = []
fusion_slot_timestamps = []
vision_slot_timestamps = []
mobileye_lane_lines_timestamps = []
mobileye_objects_timestamps = []

car_xb, car_yb = load_car_params_patch()
car_circle_x, car_circle_y, car_circle_r = load_car_circle_coord()
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

    # precept_info msg
    self.precept_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # precept_debug_info msg
    self.precept_debug_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # wave_info msg
    self.wave_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # wave_debug_info msg
    self.wave_debug_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

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
      print('plan version:', self.plan_msg['data'][0].meta.header.version)
      if len(self.plan_msg['t']) > 0:
        self.plan_msg['enable'] = True
      else:
        self.plan_msg['enable'] = False
    except:
      self.plan_msg['enable'] = False
      print("missing /iflytek/planning/plan !!!")

    # load planning debug msg
    try:
      json_value_list = ["replan_status", "ego_pos_x", "is_replan"]

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
      json_value_list = ["controller_status", "lon_enable", "lat_enable", "lat_mpc_status", "gear_plan", "gear_real", "gear_cmd",
                    "vel_ref", "vel_ref_gain", "vel_cmd", "vel_ego",
                    "path_length_plan", "remain_s_plan", "remain_s_prebreak", "remain_s_uss", "remain_s_ctrl",
                    "vel_out", "acc_vel", "vel_KP_term", "vel_KI_term", "slope_acc", "throttle_brake",
                    "steer_angle_cmd", "steer_angle", 'driver_hand_torque',
                    "lat_err", "phi_err",
                    "apa_enable", "emergency_stop_flag", "vehicle_stationary_flag", "apa_finish_flag", "break_override_flag", "gear_shifting_flag"]

      json_vector_list = ["dx_ref_vec", "dy_ref_vec", "dx_ref_mpc_vec", "dy_ref_mpc_vec", "dphi_ref_mpc_vec", "dx_mpc_vec", "dy_mpc_vec", "delta_mpc_vec", "dphi_mpc_vec"]


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

    # add obstacles in plot apa
    # load precept_info msg
    try:
      precept_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/ultrasonic_perception_info"):
        precept_msg_dict[msg.header.timestamp / 1e6] = msg
      precept_msg_dict = {key: val for key, val in sorted(precept_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in precept_msg_dict.items():
        self.precept_msg['t'].append(t)
        self.precept_msg['abs_t'].append(t)
        self.precept_msg['data'].append(msg)
      self.precept_msg['t'] = [tmp - t0 for tmp in self.precept_msg['t']]
      self.precept_msg['enable'] = True
      print('precept time:',self.precept_msg['t'][-1])
      max_time = max(max_time, self.precept_msg['t'][-1])
      if len(self.precept_msg['t']) > 0:
        self.precept_msg['enable'] = True
      else:
        self.precept_msg['enable'] = False
    except:
      self.precept_msg['enable'] = False
      print("missing /iflytek/ultrasonic_perception_info !!!")


    # precept_debug_info msg
    try:
      precept_debug_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/ultrasonic_perception_debug_info"):
        precept_debug_msg_dict[msg.header.timestamp / 1e6] = msg
      precept_debug_msg_dict = {key: val for key, val in sorted(precept_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in precept_debug_msg_dict.items():
        self.precept_debug_msg['t'].append(t)
        self.precept_debug_msg['abs_t'].append(t)
        self.precept_debug_msg['data'].append(msg)
      self.precept_debug_msg['t'] = [tmp - t0  for tmp in self.precept_debug_msg['t']]
      self.precept_debug_msg['enable'] = True
      print('precept_debug time:',self.precept_debug_msg['t'][-1])
      max_time = max(max_time, self.precept_debug_msg['t'][-1])
      if len(self.precept_debug_msg['t']) > 0:
        self.precept_debug_msg['enable'] = True
      else:
        self.precept_debug_msg['enable'] = False
    except:
      self.precept_debug_msg['enable'] = False
      print("missing /iflytek/ultrasonic_perception_debug_info !!!")

    # load wave_info msg
    try:
      wave_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/uss/wave_info"):
        wave_msg_dict[msg.header.timestamp / 1e6] = msg
      wave_msg_dict = {key: val for key, val in sorted(wave_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in wave_msg_dict.items():
        self.wave_msg['t'].append(t)
        self.wave_msg['abs_t'].append(t)
        self.wave_msg['data'].append(msg)
      self.wave_msg['t'] = [tmp - t0  for tmp in self.wave_msg['t']]
      self.wave_msg['enable'] = True
      print('wave time:',self.wave_msg['t'][-1])
      max_time = max(max_time, self.wave_msg['t'][-1])
      if len(self.wave_msg['t']) > 0:
        self.wave_msg['enable'] = True
      else:
        self.wave_msg['enable'] = False
    except:
      self.wave_msg['enable'] = False
      print("missing /iflytek/uss/wave_info !!!")

    # load wave_debug_info msg
    try:
      wave_debug_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/uss/ussdriver_debug_info"):
        wave_debug_msg_dict[msg.header.timestamp / 1e6] = msg
      wave_debug_msg_dict = {key: val for key, val in sorted(wave_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in wave_debug_msg_dict.items():
        self.wave_debug_msg['t'].append(t)
        self.wave_debug_msg['abs_t'].append(t)
        self.wave_debug_msg['data'].append(msg)
      self.wave_debug_msg['t'] = [tmp - t0  for tmp in self.wave_debug_msg['t']]
      self.wave_debug_msg['enable'] = True
      print('wave_debug time:',self.wave_debug_msg['t'][-1])
      max_time = max(max_time, self.wave_debug_msg['t'][-1])
      if len(self.wave_debug_msg['t']) > 0:
        self.wave_debug_msg['enable'] = True
      else:
        self.wave_debug_msg['enable'] = False
    except:
      self.wave_debug_msg['enable'] = False
      print("missing /iflytek/uss/ussdriver_debug_info !!!")

    return max_time

def update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data, plot_ctrl_flag=False):

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

  wave_msg_idx = 0
  if bag_loader.wave_msg['enable'] == True:
    while bag_loader.wave_msg['t'][wave_msg_idx] <= bag_time and wave_msg_idx < (len(bag_loader.wave_msg['t'])-1):
        wave_msg_idx = wave_msg_idx + 1
  local_view_data['data_index']['wave_msg_idx'] = wave_msg_idx

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

    coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)

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

    car_circle_xn = []
    car_circle_yn = []
    car_circle_rn = []
    for i in range(len(car_circle_x)):
      tmp_x, tmp_y = local2global(car_circle_x[i], car_circle_y[i], cur_pos_xn, cur_pos_yn, cur_yaw)
      car_circle_xn.append(tmp_x - cur_pos_xn0)
      car_circle_yn.append(tmp_y - cur_pos_yn0)
      car_circle_rn.append(car_circle_r[i])

    local_view_data['data_car_circle'].data.update({
      'car_circle_xn': car_circle_xn,
      'car_circle_yn': car_circle_yn,
      'car_circle_rn': car_circle_rn,
    })

    vel_ego =  bag_loader.loc_msg['data'][loc_msg_idx].pose.linear_velocity_from_wheel

    if bag_loader.vs_msg['enable'] == True:
      steer_deg = bag_loader.vs_msg['data'][vs_msg_idx].steering_wheel_angle * 57.3
    else:
      steer_deg = 0.0

    current_state = bag_loader.soc_state_msg['data'][soc_state_msg_idx].current_state
    local_view_data['data_text'].data.update({
      'vel_ego_text': ['v = {:.2f} m/s, steer = {:.1f} deg, state = {:d}'.format(round(vel_ego, 2), round(steer_deg, 1), current_state)],
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
    mpc_dx_local = []
    mpc_dy_local = []

    for i in range(len(control_result_points)):
      mpc_dx_local.append(control_result_points[i].x)
      mpc_dy_local.append(control_result_points[i].y)

    mpc_dx, mpc_dy = coord_tf.local_to_global(mpc_dx_local, mpc_dy_local)
    local_view_data['data_control'].data.update({
        'mpc_dx' : mpc_dx,
        'mpc_dy' : mpc_dy,
    })

  # load fusion slot
  if bag_loader.fus_parking_msg['enable'] == True:
    parking_fusion_slot_lists = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx].parking_fusion_slot_lists
    select_slot_id = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx].select_slot_id
    # clear data
    # local_view_data['data_target_managed_slot'].data.update({'corner_point_x': [], 'corner_point_y': [],})
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

  # load plan debug msg
  if bag_loader.plan_debug_msg['enable'] == True and bag_loader.fus_parking_msg['enable'] == True:
    local_view_data['data_target_managed_slot'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    slot_management_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].slot_management_info
    select_slot_id = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx].select_slot_id
    # print("select_slot_id in plan debug", select_slot_id)
    # print(slot_management_info)
    # print("**************************************************")
    all_managed_slot_x_vec = []
    all_managed_slot_y_vec = []
    # 1. update target managed slot
    for i in range(len(slot_management_info.slot_info_vec)):
      maganed_slot_vec = slot_management_info.slot_info_vec[i]
      corner_point = maganed_slot_vec.corner_points.corner_point
      if maganed_slot_vec.id == select_slot_id:
          target_managed_slot_x_vec = [corner_point[0].x,corner_point[2].x,corner_point[3].x,corner_point[1].x]
          target_managed_slot_y_vec = [corner_point[0].y,corner_point[2].y,corner_point[3].y,corner_point[1].y]
          local_view_data['data_target_managed_slot'].data.update({
            'corner_point_x': target_managed_slot_x_vec,
            'corner_point_y': target_managed_slot_y_vec,
            })
          #break
      slot_x = [corner_point[0].x,corner_point[2].x,corner_point[3].x,corner_point[1].x]
      slot_y = [corner_point[0].y,corner_point[2].y,corner_point[3].y,corner_point[1].y]
      all_managed_slot_x_vec.append(slot_x)
      all_managed_slot_y_vec.append(slot_y)
    local_view_data['data_all_managed_slot'].data.update({
          'corner_point_x': all_managed_slot_x_vec,
          'corner_point_y': all_managed_slot_y_vec,
          })

  # load uss wave
  if bag_loader.wave_msg['enable'] == True and bag_loader.loc_msg['enable'] == True:
    #get cur pose and uss wave
    upa_dis_info_bufs = bag_loader.wave_msg['data'][wave_msg_idx].upa_dis_info_buf
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].pose.local_position.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].pose.local_position.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].pose.euler_angles.yaw

    sector_x, sector_y, rs, start_angle, end_angle, length= [], [], [], [], [], []

    text_x, text_y = [], []
    # rs_text = []
    uss_x, uss_y = load_car_uss_patch()
    uss_angle = load_uss_angle_patch()
    wdis_index = [[0,9,6,3,1,11],[0,1,3,6,9,11]]
    m = 0
    for i in range(2):
      for j in wdis_index[i]:
          rs0 = ''
          rs1 = ''
          if upa_dis_info_bufs[i].wdis[j].wdis_value[0] <= 10 and upa_dis_info_bufs[i].wdis[j].wdis_value[0] != 0:
              rs1 = round(upa_dis_info_bufs[i].wdis[j].wdis_value[0], 2)
              # rs0 = '{:.2f}\n{:.2f}'.format(round(upa_dis_info_bufs[i].wdis[j].wdis_value[0], 2), round(upa_dis_info_bufs[i].wtype[j].wtype_value[0]))
              rs0 = '{:.2f}'.format(round(upa_dis_info_bufs[i].wdis[j].wdis_value[0], 2))
              ego_local_x, ego_local_y= local2global(uss_x[m], uss_y[m], cur_pos_xn, cur_pos_yn, cur_yaw)
              uss_angle_start = math.radians(uss_angle[m] - 30) + cur_yaw
              uss_angle_end = math.radians(uss_angle[m] +30) + cur_yaw
              x_text, y_text = one_echo_text_local(ego_local_x, ego_local_y, math.radians(uss_angle[m] - 90) + cur_yaw, rs1 - 0.5)
          elif upa_dis_info_bufs[i].wdis[j].wdis_value[0] == 0 or upa_dis_info_bufs[i].wdis[j].wdis_value[0] > 10:
              ego_local_x, ego_local_y, uss_angle_start, uss_angle_end = '', '', '', ''
              x_text, y_text = 0, 0
          text_x.append(x_text)
          text_y.append(y_text)
          sector_x.append(ego_local_x)
          sector_y.append(ego_local_y)
          # print("rs1:",rs1)
          rs.append(rs1)
          # print("rs size:",len(rs))
          length.append(rs0)
          start_angle.append(uss_angle_start)
          end_angle.append(uss_angle_end)
          m += 1
    local_view_data['data_wave'].data.update({
      'wave_x':sector_y,
      'wave_y':sector_x,
      'radius':rs,
      'start_angle':start_angle,
      'end_angle':end_angle,
    })
    local_view_data['data_wave_length_text'].data.update({
      'wave_text_x':text_y,
      'wave_text_y':text_x,
      'length':length,
    })

  if plot_ctrl_flag == True:
    names = []
    datas = []
    if bag_loader.plan_msg['enable'] == True:
      names.append("planning_status")
      datas.append(str(bag_loader.plan_msg['data'][plan_msg_idx].planning_status.apa_planning_status))

      names.append("slots_id")
      datas.append(str(bag_loader.plan_msg['data'][plan_msg_idx].successful_slot_info_list))

      names.append("plan_gear_cmd")
      datas.append(str(bag_loader.plan_msg['data'][plan_msg_idx].gear_command))

      names.append("plan_traj_available")
      datas.append(str(bag_loader.plan_msg['data'][plan_msg_idx].trajectory.available))

    # load func_state
    if bag_loader.soc_state_msg['enable'] == True:
      names.append("current_state")
      datas.append(str(bag_loader.soc_state_msg['data'][soc_state_msg_idx].current_state))

    # load vsg
    if bag_loader.vs_msg['enable'] == True:
      names.append("long_control_actuator_status")
      datas.append(str(bag_loader.vs_msg['data'][vs_msg_idx].parking_long_control_actuator_status))
      names.append("lat_control_actuator_status")
      datas.append(str(bag_loader.vs_msg['data'][vs_msg_idx].parking_lat_control_actuator_status))
      names.append("shift_lever_state")
      datas.append(str(bag_loader.vs_msg['data'][vs_msg_idx].shift_lever_state))
      names.append("shift_lever_state_available")
      datas.append(str(bag_loader.vs_msg['data'][vs_msg_idx].shift_lever_state_available))

    # load control debug
    if bag_loader.ctrl_debug_msg['enable'] == True:
      ctrl_json_data = bag_loader.ctrl_debug_msg['json']
      dx_ref_mpc_vec_local = ctrl_json_data[ctrl_debug_msg_idx]['dx_ref_mpc_vec']
      dy_ref_mpc_vec_local = ctrl_json_data[ctrl_debug_msg_idx]['dy_ref_mpc_vec']

      dx_ref_vec_local = ctrl_json_data[ctrl_debug_msg_idx]['dx_ref_vec']
      dy_ref_vec_local = ctrl_json_data[ctrl_debug_msg_idx]['dy_ref_vec']

      dx_ref_mpc_vec, dy_ref_mpc_vec = coord_tf.local_to_global(dx_ref_mpc_vec_local, dy_ref_mpc_vec_local)
      dx_ref_vec, dy_ref_vec = coord_tf.local_to_global(dx_ref_vec_local, dy_ref_vec_local)

      local_view_data['data_ref_mpc_vec'].data.update({
        'dx_ref_mpc_vec': dx_ref_mpc_vec,
        'dy_ref_mpc_vec': dy_ref_mpc_vec,
      })

      local_view_data['data_ref_vec'].data.update({
        'dx_ref_vec': dx_ref_vec,
        'dy_ref_vec': dy_ref_vec,
      })

      names.append("lat_mpc_status")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['lat_mpc_status'])
      names.append("remain_s_uss")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['remain_s_uss'])
      names.append("remain_s_ctrl")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['remain_s_ctrl'])
      names.append("vel_ref_gain")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['vel_ref_gain'])
      names.append("acc_vel")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['acc_vel'])
      names.append("slope_acc")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['slope_acc'])
      names.append("apa_enable")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['apa_enable'])
      names.append("vehicle_stationary_flag")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['vehicle_stationary_flag'])
      names.append("apa_finish_flag")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['apa_finish_flag'])
      names.append("emergency_stop_flag")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['emergency_stop_flag'])
      names.append("break_override_flag")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['break_override_flag'])
      names.append("gear_shifting_flag")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['gear_shifting_flag'])
      names.append("gear_cmd")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['gear_cmd'])
      names.append("gear_real")
      datas.append(ctrl_json_data[ctrl_debug_msg_idx]['gear_real'])

    local_view_data['ctrl_debug_data'].data.update({
      'name': names,
      'data': datas,
    })

  return local_view_data

def load_local_view_figure_parking():
  data_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
  data_car_circle = ColumnDataSource(data = {'car_circle_yn':[], 'car_circle_xn':[], 'car_circle_rn':[]})
  data_current_pos = ColumnDataSource(data = {'current_pos_y':[], 'current_pos_x':[]})
  data_ego = ColumnDataSource(data = {'ego_yn':[], 'ego_xn':[]})
  data_text = ColumnDataSource(data = {'vel_ego_text':[]})

  data_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_control = ColumnDataSource(data = {'mpc_dx':[],
                                          'mpc_dy':[],})
  data_ref_mpc_vec = ColumnDataSource(data = {'dx_ref_mpc_vec':[], 'dy_ref_mpc_vec':[],})
  data_ref_vec = ColumnDataSource(data = {'dx_ref_vec':[], 'dy_ref_vec':[],})
  data_fusion_parking = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
  data_vision_parking = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
  data_target_managed_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_final_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_fusion_parking_id = ColumnDataSource(data = {'id':[], 'id_text_x':[], 'id_text_y':[]})

  data_wave = ColumnDataSource(data = {'wave_x': [], 'wave_y': [], 'radius':[], 'start_angle':[], 'end_angle':[]})
  data_wave_length_text = ColumnDataSource(data = {'wave_text_x': [], 'wave_text_y': [], 'length':[]})

  ctrl_debug_data = ColumnDataSource({
    'name':[],
    'data':[]
  })

  data_all_managed_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
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
                'wave_msg_idx': 0,
               }

  local_view_data = {'data_car':data_car, \
                     'data_car_circle':data_car_circle, \
                     'data_current_pos': data_current_pos, \
                     'data_ego':data_ego, \
                     'data_text':data_text, \
                     'data_planning':data_planning,\
                     'data_control':data_control,\
                     'data_ref_mpc_vec':data_ref_mpc_vec, \
                     'data_ref_vec':data_ref_vec, \
                     'ctrl_debug_data':ctrl_debug_data, \
                     'data_fusion_parking':data_fusion_parking, \
                     'data_vision_parking':data_vision_parking, \
                     'data_target_managed_slot':data_target_managed_slot, \
                     'data_final_slot':data_final_slot, \
                     'data_fusion_parking_id':data_fusion_parking_id, \
                     'data_index': data_index, \
                     'data_wave':data_wave, \
                     'data_wave_length_text':data_wave_length_text, \
                     'data_all_managed_slot':data_all_managed_slot,\
                     }
  ### figures config

  fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=800, match_aspect = True, aspect_scale=1)
  fig1.x_range.flipped = True
  # figure plot
  f1 = fig1.patch('car_yn', 'car_xn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
  fig1.circle('current_pos_y','current_pos_x', source = data_current_pos, size=8, color='grey')
  fig1.circle(x ='car_circle_yn', y ='car_circle_xn', radius = 'car_circle_rn', source = data_car_circle, line_alpha = 0.5, line_width = 1, line_color = "blue", fill_alpha=0, legend_label = 'car_circle', visible = False)
  fig1.line('ego_yn', 'ego_xn', source = data_ego, line_width = 1.5, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig1.text(0.0, -2.0, text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt", legend_label = 'text')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 2.5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan')
  fig1.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 3.0, line_color = 'red', line_dash = 'solid', line_alpha = 0.8, legend_label = 'mpc')
  #fig1.line('dy_ref_mpc_vec', 'dx_ref_mpc_vec', source = data_ref_mpc_vec, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.5, legend_label = 'data_ref_mpc_vec')
  #fig1.line('dy_ref_vec', 'dx_ref_vec', source = data_ref_vec, line_width = 3.0, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'data_ref_vec')

  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_vision_parking, line_width = 3, line_color = 'lightgrey', line_dash = 'solid',legend_label = 'vision_parking_slot', visible = False)
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_fusion_parking, line_width = 2, line_color = 'red', line_dash = 'solid',legend_label = 'fusion_parking_slot')
  fig1.line('corner_point_y', 'corner_point_x', source = data_target_managed_slot, line_width = 3, line_color = 'green', line_dash = 'solid',legend_label = 'target_managed_slot')
  fig1.line('corner_point_y', 'corner_point_x', source = data_final_slot, line_width = 3, line_color = '#A52A2A', line_dash = 'dashed',legend_label = 'final_parking_slot')

  fig1.text(x = 'id_text_y', y = 'id_text_x', text = 'id', source = data_fusion_parking_id, text_color='red', text_align='center', text_font_size='10pt',legend_label = 'fusion_parking_slot')

  fig1.wedge('wave_x','wave_y', 'radius', 'start_angle', 'end_angle',source = data_wave, fill_color="lavender", line_color="black",legend_label = 'uss_wave',alpha = 0.5)
  fig1.text(x = 'wave_text_x', y = 'wave_text_y', text = 'length', source = data_wave_length_text, text_color='black', text_align='center', text_font_size='10pt',legend_label = 'uss_wave')

  # debug
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_all_managed_slot, line_width = 2, line_color = 'green', line_dash = 'solid',legend_label = 'all managed slot', visible = False)

  # toolbar
  fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

  # legend
  fig1.legend.click_policy = 'hide'
  return fig1, local_view_data

def load_local_view_figure_parking_ctrl(bag_loader, local_view_data):

  columns = [
    TableColumn(field="name", title="name",),
    TableColumn(field="data", title="data"),
  ]
  ctrl_debug_data = local_view_data['ctrl_debug_data']
  data_ctrl_debug_table = DataTable(source=ctrl_debug_data, columns=columns, width=600, height=500)

  data_control_global = ColumnDataSource(data = {
  'time': [],

  'controller_status': [],
  'lon_enable': [],
  'lat_enable': [],
  'gear_plan': [],

  'vel_cmd_plan': [],
  'vel_cmd_pos': [],
  'vel_measure': [],

  'path_length_plan': [],
  'remain_s_plan': [],
  'remain_s_prebreak': [],
  "remain_s_uss": [],
  "remain_s_ctrl": [],

  'vel_out': [],
  'vel_KP_term': [],
  'vel_KI_term': [],
  'throttle_brake': [],

  'steer_angle_cmd': [],
  'steer_angle_measure': [],
  'driver_hand_torque': [],

  'lat_err': [],
  'phi_err': [],
  })

  t_debug = []

  controller_status = []
  lon_enable = []
  lat_enable = []
  gear_plan = []

  vel_cmd_plan = []
  vel_cmd_pos = []
  vel_measure = []

  path_length_plan = []
  remain_s_plan = []
  remain_s_prebreak = []
  remain_s_uss = []
  remain_s_ctrl = []

  acc_cmd = []
  vel_kp_term = []
  vel_ki_term = []
  throttle_brake = []

  steer_angle_cmd = []
  steer_angle_measure = []
  driver_hand_torque = []

  lat_err = []
  phi_err = []

  t = 0.0

  ctrl_json_data = bag_loader.ctrl_debug_msg['json']

  for i in range(len(ctrl_json_data)):
    t_debug.append(t)

    controller_status.append(ctrl_json_data[i]['controller_status'])
    lon_enable.append(ctrl_json_data[i]['lon_enable'])
    lat_enable.append(ctrl_json_data[i]['lat_enable'])
    gear_plan.append(ctrl_json_data[i]['gear_plan'])

    vel_cmd_plan.append(ctrl_json_data[i]['vel_ref'])
    vel_cmd_pos.append(ctrl_json_data[i]['vel_cmd'])
    vel_measure.append(ctrl_json_data[i]['vel_ego'])

    path_length_plan.append(ctrl_json_data[i]['path_length_plan'])
    remain_s_plan.append(ctrl_json_data[i]['remain_s_plan'])
    remain_s_prebreak.append(ctrl_json_data[i]['remain_s_prebreak'])
    remain_s_uss.append(ctrl_json_data[i]['remain_s_uss'])
    remain_s_ctrl.append(ctrl_json_data[i]['remain_s_ctrl'])

    acc_cmd.append(ctrl_json_data[i]['vel_out'])
    vel_kp_term.append(ctrl_json_data[i]['vel_KP_term'])
    vel_ki_term.append(ctrl_json_data[i]['vel_KI_term'])
    tmp_throttle_brake = ctrl_json_data[i]['throttle_brake']
    if tmp_throttle_brake > 0.0:
      tmp_throttle_brake = tmp_throttle_brake / 1000
    throttle_brake.append(tmp_throttle_brake)

    steer_angle_cmd.append(ctrl_json_data[i]['steer_angle_cmd'] * 57.3)
    steer_angle_measure.append(ctrl_json_data[i]['steer_angle'] * 57.3)
    driver_hand_torque.append(ctrl_json_data[i]['driver_hand_torque'])

    lat_err.append(ctrl_json_data[i]['lat_err'] * 100)
    phi_err.append(ctrl_json_data[i]['phi_err'] * 57.3)

    t = t + 0.02

  data_control_global.data.update({
    'time': t_debug,

    'controller_status': controller_status,
    'lon_enable': lon_enable,
    'lat_enable': lat_enable,
    'gear_plan': gear_plan,

    'vel_cmd_plan': vel_cmd_plan,
    'vel_cmd_pos': vel_cmd_pos,
    'vel_measure': vel_measure,

    'path_length_plan': path_length_plan,
    'remain_s_plan': remain_s_plan,
    'remain_s_prebreak': remain_s_prebreak,
    'remain_s_uss': remain_s_uss,
    "remain_s_ctrl": remain_s_ctrl,

    'vel_out': acc_cmd,
    'vel_KP_term': vel_kp_term,
    'vel_KI_term': vel_ki_term,
    'throttle_brake': throttle_brake,

    'steer_angle_cmd': steer_angle_cmd,
    'steer_angle_measure': steer_angle_measure,
    'driver_hand_torque': driver_hand_torque,

    'lat_err': lat_err,
    'phi_err': phi_err,
  })

  # figures
  fig2 = bkp.figure(x_axis_label='time', y_axis_label='status',x_range = [t_debug[0], t_debug[-1]], width=500, height=200)
  fig3 = bkp.figure(x_axis_label='time', y_axis_label='vel',x_range = fig2.x_range, width=fig2.width, height=fig2.height)
  fig4 = bkp.figure(x_axis_label='time', y_axis_label='distance',x_range = fig2.x_range, width=fig2.width, height=fig2.height)
  fig5 = bkp.figure(x_axis_label='time', y_axis_label='acc',x_range = fig2.x_range, width=fig2.width, height=fig2.height)
  fig6 = bkp.figure(x_axis_label='time', y_axis_label='steer_angle',x_range = fig2.x_range, width=fig2.width, height=fig2.height)
  fig7 = bkp.figure(x_axis_label='time', y_axis_label='err',x_range = fig2.x_range, width=fig2.width, height=fig2.height)

  f2 = fig2.line('time', 'controller_status', source = data_control_global, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'controller_status')
  fig2.line('time', 'lon_enable', source = data_control_global, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'lon_enable')
  fig2.line('time', 'lat_enable', source = data_control_global, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'lat_enable')
  fig2.line('time', 'gear_plan', source = data_control_global, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'gear_plan')

  f3 = fig3.line('time', 'vel_cmd_plan', source = data_control_global, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'vel_cmd_plan')
  fig3.line('time', 'vel_cmd_pos', source = data_control_global, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'vel_cmd_pos')
  fig3.line('time', 'vel_measure', source = data_control_global, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'vel_measure')

  f4 = fig4.line('time', 'path_length_plan', source = data_control_global, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'path_length_plan')
  fig4.line('time', 'remain_s_plan', source = data_control_global, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'remain_s_plan')
  fig4.line('time', 'remain_s_prebreak', source = data_control_global, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'remain_s_prebreak')
  fig4.line('time', 'remain_s_uss', source = data_control_global, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'remain_s_uss')
  fig4.line('time', 'remain_s_ctrl', source = data_control_global, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'remain_s_ctrl')

  f5 = fig5.line('time', 'vel_out', source = data_control_global, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'acc_cmd')
  fig5.line('time', 'vel_KP_term', source = data_control_global, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'vel_kp_term')
  fig5.line('time', 'vel_KI_term', source = data_control_global, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'vel_ki_term')
  fig5.line('time', 'throttle_brake', source = data_control_global, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'throttle_brake')

  f6 = fig6.line('time', 'steer_angle_cmd', source = data_control_global, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer_angle_cmd')
  fig6.line('time', 'steer_angle_measure', source = data_control_global, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'steer_angle_measure')
  fig6.line('time', 'driver_hand_torque', source = data_control_global, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'driver_hand_torque')

  f7 = fig7.line('time', 'lat_err', source = data_control_global, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat_err')
  fig7.line('time', 'phi_err', source = data_control_global, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'phi_err')

  hover2 = HoverTool(renderers=[f2], tooltips=[('time', '@time'), ('controller_status', '@controller_status'), ('lon_enable', '@lon_enable'), ('lat_enable', '@lat_enable'),
                                              ('gear_plan', '@gear_plan')], mode='vline')

  hover3 = HoverTool(renderers=[f3], tooltips=[('time', '@time'), ('vel_cmd_plan', '@vel_cmd_plan'), ('vel_cmd_pos', '@vel_cmd_pos'),
                                              ('vel_measure', '@vel_measure')], mode='vline')

  hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time'), ('path_length_plan', '@path_length_plan'), ('remain_s_plan', '@remain_s_plan'), ('remain_s_prebreak', '@remain_s_prebreak')], mode='vline')

  hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time'), ('vel_out', '@vel_out'), ('vel_KP_term', '@vel_KP_term'), ('vel_KI_term', '@vel_KI_term'), ('throttle_brake', '@throttle_brake')], mode='vline')

  hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time'), ('steer_angle_cmd', '@steer_angle_cmd'), ('steer_angle_measure', '@steer_angle_measure'), ('driver_hand_torque', '@driver_hand_torque')], mode='vline')

  hover7 = HoverTool(renderers=[f7], tooltips=[('time', '@time'), ('lat_err', '@lat_err'), ('phi_err', '@phi_err')], mode='vline')

  fig2.add_tools(hover2)
  fig3.add_tools(hover3)
  fig4.add_tools(hover4)
  fig5.add_tools(hover5)
  fig6.add_tools(hover6)
  fig7.add_tools(hover7)

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

  return fig2, fig3, fig4, fig5, fig6, fig7, data_ctrl_debug_table


# HTML
# params 控制fig的样式
vs_car_params_apa ={
  'text_color' : 'firebrick',
  'text_align' : "center",
  'legend_label' : 'text',
  'text_font_size' : '12pt'
}

ego_car_params_apa ={
  'fill_color' : 'palegreen',
  'line_color' : "black",
  'line_width' : 1,
  'legend_label' : 'car'
}

ego_dot_params_apa ={
  'size' : 8,
  'color' : "grey",
  'legend_label' : 'car'
}
ego_circle_params_apa ={
  'line_alpha' : 0.5,
  'line_width' : 1,
  'line_color' : 'blue',
  'fill_alpha' : 0,
  'legend_label' : 'car_circle',
  'visible' : False
}
slot_params_apa = {
  'fill_color' : None,
  'line_color' : "red",
  'line_width' : 1,
  'legend_label' : 'slot'
}
fusion_slot_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "red",
  'line_width' : 2,
  'legend_label' : 'fusion_parking_slot'
}
vision_slot_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "lightgrey",
  'line_width' : 3,
  'legend_label' : 'vision_parking_slot',
  'visible' : False
}

final_slot_params_apa = {
  'line_dash' : 'dashed',
  'line_color' : "#A52A2A",
  'line_width' : 3,
  'legend_label' : 'final_parking_slot'
}

target_slot_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "green",
  'line_width' : 3,
  'legend_label' : 'target_managed_slot'
}

all_slot_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "green",
  'line_width' : 2,
  'legend_label' : 'all managed slot',
  'visible' : False
}

location_params_apa = {
  'line_width' : 1.5,
  'line_color' : 'orange',
  'line_dash' : 'solid',
  'legend_label' : 'ego_pos'
}
slot_id_params_apa = { 'text_color' : "red", 'text_align':"center", 'text_font_size':"10pt", 'legend_label' : 'fusion_parking_slot' }

location_params = {
  'line_width' : 1,
  'line_color' : 'orange',
  'line_dash' : 'solid',
  'legend_label' : 'ego_pos'
}

uss_wave_params = {
  'fill_color' : 'lavender',
  'line_color' : 'black',
  'legend_label' : 'uss_wave',
  'alpha' : 0.5
}

uss_text_params = {
  'text_color' : 'black',
  'text_align' : 'center',
  'text_font_size' : '10pt',
  'legend_label' : 'uss_wave'
}

plan_params = {
  'line_width' : 2.5, 'line_color' : 'blue', 'line_dash' : 'solid', 'line_alpha' : 0.6, 'legend_label' : 'plan'
}

mpc_params = {
  'line_width' : 3.0, 'line_color' : 'red', 'line_dash' : 'solid', 'line_alpha' : 0.8, 'legend_label' : 'mpc'
}

table_params={
    'width': 450,
    'height':500,
}

def apa_draw_local_view(dataLoader, layer_manager, plot_ctrl_flag=False):
    #define figure
    # 定义 local_view fig
    fig_local_view = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=800, match_aspect = True, aspect_scale=1)
    fig_local_view.x_range.flipped = True
    # toolbar
    fig_local_view.toolbar.active_scroll = fig_local_view.select_one(WheelZoomTool)
    # 加载planning debug部分信息, 加载planning 输入topic的时间戳
    fix_lane_xys = []
    origin_lane_xys = []
    target_lane_xys = []
    global plan_debug_ts
    global fusion_object_timestamps
    global fusion_road_timestamps
    global localization_timestamps
    global prediction_timestamps
    global vehicle_service_timestamps
    global control_output_timestamps
    global fusion_slot_timestamps
    global vision_slot_timestamps
    for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
      t = dataLoader.plan_debug_msg["t"][i]  # sub the 0 time
      plan_debug_ts.append(t)                                        # rea_t
      input_topic_timestamp = plan_debug.input_topic_timestamp
      fusion_object_timestamp = input_topic_timestamp.fusion_object  # abs_t
      fusion_road_timestamp = input_topic_timestamp.fusion_road
      localization_timestamp = input_topic_timestamp.localization
      prediction_timestamp = input_topic_timestamp.prediction
      vehicle_service_timestamp = input_topic_timestamp.vehicle_service
      control_output_timestamp = input_topic_timestamp.control_output
      slot_timestamp = input_topic_timestamp.parking_fusion

      fusion_object_timestamps.append(fusion_object_timestamp / 1e6)
      fusion_road_timestamps.append(fusion_road_timestamp / 1e6)
      localization_timestamps.append(localization_timestamp / 1e6)
      prediction_timestamps.append(prediction_timestamp / 1e6)
      vehicle_service_timestamps.append(vehicle_service_timestamp / 1e6)
      control_output_timestamps.append(control_output_timestamp / 1e6)
      fusion_slot_timestamps.append(slot_timestamp / 1e6)

  # 加载自车信息
    ego_car_generate = CommonGenerator()
    ego_center_generate = CommonGenerator()
    ego_circle_generate = CircleGenerator()
    for localization_timestamp in localization_timestamps:
      # 自车形状
      temp_cur_pos_xn = []
      temp_cur_pos_yn = []
      # 自车circle
      car_circle_xn = []
      car_circle_yn = []
      car_circle_rn = []

      flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
        # location_generator.xys.append(([],[]))
      else:
        cur_pos_xn = loc_msg.pose.local_position.x
        cur_pos_yn = loc_msg.pose.local_position.y
        cur_pos_theta = loc_msg.pose.euler_angles.yaw
        # 自车形状
        for ego_i in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[ego_i], car_yb[ego_i], cur_pos_xn, cur_pos_yn, cur_pos_theta)
          temp_cur_pos_xn.append(tmp_x)
          temp_cur_pos_yn.append(tmp_y)
        # 自车circle
        for i in range(len(car_circle_x)):
          tmp_x, tmp_y = local2global(car_circle_x[i], car_circle_y[i], cur_pos_xn, cur_pos_yn, cur_pos_theta)
          car_circle_xn.append(tmp_x)
          car_circle_yn.append(tmp_y)
          car_circle_rn.append(car_circle_r[i])
      ego_car_generate.xys.append(([temp_cur_pos_yn],[temp_cur_pos_xn]))
      ego_circle_generate.xys.append((car_circle_yn,car_circle_xn, car_circle_rn))
      # 自车中心点
      ego_center_generate.xys.append(([cur_pos_yn],[cur_pos_xn]))
    ego_car_generate.ts = np.array(plan_debug_ts)
    ego_center_generate.ts = np.array(plan_debug_ts)
    ego_circle_generate.ts = np.array(plan_debug_ts)
    ego_car_layer = PatchLayer(fig_local_view ,ego_car_params_apa)
    ego_center_layer = DotLayer(fig_local_view ,ego_dot_params_apa)
    ego_circle_layer = CircleLayer(fig_local_view ,ego_circle_params_apa)
    layer_manager.AddLayer(ego_car_layer, 'ego_car_layer', ego_car_generate, 'ego_car_generate', 2)
    layer_manager.AddLayer(ego_center_layer, 'ego_center_layer', ego_center_generate, 'ego_center_generate', 2)
    layer_manager.AddLayer(ego_circle_layer, 'ego_circle_layer', ego_circle_generate, 'ego_circle_generate', 3)

  # 加载定位
    location_generator = CommonGenerator()
    # cur_pos_xn0 = cur_pos_xn = dataLoader.loc_msg['data'][0].pose.local_position.x
    # cur_pos_yn0 = cur_pos_yn = dataLoader.loc_msg['data'][0].pose.local_position.y
    for localization_timestamp in localization_timestamps:
      ego_xb, ego_yb = [], []
      # ego_xn, ego_yn = [], []
      flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
      else:
        cur_pos_xn = loc_msg.pose.local_position.x
        cur_pos_yn = loc_msg.pose.local_position.y
        cur_yaw = loc_msg.pose.euler_angles.yaw
        ### global variables
        # pos offset
        for i in range(len(dataLoader.loc_msg['data'])):
          if (i % 10 != 0): # 下采样 10
            continue
          pos_xn_i = dataLoader.loc_msg['data'][i].pose.local_position.x
          pos_yn_i = dataLoader.loc_msg['data'][i].pose.local_position.y

          # ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

          ego_xb.append(pos_xn_i)
          ego_yb.append(pos_yn_i)
          # ego_xn.append(pos_xn_i - cur_pos_xn0)
          # ego_yn.append(pos_yn_i - cur_pos_yn0)
      location_generator.xys.append((ego_yb,ego_xb))
    location_generator.ts = np.array(plan_debug_ts)
    location_layer = CurveLayer(fig_local_view, location_params_apa)
    layer_manager.AddLayer(location_layer, 'location_layer', location_generator, 'location_generator', 2)

  # 加载vs and soc
    vs_text_generator = TextGenerator()
    for localization_timestamp in localization_timestamps:
      vel_text = []
      vel_x, vel_y = [], []
      flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
      else:
        vel_ego =  loc_msg.pose.linear_velocity_from_wheel
        soc_state_msg_idx = 0
        if dataLoader.soc_state_msg['enable'] == True:
          while dataLoader.soc_state_msg['abs_t'][soc_state_msg_idx] <= localization_timestamp and soc_state_msg_idx < (len(dataLoader.soc_state_msg['abs_t'])-1):
              soc_state_msg_idx = soc_state_msg_idx + 1
        current_state = dataLoader.soc_state_msg['data'][soc_state_msg_idx].current_state

        steer_deg = 0.0
        vs_msg_idx = 0
        if dataLoader.vs_msg['enable'] == True:
          while dataLoader.vs_msg['abs_t'][vs_msg_idx] <= localization_timestamp and vs_msg_idx < (len(dataLoader.vs_msg['abs_t'])-1):
              vs_msg_idx = vs_msg_idx + 1
          steer_deg = dataLoader.vs_msg['data'][vs_msg_idx].steering_wheel_angle * 57.3
        text = 'v = {:.2f} m/s, steer = {:.1f} deg, state = {:d}'.format(round(vel_ego, 2), round(steer_deg, 1), current_state)
        vel_text.append(text)
        vel_x.append(-2)
        vel_y.append(0)
      vs_text_generator.xys.append((vel_y, vel_x, vel_text))
    vs_text_generator.ts = np.array(plan_debug_ts)
    vs_layer = TextLayer(fig_local_view, vs_car_params_apa)
    layer_manager.AddLayer(vs_layer, 'vs_layer', vs_text_generator, 'vs_text_generator', 3)

  # 加载apa车位
    vision_slot_generate = CommonGenerator()
    fusion_slot_generate = CommonGenerator()
    target_slot_generate = CommonGenerator()
    final_slot_generate = CommonGenerator()
    all_slot_generate = CommonGenerator()
    slot_id_generate = TextGenerator()
    for slot_i, slot_timestamp in enumerate(fusion_slot_timestamps):
        flag, fusion_slot_msg = findt(dataLoader.fus_parking_msg, slot_timestamp)
        if not flag:
            print('find fusion_slot_msg error')
            fusion_slot_generate.xys.append(([], []))
            slot_id_generate.xys.append(([],[]))
            continue
        vis_parking_msg_idx = 0
        if dataLoader.vis_parking_msg['enable'] == True:
          while dataLoader.vis_parking_msg['abs_t'][vis_parking_msg_idx] <= slot_timestamp and vis_parking_msg_idx < (len(dataLoader.vis_parking_msg['abs_t'])-1):
            vis_parking_msg_idx = vis_parking_msg_idx + 1
        # vision_slot_msg = dataLoader.vis_parking_msg['data'][vis_parking_msg_idx]

      # fusion slot
        parking_fusion_slot_lists = fusion_slot_msg.parking_fusion_slot_lists
        select_slot_id = fusion_slot_msg.select_slot_id
         # 1. update slots corner points
        temp_corner_x_list = []
        temp_corner_y_list = []
        temp_slot_id_list = []
        temp_slot_id_x_list = []
        temp_slot_id_y_list = []
        for slot in parking_fusion_slot_lists:
            temp_corner_x = []
            temp_corner_y = []
            for corner_point in slot.corner_points:
                temp_corner_x.append(corner_point.x)
                temp_corner_y.append(corner_point.y)
            temp_corner_x = [temp_corner_x[0],temp_corner_x[2],temp_corner_x[3],temp_corner_x[1]]
            temp_corner_y = [temp_corner_y[0],temp_corner_y[2],temp_corner_y[3],temp_corner_y[1]]
            temp_corner_x_list.append(temp_corner_x)
            temp_corner_y_list.append(temp_corner_y)
            # 1.2 update slots limiter points in same slot_plot_vec
            single_limiter_x_vec = []
            single_limiter_y_vec = []
            if len(slot.limiter_position)!= 0:
              single_limiter_x_vec.append(slot.limiter_position[0].x)
              single_limiter_x_vec.append(slot.limiter_position[1].x)
              single_limiter_y_vec.append(slot.limiter_position[0].y)
              single_limiter_y_vec.append(slot.limiter_position[1].y)
            temp_corner_x_list.append(single_limiter_x_vec)
            temp_corner_y_list.append(single_limiter_y_vec)
            # add slot id
            temp_slot_id = slot.id
            text = '{:d}'.format(round(temp_slot_id, 2))
            temp_slot_id_list.append(text)
            temp_slot_id_x_list.append((temp_corner_x[0]+temp_corner_x[2]+temp_corner_x[3]+temp_corner_x[1])/4)
            temp_slot_id_y_list.append((temp_corner_y[0]+temp_corner_y[2]+temp_corner_y[3]+temp_corner_y[1])/4)
        fusion_slot_generate.xys.append((temp_corner_y_list, temp_corner_x_list))
        slot_id_generate.xys.append((temp_slot_id_y_list,temp_slot_id_x_list,temp_slot_id_list))
      # final slot
        temp_corner_x_list = []
        temp_corner_y_list = []
        parking_fusion_slot_lists = dataLoader.fus_parking_msg['data'][-1].parking_fusion_slot_lists
        select_slot_id = dataLoader.fus_parking_msg['data'][-1].select_slot_id
        for slot in parking_fusion_slot_lists:
            temp_corner_x = []
            temp_corner_y = []
            for corner_point in slot.corner_points:
                temp_corner_x.append(corner_point.x)
                temp_corner_y.append(corner_point.y)
            temp_corner_x = [temp_corner_x[0],temp_corner_x[2],temp_corner_x[3],temp_corner_x[1]]
            temp_corner_y = [temp_corner_y[0],temp_corner_y[2],temp_corner_y[3],temp_corner_y[1]]

            # 2. update selected fusion slot
            if select_slot_id == slot.id:
              temp_corner_x_list.append(temp_corner_x)
              temp_corner_y_list.append(temp_corner_y)
        final_slot_generate.xys.append((temp_corner_y_list, temp_corner_x_list))
        # print(final_slot_generate.xys)
      # visual slot
        temp_corner_x_list = []
        temp_corner_y_list = []
        parking_fusion_slot_lists = dataLoader.vis_parking_msg['data'][vis_parking_msg_idx].parking_slot
        if dataLoader.loc_msg['enable'] == True:
          flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamps[slot_i])
          if not flag:
            print('find loc error')
            temp_corner_x_list, temp_corner_y_list = [], []
          else:
            cur_pos_xn = loc_msg.pose.local_position.x
            cur_pos_yn = loc_msg.pose.local_position.y
            cur_yaw = loc_msg.pose.euler_angles.yaw
            # attention: fusion slots are based on odom system, visual slots are based on vehicle system
            # 1. update slots corner points
            # coord_tf = coord_transformer()
            # coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
            for slot in parking_fusion_slot_lists:
                temp_corner_x = []
                temp_corner_y = []
                for corner_point in slot.corner_points:
                  corner_x_local = corner_point.x
                  corner_y_local = corner_point.y
                  corner_x_global, corner_y_global = local2global(corner_x_local, corner_y_local, cur_pos_xn, cur_pos_yn, cur_yaw)
                  temp_corner_x.append(corner_x_global)
                  temp_corner_y.append(corner_y_global)
                temp_corner_x = [temp_corner_x[0],temp_corner_x[2],temp_corner_x[3],temp_corner_x[1]]
                temp_corner_y = [temp_corner_y[0],temp_corner_y[2],temp_corner_y[3],temp_corner_y[1]]
                temp_corner_x_list.append(temp_corner_x)
                temp_corner_y_list.append(temp_corner_y)
            # 2. update limiters
            vision_slot_limiter = dataLoader.vis_parking_msg['data'][vis_parking_msg_idx].vision_slot_limiter
            for limiter in vision_slot_limiter:
              global_limiter_x0, global_limiter_y0 = local2global(limiter.limiter_points[0].x, limiter.limiter_points[0].y, cur_pos_xn, cur_pos_yn, cur_yaw)
              global_limiter_x1, global_limiter_y1 = local2global(limiter.limiter_points[1].x, limiter.limiter_points[1].y, cur_pos_xn, cur_pos_yn, cur_yaw)
              temp_corner_x_list.append([global_limiter_x0, global_limiter_x1])
              temp_corner_y_list.append([global_limiter_y0, global_limiter_y1])
        vision_slot_generate.xys.append((temp_corner_y_list, temp_corner_x_list))

      # all slot
        slot_management_info = dataLoader.plan_debug_msg['data'][slot_i].slot_management_info
        select_slot_id = dataLoader.fus_parking_msg['data'][slot_i].select_slot_id
        all_managed_slot_x_vec = []
        all_managed_slot_y_vec = []
        temp_corner_x_list = []
        temp_corner_y_list = []
        # 1. update target managed slot
        for maganed_slot_vec in slot_management_info.slot_info_vec:
          corner_point = maganed_slot_vec.corner_points.corner_point
          slot_x = [corner_point[0].x,corner_point[2].x,corner_point[3].x,corner_point[1].x]
          slot_y = [corner_point[0].y,corner_point[2].y,corner_point[3].y,corner_point[1].y]
          if maganed_slot_vec.id == select_slot_id:
              temp_corner_x_list.append(slot_x)
              temp_corner_y_list.append(slot_y)
          all_managed_slot_x_vec.append(slot_x)
          all_managed_slot_y_vec.append(slot_y)
        target_slot_generate.xys.append((temp_corner_y_list, temp_corner_x_list))
        all_slot_generate.xys.append((all_managed_slot_y_vec, all_managed_slot_x_vec))

    fusion_slot_generate.ts = np.array(plan_debug_ts)
    slot_id_generate.ts = np.array(plan_debug_ts)
    final_slot_generate.ts = np.array(plan_debug_ts)
    vision_slot_generate.ts = np.array(plan_debug_ts)
    target_slot_generate.ts = np.array(plan_debug_ts)
    all_slot_generate.ts = np.array(plan_debug_ts)
    slot_layer = MultiCurveLayer(fig_local_view ,fusion_slot_params_apa)
    slot_id_layer = TextLayer(fig_local_view,slot_id_params_apa)
    final_slot_layer = MultiCurveLayer(fig_local_view ,final_slot_params_apa)
    vision_slot_layer = MultiCurveLayer(fig_local_view ,vision_slot_params_apa)
    target_slot_layer = MultiCurveLayer(fig_local_view ,target_slot_params_apa)
    all_slot_layer = MultiCurveLayer(fig_local_view ,all_slot_params_apa)
    layer_manager.AddLayer(slot_layer, 'slot_layer', fusion_slot_generate, 'fusion_slot_generate', 2)
    layer_manager.AddLayer(slot_id_layer, 'slot_id_layer',slot_id_generate,'slot_id_generate',3)
    layer_manager.AddLayer(final_slot_layer, 'final_slot_layer',final_slot_generate,'final_slot_generate',2)
    layer_manager.AddLayer(vision_slot_layer, 'vision_slot_layer',vision_slot_generate,'vision_slot_generate',2)
    layer_manager.AddLayer(target_slot_layer, 'target_slot_layer',target_slot_generate,'target_slot_generate',2)
    layer_manager.AddLayer(all_slot_layer, 'all_slot_layer',all_slot_generate,'all_slot_generate',2)

  # 加载plan轨迹
    plan_generator = CommonGenerator()
    for plan_debug_t in plan_debug_ts:
      flag, plan_msg = findrt(dataLoader.plan_msg, plan_debug_t)
      if not flag:
        print('find plan error')
        plan_traj_x, plan_traj_y = [], []
      else:
        trajectory = plan_msg.trajectory
        plan_traj_x = []
        plan_traj_y = []
        for j in range(len(trajectory.trajectory_points)):
          plan_traj_x.append(trajectory.trajectory_points[j].x)
          plan_traj_y.append(trajectory.trajectory_points[j].y)
      plan_generator.xys.append((plan_traj_y, plan_traj_x))
    plan_generator.ts = np.array(plan_debug_ts)
    plan_layer = CurveLayer(fig_local_view, plan_params)
    layer_manager.AddLayer(plan_layer, 'plan_layer', plan_generator, 'plane_generator', 2)

  # 加载mpc轨迹
    mpc_generator = CommonGenerator()
    for mpc_i, control_timestamp in enumerate(control_output_timestamps):
      flag, mpc_msg = findt(dataLoader.ctrl_msg, control_timestamp)
      if not flag:
        print('find mpc error')
        mpc_dx, mpc_dy = [], []
      else:
        flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamps[mpc_i])
        if not flag:
          print('find loc error')
          mpc_dx, mpc_dy = [], []
        else:
          cur_pos_xn = loc_msg.pose.local_position.x
          cur_pos_yn = loc_msg.pose.local_position.y
          cur_yaw = loc_msg.pose.euler_angles.yaw
          coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
          control_result_points = mpc_msg.control_trajectory.control_result_points
          mpc_dx_local, mpc_dy_local = [], []
          for point in control_result_points:
            mpc_dx_local.append(point.x)
            mpc_dy_local.append(point.y)
          mpc_dx, mpc_dy = coord_tf.local_to_global(mpc_dx_local, mpc_dy_local)
      mpc_generator.xys.append((mpc_dy, mpc_dx))
    mpc_generator.ts = np.array(plan_debug_ts)
    mpc_layer = CurveLayer(fig_local_view, mpc_params)
    layer_manager.AddLayer(mpc_layer, 'mpc_layer', mpc_generator, 'mpc_generator', 2)

  # 加载uss
    uss_generator = WedgesGenerator()
    uss_text_generator = TextGenerator()
    for localization_timestamp in localization_timestamps:
      sector_x, sector_y, rs, start_angle, end_angle, length= [], [], [], [], [], []
      text_x, text_y = [], []
      flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
      else:
        wave_msg_idx = 0
        if dataLoader.wave_msg['enable'] == True:
          while dataLoader.wave_msg['abs_t'][wave_msg_idx] <= (localization_timestamp) and wave_msg_idx < (len(dataLoader.wave_msg['abs_t'])-1):
              wave_msg_idx = wave_msg_idx + 1
        #get cur pose and uss wave
        upa_dis_info_bufs = dataLoader.wave_msg['data'][wave_msg_idx].upa_dis_info_buf
        cur_pos_xn = loc_msg.pose.local_position.x
        cur_pos_yn = loc_msg.pose.local_position.y
        cur_yaw = loc_msg.pose.euler_angles.yaw
        # rs_text = []
        uss_x, uss_y = load_car_uss_patch()
        uss_angle = load_uss_angle_patch()
        wdis_index = [[0,9,6,3,1,11],[0,1,3,6,9,11]]
        m = 0
        for i in range(2):
          for j in wdis_index[i]:
              rs0 = ''
              rs1 = ''
              if upa_dis_info_bufs[i].wdis[j].wdis_value[0] <= 10 and upa_dis_info_bufs[i].wdis[j].wdis_value[0] != 0:
                  rs1 = round(upa_dis_info_bufs[i].wdis[j].wdis_value[0], 2)
                  # rs0 = '{:.2f}\n{:.2f}'.format(round(upa_dis_info_bufs[i].wdis[j].wdis_value[0], 2), round(upa_dis_info_bufs[i].wtype[j].wtype_value[0]))
                  rs0 = '{:.2f}'.format(round(upa_dis_info_bufs[i].wdis[j].wdis_value[0], 2))
                  ego_local_x, ego_local_y= local2global(uss_x[m], uss_y[m], cur_pos_xn, cur_pos_yn, cur_yaw)
                  uss_angle_start = math.radians(uss_angle[m] - 30) + cur_yaw
                  uss_angle_end = math.radians(uss_angle[m] +30) + cur_yaw
                  x_text, y_text = one_echo_text_local(ego_local_x, ego_local_y, math.radians(uss_angle[m] - 90) + cur_yaw, rs1 - 0.5)
              elif upa_dis_info_bufs[i].wdis[j].wdis_value[0] == 0 or upa_dis_info_bufs[i].wdis[j].wdis_value[0] > 10:
                  ego_local_x, ego_local_y, uss_angle_start, uss_angle_end = '', '', '', ''
                  x_text, y_text = 0, 0
              text_x.append(x_text)
              text_y.append(y_text)
              sector_x.append(ego_local_x)
              sector_y.append(ego_local_y)
              # print("rs1:",rs1)
              rs.append(rs1)
              # print("rs size:",len(rs))
              length.append(rs0)
              start_angle.append(uss_angle_start)
              end_angle.append(uss_angle_end)
              m += 1
      uss_generator.xys.append((sector_y, sector_x, rs, start_angle, end_angle))
      uss_text_generator.xys.append((text_y, text_x, length))
    uss_generator.ts = np.array(plan_debug_ts)
    uss_text_generator.ts = np.array(plan_debug_ts)
    uss_layer = MultiWedgesLayer(fig_local_view, uss_wave_params)
    uss_layer_text = TextLayer(fig_local_view, uss_text_params)
    layer_manager.AddLayer(uss_layer, 'uss_layer', uss_generator, 'uss_generator', 5)
    layer_manager.AddLayer(uss_layer_text, 'uss_layer_text', uss_text_generator, 'uss_text_generator', 3)

  # legend
    fig_local_view.legend.click_policy = 'hide'
  # Table
    if plot_ctrl_flag == False:
      return fig_local_view, None
    else:
      data_ctrl_debug_data = TextGenerator()
      for plan_debug_t in plan_debug_ts:
        names = []
        datas = []
        flag, plan_msg = findrt(dataLoader.plan_msg, plan_debug_t)
        if not flag:
          print('find plan error')
          # plan_traj_x, plan_traj_y = [], []
        else:
          names.append('planning_status')
          datas.append(str(plan_msg.planning_status.apa_planning_status))

          names.append("slots_id")
          datas.append(str(plan_msg.successful_slot_info_list))

          names.append("plan_gear_cmd")
          datas.append(str(plan_msg.gear_command))

          names.append("plan_traj_available")
          datas.append(str(plan_msg.trajectory.available))

        soc_state_msg_idx = 0
        if dataLoader.soc_state_msg['enable'] == True:
          while dataLoader.soc_state_msg['t'][soc_state_msg_idx] <= plan_debug_t and soc_state_msg_idx < (len(dataLoader.soc_state_msg['t'])-1):
              soc_state_msg_idx = soc_state_msg_idx + 1
          names.append("current_state")
          datas.append(str(dataLoader.soc_state_msg['data'][soc_state_msg_idx].current_state))

        vs_msg_idx = 0
        if dataLoader.vs_msg['enable'] == True:
          while dataLoader.vs_msg['t'][vs_msg_idx] <= plan_debug_t and vs_msg_idx < (len(dataLoader.vs_msg['t'])-1):
              vs_msg_idx = vs_msg_idx + 1
          names.append("long_control_actuator_status")
          datas.append(str(dataLoader.vs_msg['data'][vs_msg_idx].parking_long_control_actuator_status))
          names.append("lat_control_actuator_status")
          datas.append(str(dataLoader.vs_msg['data'][vs_msg_idx].parking_lat_control_actuator_status))
          names.append("shift_lever_state")
          datas.append(str(dataLoader.vs_msg['data'][vs_msg_idx].shift_lever_state))
          names.append("shift_lever_state_available")
          datas.append(str(dataLoader.vs_msg['data'][vs_msg_idx].shift_lever_state_available))

        ctrl_debug_msg_idx = 0
        if dataLoader.ctrl_debug_msg['enable'] == True:
          while dataLoader.ctrl_debug_msg['t'][ctrl_debug_msg_idx] <= plan_debug_t and ctrl_debug_msg_idx < (len(dataLoader.ctrl_debug_msg['t'])-1):
              ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
          ctrl_json_data = dataLoader.ctrl_debug_msg['json']
          names.append("lat_mpc_status")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['lat_mpc_status'])
          names.append("vel_ref_gain")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['vel_ref_gain'])
          names.append("acc_vel")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['acc_vel'])
          names.append("slope_acc")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['slope_acc'])
          names.append("apa_enable")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['apa_enable'])
          names.append("vehicle_stationary_flag")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['vehicle_stationary_flag'])
          names.append("apa_finish_flag")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['apa_finish_flag'])
          names.append("emergency_stop_flag")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['emergency_stop_flag'])
          names.append("break_override_flag")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['break_override_flag'])
          names.append("gear_shifting_flag")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['gear_shifting_flag'])
          names.append("gear_cmd")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['gear_cmd'])
          names.append("gear_real")
          datas.append(ctrl_json_data[ctrl_debug_msg_idx]['gear_real'])

        data_ctrl_debug_data.xys.append((names, datas, [None] * len(names)))
      data_ctrl_debug_data.ts = np.array(plan_debug_ts)
      tab_attr_list = ['name', 'data']
      tab_debug_layer = TableLayerV2(None, tab_attr_list, table_params)
      layer_manager.AddLayer(
        tab_debug_layer, 'tab_debug_layer', data_ctrl_debug_data, 'data_ctrl_debug_data', 3)


      return fig_local_view, tab_debug_layer.plot

def apa_draw_local_view_parking_ctrl(dataLoader, layer_manager, max_time):

    json_value_list = ["controller_status", "lat_enable", "lon_enable", "gear_plan",
                        "vel_ref", "vel_cmd", "vel_ego",
                        "path_length_plan", "remain_s_plan", "remain_s_prebreak",
                        "vel_out", "vel_KP_term", "vel_KI_term", "throttle_brake",
                        "steer_angle_cmd", "steer_angle", "driver_hand_torque",
                        "lat_err", "phi_err"
                        ]

    json_value_xys_dict = GenerateJsonValueData(dataLoader.ctrl_debug_msg['json'], dataLoader.ctrl_debug_msg['t'], json_value_list)

    # fig2: control status
    fig2 = FigureLayerHover(bkp.figure(x_axis_label='time',
                                  y_axis_label='status',
                                  x_range = [0.0, max_time],
                                  width=450,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig2.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "controller_status"), "controller_status")
    fig2.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "lat_enable"), "lat_enable")
    fig2.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "lon_enable"), "lon_enable")
    fig2.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "gear_plan"), "gear_plan", last_line = True)

    # fig3: vel status
    # "vel_ref", "vel_cmd", "vel_ego"
    fig3 = FigureLayerHover(bkp.figure(x_axis_label='time',
                                  y_axis_label='vel',
                                  x_range = fig2.fig.x_range,
                                  width=450,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig3.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_ref"), "vel_cmd_plan")
    fig3.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_cmd"), "vel_cmd_pos")
    fig3.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_ego"), "vel_measure", last_line = True)

    # fig4: vel status
    # "path_length_plan", "remain_s_plan", "remain_s_prebreak"
    fig4 = FigureLayerHover(bkp.figure(x_axis_label='time',
                                  y_axis_label='distance',
                                  x_range = fig2.fig.x_range,
                                  width=450,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "path_length_plan"), "path_length_plan")
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "remain_s_plan"), "remain_s_plan")
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "remain_s_prebreak"), "remain_s_prebreak", last_line = True)

    # fig5: vel status
    # "vel_out", "vel_KP_term", "vel_KI_term", "throttle_brake"
    fig5 = FigureLayerHover(bkp.figure(x_axis_label='time',
                                  y_axis_label='acc',
                                  x_range = fig2.fig.x_range,
                                  width=450,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig5.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_out"), "acc_cmd")
    fig5.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_KP_term"), "vel_kp_term")
    fig5.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_KI_term"), "vel_ki_term")
    fig5.AddCurv(layer_manager,
                 ScalarSomeGeneratorFromJson(json_value_xys_dict, "throttle_brake"), "throttle_brake", last_line = True)

    # fig6: vel status
    # "steer_angle_cmd", "steer_angle", "driver_hand_torque"
    fig6 = FigureLayerHover(bkp.figure(x_axis_label='time',
                                  y_axis_label='steer_angle',
                                  x_range = fig2.fig.x_range,
                                  width=450,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig6.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "steer_angle_cmd", 57.3), "steer_angle_cmd")
    fig6.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "steer_angle", 57.3), "steer_angle_measure")
    fig6.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "driver_hand_torque"), "driver_hand_torque", last_line = True)

    # fig7: vel status
    # "lat_err", "phi_err"
    fig7 = FigureLayerHover(bkp.figure(x_axis_label='time',
                                  y_axis_label='err',
                                  x_range = fig2.fig.x_range,
                                  width=450,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig7.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "lat_err", 100), "lat_err")
    fig7.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "phi_err", 57.3), "phi_err", last_line = True)

    return fig2, fig3, fig4, fig5, fig6, fig7
