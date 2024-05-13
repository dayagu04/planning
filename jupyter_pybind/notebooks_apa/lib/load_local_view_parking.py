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

car_xb, car_yb = load_car_params_patch_parking()
car_circle_x, car_circle_y, car_circle_r = load_car_circle_coord()
coord_tf = coord_transformer()
max_slot_num = 20
replan_flag = False
correct_path_for_limiter = False
replan_time_list = []
correct_path_for_limiter_time_list = []
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

    # adas_debug_info msg
    self.adas_debug_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # uss_perception msg
    self.uss_percept_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    self.parking_flag = parking_flag

    self.max_time = 0

    self.enter_parking_time = 0
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
      json_value_list = ["tlane_p0_x", "tlane_p0_y", "tlane_p1_x", "tlane_p1_y", "tlane_pt_x", "tlane_pt_y", "slot_side",
                         "terminal_error_x", "terminal_error_y", "terminal_error_heading",
                         "is_replan", "is_finished", "is_replan_first", "is_replan_by_uss", "current_path_length", "gear_change_count", "replan_reason",
                         "path_plan_success", "planning_status", "spline_success", "remain_dist", "remain_dist_uss", "stuck_time",
                         "car_static_timer_by_pos", "car_static_timer_by_vel", "static_flag", "ego_heading_slot",
                         "selected_slot_id", "slot_length", "slot_width", "slot_origin_pos_x", "slot_origin_pos_y", "slot_origin_heading",
                         "slot_occupied_ratio", "pathplan_result", "target_ego_pos_slot", "path_start_seg_index", "path_end_seg_index", "path_length",
                         "uss_available", "uss_remain_dist", "uss_index", "uss_car_index",
                         "optimization_terminal_pose_error", "optimization_terminal_heading_error", "lat_path_opt_cost_time_ms",
                         "ref_gear", "ref_arc_steer",
                         "correct_path_for_limiter", "replan_flag"]

      json_vector_list = ["raw_refline_x_vec", "raw_refline_y_vec", "assembled_delta", "assembled_omega", "traj_x_vec", "traj_y_vec",
                          "obstaclesX", "obstaclesY", "slot_corner_X", "slot_corner_Y", "limiter_corner_X", "limiter_corner_Y"]

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
                    "path_length_plan", "remain_s_plan", "remain_s_prebreak",  "remain_s_uss", "remain_s_ctrl",
                    "vel_out", "acc_vel", "vel_KP_term", "vel_KI_term", "slope_acc", "throttle_brake", 'acc_vel',
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
      if (abs(self.vis_parking_msg['t'][0]) < 0.0001):
        self.vis_parking_msg['t'] = [tmp - self.vis_parking_msg['t'][1]  for tmp in self.vis_parking_msg['t']]
      else:
        self.vis_parking_msg['t'] = [tmp - self.vis_parking_msg['t'][0]  for tmp in self.vis_parking_msg['t']]
      max_time = max(max_time, self.vis_parking_msg['t'][-1])
      print('vis_parking_msg time:',self.vis_parking_msg['t'][-1])
      if len(self.vis_parking_msg['t']) > 2:
        self.vis_parking_msg['enable'] = True
      else:
        self.vis_parking_msg['enable'] = False
        print('missing /parking_slot !!!')
    except:
      self.vis_parking_msg['enable'] = False
      print('missing /parking_slot !!!')


    # load state machine msg
    try:
      soc_state_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/system_state/soc_state"):
        soc_state_msg_dict[msg.header.timestamp / 1e6] = msg
      soc_state_msg_dict = {key: val for key, val in sorted(soc_state_msg_dict.items(), key = lambda ele: ele[0])}

      enter_parking_time = None
      replan_time_list, correct_path_for_limiter_time_list = [],[]
      for t, msg in soc_state_msg_dict.items():
        self.soc_state_msg['t'].append(t)
        self.soc_state_msg['abs_t'].append(t)
        self.soc_state_msg['data'].append(msg)
        # record the start time of fusi_parking_msg
        if msg.current_state > 1 and enter_parking_time is None:
          enter_parking_time = t
        # record the replan time
        if replan_flag:
          replan_time_list.append(t)
        # record the correct_path_for_limiter time
        if correct_path_for_limiter:
          correct_path_for_limiter_time_list.append(t)


      self.soc_state_msg['t'] = [tmp - self.soc_state_msg['t'][0]  for tmp in self.soc_state_msg['t']]
      max_time = max(max_time, self.soc_state_msg['t'][-1])
      print('soc_state_msg time:',self.soc_state_msg['t'][-1])
      if len(self.soc_state_msg['t']) > 0:
        self.soc_state_msg['enable'] = True
      else:
        self.soc_state_msg['enable'] = False

      if enter_parking_time is not None:
        self.enter_parking_time = enter_parking_time - self.soc_state_msg['abs_t'][0]
        # print("enter_parking_time ：", self.enter_parking_time)
      if replan_time_list is not None:
        replan_time_list = [tmp - self.soc_state_msg['t'][0]  for tmp in replan_time_list]
      if correct_path_for_limiter_time_list is not None:
        correct_path_for_limiter_time_list = [tmp - self.soc_state_msg['t'][0]  for tmp in correct_path_for_limiter_time_list]
    except:
      self.soc_state_msg['enable'] = False
      print('missing /iflytek/system_state/soc_state !!!')

    self.max_time = max_time

    # add obstacles in plot apa
    # load precept_info msg
    # try:
    #   precept_msg_dict = {}
    #   for topic, msg, t in self.bag.read_messages("/iflytek/ultrasonic_perception_info"):
    #     precept_msg_dict[msg.header.timestamp / 1e6] = msg
    #   precept_msg_dict = {key: val for key, val in sorted(precept_msg_dict.items(), key = lambda ele: ele[0])}
    #   for t, msg in precept_msg_dict.items():
    #     self.precept_msg['t'].append(t)
    #     self.precept_msg['abs_t'].append(t)
    #     self.precept_msg['data'].append(msg)
    #   self.precept_msg['t'] = [tmp - t0 for tmp in self.precept_msg['t']]
    #   self.precept_msg['enable'] = True
    #   print('precept time:',self.precept_msg['t'][-1])
    #   max_time = max(max_time, self.precept_msg['t'][-1])
    #   if len(self.precept_msg['t']) > 0:
    #     self.precept_msg['enable'] = True
    #   else:
    #     self.precept_msg['enable'] = False
    # except:
    #   self.precept_msg['enable'] = False
    #   print("missing /iflytek/ultrasonic_perception_info !!!")


    # # precept_debug_info msg
    # try:
    #   precept_debug_msg_dict = {}
    #   for topic, msg, t in self.bag.read_messages("/iflytek/ultrasonic_perception_debug_info"):
    #     precept_debug_msg_dict[msg.header.timestamp / 1e6] = msg
    #   precept_debug_msg_dict = {key: val for key, val in sorted(precept_debug_msg_dict.items(), key = lambda ele: ele[0])}
    #   for t, msg in precept_debug_msg_dict.items():
    #     self.precept_debug_msg['t'].append(t)
    #     self.precept_debug_msg['abs_t'].append(t)
    #     self.precept_debug_msg['data'].append(msg)
    #   self.precept_debug_msg['t'] = [tmp - t0  for tmp in self.precept_debug_msg['t']]
    #   self.precept_debug_msg['enable'] = True
    #   print('precept_debug time:',self.precept_debug_msg['t'][-1])
    #   max_time = max(max_time, self.precept_debug_msg['t'][-1])
    #   if len(self.precept_debug_msg['t']) > 0:
    #     self.precept_debug_msg['enable'] = True
    #   else:
    #     self.precept_debug_msg['enable'] = False
    # except:
    #   self.precept_debug_msg['enable'] = False
    #   print("missing /iflytek/ultrasonic_perception_debug_info !!!")

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
      # max_time = max(max_time, self.wave_msg['t'][-1])
      if len(self.wave_msg['t']) > 0:
        self.wave_msg['enable'] = True
      else:
        self.wave_msg['enable'] = False
    except:
      self.wave_msg['enable'] = False
      print("missing /iflytek/uss/wave_info !!!")
   #load adas_debug_msg
    try:
      adas_debug_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/adas_function_debug"):
        adas_debug_msg_dict[msg.header.timestamp / 1e6] = msg
      adas_debug_msg_dict = {key: val for key, val in sorted(adas_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in adas_debug_msg_dict.items():
        self.adas_debug_msg['t'].append(t)
        self.adas_debug_msg['abs_t'].append(t)
        self.adas_debug_msg['data'].append(msg)
      self.adas_debug_msg['t'] = [tmp - t0  for tmp in self.adas_debug_msg['t']]
      self.adas_debug_msg['enable'] = True
      print('wave time:',self.adas_debug_msg['t'][-1])
      # max_time = max(max_time, self.adas_debug_msg['t'][-1])
      if len(self.adas_debug_msg['t']) > 0:
        self.adas_debug_msg['enable'] = True
      else:
        self.adas_debug_msg['enable'] = False
    except:
      self.adas_debug_msg['enable'] = False
      print("/iflytek/adas_function_debug !!!")

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
      # max_time = max(max_time, self.wave_debug_msg['t'][-1])
      if len(self.wave_debug_msg['t']) > 0:
        self.wave_debug_msg['enable'] = True
      else:
        self.wave_debug_msg['enable'] = False
    except:
      self.wave_debug_msg['enable'] = False
      print("missing /iflytek/uss/ussdriver_debug_info !!!")

    # load uss perception msg
    try:
      uss_percept_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/UssPerceptInfo"):
        uss_percept_msg_dict[msg.header.timestamp / 1e6] = msg
      uss_percept_msg_dict = {key: val for key, val in sorted(uss_percept_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in uss_percept_msg_dict.items():
        self.uss_percept_msg['t'].append(t)
        self.uss_percept_msg['abs_t'].append(t)
        self.uss_percept_msg['data'].append(msg)
      t0 = self.uss_percept_msg['t'][0]
      self.uss_percept_msg['t'] = [tmp - t0  for tmp in self.uss_percept_msg['t']]
      self.uss_percept_msg['enable'] = True
      print('uss_percept time:',self.uss_percept_msg['t'][-1])
      # max_time = max(max_time, self.uss_percept_msg['t'][-1])
      if len(self.uss_percept_msg['t']) > 0:
        self.uss_percept_msg['enable'] = True
      else:
        self.uss_percept_msg['enable'] = False
    except:
      self.uss_percept_msg['enable'] = False
      print("missing /iflytek/UssPerceptInfo !!!")

    return max_time

  def get_msg_index(self, bag_time):
      ### step 1: 时间戳对齐
    out = {}
    loc_msg_idx = 0
    if self.loc_msg['enable'] == True:
      while self.loc_msg['t'][loc_msg_idx] <= bag_time and loc_msg_idx < (len(self.loc_msg['t'])-1):
          loc_msg_idx = loc_msg_idx + 1
    out['loc_msg_idx'] = loc_msg_idx

    fus_msg_idx = 0
    if self.fus_msg['enable'] == True:
      while self.fus_msg['t'][fus_msg_idx] <= bag_time and fus_msg_idx < (len(self.fus_msg['t'])-1):
          fus_msg_idx = fus_msg_idx + 1
    out['fus_msg_idx'] = fus_msg_idx

    fus_parking_msg_idx = 0
    if self.fus_parking_msg['enable'] == True:
      while self.fus_parking_msg['t'][fus_parking_msg_idx] <= bag_time - self.enter_parking_time and fus_parking_msg_idx < (len(self.fus_parking_msg['t'])-1):
        fus_parking_msg_idx = fus_parking_msg_idx + 1
    out['fus_parking_msg_idx'] = fus_parking_msg_idx

    vis_parking_msg_idx = 0
    if self.vis_parking_msg['enable'] == True:
      while self.vis_parking_msg['t'][vis_parking_msg_idx] <= bag_time and vis_parking_msg_idx < (len(self.vis_parking_msg['t'])-1):
        vis_parking_msg_idx = vis_parking_msg_idx + 1
    out['vis_parking_msg_idx'] = vis_parking_msg_idx

    vs_msg_idx = 0
    if self.vs_msg['enable'] == True:
      while self.vs_msg['t'][vs_msg_idx] <= bag_time and vs_msg_idx < (len(self.vs_msg['t'])-1):
          vs_msg_idx = vs_msg_idx + 1
    out['vs_msg_idx'] = vs_msg_idx

    plan_msg_idx = 0
    if self.plan_msg['enable'] == True:
      while self.plan_msg['t'][plan_msg_idx] <= bag_time and plan_msg_idx < (len(self.plan_msg['t'])-1):
          plan_msg_idx = plan_msg_idx + 1
    out['plan_msg_idx'] = plan_msg_idx

    plan_debug_msg_idx = 0
    if self.plan_debug_msg['enable'] == True:
      while self.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(self.plan_debug_msg['t'])-1):
          plan_debug_msg_idx = plan_debug_msg_idx + 1
    out['plan_debug_msg_idx'] = plan_debug_msg_idx

    ctrl_msg_idx = 0
    if self.ctrl_msg['enable'] == True:
      while self.ctrl_msg['t'][ctrl_msg_idx] <= bag_time and ctrl_msg_idx < (len(self.ctrl_msg['t'])-1):
          ctrl_msg_idx = ctrl_msg_idx + 1
    out['ctrl_msg_idx'] = ctrl_msg_idx

    ctrl_debug_msg_idx = 0
    if self.ctrl_debug_msg['enable'] == True:
      while self.ctrl_debug_msg['t'][ctrl_debug_msg_idx] <= bag_time and ctrl_debug_msg_idx < (len(self.ctrl_debug_msg['t'])-1):
          ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
    out['ctrl_debug_msg_idx'] = ctrl_debug_msg_idx

    soc_state_msg_idx = 0
    if self.soc_state_msg['enable'] == True:
      while self.soc_state_msg['t'][soc_state_msg_idx] <= bag_time and soc_state_msg_idx < (len(self.soc_state_msg['t'])-1):
          soc_state_msg_idx = soc_state_msg_idx + 1
    out['soc_state_msg_idx'] = soc_state_msg_idx

    wave_msg_idx = 0
    if self.wave_msg['enable'] == True:
      while self.wave_msg['t'][wave_msg_idx] <= bag_time and wave_msg_idx < (len(self.wave_msg['t'])-1):
          wave_msg_idx = wave_msg_idx + 1
    out['wave_msg_idx'] = wave_msg_idx

    adas_msg_idx = 0
    if self.adas_debug_msg['enable'] == True:
      while self.adas_debug_msg['t'][adas_msg_idx] <= bag_time and adas_msg_idx < (len(self.adas_debug_msg['t'])-1):
          adas_msg_idx = adas_msg_idx + 1
    out['adas_msg_idx'] = adas_msg_idx

    uss_percept_msg_idx = 0
    if self.uss_percept_msg['enable'] == True:
      while self.uss_percept_msg['t'][uss_percept_msg_idx] <= bag_time and uss_percept_msg_idx < (len(self.uss_percept_msg['t'])-1):
          uss_percept_msg_idx = uss_percept_msg_idx + 1
    out['uss_percept_msg_idx'] = uss_percept_msg_idx

    return out


def update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data, plot_ctrl_flag=False):

  ### step 1: timestamp alignment
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

  uss_percept_msg_idx = 0
  if bag_loader.uss_percept_msg['enable'] == True:
    while bag_loader.uss_percept_msg['t'][uss_percept_msg_idx] <= bag_time and uss_percept_msg_idx < (len(bag_loader.uss_percept_msg['t'])-1):
        uss_percept_msg_idx = uss_percept_msg_idx + 1
  local_view_data['data_index']['uss_percept_msg_idx'] = uss_percept_msg_idx

  ### step 2: load positioning information
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

    half_car_width = car_yb[0]
    heading_vec = [math.cos(cur_yaw), math.sin(cur_yaw)]
    norm_vec_1 = [-half_car_width * heading_vec[1], half_car_width * heading_vec[0]]
    norm_vec_2 = [half_car_width * heading_vec[1], -half_car_width * heading_vec[0]]
    x1 = cur_pos_xn + norm_vec_1[0]
    y1 = cur_pos_yn + norm_vec_1[1]
    x2 = cur_pos_xn + norm_vec_2[0]
    y2 = cur_pos_yn + norm_vec_2[1]

    local_view_data['data_current_line'].data.update({
      'y' : [y1, y2],
      'x' : [x1, x2],
    })

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
    remain_s_ctrl = bag_loader.ctrl_debug_msg['json'][ctrl_debug_msg_idx]['remain_s_ctrl'] * 100
    if bag_loader.vs_msg['enable'] == True:
      steer_deg = bag_loader.vs_msg['data'][vs_msg_idx].steering_wheel_angle * 57.3
    else:
      steer_deg = 0.0

    selected_slot_id = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx].select_slot_id

    current_state = bag_loader.soc_state_msg['data'][soc_state_msg_idx].current_state
    # local_view_data['data_text'].data.update({
    #   'vel_ego_text': ['v = {:.2f} m/s, steer = {:.1f} deg, state = {:d}'.format(round(vel_ego, 2), round(steer_deg, 1), current_state)],
    # })
    local_view_data['data_text'].data.update({
      'vel_ego_text': ['v = {:.2f} m/s, remain_s_ctrl = {:.1f} cm, steer = {:.1f} deg, selected_slot_id = {:d}, state = {:d}'.format(round(vel_ego, 2), round(remain_s_ctrl, 1), round(steer_deg, 1),selected_slot_id, current_state)],
    })

  ### step 3: loading planning traj information
  if bag_loader.plan_msg['enable'] == True:
    trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
    plan_x = []
    plan_y = []
    plan_heading = []

    for i in range(len(trajectory.trajectory_points)):
      plan_x.append(trajectory.trajectory_points[i].x - cur_pos_xn0)
      plan_y.append(trajectory.trajectory_points[i].y - cur_pos_yn0)
      plan_heading.append(trajectory.trajectory_points[i].heading_yaw)

    local_view_data['data_planning'].data.update({
        'plan_traj_y' : plan_y,
        'plan_traj_x' : plan_x,
    })

    car_xn = []
    car_yn = []
    line_xn = []
    line_yn = []
    car_box_x_vec = []
    car_box_y_vec = []
    if (len(plan_x) > 1):
      half_car_width = car_yb[0]
      last_x = plan_x[-1]
      last_y = plan_y[-1]
      last_heading = plan_heading[-1]
      for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], last_x, last_y, last_heading)
        car_xn.append(tmp_x - cur_pos_xn0)
        car_yn.append(tmp_y - cur_pos_yn0)

      heading_vec = [math.cos(last_heading), math.sin(last_heading)]
      norm_vec_1 = [-half_car_width * heading_vec[1], half_car_width * heading_vec[0]]
      norm_vec_2 = [half_car_width * heading_vec[1], -half_car_width * heading_vec[0]]
      x1 = last_x + norm_vec_1[0]
      y1 = last_y + norm_vec_1[1]
      x2 = last_x + norm_vec_2[0]
      y2 = last_y + norm_vec_2[1]
      line_xn = [x1, x2]
      line_yn = [y1, y2]

    local_view_data['data_car_target_pos'].data.update({
      'car_xn': car_xn,
      'car_yn': car_yn,
    })

    local_view_data['data_car_target_line'].data.update({
      'x' : line_xn,
      'y' : line_yn,
    })

    for i in range(len(plan_x)):
      car_xn = []
      car_yn = []
      for j in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], plan_x[i], plan_y[i], plan_heading[i])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
      if (i%10==0):
        car_box_x_vec.append(car_xn)
        car_box_y_vec.append(car_yn)
    local_view_data['data_car_box'].data.update({
      'x_vec': car_box_x_vec,
      'y_vec': car_box_y_vec,
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

  uss_available = False
  uss_remain_dist = 0.0
  uss_index = 0
  uss_car_index = 0
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
    occupied_x_vec = []
    occupied_y_vec = []
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
      if maganed_slot_vec.is_occupied:
        occupied_x_vec.append(slot_x)
        occupied_y_vec.append(slot_y)

    local_view_data['data_all_managed_slot'].data.update({
          'corner_point_x': all_managed_slot_x_vec,
          'corner_point_y': all_managed_slot_y_vec,
          })
    local_view_data['data_all_managed_occupied_slot'].data.update({
          'occupied_slot_y': occupied_y_vec,
          'occupied_slot_x': occupied_x_vec,
          })
    limiter_x_vec = []
    limiter_y_vec = []
    for limiter in slot_management_info.limiter_points:
      limiter_x_vec.append(limiter.x)
      limiter_y_vec.append(limiter.y)
    local_view_data['data_all_managed_limiter'].data.update({
      'limiter_point_y': limiter_y_vec,
      'limiter_point_x': limiter_x_vec,
    })

    obstacle_x = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['obstaclesX']
    obstacle_y = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['obstaclesY']

    replan_flag = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['replan_flag']
    correct_path_for_limiter = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['correct_path_for_limiter']

    local_view_data['data_obs'].data.update({
      'obs_x': obstacle_x,
      'obs_y': obstacle_y,
    })

    slot_corner_X = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['slot_corner_X']
    slot_corner_Y = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['slot_corner_Y']
    limiter_corner_X = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['limiter_corner_X']
    limiter_corner_Y = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['limiter_corner_Y']

    local_view_data['data_target_managed_slot'].data.update({
      'corner_point_x': [slot_corner_X[0], slot_corner_X[2], slot_corner_X[3], slot_corner_X[1]],
      'corner_point_y': [slot_corner_Y[0], slot_corner_Y[2], slot_corner_Y[3], slot_corner_Y[1]],
    })

    local_view_data['data_all_managed_limiter'].data.update({
      'limiter_point_x': limiter_corner_X,
      'limiter_point_y': limiter_corner_Y,
    })

    uss_available = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['uss_available']
    #print("uss_available = ", uss_available)
    uss_available = bool(uss_available)
    #print("uss_available = ", uss_available)
    uss_remain_dist = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['uss_remain_dist']
    #print("uss_remain_dist = ", uss_remain_dist)
    uss_index = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['uss_index']
    uss_index = int(uss_index)
    uss_car_index = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['uss_car_index']
    uss_car_index = int(uss_car_index)

    uss_available = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['uss_available']
    #print("uss_available = ", uss_available)
    uss_available = bool(uss_available)
    #print("uss_available = ", uss_available)
    uss_remain_dist = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['uss_remain_dist']
    #print("uss_remain_dist = ", uss_remain_dist)
    uss_index = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['uss_index']
    uss_index = int(uss_index)
    uss_car_index = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['uss_car_index']
    uss_car_index = int(uss_car_index)

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

    if uss_available == True and uss_index <= len(sector_x) and current_state >= 29:
      local_view_data['data_wave_min'].data.update({
        'wave_x':[sector_y[uss_index]],
        'wave_y':[sector_x[uss_index]],
        'radius':[rs[uss_index]],
        'start_angle':[start_angle[uss_index]],
        'end_angle':[end_angle[uss_index]],
      })
    else:
      local_view_data['data_wave_min'].data.update({
        'wave_x':[],
        'wave_y':[],
        'radius':[],
        'start_angle':[],
        'end_angle':[],
      })

    if uss_available == True and uss_index <= len(sector_x) and current_state >= 29:
      local_view_data['data_wave_min'].data.update({
        'wave_x':[sector_y[uss_index]],
        'wave_y':[sector_x[uss_index]],
        'radius':[rs[uss_index]],
        'start_angle':[start_angle[uss_index]],
        'end_angle':[end_angle[uss_index]],
      })
    else:
      local_view_data['data_wave_min'].data.update({
        'wave_x':[],
        'wave_y':[],
        'radius':[],
        'start_angle':[],
        'end_angle':[],
      })

  if plot_ctrl_flag == True:
    names = []
    datas = []
    # load planning data
    if bag_loader.plan_msg['enable'] == True:
      names.append("apa_planning_status")
      datas.append(str(bag_loader.plan_msg['data'][plan_msg_idx].planning_status.apa_planning_status))

      names.append("planning_stm")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['planning_status']))

      names.append("replan_reason")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['replan_reason']))

      names.append("path_plan_success")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['path_plan_success']))

      names.append("path_plan_result")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['pathplan_result']))

      names.append("replan_flag")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['replan_flag']))

      names.append("replan_time_list")
      datas.append(str(replan_time_list))

      names.append("correct_path_for_limiter")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['correct_path_for_limiter']))

      names.append("correct_path_for_limiter_list")
      datas.append(str(correct_path_for_limiter_time_list))

      names.append("terminal_error_x")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['terminal_error_x']))

      names.append("terminal_error_y")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['terminal_error_y']))

      names.append("terminal_error_heading (deg)")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['terminal_error_heading'] * 57.3))

      names.append("stuck_time")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['stuck_time']))

      names.append("slot_occupied_ratio")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['slot_occupied_ratio']))

      names.append("current_path_length")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['current_path_length']))

      names.append("remain_dist")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['remain_dist']))

      names.append("car_static_timer_by_pos")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['car_static_timer_by_pos']))

      names.append("car_static_timer_by_vel")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['car_static_timer_by_vel']))

      names.append("static_flag")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['static_flag']))

      names.append("slot_width")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['slot_width']))

      names.append("slots_id")
      datas.append(str(bag_loader.plan_msg['data'][plan_msg_idx].successful_slot_info_list))

      names.append("plan_gear_cmd")
      datas.append(str(bag_loader.plan_msg['data'][plan_msg_idx].gear_command))

      names.append("plan_traj_available")
      datas.append(str(bag_loader.plan_msg['data'][plan_msg_idx].trajectory.available))

      names.append("optimization_terminal_pose_error")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['optimization_terminal_pose_error']))

      names.append("optimization_terminal_heading_error")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['optimization_terminal_heading_error']))

      names.append("lat_path_opt_cost_time_ms")
      datas.append(str(bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['lat_path_opt_cost_time_ms']))
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

  if bag_loader.uss_percept_msg['enable'] == True:
    model_x = []
    model_y = []
    post_x = []
    post_y = []
    for i in range(len(bag_loader.uss_percept_msg['data'][uss_percept_msg_idx].out_line_dataori)):
      for j in range(len(bag_loader.uss_percept_msg['data'][uss_percept_msg_idx].out_line_dataori[i].obj_pt)):
        x = bag_loader.uss_percept_msg['data'][uss_percept_msg_idx].out_line_dataori[i].obj_pt[j].x
        y = bag_loader.uss_percept_msg['data'][uss_percept_msg_idx].out_line_dataori[i].obj_pt[j].y
        if i == 0:
          post_x.append(x)
          post_y.append(y)
        elif i == 1:
          model_x.append(x)
          model_y.append(y)
    local_view_data['data_dluss_post'].data.update({
      'obj_pt_x': post_x,
      'obj_pt_y': post_y,
    })
    local_view_data['data_dluss_model'].data.update({
      'obj_pt_x': model_x,
      'obj_pt_y': model_y,
    })

  return local_view_data

def load_local_view_figure_parking():
  data_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
  data_car_target_pos = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
  data_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  data_car_circle = ColumnDataSource(data = {'car_circle_yn':[], 'car_circle_xn':[], 'car_circle_rn':[]})
  data_current_pos = ColumnDataSource(data = {'current_pos_y':[], 'current_pos_x':[]})
  data_ego = ColumnDataSource(data = {'ego_yn':[], 'ego_xn':[]})
  data_current_line = ColumnDataSource(data = {'y':[], 'x':[]})
  data_obs = ColumnDataSource(data = {'obs_x':[], 'obs_y':[]})
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
  data_wave_min = ColumnDataSource(data = {'wave_x': [], 'wave_y': [], 'radius':[], 'start_angle':[], 'end_angle':[]})
  data_wave_length_text = ColumnDataSource(data = {'wave_text_x': [], 'wave_text_y': [], 'length':[]})

  data_car_target_line = ColumnDataSource(data = {'y':[], 'x':[]})

  ctrl_debug_data = ColumnDataSource({
    'name':[],
    'data':[]
  })

  data_all_managed_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_all_managed_limiter = ColumnDataSource(data = {'limiter_point_y': [], 'limiter_point_x': [],})
  data_all_managed_occupied_slot = ColumnDataSource(data = {'occupied_slot_y': [], 'occupied_slot_x': [],})

  data_dluss_model = ColumnDataSource(data = {'obj_pt_y':[], 'obj_pt_x':[]})
  data_dluss_post = ColumnDataSource(data = {'obj_pt_y':[], 'obj_pt_x':[]})
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
                'uss_percept_msg_idx': 0,
               }
  local_view_data = {'data_car':data_car, \
                     'data_car_target_pos':data_car_target_pos, \
                     'data_car_box':data_car_box, \
                     'data_car_circle':data_car_circle, \
                     'data_current_pos': data_current_pos, \
                     'data_ego':data_ego, \
                     'data_current_line':data_current_line, \
                     'data_obs':data_obs, \
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
                     'data_wave_min':data_wave_min, \
                     'data_wave_length_text':data_wave_length_text, \
                     'data_all_managed_slot':data_all_managed_slot,\
                     'data_all_managed_limiter':data_all_managed_limiter,\
                     'data_all_managed_occupied_slot':data_all_managed_occupied_slot,\
                     'data_car_target_line':data_car_target_line,\
                     'data_dluss_model':data_dluss_model,\
                     'data_dluss_post':data_dluss_post,\
                     }
  ### figures config

  fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=800, match_aspect = True, aspect_scale=1)
  fig1.x_range.flipped = True
  # figure plot
  fig1.patch('car_yn', 'car_xn', source = data_car_target_pos, fill_color = "pink", line_color = "red", line_width = 1, line_alpha = 0.5, legend_label = 'car_target_pos')
  f1 = fig1.patch('car_yn', 'car_xn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, line_alpha = 0.5, legend_label = 'car')
  fig1.patches('y_vec', 'x_vec', source = data_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sampled carbox', visible = False)
  fig1.circle('current_pos_y','current_pos_x', source = data_current_pos, size=8, color='grey')
  fig1.circle(x ='car_circle_yn', y ='car_circle_xn', radius = 'car_circle_rn', source = data_car_circle, line_alpha = 0.5, line_width = 1, line_color = "blue", fill_alpha=0, legend_label = 'car_circle', visible = False)
  fig1.line('ego_yn', 'ego_xn', source = data_ego, line_width = 1.5, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig1.circle('obs_y', 'obs_x', source = data_obs, size=8, color='green', legend_label='obs')
  fig1.text(0.0, -2.0, text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt", legend_label = 'text')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 2.5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan')
  fig1.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 3.0, line_color = 'red', line_dash = 'solid', line_alpha = 0.8, legend_label = 'mpc')
  # fig1.line('dy_ref_mpc_vec', 'dx_ref_mpc_vec', source = data_ref_mpc_vec, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.5, legend_label = 'data_ref_mpc_vec')
  # fig1.line('dy_ref_vec', 'dx_ref_vec', source = data_ref_vec, line_width = 3.0, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'data_ref_vec')
  fig1.line('y', 'x', source = data_car_target_line, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.8, legend_label = 'car_target_line')
  fig1.line('y', 'x', source = data_current_line, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.8, legend_label = 'current_line')


  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_vision_parking, line_width = 3, line_color = 'lightgrey', line_dash = 'solid',legend_label = 'vision_parking_slot', visible = False)
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_fusion_parking, line_width = 2, line_color = 'red', line_dash = 'solid', line_alpha = 0.6, legend_label = 'fusion_parking_slot', visible = True)
  fig1.line('corner_point_y', 'corner_point_x', source = data_target_managed_slot, line_width = 3, line_color = 'green', line_dash = 'solid',legend_label = 'target_managed_slot')
  fig1.line('corner_point_y', 'corner_point_x', source = data_final_slot, line_width = 3, line_color = '#A52A2A', line_dash = 'dashed',legend_label = 'final_parking_slot')

  fig1.text(x = 'id_text_y', y = 'id_text_x', text = 'id', source = data_fusion_parking_id, text_color='red', text_align='center', text_font_size='10pt',legend_label = 'fusion_parking_slot', visible = True)

  fig1.wedge('wave_x','wave_y', 'radius', 'start_angle', 'end_angle',source = data_wave, fill_color="lavender", line_color="black",legend_label = 'uss_wave',alpha = 0.5, visible = False)
  fig1.wedge('wave_x','wave_y', 'radius', 'start_angle', 'end_angle',source = data_wave_min, fill_color="blue", line_color="black",legend_label = 'uss_wave',alpha = 0.8, visible = False)
  fig1.text(x = 'wave_text_x', y = 'wave_text_y', text = 'length', source = data_wave_length_text, text_color='black', text_align='center', text_font_size='10pt',legend_label = 'uss_wave', visible = False)

  # debug
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_all_managed_slot, line_width = 2, line_color = 'blue', line_dash = 'solid',legend_label = 'all managed slot', visible = False)
  fig1.line('limiter_point_y', 'limiter_point_x', source = data_all_managed_limiter, line_width = 3, line_color = 'blue', line_dash = 'solid', legend_label = 'managed limiter')
  fig1.patches('occupied_slot_y', 'occupied_slot_x', source = data_all_managed_occupied_slot, fill_color = "blue", line_color = "blue", line_width = 1, fill_alpha = 0.15, legend_label = 'all managed slot', visible = False)
  # dluss
  fig1.circle('obj_pt_y','obj_pt_x', source = data_dluss_post, size=3, color='orange', legend_label = 'dluss_post', visible = True)
  fig1.circle('obj_pt_y','obj_pt_x', source = data_dluss_model, size=3, color='blue', legend_label = 'dluss_model', visible = False)

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
  'acc_vel': [],
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
  acc_vel = []
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
    acc_vel.append(ctrl_json_data[i]['acc_vel'])
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
    'acc_vel': acc_vel,
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
  fig5.line('time', 'acc_vel', source = data_control_global, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'acc_vel')

  f6 = fig6.line('time', 'steer_angle_cmd', source = data_control_global, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer_angle_cmd')
  fig6.line('time', 'steer_angle_measure', source = data_control_global, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'steer_angle_measure')
  fig6.line('time', 'driver_hand_torque', source = data_control_global, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'driver_hand_torque')

  f7 = fig7.line('time', 'lat_err', source = data_control_global, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat_err')
  fig7.line('time', 'phi_err', source = data_control_global, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'phi_err')

  hover2 = HoverTool(renderers=[f2], tooltips=[('time', '@time'), ('controller_status', '@controller_status'), ('lon_enable', '@lon_enable'), ('lat_enable', '@lat_enable'),
                                              ('gear_plan', '@gear_plan')], mode='vline')

  hover3 = HoverTool(renderers=[f3], tooltips=[('time', '@time'), ('vel_cmd_plan', '@vel_cmd_plan'), ('vel_cmd_pos', '@vel_cmd_pos'),
                                              ('vel_measure', '@vel_measure')], mode='vline')

  hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time'), ('path_length_plan', '@path_length_plan'), ('remain_s_plan', '@remain_s_plan'),('remain_s_ctrl','@remain_s_ctrl'), ('remain_s_uss','@remain_s_uss'), ('remain_s_prebreak', '@remain_s_prebreak')], mode='vline')

  hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time'), ('vel_out', '@vel_out'), ('vel_KP_term', '@vel_KP_term'), ('vel_KI_term', '@vel_KI_term'), ('throttle_brake', '@throttle_brake'),('acc_vel', '@acc_vel')], mode='vline')

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
# params control the style of fig
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
  'line_alpha' : 0.6,
  'line_width' : 2,
  'legend_label' : 'fusion_parking_slot',
  'visible' : False
}
vision_slot_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "lightgrey",
  'line_width' : 3,
  'legend_label' : 'vision_parking_slot',
  'visible' : True
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
  'line_color' : "blue",
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
  'alpha' : 0.5,
  'visible' : False
}

uss_wave_params_min = {
  'fill_color' : 'blue',
  'line_color' : 'black',
  'legend_label' : 'uss_wave',
  'alpha' : 0.8,
  'visible' : False
}

uss_text_params = {
  'text_color' : 'black',
  'text_align' : 'center',
  'text_font_size' : '10pt',
  'legend_label' : 'uss_wave',
  'visible' : False
}

plan_params = {
  'line_width' : 2.5, 'line_color' : 'blue', 'line_dash' : 'solid', 'line_alpha' : 0.6, 'legend_label' : 'plan'
}

mpc_params = {
  'line_width' : 3.0, 'line_color' : 'red', 'line_dash' : 'solid', 'line_alpha' : 0.8, 'legend_label' : 'mpc'
}

all_managed_occupied_slot_params_apa = {
  'fill_color' : "blue",
  'line_color' : "blue",
  'line_width' : 1,
  'fill_alpha' : 0.15,
  'legend_label' : 'all managed slot',
  'visible' : False
}

all_managed_limiter_params_apa = {
  'line_width' : 3,
  'line_color' : 'blue',
  'line_dash' : 'solid',
  'legend_label' : 'managed limiter'
}

tlane_params = {
  'size' : 8, 'color' : 'green', 'legend_label' : 'obs'
}

table_params={
    'width': 600,
    'height':520,
}

current_line_params={
  'line_width' : 3.0,
  'line_color' : 'black',
  'line_dash' : 'solid',
  'line_alpha' : 0.8,
  'legend_label' : 'current_line'
}

target_line_params={
  'line_width' : 3.0,
  'line_color' : 'black',
  'line_dash' : 'solid',
  'line_alpha' : 0.8,
  'legend_label' : 'car_target_line'
}

target_pos_params={
  'fill_color' : "pink",
  'line_color' : "red",
  'line_width' : 1,
  'line_alpha' : 0.5,
  'legend_label' : 'car_target_pos'
}

sampled_carbox_params={
  'fill_color' : "#98FB98",
  'fill_alpha' : 0.0,
  'line_color' : "black",
  'line_width' : 1,
  'legend_label' : 'sampled carbox',
  'visible' : False
}

dluss_post_params={
  "size" : 3,
  "color" : 'orange',
  "legend_label" : 'dluss_post',
  "visible" : False
}

dluss_model_params={
  "size" : 3,
  "color" : 'blue',
  "legend_label" : 'dluss_model',
  "visible" : False
}

def apa_draw_local_view(dataLoader, layer_manager, max_time, time_step, plot_ctrl_flag=False):
    #define figure
    # define local_view fig
    fig_local_view = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=1000, match_aspect = True, aspect_scale=1)
    fig_local_view.x_range.flipped = True
    # toolbar
    fig_local_view.toolbar.active_scroll = fig_local_view.select_one(WheelZoomTool)
    # load the planning debug section information and load the timestamp of the planning input topic
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
    ctrl_debug_ts = []
    control_debug_timestamps = []
    plan_output_timestamps = []
    plan_debug_timestamps = []
    soc_timestamps = []
    wave_timestamps = []
    uss_percept_timestamps = []

    bag_time = 0.0
    while bag_time <= max_time + time_step / 2:
      # ctrl_debug_ts is the base time list
      ctrl_debug_ts.append(bag_time)

      loc_msg_idx = 0
      if dataLoader.loc_msg['enable'] == True:
        while dataLoader.loc_msg['t'][loc_msg_idx] <= bag_time and loc_msg_idx < (len(dataLoader.loc_msg['t'])-1):
            loc_msg_idx = loc_msg_idx + 1
        localization_timestamp = dataLoader.loc_msg['t'][loc_msg_idx]
        localization_timestamps.append(localization_timestamp)

      fus_msg_idx = 0
      if dataLoader.fus_msg['enable'] == True:
        while dataLoader.fus_msg['t'][fus_msg_idx] <= bag_time and fus_msg_idx < (len(dataLoader.fus_msg['t'])-1):
            fus_msg_idx = fus_msg_idx + 1
        fusion_object_timestamp = dataLoader.fus_msg['t'][fus_msg_idx]
        fusion_object_timestamps.append(fusion_object_timestamp)

      vs_msg_idx = 0
      if dataLoader.vs_msg['enable'] == True:
        while dataLoader.vs_msg['t'][vs_msg_idx] <= bag_time and vs_msg_idx < (len(dataLoader.vs_msg['t'])-1):
            vs_msg_idx = vs_msg_idx + 1
        vehicle_service_timestamp = dataLoader.vs_msg['t'][vs_msg_idx]
        vehicle_service_timestamps.append(vehicle_service_timestamp)

      fus_parking_msg_idx = 0
      if dataLoader.fus_parking_msg['enable'] == True:
        while dataLoader.fus_parking_msg['t'][fus_parking_msg_idx] <= bag_time and fus_parking_msg_idx < (len(dataLoader.fus_parking_msg['t'])-1):
          fus_parking_msg_idx = fus_parking_msg_idx + 1
        slot_timestamp = dataLoader.fus_parking_msg['t'][fus_parking_msg_idx]
        fusion_slot_timestamps.append(slot_timestamp)

      vis_parking_msg_idx = 0
      if dataLoader.vis_parking_msg['enable'] == True:
        while dataLoader.vis_parking_msg['t'][vis_parking_msg_idx] <= bag_time and vis_parking_msg_idx < (len(dataLoader.vis_parking_msg['t'])-1):
          vis_parking_msg_idx = vis_parking_msg_idx + 1
        vis_slot_timestamp = dataLoader.vis_parking_msg['t'][vis_parking_msg_idx]
        vision_slot_timestamps.append(vis_slot_timestamp)

      plan_msg_idx = 0
      if dataLoader.plan_msg['enable'] == True:
        while dataLoader.plan_msg['t'][plan_msg_idx] <= bag_time and plan_msg_idx < (len(dataLoader.plan_msg['t'])-1):
            plan_msg_idx = plan_msg_idx + 1
        plan_timestamp = dataLoader.plan_msg['t'][plan_msg_idx]
        plan_output_timestamps.append(plan_timestamp)

      plan_debug_msg_idx = 0
      if dataLoader.plan_debug_msg['enable'] == True:
        while dataLoader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(dataLoader.plan_debug_msg['t'])-1):
            plan_debug_msg_idx = plan_debug_msg_idx + 1
        plan_debug_timestamp = dataLoader.plan_debug_msg['t'][plan_debug_msg_idx]
        plan_debug_timestamps.append(plan_debug_timestamp)

      ctrl_msg_idx = 0
      if dataLoader.ctrl_msg['enable'] == True:
        while dataLoader.ctrl_msg['t'][ctrl_msg_idx] <= bag_time and ctrl_msg_idx < (len(dataLoader.ctrl_msg['t'])-1):
            ctrl_msg_idx = ctrl_msg_idx + 1
        control_output_timestamp = dataLoader.ctrl_msg['t'][ctrl_msg_idx]
        control_output_timestamps.append(control_output_timestamp)

      ctrl_debug_msg_idx = 0
      if dataLoader.ctrl_debug_msg['enable'] == True:
        while dataLoader.ctrl_debug_msg['t'][ctrl_debug_msg_idx] <= bag_time and ctrl_debug_msg_idx < (len(dataLoader.ctrl_debug_msg['t'])-1):
            ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
        ctrl_debug_timestamp = dataLoader.ctrl_debug_msg['t'][ctrl_debug_msg_idx]
        control_debug_timestamps.append(ctrl_debug_timestamp)

      soc_state_msg_idx = 0
      if dataLoader.soc_state_msg['enable'] == True:
        while dataLoader.soc_state_msg['t'][soc_state_msg_idx] <= bag_time and soc_state_msg_idx < (len(dataLoader.soc_state_msg['t'])-1):
            soc_state_msg_idx = soc_state_msg_idx + 1
        soc_timestamp = dataLoader.soc_state_msg['t'][soc_state_msg_idx]
        soc_timestamps.append(soc_timestamp)

      wave_msg_idx = 0
      if dataLoader.wave_msg['enable'] == True:
        while dataLoader.wave_msg['t'][wave_msg_idx] <= bag_time and wave_msg_idx < (len(dataLoader.wave_msg['t'])-1):
            wave_msg_idx = wave_msg_idx + 1
        wave_timestamp = dataLoader.wave_msg['t'][wave_msg_idx]
        wave_timestamps.append(wave_timestamp)

      uss_percept_msg_idx = 0
      if dataLoader.uss_percept_msg['enable'] == True:
        while dataLoader.uss_percept_msg['t'][uss_percept_msg_idx] <= bag_time and uss_percept_msg_idx < (len(dataLoader.uss_percept_msg['t'])-1):
            uss_percept_msg_idx = uss_percept_msg_idx + 1
        uss_timestamp = dataLoader.uss_percept_msg['t'][uss_percept_msg_idx]
        uss_percept_timestamps.append(uss_timestamp)

      bag_time += time_step

  # load self car info
    ego_car_generate = CommonGenerator()
    ego_center_generate = CommonGenerator()
    ego_circle_generate = CircleGenerator()
    current_line_generate = CommonGenerator()
    for localization_timestamp in localization_timestamps:
      # car pos
      temp_cur_pos_xn = []
      temp_cur_pos_yn = []
      # car circle
      car_circle_xn = []
      car_circle_yn = []
      car_circle_rn = []
      car_center_xn = []
      car_center_yn = []
      # current line
      current_line_xn = []
      current_line_yn = []

      flag, loc_msg = findrt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
        # location_generator.xys.append(([],[]))
      else:
        cur_pos_xn = loc_msg.pose.local_position.x
        cur_pos_yn = loc_msg.pose.local_position.y
        cur_pos_theta = loc_msg.pose.euler_angles.yaw
        car_center_xn.append(cur_pos_xn)
        car_center_yn.append(cur_pos_yn)

        # target line
        half_car_width = 0.9
        heading_vec = [math.cos(cur_pos_theta), math.sin(cur_pos_theta)]
        norm_vec_1 = [-half_car_width * heading_vec[1], half_car_width * heading_vec[0]]
        norm_vec_2 = [half_car_width * heading_vec[1], -half_car_width * heading_vec[0]]
        x1 = cur_pos_xn + norm_vec_1[0]
        y1 = cur_pos_yn + norm_vec_1[1]
        x2 = cur_pos_xn + norm_vec_2[0]
        y2 = cur_pos_yn + norm_vec_2[1]
        current_line_xn = [x1, x2]
        current_line_yn = [y1, y2]

        # car pos
        for ego_i in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[ego_i], car_yb[ego_i], cur_pos_xn, cur_pos_yn, cur_pos_theta)
          temp_cur_pos_xn.append(tmp_x)
          temp_cur_pos_yn.append(tmp_y)
        # car circle
        for i in range(len(car_circle_x)):
          tmp_x, tmp_y = local2global(car_circle_x[i], car_circle_y[i], cur_pos_xn, cur_pos_yn, cur_pos_theta)
          car_circle_xn.append(tmp_x)
          car_circle_yn.append(tmp_y)
          car_circle_rn.append(car_circle_r[i])
      # car rear axle center
      ego_center_generate.xys.append((car_center_yn,car_center_xn))
      ego_car_generate.xys.append(([temp_cur_pos_yn],[temp_cur_pos_xn]))
      ego_circle_generate.xys.append((car_circle_yn,car_circle_xn, car_circle_rn))
      current_line_generate.xys.append((current_line_yn,current_line_xn))

    ego_car_generate.ts = np.array(ctrl_debug_ts)
    ego_center_generate.ts = np.array(ctrl_debug_ts)
    ego_circle_generate.ts = np.array(ctrl_debug_ts)
    current_line_generate.ts = np.array(ctrl_debug_ts)

  # load location
    location_generator = CommonGenerator()
    # cur_pos_xn0 = cur_pos_xn = dataLoader.loc_msg['data'][0].pose.local_position.x
    # cur_pos_yn0 = cur_pos_yn = dataLoader.loc_msg['data'][0].pose.local_position.y
    for localization_timestamp in localization_timestamps:
      ego_xb, ego_yb = [], []
      # ego_xn, ego_yn = [], []
      flag, loc_msg = findrt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
      else:
        cur_pos_xn = loc_msg.pose.local_position.x
        cur_pos_yn = loc_msg.pose.local_position.y
        cur_yaw = loc_msg.pose.euler_angles.yaw
        ### global variables
        # pos offset
        for i in range(len(dataLoader.loc_msg['data'])):
          if (i % 10 != 0):
            continue
          pos_xn_i = dataLoader.loc_msg['data'][i].pose.local_position.x
          pos_yn_i = dataLoader.loc_msg['data'][i].pose.local_position.y

          # ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

          ego_xb.append(pos_xn_i)
          ego_yb.append(pos_yn_i)
          # ego_xn.append(pos_xn_i - cur_pos_xn0)
          # ego_yn.append(pos_yn_i - cur_pos_yn0)
      location_generator.xys.append((ego_yb,ego_xb))
    location_generator.ts = np.array(ctrl_debug_ts)

  # load vs and soc
    vs_text_generator = TextGenerator()
    for loc_i, localization_timestamp in enumerate(localization_timestamps):
      vel_text = []
      vel_x, vel_y = [], []
      flag, loc_msg = findrt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
      else:
        vel_ego =  loc_msg.pose.linear_velocity_from_wheel

        flag, soc_msg = findrt(dataLoader.soc_state_msg, soc_timestamps[loc_i])
        if not flag:
          print('find soc_msg error')
          current_state = -1
        else:
          current_state = soc_msg.current_state

        flag, vs_msg = findrt(dataLoader.vs_msg, vehicle_service_timestamps[loc_i])
        if not flag:
          print('find vs_msg error')
          steer_deg = 600
        else:
          steer_deg = vs_msg.steering_wheel_angle * 57.3

        # remain_s_ctrl = -1
        # ctrl_debug_msg_idx = 0
        # if dataLoader.ctrl_debug_msg['enable'] == True:
        #   while dataLoader.ctrl_debug_msg['t'][ctrl_debug_msg_idx] <= localization_timestamp and ctrl_debug_msg_idx < (len(dataLoader.ctrl_debug_msg['t'])-1):
        #       ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
        #   remain_s_ctrl = dataLoader.ctrl_debug_msg['json'][ctrl_debug_msg_idx]['remain_s_ctrl'] * 100

        flag, ctrl_debug_msg = findrt_json(dataLoader.ctrl_debug_msg, control_debug_timestamps[loc_i])
        if not flag:
          print('find ctrl_debug_msg error')
          remain_s_ctrl = -1
        else:
          remain_s_ctrl = ctrl_debug_msg['remain_s_ctrl'] * 100

        text = 'v = {:.2f} m/s, remain_s_ctrl = {:.1f} cm, steer = {:.1f} deg, state = {:d}'.format(round(vel_ego, 2), round(remain_s_ctrl, 1), round(steer_deg, 1), current_state)
        vel_text.append(text)
        vel_x.append(-2)
        vel_y.append(0)
      vs_text_generator.xys.append((vel_y, vel_x, vel_text))
    vs_text_generator.ts = np.array(ctrl_debug_ts)

  # load apa slot
    vision_slot_generate = CommonGenerator()
    fusion_slot_generate = CommonGenerator()
    target_slot_generate = CommonGenerator()
    final_slot_generate = CommonGenerator()
    all_slot_generate = CommonGenerator()
    slot_id_generate = TextGenerator()
    all_managed_occupied_slot_generate = CommonGenerator()
    all_managed_limiter_generate = CommonGenerator()
    tlane_generate = CommonGenerator()
    for slot_i, slot_timestamp in enumerate(fusion_slot_timestamps):
        flag, fusion_slot_msg = findrt(dataLoader.fus_parking_msg, slot_timestamp)
        if not flag:
            print('find fusion_slot_msg error')
            fusion_slot_generate.xys.append(([], []))
            slot_id_generate.xys.append(([], [], []))
        else:
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

        vis_parking_msg_idx = 0
        if dataLoader.vis_parking_msg['enable'] == True:
          while dataLoader.vis_parking_msg['t'][vis_parking_msg_idx] <= slot_timestamp and vis_parking_msg_idx < (len(dataLoader.vis_parking_msg['t'])-1):
            vis_parking_msg_idx = vis_parking_msg_idx + 1
        # vision_slot_msg = dataLoader.vis_parking_msg['data'][vis_parking_msg_idx]

      # final slot
        if dataLoader.fus_parking_msg['enable'] == True:
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
      # visual slot
        if dataLoader.vis_parking_msg['enable'] == True:
          temp_corner_x_list = []
          temp_corner_y_list = []
          parking_fusion_slot_lists = dataLoader.vis_parking_msg['data'][vis_parking_msg_idx].parking_slot
          if dataLoader.loc_msg['enable'] == True:
            flag, loc_msg = findrt(dataLoader.loc_msg, localization_timestamps[slot_i])
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
        if dataLoader.fus_parking_msg['enable'] == True and dataLoader.plan_debug_msg['enable'] == True:
          # slot_management_info = dataLoader.plan_debug_msg['data'][slot_i].slot_management_info
          all_managed_slot_x_vec = []
          all_managed_slot_y_vec = []
          temp_corner_x_list = []
          temp_corner_y_list = []
          occupied_x_vec = []
          occupied_y_vec = []
          limiter_x_vec = []
          limiter_y_vec = []
          flag, plan_msg = findrt(dataLoader.plan_debug_msg, plan_debug_timestamps[slot_i])
          if not flag:
            print('find plan_msg error')
          flag, plan_json = findrt_json(dataLoader.plan_debug_msg, plan_debug_timestamps[slot_i])
          if not flag:
            print('find plan_msg error')
          else:
            slot_management_info = plan_msg.slot_management_info
            # select_slot_id = dataLoader.fus_parking_msg['data'][slot_i].select_slot_id
            select_slot_id = fusion_slot_msg.select_slot_id

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
              if maganed_slot_vec.is_occupied:
                  occupied_x_vec.append(slot_x)
                  occupied_y_vec.append(slot_y)

            for limiter in slot_management_info.limiter_points:
              limiter_x_vec.append(limiter.x)
              limiter_y_vec.append(limiter.y)

            obstacle_x = plan_json['obstaclesX']
            obstacle_y = plan_json['obstaclesY']
            # tmp
            slot_corner_X = plan_json['slot_corner_X']
            slot_corner_Y = plan_json['slot_corner_Y']
            limiter_corner_X = plan_json['limiter_corner_X']
            limiter_corner_Y = plan_json['limiter_corner_Y']
            temp_corner_x_list = [[slot_corner_X[0], slot_corner_X[2], slot_corner_X[3], slot_corner_X[1]]]
            temp_corner_y_list = [[slot_corner_Y[0], slot_corner_Y[2], slot_corner_Y[3], slot_corner_Y[1]]]
            limiter_y_vec = limiter_corner_Y
            limiter_x_vec = limiter_corner_X

          target_slot_generate.xys.append((temp_corner_y_list, temp_corner_x_list))
          all_slot_generate.xys.append((all_managed_slot_y_vec, all_managed_slot_x_vec))
          all_managed_occupied_slot_generate.xys.append((occupied_y_vec, occupied_x_vec))
          all_managed_limiter_generate.xys.append((limiter_y_vec, limiter_x_vec))
          tlane_generate.xys.append((obstacle_y, obstacle_x))

  # load planning traj
    plan_generator = CommonGenerator()
    target_line_generator = CommonGenerator()
    target_pos_generator = CommonGenerator()
    car_box_generator = CommonGenerator()
    for plan_timestamp in plan_output_timestamps:
      flag, plan_msg = findrt(dataLoader.plan_msg, plan_timestamp)
      if not flag:
        print('find plan error')
        plan_traj_x, plan_traj_y, plan_heading = [], [], []
        target_line_xn, target_line_yn = [], []
        target_pos_xn, target_pos_yn = [], []
        car_box_x_vec, car_box_y_vec = [], []
      else:
        trajectory = plan_msg.trajectory
        plan_traj_x, plan_traj_y, plan_heading = [], [], []
        target_line_xn, target_line_yn = [], []
        target_pos_xn, target_pos_yn = [], []
        car_box_x_vec, car_box_y_vec = [], []
        for j in range(len(trajectory.trajectory_points)):
          plan_traj_x.append(trajectory.trajectory_points[j].x)
          plan_traj_y.append(trajectory.trajectory_points[j].y)
          plan_heading.append(trajectory.trajectory_points[j].heading_yaw)

        if (len(plan_traj_x) > 1):
          half_car_width = 0.9
          last_x = plan_traj_x[-1]
          last_y = plan_traj_y[-1]
          last_heading = plan_heading[-1]
          for ego_i in range(len(car_xb)):
            tmp_x, tmp_y = local2global(car_xb[ego_i], car_yb[ego_i], last_x, last_y, last_heading)
            target_pos_xn.append(tmp_x)
            target_pos_yn.append(tmp_y)

          heading_vec = [math.cos(last_heading), math.sin(last_heading)]
          norm_vec_1 = [-half_car_width * heading_vec[1], half_car_width * heading_vec[0]]
          norm_vec_2 = [half_car_width * heading_vec[1], -half_car_width * heading_vec[0]]
          x1 = last_x + norm_vec_1[0]
          y1 = last_y + norm_vec_1[1]
          x2 = last_x + norm_vec_2[0]
          y2 = last_y + norm_vec_2[1]
          target_line_xn = [x1, x2]
          target_line_yn = [y1, y2]

          for i in range(len(plan_traj_x)):
            car_xn = []
            car_yn = []
            for j in range(len(car_xb)):
              tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], plan_traj_x[i], plan_traj_y[i], plan_heading[i])
              car_xn.append(tmp_x)
              car_yn.append(tmp_y)
            if (i%10==0):
              car_box_x_vec.append(car_xn)
              car_box_y_vec.append(car_yn)

      plan_generator.xys.append((plan_traj_y, plan_traj_x))
      target_line_generator.xys.append((target_line_yn,target_line_xn))
      target_pos_generator.xys.append(([target_pos_yn],[target_pos_xn]))
      car_box_generator.xys.append((car_box_y_vec,car_box_x_vec))

    target_pos_generator.ts = np.array(ctrl_debug_ts)
    plan_generator.ts = np.array(ctrl_debug_ts)
    target_line_generator.ts = np.array(ctrl_debug_ts)
    car_box_generator.ts = np.array(ctrl_debug_ts)

  # load mpc traj
    mpc_generator = CommonGenerator()
    for mpc_i, control_timestamp in enumerate(control_output_timestamps):
      flag, mpc_msg = findrt(dataLoader.ctrl_msg, control_timestamp)
      if not flag:
        print('find mpc error')
        mpc_dx, mpc_dy = [], []
      else:
        flag, loc_msg = findrt(dataLoader.loc_msg, localization_timestamps[mpc_i])
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
    mpc_generator.ts = np.array(ctrl_debug_ts)

  # load uss
    uss_generator = WedgesGenerator()
    uss_text_generator = TextGenerator()
    wave_min_generator = WedgesGenerator()
    for loc_i, localization_timestamp in enumerate(localization_timestamps):
      sector_x, sector_y, rs, start_angle, end_angle, length= [], [], [], [], [], []
      text_x, text_y = [], []
      sector_x_min, sector_y_min, rs_min, start_angle_min, end_angle_min = [], [], [], [], []
      uss_available = False
      uss_remain_dist = 0.0
      uss_index = 0
      uss_car_index = 0
      flag, loc_msg = findrt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
      else:
        wave_msg_idx = 0
        if dataLoader.wave_msg['enable'] == True:
          flag, wave_msg = findrt(dataLoader.wave_msg, wave_timestamps[loc_i])
          if not flag:
            print('find wave_msg error')
          else:
            #get cur pose and uss wave
            upa_dis_info_bufs = wave_msg.upa_dis_info_buf
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
                  rs.append(rs1)
                  length.append(rs0)
                  start_angle.append(uss_angle_start)
                  end_angle.append(uss_angle_end)
                  m += 1

            flag, plan_json = findrt_json(dataLoader.plan_debug_msg, plan_debug_timestamps[loc_i])
            if not flag:
              print('find plan_msg error')
            else:
              uss_available = plan_json['uss_available']
              uss_available = bool(uss_available)
              uss_remain_dist = plan_json['uss_remain_dist']
              uss_index = plan_json['uss_index']
              uss_index = int(uss_index)
              uss_car_index = plan_json['uss_car_index']
              uss_car_index = int(uss_car_index)
            flag, soc_msg = findrt(dataLoader.soc_state_msg, soc_timestamps[loc_i])
            if not flag:
              print('find soc_msg error')
              current_state = -1
            else:
              current_state = soc_msg.current_state

            if uss_available == True and uss_index <= len(sector_x) and current_state >= 29:
              sector_y_min, sector_x_min = [sector_y[uss_index]], [sector_x[uss_index]]
              rs_min, start_angle_min, end_angle_min = [rs[uss_index]], [start_angle[uss_index]], [end_angle[uss_index]]

      uss_generator.xys.append((sector_y, sector_x, rs, start_angle, end_angle))
      uss_text_generator.xys.append((text_y, text_x, length))
      wave_min_generator.xys.append((sector_y_min, sector_x_min, rs_min, start_angle_min, end_angle_min))

    uss_generator.ts = np.array(ctrl_debug_ts)
    uss_text_generator.ts = np.array(ctrl_debug_ts)
    wave_min_generator.ts = np.array(ctrl_debug_ts)

  # uss perception
    uss_post_generator = CommonGenerator()
    uss_model_generator = CommonGenerator()
    for uss_i, uss_percept_timestamp in enumerate(uss_percept_timestamps):
      flag, uss_percept_msg = findrt(dataLoader.uss_percept_msg, uss_percept_timestamp)
      model_x, model_y = [], []
      post_x, post_y = [], []
      if not flag:
        print('find uss percept error')
      else:
        for i in range(len(uss_percept_msg.out_line_dataori)):
          for j in range(len(uss_percept_msg.out_line_dataori[i].obj_pt)):
            x = uss_percept_msg.out_line_dataori[i].obj_pt[j].x
            y = uss_percept_msg.out_line_dataori[i].obj_pt[j].y
            if i == 0:
              post_x.append(x)
              post_y.append(y)
            elif i == 1:
              model_x.append(x)
              model_y.append(y)

      uss_post_generator.xys.append((post_y, post_x))
      uss_model_generator.xys.append((model_y, model_x))
    uss_post_generator.ts = np.array(ctrl_debug_ts)
    uss_model_generator.ts = np.array(ctrl_debug_ts)


  # move Layer here for manage
    if dataLoader.plan_msg['enable'] == True:
      target_pos_layer = PatchLayer(fig_local_view , target_pos_params)
      layer_manager.AddLayer(target_pos_layer, 'target_pos_layer', target_pos_generator, 'target_pos_generator', 2)
    # self car info
    ego_car_layer = PatchLayer(fig_local_view ,ego_car_params_apa)
    layer_manager.AddLayer(ego_car_layer, 'ego_car_layer', ego_car_generate, 'ego_car_generate', 2)
    ego_center_layer = DotLayer(fig_local_view ,ego_dot_params_apa)
    layer_manager.AddLayer(ego_center_layer, 'ego_center_layer', ego_center_generate, 'ego_center_generate', 2)

    if dataLoader.plan_msg['enable'] == True:
      car_box_layer = PatchLayer(fig_local_view , sampled_carbox_params)
      layer_manager.AddLayer(car_box_layer, 'car_box_layer', car_box_generator, 'car_box_generator', 2)

    ego_circle_layer = CircleLayer(fig_local_view ,ego_circle_params_apa)
    layer_manager.AddLayer(ego_circle_layer, 'ego_circle_layer', ego_circle_generate, 'ego_circle_generate', 3)

    # location
    if dataLoader.loc_msg['enable'] == True:
      location_layer = CurveLayer(fig_local_view, location_params_apa)
      layer_manager.AddLayer(location_layer, 'location_layer', location_generator, 'location_generator', 2)

    # vs and soc
    if dataLoader.soc_state_msg['enable'] == True and dataLoader.vs_msg['enable'] == True and dataLoader.ctrl_debug_msg['enable'] == True:
      vs_layer = TextLayer(fig_local_view, vs_car_params_apa)
      layer_manager.AddLayer(vs_layer, 'vs_layer', vs_text_generator, 'vs_text_generator', 3)
    # apa slot
    if dataLoader.vis_parking_msg['enable'] == True:
      vision_slot_generate.ts = np.array(ctrl_debug_ts)
      vision_slot_layer = MultiCurveLayer(fig_local_view ,vision_slot_params_apa)
      layer_manager.AddLayer(vision_slot_layer, 'vision_slot_layer',vision_slot_generate,'vision_slot_generate',2)

    fusion_slot_generate.ts = np.array(ctrl_debug_ts)
    slot_id_generate.ts = np.array(ctrl_debug_ts)
    slot_layer = MultiCurveLayer(fig_local_view ,fusion_slot_params_apa)
    slot_id_layer = TextLayer(fig_local_view,slot_id_params_apa)
    layer_manager.AddLayer(slot_layer, 'slot_layer', fusion_slot_generate, 'fusion_slot_generate', 2)
    layer_manager.AddLayer(slot_id_layer, 'slot_id_layer',slot_id_generate,'slot_id_generate',3)

    if dataLoader.fus_parking_msg['enable'] == True:
      final_slot_generate.ts = np.array(ctrl_debug_ts)
      final_slot_layer = MultiCurveLayer(fig_local_view ,final_slot_params_apa)
      layer_manager.AddLayer(final_slot_layer, 'final_slot_layer',final_slot_generate,'final_slot_generate',2)

    if dataLoader.fus_parking_msg['enable'] == True and dataLoader.plan_debug_msg['enable'] == True:
      target_slot_generate.ts = np.array(ctrl_debug_ts)
      all_slot_generate.ts = np.array(ctrl_debug_ts)
      all_managed_occupied_slot_generate.ts = np.array(ctrl_debug_ts)
      all_managed_limiter_generate.ts = np.array(ctrl_debug_ts)
      tlane_generate.ts = np.array(ctrl_debug_ts)
      target_slot_layer = MultiCurveLayer(fig_local_view ,target_slot_params_apa)
      all_slot_layer = MultiCurveLayer(fig_local_view ,all_slot_params_apa)
      all_managed_occupied_slot_layer = PatchLayer(fig_local_view, all_managed_occupied_slot_params_apa)
      all_managed_limiter_layer = CurveLayer(fig_local_view, all_managed_limiter_params_apa)
      tlane_layer = DotLayer(fig_local_view, tlane_params)

      layer_manager.AddLayer(target_slot_layer, 'target_slot_layer',target_slot_generate,'target_slot_generate',2)
      layer_manager.AddLayer(all_slot_layer, 'all_slot_layer',all_slot_generate,'all_slot_generate',2)
      layer_manager.AddLayer(all_managed_occupied_slot_layer, 'all_managed_occupied_slot_layer',all_managed_occupied_slot_generate,'all_managed_occupied_slot_generate',2)
      layer_manager.AddLayer(all_managed_limiter_layer, 'all_managed_limiter_layer',all_managed_limiter_generate,'all_managed_limiter_generate',2)
      layer_manager.AddLayer(tlane_layer, 'tlane_layer',tlane_generate,'tlane_generate',2)

    # planning traj
    if dataLoader.plan_msg['enable'] == True:
      plan_layer = CurveLayer(fig_local_view, plan_params)
      layer_manager.AddLayer(plan_layer, 'plan_layer', plan_generator, 'plane_generator', 2)

    # mpc
    if dataLoader.ctrl_msg['enable'] == True:
      mpc_layer = CurveLayer(fig_local_view, mpc_params)
      layer_manager.AddLayer(mpc_layer, 'mpc_layer', mpc_generator, 'mpc_generator', 2)

    if dataLoader.plan_msg['enable'] == True:
      target_line_layer = CurveLayer(fig_local_view, target_line_params)
      layer_manager.AddLayer(target_line_layer, 'target_line_layer', target_line_generator, 'target_line_generator', 2)

    if dataLoader.loc_msg['enable'] == True:
      current_line_layer = CurveLayer(fig_local_view, current_line_params)
      layer_manager.AddLayer(current_line_layer, 'current_line_layer', current_line_generate, 'current_line_generate', 2)

    # uss
    if dataLoader.wave_msg['enable'] == True:
      uss_layer = MultiWedgesLayer(fig_local_view, uss_wave_params)
      layer_manager.AddLayer(uss_layer, 'uss_layer', uss_generator, 'uss_generator', 5)
      if dataLoader.plan_debug_msg['enable'] == True and dataLoader.soc_state_msg['enable'] == True:
        uss_min_layer = MultiWedgesLayer(fig_local_view, uss_wave_params_min)
        layer_manager.AddLayer(uss_min_layer, 'uss_min_layer', wave_min_generator, 'wave_min_generator', 5)
      uss_layer_text = TextLayer(fig_local_view, uss_text_params)
      layer_manager.AddLayer(uss_layer_text, 'uss_layer_text', uss_text_generator, 'uss_text_generator', 3)

    # uss perception
    if dataLoader.uss_percept_msg['enable'] == True:
      dluss_post_layer = DotLayer(fig_local_view ,dluss_post_params)
      layer_manager.AddLayer(dluss_post_layer, 'dluss_post_layer', uss_post_generator, 'uss_post_generator', 2)
      dluss_model_layer = DotLayer(fig_local_view ,dluss_model_params)
      layer_manager.AddLayer(dluss_model_layer, 'dluss_model_layer', uss_model_generator, 'uss_model_generator', 2)

  # legend
    fig_local_view.legend.click_policy = 'hide'
  # Table
    if plot_ctrl_flag == False:
      return fig_local_view, None
    else:
      data_ctrl_debug_data = TextGenerator()
      for plan_i, plan_timestamp in enumerate(plan_output_timestamps):
        names = []
        datas = []
        flag, plan_msg = findrt(dataLoader.plan_msg, plan_timestamp)
        if not flag:
          print('find plan error')
        flag, plan_json = findrt_json(dataLoader.plan_debug_msg, plan_debug_timestamps[plan_i])
        if not flag:
          print('find plan_debug error')
        else:
          names.append('apa_planning_status')
          datas.append(str(plan_msg.planning_status.apa_planning_status))

          names.append("planning_stm")
          datas.append(str(plan_json['planning_status']))

          names.append("replan_reason")
          datas.append(str(plan_json['replan_reason']))

          names.append("path_plan_success")
          datas.append(str(plan_json['path_plan_success']))

          names.append("path_plan_result")
          datas.append(str(plan_json['pathplan_result']))

          names.append("terminal_error_x")
          datas.append(str(plan_json['terminal_error_x']))

          names.append("terminal_error_y")
          datas.append(str(plan_json['terminal_error_y']))

          names.append("terminal_error_heading (deg)")
          datas.append(str(plan_json['terminal_error_heading'] * 57.3))

          names.append("stuck_time")
          datas.append(str(plan_json['stuck_time']))

          names.append("slot_occupied_ratio")
          datas.append(str(plan_json['slot_occupied_ratio']))

          names.append("current_path_length")
          datas.append(str(plan_json['current_path_length']))

          names.append("remain_dist")
          datas.append(str(plan_json['remain_dist']))

          names.append("car_static_timer_by_pos")
          datas.append(str(plan_json['car_static_timer_by_pos']))

          names.append("car_static_timer_by_vel")
          datas.append(str(plan_json['car_static_timer_by_vel']))

          names.append("static_flag")
          datas.append(str(plan_json['static_flag']))

          names.append("slot_width")
          datas.append(str(plan_json['slot_width']))

          names.append("slots_id")
          datas.append(str(plan_msg.successful_slot_info_list))

          names.append("plan_gear_cmd")
          datas.append(str(plan_msg.gear_command))

          names.append("plan_traj_available")
          datas.append(str(plan_msg.trajectory.available))

          names.append("optimization_terminal_pose_error")
          datas.append(str(plan_json['optimization_terminal_pose_error']))

          names.append("optimization_terminal_heading_error")
          datas.append(str(plan_json['optimization_terminal_heading_error']))

          names.append("lat_path_opt_cost_time_ms")
          datas.append(str(plan_json['lat_path_opt_cost_time_ms']))

        flag, soc_msg = findrt(dataLoader.soc_state_msg, soc_timestamps[plan_i])
        if not flag:
          print('find soc_msg error')
        else:
          names.append("current_state")
          datas.append(str(soc_msg.current_state))

        flag, vs_msg = findrt(dataLoader.vs_msg, vehicle_service_timestamps[plan_i])
        if not flag:
          print('find soc_msg error')
        else:
          names.append("long_control_actuator_status")
          datas.append(str(vs_msg.parking_long_control_actuator_status))
          names.append("lat_control_actuator_status")
          datas.append(str(vs_msg.parking_lat_control_actuator_status))
          names.append("shift_lever_state")
          datas.append(str(vs_msg.shift_lever_state))
          names.append("shift_lever_state_available")
          datas.append(str(vs_msg.shift_lever_state_available))

        flag, ctrl_debug_msg = findrt_json(dataLoader.ctrl_debug_msg, control_debug_timestamps[plan_i])
        if not flag:
          print('find ctrl_debug_msg error')
        else:
          names.append("lat_mpc_status")
          datas.append(ctrl_debug_msg['lat_mpc_status'])
          names.append("remain_s_uss")
          datas.append(ctrl_debug_msg['remain_s_uss'])
          names.append("remain_s_ctrl")
          datas.append(ctrl_debug_msg['remain_s_ctrl'])
          names.append("vel_ref_gain")
          datas.append(ctrl_debug_msg['vel_ref_gain'])
          names.append("acc_vel")
          datas.append(ctrl_debug_msg['acc_vel'])
          names.append("slope_acc")
          datas.append(ctrl_debug_msg['slope_acc'])
          names.append("apa_enable")
          datas.append(ctrl_debug_msg['apa_enable'])
          names.append("vehicle_stationary_flag")
          datas.append(ctrl_debug_msg['vehicle_stationary_flag'])
          names.append("apa_finish_flag")
          datas.append(ctrl_debug_msg['apa_finish_flag'])
          names.append("emergency_stop_flag")
          datas.append(ctrl_debug_msg['emergency_stop_flag'])
          names.append("break_override_flag")
          datas.append(ctrl_debug_msg['break_override_flag'])
          names.append("gear_shifting_flag")
          datas.append(ctrl_debug_msg['gear_shifting_flag'])
          names.append("gear_cmd")
          datas.append(ctrl_debug_msg['gear_cmd'])
          names.append("gear_real")
          datas.append(ctrl_debug_msg['gear_real'])
        data_ctrl_debug_data.xys.append((names, datas, [None] * len(names)))
      data_ctrl_debug_data.ts = np.array(ctrl_debug_ts)
      tab_attr_list = ['name', 'data']
      tab_debug_layer = TableLayerV2(None, tab_attr_list, table_params)
      layer_manager.AddLayer(
        tab_debug_layer, 'tab_debug_layer', data_ctrl_debug_data, 'data_ctrl_debug_data', 3)


      return fig_local_view, tab_debug_layer.plot

def apa_draw_local_view_parking_ctrl(dataLoader, layer_manager, max_time, time_step = 0.02):
    print(f'max time is {max_time}')
    assert dataLoader.ctrl_debug_msg['enable'] == True
    # not show all ctrl_debug_msg.
    bag_time = 0.0
    t_debug = []
    ctrl_json_data = []

    while bag_time <= max_time + time_step / 2:
      ctrl_debug_msg_idx = 0
      while dataLoader.ctrl_debug_msg['t'][ctrl_debug_msg_idx] <= bag_time and ctrl_debug_msg_idx < (len(dataLoader.ctrl_debug_msg['t'])-1):
          ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
      ctrl_debug_json = dataLoader.ctrl_debug_msg['json'][ctrl_debug_msg_idx]
      t_debug.append(bag_time)
      ctrl_json_data.append(ctrl_debug_json)
      bag_time += time_step
    # ## using 0.02 as time step，for align with Jupyter
    # ctrl_json_data = dataLoader.ctrl_debug_msg['json']
    # t_debug = []
    # t = 0.0
    # for i in range(len(ctrl_json_data)):
    #   t_debug.append(t)
    #   t = t + time_step

    json_value_list = ["controller_status", "lat_enable", "lon_enable", "gear_plan",
                        "vel_ref", "vel_cmd", "vel_ego",
                        "path_length_plan", "remain_s_plan", "remain_s_prebreak", "remain_s_uss", "remain_s_ctrl",
                        "vel_out", "vel_KP_term", "vel_KI_term", "throttle_brake", 'acc_vel',
                        "steer_angle_cmd", "steer_angle", "driver_hand_torque",
                        "lat_err", "phi_err"
                        ]

    # json_value_xys_dict = GenerateJsonValueData(ctrl_json_data, dataLoader.ctrl_debug_msg['t'], json_value_list)
    json_value_xys_dict = GenerateJsonValueData(ctrl_json_data, t_debug, json_value_list)

    # fig2: control status
    fig2 = FigureLayerHover(bkp.figure(x_axis_label='time',
                                  y_axis_label='status',
                                  x_range = [0.0, t_debug[-1]],
                                  # x_range = [0.0, max_time],
                                  width=600,
                                  height=250,
                                  # match_aspect = True,
                                  # aspect_scale=1
                                  ))
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
                                  width=600,
                                  height=250,
                                  # match_aspect = True,
                                  # aspect_scale=1
                                  ))
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
                                  width=600,
                                  height=250,
                                  # match_aspect = True,
                                  # aspect_scale=1
                                  ))
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "path_length_plan"), "path_length_plan")
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "remain_s_plan"), "remain_s_plan")
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "remain_s_prebreak"), "remain_s_prebreak")
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "remain_s_uss"), "remain_s_uss")
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "remain_s_ctrl"), "remain_s_ctrl", last_line = True)

    # fig5: vel status
    # "vel_out", "vel_KP_term", "vel_KI_term", "throttle_brake"
    fig5 = FigureLayerHover(bkp.figure(x_axis_label='time',
                                  y_axis_label='acc',
                                  x_range = fig2.fig.x_range,
                                  width=600,
                                  height=250,
                                  # match_aspect = True,
                                  # aspect_scale=1
                                  ))
    fig5.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_out"), "acc_cmd")
    fig5.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_KP_term"), "vel_kp_term")
    fig5.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_KI_term"), "vel_ki_term")
    fig5.AddCurv(layer_manager,
                 ScalarSomeGeneratorFromJson(json_value_xys_dict, "throttle_brake"), "throttle_brake")
    fig5.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "acc_vel"), "acc_vel", last_line = True)

    # fig6: vel status
    # "steer_angle_cmd", "steer_angle", "driver_hand_torque"
    fig6 = FigureLayerHover(bkp.figure(x_axis_label='time',
                                  y_axis_label='steer_angle',
                                  x_range = fig2.fig.x_range,
                                  width=600,
                                  height=250,
                                  # match_aspect = True,
                                  # aspect_scale=1
                                  ))
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
                                  width=600,
                                  height=250,
                                  # match_aspect = True,
                                  # aspect_scale=1
                                  ))
    fig7.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "lat_err", 100), "lat_err")
    fig7.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "phi_err", 57.3), "phi_err", last_line = True)

    return fig2, fig3, fig4, fig5, fig6, fig7
