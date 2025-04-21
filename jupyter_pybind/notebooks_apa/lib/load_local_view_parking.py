from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *

sys.path.append('../../python_proto')
from planning_debug_info_pb2 import *
from control_debug_info_pb2 import *

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
import rosbag
from ifly_parking_map_pb2 import *

plan_debug_ts = []
plan_debug_timestamps = []
fusion_object_timestamps = []
fus_occupancy_objects_timestamps = []
fusion_road_timestamps = []
ground_line_timestamps = []
localization_timestamps = []
prediction_timestamps = []
vehicle_service_timestamps = []
control_output_timestamps = []
slot_timestamps = []
fusion_slot_timestamps = []
vision_slot_timestamps = []
mobileye_lane_lines_timestamps = []
mobileye_objects_timestamps = []
fus_release_solts_id = []

car_circle_x, car_circle_y, car_circle_r = load_car_circle_coord_by_veh()
coord_tf = coord_transformer()
max_slot_num = 20
corner_points_size = 4
NUM_OF_OUTLINE_DATAORI = 4
NUM_OF_APA_SLOT_OBJ = 800
replan_flag = False
correct_path_for_limiter = False
replan_time_list = []
correct_path_for_limiter_time_list = []
enter_parking_time = 0.0
load_uss_wave_from_uss_percept_msg = False
read_uss_per_msg = load_uss_wave_from_uss_percept_msg
load_fusion_object_from_occupancy = True
version_245 = True
# OD obstacle
read_fus_obj_msg = False
corner_points_size = 4
NUM_OF_OUTLINE_DATAORI = 2
smallest_abs_t = 0.0
ego_init_x = 0.0
ego_init_y = 0.0

def rectangle_corners(cx, cy, length, width, heading, is_rad=False):
  # 将航向转换为弧度
  if not is_rad:
    radians = math.radians(heading)
  else:
    radians = heading

  unit_t = np.array([np.cos(radians), np.sin(radians)])
  unit_n = np.array([-unit_t[1], unit_t[0]]) # 逆时针旋转

  half_length = length / 2
  half_width = width / 2

  top_left = np.array([cx, cy]) + half_length * unit_t + half_width * unit_n
  top_right = np.array([cx, cy]) + half_length * unit_t - half_width * unit_n
  bottom_right = np.array([cx, cy]) - half_length * unit_t - half_width * unit_n
  bottom_left = np.array([cx, cy]) - half_length * unit_t + half_width * unit_n

  return [top_left[0], top_right[0], bottom_right[0], bottom_left[0], top_left[0]], [top_left[1], top_right[1], bottom_right[1], bottom_left[1], top_left[1]]

class LoadCyberbag:
  def __init__(self, path, parking_flag = False) -> None:
    self.bag_path = path
    start_time = time.time()
    self.bag = rosbag.Bag(path,'r',rosbag.Compression.BZ2, 768 * 1024,True, None, True)
    end_time = time.time()
    print("load bag time: ", end_time - start_time, " s")
    # loclization msg
    self.loc_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

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

    # fusion parking msg
    self.fus_parking_msg = {'abs_t':[], 't':[], 'data':[], 'json':[], 'enable':[]}

    # fusion ground line msg
    self.fus_ground_line_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # fusion object msg
    self.fus_objects_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # fusion occupancy object msg
    self.fus_occupancy_objects_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

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

    # time offset
    t0 = 0

  def load_all_data(self):
    max_time = 0.0
    global smallest_abs_t

    # load new localization msg
    try:
      loc_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/localization/egomotion"):
        loc_msg_dict[msg.msg_header.stamp / 1e6] = msg

      loc_msg_dict = {key: val for key, val in sorted(loc_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in loc_msg_dict.items():
        if t > 1e-3:
          self.loc_msg['t'].append(t)
          self.loc_msg['abs_t'].append(t)
          self.loc_msg['data'].append(msg)
      print("local init t:", self.loc_msg['t'][0])
      t0 = self.loc_msg['t'][0]
      smallest_abs_t = max(smallest_abs_t, self.loc_msg['t'][0])
      self.loc_msg['t'] = [tmp - t0  for tmp in self.loc_msg['t']]
      max_time = max(max_time, self.loc_msg['t'][-1])
      print('loc_msg time:',self.loc_msg['t'][-1])
      global ego_init_x
      global ego_init_y
      # ego pos in local and global coordinates
      ego_init_x = self.loc_msg['data'][0].position.position_boot.x
      ego_init_y =self.loc_msg['data'][0].position.position_boot.y

      if len(self.loc_msg['t']) > 0:
        self.loc_msg['enable'] = True
      else:
        self.loc_msg['enable'] = False
    except:
      self.loc_msg['enable'] = False
      print('missing /iflytek/localization/egomotion !!!')

    self.max_time = max_time

    # load vehicle service msg
    try:
      vs_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/vehicle_service"):
        vs_msg_dict[msg.msg_header.stamp / 1e6] = msg
      vs_msg_dict = {key: val for key, val in sorted(vs_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in vs_msg_dict.items():
        if t > 1e-3:
          self.vs_msg['t'].append(t)
          self.vs_msg['abs_t'].append(t)
          self.vs_msg['data'].append(msg)
      # print("self.vs_msg['t'][0]:", self.vs_msg['t'][0])
      # smallest_abs_t = min(smallest_abs_t, self.vs_msg['t'][0])
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
        plan_msg_dict[msg.msg_header.stamp / 1e6] = msg
      plan_msg_dict = {key: val for key, val in sorted(plan_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in plan_msg_dict.items():
        if t > 1e-3:
          self.plan_msg['t'].append(t)
          self.plan_msg['abs_t'].append(t)
          self.plan_msg['data'].append(msg)
      print("plan init t:", self.plan_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.plan_msg['t'][0])
      t0_plan = self.plan_msg['t'][0]
      self.plan_msg['t'] = [tmp - t0_plan  for tmp in self.plan_msg['t']]
      max_time = max(max_time, self.plan_msg['t'][-1])
      print('plan_msg time:',self.plan_msg['t'][-1])
      print('plan version:', self.plan_msg['data'][0].msg_meta.version)
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
                         "terminal_error_x", "terminal_error_y", "terminal_error_y_front", "terminal_error_heading", "car_real_time_col_lat_buffer",
                         "is_replan", "is_finished", "is_replan_first", "is_replan_by_uss", "current_path_length", "gear_change_count", "replan_reason", "plan_fail_reason",
                         "path_plan_success", "planning_status", "spline_success", "remain_dist", "remain_dist_col_det", "remain_dist_uss", "stuck_time", "replan_consume_time", "total_plan_consume_time",
                         "car_static_timer_by_pos_strict", "car_static_timer_by_pos_normal", "car_static_timer_by_vel_strict", "car_static_timer_by_vel_normal", "static_flag", "ego_heading_slot",
                         "selected_slot_id", "slot_length", "slot_width", "slot_origin_pos_x", "slot_origin_pos_y", "slot_origin_heading",
                         "slot_occupied_ratio", "pathplan_result", "target_ego_pos_slot", "path_start_seg_index", "path_end_seg_index", "path_length",
                         "uss_available", "uss_remain_dist", "uss_index", "uss_car_index",
                         "optimization_terminal_pose_error", "optimization_terminal_heading_error", "lat_path_opt_cost_time_ms",
                         "ref_gear", "ref_arc_steer",
                         "correct_path_for_limiter", "replan_flag", "path_plan_time_ms",
                         "para_tlane_is_front_vacant", "para_tlane_is_rear_vacant", "para_tlane_side_sgn",
                         "para_tlane_front_min_x_before_clamp", "para_tlane_front_min_x_after_clamp", "para_tlane_front_y",
                         "para_tlane_rear_max_x_before_clamp", "para_tlane_rear_max_x_after_clamp", "para_tlane_rear_y",
                         "slot_replan_jump_dist", "slot_replan_jump_heading", "is_path_lateral_optimized",
                         "current_gear_length", "current_gear_pt_size", "sample_ds", "move_slot_dist", "replan_move_slot_dist", "replan_count", "geometry_path_release", "pre_plan_case",
                         "statemachine_timestamp", "fusion_slot_timestamp", "localiztion_timestamp", "uss_wave_timestamp", "uss_per_timestamp", "ground_line_timestamp", "fusion_objects_timestamp", "fusion_occupancy_objects_timestamp", "control_output_timestamp"]

      json_vector_list = ["raw_refline_x_vec", "raw_refline_y_vec", "assembled_delta", "assembled_omega", "traj_x_vec", "traj_y_vec",
                          "slm_selected_obs_x", "slm_selected_obs_y", "obstaclesX", "obstaclesY", "slot_corner_X", "slot_corner_Y", "plan_traj_x", "plan_traj_y", "plan_traj_heading", "plan_traj_lat_buffer",
                          "limiter_corner_X", "limiter_corner_Y", "obstacles_x_slot", "obstacles_y_slot", "col_det_path_x", "col_det_path_y", "col_det_path_phi", "car_predict_x_vec", "car_predict_y_vec", "car_predict_heading_vec",
                          "tlane_front_que_x","tlane_front_que_y", "tlane_rear_que_x", "tlane_rear_que_y",
                          "para_tlane_obs_pt_before_uss"]

      plan_debug_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/planning/debug_info"):
        planning_debug_output = PlanningDebugInfo()
        planning_debug_output.ParseFromString(msg.debug_info)
        plan_debug_msg_dict[planning_debug_output.timestamp / 1e6] = planning_debug_output
      plan_debug_msg_dict = {key: val for key, val in sorted(plan_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in plan_debug_msg_dict.items():
        if t > 1e-3:
          self.plan_debug_msg['t'].append(t)
          self.plan_debug_msg['abs_t'].append(t)
          self.plan_debug_msg['data'].append(msg)
        try:
          json_struct = json.loads(msg.data_json, strict = False)
          json_data = {}
          LoadScalarList(json_data, json_value_list, json_struct)
          LoadVectorList(json_data, json_vector_list, json_struct)

          self.plan_debug_msg['json'].append(json_data)

          if json_data['replan_flag'] == 1:
            replan_time_list.append(t)
          if json_data['correct_path_for_limiter'] == 1:
            correct_path_for_limiter_time_list.append(t)

        except json.decoder.JSONDecodeError as jserr:
          print('except',jserr)

      for i in range(len(replan_time_list)):
        replan_time_list[i] = replan_time_list[i] - self.plan_debug_msg['t'][0]
        replan_time_list[i] = round(replan_time_list[i], 2)
      for i in range(len(correct_path_for_limiter_time_list)):
        correct_path_for_limiter_time_list[i] = correct_path_for_limiter_time_list[i] - self.plan_debug_msg['t'][0]
        correct_path_for_limiter_time_list[i] = round(correct_path_for_limiter_time_list[i], 2)
      # print("self.plan_debug_msg['t'][0]:", self.plan_debug_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.plan_debug_msg['t'][0])
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
        ctrl_msg_dict[msg.msg_header.stamp / 1e6] = msg
      ctrl_msg_dict = {key: val for key, val in sorted(ctrl_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ctrl_msg_dict.items():
        if t > 1e-3:
          self.ctrl_msg['t'].append(t)
          self.ctrl_msg['abs_t'].append(t)
          self.ctrl_msg['data'].append(msg)
      # print("self.ctrl_msg['t'][0]:", self.ctrl_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.ctrl_msg['t'][0])
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
        ctrl_debug_output = ControlDebugInfo()
        ctrl_debug_output.ParseFromString(msg.debug_info)
        ctrl_debug_msg_dict[ctrl_debug_output.timestamp / 1e6] = ctrl_debug_output
      ctrl_debug_msg_dict = {key: val for key, val in sorted(ctrl_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ctrl_debug_msg_dict.items():
        if t > 1e-3:
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
      # print("self.ctrl_debug_msg['t'][0]:", self.ctrl_debug_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.ctrl_debug_msg['t'][0])
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

    # load fusion parking msg
    try:
      fus_parking_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/parking_slot"):
        fus_parking_msg_dict[msg.msg_header.stamp / 1e6] = msg
      fus_parking_msg_dict = {key: val for key, val in sorted(fus_parking_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in fus_parking_msg_dict.items():
        if t > 1e-3:
          self.fus_parking_msg['t'].append(t)
          self.fus_parking_msg['abs_t'].append(t)
          self.fus_parking_msg['data'].append(msg)
      # print("self.fus_parking_msg['t'][0]:", self.fus_parking_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.fus_parking_msg['t'][0])
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

    # load fusion ground line msg
    try:
      fusion_ground_line_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/ground_line"):
        fusion_ground_line_msg_dict[msg.msg_header.stamp / 1e6] = msg

      fusion_ground_line_msg_dict = {key: val for key, val in sorted(fusion_ground_line_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in fusion_ground_line_msg_dict.items():
        if t > 1e-3:
          self.fus_ground_line_msg['t'].append(t)
          self.fus_ground_line_msg['abs_t'].append(t)
          self.fus_ground_line_msg['data'].append(msg)
      # print("self.fus_ground_line_msg['t'][0]:", self.fus_ground_line_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.fus_ground_line_msg['t'][0])
      if (abs(self.fus_ground_line_msg['t'][0]) < 0.0001):
        self.fus_ground_line_msg['t'] = [tmp - self.fus_ground_line_msg['t'][1]  for tmp in self.fus_ground_line_msg['t']]
      else:
        self.fus_ground_line_msg['t'] = [tmp - self.fus_ground_line_msg['t'][0]  for tmp in self.fus_ground_line_msg['t']]

      print('fus_ground_line_msg time:',self.fus_ground_line_msg['t'][-1])

      if len(self.fus_ground_line_msg['t']) > 0:
        self.fus_ground_line_msg['enable'] = True
      else:
        self.fus_ground_line_msg['enable'] = False
    except:
      self.fus_ground_line_msg['enable'] = False
      print('missing /iflytek/fusion/ground_line !!!')

    if read_fus_obj_msg:
      # load fusion objects msg
      try:
        fus_objects_msg_dict = {}
        for topic, msg, t in self.bag.read_messages("/iflytek/fusion/objects"):
            fus_objects_msg_dict[msg.msg_header.stamp / 1e6] = msg

        fus_objects_msg_dict = {key: val for key, val in sorted(fus_objects_msg_dict.items(), key = lambda ele: ele[0])}
        for t, msg in fus_objects_msg_dict.items():
          if t > 1e-3:
            self.fus_objects_msg['t'].append(t)
            self.fus_objects_msg['abs_t'].append(t)
            self.fus_objects_msg['data'].append(msg)
        # print("self.fus_objects_msg['t'][0]:", self.fus_objects_msg['t'][0])

        smallest_abs_t = min(smallest_abs_t, self.fus_objects_msg['t'][0])
        self.fus_objects_msg['t'] = [tmp - t0  for tmp in self.fus_objects_msg['t']]
        print('fus_objects_msg time:',self.fus_objects_msg['t'][-1])
        if len(self.fus_objects_msg['t']) > 0:
          self.fus_objects_msg['enable'] = True
        else:
          self.fus_objects_msg['enable'] = False
      except:
        self.fus_objects_msg['enable'] = False
        print('missing /iflytek/fusion/objects !!!')
    else:
      print('no read /iflytek/fusion/objects !!!')


    # load fusion occ objects msg
    try:
      fus_occupancy_objects_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/occupancy/objects"):
        fus_occupancy_objects_msg_dict[msg.msg_header.stamp / 1e6] = msg

      fus_occupancy_objects_msg_dict = {key: val for key, val in sorted(fus_occupancy_objects_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in fus_occupancy_objects_msg_dict.items():
        if t > 1e-3:
          self.fus_occupancy_objects_msg['t'].append(t)
          self.fus_occupancy_objects_msg['abs_t'].append(t)
          self.fus_occupancy_objects_msg['data'].append(msg)

      # print("self.fus_occupancy_objects_msg['t'][0]:", self.fus_occupancy_objects_msg['t'][0])
      # print("self.fus_occupancy_objects_msg['t'][-1]:", self.fus_occupancy_objects_msg['t'][-1])
      # smallest_abs_t = min(smallest_abs_t, self.fus_occupancy_objects_msg['t'][0])
      self.fus_occupancy_objects_msg['t'] = [tmp - t0  for tmp in self.fus_occupancy_objects_msg['t']]
      print('fus_occupancy_objects_msg time:',self.fus_occupancy_objects_msg['t'][-1])
      if len(self.fus_occupancy_objects_msg['t']) > 0:
        self.fus_occupancy_objects_msg['enable'] = True
      else:
        self.fus_occupancy_objects_msg['enable'] = False
    except:
      self.fus_occupancy_objects_msg['enable'] = False
      print('missing /iflytek/fusion/occupancy/objects !!!')

    # load visual parking msg
    try:
      vis_parking_msg_dict = {}
      # origin visual parking slot proto
      # for topic, msg, t in self.bag.read_messages("/parking_slot"):
      # new visula parking slot proto
      for topic, msg, t in self.bag.read_messages("/iflytek/camera_perception/parking_slot_list"):
        vis_parking_msg_dict[msg.msg_header.stamp / 1e6] = msg
      vis_parking_msg_dict = {key: val for key, val in sorted(vis_parking_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in vis_parking_msg_dict.items():
        if t > 1e-3:
          self.vis_parking_msg['t'].append(t)
          self.vis_parking_msg['abs_t'].append(t)
          self.vis_parking_msg['data'].append(msg)
      # print("self.vis_parking_msg['t'][0]:", self.vis_parking_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.vis_parking_msg['t'][0])
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
        print('missing /iflytek/camera_perception/parking_slot_list !!!')
    except:
      self.vis_parking_msg['enable'] = False
      print('missing /iflytek/camera_perception/parking_slot_list !!!')


    # load state machine msg
    try:
      soc_state_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fsm/soc_state"):
        soc_state_msg_dict[msg.msg_header.stamp / 1e6] = msg
      soc_state_msg_dict = {key: val for key, val in sorted(soc_state_msg_dict.items(), key = lambda ele: ele[0])}
      global enter_parking_time
      enter_parking_time = 0.0
      first_enter_apa = False
      for t, msg in soc_state_msg_dict.items():
        if t > 1e-3:
          self.soc_state_msg['t'].append(t)
          self.soc_state_msg['abs_t'].append(t)
          self.soc_state_msg['data'].append(msg)
        # record the start time of fusi_parking_msg
        if first_enter_apa == False and msg.current_state >= 23:
          enter_parking_time = t - self.soc_state_msg['t'][0]
          first_enter_apa = True
      # print("self.soc_state_msg['t'][0]:", self.soc_state_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.soc_state_msg['t'][0])
      self.soc_state_msg['t'] = [tmp - self.soc_state_msg['t'][0]  for tmp in self.soc_state_msg['t']]
      max_time = max(max_time, self.soc_state_msg['t'][-1])
      print('soc_state_msg time:',self.soc_state_msg['t'][-1])
      print("enter_parking_time", enter_parking_time)
      if len(self.soc_state_msg['t']) > 0:
        self.soc_state_msg['enable'] = True
      else:
        self.soc_state_msg['enable'] = False
    except:
      self.soc_state_msg['enable'] = False
      print('missing /iflytek/fsm/soc_state !!!')

    # self.max_time = max_time

    # add obstacles in plot apa
    # load precept_info msg
    # try:
    #   precept_msg_dict = {}
    #   for topic, msg, t in self.bag.read_messages("/iflytek/ultrasonic_perception_info"):
    #     precept_msg_dict[msg.msg_header.stamp / 1e6] = msg
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
    #     precept_debug_msg_dict[msg.msg_header.stamp / 1e6] = msg
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
      for topic, msg, t in self.bag.read_messages("/iflytek/uss/usswave_info"):
        wave_msg_dict[msg.msg_header.stamp / 1e6] = msg
      wave_msg_dict = {key: val for key, val in sorted(wave_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in wave_msg_dict.items():
        if t > 1e-3:
          self.wave_msg['t'].append(t)
          self.wave_msg['abs_t'].append(t)
          self.wave_msg['data'].append(msg)
      # print("self.wave_msg['t'][0]:", self.wave_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.wave_msg['t'][0])
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
      print("missing /iflytek/uss/usswave_info !!!")
   #load adas_debug_msg
    try:
      adas_debug_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/adas_function_debug"):
        adas_debug_msg_dict[msg.msg_header.stamp / 1e6] = msg
      adas_debug_msg_dict = {key: val for key, val in sorted(adas_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in adas_debug_msg_dict.items():
        if t > 1e-3:
          self.adas_debug_msg['t'].append(t)
          self.adas_debug_msg['abs_t'].append(t)
          self.adas_debug_msg['data'].append(msg)
      # print("self.adas_debug_msg['t'][0]:", self.adas_debug_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.adas_debug_msg['t'][0])
      self.adas_debug_msg['t'] = [tmp - t0  for tmp in self.adas_debug_msg['t']]
      self.adas_debug_msg['enable'] = True
      print('adas_debug time:',self.adas_debug_msg['t'][-1])
      # max_time = max(max_time, self.adas_debug_msg['t'][-1])
      if len(self.adas_debug_msg['t']) > 0:
        self.adas_debug_msg['enable'] = True
      else:
        self.adas_debug_msg['enable'] = False
    except:
      self.adas_debug_msg['enable'] = False
      print("missing /iflytek/adas_function_debug !!!")

    # load wave_debug_info msg
    try:
      wave_debug_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/uss/ussdriver_debug_info"):
        wave_debug_msg_dict[msg.msg_header.stamp / 1e6] = msg
      wave_debug_msg_dict = {key: val for key, val in sorted(wave_debug_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in wave_debug_msg_dict.items():
        if t > 1e-3:
          self.wave_debug_msg['t'].append(t)
          self.wave_debug_msg['abs_t'].append(t)
          self.wave_debug_msg['data'].append(msg)
      # print("self.wave_debug_msg['t'][0]:", self.wave_debug_msg['t'][0])

      # smallest_abs_t = min(smallest_abs_t, self.wave_debug_msg['t'][0])
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
    if read_uss_per_msg == True:
      try:
        uss_percept_msg_dict = {}
        for topic, msg, t in self.bag.read_messages("/iflytek/fusion/uss_perception_info"):
          uss_percept_msg_dict[msg.msg_header.stamp / 1e6] = msg
        uss_percept_msg_dict = {key: val for key, val in sorted(uss_percept_msg_dict.items(), key = lambda ele: ele[0])}
        for t, msg in uss_percept_msg_dict.items():
          if t > 0.0:
            self.uss_percept_msg['t'].append(t)
            self.uss_percept_msg['abs_t'].append(t)
            self.uss_percept_msg['data'].append(msg)
        t0 = self.uss_percept_msg['t'][0]
        # print("self.uss_percept_msg['t'][0]:", self.uss_percept_msg['t'][0])

        # smallest_abs_t = min(smallest_abs_t, self.uss_percept_msg['t'][0])
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
        print("missing /iflytek/fusion/uss_perception_info !!!")
    else:
      self.uss_percept_msg['enable'] = False
      print("no read /iflytek/fusion/uss_perception_info !!!")

    print("smallest_abs_t = ", smallest_abs_t)
    time_array = time.localtime(smallest_abs_t)
    time_string = time.strftime("%Y-%m-%d %H:%M:%S", time_array)
    print("time = ", time_string)
    return max_time

  def get_msg_index(self, bag_time):
    ### step 1: 时间戳对齐
    abs_t = bag_time + smallest_abs_t
    print("smallest_abs_t:", smallest_abs_t)
    out = {}
    loc_msg_idx = 0
    if self.loc_msg['enable'] == True:
      while self.loc_msg['abs_t'][loc_msg_idx] <= abs_t and loc_msg_idx < (len(self.loc_msg['abs_t'])-1):
          loc_msg_idx = loc_msg_idx + 1
    # print("loc_msg_idx:", loc_msg_idx)
    out['loc_msg_idx'] = loc_msg_idx

    fus_parking_msg_idx = 0
    if self.fus_parking_msg['enable'] == True:
      while self.fus_parking_msg['abs_t'][fus_parking_msg_idx] <= abs_t and fus_parking_msg_idx < (len(self.fus_parking_msg['abs_t'])-1):
          fus_parking_msg_idx = fus_parking_msg_idx + 1
    # print("fus_parking_msg_idx:", fus_parking_msg_idx)
    out['fus_parking_msg_idx'] = fus_parking_msg_idx

    fus_ground_line_msg_idx = 0
    if self.fus_ground_line_msg['enable'] == True:
      while self.fus_ground_line_msg['abs_t'][fus_ground_line_msg_idx] <= abs_t and fus_ground_line_msg_idx < (len(self.fus_ground_line_msg['abs_t'])-1):
          fus_ground_line_msg_idx = fus_ground_line_msg_idx + 1
    out['fus_ground_line_msg_idx'] = fus_ground_line_msg_idx

    fus_objects_msg_idx = 0
    if self.fus_objects_msg['enable'] == True:
      while self.fus_objects_msg['abs_t'][fus_objects_msg_idx] <= abs_t and fus_objects_msg_idx < (len(self.fus_objects_msg['abs_t'])-1):
          fus_objects_msg_idx = fus_objects_msg_idx + 1
    out['fus_objects_msg_idx'] = fus_objects_msg_idx

    fus_occupancy_objects_msg_idx = 0
    if self.fus_occupancy_objects_msg['enable'] == True:
      while self.fus_occupancy_objects_msg['abs_t'][fus_occupancy_objects_msg_idx] <= abs_t and fus_occupancy_objects_msg_idx < (len(self.fus_occupancy_objects_msg['abs_t'])-1):
        fus_occupancy_objects_msg_idx = fus_occupancy_objects_msg_idx + 1
    out['fus_occupancy_objects_msg_idx'] = fus_occupancy_objects_msg_idx

    vis_parking_msg_idx = 0
    if self.vis_parking_msg['enable'] == True:
      while self.vis_parking_msg['abs_t'][vis_parking_msg_idx] <= abs_t and vis_parking_msg_idx < (len(self.vis_parking_msg['abs_t'])-1):
        vis_parking_msg_idx = vis_parking_msg_idx + 1
    out['vis_parking_msg_idx'] = vis_parking_msg_idx

    vs_msg_idx = 0
    if self.vs_msg['enable'] == True:
      while self.vs_msg['abs_t'][vs_msg_idx] <= abs_t and vs_msg_idx < (len(self.vs_msg['abs_t'])-1):
          vs_msg_idx = vs_msg_idx + 1
    out['vs_msg_idx'] = vs_msg_idx

    plan_msg_idx = 0
    if self.plan_msg['enable'] == True:
      while self.plan_msg['abs_t'][plan_msg_idx] <= abs_t and plan_msg_idx < (len(self.plan_msg['abs_t'])-1):
          plan_msg_idx = plan_msg_idx + 1
    # print("plan_msg_idx:", plan_msg_idx)
    out['plan_msg_idx'] = plan_msg_idx

    plan_debug_msg_idx = 0
    if self.plan_debug_msg['enable'] == True:
      while self.plan_debug_msg['abs_t'][plan_debug_msg_idx] <= abs_t and plan_debug_msg_idx < (len(self.plan_debug_msg['abs_t'])-1):
          plan_debug_msg_idx = plan_debug_msg_idx + 1
    # print("plan_debug_msg_idx:", plan_debug_msg_idx)
    out['plan_debug_msg_idx'] = plan_debug_msg_idx

    ctrl_msg_idx = 0
    if self.ctrl_msg['enable'] == True:
      while self.ctrl_msg['abs_t'][ctrl_msg_idx] <= abs_t and ctrl_msg_idx < (len(self.ctrl_msg['abs_t'])-1):
          ctrl_msg_idx = ctrl_msg_idx + 1
    out['ctrl_msg_idx'] = ctrl_msg_idx

    ctrl_debug_msg_idx = 0
    if self.ctrl_debug_msg['enable'] == True:
      while self.ctrl_debug_msg['abs_t'][ctrl_debug_msg_idx] <= abs_t and ctrl_debug_msg_idx < (len(self.ctrl_debug_msg['abs_t'])-1):
          ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
    out['ctrl_debug_msg_idx'] = ctrl_debug_msg_idx

    soc_state_msg_idx = 0
    if self.soc_state_msg['enable'] == True:
      while self.soc_state_msg['abs_t'][soc_state_msg_idx] <= abs_t and soc_state_msg_idx < (len(self.soc_state_msg['abs_t'])-1):
          soc_state_msg_idx = soc_state_msg_idx + 1
    out['soc_state_msg_idx'] = soc_state_msg_idx

    wave_msg_idx = 0
    if self.wave_msg['enable'] == True:
      while self.wave_msg['abs_t'][wave_msg_idx] <= abs_t and wave_msg_idx < (len(self.wave_msg['abs_t'])-1):
          wave_msg_idx = wave_msg_idx + 1
    out['wave_msg_idx'] = wave_msg_idx

    adas_msg_idx = 0
    if self.adas_debug_msg['enable'] == True:
      while self.adas_debug_msg['abs_t'][adas_msg_idx] <= abs_t and adas_msg_idx < (len(self.adas_debug_msg['abs_t'])-1):
          adas_msg_idx = adas_msg_idx + 1
    out['adas_msg_idx'] = adas_msg_idx

    uss_percept_msg_idx = 0
    if self.uss_percept_msg['enable'] == True:
      while self.uss_percept_msg['abs_t'][uss_percept_msg_idx] <= abs_t and uss_percept_msg_idx < (len(self.uss_percept_msg['abs_t'])-1):
          uss_percept_msg_idx = uss_percept_msg_idx + 1
    out['uss_percept_msg_idx'] = uss_percept_msg_idx

    return out

  def get_localization_msg_index(self, bag_time):
    loc_msg_idx = 0
    if self.loc_msg['enable'] == True:
      while self.loc_msg['t'][loc_msg_idx] <= bag_time and loc_msg_idx < (len(self.loc_msg['t'])-1):
          loc_msg_idx = loc_msg_idx + 1

    return loc_msg_idx


def update_local_view_data_parking(fig1, bag_loader, bag_time, vehicle_type, car_inflation, local_view_data, plot_ctrl_flag=False):

  car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type, car_inflation)
  car_circle_x, car_circle_y, car_circle_r = load_car_circle_coord_by_veh(vehicle_type, car_inflation)
  abs_t = bag_time + smallest_abs_t

  ### step 1: timestamp alignment
  loc_msg_idx = 0
  if bag_loader.loc_msg['enable'] == True:
    while bag_loader.loc_msg['abs_t'][loc_msg_idx] <= abs_t and loc_msg_idx < (len(bag_loader.loc_msg['abs_t'])-1):
        loc_msg_idx = loc_msg_idx + 1
  local_view_data['data_index']['loc_msg_idx'] = loc_msg_idx

  fus_parking_msg_idx = 0
  if bag_loader.fus_parking_msg['enable'] == True:
    while bag_loader.fus_parking_msg['abs_t'][fus_parking_msg_idx] <= abs_t and fus_parking_msg_idx < (len(bag_loader.fus_parking_msg['abs_t'])-1):
        fus_parking_msg_idx = fus_parking_msg_idx + 1
  local_view_data['data_index']['fus_parking_msg_idx'] = fus_parking_msg_idx

  fus_ground_line_msg_idx = 0
  if bag_loader.fus_ground_line_msg['enable'] == True:
    while bag_loader.fus_ground_line_msg['abs_t'][fus_ground_line_msg_idx] <= abs_t and fus_ground_line_msg_idx < (len(bag_loader.fus_ground_line_msg['abs_t'])-1):
        fus_ground_line_msg_idx = fus_ground_line_msg_idx + 1
  local_view_data['fus_ground_line_msg_idx'] = fus_ground_line_msg_idx

  fus_objects_msg_idx = 0
  if bag_loader.fus_objects_msg['enable'] == True:
    while bag_loader.fus_objects_msg['abs_t'][fus_objects_msg_idx] <= abs_t and fus_objects_msg_idx < (len(bag_loader.fus_objects_msg['abs_t'])-1):
        fus_objects_msg_idx = fus_objects_msg_idx + 1
  local_view_data['data_index']['fus_objects_msg_idx'] = fus_objects_msg_idx

  fus_occupancy_objects_msg_idx = 0
  if bag_loader.fus_occupancy_objects_msg['enable'] == True:
    while bag_loader.fus_occupancy_objects_msg['abs_t'][fus_occupancy_objects_msg_idx] <= abs_t and fus_occupancy_objects_msg_idx < (len(bag_loader.fus_occupancy_objects_msg['abs_t'])-1):
      fus_occupancy_objects_msg_idx = fus_occupancy_objects_msg_idx + 1
  local_view_data['data_index']['fus_occupancy_objects_msg_idx'] = fus_occupancy_objects_msg_idx

  vis_parking_msg_idx = 0
  if bag_loader.vis_parking_msg['enable'] == True:
    while bag_loader.vis_parking_msg['abs_t'][vis_parking_msg_idx] <= abs_t and vis_parking_msg_idx < (len(bag_loader.vis_parking_msg['abs_t'])-1):
        vis_parking_msg_idx = vis_parking_msg_idx + 1
  local_view_data['data_index']['vis_parking_msg_idx'] = vis_parking_msg_idx

  vs_msg_idx = 0
  if bag_loader.vs_msg['enable'] == True:
    while bag_loader.vs_msg['abs_t'][vs_msg_idx] <= abs_t and vs_msg_idx < (len(bag_loader.vs_msg['abs_t'])-1):
        vs_msg_idx = vs_msg_idx + 1
  local_view_data['data_index']['vs_msg_idx'] = vs_msg_idx

  plan_msg_idx = 0
  if bag_loader.plan_msg['enable'] == True:
    while bag_loader.plan_msg['abs_t'][plan_msg_idx] <= abs_t and plan_msg_idx < (len(bag_loader.plan_msg['abs_t'])-1):
        plan_msg_idx = plan_msg_idx + 1
  local_view_data['data_index']['plan_msg_idx'] = plan_msg_idx

  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['abs_t'][plan_debug_msg_idx] <= abs_t and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['abs_t'])-1):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
  local_view_data['data_index']['plan_debug_msg_idx'] = plan_debug_msg_idx

  ctrl_msg_idx = 0
  if bag_loader.ctrl_msg['enable'] == True:
    while bag_loader.ctrl_msg['abs_t'][ctrl_msg_idx] <= abs_t and ctrl_msg_idx < (len(bag_loader.ctrl_msg['abs_t'])-1):
        ctrl_msg_idx = ctrl_msg_idx + 1
  local_view_data['data_index']['ctrl_msg_idx'] = ctrl_msg_idx

  ctrl_debug_msg_idx = 0
  if bag_loader.ctrl_debug_msg['enable'] == True:
    while bag_loader.ctrl_debug_msg['abs_t'][ctrl_debug_msg_idx] <= abs_t and ctrl_debug_msg_idx < (len(bag_loader.ctrl_debug_msg['abs_t'])-1):
        ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
  local_view_data['data_index']['ctrl_debug_msg_idx'] = ctrl_debug_msg_idx

  soc_state_msg_idx = 0
  if bag_loader.soc_state_msg['enable'] == True:
    while bag_loader.soc_state_msg['abs_t'][soc_state_msg_idx] <= abs_t and soc_state_msg_idx < (len(bag_loader.soc_state_msg['abs_t'])-1):
        soc_state_msg_idx = soc_state_msg_idx + 1
  local_view_data['data_index']['soc_state_msg_idx'] = soc_state_msg_idx

  wave_msg_idx = 0
  if bag_loader.wave_msg['enable'] == True:
    while bag_loader.wave_msg['abs_t'][wave_msg_idx] <= abs_t and wave_msg_idx < (len(bag_loader.wave_msg['abs_t'])-1):
        wave_msg_idx = wave_msg_idx + 1
  local_view_data['data_index']['wave_msg_idx'] = wave_msg_idx

  uss_percept_msg_idx = 0
  if bag_loader.uss_percept_msg['enable'] == True:
    while bag_loader.uss_percept_msg['abs_t'][uss_percept_msg_idx] <= abs_t and uss_percept_msg_idx < (len(bag_loader.uss_percept_msg['abs_t'])-1):
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
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw

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

    # front wheel pos line
    norm_vec_3 = [wheel_base * heading_vec[0], wheel_base * heading_vec[1]]
    x3 = x1 + norm_vec_3[0]
    y3 = y1 + norm_vec_3[1]
    x4 = x2 + norm_vec_3[0]
    y4 = y2 + norm_vec_3[1]

    local_view_data['data_current_front_line'].data.update({
      'y' : [y3, y4],
      'x' : [x3, x4],
    })

    coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)

    ego_xn, ego_yn = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.loc_msg['data'])):
      pos_xn_i = bag_loader.loc_msg['data'][i].position.position_boot.x
      pos_yn_i = bag_loader.loc_msg['data'][i].position.position_boot.y
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

    vel_ego = bag_loader.loc_msg['data'][loc_msg_idx].velocity.velocity_body.vx
    try:
      remain_s_ctrl = bag_loader.ctrl_debug_msg['json'][ctrl_debug_msg_idx]['remain_s_ctrl'] * 100
    except:
      remain_s_ctrl = 10.0
    if bag_loader.vs_msg['enable'] == True:
      steer_deg = bag_loader.vs_msg['data'][vs_msg_idx].steering_wheel_angle * 57.3
    else:
      steer_deg = 0.0

    if bag_loader.fus_parking_msg['enable'] == True:
      selected_slot_id = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx].select_slot_id
    else:
      selected_slot_id = -1

    current_state = bag_loader.soc_state_msg['data'][soc_state_msg_idx].current_state
    # local_view_data['data_text'].data.update({
    #   'vel_ego_text': ['v = {:.2f} m/s, steer = {:.1f} deg, state = {:d}'.format(round(vel_ego, 2), round(steer_deg, 1), current_state)],
    # })
    abs_t_string = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(abs_t))
    local_view_data['data_text'].data.update({
      'vel_ego_text': ['v = {:.2f} m/s, remain_s_ctrl = {:.1f} cm, steer = {:.1f} deg, selected_slot_id = {:d}, state = {:d}, time = {:s}, abs_t = {:} s'.format(round(vel_ego, 2), round(remain_s_ctrl, 1), round(steer_deg, 1), selected_slot_id, current_state, abs_t_string, round(abs_t, 1))],
    })

  ### step 3: loading planning traj information
  if bag_loader.plan_msg['enable'] == True:
    trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
    plan_x = []
    plan_y = []
    plan_heading = []

    for i in range(trajectory.trajectory_points_size):
      plan_x.append(trajectory.trajectory_points[i].x - cur_pos_xn0)
      plan_y.append(trajectory.trajectory_points[i].y - cur_pos_yn0)
      plan_heading.append(trajectory.trajectory_points[i].heading_yaw)

    if len(plan_x) > 0:
      print("last_x, last_y, last_heading = ", plan_x[-1], plan_y[-1], plan_heading[-1] * 57.3)

    local_view_data['data_planning'].data.update({
        'plan_traj_y' : plan_y,
        'plan_traj_x' : plan_x,
    })

    car_xn = []
    car_yn = []
    line_xn = []
    line_yn = []
    front_line_xn = []
    front_line_yn = []
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

      # front wheel pos line
      norm_vec_3 = [wheel_base * heading_vec[0], wheel_base * heading_vec[1]]
      x3 = x1 + norm_vec_3[0]
      y3 = y1 + norm_vec_3[1]
      x4 = x2 + norm_vec_3[0]
      y4 = y2 + norm_vec_3[1]
      front_line_xn = [x3, x4]
      front_line_yn = [y3, y4]

    local_view_data['data_car_target'].data.update({
      'car_xn': car_xn,
      'car_yn': car_yn,
    })

    if (len(plan_x) > 0):
      local_view_data['data_target_pos'].data.update({
        'target_pos_x': [plan_x[-1]],
        'target_pos_y': [plan_y[-1]],
      })
    else:
      local_view_data['data_target_pos'].data.update({
        'target_pos_x': [],
        'target_pos_y': [],
      })

    local_view_data['data_car_target_line'].data.update({
      'x' : line_xn,
      'y' : line_yn,
    })

    local_view_data['data_front_target_line'].data.update({
      'x' : front_line_xn,
      'y' : front_line_yn,
    })

    plan_traj_x_vec = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]["plan_traj_x"]
    plan_traj_y_vec = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]["plan_traj_y"]
    plan_traj_heading_vec = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]["plan_traj_heading"]
    plan_traj_lat_buffert_vec = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]["plan_traj_lat_buffer"]
    complete_x_vec, complete_y_vec = [], []
    if len(plan_traj_x_vec) < 21:
      for i in range(len(plan_x)):
        car_xn = []
        car_yn = []
        for j in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], plan_x[i], plan_y[i], plan_heading[i])
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)
        if (i%5==0):
          car_box_x_vec.append(car_xn)
          car_box_y_vec.append(car_yn)
    else:
      for i in range(len(plan_traj_x_vec)):
        complete_x_vec.append(plan_traj_x_vec[i])
        complete_y_vec.append(plan_traj_y_vec[i])
        car_xn = []
        car_yn = []
        car_xb_temp, car_yb_temp, wheel_base_temp = load_car_params_patch_parking(vehicle_type, plan_traj_lat_buffert_vec[i])
        for j in range(len(car_xb_temp)):
          tmp_x, tmp_y = local2global(car_xb_temp[j], car_yb_temp[j], plan_traj_x_vec[i], plan_traj_y_vec[i], plan_traj_heading_vec[i])
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)
        if (i % 5 == 0 or i == len(plan_traj_x_vec) - 1):
          car_box_x_vec.append(car_xn)
          car_box_y_vec.append(car_yn)
    local_view_data['data_car_box'].data.update({
      'x_vec': car_box_x_vec,
      'y_vec': car_box_y_vec,
    })

    local_view_data['data_complete_planning'].data.update({
      'plan_traj_x': complete_x_vec,
      'plan_traj_y': complete_y_vec,
    })

  # load control
  if bag_loader.ctrl_msg['enable'] == True:
    control_result_points = []
    for i in range(bag_loader.ctrl_msg['data'][ctrl_msg_idx].control_trajectory.control_result_points_size):
      control_result_points.append(bag_loader.ctrl_msg['data'][ctrl_msg_idx].control_trajectory.control_result_points[i])
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
    parking_fusion_slot_lists_size = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx].parking_fusion_slot_lists_size
    select_slot_id = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx].select_slot_id
    # clear data
    # local_view_data['data_target_managed_slot'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_fusion_slot1'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_fusion_slot2'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_fusion_slot3'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_fusion_parking_id'].data.update({'id':[], 'id_text_x':[], 'id_text_y':[],})

    # 1 camera; 2 uss; 3 camera and uss;
    slots_x0_vec = []
    slots_y0_vec = []

    slots_x1_vec = []
    slots_y1_vec = []

    slots_x2_vec = []
    slots_y2_vec = []

    slots_x3_vec = []
    slots_y3_vec = []

    occupied_x1_vec = []
    occupied_y1_vec = []

    occupied_x2_vec = []
    occupied_y2_vec = []

    occupied_x3_vec = []
    occupied_y3_vec = []

    id_vec = []
    fus_release_solts_id = []
    id_text_x_vec = []
    id_text_y_vec = []
    selected_slot_confidence = 0
    for j in range(parking_fusion_slot_lists_size):
      slot = parking_fusion_slot_lists[j]
      single_slot_x_vec = []
      single_slot_y_vec = []
      if slot.id == select_slot_id:
        selected_slot_confidence = slot.confidence
      # attention: fusion slots are based on odom system, visual slots are based on vehicle system
      # 1. update slots corner points
      for k in range(corner_points_size):
        corner_x_global = slot.corner_points[k].x
        corner_y_global = slot.corner_points[k].y
        single_slot_x_vec.append(corner_x_global - cur_pos_xn0)
        single_slot_y_vec.append(corner_y_global - cur_pos_yn0)
      slot_plot_x_vec = [single_slot_x_vec[0],single_slot_x_vec[2],single_slot_x_vec[3],single_slot_x_vec[1]]
      slot_plot_y_vec = [single_slot_y_vec[0],single_slot_y_vec[2],single_slot_y_vec[3],single_slot_y_vec[1]]
      if slot.fusion_source == 1:
        slots_x1_vec.append(slot_plot_x_vec)
        slots_y1_vec.append(slot_plot_y_vec)
      elif slot.fusion_source == 2:
        slots_x2_vec.append(slot_plot_x_vec)
        slots_y2_vec.append(slot_plot_y_vec)
      elif slot.fusion_source == 3:
        slots_x3_vec.append(slot_plot_x_vec)
        slots_y3_vec.append(slot_plot_y_vec)

      if slot.resource_type == 1:
        slots_x0_vec.append(slot_plot_x_vec)
        slots_y0_vec.append(slot_plot_y_vec)

      # 1.2 update slots limiter points in same slot_plot_vec
      single_limiter_x_vec = []
      single_limiter_y_vec = []
      if slot.limiters_size == 1:
        single_limiter_x_vec.append(slot.limiters[0].end_points[0].x - cur_pos_xn0)
        single_limiter_x_vec.append(slot.limiters[0].end_points[1].x - cur_pos_xn0)
        single_limiter_y_vec.append(slot.limiters[0].end_points[0].y - cur_pos_yn0)
        single_limiter_y_vec.append(slot.limiters[0].end_points[1].y - cur_pos_yn0)
      elif slot.limiters_size == 2:
        single_limiter_x_vec.append(slot.limiters[0].end_points[0].x - cur_pos_xn0)
        single_limiter_x_vec.append(slot.limiters[1].end_points[1].x - cur_pos_xn0)
        single_limiter_y_vec.append(slot.limiters[0].end_points[0].y - cur_pos_yn0)
        single_limiter_y_vec.append(slot.limiters[1].end_points[1].y - cur_pos_yn0)
      if slot.fusion_source == 1:
        slots_x1_vec.append(single_limiter_x_vec)
        slots_y1_vec.append(single_limiter_y_vec)
      elif slot.fusion_source == 2:
        slots_x2_vec.append(single_limiter_x_vec)
        slots_y2_vec.append(single_limiter_y_vec)
      elif slot.fusion_source == 3:
        slots_x3_vec.append(single_limiter_x_vec)
        slots_y3_vec.append(single_limiter_y_vec)

      # 3. update slot ids' text with their position
      id_vec.append(slot.id)
      id_text_x_vec.append((slot_plot_x_vec[0] + slot_plot_x_vec[1] + slot_plot_x_vec[2] + slot_plot_x_vec[3]) * 0.25)
      id_text_y_vec.append((slot_plot_y_vec[0] + slot_plot_y_vec[1] + slot_plot_y_vec[2] + slot_plot_y_vec[3]) * 0.25)
      if slot.allow_parking == 0:
        if slot.fusion_source == 1:
          occupied_x1_vec.append(slot_plot_x_vec)
          occupied_y1_vec.append(slot_plot_y_vec)
        elif slot.fusion_source == 2:
          occupied_x2_vec.append(slot_plot_x_vec)
          occupied_y2_vec.append(slot_plot_y_vec)
        elif slot.fusion_source == 3:
          occupied_x3_vec.append(slot_plot_x_vec)
          occupied_y3_vec.append(slot_plot_y_vec)

      # 4.
      if slot.allow_parking == 1:
        fus_release_solts_id.append(slot.id)

    local_view_data['data_fusion_slot0'].data.update({'corner_point_x': slots_x0_vec, 'corner_point_y': slots_y0_vec,})
    local_view_data['data_fusion_slot1'].data.update({'corner_point_x': slots_x1_vec, 'corner_point_y': slots_y1_vec,})
    local_view_data['data_fusion_slot2'].data.update({'corner_point_x': slots_x2_vec, 'corner_point_y': slots_y2_vec,})
    local_view_data['data_fusion_slot3'].data.update({'corner_point_x': slots_x3_vec, 'corner_point_y': slots_y3_vec,})
    local_view_data['data_fusion_parking_id'].data.update({'id':id_vec,'id_text_x':id_text_x_vec,'id_text_y':id_text_y_vec,})

    local_view_data['data_fusion_occupied_slot1'].data.update({
          'occupied_slot_y': occupied_y1_vec,
          'occupied_slot_x': occupied_x1_vec,
          })
    local_view_data['data_fusion_occupied_slot2'].data.update({
          'occupied_slot_y': occupied_y2_vec,
          'occupied_slot_x': occupied_x2_vec,
          })
    local_view_data['data_fusion_occupied_slot3'].data.update({
          'occupied_slot_y': occupied_y3_vec,
          'occupied_slot_x': occupied_x3_vec,
          })

    parking_fusion_slot_lists = bag_loader.fus_parking_msg['data'][-1].parking_fusion_slot_lists
    parking_fusion_slot_lists_size = bag_loader.fus_parking_msg['data'][-1].parking_fusion_slot_lists_size
    select_slot_id = bag_loader.fus_parking_msg['data'][-1].select_slot_id
    slots_x_vec = []
    slots_y_vec = []

    for j in range(parking_fusion_slot_lists_size):
      slot = parking_fusion_slot_lists[j]
      single_slot_x_vec = []
      single_slot_y_vec = []
      # attention: fusion slots are based on odom system, visual slots are based on vehicle system
      # 1. update slots corner points
      for k in range(corner_points_size):
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
    parking_slots = bag_loader.vis_parking_msg['data'][vis_parking_msg_idx].parking_slots
    parking_slots_size = bag_loader.vis_parking_msg['data'][vis_parking_msg_idx].parking_slots_size
    local_view_data['data_vision_parking'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    slots_x_vec = []
    slots_y_vec = []
    limiter_x_vec = []
    limiter_y_vec = []

    # attention: vision slots and limiters are based on vehicle system, needed to be transferred into global system
    # 1. updatge slot points
    for j in range(parking_slots_size):
      slot = parking_slots[j]
      single_slot_x_vec = []
      single_slot_y_vec = []
      slot_plot_x_vec = []
      slot_plot_y_vec = []
      for k in range(corner_points_size):
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
    vision_slot_limiters = bag_loader.vis_parking_msg['data'][vis_parking_msg_idx].vision_slot_limiters
    vision_slot_limiters_size = bag_loader.vis_parking_msg['data'][vis_parking_msg_idx].vision_slot_limiters_size
    for j in range(vision_slot_limiters_size):
      limiter = vision_slot_limiters[j]
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
    local_view_data['data_target_line'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_origin_pose'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_planning_slot'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_planning_line'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_planning_pose'].data.update({'corner_point_x': [], 'corner_point_y': [],})
    local_view_data['data_all_managed_limiter'].data.update({'limiter_point_y': [], 'limiter_point_x': [],})
    local_view_data['data_all_managed_slot_id'].data.update({'id':[], 'id_text_x':[], 'id_text_y':[],})
    slot_management_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].slot_management_info
    select_slot_id = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx].select_slot_id
    # print("select_slot_id in plan debug", select_slot_id)
    # print(slot_management_info)
    # print("**************************************************")
    all_managed_slot_x_vec = []
    all_managed_slot_y_vec = []
    occupied_x_vec = []
    occupied_y_vec = []
    id_vec = []
    id_text_x_vec = []
    id_text_y_vec = []
    # 1. update target managed slot
    for i in range(len(slot_management_info.slot_info_vec)):
      maganed_slot_vec = slot_management_info.slot_info_vec[i]
      corner_point = maganed_slot_vec.corner_points.corner_point
      if len(maganed_slot_vec.corner_points.corner_point) != 4:
        continue

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

      id_vec.append(maganed_slot_vec.id )
      id_text_x_vec.append((slot_x[0] + slot_x[1] + slot_x[2] + slot_x[3]) * 0.25)
      id_text_y_vec.append((slot_y[0] + slot_y[1] + slot_y[2] + slot_y[3]) * 0.25)

    local_view_data['data_all_managed_slot'].data.update({
          'corner_point_x': all_managed_slot_x_vec,
          'corner_point_y': all_managed_slot_y_vec,
          })


    local_view_data['data_all_managed_slot_id'].data.update({'id':id_vec,'id_text_x':id_text_x_vec,'id_text_y':id_text_y_vec,})

    local_view_data['data_all_managed_occupied_slot'].data.update({
          'occupied_slot_y': occupied_y_vec,
          'occupied_slot_x': occupied_x_vec,
          })

    obstacle_x = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['obstaclesX']
    obstacle_y = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['obstaclesY']

    replan_flag = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['replan_flag']
    correct_path_for_limiter = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['correct_path_for_limiter']
    slot_replan_jump_dist = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['slot_replan_jump_dist']
    slot_replan_jump_heading = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['slot_replan_jump_heading']

    local_view_data['data_obs'].data.update({
      'obs_x': obstacle_x,
      'obs_y': obstacle_y,
    })

    local_view_data['data_col_det_path'].data.update({
      'x': bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['col_det_path_x'],
      'y': bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['col_det_path_y'],
    })

    car_predict_x_vec = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['car_predict_x_vec']
    car_predict_y_vec = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['car_predict_y_vec']
    car_predict_heading_vec = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['car_predict_heading_vec']
    local_view_data['data_car_prediction_traj'].data.update({
      'x': car_predict_x_vec,
      'y': car_predict_y_vec,
    })

    temp_car_xb, temp_car_yb, wheel_base = load_car_params_patch_parking(vehicle_type, bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['car_real_time_col_lat_buffer'])
    car_box_x_vec = []
    car_box_y_vec = []
    for k in range(len(car_predict_x_vec)):
      car_xn = []
      car_yn = []
      for i in range(len(temp_car_xb)):
          tmp_x, tmp_y = local2global(temp_car_xb[i], temp_car_yb[i], car_predict_x_vec[k], car_predict_y_vec[k], car_predict_heading_vec[k])
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)
      car_box_x_vec.append(car_xn)
      car_box_y_vec.append(car_yn)
    local_view_data['data_car_prediction_traj_box'].data.update({
      'x_vec': car_box_x_vec,
      'y_vec': car_box_y_vec,
    })

    local_view_data['data_origin_pose'].data.update({
      'corner_point_x': [bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['slot_origin_pos_x']],
      'corner_point_y': [bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['slot_origin_pos_y']],
    })

    slot_corner_X = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['slot_corner_X']
    slot_corner_Y = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['slot_corner_Y']
    limiter_corner_X = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['limiter_corner_X']
    limiter_corner_Y = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]['limiter_corner_Y']

    if bag_loader.soc_state_msg['data'][soc_state_msg_idx].current_state != 14 and bag_loader.soc_state_msg['data'][soc_state_msg_idx].current_state != 18:
      if (len(slot_corner_X) > 3):
        local_view_data['data_target_managed_slot'].data.update({
          'corner_point_x': [slot_corner_X[0], slot_corner_X[2], slot_corner_X[3], slot_corner_X[1]],
          'corner_point_y': [slot_corner_Y[0], slot_corner_Y[2], slot_corner_Y[3], slot_corner_Y[1]],
        })

      if (len(slot_corner_X) > 12):
        local_view_data['data_target_line'].data.update({
          'corner_point_x': [slot_corner_X[4],slot_corner_X[5]],
          'corner_point_y': [slot_corner_Y[4],slot_corner_Y[5]],
        })

        local_view_data['data_planning_slot'].data.update({
          'corner_point_x': [slot_corner_X[6], slot_corner_X[8], slot_corner_X[9], slot_corner_X[7]],
          'corner_point_y': [slot_corner_Y[6], slot_corner_Y[8], slot_corner_Y[9], slot_corner_Y[7]],
        })
        local_view_data['data_planning_line'].data.update({
          'corner_point_x': [slot_corner_X[10],slot_corner_X[11]],
          'corner_point_y': [slot_corner_Y[10],slot_corner_Y[11]],
        })
        local_view_data['data_planning_pose'].data.update({
          'corner_point_x': [slot_corner_X[12]],
          'corner_point_y': [slot_corner_Y[12]],
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

  if bag_loader.uss_percept_msg['enable'] == True and bag_loader.loc_msg['enable'] == True and load_uss_wave_from_uss_percept_msg:
    # load uss wave from uss_percept_msg
    #get cur pose and uss wave
    uss_dis_info = bag_loader.uss_percept_msg['data'][uss_percept_msg_idx].dis_from_car_to_obj
    # uss_dis_info = bag_loader.uss_percept_msg['data'][uss_percept_msg_idx].out_line_dataori[0].dis_from_car_to_obj

    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw

    sector_x, sector_y, rs, start_angle, end_angle, length= [], [], [], [], [], []

    text_x, text_y = [], []
    # rs_text = []
    uss_x, uss_y = load_car_uss_patch(vehicle_type)
    uss_angle = load_uss_angle_patch(vehicle_type)
    wdis_index = [[8, 0, 1, 2, 3, 9],[10, 4, 5, 6, 7, 11]]
    m = 0
    for i in range(2):
      for j in wdis_index[i]:
          rs0 = ''
          rs1 = ''
          if uss_dis_info[j] * 0.001 <= 10 and uss_dis_info[i] * 0.001 != 0:
              rs1 = round(uss_dis_info[j] * 0.001, 2)
              # rs0 = '{:.2f}\n{:.2f}'.format(round(upa_dis_info_bufs[i].wdis[j].wdis_value[0], 2), round(upa_dis_info_bufs[i].wtype[j].wtype_value[0]))
              rs0 = '{:.2f}'.format(round(uss_dis_info[j] * 0.001, 2))
              ego_local_x, ego_local_y= local2global(uss_x[m], uss_y[m], cur_pos_xn, cur_pos_yn, cur_yaw)
              uss_angle_start = math.radians(uss_angle[m] - 30) + cur_yaw
              uss_angle_end = math.radians(uss_angle[m] +30) + cur_yaw
              x_text, y_text = one_echo_text_local(ego_local_x, ego_local_y, math.radians(uss_angle[m] - 90) + cur_yaw, rs1 - 0.5)
          elif uss_dis_info[j] * 0.001 == 0 or uss_dis_info[j] * 0.001 > 10:
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
  elif bag_loader.wave_msg['enable'] == True and bag_loader.loc_msg['enable'] == True and not load_uss_wave_from_uss_percept_msg:
    # load uss wave from wave_msg
    #get cur pose and uss wave
    upa_dis_info_bufs = bag_loader.wave_msg['data'][wave_msg_idx].upa_dis_info_buf
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw

    sector_x, sector_y, rs, start_angle, end_angle, length= [], [], [], [], [], []

    text_x, text_y = [], []
    # rs_text = []
    uss_x, uss_y = load_car_uss_patch(vehicle_type)
    uss_angle = load_uss_angle_patch(vehicle_type)
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

  if plot_ctrl_flag == True:
    names = []
    datas = []
    # load planning data
    if bag_loader.plan_msg['enable'] == True:
      plan_data = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx]
      plan_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]

      # load func_state
      if bag_loader.soc_state_msg['enable'] == True:
        names.append("current_state")
        datas.append(str(bag_loader.soc_state_msg['data'][soc_state_msg_idx].current_state))

      names.append("apa_planning_status")
      apa_planning_status = bag_loader.plan_msg['data'][plan_msg_idx].planning_status.apa_planning_status
      status_dict = {0: 'NONE', 1: 'IN_PROGRESS', 2: 'FINISHED', 3: 'FAILED'}
      status = status_dict.get(apa_planning_status, 'UNKNOWN')
      datas.append(str(apa_planning_status) + ": " + str(status))

      names.append("plan_release_slots_id")
      datas.append(str(bag_loader.plan_msg['data'][plan_msg_idx].successful_slot_info_list))

      names.append("fusion_release_slots_id")
      datas.append(str(fus_release_solts_id))

      names.append("selected_slot_confidence")
      datas.append(str(selected_slot_confidence))

      names.append("plan_gear_cmd")
      gear_command = bag_loader.plan_msg['data'][plan_msg_idx].gear_command.gear_command_value
      gear_command_dict = {0: 'GEAR_COMMAND_VALUE_NONE', 1: 'GEAR_COMMAND_VALUE_PARKING', 2: 'GEAR_COMMAND_VALUE_REVERSE', 3: 'GEAR_COMMAND_VALUE_NEUTRAL', 4: 'GEAR_COMMAND_VALUE_DRIVE', 5: 'GEAR_COMMAND_VALUE_LOW'}
      gear = gear_command_dict.get(gear_command, 'UNKNOWN')
      datas.append(str(gear_command) + ": " + str(gear))

      names.append("plan_traj_available")
      datas.append(str(bag_loader.plan_msg['data'][plan_msg_idx].trajectory.available))

      names.append("apa_planning_method")
      geometry_path_release = plan_json['geometry_path_release']
      if geometry_path_release:
        datas.append(str("geometry_plan"))
      else:
        datas.append(str("astar_plan"))

      names.append("planning_stm")
      planning_status = plan_json['planning_status']
      planning_stm_dict = {0: 'PARKING_IDLE', 1: 'PARKING_RUNNING', 2: 'PARKING_GEARCHANGE', 3: 'PARKING_PLANNING', 4: 'PARKING_FINISHED', 5: 'PARKING_FAILED', 6: 'PARKING_PAUSED'}
      planning_stm = planning_stm_dict.get(planning_status, 'UNKNOWN')
      datas.append(str(planning_status) + ": " + str(planning_stm))

      names.append("replan_reason")
      replan_reason = plan_json['replan_reason']
      replan_reason_dict = {0: 'NOT_REPLAN', 1: 'FIRST_PLAN', 2: 'SEG_COMPLETED_PATH', 3: 'SEG_COMPLETED_USS', 4: 'STUCKED', 5: 'DYNAMIC', 6: 'SEG_COMPLETED_COL_DET'}
      reason = replan_reason_dict.get(replan_reason, 'UNKNOWN')
      datas.append(str(replan_reason) + ": " + str(reason))

      names.append("plan_fail_reason")
      plan_fail_reason = plan_json['plan_fail_reason']
      plan_fail_reason_dict = {0: 'NOT_FAILED', 1: 'PAUSE_FAILED_TIME', 2: 'STUCK_FAILED_TIME', 3: 'UPDATE_EGO_SLOT_INFO', 4: 'POST_PROCESS_PATH_POINT_SIZE', 5: 'POST_PROCESS_PATH_POINT_SAME', 6: 'SET_SEG_INDEX', 7: 'CHECK_GEAR_LENGTH', 8: 'PATH_PLAN_FAILED', 9: 'PLAN_COUNT_EXCEED_LIMIT'}
      fail_reason = plan_fail_reason_dict.get(plan_fail_reason, 'UNKNOWN')
      datas.append(str(plan_fail_reason) + ": " + str(fail_reason))

      names.append("path_plan_success")
      datas.append(str(plan_json['path_plan_success']))

      names.append("path_plan_result")
      pathplan_result = plan_json['pathplan_result']
      pathplan_result_dict = {0: 'PLAN_FAILED', 1: 'PLAN_HOLD', 2: 'PLAN_UPDATE'}
      result = pathplan_result_dict.get(pathplan_result, 'UNKNOWN')
      datas.append(str(pathplan_result) + ": " + str(result))

      names.append("is_path_lateral_optimized")
      datas.append(str(plan_json['is_path_lateral_optimized']))

      names.append("terminal_error_x")
      datas.append(str(plan_json['terminal_error_x']))

      names.append("terminal_error_y")
      datas.append(str(plan_json['terminal_error_y']))

      names.append("terminal_error_y_front")
      datas.append(str(plan_json['terminal_error_y_front']))

      names.append("terminal_error_heading (deg)")
      datas.append(str(plan_json['terminal_error_heading'] * 57.3))

      names.append("move_slot_dist")
      datas.append(str(plan_json['move_slot_dist']))

      names.append("replan_move_slot_dist")
      datas.append(str(plan_json['replan_move_slot_dist']))

      names.append("replan_count")
      datas.append(str(plan_json['replan_count']))

      names.append("stuck_time (s)")
      datas.append(str(plan_json['stuck_time']))

      names.append("replan_consume_time (ms)")
      datas.append(str(plan_json['replan_consume_time']))

      names.append("total_plan_consume_time (ms)")
      datas.append(str(plan_json['total_plan_consume_time']))

      names.append("slot_occupied_ratio")
      datas.append(str(plan_json['slot_occupied_ratio']))

      names.append("remain_dist")
      datas.append(str(plan_json['remain_dist']))

      names.append("remain_dist_uss")
      datas.append(str(plan_json['remain_dist_uss']))

      names.append("remain_dist_col_det")
      datas.append(str(plan_json['remain_dist_col_det']))

      names.append("replan_flag")
      datas.append(str(plan_json['replan_flag']))

      names.append("replan_time_list")
      datas.append(str(replan_time_list))

      names.append("correct_path_for_limiter")
      datas.append(str(plan_json['correct_path_for_limiter']))

      names.append("correct_path_for_limiter_list")
      datas.append(str(correct_path_for_limiter_time_list))

      names.append("current_gear_length")
      datas.append(str(plan_json['current_gear_length']))

      names.append("current_gear_pt_size")
      datas.append(str(plan_json['current_gear_pt_size']))

      names.append("sample_ds")
      datas.append(str(plan_json['sample_ds']))

      names.append("pre_plan_case")
      pre_plan_case = plan_json['pre_plan_case']
      pre_plan_case_dict = {0: 'FAILED', 1: 'EGO_POSE', 2: 'MID_POINT'}
      pre_plan_case1 = pre_plan_case_dict.get(pre_plan_case, 'UNKNOWN')
      datas.append(str(pre_plan_case) + ": " + str(pre_plan_case1))

      names.append("slot_replan_jump_dist")
      datas.append(str(slot_replan_jump_dist))

      names.append("slot_replan_jump_heading")
      datas.append(str(slot_replan_jump_heading))

      names.append("current_path_length")
      datas.append(str(plan_json['current_path_length']))

      names.append("car_static_timer_by_pos_strict (s)")
      datas.append(str(plan_json['car_static_timer_by_pos_strict']))

      names.append("car_static_timer_by_pos_normal (s)")
      datas.append(str(plan_json['car_static_timer_by_pos_normal']))

      names.append("car_static_timer_by_vel_strict (s)")
      datas.append(str(plan_json['car_static_timer_by_vel_strict']))

      names.append("car_static_timer_by_vel_normal (s)")
      datas.append(str(plan_json['car_static_timer_by_vel_normal']))

      names.append("static_flag")
      datas.append(str(plan_json['static_flag']))

      names.append("slot_width")
      datas.append(str(plan_json['slot_width']))

      names.append("optimization_terminal_pose_error")
      datas.append(str(plan_json['optimization_terminal_pose_error']))

      names.append("optimization_terminal_heading_error")
      datas.append(str(plan_json['optimization_terminal_heading_error']))

      names.append("lat_path_opt_cost_time_ms")
      datas.append(str(plan_json['lat_path_opt_cost_time_ms']))

      names.append("statemachine_timestamp")
      datas.append(str(plan_json['statemachine_timestamp']))

      names.append("fusion_slot_timestamp")
      datas.append(str(plan_json['fusion_slot_timestamp']))

      names.append("localiztion_timestamp")
      datas.append(str(plan_json['localiztion_timestamp']))

      names.append("uss_wave_timestamp")
      datas.append(str(plan_json['uss_wave_timestamp']))

      names.append("uss_per_timestamp")
      datas.append(str(plan_json['uss_per_timestamp']))

      names.append("ground_line_timestamp")
      datas.append(str(plan_json['ground_line_timestamp']))

      names.append("fusion_objects_timestamp")
      datas.append(str(plan_json['fusion_objects_timestamp']))

      names.append("fusion_occupancy_objects_timestamp")
      datas.append(str(plan_json['fusion_occupancy_objects_timestamp']))

      names.append("control_output_timestamp")
      datas.append(str(plan_json['control_output_timestamp']))

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
      ctrl_gear_cmd = ctrl_json_data[ctrl_debug_msg_idx]['gear_cmd']
      ctrl_gear_cmd_dict = {0: 'GEAR_COMMAND_VALUE_NONE', 1: 'GEAR_COMMAND_VALUE_PARKING', 2: 'GEAR_COMMAND_VALUE_REVERSE', 3: 'GEAR_COMMAND_VALUE_NEUTRAL', 4: 'GEAR_COMMAND_VALUE_DRIVE', 5: 'GEAR_COMMAND_VALUE_LOW'}
      ctrl_gear = ctrl_gear_cmd_dict.get(ctrl_gear_cmd, 'UNKNOWN')
      datas.append(str(ctrl_gear_cmd) + ": " + str(ctrl_gear))

      names.append("gear_real")
      ctrl_gear_real = ctrl_json_data[ctrl_debug_msg_idx]['gear_real']
      ctrl_gear_real_dict = {0: 'GEAR_COMMAND_VALUE_NONE', 1: 'GEAR_COMMAND_VALUE_PARKING', 2: 'GEAR_COMMAND_VALUE_REVERSE', 3: 'GEAR_COMMAND_VALUE_NEUTRAL', 4: 'GEAR_COMMAND_VALUE_DRIVE', 5: 'GEAR_COMMAND_VALUE_LOW'}
      ctrl_gear = ctrl_gear_real_dict.get(ctrl_gear_real, 'UNKNOWN')
      datas.append(str(ctrl_gear_real) + ": " + str(ctrl_gear))

    local_view_data['ctrl_debug_data'].data.update({
      'name': names,
      'data': datas,
    })

  if bag_loader.uss_percept_msg['enable'] == True:
    model_x, model_y = [], []
    post_x, post_y = [], []
    parking_slot_x, parking_slot_y = [], []
    for i in range(NUM_OF_OUTLINE_DATAORI):
      single_out_line_dataori = bag_loader.uss_percept_msg['data'][uss_percept_msg_idx].out_line_dataori[i]
      if version_245:
        obj_count = single_out_line_dataori.obj_pt_cnt
      else:
        obj_count = NUM_OF_APA_SLOT_OBJ
      obj_count = min(NUM_OF_APA_SLOT_OBJ, obj_count)
      for j in range(obj_count):
        if version_245:
          x = single_out_line_dataori.obj_pt_global[j].x
          y = single_out_line_dataori.obj_pt_global[j].y
        else:
          x = single_out_line_dataori.obj_pt[j].x
          y = single_out_line_dataori.obj_pt[j].y

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

    # if len(bag_loader.uss_percept_msg['data'][uss_percept_msg_idx].uss_slots) != 0:
    #   for parking_slot in bag_loader.uss_percept_msg['data'][uss_percept_msg_idx].uss_slots:
    #     for corner_index in [0,3,2,1]:
    #       parking_slot_x.append(parking_slot.global_corner_point[corner_index].x)
    #       parking_slot_y.append(parking_slot.global_corner_point[corner_index].y)
    # local_view_data['data_spatial_parking_slot'].data.update({
    #   'corner_point_x': [parking_slot_x],
    #   'corner_point_y': [parking_slot_y],
    # })

  if bag_loader.uss_percept_msg['enable'] == True and load_uss_wave_from_uss_percept_msg:
    # load uss wave for uss_percept_msg
    #get cur pose and uss wave
    uss_dis_info = bag_loader.uss_percept_msg['data'][uss_percept_msg_idx].dis_from_car_to_obj
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw

    sector_x, sector_y, rs, start_angle, end_angle, length= [], [], [], [], [], []

    text_x, text_y = [], []
    # rs_text = []
    uss_x, uss_y = load_car_uss_patch(vehicle_type)
    uss_angle = load_uss_angle_patch(vehicle_type)
    wdis_index = [[8, 0, 1, 2, 3, 9],[10, 4, 5, 6, 7, 11]]
    m = 0
    for i in range(2):
      for j in wdis_index[i]:
          rs0 = ''
          rs1 = ''
          if uss_dis_info[j] * 0.001 <= 10 and uss_dis_info[i] * 0.001 != 0:
              rs1 = round(uss_dis_info[j] * 0.001, 2)
              # rs0 = '{:.2f}\n{:.2f}'.format(round(upa_dis_info_bufs[i].wdis[j].wdis_value[0], 2), round(upa_dis_info_bufs[i].wtype[j].wtype_value[0]))
              rs0 = '{:.2f}'.format(round(uss_dis_info[j] * 0.001, 2))
              ego_local_x, ego_local_y= local2global(uss_x[m], uss_y[m], cur_pos_xn, cur_pos_yn, cur_yaw)
              uss_angle_start = math.radians(uss_angle[m] - 30) + cur_yaw
              uss_angle_end = math.radians(uss_angle[m] +30) + cur_yaw
              x_text, y_text = one_echo_text_local(ego_local_x, ego_local_y, math.radians(uss_angle[m] - 90) + cur_yaw, rs1 - 0.5)
          elif uss_dis_info[j] * 0.001 == 0 or uss_dis_info[j] * 0.001 > 10:
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
  elif bag_loader.wave_msg['enable'] == True and bag_loader.loc_msg['enable'] == True and not load_uss_wave_from_uss_percept_msg:
    # load uss wave for wave_msg
    #get cur pose and uss wave
    upa_dis_info_bufs = bag_loader.wave_msg['data'][wave_msg_idx].upa_dis_info_buf
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw

    sector_x, sector_y, rs, start_angle, end_angle, length= [], [], [], [], [], []

    text_x, text_y = [], []
    # rs_text = []
    uss_x, uss_y = load_car_uss_patch(vehicle_type)
    uss_angle = load_uss_angle_patch(vehicle_type)
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

  obs_text_vec = []
  obs_text_x_vec = []
  obs_text_y_vec = []
  if bag_loader.fus_occupancy_objects_msg['enable'] == True and load_fusion_object_from_occupancy:
    pos_x, pos_y = [], []
    box_x_vec, box_y_vec = [], []

    local_view_data['data_fusion_obj_semantic'].data.update({'text':[], 'text_x':[], 'text_y':[],})

    for i in range(bag_loader.fus_occupancy_objects_msg['data'][fus_occupancy_objects_msg_idx].fusion_object_size):
      obj  =  bag_loader.fus_occupancy_objects_msg['data'][fus_occupancy_objects_msg_idx].fusion_object[i]

      # hack: need to retire
      # if obj.common_occupancy_info.type == 58:
      #   continue

      polygon_points =  obj.additional_occupancy_info.polygon_points
      for j in range(obj.additional_occupancy_info.polygon_points_size):
        x = polygon_points[j].x
        y = polygon_points[j].y
        pos_x.append(x)
        pos_y.append(y)
      center_x = obj.common_occupancy_info.center_position.x
      center_y = obj.common_occupancy_info.center_position.y
      heading = obj.common_occupancy_info.heading_angle
      length_ = obj.common_occupancy_info.shape.length
      width_ = obj.common_occupancy_info.shape.width
      box_x, box_y = rectangle_corners(center_x, center_y, length_, width_, heading)

      box_x_vec.append(box_x)
      box_y_vec.append(box_y)

      # plot text
      if len(polygon_points) > 0 :
        obs_text_vec.append(obj.common_occupancy_info.type)
        obs_text_x_vec.append(polygon_points[0].x)
        obs_text_y_vec.append(polygon_points[0].y)

    local_view_data['data_fusion_obj'].data.update({
      'y': pos_y,
      'x': pos_x,
    })

    # print("box_y_vec = ", box_y_vec)
    # print("box_x_vec = ", box_x_vec)

  if bag_loader.fus_objects_msg['enable'] == True and read_fus_obj_msg:
    pos_x, pos_y = [], []
    box_x_vec, box_y_vec = [], []

    for i in range(bag_loader.fus_objects_msg['data'][fus_objects_msg_idx].fusion_object_size):
      obj  =  bag_loader.fus_objects_msg['data'][fus_objects_msg_idx].fusion_object[i]

      if (obj.common_info.type == 23 or obj.common_info.type == 26):
        continue

      box_x, box_y = [], []
      center_x = obj.common_info.center_position.x
      center_y = obj.common_info.center_position.y
      heading = obj.common_info.heading_angle
      length_ = obj.common_info.shape.length
      width_ = obj.common_info.shape.width
      # print("length = ", length_, "  width = ", width_, "  center_x = ", center_x, "  center_y = ", center_y, "  heading = ", heading,)
      box_x, box_y = rectangle_corners(center_x, center_y, length_, width_, heading, True)
      box_x_vec.append(box_x)
      box_y_vec.append(box_y)

      # plot text
      speed = obj.common_info.velocity.x *obj.common_info.velocity.x + obj.common_info.velocity.y*obj.common_info.velocity.y
      speed = math.sqrt(speed)
      speed = '%.1f'%speed
      obs_text_vec.append(str(obj.common_info.type)+',v '+str(speed))
      obs_text_x_vec.append(center_x)
      obs_text_y_vec.append(center_y)

    # print("box_y_vec = ", box_y_vec)
    # print("box_x_vec = ", box_x_vec)

    local_view_data['data_fusion_obj_box'].data.update({
      'y': box_y_vec,
      'x': box_x_vec,
    })

  # plot occ+od type
  local_view_data['data_fusion_obj_semantic'].data.update({'text':obs_text_vec,'text_x':obs_text_x_vec,'text_y':obs_text_y_vec,})

  if bag_loader.fus_ground_line_msg['enable'] == True:
    pos_x, pos_y = [], []
    print("ground_lines_size = ", bag_loader.fus_ground_line_msg['data'][fus_ground_line_msg_idx].groundline_size)
    for i in range(bag_loader.fus_ground_line_msg['data'][fus_ground_line_msg_idx].groundline_size):
      ground_line = bag_loader.fus_ground_line_msg['data'][fus_ground_line_msg_idx].groundline[i]

      if(ground_line.resource_type != 2):
        continue

      for j in range(ground_line.groundline_point_size):
        point_2d = ground_line.groundline_point[j]
        pos_x.append(point_2d.x)
        pos_y.append(point_2d.y)
    # for ground_line in bag_loader.fus_ground_line_msg['data'][fus_ground_line_msg_idx].ground_lines:
    #   points_3d = ground_line.points_3d
    #   for point_3d in points_3d:
    #     pos_x.append(point_3d.x - cur_pos_xn0)
    #     pos_y.append(point_3d.y - cur_pos_yn0)

    local_view_data['data_ground_line_obj'].data.update({
      'yn': pos_y,
      'xn': pos_x,
    })

  return local_view_data

def load_local_view_figure_parking():
  data_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
  data_current_pos = ColumnDataSource(data = {'current_pos_y':[], 'current_pos_x':[]})
  data_current_line = ColumnDataSource(data = {'y':[], 'x':[]})
  data_current_front_line = ColumnDataSource(data = {'y':[], 'x':[]})
  data_car_target = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
  data_target_pos = ColumnDataSource(data = {'target_pos_y':[], 'target_pos_x':[]})
  data_car_target_line = ColumnDataSource(data = {'y':[], 'x':[]})
  data_front_target_line = ColumnDataSource(data = {'x':[], 'y':[]})
  data_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
  data_car_circle = ColumnDataSource(data = {'car_circle_yn':[], 'car_circle_xn':[], 'car_circle_rn':[]})
  data_ego = ColumnDataSource(data = {'ego_yn':[], 'ego_xn':[]})
  data_obs = ColumnDataSource(data = {'obs_x':[], 'obs_y':[]})
  data_col_det_path = ColumnDataSource(data = {'x':[], 'y':[]})
  data_text = ColumnDataSource(data = {'vel_ego_text':[]})

  data_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})

  data_complete_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                    'plan_traj_x':[],})

  data_control = ColumnDataSource(data = {'mpc_dx':[],
                                          'mpc_dy':[],})
  data_ref_mpc_vec = ColumnDataSource(data = {'dx_ref_mpc_vec':[], 'dy_ref_mpc_vec':[],})
  data_ref_vec = ColumnDataSource(data = {'dx_ref_vec':[], 'dy_ref_vec':[],})
  data_ehr_slot = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
  data_fusion_slot0 = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
  data_fusion_slot1 = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
  data_fusion_slot2= ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
  data_fusion_slot3 = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
  data_fusion_occupied_slot1 = ColumnDataSource(data = {'occupied_slot_y': [], 'occupied_slot_x': [],})
  data_fusion_occupied_slot2 = ColumnDataSource(data = {'occupied_slot_y': [], 'occupied_slot_x': [],})
  data_fusion_occupied_slot3 = ColumnDataSource(data = {'occupied_slot_y': [], 'occupied_slot_x': [],})
  data_vision_parking = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
  data_target_managed_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_target_line = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_origin_pose = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_planning_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_planning_line = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_planning_pose = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_final_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_fusion_parking_id = ColumnDataSource(data = {'id':[], 'id_text_x':[], 'id_text_y':[]})

  data_wave = ColumnDataSource(data = {'wave_x': [], 'wave_y': [], 'radius':[], 'start_angle':[], 'end_angle':[]})
  data_wave_min = ColumnDataSource(data = {'wave_x': [], 'wave_y': [], 'radius':[], 'start_angle':[], 'end_angle':[]})
  data_wave_length_text = ColumnDataSource(data = {'wave_text_x': [], 'wave_text_y': [], 'length':[]})

  data_fusion_obj = ColumnDataSource(data = {'y':[], 'x':[]})
  data_fusion_obj_box = ColumnDataSource(data = {'y':[], 'x':[]})
  data_fusion_obj_semantic = ColumnDataSource(data = {'text':[], 'text_x':[], 'text_y':[]})

  data_ground_line_obj = ColumnDataSource(data = {'yn':[], 'xn':[]})

  data_car_prediction_traj = ColumnDataSource(data = {'y':[], 'x':[]})
  data_car_prediction_traj_box = ColumnDataSource(data = {'y_vec':[], 'x_vec':[]})


  ctrl_debug_data = ColumnDataSource({
    'name':[],
    'data':[]
  })

  data_all_managed_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_all_managed_slot_id = ColumnDataSource(data = {'id':[], 'id_text_x':[], 'id_text_y':[]})
  data_all_managed_limiter = ColumnDataSource(data = {'limiter_point_y': [], 'limiter_point_x': [],})
  data_all_managed_occupied_slot = ColumnDataSource(data = {'occupied_slot_y': [], 'occupied_slot_x': [],})

  data_dluss_model = ColumnDataSource(data = {'obj_pt_y':[], 'obj_pt_x':[]})
  data_dluss_post = ColumnDataSource(data = {'obj_pt_y':[], 'obj_pt_x':[]})
  data_spatial_parking_slot = ColumnDataSource(data = {'corner_point_y':[], 'corner_point_x':[]})
  data_index = {'loc_msg_idx': 0,
                'road_msg_idx': 0,
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
                     'data_current_pos': data_current_pos, \
                     'data_current_line':data_current_line, \
                     'data_current_front_line':data_current_front_line, \
                     'data_car_target':data_car_target, \
                     'data_target_pos':data_target_pos, \
                     'data_car_target_line':data_car_target_line,\
                     'data_front_target_line':data_front_target_line,\
                     'data_car_box':data_car_box, \
                     'data_car_circle':data_car_circle, \
                     'data_ego':data_ego, \
                     'data_obs':data_obs, \
                     'data_col_det_path': data_col_det_path, \
                     'data_car_prediction_traj': data_car_prediction_traj, \
                     'data_car_prediction_traj_box': data_car_prediction_traj_box, \
                     'data_text':data_text, \
                     'data_planning':data_planning,\
                     'data_complete_planning':data_complete_planning,\
                     'data_control':data_control,\
                     'data_ref_mpc_vec':data_ref_mpc_vec, \
                     'data_ref_vec':data_ref_vec, \
                     'ctrl_debug_data':ctrl_debug_data, \
                     'data_ehr_slot':data_ehr_slot, \
                     'data_fusion_slot0':data_fusion_slot0, \
                     'data_fusion_slot1':data_fusion_slot1, \
                     'data_fusion_slot2':data_fusion_slot2, \
                     'data_fusion_slot3':data_fusion_slot3, \
                     'data_fusion_occupied_slot1':data_fusion_occupied_slot1,\
                     'data_fusion_occupied_slot2':data_fusion_occupied_slot2,\
                     'data_fusion_occupied_slot3':data_fusion_occupied_slot3,\
                     'data_vision_parking':data_vision_parking, \
                     'data_target_managed_slot':data_target_managed_slot, \
                     'data_target_line':data_target_line, \
                     'data_origin_pose':data_origin_pose, \
                     'data_planning_slot':data_planning_slot, \
                     'data_planning_line':data_planning_line, \
                     'data_planning_pose':data_planning_pose, \
                     'data_final_slot':data_final_slot, \
                     'data_fusion_parking_id':data_fusion_parking_id, \
                     'data_index': data_index, \
                     'data_wave':data_wave, \
                     'data_wave_min':data_wave_min, \
                     'data_wave_length_text':data_wave_length_text, \
                     'data_all_managed_slot':data_all_managed_slot,\
                     'data_all_managed_slot_id':data_all_managed_slot_id,\
                     'data_all_managed_limiter':data_all_managed_limiter,\
                     'data_all_managed_occupied_slot':data_all_managed_occupied_slot,\
                     'data_dluss_model':data_dluss_model,\
                     'data_dluss_post':data_dluss_post,\
                     'data_spatial_parking_slot':data_spatial_parking_slot,\
                     'data_fusion_obj':data_fusion_obj,\
                     'data_fusion_obj_box':data_fusion_obj_box,\
                     'data_fusion_obj_semantic':data_fusion_obj_semantic,\
                     'data_ground_line_obj' :data_ground_line_obj,\
                     }
  ### figures config
  fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=1250, height=1250, match_aspect = False, aspect_scale=1, y_range=(ego_init_x - 20.0, ego_init_x + 20.0), x_range=(ego_init_y + 20.0, ego_init_y - 20.0))
  #fig1.x_range.flipped = True
  # figure plot
  fig1.patch('car_yn', 'car_xn', source = data_car_target, fill_color = "pink", line_color = "red", line_width = 1, line_alpha = 0.5, legend_label = 'car_target')
  fig1.patch('car_yn', 'car_xn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, line_alpha = 0.5, legend_label = 'car')
  fig1.circle('current_pos_y','current_pos_x', source = data_current_pos, size=8, color='grey', legend_label = 'car')
  fig1.line('y', 'x', source = data_current_line, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.8, legend_label = 'car')
  fig1.line('y', 'x', source = data_current_front_line, line_width = 3.0, line_color = 'gray', line_dash = 'solid', line_alpha = 0.8, legend_label = 'car')
  fig1.circle('target_pos_y','target_pos_x', source = data_target_pos, size=8, color='blue', legend_label = 'car_target')
  fig1.line('y', 'x', source = data_car_target_line, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.8, legend_label = 'car_target')
  fig1.line('y', 'x', source = data_front_target_line, line_width = 3.0, line_color = 'gray', line_dash = 'solid', line_alpha = 0.8, legend_label = 'car_target')
  fig1.patches('y_vec', 'x_vec', source = data_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sampled carbox', visible = False)
  fig1.circle(x ='car_circle_yn', y ='car_circle_xn', radius = 'car_circle_rn', source = data_car_circle, line_alpha = 0.5, line_width = 1, line_color = "blue", fill_alpha=0, legend_label = 'car_circle', visible = False)
  fig1.line('ego_yn', 'ego_xn', source = data_ego, line_width = 1.5, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig1.circle('obs_y', 'obs_x', source = data_obs, size=8, color='green', legend_label='obs')
  fig1.text(0.0, -2.0, text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt", legend_label = 'text')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 2.5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_complete_planning, line_width = 2.5, line_color = 'red', line_dash = 'dashed', line_alpha = 0.6, legend_label = 'complete_planning', visible = False)
  # fig1.circle('y', 'x', source = data_col_det_path, size=4, color='red', legend_label = 'col_det_path')
  # fig1.line('y', 'x', source = data_col_det_path, line_width = 6, line_color = 'grey', line_dash = 'solid', line_alpha = 0.5, legend_label = 'col_det_path', visible = False)
  fig1.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 3.0, line_color = 'red', line_dash = 'solid', line_alpha = 0.8, legend_label = 'mpc', visible = False)
  # fig1.line('dy_ref_mpc_vec', 'dx_ref_mpc_vec', source = data_ref_mpc_vec, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.5, legend_label = 'data_ref_mpc_vec')
  # fig1.line('dy_ref_vec', 'dx_ref_vec', source = data_ref_vec, line_width = 3.0, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'data_ref_vec')

  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_vision_parking, line_width = 3, line_color = 'lightgrey', line_dash = 'solid',legend_label = 'vision_parking_slot', visible = False)
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_fusion_slot0, line_width = 2, line_color = 'black', line_dash = 'solid', line_alpha = 0.6, legend_label = 'map_parking_slot', visible = True)
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_fusion_slot1, line_width = 2, line_color = 'red', line_dash = 'solid', line_alpha = 0.6, legend_label = 'fusion_parking_slot', visible = True)
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_fusion_slot2, line_width = 2, line_color = 'cyan', line_dash = 'solid', line_alpha = 0.6, legend_label = 'fusion_parking_slot', visible = True)
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_fusion_slot3, line_width = 2, line_color = 'purple', line_dash = 'solid', line_alpha = 0.6, legend_label = 'fusion_parking_slot', visible = True)
  fig1.patches('occupied_slot_y', 'occupied_slot_x', source = data_fusion_occupied_slot1, fill_color = "red", line_color = "red", line_width = 1, fill_alpha = 0.15, legend_label = 'fusion_parking_slot', visible = True)
  fig1.patches('occupied_slot_y', 'occupied_slot_x', source = data_fusion_occupied_slot2, fill_color = "cyan", line_color = "cyan", line_width = 1, fill_alpha = 0.15, legend_label = 'fusion_parking_slot', visible = True)
  fig1.patches('occupied_slot_y', 'occupied_slot_x', source = data_fusion_occupied_slot3, fill_color = "purple", line_color = "purple", line_width = 1, fill_alpha = 0.15, legend_label = 'fusion_parking_slot', visible = True)


  fig1.line('corner_point_y', 'corner_point_x', source = data_target_managed_slot, line_width = 3, line_color = 'green', line_dash = 'solid',legend_label = 'target_managed_slot')
  fig1.line('corner_point_y', 'corner_point_x', source = data_target_line, line_width = 3, line_color = 'green', line_dash = 'dashed',legend_label = 'target_managed_slot')
  fig1.circle('corner_point_y', 'corner_point_x', source = data_origin_pose, size=6, color='grey', legend_label = 'planning_slot')
  fig1.line('corner_point_y', 'corner_point_x', source = data_planning_slot, line_width = 3, line_color = 'blue', line_dash = 'solid',legend_label = 'planning_slot')
  fig1.line('corner_point_y', 'corner_point_x', source = data_planning_line, line_width = 3, line_color = 'blue', line_dash = 'dashed',legend_label = 'planning_slot')
  fig1.circle('corner_point_y', 'corner_point_x', source = data_planning_pose, size=6, color='black', legend_label = 'planning_slot')
  fig1.line('corner_point_y', 'corner_point_x', source = data_final_slot, line_width = 3, line_color = '#A52A2A', line_dash = 'dashed',legend_label = 'final_parking_slot')

  fig1.text(x = 'id_text_y', y = 'id_text_x', text = 'id', source = data_fusion_parking_id, text_color='black', text_align='center', text_font_size='10pt',legend_label = 'fusion_parking_slot', visible = True)

  fig1.wedge('wave_x','wave_y', 'radius', 'start_angle', 'end_angle',source = data_wave, fill_color="lavender", line_color="black",legend_label = 'uss_wave',alpha = 0.5, visible = False)
  fig1.wedge('wave_x','wave_y', 'radius', 'start_angle', 'end_angle',source = data_wave_min, fill_color="blue", line_color="black",legend_label = 'uss_wave',alpha = 0.8, visible = False)
  fig1.text(x = 'wave_text_x', y = 'wave_text_y', text = 'length', source = data_wave_length_text, text_color='black', text_align='center', text_font_size='10pt',legend_label = 'uss_wave', visible = False)

  # debug
  # fig1.multi_line('corner_point_y', 'corner_point_x', source = data_all_managed_slot, line_width = 2, line_color = 'blue', line_dash = 'solid',legend_label = 'all managed slot', visible = False)
  # fig1.text(x = 'id_text_y', y = 'id_text_x', text = 'id', source = data_all_managed_slot_id, text_color='blue', text_align='center', text_font_size='10pt',legend_label = 'all managed slot', visible = False)

  fig1.line('limiter_point_y', 'limiter_point_x', source = data_all_managed_limiter, line_width = 3, line_color = 'blue', line_dash = 'solid', legend_label = 'managed limiter')
  # fig1.patches('occupied_slot_y', 'occupied_slot_x', source = data_all_managed_occupied_slot, fill_color = "blue", line_color = "blue", line_width = 1, fill_alpha = 0.15, legend_label = 'all managed slot', visible = False)

  # dluss
  fig1.circle('obj_pt_y','obj_pt_x', source = data_dluss_post, size=3, color='orange', legend_label = 'dluss_post', visible = True)
  fig1.circle('obj_pt_y','obj_pt_x', source = data_dluss_model, size=3, color='blue', legend_label = 'dluss_model', visible = False)
  fig1.multi_line('corner_point_y', 'corner_point_x', source = data_spatial_parking_slot, line_width = 2, line_color = 'orange', line_dash = 'solid',legend_label = 'spatial pariking slot', visible = False)
  fig1.circle('y','x', source = data_fusion_obj, size=3, color='blue', legend_label = 'fusion_objects', visible = True)
  fig1.circle('yn','xn', source = data_ground_line_obj, size=3, color='black', legend_label = 'ground line', visible = True)
  fig1.multi_line('y', 'x', source = data_fusion_obj_box, line_width = 1, line_color = 'black', line_dash = 'solid',legend_label = 'fusion_objects', visible = True)
  fig1.text(x = 'text_y', y = 'text_x', text = 'text', source = data_fusion_obj_semantic, text_color='black', text_align='center', text_font_size='10pt',legend_label = 'fusion_objects', visible = True)

  # car prediction traj
  fig1.circle('y', 'x', source = data_car_prediction_traj, size=4, color='orange', legend_label = 'car_prediction_traj', visible = False)
  fig1.line('y', 'x', source = data_car_prediction_traj, line_width = 6, line_color = 'orange', line_dash = 'dashed', line_alpha = 0.5, legend_label = 'car_prediction_traj', visible = False)
  fig1.patches('y_vec', 'x_vec', source = data_car_prediction_traj_box, fill_color = "#89FB89", fill_alpha = 0.0, line_color = "orange", line_width = 1, legend_label = 'car_prediction_traj', visible = False)


  # toolbar
  fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

  # legend
  fig1.legend.click_policy = 'hide'
  return fig1, local_view_data

def load_local_view_figure_parking_ctrl(bag_loader, local_view_data, max_time, dt=0.02):

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
  'vel_plan_target': [],
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
  vel_plan_target = []
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

  bag_time = 0.0
  while bag_time <= max_time + dt / 2:
      ctrl_debug_msg_idx = 0
      abs_t = bag_time + smallest_abs_t
      while bag_loader.ctrl_debug_msg['abs_t'][ctrl_debug_msg_idx] <= abs_t and ctrl_debug_msg_idx < (len(bag_loader.ctrl_debug_msg['abs_t'])-1):
          ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1

      t_debug.append(bag_time)
      ctrl_debug_json = bag_loader.ctrl_debug_msg['json'][ctrl_debug_msg_idx]
      controller_status.append(ctrl_debug_json['controller_status'])
      lon_enable.append(ctrl_debug_json['lon_enable'])
      lat_enable.append(ctrl_debug_json['lat_enable'])
      gear_plan.append(ctrl_debug_json['gear_plan'])

      vel_cmd_plan.append(ctrl_debug_json['vel_ref'])
      vel_plan_target.append(ctrl_debug_json['vel_ref_gain'])
      vel_cmd_pos.append(ctrl_debug_json['vel_cmd'])
      vel_measure.append(ctrl_debug_json['vel_ego'])

      path_length_plan.append(ctrl_debug_json['path_length_plan'])
      remain_s_plan.append(ctrl_debug_json['remain_s_plan'])
      remain_s_prebreak.append(ctrl_debug_json['remain_s_prebreak'])
      remain_s_uss.append(ctrl_debug_json['remain_s_uss'])
      remain_s_ctrl.append(ctrl_debug_json['remain_s_ctrl'])
      acc_cmd.append(ctrl_debug_json['vel_out'])
      vel_kp_term.append(ctrl_debug_json['vel_KP_term'])
      vel_ki_term.append(ctrl_debug_json['vel_KI_term'])
      tmp_throttle_brake = ctrl_debug_json['throttle_brake']
      if tmp_throttle_brake > 0.0:
        tmp_throttle_brake = tmp_throttle_brake / 1000
      throttle_brake.append(tmp_throttle_brake)
      acc_vel.append(ctrl_debug_json['acc_vel'])
      steer_angle_cmd.append(ctrl_debug_json['steer_angle_cmd'] * 57.3)
      steer_angle_measure.append(ctrl_debug_json['steer_angle'] * 57.3)
      driver_hand_torque.append(ctrl_debug_json['driver_hand_torque'])

      lat_err.append(ctrl_debug_json['lat_err'] * 100)
      phi_err.append(ctrl_debug_json['phi_err'] * 57.3)

      bag_time += dt

  data_control_global.data.update({
    'time': t_debug,

    'controller_status': controller_status,
    'lon_enable': lon_enable,
    'lat_enable': lat_enable,
    'gear_plan': gear_plan,

    'vel_cmd_plan': vel_cmd_plan,
    'vel_plan_target': vel_plan_target,
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
  fig3.line('time', 'vel_plan_target', source = data_control_global, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'vel_plan_target')
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

  hover3 = HoverTool(renderers=[f3], tooltips=[('time', '@time'), ('vel_cmd_plan', '@vel_cmd_plan'), ('vel_plan_target', '@vel_plan_target'), ('vel_cmd_pos', '@vel_cmd_pos'),
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

current_line_params={
  'line_width' : 3.0,
  'line_color' : 'black',
  'line_dash' : 'solid',
  'line_alpha' : 0.8,
  'legend_label' : 'car'
}

target_pos_params={
  'fill_color' : "pink",
  'line_color' : "red",
  'line_width' : 1,
  'line_alpha' : 0.5,
  'legend_label' : 'car_target'
}

target_dot_params_apa ={
  'size' : 8,
  'color' : "blue",
  'legend_label' : 'car_target'
}

target_line_params={
  'line_width' : 3.0,
  'line_color' : 'black',
  'line_dash' : 'solid',
  'line_alpha' : 0.8,
  'legend_label' : 'car_target'
}

sampled_carbox_params={
  'fill_color' : "#98FB98",
  'fill_alpha' : 0.0,
  'line_color' : "black",
  'line_width' : 1,
  'legend_label' : 'sampled carbox',
  'visible' : False
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
fusion_slot1_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "red",
  'line_alpha' : 0.6,
  'line_width' : 2,
  'legend_label' : 'fusion_parking_slot',
  'visible' : True
}
fusion_slot2_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "cyan",
  'line_alpha' : 0.6,
  'line_width' : 2,
  'legend_label' : 'fusion_parking_slot',
  'visible' : True
}
fusion_slot3_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "purple",
  'line_alpha' : 0.6,
  'line_width' : 2,
  'legend_label' : 'fusion_parking_slot',
  'visible' : True
}
fusion_occupied_slot1_params_apa = {
  'fill_color' : "red",
  'line_color' : "red",
  'line_width' : 1,
  'fill_alpha' : 0.15,
  'legend_label' : 'fusion_parking_slot',
  'visible' : True
}
fusion_occupied_slot2_params_apa = {
  'fill_color' : "cyan",
  'line_color' : "cyan",
  'line_width' : 1,
  'fill_alpha' : 0.15,
  'legend_label' : 'fusion_parking_slot',
  'visible' : True
}
fusion_occupied_slot3_params_apa = {
  'fill_color' : "purple",
  'line_color' : "purple",
  'line_width' : 1,
  'fill_alpha' : 0.15,
  'legend_label' : 'fusion_parking_slot',
  'visible' : True
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

target_slot_line_params_apa = {
  'line_dash' : 'dashed',
  'line_color' : "green",
  'line_width' : 3,
  'legend_label' : 'target_managed_slot'
}

origin_pose_params_apa = {
  "size" : 6,
  "color" : 'grey',
  'legend_label' : 'planning_slot'
}

planning_slot_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "blue",
  'line_width' : 3,
  'legend_label' : 'planning_slot'
}

planning_line_params_apa = {
  'line_dash' : 'dashed',
  'line_color' : "blue",
  'line_width' : 3,
  'legend_label' : 'planning_slot'
}

plan_pose_params_apa = {
  "size" : 6,
  "color" : 'black',
  'legend_label' : 'planning_slot'
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
slot_id_params_apa = { 'text_color' : "black", 'text_align':"center", 'text_font_size':"10pt", 'legend_label' : 'fusion_parking_slot' }

all_slot_id_params_apa = {
  'text_color' : "blue",
  'text_align':"center",
  'text_font_size':"10pt",
  'legend_label' : 'all managed slot',
  'visible' : False}

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

complete_plan_params = {
  'line_width' : 2.5, 'line_color' : 'red', 'line_dash' : 'dashed', 'line_alpha' : 0.6, 'legend_label' : 'complete_plan', 'visible' : False
}

mpc_params = {
  'line_width' : 3.0, 'line_color' : 'red', 'line_dash' : 'solid', 'line_alpha' : 0.8, 'legend_label' : 'mpc', 'visible' : False
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

col_det_path_params = {
  'line_width' : 6.0, 'line_color' : 'grey', 'line_dash' : 'solid', 'line_alpha' : 0.5, 'legend_label' : 'col_det_path', "visible" : False
}

table_params={
    'width': 600,
    'height':520,
}

dluss_post_params={
  "size" : 3,
  "color" : 'orange',
  "legend_label" : 'dluss_post',
  "visible" : True
}

dluss_model_params={
  "size" : 3,
  "color" : 'blue',
  "legend_label" : 'dluss_model',
  "visible" : False
}

ground_line_params={
  "size" : 3,
  "color" : 'black',
  "legend_label" : 'ground line',
  "visible" : True
}

fus_objects_params={
  "size" : 3,
  "color" : 'blue',
  "legend_label" : 'fusion_objects',
  "visible" : True
}

car_predict_traj_path_params1 = {
  'line_width' : 6,
  'line_color' : 'orange',
  'line_dash' : 'dashed',
  'line_alpha' : 0.5,
  'legend_label' : 'car_predict_traj_path',
  "visible" : False
}

car_predict_traj_path_params2={
  "size" : 4,
  "color" : 'orange',
  "legend_label" : 'car_predict_traj_path',
  "visible" : False
}

car_predict_traj_path_params3={
  'fill_color' : "#89FB89",
  'fill_alpha' : 0.0,
  'line_color' : "orange",
  'line_width' : 1,
  'legend_label' : 'car_predict_traj_path',
  'visible' : False
}

def apa_draw_local_view(dataLoader, layer_manager, max_time, time_step, vehicle_type, plot_ctrl_flag=False):
    #define figure
    car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type)
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
    global fus_occupancy_objects_timestamps
    global ground_line_timestamps
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
    fusion_slot_timestamps = []

    bag_time = 0.0
    while bag_time <= max_time + time_step / 2:
      # ctrl_debug_ts is the base time list
      ctrl_debug_ts.append(bag_time)

      abs_t = bag_time + smallest_abs_t

      loc_msg_idx = 0
      if dataLoader.loc_msg['enable'] == True:
        while dataLoader.loc_msg['abs_t'][loc_msg_idx] <= abs_t and loc_msg_idx < (len(dataLoader.loc_msg['abs_t'])-1):
            loc_msg_idx = loc_msg_idx + 1
        localization_timestamp = dataLoader.loc_msg['abs_t'][loc_msg_idx]
        localization_timestamps.append(localization_timestamp)

      vs_msg_idx = 0
      if dataLoader.vs_msg['enable'] == True:
        while dataLoader.vs_msg['abs_t'][vs_msg_idx] <= abs_t and vs_msg_idx < (len(dataLoader.vs_msg['abs_t'])-1):
            vs_msg_idx = vs_msg_idx + 1
        vehicle_service_timestamp = dataLoader.vs_msg['abs_t'][vs_msg_idx]
        vehicle_service_timestamps.append(vehicle_service_timestamp)

      fus_parking_msg_idx = 0
      if dataLoader.fus_parking_msg['enable'] == True:
        while dataLoader.fus_parking_msg['abs_t'][fus_parking_msg_idx] <= abs_t and fus_parking_msg_idx < (len(dataLoader.fus_parking_msg['abs_t'])-1):
          fus_parking_msg_idx = fus_parking_msg_idx + 1
        slot_timestamp = dataLoader.fus_parking_msg['abs_t'][fus_parking_msg_idx]
        fusion_slot_timestamps.append(slot_timestamp)

      fus_ground_line_msg_idx = 0
      if dataLoader.fus_ground_line_msg['enable'] == True:
        while dataLoader.fus_ground_line_msg['abs_t'][fus_ground_line_msg_idx] <= abs_t and fus_ground_line_msg_idx < (len(dataLoader.fus_ground_line_msg['abs_t'])-1):
            fus_ground_line_msg_idx = fus_ground_line_msg_idx + 1
        ground_line_timestamp = dataLoader.fus_ground_line_msg['abs_t'][fus_ground_line_msg_idx]
        ground_line_timestamps.append(ground_line_timestamp)

      fus_objects_msg_idx = 0
      if dataLoader.fus_objects_msg['enable'] == True:
        while dataLoader.fus_objects_msg['abs_t'][fus_objects_msg_idx] <= abs_t and fus_objects_msg_idx < (len(dataLoader.fus_objects_msg['abs_t'])-1):
            fus_objects_msg_idx = fus_objects_msg_idx + 1
        fusion_object_timestamp = dataLoader.fus_objects_msg['abs_t'][fus_objects_msg_idx]
        fusion_object_timestamps.append(fusion_object_timestamp)

      fus_occupancy_objects_msg_idx = 0
      if dataLoader.fus_occupancy_objects_msg['enable'] == True:
        while dataLoader.fus_occupancy_objects_msg['abs_t'][fus_occupancy_objects_msg_idx] <= abs_t and fus_occupancy_objects_msg_idx < (len(dataLoader.fus_occupancy_objects_msg['abs_t'])-1):
            fus_occupancy_objects_msg_idx = fus_occupancy_objects_msg_idx + 1
        fus_occupancy_objects_timestamp = dataLoader.fus_occupancy_objects_msg['abs_t'][fus_occupancy_objects_msg_idx]
        fus_occupancy_objects_timestamps.append(fus_occupancy_objects_timestamp)

      vis_parking_msg_idx = 0
      if dataLoader.vis_parking_msg['enable'] == True:
        while dataLoader.vis_parking_msg['abs_t'][vis_parking_msg_idx] <= abs_t and vis_parking_msg_idx < (len(dataLoader.vis_parking_msg['abs_t'])-1):
          vis_parking_msg_idx = vis_parking_msg_idx + 1
        vis_slot_timestamp = dataLoader.vis_parking_msg['abs_t'][vis_parking_msg_idx]
        vision_slot_timestamps.append(vis_slot_timestamp)

      plan_msg_idx = 0
      if dataLoader.plan_msg['enable'] == True:
        while dataLoader.plan_msg['abs_t'][plan_msg_idx] <= abs_t and plan_msg_idx < (len(dataLoader.plan_msg['abs_t'])-1):
            plan_msg_idx = plan_msg_idx + 1
        plan_timestamp = dataLoader.plan_msg['abs_t'][plan_msg_idx]
        plan_output_timestamps.append(plan_timestamp)

      plan_debug_msg_idx = 0
      if dataLoader.plan_debug_msg['enable'] == True:
        while dataLoader.plan_debug_msg['abs_t'][plan_debug_msg_idx] <= abs_t and plan_debug_msg_idx < (len(dataLoader.plan_debug_msg['abs_t'])-1):
            plan_debug_msg_idx = plan_debug_msg_idx + 1
        plan_debug_timestamp = dataLoader.plan_debug_msg['abs_t'][plan_debug_msg_idx]
        plan_debug_timestamps.append(plan_debug_timestamp)

      ctrl_msg_idx = 0
      if dataLoader.ctrl_msg['enable'] == True:
        while dataLoader.ctrl_msg['abs_t'][ctrl_msg_idx] <= abs_t and ctrl_msg_idx < (len(dataLoader.ctrl_msg['abs_t'])-1):
            ctrl_msg_idx = ctrl_msg_idx + 1
        control_output_timestamp = dataLoader.ctrl_msg['abs_t'][ctrl_msg_idx]
        control_output_timestamps.append(control_output_timestamp)

      ctrl_debug_msg_idx = 0
      if dataLoader.ctrl_debug_msg['enable'] == True:
        while dataLoader.ctrl_debug_msg['abs_t'][ctrl_debug_msg_idx] <= abs_t and ctrl_debug_msg_idx < (len(dataLoader.ctrl_debug_msg['abs_t'])-1):
            ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
        ctrl_debug_timestamp = dataLoader.ctrl_debug_msg['abs_t'][ctrl_debug_msg_idx]
        control_debug_timestamps.append(ctrl_debug_timestamp)

      soc_state_msg_idx = 0
      if dataLoader.soc_state_msg['enable'] == True:
        while dataLoader.soc_state_msg['abs_t'][soc_state_msg_idx] <= abs_t and soc_state_msg_idx < (len(dataLoader.soc_state_msg['abs_t'])-1):
            soc_state_msg_idx = soc_state_msg_idx + 1
        soc_timestamp = dataLoader.soc_state_msg['abs_t'][soc_state_msg_idx]
        soc_timestamps.append(soc_timestamp)

      wave_msg_idx = 0
      if dataLoader.wave_msg['enable'] == True:
        while dataLoader.wave_msg['abs_t'][wave_msg_idx] <= abs_t and wave_msg_idx < (len(dataLoader.wave_msg['abs_t'])-1):
            wave_msg_idx = wave_msg_idx + 1
        wave_timestamp = dataLoader.wave_msg['abs_t'][wave_msg_idx]
        wave_timestamps.append(wave_timestamp)

      uss_percept_msg_idx = 0
      if dataLoader.uss_percept_msg['enable'] == True:
        while dataLoader.uss_percept_msg['abs_t'][uss_percept_msg_idx] <= abs_t and uss_percept_msg_idx < (len(dataLoader.uss_percept_msg['abs_t'])-1):
            uss_percept_msg_idx = uss_percept_msg_idx + 1
        uss_timestamp = dataLoader.uss_percept_msg['abs_t'][uss_percept_msg_idx]
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

      flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
        # location_generator.xys.append(([],[]))
      else:
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_pos_theta = loc_msg.orientation.euler_boot.yaw
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
    # cur_pos_xn0 = cur_pos_xn = dataLoader.loc_msg['data'][0].position.position_boot.x
    # cur_pos_yn0 = cur_pos_yn = dataLoader.loc_msg['data'][0].position.position_boot.y
    for localization_timestamp in localization_timestamps:
      ego_xb, ego_yb = [], []
      # ego_xn, ego_yn = [], []
      flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
      else:
        cur_pos_xn = loc_msg.position.position_boot.x
        cur_pos_yn = loc_msg.position.position_boot.y
        cur_yaw = loc_msg.orientation.euler_boot.yaw
        ### global variables
        # pos offset
        for i in range(len(dataLoader.loc_msg['data'])):
          if (i % 10 != 0):
            continue
          pos_xn_i = dataLoader.loc_msg['data'][i].position.position_boot.x
          pos_yn_i = dataLoader.loc_msg['data'][i].position.position_boot.y

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
      flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
      else:
        vel_ego = dataLoader.loc_msg['data'][loc_msg_idx].velocity.velocity_body.vx

        if dataLoader.soc_state_msg['enable'] == True:
          flag, soc_msg = findt(dataLoader.soc_state_msg, soc_timestamps[loc_i])
          if not flag:
            print('find soc_msg error')
            current_state = -1
          else:
            current_state = soc_msg.current_state
        else:
          print('find soc_msg error')
          current_state = -1

        if dataLoader.vs_msg['enable'] == True:
          flag, vs_msg = findt(dataLoader.vs_msg, vehicle_service_timestamps[loc_i])
          if not flag:
            print('find vs_msg error')
            steer_deg = 600
          else:
            steer_deg = vs_msg.steering_wheel_angle * 57.3
        else:
          print('find vs_msg error')
          steer_deg = 600

        if dataLoader.ctrl_debug_msg['enable'] == True:
          flag, ctrl_debug_msg = findt_json(dataLoader.ctrl_debug_msg, control_debug_timestamps[loc_i])
          if not flag:
            print('find ctrl_debug_msg error')
            remain_s_ctrl = -1
          else:
            remain_s_ctrl = ctrl_debug_msg['remain_s_ctrl'] * 100
        else:
          print('find ctrl_debug_msg error')
          remain_s_ctrl = -1

        abs_t_string = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(localization_timestamp))
        text = 'v = {:.2f} m/s, remain_s_ctrl = {:.1f} cm, steer = {:.1f} deg, state = {:d}, time = {:s}, abs_t = {:} s'.format(round(vel_ego, 2), round(remain_s_ctrl, 1), round(steer_deg, 1), current_state, abs_t_string, round(localization_timestamp, 1))
        vel_text.append(text)
        vel_x.append(-2)
        vel_y.append(0)
      vs_text_generator.xys.append((vel_y, vel_x, vel_text))
    vs_text_generator.ts = np.array(ctrl_debug_ts)

  # load apa slot
    vision_slot_generate = CommonGenerator()
    fusion_slot1_generate = CommonGenerator()
    fusion_slot2_generate = CommonGenerator()
    fusion_slot3_generate = CommonGenerator()
    fusion_occupied_slot1_generate = CommonGenerator()
    fusion_occupied_slot2_generate = CommonGenerator()
    fusion_occupied_slot3_generate = CommonGenerator()
    target_slot_generate = CommonGenerator()
    target_slot_line_generate = CommonGenerator()
    origin_pose_generate = CommonGenerator()
    planning_slot_generate = CommonGenerator()
    planning_line_generate = CommonGenerator()
    plan_pose_generate = CommonGenerator()
    final_slot_generate = CommonGenerator()
    all_slot_generate = CommonGenerator()
    slot_id_generate = TextGenerator()
    all_slot_id_generate = TextGenerator()
    all_managed_occupied_slot_generate = CommonGenerator()
    all_managed_limiter_generate = CommonGenerator()
    tlane_generate = CommonGenerator()
    col_det_path_generate = CommonGenerator()
    car_predict_traj_path1_generate = CommonGenerator()
    car_predict_traj_path2_generate = CommonGenerator()
    car_predict_traj_path3_generate = CommonGenerator()

    for slot_i, slot_timestamp in enumerate(fusion_slot_timestamps):
        flag, fusion_slot_msg = findt(dataLoader.fus_parking_msg, slot_timestamp)
        if not flag:
            print('find fusion_slot_msg error')
            fusion_slot1_generate.xys.append(([], []))
            fusion_slot2_generate.xys.append(([], []))
            fusion_slot3_generate.xys.append(([], []))
            slot_id_generate.xys.append(([], [], []))
        else:
          # fusion slot
            parking_fusion_slot_lists = fusion_slot_msg.parking_fusion_slot_lists
            select_slot_id = fusion_slot_msg.select_slot_id
            # 1. update slots corner points
            temp_corner_x1_list = []
            temp_corner_y1_list = []
            temp_corner_x2_list = []
            temp_corner_y2_list = []
            temp_corner_x3_list = []
            temp_corner_y3_list = []
            temp_slot_id_list = []
            temp_slot_id_x_list = []
            temp_slot_id_y_list = []
            occupied_x1_list = []
            occupied_y1_list = []
            occupied_x2_list = []
            occupied_y2_list = []
            occupied_x3_list = []
            occupied_y3_list = []
            for slot in parking_fusion_slot_lists:
                temp_corner_x = []
                temp_corner_y = []
                for corner_point in slot.corner_points:
                    temp_corner_x.append(corner_point.x)
                    temp_corner_y.append(corner_point.y)
                temp_corner_x = [temp_corner_x[0],temp_corner_x[2],temp_corner_x[3],temp_corner_x[1]]
                temp_corner_y = [temp_corner_y[0],temp_corner_y[2],temp_corner_y[3],temp_corner_y[1]]
                if slot.fusion_source == 1:
                  temp_corner_x1_list.append(temp_corner_x)
                  temp_corner_y1_list.append(temp_corner_y)
                elif slot.fusion_source == 2:
                  temp_corner_x2_list.append(temp_corner_x)
                  temp_corner_y2_list.append(temp_corner_y)
                elif slot.fusion_source == 3:
                  temp_corner_x3_list.append(temp_corner_x)
                  temp_corner_y3_list.append(temp_corner_y)
                # 1.2 update slots limiter points in same slot_plot_vec
                single_limiter_x_vec = []
                single_limiter_y_vec = []
                if slot.limiters_size == 1:
                  single_limiter_x_vec.append(slot.limiters[0].end_points[0].x)
                  single_limiter_x_vec.append(slot.limiters[0].end_points[1].x)
                  single_limiter_y_vec.append(slot.limiters[0].end_points[0].y)
                  single_limiter_y_vec.append(slot.limiters[0].end_points[1].y)
                elif slot.limiters_size == 2:
                  single_limiter_x_vec.append(slot.limiters[0].end_points[0].x)
                  single_limiter_x_vec.append(slot.limiters[1].end_points[1].x)
                  single_limiter_y_vec.append(slot.limiters[0].end_points[0].y)
                  single_limiter_y_vec.append(slot.limiters[1].end_points[1].y)
                if slot.fusion_source == 1:
                  temp_corner_x1_list.append(single_limiter_x_vec)
                  temp_corner_y1_list.append(single_limiter_y_vec)
                elif slot.fusion_source == 2:
                  temp_corner_x2_list.append(single_limiter_x_vec)
                  temp_corner_y2_list.append(single_limiter_y_vec)
                elif slot.fusion_source == 3:
                  temp_corner_x3_list.append(single_limiter_x_vec)
                  temp_corner_y3_list.append(single_limiter_y_vec)
                # add slot id
                temp_slot_id = slot.id
                text = '{:d}'.format(round(temp_slot_id, 2))
                temp_slot_id_list.append(text)
                temp_slot_id_x_list.append((temp_corner_x[0]+temp_corner_x[2]+temp_corner_x[3]+temp_corner_x[1])/4)
                temp_slot_id_y_list.append((temp_corner_y[0]+temp_corner_y[2]+temp_corner_y[3]+temp_corner_y[1])/4)
                #
                if slot.allow_parking == 0:
                  if slot.fusion_source == 1:
                    occupied_x1_list.append(temp_corner_x)
                    occupied_y1_list.append(temp_corner_y)
                  elif slot.fusion_source == 2:
                    occupied_x2_list.append(temp_corner_x)
                    occupied_y2_list.append(temp_corner_y)
                  elif slot.fusion_source == 3:
                    occupied_x3_list.append(temp_corner_x)
                    occupied_y3_list.append(temp_corner_y)
            fusion_slot1_generate.xys.append((temp_corner_y1_list, temp_corner_x1_list))
            fusion_slot2_generate.xys.append((temp_corner_y2_list, temp_corner_x2_list))
            fusion_slot3_generate.xys.append((temp_corner_y3_list, temp_corner_x3_list))
            fusion_occupied_slot1_generate.xys.append((occupied_y1_list, occupied_x1_list))
            fusion_occupied_slot2_generate.xys.append((occupied_y2_list, occupied_x2_list))
            fusion_occupied_slot3_generate.xys.append((occupied_y3_list, occupied_x3_list))
            slot_id_generate.xys.append((temp_slot_id_y_list,temp_slot_id_x_list,temp_slot_id_list))

        vis_parking_msg_idx = 0
        if dataLoader.vis_parking_msg['enable'] == True:
          while dataLoader.vis_parking_msg['abs_t'][vis_parking_msg_idx] <= slot_timestamp and vis_parking_msg_idx < (len(dataLoader.vis_parking_msg['abs_t'])-1):
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
          parking_slots = dataLoader.vis_parking_msg['data'][vis_parking_msg_idx].parking_slots
          parking_slots_size = dataLoader.vis_parking_msg['data'][vis_parking_msg_idx].parking_slots_size
          if dataLoader.loc_msg['enable'] == True:
            flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamps[slot_i])
            if not flag:
              print('find loc error')
              temp_corner_x_list, temp_corner_y_list = [], []
            else:
              cur_pos_xn = loc_msg.position.position_boot.x
              cur_pos_yn = loc_msg.position.position_boot.y
              cur_yaw = loc_msg.orientation.euler_boot.yaw
              # attention: fusion slots are based on odom system, visual slots are based on vehicle system
              # 1. update slots corner points
              # coord_tf = coord_transformer()
              # coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
              for j in range(parking_slots_size):
                  slot = parking_slots[j]
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
              vision_slot_limiters = dataLoader.vis_parking_msg['data'][vis_parking_msg_idx].vision_slot_limiters
              for limiter in vision_slot_limiters:
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
          target_slot_line_x_list, target_slot_line_y_list = [], []
          plan_slot_corner_x_list, plan_slot_corner_y_list = [], []
          plan_slot_line_x_list, plan_slot_line_y_list = [], []
          plan_pose_x_list, plan_pose_y_list = [], []
          occupied_x_vec = []
          occupied_y_vec = []
          limiter_x_vec = []
          limiter_y_vec = []
          flag, plan_msg = findt(dataLoader.plan_debug_msg, plan_debug_timestamps[slot_i])
          flag, soc_msg = findt(dataLoader.soc_state_msg, soc_timestamps[slot_i])
          if not flag:
            print('find plan_msg error')
            all_slot_id_generate.xys.append(([], [], []))
          flag, plan_json = findt_json(dataLoader.plan_debug_msg, plan_debug_timestamps[slot_i])
          if not flag:
            print('find plan_msg error')
          else:
            slot_management_info = plan_msg.slot_management_info
            # select_slot_id = dataLoader.fus_parking_msg['data'][slot_i].select_slot_id
            select_slot_id = fusion_slot_msg.select_slot_id
            all_slot_id_list = []
            all_slot_id_x_list = []
            all_slot_id_y_list = []
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
              # add slot id
              maganed_slot_id = maganed_slot_vec.id
              text = '{:d}'.format(round(maganed_slot_id, 2))
              all_slot_id_list.append(text)
              all_slot_id_x_list.append((slot_x[0]+slot_x[2]+slot_x[3]+slot_x[1]) * 0.25)
              all_slot_id_y_list.append((slot_y[0]+slot_y[2]+slot_y[3]+slot_y[1]) * 0.25)

            for limiter in slot_management_info.limiter_points:
              limiter_x_vec.append(limiter.x)
              limiter_y_vec.append(limiter.y)

            car_predict_traj_x = plan_json['car_predict_x_vec']
            car_predict_traj_y = plan_json['car_predict_y_vec']
            car_predict_traj_heading = plan_json['car_predict_heading_vec']
            car_real_time_col_lat_buffer =  plan_json['car_real_time_col_lat_buffer']
            temp_car_xb, temp_car_yb, wheel_base = load_car_params_patch_parking(vehicle_type, car_real_time_col_lat_buffer)
            car_box_x_vec = []
            car_box_y_vec = []
            for k in range(len(car_predict_traj_x)):
              car_xn = []
              car_yn = []
              for i in range(len(temp_car_xb)):
                  tmp_x, tmp_y = local2global(temp_car_xb[i], temp_car_yb[i], car_predict_traj_x[k], car_predict_traj_y[k], car_predict_traj_heading[k])
                  car_xn.append(tmp_x)
                  car_yn.append(tmp_y)
              car_box_x_vec.append(car_xn)
              car_box_y_vec.append(car_yn)

            obstacle_x = plan_json['obstaclesX']
            obstacle_y = plan_json['obstaclesY']
            col_det_path_x = plan_json['col_det_path_x']
            col_det_path_y = plan_json['col_det_path_y']
            # tmp
            slot_corner_X = plan_json['slot_corner_X']
            slot_corner_Y = plan_json['slot_corner_Y']
            limiter_corner_X = plan_json['limiter_corner_X']
            limiter_corner_Y = plan_json['limiter_corner_Y']
            temp_corner_x_list = []
            temp_corner_y_list = []
            if (len(slot_corner_X) > 3):
              temp_corner_x_list = [[slot_corner_X[0], slot_corner_X[2], slot_corner_X[3], slot_corner_X[1]]]
              temp_corner_y_list = [[slot_corner_Y[0], slot_corner_Y[2], slot_corner_Y[3], slot_corner_Y[1]]]
            limiter_y_vec = limiter_corner_Y
            limiter_x_vec = limiter_corner_X

            if (len(slot_corner_X) > 12):
              target_slot_line_x_list = [slot_corner_X[4],slot_corner_X[5]]
              target_slot_line_y_list = [slot_corner_Y[4],slot_corner_Y[5]]
              plan_slot_corner_x_list = [[slot_corner_X[6], slot_corner_X[8], slot_corner_X[9], slot_corner_X[7]]]
              plan_slot_corner_y_list = [[slot_corner_Y[6], slot_corner_Y[8], slot_corner_Y[9], slot_corner_Y[7]]]
              plan_slot_line_x_list = [slot_corner_X[10],slot_corner_X[11]]
              plan_slot_line_y_list = [slot_corner_Y[10],slot_corner_Y[11]]
              plan_pose_x_list = [slot_corner_X[12]]
              plan_pose_y_list = [slot_corner_Y[12]]

          if soc_msg.current_state == 14 or soc_msg.current_state == 18:
            temp_corner_y_list, temp_corner_x_list, target_slot_line_y_list, target_slot_line_x_list, plan_slot_corner_y_list, plan_slot_corner_x_list, plan_slot_line_y_list, plan_slot_line_x_list, limiter_y_vec, limiter_x_vec = [], [], [], [], [], [], [], [], [], []
          origin_pose_generate.xys.append(([plan_json['slot_origin_pos_y']], [plan_json['slot_origin_pos_x']]))
          target_slot_generate.xys.append((temp_corner_y_list, temp_corner_x_list))
          target_slot_line_generate.xys.append((target_slot_line_y_list, target_slot_line_x_list))
          planning_slot_generate.xys.append((plan_slot_corner_y_list, plan_slot_corner_x_list))
          planning_line_generate.xys.append((plan_slot_line_y_list, plan_slot_line_x_list))
          plan_pose_generate.xys.append((plan_pose_y_list, plan_pose_x_list))
          all_managed_limiter_generate.xys.append((limiter_y_vec, limiter_x_vec))

          all_slot_generate.xys.append((all_managed_slot_y_vec, all_managed_slot_x_vec))
          all_slot_id_generate.xys.append((all_slot_id_y_list,all_slot_id_x_list,all_slot_id_list))
          all_managed_occupied_slot_generate.xys.append((occupied_y_vec, occupied_x_vec))
          tlane_generate.xys.append((obstacle_y, obstacle_x))
          col_det_path_generate.xys.append((col_det_path_y, col_det_path_x))
          car_predict_traj_path1_generate.xys.append((car_predict_traj_y, car_predict_traj_x))
          car_predict_traj_path2_generate.xys.append((car_predict_traj_y, car_predict_traj_x))
          car_predict_traj_path3_generate.xys.append((car_box_y_vec, car_box_x_vec))

  # load planning traj
    plan_generator = CommonGenerator()
    complete_plan_generator = CommonGenerator()
    target_line_generator = CommonGenerator()
    target_pos_generator = CommonGenerator()
    target_pt_generator = CommonGenerator()
    car_box_generator = CommonGenerator()
    for plan_i, plan_timestamp in enumerate(plan_output_timestamps):
      flag, plan_msg = findt(dataLoader.plan_msg, plan_timestamp)
      flag, plan_json = findt_json(dataLoader.plan_debug_msg, plan_debug_timestamps[plan_i])
      if not flag:
        print('find plan error')
        plan_traj_x, plan_traj_y, plan_heading = [], [], []
        target_line_xn, target_line_yn = [], []
        target_pos_xn, target_pos_yn = [], []
        target_pt_x, target_pt_y = [], []
        car_box_x_vec, car_box_y_vec = [], []
        complete_x_vec, complete_y_vec = [], []
      else:
        trajectory = plan_msg.trajectory
        plan_traj_x, plan_traj_y, plan_heading = [], [], []
        target_line_xn, target_line_yn = [], []
        target_pos_xn, target_pos_yn = [], []
        target_pt_x, target_pt_y = [], []
        car_box_x_vec, car_box_y_vec = [], []
        complete_x_vec, complete_y_vec = [], []
        for j in range(trajectory.trajectory_points_size):
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
          target_pt_x = [last_x]
          target_pt_y = [last_y]

          plan_traj_x_vec = plan_json["plan_traj_x"]
          plan_traj_y_vec = plan_json["plan_traj_y"]
          plan_traj_heading_vec = plan_json["plan_traj_heading"]
          plan_traj_lat_buffer_vec = plan_json["plan_traj_lat_buffer"]
          if len(plan_traj_x_vec) < 21:
            for i in range(len(plan_traj_x)):
              car_xn = []
              car_yn = []
              for j in range(len(car_xb)):
                tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], plan_traj_x[i], plan_traj_y[i], plan_heading[i])
                car_xn.append(tmp_x)
                car_yn.append(tmp_y)
              if (i % 5 == 0):
                car_box_x_vec.append(car_xn)
                car_box_y_vec.append(car_yn)
          else:
            for i in range(len(plan_traj_x_vec)):
              car_xn = []
              car_yn = []
              complete_x_vec.append(plan_traj_x_vec[i])
              complete_y_vec.append(plan_traj_y_vec[i])
              car_xb_temp, car_yb_temp, wheel_base_temp = load_car_params_patch_parking(vehicle_type, plan_traj_lat_buffer_vec[i])
              for j in range(len(car_xb_temp)):
                tmp_x, tmp_y = local2global(car_xb_temp[j], car_yb_temp[j], plan_traj_x_vec[i], plan_traj_y_vec[i], plan_traj_heading_vec[i])
                car_xn.append(tmp_x)
                car_yn.append(tmp_y)
              if (i % 5 == 0 or i == len(plan_traj_x_vec) - 1):
                car_box_x_vec.append(car_xn)
                car_box_y_vec.append(car_yn)

      plan_generator.xys.append((plan_traj_y, plan_traj_x))
      target_line_generator.xys.append((target_line_yn,target_line_xn))
      target_pt_generator.xys.append((target_pt_y,target_pt_x))
      target_pos_generator.xys.append(([target_pos_yn],[target_pos_xn]))
      car_box_generator.xys.append((car_box_y_vec,car_box_x_vec))
      complete_plan_generator.xys.append((complete_y_vec,complete_x_vec))

    target_pos_generator.ts = np.array(ctrl_debug_ts)
    target_pt_generator.ts = np.array(ctrl_debug_ts)
    plan_generator.ts = np.array(ctrl_debug_ts)
    target_line_generator.ts = np.array(ctrl_debug_ts)
    car_box_generator.ts = np.array(ctrl_debug_ts)
    complete_plan_generator.ts = np.array(ctrl_debug_ts)

  # load mpc traj
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
          cur_pos_xn = loc_msg.position.position_boot.x
          cur_pos_yn = loc_msg.position.position_boot.y
          cur_yaw = loc_msg.orientation.euler_boot.yaw
          coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
          control_result_points = mpc_msg.control_trajectory.control_result_points
          mpc_dx_local, mpc_dy_local = [], []
          for point in control_result_points:
            mpc_dx_local.append(point.x)
            mpc_dy_local.append(point.y)
          mpc_dx, mpc_dy = coord_tf.local_to_global(mpc_dx_local, mpc_dy_local)
      mpc_generator.xys.append((mpc_dy, mpc_dx))
    mpc_generator.ts = np.array(ctrl_debug_ts)

  # uss perception
    uss_post_generator = CommonGenerator()
    uss_model_generator = CommonGenerator()
    for uss_i, uss_percept_timestamp in enumerate(uss_percept_timestamps):
      flag, uss_percept_msg = findt(dataLoader.uss_percept_msg, uss_percept_timestamp)
      model_x, model_y = [], []
      post_x, post_y = [], []
      if not flag:
        print('find uss percept error')
      else:
        for i in range(NUM_OF_OUTLINE_DATAORI):
          single_out_line_dataori = uss_percept_msg.out_line_dataori[i]
          if version_245:
            obj_count = single_out_line_dataori.obj_pt_cnt
          else:
            obj_count = NUM_OF_APA_SLOT_OBJ
          obj_count = min(NUM_OF_APA_SLOT_OBJ, obj_count)
          for j in range(obj_count):
            if version_245:
              x = single_out_line_dataori.obj_pt_global[j].x
              y = single_out_line_dataori.obj_pt_global[j].y
            else:
              x = single_out_line_dataori.obj_pt[j].x
              y = single_out_line_dataori.obj_pt[j].y

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

  # ground line
    ground_line_generator = CommonGenerator()
    for gl_i, ground_line_timestamp in enumerate(ground_line_timestamps):
      flag, fus_ground_line_msg = findt(dataLoader.fus_ground_line_msg, ground_line_timestamp)
      pos_x, pos_y = [], []
      if not flag:
        print('find ground line error')
      else:
        #print("ground_lines_size = ",fus_ground_line_msg.ground_lines_size)
        for i in range(fus_ground_line_msg.ground_lines_size):
          ground_line = fus_ground_line_msg.ground_lines[i]
          points_3d = ground_line.points_3d
          for j in range(ground_line.points_3d_size):
            point_3d = points_3d[j]
            pos_x.append(point_3d.x)
            pos_y.append(point_3d.y)
        # for ground_line in fus_ground_line_msg.ground_lines:
        #   points_3d = ground_line.points_3d
        #   for point_3d in points_3d:
        #     pos_x.append(point_3d.x)
        #     pos_y.append(point_3d.y)
      ground_line_generator.xys.append((pos_y, pos_x))
    ground_line_generator.ts = np.array(ctrl_debug_ts)

  # fus objects
    fus_objects_generator = CommonGenerator()
    for fus_obj_i, fus_occupancy_objects_timestamp in enumerate(fus_occupancy_objects_timestamps):
      if dataLoader.fus_occupancy_objects_msg['enable'] == True and load_fusion_object_from_occupancy:
        flag, fus_occupancy_objects_msg = findt(dataLoader.fus_occupancy_objects_msg, fus_occupancy_objects_timestamp)
        pos_x, pos_y = [], []
        if not flag:
          print('find fus occupancy objects  error')
        else:
          for i in range(fus_occupancy_objects_msg.fusion_object_size):
            obj  =  fus_occupancy_objects_msg.fusion_object[i]
            polygon_points =  obj.additional_occupancy_info.polygon_points
            for j in range(obj.additional_occupancy_info.polygon_points_size):
              x = polygon_points[j].x
              y = polygon_points[j].y
              pos_x.append(x)
              pos_y.append(y)
      elif dataLoader.fus_objects_msg['enable'] == True and not load_fusion_object_from_occupancy:
        flag, fus_objects_msg = findt(dataLoader.fus_objects_msg, fusion_object_timestamps[fus_obj_i])
        pos_x, pos_y = [], []
        if not flag:
          print('find fus objects  error')
        else:
          for i in range(len(fus_objects_msg.fusion_object_size)):
            obj  =  fus_objects_msg.fusion_object[i]
            polygon_points = obj.additional_info.polygon_points
            for j in range(obj.additional_info.polygon_points_size):
              x = polygon_points[j].x
              y = polygon_points[j].y
              pos_x.append(x)
              pos_y.append(y)

      fus_objects_generator.xys.append((pos_y, pos_x))
    fus_objects_generator.ts = np.array(ctrl_debug_ts)

    # load cur pose and uss wave
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
      flag, loc_msg = findt(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        print('find loc_msg error')
      else:
        if dataLoader.uss_percept_msg['enable'] == True and load_uss_wave_from_uss_percept_msg:
          flag, uss_percept_msg = findt(dataLoader.uss_percept_msg, uss_percept_timestamps[loc_i])
          if not flag:
            print('find uss_percept_msg error')
          else:
            #get cur pose and uss wave
            if version_245:
              uss_dis_info = uss_percept_msg.dis_from_car_to_obj
            else:
              uss_dis_info = uss_percept_msg.out_line_dataori[0].dis_from_car_to_obj
            cur_pos_xn = loc_msg.position.position_boot.x
            cur_pos_yn = loc_msg.position.position_boot.y
            cur_yaw = loc_msg.orientation.euler_boot.yaw
            # rs_text = []
            uss_x, uss_y = load_car_uss_patch(vehicle_type)
            uss_angle = load_uss_angle_patch(vehicle_type)
            wdis_index = [[8, 0, 1, 2, 3, 9],[10, 4, 5, 6, 7, 11]]
            m = 0
            for i in range(2):
              for j in wdis_index[i]:
                  rs0 = ''
                  rs1 = ''
                  if uss_dis_info[j] * 0.001 <= 10 and uss_dis_info[i] * 0.001 != 0:
                      rs1 = round(uss_dis_info[j] * 0.001, 2)
                      # rs0 = '{:.2f}\n{:.2f}'.format(round(upa_dis_info_bufs[i].wdis[j].wdis_value[0], 2), round(upa_dis_info_bufs[i].wtype[j].wtype_value[0]))
                      rs0 = '{:.2f}'.format(round(uss_dis_info[j] * 0.001, 2))
                      ego_local_x, ego_local_y= local2global(uss_x[m], uss_y[m], cur_pos_xn, cur_pos_yn, cur_yaw)
                      uss_angle_start = math.radians(uss_angle[m] - 30) + cur_yaw
                      uss_angle_end = math.radians(uss_angle[m] +30) + cur_yaw
                      x_text, y_text = one_echo_text_local(ego_local_x, ego_local_y, math.radians(uss_angle[m] - 90) + cur_yaw, rs1 - 0.5)
                  elif uss_dis_info[j] * 0.001 == 0 or uss_dis_info[j] * 0.001 > 10:
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

            flag, plan_json = findt_json(dataLoader.plan_debug_msg, plan_debug_timestamps[loc_i])
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
            flag, soc_msg = findt(dataLoader.soc_state_msg, soc_timestamps[loc_i])
            if not flag:
              print('find soc_msg error')
              current_state = -1
            else:
              current_state = soc_msg.current_state

            if uss_available == True and uss_index <= len(sector_x) and current_state >= 29:
              sector_y_min, sector_x_min = [sector_y[uss_index]], [sector_x[uss_index]]
              rs_min, start_angle_min, end_angle_min = [rs[uss_index]], [start_angle[uss_index]], [end_angle[uss_index]]
        elif dataLoader.wave_msg['enable'] == True and not load_uss_wave_from_uss_percept_msg:
          flag, wave_msg = findt(dataLoader.wave_msg, wave_timestamps[loc_i])
          if not flag:
            print('find wave_msg error')
          else:
            #get cur pose and uss wave
            upa_dis_info_bufs = wave_msg.upa_dis_info_buf
            cur_pos_xn = loc_msg.position.position_boot.x
            cur_pos_yn = loc_msg.position.position_boot.y
            cur_yaw = loc_msg.orientation.euler_boot.yaw

            sector_x, sector_y, rs, start_angle, end_angle, length= [], [], [], [], [], []

            text_x, text_y = [], []
            # rs_text = []
            uss_x, uss_y = load_car_uss_patch(vehicle_type)
            uss_angle = load_uss_angle_patch(vehicle_type)
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

            flag, plan_json = findt_json(dataLoader.plan_debug_msg, plan_debug_timestamps[loc_i])
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
            flag, soc_msg = findt(dataLoader.soc_state_msg, soc_timestamps[loc_i])
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

    fusion_slot1_generate.ts = np.array(ctrl_debug_ts)
    fusion_slot2_generate.ts = np.array(ctrl_debug_ts)
    fusion_slot3_generate.ts = np.array(ctrl_debug_ts)
    fusion_occupied_slot1_generate.ts = np.array(ctrl_debug_ts)
    fusion_occupied_slot2_generate.ts = np.array(ctrl_debug_ts)
    fusion_occupied_slot3_generate.ts = np.array(ctrl_debug_ts)
    slot_id_generate.ts = np.array(ctrl_debug_ts)
    slot1_layer = MultiCurveLayer(fig_local_view ,fusion_slot1_params_apa)
    slot2_layer = MultiCurveLayer(fig_local_view ,fusion_slot2_params_apa)
    slot3_layer = MultiCurveLayer(fig_local_view ,fusion_slot3_params_apa)
    occupied_slot1_layer = PatchLayer(fig_local_view ,fusion_occupied_slot1_params_apa)
    occupied_slot2_layer = PatchLayer(fig_local_view ,fusion_occupied_slot2_params_apa)
    occupied_slot3_layer = PatchLayer(fig_local_view ,fusion_occupied_slot3_params_apa)
    slot_id_layer = TextLayer(fig_local_view,slot_id_params_apa)
    layer_manager.AddLayer(slot1_layer, 'slot1_layer', fusion_slot1_generate, 'fusion_slot1_generate', 2)
    layer_manager.AddLayer(slot2_layer, 'slot2_layer', fusion_slot2_generate, 'fusion_slot2_generate', 2)
    layer_manager.AddLayer(slot3_layer, 'slot3_layer', fusion_slot3_generate, 'fusion_slot3_generate', 2)
    layer_manager.AddLayer(occupied_slot1_layer, 'occupied_slot1_layer', fusion_occupied_slot1_generate, 'fusion_occupied_slot1_generate', 2)
    layer_manager.AddLayer(occupied_slot2_layer, 'occupied_slot2_layer', fusion_occupied_slot2_generate, 'fusion_occupied_slot2_generate', 2)
    layer_manager.AddLayer(occupied_slot3_layer, 'occupied_slot3_layer', fusion_occupied_slot3_generate, 'fusion_occupied_slot3_generate', 2)
    layer_manager.AddLayer(slot_id_layer, 'slot_id_layer',slot_id_generate,'slot_id_generate',3)

    if dataLoader.fus_parking_msg['enable'] == True:
      final_slot_generate.ts = np.array(ctrl_debug_ts)
      final_slot_layer = MultiCurveLayer(fig_local_view ,final_slot_params_apa)
      layer_manager.AddLayer(final_slot_layer, 'final_slot_layer',final_slot_generate,'final_slot_generate',2)

    if dataLoader.fus_parking_msg['enable'] == True and dataLoader.plan_debug_msg['enable'] == True:
      target_slot_generate.ts = np.array(ctrl_debug_ts)
      target_slot_line_generate.ts = np.array(ctrl_debug_ts)
      origin_pose_generate.ts = np.array(ctrl_debug_ts)
      planning_slot_generate.ts = np.array(ctrl_debug_ts)
      planning_line_generate.ts = np.array(ctrl_debug_ts)
      plan_pose_generate.ts = np.array(ctrl_debug_ts)
      all_slot_generate.ts = np.array(ctrl_debug_ts)
      all_slot_id_generate.ts = np.array(ctrl_debug_ts)
      all_managed_occupied_slot_generate.ts = np.array(ctrl_debug_ts)
      all_managed_limiter_generate.ts = np.array(ctrl_debug_ts)
      tlane_generate.ts = np.array(ctrl_debug_ts)
      col_det_path_generate.ts = np.array(ctrl_debug_ts)
      car_predict_traj_path1_generate.ts = np.array(ctrl_debug_ts)
      car_predict_traj_path2_generate.ts = np.array(ctrl_debug_ts)
      car_predict_traj_path3_generate.ts = np.array(ctrl_debug_ts)
      target_slot_layer = MultiCurveLayer(fig_local_view ,target_slot_params_apa)
      target_slot_line_layer = CurveLayer(fig_local_view ,target_slot_line_params_apa)
      origin_pose_layer = DotLayer(fig_local_view, origin_pose_params_apa)
      plan_slot_layer = MultiCurveLayer(fig_local_view, planning_slot_params_apa)
      plan_line_layer = CurveLayer(fig_local_view, planning_line_params_apa)
      plan_pose_layer = DotLayer(fig_local_view, plan_pose_params_apa)
      # all_slot_layer = MultiCurveLayer(fig_local_view ,all_slot_params_apa)
      # all_slot_id_layer = TextLayer(fig_local_view, all_slot_id_params_apa)
      # all_managed_occupied_slot_layer = PatchLayer(fig_local_view, all_managed_occupied_slot_params_apa)
      all_managed_limiter_layer = CurveLayer(fig_local_view, all_managed_limiter_params_apa)
      tlane_layer = DotLayer(fig_local_view, tlane_params)
      # col_det_path_layer = CurveLayer(fig_local_view, col_det_path_params)
      car_predict_traj_path1_layer = CurveLayer(fig_local_view, car_predict_traj_path_params1)
      car_predict_traj_path2_layer = DotLayer(fig_local_view, car_predict_traj_path_params2)
      car_predict_traj_path3_layer = PatchLayer(fig_local_view, car_predict_traj_path_params3)

      layer_manager.AddLayer(target_slot_layer, 'target_slot_layer',target_slot_generate,'target_slot_generate',2)
      layer_manager.AddLayer(target_slot_line_layer, 'target_slot_line_layer',target_slot_line_generate,'target_slot_line_generate',2)
      layer_manager.AddLayer(origin_pose_layer, 'origin_pose_layer',origin_pose_generate,'origin_pose_generate',2)
      layer_manager.AddLayer(plan_slot_layer, 'plan_slot_layer',planning_slot_generate,'planning_slot_generate',2)
      layer_manager.AddLayer(plan_line_layer, 'plan_line_layer',planning_line_generate,'planning_line_generate',2)
      layer_manager.AddLayer(plan_pose_layer, 'plan_pose_layer',plan_pose_generate,'plan_pose_generate',2)
      # layer_manager.AddLayer(all_slot_layer, 'all_slot_layer',all_slot_generate,'all_slot_generate',2)
      # layer_manager.AddLayer(all_slot_id_layer, 'all_slot_id_layer',all_slot_id_generate,'all_slot_id_generate',3)
      # layer_manager.AddLayer(all_managed_occupied_slot_layer, 'all_managed_occupied_slot_layer',all_managed_occupied_slot_generate,'all_managed_occupied_slot_generate',2)
      layer_manager.AddLayer(all_managed_limiter_layer, 'all_managed_limiter_layer',all_managed_limiter_generate,'all_managed_limiter_generate',2)
      layer_manager.AddLayer(tlane_layer, 'tlane_layer',tlane_generate,'tlane_generate',2)
      #layer_manager.AddLayer(col_det_path_layer, 'col_det_path_layer',col_det_path_generate,'col_det_path_generate',2)
      layer_manager.AddLayer(car_predict_traj_path1_layer, 'car_predict_traj_path1_layer',car_predict_traj_path1_generate,'car_predict_traj_path1_generate',2)
      layer_manager.AddLayer(car_predict_traj_path2_layer, 'car_predict_traj_path2_layer',car_predict_traj_path2_generate,'car_predict_traj_path2_generate',2)
      layer_manager.AddLayer(car_predict_traj_path3_layer, 'car_predict_traj_path3_layer',car_predict_traj_path3_generate,'car_predict_traj_path3_generate',2)

    # planning traj
    if dataLoader.plan_msg['enable'] == True:
      plan_layer = CurveLayer(fig_local_view, plan_params)
      layer_manager.AddLayer(plan_layer, 'plan_layer', plan_generator, 'plane_generator', 2)
      complete_plan_layer = CurveLayer(fig_local_view, complete_plan_params)
      layer_manager.AddLayer(complete_plan_layer, 'complete_plan_layer', complete_plan_generator, 'complete_plane_generator', 2)

    # mpc
    if dataLoader.ctrl_msg['enable'] == True:
      mpc_layer = CurveLayer(fig_local_view, mpc_params)
      layer_manager.AddLayer(mpc_layer, 'mpc_layer', mpc_generator, 'mpc_generator', 2)

    if dataLoader.plan_msg['enable'] == True:
      target_line_layer = CurveLayer(fig_local_view, target_line_params)
      layer_manager.AddLayer(target_line_layer, 'target_line_layer', target_line_generator, 'target_line_generator', 2)
      target_pt_layer = DotLayer(fig_local_view, target_dot_params_apa)
      layer_manager.AddLayer(target_pt_layer, 'target_pt_layer', target_pt_generator, 'target_pt_generator', 2)

    if dataLoader.loc_msg['enable'] == True:
      current_line_layer = CurveLayer(fig_local_view, current_line_params)
      layer_manager.AddLayer(current_line_layer, 'current_line_layer', current_line_generate, 'current_line_generate', 2)

    # uss
    if dataLoader.wave_msg['enable'] == True or dataLoader.uss_percept_msg['enable'] == True:
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

    # ground line
    if dataLoader.fus_ground_line_msg['enable'] == True:
      ground_line_layer = DotLayer(fig_local_view ,ground_line_params)
      layer_manager.AddLayer(ground_line_layer, 'ground_line_layer', ground_line_generator, 'ground_line_generator', 2)

    # fus objects
    if dataLoader.fus_objects_msg['enable'] == True or dataLoader.fus_occupancy_objects_msg['enable'] == True:
      fus_objects_layer = DotLayer(fig_local_view ,fus_objects_params)
      layer_manager.AddLayer(fus_objects_layer, 'fus_objects_layer', fus_objects_generator, 'fus_objects_generator', 2)

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
        if dataLoader.soc_state_msg['enable'] == True and dataLoader.plan_debug_msg['enable'] == True:
          flag, plan_msg = findt(dataLoader.plan_msg, plan_timestamp)
          if not flag:
            print('find plan error')
          flag, plan_json = findt_json(dataLoader.plan_debug_msg, plan_debug_timestamps[plan_i])
          flag, plan_data = findt(dataLoader.plan_debug_msg, plan_debug_timestamps[plan_i])
          if not flag:
            print('find plan_debug error')
          else:
            if dataLoader.soc_state_msg['enable'] == True:
              flag, soc_msg = findt(dataLoader.soc_state_msg, soc_timestamps[plan_i])
              if not flag:
                print('find soc_msg error')
              else:
                names.append("current_state")
                datas.append(str(soc_msg.current_state))
            else:
              print('find soc_msg error')

            names.append("apa_planning_status")
            apa_planning_status = plan_msg.planning_status.apa_planning_status
            status_dict = {0: 'NONE', 1: 'IN_PROGRESS', 2: 'FINISHED', 3: 'FAILED'}
            status = status_dict.get(apa_planning_status, 'UNKNOWN')
            datas.append(str(apa_planning_status) + ": " + str(status))

            names.append("plan_release_slots_id")
            datas.append(str(plan_msg.successful_slot_info_list))

            if dataLoader.fus_parking_msg['enable'] == True:
              flag, fusion_msg = findt(dataLoader.fus_parking_msg, fusion_slot_timestamps[plan_i])
              if not flag:
                print('find fusion_msg error')
              else:
                parking_fusion_slot_lists = fusion_msg.parking_fusion_slot_lists
                release_id = []
                selected_slot_confidence = 0
                for slot in parking_fusion_slot_lists:
                  if slot.allow_parking == 1:
                    release_id.append(slot.id)
                  if slot.id == fusion_msg.select_slot_id:
                    selected_slot_confidence = slot.confidence
                names.append("fusion_release_slots_id")
                datas.append(str(release_id))
                names.append("selected_slot_confidence")
                datas.append(str(selected_slot_confidence))
            else:
              print('find fusion_msg error')

            names.append("plan_gear_cmd")
            gear_command = plan_msg.gear_command.gear_command_value
            gear_command_dict = {0: 'GEAR_COMMAND_VALUE_NONE', 1: 'GEAR_COMMAND_VALUE_PARKING', 2: 'GEAR_COMMAND_VALUE_REVERSE', 3: 'GEAR_COMMAND_VALUE_NEUTRAL', 4: 'GEAR_COMMAND_VALUE_DRIVE', 5: 'GEAR_COMMAND_VALUE_LOW'}
            gear = gear_command_dict.get(gear_command, 'UNKNOWN')
            datas.append(str(gear_command) + ": " + str(gear))

            names.append("plan_traj_available")
            datas.append(str(plan_msg.trajectory.available))

            names.append("apa_planning_method")
            geometry_path_release = plan_json['geometry_path_release']
            if geometry_path_release:
              datas.append(str("geometry_plan"))
            else:
              datas.append(str("astar_plan"))

            names.append("planning_stm")
            planning_status = plan_json['planning_status']
            planning_stm_dict = {0: 'PARKING_IDLE', 1: 'PARKING_RUNNING', 2: 'PARKING_GEARCHANGE', 3: 'PARKING_PLANNING', 4: 'PARKING_FINISHED', 5: 'PARKING_FAILED', 6: 'PARKING_PAUSED'}
            planning_stm = planning_stm_dict.get(planning_status, 'UNKNOWN')
            datas.append(str(planning_status) + ": " + str(planning_stm))

            names.append("replan_reason")
            replan_reason = plan_json['replan_reason']
            replan_reason_dict = {0: 'NOT_REPLAN', 1: 'FIRST_PLAN', 2: 'SEG_COMPLETED_PATH', 3: 'SEG_COMPLETED_USS', 4: 'STUCKED', 5: 'DYNAMIC', 6: 'SEG_COMPLETED_COL_DET'}
            reason = replan_reason_dict.get(replan_reason, 'UNKNOWN')
            datas.append(str(replan_reason) + ": " + str(reason))

            names.append("plan_fail_reason")
            plan_fail_reason = plan_json['plan_fail_reason']
            plan_fail_reason_dict = {0: 'NOT_FAILED', 1: 'PAUSE_FAILED_TIME', 2: 'STUCK_FAILED_TIME', 3: 'UPDATE_EGO_SLOT_INFO', 4: 'POST_PROCESS_PATH_POINT_SIZE', 5: 'POST_PROCESS_PATH_POINT_SAME', 6: 'SET_SEG_INDEX', 7: 'CHECK_GEAR_LENGTH', 8: 'PATH_PLAN_FAILED', 9: 'PLAN_COUNT_EXCEED_LIMIT'}
            fail_reason = plan_fail_reason_dict.get(plan_fail_reason, 'UNKNOWN')
            datas.append(str(plan_fail_reason) + ": " + str(fail_reason))

            names.append("path_plan_success")
            datas.append(str(plan_json['path_plan_success']))

            names.append("path_plan_result")
            pathplan_result = plan_json['pathplan_result']
            pathplan_result_dict = {0: 'PLAN_FAILED', 1: 'PLAN_HOLD', 2: 'PLAN_UPDATE'}
            result = pathplan_result_dict.get(pathplan_result, 'UNKNOWN')
            datas.append(str(pathplan_result) + ": " + str(result))

            names.append("is_path_lateral_optimized")
            datas.append(str(plan_json['is_path_lateral_optimized']))

            names.append("terminal_error_x")
            datas.append(str(plan_json['terminal_error_x']))

            names.append("terminal_error_y")
            datas.append(str(plan_json['terminal_error_y']))

            names.append("terminal_error_y_front")
            datas.append(str(plan_json['terminal_error_y_front']))

            names.append("terminal_error_heading (deg)")
            datas.append(str(plan_json['terminal_error_heading'] * 57.3))

            names.append("move_slot_dist")
            datas.append(str(plan_json['move_slot_dist']))

            names.append("replan_move_slot_dist")
            datas.append(str(plan_json['replan_move_slot_dist']))

            names.append("replan_count")
            datas.append(str(plan_json['replan_count']))

            names.append("stuck_time (s)")
            datas.append(str(plan_json['stuck_time']))

            names.append("replan_consume_time (ms)")
            datas.append(str(plan_json['replan_consume_time']))

            names.append("total_plan_consume_time (ms)")
            datas.append(str(plan_json['total_plan_consume_time']))

            names.append("slot_occupied_ratio")
            datas.append(str(plan_json['slot_occupied_ratio']))

            names.append("replan_flag")
            datas.append(str(plan_json['replan_flag']))

            names.append("replan_time_list")
            datas.append(str(replan_time_list))

            names.append("correct_path_for_limiter")
            datas.append(str(plan_json['correct_path_for_limiter']))

            names.append("correct_path_for_limiter_list")
            datas.append(str(correct_path_for_limiter_time_list))

            names.append("slot_replan_jump_dist")
            datas.append(str(plan_json['slot_replan_jump_dist']))

            names.append("slot_replan_jump_heading")
            datas.append(str(plan_json['slot_replan_jump_heading']))

            names.append("current_gear_length")
            datas.append(str(plan_json['current_gear_length']))

            names.append("current_gear_pt_size")
            datas.append(str(plan_json['current_gear_pt_size']))

            names.append("sample_ds")
            datas.append(str(plan_json['sample_ds']))

            names.append("pre_plan_case")
            pre_plan_case = plan_json['pre_plan_case']
            pre_plan_case_dict = {0: 'FAILED', 1: 'EGO_POSE', 2: 'MID_POINT'}
            pre_plan_case1 = pre_plan_case_dict.get(pre_plan_case, 'UNKNOWN')
            datas.append(str(pre_plan_case) + ": " + str(pre_plan_case1))

            names.append("current_path_length")
            datas.append(str(plan_json['current_path_length']))

            names.append("remain_dist")
            datas.append(str(plan_json['remain_dist']))

            names.append("remain_dist_uss")
            datas.append(str(plan_json['remain_dist_uss']))

            names.append("remain_dist_col_det")
            datas.append(str(plan_json['remain_dist_col_det']))

            names.append("car_static_timer_by_pos_strict (s)")
            datas.append(str(plan_json['car_static_timer_by_pos_strict']))

            names.append("car_static_timer_by_pos_normal (s)")
            datas.append(str(plan_json['car_static_timer_by_pos_normal']))

            names.append("car_static_timer_by_vel_strict (s)")
            datas.append(str(plan_json['car_static_timer_by_vel_strict']))

            names.append("car_static_timer_by_vel_normal (s)")
            datas.append(str(plan_json['car_static_timer_by_vel_normal']))

            names.append("static_flag")
            datas.append(str(plan_json['static_flag']))

            names.append("slot_width")
            datas.append(str(plan_json['slot_width']))

            names.append("optimization_terminal_pose_error")
            datas.append(str(plan_json['optimization_terminal_pose_error']))

            names.append("optimization_terminal_heading_error")
            datas.append(str(plan_json['optimization_terminal_heading_error']))

            names.append("lat_path_opt_cost_time_ms")
            datas.append(str(plan_json['lat_path_opt_cost_time_ms']))

            names.append("statemachine_timestamp")
            datas.append(str(plan_json['statemachine_timestamp']))

            names.append("fusion_slot_timestamp")
            datas.append(str(plan_json['fusion_slot_timestamp']))

            names.append("localiztion_timestamp")
            datas.append(str(plan_json['localiztion_timestamp']))

            names.append("uss_wave_timestamp")
            datas.append(str(plan_json['uss_wave_timestamp']))

            names.append("uss_per_timestamp")
            datas.append(str(plan_json['uss_per_timestamp']))

            names.append("ground_line_timestamp")
            datas.append(str(plan_json['ground_line_timestamp']))

            names.append("fusion_objects_timestamp")
            datas.append(str(plan_json['fusion_objects_timestamp']))

            names.append("fusion_occupancy_objects_timestamp")
            datas.append(str(plan_json['fusion_occupancy_objects_timestamp']))

            names.append("control_output_timestamp")
            datas.append(str(plan_json['control_output_timestamp']))
        else:
          print('find plan or plan_debug error')

        if dataLoader.vs_msg['enable'] == True:
          flag, vs_msg = findt(dataLoader.vs_msg, vehicle_service_timestamps[plan_i])
          if not flag:
            print('find vs_msg error')
          else:
            names.append("long_control_actuator_status")
            datas.append(str(vs_msg.parking_long_control_actuator_status))
            names.append("lat_control_actuator_status")
            datas.append(str(vs_msg.parking_lat_control_actuator_status))
            names.append("shift_lever_state")
            datas.append(str(vs_msg.shift_lever_state))
            names.append("shift_lever_state_available")
            datas.append(str(vs_msg.shift_lever_state_available))
        else:
          print('find vs_msg error')

        if dataLoader.ctrl_debug_msg['enable'] == True:
          flag, ctrl_debug_msg = findt_json(dataLoader.ctrl_debug_msg, control_debug_timestamps[plan_i])
          if not flag:
            print('find ctrl_debug_msg error')
          else:
            names.append("lat_mpc_status")
            datas.append(ctrl_debug_msg['lat_mpc_status'])
            names.append("remain_s_uss")
            datas.append(ctrl_debug_msg['remain_s_uss'])
            names.append("remain_s_ctrl")
            datas.append(ctrl_debug_msg['remain_s_ctrl'])
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
            ctrl_gear_cmd = ctrl_debug_msg['gear_cmd']
            ctrl_gear_cmd_dict = {0: 'GEAR_COMMAND_VALUE_NONE', 1: 'GEAR_COMMAND_VALUE_PARKING', 2: 'GEAR_COMMAND_VALUE_REVERSE', 3: 'GEAR_COMMAND_VALUE_NEUTRAL', 4: 'GEAR_COMMAND_VALUE_DRIVE', 5: 'GEAR_COMMAND_VALUE_LOW'}
            ctrl_gear = ctrl_gear_cmd_dict.get(ctrl_gear_cmd, 'UNKNOWN')
            datas.append(str(ctrl_gear_cmd) + ": " + str(ctrl_gear))

            names.append("gear_real")
            ctrl_gear_real = ctrl_debug_msg['gear_real']
            ctrl_gear_real_dict = {0: 'GEAR_COMMAND_VALUE_NONE', 1: 'GEAR_COMMAND_VALUE_PARKING', 2: 'GEAR_COMMAND_VALUE_REVERSE', 3: 'GEAR_COMMAND_VALUE_NEUTRAL', 4: 'GEAR_COMMAND_VALUE_DRIVE', 5: 'GEAR_COMMAND_VALUE_LOW'}
            ctrl_gear = ctrl_gear_real_dict.get(ctrl_gear_real, 'UNKNOWN')
            datas.append(str(ctrl_gear_real) + ": " + str(ctrl_gear))
        else:
          print('find ctrl_debug_msg error')
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
      abs_t = bag_time + smallest_abs_t
      while dataLoader.ctrl_debug_msg['abs_t'][ctrl_debug_msg_idx] <= abs_t and ctrl_debug_msg_idx < (len(dataLoader.ctrl_debug_msg['abs_t'])-1):
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
                        "vel_ref", "vel_ref_gain", "vel_cmd", "vel_ego",
                        "path_length_plan", "remain_s_plan", "remain_s_prebreak", "remain_s_uss", "remain_s_ctrl",
                        "vel_out", "vel_KP_term", "vel_KI_term", "throttle_brake", 'acc_vel',
                        "steer_angle_cmd", "steer_angle", "driver_hand_torque",
                        "lat_err", "phi_err"
                        ]

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
    # "vel_ref", "vel_ref_gain", "vel_cmd", "vel_ego"
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
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_ref_gain"), "vel_plan_target")
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
