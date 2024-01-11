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
from bokeh.models import WheelZoomTool, HoverTool, TapTool, CustomJS, CheckboxGroup
from cyber_record.record import Record
from google.protobuf.json_format import MessageToJson

car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()
Max_line_size = 200
Road_boundary_max_line_size = 50
Lane_boundary_max_line_size = 300
timestamp_shrink = 1e6
is_vis_map = False
g_is_display_enu = True

class LoadCyberbag:
  def __init__(self, path) -> None:
    self.bag_path = path
    self.bag = Record(path)
    # loclization msg
    self.loc_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}
    
    self.old_loc_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}
    
    # road msg
    self.road_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # fusion object msg
    self.fus_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # me object msg
    self.me_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # radar_fm object msg
    self.radar_fm_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # radar_fl object msg
    self.radar_fl_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # radar_fr object msg
    self.radar_fr_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # radar_rl object msg
    self.radar_rl_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # radar_rr object msg
    self.radar_rr_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # vehicle service msg
    self.vs_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}
    # car pos in local coordinates

    # prediction_msg
    self.prediction_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

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

    # soc state machine
    self.soc_state_msg = {'abs_t':[], 't':[], 'data':[], 'json':[], 'enable':[]}

    # ehr static map msg
    self.ehr_static_map_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # ehr parking map msg
    self.ehr_parking_map_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}
    
    # ground_line_msg
    self.ground_line_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}
    
    # planning hmi msg
    self.planning_hmi_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # time offset
    t0 = 0

  def load_all_data(self):
    max_time = 0.0
    # load localization msg
    try:
      loc_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/localization/egomotion"):
        loc_msg_dict[msg.header.timestamp / timestamp_shrink] = msg
      loc_msg_dict = {key: val for key, val in sorted(loc_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in loc_msg_dict.items():
        self.loc_msg['t'].append(t)
        self.loc_msg['abs_t'].append(t)
        self.loc_msg['data'].append(msg)
      t0 = self.loc_msg['t'][0]
      print("T0 in loc msg:",t0)
      self.loc_msg['t'] = [tmp - t0  for tmp in self.loc_msg['t']]
      max_time = max(max_time, self.loc_msg['t'][-1])
      print('loc_msg time:',self.loc_msg['t'][-1])
      if len(self.loc_msg['t']) > 0:
        self.loc_msg['enable'] = True
      else:
        self.loc_msg['enable'] = False
    except:
      self.loc_msg['enable'] = False
      print('missing /iflytek/localization/egomotion !!!')

    # try:
    #   loc_msg_dict = {}
    #   for topic, msg, t in self.bag.read_messages("/iflytek/localization/ego_pose"):
    #     loc_msg_dict[msg.header.timestamp / timestamp_shrink] = msg
    #   loc_msg_dict = {key: val for key, val in sorted(loc_msg_dict.items(), key = lambda ele: ele[0])}
    #   for t, msg in loc_msg_dict.items():
    #     self.old_loc_msg['t'].append(t)
    #     self.old_loc_msg['abs_t'].append(t)
    #     self.old_loc_msg['data'].append(msg)
    #   t0 = self.old_loc_msg['t'][0]
    #   print("T0 in loc msg:",t0)
    #   self.old_loc_msg['t'] = [tmp - t0  for tmp in self.old_loc_msg['t']]
    #   max_time = max(max_time, self.old_loc_msg['t'][-1])
    #   print('loc_msg time:',self.old_loc_msg['t'][-1])
    #   if len(self.old_loc_msg['t']) > 0:
    #     self.old_loc_msg['enable'] = True
    #   else:
    #     self.old_loc_msg['enable'] = False
    # except:
    #   self.old_loc_msg['enable'] = False
    #   print('missing /iflytek/localization/ego_pose !!!')
      
    # load road_fusion msg
    try:
      road_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/road_fusion"):
        road_msg_dict[msg.header.timestamp / 1e6] = msg
      road_msg_dict = {key: val for key, val in sorted(road_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in road_msg_dict.items():
        self.road_msg['t'].append(t)
        self.road_msg['abs_t'].append(t)
        self.road_msg['data'].append(msg)
      self.road_msg['t'] = [tmp - t0  for tmp in self.road_msg['t']]
      print('road_msg time:',self.road_msg['t'][-1])
      if len(self.road_msg['t']) > 0:
        self.road_msg['enable'] = True
      else:
        self.road_msg['enable'] = False
    except:
      self.road_msg['enable'] = False
      print('missing /iflytek/fusion/road_fusion topic !!!')

    # load fusion objects msg
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

    # load mobile_eye_camera objects msg
    try:
      me_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/mobileye/camera_perception/objects"):
        me_msg_dict[msg.header.timestamp / 1e6] = msg
      me_msg_dict = {key: val for key, val in sorted(me_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in me_msg_dict.items():
        self.me_msg['t'].append(t)
        self.me_msg['abs_t'].append(t)
        self.me_msg['data'].append(msg)
      self.me_msg['t'] = [tmp - t0  for tmp in self.me_msg['t']]
      print('me_msg time:',self.me_msg['t'][-1])
      if len(self.me_msg['t']) > 0:
        self.me_msg['enable'] = True
      else:
        self.me_msg['enable'] = False
    except:
      self.me_msg['enable'] = False
      print('missing /mobileye/camera_perception/objects !!!')

    # load radar objects msg
    radar_msg = [self.radar_fm_msg,self.radar_fl_msg,self.radar_fr_msg,self.radar_rl_msg,self.radar_rr_msg]
    topic = ["/iflytek/radar_fm_perception_info","/iflytek/radar_fl_perception_info","/iflytek/radar_fr_perception_info","/iflytek/radar_rl_perception_info","/iflytek/radar_rr_perception_info"]
    # for i in range(5):
    #   print(topic[i])
    for i in range(5):
      try:
        radar_msg_dict = {}
        if i == 0:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_fm_perception_info"):
            radar_msg_dict[msg.header.timestamp / 1e6] = msg
        elif i == 1:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_fl_perception_info"):
            radar_msg_dict[msg.header.timestamp / 1e6] = msg
        elif i == 2:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_fr_perception_info"):
            radar_msg_dict[msg.header.timestamp / 1e6] = msg
        elif i == 3:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_rl_perception_info"):
            radar_msg_dict[msg.header.timestamp / 1e6] = msg
        elif i == 4:
          for topic, msg, t in self.bag.read_messages("/iflytek/radar_rr_perception_info"):
            radar_msg_dict[msg.header.timestamp / 1e6] = msg
        radar_msg_dict = {key: val for key, val in sorted(radar_msg_dict.items(), key = lambda ele: ele[0])}
        for t, msg in radar_msg_dict.items():
          radar_msg[i]['t'].append(t)
          radar_msg[i]['abs_t'].append(t)
          radar_msg[i]['data'].append(msg)
          temp_t = radar_msg[i]['t']
        radar_msg[i]['t'] = [tmp - t0  for tmp in temp_t]
          # print("load message:",i)
        # print(i,'_time:',radar_msg[i]['t'][-1])
        if len(radar_msg[i]['t']) > 0:
          radar_msg[i]['enable'] = True
          # print("true:",i)
        else:
          radar_msg[i]['enable'] = False
      except:
        radar_msg[i]['enable'] = False
        print('missing',topic[i])

    # # load radar_fl objects msg
    # try:
    #   radar_fl_msg_dict = {}
    #   for topic, msg, t in self.bag.read_messages("/iflytek/radar_fl_perception_info"):
    #     radar_fl_msg_dict[msg.header.timestamp / 1e6] = msg
    #   radar_fl_msg_dict = {key: val for key, val in sorted(radar_fl_msg_dict.items(), key = lambda ele: ele[0])}
    #   for t, msg in radar_fl_msg_dict.items():
    #     self.radar_fl_msg['t'].append(t)
    #     self.radar_fl_msg['abs_t'].append(t)
    #     self.radar_fl_msg['data'].append(msg)
    #   self.radar_fl_msg['t'] = [tmp - t0  for tmp in self.radar_fl_msg['t']]
    #   print('radar_fl_msg time:',self.radar_fl_msg['t'][-1])
    #   if len(self.radar_fl_msg['t']) > 0:
    #     self.radar_fl_msg['enable'] = True
    #   else:
    #     self.radar_fl_msg['enable'] = False
    # except:
    #   self.radar_fl_msg['enable'] = False
    #   print('missing /iflytek/radar_fl/objects !!!')

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
      self.plan_msg['t'] = [tmp - t0  for tmp in self.plan_msg['t']]
      max_time = max(max_time, self.plan_msg['t'][-1])
      print('plan_msg time:',self.plan_msg['t'][-1])
      if len(self.plan_msg['t']) > 0:
        self.plan_msg['enable'] = True
      else:
        self.plan_msg['enable'] = False
    except:
      self.plan_msg['enable'] = False
      print("missing /iflytek/planning/plan !!!")


    # load prediction msg
    try:
      prediction_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/prediction/prediction_result"):
        prediction_msg_dict[msg.header.timestamp / 1e6] = msg
      prediction_msg_dict = {key: val for key, val in sorted(prediction_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in prediction_msg_dict.items():
        self.prediction_msg['t'].append(t)
        self.prediction_msg['abs_t'].append(t)
        self.prediction_msg['data'].append(msg)
      self.prediction_msg['t'] = [tmp - t0  for tmp in self.prediction_msg['t']]
      self.prediction_msg['enable'] = True
      max_time = max(max_time, self.prediction_msg['t'][-1])
      print('prediction_msg time:',self.prediction_msg['t'][-1])
      if len(self.prediction_msg['t']) > 0:
        self.prediction_msg['enable'] = True
      else:
        self.prediction_msg['enable'] = False
    except:
      self.prediction_msg['enable'] = False
      print("missing /iflytek/prediction/prediction_result !!!")


    # load planning debug msg
    try:
      json_value_list = ["replan_status", "ego_pos_x", "ego_pos_y", "ego_pos_yaw", 'VisionLonBehavior_a_target_high',
                         "VisionLonBehavior_a_target_low", "VisionLonBehavior_v_target", "RealTime_v_ref",
                         "VisionLonBehavior_lead_one_id", "VisionLonBehavior_lead_one_dis", "VisionLonBehavior_lead_one_vel", "VisionLonBehavior_v_target_lead_one",
                         "VisionLonBehavior_lead_two_id", "VisionLonBehavior_lead_two_dis", "VisionLonBehavior_lead_two_vel", "VisionLonBehavior_v_target_lead_two",
                         "VisionLonBehavior_temp_lead_one_id", "VisionLonBehavior_temp_lead_one_dis", "VisionLonBehavior_temp_lead_one_vel", "VisionLonBehavior_v_target_temp_lead_one",
                         "VisionLonBehavior_temp_lead_two_id", "VisionLonBehavior_temp_lead_two_dis", "VisionLonBehavior_temp_lead_two_vel", "VisionLonBehavior_v_target_temp_lead_two",
                         "VisionLonBehavior_v_target_ramp", "VisionLonBehavior_road_radius", "dis_to_ramp",
                         "VisionLonBehavior_potental_cutin_track_id", "VisionLonBehavior_potental_cutin_v_target", "VisionLonBehavior_cutin_v_target",
                         "VisionLonBehavior_v_limit_road", "VisionLonBehavior_v_limit_in_turns",
                         "VisionLonBehavior_nearest_car_track_id_one", "VisionLonBehavior_nearest_car_track_id_two", "VisionLonBehavior_nearest_car_track_id_three",
                         "VisionLonBehavior_cutin_v_limit", "VisionLonBehavior_cutin_status",
                         'VisionLonBehavior_stop_start_state', 'VisionLonBehavior_v_target_start_stop', 'VisionLonBehavior_STANDSTILL', "VisionLonBehavior_final_v_target",
                         "solver_condition", "dist_err", "lat_err", "theta_err", "lon_err", "dbw_status",
                         "RealTime_v_ego", "RealTime_gap_v_limit_lc",
                         "REALTIME_fast_lead_id", "REALTIME_slow_lead_id", "REALTIME_fast_car_cut_in_id", "REALTIME_slow_car_cut_in_id",
                         "RealTime_lead_one_id", "RealTime_lead_one_distance", "RealTime_lead_one_velocity", "RealTime_lead_one_desire_vel",
                         "RealTime_lead_two_id", "RealTime_lead_two_distance", "RealTime_lead_two_velocity", "RealTime_lead_two_desire_vel",
                         "RealTime_temp_lead_one_id", "RealTime_temp_lead_one_distance", "RealTime_temp_lead_one_velocity", "RealTime_temp_lead_one_desire_vel",
                         "RealTime_temp_lead_two_id", "RealTime_temp_lead_two_distance", "RealTime_temp_lead_two_velocity", "RealTime_temp_lead_two_desire_vel",
                         "RealTime_potential_cutin_track_id", "RealTime_potential_cutin_v_target", "RealTime_desired_distance_rss", "RealTime_desired_distance_calibrate",
                         "RealTimeLateralMotionCostTime", "EnvironmentalModelManagerCost", "GeneralPlannerModuleCostTime",
                         "RealTimeLonBehaviorCostTime", "RealTimeLonMotionCostTime", "RealTime_stop_start_state", "RealTime_v_target_start_stop", "RealTime_STANDSTILL"]

      json_vector_list = ["raw_refline_x_vec", "raw_refline_y_vec", "assembled_x", "assembled_y", "assembled_theta", "assembled_delta", "assembled_omega", "traj_s_vec", "traj_x_vec", "traj_y_vec", "limit_v_type"]

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
          try:
            # print(json_struct['assembled_omega'])
            print(json_struct['limit_v_type'])
          except:
            pass
          self.plan_debug_msg['json'].append(json_data)
        except json.decoder.JSONDecodeError as jserr:
          print('except',jserr)

      self.plan_debug_msg['t'] = [tmp - t0  for tmp in self.plan_debug_msg['t']]
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
      self.ctrl_msg['t'] = [tmp - t0  for tmp in self.ctrl_msg['t']]
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
      json_value_list = ["steer_angle_cmd", "steer_angle", "acc_ego", "acc_vel", "vel_ego", "vel_wheel", "wheel_angle_cmd",
        "slope_acc", "vel_cmd",  "vel_raw_cmd","vel_error", "vel_fdbk_out", "vel_raw_error", "vel_ffwd_out", "vel_out",
        "vel_raw_out", "lon_err", "lat_err", "phi_err", "controller_status", "driver_hand_torque", "lat_enable", "lon_enable",
        "lat_mpc_status", "planning_type", "planning_time_offset", "planning_update_flag", "vel_KP_term", "vel_KI_term", "yaw_conti",
        "steer_angle_bias_deg", "steer_bias_deg", "axle_torque", "throttle_brake", "euler_angle_pitch", "euler_angle_yaw" ]

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

    # load ehr static map msg
    try:
      ehr_static_map_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/ehr/static_map"):
        ehr_static_map_msg_dict[msg.header.timestamp / 1e3] = msg
      ehr_static_map_msg_dict = {key: val for key, val in sorted(ehr_static_map_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ehr_static_map_msg_dict.items():
        self.ehr_static_map_msg['t'].append(t)
        self.ehr_static_map_msg['abs_t'].append(t)
        self.ehr_static_map_msg['data'].append(msg)
      self.ehr_static_map_msg['t'] = [tmp - t0  for tmp in self.ehr_static_map_msg['t']]
      print('ehr_static_map_msg time:',self.ehr_static_map_msg['t'][-1])
      if len(self.ehr_static_map_msg['t']) > 0:
        self.ehr_static_map_msg['enable'] = True
      else:
        self.ehr_static_map_msg['enable'] = False
    except:
      self.ehr_static_map_msg['enable'] = False
      print('missing /iflytek/ehr/static_map topic !!!')

    # load ehr parking map msg
    try:
      ehr_parking_map_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/ehr/parking_map"):
        ehr_parking_map_msg_dict[msg.header.timestamp / 1e6] = msg
      ehr_parking_map_msg_dict = {key: val for key, val in sorted(ehr_parking_map_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ehr_parking_map_msg_dict.items():
        self.ehr_parking_map_msg['t'].append(t)
        self.ehr_parking_map_msg['abs_t'].append(t)
        self.ehr_parking_map_msg['data'].append(msg)
      self.ehr_parking_map_msg['t'] = [tmp - t0  for tmp in self.ehr_parking_map_msg['t']]
      print('ehr_parking_map_msg time:',self.ehr_parking_map_msg['t'][-1])
      if len(self.ehr_parking_map_msg['t']) > 0:
        self.ehr_parking_map_msg['enable'] = True
      else:
        self.ehr_parking_map_msg['enable'] = False
    except:
      self.ehr_parking_map_msg['enable'] = False
      print('missing /iflytek/ehr/parking_map topic !!!')
      
    # load ground_line_msg
    try:
      ground_line_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/ground_line"):
        ground_line_msg_dict[msg.header.timestamp / 1e6] = msg
      ground_line_msg_dict = {key: val for key, val in sorted(ground_line_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in ground_line_msg_dict.items():
        self.ground_line_msg['t'].append(t)
        self.ground_line_msg['abs_t'].append(t)
        self.ground_line_msg['data'].append(msg)
      self.ground_line_msg['t'] = [tmp - t0  for tmp in self.ground_line_msg['t']]
      print('ground_line_msg time:',self.ground_line_msg['t'][-1])
      if len(self.ground_line_msg['t']) > 0:
        self.ground_line_msg['enable'] = True
      else:
        self.ground_line_msg['enable'] = False
    except:
      self.ground_line_msg['enable'] = False
      print('missing /iflytek/fusion/ground_line topic !!!')
        
    # load planning hmi msg
    try:
      planning_hmi_msg_dict = {}
      for topic, msg, t in self.bag.read_messages("/iflytek/planning/hmi"):
        planning_hmi_msg_dict[msg.header.timestamp / 1e6] = msg
      planning_hmi_msg_dict = {key: val for key, val in sorted(planning_hmi_msg_dict.items(), key = lambda ele: ele[0])}
      for t, msg in planning_hmi_msg_dict.items():
        self.planning_hmi_msg['t'].append(t)
        self.planning_hmi_msg['abs_t'].append(t)
        self.planning_hmi_msg['data'].append(msg)
      self.planning_hmi_msg['t'] = [tmp - t0  for tmp in self.planning_hmi_msg['t']]
      print('planning_hmi_msg time:',self.planning_hmi_msg['t'][-1])
      if len(self.planning_hmi_msg['t']) > 0:
        self.planning_hmi_msg['enable'] = True
      else:
        self.planning_hmi_msg['enable'] = False
    except:
      self.planning_hmi_msg['enable'] = False
      print('missing /iflytek/planning/hmi topic !!!')
    return max_time
#/mobileye/camera_perception/objects
  def msg_timeline_figure(self):
    topic_list = [
      '/iflytek/localization/egomotion',
      '/iflytek/localization/ego_pose',
      '/iflytek/fusion/road_fusion',
      '/iflytek/fusion/objects',
      '/mobileye/camera_perception/objects',
      '/iflytek/radar_fm_perception_info',
      '/iflytek/radar_fl_perception_info',
      '/iflytek/radar_fr_perception_info',
      '/iflytek/radar_rl_perception_info',
      '/iflytek/radar_rr_perception_info',
      '/iflytek/vehicle_service',
      '/iflytek/prediction/prediction_result',
      '/iflytek/planning/plan',
      '/iflytek/planning/debug_info',
      '/iflytek/control/control_command',
      '/iflytek/control/debug_info',
      '/iflytek/fusion/parking_slot',
      '/iflytek/system_state/soc_state',
    ]
    detail_list = [
      '/iflytek/planning/plan',
      '/iflytek/planning/debug_info',
      '/iflytek/system_state/soc_state',
    ]
    print("========【使用说明】========")
    print("========鼠标滚轮缩放时间轴，鼠标悬停或点击以下topic的点，可以在浏览器控制台（F12打开）查看该msg具体内容: ")
    print("========", detail_list)
    data_list = [
      self.loc_msg,
      self.old_loc_msg,
      self.road_msg,
      self.fus_msg,
      self.me_msg,
      self.radar_fm_msg,
      self.radar_fl_msg,
      self.radar_fr_msg,
      self.radar_rl_msg,
      self.radar_rr_msg,
      self.vs_msg,
      self.prediction_msg,
      self.plan_msg,
      self.plan_debug_msg,
      self.ctrl_msg,
      self.ctrl_debug_msg,
      self.fus_parking_msg,
      self.soc_state_msg,
    ]

    topic_list_with_hz = topic_list[:]
    for i in range(len(topic_list)):
      if len(data_list[i]['t']) != 0:
        time_span = max(data_list[i]['abs_t']) - min(data_list[i]['abs_t'])
        hz = int(len(data_list[i]['t']) / time_span) if time_span != 0.0 else 0
        topic_list_with_hz[i] += ' (' + str(hz) + 'hz)'

    data = {'topic_with_hz':[], 't':[], 'msg':[]}
    min_time = sys.maxsize
    for i in range(len(topic_list)):
      for j in range(len(data_list[i]['abs_t'])):
        data['topic_with_hz'].append(topic_list_with_hz[i])
        if topic_list[i] in detail_list:
          data['msg'].append(MessageToJson(data_list[i]['data'][j]))
        else:
          data['msg'].append('')
        t = data_list[i]['abs_t'][j]
        data['t'].append(t)
        if t != 0 and t < min_time:
          min_time = t

    # if header time is 0, don't minus
    for i in range(len(data['t'])):
      if data['t'][i] == 0:
        data['t'][i] = 0
      else:
        data['t'][i] = data['t'][i] - min_time

    source = ColumnDataSource(data=data)
    hover = HoverTool(tooltips=[('topic_with_hz', '@topic_with_hz'), ('t', '@t'), ('msg', '@msg')])
    #args=dict(msg_data=msg_data),
    callback = CustomJS(code="""
        //console.log(cb_obj);
        //console.log(cb_data);

        var data = cb_data.source.data;
        var indices = cb_data.source.selected.indices;
        var selected_msgs='';

        for (let i = 0; i < indices.length; i++) {
            const index = indices[i]
            selected_msgs += data['msg'][index];
        }

        console.log(selected_msgs);
    """)
    taptool = TapTool(callback=callback)

    fig1 = bkp.figure(plot_width=1600, plot_height=400,
              y_range=topic_list_with_hz, x_axis_type='datetime', title=self.bag_path,
              tools=[hover, taptool, "xwheel_zoom,reset"], active_scroll='xwheel_zoom')
    fig1.circle(x='t', y='topic_with_hz', source=source)
    return fig1

def update_local_view_data(fig1, bag_loader, bag_time, local_view_data):
  ### step 1: 时间戳对齐
  loc_msg_idx = 0
  if bag_loader.loc_msg['enable'] == True:
    while bag_loader.loc_msg['t'][loc_msg_idx] <= bag_time and loc_msg_idx < (len(bag_loader.loc_msg['t'])-2):
        loc_msg_idx = loc_msg_idx + 1
  local_view_data['data_index']['loc_msg_idx'] = loc_msg_idx

  road_msg_idx = 0
  if bag_loader.road_msg['enable'] == True:
    while bag_loader.road_msg['t'][road_msg_idx] <= bag_time and road_msg_idx < (len(bag_loader.road_msg['t'])-2):
        road_msg_idx = road_msg_idx + 1
  local_view_data['data_index']['road_msg_idx'] = road_msg_idx

  fus_msg_idx = 0
  if bag_loader.fus_msg['enable'] == True:
    while bag_loader.fus_msg['t'][fus_msg_idx] <= bag_time and fus_msg_idx < (len(bag_loader.fus_msg['t'])-2):
        fus_msg_idx = fus_msg_idx + 1
  local_view_data['data_index']['fus_msg_idx'] = fus_msg_idx

  me_msg_idx = 0
  if bag_loader.me_msg['enable'] == True:
    while bag_loader.me_msg['t'][me_msg_idx] <= bag_time and me_msg_idx < (len(bag_loader.me_msg['t'])-2):
        me_msg_idx = me_msg_idx + 1
        # print("bag_loader.me_msg['t'][me_msg_idx]:",bag_loader.me_msg['t'][me_msg_idx])
    # print("me_msg_idx:",me_msg_idx)
  local_view_data['data_index']['me_msg_idx'] = me_msg_idx

  # radar_msg_idx = dict()
  radar_msg_idx_list = ['radar_fm_msg_idx','radar_fl_msg_idx','radar_fr_msg_idx','radar_rl_msg_idx','radar_rr_msg_idx']
  radar_msg_idx = [0,0,0,0,0]
  bag_loader_radar_msg = [bag_loader.radar_fm_msg, bag_loader.radar_fl_msg, bag_loader.radar_fr_msg,
                          bag_loader.radar_rl_msg, bag_loader.radar_rr_msg]
  # print("bag_loader.radar_fm_msg:",bag_loader.radar_fm_msg['t'][0])
  for i in range(5):
    if bag_loader_radar_msg[i]['enable'] == True:
      while bag_loader_radar_msg[i]['t'][radar_msg_idx[i]] <= bag_time and radar_msg_idx[i] < (len(bag_loader_radar_msg[i]['t'])-2):
          radar_msg_idx[i] = radar_msg_idx[i] + 1
          # print("bag_loader_radar_msg[i]['t'][radar_msg_idx[i]]:",bag_loader_radar_msg[i]['t'][radar_msg_idx[i]])
          # print("bag_time:",bag_time)
          # print("radar_msg_idx[i]:",radar_msg_idx[i])
      # print('radar_msg_idx:',i,radar_msg_idx[i])
    local_view_data['data_index'][radar_msg_idx_list[i]] = radar_msg_idx[i]

  # radar_fm_msg_idx = 0
  # if bag_loader.radar_fm_msg['enable'] == True:
  #   while bag_loader.radar_fm_msg['t'][radar_fm_msg_idx] <= bag_time and radar_fm_msg_idx < (len(bag_loader.radar_fm_msg['t'])-2):
  #       radar_fm_msg_idx = radar_fm_msg_idx + 1
  # local_view_data['data_index']['radar_fm_msg_idx'] = radar_fm_msg_idx

  vs_msg_idx = 0
  if bag_loader.vs_msg['enable'] == True:
    while bag_loader.vs_msg['t'][vs_msg_idx] <= bag_time and vs_msg_idx < (len(bag_loader.vs_msg['t'])-2):
        vs_msg_idx = vs_msg_idx + 1
  local_view_data['data_index']['vs_msg_idx'] = vs_msg_idx

  plan_msg_idx = 0
  if bag_loader.plan_msg['enable'] == True:
    while bag_loader.plan_msg['t'][plan_msg_idx] <= bag_time and plan_msg_idx < (len(bag_loader.plan_msg['t'])-2):
        plan_msg_idx = plan_msg_idx + 1
  local_view_data['data_index']['plan_msg_idx'] = plan_msg_idx

  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
  local_view_data['data_index']['plan_debug_msg_idx'] = plan_debug_msg_idx

  pred_msg_idx = 0
  if bag_loader.prediction_msg['enable'] == True:
    while bag_loader.prediction_msg['t'][pred_msg_idx] <= bag_time and pred_msg_idx < (len(bag_loader.prediction_msg['t'])-2):
        pred_msg_idx = pred_msg_idx + 1
  local_view_data['data_index']['pred_msg_idx'] = pred_msg_idx

  ctrl_msg_idx = 0
  if bag_loader.ctrl_msg['enable'] == True:
    while bag_loader.ctrl_msg['t'][ctrl_msg_idx] <= bag_time and ctrl_msg_idx < (len(bag_loader.ctrl_msg['t'])-2):
        ctrl_msg_idx = ctrl_msg_idx + 1
  local_view_data['data_index']['ctrl_msg_idx'] = ctrl_msg_idx

  ctrl_debug_msg_idx = 0
  if bag_loader.ctrl_debug_msg['enable'] == True:
    while bag_loader.ctrl_debug_msg['t'][ctrl_debug_msg_idx] <= bag_time and ctrl_debug_msg_idx < (len(bag_loader.ctrl_debug_msg['t'])-2):
        ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
  local_view_data['data_index']['ctrl_debug_msg_idx'] = ctrl_debug_msg_idx

  ehr_static_map_msg_idx = 0
  if bag_loader.ehr_static_map_msg['enable'] == True:
    while bag_loader.ehr_static_map_msg['t'][ehr_static_map_msg_idx] <= bag_time and ehr_static_map_msg_idx < (len(bag_loader.ehr_static_map_msg['t'])-2):
        ehr_static_map_msg_idx = ehr_static_map_msg_idx + 1
  local_view_data['data_index']['ehr_static_map_msg_idx'] = ehr_static_map_msg_idx

  ehr_parking_map_msg_idx = 0
  if bag_loader.ehr_parking_map_msg['enable'] == True:
    while bag_loader.ehr_parking_map_msg['t'][ehr_parking_map_msg_idx] <= bag_time and ehr_parking_map_msg_idx < (len(bag_loader.ehr_parking_map_msg['t'])-2):
        ehr_parking_map_msg_idx = ehr_parking_map_msg_idx + 1
  local_view_data['data_index']['ehr_parking_map_msg_idx'] = ehr_parking_map_msg_idx
  
  groundline_msg_idx = 0
  if bag_loader.ground_line_msg['enable'] == True:
    while bag_loader.ground_line_msg['t'][groundline_msg_idx] <= bag_time and groundline_msg_idx < (len(bag_loader.ground_line_msg['t'])-2):
        groundline_msg_idx = groundline_msg_idx + 1
  local_view_data['data_index']['ground_line_msg_idx'] = groundline_msg_idx
  
  planning_hmi_msg_idx = 0
  if bag_loader.planning_hmi_msg['enable'] == True:
    while bag_loader.planning_hmi_msg['t'][planning_hmi_msg_idx] <= bag_time and planning_hmi_msg_idx < (len(bag_loader.planning_hmi_msg['t'])-2):
        planning_hmi_msg_idx = planning_hmi_msg_idx + 1
  local_view_data['data_index']['planning_hmi_msg_idx'] = planning_hmi_msg_idx
  
  ### step 2: 加载定位信息
  cur_pos_xn = 0
  cur_pos_yn = 0
  cur_yaw = 0
  if bag_loader.loc_msg['enable'] == True:
    # ego pos in local and global coordinates
    loc_msg = bag_loader.loc_msg['data'][loc_msg_idx]
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw
    coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)

    ego_xb, ego_yb = [], []
    ego_xn, ego_yn = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.loc_msg['data'])):
      if (i % 10 != 0): # 下采样 10
        continue
      # if bag_loader.loc_msg['data'][i].msf_status.msf_status == 2 :
      #   continue
      pos_xn_i = bag_loader.loc_msg['data'][i].position.position_boot.x
      pos_yn_i = bag_loader.loc_msg['data'][i].position.position_boot.y
      if g_is_display_enu:
        ego_local_x, ego_local_y = pos_xn_i, pos_yn_i
      else:
        ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

      ego_xb.append(ego_local_x)
      ego_yb.append(ego_local_y)
      ego_xn.append(pos_xn_i)
      ego_yn.append(pos_yn_i)

    local_view_data['data_ego'].data.update({
      'ego_xb': ego_xb,
      'ego_yb': ego_yb,
      'ego_xn': ego_xn,
      'ego_yn': ego_yn,
    })

    if g_is_display_enu: 
      # car pos in global coordinates
      car_xn = []
      car_yn = []
      for i in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], cur_pos_xn, cur_pos_yn, cur_yaw)
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)
      local_view_data['data_car'].data.update({
        'car_xb': car_xn,
        'car_yb': car_yn,
      })
      
      local_view_data['data_ego_pos_point'].data.update({
        'ego_pos_point_x': [cur_pos_xn],
        'ego_pos_point_y': [cur_pos_yn],
        'ego_pos_point_theta': [cur_yaw],
      })
    else:
       local_view_data['data_car'].data.update({
        'car_xb': car_xb,
        'car_yb': car_yb,
      })
       local_view_data['data_ego_pos_point'].data.update({
        'ego_pos_point_x': [0],
        'ego_pos_point_y': [0],
        'ego_pos_point_theta': [0],
      })

    try:
      vel_ego = bag_loader.vs_msg['data'][vs_msg_idx].vehicle_speed
    except:
      linear_velocity_from_wheel = math.sqrt(bag_loader.loc_msg['data'][loc_msg_idx].velocity.velocity_boot.vx * bag_loader.loc_msg['data'][loc_msg_idx].velocity.velocity_boot.vx + \
                bag_loader.loc_msg['data'][loc_msg_idx].velocity.velocity_boot.vy * bag_loader.loc_msg['data'][loc_msg_idx].velocity.velocity_boot.vy + \
                bag_loader.loc_msg['data'][loc_msg_idx].velocity.velocity_boot.vz * bag_loader.loc_msg['data'][loc_msg_idx].velocity.velocity_boot.vz)
      vel_ego =  linear_velocity_from_wheel

    

    steer_deg = bag_loader.vs_msg['data'][vs_msg_idx].steering_wheel_angle * 57.3
    if g_is_display_enu:
      local_view_data['data_text'].data.update({
        'vel_ego_text': ['v={:.2f}\nsteer={:.2f}'.format(round(vel_ego, 2), round(steer_deg, 2))],
        'text_xn': [cur_pos_xn],
        'text_yn': [cur_pos_yn],
      })
    else:
      local_view_data['data_text'].data.update({
        'vel_ego_text': ['v={:.2f}\nsteer={:.2f}'.format(round(vel_ego, 2), round(steer_deg, 2))],
        'text_xn': [-2],
        'text_yn': [0],
      })

  # step 3: 加载车道线信息
  if bag_loader.road_msg['enable'] == True:
    # load lane info
    try:
      line_info_list = load_lane_lines(bag_loader.road_msg['data'][road_msg_idx].reference_line_msg)
    except:
      print("old interface before 2.2.3")
      line_info_list = load_lane_lines(bag_loader.road_msg['data'][road_msg_idx].lanes)

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
      10:local_view_data['data_lane_10'],
      11:local_view_data['data_lane_11'],
      12:local_view_data['data_lane_12'],
      13:local_view_data['data_lane_13'],
      14:local_view_data['data_lane_14'],
      15:local_view_data['data_lane_15'],
      16:local_view_data['data_lane_16'],
      17:local_view_data['data_lane_17'],
      18:local_view_data['data_lane_18'],
      19:local_view_data['data_lane_19'],
    }
    data_center_line_dict = {
      0:local_view_data['data_center_line_0'],
      1:local_view_data['data_center_line_1'],
      2:local_view_data['data_center_line_2'],
      3:local_view_data['data_center_line_3'],
      4:local_view_data['data_center_line_4'],
      5:local_view_data['data_center_line_5'],
      6:local_view_data['data_center_line_6'],
      7:local_view_data['data_center_line_7'],
      8:local_view_data['data_center_line_8'],
      9:local_view_data['data_center_line_9'],
    }

    for i in range(20):
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
        print('error')
        pass

    find_enu = False
    center_line_list = load_lane_center_lines(bag_loader.road_msg['data'][road_msg_idx].reference_line_msg, find_enu, loc_msg, g_is_display_enu)
    # print(center_line_list)
    trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory

    for i in range(10):
      # try:
        # if (trajectory.trajectory_type == 0) or (trajectory.trajectory_type == 1 and trajectory.target_reference.lateral_maneuver_gear == 2) :
        data_center_line = data_center_line_dict[i]
        data_center_line.data.update({
          'center_line_{}_x'.format(i): center_line_list[i]['line_x_vec'],
          'center_line_{}_y'.format(i): center_line_list[i]['line_y_vec'],
        })
        # else:
        #   data_center_line = data_center_line_dict[i]
        #   line_x_rel = []
        #   line_y_rel = []
        #   for index in range(len(center_line_list[i]['line_x_vec'])):
        #     pos_xn_i, pos_yn_i = center_line_list[i]['line_x_vec'][index], center_line_list[i]['line_y_vec'][index]
        #     ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)
        #     line_x_rel.append(ego_local_x)
        #     line_y_rel.append(ego_local_y)
        #   center_line_list[i]['line_x_vec'] = line_x_rel
        #   center_line_list[i]['line_y_vec'] = line_y_rel
        #   if center_line_list[i]['relative_id'] == 0:
        #     fig1.renderers[13 + i].glyph.line_dash = 'dotdash'
        #     fig1.renderers[13 + i].glyph.line_alpha = 1
        #     fig1.renderers[13 + i].glyph.line_width = 2
        #   else:
        #     fig1.renderers[13 + i].glyph.line_dash = 'dotted'
        #     fig1.renderers[13 + i].glyph.line_alpha = 0.8
        #     fig1.renderers[13 + i].glyph.line_width = 1

          # if center_line_list[i]['relative_id'] == 1000:  # 车道不存在
          #   center_line_list[i]['line_x_vec'] = []
          #   center_line_list[i]['line_y_vec'] = []
          # data_center_line.data.update({
          #   'center_line_{}_x'.format(i): center_line_list[i]['line_x_vec'],
          #   'center_line_{}_y'.format(i): center_line_list[i]['line_y_vec'],
          # })
      # except:
      #   print('error')
      #   pass

  # fix_lane,origin_lane
  if bag_loader.plan_debug_msg['enable'] == True:
    try:
      print("planning debug info:", bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].frame_info)
    except:
      pass
    lat_behavior_common = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lat_behavior_common
    environment_model_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].environment_model_info
    current_lane_virtual_id = environment_model_info.currrent_lane_vitual_id
    fix_lane_ralative_id = lat_behavior_common.fix_lane_virtual_id - current_lane_virtual_id
    target_lane_ralative_id = lat_behavior_common.target_lane_virtual_id - current_lane_virtual_id
    origin_lane_ralative_id = lat_behavior_common.origin_lane_virtual_id - current_lane_virtual_id
    for i in range(10):
      if center_line_list[i]['relative_id'] == fix_lane_ralative_id:
        local_view_data['data_fix_lane'].data.update({
          'fix_lane_x': center_line_list[i]['line_x_vec'],
          'fix_lane_y':center_line_list[i]['line_y_vec']
        })
      if center_line_list[i]['relative_id'] == target_lane_ralative_id:
        local_view_data['data_target_lane'].data.update({
          'target_lane_x': center_line_list[i]['line_x_vec'],
          'target_lane_y':center_line_list[i]['line_y_vec']
        })
      if center_line_list[i]['relative_id'] == origin_lane_ralative_id:
        local_view_data['data_origin_lane'].data.update({
          'origin_lane_x': center_line_list[i]['line_x_vec'],
          'origin_lane_y':center_line_list[i]['line_y_vec']
        })
    
    plan_debug_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]
    plan_traj_x = plan_debug_json["assembled_x"]
    plan_traj_y = plan_debug_json["assembled_y"]
    plan_traj_theta = plan_debug_json["assembled_theta"]
    plan_traj_s = plan_debug_json["traj_s_vec"]
    if min(plan_traj_x) != max(plan_traj_x) or min(plan_traj_y) != max(plan_traj_y):
      if not g_is_display_enu:
        plan_traj_x, plan_traj_y = coord_tf.global_to_local(plan_traj_x, plan_traj_y)
        cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw
        plan_traj_theta_local = []
        for i in range(len(plan_traj_theta)):
          plan_traj_theta_local.append(plan_traj_theta[i] - cur_yaw)
        plan_traj_theta = plan_traj_theta_local
          
      local_view_data['data_planning_raw'].data.update({
          'plan_traj_y' : plan_traj_y,
          'plan_traj_x' : plan_traj_x,
          'plan_traj_s': plan_traj_s
      })

      car_xb_traj = []
      car_yb_traj = []
      for i in range(len(plan_traj_x) - 1, -1, -1):
        car_xb_traj_point = []
        car_yb_traj_point = []
        for j in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], plan_traj_x[i], plan_traj_y[i], plan_traj_theta[i])
          car_xb_traj_point.append(tmp_x)
          car_yb_traj_point.append(tmp_y)
        car_xb_traj.append(car_xb_traj_point)
        car_yb_traj.append(car_yb_traj_point)
      local_view_data['data_car_traj_raw'].data.update({
        'car_yb_traj' : car_yb_traj,
        'car_xb_traj' : car_xb_traj,
      })
    else:
      print("no car_traj_raw json debug info!")

    lat_init_state = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.init_state
    init_state_x = lat_init_state.x
    init_state_y = lat_init_state.y
    init_state_theta = lat_init_state.theta
    init_state_delta = lat_init_state.delta
    lon_init_state = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].longitudinal_motion_planning_input.init_state
    init_state_s = lon_init_state.s
    init_state_v = lon_init_state.v
    init_state_a = lon_init_state.a
    replan_status = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]["replan_status"]
    init_pos_point_x = []
    init_pos_point_y = []
    init_pos_point_theta = []
    if g_is_display_enu:
      init_pos_point_x.append(init_state_x)
      init_pos_point_y.append(init_state_y)
      init_pos_point_theta.append(init_state_theta)
    else:
      init_pos_point_x, init_pos_point_y = coord_tf.global_to_local([init_state_x], [init_state_y])
      temp_theta = init_state_theta - bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw
      init_pos_point_theta.append(temp_theta)
      
    local_view_data['data_init_pos_point'].data.update({
      'init_pos_point_y': init_pos_point_y,
      'init_pos_point_x': init_pos_point_x,
      'init_pos_point_theta': init_pos_point_theta,
      'init_state_x': [init_state_x], 
      'init_state_y': [init_state_y],
      'init_state_theta': [init_state_theta], 
      'init_state_delta': [init_state_delta],
      'init_state_s': [init_state_s],
      'init_state_v': [init_state_v], 
      'init_state_a': [init_state_a],
      'replan_status': [replan_status],
    })

    # local_view_data['data_text'].data.update({
    #   'vel_ego_text': ['v={:.2f}({:d})\nsteer={:.2}'.format(round(vel_ego, 2),current_lane_virtual_id, round(steer_deg, 2))],
    #   'text_xn': [text_xn],
    #   'text_yn': [text_yn],
    # })
  ### step 4: 加载障碍物信息
  # load fus_obj
  if bag_loader.fus_msg['enable'] == True:
    fusion_objects = bag_loader.fus_msg['data'][fus_msg_idx].fusion_object
    obstacles_info_all = load_obstacle_params(fusion_objects)
    local_view_data['data_fus_obj'].data.update({
            'obstacles_x': [],
            'obstacles_y': [],
            'pos_x' : [],
            'pos_y' : [],
            'obs_label' : [],
          })
    local_view_data['data_snrd_obj'].data.update({
            'obstacles_x': [],
            'obstacles_y': [],
            'pos_x' : [],
            'pos_y' : [],
            'obs_label' : [],
          })
    if 1:
      for key in obstacles_info_all:
        obstacles_info = obstacles_info_all[key]
        if key == 1:
          key_name = 'data_fus_obj'
        else:
          key_name = 'data_snrd_obj'
        if g_is_display_enu:
          local_view_data[key_name].data.update({
            'obstacles_x': obstacles_info['obstacles_x'],
            'obstacles_y': obstacles_info['obstacles_y'],
            'pos_x' : obstacles_info['pos_x'],
            'pos_y' : obstacles_info['pos_y'],
            'obs_label' : obstacles_info['obs_label'],
          })
        else:
          local_view_data[key_name].data.update({
            'obstacles_x': obstacles_info['obstacles_x_rel'],
            'obstacles_y': obstacles_info['obstacles_y_rel'],
            'pos_x' : obstacles_info['pos_x_rel'],
            'pos_y' : obstacles_info['pos_y_rel'],
            'obs_label' : obstacles_info['obs_label'],
          })

  # load me_obj
  if bag_loader.me_msg['enable'] == True:
    # print("me_msg_idx:",me_msg_idx)
    me_camera_objects = bag_loader.me_msg['data'][me_msg_idx].camera_perception_object_list
    # print(me_camera_objects.size())
    obstacles_info_all1 = load_obstacle_me(me_camera_objects)
    # 加载自车坐标系下的数据
    if 1:
      for key in obstacles_info_all1:
        obstacles_info = obstacles_info_all1[key]
        if key == 1:
          key_name = 'data_me_obj'
        else:
          key_name = 'data_snrd_obj'
        if g_is_display_enu:
          local_view_data[key_name].data.update({
            'obstacles_x': obstacles_info['obstacles_x'],
            'obstacles_y': obstacles_info['obstacles_y'],
            'pos_x' : obstacles_info['pos_x'],
            'pos_y' : obstacles_info['pos_y'],
            'obs_label' : obstacles_info['obs_label'],
          })
        else:
          local_view_data[key_name].data.update({
            'obstacles_x': obstacles_info['obstacles_x_rel'],
            'obstacles_y': obstacles_info['obstacles_y_rel'],
            'pos_x' : obstacles_info['pos_x_rel'],
            'pos_y' : obstacles_info['pos_y_rel'],
            'obs_label' : obstacles_info['obs_label'],
          })
  # load radar_obj
  data_radar_obj = ['data_radar_fm_obj','data_radar_fl_obj','data_radar_fr_obj','data_radar_rl_obj','data_radar_rr_obj']
  for i in range(5):
    # print(data_radar_obj[i])
    if bag_loader_radar_msg[i]['enable'] == True:
      # print(radar_msg_idx[i])
      radar_objects = bag_loader_radar_msg[i]['data'][radar_msg_idx[i]].radar_perception_object_list
      # print(me_camera_objects.length())
      obstacles_info_all2 = load_obstacle_radar(radar_objects,i)
      
      for key in obstacles_info_all2:
        obstacles_info = obstacles_info_all2[key]
        if key == 11 or key == 12 or key == 13 or key == 14 or key == 15 :
          key_name = data_radar_obj[i]
        else:
          continue
        if g_is_display_enu:
          local_view_data[key_name].data.update({
            'obstacles_x': [],
            'obstacles_y': [],
            'pos_x' : [],
            'pos_y' : [],
            'obs_label' : [],
          })
        else:
          local_view_data[key_name].data.update({
            'obstacles_x': obstacles_info['obstacles_x_rel'],
            'obstacles_y': obstacles_info['obstacles_y_rel'],
            'pos_x' : obstacles_info['pos_x_rel'],
            'pos_y' : obstacles_info['pos_y_rel'],
            'obs_label' : obstacles_info['obs_label'],
          })

  #  加载fix_lane, target_lane信息
  ### step 3: 加载planning轨迹信息
  if bag_loader.plan_msg['enable'] == True:
    trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
    plan_traj_s = []
    for i in range(len(trajectory.trajectory_points)):
      plan_traj_s.append(trajectory.trajectory_points[i].distance)
    if trajectory.trajectory_type == 0: # 实时轨迹
      try:
        planning_polynomial = trajectory.target_reference.polynomial
        plan_traj_x, plan_traj_y = gen_line(planning_polynomial[3],planning_polynomial[2], planning_polynomial[1], planning_polynomial[0], 0, 50)
      except:
        plan_traj_x, plan_traj_y = [], []
    else:
      plan_traj_x, plan_traj_y, plan_traj_theta, plan_dict = generate_planning_trajectory(trajectory, bag_loader.loc_msg['data'][loc_msg_idx], g_is_display_enu)
      for i in range(5):
        local_view_data['data_planning_' + str(i)].data.update({
          'plan_traj_y' : plan_dict[i]['y'],
          'plan_traj_x' : plan_dict[i]['x'],
      })
        
    local_view_data['data_planning'].data.update({
        'plan_traj_y' : plan_traj_y,
        'plan_traj_x' : plan_traj_x,
        'plan_traj_s' : plan_traj_s,
    })

    car_xb_traj = []
    car_yb_traj = []
    for i in range(len(plan_traj_x) - 1, -1, -1):
      car_xb_traj_point = []
      car_yb_traj_point = []
      for j in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], plan_traj_x[i], plan_traj_y[i], plan_traj_theta[i])
        car_xb_traj_point.append(tmp_x)
        car_yb_traj_point.append(tmp_y)
      car_xb_traj.append(car_xb_traj_point)
      car_yb_traj.append(car_yb_traj_point)
    local_view_data['data_car_traj'].data.update({
      'car_yb_traj' : car_yb_traj,
      'car_xb_traj' : car_xb_traj,
    })
      
  # 加载prediction_msg
  if bag_loader.prediction_msg['enable'] == True:
    try:
      for i in range(5):
        local_view_data['data_prediction_' + str(i)].data.update({
          'prediction_y' : [],
          'prediction_x' : [],
        })
      prediction_info = bag_loader.prediction_msg['data'][pred_msg_idx]
      # 定位的选择需要修改
      prediction_dict = load_prediction_objects(prediction_info.prediction_obstacle_list, loc_msg, g_is_display_enu)
      for i in range(5):
        local_view_data['data_prediction_' + str(i)].data.update({
          'prediction_y' : prediction_dict[i]['y'],
          'prediction_x' : prediction_dict[i]['x'],
        })
    except:
      print("prediction error")
      pass

  # load control
  if bag_loader.ctrl_msg['enable'] == True:
    mpc_dx, mpc_dy, mpc_dtheta = generate_control(bag_loader.ctrl_msg['data'][ctrl_msg_idx], loc_msg, g_is_display_enu)
    local_view_data['data_control'].data.update({
        'mpc_dx' : mpc_dx,
        'mpc_dy' : mpc_dy,
    })

    car_xb_traj = []
    car_yb_traj = []
    for i in range(len(mpc_dx) - 1, -1, -1):
      car_xb_traj_point = []
      car_yb_traj_point = []
      for j in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], mpc_dx[i], mpc_dy[i], mpc_dtheta[i])
        car_xb_traj_point.append(tmp_x)
        car_yb_traj_point.append(tmp_y)
      car_xb_traj.append(car_xb_traj_point)
      car_yb_traj.append(car_yb_traj_point)
    local_view_data['data_car_traj_mpc'].data.update({
      'car_yb_traj' : car_yb_traj,
      'car_xb_traj' : car_xb_traj,
    })

  # # 加载ehr的lane信息
  if is_vis_map and bag_loader.ehr_static_map_msg['enable'] == True :
    #load ehr static map info
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw

    if bag_loader.planning_hmi_msg['enable'] ==True:
      noa_output_info_msg = bag_loader.planning_hmi_msg['data'][planning_hmi_msg_idx].noa_output_info
      print("dis to ramp:",noa_output_info_msg.dis_to_ramp)
      print("dis to split:",noa_output_info_msg.dis_to_split)
      print("dis to merge:",noa_output_info_msg.dis_to_merge)

    print("ehr static map timestamp:",bag_loader.ehr_static_map_msg['data'][ehr_static_map_msg_idx].header)
    print("road_map.lanes len:",len(bag_loader.ehr_static_map_msg['data'][ehr_static_map_msg_idx].road_map.lanes))
    #load center line

    ehr_line_info_list = ehr_load_center_lane_lines(bag_loader.ehr_static_map_msg['data'][ehr_static_map_msg_idx].road_map.lanes,
                                             cur_pos_xn,cur_pos_yn,cur_yaw,Max_line_size)
    ehr_data_lane_dict = {}
    for i in range(Max_line_size):
      ehr_data_lane_dict[i] = local_view_data['ehr_data_lane_{}'.format(i)]
    for i in range(len(ehr_line_info_list)):
      if ehr_line_info_list[i]['ehr_relative_id'] == 1000: #车道不存在
        ehr_line_info_list[i]['ehr_line_x_vec'] = []
        ehr_line_info_list[i]['ehr_line_y_vec'] = []
      ehr_data_line = ehr_data_lane_dict[i]
      # print("ehr_line_info_list:",len(ehr_line_info_list))
      ehr_data_line.data.update({
            'ehr_line_{}_x'.format(i): ehr_line_info_list[i]['ehr_line_x_vec'],
            'ehr_line_{}_y'.format(i): ehr_line_info_list[i]['ehr_line_y_vec'],
          })

    #load road boundary
    print("road_map.road_boundaries len:",len(bag_loader.ehr_static_map_msg['data'][ehr_static_map_msg_idx].road_map.road_boundaries))
    ehr_load_road_boundary_info_list = ehr_load_road_boundary_lines(bag_loader.ehr_static_map_msg['data'][ehr_static_map_msg_idx].road_map.road_boundaries,
                                             cur_pos_xn,cur_pos_yn,cur_yaw,Road_boundary_max_line_size)
    ehr_data_road_boundary_dict = {}
    for i in range(Road_boundary_max_line_size):
      ehr_data_road_boundary_dict[i] = local_view_data['ehr_road_boundary_{}'.format(i)]
    for i in range(len(ehr_load_road_boundary_info_list)):
      ehr_data_road_boundary = ehr_data_road_boundary_dict[i]
      # print("ehr_line_info_list:",len(ehr_line_info_list))
      ehr_data_road_boundary.data.update({
            'ehr_road_boundary_{}_x'.format(i): ehr_load_road_boundary_info_list[i]['ehr_road_boundary_x_vec'],
            'ehr_road_boundary_{}_y'.format(i): ehr_load_road_boundary_info_list[i]['ehr_road_boundary_y_vec'],
          })
    #load lane boundary
    print("road_map.lane_boundaries len:",len(bag_loader.ehr_static_map_msg['data'][ehr_static_map_msg_idx].road_map.lane_boundaries))
    ehr_lane_boundary_info_list = ehr_load_lane_boundary_lines(bag_loader.ehr_static_map_msg['data'][ehr_static_map_msg_idx].road_map.lane_boundaries,
                                             cur_pos_xn,cur_pos_yn,cur_yaw,Lane_boundary_max_line_size)
    ehr_data_lane_boundary_dict = {}
    for i in range(Lane_boundary_max_line_size):
      ehr_data_lane_boundary_dict[i] = local_view_data['ehr_lane_boundary_{}'.format(i)]
    for i in range(len(ehr_lane_boundary_info_list)):
      ehr_data_lane_boundary = ehr_data_lane_boundary_dict[i]
      ehr_data_lane_boundary.data.update({
            'ehr_lane_boundary_{}_x'.format(i): ehr_lane_boundary_info_list[i]['ehr_lane_boundary_x_vec'],
            'ehr_lane_boundary_{}_y'.format(i): ehr_lane_boundary_info_list[i]['ehr_lane_boundary_y_vec'],
          })

  # 加载ehr_parking_map
  if bag_loader.ehr_parking_map_msg['enable'] == True:
    ehr_parking_map_msg = bag_loader.ehr_parking_map_msg['data'][ehr_parking_map_msg_idx]
    parking_space_boxes_x, parking_space_boxes_y, road_mark_boxes_x, road_mark_boxes_y = generate_ehr_parking_map(ehr_parking_map_msg, loc_msg, g_is_display_enu)
    local_view_data['data_parking_space'].data.update({
      'parking_space_x' : parking_space_boxes_x,
      'parking_space_y' : parking_space_boxes_y,
    })
    local_view_data['data_road_mark'].data.update({
      'road_mark_x' : road_mark_boxes_x,
      'road_mark_y' : road_mark_boxes_y,
    })
    
  if bag_loader.ground_line_msg['enable'] == True:
    print("groundline_msg_idx", groundline_msg_idx)
    ground_line_msg = bag_loader.ground_line_msg['data'][groundline_msg_idx]
    groundline_x_vec, groundline_y_vec = generate_ground_line(ground_line_msg, loc_msg, g_is_display_enu)
    local_view_data['data_ground_line'].data.update({
      'ground_line_x' : groundline_x_vec,
      'ground_line_y' : groundline_y_vec,
    })
  return local_view_data

def load_local_view_figure():
  data_car = ColumnDataSource(data = {'car_yb':[], 'car_xb':[]})
  data_car_traj = ColumnDataSource(data = {'car_yb_traj':[], 'car_xb_traj':[]})
  data_car_traj_raw = ColumnDataSource(data = {'car_yb_traj':[], 'car_xb_traj':[]})
  data_car_traj_mpc = ColumnDataSource(data = {'car_yb_traj':[], 'car_xb_traj':[]})
  data_ego = ColumnDataSource(data = {'ego_yb':[], 'ego_xb':[]})
  data_ego_pos_point = ColumnDataSource(data = {'ego_pos_point_y':[],
                                                'ego_pos_point_x':[],
                                                'ego_pos_point_theta':[]})
  data_init_pos_point = ColumnDataSource(data = {'init_pos_point_y':[],
                                                 'init_pos_point_x':[],
                                                 'init_pos_point_theta':[],
                                                 'init_state_x':[], 
                                                 'init_state_y':[],
                                                 'init_state_theta':[], 
                                                 'init_state_delta':[],
                                                 'init_state_s':[],
                                                 'init_state_v':[], 
                                                 'init_state_a':[],
                                                 'replan_status':[]})
  data_text = ColumnDataSource(data = {'vel_ego_text':[], 'text_xn': [],  'text_yn': []})
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
  data_lane_10 = ColumnDataSource(data = {'line_10_y':[], 'line_10_x':[]})
  data_lane_11 = ColumnDataSource(data = {'line_11_y':[], 'line_11_x':[]})
  data_lane_12 = ColumnDataSource(data = {'line_12_y':[], 'line_12_x':[]})
  data_lane_13 = ColumnDataSource(data = {'line_13_y':[], 'line_13_x':[]})
  data_lane_14 = ColumnDataSource(data = {'line_14_y':[], 'line_14_x':[]})
  data_lane_15 = ColumnDataSource(data = {'line_15_y':[], 'line_15_x':[]})
  data_lane_16 = ColumnDataSource(data = {'line_16_y':[], 'line_16_x':[]})
  data_lane_17 = ColumnDataSource(data = {'line_17_y':[], 'line_17_x':[]})
  data_lane_18 = ColumnDataSource(data = {'line_18_y':[], 'line_18_x':[]})
  data_lane_19 = ColumnDataSource(data = {'line_19_y':[], 'line_19_x':[]})

  if is_vis_map:
    ehr_data_lanes = []
    for i in range(Max_line_size):
      ehr_data_lanes.append(ColumnDataSource(data={'ehr_line_{}_y'.format(i): [], 'ehr_line_{}_x'.format(i): []}))

    ehr_road_boundary_lanes = []
    for i in range(Road_boundary_max_line_size):
      ehr_road_boundary_lanes.append(ColumnDataSource(data={'ehr_road_boundary_{}_y'.format(i): [], 'ehr_road_boundary_{}_x'.format(i): []}))

    ehr_lane_boundary_lanes = []
    for i in range(Lane_boundary_max_line_size):
      ehr_lane_boundary_lanes.append(ColumnDataSource(data={'ehr_lane_boundary_{}_y'.format(i): [], 'ehr_lane_boundary_{}_x'.format(i): []}))

  data_center_line_0 = ColumnDataSource(data = {'center_line_0_y':[], 'center_line_0_x':[]})
  data_center_line_1 = ColumnDataSource(data = {'center_line_1_y':[], 'center_line_1_x':[]})
  data_center_line_2 = ColumnDataSource(data = {'center_line_2_y':[], 'center_line_2_x':[]})
  data_center_line_3 = ColumnDataSource(data = {'center_line_3_y':[], 'center_line_3_x':[]})
  data_center_line_4 = ColumnDataSource(data = {'center_line_4_y':[], 'center_line_4_x':[]})
  data_center_line_5 = ColumnDataSource(data = {'center_line_5_y':[], 'center_line_5_x':[]})
  data_center_line_6 = ColumnDataSource(data = {'center_line_6_y':[], 'center_line_6_x':[]})
  data_center_line_7 = ColumnDataSource(data = {'center_line_7_y':[], 'center_line_7_x':[]})
  data_center_line_8 = ColumnDataSource(data = {'center_line_8_y':[], 'center_line_8_x':[]})
  data_center_line_9 = ColumnDataSource(data = {'center_line_9_y':[], 'center_line_9_x':[]})
  data_fix_lane = ColumnDataSource(data = {'fix_lane_y':[], 'fix_lane_x':[]})
  data_target_lane = ColumnDataSource(data = {'target_lane_y':[], 'target_lane_x':[]})
  data_origin_lane = ColumnDataSource(data = {'origin_lane_y':[], 'origin_lane_x':[]})
  data_fus_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_me_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_radar_fm_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_radar_fl_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_radar_fr_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_radar_rl_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_radar_rr_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_snrd_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
  data_planning_raw = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],
                                      'plan_traj_s':[],})
  data_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],
                                      'plan_traj_s':[],})
  data_planning_0 = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_planning_1 = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_planning_2 = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_planning_3 = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_planning_4 = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_control = ColumnDataSource(data = {'mpc_dx':[],
                                          'mpc_dy':[],})
  data_prediction_0 = ColumnDataSource(data = {'prediction_y':[],
                                             'prediction_x':[],})
  data_prediction_1 = ColumnDataSource(data = {'prediction_y':[],
                                             'prediction_x':[],})
  data_prediction_2 = ColumnDataSource(data = {'prediction_y':[],
                                             'prediction_x':[],})
  data_prediction_3 = ColumnDataSource(data = {'prediction_y':[],
                                             'prediction_x':[],})
  data_prediction_4 = ColumnDataSource(data = {'prediction_y':[],
                                             'prediction_x':[],})
  data_parking_space = ColumnDataSource(data = {'parking_space_y':[],
                                             'parking_space_x':[],})
  data_road_mark = ColumnDataSource(data = {'road_mark_y':[],
                                             'road_mark_x':[],})
  data_ground_line = ColumnDataSource(data = {'ground_line_y':[],
                                             'ground_line_x':[],})
  data_index = {'loc_msg_idx': 0,
                'road_msg_idx': 0,
                'fus_msg_idx': 0,
                'me_msg_idx':0,
                'radar_fm_msg_idx':0,
                'radar_fl_msg_idx':0,
                'radar_fr_msg_idx':0,
                'radar_rl_msg_idx':0,
                'radar_rr_msg_idx':0,
                'vs_msg_idx': 0,
                'plan_msg_idx': 0,
                'plan_debug_msg_idx': 0,
                'pred_msg_idx': 0,
                'ctrl_msg_idx': 0,
                'ctrl_debug_msg_idx': 0,
                'ehr_static_map_msg_idx': 0,
                'planning_hmi_msg_idx': 0,
               }

  local_view_data = {'data_car':data_car, \
                     'data_car_traj':data_car_traj, \
                     'data_car_traj_raw':data_car_traj_raw, \
                     'data_car_traj_mpc':data_car_traj_mpc, \
                     'data_ego':data_ego, \
                     'data_ego_pos_point': data_ego_pos_point, \
                     'data_init_pos_point': data_init_pos_point, \
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
                     'data_lane_10':data_lane_10, \
                     'data_lane_11':data_lane_11, \
                     'data_lane_12':data_lane_12, \
                     'data_lane_13':data_lane_13, \
                     'data_lane_14':data_lane_14, \
                     'data_lane_15':data_lane_15, \
                     'data_lane_16':data_lane_16, \
                     'data_lane_17':data_lane_17, \
                     'data_lane_18':data_lane_18, \
                     'data_lane_19':data_lane_19, \
                     'data_center_line_0':data_center_line_0, \
                     'data_center_line_1':data_center_line_1, \
                     'data_center_line_2':data_center_line_2, \
                     'data_center_line_3':data_center_line_3, \
                     'data_center_line_4':data_center_line_4, \
                     'data_center_line_5':data_center_line_5, \
                     'data_center_line_6':data_center_line_6, \
                     'data_center_line_7':data_center_line_7, \
                     'data_center_line_8':data_center_line_8, \
                     'data_center_line_9':data_center_line_9, \
                     'data_fix_lane': data_fix_lane ,\
                     'data_target_lane': data_target_lane ,\
                     'data_origin_lane': data_origin_lane ,\
                     'data_prediction_0' : data_prediction_0 ,\
                     'data_prediction_1' : data_prediction_1 ,\
                     'data_prediction_2' : data_prediction_2 ,\
                     'data_prediction_3' : data_prediction_3 ,\
                     'data_prediction_4' : data_prediction_4 ,\
                     'data_parking_space' : data_parking_space , \
                     'data_road_mark' : data_road_mark , \
                     'data_ground_line' : data_ground_line, \
                     'data_fus_obj':data_fus_obj, \
                     'data_me_obj':data_me_obj, \
                     'data_radar_fm_obj':data_radar_fm_obj, \
                     'data_radar_fl_obj':data_radar_fl_obj, \
                     'data_radar_fr_obj':data_radar_fr_obj, \
                     'data_radar_rl_obj':data_radar_rl_obj, \
                     'data_radar_rr_obj':data_radar_rr_obj, \
                     'data_snrd_obj':data_snrd_obj, \
                     'data_planning_raw':data_planning_raw,\
                     'data_planning':data_planning,\
                     'data_planning_0':data_planning_0,\
                     'data_planning_1':data_planning_1,\
                     'data_planning_2':data_planning_2,\
                     'data_planning_3':data_planning_3,\
                     'data_planning_4':data_planning_4,\
                     'data_control':data_control,\
                     'data_index': data_index, \
                     }
  if is_vis_map:
    for i in range(len(ehr_data_lanes)):
      key = 'ehr_data_lane_' + str(i)
      value = ehr_data_lanes[i]
      local_view_data[key] = value

    for i in range(len(ehr_road_boundary_lanes)):
      key = 'ehr_road_boundary_' + str(i)
      value = ehr_road_boundary_lanes[i]
      local_view_data[key] = value

    for i in range (len(ehr_lane_boundary_lanes)):
      key = 'ehr_lane_boundary_' + str(i)
      value = ehr_lane_boundary_lanes[i]
      local_view_data[key] = value

  ### figures config
  fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=1000, height=1100, match_aspect = True, aspect_scale=1)

  fig1.x_range.flipped = True
  # figure plot
  f1 = fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj, fill_color = "palegreen", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj')
  fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj_raw, fill_color = "deepskyblue", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj_raw')
  fig1.patches('car_yb_traj', 'car_xb_traj', source = data_car_traj_mpc, fill_color = "salmon", fill_alpha = 0.05, line_color = "black", line_alpha = 0.3, line_width = 1, legend_label = 'car_traj_mpc')
  fig1.patch('car_yb', 'car_xb', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
  fig1.circle('init_pos_point_y', 'init_pos_point_x', source = data_init_pos_point, radius = 0.1, line_width = 2,  line_color = 'black', line_alpha = 1, fill_color = "deepskyblue", fill_alpha = 1, legend_label = 'init_state')
  fig1.circle('ego_pos_point_y', 'ego_pos_point_x', source = data_ego_pos_point, radius = 0.1, line_width = 2,  line_color = 'purple', line_alpha = 1, fill_alpha = 1, legend_label = 'ego_pos_point')
  fig1.line('ego_yb', 'ego_xb', source = data_ego, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
  fig1.text('text_yn', 'text_xn', text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt", legend_label = 'car')
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
  fig1.line('line_10_y', 'line_10_x', source = data_lane_10, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_11_y', 'line_11_x', source = data_lane_11, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_12_y', 'line_12_x', source = data_lane_12, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_13_y', 'line_13_x', source = data_lane_13, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_14_y', 'line_14_x', source = data_lane_14, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_15_y', 'line_15_x', source = data_lane_15, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_16_y', 'line_16_x', source = data_lane_16, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_17_y', 'line_17_x', source = data_lane_17, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_18_y', 'line_18_x', source = data_lane_18, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
  fig1.line('line_19_y', 'line_19_x', source = data_lane_19, line_width = 1.5, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')

  if is_vis_map:
    for i in range (len(ehr_data_lanes)):
      keyy = 'ehr_line_{}_y'.format(i)
      keyx = 'ehr_line_{}_x'.format(i)
      fig1.line(keyy,keyx,source = ehr_data_lanes[i], line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'ehr_center_lane')

    for i in range(len(ehr_road_boundary_lanes)):
      keyy = 'ehr_road_boundary_{}_y'.format(i)
      keyx = 'ehr_road_boundary_{}_x'.format(i)
      fig1.line(keyy,keyx,source = ehr_road_boundary_lanes[i], line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'ehr_road_boundary')

    for i in range(len(ehr_lane_boundary_lanes)):
      keyy = 'ehr_lane_boundary_{}_y'.format(i)
      keyx = 'ehr_lane_boundary_{}_x'.format(i)
      fig1.line(keyy,keyx,source = ehr_lane_boundary_lanes[i], line_width = 1, line_color = 'blue', line_dash = 'dashed', legend_label = 'ehr_lane_boundary')


  fig1.line('center_line_0_y', 'center_line_0_x', source = data_center_line_0, line_width = 2, line_color = 'blue', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line')
  fig1.line('center_line_1_y', 'center_line_1_x', source = data_center_line_1, line_width = 2, line_color = 'blue', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line')
  fig1.line('center_line_2_y', 'center_line_2_x', source = data_center_line_2, line_width = 2, line_color = 'blue', line_dash = 'dotted', line_alpha = 1, legend_label = 'center_line')
  # fig1.line('center_line_3_y', 'center_line_3_x', source = data_center_line_3, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  # fig1.line('center_line_4_y', 'center_line_4_x', source = data_center_line_4, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig1.line('fix_lane_y', 'fix_lane_x', source = data_fix_lane, line_width = 1, line_color = 'red', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'fix_lane')
  fig1.line('target_lane_y', 'target_lane_x', source = data_target_lane, line_width = 1, line_color = 'orange', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'taget_lane')
  fig1.line('origin_lane_y', 'origin_lane_x', source = data_origin_lane, line_width = 1, line_color = 'black', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'origin_lane')
  fig1.patches('obstacles_y', 'obstacles_x', source = data_fus_obj, fill_color = "gray", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'obj')
  #if 0:
  fig1.patches('obstacles_y', 'obstacles_x', source = data_me_obj, fill_color = "orange", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'me_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_fm_obj, fill_color = "green", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_fm_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_fl_obj, fill_color = "red", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_fl_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_fr_obj, fill_color = "blue", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_fr_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_rl_obj, fill_color = "yellow", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_rl_obj',visible = False)
  fig1.patches('obstacles_y', 'obstacles_x', source = data_radar_rr_obj, fill_color = "black", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'radar_rr_obj',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_me_obj, text_color="orange", text_align="center", text_font_size="8pt", legend_label = 'me_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_fm_obj, text_color="palegreen", text_align="center", text_font_size="8pt", legend_label = 'radar_fm_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_fl_obj, text_color="red", text_align="center", text_font_size="8pt", legend_label = 'radar_fl_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_fr_obj, text_color="blue", text_align="center", text_font_size="8pt", legend_label = 'radar_fr_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_rl_obj, text_color="yellow", text_align="center", text_font_size="8pt", legend_label = 'radar_rl_info',visible = False)
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_radar_rr_obj, text_color="black", text_align="center", text_font_size="8pt", legend_label = 'radar_rr_info',visible = False)

  fig1.patches('obstacles_y', 'obstacles_x', source = data_snrd_obj, fill_color = "black", line_color = "black", line_width = 1, fill_alpha = 0.5, legend_label = 'snrd')
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_fus_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'fusion_info')
  fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_snrd_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'snrd_info')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning_raw, line_width = 5, line_color = 'deepskyblue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'raw plan')
  fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_0, radius = 0.03, line_width = 1,  line_color = 'red', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_1, radius = 0.03, line_width = 1,  line_color = 'blue', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_2, radius = 0.03, line_width = 1,  line_color = 'orange', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_3, radius = 0.03, line_width = 1,  line_color = 'black', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.circle('plan_traj_y', 'plan_traj_x', source = data_planning_4, radius = 0.03, line_width = 1,  line_color = 'purple', line_alpha = 1, fill_alpha = 0, legend_label = 'plan_point')
  fig1.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 5, line_color = 'green', line_dash = 'dashed', line_alpha = 0.8, legend_label = 'ctrl_traj')
  fig1.circle('prediction_y', 'prediction_x', source = data_prediction_0, radius = 0.3, line_width = 1,  line_color = 'red', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  fig1.circle('prediction_y', 'prediction_x', source = data_prediction_1, radius = 0.3, line_width = 1,  line_color = 'blue', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  fig1.circle('prediction_y', 'prediction_x', source = data_prediction_2, radius = 0.3, line_width = 1,  line_color = 'orange', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  fig1.circle('prediction_y', 'prediction_x', source = data_prediction_3, radius = 0.3, line_width = 1,  line_color = 'black', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  fig1.circle('prediction_y', 'prediction_x', source = data_prediction_4, radius = 0.3, line_width = 1,  line_color = 'purple', line_alpha = 1, fill_alpha = 0, legend_label = 'prediction')
  fig1.patches('parking_space_y', 'parking_space_x', source = data_parking_space, fill_color = "gray", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'parking_space')
  fig1.patches('road_mark_y', 'road_mark_x', source = data_road_mark, fill_color = "green", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'road_mark')
  fig1.multi_line('ground_line_y', 'ground_line_x', source = data_ground_line, line_width = 2, line_color = 'green', line_dash = 'dotted', legend_label = 'ground_line')
  
  hover1_1 = HoverTool(renderers=[fig1.renderers[4]], tooltips=[('init pos x', '@init_pos_point_x'), ('init pos y', '@init_pos_point_y'), ('init pos theta', '@init_pos_point_theta'),
                                                                ('lat init x', '@init_state_x'), ('lat init y', '@init_state_y'), ('lat init theta', '@init_state_theta'), 
                                                                ('lat init delta', '@init_state_delta'), ('lon init s', '@init_state_s'), ('lon init v', '@init_state_v'), 
                                                                ('lon init a', '@init_state_a'), ('replan status', '@replan_status')])
  hover1_2 = HoverTool(renderers=[fig1.renderers[5]], tooltips=[('ego pos x', '@ego_pos_point_x'), ('ego pos y', '@ego_pos_point_y'), ('ego pos theta', '@ego_pos_point_theta')])
  hover1_3 = HoverTool(renderers=[fig1.renderers[50]], tooltips=[('index', '$index'), ('s', '@plan_traj_s)')])
  hover1_4 = HoverTool(renderers=[fig1.renderers[51]], tooltips=[('index', '$index'), ('s', '@plan_traj_s)')])
  hover1_5 = HoverTool(renderers=[fig1.renderers[57]], tooltips=[('index', '$index')])

  fig1.add_tools(hover1_1)
  fig1.add_tools(hover1_2)
  fig1.add_tools(hover1_3)
  fig1.add_tools(hover1_4)
  fig1.add_tools(hover1_5)
  # toolbar
  fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

  # legend
  fig1.legend.click_policy = 'hide'
  return fig1, local_view_data


#start
def update_local_view_data_index(fig1, bag_loader, bag_time, local_view_data):

  ### step 1: 时间戳对齐
  loc_msg_idx = 0
  if bag_loader.loc_msg['enable'] == True:
    while bag_loader.loc_msg['t'][loc_msg_idx] <= bag_time and loc_msg_idx < (len(bag_loader.loc_msg['t'])-2):
        loc_msg_idx = loc_msg_idx + 1
  local_view_data['data_index']['loc_msg_idx'] = loc_msg_idx

  road_msg_idx = 0
  if bag_loader.road_msg['enable'] == True:
    while bag_loader.road_msg['t'][road_msg_idx] <= bag_time and road_msg_idx < (len(bag_loader.road_msg['t'])-2):
        road_msg_idx = road_msg_idx + 1
  local_view_data['data_index']['road_msg_idx'] = road_msg_idx

  fus_msg_idx = 0
  if bag_loader.fus_msg['enable'] == True:
    while bag_loader.fus_msg['t'][fus_msg_idx] <= bag_time and fus_msg_idx < (len(bag_loader.fus_msg['t'])-2):
        fus_msg_idx = fus_msg_idx + 1
    # print("fus_msg_idx:",fus_msg_idx)
  local_view_data['data_index']['fus_msg_idx'] = fus_msg_idx

  me_msg_idx = 0
  if bag_loader.me_msg['enable'] == True:
    while bag_loader.me_msg['t'][me_msg_idx] <= bag_time and me_msg_idx < (len(bag_loader.me_msg['t'])-2):
        me_msg_idx = me_msg_idx + 1
        # print("bag_loader.me_msg['t'][me_msg_idx]:",bag_loader.me_msg['t'][me_msg_idx])
    # print("me_msg_idx:",me_msg_idx)
  local_view_data['data_index']['me_msg_idx'] = me_msg_idx

  # radar_msg_idx = dict()
  radar_msg_idx_list = ['radar_fm_msg_idx','radar_fl_msg_idx','radar_fr_msg_idx','radar_rl_msg_idx','radar_rr_msg_idx']
  radar_msg_idx = [0,0,0,0,0]
  bag_loader_radar_msg = [bag_loader.radar_fm_msg, bag_loader.radar_fl_msg, bag_loader.radar_fr_msg,
                          bag_loader.radar_rl_msg, bag_loader.radar_rr_msg]
  # print("bag_loader.radar_fm_msg:",bag_loader.radar_fm_msg['t'][0])
  for i in range(5):
    if bag_loader_radar_msg[i]['enable'] == True:
      while bag_loader_radar_msg[i]['t'][radar_msg_idx[i]] <= bag_time and radar_msg_idx[i] < (len(bag_loader_radar_msg[i]['t'])-2):
          radar_msg_idx[i] = radar_msg_idx[i] + 1
          # print("bag_loader_radar_msg[i]['t'][radar_msg_idx[i]]:",bag_loader_radar_msg[i]['t'][radar_msg_idx[i]])
          # print("bag_time:",bag_time)
          # print("radar_msg_idx[i]:",radar_msg_idx[i])
      #print('radar_msg_idx:',i,radar_msg_idx[i])
    local_view_data['data_index'][radar_msg_idx_list[i]] = radar_msg_idx[i]

  # radar_fm_msg_idx = 0
  # if bag_loader.radar_fm_msg['enable'] == True:
  #   while bag_loader.radar_fm_msg['t'][radar_fm_msg_idx] <= bag_time and radar_fm_msg_idx < (len(bag_loader.radar_fm_msg['t'])-2):
  #       radar_fm_msg_idx = radar_fm_msg_idx + 1
  # local_view_data['data_index']['radar_fm_msg_idx'] = radar_fm_msg_idx

  vs_msg_idx = 0
  if bag_loader.vs_msg['enable'] == True:
    while bag_loader.vs_msg['t'][vs_msg_idx] <= bag_time and vs_msg_idx < (len(bag_loader.vs_msg['t'])-2):
        vs_msg_idx = vs_msg_idx + 1
  local_view_data['data_index']['vs_msg_idx'] = vs_msg_idx

  plan_msg_idx = 0
  if bag_loader.plan_msg['enable'] == True:
    while bag_loader.plan_msg['t'][plan_msg_idx] <= bag_time and plan_msg_idx < (len(bag_loader.plan_msg['t'])-2):
        plan_msg_idx = plan_msg_idx + 1
  local_view_data['data_index']['plan_msg_idx'] = plan_msg_idx

  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
  local_view_data['data_index']['plan_debug_msg_idx'] = plan_debug_msg_idx

  pred_msg_idx = 0
  if bag_loader.prediction_msg['enable'] == True:
    while bag_loader.prediction_msg['t'][pred_msg_idx] <= bag_time and pred_msg_idx < (len(bag_loader.prediction_msg['t'])-2):
        pred_msg_idx = pred_msg_idx + 1
  local_view_data['data_index']['pred_msg_idx'] = pred_msg_idx

  ctrl_msg_idx = 0
  if bag_loader.ctrl_msg['enable'] == True:
    while bag_loader.ctrl_msg['t'][ctrl_msg_idx] <= bag_time and ctrl_msg_idx < (len(bag_loader.ctrl_msg['t'])-2):
        ctrl_msg_idx = ctrl_msg_idx + 1
  local_view_data['data_index']['ctrl_msg_idx'] = ctrl_msg_idx

  ctrl_debug_msg_idx = 0
  if bag_loader.ctrl_debug_msg['enable'] == True:
    while bag_loader.ctrl_debug_msg['t'][ctrl_debug_msg_idx] <= bag_time and ctrl_debug_msg_idx < (len(bag_loader.ctrl_debug_msg['t'])-2):
        ctrl_debug_msg_idx = ctrl_debug_msg_idx + 1
  local_view_data['data_index']['ctrl_debug_msg_idx'] = ctrl_debug_msg_idx

  return local_view_data
