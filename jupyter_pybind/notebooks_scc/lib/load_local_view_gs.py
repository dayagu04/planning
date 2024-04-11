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

class LoadCyberbag:
  def __init__(self, path) -> None:
    self.bag_path = path
    self.bag = Record(path)
    # loclization msg
    self.loc_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # road msg
    self.road_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

    # fusion object msg
    self.fus_msg = {'abs_t':[], 't':[], 'data':[], 'enable':[]}

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
                         "VisionLonBehavior_a_target_low", "VisionLonBehavior_v_limit_road", "VisionLonBehavior_v_limit_in_turns",
                         "VisionLonBehavior_v_target", "VisionLonBehavior_lead_one_id", 'VisionLonBehavior_stop_start_state',
                         "VisionLonBehavior_lead_one_dis", "VisionLonBehavior_lead_one_vel", "VisionLonBehavior_lead_two_id",
                         "VisionLonBehavior_lead_two_dis", "VisionLonBehavior_lead_two_vel", "solver_condition", "dist_err", "lat_err", "lon_err",
                         "dbw_status"]

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
    return max_time

  def msg_timeline_figure(self):
    topic_list = [
      '/iflytek/localization/ego_pose',
      '/iflytek/fusion/road_fusion',
      '/iflytek/fusion/objects',
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
      self.road_msg,
      self.fus_msg,
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

    fig1 = bkp.figure(plot_width=1200, plot_height=300,
              y_range=topic_list_with_hz, x_axis_type='datetime', title=self.bag_path,
              tools=[hover, taptool, "xwheel_zoom,reset"], active_scroll='xwheel_zoom')
    fig1.circle(x='t', y='topic_with_hz', source=source)
    return fig1

def get_plan_debug_msg_idx(bag_loader, bag_time):
  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
  return plan_debug_msg_idx

def update_local_view_data(fig1, bag_loader, bag_time, local_view_data
                           ):
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

    coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)

    ego_xb, ego_yb = [], []
    ego_xn, ego_yn = [], []
    ### global variables
    # pos offset
    for i in range(len(bag_loader.loc_msg['data'])):
      if (i % 10 != 0): # 下采样 10
        continue
      pos_xn_i = bag_loader.loc_msg['data'][i].pose.local_position.x
      pos_yn_i = bag_loader.loc_msg['data'][i].pose.local_position.y

      ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

      ego_xb.append(ego_local_x)
      ego_yb.append(ego_local_y)
      ego_xn.append(pos_xn_i - cur_pos_xn0)
      ego_yn.append(pos_yn_i - cur_pos_yn0)


    local_view_data['data_ego'].data.update({
      'ego_xb': ego_xb,
      'ego_yb': ego_yb,
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
      'car_xb': car_xb,
      'car_yb': car_yb,
    })

    try:
      vel_ego =  bag_loader.loc_msg['data'][loc_msg_idx].pose.linear_velocity_from_wheel
    except:
      vel_ego = bag_loader.vs_msg['data'][vs_msg_idx].vehicle_speed

    text_xn = cur_pos_xn - cur_pos_xn0 - 2.0
    text_yn = cur_pos_yn - cur_pos_yn0 + 2.0

    steer_deg = bag_loader.vs_msg['data'][vs_msg_idx].steering_wheel_angle * 57.3

    local_view_data['data_text'].data.update({
      'vel_ego_text': ['v={:.2f}\nsteer={:.2f}'.format(round(vel_ego, 2), round(steer_deg, 2))],
      'text_xn': [text_xn],
      'text_yn': [text_yn],
    })


  ### step 3: 加载车道线信息
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
        print('error')
        pass

    try:
      center_line_list = load_lane_center_lines(bag_loader.road_msg['data'][road_msg_idx].reference_line_msg)
    except:
      print("old interface before 2.2.3")
      center_line_list = load_lane_center_lines(bag_loader.road_msg['data'][road_msg_idx].lanes)
    # print(center_line_list)
    for i in range(5):
      # try:
        if 1:
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
            ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)
            line_x_rel.append(ego_local_x)
            line_y_rel.append(ego_local_y)
          center_line_list[i]['line_x_vec'] = line_x_rel
          center_line_list[i]['line_y_vec'] = line_y_rel
          if center_line_list[i]['relative_id'] == 0:
            fig1.renderers[13 + i].glyph.line_dash = 'dotdash'
            fig1.renderers[13 + i].glyph.line_alpha = 1
            fig1.renderers[13 + i].glyph.line_width = 2
          else:
            fig1.renderers[13 + i].glyph.line_dash = 'dotted'
            fig1.renderers[13 + i].glyph.line_alpha = 0.8
            fig1.renderers[13 + i].glyph.line_width = 1

          if center_line_list[i]['relative_id'] == 1000:  # 车道不存在
            center_line_list[i]['line_x_vec'] = []
            center_line_list[i]['line_y_vec'] = []
          data_center_line.data.update({
            'center_line_{}_x'.format(i): center_line_list[i]['line_x_vec'],
            'center_line_{}_y'.format(i): center_line_list[i]['line_y_vec'],
          })
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
    for i in range(5):
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

    local_view_data['data_text'].data.update({
      'vel_ego_text': ['v={:.2f}({:d})\nsteer={:.2}'.format(round(vel_ego, 2),current_lane_virtual_id, round(steer_deg, 2))],
      'text_xn': [text_xn],
      'text_yn': [text_yn],
    })
  ### step 4: 加载障碍物信息
  # load fus_obj
  if bag_loader.fus_msg['enable'] == True:
    fusion_objects = bag_loader.fus_msg['data'][fus_msg_idx].fusion_object
    obstacles_info_all = load_obstacle_paramsV1(fusion_objects)
    # 加载自车坐标系下的数据
    if 0:
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

  #  加载fix_lane, target_lane信息
  ### step 3: 加载planning轨迹信息
  if bag_loader.plan_msg['enable'] == True:
    trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
    # print("trajectory:", trajectory)
    try:
      planning_polynomial = trajectory.target_reference.polynomial
      plan_traj_x, plan_traj_y = gen_line(planning_polynomial[3],planning_polynomial[2], planning_polynomial[1], planning_polynomial[0], 0, 50)

    except:
      plan_x = []
      plan_y = []
      for i in range(len(trajectory.trajectory_points)):
        plan_x.append(trajectory.trajectory_points[i].x)
        plan_y.append(trajectory.trajectory_points[i].y)
      try:
        if plan_x[0] == 0.0 and plan_y[0] == 0.0:
          plan_traj_x = plan_x
          plan_traj_y = plan_y
        else:
          plan_traj_x, plan_traj_y = coord_tf.global_to_local(plan_x, plan_y)
      except:
        pass

    local_view_data['data_planning'].data.update({
        'plan_traj_y' : plan_traj_y,
        'plan_traj_x' : plan_traj_x,
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

  return coord_tf

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
  data_fix_lane = ColumnDataSource(data = {'fix_lane_y':[], 'fix_lane_x':[]})
  data_target_lane = ColumnDataSource(data = {'target_lane_y':[], 'target_lane_x':[]})
  data_origin_lane = ColumnDataSource(data = {'origin_lane_y':[], 'origin_lane_x':[]})
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
  data_planning = ColumnDataSource(data = {'plan_traj_y':[],
                                      'plan_traj_x':[],})
  data_control = ColumnDataSource(data = {'mpc_dx':[],
                                          'mpc_dy':[],})
  data_index = {'loc_msg_idx': 0,
                'road_msg_idx': 0,
                'fus_msg_idx': 0,
                'vs_msg_idx': 0,
                'plan_msg_idx': 0,
                'plan_debug_msg_idx': 0,
                'pred_msg_idx': 0,
                'ctrl_msg_idx': 0,
                'ctrl_debug_msg_idx': 0,
               }

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
                     'data_fix_lane': data_fix_lane ,\
                     'data_target_lane': data_target_lane ,\
                     'data_origin_lane': data_origin_lane ,\
                     'data_fus_obj':data_fus_obj, \
                     'data_snrd_obj':data_snrd_obj, \
                     'data_planning':data_planning,\
                     'data_control':data_control,\
                     'data_index': data_index, \
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
  # fig1.line('center_line_3_y', 'center_line_3_x', source = data_center_line_3, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  # fig1.line('center_line_4_y', 'center_line_4_x', source = data_center_line_4, line_width = 1, line_color = 'blue', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'center_line')
  fig1.line('fix_lane_y', 'fix_lane_x', source = data_fix_lane, line_width = 1, line_color = 'red', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'fix_lane')
  fig1.line('target_lane_y', 'target_lane_x', source = data_target_lane, line_width = 1, line_color = 'orange', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'taget_lane')
  fig1.line('origin_lane_y', 'origin_lane_x', source = data_origin_lane, line_width = 1, line_color = 'black', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'origin_lane')
  fig1.patches('obstacles_y_rel', 'obstacles_x_rel', source = data_fus_obj, fill_color = "gray", line_color = "black", line_width = 1, fill_alpha = 0.3, legend_label = 'obj')
  fig1.patches('obstacles_y_rel', 'obstacles_x_rel', source = data_snrd_obj, fill_color = "black", line_color = "black", line_width = 1, fill_alpha = 0.5, legend_label = 'snrd')
  fig1.text('pos_y_rel', 'pos_x_rel', text = 'obs_label' ,source = data_fus_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'fusion_info')
  fig1.text('pos_y_rel', 'pos_x_rel', text = 'obs_label' ,source = data_snrd_obj, text_color="red", text_align="center", text_font_size="10pt", legend_label = 'snrd_info')
  #fig1.line('plan_traj_y', 'plan_traj_x', source = data_planning, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.6, legend_label = 'plan')
  #fig1.line('mpc_dy', 'mpc_dx', source = data_control, line_width = 5, line_color = 'green', line_dash = 'dashed', line_alpha = 0.8, legend_label = 'ctrl_traj')
  # toolbar
  fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

  # legend
  fig1.legend.click_policy = 'hide'
  return fig1, local_view_data
