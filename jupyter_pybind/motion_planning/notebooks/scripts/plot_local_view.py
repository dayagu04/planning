import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_config import *
from lib.load_struct import *
from lib.load_rotate import *
sys.path.append('../..')
sys.path.append('../../../')


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
from lib.basic_layers import *
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, HoverTool
from cyber_record.record import Record

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

# bag path and frame dt
bag_path = "/docker_share/data/clren/code/new_planning2/planning/20230515203542.record.00000"
frame_dt = 0.02

# data load
class LoadCyberbag:
  def __init__(self, path) -> None:
    self.bag = Record(path)
    # loclization msg
    self.loc_msg = {'t':[], 'data':[]}

    # road msg
    self.road_msg = {'t':[], 'data':[]}

    # fusion object msg
    self.fus_msg = {'t':[], 'data':[]}

    # vehicle service msg
    self.vs_msg = {'t':[], 'data':[]}


  def load_all_data(self):
    # 1. load localization msg
    try:
      for topic, msg, t in self.bag.read_messages("/localization"):
        # load timestamp
        self.loc_msg['t'].append(msg.header.timestamp / 1e6)
        self.loc_msg['data'].append(msg)
      self.loc_msg['t'] = [tmp - self.loc_msg['t'][0]  for tmp in self.loc_msg['t']]
    except:
      print('missing /iflytek/localization/ego_pose !!!')

    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/road_fusion"):
        self.road_msg['t'].append(msg.header.timestamp / 1e6)
        self.road_msg['data'].append(msg)
      self.road_msg['t'] = [tmp - self.road_msg['t'][0]  for tmp in self.road_msg['t']]
    except:
      print('missing /iflytek/fusion/road_fusion topic !!!')

    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/fusion/objects"):
        self.fus_msg['t'].append(msg.header.timestamp / 1e6)
        self.fus_msg['data'].append(msg)
      self.fus_msg['t'] = [tmp - self.fus_msg['t'][0]  for tmp in self.fus_msg['t']]
    except:
      print('missing /iflytek/fusion/objects !!!')

    try:
      for topic, msg, t in self.bag.read_messages("/iflytek/vehicle_service"):
        self.vs_msg['t'].append(msg.header.timestamp / 1e3)
        self.vs_msg['data'].append(msg)
      self.vs_msg['t'] = [tmp - self.vs_msg['t'][0]  for tmp in self.vs_msg['t']]
    except:
      print("missing /iflytek/vehicle_service !!!")

def load_lane_lines(lanes):
  line_info_list = []

  for i in range(6):
    lane_info_l = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
    if i< len(lanes):
        lane = lanes[i]
        left_line = lane.left_lane_boundary
        left_line_coef = left_line.poly_coefficient
        line_x, line_y = gen_line(left_line_coef[0], left_line_coef[1], left_line_coef[2], left_line_coef[3], \
          left_line.begin, left_line.end)

        lane_info_l['line_x_vec'] = line_x
        lane_info_l['line_y_vec'] = line_y
        lane_info_l['type'] = left_line.segment[0].type

        line_info_list.append(lane_info_l)

        lane_info_r = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
        right_line = lane.right_lane_boundary
        right_line_coef = right_line.poly_coefficient
        line_x, line_y = gen_line(right_line_coef[0], right_line_coef[1], right_line_coef[2], right_line_coef[3], \
          right_line.begin, right_line.end)

        lane_info_r['line_x_vec'] = line_x
        lane_info_r['line_y_vec'] = line_y
        lane_info_r['type'] = left_line.segment[0].type
        line_info_list.append(lane_info_r)
    else:
        line_x, line_y = gen_line(0,0,0,0,0,0)
        lane_info_l['line_x_vec'] = line_x
        lane_info_l['line_y_vec'] = line_y
        lane_info_l['type'] = []
        line_info_list.append(lane_info_l)
        line_info_list.append(lane_info_l)

  return line_info_list


bag_loder = LoadCyberbag(bag_path)
bag_loder.load_all_data()
# sliders
class LocalViewSlider:
  def __init__(self, bag_data, silder_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=bag_loder.vs_msg['t'][-1], value=-0.1, step=frame_dt)
    ipywidgets.interact(silder_callback, bag_time = self.time_slider)

# figures
fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=500, height=800, match_aspect = True, aspect_scale=1)
fig1.x_range.flipped = True

data_car = ColumnDataSource(data = {'car_yb':[], 'car_xb':[]})
data_ego = ColumnDataSource(data = {'ego_yb':[], 'ego_xb':[]})
data_text = ColumnDataSource(data = {'vel_ego_text':[]})
data_lane_0 = ColumnDataSource(data = {'line_0_y':[], 'line_0_x':[]})
data_lane_1 = ColumnDataSource(data = {'line_1_y':[], 'line_1_x':[]})
data_lane_2 = ColumnDataSource(data = {'line_2_y':[], 'line_2_x':[]})
data_lane_3 = ColumnDataSource(data = {'line_3_y':[], 'line_3_x':[]})
data_lane_4 = ColumnDataSource(data = {'line_4_y':[], 'line_4_x':[]})
data_lane_5 = ColumnDataSource(data = {'line_5_y':[], 'line_5_x':[]})

data_fus_obj = ColumnDataSource(data = {'obstacles_y':[], 'obstacles_x':[],
                                        'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
f1 = fig1.patch('car_yb', 'car_xb', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
fig1.line('ego_yb', 'ego_xb', source = data_ego, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
fig1.text(2.0, -2.0, text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt")
fig1.line('line_0_y', 'line_0_x', source = data_lane_0, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
fig1.line('line_1_y', 'line_1_x', source = data_lane_1, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
fig1.line('line_2_y', 'line_2_x', source = data_lane_2, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
fig1.line('line_3_y', 'line_3_x', source = data_lane_3, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
fig1.line('line_4_y', 'line_4_x', source = data_lane_4, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
fig1.line('line_5_y', 'line_5_x', source = data_lane_5, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
fig1.patches('obstacles_y', 'obstacles_x', source = data_fus_obj, fill_color = "gray", line_color = "black", line_width = 1, fill_alpha = 0.5, legend_label = 'obj')
fig1.text('pos_y', 'pos_x', text = 'obs_label' ,source = data_fus_obj, text_color="gray", text_align="center", text_font_size="8pt", legend_label = 'obj_info')
fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

fig1.legend.click_policy = 'hide'

cur_pos_xn0 = cur_pos_xn = bag_loder.loc_msg['data'][0].pose.local_position.x
cur_pos_yn0 = cur_pos_yn = bag_loder.loc_msg['data'][0].pose.local_position.y


# car pos in local coordinates
car_xb, car_yb = load_car_params_patch()

def silder_callback(bag_time):
  kwargs = locals()

  # step1: 加载定位信息
  try:
    loc_msg_idx = 0
    while bag_loder.loc_msg['t'][loc_msg_idx] <= bag_time and loc_msg_idx < (len(bag_loder.loc_msg['t'])-2):
        loc_msg_idx = loc_msg_idx + 1
    # ego pos in local and global coordinates
    cur_pos_xn = bag_loder.loc_msg['data'][loc_msg_idx].pose.local_position.x
    cur_pos_yn = bag_loder.loc_msg['data'][loc_msg_idx].pose.local_position.y

    cur_yaw = bag_loder.loc_msg['data'][loc_msg_idx].pose.euler_angles.yaw
    ego_xb, ego_yb = [], []
    ego_xn, ego_yn = [], []

    for i in range(len(bag_loder.loc_msg['data'])):
      pos_xn_i = bag_loder.loc_msg['data'][i].pose.local_position.x
      pos_yn_i = bag_loder.loc_msg['data'][i].pose.local_position.y

      ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

      ego_xb.append(ego_local_x)
      ego_yb.append(ego_local_y)
      ego_xn.append(pos_xn_i - cur_pos_xn0)
      ego_yn.append(pos_yn_i - cur_pos_yn0)

    data_ego.data.update({
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


    data_car.data.update({
      'car_xb': car_xb,
      'car_yb': car_yb,
    })

    # an example for printed text
    vel_ego =  bag_loder.vs_msg['data'][vs_msg_idx].vehicle_speed

    text_xn = cur_pos_xn - cur_pos_xn0 - 2.0
    text_yn = cur_pos_yn - cur_pos_yn0 + 2.0
    data_text.data.update({
      'vel_ego_text': ['vel_ego={:.2f}m/s'.format(round(vel_ego, 2))],
      'text_xn': [text_xn],
      'text_yn': [text_yn],
    })
  except:
    print('no loc_msg')

  # step2: 加载车道线信息
  try:
    road_msg_idx = 0
    while bag_loder.road_msg['t'][road_msg_idx] <= bag_time and road_msg_idx < (len(bag_loder.road_msg['t'])-2):
        road_msg_idx = road_msg_idx + 1
    # load lane info
    line_info_list = load_lane_lines(bag_loder.road_msg['data'][road_msg_idx].lanes)
    # update lane info
    data_lane_dict = {
      0:data_lane_0,
      1:data_lane_1,
      2:data_lane_2,
      3:data_lane_3,
      4:data_lane_4,
      5:data_lane_5,
    }
    for i in range(6):
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
        print('update lane line error')        
  except:
    print('no road_msg')
  
  # step3: 加载障碍物信息
  try:
    fus_msg_idx = 0
    while bag_loder.fus_msg['t'][fus_msg_idx] <= bag_time and fus_msg_idx < (len(bag_loder.fus_msg['t'])-2):
        fus_msg_idx = fus_msg_idx + 1
    # load fus_obj
    try:
      fusion_objects = bag_loder.fus_msg['data'][fus_msg_idx].fusion_object
      obstacles_info = load_obstacle_params(fusion_objects)
      data_fus_obj.data.update({
        'obstacles_x': obstacles_info['obstacles_x'],
        'obstacles_y': obstacles_info['obstacles_y'],
        'pos_x' : obstacles_info['pos_x'],
        'pos_y' : obstacles_info['pos_y'],
        'obs_label' : obstacles_info['obs_label'],
      })
    except:
      print('update obstacle error')
  except:
    print('no fus_msg')

  # step3: 加载vehicle service信息
  try:
    vs_msg_idx = 0
    while bag_loder.vs_msg['t'][vs_msg_idx] <= bag_time and vs_msg_idx < (len(bag_loder.vs_msg['t'])-2):
        vs_msg_idx = vs_msg_idx + 1
  except:
    print('no vs_msg')



  # load fus_obj
  try:
    fusion_objects = bag_loder.fus_msg['data'][fus_msg_idx].fusion_object
    obstacles_info = load_obstacle_params(fusion_objects)
    data_fus_obj.data.update({
      'obstacles_x': obstacles_info['obstacles_x'],
      'obstacles_y': obstacles_info['obstacles_y'],
      'pos_x' : obstacles_info['pos_x'],
      'pos_y' : obstacles_info['pos_y'],
      'obs_label' : obstacles_info['obs_label'],
    })
  except:
    print('update obstacle error')
  
  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(bag_loder.loc_msg, silder_callback)
