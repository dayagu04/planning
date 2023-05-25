import numpy as np
import time
import ipywidgets
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from IPython.core.display import display, HTML
from bokeh.models import Label, LabelSet
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

# data load
planning_json_value_list = ['VisionLonBehavior_a_target_high', 'VisionLonBehavior_a_target_low', 'VisionLonBehavior_v_target', \
                            'VisionLonBehavior_lead_one_id', 'VisionLonBehavior_lead_one_dis', 'VisionLonBehavior_lead_one_vel', \
                            'VisionLonBehavior_lead_two_id', 'VisionLonBehavior_lead_two_dis', 'VisionLonBehavior_lead_two_vel']

class LoadCyberbag:
  def __init__(self, path) -> None:
    self.bag = Record(path)
    self.plan_msg = {'t':[], 'data':[]}
    self.plan_debug_msg = {'t':[], 'data':[]}
    self.plan_debug_json_info = {'t':[], 'data':[]}


  def load_all_data(self):
    # 1. load planning msg
    for topic, msg, t in self.bag.read_messages("/iflytek/planning/plan"):
      # load timestamp
      self.plan_msg['t'].append(msg.meta.header.timestamp / 1e6)
      self.plan_msg['data'].append(msg)

    # 2. load planning debug msg and json
    for topic, msg, t in self.bag.read_messages("/iflytek/planning/debug_info"):

      self.plan_debug_msg['t'].append(msg.timestamp / 1e3)
      self.plan_debug_msg['data'].append(msg)
      # json decoding
      try:
          json_struct = json.loads(msg.data_json, strict = False)
      except json.decoder.JSONDecodeError as jserr:
          print('except',jserr)

      # load time stramp
      self.plan_debug_json_info['t'].append(msg.timestamp / 1e3)

      # load data from json
      plan_debug_data = {}
      for i in range(len(planning_json_value_list)):
        LoadScalar(plan_debug_data, json_struct, planning_json_value_list[i])
      self.plan_debug_json_info['data'].append(plan_debug_data)

    # time alignment
    t0 = self.plan_debug_msg['t'][0]
    self.plan_msg['t'] = [tmp - t0  for tmp in self.plan_msg['t']]
    self.plan_debug_msg['t'] = [tmp - t0  for tmp in self.plan_debug_msg['t']]
    self.plan_debug_json_info['t'] = [tmp - t0  for tmp in self.plan_debug_json_info['t']]

def load_lon_plan_figure():
  data_st = ColumnDataSource(data = {'t':[], 's':[], 'obs_low':[], 'obs_high':[], 'obs_low_id':[], 'obs_high_id':[], 'obs_low_type':[], 'obs_high_type':[]})
  data_st_plan = ColumnDataSource(data = {'t_long':[], 's_plan':[], 'v_plan':[]})
  data_sv = ColumnDataSource(data = {'s':[], 'v':[], 'v_low':[], 'v_high':[]})
  data_tv = ColumnDataSource(data = {'t':[], 'vel':[]})
  data_ta = ColumnDataSource(data = {'t':[], 'acc':[]})
  data_tj = ColumnDataSource(data = {'t':[], 'jerk':[]})
  data_text = ColumnDataSource(data = {'leadone':[], 'leadtwo':[]})

  lon_plot_data = {'data_st':data_st, \
                   'data_st_plan':data_st_plan, \
                   'data_text':data_text, \
                   'data_sv':data_sv, \
                   'data_tv':data_tv, \
                   'data_ta':data_ta, \
                   'data_tj':data_tj
  }

  hover = HoverTool(tooltips = [
     ('index','$index'),
     ('id_low','@obs_low_id'),
     ('id_high','@obs_high_id'),
     ('low_type','@obs_low_type'),
     ('high_type','@obs_high_type'),
  ])
  # fig1 S-T
  fig1 = bkp.figure(x_axis_label='t', y_axis_label='s', width=600, height=400, tools=[hover,'pan,wheel_zoom,box_zoom,reset'], match_aspect = True, aspect_scale=1)
  fig1.x_range.flipped = False
  # fig2 S-V
  fig2 = bkp.figure(x_axis_label='s', y_axis_label='v', width=600, height=400, match_aspect = True, aspect_scale=1)
  fig2.x_range.flipped = False

  f1 = fig1.line('t', 's', source = data_st, line_width = 2, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_st')
  fig1.line('t', 'obs_low', source = data_st, line_width = 2, line_color = 'cyan', line_dash = 'solid', legend_label = 'obs_low_bound')
  fig1.line('t', 'obs_high', source = data_st, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'obs_high_bound')
  fig1.line('t_long', 's_plan', source = data_st_plan, line_width = 2, line_color = 'green', line_dash = 'solid', legend_label = 's_plan_result')
  fig1.circle('t', 'obs_low', source = data_st, size = 5, alpha = 1.0, legend_label = 'obs_low_bound_pos')
  #label_low_id = LabelSet(x='t', y='obs_low', text='obs_low_id', x_offset=2, y_offset=2, source=data_st)
  #fig1.add_layout(label_low_id)
  fig1.circle('t', 'obs_high', source = data_st, size = 5, alpha = 1.0, legend_label = 'obs_high_bound_pos')
  #label_high_id = LabelSet(x='t', y='obs_high', text='obs_high_id', x_offset=2, y_offset=2, source=data_st)
  #fig1.add_layout(label_high_id)
  f2 = fig2.line('s', 'v', source = data_sv, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'ego_sv')
  fig2.line('s', 'v_low', source = data_sv, line_width = 2, line_color = 'cyan', line_dash = 'solid', legend_label = 'v_low_bound')
  fig2.line('s', 'v_high', source = data_sv, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'v_high_bound')
  fig2.line('s_plan', 'v_plan', source = data_st_plan, line_width = 2, line_color = 'green', line_dash = 'solid', legend_label = 'v_plan')
  
  fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
  fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
  fig1.legend.click_policy = 'hide'
  fig2.legend.click_policy = 'hide'
  return lon_plot_data, fig1, fig2

def update_lon_plan_figure(bag_time, bag_loder, lon_plot_data):
  plan_msg_idx = 0
  while bag_loder.plan_msg['t'][plan_msg_idx] <= bag_time and plan_msg_idx < (len(bag_loder.plan_msg['t'])-2):
      plan_msg_idx = plan_msg_idx + 1

  plan_debug_msg_idx = 0
  while bag_loder.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loder.plan_debug_msg['t'])-2):
      plan_debug_msg_idx = plan_debug_msg_idx + 1

  plan_debug_json_info_idx = 0
  while bag_loder.plan_debug_json_info['t'][plan_debug_json_info_idx] <= bag_time and plan_debug_json_info_idx < (len(bag_loder.plan_debug_json_info['t'])-2):
      plan_debug_json_info_idx = plan_debug_json_info_idx + 1

  t_vec = list(bag_loder.plan_debug_msg['data'][plan_debug_msg_idx].long_ref_path.t_list)

  t_long_vec = []
  for item in (bag_loder.plan_msg['data'][plan_msg_idx].trajectory.trajectory_points):
     t_long_vec.append(item.t)
  s_plan_vec =  []
  for item in (bag_loder.plan_msg['data'][plan_msg_idx].trajectory.trajectory_points):
     s_plan_vec.append(item.distance)
  v_plan_vec =  []
  for item in (bag_loder.plan_msg['data'][plan_msg_idx].trajectory.trajectory_points):
     v_plan_vec.append(item.v)

  s_ref_vec = []
  for item in (bag_loder.plan_debug_msg['data'][plan_debug_msg_idx].long_ref_path.s_refs):
     s_ref_vec.append(item.first)

  obs_low_vec = []
  obs_high_vec = []
  obs_low_id_vec = []
  obs_high_id_vec = []
  obs_low_type_vec = []
  obs_high_type_vec = []
  for item in (bag_loder.plan_debug_msg['data'][plan_debug_msg_idx].long_ref_path.bounds):
     low_bound = item.bound[0].lower
     high_bound = item.bound[0].upper
     low_bound_type = item.bound[0].bound_info.type
     high_bound_type = item.bound[0].bound_info.type
     low_bound_id = item.bound[0].bound_info.id
     high_bound_id = item.bound[0].bound_info.id
     for one_bound in item.bound:
        if one_bound.lower > low_bound:
           low_bound = one_bound.lower
           low_bound_type = one_bound.bound_info.type
           low_bound_id = one_bound.bound_info.id
        if one_bound.upper < high_bound:
           high_bound = one_bound.upper
           high_bound_type = one_bound.bound_info.type
           high_bound_id = one_bound.bound_info.id
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

  print(obs_low_id_vec)
  print(obs_high_id_vec)
  print(t_vec)
  v_ref_vec = []
  for item in (bag_loder.plan_debug_msg['data'][plan_debug_msg_idx].long_ref_path.ds_refs):
     v_ref_vec.append(item.first)

  v_bound_low_vec = []
  for item in (bag_loder.plan_debug_msg['data'][plan_debug_msg_idx].long_ref_path.lon_bound_v.bound):
     v_bound_low_vec.append(item.lower)

  v_bound_high_vec = []
  for item in (bag_loder.plan_debug_msg['data'][plan_debug_msg_idx].long_ref_path.lon_bound_v.bound):
     v_bound_high_vec.append(item.upper)

  a_bound_low_vec = []
  for item in (bag_loder.plan_debug_msg['data'][plan_debug_msg_idx].long_ref_path.lon_bound_a.bound):
     a_bound_low_vec.append(item.lower)

  a_bound_high_vec = []
  for item in (bag_loder.plan_debug_msg['data'][plan_debug_msg_idx].long_ref_path.lon_bound_a.bound):
     a_bound_high_vec.append(item.upper)

  lon_plot_data['data_st'].data.update({
    't': t_vec,
    's': s_ref_vec,
    'obs_low': obs_low_vec,
    'obs_high': obs_high_vec,
    'obs_low_id': obs_low_id_vec,
    'obs_high_id': obs_high_id_vec,
    'obs_low_type': obs_low_type_vec,
    'obs_high_type': obs_high_type_vec
  })

  lon_plot_data['data_st_plan'].data.update({
    't_long': t_long_vec,
    's_plan': s_plan_vec,
    'v_plan': v_plan_vec
  })

  lon_plot_data['data_sv'].data.update({
    's': s_ref_vec,
    'v': v_ref_vec,
    'v_low': v_bound_low_vec,
    'v_high': v_bound_high_vec,
  })

  # an example for printed text
  """ vel_ego = bag_loder.plan_debug_json_info['data'][plan_debug_json_info_idx]['vel_ego']
  text_xn = cur_pos_xn - cur_pos_xn0 - 2.0
  text_yn = cur_pos_yn - cur_pos_yn0 + 2.0
  data_text.data.update({
    'vel_ego_text': ['vel_ego={:.2f}m/s'.format(round(vel_ego, 2))],
    'text_xn': [text_xn],
    'text_yn': [text_yn],
  }) """


