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
from lib.load_json import *
from lib.load_struct import *

def load_lc_path_figure():
  data_st = ColumnDataSource(data = {'front_obj_s':[], 'rear_obj_s':[], 'ego_s':[], 't':[], 'front_obj_s_tar_lane':[],
  'front_obj_need_dis_vec':[], 'rear_obj_need_dis_vec':[], 'front_obj_future_v_vec':[], 'rear_obj_future_v_vec':[], 'ego_future_v_vec':[]})
  fig2 = bkp.figure(x_axis_label='t', y_axis_label='s', x_range = [-0.1, 7.0], width=500, height=500, match_aspect = True, aspect_scale=1)
  fig2.line('t', 'front_obj_s', source = data_st, legend_label="FObj S", line_width=2, color = "purple")
  fig2.line('t', 'rear_obj_s', source = data_st, legend_label="RObj S", line_width=2, color = "blue")
  fig2.line('t', 'ego_s', source = data_st, legend_label="Ego", line_width=2, color = "black")
  fig2.line('t', 'front_obj_s_tar_lane', source = data_st, legend_label="TLFObj S", line_width=2, color = "green")
  fig2.line('t', 'front_obj_need_dis_vec', source = data_st, legend_label="FOBJ need s", line_width=2, color = "green", line_dash='dashed')
  fig2.line('t', 'rear_obj_need_dis_vec', source = data_st, legend_label="ROBJ need s", line_width=2, color = "blue", line_dash='dashed')
  fig2.legend.click_policy="hide"

  fig3 = bkp.figure(x_axis_label='t', y_axis_label='v', x_range = [-0.1, 7.0], width=500, height=500, match_aspect = True, aspect_scale=1)
  fig3.line('t', 'front_obj_future_v_vec', source = data_st, legend_label="FObj V", line_width=2, color = "purple")
  fig3.line('t', 'rear_obj_future_v_vec', source = data_st, legend_label="RObj V", line_width=2, color = "blue")
  fig3.line('t', 'ego_future_v_vec', source = data_st, legend_label="Ego_V", line_width=2, color = "black")
  fig3.legend.click_policy="hide"
  
  return fig2, fig3, data_st

def update_lc_path_figure(data_st,bag_loader, bag_time, local_view_data):
  plan_debug_json_info = local_view_data['data_msg']['plan_debug_json_msg']
  # 调整 front_obj_s_vec 的长度以匹配 t_vec 的长度
  front_obj_s_vec = []
  front_obj_s_tar_lane_vec = []
  rear_obj_s_vec = []
  front_obj_need_dis_vec = []
  rear_obj_need_dis_vec = []

  front_obj_future_v_vec = []
  rear_obj_future_v_vec = []
  ego_future_v_vec = []

  t_vec_length = len(plan_debug_json_info['t_vec'])
  front_obj_s_vec = plan_debug_json_info['front_obj_s_vec']
  front_obj_s_tar_lane_vec = plan_debug_json_info['front_obj_s_tar_lane_vec']
  rear_obj_s_vec = plan_debug_json_info['rear_obj_s_vec']

  front_obj_need_dis_vec = plan_debug_json_info['front_obj_need_dis_vec']
  rear_obj_need_dis_vec = plan_debug_json_info['rear_obj_need_dis_vec']

  front_obj_future_v_vec = plan_debug_json_info['front_obj_future_v_vec']
  rear_obj_future_v_vec = plan_debug_json_info['rear_obj_future_v_vec']
  ego_future_v_vec = plan_debug_json_info['ego_future_v_vec']
  
  if len(front_obj_s_vec) < t_vec_length:
      # 如果 front_obj_s_vec 太短，添加默认值（这里使用0作为示例）
      front_obj_s_vec.extend([0] * (t_vec_length - len(front_obj_s_vec)))
  elif len(front_obj_s_vec) > t_vec_length:
      # 如果 front_obj_s_vec 太长，进行截断
      front_obj_s_vec = front_obj_s_vec[:t_vec_length]
  if len(front_obj_s_tar_lane_vec) < t_vec_length:
      # 如果 front_obj_s_vec 太短，添加默认值（这里使用0作为示例）
      front_obj_s_tar_lane_vec.extend([0] * (t_vec_length - len(front_obj_s_tar_lane_vec)))
  elif len(front_obj_s_tar_lane_vec) > t_vec_length:
      # 如果 front_obj_s_vec 太长，进行截断
      front_obj_s_tar_lane_vec = front_obj_s_tar_lane_vec[:t_vec_length]
  if len(rear_obj_s_vec) < t_vec_length:
      # 如果 front_obj_s_vec 太短，添加默认值（这里使用0作为示例）
      rear_obj_s_vec.extend([0] * (t_vec_length - len(rear_obj_s_vec)))
  elif len(rear_obj_s_vec) > t_vec_length:
      # 如果 front_obj_s_vec 太长，进行截断
      rear_obj_s_vec = rear_obj_s_vec[:t_vec_length]

  if len(front_obj_need_dis_vec) < t_vec_length:
      # 如果 front_obj_s_vec 太短，添加默认值（这里使用0作为示例）
      front_obj_need_dis_vec.extend([0] * (t_vec_length - len(front_obj_need_dis_vec)))
  elif len(front_obj_need_dis_vec) > t_vec_length:
      # 如果 front_obj_s_vec 太长，进行截断
      front_obj_need_dis_vec = front_obj_need_dis_vec[:t_vec_length]

  if len(rear_obj_need_dis_vec) < t_vec_length:
      # 如果 front_obj_s_vec 太短，添加默认值（这里使用0作为示例）
      rear_obj_need_dis_vec.extend([0] * (t_vec_length - len(rear_obj_need_dis_vec)))
  elif len(rear_obj_need_dis_vec) > t_vec_length:
      # 如果 front_obj_s_vec 太长，进行截断
      rear_obj_need_dis_vec = rear_obj_need_dis_vec[:t_vec_length]

  if len(front_obj_future_v_vec) < t_vec_length:
      # 如果 front_obj_s_vec 太短，添加默认值（这里使用0作为示例）
      front_obj_future_v_vec.extend([0] * (t_vec_length - len(front_obj_future_v_vec)))
  elif len(front_obj_future_v_vec) > t_vec_length:
      # 如果 front_obj_s_vec 太长，进行截断
      front_obj_future_v_vec = front_obj_future_v_vec[:t_vec_length]

  if len(rear_obj_future_v_vec) < t_vec_length:
      # 如果 front_obj_s_vec 太短，添加默认值（这里使用0作为示例）
      rear_obj_future_v_vec.extend([0] * (t_vec_length - len(rear_obj_future_v_vec)))
  elif len(rear_obj_future_v_vec) > t_vec_length:
      # 如果 front_obj_s_vec 太长，进行截断
      rear_obj_future_v_vec = rear_obj_future_v_vec[:t_vec_length]

  if len(ego_future_v_vec) < t_vec_length:
      # 如果 front_obj_s_vec 太短，添加默认值（这里使用0作为示例）
      ego_future_v_vec.extend([0] * (t_vec_length - len(ego_future_v_vec)))
  elif len(ego_future_v_vec) > t_vec_length:
      # 如果 front_obj_s_vec 太长，进行截断
      ego_future_v_vec = ego_future_v_vec[:t_vec_length]
  
  data_st.data.update({
    'front_obj_s': front_obj_s_vec,
    'front_obj_s_tar_lane': front_obj_s_tar_lane_vec,
    'rear_obj_s': rear_obj_s_vec,
    'front_obj_need_dis_vec': front_obj_need_dis_vec,
    'rear_obj_need_dis_vec': rear_obj_need_dis_vec,
    'ego_s': plan_debug_json_info['ego_s_vec'],
    't': plan_debug_json_info['t_vec'],
    'front_obj_future_v_vec': front_obj_future_v_vec,
    'rear_obj_future_v_vec': rear_obj_future_v_vec,
    'ego_future_v_vec': ego_future_v_vec,
  })

