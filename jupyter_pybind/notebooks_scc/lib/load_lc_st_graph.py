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

coord_tf = coord_transformer()

def load_lc_path_figure(fig1):
  data_st = ColumnDataSource(data = {'front_obj_s':[], 'rear_obj_s':[], 'ego_s':[],
                                      't':[], 'front_obj_s_tar_lane':[],'front_obj_need_dis_vec':[],
                                      'rear_obj_need_dis_vec':[], 'front_obj_future_v_vec':[],
                                      'rear_obj_future_v_vec':[], 'ego_future_v_vec':[],
                                      'ego_sim_s':[]})
  lat_data_vec = ColumnDataSource(data={'lat_path_x_vec':[], 'lat_path_y_vec':[]})
  ori_lat_data_vec = ColumnDataSource(data={'ori_lat_path_x_vec':[], 'ori_lat_path_y_vec':[]})

  lc_path_data_vec = ColumnDataSource(data={'lat_path_v':[] , 'lat_path_t':[]})
  fig1.line('lat_path_y_vec', 'lat_path_x_vec', source = lat_data_vec, legend_label="Lat Path", line_width=2, color = "green")
  fig1.line('ori_lat_path_y_vec','ori_lat_path_x_vec', source = ori_lat_data_vec, legend_label="ori Lat Path", line_width=2, color = "red", line_dash="dashed")
  fig2 = bkp.figure(x_axis_label='t', y_axis_label='s', x_range = [-0.1, 7.0], width=500, height=500, match_aspect = True, aspect_scale=1)
  fig2.line('t', 'front_obj_s', source = data_st, legend_label="FObj S", line_width=2, color = "purple")
  fig2.line('t', 'rear_obj_s', source = data_st, legend_label="RObj S", line_width=2, color = "blue")
  fig2.line('t', 'ego_s', source = data_st, legend_label="Ego", line_width=2, color = "black")
  fig2.line('t', 'ego_sim_s', source = data_st, legend_label="Ego_sim_s", line_width=2, color = "black", line_dash='dashed')
  fig2.line('t', 'front_obj_s_tar_lane', source = data_st, legend_label="TLFObj S", line_width=2, color = "green")
  fig2.line('t', 'front_obj_need_dis_vec', source = data_st, legend_label="FOBJ need s", line_width=2, color = "green", line_dash='dashed')
  fig2.line('t', 'rear_obj_need_dis_vec', source = data_st, legend_label="ROBJ need s", line_width=2, color = "blue", line_dash='dashed')
  fig2.legend.click_policy="hide"

  fig3 = bkp.figure(x_axis_label='t', y_axis_label='v', x_range = [-0.1, 7.0], width=500, height=500, match_aspect = True, aspect_scale=1)
  fig3.line('t', 'front_obj_future_v_vec', source = data_st, legend_label="FObj V", line_width=2, color = "purple")
  fig3.line('t', 'rear_obj_future_v_vec', source = data_st, legend_label="RObj V", line_width=2, color = "blue")
  fig3.line('t', 'ego_future_v_vec', source = data_st, legend_label="Ego_V", line_width=2, color = "black")
  fig3.line('lat_path_t', 'lat_path_v', source = lc_path_data_vec, legend_label="lc_path_V", line_width=2, color = "green")

  fig3.legend.click_policy="hide"

  return fig1, fig2, fig3, data_st, lat_data_vec, ori_lat_data_vec, lc_path_data_vec

def update_lc_path_figure(data_st, lat_data_vec, ori_lat_data_vec, lc_path_data_vec, bag_loader, bag_time, local_view_data,
                          ego_box_data_vec, agent_box_data_vec):
  plan_debug_json_info = local_view_data['data_msg']['plan_debug_json_msg']

  lat_path_x_vec = []
  lat_path_y_vec = []
  ori_lat_path_x_vec = []
  ori_lat_path_y_vec = []
  agent_box_data_x_vec = []
  agent_box_data_y_vec = []
  ego_box_data_x_vec = []
  ego_box_data_y_vec = []
  lat_path_x_vec = plan_debug_json_info['lat_path_x']
  lat_path_y_vec = plan_debug_json_info['lat_path_y']
  ori_lat_path_x_vec = plan_debug_json_info['ori_lat_path_x']
  ori_lat_path_y_vec = plan_debug_json_info['ori_lat_path_y']
  agent_box_data_x_vec = plan_debug_json_info['agent_box_corners_x']
  agent_box_data_y_vec = plan_debug_json_info['agent_box_corners_y']
  ego_box_data_x_vec = plan_debug_json_info['ego_box_corners_x']
  ego_box_data_y_vec = plan_debug_json_info['ego_box_corners_y']

  lat_path_v = []
  lat_path_t = []
  lat_path_v = plan_debug_json_info['lat_path_v']
  lat_path_t = plan_debug_json_info['lat_path_t']


  # 加载转换坐标系##########
  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = local_view_data['data_msg']['loc_msg'].position.position_boot.x
    cur_pos_yn = local_view_data['data_msg']['loc_msg'].position.position_boot.y
    cur_yaw = local_view_data['data_msg']['loc_msg'].orientation.euler_boot.yaw

    coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)
  ########################

  lat_path_x_vec, lat_path_y_vec = coord_tf.global_to_local(lat_path_x_vec, lat_path_y_vec)
  ori_lat_path_x_vec, ori_lat_path_y_vec = coord_tf.global_to_local(ori_lat_path_x_vec, ori_lat_path_y_vec)
  agent_box_data_x_vec, agent_box_data_y_vec = coord_tf.global_to_local(agent_box_data_x_vec, agent_box_data_y_vec)
  ego_box_data_x_vec, ego_box_data_y_vec = coord_tf.global_to_local(ego_box_data_x_vec, ego_box_data_y_vec)
  # 调整 front_obj_s_vec 的长度以匹配 t_vec 的长度
  front_obj_s_vec = []
  front_obj_s_tar_lane_vec = []
  rear_obj_s_vec = []
  front_obj_need_dis_vec = []
  rear_obj_need_dis_vec = []

  front_obj_future_v_vec = []
  rear_obj_future_v_vec = []
  ego_future_v_vec = []

  ego_sim_s = []

  t_vec_length = len(plan_debug_json_info['t_vec'])
  front_obj_s_vec = plan_debug_json_info['front_obj_s_vec']
  front_obj_s_tar_lane_vec = plan_debug_json_info['front_obj_s_tar_lane_vec']
  rear_obj_s_vec = plan_debug_json_info['rear_obj_s_vec']

  front_obj_need_dis_vec = plan_debug_json_info['front_obj_need_dis_vec']
  rear_obj_need_dis_vec = plan_debug_json_info['rear_obj_need_dis_vec']

  front_obj_future_v_vec = plan_debug_json_info['front_obj_future_v_vec']
  rear_obj_future_v_vec = plan_debug_json_info['rear_obj_future_v_vec']
  ego_future_v_vec = plan_debug_json_info['ego_future_v_vec']

  ego_sim_s = plan_debug_json_info['ego_sim_s']

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

  if len(ego_sim_s) < t_vec_length:
      # 如果 front_obj_s_vec 太短，添加默认值（这里使用0作为示例）
      ego_sim_s.extend([0] * (t_vec_length - len(ego_sim_s)))
  elif len(ego_sim_s) > t_vec_length:
      # 如果 front_obj_s_vec 太长，进行截断
      ego_sim_s = ego_sim_s[:t_vec_length]

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
    'ego_sim_s': ego_sim_s
  })

  lat_data_vec.data.update({
    'lat_path_x_vec': lat_path_x_vec,
    'lat_path_y_vec': lat_path_y_vec
  })

  ori_lat_data_vec.data.update({
    'ori_lat_path_x_vec': ori_lat_path_x_vec,
    'ori_lat_path_y_vec': ori_lat_path_y_vec
  })

  lc_path_data_vec.data.update({
    'lat_path_t' : lat_path_t,
    'lat_path_v' : lat_path_v
  })

  ego_box_data_y_vec = [ego_box_data_y_vec[i:i+4] for i in range(0, len(ego_box_data_y_vec), 4)]
  ego_box_data_x_vec = [ego_box_data_x_vec[i:i+4] for i in range(0, len(ego_box_data_x_vec), 4)]
  agent_box_data_y_vec = [agent_box_data_y_vec[i:i+4] for i in range(0, len(agent_box_data_y_vec), 4)]
  agent_box_data_x_vec = [agent_box_data_x_vec[i:i+4] for i in range(0, len(agent_box_data_x_vec), 4)]



  ego_box_data_vec.data.update({
    'corners_y' : ego_box_data_y_vec,
    'corners_x' : ego_box_data_x_vec
  })
  agent_box_data_vec.data.update({
    'corners_y' : agent_box_data_y_vec,
    'corners_x' : agent_box_data_x_vec
  })




