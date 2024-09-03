

import sys
import os
from abc import ABC, abstractmethod
import bokeh.plotting as bkp
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs
from bokeh.io import output_notebook, push_notebook, output_file, export_png
from bokeh.layouts import layout, column, row
from bokeh.plotting import figure, output_file, show, ColumnDataSource

import numpy as np
from IPython.core.display import display, HTML

import logging
sys.path.append('..')
sys.path.append('../lib/')
sys.path.append('../..')
sys.path.append('../../..')
from lib.basic_layers import *
from lib.load_ros_bag import *
from lib.local_view_lib import *
import lib.load_ros_bag

# 先手动写死bag
bag_path = "/share//data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20240607/20240607-16-37-00/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-06-07-16-37-00_no_camera.bag"
html_file = bag_path +".latplan.html"

# bokeh创建的html在jupyter中显示
if isINJupyter():
    display(HTML("<style>.container { width:95% !important;  }</style>"))
    display(
        HTML('''<style>.widget-label {min-width: 25ex !important; }</style>'''))
    output_notebook()

# params 控制fig的样式
ref_path_params = {
  'legend_label' : 'ref_path',
  'line_width' : 5,
  'line_color' : 'red',
  'line_dash' : 'solid',
  'line_alpha' : 0.35
}

soft_upper_bound_params = {
  'legend_label' : 'soft_upper_bound',
  'line_width' : 4,
  'line_color' : 'darkorange',
  'line_dash' : 'solid',
  'line_alpha' : 0.7
}

soft_lower_bound_params = {
  'legend_label' : 'soft_lower_bound',
  'line_width' : 4,
  'line_color' : 'darkorange',
  'line_dash' : 'solid',
  'line_alpha' : 0.7
}

hard_upper_bound_params = {
  'legend_label' : 'hard_upper_bound',
  'line_width' : 4,
  'line_color' : 'maroon',
  'line_dash' : 'solid',
  'line_alpha' : 0.35
}

hard_lower_bound_params = {
  'legend_label' : 'hard_lower_bound',
  'line_width' : 4,
  'line_color' : 'maroon',
  'line_dash' : 'solid',
  'line_alpha' : 0.35,
}

soft_upper_bound_circle_params = {
  'legend_label' : 'soft_upper_bound',
  'line_width' : 4,
  'line_color' : 'darkorange',
  'line_alpha' : 0.7,
  'fill_color' : 'gold',
  'fill_alpha' : 0.7
}

soft_lower_bound_circle_params = {
  'legend_label' : 'soft_lower_bound',
  'line_width' : 4,
  'line_color' : 'darkorange',
  'line_alpha' : 0.7,
  'fill_color' : 'gold',
  'fill_alpha' : 0.7
}

hard_upper_bound_circle_params = {
  'legend_label' : 'hard_upper_bound',
  'line_width' : 4,
  'line_color' : 'maroon',
  'line_alpha' : 0.5,
  'fill_color' : 'gold',
  'fill_alpha' : 0.5
}

hard_lower_bound_circle_params = {
  'legend_label' : 'hard_lower_bound',
  'line_width' : 4,
  'line_color' : 'maroon',
  'line_alpha' : 0.5,
  'fill_color' : 'gold',
  'fill_alpha' : 0.5
}

lat_plan_path_params = {
  'legend_label' : 'lat_plan',
  'line_width' : 5,
  'line_color' : 'violet',
  'line_dash' : 'solid',
  'line_alpha' : 0.6
}

lateral_offset_params = {
    'line_width' : 1, 'line_color' : 'red', 'line_dash' : 'solid', 'legend_label' : 'lateral_offset'
}
smooth_lateral_offset_params = {
    'line_width' : 1, 'line_color' : 'blue', 'line_dash' : 'solid', 'legend_label' : 'smooth_lateral_offset'
}
avoid_way_params = {
    'line_width' : 1, 'line_color' : 'black', 'line_dash' : 'solid', 'legend_label' : 'avoid_way'
}

ref_theta_params = {
    'line_width' : 1, 'line_color' : 'black', 'line_dash' : 'dashed', 'legend_label' : 'ref_theta'
}
traj_theta_params = {
    'line_width' : 1, 'line_color' : 'red', 'line_dash' : 'solid', 'legend_label' : 'theta'
}
ref_y_params = {
    'line_width' : 1, 'line_color' : 'black', 'line_dash' : 'dashed', 'legend_label' : 'ref_y'
}
traj_y_params = {
    'line_width' : 1, 'line_color' : 'red', 'line_dash' : 'solid', 'legend_label' : 'y'
}
traj_acc_params = {
    'line_width' : 1, 'line_color' : 'red', 'line_dash' : 'solid', 'legend_label' : 'lat acc'
}
traj_jerk_params = {
    'line_width' : 1, 'line_color' : 'red', 'line_dash' : 'solid', 'legend_label' : 'lat jerk'
}
traj_steer_params = {
    'line_width' : 1, 'line_color' : 'red', 'line_dash' : 'solid', 'legend_label' : 'steer'
}
traj_steer_dot_params = {
    'line_width' : 1, 'line_color' : 'red', 'line_dash' : 'solid', 'legend_label' : 'steer dot'
}
acc_upper_bound_params = {
    'line_width' : 1, 'line_color' : 'green', 'line_dash' : 'solid', 'legend_label' : 'acc bound'
}
acc_lower_bound_params = {
    'line_width' : 1, 'line_color' : 'green', 'line_dash' : 'solid', 'legend_label' : 'acc bound'
}
jerk_upper_bound_params = {
    'line_width' : 1, 'line_color' : 'green', 'line_dash' : 'solid', 'legend_label' : 'jerk bound'
}
jerk_lower_bound_params = {
    'line_width' : 1, 'line_color' : 'green', 'line_dash' : 'solid', 'legend_label' : 'jerk bound'
}
steer_upper_bound_params = {
    'line_width' : 1, 'line_color' : 'green', 'line_dash' : 'solid', 'legend_label' : 'steer bound'
}
steer_lower_bound_params = {
    'line_width' : 1, 'line_color' : 'green', 'line_dash' : 'solid', 'legend_label' : 'steer bound'
}
steer_dot_upper_bound_params = {
    'line_width' : 1, 'line_color' : 'green', 'line_dash' : 'solid', 'legend_label' : 'steer dot bound'
}
steer_dot_lower_bound_params = {
    'line_width' : 1, 'line_color' : 'green', 'line_dash' : 'solid', 'legend_label' : 'steer dot bound'
}
acc_upper_bound_triangle_params = {
    'size': 5, 'fill_color' : 'grey', 'line_color' : 'grey', 'alpha' : 0.5 , 'legend_label' : 'acc bound'
}
acc_lower_bound_triangle_params = {
    'size': 5, 'fill_color' : 'grey', 'line_color' : 'grey', 'alpha' : 0.5 , 'legend_label' : 'acc bound'
}
jerk_upper_bound_triangle_params = {
    'size': 5, 'fill_color' : 'grey', 'line_color' : 'grey', 'alpha' : 0.5 , 'legend_label' : 'jerk bound'
}
jerk_lower_bound_triangle_params = {
    'size': 5, 'fill_color' : 'grey', 'line_color' : 'grey', 'alpha' : 0.5 , 'legend_label' : 'jerk bound'
}
steer_upper_bound_triangle_params = {
    'size': 5, 'fill_color' : 'grey', 'line_color' : 'grey', 'alpha' : 0.5 , 'legend_label' : 'steer bound'
}
steer_lower_bound_triangle_params = {
    'size': 5, 'fill_color' : 'grey', 'line_color' : 'grey', 'alpha' : 0.5 , 'legend_label' : 'steer bound'
}
steer_dot_upper_bound_triangle_params = {
    'size': 5, 'fill_color' : 'grey', 'line_color' : 'grey', 'alpha' : 0.5 , 'legend_label' : 'steer dot bound'
}
steer_dot_lower_bound_triangle_params = {
    'size': 5, 'fill_color' : 'grey', 'line_color' : 'grey', 'alpha' : 0.5 , 'legend_label' : 'steer dot bound'
}

# 判断是否在jupyter中运行
def isINJupyter():
    try:
        __file__
    except NameError:
        return True
    else:
        return False

def draw_lateral_offset(plan_debug_msg, layer_manager):
    #define figure
    hover = HoverTool(tooltips = [('frame_num', '@pts_xs'),
     ('value', '@pts_ys')
    ])

    lateral_offsets_generate = CommonGenerator()
    smooth_lateral_offsets_generate = CommonGenerator()
    avoid_ways_generate = CommonGenerator()
    global plan_debug_ts
    if plan_debug_msg['enable'] == True:
      lateral_offsets = []
      smooth_lateral_offsets = []
      frame_nums = []
      avoid_ways = []
      # plan_debug_ts =[]
      for i, plan_json_debug in enumerate(plan_debug_msg['json']):
        # t = plan_debug_msg["t"][i]
        # plan_debug_ts.append(t)
        plan_debug = plan_debug_msg['data'][i]
        frame_nums.append(plan_debug.frame_info.frame_num)
        lateral_offsets.append(plan_json_debug['lat_offset'])
        smooth_lateral_offsets.append(plan_json_debug['smooth_lateral_offset'])
        avoid_ways.append(plan_json_debug['avoid_way'] * 0.1)
      frame_num_0 = frame_nums[0]
      frame_nums = [frame_num - frame_num_0 for frame_num in frame_nums]
    fig_lateral_offset = bkp.figure(x_axis_label='frame_num', y_axis_label='lat_offset',x_range = [frame_nums[0], frame_nums[-1]], width=800, height=200)

    lateral_offsets_generate.xys.append((frame_nums, lateral_offsets))
    lateral_offsets_generate.ts = plan_debug_ts
    smooth_lateral_offsets_generate.xys.append((frame_nums, smooth_lateral_offsets))
    smooth_lateral_offsets_generate.ts = plan_debug_ts
    avoid_ways_generate.xys.append((frame_nums, avoid_ways))
    avoid_ways_generate.ts = plan_debug_ts
    lateral_offsets_layer = CurveLayer(fig_lateral_offset ,lateral_offset_params)
    smooth_lateral_offsets_layer = CurveLayer(fig_lateral_offset ,smooth_lateral_offset_params)
    avoid_ways_layer = CurveLayer(fig_lateral_offset ,avoid_way_params)
    layer_manager.AddLayer(lateral_offsets_layer, 'lateral_offsets_layer', lateral_offsets_generate)
    layer_manager.AddLayer(smooth_lateral_offsets_layer, 'smooth_lateral_offsets_layer', smooth_lateral_offsets_generate)
    layer_manager.AddLayer(avoid_ways_layer, 'avoid_ways_layer', avoid_ways_generate)

    fig_lateral_offset.add_tools(hover)
    fig_lateral_offset.toolbar.active_scroll = fig_lateral_offset.select_one(WheelZoomTool)
    fig_lateral_offset.legend.click_policy = "hide"
    return fig_lateral_offset

def draw_lateral_offset_assist(plan_debug_msg, layer_manager):
    #define figure
    global plan_debug_ts
    lateral_offsets_table_generate = TextGenerator()
    if plan_debug_msg['enable'] == True:
      for i, plan_json_debug in enumerate(plan_debug_msg['json']):
        plan_debug = plan_debug_msg['data'][i]
        data_dict = load_avoid(plan_debug, plan_json_debug)
        names = list(data_dict.keys())
        values = list(data_dict.values())
        lateral_offsets_table_generate.xys.append((names, values, [None] * len(names)))
    lateral_offsets_table_generate.ts = plan_debug_ts
    tab_attr_list = ['Attr', 'Val']
    tab_lateral_offsets_layer = TableLayerV2(None, tab_attr_list, table_params)
    layer_manager.AddLayer(
      tab_lateral_offsets_layer, 'tab_lateral_offsets_layer', lateral_offsets_table_generate, 'lateral_offsets_table_generate', 3)
    return tab_lateral_offsets_layer.plot

def draw_lateral_motion(fig_lv, plan_debug_msg, loc_msg, layer_manager):
    # 加载横向plan path、ref path和bound
    coord_tf = coord_transformer()
    ref_path_xys = []
    soft_upper_bound_xys = []
    soft_lower_bound_xys = []
    hard_upper_bound_xys = []
    hard_lower_bound_xys = []
    soft_upper_bound_circle_xys = []
    soft_lower_bound_circle_xys = []
    hard_upper_bound_circle_xys = []
    hard_lower_bound_circle_xys = []
    lat_plan_path_xys = []
    circle_radius = [0.03] * 26
    if plan_debug_msg['enable'] == True:
      for i, plan_debug in enumerate(plan_debug_msg['data']):
        input_topic_timestamp = plan_debug.input_topic_timestamp
        if lib.load_ros_bag.is_new_loc:
          localization_timestamp = input_topic_timestamp.localization
        else :
          if is_bag_main:
            localization_timestamp = input_topic_timestamp.localization_estimate #main分支录制的包
          else:
            localization_timestamp = input_topic_timestamp.localization # main分支之前录得包
        match_loc_msg = find(loc_msg, localization_timestamp)
        lat_motion_plan_input = plan_debug.lateral_motion_planning_input
        lat_motion_plan_output = plan_debug.lateral_motion_planning_output

        if g_is_display_enu:
          ref_x = [lat_motion_plan_input.ref_x_vec[j] for j in range(len(lat_motion_plan_input.ref_x_vec))]
          ref_y = [lat_motion_plan_input.ref_y_vec[j] for j in range(len(lat_motion_plan_input.ref_y_vec))]
          soft_upper_bound_x0_vec = [lat_motion_plan_input.soft_upper_bound_x0_vec[j] for j in range(len(lat_motion_plan_input.soft_upper_bound_x0_vec))]
          soft_upper_bound_y0_vec = [lat_motion_plan_input.soft_upper_bound_y0_vec[j] for j in range(len(lat_motion_plan_input.soft_upper_bound_y0_vec))]
          soft_lower_bound_x0_vec = [lat_motion_plan_input.soft_lower_bound_x0_vec[j] for j in range(len(lat_motion_plan_input.soft_lower_bound_x0_vec))]
          soft_lower_bound_y0_vec = [lat_motion_plan_input.soft_lower_bound_y0_vec[j] for j in range(len(lat_motion_plan_input.soft_lower_bound_y0_vec))]
          soft_upper_bound_x1_vec = [lat_motion_plan_input.soft_upper_bound_x1_vec[j] for j in range(len(lat_motion_plan_input.soft_upper_bound_x1_vec))]
          soft_upper_bound_y1_vec = [lat_motion_plan_input.soft_upper_bound_y1_vec[j] for j in range(len(lat_motion_plan_input.soft_upper_bound_y1_vec))]
          soft_lower_bound_x1_vec = [lat_motion_plan_input.soft_lower_bound_x1_vec[j] for j in range(len(lat_motion_plan_input.soft_lower_bound_x1_vec))]
          soft_lower_bound_y1_vec = [lat_motion_plan_input.soft_lower_bound_y1_vec[j] for j in range(len(lat_motion_plan_input.soft_lower_bound_y1_vec))]
          hard_upper_bound_x0_vec = [lat_motion_plan_input.hard_upper_bound_x0_vec[j] for j in range(len(lat_motion_plan_input.hard_upper_bound_x0_vec))]
          hard_upper_bound_y0_vec = [lat_motion_plan_input.hard_upper_bound_y0_vec[j] for j in range(len(lat_motion_plan_input.hard_upper_bound_y0_vec))]
          hard_lower_bound_x0_vec = [lat_motion_plan_input.hard_lower_bound_x0_vec[j] for j in range(len(lat_motion_plan_input.hard_lower_bound_x0_vec))]
          hard_lower_bound_y0_vec = [lat_motion_plan_input.hard_lower_bound_y0_vec[j] for j in range(len(lat_motion_plan_input.hard_lower_bound_y0_vec))]
          hard_upper_bound_x1_vec = [lat_motion_plan_input.hard_upper_bound_x1_vec[j] for j in range(len(lat_motion_plan_input.hard_upper_bound_x1_vec))]
          hard_upper_bound_y1_vec = [lat_motion_plan_input.hard_upper_bound_y1_vec[j] for j in range(len(lat_motion_plan_input.hard_upper_bound_y1_vec))]
          hard_lower_bound_x1_vec = [lat_motion_plan_input.hard_lower_bound_x1_vec[j] for j in range(len(lat_motion_plan_input.hard_lower_bound_x1_vec))]
          hard_lower_bound_y1_vec = [lat_motion_plan_input.hard_lower_bound_y1_vec[j] for j in range(len(lat_motion_plan_input.hard_lower_bound_y1_vec))]
          path_x_vec = [lat_motion_plan_output.x_vec[j] for j in range(len(lat_motion_plan_output.x_vec))]
          path_y_vec = [lat_motion_plan_output.y_vec[j] for j in range(len(lat_motion_plan_output.y_vec))]
        else:
          ref_x, ref_y = [], []
          soft_upper_bound_x0_vec, soft_upper_bound_y0_vec = [], []
          soft_lower_bound_x0_vec, soft_lower_bound_y0_vec = [], []
          soft_upper_bound_x1_vec, soft_upper_bound_y1_vec = [], []
          soft_lower_bound_x1_vec, soft_lower_bound_y1_vec = [], []
          hard_upper_bound_x0_vec, hard_upper_bound_y0_vec = [], []
          hard_lower_bound_x0_vec, hard_lower_bound_y0_vec = [], []
          hard_upper_bound_x1_vec, hard_upper_bound_y1_vec = [], []
          hard_lower_bound_x1_vec, hard_lower_bound_y1_vec = [], []
          path_x_vec, path_y_vec = [], []
          if match_loc_msg != None: # 长时轨迹
            cur_pos_xn = match_loc_msg.position.position_boot.x
            cur_pos_yn = match_loc_msg.position.position_boot.y
            cur_yaw = match_loc_msg.orientation.euler_boot.yaw
            coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
            ref_x, ref_y = coord_tf.global_to_local(lat_motion_plan_input.ref_x_vec, lat_motion_plan_input.ref_y_vec)
            soft_upper_bound_x0_vec, soft_upper_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.soft_upper_bound_x0_vec, \
              lat_motion_plan_input.soft_upper_bound_y0_vec)
            soft_lower_bound_x0_vec, soft_lower_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.soft_lower_bound_x0_vec, \
              lat_motion_plan_input.soft_lower_bound_y0_vec)
            soft_upper_bound_x1_vec, soft_upper_bound_y1_vec = coord_tf.global_to_local(lat_motion_plan_input.soft_upper_bound_x1_vec, \
              lat_motion_plan_input.soft_upper_bound_y1_vec)
            soft_lower_bound_x1_vec, soft_lower_bound_y1_vec = coord_tf.global_to_local(lat_motion_plan_input.soft_lower_bound_x1_vec, \
              lat_motion_plan_input.soft_lower_bound_y1_vec)
            hard_upper_bound_x0_vec, hard_upper_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.hard_upper_bound_x0_vec, \
              lat_motion_plan_input.hard_upper_bound_y0_vec)
            hard_lower_bound_x0_vec, hard_lower_bound_y0_vec = coord_tf.global_to_local(lat_motion_plan_input.hard_lower_bound_x0_vec, \
              lat_motion_plan_input.hard_lower_bound_y0_vec)
            hard_upper_bound_x1_vec, hard_upper_bound_y1_vec = coord_tf.global_to_local(lat_motion_plan_input.hard_upper_bound_x1_vec, \
              lat_motion_plan_input.hard_upper_bound_y1_vec)
            hard_lower_bound_x1_vec, hard_lower_bound_y1_vec = coord_tf.global_to_local(lat_motion_plan_input.hard_lower_bound_x1_vec, \
              lat_motion_plan_input.hard_lower_bound_y1_vec)
            path_x_vec, path_y_vec = coord_tf.global_to_local(lat_motion_plan_output.x_vec, lat_motion_plan_output.y_vec)

        if len(soft_upper_bound_x0_vec) > 1:
          soft_upper_bound_x0_vec[len(soft_upper_bound_x0_vec) - 1] = soft_upper_bound_x1_vec[len(soft_upper_bound_x1_vec) - 1]
          soft_upper_bound_y0_vec[len(soft_upper_bound_y0_vec) - 1] = soft_upper_bound_y1_vec[len(soft_upper_bound_y1_vec) - 1]
          soft_lower_bound_x0_vec[len(soft_lower_bound_x0_vec) - 1] = soft_lower_bound_x1_vec[len(soft_lower_bound_x1_vec) - 1]
          soft_lower_bound_y0_vec[len(soft_lower_bound_y0_vec) - 1] = soft_lower_bound_y1_vec[len(soft_lower_bound_y1_vec) - 1]
          hard_upper_bound_x0_vec[len(hard_upper_bound_x0_vec) - 1] = hard_upper_bound_x1_vec[len(hard_upper_bound_x1_vec) - 1]
          hard_upper_bound_y0_vec[len(hard_upper_bound_y0_vec) - 1] = hard_upper_bound_y1_vec[len(hard_upper_bound_y1_vec) - 1]
          hard_lower_bound_x0_vec[len(hard_lower_bound_x0_vec) - 1] = hard_lower_bound_x1_vec[len(hard_lower_bound_x1_vec) - 1]
          hard_lower_bound_y0_vec[len(hard_lower_bound_y0_vec) - 1] = hard_lower_bound_y1_vec[len(hard_lower_bound_y1_vec) - 1]

        ref_path_xys.append((ref_y, ref_x))
        soft_upper_bound_xys.append((soft_upper_bound_y0_vec, soft_upper_bound_x0_vec))
        soft_lower_bound_xys.append((soft_lower_bound_y0_vec, soft_lower_bound_x0_vec))
        hard_upper_bound_xys.append((hard_upper_bound_y0_vec, hard_upper_bound_x0_vec))
        hard_lower_bound_xys.append((hard_lower_bound_y0_vec, hard_lower_bound_x0_vec))
        soft_upper_bound_circle_xys.append((soft_upper_bound_y0_vec, soft_upper_bound_x0_vec, circle_radius))
        soft_lower_bound_circle_xys.append((soft_lower_bound_y0_vec, soft_lower_bound_x0_vec, circle_radius))
        hard_upper_bound_circle_xys.append((hard_upper_bound_y0_vec, hard_upper_bound_x0_vec, circle_radius))
        hard_lower_bound_circle_xys.append((hard_lower_bound_y0_vec, hard_lower_bound_x0_vec, circle_radius))
        lat_plan_path_xys.append((path_y_vec, path_x_vec))

      ref_path_generate = CommonGenerator()
      soft_upper_bound_generate = CommonGenerator()
      soft_lower_bound_generate = CommonGenerator()
      hard_upper_bound_generate = CommonGenerator()
      hard_lower_bound_generate = CommonGenerator()
      soft_upper_bound_circle_generate = CircleGenerator()
      soft_lower_bound_circle_generate = CircleGenerator()
      hard_upper_bound_circle_generate = CircleGenerator()
      hard_lower_bound_circle_generate = CircleGenerator()
      lat_plan_path_generate = CommonGenerator()
      ref_path_generate.xys = ref_path_xys
      soft_upper_bound_generate.xys = soft_upper_bound_xys
      soft_lower_bound_generate.xys = soft_lower_bound_xys
      hard_upper_bound_generate.xys = hard_upper_bound_xys
      hard_lower_bound_generate.xys = hard_lower_bound_xys
      soft_upper_bound_circle_generate.xys = soft_upper_bound_circle_xys
      soft_lower_bound_circle_generate.xys = soft_lower_bound_circle_xys
      hard_upper_bound_circle_generate.xys = hard_upper_bound_circle_xys
      hard_lower_bound_circle_generate.xys = hard_lower_bound_circle_xys
      lat_plan_path_generate.xys = lat_plan_path_xys
      ref_path_generate.ts = np.array(plan_debug_ts)
      soft_upper_bound_generate.ts = np.array(plan_debug_ts)
      soft_lower_bound_generate.ts = np.array(plan_debug_ts)
      hard_upper_bound_generate.ts = np.array(plan_debug_ts)
      hard_lower_bound_generate.ts = np.array(plan_debug_ts)
      soft_upper_bound_circle_generate.ts = np.array(plan_debug_ts)
      soft_lower_bound_circle_generate.ts = np.array(plan_debug_ts)
      hard_upper_bound_circle_generate.ts = np.array(plan_debug_ts)
      hard_lower_bound_circle_generate.ts = np.array(plan_debug_ts)
      lat_plan_path_generate.ts = np.array(plan_debug_ts)
      ref_path_layer = CurveLayer(fig_lv, ref_path_params)
      soft_upper_bound_layer = CurveLayer(fig_lv, soft_upper_bound_params)
      soft_lower_bound_layer = CurveLayer(fig_lv, soft_lower_bound_params)
      hard_upper_bound_layer = CurveLayer(fig_lv, hard_upper_bound_params)
      hard_lower_bound_layer = CurveLayer(fig_lv, hard_lower_bound_params)
      soft_upper_bound_circle_layer = CircleLayer(fig_lv, soft_upper_bound_circle_params)
      soft_lower_bound_circle_layer = CircleLayer(fig_lv, soft_lower_bound_circle_params)
      hard_upper_bound_circle_layer = CircleLayer(fig_lv, hard_upper_bound_circle_params)
      hard_lower_bound_circle_layer = CircleLayer(fig_lv, hard_lower_bound_circle_params)
      lat_plan_path_layer = CurveLayer(fig_lv, lat_plan_path_params)
      layer_manager.AddLayer(ref_path_layer, 'ref_path_layer', ref_path_generate, 'ref_path_generate', 2)
      layer_manager.AddLayer(soft_upper_bound_layer, 'soft_upper_bound_layer', soft_upper_bound_generate, 'soft_upper_bound_generate', 2)
      layer_manager.AddLayer(soft_lower_bound_layer, 'soft_lower_bound_layer', soft_lower_bound_generate, 'soft_lower_bound_generate', 2)
      layer_manager.AddLayer(hard_upper_bound_layer, 'hard_upper_bound_layer', hard_upper_bound_generate, 'hard_upper_bound_generate', 2)
      layer_manager.AddLayer(hard_lower_bound_layer, 'hard_lower_bound_layer', hard_lower_bound_generate, 'hard_lower_bound_generate', 2)
      layer_manager.AddLayer(soft_upper_bound_circle_layer, 'soft_upper_bound_circle_layer', soft_upper_bound_circle_generate, 'soft_upper_bound_circle_generate', 3)
      layer_manager.AddLayer(soft_lower_bound_circle_layer, 'soft_lower_bound_circle_layer', soft_lower_bound_circle_generate, 'soft_lower_bound_circle_generate', 3)
      layer_manager.AddLayer(hard_upper_bound_circle_layer, 'hard_upper_bound_circle_layer', hard_upper_bound_circle_generate, 'hard_upper_bound_circle_generate', 3)
      layer_manager.AddLayer(hard_lower_bound_circle_layer, 'hard_lower_bound_circle_layer', hard_lower_bound_circle_generate, 'hard_lower_bound_circle_generate', 3)
      layer_manager.AddLayer(lat_plan_path_layer, 'lat_plan_path_layer', lat_plan_path_generate, 'lat_plan_path_generate', 2)

def draw_lateral_traj_info(plan_debug_msg, loc_msg, vs_msg, layer_manager):
    #define figure
    fig_lat_traj_info1 = bkp.figure(x_axis_label='time', y_axis_label='theta',x_range = [-0.1, 6.0], width=750, height=160)
    fig_lat_traj_info2 = bkp.figure(x_axis_label='time', y_axis_label='y',x_range = fig_lat_traj_info1.x_range, width=750, height=160)
    fig_lat_traj_info3 = bkp.figure(x_axis_label='time', y_axis_label='lat acc',x_range = fig_lat_traj_info1.x_range, width=750, height=160)
    fig_lat_traj_info4 = bkp.figure(x_axis_label='time', y_axis_label='lat jerk',x_range = fig_lat_traj_info1.x_range, width=750, height=160)
    fig_lat_traj_info5 = bkp.figure(x_axis_label='time', y_axis_label='steer',x_range = fig_lat_traj_info1.x_range, width=750, height=160)
    fig_lat_traj_info6 = bkp.figure(x_axis_label='time', y_axis_label='steer dot',x_range = fig_lat_traj_info1.x_range, width=750, height=160)
    global plan_debug_ts
    steer_ratio = 15.7
    rad_to_deg = 57.3
    coord_tf = coord_transformer()
    ref_theta_generate = CommonGenerator()
    traj_theta_generate = CommonGenerator()
    ref_y_generate = CommonGenerator()
    traj_y_generate = CommonGenerator()
    traj_acc_generate = CommonGenerator()
    traj_jerk_generate = CommonGenerator()
    traj_steer_generate = CommonGenerator()
    traj_steer_dot_generate = CommonGenerator()
    acc_upper_bound_generate = CommonGenerator()
    acc_lower_bound_generate = CommonGenerator()
    jerk_upper_bound_generate = CommonGenerator()
    jerk_lower_bound_generate = CommonGenerator()
    steer_upper_bound_generate = CommonGenerator()
    steer_lower_bound_generate = CommonGenerator()
    steer_dot_upper_bound_generate = CommonGenerator()
    steer_dot_lower_bound_generate = CommonGenerator()
    if plan_debug_msg['enable'] == True:
      for i, plan_debug in enumerate(plan_debug_msg['data']):
        lat_motion_plan_input = plan_debug.lateral_motion_planning_input
        lat_motion_plan_output = plan_debug.lateral_motion_planning_output
        input_topic_timestamp = plan_debug.input_topic_timestamp
        if lib.load_ros_bag.is_new_loc:
          localization_timestamp = input_topic_timestamp.localization
          #localization_timestamp = input_topic_timestamp.localization_estimate
        else :
          if is_bag_main:
            localization_timestamp = input_topic_timestamp.localization_estimate #main分支录制的包
          else:
            localization_timestamp = input_topic_timestamp.localization # main分支之前录得包
        vehicle_service_timestamp = input_topic_timestamp.vehicle_service
        match_loc_msg = find(loc_msg, localization_timestamp)
        match_vs_msg = find(vs_msg, vehicle_service_timestamp)
        delta_bound = 360.0 / 14.5 / 57.3
        omega_bound = 240.0 / 14.5 / 57.3
        try:
          delta_bound = min(delta_bound, lat_motion_plan_input.acc_bound / (lat_motion_plan_input.curv_factor * match_vs_msg.vehicle_speed * match_vs_msg.vehicle_speed))
          omega_bound = min(omega_bound, lat_motion_plan_input.jerk_bound / (lat_motion_plan_input.curv_factor * match_vs_msg.vehicle_speed * match_vs_msg.vehicle_speed))
        except:
          print("min delta_bound and min omega_bound")
        time_vec = []
        ref_theta_vec = []
        traj_theta_vec = []
        ref_x_vec = []
        ref_y_vec = []
        traj_x_vec = []
        traj_y_vec = []
        traj_acc_vec = []
        traj_jerk_vec = []
        traj_steer_vec = []
        traj_steer_dot_vec = []
        acc_upper_bound = []
        acc_lower_bound = []
        jerk_upper_bound = []
        jerk_lower_bound = []
        steer_upper_bound = []
        steer_lower_bound = []
        steer_dot_upper_bound = []
        steer_dot_lower_bound = []
        for i in range(len(lat_motion_plan_output.time_vec)):
          time_vec.append(lat_motion_plan_output.time_vec[i])
          ref_theta_vec.append(lat_motion_plan_input.ref_theta_vec[i] * rad_to_deg)
          traj_theta_vec.append(lat_motion_plan_output.theta_vec[i] * rad_to_deg)
          ref_x_vec.append(lat_motion_plan_input.ref_x_vec[i])
          ref_y_vec.append(lat_motion_plan_input.ref_y_vec[i])
          traj_x_vec.append(lat_motion_plan_output.x_vec[i])
          traj_y_vec.append(lat_motion_plan_output.y_vec[i])
          traj_acc_vec.append(lat_motion_plan_output.acc_vec[i])
          traj_jerk_vec.append(lat_motion_plan_output.jerk_vec[i])
          traj_steer_vec.append(lat_motion_plan_output.delta_vec[i] * steer_ratio * rad_to_deg)
          traj_steer_dot_vec.append(lat_motion_plan_output.omega_vec[i] * steer_ratio * rad_to_deg)
          acc_upper_bound.append(lat_motion_plan_input.acc_bound)
          acc_lower_bound.append(-lat_motion_plan_input.acc_bound)
          jerk_upper_bound.append(lat_motion_plan_input.jerk_bound)
          jerk_lower_bound.append(-lat_motion_plan_input.jerk_bound)
          steer_upper_bound.append((delta_bound * steer_ratio * rad_to_deg))
          steer_lower_bound.append(-(delta_bound * steer_ratio * rad_to_deg))
          steer_dot_upper_bound.append((omega_bound * steer_ratio * rad_to_deg))
          steer_dot_lower_bound.append(-(omega_bound * steer_ratio * rad_to_deg))
        ref_theta_generate.xys.append((time_vec, ref_theta_vec))
        traj_theta_generate.xys.append((time_vec, traj_theta_vec))
        if not g_is_display_enu:
          if match_loc_msg != None: # 长时轨迹
            cur_pos_xn = match_loc_msg.position.position_boot.x
            cur_pos_yn = match_loc_msg.position.position_boot.y
            cur_yaw = match_loc_msg.orientation.euler_boot.yaw
            coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
            ref_x_vec, ref_y_vec = coord_tf.global_to_local(ref_x_vec, ref_y_vec)
            traj_x_vec, traj_y_vec = coord_tf.global_to_local(traj_x_vec, traj_y_vec)
        ref_y_generate.xys.append((time_vec, ref_y_vec))
        traj_y_generate.xys.append((time_vec, traj_y_vec))
        traj_acc_generate.xys.append((time_vec, traj_acc_vec))
        traj_jerk_generate.xys.append((time_vec, traj_jerk_vec))
        traj_steer_generate.xys.append((time_vec, traj_steer_vec))
        traj_steer_dot_generate.xys.append((time_vec, traj_steer_dot_vec))
        acc_upper_bound_generate.xys.append((time_vec, acc_upper_bound))
        acc_lower_bound_generate.xys.append((time_vec, acc_lower_bound))
        jerk_upper_bound_generate.xys.append((time_vec, jerk_upper_bound))
        jerk_lower_bound_generate.xys.append((time_vec, jerk_lower_bound))
        steer_upper_bound_generate.xys.append((time_vec, steer_upper_bound))
        steer_lower_bound_generate.xys.append((time_vec, steer_lower_bound))
        steer_dot_upper_bound_generate.xys.append((time_vec, steer_dot_upper_bound))
        steer_dot_lower_bound_generate.xys.append((time_vec, steer_dot_lower_bound))

      ref_theta_generate.ts = plan_debug_ts
      traj_theta_generate.ts = plan_debug_ts
      ref_y_generate.ts = plan_debug_ts
      traj_y_generate.ts = plan_debug_ts
      traj_acc_generate.ts = plan_debug_ts
      traj_jerk_generate.ts = plan_debug_ts
      traj_steer_generate.ts = plan_debug_ts
      traj_steer_dot_generate.ts = plan_debug_ts
      acc_upper_bound_generate.ts = plan_debug_ts
      acc_lower_bound_generate.ts = plan_debug_ts
      jerk_upper_bound_generate.ts = plan_debug_ts
      jerk_lower_bound_generate.ts = plan_debug_ts
      steer_upper_bound_generate.ts = plan_debug_ts
      steer_lower_bound_generate.ts = plan_debug_ts
      steer_dot_upper_bound_generate.ts = plan_debug_ts
      steer_dot_lower_bound_generate.ts = plan_debug_ts

      ref_theta_layer = CurveLayer(fig_lat_traj_info1, ref_theta_params)
      traj_theta_layer = CurveLayer(fig_lat_traj_info1, traj_theta_params)
      ref_y_layer = CurveLayer(fig_lat_traj_info2, ref_y_params)
      traj_y_layer = CurveLayer(fig_lat_traj_info2, traj_y_params)
      traj_acc_layer = CurveLayer(fig_lat_traj_info3, traj_acc_params)
      traj_jerk_layer = CurveLayer(fig_lat_traj_info4, traj_jerk_params)
      traj_steer_layer = CurveLayer(fig_lat_traj_info5, traj_steer_params)
      traj_steer_dot_layer = CurveLayer(fig_lat_traj_info6, traj_steer_dot_params)
      acc_upper_bound_layer = CurveLayer(fig_lat_traj_info3, acc_upper_bound_params)
      acc_lower_bound_layer = CurveLayer(fig_lat_traj_info3, acc_lower_bound_params)
      jerk_upper_bound_layer = CurveLayer(fig_lat_traj_info4, jerk_upper_bound_params)
      jerk_lower_bound_layer = CurveLayer(fig_lat_traj_info4, jerk_lower_bound_params)
      steer_upper_bound_layer = CurveLayer(fig_lat_traj_info5, steer_upper_bound_params)
      steer_lower_bound_layer = CurveLayer(fig_lat_traj_info5, steer_lower_bound_params)
      steer_dot_upper_bound_layer = CurveLayer(fig_lat_traj_info6, steer_dot_upper_bound_params)
      steer_dot_lower_bound_layer = CurveLayer(fig_lat_traj_info6, steer_dot_lower_bound_params)
      acc_upper_bound_triangle_layer = TrianglePointsLayer(fig_lat_traj_info3, 'inverted_triangle', acc_upper_bound_triangle_params)
      acc_lower_bound_triangle_layer = TrianglePointsLayer(fig_lat_traj_info3, 'triangle', acc_lower_bound_triangle_params)
      jerk_upper_bound_triangle_layer = TrianglePointsLayer(fig_lat_traj_info4, 'inverted_triangle', jerk_upper_bound_triangle_params)
      jerk_lower_bound_triangle_layer = TrianglePointsLayer(fig_lat_traj_info4, 'triangle', jerk_lower_bound_triangle_params)
      steer_upper_bound_triangle_layer = TrianglePointsLayer(fig_lat_traj_info5, 'inverted_triangle', steer_upper_bound_triangle_params)
      steer_lower_bound_triangle_layer = TrianglePointsLayer(fig_lat_traj_info5, 'triangle', steer_lower_bound_triangle_params)
      steer_dot_upper_bound_triangle_layer = TrianglePointsLayer(fig_lat_traj_info6, 'inverted_triangle', steer_dot_upper_bound_triangle_params)
      steer_dot_lower_bound_triangle_layer = TrianglePointsLayer(fig_lat_traj_info6, 'triangle', steer_dot_lower_bound_triangle_params)

      layer_manager.AddLayer(ref_theta_layer, 'ref_theta_layer', ref_theta_generate, 'ref_theta_generate', 2)
      layer_manager.AddLayer(traj_theta_layer, 'traj_theta_layer', traj_theta_generate, 'traj_theta_generate', 2)
      layer_manager.AddLayer(ref_y_layer, 'ref_y_layer', ref_y_generate, 'ref_y_generate', 2)
      layer_manager.AddLayer(traj_y_layer, 'traj_y_layer', traj_y_generate, 'traj_y_generate', 2)
      layer_manager.AddLayer(traj_acc_layer, 'traj_acc_layer', traj_acc_generate, 'traj_acc_generate', 2)
      layer_manager.AddLayer(traj_jerk_layer, 'traj_jerk_layer', traj_jerk_generate, 'traj_jerk_generate', 2)
      layer_manager.AddLayer(traj_steer_layer, 'traj_steer_layer', traj_steer_generate, 'traj_steer_generate', 2)
      layer_manager.AddLayer(traj_steer_dot_layer, 'traj_steer_dot_layer', traj_steer_dot_generate, 'traj_steer_dot_generate', 2)
      layer_manager.AddLayer(acc_upper_bound_layer, 'acc_upper_bound_layer', acc_upper_bound_generate, 'acc_upper_bound_generate', 2)
      layer_manager.AddLayer(acc_lower_bound_layer, 'acc_lower_bound_layer', acc_lower_bound_generate, 'acc_lower_bound_generate', 2)
      layer_manager.AddLayer(jerk_upper_bound_layer, 'jerk_upper_bound_layer', jerk_upper_bound_generate, 'jerk_upper_bound_generate', 2)
      layer_manager.AddLayer(jerk_lower_bound_layer, 'jerk_lower_bound_layer', jerk_lower_bound_generate, 'jerk_lower_bound_generate', 2)
      layer_manager.AddLayer(steer_upper_bound_layer, 'steer_upper_bound_layer', steer_upper_bound_generate, 'steer_upper_bound_generate', 2)
      layer_manager.AddLayer(steer_lower_bound_layer, 'steer_lower_bound_layer', steer_lower_bound_generate, 'steer_lower_bound_generate', 2)
      layer_manager.AddLayer(steer_dot_upper_bound_layer, 'steer_dot_upper_bound_layer', steer_dot_upper_bound_generate, 'steer_dot_upper_bound_generate', 2)
      layer_manager.AddLayer(steer_dot_lower_bound_layer, 'steer_dot_lower_bound_layer', steer_dot_lower_bound_generate, 'steer_dot_lower_bound_generate', 2)
      layer_manager.AddLayer(acc_upper_bound_triangle_layer, 'acc_upper_bound_triangle_layer', acc_upper_bound_generate, 'acc_upper_bound_generate', 2)
      layer_manager.AddLayer(acc_lower_bound_triangle_layer, 'acc_lower_bound_triangle_layer', acc_lower_bound_generate, 'acc_lower_bound_generate', 2)
      layer_manager.AddLayer(jerk_upper_bound_triangle_layer, 'jerk_upper_bound_triangle_layer', jerk_upper_bound_generate, 'jerk_upper_bound_generate', 2)
      layer_manager.AddLayer(jerk_lower_bound_triangle_layer, 'jerk_lower_bound_triangle_layer', jerk_lower_bound_generate, 'jerk_lower_bound_generate', 2)
      layer_manager.AddLayer(steer_upper_bound_triangle_layer, 'steer_upper_bound_triangle_layer', steer_upper_bound_generate, 'steer_upper_bound_generate', 2)
      layer_manager.AddLayer(steer_lower_bound_triangle_layer, 'steer_lower_bound_triangle_layer', steer_lower_bound_generate, 'steer_lower_bound_generate', 2)
      layer_manager.AddLayer(steer_dot_upper_bound_triangle_layer, 'steer_dot_upper_bound_triangle_layer', steer_dot_upper_bound_generate, 'steer_dot_upper_bound_generate', 2)
      layer_manager.AddLayer(steer_dot_lower_bound_triangle_layer, 'steer_dot_lower_bound_triangle_layer', steer_dot_lower_bound_generate, 'steer_dot_lower_bound_generate', 2)

    hover1 = HoverTool(tooltips = [('time', '@pts_xs'), ('theta', '@pts_ys')])
    hover2 = HoverTool(tooltips = [('time', '@pts_xs'), ('y', '@pts_ys')])
    hover3 = HoverTool(tooltips = [('time', '@pts_xs'), ('acc', '@pts_ys')])
    hover4 = HoverTool(tooltips = [('time', '@pts_xs'), ('jerk', '@pts_ys')])
    hover5 = HoverTool(tooltips = [('time', '@pts_xs'), ('steer', '@pts_ys')])
    hover6 = HoverTool(tooltips = [('time', '@pts_xs'), ('steer dot', '@pts_ys')])

    fig_lat_traj_info1.add_tools(hover1)
    fig_lat_traj_info2.add_tools(hover2)
    fig_lat_traj_info3.add_tools(hover3)
    fig_lat_traj_info4.add_tools(hover4)
    fig_lat_traj_info5.add_tools(hover5)
    fig_lat_traj_info6.add_tools(hover6)
    fig_lat_traj_info1.toolbar.active_scroll = fig_lat_traj_info1.select_one(WheelZoomTool)
    fig_lat_traj_info2.toolbar.active_scroll = fig_lat_traj_info2.select_one(WheelZoomTool)
    fig_lat_traj_info3.toolbar.active_scroll = fig_lat_traj_info3.select_one(WheelZoomTool)
    fig_lat_traj_info4.toolbar.active_scroll = fig_lat_traj_info4.select_one(WheelZoomTool)
    fig_lat_traj_info5.toolbar.active_scroll = fig_lat_traj_info5.select_one(WheelZoomTool)
    fig_lat_traj_info6.toolbar.active_scroll = fig_lat_traj_info6.select_one(WheelZoomTool)
    fig_lat_traj_info1.legend.click_policy = "hide"
    fig_lat_traj_info2.legend.click_policy = "hide"
    fig_lat_traj_info3.legend.click_policy = "hide"
    fig_lat_traj_info4.legend.click_policy = "hide"
    fig_lat_traj_info5.legend.click_policy = "hide"
    fig_lat_traj_info6.legend.click_policy = "hide"

    return fig_lat_traj_info1, fig_lat_traj_info2, fig_lat_traj_info3, fig_lat_traj_info4, fig_lat_traj_info5, fig_lat_traj_info6

def draw_lateral_traj_param(plan_debug_msg, layer_manager):
    #define figure
    global plan_debug_ts
    lat_motion_q_table_generate = TextGenerator()
    if plan_debug_msg['enable'] == True:
      for i, plan_debug in enumerate(plan_debug_msg['data']):
        lat_motion_plan_input = plan_debug.lateral_motion_planning_input
        names = []
        values = []
        names.append('ref_vel')
        names.append('q_xy')
        names.append('q_theta')
        names.append('q_acc')
        names.append('q_jerk')
        names.append('q_acc_bound')
        names.append('q_jerk_bound')
        names.append('q_soft_bound')
        names.append('q_hard_bound')
        values.append(lat_motion_plan_input.ref_vel)
        values.append(lat_motion_plan_input.q_ref_x)
        values.append(lat_motion_plan_input.q_ref_theta)
        values.append(lat_motion_plan_input.q_acc)
        values.append(lat_motion_plan_input.q_jerk)
        values.append(lat_motion_plan_input.q_acc_bound)
        values.append(lat_motion_plan_input.q_jerk_bound)
        values.append(lat_motion_plan_input.q_soft_corridor)
        values.append(lat_motion_plan_input.q_hard_corridor)
        lat_motion_q_table_generate.xys.append((names, values, [None] * len(names)))
    lat_motion_q_table_generate.ts = plan_debug_ts
    tab_attr_list = ['Attr', 'Val']
    tab_lat_motion_q_layer = TableLayerV2(None, tab_attr_list, table_params)
    layer_manager.AddLayer(
      tab_lat_motion_q_layer, 'tab_lat_motion_q_layer', lat_motion_q_table_generate, 'lat_motion_q_table_generate', 3)
    return tab_lat_motion_q_layer.plot

def plotOnce(bag_path, html_file):
    # 加载bag
    try:
        dataLoader = LoadRosbag(bag_path)
    except:
        print('load cyber_bag error!')
        return

    if isINJupyter():
        max_time = dataLoader.load_all_data()
        print("is in jupyter now!")
    else:
        max_time = dataLoader.load_all_data(False)

    #dataLoader = LoadCyberbag(bag_path)
    #max_time = dataLoader.load_all_data(False)
    plan_debug_msg = dataLoader.plan_debug_msg
    loc_msg = dataLoader.loc_msg
    vs_msg = dataLoader.vs_msg
    layer_manager = LayerManager()

    fig_lv, _ = draw_local_view(dataLoader, layer_manager)
    draw_lateral_motion(fig_lv, plan_debug_msg, loc_msg, layer_manager)
    fig_lateral_offset = draw_lateral_offset(plan_debug_msg, layer_manager)
    fig_lateral_offset_assist = draw_lateral_offset_assist(plan_debug_msg, layer_manager)
    fig_lat_traj_info1, fig_lat_traj_info2, fig_lat_traj_info3, fig_lat_traj_info4, fig_lat_traj_info5, fig_lat_traj_info6 = draw_lateral_traj_info(plan_debug_msg, loc_msg, vs_msg, layer_manager)
    fig_lat_traj_param = draw_lateral_traj_param(plan_debug_msg, layer_manager)

    min_t = sys.maxsize
    max_t = 0
    for gdlabel in layer_manager.gds.keys():
        gd = layer_manager.gds[gdlabel]
        min_t = min(min_t, gd.getMinT())
        max_t = max(max_t, gd.getMaxT())

    data_tmp = {'mt': [min_t]}

    for gdlabel in layer_manager.data_key.keys():
        gd = layer_manager.gds[gdlabel]
        data_label = layer_manager.data_key[gdlabel]
        data_tmp[data_label+'s'] = [gd.xys]
        data_tmp[data_label+'ts'] = [gd.ts]
    bag_data = ColumnDataSource(data=data_tmp)

    callback_arg = slider_callback_arg(bag_data)
    for layerlabels in layer_manager.layers.keys():
        callback_arg.AddSource(layerlabels, layer_manager.layers[layerlabels])
    # find the front one of ( the first which is larger than k)
    binary_search = """
            function binarySearch(ts, k){
                if(ts.length == 0){
                    return 0;
                }
                var left = 0;
                var right = ts.length -1;
                while(left<right){
                    var mid = Math.floor((left + right) / 2);
                    if(ts[mid]<=k){ //if the middle value is less than or equal to k
                        left = mid + 1; //set the left value to the middle value + 1
                    }else{
                        right = mid; //set the right value to the middle value
                    }
                }
                if(left == 0){ //if the left value is 0
                    return 0; //return 0
                }
                return left-1; //return the left value - 1
            }
    """

    car_slider = Slider(start=0, end=max_time-0,
                        value=0, step=0.1, title="time")
    code0 = """
    %s
            console.log("cb_objS");
            console.log(cb_obj);
            console.log("cb_obj.value");
            console.log(cb_obj.value);
            console.log("bag_source");
            console.log(bag_source);
            console.log("bag_source.data");
            console.log(Object.keys(bag_source.data));
            const step = cb_obj.value;
            const data = bag_source.data;

    """

    codes = (layer_manager.code) % (code0) % (binary_search)
    callback = CustomJS(args=callback_arg.arg, code=codes)

    car_slider.js_on_change('value', callback)

    for gdlabel in layer_manager.gds.keys():
        gd = layer_manager.gds[gdlabel]
        if gdlabel is 'ep_source' or gdlabel is 'ep_source2' or gdlabel.startswith('global'):
            gd_frame = gd.atT(max_t)

        else:
            gd_frame = gd.atT(min_t)
        if gdlabel is 'online_obj_source' or gdlabel is 'onlinel_obj_source':
            layer_manager.layers[gdlabel].update(
                gd_frame[0], gd_frame[1], gd_frame[2])
            # print(gd_frame)
        elif gdlabel is 'cfb_source':
            pass
        else:
            if layer_manager.plotdim[gdlabel] == 3:
                layer_manager.layers[gdlabel].update(
                    gd_frame[0], gd_frame[1], gd_frame[2])
            else:
                layer_manager.layers[gdlabel].update(gd_frame[0], gd_frame[1])

    output_file(html_file)

    if isINJupyter():
        # display in jupyter notebook
        output_notebook()

    # pan_1 = Panel(child=row(column(fig_lat_traj_info1, fig_lat_traj_info2, fig_lat_traj_info3, fig_lat_traj_info4, fig_lat_traj_info5, fig_lat_traj_info6), column(fig_lateral_offset, fig_lateral_offset_assist, fig_lat_traj_param)), title="1")
    # pan_2 = Panel(child=row(column(fig_lateral_offset_assist)), title="2")
    # pans = Tabs(tabs=[ pan_1, pan_2 ])
    bkp.show(layout(car_slider, row(fig_lv, column(fig_lat_traj_info1, fig_lat_traj_info2, fig_lat_traj_info3, fig_lat_traj_info4, fig_lat_traj_info5, fig_lat_traj_info6), column(fig_lateral_offset, fig_lateral_offset_assist, fig_lat_traj_param))))

def plotMain():
    # print('sys.argv = ', sys.argv)

    if(len(sys.argv) == 2):
        bag_path = str(sys.argv[1])
        html_file = bag_path +".html"

    else:
        bag_path = str(sys.argv[1])
        html_file = str(sys.argv[2])

    bag_path = sys.argv[1]
    html_path = bag_path

    # print("bag_path: {}\nhtml_path: {}".format(bag_path, html_path))

    if os.path.isfile(bag_path) and (not os.path.isdir(html_path)):
        print("process one bag ...")
        html_file = bag_path + ".lat_plan" + ".html"
        plotOnce(bag_path, html_file)
        return

    if (not os.path.isdir(bag_path)) or (not os.path.isdir(html_path)):
        print("INVALID ARGV:\n bag_path: {}\nhtml_path: {}".format(
            bag_path, html_path))
        return

    if not os.path.exists(html_path):
        os.makedirs(html_path)

    all_bag_files = os.listdir(bag_path)
    print("find {} files".format(len(all_bag_files)))
    generated_count = 0
    for bag_name in all_bag_files:
        if (".0000" in bag_name) and (bag_name.find(".html") == -1):
            print("process {} ...".format(bag_name))
            bag_file = os.path.join(bag_path, bag_name)
            html_file = bag_file + ".lat_plan" + ".html"
            try:
                plotOnce(bag_file, html_file)
                print("html_file = ", html_file)
                generated_count += 1
            except Exception:
                print('failed')

    print("{} html files generated\n".format(generated_count))

if __name__ == '__main__':
    if isINJupyter():
        plotOnce(bag_path, html_file)
    else:
        plotMain()
