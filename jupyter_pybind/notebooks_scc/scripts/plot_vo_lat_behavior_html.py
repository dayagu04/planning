from ast import List
import sys
import os
from abc import ABC, abstractmethod
import bokeh.plotting as bkp
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, NumericInput, DataTable, TableColumn, Panel, Tabs
from bokeh.io import output_notebook, push_notebook, output_file, export_png
from bokeh.layouts import layout, column, row
from bokeh.plotting import figure, output_file, show, ColumnDataSource

# +
import numpy as np
from IPython.core.display import display, HTML
from plot_local_view_html import *
import logging
from plot_lon_plan_html import *
sys.path.append('..')
sys.path.append('../lib/')
sys.path.append('../..')
sys.path.append('../../..')
from lib.basic_layers import *
from lib.load_ros_bag import *
from lib.local_view_lib import *
# 先手动写死bag

bag_path = "/root/clzhao/lane_borrow_data/2024-11-26-15-32-53.bag_2024-11-26-20-07-40.1732672608.close-loop.scc.plan"# danger left
bag_path = '/root/clzhao/lane_borrow_data/2024-11-26-15-32-53.bag_2024-11-26-20-07-40.1732689115.close-loop.scc.plan'
abg_path = '/root/clzhao/lane_borrow_data/2024-11-26-15-32-53.bag_2024-11-26-20-07-40.1732693102.open-loop.scc.plan'
bag_path = '/root/clzhao/lane_borrow_data/2024-11-26-15-32-32.bag_2024-11-26-19-45-49.1732694992.close-loop.scc.plan'
bag_path= '/root/clzhao/lane_borrow_data/2024-11-27-20-21-44.bag_2024-11-28-10-19-23.1732777296.close-loop.scc.plan'
bag_path = '/root/clzhao/lane_borrow_data/2024-11-27-11-44-13.bag_2024-11-27-13-51-09.1732789149.close-loop.scc.plan'
bag_path = '/root/clzhao/lane_borrow_data/2024-11-27-11-44-13.bag_2024-11-28-18-31-16.1732789949.close-loop.scc.plan'
bag_path = '/root/clzhao/lane_borrow_data/adata_2024-11-21-17-14-32.bag_2024-11-21-21-26-20.1733277505.close-loop.scc.plan'
bag_path=  '/root/clzhao/lane_borrow_data/non_centirc_2024-12-07-11-23-35.bag_2024-12-09-13-39-10.1733798605.close-loop.scc.plan'


html_file = bag_path +".vo_lat_behavior.html"
# -

# bokeh创建的html在jupyter中显示
if isINJupyter():
    display(HTML("<style>.container { width:95% !important;  }</style>"))
    display(
        HTML('''<style>.widget-label {min-width: 25ex !important; }</style>'''))
    output_notebook()

table_params={
    'width': 300,
    'height':800,
}

table_params1={
    'width': 300,
    'height':300,
}

table_params2={
    'width': 300,
    'height':200,
}

table_params4={
    'width': 300,
    'height':500,
}

speed_search_s_params = {
    'line_width': 2,
    'line_color': 'blue',
    'line_dash': 'dashed',
    'legend_label': 's_search'
}

speed_search_v_params = {
    'line_width': 2,
    'line_color': 'blue',
    'line_dash': 'dashed',
    'legend_label': 'v_search'
}

speed_search_a_params = {
    'line_width': 2,
    'line_color': 'blue',
    'line_dash': 'dashed',
    'legend_label': 'a_search'
}

speed_search_j_params = {
    'line_width': 2,
    'line_color': 'blue',
    'line_dash': 'dashed',
    'legend_label': 'j_search'
}

lane_borrow_obstacle_params = {
  'line_color' : "red",
  'line_width' : 2.0,
  # 'fill_alpha' : 0.3,
  'line_dash': 'dashed',
  'legend_label': 'lane_borrow_static_area'
}

def isINJupyter():
    try:
        __file__
    except NameError:
        return True
    else:
        return False
def draw_speed_adjust_decider(dataLoader, layer_manager):
  speed_adjust_decider_table = TextGenerator()
  plan_debug_ts = []
  for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
    t = dataLoader.plan_debug_msg["t"][i]
    plan_debug_ts.append(t)
    speed_decider_info = plan_debug.st_search_decider_info
    vars = ['ego_v', 'ego_v_cruise', 'st_search_status', 'front_gap_id', \
            'back_gap_id', 'target_objs_flow_vel', 'slot_changed']
    names  = []
    datas = []
    for name in vars:
      try:
          # print(getattr(vo_lat_behavior_plan,name))
        datas.append(getattr(speed_decider_info,name))
        names.append(name)
      except:
        pass
    speed_adjust_decider_table.xys.append((names, datas, [None] * len(names)))
  speed_adjust_decider_table.ts = plan_debug_ts
  tab_attr_list = ['Attr', 'Val']
  tab_speed_adjust_decider_table_layer1 = TableLayerV2(None, tab_attr_list, table_params)
  layer_manager.AddLayer(
      tab_speed_adjust_decider_table_layer1, 'speed_adjust_decider1', speed_adjust_decider_table, 'speed_adjust_decider_table1', 3)
  return tab_speed_adjust_decider_table_layer1.plot

def draw_vo_lat_behavior(dataLoader, layer_manager):
  lat_behavior_table1 = TextGenerator()
  lat_behavior_table2 = TextGenerator()
  plan_debug_ts = []
  # 1. 可视化横向状态机debug信息
  for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
      t = dataLoader.plan_debug_msg["t"][i]
      plan_debug_ts.append(t)

      vo_lat_motion_plan = plan_debug.vo_lat_motion_plan
      vo_lat_behavior_plan = plan_debug.vo_lat_behavior_plan
      lat_behavior_common = plan_debug.lat_behavior_common
      vars = ['fix_lane_virtual_id','target_lane_virtual_id','origin_lane_virtual_id',\
              'lc_request','lc_request_source','turn_light','map_turn_light','lc_turn_light','act_request_source','lc_back_invalid_reason','lc_status',\
                'is_lc_valid','lc_valid_cnt','lc_invalid_obj_id','lc_invalid_reason',\
          'lc_valid_back','lc_back_obj_id','lc_back_cnt','lc_back_invalid_reason',\
            'v_relative_left_lane','is_faster_left_lane','faster_left_lane_cnt','v_relative_right_lane',\
              'is_faster_right_lane','faster_right_lane_cnt','is_forbid_left_alc_car','is_forbid_right_alc_car',\
                'is_side_borrow_bicycle_lane','is_side_borrow_lane','has_origin_lane',\
                  'has_target_lane','enable_left_lc','enable_right_lc','lc_back_reason', ]
      # 'near_car_ids_origin','near_car_ids_target', 'left_alc_car_ids','right_alc_car_ids', ,'avoid_car_ids','avoid_car_allow_max_opposite_offset'
      names  = []
      datas = []
      for name in vars:
        try:
          # print(getattr(vo_lat_behavior_plan,name))
          datas.append(getattr(lat_behavior_common,name))
          names.append(name)
        except:
          pass
      lat_behavior_table1.xys.append((names, datas, [None] * len(names)))

      names  = []
      datas = []
      try:
      # 横向运动规划offset 可视化
        basic_dpoly = vo_lat_motion_plan.basic_dpoly
        datas.append(vo_lat_motion_plan.premove_dpoly_c0 - basic_dpoly[3])
        names.append('premove_dpoly_c0')
        datas.append(vo_lat_motion_plan.avoid_dpoly_c0 - basic_dpoly[3])
        names.append('avoid_dpoly_c0')
      except:
        pass

      names.append('avoid_car_id')
      avoid_car_id_str = ""
      for avoid_car_id in vo_lat_behavior_plan.avoid_car_ids:
        avoid_car_id_str = avoid_car_id_str + str(avoid_car_id) + ' '
      datas.append(avoid_car_id_str)
      # 添加可视化left_alc_car_ids、right_alc_car_ids可视化
      names.append('left_alc_car_ids')
      names.append('right_alc_car_ids')
      left_alc_car_id_str = ""
      right_alc_car_id_str = ""
      for left_alc_car_id in lat_behavior_common.left_alc_car_ids:
        left_alc_car_id_str = left_alc_car_id_str + str(left_alc_car_id) + ' '
      for right_alc_car_id in lat_behavior_common.right_alc_car_ids:
        right_alc_car_id_str = right_alc_car_id_str + str(right_alc_car_id) + ' '
      datas.append(left_alc_car_id_str)
      datas.append(right_alc_car_id_str)
      lat_behavior_table2.xys.append((names, datas, [None] * len(names)))
  lat_behavior_table1.ts = plan_debug_ts
  lat_behavior_table2.ts = plan_debug_ts

  tab_attr_list = ['Attr', 'Val']
  tab_rt_layer1 = TableLayerV2(None, tab_attr_list, table_params)
  layer_manager.AddLayer(
      tab_rt_layer1, 'rt_table_source1', lat_behavior_table1, 'lat_behavior_table1', 3)

  tab_rt_layer2 = TableLayerV2(None, tab_attr_list, table_params2)
  layer_manager.AddLayer(
      tab_rt_layer2, 'rt_table_source2', lat_behavior_table2, 'lat_behavior_table2', 3)

  # 2. 可视化障碍物数据debug信息
  obj_vars = ['id','type','s','l','s_to_ego','max_l_to_ref','min_l_to_ref','nearest_l_to_desire_path', \
            'nearest_l_to_ego', 'vs_lat_relative','vs_lon_relative','vs_lon',
            'nearest_y_to_desired_path','is_accident_car','is_accident_cnt','is_avoid_car','is_lane_lead_obstacle',
            'current_lead_obstacle_to_ego','cutin_p']
  # 'vs_lat_relative','vs_lon_relative'
  obstacle_ids = set()
  obstacle_generates = {}
  # 加载所有障碍物的id
  for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
    environment_model_info = plan_debug.environment_model_info
    for obstacle in environment_model_info.obstacle:
      obstacle_ids.add(obstacle.id)

  for obstacle_id in obstacle_ids:
      obstacle_generates['obstacle_generate_table_' + str(obstacle_id)] = ObjTextGenerator()
      obstacle_generates['obstacle_generate_table_' + str(obstacle_id)].ts = plan_debug_ts

  # 每一个障碍物id对应一个 Generator
  global fusion_object_timestamps
  for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
    fus_msg = find(dataLoader.fus_msg, fusion_object_timestamps[i])
    # if not flag:
    #     # print('find fus_msg error')
    #     obstacle_fusion_generate.xys.append(([], []))
    #     obstacle_snrd_generate.xys.append(([], []))
    #     obstacle_fusion_text_generate.xys.append(([], [], []))
    #     obstacle_snrd_text_generate.xys.append(([], [], []))
    #     continue
    environment_model_info = plan_debug.environment_model_info
    for obstacle_id in obstacle_ids:
      obstacle_generate = obstacle_generates['obstacle_generate_table_' + str(obstacle_id)]
      names  = []
      datas = []
      flag_obj = False
      for obstacle in environment_model_info.obstacle:
        if obstacle_id == obstacle.id:
          flag_obj = True
          for name in obj_vars:
            try:
              # print(getattr(obstacle,name))
              datas.append(getattr(obstacle,name))
              names.append(name)
            except:
              pass
          # 加载对应的笛卡尔下的障碍物速度
          if fus_msg != None:
            for obj in fus_msg.fusion_object:
              if obstacle_id == obj.additional_info.track_id:
                names.append('v_x')
                names.append('v_y')
                datas.append(obj.common_info.relative_velocity.x)
                datas.append(obj.common_info.relative_velocity.y)
                break

          obstacle_generate.xys.append((names, datas, [None] * len(names)))
          break
      if not flag_obj:
        obstacle_generate.xys.append((names, datas, [None] * len(names)))
  tab_attr_list = ['Attr', 'Val']
  tab_lat_rt_obstacle_layer = TableLayerV2(None, tab_attr_list, table_params4)
  lat_rt_obstacle_table = ObjTextGenerator()
  lat_rt_obstacle_table.ts = plan_debug_ts
  names  = ['']
  datas = ['']
  for i,_ in enumerate(plan_debug_ts):
    lat_rt_obstacle_table.xys.append((names, datas, [None] * len(names)))
  layer_manager.AddLayer(
      tab_lat_rt_obstacle_layer, 'lat_rt_obstacle_table_source', lat_rt_obstacle_table, 'lat_rt_obstacle_table', 3)
  return tab_rt_layer1.plot, tab_rt_layer2.plot, tab_lat_rt_obstacle_layer.plot, obstacle_generates

def draw_mlc_data_view(dataLoader, layer_manager):
  mlc_data_table = TextGenerator()
  noa_info_table = TextGenerator()
  plan_debug_ts = []
  for i, plan_debug in enumerate(dataLoader.plan_debug_msg['json']):
      t = dataLoader.plan_debug_msg["t"][i]
      plan_debug_ts.append(t)

      names = []
      datas = []
      vars_lc = ['sdmap_valid_','lane_change_cmd_','cur_state','lc_map_decision','is_in_merge_area',
                'current_lane_order_id','current_lane_virtual_id','current_lane_relative_id',
                'left_boundary_type','right_boundary_type']
      for name in vars_lc:
        try:
          datas.append((plan_debug[name]))
          names.append(name)
        except:
          pass
      mlc_data_table.xys.append((names, datas, [None] * len(names)))

  for i, plan_hmi in enumerate(dataLoader.planning_hmi_msg['data']):
      ad_info = plan_hmi.ad_info
      vars_noa = ['distance_to_ramp','distance_to_split','distance_to_merge']

      names = []
      datas = []
      for name in vars_noa:
        try:
          datas.append(getattr(ad_info,name))
          names.append(name)
        except:
          pass
      noa_info_table.xys.append((names, datas, [None] * len(names)))
  mlc_data_table.ts = plan_debug_ts
  noa_info_table.ts = plan_debug_ts

  tab_attr_list = ['Attr', 'Val']
  table_rt_layer1 = TableLayerV2(None, tab_attr_list, table_params1)
  layer_manager.AddLayer(
      table_rt_layer1, 'decide_table_source1', mlc_data_table, 'mlc_data_table', 3)

  table_rt_layer2 = TableLayerV2(None, tab_attr_list, table_params2)
  layer_manager.AddLayer(
      table_rt_layer2, 'decide_table_source2', noa_info_table, 'noa_info_table', 3)
  return table_rt_layer1.plot, table_rt_layer2.plot

def draw_overtake_lc_data_view(dataLoader, layer_manager):
  overtake_lc_data_table = TextGenerator()
  plan_debug_ts = []
  for i, plan_debug in enumerate(dataLoader.plan_debug_msg['json']):
      t = dataLoader.plan_debug_msg["t"][i]
      plan_debug_ts.append(t)

      names = []
      datas = []
      vars_lc = ["enable_l_", "enable_r_", "is_left_lane_change_safe_", "is_right_lane_change_safe_",
                 "overtake_count_", "is_left_overtake", "is_right_overtake", "trigger_left_overtake",
                 "trigger_right_overtake", "overtake_vehicle_id",  "dash_line_len",
                 "left_route_traffic_speed", "right_route_traffic_speed", "speed_threshold"]
      for name in vars_lc:
        try:
          datas.append((plan_debug[name]))
          names.append(name)
        except:
          pass
      overtake_lc_data_table.xys.append((names, datas, [None] * len(names)))

  overtake_lc_data_table.ts = plan_debug_ts

  tab_attr_list = ['Attr', 'Val']
  table_rt_layer3 = TableLayerV2(None, tab_attr_list, table_params4)
  layer_manager.AddLayer(
      table_rt_layer3, 'decide_table_source4', overtake_lc_data_table, 'overtake_lc_data_table', 3)

  return table_rt_layer3.plot

def get_speed_search_st(plan_debug_msg):
  ts = []
  xys = []
  for i, debug_info in enumerate(plan_debug_msg["data"]):
    ts.append(plan_debug_msg["t"][i])
    one_t_vec = list(debug_info.st_search_decider_info.search_t_vec)
    one_s_vec = list(debug_info.st_search_decider_info.search_s_vec)
    xys.append((one_t_vec, one_s_vec))
  speed_search_base_s = DataGeneratorBase(xys, ts)

  ts = []
  xys = []
  for i, debug_info in enumerate(plan_debug_msg["data"]):
    ts.append(plan_debug_msg["t"][i])
    one_t_vec = list(debug_info.st_search_decider_info.search_t_vec)
    one_v_vec = list(debug_info.st_search_decider_info.search_v_vec)
    xys.append((one_t_vec, one_v_vec))
  speed_search_base_v = DataGeneratorBase(xys, ts)

  ts = []
  xys = []
  for i, debug_info in enumerate(plan_debug_msg["data"]):
    ts.append(plan_debug_msg["t"][i])
    one_t_vec = list(debug_info.st_search_decider_info.search_t_vec)
    one_a_vec = list(debug_info.st_search_decider_info.search_a_vec)
    xys.append((one_t_vec, one_a_vec))
  speed_search_base_a = DataGeneratorBase(xys, ts)

  ts = []
  xys = []
  for i, debug_info in enumerate(plan_debug_msg["data"]):
    ts.append(plan_debug_msg["t"][i])
    one_t_vec = list(debug_info.st_search_decider_info.search_t_vec)
    one_j_vec = list(debug_info.st_search_decider_info.search_j_vec)
    xys.append((one_t_vec, one_j_vec))
  speed_search_base_j = DataGeneratorBase(xys, ts)

  return speed_search_base_s, speed_search_base_v, speed_search_base_a, speed_search_base_j

def draw_v_a_j_fig():
    hover_v = HoverTool(tooltips = [('t', '@pts_xs'),
     ('s', '@pts_ys')
    ])
    fig_vt = bkp.figure(x_axis_label='t',
                        y_axis_label='v',
                        x_range = [-0.1, 7.0],
                        tools=[hover_v, 'pan,wheel_zoom,box_zoom,reset'],
                        width=600,
                        height=400,
                        match_aspect = True,
                        aspect_scale=1)

    fig_vt.toolbar.active_scroll = fig_vt.select_one(WheelZoomTool)
    fig_vt.legend.click_policy = "hide"

    hover_a = HoverTool(tooltips = [('t', '@pts_xs'),
     ('s', '@pts_ys')
    ])
    fig_at = bkp.figure(x_axis_label='t',
                        y_axis_label='a',
                        x_range = [-0.1, 7.0],
                        tools=[hover_a, 'pan,wheel_zoom,box_zoom,reset'],
                        width=600,
                        height=400,
                        match_aspect = True,
                        aspect_scale=1)

    fig_at.toolbar.active_scroll = fig_at.select_one(WheelZoomTool)
    fig_at.legend.click_policy = "hide"

    hover_j = HoverTool(tooltips = [('t', '@pts_xs'),
     ('s', '@pts_ys')
    ])
    fig_jt = bkp.figure(x_axis_label='t',
                        y_axis_label='j',
                        x_range = [-0.1, 7.0],
                        tools=[hover_j, 'pan,wheel_zoom,box_zoom,reset'],
                        width=600,
                        height=400,
                        match_aspect = True,
                        aspect_scale=1)

    fig_jt.toolbar.active_scroll = fig_jt.select_one(WheelZoomTool)
    fig_jt.legend.click_policy = "hide"
    return fig_vt, fig_at, fig_jt

def load_lane_borrow_fig_info(dataLoader, layer_manager, fig_local_view):
  loc_msg = dataLoader.loc_msg
  ts = []
  xys = []
  coord_tf = coord_transformer()
  for i, debug_info in enumerate(dataLoader.plan_debug_msg["data"]):
    input_topic_timestamp = debug_info.input_topic_timestamp
    if lib.load_ros_bag.is_new_loc:
      if 0 != input_topic_timestamp.localization:
        localization_timestamp = input_topic_timestamp.localization
      else :
        localization_timestamp = input_topic_timestamp.localization_estimate
    else :
      if is_bag_main:
        localization_timestamp = input_topic_timestamp.localization_estimate #main分支录制的包
      else:
        localization_timestamp = input_topic_timestamp.localization # main分支之前录得包
    match_loc_msg = find(loc_msg, localization_timestamp)
    if match_loc_msg != None: # 长时轨迹
      cur_pos_xn = match_loc_msg.position.position_boot.x
      cur_pos_yn = match_loc_msg.position.position_boot.y
      cur_yaw = match_loc_msg.orientation.euler_boot.yaw
      coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)

    ts.append(dataLoader.plan_debug_msg["t"][i])
    corner_point_x = []
    corner_point_y = []
    rel_front_left_corner_x, rel_front_left_corner_y = coord_tf.global_to_local(debug_info.lane_borrow_decider_info.block_obs_area.front_left_corner.x,\
                                                                                debug_info.lane_borrow_decider_info.block_obs_area.front_left_corner.y)
    rel_front_right_corner_x, rel_front_right_corner_y = coord_tf.global_to_local(debug_info.lane_borrow_decider_info.block_obs_area.front_right_corner.x,\
                                                                                debug_info.lane_borrow_decider_info.block_obs_area.front_right_corner.y)
    rel_back_right_corner_x, rel_back_right_corner_y = coord_tf.global_to_local(debug_info.lane_borrow_decider_info.block_obs_area.back_right_corner.x,\
                                                                                debug_info.lane_borrow_decider_info.block_obs_area.back_right_corner.y)
    rel_back_left_corner_x, rel_back_left_corner_y = coord_tf.global_to_local(debug_info.lane_borrow_decider_info.block_obs_area.back_left_corner.x,\
                                                                                debug_info.lane_borrow_decider_info.block_obs_area.back_left_corner.y)
    corner_point_x.append(rel_front_left_corner_x)
    corner_point_x.append(rel_front_right_corner_x)
    corner_point_x.append(rel_back_right_corner_x)
    corner_point_x.append(rel_back_left_corner_x)
    corner_point_x.append(rel_front_left_corner_x)

    corner_point_y.append(rel_front_left_corner_y)
    corner_point_y.append(rel_front_right_corner_y)
    corner_point_y.append(rel_back_right_corner_y)
    corner_point_y.append(rel_back_left_corner_y)
    corner_point_y.append(rel_front_left_corner_y)
    xys.append((corner_point_y, corner_point_x))
  lane_borrow_base_static_obs_area_generator = CommonGenerator()
  lane_borrow_base_static_obs_area_generator.xys = xys
  lane_borrow_base_static_obs_area_generator.ts = ts
  lane_borrow_base_static_obs_area_layer = CurveLayer(fig_local_view, lane_borrow_obstacle_params)
  layer_manager.AddLayer(lane_borrow_base_static_obs_area_layer, 'lane_borrow_base_static_obs_area_layer', lane_borrow_base_static_obs_area_generator, 'lane_borrow_base_static_obs_area_generator', 2)

def load_lane_borrow_tab_info(dataLoader, layer_manager):
  lane_borrow_decider_table = TextGenerator()
  plan_debug_ts = []
  for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
    t = dataLoader.plan_debug_msg["t"][i]
    plan_debug_ts.append(t)
    lane_borrow_decider_info = plan_debug.lane_borrow_decider_info
    vars = ['lane_borrow_decider_status', 'ego_l','target_left_l','target_right_l',
            'start_solid_lane_dis', 'end_solid_lane_dis','dis_to_tfls','safe_left_borrow',
              'safe_right_borrow', 'static_blocked_obj_vec', 'intersection_state', 'lane_borrow_failed_reason']
    names  = []
    datas = []
    for name in vars:
      try:
        value = getattr(lane_borrow_decider_info, name)
        if name == 'static_blocked_obj_vec':
          data_i = []
          for value_i in value:
            data_i.append(value_i)
          datas.append(data_i)
        else:
          datas.append(value)

        names.append(name)
      except:
        pass
    lane_borrow_decider_table.xys.append((names, datas, [None] * len(names)))
  lane_borrow_decider_table.ts = plan_debug_ts
  tab_attr_list = ['Attr', 'Val']

  lane_borrow_decider_table_layer = TableLayerV2(None, tab_attr_list, table_params)
  layer_manager.AddLayer(
      lane_borrow_decider_table_layer, 'lane_borrow_decider1', lane_borrow_decider_table, 'lane_borrow_decider_table1', 3)
  return lane_borrow_decider_table_layer.plot

def plotOnce(bag_path, html_file):
    # 加载bag
    try:
        dataLoader = LoadRosbag(bag_path)
    except:
        print('load ros_bag error!')
        return

    if isINJupyter():
       max_time = dataLoader.load_all_data()
       print("is in jupter now!")
    else:
       max_time = dataLoader.load_all_data(False)
    layer_manager = LayerManager()

    fig_local_view, _ = draw_local_view(dataLoader, layer_manager)
    tab_rt1, tab_rt2,tab_lat_rt_obstacle, obstacle_generates = draw_vo_lat_behavior(dataLoader, layer_manager)
    mlc_info_view, noa_info_view = draw_mlc_data_view(dataLoader, layer_manager)
    overtake_lc_info_view = draw_overtake_lc_data_view(dataLoader, layer_manager)

    tab_speed_adjust_decider = draw_speed_adjust_decider(dataLoader, layer_manager)

    load_lane_borrow_fig_info(dataLoader, layer_manager, fig_local_view)
    tab_lane_borrow_decider = load_lane_borrow_tab_info(dataLoader, layer_manager)
    plan_debug_msg = dataLoader.plan_debug_msg
    speed_search_base_s, speed_search_base_v, speed_search_base_a, speed_search_base_j = get_speed_search_st(plan_debug_msg)
    fig_st = draw_lon_st(plan_debug_msg, layer_manager)
    speed_search_layer = CurveLayer(fig_st, speed_search_s_params)
    layer_manager.AddLayer(speed_search_layer, 'speed_search_source', speed_search_base_s, 'speed_search_ref', 2)

    fig_vt, fig_at, fig_jt = draw_v_a_j_fig()

    v_search_layer = CurveLayer(fig_vt, speed_search_v_params)
    layer_manager.AddLayer(v_search_layer, 'v_speed_search_source', speed_search_base_v, 'v_search_ref', 2)

    a_search_layer = CurveLayer(fig_at, speed_search_a_params)
    layer_manager.AddLayer(a_search_layer, 'a_speed_search_source', speed_search_base_a, 'a_search_ref', 2)

    j_search_layer = CurveLayer(fig_jt, speed_search_j_params)
    layer_manager.AddLayer(j_search_layer, 'j_speed_search_source', speed_search_base_j, 'j_search_ref', 2)

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

    for obstacle_generate_key in obstacle_generates.keys():
      data_tmp[obstacle_generate_key+'s'] = [obstacle_generates[obstacle_generate_key].xys]
      data_tmp[obstacle_generate_key+'ts'] = [obstacle_generates[obstacle_generate_key].ts]

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
                        value=0, step=0.05, title="time")
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
            var obstacle_selector_value=0
            console.log("obstacle_selector", obstacle_selector.value);
            obstacle_selector_value = obstacle_selector.value

    """

    codes = (layer_manager.code) % (code0) % (binary_search)
    codes +="""
    let obstacle_generate_table_index = "obstacle_generate_table_"+obstacle_selector_value+"s"
    if (obstacle_generate_table_index in data){
      lat_rt_obstacle_table_source.data['pts_xs'] = data[obstacle_generate_table_index][0][lat_rt_obstacle_table_index][0];
      lat_rt_obstacle_table_source.data['pts_ys'] = data[obstacle_generate_table_index][0][lat_rt_obstacle_table_index][1];
      lat_rt_obstacle_table_source.change.emit();
    }else{
    lat_rt_obstacle_table_source.data["id"]="not exist"
    }
  """
    callback = CustomJS(args=callback_arg.arg, code=codes)
    selector_callback=CustomJS(args=dict(
       car_slider=car_slider
    ),code="""
    console.log("obstacle_selector")
    console.log(cb_obj)
    console.log("car_slider",car_slider.value);
    let val = car_slider.value;
    car_slider.value=val-1.0;
    car_slider.value=val;
    """)
    car_slider.js_on_change('value', callback)
    obstacle_selector.js_on_change('value',selector_callback)

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

    pan_general_info = Panel(child = row(column(tab_lat_rt_obstacle, overtake_lc_info_view), tab_rt1, column(tab_rt2, mlc_info_view, noa_info_view)), title="GeneralInfo")
    pan_speed_search_info = Panel(child = row(column(fig_st, fig_vt, tab_speed_adjust_decider), column(fig_at, fig_jt)), title="SpeedSearchInfo")
    pan_lane_borrow_info = Panel(child = row(column(tab_lane_borrow_decider)), title="LaneBorrowDeciderInfo")

    pans = Tabs(tabs=[ pan_lane_borrow_info,pan_general_info, pan_speed_search_info])
    bkp.show(layout(car_slider, row(column(fig_local_view, obstacle_selector), pans)))

def printHelp():
    print('''\n
USAGE:
    1. <jupyter mode>      change “bag_path” and "html_file" and run
    2. <single file mode>  python3 plot_bag.py bag_file html_file
    3. <folder batch mode> python3 plot_bag.py bag_folder html_folder
\n''')

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
        html_file = bag_path + ".vo_lat_behavior" + ".html"
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
