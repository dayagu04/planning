import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_local_view import *
from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *
# from lib.load_ros_bag import is_match_planning, is_bag_main
from lib.load_ros_bag import LoadRosbag
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

import numpy as np
import time
import ipywidgets
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from IPython.core.display import display, HTML
from bokeh.models import Label, DataTable, TableColumn
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
from bokeh.models import WheelZoomTool, HoverTool
from google.protobuf.descriptor import FieldDescriptor

# from jupyter_pybind import dp_path_decider_py
from python_proto import dp_road_graph_pb2 #?
# load bag info
car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_20260/trigger/20250217/20250217-14-43-16/data_collection_CHERY_E0Y_20260_EVENT_MANUAL_2025-02-17-14-43-16_no_camera.bag.1743412122.open-loop.scc.plan"
bag_path = '/root/clzhao/lane_borrow_data/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-03-26-11-22-40.bag_2025-04-01-10-18-13.1743494800.open-loop.scc.plan'
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250401/20250401-16-12-19/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-04-01-16-12-19_no_camera.bag.1743561549.open-loop.scc.plan"
bag_path = '/root/clzhao/lane_borrow_data/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-03-19-16-38-14.bag_2025-04-07-16-02-38.1744015116.open-loop.scc.plan'
bag_path= "/root/clzhao/lane_borrow_data/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-03-19-16-38-14.bag_2025-04-07-16-02-38.1744441529.open-loop.scc.plan"
bag_path='/home/hanli13/data/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-03-19-16-38-14.bag_2025-04-07-16-02-38.1744634643.close-loop.scc.plan'
bag_path="/home/hanli13/data/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-03-19-16-38-14.bag_2025-04-07-16-02-38.1744695345.close-loop.scc.plan"
bag_path ="/root/clzhao/lane_borrow_data/data_collection_CHERY_E0Y_14520_EVENT_MANUAL_2025-04-14-17-09-06.bag_2025-04-17-10-17-02.1744871875.close-loop.scc.plan"
bag_path = "/root/clzhao/log_data/data_collection_CHERY_E0Y_13484_EVENT_FILTER_2025-04-24-19-09-30.bag_2025-04-25-16-12-39.1745745181.close-loop.scc.plan"
frame_dt = 0.1

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadRosbag(bag_path) # 1-2 bag_loader
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()
fig1.height = 900
fig1.width = 600
#default value
for i, debug_msg in enumerate(bag_loader.plan_debug_msg['data']):
  if i<10:
     continue
  else:
    dp_road_info =  debug_msg.dp_road_info
    v_cruise =dp_road_info.v_cruise
    s_range = dp_road_info.s_range
    l_range = dp_road_info.l_range
    coeff_l_cost =dp_road_info.dp_param.coeff_l_cost
    coeff_dl_cost =dp_road_info.dp_param.coeff_dl_cost
    coeff_ddl_cost =dp_road_info.dp_param.coeff_ddl_cost
    coeff_end_l_cost =dp_road_info.dp_param.coeff_end_l_cost
    coeff_collision_cost = dp_road_info.dp_param.coeff_collision_cost
    collision_distance =dp_road_info.dp_param.collision_distance
    path_resolution = dp_road_info.dp_param.path_resolution
    break
  # print("default s_range ",s_range,"default l_range ",l_range )
#   if hasattr(dp_road_info, 'dp_param'):


# init pybind
# dp_path_decider_py.Init()
# bag_loader, bag_data):
fig2 = bkp.figure(x_axis_label='L', y_axis_label='S', width=600, height=900,x_range=(18, -18), y_range=(30, 150), match_aspect = True, aspect_scale=1)
#reserve
max_objects = 50

obstacles_info_data_sources = [ColumnDataSource(data={'L': [], 'S': []}) for _ in range(max_objects)]
lines = [
    fig2.patch(
        'L', 'S',
        source=source,
        fill_alpha=0.2, fill_color='black', line_color='black',
        legend_label=f"obj_{i}")
    for i, source in enumerate(obstacles_info_data_sources)
]

sampled_points_data_source = ColumnDataSource(data={'L': [], 'S': []})
update_sampled_points_data_source = ColumnDataSource(data={'L': [], 'S': []})
fined_path_point_data_source = ColumnDataSource(data={'L': [], 'S': []})
update_fined_path_point_data_source= ColumnDataSource(data={'L': [], 'S': []})
# obstacles_info_data_source = ColumnDataSource(data={'L': [], 'S': []})

fig2.circle_dot('L', 'S', source = sampled_points_data_source, size=4, color='red', legend_label = "samped lanes")
fig2.circle_dot('L', 'S', source = update_sampled_points_data_source, size=3, color='green', legend_label = "update samped lanes")
fig2.line('L', 'S', source = fined_path_point_data_source, line_width=3, line_color='orange', line_dash = 'dashed',legend_label = "fined path")
fig2.line('L', 'S', source = update_fined_path_point_data_source, line_width=3, line_color='blue', line_dash = 'dashed',legend_label = "update fined path")
# fig2.patch('L', 'S', source=obstacles_info_data_source, fill_alpha=0.5, fill_color='black', line_color='black',legend_label = "obstacles")
# table_info_name= ["ego_s","ego_l","ego_v","dp_cost_time"]
table_info =  ColumnDataSource(data = {'name':[], 'info':[]})
info_column = [
  TableColumn(field="name", title="name"),
  TableColumn(field="info", title="info")
]
tab2 = DataTable(source = table_info, columns = info_column, width = 400, height = 400)

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    # self.enable_pybind = ipywidgets.Checkbox(description= "enable_pybind", value=True)
    # self.update_param = ipywidgets.Checkbox(description= "update_param", value=False)
    # self.update_v_cruise = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "v_cruise",min=0.0, max=100.0, value=v_cruise, step=frame_dt)
    self.update_s_range = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "s_range",min=1.0, max=50.0, value=s_range, step=frame_dt)
    self.update_l_range = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "l_range",min=0.1, max=4.0, value=l_range, step=frame_dt)
    self.update_coeff_l_cost =ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "coeff_l_cost",min=0.1, max=1000.0, value=coeff_l_cost, step=frame_dt)
    self.update_coeff_dl_cost =ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "coeff_dl_cost",min=0.0, max=1000.0, value=coeff_dl_cost, step=frame_dt)
    self.update_coeff_ddl_cost =ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "coeff_ddl_cost",min=0.0, max=1000.0, value=coeff_ddl_cost, step=frame_dt)
    self.update_path_resolution =ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "path_resolution",min=0.1, max=5.0, value=path_resolution, step= frame_dt)
    self.update_coeff_end_l_cost =ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "coeff_end_l_cost",min=0.1, max=1000.0, value=coeff_end_l_cost, step=frame_dt)
    self.update_coeff_collision_cost =ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "coeff_collision_cost",min=0.1, max=1000.0, value=coeff_collision_cost, step=frame_dt)
    self.update_collision_distance =ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "collision_distance",min=0.1, max=5.0, value=collision_distance, step= frame_dt)

    ipywidgets.interact(slider_callback,
                        bag_time = self.time_slider,
                        update_s_range = self.update_s_range,
                        update_l_range = self.update_l_range,
                        update_coeff_l_cost = self.update_coeff_l_cost,
                        update_coeff_dl_cost = self.update_coeff_dl_cost,
                        update_coeff_ddl_cost = self.update_coeff_ddl_cost,
                        update_coeff_end_l_cost = self.update_coeff_end_l_cost,
                        update_coeff_collision_cost = self.update_coeff_collision_cost,
                        update_path_resolution = self.update_path_resolution,
                        update_collision_distance = self.update_collision_distance)

def plot_sample_lane_info(dp_road_debug_msg):
  ego_s = dp_road_debug_msg.print_info.ego_s
  sampled_points = dp_road_debug_msg.sample_lanes_info.sampled_points
  if not sampled_points:
    print("Warning: No sampled points found in dp_road_debug_msg!")
    return
  sampled_s_values = []
  sampled_l_values = []
  for level_points in sampled_points:
      # 遍历 level_points（SampleSLPoint 列表）
      for point in level_points.level_points:
          sampled_s_values.append(point.s)
          sampled_l_values.append(point.l)
  sampled_points_data_source.data.update({'L': sampled_l_values, 'S': sampled_s_values})
def plot_static_obstacles(dp_road_debug_msg):
  obstacles_info = dp_road_debug_msg.obstacles_info
  if not obstacles_info:
    print("Warning: No obstacles_info found in dp_road_debug_msg!")
    return

  for i,obs in enumerate(obstacles_info):
    s_start, s_end = obs.s_start, obs.s_end
    l_start, l_end = obs.l_start, obs.l_end
    # 创建障碍物的四个点
    s_points = [s_start, s_start, s_end, s_end]
    l_points = [l_start, l_end, l_end, l_start]
    if i >= max_objects:
       continue
    obstacles_info_data_sources[i].data.update({'L':l_points, 'S':s_points})

def plot_dp_path(dp_road_debug_msg):
  fined_points = dp_road_debug_msg.dp_result_path.fined_points
  fined_path_s_values = []
  fined_path_l_values = []
  for point in fined_points:
    fined_path_s_values.append(point.s)
    fined_path_l_values.append(point.l)
  fined_path_point_data_source.data.update({'L':fined_path_l_values,'S':fined_path_s_values})

def plot_update_sample_points(sample_points_vec):
  sampled_s_values = []
  sampled_l_values = []
  for point in sample_points_vec:
      sampled_s_values.append(point[0])
      sampled_l_values.append(point[1])
  update_sampled_points_data_source.data.update({'L': sampled_l_values, 'S': sampled_s_values})

def plot_update_dp_path(path_vec):
  path_s_values = []
  path_l_values = []
  for point in path_vec:
      path_s_values.append(point[0])
      path_l_values.append(point[1])
  update_fined_path_point_data_source.data.update({'L': path_l_values, 'S': path_s_values})
def extract_fields_to_table(message, table_name, table_value):
    for field, value in message.ListFields():
        if field.message_type:  # if proto defined message
            table_name.append(field.name)
            table_value.append(None)
            extract_fields_to_table(value, table_name, table_value)
        else:
            table_name.append(field.name)
            table_value.append(value)

def extract_fields_with_names(proto, prefix="DP"):
    table_name = []
    table_value = []

    for field in proto.DESCRIPTOR.fields:
        field_name = field.name
        full_name = f"{prefix}.{field_name}" if prefix else field_name

        value = getattr(proto, field_name)
        if field.type == FieldDescriptor.TYPE_MESSAGE:
            if value.ListFields():  #
                sub_table_name, sub_table_value = extract_fields_with_names(value, full_name)
                table_name.extend(sub_table_name)
                table_value.extend(sub_table_value)
            else:
                table_name.append(full_name)
                table_value.append(None)
        else:
            # 普通字段直接添加
            table_name.append(full_name)
            table_value.append(value)

    return table_name, table_value

def plot_table_info(sample_print_table_info):
  table_name, table_var = extract_fields_with_names(sample_print_table_info)
  table_info.data.update({'name':table_name, 'info':table_var})

def ClearFig():
  sampled_points_data_source.data.update({'L':[], 'S':[]})
  update_sampled_points_data_source.data.update({'L':[], 'S':[]})
  update_fined_path_point_data_source.data.update({'L':[], 'S':[]})
  fined_path_point_data_source.data.update({'L':[], 'S':[]})

  for i, obs in enumerate(obstacles_info_data_sources):
    obstacles_info_data_sources[i].data.update({'L':[], 'S':[]})


def slider_callback(bag_time,
                    update_s_range,
                    update_l_range,
                    update_coeff_l_cost,
                    update_coeff_dl_cost,
                    update_coeff_ddl_cost,
                   update_coeff_end_l_cost,
                   update_coeff_collision_cost,
                   update_path_resolution,
                   update_collision_distance):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)

  plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
  dp_road_info_debug_msg = plan_debug_msg.dp_road_info
  # print(local_view_data['data_msg']['plan_debug_msg'].dp_road_info.obstacles_info)
  ClearFig()
  plot_sample_lane_info(dp_road_info_debug_msg)
  plot_dp_path(dp_road_info_debug_msg)
  plot_static_obstacles(dp_road_info_debug_msg)
  plot_table_info(dp_road_info_debug_msg.print_info)
  # dp_road_info_debug_msg_string = dp_road_info_debug_msg.SerializeToString()
  # dp_path_decider_py.InputProcess(dp_road_info_debug_msg_string) # input from logs
  # dp_path_decider_py.UpdateParams( update_s_range,
  #                                 update_l_range,
  #                                 update_coeff_l_cost,
  #                                 update_coeff_dl_cost,
  #                                 update_coeff_ddl_cost,
  #                                 update_coeff_end_l_cost,
  #                                 update_coeff_collision_cost,
  #                                 update_path_resolution,
  #                                 update_collision_distance)
  # dp_path_decider_py.Execute()
  # sampled_points = dp_path_decider_py.GetSamplePoints()
  # dp_path = dp_path_decider_py.GetFinedDPPath()
  # # print("cost time : ",dp_road_debug_msg.dp_cost_time)
  # print("bag s_range ",dp_road_info_debug_msg.s_range,"update l_range ",update_s_range )

  # # searalized_table_string = dp_path_decider_py.get_print_table_string()
  # # update_table_info = dp_road_graph_pb2.PrintInfo() # ?
  # # update_table_info.ParseFromString(searalized_table_string)

  # plot_update_sample_points(sampled_points)
  # plot_update_dp_path(dp_path)
  push_notebook()

bkp.show(row(fig1,fig2, tab2), notebook_handle = True)
slider_class = LocalViewSlider(slider_callback)


