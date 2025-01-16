import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_local_view import *
from lib.load_struct import *
from lib.load_rotate import *
from lib.load_json import *
from lib.load_ros_bag import is_match_planning, is_bag_main
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

from jupyter_pybind import sample_poly_speed_adjust_decider_py
from python_proto import st_search_decider_pb2
# load bag info
car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()

# +
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_10034_EVENT_MANUAL_2024-12-30-15-16-04.bag_2025-01-02-15-11-39.PP"
bag_path = "/root/code/planning/data_collection_CHERY_E0Y_10034_EVENT_MANUAL_2024-12-30-17-34-01.bag_2025-01-02-15-19-15.PP.1736149504.open-loop.noa.plan"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_10034_EVENT_MANUAL_2024-12-30-15-16-04.bag_2025-01-02-15-11-39.PP.1736149685.open-loop.noa.plan"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_10034_EVENT_MANUAL_2024-12-30-14-46-25.bag_2025-01-02-15-06-20.PP.1736162075.close-loop.noa.plan"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-18-10-35-40.bag_2024-10-21-16-16-11.PP.1736234394.close-loop.scc.plan"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-21-30.bag_2024-10-18-11-00-55.PP"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-21-30.bag_2024-10-18-11-00-55.PP.1736236060.close-loop.scc.plan" #汇流区变道，加入减速逻辑

bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-24-44.bag_2024-10-18-10-56-02.PP.1736236314.close-loop.scc.plan" #汇流区变道，加入减速逻辑
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-26-27.bag_2024-10-18-10-49-32.PP.1736236490.close-loop.scc.plan"#汇流区变道，加入减速逻辑
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-28-08.bag_2024-10-18-10-24-48.PP.1736236617.close-loop.scc.plan"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-32-33.bag_2024-10-18-10-19-17.PP" # 汇流区有加速倾向，加入减速逻辑

bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-15-39-46.bag_2024-10-17-21-10-45.PP.1736237217.close-loop.noa.plan"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_48160_EVENT_MANUAL_2024-09-24-14-48-23.bag_2024-09-25-09-47-35.PP.1736237366.close-loop.scc.plan"# 汇流区减速不足

bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-32-33.bag_2024-10-18-10-19-17.PP.1736238081.close-loop.scc.plan"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-32-33.bag_2024-10-18-10-19-17.PP.1736239958.close-loop.scc.plan"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-32-33.bag_2024-10-18-10-19-17.PP.1736240478.close-loop.scc.plan"

bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_48160_EVENT_MANUAL_2024-09-24-14-48-23.bag_2024-09-25-09-47-35.PP.1736240663.close-loop.scc.plan"

bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-15-39-46.bag_2024-10-17-21-10-45.PP.1736240807.close-loop.noa.plan"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-26-27.bag_2024-10-18-10-49-32.PP"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-10-17-16-26-27.bag_2024-10-18-10-49-32.PP.1736325728.close-loop.scc.plan"
bag_path = "/root/bags/speed_decider/data_collection_CHERY_E0Y_10034_EVENT_MANUAL_2024-10-21-15-39-25.bag_2024-10-22-11-41-57.PP.1736327152.close-loop.noa.plan"

bag_path = "/root/code/bags/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-12-24-15-19-46.bag_2024-12-26-17-12-06.PP"
frame_dt = 0.1
# -

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()
fig1.height = 1550

for i, debug_msg in enumerate(bag_loader.plan_debug_msg['data']):
  sample_poly_speed_info =  debug_msg.st_search_decider_info.sample_poly_speed_info
  if hasattr(sample_poly_speed_info, 'sample_param'):
    weight_stop_line0 =sample_poly_speed_info.sample_param.weight_stop_line
    weight_follow_vel0 =sample_poly_speed_info.sample_param.weight_follow_vel
    weight_match_gap_s0 =sample_poly_speed_info.sample_param.weight_match_gap_s
    weight_match_gap_vel0 =sample_poly_speed_info.sample_param.weight_match_gap_vel
    weight_leading_safe_s0 =sample_poly_speed_info.sample_param.weight_leading_safe_s
    weight_leading_safe_v0 =sample_poly_speed_info.sample_param.weight_leading_safe_v
    weight_vel_variable0 = sample_poly_speed_info.sample_param.weight_vel_variable
    weight_gap_avaliable0 = sample_poly_speed_info.sample_param.weight_gap_avaliable
    break
# init pybind
sample_poly_speed_adjust_decider_py.Init()

# def update_sample_speed_decider_data(bag_loader, bag_data):
fig2 = bkp.figure(x_axis_label='t', y_axis_label='s', width=800, height=600,x_range=(-1, 6), y_range=(-10, 150), match_aspect = True, aspect_scale=1)
fig3 = bkp.figure(x_axis_label='t', y_axis_label='v', width=600, height=400,x_range=(-1, 6), y_range=(-10, 45), match_aspect = True, aspect_scale=1)
fig4 = bkp.figure(x_axis_label='t', y_axis_label='a', width=600, height=400,x_range=(-1, 6), y_range=(-10, 20), match_aspect = True, aspect_scale=1)
fig5 = bkp.figure(x_axis_label='t', y_axis_label='j', width=600, height=400,x_range=(-1, 6), y_range=(-10, 10), match_aspect = True, aspect_scale=1)

#reserve
max_objects = 20

obj_data_sources = [ColumnDataSource(data={'t': [], 's': []}) for _ in range(max_objects)]
lines = [
    fig2.line(
        't', 's',
        source=source,
        line_width=2,
        line_color='blue',
        legend_label=f"obj_{i}")
    for i, source in enumerate(obj_data_sources)
]

leading_data_source = ColumnDataSource(data={'t': [], 's': []})
fig2.line('t', 's', source = leading_data_source, line_width=2, line_color='red', line_dash='dashed', legend_label = "leading veh obj")

update_traj_source = ColumnDataSource(data={'t': [], 's': []})
fig2.line('t', 's', source = update_traj_source, line_width=2, line_color='red',  legend_label = "update traj")

origin_min_cost_traj_source = ColumnDataSource(data={'t': [], 's': []})
fig2.line('t', 's', source = origin_min_cost_traj_source, line_width=2, line_color='green', line_dash='solid', line_alpha=0.7, legend_label='origin min cost traj')

stitched_last_traj_source = ColumnDataSource(data={'t': [], 's': []})
fig2.line('t', 's', source = stitched_last_traj_source, line_width=2, line_color='yellow', line_dash='solid', line_alpha=0.7, legend_label='stitched last min cost traj')

ego_init_v_traj_source = ColumnDataSource(data={'t': [], 's': []})
fig2.line('t', 's', source = ego_init_v_traj_source, line_width=2, line_color='green', line_dash='dashed', line_alpha=0.7, legend_label='ego_init_v_traj')

origin_v_min_cost_traj_source = ColumnDataSource(data={'t': [], 's': []})
fig3.line('t', 's', source = origin_v_min_cost_traj_source, line_width=2, line_color='green', line_dash='solid', line_alpha=0.7, legend_label='origin_v')

origin_a_min_cost_traj_source = ColumnDataSource(data={'t': [], 's': []})
fig4.line('t', 's', source = origin_a_min_cost_traj_source, line_width=2, line_color='green', line_dash='solid', line_alpha=0.7, legend_label='origin_a')

origin_j_min_cost_traj_source = ColumnDataSource(data={'t': [], 's': []})
fig5.line('t', 's', source = origin_j_min_cost_traj_source, line_width=2, line_color='green', line_dash='solid', line_alpha=0.7, legend_label='origin_j')

ego_v_keep_traj_source = ColumnDataSource(data={'t': [], 's': []})
fig3.line('t', 's', source = ego_v_keep_traj_source, line_width=2, line_color='green', line_dash='dashed', line_alpha=0.7, legend_label='v_cruise')

table_info_name= ["match_gap_cost", "follow_vel_cost", "vel_bound_cost", "stop_line_cost", "acc_bound_cost", "jerk_cost", "leading_veh_safe_cost", "vel_variable_cost", "end_s", \
                  "end_v", "ego_v", "v_suggested", "target_lane_objs_flow_vel", "traffic_density","leading_veh_id", "sample status", "sample_scene","count_normal_to_hover_state", "count_hover_to_normal_state",\
                  "is_nearing_ramp","merge_emegency_distance", "is_in_merge_region", "distance_to_road_merge", "distance_to_road_split","stitched_match_front_gap_id","stitched_match_back_gap_id",\
                   "current_match_front_gap_id", "current_match_back_gap_id"
                    ]
table_info =  ColumnDataSource(data = {'name':[], 'info':[]})
info_column = [
  TableColumn(field="name", title="name"),
  TableColumn(field="info", title="info")
]
tab2 = DataTable(source = table_info, columns = info_column, width = 300, height = 600)

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.enable_pybind = ipywidgets.Checkbox(description= "enable_pybind", value=True)
    self.update_param = ipywidgets.Checkbox(description= "update_param", value=False)
    self.update_suggested_v = ipywidgets.Checkbox(description= "update_suggested_v", value=False)
    self.weight_follow_vel = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "w_follow_vel",min=0.0, max=100.0, value=weight_follow_vel0, step=frame_dt)
    self.weight_match_gap_s = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "w_match_gap_s",min=0.0, max=100.0, value=weight_match_gap_s0, step=frame_dt)
    self.weight_match_gap_vel = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "w_match_gap_vel",min=0.0, max=100.0, value=weight_match_gap_vel0, step=frame_dt)
    self.weight_stop_line = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "w_stop_line",min=0.0, max=100.0, value=weight_stop_line0, step=frame_dt)
    self.weight_leading_safe_s = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "w_leading_safe_s",min=0.0, max=100.0, value=weight_leading_safe_s0, step=frame_dt)
    self.weight_leading_safe_v = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "w_leading_safe_v",min=0.0, max=100.0, value=weight_leading_safe_v0, step=frame_dt)
    self.weight_vel_variable = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "w_vel_variable",min=0.0, max=100.0, value=weight_vel_variable0, step=frame_dt)
    self.weight_gap_avaliable = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "weight_gap_avaliable",min=0.0, max=100.0, value=weight_gap_avaliable0, step=frame_dt)
    self.set_suggested_v = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "set_suggested_v",min=0.0, max=100.0, value=15.0, step=frame_dt)

    ipywidgets.interact(slider_callback, enable_pybind = self.enable_pybind, update_param = self.update_param, update_suggested_v = self.update_suggested_v,\
                        bag_time = self.time_slider, weight_follow_vel = self.weight_follow_vel, \
                        weight_match_gap_s = self.weight_match_gap_s, weight_match_gap_vel = self.weight_match_gap_vel,\
                        weight_stop_line = self.weight_stop_line, weight_leading_safe_s = self.weight_leading_safe_s,weight_leading_safe_v = self.weight_leading_safe_v, weight_vel_variable = self.weight_vel_variable,\
                        weight_gap_avaliable = self.weight_gap_avaliable,\
                        set_suggested_v = self.set_suggested_v
                                    )

def plot_env_info(sample_poly_debug_msg):
  ego_s = sample_poly_debug_msg.sample_print_table_info.ego_s
  agent_input = sample_poly_debug_msg.agent_infos
  agent_nums = len(agent_input)
  data_st_objs = [ColumnDataSource(data={'t': [], 's': []}) for _ in range(agent_nums)]
  print("agnet info:\n", agent_input)
  t_values = [0.0, 5.0, 5.0, 0.0, 0.0]
  for i, agent in enumerate(agent_input):
    s = [ego_s + agent.center_s - agent.half_length, ego_s + agent.center_s - agent.half_length + 5 * agent.v,
         ego_s + agent.center_s + agent.half_length + 5 * agent.v, ego_s + agent.center_s + agent.half_length,
         ego_s + agent.center_s - agent.half_length]
    obj_data_sources[i].data.update({'t':t_values, 's':s})

  leading_veh_intput = sample_poly_debug_msg.leading_veh_info
  if (hasattr(sample_poly_debug_msg, 'leading_veh_info') and leading_veh_intput.id !=-1 and leading_veh_intput.id !=-400 ):
    s_leading_veh = [ego_s + leading_veh_intput.center_s - leading_veh_intput.half_length, ego_s + leading_veh_intput.center_s - leading_veh_intput.half_length + 5 * leading_veh_intput.v,
                     ego_s + leading_veh_intput.center_s + leading_veh_intput.half_length + 5 *leading_veh_intput.v, \
                     ego_s + leading_veh_intput.center_s + leading_veh_intput.half_length, ego_s + leading_veh_intput.center_s - leading_veh_intput.half_length]
    leading_data_source.data.update({'t': t_values, 's':s_leading_veh})

def plot_sample_traj_info(sample_poly_debug_msg):
  origin_s_vec = sample_poly_debug_msg.sample_s_vec
  origin_v_vec = sample_poly_debug_msg.sample_v_vec
  origin_a_vec = sample_poly_debug_msg.sample_a_vec
  origin_j_vec = sample_poly_debug_msg.sample_j_vec
  origin_t_vec = sample_poly_debug_msg.sample_t_vec
  stitched_s_vec = sample_poly_debug_msg.stitched_sample_s_vec
  stitched_t_vec = sample_poly_debug_msg.stitched_sample_t_vec
  print("stitched s vec: ", stitched_s_vec)
  print("origin s vec: ", origin_s_vec)

  origin_s_values = []
  origin_v_values = []
  origin_a_values = []
  origin_j_values = []
  origin_t_values = []
  ego_v = sample_poly_debug_msg.sample_print_table_info.ego_v
  ego_s = sample_poly_debug_msg.sample_print_table_info.ego_s
  for i in range(len(origin_s_vec)):
    origin_s_values.append(origin_s_vec[i]+ ego_s)
    origin_v_values.append(origin_v_vec[i])
    origin_a_values.append(origin_a_vec[i])
    origin_j_values.append(origin_j_vec[i])
    origin_t_values.append(origin_t_vec[i])

  ego_v_values = []
  ego_t_values = []
  ego_v_values.append(ego_v)
  ego_v_values.append(ego_v)
  ego_t_values.append(0.0)
  ego_t_values.append(5.0)

  ego_s_initv_values = []
  ego_t_initv_values = []
  ego_s_initv_values.append(ego_s)
  ego_s_initv_values.append(ego_s + ego_v * 5.0)
  ego_t_initv_values.append(0.0)
  ego_t_initv_values.append(5.0)

  stiched_s_values = []
  stiched_t_valuse = []
  for i in range(len(stitched_s_vec)):
    stiched_s_values.append(stitched_s_vec[i]+ ego_s)
    stiched_t_valuse.append(stitched_t_vec[i])

  origin_min_cost_traj_source.data.update({'t': origin_t_values, 's': origin_s_values})
  stitched_last_traj_source.data.update({'t':stiched_t_valuse, 's':stiched_s_values})
  ego_init_v_traj_source.data.update({'t': ego_t_initv_values, 's': ego_s_initv_values})
  origin_v_min_cost_traj_source.data.update({'t': origin_t_values, 's': origin_v_values})
  origin_a_min_cost_traj_source.data.update({'t': origin_t_values, 's': origin_a_values})
  origin_j_min_cost_traj_source.data.update({'t': origin_t_values, 's': origin_j_values})
  ego_v_keep_traj_source.data.update({'t': ego_t_values, 's':ego_v_values})

# def plot_table_info(sample_poly_debug_msg):


def plot_execute_result(min_cost_traj):
  s_candidate_values = []
  v_candidate_values = []
  t_candidate_values = []
  t_candidate_values = [traj_point[0] for traj_point in min_cost_traj]
  s_candidate_values = [traj_point[1] for traj_point in min_cost_traj]
  v_candidate_values = [traj_point[2] for traj_point in min_cost_traj]
  update_traj_source.data.update({'t':t_candidate_values, 's':s_candidate_values})

def extract_fields_to_table(message, table_name, table_value):
    for field, value in message.ListFields():
        if field.message_type:  # if proto defined message
            table_name.append(field.name)
            table_value.append(None)
            extract_fields_to_table(value, table_name, table_value)
        else:
            table_name.append(field.name)
            table_value.append(value)

def extract_fields_with_names(proto, prefix=""):
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
  update_traj_source.data.update({'t':[], 's':[]})
  for i, agent in enumerate(obj_data_sources):
    obj_data_sources[i].data.update({'t':[], 's':[]})
  leading_data_source.data.update({'t':[], 's':[]})
  ego_init_v_traj_source.data.update({'t':[], 's':[]})


def slider_callback(enable_pybind, bag_time, update_param, weight_follow_vel,  \
                   weight_match_gap_s, weight_match_gap_vel, weight_stop_line, weight_leading_safe_s, weight_leading_safe_v,\
                    weight_vel_variable, weight_gap_avaliable):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)

  plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
  # print("st search decider info: ", plan_debug_msg.st_search_decider_info)
  sample_poly_debug_msg = plan_debug_msg.st_search_decider_info.sample_poly_speed_info

  ClearFig()
  plot_env_info(sample_poly_debug_msg)
  plot_sample_traj_info(sample_poly_debug_msg)
  plot_table_info(sample_poly_debug_msg.sample_print_table_info)

  sample_poly_debug_msg_string = sample_poly_debug_msg.SerializeToString()

  if (enable_pybind):
    if(sample_poly_speed_adjust_decider_py.ProcessInput(sample_poly_debug_msg_string)):
      if(update_param):
        sample_poly_speed_adjust_decider_py.UpdateParam(weight_follow_vel,weight_match_gap_s, \
                                                        weight_match_gap_vel, weight_stop_line, weight_leading_safe_s, \
                                                        weight_leading_safe_v, weight_vel_variable, weight_gap_avaliable)

        sample_poly_speed_adjust_decider_py.Execute()

        min_cost_traj = sample_poly_speed_adjust_decider_py.GetMinCostTraj()
        searalized_table_string = sample_poly_speed_adjust_decider_py.get_print_table_string()
        update_table_info = st_search_decider_pb2.SamplePrintTableInfo()
        update_table_info.ParseFromString(searalized_table_string)

        plot_execute_result(min_cost_traj)
        plot_table_info(update_table_info)
    else:
      print("no lane change!")
  push_notebook()

bkp.show(row(fig1,column(fig2, fig3, tab2), column(fig4, fig5)), notebook_handle = True)
slider_class = LocalViewSlider(slider_callback)


