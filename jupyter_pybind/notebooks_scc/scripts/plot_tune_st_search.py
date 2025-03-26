import sys, os
sys.path.append("..")

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
from bokeh.models import ColumnDataSource, HoverTool
import bokeh.plotting as bkp
from bokeh.plotting import show
from bokeh.models import WheelZoomTool, HoverTool, TapTool, CustomJS
from bokeh.models import DataTable, DateFormatter, TableColumn
from bokeh.models import TextInput
from cyber_record.record import Record
from google.protobuf.json_format import MessageToJson
import rosbag
from bokeh.resources import INLINE
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')
sys.path.append('../../python_proto')

from jupyter_pybind import st_search_py

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

# column data st source
data_st_boundary_1 = ColumnDataSource(data = {'s':[], 't':[]})
data_st_boundary_2 = ColumnDataSource(data = {'s':[], 't':[]})
data_st_boundary_3 = ColumnDataSource(data = {'s':[], 't':[]})
data_st_boundary_4 = ColumnDataSource(data = {'s':[], 't':[]})
data_st_boundary_5 = ColumnDataSource(data = {'s':[], 't':[]})
data_st_boundary_6 = ColumnDataSource(data = {'s':[], 't':[]})
data_st_boundary_7 = ColumnDataSource(data = {'s':[], 't':[]})

data_interval_all_points = ColumnDataSource(data = {'s':[], 't':[]})
data_heuristic_s_list_first = ColumnDataSource(data = {'s':[], 't':[]})
data_heuristic_s_list_second = ColumnDataSource(data = {'s':[], 't':[]})
data_heuristic_s_list_third= ColumnDataSource(data = {'s':[], 't':[]})


data_ego_init_state = ColumnDataSource(data = {'s':[], 'v':[], 'a':[], 't':[]})
data_heruistic_state = ColumnDataSource(data = {'s':[], 't':[]}) # appending...
data_heuristic_s_list = ColumnDataSource(data = {'s':[], 't':[]} )
data_nodes_list = ColumnDataSource(data = {'s':[], 't':[],'v':[], 'vel_cost':[], 'aligned_v':[],\
                                            's_dis_cost':[],
                                           'acc_cost':[],
                                           'jerk_cost':[],
                                           'h_s_cost':[],
                                           'h_v_cost':[],
                                           'match_slot_list':[],
                                   'h_cost':[], 'g_cost':[] })

data_search_res = ColumnDataSource(data = {'s':[], 't':[]}) # appending....
data_stitch_res = ColumnDataSource(data = {'s':[], 't':[]}) # appending....




#fig plot
fig1 = bkp.figure(x_axis_label='t', y_axis_label='s', width=950, height=840,x_range=(-1, 6), y_range=(-50, 170), match_aspect = True, aspect_scale=1)

fig1.circle('t','s', source = data_ego_init_state, size=8, color='grey', legend_label = 'ego init point')
# fig1.circle('s','t', source = data_heruistic_state, size=8, color='red', legend_label = 'target state ')

fig1.line('t','s', source=data_heuristic_s_list, line_width=2, line_color='green', line_dash='dashed', line_alpha=0.7, legend_label='target s')

fig1.circle('t','s', source = data_nodes_list, size=8, color='blue', legend_label = 'search nodes')
hover_nodes = HoverTool(tooltips =[("@match_slot_list, @aligned_v", "@match_slot_list, @aligned_v"),("@s_dis_cost","@s_dis_cost"),("@v","@v"),("(vel_cost, acc_cost, jerk_cost)", "(@vel_cost, @acc_cost, @jerk_cost)"),("(h_s_cost, h_v_cost)", "(@h_s_cost, @h_v_cost)")\
  ,("(h_cost, g_cost)", "(@h_cost, @g_cost)")] )
fig1.add_tools(hover_nodes)

fig1.line('t','s', source=data_search_res, line_width=2, line_color='green', line_dash='solid', line_alpha=0.7, legend_label='search path')
fig1.circle("t", 's',  source = data_search_res, size=6, color='green', legend_label = 'search path points')

fig1.line('t','s', source=data_stitch_res, line_width=2, line_color='green', line_dash='dashed', line_alpha=0.7, legend_label='stitched path')

fig1.circle('t','s', source=data_heuristic_s_list_first, size=6, color='black', legend_label='1 aligned s')
fig1.circle('t','s', source=data_heuristic_s_list_second, size=6, color='black',legend_label='2 aligned s')
fig1.circle('t','s', source=data_heuristic_s_list_third, size=6, color='black', legend_label='3 aligned s')



fig1.circle("t", 's',  source = data_interval_all_points, size=4, color='grey', legend_label = 'interval edge')

try:
  fig1.line('s','t', source=data_st_boundary_1, line_width=2, line_color='red', line_dash='solid', line_alpha=0.7, legend_label='obj-1')
  # fig1.patches('t', 's', source = data_st_boundary_1, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'data_st_boundary_1')
except:
  print("no 1 st boundary")
try:
  fig1.line('s', 't', source=data_st_boundary_2, line_width=2, line_color='red', line_dash='solid', line_alpha=0.7, legend_label='obj-2')
  # fig1.patches('t', 's', source = data_st_boundary_2, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'data_st_boundary_2')
except:
  print("no 2 st boundary")
try:
  fig1.line('s', 't', source=data_st_boundary_3, line_width=2, line_color='red', line_dash='solid', line_alpha=0.7, legend_label='obj-3')
except:
  print("no 3 st boundary")
try:
  fig1.line('s', 't', source=data_st_boundary_4, line_width=2, line_color='red', line_dash='solid', line_alpha=0.7, legend_label='obj-4')
except:
  print("no 4 st boundary")
try:
  fig1.line('s', 't', source=data_st_boundary_5, line_width=2, line_color='red', line_dash='solid', line_alpha=0.7, legend_label='obj-5')
except:
  print("no 5 st boundary")
try:
  fig1.line('s', 't', source=data_st_boundary_6, line_width=2, line_color='red', line_dash='solid', line_alpha=0.7, legend_label='obj-6')
except:
  print("no 6 st boundary")
try:
  fig1.line('s', 't', source=data_st_boundary_7, line_width=2, line_color='red', line_dash='solid', line_alpha=0.7, legend_label='obj-7')
except:
  print("no 7 st boundary")

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'
st_search_py.Init()

class LocalViewSlider:
    def __init__(self, slider_callback):
        self.show_print = ipywidgets.Checkbox(
            description="show_print", value=False)
        self.fix_result = ipywidgets.Checkbox(layout=ipywidgets.Layout(
           width='40%', height = '10%',display=''),
            description="fix_result", value=False)
        self.clear_objs = ipywidgets.Checkbox(layout=ipywidgets.Layout(
           width='40%', height = '10%',display=''),
            description="clear_objs", value=False)
        self.fix_objs = ipywidgets.Checkbox(layout=ipywidgets.Layout(
           width='40%', height = '10%',display=''),description="fix_objs", value=False)
        self.use_default_obj = ipywidgets.Checkbox(layout=ipywidgets.Layout(
           width='40%', height = '10%',display=''),description="use_default_obj", value=True)
        self.adjust_all_obj= ipywidgets.Checkbox(layout=ipywidgets.Layout(
           width='40%', height = '10%',display=''),description="adjust_all_obj", value=False)
        self.use_stop_line = ipywidgets.Checkbox(layout=ipywidgets.Layout(
           width='40%', height = '10%',display=''),description="use_stop_line", value=True)
        # st search config
        self.max_acc_limit = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="max_acc_limit", min=0.0, max=8.0, value=4.0, step=0.1)
        self.min_acc_limit = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="min_acc_limit", min=-8.0, max=0.0, value=-4.0, step=0.1)
        self.max_jerk_limit = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="max_jerk_limit", min=0.0, max=8.0, value=6.0, step=0.1)
        self.min_jerk_limit = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="min_jerk_limit", min=-8.0, max=0.0, value=-6.0, step=0.1)
        self.speed_limit = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="speed_limit", min=0.0, max=45.0, value=25.0, step=0.1)
        self.speed_limit_scale = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="speed_limit_scale", min=0.0, max=2, value=1.2, step=0.1)
        self.v_cruise = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="v_cruise", min=0.0, max=40.0, value=38.2, step=0.1)
        self.v_min = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="v_min", min=0.0, max=40.0, value=13.8, step=0.1)
        self.collision_ttc = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="collision_ttc", min=0.0, max=8.0, value=2.0, step=0.1)
        self.min_collision_dist = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="min_collision_dist", min=0.0, max=20.0, value=5.0, step=0.1)
        self.max_collision_dist = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="max_collision_dist", min=0.0, max=20.0, value=10.0, step=0.1)
        self.s_step= ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="s_step", min=0.0, max=4.0, value=0.2, step=0.02)
        self.t_step= ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="t_step", min=0.0, max=1.0, value=0.2, step=0.01)
        self.vel_step= ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="vel_step", min=0.0, max=1.0, value=0.2, step=0.01)
        self.acc_search_step = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="acc_search_step", min=0.0, max=5.0, value=1.0, step=0.02)
        self.max_search_time = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="max_search_time", min=0.0, max=20.0, value=20.0, step=0.1)
        self.acc_search_max = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="acc_search_max", min=0.0, max=8.0, value=4.0, step=0.1)
        self.acc_search_min = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="acc_search_min", min=-8.0, max=0.0, value=-4.0, step=0.1)
        self.vel_tolerance = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="vel_tolerance", min=0.0, max=5.0, value=3.0, step=0.1)
        self.propoper_accel_value = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="propoper_accel_value", min=0.0, max=5.0, value=2.0, step=0.1)
        self.planning_time_horizon = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="planning_time_horizon", min=0.0, max=5.0, value=5.0, step=0.1)
        self.rel_ego_stop_s = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="rel_ego_stop_s", min=0.0, max=800, value=300.0, step=0.1)
        self.stop_line_first_s =ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="stop_line_first_s", min=0.0, max=150, value=15.0, step=0.1)
        self.stop_line_second_s =ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="stop_line_second_s", min=0.0, max=150, value=25.0, step=0.1)
        self.two_obj_rel_dis =ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="two_obj_rel_dis", min=0.0, max=150, value=35.0, step=0.1)
        self.max_count = ipywidgets.IntSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="max_count", min=1, max=50000, value=5000, step=1)
        # cost config
        self.yield_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="yield_weight", min=0.0, max=10.0, value=0.0, step=0.1)
        self.overtake_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="overtake_weight", min=0.0, max=10.0, value=0.0, step=0.1)
        self.vel_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='30%',height = '80%', display=''), description="vel_weight", visible=False ,min=-8.0, max=1000.0, value=0.0, step=0.1)
        self.accel_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='30%', height = '80%',display=''), description="accel_weight", min=0.0, max=1000.0, value=0.2, step=0.1)
        self.accel_sign_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="accel_sign_weight", min=0.0, max=10.0, value=0.0, step=0.1)
        self.jerk_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='30%', height = '80%',display=''), description="jerk_weight", min=0.0, max=1000.0, value=0.10, step=0.1)
        self.virtual_yield_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="virtual_yield_weight", min=0.0, max=10.0, value=0.0, step=0.1)
        self.length_t_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="length_t_weight", min=0.0, max=10.0, value=0.0, step=0.1)
        self.hcost_t_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='30%',height = '80%', display=''), description="hcost_t_weight", min=0.0, max=1000.0, value=0.0, step=0.1)
        self.hcost_s_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='30%', height = '80%',display=''), description="hcost_s_weight", min=0.0, max=1000.0, value=5.0, step=0.1)
        self.hcost_v_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='30%', height = '80%',display=''), description="hcost_v_weight", min=0.0, max=1000.0, value=15.0, step=0.1)
        self.upper_trancation_time_buffer = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="upper_trancation_time_buffer", min=0.0, max=5.0, value=0.0, step=0.1)
        self.lower_trancation_time_buffer = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="lower_trancation_time_buffer", min=0.0, max=5.0, value=0.0, step=0.1)
        self.min_upper_distance_buffer = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="min_upper_distance_buffer", min=0.0, max=20.0, value=15.0, step=0.1)
        self.min_lower_distance_buffer = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="min_lower_distance_buffer", min=0.0, max=20.0, value=15.0, step=0.1)
        self.comfort_gap_length = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="comfort_gap_length", min=0.0, max=100.0, value=15.0, step=0.1)
        self.violation_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="violation_weight", min=0.0, max=50.0, value=0.0, step=0.1)
        self.s_dis_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="s_dis_weight", min=0.0, max=50.0, value=0.4, step=0.1)
        #ego init state
        self.init_s = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="init_s", min=0.0, max=150, value=25.0, step=0.1)
        self.init_v = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="init_v", min=25.0, max=130.0, value=97.2, step=0.1)
        self.init_a = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="init_a", min=0.0, max=8.0, value=0.0, step=0.1)
        self.updated_heuristic_s = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="updated_heuristic_s", min=0.0, max=150, value=100.0, step=0.1)
        #obj info
        self.obj_id = ipywidgets.IntText(value=-1, description='obj_id:')
        self.obj_s = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="obj_s", min=-80, max=150, value=0.0, step=0.1)
        self.obj_v = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="obj_v", min=0.0, max=130, value=60.0, step=0.1)
        ipywidgets.interact(slider_callback,
                            show_print=self.show_print,
                            fix_result = self.fix_result,
                            clear_objs = self.clear_objs,
                            fix_objs =self.fix_objs,
                            use_default_obj = self.use_default_obj,
                            adjust_all_obj = self.adjust_all_obj,
                            #st search config
                            max_acc_limit = self.max_acc_limit,
                            min_acc_limit = self.min_acc_limit,
                            max_jerk_limit =  self.max_jerk_limit,
                            min_jerk_limit =self.min_jerk_limit,
                            speed_limit = self.speed_limit,
                            speed_limit_scale = self.speed_limit_scale,
                            v_cruise = self.v_cruise,
                            v_min =self.v_min,
                            collision_ttc = self.collision_ttc,
                            min_collision_dist = self.min_collision_dist,
                            max_collision_dist = self.max_collision_dist,
                            s_step= self.s_step,
                            t_step= self.t_step,
                            vel_step=self.vel_step,
                            acc_search_step = self.acc_search_step,
                            max_search_time = self.max_search_time,
                            acc_search_max =self.acc_search_max,
                            acc_search_min = self.acc_search_min,
                            vel_tolerance =self.vel_tolerance,
                            propoper_accel_value =self.propoper_accel_value,
                            planning_time_horizon = self.planning_time_horizon,
                            rel_ego_stop_s = self.rel_ego_stop_s,
                            use_stop_line = self.use_stop_line,
                            max_count = self.max_count,
                            # stop_line_first_s = self.stop_line_first_s,
                            two_obj_rel_dis= self.two_obj_rel_dis,
                            # stop_line_second_s =self.stop_line_second_s,
                            #cost config
                            yield_weight=self.yield_weight,
                            overtake_weight=self.overtake_weight,
                            vel_weight=self.vel_weight,
                            accel_weight=self.accel_weight,
                            accel_sign_weight=self.accel_sign_weight,
                            jerk_weight=self.jerk_weight,
                            virtual_yield_weight=self.virtual_yield_weight,
                            length_t_weight=self.length_t_weight,
                            hcost_t_weight=self.hcost_t_weight,
                            hcost_s_weight = self.hcost_s_weight,
                            hcost_v_weight = self.hcost_v_weight,
                            upper_trancation_time_buffer=self.upper_trancation_time_buffer,
                            lower_trancation_time_buffer=self.lower_trancation_time_buffer,
                            min_upper_distance_buffer=self.min_upper_distance_buffer,
                            min_lower_distance_buffer=self.min_lower_distance_buffer,
                            comfort_gap_length = self.comfort_gap_length,
                            violation_weight = self.violation_weight,
                            s_dis_weight = self.s_dis_weight,
                            #ego init state
                            init_s=self.init_s,
                            init_v=self.init_v,
                            init_a=self.init_a,
                            updated_heuristic_s=self.updated_heuristic_s,
                            #obj info
                            obj_id = self.obj_id,
                            obj_s = self.obj_s,
                            obj_v = self.obj_v
                            )
def ClearStFig():
  s = []
  t = []
  data_st_boundary_1.data.update({'s': s, 't': t})
  data_st_boundary_2.data.update({'s': s, 't': t})
  data_st_boundary_3.data.update({'s': s, 't': t})
  data_st_boundary_4.data.update({'s': s, 't': t})
  data_st_boundary_5.data.update({'s': s, 't': t})
  data_st_boundary_6.data.update({'s': s, 't': t})
  data_st_boundary_7.data.update({'s': s, 't': t})

  data_interval_all_points.data.update({'s':s, 't':t})

  data_ego_init_state.data.update({'s':[], 'v':[], 'a':[], 't':[]})
  data_heruistic_state.data.update({'s':[], 't':[]})
  data_search_res.data.update({'s':[], 't':[]})
  data_stitch_res.data.update({'s':[], 't':[]})
  data_heuristic_s_list.data.update({'s':[], 't':[]})
  data_nodes_list.data.update({'s': [], 't': [],'v':[], 'vel_cost': [], 'acc_cost': [],\
                                'jerk_cost':[], 'h_s_cost':[], 'h_v_cost':[],\
                                'match_slot_list':[], 'aligned_v':[], 's_dis_cost':[]})
  data_heuristic_s_list_first.data.update({'s':[], 't':[]})
  data_heuristic_s_list_second.data.update({'s':[], 't':[]})
  data_heuristic_s_list_third.data.update({'s':[], 't':[]})

def AddDefaultObjs():
  st_search_py.AddObj(5, 31.6, 120/3.4)
  st_search_py.AddObj(6, -52.6, 120.0/3.4)
  # st_search_py.AddObj(6, -15.0, 14.0)
  # st_search_py.AddObj(7, 15.0, 15.0)
  # st_search_py.AddObj(8, -25.0, 15.0)

def UseTwoObjs(obj_s, obj_v, two_obj_rel_dis):
  st_search_py.AddObj(5, obj_s, obj_v/3.4)
  st_search_py.AddObj(6, obj_s - two_obj_rel_dis, obj_v/3.4)

def UpdateIntervals(intervals):
  time_horizions = len(intervals)
  print("\n time horizion: ", time_horizions)
  s =[]
  t =[]
  for i in range(time_horizions):
    interval_size =len(intervals[i])
    for j in range(interval_size):
      s.append(intervals[i][j][1])
      t.append(intervals[i][j][0])
  # print("\n s:", s)
  # print("\n t:", t)

  data_interval_all_points.data.update({'s':s, 't':t})

def UpdateHeuristicS(heuristic_s_list):
  s=[]
  t=[]
  s1=[]
  s2=[]
  print("heuristic_s_list", heuristic_s_list)
  for i in range(len(heuristic_s_list)):
    for j in range(len(heuristic_s_list[i])):
      if i == 0:
        s.append(heuristic_s_list[i][j])
        t.append(0.2 * j)
      if i == 1 :
        s1.append(heuristic_s_list[i][j])
        t.append(0.2 * j)
      if i == 2 :
        s2.append(heuristic_s_list[i][j])
        t.append(0.2 * j)

  try:
    data_heuristic_s_list_first.data.update({'s':s, 't':t})
    print("\n s0:", s)
  except:
    data_heuristic_s_list_first.data.update({'s':[], 't':[]})

  try:
    data_heuristic_s_list_second.data.update({'s':s1, 't':t})
    print("\n s1:", s1)
  except:
    data_heuristic_s_list_second.data.update({'s':[], 't':[]})

  try:
    data_heuristic_s_list_third.data.update({'s':s2, 't':t})
    print("\n s2:", s2)
  except:
    data_heuristic_s_list_third.data.update({'s':[], 't':[]})


def slider_callback(show_print, fix_result,clear_objs,fix_objs,use_default_obj,adjust_all_obj, max_acc_limit,  min_acc_limit, max_jerk_limit,  min_jerk_limit,
                  speed_limit,  speed_limit_scale, v_cruise, v_min, collision_ttc,  min_collision_dist, max_collision_dist,
                  s_step,  t_step, vel_step, acc_search_step, max_search_time, acc_search_max,  acc_search_min,
                  vel_tolerance, propoper_accel_value,  planning_time_horizon, rel_ego_stop_s,use_stop_line,\
                  yield_weight,  overtake_weight,violation_weight,s_dis_weight,max_count, two_obj_rel_dis,#cost config
                  vel_weight,accel_weight,  accel_sign_weight, jerk_weight,  virtual_yield_weight,length_t_weight,
                  hcost_t_weight, hcost_s_weight, hcost_v_weight, upper_trancation_time_buffer,lower_trancation_time_buffer, min_upper_distance_buffer,
                  min_lower_distance_buffer,comfort_gap_length,
                  init_s, init_v, init_a, updated_heuristic_s,#ego init state
                  obj_id, obj_s, obj_v, #obj info
                          ):
  if clear_objs == 1:
    st_search_py.ClearObjs()
    ClearStFig()
  if use_default_obj ==1:
    AddDefaultObjs()
  elif adjust_all_obj ==1 and fix_objs == 0:
    UseTwoObjs(obj_s, obj_v, two_obj_rel_dis)
  else:
    st_search_py.AddObj(obj_id, obj_s, obj_v/3.4)

  update_params = st_search_py.UpdateParams( max_acc_limit,  min_acc_limit, max_jerk_limit,
                                             min_jerk_limit, speed_limit,  speed_limit_scale,
                                             v_cruise, v_min,collision_ttc, min_collision_dist, max_collision_dist,
                                             s_step,  t_step, vel_step, acc_search_step, max_search_time,
                                             acc_search_max,  acc_search_min,vel_tolerance, propoper_accel_value,
                                             planning_time_horizon,rel_ego_stop_s,use_stop_line,max_count)
  update_weight = st_search_py.UpdateWeight(yield_weight,  overtake_weight,vel_weight,accel_weight,  accel_sign_weight, jerk_weight,  virtual_yield_weight,length_t_weight,
                  hcost_t_weight, hcost_s_weight,hcost_v_weight, upper_trancation_time_buffer,lower_trancation_time_buffer, min_upper_distance_buffer,
                  min_lower_distance_buffer,comfort_gap_length,violation_weight,s_dis_weight)
  update_ego_state = st_search_py.UpdateEgoState(init_s, init_v/3.4, init_a, updated_heuristic_s)
  st_search_py.MakeStPoints()

  if fix_result == 0:
    # update_params = st_search_py.UpdateParams( max_acc_limit,  min_acc_limit, max_jerk_limit,
    #                                                 min_jerk_limit, speed_limit,  speed_limit_scale,
    #                                                 v_cruise, collision_ttc, min_collision_dist, max_collision_dist,
    #                                                 s_step,  t_step, vel_step, acc_search_step, max_search_time,
    #                                                 acc_search_max,  acc_search_min,vel_tolerance, propoper_accel_value,
    #                                                 planning_time_horizon)
    # update_weight = st_search_py.UpdateWeight(yield_weight,  overtake_weight,vel_weight,accel_weight,  accel_sign_weight, jerk_weight,  virtual_yield_weight,length_t_weight,
    #               hcost_t_weight, upper_trancation_time_buffer,lower_trancation_time_buffer, min_upper_distance_buffer,
    #               min_lower_distance_buffer)
    # update_ego_state = st_search_py.UpdateEgoState(init_s, init_v, init_a, updated_heuristic_s)

    st_search_py.UpdateSearch()

  nodes = st_search_py.nodes()
  nodes_vel_cost_list = st_search_py.vel_cost_list()
  nodes_acc_cost_list = st_search_py.acc_cost_list()
  nodes_jerk_cost_list = st_search_py.jerk_cost_list()
  nodes_h_s_cost_list = st_search_py.h_s_cost_list()
  nodes_h_v_cost_list = st_search_py.h_v_cost_list()
  nodes_h_cost_list = st_search_py.h_cost_list()
  nodes_g_cost_list = st_search_py.g_cost_list()
  nodes_match_slot_list = st_search_py.match_slot_list()
  nodes_v_list = st_search_py.v_list()
  s_dis_cost_list = st_search_py.s_dis_cost_list()
  aligned_v_list = st_search_py.aligned_v_list()
  # st_search_py.UpdateObjInfo()
  # search_path = st_search_py.GetSearchPath()
  # print("node_s ", node_s)

  st_boundaries = st_search_py.GetAllSTBoundary()
  heuristic_v_list = st_search_py.GenerateHeruisticVList()
  heuristic_s_list = st_search_py.GenerateHeruisticSList()
  print("heruistic v: \n", heuristic_v_list)
  intervals = st_search_py.GetSTIntervals()
  rel_ego_stop_s = st_search_py.GetStopLineS()
  # print("\nstop s: ", rel_ego_stop_s)
  ClearStFig()

  UpdateIntervals(intervals)
  UpdateHeuristicS(heuristic_s_list)
  for i in range(len(st_boundaries)):
    s = []
    t = []
    for j in range(len(st_boundaries[i])):
      s.append(st_boundaries[i][j][0])
      t.append(st_boundaries[i][j][1])
    s.append(st_boundaries[i][0][0])
    t.append(st_boundaries[i][0][1])
    if i == 0:
      data_st_boundary_1.data.update({'s': s, 't': t})
    if i == 1:
      data_st_boundary_2.data.update({'s': s, 't': t})
    if i == 2:
      data_st_boundary_3.data.update({'s': s, 't': t})
    if i == 3:
      data_st_boundary_4.data.update({'s': s, 't': t})
    if i == 4:
      data_st_boundary_5.data.update({'s': s, 't': t})
    if i == 5:
      data_st_boundary_6.data.update({'s': s, 't': t})
    if i == 6:
      data_st_boundary_7.data.update({'s': s, 't': t})
    if i == 7:
          print("Target lane obj is more than 7!")
  data_ego_init_state.update(data = {'s':[init_s], 'v':[init_v], 'a':[init_a], 't':[0]})

  # heuristic_s = []
  # heuristic_t = []
  # for j in range(len(slot_point_info)):
  #   print("heuristi_s: ", slot_point_info)
  #   heuristic_s.append(slot_point_info[j])
  #   heuristic_s.append(slot_point_info[j])
  #   heuristic_t.append(0.0)
  #   heuristic_t.append(5.0)
  #   data_heuristic_s_list.update(data = {'s':heuristic_s, 't':heuristic_t})
  node_s =[]
  node_t =[]
  vel_cost_list =[]
  acc_cost_list =[]
  jerk_cost_list =[]
  h_s_cost_list =[]
  h_v_cost_list =[]
  h_cost_list =[]
  g_cost_list=[]
  match_slot_list=[]
  s_dis_cost=[]
  v=[]
  node_aligned_v_list =[]
  if show_print == 1:
    for m in range(len(nodes)):
      # print("node s", nodes)
      node_s.append(nodes[m][0])
      node_t.append(nodes[m][1])
      vel_cost_list.append(nodes_vel_cost_list[m])
      acc_cost_list.append(nodes_acc_cost_list[m])
      jerk_cost_list.append(nodes_jerk_cost_list[m])
      h_s_cost_list.append(nodes_h_s_cost_list[m])
      h_v_cost_list.append(nodes_h_v_cost_list[m])
      h_cost_list.append(nodes_h_cost_list[m])
      g_cost_list.append(nodes_g_cost_list[m])
      match_slot_list.append(nodes_match_slot_list[m])
      v.append(nodes_v_list[m] * 3.4)
      s_dis_cost.append(s_dis_cost_list[m])
      node_aligned_v_list.append(aligned_v_list[m] * 3.4)

  data_nodes_list.update(data= {'s': node_s, 't': node_t,'v':v, 's_dis_cost':s_dis_cost,'vel_cost': vel_cost_list, 'acc_cost': acc_cost_list,\
                                'jerk_cost':jerk_cost_list, 'h_s_cost':h_s_cost_list, 'h_v_cost':h_v_cost_list,\
                                'match_slot_list':match_slot_list, 'aligned_v':node_aligned_v_list,
                                'h_cost':h_cost_list,'g_cost':g_cost_list})


  data_search_res.data.update({'s': st_search_py.GetSearchPathS(),
                               't': st_search_py.GetSearchPathT(),})
  print("search s: ", st_search_py.GetSearchPathS())
  print("search t: ", st_search_py.GetSearchPathT())
  data_stitch_res.data.update({'s': st_search_py.GetStitchPathS(),
                               't': st_search_py.GetStitchPathT(),})
  print("stitch s: ", st_search_py.GetStitchPathS())
  print("stitch t: ", st_search_py.GetStitchPathT())
  # print("search path s: ", st_search_py.GetSearchPathS())
  # print("search path t: ", st_search_py.GetSearchPathT())
  push_notebook()


bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
