import sys, os
sys.path.append("..")
from lib.load_rosbag import *
from lib.load_config import *
from lib.load_struct import *
from lib.load_rotate import *

sys.path.append('../..')
from build import longitudinal_motion_planning_py

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
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, HoverTool
from collections import namedtuple
from functools import  partial
from lib.basic_layers import *
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, HoverTool

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_path = "/root/code/bags/cp/ring_road/PLAA72032_event_light_recording_20221228-144448_0.bag"

bag_loder = LoadRosbag(bag_path)
pybind_func_param = longitudinal_motion_planning_py.PybindFuncParam()
planning_input = longitudinal_motion_planning_py.LongitudinalMotionPlanningInput()
planning_output = longitudinal_motion_planning_py.LongitudinalMotionPlanningOutput()

solver_info = longitudinal_motion_planning_py.iLqrSolverInfo()

planning_input_generator = longitudinal_motion_planning_py.LongitudinalMotionPlanningPybindFunc()
longitudinal_motion_planner = longitudinal_motion_planning_py.LongitudinalMotionPlanningProblem()
longitudinal_motion_planner.Init()

global gloab_stop_enable_enable
gloab_stop_enable_enable = False

class LongitudinalMotionPlanningSlider:
    def __init__(self, silder_callback):
        self.set_vel_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "set_vel",min=0.0, max=35.0, value=15.0, step= 0.05)
        self.ref_acc_inc_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ref_acc_inc",min=0.0, max=2.0, value=0.5, step= 0.05)
        self.ref_acc_dec_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ref_acc_dec",min=-4.0, max=0.0, value=-0.5, step= 0.05)
        self.vel_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "vel",min=0.0, max=35.0, value=14.0, step= 0.05)
        self.acc_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "acc",min=-6.0, max=6.0, value=0.8, step= 0.05)
        self.cipv_vel_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "cipv_vel",min=0.0, max=35.0, value=14.0, step= 0.05)
        self.cipv_dist_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "cipv_dist",min=6.0, max=100.0, value=8.0, step= 0.05)
        self.stop_enable_silder = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='10%'), description= "stop_enable",min=0, max=1, value=0, step= 1)
        # self.stop_s_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "stop_s",min=0.0, max=100.0, value=0.1, step= 0.05)
        self.q_ref_pos_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "q_ref_pos",min=0.0, max=100.0, value=0.1, step= 0.05)
        self.q_ref_vel_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "q_ref_vel",min=0.0, max=100.0, value=1.0, step= 0.05)
        self.q_acc_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "q_acc",min=0.0, max=100.0, value=1.0, step= 0.05)
        self.q_jerk_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "q_jerk",min=0.0, max=100.0, value=1.0, step= 0.05)
        self.q_snap_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "q_snap",min=0.0, max=100.0, value=1.0, step= 0.05)
        self.q_pos_bound_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "q_pos_bound",min=0.0, max=5000.0, value=3800.0, step= 0.05)
        self.q_vel_bound_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "q_vel_bound",min=0.0, max=5000.0, value=100.0, step= 0.05)
        self.q_acc_bound_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "q_acc_bound",min=0.0, max=5000.0, value=100.0, step= 0.05)
        self.q_jerk_bound_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "q_jerk_bound",min=0.0, max=5000.0, value=100.0, step= 0.05)
        self.q_s_stop_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "q_stop_s",min=0.0, max=15000.0, value=5000.0, step= 0.05)
        self.max_iter_silder = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='25%'), description= "max_iter",min=3, max=200, value=10, step= 1)
        self.warm_start_silder = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='10%'), description= "warm_start",min=0, max=1, value=0, step= 1)

        ipywidgets.interact(silder_callback, set_vel = self.set_vel_silder, ref_acc_inc = self.ref_acc_inc_silder, ref_acc_dec = self.ref_acc_dec_silder,\
            vel = self.vel_silder, acc = self.acc_silder, cipv_vel = self.cipv_vel_silder, cipv_dist = self.cipv_dist_silder, stop_enable = self.stop_enable_silder, \
            # stop_s = self.stop_s_silder, \
            q_ref_pos = self.q_ref_pos_silder, q_ref_vel = self.q_ref_vel_silder, q_acc = self.q_acc_silder, q_jerk = self.q_jerk_silder, q_snap = self.q_snap_silder, \
            q_pos_bound = self.q_pos_bound_silder, q_vel_bound = self.q_vel_bound_silder, q_acc_bound = self.q_acc_bound_silder, q_jerk_bound = self.q_jerk_bound_silder,\
            q_stop_s = self.q_s_stop_silder, max_iter = self.max_iter_silder, warm_start = self.warm_start_silder)

# figures config
fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=400, height=720, match_aspect = True, aspect_scale=1)
fig1.x_range.flipped = True
fig2 = bkp.figure(x_axis_label='time', y_axis_label='longitudinal pos',x_range = [-0.1, 5.0], width=600, height=180)
fig3 = bkp.figure(x_axis_label='time', y_axis_label='longitudinal vel',x_range = fig2.x_range, width=600, height=180)
fig4 = bkp.figure(x_axis_label='time', y_axis_label='longitudinal acc',x_range = fig2.x_range, width=600, height=180)
fig5 = bkp.figure(x_axis_label='time', y_axis_label='longitudinal jerk',x_range = fig2.x_range, width=600, height=180)

data_car = ColumnDataSource()
data_reference_planning = ColumnDataSource()
data_text = ColumnDataSource()

f1 = fig1.patch('car_yb', 'car_xb', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'ego')
fig1.patch('car_yb_cipv', 'car_xb_cipv', source = data_car, fill_color = "red", line_color = "black", line_width = 1, legend_label = 'cipv')
fig1.line('ref_yb_vec', 'pos_vec', source = data_reference_planning, line_width = 12.0, line_color = 'blue', line_dash = 'solid', alpha = 0.6, legend_label = 'planning')
fig1.line('ref_yb_vec', 'pos_max_vec', source = data_reference_planning, line_width = 8.0, line_color = '#FFA500', line_dash = 'solid', alpha = 0.3, legend_label = 'prediction')
fig1.text(2.0, -2.0, text = 'fig1_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt")

f2 = fig2.line('time_vec', 'ref_pos_vec', source = data_reference_planning, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'ref_pos')
fig2.line('time_vec', 'pos_max_vec', source = data_reference_planning, line_width = 3, line_color = 'grey', line_dash = 'dashed', legend_label = 'pos_max')
# fig2.triangle('time_vec', 'pos_max_vec', source = data_reference_planning, size=10, fill_color='grey',legend_label = 'pos_max')
fig2.line('time_vec', 'pos_vec', source = data_reference_planning, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'pos')
hover2 = HoverTool(renderers=[f2], tooltips=[('time', '@time_vec'), ('ref_pos', '@ref_pos_vec'), ('pos_max', '@pos_max_vec'), ('pos_vec', '@pos_vec')], mode='vline')
fig2.add_tools(hover2)

f3 = fig3.line('time_vec', 'ref_vel_vec', source = data_reference_planning, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'ref_vel')
fig3.line('time_vec', 'vel_vec', source = data_reference_planning, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'vel')
hover3 = HoverTool(renderers=[f3], tooltips=[('time', '@time_vec'), ('ref_vel', '@ref_vel_vec'), ('vel', '@vel_vec')], mode='vline')
fig3.add_tools(hover3)

f4 = fig4.line('time_vec', 'acc_vec', source = data_reference_planning, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'acc')
hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time_vec'), ('acc', '@acc_vec')], mode='vline')
fig4.add_tools(hover4)

f5 = fig5.line('time_vec', 'jerk_vec', source = data_reference_planning, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'jerk')
hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('jerk', '@jerk_vec')], mode='vline')
fig5.add_tools(hover5)

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)

fig1.legend.click_policy = 'hide'
fig2.legend.click_policy = 'hide'
fig3.legend.click_policy = 'hide'
fig4.legend.click_policy = 'hide'
fig5.legend.click_policy = 'hide'

def silder_callback(set_vel, ref_acc_inc, ref_acc_dec, vel, acc, cipv_vel, cipv_dist, stop_enable, q_ref_pos, q_ref_vel, q_acc, q_jerk, q_snap, \
    q_pos_bound, q_vel_bound, q_acc_bound, q_jerk_bound, q_stop_s, max_iter, warm_start):

  kwargs = locals()
  global gloab_stop_enable_enable

  # update car
  car_xb, car_yb = load_car_params_patch()

  # update cipv car
  car_xb_cipv = []
  car_yb_cipv = []
  for i in range(len(car_xb)):
      car_xb_cipv.append(car_xb[i] + cipv_dist)
      car_yb_cipv.append(car_yb[i])

  data_car.data.update({
    'car_xb': car_xb,
    'car_yb': car_yb,
    'car_xb_cipv': car_xb_cipv,
    'car_yb_cipv': car_yb_cipv})

  # generate planning input
  pybind_func_param.set_vel = set_vel
  pybind_func_param.ref_acc_inc = ref_acc_inc
  pybind_func_param.ref_acc_dec = ref_acc_dec
  pybind_func_param.vel = vel
  pybind_func_param.acc = acc
  pybind_func_param.cipv_vel = cipv_vel
  pybind_func_param.cipv_dist = cipv_dist
  pybind_func_param.stop_enable = stop_enable
  # pybind_func_param.stop_s = stop_s
  pybind_func_param.q_ref_pos = q_ref_pos
  pybind_func_param.q_ref_vel = q_ref_vel
  pybind_func_param.q_ref_pos = q_ref_pos
  pybind_func_param.q_acc = q_acc
  pybind_func_param.q_jerk = q_jerk
  pybind_func_param.q_snap = q_snap
  pybind_func_param.q_pos_bound = q_pos_bound
  pybind_func_param.q_vel_bound = q_vel_bound
  pybind_func_param.q_acc_bound = q_acc_bound
  pybind_func_param.q_jerk_bound = q_jerk_bound
  pybind_func_param.q_stop_s = q_stop_s

  if (gloab_stop_enable_enable == False) & (stop_enable == True):
    slider_class.set_vel_silder.value  = 0.0
    slider_class.vel_silder.value = 5.0
    slider_class.cipv_vel_silder.value = 0.0

  if (gloab_stop_enable_enable == True) & (stop_enable == False):
    slider_class.set_vel_silder.value  = 15.0
    slider_class.vel_silder.value = 14.0
    slider_class.cipv_vel_silder.value = 14.0

  gloab_stop_enable_enable = stop_enable

  if stop_enable == True:
    pybind_func_param.stop_s = cipv_dist

  planning_input_generator.Update(pybind_func_param)
  planning_input = planning_input_generator.GetOutput()

  ref_yb_vec = [0] * 41
  time_vec = [i * 0.1 for i in range(41)]
  data_reference_planning.data.update({
      'time_vec': time_vec,
      'ref_pos_vec': planning_input.ref_pos_vec,
      'ref_yb_vec': ref_yb_vec,
      'ref_vel_vec': planning_input.ref_vel_vec,
      'pos_max_vec': planning_input.pos_max_vec,
  })

  # longitudinal motion planning
  longitudinal_motion_planner.SetWarmStart(warm_start)
  longitudinal_motion_planner.SetMaxIter(max_iter)

  flag = longitudinal_motion_planner.Update(planning_input)

  planning_output = longitudinal_motion_planner.GetOutput()

  data_reference_planning.data.update({
    'time_vec': time_vec,
    'pos_vec': planning_output.pos_vec,
    'vel_vec': planning_output.vel_vec,
    'acc_vec': planning_output.acc_vec,
    'jerk_vec':planning_output.jerk_vec,
  })

  data_text.data.update({
    'fig1_text': ['vel_ego={:.2f}m/s\n'.format(round(vel, 2)), 'vel_cipv={:.2f}m/s'.format(round(cipv_vel, 2))],
  })

  push_notebook()

bkp.show(row(fig1, column(fig2, fig3, fig4, fig5)), notebook_handle=True)
slider_class = LongitudinalMotionPlanningSlider(silder_callback)

