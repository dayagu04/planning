import sys, os
sys.path.append("..")
from lib.load_rosbag import *
from lib.load_config import *
from lib.load_struct import *
from lib.load_rotate import *

sys.path.append('../..')
from build import mpc_common
from build import lateral_motion_planning_py

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

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_path = "/root/code/bags/cp/ring_road/PLAA72032_event_light_recording_20221228-144448_0.bag"

bag_loder = LoadRosbag(bag_path)
ego_info = bag_loder.LoadEgoInfo()
lane_info = bag_loder.LoadLaneInfo()
plan_info = bag_loder.LoadPlanInfo()
ctrl_info = bag_loder.LoadCtrlInfo()

pybind_func_param = lateral_motion_planning_py.PybindFuncParam()
planning_input = lateral_motion_planning_py.LateralMotionPlanningInput()
planning_output = lateral_motion_planning_py.LateralMotionPlanningOutput()

solver_info = lateral_motion_planning_py.iLqrSolverInfo()

planning_input_generator = lateral_motion_planning_py.LateralMotionPlanningPybindFunc()
lateral_motion_planner = lateral_motion_planning_py.LateralMotionPlanningProblem()
lateral_motion_planner.Init()

global bag_time_all
bag_time_all = 0.0

global click_flag
click_flag = False

class LateralMotionPlanningSlider:
    def __init__(self, bag_data, silder_callback):
        self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=bag_data['t'][-1], value=-0.1, step= 0.1)
        self.set_vel_gain_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='25%'), description= "set_vel_gain",min=0.0, max=3.0, value=1.0, step= 0.05)
        self.q_xy_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_xy",min=0.0, max=50.0, value=15.0, step= 0.05)
        self.q_theta_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_theta",min=0.0, max=50.0, value=10.0, step= 0.05)
        self.q_acc_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_acc",min=0.0, max=10.0, value=1.0, step= 0.01)
        self.q_jerk_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_jerk",min=0.0, max=10.0, value=1.2, step= 0.01)
        self.q_snap_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_snap",min=0.0, max=100.0, value=1.2, step= 0.01)
        self.q_continuity_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_continuity",min=0.0, max=5.0, value=0.15, step= 0.02)
        self.q_soft_corridor_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_soft_corridor",min=0.0, max=2000.0, value=100.0, step= 1.0)
        self.q_hard_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_hard_bound",min=0.0, max=20000.0, value=2000.0, step= 1.0)
        self.ref_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "ref_offset",min=-4.0, max=4.0, value=0.0, step= 0.02)
        self.upper_corridor_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "upper_corridor offset",min=-10.0, max=10.0, value=5.0, step= 0.02)
        self.lower_corridor_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "lower_corridor offset",min=-10.0, max=10.0, value=5.0, step= 0.02)
        self.acc_bound_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "acc_bound",min=0.0, max=10.0, value=5.0, step= 0.01)
        self.jerk_bound_silder = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "jerk_bound",min=0.0, max=10.0, value=3.0, step= 0.01)
        self.max_iter_silder = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='25%'), description= "max_iter",min=3, max=200, value=6, step= 1)
        self.warm_start_silder = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='10%'), description= "warm_start",min=0, max=1, value=0, step= 1)

        ipywidgets.interact(silder_callback, bag_time = self.time_slider, set_vel_gain = self.set_vel_gain_silder, \
                q_xy = self.q_xy_silder, q_theta = self.q_theta_silder, q_acc = self.q_acc_silder, q_jerk = self.q_jerk_slider, q_snap = self.q_snap_slider, \
                q_continuity = self.q_continuity_silder,\
                q_soft_corridor = self.q_soft_corridor_slider, q_hard_bound = self.q_hard_bound_slider,  ref_offset = self.ref_offset_slider, upper_corridor = self.upper_corridor_slider, \
                lower_corridor = self.lower_corridor_slider, acc_bound = self.acc_bound_silder, jerk_bound = self.jerk_bound_silder, max_iter = self.max_iter_silder, warm_start = self.warm_start_silder)

fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=400, height=720, match_aspect = True, aspect_scale=1)
fig1.x_range.flipped = True
fig2 = bkp.figure(x_axis_label='y', y_axis_label='x', width=850, height=470, match_aspect = True, aspect_scale=1)
fig2.x_range.flipped = True

fig3 = bkp.figure(x_axis_label='init/iter', y_axis_label='cost term',x_range = [-0.1, 10.1], width=850, height=250)
fig4 = bkp.figure(x_axis_label='time', y_axis_label='lateral acc',x_range = [-0.1, 2.6], width=600, height=180)
fig5 = bkp.figure(x_axis_label='time', y_axis_label='lateral jerk',x_range = fig4.x_range, width=600, height=180)
fig6 = bkp.figure(x_axis_label='time', y_axis_label='delta',x_range = fig5.x_range, width=600, height=180)
fig7 = bkp.figure(x_axis_label='time', y_axis_label='omega',x_range = fig5.x_range, width=600, height=180)

data_car = ColumnDataSource()
data_ego = ColumnDataSource()
data_lane = ColumnDataSource()
data_reference_planning = ColumnDataSource()
data_corridor = ColumnDataSource()
data_reference_planning = ColumnDataSource()
data_cost = ColumnDataSource()
data_text = ColumnDataSource()

f1 = fig1.patch('car_yb', 'car_xb', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
fig1.line('ref_yb_vec', 'ref_xb_vec', source = data_reference_planning, line_width = 3.0, line_color = 'red', line_dash = 'solid', alpha = 0.5, legend_label = 'reference')
fig1.line('last_yb_vec', 'last_xb_vec', source = data_reference_planning, line_width = 1.8, line_color = 'green', line_dash = 'dashed', alpha = 1.0, legend_label = 'last trajectory')
fig1.multi_line('lane_yb', 'lane_xb', source = data_lane, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
fig1.multi_line('soft_bound_yb', 'soft_bound_xb', source = data_corridor, line_width = 8.0, line_color = "#FFA500", line_dash = 'solid', alpha = 0.2, legend_label = 'soft corridor')
fig1.line('yb_vec', 'xb_vec', source = data_reference_planning, line_width = 5.0, line_color = "blue", line_dash = 'solid', alpha = 0.3, legend_label = 'planning')
fig1.text(2.0, -2.0, text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt")

f2 = fig2.patch('car_yn', 'car_xn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
fig2.line('ref_yn_vec', 'ref_xn_vec', source = data_reference_planning, line_width = 3.0, line_color = 'red', line_dash = 'solid', alpha = 0.5, legend_label = 'reference')
fig2.line('last_yn_vec', 'last_xn_vec', source = data_reference_planning, line_width = 1.8, line_color = 'green', line_dash = 'dashed', alpha = 1.0, legend_label = 'last trajectory')
fig2.line('ego_yn', 'ego_xn', source = data_ego, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
fig2.multi_line('lane_yn', 'lane_xn', source = data_lane, line_width = 1, line_color = 'black', line_dash = 'dashed', legend_label = 'lane')
fig2.multi_line('soft_bound_yn', 'soft_bound_xn', source = data_corridor, line_width = 8.0, line_color = "#FFA500", line_dash = 'solid', alpha = 0.2, legend_label = 'soft corridor')
fig2.line('yn_vec', 'xn_vec', source = data_reference_planning, line_width = 5.0, line_color = "blue", line_dash = 'solid', alpha = 0.3, legend_label = 'planning')
fig2.text('text_yn', 'text_xn', text = 'vel_ego_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt")

f3 = fig3.line('index', 'cost0', source = data_cost, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'reference cost')
fig3.line('index', 'cost1', source = data_cost, line_width = 1, line_color = 'red', line_dash = 'dashed', legend_label = 'continuity cost')
fig3.line('index', 'cost2', source = data_cost, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'acc cost')
fig3.line('index', 'cost3', source = data_cost, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'jerk cost')
fig3.line('index', 'cost4', source = data_cost, line_width = 1, line_color = 'blue', line_dash = 'dashed', legend_label = 'acc bound cost')
fig3.line('index', 'cost5', source = data_cost, line_width = 1, line_color = 'purple', line_dash = 'dashed', legend_label = 'jerk bound cost')
fig3.line('index', 'cost6', source = data_cost, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'soft corridor cost')
fig3.text(4.0, 2.0, text = 'cost_text' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt")
hover3 = HoverTool(renderers=[f3], tooltips=[('index', '@index'), ('reference cost', '@cost0'), ('continuity cost', '@cost1'), ('acc cost', '@cost2'), ('jerk cost', '@cost3'), \
        ('acc bound cost', '@cost4'), ('jerk bound cost', '@cost5'), ('soft corridor cost', '@cost6')], mode='vline')
fig3.add_tools(hover3)

f4 = fig4.line('time_vec', 'acc_vec', source = data_reference_planning, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'acc')
hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time_vec'), ('lateral acc', '@acc_vec')], mode='vline')
fig4.add_tools(hover4)

f5 = fig5.line('time_vec', 'jerk_vec', source = data_reference_planning, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'jerk')
hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('lateral jerk', '@jerk_vec')], mode='vline')
fig5.add_tools(hover5)

f6 = fig6.line('time_vec', 'delta_vec', source = data_reference_planning, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steering angle/(deg)')
hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time_vec'), ('steering angle', '@delta_vec')], mode='vline')
fig6.add_tools(hover6)

f7 = fig7.line('time_vec', 'omega_vec', source = data_reference_planning, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steering rate/(deg/s)')
hover7 = HoverTool(renderers=[f7], tooltips=[('time', '@time_vec'), ('steering rate', '@omega_vec')], mode='vline')
fig7.add_tools(hover7)

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
fig6.toolbar.active_scroll = fig6.select_one(WheelZoomTool)
fig7.toolbar.active_scroll = fig7.select_one(WheelZoomTool)

fig1.legend.click_policy = 'hide'
fig2.legend.click_policy = 'hide'
fig3.legend.click_policy = 'hide'
fig4.legend.click_policy = 'hide'
fig5.legend.click_policy = 'hide'
fig6.legend.click_policy = 'hide'
fig7.legend.click_policy = 'hide'

def silder_callback(bag_time, set_vel_gain, q_xy, q_theta, q_acc, q_jerk, q_snap, q_continuity, \
    q_soft_corridor, q_hard_bound, ref_offset, upper_corridor, lower_corridor, acc_bound, jerk_bound, max_iter, warm_start):

  kwargs = locals()

  global bag_time_all
  bag_time_all = bag_time

  ctrl_idx = 0
  while ctrl_info['t'][ctrl_idx] <= bag_time and ctrl_idx < (len(ctrl_info['t'])-2):
      ctrl_idx = ctrl_idx + 1

  ego_idx = 0
  while ego_info['t'][ego_idx] <= bag_time and ego_idx < (len(ego_info['t'])-1):
      ego_idx = ego_idx + 1

  lane_idx = 0
  while lane_info['t'][lane_idx] <= bag_time and lane_idx < (len(lane_info['t'])-1):
      lane_idx = lane_idx + 1

  cur_pos_xn = ego_info['data_global'][ego_idx]['x']
  cur_pos_yn = ego_info['data_global'][ego_idx]['y']
  cur_yaw = ego_info['data_global'][ego_idx]['yaw']
  cur_pos_xn0 = ego_info['data_global'][0]['x']
  cur_pos_yn0 = ego_info['data_global'][0]['y']
  vel_ego = ctrl_info['data'][ctrl_idx]['vel_ego']

  ego_xb, ego_yb = [], []
  ego_xn, ego_yn = [], []
  for i in range(len(ego_info['data_global'])):
      ego_local_x, ego_local_y= global2local(ego_info['data_global'][i]['x'], ego_info['data_global'][i]['y'], \
                                               cur_pos_xn, cur_pos_yn, cur_yaw)
      ego_xb.append(ego_local_x)
      ego_yb.append(ego_local_y)

      ego_xn.append(ego_info['data_global'][i]['x'] - cur_pos_xn0)
      ego_yn.append(ego_info['data_global'][i]['y'] - cur_pos_yn0)

  car_xb, car_yb = load_car_params_patch()

  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], cur_pos_xn, cur_pos_yn, cur_yaw)
      car_xn.append(tmp_x - cur_pos_xn0)
      car_yn.append(tmp_y - cur_pos_yn0)

  data_car.data.update({
    'car_xb': car_xb,
    'car_yb': car_yb,
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  lat_mpc_input = mpc_common.LatMpcInput()
  lat_mpc_input.curv_factor_ = ctrl_info['data'][ctrl_idx]['curv_factor']
  lat_mpc_input.dx_ref_mpc_vec_   = ctrl_info['data'][ctrl_idx]['dx_ref_vec_mpc']
  lat_mpc_input.dy_ref_mpc_vec_   = ctrl_info['data'][ctrl_idx]['dy_ref_vec_mpc']
  lat_mpc_input.dphi_ref_mpc_vec_ = ctrl_info['data'][ctrl_idx]['dphi_ref_vec_mpc']
  lat_mpc_input.vel_lat_ctrl_mpc_vec_ = ctrl_info['data'][ctrl_idx]['vel_ref_vec_mpc']

  data_ego.data.update({
    'ego_xb': ego_xb,
    'ego_yb': ego_yb,
    'ego_xn': ego_xn,
    'ego_yn': ego_yn,
  })

  text_xn = cur_pos_xn - cur_pos_xn0 - 2.0
  text_yn = cur_pos_yn - cur_pos_yn0 + 2.0
  data_text.data.update({
    'vel_ego_text': ['vel_ego={:.2f}m/s'.format(round(vel_ego, 2))],
    'text_xn': [text_xn],
    'text_yn': [text_yn],
    })

  data_lane.data.update({
    'lane_xn': lane_info['data'][lane_idx][0],
    'lane_yn': lane_info['data'][lane_idx][1],
    'lane_xb': lane_info['data'][lane_idx][3],
    'lane_yb': lane_info['data'][lane_idx][2],
  })

  # generate planning input
  pybind_func_param.pos_x = cur_pos_xn
  pybind_func_param.pos_y = cur_pos_yn
  pybind_func_param.theta = cur_yaw
  pybind_func_param.delta = ctrl_info['data'][ctrl_idx]['delta_angle_vec_mpc'][0]

  pybind_func_param.q_ref_x = q_xy
  pybind_func_param.q_ref_y = q_xy
  pybind_func_param.q_ref_theta = q_theta * (1 + vel_ego * vel_ego * set_vel_gain * set_vel_gain)
  pybind_func_param.q_acc = q_acc
  pybind_func_param.q_jerk = q_jerk
  pybind_func_param.acc_bound = acc_bound
  pybind_func_param.jerk_bound = jerk_bound
  pybind_func_param.set_vel_gain = set_vel_gain
  pybind_func_param.ref_offset = -ref_offset
  pybind_func_param.q_acc_bound = q_hard_bound
  pybind_func_param.q_jerk_bound = q_hard_bound
  pybind_func_param.corridor_upper_offset = upper_corridor
  pybind_func_param.corridor_lower_offset = lower_corridor
  pybind_func_param.q_soft_corridor = q_soft_corridor
  pybind_func_param.q_continuity = q_continuity
  pybind_func_param.q_snap = q_snap * (1 + vel_ego)


  planning_input_generator.Update(lat_mpc_input, pybind_func_param)
  planning_input = planning_input_generator.GetOutput()

  # reference traj
  ref_xn_vec, ref_yn_vec, ref_xb_vec, ref_yb_vec = \
    getposbodyandworld(planning_input.ref_x_vec, planning_input.ref_y_vec, cur_pos_xn, cur_pos_yn, cur_yaw, cur_pos_xn0, cur_pos_yn0)

  # last reference traj
  last_xn_vec, last_yn_vec, last_ref_xb_vec, last_ref_yb_vec = \
    getposbodyandworld(planning_input.last_x_vec, planning_input.last_y_vec, cur_pos_xn, cur_pos_yn, cur_yaw, cur_pos_xn0, cur_pos_yn0)

  data_reference_planning.data.update({
    'ref_xn_vec': ref_xn_vec,
    'ref_yn_vec': ref_yn_vec,
    'ref_xb_vec': ref_xb_vec,
    'ref_yb_vec': ref_yb_vec,
    'last_xn_vec': last_xn_vec,
    'last_yn_vec': last_yn_vec,
    'last_xb_vec': last_ref_xb_vec,
    'last_yb_vec': last_ref_yb_vec,
  })

  # soft corridor
  soft_upper_bound_xn_vec, soft_upper_bound_yn_vec, soft_upper_bound_xb_vec, soft_upper_bound_yb_vec = \
    getposbodyandworld(planning_input.soft_upper_bound_x0_vec, planning_input.soft_upper_bound_y0_vec, \
      cur_pos_xn, cur_pos_yn, cur_yaw, cur_pos_xn0, cur_pos_yn0)

  soft_lower_bound_xn_vec, soft_lower_bound_yn_vec, soft_lower_bound_xb_vec, soft_lower_bound_yb_vec = \
    getposbodyandworld(planning_input.soft_lower_bound_x0_vec, planning_input.soft_lower_bound_y0_vec, \
      cur_pos_xn, cur_pos_yn, cur_yaw, cur_pos_xn0, cur_pos_yn0)

  print("vel_ego = %.2f m/s (%.1f kph)" %(vel_ego, vel_ego * 3.6))
  print("ref_vel = %.2f m/s (%.1f kph)" %(planning_input.ref_vel, planning_input.ref_vel * 3.6))

  data_corridor.data.update({
    'soft_bound_xn': [soft_upper_bound_xn_vec, soft_lower_bound_xn_vec],
    'soft_bound_yn': [soft_upper_bound_yn_vec, soft_lower_bound_yn_vec],
    'soft_bound_xb': [soft_upper_bound_xb_vec, soft_lower_bound_xb_vec],
    'soft_bound_yb': [soft_upper_bound_yb_vec, soft_lower_bound_yb_vec],
  })

  # if you wanna try switching between solid and dashed, like this
  if bag_time > 10.0:
    fig1.renderers[4].glyph.line_dash = 'dashed'
  else:
    fig1.renderers[4].glyph.line_dash = 'solid'

  # lateral motion planning
  lateral_motion_planner.SetWarmStart(warm_start)
  lateral_motion_planner.SetMaxIter(max_iter)

  flag = lateral_motion_planner.Update(planning_input)

  planning_output = lateral_motion_planner.GetOutput()
  # planning trajectory
  xn_vec, yn_vec, xb_vec, yb_vec = \
    getposbodyandworld(planning_output.x_vec, planning_output.y_vec, \
      cur_pos_xn, cur_pos_yn, cur_yaw, cur_pos_xn0, cur_pos_yn0)

  delta_to_steer_deg = 14.5 * 57.3

  data_reference_planning.data.update({
    'time_vec': planning_output.time_vec,
    'xn_vec': xn_vec,
    'yn_vec': yn_vec,
    'theta_vec': planning_output.theta_vec,
    'delta_vec': [x * delta_to_steer_deg for x in planning_output.delta_vec],
    'omega_vec': [x * delta_to_steer_deg for x in planning_output.omega_vec],
    'acc_vec': planning_output.acc_vec,
    'jerk_vec': planning_output.jerk_vec,
    'xb_vec': xb_vec,
    'yb_vec': yb_vec
  })

  # solver info
  solver_info = planning_output.solver_info

  # cost info
  cost0 = []
  cost1 = []
  cost2 = []
  cost3 = []
  cost4 = []
  cost5 = []
  cost6 = []
  try:
    for i in range(solver_info.iter_count + 1):
      cost0.append(solver_info.cost_iter_vec[i][0])
      cost1.append(solver_info.cost_iter_vec[i][1])
      cost2.append(solver_info.cost_iter_vec[i][2])
      cost3.append(solver_info.cost_iter_vec[i][3])
      cost4.append(solver_info.cost_iter_vec[i][4])
      cost5.append(solver_info.cost_iter_vec[i][5])
      cost6.append(solver_info.cost_iter_vec[i][6])
  except:
    pass

  index = list(range(solver_info.iter_count + 1))
  data_cost.data.update({
    'index': index,
    'cost0': cost0,
    'cost1': cost1,
    'cost2': cost2,
    'cost3': cost3,
    'cost4': cost4,
    'cost5': cost5,
    'cost6': cost6,
  })

  data_text.data.update({
  'cost_text': ['solver_condition={:d}'.format(solver_info.solver_condition)],
  })

  print("solver_flag = %d" %(flag))

  push_notebook()

button_play = widgets.Button(description="play")
display(button_play)

bkp.show(row(fig1, column(fig2, fig3), column(fig4, fig5, fig6, fig7)), notebook_handle=True)
slider_class = LateralMotionPlanningSlider(ctrl_info, silder_callback)


def play_thread(start_value, end_value, step_size):
    global click_flag
    value = start_value
    try:
        while (value <= end_value) and click_flag:
            slider_class.time_slider.value = value
            value += step_size
            time.sleep(0.1)
    except:
        pass
    finally:
        # clear_output(wait=True) # 清空输出结果
        click_flag = False # 将click_flag标记为False，表示播放结束

def on_button_play_clicked(b):
    global click_flag
    if click_flag: # 如果click_flag为True，说明正在播放，需要停止播放
        click_flag = False
    else: # 如果click_flag为False，说明未播放或已经播放完毕，需要开始播放
        click_flag = True
        start_value = bag_time_all
        end_value = ctrl_info['t'][-1]
        step_size = 0.1
        t = threading.Thread(target=play_thread, args=(start_value, end_value, step_size))
        t.start()

button_play.on_click(on_button_play_clicked)
