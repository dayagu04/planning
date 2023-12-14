

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
sys.path.append('../..')
sys.path.append('../../..')
from lib.basic_layers import *
from lib.bag_loader import *
from lib.local_view_lib import *

# 先手动写死bag
bag_path = "/share/data_cold/abu_zone/S811-7/hpp_1215_crusing/hpp_1215_crusing_6.00000"
html_file = bag_path +".lonplan.html"

# bokeh创建的html在jupyter中显示
if isINJupyter():
    display(HTML("<style>.container { width:95% !important;  }</style>"))
    display(
        HTML('''<style>.widget-label {min-width: 25ex !important; }</style>'''))
    output_notebook()

# params 控制fig的样式
points_param = {
    'legend_label': 'obs_points',
    'size': 3,
    'color': 'firebrick'
}
ego_path_params = {
    'legend_label': 'real_path',
    'line_width': 3,
    'line_color': 'brown',
    'alpha': 0.5
}

origin_s_ref_params = {
    'legend_label': 'origin_s_ref',
    'line_width': 2,
    'line_color': 'green',
    'line_dash': 'dashed'
}
obs_lb_params = {
    'line_width': 2,
    'line_color': 'grey',
    'line_dash': 'solid',
    'legend_label': 'obs_lb'
}

obs_ub_params = {
    'line_width': 2,
    'line_color': 'grey',
    'line_dash': 'solid',
    'legend_label': 'obs_ub'
}

soft_bound_lb_params = {
    'line_width': 2,
    'line_color': 'yellow',
    'line_dash': 'solid',
    'legend_label': 'soft_bound_lb'
}

soft_bound_ub_params = {
    'line_width': 2,
    'line_color': '#FFA500',
    'line_dash': 'solid',
    'legend_label': 'soft_bound_ub'
}

s_ref_params = {
    'line_width': 2.5,
    'line_color': 'red',
    'line_dash': 'dashed',
    'legend_label': 's_ref'
}

s_plan_params = {
    'line_width': 2,
    'line_color': 'blue',
    'line_dash': 'solid',
    'legend_label': 's_plan'
}

obs_bound_params = {
    'line_width': 2,
    'line_color': 'firebrick',
    'line_dash': 'solid',
    'legend_label': 'obs_bounds'
}

obs_lead_params = {
    'line_width': 2,
    'line_color': 'green',
    'line_dash': 'solid',
    'legend_label': 'obs_lead'
}

obs_lb_pts_params = {
    'size':10,
    'fill_color': 'grey',
    'line_color': 'grey',
    'alpha': 0.7,
    'legend_label': 'obs_lb_pts'
}

obs_ub_pts_params = {
    'size':10,
    'fill_color': 'grey',
    'line_color': 'grey',
    'alpha': 0.7,
    'legend_label': 'obs_ub_pts'
}

v_ref_params = {
    'legend_label': 'v_ref',
    'line_width': 2,
    'line_color': 'red',
    'line_dash': 'dashed'
}
v_plan_params = {
    'legend_label': 'v_plan',
    'line_width': 2,
    'line_color': 'blue',
    'line_dash': 'solid'
}
v_lb_params = {
    'line_width': 2,
    'line_color': 'grey',
    'line_dash': 'solid',
    'legend_label': 'v_lb'
}
v_ub_params = {
    'line_width': 2,
    'line_color': 'grey',
    'line_dash': 'solid',
    'legend_label': 'v_ub'
}
v_lb_pts_params = {
    'size':10,
    'fill_color': 'grey',
    'line_color': 'grey',
    'alpha': 0.7,
    'legend_label': 'v_lb_pts'
}

v_ub_pts_params = {
    'size':10,
    'fill_color': 'grey',
    'line_color': 'grey',
    'alpha': 0.7,
    'legend_label': 'v_ub_pts'
}

a_plan_params = {
    'legend_label': 'a_plan',
    'line_width': 2,
    'line_color': 'blue',
    'line_dash': 'solid'
}
a_lb_params = {
    'line_width': 2,
    'line_color': 'grey',
    'line_dash': 'solid',
    'legend_label': 'a_lb'
}
a_ub_params = {
    'line_width': 2,
    'line_color': 'grey',
    'line_dash': 'solid',
    'legend_label': 'a_ub'
}
a_lb_pts_params = {
    'size':10,
    'fill_color': 'grey',
    'line_color': 'grey',
    'alpha': 0.7,
    'legend_label': 'a_lb_pts'
}

a_ub_pts_params = {
    'size':10,
    'fill_color': 'grey',
    'line_color': 'grey',
    'alpha': 0.7,
    'legend_label': 'a_ub_pts'
}

j_plan_params = {
    'legend_label': 'j_plan',
    'line_width': 2,
    'line_color': 'blue',
    'line_dash': 'solid'
}
j_lb_params = {
    'line_width': 2,
    'line_color': 'grey',
    'line_dash': 'solid',
    'legend_label': 'j_lb'
}
j_ub_params = {
    'line_width': 2,
    'line_color': 'grey',
    'line_dash': 'solid',
    'legend_label': 'j_ub'
}
j_lb_pts_params = {
    'size':10,
    'fill_color': 'grey',
    'line_color': 'grey',
    'alpha': 0.7,
    'legend_label': 'j_lb_pts'
}

j_ub_pts_params = {
    'size':10,
    'fill_color': 'grey',
    'line_color': 'grey',
    'alpha': 0.7,
    'legend_label': 'j_ub_pts'
}

target_vel_params = {
    'line_width': 1,
    'color': 'green',
    'legend_label': 'target_velocity'
}

ego_vel_params = {
    'line_width': 1,
    'color': 'blue',
    'legend_label': 'ego_velocity'
}

leadone_vel_params = {
    'line_width': 1,
    'color': 'red',
    'legend_label': 'leadone_velocity'
}

leadtwo_vel_params = {
    'line_width': 1,
    'color': 'black',
    'legend_label': 'leadtwo_velocity'
}

target_vel_start_stop_params = {
    'line_width': 1,
    'color': 'brown',
    'legend_label': 'target_velocity_start_stop'
}

lon_rt_table_params={
    'width': 700,
    'height':800,
}

ego_acc_params = {
    'line_width': 1,
    'color': 'blue',
    'legend_label': 'ego_acc'
}

acc_min_params = {
    'line_width': 1,
    'color': 'brown',
    'legend_label': 'acc_min'
}

acc_max_params = {
    'line_width': 1,
    'color': 'red',
    'legend_label': 'acc_max'
}

lead_one_dis_params = {
    'line_width': 1,
    'color': 'red',
    'legend_label': 'lead_one_dis'    
}

lead_two_dis_params = {
    'line_width': 1,
    'color': 'yellow',
    'legend_label': 'lead_two_dis'    
}

temp_lead_one_dis_params = {
    'line_width': 1,
    'color': 'blue',
    'legend_label': 'temp_lead_one_dis'    
}

temp_lead_two_dis_params = {
    'line_width': 1,
    'color': 'grey',
    'legend_label': 'temp_lead_two_dis'    
}

des_dis_rss_params = {
    'line_width': 1,
    'color': 'green',
    'legend_label': 'des_dis_rss' 
}

des_dis_cab_params = {
    'line_width': 1,
    'color': 'purple',
    'legend_label': 'des_dis_cab' 
}

behav_cost_params = {
    'line_width': 1,
    'color': 'red',
    'legend_label': 'behav_cost' 
}

motion_cost_params = {
    'line_width': 1,
    'color': 'blue',
    'legend_label': 'motion_cost' 
}

box_params = {
    'legend_label': 'obs_boxes',
    'line_color': 'red',
    'fill_color': 'red',
    'line_width': 1.5,
    'alpha': 0.7
}

# 判断是否在jupyter中运行
def isINJupyter():
    try:
        __file__
    except NameError:
        return True
    else:
        return False

class ScalarGenerator(DataGeneratorBase):
    def __init__(self, data, val_type, accu=False, name="Default Name"):
        xys, ts = self._convert(data, val_type)
        super().__init__(xys, ts, accu, name)

    def _convert(self, data, val_type):
        if len(data['t']) == 0:
            return [[], []]
        ts = []
        xs = []
        ys = []
        if val_type == 'ego_velocity':
            for i, v in enumerate(data["data"]):
                ts.append(data["t"][i])
                xs.append(data["t"][i])
                linear_velocity_from_wheel = math.sqrt(v.velocity.velocity_boot.vx * v.velocity.velocity_boot.vx + \
                v.velocity.velocity_boot.vy * v.velocity.velocity_boot.vy + \
                v.velocity.velocity_boot.vz * v.velocity.velocity_boot.vz)
                ys.append(round(linear_velocity_from_wheel, 2))
                # ys.append(round(v.pose.linear_velocity_from_wheel, 2))
        elif val_type == 'ego_acc':
            for i, v in enumerate(data["data"]):
                ts.append(data["t"][i])
                xs.append(data["t"][i])
                ys.append(round(v.long_acceleration, 2))
        else:
            for i, v in enumerate(data["json"]):
                ts.append(data["t"][i])
                xs.append(data["t"][i])
                if val_type == 'target_velocity':
                    ys.append(round(v['VisionLonBehavior_v_target'], 2))

                elif val_type == 'ego_velocity':
                    ys.append(round(v['VisionLonBehavior_v_target']-2, 2))

                elif val_type == 'leadone_velocity':
                    ys.append(round(v['RealTime_lead_one_velocity'], 2))

                elif val_type == 'leadtwo_velocity':
                    ys.append(round(v['RealTime_lead_two_velocity'], 2))

                elif val_type == 'acc_min':
                    ys.append(round(v['VisionLonBehavior_a_target_low'], 2))

                elif val_type == 'acc_max':
                    ys.append(round(v['VisionLonBehavior_a_target_high'], 2))

                elif val_type == 'lead_one_dis':
                    ys.append(round(v['RealTime_lead_one_distance'], 2))

                elif val_type == 'lead_two_dis':
                    ys.append(round(v['RealTime_lead_two_distance'], 2))

                elif val_type == 'temp_lead_one_dis':
                    ys.append(round(v['RealTime_temp_lead_one_distance'], 2)) 

                elif val_type == 'temp_lead_two_dis':
                    ys.append(round(v['RealTime_temp_lead_two_distance'], 2)) 

                elif val_type == 'des_dis_rss':
                    ys.append(round(v['RealTime_desired_distance_rss'], 2)) 

                elif val_type == 'des_dis_cab':
                    ys.append(round(v['RealTime_desired_distance_calibrate'], 2)) 

                elif val_type == 'behav_cost':
                    ys.append(round(v['RealTimeLonBehaviorCostTime'], 2))                 
                     
                elif val_type == 'motion_cost':
                    ys.append(round(v['RealTimeLonMotionCostTime'], 2)) 

                elif val_type == 'target_velocity_start_stop':
                    if hasattr(v, 'VisionLonBehavior_v_target_start_stop'):
                        ys.append(round(v['VisionLonBehavior_v_target_start_stop'], 2))
                    else:
                        ys.append(0)
                else:
                    pass

        xys = (xs, ys)
        return (xys, ts)

class BoundLineGenerator(DataGeneratorBase):
    def __init__(self, data, bound_type):
        xys, ts = self._convert(data, bound_type)
        super().__init__(xys, ts)
        self.bound_type = bound_type

    def _convert(self, data, bound_type):
        if data is None:
            return [[], []]

        ts = []
        xys = []
        for i, v in enumerate(data["data"]):
            ts.append(data["t"][i])
            one_t_vec = list(v.long_ref_path.t_list)
            if bound_type == "st_origin_s_ref" or bound_type == 't_pos_origin_ref':
                one_s_refs = []
                for item in (v.long_ref_path.s_refs):
                    one_s_refs.append(item.first)
                xys.append((one_t_vec, one_s_refs))

            elif bound_type == "st_obs_lb":
                obs_low_vec = []
                for idx in range(len(v.long_ref_path.bounds)):
                    low_bound = v.long_ref_path.bounds[idx].bound[0].lower
                    for one_bound in v.long_ref_path.bounds[idx].bound:
                        if one_bound.lower > low_bound:
                            low_bound = one_bound.lower
                    obs_low_vec.append(low_bound)
                xys.append((one_t_vec, obs_low_vec))

            elif bound_type == "st_obs_ub":
                obs_high_vec = []
                for idx in range(len(v.long_ref_path.bounds)):
                    high_bound = v.long_ref_path.bounds[idx].bound[0].upper
                    for one_bound in v.long_ref_path.bounds[idx].bound:
                        if one_bound.upper < high_bound:
                            high_bound = one_bound.upper
                    obs_high_vec.append(high_bound)
                xys.append((one_t_vec, obs_high_vec))

            elif bound_type == "st_s_soft_bound_ub":
                soft_bound_high_vec = []
                try:
                    for idx in range(len(v.long_ref_path.soft_bounds)):
                        soft_high_bound = v.long_ref_path.soft_bounds[idx].bound[0].upper
                        for one_bound in v.long_ref_path.soft_bounds[idx].bound:
                            if one_bound.upper < soft_high_bound:
                                soft_high_bound = one_bound.upper
                        soft_bound_high_vec.append(soft_high_bound)
                    xys.append((one_t_vec, soft_bound_high_vec))
                except:
                    xys.append(([], []))  
                
            elif bound_type == "st_s_soft_bound_lb":
                soft_bound_low_vec = []
                try:
                    for idx in range(len(v.long_ref_path.soft_bounds)):
                        soft_low_bound = v.long_ref_path.soft_bounds[idx].bound[0].lower
                        for one_bound in v.long_ref_path.soft_bounds[idx].bound:
                            if one_bound.lower > soft_low_bound:
                                soft_low_bound = one_bound.lower
                        soft_bound_low_vec.append(soft_low_bound)
                    xys.append((one_t_vec, soft_bound_low_vec))
                except:
                    xys.append(([], []))

            elif bound_type == "st_s_ref" or bound_type == "t_pos_ref":
                one_s_ref = list(v.longitudinal_motion_planning_input.ref_pos_vec)
                xys.append((one_t_vec, one_s_ref))

            elif bound_type == "st_s_plan" or bound_type == "t_pos_plan":
                one_pos_vec = list(v.longitudinal_motion_planning_output.pos_vec)
                xys.append((one_t_vec, one_pos_vec))

            elif bound_type == "st_obs_lead":
                lead_bound_vec = []
                end_idx = len(one_t_vec)
                for idx in range(len(v.long_ref_path.lon_lead_bounds)):
                    if(len(v.long_ref_path.lon_lead_bounds[idx].bound) == 0):
                        end_idx = idx
                        break
                    lead_bound = v.long_ref_path.lon_lead_bounds[idx].bound[0].s_lead
                    for one_bound in v.long_ref_path.lon_lead_bounds[idx].bound:
                        if one_bound.s_lead < lead_bound:
                            lead_bound = one_bound.s_lead
                    lead_bound_vec.append(lead_bound)
                xys.append((one_t_vec[:end_idx], lead_bound_vec))
                # print(len(lead_bound_vec))

            elif bound_type.startswith("st_obs_id_"):
                one_obs_vec = []
                one_obs_t_vec = []
                for idx in range(len(v.long_ref_path.bounds)):
                    for item in v.long_ref_path.bounds[idx].bound:
                        if item.bound_info.type == "obstacle" and str(item.bound_info.id) == bound_type[10:]:
                            one_obs_vec.append(item.upper)
                            one_obs_t_vec.append(one_t_vec[idx])
                xys.append((one_obs_t_vec, one_obs_vec))
                # print(len(one_obs_vec))

            elif bound_type == "sv_v_ref":
                one_s_refs = []
                one_v_refs = []
                for item in (v.long_ref_path.s_refs):
                    one_s_refs.append(item.first)
                for item in (v.long_ref_path.ds_refs):
                    one_v_refs.append(item.first)
                xys.append((one_s_refs, one_v_refs))

            elif bound_type == "sv_v_plan":
                one_v_plans = list(v.longitudinal_motion_planning_output.vel_vec)
                one_pos_plans = list(v.longitudinal_motion_planning_output.pos_vec)
                xys.append((one_pos_plans, one_v_plans))

            elif bound_type == "sv_v_lb":
                one_s_refs = []
                one_lb_vec = []
                for item in (v.long_ref_path.s_refs):
                    one_s_refs.append(item.first)
                for item in (v.long_ref_path.lon_bound_v.bound):
                    one_lb_vec.append(item.lower)
                xys.append((one_s_refs, one_lb_vec))

            elif bound_type == "sv_v_ub":
                one_s_refs = []
                one_ub_vec = []
                for item in (v.long_ref_path.s_refs):
                    one_s_refs.append(item.first)
                for item in (v.long_ref_path.lon_bound_v.bound):
                    one_ub_vec.append(item.upper)
                xys.append((one_s_refs, one_ub_vec))

            elif bound_type == "t_pos_max":
                if hasattr(v.longitudinal_motion_planning_input, 'pos_max_vec'):
                    one_pos_max = list(v.longitudinal_motion_planning_input.pos_max_vec)
                else:
                    one_pos_max = list(v.longitudinal_motion_planning_input.hard_pos_max_vec)
                xys.append((one_t_vec, one_pos_max))

            elif bound_type == "t_pos_min":
                if hasattr(v.longitudinal_motion_planning_input, 'pos_min_vec'):
                    one_pos_min = list(v.longitudinal_motion_planning_input.pos_min_vec)
                else:
                    one_pos_min = list(v.longitudinal_motion_planning_input.hard_pos_min_vec)
                xys.append((one_t_vec, one_pos_min))

            elif bound_type == "t_vel_max":
                one_vel_max = list(v.longitudinal_motion_planning_input.vel_max_vec)
                xys.append((one_t_vec, one_vel_max))

            elif bound_type == "t_vel_min":
                one_vel_min = list(v.longitudinal_motion_planning_input.vel_min_vec)
                xys.append((one_t_vec, one_vel_min))

            elif bound_type == "t_vel_ref":
                one_vel_ref = list(v.longitudinal_motion_planning_input.ref_vel_vec)
                xys.append((one_t_vec, one_vel_ref))

            elif bound_type == "t_vel_plan":
                one_vel_plan = list(v.longitudinal_motion_planning_output.vel_vec)
                xys.append((one_t_vec, one_vel_plan))

            elif bound_type == "t_acc_max":
                one_acc_max = list(v.longitudinal_motion_planning_input.acc_max_vec)
                xys.append((one_t_vec, one_acc_max))

            elif bound_type == "t_acc_min":
                one_acc_min = list(v.longitudinal_motion_planning_input.acc_min_vec)
                xys.append((one_t_vec, one_acc_min))

            elif bound_type == "t_acc_plan":
                one_acc_plan = list(v.longitudinal_motion_planning_output.acc_vec)
                xys.append((one_t_vec, one_acc_plan))

            elif bound_type == "t_jerk_max":
                one_jerk_max = list(v.longitudinal_motion_planning_input.jerk_max_vec)
                xys.append((one_t_vec, one_jerk_max))

            elif bound_type == "t_jerk_min":
                one_jerk_min = list(v.longitudinal_motion_planning_input.jerk_min_vec)
                xys.append((one_t_vec, one_jerk_min))

            elif bound_type == "t_jerk_plan":
                one_jerk_plan = list(v.longitudinal_motion_planning_output.jerk_vec)
                xys.append((one_t_vec, one_jerk_plan))
            else:
                pass              
        if len(xys) == 0:
                xys = [([], [])]

        return (xys, ts)

# 生成Text的类
class TextGenerator4Lon(DataGeneratorBase):
    def __init__(self, data, text_type, local=False):
        xys, ts = self._convert(data, text_type, local)
        super().__init__(xys, ts)
        self.txt = text_type

    def _convert(self, data, text_type, local):
        if data is None:
            return [[], []]

        ts = []
        xys = []
        if text_type == "real_time_json_value":
            planning_json_value_list = ['VisionLonBehavior_a_target_high', 'VisionLonBehavior_a_target_low', \
                          'VisionLonBehavior_v_limit_road', 'VisionLonBehavior_v_limit_in_turns','VisionLonBehavior_v_target', \
                          'VisionLonBehavior_lead_one_id', 'VisionLonBehavior_lead_one_dis', 'VisionLonBehavior_lead_one_vel', "VisionLonBehavior_v_target_lead_one", \
                          'VisionLonBehavior_lead_two_id', 'VisionLonBehavior_lead_two_dis', 'VisionLonBehavior_lead_two_vel', "VisionLonBehavior_v_target_lead_two", \
                          'VisionLonBehavior_temp_lead_one_id', 'VisionLonBehavior_temp_lead_one_dis', 'VisionLonBehavior_temp_lead_one_vel', "VisionLonBehavior_v_target_temp_lead_one", \
                          'VisionLonBehavior_temp_lead_two_id', 'VisionLonBehavior_temp_lead_two_dis', 'VisionLonBehavior_temp_lead_two_vel', "VisionLonBehavior_v_target_temp_lead_two", \
                          'VisionLonBehavior_potental_cutin_track_id', 'VisionLonBehavior_potental_cutin_v_target', "VisionLonBehavior_cutin_v_target", \
                          'VisionLonBehavior_stop_start_state', 'VisionLonBehavior_v_target_start_stop', \
                          'RealTime_v_ego', 'RealTime_gap_v_limit_lc', \
                          'RealTime_lead_one_id', 'RealTime_lead_one_distance', 'RealTime_lead_one_velocity', 'RealTime_lead_one_desire_vel', \
                          'RealTime_lead_two_id', 'RealTime_lead_two_distance', 'RealTime_lead_two_velocity', 'RealTime_lead_two_desire_vel', \
                          'RealTime_temp_lead_one_id', 'RealTime_temp_lead_one_distance', 'RealTime_temp_lead_one_velocity', 'RealTime_temp_lead_one_desire_vel', \
                          'RealTime_temp_lead_two_id', 'RealTime_temp_lead_two_distance', 'RealTime_temp_lead_two_velocity', 'RealTime_temp_lead_two_desire_vel', \
                          'RealTime_potential_cutin_track_id', 'RealTime_potential_cutin_v_target', \
                          "REALTIME_fast_lead_id", "REALTIME_slow_lead_id", "REALTIME_fast_car_cut_in_id", "REALTIME_slow_lead_id", \
                          "RealTime_desired_distance_rss", "RealTime_desired_distance_calibrate", "RealTimeLonBehaviorCostTime", "RealTimeLonMotionCostTime", \
                          "RealTime_stop_start_state", "RealTime_v_target_start_stop", "RealTime_STANDSTILL"]

            for i, v in enumerate(data["json"]):
                ts.append(data["t"][i])
                val_vec = []
                for key in planning_json_value_list:
                    if key in v.keys():
                        val_vec.append(v[key])
                    else:
                        val_vec.append(0)
                xys.append((planning_json_value_list, val_vec, [None] * len(planning_json_value_list)))
            if len(xys) == 0:
                xys = [([], [], [])]

        else:
            xys = [([], [], [])]
        return (xys,  ts)

def draw_lon_st(plan_debug_msg, layer_manager):
    #define figure
    hover = HoverTool(tooltips = [('t', '@pts_xs'),
     ('s', '@pts_ys')
    ])
    fig_st = bkp.figure(x_axis_label='t',
                        y_axis_label='s',
                        x_range = [-0.1, 7.0],
                        tools=[hover, 'pan,wheel_zoom,box_zoom,reset'],
                        width=600,
                        height=400,
                        match_aspect = True,
                        aspect_scale=1)

    #st graph data generator
    st_origin_s_ref = BoundLineGenerator(plan_debug_msg, "st_origin_s_ref")
    st_obs_lb = BoundLineGenerator(plan_debug_msg, "st_obs_lb")
    st_obs_ub = BoundLineGenerator(plan_debug_msg, "st_obs_ub")
    st_s_ref = BoundLineGenerator(plan_debug_msg, "st_s_ref")
    st_s_plan = BoundLineGenerator(plan_debug_msg, "st_s_plan")
    st_obs_lead = BoundLineGenerator(plan_debug_msg, "st_obs_lead")
    st_s_soft_bound_ub = BoundLineGenerator(plan_debug_msg, "st_s_soft_bound_ub")
    st_s_soft_bound_lb = BoundLineGenerator(plan_debug_msg, "st_s_soft_bound_lb")

    #obstacles st plotting
    obs_st_ids = []
    for ind in range(len(plan_debug_msg['data'])):
      for item in plan_debug_msg['data'][ind].long_ref_path.bounds:
         for one_bound in item.bound:
            if(one_bound.bound_info.type == 'obstacle' and one_bound.bound_info.id > 0 and one_bound.bound_info.id not in obs_st_ids):
               obs_st_ids.append(one_bound.bound_info.id)
    # print(obs_st_ids)

    for one_id in obs_st_ids:
        st_one_obs = BoundLineGenerator(plan_debug_msg, "st_obs_id_" + str(one_id))
        st_one_obs_layer = CurveLayer(fig_st, obs_bound_params)
        layer_manager.AddLayer(st_one_obs_layer, 'st_obs_source_' + str(one_id), st_one_obs, 'st_obs_' + str(one_id), 2)

    #st layers
    origin_s_ref_layer = CurveLayer(fig_st, origin_s_ref_params)
    obs_lb_layer = CurveLayer(fig_st, obs_lb_params)
    obs_ub_layer = CurveLayer(fig_st, obs_ub_params)
    obs_lb_pts_layer = TrianglePointsLayer(fig_st, 'triangle', obs_lb_pts_params)
    obs_ub_pts_layer = TrianglePointsLayer(fig_st, 'inverted_triangle', obs_ub_pts_params)
    s_ref_layer = CurveLayer(fig_st, s_ref_params)
    s_plan_layer = CurveLayer(fig_st, s_plan_params)
    obs_lead_layer = CurveLayer(fig_st, obs_lead_params)
    s_soft_bound_ub_layer = CurveLayer(fig_st, soft_bound_ub_params)
    s_soft_bound_lb_layer = CurveLayer(fig_st, soft_bound_lb_params)

    #add st layers
    layer_manager.AddLayer(
        origin_s_ref_layer, 'origin_s_ref_source', st_origin_s_ref, 'origin_s_ref', 2)
    layer_manager.AddLayer(
        obs_lb_layer, 'obs_lb_source', st_obs_lb, 'obs_lb', 2)
    layer_manager.AddLayer(
        obs_ub_layer, 'obs_ub_source', st_obs_ub, 'obs_ub', 2)
    layer_manager.AddLayer(
        obs_lb_pts_layer, 'obs_lb_pts_source', st_obs_lb, 'obs_pts_lb', 2)
    layer_manager.AddLayer(
        obs_ub_pts_layer, 'obs_ub_pts_source', st_obs_ub, 'obs_pts_ub', 2)
    layer_manager.AddLayer(
        s_ref_layer, 's_ref_source', st_s_ref, 's_ref', 2)
    layer_manager.AddLayer(
        s_plan_layer, 's_plan_source', st_s_plan, 's_plan', 2)
    layer_manager.AddLayer(
        obs_lead_layer, 'obs_lead_source', st_obs_lead, 'obs_lead', 2)
    layer_manager.AddLayer(
        s_soft_bound_ub_layer, 'soft_bound_ub_source', st_s_soft_bound_ub, 'soft_bound_ub', 2)
    layer_manager.AddLayer(
        s_soft_bound_lb_layer, 'soft_bound_lb_source', st_s_soft_bound_lb, 'soft_bound_lb', 2)

    fig_st.toolbar.active_scroll = fig_st.select_one(WheelZoomTool)
    fig_st.legend.click_policy = "hide"
    return fig_st

def draw_lon_sv(plan_debug_msg, layer_manager):
    #define figure
    hover = HoverTool(tooltips = [('pos', '@pts_xs'),
     ('vel', '@pts_ys')
    ])
    fig_sv = bkp.figure(x_axis_label='s',
                        y_axis_label='v',
                        width=600,
                        height=400,
                        tools=[hover, 'pan,wheel_zoom,box_zoom,reset'],
                        match_aspect = True,
                        aspect_scale=1)

    #sv graph data generator
    sv_v_ref = BoundLineGenerator(plan_debug_msg, "sv_v_ref")
    sv_v_plan = BoundLineGenerator(plan_debug_msg, "sv_v_plan")
    sv_v_lb = BoundLineGenerator(plan_debug_msg, "sv_v_lb")
    sv_v_ub = BoundLineGenerator(plan_debug_msg, "sv_v_ub")

    #sv layers
    v_ref_layer = CurveLayer(fig_sv, v_ref_params)
    v_lb_layer = CurveLayer(fig_sv, v_lb_params)
    v_ub_layer = CurveLayer(fig_sv, v_ub_params)
    v_lb_pts_layer = TrianglePointsLayer(fig_sv, 'triangle', v_lb_pts_params)
    v_ub_pts_layer = TrianglePointsLayer(fig_sv, 'inverted_triangle', v_ub_pts_params)
    v_plan_layer = CurveLayer(fig_sv, v_plan_params)

    #add sv layers
    layer_manager.AddLayer(
        v_ref_layer, 'v_ref_source', sv_v_ref, 'v_ref', 2)
    layer_manager.AddLayer(
        v_lb_layer, 'v_lb_source', sv_v_lb, 'v_lb', 2)
    layer_manager.AddLayer(
        v_ub_layer, 'v_ub_source', sv_v_ub, 'v_ub', 2)
    layer_manager.AddLayer(
        v_lb_pts_layer, 'v_lb_pts_source', sv_v_lb, 'v_pts_lb', 2)
    layer_manager.AddLayer(
        v_ub_pts_layer, 'v_ub_pts_source', sv_v_ub, 'v_pts_ub', 2)
    layer_manager.AddLayer(
        v_plan_layer, 'v_plan_source', sv_v_plan, 'v_plan', 2)

    fig_sv.legend.click_policy = "hide"
    fig_sv.toolbar.active_scroll = fig_sv.select_one(WheelZoomTool)
    return fig_sv

def draw_lon_tp(plan_debug_msg, layer_manager):
    #define figure
    hover = HoverTool(tooltips = [('time', '@pts_xs'),
     ('pos', '@pts_ys')
    ])
    fig_tp = bkp.figure(x_axis_label='t',
                        y_axis_label='pos',
                        x_range = [-0.1, 6.5],
                        tools=[hover, 'pan,wheel_zoom,box_zoom,reset'],
                        width=600, height=200)
    #t-pos graph data generator
    t_pos_max = BoundLineGenerator(plan_debug_msg, "t_pos_max")
    t_pos_min = BoundLineGenerator(plan_debug_msg, "t_pos_min")
    t_pos_ref = BoundLineGenerator(plan_debug_msg, "t_pos_ref")
    t_pos_plan = BoundLineGenerator(plan_debug_msg, "t_pos_plan")
    t_pos_origin_ref = BoundLineGenerator(plan_debug_msg, "t_pos_origin_ref")

    #t-pos layers
    tp_origin_ref_layer = CurveLayer(fig_tp, origin_s_ref_params)
    tp_lb_layer = CurveLayer(fig_tp, obs_lb_params)
    tp_ub_layer = CurveLayer(fig_tp, obs_ub_params)
    tp_lb_pts_layer = TrianglePointsLayer(fig_tp, 'triangle', obs_lb_pts_params)
    tp_ub_pts_layer = TrianglePointsLayer(fig_tp, 'inverted_triangle', obs_ub_pts_params)
    tp_ref_layer = CurveLayer(fig_tp, s_ref_params)
    tp_plan_layer = CurveLayer(fig_tp, s_plan_params)
    """ tp_layer_list =  []
    tp_layer_list.append(tp_origin_ref_layer)
    tp_layer_list.append(tp_lb_layer)
    tp_layer_list.append(tp_ub_layer)
    tp_layer_list.append(tp_lb_pts_layer)
    tp_layer_list.append(tp_ub_pts_layer)
    tp_layer_list.append(tp_ref_layer)
    tp_layer_list.append(tp_plan_layer) """

    #add t-pos layers
    layer_manager.AddLayer(
        tp_origin_ref_layer, 'tp_origin_ref_source', t_pos_origin_ref, 'tp_origin_ref', 2)
    layer_manager.AddLayer(
        tp_lb_layer, 'tp_lb_source', t_pos_min, 'tp_lb', 2)
    layer_manager.AddLayer(
        tp_ub_layer, 'tp_ub_source', t_pos_max, 'tp_ub', 2)
    layer_manager.AddLayer(
        tp_lb_pts_layer, 'tp_lb_pts_source', t_pos_min, 'tp_pts_lb', 2)
    layer_manager.AddLayer(
        tp_ub_pts_layer, 'tp_ub_pts_source', t_pos_max, 'tp_pts_ub', 2)
    layer_manager.AddLayer(
        tp_ref_layer, 'tp_ref_source', t_pos_ref, 'tp_ref', 2)
    layer_manager.AddLayer(
        tp_plan_layer, 'tp_plan_source', t_pos_plan, 'tp_plan', 2)

    """ for item in tp_layer_list:
        hover_tp = HoverTool(renderers=[item.plot],
                        tooltips=[('time', '@pts_xs'),
                                    ('val', '@pts_ys')],
                                    mode='vline'
                                    )
        fig_tp.add_tools(hover_tp) """

    fig_tp.legend.click_policy = "hide"
    fig_tp.toolbar.active_scroll = fig_tp.select_one(WheelZoomTool)
    return fig_tp

def draw_lon_tv(plan_debug_msg, layer_manager):
    #define figure
    hover = HoverTool(tooltips = [('time', '@pts_xs'),
     ('vel', '@pts_ys')
    ])
    fig_tv = bkp.figure(x_axis_label='t',
                        y_axis_label='vel',
                        x_range = [-0.1, 6.5],
                        tools=[hover, 'pan,wheel_zoom,box_zoom,reset'],
                        width=600, height=200)

    #t-vel graph data generator
    t_vel_max = BoundLineGenerator(plan_debug_msg, "t_vel_max")
    t_vel_min = BoundLineGenerator(plan_debug_msg, "t_vel_min")
    t_vel_ref = BoundLineGenerator(plan_debug_msg, "t_vel_ref")
    t_vel_plan = BoundLineGenerator(plan_debug_msg, "t_vel_plan")

    #t-vel layers
    tv_lb_layer = CurveLayer(fig_tv, v_lb_params)
    tv_ub_layer = CurveLayer(fig_tv, v_ub_params)
    tv_lb_pts_layer = TrianglePointsLayer(fig_tv, 'triangle', v_lb_pts_params)
    tv_ub_pts_layer = TrianglePointsLayer(fig_tv, 'inverted_triangle', v_ub_pts_params)
    tv_plan_layer = CurveLayer(fig_tv, v_plan_params)
    tv_ref_layer = CurveLayer(fig_tv, v_ref_params)

    #add t-vel layers
    layer_manager.AddLayer(
        tv_lb_layer, 'tv_lb_source', t_vel_min, 'tv_lb', 2)
    layer_manager.AddLayer(
        tv_ub_layer, 'tv_ub_source', t_vel_max, 'tv_ub', 2)
    layer_manager.AddLayer(
        tv_lb_pts_layer, 'tv_lb_pts_source', t_vel_min, 'tv_pts_lb', 2)
    layer_manager.AddLayer(
        tv_ub_pts_layer, 'tv_ub_pts_source', t_vel_max, 'tv_pts_ub', 2)
    layer_manager.AddLayer(
        tv_ref_layer, 'tv_ref_source', t_vel_ref, 'tv_ref', 2)
    layer_manager.AddLayer(
        tv_plan_layer, 'tv_plan_source', t_vel_plan, 'tv_plan', 2)

    fig_tv.toolbar.active_scroll = fig_tv.select_one(WheelZoomTool)
    fig_tv.legend.click_policy = "hide"
    return fig_tv


def draw_lon_ta(plan_debug_msg, layer_manager):
    #define figure
    hover = HoverTool(tooltips = [('time', '@pts_xs'),
     ('acc', '@pts_ys')
    ])
    fig_ta = bkp.figure(x_axis_label='t',
                        y_axis_label='acc',
                        x_range = [-0.1, 6.5],
                        tools=[hover, 'pan,wheel_zoom,box_zoom,reset'],
                        width=600, height=200)

    #t-acc graph data generator
    t_acc_max = BoundLineGenerator(plan_debug_msg, "t_acc_max")
    t_acc_min = BoundLineGenerator(plan_debug_msg, "t_acc_min")
    t_acc_plan =  BoundLineGenerator(plan_debug_msg, "t_acc_plan")

    #t-acc layers
    ta_lb_layer = CurveLayer(fig_ta, a_lb_params)
    ta_ub_layer = CurveLayer(fig_ta, a_ub_params)
    ta_lb_pts_layer = TrianglePointsLayer(fig_ta, 'triangle', a_lb_pts_params)
    ta_ub_pts_layer = TrianglePointsLayer(fig_ta, 'inverted_triangle', a_ub_pts_params)
    ta_plan_layer = CurveLayer(fig_ta, a_plan_params)

    #add t-acc layers
    layer_manager.AddLayer(
        ta_lb_layer, 'ta_lb_source', t_acc_min, 'ta_lb', 2)
    layer_manager.AddLayer(
        ta_ub_layer, 'ta_ub_source', t_acc_max, 'ta_ub', 2)
    layer_manager.AddLayer(
        ta_lb_pts_layer, 'ta_lb_pts_source', t_acc_min, 'ta_pts_lb', 2)
    layer_manager.AddLayer(
        ta_ub_pts_layer, 'ta_ub_pts_source', t_acc_max, 'ta_pts_ub', 2)
    layer_manager.AddLayer(
        ta_plan_layer, 'ta_plan_source', t_acc_plan, 'ta_plan', 2)

    fig_ta.toolbar.active_scroll = fig_ta.select_one(WheelZoomTool)
    fig_ta.legend.click_policy = "hide"
    return fig_ta

def draw_lon_tj(plan_debug_msg, layer_manager):
    #define figure
    hover = HoverTool(tooltips = [('time', '@pts_xs'),
     ('jerk', '@pts_ys')
    ])
    fig_tj = bkp.figure(x_axis_label='t',
                        y_axis_label='jerk',
                        x_range = [-0.1, 6.5],
                        tools=[hover, 'pan,wheel_zoom,box_zoom,reset'],
                        width=600, height=200)

    #t-jerk graph data generator
    t_jerk_max = BoundLineGenerator(plan_debug_msg, "t_jerk_max")
    t_jerk_min = BoundLineGenerator(plan_debug_msg, "t_jerk_min")
    t_jerk_plan =  BoundLineGenerator(plan_debug_msg, "t_jerk_plan")

    #t-jerk layers
    tj_lb_layer = CurveLayer(fig_tj, j_lb_params)
    tj_ub_layer = CurveLayer(fig_tj, j_ub_params)
    tj_lb_pts_layer = TrianglePointsLayer(fig_tj, 'triangle', j_lb_pts_params)
    tj_ub_pts_layer = TrianglePointsLayer(fig_tj, 'inverted_triangle', j_ub_pts_params)
    tj_plan_layer = CurveLayer(fig_tj, j_plan_params)

    #add t-jerk layers
    layer_manager.AddLayer(
        tj_lb_layer, 'tj_lb_source', t_jerk_min, 'tj_lb', 2)
    layer_manager.AddLayer(
        tj_ub_layer, 'tj_ub_source', t_jerk_max, 'tj_ub', 2)
    layer_manager.AddLayer(
        tj_lb_pts_layer, 'tj_lb_pts_source', t_jerk_min, 'tj_pts_lb', 2)
    layer_manager.AddLayer(
        tj_ub_pts_layer, 'tj_ub_pts_source', t_jerk_max, 'tj_pts_ub', 2)
    layer_manager.AddLayer(
        tj_plan_layer, 'tj_plan_source', t_jerk_plan, 'tj_plan', 2)

    fig_tj.toolbar.active_scroll = fig_tj.select_one(WheelZoomTool)
    fig_tj.legend.click_policy = "hide"
    return fig_tj

def draw_rt_vel(plan_debug_msg, loc_msg, layer_manager):
    #define figure
    fig_rtv = bkp.figure(title='车速',
                         x_axis_label='time/s',
                         y_axis_label='velocity/(m/s)',
                         width=600,height=225)

    rt_target_vel = ScalarGenerator(plan_debug_msg, 'target_velocity', accu=True, name="rt_target_vel")
    rt_ego_vel = ScalarGenerator(loc_msg, 'ego_velocity', accu=True, name="rt_ego_vel")
    rt_leadone_vel = ScalarGenerator(plan_debug_msg, 'leadone_velocity', accu=True, name="rt_leadone_vel")
    rt_leadtwo_vel = ScalarGenerator(plan_debug_msg, 'leadtwo_velocity', accu=True, name="rt_leadtwo_vel")
    rt_target_vel_start_stop = ScalarGenerator(plan_debug_msg, 'target_velocity_start_stop', accu=True, name="rt_target_vel_start_stop")

    target_vel_layer = CurveLayer(fig_rtv, target_vel_params)
    ego_vel_layer = CurveLayer(fig_rtv, ego_vel_params)
    leadone_vel_layer = CurveLayer(fig_rtv, leadone_vel_params)
    leadtwo_vel_layer = CurveLayer(fig_rtv, leadtwo_vel_params)
    target_vel_start_stop_layer = CurveLayer(fig_rtv, target_vel_start_stop_params)

    layer_manager.AddLayer(target_vel_layer, 'global_target_vel', rt_target_vel)
    layer_manager.AddLayer(ego_vel_layer, 'global_ego_vel', rt_ego_vel)
    layer_manager.AddLayer(leadone_vel_layer, 'global_leadone_vel', rt_leadone_vel)
    layer_manager.AddLayer(leadtwo_vel_layer, 'global_leadtwo_vel', rt_leadtwo_vel)
    layer_manager.AddLayer(target_vel_start_stop_layer, 'global_target_vel_start_stop', rt_target_vel_start_stop)

    fig_rtv.toolbar.active_scroll = fig_rtv.select_one(WheelZoomTool)
    fig_rtv.legend.click_policy = "hide"

    return fig_rtv

def draw_rt_acc(plan_debug_msg, vs_msg, layer_manager):
    #define figure
    fig_rta = bkp.figure(title='加速度',
                         x_axis_label='time/s',
                         y_axis_label='acc/(m/s2)',
                         width=600,height=225)

    rt_ego_acc = ScalarGenerator(vs_msg, 'ego_acc', accu=True, name="rt_ego_acc")
    rt_acc_min = ScalarGenerator(plan_debug_msg, 'acc_min', accu=True, name="rt_acc_min")
    rt_acc_max = ScalarGenerator(plan_debug_msg, 'acc_max', accu=True, name="rt_acc_max")

    ego_acc_layer = CurveLayer(fig_rta, ego_acc_params)
    acc_min_layer = CurveLayer(fig_rta, acc_min_params)
    acc_max_layer = CurveLayer(fig_rta, acc_max_params)

    layer_manager.AddLayer(ego_acc_layer, 'global_ego_acc', rt_ego_acc)
    layer_manager.AddLayer(acc_min_layer, 'global_acc_min', rt_acc_min)
    layer_manager.AddLayer(acc_max_layer, 'global_acc_max', rt_acc_max)

    fig_rta.toolbar.active_scroll = fig_rta.select_one(WheelZoomTool)
    fig_rta.legend.click_policy = "hide"

    return fig_rta

def draw_rt_distance(plan_debug_msg, vs_msg, layer_manager):
    fig_rt_dis = bkp.figure(title='距离',
                         x_axis_label='time/s',
                         y_axis_label='distance/m',
                         width=600,height=225)    

    rt_lead_one_dis = ScalarGenerator(plan_debug_msg, 'lead_one_dis', accu=True, name="rt_lead_one_dis")
    rt_lead_two_dis = ScalarGenerator(plan_debug_msg, 'lead_two_dis', accu=True, name="rt_lead_two_dis")
    rt_temp_lead_one_dis = ScalarGenerator(plan_debug_msg, 'temp_lead_one_dis', accu=True, name="rt_temp_lead_one_dis")
    rt_temp_lead_two_dis = ScalarGenerator(plan_debug_msg, 'temp_lead_two_dis', accu=True, name="rt_temp_lead_two_dis")
    rt_des_dis_rss = ScalarGenerator(plan_debug_msg, 'des_dis_rss', accu=True, name="rt_des_dis_rss")
    rt_des_dis_cab = ScalarGenerator(plan_debug_msg, 'des_dis_cab', accu=True, name="rt_des_dis_cab")

    lead_one_dis_layer = CurveLayer(fig_rt_dis, lead_one_dis_params)
    lead_two_dis_layer = CurveLayer(fig_rt_dis, lead_two_dis_params)
    temp_lead_one_dis_layer = CurveLayer(fig_rt_dis, temp_lead_one_dis_params)
    temp_lead_two_dis_layer = CurveLayer(fig_rt_dis, temp_lead_two_dis_params)
    des_dis_rss_layer = CurveLayer(fig_rt_dis, des_dis_rss_params)
    des_dis_cab_layer = CurveLayer(fig_rt_dis, des_dis_cab_params)

    layer_manager.AddLayer(lead_one_dis_layer, 'global_lead_one_dis', rt_lead_one_dis)
    layer_manager.AddLayer(lead_two_dis_layer, 'global_lead_two_dis', rt_lead_two_dis)
    layer_manager.AddLayer(temp_lead_one_dis_layer, 'global_temp_lead_one_dis', rt_temp_lead_one_dis)
    layer_manager.AddLayer(temp_lead_two_dis_layer, 'global_temp_lead_two_dis', rt_temp_lead_two_dis)
    layer_manager.AddLayer(des_dis_rss_layer, 'global_des_dis_rss', rt_des_dis_rss)
    layer_manager.AddLayer(des_dis_cab_layer, 'global_des_dis_cab', rt_des_dis_cab)

    fig_rt_dis.toolbar.active_scroll = fig_rt_dis.select_one(WheelZoomTool)
    fig_rt_dis.legend.click_policy = "hide"

    return fig_rt_dis

def draw_rt_cost(plan_debug_msg, vs_msg, layer_manager):
    fig_rt_cost = bkp.figure(title='耗时',
                         x_axis_label='time/s',
                         y_axis_label='time_cost/ms',
                         width=600,height=225)  

    rt_behav_cost = ScalarGenerator(plan_debug_msg, 'behav_cost', accu=True, name="rt_behav_cost")                       
    rt_motion_cost = ScalarGenerator(plan_debug_msg, 'motion_cost', accu=True, name="rt_motion_cost")

    rt_behav_cost_layer = CurveLayer(fig_rt_cost, behav_cost_params)
    rt_motion_cost_layer = CurveLayer(fig_rt_cost, motion_cost_params)

    layer_manager.AddLayer(rt_behav_cost_layer, 'global_behav_cost', rt_behav_cost)
    layer_manager.AddLayer(rt_motion_cost_layer, 'global_motion_cost', rt_motion_cost)

    fig_rt_cost.toolbar.active_scroll = fig_rt_cost.select_one(WheelZoomTool)
    fig_rt_cost.legend.click_policy = "hide"

    return fig_rt_cost

def draw_rt_table(plan_debug_msg, layer_manager):

    tab_rt = TextGenerator4Lon(plan_debug_msg, 'real_time_json_value')
    tab_attr_list = ['VisionLonAttr', 'VisionLonVal', 'others']
    tab_rt_layer = TableLayer(None, tab_attr_list, lon_rt_table_params)
    layer_manager.AddLayer(
        tab_rt_layer, 'rt_table_source', tab_rt, 'rt_table', 3)

    return tab_rt_layer.plot

def plotOnce(bag_path, html_file):
    # 加载bag
    try:
        dataLoader = LoadCyberbag(bag_path)
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
    fig_st = draw_lon_st(plan_debug_msg, layer_manager)
    fig_sv = draw_lon_sv(plan_debug_msg, layer_manager)
    fig_tp = draw_lon_tp(plan_debug_msg, layer_manager)
    fig_tv = draw_lon_tv(plan_debug_msg, layer_manager)
    fig_ta = draw_lon_ta(plan_debug_msg, layer_manager)
    fig_tj = draw_lon_tj(plan_debug_msg, layer_manager)
    fig_rtv = draw_rt_vel(plan_debug_msg, loc_msg, layer_manager)
    fig_rta = draw_rt_acc(plan_debug_msg, vs_msg, layer_manager)
    fig_rt_dis = draw_rt_distance(plan_debug_msg, vs_msg, layer_manager)
    fig_rt_cost = draw_rt_cost(plan_debug_msg, vs_msg, layer_manager)

    tab_rt = draw_rt_table(plan_debug_msg, layer_manager)

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

    #pan_lt = Panel(child=row(column(fig_st, fig_sv), column(fig_tp, fig_tv, fig_ta, fig_tj)), title="Longtime")
    pan_lt = Panel(child=row(column(fig_st, fig_sv), column(fig_tp, fig_tv, fig_ta, fig_tj)), title="Longtime")
    pan_rt = Panel(child=row(tab_rt, column(fig_rtv, fig_rta, fig_rt_dis, fig_rt_cost)), title="Realtime")
    #pan_rt = Panel(child=row(tab_rt), title="Realtime")
    pans = Tabs(tabs=[ pan_lt, pan_rt ])
    #pans = Tabs(tabs=[ pan_rt ])
    bkp.show(layout(car_slider, row(fig_lv, pans)))
    #bkp.show(layout(car_slider, row(pans)))

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
        html_file = bag_path + ".lon_plan" + ".html"
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
            html_file = bag_file + ".lon_plan" + ".html"
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
