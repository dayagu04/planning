import sys
import os
sys.path.append("..")
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')

from python_proto import st_search_decider_pb2
from lib.load_local_view_gs import *
from bokeh.models import ColumnDataSource, DataTable, TableColumn, TextInput
from ipywidgets import Layout
from typing import List
from python_proto import common_pb2
from jupyter_pybind import st_search_py


# bag path and frame dt
# bag_path="/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20240525/20240525-15-31-31/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-05-25-15-31-31.bag"
bag_path = "/data_cold/abu_zone/autoparse/jac_s811_72kx6/trigger/20240520/20240520-16-29-45/data_collection_JAC_S811_72KX6_EVENT_MANUAL_2024-05-20-16-29-45_no_camera.record.1716962554.plan"
# bag_path = "/data_cold/abu_zone/autoparse/jac_s811_72kx6/trigger/20240524/20240524-14-12-14/data_collection_JAC_S811_72KX6_EVENT_MANUAL_2024-05-24-14-12-14_no_camera.record.1716961967.plan"
frame_dt = 0.1  # sec

display(HTML("<style>.container {width:95% !important;  } </style>"))
output_notebook()


class STPoint:
    def __init__(self, s: float, t: float):
        self.s = s
        self.t = t


class STBoundary:
    def __init__(self, upper_points: List[STPoint], lower_points: List[STPoint], clock_wise_points: List[STPoint], tmin: float, tmax: float, lower_min_s: float,
                 lower_max_s: float, upper_min_s: float, upper_max_s: float):
        self.upper_points = upper_points
        self.lower_points = lower_points
        self.clock_wise_points = clock_wise_points
        self.tmin = 0.0
        self.tmax = 5.0
        self.lower_min_s = 0.0
        self.lower_max_s = 100
        self.upper_min_s = 0.0
        self.upper_max_s = 100
        self.id = -1


bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

fig1, fig2, fig3, fig4, fig5, fig6, fig10, fig11, lat_plan_data = load_lat_plan_figure(
    fig1)

coord_tf = coord_transformer()

# ### sliders config

ST_Boundaries = []
rect_1 = ColumnDataSource(data={'t': [], 's': [], 'id': []})
rect_2 = ColumnDataSource(data={'t': [], 's': [], 'id': []})
rect_3 = ColumnDataSource(data={'t': [], 's': [], 'id': []})
rect_4 = ColumnDataSource(data={'t': [], 's': [], 'id': []})
rect_5 = ColumnDataSource(data={'t': [], 's': [], 'id': []})

fig7 = bkp.figure(x_axis_label='t', y_axis_label='s', width=900,
                  height=600, x_range=(0, 5), y_range=(0, 200))
l1 = fig7.line('t', 's', source=rect_1, line_width=2, line_color='red',
               line_dash='solid', line_alpha=0.7, legend_label='obj-1')
l2 = fig7.line('t', 's', source=rect_2, line_width=2, line_color='blue',
               line_dash='solid', line_alpha=0.7, legend_label='obj-2')
l3 = fig7.line('t', 's', source=rect_3, line_width=2, line_color='grey',
               line_dash='solid', line_alpha=0.7, legend_label='obj-3')
l4 = fig7.line('t', 's', source=rect_4, line_width=2, line_color='yellow',
               line_dash='solid', line_alpha=0.7, legend_label='obj-4')
l5 = fig7.line('t', 's', source=rect_5, line_width=2, line_color='green',
               line_dash='solid', line_alpha=0.7, legend_label='obj-5')

hover1 = HoverTool(renderers=[l1], tooltips=[
                   ('time', '@t'), ('id', '@id')], mode='vline')
hover2 = HoverTool(renderers=[l2], tooltips=[
                   ('time', '@t'), ('id', '@id')], mode='vline')
hover3 = HoverTool(renderers=[l3], tooltips=[
                   ('time', '@t'), ('id', '@id')], mode='vline')
hover4 = HoverTool(renderers=[l4], tooltips=[
                   ('time', '@t'), ('id', '@id')], mode='vline')
hover5 = HoverTool(renderers=[l5], tooltips=[
                   ('time', '@t'), ('id', '@id')], mode='vline')
# fig7.multi_line("t", "s", source = rect, line_width = 2)
st_search_py.Init()

class LocalViewSlider:
    def __init__(self, slider_callback):
        self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="bag_time", min=0.1, max=max_time, value=0.1, step=frame_dt)
        self.show_print = ipywidgets.Checkbox(
            description="show_print", value=False)
        self.fix_result = ipywidgets.Checkbox(
            description="fix_result", value=False)
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
            width='80%', display='flex'), description="speed_limit_scale", min=0.0, max=2, value=1.5, step=0.1)
        self.v_cruise = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="v_cruise", min=0.0, max=40.0, value=25.0, step=0.1)
        self.collision_ttc = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="collision_ttc", min=0.0, max=8.0, value=2.0, step=0.1)
        self.min_collision_dist = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="min_collision_dist", min=0.0, max=20.0, value=5.0, step=0.1)
        self.max_collision_dist = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="max_collision_dist", min=0.0, max=20.0, value=10.0, step=0.1)
        self.s_step= ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="s_step", min=0.0, max=4.0, value=1.0, step=0.1)
        self.t_step= ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="t_step", min=0.0, max=1.0, value=0.1, step=0.1)
        self.vel_step= ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="vel_step", min=0.0, max=1.0, value=0.5, step=0.1)
        self.acc_search_step = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="acc_search_step", min=0.0, max=1.0, value=0.5, step=0.1)
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

        # cost config
        self.yield_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="yield_weight", min=0.0, max=10.0, value=1.0, step=0.1)
        self.overtake_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="overtake_weight", min=0.0, max=10.0, value=1.0, step=0.1)
        self.vel_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="vel_weight", min=-8.0, max=10.0, value=1.0, step=0.1)
        self.accel_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="accel_weight", min=0.0, max=10.0, value=1.0, step=0.1)
        self.accel_sign_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="accel_sign_weight", min=0.0, max=10.0, value=1.0, step=0.1)
        self.jerk_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="jerk_weight", min=0.0, max=10.0, value=1.0, step=0.1)
        self.virtual_yield_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="virtual_yield_weight", min=0.0, max=10.0, value=1.0, step=0.1)
        self.length_t_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="length_t_weight", min=0.0, max=10.0, value=1.0, step=0.1)
        self.hcost_t_weight = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="hcost_t_weight", min=-8.0, max=10.0, value=1.0, step=0.1)
        self.upper_trancation_time_buffer = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="upper_trancation_time_buffer", min=0.0, max=5.0, value=2.0, step=0.1)
        self.lower_trancation_time_buffer = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="lower_trancation_time_buffer", min=0.0, max=5.0, value=2.0, step=0.1)
        self.min_upper_distance_buffer = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="min_upper_distance_buffer", min=0.0, max=10.0, value=5.0, step=0.1)
        self.min_lower_distance_buffer = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="min_lower_distance_buffer", min=0.0, max=10.0, value=5.0, step=0.1)
        ipywidgets.interact(slider_callback,
                            show_print=self.show_print,
                            fix_result = self.fix_result,
                            max_acc_limit = self.max_acc_limit,
                            min_acc_limit = self.min_acc_limit,
                            max_jerk_limit =  self.max_jerk_limit,
                            min_jerk_limit =self.min_jerk_limit,
                            speed_limit = self.speed_limit,
                            speed_limit_scale = self.speed_limit_scale,
                            v_cruise = self.v_cruise,
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
                            yield_weight=self.yield_weight,
                            overtake_weight=self.overtake_weight,
                            vel_weight=self.vel_weight,
                            accel_weight=self.accel_weight,
                            accel_sign_weight=self.accel_sign_weight,
                            jerk_weight=self.jerk_weight,
                            virtual_yield_weight=self.virtual_yield_weight,
                            length_t_weight=self.length_t_weight,
                            hcost_t_weight=self.hcost_t_weight,
                            upper_trancation_time_buffer=self.upper_trancation_time_buffer,
                            lower_trancation_time_buffer=self.lower_trancation_time_buffer,
                            min_upper_distance_buffer=self.min_upper_distance_buffer,
                            min_lower_distance_buffer=self.min_lower_distance_buffer,
                            )
def load_st_info(st_infos, show_print):
    st_boundaries = []
    for i in range(len(st_infos)):
        upper_points = []
        lower_points = []
        clock_wise_points = []
        st_boundary = st_infos[i]
        if show_print:
          print("id:\n", st_boundary.obj_id)
        min_t = 0.0
        max_t = 5.0
        lower_min_s = 10000
        lower_max_s = -10000
        upper_min_s = 10000
        upper_max_s = -10000
        boundary = STBoundary(upper_points=upper_points, lower_points=lower_points,
                              clock_wise_points=clock_wise_points, tmin=min_t, tmax=max_t, lower_min_s=lower_min_s, lower_max_s=lower_max_s, upper_min_s=upper_min_s, upper_max_s=upper_max_s)

        for j in range(len(st_boundary.lower_point)):
            clock_wise_points.append(
                STPoint(st_boundary.lower_point[j].s, st_boundary.lower_point[j].t))
            lower_points.append(
                STPoint(st_boundary.lower_point[j].s, st_boundary.lower_point[j].t))
            min_t = min(min_t, st_boundary.lower_point[j].t)
            max_t = max(max_t, st_boundary.lower_point[j].t)
            lower_min_s = min(lower_min_s, st_boundary.lower_point[j].s)
            lower_max_s = max(lower_max_s, st_boundary.lower_point[j].s)
        for _j in range(len(st_boundary.upper_point)):
            clock_wise_points.append(
                STPoint(st_boundary.upper_point[_j].s, st_boundary.upper_point[_j].t))
            upper_points.append(
                STPoint(st_boundary.upper_point[_j].s, st_boundary.upper_point[_j].t))
            upper_min_s = min(upper_min_s, st_boundary.upper_point[_j].s)
            upper_max_s = max(upper_max_s, st_boundary.upper_point[_j].s)
            # print("upper bound\n", st_boundary.upper_point)
        boundary.lower_max_s = lower_max_s
        boundary.lower_min_s = lower_min_s
        boundary.upper_max_s = upper_max_s
        boundary.upper_min_s = upper_min_s
        boundary.id = st_boundary.obj_id
        if show_print:
          print("generate st boundary:\n", st_boundary.upper_point)
        st_boundaries.append(boundary)

    return st_boundaries


# slider_callback
def slider_callback(bag_time, show_print, fix_result, max_acc_limit,  min_acc_limit, max_jerk_limit,  min_jerk_limit,
                  speed_limit,  speed_limit_scale, v_cruise, collision_ttc,  min_collision_dist, max_collision_dist,
                  s_step,  t_step, vel_step, acc_search_step, max_search_time, acc_search_max,  acc_search_min,
                  vel_tolerance, propoper_accel_value,  planning_time_horizon,yield_weight,  overtake_weight,
                  vel_weight,accel_weight,  accel_sign_weight, jerk_weight,  virtual_yield_weight,length_t_weight,
                  hcost_t_weight, upper_trancation_time_buffer,lower_trancation_time_buffer, min_upper_distance_buffer,
                  min_lower_distance_buffer):
    kwargs = locals()
    coord_info = [0, 0, 0]
    coord_tf = update_local_view_data(
        fig1, bag_loader, bag_time, local_view_data)
    plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
    loc_msg_idx = local_view_data['data_index']['loc_msg_idx']

    try:
      st_search_decider_input = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].StSearchDeciderInfo
    except:
      print("Input failed!")

    st_search_decider_input_string = st_search_decider_input.SerializeToString()
    update_st_graph_input_status = st_search_py.UpdateStGraphBase(st_search_decider_input_string)
    if fix_result == 0:
       update_paras =  st_search_py.UpdateParams( max_acc_limit,  min_acc_limit, max_jerk_limit,
                                                    min_jerk_limit, speed_limit,  speed_limit_scale,
                                                    v_cruise, collision_ttc, min_collision_dist, max_collision_dist,
                                                    s_step,  t_step, vel_step, acc_search_step, max_search_time,
                                                    acc_search_max,  acc_search_min,vel_tolerance, propoper_accel_value,
                                                    planning_time_horizon)
       update_weight = st_search_py.UpdateWeight(yield_weight,  overtake_weight,vel_weight,accel_weight,  accel_sign_weight, jerk_weight,  virtual_yield_weight,length_t_weight,
                  hcost_t_weight, upper_trancation_time_buffer,lower_trancation_time_buffer, min_upper_distance_buffer,
                  min_lower_distance_buffer)

    st_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].st_boundaries_info
    ST_Boundaries = load_st_info(st_info, show_print)

    rect_1.data.update({'t': [], "s": [], "id": []})
    rect_2.data.update({'t': [], "s": [], "id": []})
    rect_3.data.update({'t': [], "s": [], "id": []})
    rect_4.data.update({'t': [], "s": [], "id": []})
    rect_5.data.update({'t': [], "s": [], "id": []})
    for i in range(len(ST_Boundaries)):
        s = []
        t = []
        id = []
        s.append(ST_Boundaries[i].lower_min_s)
        s.append(ST_Boundaries[i].lower_max_s)
        s.append(ST_Boundaries[i].upper_max_s)
        s.append(ST_Boundaries[i].upper_min_s)
        s.append(ST_Boundaries[i].lower_min_s)
        t.append(ST_Boundaries[i].tmin)
        t.append(ST_Boundaries[i].tmax)
        t.append(ST_Boundaries[i].tmax)
        t.append(ST_Boundaries[i].tmin)
        t.append(ST_Boundaries[i].tmin)
        for j in range(5):
            id.append(ST_Boundaries[i].id)

        if i == 0:
            rect_1.data.update({'t': t, "s": s, "id": id})
        if i == 1:
            rect_2.data.update({'t': t, "s": s, "id": id})
        if i == 2:
            rect_3.data.update({'t': t, "s": s, "id": id})
        if i == 3:
            rect_4.data.update({'t': t, "s": s, "id": id})
        if i == 4:
            rect_5.data.update({'t': t, "s": s, "id": id})
        if i == 5:
          print("Target lane obj is more than 5!")


    push_notebook()

slider_class = LocalViewSlider(slider_callback)
# load lateral planning
bkp.show(row(fig7, fig1), notebook_handle=True)

