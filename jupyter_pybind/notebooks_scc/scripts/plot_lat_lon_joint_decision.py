#!/usr/bin/env python3
import sys, os
import warnings
import logging

warnings.filterwarnings('ignore', category=UserWarning, module='bokeh')
warnings.filterwarnings('ignore', category=UserWarning, message='.*ColumnDataSource.*')
logging.getLogger('bokeh').setLevel(logging.ERROR)

sys.path.append("..")
sys.path.append("../lib/")
from load_local_view import *
from load_lat_lon_joint_decision import *
from lib.load_ros_bag import LoadRosbag
from bokeh.resources import INLINE
sys.path.append('../..')
sys.path.append('../../../')

bag_path =  "/data_cold/abu_zone/autoparse/chery_m32t_74563/trigger/20251105/20251105-11-54-33/data_collection_CHERY_M32T_74563_EVENT_FUNEXIT_2025-11-05-11-54-33_no_camera.bag.1762324516.close-loop.scc.plan"

frame_dt = 0.1

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()

global_var.set_value('car_type', 'CHERY_E0X')
global_var.set_value('g_is_display_enu', False)

fig1, local_view_data = load_local_view_figure()
load_measure_distance_tool(fig1)

# Load cost time figure and print average cost statistics
cost_time_fig, cost_time_data_source = load_joint_planner_cost_time_figure(bag_loader)

pans, joint_plan_data = load_joint_plan_figure(fig1, bag_loader)


class LocalViewSlider:
    def __init__(self, slider_callback):
        self.time_slider = ipywidgets.FloatSlider(
            layout=ipywidgets.Layout(width='100%'),
            description="bag_time",
            min=0.0,
            max=max_time,
            value=0.1,
            step=frame_dt)
        self.prediction_obstacle_id = ipywidgets.Text(description='predict_id:')
        self.obstacle_polygon_id = ipywidgets.Text(description='polygon_id:')

        self.interactive_widget = ipywidgets.interact(
            slider_callback,
            bag_time=self.time_slider,
            prediction_obstacle_id=self.prediction_obstacle_id,
            obstacle_polygon_id=self.obstacle_polygon_id)


def slider_callback(bag_time, prediction_obstacle_id, obstacle_polygon_id):
    kwargs = locals()
    update_select_obstacle_id(prediction_obstacle_id, obstacle_polygon_id, local_view_data)
    update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
    update_joint_plan_data(bag_loader, bag_time, local_view_data, joint_plan_data)
    push_notebook()


bkp.show(row(fig1, pans), notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)

