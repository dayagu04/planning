import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from load_local_view import *
from load_lon_st_graph import *
from lib.load_ros_bag import LoadRosbag
from bokeh.resources import INLINE
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_50815/trigger/20250711/20250711-17-33-39/data_collection_CHERY_M32T_50815_EVENT_MANUAL_2025-07-11-17-33-39_no_camera.bag.1752665236.close-loop.scc.plan"

frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

#bag_loader = LoadCyberbag(bag_path)
bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
# JAC_S811 CHERY_T26 CHERY_E0X CHERY_M32T
global_var.set_value('car_type', 'CHERY_E0X')
global_var.set_value('g_is_display_enu', False)
fig1, local_view_data = load_local_view_figure()

velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status,topic_latency_fig= load_lon_global_figure(bag_loader)
load_measure_distance_tool(fig1)

# load lateral planning (behavior and motion)
pans, lon_plan_data = load_lon_plan_figure(fig1, velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status,topic_latency_fig)

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=0.1, step=frame_dt)
    self.prediction_obstacle_id = ipywidgets.Text(description='predict_id:')
    self.obstacle_polygon_id = ipywidgets.Text(description='polygon_id:')

    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                         prediction_obstacle_id = self.prediction_obstacle_id,
                                         obstacle_polygon_id = self.obstacle_polygon_id)


### sliders callback
def slider_callback(bag_time, prediction_obstacle_id, obstacle_polygon_id):
  kwargs = locals()
  update_select_obstacle_id(prediction_obstacle_id, obstacle_polygon_id, local_view_data)
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_lon_plan_data(bag_loader, bag_time, local_view_data, lon_plan_data)

  push_notebook()

bkp.show(row(fig1, pans), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
