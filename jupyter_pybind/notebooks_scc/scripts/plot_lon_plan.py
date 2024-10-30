import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
from lib.load_lon_plan import *
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_18047/trigger/20241214/20241214-09-15-09/data_collection_CHERY_E0Y_18047_EVENT_MANUAL_2024-12-14-09-15-09_no_camera.bag.1736404530.open-loop.scc.plan"
frame_dt = 0.1 # sec
global_var.set_value('g_is_display_enu', True)
global_var.set_value('is_vis_sdmap', False)
global_var.set_value('is_vis_hpp', True)
global_var.set_value('is_vis_stop_line', True)
global_var.set_value('is_vis_zebra_crossing_line', True)
global_var.set_value('is_vis_merge_point', True)
# -

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

#bag_loader = LoadCyberbag(bag_path)
bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status,topic_latency_fig, tfl_status_fig = load_lon_global_figure(bag_loader)

# load lateral planning (behavior and motion)
pans, lon_plan_data = load_lon_plan_figure(fig1, velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status,topic_latency_fig, tfl_status_fig)

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)


### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_lon_plan_data(bag_loader, bag_time, local_view_data, lon_plan_data)

  push_notebook()

bkp.show(row(fig1, pans), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
