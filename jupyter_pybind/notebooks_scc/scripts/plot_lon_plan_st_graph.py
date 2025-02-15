import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from load_local_view import *
from load_lon_st_graph import *
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_14529/trigger/20241218/20241218-16-16-50/data_collection_CHERY_E0Y_14529_EVENT_MANUAL_2024-12-18-16-16-50_no_camera.bag.1734955978.open-loop.scc.plan"

frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

#bag_loader = LoadCyberbag(bag_path)
bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status,topic_latency_fig= load_lon_global_figure(bag_loader)

# load lateral planning (behavior and motion)
pans, lon_plan_data = load_lon_plan_figure(fig1, velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status,topic_latency_fig)

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
