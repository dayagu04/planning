import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
from lib.load_hybrid_ara_path import *

sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_18049/trigger/20241225/20241225-22-06-51/data_collection_CHERY_E0Y_18049_ALL_FILTER_2024-12-25-22-06-52_no_camera.bag.215-250.split.1738898757.open-loop.hpp.plan"

frame_dt = 0.1 # sec
# global_var.set_value('g_is_display_enu', True)
g_is_display_enu = global_var.get_value('g_is_display_enu')

# plot global figure?
global_fig_plot = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()
fig1.height = 1500
fig_lat_offset = load_lateral_offset(bag_loader)

# load lateral planning (behavior and motion)
fig1, fig2, hybrid_ara_path_data = load_hybrid_ara_path_figure(fig1)
load_measure_distance_tool(fig1)

def get_plan_debug_msg_idx(bag_loader, bag_time):
  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
    plan_debug_msg = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx]
  return plan_debug_msg_idx
max_expand_step = 1
for t in np.arange(0.0, max_time, frame_dt):
  plan_debug_msg_idx = get_plan_debug_msg_idx(bag_loader, t)
  hybrid_ara_expand = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].hybrid_ara_info.hybrid_ara_expand
  max_expand_step = max(max_expand_step, len(hybrid_ara_expand.hybrid_ara_expand))

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.expand_step_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='75%'), description= "expand_step",min=1, max=max_expand_step, value=1, step=1)
    self.expand_open_list_node_idx = ipywidgets.Text(description='expand_open_list_node_idx:')

    ipywidgets.interact(slider_callback,
                        bag_time = self.time_slider,
                        expand_step = self.expand_step_slider,
                        expand_open_list_node_idx = self.expand_open_list_node_idx)


### sliders callback
def slider_callback(bag_time, expand_step, expand_open_list_node_idx):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_hybrid_ara_path_data(fig2, bag_loader, bag_time, local_view_data, hybrid_ara_path_data, expand_step, expand_open_list_node_idx, g_is_display_enu)

  push_notebook()

if global_fig_plot:
  bkp.show(row(fig1, column(fig2, fig_lat_offset)), notebook_handle=True)
else:
  bkp.show(row(fig1), notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)
