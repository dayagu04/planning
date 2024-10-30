import sys, os
sys.path.append("..")
sys.path.append("../lib/")
sys.path.append('../..')
sys.path.append('../../../')

# sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
# bag path and frame dt
bag_path = "/share/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20240612/20240612-16-55-01/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-06-12-16-55-01_no_camera.bag.1718343840.close-loop.plan"

frame_dt = 0.02 # sec
global_var.set_value('g_is_display_enu', False)

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='100%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)


### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  # plan_msg_idx = 0
  # if bag_loader.plan_msg['enable'] == True:
  #   while bag_loader.plan_msg['t'][plan_msg_idx] <= bag_time and plan_msg_idx < (len(bag_loader.plan_msg['t'])-2):
  #       plan_msg_idx = plan_msg_idx + 1

  # bag_loader.plan_msg['data'][plan_msg_idx]

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
# layout = column(fig1, sizing_mode='scale_width')
# bkp.show(layout)
slider_class = LocalViewSlider(slider_callback)
