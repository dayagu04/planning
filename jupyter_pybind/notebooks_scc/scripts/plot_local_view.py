import sys, os
sys.path.append("..")
sys.path.append("../lib/")
sys.path.append('../..')
sys.path.append('../../../')

# sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
from bokeh.resources import INLINE

# bag path and frame dt
bag_path = "/share//data_cold/abu_zone/autoparse/chery_e0y_20260/common_frame/20250325/20250325-08-42-37/data_collection_CHERY_E0Y_20260_ALL_MANUAL_2025-03-25-08-42-37_no_camera.bag"

frame_dt = 0.02 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

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
