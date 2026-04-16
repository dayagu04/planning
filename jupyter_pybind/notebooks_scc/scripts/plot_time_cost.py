import sys, os
import warnings
import logging

warnings.filterwarnings('ignore', category=UserWarning, module='bokeh')
logging.getLogger('bokeh').setLevel(logging.ERROR)

sys.path.append("..")
sys.path.append("../lib/")
from load_local_view import *
from load_time_cost import *
from lib.load_ros_bag import LoadRosbag
from bokeh.resources import INLINE
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/bestune_e541_35401/trigger/20260415/20260415-22-29-22/data_collection_BESTUNE_E541_35401_EVENT_DOWNGRADE_2026-04-15-22-29-22_no_camera.bag"

frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

#bag_loader = LoadCyberbag(bag_path)
bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
# fig1, local_view_data = load_local_view_figure()

cost_time_fig= load_time_cost_fig(bag_loader)

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    pass
    # self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='100%'), description= "bag_time",min=0.0, max=max_time, value=0.1, step=frame_dt)
    # self.interactive_widget = ipywidgets.interact(slider_callback, bag_time = self.time_slider)

# ## sliders callback
# def slider_callback(bag_time, prediction_obstacle_id, obstacle_polygon_id):
#   kwargs = locals()
#   update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
#   push_notebook()

# slider_class = LocalViewSlider(slider_callback)

bkp.show((cost_time_fig), notebook_handle=True)
