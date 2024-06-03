import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_local_view import *
from lib.load_lat_plan import *

sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_tiggo9_06826/trigger/20240530/20240530-12-19-57/data_collection_CHERY_TIGGO9_06826_EVENT_MANUAL_2024-05-30-12-19-57_no_camera.record.1717070062.plan"

frame_dt = 0.1 # sec

# plot global figure?
global_fig_plot = False

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

# load lateral planning (behavior and motion)
fig1, fig2, fig3, fig4, fig5, fig6, fig7, fig8, fig9, lat_plan_data = load_lat_plan_figure(fig1)

fig_lat_offset = load_lateral_offset(bag_loader)
### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)


### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_lat_plan_data(bag_loader, bag_time, local_view_data, lat_plan_data, g_is_display_enu)

  push_notebook()

if global_fig_plot:
  bkp.show(row(fig1, fig7, column(fig2, fig9, fig3, fig4, fig5, fig6)), notebook_handle=True)
else:
  bkp.show(row(fig1, column(fig2, fig9, fig3, fig4, fig5, fig6, fig_lat_offset)), notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)
