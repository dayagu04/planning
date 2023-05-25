import sys, os
sys.path.append("..")
from lib.load_lat_plan import *
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/root/bag/20230525222850.record.00000"
frame_dt = 0.02 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)


### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)


  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
