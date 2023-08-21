import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = '/home/xlwang71/Downloads/apa_bag/apa_2.00000'
frame_dt = 0.1 # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)


### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data)
  # plan_msg_idx = 0
  # if bag_loader.plan_msg['enable'] == True:
  #   while bag_loader.plan_msg['t'][plan_msg_idx] <= bag_time and plan_msg_idx < (len(bag_loader.plan_msg['t'])-2):
  #       plan_msg_idx = plan_msg_idx + 1

  # bag_loader.plan_msg['data'][plan_msg_idx]

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
