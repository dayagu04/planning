import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, planning_plan_pb2
from jupyter_pybind import collision_detection_py

# bag path and frame dt
bag_path = '/pnc_x86_ywwang33/dataAnalysis/test_24.00000'
frame_dt = 0.1 # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

collision_detection_py.Init()

bag_time0 = 0.0

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=bag_time0, step=frame_dt)

    ipywidgets.interact(slider_callback, bag_time = self.time_slider)
    
### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data)
  
  soc_state_msg_idx = local_view_data['data_index']['soc_state_msg_idx']
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']
  vs_msg_idx = local_view_data['data_index']['vs_msg_idx']
  wave_msg_idx = local_view_data['data_index']['wave_msg_idx']
  plan_msg_idx = local_view_data['data_index']['plan_msg_idx']

  soc_state_input = bag_loader.soc_state_msg['data'][soc_state_msg_idx]
  loc_msg_input = bag_loader.loc_msg['data'][loc_msg_idx]
  vs_msg_input = bag_loader.vs_msg['data'][vs_msg_idx]
  wave_msg_input = bag_loader.wave_msg['data'][wave_msg_idx]
  plan_msg_input = bag_loader.plan_msg['data'][plan_msg_idx]