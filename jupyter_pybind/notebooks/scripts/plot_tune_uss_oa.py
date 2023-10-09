import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, planning_plan_pb2
from jupyter_pybind import uss_obstacle_avoidance_py

# bag path and frame dt
bag_path = '/home/xlwang71/Downloads/APA/1007/test_16.00000'
frame_dt = 0.1 # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

uss_obstacle_avoidance_py.Init()

bag_time0 = 0.0

### sliders config
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

  # planning_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]

  uss_obstacle_avoidance_py.UpdateBytes(soc_state_input.SerializeToString(),
                                        loc_msg_input.SerializeToString(),
                                        vs_msg_input.SerializeToString(),
                                        wave_msg_input.SerializeToString(),
                                        plan_msg_input.SerializeToString())

  remain_dist = uss_obstacle_avoidance_py.GetRemainDist()
  car_arc_index = uss_obstacle_avoidance_py.GetMinDistCarArcIndex()
  uss_arc_index = uss_obstacle_avoidance_py.GetMinDistUssArcIndex()
  car_vertex = uss_obstacle_avoidance_py.GetCarVertex()
  uss_vertex = uss_obstacle_avoidance_py.GetUssVertex()
  uss_raw_dist = uss_obstacle_avoidance_py.GetUssRawDist()

  # print("car_vertex = ", car_vertex)
  # print("uss_vertex = ", uss_vertex)
  # print("uss_raw_dist = ", uss_raw_dist)

  print("remain_dist = ", remain_dist)
  print("car_arc_index = ", car_arc_index)
  print("uss_arc_index = ", uss_arc_index)

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
