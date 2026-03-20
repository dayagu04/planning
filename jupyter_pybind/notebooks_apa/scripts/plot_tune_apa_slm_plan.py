import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')
from python_proto import slot_management_info_pb2
from struct_msgs.msg import PlanningOutput
from jupyter_pybind import diag_slot_planning_py

# bag path and frame dt
bag_path = '/home/xlwang71/Downloads/APA/20230919/3_17.00000'
frame_dt = 0.1 # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

origin_selected_id = bag_loader.fus_parking_msg['data'][int(len(bag_loader.fus_parking_msg['t'])/2)].select_slot_id
diag_slot_planning_py.Init()

# data source
data_planning_tune = ColumnDataSource(data = {'plan_traj_x':[],
                                              'plan_traj_y':[],})

data_slot_management_vec = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})

# fig configs
fig1.multi_line('corner_point_y', 'corner_point_x', source = data_slot_management_vec, line_width = 1, line_color = 'blue', line_dash = 'solid',legend_label = 'managed slots')

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=0, step=frame_dt)
    self.selected_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='25%'), description= "selected_id",min=0, max=100, value=origin_selected_id, step=1)
    self.force_planning_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='13%'), description= "force_planning",min=0, max=1, value=0, step=1)
    self.turn_on_force_last_seg_name_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='13%'), description= "turn_on_force_last_seg_name",min=0, max=1, value=0, step=1)
    self.force_last_seg_name_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='20%'), description= "force_last_seg_name",min=0, max=4, value=0, step=1)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                        selected_id = self.selected_id_slider,
                                        force_planning = self.force_planning_slider,
                                        turn_on_force_last_seg_name = self.turn_on_force_last_seg_name_slider,
                                        force_last_seg_name = self.force_last_seg_name_slider)

def load_slot_management_info(slot_management_info):
  slots_x_vec = []
  slots_y_vec = []
  for slot in slot_management_info.slot_info_vec:
    single_slot_x_vec = []
    single_slot_y_vec = []
    corner_points = slot.corner_points
    single_slot_x_vec.append(corner_points.corner_point[0].x)
    single_slot_y_vec.append(corner_points.corner_point[0].y)
    single_slot_x_vec.append(corner_points.corner_point[2].x)
    single_slot_y_vec.append(corner_points.corner_point[2].y)
    single_slot_x_vec.append(corner_points.corner_point[3].x)
    single_slot_y_vec.append(corner_points.corner_point[3].y)
    single_slot_x_vec.append(corner_points.corner_point[1].x)
    single_slot_y_vec.append(corner_points.corner_point[1].y)
    slots_x_vec.append(single_slot_x_vec)
    slots_y_vec.append(single_slot_y_vec)

  data_slot_management_vec.data.update({'corner_point_y': [], 'corner_point_x': [],})
  data_slot_management_vec.data.update({'corner_point_y': slots_y_vec, 'corner_point_x': slots_x_vec,})


### sliders callback
def slider_callback(bag_time, selected_id, force_planning,turn_on_force_last_seg_name, force_last_seg_name):
  kwargs = locals()
  update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data)

  soc_state_msg_idx = local_view_data['data_index']['soc_state_msg_idx']
  fus_parking_msg_idx = local_view_data['data_index']['fus_parking_msg_idx']
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']
  vs_msg_idx = local_view_data['data_index']['vs_msg_idx']
  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']

  soc_state_input = bag_loader.soc_state_msg['data'][soc_state_msg_idx]
  fus_parking_input = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx]
  loc_msg_input = bag_loader.loc_msg['data'][loc_msg_idx]
  vs_msg_input = bag_loader.vs_msg['data'][vs_msg_idx]
  planning_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]

  planning_output = PlanningOutput()
  slot_management_info = slot_management_info_pb2.SlotManagementInfo()

  last_seg_name = 0
  if turn_on_force_last_seg_name == 1:
    last_seg_name = force_last_seg_name
  else:
    pass
    # last_seg_name = planning_json['last_segment_name']

  try:
    diag_slot_planning_py.UpdateBytesByParam(soc_state_input.SerializeToString(),
                                      fus_parking_input.SerializeToString(),
                                      loc_msg_input.SerializeToString(),
                                      vs_msg_input.SerializeToString(), selected_id, force_planning, last_seg_name)

    planning_output.deserialize(diag_slot_planning_py.GetOutputBytes())
    slot_management_info.ParseFromString(diag_slot_planning_py.GetSlotManagementOutputBytes())
  except:
    pass
  # print(fus_parking_input)
  # print("-------------------------------------")



  # print(slot_management_info)
  load_slot_management_info(slot_management_info)

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
