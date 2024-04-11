import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, slot_management_info_pb2
from jupyter_pybind import slot_management_py

# bag path and frame dt
bag_path = '/data_cold/abu_zone/APA/planning-601f469a/test_8.00000'
frame_dt = 0.1 # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

# try before sliders
slot_management_py.Init()


# data source
data_slot_management_vec = ColumnDataSource(data = {'corner_point_y': [], 'corner_point_x': [],})
data_limiter_management_vec = ColumnDataSource(data = {'limiter_point_y': [], 'limiter_point_x': [],})
data_occupied_slot_vec = ColumnDataSource(data = {'occupied_slot_y': [], 'occupied_slot_x': [],})

# fig configs
fig1.multi_line('corner_point_y', 'corner_point_x', source = data_slot_management_vec, line_width = 1, line_color = 'blue', line_dash = 'solid',legend_label = 'test slots')
fig1.line('limiter_point_y', 'limiter_point_x', source = data_limiter_management_vec, line_width = 3, line_color = 'blue', line_dash = 'solid', legend_label = 'test limiter')
fig1.patches('occupied_slot_y', 'occupied_slot_x', source = data_occupied_slot_vec, fill_color = "blue", line_color = "blue", line_width = 1, fill_alpha = 0.3, legend_label = 'test occupied slot')
### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    self.force_apa_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "force_apa",min=0, max=1, value=0, step=1)
    self.force_clear_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "force_clear",min=0, max=1, value=0, step=1)
    self.max_slots_update_angle_dis_limit_deg_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "max_slots_update_angle_dis_limit_deg",min=0, max=60, value=20, step=0.5)
    self.max_slot_boundary_line_angle_dif_deg_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "max_slot_boundary_line_angle_dif_deg",min=0, max=60, value=20, step=1)
    self.outside_lon_dist_max_slot2mirror_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "max lon dif",min=0, max=3.0, value=1.6, step=0.05)
    self.outside_lon_dist_min_slot2mirror_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "min lon dif",min=-3.0, max=3.0, value=0.35, step=0.05)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                        force_apa = self.force_apa_slider,
                                        force_clear = self.force_clear_slider,
                                        max_slots_update_angle_dis_limit_deg = self.max_slots_update_angle_dis_limit_deg_slider,
                                        outside_lon_dist_max_slot2mirror = self.outside_lon_dist_max_slot2mirror_slider,
                                        outside_lon_dist_min_slot2mirror = self.outside_lon_dist_min_slot2mirror_slider,
                                        max_slot_boundary_line_angle_dif_deg=self.max_slot_boundary_line_angle_dif_deg_slider)

def load_slot_management_info(slot_management_info, force_clear):
  occupied_x_vec = []
  occupied_y_vec = []
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
    # print("slot.is_occupied = ", slot.is_occupied)
    if slot.is_occupied:
      occupied_x_vec.append(single_slot_x_vec)
      occupied_y_vec.append(single_slot_y_vec)

  limiter_x_vec = []
  limiter_y_vec = []
  for limiter in  slot_management_info.limiter_points:
      limiter_x_vec.append(limiter.x)
      limiter_y_vec.append(limiter.y)

  data_slot_management_vec.data.update({'corner_point_y': [], 'corner_point_x': [],})
  data_slot_management_vec.data.update({'corner_point_y': slots_y_vec, 'corner_point_x': slots_x_vec,})
  data_limiter_management_vec.data.update({'limiter_point_y': [], 'limiter_point_x': [],})
  data_limiter_management_vec.data.update({'limiter_point_y': limiter_y_vec, 'limiter_point_x': limiter_x_vec,})
  data_occupied_slot_vec.data.update({'occupied_slot_y': [], 'occupied_slot_x': [],})
  data_occupied_slot_vec.data.update({'occupied_slot_y': occupied_y_vec, 'occupied_slot_x': occupied_x_vec,})
### sliders callback
def slider_callback(bag_time, force_apa, force_clear,
                    max_slots_update_angle_dis_limit_deg,
                    max_slot_boundary_line_angle_dif_deg,
                    outside_lon_dist_max_slot2mirror,
                    outside_lon_dist_min_slot2mirror):
  kwargs = locals()
  update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data)

  soc_state_msg_idx = local_view_data['data_index']['soc_state_msg_idx']
  fus_parking_msg_idx = local_view_data['data_index']['fus_parking_msg_idx']
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']
  uss_wave_idx = local_view_data['data_index']['wave_msg_idx']
  uss_percept_idx = local_view_data['data_index']['uss_percept_msg_idx']

  soc_state_input = bag_loader.soc_state_msg['data'][soc_state_msg_idx]
  fus_parking_input = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx]
  loc_msg_input = bag_loader.loc_msg['data'][loc_msg_idx]
  uss_wave_input = bag_loader.wave_msg['data'][uss_wave_idx]
  uss_percept_input = bag_loader.uss_percept_msg['data'][uss_percept_idx]

  slot_management_py.UpdateBytesByParam(soc_state_input.SerializeToString(),
                                        fus_parking_input.SerializeToString(),
                                        loc_msg_input.SerializeToString(),
                                        uss_wave_input.SerializeToString(),
                                        uss_percept_input.SerializeToString(),
                                        force_apa, force_clear,
                                        max_slots_update_angle_dis_limit_deg,
                                        max_slot_boundary_line_angle_dif_deg,
                                        outside_lon_dist_max_slot2mirror,
                                        outside_lon_dist_min_slot2mirror)
  # print(fus_parking_input)
  # print("-------------------------------------")
  slot_management_info = slot_management_info_pb2.SlotManagementInfo()
  slot_management_info.ParseFromString(slot_management_py.GetOutputBytes())

  # print(slot_management_info)
  load_slot_management_info(slot_management_info, force_clear)

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
