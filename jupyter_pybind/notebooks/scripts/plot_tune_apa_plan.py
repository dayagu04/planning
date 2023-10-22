import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, planning_plan_pb2
from jupyter_pybind import diag_slot_planning_py

# bag path and frame dt
bag_path = '/home/xlwang71/Downloads/APA/1021-2015/test_11.00000'
frame_dt = 0.1 # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

origin_selected_id = bag_loader.fus_parking_msg['data'][int(len(bag_loader.fus_parking_msg['t'])/2)].select_slot_id
diag_slot_planning_py.Init()

data_planning_tune = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})
data_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'theta_vec':[]})
data_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_obstacles = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_car_circle = ColumnDataSource(data = {'circle_xn':[], 'circle_yn':[], 'circle_rn':[]})

fig1.patches('y_vec', 'x_vec', source = data_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sampled carbox')
fig1.line('plan_path_y', 'plan_path_x', source = data_planning_tune, line_width = 6, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'tuned plan')
fig1.multi_line('y_vec', 'x_vec', source = data_obstacles, line_width = 2, line_color = 'black', line_dash = 'solid', line_alpha = 1.0, legend_label = 'obstacles')
fig1.circle(y ='circle_xn', x ='circle_yn', radius = 'circle_rn', source=data_car_circle, line_alpha = 0.5, line_width = 1, line_color = "grey", fill_alpha=0, legend_label = 'car_circle_cc', visible = False)

# find bag time when planning at real test
# plan_msg_idx = 0
# if bag_loader.plan_msg['enable'] == True:
#   while plan_msg_idx < (len(bag_loader.plan_msg['t'])-1):
#     trajectory = bag_loader.plan_msg['data'][plan_msg_idx].trajectory
#     print(trajectory)
#     if len(trajectory.trajectory_points) > 1:
#       break
#     plan_msg_idx = plan_msg_idx + 1
# bag_time0 = plan_msg_idx * 0.1
bag_time0 = 0.0


### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=bag_time0, step=frame_dt)
    self.selected_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='25%'), description= "selected_id",min=0, max=100, value=origin_selected_id, step=1)
    self.force_replan_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='13%'), description= "force_replan",min=0, max=1, value=0, step=1)
    self.is_complete_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='13%'), description= "is_complete",min=0, max=1, value=0, step=1)
    self.is_replay_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='13%'), description= "is_replay",min=0, max=1, value=1, step=1)
    self.sample_ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='25%'), description= "sample_ds",min=0.025, max=2.0, value=0.5, step=0.025)
    self.chn_length_one_side_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='25%'), description= "chn_length_one_side",min=0.0, max=15.0, value=7.0, step=0.1)
    self.chn_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='25%'), description= "chn_width",min=0.0, max=15.0, value=5.3, step=0.1)
    self.slot_width_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='25%'), description= "slot_width_offset",min=0.0, max=5.0, value=0.15, step=0.05)


    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                        selected_id = self.selected_id_slider,
                                        force_replan = self.force_replan_slider,
                                        is_complete = self.is_complete_slider,
                                        is_replay = self.is_replay_slider,
                                        sample_ds = self.sample_ds_slider,
                                        chn_length_one_side = self.chn_length_one_side_slider,
                                        chn_width = self.chn_width_slider,
                                        slot_width_offset = self.slot_width_offset_slider)


### sliders callback
def slider_callback(bag_time, selected_id, force_replan, is_replay, is_complete, sample_ds, chn_length_one_side, chn_width, slot_width_offset):
  kwargs = locals()
  update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data)

  soc_state_msg_idx = local_view_data['data_index']['soc_state_msg_idx']
  fus_parking_msg_idx = local_view_data['data_index']['fus_parking_msg_idx']
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']
  vs_msg_idx = local_view_data['data_index']['vs_msg_idx']
  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
  plan_msg_idx = local_view_data['data_index']['plan_msg_idx']
  wave_msg_idx = local_view_data['data_index']['wave_msg_idx']

  soc_state_input = bag_loader.soc_state_msg['data'][soc_state_msg_idx]
  fus_parking_input = bag_loader.fus_parking_msg['data'][fus_parking_msg_idx]
  loc_msg_input = bag_loader.loc_msg['data'][loc_msg_idx]
  vs_msg_input = bag_loader.vs_msg['data'][vs_msg_idx]
  wave_msg_input = bag_loader.wave_msg['data'][wave_msg_idx]

  planning_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]
  planning_data = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx]
  origin_plan_output = bag_loader.plan_msg['data'][plan_msg_idx]


  # print("soc_state_input = ", soc_state_input)
  # print("fus_parking_input = ", fus_parking_input)
  # print("loc_msg_input = ", loc_msg_input)
  # print("vs_msg_input = ", vs_msg_input)
  planning_output = planning_plan_pb2.PlanningOutput()

  # if turn_on_plan_stm == 1:
  #   last_seg_name = plan_stm
  # else:
  #   last_seg_name = planning_json['last_segment_name']
  # print("plan_statemachine = ", plan_statemachine)

  # print(origin_plan_output)
  is_replan = 0
  if is_replay:
    is_replan = planning_json['is_replan']

  print("is_replan = ", is_replan)
  print("ego_yaw_deg = ", loc_msg_input.pose.euler_angles.yaw * 57.3)
  try:
    diag_slot_planning_py.UpdateBytesByParam(soc_state_input.SerializeToString(),
                                      fus_parking_input.SerializeToString(),
                                      loc_msg_input.SerializeToString(),
                                      vs_msg_input.SerializeToString(),
                                      wave_msg_input.SerializeToString(),
                                      planning_data.slot_management_info.SerializeToString(),
                                      selected_id, is_replan or force_replan, is_complete, chn_length_one_side,
                                      chn_length_one_side, chn_width, sample_ds, slot_width_offset)
    planning_output.ParseFromString(diag_slot_planning_py.GetOutputBytes())
  except:
    print("error")
    pass

  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []
  for i in range(len(planning_output.trajectory.trajectory_points)):
    plan_path_x.append(planning_output.trajectory.trajectory_points[i].x)
    plan_path_y.append(planning_output.trajectory.trajectory_points[i].y)
    plan_path_heading.append(planning_output.trajectory.trajectory_points[i].heading_yaw)

  data_planning_tune.data.update({
    'plan_path_x': plan_path_x,
    'plan_path_y': plan_path_y,
    'plan_path_heading': plan_path_heading,
  })

  # path ego car
  car_box_x_vec = []
  car_box_y_vec = []
  for k in range(len(plan_path_x)):
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], plan_path_x[k], plan_path_y[k], plan_path_heading[k])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)

  data_car_box.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })

  # print("planning_output:\n", planning_output)

  # obstacles
  obstacles_x_vec = diag_slot_planning_py.GetObstaclesX()
  obstacles_y_vec = diag_slot_planning_py.GetObstaclesY()



  obs_x_vec = []
  obs_y_vec = []
  for k in range(int(len(obstacles_x_vec) / 2)):
    line_x_vec = [obstacles_x_vec[2 * k], obstacles_x_vec[2 * k + 1]]
    line_y_vec = [obstacles_y_vec[2 * k], obstacles_y_vec[2 * k + 1]]
    obs_x_vec.append(line_x_vec)
    obs_y_vec.append(line_y_vec)

  data_obstacles.data.update({
    'x_vec': obs_x_vec,
    'y_vec': obs_y_vec,
  })

  # car_circle
  car_circle_vec = diag_slot_planning_py.GetCarCircleByEgoCarGlobalPb()
  car_circle_x_vec = []
  car_circle_y_vec = []
  car_circle_r_vec = []
  for i in range(len(car_circle_vec)):
    circle = car_circle_vec[i]
    car_circle_x_vec.append(circle[0])
    car_circle_y_vec.append(circle[1])
    car_circle_r_vec.append(circle[2])

  data_car_circle.data.update({
    'circle_xn': car_circle_x_vec,
    'circle_yn': car_circle_y_vec,
    'circle_rn': car_circle_r_vec,
  })

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
