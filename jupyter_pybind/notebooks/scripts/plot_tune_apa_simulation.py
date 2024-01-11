import sys, os, copy
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import planning_plan_pb2
from jupyter_pybind import apa_simulation_py

# bag path and frame dt
bag_path = '/data_cold/abu_zone/APA/0109cp/test_22.00000'
frame_dt = 0.1 # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

# try before sliders
apa_simulation_py.Init()

data_planning_tune = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})

data_sim_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_sim_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_tlane = ColumnDataSource(data = {'x':[-11.3691, -11.56], 'y':[-4.08125, -6.54795]})
# data_tlane = ColumnDataSource(data = {'x':[1.12249], 'y':[0.618948]})

fig1.circle('plan_path_y', 'plan_path_x', source = data_planning_tune, size=4, color='yellow', legend_label = 'tuned plan')
fig1.line('plan_path_y', 'plan_path_x', source = data_planning_tune, line_width = 6, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'tuned plan')
fig1.circle('y','x', source = data_sim_pos, size=8, color='red')
fig1.patch('car_yn', 'car_xn', source = data_sim_car, fill_color = "red", fill_alpha=0.25, line_color = "black", line_width = 1, legend_label = 'sim_car', visible = False)
fig1.patches('y_vec', 'x_vec', source = data_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sampled carbox', visible = False)
fig1.circle('y','x', source = data_tlane, size=8, color='green', legend_label='tlane')


### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    self.select_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='18%'), description= "select_id",min=0, max=20, value=0, step=1)
    self.force_plan_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "force_plan",min=0, max=1, value=0, step=1)
    self.is_reset_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_reset",min=0, max=1, value=0, step=1)
    self.is_complete_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_complete_path",min=0, max=1, value=0, step=1)
    self.sample_ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='25%'), description= "sample_ds",min=0.02, max=2.0, value=0.12, step=0.02)
    self.lon_pos_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "lon_pos_dif",min=-20.0, max=20.0, value=0.0, step=0.01)
    self.lat_pos_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "lat_pos_dif",min=-20.0, max=20.0, value=0.0, step=0.01)
    self.heading_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "heading_dif",min=-45.0, max=45.0, value=0.0, step=0.1)

    ipywidgets.interact(slider_callback,
                        bag_time = self.time_slider,
                        select_id = self.select_id_slider,
                        force_plan = self.force_plan_slider,
                        is_reset = self.is_reset_slider,
                        is_complete_path = self.is_complete_path_slider,
                        sample_ds = self.sample_ds_slider,
                        lon_pos_dif = self.lon_pos_dif_slider,
                        lat_pos_dif = self.lat_pos_dif_slider,
                        heading_dif = self.heading_dif_slider)

### sliders callback
def slider_callback(bag_time, select_id, force_plan, is_reset, is_complete_path, sample_ds, lon_pos_dif, lat_pos_dif, heading_dif):
  kwargs = locals()
  update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data)
  index_map = bag_loader.get_msg_index(bag_time)

  plan_debug_msg = bag_loader.plan_debug_msg['json'][index_map['plan_debug_msg_idx']]
  fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]
  wave_msg = bag_loader.wave_msg['data'][index_map['wave_msg_idx']]
  vs_msg = bag_loader.vs_msg['data'][index_map['vs_msg_idx']]
  soc_state_msg = bag_loader.soc_state_msg['data'][index_map['soc_state_msg_idx']]
  loc_msg = copy.deepcopy(bag_loader.loc_msg['data'][index_map['loc_msg_idx']])

  if soc_state_msg.current_state == 30:
    tlane_p0_x = plan_debug_msg['tlane_p0_x']
    tlane_p0_y = plan_debug_msg['tlane_p0_y']
    tlane_p1_x = plan_debug_msg['tlane_p1_x']
    tlane_p1_y = plan_debug_msg['tlane_p1_y']
    obstacle_x = plan_debug_msg['obstaclesX']
    obstacle_x.append(tlane_p0_x)
    obstacle_x.append(tlane_p1_x)
    obstacle_y = plan_debug_msg['obstaclesY']
    obstacle_y.append(tlane_p0_y)
    obstacle_y.append(tlane_p1_y)
    data_tlane.data.update({
      'x': obstacle_x,
      'y': obstacle_y,
    })
  else:
    data_tlane.data.update({
      'y': [],
      'x': [],
    })

  current_ego_x = loc_msg.pose.local_position.x
  current_ego_y = loc_msg.pose.local_position.y
  sim_ego_heading = loc_msg.pose.euler_angles.yaw + heading_dif / 57.2958

  sim_ego_x = current_ego_x + lon_pos_dif * math.cos(sim_ego_heading) - lat_pos_dif * math.sin(sim_ego_heading)
  sim_ego_y = current_ego_y + lon_pos_dif * math.sin(sim_ego_heading) + lat_pos_dif * math.cos(sim_ego_heading)

  loc_msg.pose.local_position.x = sim_ego_x
  loc_msg.pose.local_position.y = sim_ego_y
  loc_msg.pose.euler_angles.yaw = sim_ego_heading
  loc_msg.pose.heading = sim_ego_heading

  data_sim_pos.data.update({
    'x': [sim_ego_x],
    'y': [sim_ego_y],
  })

  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
    tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], sim_ego_x, sim_ego_y, sim_ego_heading)
    car_xn.append(tmp_x)
    car_yn.append(tmp_y)

  data_sim_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  res = apa_simulation_py.InterfaceUpdateParam(soc_state_msg.SerializeToString(),
                                    fus_parking_msg.SerializeToString(),
                                    loc_msg.SerializeToString(),
                                    vs_msg.SerializeToString(),
                                    wave_msg.SerializeToString(),
                                    select_id, force_plan, is_reset, is_complete_path, sample_ds)

  data_planning_tune.data = {'plan_path_x': [],
                             'plan_path_y': [],
                             'plan_path_heading': []}

  if res == True:
    tuned_planning_output = planning_plan_pb2.PlanningOutput()
    tuned_planning_output.ParseFromString(apa_simulation_py.GetPlanningOutput())

    plan_path_x = []
    plan_path_y = []
    plan_path_heading = []
    for i in range(len(tuned_planning_output.trajectory.trajectory_points)):
      plan_path_x.append(tuned_planning_output.trajectory.trajectory_points[i].x)
      plan_path_y.append(tuned_planning_output.trajectory.trajectory_points[i].y)
      plan_path_heading.append(tuned_planning_output.trajectory.trajectory_points[i].heading_yaw)

    data_planning_tune.data.update({
      'plan_path_x': plan_path_x,
      'plan_path_y': plan_path_y,
      'plan_path_heading': plan_path_heading,
    })

    # path ego car
    car_box_x_vec = []
    car_box_y_vec = []
    for k in range(len(tuned_planning_output.trajectory.trajectory_points)):
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

    print("tuned_gear_command = ", tuned_planning_output.gear_command)

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


