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
bag_path = '/data_cold/abu_zone/autoparse/jac_s811_96tj0/parking/20240123/20240123-15-24-08/park_in_data_collection_JAC_S811_96TJ0_MANUAL_ALL_2024-01-23-15-24-08_no_camera.record'
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
# data_tlane = ColumnDataSource(data = {'x':[-11.3691, -11.56], 'y':[-4.08125, -6.54795]})


for bag_time in np.arange(0.0, 35, 0.1):
  kwargs = locals()
  update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data)
  index_map = bag_loader.get_msg_index(bag_time)

  plan_debug_msg = bag_loader.plan_debug_msg['json'][index_map['plan_debug_msg_idx']]
  fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]
  wave_msg = bag_loader.wave_msg['data'][index_map['wave_msg_idx']]
  vs_msg = bag_loader.vs_msg['data'][index_map['vs_msg_idx']]
  soc_state_msg = bag_loader.soc_state_msg['data'][index_map['soc_state_msg_idx']]
  loc_msg = copy.deepcopy(bag_loader.loc_msg['data'][index_map['loc_msg_idx']])

  slot_management_info = bag_loader.plan_debug_msg['data'][index_map['plan_debug_msg_idx']].slot_management_info
  select_slot_id = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']].select_slot_id
  target_managed_slot_x_vec = []
  target_managed_slot_y_vec = []
  for i in range(len(slot_management_info.slot_info_vec)):
    maganed_slot_vec = slot_management_info.slot_info_vec[i]
    corner_point = maganed_slot_vec.corner_points.corner_point
    if maganed_slot_vec.id == select_slot_id:
      target_managed_slot_x_vec = [corner_point[0].x,corner_point[1].x,corner_point[2].x,corner_point[3].x]
      target_managed_slot_y_vec = [corner_point[0].y,corner_point[1].y,corner_point[2].y,corner_point[3].y]

  target_managed_slot_x_vec = []
  target_managed_slot_y_vec = []
  target_managed_limiter_x_vec = []
  target_managed_limiter_y_vec = []
  if soc_state_msg.current_state == 29 or soc_state_msg.current_state == 30:
    target_managed_slot_x_vec = plan_debug_msg['slot_corner_X']
    target_managed_slot_y_vec = plan_debug_msg['slot_corner_Y']
    target_managed_limiter_x_vec = plan_debug_msg['limiter_corner_X']
    target_managed_limiter_y_vec = plan_debug_msg['limiter_corner_Y']

  current_ego_x = loc_msg.pose.local_position.x
  current_ego_y = loc_msg.pose.local_position.y
  sim_ego_heading = loc_msg.pose.euler_angles.yaw + 0.0 / 57.2958

  sim_ego_x = current_ego_x + 0.0 * math.cos(sim_ego_heading) - 0.0 * math.sin(sim_ego_heading)
  sim_ego_y = current_ego_y + 0.0 * math.sin(sim_ego_heading) + 0.0 * math.cos(sim_ego_heading)

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
                                    0, False, False, False, False, 0.02, target_managed_slot_x_vec, target_managed_slot_y_vec,
                                    target_managed_limiter_x_vec, target_managed_limiter_y_vec)

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


