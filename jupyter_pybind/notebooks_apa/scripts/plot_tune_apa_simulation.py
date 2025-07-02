import sys, os, copy
sys.path.append("..")
from io import BytesIO
from lib.load_local_view_parking import *
from lib.load_lon_plan import *
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')
sys.path.append('../../../build/devel/lib/python3/dis-packagers')

sys.path.append('python_proto')
from jupyter_pybind.python_proto import planning_debug_info_pb2
from jupyter_pybind import apa_simulation_py
from struct_msgs.msg import PlanningOutput, UssPerceptInfo, GroundLinePerceptionInfo, FusionObjectsInfo, FusionOccupancyObjectsInfo, UssWaveInfo, ParkingFusionInfo, VehicleServiceOutputInfo, FuncStateMachine, IFLYLocalization, ControlOutput

# e0y-1:  10034
# e0y-2:  04228
# e0y-3:  18047
# e0y-8:  14520
# e0y-9:  18049
# e0y-10: 20267
# bag path and frame dt
bag_path = '/data_cold/abu_zone/autoparse/chery_e0y_20267/trigger/20250701/20250701-16-50-10/park_in_data_collection_CHERY_E0Y_20267_ALL_FILTER_2025-07-01-16-50-11_no_camera.bag'

frame_dt = 0.1 # sec
parking_flag = True
global last_plan_pose_
last_plan_pose_ = []
plot_speed_graph = False

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

start_time = time.time()
bag_loader = LoadCyberbag(bag_path, parking_flag)

load_bag_end_time = time.time()
print(f"load bag time: {load_bag_end_time - start_time:.6f} seconds")

max_time = bag_loader.load_all_data()
load_all_data_end_time = time.time()
print(f"load all data time: {load_all_data_end_time - load_bag_end_time:.6f} seconds")

fig1, local_view_data = load_local_view_figure_parking()

if plot_speed_graph:
  # plot speed
  load_lon_global_data_figure(bag_loader)
  pans, lon_plan_data = create_lon_plan_figure(fig1)

source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=10, source=source, color='red', legend_label='measure tool')
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool')
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool')

# Define the JavaScript callback code
callback_code = """
    var x = cb_obj.x;
    var y = cb_obj.y;

    source.data['x'].push(x);
    source.data['y'].push(y);

    if (source.data['x'].length > 2) {
        source.data['x'].shift();
        source.data['y'].shift();
        source.data['x'].shift();
        source.data['y'].shift();
    }
    source.change.emit();

    if (source.data['x'].length >= 2) {
        var x1 = source.data['x'][source.data['x'].length - 2];
        var y1 = source.data['y'][source.data['y'].length - 2];
        var x2 = x;
        var y2 = y;
        var x3 = (x1 + x2) / 2;
        var y3 = (y1 + y2) / 2;

        var distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

        console.log("Distance between the last two points: " + distance);

        distance = distance.toFixed(4);
        text_source.data = {'x': [x3], 'y': [y3], 'text': [distance]};
        text_source.change.emit();

        line_source.data = {'x': [x1, x2], 'y': [y1, y2]};
        line_source.change.emit();
    }

    if (source.data['x'].length == 1) {
        text_source.data['x'].shift();
        text_source.data['y'].shift();
        text_source.data['text'].shift();
    }
    text_source.change.emit();
"""

# Create a CustomJS callback with the defined code
callback = CustomJS(args=dict(source=source, line_source=line_source, text_source=text_source), code=callback_code)

# Attach the callback to the Tap event on the plot
fig1.js_on_event(Tap, callback)

# try before sliders
apa_simulation_py.Init()

data_planning_tune = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})

data_complete_planning_tune = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})

data_sim_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_sim_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_sim_target_line = ColumnDataSource(data = {'x':[], 'y':[]})
data_sim_target_pos = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_simu_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_sim_obs = ColumnDataSource(data = {'obs_x':[], 'obs_y':[]})
data_sim_col_det_path = ColumnDataSource(data = {'x':[], 'y':[]})
data_sim_limiter = ColumnDataSource(data = {'x':[], 'y':[]})

data_sim_car_predict_traj_path = ColumnDataSource(data = {'x':[], 'y':[]})
data_sim_car_predict_traj_path_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
# this traj is not computed by optimizer, we fill it by zero speed
non_optimizer_traj = ColumnDataSource(data={'x': [], 'y': [], 'heading': [], })
stop_signs = ColumnDataSource(data = {'x':[], 'y':[]})
data_od_traj = ColumnDataSource(data = {'x':[], 'y':[]})


fig1.line('plan_path_y', 'plan_path_x', source = data_planning_tune, line_width = 6, line_color = 'green', line_dash = 'solid', line_alpha = 0.7, legend_label = 'sim_tuned_plan')
fig1.line('plan_path_y', 'plan_path_x', source = data_complete_planning_tune, line_width = 6, line_color = 'red', line_dash = 'dashed', line_alpha = 0.7, legend_label = 'sim_tuned_complete_plan', visible = False)
fig1.patch('car_yn', 'car_xn', source = data_sim_car, fill_color = "red", fill_alpha=0.25, line_color = "black", line_width = 1, legend_label = 'sim_car', visible = False)
fig1.patches('y_vec', 'x_vec', source = data_simu_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sim_sampled_carbox', visible = False)
fig1.patch('car_yn', 'car_xn', source = data_sim_target_pos, fill_color = "blue", line_color = "black", line_width = 1, line_alpha = 0.5, legend_label = 'data_sim_target_pos', visible = False)
fig1.line('y', 'x', source = data_sim_target_line, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.8, legend_label = 'data_sim_target_pos', visible = False)
fig1.line('y', 'x', source = data_sim_limiter, line_width = 3.0, line_color = 'red', line_dash = 'solid', line_alpha = 0.8, legend_label = 'data_sim_limiter', visible = False)
fig1.circle('obs_y', 'obs_x', source = data_sim_obs, size=6.0, color='red', legend_label='sim obs', visible = False)
# fig1.circle('y', 'x', source = data_sim_col_det_path, size=4, color='red', legend_label = 'sim_tuned_col_det_path')
# fig1.line('y', 'x', source = data_sim_col_det_path, line_width = 6, line_color = 'blue', line_dash = 'solid', line_alpha = 0.5, legend_label = 'sim_tuned_col_det_path')

fig1.circle('y', 'x', source = data_sim_car_predict_traj_path, size=4, color='orange', legend_label = 'sim_car_predict_traj_path', visible = False)
fig1.line('y', 'x', source = data_sim_car_predict_traj_path, line_width = 6, line_color = 'orange', line_dash = 'dashed', line_alpha = 0.5, legend_label = 'sim_car_predict_traj_path', visible = False)
fig1.patches('y_vec', 'x_vec', source = data_sim_car_predict_traj_path_car_box, fill_color = "#89FB89", fill_alpha = 0.0, line_color = "orange", line_width = 1, legend_label = 'sim_car_predict_traj_path', visible = False)
fig1.line('y', 'x', source = non_optimizer_traj, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.6, legend_label = 'non_optimizer_traj')
fig1.multi_line('y', 'x',source = stop_signs, line_width = 4.0, line_color = 'purple', line_dash = 'solid',legend_label = 'stop_signs',visible = True)
fig1.circle('y', 'x', source = data_od_traj, size=4.0, color='black', legend_label='fusion_objects', visible = True)


### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    self.vehicle_type_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "vehicle_type",min=0, max=3, value=2, step=1)
    self.sim_to_target_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "sim_to_target",min=0, max=1, value=0, step=1)
    self.plan_type_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "plan_type",min=0, max=1, value=0, step=1)
    self.pybind_state_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "pybind_state",min=0, max=50, value=0, step=1)
    self.use_obs_in_bag_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "use_obs_in_bag",min=0, max=1, value=1, step=1)
    self.select_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='18%'), description= "select_id",min=0, max=1000, value=0, step=1)
    self.force_plan_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "force_plan",min=0, max=1, value=0, step=1)
    self.car_inflation = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "car_inflation",min=0.0, max=0.15, value=0.0, step=0.01)
    self.is_path_optimization_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "path_optimization",min=0, max=1, value=0, step=1)
    self.is_cilqr_enable_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "cilqr_enable",min=0, max=1, value=1, step=1)
    self.is_reset_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_reset",min=0, max=1, value=0, step=1)
    self.is_complete_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_complete_path",min=0, max=1, value=0, step=1)
    self.sample_ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='25%'), description= "sample_ds",min=0.02, max=2.0, value=0.1, step=0.02)
    self.lon_pos_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "lon_pos_dif",min=-20.0, max=20.0, value=0.0, step=0.01)
    self.lat_pos_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "lat_pos_dif",min=-20.0, max=20.0, value=0.0, step=0.01)
    self.heading_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "heading_dif",min=-90.0, max=90.0, value=0.0, step=0.1)

    self.q_ref_xy_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_ref_xy", min=0.0, max=20000.0, value=100.0, step=10)
    self.q_ref_theta_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_ref_theta", min=0.0, max=100000.0, value=100.0, step=10)
    self.q_terminal_xy = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_terminal_xy", min=0.0, max=100000.0, value=5000.0, step=100)
    self.q_terminal_theta = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_terminal_theta", min=0.0, max=200000.0, value=168000.0, step=100)
    self.q_k_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="q_k", min=0.0, max=200.0, value=10.0, step=1)
    self.q_u_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_u", min=0.0, max=200.0, value=10.0, step=1)
    self.q_k_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="q_k_bound", min=0.0, max=2000.0, value=360.0, step=10)
    self.q_u_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_u_bound", min=0.0, max=2000.0, value=360.0, step=10)

    ipywidgets.interact(slider_callback,
                        bag_time = self.time_slider,
                        vehicle_type = self.vehicle_type_slider,
                        sim_to_target = self.sim_to_target_slider,
                        plan_type = self.plan_type_slider,
                        pybind_state = self.pybind_state_slider,
                        use_obs_in_bag = self.use_obs_in_bag_slider,
                        select_id = self.select_id_slider,
                        force_plan = self.force_plan_slider,
                        car_inflation = self.car_inflation,
                        is_path_optimization = self.is_path_optimization_slider,
                        is_cilqr_enable = self.is_cilqr_enable_slider,
                        is_reset = self.is_reset_slider,
                        is_complete_path = self.is_complete_path_slider,
                        sample_ds = self.sample_ds_slider,
                        lon_pos_dif = self.lon_pos_dif_slider,
                        lat_pos_dif = self.lat_pos_dif_slider,
                        heading_dif = self.heading_dif_slider,
                        q_ref_xy=self.q_ref_xy_slider,
                        q_ref_theta=self.q_ref_theta_slider,
                        q_terminal_theta=self.q_terminal_theta,
                        q_terminal_xy=self.q_terminal_xy,
                        q_k=self.q_k_slider,
                        q_u=self.q_u_slider,
                        q_k_bound=self.q_k_bound,
                        q_u_bound=self.q_u_bound,)

### sliders callback
def slider_callback(bag_time, vehicle_type, sim_to_target, plan_type, pybind_state, use_obs_in_bag, select_id, force_plan, car_inflation, is_path_optimization, is_cilqr_enable, is_reset, is_complete_path, sample_ds, lon_pos_dif, lat_pos_dif, heading_dif, q_ref_xy, q_ref_theta, q_terminal_xy, q_terminal_theta, q_k, q_u, q_k_bound, q_u_bound):
  kwargs = locals()

  if vehicle_type == 0:
    vehicle_type = 'JAC_S811'
  elif vehicle_type == 1:
    vehicle_type = 'CHERY_T26'
  elif vehicle_type == 2:
    vehicle_type = 'CHERY_E0X'
  elif vehicle_type == 3:
    vehicle_type = 'CHERY_M32T'

  update_local_view_data_parking(fig1, bag_loader, bag_time, vehicle_type, car_inflation, local_view_data)
  car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type, car_inflation)
  index_map = bag_loader.get_msg_index(bag_time)

  if bag_loader.plan_debug_msg['enable'] == True:
    plan_debug_msg = bag_loader.plan_debug_msg['json'][index_map['plan_debug_msg_idx']]
  else:
    plan_debug_msg = planning_debug_info_pb2.PlanningDebugInfo()

  if bag_loader.fus_parking_msg['enable'] == True:
    fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]
  else:
    fus_parking_msg = ParkingFusionInfo()

  if bag_loader.wave_msg['enable'] == True:
    wave_msg = bag_loader.wave_msg['data'][index_map['wave_msg_idx']]
  else:
    wave_msg = UssWaveInfo()

  if bag_loader.vs_msg['enable'] == True:
    vs_msg = bag_loader.vs_msg['data'][index_map['vs_msg_idx']]
  else:
    vs_msg = VehicleServiceOutputInfo()

  if bag_loader.ctrl_msg['enable'] == True:
    control_msg = bag_loader.ctrl_msg['data'][index_map['ctrl_msg_idx']]
  else:
    control_msg = ControlOutput()

  if bag_loader.soc_state_msg['enable'] == True:
    soc_state_msg = bag_loader.soc_state_msg['data'][index_map['soc_state_msg_idx']]
  else:
    soc_state_msg = FuncStateMachine()

  if bag_loader.loc_msg['enable'] == True:
    loc_msg = copy.deepcopy(bag_loader.loc_msg['data'][index_map['loc_msg_idx']])
  else:
    loc_msg = copy.deepcopy(IFLYLocalization())

  if bag_loader.uss_percept_msg['enable'] == True:
    uss_perception_msg = bag_loader.uss_percept_msg['data'][index_map['uss_percept_msg_idx']]
  else:
    uss_perception_msg = UssPerceptInfo()
  if bag_loader.fus_ground_line_msg['enable'] == True:
    gl_msg = bag_loader.fus_ground_line_msg['data'][index_map['fus_ground_line_msg_idx']]
  else:
    gl_msg = GroundLinePerceptionInfo()
  if bag_loader.fus_objects_msg['enable'] == True:
    fus_obj_msg = bag_loader.fus_objects_msg['data'][index_map['fus_objects_msg_idx']]
  else:
    fus_obj_msg = FusionObjectsInfo()
  if bag_loader.fus_occupancy_objects_msg['enable'] == True:
    fus_occ_obj_msg = bag_loader.fus_occupancy_objects_msg['data'][index_map['fus_occupancy_objects_msg_idx']]
  else:
    fus_occ_obj_msg = FusionOccupancyObjectsInfo()

  slot_management_info = bag_loader.plan_debug_msg['data'][index_map['plan_debug_msg_idx']].slot_management_info
  try:
    select_slot_id = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']].select_slot_id
  except:
    select_slot_id = -1
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
  obs_x_vec = []
  obs_y_vec = []
  if soc_state_msg.current_state >= 26:
    target_managed_slot_x_vec = plan_debug_msg['slot_corner_X']
    target_managed_slot_y_vec = plan_debug_msg['slot_corner_Y']
    target_managed_limiter_x_vec = plan_debug_msg['limiter_corner_X']
    target_managed_limiter_y_vec = plan_debug_msg['limiter_corner_Y']
    obs_x_vec = plan_debug_msg['obstaclesX']
    obs_y_vec = plan_debug_msg['obstaclesY']

  current_ego_x = loc_msg.position.position_boot.x
  current_ego_y = loc_msg.position.position_boot.y
  sim_ego_heading = loc_msg.orientation.euler_boot.yaw + heading_dif / 57.2958

  sim_ego_x = current_ego_x + lon_pos_dif * math.cos(sim_ego_heading) - lat_pos_dif * math.sin(sim_ego_heading)
  sim_ego_y = current_ego_y + lon_pos_dif * math.sin(sim_ego_heading) + lat_pos_dif * math.cos(sim_ego_heading)

  if len(last_plan_pose_) > 0 and sim_to_target == 1:
    sim_ego_x = last_plan_pose_[0]
    sim_ego_y = last_plan_pose_[1]
    sim_ego_heading = last_plan_pose_[2]

  loc_msg.position.position_boot.x = sim_ego_x
  loc_msg.position.position_boot.y = sim_ego_y
  loc_msg.orientation.euler_boot.yaw = sim_ego_heading

  data_sim_pos.data.update({
    'x': [sim_ego_x],
    'y': [sim_ego_y],
  })

  car_xn = []
  car_yn = []
  # car_xb is car param
  for i in range(len(car_xb)):
    tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], sim_ego_x, sim_ego_y, sim_ego_heading)
    car_xn.append(tmp_x)
    car_yn.append(tmp_y)

  data_sim_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  soc_state_msg_buff = BytesIO()
  soc_state_msg.serialize(soc_state_msg_buff)
  soc_state_msg_bytes = soc_state_msg_buff.getvalue()

  fus_parking_msg_buff = BytesIO()
  fus_parking_msg.serialize(fus_parking_msg_buff)
  fus_parking_msg_bytes = fus_parking_msg_buff.getvalue()

  loc_msg_buff = BytesIO()
  loc_msg.serialize(loc_msg_buff)
  loc_msg_bytes = loc_msg_buff.getvalue()

  vs_msg_buff = BytesIO()
  vs_msg.serialize(vs_msg_buff)
  vs_msg_bytes = vs_msg_buff.getvalue()

  control_msg_buff = BytesIO()
  control_msg.serialize(control_msg_buff)
  control_msg_bytes = control_msg_buff.getvalue()

  wave_msg_buff = BytesIO()
  wave_msg.serialize(wave_msg_buff)
  wave_msg_bytes = wave_msg_buff.getvalue()

  uss_perception_msg_buff = BytesIO()
  uss_perception_msg.serialize(uss_perception_msg_buff)
  uss_perception_msg_bytes = uss_perception_msg_buff.getvalue()

  ground_line_perception_msg_buff = BytesIO()
  gl_msg.serialize(ground_line_perception_msg_buff)
  ground_line_perception_msg_bytes = ground_line_perception_msg_buff.getvalue()

  fus_obj_msg_buff = BytesIO()
  fus_obj_msg.serialize(fus_obj_msg_buff)
  fus_obj_msg_bytes = fus_obj_msg_buff.getvalue()

  fus_occ_obj_msg_buff = BytesIO()
  fus_occ_obj_msg.serialize(fus_occ_obj_msg_buff)
  fus_occ_obj_msg_bytes = fus_occ_obj_msg_buff.getvalue()

  lat_path_optimizier_params = [q_ref_xy, q_ref_theta, q_terminal_xy, q_terminal_theta, q_k, q_u, q_k_bound, q_u_bound]

  res = apa_simulation_py.InterfaceUpdateParam(soc_state_msg_bytes,
                                    fus_parking_msg_bytes,
                                    loc_msg_bytes,
                                    vs_msg_bytes,
                                    wave_msg_bytes,
                                    uss_perception_msg_bytes,
                                    ground_line_perception_msg_bytes,
                                    fus_obj_msg_bytes,
                                    fus_occ_obj_msg_bytes,
                                    control_msg_bytes,
                                    plan_type,
                                    select_id, force_plan, is_path_optimization,
                                    is_cilqr_enable, is_reset, is_complete_path,
                                    sim_to_target, pybind_state, use_obs_in_bag, sample_ds,
                                    target_managed_slot_x_vec, target_managed_slot_y_vec,
                                    target_managed_limiter_x_vec, target_managed_limiter_y_vec,
                                    obs_x_vec, obs_y_vec, lat_path_optimizier_params)

  data_planning_tune.data = {'plan_path_x': [],
                             'plan_path_y': [],
                             'plan_path_heading': []}

  data_complete_planning_tune.data = {'plan_path_x': [],
                          'plan_path_y': [],
                          'plan_path_heading': []}

  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []
  plan_path_lat_buffer = []
  line_xn = []
  line_yn = []
  car_xn = []
  car_yn = []
  car_box_x_vec = []
  car_box_y_vec = []
  obstacle_x = []
  obstacle_y = []
  col_det_path_x = []
  col_det_path_y = []
  col_det_path_phi = []
  limiter_x = []
  limiter_y = []
  car_predict_x_vec = []
  car_predict_y_vec = []
  car_predict_heading_vec = []
  real_col_det_car_inflation = 0.0
  plan_traj_x_vec = []
  plan_traj_y_vec = []
  plan_traj_heading_vec = []
  plan_traj_lat_buffer_vec = []
  complete_x_vec, complete_y_vec = [], []

  if res == True:
    tuned_planning_output = PlanningOutput()
    tuned_planning_output.deserialize(apa_simulation_py.GetPlanningOutput())
    print("plan release slot id = ", tuned_planning_output.successful_slot_info_list)

    tuned_planning_debug_info = planning_debug_info_pb2.PlanningDebugInfo()
    tuned_planning_debug_info.ParseFromString(apa_simulation_py.GetPlanningDebugInfo())
    data_planning_debug = json.loads(tuned_planning_debug_info.data_json)
    if "obstaclesX" in data_planning_debug:
      obstacle_x = data_planning_debug["obstaclesX"]
      obstacle_y = data_planning_debug["obstaclesY"]

    if "col_det_path_x" in data_planning_debug:
      col_det_path_x = data_planning_debug["col_det_path_x"]
      col_det_path_y = data_planning_debug["col_det_path_y"]
      col_det_path_phi = data_planning_debug["col_det_path_phi"]

    if "limiter_corner_X" in data_planning_debug:
      limiter_x = data_planning_debug["limiter_corner_X"]
      limiter_y = data_planning_debug["limiter_corner_Y"]

    if "car_predict_x_vec" in data_planning_debug:
      car_predict_x_vec = data_planning_debug["car_predict_x_vec"]
      car_predict_y_vec = data_planning_debug["car_predict_y_vec"]
      car_predict_heading_vec = data_planning_debug["car_predict_heading_vec"]
      real_col_det_car_inflation =  data_planning_debug["car_real_time_col_lat_buffer"]

    if "plan_traj_x" in data_planning_debug:
      plan_traj_x_vec = data_planning_debug["plan_traj_x"]
      plan_traj_y_vec = data_planning_debug["plan_traj_y"]
      plan_traj_heading_vec = data_planning_debug["plan_traj_heading"]
      plan_traj_lat_buffer_vec = data_planning_debug["plan_traj_lat_buffer"]

    # print("obstaclesX = ",data_planning_debug["obstaclesX"])

    for i in range(len(tuned_planning_output.trajectory.trajectory_points)):
      plan_path_x.append(tuned_planning_output.trajectory.trajectory_points[i].x)
      plan_path_y.append(tuned_planning_output.trajectory.trajectory_points[i].y)
      plan_path_heading.append(tuned_planning_output.trajectory.trajectory_points[i].heading_yaw)
      plan_path_lat_buffer.append(0.0)

    if (len(plan_path_x) > 2):
      half_car_width = 0.9
      i = 0
      gear_change = False
      gear_change_index = -1
      while i <= len(plan_path_x) - 3:
        pt0 = [plan_path_x[i], plan_path_y[i]]
        pt1 = [plan_path_x[i+1], plan_path_y[i+1]]
        pt2 = [plan_path_x[i+2], plan_path_y[i+2]]
        A = np.array([pt0[0] - pt1[0], pt0[1] - pt1[1]])
        B = np.array([pt2[0] - pt1[0], pt2[1] - pt1[1]])
        # 计算点积
        dot_product = np.dot(A, B)

        # 计算模（长度）
        magnitude_A = np.linalg.norm(A)
        magnitude_B = np.linalg.norm(B)

        # 计算夹角（弧度）
        cos_theta = dot_product / (magnitude_A * magnitude_B)

        # theta_radians = np.arccos(cos_theta)
        # theta_degrees = np.degrees(theta_radians)

        if cos_theta > 0.0:
          gear_change = True
          break

        i = i + 1

      if gear_change == True:
        gear_change_index = i + 1

      last_x = plan_path_x[gear_change_index]
      last_y = plan_path_y[gear_change_index]
      last_heading = plan_path_heading[gear_change_index]
      print("simu last_x, last_y, last_heading = ", last_x, last_y, last_heading * 57.3)
      last_plan_pose_.clear()
      last_plan_pose_.append(last_x)
      last_plan_pose_.append(last_y)
      last_plan_pose_.append(last_heading)
      for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], last_x, last_y, last_heading)
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)

      heading_vec = [math.cos(last_heading), math.sin(last_heading)]
      norm_vec_1 = [-half_car_width * heading_vec[1], half_car_width * heading_vec[0]]
      norm_vec_2 = [half_car_width * heading_vec[1], -half_car_width * heading_vec[0]]
      x1 = last_x + norm_vec_1[0]
      y1 = last_y + norm_vec_1[1]
      x2 = last_x + norm_vec_2[0]
      y2 = last_y + norm_vec_2[1]
      line_xn = [x1, x2]
      line_yn = [y1, y2]

    plan_traj_x_list = []
    plan_traj_y_list = []
    plan_traj_heading_list = []
    plan_traj_lat_buffer_list = []
    if isinstance(plan_traj_x_vec, str) and len(plan_traj_x_vec) > 0:
      plan_traj_x_list = [float(x) for x in plan_traj_x_vec.split(',')]
    elif not isinstance(plan_traj_x_vec, str):
      plan_traj_x_list = plan_traj_x_vec

    if isinstance(plan_traj_y_vec, str) and len(plan_traj_y_vec) > 0:
      plan_traj_y_list = [float(y) for y in plan_traj_y_vec.split(',')]
    elif not isinstance(plan_traj_y_vec, str):
      plan_traj_y_list = plan_traj_y_vec

    if isinstance(plan_traj_heading_vec, str) and len(plan_traj_heading_vec) > 0:
      plan_traj_heading_list = [float(heading) for heading in plan_traj_heading_vec.split(',')]
    elif not isinstance(plan_traj_heading_vec, str):
      plan_traj_heading_list = plan_traj_heading_vec

    if isinstance(plan_traj_lat_buffer_vec, str) and len(plan_traj_lat_buffer_vec) > 0:
      plan_traj_lat_buffer_list = [float(lat_buffer) for lat_buffer in plan_traj_lat_buffer_vec.split(',')]
    elif not isinstance(plan_traj_lat_buffer_vec, str):
      plan_traj_lat_buffer_list = plan_traj_lat_buffer_vec

    # path ego car
    if len(plan_traj_x_list) < 2:
      for i in range(len(plan_path_x)):
        car_xn_temp = []
        car_yn_temp = []
        for j in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[j], car_yb[j], plan_path_x[i], plan_path_y[i], plan_path_heading[i])
          car_xn_temp.append(tmp_x)
          car_yn_temp.append(tmp_y)
        if i % 1 == 0 or i == len(plan_path_x) - 1:
          car_box_x_vec.append(car_xn_temp)
          car_box_y_vec.append(car_yn_temp)
    else:
      for i in range(len(plan_traj_x_list)):
        car_xn_temp = []
        car_yn_temp = []
        complete_x_vec.append(plan_traj_x_list[i])
        complete_y_vec.append(plan_traj_y_list[i])
        car_xb_temp, car_yb_temp, wheel_base_temp = load_car_params_patch_parking(vehicle_type, plan_traj_lat_buffer_list[i])
        for j in range(len(car_xb_temp)):
          tmp_x, tmp_y = local2global(car_xb_temp[j], car_yb_temp[j], plan_traj_x_list[i], plan_traj_y_list[i], plan_traj_heading_list[i])
          car_xn_temp.append(tmp_x)
          car_yn_temp.append(tmp_y)
        if i % 2 == 0 or i == len(plan_traj_x_list) - 1 or i == gear_change_index:
          car_box_x_vec.append(car_xn_temp)
          car_box_y_vec.append(car_yn_temp)

    print("tuned_gear_command = ", tuned_planning_output.gear_command)

  data_planning_tune.data.update({
    'plan_path_x': plan_path_x,
    'plan_path_y': plan_path_y,
    'plan_path_heading': plan_path_heading,
  })

  data_complete_planning_tune.data.update({
    'plan_path_x': complete_x_vec,
    'plan_path_y': complete_y_vec,
    'plan_path_heading': complete_y_vec,
  })

  data_simu_car_box.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })

  data_sim_target_pos.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  data_sim_target_line.data.update({
    'x' : line_xn,
    'y' : line_yn,
  })

  obstacle_x_list, obstacle_y_list = [], []

  if isinstance(obstacle_x, str) and len(obstacle_x) > 0:
    obstacle_x_list = [float(x) for x in obstacle_x.split(',')]
  elif not isinstance(obstacle_x, str):
    obstacle_x_list = obstacle_x

  if isinstance(obstacle_y, str) and len(obstacle_y) > 0:
    obstacle_y_list = [float(y) for y in obstacle_y.split(',')]
  elif not isinstance(obstacle_y, str):
    obstacle_y_list = obstacle_y

  data_sim_obs.data.update({
    'obs_x': obstacle_x_list,
    'obs_y': obstacle_y_list,
  })

  col_det_path_x_list = []
  col_det_path_y_list = []
  if isinstance(col_det_path_x, str) and len(col_det_path_x) > 0:
    col_det_path_x_list = [float(x) for x in col_det_path_x.split(',')]
  elif not isinstance(col_det_path_x, str):
    col_det_path_x_list = col_det_path_x

  if isinstance(col_det_path_y, str) and len(col_det_path_y) > 0:
    col_det_path_y_list = [float(y) for y in col_det_path_y.split(',')]
  elif not isinstance(col_det_path_x, str):
    col_det_path_y_list = col_det_path_y

  data_sim_col_det_path.data.update({
    'x': col_det_path_x_list,
    'y': col_det_path_y_list,
  })

  limiter_x_list = []
  limiter_y_list = []
  if isinstance(limiter_x, str) and len(limiter_x) > 0:
    limiter_x_list = [float(x) for x in limiter_x.split(',')]
  elif not isinstance(limiter_x, str):
    limiter_x_list = limiter_x

  if isinstance(limiter_y, str) and len(limiter_y) > 0:
    limiter_y_list = [float(y) for y in limiter_y.split(',')]
  elif not isinstance(limiter_y, str):
    limiter_y_list = limiter_y

  data_sim_limiter.data.update({
    'x': limiter_x_list,
    'y': limiter_y_list,
  })

  car_predict_x_vec_list = []
  car_predict_y_vec_list = []
  car_predict_heading_vec_list = []
  if isinstance(car_predict_x_vec, str) and len(car_predict_x_vec) > 0:
    car_predict_x_vec_list = [float(x) for x in car_predict_x_vec.split(',')]
  elif not isinstance(car_predict_x_vec, str):
    car_predict_x_vec_list = car_predict_x_vec

  if isinstance(car_predict_y_vec, str) and len(car_predict_y_vec) > 0:
    car_predict_y_vec_list = [float(y) for y in car_predict_y_vec.split(',')]
  elif not isinstance(car_predict_y_vec, str):
    car_predict_y_vec_list = car_predict_y_vec

  if isinstance(car_predict_heading_vec, str) and len(car_predict_heading_vec) > 0:
    car_predict_heading_vec_list = [float(heading) for heading in car_predict_heading_vec.split(',')]
  elif not isinstance(car_predict_heading_vec, str):
    car_predict_heading_vec_list = car_predict_heading_vec

  data_sim_car_predict_traj_path.data.update({
    'x': car_predict_x_vec_list,
    'y': car_predict_y_vec_list,
  })

  car_box_x_vec = []
  car_box_y_vec = []
  car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type, real_col_det_car_inflation)
  # path ego car
  for k in range(len(car_predict_x_vec_list)):
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], car_predict_x_vec_list[k], car_predict_y_vec_list[k], car_predict_heading_vec_list[k])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)


  data_sim_car_predict_traj_path_car_box.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })

  if plot_speed_graph == True:
    dp_speed_constraints = apa_simulation_py.GetDpSpeedConstraints()
    qp_speed_constraints = apa_simulation_py.GetQPSpeedConstraints()
    ref_cruise_speed = apa_simulation_py.GetRefCruiseSpeed()
    dp_speed_data = apa_simulation_py.GetDPSpeedOptimizationData()
    qp_speed_data = apa_simulation_py.GetQPSpeedOptimizationData()

    update_lon_plan_online_data(
        dp_speed_constraints, qp_speed_constraints, ref_cruise_speed,
        dp_speed_data, qp_speed_data, lon_plan_data)

    # jlt data
    jlt_speed_data = apa_simulation_py.GetJLTSpeedData()
    update_jlt_online_data(jlt_speed_data, lon_plan_data)

    traj_speed_profile = []
    if index_map['plan_msg_idx'] < len(bag_loader.plan_msg['data']):
      print('traj size = ', bag_loader.plan_msg['data']
            [index_map['plan_msg_idx']].trajectory.trajectory_points_size)

      for i in range(bag_loader.plan_msg['data'][index_map['plan_msg_idx']].trajectory.trajectory_points_size):
        point = bag_loader.plan_msg['data'][index_map['plan_msg_idx']].trajectory.trajectory_points[i]

        speed_point = []
        speed_point.append(point.distance)
        speed_point.append(point.t)
        speed_point.append(point.v)
        speed_point.append(point.a)
        speed_point.append(point.jerk)
        traj_speed_profile.append(speed_point)

    update_record_speed_data(traj_speed_profile, lon_plan_data)

    if res == True:
      non_optimizer_path_x = []
      non_optimizer_path_y = []
      non_optimizer_path_theta = []
      size = len(tuned_planning_output.trajectory.trajectory_points)
      print('online size = ', size)

      for i in range(len(tuned_planning_output.trajectory.trajectory_points)):
        id = size - i - 1
        if id < 0:
          break

        point = tuned_planning_output.trajectory.trajectory_points[id]
        if point.v > 0.0 :
          break

        non_optimizer_path_x.append(point.x)
        non_optimizer_path_y.append(point.y)
        non_optimizer_path_theta.append(point.heading_yaw)

      non_optimizer_traj.data.update({
          'x': non_optimizer_path_x,
          'y': non_optimizer_path_y,
          'heading': non_optimizer_path_theta,
          })

    # plot stop signs
    stop_sign_lines = apa_simulation_py.GetStopSigns()
    stop_sign_lines_x = []
    stop_sign_lines_y = []
    for k in range(len(stop_sign_lines)):
      stop_sign = stop_sign_lines[k]
      stop_sign_lines_x.append([stop_sign[0], stop_sign[2]])
      stop_sign_lines_y.append([stop_sign[1], stop_sign[3]])

      print(stop_sign[0])
      print(stop_sign[1])
      print(stop_sign[2])
      print(stop_sign[3])

    stop_signs.data.update({
        'x': stop_sign_lines_x,
        'y': stop_sign_lines_y,
    })

    # plot od
    trajs_x = []
    trajs_y = []
    data_od_traj.data.update({
        'x': trajs_x,
        'y': trajs_y,
    })

    trajs = apa_simulation_py.GetODTraj()
    for k in range(len(trajs)):
      traj = trajs[k]
      for i in range(len(traj)):
        trajs_x.append(traj[i][0])
        trajs_y.append(traj[i][1])

    data_od_traj.data.update({
        'x': trajs_x,
        'y': trajs_y,
    })

  push_notebook()

if plot_speed_graph == False:
  bkp.show(row(fig1), notebook_handle=True)
else:
  bkp.show(row(fig1, pans), notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)


