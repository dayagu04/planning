import sys, os, copy
sys.path.append("..")
from io import BytesIO
from lib.load_local_view_parking import *
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')
sys.path.append('../../../build/devel/lib/python3/dis-packagers')

sys.path.append('python_proto')
from jupyter_pybind.python_proto import planning_debug_info_pb2
from jupyter_pybind import apa_simulation_py
from struct_msgs.msg import PlanningOutput, UssPerceptInfo, GroundLinePerceptionInfo, FusionObjectsInfo, FusionOccupancyObjectsInfo

# bag path and frame dt
bag_path = '/data_cold/abu_zone/autoparse/chery_e0y_18047/trigger/20240724/20240724-10-49-02/park_in_data_collection_CHERY_E0Y_18047_ALL_FILTER_2024-07-24-10-49-03_no_camera.bag'
frame_dt = 0.1 # sec
parking_flag = True
global last_plan_pose_
last_plan_pose_ = []

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

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

data_sim_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_sim_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_sim_target_line = ColumnDataSource(data = {'x':[], 'y':[]})
data_sim_target_pos = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_simu_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_sim_obs = ColumnDataSource(data = {'obs_x':[], 'obs_y':[]})

fig1.circle('plan_path_y', 'plan_path_x', source = data_planning_tune, size=4, color='yellow', legend_label = 'sim_tuned_plan')
fig1.line('plan_path_y', 'plan_path_x', source = data_planning_tune, line_width = 6, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'sim_tuned_plan')
fig1.circle('y','x', source = data_sim_pos, size=8, color='red')
fig1.patch('car_yn', 'car_xn', source = data_sim_car, fill_color = "red", fill_alpha=0.25, line_color = "black", line_width = 1, legend_label = 'sim_car', visible = False)
fig1.patches('y_vec', 'x_vec', source = data_simu_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sim_sampled_carbox', visible = False)
fig1.patch('car_yn', 'car_xn', source = data_sim_target_pos, fill_color = "blue", line_color = "black", line_width = 1, line_alpha = 0.5, legend_label = 'data_sim_target_pos', visible = False)
fig1.line('y', 'x', source = data_sim_target_line, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.8, legend_label = 'data_sim_target_pos', visible = False)
fig1.circle('obs_y', 'obs_x', source = data_sim_obs, size=6.0, color='red', legend_label='sim obs', visible = False)

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    self.vehicle_type_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "vehicle_type",min=0, max=2, value=0, step=1)
    self.sim_to_target_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "sim_to_target",min=0, max=1, value=0, step=1)
    self.use_slot_in_bag_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "use_slot_in_bag",min=0, max=1, value=1, step=1)
    self.use_obs_in_bag_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "use_obs_in_bag",min=0, max=1, value=1, step=1)
    self.select_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='18%'), description= "select_id",min=0, max=20, value=0, step=1)
    self.force_plan_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "force_plan",min=0, max=1, value=0, step=1)
    self.is_path_optimization_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "path_optimization",min=0, max=1, value=0, step=1)
    self.is_cilqr_enable_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "cilqr_enable",min=0, max=1, value=1, step=1)
    self.is_reset_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_reset",min=0, max=1, value=0, step=1)
    self.is_complete_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_complete_path",min=0, max=1, value=0, step=1)
    self.sample_ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='25%'), description= "sample_ds",min=0.02, max=2.0, value=0.1, step=0.02)
    self.lon_pos_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "lon_pos_dif",min=-20.0, max=20.0, value=0.0, step=0.01)
    self.lat_pos_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "lat_pos_dif",min=-20.0, max=20.0, value=0.0, step=0.01)
    self.heading_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "heading_dif",min=-90.0, max=90.0, value=0.0, step=0.1)

    ipywidgets.interact(slider_callback,
                        bag_time = self.time_slider,
                        vehicle_type = self.vehicle_type_slider,
                        sim_to_target = self.sim_to_target_slider,
                        use_slot_in_bag = self.use_slot_in_bag_slider,
                        use_obs_in_bag = self.use_obs_in_bag_slider,
                        select_id = self.select_id_slider,
                        force_plan = self.force_plan_slider,
                        is_path_optimization = self.is_path_optimization_slider,
                        is_cilqr_enable = self.is_cilqr_enable_slider,
                        is_reset = self.is_reset_slider,
                        is_complete_path = self.is_complete_path_slider,
                        sample_ds = self.sample_ds_slider,
                        lon_pos_dif = self.lon_pos_dif_slider,
                        lat_pos_dif = self.lat_pos_dif_slider,
                        heading_dif = self.heading_dif_slider)

### sliders callback
def slider_callback(bag_time, vehicle_type, sim_to_target, use_slot_in_bag, use_obs_in_bag, select_id, force_plan, is_path_optimization, is_cilqr_enable, is_reset, is_complete_path, sample_ds, lon_pos_dif, lat_pos_dif, heading_dif):
  kwargs = locals()

  if vehicle_type == 0:
    vehicle_type = 'JAC_S811'
  elif vehicle_type == 1:
    vehicle_type = 'CHERY_T26'
  elif vehicle_type == 2:
    vehicle_type = 'CHERY_E0X'

  update_local_view_data_parking(fig1, bag_loader, bag_time, vehicle_type, local_view_data)
  car_xb, car_yb = load_car_params_patch_parking(vehicle_type)
  index_map = bag_loader.get_msg_index(bag_time)

  plan_debug_msg = bag_loader.plan_debug_msg['json'][index_map['plan_debug_msg_idx']]
  fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]
  wave_msg = bag_loader.wave_msg['data'][index_map['wave_msg_idx']]
  vs_msg = bag_loader.vs_msg['data'][index_map['vs_msg_idx']]
  soc_state_msg = bag_loader.soc_state_msg['data'][index_map['soc_state_msg_idx']]

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
  obs_x_vec = []
  obs_y_vec = []
  if soc_state_msg.current_state >= 26:
    target_managed_slot_x_vec = plan_debug_msg['slot_corner_X']
    target_managed_slot_y_vec = plan_debug_msg['slot_corner_Y']
    target_managed_limiter_x_vec = plan_debug_msg['limiter_corner_X']
    target_managed_limiter_y_vec = plan_debug_msg['limiter_corner_Y']
    obs_x_vec = plan_debug_msg['obstaclesX']
    obs_y_vec = plan_debug_msg['obstaclesY']

  current_ego_x = loc_msg.pose.local_position.x
  current_ego_y = loc_msg.pose.local_position.y
  sim_ego_heading = loc_msg.pose.euler_angles.yaw + heading_dif / 57.2958

  sim_ego_x = current_ego_x + lon_pos_dif * math.cos(sim_ego_heading) - lat_pos_dif * math.sin(sim_ego_heading)
  sim_ego_y = current_ego_y + lon_pos_dif * math.sin(sim_ego_heading) + lat_pos_dif * math.cos(sim_ego_heading)

  if len(last_plan_pose_) > 0 and sim_to_target == 1:
    sim_ego_x = last_plan_pose_[0]
    sim_ego_y = last_plan_pose_[1]
    sim_ego_heading = last_plan_pose_[2]

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

  res = apa_simulation_py.InterfaceUpdateParam(soc_state_msg_bytes,
                                    fus_parking_msg_bytes,
                                    loc_msg_bytes,
                                    vs_msg_bytes,
                                    wave_msg_bytes,
                                    uss_perception_msg_bytes,
                                    ground_line_perception_msg_bytes,
                                    fus_obj_msg_bytes,
                                    fus_occ_obj_msg_bytes,
                                    select_id, force_plan, is_path_optimization,
                                    is_cilqr_enable, is_reset, is_complete_path,
                                    sim_to_target, use_slot_in_bag, use_obs_in_bag, sample_ds,
                                    target_managed_slot_x_vec, target_managed_slot_y_vec,
                                    target_managed_limiter_x_vec, target_managed_limiter_y_vec,
                                    obs_x_vec, obs_y_vec)

  data_planning_tune.data = {'plan_path_x': [],
                             'plan_path_y': [],
                             'plan_path_heading': []}

  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []
  line_xn = []
  line_yn = []
  car_xn = []
  car_yn = []
  car_box_x_vec = []
  car_box_y_vec = []
  obstacle_x = []
  obstacle_y = []
  if res == True:
    tuned_planning_output = PlanningOutput()
    tuned_planning_output.deserialize(apa_simulation_py.GetPlanningOutput())
    print("plan release slot id = ", tuned_planning_output.successful_slot_info_list)

    tuned_planning_debug_info = planning_debug_info_pb2.PlanningDebugInfo()
    tuned_planning_debug_info.deserialize(apa_simulation_py.GetPlanningDebugInfo())
    date_planning_debug = json.loads(tuned_planning_debug_info.data_json)
    obstacle_x = date_planning_debug["obstaclesX"]
    obstacle_y = date_planning_debug["obstaclesY"]

    for i in range(len(tuned_planning_output.trajectory.trajectory_points)):
      plan_path_x.append(tuned_planning_output.trajectory.trajectory_points[i].x)
      plan_path_y.append(tuned_planning_output.trajectory.trajectory_points[i].y)
      plan_path_heading.append(tuned_planning_output.trajectory.trajectory_points[i].heading_yaw)

    if (len(plan_path_x) > 1):
      half_car_width = 0.9
      last_x = plan_path_x[-1]
      last_y = plan_path_y[-1]
      last_heading = plan_path_heading[-1]
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

    # path ego car
    for k in range(len(tuned_planning_output.trajectory.trajectory_points)):
      car_xn = []
      car_yn = []
      for i in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], plan_path_x[k], plan_path_y[k], plan_path_heading[k])
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)
      car_box_x_vec.append(car_xn)
      car_box_y_vec.append(car_yn)

    print("tuned_gear_command = ", tuned_planning_output.gear_command)

  data_planning_tune.data.update({
    'plan_path_x': plan_path_x,
    'plan_path_y': plan_path_y,
    'plan_path_heading': plan_path_heading,
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

  if isinstance(obstacle_x, str):
    obstacle_x_list = [float(x) for x in obstacle_x.split(',')]
  else:
    obstacle_x_list = obstacle_x

  if isinstance(obstacle_y, str):
    obstacle_y_list = [float(y) for y in obstacle_y.split(',')]
  else:
    obstacle_y_list = obstacle_y

  data_sim_obs.data.update({
    'obs_x': obstacle_x_list,
    'obs_y': obstacle_y_list,
  })

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


