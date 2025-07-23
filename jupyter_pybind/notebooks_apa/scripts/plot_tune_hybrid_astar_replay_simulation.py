import sys, os, copy
sys.path.append("..")
from io import BytesIO
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
from lib.load_lon_plan import *
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')
sys.path.append('../../../build/devel/lib/python3/dis-packagers')

sys.path.append('python_proto')
from jupyter_pybind.python_proto import planning_debug_info_pb2
from jupyter_pybind import replay_simulation_hybrid_astar
from struct_msgs.msg import PlanningOutput, UssPerceptInfo, GroundLinePerceptionInfo, FusionObjectsInfo, FusionOccupancyObjectsInfo,ParkingFusionInfo,ControlOutput


# bag path and frame dt
# e0y1:  10034
# e0y2:  04228
# e0y8:  14520
# e0y9:  18049
# e0y10: 20267
bag_path ='/data_cold/abu_zone/autoparse/chery_m32t_40735/trigger/20250722/20250722-16-46-24/park_in_data_collection_CHERY_M32T_40735_ALL_FILTER_2025-07-22-16-46-25_no_camera.bag'
frame_dt = 0.1 # sec
parking_flag = True
global last_plan_pose_
last_plan_pose_ = []
astar_path_start_time = -1

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

start_time = time.time()
bag_loader = LoadCyberbag(bag_path, parking_flag)
end_time = time.time()
print('load LoadCyberbag, ms====== ', (end_time - start_time) * 1000)

start_time = time.time()
max_time = bag_loader.load_all_data()
end_time = time.time()
print('load_all_data, ms====== ', (end_time - start_time) * 1000)

start_time = time.time()
fig1, local_view_data = load_local_view_figure_parking()
end_time = time.time()
print('load_local_view_figure_parking, ms===== ', (end_time - start_time) * 1000)

# plot speed
plot_speed = True
if plot_speed:
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
replay_simulation_hybrid_astar.Init()

data_sim_pos = ColumnDataSource(data = {'x':[], 'y':[]})
# record包中的定位信息
data_sim_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_path_end = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_astar_target_pos = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_astar_collision_pos = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
# 路径关键点处的圆
data_anchor_point_circle = ColumnDataSource(data = {'car_circle_yn':[], 'car_circle_xn':[], 'car_circle_rn':[]})
# 路径所有的圆
data_path_all_circle = ColumnDataSource(data = {'car_circle_yn':[], 'car_circle_xn':[], 'car_circle_rn':[]})
data_rs_path = ColumnDataSource(data={'plan_path_x': [],
                                      'plan_path_y': [],
                                      'plan_path_heading': [], })
data_record_rs_path = ColumnDataSource(data={'plan_path_x': [],
                                      'plan_path_y': [],
                                      'plan_path_heading': [], })
data_astar_path = ColumnDataSource(data={'plan_path_x': [],
                                      'plan_path_y': [],
                                      'plan_path_heading': [], })
data_record_astar_path = ColumnDataSource(data={'plan_path_x': [],
                                      'plan_path_y': [],
                                      'plan_path_heading': [], })
data_virtual_wall = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_record_node_list = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_real_time_node_list = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_astar_path_envelop = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_current_gear_path_envelop = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
all_rs_heuristic_path = ColumnDataSource(data = {'x':[], 'y':[]})
data_localization_text = ColumnDataSource(data = {'info':[]})
data_obstacle_points = ColumnDataSource(data = {'x':[], 'y':[]})
data_plot_ref_line = ColumnDataSource(data={'plan_path_x': [],
                                      'plan_path_y': [],
                                      })
data_search_sequence_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_coordinate_system = ColumnDataSource(data = {'x':[], 'y':[]})
data_all_search_node = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_all_delete_node = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_all_search_collision_node = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_gear_switch_node = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_polynomial_path = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})
optimizer_traj = ColumnDataSource(data={'plan_path_x': [],
                                      'plan_path_y': [],
                                      'plan_path_heading': [], })
non_optimizer_traj = ColumnDataSource(data={'plan_path_x': [],
                                      'plan_path_y': [],
                                      'plan_path_heading': [], })

fig1.line('plan_path_y', 'plan_path_x', source = data_rs_path, line_width = 3, line_color = 'orange', line_dash = 'solid', line_alpha = 0.5, legend_label = 'rs_path',visible = False)
fig1.line('plan_path_y', 'plan_path_x', source = data_record_rs_path, line_width = 3, line_color = 'orange', line_dash = 'solid', line_alpha = 0.5, legend_label = 'record_rs_path',visible = False)
fig1.line('plan_path_y', 'plan_path_x', source = data_plot_ref_line, line_width = 2, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'ref_line')
fig1.line('plan_path_y', 'plan_path_x', source = data_astar_path, line_width = 3, line_color = '#7b0ec4', line_dash = 'solid', line_alpha = 0.5, legend_label = 'astar_path')
fig1.line('plan_path_y', 'plan_path_x', source = optimizer_traj, line_width = 8, line_color = 'green', line_dash = 'solid', line_alpha = 0.8, legend_label = 'optimizer_traj')
fig1.line('plan_path_y', 'plan_path_x', source = non_optimizer_traj, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.6, legend_label = 'non_optimizer_traj')
fig1.line('plan_path_y', 'plan_path_x', source = data_record_astar_path, line_width = 3, line_color = 'gray', line_dash = 'solid', line_alpha = 0.5, legend_label = 'record_astar_path',visible = False)
fig1.circle('y','x', source = data_sim_pos, size=8, color='red')
fig1.circle('y','x', source = data_coordinate_system, size=8, color='purple')
fig1.patch('car_yn', 'car_xn', source = data_sim_car, fill_color = "red", fill_alpha=0.25, line_color = "black", line_width = 1, legend_label = 'sim_car', visible = False)
fig1.patch('car_yn', 'car_xn', source = data_path_end, fill_color = "blue",fill_alpha = 0.2, line_color = "black", line_width = 1, line_alpha = 0.5, legend_label = 'path_end', visible = False)
fig1.patch('car_yn', 'car_xn', source = data_astar_target_pos, fill_color = "blue",fill_alpha = 0.2, line_color = "black", line_width = 1, line_alpha = 0.5, legend_label = 'astar_target', visible = False)
fig1.multi_line('y_vec', 'x_vec', source=data_record_node_list, line_width=1.0, line_color='green', line_dash='solid', legend_label='record_node_list')
fig1.multi_line('y_vec', 'x_vec', source=data_real_time_node_list, line_width=1.0, line_color='red', line_dash='solid', legend_label='real_time_node_list')
fig1.patches('y_vec', 'x_vec', source = data_astar_path_envelop, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'all_path_envelop', visible = False)
fig1.patches('y_vec', 'x_vec', source = data_current_gear_path_envelop, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'online_traj_envelop', visible = False)
fig1.multi_line('y', 'x',source = all_rs_heuristic_path, line_width = 1.5, line_color = 'purple', line_dash = 'solid',legend_label = 'rs_h_path',visible = False)
fig1.circle('y', 'x', source = data_obstacle_points, size=4, color='red', legend_label = 'virtual_wall')
fig1.circle(x ='car_circle_yn', y ='car_circle_xn', radius = 'car_circle_rn', source = data_anchor_point_circle, line_alpha = 0.5, line_width = 1, line_color = "blue", fill_alpha=0, legend_label = 'anchor_point_circle', visible = False)
fig1.circle(x ='car_circle_yn', y ='car_circle_xn', radius = 'car_circle_rn', source = data_path_all_circle, line_alpha = 0.5, line_width = 1, line_color = "blue", fill_alpha=0, legend_label = 'path_circle', visible = False)
fig1.line('y_vec', 'x_vec', source = data_search_sequence_path, line_width = 2, line_color = 'blue', line_dash = 'solid', line_alpha = 0.8, legend_label = 'search_sequence',visible = False)
fig1.circle('y_vec', 'x_vec', source = data_all_search_node, size=4, color='black',  legend_label = 'all_search_node')
fig1.circle('y_vec', 'x_vec', source = data_all_delete_node, size=4, color='red',  legend_label = 'all_delete_node')
fig1.circle('y_vec', 'x_vec', source = data_all_search_collision_node, size=4, color='gray',  legend_label = 'all_collision_node')
fig1.circle('y_vec', 'x_vec', source = data_gear_switch_node, size=4, color='purple',  legend_label = 'gear_switch_node')
fig1.line('plan_path_x', 'plan_path_y', source = data_polynomial_path, line_width = 6, line_color = 'purple', line_dash = 'solid', line_alpha = 0.5, legend_label = 'polynomial')


### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    self.select_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='18%'), description= "select_id",min=0, max=20, value=0, step=1)
    self.sim_to_target_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "sim_to_target",min=0, max=1, value=0, step=1)
    self.search_sequence_num = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description="search_sequence_num", min=0, max=500000, value=1, step=1)
    self.force_plan_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "force_plan",min=0, max=1, value=0, step=1)
    self.refresh_thread = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "refresh_thread",min=0, max=1, value=0, step=1)
    self.is_path_optimization_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "path_optimization",min=0, max=1, value=1, step=1)
    self.is_cilqr_enable_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "cilqr_enable",min=0, max=1, value=1, step=1)
    self.is_reset_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_reset",min=0, max=1, value=0, step=1)
    self.is_complete_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_complete_path",min=0, max=1, value=0, step=1)
    self.sample_ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='25%'), description= "sample_ds",min=0.02, max=2.0, value=0.12, step=0.02)
    self.lon_pos_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "lon_pos_dif",min=-20.0, max=20.0, value=0.0, step=0.01)
    self.lat_pos_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "lat_pos_dif",min=-20.0, max=20.0, value=0.0, step=0.01)
    self.heading_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description= "heading_dif",min=-90.0, max=90.0, value=0.0, step=0.1)
    self.plot_child_node = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description="plot_child_node", min=0, max=1, value=0, step=1)
    self.use_state_machine = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description="use_state_machine", min=0, max=1, value=0, step=1)
    self.state_machine = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description="state_machine", min=0, max=100, value=0, step=1)
    self.path_plan_method = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description="path_plan_method", min=0, max=10, value=1, step=1)
    self.swap_start_goal = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description="swap_start_goal", min=0, max=2, value=0, step=1)

    ipywidgets.interact(slider_callback,
                        bag_time = self.time_slider,
                        select_id = self.select_id_slider,
                        sim_to_target = self.sim_to_target_slider,
                        search_sequence_num = self.search_sequence_num,
                        force_plan = self.force_plan_slider,
                        refresh_thread = self.refresh_thread,
                        is_path_optimization = self.is_path_optimization_slider,
                        is_cilqr_enable = self.is_cilqr_enable_slider,
                        is_reset = self.is_reset_slider,
                        is_complete_path = self.is_complete_path_slider,
                        sample_ds = self.sample_ds_slider,
                        lon_pos_dif = self.lon_pos_dif_slider,
                        lat_pos_dif = self.lat_pos_dif_slider,
                        heading_dif = self.heading_dif_slider,
                        plot_child_node=self.plot_child_node,
                        use_state_machine=self.use_state_machine,
                        state_machine=self.state_machine,
                        path_plan_method=self.path_plan_method,
                        swap_start_goal=self.swap_start_goal)

### sliders callback
def slider_callback(bag_time, select_id,sim_to_target, search_sequence_num, force_plan, refresh_thread,is_path_optimization,
                    is_cilqr_enable, is_reset, is_complete_path, sample_ds,
                    lon_pos_dif, lat_pos_dif, heading_dif,plot_child_node,use_state_machine,state_machine,
                    path_plan_method,swap_start_goal):

  time0 = time.time()

  kwargs = locals()
  # vehicle_type = 'JAC_S811'
  vehicle_type = 'CHERY_E0X'
  # vehicle_type = 'CHERY_T26'
  update_local_view_data_parking(fig1, bag_loader, bag_time, vehicle_type,0, local_view_data,False)
  car_circle_x, car_circle_y, car_circle_r = load_car_circle_coord_by_veh(vehicle_type)
  car_polygon_x, car_polygon_y, wheel_base = load_car_params_patch_parking(
      vehicle_type, 0.0)
  index_map = bag_loader.get_msg_index(bag_time)

  start_time = time.time()
  print('time0, ms ', (start_time - time0) * 1000)

  data_valid = {'plan_msg_idx':False,
                'fus_parking_msg_idx':False,
                'wave_msg_idx':False,
                'vs_msg_idx':False,
                'soc_state_msg_idx':False,
                'plan_debug_msg_idx':False,
                'fus_parking_msg_idx':False,
                'fus_objects_msg_idx':False,
                'fus_ground_line_msg_idx':False,
                'loc_msg_idx':False,
                'uss_percept_msg_idx':False,
                 }

  if index_map['fus_objects_msg_idx'] < len(bag_loader.fus_objects_msg['data']):
    fus_obj_msg = bag_loader.fus_objects_msg['data'][index_map['fus_objects_msg_idx']]

    print('fusion obj size', fus_obj_msg.fusion_object_size)
    data_valid['fus_objects_msg_idx'] = True
  else:
    fus_obj_msg = FusionObjectsInfo()

  if index_map['fus_ground_line_msg_idx'] < len(bag_loader.fus_ground_line_msg['data']):
    ground_line_msg = bag_loader.fus_ground_line_msg['data'][index_map['fus_ground_line_msg_idx']]
    data_valid['fus_ground_line_msg_idx'] = True
  else:
    ground_line_msg = GroundLinePerceptionInfo()

  if index_map['fus_parking_msg_idx'] < len(bag_loader.fus_parking_msg['data']):
    fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]
    data_valid['fus_parking_msg_idx'] = True
  else:
    print('fus_parking_msg invalid')
    fus_parking_msg = ParkingFusionInfo()


  if index_map['wave_msg_idx'] < len(bag_loader.wave_msg['data']):
    wave_msg = bag_loader.wave_msg['data'][index_map['wave_msg_idx']]
    data_valid['wave_msg_idx'] = True
  else:
    print('wave_msg_idx invalid')


  if index_map['vs_msg_idx'] < len(bag_loader.vs_msg['data']):
    vs_msg = bag_loader.vs_msg['data'][index_map['vs_msg_idx']]
    data_valid['vs_msg_idx'] = True
  else:
      print('vs_msg_idx invalid')

  if index_map['soc_state_msg_idx'] < len(bag_loader.soc_state_msg['data']):
    soc_state_msg = bag_loader.soc_state_msg['data'][index_map['soc_state_msg_idx']]
    data_valid['soc_state_msg_idx'] = True
  else:
      print('soc_state_msg_idx invalid')

  if index_map['uss_percept_msg_idx'] < len(bag_loader.uss_percept_msg['data']):
    uss_perception_msg = bag_loader.uss_percept_msg['data'][index_map['uss_percept_msg_idx']]
    data_valid['uss_percept_msg_idx'] = True
  else:
    uss_perception_msg = UssPerceptInfo()

  if index_map['loc_msg_idx'] < len(bag_loader.loc_msg['data']):
    loc_msg = copy.deepcopy(bag_loader.loc_msg['data'][index_map['loc_msg_idx']])
    data_valid['loc_msg_idx'] = True


  if index_map['plan_debug_msg_idx'] < len(bag_loader.plan_debug_msg['data']):
    slot_management_info = bag_loader.plan_debug_msg['data'][index_map['plan_debug_msg_idx']].slot_management_info
    data_valid['plan_debug_msg_idx'] = True
  else:
      print('plan_debug_msg_idx invalid')

  if index_map['fus_parking_msg_idx'] < len(bag_loader.fus_parking_msg['data']):
    select_slot_id = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']].select_slot_id
    data_valid['fus_parking_msg_idx'] = True
  else:
      print('fus_parking_msg_idx invalid')

  if bag_loader.fus_occupancy_objects_msg['enable'] == True:
    fus_occ_obj_msg = bag_loader.fus_occupancy_objects_msg['data'][index_map['fus_occupancy_objects_msg_idx']]
  else:
    fus_occ_obj_msg = FusionOccupancyObjectsInfo()

  if bag_loader.ctrl_msg['enable'] == True:
    control_msg = bag_loader.ctrl_msg['data'][index_map['ctrl_msg_idx']]
  else:
    control_msg = ControlOutput()

  record_plan_path_x =[]
  record_plan_path_y =[]
  record_plan_path_heading =[]
  rs_plan_path_x =[]
  rs_plan_path_y =[]
  rs_plan_path_heading =[]

  if index_map['plan_debug_msg_idx'] < len(bag_loader.plan_debug_msg['data']):
    plan_debug_msg = bag_loader.plan_debug_msg['data'][index_map['plan_debug_msg_idx']]

    for i in range(len(plan_debug_msg.refline_info)):
      path_point = plan_debug_msg.refline_info[i]
      record_plan_path_x.append(path_point.x)
      record_plan_path_y.append(path_point.y)
      record_plan_path_heading.append(path_point.heading_angle)

      if(path_point.l < 0.0):
        rs_plan_path_x.append(path_point.x)
        rs_plan_path_y.append(path_point.y)
        rs_plan_path_heading.append(path_point.heading_angle)

    print('speed type',
          plan_debug_msg.apa_speed_debug.speed_type)

    # update value
    data_record_astar_path.data.update({
        'plan_path_x': record_plan_path_x,
        'plan_path_y': record_plan_path_y,
        'plan_path_heading': record_plan_path_heading,
    })

    # update value
    data_record_rs_path.data.update({
        'plan_path_x': rs_plan_path_x,
        'plan_path_y': rs_plan_path_y,
        'plan_path_heading': rs_plan_path_heading,
    })



  # if plot_child_node and bag_loader.plan_debug_msg['data'][index_map['plan_debug_msg_idx']].node_list:
  #   line_list_x_vec, line_list_y_vec = [], []
  #   if index_map['plan_debug_msg_idx'] < len(bag_loader.plan_debug_msg['data']):
  #     node_list = bag_loader.plan_debug_msg['data'][index_map['plan_debug_msg_idx']].node_list

  #     print('size')
  #     print(len(node_list.nodes))

  #     for i in range(len(node_list.nodes)):
  #       plan_path_x =[]
  #       plan_path_y =[]
  #       plan_path_heading =[]
  #       for j in range(len(node_list.nodes[i].path_point)):
  #         path_point = node_list.nodes[i].path_point[j]
  #         plan_path_x.append(path_point.x)
  #         plan_path_y.append(path_point.y)
  #         plan_path_heading.append(path_point.heading_angle)

  #       line_list_x_vec.append(plan_path_x)
  #       line_list_y_vec.append(plan_path_y)

  #     data_record_node_list.data.update({
  #           'x_vec': [],
  #           'y_vec': [],
  #       })
  #     data_record_node_list.data.update({
  #       'x_vec': line_list_x_vec,
  #       'y_vec': line_list_y_vec,})
  # else:
  #     data_record_node_list.data.update({
  #           'x_vec': [],
  #           'y_vec': [],
  #       })

  traj_speed_profile = []
  if index_map['plan_msg_idx'] < len(bag_loader.plan_msg['data']):
    for i in range(bag_loader.plan_msg['data'][index_map['plan_msg_idx']].trajectory.trajectory_points_size):
      point = bag_loader.plan_msg['data'][index_map['plan_msg_idx']].trajectory.trajectory_points[i]

      speed_point = []
      speed_point.append(point.distance)
      speed_point.append(point.t)
      speed_point.append(point.v)
      speed_point.append(point.a)
      speed_point.append(point.jerk)
      traj_speed_profile.append(speed_point)

  target_managed_slot_x_vec = []
  target_managed_slot_y_vec = []
  for i in range(len(slot_management_info.slot_info_vec)):
    maganed_slot_vec = slot_management_info.slot_info_vec[i]
    corner_point = maganed_slot_vec.corner_points.corner_point
    if maganed_slot_vec.id == select_slot_id:
      target_managed_slot_x_vec = [corner_point[0].x,corner_point[1].x,corner_point[2].x,corner_point[3].x]
      target_managed_slot_y_vec = [corner_point[0].y,corner_point[1].y,corner_point[2].y,corner_point[3].y]

  # print('target_managed_slot_x_vec',
  #       target_managed_slot_x_vec, target_managed_slot_y_vec)

  target_managed_slot_x_vec = []
  target_managed_slot_y_vec = []
  target_managed_limiter_x_vec = []
  target_managed_limiter_y_vec = []

  print('soc_state_msg.current_state')
  print(soc_state_msg.current_state)
  if soc_state_msg.current_state >= 13:
    plan_debug_msg = bag_loader.plan_debug_msg['json'][index_map['plan_debug_msg_idx']]


    target_managed_slot_x_vec = plan_debug_msg['slot_corner_X']
    target_managed_slot_y_vec = plan_debug_msg['slot_corner_Y']
    target_managed_limiter_x_vec = plan_debug_msg['limiter_corner_X']
    target_managed_limiter_y_vec = plan_debug_msg['limiter_corner_Y']

  print('target_managed_slot_x_vec', target_managed_slot_x_vec,target_managed_slot_y_vec )
  print('target_managed_limiter_x_vec', target_managed_limiter_x_vec,target_managed_limiter_y_vec )

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
  for i in range(len(car_polygon_x)):
    tmp_x, tmp_y = local2global(car_polygon_x[i], car_polygon_y[i], sim_ego_x, sim_ego_y, sim_ego_heading)
    car_xn.append(tmp_x)
    car_yn.append(tmp_y)

  data_sim_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  res = False

  all_data_valid = True
  for i in data_valid.values():
    if i is True:
      all_data_valid = False
      break


  end_time = time.time()

  print('time, ms ', (end_time - start_time) * 1000)

  soc_state_msg_buff = BytesIO()
  if(use_state_machine):
    soc_state_msg.current_state = state_machine

  soc_state_msg.serialize(soc_state_msg_buff)
  soc_state_msg_bytes = soc_state_msg_buff.getvalue()
  current_state = soc_state_msg.current_state

  print('apa_work_mode',soc_state_msg.parking_req.apa_work_mode)
  print('apa_parking_direction',soc_state_msg.parking_req.apa_parking_direction)

  fus_parking_msg_buff = BytesIO()
  fus_parking_msg.serialize(fus_parking_msg_buff)
  fus_parking_msg_bytes = fus_parking_msg_buff.getvalue()

  loc_msg_buff = BytesIO()
  loc_msg.serialize(loc_msg_buff)
  loc_msg_bytes = loc_msg_buff.getvalue()

  vs_msg_buff = BytesIO()
  vs_msg.serialize(vs_msg_buff)
  vs_msg_bytes = vs_msg_buff.getvalue()

  if data_valid['wave_msg_idx'] == True:
    wave_msg_buff = BytesIO()
    wave_msg.serialize(wave_msg_buff)
    wave_msg_bytes = wave_msg_buff.getvalue()

  uss_perception_msg_buff = BytesIO()
  uss_perception_msg.serialize(uss_perception_msg_buff)
  uss_perception_msg_bytes = uss_perception_msg_buff.getvalue()


  ground_line_perception_msg_buff = BytesIO()
  ground_line_msg.serialize(ground_line_perception_msg_buff)
  ground_line_perception_msg_bytes = ground_line_perception_msg_buff.getvalue()

  fus_obj_msg_buff = BytesIO()
  fus_obj_msg.serialize(fus_obj_msg_buff)
  fus_obj_msg_bytes = fus_obj_msg_buff.getvalue()

  fus_occ_obj_msg_buff = BytesIO()
  fus_occ_obj_msg.serialize(fus_occ_obj_msg_buff)
  fus_occ_obj_msg_bytes = fus_occ_obj_msg_buff.getvalue()

  print('fusion occ obj size', fus_occ_obj_msg.fusion_object_size)

  control_msg_buff = BytesIO()
  control_msg.serialize(control_msg_buff)
  control_msg_bytes = control_msg_buff.getvalue()

  # if data_valid['soc_state_msg_idx'] and data_valid['fus_parking_msg_idx'] and data_valid['loc_msg_idx'] and data_valid['vs_msg_idx'] and data_valid['wave_msg_idx'] and data_valid['uss_percept_msg_idx'] and fus_parking_msg.select_slot_id > 0:
  # if data_valid['soc_state_msg_idx'] and data_valid['fus_parking_msg_idx'] and data_valid['loc_msg_idx'] and data_valid['vs_msg_idx'] and data_valid['wave_msg_idx'] and data_valid['uss_percept_msg_idx']:
  if data_valid['soc_state_msg_idx'] and data_valid['fus_parking_msg_idx'] and data_valid['loc_msg_idx'] and data_valid['vs_msg_idx']:

    print('plan once')
    res = replay_simulation_hybrid_astar.PlanOnce(
        soc_state_msg_bytes,
        fus_parking_msg_bytes,
        loc_msg_bytes,
        vs_msg_bytes,
        wave_msg_bytes,
        uss_perception_msg_bytes,
        ground_line_perception_msg_bytes,
        fus_obj_msg_bytes,
        fus_occ_obj_msg_bytes,
        control_msg_bytes,
        select_id, force_plan, is_path_optimization,
        is_cilqr_enable, is_reset, is_complete_path,
        sample_ds, target_managed_slot_x_vec,
        target_managed_slot_y_vec,
        target_managed_limiter_x_vec,
        target_managed_limiter_y_vec,
        path_plan_method,
        swap_start_goal)

    print('end')
  else:
    print('no plan call')


  if refresh_thread:
    replay_simulation_hybrid_astar.RefreshThreadResult()


  end_time2 = time.time()
  print('time2, ms ', (end_time2 - end_time) * 1000)

  current_gear_path_x = []
  current_gear_path_y = []
  current_gear_path_heading = []
  line_xn = []
  line_yn = []
  car_xn = []
  car_yn = []
  car_box_x_vec = []
  car_box_y_vec = []
  current_gear_end_x=0
  current_gear_end_y=0
  current_gear_end_theta=0
  non_optimizer_path_x = []
  non_optimizer_path_y = []
  non_optimizer_path_theta = []

  if res == True:
    tuned_planning_output = PlanningOutput()
    tuned_planning_output.deserialize(replay_simulation_hybrid_astar.GetPlanningOutput())

    for i in range(len(tuned_planning_output.trajectory.trajectory_points)):
      point = tuned_planning_output.trajectory.trajectory_points[i]
      current_gear_path_x.append(tuned_planning_output.trajectory.trajectory_points[i].x)
      current_gear_path_y.append(tuned_planning_output.trajectory.trajectory_points[i].y)
      current_gear_path_heading.append(tuned_planning_output.trajectory.trajectory_points[i].heading_yaw)

      current_gear_end_x = tuned_planning_output.trajectory.trajectory_points[i].x
      current_gear_end_y = tuned_planning_output.trajectory.trajectory_points[i].y
      current_gear_end_theta = tuned_planning_output.trajectory.trajectory_points[i].heading_yaw

      if (point.v <= 0.0 and point.distance > 0.0):
        non_optimizer_path_x.append(point.x)
        non_optimizer_path_y.append(point.y)
        non_optimizer_path_theta.append(point.heading_yaw)

    if (len(current_gear_path_x) > 1):
      half_car_width = 0.9
      last_x = current_gear_path_x[-1]
      last_y = current_gear_path_y[-1]
      last_heading = current_gear_path_heading[-1]
      last_plan_pose_.clear()
      last_plan_pose_.append(last_x)
      last_plan_pose_.append(last_y)
      last_plan_pose_.append(last_heading)
      for i in range(len(car_polygon_x)):
        tmp_x, tmp_y = local2global(car_polygon_x[i], car_polygon_y[i], last_x, last_y, last_heading)
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
      for i in range(len(car_polygon_x)):
          tmp_x, tmp_y = local2global(car_polygon_x[i], car_polygon_y[i], current_gear_path_x[k], current_gear_path_y[k], current_gear_path_heading[k])
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)
      car_box_x_vec.append(car_xn)
      car_box_y_vec.append(car_yn)

    print("tuned_gear_command = ", tuned_planning_output.gear_command.gear_command_value)

  # draw current gear path envelop
  optimizer_traj.data.update({
      'plan_path_x': current_gear_path_x,
      'plan_path_y': current_gear_path_y,
      'plan_path_heading': current_gear_path_heading,
  })

  cur_gear_path_box_x_vec = []
  cur_gear_path_box_y_vec = []
  for k in range(len(current_gear_path_x)):
    car_xn = []
    car_yn = []
    for i in range(len(car_polygon_x)):
        tmp_x, tmp_y = local2global(car_polygon_x[i], car_polygon_y[i], current_gear_path_x[k], current_gear_path_y[k], current_gear_path_heading[k])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
    cur_gear_path_box_x_vec.append(car_xn)
    cur_gear_path_box_y_vec.append(car_yn)

  data_current_gear_path_envelop.data.update({
    'x_vec': cur_gear_path_box_x_vec,
    'y_vec': cur_gear_path_box_y_vec,
  })

  # astar path collision pose
  data_astar_collision_pos.data.update({
      'car_xn': [],
      'car_yn': [],
    })

  path_collision_info = replay_simulation_hybrid_astar.GetAstarPathCollisionID()
  point_size = len(current_gear_path_x)
  if path_collision_info[1] >0 and path_collision_info[0] >= 0 and path_collision_info[0] < point_size:
    pose = []

    id = path_collision_info[0]
    pose.append(current_gear_path_x[id])
    pose.append( current_gear_path_y[id])
    pose.append(current_gear_path_heading[id])

    car_xn = []
    car_yn = []
    for i in range(len(car_polygon_x)):
        tmp_x, tmp_y = local2global(car_polygon_x[i], car_polygon_y[i], pose[0], pose[1], pose[2])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)

    data_astar_collision_pos.data.update({
      'car_xn': car_xn,
      'car_yn': car_yn,
    })

  end_time3 = time.time()
  print('time3, ms ', (end_time3 - end_time2) * 1000)

  # rs
  rs_path = replay_simulation_hybrid_astar.GetReedsShapePath()
  # print('rs path',len(rs_path))

  if (len(rs_path) > 0):
    data_rs_path.data.update({
        'plan_path_x': [],
        'plan_path_y': [],
        'plan_path_heading': [],
    })

    path_x = []
    path_y = []
    path_heading = []

    for i in range(len(rs_path)):
        path_x.append(rs_path[i][0])
        path_y.append(rs_path[i][1])
        path_heading.append(rs_path[i][2])

    data_rs_path.data.update({
        'plan_path_x': path_x,
        'plan_path_y': path_y,
        'plan_path_heading': path_heading,
    })

  end_time4 = time.time()
  print('time4, ms ', (end_time4 - end_time3) * 1000)

  # # all search node
  if plot_child_node:
    line_list_x_vec, line_list_y_vec = [], []
    node_list = replay_simulation_hybrid_astar.GetAstarAllNodes()
    for i in range(len(node_list)):
      path_x =[]
      path_y =[]
      for j in range(len(node_list[i])):
        path_point = node_list[i][j]
        path_x.append(path_point[0])
        path_y.append(path_point[1])

      line_list_x_vec.append(path_x)
      line_list_y_vec.append(path_y)

    data_real_time_node_list.data.update({
          'x_vec': [],
          'y_vec': [],
      })
    data_real_time_node_list.data.update({
      'x_vec': line_list_x_vec,
      'y_vec': line_list_y_vec,})


  else:
    data_real_time_node_list.data.update({
          'x_vec': [],
          'y_vec': [],
      })


  end_time5 = time.time()
  print('time5, ms ', (end_time5 - end_time4) * 1000)

  # astar target
  pose = replay_simulation_hybrid_astar.GetAstarEndPose()
  car_xn = []
  car_yn = []
  for i in range(len(car_polygon_x)):
      tmp_x, tmp_y = local2global(car_polygon_x[i], car_polygon_y[i], pose[0], pose[1], pose[2])
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
  car_box_x_vec.append(car_xn)
  car_box_y_vec.append(car_yn)

  data_astar_target_pos.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  # anchor point circle
  car_circle_xn = []
  car_circle_yn = []
  car_circle_rn = []

  car_circle_x = []
  car_circle_y = []
  car_circle_r = []

  current_gear = tuned_planning_output.gear_command.gear_command_value
  print('gear',current_gear)
  footprint_model = replay_simulation_hybrid_astar.GetFootPrintModel(current_gear)
  for i in range(len(footprint_model)):
    car_circle_x.append(footprint_model[i][0])
    car_circle_y.append(footprint_model[i][1])
    car_circle_r.append(footprint_model[i][2])

  for i in range(len(car_circle_x)):
    tmp_x, tmp_y = local2global(car_circle_x[i], car_circle_y[i], pose[0], pose[1], pose[2])
    car_circle_xn.append(tmp_x)
    car_circle_yn.append(tmp_y)
    car_circle_rn.append(car_circle_r[i])

  # ego pose circle list
  for i in range(len(car_circle_x)):

    x = loc_msg.position.position_boot.x
    y = loc_msg.position.position_boot.y
    heading = loc_msg.orientation.euler_boot.yaw
    tmp_x, tmp_y = local2global(
        car_circle_x[i], car_circle_y[i], x, y, heading)
    car_circle_xn.append(tmp_x)
    car_circle_yn.append(tmp_y)
    car_circle_rn.append(car_circle_r[i])

  # astar current gear path target
  for i in range(len(car_circle_x)):
    tmp_x, tmp_y = local2global(
        car_circle_x[i], car_circle_y[i], current_gear_end_x, current_gear_end_y, current_gear_end_theta)

    car_circle_xn.append(tmp_x)
    car_circle_yn.append(tmp_y)
    car_circle_rn.append(car_circle_r[i])

  data_anchor_point_circle.data.update({
    'car_circle_xn': car_circle_xn,
    'car_circle_yn': car_circle_yn,
    'car_circle_rn': car_circle_rn,
  })

  # plot current gear path circle
  car_circle_xn = []
  car_circle_yn = []
  car_circle_rn = []
  for k in range(len(current_gear_path_x)):
    for i in range(len(car_circle_x)):
      # 最大圆不需要plot
      if i== 0:
        continue

      tmp_x, tmp_y = local2global(
        car_circle_x[i], car_circle_y[i], current_gear_path_x[k], current_gear_path_y[k], current_gear_path_heading[k])

      car_circle_xn.append(tmp_x)
      car_circle_yn.append(tmp_y)
      car_circle_rn.append(car_circle_r[i])

  data_path_all_circle.data.update({
    'car_circle_xn': car_circle_xn,
    'car_circle_yn': car_circle_yn,
    'car_circle_rn': car_circle_rn,
  })

  # print('target')

  # a star path
  # reset
  data_astar_path.data.update({
        'plan_path_x': [],
        'plan_path_y': [],
        'plan_path_heading': [],
    })

  path_x = []
  path_y = []
  path_heading = []
  astar_path = replay_simulation_hybrid_astar.GetAstarPath()

  # print('astar_path',len(astar_path))
  if (len(astar_path) > 0):
    for i in range(len(astar_path)):
        path_x.append(astar_path[i][0])
        path_y.append(astar_path[i][1])
        path_heading.append(astar_path[i][2])

    # update value
    data_astar_path.data.update({
        'plan_path_x': path_x,
        'plan_path_y': path_y,
        'plan_path_heading': path_heading,
    })

  # envelop
  for k in range(len(path_x)):
    car_xn = []
    car_yn = []
    for i in range(len(car_polygon_x)):
        tmp_x, tmp_y = local2global(car_polygon_x[i], car_polygon_y[i], path_x[k], path_y[k], path_heading[k])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)

  data_astar_path_envelop.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })

  data_path_end.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  # print('envelop')

  # virtual wall line
  # line_list = replay_simulation_hybrid_astar.GetVirtualWall()
  # line_list_x_vec, line_list_y_vec = [], []

  # for i in range(len(line_list)):
  #   obs_point_num = len(line_list[i])
  #   if obs_point_num < 4:
  #     continue

  #   # print('start ', line_list[i][0], line_list[i][1],
  #   #       'end ', line_list[i][2], line_list[i][3])

  #   line_list_x_vec.append([line_list[i][0], line_list[i][2]])
  #   line_list_y_vec.append([line_list[i][1], line_list[i][3]])

  # data_virtual_wall.data.update({
  #       'x_vec': [],
  #       'y_vec': [],
  #   })
  # data_virtual_wall.data.update({
  #   'x_vec': line_list_x_vec,
  #   'y_vec': line_list_y_vec,})

  # circle obs
  obs_pts = replay_simulation_hybrid_astar.GetVirtualWall()
  obs_pt_x, obs_pt_y = [], []
  for i in range(len(obs_pts)):
    obs_pt_x.append(obs_pts[i][0])
    obs_pt_y.append(obs_pts[i][1])

  data_obstacle_points.data.update({
    'x': obs_pt_x,
    'y': obs_pt_y,
  })

  print('virtual_wall')

  # get all rs path
  paths = replay_simulation_hybrid_astar.GetRSHeuristicPath()
  path_x = []
  path_y = []

  all_rs_heuristic_path.data.update({
       'x': path_x,
       'y': path_y,
  })

  start=[]
  end=[]

  for k in range(len(paths)):
    for i in range(len(paths[k])-1):
        start = paths[k][i]

        end = paths[k][i+1]

        path_x.append([start[0], end[0]])
        path_y.append([start[1], end[1]])

  all_rs_heuristic_path.data.update({
      'x': path_x,
      'y': path_y,
  })

  # ref line
  ref_line_x =[]
  ref_line_y =[]
  ref_line_heading =[]

  points = replay_simulation_hybrid_astar.GetPlotRefLine()
  ref_line_x.append(points[0][0])
  ref_line_x.append(points[1][0])

  ref_line_y.append(points[0][1])
  ref_line_y.append(points[1][1])

  data_plot_ref_line.data.update({
        'plan_path_x': ref_line_x,
        'plan_path_y': ref_line_y,
  })


  # open list search sequence path
  data_search_sequence_path.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  path = replay_simulation_hybrid_astar.GetSearchSequencePath()
  path_x = []
  path_y = []

  for i in range(len(path)):
     if (i > search_sequence_num):
       break
     path_x.append(path[i][0])
     path_y.append(path[i][1])

  data_search_sequence_path.data.update({
    'x_vec': path_x,
    'y_vec': path_y
  })

  end_time6 = time.time()
  print('time6, ms ', (end_time6 - end_time5) * 1000)
  # print('loop over')

  pose = replay_simulation_hybrid_astar.GetCoordinateSystem()
  data_coordinate_system.data.update({
    'x': [pose[0]],
    'y': [pose[1]],
  })

  # all delete node
  data_all_delete_node.data.update({
    'x_vec': [],
    'y_vec': [],
  })

  nodes = replay_simulation_hybrid_astar.GetDelNodeSequencePath()
  node_x = []
  node_y = []

  for i in range(len(nodes)):
    node_x.append(nodes[i][0])
    node_y.append(nodes[i][1])

  data_all_delete_node.data.update({
    'x_vec': node_x,
    'y_vec': node_y
  })

  # all search node
  data_all_search_node.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  nodes = replay_simulation_hybrid_astar.GetAllSearchNode()
  safe_node_x = []
  safe_node_y = []
  collision_node_x = []
  collision_node_y = []
  gear_switch_node_x = []
  gear_switch_node_y = []

  for i in range(len(nodes)):
    if (nodes[i][2] > 0.8):
      if nodes[i][3] > 0.8:
        gear_switch_node_x.append(nodes[i][0])
        gear_switch_node_y.append(nodes[i][1])
      else:
        safe_node_x.append(nodes[i][0])
        safe_node_y.append(nodes[i][1])
    else:
      collision_node_x.append(nodes[i][0])
      collision_node_y.append(nodes[i][1])

  data_all_search_node.data.update({
    'x_vec': safe_node_x,
    'y_vec': safe_node_y
  })
  data_all_search_collision_node.data.update({
    'x_vec': collision_node_x,
    'y_vec': collision_node_y
  })
  data_gear_switch_node.data.update({
    'x_vec': gear_switch_node_x,
    'y_vec': gear_switch_node_y
  })

  # plot polynomial
  polynomial_path = replay_simulation_hybrid_astar.GetPolynomialPath()
  plan_path_x = []
  plan_path_y = []
  plan_path_heading = []

  for i in range(len(polynomial_path)):
     plan_path_x.append(polynomial_path[i][0])
     plan_path_y.append(polynomial_path[i][1])
     plan_path_heading.append(polynomial_path[i][2])

  data_polynomial_path.data.update({
    'plan_path_x': plan_path_x,
    'plan_path_y': plan_path_y,
    'plan_path_heading': plan_path_heading,
  })

  if (is_reset):
    replay_simulation_hybrid_astar.StopPybind()

  if plot_speed:
    dp_speed_constraints = replay_simulation_hybrid_astar.GetDpSpeedConstraints()
    qp_speed_constraints = replay_simulation_hybrid_astar.GetQPSpeedConstraints()
    ref_cruise_speed = replay_simulation_hybrid_astar.GetRefCruiseSpeed()
    dp_speed_data = replay_simulation_hybrid_astar.GetDPSpeedOptimizationData()
    qp_speed_data = replay_simulation_hybrid_astar.GetQPSpeedOptimizationData()

    update_lon_plan_online_data(
        dp_speed_constraints, qp_speed_constraints, ref_cruise_speed,
        dp_speed_data, qp_speed_data, lon_plan_data)

    # jlt data
    jlt_speed_data = replay_simulation_hybrid_astar.GetJLTSpeedData()
    update_jlt_online_data(jlt_speed_data, lon_plan_data)

    # online traj speed
    update_publish_data(tuned_planning_output,lon_plan_data)

    update_record_speed_data(traj_speed_profile, lon_plan_data)
    non_optimizer_traj.data.update({
      'plan_path_x': non_optimizer_path_x,
      'plan_path_y': non_optimizer_path_y,
      'plan_path_heading': non_optimizer_path_theta,
      })

  push_notebook()

  print('pybind end')

if plot_speed:
  bkp.show(row(fig1, pans), notebook_handle=True)
else:
  bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
