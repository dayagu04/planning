import sys, os
sys.path.append("..")
from lib.load_local_view_gs import *
from lib.load_gap_selector import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from bokeh.models import ColumnDataSource, DataTable, TableColumn, TextInput
from ipywidgets import Layout
from python_proto import common_pb2, gap_selector_pb2
from jupyter_pybind import gap_selector_py

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/jac_s811_35kw2/trigger/20240104/20240104-18-51-49/data_collection_JAC_S811_35KW2_EVENT_MANUAL_2024-01-04-18-51-49.record.1704873529.plan"
frame_dt = 0.1 # sec

display(HTML("<style>.container {width:95% !important;  } </style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

#gs data define
data_gs_traj = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 't_vec':[], 'v_vec':[], 's_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_gs_traj, size=8, color='red', legend_label = 'gs_traj', visible = True)

data_last_path_spline = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.line('y_vec', 'x_vec', source = data_last_path_spline, line_width = 2, line_color='red', line_dash = 'solid', line_alpha = 0.7,legend_label = 'last path spline points')

data_frenet_coord_point = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_target_coord_point = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_frenet_coord_point, size=2, color='black', legend_label = 'frenet_coord_point', visible = True)
fig1.circle('y_vec', 'x_vec', source = data_target_coord_point, size=2, color='green', legend_label = 'target_coord_point', visible = True)

data_front_gap_car_point = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_front_gap_car_point, size=6, color='orange', legend_label = 'front gap car traj', visible = True)
data_rear_gap_car_point = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_rear_gap_car_point, size=6, color='pink', legend_label = 'rear gap car traj', visible = True)

origin_spline_points_vec = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
stitch_spline_points_vec = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.line('y_vec', 'x_vec', source = origin_spline_points_vec, line_width = 2, color='purple', line_dash = 'solid', legend_label = 'origin_spline_points', visible = True)
fig1.line('y_vec', 'x_vec', source = stitch_spline_points_vec, line_width = 2, color='purple', line_dash = 'dashed', legend_label = 'stitch_spline_points', visible = True)

data_cross_line_point = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_cross_line_point,size=6, color='purple', legend_label = 'cross_line_point', visible = True)

# point_quintic_x0 = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
# point_quintic_xT = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
# fig1.circle('y_vec', 'x_vec', source = point_quintic_x0, size=6, color='orange', legend_label = 'quintic x0', visible = True)
# fig1.circle('y_vec', 'x_vec', source = point_quintic_xT, size=6, color='green', legend_label = 'quintic xT', visible = True)

state_limit_vec = ColumnDataSource(data = {'t_vec':[], 'v_end_vec':[], 'v_max_vec':[], 'v_min_vec':[]})
fig7 = bkp.figure(x_axis_label='t', y_axis_label='v', width=600, height=300)
fig7.line('t_vec', 'v_vec', source = data_gs_traj, line_width = 2, line_color = 'blue', line_dash = 'solid', line_alpha = 0.4, legend_label = 'V-T')
try:
  fig7.line('t_vec', 'v_end_vec', source = state_limit_vec, line_width = 2, line_color = 'yellow', line_dash = 'solid', line_alpha = 0.4, legend_label = 'v-end')
  fig7.line('t_vec', 'v_max_vec', source = state_limit_vec, line_width = 2, line_color = 'red', line_dash = 'solid', line_alpha = 0.4, legend_label = 'v-max')
  fig7.line('t_vec', 'v_min_vec', source = state_limit_vec, line_width = 2, line_color = 'grey', line_dash = 'solid', line_alpha = 0.4, legend_label = 'v-min')
except:
  print("No state limit !")


fig8 = bkp.figure(x_axis_label='t', y_axis_label='s', width=600, height=300)
fig8.circle('t_vec', 's_vec', source = data_gs_traj, size=6, color='green', legend_label = 'S-T', visible = True)

data_front_gap_car_st_boundary = ColumnDataSource(data = {'t_vec':[], 'upper_vec':[], 'lower_vec':[]})
data_rear_gap_car_st_boundary = ColumnDataSource(data = {'t_vec':[], 'upper_vec':[], 'lower_vec':[]})
data_careful_car_st_boundary = ColumnDataSource(data = {'t_vec':[], 'upper_vec':[], 'lower_vec':[]})

try:
  fig8.circle('t_vec', 'upper_vec', source = data_front_gap_car_st_boundary, size=6, color='red', line_alpha = 0.4, legend_label = 'front-car-upper', visible = True)
  fig8.circle('t_vec', 'lower_vec', source = data_front_gap_car_st_boundary, size=6, color='red',legend_label = 'front-car-lowerr', visible = True)
except:
  print("no front gap car")

try:
  fig8.circle('t_vec', 'upper_vec', source = data_rear_gap_car_st_boundary, size=6, color='blue', line_alpha = 0.4, legend_label = 'rear-car-upper', visible = True)
  fig8.circle('t_vec', 'lower_vec', source = data_rear_gap_car_st_boundary, size=6, color='blue',legend_label = 'rear-car-lowerr', visible = True)
except:
  print("no rear gap car")

try:
  fig8.circle('t_vec', 'upper_vec', source = data_careful_car_st_boundary, size=6, color='grey', line_alpha = 0.4, legend_label = 'careful-car-upper', visible = True)
  fig8.circle('t_vec', 'lower_vec', source = data_careful_car_st_boundary, size=6, color='grey',legend_label = 'careful-car-lowerr', visible = True)
except:
  print("no front careful car in origin lane")

fig9 = bkp.figure(x_axis_label = 't', y_axis_label ='x',width=600, height =300)
try:
  fig9.line('t_vec', 'x_vec', source = data_gs_traj, line_width = 2, line_color = 'blue', line_dash = 'solid', line_alpha = 0.4, legend_label = 'x-t')
  fig9.line('t_vec', 'y_vec', source = data_gs_traj, line_width = 2, line_color = 'red', line_dash = 'solid', line_alpha = 0.4, legend_label = 'y-t')
except:
  print("no data trj x")

#general table text
general_table_text = ColumnDataSource(data = {'NameAttr':[], 'Value':[]})
general_table_columns = [
                TableColumn(field="NameAttr", title="Name"),
                TableColumn(field="Value", title="Value")
                ]
general_vision_gs_attr_vec = ['gap_selector_status', 'drive_style', \
                       'front_gap_obj_id','rear_gap_obj_id',\
                       'refine_lc_time', 'expected_lc_l',\
                       'ego_cur_l', 'expected_lc_s',\
                       'front_gap_obj_cur_s', 'front_gap_obj_raw_vel',\
                       'rear_gap_obj_cur_s', 'rear_gap_obj_raw_vel',\
                       'ego_init_s', 'ego_init_v',\
                       'lc_v_end',\
                       'cur_lc_path_collision_idx', 'last_lc_path_collision_idx',\
                       'last_path_spline_status', 'cur_path_spline_status',\
                       'lh_v_end',\
                       'lc_request_buffer', \
                       'ego_l_buffer',\
                       'lc_request',\
                       'origin_lane_id',\
                       'target_lane_id',\
                       'current_lane_id']

#quintic table text
quintic_table_text = ColumnDataSource(data = {'NameAttr':[], 'Value':[]})
quintic_table_columns = [
                TableColumn(field="NameAttr", title="Name"),
                TableColumn(field="Value", title="Value")
                ]
quintic_vision_gs_attr_vec = ['x0', 'dx0', \
                       'ddx0','xT',\
                       'dxT', 'ddxT',\
                       'lc_time', 'expected_dis',\
                       'expected_l']

#front gap input
front_obj_id = TextInput(value = "-1", title= "front_obj_id")
front_obj_add_vel = TextInput(value = "0", title= "front_obj_add_vel")
front_obj_add_s = TextInput(value = "0", title= "front_obj_add_s")

#rear gap input
rear_obj_id = TextInput(value = "-1", title= "rear_obj_id")
rear_obj_add_vel = TextInput(value = "0", title= "rear_obj_add_vel")
rear_obj_add_s = TextInput(value = "0", title= "rear_obj_add_s")

tab1 = DataTable(source=general_table_text, columns=general_table_columns, width=500, height=530)
tab2 = DataTable(source=quintic_table_text, columns=quintic_table_columns, width=500, height=400)
#init pybind
gap_selector_py.Init()

try:
  gap_selector_input = bag_loader.plan_debug_msg['data'][-1].gap_selector_input
except:
  print('check gap_selector_input',bag_loader.plan_debug_msg['data'][-1])
  pass

fig1, fig2, fig3, fig4, fig5, fig6, fig10, fig11, lat_plan_data = load_lat_plan_figure(fig1)

coord_tf = coord_transformer()

# ### sliders config
class LocalViewSlider:
  def __init__(self, slider_callback):
    self.time_slider =ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.closed_loop = ipywidgets.Checkbox( description= "closed_loop", value=False)
    self.enable = ipywidgets.Checkbox( description= "enable", value=True)
    self.front_obj_id = ipywidgets.IntText(value=-1, description='f_id:')
    self.front_obj_add_vel = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description='f_add_vel:',min=-20.0, max=20.0, value=0.0, step=0.1)
    self.front_obj_add_s = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description='f_add_s:',min=-20.0, max=20.0, value=0.0, step=0.1)
    self.rear_obj_id = ipywidgets.IntText(value=-1, description='r_id:')
    self.rear_obj_add_vel = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description='r_add_vel:',min=-20.0, max=20.0, value=0.0, step=0.1)
    self.rear_obj_add_s = ipywidgets.FloatText(layout=ipywidgets.Layout(width='50%'), description='r_add_s:',min=-20.0, max=20.0, value=0.0, step=0.1)
    self.v_cruise = ipywidgets.FloatText(layout=ipywidgets.Layout(width='50%'), description='v_cruise:',min=-1.0, max=45.0, value=-1.0, step=0.1)

    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                        #left_turn_signal = self.left_turn_signal,
                       # right_turn_signal = self.right_turn_signal,
                        #clear_state_machine = self.clear_state_machine,
                        closed_loop = self.closed_loop,
                        enable = self.enable,
                        front_obj_id = self.front_obj_id,
                        front_obj_add_vel = self.front_obj_add_vel,
                        front_obj_add_s = self.front_obj_add_s,
                        rear_obj_id = self.rear_obj_id,
                        rear_obj_add_vel = self.rear_obj_add_vel,
                        rear_obj_add_s = self.rear_obj_add_s,
                        v_cruise = self.v_cruise
                                         )

def load_gap_selector_output(gap_selector_output,state_limit,
                             #front_gap_obj, rear_gap_obj,
                             coord_tf):
  gs_traj_x_global = []
  gs_traj_y_global = []
  gs_traj_x_local = []
  gs_traj_y_local = []

  gs_traj_v = []
  gs_traj_s = []
  gs_traj_t = []

  for i in range(len(gap_selector_output.gs_traj)):
    gs_traj_x_global.append(gap_selector_output.gs_traj[i].x)
    gs_traj_y_global.append(gap_selector_output.gs_traj[i].y)
    gs_traj_v.append(gap_selector_output.gs_traj[i].v)
    gs_traj_s.append(gap_selector_output.gs_traj[i].s)
    gs_traj_t.append(gap_selector_output.gs_traj[i].t)

  try:
    gs_traj_x_local, gs_traj_y_local = coord_tf.global_to_local(gs_traj_x_global, gs_traj_y_global)
    #print("gap_selector_output", gap_selector_output.gs_traj)
  except:
    print("gs_traj error")

  data_gs_traj.data.update({
    'x_vec': gs_traj_x_local,
    'y_vec': gs_traj_y_local,
    'v_vec': gs_traj_v,
    's_vec': gs_traj_s,
    't_vec': gs_traj_t,
  })

  front_gap_car_upper_vec = []
  front_gap_car_lower_vec = []
  front_gap_car_t_vec = []
  for i in range(len(gap_selector_output.upper_st_boundary_front_car)):
    front_gap_car_upper_vec.append(gap_selector_output.upper_st_boundary_front_car[i].s)
    front_gap_car_lower_vec.append(gap_selector_output.lower_st_boundary_front_car[i].s)
    front_gap_car_t_vec.append(gap_selector_output.lower_st_boundary_front_car[i].t)
  data_front_gap_car_st_boundary.data.update({
    'upper_vec': front_gap_car_upper_vec,
    'lower_vec': front_gap_car_lower_vec,
    't_vec':front_gap_car_t_vec
  })

  rear_gap_car_upper_vec = []
  rear_gap_car_lower_vec = []
  rear_gap_car_t_vec = []
  for i in range(len(gap_selector_output.upper_st_boundary_rear_car)):
    rear_gap_car_upper_vec.append(gap_selector_output.upper_st_boundary_rear_car[i].s)
    rear_gap_car_lower_vec.append(gap_selector_output.lower_st_boundary_rear_car[i].s)
    rear_gap_car_t_vec.append(gap_selector_output.lower_st_boundary_rear_car[i].t)
  data_rear_gap_car_st_boundary.data.update({
    'upper_vec': rear_gap_car_upper_vec,
    'lower_vec': rear_gap_car_lower_vec,
    't_vec':rear_gap_car_t_vec
  })

  careful_car_upper_vec = []
  careful_car_lower_vec = []
  careful_car_t_vec = []
  for i in range(len(gap_selector_output.upper_st_boundary_origin_front_car)):
    careful_car_upper_vec.append(gap_selector_output.upper_st_boundary_origin_front_car[i].s)
    careful_car_lower_vec.append(gap_selector_output.lower_st_boundary_origin_front_car[i].s)
    careful_car_t_vec.append(gap_selector_output.upper_st_boundary_origin_front_car[i].t)
  data_careful_car_st_boundary.data.update({
    'upper_vec': careful_car_upper_vec,
    'lower_vec': careful_car_lower_vec,
    't_vec':careful_car_t_vec
  })

  for i in range(len(gap_selector_output.path_spline_data)):
    path_traj_x_global = []
    path_traj_y_global = []
    path_traj_x_local = []
    path_traj_y_local = []
    for j in range(len(gap_selector_output.path_spline_data[i].points)):
      path_traj_x_global.append(gap_selector_output.path_spline_data[i].points[j].x)
      path_traj_y_global.append(gap_selector_output.path_spline_data[i].points[j].y)
    try:
      path_traj_x_local, path_traj_y_local = coord_tf.global_to_local(path_traj_x_global, path_traj_y_global)
      #print("path spline x", path_traj_x_local)
    except:
      print("path_spline error")
    if i == 1:
      data_last_path_spline.data.update({
        'x_vec': path_traj_x_local,
        'y_vec': path_traj_y_local,})

  frenet_coord_point_x_global = []
  frenet_coord_point_y_global = []
  frenet_coord_point_x_local = []
  frenet_coord_point_y_local = []
  for i in range(len(gap_selector_output.frenet_coord_points)):
    frenet_coord_point_x_global.append(gap_selector_output.frenet_coord_points[i].x)
    frenet_coord_point_y_global.append(gap_selector_output.frenet_coord_points[i].y)
  try:
    frenet_coord_point_x_local, frenet_coord_point_y_local = coord_tf.global_to_local(frenet_coord_point_x_global, frenet_coord_point_y_global)
    # print("frenet_coord_points", gap_selector_output.frenet_coord_points)
  except:
    print("frenet_coord_points error")

  data_frenet_coord_point.data.update({
    'x_vec': frenet_coord_point_x_local,
    'y_vec': frenet_coord_point_y_local,
  })

  target_coord_point_x_global = []
  target_coord_point_y_global = []
  target_coord_point_x_local = []
  target_coord_point_y_local = []
  for i in range(len(gap_selector_output.target_coord_points)):
    target_coord_point_x_global.append(gap_selector_output.target_coord_points[i].x)
    target_coord_point_y_global.append(gap_selector_output.target_coord_points[i].y)

  try:
    target_coord_point_x_local, target_coord_point_y_local = coord_tf.global_to_local(target_coord_point_x_global, target_coord_point_y_global)
    # print("frenet_coord_points", gap_selector_output.frenet_coord_points)
  except:
    print("target_coord_points error")

  data_target_coord_point.data.update({
    'x_vec': target_coord_point_x_local,
    'y_vec': target_coord_point_y_local,
  })

  state_limit_v_end = []
  state_limit_v_max = []
  state_limit_v_min = []
  for i in range(len(gs_traj_t)):
      state_limit_v_end.append(state_limit.v_end)
      state_limit_v_max.append(state_limit.v_max)
      state_limit_v_min.append(state_limit.v_min)
  try:
    state_limit_vec.data.update({'t_vec':gs_traj_t,
                                   'v_end_vec':state_limit_v_end,
                                   'v_max_vec':state_limit_v_max,
                                   'v_min_vec':state_limit_v_min})
  except:
    print("state limit update error")

  origin_spline_points_x_global = []
  origin_spline_points_y_global = []
  origin_spline_points_x_local = []
  origin_spline_points_y_local = []
  for i in range(len(gap_selector_output.origin_spline_points)):
    origin_spline_points_x_global.append(gap_selector_output.origin_spline_points[i].x)
    origin_spline_points_y_global.append(gap_selector_output.origin_spline_points[i].y)

  try:
    origin_spline_points_x_local, origin_spline_points_y_local = coord_tf.global_to_local(origin_spline_points_x_global, origin_spline_points_y_global)
    # print("obstacle_predicate", gap_selector_output.obstacle_predicate_points)
  except:
    print("origin spline points error")

  stitch_spline_points_x_global = []
  stitch_spline_points_y_global = []
  stitch_spline_points_x_local = []
  stitch_spline_points_y_local = []
  try:
    stitch_spline_points_x_global.append(origin_spline_points_x_global[-1])
    stitch_spline_points_y_global.append(origin_spline_points_y_global[-1])
  except:
    print("no origin spline points")

  for i in range(len(gap_selector_output.stitch_spline_points)):
    stitch_spline_points_x_global.append(gap_selector_output.stitch_spline_points[i].x)
    stitch_spline_points_y_global.append(gap_selector_output.stitch_spline_points[i].y)

  try:
    stitch_spline_points_x_local, stitch_spline_points_y_local = coord_tf.global_to_local(stitch_spline_points_x_global, stitch_spline_points_y_global)
    # print("obstacle_predicate", gap_selector_output.obstacle_predicate_points)
  except:
    print("origin spline points error")

  try:
    origin_spline_points_vec.data.update({'x_vec':origin_spline_points_x_local,
                                          'y_vec':origin_spline_points_y_local})
    stitch_spline_points_vec.data.update({'x_vec':stitch_spline_points_x_local,
                                          'y_vec':stitch_spline_points_y_local})
    # print("origin_spline_points x:\n",origin_spline_points_x_global)
    # print("stitch_spline_points y:\n",origin_spline_points_y_global)
    # print("origin_spline_points x:\n",stitch_spline_points_x_global)
    # print("stitch_spline_points y:\n",stitch_spline_points_y_global)
  except:
    print("origin spline points update error")

  front_gap_obj_x_vec_global =[]
  front_gap_obj_y_vec_global =[]
  front_gap_obj_x_vec =[]
  front_gap_obj_y_vec =[]
  try:
    for i in range(len(gap_selector_output.obstacle_predicate_points_front_gap_car)):
      front_gap_obj_x_vec_global.append(gap_selector_output.obstacle_predicate_points_front_gap_car[i].x)
      front_gap_obj_y_vec_global.append(gap_selector_output.obstacle_predicate_points_front_gap_car[i].y)
    front_gap_obj_x_vec, front_gap_obj_y_vec = coord_tf.global_to_local(front_gap_obj_x_vec_global, front_gap_obj_y_vec_global)
    data_front_gap_car_point.data.update({'y_vec':front_gap_obj_y_vec,
                                          'x_vec':front_gap_obj_x_vec})
  except:
    print("no front gap car ")

  rear_gap_obj_x_vec_global =[]
  rear_gap_obj_y_vec_global =[]
  rear_gap_obj_x_vec =[]
  rear_gap_obj_y_vec =[]
  try:
    for i in range(len(gap_selector_output.obstacle_predicate_points_rear_gap_car)):
      rear_gap_obj_x_vec_global.append(gap_selector_output.obstacle_predicate_points_rear_gap_car[i].x)
      rear_gap_obj_y_vec_global.append(gap_selector_output.obstacle_predicate_points_rear_gap_car[i].y)
    rear_gap_obj_x_vec, rear_gap_obj_y_vec = coord_tf.global_to_local(rear_gap_obj_x_vec_global, rear_gap_obj_y_vec_global)
    data_rear_gap_car_point.data.update({'y_vec':rear_gap_obj_y_vec,
                                         'x_vec':rear_gap_obj_x_vec})
  except:
    print("no rear gap car ")

  cross_line_point_x_global =[]
  cross_line_point_y_global = []
  cross_line_point_x_local = []
  cross_line_point_y_local = []
  try:
    cross_line_point_x_global.append(gap_selector_output.cross_line_point.x)
    cross_line_point_y_global.append(gap_selector_output.cross_line_point.y)
    cross_line_point_x_local, cross_line_point_y_local = coord_tf.global_to_local(cross_line_point_x_global, cross_line_point_y_global)
    data_cross_line_point.data.update({'y_vec':cross_line_point_y_local, 'x_vec':cross_line_point_x_local})
  except:
    print("no data cross line point ")

#### slider_callback
def slider_callback(bag_time,
                    closed_loop,
                    enable,
                    front_obj_id,
                    front_obj_add_vel,
                    front_obj_add_s ,
                    rear_obj_id,
                    rear_obj_add_vel,
                    rear_obj_add_s,
                    v_cruise
                    ):
  kwargs = locals()
  coord_info = [0, 0, 0]
  coord_tf = update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']
  try:
    gap_selector_input = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].gap_selector_input
  except:
    print("Input failed!")

  gap_selector_input_string = gap_selector_input.SerializeToString()
  gap_selector_status = gap_selector_py.UpdateBytes(gap_selector_input_string, closed_loop, enable,  int(front_obj_id), \
                    front_obj_add_vel,
                    front_obj_add_s ,
                    int(rear_obj_id),
                    rear_obj_add_vel,
                    rear_obj_add_s,
                    v_cruise)
  gap_selector_output = gap_selector_pb2.GapSelectorOutput()
  gap_selector_output.ParseFromString(gap_selector_py.GetOutputBytes())

  # print("gap selector output\n", gap_selector_output)
  drive_style = gap_selector_py.GetDriveStyle()
  state_limit = gap_selector_pb2.StateLimit()
  state_limit.ParseFromString(gap_selector_py.TimeOptimalStateLimit())

  #table info
  vision_info = gap_selector_pb2.VisionInfo()
  vision_info.ParseFromString(gap_selector_py.GetVisionInfoBytes(gap_selector_input_string))

  vision_gs_value_vec =[]
  vision_gs_value_vec.append(vision_info.gap_selector_status)
  vision_gs_value_vec.append(vision_info.drive_style)
  vision_gs_value_vec.append(vision_info.front_gap_obj_id)
  vision_gs_value_vec.append(vision_info.rear_gap_obj_id)
  vision_gs_value_vec.append(round(vision_info.refine_lc_time,3))
  vision_gs_value_vec.append(round(vision_info.expected_lc_l,2))
  vision_gs_value_vec.append(round(vision_info.ego_cur_l,2))
  vision_gs_value_vec.append(round(vision_info.expected_lc_s,2))
  vision_gs_value_vec.append(round(vision_info.front_gap_obj_cur_s,2))
  vision_gs_value_vec.append(round(vision_info.front_gap_obj_raw_vel,2))
  vision_gs_value_vec.append(round(vision_info.rear_gap_obj_cur_s,2))
  vision_gs_value_vec.append(round(vision_info.rear_gap_obj_raw_vel,2))
  vision_gs_value_vec.append(round(vision_info.ego_init_s,2))
  vision_gs_value_vec.append(round(vision_info.ego_init_v,2))
  vision_gs_value_vec.append(round(vision_info.lc_v_end,2))
  vision_gs_value_vec.append(vision_info.cur_lc_path_collision_idx)
  vision_gs_value_vec.append(vision_info.last_lc_path_collision_idx)
  vision_gs_value_vec.append(vision_info.last_path_spline_status)
  vision_gs_value_vec.append(vision_info.cur_path_spline_status)
  vision_gs_value_vec.append(round(vision_info.lh_v_end,2))
  vision_gs_value_vec.append('[' + str(vision_info.lc_request_buffer[0]) + ', ' + str(vision_info.lc_request_buffer[1])+', ' +str(vision_info.lc_request_buffer[2])+ ']')
  vision_gs_value_vec.append('[' + str(round(vision_info.ego_l_buffer[0],2)) + ', ' + str(round(vision_info.ego_l_buffer[1],2))+', ' +str(round(vision_info.ego_l_buffer[2],2))+ ']')
  vision_gs_value_vec.append(vision_info.request)
  vision_gs_value_vec.append(vision_info.origin_lane_id)
  vision_gs_value_vec.append(vision_info.target_lane_id)
  vision_gs_value_vec.append(vision_info.current_lane_id)

  general_table_text.data.update({'NameAttr': general_vision_gs_attr_vec, 'Value': vision_gs_value_vec})

  quintic_gs_value_vec =[]
  quintic_gs_value_vec.append('(' + str(round(vision_info.quintic_parames.x0.x,2)) + ', ' + str(round(vision_info.quintic_parames.x0.y,2)) + ')')
  quintic_gs_value_vec.append('(' + str(round(vision_info.quintic_parames.dx0.x,2)) + ', ' + str(round(vision_info.quintic_parames.dx0.y,2)) + ')')
  quintic_gs_value_vec.append('(' + str(round(vision_info.quintic_parames.ddx0.x,2)) + ', ' + str(round(vision_info.quintic_parames.ddx0.y,2)) + ')')
  quintic_gs_value_vec.append('(' + str(round(vision_info.quintic_parames.xT.x,2)) + ', ' + str(round(vision_info.quintic_parames.xT.y,2)) + ')')
  quintic_gs_value_vec.append('(' + str(round(vision_info.quintic_parames.dxT.x,2)) + ', ' + str(round(vision_info.quintic_parames.dxT.y,2)) + ')')
  quintic_gs_value_vec.append('(' + str(round(vision_info.quintic_parames.ddxT.x,2)) + ', ' + str(round(vision_info.quintic_parames.ddxT.y,2)) + ')')
  quintic_gs_value_vec.append(round(vision_info.quintic_parames.lc_time,3))
  quintic_gs_value_vec.append(round(vision_info.quintic_parames.expected_dis,2))
  quintic_gs_value_vec.append(round(vision_info.quintic_parames.expected_l,3))
  quintic_table_text.data.update({'NameAttr': quintic_vision_gs_attr_vec, 'Value': quintic_gs_value_vec})

  load_gap_selector_output(gap_selector_output,state_limit,
                           coord_tf)
  push_notebook()

# load lateral planning
bkp.show(row(fig1, column(tab1, tab2), column(fig7, fig8, fig9)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)




