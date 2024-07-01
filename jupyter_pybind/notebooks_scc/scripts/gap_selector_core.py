# This script is used by python gdb

import sys, os
sys.path.append("..")
from lib.load_local_view_gs import *
from lib.load_gap_selector import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')
from python_proto import gap_selector_pb2
from jupyter_pybind import gap_selector_py

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/jac_s811_35kw2/trigger/20240104/20240104-18-51-49/data_collection_JAC_S811_35KW2_EVENT_MANUAL_2024-01-04-18-51-49.record.1704797644.plan"
frame_dt = 0.1 # sec

display(HTML("<style>.container {width:95% !important;  } </style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

#gs data define
data_gs_traj = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 't_vec':[], 'v_vec':[], 's_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_gs_traj, size=8, color='red', legend_label = 'gs_traj', visible = True)

data_cur_path_spline = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_last_path_spline = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.line('y_vec','x_vec', source = data_cur_path_spline,  line_width = 2, line_color = 'red', line_dash = 'solid', line_alpha = 0.4, legend_label = 'current_path_spline')
fig1.line('y_vec', 'x_vec', source = data_last_path_spline, line_width = 2, line_color='red', line_dash = 'solid', line_alpha = 0.7,legend_label = 'last path spline points')

data_frenet_coord_point = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_target_coord_point = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_frenet_coord_point, size=2, color='black', legend_label = 'frenet_coord_point', visible = True)
fig1.circle('y_vec', 'x_vec', source = data_target_coord_point, size=2, color='green', legend_label = 'target_coord_point', visible = True)
data_obstacle_predicate_point = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.line('y_vec', 'x_vec', source = data_obstacle_predicate_point, line_width = 5, color='purple', line_dash = 'dashed', legend_label = 'obstacle_predicate_point', visible = True)

origin_spline_points_vec = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
stitch_spline_points_vec = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.line('y_vec', 'x_vec', source = origin_spline_points_vec, line_width = 2, color='purple', line_dash = 'solid', legend_label = 'origin_spline_points', visible = True)
fig1.line('y_vec', 'x_vec', source = stitch_spline_points_vec, line_width = 2, color='purple', line_dash = 'dashed', legend_label = 'stitch_spline_points', visible = True)



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
fig8.line('t_vec', 's_vec', source = data_gs_traj, line_width = 2, line_color = 'blue', line_dash = 'solid', line_alpha = 0.4, legend_label = 'S-T')
data_upper_st_boundary = ColumnDataSource(data = {'t_vec':[], 's_vec':[]})
data_lower_st_boundary = ColumnDataSource(data = {'t_vec':[], 's_vec':[]})
try:
  fig8.line('t_vec', 's_vec', source = data_upper_st_boundary, line_width = 2, line_color = 'red', line_dash = 'solid', line_alpha = 0.4, legend_label = 'S-T-Upper')
except:
  print("no upper boundary")
try:
  fig8.line('t_vec', 's_vec', source = data_lower_st_boundary, line_width = 2, line_color = 'grey', line_dash = 'solid', line_alpha = 0.4, legend_label = 'S-T-Lower')
except:
  print("no lower boundary")

#gap obj
data_front_gap_obj = ColumnDataSource(data = {'pos_y_local':[], 'pos_x_local':[],
                                        })
data_rear_gap_obj = ColumnDataSource(data = {'pos_y_local':[], 'pos_x_local':[],
                                        })
try:
  fig1.patches('pos_y_local', 'pos_x_local', source = data_front_gap_obj, fill_color = "pink", line_color = "pink", line_width = 1, fill_alpha = 0.3, legend_label = 'front_gap_car')
except:
  print("plot no front gap")
try:
  fig1.patches('pos_y_local', 'pos_x_local', source = data_rear_gap_obj, fill_color = "yellow", line_color = "yellow", line_width = 1, fill_alpha = 0.5, legend_label = 'rear_gap_car')
except:
  print("plot no rear gap")
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
    self.time_slider =ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    #self.left_turn_signal = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "left_turn_signal",min=0., max=1, value=0.5, step=1)
    #self.right_turn_signal = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "right_turn_signal",min=0., max=1, value=0, step=1)
    #self.clear_state_machine = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "clear_state_machine",min=0., max=1, value=0, step=1)
    self.closed_loop = ipywidgets.Checkbox( description= "closed_loop", value=False)

    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                        #left_turn_signal = self.left_turn_signal,
                       # right_turn_signal = self.right_turn_signal,
                        #clear_state_machine = self.clear_state_machine,
                        closed_loop = self.closed_loop
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

  obj_upper_st_boundary_s = []
  obj_upper_st_boundary_t = []
  for i in range(len(gap_selector_output.upper_obj_st_points)):
    obj_upper_st_boundary_s.append(gap_selector_output.upper_obj_st_points[i].s)
    obj_upper_st_boundary_t.append(gap_selector_output.upper_obj_st_points[i].t)
  data_upper_st_boundary.data.update({
    's_vec': obj_upper_st_boundary_s,
    't_vec': obj_upper_st_boundary_t,
  })

  obj_lower_st_boundary_s = []
  obj_lower_st_boundary_t = []
  for i in range(len(gap_selector_output.lower_obj_st_points)):
    obj_lower_st_boundary_s.append(gap_selector_output.lower_obj_st_points[i].s)
    obj_lower_st_boundary_t.append(gap_selector_output.lower_obj_st_points[i].t)
  data_lower_st_boundary.data.update({
    's_vec': obj_lower_st_boundary_s,
    't_vec': obj_lower_st_boundary_t,
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
    if i == 0:
      data_cur_path_spline.data.update({
        'x_vec': path_traj_x_local,
        'y_vec': path_traj_y_local,})
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

  # obstacle_predicate_point_x_global = []
  # obstacle_predicate_point_y_global = []
  # obstacle_predicate_point_x_local = []
  # obstacle_predicate_point_y_local = []
  # for i in range(len(gap_selector_output.obstacle_predicate_points)):
  #   obstacle_predicate_point_x_global.append(gap_selector_output.obstacle_predicate_points[i].x)
  #   obstacle_predicate_point_y_global.append(gap_selector_output.obstacle_predicate_points[i].y)

  # try:
  #   obstacle_predicate_point_x_local, obstacle_predicate_point_y_local = coord_tf.global_to_local(obstacle_predicate_point_x_global, obstacle_predicate_point_y_global)
  #   # print("obstacle_predicate", gap_selector_output.obstacle_predicate_points)
  # except:
  #   print("obstacle_predicate error")

  # data_obstacle_predicate_point.data.update({
  #   'x_vec': obstacle_predicate_point_x_local,
  #   'y_vec': obstacle_predicate_point_y_local,
  # })

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
  # front_gap_obj_x_vec_global =[]
  # front_gap_obj_y_vec_global =[]
  # front_gap_obj_x_vec =[]
  # front_gap_obj_y_vec =[]
  # try:
  #   for i in range(len(front_gap_obj.x_vec)):
  #     front_gap_obj_x_vec_global.append(front_gap_obj.x_vec[i])
  #     front_gap_obj_y_vec_global.append(front_gap_obj.y_vec[i])
  #   front_gap_obj_x_vec, front_gap_obj_y_vec = coord_tf.global_to_local(front_gap_obj_x_vec_global, front_gap_obj_y_vec_global)
  #   data_front_gap_obj.data.update({'pos_y_local':front_gap_obj_y_vec,
  #                                   'pos_x_local':front_gap_obj_x_vec})
  # except:
  #   print("no front gap car ")

  # rear_gap_obj_x_vec_global =[]
  # rear_gap_obj_y_vec_global =[]
  # rear_gap_obj_x_vec =[]
  # rear_gap_obj_y_vec =[]
  # try:
  #   for i in range(len(rear_gap_obj.x_vec)):
  #     rear_gap_obj_x_vec_global.append(rear_gap_obj.x_vec[i])
  #     rear_gap_obj_y_vec_global.append(rear_gap_obj.y_vec[i])
  #   rear_gap_obj_x_vec, rear_gap_obj_y_vec = coord_tf.global_to_local(rear_gap_obj_x_vec_global, rear_gap_obj_y_vec_global)
  #   data_rear_gap_obj.data.update({'pos_y_local':rear_gap_obj_y_vec,
  #                                  'pos_x_local':rear_gap_obj_x_vec})
  # except:
  #   print("no rear gap car ")

#### slider_callback
for bag_time in np.arange(0., max_time, 0.1):
  closed_loop = False
  enable = True
  front_obj_id = -1
  front_obj_add_vel = 0.0
  front_obj_add_s = 0.0
  rear_obj_id = -1
  rear_obj_add_vel = 0.0
  rear_obj_add_s = 0.0
  v_cruise = -1

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
  # print("\n closed_loop:", closed_loop)
  # print("\n enable:", enable)
  # print("\n front_obj_id:", front_obj_id)
  # print("\n front_obj_add_vel:", front_obj_add_vel)
  # print("\n front_obj_add_s:", front_obj_add_s)
  # print("\n rear_obj_id:", rear_obj_id)
  # print("\n rear_obj_add_vel:", rear_obj_add_vel)
  # print("\n rear_obj_add_vel:", rear_obj_add_vel)
  # print("\n rear_obj_add_s:", rear_obj_add_s)
  # print("\n v_cruise:", v_cruise)

  gap_selector_status = gap_selector_py.UpdateBytes( gap_selector_input_string, bool(closed_loop), bool(enable),  int(front_obj_id), \
                    front_obj_add_vel,
                    front_obj_add_s ,
                    int(rear_obj_id),
                    rear_obj_add_vel,
                    rear_obj_add_s,
                    v_cruise)
  gap_selector_output = gap_selector_pb2.GapSelectorOutput()
  gap_selector_output.ParseFromString(gap_selector_py.GetOutputBytes())

  drive_style = gap_selector_py.GetDriveStyle()
  state_limit = gap_selector_pb2.StateLimit()
  state_limit.ParseFromString(gap_selector_py.TimeOptimalStateLimit())

  vision_info = gap_selector_pb2.VisionInfo()
  vision_info.ParseFromString(gap_selector_py.GetVisionInfoBytes())

  # rear_gap_obj = gap_selector_pb2.ObstacleCornalPoints()
  # rear_gap_obj.ParseFromString(gap_selector_py.GetRearGapAgentInfo())


  #table print info
  # print("gap_selector_status:\n ", gap_selector_status)
  # print("drive_style:\n ", drive_style)
  # print("v_end:\n", state_limit.v_end)

  load_gap_selector_output(gap_selector_output,state_limit,
                           #front_gap_obj, rear_gap_obj,
                           coord_tf)
  # print("front_gap_obj:\n", gap_selector_py.GetFrontGapAgentInfo())
  # print("rear_gap_obj:\n", gap_selector_py.GetRearGapAgentInfo())
  # try:
  #   print("obj_x0:\n, ", gap_selector_output.obstacle_predicate_points[0].x)
  # except:
  #   print("no obj pred points")
  push_notebook()




