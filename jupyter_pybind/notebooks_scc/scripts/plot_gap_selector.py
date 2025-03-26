import sys, os
sys.path.append("..")
from lib.load_local_view_gs import *
from lib.load_gap_selector import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')
from bokeh.models import ColumnDataSource, DataTable, TableColumn, TextInput
from ipywidgets import Layout
from python_proto import gap_selector_pb2
from bokeh.resources import INLINE
# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/jac_s811_35kw2/trigger/20240220/20240220-16-03-05/data_collection_JAC_S811_35KW2_EVENT_MANUAL_2024-02-20-16-03-05.record.1708496697.plan"
frame_dt = 0.1 # sec

display(HTML("<style>.container {width:95% !important;  } </style>"))
output_notebook(resources=INLINE)

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

#st data define
data_ego_time_optimal_st = ColumnDataSource(data = {'t_vec':[], 's_vec':[]})
data_front_gap_car_st = ColumnDataSource(data = {'t_vec':[], 's_vec':[]})
data_rear_gap_car_st = ColumnDataSource(data = {'t_vec':[], 's_vec':[]})
#gs data define

data_refline_point = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_refline_point, size=8, color='yellow', legend_label = 'refline_point', visible = True)

data_gs_traj = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 't_vec':[], 'v_vec':[], 's_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_gs_traj, size=8, color='red', legend_label = 'gs_traj', visible = True)

data_last_path_spline = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.line('y_vec', 'x_vec', source = data_last_path_spline, line_width = 2, line_color='red', line_dash = 'solid', line_alpha = 0.7,legend_label = 'last path spline points')

data_cross_line_point = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_cross_line_point,size=6, color='purple', legend_label = 'cross_line_point', visible = True)

data_qutic_p0= ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_qutic_p0,size=6, color='black', legend_label = 'data_qutic_p0', visible = True)

data_qutic_pe = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_qutic_pe,size=6, color='yellow', legend_label = 'data_qutic_pe', visible = True)

data_qutic_stitch_pe = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
fig1.circle('y_vec', 'x_vec', source = data_qutic_stitch_pe,size=6, color='blue', legend_label = 'data_qutic_stitch_pe', visible = True)


fig2 = bkp.figure(x_axis_label = 't', y_axis_label ='x',width=600, height =300)
try:
  fig2.line('t_vec', 'x_vec', source = data_gs_traj, line_width = 2, line_color = 'blue', line_dash = 'solid', line_alpha = 0.4, legend_label = 'x-t')
  fig2.line('t_vec', 'y_vec', source = data_gs_traj, line_width = 2, line_color = 'red', line_dash = 'solid', line_alpha = 0.4, legend_label = 'y-t')
except:
  print("no data trj x")

fig7 = bkp.figure(x_axis_label = 't', y_axis_label = 's', width = 600, height =300)
fig7.circle('t_vec', 's_vec', source = data_ego_time_optimal_st, size=6, color='green', legend_label = 'ego_time_optimal_st', visible = True)
fig7.circle('t_vec', 's_vec', source = data_front_gap_car_st, size=6, color='red', legend_label = 'data_front_gap_car_st', visible = True)
fig7.circle('t_vec', 's_vec', source = data_rear_gap_car_st, size=6, color='grey', legend_label = 'data_rear_gap_car_st', visible = True)
#general table text
general_table_text = ColumnDataSource(data = {'NameAttr':[], 'Value':[]})
general_table_columns = [
                TableColumn(field="NameAttr", title="Name"),
                TableColumn(field="Value", title="Value")
                ]
general_vision_gs_attr_vec = ['lane_cross', 'lc_triggered', \
                       'lb_triggered','lc_in',\
                       'lb_in', 'gs_skip',\
                       'lc_cancel', 'lc_request',\
                       'lc_pass_time', 'lc_wait_time',\
                       'lc_request_buffer', 'ego_l_buffer',\
                       'path_requintic',
                       'origin_lane_id',\
                       'target_lane_id',\
                       'current_lane_id',\
                       'front_car_id',\
                       'rear_car_id',\
                       'gap_status']


tab1 = DataTable(source=general_table_text, columns=general_table_columns, width=500, height=530)

# try:
#   gap_selector_input = bag_loader.plan_debug_msg['data'][-1].gap_selector_input
# except:
#   print('check gap_selector_input',bag_loader.plan_debug_msg['data'][-1])
#   pass

fig1, fig2, fig3, fig4, fig5, fig6, fig10, fig11, lat_plan_data = load_lat_plan_figure(fig1)

coord_tf = coord_transformer()

# ### sliders config
class LocalViewSlider:
  def __init__(self, slider_callback):
    self.time_slider =ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)

    ipywidgets.interact(slider_callback, bag_time = self.time_slider)

def load_gap_selector_replay_info(gap_selector_replay_info,refline_info,
                           coord_tf):
  gs_traj_x_global = []
  gs_traj_y_global = []
  gs_traj_x_local = []
  gs_traj_y_local = []

  gs_traj_v = []
  gs_traj_s = []
  gs_traj_t = []

  for i in range(len(gap_selector_replay_info.gs_traj_points)):
    gs_traj_x_global.append(gap_selector_replay_info.gs_traj_points[i].x)
    gs_traj_y_global.append(gap_selector_replay_info.gs_traj_points[i].y)
    gs_traj_v.append(gap_selector_replay_info.gs_traj_points[i].v)
    gs_traj_s.append(gap_selector_replay_info.gs_traj_points[i].s)
    gs_traj_t.append(gap_selector_replay_info.gs_traj_points[i].t)

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

  path_traj_x_global = []
  path_traj_y_global = []
  path_traj_x_local = []
  path_traj_y_local = []
  for j in range(len(gap_selector_replay_info.gap_selector_path_spline.x_vector_spline)):
      path_traj_x_global.append(gap_selector_replay_info.gap_selector_path_spline.x_vector_spline[j])
      path_traj_y_global.append(gap_selector_replay_info.gap_selector_path_spline.y_vector_spline[j])
  try:
      path_traj_x_local, path_traj_y_local = coord_tf.global_to_local(path_traj_x_global, path_traj_y_global)
      #print("path spline x", path_traj_x_local)
  except:
      print("path_spline error")
  try:
    data_last_path_spline.data.update({
          'x_vec': path_traj_x_local,
          'y_vec': path_traj_y_local,})
  except:
    print("path spline data error")

  cross_line_point_x_global =[]
  cross_line_point_y_global = []
  cross_line_point_x_local = []
  cross_line_point_y_local = []
  try:
    cross_line_point_x_global.append(gap_selector_replay_info.cross_line_point_global.x)
    cross_line_point_y_global.append(gap_selector_replay_info.cross_line_point_global.y)
    cross_line_point_x_local, cross_line_point_y_local = coord_tf.global_to_local(cross_line_point_x_global, cross_line_point_y_global)
    data_cross_line_point.data.update({'y_vec':cross_line_point_y_local, 'x_vec':cross_line_point_x_local})
  except:
    print("no data cross line point ")

  front_gap_car_s = []
  front_gap_car_t = []
  for i in range(len(gap_selector_replay_info.obstacle_predicate_points_front_gap_car)):
    front_gap_car_s.append(gap_selector_replay_info.obstacle_predicate_points_front_gap_car[i].s)
    front_gap_car_t.append(gap_selector_replay_info.obstacle_predicate_points_front_gap_car[i].t)
    print("\n front obj t",gap_selector_replay_info.obstacle_predicate_points_front_gap_car[i].t)
  try:
    data_front_gap_car_st.data.update({
          's_vec': front_gap_car_s,
          't_vec': front_gap_car_t})
  except:
    print("front gap car st error")

  rear_gap_car_s = []
  rear_gap_car_t = []
  for i in range(len(gap_selector_replay_info.obstacle_predicate_points_rear_gap_car)):
    rear_gap_car_s.append(gap_selector_replay_info.obstacle_predicate_points_rear_gap_car[i].s)
    rear_gap_car_t.append(gap_selector_replay_info.obstacle_predicate_points_rear_gap_car[i].t)
  try:
    data_rear_gap_car_st.data.update({
          's_vec': rear_gap_car_s,
          't_vec': rear_gap_car_t})
  except:
    print("rear gap car st error")

  ego_time_optimal_s = []
  ego_time_optimal_t = []
  for i in range(len(gap_selector_replay_info.ego_time_optimal)):
    ego_time_optimal_s.append(gap_selector_replay_info.ego_time_optimal[i].s)
    ego_time_optimal_t.append(gap_selector_replay_info.ego_time_optimal[i].t)
  try:
    data_ego_time_optimal_st.data.update({
          's_vec': ego_time_optimal_s,
          't_vec': ego_time_optimal_t})
  except:
    print("ego st error")

  refline_x_vec = []
  refline_y_vec = []
  refline_x_vec_local = []
  refline_y_vec_local = []
  for i in range(len(refline_info)):
    refline_x_vec.append(refline_info[i].x)
    refline_y_vec.append(refline_info[i].y)
  try:
    refline_x_vec_local, refline_y_vec_local = coord_tf.global_to_local(refline_x_vec, refline_y_vec)
  except:
    print("no refline point ")
  try:
    data_refline_point.data.update({'y_vec':refline_y_vec_local, 'x_vec':refline_x_vec_local})
  except:
    print("rear gap car st error")

  quintic_p0_x_local = []
  quintic_p0_y_local = []
  quintic_pe_x_local = []
  quintic_pe_y_local = []
  stitch_pe_x_local = []
  stitch_pe_y_local = []
  print("\ndata_qutic_p0", gap_selector_replay_info.quintic_p0)
  try:
    quintic_p0_x = []
    quintic_p0_y = []
    quintic_p0_x.append(gap_selector_replay_info.quintic_p0.x)
    quintic_p0_y.append(gap_selector_replay_info.quintic_p0.y)
    quintic_p0_x_local, quintic_p0_y_local = coord_tf.global_to_local(quintic_p0_x,quintic_p0_y)

    quintic_pe_x = []
    quintic_pe_y = []
    print("\n quintic_pe", gap_selector_replay_info.quintic_pe )
    quintic_pe_x.append(gap_selector_replay_info.quintic_pe.x)
    quintic_pe_y.append(gap_selector_replay_info.quintic_pe.y)
    quintic_pe_x_local, quintic_pe_y_local = coord_tf.global_to_local(quintic_pe_x, quintic_pe_y)

    stitch_pe_x = []
    stitch_pe_y = []
    stitch_pe_x.append(gap_selector_replay_info.quintic_stitched_p.x)
    stitch_pe_y.append(gap_selector_replay_info.quintic_stitched_p.y)
    stitch_pe_x_local, stitch_pe_y_local = coord_tf.global_to_local(stitch_pe_x, stitch_pe_y)
    data_qutic_p0.data.update({'y_vec':quintic_p0_y_local, 'x_vec':quintic_p0_x_local})
    data_qutic_pe.data.update({'y_vec':quintic_pe_y_local, 'x_vec':quintic_pe_x_local})
    data_qutic_stitch_pe.data.update({'y_vec':stitch_pe_y_local, 'x_vec':stitch_pe_x_local})


  except:
    print("no quintic path info ")



#### slider_callback
def slider_callback(bag_time):
  kwargs = locals()
  coord_info = [0, 0, 0]
  coord_tf = update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']
  try:
    gap_selector_replay_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].gap_selector_replay_info
    refline_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].refline_info
  except:
    print("Input failed!")
  try:
    print("\ninit state x: ",  bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.init_state.x)
    print("\ninit state y: ",  bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input.init_state.y)
  except:
    print("no lat input")
  general_vision_value_vec =[]
  general_vision_value_vec.append(gap_selector_replay_info.lane_cross)
  general_vision_value_vec.append(gap_selector_replay_info.lc_triggered)
  general_vision_value_vec.append(gap_selector_replay_info.lb_triggered)
  general_vision_value_vec.append(gap_selector_replay_info.lc_in)
  general_vision_value_vec.append(gap_selector_replay_info.lb_in)
  general_vision_value_vec.append(gap_selector_replay_info.gs_skip)
  general_vision_value_vec.append(gap_selector_replay_info.lc_cancel)
  general_vision_value_vec.append(gap_selector_replay_info.lc_request)
  general_vision_value_vec.append(gap_selector_replay_info.lc_pass_time)
  general_vision_value_vec.append(gap_selector_replay_info.lc_wait_time)
  try:
    general_vision_value_vec.append('[' + str(gap_selector_replay_info.lc_request_buffer[0]) + ', ' + str(gap_selector_replay_info.lc_request_buffer[1])+', ' +str(gap_selector_replay_info.lc_request_buffer[2])+ ']')
    general_vision_value_vec.append('[' + str(round(gap_selector_replay_info.ego_l_buffer[0],2)) + ', ' + str(round(gap_selector_replay_info.ego_l_buffer[1],2))+', ' +str(round(gap_selector_replay_info.ego_l_buffer[2],2))+ ']')
  except:
    general_vision_value_vec.append('[error]')
    general_vision_value_vec.append('[error]')
  general_vision_value_vec.append(gap_selector_replay_info.path_requintic)
  general_vision_value_vec.append(gap_selector_replay_info.origin_lane_id)
  general_vision_value_vec.append(gap_selector_replay_info.target_lane_id)
  general_vision_value_vec.append(gap_selector_replay_info.current_lane_id)
  general_vision_value_vec.append(gap_selector_replay_info.nearby_gap.front_agent_id)
  general_vision_value_vec.append(gap_selector_replay_info.nearby_gap.rear_agent_id)
  general_vision_value_vec.append(str(gap_selector_replay_info.gap_selector_status))

  general_table_text.data.update({'NameAttr': general_vision_gs_attr_vec, 'Value': general_vision_value_vec})

  load_gap_selector_replay_info(gap_selector_replay_info,refline_info,
                           coord_tf)
  push_notebook()

# load lateral planning
bkp.show(row(fig1, column(tab1),fig7), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)




