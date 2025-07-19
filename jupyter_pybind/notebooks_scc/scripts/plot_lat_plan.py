import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
from lib.load_lat_plan import *
from bokeh.models import DataTable, TableColumn, Panel, Tabs
from bokeh.resources import INLINE
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_50815/trigger/20250717/20250717-16-45-36/data_collection_CHERY_M32T_50815_EVENT_MANUAL_2025-07-17-16-45-36_no_camera.bag"
# bag_path = "bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_10034/trigger/20240723/20240723-19-33-25/data_collection_CHERY_E0Y_10034_EVENT_MANUAL_2024-07-23-19-33-25_no_camera.bag

# frame dt
frame_dt = 0.1 # sec

# plot global figure?
global_fig_plot = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
# JAC_S811 CHERY_T26 CHERY_E0X CHERY_M32T
global_var.set_value('car_type', 'CHERY_E0X')
# global_var.set_value('g_is_display_enu', False)
# global_var.set_value('is_vis_map', True)
fig1, local_view_data = load_local_view_figure()

# load lateral planning (behavior and motion)
fig1, fig2, fig3, fig4, fig5, fig6, fig7, fig8, fig9, lat_plan_data = load_lat_plan_figure(fig1, local_view_data)
fig1.height = 1500

load_measure_distance_tool(fig1)
fig_12, data_center_line_info = load_center_line_info()
fig_lat_offset = load_lateral_offset(bag_loader)
# data_select_obstacle_polygon = load_select_obstacle_polygon(fig1)

behavior_data_1 = ColumnDataSource({
  'name':[],
  'data':[]
})
columns = [
        TableColumn(field="name", title="name",),
        TableColumn(field="data", title="data"),
    ]
data_behavior_table_1 = DataTable(source=behavior_data_1, columns=columns, width=400, height=250)

def update_lat_behavior_data(lat_behavior_common):
  vars = ['fix_lane_virtual_id','target_lane_virtual_id','origin_lane_virtual_id',\
          'lc_request','lc_request_source','turn_light','map_turn_light','lc_turn_light','act_request_source','lc_back_invalid_reason','lc_status',\
            'is_lc_valid','lc_valid_cnt','lc_invalid_obj_id','lc_invalid_reason',\
      'lc_valid_back','lc_back_obj_id','lc_back_cnt','lc_back_invalid_reason',\
        'v_relative_left_lane','is_faster_left_lane','faster_left_lane_cnt','v_relative_right_lane',\
          'is_faster_right_lane','faster_right_lane_cnt','is_forbid_left_alc_car','is_forbid_right_alc_car',\
            'is_side_borrow_bicycle_lane','is_side_borrow_lane','has_origin_lane',\
              'has_target_lane','enable_left_lc','enable_right_lc','lc_back_reason', ]
  # 'near_car_ids_origin','near_car_ids_target', 'left_alc_car_ids','right_alc_car_ids', ,'avoid_car_ids','avoid_car_allow_max_opposite_offset'
  names  = []
  datas = []
  for name in vars:
    try:
      # print(getattr(vo_lat_behavior_plan,name))
      datas.append(getattr(lat_behavior_common,name))
      names.append(name)
    except:
      pass

  behavior_data_1.data.update({
    'name': names,
    'data': datas,
  })

hmi_hpp_info_data = ColumnDataSource({
  'name':[],
  'data':[]
})
columns = [
        TableColumn(field="name", title="name",),
        TableColumn(field="data", title="data"),
    ]
data_hmi_hpp_info_table = DataTable(source=hmi_hpp_info_data, columns=columns, width=400, height=600)

def update_hmi_hpp_info(hmi_hpp_info):
  vars = ['is_avaliable', 'distance_to_parking_space', 'avoid_status', \
          'avoid_obstacle_type', 'aovid_id', 'is_approaching_turn', \
          'is_left_turn', 'is_approaching_intersection', 'is_approaching_speed_bumps', \
          'emergency_level', 'is_parking_space_occupied', 'is_new_parking_space_found', \
          'is_on_hpp_lane', 'is_reached_hpp_trace_start', 'accumulated_driving_distance', \
          'estimated_remaining_time', 'hpp_state_switch']
  names  = []
  datas = []
  for name in vars:
    try:
      datas.append(getattr(hmi_hpp_info, name))
      names.append(name)
    except:
      pass

  hmi_hpp_info_data.data.update({
    'name': names,
    'data': datas,
  })

def get_plan_debug_msg_idx(bag_loader, bag_time):
  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
    plan_debug_msg = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx]
  return plan_debug_msg_idx

def get_vs_msg_idx(bag_loader, bag_time):
  vs_msg_idx = 0
  if bag_loader.vs_msg['enable'] == True:
    while bag_loader.vs_msg['t'][vs_msg_idx] <= bag_time and vs_msg_idx < (len(bag_loader.vs_msg['t'])-2):
        vs_msg_idx = vs_msg_idx + 1
    vs_msg = bag_loader.vs_msg['data'][vs_msg_idx]
  return vs_msg_idx

fig10 = bkp.figure(x_axis_label='time', y_axis_label='steer deg', width=800, height=160)
fig11 = bkp.figure(x_axis_label='time', y_axis_label='steer dot deg', width=800, height=160)
data_steer = ColumnDataSource(data ={
  'time': [],
  'plan_steer_deg':[],
  'plan_steer_dot_deg':[],
  'ego_steer_deg':[],
  'ego_steer_dot_deg':[]
})
steer_time = []
plan_steer_deg = []
plan_steer_dot_deg = []
ego_steer_deg = []
ego_steer_dot_deg = []
for t in np.arange(0.0, max_time, frame_dt):
  steer_time.append(t)
  # plan_debug_msg
  try:
    plan_debug_msg_idx = get_plan_debug_msg_idx(bag_loader, t)
    lateral_motion_planning_output = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_output
    plan_steer_deg.append(lateral_motion_planning_output.delta_vec[0] * 13 * 57.3)
    plan_steer_dot_deg.append(lateral_motion_planning_output.omega_vec[0] * 13 * 57.3)
  except:
    plan_steer_deg.append(0.0)
    plan_steer_dot_deg.append(0.0)
    pass
  # vs_msg
  try:
    vs_msg_idx = get_vs_msg_idx(bag_loader, t)
    vs_msg = bag_loader.vs_msg['data'][vs_msg_idx]
    ego_steer_deg.append(vs_msg.steering_wheel_angle * 57.3)
    ego_steer_dot_deg.append(vs_msg.steering_wheel_angle_speed * 57.3)
  except:
    ego_steer_deg.append(0.0)
    ego_steer_dot_deg.append(0.0)
    pass

data_steer.data.update({
  'time': steer_time,
  'plan_steer_deg': plan_steer_deg,
  'plan_steer_dot_deg': plan_steer_dot_deg,
  'ego_steer_deg': ego_steer_deg,
  'ego_steer_dot_deg': ego_steer_dot_deg,
})
f8 = fig10.line('time', 'plan_steer_deg', source = data_steer, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'plan_steer_deg')
fig10.line('time', 'ego_steer_deg', source = data_steer, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'ego_steer_deg')
f11 = fig11.line('time', 'plan_steer_dot_deg', source = data_steer, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'plan_steer_dot_deg')
fig11.line('time', 'ego_steer_dot_deg', source = data_steer, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'ego_steer_dot_deg')

hover8 = HoverTool(renderers=[f8], tooltips=[('time', '@time'), ('plan_steer_deg', '@plan_steer_deg'), ('ego_steer_deg', '@ego_steer_deg')], mode='vline')
hover11 = HoverTool(renderers=[f11], tooltips=[('time', '@time'), ('plan_steer_dot_deg', '@plan_steer_dot_deg'), ('ego_steer_dot_deg', '@ego_steer_dot_deg')], mode='vline')

fig10.add_tools(hover8)
fig11.add_tools(hover11)

fig10.toolbar.active_scroll = fig10.select_one(WheelZoomTool)
fig11.toolbar.active_scroll = fig11.select_one(WheelZoomTool)
fig10.legend.click_policy = 'hide'
fig11.legend.click_policy = 'hide'

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.prediction_obstacle_id = ipywidgets.Text(description='predict_id:')
    self.obstacle_polygon_id = ipywidgets.Text(description='polygon_id:')

    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                         prediction_obstacle_id = self.prediction_obstacle_id,
                                         obstacle_polygon_id = self.obstacle_polygon_id)


### sliders callback
def slider_callback(bag_time, prediction_obstacle_id, obstacle_polygon_id):
  kwargs = locals()
  update_select_obstacle_id(prediction_obstacle_id, obstacle_polygon_id, local_view_data)
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_lat_plan_data(fig7, bag_loader, bag_time, local_view_data, lat_plan_data)
  update_center_line_info(data_center_line_info, local_view_data)
  # update_select_obstacle_polygon(data_select_obstacle_polygon, local_view_data)
  if bag_loader.plan_debug_msg['enable'] == True:
    lat_behavior_common = local_view_data['data_msg']['plan_debug_msg'].lat_behavior_common
    update_lat_behavior_data(lat_behavior_common)
  if bag_loader.planning_hmi_msg['enable'] ==True:
    hmi_hpp_info = local_view_data['data_msg']['planning_hmi_msg'].hpp_info
    data_hmi_hpp_info_table = update_hmi_hpp_info(hmi_hpp_info)
  push_notebook()

pan1 = Panel(child=row(column(fig2, fig9, fig3, fig4, fig5, fig6, fig10, fig11, fig_12)), title="CurveFigure")
pan2 = Panel(child=row(column(fig_lat_offset, row(data_behavior_table_1, column(data_hmi_hpp_info_table)))), title="TableInfo")
pan3 = Panel(child=row(column(fig7)), title="!Figure")
pans = Tabs(tabs=[ pan1, pan2, pan3 ])
if global_fig_plot:
  bkp.show(row(fig1, pans), notebook_handle=True)
else:
  bkp.show(row(fig1, column(fig2, fig9, fig3, fig4, fig5, fig6, fig_lat_offset)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
