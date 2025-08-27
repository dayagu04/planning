import sys, os
sys.path.append("..")
sys.path.append("../lib/")
# from lib.load_cyberbag import *
from lib.load_local_view import *
from lib.load_ros_bag import LoadRosbag
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/data_cold/abu_zone/user/thzhang5/ihc_test_0822/data_collection_CHERY_M32T_81865_EVENT_MANUAL_2025-08-25-01-58-26_no_camera.bag.1756281308.open-loop.scc.plan"
frame_dt = 0.02 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

# bag_loader = LoadCyberbag(bag_path)
bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()


data_mpc = ColumnDataSource(data ={
  'time_vec': [],
  'dx_ref_mpc_vec':[],
  'dy_ref_mpc_vec':[],
  'dx_mpc_vec':[],
  'dy_mpc_vec':[],
  'dphi_ref_mpc_vec':[],
  'dphi_mpc_vec':[],
  'delta_mpc_vec': []
})


# dt_json = time_all / len(ctrl_json_data)

#f
adas_json_data = bag_loader.plan_debug_msg['json']
adas_json_data_t = bag_loader.plan_debug_msg['t']
counter = 0

# +
ihc_json_value_list = [
                         #ihc debug info:
                         "ihc_function::ihc_enable_code", "ihc_function::ihc_fault_code", "ihc_function::ihc_state","ihc_function::ihc_request_status",
                         "ihc_function::ihc_request","ihc_function::ihc_main_switch","ihc_function::auto_light_state",
                         "ihc_function::low_beam_due_to_same_dir_vehicle", "ihc_function::low_beam_due_to_oncomming_vehicle", 
                         "ihc_function::low_beam_due_to_oncomming_cycle", "ihc_function::lighting_condition",
                        ]
adas_json_value_list =  [ #adas_debug info
                         "params_dt","params_ego_length","params_ego_width", "params_origin_2_front_bumper", "params_origin_2_rear_bumper", "params_steer_ratio","params_wheel_base",
                         "params_ldp_c0_right_offset", "params_ldp_center_line_offset","params_ldp_ttlc_right_hack","params_ldp_tlc_thrd","params_ldw_enable_speed",
                         "state_left_turn_light_off_time","state_right_turn_light_off_time","state_driver_hand_trq","state_ego_curvature","state_fl_wheel_distance_to_line",
                         "state_fr_wheel_distance_to_line","state_vehicle_speed", "state_yaw_rate","state_left_departure_speed","state_right_departure_speed","state_steer_wheel_angle_degree",
                         "state_yaw_rate_observer",
                         "road_left_line_boundary_type", "road_left_line_line_type","road_left_line_begin","road_left_line_end","road_left_line_c0","road_left_line_c1","road_left_line_c2","road_left_line_c3","state_fl_wheel_distance_to_roadedge",
                         "road_right_line_boundary_type","road_right_line_line_type","road_right_line_begin", "road_right_line_end","road_right_line_c0","road_right_line_c1","road_right_line_c2","road_right_line_c3","state_fr_wheel_distance_to_roadedge",
                         "road_left_line_valid","road_right_line_valid","road_left_roadedge_valid","road_right_roadedge_valid","road_lane_width_valid","road_lane_width",
                         "road_left_roadedge_begin_x","road_left_roadedge_end_x","road_right_roadedge_begin_x","road_right_roadedge_end_x",]
adas_json_value_list = ihc_json_value_list + adas_json_value_list
adas_json_list_dict = {}
adas_t_debug = []
for i in range(len(adas_json_value_list)):
   new_list = []
   adas_json_list_dict[adas_json_value_list[i]] = new_list
for i in range(len(adas_json_data)):
  adas_t_debug.append(adas_json_data_t[i])
  for j in range(len(adas_json_value_list)):
     value = adas_json_data[i][adas_json_value_list[j]]
     adas_json_list_dict[adas_json_value_list[j]].append(value)
adas_json_list_dict['time'] = adas_t_debug

# figures
fig1.line('dy_ref_mpc_vec', 'dx_ref_mpc_vec', source = data_mpc, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref', visible=True)
fig1.line('dy_mpc_vec', 'dx_mpc_vec', source = data_mpc, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.35, legend_label = 'mpc', visible=True)
fig1.width = 700
fig1.height = 600

#f
# fig_control_1 = bkp.figure(x_axis_label='time', y_axis_label='steering angle',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=200)

fig_machine = bkp.figure(x_axis_label='time', y_axis_label='ihc state_machine',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=500, height=200)
fig_machine.yaxis.axis_label_text_font_style = 'bold'

# lighting_condition数值含义:
# 0: UNKNOWN (未知)
# 1: BRIGHT (明亮环境)
# 2: MEDIUM (中等亮度环境)  
# 3: DARK (昏暗环境)

fig_state_code = bkp.figure(x_axis_label='time', y_axis_label='ihc code',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=500, height=200)
fig_state_code.yaxis.axis_label_text_font_style = 'bold'

fig_low_beam_reason = bkp.figure(x_axis_label='time', y_axis_label='low beam reason',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=500, height=200)
fig_low_beam_reason.yaxis.axis_label_text_font_style = 'bold'

# fig_dynamic_state = bkp.figure(x_axis_label='time', y_axis_label='ihc state',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=200)
# fig_dynamic_state.yaxis.axis_label_text_font_style = 'bold'

f_machine = fig_machine.line('time', 'ihc_function::ihc_state', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'ihc_state')
fig_machine.line('time', 'ihc_function::ihc_request_status', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'ihc_request_status')
fig_machine.line('time', 'ihc_function::ihc_request', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'ihc_request')
fig_machine.line('time', 'ihc_function::ihc_main_switch', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ihc_main_switch')
fig_machine.line('time', 'ihc_function::auto_light_state', source = adas_json_list_dict, line_width = 1, line_color = 'yellow', line_dash = 'solid', legend_label = 'auto_light_state')

f_state_code = fig_state_code.line('time', 'ihc_function::ihc_enable_code', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'ihc_enable_code')
fig_state_code.line('time', 'ihc_function::ihc_fault_code', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'ihc_fault_code')

f_low_beam_reason = fig_low_beam_reason.line('time', 'ihc_function::low_beam_due_to_same_dir_vehicle', source = adas_json_list_dict, line_width = 2, line_color = 'blue', line_dash = 'solid', legend_label = 'same_dir_vehicle')
fig_low_beam_reason.line('time', 'ihc_function::low_beam_due_to_oncomming_vehicle', source = adas_json_list_dict, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'oncomming_vehicle')
fig_low_beam_reason.line('time', 'ihc_function::low_beam_due_to_oncomming_cycle', source = adas_json_list_dict, line_width = 2, line_color = 'orange', line_dash = 'solid', legend_label = 'oncomming_cycle')
fig_low_beam_reason.line('time', 'ihc_function::lighting_condition', source = adas_json_list_dict, line_width = 2, line_color = 'cyan', line_dash = 'solid', legend_label = 'lighting_condition')

hover_machine = HoverTool(renderers=[f_machine], tooltips=[('time', '@time'), ('ihc_state', '@{ihc_function::ihc_state}'),('ihc_request_status', '@{ihc_function::ihc_request_status}'), ('ihc_request', '@{ihc_function::ihc_request}'),
                                                        ('ihc_main_switch', '@{ihc_function::ihc_main_switch}'), ('auto_light_state', '@{ihc_function::auto_light_state}')], mode='vline')
hover_state_code = HoverTool(renderers=[f_state_code], tooltips=[('time', '@time'), ('ihc_enable_code', '@{ihc_function::ihc_enable_code}'),
                                                       ('ihc_fault_code', '@{ihc_function::ihc_fault_code}')], mode='vline')
hover_low_beam_reason = HoverTool(renderers=[f_low_beam_reason], tooltips=[('time', '@time'), ('same_direction_vehicle', '@{ihc_function::low_beam_due_to_same_dir_vehicle}'),
                                                       ('oncoming_motor_vehicle', '@{ihc_function::low_beam_due_to_oncomming_vehicle}'), ('oncoming_nonmotor_vehicle', '@{ihc_function::low_beam_due_to_oncomming_cycle}'), 
                                                       ('lighting_condition', '@{ihc_function::lighting_condition}')], mode='vline')


fig_machine.add_tools(hover_machine)
fig_state_code.add_tools(hover_state_code)
fig_low_beam_reason.add_tools(hover_low_beam_reason)


fig_machine.toolbar.active_scroll = fig_machine.select_one(WheelZoomTool)
fig_state_code.toolbar.active_scroll = fig_state_code.select_one(WheelZoomTool)
fig_low_beam_reason.toolbar.active_scroll = fig_low_beam_reason.select_one(WheelZoomTool)

fig_machine.legend.click_policy = 'hide'
fig_state_code.legend.click_policy = 'hide'
fig_low_beam_reason.legend.click_policy = 'hide'

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=0.02, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)

### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)

  t0 = 0
  time_vec = []
  steer_mpc_vec = []
  dphi_deg_ref_mpc_vec = []
  dphi_deg_mpc_vec = []


  data_mpc.data.update({
    'time_vec': time_vec,
    # 'dx_ref_mpc_vec':dx_ref_mpc_vec,
    # 'dy_ref_mpc_vec':dy_ref_mpc_vec,
    # 'dx_mpc_vec':dx_mpc_vec,
    # 'dy_mpc_vec':dy_mpc_vec,
    # 'dphi_ref_mpc_vec':dphi_deg_ref_mpc_vec,
    # 'dphi_mpc_vec':dphi_deg_mpc_vec,
    # 'delta_mpc_vec': steer_mpc_vec,
  })

  push_notebook()

bkp.show(row(fig1, column(fig_machine, fig_state_code, fig_low_beam_reason)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
