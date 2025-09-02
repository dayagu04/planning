import sys, os
sys.path.append("..")
sys.path.append("../lib/")
# from lib.load_cyberbag import *
from lib.load_local_view import *
from lib.load_ros_bag import LoadRosbag
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
#bag_path = "/home/xlwang71/Downloads/0721/long_tme_9.00000"
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_40737/trigger/20250804/20250804-16-00-21/data_collection_CHERY_M32T_40737_EVENT_MANUAL_2025-08-04-16-00-21_no_camera.bag"
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
lka_json_value_list = [
                         #new_ldw debug info:
                         "ldw_main_switch_","ldw_enable_code_", "ldw_disable_code_", "ldw_fault_code_", "ldw_left_suppression_code_","ldw_left_kickdown_code_",
                         "ldw_right_suppression_code_","ldw_right_kickdown_code_","ldw_tlc_threshold_","ldw_left_intervention_","ldw_right_intervention_","ldw_state_",
                         "ldw_preview_left_y_gap","ldw_preview_right_y_gap",
                         #new_ldp debug info:
                         "ldp_main_switch_","ldp_enable_code_", "ldp_disable_code_", "ldp_fault_code_", "ldp_left_suppression_code_","ldp_left_kickdown_code_",
                         "ldp_right_suppression_code_","ldp_right_kickdown_code_","ldp_tlc_threshold_","ldp_left_intervention_","ldp_right_intervention_","ldp_state_",
                         "ldp_preview_left_y_gap","ldp_preview_right_y_gap","ldp_left_intervention_by_line","ldp_left_intervention_by_roadedge",
                         "ldp_right_intervention_by_line","ldp_right_intervention_by_roadedge","ldp_roadedge_tlc_threshold_",
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
adas_json_value_list = lka_json_value_list + adas_json_value_list
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

# adas_json_list_dict = {}
# adas_t_debug = []
# for i in range(len(adas_json_value_list)):
#    new_list = []
#    adas_json_list_dict[adas_json_value_list[i]] = new_list
# for i in range(len(ldp_json_data)):
#   adas_t_debug.append(ldp_json_data_t[i])
#   for j in range(len(adas_json_value_list)):
#      value = ldp_json_data[i][adas_json_value_list[j]]
#      adas_json_list_dict[adas_json_value_list[j]].append(value)
# adas_json_list_dict['time'] = adas_t_debug


# figures
fig1.line('dy_ref_mpc_vec', 'dx_ref_mpc_vec', source = data_mpc, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref', visible=True)
fig1.line('dy_mpc_vec', 'dx_mpc_vec', source = data_mpc, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.35, legend_label = 'mpc', visible=True)
fig1.width = 600
fig1.height = 1000

#f
# fig_control_1 = bkp.figure(x_axis_label='time', y_axis_label='steering angle',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=200)

fig_machine = bkp.figure(x_axis_label='time', y_axis_label='ldw state_machine',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=200)
fig_machine.yaxis.axis_label_text_font_style = 'bold'

fig_vehicle_state = bkp.figure(x_axis_label='time', y_axis_label='vehicle state',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=200)
fig_vehicle_state.yaxis.axis_label_text_font_style = 'bold'

fig_dynamic_state = bkp.figure(x_axis_label='time', y_axis_label='dynamic state',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=200)
fig_dynamic_state.yaxis.axis_label_text_font_style = 'bold'

fig_left_machine = bkp.figure(x_axis_label='time', y_axis_label='ldw left machine',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=200)
fig_left_machine.yaxis.axis_label_text_font_style = 'bold'

fig_right_machine = bkp.figure(x_axis_label='time', y_axis_label='ldw right machine',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=200)
fig_right_machine.yaxis.axis_label_text_font_style = 'bold'

fig_trig_caculate = bkp.figure(x_axis_label='time', y_axis_label='line trig caculate',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=280)
fig_trig_caculate.yaxis.axis_label_text_font_style = 'bold'

fig_left_line_info = bkp.figure(x_axis_label='time', y_axis_label='left line info',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=220)
fig_left_line_info.yaxis.axis_label_text_font_style = 'bold'

fig_right_line_info = bkp.figure(x_axis_label='time', y_axis_label='right line info',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=220)
fig_right_line_info.yaxis.axis_label_text_font_style = 'bold'

fig_relative_line_info = bkp.figure(x_axis_label='time', y_axis_label='relative line info',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=200)
fig_relative_line_info.yaxis.axis_label_text_font_style = 'bold'

fig_left_line_base_info = bkp.figure(x_axis_label='time', y_axis_label='left line base info',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=280)
fig_left_line_base_info.yaxis.axis_label_text_font_style = 'bold'

fig_right_line_base_info = bkp.figure(x_axis_label='time', y_axis_label='right line base info',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=280)
fig_right_line_base_info.yaxis.axis_label_text_font_style = 'bold'

f_machine = fig_machine.line('time', 'ldw_state_', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'ldw_state')
fig_machine.line('time', 'ldw_main_switch_', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'ldw_main_switch')
fig_machine.line('time', 'ldw_enable_code_', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'ldw_enable_code')
fig_machine.line('time', 'ldw_disable_code_', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'ldw_disable_code')
fig_machine.line('time', 'ldw_fault_code_', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'ldw_fault_code')

f_vehicle_state = fig_vehicle_state.line('time', 'state_vehicle_speed', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'vehicle_speed')
fig_vehicle_state.line('time', 'state_yaw_rate', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'yaw_rate')
fig_vehicle_state.line('time', 'state_yaw_rate_observer', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'yaw_rate_observer')
fig_vehicle_state.line('time', 'state_driver_hand_trq', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'driver_hand_trq')
fig_vehicle_state.line('time', 'state_ego_curvature', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_curvature')
fig_vehicle_state.line('time', 'state_steer_wheel_angle_degree', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer_wheel_angle_degree')


f_dynamic_state = fig_dynamic_state.line('time', 'state_left_departure_speed', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'left_departure_speed')
fig_dynamic_state.line('time', 'state_right_departure_speed', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'right_departure_speed')
fig_dynamic_state.line('time', 'state_left_turn_light_off_time', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'left_turn_light_off_time')
fig_dynamic_state.line('time', 'state_right_turn_light_off_time', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'right_turn_light_off_time')

f_left_machine = fig_left_machine.line('time', 'ldw_state_', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'ldw_state')
fig_left_machine.line('time', 'ldw_left_suppression_code_', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'ldw_left_suppression_code')
fig_left_machine.line('time', 'ldw_left_kickdown_code_', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'ldw_left_kickdown_code')
fig_left_machine.line('time', 'ldw_left_intervention_', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ldw_left_intervention')
# fig_left_machine.line('time', 'lkas_function::ldw::left_intervention', source = lkas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'lkas_function::ldw::left_intervention')
# fig_left_machine.line('time', 'ldw_left_intervention_by_line', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'left_intervention_by_line')
# fig_left_machine.line('time', 'ldw_left_intervention_by_roadedge', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'left_intervention_by_roadedge')

f_right_machine = fig_right_machine.line('time', 'ldw_state_', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'ldw_state')
fig_right_machine.line('time', 'ldw_right_suppression_code_', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'ldw_right_suppression_code')
fig_right_machine.line('time', 'ldw_right_kickdown_code_', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'ldw_right_kickdown_code')
fig_right_machine.line('time', 'ldw_right_intervention_', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ldw_right_intervention')
# fig_right_machine.line('time', 'lkas_function::ldw::right_intervention', source = lkas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'lkas_function::ldw::right_intervention')
# fig_right_machine.line('time', 'ldw_right_intervention_by_line', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'right_intervention_by_line')
# fig_right_machine.line('time', 'ldw_right_intervention_by_roadedge', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'right_intervention_by_roadedge')

f_trig_caculate = fig_trig_caculate.line('time', 'ldw_state_', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'ldw_state')
f_trig_caculate = fig_trig_caculate.line('time', 'ldw_preview_left_y_gap', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'preview_left_y_gap')
f_trig_caculate = fig_trig_caculate.line('time', 'ldw_preview_right_y_gap', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'preview_right_y_gap')
f_trig_caculate = fig_trig_caculate.line('time', 'ldw_tlc_threshold_', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'tlc_threshold_')
# f_trig_caculate = fig_trig_caculate.line('time', 'ldw_roadedge_tlc_threshold_', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'roadedge_tlc_threshold_')

f_left_line_info = fig_left_line_info.line('time', 'road_left_line_valid', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'left_line_valid')
fig_left_line_info.line('time', 'road_left_roadedge_valid', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'left_roadedge_valid')
fig_left_line_info.line('time', 'road_left_line_line_type', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'left_line_line_type')
fig_left_line_info.line('time', 'road_left_line_boundary_type', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'left_line_boundary_type')
# fig_left_line_info.line('time', 'state_fl_wheel_distance_to_roadedge', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'left_roadedge_distance')

f_right_line_info = fig_right_line_info.line('time', 'road_right_line_valid', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'right_line_valid')
fig_right_line_info.line('time', 'road_right_roadedge_valid', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'right_roadedge_valid')
fig_right_line_info.line('time', 'road_right_line_line_type', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'right_line_line_type')
fig_right_line_info.line('time', 'road_right_line_boundary_type', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'right_line_boundary_type')
# fig_right_line_info.line('time', 'state_fr_wheel_distance_to_roadedge', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'right_roadedge_distance')

f_relative_line_info = fig_relative_line_info.line('time', 'state_fl_wheel_distance_to_line', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'fl_wheel_distance_to_line')
fig_relative_line_info.line('time', 'state_fl_wheel_distance_to_roadedge', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'left_roadedge_distance')
fig_relative_line_info.line('time', 'state_fr_wheel_distance_to_line', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'fr_wheel_distance_to_line')
fig_relative_line_info.line('time', 'state_fr_wheel_distance_to_roadedge', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'right_roadedge_distance')
fig_relative_line_info.line('time', 'road_lane_width_valid', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lane_width_valid')
fig_relative_line_info.line('time', 'road_lane_width', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'lane_width')

f_left_line_base_info = fig_left_line_base_info.line('time', 'road_left_line_begin', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'left_line_begin_x')
fig_left_line_base_info.line('time', 'road_left_line_end', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'left_line_end_x')
fig_left_line_base_info.line('time', 'road_left_roadedge_begin_x', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'left_roadedge_begin_x')
fig_left_line_base_info.line('time', 'road_left_roadedge_end_x', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'left_roadedge_end_x')
fig_left_line_base_info.line('time', 'road_left_line_c0', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'left_line_c0')
fig_left_line_base_info.line('time', 'road_left_line_c1', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'left_line_c1')
fig_left_line_base_info.line('time', 'road_left_line_c2', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'left_line_c2')
fig_left_line_base_info.line('time', 'road_left_line_c3', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'left_line_c3')
# fig_left_line_info.line('time', 'road_left_line_valid', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'left_line_valid')
# fig_left_line_info.line('time', 'road_left_roadedge_valid', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'left_roadedge_valid')
# fig_left_line_info.line('time', 'road_left_line_line_type', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'left_line_line_type')
# fig_left_line_info.line('time', 'road_left_line_boundary_type', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'left_line_boundary_type')
# fig_left_line_info.line('time', 'state_fl_wheel_distance_to_roadedge', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'left_roadedge_distance')

f_right_line_base_info = fig_right_line_base_info.line('time', 'road_right_line_begin', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'right_line_begin')
fig_right_line_base_info.line('time', 'road_right_line_end', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'right_line_end')
fig_right_line_base_info.line('time', 'road_right_roadedge_begin_x', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'right_roadedge_begin_x')
fig_right_line_base_info.line('time', 'road_right_roadedge_end_x', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'right_roadedge_end_x')
fig_right_line_base_info.line('time', 'road_right_line_c0', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'right_line_c0')
fig_right_line_base_info.line('time', 'road_right_line_c1', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'right_line_c1')
fig_right_line_base_info.line('time', 'road_right_line_c2', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'right_line_c2')
fig_right_line_base_info.line('time', 'road_right_line_c3', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'right_line_c3')
# fig_right_line_info.line('time', 'road_right_line_valid', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'right_line_valid')
# fig_right_line_info.line('time', 'road_right_roadedge_valid', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'right_roadedge_valid')
# fig_right_line_info.line('time', 'road_right_line_line_type', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'right_line_line_type')
# fig_right_line_info.line('time', 'road_right_line_boundary_type', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'right_line_boundary_type')
# fig_right_line_info.line('time', 'state_fr_wheel_distance_to_roadedge', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'right_roadedge_distance')


hover_machine = HoverTool(renderers=[f_machine], tooltips=[('time', '@time'), ('ldw_state_', '@ldw_state_'),('ldw_main_switch_', '@ldw_main_switch_'), ('ldw_enable_code_', '@ldw_enable_code_'),
                                                       ('ldw_disable_code_', '@ldw_disable_code_'), ('ldw_fault_code_', '@ldw_fault_code_')], mode='vline')
hover_vehicle_state = HoverTool(renderers=[f_vehicle_state], tooltips=[('time', '@time'), ('state_vehicle_speed', '@state_vehicle_speed'), ('state_yaw_rate', '@state_yaw_rate'), ('state_yaw_rate_observer', '@state_yaw_rate_observer'),
                                               ('state_driver_hand_trq', '@state_driver_hand_trq'), ('state_ego_curvature', '@state_ego_curvature'), ('state_steer_wheel_angle_degree', '@state_steer_wheel_angle_degree')], mode='vline')
hover_dynamic_state = HoverTool(renderers=[f_dynamic_state], tooltips=[('time', '@time'), ('state_left_departure_speed', '@state_left_departure_speed'), ('state_right_departure_speed', '@state_right_departure_speed'), ('state_left_turn_light_off_time', '@state_left_turn_light_off_time'),
                                               ('state_right_turn_light_off_time', '@state_right_turn_light_off_time')], mode='vline')
hover_left_machine = HoverTool(renderers=[f_left_machine], tooltips=[('time', '@time'), ('ldw_state_', '@ldw_state_'),('ldw_left_suppression_code_', '@ldw_left_suppression_code_'), ('ldw_left_kickdown_code_', '@ldw_left_kickdown_code_'),
                                                       ('ldw_left_intervention_', '@ldw_left_intervention_')], mode='vline')
hover_right_machine = HoverTool(renderers=[f_right_machine], tooltips=[('time', '@time'), ('ldw_state_', '@ldw_state_'), ('ldw_right_suppression_code_', '@ldw_right_suppression_code_'),
                                                       ('ldw_right_kickdown_code_', '@ldw_right_kickdown_code_'), ('ldw_right_intervention_', '@ldw_right_intervention_')], mode='vline')
hover_trig_caculate = HoverTool(renderers=[f_trig_caculate], tooltips=[('time', '@time'), ('ldw_state_', '@ldw_state_'), ('ldw_preview_left_y_gap', '@ldw_preview_left_y_gap'),
                                                         ('ldw_preview_right_y_gap', '@ldw_preview_right_y_gap'), ('ldw_tlc_threshold_', '@ldw_tlc_threshold_'),('ldw_roadedge_tlc_threshold_', '@ldw_roadedge_tlc_threshold_'),], mode='vline')
hover_left_line_info = HoverTool(renderers=[f_left_line_info], tooltips=[('time', '@time'), ('road_left_line_valid', '@road_left_line_valid'), ('road_left_roadedge_valid', '@road_left_roadedge_valid'),
                                                         ('road_left_line_line_type', '@road_left_line_line_type'), ('road_left_line_boundary_type', '@road_left_line_boundary_type'),], mode='vline')
hover_right_line_info = HoverTool(renderers=[f_right_line_info], tooltips=[('time', '@time'), ('road_right_line_valid', '@road_right_line_valid'), ('road_right_roadedge_valid', '@road_right_roadedge_valid'),
                                             ('road_right_line_line_type', '@road_right_line_line_type'), ('road_right_line_boundary_type', '@road_right_line_boundary_type'),], mode='vline')
hover_relative_line_info = HoverTool(renderers=[f_relative_line_info], tooltips=[('time', '@time'), ('state_fl_wheel_distance_to_line', '@state_fl_wheel_distance_to_line'), ('state_fl_wheel_distance_to_roadedge', '@state_fl_wheel_distance_to_roadedge'),
                                             ('state_fr_wheel_distance_to_line', '@state_fr_wheel_distance_to_line'), ('state_fr_wheel_distance_to_roadedge', '@state_fr_wheel_distance_to_roadedge'), ('road_lane_width_valid', '@road_lane_width_valid'),
                                             ('road_lane_width', '@road_lane_width')], mode='vline')
hover_left_line_base_info = HoverTool(renderers=[f_left_line_base_info], tooltips=[('time', '@time'), ('road_left_line_begin', '@road_left_line_begin'), ('road_left_line_end', '@road_left_line_end'),
                                                         ('road_left_roadedge_begin_x', '@road_left_roadedge_begin_x'), ('road_left_roadedge_end_x', '@road_left_roadedge_end_x'),('road_left_line_c0', '@road_left_line_c0'),
                                                         ('road_left_line_c1', '@road_left_line_c1'),('road_left_line_c2', '@road_left_line_c2'),('road_left_line_c3', '@road_left_line_c3'),], mode='vline')
hover_right_line_base_info = HoverTool(renderers=[f_right_line_base_info], tooltips=[('time', '@time'), ('road_right_line_begin', '@road_right_line_begin'), ('road_right_line_end', '@road_right_line_end'),
                                                         ('road_right_roadedge_begin_x', '@road_right_roadedge_begin_x'), ('road_right_roadedge_end_x', '@road_right_roadedge_end_x'),('road_right_line_c0', '@road_right_line_c0'),
                                                         ('road_right_line_c1', '@road_right_line_c1'),('road_right_line_c2', '@road_right_line_c2'),('road_right_line_c3', '@road_right_line_c3'),], mode='vline')


fig_machine.add_tools(hover_machine)
fig_vehicle_state.add_tools(hover_vehicle_state)
fig_dynamic_state.add_tools(hover_dynamic_state)
fig_left_machine.add_tools(hover_left_machine)
fig_right_machine.add_tools(hover_right_machine)
fig_trig_caculate.add_tools(hover_trig_caculate)
fig_left_line_info.add_tools(hover_left_line_info)
fig_right_line_info.add_tools(hover_right_line_info)
fig_relative_line_info.add_tools(hover_relative_line_info)
fig_left_line_base_info.add_tools(hover_left_line_base_info)
fig_right_line_base_info.add_tools(hover_right_line_base_info)


fig_machine.toolbar.active_scroll = fig_machine.select_one(WheelZoomTool)
fig_vehicle_state.toolbar.active_scroll = fig_vehicle_state.select_one(WheelZoomTool)
fig_dynamic_state.toolbar.active_scroll = fig_dynamic_state.select_one(WheelZoomTool)
fig_left_machine.toolbar.active_scroll = fig_left_machine.select_one(WheelZoomTool)
fig_right_machine.toolbar.active_scroll = fig_right_machine.select_one(WheelZoomTool)
fig_trig_caculate.toolbar.active_scroll = fig_trig_caculate.select_one(WheelZoomTool)
fig_left_line_info.toolbar.active_scroll = fig_left_line_info.select_one(WheelZoomTool)
fig_right_line_info.toolbar.active_scroll = fig_right_line_info.select_one(WheelZoomTool)
fig_relative_line_info.toolbar.active_scroll = fig_relative_line_info.select_one(WheelZoomTool)
fig_left_line_base_info.toolbar.active_scroll = fig_left_line_base_info.select_one(WheelZoomTool)
fig_right_line_base_info.toolbar.active_scroll = fig_right_line_base_info.select_one(WheelZoomTool)


fig_machine.legend.click_policy = 'hide'
fig_vehicle_state.legend.click_policy = 'hide'
fig_dynamic_state.legend.click_policy = 'hide'
fig_left_machine.legend.click_policy = 'hide'
fig_right_machine.legend.click_policy = 'hide'
fig_trig_caculate.legend.click_policy = 'hide'
fig_left_line_info.legend.click_policy = 'hide'
fig_right_line_info.legend.click_policy = 'hide'
fig_relative_line_info.legend.click_policy = 'hide'
fig_left_line_base_info.legend.click_policy = 'hide'
fig_right_line_base_info.legend.click_policy = 'hide'

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

bkp.show(row(fig1, column(fig_machine, fig_vehicle_state, fig_dynamic_state, fig_left_machine, fig_right_machine,fig_trig_caculate),
             column( fig_left_line_info, fig_right_line_info,fig_relative_line_info, fig_left_line_base_info, fig_right_line_base_info)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
