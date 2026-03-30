import sys, os
sys.path.append("..")
sys.path.append("../lib/")
# from lib.load_cyberbag import *
from lib.load_local_view_meb import *
from lib.load_ros_bag import LoadRosbag
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
#bag_path = "/home/xlwang71/Downloads/0721/long_tme_9.00000"
bag_path = "/mnt/data/data_collection_BESTUNE_E541_80716_EVENT_MANUAL_2026-01-13-13-46-13/data_collection_BESTUNE_E541_80716_EVENT_MANUAL_2026-01-13-13-46-13.bag.1772631544.open-loop.scc.plan"
frame_dt = 0.02
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


meb_od_obs_data = ColumnDataSource(data ={
  'meb_od_obs_x_vector': [],
  'meb_od_obs_y_vector':[],
})

meb_occ_obs_data = ColumnDataSource(data ={
  'meb_occ_obs_x_vector': [],
  'meb_occ_obs_y_vector':[]
})

meb_uss_obs_data = ColumnDataSource(data ={
  'meb_uss_obs_x_vector': [],
  'meb_uss_obs_y_vector':[]
})

meb_traj_data = ColumnDataSource(data ={
  'meb_traj_x_vector': [],
  'meb_traj_y_vector':[]
})

meb_point_data = ColumnDataSource(data ={
  'meb_point_x_vector': [],
  'meb_point_y_vector': [],
})

meb_uss_data =ColumnDataSource(data ={
  'uss_distance_vec': [],
  'uss_start_angle': [],
  'uss_end_angle': [],
  'uss_pos_x': [],
  'uss_pos_y': [],
})

#f
adas_json_data = bag_loader.plan_debug_msg['json']
adas_json_data_t = bag_loader.plan_debug_msg['t']
counter = 0

# +
lka_json_value_list = [
                         #meb debug info:
                         "meb_first_state","meb_second_state","meb_state", "meb_main_switch", "meb_enable_code", "meb_disable_code","meb_fault_code",
                         "meb_intervention_flag","meb_intervention_ttc","meb_od_obs_collision_flag",
                         "meb_occ_obs_collision_flag","meb_kickdown_code","meb_all_obs_point_num","meb_obs_distance","meb_od_obs_distance",
                         "meb_occ_obs_distance","meb_radius","meb_fusion_occ_obs_size","meb_fusion_od_obs_size","meb_relative_distance_min",
                         "meb_occ_path_distance","meb_uss_obs_distance","meb_uss_collision_flag","meb_od_acc_min","meb_occ_acc_min","meb_uss_acc_min",
                         "meb_od_ttc_min","meb_occ_ttc_min","meb_uss_ttc_min",
                         "meb_fusion_uss_obs_size","meb_od_box_collision_flag","meb_od_box_id",
                         "meb_supp_code","meb_input_.ego_radius","meb_cooling_time_remain","meb_hold_duration","meb_no_response_duration","meb_brake_duration",
                         "meb_request_status","meb_request_value","meb_request_direction",
                         "meb_od_straight_scene_code_","meb_occ_straight_scene_code_","meb_uss_straight_scene_code_","meb_od_crossing_scene_code",
                        ]
adas_json_value_list =  [ #adas_debug info
                         "params_dt","params_ego_length","params_ego_width", "params_origin_2_front_bumper", "params_origin_2_rear_bumper", "params_steer_ratio","params_wheel_base",
                         "params_ldp_c0_right_offset", "params_ldp_center_line_offset","params_ldp_ttlc_right_hack","params_ldp_tlc_thrd","params_ldw_enable_speed",
                         "state_left_turn_light_off_time","state_right_turn_light_off_time","state_driver_hand_trq","state_ego_curvature","state_fl_wheel_distance_to_line",
                         "state_fr_wheel_distance_to_line","state_vehicle_speed", "state_yaw_rate","state_left_departure_speed","state_right_departure_speed","state_steer_wheel_angle_degree",
                         "state_yaw_rate_observer","state_brake_pedal_pressed","state_vel_acc","state_shift_lever",
                         "road_left_line_boundary_type", "road_left_line_line_type","road_left_line_begin","road_left_line_end","road_left_line_c0","road_left_line_c1","road_left_line_c2","road_left_line_c3","state_fl_wheel_distance_to_roadedge",
                         "road_right_line_boundary_type","road_right_line_line_type","road_right_line_begin", "road_right_line_end","road_right_line_c0","road_right_line_c1","road_right_line_c2","road_right_line_c3","state_fr_wheel_distance_to_roadedge",
                         "road_left_line_valid","road_right_line_valid","road_left_roadedge_valid","road_right_roadedge_valid","road_lane_width_valid","road_lane_width",
                         "road_left_roadedge_begin_x","road_left_roadedge_end_x","road_right_roadedge_begin_x","road_right_roadedge_end_x",
                         #new
                         "state_accelerator_pedal_pos", "state_brake_pedal_pos","meb_od_obs_stop_distance_buffer_vector",
                         ]

json_vector_list = ["meb_all_obs_x_vector","meb_all_obs_y_vector",]
adas_json_value_list = lka_json_value_list + adas_json_value_list
adas_json_list_dict = {}
adas_t_debug = []
for i in range(len(adas_json_value_list)):
   new_list = []
   adas_json_list_dict[adas_json_value_list[i]] = new_list
for i in range(len(adas_json_data)):
  adas_t_debug.append(adas_json_data_t[i])
  for j in range(len(adas_json_value_list)):
    key = adas_json_value_list[j]
    # 使用 .get(key, default_value) 安全获取，默认值为 0
    raw_val = adas_json_data[i].get(key, -0.01)
    
    if key == "state_steer_wheel_angle_degree":
       value = raw_val / 57.3
    elif key == "state_vel_acc" and raw_val > 0.0:
       value = -1.0 * raw_val
    else:
       value = raw_val       
    adas_json_list_dict[key].append(value)
adas_json_list_dict['time'] = adas_t_debug


# figures
# fig1.line('dy_ref_mpc_vec', 'dx_ref_mpc_vec', source = data_mpc, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref', visible=True)
# fig1.line('dy_mpc_vec', 'dx_mpc_vec', source = data_mpc, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.35, legend_label = 'mpc', visible=True)
fig1.line('meb_traj_y_vector', 'meb_traj_x_vector', source = meb_traj_data, line_width = 5, line_color = 'green', line_dash = 'dashed', line_alpha = 0.35, legend_label = 'meb_traj', visible=True)

# fig1.circle('meb_all_obs_y_vector', 'meb_all_obs_x_vector', source=meb_obs_data, size=5, color='purple',radius = 0.1,
#             legend_label='all points')

fig1.circle('meb_od_obs_y_vector', 'meb_od_obs_x_vector', source=meb_od_obs_data, size=5, color='black',radius = 0.025,
            legend_label='od points')
fig1.circle('meb_occ_obs_y_vector', 'meb_occ_obs_x_vector', source=meb_occ_obs_data, size=5, color='green',radius = 0.025,
            legend_label='occ points')
fig1.circle('meb_uss_obs_y_vector', 'meb_uss_obs_x_vector', source=meb_uss_obs_data, size=5, color='red',radius = 0.025,
            legend_label='uss points')
fig1.circle('meb_point_y_vector', 'meb_point_x_vector', source=meb_point_data, size=5, color='purple',radius = 0.05,
            legend_label='predict_point')
fig1.wedge('uss_pos_y','uss_pos_x', 'uss_distance_vec', 'uss_start_angle', 'uss_end_angle',source = meb_uss_data, fill_color="lavender",
           line_color="black",legend_label = 'uss_wave',alpha = 0.5)
fig1.width = 1000
fig1.height = 1100

source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=6, source=source, color='red', legend_label='measure tool')
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool')
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool')
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
callback = CustomJS(args=dict(source=source, line_source=line_source, text_source=text_source), code=callback_code)
fig1.js_on_event(Tap, callback)
#f
# fig_control_1 = bkp.figure(x_axis_label='time', y_axis_label='steering angle',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=200)

fig_machine = bkp.figure(x_axis_label='time', y_axis_label='meb state_machine',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=300)
fig_machine.yaxis.axis_label_text_font_style = 'bold'

fig_vehicle_state = bkp.figure(x_axis_label='time', y_axis_label='vehicle state',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=300)
fig_vehicle_state.yaxis.axis_label_text_font_style = 'bold'

fig_dynamic_state = bkp.figure(x_axis_label='time', y_axis_label='dynamic state',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=250)
fig_dynamic_state.yaxis.axis_label_text_font_style = 'bold'

fig_result = bkp.figure(x_axis_label='time', y_axis_label='result',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=300)
fig_result.yaxis.axis_label_text_font_style = 'bold'

fig_result_od = bkp.figure(x_axis_label='time', y_axis_label='result',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=300)
fig_result_od.yaxis.axis_label_text_font_style = 'bold'
fig_result_occ = bkp.figure(x_axis_label='time', y_axis_label='result',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=300)
fig_result_occ.yaxis.axis_label_text_font_style = 'bold'
fig_result_uss = bkp.figure(x_axis_label='time', y_axis_label='result',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=300)
fig_result_uss.yaxis.axis_label_text_font_style = 'bold'

fig_obs_manager = bkp.figure(x_axis_label='time', y_axis_label='result',x_range = [adas_t_debug[0], adas_t_debug[-1]], width=700, height=300)
fig_obs_manager.yaxis.axis_label_text_font_style = 'bold'



f_machine = fig_machine.line('time', 'meb_state', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'meb_state')
fig_machine.line('time', 'meb_first_state', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'meb_first_state')
fig_machine.line('time', 'meb_second_state', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'meb_second_state')
fig_machine.line('time', 'meb_main_switch', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'meb_main_switch')
fig_machine.line('time', 'meb_enable_code', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'meb_enable_code')
fig_machine.line('time', 'meb_disable_code', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'meb_disable_code')
fig_machine.line('time', 'meb_kickdown_code', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'meb_kickdown_code')
fig_machine.line('time', 'meb_fault_code', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'meb_fault_code')
fig_machine.line('time', 'meb_intervention_flag', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'meb_intervention_flag')
fig_machine.line('time', 'meb_supp_code', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'meb_supp_code')

f_vehicle_state = fig_vehicle_state.line('time', 'state_vehicle_speed', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'vehicle_speed')
fig_vehicle_state.line('time', 'state_yaw_rate', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'yaw_rate')
fig_vehicle_state.line('time', 'state_yaw_rate_observer', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'yaw_rate_observer')
fig_vehicle_state.line('time', 'state_driver_hand_trq', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'driver_hand_trq')
# fig_vehicle_state.line('time', 'state_ego_curvature', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_curvature')
fig_vehicle_state.line('time', 'state_steer_wheel_angle_degree', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer_angle_rad')
fig_vehicle_state.line('time', 'state_brake_pedal_pressed', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'brake_pedal_pressed')
fig_vehicle_state.line('time', 'state_accelerator_pedal_pos', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'accelerator_pedal_pos')
fig_vehicle_state.line('time', 'state_brake_pedal_pos', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'brake_pedal_pos')
fig_vehicle_state.line('time', 'state_shift_lever', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'dashed', legend_label = 'shift_lever')

f_dynamic_state = fig_dynamic_state.line('time', 'meb_input_.ego_radius', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'meb_input_.ego_radius')
fig_dynamic_state.line('time', 'state_ego_curvature', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'state_ego_curvature')
fig_dynamic_state.line('time', 'meb_od_box_id', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'od_box_id')
# fig_dynamic_state.line('time', 'state_right_turn_light_off_time', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'right_turn_light_off_time')

f_result = fig_result.line('time', 'meb_intervention_flag', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'meb_intervention_flag')
# fig_result.line('time', 'meb_obs_distance', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'obs_distance')
# fig_result.line('time', 'meb_intervention_ttc', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'intervention_ttc')
# fig_result.line('time', 'meb_relative_distance_min', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'relative_distance_min')
fig_result.line('time', 'meb_all_obs_point_num', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'all_obs_point_num')
fig_result.line('time', 'meb_fusion_occ_obs_size', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'fusion_occ_obs_size')
fig_result.line('time', 'meb_fusion_od_obs_size', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'fusion_od_obs_size')
fig_result.line('time', 'meb_fusion_uss_obs_size', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'fusion_uss_obs_size')

f_result_od = fig_result_od.line('time', 'meb_intervention_flag', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'meb_intervention_flag')
fig_result_od.line('time', 'meb_od_obs_collision_flag', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'dashed', legend_label = 'od_obs_collision_flag')
fig_result_od.line('time', 'meb_od_straight_scene_code_', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'meb_od_straight_scene_code_',visible = False)
fig_result_od.line('time', 'meb_od_crossing_scene_code', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'meb_od_crossing_scene_code',visible = False)
fig_result_od.line('time', 'meb_od_ttc_min', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'od_ttc_min')
fig_result_od.line('time', 'meb_od_box_collision_flag', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'box_collision_flag')


f_result_occ = fig_result_occ.line('time', 'meb_intervention_flag', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'meb_intervention_flag')
fig_result_occ.line('time', 'meb_occ_obs_collision_flag', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'dashed', legend_label = 'occ_obs_collision_flag')
fig_result_occ.line('time', 'meb_occ_obs_distance', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'occ_obs_distance')
fig_result_occ.line('time', 'meb_occ_acc_min', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'occ_acc_min')
fig_result_occ.line('time', 'meb_occ_ttc_min', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'occ_ttc_min')

f_result_uss = fig_result_uss.line('time', 'meb_intervention_flag', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'meb_intervention_flag')
fig_result_uss.line('time', 'meb_uss_collision_flag', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'dashed', legend_label = 'uss_collision_flag')
fig_result_uss.line('time', 'meb_uss_obs_distance', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'uss_obs_distance')
fig_result_uss.line('time', 'meb_uss_acc_min', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'uss_acc_min')
fig_result_uss.line('time', 'meb_uss_ttc_min', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'uss_ttc_min')

f_obs_manager = fig_obs_manager.line('time', 'meb_request_status', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'meb_request_status')
fig_obs_manager.line('time', 'meb_request_value', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'meb_request_value')
fig_obs_manager.line('time', 'meb_request_direction', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'meb_request_direction')
fig_obs_manager.line('time', 'state_vel_acc', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'vel_acc')



hover_machine = HoverTool(renderers=[f_machine], tooltips=[('time', '@time'), ('ldw_state_', '@ldw_state_'),('ldw_main_switch_', '@ldw_main_switch_'), ('ldw_enable_code_', '@ldw_enable_code_'),
                                                       ('ldw_disable_code_', '@ldw_disable_code_'), ('ldw_fault_code_', '@ldw_fault_code_')], mode='vline')
hover_vehicle_state = HoverTool(renderers=[f_vehicle_state], tooltips=[('time', '@time'), ('state_vehicle_speed', '@state_vehicle_speed'), ('state_yaw_rate', '@state_yaw_rate'), ('state_yaw_rate_observer', '@state_yaw_rate_observer'),
                                               ('state_driver_hand_trq', '@state_driver_hand_trq'), ('state_ego_curvature', '@state_ego_curvature'), ('state_steer_wheel_angle_degree', '@state_steer_wheel_angle_degree')], mode='vline')
hover_dynamic_state = HoverTool(renderers=[f_dynamic_state], tooltips=[('time', '@time'), ('state_left_departure_speed', '@state_left_departure_speed'), ('state_right_departure_speed', '@state_right_departure_speed'), ('state_left_turn_light_off_time', '@state_left_turn_light_off_time'),
                                               ('state_right_turn_light_off_time', '@state_right_turn_light_off_time')], mode='vline')
hover_f_result = HoverTool(renderers=[f_result], tooltips=[('time', '@time'), ('ldw_state_', '@ldw_state_'),('ldw_left_suppression_code_', '@ldw_left_suppression_code_'), ('ldw_left_kickdown_code_', '@ldw_left_kickdown_code_'),
                                                       ('ldw_left_intervention_', '@ldw_left_intervention_')], mode='vline')
hover_f_result_od = HoverTool(renderers=[f_result_od], tooltips=[('time', '@time'), ('ldw_state_', '@ldw_state_'),('ldw_left_suppression_code_', '@ldw_left_suppression_code_'), ('ldw_left_kickdown_code_', '@ldw_left_kickdown_code_'),
                                                       ('ldw_left_intervention_', '@ldw_left_intervention_')], mode='vline')
hover_f_result_occ = HoverTool(renderers=[f_result_occ], tooltips=[('time', '@time'), ('ldw_state_', '@ldw_state_'),('ldw_left_suppression_code_', '@ldw_left_suppression_code_'), ('ldw_left_kickdown_code_', '@ldw_left_kickdown_code_'),
                                                       ('ldw_left_intervention_', '@ldw_left_intervention_')], mode='vline')
hover_f_result_uss = HoverTool(renderers=[f_result_uss], tooltips=[('time', '@time'), ('ldw_state_', '@ldw_state_'),('ldw_left_suppression_code_', '@ldw_left_suppression_code_'), ('ldw_left_kickdown_code_', '@ldw_left_kickdown_code_'),
                                                       ('ldw_left_intervention_', '@ldw_left_intervention_')], mode='vline')
hover_fig_obs_manager = HoverTool(renderers=[f_result], tooltips=[('time', '@time'), ('ldw_state_', '@ldw_state_'),('ldw_left_suppression_code_', '@ldw_left_suppression_code_'), ('ldw_left_kickdown_code_', '@ldw_left_kickdown_code_'),
                                                       ('ldw_left_intervention_', '@ldw_left_intervention_')], mode='vline')

fig_machine.add_tools(hover_machine)
fig_vehicle_state.add_tools(hover_vehicle_state)
fig_dynamic_state.add_tools(hover_dynamic_state)
fig_result.add_tools(hover_f_result)
fig_result_od.add_tools(hover_f_result_od)
fig_result_occ.add_tools(hover_f_result_occ)
fig_result_uss.add_tools(hover_f_result_uss)
fig_obs_manager.add_tools(hover_fig_obs_manager)

fig_machine.toolbar.active_scroll = fig_machine.select_one(WheelZoomTool)
fig_vehicle_state.toolbar.active_scroll = fig_vehicle_state.select_one(WheelZoomTool)
fig_dynamic_state.toolbar.active_scroll = fig_dynamic_state.select_one(WheelZoomTool)
fig_result.toolbar.active_scroll = fig_result.select_one(WheelZoomTool)
fig_result_od.toolbar.active_scroll = fig_result_od.select_one(WheelZoomTool)
fig_result_occ.toolbar.active_scroll = fig_result_occ.select_one(WheelZoomTool)
fig_result_uss.toolbar.active_scroll = fig_result_uss.select_one(WheelZoomTool)
fig_obs_manager.toolbar.active_scroll = fig_result.select_one(WheelZoomTool)


fig_machine.legend.click_policy = 'hide'
fig_vehicle_state.legend.click_policy = 'hide'
fig_dynamic_state.legend.click_policy = 'hide'
fig_result.legend.click_policy = 'hide'
fig_result_od.legend.click_policy = 'hide'
fig_result_occ.legend.click_policy = 'hide'
fig_result_uss.legend.click_policy = 'hide'
fig_obs_manager.legend.click_policy = 'hide'

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=0.02, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)

### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)




  uss_start_angle = [120.0, 90.0, 60.0, 60.0, 30.0, 0.0,
                     -30.0, -90.0, -120.0, -120.0, -150.0, -180.0]
  uss_end_angle = [180.0, 150.0, 120.0, 120.0, 90.0, 60.0,
                   0.0,-30.0, -60.0,-60.0, -90.0,-120.0]
  for i in range(len(uss_start_angle)):
    uss_start_angle[i] = uss_start_angle[i] /57.3
    uss_end_angle[i] = uss_end_angle[i] /57.3
  uss_pos_x = [3.245,  3.58,   3.725, 3.725,
                                            3.58,   3.245,  -0.5,  -0.96,
                                            -1.064, -1.064, -0.96, -0.5]
  uss_pos_y = [0.941,  0.694,  0.331,  -0.331,
                                            -0.694, -0.941, -0.912, -0.731,
                                            -0.328, 0.328,  0.731,  0.912]
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

  planning_json = local_view_data['data_msg']['plan_debug_json_msg']
  meb_od_obs_x_vector = planning_json['meb_od_obs_x_vector']
  meb_od_obs_y_vector = planning_json['meb_od_obs_y_vector']
  meb_od_obs_stop_distance_buffer_vector = planning_json['meb_od_obs_stop_distance_buffer_vector']
  meb_occ_obs_x_vector = planning_json['meb_occ_obs_x_vector']
  meb_occ_obs_y_vector = planning_json['meb_occ_obs_y_vector']
  meb_uss_obs_x_vector = planning_json['meb_uss_obs_x_vector']
  meb_uss_obs_y_vector = planning_json['meb_uss_obs_y_vector']
  meb_traj_x_vector = planning_json['meb_traj_x_vector']
  meb_traj_y_vector = planning_json['meb_traj_y_vector']

  meb_point_x_vector = planning_json['meb_point_x_vector']
  meb_point_y_vector = planning_json['meb_point_y_vector']
  uss_distance_vec = planning_json['uss_distance_vec']
  state_vehicle_speed = planning_json['state_vehicle_speed']
  print("uss_distance_vec",uss_distance_vec)
  for i in range(len(uss_distance_vec)):
    uss_distance_vec[i] = uss_distance_vec[i] * 0.5
#     if uss_distance_vec[i] > 1.0:
#       uss_distance_vec[i] = 1.0
#     if uss_distance_vec[i] < 0.1:
#       uss_distance_vec[i] = 0.1
  uss_acc_vec_ = planning_json['uss_acc_vec_']



  print("meb_cooling_time_remain",planning_json['meb_cooling_time_remain'])
  print("meb_hold_duration",planning_json['meb_hold_duration'])
  print("meb_no_response_duration",planning_json['meb_no_response_duration'])
  print("meb_brake_duration",planning_json['meb_brake_duration'])


  print("meb_od_obs_x_vector",meb_od_obs_x_vector)
  print("meb_od_obs_y_vector",meb_od_obs_y_vector)
  print("meb_od_obs_stop_distance_buffer_vector",meb_od_obs_stop_distance_buffer_vector)

#   print("meb_occ_obs_x_vector",meb_occ_obs_x_vector)
#   print("meb_occ_obs_y_vector",meb_occ_obs_y_vector)
  # print("meb_traj_x_vector",meb_traj_x_vector)
  # print("meb_traj_y_vector",meb_traj_y_vector)
#   print("meb_point_x_vector",meb_point_x_vector)
#   print("meb_point_y_vector",meb_point_y_vector)


  meb_od_obs_data.data.update({
   'meb_od_obs_x_vector': meb_od_obs_x_vector,
   'meb_od_obs_y_vector': meb_od_obs_y_vector,
  })

  meb_occ_obs_data.data.update({
   'meb_occ_obs_x_vector': meb_occ_obs_x_vector,
   'meb_occ_obs_y_vector': meb_occ_obs_y_vector,
  })

  meb_uss_obs_data.data.update({
   'meb_uss_obs_x_vector': meb_uss_obs_x_vector,
   'meb_uss_obs_y_vector': meb_uss_obs_y_vector,
  })

  meb_traj_data.data.update({
   'meb_traj_x_vector': meb_traj_x_vector,
   'meb_traj_y_vector': meb_traj_y_vector,
  })

  meb_point_data.data.update({
  'meb_point_x_vector': meb_point_x_vector,
  'meb_point_y_vector': meb_point_y_vector,
  })

  meb_uss_data.data.update({
  'uss_distance_vec': uss_distance_vec,
  'uss_start_angle': uss_start_angle,
  'uss_end_angle': uss_end_angle,
  'uss_pos_x': uss_pos_x,
  'uss_pos_y': uss_pos_y,
})

  push_notebook()

bkp.show(row(fig1, column(fig_obs_manager,fig_result_od,fig_result_occ,fig_result_uss),column(fig_machine, fig_vehicle_state, fig_dynamic_state,fig_result)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
