import sys, os
sys.path.append("..")
sys.path.append("../lib/")
# from lib.load_cyberbag import *
from lib.load_local_view import *
from bokeh.events import Tap
from lib.load_ros_bag import LoadRosbag
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
#bag_path = "/home/xlwang71/Downloads/0721/long_tme_9.00000"
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250415/20250415-12-55-15/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-04-15-12-55-15_no_camera.bag.1744879087.open-loop.scc.plan"

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

# bag_loader = LoadCyberbag(bag_path)
bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()
frame_dt = 0.02

data_vector = ColumnDataSource(data ={
  # 'time_vec': [],
  # 'dx_ref_mpc_vec':[],
  # 'dy_ref_mpc_vec':[],
  # 'dx_mpc_vec':[],
  # 'dy_mpc_vec':[],
  # 'dphi_ref_mpc_vec':[],
  # 'dphi_mpc_vec':[],
  # 'delta_mpc_vec': [],
  'road_left_roadedge_all_dx_vec_':[],
  'road_left_roadedge_all_dy_vec_': [],
  'road_right_roadedge_all_dx_vec_': [],
  'road_right_roadedge_all_dy_vec_': [],
})
data_car_preview = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_fl_obj_selected = ColumnDataSource(data = {'obj_yn':[], 'obj_xn':[]})
data_fm_obj_selected = ColumnDataSource(data = {'obj_yn':[], 'obj_xn':[]})
data_fr_obj_selected = ColumnDataSource(data = {'obj_yn':[], 'obj_xn':[]})
data_ml_obj_selected = ColumnDataSource(data = {'obj_yn':[], 'obj_xn':[]})
data_mr_obj_selected = ColumnDataSource(data = {'obj_yn':[], 'obj_xn':[]})
data_rl_obj_selected = ColumnDataSource(data = {'obj_yn':[], 'obj_xn':[]})
data_rm_obj_selected = ColumnDataSource(data = {'obj_yn':[], 'obj_xn':[]})
data_rr_obj_selected = ColumnDataSource(data = {'obj_yn':[], 'obj_xn':[]})



ctrl_json_data = bag_loader.ctrl_debug_msg['json']
time_all = bag_loader.ctrl_msg['t'][-1]

# dt_json = time_all / len(ctrl_json_data)

#f
elk_json_data = bag_loader.plan_debug_msg['json']
elk_json_data_t = bag_loader.plan_debug_msg['t']
counter = 0

# +
lka_json_value_list = [#new_ldw debug info:
                         "ldw_main_switch_","ldw_enable_code_", "ldw_disable_code_", "ldw_fault_code_", "ldw_left_suppression_code_","ldw_left_kickdown_code_",
                         "ldw_right_suppression_code_","ldw_right_kickdown_code_","ldw_tlc_threshold_","ldw_left_intervention_","ldw_right_intervention_","ldw_state_",
                         "ldw_preview_left_y_gap","ldw_preview_right_y_gap",
                         #new_ldp debug info:
                         "ldp_main_switch_","ldp_enable_code_", "ldp_disable_code_", "ldp_fault_code_", "ldp_left_suppression_code_","ldp_left_kickdown_code_",
                         "ldp_right_suppression_code_","ldp_right_kickdown_code_","ldp_tlc_threshold_","ldp_roadedge_tlc_threshold_","ldp_left_intervention_","ldp_right_intervention_","ldp_state_",
                         "ldp_preview_left_y_gap","ldp_preview_right_y_gap","ldp_left_intervention_by_line","ldp_left_intervention_by_roadedge",
                         "ldp_right_intervention_by_line","ldp_right_intervention_by_roadedge","ldp_roadedge_offset",
                         #new_elk debug info:
                         "elk_main_switch_","elk_enable_code_", "elk_disable_code_", "elk_fault_code_", "elk_left_suppression_code_","elk_left_kickdown_code_",
                         "elk_right_suppression_code_","elk_right_kickdown_code_","elk_tlc_threshold_","elk_roadedge_tlc_threshold_","elk_left_intervention_","elk_right_intervention_","elk_state_",
                         "elk_preview_left_y_gap","elk_preview_right_y_gap","elk_left_intervention_by_line","elk_left_intervention_by_roadedge",
                         "elk_right_intervention_by_line","elk_right_intervention_by_roadedge","elk_roadedge_offset",
                         "elk_left_has_risk","elk_right_has_risk","elk_fl_risk_code","elk_ml_risk_code","elk_rl_risk_code","elk_fr_risk_code","elk_mr_risk_code","elk_rr_risk_code",
                        ]

adas_json_value_list =  [ #adas_debug info
                         "params_dt","params_ego_length","params_ego_width", "params_origin_2_front_bumper", "params_origin_2_rear_bumper", "params_steer_ratio","params_wheel_base",
                         "params_ldp_c0_right_offset", "params_ldp_center_line_offset","params_ldp_ttlc_right_hack","params_ldp_tlc_thrd","params_ldw_enable_speed",
                         "state_left_turn_light_off_time","state_right_turn_light_off_time","state_driver_hand_trq","state_ego_curvature","state_fl_wheel_distance_to_line",
                         "state_fr_wheel_distance_to_line","state_vehicle_speed", "state_yaw_rate","state_left_departure_speed","state_right_departure_speed","state_steer_wheel_angle_degree",
                         "state_yaw_rate_observer","state_yaw_rate_loc",
                         "road_left_line_boundary_type", "road_left_line_line_type","road_left_line_begin","road_left_line_end","road_left_line_c0","road_left_line_c1","road_left_line_c2","road_left_line_c3","state_fl_wheel_distance_to_roadedge",
                         "road_right_line_boundary_type","road_right_line_line_type","road_right_line_begin", "road_right_line_end","road_right_line_c0","road_right_line_c1","road_right_line_c2","road_right_line_c3","state_fr_wheel_distance_to_roadedge",
                         "road_left_line_valid","road_right_line_valid","road_left_roadedge_valid","road_right_roadedge_valid","road_lane_width_valid","road_lane_width",
                         "road_left_roadedge_begin_x","road_left_roadedge_end_x","road_right_roadedge_begin_x","road_right_roadedge_end_x",
                         "road_left_line_segement0_length","road_left_line_segement0_type","road_left_line_segement1_length","road_left_line_segement1_type",
                         "road_left_line_segement2_length","road_left_line_segement2_type","road_left_line_segement3_length","road_left_line_segement3_type",
                         "road_right_line_segement0_length","road_right_line_segement0_type","road_right_line_segement1_length","road_right_line_segement1_type",
                         "road_right_line_segement2_length","road_right_line_segement2_type","road_right_line_segement3_length","road_right_line_segement3_type",]

json_vector_list = ["road_left_roadedge_all_dx_vec_","road_left_roadedge_all_dy_vec_","road_right_roadedge_all_dx_vec_","road_right_roadedge_all_dy_vec_"]

lkas_json_list_dict = {}
lka_t_elk_debug = []
for i in range(len(lka_json_value_list)):
   new_list = []
   lkas_json_list_dict[lka_json_value_list[i]] = new_list
for i in range(len(elk_json_data)):
  lka_t_elk_debug.append(elk_json_data_t[i])
  for j in range(len(lka_json_value_list)):
     value = elk_json_data[i][lka_json_value_list[j]]
     lkas_json_list_dict[lka_json_value_list[j]].append(value)
lkas_json_list_dict['time'] = lka_t_elk_debug

adas_json_list_dict = {}
lka_t_elk_debug = []
for i in range(len(adas_json_value_list)):
   new_list = []
   adas_json_list_dict[adas_json_value_list[i]] = new_list
for i in range(len(elk_json_data)):
  lka_t_elk_debug.append(elk_json_data_t[i])
  for j in range(len(adas_json_value_list)):
     value = elk_json_data[i][adas_json_value_list[j]]
     adas_json_list_dict[adas_json_value_list[j]].append(value)
adas_json_list_dict['time'] = lka_t_elk_debug



# figures
# fig1.line('dy_ref_mpc_vec', 'dx_ref_mpc_vec', source = data_vector, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref', visible=True)
# fig1.line('dy_mpc_vec', 'dx_mpc_vec', source = data_vector, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.35, legend_label = 'mpc', visible=True)
fig1.line('road_left_roadedge_all_dy_vec_', 'road_left_roadedge_all_dx_vec_', source = data_vector, line_width = 3, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'left_roadedge', visible=True)
fig1.line('road_right_roadedge_all_dy_vec_', 'road_right_roadedge_all_dx_vec_', source = data_vector, line_width = 3, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'right_roadedge', visible=True)
fig1.patch('car_yn', 'car_xn', source = data_car_preview, fill_color = "red", fill_alpha = 0.25, line_color = "black", line_width = 1, legend_label = 'preview_eog_pose', visible = True)
fig1.patch('obj_yn', 'obj_xn', source = data_fl_obj_selected, fill_color = "green", fill_alpha = 0.2, line_color = "black", line_width = 1, legend_label = 'fl_obj', visible = True)
fig1.patch('obj_yn', 'obj_xn', source = data_ml_obj_selected, fill_color = "green", fill_alpha = 0.5, line_color = "black", line_width = 1, legend_label = 'ml_obj', visible = True)
fig1.patch('obj_yn', 'obj_xn', source = data_rl_obj_selected, fill_color = "green", fill_alpha = 0.9, line_color = "black", line_width = 1, legend_label = 'rl_obj', visible = True)

fig1.patch('obj_yn', 'obj_xn', source = data_fm_obj_selected, fill_color = "purple", fill_alpha = 0.2, line_color = "black", line_width = 1, legend_label = 'fm_obj', visible = True)
fig1.patch('obj_yn', 'obj_xn', source = data_rm_obj_selected, fill_color = "purple", fill_alpha = 0.75, line_color = "black", line_width = 1, legend_label = 'rm_obj', visible = True)

fig1.patch('obj_yn', 'obj_xn', source = data_fr_obj_selected, fill_color = "blue", fill_alpha = 0.2, line_color = "black", line_width = 1, legend_label = 'fr_obj', visible = True)
fig1.patch('obj_yn', 'obj_xn', source = data_mr_obj_selected, fill_color = "blue", fill_alpha = 0.5, line_color = "black", line_width = 1, legend_label = 'mr_obj', visible = True)
fig1.patch('obj_yn', 'obj_xn', source = data_rr_obj_selected, fill_color = "blue", fill_alpha = 0.9, line_color = "black", line_width = 1, legend_label = 'rr_obj', visible = True)
fig1.width = 600
fig1.height = 1380

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
# fig_control_1 = bkp.figure(x_axis_label='time', y_axis_label='steering angle',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=200)

fig_machine = bkp.figure(x_axis_label='time', y_axis_label='ldw state_machine',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=200)
fig_machine.yaxis.axis_label_text_font_style = 'bold'

fig_vehicle_state = bkp.figure(x_axis_label='time', y_axis_label='vehicle state',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=260)
fig_vehicle_state.yaxis.axis_label_text_font_style = 'bold'

fig_dynamic_state = bkp.figure(x_axis_label='time', y_axis_label='dynamic state',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=200)
fig_dynamic_state.yaxis.axis_label_text_font_style = 'bold'

fig_left_machine = bkp.figure(x_axis_label='time', y_axis_label='elk left machine',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=300)
fig_left_machine.yaxis.axis_label_text_font_style = 'bold'

fig_right_machine = bkp.figure(x_axis_label='time', y_axis_label='elk right machine',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=300)
fig_right_machine.yaxis.axis_label_text_font_style = 'bold'

fig_trig_caculate = bkp.figure(x_axis_label='time', y_axis_label='line trig caculate',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=280)
fig_trig_caculate.yaxis.axis_label_text_font_style = 'bold'

fig_left_line_info = bkp.figure(x_axis_label='time', y_axis_label='left line info',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=220)
fig_left_line_info.yaxis.axis_label_text_font_style = 'bold'

fig_right_line_info = bkp.figure(x_axis_label='time', y_axis_label='right line info',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=220)
fig_right_line_info.yaxis.axis_label_text_font_style = 'bold'

fig_relative_line_info = bkp.figure(x_axis_label='time', y_axis_label='relative line info',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=200)
fig_relative_line_info.yaxis.axis_label_text_font_style = 'bold'

fig_left_line_base_info = bkp.figure(x_axis_label='time', y_axis_label='left line base info',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=280)
fig_left_line_base_info.yaxis.axis_label_text_font_style = 'bold'

fig_right_line_base_info = bkp.figure(x_axis_label='time', y_axis_label='right line base info',x_range = [lka_t_elk_debug[0], lka_t_elk_debug[-1]], width=700, height=280)
fig_right_line_base_info.yaxis.axis_label_text_font_style = 'bold'

f_machine = fig_machine.line('time', 'elk_state_', source = lkas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'elk_state')
fig_machine.line('time', 'elk_main_switch_', source = lkas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'elk_main_switch')
fig_machine.line('time', 'elk_enable_code_', source = lkas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'elk_enable_code')
fig_machine.line('time', 'elk_disable_code_', source = lkas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'elk_disable_code')
fig_machine.line('time', 'elk_fault_code_', source = lkas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'elk_fault_code')

f_vehicle_state = fig_vehicle_state.line('time', 'state_vehicle_speed', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'vehicle_speed')
fig_vehicle_state.line('time', 'state_yaw_rate', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'yaw_rate')
fig_vehicle_state.line('time', 'state_yaw_rate_observer', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'yaw_rate_cal')
fig_vehicle_state.line('time', 'state_yaw_rate_loc', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'yaw_rate_loc')
fig_vehicle_state.line('time', 'state_driver_hand_trq', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'driver_hand_trq')
fig_vehicle_state.line('time', 'state_ego_curvature', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_curvature')
fig_vehicle_state.line('time', 'state_steer_wheel_angle_degree', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer_wheel_angle_degree')

f_dynamic_state = fig_dynamic_state.line('time', 'state_left_departure_speed', source = adas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'left_departure_speed')
fig_dynamic_state.line('time', 'state_right_departure_speed', source = adas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'right_departure_speed')
fig_dynamic_state.line('time', 'state_left_turn_light_off_time', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'left_turn_light_off_time')
fig_dynamic_state.line('time', 'state_right_turn_light_off_time', source = adas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'right_turn_light_off_time')

f_left_machine = fig_left_machine.line('time', 'elk_state_', source = lkas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'elk_state')
fig_left_machine.line('time', 'elk_left_suppression_code_', source = lkas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'elk_left_suppression_code')
fig_left_machine.line('time', 'elk_left_kickdown_code_', source = lkas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'elk_left_kickdown_code')
fig_left_machine.line('time', 'elk_left_intervention_', source = lkas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'elk_left_intervention')
# fig_left_machine.line('time', 'lkas_function::elk::left_intervention', source = lkas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'lkas_function::elk::left_intervention')
fig_left_machine.line('time', 'elk_left_intervention_by_line', source = lkas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'left_intervention_by_line')
fig_left_machine.line('time', 'elk_left_intervention_by_roadedge', source = lkas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'left_intervention_by_roadedge')
fig_left_machine.line('time', 'elk_left_has_risk', source = lkas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'left_has_risk')
fig_left_machine.line('time', 'elk_fl_risk_code', source = lkas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'fl_risk_code')
fig_left_machine.line('time', 'elk_ml_risk_code', source = lkas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'ml_risk_code')
fig_left_machine.line('time', 'elk_rl_risk_code', source = lkas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'rl_risk_code')

f_right_machine = fig_right_machine.line('time', 'elk_state_', source = lkas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'ldw_state')
fig_right_machine.line('time', 'elk_right_suppression_code_', source = lkas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'elk_right_suppression_code')
fig_right_machine.line('time', 'elk_right_kickdown_code_', source = lkas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'elk_right_kickdown_code')
fig_right_machine.line('time', 'elk_right_intervention_', source = lkas_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'elk_right_intervention')
# fig_right_machine.line('time', 'lkas_function::elk::right_intervention', source = lkas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'lkas_function::elk::right_intervention')
fig_right_machine.line('time', 'elk_right_intervention_by_line', source = lkas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'right_intervention_by_line')
fig_right_machine.line('time', 'elk_right_intervention_by_roadedge', source = lkas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'right_intervention_by_roadedge')
fig_right_machine.line('time', 'elk_right_has_risk', source = lkas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'right_has_risk')
fig_right_machine.line('time', 'elk_fr_risk_code', source = lkas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'fr_risk_code')
fig_right_machine.line('time', 'elk_mr_risk_code', source = lkas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'mr_risk_code')
fig_right_machine.line('time', 'elk_rr_risk_code', source = lkas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'rr_risk_code')

f_trig_caculate = fig_trig_caculate.line('time', 'elk_state_', source = lkas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'elk_state')
f_trig_caculate = fig_trig_caculate.line('time', 'elk_preview_left_y_gap', source = lkas_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'preview_left_y_gap')
f_trig_caculate = fig_trig_caculate.line('time', 'elk_preview_right_y_gap', source = lkas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'preview_right_y_gap')
f_trig_caculate = fig_trig_caculate.line('time', 'elk_tlc_threshold_', source = lkas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'tlc_threshold_')
f_trig_caculate = fig_trig_caculate.line('time', 'elk_roadedge_tlc_threshold_', source = lkas_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'roadedge_tlc_threshold_')
f_trig_caculate = fig_trig_caculate.line('time', 'elk_roadedge_offset', source = lkas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'elk_roadedge_offset')

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
fig_relative_line_info.line('time', 'state_fr_wheel_distance_to_roadedge', source = adas_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'right_roadedge_distance')
fig_relative_line_info.line('time', 'road_lane_width_valid', source = adas_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lane_width_valid')
fig_relative_line_info.line('time', 'road_lane_width', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'lane_width')

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


hover_machine = HoverTool(renderers=[f_machine], tooltips=[('time', '@time'), ('elk_state_', '@elk_state_'),('elk_main_switch_', '@elk_main_switch_'), ('elk_enable_code_', '@elk_enable_code_'),
                                                       ('elk_disable_code_', '@elk_disable_code_'), ('elk_fault_code_', '@elk_fault_code_')], mode='vline')
hover_vehicle_state = HoverTool(renderers=[f_vehicle_state], tooltips=[('time', '@time'), ('state_vehicle_speed', '@state_vehicle_speed'), ('state_yaw_rate', '@state_yaw_rate'), ('state_yaw_rate_observer', '@state_yaw_rate_observer'),
                                               ('state_driver_hand_trq', '@state_driver_hand_trq'), ('state_ego_curvature', '@state_ego_curvature'), ('state_steer_wheel_angle_degree', '@state_steer_wheel_angle_degree')], mode='vline')
hover_dynamic_state = HoverTool(renderers=[f_dynamic_state], tooltips=[('time', '@time'), ('state_left_departure_speed', '@state_left_departure_speed'), ('state_right_departure_speed', '@state_right_departure_speed'), ('state_left_turn_light_off_time', '@state_left_turn_light_off_time'),
                                               ('state_right_turn_light_off_time', '@state_right_turn_light_off_time')], mode='vline')
hover_left_machine = HoverTool(renderers=[f_left_machine], tooltips=[('time', '@time'), ('elk_state_', '@elk_state_'),('elk_left_suppression_code_', '@elk_left_suppression_code_'), ('elk_left_kickdown_code_', '@elk_left_kickdown_code_'),
                                                       ('elk_left_intervention_', '@elk_left_intervention_'),('elk_left_intervention_by_line', '@elk_left_intervention_by_line'),('elk_left_intervention_by_roadedge', '@elk_left_intervention_by_roadedge')], mode='vline')
hover_right_machine = HoverTool(renderers=[f_right_machine], tooltips=[('time', '@time'), ('elk_state_', '@elk_state_'), ('elk_right_suppression_code_', '@elk_right_suppression_code_'),
                                                       ('elk_right_kickdown_code_', '@elk_right_kickdown_code_'), ('elk_right_intervention_', '@elk_right_intervention_'),
                                                       ('elk_right_intervention_by_line','@elk_right_intervention_by_line'),('elk_right_intervention_by_roadedge','@elk_right_intervention_by_roadedge')], mode='vline')
hover_trig_caculate = HoverTool(renderers=[f_trig_caculate], tooltips=[('time', '@time'), ('elk_state_', '@elk_state_'), ('elk_preview_left_y_gap', '@elk_preview_left_y_gap'),
                                                         ('elk_preview_right_y_gap', '@elk_preview_right_y_gap'), ('elk_tlc_threshold_', '@elk_tlc_threshold_'),('elk_roadedge_tlc_threshold_', '@elk_roadedge_tlc_threshold_'),], mode='vline')
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

  count = (int)(bag_time / 0.1)
  count_max = len(adas_json_list_dict["road_left_line_segement0_length"])
  print(count_max)
  if count >= count_max:
    count = count_max - 1
  # plan_debug_msg = find_nearest(bag_loader.plan_debug_msg, bag_time)
  print("left_segment_0_type = ",adas_json_list_dict["road_left_line_segement0_type"][count],"left_segment_0_length = ",adas_json_list_dict["road_left_line_segement0_length"][count])
  print("left_segment_1_type = ",adas_json_list_dict["road_left_line_segement1_type"][count],"left_segment_0_length = ",adas_json_list_dict["road_left_line_segement1_length"][count])
  print("left_segment_2_type = ",adas_json_list_dict["road_left_line_segement2_type"][count],"left_segment_0_length = ",adas_json_list_dict["road_left_line_segement2_length"][count])
  print("left_segment_3_type = ",adas_json_list_dict["road_left_line_segement3_type"][count],"left_segment_0_length = ",adas_json_list_dict["road_left_line_segement3_length"][count])
  print("**************")
  print("right_segment_0_type = ",adas_json_list_dict["road_right_line_segement0_type"][count],"right_segment_0_length = ",adas_json_list_dict["road_right_line_segement0_length"][count])
  print("right_segment_1_type = ",adas_json_list_dict["road_right_line_segement1_type"][count],"right_segment_1_length = ",adas_json_list_dict["road_right_line_segement1_length"][count])
  print("right_segment_2_type = ",adas_json_list_dict["road_right_line_segement2_type"][count],"right_segment_2_length = ",adas_json_list_dict["road_right_line_segement2_length"][count])
  print("right_segment_3_type = ",adas_json_list_dict["road_right_line_segement3_type"][count],"right_segment_3_length = ",adas_json_list_dict["road_right_line_segement3_length"][count])
  steer_radius = 0.0
  try:
   curvate = 0.5 * adas_json_list_dict["road_left_line_c2"][count] + 0.5 * adas_json_list_dict["road_right_line_c2"][count]
   steer_radius =  1.0 / curvate
   if abs(steer_radius) > 10000.0:
    steer_radius = 10000
   if abs(steer_radius) < 100.0:
    steer_radius = 100.0
  except:
    steer_radius = 10001.0
  print("steer_radius = ",steer_radius)

  planning_json = local_view_data['data_msg']['plan_debug_json_msg']
  road_left_roadedge_all_dx_vec_ = planning_json['road_left_roadedge_all_dx_vec_']
  road_left_roadedge_all_dy_vec_ = planning_json['road_left_roadedge_all_dy_vec_']
  road_right_roadedge_all_dx_vec_ = planning_json['road_right_roadedge_all_dx_vec_']
  road_right_roadedge_all_dy_vec_ = planning_json['road_right_roadedge_all_dy_vec_']


  data_vector.data.update({
    # 'time_vec': time_vec,
    # 'dx_ref_mpc_vec':dx_ref_mpc_vec,
    # 'dy_ref_mpc_vec':dy_ref_mpc_vec,
    # 'dx_mpc_vec':dx_mpc_vec,
    # 'dy_mpc_vec':dy_mpc_vec,
    # 'dphi_ref_mpc_vec':dphi_deg_ref_mpc_vec,
    # 'dphi_mpc_vec':dphi_deg_mpc_vec,
    # 'delta_mpc_vec': steer_mpc_vec,
   'road_left_roadedge_all_dx_vec_': road_left_roadedge_all_dx_vec_,
   'road_left_roadedge_all_dy_vec_': road_left_roadedge_all_dy_vec_,
   'road_right_roadedge_all_dx_vec_': road_right_roadedge_all_dx_vec_,
   'road_right_roadedge_all_dy_vec_': road_right_roadedge_all_dy_vec_,
  })

  car_xn = []
  car_yn = []
  preview_ego_pose_list = planning_json['obj_preview_ego_pos_vec']
  car_xn.append(preview_ego_pose_list[0])
  car_yn.append(preview_ego_pose_list[1])
  car_xn.append(preview_ego_pose_list[2])
  car_yn.append(preview_ego_pose_list[3])
  car_xn.append(preview_ego_pose_list[6])
  car_yn.append(preview_ego_pose_list[7])
  car_xn.append(preview_ego_pose_list[4])
  car_yn.append(preview_ego_pose_list[5])
  data_car_preview.data.update({
      'car_xn': car_xn,
      'car_yn': car_yn,
    })
  obj_xn = []
  obj_yn = []
  obj_loc_vec_list = planning_json['obj_fl_obj_loc_vec']
  obj_xn.append(obj_loc_vec_list[0])
  obj_yn.append(obj_loc_vec_list[1])
  obj_xn.append(obj_loc_vec_list[2])
  obj_yn.append(obj_loc_vec_list[3])
  obj_xn.append(obj_loc_vec_list[6])
  obj_yn.append(obj_loc_vec_list[7])
  obj_xn.append(obj_loc_vec_list[4])
  obj_yn.append(obj_loc_vec_list[5])
  data_fl_obj_selected.data.update({
      'obj_xn': obj_xn,
      'obj_yn': obj_yn,
    })
  obj_xn = []
  obj_yn = []
  obj_loc_vec_list = planning_json['obj_fm_obj_loc_vec']
  obj_xn.append(obj_loc_vec_list[0])
  obj_yn.append(obj_loc_vec_list[1])
  obj_xn.append(obj_loc_vec_list[2])
  obj_yn.append(obj_loc_vec_list[3])
  obj_xn.append(obj_loc_vec_list[6])
  obj_yn.append(obj_loc_vec_list[7])
  obj_xn.append(obj_loc_vec_list[4])
  obj_yn.append(obj_loc_vec_list[5])
  data_fm_obj_selected.data.update({
      'obj_xn': obj_xn,
      'obj_yn': obj_yn,
    })
  obj_xn = []
  obj_yn = []
  obj_loc_vec_list = planning_json['obj_fr_obj_loc_vec']
  obj_xn.append(obj_loc_vec_list[0])
  obj_yn.append(obj_loc_vec_list[1])
  obj_xn.append(obj_loc_vec_list[2])
  obj_yn.append(obj_loc_vec_list[3])
  obj_xn.append(obj_loc_vec_list[6])
  obj_yn.append(obj_loc_vec_list[7])
  obj_xn.append(obj_loc_vec_list[4])
  obj_yn.append(obj_loc_vec_list[5])
  data_fr_obj_selected.data.update({
      'obj_xn': obj_xn,
      'obj_yn': obj_yn,
    })
  obj_xn = []
  obj_yn = []
  obj_loc_vec_list = planning_json['obj_ml_obj_loc_vec']
  obj_xn.append(obj_loc_vec_list[0])
  obj_yn.append(obj_loc_vec_list[1])
  obj_xn.append(obj_loc_vec_list[2])
  obj_yn.append(obj_loc_vec_list[3])
  obj_xn.append(obj_loc_vec_list[6])
  obj_yn.append(obj_loc_vec_list[7])
  obj_xn.append(obj_loc_vec_list[4])
  obj_yn.append(obj_loc_vec_list[5])
  data_ml_obj_selected.data.update({
      'obj_xn': obj_xn,
      'obj_yn': obj_yn,
    })
  obj_xn = []
  obj_yn = []
  obj_loc_vec_list = planning_json['obj_mr_obj_loc_vec']
  obj_xn.append(obj_loc_vec_list[0])
  obj_yn.append(obj_loc_vec_list[1])
  obj_xn.append(obj_loc_vec_list[2])
  obj_yn.append(obj_loc_vec_list[3])
  obj_xn.append(obj_loc_vec_list[6])
  obj_yn.append(obj_loc_vec_list[7])
  obj_xn.append(obj_loc_vec_list[4])
  obj_yn.append(obj_loc_vec_list[5])
  data_mr_obj_selected.data.update({
      'obj_xn': obj_xn,
      'obj_yn': obj_yn,
    })
  obj_xn = []
  obj_yn = []
  obj_loc_vec_list = planning_json['obj_rl_obj_loc_vec']
  obj_xn.append(obj_loc_vec_list[0])
  obj_yn.append(obj_loc_vec_list[1])
  obj_xn.append(obj_loc_vec_list[2])
  obj_yn.append(obj_loc_vec_list[3])
  obj_xn.append(obj_loc_vec_list[6])
  obj_yn.append(obj_loc_vec_list[7])
  obj_xn.append(obj_loc_vec_list[4])
  obj_yn.append(obj_loc_vec_list[5])
  data_rl_obj_selected.data.update({
      'obj_xn': obj_xn,
      'obj_yn': obj_yn,
    })
  obj_xn = []
  obj_yn = []
  obj_loc_vec_list = planning_json['obj_rm_obj_loc_vec']
  obj_xn.append(obj_loc_vec_list[0])
  obj_yn.append(obj_loc_vec_list[1])
  obj_xn.append(obj_loc_vec_list[2])
  obj_yn.append(obj_loc_vec_list[3])
  obj_xn.append(obj_loc_vec_list[6])
  obj_yn.append(obj_loc_vec_list[7])
  obj_xn.append(obj_loc_vec_list[4])
  obj_yn.append(obj_loc_vec_list[5])
  data_rm_obj_selected.data.update({
      'obj_xn': obj_xn,
      'obj_yn': obj_yn,
    })
  obj_xn = []
  obj_yn = []
  obj_loc_vec_list = planning_json['obj_rr_obj_loc_vec']
  obj_xn.append(obj_loc_vec_list[0])
  obj_yn.append(obj_loc_vec_list[1])
  obj_xn.append(obj_loc_vec_list[2])
  obj_yn.append(obj_loc_vec_list[3])
  obj_xn.append(obj_loc_vec_list[6])
  obj_yn.append(obj_loc_vec_list[7])
  obj_xn.append(obj_loc_vec_list[4])
  obj_yn.append(obj_loc_vec_list[5])
  data_rr_obj_selected.data.update({
      'obj_xn': obj_xn,
      'obj_yn': obj_yn,
    })


  push_notebook()

bkp.show(row(fig1, column(fig_machine, fig_vehicle_state, fig_dynamic_state, fig_left_machine, fig_right_machine),
             column(fig_trig_caculate, fig_left_line_info, fig_right_line_info,fig_relative_line_info, fig_left_line_base_info, fig_right_line_base_info)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
