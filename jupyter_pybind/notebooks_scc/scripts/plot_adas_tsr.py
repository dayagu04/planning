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
# bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_50815/trigger/20250812/20250812-13-34-29/data_collection_CHERY_M32T_50815_EVENT_DOWNGRADE_2025-08-12-13-34-29_no_camera.bag.1755071599.open-loop.noa.plan"
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_50815/trigger/20250812/20250812-13-34-29/data_collection_CHERY_M32T_50815_EVENT_DOWNGRADE_2025-08-12-13-34-29_no_camera.bag.1755071599.open-loop.noa.plan"
frame_dt = 0.02 # sec

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
tsr_json_data = bag_loader.plan_debug_msg['json']
tsr_json_data_t = bag_loader.plan_debug_msg['t']
counter = 0

# +
tsr_json_value_list = [
                         #tsr debug info
                         "tsr_main_switch_","tsr_enable_code_","tsr_disable_code_","tsr_fault_code_","tsr_state_", "tsr_speed_limit_", "current_map_speed_limit_", "current_map_speed_limit_valid_", "speed_limit_suppression_flag_",
                         "end_of_speed_sign_display_flag_", "tsr_speed_limit_valid_","tsr_warning_image_","tsr_warning_voice_","tsr_overspeed_status_","tsr_overspeed_duration_time_",
                         "tsr_speed_limit_change_flag_","tsr_speed_limit_exist_in_view_flag_","tsr_speed_limit_exist_in_view_","tsr_accumulated_path_length_",
                         "tsr_output_supp_sign_info_", "supp_sign_in_suppression_flag_",
                        ]

adas_json_value_list =  [ #adas_debug info
                         "params_dt","params_ego_length","params_ego_width", "params_origin_2_front_bumper", "params_origin_2_rear_bumper", "params_steer_ratio","params_wheel_base",
                         "params_ldp_c0_right_offset", "params_ldp_center_line_offset","params_ldp_ttlc_right_hack","params_ldp_tlc_thrd","params_ldw_enable_speed",
                         "state_left_turn_light_off_time","state_right_turn_light_off_time","state_driver_hand_trq","state_ego_curvature","state_fl_wheel_distance_to_line",
                         "state_fr_wheel_distance_to_line","state_vehicle_speed", "state_yaw_rate","state_left_departure_speed","state_right_departure_speed","state_steer_wheel_angle_degree",
                         "state_yaw_rate_observer","state_yaw_rate_loc","state_dispaly_vehicle_speed",
                         "road_left_line_boundary_type", "road_left_line_line_type","road_left_line_begin","road_left_line_end","road_left_line_c0","road_left_line_c1","road_left_line_c2","road_left_line_c3","state_fl_wheel_distance_to_roadedge",
                         "road_right_line_boundary_type","road_right_line_line_type","road_right_line_begin", "road_right_line_end","road_right_line_c0","road_right_line_c1","road_right_line_c2","road_right_line_c3","state_fr_wheel_distance_to_roadedge",
                         "road_left_line_valid","road_right_line_valid","road_left_roadedge_valid","road_right_roadedge_valid","road_lane_width_valid","road_lane_width",
                         "road_left_roadedge_begin_x","road_left_roadedge_end_x","road_right_roadedge_begin_x","road_right_roadedge_end_x",
                         "road_left_line_segement0_length","road_left_line_segement0_type","road_left_line_segement1_length","road_left_line_segement1_type",
                         "road_left_line_segement2_length","road_left_line_segement2_type","road_left_line_segement3_length","road_left_line_segement3_type",
                         "road_right_line_segement0_length","road_right_line_segement0_type","road_right_line_segement1_length","road_right_line_segement1_type",
                         "road_right_line_segement2_length","road_right_line_segement2_type","road_right_line_segement3_length","road_right_line_segement3_type",]

json_vector_list = ["road_left_roadedge_all_dx_vec_","road_left_roadedge_all_dy_vec_","road_right_roadedge_all_dx_vec_","road_right_roadedge_all_dy_vec_"]

tsr_json_list_dict = {}
tsr_t_tsr_debug = []
for i in range(len(tsr_json_value_list)):
   new_list = []
   tsr_json_list_dict[tsr_json_value_list[i]] = new_list
for i in range(len(tsr_json_data)):
  tsr_t_tsr_debug.append(tsr_json_data_t[i])
  for j in range(len(tsr_json_list_dict)):
     value = tsr_json_data[i][tsr_json_value_list[j]]
     tsr_json_list_dict[tsr_json_value_list[j]].append(value)
tsr_json_list_dict['time'] = tsr_t_tsr_debug

adas_json_list_dict = {}
tsr_t_tsr_debug = []
for i in range(len(adas_json_value_list)):
   new_list = []
   adas_json_list_dict[adas_json_value_list[i]] = new_list
for i in range(len(tsr_json_data)):
  tsr_t_tsr_debug.append(tsr_json_data_t[i])
  for j in range(len(adas_json_value_list)):
     value = tsr_json_data[i][adas_json_value_list[j]]
     if adas_json_value_list[j] == "state_dispaly_vehicle_speed":
       adas_json_list_dict[adas_json_value_list[j]].append((value * 3.6))
     else:
       adas_json_list_dict[adas_json_value_list[j]].append(value)
adas_json_list_dict['time'] = tsr_t_tsr_debug



# figures
# fig1.line('dy_ref_mpc_vec', 'dx_ref_mpc_vec', source = data_vector, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref', visible=True)
# fig1.line('dy_mpc_vec', 'dx_mpc_vec', source = data_vector, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.35, legend_label = 'mpc', visible=True)
# fig1.line('road_left_roadedge_all_dy_vec_', 'road_left_roadedge_all_dx_vec_', source = data_vector, line_width = 3, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'left_roadedge', visible=True)
# fig1.line('road_right_roadedge_all_dy_vec_', 'road_right_roadedge_all_dx_vec_', source = data_vector, line_width = 3, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'right_roadedge', visible=True)
# fig1.width = 600
# fig1.height = 1380

# source = ColumnDataSource(data=dict(x=[], y=[]))
# fig1.circle('x', 'y', size=6, source=source, color='red', legend_label='measure tool')
# line_source = ColumnDataSource(data=dict(x=[], y=[]))
# fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool')
# text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
# fig1.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool')
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
# callback = CustomJS(args=dict(source=source, line_source=line_source, text_source=text_source), code=callback_code)
# fig1.js_on_event(Tap, callback)
#f
# fig_control_1 = bkp.figure(x_axis_label='time', y_axis_label='steering angle',x_range = [lka_t_tsr_debug[0], lka_t_tsr_debug[-1]], width=700, height=200)

fig_machine = bkp.figure(x_axis_label='time', y_axis_label='tsr state_machine',x_range = [tsr_t_tsr_debug[0], tsr_t_tsr_debug[-1]], width=700, height=200)
fig_machine.yaxis.axis_label_text_font_style = 'bold'

fig_speed_info = bkp.figure(x_axis_label='time', y_axis_label='speed info',x_range = [tsr_t_tsr_debug[0], tsr_t_tsr_debug[-1]], width=700, height=260)
fig_speed_info.yaxis.axis_label_text_font_style = 'bold'

fig_display_info = bkp.figure(x_axis_label='time', y_axis_label='vehicle state',x_range = [tsr_t_tsr_debug[0], tsr_t_tsr_debug[-1]], width=700, height=260)
fig_display_info.yaxis.axis_label_text_font_style = 'bold'

fig_dynamic_state = bkp.figure(x_axis_label='time', y_axis_label='dynamic state',x_range = [tsr_t_tsr_debug[0], tsr_t_tsr_debug[-1]], width=700, height=300)
fig_dynamic_state.yaxis.axis_label_text_font_style = 'bold'

fig_path_info = bkp.figure(x_axis_label='time', y_axis_label='path info',x_range = [tsr_t_tsr_debug[0], tsr_t_tsr_debug[-1]], width=700, height=280)
fig_path_info.yaxis.axis_label_text_font_style = 'bold'

# 状态机
f_machine = fig_machine.line('time', 'tsr_state_', source = tsr_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'tsr_state')
fig_machine.line('time', 'tsr_main_switch_', source = tsr_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'tsr_main_switch')
fig_machine.line('time', 'tsr_enable_code_', source = tsr_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'tsr_enable_code')
fig_machine.line('time', 'tsr_disable_code_', source = tsr_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'tsr_disable_code')
fig_machine.line('time', 'tsr_fault_code_', source = tsr_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'tsr_fault_code')

# 速度相关
f_speed_info = fig_speed_info.line('time', 'state_dispaly_vehicle_speed', source = adas_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'display_vehicle_speed')
f_tsr_speed_limit = fig_speed_info.line('time', 'tsr_speed_limit_', source = tsr_json_list_dict, line_width = 2, line_color = 'red', line_dash = 'solid', legend_label = 'speed_limit')
f_map_speed_limit = fig_speed_info.line('time', 'current_map_speed_limit_', source = tsr_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'current_map_speed_limit')
fig_speed_info.line('time', 'tsr_speed_limit_exist_in_view_', source = tsr_json_list_dict, line_width = 1, line_color = 'navy', line_dash = 'solid', legend_label = 'speed_limit_exist_in_view')

# 车机显示信息
f_display_info = fig_display_info.line('time', 'tsr_output_supp_sign_info_', source = tsr_json_list_dict, line_width = 1, line_color = 'gray', line_dash = 'solid', legend_label = 'output_supp_sign_info')
fig_display_info.line('time', 'tsr_overspeed_status_', source = tsr_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'overspeed_status')
fig_display_info.line('time', 'tsr_warning_image_', source = tsr_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'warning_image')
fig_display_info.line('time', 'tsr_warning_voice_', source = tsr_json_list_dict, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'warning_voice')

# 动态状态
f_dynamic_state = fig_dynamic_state.line('time', 'tsr_speed_limit_valid_', source = tsr_json_list_dict, line_width = 1, line_color = 'purple', line_dash = 'solid', legend_label = 'tsr_speed_limit_valid')
fig_dynamic_state.line('time', 'current_map_speed_limit_valid_', source = tsr_json_list_dict, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'current_map_speed_limit_valid')
fig_dynamic_state.line('time', 'speed_limit_suppression_flag_', source = tsr_json_list_dict, line_width = 1, line_color = 'yellow', line_dash = 'solid', legend_label = 'speed_limit_suppression_flag')
fig_dynamic_state.line('time', 'end_of_speed_sign_display_flag_', source = tsr_json_list_dict, line_width = 1, line_color = 'pink', line_dash = 'solid', legend_label = 'end_of_speed_sign_display_flag')
fig_dynamic_state.line('time', 'supp_sign_in_suppression_flag_', source = tsr_json_list_dict, line_width = 1, line_color = 'gray', line_dash = 'solid', legend_label = 'supp_sign_in_suppression_flag')
fig_dynamic_state.line('time', 'tsr_speed_limit_exist_in_view_flag_', source = tsr_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'speed_limit_exist_in_view_flag')
fig_dynamic_state.line('time', 'tsr_speed_limit_change_flag_', source = tsr_json_list_dict, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'speed_limit_change_flag')

f_path_info = fig_path_info.line('time', 'tsr_accumulated_path_length_', source = tsr_json_list_dict, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'accumulated_path_length')
fig_path_info.line('time', 'tsr_overspeed_duration_time_', source = tsr_json_list_dict, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'overspeed_duration_time')

hover_machine = HoverTool(renderers=[f_machine], tooltips=[('time', '@time'), ('tsr_state_', '@tsr_state_'),('tsr_main_switch_', '@tsr_main_switch_'), ('tsr_enable_code_', '@tsr_enable_code_'),
                                                       ('tsr_disable_code_', '@tsr_disable_code_'), ('tsr_fault_code_', '@tsr_fault_code_')], mode='vline')
hover_speed_info_vehicle = HoverTool(renderers=[f_speed_info], tooltips=[('time', '@time'), ('state_dispaly_vehicle_speed', '@state_dispaly_vehicle_speed')], mode='vline')
hover_speed_info_tsr = HoverTool(renderers=[f_tsr_speed_limit, f_map_speed_limit], tooltips=[('time', '@time'), ('tsr_speed_limit_', '@tsr_speed_limit_'), ('current_map_speed_limit_', '@current_map_speed_limit_'), ('tsr_speed_limit_exist_in_view_', '@tsr_speed_limit_exist_in_view_')], mode='vline')
hover_display_info = HoverTool(renderers=[f_display_info], tooltips=[('time', '@time'), ('tsr_output_supp_sign_info_', '@tsr_output_supp_sign_info_'),
                                                                     ('tsr_overspeed_status_', '@tsr_overspeed_status_'), ('tsr_warning_image_', '@tsr_warning_image_'),
                                                                     ('tsr_warning_voice_', '@tsr_warning_voice_')], mode='vline')
hover_dynamic_state = HoverTool(renderers=[f_dynamic_state], tooltips=[('time', '@time'), ('tsr_speed_limit_valid_', '@tsr_speed_limit_valid_'), ('current_map_speed_limit_valid_', '@current_map_speed_limit_valid_'),
                                                                     ('speed_limit_suppression_flag_', '@speed_limit_suppression_flag_'), ('end_of_speed_sign_display_flag_', '@end_of_speed_sign_display_flag_'),
                                                                     ('supp_sign_in_suppression_flag_', '@supp_sign_in_suppression_flag_'), ('tsr_speed_limit_exist_in_view_flag_', '@tsr_speed_limit_exist_in_view_flag_'),
                                                                     ('tsr_speed_limit_change_flag_', '@tsr_speed_limit_change_flag_')], mode='vline')
hover_path_info = HoverTool(renderers=[f_path_info], tooltips=[('time', '@time'), ('tsr_speed_limit_change_flag_', '@tsr_speed_limit_change_flag_'),('tsr_accumulated_path_length_', '@tsr_accumulated_path_length_')], mode='vline')

fig_machine.add_tools(hover_machine)
fig_speed_info.add_tools(hover_speed_info_vehicle)
fig_speed_info.add_tools(hover_speed_info_tsr)
fig_display_info.add_tools(hover_display_info)
fig_dynamic_state.add_tools(hover_dynamic_state)
fig_path_info.add_tools(hover_path_info)

fig_machine.toolbar.active_scroll = fig_machine.select_one(WheelZoomTool)
fig_speed_info.toolbar.active_scroll = fig_speed_info.select_one(WheelZoomTool)
fig_display_info.toolbar.active_scroll = fig_display_info.select_one(WheelZoomTool)
fig_dynamic_state.toolbar.active_scroll = fig_dynamic_state.select_one(WheelZoomTool)
fig_path_info.toolbar.active_scroll = fig_path_info.select_one(WheelZoomTool)
fig_machine.legend.click_policy = 'hide'
fig_speed_info.legend.click_policy = 'hide'
fig_display_info.legend.click_policy = 'hide'
fig_dynamic_state.legend.click_policy = 'hide'
fig_path_info.legend.click_policy = 'hide'
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

  push_notebook()

bkp.show(row(column(fig_machine, fig_speed_info, fig_display_info, fig_dynamic_state,fig_path_info)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
