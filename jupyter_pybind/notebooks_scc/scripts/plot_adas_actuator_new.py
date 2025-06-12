from cyber_record.record import Record
import bokeh.plotting as bkp
from bokeh.layouts import row, column
import csv
import rosbag


# 设置cyber_bag文件的名称及路径
bag_file = "/mnt/xinliu37/C_struct/20240425/20240425-10-26-52/data_collection_JAC_S811_37XU2_EVENT_MANUAL_2024-04-25-10-26-52.bag"
bag_file = "/mnt/xinliu37/C_struct/20240425/20240425-10-23-36/data_collection_JAC_S811_37XU2_EVENT_MANUAL_2024-04-25-10-23-36.bag"
bag_file = "/data_cold/abu_zone/autoparse/chery_e0y_20267/trigger/20250604/20250604-20-56-28/park_in_data_collection_CHERY_E0Y_20267_ALL_FILTER_2025-06-04-20-56-28_no_camera.bag"

# 设置html的名称及路径
result = bag_file.split('/')[-1]
html_file = result +"control.html"

# 生成一个html文件
bkp.output_file(html_file)


# 获取bag包中的topic_list
def get_topic_list(bag_file_name):
    record = rosbag.Bag(bag_file_name)  # 解析bag包,将所有信息存储在record中
    topic_list = []  # 用于存储topic名称
    for topic, message, t in record.read_messages():
        if not(topic in topic_list):
            topic_list.append(topic)  # 将新获取的topic名称加入列表中
    print("topic_list如下:")
    for item in topic_list:
        print(item)


# 根据指定topic的内容
def get_topic_data(record_bag_data, topic_name):
    topic_data = list()   # 用于存储topic内容
    for topic, message, t in record_bag_data.read_messages(topic_name):
        topic_data.append(message)
    return topic_data


# 根据指定topic中指定信号的内容
def get_signal_data(record_topic_data, signal_name):
    signal_data = list()   # 用于存储topic内容
    for record_topic_data_count in range(len(record_topic_data)):
        signal_data.append(getattr(record_topic_data[record_topic_data_count], signal_name))
    return signal_data


# 获取所有topic的名称
get_topic_list(bag_file)
# 解析bag包,将所有信息存储在bag_data中
bag_data = rosbag.Bag(bag_file)




# 解析control_debug
topic_name_control_debug= "/iflytek/control/debug_info"  # 设置需要特定读取的topic名称
control_debug_data = get_topic_data(bag_data, topic_name_control_debug)



# 解析control
topic_name_control = "/iflytek/control/control_command"  # 设置需要特定读取的topic名称
control_data = get_topic_data(bag_data, topic_name_control)
control_time = list()
control_time_gap = list()
for i in range(len(control_data)):
    control_time.append((control_data[i].msg_header.stamp-control_data[0].msg_header.stamp)/1000000)
    if i == 0:
      control_time_gap.append(0.0)
    else:
      control_time_gap.append((control_data[i].msg_header.stamp-control_data[i-1].msg_header.stamp)/1000)

lon_control_mode = list()
for i in range(len(control_data)):
    lon_control_mode.append(control_data[i].lon_control_mode.lon_control_mode)
lat_control_mode = list()
control_status = list()
stop_distance = list()
gear_cmd_ctrl = list()
control_result_points_size = list()
version = list()
for i in range(len(control_data)):
    lat_control_mode.append(control_data[i].lat_control_mode.lat_control_mode)
    control_status.append(control_data[i].control_status.control_status_type)
    stop_distance.append(control_data[i].stop_distance)
    gear_cmd_ctrl.append(control_data[i].gear_command_value)
    control_result_points_size.append(control_data[i].control_trajectory.control_result_points_size)
fig_ctrl_1 = bkp.figure(title="control_mode", x_axis_label='x', y_axis_label='y',
                  width=500, height=500)
fig_ctrl_1.line(control_time, control_status,
          legend_label="control_status", line_width=2, color='red')
fig_ctrl_1.line(control_time, lon_control_mode,
          legend_label="lon_control_mode", line_width=2, color='green')
fig_ctrl_1.line(control_time, lat_control_mode,
          legend_label="lat_control_mode", line_width=2, color='blue')
fig_ctrl_1.line(control_time, stop_distance,
          legend_label="stop_distance", line_width=2, color='black')
fig_ctrl_1.line(control_time, gear_cmd_ctrl,
          legend_label="gear_cmd_ctrl", line_width=2, color='blue')
fig_ctrl_1.line(control_time, control_result_points_size,
          legend_label="points_size", line_width=2, color='red')
fig_ctrl_1.legend.click_policy = 'hide'
axle_torque = list()
for i in range(len(control_data)):
    axle_torque.append(control_data[i].axle_torque)
acceleration = list()
for i in range(len(control_data)):
    acceleration.append(control_data[i].acceleration)


fig_ctrl_2 = bkp.figure(title="axle_torque", x_axis_label='x', y_axis_label='y',
                  width=500, height=500)
fig_ctrl_2.line(control_time, axle_torque,
          legend_label="axle_torque_req", line_width=2, color='red')
fig_ctrl_2.legend.click_policy = 'hide'


fig_ctrl_3 = bkp.figure(title="acceleration", x_axis_label='x', y_axis_label='y',
                  width=500, height=500)
fig_ctrl_3.line(control_time, acceleration,
          legend_label="acceleration", line_width=2, color='red')
fig_ctrl_3.legend.click_policy = 'hide'
steering = list()
for i in range(len(control_data)):
    steering.append(control_data[i].steering)
fig_ctrl_4 = bkp.figure(title="steering", x_axis_label='x', y_axis_label='y',
                  width=500, height=500)
fig_ctrl_4.line(control_time, steering,
          legend_label="steering", line_width=2, color='red')
fig_ctrl_4.legend.click_policy = 'hide'

fig_ctrl_5 = bkp.figure(title="steering", x_axis_label='x', y_axis_label='y',
                  width=500, height=500)
fig_ctrl_5.line(control_time, control_time_gap,
          legend_label="control_time_gap", line_width=2, color='red')
fig_ctrl_5.legend.click_policy = 'hide'

# 解析vehicle_service
topic_name_vehicle_service = "/iflytek/vehicle_service"  # 设置需要特定读取的topic名称
vehicle_service_data = get_topic_data(bag_data, topic_name_vehicle_service)
vehicle_service_time = list()
for i in range(len(vehicle_service_data)):
    vehicle_service_time.append(
        (vehicle_service_data[i].msg_header.stamp -
         control_data[0].msg_header.stamp)/1000000)
pilot_long_control_actuator_status = list()
for i in range(len(vehicle_service_data)):
    pilot_long_control_actuator_status.append(vehicle_service_data[i].pilot_long_control_actuator_status)
pilot_lat_control_actuator_status = list()
for i in range(len(vehicle_service_data)):
    pilot_lat_control_actuator_status.append(vehicle_service_data[i].pilot_lat_control_actuator_status)
pilot_lat_control_actuator_status_available = list()
for i in range(len(vehicle_service_data)):
    pilot_lat_control_actuator_status_available.append(vehicle_service_data[i].pilot_lat_control_actuator_status_available)
parking_lat_control_actuator_status = list()
for i in range(len(vehicle_service_data)):
    parking_lat_control_actuator_status.append(vehicle_service_data[i].parking_lat_control_actuator_status)
parking_long_control_actuator_status = list()
for i in range(len(vehicle_service_data)):
    parking_long_control_actuator_status.append(vehicle_service_data[i].parking_long_control_actuator_status)


power_train_current_torque = list()
for i in range(len(vehicle_service_data)):
    power_train_current_torque.append(vehicle_service_data[i].power_train_current_torque)

# 创建一个带有标题和轴标签的新图表
fig_veh_service_1 = bkp.figure(title="pilot_control_actuator_status", x_axis_label='x', y_axis_label='y',
                  width=500, height=500)
fig_veh_service_1.line(vehicle_service_time, parking_long_control_actuator_status,
          legend_label="parking_long_control_actuator_status", line_width=2, color='red')
fig_veh_service_1.line(vehicle_service_time, pilot_lat_control_actuator_status,
          legend_label="pilot_lat_control_actuator_status", line_width=2, color='blue')
fig_veh_service_1.line(vehicle_service_time, parking_lat_control_actuator_status,
          legend_label="parking_lat_control_actuator_status", line_width=2, color='black')
fig_veh_service_1.line(vehicle_service_time, pilot_long_control_actuator_status,
          legend_label="pilot_long_control_actuator_status", line_width=2, color='green')
fig_veh_service_1.legend.click_policy = 'hide'
brake_pedal_pressed = list()
for i in range(len(vehicle_service_data)):
    brake_pedal_pressed.append(vehicle_service_data[i].brake_pedal_pressed)

power_train_override_flag = list()
gear_actual=list()
epb_state=list()
for i in range(len(vehicle_service_data)):
    power_train_override_flag.append(vehicle_service_data[i].power_train_override_flag)
for i in range(len(vehicle_service_data)):
    gear_actual.append(vehicle_service_data[i].shift_lever_state)
epb_state=get_signal_data(vehicle_service_data, "epb_state")

fig_veh_service_2 = bkp.figure(title="vehcle_service", x_axis_label='x', y_axis_label='y',
                  width=500, height=500)
fig_veh_service_2.line(vehicle_service_time, brake_pedal_pressed,
          legend_label="brake_pedal_pressed", line_width=2, color='blue')

fig_veh_service_2.line(vehicle_service_time, power_train_override_flag,
          legend_label="power_train_override_flag", line_width=2, color='red')
fig_veh_service_2.line(vehicle_service_time, gear_actual,
          legend_label="gear_actual", line_width=2, color='red')
fig_veh_service_2.line(vehicle_service_time, epb_state,
          legend_label="epb_state", line_width=2, color='black')

# 0:Release(Fully Released)
# 1:Apply(Fully Applied)
# 2:Releasing
# 3:Applying
fig_veh_service_2.legend.click_policy = 'hide'
long_acceleration = list()
for i in range(len(vehicle_service_data)):
    long_acceleration.append(vehicle_service_data[i].long_acceleration)
fig_veh_service_3 = bkp.figure(title="long_acceleration", x_axis_label='x', y_axis_label='y',
                  width=500, height=500)
fig_veh_service_3.line(control_time, acceleration,
          legend_label="acceleration", line_width=2, color='red')
fig_veh_service_3.line(vehicle_service_time, long_acceleration,
          legend_label="long_acceleration", line_width=2, color='blue')
fig_veh_service_3.legend.click_policy = 'hide'
steering_wheel_angle = get_signal_data(vehicle_service_data, "steering_wheel_angle")
steering_wheel_angle = [x*57.3 for x in steering_wheel_angle]
fig_veh_service_4 = bkp.figure(title="steering_wheel_angle", x_axis_label='x', y_axis_label='y',
                  width=500, height=500)
fig_veh_service_4.line(control_time, steering,
          legend_label="steering", line_width=2, color='red')
fig_veh_service_4.line(vehicle_service_time, steering_wheel_angle,
          legend_label="steering_wheel_angle_deg", line_width=2, color='blue')
fig_veh_service_4.legend.click_policy = 'hide'
vehicle_speed = get_signal_data(vehicle_service_data, "vehicle_speed")
vehicle_speed = [x*3.6 for x in vehicle_speed]
fig_veh_service_5 = bkp.figure(title="vehicle_speed", x_axis_label='x', y_axis_label='y',
                  width=500, height=500)
fig_veh_service_5.line(vehicle_service_time, vehicle_speed,
          legend_label="vehicle_speed_kph", line_width=2, color='red')
fig_veh_service_5.legend.click_policy = 'hide'
driver_hand_torque = get_signal_data(vehicle_service_data, "driver_hand_torque")
fig_veh_service_6 = bkp.figure(title="driver_hand_torque", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
fig_veh_service_6.line(vehicle_service_time, driver_hand_torque,
           legend_label="driver_hand_torque", line_width=2, color='red')
fig_veh_service_6.legend.click_policy = 'hide'

# 解析soc_state
topic_name_system_state = "/iflytek/fsm/soc_state"  # 设置需要特定读取的topic名称
system_state_data = get_topic_data(bag_data, topic_name_system_state)
system_state_time = list()
system_state_time_gap = list()
for i in range(len(system_state_data)):
    #system_state_time.append(i*0.02)
    system_state_time.append(
        (system_state_data[i].msg_header.stamp -
         control_data[0].msg_header.stamp)/1000000)
    if i == 0:
       system_state_time_gap.append(0.0)
    else:
       system_state_time_gap.append(
        (system_state_data[i].msg_header.stamp -
         system_state_data[i-1].msg_header.stamp)/1000)

current_state = get_signal_data(system_state_data, "current_state")
switch_state_date = get_signal_data(system_state_data, "switch_sts")
ldw_switch = get_signal_data(switch_state_date, "ldw_main_switch")
ldp_switch = get_signal_data(switch_state_date, "ldp_main_switch")
elk_switch = get_signal_data(switch_state_date, "elk_main_switch")

fig_soc_state_1 = bkp.figure(title="current_state", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
fig_soc_state_1.line(system_state_time, current_state,
           legend_label="current_state", line_width=2, color='red')
fig_soc_state_1.line(system_state_time, ldw_switch,
           legend_label="ldw_switch", line_width=2, color='red')
fig_soc_state_1.line(system_state_time, ldp_switch,
           legend_label="ldp_switch", line_width=2, color='red')
fig_soc_state_1.line(system_state_time, elk_switch,
           legend_label="elk_switch", line_width=2, color='red')
fig_soc_state_1.legend.click_policy = 'hide'

fig_soc_state_2 = bkp.figure(title="system_state_time_gap", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
fig_soc_state_2.line(system_state_time, system_state_time_gap,
           legend_label="system_state_time_gap", line_width=2, color='red')
fig_soc_state_2.legend.click_policy = 'hide'



# 解析adas_function_debug
topic_name_adas_function_debug = "/iflytek/adas_function/debug_info"  # 设置需要特定读取的topic名称/iflytek/adas_function_debug
adas_function_debug_data = get_topic_data(bag_data, topic_name_adas_function_debug)
adas_function_debug_time = list()
for i in range(len(adas_function_debug_data)):
    adas_function_debug_time.append(
        (adas_function_debug_data[i].msg_header.stamp -
         control_data[0].msg_header.stamp)/1000000)
control_adaptor_debug_output_info_data = get_signal_data(adas_function_debug_data, "control_adaptor_debug_output_info")
bsd_output_info = get_signal_data(adas_function_debug_data, "bsd_output_info")
fctb_output_info = get_signal_data(adas_function_debug_data, "fctb_output_info")
rctb_output_info= get_signal_data(adas_function_debug_data, "rctb_output_info")
fcta_output_info= get_signal_data(adas_function_debug_data, "fcta_output_info")
eth_debug_output_info= get_signal_data(adas_function_debug_data, "eth_debug_output_info")
soc_to_mcu_long_acc= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_trq_feedback_p")
soc_to_mcu_steering = get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_desired_angle")
soc_to_mcu_current_state = get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_desired_angle_dt")
park_brake_mode= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_desired_angle_error")
park_abnormal_code= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_desired_angle_error_dt")
gear_req_value= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_trq_feedforward")
func_state_time_gap= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_trq_feedback")
soc_to_mcu_acceleration= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_trq_feedback_p")
epb_req_value= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_trq_feedback_p_gain")
control_adp_time_gap= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_trq_feedback_i")
ONEBOX_5_APCStatus= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_trq_feedback_i_gain")
soc_mcu_time_stamp_availabele= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_trq_req_status")
soc_mcu_time_stamp= get_signal_data(control_adaptor_debug_output_info_data, "lat_ctrl_trq_req_value")
ctr_adp_out_strwheel_angle_req_value=get_signal_data(fctb_output_info, "fctb_deceleration_request_value")
temp_control_adapter_input_info_steering=get_signal_data(rctb_output_info, "rctb_deceleration_request_value")
park_acc_to_veh=get_signal_data(eth_debug_output_info, "park_acc_to_veh")
park_steer_to_veh=get_signal_data(eth_debug_output_info, "park_steer_to_veh")
park_torque_to_veh=get_signal_data(eth_debug_output_info, "park_torque_to_veh")


fcw_level=[]
for i in range(len(adas_function_debug_data)):
    fcw_level.append(adas_function_debug_data[i].fcw_output_info.fcw_warning_level)


adas_function_debug_data[101]

adas_function_1 = bkp.figure(title="转角通讯延迟判断", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
adas_function_1.line(adas_function_debug_time, ONEBOX_5_APCStatus,
           legend_label="ONEBOX_5_APCStatus", line_width=2, color='blue')
adas_function_1.line(adas_function_debug_time, park_steer_to_veh,
           legend_label="park_steer_to_veh", line_width=2, color='blue')
adas_function_1.line(adas_function_debug_time, fcw_level,
           legend_label="fcw_level", line_width=2, color='yellow')

adas_function_1.line(adas_function_debug_time, func_state_time_gap,
           legend_label="func_state_time_gap",line_width=2, color='brown')

adas_function_1.legend.click_policy = 'hide'



adas_function_2 = bkp.figure(title="延迟有效位判断", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
adas_function_2.line(adas_function_debug_time, soc_mcu_time_stamp_availabele,
           legend_label="soc_mcu_time_stamp_availabele", line_width=2, color='red')
adas_function_2.line(adas_function_debug_time, soc_mcu_time_stamp,
           legend_label="soc_mcu_time_stamp", line_width=2, color='blue')
adas_function_2.circle(adas_function_debug_time, soc_mcu_time_stamp,
           legend_label="soc_mcu_time_stamp", size=5, color='blue', alpha=0.5)
adas_function_2.legend.click_policy = 'hide'


adas_function_3 = bkp.figure(title="纵向通讯延迟判断", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
adas_function_3.line(adas_function_debug_time, soc_to_mcu_acceleration,
           legend_label="soc_to_mcu_acceleration", line_width=2, color='blue')
adas_function_3.line(control_time, acceleration,
           legend_label="acceleration",line_width=2, color='black')
adas_function_3.legend.click_policy = 'hide'


adas_function_4 = bkp.figure(title="状态机延迟", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
adas_function_4.line(adas_function_debug_time, soc_to_mcu_current_state,
           legend_label="soc_to_mcu_current_state", line_width=2, color='red')
adas_function_4.line(system_state_time, current_state,
           legend_label="current_state", line_width=2, color='black')
adas_function_4.legend.click_policy = 'hide'

adas_function_5 = bkp.figure(title="泊车状态判断", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
adas_function_5.line(adas_function_debug_time, gear_req_value,
           legend_label="ctr_gear_req_value", line_width=2, color='red')
adas_function_5.line(adas_function_debug_time, park_brake_mode,
           legend_label="park_brake_mode", line_width=2, color='black')
adas_function_5.line(adas_function_debug_time, gear_cmd_ctrl,
           legend_label="ctr_epb_req_value", line_width=2, color='green')
adas_function_5.line(adas_function_debug_time, park_abnormal_code,
           legend_label="park_abnormal_code", line_width=2, color='yellow')
adas_function_5.legend.click_policy = 'hide'

adas_function_6 = bkp.figure(title="IPB主缸压力&扭矩请求", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)

power_train_current_torque = get_signal_data(vehicle_service_data, "power_train_current_torque")
power_train_current_torque = [x/100 for x in power_train_current_torque]
esp_pressure= get_signal_data(vehicle_service_data, "esp_pressure")

adas_function_6.line(vehicle_service_time, power_train_current_torque,
           legend_label="power_train_current_torque", line_width=2, color='red')
adas_function_6.line(vehicle_service_time, esp_pressure,
           legend_label="esp_pressure", line_width=2, color='black')
adas_function_6.legend.click_policy = 'hide'

# 解析/planning/plan
topic_name_plan= "/iflytek/planning/plan"  # 设置需要特定读取的topic名称
plan_data = get_topic_data(bag_data, topic_name_plan)
plan_time = list()
plan_traj=get_signal_data(plan_data,"trajectory")
plan_traj_available =get_signal_data(plan_traj,"available")
plan_frequency = list()
plan_time_gap = list()
for i in range(len(plan_data)):
    plan_time.append((plan_data[i].msg_header.stamp-control_data[0].msg_header.stamp)/1000000)
    if i == 0:
        plan_time_gap.append(0.0)
    else:
        plan_time_gap.append((plan_data[i].msg_header.stamp-plan_data[i-1].msg_header.stamp)/1000)

apa_planning_status = list()
planning_gear_cmd_available = list()
planning_gear_cmd = list()
trajectory_points_size = list()
trajectory_points_ok_flag = list()
for i in range(len(plan_data)):
    apa_planning_status.append(plan_data[i].planning_status.apa_planning_status)
    planning_gear_cmd_available.append(plan_data[i].gear_command.available)
    planning_gear_cmd.append(plan_data[i].gear_command.gear_command_value)
    trajectory_points_size.append(plan_data[i].trajectory.trajectory_points_size)
    try:
      if abs(plan_data[i].trajectory.trajectory_points[-1].x - plan_data[i].trajectory.trajectory_points[-2].x) + abs(plan_data[i].trajectory.trajectory_points[-1].y - plan_data[i].trajectory.trajectory_points[-2].y) < 0.0001:
       trajectory_points_ok_flag.append(0)
      else:
       trajectory_points_ok_flag.append(1)
    except:
       trajectory_points_ok_flag.append(2)

planning_1 = bkp.figure(title="plan_traj_available", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
planning_1.line(plan_time, apa_planning_status,
           legend_label="apa_planning_status", line_width=2, color='red')
planning_1.line(plan_time, plan_traj_available,
           legend_label="plan_traj_available", line_width=2, color='blue')
planning_1.line(plan_time, trajectory_points_size,
           legend_label="trajectory_points_size", line_width=2, color='blue')
planning_1.line(plan_time, trajectory_points_ok_flag,
           legend_label="trajectory_points_ok_flag", line_width=2, color='blue')
planning_1.line(plan_time, planning_gear_cmd_available,
           legend_label="planning_gear_cmd_available", line_width=2, color='green')
planning_1.line(plan_time, planning_gear_cmd,
           legend_label="planning_gear_cmd", line_width=2, color='green')
planning_1.legend.click_policy = 'hide'

planning_2 = bkp.figure(title="plan_time_gap", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
planning_2.line(plan_time, plan_time_gap,
           legend_label="plan_time_gap", line_width=2, color='blue')
planning_2.legend.click_policy = 'hide'

# /iflytek/hmi/inner"/iflytek/hmi/inner"
# 解析/plan
topic_name_hmi_inner = "/iflytek/hmi/inner"  # 设置需要特定读取的topic名称
hmi_inner_data = get_topic_data(bag_data, topic_name_hmi_inner)
adas_in_info= get_signal_data(hmi_inner_data, "adas_in")
ldw_main_switch= get_signal_data(adas_in_info, "ldw_main_switch")
ldp_main_switch= get_signal_data(adas_in_info, "ldp_main_switch")
elk_main_switch= get_signal_data(adas_in_info, "elk_main_switch")

hmi_inner_time=list()
for i in range(len(hmi_inner_data)):
    hmi_inner_time.append((hmi_inner_data[i].msg_header.stamp-control_data[0].msg_header.stamp)/1000000)

hmi_inner_1 = bkp.figure(title="hmi_inner_info", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
hmi_inner_1.line(hmi_inner_time, ldw_main_switch,
           legend_label="ldw_main_switch", line_width=2, color='blue')
hmi_inner_1.line(hmi_inner_time, ldp_main_switch,
           legend_label="ldp_main_switch", line_width=2, color='red')
hmi_inner_1.line(hmi_inner_time, elk_main_switch,
           legend_label="elk_main_switch", line_width=2, color='green')
hmi_inner_1.legend.click_policy = 'hide'




# #######################

# 解析/plan
topic_name_plan_hmi = "/iflytek/planning/hmi"  # 设置需要特定读取的topic名称
plan_hmi_data = get_topic_data(bag_data, topic_name_plan_hmi)
ldw_output_info= get_signal_data(plan_hmi_data, "ldw_output_info")
ldp_output_info= get_signal_data(plan_hmi_data, "ldp_output_info")
elk_output_info= get_signal_data(plan_hmi_data, "elk_output_info")
ldw_left_warning= get_signal_data(ldw_output_info, "ldw_left_warning")
ldw_right_warning= get_signal_data(ldw_output_info, "ldw_right_warning")
ldp_left_intervention_flag= get_signal_data(ldp_output_info, "ldp_left_intervention_flag")
ldp_right_intervention_flag= get_signal_data(ldp_output_info, "ldp_right_intervention_flag")
elk_left_intervention_flag= get_signal_data(elk_output_info, "elk_left_intervention_flag")
elk_right_intervention_flag= get_signal_data(elk_output_info, "elk_right_intervention_flag")
elk_state= get_signal_data(elk_output_info, "elk_state")
ldp_state= get_signal_data(ldp_output_info, "ldp_state")
ldw_state= get_signal_data(ldw_output_info, "ldw_state")

plan_hmi_time=list()
for i in range(len(plan_hmi_data)):
    plan_hmi_time.append((plan_hmi_data[i].msg_header.stamp-control_data[0].msg_header.stamp)/1000000)

planning_hmi_1 = bkp.figure(title="elk-info", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
planning_hmi_1.line(plan_hmi_time, ldp_left_intervention_flag,
           legend_label="ldp_left_intervention_flag", line_width=2, color='blue')
planning_hmi_1.line(plan_hmi_time, ldp_right_intervention_flag,
           legend_label="ldp_right_intervention_flag", line_width=2, color='blue')
planning_hmi_1.line(plan_hmi_time, elk_left_intervention_flag,
           legend_label="elk_left_intervention_flag", line_width=2, color='green')
planning_hmi_1.line(plan_hmi_time, elk_right_intervention_flag,
           legend_label="elk_right_intervention_flag", line_width=2, color='green')
planning_hmi_1.line(plan_hmi_time, elk_state,
           legend_label="elk_state", line_width=2, color='red')
planning_hmi_1.line(plan_hmi_time, ldp_state,
           legend_label="ldp_state", line_width=2, color='purple')
planning_hmi_1.line(plan_hmi_time, ldw_state,
           legend_label="ldw_state", line_width=2, color='black')
planning_hmi_1.legend.click_policy = 'hide'

topic_name_control = "/iflytek/localization/ego_pose"  # 设置需要特定读取的topic名称
loc_data = get_topic_data(bag_data, topic_name_control)
loc_time = list()
loc_time_gap = list()
for i in range(len(loc_data)):
    loc_time.append((loc_data[i].msg_header.stamp-control_data[0].msg_header.stamp)/1000000)
    if i == 0:
        loc_time_gap.append(0.0)
    else:
        loc_time_gap.append((loc_data[i].msg_header.stamp-loc_data[i-1].msg_header.stamp)/1000)
loc_1= bkp.figure(title="loc_time_gap", x_axis_label='x', y_axis_label='y',
                   width=500, height=500)
loc_1.line(loc_time, loc_time_gap,
           legend_label="loc_time_gap", line_width=2, color='blue')
loc_1.legend.click_policy = 'hide'
print(loc_time_gap)

bkp.show(column(
    row(fig_ctrl_1, fig_ctrl_2, fig_ctrl_3, fig_ctrl_4, fig_ctrl_5),
    row(fig_veh_service_1, fig_veh_service_2, fig_veh_service_3, fig_veh_service_4),
    row(fig_veh_service_5, fig_veh_service_6),
    row(adas_function_1, adas_function_2, adas_function_3 , adas_function_4),
    row(adas_function_5,adas_function_6,),
    row(fig_soc_state_1,fig_soc_state_2,planning_1,planning_2,planning_hmi_1),
    row(loc_1,hmi_inner_1)
))

print(type(plan_data))
