import bokeh.plotting as bkp
from bokeh.layouts import row, column
from bokeh.resources import INLINE
import rosbag
# 设置bag文件的名称及路径
bag_file = "/data_cold/abu_zone/autoparse/chery_e0y_18047/trigger/20250610/20250610-16-52-07/data_collection_CHERY_E0Y_18047_EVENT_FILTER_2025-06-10-16-52-07_no_camera.bag"

# 设置html的名称及路径
result = bag_file.split('/')[-1]
html_file = result +"frame_time_diff.html"
# 生成一个html文件
bkp.output_file(html_file, mode = 'inline')


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
#get_topic_list(bag_file)
# 解析bag包,将所有信息存储在bag_data中
bag_data = rosbag.Bag(bag_file)

# 解析vehicle_service
topic_name_vehicle_service = "/iflytek/vehicle_service"  # 设置需要特定读取的topic名称
vehicle_service_data = get_topic_data(bag_data, topic_name_vehicle_service)
vehicle_service_time = list()
for i in range(len(vehicle_service_data)):
    vehicle_service_time.append(
        (vehicle_service_data[i].msg_header.stamp -
         vehicle_service_data[0].msg_header.stamp)/1000000)
vehicle_service_time_diff_ms = list()
for i in range(len(vehicle_service_time)):
    if i == 0:
        vehicle_service_time_diff_ms.append(0)
    else:
        vehicle_service_time_diff_ms.append((vehicle_service_time[i] - vehicle_service_time[i - 1])*1000)
fig1 = bkp.figure(title="vehicle_service", x_axis_label='x', y_axis_label='diff_ms', width=900, height=200)
fig1.circle(vehicle_service_time[1:-1], vehicle_service_time_diff_ms[1:-1], size=5, color='blue', alpha=0.5)

# 解析control
topic_name_control = "/iflytek/control/control_command"  # 设置需要特定读取的topic名称
control_data = get_topic_data(bag_data, topic_name_control)
control_time = list()
for i in range(len(control_data)):
    control_time.append(
        (control_data[i].msg_header.stamp -
         control_data[0].msg_header.stamp)/1000000)
control_time_diff_ms = list()
for i in range(len(control_time)):
    if i == 0:
        control_time_diff_ms.append(0)
    else:
        control_time_diff_ms.append((control_time[i] - control_time[i - 1])*1000)
fig2 = bkp.figure(title="control", x_axis_label='x', y_axis_label='diff_ms', width=900, height=200)
fig2.circle(control_time[1:-1], control_time_diff_ms[1:-1], size=5, color='blue', alpha=0.5)

# 解析perception_objects
topic_name_perception_objects = "/iflytek/camera_perception/objects"  # 设置需要特定读取的topic名称
perception_objects_data = get_topic_data(bag_data, topic_name_perception_objects)
perception_objects_time = list()
for i in range(len(perception_objects_data)):
    perception_objects_time.append(
        (perception_objects_data[i].msg_header.stamp -
         perception_objects_data[0].msg_header.stamp)/1000000)
perception_objects_time_diff_ms = list()
for i in range(len(perception_objects_time)):
    if i == 0:
        perception_objects_time_diff_ms.append(0)
    else:
        perception_objects_time_diff_ms.append((perception_objects_time[i] - perception_objects_time[i - 1])*1000)
fig3 = bkp.figure(title="perception_objects", x_axis_label='x', y_axis_label='diff_ms', width=900, height=200)
fig3.circle(perception_objects_time[1:-1], perception_objects_time_diff_ms[1:-1], size=5, color='blue', alpha=0.5)

# 解析perception_lane
topic_name_perception_lane = "/iflytek/camera_perception/lane_lines"  # 设置需要特定读取的topic名称
perception_lane_data = get_topic_data(bag_data, topic_name_perception_lane)
perception_lane_time = list()
for i in range(len(perception_lane_data)):
    perception_lane_time.append(
        (perception_lane_data[i].msg_header.stamp -
         perception_lane_data[0].msg_header.stamp)/1000000)
perception_lane_time_diff_ms = list()
for i in range(len(perception_lane_time)):
    if i == 0:
        perception_lane_time_diff_ms.append(0)
    else:
        perception_lane_time_diff_ms.append((perception_lane_time[i] - perception_lane_time[i - 1])*1000)
fig4 = bkp.figure(title="perception_lane", x_axis_label='x', y_axis_label='diff_ms', width=900, height=200)
fig4.circle(perception_lane_time[1:-1], perception_lane_time_diff_ms[1:-1], size=5, color='blue', alpha=0.5)

# 解析plan
topic_name_plan = "/iflytek/planning/plan"  # 设置需要特定读取的topic名称
plan_data = get_topic_data(bag_data, topic_name_plan)
plan_time = list()
for i in range(len(plan_data)):
    plan_time.append(
        (plan_data[i].msg_header.stamp -
         plan_data[0].msg_header.stamp)/1000000)
plan_time_diff_ms = list()
for i in range(len(plan_time)):
    if i == 0:
        plan_time_diff_ms.append(0)
    else:
        plan_time_diff_ms.append((plan_time[i] - plan_time[i - 1])*1000)
fig5 = bkp.figure(title="plan", x_axis_label='x', y_axis_label='diff_ms', width=900, height=200)
fig5.circle(plan_time[1:-1], plan_time_diff_ms[1:-1], size=5, color='blue', alpha=0.5)

# 解析plan_hmi
topic_name_plan_hmi = "/iflytek/planning/hmi"  # 设置需要特定读取的topic名称
plan_hmi_data = get_topic_data(bag_data, topic_name_plan_hmi)
plan_hmi_time = list()
for i in range(len(plan_hmi_data)):
    plan_hmi_time.append(
        (plan_hmi_data[i].msg_header.stamp -
         plan_hmi_data[0].msg_header.stamp)/1000000)
plan_hmi_time_diff_ms = list()
for i in range(len(plan_hmi_time)):
    if i == 0:
        plan_hmi_time_diff_ms.append(0)
    else:
        plan_hmi_time_diff_ms.append((plan_hmi_time[i] - plan_hmi_time[i - 1])*1000)
fig6 = bkp.figure(title="plan_hmi", x_axis_label='x', y_axis_label='diff_ms', width=900, height=200)
fig6.circle(plan_hmi_time[1:-1], plan_hmi_time_diff_ms[1:-1], size=5, color='blue', alpha=0.5)

# 解析fusion_objects
topic_name_fusion_objects = "/iflytek/fusion/objects"  # 设置需要特定读取的topic名称
fusion_objects_data = get_topic_data(bag_data, topic_name_fusion_objects)
fusion_objects_time = list()
for i in range(len(fusion_objects_data)):
    fusion_objects_time.append(
        (fusion_objects_data[i].msg_header.stamp -
         fusion_objects_data[0].msg_header.stamp)/1000000)
fusion_objects_time_diff_ms = list()
for i in range(len(fusion_objects_time)):
    if i == 0:
        fusion_objects_time_diff_ms.append(0)
    else:
        fusion_objects_time_diff_ms.append((fusion_objects_time[i] - fusion_objects_time[i - 1])*1000)
fig7 = bkp.figure(title="fusion_objects", x_axis_label='x', y_axis_label='diff_ms', width=900, height=200)
fig7.circle(fusion_objects_time[1:-1], fusion_objects_time_diff_ms[1:-1], size=5, color='blue', alpha=0.5)

# 解析fusion_road
topic_name_fusion_road = "/iflytek/fusion/road_fusion"  # 设置需要特定读取的topic名称
fusion_road_data = get_topic_data(bag_data, topic_name_fusion_road)
fusion_road_time = list()
for i in range(len(fusion_road_data)):
    fusion_road_time.append(
        (fusion_road_data[i].msg_header.stamp -
         fusion_road_data[0].msg_header.stamp)/1000000)
fusion_road_time_diff_ms = list()
for i in range(len(fusion_road_time)):
    if i == 0:
        fusion_road_time_diff_ms.append(0)
    else:
        fusion_road_time_diff_ms.append((fusion_road_time[i] - fusion_road_time[i - 1])*1000)
fig8 = bkp.figure(title="fusion_road", x_axis_label='x', y_axis_label='diff_ms', width=900, height=200)
fig8.circle(fusion_road_time[1:-1], fusion_road_time_diff_ms[1:-1], size=5, color='blue', alpha=0.5)


# 解析localization
topic_name_localization = "/iflytek/localization/egomotion"  # 设置需要特定读取的topic名称
local_data = get_topic_data(bag_data, topic_name_localization)
local_time = list()
for i in range(len(local_data)):
    local_time.append(
        (local_data[i].msg_header.stamp -
         local_data[0].msg_header.stamp)/1000000)
local_time_diff_ms = list()
for i in range(len(local_time)):
    if i == 0:
        local_time_diff_ms.append(0)
    else:
        local_time_diff_ms.append((local_time[i] - local_time[i - 1])*1000)
fig9 = bkp.figure(title="localization", x_axis_label='x', y_axis_label='diff_ms', width=900, height=200)
fig9.circle(local_time[1:-1], local_time_diff_ms[1:-1], size=5, color='blue', alpha=0.5)

# 解析function state machine
topic_name_fsm = "/iflytek/fsm/soc_state"  # 设置需要特定读取的topic名称
fsm_data = get_topic_data(bag_data, topic_name_fsm)
fsm_time = list()
for i in range(len(fsm_data)):
    fsm_time.append(
        (fsm_data[i].msg_header.stamp -
         fsm_data[0].msg_header.stamp)/1000000)
fsm_time_diff_ms = list()
for i in range(len(fsm_time)):
    if i == 0:
        fsm_time_diff_ms.append(0)
    else:
        fsm_time_diff_ms.append((fsm_time[i] - fsm_time[i - 1])*1000)
fig10 = bkp.figure(title="function state machine", x_axis_label='x', y_axis_label='diff_ms', width=900, height=200)
fig10.circle(fsm_time[1:-1], fsm_time_diff_ms[1:-1], size=5, color='blue', alpha=0.5)

bkp.show(column(
    row(fig1, fig2),
    row(fig3, fig4),
    row(fig5, fig6),
    row(fig7, fig8),
    row(fig9, fig10)
))

