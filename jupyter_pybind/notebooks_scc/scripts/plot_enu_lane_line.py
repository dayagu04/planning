import sys, os
# import matplotlib.pyplot as plt
from bokeh.plotting import figure, show
from bokeh.models import ColumnDataSource, LabelSet
from bokeh.resources import INLINE
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_local_view import *
from lib.load_lat_plan import *
from lib.load_ros_bag import LoadRosbag

sys.path.append('../..')
sys.path.append('../../../')
display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)
bag_path = "/share//data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20240625/20240625-16-03-07/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-06-25-16-03-07_no_camera.bag"

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()

vis_distance = 100
def load_lane_center_lines(road_msg, is_enu_to_car = False, loc_msg = None, g_is_display_enu = False):
  line_info_list = []
  reference_line_msg = road_msg.reference_line_msg
  reference_line_msg_size = road_msg.reference_line_msg_size
  default_line_x, default_line_y = gen_line(0,0,0,0,0,0)
  for i in range(reference_line_msg_size):
    lane_info = {'line_x_vec':[], 'line_y_vec':[], 'relative_id':[],'type':[], 'line_s_vec':[], 'curvature_vec':[]}
    if i< reference_line_msg_size:
      lane = reference_line_msg[i]
      if lane.relative_id != 0:
        continue
      virtual_lane_refline_points = lane.lane_reference_line.virtual_lane_refline_points
      virtual_lane_refline_points_size = lane.lane_reference_line.virtual_lane_refline_points_size
      line_x = []
      line_y = []
      line_curvature = []
      line_s = []
      if g_is_display_enu:
        line_x = [virtual_lane_refline_points[j].enu_point.x for j in range(virtual_lane_refline_points_size)]
        line_y = [virtual_lane_refline_points[j].enu_point.y for j in range(virtual_lane_refline_points_size)]
        if loc_msg != None: # 长时轨迹
          cur_pos_xn = loc_msg.position.position_boot.x
          cur_pos_yn = loc_msg.position.position_boot.y

          min_distance = 99999
          min_distance_index = 0
          max_distance_index = len(line_x) - 1
          # print(line_x)
          # print(cur_pos_xn)
          for ii in range(1, len(line_x) - 2):
            last_distance = (cur_pos_xn - line_x[ii - 1]) * (cur_pos_xn - line_x[ii - 1]) + (cur_pos_yn - line_y[ii - 1]) * (cur_pos_yn - line_y[ii - 1])
            cur_distance = (cur_pos_xn - line_x[ii]) * (cur_pos_xn - line_x[ii]) + (cur_pos_yn - line_y[ii]) * (cur_pos_yn - line_y[ii])
            next_distance = (cur_pos_xn - line_x[ii + 1]) * (cur_pos_xn - line_x[ii + 1]) + (cur_pos_yn - line_y[ii + 1]) * (cur_pos_yn - line_y[ii + 1])
            # print(last_distance)
            # print(cur_distance)
            # print(next_distance)
            if (cur_distance <= last_distance and cur_distance <= next_distance):
              min_distance_index = ii
              break
          for ii in range(min_distance_index, len(line_x) - 1):
            cur_distance = (cur_pos_xn - line_x[ii]) * (cur_pos_xn - line_x[ii]) + (cur_pos_yn - line_y[ii]) * (cur_pos_yn - line_y[ii])
            if cur_distance > vis_distance * vis_distance:
              max_distance_index = ii
              break
          # print(min_distance_index)
          # print(max_distance_index)
          line_x = line_x[min_distance_index:max_distance_index]
          line_y = line_y[min_distance_index:max_distance_index]
          # print(len(line_x))
      else:
        if is_enu_to_car:
          coord_tf = coord_transformer()
          if loc_msg != None: # 长时轨迹
            cur_pos_xn = loc_msg.position.position_boot.x
            cur_pos_yn = loc_msg.position.position_boot.y
            cur_yaw = loc_msg.orientation.euler_boot.yaw
            coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
            line_x = [virtual_lane_refline_points[j].enu_point.x for j in range(virtual_lane_refline_points_size)]
            line_y = [virtual_lane_refline_points[j].enu_point.y for j in range(virtual_lane_refline_points_size)]
            line_x, line_y = coord_tf.global_to_local(line_x, line_y)
            """ for virtual_lane_refline_point in virtual_lane_refline_points:
              car_point_x, car_point_y = coord_tf.global_to_local([virtual_lane_refline_point.enu_point.x], [virtual_lane_refline_point.enu_point.y])
              line_x.append(car_point_x[0])
              line_y.append(car_point_y[0]) """
          else:
            line_x = default_line_x
            line_y = default_line_y
        else:
          line_x = [virtual_lane_refline_points[j].car_point.x for j in range(virtual_lane_refline_points_size)]
          line_y = [virtual_lane_refline_points[j].car_point.y for j in range(virtual_lane_refline_points_size)]

      line_s = [virtual_lane_refline_points[j].s for j in range(virtual_lane_refline_points_size)]
      line_curvature = [max(min(1.0 / (virtual_lane_refline_points[j].curvature + 1e-6), 10000.0), -10000.0) for j in range(virtual_lane_refline_points_size)]

      lane_info['line_x_vec'] = line_x
      lane_info['line_y_vec'] = line_y
      lane_info['relative_id'] = lane.relative_id
      lane_info['type'] = 0
      lane_info['line_s_vec'] = line_s
      lane_info['curvature_vec'] = line_curvature

      line_info_list.append(lane_info)
    else:
      lane_info['line_x_vec'] = default_line_x
      lane_info['line_y_vec'] = default_line_y
      lane_info['relative_id'] = 1000
      lane_info['type'] = 0
      lane_info['line_s_vec'] = 0
      lane_info['curvature_vec'] = 0
      line_info_list.append(lane_info)

  return line_info_list

frame_dt = 0.1
is_new_loc = False
fusion_road_timestamps = []
plan_debug_ts =[]
plan_debug_timestamps = []
localization_timestamps =[]
loc_msgs = []
fusion_road_msgs = []
# for i, plan_debug in enumerate(bag_loader.plan_debug_msg['data']):
#   t = bag_loader.plan_debug_msg["t"][i]
#   plan_debug_ts.append(t)
#   plan_debug_timestamps.append(bag_loader.plan_debug_msg["timestamp"][i])
#   input_topic_timestamp = plan_debug.input_topic_timestamp
#   #print(input_topic_timestamp)
#   fusion_object_timestamp = input_topic_timestamp.fusion_object
#   fusion_road_timestamp = input_topic_timestamp.fusion_road
#   if is_new_loc:
#     localization_timestamp = input_topic_timestamp.localization
#     #localization_timestamp = input_topic_timestamp.localization_estimate
#   else :
#     if is_bag_main:
#       localization_timestamp = input_topic_timestamp.localization_estimate #main分支录制的包
#     else:
#       localization_timestamp = input_topic_timestamp.localization # main分支之前录得包

#   # prediction_timestamp = input_topic_timestamp.prediction
#   # vehicle_service_timestamp = input_topic_timestamp.vehicle_service
#   # control_output_timestamp = input_topic_timestamp.control_output
#   # ehr_parking_map_timestamp = input_topic_timestamp.ehr_parking_map
#   # ground_line_timestamp = input_topic_timestamp.ground_line
#   # fusion_object_timestamps.append(fusion_object_timestamp)
#   loc_msg = find_nearest(bag_loader.loc_msg, localization_timestamp)
#   fus_msg = find_nearest(bag_loader.fus_msg, fusion_road_timestamp)
#   loc_msgs.append(loc_msg)
#   fusion_road_msgs.append(fus_msg)
#   fusion_road_timestamps.append(fusion_road_timestamp)
#   localization_timestamps.append(localization_timestamp)

  #print("len of loc is " + str(len(localization_timestamps)))
  # prediction_timestamps.append(prediction_timestamp)
  # vehicle_service_timestamps.append(vehicle_service_timestamp)
  # control_output_timestamps.append(control_output_timestamp)
  # ehr_parking_map_timestamps.append(ehr_parking_map_timestamp)
  # ground_line_timestamps.append(ground_line_timestamp)

# localization_timestamps = np.array(localization_timestamps)
# class LocalViewSlider:
#   def __init__(self,  slider_callback):
#     self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='100%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
#     ipywidgets.interact(slider_callback, bag_time = self.time_slider)

# def update_local_view_data(fig1, bag_loader, bag_time, local_view_data):
#   plan_debug_msg = find_nearest(bag_loader.plan_debug_msg, bag_time)

#   input_topic_timestamp = plan_debug_msg.input_topic_timestamp
#   localization_timestamp = input_topic_timestamp.localization_estimate
#   loc_msg = find(bag_loader.loc_msg, localization_timestamp)
#   cur_pos_xn = loc_msg.position.position_boot.x
#   cur_pos_yn = loc_msg.position.position_boot.y

#   localization_index = np.where(localization_timestamps, localization_timestamp)[0]

#   for i in range(localization_index, 0, -1):
#     loc_msg_tmp = loc_msgs[i]
#     pos_xn = loc_msg_tmp.position.position_boot.x
#     pos_yn = loc_msg_tmp.position.position_boot.y
#     if ((cur_pos_xn - pos_xn) * (cur_pos_xn - pos_xn) + (cur_pos_yn - pos_yn) * (cur_pos_yn - pos_yn)) > 40 * 40:
#       break
#     line_info_list = load_lane_center_lines(fusion_road_msgs[i], False, None, True)
#     for line in line_info_list:
#       if line['relative_id'] == 0:
#         p.line('x', 'y', source=ColumnDataSource(data=dict(x=line['line_x_vec'], y=line['line_y_vec'])), legend_label="Line " + str(i+1))

#   pass
p = figure(title='Scatter Plot', x_axis_label='Y', y_axis_label='X', width=1500, height=1500, match_aspect = True, aspect_scale=1)
p.x_range.flipped = True
# ### sliders callback
# def slider_callback(bag_time):
#   kwargs = locals()
#   update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
#   # plan_msg_idx = 0
#   # if bag_loader.plan_msg['enable'] == True:
#   #   while bag_loader.plan_msg['t'][plan_msg_idx] <= bag_time and plan_msg_idx < (len(bag_loader.plan_msg['t'])-2):
#   #       plan_msg_idx = plan_msg_idx + 1

#   # bag_loader.plan_msg['data'][plan_msg_idx]

#   push_notebook()

# data = {
#     'x': [1, 2, 3, 4, 5],
#     'y': [2, 4, 6, 8, 10]
# }
# source = ColumnDataSource(data)

# ego_xn, ego_yn = [], []
# if bag_loader.loc_msg['enable'] == True:
#   for i in range(len(bag_loader.loc_msg['data'])):
#     # if (i % 10 != 0): # 下采样 10
#     #   continue
#     ego_xn.append(bag_loader.loc_msg['data'][i].position.position_boot.x)
#     ego_yn.append(bag_loader.loc_msg['data'][i].position.position_boot.y)
# p.line(ego_xn, ego_yn, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')

data_center_line_0 = ColumnDataSource(data = {'center_line_0_y':[], 'center_line_0_x':[]})

linx_x = []
linx_y = []
data_rdg_obj = ColumnDataSource(data = {'pos_y':[], 'pos_x':[],
                                        'obs_label':[]})
# for x, y, text, color, font_size in zip(x_coords, y_coords, texts, colors, font_sizes):
#     print(x)
#     p.text(x=x, y=y, text=text, text_font_size=font_size, text_color=color)
index = 0

pos_xs = []
pos_ys = []
labels = []
line_xs = []
line_ys = []
ego_xn, ego_yn, ego_labels = [], [], []
for i,road_msg in enumerate(bag_loader.road_msg['data']):
  # if i > 10:
  #   break
  loc_msg = find_nearest(bag_loader.loc_msg, bag_loader.road_msg['t'][i])
  ego_xn.append(loc_msg.position.position_boot.x)
  ego_yn.append(loc_msg.position.position_boot.y)
  ego_labels.append(str(i))
  line_info_list = load_lane_center_lines(road_msg, False, loc_msg, True)
  for line in line_info_list:
    if line['relative_id'] == 0:
      # print(line['line_x_vec'])
      # print(line['line_y_vec'])
      # p.line('x', 'y', source=ColumnDataSource(data=dict(x=line['line_x_vec'], y=line['line_y_vec'])), legend_label="Line " + str(i+1))

      text_x = line['line_x_vec']
      text_y = line['line_y_vec']
      line_xs.append(line['line_x_vec'])
      line_ys.append(line['line_y_vec'])
      pos_xs = pos_xs + line['line_x_vec']
      pos_ys = pos_ys + line['line_y_vec']
      labels = labels + [str(i)] * len(line['line_x_vec'])
      # for x_, y_, text, color, font_size in zip(x_coords, y_coords, texts, colors, font_sizes):
      #   p.text(x=x_, y=y_, text=text, text_font_size=font_size, text_color=color)
p.line(ego_yn, ego_xn, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'ego_pos')
p.text(ego_yn, ego_xn, ego_labels , text_color="green", text_align="center", text_font_size="10pt", legend_label = 'ego_pos_info',visible = True)
p.multi_line(line_ys, line_xs, line_width = 1, line_color = 'lightgrey', line_dash = 'solid',legend_label = 'line', visible = True)
p.text(pos_ys, pos_xs, labels , text_color="red", text_align="center", text_font_size="10pt", legend_label = 'line_info',visible = True)
      # for x, y,z in zip(text_x, text_y, text_y):
      #   print(x)
      #   p.text(x=int(x), y=int(y), text=str(index) +'hhh', text_font_size="20pt", text_color="green")
      #   index += 1
      # p.text(x='x', y='y', text=str(i), source=ColumnDataSource(data={'x':[text_x], 'y':[text_y]}), text_font="CustomFont", text_font_size="12pt")
      # labels = LabelSet(x='x', y='y', text=str(i), x_offset=0, y_offset=0, source=ColumnDataSource(data={'x':[text_x], 'y':[text_y]}))
      # p.add_layout(labels)

      # p.scatter(x='x', y='y', source=ColumnDataSource(data=dict(x=line['line_x_vec'], y=line['line_y_vec'])), size=0.5, color="blue", alpha=1)
      # p.scatter(line['line_x_vec'], line['line_y_vec'])
# plt.show()
# p.circle(x='center_line_0_x', y='center_line_0_y', source=data_center_line_0, size=1, color='blue', fill_color='white')
# data_center_line_0.data.update({
#   'center_line_{}_x'.format(0): linx_x,
#   'center_line_{}_y'.format(0): linx_y,
# })
# p.circle(x='center_line_0_x', y='center_line_0_y', source=data_center_line_0, size=1, color='blue', fill_color='white')
# 显示图形
p.legend.click_policy = 'hide'
p.toolbar.active_scroll = p.select_one(WheelZoomTool)
bkp.show(row(p), notebook_handle=True)
push_notebook()
# show(p)
