import re
import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs, Arrow, NormalHead, Label
from bokeh.events import Tap
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = '/data_cold/abu_zone/autoparse/chery_e0y_18047/trigger/20250102/20250102-19-24-00/park_in_data_collection_CHERY_E0Y_18047_ALL_FILTER_2025-01-02-19-24-00_no_camera.bag'
bag_path = '/data_cold/abu_zone/autoparse/chery_e0y_20267/trigger/20250327/20250327-11-32-26/park_in_data_collection_CHERY_E0Y_20267_ALL_FILTER_2025-03-27-11-32-27_no_camera.bag'

frame_dt = 0.1 # sec
plot_ctrl_flag = True
cur_pos = [0.0, 0.0]

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, True)
max_time = bag_loader.load_all_data()
print("max_time = ", max_time)
fig1, local_view_data = load_local_view_figure_parking()

source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=10, source=source, color='red', legend_label='measure tool')
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool')
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool')

data_obs_slm_filtered = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=2, source=data_obs_slm_filtered, color='blue', legend_label='obs after filtered by slm')
# Define the JavaScript callback code
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

# Create a CustomJS callback with the defined code
callback = CustomJS(args=dict(source=source, line_source=line_source, text_source=text_source), code=callback_code)

# Attach the callback to the Tap event on the plot
fig1.js_on_event(Tap, callback)

if plot_ctrl_flag:
  fig2, fig3, fig4, fig5, fig6, fig7, data_ctrl_debug_table = load_local_view_figure_parking_ctrl(bag_loader, local_view_data, max_time, 0.02)

def get_next_filename(folder):
  # 获取文件夹中的所有文件
  files = os.listdir(folder)
  # 记录最高的编号
  max_number = 0

  # 遍历文件名，寻找以 data_ 开头且以 .json 结尾的文件
  for file in files:
    if file.startswith("data_") and file.endswith(".json"):
      try:
        # 从文件名中提取编号，并更新最大编号
        number = int(file[len("data_"):-len(".json")])
        if number > max_number:
          max_number = number
      except ValueError:
        pass  # 如果转换失败，则跳过该文件

  # 生成下一个文件名
  next_number = max_number + 1
  next_filename = f"data_{next_number}.json"
  return next_filename

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    self.vehicle_type = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "vehicle_type",min=0, max=2, value=2, step=1)
    self.car_inflation = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "car_inflation",min=0.0, max=0.15, value=0.0, step=0.01)
    self.save_data = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "save_data",min=0, max=1, value=0, step=1)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                         vehicle_type = self.vehicle_type,
                                         car_inflation = self.car_inflation,
                                         save_data = self.save_data)


### sliders callback
def slider_callback(bag_time, vehicle_type, car_inflation, save_data):
  kwargs = locals()

  if vehicle_type == 0:
    vehicle_type = 'JAC_S811'
  elif vehicle_type == 1:
    vehicle_type = 'CHERY_T26'
  elif vehicle_type == 2:
    vehicle_type = 'CHERY_E0X'

  update_local_view_data_parking(fig1, bag_loader, bag_time, vehicle_type, car_inflation, local_view_data, plot_ctrl_flag)
  # print("bag_time:", bag_time)
  index_map = bag_loader.get_msg_index(bag_time)
  # print("index_map:", index_map)

  if bag_loader.loc_msg['enable'] == True:
    loc_msg = bag_loader.loc_msg['data'][index_map['loc_msg_idx']]

    velocity = math.hypot(loc_msg.velocity.velocity_boot.vx,loc_msg.velocity.velocity_boot.vy)
    print("local_vel = ", velocity)
    temp_pos = [loc_msg.position.position_boot.x, loc_msg.position.position_boot.y]
    global cur_pos
    print("local_move_dist = ",  math.hypot(temp_pos[0] - cur_pos[0], temp_pos[1] - cur_pos[1]))
    cur_pos = temp_pos


  if bag_loader.plan_msg['enable'] == True:
    plan_msg = bag_loader.plan_msg['data'][index_map['plan_msg_idx']]
    # print("plan_msg = ", plan_msg.trajectory.trajectory_points)
    print("plan_release_slots_id = ", plan_msg.successful_slot_info_list)
    print("vel_tar = ", plan_msg.trajectory.target_reference.target_velocity)

  if bag_loader.soc_state_msg['enable'] == True:
    soc_state_msg = bag_loader.soc_state_msg['data'][index_map['soc_state_msg_idx']]
    # print("plan_msg = ", plan_msg.trajectory.trajectory_points)
    # print("soc_state_msg = ", soc_state_msg.parking_req)

  if bag_loader.plan_debug_msg['enable'] == True:
    planning_json = bag_loader.plan_debug_msg['json'][index_map['plan_debug_msg_idx']]
    print("remain_dist = ", planning_json['remain_dist'])
    print("remain_dist_uss =", planning_json['remain_dist_uss'])
    # print("path plan time =", planning_json['path_plan_time_ms'])

  if bag_loader.fus_parking_msg['enable'] == True:
    fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]
    print("selected slot id = ", fus_parking_msg.select_slot_id)
    # for i in range(len(fus_parking_msg.parking_fusion_slot_lists)):
    #   slot_info = fus_parking_msg.parking_fusion_slot_lists[i]
    #   print("slot_id = ", slot_info.id, "  slot_type = ", slot_info.type,
    #         "slot_allow_parking = ", slot_info.allow_parking, "slot_fusion_source = ", slot_info.fusion_source,
    #         "slot_slot_side = ", slot_info.slot_side, "slot_uss_id = ", slot_info.uss_id,
    #         )

  if bag_loader.uss_percept_msg['enable'] == True:
    uss_percept_msg = bag_loader.uss_percept_msg['data'][index_map['uss_percept_msg_idx']]
  #print("fus_parking_msg",fus_parking_msg.parking_fusion_slot_lists)

  if bag_loader.fus_occupancy_objects_msg['enable'] == True:
    fus_occupancy_objects_msg = bag_loader.fus_occupancy_objects_msg['data'][index_map['fus_occupancy_objects_msg_idx']]

  if bag_loader.fus_ground_line_msg['enable'] == True:
    fus_ground_line_msg = bag_loader.fus_ground_line_msg['data'][index_map['fus_ground_line_msg_idx']]

  if bag_loader.fus_parking_msg['enable'] == True:
    fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]

  if save_data:
    print("save_data")

    fusion_obs = []
    if bag_loader.fus_occupancy_objects_msg['enable'] == True:
      for i in range(fus_occupancy_objects_msg.fusion_object_size):
        single_fus_obs = []
        origin_single_fus_obs = fus_occupancy_objects_msg.fusion_object[i].additional_occupancy_info
        for j in range(origin_single_fus_obs.polygon_points_size):
          single_fus_obs.append([origin_single_fus_obs.polygon_points[j].x, origin_single_fus_obs.polygon_points[j].y])
        fusion_obs.append(single_fus_obs)

    uss_obs = []
    if bag_loader.uss_percept_msg['enable'] == True:
      for i in range(4):
        single_uss_obs = []
        origin_single_uss_obs = uss_percept_msg.out_line_dataori[i]
        for j in range(origin_single_uss_obs.obj_pt_cnt):
          single_uss_obs.append([origin_single_uss_obs.obj_pt_global[j].x, origin_single_uss_obs.obj_pt_global[j].y])
        uss_obs.append(single_uss_obs)

    gl_obs = []
    if bag_loader.fus_ground_line_msg['enable'] == True:
      for i in range(fus_ground_line_msg.groundline_size):
        single_gl_obs = []
        origin_single_gl_obs = fus_ground_line_msg.groundline[i]
        for j in range(origin_single_gl_obs.groundline_point_size):
          single_gl_obs.append([origin_single_gl_obs.groundline_point[j].x, origin_single_gl_obs.groundline_point[j].y])
        gl_obs.append(single_gl_obs)

    fusion_slot = []
    if bag_loader.fus_parking_msg['enable'] == True:
      for i in range(fus_parking_msg.parking_fusion_slot_lists_size):
        single_fusion_slot = []
        origin_single_fusion_slot = fus_parking_msg.parking_fusion_slot_lists[i]
        single_fusion_slot.append(origin_single_fusion_slot.id)
        single_fusion_slot.append(origin_single_fusion_slot.type)
        corner_points = origin_single_fusion_slot.corner_points
        single_fusion_slot.append([[corner_points[0].x, corner_points[0].y], [corner_points[1].x, corner_points[1].y],
                                  [corner_points[2].x, corner_points[2].y], [corner_points[3].x, corner_points[3].y]])

        limiter = []
        for j in range(origin_single_fusion_slot.limiters_size):
          limiter.append([[origin_single_fusion_slot.limiters[j].end_points[0].x, origin_single_fusion_slot.limiters[j].end_points[0].y],
                          [origin_single_fusion_slot.limiters[j].end_points[1].x, origin_single_fusion_slot.limiters[j].end_points[1].y]])
        single_fusion_slot.append(limiter)
        fusion_slot.append(single_fusion_slot)

    plan_traj_x_vec = planning_json["plan_traj_x"]
    plan_traj_y_vec = planning_json["plan_traj_y"]
    plan_traj_heading_vec = planning_json["plan_traj_heading"]
    plan_traj_lat_buffert_vec = planning_json["plan_traj_lat_buffer"]

    # bag_path = '/data_cold/abu_zone/autoparse/chery_e0y_20267/trigger/20241225/20241225-20-18-54/park_in_data_collection_CHERY_E0Y_20267_ALL_FILTER_2024-12-25-20-18-55_no_camera.bag'
    # http://localhost:2799/view/chery_e0y_20267/trigger/20241225/20241225-20-10-36/park_in_data_collection_CHERY_E0Y_20267_ALL_FILTER_2024-12-25-20-10-36_no_camera.bag.apa.html
    # http://10.103.227.10/html/20241218/park_in_data_collection_CHERY_E0Y_20267_ALL_FILTER_2024-12-18-16-10-31_no_camera.bag.apa.html
    html_path = "http://10.103.227.10/html/" + bag_path[54:63] + bag_path[81:] + ".apa.html"
    print("html_path = ", html_path)
    data = {"bag_path": bag_path,
            "html_path": html_path,
            "plan_traj":[plan_traj_x_vec, plan_traj_y_vec, plan_traj_heading_vec, plan_traj_lat_buffert_vec],
            "fusion_obs": fusion_obs,
            "uss_obs": uss_obs,
            "gl_obs": gl_obs,
            "select_id": fus_parking_msg.select_slot_id,
            "fusion_slot": fusion_slot,
            "loc_pos": [loc_msg.position.position_boot.x, loc_msg.position.position_boot.y, loc_msg.orientation.euler_boot.yaw]}
    folder_path  = "../scenario/geometry_tail_in/"
    if not os.path.exists(folder_path):
      os.makedirs(folder_path)
    file_name = get_next_filename(folder_path)
    print("file_name = ", file_name)
    file_path =  os.path.join(folder_path, file_name)
    with open(file_path, "w") as json_file:
      json.dump(data, json_file)

  push_notebook()

if plot_ctrl_flag:
  bkp.show(row(fig1, column(fig2, fig3, fig4, fig5), column(fig6, fig7, data_ctrl_debug_table)), notebook_handle=True)
else:
  bkp.show(fig1, notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)
