import copy
import math
import numpy as np
import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs, Arrow, NormalHead, Label
from bokeh.events import Tap
from contruct_scenario import construct_scenario
from lib.load_local_view_parking import *
from bokeh.events import Tap
from bokeh.models import Range1d
from bokeh.io import export_svgs
from bokeh.models import SingleIntervalTicker
from scipy.spatial import ConvexHull
from bokeh.io import curdoc
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')
from jupyter_pybind import perpendicular_slant_tail_in_with_json_py
from struct_msgs.msg import ParkingFusionSlot, PlanningOutput, UssPerceptInfo, GroundLinePerceptionInfo, FusionObjectsInfo, FusionOccupancyObjectsInfo, UssWaveInfo, ParkingFusionInfo, VehicleServiceOutputInfo, FuncStateMachine, IFLYLocalization, ControlOutput
display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()


fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=1200, height=800, match_aspect = True, aspect_scale=1)
# fig1.background_fill_color = "#E0E0E0"  # 你也可以用 '#D3D3D3'
fig1.x_range.flipped = False

fig1.outline_line_color = "black"
fig1.outline_line_width = 1.0  # 可以调整边框线条的宽度

# fig1.x_range = Range1d(start = -6.0, end = 14.0)
# fig1.y_range = Range1d(start = -3.0, end = 9.0)

fig1.xaxis.axis_label_text_font_size = '18pt'
fig1.xaxis.axis_label_text_font = 'Times New Roman'

fig1.yaxis.axis_label_text_font_size = '18pt'
fig1.yaxis.axis_label_text_font = 'Times New Roman'

fig1.xaxis.major_label_text_font_size = '18pt'  # 设置x轴字体大小
fig1.xaxis.major_label_text_font = 'Times New Roman'      # 设置字体类型

fig1.yaxis.major_label_text_font_size = '18pt'
fig1.yaxis.major_label_text_font = 'Times New Roman'

# fig1.xaxis.ticker = SingleIntervalTicker(interval = 4, num_minor_ticks=0)
# fig1.yaxis.ticker = SingleIntervalTicker(interval = 4, num_minor_ticks=0)

# fig1.xgrid.grid_line_color = None
# fig1.ygrid.grid_line_color = None


# 去除图形四周边框
# fig1.outline_line_color = None
# fig1.xaxis.visible = False
# fig1.yaxis.visible = False
# fig1.xaxis.major_label_text_font_size = '0pt'  # 设置字体大小
# fig1.yaxis.major_label_text_font_size = '0pt'


# 尝试确保图表内容比例一致
# aspect_ratio = (fig1.x_range.end - fig1.x_range.start) / (fig1.y_range.end - fig1.y_range.start)
# fig1.plot_height = int(fig1.plot_width / aspect_ratio)

data_car_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_fusion_limiter = ColumnDataSource(data = {'corner_point_x': [], 'corner_point_y': [],})
data_fusion_slot = ColumnDataSource(data = {'corner_point_x': [], 'corner_point_y': [],})
data_fusion_parking_id = ColumnDataSource(data = {'id':[], 'id_text_x':[], 'id_text_y':[]})
data_fusion_obj = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_uss_obj = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_gl_obj = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_planning_tune_complete = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})
data_simu_car_box_complete = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_planning_tune_cur_gear = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})
data_simu_car_box_cur_gear = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_planning_complete = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})
data_car_box_complete = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_virtual_obs_pos = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_planning_tune_substitute = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})
data_simu_car_box_substitute = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_planning_tune_substitute_selected = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})


fig1.patch('car_xn', 'car_yn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car_start_pos')
fig1.circle('x','y', source = data_car_start_pos, size=8, color='red', legend_label = 'car_start_pos')
fig1.text(x = 'id_text_x', y = 'id_text_y', text = 'id', source = data_fusion_parking_id, text_color='black', text_align='center', text_font_size='10pt',legend_label = 'fusion_parking_slot', visible = True)
fig1.multi_line('corner_point_x', 'corner_point_y', source = data_fusion_slot, line_width = 2, line_color = 'red', line_dash = 'solid', line_alpha = 0.6, legend_label = 'fusion_parking_slot', visible = True)
fig1.multi_line('corner_point_x', 'corner_point_y', source = data_fusion_limiter, line_width = 2, line_color = 'red', line_dash = 'solid', line_alpha = 0.6, legend_label = 'fusion_parking_slot', visible = True)
fig1.circle('x_vec','y_vec', source = data_fusion_obj, size=3, color='blue', legend_label = 'fusion_objects', visible = True)
fig1.circle('x_vec','y_vec', source = data_uss_obj, size=3, color='orange', legend_label = 'uss_objects', visible = True)
fig1.circle('x_vec','y_vec', source = data_gl_obj, size=3, color='black', legend_label = 'gl_objects', visible = True)
fig1.circle('plan_path_x', 'plan_path_y', source = data_planning_tune_complete, size=4, color='yellow', legend_label = 'sim_tuned_plan_complete')
fig1.line('plan_path_x', 'plan_path_y', source = data_planning_tune_complete, line_width = 6, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'sim_tuned_plan_complete')
fig1.patches('x_vec', 'y_vec', source = data_simu_car_box_complete, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sim_sampled_carbox_complete', visible = False)
fig1.circle('plan_path_x', 'plan_path_y', source = data_planning_tune_cur_gear, size=4, color='orange', legend_label = 'sim_tuned_plan_cur_gear')
fig1.line('plan_path_x', 'plan_path_y', source = data_planning_tune_cur_gear, line_width = 6, line_color = 'blue', line_dash = 'solid', line_alpha = 0.5, legend_label = 'sim_tuned_plan_cur_gear')
fig1.patches('x_vec', 'y_vec', source = data_simu_car_box_cur_gear, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "grey", line_width = 1, legend_label = 'sim_sampled_carbox_cur_gear', visible = False)
fig1.circle('plan_path_x', 'plan_path_y', source = data_planning_complete, size=4, color='black', legend_label = 'plan_complete')
fig1.line('plan_path_x', 'plan_path_y', source = data_planning_complete, line_width = 6, line_color = 'red', line_dash = 'solid', line_alpha = 0.5, legend_label = 'plan_complete')
fig1.patches('x_vec', 'y_vec', source = data_car_box_complete, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "cyan", line_width = 1, legend_label = 'sampled_carbox_complete', visible = False)
fig1.circle('x_vec','y_vec', source = data_virtual_obs_pos, size=8, color='orange', alpha = 0.1, legend_label = 'virtual_obs_pos', visible = True)
fig1.circle('plan_path_x', 'plan_path_y', source = data_planning_tune_substitute, size=2, color='yellow', legend_label = 'sim_tuned_plan_substitute')
fig1.multi_line('plan_path_x', 'plan_path_y', source = data_planning_tune_substitute, line_width = 4, line_color = 'green', line_dash = 'solid', line_alpha = 0.3, legend_label = 'sim_tuned_plan_substitute')
fig1.patches('x_vec', 'y_vec', source = data_simu_car_box_substitute, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "red", line_width = 1, legend_label = 'sim_sampled_carbox_substitute', visible = False)
fig1.circle('plan_path_x', 'plan_path_y', source = data_planning_tune_substitute_selected, size=2, color='grey', legend_label = 'sim_tuned_plan_substitute_selected')
fig1.line('plan_path_x', 'plan_path_y', source = data_planning_tune_substitute_selected, line_width = 4, line_color = 'cyan', line_dash = 'solid', line_alpha = 0.3, legend_label = 'sim_tuned_plan_substitute_selected')

coord_tf = coord_transformer()

source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=10, source=source, color='red', legend_label='measure tool')
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool')
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool')

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


fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

# Label, possible attributes are angle, angle_units, background_fill_alpha, background_fill_color, border_line_alpha, border_line_cap,
# border_line_color, border_line_dash, border_line_dash_offset, border_line_join, border_line_width, coordinates, group, js_event_callbacks,
# js_property_callbacks, level, name, render_mode, subscribed_events, syncable, tags, text, text_align, text_alpha, text_baseline, text_color,
# text_font, text_font_size, text_font_style, text_line_height, visible, x, x_offset, x_range_name, x_units, y, y_offset, y_range_name or y_units
x_rect = 15
y_rect = 650
rect_width = 150
rect_height = 50
text_label = Label(x=x_rect, y=y_rect,  # 这里的 x, y 值可以根据需要调整
                  text="",
                  text_color="red",
                  text_font_size="12pt",
                  x_units='screen',  # 用像素为单位
                  y_units='screen', border_line_color='black', border_line_width=2)   # 用像素为单位

fig1.add_layout(text_label)

perpendicular_slant_tail_in_with_json_py.Init()

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.vehicle_type_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "vehicle_type",min=0, max=2, value=2, step=1)
    self.trigger_plan_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "trigger_plan",min=0, max=1, value=0, step=1)
    self.force_mid_process_plan_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "force_mid_process_plan",min=0, max=2, value=0, step=1)
    self.selected_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "selected_id",min=0, max=20, value=0, step=1)
    self.substitute_path_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='50%'), description= "substitute_path_id",min=0, max=20, value=0, step=1)
    self.sample_ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='25%'), description= "sample_ds",min=0.02, max=2.0, value=0.1, step=0.02)
    self.car_inflation_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "car_inflation",min=0.0, max=0.30, value=0.0, step=0.01)
    self.data_json_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='50%'), description= "data_json_id",min=1, max=50, value=1, step=1)
    self.ego_offset_lon_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_offset_lon",min=-10, max=10, value=0.0, step=0.01)
    self.ego_offset_lat_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_offset_lat",min=-10, max=10, value=0.0, step=0.01)
    self.ego_offset_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_offset_heading",min=-180, max=180, value=0.0, step=0.2)
    self.is_path_optimization_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "path_optimization",min=0, max=1, value=0, step=1)
    self.is_cilqr_enable_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "cilqr_enable",min=0, max=1, value=1, step=1)
    self.is_complete_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "complete_path",min=0, max=1, value=0, step=1)


    ipywidgets.interact(slider_callback, vehicle_type = self.vehicle_type_slider,
                                         car_inflation = self.car_inflation_slider,
                                         sample_ds = self.sample_ds_slider,
                                         trigger_plan = self.trigger_plan_slider,
                                         force_mid_process_plan = self.force_mid_process_plan_slider,
                                         data_json_id = self.data_json_id_slider,
                                         selected_id = self.selected_id_slider,
                                         substitute_path_id = self.substitute_path_id_slider,
                                         ego_offset_lon = self.ego_offset_lon_slider,
                                         ego_offset_lat = self.ego_offset_lat_slider,
                                         ego_offset_heading = self.ego_offset_heading_slider,
                                         is_path_optimization = self.is_path_optimization_slider,
                                         is_cilqr_enable = self.is_cilqr_enable_slider,
                                         is_complete_path = self.is_complete_path_slider,
                                       )

### sliders callback
def slider_callback(vehicle_type, car_inflation, sample_ds, selected_id, substitute_path_id, trigger_plan, force_mid_process_plan, data_json_id, ego_offset_lon, ego_offset_lat, ego_offset_heading, is_path_optimization, is_cilqr_enable, is_complete_path):
  kwargs = locals()

  data_car_start_pos.data.update({'x': [],'y': [],})
  data_car.data.update({'car_xn': [], 'car_yn': [],})
  data_fusion_limiter.data.update({'corner_point_x': [], 'corner_point_y': [],})
  data_fusion_slot.data.update({'corner_point_x': [], 'corner_point_y': [],})
  data_fusion_parking_id.data.update({'id':[], 'id_text_x':[], 'id_text_y':[]})
  data_fusion_obj.data.update({'x_vec':[], 'y_vec':[]})
  data_uss_obj.data.update({'x_vec':[], 'y_vec':[]})
  data_gl_obj.data.update({'x_vec':[], 'y_vec':[]})
  data_planning_tune_complete.data.update({'plan_path_x':[],
                                                'plan_path_y':[],
                                                'plan_path_heading':[],})
  data_simu_car_box_complete.data.update({'x_vec':[], 'y_vec':[]})
  data_planning_tune_cur_gear.data.update({'plan_path_x':[],
                                                'plan_path_y':[],
                                                'plan_path_heading':[],})
  data_simu_car_box_cur_gear.data.update({'x_vec':[], 'y_vec':[]})
  data_planning_complete.data.update({'plan_path_x':[],
                                                'plan_path_y':[],
                                                'plan_path_heading':[],})
  data_car_box_complete.data.update({'x_vec':[], 'y_vec':[]})
  data_virtual_obs_pos.data.update({'x_vec':[], 'y_vec':[]})
  data_planning_tune_substitute.data.update({'plan_path_x':[],
                                                'plan_path_y':[],
                                                'plan_path_heading':[],})
  data_simu_car_box_substitute.data.update({'x_vec':[], 'y_vec':[]},)
  data_planning_tune_substitute_selected.data.update({'plan_path_x':[],
                                                'plan_path_y':[],
                                                'plan_path_heading':[],})

  if vehicle_type == 0:
    vehicle_type = JAC_S811
  elif vehicle_type == 1:
    vehicle_type = CHERY_T26
  elif vehicle_type == 2:
    vehicle_type = CHERY_E0X

  # 读取json
  folder_path  = "../scenario/geometry_tail_in/"
  file_name = "data_" + str(data_json_id) + ".json"
  file_path = os.path.join(folder_path, file_name)
  print("file_path = ", file_path)
  if not os.path.isfile(file_path):
    file_name = "data_1.json"
    file_path = os.path.join(folder_path, file_name)
  with open(file_path, "r") as json_file:
    loaded_data = json.load(json_file)

  # 更新仿真参数
  perpendicular_slant_tail_in_with_json_py.UpdateSimuParams(is_path_optimization, is_cilqr_enable, is_complete_path, force_mid_process_plan, sample_ds)

  # 读取定位信息
  loc_data = loaded_data["loc_pos"]
  ego_heading = loc_data[2] + ego_offset_heading / 57.3
  ego_x = loc_data[0] + ego_offset_lon * math.cos(ego_heading) - ego_offset_lat * math.sin(ego_heading)
  ego_y = loc_data[1] + ego_offset_lon * math.sin(ego_heading) + ego_offset_lat * math.cos(ego_heading)
  car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type, car_inflation)
  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_x, ego_y, ego_heading)
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
  data_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })
  data_car_start_pos.data.update({
    'x': [ego_x],
    'y': [ego_y],
  })
  perpendicular_slant_tail_in_with_json_py.UpdateLocalization([ego_x, ego_y, ego_heading])

  # 读取车位信息
  slot_data = loaded_data["fusion_slot"]
  slot_id_vec = []
  slot_type_vec = []
  corner_points_vec = []
  limit_vec = []
  for i in range(len(slot_data)):
    single_slot = slot_data[i]

    slot_id_vec.append(single_slot[0])
    slot_type_vec.append(single_slot[1])

    single_corner_points = []
    for j in range(len(single_slot[2])):
      single_corner_points.append([single_slot[2][j][0], single_slot[2][j][1]])
    corner_points_vec.append(single_corner_points)

    single_limit = []
    for j in range(len(single_slot[3])):
      single_limit.append([[single_slot[3][j][0][0], single_slot[3][j][0][1]], [single_slot[3][j][1][0], single_slot[3][j][1][1]]])
    limit_vec.append(single_limit)

  if selected_id == 0:
    sel_id = loaded_data["select_id"]
  else:
    sel_id = selected_id
  perpendicular_slant_tail_in_with_json_py.UpdateSlot(sel_id, slot_id_vec, slot_type_vec, corner_points_vec, limit_vec)

  slots_x_vec = []
  slots_y_vec = []
  limiters_x_vec = []
  limiters_y_vec = []
  id_vec = []
  id_text_x_vec = []
  id_text_y_vec = []
  for i in range(len(slot_id_vec)):
    single_slot_x_vec = []
    single_slot_y_vec = []
    single_corner_points = corner_points_vec[i]
    # 1. update slots corner points
    for j in range(len(single_corner_points)):
      corner_x_global = single_corner_points[j][0]
      corner_y_global = single_corner_points[j][1]
      single_slot_x_vec.append(corner_x_global)
      single_slot_y_vec.append(corner_y_global)

    slot_plot_x_vec = [single_slot_x_vec[0],single_slot_x_vec[2],single_slot_x_vec[3],single_slot_x_vec[1]]
    slot_plot_y_vec = [single_slot_y_vec[0],single_slot_y_vec[2],single_slot_y_vec[3],single_slot_y_vec[1]]

    slots_x_vec.append(slot_plot_x_vec)
    slots_y_vec.append(slot_plot_y_vec)

    # 2.update slots limiter points in same slot_plot_vec
    single_limiter_x_vec = []
    single_limiter_y_vec = []
    single_limit = limit_vec[i]
    if (len(single_limit) == 1):
      single_limiter_x_vec.append(single_limit[0][0][0])
      single_limiter_y_vec.append(single_limit[0][0][1])
      single_limiter_x_vec.append(single_limit[0][1][0])
      single_limiter_y_vec.append(single_limit[0][1][1])
    elif (len(single_limit) == 2):
      single_limiter_x_vec.append(single_limit[0][0][0])
      single_limiter_y_vec.append(single_limit[0][0][1])
      single_limiter_x_vec.append(single_limit[1][1][0])
      single_limiter_y_vec.append(single_limit[1][1][1])

    limiters_x_vec.append(single_limiter_x_vec)
    limiters_y_vec.append(single_limiter_y_vec)

    # 3. update slot ids' text with their position
    id_vec.append(slot_id_vec[i])
    id_text_x_vec.append((slot_plot_x_vec[0] + slot_plot_x_vec[1] + slot_plot_x_vec[2] + slot_plot_x_vec[3]) * 0.25)
    id_text_y_vec.append((slot_plot_y_vec[0] + slot_plot_y_vec[1] + slot_plot_y_vec[2] + slot_plot_y_vec[3]) * 0.25)

  data_fusion_parking_id.data.update({'id':id_vec, 'id_text_x':id_text_x_vec, 'id_text_y':id_text_y_vec,})
  data_fusion_slot.data.update({'corner_point_x': slots_x_vec, 'corner_point_y': slots_y_vec,})
  data_fusion_limiter.data.update(
    {'corner_point_x': limiters_x_vec,
     'corner_point_y': limiters_y_vec,
  })

  # 读取occ obs
  occ_data = loaded_data["fusion_obs"]
  perpendicular_slant_tail_in_with_json_py.UpdateFusionObs(occ_data)
  obs_x_vec = []
  obs_y_vec = []
  for i in range(len(occ_data)):
    single_occ_obs = occ_data[i]
    for j in range(len(single_occ_obs)):
      obs_x_vec.append(single_occ_obs[j][0])
      obs_y_vec.append(single_occ_obs[j][1])
  data_fusion_obj.data.update({'x_vec': obs_x_vec, 'y_vec': obs_y_vec,})

  # 读取ground line
  gl_data = loaded_data["gl_obs"]
  perpendicular_slant_tail_in_with_json_py.UpdateGroundLineObs(gl_data)
  obs_x_vec = []
  obs_y_vec = []
  for i in range(len(gl_data)):
    single_gl_obs = gl_data[i]
    for j in range(len(single_gl_obs)):
      obs_x_vec.append(single_gl_obs[j][0])
      obs_y_vec.append(single_gl_obs[j][1])
  data_gl_obj.data.update({'x_vec': obs_x_vec, 'y_vec': obs_y_vec,})

  # 读取 uss obs
  uss_pt_data = loaded_data["uss_obs"]
  perpendicular_slant_tail_in_with_json_py.UpdateUssPerceptionObs(uss_pt_data)
  obs_x_vec = []
  obs_y_vec = []
  for i in range(len(uss_pt_data)):
    single_uss_obs = uss_pt_data[i]
    for j in range(len(single_uss_obs)):
      obs_x_vec.append(single_uss_obs[j][0])
      obs_y_vec.append(single_uss_obs[j][1])
  data_uss_obj.data.update({'x_vec': obs_x_vec, 'y_vec': obs_y_vec,})

  perpendicular_slant_tail_in_with_json_py.UpdateStateMachine()
  complete_path_pt_vec = []
  cur_gear_path_pt_vec = []
  virtual_obs_vec = []
  substitute_path_pt_vec = []
  if (trigger_plan == 1):
    perpendicular_slant_tail_in_with_json_py.Update()
    complete_path_pt_vec = perpendicular_slant_tail_in_with_json_py.GetCompletePlanPath()
    cur_gear_path_pt_vec = perpendicular_slant_tail_in_with_json_py.GetCurrentGearPlanPath()
    virtual_obs_vec = perpendicular_slant_tail_in_with_json_py.GetObsVec()
    substitute_path_pt_vec = perpendicular_slant_tail_in_with_json_py.GetPerferredPlanPath()

  substitute_path_x_vec_vec, substitute_path_y_vec_vec, substitute_path_heading_vec_vec, substitute_path_lat_buffer_vec_vec = [], [], [], []
  for i in range(len(substitute_path_pt_vec)):
    substitute_path_x_vec, substitute_path_y_vec, substitute_path_heading_vec, substitute_path_lat_buffer_vec = [], [], [], []
    for j in range(len(substitute_path_pt_vec[i])):
      substitute_path_x_vec.append(substitute_path_pt_vec[i][j][0])
      substitute_path_y_vec.append(substitute_path_pt_vec[i][j][1])
      substitute_path_heading_vec.append(substitute_path_pt_vec[i][j][2])
      substitute_path_lat_buffer_vec.append(substitute_path_pt_vec[i][j][3])
    substitute_path_x_vec_vec.append(substitute_path_x_vec)
    substitute_path_y_vec_vec.append(substitute_path_y_vec)
    substitute_path_heading_vec_vec.append(substitute_path_heading_vec)
    substitute_path_lat_buffer_vec_vec.append(substitute_path_lat_buffer_vec)
  data_planning_tune_substitute.data.update({
    'plan_path_x': substitute_path_x_vec_vec,
    'plan_path_y': substitute_path_y_vec_vec,
    'plan_path_heading': substitute_path_heading_vec_vec})

  car_box_x_vec = []
  car_box_y_vec = []
  for i in range(len(substitute_path_x_vec_vec)):
    if (i != substitute_path_id):
      continue
    substitute_path_x_vec = substitute_path_x_vec_vec[i]
    substitute_path_y_vec = substitute_path_y_vec_vec[i]
    substitute_path_heading_vec = substitute_path_heading_vec_vec[i]
    substitute_path_lat_buffer_vec = substitute_path_lat_buffer_vec_vec[i]

    data_planning_tune_substitute_selected.data.update({
      'plan_path_x': substitute_path_x_vec,
      'plan_path_y': substitute_path_y_vec,
      'plan_path_heading': substitute_path_heading_vec
    })

    for k in range(len(substitute_path_x_vec)):
      if (k % 1 != 0):
        continue
      car_xn = []
      car_yn = []
      car_xb_temp, car_yb_temp, wheel_base = load_car_params_patch_parking(vehicle_type, substitute_path_lat_buffer_vec[k])
      for j in range(len(car_xb_temp)):
          tmp_x, tmp_y = local2global(car_xb_temp[j], car_yb_temp[j], substitute_path_x_vec[k], substitute_path_y_vec[k], substitute_path_heading_vec[k])
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)
      car_box_x_vec.append(car_xn)
      car_box_y_vec.append(car_yn)

  data_simu_car_box_substitute.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })


  print("virtual_obs_vec size = ", len(virtual_obs_vec))
  virtual_obs_x_vec, virtual_obs_y_vec = [], []
  for i in range(len(virtual_obs_vec)):
    virtual_obs_x_vec.append(virtual_obs_vec[i][0])
    virtual_obs_y_vec.append(virtual_obs_vec[i][1])
  print("virtual_obs_x_vec size = ", len(virtual_obs_x_vec))
  print("virtual_obs_y_vec size = ", len(virtual_obs_y_vec))
  data_virtual_obs_pos.data.update({'x_vec':virtual_obs_x_vec, 'y_vec':virtual_obs_y_vec})

  complete_path_x_vec, complete_path_y_vec, complete_path_heading_vec, complete_path_lat_buffer_vec = [], [], [], []
  for i in range(len(complete_path_pt_vec)):
    complete_path_x_vec.append(complete_path_pt_vec[i][0])
    complete_path_y_vec.append(complete_path_pt_vec[i][1])
    complete_path_heading_vec.append(complete_path_pt_vec[i][2])
    complete_path_lat_buffer_vec.append(complete_path_pt_vec[i][3])
  data_planning_tune_complete.data.update({
    'plan_path_x': complete_path_x_vec,
    'plan_path_y': complete_path_y_vec,
    'plan_path_heading': complete_path_heading_vec})

  car_box_x_vec = []
  car_box_y_vec = []
  for k in range(len(complete_path_x_vec)):
    if (k % 10 != 0):
      continue
    car_xn = []
    car_yn = []
    car_xb_temp, car_yb_temp, wheel_base = load_car_params_patch_parking(vehicle_type, complete_path_lat_buffer_vec[k])
    for i in range(len(car_xb_temp)):
        tmp_x, tmp_y = local2global(car_xb_temp[i], car_yb_temp[i], complete_path_x_vec[k], complete_path_y_vec[k], complete_path_heading_vec[k])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)

  data_simu_car_box_complete.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })

  cur_gear_path_x_vec, cur_gear_path_y_vec, cur_gear_path_heading_vec, cur_gear_path_lat_buffer_vec = [], [], [], []
  for i in range(len(cur_gear_path_pt_vec)):
    cur_gear_path_x_vec.append(cur_gear_path_pt_vec[i][0])
    cur_gear_path_y_vec.append(cur_gear_path_pt_vec[i][1])
    cur_gear_path_heading_vec.append(cur_gear_path_pt_vec[i][2])
    cur_gear_path_lat_buffer_vec.append(cur_gear_path_pt_vec[i][3])
  data_planning_tune_cur_gear.data.update({
    'plan_path_x': cur_gear_path_x_vec,
    'plan_path_y': cur_gear_path_y_vec,
    'plan_path_heading': cur_gear_path_heading_vec})

  car_box_x_vec = []
  car_box_y_vec = []
  for k in range(len(cur_gear_path_x_vec)):
    if (k % 5 != 0):
      continue
    car_xn = []
    car_yn = []
    car_xb_temp, car_yb_temp, wheel_base = load_car_params_patch_parking(vehicle_type, cur_gear_path_lat_buffer_vec[k])
    for i in range(len(car_xb_temp)):
        tmp_x, tmp_y = local2global(car_xb_temp[i], car_yb_temp[i], cur_gear_path_x_vec[k], cur_gear_path_y_vec[k], cur_gear_path_heading_vec[k])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)

  data_simu_car_box_cur_gear.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })

  # 读取origin plan trajectory
  plan_traj = loaded_data["plan_traj"]
  data_planning_complete.data.update({
    'plan_path_x': plan_traj[0],
    'plan_path_y': plan_traj[1],
    'plan_path_heading': plan_traj[2],})

  car_box_x_vec = []
  car_box_y_vec = []
  for k in range(len(plan_traj[0])):
    if (k % 5 != 0):
      continue
    car_xn = []
    car_yn = []
    car_xb_temp, car_yb_temp, wheel_base = load_car_params_patch_parking(vehicle_type, plan_traj[3][k])
    for i in range(len(car_xb_temp)):
        tmp_x, tmp_y = local2global(car_xb_temp[i], car_yb_temp[i], plan_traj[0][k], plan_traj[1][k], plan_traj[2][k])
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)

  data_car_box_complete.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })


  fig1.x_range = Range1d(start = ego_x - 10.0, end = ego_x + 10.0)
  fig1.y_range = Range1d(start = ego_y - 10.0, end = ego_y + 10.0)

  fig1.x_range.start = ego_x - 10
  fig1.x_range.end = ego_x + 10
  fig1.y_range.start = ego_y - 10
  fig1.y_range.end = ego_y + 10


  bag_path = loaded_data["bag_path"][30:80]
  html_path = loaded_data["html_path"][30:80]
  dis_string = ""
  dis_string = dis_string + "bag_path = " + str(bag_path) + "\n"
  text_label.text = dis_string

  push_notebook()

bkp.show(fig1, notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)



