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
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')
from jupyter_pybind import perpendicular_slant_tail_in_py

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

coord_tf = coord_transformer()

data_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_target_pos = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_target_line = ColumnDataSource(data = {'y':[], 'x':[]})
data_car_safe_pos = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_safe_line = ColumnDataSource(data = {'y':[], 'x':[]})
data_car_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_car_end_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_safe_circle_tang_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_safe_circle_tang_line = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_slot = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_rectangle_slot = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_simu_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_pt_inside_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_obs_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_planning_tune_complete = ColumnDataSource(data = {'plan_path_x':[],
                                              'plan_path_y':[],
                                              'plan_path_heading':[],})

data_obs_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})

data_obs_column = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})

data_obs_wall = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})

fig1 = bkp.figure(width=1200, height=800, match_aspect = True, aspect_scale=1)
# fig1.background_fill_color = "#E0E0E0"  # 你也可以用 '#D3D3D3'
fig1.x_range.flipped = False

fig1.outline_line_color = "black"
fig1.outline_line_width = 1.0  # 可以调整边框线条的宽度

fig1.x_range = Range1d(start = -6.0, end = 14.0)
fig1.y_range = Range1d(start = -3.0, end = 9.0)

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
aspect_ratio = (fig1.x_range.end - fig1.x_range.start) / (fig1.y_range.end - fig1.y_range.start)
fig1.plot_height = int(fig1.plot_width / aspect_ratio)
fig1.patch('car_xn', 'car_yn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car_start_pos')
fig1.patch('car_xn', 'car_yn', source = data_car_target_pos, fill_color = "blue", line_color = "red", line_width = 1, line_alpha = 0.5, legend_label = 'car_end_pos')
fig1.patch('car_xn', 'car_yn', source = data_car_safe_pos, fill_color = "orange", line_color = "red", line_width = 1, line_alpha = 0.5, legend_label = 'car_safe_pos', visible = False)
fig1.line('x', 'y', source = data_car_target_line, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.8, legend_label = 'car_end_pos')
fig1.line('x', 'y', source = data_car_safe_line, line_width = 3.0, line_color = 'black', line_dash = 'solid', line_alpha = 0.8, legend_label = 'car_safe_pos', visible = False)
fig1.patches('x_vec', 'y_vec', source = data_simu_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sim_sampled_carbox', visible = False)

fig1.circle('x','y', source = data_pt_inside_pos, size=8, color='green', legend_label = 'pt_inside_pos', visible = False)

fig1.circle('x','y', source = data_obs_pos, size=8, color='orange', alpha = 0.1, legend_label = 'obs_pos', visible = True)


fig1.circle('x','y', source = data_car_start_pos, size=8, color='red', legend_label = 'car_start_pos')
fig1.circle('x','y', source = data_car_end_pos, size=8, color='blue', legend_label = 'car_end_pos')
fig1.circle('x','y', source = data_safe_circle_tang_pos, size=8, color='black', legend_label = 'car_safe_pos', visible = False)
fig1.multi_line('x_vec', 'y_vec',source = data_safe_circle_tang_line, line_width = 3, line_color = 'black', line_dash = 'solid',legend_label = 'safe_circle_tang_line', visible = False)
fig1.multi_line('x_vec', 'y_vec',source = data_slot, line_width = 1.5, line_color = 'black', line_dash = 'solid',legend_label = 'slot')
fig1.multi_line('x_vec', 'y_vec',source = data_rectangle_slot, line_width = 1.5, line_color = 'blue', line_dash = 'solid',legend_label = 'rectangle slot')

fig1.circle('plan_path_x', 'plan_path_y', source = data_planning_tune_complete, size=4, color='yellow', legend_label = 'sim_tuned_plan_complete')
fig1.line('plan_path_x', 'plan_path_y', source = data_planning_tune_complete, line_width = 6, line_color = 'green', line_dash = 'solid', line_alpha = 0.5, legend_label = 'sim_tuned_plan_complete')

fig1.patches('car_xn', 'car_yn', source = data_obs_car, fill_color = "skyblue", fill_alpha = 1.0, line_color = "black", line_width = 1, legend_label = 'obs_car')

fig1.patches('car_xn', 'car_yn', source = data_obs_column, fill_color = "orange", fill_alpha = 1.0, line_color = "black", line_width = 1, legend_label = 'obs_column')

fig1.patches('car_xn', 'car_yn', source = data_obs_wall, fill_color = "black", fill_alpha = 1.0, line_color = "black", line_width = 1, legend_label = 'obs_wall')


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

perpendicular_slant_tail_in_py.Init()

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.vehicle_type_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "vehicle_type",min=0, max=2, value=2, step=1)
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-10, max=10, value=-0.83, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value=-0.6, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=0, max=360, value=0.0, step=0.2)

    self.is_astar_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_astar",min=0, max=1, value=1, step=1)

    self.is_left_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_left",min=0, max=1, value=0, step=1)
    self.slot_phi_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_phi",min=45, max=90, value=90, step=15.0)

    self.slot_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_width",min=0, max=3, value=2.4, step=0.01)
    self.slot_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_length",min=0, max=6, value=5.0, step=0.01)

    self.slot_pt0_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_pt0_x",min=-10, max=10, value=2.0, step=0.01)
    self.slot_pt0_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot_pt0_y",min=-10, max=10, value=-2.0, step=0.01)

    self.car_inflation_slide = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "car_inflation",min=0.0, max=0.30, value=0.0, step=0.01)

    self.obs_car1_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_car1_x",min=-10, max=10, value=-1.9, step=0.01)
    self.obs_car1_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_car1_y",min=-10, max=10, value=-6.1, step=0.01)
    self.obs_car1_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_car1_heading",min=0, max=360, value=90.0, step=0.2)

    self.obs_car2_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_car2_x",min=-10, max=10, value=3.6, step=0.01)
    self.obs_car2_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_car2_y",min=-10, max=10, value=-6.1, step=0.01)
    self.obs_car2_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_car2_heading",min=0, max=360, value=90.0, step=0.2)

    self.obs_column1_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_column1_x",min=-10, max=10, value=6.19, step=0.01)
    self.obs_column1_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_column1_y",min=-10, max=10, value=4.67, step=0.01)
    self.obs_column1_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_column1_heading",min=0, max=360, value=90.0, step=0.2)

    self.obs_wall1_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_wall1_x",min=-10, max=10, value=9.82, step=0.01)
    self.obs_wall1_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_wall1_y",min=-10, max=10, value=0.42, step=0.01)
    self.obs_wall1_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs_wall1_heading",min=0, max=360, value=0.0, step=0.2)

    ipywidgets.interact(slider_callback, vehicle_type = self.vehicle_type_slider,
                                         ego_x = self.ego_x_slider,
                                         ego_y = self.ego_y_slider,
                                         ego_heading = self.ego_heading_slider,

                                         is_astar = self.is_astar_slider,

                                         is_left = self.is_left_slider,
                                         slot_phi = self.slot_phi_slider,

                                         slot_pt0_x = self.slot_pt0_x_slider,
                                         slot_pt0_y  = self.slot_pt0_y_slider,

                                         slot_width = self.slot_width_slider,
                                         slot_length = self.slot_length_slider,

                                         car_inflation = self.car_inflation_slide,

                                         obs_car1_x = self.obs_car1_x_slider,
                                         obs_car1_y = self.obs_car1_y_slider,
                                         obs_car1_heading = self.obs_car1_heading_slider,

                                         obs_car2_x = self.obs_car2_x_slider,
                                         obs_car2_y = self.obs_car2_y_slider,
                                         obs_car2_heading = self.obs_car2_heading_slider,

                                         obs_column1_x = self.obs_column1_x_slider,
                                         obs_column1_y = self.obs_column1_y_slider,
                                         obs_column1_heading = self.obs_column1_heading_slider,

                                         obs_wall1_x = self.obs_wall1_x_slider,
                                         obs_wall1_y = self.obs_wall1_y_slider,
                                         obs_wall1_heading = self.obs_wall1_heading_slider,
                                       )

### sliders callback
def slider_callback(vehicle_type, car_inflation, ego_x, ego_y, ego_heading, is_left, is_astar, slot_phi,
                   slot_pt0_x, slot_pt0_y, slot_width, slot_length,
                   obs_car1_x, obs_car1_y, obs_car1_heading, obs_car2_x, obs_car2_y, obs_car2_heading,
                   obs_column1_x, obs_column1_y, obs_column1_heading,
                   obs_wall1_x, obs_wall1_y, obs_wall1_heading):
  kwargs = locals()

  if vehicle_type == 0:
    vehicle_type = JAC_S811
  elif vehicle_type == 1:
    vehicle_type = CHERY_T26
  elif vehicle_type == 2:
    vehicle_type = CHERY_E0X

  car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type, car_inflation)

  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_x, ego_y, ego_heading/57.3)
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

  perpendicular_slant_tail_in_py.UpdateLocalization([ego_x, ego_y, ego_heading/57.3])

  delta_b = 0

  if is_left == 0:
    phi_rad = slot_phi/57.3
    heading_rad = phi_rad
    parallelogram_a = slot_width/math.sin(phi_rad)
    delta_b = slot_width/math.tan(phi_rad)
    parallelogram_b = delta_b + slot_length

    # (x0,y0)-->(x1,y1)
    delta_y1 = 0
    delta_x1 = parallelogram_a
    # (x0,y0)-->(x2,y2)
    delta_y2 = parallelogram_b * math.sin(heading_rad)
    delta_x2 = parallelogram_b * math.cos(heading_rad)
    # (x2,y2)-->(x3,y3)
    delta_y3 = 0
    delta_x3 = parallelogram_a

    slot_pt1_x = slot_pt0_x - delta_x1
    slot_pt1_y = slot_pt0_y + delta_y1

    slot_pt2_x = slot_pt0_x - delta_x2
    slot_pt2_y = slot_pt0_y - delta_y2

    slot_pt3_x = slot_pt2_x - delta_x3
    slot_pt3_y = slot_pt2_y + delta_y3
  else:
    phi_rad = (180 - slot_phi)/57.3
    heading_rad = phi_rad
    parallelogram_a = slot_width/math.sin(180/57.3 - phi_rad)
    delta_b = slot_width/math.tan(180/57.3 - phi_rad)
    parallelogram_b = delta_b + slot_length

    # (x0,y0)-->(x1,y1)
    delta_y1 = 0
    delta_x1 = parallelogram_a
    # (x0,y0)-->(x2,y2)
    delta_y2 = parallelogram_b * math.sin(heading_rad)
    delta_x2 = parallelogram_b * math.cos(heading_rad)
    # (x2,y2)-->(x3,y3)
    delta_y3 = 0
    delta_x3 = parallelogram_a

    slot_pt1_x = slot_pt0_x - delta_x1
    slot_pt1_y = slot_pt0_y + delta_y1

    slot_pt2_x = slot_pt0_x - delta_x2
    slot_pt2_y = slot_pt0_y - delta_y2

    slot_pt3_x = slot_pt2_x - delta_x3
    slot_pt3_y = slot_pt2_y + delta_y3

  slot_bound_x_vec, slot_bound_y_vec = [], []
  slot_bound_x_vec.append([slot_pt0_x, slot_pt2_x])
  slot_bound_y_vec.append([slot_pt0_y, slot_pt2_y])

  slot_bound_x_vec.append([slot_pt2_x, slot_pt3_x])
  slot_bound_y_vec.append([slot_pt2_y, slot_pt3_y])

  slot_bound_x_vec.append([slot_pt3_x, slot_pt1_x])
  slot_bound_y_vec.append([slot_pt3_y, slot_pt1_y])

  slot_bound_x_vec.append([slot_pt1_x, slot_pt0_x])
  slot_bound_y_vec.append([slot_pt1_y, slot_pt0_y])

  data_slot.data.update({
    'x_vec': slot_bound_x_vec,
    'y_vec': slot_bound_y_vec,})

  pt_0 = [slot_pt0_x, slot_pt0_y]
  pt_1 = [slot_pt1_x, slot_pt1_y]
  pt_2 = [slot_pt2_x, slot_pt2_y]
  pt_3 = [slot_pt3_x, slot_pt3_y]

  # 生成障碍物车
  car_xn = []
  car_yn = []
  car_box_x_vec = []
  car_box_y_vec = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], obs_car1_x, obs_car1_y, obs_car1_heading/57.3)
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
  car_box_x_vec.append(car_xn)
  car_box_y_vec.append(car_yn)
  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], obs_car2_x, obs_car2_y, obs_car2_heading/57.3)
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
  car_box_x_vec.append(car_xn)
  car_box_y_vec.append(car_yn)
  car_xn = []
  car_yn = []
  data_obs_car.data.update({
    'car_xn': car_box_x_vec,
    'car_yn': car_box_y_vec,
  })


  # 生成障碍物柱子
  column_width = 1.0
  column_length = 1.0
  column_xn = []
  column_yn = []
  column_box_x_vec = []
  column_box_y_vec = []
  column_xb = [-0.5, 0.5, 0.5, -0.5]
  column_yb = [0.5, 0.5, -0.5, -0.5]
  column_xb = [x * column_width for x in column_xb]
  column_yb = [y * column_length for y in column_yb]
  for i in range(len(column_xb)):
      tmp_x, tmp_y = local2global(column_xb[i], column_yb[i], obs_column1_x, obs_column1_y, obs_column1_heading/57.3)
      column_xn.append(tmp_x)
      column_yn.append(tmp_y)
  column_box_x_vec.append(column_xn)
  column_box_y_vec.append(column_yn)
  column_xn = []
  column_yn = []
  data_obs_column.data.update({
    'car_xn': column_box_x_vec,
    'car_yn': column_box_y_vec,
  })


  # 生成障碍物墙
  wall_width = 0.25
  wall_length = 12.68
  wall_xn = []
  wall_yn = []
  wall_box_x_vec = []
  wall_box_y_vec = []
  wall_xb = [-0.5, 0.5, 0.5, -0.5]
  wall_yb = [0.5, 0.5, -0.5, -0.5]
  wall_xb = [x * wall_width for x in wall_xb]
  wall_yb = [y * wall_length for y in wall_yb]
  for i in range(len(wall_xb)):
      tmp_x, tmp_y = local2global(wall_xb[i], wall_yb[i], obs_wall1_x, obs_wall1_y, obs_wall1_heading/57.3)
      wall_xn.append(tmp_x)
      wall_yn.append(tmp_y)
  wall_box_x_vec.append(wall_xn)
  wall_box_y_vec.append(wall_yn)
  wall_xn = []
  wall_yn = []
  data_obs_wall.data.update({
    'car_xn': wall_box_x_vec,
    'car_yn': wall_box_y_vec,
  })


  perpendicular_slant_tail_in_py.UpdateSlot([pt_0, pt_1, pt_2, pt_3])

  perpendicular_slant_tail_in_py.UpdateStateMachine()

  perpendicular_slant_tail_in_py.Update()

  complete_path_pt_vec = perpendicular_slant_tail_in_py.GetCompletePlanPath()
  cur_gear_path_pt_vec = perpendicular_slant_tail_in_py.GetCurrentGearPlanPath()

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
    if (k % 5 != 0):
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

  data_simu_car_box.data.update({
    'x_vec': car_box_x_vec,
    'y_vec': car_box_y_vec,
  })

  obs_vec = perpendicular_slant_tail_in_py.GetObsVec()
  obs_pt_x, obs_pt_y = [], []
  for i in range(len(obs_vec)):
    obs_pt_x.append(obs_vec[i][0])
    obs_pt_y.append(obs_vec[i][1])

  data_obs_pos.data.update({
    'x': obs_pt_x,
    'y': obs_pt_y,
  })


  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
