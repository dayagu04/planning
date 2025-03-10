from math import cos, fabs, pi, sin
import sys, os
sys.path.append("..")

sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

from contruct_scenario import construct_scenario
from lib.load_local_view_parking import *
from bokeh.events import Tap
from bokeh.models import Range1d, Arrow, NormalHead
from bokeh.io import export_svgs
from jupyter_pybind import parallel_planning_py
from bokeh.models import SingleIntervalTicker
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
import os
import sys
import cairosvg
from bokeh.io import export_png
# sys.path.append('/root/miniconda3/lib/python3.7/site-packages')

kRad2Deg = 180.0 / pi
kDeg2Rad = pi / 180.0
name = "parking_out_normal_channel"

name = "parking_out_narrow_channel"


if name == "parking_out_narrow_channel":
  channel_width = 3.14
elif name == "parking_out_normal_channel":
  channel_width = 4.34

def find_turning_points(parking_out_x_vec, parking_out_y_vec):
    # 将输入数组转换为 NumPy 数组
    x = np.array(parking_out_x_vec)
    y = np.array(parking_out_y_vec)

    # 计算相邻点的方向向量
    dx = np.diff(x)
    dy = np.diff(y)

    # 计算方向向量的角度（以弧度为单位）
    angles = np.arctan2(dy, dx)

    # 计算相邻角度的差值
    angle_diff = np.diff(angles)

    # 将角度差值归一化到 [-pi, pi] 范围内
    angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

    # 转向改变的点是在角度差值绝对值较大的地方
    turning_points = np.where(np.abs(angle_diff) > np.pi / 4)[0] + 1

    return turning_points

def cal_common_circle(x0, y0, x1, y1, x2, y2):
    # 计算向量 v1 = P1 - P0, v2 = P2 - P0
    v1 = (x1 - x0, y1 - y0)
    v2 = (x2 - x0, y2 - y0)

    # 计算垂直向量 v1_perp = (-v1_y, v1_x), v2_perp = (-v2_y, v2_x)
    v1_perp = (-v1[1], v1[0])
    v2_perp = (-v2[1], v2[0])

    # 计算中点 M1 = (P0 + P1) / 2, M2 = (P0 + P2) / 2
    M1 = ((x0 + x1) / 2, (y0 + y1) / 2)
    M2 = ((x0 + x2) / 2, (y0 + y2) / 2)

    # 计算圆心 (a, b) 作为两条垂直平分线的交点
    # 方程形式：M1 + t * v1_perp = M2 + s * v2_perp
    # 解方程组得到 t 或 s
    A = v1_perp[0]
    B = -v2_perp[0]
    C = v1_perp[1]
    D = -v2_perp[1]
    E = M2[0] - M1[0]
    F = M2[1] - M1[1]

    # 解线性方程组
    denominator = A * D - B * C
    if denominator == 0:
        raise ValueError("三个点共线，无法确定唯一的圆")

    t = (D * E - B * F) / denominator

    # 计算圆心
    a = M1[0] + t * v1_perp[0]
    b = M1[1] + t * v1_perp[1]

    # 计算半径
    r = ((a - x0) ** 2 + (b - y0) ** 2) ** 0.5

    return [a, b], r

def load_car_box(path_x_vec, path_y_vec, path_theta_vec, car_xb, car_yb):
  car_box_x_vec = []
  car_box_y_vec = []
  for k in range(0, len(path_x_vec)):
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], path_x_vec[k], path_y_vec[k], path_theta_vec[k])
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)
  return car_box_x_vec, car_box_y_vec

def load_ego_car_corner_path(path_x_vec, path_y_vec, path_heading_vec, car_local_x, car_local_y):
  corner_path_x_vec = []
  corner_path_y_vec = []
  for i in range(len(path_x_vec)):
    tmp_x, tmp_y = local2global( car_local_x, car_local_y, path_x_vec[i], path_y_vec[i], path_heading_vec[i])
    corner_path_x_vec.append(tmp_x)
    corner_path_y_vec.append(tmp_y)
  return corner_path_x_vec, corner_path_y_vec

def load_ego_car_box(ego_x, ego_y, ego_heading, car_xb, car_yb):
  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
    tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_x, ego_y, ego_heading)
    car_xn.append(tmp_x)
    car_yn.append(tmp_y)
  return car_xn, car_yn


display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type = CHERY_E0X, car_lat_inflation = 0.0)
coord_tf = coord_transformer()

data_start_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_target_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_PA = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_AB = ColumnDataSource(data=dict(x=[0], y=[0], radius=[1]))
data_BT = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_target_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_preparing_line = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'theta_vec':[]})
data_corner_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_preparing_step_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_preparing_line_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_parking_out_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_in_slot_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_tra_search_out_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_tra_corner_search_out_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_all_debug_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_tra_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_slot = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_other_slot = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_fus_obs = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_obs_pt = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_virtual_obs_pt = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_tra_tb_pt = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_debug_arc = ColumnDataSource(data = {'cx_vec':[],
                                        'cy_vec':[],
                                        'radius_vec':[],
                                        'pBx_vec':[],
                                        'pBy_vec':[],
                                        'pCx_vec':[],
                                        'pCy_vec':[]})
data_extra_region = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_obs_car_polygon = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_arc_center = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_lines = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_swap_line = ColumnDataSource(data = {'x0':[], 'y0':[], 'x1':[], 'y1':[]})
data_text = ColumnDataSource(data={
    'x': [],
    'y': [],
    'text': []
})

# fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=700, height=600, match_aspect = True, aspect_scale=1)

fig1 = bkp.figure(width=1200, height=800, match_aspect = True, aspect_scale=1)
# fig1.background_fill_color = "#E0E0E0"  # 你也可以用 '#D3D3D3'

word_size = '22pt'

fig1.x_range.flipped = False
fig1.outline_line_color = "black"
fig1.outline_line_width = 1.0  # 可以调整边框线条的宽度

fig1.x_range = Range1d(start = -3.0, end = 16)
fig1.y_range = Range1d(start = -3.2, end = 9.2)

fig1.xaxis.axis_label_text_font_size = word_size
fig1.xaxis.axis_label_text_font = 'Times New Roman'

fig1.yaxis.axis_label_text_font_size = word_size
fig1.yaxis.axis_label_text_font = 'Times New Roman'

fig1.xaxis.major_label_text_font_size = word_size  # 设置x轴字体大小
fig1.xaxis.major_label_text_font = 'Times New Roman'      # 设置字体类型

fig1.yaxis.major_label_text_font_size = word_size
fig1.yaxis.major_label_text_font = 'Times New Roman'

fig1.xaxis.ticker = SingleIntervalTicker(interval = 4, num_minor_ticks=0)
fig1.yaxis.ticker = SingleIntervalTicker(interval = 4, num_minor_ticks=0)

fig1.xgrid.grid_line_color = None
fig1.ygrid.grid_line_color = None


# # 去除图形四周边框
# fig1.outline_line_color = None
# fig1.xaxis.visible = False
# fig1.yaxis.visible = False
# fig1.xaxis.major_label_text_font_size = '0pt'  # 设置字体大小
# fig1.yaxis.major_label_text_font_size = '0pt'
# measure tool
source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=10, source=source, color='red')
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid')
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='18pt')
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

# 尝试确保图表内容比例一致
aspect_ratio = (fig1.x_range.end - fig1.x_range.start) / (fig1.y_range.end - fig1.y_range.start)
fig1.plot_height = int(fig1.plot_width / aspect_ratio)

fig1.patches('x_vec', 'y_vec', source = data_car_box, fill_color = "lightskyblue", fill_alpha=0.3,  line_color= None)
# car box at start pose
fig1.circle('x', 'y', source = data_start_pos, size=8, color='green', visible = True)
fig1.patch( 'car_xn', 'car_yn', source = data_start_car, fill_color = 'green', fill_alpha = 0.5, line_color = "green", line_width = 1.0, visible = True)

# target pose
fig1.circle('x', 'y', source = data_target_pos, size=8, color='blue')
fig1.patch( 'car_xn', 'car_yn', source = data_target_car, fill_color = 'blue', fill_alpha = 0.20, line_color = "blue", line_width = 1.0)

fig1.multi_line('x_vec','y_vec',source =data_corner_path, line_width = 3.0, line_color = 'blue', line_dash = 'dashed', visible = True)
fig1.line('x_vec','y_vec',source =data_path, line_width = 3.0, line_color = 'blue', line_dash = 'solid', visible = True)

# target slot
# fig1.line('x_vec','y_vec',source =data_slot,  line_width = 1.0, line_color = 'black', line_dash = 'solid',legend_label = 'slot', visible = True)
fig1.multi_line('x_vec','y_vec',source =data_slot,  line_width = 0.5, line_color = 'black', line_dash = 'solid')
fig1.multi_line('x_vec','y_vec',source =data_other_slot,  line_width = 0.5, line_color = 'black', line_dash = 'solid')

fig1.patches("x_vec", "y_vec", source=data_obs_car_polygon, color='white', fill_alpha=1, line_color=None, visible = False)

# obstacles
fig1.scatter("x_vec", "y_vec", source=data_fus_obs, size=3, color='grey', visible = True)
fig1.scatter("x_vec", "y_vec", source=data_obs_pt, size=3, color='red', visible = True)


fig1.line('x_vec','y_vec',source = data_preparing_line, line_width = 1.5, line_color = 'green', line_dash = 'dashed', visible = True)

# fig1.scatter("x_vec", "y_vec", source=data_virtual_obs_pt, size=8, color='red', marker='star', visible = False)

fig1.circle('x_vec', 'y_vec', source = data_arc_center, size=4, color='black')

fig1.multi_line('x_vec', 'y_vec', source = data_lines, line_width = 0.5, line_dash = 'dotted', line_color='black')

label = Label(x=-2.0 , y=5.4, text="Step 1 circle", text_color="black", text_font_size=word_size,
              x_offset=0, y_offset=0)
fig1.add_layout(label)

if name == "parking_out_normal_channel":
  arrow_start = Arrow(
    end=NormalHead(size=10, fill_color="black"),
    x_start='x0',
    y_start='y0',
    x_end='x1',
    y_end='y1',
    line_width=0.0,
    line_color="black",
    source=data_swap_line  # 绑定数据源
  )

  arrow_end = Arrow(
    end=NormalHead(size=10, fill_color="black"),
    x_start='x1',
    y_start='y1',
    x_end='x0',
    y_end='y0',
    line_width=0.0,
    line_color="black",
    source=data_swap_line  # 绑定数据源
  )
  fig1.add_layout(arrow_start)
  fig1.add_layout(arrow_end)

  fig1.text(
    x='x',
    y='y',
    text='text',
    source=data_text,
    text_color="black",
    text_align="center",
    text_font_size= word_size
  )
  x2 = 7.8
  y2 = -2.9
else:
  x2 = 3.5
  y2 = -2.9
  label = Label(x=9.5, y=-2.9, text="Step 3 circle", text_color="black", text_font_size=word_size,
              x_offset=0, y_offset=0)
  fig1.add_layout(label)


label = Label(x=x2, y=y2, text="Step 2 circle", text_color="black", text_font_size=word_size,
              x_offset=0, y_offset=0)
fig1.add_layout(label)

fig1.line('x_vec','y_vec',source =data_parking_out_path,  line_width = 3.0, line_color = 'blue', line_dash = 'solid',legend_label = 'Inversed parking out step')

fig1.legend.visible = False
fig1.legend.location = 'top_left'
fig1.legend.label_text_font_size = word_size
fig1.legend.label_text_font = "Times New Roman"
fig1.legend.click_policy = 'hide'

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

parallel_planning_py.Init()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.is_front_occupied_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is front occupied",min=0, max=1, value= 1, step=1)
    self.is_rear_occupied_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is rear occupied",min=0, max=1, value= 1, step=1)
    self.is_all_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "all debug path",min=0, max=1, value= 0, step=1)
    # ego pose
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-15, max=15, value= 13.58, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value= 2.93, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=-180, max=180, value= 0, step=0.1)
    # slot pt
    self.slot_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot length",min=5.0, max=8.0, value=6.0, step=0.01)
    self.slot_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "width",min=2.0, max=3.0, value=2.2, step=0.01)
    # obs
    self.lon_space_dx_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "lon_space_dx",min=-1.0, max=4.0, value=0.0, step=0.01)
    self.curb_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "curb offset ",min=-1.0, max=1.0, value=0.3, step=0.01)
    self.channel_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel width",min=2.5, max=10.0, value=channel_width, step=0.01) # 3.24

    self.front_car_y_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "front obs y",min=-2.0, max=4.0, value=0.2, step=0.01)
    self.front_car_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "front obs heading",min=-180.0, max=180.0, value=0.0, step=0.1)

    self.rear_car_y_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "rear obs y",min=-2.0, max=4.0, value=0.2, step=0.01)
    self.rear_car_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "rear obs heading",min=-180.0, max=180.0, value=0.0, step=0.1)

    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "path ds",min=0.01, max=1.0, value=0.01, step=0.001)
    ipywidgets.interact(slider_callback,
                                         is_front_occupied = self.is_front_occupied_slider,
                                         is_rear_occupied = self.is_rear_occupied_slider,
                                         is_all_path = self.is_all_path_slider,

                                         ego_x = self.ego_x_slider,
                                         ego_y = self.ego_y_slider,
                                         ego_heading = self.ego_heading_slider,

                                         slot_length = self.slot_length_slider,
                                         slot_width = self.slot_width_slider,
                                         lon_space_dx  =  self.lon_space_dx_slider,
                                         curb_offset = self.curb_offset_slider,
                                         channel_width = self.channel_width_slider,

                                         front_car_y_offset = self.front_car_y_offset_slider,
                                         front_car_heading = self.front_car_heading_slider,
                                         rear_car_y_offset = self.rear_car_y_offset_slider,
                                         rear_car_heading = self.rear_car_heading_slider,

                                         ds = self.ds_slider)

### sliders callback
def slider_callback(is_front_occupied, is_rear_occupied, is_all_path, ego_x, ego_y, ego_heading,
                    slot_length, slot_width, lon_space_dx, curb_offset, channel_width,
                    front_car_y_offset, front_car_heading, rear_car_y_offset, rear_car_heading,
                    ds):
  kwargs = locals()

  # print("before construct_scenario")

  front_car_heading_rad = front_car_heading * kDeg2Rad
  rear_car_heading_rad = rear_car_heading * kDeg2Rad
  ego_heading_rad = ego_heading * kDeg2Rad

  construct_scenario(slot_width, slot_length, curb_offset, lon_space_dx, channel_width,
      front_car_y_offset, front_car_heading_rad, rear_car_y_offset, rear_car_heading_rad,
      is_front_occupied, is_rear_occupied)

  #read obs
  read_file = "{}{}_{}{}".format("/asw/data/",
      "front_occupied" if is_front_occupied else "front_vacant",
      "rear_occupied" if is_rear_occupied else "rear_vacant",
      ".json"
  )

  with open(read_file, 'r') as file:
    json_data = json.load(file)

  obs_x_vec = json_data["obs_x"]
  obs_y_vec = json_data["obs_y"]

  # obstacles
  data_fus_obs.data.update({
    'x_vec': obs_x_vec,
    'y_vec': obs_y_vec,
  })

  half_slot_width_sgn = 0.5 * slot_width
  # slot_x_vec = [slot_length, 0.0, 0.0, slot_length]
  # slot_y_vec = [half_slot_width_sgn, half_slot_width_sgn, -half_slot_width_sgn, -half_slot_width_sgn]

  slot_x_vec = [[slot_length, 0.0], [slot_length, 0.0]]
  slot_y_vec = [[half_slot_width_sgn, half_slot_width_sgn], [-half_slot_width_sgn, -half_slot_width_sgn]]
  data_slot.data.update({
     'x_vec': slot_x_vec,
     'y_vec': slot_y_vec})

  slot_x_vec = [slot_length, 0.0, 0.0, slot_length]
  slot_y_vec = [half_slot_width_sgn, half_slot_width_sgn, -half_slot_width_sgn, -half_slot_width_sgn]

  other_slot_x_vec1 = [x-slot_length for x in slot_x_vec]
  other_slot_x_vec1.append(other_slot_x_vec1[0])
  other_slot_x_vec2 = [x+slot_length for x in slot_x_vec]
  other_slot_x_vec2.append(other_slot_x_vec2[0])
  slot_y_vec.append(slot_y_vec[0])

  data_other_slot.data.update({
     'x_vec': [other_slot_x_vec1, other_slot_x_vec2],
     'y_vec': [slot_y_vec, slot_y_vec]})



  # clear
  data_path.data.update({
    'x_vec': [],
    'y_vec': [],
    'theta_vec': [],
  })

  data_all_debug_path.data.update({
    'x_vec': [],
    'y_vec': [],
  })

  data_car_box.data.update({
      'x_vec': [],
      'y_vec': [],
  })


  data_parking_out_path.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  if parallel_planning_py.UpdateByJson(obs_x_vec, obs_y_vec, slot_width, slot_length, ego_x, ego_y,
                ego_heading_rad, ds):

    path_x_vec = parallel_planning_py.GetPathEle(0)
    path_y_vec = parallel_planning_py.GetPathEle(1)
    path_theta_vec = parallel_planning_py.GetPathEle(2)

    # select inversed parking step path
    #path in different step
    preparing_step_end_idx = 0
    preparing_line_end_idx = 0
    parking_out_end_idx = 0
    path_size = len(path_theta_vec)
    in_slot_end_idx = path_size - 1
    for i in range(len(path_theta_vec)):
      if fabs(path_theta_vec[i] * kRad2Deg) < 1:
        preparing_step_end_idx = i
        break

    for i in range(preparing_step_end_idx,len(path_theta_vec)):
      if fabs(path_theta_vec[i] * kRad2Deg) > 1:
        preparing_line_end_idx = i
        break
    for i in range(preparing_line_end_idx + 2,len(path_theta_vec)):
      v_ba_x = path_x_vec[i - 1] - path_x_vec[i]
      v_ba_y = path_y_vec[i - 1] - path_y_vec[i]
      dot = v_ba_x * cos(path_theta_vec[i]) + v_ba_y * sin(path_theta_vec[i])
      if dot < 0.0:
        parking_out_end_idx = i - 1
        break

    parking_out_x_vec = path_x_vec[preparing_line_end_idx - 1 : parking_out_end_idx]
    parking_out_y_vec = path_y_vec[preparing_line_end_idx - 1 : parking_out_end_idx]
    parking_out_theta_vec = path_theta_vec[preparing_line_end_idx - 1 : parking_out_end_idx]

    # car path
    data_parking_out_path.data.update({
      'x_vec': parking_out_x_vec,
      'y_vec': parking_out_y_vec,
    })
    # car box envelope
    parking_out_car_box_x_vec, parking_out_car_box_y_vec = load_car_box(parking_out_x_vec, parking_out_y_vec, parking_out_theta_vec, car_xb, car_yb)

    data_car_box.data.update({
      'x_vec': parking_out_car_box_x_vec,
      'y_vec': parking_out_car_box_y_vec,
    })

    fl_corner_x_vec, fl_corner_y_vec = load_ego_car_corner_path(parking_out_x_vec, parking_out_y_vec, parking_out_theta_vec, car_xb[0], car_yb[0])
    fr_corner_x_vec, fr_corner_y_vec = load_ego_car_corner_path(parking_out_x_vec, parking_out_y_vec, parking_out_theta_vec, car_xb[5], car_yb[5])
    rs_corner_x_vec, rs_corner_y_vec = load_ego_car_corner_path(parking_out_x_vec, parking_out_y_vec, parking_out_theta_vec, 0.0, car_yb[5])
    l_mirror_x_vec, l_mirror_y_vec = load_ego_car_corner_path(parking_out_x_vec, parking_out_y_vec, parking_out_theta_vec, car_xb[18], car_yb[18])


    data_corner_path.data.update({
      'x_vec': [fl_corner_x_vec, fr_corner_x_vec, rs_corner_x_vec, l_mirror_x_vec],
      'y_vec': [fl_corner_y_vec, fr_corner_y_vec, rs_corner_y_vec, l_mirror_y_vec]
    })

    reversed_park_out_x_vec = parking_out_x_vec[::-1]
    reversed_park_out_y_vec = parking_out_y_vec[::-1]

    center_1, radius1 = cal_common_circle(reversed_park_out_x_vec[0], reversed_park_out_y_vec[0],
                                          reversed_park_out_x_vec[1], reversed_park_out_y_vec[1],
                                          reversed_park_out_x_vec[2], reversed_park_out_y_vec[2])
    center_1 = np.array(center_1)
    curvature_change_idx = 0
    # print("radius1 = ", radius1)
    for i in range(len(reversed_park_out_x_vec)):
      dist = math.sqrt( (center_1[0] - reversed_park_out_x_vec[i])**2 + (center_1[1] - reversed_park_out_y_vec[i])**2)
      # print("i = ", i, ", dist = ", dist)
      if dist > radius1 + 1e-8:
        curvature_change_idx = i
        break
    curvature_change_idx -= 1

    curvature_change_pt = np.array([
      reversed_park_out_x_vec[curvature_change_idx],
      reversed_park_out_y_vec[curvature_change_idx],
    ])

    v_c1_to_c2 = curvature_change_pt - np.array(center_1)
    v_c1_to_c2 = v_c1_to_c2 / np.linalg.norm(v_c1_to_c2)
    center_2 = center_1 + 2.0 * radius1 * v_c1_to_c2

    center_2, radius2 = cal_common_circle(reversed_park_out_x_vec[curvature_change_idx], reversed_park_out_y_vec[curvature_change_idx],
                                      reversed_park_out_x_vec[curvature_change_idx + 1], reversed_park_out_y_vec[curvature_change_idx + 1],
                                      reversed_park_out_x_vec[curvature_change_idx + 2], reversed_park_out_y_vec[curvature_change_idx + 2])

    cur_change2_idx = 0
    if name == "parking_out_narrow_channel":
      for i in range(curvature_change_idx, len(reversed_park_out_x_vec) - 3):
        pt0 = np.array([ reversed_park_out_x_vec[i], reversed_park_out_y_vec[i]])
        pt1 = np.array([ reversed_park_out_x_vec[i + 1], reversed_park_out_y_vec[i + 1]])
        pt2 = np.array([ reversed_park_out_x_vec[i + 2], reversed_park_out_y_vec[i + 2]])
        v_01 = pt1 - pt0
        v_01 = v_01 / np.linalg.norm(v_01)
        v_12 = pt2 - pt1
        v_12 = v_12 / np.linalg.norm(v_12)
        if np.dot(v_01, v_12) > cos(0.01 * kDeg2Rad):
          cur_change2_idx = i
          break

      for i in range (cur_change2_idx, len(reversed_park_out_x_vec)):
        pt0 = np.array([ reversed_park_out_x_vec[i], reversed_park_out_y_vec[i]])
        pt1 = np.array([ reversed_park_out_x_vec[i + 1], reversed_park_out_y_vec[i + 1]])
        pt2 = np.array([ reversed_park_out_x_vec[i + 2], reversed_park_out_y_vec[i + 2]])
        v_01 = pt1 - pt0
        v_01 = v_01 / np.linalg.norm(v_01)
        v_12 = pt2 - pt1
        v_12 = v_12 / np.linalg.norm(v_12)
        if np.dot(v_01, v_12) < cos(0.01 * kDeg2Rad):
          cur_change3_idx = i + 1
          core_pt_3 = [reversed_park_out_x_vec[cur_change3_idx], reversed_park_out_y_vec[cur_change3_idx]]
          center_3, radius3 = cal_common_circle(reversed_park_out_x_vec[cur_change3_idx], reversed_park_out_y_vec[cur_change3_idx],
                            reversed_park_out_x_vec[cur_change3_idx + 1], reversed_park_out_y_vec[cur_change3_idx + 1],
                            reversed_park_out_x_vec[cur_change3_idx + 2], reversed_park_out_y_vec[cur_change3_idx + 2])
          break

    else:
        cur_change2_idx = len(reversed_park_out_x_vec) -1


    arc_2_end_pos = [reversed_park_out_x_vec[cur_change2_idx], reversed_park_out_y_vec[cur_change2_idx]]

    data_arc_center_x_vec = [center_1[0], center_2[0], 0.5 * (center_1[0] + center_2[0])]
    data_arc_center_y_vec = [center_1[1], center_2[1], 0.5 * (center_1[1] + center_2[1])]

    line_x_vec = [[center_1[0], parking_out_x_vec[-1]],
              [center_1[0], center_2[0]],
              [center_2[0], arc_2_end_pos[0]]
              ]

    line_y_vec = [[center_1[1], parking_out_y_vec[-1]],
              [center_1[1], center_2[1]],
              [center_2[1], arc_2_end_pos[1]]
              ]

    if name == "parking_out_narrow_channel":
      data_arc_center_x_vec.append(core_pt_3[0])
      data_arc_center_y_vec.append(core_pt_3[1])

      data_arc_center_x_vec.append(center_3[0])
      data_arc_center_y_vec.append(center_3[1])

      line_x_vec.append([core_pt_3[0], center_3[0]])
      line_y_vec.append([core_pt_3[1], center_3[1]])

      line_x_vec.append([reversed_park_out_x_vec[-1], center_3[0]])
      line_y_vec.append([reversed_park_out_y_vec[-1], center_3[1]])
    else:
      ego_fl_corner_x0 = fl_corner_x_vec[0]
      ego_fl_corner_y0 = fl_corner_y_vec[0]
      line_x_vec.append([ego_fl_corner_x0 + 0.3, 12.2])
      line_y_vec.append([ego_fl_corner_y0, ego_fl_corner_y0])

      max_y_idx = fl_corner_y_vec.index(max(fl_corner_y_vec))
      peak_x = fl_corner_x_vec[max_y_idx]
      peak_y = fl_corner_y_vec[max_y_idx]

      line_x_vec.append([peak_x, 12.2])
      line_y_vec.append([peak_y, peak_y])

      line_x_vec.append([12, 12])
      line_y_vec.append([ego_fl_corner_y0, peak_y])

      data_swap_line.data.update({
        'x0': [12],
        'y0': [peak_y],
        'x1': [12],
        'y1': [ego_fl_corner_y0]
      })

      data_text.data.update({
        'x': [13],
        'y': [0.5 * (peak_y + ego_fl_corner_y0) + 0.5],
        'text': ["Minimal lateral distance"]
      })
      print("minimal lateral dist = ",peak_y - ego_fl_corner_y0)

    data_arc_center.data.update({
      'x_vec': data_arc_center_x_vec,
      'y_vec': data_arc_center_y_vec
    })

    data_lines.data.update({
      'x_vec':line_x_vec,
      'y_vec':line_y_vec
    })

    # ego start position
    data_start_pos.data.update({'x':[parking_out_x_vec[0]],'y':[parking_out_y_vec[0]]})
    car_xn, car_yn = load_ego_car_box(parking_out_x_vec[0], parking_out_y_vec[0], parking_out_theta_vec[0], car_xb, car_yb)
    data_start_car.data.update({
      'car_xn': car_xn,
      'car_yn': car_yn,
    })

    data_preparing_line.data.update({
      'x_vec': [-0.5, 16.0],
      'y_vec': [parking_out_y_vec[0], parking_out_y_vec[0]]
    })

    # target pose
    data_target_pos.data.update({
      'x': [parking_out_x_vec[-1]],
      'y': [parking_out_y_vec[-1]],
    })
    target_car_xn, target_car_yn = load_ego_car_box(parking_out_x_vec[-1], parking_out_y_vec[-1], parking_out_theta_vec[-1], car_xb, car_yb)
    data_target_car.data.update({
      'car_xn': target_car_xn,
      'car_yn': target_car_yn,
    })




  obs_in_tboundary = parallel_planning_py.GetParkPlannerObs()
  # obstacles
  data_obs_pt.data.update({
    'x_vec': obs_in_tboundary[0],
    'y_vec': obs_in_tboundary[1],
  })
  data_virtual_obs_pt.data.update({
    'x_vec': parallel_planning_py.GetVirtualObsX(),
    'y_vec': parallel_planning_py.GetVirtualObsY(),
  })

  data_tra_tb_pt.data.update({
  'x_vec':[],
  'y_vec':[]
  })

  tra_tb_x_vec, tra_tb_y_vec = parallel_planning_py.GenTraTBoundary(slot_length)
  tra_tb_x_vec.append(tra_tb_x_vec[0])
  tra_tb_y_vec.append(tra_tb_y_vec[0])
  data_tra_tb_pt.data.update({
  'x_vec':tra_tb_x_vec,
  'y_vec':tra_tb_y_vec
  })

  extra_x_vec = []
  extra_y_vec = []
  x_min = tra_tb_x_vec[0]
  x_max = tra_tb_x_vec[len(tra_tb_x_vec) - 3]
  channel_y_min = tra_tb_y_vec[len(tra_tb_x_vec) - 3]
  y_max = 7.9
  Polygon_channel = Polygon([(x_min, channel_y_min), (x_max, channel_y_min), (x_max, y_max), (x_min, y_max)])
  tmp_x, tmp_y = map(list, Polygon_channel.exterior.xy)
  extra_x_vec.append(tmp_x)
  extra_y_vec.append(tmp_y)

  if is_rear_occupied and len(obs_in_tboundary) > 0:
    planner_obs_x_vec = obs_in_tboundary[0]
    planner_obs_y_vec = obs_in_tboundary[1]
    rear_box_min_y = 10.0
    for i in range(len(planner_obs_x_vec)):
      if planner_obs_x_vec[i] > -2.0 and planner_obs_x_vec[i] < -0.8 and planner_obs_y_vec[i] > 0.0:
        rear_box_min_y = min(rear_box_min_y, planner_obs_y_vec[i])
    Polygon_rear_box = Polygon([(x_min, tra_tb_y_vec[0]), (tra_tb_x_vec[1], tra_tb_y_vec[1]), (tra_tb_x_vec[1], rear_box_min_y), (x_min, rear_box_min_y)])
    tmp_x, tmp_y = map(list, Polygon_rear_box.exterior.xy)
    extra_x_vec.append(tmp_x)
    extra_y_vec.append(tmp_y)

  if is_front_occupied and len(obs_in_tboundary) > 0:
      planner_obs_x_vec = obs_in_tboundary[0]
      planner_obs_y_vec = obs_in_tboundary[1]
      front_box_min_y = 10.0
      for i in range(len(planner_obs_x_vec)):
        if planner_obs_x_vec[i] > 10.0 and planner_obs_x_vec[i] < 20.0 and planner_obs_y_vec[i] > 0.0:
          front_box_min_y = min(front_box_min_y, planner_obs_y_vec[i])
      Polygon_rear_box = Polygon([(tra_tb_x_vec[4], tra_tb_y_vec[4]), (tra_tb_x_vec[5], tra_tb_y_vec[5]), (tra_tb_x_vec[5], front_box_min_y), (tra_tb_x_vec[4], front_box_min_y)])
      tmp_x, tmp_y = map(list, Polygon_rear_box.exterior.xy)
      extra_x_vec.append(tmp_x)
      extra_y_vec.append(tmp_y)


  data_extra_region.data.update({
  'x_vec':extra_x_vec,
  'y_vec':extra_y_vec
  })

  obs_car_polygon_x_vec = []
  obs_car_polygon_y_vec = []

  channel_car_size = int(len(json_data["channel_matrix"]) / 2)
  for i in range(channel_car_size):
    channel_vec = [json_data["channel_matrix"][2 * i], json_data["channel_matrix"][2 * i + 1]]
    points = np.column_stack((channel_vec[0], channel_vec[1]))
    hull = ConvexHull(points)
    polygon = Polygon(points[hull.vertices])
    if polygon.geom_type == 'Polygon':
      tmp_x, tmp_y = polygon.exterior.xy
      obs_car_polygon_x_vec.append(list(tmp_x))
      obs_car_polygon_y_vec.append(list(tmp_y))

  rear_obs_car_matrix = json_data["rear_obs_car_matrix"]
  # print("rear_obs_car_matrix size = ", len(rear_obs_car_matrix))

  if len(rear_obs_car_matrix) > 0:
    # print("rear_obs_car_ obs size = ", len(rear_obs_car_matrix[0]))
    points = np.column_stack((rear_obs_car_matrix[0], rear_obs_car_matrix[1]))
    hull = ConvexHull(points)
    polygon = Polygon(points[hull.vertices])
    if polygon.geom_type == 'Polygon':
      tmp_x, tmp_y = polygon.exterior.xy
      obs_car_polygon_x_vec.append(list(tmp_x))
      obs_car_polygon_y_vec.append(list(tmp_y))

  front_obs_car_matrix = json_data["front_obs_car_matrix"]
  # print("front_obs_car_matrix size = ", len(front_obs_car_matrix))

  if len(front_obs_car_matrix) > 0:
    # print("front_obs_car obs size = ", len(front_obs_car_matrix[0]))
    points = np.column_stack((front_obs_car_matrix[0], front_obs_car_matrix[1]))
    hull = ConvexHull(points)
    polygon = Polygon(points[hull.vertices])
    if polygon.geom_type == 'Polygon':
      tmp_x, tmp_y = polygon.exterior.xy
      obs_car_polygon_x_vec.append(list(tmp_x))
      obs_car_polygon_y_vec.append(list(tmp_y))

  data_obs_car_polygon.data.update({
  'x_vec': obs_car_polygon_x_vec,
  'y_vec': obs_car_polygon_y_vec
  })

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)

os.environ['WEB_BROWSER'] = 'chrome'
fig1.output_backend = "svg"

svg_file_path = name +'.svg'
eps_file_path = name +'.eps'
export_svgs(fig1, filename=svg_file_path)

# 使用 CairoSVG 直接转换
cairosvg.svg2eps(url=svg_file_path, write_to=eps_file_path)

print(f"SVG 文件已直接转换为 EPS：'{eps_file_path}'")

cairosvg.svg2pdf(url=svg_file_path, write_to=name +'.pdf')
