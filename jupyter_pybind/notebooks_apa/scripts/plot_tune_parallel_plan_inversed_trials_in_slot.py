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

lon_dist = 0.9
word_size_pt = '22pt'

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

def find_shift_points(path_x_vec, path_y_vec, path_heading_vec):
  x_vec_size = len(path_x_vec)
  y_vec_size = len(path_y_vec)
  heading_vec_size = len(path_heading_vec)
  if x_vec_size != y_vec_size or x_vec_size != heading_vec_size or x_vec_size < 3:
    return [],[]

  x_res_vec = []
  y_res_vec = []
  heading_res_vec = []
  for i in range(1, x_vec_size - 1):
    v_1 = np.array([path_x_vec[i+1] - path_x_vec[i], path_y_vec[i+1] - path_y_vec[i]])
    v_0 = np.array([path_x_vec[i] - path_x_vec[i-1], path_y_vec[i] - path_y_vec[i-1]])
    if np.dot(v_1, v_0) < 1e-9:
      x_res_vec.append(path_x_vec[i])
      y_res_vec.append(path_y_vec[i])
      heading_res_vec.append(path_heading_vec[i])
  return x_res_vec, y_res_vec, heading_res_vec

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

def load_ego_car_given_corner_path(path_x_vec, path_y_vec, path_heading_vec, car_local_x, car_local_y, idx_vec):
  corner_path_x_vec = []
  corner_path_y_vec = []
  for idx in idx_vec:
    tmp_x_vec, tmp_y_vec = load_ego_car_corner_path(path_x_vec, path_y_vec, path_heading_vec, car_local_x[idx], car_local_y[idx])
    corner_path_x_vec.append(tmp_x_vec)
    corner_path_y_vec.append(tmp_y_vec)
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

car_idx_vec = [5]
car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type = CHERY_E0X, car_lat_inflation = 0.0)
coord_tf = coord_transformer()

data_start_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_target_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_PA = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_AB = ColumnDataSource(data=dict(x=[0], y=[0], radius=[1]))
data_BT = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_target_pos = ColumnDataSource(data = {'x':[], 'y':[]})

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

data_tra_park_out_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_adv_park_out_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

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

data_adv_arrow = ColumnDataSource(data={
    'x_start': [],
    'y_start': [],
    'x_end': [],
    'y_end': []
})

data_tra_arrow = ColumnDataSource(data={
    'x_start': [],
    'y_start': [],
    'x_end': [],
    'y_end': []
})

tra_park_out_pos_heading = 100
adv_park_out_pos_heading = 100
tra_min_corner_dist = 100
adv_min_corner_dist = 100

# fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=700, height=600, match_aspect = True, aspect_scale=1)

fig1 = bkp.figure(width=1200, height=800, match_aspect = True, aspect_scale=1)
# fig1.background_fill_color = "#E0E0E0"  # 你也可以用 '#D3D3D3'
fig1.x_range.flipped = False

fig1.outline_line_color = "black"
fig1.outline_line_width = 1.0  # 可以调整边框线条的宽度

fig1.x_range = Range1d(start = -1.1, end = 7.2)
fig1.y_range = Range1d(start = -2.1, end = 2.5)

fig1.xaxis.axis_label_text_font_size = word_size_pt
fig1.xaxis.axis_label_text_font = 'Times New Roman'

fig1.yaxis.axis_label_text_font_size = word_size_pt
fig1.yaxis.axis_label_text_font = 'Times New Roman'

fig1.xaxis.major_label_text_font_size = word_size_pt  # 设置x轴字体大小
fig1.xaxis.major_label_text_font = 'Times New Roman'      # 设置字体类型

fig1.yaxis.major_label_text_font_size = word_size_pt
fig1.yaxis.major_label_text_font = 'Times New Roman'

fig1.xaxis.ticker = SingleIntervalTicker(interval = 2, num_minor_ticks=0)
fig1.yaxis.ticker = SingleIntervalTicker(interval = 2, num_minor_ticks=0)

fig1.xgrid.grid_line_color = None
fig1.ygrid.grid_line_color = None


# # 去除图形四周边框
# fig1.outline_line_color = None
# fig1.xaxis.visible = False
# fig1.yaxis.visible = False
# fig1.xaxis.major_label_text_font_size = '0pt'  # 设置字体大小
# fig1.yaxis.major_label_text_font_size = '0pt'


# 尝试确保图表内容比例一致
aspect_ratio = (fig1.x_range.end - fig1.x_range.start) / (fig1.y_range.end - fig1.y_range.start)
fig1.plot_height = int(fig1.plot_width / aspect_ratio)

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


fig1.patch('x_vec', 'y_vec', source = data_tra_park_out_box, fill_color = 'green', fill_alpha = 0.3, line_color = "green", line_width = 1.0, visible = True)
fig1.patch('x_vec', 'y_vec', source = data_adv_park_out_box, fill_color = 'blue', fill_alpha = 0.3, line_color = "blue", line_width = 1.0, visible = True)

fig1.patches('x_vec', 'y_vec', source = data_tra_car_box, fill_color = None, fill_alpha = 0.0, line_color = "green", line_width = 1.0, visible = True)
fig1.patches('x_vec', 'y_vec', source = data_car_box, fill_color = None, fill_alpha = 0.0, line_color = "blue", line_width = 1.0, visible = True)

fig1.multi_line('x_vec','y_vec',source =data_corner_path, line_width = 3.0, line_color = 'blue', line_dash = 'solid',legend_label = 'Proposed method', visible = True)
fig1.multi_line('x_vec','y_vec',source =data_tra_corner_search_out_path,  line_width = 3.5, line_color = 'green', line_dash = 'dashed',legend_label = 'Traditional method', visible = True)

fig1.line('x_vec','y_vec',source =data_tra_search_out_path,  line_width = 3.5, line_color = 'green', line_dash = 'dashed',legend_label = 'Traditional method', visible = True)
fig1.line('x_vec','y_vec',source =data_path, line_width = 3.0, line_color = 'blue', line_dash = 'solid',legend_label = 'Proposed method', visible = True)

# target slot
# fig1.line('x_vec','y_vec',source =data_slot,  line_width = 1.0, line_color = 'black', line_dash = 'solid',legend_label = 'slot', visible = True)
fig1.multi_line('x_vec','y_vec',source =data_slot,  line_width = 0.5, line_color = 'black', line_dash = 'solid')
fig1.multi_line('x_vec','y_vec',source =data_other_slot,  line_width = 0.5, line_color = 'black', line_dash = 'solid')

# fig1.patches("x_vec", "y_vec", source=data_extra_region, color='green', fill_alpha=0.6, line_color=None, legend_label='Extra available space')
fig1.patches("x_vec", "y_vec", source=data_obs_car_polygon, color='white', fill_alpha=1, line_color=None, visible = False)

# obstacles
fig1.scatter("x_vec", "y_vec", source=data_fus_obs, size=3, color='grey', visible = True)
fig1.scatter("x_vec", "y_vec", source=data_obs_pt, size=3, color='red', visible = True)

# fig1.line('x_vec', 'y_vec', source = data_tra_tb_pt, line_width=1, line_color = 'blue', line_dash = 'solid', legend_label='Traditional boundary')
# fig1.patch('x_vec', 'y_vec', source = data_tra_tb_pt, fill_color='skyblue', fill_alpha=0.2, line_color=None)

# car box at start pose
# fig1.circle('x', 'y', source = data_start_pos, size=8, color='lightskyblue', visible = True)
# fig1.patch( 'car_xn', 'car_yn', source = data_start_car, fill_color = "lightskyblue", fill_alpha = 0.2, line_color = "black", line_width = 0.5, visible = True)

# target pose
fig1.circle('x', 'y', source = data_target_pos, size=8, color='blue')
fig1.patch( 'car_xn', 'car_yn', source = data_target_car, fill_color = None, fill_alpha = 0.0, line_color = "black", line_width = 1.5)

# fig1.scatter("x_vec", "y_vec", source=data_virtual_obs_pt, size=8, color='red', marker='star', visible = False)
fig1.circle(x = 'cx_vec', y = 'cy_vec', radius = 'radius_vec', source = data_debug_arc, line_alpha = 1, line_width = 2, line_color = "red", fill_alpha=0, visible = False)


# fig1.line('x_vec','y_vec',source =data_preparing_step_path,  line_width = 3.0, line_color = 'red', line_dash = 'solid',legend_label = 'Preparing step')
# fig1.line('x_vec','y_vec',source =data_preparing_line_path,  line_width = 3.0, line_color = 'green', line_dash = 'solid',legend_label = 'Preparing line')
# fig1.line('x_vec','y_vec',source =data_parking_out_path,  line_width = 3.0, line_color = 'blue', line_dash = 'solid',legend_label = 'Inversed parking out step')
# fig1.line('x_vec','y_vec',source =data_in_slot_path,  line_width = 3.0, line_color = 'black', line_dash = 'solid',legend_label = 'Inversed trials in slot')

adv_arraw = Arrow(end=NormalHead(fill_color="black", size=10),
               x_start='x_start', y_start='y_start',
               x_end='x_end', y_end='y_end',
               line_color="black", line_width = 0.5, source=data_adv_arrow)
fig1.add_layout(adv_arraw)

adv_arraw = Arrow(end=NormalHead(fill_color="black", size=10),
               x_start='x_end', y_start='y_end',
               x_end='x_start', y_end='y_start',
              line_color="black", line_width = 0.5,
               source=data_adv_arrow)
fig1.add_layout(adv_arraw)

tra_arraw = Arrow(end=NormalHead(fill_color="black", size=10),
               x_start='x_start', y_start='y_start',
               x_end='x_end', y_end='y_end',
                line_color="black", line_width = 0.5, source=data_tra_arrow)
fig1.add_layout(tra_arraw)

tra_arraw = Arrow(end=NormalHead(fill_color="black", size=10),
               x_start='x_end', y_start='y_end',
               x_end='x_start', y_end='y_start',
              line_color="black", line_width = 0.5,
               source=data_tra_arrow)
fig1.add_layout(tra_arraw)


fig1.legend.visible = False
fig1.legend.location = 'top_left'
fig1.legend.label_text_font_size = word_size_pt
fig1.legend.label_text_font = "Times New Roman"
fig1.legend.click_policy = 'hide'
fig1.legend.border_line_color = "white"
fig1.legend.background_fill_color = None

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
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value= 4.66, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=-180, max=180, value= 0, step=0.1)
    # slot pt
    self.slot_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot length",min=5.0, max=8.0, value=6.0, step=0.01)
    self.slot_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "width",min=2.0, max=3.0, value=2.2, step=0.01)
    # obs
    self.lon_space_dx_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "lon_space_dx",min=-1.0, max=4.0, value=lon_dist, step=0.01)
    self.curb_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "curb offset ",min=-1.0, max=1.0, value=0.3, step=0.01)
    self.channel_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel width",min=2.5, max=10.0, value=4.72, step=0.01)

    self.front_car_y_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "front obs y",min=-2.0, max=4.0, value=0.2, step=0.01)
    self.front_car_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "front obs heading",min=-180.0, max=180.0, value=0.0, step=0.1)

    self.rear_car_y_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "rear obs y",min=-2.0, max=4.0, value=0.2, step=0.01)
    self.rear_car_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "rear obs heading",min=-180.0, max=180.0, value=0.0, step=0.1)

    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "path ds",min=0.025, max=1.0, value=0.03, step=0.001)
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

  # ego start position
  data_start_pos.data.update({'x':[ego_x],'y':[ego_y]})
  # ego car
  car_xn, car_yn = load_ego_car_box(ego_x, ego_y, ego_heading_rad, car_xb, car_yb)
  data_start_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

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

  data_preparing_step_path.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  data_preparing_line_path.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  data_parking_out_path.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  data_in_slot_path.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  data_all_debug_path.data.update({
        'x_vec': [],
        'y_vec': [],
  })

  if parallel_planning_py.UpdateByJson(obs_x_vec, obs_y_vec, slot_width, slot_length, ego_x, ego_y,
                ego_heading_rad, ds):

    # tradition method
    tra_search_out_path = parallel_planning_py.GetTraSearchOutPath()
    tra_idx = next((i for i, value in enumerate(tra_search_out_path[1]) if value >= 1.5), len(tra_search_out_path[1]))
    tra_path_x_vec = tra_search_out_path[0][0:tra_idx]
    tra_path_y_vec = tra_search_out_path[1][0:tra_idx]
    tra_path_heading_vec = tra_search_out_path[2][0:tra_idx]

    data_tra_search_out_path.data.update({
      'x_vec': tra_path_x_vec,
      'y_vec': tra_path_y_vec,
    })

    tra_corner_x_vec, tra_corner_y_vec = load_ego_car_given_corner_path(tra_path_x_vec,  tra_path_y_vec, tra_path_heading_vec, car_xb, car_yb, car_idx_vec)
    data_tra_corner_search_out_path.data.update({
      'x_vec': tra_corner_x_vec,
      'y_vec': tra_corner_y_vec,
    })

    tra_shift_x_vec, tra_shift_y_vec, tra_shift_heading_vec = find_shift_points(tra_path_x_vec, tra_path_y_vec, tra_path_heading_vec)
    tra_car_box_x_vec, tra_car_box_y_vec = load_car_box(tra_shift_x_vec, tra_shift_y_vec, tra_shift_heading_vec, car_xb, car_yb)
    data_tra_car_box.data.update({
      'x_vec': tra_car_box_x_vec,
      'y_vec': tra_car_box_y_vec
    })

    park_out_pos_x = tra_shift_x_vec[-1]
    park_out_pos_y = tra_shift_y_vec[-1]
    tra_park_out_pos_heading = tra_shift_heading_vec[-1]
    tra_park_out_x_vec, tra_park_out_y_vec = load_ego_car_box(park_out_pos_x, park_out_pos_y, tra_park_out_pos_heading, car_xb, car_yb)
    data_tra_park_out_box.data.update({
      'x_vec': tra_park_out_x_vec,
      'y_vec': tra_park_out_y_vec
    })

    # 1.5 calc col dist in last circle step
    min_dist_obs = np.array([0.0, 0.0])
    tra_min_corner_dist = 100.0
    tra_path_point_size = len(tra_path_x_vec)
    center, radius = cal_common_circle(tra_path_x_vec[tra_path_point_size-3], tra_path_y_vec[tra_path_point_size-3],
                                       tra_path_x_vec[tra_path_point_size-2], tra_path_y_vec[tra_path_point_size-2],
                                       tra_path_x_vec[tra_path_point_size-1], tra_path_y_vec[tra_path_point_size-1])
    center_to_corner = np.array([car_xb[0], radius + car_yb[0]])
    radius = np.linalg.norm(center_to_corner)
    center = np.array(center)

    for i in range(len(obs_x_vec)):
      obs_pt = np.array([obs_x_vec[i], obs_y_vec[i]])
      if slot_length - 0.5 <= obs_pt[0] <= slot_length + 1.0 and 0.0 <= obs_pt[1] <= 1.5:
        v_obs_to_center = center - obs_pt
        dist = np.linalg.norm(v_obs_to_center)
        if dist < tra_min_corner_dist:
          min_dist_obs = obs_pt
          tra_min_corner_dist = dist
    tra_min_corner_dist -= radius
    v_obs_to_corner = center - min_dist_obs
    v_obs_to_corner = v_obs_to_corner / np.linalg.norm(v_obs_to_corner) * tra_min_corner_dist
    corner = v_obs_to_corner + min_dist_obs
    print("corner col det dist = ", tra_min_corner_dist)

    data_tra_arrow.data.update({
      'x_start': [corner[0]],
      'y_start': [corner[1]],
      'x_end': [min_dist_obs[0]],
      'y_end': [min_dist_obs[1]]
    })

    path_x_vec = parallel_planning_py.GetPathEle(0)
    path_y_vec = parallel_planning_py.GetPathEle(1)
    path_theta_vec = parallel_planning_py.GetPathEle(2)

    path_x_vec.append(tra_path_x_vec[0])
    path_y_vec.append(tra_path_y_vec[0])
    path_theta_vec.append(tra_path_heading_vec[0])

    target_pos_x = path_x_vec[-1]
    target_pos_y = path_y_vec[-1]
    target_pose_heading = path_theta_vec[-1]
    data_target_pos.data.update({
      'x': [target_pos_x],
      'y': [target_pos_y],
    })
    # target car
    target_car_xn, target_car_yn = load_ego_car_box(target_pos_x, target_pos_y, target_pose_heading, car_xb, car_yb)
    data_target_car.data.update({
      'car_xn': target_car_xn,
      'car_yn': target_car_yn,
    })

    ## 2. advanced reversed trials in slot
    adv_idx = next((i for i, value in enumerate(path_y_vec) if value < 1.5), len(path_y_vec))
    adv_x_vec = path_x_vec[adv_idx : -1]
    adv_y_vec = path_y_vec[adv_idx : -1]
    adv_heading_vec = path_theta_vec[adv_idx : -1]
    # 2.1 update adv path
    data_path.data.update({
      'x_vec': adv_x_vec,
      'y_vec': adv_y_vec,
      'theta_vec': adv_heading_vec,
    })
    # 2.2 adv corner
    adv_corner_x_vec, adv_corner_y_vec = load_ego_car_given_corner_path(adv_x_vec, adv_y_vec, adv_heading_vec, car_xb, car_yb, car_idx_vec)
    data_corner_path.data.update({
      'x_vec': adv_corner_x_vec,
      'y_vec': adv_corner_y_vec,
    })
    # 2.3 adv all shift pose box
    adv_shift_x_vec, adv_shift_y_vec, adv_shift_heading_vec = find_shift_points(adv_x_vec, adv_y_vec, adv_heading_vec)
    car_box_x_vec, car_box_y_vec = load_car_box(adv_shift_x_vec, adv_shift_y_vec, adv_shift_heading_vec, car_xb, car_yb)
    data_car_box.data.update({
      'x_vec': car_box_x_vec,
      'y_vec': car_box_y_vec,
    })
    # 2.4 adv parking out pose box
    park_out_pos_x = adv_shift_x_vec[0]
    park_out_pos_y = adv_shift_y_vec[0]
    adv_park_out_pos_heading = adv_shift_heading_vec[0]
    adv_park_out_x_vec, adv_park_out_y_vec = load_ego_car_box(park_out_pos_x, park_out_pos_y, adv_park_out_pos_heading, car_xb, car_yb)
    data_adv_park_out_box.data.update({
      'x_vec': adv_park_out_x_vec,
      'y_vec': adv_park_out_y_vec
    })

    # 2.5 calc col dist in last circle step
    min_dist_obs = np.array([0.0, 0.0])
    adv_min_corner_dist = 100.0
    center, radius = cal_common_circle(adv_x_vec[20], adv_y_vec[20], adv_x_vec[21], adv_y_vec[21], adv_x_vec[22], adv_y_vec[22])
    center_to_corner = np.array([car_xb[0], radius + car_yb[0]])
    radius = np.linalg.norm(center_to_corner)
    center = np.array(center)

    for i in range(len(obs_x_vec)):
      obs_pt = np.array([obs_x_vec[i], obs_y_vec[i]])
      if slot_length - 0.5 <= obs_pt[0] <= slot_length + 1.0 and 0.0 <= obs_pt[1] <= 1.5:
        v_obs_to_center = center - obs_pt
        dist = np.linalg.norm(v_obs_to_center)
        if dist < adv_min_corner_dist:
          min_dist_obs = obs_pt
          adv_min_corner_dist = dist
    adv_min_corner_dist -= radius


    v_obs_to_corner = center - min_dist_obs
    v_obs_to_corner = v_obs_to_corner / np.linalg.norm(v_obs_to_corner) * adv_min_corner_dist
    corner = v_obs_to_corner + min_dist_obs
    data_adv_arrow.data.update({
      'x_start': [corner[0]],
      'y_start': [corner[1]],
      'x_end': [min_dist_obs[0]],
      'y_end': [min_dist_obs[1]]
    })
    print("   tra  ---- vs ---- adv")
    print("parking out heading = ", tra_park_out_pos_heading *kRad2Deg, "   vs   ", adv_park_out_pos_heading *kRad2Deg)
    print("adv_min_corner_dist = ", tra_min_corner_dist, "   vs   ", adv_min_corner_dist)


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

    preparing_step_x_vec = path_x_vec[0 : preparing_step_end_idx]
    preparing_step_y_vec = path_y_vec[0 : preparing_step_end_idx]

    preparing_line_x_vec = path_x_vec[preparing_step_end_idx - 1 : preparing_line_end_idx]
    preparing_line_y_vec = path_y_vec[preparing_step_end_idx - 1 : preparing_line_end_idx]

    parking_out_x_vec = path_x_vec[preparing_line_end_idx - 1 : parking_out_end_idx]
    parking_out_y_vec = path_y_vec[preparing_line_end_idx - 1 : parking_out_end_idx]

    in_slot_x_vec = path_x_vec[parking_out_end_idx : in_slot_end_idx]
    in_slot_y_vec = path_y_vec[parking_out_end_idx : in_slot_end_idx]

    data_preparing_step_path.data.update({
      'x_vec': preparing_step_x_vec,
      'y_vec': preparing_step_y_vec,
    })

    data_preparing_line_path.data.update({
      'x_vec': preparing_line_x_vec,
      'y_vec': preparing_line_y_vec,
    })

    data_parking_out_path.data.update({
      'x_vec': parking_out_x_vec,
      'y_vec': parking_out_y_vec,
    })

    data_in_slot_path.data.update({
      'x_vec': in_slot_x_vec,
      'y_vec': in_slot_y_vec,
    })

    if is_all_path:
      parallel_planning_py.SampleAllDebugPaths()
      x_debug_paths = parallel_planning_py.GetDebugPathsX()
      y_debug_paths = parallel_planning_py.GetDebugPathsY()
      # 将 C++ 返回的二维数据转换为 Python 列表格式
      x_vec = [list(x_path) for x_path in x_debug_paths]
      y_vec = [list(y_path) for y_path in y_debug_paths]
      data_all_debug_path.data.update({
        'x_vec': x_vec,
        'y_vec': y_vec,
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



  res = parallel_planning_py.GetInverseArcVec()
  # print("arc info: cx, cy, radius, pBx, pBy")
  # print("arc info len: ", len(res))
  cx_vec = []
  cy_vec = []
  radius_vec = []
  pBx_vec = []
  pBy_vec = []

  pCx_vec = []
  pCy_vec = []

  info_len = 7
  arc_size = int(len(res) / info_len)
  for i in range(arc_size):
    # print(i, ": [", res[info_len * i], res[info_len * i + 1], res[info_len * i + 2], res[info_len * i + 3], res[info_len * i + 4], "]")
    cx_vec.append(res[info_len * i])
    cy_vec.append(res[info_len * i + 1])
    radius_vec.append(res[info_len * i + 2])

    pBx_vec.append(res[info_len * i + 3])
    pBy_vec.append(res[info_len * i + 4])

    pCx_vec.append(res[info_len * i + 5])
    pCy_vec.append(res[info_len * i + 6])

  data_debug_arc.data.update({
    'cx_vec':cx_vec,
    'cy_vec':cy_vec,
    'radius_vec':radius_vec,
    'pBx_vec':pBx_vec,
    'pBy_vec':pBy_vec,
    'pCx_vec':pCx_vec,
    'pCy_vec':pCy_vec
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
name = 'reversed_trils_in_slot_' + f"{lon_dist}m"

svg_file_path = name +'.svg'
# eps_file_path = name +'.eps'
export_svgs(fig1, filename=svg_file_path)
cairosvg.svg2pdf(url=svg_file_path, write_to=name +'.pdf')

# 使用 CairoSVG 直接转换 eps
# cairosvg.svg2eps(url=svg_file_path, write_to=eps_file_path)
# print(f"SVG 文件已直接转换为 EPS：'{eps_file_path}'")


