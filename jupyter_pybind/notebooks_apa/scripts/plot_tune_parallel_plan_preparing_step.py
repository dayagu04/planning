from math import cos, fabs, pi, sin
import sys, os
sys.path.append("..")

sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

from contruct_scenario import construct_scenario
from lib.load_local_view_parking import *
from bokeh.events import Tap
from bokeh.models import Range1d
from bokeh.io import export_svgs
from jupyter_pybind import parallel_preparing_planning_py
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

case = 'SCC'

if case == 'S-C':
  prepare_x0 = 2.6
  prepare_y0 = 3.47
  prepare_heading0_deg = 0.0
  ego_x0 = 10.41
  ego_y0 = 3.89
  ego_heading0 = 29.9
elif case == 'SC':
  prepare_x0 = 1.75
  prepare_y0 = 2.73
  prepare_heading0_deg = 0.0
  ego_x0 = 2.2
  ego_y0 = 1.37
  ego_heading0 = 29.9
elif case == 'C-C':
  prepare_x0 = 2.6
  prepare_y0 = 3.04
  prepare_heading0_deg = 0.0
  ego_x0 = 5.98
  ego_y0 = 3.31
  ego_heading0 = 38.1
elif case == 'CC':
  prepare_x0 = 2.6
  prepare_y0 = 3.04
  prepare_heading0_deg = 0.0
  ego_x0 = 0.42
  ego_y0 = 2.51
  ego_heading0 = -14.6
elif case == 'CSC':
  prepare_x0 = 6.65
  prepare_y0 = 2.98
  prepare_heading0_deg = 14.38
  ego_x0 = 1.1
  ego_y0 = 2.95
  ego_heading0 = -14.6
elif case == 'C-SC':
  prepare_x0 = 6.07
  prepare_y0 = 3.34
  prepare_heading0_deg = 18.49
  ego_x0 = 3.71
  ego_y0 = 3.32
  ego_heading0 = 27.8
elif case == 'S-CC':
  prepare_x0 = 9.53
  prepare_y0 = 3.39
  prepare_heading0_deg = 10.01
  ego_x0 = 5.49
  ego_y0 = 4.52
  ego_heading0 = 4.70
elif case == 'SCC':
  prepare_x0 = 7.31
  prepare_y0 = 2.79
  prepare_heading0_deg = 10.01
  ego_x0 = 3.13
  ego_y0 = 2.87
  ego_heading0 = 4.40
elif case == 'CCC':
  prepare_x0 = 7.31
  prepare_y0 = 2.79
  prepare_heading0_deg = 10.01
  ego_x0 = 3.13
  ego_y0 = 2.87
  ego_heading0 = 4.40

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

car_xb, car_yb, wheel_base = load_car_params_patch_parking(vehicle_type = CHERY_E0X, car_lat_inflation = 0.0)
coord_tf = coord_transformer()

data_start_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_target_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_PA = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_AB = ColumnDataSource(data=dict(x=[0], y=[0], radius=[1]))
data_BT = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_prepare_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_target_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_preparing_line = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'theta_vec':[]})
data_corner_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_preparing_step_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_preparing_line_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_preparing_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
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

# fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=700, height=600, match_aspect = True, aspect_scale=1)

fig1 = bkp.figure(width=1200, height=800, match_aspect = True, aspect_scale=1)
# fig1.background_fill_color = "#E0E0E0"  # 你也可以用 '#D3D3D3'

word_size = '22pt'

fig1.x_range.flipped = False
fig1.outline_line_color = "black"
fig1.outline_line_width = 1.0  # 可以调整边框线条的宽度

fig1.x_range = Range1d(start = -1.0, end = 16.7)
fig1.y_range = Range1d(start = -2.2, end = 9.2)

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
fig1.xaxis.visible = False
fig1.yaxis.visible = False
fig1.xaxis.major_label_text_font_size = '0pt'  # 设置字体大小
fig1.yaxis.major_label_text_font_size = '0pt'


# 尝试确保图表内容比例一致
aspect_ratio = (fig1.x_range.end - fig1.x_range.start) / (fig1.y_range.end - fig1.y_range.start)
fig1.plot_height = int(fig1.plot_width / aspect_ratio)

fig1.patches('x_vec', 'y_vec', source = data_car_box, fill_color = "lightskyblue", fill_alpha=0.3, line_color = None)
# car box at start pose

fig1.patch( 'car_xn', 'car_yn', source = data_start_car, fill_color = 'blue', fill_alpha = 0.2, line_color = "blue", line_width = 1.0, visible = True)

# target pose
fig1.circle('x', 'y', source = data_target_pos, size=8, color='green')
fig1.patch( 'car_xn', 'car_yn', source = data_target_car, fill_color = 'green', fill_alpha = 0.5, line_color = "green", line_width = 1.0)

fig1.multi_line('x_vec','y_vec',source =data_corner_path, line_width = 3.0, line_color = 'blue', line_dash = 'dashed', visible = True)
# fig1.line('x_vec','y_vec',source =data_path, line_width = 3.0, line_color = 'red', line_dash = 'solid', visible = True)
fig1.line('x_vec','y_vec',source =data_preparing_path,  line_width = 3.0, line_color = 'red', line_dash = 'solid')

# target slot
# fig1.line('x_vec','y_vec',source =data_slot,  line_width = 1.0, line_color = 'black', line_dash = 'solid',legend_label = 'slot', visible = True)
fig1.multi_line('x_vec','y_vec',source =data_slot,  line_width = 0.5, line_color = 'black', line_dash = 'solid')
fig1.multi_line('x_vec','y_vec',source =data_other_slot,  line_width = 0.5, line_color = 'black', line_dash = 'solid')

fig1.patches("x_vec", "y_vec", source=data_obs_car_polygon, color='white', fill_alpha=1, line_color=None, visible = False)

# obstacles
fig1.scatter("x_vec", "y_vec", source=data_fus_obs, size=3, color='grey', visible = True)
fig1.scatter("x_vec", "y_vec", source=data_obs_pt, size=3, color='red', visible = True)


fig1.line('x_vec','y_vec',source = data_preparing_line, line_width = 1.5, line_color = 'green', line_dash = 'dashed', visible = True)

fig1.circle('x', 'y', source = data_start_pos, size=8, color='blue', visible = True)
# fig1.circle('x', 'y', source = data_prepare_pos, size=8, color='red', visible = True)

# fig1.scatter("x_vec", "y_vec", source=data_virtual_obs_pt, size=8, color='red', marker='star', visible = False)
# measure tool
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

fig1.legend.visible = False
fig1.legend.location = 'top_left'
fig1.legend.label_text_font_size = word_size
fig1.legend.label_text_font = "Times New Roman"
fig1.legend.click_policy = 'hide'

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)

parallel_preparing_planning_py.Init()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.is_front_occupied_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is front occupied",min=0, max=1, value= 1, step=1)
    self.is_rear_occupied_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is rear occupied",min=0, max=1, value= 1, step=1)
    self.is_all_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "all debug path",min=0, max=1, value= 0, step=1)

    self.preparing_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "preparing x",min=-8, max=16, value= prepare_x0, step=0.01)
    self.preparing_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "preparing y",min=0, max=10, value= prepare_y0, step=0.01)
    self.preparing_heading_deg_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "preparing heading",min=-180, max=180, value= prepare_heading0_deg, step=0.01)

    # ego pose
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-15, max=15, value= ego_x0, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value= ego_y0, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=-180, max=180, value= ego_heading0, step=0.1)
    # slot pt
    self.slot_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot length",min=5.0, max=8.0, value=6.0, step=0.01)
    self.slot_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "width",min=2.0, max=3.0, value=2.2, step=0.01)
    # obs
    self.lon_space_dx_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "lon_space_dx",min=-1.0, max=4.0, value=0.0, step=0.01)
    self.curb_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "curb offset ",min=-1.0, max=1.0, value=0.3, step=0.01)
    self.channel_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel width",min=2.5, max=10.0, value=4.34, step=0.01) # 3.14

    self.front_car_y_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "front obs y",min=-2.0, max=4.0, value=0.65, step=0.01)
    self.front_car_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "front obs heading",min=-180.0, max=180.0, value=-13.0, step=0.1)

    self.rear_car_y_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "rear obs y",min=-2.0, max=4.0, value=0.2, step=0.01)
    self.rear_car_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "rear obs heading",min=-180.0, max=180.0, value=0.0, step=0.1)

    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "path ds",min=0.01, max=1.0, value=0.01, step=0.001)
    ipywidgets.interact(slider_callback,
                                         is_front_occupied = self.is_front_occupied_slider,
                                         is_rear_occupied = self.is_rear_occupied_slider,
                                         is_all_path = self.is_all_path_slider,
                                         preparing_x = self.preparing_x_slider,
                                         preparing_y = self.preparing_y_slider,
                                         preparing_heading_deg = self.preparing_heading_deg_slider,

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
def slider_callback(is_front_occupied, is_rear_occupied, is_all_path, preparing_x, preparing_y, preparing_heading_deg, ego_x, ego_y, ego_heading,
                    slot_length, slot_width, lon_space_dx, curb_offset, channel_width,
                    front_car_y_offset, front_car_heading, rear_car_y_offset, rear_car_heading,
                    ds):
  kwargs = locals()

  print("before construct_scenario")

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


  data_preparing_path.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  # ego start position
  data_start_pos.data.update({'x':[ego_x],'y':[ego_y]})
  car_xn, car_yn = load_ego_car_box(ego_x, ego_y, ego_heading_rad, car_xb, car_yb)
  data_start_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  preparing_heading = preparing_heading_deg * kDeg2Rad
  preparing_heading_vec =np.array([cos(preparing_heading), sin(preparing_heading)])
  preparing_pos = np.array([preparing_x, preparing_y])
  preparing_line_pt1 = preparing_pos - 6 * preparing_heading_vec
  preparing_line_pt2 = preparing_pos + 6 * preparing_heading_vec

  data_prepare_pos.data.update({
    'x': [preparing_x],
    'y': [preparing_y]
  })

  if preparing_heading_deg == 0.0:
    data_preparing_line.data.update({
      'x_vec': [-0.5, 16.0],
      'y_vec': [preparing_y, preparing_y]
    })
  else:
    data_preparing_line.data.update({
      'x_vec': [preparing_line_pt1[0], preparing_line_pt2[0]],
      'y_vec': [preparing_line_pt1[1], preparing_line_pt2[1]]
    })

  if parallel_preparing_planning_py.UpdateByJson(obs_x_vec, obs_y_vec, slot_width, slot_length, ego_x, ego_y,
                ego_heading_rad, ds, preparing_x, preparing_y, preparing_heading_deg *kDeg2Rad):
    preparing_x_vec = parallel_preparing_planning_py.GetDebugPathsX()
    preparing_y_vec = parallel_preparing_planning_py.GetDebugPathsY()
    preparing_theta_vec = parallel_preparing_planning_py.GetDebugPathsHeading()

    if (len(preparing_x_vec) > 0):
      # car path
      data_preparing_path.data.update({
        'x_vec': preparing_x_vec,
        'y_vec': preparing_y_vec,
      })
      # car box envelope
      parking_out_car_box_x_vec, parking_out_car_box_y_vec = load_car_box(preparing_x_vec, preparing_y_vec, preparing_theta_vec, car_xb, car_yb)

      data_car_box.data.update({
        'x_vec': parking_out_car_box_x_vec,
        'y_vec': parking_out_car_box_y_vec,
      })


      idx_vec = []
      if case == 'S-C':
        idx_vec = [0, 5, 10, 15, 17]
      elif case == 'SC':
        idx_vec = [0, 8, 18]
      elif case == 'C-C':
        idx_vec = [0, 10, 15]
      elif case == 'CC':
        idx_vec = [0, 5, 17]
      elif case == 'CSC':
        idx_vec = [5, 7, 17]
      elif case == 'C-SC':
        idx_vec = [5, 7, 10, 15]
      elif case == 'SCC':
        idx_vec = [0, 5, 8, 17]
      elif case == 'S-CC':
        idx_vec = [0, 5, 8, 10, 15, 17]

      corner_x_vec, corner_y_vec = load_ego_car_given_corner_path(preparing_x_vec, preparing_y_vec, preparing_theta_vec, car_xb, car_yb, idx_vec)

      if case == 'SC':
        rs_x_vec, rs_y_vec = load_ego_car_corner_path(preparing_x_vec, preparing_y_vec, preparing_theta_vec, 0.0, car_yb[5])
        corner_x_vec.append(rs_x_vec)
        corner_y_vec.append(rs_y_vec)
      elif case == 'CC' or case == 'CSC':
        rs_x_vec, rs_y_vec = load_ego_car_corner_path(preparing_x_vec, preparing_y_vec, preparing_theta_vec, 0.0, car_yb[0])
        corner_x_vec.append(rs_x_vec)
        corner_y_vec.append(rs_y_vec)

      print("corner_x_vec = ", corner_x_vec)
      data_corner_path.data.update({
        'x_vec': corner_x_vec,
        'y_vec': corner_y_vec
      })




      # target pose
      data_target_pos.data.update({
        'x': [preparing_x_vec[-1]],
        'y': [preparing_y_vec[-1]],
      })
      target_car_xn, target_car_yn = load_ego_car_box(preparing_x_vec[-1], preparing_y_vec[-1], preparing_theta_vec[-1], car_xb, car_yb)
      data_target_car.data.update({
        'car_xn': target_car_xn,
        'car_yn': target_car_yn,
      })

  obs_in_tboundary = parallel_preparing_planning_py.GetParkPlannerObs()
  # obstacles
  data_obs_pt.data.update({
    'x_vec': obs_in_tboundary[0],
    'y_vec': obs_in_tboundary[1],
  })
  data_virtual_obs_pt.data.update({
    'x_vec': parallel_preparing_planning_py.GetVirtualObsX(),
    'y_vec': parallel_preparing_planning_py.GetVirtualObsY(),
  })

  data_tra_tb_pt.data.update({
  'x_vec':[],
  'y_vec':[]
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
  print("rear_obs_car_matrix size = ", len(rear_obs_car_matrix))

  if len(rear_obs_car_matrix) > 0:
    print("rear_obs_car_ obs size = ", len(rear_obs_car_matrix[0]))
    points = np.column_stack((rear_obs_car_matrix[0], rear_obs_car_matrix[1]))
    hull = ConvexHull(points)
    polygon = Polygon(points[hull.vertices])
    if polygon.geom_type == 'Polygon':
      tmp_x, tmp_y = polygon.exterior.xy
      obs_car_polygon_x_vec.append(list(tmp_x))
      obs_car_polygon_y_vec.append(list(tmp_y))

  front_obs_car_matrix = json_data["front_obs_car_matrix"]
  print("front_obs_car_matrix size = ", len(front_obs_car_matrix))

  if len(front_obs_car_matrix) > 0:
    print("front_obs_car obs size = ", len(front_obs_car_matrix[0]))
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
name = case

svg_file_path = name +'.svg'
eps_file_path = name +'.eps'
export_svgs(fig1, filename=svg_file_path)

# 使用 CairoSVG 直接转换
cairosvg.svg2eps(url=svg_file_path, write_to=eps_file_path)

print(f"SVG 文件已直接转换为 EPS：'{eps_file_path}'")

cairosvg.svg2pdf(url=svg_file_path, write_to=name +'.pdf')
