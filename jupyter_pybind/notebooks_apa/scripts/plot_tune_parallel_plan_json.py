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
from jupyter_pybind import parallel_planning_py
from bokeh.models import SingleIntervalTicker
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
import os
import sys
import cairosvg
# sys.path.append('/root/miniconda3/lib/python3.7/site-packages')

kRad2Deg = 180.0 / pi
kDeg2Rad = pi / 180.0

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

data_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'theta_vec':[]})
data_preparing_step_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_preparing_line_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_parking_out_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_in_slot_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_tra_search_out_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_all_debug_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_all_prepare_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_car_in_slot_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

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

fig1.xaxis.ticker = SingleIntervalTicker(interval = 1, num_minor_ticks=0)
fig1.yaxis.ticker = SingleIntervalTicker(interval = 1, num_minor_ticks=0)

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

# start drawing debug info
fig1.line('x_vec','y_vec',source =data_tra_search_out_path,  line_width = 3.0, line_color = 'red', line_dash = 'solid',legend_label = 'tra_search_out', visible = False)
fig1.multi_line('x_vec', 'y_vec', source = data_all_debug_path, line_width = 2, line_color = 'orange', line_dash = 'solid',legend_label = 'all debug path', visible = True)
fig1.multi_line('x_vec', 'y_vec', source = data_car_in_slot_path, line_width = 1.0, line_color = 'blue', line_dash = 'solid',legend_label = 'car in slot line', visible = True)
# fig1.scatter("x_vec", "y_vec", source=data_all_prepare_path, size=1, color='grey',legend_label = 'all prepare line pt')
fig1.multi_line('x_vec', 'y_vec', source = data_all_prepare_path, line_width = 1.0, line_color = 'pink', line_dash = 'dashed',legend_label = 'all prepare line pt', visible = True)
fig1.line('x_vec','y_vec',source =data_path,  line_width = 3.0, line_color = 'green', line_dash = 'solid',legend_label = 'Car Path', visible = True)

# target slot
# fig1.patch('x_vec', 'y_vec', source = data_slot, fill_color='blue', line_color='grey', fill_alpha=0.3, line_width = 0.3, legend_label = 'Target slot')
fig1.line('x_vec','y_vec',source =data_slot,  line_width = 2.0, line_color = 'black', line_dash = 'solid',legend_label = 'slot', visible = True)

# nearby slots
# fig1.patches('x_vec', 'y_vec', source = data_other_slot, fill_color='grey', line_color='grey', fill_alpha=0.2, line_width = 0.3, legend_label = 'Nearby slots')
fig1.multi_line('x_vec','y_vec',source =data_other_slot,  line_width = 2.0, line_color = 'black', line_dash = 'solid',legend_label = 'slot', visible = True)

fig1.patches("x_vec", "y_vec", source=data_extra_region, color='green', fill_alpha=0.6, line_color=None, legend_label='Extra available space')
fig1.patches("x_vec", "y_vec", source=data_obs_car_polygon, color='white', fill_alpha=1, line_color=None, legend_label='obs obstacles')

# obstacles
fig1.scatter("x_vec", "y_vec", source=data_fus_obs, size=3, color='grey',legend_label = 'External obstacles')
fig1.scatter("x_vec", "y_vec", source=data_obs_pt, size=3, color='red',legend_label = 'Internal obstacles')

# fig1.line('x_vec', 'y_vec', source = data_tra_tb_pt, line_width=1, line_color = 'blue', line_dash = 'solid', legend_label='Traditional boundary')
fig1.patch('x_vec', 'y_vec', source = data_tra_tb_pt, fill_color='skyblue', fill_alpha=0.2, line_color=None)

fig1.patches('x_vec', 'y_vec', source = data_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 0.2, legend_label = 'Envelope')
# car box at start pose
fig1.circle('x', 'y', source = data_start_pos, size=8, color='black', legend_label = 'Start pose')
fig1.patch( 'car_xn', 'car_yn', source = data_start_car, fill_color = "red", fill_alpha = 0.2, line_color = "black", line_width = 0.5, legend_label = 'Start pose')

# target pose
fig1.circle('x', 'y', source = data_target_pos, size=8, color='black', legend_label = 'Target pose')
fig1.patch( 'car_xn', 'car_yn', source = data_target_car, fill_color = "red", fill_alpha = 0.3, line_color = "black", line_width = 1, legend_label = 'Target pose')

fig1.scatter("x_vec", "y_vec", source=data_virtual_obs_pt, size=8, color='red', marker='star', legend_label = 'virtual obstacles', visible = False)
fig1.circle(x = 'cx_vec', y = 'cy_vec', radius = 'radius_vec', source = data_debug_arc, line_alpha = 1, line_width = 2, line_color = "red", fill_alpha=0, legend_label = 'data_debug_arc', visible = False)


# fig1.line('x_vec','y_vec',source =data_preparing_step_path,  line_width = 3.0, line_color = 'red', line_dash = 'solid',legend_label = 'Preparing step')
# fig1.line('x_vec','y_vec',source =data_preparing_line_path,  line_width = 3.0, line_color = 'green', line_dash = 'solid',legend_label = 'Preparing line')
# fig1.line('x_vec','y_vec',source =data_parking_out_path,  line_width = 3.0, line_color = 'blue', line_dash = 'solid',legend_label = 'Inversed parking out step')
# fig1.line('x_vec','y_vec',source =data_in_slot_path,  line_width = 3.0, line_color = 'black', line_dash = 'solid',legend_label = 'Inversed trials in slot')


fig1.legend.label_text_font = "Times New Roman"  # 设置图例字体类型
fig1.legend.label_text_font_size = '14pt'  # 设置图例字体大小
fig1.legend.location = 'top_left'

# fig1.legend.visible = False

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'
parallel_planning_py.Init()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.is_front_occupied_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is front occupied",min=0, max=1, value= 1, step=1)
    self.is_rear_occupied_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is rear occupied",min=0, max=1, value= 1, step=1)
    self.is_all_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "all debug path",min=0, max=1, value= 0, step=1)
    # ego pose
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-15, max=15, value= -1.11, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value= 2.33, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=-180, max=180, value= 0, step=0.1)
    # slot pt
    self.slot_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "slot length",min=5.0, max=8.0, value=6.0, step=0.01)
    self.slot_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "width",min=2.0, max=3.0, value=2.2, step=0.01)
    # obs
    self.lon_space_dx_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "lon_space_dx",min=-1.0, max=4.0, value=0.79, step=0.01)
    self.curb_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "curb offset ",min=-1.0, max=1.0, value=0.3, step=0.01)
    self.channel_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel width",min=2.5, max=10.0, value=6.25, step=0.01)

    self.front_car_y_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "front obs y",min=-2.0, max=4.0, value=2.16, step=0.01)
    self.front_car_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "front obs heading",min=-180.0, max=180.0, value=0.0, step=0.1)

    self.rear_car_y_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "rear obs y",min=-2.0, max=4.0, value=0.0, step=0.01)
    self.rear_car_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "rear obs heading",min=-180.0, max=180.0, value=0.0, step=0.1)

    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "path ds",min=0.025, max=1.0, value=0.2, step=0.025)
    self.obs_ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "obs ds",min=0.025, max=1.0, value=0.38, step=0.025)
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
  slot_x_vec = [slot_length, 0.0, 0.0, slot_length]
  slot_y_vec = [half_slot_width_sgn, half_slot_width_sgn, -half_slot_width_sgn, -half_slot_width_sgn]

  data_slot.data.update({
     'x_vec': slot_x_vec,
     'y_vec': slot_y_vec})

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
  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_x, ego_y,
                                  ego_heading_rad)
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)

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
  data_all_prepare_path.data.update({
        'x_vec': [],
        'y_vec': [],
  })
  data_car_in_slot_path.data.update({
      'x_vec': [],
      'y_vec': [],
  })

  if parallel_planning_py.UpdateByJson(obs_x_vec, obs_y_vec, slot_width, slot_length, ego_x, ego_y,
                ego_heading_rad, ds):

    path_x_vec = parallel_planning_py.GetPathEle(0)
    path_y_vec = parallel_planning_py.GetPathEle(1)
    path_theta_vec = parallel_planning_py.GetPathEle(2)

    target_pos_x = path_x_vec[-1]
    target_pos_y = path_y_vec[-1]
    data_target_pos.data.update({
      'x': [target_pos_x],
      'y': [target_pos_y],
    })
    # target car
    target_car_xn = []
    target_car_yn = []
    for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], target_pos_x, target_pos_y, 0.00)
        target_car_xn.append(tmp_x)
        target_car_yn.append(tmp_y)
    data_target_car.data.update({
      'car_xn': target_car_xn,
      'car_yn': target_car_yn,
    })

    data_path.data.update({
      'x_vec': path_x_vec,
      'y_vec': path_y_vec,
      'theta_vec': path_theta_vec,
    })


    # path ego car
    car_box_x_vec = []
    car_box_y_vec = []
    for k in range(len(path_x_vec)):
      car_xn = []
      car_yn = []
      for i in range(len(car_xb)):
          tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], path_x_vec[k], path_y_vec[k], path_theta_vec[k])
          car_xn.append(tmp_x)
          car_yn.append(tmp_y)
      car_box_x_vec.append(car_xn)
      car_box_y_vec.append(car_yn)

    data_car_box.data.update({
      'x_vec': car_box_x_vec,
      'y_vec': car_box_y_vec,
    })

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

      x_vec = [list(x_path) for x_path in x_debug_paths]
      y_vec = [list(y_path) for y_path in y_debug_paths]
      data_all_debug_path.data.update({
        'x_vec': x_vec,
        'y_vec': y_vec,
      })

    tra_search_out_path = parallel_planning_py.GetTraSearchOutPath()
    data_tra_search_out_path.data.update({
      'x_vec': tra_search_out_path[0],
      'y_vec': tra_search_out_path[1],
    })

  # prepare_pat = parallel_planning_py.GetPreparePath()
  # print(f"len: {len(prepare_pat)} x: {prepare_pat[0][0]} y: {prepare_pat[1][0]}")
  # data_all_prepare_path.data.update({
  #   'x_vec': [list(x_path) for x_path in prepare_pat[0]],
  #   'y_vec': [list(y_path) for y_path in prepare_pat[1]],
  # })
  car_in_slot_path = parallel_planning_py.GetDebugInSlotPath()
  data_car_in_slot_path.data.update({
    'x_vec': [list(x_path) for x_path in car_in_slot_path[0]],
    'y_vec': [list(y_path) for y_path in car_in_slot_path[1]],
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
  print("arc info: cx, cy, radius, pBx, pBy")
  print("arc info len: ", len(res))
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
    print(i, ": [", res[info_len * i], res[info_len * i + 1], res[info_len * i + 2], res[info_len * i + 3], res[info_len * i + 4], "]")
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

# os.environ['WEB_BROWSER'] = 'chrome'
# fig1.output_backend = "svg"
# export_svgs(fig1, filename="illustration.svg")

# svg_file_path = 'illustration.svg'
# eps_file_path = 'illustration.eps'

# # 使用 CairoSVG 直接转换
# cairosvg.svg2eps(url=svg_file_path, write_to=eps_file_path)

# print(f"SVG 文件已直接转换为 EPS：'{eps_file_path}'")

# cairosvg.svg2pdf(url="illustration.svg", write_to="illustration.pdf")
