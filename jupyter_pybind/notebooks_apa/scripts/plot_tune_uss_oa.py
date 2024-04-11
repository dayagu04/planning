import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')
# from bokeh.models import WheelZoomTool, HoverTool, TapTool, CustomJS
# from bokeh.models import DataTable, DateFormatter, TableColumn
# from bokeh.models import TextInput
sys.path.append('python_proto')
from python_proto import common_pb2, planning_plan_pb2
from jupyter_pybind import uss_obstacle_avoidance_py

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()


# load car local vertex
car_local_x_list, car_local_y_list = load_car_params_patch()
# load uss local vertex and normal angle
uss_local_x_list, uss_local_y_list = load_car_uss_patch()
uss_angle_list = load_uss_angle_patch()

data_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})

data_car_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_car_line = ColumnDataSource(data = {'x':[], 'y':[]})

data_car_arc = ColumnDataSource(data = {'x':[], 'y':[], 'r':[]})

data_wave = ColumnDataSource(data = {'wave_x': [], 'wave_y': [], 'radius':[], 'start_angle':[], 'end_angle':[]})

data_min_wave = ColumnDataSource(data = {'wave_x': [], 'wave_y': [], 'radius':[], 'start_angle':[], 'end_angle':[]})

data_min_pos = ColumnDataSource(data = {'x':[], 'y':[]})

fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=960, height=640, match_aspect = True, aspect_scale=1)
fig1.patch('car_xn', 'car_yn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
fig1.circle('x','y', source = data_car_start_pos, size=8, color='red', legend_label = 'car_start_pos')
fig1.line('x', 'y', source = data_car_line, line_width = 2.0, line_color = 'blue', line_dash = 'solid', legend_label = 'car_line')
fig1.wedge('wave_x','wave_y', 'radius', 'start_angle', 'end_angle',source = data_wave, fill_color="lavender", line_color="black",legend_label = 'uss_wave',alpha = 0.5)
fig1.circle(x ='x', y ='y', radius = 'r', source=data_car_arc, line_alpha = 0.5, line_width = 2, line_color = "blue", fill_alpha=0, legend_label = 'car_arc', visible = True)

fig1.wedge('wave_x','wave_y', 'radius', 'start_angle', 'end_angle',source = data_min_wave, fill_color="grey", line_color="black",legend_label = 'uss_min_wave',alpha = 1.0)
fig1.circle('x','y', source = data_min_pos, size=8, color='blue', legend_label = 'car_min_pos')


fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

columns = [
  TableColumn(field="names", title="names",),
  TableColumn(field="datas", title="datas"),
]
debug_data = ColumnDataSource(data = {'names':[], 'datas':[]})
debug_table = DataTable(source=debug_data, columns=columns, width=600, height=500)

uss_obstacle_avoidance_py.Init()

class LocalViewSlider:
  def __init__(self,  slider_callback):

    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-10, max=10, value=0.0, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value=0.0, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=0, max=360, value=90.0, step=1)

    self.steer_angle_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "steer_angle",min=-500, max=500, value=0.0, step=50.0)
    self.forward_back_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "forward_back",min=0, max=1, value=0, step=1)

    self.detection_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "detection_distance",min=0, max=2.6, value=2.5, step=0.01)

    self.lat_inflation_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "lat_inflation",min=0, max=0.3, value=0.18, step=0.01)

    self.uss_raw_dist_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "uss_raw_dist",min=0.15, max=4.7, value=1.98, step=0.01)

    ipywidgets.interact(slider_callback, ego_x = self.ego_x_slider,
                                         ego_y = self.ego_y_slider,
                                         ego_heading = self.ego_heading_slider,
                                         steer_angle = self.steer_angle_slider,
                                         forward_back = self.forward_back_slider,
                                         detection_distance = self.detection_distance_slider,
                                         lat_inflation = self.lat_inflation_slider,
                                         uss_raw_dist = self.uss_raw_dist_slider,)

### sliders callback
def slider_callback(ego_x, ego_y, ego_heading, steer_angle, forward_back, detection_distance, lat_inflation, uss_raw_dist):
  kwargs = locals()

  # ego_heading is car position heading Angle, not necessarily car speed heading angle
  x_start = ego_x
  y_start = ego_y

  # update start point pos
  data_car_start_pos.data.update({
    'x': [x_start],
    'y': [y_start],
  })


  # update ego car pos
  car_global_x_list = []
  car_global_y_list = []
  for i in range(len(car_local_x_list)):
      car_global_x, car_global_y = local2global(car_local_x_list[i], car_local_y_list[i], x_start, y_start, math.radians(ego_heading))
      car_global_x_list.append(car_global_x)
      car_global_y_list.append(car_global_y)
  data_car.data.update({
    'car_xn': car_global_x_list,
    'car_yn': car_global_y_list,
  })

  uss_global_x_list, uss_global_y_list, uss_radius_list, uss_angle_start_list, uss_angle_end_list = [], [], [], [], []
  for i in range(len(uss_local_x_list)):
    uss_global_x, uss_global_y = local2global(uss_local_x_list[i], uss_local_y_list[i], ego_x, ego_y, math.radians(ego_heading))

    uss_scan_angle = 60
    if i == 1 or i == 4 or i == 7 or i == 10:
      uss_scan_angle *= 1.4

    uss_angle_start = math.radians(uss_angle_list[i] - uss_scan_angle / 2 + ego_heading - 90)
    uss_angle_end = math.radians(uss_angle_list[i] + uss_scan_angle / 2 + ego_heading - 90)

    uss_global_x_list.append(uss_global_x)
    uss_global_y_list.append(uss_global_y)
    uss_radius_list.append(uss_raw_dist)
    uss_angle_start_list.append(uss_angle_start)
    uss_angle_end_list.append(uss_angle_end)


  data_wave.data.update({
    'wave_x':uss_global_x_list,
    'wave_y':uss_global_y_list,
    'radius':uss_radius_list,
    'start_angle':uss_angle_start_list,
    'end_angle':uss_angle_end_list,
  })

  uss_obstacle_avoidance_py.SetParam(detection_distance, lat_inflation)
  uss_obstacle_avoidance_py.SetUssRawDist(uss_raw_dist)
  uss_obstacle_avoidance_py.SetCarMotionInfo(math.radians(steer_angle), forward_back)
  uss_obstacle_avoidance_py.UpdateUssDis();

  remain_dist = uss_obstacle_avoidance_py.GetRemainDist()
  available_flag = uss_obstacle_avoidance_py.GetAvailable();
  uss_index = uss_obstacle_avoidance_py.GetUssIndex()
  car_index = uss_obstacle_avoidance_py.GetCarIndex()
  if available_flag == False:
    uss_index = -1
    car_index = -1
  names = []
  datas = []
  names.append("available_flag")
  datas.append(available_flag)
  names.append("remain_dist")
  datas.append(remain_dist)
  names.append("uss_index")
  datas.append(uss_index)
  names.append("car_index")
  datas.append(car_index)
  debug_data.data.update({
    'names': names,
    'datas': datas,
  })

  if available_flag == True:
    data_min_wave.data.update({
      'wave_x':[uss_global_x_list[uss_index]],
      'wave_y':[uss_global_y_list[uss_index]],
      'radius':[uss_radius_list[uss_index]],
      'start_angle':[uss_angle_start_list[uss_index]],
      'end_angle':[uss_angle_end_list[uss_index]],
    })

    data_min_pos.data.update({
      'x': [car_global_x_list[car_index]],
      'y': [car_global_y_list[car_index]],
    })
  else:
    data_min_wave.data.update({
      'wave_x':[],
      'wave_y':[],
      'radius':[],
      'start_angle':[],
      'end_angle':[],
    })

    data_min_pos.data.update({
      'x': [],
      'y': [],
    })


  arc_line_shift_steer_angle_deg = 2.5

  if math.fabs(steer_angle) < arc_line_shift_steer_angle_deg:
    car_line = uss_obstacle_avoidance_py.GetCarLine()

    car_local_line_x_list = [car_line[0][0], car_line[1][0]]
    car_local_line_y_list = [car_line[0][1], car_line[1][1]]

    car_global_line_x_list, car_global_line_y_list = [], []
    for i in range(len(car_local_line_x_list)):
      car_global_line_x, car_global_line_y = local2global(car_local_line_x_list[i], car_local_line_y_list[i], x_start, y_start, math.radians(ego_heading))
      car_global_line_x_list.append(car_global_line_x)
      car_global_line_y_list.append(car_global_line_y)

    data_car_line.data.update({
      'x': car_global_line_x_list,
      'y': car_global_line_y_list,
    })

    data_car_arc.data.update({
      'x': [],
      'y': [],
      'r': [],
    })
  else:
    car_arc = uss_obstacle_avoidance_py.GetCarArc()
    car_arc_local_x = car_arc[0]
    car_arc_local_y = car_arc[1]
    car_arc_radius = car_arc[2]

    car_arc_global_x, car_arc_global_y = local2global(car_arc_local_x, car_arc_local_y, x_start, y_start, math.radians(ego_heading))

    data_car_line.data.update({
      'x': [],
      'y': [],
    })

    data_car_arc.data.update({
      'x': [car_arc_global_x],
      'y': [car_arc_global_y],
      'r': [car_arc_radius],
    })

  push_notebook()

bkp.show(row(fig1,debug_table), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
