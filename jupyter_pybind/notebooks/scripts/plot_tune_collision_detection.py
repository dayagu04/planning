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
from jupyter_pybind import collision_detection_py

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

car_xb, car_yb = load_car_params_patch()

coord_tf = coord_transformer()

data_car = ColumnDataSource(data = {'car_xn':[], 'car_yn':[]})
data_car_circle = ColumnDataSource(data = {'circle_xn':[], 'circle_yn':[], 'circle_rn':[]})

#data_car_circle = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'r_vec':[]})
data_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_target_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'theta_vec':[]})
data_obstacle_line = ColumnDataSource(data = {'x':[], 'y':[]})

fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=960, height=640, match_aspect = True, aspect_scale=1)
#fig1.x_range.flipped = True
fig1.patch('car_xn', 'car_yn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')

fig1.circle(x ='circle_xn', y ='circle_yn', radius = 'circle_rn', source=data_car_circle, line_alpha = 0.5, line_width = 1, line_color = "blue", fill_alpha=0, legend_label = 'car_circle', visible = True)

fig1.circle('x','y', source = data_start_pos, size=8, color='red', legend_label = 'start_pos')
fig1.circle('x','y', source = data_target_pos, size=8, color='blue', legend_label = 'target_pos')

fig1.line('x_vec', 'y_vec', source = data_path, line_width = 8, line_color = 'green', line_dash = 'solid', line_alpha = 0.4,legend_label = 'sampled path')
# fig1.circle('y_vec','x_vec', source = data_path, size=4, color='green', legend_label = 'sampled path')

fig1.line('x', 'y', source = data_obstacle_line, line_width = 2, line_color = 'grey', line_dash = 'solid', line_alpha = 0.4,legend_label = 'obstacle_line')


fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

columns = [
  TableColumn(field="names", title="names",),
  TableColumn(field="datas", title="datas"),
]
debug_data = ColumnDataSource(data = {'names':[], 'datas':[]})
debug_table = DataTable(source=debug_data, columns=columns, width=600, height=500)

collision_detection_py.Init()

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.obstacle_line_start_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='90%'), description= "x_start_ob_line",min=-15, max=15, value=-3.88, step=0.01)
    self.obstacle_line_start_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='90%'), description= "y_start_ob_line",min=-15, max=15, value=10.56, step=0.01)
    self.obstacle_line_end_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='90%'), description= "x_end_ob_line",min=-15, max=15, value=13.85, step=0.01)
    self.obstacle_line_end_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='90%'), description= "y_end_ob_line",min=-15, max=15, value=7.99, step=0.01)
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-10, max=10, value=0.0, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value=0.0, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=-180, max=180, value=90.0, step=1)
    self.s_init_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "s_init",min=-10.0, max=10.0, value=0.0, step=0.01)
    self.target_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_x",min=0.0, max=15.0, value=5.2, step=0.01)
    self.target_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_y",min=-15.0, max=15.0, value=12.2, step=0.01)
    self.target_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_heading",min=-90.0, max=0.0, value=0.0, step=1)
    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "ds",min=0.025, max=5.0, value=3.8, step=0.025)
    self.radius_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "radius",min=2.0, max=10.0, value=5.2, step=0.01)
    self.dubins_type_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='25%'), description= "dubins_type",min=0, max=3, value=0, step=1)
    self.case_type_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "case_type",min=0, max=1, value=0, step=1)
    self.set_start_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_start",min=0, max=1, value=0, step=1)
    self.reset_target_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "reset_target",min=0, max=1, value=0, step=1)
    self.fix_result_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "fix_result",min=0, max=1, value=0, step=1)
    self.set_pB_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_pB",min=0, max=1, value=0, step=1)
    self.set_pC_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_pC",min=0, max=1, value=0, step=1)
    self.set_pD_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_pD",min=0, max=1, value=0, step=1)
    self.line_arc_enable_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "line_arc_enable",min=0, max=1, value=0, step=1)
    self.line_arc_type_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "line_arc_type",min=0, max=3, value=0, step=1)
    self.is_complete_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_complete",min=0, max=1, value=0, step=1)

    ipywidgets.interact(slider_callback, obstacle_line_start_x = self.obstacle_line_start_x_slider,
                                         obstacle_line_start_y = self.obstacle_line_start_y_slider,
                                         obstacle_line_end_x = self.obstacle_line_end_x_slider,
                                         obstacle_line_end_y = self.obstacle_line_end_y_slider,
                                         ego_x = self.ego_x_slider,
                                         ego_y = self.ego_y_slider,
                                         ego_heading = self.ego_heading_slider,
                                         s_init = self.s_init_slider,
                                         target_x = self.target_x_slider,
                                         target_y = self.target_y_slider,
                                         target_heading = self.target_heading_slider,
                                         radius = self.radius_slider,
                                         dubins_type = self.dubins_type_slider,
                                         case_type = self.case_type_slider,
                                         set_start = self.set_start_slider,
                                         reset_target = self.reset_target_slider,
                                         fix_result = self.fix_result_slider,
                                         set_pB = self.set_pB_slider,
                                         set_pC = self.set_pC_slider,
                                         set_pD = self.set_pD_slider,
                                         line_arc_enable = self.line_arc_enable_slider,
                                         line_arc_type = self.line_arc_type_slider,
                                         ds = self.ds_slider,
                                         is_complete = self.is_complete_slider)


### sliders callback
def slider_callback(obstacle_line_start_x, obstacle_line_start_y, obstacle_line_end_x, obstacle_line_end_y, ego_x, ego_y, ego_heading, s_init,
                    target_x, target_y, target_heading, radius, dubins_type, case_type, set_start, reset_target, fix_result, set_pB, set_pC, set_pD,
                    line_arc_enable, line_arc_type, ds, is_complete):
  kwargs = locals()

  # if set_start == 1:
  #   slider_class.s_init_slider.value = 0.0
  #   slider_class.ego_x_slider.value = 6.0
  #   slider_class.ego_y_slider.value = 7.47
  #   slider_class.ego_heading_slider.value = -88

  # if reset_target == 1:
  #   slider_class.target_x_slider.value = 12.39
  #   slider_class.target_y_slider.value = 1.05
  #   slider_class.target_heading_slider.value = -4.0

  if set_start == 1:
    slider_class.s_init_slider.value = 0.0
    slider_class.ego_x_slider.value = target_x
    slider_class.ego_y_slider.value = target_y
    slider_class.ego_heading_slider.value = target_heading

  if reset_target == 1:
    slider_class.target_x_slider.value = 1.0
    slider_class.target_y_slider.value = 0.0
    slider_class.target_heading_slider.value = 0.0

  x_start = ego_x + s_init * math.cos(ego_heading / 57.2958)
  y_start = ego_y + s_init * math.sin(ego_heading / 57.2958)

  x_target = target_x
  y_target = target_y

  # ego car
  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], x_start, y_start, ego_heading / 57.2958)
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
  print("car_xn",  car_xn)
  data_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  data_start_pos.data.update({
    'x': [x_start],
    'y': [y_start],
  })

  data_target_pos.data.update({
    'x': [x_target],
    'y': [y_target],
  })

  if fix_result == 0:
    if line_arc_enable == 0:
      collision_detection_py.Update(x_start, y_start, ego_heading / 57.2958, x_target, y_target, target_heading / 57.2958, radius, dubins_type, case_type, ds, is_complete)
    else:
      collision_detection_py.UpdateLineArc(x_start, y_start, ego_heading / 57.2958, x_target, y_target, target_heading / 57.2958, radius, line_arc_type, ds, is_complete)

  path_x_vec = collision_detection_py.GetPathEle(0)
  path_y_vec = collision_detection_py.GetPathEle(1)
  path_theta_vec = collision_detection_py.GetPathEle(2)

  data_path.data.update({
    'x_vec': path_x_vec,
    'y_vec': path_y_vec,
    'theta_vec': path_theta_vec,
  })

  print("sample_point_size:", len(path_x_vec))

  collision_detection_py.GenCarCirclePb(path_x_vec, path_y_vec, path_theta_vec)
  circle_path_vec = collision_detection_py.GetCarCirclePb()
  circle_x_vec = []
  circle_y_vec = []
  circle_r_vec = []
  for i in range(len(circle_path_vec)):
    circle_vec = circle_path_vec[i]
    for j in range(len(circle_vec)):
      circle = circle_vec[j]
      circle_x_vec.append(circle[0])
      circle_y_vec.append(circle[1])
      circle_r_vec.append(circle[2])

  data_car_circle.data.update({
    'circle_xn': circle_x_vec,
    'circle_yn': circle_y_vec,
    'circle_rn': circle_r_vec,
  })


  obstacle_line_x = [obstacle_line_start_x, obstacle_line_end_x]
  obstacle_line_y = [obstacle_line_start_y, obstacle_line_end_y]

  data_obstacle_line.data.update({
    'x': obstacle_line_x,
    'y': obstacle_line_y,
  })

  obstacle_line_start = [[obstacle_line_start_x, obstacle_line_start_y]]
  obstacle_line_end = [[obstacle_line_end_x, obstacle_line_end_y]]
  collision_detection_py.GenObstacleLinePb(obstacle_line_start, obstacle_line_end)

  collision_detection_flag = collision_detection_py.CollisionDetectPb()

  print("collision_detection_flag:", collision_detection_flag)


  names = []
  datas = []
  names.append("collision_detection_flag")
  datas.append(collision_detection_flag)
  debug_data.data.update({
    'names': names,
    'datas': datas,
  })

  push_notebook()

bkp.show(row(fig1, debug_table), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
