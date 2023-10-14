import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, planning_plan_pb2
from jupyter_pybind import dubins_lib_py

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()


car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()
slot_x = [4.8, 0.0, 0.0, 4.8]
slot_y = [1.2, 1.2, -1.2, -1.2]

data_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_AB = ColumnDataSource(data=dict(x=[0], y=[0], radius=[1]))
data_CD = ColumnDataSource(data=dict(x=[0], y=[0], radius=[1]))
data_BC = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_target_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_O1A = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_O2D = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'theta_vec':[]})
data_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=640, match_aspect = True, aspect_scale=1)
fig1.x_range.flipped = True
fig1.line(slot_y, slot_x, line_width = 3, line_color = 'green', line_dash = 'solid',legend_label = 'slot')
fig1.patch('car_yn', 'car_xn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
fig1.circle(x = 'y', y = 'x', radius = 'radius', source = data_AB, line_alpha = 1, line_width = 2, line_color = "red", fill_alpha=0, legend_label = 'circle AB')
fig1.line('y_vec', 'x_vec', source = data_O1A, line_width = 1.5, line_color = 'red', line_dash = 'dashed',legend_label = 'circle AB')
fig1.circle(x = 'y', y = 'x', radius = 'radius', source = data_CD, line_alpha = 1, line_width = 2, line_color = "blue", fill_alpha=0, legend_label = 'circle CD')
fig1.line('y_vec', 'x_vec', source = data_O2D, line_width = 1.5, line_color = 'blue', line_dash = 'dashed',legend_label = 'circle CD')
fig1.line('y_vec', 'x_vec', source = data_BC, line_width = 2, line_color = 'black', line_dash = 'solid',legend_label = 'line BC')
fig1.circle('y','x', source = data_start_pos, size=8, color='red', legend_label = 'circle AB')
fig1.circle('y','x', source = data_target_pos, size=8, color='blue', legend_label = 'circle CD')
fig1.circle('y_vec', 'x_vec', source = data_BC, size=8, color='black', legend_label = 'line BC')
fig1.line('y_vec', 'x_vec', source = data_path, line_width = 8, line_color = 'green', line_dash = 'solid', line_alpha = 0.4,legend_label = 'sampled path')
fig1.circle('y_vec','x_vec', source = data_path, size=4, color='green', legend_label = 'sampled path')
fig1.patches('y_vec', 'x_vec', source = data_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sampled carbox')

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

dubins_lib_py.Init()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-10, max=10, value=5.5, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value=0, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=-180, max=180, value=-90, step=0.1)
    self.s_init_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "s_init",min=-10.0, max=10.0, value=0.0, step=0.01)
    self.target_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_x",min=0.0, max=8.0, value=1.5, step=0.01)
    self.target_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_y",min=-3.0, max=3.0, value=0.0, step=0.01)
    self.target_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_heading",min=-90.0, max=90.0, value=0.0, step=0.1)
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
    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "ds",min=0.025, max=1.0, value=0.5, step=0.025)
    self.is_complete_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_complete",min=0, max=1, value=0, step=1)

    ipywidgets.interact(slider_callback, ego_x = self.ego_x_slider,
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
def slider_callback(ego_x, ego_y, ego_heading, s_init, target_x, target_y, target_heading, radius,
                    dubins_type, case_type, set_start, reset_target, fix_result, set_pB, set_pC, set_pD,
                    line_arc_enable, line_arc_type, ds, is_complete):
  kwargs = locals()

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

  data_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  if fix_result == 0:
    if line_arc_enable == 0:
      dubins_lib_py.Update(x_start, y_start, ego_heading / 57.2958, x_target, y_target, target_heading / 57.2958, radius, dubins_type, case_type, ds, is_complete)
    else:
      dubins_lib_py.UpdateLineArc(x_start, y_start, ego_heading / 57.2958, x_target, y_target, target_heading / 57.2958, radius, line_arc_type, ds, is_complete)


  AB_center = dubins_lib_py.GetABCenter()
  CD_center = dubins_lib_py.GetCDCenter()
  pB = dubins_lib_py.GetpB()
  pC = dubins_lib_py.GetpC()
  pD = dubins_lib_py.GetpD()
  # path_length = dubins_lib_py.GetLength()

  theta_BC = dubins_lib_py.GetThetaBC() * 57.2958
  theta_D = dubins_lib_py.GetThetaD() * 57.2958
  # path_available= dubins_lib_py.GetPathAvailiable()
  # gear_cmd_vec = dubins_lib_py.GetGearCmdVec()
  # gear_change_count = dubins_lib_py.GetGearChangeCount()
  path_radius = dubins_lib_py.GetRadius()

  path_x_vec = dubins_lib_py.GetPathEle(0)
  path_y_vec = dubins_lib_py.GetPathEle(1)
  path_theta_vec = dubins_lib_py.GetPathEle(2)

  # print("path_available= ", path_available)
  # print("AB_center = ", AB_center)
  # print("CD_center = ", CD_center)
  # print("pA = ", [x_start, y_start])
  # print("pB = ", pB)
  # print("pC = ", pC)
  # print("pD = ", pD)
  # print("theta_BC = ", theta_BC)
  # print("theta_D = ", theta_D)
  # print("path length = ", path_length)
  # print("gear_cmd_vec = ", gear_cmd_vec)
  # print("gear_change_count = ", gear_change_count)
  # print("path_radius = ", path_radius)

  data_start_pos.data.update({
    'x': [x_start, AB_center[0]],
    'y': [y_start, AB_center[1]],
  })

  data_target_pos.data.update({
    'x': [x_target, CD_center[0]],
    'y': [y_target, CD_center[1]],
  })

  data_AB.data.update(dict(x=[AB_center[0]], y=[AB_center[1]], radius=[path_radius]))
  data_CD.data.update(dict(x=[CD_center[0]], y=[CD_center[1]], radius=[path_radius]))

  data_BC.data.update({
    'x_vec': [pB[0], pC[0]],
    'y_vec': [pB[1], pC[1]],
  })

  data_O1A.data.update({
    'x_vec': [AB_center[0], x_start],
    'y_vec': [AB_center[1], y_start],
  })

  data_O2D.data.update({
    'x_vec': [CD_center[0], x_target],
    'y_vec': [CD_center[1], y_target],
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

  if set_pB:
    slider_class.ego_x_slider.value = pB[0]
    slider_class.ego_y_slider.value = pB[1]
    slider_class.ego_heading_slider.value = theta_BC
    slider_class.s_init_slider.value = 0.0

  if set_pC:
    slider_class.ego_x_slider.value = pC[0]
    slider_class.ego_y_slider.value = pC[1]
    slider_class.ego_heading_slider.value = theta_BC
    slider_class.s_init_slider.value = 0.0

  if set_pD:
    slider_class.ego_x_slider.value = pD[0]
    slider_class.ego_y_slider.value = pD[1]
    slider_class.ego_heading_slider.value = theta_D
    slider_class.s_init_slider.value = 0.0

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


