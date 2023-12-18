import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, planning_plan_pb2
from jupyter_pybind import perpendicular_planning_py

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()


car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()

data_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
data_PA = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_AB = ColumnDataSource(data=dict(x=[0], y=[0], radius=[1]))
data_BT = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_start_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_target_pos = ColumnDataSource(data = {'x':[], 'y':[]})

data_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'theta_vec':[]})
data_car_box = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_tlane = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_min_safe_circle = ColumnDataSource(data = {'x':[], 'y':[],'r':[]})

data_tag_point = ColumnDataSource(data = {'x':[], 'y':[]})

data_channel = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_obstacles = ColumnDataSource(data = {'x':[], 'y':[]})


fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=640, match_aspect = True, aspect_scale=1)
fig1.x_range.flipped = True

fig1.line('y_vec','x_vec',source =data_path,  line_width = 3, line_color = 'green', line_dash = 'solid',legend_label = 'car_path')
fig1.patches('y_vec', 'x_vec', source = data_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sampled carbox')
## t-lane
fig1.multi_line('y_vec', 'x_vec',source = data_tlane, line_width = 1.5, line_color = 'black', line_dash = 'solid',legend_label = 'T-lane')
fig1.line('y_vec','x_vec',source =data_channel,  line_width = 1, line_color = 'black', line_dash = 'solid',legend_label = 'channel')
# safe circle
fig1.circle(x = 'y', y = 'x', radius = 'r', source = data_min_safe_circle, line_alpha = 1, line_width = 0.5, line_color = "red", fill_alpha=0,line_dash = 'dashed', legend_label = 'min safe circle')
fig1.patch('car_yn', 'car_xn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
fig1.circle('y','x', source = data_tag_point, size=8, color='grey', legend_label = 'tag point')


# key point of segment
fig1.line('y_vec', 'x_vec', source = data_PA, line_width = 2, line_color = 'black', line_dash = 'solid',legend_label = 'line PA',visible = False)
fig1.circle(x = 'y', y = 'x', radius = 'radius', source = data_AB, line_alpha = 1, line_width = 2, line_color = "red", fill_alpha=0, legend_label = 'circle AB')
fig1.line('y_vec', 'x_vec', source = data_BT, line_width = 2, line_color = 'black', line_dash = 'solid',legend_label = 'line BT',visible = False)

fig1.circle('y','x', source = data_start_pos, size=8, color='red', legend_label = 'start_pos')
fig1.circle('y','x', source = data_target_pos, size=8, color='blue', legend_label = 'target_pos')

fig1.circle('y','x', source = data_obstacles, size=8, color='blue', legend_label = 'obstacles')


fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

perpendicular_planning_py.Init()




### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=0, max=15, value=5.39277, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value=0.347403, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=-180, max=180, value=-45.9641, step=0.1)

    self.s_init_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "s_init",min=-10.0, max=10.0, value=0.0, step=0.01)

    self.target_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_x",min=0.0, max=8.0, value=6.7256, step=0.01)
    self.target_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_y",min=-8.0, max=8.0, value=0.664634, step=0.01)
    self.target_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_heading",min=-90.0, max=90.0, value=0.0, step=0.1)

    self.tlane_p0_deta_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p0_deta_x",min=-5, max=5, value=0.0, step=0.01)
    self.tlane_p0_deta_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p0_deta_y",min=-15, max=15, value=0.0, step=0.01)

    self.tlane_p1_deta_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p1_deta_x",min=-5, max=5, value=-1.5, step=0.01)
    self.tlane_p1_deta_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p1_deta_y",min=-15, max=15, value=-0.3, step=0.01)

    self.tlane_pt_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_pt_x",min=0, max=15, value=1.1, step=0.01)
    self.tlane_pt_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_pt_x",min=-0.5, max=0.5, value=0, step=0.01)
    self.channel_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel_x",min=5.0, max=20.0, value=12.0, step=0.1)

    self.set_start_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_start",min=0, max=1, value=0, step=1)
    self.reset_target_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "reset_target",min=0, max=1, value=0, step=1)
    self.fix_result_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "fix_result",min=0, max=1, value=0, step=1)

    self.set_pA_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_pA",min=0, max=1, value=0, step=1)
    self.set_pB_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_pB",min=0, max=1, value=0, step=1)

    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='30%'), description= "ds",min=0.025, max=1.0, value=0.5, step=0.025)
    self.is_complete_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_complete_path",min=0, max=1, value=1, step=1)

    ipywidgets.interact(slider_callback, ego_x = self.ego_x_slider,
                                         ego_y = self.ego_y_slider,
                                         ego_heading = self.ego_heading_slider,
                                         s_init = self.s_init_slider,
                                         tlane_p0_deta_x = self.tlane_p0_deta_x_slider,
                                         tlane_p0_deta_y = self.tlane_p0_deta_y_slider,
                                         tlane_p1_deta_x = self.tlane_p1_deta_x_slider,
                                         tlane_p1_deta_y = self.tlane_p1_deta_y_slider,
                                         tlane_pt_x = self.tlane_pt_x_slider,
                                         tlane_pt_y = self.tlane_pt_y_slider,
                                         channel_x = self.channel_x_slider,
                                         set_start = self.set_start_slider,
                                         reset_target = self.reset_target_slider,
                                         fix_result = self.fix_result_slider,
                                         set_pA = self.set_pA_slider,
                                         set_pB = self.set_pB_slider,
                                         ds = self.ds_slider,
                                         is_complete_path = self.is_complete_path_slider)


### sliders callback
def slider_callback(ego_x, ego_y, ego_heading, s_init, tlane_p0_deta_x, tlane_p0_deta_y,
                    tlane_p1_deta_x,tlane_p1_deta_y,tlane_pt_x,tlane_pt_y,channel_x,
                    set_start,reset_target,fix_result,set_pA,set_pB,ds,is_complete_path):
  kwargs = locals()

  kNormalSlotDepth = 4.8
  kNormalSlotWidth = 2.2

  tlane_p0_x = kNormalSlotDepth + tlane_p0_deta_x
  tlane_p1_x = kNormalSlotDepth + tlane_p1_deta_x

  tlane_p0_y = 0.5 * kNormalSlotWidth + tlane_p0_deta_y
  tlane_p1_y = -0.5 * kNormalSlotWidth + tlane_p1_deta_y

  if set_start == 1:
    slider_class.s_init_slider.value = 0.0
    slider_class.ego_x_slider.value = tlane_pt_x
    slider_class.ego_y_slider.value = tlane_pt_y

  if reset_target == 1:
    slider_class.tlane_pt_x_slider.value = 1.0
    slider_class.tlane_pt_y_slider.value = 0.0



  slot_side = 1.0
  if tlane_p0_y < tlane_p1_y:
    slot_side = -1.0
  tlane_bounds_x_vec = []
  tlane_bounds_y_vec = []

  ## first parallel line to p0
  tlane_bounds_x_vec.append([tlane_p0_x,tlane_p0_x])
  tlane_bounds_y_vec.append([tlane_p0_y + slot_side *3.0, tlane_p0_y])

  ## second parallel line to p1
  tlane_bounds_x_vec.append([tlane_p1_x,tlane_p1_x])
  tlane_bounds_y_vec.append([tlane_p1_y - slot_side *3.0, tlane_p1_y])

  ## vertical line to p0
  tlane_bottom_x = 0
  tlane_bounds_x_vec.append([tlane_p0_x,tlane_bottom_x])
  tlane_bounds_y_vec.append([tlane_p0_y,tlane_p0_y])

  ##vertical line to p1
  tlane_bounds_x_vec.append([tlane_p1_x,tlane_bottom_x])
  tlane_bounds_y_vec.append([tlane_p1_y,tlane_p1_y])

  ##bottom parallel line
  tlane_bounds_x_vec.append([tlane_bottom_x,tlane_bottom_x])
  tlane_bounds_y_vec.append([tlane_p0_y,tlane_p1_y])

  data_tlane.data.update({
     'x_vec': tlane_bounds_x_vec,
     'y_vec': tlane_bounds_y_vec,})

  # channel
  data_channel.data.update({'x_vec':[channel_x,channel_x],'y_vec':[-8,8]})

  # ego start position
  data_start_pos.data.update({'x':[ego_x],'y':[ego_y]})
  data_target_pos.data.update({'x':[tlane_pt_x],'y':[tlane_pt_y]})

  x_start = ego_x + s_init * math.cos(ego_heading / 57.2958)
  y_start = ego_y + s_init * math.sin(ego_heading / 57.2958)

  x_target = tlane_pt_x
  y_target = tlane_pt_y
  # ego car
  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], x_start, y_start,
                                  ego_heading / 57.2958)
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)

  data_car.data.update({
    'car_xn': car_xn,
    'car_yn': car_yn,
  })

  if fix_result == 0:
    perpendicular_planning_py.Update(ego_x, ego_y, ego_heading / 57.3, tlane_p0_x,
                                   tlane_p0_y,tlane_p1_x,tlane_p1_y, x_target,
                                   y_target,channel_x, ds,is_complete_path)

  AB_center = perpendicular_planning_py.GetABCenter()
  pA = perpendicular_planning_py.GetpA()
  pB = perpendicular_planning_py.GetpB()

  mono_safe_circle = perpendicular_planning_py.GetMinSafeCircle()
  data_min_safe_circle.data.update({
     'x':[mono_safe_circle[0]],
     'y':[mono_safe_circle[1]],
     'r':[mono_safe_circle[2]],
  })

  tag_point = perpendicular_planning_py.GetTagPoint()

  headingB = perpendicular_planning_py.GetHeadingB()

  print("headingB = ", headingB * 57.3)

  data_tag_point.data.update({
    'x':[tag_point[0]],
    'y':[tag_point[1]],
  })

  data_PA.data.update({
    'x_vec': [x_start, pA[0]],
    'y_vec': [y_start, pA[1]],
  })

  data_AB.data.update(dict(x=[AB_center[0]], y=[AB_center[1]], radius=[mono_safe_circle[2]]))

  data_BT.data.update({
    'x_vec': [pB[0], tlane_pt_x],
    'y_vec': [pB[1], tlane_pt_y],
  })

  # clear
  data_path.data.update({
    'x_vec': [],
    'y_vec': [],
    'theta_vec': [],
  })

  path_x_vec = perpendicular_planning_py.GetPathEle(0)
  path_y_vec = perpendicular_planning_py.GetPathEle(1)
  path_theta_vec = perpendicular_planning_py.GetPathEle(2)
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

  if set_pA:
    slider_class.ego_x_slider.value = pA[0]
    slider_class.ego_y_slider.value = pA[1]
    slider_class.ego_heading_slider.value = ego_heading
    slider_class.s_init_slider.value = 0.0

  if set_pB:
    slider_class.ego_x_slider.value = pB[0]
    slider_class.ego_y_slider.value = pB[1]
    slider_class.ego_heading_slider.value = 0.0
    slider_class.s_init_slider.value = 0.0

  obstacles = perpendicular_planning_py.GetObstacles()
  # print("obstacles = ", obstacles)
  x_obstacles = []
  y_obstacles = []
  for i in range(len(obstacles)):
    x_obstacles.append(obstacles[i][0])
    y_obstacles.append(obstacles[i][1])
  # print("x_set = ", x_obstacles)
  # print("y_set = ", y_obstacles)
  data_obstacles.data.update({
    'x':x_obstacles,
    'y':y_obstacles,
  })


  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


