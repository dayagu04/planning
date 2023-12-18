import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, planning_plan_pb2
from jupyter_pybind import parallel_planning_py

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

data_tag_point = ColumnDataSource(data = {'x':[], 'y':[]})

data_channel = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})


fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=640, match_aspect = True, aspect_scale=1)
fig1.x_range.flipped = True

fig1.line('y_vec','x_vec',source =data_path,  line_width = 3, line_color = 'green', line_dash = 'solid',legend_label = 'car_path')
fig1.patches('y_vec', 'x_vec', source = data_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sampled carbox')
## t-lane
fig1.multi_line('y_vec', 'x_vec',source = data_tlane, line_width = 1.5, line_color = 'black', line_dash = 'solid',legend_label = 'T-lane')
fig1.line('y_vec','x_vec',source =data_channel,  line_width = 1, line_color = 'black', line_dash = 'solid',legend_label = 'channel')

fig1.patch('car_yn', 'car_xn', source = data_car, fill_color = "palegreen", line_color = "black", line_width = 1, legend_label = 'car')
fig1.circle('y','x', source = data_tag_point, size=8, color='grey', legend_label = 'tag point')

fig1.circle('y','x', source = data_start_pos, size=8, color='red', legend_label = 'start_pos')
fig1.circle('y','x', source = data_target_pos, size=8, color='blue', legend_label = 'target_pos')

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

parallel_planning_py.Init()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-15, max=15, value=-2.0, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value=3.0, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=-180, max=180, value=0.0, step=0.1)

    self.s_init_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "s_init",min=-10.0, max=10.0, value=0.0, step=0.01)

    self.target_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_x",min=-15, max=15.0, value=-2.0, step=0.01)
    self.target_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_y",min=-8.0, max=15.0, value=0.0, step=0.01)
    self.target_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_heading",min=-90.0, max=90.0, value=0.0, step=0.1)

    self.tlane_p0_deta_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p0_deta_x",min=-15, max=15, value=0.0, step=0.01)
    self.tlane_p0_deta_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p0_deta_y",min=-15, max=15, value=0.0, step=0.01)

    self.tlane_p1_deta_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p1_deta_x",min=-15, max=15, value=0.0, step=0.01)
    self.tlane_p1_deta_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p1_deta_y",min=-15, max=15, value=0.0, step=0.01)

    self.tlane_pt_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_pt_x",min=-5.0, max=5.0, value=-2.0, step=0.01)
    self.tlane_pt_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_pt_x",min=-0.5, max=0.5, value=0, step=0.01)
    self.channel_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel_y",min=5.0, max=20.0, value=8.0, step=0.1)

    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ds",min=0.025, max=1.0, value=0.3, step=0.025)

    self.set_start_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_start",min=0, max=1, value=0, step=1)
    self.reset_target_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "reset_target",min=0, max=1, value=0, step=1)
    self.fix_result_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "fix_result",min=0, max=1, value=0, step=1)

    self.set_prepare_tangent_pt_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_prepare_tangent_pt",min=0, max=1, value=0, step=1)
    self.is_complete_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_complete_path",min=0, max=1, value=1, step=1)
    self.is_plan_first_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_plan_first",min=0, max=1, value=1, step=1)

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
                                         channel_y = self.channel_y_slider,
                                         ds = self.ds_slider,
                                         set_start = self.set_start_slider,
                                         reset_target = self.reset_target_slider,
                                         fix_result = self.fix_result_slider,
                                         set_prepare_tangent_pt = self.set_prepare_tangent_pt_slider,
                                         is_complete_path = self.is_complete_path_slider,
                                         is_plan_first = self.is_plan_first_slider)

### sliders callback
def slider_callback(ego_x, ego_y, ego_heading, s_init, tlane_p0_deta_x, tlane_p0_deta_y,
                    tlane_p1_deta_x,tlane_p1_deta_y,tlane_pt_x,tlane_pt_y,channel_y,
                    set_start,reset_target,fix_result,set_prepare_tangent_pt,ds,is_complete_path,is_plan_first):
  kwargs = locals()

  kNormalSlotDepth = 2.4
  kNormalSlotWidth = 6.2

  tlane_p0_y = 0.5 * kNormalSlotDepth + tlane_p0_deta_y
  tlane_p1_y = 0.5 * kNormalSlotDepth + tlane_p1_deta_y

  tlane_p0_x = -0.5 * kNormalSlotWidth + tlane_p0_deta_x
  tlane_p1_x = 0.5 * kNormalSlotWidth + tlane_p1_deta_x

  if set_start == 1:
    slider_class.s_init_slider.value = 0.0
    slider_class.ego_x_slider.value = ego_x
    slider_class.ego_y_slider.value = ego_y

  if reset_target == 1:
    slider_class.tlane_pt_x_slider.value = -2.0
    slider_class.tlane_pt_y_slider.value = 0.0



  slot_side = 1.0
  if tlane_p0_x > tlane_p1_x:
    slot_side = -1.0
  tlane_bounds_x_vec = []
  tlane_bounds_y_vec = []

  ## first parallel line to p0
  tlane_bounds_x_vec.append([tlane_p0_x - slot_side*3.0, tlane_p0_x])
  tlane_bounds_y_vec.append([tlane_p0_y, tlane_p0_y])

  ## second parallel line to p1
  tlane_bounds_x_vec.append([tlane_p1_x + slot_side*3.0, tlane_p1_x])
  tlane_bounds_y_vec.append([tlane_p1_y, tlane_p1_y])

  ## vertical line to p0
  tlane_bottom_y = -1.2
  tlane_bounds_x_vec.append([tlane_p0_x,tlane_p0_x])
  tlane_bounds_y_vec.append([tlane_p0_y,tlane_bottom_y])

  ##vertical line to p1
  tlane_bounds_x_vec.append([tlane_p1_x,tlane_p1_x])
  tlane_bounds_y_vec.append([tlane_p1_y,tlane_bottom_y])

  ##bottom parallel line
  tlane_bounds_x_vec.append([tlane_p0_x,tlane_p1_x])
  tlane_bounds_y_vec.append([tlane_bottom_y,tlane_bottom_y])

  data_tlane.data.update({
     'x_vec': tlane_bounds_x_vec,
     'y_vec': tlane_bounds_y_vec,})

  # channel
  data_channel.data.update({'y_vec':[channel_y,channel_y],'x_vec':[-10,10]})

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
    parallel_planning_py.Update(ego_x, ego_y, ego_heading / 57.3, tlane_p0_x,
                                   tlane_p0_y,tlane_p1_x,tlane_p1_y, x_target,
                                   y_target,channel_y, ds,is_complete_path,is_plan_first)



  # tag_point = parallel_planning_py.GetTagPoint()


  # clear
  data_path.data.update({
    'x_vec': [],
    'y_vec': [],
    'theta_vec': [],
  })

  path_x_vec = parallel_planning_py.GetPathEle(0)
  path_y_vec = parallel_planning_py.GetPathEle(1)
  path_theta_vec = parallel_planning_py.GetPathEle(2)
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

  # if set_pA:
  #   slider_class.ego_x_slider.value = pA[0]
  #   slider_class.ego_y_slider.value = pA[1]
  #   slider_class.ego_heading_slider.value = ego_heading
  #   slider_class.s_init_slider.value = 0.0

  # if set_pB:
  #   slider_class.ego_x_slider.value = pB[0]
  #   slider_class.ego_y_slider.value = pB[1]
  #   slider_class.ego_heading_slider.value = 0.0
  #   slider_class.s_init_slider.value = 0.0
  if set_prepare_tangent_pt:
    slider_class.ego_x_slider.value = path_x_vec[-1]
    slider_class.ego_y_slider.value = path_y_vec[-1]
    slider_class.ego_heading_slider.value = path_theta_vec[-1] * 57.3
    slider_class.s_init_slider.value = 0.0

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


