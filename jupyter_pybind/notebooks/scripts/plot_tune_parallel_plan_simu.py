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

data_fix_car_pos = ColumnDataSource(data = {'x':[], 'y':[]})
data_fix_car = ColumnDataSource(data = {'car_yn':[], 'car_xn':[]})
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

data_obs_pt = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})

data_debug_arc = ColumnDataSource(data = {'cx_vec':[],
                                        'cy_vec':[],
                                        'radius_vec':[],
                                        'pBx_vec':[],
                                        'pBy_vec':[],
                                        'pCx_vec':[],
                                        'pCy_vec':[]})


fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=640, match_aspect = True, aspect_scale=1)
fig1.x_range.flipped = True

fig1.line('y_vec','x_vec',source =data_path,  line_width = 3, line_color = 'green', line_dash = 'solid',legend_label = 'car_path')
fig1.patches('y_vec', 'x_vec', source = data_car_box, fill_color = "#98FB98", fill_alpha = 0.0, line_color = "black", line_width = 1, legend_label = 'sampled carbox')
## t-lane
fig1.multi_line('y_vec', 'x_vec',source = data_tlane, line_width = 1.5, line_color = 'black', line_dash = 'solid',legend_label = 'T-lane')
fig1.line('y_vec','x_vec',source =data_channel,  line_width = 1, line_color = 'black', line_dash = 'solid',legend_label = 'channel')

# car box at start pose
fig1.patch('car_yn', 'car_xn', source = data_car, fill_color = "palegreen", fill_alpha = 0.2, line_color = "black", line_width = 1, legend_label = 'car')
# fix car
fig1.patch('car_yn', 'car_xn', source = data_fix_car, fill_color = "yellow", fill_alpha = 0.2, line_color = "black", line_width = 1, legend_label = 'fix car', visible =False)
fig1.circle('y','x', source = data_fix_car_pos, size= 8, color='green', legend_label = 'fix car', visible = False)

fig1.circle('y','x', source = data_tag_point, size=8, color='grey', legend_label = 'tag point')

fig1.circle('y','x', source = data_start_pos, size=8, color='red', legend_label = 'start_pos')
# target pose
fig1.circle('y','x', source = data_target_pos, size=8, color='green', legend_label = 'target_pos')
# obstacles
fig1.scatter("y_vec", "x_vec", source=data_obs_pt, size=8, color='blue',legend_label = 'obstacle pts')

fig1.circle(x = 'cy_vec', y = 'cx_vec', radius = 'radius_vec', source = data_debug_arc, line_alpha = 1, line_width = 2, line_color = "green",
            fill_alpha=0, legend_label = 'data_debug_arc')


fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

parallel_planning_py.Init()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_x",min=-15, max=15, value= 3.00268, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_y",min=-10, max=10, value= 3.05401, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ego_heading",min=-180, max=180, value= 2.6507, step=0.1)
    self.s_init_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "s_init",min=-10.0, max=10.0, value=0.0, step=0.01)

    self.tlane_p_outside_deta_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p_outside_deta_x",min=-15, max=15, value=-1.5, step=0.01)
    self.tlane_p_outside_deta_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p_outside_deta_y",min=-15, max=15, value=1.1748, step=0.01)
    self.tlane_p_inside_deta_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p_inside_deta_x",min=-15, max=15, value= 7.412, step=0.01)# 6.74947
    self.tlane_p_inside_deta_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_p_inside_deta_y",min=-15, max=15, value= 1.17481, step=0.01)

    self.tlane_pt_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_pt_x",min=-2.0, max=2.0, value=1.497, step=0.01)
    self.tlane_pt_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "tlane_pt_y",min=-2.0, max=2.0, value=0.0, step=0.01)
    self.channel_width_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel_width",min=3.0, max=20.0, value=6.5, step=0.1)
    self.channel_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "channel_length",min=8.0, max=30.0, value=18.0, step=0.1)
    self.curb_offset_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "curb_offset",min=1.2, max=5.0, value=2.2, step=0.1)
    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "ds",min=0.025, max=1.0, value=0.2, step=0.025)


    self.fix_car_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "fix_car",min=-20.0, max=20.0, value=1.30479, step=0.01)
    self.fix_car_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "fix_car",min=-20.0, max=20.0, value=-0.262434, step=0.01)
    self.fix_car_heading_deg_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "fix_car",min=-180.0, max=180.0, value=25.8808, step=0.01)

    self.set_left_slot_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set left slot",min=0, max=1, value=0, step=1)
    self.set_start_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_start",min=0, max=1, value=0, step=1)
    self.reset_target_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "reset_target",min=0, max=1, value=0, step=1)
    self.fix_result_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "fix_result",min=0, max=1, value=0, step=1)

    self.set_prepare_tangent_pt_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_prepare_tangent_pt",min=0, max=1, value=0, step=1)
    self.is_complete_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_complete_path",min=0, max=1, value=1, step=1)
    self.is_plan_first_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "is_plan_first",min=0, max=1, value=1, step=1)
    self.set_pin_obs_slider =  ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_pin_obs",min=0, max=1, value=1, step=1)
    self.set_pout_obs_slider =  ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_pout_obs",min=0, max=1, value=0, step=1)
    self.set_pin_line_obs_slider =  ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_pin_line_obs",min=0, max=1, value=1, step=1)
    self.set_pout_line_obs_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "set_pout_line_obs",min=0, max=1, value=1, step=1)
    self.seg_pin_parallel_line_obs_slider =ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "seg_pin_parallel_line_obs",min=0, max=1, value=1, step=1)
    self.ref_gear_drive_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "dirve",min=0, max=1, value=0, step=1)
    self.ref_steer_left_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='15%'), description= "left",min=0, max=1, value=0, step=1)


    ipywidgets.interact(slider_callback, ego_x = self.ego_x_slider,
                                         ego_y = self.ego_y_slider,
                                         ego_heading = self.ego_heading_slider,
                                         s_init = self.s_init_slider,
                                         tlane_p_outside_deta_x = self.tlane_p_outside_deta_x_slider,
                                         tlane_p_outside_deta_y = self.tlane_p_outside_deta_y_slider,
                                         tlane_p_inside_deta_x = self.tlane_p_inside_deta_x_slider,
                                         tlane_p_inside_deta_y = self.tlane_p_inside_deta_y_slider,
                                         tlane_pt_x = self.tlane_pt_x_slider,
                                         tlane_pt_y = self.tlane_pt_y_slider,
                                         channel_width = self.channel_width_slider,
                                         channel_length = self.channel_length_slider,
                                         curb_offset = self.curb_offset_slider,
                                         ds = self.ds_slider,
                                         fix_car_x = self.fix_car_x_slider,
                                         fix_car_y = self.fix_car_y_slider,
                                         fix_car_heading_deg = self.fix_car_heading_deg_slider,
                                         set_left_slot = self.set_left_slot_slider,
                                         set_start = self.set_start_slider,
                                         reset_target = self.reset_target_slider,
                                         fix_result = self.fix_result_slider,
                                         set_prepare_tangent_pt = self.set_prepare_tangent_pt_slider,
                                         is_complete_path = self.is_complete_path_slider,
                                         is_plan_first = self.is_plan_first_slider,
                                         set_pin_obs = self.set_pin_obs_slider,
                                         set_pout_obs = self.set_pout_obs_slider,
                                         set_pin_line_obs = self.set_pin_line_obs_slider,
                                         set_pout_line_obs = self.set_pout_line_obs_slider,
                                         seg_pin_parallel_line_obs = self.seg_pin_parallel_line_obs_slider,
                                         ref_gear_drive = self.ref_gear_drive_slider,
                                         ref_steer_left = self.ref_steer_left_slider)

### sliders callback
def slider_callback(ego_x, ego_y, ego_heading, s_init,
                    tlane_p_outside_deta_x, tlane_p_outside_deta_y,
                    tlane_p_inside_deta_x,tlane_p_inside_deta_y,
                    tlane_pt_x,tlane_pt_y,
                    channel_width,channel_length,
                    curb_offset,ds,
                    fix_car_x, fix_car_y, fix_car_heading_deg,
                    set_left_slot, set_start,reset_target,
                    fix_result,set_prepare_tangent_pt,is_complete_path,
                    is_plan_first,
                    set_pin_obs,set_pout_obs, set_pin_line_obs, set_pout_line_obs, seg_pin_parallel_line_obs,
                    ref_gear_drive, ref_steer_left):
  kwargs = locals()


  slot_side_sgn = 1.0
  if set_left_slot:
    slot_side_sgn = -1.0

  kNormalSlotDepth = 2.0
  kNormalSlotLength = 6.0
  # tlane_p0_y = 0.5 * slot_side_sgn * kNormalSlotDepth + tlane_p_outside_deta_y
  # tlane_p1_y = 0.5 * slot_side_sgn * kNormalSlotDepth + tlane_p_inside_deta_y
  # tlane_p0_x = tlane_p_outside_deta_x
  # tlane_p1_x = kNormalSlotLength + tlane_p_inside_deta_x

  tlane_p0_x = tlane_p_outside_deta_x
  tlane_p0_y = tlane_p_outside_deta_y

  tlane_p1_x = tlane_p_inside_deta_x
  tlane_p1_y = tlane_p_inside_deta_y

  if set_start == 1:
    slider_class.s_init_slider.value = 0.0
    slider_class.ego_x_slider.value = ego_x
    slider_class.ego_y_slider.value = ego_y

  if reset_target == 1:
    slider_class.tlane_pt_x_slider.value = 1.2
    slider_class.tlane_pt_y_slider.value = 0.0

  tlane_bounds_x_vec = []
  tlane_bounds_y_vec = []

  ## first parallel line to p0
  tlane_bounds_x_vec.append([tlane_p0_x -slot_side_sgn * 3.0, tlane_p0_x])
  tlane_bounds_y_vec.append([tlane_p0_y, tlane_p0_y])

  ## second parallel line to p1
  tlane_bounds_x_vec.append([tlane_p1_x + 3.0, tlane_p1_x])
  tlane_bounds_y_vec.append([tlane_p1_y, tlane_p1_y])

  ## vertical line to p0
  tlane_bottom_y = -curb_offset
  tlane_bounds_x_vec.append([tlane_p0_x, tlane_p0_x])
  tlane_bounds_y_vec.append([tlane_p0_y, tlane_bottom_y * slot_side_sgn])

  ##vertical line to p1
  tlane_bounds_x_vec.append([tlane_p1_x,tlane_p1_x])
  tlane_bounds_y_vec.append([tlane_p1_y,tlane_bottom_y * slot_side_sgn])

  ##bottom parallel line
  tlane_bounds_x_vec.append([tlane_p0_x,tlane_p1_x])
  tlane_bounds_y_vec.append([tlane_bottom_y * slot_side_sgn, tlane_bottom_y * slot_side_sgn])

  data_tlane.data.update({
     'x_vec': tlane_bounds_x_vec,
     'y_vec': tlane_bounds_y_vec,})

  # channel
  channel_y = slot_side_sgn * (channel_width + 0.5 * kNormalSlotDepth)
  data_channel.data.update({'y_vec':[channel_y,channel_y],'x_vec':[0.0, channel_length]})

  # ego start position
  data_start_pos.data.update({'x':[ego_x],'y':[ego_y]})
  data_target_pos.data.update({'x':[tlane_pt_x],'y':[tlane_pt_y]})

  x_start = ego_x + s_init * math.cos(ego_heading / 57.2958)
  y_start = ego_y + s_init * math.sin(ego_heading / 57.2958)

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

  # fix car
  fix_car_xn = []
  fix_car_yn = []
  for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], fix_car_x, fix_car_y,
                                  fix_car_heading_deg /  57.2958)
      fix_car_xn.append(tmp_x)
      fix_car_yn.append(tmp_y)

  data_fix_car.data.update({
    'car_xn': fix_car_xn,
    'car_yn': fix_car_yn,
  })

  data_fix_car_pos.data.update({
    'x': [fix_car_x],
    'y': [fix_car_y],
  })

  if fix_result == 0:
    parallel_planning_py.Update(ego_x, ego_y, ego_heading / 57.2958, \
                                tlane_p0_x, tlane_p0_y, \
                                tlane_p1_x, tlane_p1_y, \
                                tlane_pt_x, tlane_pt_y, \
                                channel_width, channel_length, curb_offset, \
                                ds, is_complete_path, \
                                is_plan_first, set_left_slot, \
                                set_pin_obs,set_pout_obs, \
                                set_pin_line_obs, set_pout_line_obs, \
                                seg_pin_parallel_line_obs,\
                                ref_gear_drive, ref_steer_left)

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

  # obstacles
  data_obs_pt.data.update({
    'x_vec': parallel_planning_py.GetObsX(),
    'y_vec': parallel_planning_py.GetObsY(),
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

  if set_prepare_tangent_pt:
    slider_class.ego_x_slider.value = path_x_vec[-1]
    slider_class.ego_y_slider.value = path_y_vec[-1]
    slider_class.ego_heading_slider.value = path_theta_vec[-1] * 57.3
    slider_class.s_init_slider.value = 0.0


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

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


