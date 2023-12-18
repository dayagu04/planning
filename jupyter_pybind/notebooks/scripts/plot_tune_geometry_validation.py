import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, planning_plan_pb2
from jupyter_pybind import geometry_math_validation_py

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

check_line_arc_line = True

data_start_pose_line = ColumnDataSource(data = {'x_vec':[],
                                                'y_vec':[]})
data_start_pos = ColumnDataSource(data = {'x':[],
                                          'y':[]})
data_tag_pos = ColumnDataSource(data = {'x':[],
                                        'y':[]})

data_tag_pos2 = ColumnDataSource(data = {'x':[],
                                        'y':[]})

data_target_pose_line = ColumnDataSource(data = {'x_vec':[],
                                                 'y_vec':[]})

data_res_arc = ColumnDataSource(data = {'cx_vec':[],
                                        'cy_vec':[],
                                        'radius_vec':[],
                                        'pBx_vec':[],
                                        'pBy_vec':[],
                                        'pCx_vec':[],
                                        'pCy_vec':[]})

fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=960, height=640, match_aspect = True, aspect_scale=1)
fig1.x_range.flipped = True

# key point of segment
fig1.line('y_vec', 'x_vec', source = data_start_pose_line, line_width = 2, line_color = 'blue', line_dash = 'solid',legend_label = 'start_pose')
fig1.line('y_vec', 'x_vec', source = data_target_pose_line, line_width = 2, line_color = 'red', line_dash = 'dashed',legend_label = 'target_pose line')
fig1.circle('y','x', source = data_start_pos, size=8, color='red', legend_label = 'start point')
fig1.circle('y','x', source = data_tag_pos, size=8, color='green', legend_label = 'tag point')
fig1.circle(x = 'cy_vec', y = 'cx_vec', radius = 'radius_vec', source = data_res_arc, line_alpha = 1, line_width = 2, line_color = "green",
            fill_alpha=0, legend_label = 'res arc')
if check_line_arc_line:
  fig1.circle('y','x', source = data_tag_pos2, size=8, color='blue', legend_label = 'tag point2')

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "ego_x",min=-15, max=15, value=0.0, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "ego_y",min=-15, max=15, value=0.0, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "ego_heading",min=-180, max=180, value=0, step=0.1)

    self.target_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_x",min=-15, max=15.0, value=0, step=0.01)
    self.target_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_y",min=-15, max=15.0, value=0.3, step=0.01)
    self.target_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_heading",min=-180, max=180, value=0, step=0.1)
    self.radius_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "radius",min=0.0, max=20.0, value=5.5, step=0.1)
    self.is_left_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='20%'), description= "is_left",min=0, max=1, value=1, step=1)
    self.is_advance_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='20%'), description= "is_advance",min=0, max=1, value=1, step=1)
    self.ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "ds",min=0.0, max=1.0, value=0.5, step=0.01)
    self.traj_length_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "traj_length",min=0.3, max=8.5, value=3.8, step=0.1)
    self.is_anticlockwise_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='20%'), description= "is_anticlockwise",min=0, max=1, value=1, step=1)

    ipywidgets.interact(slider_callback, ego_x = self.ego_x_slider,
                                         ego_y = self.ego_y_slider,
                                         ego_heading = self.ego_heading_slider,
                                         target_x =  self.target_x_slider,
                                         target_y = self.target_y_slider,
                                         target_heading = self.target_heading_slider,
                                         radius = self.radius_slider,
                                         is_left = self.is_left_slider,
                                         is_advance = self.is_advance_slider,
                                         ds = self.ds_slider,
                                         traj_length = self.traj_length_slider,
                                         is_anticlockwise= self.is_anticlockwise_slider)
### sliders callback
def slider_callback(ego_x, ego_y, ego_heading, target_x, target_y, target_heading, radius, is_left, is_advance, ds, traj_length, is_anticlockwise):
  kwargs = locals()

  # geometry_math_validation_py.UpdateOneStepArcTargetLine(ego_x, ego_y, ego_heading / 57.3, target_x, target_y, target_heading / 57.3)
  # geometry_math_validation_py.UpdateOneStepArcTargetLineGivenTurn(ego_x, ego_y, ego_heading / 57.3, target_x, target_y, target_heading / 57.3, is_left)
  # geometry_math_validation_py.UpdateOneStepArcTargetLineByGear(ego_x, ego_y, ego_heading / 57.3, target_x, target_y, target_heading / 57.3, is_advance)
  # geometry_math_validation_py.UpdateOneStepArcTargetHeading(ego_x, ego_y, ego_heading / 57.3, target_x, target_y, target_heading / 57.3, is_left)
  # geometry_math_validation_py.UpdateOneStepParallelShift(ego_x, ego_y, ego_heading / 57.3, target_x, target_y, is_advance, radius)
  geometry_math_validation_py.UpdateLineArcLine(ego_x, ego_y,ego_heading / 57.3, target_x, target_y, target_heading / 57.3, is_advance, is_left, radius)
  res = geometry_math_validation_py.GetOutput()

  info_len = 5
  if check_line_arc_line:
    info_len = 7
  arc_size = int(len(res) / info_len)

  print("arc info: cx, cy, radius, pBx, pBy")
  cx_vec = []
  cy_vec = []
  radius_vec = []
  pBx_vec = []
  pBy_vec = []

  pCx_vec = []
  pCy_vec = []

  for i in range(arc_size):
    print(i, ": [", res[info_len * i], res[info_len * i + 1], res[info_len * i + 2], res[info_len * i + 3], res[info_len * i + 4], "]")
    cx_vec.append(res[info_len * i])
    cy_vec.append(res[info_len * i + 1])
    radius_vec.append(res[info_len * i + 2])

    pBx_vec.append(res[info_len * i + 3])
    pBy_vec.append(res[info_len * i + 4])

    if check_line_arc_line:
      pCx_vec.append(res[info_len * i + 5])
      pCy_vec.append(res[info_len * i + 6])


  data_res_arc.data.update({
    'cx_vec':cx_vec,
    'cy_vec':cy_vec,
    'radius_vec':radius_vec,
    'pBx_vec':pBx_vec,
    'pBy_vec':pBy_vec,
    'pCx_vec':pCx_vec,
    'pCy_vec':pCy_vec
  })

  data_start_pos.data.update({
    'x':[ego_x],
    'y':[ego_y],
  })

  data_tag_pos.data.update({
    'x':pBx_vec,
    'y':pBy_vec,
  })

  if check_line_arc_line:
      data_tag_pos2.data.update({
    'x':pCx_vec,
    'y':pCy_vec,
  })

  data_start_pose_line.data.update({
    'x_vec':[ego_x, ego_x + math.cos(ego_heading / 57.3) * 1.0],
    'y_vec':[ego_y, ego_y + math.sin(ego_heading / 57.3) * 1.0],
  })

  data_target_pose_line.data.update({
    'x_vec':[target_x - math.cos(target_heading / 57.3) * 8.0, target_x + math.cos(target_heading / 57.3) * 8.0],
    'y_vec':[target_y - math.sin(target_heading / 57.3) * 8.0, target_y + math.sin(target_heading / 57.3) * 8.0],
  })

  # point_start = [ego_x, ego_y]
  # point_end = [target_x, target_y]
  # point_set = geometry_math_validation_py.UpdateSamplePointSet(point_start, ego_heading / 57.3, point_end, target_heading / 57.3, radius, ds, traj_length, is_anticlockwise)
  # # print("point_set = ", point_set)
  # x_set = []
  # y_set = []
  # for i in range(len(point_set)):
  #   x_set.append(point_set[i][0])
  #   y_set.append(point_set[i][1])
  # print("x_set = ", x_set)
  # print("y_set = ", y_set)
  # data_start_pos.data.update({
  #   'x':x_set,
  #   'y':y_set,
  # })

  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


