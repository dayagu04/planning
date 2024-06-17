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
import numpy as np

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

data_arc = ColumnDataSource(data = {'cx_vec':[],
                                    'cy_vec':[],
                                    'radius_vec':[]})

data_text = ColumnDataSource(data = {'function':[]})

fig1 = bkp.figure(x_axis_label='x', y_axis_label='y', width=960, height=640, match_aspect = True, aspect_scale=1)
fig1.x_range.flipped = True

# key point of segment
fig1.line('x_vec', 'y_vec', source = data_start_pose_line, line_width = 2, line_color = 'blue', line_dash = 'solid',legend_label = 'start_pose')
fig1.line('x_vec', 'y_vec', source = data_target_pose_line, line_width = 2, line_color = 'red', line_dash = 'dashed',legend_label = 'target_pose line')
fig1.circle('x','y', source = data_start_pos, size=8, color='red', legend_label = 'start point')
fig1.circle('x','y', source = data_tag_pos, size=8, color='green', legend_label = 'target point')
fig1.circle(x = 'cx_vec', y = 'cy_vec', radius = 'radius_vec', source = data_arc, line_alpha = 1, line_width = 2, line_color = "green", fill_color=None, legend_label = 'arc')

fig1.text(0.0, -2.0, text = 'function' ,source = data_text, text_color="firebrick", text_align="center", text_font_size="12pt", legend_label = 'text')

if check_line_arc_line:
  fig1.circle('x','y', source = data_tag_pos2, size=8, color='blue', legend_label = 'tag point2')

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.function_id = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='20%'), description= "function_id",min=0, max=8, value=0, step=1)
    self.ego_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "ego_x",min=-15, max=15, value=0.0, step=0.01)
    self.ego_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "ego_y",min=-15, max=15, value=0.0, step=0.01)
    self.ego_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "ego_heading",min=-180, max=180, value=0, step=0.1)

    self.target_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_x",min=-15, max=15.0, value=0, step=0.01)
    self.target_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_y",min=-15, max=15.0, value=0.3, step=0.01)
    self.target_heading_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "target_heading",min=-180, max=180, value=0, step=0.1)
    self.radius_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "radius",min=0.0, max=20.0, value=5.5, step=0.1)
    self.is_left_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='20%'), description= "is_left",min=0, max=1, value=1, step=1)
    self.is_advance_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='20%'), description= "is_advance",min=0, max=1, value=1, step=1)

    self.obs_x_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obs_x",min=-15, max=15.0, value=0.5, step=0.01)
    self.obs_y_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obs_y",min=-15, max=15.0, value=0.5, step=0.01)

    ipywidgets.interact(slider_callback, function_id = self.function_id,
                                         ego_x = self.ego_x_slider,
                                         ego_y = self.ego_y_slider,
                                         ego_heading = self.ego_heading_slider,
                                         target_x =  self.target_x_slider,
                                         target_y = self.target_y_slider,
                                         target_heading = self.target_heading_slider,
                                         radius = self.radius_slider,
                                         is_left = self.is_left_slider,
                                         is_advance = self.is_advance_slider,
                                         obs_x = self.obs_x_slider,
                                         obs_y = self.obs_y_slider)
### sliders callback
def slider_callback(function_id, ego_x, ego_y, ego_heading, target_x, target_y, target_heading, radius,
                    is_left, is_advance, obs_x, obs_y):
  kwargs = locals()
  ego_heading = - ego_heading
  target_heading = - target_heading

  def GetTangVecByHeading(heading):
      return np.array([np.cos(heading) * 10, np.sin(heading) * 10])
  p1 = np.array([ego_x, ego_y])
  heading1 = ego_heading/57.3  # in radians
  p1_next = p1 + GetTangVecByHeading(heading1)
  p1_previous = p1 - GetTangVecByHeading(heading1)
  p2 = np.array([target_x, target_y])
  heading2 = target_heading/57.3  # in radians
  p2_next = p2 + GetTangVecByHeading(heading2)
  p2_previous = p2 - GetTangVecByHeading(heading2)
  p0 = np.array([obs_x, obs_y])
  if function_id == 0:
    possible_centers = geometry_math_validation_py.CalTangCirOfTwoLine(p1, heading1, p2, heading2, radius)
    xn = []
    yn = []
    radius_vec = []
    for center in possible_centers:
      xn.append(center[0])
      yn.append(center[1])
      radius_vec.append(radius)
    data_arc.data.update({
    'cx_vec':xn,
    'cy_vec':yn,
    'radius_vec':radius_vec,
    })
    data_start_pose_line.data.update({
      'x_vec':[p1_previous[0], p1_next[0]],
      'y_vec':[p1_previous[1], p1_next[1]],
    })
    data_target_pose_line.data.update({
      'x_vec':[p2_previous[0], p2_next[0]],
      'y_vec':[p2_previous[1], p2_next[1]],
    })
    data_start_pos.data.update({
      'x':[ego_x],
      'y':[ego_y],
    })
    data_tag_pos.data.update({
      'x':[p2[0]],
      'y':[p2[1]],
    })
    data_tag_pos2.data.update({
      'x':[],
      'y':[],
    })
    data_text.data.update({'function':['CalTangCirOfTwoLine']})
  elif function_id == 1:
    actual_center_info = geometry_math_validation_py.CalSetTangCirOfTwoLine(p1, heading1, p2, heading2, radius, is_advance, is_left)

    actual_center = actual_center_info[0]
    data_arc.data.update({
    'cx_vec':[actual_center[0]],
    'cy_vec':[actual_center[1]],
    'radius_vec':[radius],
    })
    data_start_pose_line.data.update({
      'x_vec':[p1_previous[0], p1_next[0]],
      'y_vec':[p1_previous[1], p1_next[1]],
    })
    data_target_pose_line.data.update({
      'x_vec':[p2_previous[0], p2_next[0]],
      'y_vec':[p2_previous[1], p2_next[1]],
    })
    data_start_pos.data.update({
      'x':[ego_x],
      'y':[ego_y],
    })
    tangent_pt1 = actual_center_info[1]
    data_tag_pos.data.update({
      'x':[tangent_pt1[0]],
      'y':[tangent_pt1[1]],
    })
    tangent_pt2 = actual_center_info[2]
    data_tag_pos2.data.update({
      'x':[tangent_pt2[0]],
      'y':[tangent_pt2[1]],
    })
    data_text.data.update({'function':['CalSetTangCirOfTwoLine']})
  elif function_id == 2:
    circle_info = geometry_math_validation_py.CalTangCircleByPoseAndLine(p1, heading1, p2, heading2, is_advance, is_left)
    print(circle_info)

    data_arc.data.update({
    'cx_vec':[circle_info[2]],
    'cy_vec':[circle_info[3]],
    'radius_vec':[circle_info[4]],
    })
    data_start_pose_line.data.update({
      'x_vec':[],
      'y_vec':[],
    })
    data_target_pose_line.data.update({
      'x_vec':[p2[0], p2_next[0]],
      'y_vec':[p2[1], p2_next[1]],
    })
    data_start_pos.data.update({
      'x':[ego_x],
      'y':[ego_y],
    })
    data_tag_pos.data.update({
      'x':[circle_info[0]],
      'y':[circle_info[1]],
    })
    data_tag_pos2.data.update({
      'x':[],
      'y':[],
    })
    data_text.data.update({'function':['CalTangCircleByPoseAndLine']})
  elif function_id == 3:
    circle_info = geometry_math_validation_py.CalOneArcByTargetHeading(p1, heading1, p2, heading2, radius, is_advance)
    print(circle_info)

    data_arc.data.update({
    'cx_vec':[circle_info[0]],
    'cy_vec':[circle_info[1]],
    'radius_vec':[radius],
    })
    data_start_pose_line.data.update({
      'x_vec':[],
      'y_vec':[],
    })
    arc_pB = np.array([circle_info[2], circle_info[3]])
    arc_pB_next = arc_pB + GetTangVecByHeading(heading2)
    data_target_pose_line.data.update({
      'x_vec':[arc_pB[0], arc_pB_next[0]],
      'y_vec':[arc_pB[1], arc_pB_next[1]],
    })
    data_start_pos.data.update({
      'x':[ego_x],
      'y':[ego_y],
    })
    data_tag_pos.data.update({
      'x':[circle_info[2]],
      'y':[circle_info[3]],
    })
    data_tag_pos2.data.update({
      'x':[],
      'y':[],
    })
    data_text.data.update({'function':['CalOneArcByTargetHeading']})
  elif function_id == 4:
    circle_info = geometry_math_validation_py.CalTwoArcBySameHeading(p1, heading1, p2, heading2, radius, is_advance)
    print(circle_info)

    data_arc.data.update({
    'cx_vec':[circle_info[0],circle_info[2]],
    'cy_vec':[circle_info[1],circle_info[3]],
    'radius_vec':[radius, radius],
    })
    data_start_pose_line.data.update({
      'x_vec':[],
      'y_vec':[],
    })
    data_target_pose_line.data.update({
      'x_vec':[],
      'y_vec':[],
    })
    data_start_pos.data.update({
      'x':[ego_x],
      'y':[ego_y],
    })
    data_tag_pos.data.update({
      'x':[target_x],
      'y':[target_y],
    })
    data_tag_pos2.data.update({
      'x':[circle_info[4]],
      'y':[circle_info[5]],
    })
    data_text.data.update({'function':['CalTwoArcBySameHeading']})
  elif function_id == 5:
    circle_info = geometry_math_validation_py.CalTwoArcByLine(p1, heading1, p2, heading2, radius, is_advance, is_left)
    print(circle_info)

    data_arc.data.update({
    'cx_vec':[circle_info[0],circle_info[2]],
    'cy_vec':[circle_info[1],circle_info[3]],
    'radius_vec':[radius, radius],
    })
    data_start_pose_line.data.update({
      'x_vec':[],
      'y_vec':[],
    })
    data_target_pose_line.data.update({
      'x_vec':[p2_previous[0], p2_next[0]],
      'y_vec':[p2_previous[1], p2_next[1]],
    })
    data_start_pos.data.update({
      'x':[ego_x],
      'y':[ego_y],
    })
    data_tag_pos.data.update({
      'x':[circle_info[7]],
      'y':[circle_info[8]],
    })
    data_tag_pos2.data.update({
      'x':[circle_info[4]],
      'y':[circle_info[5]],
    })
    data_text.data.update({'function':['CalTwoArcByLine']})
  elif function_id == 6:
    circle_info = geometry_math_validation_py.CalTwoArcByLine(p1, heading1, p2, heading2, radius, is_advance, is_left)
    print(circle_info)

    data_arc.data.update({
    'cx_vec':[circle_info[0],circle_info[2]],
    'cy_vec':[circle_info[1],circle_info[3]],
    'radius_vec':[radius, radius],
    })
    data_start_pose_line.data.update({
      'x_vec':[],
      'y_vec':[],
    })
    data_target_pose_line.data.update({
      'x_vec':[p2_previous[0], p2_next[0]],
      'y_vec':[p2_previous[1], p2_next[1]],
    })
    data_start_pos.data.update({
      'x':[ego_x],
      'y':[ego_y],
    })
    data_tag_pos.data.update({
      'x':[circle_info[7]],
      'y':[circle_info[8]],
    })
    data_tag_pos2.data.update({
      'x':[circle_info[4]],
      'y':[circle_info[5]],
    })
    data_text.data.update({'function':['CalTwoArcByLine']})
  elif function_id == 7:
    circle_info = geometry_math_validation_py.CalOneArcByLine(p1, heading1, p2, heading2, radius, is_advance, is_left)
    print(circle_info)

    data_arc.data.update({
    'cx_vec':[circle_info[0]],
    'cy_vec':[circle_info[1]],
    'radius_vec':[circle_info[2]],
    })
    data_start_pose_line.data.update({
      'x_vec':[],
      'y_vec':[],
    })
    data_target_pose_line.data.update({
      'x_vec':[p2_previous[0], p2_next[0]],
      'y_vec':[p2_previous[1], p2_next[1]],
    })
    data_start_pos.data.update({
      'x':[ego_x],
      'y':[ego_y],
    })
    data_tag_pos.data.update({
      'x':[circle_info[3]],
      'y':[circle_info[4]],
    })
    data_tag_pos2.data.update({
      'x':[],
      'y':[],
    })
    data_text.data.update({'function':['CalOneArcByLine']})
  elif function_id == 8:
    dis_info = geometry_math_validation_py.CalPoint2LineSegDistPb(p0, p1, p2)
    print("dis = ", dis_info[4])

    data_arc.data.update({
    'cx_vec':[],
    'cy_vec':[],
    'radius_vec':[],
    })
    data_start_pose_line.data.update({
      'x_vec':[],
      'y_vec':[],
    })
    data_target_pose_line.data.update({
      'x_vec':[p1[0],p2[0]],
      'y_vec':[p1[1],p2[1]],
    })
    data_start_pos.data.update({
      'x':[obs_x],
      'y':[obs_y],
    })
    data_tag_pos.data.update({
      'x':[],
      'y':[],
    })
    data_tag_pos2.data.update({
      'x':[],
      'y':[],
    })









  push_notebook()

bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


