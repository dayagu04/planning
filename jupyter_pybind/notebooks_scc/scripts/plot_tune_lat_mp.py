import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
from lib.load_tune_lat_mp import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')
from python_proto import lateral_motion_planner_pb2
from jupyter_pybind import lateral_motion_planning_py
from bokeh.resources import INLINE
# bag path and frame dt
bag_path = "/share//data_cold/abu_zone/hpp/1219bag/memory1219_12.00000"
bag_path = "/data_cold/abu_zone/cailiu2/0802/165_66a1bb682933546a92b11980_66a77fb17af211090bc13d38.bag.PP"

frame_dt = 0.1 # sec
steer_ratio = 13.0 # e0y

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
# global_var.set_value('g_is_display_enu', True)
fig1, local_view_data = load_local_view_figure()
fig1.height = 1500
# init pybind
lateral_motion_planning_py.Init()

lat_motion_plan_input0 = bag_loader.plan_debug_msg['data'][-1].lateral_motion_planning_input

# load lateral planning (behavior and motion)
fig1, fig2, fig3, fig4, fig5, fig6, fig7, lat_plan_data = load_lat_plan_figure(fig1)
load_measure_distance_tool(fig1)
load_measure_distance_tool(fig7)

init_info_name = ["dbw status", "replan status", "lat err", "theta err", "lon err", "dist err", "init theta", "ego theta", "ref theta", "ref vel", "steer angle", "steer angle rate", "ego_lat_jerk"]
init_info = ColumnDataSource(data = {'name':[], 'init info':[]})
init_info_columns = [
        TableColumn(field="name", title="name"),
        TableColumn(field="init info", title="init info"),
      ]
tab2 = DataTable(source = init_info, columns = init_info_columns, width = 300, height = 400)

param_name = ["q_ref_xy", "q_ref_theta", "q_acc", "q_jerk", "q_continuity", "q_acc_bound", "q_jerk_bound", "acc_bound", "jerk_bound", "q_safe_bound", "q_hard_bound", "start_q_jerk"]
param = ColumnDataSource(data = {'name':[], 'origin param':[], 'new param':[]})
param_columns = [
        TableColumn(field="name", title="name"),
        TableColumn(field="origin param", title="origin param"),
        TableColumn(field="new param", title="current param"),
      ]
tab3 = DataTable(source = param, columns = param_columns, width = 450, height = 500)

def get_plan_debug_msg_idx(bag_loader, bag_time):
  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
    plan_debug_msg = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx]
  return plan_debug_msg_idx

def get_vs_msg_idx(bag_loader, bag_time):
  vs_msg_idx = 0
  if bag_loader.vs_msg['enable'] == True:
    while bag_loader.vs_msg['t'][vs_msg_idx] <= bag_time and vs_msg_idx < (len(bag_loader.vs_msg['t'])-2):
        vs_msg_idx = vs_msg_idx + 1
    vs_msg = bag_loader.vs_msg['data'][vs_msg_idx]
  return vs_msg_idx

fig8 = bkp.figure(x_axis_label='time', y_axis_label='steer deg', width=600, height=160)
fig9 = bkp.figure(x_axis_label='time', y_axis_label='steer dot deg', width=600, height=160)
fig10 = bkp.figure(x_axis_label='s', y_axis_label='k_radius', width=600, height=160)
data_steer = ColumnDataSource(data ={
  'time': [],
  'plan_steer_deg':[],
  'plan_steer_dot_deg':[],
  'ego_steer_deg':[],
  'ego_steer_dot_deg':[]
})
steer_time = []
plan_steer_deg = []
plan_steer_dot_deg = []
ego_steer_deg = []
ego_steer_dot_deg = []
for t in np.arange(0.0, max_time, frame_dt):
  steer_time.append(t)
  plan_debug_msg_idx = get_plan_debug_msg_idx(bag_loader, t)
  lateral_motion_planning_output = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_output
  if (len(lateral_motion_planning_output.delta_vec) > 0):
    plan_steer_deg.append(lateral_motion_planning_output.delta_vec[0] * 13 * 57.3)
    plan_steer_dot_deg.append(lateral_motion_planning_output.omega_vec[0] * 13 * 57.3)
  else:
    plan_steer_deg.append(0.0)
    plan_steer_dot_deg.append(0.0)
  vs_msg_idx = get_vs_msg_idx(bag_loader, t)
  vs_msg = bag_loader.vs_msg['data'][vs_msg_idx]
  ego_steer_deg.append(vs_msg.steering_wheel_angle * 57.3)
  ego_steer_dot_deg.append(vs_msg.steering_wheel_angle_speed * 57.3)

data_steer.data.update({
  'time': steer_time,
  'plan_steer_deg': plan_steer_deg,
  'plan_steer_dot_deg': plan_steer_dot_deg,
  'ego_steer_deg': ego_steer_deg,
  'ego_steer_dot_deg': ego_steer_dot_deg,
})
f8 = fig8.line('time', 'plan_steer_deg', source = data_steer, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'plan_steer_deg')
fig8.line('time', 'ego_steer_deg', source = data_steer, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'ego_steer_deg')
f9 = fig9.line('time', 'plan_steer_dot_deg', source = data_steer, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'plan_steer_dot_deg')
fig9.line('time', 'ego_steer_dot_deg', source = data_steer, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'ego_steer_dot_deg')
f10_1 = fig10.line('center_line_s', 'center_line_curvature', source = lat_plan_data['data_center_line_curvature'], line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'kappa radius')
fig10.line('center_line_s', 'center_line_d_poly_curvature', source = lat_plan_data['data_center_line_curvature'], line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'road radius')
refline_kappa_radius = ColumnDataSource(data = {'refline_s':[], 'refline_curvature':[]})
f10_2 = fig10.line('refline_s', 'refline_curvature', source = refline_kappa_radius, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'ref kappa radius')
fig10.line('center_line_s', 'center_line_confidence', source = lat_plan_data['data_center_line_curvature'], line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'road confidence')
hover8 = HoverTool(renderers=[f8], tooltips=[('time', '@time'), ('plan_steer_deg', '@plan_steer_deg'), ('ego_steer_deg', '@ego_steer_deg')], mode='vline')
hover9 = HoverTool(renderers=[f9], tooltips=[('time', '@time'), ('plan_steer_dot_deg', '@plan_steer_dot_deg'), ('ego_steer_dot_deg', '@ego_steer_dot_deg')], mode='vline')
hover10_1 = HoverTool(renderers=[f10_1], tooltips=[('s', '@center_line_s'), ('radius', '@center_line_curvature')], mode='vline')
hover10_2 = HoverTool(renderers=[f10_2], tooltips=[('s', '@refline_s'), ('radius', '@refline_curvature')], mode='vline')
fig8.add_tools(hover8)
fig9.add_tools(hover9)
fig10.add_tools(hover10_1)
fig10.add_tools(hover10_2)
fig8.toolbar.active_scroll = fig8.select_one(WheelZoomTool)
fig9.toolbar.active_scroll = fig9.select_one(WheelZoomTool)
fig10.toolbar.active_scroll = fig10.select_one(WheelZoomTool)
fig8.legend.click_policy = 'hide'
fig9.legend.click_policy = 'hide'
fig10.legend.click_policy = 'hide'

coord_tf = coord_transformer()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.q_ref_xy_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_ref_xy",min=0.1, max=1000.0, value=lat_motion_plan_input0.q_ref_x, step=0.1)
    self.q_ref_theta_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_ref_theta",min=0.1, max=100000.0, value=lat_motion_plan_input0.q_ref_theta, step=0.1)
    self.q_acc_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_acc",min=0.1, max=1000.0, value=lat_motion_plan_input0.q_acc, step=0.01)
    self.q_jerk_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_jerk",min=0.01, max=1000.0, value=lat_motion_plan_input0.q_jerk, step=0.01)
    self.q_continuity_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_continuity",min=0.0, max=10.0, value=lat_motion_plan_input0.q_continuity, step=0.01)

    self.q_acc_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_acc_bound",min=0.0, max=10000.0, value=lat_motion_plan_input0.q_acc_bound, step=0.1)
    self.q_jerk_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_jerk_bound",min=0.0, max=1000000.0, value=lat_motion_plan_input0.q_jerk_bound, step=0.1)
    self.acc_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "acc_bound",min=0.0, max=10.0, value=lat_motion_plan_input0.acc_bound, step=0.1)
    self.jerk_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "jerk_bound",min=0.0, max=10.0, value=lat_motion_plan_input0.jerk_bound, step=0.1)
    self.q_safe_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_safe_bound",min=0.0, max=10000.0, value=lat_motion_plan_input0.q_soft_corridor, step=0.1)
    self.q_hard_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_hard_bound",min=0.0, max=10000.0, value=lat_motion_plan_input0.q_hard_corridor, step=0.1)

    self.ref_xy_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "ref_xy",min=-10., max=10.0, value=0.0, step=0.1)
    self.upper_safe_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "upper_safe_bound",min=-10., max=10.0, value=0.0, step=0.05)
    self.lower_safe_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "lower_safe_bound",min=-10., max=10.0, value=0.0, step=0.05)
    self.upper_hard_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "upper_hard_bound",min=-10., max=10.0, value=0.0, step=0.05)
    self.lower_hard_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "lower_hard_bound",min=-10., max=10.0, value=0.0, step=0.05)

    self.complete_follow = ipywidgets.Checkbox(value=lat_motion_plan_input0.complete_follow, description='complete_follow')
    self.motion_plan_concerned_start_index = ipywidgets.IntText(value=0, description='motion_plan_concerned_start_index:')
    self.motion_plan_concerned_end_index = ipywidgets.IntText(value=lat_motion_plan_input0.motion_plan_concerned_index, description='motion_plan_concerned_end_index:')
    self.q_start_jerk_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_start_jerk",min=0.0, max=1000.0, value=lat_motion_plan_input0.q_jerk, step=0.01)
    self.curv_factor_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "curv_factor",min=0.0, max=1.0, value=lat_motion_plan_input0.curv_factor, step=0.01)
    self.expected_acc_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "expected_acc",min=-5.0, max=5.0, value=0.0, step=0.01)
    self.start_acc_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "start_acc",min=0.1, max=1000.0, value=lat_motion_plan_input0.q_acc, step=0.01)
    self.end_acc_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "end_acc",min=0.1, max=1000.0, value=lat_motion_plan_input0.q_acc, step=0.01)
    self.end_ratio1_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "end_ratio1",min=0.0, max=10.0, value=0.3, step=0.1)
    self.end_ratio2_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "end_ratio2",min=0.0, max=10.0, value=0.3, step=0.1)
    self.end_ratio3_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "end_ratio3",min=0.0, max=10.0, value=1.5, step=0.1)
    self.max_iter_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='50%'), description= "max_iter",min=0, max=10, value=10, step=1)

    self.safe_ub_start_idx = ipywidgets.IntText(value=0, description='safe_ub_start_idx:')
    self.safe_ub_end_idx = ipywidgets.IntText(value=26, description='safe_ub_end_idx:')
    self.safe_lb_start_idx = ipywidgets.IntText(value=0, description='safe_lb_start_idx:')
    self.safe_lb_end_idx = ipywidgets.IntText(value=26, description='safe_lb_end_idx:')
    self.hard_ub_start_idx = ipywidgets.IntText(value=0, description='hard_ub_start_idx:')
    self.hard_ub_end_idx = ipywidgets.IntText(value=26, description='hard_ub_end_idx:')
    self.hard_lb_start_idx = ipywidgets.IntText(value=0, description='hard_lb_start_idx:')
    self.hard_lb_end_idx = ipywidgets.IntText(value=26, description='hard_lb_end_idx:')

    self.use_new_param = ipywidgets.Checkbox(value=False, description='use_new_param')
    self.bag_dt_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "bag_dt",min=-10.0, max=10.0, value=0.1, step=0.1)

    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                         bag_dt = self.bag_dt_slider,
                                         use_new_param = self.use_new_param,
                                         q_ref_xy = self.q_ref_xy_slider,
                                         q_ref_theta = self.q_ref_theta_slider,
                                         q_acc = self.q_acc_slider,
                                         q_jerk = self.q_jerk_slider,
                                         q_continuity = self.q_continuity_slider,
                                         q_acc_bound = self.q_acc_bound_slider,
                                         q_jerk_bound = self.q_jerk_bound_slider,
                                         acc_bound = self.acc_bound_slider,
                                         jerk_bound = self.jerk_bound_slider,
                                         q_safe_bound = self.q_safe_bound_slider,
                                         q_hard_bound = self.q_hard_bound_slider,
                                         ref_xy = self.ref_xy_slider,
                                         upper_safe_bound = self.upper_safe_bound,
                                         lower_safe_bound = self.lower_safe_bound,
                                         upper_hard_bound = self.upper_hard_bound,
                                         lower_hard_bound = self.lower_hard_bound,
                                         safe_ub_start_idx= self.safe_ub_start_idx,
                                         safe_ub_end_idx= self.safe_ub_end_idx,
                                         safe_lb_start_idx= self.safe_lb_start_idx,
                                         safe_lb_end_idx= self.safe_lb_end_idx,
                                         hard_ub_start_idx= self.hard_ub_start_idx,
                                         hard_ub_end_idx= self.hard_ub_end_idx,
                                         hard_lb_start_idx= self.hard_lb_start_idx,
                                         hard_lb_end_idx= self.hard_lb_end_idx,
                                         complete_follow = self.complete_follow,
                                         motion_plan_concerned_start_index = self.motion_plan_concerned_start_index,
                                         motion_plan_concerned_end_index = self.motion_plan_concerned_end_index,
                                         q_start_jerk = self.q_start_jerk_slider,
                                         curv_factor = self.curv_factor_slider,
                                         expected_acc = self.expected_acc_slider,
                                         start_acc = self.start_acc_slider,
                                         end_acc = self.end_acc_slider,
                                         end_ratio1 = self.end_ratio1_slider,
                                         end_ratio2 = self.end_ratio2_slider,
                                         end_ratio3 = self.end_ratio3_slider,
                                         max_iter = self.max_iter_slider)


### sliders callback
def slider_callback(bag_time, bag_dt, use_new_param, q_ref_xy, q_ref_theta, q_acc, q_jerk, q_continuity, q_acc_bound, q_jerk_bound, acc_bound, jerk_bound, q_safe_bound, q_hard_bound, ref_xy, upper_safe_bound, lower_safe_bound,
                    upper_hard_bound, lower_hard_bound, safe_ub_start_idx, safe_ub_end_idx, safe_lb_start_idx, safe_lb_end_idx, hard_ub_start_idx, hard_ub_end_idx, hard_lb_start_idx, hard_lb_end_idx,
                    complete_follow, motion_plan_concerned_start_index, motion_plan_concerned_end_index, q_start_jerk, curv_factor, expected_acc, start_acc, end_acc, end_ratio1, end_ratio2, end_ratio3, max_iter):
  g_is_display_enu = global_var.get_value('g_is_display_enu')
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_tune_lat_plan_data(fig7, bag_loader, bag_time, bag_time + bag_dt, local_view_data, lat_plan_data, ref_xy, upper_safe_bound, lower_safe_bound, upper_hard_bound, lower_hard_bound, safe_ub_start_idx, safe_ub_end_idx, safe_lb_start_idx, safe_lb_end_idx, hard_ub_start_idx, hard_ub_end_idx, hard_lb_start_idx, hard_lb_end_idx, g_is_display_enu)

  vs_msg = find_nearest(bag_loader.vs_msg, bag_time)
  plan_msg = local_view_data['data_msg']['plan_msg']
  plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
  loc_msg = local_view_data['data_msg']['loc_msg']
  plan_debug_json_msg = local_view_data['data_msg']['plan_debug_json_msg']

  lat_motion_plan_input = plan_debug_msg.lateral_motion_planning_input
  planning_json = plan_debug_json_msg
  lat_behavior_common = plan_debug_msg.lat_behavior_common
  print("init curv: ",lat_motion_plan_input.init_state.curv)
  print("road curv: ",planning_json["road_radius"])
  print("far_kappa_radius", planning_json["far_kappa_radius"])
  print("left_turn_light_state_available: ",vs_msg.left_turn_light_state_available)
  print("left_turn_light_state: ",vs_msg.left_turn_light_state)
  print("right_turn_light_state_available: ",vs_msg.right_turn_light_state_available)
  print("right_turn_light_state: ",vs_msg.right_turn_light_state)
  print("current_state:", lat_behavior_common.current_state)
  if lat_behavior_common.current_state in [1, 2, 3]:
    print("Lane Change")
  elif lat_behavior_common.current_state == 4:
    print("Lane Change Back")
  else:
    print("Cruising")

  try:
    refline_kappa_radius.data.update({
      'refline_s': planning_json['raw_refline_s_vec'],
      'refline_curvature': planning_json['raw_refline_k_vec'],
    })
  except:
    print("no plan debug json: raw_refline_k_vec")
  if len(lat_motion_plan_input.ref_theta_vec) > 0:
    ref_vel = lat_motion_plan_input.ref_vel
    ego_vel = vs_msg.vehicle_speed
    init_info_vec = []
    init_info_vec.append(planning_json['dbw_status'])
    init_info_vec.append(planning_json['replan_status'])
    init_info_vec.append(planning_json['lat_err'])
    init_info_vec.append(planning_json['theta_err'])
    init_info_vec.append(planning_json['lon_err'])
    init_info_vec.append(planning_json['dist_err'])
    init_info_vec.append(lat_motion_plan_input.init_state.theta)
    init_info_vec.append(loc_msg.orientation.euler_boot.yaw)
    init_info_vec.append(lat_motion_plan_input.ref_theta_vec[0])
    init_info_vec.append(ref_vel)
    init_info_vec.append(vs_msg.steering_wheel_angle * 57.3)
    init_info_vec.append(vs_msg.steering_wheel_angle_speed * 57.3)
    init_info_vec.append(lat_motion_plan_input.curv_factor * ego_vel * ego_vel * (vs_msg.steering_wheel_angle_speed / steer_ratio))
    init_info.data.update({
      'name': init_info_name,
      'init info': init_info_vec,
    })

    origin_param_vec = []
    origin_param_vec.append(lat_motion_plan_input.q_ref_x)
    origin_param_vec.append(lat_motion_plan_input.q_ref_theta)
    origin_param_vec.append(lat_motion_plan_input.q_acc)
    origin_param_vec.append(lat_motion_plan_input.q_jerk)
    origin_param_vec.append(lat_motion_plan_input.q_continuity)
    origin_param_vec.append(lat_motion_plan_input.q_acc_bound)
    origin_param_vec.append(lat_motion_plan_input.q_jerk_bound)
    origin_param_vec.append(lat_motion_plan_input.acc_bound)
    origin_param_vec.append(lat_motion_plan_input.jerk_bound)
    origin_param_vec.append(lat_motion_plan_input.q_soft_corridor)
    origin_param_vec.append(lat_motion_plan_input.q_hard_corridor)
    origin_param_vec.append(planning_json['concerned_start_q_jerk'])

    if not use_new_param:
      q_ref_xy = lat_motion_plan_input.q_ref_x
      q_ref_theta = lat_motion_plan_input.q_ref_theta
      q_acc = lat_motion_plan_input.q_acc
      q_jerk = lat_motion_plan_input.q_jerk
      q_continuity = lat_motion_plan_input.q_continuity
      q_acc_bound = lat_motion_plan_input.q_acc_bound
      q_jerk_bound = lat_motion_plan_input.q_jerk_bound
      acc_bound = lat_motion_plan_input.acc_bound
      jerk_bound = lat_motion_plan_input.jerk_bound
      q_safe_bound = lat_motion_plan_input.q_soft_corridor
      q_hard_bound = lat_motion_plan_input.q_hard_corridor
      complete_follow = lat_motion_plan_input.complete_follow
      motion_plan_concerned_end_index = lat_motion_plan_input.motion_plan_concerned_index
      if planning_json['concerned_start_q_jerk'] > 0:
        q_start_jerk = planning_json['concerned_start_q_jerk']

    new_param_vec = []
    new_param_vec.append(q_ref_xy)
    new_param_vec.append(q_ref_theta)
    new_param_vec.append(q_acc)
    new_param_vec.append(q_jerk)
    new_param_vec.append(q_continuity)
    new_param_vec.append(q_acc_bound)
    new_param_vec.append(q_jerk_bound)
    new_param_vec.append(acc_bound)
    new_param_vec.append(jerk_bound)
    new_param_vec.append(q_safe_bound)
    new_param_vec.append(q_hard_bound)
    new_param_vec.append(q_start_jerk)

    param.data.update({
      'name': param_name,
      'origin param': origin_param_vec,
      'new param': new_param_vec,
    })

    ref_s = ref_vel * 25.0 * 0.2
    print("ref_s:", ref_s)
    input_string = lat_motion_plan_input.SerializeToString()
    start_time = time.time()
    lateral_motion_planning_py.UpdateByParams(input_string, q_ref_xy, q_ref_theta, q_acc, q_jerk, q_continuity, q_acc_bound, q_jerk_bound, acc_bound, jerk_bound, q_safe_bound, q_hard_bound,
                                              ref_xy, upper_safe_bound, lower_safe_bound, upper_hard_bound, lower_hard_bound, safe_ub_start_idx, safe_ub_end_idx,
                                              safe_lb_start_idx, safe_lb_end_idx, hard_ub_start_idx, hard_ub_end_idx, hard_lb_start_idx, hard_lb_end_idx, complete_follow,
                                              motion_plan_concerned_start_index, motion_plan_concerned_end_index, curv_factor, q_start_jerk, max(ego_vel, 1.5), expected_acc,
                                              start_acc, end_acc, end_ratio1, end_ratio2, end_ratio3, max_iter)
    end_time = time.time()
    planning_output = lateral_motion_planner_pb2.LateralPlanningOutput()
    output_string_tmp = lateral_motion_planning_py.GetOutputBytes()
    planning_output.ParseFromString(output_string_tmp)

    print("\n ------------------------------------------- \n")
    try:
      delta_bound = acc_bound / (lat_motion_plan_input.curv_factor * max(ego_vel, 1.0) * max(ego_vel, 1.0))
      omega_bound = jerk_bound / (lat_motion_plan_input.curv_factor * max(ego_vel, 1.0) * max(ego_vel, 1.0))
    except:
      delta_bound = 540.0 / steer_ratio / 57.3
      omega_bound = 360.0 / steer_ratio / 57.3
      print("no ego_vel!")
    print("origin complete_follow : ", lat_motion_plan_input.complete_follow)
    print("origin motion_plan_concerned_end_index : ", lat_motion_plan_input.motion_plan_concerned_index)
    print("new complete_follow : ", complete_follow)
    print("new motion_plan_concerned_end_index : ", motion_plan_concerned_end_index)
    print("new motion_plan_concerned_start_index : ", motion_plan_concerned_start_index)

    theta_error = lat_motion_plan_input.init_state.theta - lat_motion_plan_input.ref_theta_vec[0]
    theta_error1 = loc_msg.orientation.euler_boot.yaw - lat_motion_plan_input.ref_theta_vec[0]
    theta_error2 = lat_motion_plan_input.init_state.theta - loc_msg.orientation.euler_boot.yaw
    print("init theta - ref theta = ", theta_error, " rad ---> ", theta_error * 57.3, " deg")
    print("ego theta - ref theta = ", theta_error1, " rad ---> ", theta_error1 * 57.3, " deg")
    print("init theta - ego theta = ", theta_error2, " rad ---> ", theta_error2 * 57.3, " deg")

    print("cost size = ", planning_output.solver_info.cost_size)
    print("min cost = ", planning_output.solver_info.iter_info[max(planning_output.solver_info.iter_count - 1, 0)].cost)
    print("ilqr iter count = ", planning_output.solver_info.iter_count)
    for i in range(len(planning_output.solver_info.iter_info)):
      print("iteration[", (i + 1), "]: backward count = ", planning_output.solver_info.iter_info[i].backward_pass_count)
    print("new solve time:", float(end_time - start_time) * 1000.0, "ms")
    print("solver condition = ", planning_output.solver_info.solver_condition)

    if bag_loader.loc_msg['enable'] == True:
      cur_pos_xn = loc_msg.position.position_boot.x
      cur_pos_yn = loc_msg.position.position_boot.y
      cur_yaw = loc_msg.orientation.euler_boot.yaw

    # try:
    #   json_pos_x = planning_json['ego_pos_x']
    #   json_pos_y = planning_json['ego_pos_y']
    #   json_yaw = planning_json['ego_pos_yaw']
    #   coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
    # except:
    coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)

    if g_is_display_enu:
      x_vec, y_vec = planning_output.x_vec, planning_output.y_vec
    else:
      x_vec, y_vec = coord_tf.global_to_local(planning_output.x_vec, planning_output.y_vec)
    time_vec = planning_output.time_vec

    ref_theta_deg_vec = []
    theta_deg_vec = []
    steer_deg_vec = []
    steer_dot_deg_vec =[]
    acc_upper_bound = []
    acc_lower_bound = []
    jerk_upper_bound = []
    jerk_lower_bound = []
    steer_deg_upper_bound = []
    steer_deg_lower_bound = []
    steer_dot_deg_upper_bound = []
    steer_dot_deg_lower_bound = []

    for i in range(len(time_vec)):
      ref_theta_deg_vec.append(lat_motion_plan_input.ref_theta_vec[i] * 57.3)
      theta_deg_vec.append(planning_output.theta_vec[i] * 57.3)
      steer_deg_vec.append(planning_output.delta_vec[i] * 57.3 * steer_ratio)
      steer_dot_deg_vec.append(planning_output.omega_vec[i] * 57.3 * steer_ratio)
      acc_upper_bound.append(acc_bound)
      acc_lower_bound.append(-acc_bound)
      jerk_upper_bound.append(jerk_bound)
      jerk_lower_bound.append(-jerk_bound)
      steer_dot_deg_upper_bound.append((omega_bound * 57.3 * steer_ratio))
      steer_dot_deg_lower_bound.append(-(omega_bound * 57.3 * steer_ratio))
      steer_deg_upper_bound.append((delta_bound * 57.3 * steer_ratio))
      steer_deg_lower_bound.append(-(delta_bound * 57.3 * steer_ratio))

    acc_vec = planning_output.acc_vec
    jerk_vec = planning_output.jerk_vec

    xn_vec_t, yn_vec_t = planning_output.x_vec, planning_output.y_vec
    if g_is_display_enu:
      xn_vec_t, yn_vec_t = coord_tf.global_to_local(planning_output.x_vec, planning_output.y_vec)

    lat_plan_data['data_lat_motion_plan_output'].data.update({
      'time_vec_t': time_vec,
      'x_vec_t': x_vec,
      'y_vec_t': y_vec,
      'xn_vec_t': xn_vec_t,
      'yn_vec_t': yn_vec_t,
      'ref_theta_deg_vec_t': ref_theta_deg_vec,
      'theta_deg_vec_t': theta_deg_vec,
      'steer_deg_vec_t': steer_deg_vec,
      'steer_dot_deg_vec_t': steer_dot_deg_vec,
      'acc_vec_t': acc_vec,
      'jerk_vec_t': jerk_vec,
      'acc_upper_bound': acc_upper_bound,
      'acc_lower_bound': acc_lower_bound,
      'jerk_upper_bound': jerk_upper_bound,
      'jerk_lower_bound': jerk_lower_bound,
      'steer_deg_upper_bound': steer_deg_upper_bound,
      'steer_deg_lower_bound': steer_deg_lower_bound,
      'steer_dot_deg_upper_bound': steer_dot_deg_upper_bound,
      'steer_dot_deg_lower_bound': steer_dot_deg_lower_bound,
    })

  push_notebook()

bkp.show(row(fig1, column(fig7, row(tab2, tab3)), column(fig2, fig3, fig4, fig5, fig6, fig8, fig9, fig10)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
