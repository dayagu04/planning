import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
sys.path.append('../..')
sys.path.append('../../../')
from bokeh.resources import INLINE

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20240701/20240701-18-12-17/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2024-07-01-18-12-17_no_camera.bag"

frame_dt = 0.02 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

#bag_loader = LoadCyberbag(bag_path)
bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
global_var.set_value('g_is_display_enu', False)
fig1, local_view_data = load_local_view_figure()

data_control_command = ColumnDataSource(data ={
  'time': [],
  'steering_cmd_deg':[],
  'steering_deg':[],
  # 'acc_ego':[],
  # 'acc_vel':[],
  'vel_ego':[],
  'vel_cmd':[],
  'vel_raw_cmd':[],
  'vel_error':[],
  # 'vel_fdbk_out':[],
  # 'vel_ffwd_out':[],
  # 'vel_out':[],
  # 'lon_err':[],
  'lat_err':[],
  'phi_err':[],
  'controller_status': [],
  'lat_enable': [],
  'lon_enable': [],
  'wheel_angle_cmd': [],
  'lat_mpc_status': [],
  'planning_type': [],
  'planning_time_offset': [],
  'planning_update_flag': [],
  'vel_ffwd_out': [],
  'vel_fdbk_out': [],
  'vel_raw_out': [],
  'vel_out': [],
  'vel_KP_term': [],
  'vel_KI_term': [],
  'acc_ego': [],
  'acc_vel': [],
  'slope_acc': [],
  'euler_angle_yaw': [],
  'euler_angle_pitch': [],
})

data_cursor_fig2 = ColumnDataSource(data ={
  'x': [],
  'y': [],
})

data_cursor_fig3 = ColumnDataSource(data ={
  'x': [],
  'y': [],
})

data_cursor_fig4 = ColumnDataSource(data ={
  'x': [],
  'y': [],
})

data_cursor_fig8 = ColumnDataSource(data ={
  'x': [],
  'y': [],
})

data_cursor_fig9 = ColumnDataSource(data ={
  'x': [],
  'y': [],
})

data_cursor_fig11 = ColumnDataSource(data ={
  'x': [],
  'y': [],
})

data_mpc = ColumnDataSource(data ={
  'time_vec': [],
  'dx_ref_mpc_vec':[],
  'dy_ref_mpc_vec':[],
  'dx_mpc_vec':[],
  'dy_mpc_vec':[],
  'dphi_ref_mpc_vec':[],
  'dphi_mpc_vec':[],
  'delta_mpc_vec': []
})

steering_cmd_deg = []
vel_ego = []
vel_cmd = []
vel_raw_cmd = []
vel_error = []
lat_err = []
steering_deg = []
acceleration = []
axle_torque = []
t_ctrl_debug = []
controller_status = []
lat_enable = []
lon_enable = []
wheel_angle_cmd = []
lat_mpc_status = []
planning_type = []
driver_hand_torque = []
planning_time_offset = []
planning_update_flag= []
vel_ffwd_out = []
vel_fdbk_out = []
vel_raw_out = []
vel_out = []
vel_KP_term = []
vel_KI_term = []
acc_ego = []
acc_vel = []
slope_acc = []
yaw_conti = []
steer_angle_bias_deg = []
steer_bias_deg = []
axle_torque = []
throttle_brake = []
euler_angle_yaw = []
euler_angle_pitch = []
phi_err = []

t = 0.0

ctrl_json_data = bag_loader.ctrl_debug_msg['json']
time_all = bag_loader.ctrl_msg['t'][-1]

dt_json = time_all / len(ctrl_json_data)

for i in range(len(ctrl_json_data)):
  t_ctrl_debug.append(t)
  steering_cmd_deg.append(ctrl_json_data[i]['steer_angle_cmd'] * 57.3)
  steering_deg.append(ctrl_json_data[i]['steer_angle'] * 57.3)
  vel_ego.append(ctrl_json_data[i]['vel_ego'])
  vel_cmd.append(ctrl_json_data[i]['vel_cmd'])
  vel_raw_cmd.append(ctrl_json_data[i]['vel_raw_cmd'])
  vel_error.append(ctrl_json_data[i]['vel_error'])
  lat_err.append(ctrl_json_data[i]['lat_err'])
  controller_status.append(ctrl_json_data[i]['controller_status'])
  lat_enable.append(ctrl_json_data[i]['lat_enable'])
  lon_enable.append(ctrl_json_data[i]['lon_enable'])
  wheel_angle_cmd.append(ctrl_json_data[i]['wheel_angle_cmd'] * 15.7 * 57.3)
  lat_mpc_status.append(ctrl_json_data[i]['lat_mpc_status'])
  planning_type.append(ctrl_json_data[i]['planning_type'])
  driver_hand_torque.append(ctrl_json_data[i]['driver_hand_torque'])
  planning_time_offset.append(ctrl_json_data[i]['planning_time_offset'])
  planning_update_flag.append(ctrl_json_data[i]['planning_update_flag'])
  vel_ffwd_out.append(ctrl_json_data[i]['vel_ffwd_out'])
  vel_fdbk_out.append(ctrl_json_data[i]['vel_fdbk_out'])
  vel_raw_out.append(ctrl_json_data[i]['vel_raw_out'])
  vel_out.append(ctrl_json_data[i]['vel_out'])
  vel_KP_term.append(ctrl_json_data[i]['vel_KP_term'])
  vel_KI_term.append(ctrl_json_data[i]['vel_KI_term'])
  acc_ego.append(ctrl_json_data[i]['acc_ego'])
  acc_vel.append(ctrl_json_data[i]['acc_vel'])
  slope_acc.append(ctrl_json_data[i]['slope_acc'])
  yaw_conti.append(ctrl_json_data[i]['yaw_conti'] * 57.3)
  steer_angle_bias_deg.append(-ctrl_json_data[i]['steer_angle_bias_deg'])
  steer_bias_deg.append(-ctrl_json_data[i]['steer_bias_deg'])
  axle_torque.append(ctrl_json_data[i]['axle_torque'] / 1000)
  tmp_throttle_brake = ctrl_json_data[i]['throttle_brake']
  if tmp_throttle_brake > 0.0:
    tmp_throttle_brake = tmp_throttle_brake / 1000
  throttle_brake.append(tmp_throttle_brake)
  euler_angle_yaw.append(ctrl_json_data[i]['euler_angle_yaw'] * 57.3)
  euler_angle_pitch.append(ctrl_json_data[i]['euler_angle_pitch'] * 57.3)
  phi_err.append(ctrl_json_data[i]['phi_err'] * 57.3)
  t = t + 0.02

data_control_command.data.update({
  'time': t_ctrl_debug,
  'steering_deg': steering_deg,
  'steering_cmd_deg': steering_cmd_deg,
  # 'acc_ego':[],
  # 'acc_vel':[],
  'vel_ego': vel_ego,
  'vel_cmd': vel_cmd,
  'vel_raw_cmd': vel_raw_cmd,
  'vel_error':vel_error,
  # 'vel_fdbk_out':[],
  # 'vel_ffwd_out':[],
  # 'vel_out':[],
  # 'lon_err':[],
  'lat_err': lat_err,
  'phi_err':phi_err,
  'controller_status': controller_status,
  'lat_enable': lat_enable,
  'lon_enable': lon_enable,
  'wheel_angle_cmd': wheel_angle_cmd,
  'lat_mpc_status': lat_mpc_status,
  'planning_type': planning_type,
  'driver_hand_torque': driver_hand_torque,
  'planning_time_offset': planning_time_offset,
  'planning_update_flag': planning_update_flag,
  'vel_ffwd_out': vel_ffwd_out,
  'vel_fdbk_out': vel_fdbk_out,
  'vel_raw_out': vel_raw_out,
  'vel_out': vel_out,
  'vel_KP_term': vel_KP_term,
  'vel_KI_term': vel_KI_term,
  'acc_ego': acc_ego,
  'acc_vel': acc_vel,
  'slope_acc': slope_acc,
  'yaw_conti': yaw_conti,
  'steer_angle_bias_deg': steer_angle_bias_deg,
  'steer_bias_deg': steer_bias_deg,
  'axle_torque': axle_torque,
  'throttle_brake': throttle_brake,
  'euler_angle_yaw': euler_angle_yaw,
  'euler_angle_pitch': euler_angle_pitch
})

# figures
fig1.line('dy_ref_mpc_vec', 'dx_ref_mpc_vec', source = data_mpc, line_width = 5, line_color = 'red', line_dash = 'solid', line_alpha = 0.35, legend_label = 'ref', visible=True)
fig1.line('dy_mpc_vec', 'dx_mpc_vec', source = data_mpc, line_width = 5, line_color = 'blue', line_dash = 'solid', line_alpha = 0.35, legend_label = 'mpc', visible=True)
fig1.width = 600
fig1.height = 1200

fig2 = bkp.figure(x_axis_label='time', y_axis_label='status',x_range = [t_ctrl_debug[0], t_ctrl_debug[-1]], width=700, height=200)
fig3 = bkp.figure(x_axis_label='time', y_axis_label='steering angle',x_range = fig2.x_range, width=700, height=200)
fig4 = bkp.figure(x_axis_label='time', y_axis_label='vel',x_range = fig2.x_range, width=700, height=200)

fig5 = bkp.figure(x_axis_label='time', y_axis_label='dy',x_range = [-0.1, 2.6], width=600, height=200)
fig6 = bkp.figure(x_axis_label='time', y_axis_label='dphi',x_range = fig5.x_range, width=600, height=200)
fig7 = bkp.figure(x_axis_label='time', y_axis_label='delta',x_range = fig5.x_range, width=600, height=200)

fig8 = bkp.figure(x_axis_label='time', y_axis_label='vel ctrl',x_range = fig2.x_range, width=700, height=200)
fig9 = bkp.figure(x_axis_label='time', y_axis_label='acc state',x_range = fig2.x_range, width=700, height=200)
fig10 = bkp.figure(x_axis_label='time', y_axis_label='atti',x_range = fig2.x_range, width=700, height=200)
fig11 = bkp.figure(x_axis_label='time', y_axis_label='err',x_range = fig2.x_range, width=700, height=200)

f2 = fig2.line('time', 'controller_status', source = data_control_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'controller_status')
fig2.line('time', 'lat_mpc_status', source = data_control_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat_mpc_status')
fig2.line('time', 'lat_enable', source = data_control_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'lat_enable')
fig2.line('time', 'lon_enable', source = data_control_command, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'lon_enable')
fig2.line('time', 'planning_type', source = data_control_command, line_width = 2, line_color = 'black', line_dash = 'dashed', legend_label = 'planning_type')
fig2.line('x', 'y', source = data_cursor_fig2, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'cursor')

f3 = fig3.line('time', 'steering_cmd_deg', source = data_control_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steering_cmd_deg')
fig3.line('time', 'steering_deg', source = data_control_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'steering_deg')
fig3.line('time', 'wheel_angle_cmd', source = data_control_command, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'wheel_angle_cmd', visible=False)
fig3.line('time', 'driver_hand_torque', source = data_control_command, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'driver_hand_torque')
fig3.line('time', 'steer_angle_bias_deg', source = data_control_command, line_width = 1, line_color = 'blue', line_dash = 'dashed', legend_label = 'steer_angle_bias_deg', visible=False)
fig3.line('time', 'steer_bias_deg', source = data_control_command, line_width = 1, line_color = 'red', line_dash = 'dashed', legend_label = 'steer_bias_deg', visible=False)
fig3.line('x', 'y', source = data_cursor_fig3, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'cursor')

f4 = fig4.line('time', 'vel_raw_cmd', source = data_control_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'vel_raw_cmd')
fig4.line('time', 'vel_cmd', source = data_control_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'vel_cmd')
fig4.line('time', 'vel_ego', source = data_control_command, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'vel_ego')
fig4.line('x', 'y', source = data_cursor_fig4, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'cursor')


f5 = fig5.line('time_vec', 'dy_ref_mpc_vec', source = data_mpc, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'dy_ref_mpc')
fig5.line('time_vec', 'dy_mpc_vec', source = data_mpc, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'dy_mpc')

f6 = fig6.line('time_vec', 'dphi_ref_mpc_vec', source = data_mpc, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'dphi_ref_mpc')
fig6.line('time_vec', 'dphi_mpc_vec', source = data_mpc, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'dphi_mpc')

f7 = fig7.line('time_vec', 'delta_mpc_vec', source = data_mpc, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'steer_deg_mpc')

f8 = fig8.line('time', 'vel_out', source = data_control_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'vel_out')
fig8.line('time', 'vel_raw_out', source = data_control_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'vel_raw_out')
fig8.line('time', 'vel_ffwd_out', source = data_control_command, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'vel_ffwd_out')
fig8.line('time', 'vel_KP_term', source = data_control_command, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'vel_KP_term')
fig8.line('time', 'vel_KI_term', source = data_control_command, line_width = 1.5, line_color = 'orange', line_dash = 'dashed', legend_label = 'vel_KI_term')
fig8.line('x', 'y', source = data_cursor_fig8, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'cursor')

f9 = fig9.line('time', 'vel_out', source = data_control_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'acc_cmd')
fig9.line('time', 'acc_vel', source = data_control_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'acc_vel')
fig9.line('time', 'acc_ego', source = data_control_command, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'acc_ego')
fig9.line('time', 'slope_acc', source = data_control_command, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'slope_acc')
fig9.line('time', 'throttle_brake', source = data_control_command, line_width = 1, line_color = 'blue', line_dash = 'dashed', legend_label = 'throttle_brake')
fig9.line('x', 'y', source = data_cursor_fig9, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'cursor')

f10 = fig10.line('time', 'yaw_conti', source = data_control_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'yaw_conti')
fig10.line('time', 'euler_angle_yaw', source = data_control_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'euler_angle_yaw')
fig10.line('time', 'euler_angle_pitch', source = data_control_command, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'euler_angle_pitch')

f11 = fig11.line('time', 'lat_err', source = data_control_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'lat_err')
fig11.line('time', 'vel_error', source = data_control_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'vel_error')
fig11.line('time', 'phi_err', source = data_control_command, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'phi_err')
fig11.line('x', 'y', source = data_cursor_fig11, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'cursor')

hover2 = HoverTool(renderers=[f2], tooltips=[('time', '@time'), ('controller_status', '@controller_status'), ('lat_enable', '@lat_enable'), ('lon_enable', '@lon_enable'), ('planning_type', '@planning_type')], mode='vline')
hover3 = HoverTool(renderers=[f3], tooltips=[('time', '@time'), ('steering_cmd_deg', '@steering_cmd_deg'), ('steering_deg', '@steering_deg'),
                                             ('driver_hand_torque', '@driver_hand_torque'), ('steer_angle_bias_deg', '@steer_angle_bias_deg'), ('steer_bias_deg', '@steer_bias_deg')], mode='vline')
hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time'), ('vel_raw_cmd', '@vel_raw_cmd'), ('vel_cmd', '@vel_cmd'), ('vel_ego', '@vel_ego')], mode='vline')
hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time_vec'), ('dy_ref_mpc', '@dy_ref_mpc_vec'), ('dy_mpc', '@dy_mpc_vec')], mode='vline')
hover6 = HoverTool(renderers=[f6], tooltips=[('time', '@time_vec'), ('dphi_ref_mpc', '@dphi_ref_mpc_vec'), ('dphi_mpc', '@dphi_mpc_vec')], mode='vline')
hover7 = HoverTool(renderers=[f7], tooltips=[('time', '@time_vec'), ('delta_mpc', '@delta_mpc_vec')], mode='vline')
hover8 = HoverTool(renderers=[f8], tooltips=[('time', '@time'), ('vel_out', '@vel_out'), ('vel_raw_out', '@vel_raw_out'), ('vel_ffwd_out', '@vel_ffwd_out'), ('vel_KP_term', '@vel_KP_term'), \
    ('vel_KI_term', '@vel_KI_term')], mode='vline')
hover9 = HoverTool(renderers=[f9], tooltips=[('time', '@time'), ('acc_cmd', '@vel_out'), ('acc_vel', '@acc_vel'), ('acc_ego', '@acc_ego'), ('slope_acc', '@slope_acc'), ('throttle_brake', '@throttle_brake')], mode='vline')
hover10 = HoverTool(renderers=[f10], tooltips=[('time', '@time'), ('yaw_conti', '@yaw_conti'), ('yaw', '@euler_angle_yaw'), ('pitch', '@euler_angle_pitch')], mode='vline')
hover11 = HoverTool(renderers=[f11], tooltips=[('time', '@time'), ('lat_err', '@lat_err'), ('vel_error', '@vel_error'),('phi_err','@phi_err')], mode='vline')

fig2.add_tools(hover2)
fig3.add_tools(hover3)
fig4.add_tools(hover4)
fig5.add_tools(hover5)
fig6.add_tools(hover6)
fig7.add_tools(hover7)
fig8.add_tools(hover8)
fig9.add_tools(hover9)
fig10.add_tools(hover10)
fig11.add_tools(hover11)

fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
fig6.toolbar.active_scroll = fig6.select_one(WheelZoomTool)
fig7.toolbar.active_scroll = fig7.select_one(WheelZoomTool)
fig8.toolbar.active_scroll = fig8.select_one(WheelZoomTool)
fig9.toolbar.active_scroll = fig9.select_one(WheelZoomTool)
fig10.toolbar.active_scroll = fig10.select_one(WheelZoomTool)
fig11.toolbar.active_scroll = fig11.select_one(WheelZoomTool)

fig2.legend.click_policy = 'hide'
fig3.legend.click_policy = 'hide'
fig4.legend.click_policy = 'hide'
fig5.legend.click_policy = 'hide'
fig6.legend.click_policy = 'hide'
fig7.legend.click_policy = 'hide'
fig8.legend.click_policy = 'hide'
fig9.legend.click_policy = 'hide'
fig10.legend.click_policy = 'hide'
fig11.legend.click_policy = 'hide'

coord_tf = coord_transformer()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=0.02, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)

### sliders callback
def slider_callback(bag_time):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)

  ctrl_debug_json_msg = local_view_data['data_msg']['ctrl_debug_json_msg']
  dx_ref_mpc_vec = ctrl_debug_json_msg['dx_ref_mpc_vec']
  dy_ref_mpc_vec = ctrl_debug_json_msg['dy_ref_mpc_vec']
  dx_mpc_vec = ctrl_debug_json_msg['dx_mpc_vec']
  dy_mpc_vec = ctrl_debug_json_msg['dy_mpc_vec']
  delta_mpc_vec = ctrl_debug_json_msg['delta_mpc_vec']
  dphi_ref_mpc_vec = ctrl_debug_json_msg['dphi_ref_mpc_vec']
  dphi_mpc_vec = ctrl_debug_json_msg['dphi_mpc_vec']

  loc_msg = local_view_data['data_msg']['loc_msg']
  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = loc_msg.position.position_boot.x
    cur_pos_yn = loc_msg.position.position_boot.y
    cur_yaw = loc_msg.orientation.euler_boot.yaw

  coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)
  if global_var.get_value('g_is_display_enu'):
    dx_ref_mpc_vec, dy_ref_mpc_vec = coord_tf.local_to_global(dx_ref_mpc_vec, dy_ref_mpc_vec)
    dx_mpc_vec, dy_mpc_vec = coord_tf.local_to_global(dx_mpc_vec, dy_mpc_vec)

  t0 = 0
  time_vec = []
  steer_mpc_vec = []
  dphi_deg_ref_mpc_vec = []
  dphi_deg_mpc_vec = []

  for i in range(len(dx_ref_mpc_vec)):
    time_vec.append(t0)
    t0 = t0 + 0.1
    steer_mpc_vec.append(delta_mpc_vec[i] * 15.7 * 57.3)
    dphi_deg_ref_mpc_vec.append(dphi_ref_mpc_vec[i] * 57.3)
    dphi_deg_mpc_vec.append(dphi_mpc_vec[i] * 57.3)


  data_mpc.data.update({
    'time_vec': time_vec,
    'dx_ref_mpc_vec':dx_ref_mpc_vec,
    'dy_ref_mpc_vec':dy_ref_mpc_vec,
    'dx_mpc_vec':dx_mpc_vec,
    'dy_mpc_vec':dy_mpc_vec,
    'dphi_ref_mpc_vec':dphi_deg_ref_mpc_vec,
    'dphi_mpc_vec':dphi_deg_mpc_vec,
    'delta_mpc_vec': steer_mpc_vec,
  })

  data_cursor_fig2.data.update({
    'x':[bag_time, bag_time],
    'y':[-2.5, 3.5],
  })

  data_cursor_fig3.data.update({
    'x':[bag_time, bag_time],
    'y':[-100, 100],
  })

  data_cursor_fig4.data.update({
    'x':[bag_time, bag_time],
    'y':[-2, 30],
  })

  data_cursor_fig8.data.update({
    'x':[bag_time, bag_time],
    'y':[-5.2, 5.2],
  })

  data_cursor_fig9.data.update({
    'x':[bag_time, bag_time],
    'y':[-5.2, 5.2],
  })

  data_cursor_fig11.data.update({
    'x':[bag_time, bag_time],
    'y':[-4, 4],
  })

  push_notebook()

bkp.show(row(fig1, column(fig2, fig3, fig10, fig4, fig8), column(fig5, fig6, fig7, fig9, fig11)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
