import sys, os
sys.path.append("..")
from lib.load_local_view import *
from lib.load_tune_lat_mp import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('python_proto')
from python_proto import common_pb2, lateral_motion_planner_pb2
from jupyter_pybind import lateral_motion_planning_py

# bag path and frame dt
bag_path = "/share//data_cold/abu_zone/hpp/1219bag/memory1219_12.00000"
bag_path = "/share//data_cold/abu_zone/hpp/1219bag/memory1219_12.00000"
frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

# init pybind
lateral_motion_planning_py.Init()

lat_motion_plan_input0 = bag_loader.plan_debug_msg['data'][-1].lateral_motion_planning_input

# load lateral planning (behavior and motion)
fig1, fig2, fig3, fig4, fig5, fig6, fig7, tab1, lat_plan_data = load_lat_plan_figure(fig1)

coord_tf = coord_transformer()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.q_ref_xy_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_ref_xy",min=0.1, max=200.0, value=lat_motion_plan_input0.q_ref_x, step=0.1)
    self.q_ref_theta_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_ref_theta",min=0.1, max=200.0, value=lat_motion_plan_input0.q_ref_theta, step=0.1)
    self.q_acc_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_acc",min=0.1, max=10.0, value=lat_motion_plan_input0.q_acc, step=0.01)
    self.q_jerk_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_jerk",min=0.01, max=10.0, value=lat_motion_plan_input0.q_jerk, step=0.01)

    self.q_acc_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_acc_bound",min=0.0, max=1000.0, value=lat_motion_plan_input0.q_acc_bound, step=0.1)
    self.q_jerk_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_jerk_bound",min=0.0, max=1000.0, value=lat_motion_plan_input0.q_jerk_bound, step=0.1)
    self.acc_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "acc_bound",min=0.0, max=10.0, value=lat_motion_plan_input0.acc_bound, step=0.1)
    self.jerk_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "jerk_bound",min=0.0, max=10.0, value=lat_motion_plan_input0.jerk_bound, step=0.1)
    self.q_safe_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_safe_bound",min=0.0, max=10000.0, value=lat_motion_plan_input0.q_soft_corridor, step=0.1)
    self.q_hard_bound_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_hard_bound",min=0.0, max=10000.0, value=lat_motion_plan_input0.q_hard_corridor, step=0.1)
    self.upper_safe_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "upper_safe_bound",min=-10., max=10.0, value=0.0, step=0.1)
    self.lower_safe_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "lower_safe_bound",min=-10., max=10.0, value=0.0, step=0.1)

    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                         q_ref_xy = self.q_ref_xy_slider,
                                         q_ref_theta = self.q_ref_theta_slider,
                                         q_acc = self.q_acc_slider,
                                         q_jerk = self.q_jerk_slider,
                                         q_acc_bound = self.q_acc_bound_slider,
                                         q_jerk_bound = self.q_jerk_bound_slider,
                                         acc_bound = self.acc_bound_slider,
                                         jerk_bound = self.jerk_bound_slider,
                                         q_safe_bound = self.q_safe_bound_slider,
                                         q_hard_bound = self.q_hard_bound_slider,
                                         upper_safe_bound = self.upper_safe_bound,
                                         lower_safe_bound = self.lower_safe_bound
                                         )


### sliders callback
def slider_callback(bag_time, q_ref_xy, q_ref_theta, q_acc, q_jerk, q_acc_bound, q_jerk_bound, acc_bound, jerk_bound, q_safe_bound,q_hard_bound, upper_safe_bound, lower_safe_bound):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_tune_lat_plan_data(bag_loader, bag_time, local_view_data, lat_plan_data,upper_safe_bound, lower_safe_bound)

  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']

  lat_motion_plan_input = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input

  input_string = lat_motion_plan_input.SerializeToString()
  lateral_motion_planning_py.UpdateByParams(input_string, q_ref_xy, q_ref_theta, q_acc, q_jerk, q_acc_bound, q_jerk_bound, acc_bound, jerk_bound, q_safe_bound, q_hard_bound, upper_safe_bound, lower_safe_bound)

  planning_output = lateral_motion_planner_pb2.LateralPlanningOutput()
  output_string_tmp = lateral_motion_planning_py.GetOutputBytes()
  planning_output.ParseFromString(output_string_tmp)



  if bag_loader.loc_msg['enable'] == True:
    cur_pos_xn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.x
    cur_pos_yn = bag_loader.loc_msg['data'][loc_msg_idx].position.position_boot.y
    cur_yaw = bag_loader.loc_msg['data'][loc_msg_idx].orientation.euler_boot.yaw
    planning_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]

  try:
    json_pos_x = planning_json['ego_pos_x']
    json_pos_y = planning_json['ego_pos_y']
    json_yaw = planning_json['ego_pos_yaw']
    coord_tf.set_info( json_pos_x, json_pos_y, json_yaw)
  except:
    coord_tf.set_info( cur_pos_xn, cur_pos_yn, cur_yaw)


  x_vec, y_vec = coord_tf.global_to_local(planning_output.x_vec, planning_output.y_vec)
  time_vec = planning_output.time_vec

  ref_theta_deg_vec = []
  theta_deg_vec = []
  steer_deg_vec = []
  steer_dot_deg_vec =[]

  for i in range(len(time_vec)):
    ref_theta_deg_vec.append(lat_motion_plan_input.ref_theta_vec[i] * 57.3)
    theta_deg_vec.append(planning_output.theta_vec[i] * 57.3)
    steer_deg_vec.append(planning_output.delta_vec[i] * 57.3 * 15.7)
    steer_dot_deg_vec.append(planning_output.omega_vec[i] * 57.3 * 15.7)

  acc_vec = planning_output.acc_vec
  jerk_vec = planning_output.jerk_vec

  # comb_x_vec = []
  # comb_y_vec = []

  # lat_err_tab = [0.0, theta1, theta2, 100.0]
  # alpha_tab = [1.0, 1.0, 0.0, 0.0]

  # f = interp1d(lat_err_tab, alpha_tab)

  # lat_err = abs(ref_y_vec[0])
  # alpha = f(lat_err)

  # for i in range(len(ref_x_vec)):
  #   comb_x_vec.append(x_vec[i] * (1.0 - alpha) + alpha * ref_x_vec[i])
  #   comb_y_vec.append(y_vec[i] * (1.0 - alpha) + alpha * ref_y_vec[i])
    
  lat_plan_data['data_lat_motion_plan_output'].data.update({
    'time_vec_t': time_vec,
    'x_vec_t': x_vec,
    'y_vec_t': y_vec,
    'xn_vec_t': planning_output.x_vec,
    'yn_vec_t': planning_output.y_vec,
    'ref_theta_deg_vec_t': ref_theta_deg_vec,
    'theta_deg_vec_t': theta_deg_vec,
    'steer_deg_vec_t': steer_deg_vec,
    'steer_dot_deg_vec_t': steer_dot_deg_vec,
    'acc_vec_t': acc_vec,
    'jerk_vec_t': jerk_vec,
    # 'comb_x_vec': comb_x_vec,
    # 'comb_y_vec': comb_y_vec,
  })

  push_notebook()

bkp.show(row(fig1, column(fig7, tab1), column(fig2, fig3, fig4, fig5, fig6)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
