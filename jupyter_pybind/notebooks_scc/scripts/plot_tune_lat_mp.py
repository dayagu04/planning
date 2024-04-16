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
bag_path = "/share/data_cold/abu_zone/autoparse/jac_s811_35kw2/trigger/20240228/20240228-10-24-47/data_collection_JAC_S811_35KW2_EVENT_MANUAL_2024-02-28-10-24-47_no_camera.record.1710230100.plan"
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
fig1, fig2, fig3, fig4, fig5, fig6, fig7, lat_plan_data = load_lat_plan_figure(fig1)

init_info_name = ["dbw status", "replan status", "lat err", "theta err", "lon err", "dist err", "init theta", "ego theta", "ref theta", "ref vel"]
init_info = ColumnDataSource(data = {'name':[], 'init info':[]})
init_info_columns = [
        TableColumn(field="name", title="name"),
        TableColumn(field="init info", title="init info"),
      ]
tab2 = DataTable(source = init_info, columns = init_info_columns, width = 300, height = 300)

param_name = ["q_ref_xy", "q_ref_theta", "q_ref_theta_real", "q_acc", "q_jerk", "q_acc_bound", "q_jerk_bound", "acc_bound", "jerk_bound", "q_safe_bound", "q_hard_bound", "q_ref_xy_remote", "q_ref_theta_remote", "q_ref_theta_real_remote", "q_safe_bound_remote", "q_hard_bound_remote"]
param = ColumnDataSource(data = {'name':[], 'origin param':[], 'new param':[]})
param_columns = [
        TableColumn(field="name", title="name"),
        TableColumn(field="origin param", title="origin param"),
        TableColumn(field="new param", title="current param"),
      ]
tab3 = DataTable(source = param, columns = param_columns, width = 450, height = 500)

coord_tf = coord_transformer()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.q_ref_xy_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "q_ref_xy",min=0.1, max=1000.0, value=lat_motion_plan_input0.q_ref_x, step=0.1)
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
    self.upper_hard_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "upper_hard_bound",min=-10., max=10.0, value=0.0, step=0.1)
    self.lower_hard_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "lower_hard_bound",min=-10., max=10.0, value=0.0, step=0.1)

    self.complete_follow = ipywidgets.Checkbox(value=lat_motion_plan_input0.complete_follow, description='complete_follow')
    self.motion_plan_concerned_index = ipywidgets.IntText(value=lat_motion_plan_input0.motion_plan_concerned_index, description='motion_plan_concerned_index:')

    self.safe_ub_start_idx = ipywidgets.IntText(value=0, description='safe_ub_start_idx:')
    self.safe_ub_end_idx = ipywidgets.IntText(value=26, description='safe_ub_end_idx:')
    self.safe_lb_start_idx = ipywidgets.IntText(value=0, description='safe_lb_start_idx:')
    self.safe_lb_end_idx = ipywidgets.IntText(value=26, description='safe_lb_end_idx:')
    self.hard_ub_start_idx = ipywidgets.IntText(value=0, description='hard_ub_start_idx:')
    self.hard_ub_end_idx = ipywidgets.IntText(value=26, description='hard_ub_end_idx:')
    self.hard_lb_start_idx = ipywidgets.IntText(value=0, description='hard_lb_start_idx:')
    self.hard_lb_end_idx = ipywidgets.IntText(value=26, description='hard_lb_end_idx:')

    self.use_new_param = ipywidgets.Checkbox(value=False, description='use_new_param')

    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                         use_new_param = self.use_new_param,
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
                                         motion_plan_concerned_index = self.motion_plan_concerned_index
                                         )


### sliders callback
def slider_callback(bag_time, use_new_param, q_ref_xy, q_ref_theta, q_acc, q_jerk, q_acc_bound, q_jerk_bound, acc_bound, jerk_bound, q_safe_bound, q_hard_bound, upper_safe_bound, lower_safe_bound, 
                    upper_hard_bound, lower_hard_bound, safe_ub_start_idx, safe_ub_end_idx, safe_lb_start_idx, safe_lb_end_idx, hard_ub_start_idx, hard_ub_end_idx, hard_lb_start_idx, hard_lb_end_idx, 
                    complete_follow, motion_plan_concerned_index):
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  # update_tune_lat_plan_data(bag_loader, bag_time, local_view_data, lat_plan_data, upper_safe_bound, lower_safe_bound, g_is_display_enu)
  update_tune_lat_plan_data(bag_loader, bag_time, local_view_data, lat_plan_data, upper_safe_bound, lower_safe_bound, upper_hard_bound, lower_hard_bound, safe_ub_start_idx, safe_ub_end_idx, safe_lb_start_idx, safe_lb_end_idx, hard_ub_start_idx, hard_ub_end_idx, hard_lb_start_idx, hard_lb_end_idx, g_is_display_enu)

  plan_debug_msg_idx = local_view_data['data_index']['plan_debug_msg_idx']
  loc_msg_idx = local_view_data['data_index']['loc_msg_idx']

  lat_motion_plan_input = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].lateral_motion_planning_input
  loc_msg = bag_loader.loc_msg['data'][loc_msg_idx]
  planning_json = bag_loader.plan_debug_msg['json'][plan_debug_msg_idx]

  if len(lat_motion_plan_input.ref_theta_vec) > 0:
    ref_vel = lat_motion_plan_input.ref_vel
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
    init_info.data.update({
      'name': init_info_name,
      'init info': init_info_vec,
    })

    origin_param_vec = []
    origin_param_vec.append(lat_motion_plan_input.q_ref_x)
    origin_param_vec.append(lat_motion_plan_input.q_ref_theta)
    origin_param_vec.append(lat_motion_plan_input.q_ref_theta * (1.0 + ref_vel * ref_vel))
    origin_param_vec.append(lat_motion_plan_input.q_acc)
    origin_param_vec.append(lat_motion_plan_input.q_jerk)
    origin_param_vec.append(lat_motion_plan_input.q_acc_bound)
    origin_param_vec.append(lat_motion_plan_input.q_jerk_bound)
    origin_param_vec.append(lat_motion_plan_input.acc_bound)
    origin_param_vec.append(lat_motion_plan_input.jerk_bound)
    origin_param_vec.append(lat_motion_plan_input.q_soft_corridor)
    origin_param_vec.append(lat_motion_plan_input.q_hard_corridor)
    if ((not lat_motion_plan_input.complete_follow) and (lat_motion_plan_input.motion_plan_concerned_index < 25)):
      origin_param_vec.append(0.2 * lat_motion_plan_input.q_ref_x)
      origin_param_vec.append(0.2 * lat_motion_plan_input.q_ref_theta)
      origin_param_vec.append(0.2 * lat_motion_plan_input.q_ref_theta * (1.0 + ref_vel * ref_vel))
      origin_param_vec.append(0.2 * lat_motion_plan_input.q_soft_corridor)
      origin_param_vec.append(0.2 * lat_motion_plan_input.q_hard_corridor)
    else:
      origin_param_vec.append(lat_motion_plan_input.q_ref_x)
      origin_param_vec.append(lat_motion_plan_input.q_ref_theta)
      origin_param_vec.append(lat_motion_plan_input.q_ref_theta * (1.0 + ref_vel * ref_vel))
      origin_param_vec.append(lat_motion_plan_input.q_soft_corridor)
      origin_param_vec.append(lat_motion_plan_input.q_hard_corridor)
    
    if not use_new_param:
      q_ref_xy = lat_motion_plan_input.q_ref_x
      q_ref_theta = lat_motion_plan_input.q_ref_theta
      q_acc = lat_motion_plan_input.q_acc
      q_jerk = lat_motion_plan_input.q_jerk
      q_acc_bound = lat_motion_plan_input.q_acc_bound
      q_jerk_bound = lat_motion_plan_input.q_jerk_bound
      acc_bound = lat_motion_plan_input.acc_bound
      jerk_bound = lat_motion_plan_input.jerk_bound
      q_safe_bound = lat_motion_plan_input.q_soft_corridor
      q_hard_bound = lat_motion_plan_input.q_hard_corridor
      complete_follow = lat_motion_plan_input.complete_follow
      motion_plan_concerned_index = lat_motion_plan_input.motion_plan_concerned_index
    
    new_param_vec = []
    new_param_vec.append(q_ref_xy)
    new_param_vec.append(q_ref_theta)
    new_param_vec.append(q_ref_theta * (1.0 + ref_vel * ref_vel))
    new_param_vec.append(q_acc)
    new_param_vec.append(q_jerk)
    new_param_vec.append(q_acc_bound)
    new_param_vec.append(q_jerk_bound)
    new_param_vec.append(acc_bound)
    new_param_vec.append(jerk_bound)
    new_param_vec.append(q_safe_bound)
    new_param_vec.append(q_hard_bound)
    
    if ((not complete_follow) and (motion_plan_concerned_index < 25)):
      new_param_vec.append(0.2 * q_ref_xy)
      new_param_vec.append(0.2 * q_ref_theta)
      new_param_vec.append(0.2 * q_ref_theta * (1.0 + ref_vel * ref_vel))
      new_param_vec.append(0.2 * q_safe_bound)
      new_param_vec.append(0.2 * q_hard_bound)
    else:
      new_param_vec.append(q_ref_xy)
      new_param_vec.append(q_ref_theta)
      new_param_vec.append(q_ref_theta * (1.0 + ref_vel * ref_vel))
      new_param_vec.append(q_safe_bound)
      new_param_vec.append(q_hard_bound)
    
    param.data.update({
      'name': param_name,
      'origin param': origin_param_vec,
      'new param': new_param_vec,
    })

    input_string = lat_motion_plan_input.SerializeToString()
    start_time = time.time()
    # lateral_motion_planning_py.UpdateByParams(input_string, q_ref_xy, q_ref_theta, q_acc, q_jerk, q_acc_bound, q_jerk_bound, acc_bound, jerk_bound, q_safe_bound, q_hard_bound, upper_safe_bound, lower_safe_bound, 0)
    lateral_motion_planning_py.UpdateByParams(input_string, q_ref_xy, q_ref_theta, q_acc, q_jerk, q_acc_bound, q_jerk_bound, acc_bound, jerk_bound, q_safe_bound, q_hard_bound, upper_safe_bound, lower_safe_bound, upper_hard_bound, lower_hard_bound, 
                                              safe_ub_start_idx, safe_ub_end_idx, safe_lb_start_idx, safe_lb_end_idx, hard_ub_start_idx, hard_ub_end_idx, hard_lb_start_idx, hard_lb_end_idx, complete_follow, motion_plan_concerned_index)
    end_time = time.time()
    planning_output = lateral_motion_planner_pb2.LateralPlanningOutput()
    output_string_tmp = lateral_motion_planning_py.GetOutputBytes()
    planning_output.ParseFromString(output_string_tmp)

    print("\n------------------------------------------\n")

    delta_bound = min(360.0 / 14.5 / 57.3, acc_bound / (lat_motion_plan_input.curv_factor * ref_vel * ref_vel))
    omega_bound = min(240.0 / 14.5 / 57.3, jerk_bound / (lat_motion_plan_input.curv_factor * ref_vel * ref_vel))
    
    print("origin complete_follow : ", lat_motion_plan_input.complete_follow)
    print("origin motion_plan_concerned_index : ", lat_motion_plan_input.motion_plan_concerned_index)
    print("new complete_follow : ", complete_follow)
    print("new motion_plan_concerned_index : ", motion_plan_concerned_index)
    
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
      steer_deg_vec.append(planning_output.delta_vec[i] * 57.3 * 15.7)
      steer_dot_deg_vec.append(planning_output.omega_vec[i] * 57.3 * 15.7)
      acc_upper_bound.append(acc_bound)
      acc_lower_bound.append(-acc_bound)
      jerk_upper_bound.append(jerk_bound)
      jerk_lower_bound.append(-jerk_bound)
      steer_deg_upper_bound.append((delta_bound * 57.3 * 15.7))
      steer_deg_lower_bound.append(-(delta_bound * 57.3 * 15.7))
      steer_dot_deg_upper_bound.append((omega_bound * 57.3 * 15.7))
      steer_dot_deg_lower_bound.append(-(omega_bound * 57.3 * 15.7))

    acc_vec = planning_output.acc_vec
    jerk_vec = planning_output.jerk_vec
      
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

bkp.show(row(fig1, column(fig7, row(tab2, tab3)), column(fig2, fig3, fig4, fig5, fig6)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
