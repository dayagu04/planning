import sys, os
sys.path.append("..")
sys.path.append("../lib/")
from lib.load_local_view import *
from lib.load_spatio_temporal_union_plan import *
from lib.load_ros_bag import LoadRosbag
from bokeh.resources import INLINE

sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')
from python_proto import spatio_temporal_union_dp_input_pb2
from python_proto import basic_types_pb2
from jupyter_pybind import spatio_temporal_union_planning_py

# +
# bag path and frame dt
# bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250624/20250624-14-34-41/data_collection_CHERY_E0Y_04228_EVENT_FILTER_2025-06-24-14-34-41_no_camera.bag.1751002868.close-loop.scc.plan"
bag_path = "/data_cold/abu_zone/autoparse/chery_m32t_50818/trigger/20250717/20250717-11-54-17/data_collection_CHERY_M32T_50818_EVENT_FILTER_2025-07-17-11-54-17_no_camera.bag.1752740613.open-loop.scc.plan"

# bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_18047/trigger/20250120/20250120-16-05-39/data_collection_CHERY_E0Y_18047_EVENT_MANUAL_2025-01-20-16-05-39_no_camera.bag.22-35.split.1742284603.close-loop.noa.plan"
frame_dt = 0.1 # sec
steer_ratio = 13.0 # e0y
# -

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

# +
bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()

fig1, local_view_data = load_local_view_figure()
fig1.height = 1550
# init pybind
spatio_temporal_union_planning_py.Init()
# -

spatio_temporal_union_plan_input = bag_loader.plan_debug_msg['data'][-1].spatio_temporal_union_plan_input

# load spatio_temporal_union planning
fig1, spatio_temporal_union_plan_data = load_spatio_temporal_union_plan_figure(fig1)

fig_sl_data = load_spatio_temporal_union_sl_data(bag_loader, spatio_temporal_union_plan_data, local_view_data)

fig_st_data = load_spatio_temporal_union_st_data(bag_loader, spatio_temporal_union_plan_data, local_view_data)

# fig_vt_data = load_spatio_temporal_union_vt_data(bag_loader, spatio_temporal_union_plan_data, local_view_data)

coord_tf = coord_transformer()

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.unit_t_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "unit_t",min=0.1, max=1.0, value=spatio_temporal_union_plan_input.dp_search_paramms.unit_t, step=0.1)
    self.dense_unit_s_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "dense_unit_s",min=0.1, max=0.8, value=spatio_temporal_union_plan_input.dp_search_paramms.dense_unit_s, step=0.1)
    self.sparse_unit_s_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "sparse_unit_s",min=1.0, max=5.0, value=spatio_temporal_union_plan_input.dp_search_paramms.sparse_unit_s, step=1.0)
    self.unit_l_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "unit_l",min=0.1, max=0.5, value=spatio_temporal_union_plan_input.dp_search_paramms.unit_l, step=0.05)
    self.dense_dimension_s_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "dense_dimension_s",min=21.0, max=501.0, value=spatio_temporal_union_plan_input.dp_search_paramms.dense_dimension_s, step=10.0)
    self.dimension_l_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "dimension_l",min=2.0, max=51.0, value=spatio_temporal_union_plan_input.dp_search_paramms.dimension_l, step=1.0)
    self.max_acceleration_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "max_acceleration",min=0.0, max=2.0, value=spatio_temporal_union_plan_input.dp_search_paramms.max_acceleration, step=0.1)
    self.max_deceleration_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "max_deceleration",min=-4.0, max=0.0, value=spatio_temporal_union_plan_input.dp_search_paramms.max_deceleration, step=0.1)

    self.path_l_cost_param_l0_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_l_cost_param_l0",min=0.0, max=5.0, value=spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost_param_l0, step=0.2)
    self.path_l_cost_param_b_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_l_cost_param_b",min=0.0, max=5.0, value=spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost_param_b, step=0.2)
    self.path_l_cost_param_k_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_l_cost_param_k",min=0.0, max=5.0, value=spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost_param_k, step=0.2)
    self.path_l_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_l_cost",min=0.0, max=1e5, value=spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost, step=100)
    self.path_dl_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_dl_cost",min=0.0, max=1e5, value=spatio_temporal_union_plan_input.lat_path_weight_params.path_dl_cost, step=10)
    self.path_ddl_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_ddl_cost",min=0.0, max=1e5, value=spatio_temporal_union_plan_input.lat_path_weight_params.path_ddl_cost, step=10)
    self.path_end_l_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_end_l_cost",min=0.0, max=1e3, value=spatio_temporal_union_plan_input.lat_path_weight_params.path_end_l_cost, step=10)
    self.path_l_stitching_cost_param_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_l_stitching_cost_param",min=0.0, max=1e5, value=spatio_temporal_union_plan_input.stitching_cost_params.path_l_stitching_cost_param, step=10)
    self.stitching_cost_time_decay_factor_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "stitching_cost_time_decay_factor",min=0.0, max=1.0, value=spatio_temporal_union_plan_input.stitching_cost_params.stitching_cost_time_decay_factor, step=0.1)

    self.obstacle_ignore_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obstacle_ignore_distance",min=0.0, max=50.0, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_ignore_distance, step=0.5)
    self.obstacle_collision_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obstacle_collision_cost",min=1e3, max=1.0e9, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_collision_cost, step=1e3)
    self.obstacle_longit_collision_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obstacle_longit_collision_distance",min=0.0, max=2.0, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_longit_collision_distance, step=0.1)
    self.obstacle_longit_risk_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obstacle_longit_risk_distance",min=0.0, max=10.0, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_longit_risk_distance, step=1.0)
    self.dynamic_obstacle_weight_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "dynamic_obstacle_weight",min=1e-4, max=1e-1, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.dynamic_obstacle_weight, step=1e-4)
    self.default_obstacle_cost_weight_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "default_obstacle_cost_weight",min=0.01, max=1.0, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.default_obstacle_cost_weight, step=0.01)
    self.obstacle_lateral_risk_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obstacle_lateral_risk_distance",min=0.0, max=2.0, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_lateral_risk_distance, step=0.1)
    self.obstacle_collision_cost_without_lateral_overlap_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obstacle_collision_cost_without_lateral_overlap",min=1e2, max=1e8, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_collision_cost_without_lateral_overlap, step=100)
    self.obstacle_lateral_collision_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obstacle_lateral_collision_distance",min=0.0, max=2.0, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_lateral_collision_distance, step=0.1)

    self.spatial_potential_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "spatial_potential_penalty",min=1e1, max=1e2, value=spatio_temporal_union_plan_input.long_weight_params.spatial_potential_penalty, step=1e1)
    self.keep_clear_low_speed_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "keep_clear_low_speed_penalty",min=0.0, max=1.0e2, value=spatio_temporal_union_plan_input.long_weight_params.keep_clear_low_speed_penalty, step=1.0)
    self.default_speed_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "default_speed_cost",min=0.0, max=1.0e3, value=spatio_temporal_union_plan_input.long_weight_params.default_speed_cost, step=1.0)
    self.exceed_speed_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "exceed_speed_penalty",min=0.0, max=1.0e3, value=spatio_temporal_union_plan_input.long_weight_params.exceed_speed_penalty, step=1.0)
    self.low_speed_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "low_speed_penalty",min=0.0, max=1.0e2, value=spatio_temporal_union_plan_input.long_weight_params.low_speed_penalty, step=1.0)
    self.reference_speed_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "reference_speed_penalty",min=0.0, max=1.0e2, value=spatio_temporal_union_plan_input.long_weight_params.reference_speed_penalty, step=1.0)
    self.accel_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "accel_penalty",min=0.0, max=1e2, value=spatio_temporal_union_plan_input.long_weight_params.accel_penalty, step=1.0)
    self.decel_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "decel_penalty",min=0.0, max=1e2, value=spatio_temporal_union_plan_input.long_weight_params.decel_penalty, step=1.0)
    self.positive_jerk_coeff_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "positive_jerk_coeff",min=0.0, max=1.0e2, value=spatio_temporal_union_plan_input.long_weight_params.positive_jerk_coeff, step=1.0)
    self.negative_jerk_coeff_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "negative_jerk_coeff",min=0.0, max=1.0e2, value=spatio_temporal_union_plan_input.long_weight_params.negative_jerk_coeff, step=1.0)
    # ipywidgets.IntSlider(layout=ipywidgets.Layout(width='50%'), description= "max_iter",min=0, max=10, value=10, step=1)

    self.bag_dt_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "bag_dt",min=-10.0, max=10.0, value=0.1, step=0.1)

    ipywidgets.interact(slider_callback, bag_time = self.time_slider,
                                         unit_t = self.unit_t_slider,
                                         dense_unit_s = self.dense_unit_s_slider,
                                         sparse_unit_s = self.sparse_unit_s_slider,
                                         unit_l = self.unit_l_slider,
                                         dense_dimension_s = self.dense_dimension_s_slider,
                                         dimension_l = self.dimension_l_slider,
                                         max_acceleration = self.max_acceleration_slider,
                                         max_deceleration = self.max_deceleration_slider,
                                         path_l_cost_param_l0 = self.path_l_cost_param_l0_slider,
                                         path_l_cost_param_b = self.path_l_cost_param_b_slider,
                                         path_l_cost_param_k = self.path_l_cost_param_k_slider,
                                         path_l_cost = self.path_l_cost_slider,
                                         path_dl_cost = self.path_dl_cost_slider,
                                         path_ddl_cost = self.path_ddl_cost_slider,
                                         path_end_l_cost = self.path_end_l_cost_slider,
                                         path_l_stitching_cost_param = self.path_l_stitching_cost_param_slider,
                                         stitching_cost_time_decay_factor = self.stitching_cost_time_decay_factor_slider,
                                         obstacle_ignore_distance = self.obstacle_ignore_distance_slider,
                                         obstacle_collision_cost = self.obstacle_collision_cost_slider,
                                         obstacle_longit_collision_distance = self.obstacle_longit_collision_distance_slider,
                                         obstacle_longit_risk_distance = self.obstacle_longit_risk_distance_slider,
                                         dynamic_obstacle_weight = self.dynamic_obstacle_weight_slider,
                                         default_obstacle_cost_weight = self.default_obstacle_cost_weight_slider,
                                         obstacle_lateral_risk_distance = self.obstacle_lateral_risk_distance_slider,
                                         obstacle_lateral_collision_distance = self.obstacle_lateral_collision_distance_slider,
                                         obstacle_collision_cost_without_lateral_overlap = self.obstacle_collision_cost_without_lateral_overlap_slider,
                                         spatial_potential_penalty = self.spatial_potential_penalty_slider,
                                         keep_clear_low_speed_penalty = self.keep_clear_low_speed_penalty_slider,
                                         default_speed_cost = self.default_speed_cost_slider,
                                         exceed_speed_penalty = self.exceed_speed_penalty_slider,
                                         low_speed_penalty = self.low_speed_penalty_slider,
                                         reference_speed_penalty = self.reference_speed_penalty_slider,
                                         accel_penalty = self.accel_penalty_slider,
                                         decel_penalty = self.decel_penalty_slider,
                                         positive_jerk_coeff = self.positive_jerk_coeff_slider,
                                         negative_jerk_coeff = self.negative_jerk_coeff_slider)


### sliders callback
def slider_callback(bag_time, unit_t, dense_unit_s, sparse_unit_s, unit_l, dense_dimension_s,
                    dimension_l, max_acceleration, max_deceleration,
                   path_l_cost_param_l0, path_l_cost_param_b, path_l_cost_param_k,
                     path_l_cost, path_dl_cost, path_ddl_cost, path_end_l_cost,
                   path_l_stitching_cost_param, stitching_cost_time_decay_factor,
                   obstacle_ignore_distance, obstacle_collision_cost, obstacle_longit_collision_distance,
                   obstacle_longit_risk_distance, dynamic_obstacle_weight, default_obstacle_cost_weight,
                   obstacle_lateral_risk_distance, obstacle_lateral_collision_distance, obstacle_collision_cost_without_lateral_overlap,
                   spatial_potential_penalty, keep_clear_low_speed_penalty, default_speed_cost,
                   exceed_speed_penalty, low_speed_penalty, reference_speed_penalty,
                   accel_penalty, decel_penalty, positive_jerk_coeff, negative_jerk_coeff):
  g_is_display_enu = global_var.get_value('g_is_display_enu')
  kwargs = locals()
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_spatio_temporal_union_plan_data(bag_loader, bag_time, local_view_data, spatio_temporal_union_plan_data)

  loc_msg = local_view_data['data_msg']['loc_msg']
  plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
  spatio_temporal_union_plan_input = plan_debug_msg.spatio_temporal_union_plan_input
  # print("spatio_temporal_union_plan_input: ", spatio_temporal_union_plan_input)
  if len(spatio_temporal_union_plan_input.ref_points_vec) > 0:
    # print("spatio_temporal_union_plan_input.ref_points_vec x:", spatio_temporal_union_plan_input.ref_points_vec[0].x)
    # print("spatio_temporal_union_plan_input.ref_points_vec y:", spatio_temporal_union_plan_input.ref_points_vec[0].y)
    print("spatio_temporal_union_plan_input.init_state s:", spatio_temporal_union_plan_input.init_state.s)
    print("spatio_temporal_union_plan_input.init_state l:", spatio_temporal_union_plan_input.init_state.l)

    agents_time_corners = spatio_temporal_union_plan_input.agent_time_corners
    for agent_time_and_corner in agents_time_corners:
      print("agent_id:", agent_time_and_corner.agent_id)
      for max_box_corner in agent_time_and_corner.max_box_corners:
        print("max_box_corners: ", max_box_corner)
      for time_and_corners in agent_time_and_corner.time_and_corners:
        print("time:", time_and_corners.time)
        for agent_corner in time_and_corners.agent_corner:
          print("corners:", agent_corner)

    input_string = spatio_temporal_union_plan_input.SerializeToString()
    start_time = time.time()
    spatio_temporal_union_planning_py.UpdateByParams(input_string, unit_t, dense_unit_s, sparse_unit_s, unit_l,
                                                    dense_dimension_s, dimension_l, max_acceleration, max_deceleration, path_l_cost_param_l0, path_l_cost_param_b, path_l_cost_param_k,
                                                    path_l_cost, path_dl_cost, path_ddl_cost, path_end_l_cost, path_l_stitching_cost_param, stitching_cost_time_decay_factor,
                                                    obstacle_ignore_distance, obstacle_collision_cost, obstacle_longit_collision_distance, obstacle_longit_risk_distance, dynamic_obstacle_weight, default_obstacle_cost_weight, obstacle_lateral_risk_distance, obstacle_lateral_collision_distance, obstacle_collision_cost_without_lateral_overlap,
                                                    spatial_potential_penalty, keep_clear_low_speed_penalty, default_speed_cost, exceed_speed_penalty, low_speed_penalty,
                                                    reference_speed_penalty, accel_penalty, decel_penalty, positive_jerk_coeff, negative_jerk_coeff)

    end_time = time.time()
    planning_output = basic_types_pb2.TrajectoryPoints()
    output_string_tmp = spatio_temporal_union_planning_py.GetOutputBytes()
    planning_output.ParseFromString(output_string_tmp)


  print("\n ------------------------------------------- \n")

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
  print("cur_pos_xn:", cur_pos_xn)
  print("cur_pos_yn:", cur_pos_yn)
  print("cur_yaw:", cur_yaw)

  traj_x, traj_y, time_vec = [], [], []
  # if g_is_display_enu:
  #   for trajectory_point in planning_output.point:
  #     traj_x.append([trajectory_point.x])
  #     traj_y.append([trajectory_point.y])
  #     time_vec.append(trajectory_point.t)
  # else:
  #   for trajectory_point in planning_output.point:
  #     traj_x_local, traj_y_local = coord_tf.global_to_local([trajectory_point.x], [trajectory_point.y])
  #     traj_x.append(traj_x_local)
  #     traj_y.append(traj_y_local)
  #     time_vec.append(trajectory_point.t)

  try:
    for trajectory_point in planning_output.point:
      traj_x.append(trajectory_point.x)
      traj_y.append(trajectory_point.y)
      time_vec.append(trajectory_point.t)
    if not g_is_display_enu:
      traj_x_local, traj_y_local = coord_tf.global_to_local(traj_x, traj_y)
      traj_x = traj_x_local
      traj_y = traj_y_local
    print("traj_x size: ", len(traj_x))
  except:
    print("planning_output.point.size() = 0 !")

  spatio_temporal_union_plan_data['data_spatio_temporal_trajs_back'].data.update({
      'traj_y': traj_y,
      'traj_x': traj_x,
      'time_vec': time_vec,
  })

  # print("traj_x:", traj_x)
  # print("traj_y:", traj_y[0])
  # print("time:", time_vec[0])
  # print("planning_output.point.x:", planning_output.point[0].x)
  # print("planning_output.point.y:", planning_output.point[0].y)
  push_notebook()


bkp.show(row(fig1,column(fig_sl_data, fig_st_data)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)

