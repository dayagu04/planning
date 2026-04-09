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
bag_path = "/share/data/clren/code/planning8/data_collection_CHERY_E0Y_04228_EVENT_FILTER_2025-06-21-10-18-12.bag_1750472282000_1750472297000_no_camera_9d5895e02c392aa16833026421de46234675a22c.bag.1775617069.open-loop.scc.plan"

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

# 默认隐藏 ctrl_traj 和 lane_mark_point
for item in fig1.legend[0].items:
    label = item.label.get('value', '')
    if label in ('ctrl_traj', 'lane_mark_point','prediction', 'lat plan', 'plan', 'plan_point', 'lane_mark', 'traffic_light', 'traffic_light_type'):
        for r in item.renderers:
            r.visible = False
# init pybind
spatio_temporal_union_planning_py.Init()
# -

spatio_temporal_union_plan_input = bag_loader.plan_debug_msg['data'][-1].spatio_temporal_union_plan_input
for  i, plan_debug_msg in enumerate(bag_loader.plan_debug_msg['data']):
  if plan_debug_msg.spatio_temporal_union_plan_input.dp_search_paramms.unit_t > 0:
    spatio_temporal_union_plan_input = plan_debug_msg.spatio_temporal_union_plan_input

# load spatio_temporal_union planning
fig1, spatio_temporal_union_plan_data = load_spatio_temporal_union_plan_figure(fig1)

fig_sl_data = load_spatio_temporal_union_sl_data(bag_loader, spatio_temporal_union_plan_data, local_view_data)

fig_st_data = load_spatio_temporal_union_st_data(bag_loader, spatio_temporal_union_plan_data, local_view_data)

# fig_vt_data = load_spatio_temporal_union_vt_data(bag_loader, spatio_temporal_union_plan_data, local_view_data)

coord_tf = coord_transformer()
load_measure_distance_tool(fig1)

# Initialize debug_source before it's used in callbacks
from bokeh.models import ColumnDataSource
debug_source = ColumnDataSource(data={'s': [], 'total_cost': [], 'obstacle_cost': [], 'path_cost': [], 'long_cost': []})
sample_source = ColumnDataSource(data={'x': [], 'y': [], 's': [], 'l': [], 't': [], 'total_cost': [], 't_index': [], 's_index': [], 'l_index': []})
debug_path_source = ColumnDataSource(data={'x': [], 'y': [], 's': [], 'l': [], 't': [], 'total_cost': [], 'obstacle_cost': [], 'path_cost': [], 'path_l_cost': [], 'path_dl_cost': [], 'path_ddl_cost': [], 'stitching_cost': [], 'long_cost': [], 'pre_t_index': [], 'pre_s_index': [], 'pre_l_index': [], 't_index': [], 's_index': [], 'l_index': []})

last_frame_path_source = ColumnDataSource(data={'x': [], 'y': []})
all_nodes_source = ColumnDataSource(data={
    'x': [], 'y': [], 's': [], 'l': [], 't': [],
    'total_cost': [], 'obstacle_cost': [], 'path_cost': [],
    'path_l_cost': [], 'path_dl_cost': [], 'path_ddl_cost': [],
    'stitching_cost': [], 'long_cost': [], 'speed': [], 'acc': [],
    't_index': [], 's_index': [], 'l_index': [],
    'pre_t_index': [], 'pre_s_index': [], 'pre_l_index': [],
    'is_optimal': [], 'node_color': [], 'node_size': [],
})

# Global variable to store last frame trajectory in global coordinates
debug_x, debug_y = [], []  # Initialize for cross-block access

### sliders config
class LocalViewSlider:
  def __init__(self):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
        width='75%'), description="bag_time", min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.unit_t_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="unit_t", min=0.1, max=1.0,
                                                value=spatio_temporal_union_plan_input.dp_search_paramms.unit_t, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    self.dense_unit_s_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "dense_unit_s",min=0.1, max=0.8, value=spatio_temporal_union_plan_input.dp_search_paramms.dense_unit_s, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    self.sparse_unit_s_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="sparse_unit_s", min=1.0, max=5.0,
                                                      value=spatio_temporal_union_plan_input.dp_search_paramms.sparse_unit_s, step=1.0, style=widgets.SliderStyle(description_width='initial'))
    self.unit_l_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="unit_l", min=0.1, max=0.5,
                                                value=spatio_temporal_union_plan_input.dp_search_paramms.unit_l, step=0.05, style=widgets.SliderStyle(description_width='initial'))
    self.dense_dimension_s_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="dense_dimension_s", min=21.0, max=501.0,
                                                          value=spatio_temporal_union_plan_input.dp_search_paramms.dense_dimension_s, step=10.0, style=widgets.SliderStyle(description_width='initial'))
    self.dimension_l_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="dimension_l", min=2.0, max=51.0,
                                                    value=spatio_temporal_union_plan_input.dp_search_paramms.dimension_l, step=1.0, style=widgets.SliderStyle(description_width='initial'))
    self.max_acceleration_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "max_acceleration",min=0.0, max=2.0, value=spatio_temporal_union_plan_input.dp_search_paramms.max_acceleration, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    self.max_deceleration_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="max_deceleration", min=-4.0, max=0.0,
                                                          value=spatio_temporal_union_plan_input.dp_search_paramms.max_deceleration, step=0.1, style=widgets.SliderStyle(description_width='initial'))

    self.path_l_cost_param_l0_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="path_l_cost_param_l0", min=0.0, max=5.0,
                                                              value=spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost_param_l0, step=0.2, style=widgets.SliderStyle(description_width='initial'))
    self.path_l_cost_param_b_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_l_cost_param_b",min=0.0, max=5.0, value=spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost_param_b, step=0.2, style=widgets.SliderStyle(description_width='initial'))
    self.path_l_cost_param_k_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="path_l_cost_param_k", min=0.0, max=5.0,
                                                            value=spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost_param_k, step=0.2, style=widgets.SliderStyle(description_width='initial'))
    self.path_l_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_l_cost",min=0.0, max=1e5, value=spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost, step=100, style=widgets.SliderStyle(description_width='initial'))
    self.path_dl_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "path_dl_cost",min=0.0, max=1e6, value=spatio_temporal_union_plan_input.lat_path_weight_params.path_dl_cost, step=10, style=widgets.SliderStyle(description_width='initial'))
    self.path_ddl_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="path_ddl_cost", min=0.0, max=1e7,
                                                      value=spatio_temporal_union_plan_input.lat_path_weight_params.path_ddl_cost, step=10, style=widgets.SliderStyle(description_width='initial'))
    self.path_end_l_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="path_end_l_cost", min=0.0, max=1e3,
                                                        value=spatio_temporal_union_plan_input.lat_path_weight_params.path_end_l_cost, step=10, style=widgets.SliderStyle(description_width='initial'))
    self.path_l_stitching_cost_param_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="path_l_stitching_cost_param", min=0.0, max=1e5,
                                                                    value=spatio_temporal_union_plan_input.stitching_cost_params.path_l_stitching_cost_param, step=10, style=widgets.SliderStyle(description_width='initial'))
    self.stitching_cost_time_decay_factor_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="stitching_cost_time_decay_factor", min=0.0,
                                                                          max=1.0, value=spatio_temporal_union_plan_input.stitching_cost_params.stitching_cost_time_decay_factor, step=0.1, style=widgets.SliderStyle(description_width='initial'))

    self.obstacle_ignore_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obstacle_ignore_distance",min=0.0, max=50.0, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_ignore_distance, step=0.5, style=widgets.SliderStyle(description_width='initial'))
    self.obstacle_collision_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="obstacle_collision_cost", min=0, max=1.0e9,
                                                                value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_collision_cost, step=1e3, style=widgets.SliderStyle(description_width='initial'))
    self.obstacle_longit_collision_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="obstacle_longit_collision_distance", min=0.0, max=2.0,
                                                                            value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_longit_collision_distance, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    self.obstacle_longit_risk_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="obstacle_longit_risk_distance", min=0.0, max=10.0,
                                                                      value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_longit_risk_distance, step=1.0, style=widgets.SliderStyle(description_width='initial'))
    self.dynamic_obstacle_weight_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="dynamic_obstacle_weight", min=1e-4, max=1e-1,
                                                                value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.dynamic_obstacle_weight, step=1e-4, style=widgets.SliderStyle(description_width='initial'))
    self.default_obstacle_cost_weight_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "default_obstacle_cost_weight",min=0.01, max=1.0, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.default_obstacle_cost_weight, step=0.01, style=widgets.SliderStyle(description_width='initial'))
    self.obstacle_lateral_risk_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="obstacle_lateral_risk_distance", min=0.0, max=2.0,
                                                                        value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_lateral_risk_distance, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    self.obstacle_collision_cost_without_lateral_overlap_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "obstacle_collision_cost_without_lateral_overlap",min=1e2, max=1e8, value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_collision_cost_without_lateral_overlap, step=100)
    self.obstacle_lateral_collision_distance_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="obstacle_lateral_collision_distance", min=0.0, max=2.0,
                                                                            value=spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_lateral_collision_distance, step=0.1, style=widgets.SliderStyle(description_width='initial'))

    self.spatial_potential_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="spatial_potential_penalty", min=1e1, max=2e5,
                                                                  value=spatio_temporal_union_plan_input.long_weight_params.spatial_potential_penalty, step=1e1, style=widgets.SliderStyle(description_width='initial'))
    self.keep_clear_low_speed_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="keep_clear_low_speed_penalty", min=0.0, max=1.0e2,
                                                                      value=spatio_temporal_union_plan_input.long_weight_params.keep_clear_low_speed_penalty, step=1.0, style=widgets.SliderStyle(description_width='initial'))
    self.default_speed_cost_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="default_speed_cost", min=0.0, max=1.0e3,
                                                            value=spatio_temporal_union_plan_input.long_weight_params.default_speed_cost, step=1.0, style=widgets.SliderStyle(description_width='initial'))
    self.exceed_speed_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="exceed_speed_penalty", min=0.0, max=1.0e3,
                                                              value=spatio_temporal_union_plan_input.long_weight_params.exceed_speed_penalty, step=1.0, style=widgets.SliderStyle(description_width='initial'))
    self.low_speed_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="low_speed_penalty", min=0.0, max=1.0e2,
                                                          value=spatio_temporal_union_plan_input.long_weight_params.low_speed_penalty, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    self.reference_speed_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "reference_speed_penalty",min=0.0, max=1.0e2, value=spatio_temporal_union_plan_input.long_weight_params.reference_speed_penalty, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    self.accel_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "accel_penalty",min=0.0, max=1e2, value=spatio_temporal_union_plan_input.long_weight_params.accel_penalty, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    self.decel_penalty_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "decel_penalty",min=0.0, max=1e2, value=spatio_temporal_union_plan_input.long_weight_params.decel_penalty, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    self.positive_jerk_coeff_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "positive_jerk_coeff",min=0.0, max=1.0e2, value=spatio_temporal_union_plan_input.long_weight_params.positive_jerk_coeff, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    self.negative_jerk_coeff_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "negative_jerk_coeff",min=0.0, max=1.0e2, value=spatio_temporal_union_plan_input.long_weight_params.negative_jerk_coeff, step=0.1, style=widgets.SliderStyle(description_width='initial'))
    # ipywidgets.IntSlider(layout=ipywidgets.Layout(width='50%'), description= "max_iter",min=0, max=10, value=10, step=1)
    self.bag_dt_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "bag_dt",min=-10.0, max=10.0, value=0.1, step=0.1)
    self.use_origin_param = ipywidgets.Checkbox(
        value=False, description='use_origin_param')

    ipywidgets.interact(self.slider_callback, bag_time = self.time_slider,
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
                                         negative_jerk_coeff = self.negative_jerk_coeff_slider,
                                         use_origin_param=self.use_origin_param)


  def slider_callback(self, bag_time, unit_t, dense_unit_s, sparse_unit_s, unit_l, dense_dimension_s,
                      dimension_l, max_acceleration, max_deceleration,
                    path_l_cost_param_l0, path_l_cost_param_b, path_l_cost_param_k,
                      path_l_cost, path_dl_cost, path_ddl_cost, path_end_l_cost,
                    path_l_stitching_cost_param, stitching_cost_time_decay_factor,
                    obstacle_ignore_distance, obstacle_collision_cost, obstacle_longit_collision_distance,
                    obstacle_longit_risk_distance, dynamic_obstacle_weight, default_obstacle_cost_weight,
                    obstacle_lateral_risk_distance, obstacle_lateral_collision_distance, obstacle_collision_cost_without_lateral_overlap,
                    spatial_potential_penalty, keep_clear_low_speed_penalty, default_speed_cost,
                    exceed_speed_penalty, low_speed_penalty, reference_speed_penalty,
                      accel_penalty, decel_penalty, positive_jerk_coeff, negative_jerk_coeff, use_origin_param):
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
      # print("spatio_temporal_union_plan_input.init_state s:", spatio_temporal_union_plan_input.init_state.s)
      # print("spatio_temporal_union_plan_input.init_state l:", spatio_temporal_union_plan_input.init_state.l)

      agents_time_corners = spatio_temporal_union_plan_input.agent_time_corners
      # for agent_time_and_corner in agents_time_corners:
      #   print("agent_id:", agent_time_and_corner.agent_id)
      #   for max_box_corner in agent_time_and_corner.max_box_corners:
      #     print("max_box_corners: ", max_box_corner)
      #   for time_and_corners in agent_time_and_corner.time_and_corners:
      #     print("time:", time_and_corners.time)
      #     for agent_corner in time_and_corners.agent_corner:
      #       print("corners:", agent_corner)

      input_string = spatio_temporal_union_plan_input.SerializeToString()
      start_time = time.time()

      # Set last frame trajectory from bag data (previous frame before current bag_time)
      # so stitching cost uses the correct previous frame, not the previous tuning call output
      prev_plan_debug_msg = None
      for i, msg in enumerate(bag_loader.plan_debug_msg['data']):
        if bag_loader.plan_debug_msg['t'][i] >= bag_time:
          if i > 0:
            prev_plan_debug_msg = bag_loader.plan_debug_msg['data'][i - 1]
          break
      if prev_plan_debug_msg is not None:
        prev_traj = prev_plan_debug_msg.spatio_temporal_union_plan.trajectory_points
        if len(prev_traj) > 0:
          prev_traj_proto = basic_types_pb2.TrajectoryPoints()
          for pt in prev_traj:
            new_pt = prev_traj_proto.point.add()
            new_pt.x = pt.x
            new_pt.y = pt.y
            new_pt.s = pt.s
            new_pt.l = pt.l
            new_pt.t = pt.t
            new_pt.v = pt.v
          spatio_temporal_union_planning_py.SetLastFrameTrajectory(prev_traj_proto.SerializeToString())

      if use_origin_param:
        path_l_cost_param_l0 = spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost_param_l0
        path_l_cost_param_b = spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost_param_b
        path_l_cost_param_k = spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost_param_k
        path_l_cost = spatio_temporal_union_plan_input.lat_path_weight_params.path_l_cost
        path_dl_cost = spatio_temporal_union_plan_input.lat_path_weight_params.path_dl_cost
        path_ddl_cost = spatio_temporal_union_plan_input.lat_path_weight_params.path_ddl_cost
        path_end_l_cost = spatio_temporal_union_plan_input.lat_path_weight_params.path_end_l_cost
        path_l_stitching_cost_param = spatio_temporal_union_plan_input.stitching_cost_params.path_l_stitching_cost_param
        stitching_cost_time_decay_factor = spatio_temporal_union_plan_input.stitching_cost_params.stitching_cost_time_decay_factor

        obstacle_ignore_distance = spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_ignore_distance
        obstacle_collision_cost = spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_collision_cost
        obstacle_longit_collision_distance = spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_longit_collision_distance
        obstacle_longit_risk_distance = spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_longit_risk_distance
        dynamic_obstacle_weight = spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.dynamic_obstacle_weight
        default_obstacle_cost_weight = spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.default_obstacle_cost_weight
        obstacle_lateral_risk_distance = spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_lateral_risk_distance
        obstacle_collision_cost_without_lateral_overlap = spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_collision_cost_without_lateral_overlap
        obstacle_lateral_collision_distance = spatio_temporal_union_plan_input.dp_dynamic_agent_weight_params.obstacle_lateral_collision_distance

        spatial_potential_penalty = spatio_temporal_union_plan_input.long_weight_params.spatial_potential_penalty
        keep_clear_low_speed_penalty = spatio_temporal_union_plan_input.long_weight_params.keep_clear_low_speed_penalty
        default_speed_cost= spatio_temporal_union_plan_input.long_weight_params.default_speed_cost
        exceed_speed_penalty= spatio_temporal_union_plan_input.long_weight_params.exceed_speed_penalty
        low_speed_penalty = spatio_temporal_union_plan_input.long_weight_params.low_speed_penalty
        reference_speed_penalty= spatio_temporal_union_plan_input.long_weight_params.reference_speed_penalty
        accel_penalty= spatio_temporal_union_plan_input.long_weight_params.accel_penalty
        decel_penalty = spatio_temporal_union_plan_input.long_weight_params.decel_penalty
        positive_jerk_coeff= spatio_temporal_union_plan_input.long_weight_params.positive_jerk_coeff
        negative_jerk_coeff = spatio_temporal_union_plan_input.long_weight_params.negative_jerk_coeff

        self.path_l_cost_param_l0_slider.value = path_l_cost_param_l0

        self.path_l_cost_param_b_slider.value = path_l_cost_param_b
        self.path_l_cost_param_k_slider.value = path_l_cost_param_k

        self.path_l_cost_slider.value = path_l_cost
        self.path_dl_cost_slider.value = path_dl_cost
        self.path_ddl_cost_slider.value = path_ddl_cost

        self.path_end_l_cost_slider.value = path_end_l_cost

        self.path_l_stitching_cost_param_slider.value = path_l_stitching_cost_param

        self.stitching_cost_time_decay_factor_slider.value = stitching_cost_time_decay_factor

        self.obstacle_ignore_distance_slider.value = obstacle_ignore_distance
        self.obstacle_collision_cost_slider.value = obstacle_collision_cost

        self.obstacle_longit_collision_distance_slider.value = obstacle_longit_collision_distance

        self.obstacle_longit_risk_distance_slider.value = obstacle_longit_risk_distance

        self.dynamic_obstacle_weight_slider.value = dynamic_obstacle_weight

        self.default_obstacle_cost_weight_slider.value = default_obstacle_cost_weight
        self.obstacle_lateral_risk_distance_slider.value = obstacle_lateral_risk_distance

        self.obstacle_collision_cost_without_lateral_overlap_slider.value = obstacle_collision_cost_without_lateral_overlap
        self.obstacle_lateral_collision_distance_slider.value = obstacle_lateral_collision_distance

        self.spatial_potential_penalty_slider.value = spatial_potential_penalty

        self.keep_clear_low_speed_penalty_slider.value = keep_clear_low_speed_penalty

        self.default_speed_cost_slider.value = default_speed_cost

        self.exceed_speed_penalty_slider.value = exceed_speed_penalty

        self.low_speed_penalty_slider.value = low_speed_penalty

        self.reference_speed_penalty_slider.value = reference_speed_penalty
        self.accel_penalty_slider.value = accel_penalty
        self.decel_penalty_slider.value = decel_penalty
        self.positive_jerk_coeff_slider.value = positive_jerk_coeff
        self.negative_jerk_coeff_slider.value = negative_jerk_coeff

        print('use_origin_param')
      else :
        print('use_new_param')
      # print('spatio_temporal_union_plan_input.init_state',
      #       spatio_temporal_union_plan_input)
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
    # print("cur_pos_xn:", cur_pos_xn)
    # print("cur_pos_yn:", cur_pos_yn)
    # print("cur_yaw:", cur_yaw)

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
      # print("traj_x size: ", len(traj_x))
    except:
      print("planning_output.point.size() = 0 !")

    spatio_temporal_union_plan_data['data_spatio_temporal_trajs_back'].data.update({
        'traj_y': traj_y,
        'traj_x': traj_x,
        'time_vec': time_vec,
    })

    # Update debug points visualization
    try:
      debug_points_bytes = spatio_temporal_union_planning_py.GetDebugPlanBytes()
      from python_proto import spatio_temporal_union_plan_pb2
      from google.protobuf.internal.decoder import _DecodeVarint32

      debug_points = []
      pos = 0
      while pos < len(debug_points_bytes):
        msg_len, new_pos = _DecodeVarint32(debug_points_bytes, pos)
        pos = new_pos
        debug_point = spatio_temporal_union_plan_pb2.DebugPoint()
        debug_point.ParseFromString(debug_points_bytes[pos:pos+msg_len])
        debug_points.append(debug_point)
        pos += msg_len

      if len(debug_points) > 0:
        s_vals = [p.s for p in debug_points]
        l_vals = [p.l for p in debug_points]
        t_vals = [p.t for p in debug_points]
        total_costs = [p.total_cost for p in debug_points]
        obstacle_costs = [p.obstacle_cost for p in debug_points]
        path_costs = [p.path_cost for p in debug_points]
        path_l_costs = [p.path_l_cost for p in debug_points]
        path_dl_costs = [p.path_dl_cost for p in debug_points]
        path_ddl_costs = [p.path_ddl_cost for p in debug_points]
        stitching_costs = [p.stitching_cost for p in debug_points]
        long_costs = [p.long_cost for p in debug_points]
        pre_t_indices = [p.pre_t_index for p in debug_points]
        pre_s_indices = [p.pre_s_index for p in debug_points]
        pre_l_indices = [p.pre_l_index for p in debug_points]
        debug_source.data.update({
          's': s_vals,
          'total_cost': total_costs,
          'obstacle_cost': obstacle_costs,
          'path_cost': path_costs,
          'long_cost': long_costs
        })

        # Get debug path xy coordinates
        debug_path = spatio_temporal_union_planning_py.GetDebugPathXY()
        debug_x = debug_path['x']
        debug_y = debug_path['y']

        if not g_is_display_enu:
          debug_x, debug_y = coord_tf.global_to_local(debug_x, debug_y)

        t_indices_opt = [p.t_index for p in debug_points]
        s_indices_opt = [p.s_index for p in debug_points]
        l_indices_opt = [p.l_index for p in debug_points]

        debug_path_source.data.update({
          'x': debug_x,
          'y': debug_y,
          's': s_vals,
          'l': l_vals,
          't': t_vals,
          'total_cost': total_costs,
          'obstacle_cost': obstacle_costs,
          'path_cost': path_costs,
          'path_l_cost': path_l_costs,
          'path_dl_cost': path_dl_costs,
          'path_ddl_cost': path_ddl_costs,
          'stitching_cost': stitching_costs,
          'long_cost': long_costs,
          'pre_t_index': pre_t_indices,
          'pre_s_index': pre_s_indices,
          'pre_l_index': pre_l_indices,
          't_index': t_indices_opt,
          's_index': s_indices_opt,
          'l_index': l_indices_opt,
        })

    except Exception as e:
      print(f"error 3: {e}")
      import traceback
      traceback.print_exc()
      pass

    # Get sample points from C++
    try:
      sample_points = spatio_temporal_union_planning_py.GetSamplePointsXY()
      sample_x = sample_points['x']
      sample_y = sample_points['y']

      if not g_is_display_enu:
        sample_x, sample_y = coord_tf.global_to_local(sample_x, sample_y)

      sample_source.data.update({
        'x': sample_x, 'y': sample_y,
        's': sample_points['s'], 'l': sample_points['l'], 't': sample_points['t'],
        'total_cost': sample_points['total_cost'],
        't_index': sample_points['t_index'], 's_index': sample_points['s_index'], 'l_index': sample_points['l_index']
      })
    except Exception as e:
      print(f"error getting samples: {e}")

    # Get all cost table nodes (full DP cost landscape)
    try:
      all_nodes = spatio_temporal_union_planning_py.GetAllNodesXY()
      all_x = all_nodes['x']
      all_y = all_nodes['y']

      if not g_is_display_enu:
        all_x, all_y = coord_tf.global_to_local(all_x, all_y)

      # Build optimal path set for color marking (use index for exact match)
      optimal_set = set(zip(debug_path_source.data['t_index'], debug_path_source.data['s_index'], debug_path_source.data['l_index']))
      is_optimal = [(ti, si, li) in optimal_set
                    for ti, si, li in zip(all_nodes['t_index'], all_nodes['s_index'], all_nodes['l_index'])]
      node_color = ['#cc0000' if v else '#2b83ba' for v in is_optimal]
      node_size  = [8 if v else 4 for v in is_optimal]

      all_nodes_source.data.update({
        'x': all_x, 'y': all_y,
        's': all_nodes['s'], 'l': all_nodes['l'], 't': all_nodes['t'],
        'total_cost': all_nodes['total_cost'],
        'obstacle_cost': all_nodes['obstacle_cost'],
        'path_cost': all_nodes['path_cost'],
        'path_l_cost': all_nodes['path_l_cost'],
        'path_dl_cost': all_nodes['path_dl_cost'],
        'path_ddl_cost': all_nodes['path_ddl_cost'],
        'stitching_cost': all_nodes['stitching_cost'],
        'long_cost': all_nodes['long_cost'],
        'speed': all_nodes['speed'],
        'acc': all_nodes['acc'],
        't_index': all_nodes['t_index'],
        's_index': all_nodes['s_index'],
        'l_index': all_nodes['l_index'],
        'pre_t_index': all_nodes['pre_t_index'],
        'pre_s_index': all_nodes['pre_s_index'],
        'pre_l_index': all_nodes['pre_l_index'],
        'is_optimal': is_optimal,
        'node_color': node_color,
        'node_size': node_size,
      })
    except Exception as e:
      print(f"error getting all nodes: {e}")
    # Use bag data directly (same prev_plan_debug_msg already found above)
    try:
      if prev_plan_debug_msg is not None:
        prev_traj = prev_plan_debug_msg.spatio_temporal_union_plan.trajectory_points
        if len(prev_traj) > 0:
          last_x_global = [pt.x for pt in prev_traj]
          last_y_global = [pt.y for pt in prev_traj]
          if g_is_display_enu:
            last_frame_path_source.data.update({'x': last_x_global, 'y': last_y_global})
          else:
            last_x_local, last_y_local = coord_tf.global_to_local(last_x_global, last_y_global)
            last_frame_path_source.data.update({'x': last_x_local, 'y': last_y_local})
    except Exception as e:
      print(f"error transforming last frame: {e}")

    push_notebook()

# Create debug info figure
from bokeh.plotting import figure

fig_debug = figure(width=800, height=400, title="Debug Points - Total Cost vs S", x_axis_label='s (m)', y_axis_label='Total Cost')
fig_debug.line('s', 'total_cost', source=debug_source, legend_label='Total Cost', line_width=2, color='blue')
fig_debug.line('s', 'obstacle_cost', source=debug_source, legend_label='Obstacle Cost', line_width=2, color='red')
fig_debug.line('s', 'path_cost', source=debug_source, legend_label='Path Cost', line_width=2, color='green')
fig_debug.line('s', 'long_cost', source=debug_source, legend_label='Long Cost', line_width=2, color='orange')
fig_debug.legend.click_policy = "hide"

# Add all DP nodes (full cost landscape), optimal path nodes highlighted in red
all_nodes_renderer = fig1.circle('y', 'x', source=all_nodes_source,
                                  size='node_size', color='node_color', alpha=0.7,
                                  legend_label='DP nodes')
hover_all_nodes = HoverTool(renderers=[all_nodes_renderer], tooltips=[
    ('is_optimal', '@is_optimal'),
    ('t_index', '@t_index'),
    ('s_index', '@s_index'),
    ('l_index', '@l_index'),
    ('s', '@s{0.00}'),
    ('l', '@l{0.00}'),
    ('t', '@t{0.00}'),
    ('speed', '@speed{0.00}'),
    ('acc', '@acc{0.000}'),
    ('total_cost', '@total_cost{0.00}'),
    ('obstacle_cost', '@obstacle_cost{0.00}'),
    ('path_cost', '@path_cost{0.00}'),
    ('  path_l_cost', '@path_l_cost{0.00}'),
    ('  path_dl_cost', '@path_dl_cost{0.00}'),
    ('  path_ddl_cost', '@path_ddl_cost{0.00}'),
    ('  stitching_cost', '@stitching_cost{0.00}'),
    ('long_cost', '@long_cost{0.00}'),
    ('pre_t_index', '@pre_t_index'),
    ('pre_s_index', '@pre_s_index'),
    ('pre_l_index', '@pre_l_index'),
])
fig1.add_tools(hover_all_nodes)

# Add sample points to main map
sample_renderer = fig1.circle('y', 'x', source=sample_source, size=5, color='orange', alpha=0.6, legend_label='sample points', visible=False)
hover_sample = HoverTool(renderers=[sample_renderer], tooltips=[
    ('t_index', '@t_index'),
    ('s_index', '@s_index'),
    ('l_index', '@l_index'),
    ('s', '@s{0.00}'),
    ('l', '@l{0.00}'),
    ('t', '@t{0.00}'),
    ('total_cost', '@total_cost{0.00}'),
])
fig1.add_tools(hover_sample)
# Add last frame path
fig1.line('y', 'x', source=last_frame_path_source, line_width=3, color='blue', alpha=0.7, line_dash='dashed', legend_label='last frame path')

bkp.show(row(fig1,column(fig_sl_data, fig_st_data)), notebook_handle=True)
slider_class = LocalViewSlider()
