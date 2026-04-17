import sys, os
import warnings
import logging

warnings.filterwarnings('ignore', category=UserWarning, module='bokeh')
logging.getLogger('bokeh').setLevel(logging.ERROR)

sys.path.append("..")
sys.path.append("../lib/")
from load_local_view import *
from load_lon_st_graph import *
from lib.load_ros_bag import LoadRosbag
from lib.load_lat_plan import *
from bokeh.models import DataTable, TableColumn, Panel, Tabs, HoverTool, WheelZoomTool, ColumnDataSource
from bokeh.resources import INLINE
import bokeh.plotting as bkp
import numpy as np
sys.path.append('../..')
sys.path.append('../../../')

# ============================================================
# 配置区域 - 按需修改
# ============================================================
# mode: 'lat' 仅横向分析, 'lon' 仅纵向分析, 'both' 同时分析
mode = 'both'

bag_path = "/data_cold/abu_zone/autoparse/bestune_e541_36718/trigger/20260416/20260416-20-52-16/data_collection_BESTUNE_E541_36718_EVENT_KEY_2026-04-16-20-52-16_no_camera.bag"
bag_path = "/data_cold/abu_zone/autoparse/bestune_e541_36718/trigger/20260416/20260416-20-53-11/data_collection_BESTUNE_E541_36718_EVENT_KEY_2026-04-16-20-53-11_no_camera.bag"

frame_dt = 0.05  # sec
global_fig_plot = True

# ============================================================
# 初始化
# ============================================================
display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()

car_type = global_var.get_value('car_type')
scene_type = global_var.get_value('scene_type')

fig1, local_view_data = load_local_view_figure()
load_measure_distance_tool(fig1)

# ============================================================
# 横向规划 (Lateral)
# ============================================================
if mode in ('lat', 'both'):
  steer_ratio = load_steer_ratio(car_type)
  fig1, fig2, fig3, fig4, fig5, fig6, fig7, fig8, fig9, lat_plan_data = load_lat_plan_figure(fig1, local_view_data)
  if scene_type == 'HPP':
    fig1.width = 1200
    fig1.height = 1300
  load_measure_distance_tool(fig7)
  fig_lat_offset = load_lateral_offset(bag_loader)
  fig_receive_topic_time = load_receive_topic_time(bag_loader)
  hmi_info_data, ad_info_table, hpp_info_table, nsa_info_table, rads_info_table = load_planning_hmi_info_table()
  planning_request_data, planning_request_table = load_planning_request_table()
  fig_hmi = load_avoid_hmi(bag_loader)
  fig_hmi = load_enter_fuction_hmi(fig_hmi, bag_loader)
  fig_curve = load_road_curve(bag_loader, lat_plan_data)

  behavior_data_common = ColumnDataSource({'name': [], 'data': []})
  behavior_data_dynamic_lane_change = ColumnDataSource({'name': [], 'data': []})
  behavior_table_columns = [TableColumn(field="name", title="name"), TableColumn(field="data", title="data")]
  behavior_table_common = DataTable(source=behavior_data_common, columns=behavior_table_columns, width=400, height=1000)
  behavior_table_dynamic_lane_change = DataTable(source=behavior_data_dynamic_lane_change, columns=behavior_table_columns, width=400, height=1000)

  def get_plan_debug_msg_idx(bag_loader, bag_time):
    plan_debug_msg_idx = 0
    if bag_loader.plan_debug_msg['enable'] == True:
      while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx += 1
    return plan_debug_msg_idx

  def get_vs_msg_idx(bag_loader, bag_time):
    vs_msg_idx = 0
    if bag_loader.vs_msg['enable'] == True:
      while bag_loader.vs_msg['t'][vs_msg_idx] <= bag_time and vs_msg_idx < (len(bag_loader.vs_msg['t'])-2):
        vs_msg_idx += 1
    return vs_msg_idx

  fig_steer_angle = bkp.figure(x_axis_label='time', y_axis_label='steer deg', width=800, height=160)
  fig_steer_angle_rate = bkp.figure(x_axis_label='time', y_axis_label='steer dot deg', width=800, height=160)
  steer_data = ColumnDataSource(data={'time': [], 'plan_steer_deg': [], 'plan_steer_dot_deg': [], 'ego_steer_deg': [], 'ego_steer_dot_deg': []})
  steer_time, plan_steer_deg, plan_steer_dot_deg, ego_steer_deg, ego_steer_dot_deg = [], [], [], [], []
  for t in np.arange(0.0, max_time, frame_dt):
    steer_time.append(t)
    try:
      pdm_idx = get_plan_debug_msg_idx(bag_loader, t)
      lmpo = bag_loader.plan_debug_msg['data'][pdm_idx].lateral_motion_planning_output
      plan_steer_deg.append(lmpo.delta_vec[0] * steer_ratio * 57.3)
      plan_steer_dot_deg.append(lmpo.omega_vec[0] * steer_ratio * 57.3)
    except:
      plan_steer_deg.append(0.0)
      plan_steer_dot_deg.append(0.0)
    try:
      vs_idx = get_vs_msg_idx(bag_loader, t)
      vs_msg = bag_loader.vs_msg['data'][vs_idx]
      ego_steer_deg.append(vs_msg.steering_wheel_angle * 57.3)
      ego_steer_dot_deg.append(vs_msg.steering_wheel_angle_speed * 57.3)
    except:
      ego_steer_deg.append(0.0)
      ego_steer_dot_deg.append(0.0)
  steer_data.data.update({'time': steer_time, 'plan_steer_deg': plan_steer_deg,
    'plan_steer_dot_deg': plan_steer_dot_deg, 'ego_steer_deg': ego_steer_deg, 'ego_steer_dot_deg': ego_steer_dot_deg})
  line_plan_steer       = fig_steer_angle.line('time', 'plan_steer_deg',    source=steer_data, line_width=1, line_color='red',   line_dash='solid', legend_label='plan_steer_deg')
  fig_steer_angle.line('time', 'ego_steer_deg',                              source=steer_data, line_width=1, line_color='green', line_dash='solid', legend_label='ego_steer_deg')
  line_plan_steer_rate  = fig_steer_angle_rate.line('time', 'plan_steer_dot_deg', source=steer_data, line_width=1, line_color='red',   line_dash='solid', legend_label='plan_steer_dot_deg')
  fig_steer_angle_rate.line('time', 'ego_steer_dot_deg',                     source=steer_data, line_width=1, line_color='green', line_dash='solid', legend_label='ego_steer_dot_deg')
  fig_steer_angle.add_tools(HoverTool(renderers=[line_plan_steer],      tooltips=[('time','@time'),('plan_steer_deg','@plan_steer_deg'),('ego_steer_deg','@ego_steer_deg')], mode='vline'))
  fig_steer_angle_rate.add_tools(HoverTool(renderers=[line_plan_steer_rate], tooltips=[('time','@time'),('plan_steer_dot_deg','@plan_steer_dot_deg'),('ego_steer_dot_deg','@ego_steer_dot_deg')], mode='vline'))
  fig_steer_angle.toolbar.active_scroll = fig_steer_angle.select_one(WheelZoomTool)
  fig_steer_angle_rate.toolbar.active_scroll = fig_steer_angle_rate.select_one(WheelZoomTool)
  fig_steer_angle.legend.click_policy = 'hide'
  fig_steer_angle_rate.legend.click_policy = 'hide'

# ============================================================
# 纵向规划 (Longitudinal)
# ============================================================
if mode in ('lon', 'both'):
  velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status, topic_latency_fig = load_lon_global_figure(bag_loader)
  lon_pans, lon_plan_data = load_lon_plan_figure(fig1, velocity_fig, acc_fig, lead_fig, cost_time_fig, cutin_fig, obs_st_ids, fig_fsm_state, fig_replan_status, topic_latency_fig)

# ============================================================
# 更新函数
# ============================================================
def update_dynamic_agent_emergency_lane_change_behavior_data(local_view_data):
  if mode not in ('lat', 'both'):
    return
  planning_json = local_view_data['data_msg']['plan_debug_json_msg']
  vars = ['recommend_dynamic_agent_emergency_avoidance_direction', 'risk_level',
          'dynamic_agent_emergency_situation_timetstamp', 'dynamic_agent_emergency_lane_change_direction',
          'brake_failure_obstacle_id', 'is_brake_failure_detected', 'brake_failure_situation_timestamp']
  names, datas = [], []
  for name in vars:
    try:
      datas.append(planning_json[name])
      names.append(name)
    except:
      pass
  behavior_data_dynamic_lane_change.data.update({'name': names, 'data': datas})

def update_lat_behavior_data(local_view_data):
  if mode not in ('lat', 'both'):
    return
  lat_behavior_common = local_view_data['data_msg']['plan_debug_msg'].lat_behavior_common
  planning_json = local_view_data['data_msg']['plan_debug_json_msg']
  vars = ['fix_lane_virtual_id','target_lane_virtual_id','origin_lane_virtual_id',
          'lc_request','lc_request_source','turn_light','map_turn_light','lc_turn_light','act_request_source','lc_back_invalid_reason','lc_status',
          'is_lc_valid','lc_valid_cnt','lc_invalid_obj_id','lc_invalid_reason',
          'lc_valid_back','lc_back_obj_id','lc_back_cnt','lc_back_invalid_reason',
          'v_relative_left_lane','is_faster_left_lane','faster_left_lane_cnt','v_relative_right_lane',
          'is_faster_right_lane','faster_right_lane_cnt','is_forbid_left_alc_car','is_forbid_right_alc_car',
          'is_side_borrow_bicycle_lane','is_side_borrow_lane','has_origin_lane',
          'has_target_lane','enable_left_lc','enable_right_lc','lc_back_reason',
          'emergency_avoid_obstacle_ids', 'lon_overtake_avoid', 'potential_dangerous_agent_id','pre_follow_within_lane_ids',
          'lane_borrow_decider_status', 'static_blocked_obj_id_vec']
  names, datas = [], []
  for name in vars:
    try:
      datas.append(getattr(lat_behavior_common, name))
      names.append(name)
    except:
      pass
  behavior_data_common.data.update({'name': names, 'data': datas})

# ============================================================
# Slider 和回调
# ============================================================
class LocalViewSlider:
  def __init__(self, slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description="bag_time", min=0.1, max=max_time, value=0.1, step=frame_dt)
    self.prediction_obstacle_id = ipywidgets.Text(description='predict_id:')
    self.obstacle_polygon_id = ipywidgets.Text(description='polygon_id:')
    ipywidgets.interact(slider_callback, bag_time=self.time_slider,
                        prediction_obstacle_id=self.prediction_obstacle_id,
                        obstacle_polygon_id=self.obstacle_polygon_id)

def slider_callback(bag_time, prediction_obstacle_id, obstacle_polygon_id):
  update_select_obstacle_id(prediction_obstacle_id, obstacle_polygon_id, local_view_data)
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)

  if mode in ('lat', 'both'):
    update_lat_plan_data(fig7, bag_loader, bag_time, local_view_data, lat_plan_data)
    if bag_loader.plan_debug_msg['enable'] == True:
      update_lat_behavior_data(local_view_data)
      update_dynamic_agent_emergency_lane_change_behavior_data(local_view_data)
    update_planning_hmi_info_data(bag_loader, local_view_data, hmi_info_data)
    update_planning_request_data(bag_loader, local_view_data, planning_request_data)

  if mode in ('lon', 'both'):
    update_lon_plan_data(bag_loader, bag_time, local_view_data, lon_plan_data)

  push_notebook()

# ============================================================
# 显示布局
# ============================================================
if mode == 'lat':
  pan_curve    = Panel(child=row(column(fig2, fig8, fig9, fig3, fig4, fig5, fig6, fig_steer_angle, fig_steer_angle_rate, fig_curve)), title="CurveFigure")
  pan_behavior = Panel(child=row(column(fig_hmi, fig_lat_offset, row(row(behavior_table_common, behavior_table_dynamic_lane_change)))), title="BehaviorInfo")
  pan_hmi      = Panel(child=row(column(column(fig_receive_topic_time, row(ad_info_table, column(hpp_info_table, nsa_info_table, rads_info_table, planning_request_table))))), title="Hmi")
  pan_lat_path = Panel(child=row(column(fig7)), title="!Figure")
  if scene_type == "HPP":
    pans = Tabs(tabs=[pan_curve, pan_behavior, pan_hmi, pan_lat_path], height=1200)
  else:
    pans = Tabs(tabs=[pan_curve, pan_behavior, pan_hmi, pan_lat_path])
  if global_fig_plot:
    bkp.show(row(fig1, pans), notebook_handle=True)
  else:
    bkp.show(row(fig1, column(fig2, fig9, fig3, fig4, fig5, fig6, fig_lat_offset)), notebook_handle=True)

if mode == 'lon':
  bkp.show(row(fig1, lon_pans), notebook_handle=True)

if mode == 'both':
  pan_curve    = Panel(child=row(column(fig2, fig8, fig9, fig3, fig4, fig5, fig6, fig_steer_angle, fig_steer_angle_rate, fig_curve)), title="Lat_CurveFigure")
  pan_behavior = Panel(child=row(column(fig_hmi, fig_lat_offset, row(row(behavior_table_common, behavior_table_dynamic_lane_change)))), title="Lat_BehaviorInfo")
  pan_hmi      = Panel(child=row(column(column(fig_receive_topic_time, row(ad_info_table, column(hpp_info_table, nsa_info_table, rads_info_table, planning_request_table))))), title="Lat_Hmi")
  pan_lat_path = Panel(child=row(column(fig7)), title="Lat_!Figure")
  combined_tabs = Tabs(tabs=[pan_curve, pan_behavior, pan_hmi, pan_lat_path] + lon_pans.tabs, height=1200 if scene_type == "HPP" else 800)
  bkp.show(row(fig1, combined_tabs), notebook_handle=True)

slider_class = LocalViewSlider(slider_callback)
display(slider_class.prediction_obstacle_id, slider_class.obstacle_polygon_id, slider_class.time_slider)
