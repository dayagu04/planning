import sys, os
sys.path.append("..")
sys.path.append("../lib/")
import inspect
# from lib.load_cyberbag import *
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
from lib.load_lc_st_graph import *
from lib.load_lat_lon_joint_decision import *  # 新增：导入联合规划可视化
sys.path.append('../..')
sys.path.append('../../../')
from bokeh.models import ColumnDataSource, DataTable, DateFormatter, TableColumn
from bokeh.models import TextInput, Panel, Tabs, Div
from bokeh.resources import INLINE
# bag path and frame dt
# bag_path = "/pnc_x86_data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250311/20250311-10-51-40/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-03-11-10-51-40_no_camera.bag"
# bag_path = "/pnc_x86_data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250311/20250311-10-58-23/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-03-11-10-58-23_no_camera.bag"
bag_path = "/home/ros/code/bags/data_collection_BESTUNE_E541_20404_EVENT_MANUAL_2026-03-24-11-12-56_1774321956000.bag.1774592561.close-loop.scc.plan"
frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
global_var.set_value('g_is_display_enu', False)
global_var.set_value('is_vis_sdpromap', True)
fig1, local_view_data = load_local_view_figure()
fig1.legend.label_text_font_size = "8pt"
fig1.height = 1350
fig1.width = 900

fig1, fig2, fig3, data_st, lat_data_vec, ori_lat_data_vec, lc_path_data_vec = load_lc_path_figure(fig1)
agent_box_data_vec = ColumnDataSource(data = {'corners_y':[],
                                               'corners_x':[]})
ego_box_data_vec = ColumnDataSource(data = {'corners_y':[],
                                               'corners_x':[]})
fig1.patches('corners_y', 'corners_x', source = agent_box_data_vec, fill_color = "grey", fill_alpha = 0.1, line_width = 1,  line_color = 'red', line_alpha = 1, legend_label = 'agent')
fig1.patches('corners_y', 'corners_x', source = ego_box_data_vec, fill_color = "green", fill_alpha = 0.1, line_width = 1,  line_color = 'red', line_alpha = 1, legend_label = 'ego')

# 新增：加载联合规划轨迹图
pans, joint_plan_data = load_joint_plan_figure(fig1, bag_loader)

plan_debug_msg_idx = 0
obj_id = 0
### sliders config
class LatBehaviorSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)

# 障碍物的id选择
class ObjText:
  def __init__(self,  obj_callback):
    self.id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_id",min=0.0, max=10000)
    ipywidgets.interact(obj_callback, id = self.id)

behavior_data_1 = ColumnDataSource({
  'name':[],
  'data':[]
})

lc_data_3 = ColumnDataSource({
  'name':[],
  'data':[]
})

columns = [
        TableColumn(field="name", title="name",),
        TableColumn(field="data", title="data"),
    ]
data_behavior_table_1 = DataTable(source=behavior_data_1, columns=columns, width=250, height=900)
data_lc_table_3 = DataTable(source=lc_data_3, columns=columns, width=250, height=900)

def update_data(lat_behavior_common, vo_lat_motion_plan):
  vars = ['fix_lane_virtual_id','target_lane_virtual_id','origin_lane_virtual_id',\
          'lc_request','lc_request_source','turn_light','map_turn_light','lc_turn_light','act_request_source','lc_back_invalid_reason','lc_status',\
            'is_lc_valid','lc_valid_cnt','lc_invalid_obj_id','lc_invalid_reason',\
      'lc_valid_back','lc_back_obj_id','lc_back_cnt','lc_back_invalid_reason',\
        'v_relative_left_lane','is_faster_left_lane','faster_left_lane_cnt','v_relative_right_lane',\
          'is_faster_right_lane','faster_right_lane_cnt','is_forbid_left_alc_car','is_forbid_right_alc_car',\
            'is_side_borrow_bicycle_lane','is_side_borrow_lane','has_origin_lane',\
              'has_target_lane','enable_left_lc','enable_right_lc','lc_back_reason']
  # 'near_car_ids_origin','near_car_ids_target', 'left_alc_car_ids','right_alc_car_ids', ,'avoid_car_ids','avoid_car_allow_max_opposite_offset'
  names  = []
  datas = []
  for name in vars:
    try:
      # print(getattr(vo_lat_behavior_plan,name))
      datas.append(getattr(lat_behavior_common,name))
      names.append(name)
    except:
      pass

  behavior_data_1.data.update({
    'name': names,
    'data': datas,
  })

  push_notebook()

def update_lc_data (noa_info, plan_debug_json):
  # vars_noa = ['distance_to_ramp','distance_to_split','distance_to_merge']
  names  = []
  datas = []
  # for name in vars_noa:
  #   try:
  #     datas.append(getattr(noa_info,name))
  #     names.append(name)
  #   except:
  #     pass
  vars_lc = ['sdpromap_valid_', 'is_in_sdmaproad', 'is_miss_split_point', 'is_find_exc_fp', 'turn_switch_state','lane_change_cmd_','cur_state','lc_map_decision','ramp_direction',
             "first_split_direction","first_merge_direction",'is_ego_on_expressway','current_lane_order_id','current_lane_virtual_id','current_lane_relative_id',
             'left_boundary_type','right_boundary_type',"current_segment_id","distance_to_route_end","sum_dis_to_last_merge_point",
             'road_to_ramp_turn_signal','merge_lane_virtual_id','is_merge_region',"cur_lane_is_continue",
             'is_split_region', 'distance_to_ramp','distance_to_first_road_merge','distance_to_first_road_split',
            'current_segment_passed_distance', 'forward_lane_num', 'is_ego_on_split_region',
            'ego_status_on_route','bd_mlc_scene', 'left_lane_num', 'minVal_seq',
            'maxVal_seq', 'right_lane_num', 'emergency_lane_num', 'average_curve','lsl_length', 'lat_offset_lc_hold',
            'ramp_pass_sts', 'target_lane_congestion_level', 'lat_offset_propose','is_emergency_scene',
            'front_agent_id', 'front_other_id', 'rear_agent_id', 'side_id','merging_rear_id', 'is_aggressive_scence', 'is_default_aggressive_scence']

  for name in vars_lc:
    try:
      datas.append((plan_debug_json[name]))
      names.append(name)
    except:
      pass
  lc_data_3.data.update({
    'name': names,
    'data': datas,
  })
  push_notebook()

# 创建日志输出区域（需要在 slider_callback 之前定义）
log_output = ipywidgets.Output(layout=ipywidgets.Layout(width='100%', height='200px', border='1px solid gray'))

def _fill_or_empty(vec, length):
  """辅助：vec 非空则原样返回，否则返回 length 个 0"""
  return vec if len(vec) > 0 else [0] * length

def _update_vel_range(fig, ego_vel_vec):
  """动态调整速度图纵坐标范围（自车速度 ± 15 kph）"""
  if len(ego_vel_vec) > 0:
    avg = sum(ego_vel_vec) / len(ego_vel_vec)
    margin = 15.0 / 3.6
    fig.y_range.start = max(0, avg - margin)
    fig.y_range.end = avg + margin

def update_safety_check_data(plan_debug_json_msg):
  """更新前车和后车的安全检查数据"""
  try:
    # ---- 自车速度（前后车共用） ----
    ego_vel = plan_debug_json_msg.get('ego_vel_vec', [])

    # ---- 后车数据 ----
    rear_buff = plan_debug_json_msg.get('rear_box_longitudinal_buff_vec', [])
    rear_ttc = plan_debug_json_msg.get('rear_box_ttc_vec', [])
    rear_dist = plan_debug_json_msg.get('rear_distance_vec', [])
    rear_agent_vel = plan_debug_json_msg.get('rear_agent_vel_vec', [])
    rear_gap = plan_debug_json_msg.get('rear_actual_gap_vec', [])

    rear_len = max(len(rear_buff), len(rear_ttc), len(rear_dist),
                   len(rear_agent_vel), len(rear_gap))
    if rear_len > 0:
      idx = list(range(rear_len))
      rear_safety_data.data = {
          'index': idx,
          'box_longitudinal_buff': _fill_or_empty(rear_buff, rear_len),
          'box_ttc': _fill_or_empty(rear_ttc, rear_len),
          'distance': _fill_or_empty(rear_dist, rear_len),
          'agent_vel': _fill_or_empty(rear_agent_vel, rear_len),
          'ego_vel': _fill_or_empty(ego_vel, rear_len),
          'actual_gap': _fill_or_empty(rear_gap, rear_len),
      }
      _update_vel_range(fig_rear_vel, ego_vel)
      print(f"后车安全数据: {rear_len} 点, buff={rear_buff[:3] if rear_buff else '[]'}")
    else:
      print("后车安全检查数据为空")

    # ---- 前车数据 ----
    front_buff = plan_debug_json_msg.get('front_box_longitudinal_buff_vec', [])
    front_dist = plan_debug_json_msg.get('front_distance_vec', [])
    front_agent_vel = plan_debug_json_msg.get('front_agent_vel_vec', [])
    front_gap = plan_debug_json_msg.get('front_actual_gap_vec', [])

    front_len = max(len(front_buff), len(front_dist),
                    len(front_agent_vel), len(front_gap))
    if front_len > 0:
      idx = list(range(front_len))
      front_safety_data.data = {
          'index': idx,
          'box_longitudinal_buff': _fill_or_empty(front_buff, front_len),
          'distance': _fill_or_empty(front_dist, front_len),
          'agent_vel': _fill_or_empty(front_agent_vel, front_len),
          'ego_vel': _fill_or_empty(ego_vel, front_len),
          'actual_gap': _fill_or_empty(front_gap, front_len),
      }
      _update_vel_range(fig_front_vel, ego_vel)
      print(f"前车安全数据: {front_len} 点, buff={front_buff[:3] if front_buff else '[]'}")
    else:
      print("前车安全检查数据为空")

    # ---- 压线率 ----
    lc_ego_press = plan_debug_json_msg.get('lc_ego_press_line_ratio', None)
    if lc_ego_press is not None:
      press_line_div.text = f"<h3>压线比例: <span style='color:blue;'>{lc_ego_press:.3f}</span></h3>"
    else:
      press_line_div.text = f"<h3>压线比例: <span style='color:gray;'>N/A</span></h3>"

  except Exception as e:
    print(f"更新安全检查数据失败: {e}")
    import traceback
    traceback.print_exc()

def slider_callback(bag_time):
  global plan_debug_msg_idx

  # 将所有 print 输出重定向到日志区域
  with log_output:
    local_view_data_ = update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
    update_lc_path_figure (data_st, lat_data_vec, ori_lat_data_vec, lc_path_data_vec, bag_loader, bag_time, local_view_data,
                           ego_box_data_vec, agent_box_data_vec)

    # 新增：更新联合规划轨迹数据
    update_joint_plan_data(bag_loader, bag_time, local_view_data, joint_plan_data)

    if bag_loader.plan_debug_msg['enable'] == True and bag_loader.planning_hmi_msg['enable'] == True:
      plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
      plan_debug_json_msg = local_view_data['data_msg']['plan_debug_json_msg']
      planning_hmi_msg = local_view_data['data_msg']['planning_hmi_msg']
      vo_lat_motion_plan = plan_debug_msg.vo_lat_motion_plan
      lat_behavior_common = plan_debug_msg.lat_behavior_common

      noa_info = planning_hmi_msg.ad_info
      try:
        update_data(lat_behavior_common, vo_lat_motion_plan)
      except:
        pass
      update_lc_data(noa_info, plan_debug_json_msg)

      # 更新安全检查数据
      update_safety_check_data(plan_debug_json_msg)

  push_notebook()

# +

# 创建可切换的右侧面板
# Tab 1: Longtime（联合规划图表）
tab_longtime = Panel(child=pans, title="Longtime")

# ======== 安全检查 Tab ========
rear_safety_data = ColumnDataSource(data={
    'index': [], 'box_longitudinal_buff': [], 'box_ttc': [],
    'distance': [], 'agent_vel': [], 'ego_vel': [], 'actual_gap': []
})
front_safety_data = ColumnDataSource(data={
    'index': [], 'box_longitudinal_buff': [],
    'distance': [], 'agent_vel': [], 'ego_vel': [], 'actual_gap': []
})

# 图1: 距离和缓冲区
fig_safety_dist = bkp.figure(x_axis_label='Prediction Step', y_axis_label='Distance (m)',
                             x_range=[0, 16], y_range=[-5, 30],
                             width=525, height=330, title="距离与缓冲区")
# 图1 仅后车
fig_safety_dist.line('index', 'distance', source=rear_safety_data,
                     line_width=2, line_color='red', legend_label='rear_box_dist')
fig_safety_dist.circle('index', 'distance', source=rear_safety_data, size=4, color='red')
fig_safety_dist.line('index', 'actual_gap', source=rear_safety_data,
                     line_width=2, line_color='brown', line_dash='dashed', legend_label='rear_actual_gap')
fig_safety_dist.circle('index', 'actual_gap', source=rear_safety_data, size=4, color='brown')
fig_safety_dist.line('index', 'box_longitudinal_buff', source=rear_safety_data,
                     line_width=2, line_color='blue', line_dash='dotted', legend_label='rear_lon_buff')
fig_safety_dist.circle('index', 'box_longitudinal_buff', source=rear_safety_data, size=4, color='blue')
fig_safety_dist.legend.click_policy = 'hide'

# 图2: 后车 TTC + 前车距离缓冲区
fig_safety_ttc = bkp.figure(x_axis_label='Prediction Step', y_axis_label='Value',
                            x_range=fig_safety_dist.x_range, y_range=[-5, 30],
                            width=525, height=330, title="后车TTC / 前车距离")
# 后车 TTC
fig_safety_ttc.line('index', 'box_ttc', source=rear_safety_data,
                    line_width=2, line_color='green', legend_label='rear_ttc')
fig_safety_ttc.circle('index', 'box_ttc', source=rear_safety_data, size=4, color='green')
# 前车距离缓冲区
fig_safety_ttc.line('index', 'distance', source=front_safety_data,
                    line_width=2, line_color='red', legend_label='front_box_dist')
fig_safety_ttc.circle('index', 'distance', source=front_safety_data, size=4, color='red')
fig_safety_ttc.line('index', 'actual_gap', source=front_safety_data,
                    line_width=2, line_color='brown', line_dash='dashed', legend_label='front_actual_gap')
fig_safety_ttc.circle('index', 'actual_gap', source=front_safety_data, size=4, color='brown')
fig_safety_ttc.line('index', 'box_longitudinal_buff', source=front_safety_data,
                    line_width=2, line_color='orange', line_dash='dotted', legend_label='front_lon_buff')
fig_safety_ttc.circle('index', 'box_longitudinal_buff', source=front_safety_data, size=4, color='orange')
fig_safety_ttc.legend.click_policy = 'hide'

# 图3: 速度对比（自车紫色，后车蓝色，前车橙色）
fig_safety_vel = bkp.figure(x_axis_label='Prediction Step', y_axis_label='Velocity (m/s)',
                            x_range=fig_safety_dist.x_range,
                            width=525, height=330, title="速度对比")
fig_safety_vel.line('index', 'ego_vel', source=rear_safety_data,
                    line_width=2, line_color='purple', line_dash='dashed', legend_label='ego_vel')
fig_safety_vel.circle('index', 'ego_vel', source=rear_safety_data, size=4, color='purple')
fig_safety_vel.line('index', 'agent_vel', source=rear_safety_data,
                    line_width=2, line_color='blue', legend_label='rear_vel')
fig_safety_vel.circle('index', 'agent_vel', source=rear_safety_data, size=4, color='blue')
fig_safety_vel.line('index', 'agent_vel', source=front_safety_data,
                    line_width=2, line_color='orange', legend_label='front_vel')
fig_safety_vel.circle('index', 'agent_vel', source=front_safety_data, size=4, color='orange')
fig_safety_vel.legend.click_policy = 'hide'

fig_rear_vel = fig_safety_vel
fig_front_vel = fig_safety_vel

# 压线比例
press_line_div = Div(text="<h3>压线比例: <span style='color:blue;'>N/A</span></h3>",
                     width=525, height=100)

safety_check_layout = column(row(fig_safety_dist, fig_safety_ttc),
                             row(fig_safety_vel, press_line_div))
tab_safety_check = Panel(child=safety_check_layout, title="安全检查")

# 创建可切换的 Tabs
right_tabs = Tabs(tabs=[tab_longtime, tab_safety_check])

# 先创建并显示时间滑块（在图表上方）
slider_class = LatBehaviorSlider(slider_callback)

# 然后显示所有图表：主视图 + 表格 + 可切换的右侧面板
bkp.show(row(fig1, row(data_behavior_table_1, data_lc_table_3), right_tabs), notebook_handle=True)

# 最后显示日志输出区域（在最下方）
display(log_output)

# slider_class = ObjText(obj_id_handler)
