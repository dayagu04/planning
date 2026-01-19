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
bag_path = "/data_cold/abu_zone/autoparse/bestune_e541_20406/trigger/20260124/20260124-10-37-52/data_collection_BESTUNE_E541_20406_EVENT_FUNEXIT_2026-01-24-10-37-52_no_camera.bag.1770123900.open-loop.scc.plan"
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
              'has_target_lane','enable_left_lc','enable_right_lc','lc_back_reason', ]
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
            'ramp_pass_sts', 'target_lane_congestion_level', 'lat_offset_propose',
            'front_agent_id', 'front_other_id', 'rear_agent_id', 'side_id','merging_rear_id','merge_fail','merge_hard']

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

def update_safety_check_data(plan_debug_json_msg):
  """更新安全检查数据"""
  try:
    box_longitudinal_buff_vec = plan_debug_json_msg.get('box_longitudinal_buff_vec', [])
    box_ttc_vec = plan_debug_json_msg.get('box_ttc_vec', [])
    distance_vec = plan_debug_json_msg.get('distance_vec', [])
    agent_vel_vec = plan_debug_json_msg.get('agent_vel_vec', [])
    ego_vel_vec = plan_debug_json_msg.get('ego_vel_vec', [])
    rear_distance_vec = plan_debug_json_msg.get('rear_distance_vec', [])

    # 调试输出
    print(f"安全检查数据长度: buff={len(box_longitudinal_buff_vec)}, ttc={len(box_ttc_vec)}, dist={len(distance_vec)}")
    print(f"速度数据长度: agent={len(agent_vel_vec)}, ego={len(ego_vel_vec)}, rear_dist={len(rear_distance_vec)}")
    print(f"压线率: lc_ego_press_line_ratio={plan_debug_json_msg.get('lc_ego_press_line_ratio', 'N/A')}")
    if len(box_longitudinal_buff_vec) > 0:
      print(f"box_longitudinal_buff_vec 前3个值: {box_longitudinal_buff_vec[:3]}")

    # 创建索引
    length = max(len(box_longitudinal_buff_vec), len(box_ttc_vec), len(distance_vec),
                 len(agent_vel_vec), len(ego_vel_vec), len(rear_distance_vec))
    if length > 0:
      index = list(range(length))

      # 更新数据源
      safety_check_data.data = {
          'index': index,
          'box_longitudinal_buff': box_longitudinal_buff_vec if len(box_longitudinal_buff_vec) > 0 else [0] * length,
          'box_ttc': box_ttc_vec if len(box_ttc_vec) > 0 else [0] * length,
          'distance': distance_vec if len(distance_vec) > 0 else [0] * length,
          'agent_vel': agent_vel_vec if len(agent_vel_vec) > 0 else [0] * length,
          'ego_vel': ego_vel_vec if len(ego_vel_vec) > 0 else [0] * length,
          'rear_distance': rear_distance_vec if len(rear_distance_vec) > 0 else [0] * length,
      }
      print(f"已更新安全检查数据，共 {length} 个点")

      # 动态调整速度图的纵坐标范围（自车速度 ± 15 kph）
      if len(ego_vel_vec) > 0:
        avg_ego_vel = sum(ego_vel_vec) / len(ego_vel_vec)
        vel_margin = 15.0 / 3.6  # 15 kph 转换为 m/s ≈ 4.17 m/s
        vel_min = max(0, avg_ego_vel - vel_margin)
        vel_max = avg_ego_vel + vel_margin
        fig_safety_vel.y_range.start = vel_min
        fig_safety_vel.y_range.end = vel_max
        print(f"速度图纵坐标范围: [{vel_min:.2f}, {vel_max:.2f}] m/s")
    else:
      print("警告: 安全检查数据为空")

    # 更新压线比例显示（独立于数据长度）
    lc_ego_press = plan_debug_json_msg.get('lc_ego_press_line_ratio', None)

    if lc_ego_press is not None:
      press_line_div.text = f"<h3>压线比例: <span style='color:blue;'>{lc_ego_press:.3f}</span></h3>"
      print(f"更新压线率显示: {lc_ego_press:.3f}")
    else:
      press_line_div.text = f"<h3>压线比例: <span style='color:gray;'>N/A</span></h3>"
      print("压线率: N/A")

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

# Tab 2: 安全检查（创建安全检查图表）
# 创建安全检查数据源
safety_check_data = ColumnDataSource(data={
    'index': [],
    'box_longitudinal_buff': [],
    'box_ttc': [],
    'distance': [],
    'ego_press_line_ratio': [],
    'agent_vel': [],  # 后车速度
    'ego_vel': [],    # 自车速度
    'rear_distance': []  # 后车到自车的纵向距离
})

# 创建三个图表（仿照 vel 和 s 图表的样式）
# 图1: 距离和缓冲区（合并）
fig_safety_dist = bkp.figure(x_axis_label='Prediction Step', y_axis_label='Distance (m)',
                             x_range=[0, 16], y_range=[-5, 30],
                             width=525, height=330,
                             title="距离与缓冲区")
# 超出安全距离（>= 0 表示安全）
dist_renderer = fig_safety_dist.line('index', 'distance', source=safety_check_data,
                     line_width=2, line_color='red', line_dash='solid',
                     legend_label='over-safety_distance > 0')
fig_safety_dist.circle('index', 'distance', source=safety_check_data,
                       size=4, color='red')
# 轨迹后车间隔
rear_dist_renderer = fig_safety_dist.line('index', 'rear_distance', source=safety_check_data,
                     line_width=2, line_color='brown', line_dash='dashed',
                     legend_label='net_gap_on_opt')
fig_safety_dist.circle('index', 'rear_distance', source=safety_check_data,
                       size=4, color='brown')
# 最小安全缓冲区
buff_renderer = fig_safety_dist.line('index', 'box_longitudinal_buff', source=safety_check_data,
                     line_width=2, line_color='blue', line_dash='dotted',
                     legend_label='mini_safety_buff')
fig_safety_dist.circle('index', 'box_longitudinal_buff', source=safety_check_data,
                       size=4, color='blue')
fig_safety_dist.legend.click_policy = 'hide'

# 图2: TTC
fig_safety_ttc = bkp.figure(x_axis_label='Prediction Step', y_axis_label='TTC (s)',
                            x_range=fig_safety_dist.x_range, y_range=[0, 10],
                            width=525, height=330,
                            title="时间碰撞余量")
ttc_renderer = fig_safety_ttc.line('index', 'box_ttc', source=safety_check_data,
                    line_width=2, line_color='green', line_dash='solid',
                    legend_label='ttc')
fig_safety_ttc.circle('index', 'box_ttc', source=safety_check_data,
                      size=4, color='green')
fig_safety_ttc.legend.click_policy = 'hide'

# 图3: 速度对比图（仅后车）- 纵坐标动态调整
fig_safety_vel = bkp.figure(x_axis_label='Prediction Step', y_axis_label='Velocity (m/s)',
                            x_range=fig_safety_dist.x_range,
                            width=525, height=330,
                            title="速度对比（后车）")
# 后车速度
agent_vel_renderer = fig_safety_vel.line('index', 'agent_vel', source=safety_check_data,
                    line_width=2, line_color='orange', line_dash='solid',
                    legend_label='agent_vel')
fig_safety_vel.circle('index', 'agent_vel', source=safety_check_data,
                      size=4, color='orange')
# 自车速度
ego_vel_renderer = fig_safety_vel.line('index', 'ego_vel', source=safety_check_data,
                    line_width=2, line_color='purple', line_dash='dashed',
                    legend_label='ego_vel')
fig_safety_vel.circle('index', 'ego_vel', source=safety_check_data,
                      size=4, color='purple')
fig_safety_vel.legend.click_policy = 'hide'

# 压线比例显示（单个数值，使用 Div）
press_line_div = Div(text="<h3>压线比例: <span style='color:blue;'>N/A</span></h3>",
                     width=525, height=100)

# 组合安全检查图表
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
