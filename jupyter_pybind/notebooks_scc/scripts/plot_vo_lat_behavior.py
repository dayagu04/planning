import sys, os
sys.path.append("..")
sys.path.append("../lib/")
import inspect
# from lib.load_cyberbag import *
from lib.load_local_view import *
sys.path.append('../..')
sys.path.append('../../../')
from bokeh.models import ColumnDataSource, DataTable, DateFormatter, TableColumn
from bokeh.models import TextInput
# bag path and frame dt
bag_path = '/root/clzhao/lane_borrow_data/2024-11-27-11-44-13.bag_2024-11-27-13-51-09.1732782549.close-loop.scc.plan'
frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()
fig1.legend.label_text_font_size = "8pt"
fig1.height = 950
fig1.width = 700

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

obstacle_data = ColumnDataSource({
  'name':[],
  'data':[]
})
behavior_data_1 = ColumnDataSource({
  'name':[],
  'data':[]
})
behavior_data_2 = ColumnDataSource({
  'name':[],
  'data':[]
})
data_d_poly = ColumnDataSource({
  'd_poly_y':[],
  'd_poly_x':[]
})
data_fix_lane = ColumnDataSource({
  'fixlane_y':[],
  'fixlane_x':[]
})
data_avd_cars = ColumnDataSource({
  'pos_y':[],
  'pos_x':[]
})
lc_data_3 = ColumnDataSource({
  'name':[],
  'data':[]
})
overtake_lc_data = ColumnDataSource({
  'name':[],
  'data':[]
})
cone_lc_data = ColumnDataSource({
  'name':[],
  'data':[]
})
merge_lc_data = ColumnDataSource({
  'name':[],
  'data':[]
})
columns = [
        TableColumn(field="name", title="name",),
        TableColumn(field="data", title="data"),
    ]
data_obstacle_table = DataTable(source=obstacle_data, columns=columns, width=350, height=600)
data_behavior_table_1 = DataTable(source=behavior_data_1, columns=columns, width=350, height=900)
data_behavior_table_2 = DataTable(source=behavior_data_2, columns=columns, width=350, height=100)
data_lc_table_3 = DataTable(source=lc_data_3, columns=columns, width=350, height=1000)
data_overtake_lc_table = DataTable(source=overtake_lc_data,columns=columns, width=350, height=500)
data_cone_lc_table = DataTable(source=cone_lc_data,columns=columns, width=350, height=200)
data_merge_lc_table = DataTable(source=merge_lc_data,columns=columns, width=350, height=200)


fig1.line('d_poly_y', 'd_poly_x', source = data_d_poly, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'd_poly')
fig1.line('fixlane_y', 'fixlane_x', source = data_fix_lane, line_width = 1, line_color = 'black', line_dash = 'dotted', line_alpha = 0.8, legend_label = 'fix_lane')
fig1.circle('pos_y', 'pos_x', source = data_avd_cars, color='red',radius=1,fill_color='yellow',line_color='green',fill_alpha = 0.3, legend_label = 'avoid_car')
# 障碍物id的文本框的回调函数
def obj_id_handler(id):
  global obj_id
  obj_id = id
  if bag_loader.plan_debug_msg['enable'] == True:
    plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
    environment_model_info = plan_debug_msg.environment_model_info
    obj_vars = ['id','type','s','l','s_to_ego','max_l_to_ref','min_l_to_ref','nearest_l_to_desire_path', \
            'nearest_l_to_ego', 'vs_lat_relative','vs_lon_relative','vs_lon',
              'nearest_y_to_desired_path','is_accident_car','is_accident_cnt','is_avoid_car','is_lane_lead_obstacle',
              'current_lead_obstacle_to_ego','cutin_p']

    names  = []
    datas = []
    is_find = False
    for obstacle in environment_model_info.obstacle:
      if obstacle.id == id:
        is_find = True
        for name in obj_vars:
          try:
            # print(getattr(obstacle,name))
            datas.append(getattr(obstacle,name))
            names.append(name)
          except:
            pass
    if not is_find:
      for obstacle in environment_model_info.obstacle:
        id = obstacle.id
        for name in obj_vars:
          try:
            datas.append(getattr(obstacle,name))
            names.append(name)
          except:
            pass
        break
    # try:
    names.append('ego_s')
    names.append('ego_l')
    datas.append(environment_model_info.ego_s)
    datas.append(environment_model_info.ego_l)
    # except:
      # pass
    obstacle_data.data.update({
      'name': names,
      'data': datas,
    })

  push_notebook()

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

  names  = []
  datas = []
  try:
    # 横向运动规划offset 可视化
    names.append('premove_dpoly_c0')
    names.append('avoid_dpoly_c0')
    basic_dpoly = vo_lat_motion_plan.basic_dpoly
    datas.append(vo_lat_motion_plan.premove_dpoly_c0 - basic_dpoly[3])
    datas.append(vo_lat_motion_plan.avoid_dpoly_c0 - basic_dpoly[3])
  except:
    pass

  # 添加可视化left_alc_car_ids、right_alc_car_ids可视化
  names.append('left_alc_car_ids')
  names.append('right_alc_car_ids')
  left_alc_car_id_str = ""
  right_alc_car_id_str = ""
  for left_alc_car_id in lat_behavior_common.left_alc_car_ids:
    left_alc_car_id_str = left_alc_car_id_str + str(left_alc_car_id) + ' '
  for right_alc_car_id in lat_behavior_common.right_alc_car_ids:
    right_alc_car_id_str = right_alc_car_id_str + str(right_alc_car_id) + ' '
  datas.append(left_alc_car_id_str)
  datas.append(right_alc_car_id_str)
  behavior_data_2.data.update({
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
  vars_lc = ['sdmap_valid_', 'turn_switch_state','lane_change_cmd_','cur_state','lc_map_decision','ramp_direction',
             'is_ego_on_expressway','current_lane_order_id','current_lane_virtual_id','current_lane_relative_id',
             'left_boundary_type','right_boundary_type',"current_segment_id","distance_to_route_end","sum_dis_to_last_merge_point",
             'is_leaving_ramp','is_nearing_ramp','road_to_ramp_turn_signal','merge_lane_virtual_id','is_merge_region',"cur_lane_is_continue",
             'is_split_region', 'ego_lane_boundary_exist_virtual_line','target_lane_boundary_exist_virtual_line',
             'is_left_merge_direction', 'is_right_merge_direction',
             'distance_to_ramp','distance_to_first_road_merge','distance_to_first_road_split','is_nearing_other_lane_merge_to_road_point',
             'virtual_lane_relative_id_switch_flag',
             'is_exist_split_on_ramp','is_exist_ramp_on_road','is_exist_split_on_expressway','is_exist_intersection_split',
             'current_segment_passed_distance','is_in_ramp_select_split_situation','is_on_road_select_ramp_situation',
             'select_ego_lane_without_plan', 'select_ego_lane_with_plan', 'forward_lane_num',
             'is_ego_on_split_region', 'last_split_seg_dir', 'need_continue_lc_num_on_off_ramp_region']
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

def update_overtake_request_lc_data (plan_debug_json):
  names  = []
  datas = []
  overtake_lc_vars_ = ["enable_l_", "enable_r_", "is_left_lane_change_safe_", "is_right_lane_change_safe_",
                       "overtake_count_", "is_left_overtake", "is_right_overtake", "trigger_left_overtake",
                       "trigger_right_overtake", "overtake_vehicle_id", "dash_line_len",
                       "left_route_traffic_speed", "right_route_traffic_speed", "speed_threshold"]
  for name in overtake_lc_vars_:
    try:
      datas.append((plan_debug_json[name]))
      names.append(name)
    except:
      pass
  overtake_lc_data.data.update({
    'name': names,
    'data': datas,
  })
  push_notebook()

def update_cone_request_lc_data (plan_debug_json):
  names  = []
  datas = []
  cone_lc_vars_ = ["is_cone_lane_change_situation_", "cone_alc_trigger_counter_", "cone_lane_change_direction_",
                   "cone_nums_of_front_objects", "is_emergency_avoidance_situation_", "leading_vehicle_id_"]
  for name in cone_lc_vars_:
    try:
      datas.append((plan_debug_json[name]))
      names.append(name)
    except:
      pass
  cone_lc_data.data.update({
    'name': names,
    'data': datas,
  })
  push_notebook()

def update_merge_request_lc_data (plan_debug_json):
  names  = []
  datas = []
  merge_lc_vars_ = ["both_lane_line_exist_virtual_or_not_","is_merge_lane_change_situation_",
                    "merge_alc_trigger_counter_", "left_boundary_exist_virtual_type",
                    "right_boundary_exist_virtual_type"]
  for name in merge_lc_vars_:
    try:
      datas.append((plan_debug_json[name]))
      names.append(name)
    except:
      pass
  merge_lc_data.data.update({
    'name': names,
    'data': datas,
  })
  push_notebook()

def slider_callback(bag_time):
  global plan_debug_msg_idx
  local_view_data_ = update_local_view_data(fig1, bag_loader, bag_time, local_view_data)

  obj_id_handler(obj_id)
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
    update_overtake_request_lc_data(plan_debug_json_msg)
    update_cone_request_lc_data(plan_debug_json_msg)
    update_merge_request_lc_data(plan_debug_json_msg)

    lat_behavior_plan = plan_debug_msg.vo_lat_behavior_plan

    # 可视化avoid cars
    pos_y_rels = []
    pos_x_rels = []
    for id in lat_behavior_plan.avoid_car_ids:
      tmp_str = '(' + str(id) + ')'
      for index, obs_label in enumerate(local_view_data_['data_fus_obj'].data['obs_label']):
        if tmp_str in obs_label and 'pos_x_rel' in local_view_data_['data_fus_obj'].data:
          pos_x_rel = local_view_data_['data_fus_obj'].data['pos_x_rel'][index]
          pos_y_rel = local_view_data_['data_fus_obj'].data['pos_y_rel'][index]
          pos_y_rels.append(pos_y_rel)
          pos_x_rels.append(pos_x_rel)
          break
    # print('avoid obstacle: ', lat_behavior_plan.avoid_car_ids)
    data_avd_cars.data.update({
      'pos_y':pos_y_rels,
      'pos_x':pos_x_rels,
    })

  push_notebook()

# +
bkp.show(row(fig1, column(data_behavior_table_1), column(data_lc_table_3),
            column(data_overtake_lc_table, data_merge_lc_table),column(data_obstacle_table, data_behavior_table_2, data_cone_lc_table)), notebook_handle=True)
slider_class = LatBehaviorSlider(slider_callback)

# slider_class = ObjText(obj_id_handler)
