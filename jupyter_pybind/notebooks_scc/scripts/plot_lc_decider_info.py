import sys, os
sys.path.append("..")
sys.path.append("../lib/")
import inspect
# from lib.load_cyberbag import *
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
from lib.load_lc_st_graph import *
sys.path.append('../..')
sys.path.append('../../../')
from bokeh.models import ColumnDataSource, DataTable, DateFormatter, TableColumn
from bokeh.models import TextInput
from bokeh.resources import INLINE
# bag path and frame dt
# bag_path = "/pnc_x86_data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250311/20250311-10-51-40/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-03-11-10-51-40_no_camera.bag"
# bag_path = "/pnc_x86_data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20250311/20250311-10-58-23/data_collection_CHERY_E0Y_04228_EVENT_MANUAL_2025-03-11-10-58-23_no_camera.bag"
bag_path = "/pnc_x86_data_cold/abu_zone/autoparse/chery_m32t_82006/trigger/20250815/20250815-09-30-44/data_collection_CHERY_M32T_82006_EVENT_KEY_2025-08-15-09-30-44_no_camera.bag"
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
data_behavior_table_1 = DataTable(source=behavior_data_1, columns=columns, width=350, height=900)
data_lc_table_3 = DataTable(source=lc_data_3, columns=columns, width=350, height=1000)

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
  vars_lc = ['sdpromap_valid_', 'turn_switch_state','lane_change_cmd_','cur_state','lc_map_decision','ramp_direction',
             "first_split_direction","first_merge_direction",'is_ego_on_expressway','current_lane_order_id','current_lane_virtual_id','current_lane_relative_id',
             'left_boundary_type','right_boundary_type',"current_segment_id","distance_to_route_end","sum_dis_to_last_merge_point",
             'is_leaving_ramp','is_nearing_ramp','road_to_ramp_turn_signal','merge_lane_virtual_id','is_merge_region',"cur_lane_is_continue",
             'is_split_region', 'distance_to_ramp','distance_to_first_road_merge','distance_to_first_road_split','is_nearing_other_lane_merge_to_road_point',
             'current_segment_passed_distance', 'forward_lane_num',
             'is_ego_on_split_region', 'last_split_seg_dir', 'need_continue_lc_num_on_off_ramp_region',
             'ramp_pass_sts', 'target_lane_congestion_level','lat_offset_propose','lat_offset_lc_hold', "ego_press_line_ratio",'ego_status_on_route','left_lane_num', 'minVal_seq', 'maxVal_seq']
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

def slider_callback(bag_time):
  global plan_debug_msg_idx
  local_view_data_ = update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  update_lc_path_figure (data_st, lat_data_vec, ori_lat_data_vec, lc_path_data_vec, bag_loader, bag_time, local_view_data,
                         ego_box_data_vec, agent_box_data_vec)

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

  push_notebook()

# +

bkp.show(row(fig1, column(data_behavior_table_1), column(data_lc_table_3), column(fig2, fig3)), notebook_handle=True)
slider_class = LatBehaviorSlider(slider_callback)

# slider_class = ObjText(obj_id_handler)
