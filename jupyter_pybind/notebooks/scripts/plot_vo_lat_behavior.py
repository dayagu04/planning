import sys, os
sys.path.append("..")
import inspect
# from lib.load_cyberbag import *
from lib.load_local_view import *
sys.path.append('../..')
sys.path.append('../../../')
from bokeh.models import ColumnDataSource, DataTable, DateFormatter, TableColumn
from bokeh.models import TextInput
# bag path and frame dt
bag_path = "/docker_share/data/clren/code/new_planning_3/planning/20230207132027-plan.record.00000"
frame_dt = 0.02 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

plan_debug_msg_idx = 0
obj_id = 0
### sliders config
class LatBehaviorSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
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
behavior_data = ColumnDataSource({
  'name':[],
  'data':[]
})
data_d_poly = ColumnDataSource({
  'd_poly_y':[],
  'd_poly_x':[]
})
columns = [
        TableColumn(field="name", title="name",),
        TableColumn(field="data", title="data"),
    ]
data_obstacle_table = DataTable(source=obstacle_data, columns=columns, width=400, height=800)
data_behavior_table = DataTable(source=behavior_data, columns=columns, width=400, height=800)
fig1.line('d_poly_y', 'd_poly_x', source = data_d_poly, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'd_poly')
# 障碍物id的文本框的回调函数
def obj_id_handler(id):
  global obj_id
  obj_id = id
  if bag_loader.plan_debug_msg['enable'] == True:
    environment_model_info = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].environment_model_info
    obj_vars = ['id','s','l','s_to_ego','max_l_to_ref','min_l_to_ref','nearest_l_to_desire_path', \
            'nearest_l_to_ego', 'vs_lat_relative','vs_lon_relative','vs_lon',
              'nearest_y_to_desired_path','is_accident_car','is_accident_cnt','is_avoid_car','is_lane_lead_obstacle',
              'current_lead_obstacle_to_ego']

    names  = []
    datas = []
    for obstacle in environment_model_info.obstacle:
      if obstacle.id == id:
        for name in obj_vars:
          try:
            # print(getattr(obstacle,name))
            names.append(name)
            datas.append(getattr(obstacle,name))
          except:
            pass
    obstacle_data.data.update({
      'name': names,
      'data': datas,
    })
  push_notebook()  


def update_behavior_data(vo_lat_behavior_plan):
  vars = ['lc_request','lc_request_source','lc_status','is_lc_valid','lc_valid_cnt','lc_invalid_obj_id','lc_invalid_reason',\
      'lc_valid_back','lc_back_obj_id','lc_back_cnt','lc_back_invalid_reason',\
        'turn_light','turn_light_source','v_relative_left_lane','is_faster_left_lane','faster_left_lane_cnt','v_relative_right_lane',\
          'is_faster_right_lane','faster_right_lane_cnt','is_forbid_left_alc_car','is_forbid_right_alc_car',\
            'is_side_borrow_bicycle_lane','is_side_borrow_lane','fix_lane_virtual_id','origin_lane_virtual_id','target_lane_virtual_id','has_origin_lane',\
              'has_target_lane','enable_left_lc','enable_right_lc','lc_back_reason']
  # 'near_car_ids_origin','near_car_ids_target', 'left_alc_car_ids','right_alc_car_ids', ,'avoid_car_ids','avoid_car_allow_max_opposite_offset'
  names  = []
  datas = []
  for name in vars:
    try:
      # print(getattr(vo_lat_behavior_plan,name))
      names.append(name)
      datas.append(getattr(vo_lat_behavior_plan,name))
    except:
      pass
  behavior_data.data.update({
    'name': names,
    'data': datas,
  })    
  push_notebook()

def slider_callback(bag_time):
  global plan_debug_msg_idx
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  plan_debug_msg_idx = 0
  if bag_loader.plan_debug_msg['enable'] == True:
    while bag_loader.plan_debug_msg['t'][plan_debug_msg_idx] <= bag_time and plan_debug_msg_idx < (len(bag_loader.plan_debug_msg['t'])-2):
        plan_debug_msg_idx = plan_debug_msg_idx + 1
  obj_id_handler(obj_id)
  if bag_loader.plan_debug_msg['enable'] == True:
    vo_lat_motion_plan = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].vo_lat_motion_plan
    basic_dpoly = vo_lat_motion_plan.basic_dpoly
    d_poly_x, d_poly_y = gen_line(basic_dpoly[3], basic_dpoly[2], basic_dpoly[1], basic_dpoly[0], 0,60)
    data_d_poly.data.update({
      'd_poly_y':d_poly_y,
      'd_poly_x':d_poly_x
    })
    
    vo_lat_behavior_plan = bag_loader.plan_debug_msg['data'][plan_debug_msg_idx].vo_lat_behavior_plan
    update_behavior_data(vo_lat_behavior_plan)
  push_notebook()

bkp.show(row(fig1, data_obstacle_table, data_behavior_table), notebook_handle=True)
slider_class = LatBehaviorSlider(slider_callback)
slider_class = ObjText(obj_id_handler)
