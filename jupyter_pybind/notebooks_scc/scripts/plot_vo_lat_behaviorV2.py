import sys, os
sys.path.append("..")
sys.path.append("../lib/")
import inspect
from lib.load_ros_bag import LoadRosbag
from lib.load_local_view import *
sys.path.append('../..')
sys.path.append('../../../')
from bokeh.models import ColumnDataSource, DataTable, DateFormatter, TableColumn
from bokeh.models import TextInput
from bokeh.resources import INLINE
from google.protobuf.descriptor import FieldDescriptor

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_20260/trigger/20250610/20250610-16-11-12/data_collection_CHERY_E0Y_20260_EVENT_FILTER_2025-06-10-16-11-12_no_camera.bag.1749623912.close-loop.scc.plan"
frame_dt = 0.1 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
global_var.set_value('g_is_display_enu', False)
fig1, local_view_data = load_local_view_figure()
fig1.legend.label_text_font_size = "8pt"
fig1.height = 900
# fig1.width = 700

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

lane_borrow_info_data = ColumnDataSource({# data launch
  'name':[],
  'data':[]
})
dp_path_info_data = ColumnDataSource({# data launch
  'name':[],
  'data':[]
})
columns = [
        TableColumn(field="name", title="name",),
        TableColumn(field="data", title="data"),
    ]
lane_borrow_info_table = DataTable(source=lane_borrow_info_data, columns=columns, width=350, height=1000)# table launch, data connect
dp_path_info_table = DataTable(source=dp_path_info_data, columns=columns, width=350, height=1000)# table launch, data connect

def extract_fields_with_names(proto, prefix="DP"):
    table_name = []
    table_value = []

    for field in proto.DESCRIPTOR.fields:
        field_name = field.name
        full_name = f"{prefix}.{field_name}" if prefix else field_name

        value = getattr(proto, field_name)
        if field.type == FieldDescriptor.TYPE_MESSAGE:
            if value.ListFields():  #
                sub_table_name, sub_table_value = extract_fields_with_names(value, full_name)
                table_name.extend(sub_table_name)
                table_value.extend(sub_table_value)
            else:
                table_name.append(full_name)
                table_value.append(None)
        else:
            # 普通字段直接添加
            table_name.append(full_name)
            table_value.append(value)

    return table_name, table_value

def update_lane_borrow_data(lane_borrow_common): # update function
  datas = []
  names = []
  vars = ['lane_borrow_decider_status', 'ego_l','target_left_l','target_right_l',
            'start_solid_lane_dis', 'end_solid_lane_dis','dis_to_traffic_lights','safe_left_borrow',
              'safe_right_borrow', 'static_blocked_obj_id_vec', 'intersection_state', 'lane_borrow_failed_reason','borrow_turn_circle',
              'front_obs_center']
  for name in vars:
      try:
        value = getattr(lane_borrow_common, name)
        if name == 'static_blocked_obj_id_vec':
          data_i = []
          for value_i in value:
            data_i.append(value_i)
          datas.append(data_i)
        elif name == 'borrow_turn_circle':
          data_i =[]
          data_i.append(value.corner_radius)
          data_i.append(value.center.x)
          data_i.append(value.center.y)
          datas.append(data_i)
        else:
          datas.append(value)
        names.append(name)
      except:
        pass
  lane_borrow_info_data.data.update({ # data update
    'name': names,
    'data': datas,
  })
  push_notebook()
def update_dp_path_data(dp_path_common):
  print_info = dp_path_common.print_info
  table_name, table_var = extract_fields_with_names(print_info)
  dp_path_info_data.data.update({
     'name':table_name,
     'data':table_var,
  })
def slider_callback(bag_time):
  global plan_debug_msg_idx
  local_view_data_ = update_local_view_data(fig1, bag_loader, bag_time, local_view_data)

  if bag_loader.plan_debug_msg['enable'] == True:
    lane_borrow_common = local_view_data['data_msg']['plan_debug_msg'].lane_borrow_decider_info
    dp_path_commom = local_view_data['data_msg']['plan_debug_msg'].dp_road_info
    try:
      update_lane_borrow_data(lane_borrow_common)# call update function
      update_dp_path_data(dp_path_commom)
    except:
      pass
  push_notebook()


# +
bkp.show(row(fig1, column(lane_borrow_info_table),column(dp_path_info_table)), notebook_handle=True) # table combine
slider_class = LatBehaviorSlider(slider_callback)

# slider_class = ObjText(obj_id_handler)
