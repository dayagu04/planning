

import sys
import os
from abc import ABC, abstractmethod
import bokeh.plotting as bkp
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, NumericInput, DataTable, TableColumn, Panel, Tabs
from bokeh.io import output_notebook, push_notebook, output_file, export_png
from bokeh.layouts import layout, column, row
from bokeh.plotting import figure, output_file, show, ColumnDataSource

import numpy as np
from IPython.core.display import display, HTML

import logging
sys.path.append('..')
sys.path.append('../..')
sys.path.append('../../..')
from lib.basic_layers import *
from lib.bag_loader import *
from lib.load_struct import *

plan_debug_ts = []
plan_debug_timestamps = []
fusion_object_timestamps = []
fusion_road_timestamps = []
localization_timestamps = []
prediction_timestamps = []
vehicle_service_timestamps = []
control_output_timestamps = []
slot_timestamps = []
fusion_slot_timestamps = []
vision_slot_timestamps = []
mobileye_lane_lines_timestamps = []
mobileye_objects_timestamps = []

car_xb, car_yb = load_car_params_patch()
car_circle_x, car_circle_y, car_circle_r = load_car_circle_coord()
coord_tf = coord_transformer()

# params 控制fig的样式
vs_car_params_apa ={
  'text_color' : 'firebrick',
  'text_align' : "center",
  'legend_label' : 'text',
  'text_font_size' : '12pt'
}

ego_car_params_apa ={
  'fill_color' : 'palegreen',
  'line_color' : "black",
  'line_width' : 1,
  'legend_label' : 'car'
}
ego_pose_params_apa ={
  'fill_color' : "red",
  'line_color' : "black",
  'line_width' : 1,
  'legend_label' : 'car_rear_alxe_center'
}
ego_dot_params_apa ={
  'size' : 8,
  'color' : "grey",
  'legend_label' : 'car'
}
ego_circle_params_apa ={
  'line_alpha' : 0.5,
  'line_width' : 1,
  'line_color' : 'blue',
  'fill_alpha' : 0,
  'legend_label' : 'car_circle',
  'visible' : False
}
slot_params_apa = {
  'fill_color' : None,
  'line_color' : "red",
  'line_width' : 1,
  'legend_label' : 'slot'
}
fusion_slot_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "red",
  'line_width' : 2,
  'legend_label' : 'fusion_parking_slot'
}
vision_slot_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "lightgrey",
  'line_width' : 3,
  'legend_label' : 'vision_parking_slot',
  'visible' : False
}

final_slot_params_apa = {
  'line_dash' : 'dashed',
  'line_color' : "#A52A2A",
  'line_width' : 3,
  'legend_label' : 'final_parking_slot'
}

target_slot_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "green",
  'line_width' : 3,
  'legend_label' : 'target_managed_slot'
}

all_slot_params_apa = {
  'line_dash' : 'solid',
  'line_color' : "green",
  'line_width' : 2,
  'legend_label' : 'all managed slot',
  'visible' : False
}

location_params_apa = {
  'line_width' : 1.5,
  'line_color' : 'orange',
  'line_dash' : 'solid',
  'legend_label' : 'ego_pos'
}
slot_id_params_apa = { 'text_color' : "red", 'text_align':"center", 'text_font_size':"10pt", 'legend_label' : 'fusion_parking_slot' }
ego_pose_params ={
  'fill_color' : "palegreen",
  'line_color' : "black",
  'line_width' : 1,
  'legend_label' : 'car'
}
ego_info_paramsv2 = { 'text_color' : "firebrick", 'text_align':"center", 'text_font_size':"12pt", 'legend_label' : 'car' }
ego_info_params = { 'text_color' : "firebrick", 'text_align':"center", 'text_font_size':"12pt", 'legend_label' : 'car1' }
location_params = {
  'line_width' : 1,
  'line_color' : 'orange',
  'line_dash' : 'solid',
  'legend_label' : 'ego_pos'
}

uss_wave_params = {
  'fill_color' : 'lavender',
  'line_color' : 'black',
  'legend_label' : 'uss_wave',
  'alpha' : 0.5
}

uss_text_params = {
  'text_color' : 'black',
  'text_align' : 'center',
  'text_font_size' : '10pt',
  'legend_label' : 'uss_wave'
}

lane_params = {
    'legend_label': 'lane',
    'line_width': 1.5,
    'line_color': 'black',
    'line_dash' : 'dashed',
    'alpha': 1
}

center_line_params = {
    'line_width' : 1,
    'line_color' : 'blue',
    'line_dash' : 'dotted',
    'line_alpha' : 0.8
}
center_lane_params_2 = {
    'line_width' : 1,
    'line_color' : 'blue',
    'line_dash' : 'dotted',
    'line_alpha' : 0.8
}
center_lane_params_3 = {
    'line_width' : 1,
    'line_color' : 'blue',
    'line_dash' : 'dotted',
    'line_alpha' : 0.8
}
center_lane_params_4 = {
    'line_width' : 1, 'line_color' : 'blue', 'line_dash' : 'dotted', 'line_alpha' : 0.8
}
center_lane_params_5 = {
    'line_width' : 1, 'line_color' : 'blue', 'line_dash' : 'dotted', 'line_alpha' : 0.8
}

fix_lane_params = {
    'legend_label': 'flane',
    'line_width' : 1.3,
    'line_color' : 'red',
    'line_dash' : 'dotted',
    'line_alpha' : 0.8
}

target_lane_params = {
    'legend_label': 'tlane',
    'line_width' : 1.3,
    'line_color' : 'black',
    'line_dash' : 'dotted',
    'line_alpha' : 0.8
}

origin_lane_params = {
  'legend_label': 'olane',
    'line_width' : 1,
    'line_color' : 'orange',
    'line_dash' : 'dotted',
    'line_alpha' : 0.8
}

mobileye_lane_lines_params = {
  'legend_label': 'mlane',
  'line_width' : 1.3,
  'line_color' : 'chocolate',
  'line_dash' : 'dashed',
  'line_alpha' : 1.0
}

obstacle_fusion_params = {
    'fill_color' : "gray",
    'line_color' : "black",
    'line_width' : 1,
    'fill_alpha' : 0.3,
    'legend_label' : 'obj'
}

obstacle_snrd_params = {
    'fill_color' : "black",
    'line_color' : "black",
    'line_width' : 1,
    'fill_alpha' : 0.5,
    'legend_label' : 'obj'
}

obstacle_text_params = {
  'legend_label' : 'obj_info',
  'text_color' : "red",
  'text_align':"center",
  'text_font_size':"10pt"
}

obstacle_mobileye_params = {
  'fill_color' : "sienna",
  'line_color' : "black",
  'line_width' : 1,
  'fill_alpha' : 0.5,
  'legend_label' : 'mobj'
}

obstacle_mobileye_text_params = {
  'legend_label' : 'mobj_info',
  'text_color' : "salmon",
  'text_align':"center",
  'text_font_size':"10pt"
}

prediction_params = {
  'line_width' : 6, 'line_color' : 'orange', 'line_dash' : 'solid', 'line_alpha' : 0.5, 'legend_label' : 'prediction'
}

plan_params = {
  'line_width' : 2.5, 'line_color' : 'blue', 'line_dash' : 'solid', 'line_alpha' : 0.6, 'legend_label' : 'plan'
}

mpc_params = {
  'line_width' : 3.0, 'line_color' : 'red', 'line_dash' : 'solid', 'line_alpha' : 0.8, 'legend_label' : 'mpc'
}

control_params = {
  'line_width' : 5, 'line_color' : 'green', 'line_dash' : 'dashed', 'line_alpha' : 0.8, 'legend_label' : 'ctrl_traj'
}

table_params={
    'width': 600,
    'height':500,
}

# 判断是否在jupyter中运行
def isINJupyter():
    try:
        __file__
    except NameError:
        return True
    else:
        return False

# 数据处理的基本类
class DataGeneratorBase(ABC):
    def __init__(self, xys, ts, accu=False, name="Default Name"):
        self.xys = xys
        self.ts = np.array(ts)

        self.accu = accu
        self.name = name

    def getMinT(self):
        if len(self.ts) > 0:
            return self.ts[0]
        else:
            return float('inf')

    def getMaxT(self):
        if len(self.ts) > 0:
            return self.ts[-1]
        else:
            return -1.0

    def atTa(self, T):
        if len(self.xys[0]) == 0:
            return [[], []]
        if self.getMinT() > T:
            return (self.xys[0][0:1], self.xys[1][0:1])

        first_larger_index = np.where(self.ts > T)[0]
        if first_larger_index.size == 0:
            return (self.xys[0], self.xys[1])

        return (self.xys[0][0: first_larger_index[0]], self.xys[1][0:first_larger_index[0]])

    def atT(self, T):
        if self.accu:
            return self.atTa(T)
        if len(self.xys) == 0:
            return [[], []]
        if self.getMinT() > T:
            return self.xys[0]

        first_larger_index = np.where(self.ts > T)[0]
        if first_larger_index.size == 0:
            return self.xys[-1]

        return self.xys[first_larger_index[0] - 1]

class LineGenerator(DataGeneratorBase):
  def __init__(self, attr):
    self.ts = []
    self.xys = []
    super().__init__(self.xys, self.ts)
    self.line = attr
    return


class CommonGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    super().__init__(self.xys, self.ts)
    return
class TextGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    self.txt = "text"
    super().__init__(self.xys, self.ts)
    return
  
class CircleGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    self.rs = []
    super().__init__(self.xys, self.ts)
    return

class WedgesGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    self.rs = []
    self.min_angle = []
    self.max_angle = []
    super().__init__(self.xys, self.ts)
    return
class DotGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    super().__init__(self.xys, self.ts)
    return

class ObjTextGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    self.obj_txt = "text"
    super().__init__(self.xys, self.ts)
    return

# MeasureTools类,用于在fig中测试距离
class MeasureTools:
    def __init__(self, fig) -> None:
        self.fig = fig
        line_params = {
            'legend_label': 'measure tool',
            'alpha': 0.5,
            'line_width': 4,
            'line_color': 'purple'
        }
        text_params = {}
        point_params = {
            'size': 6,
            'color': 'firebrick'
        }
        self.ends = PointsLayer(fig, point_params)
        self.line = MultiLinesLayer(fig, line_params)
        self.text = TextLabelLayer(fig, text_params)

        self.old_xs, self.old_ys = 'xs', 'ys'

        self.old_datasource = ColumnDataSource(data={
            self.old_xs: [],
            self.old_ys: []
        })

        self.setEvent()

    def setEvent(self):

        callback = CustomJS(
            args=dict(
                line_source=self.line.data_source,
                text_source=self.text.data_source,
                end_source=self.ends.data_source,
                old_source=self.old_datasource
            ),
            code="""
const x=cb_obj.x;
const y=cb_obj.y;
console.log('Tap event occurred at x-position: ' + cb_obj.x)
console.log('Tap event occurred at y-position: ' + cb_obj.y)

if(old_source.data['xs'].length == 0){
    old_source.data['xs']=[x];
    old_source.data['ys']=[y];

    end_source.data['pts_xs']=[x];
    end_source.data['pts_ys']=[y];

    line_source.data['pts_xs']=[];
    line_source.data['pts_ys']=[];
    text_source.data['pts_xs']=[];
    text_source.data['pts_ys']=[];
    text_source.data['texts']=[];
}else{

    // get last point
    const ox = old_source.data['xs'][0];
    const oy = old_source.data['ys'][0];
    console.log(x,y,ox,oy);

    // clear last point
    old_source.data['xs']=[];
    old_source.data['ys']=[];

    // put last and current point
    end_source.data['pts_xs']=[x, ox];
    end_source.data['pts_ys']=[y, oy];
    line_source.data['pts_xs']=[[x, ox]];
    line_source.data['pts_ys']=[[y,oy]];
    text_source.data['pts_xs']=[(x+ox)/2.0];
    text_source.data['pts_ys']=[(y + oy)/2.0];

    const dis = Math.sqrt(Math.pow(ox-x,2) + Math.pow(oy-y,2));
    text_source.data['texts']=[dis.toFixed(4)];

}

line_source.change.emit();
text_source.change.emit();
old_source.change.emit();
end_source.change.emit();

""")
        self.fig.js_on_event('tap', callback)

# 用于管理所有的layer
class LayerManager():
    def __init__(self):
        self.layers = dict()
        self.gds = dict()
        self.data_key = dict()
        self.plotdim = dict()
        self.code = """
            %s
            """

    def AddLayer(self, newLayer, layer_label, gd, data_key=None, plotdim=None):
        self.layers[layer_label] = newLayer
        self.plotdim[layer_label] = plotdim

        # if gd.xys == []:
        #     return

        self.gds[layer_label] = gd

        if gd and data_key and plotdim:
            self.data_key[layer_label] = data_key
            layer_code = """%s
            var {}_index = binarySearch(data['{}ts'][0],data['mt'][0]+step);
            {}.data['pts_xs'] = data['{}s'][0][{}_index][0];
            {}.data['pts_ys'] = data['{}s'][0][{}_index][1];
            """.format(data_key, data_key, layer_label, data_key, data_key, layer_label, data_key, data_key)
            # 更改车道线类型属性
            if hasattr(gd, 'line'):
              if gd.line == 'line':
                layer_append = """%s
                var fig_index = data['{}s'][0][{}_index][3][0];
              {}.renderers[fig_index].glyph.line_dash = data['{}s'][0][{}_index][2][0];
            """.format( data_key,data_key,layer_label + '_fig', data_key,data_key)
                layer_code = (layer_append) % (layer_code)
            # 更改当前车道中心线的属性
              if gd.line == 'center_line':
                layer_append = """%s
                var ralative_id = data['{0}s'][0][{1}_index][2][0]
                var fig_index = data['{2}s'][0][{3}_index][3][0];
                console.log(fig_index);
                var line_dash = "dotted";
                var line_alpha = 0.8;
                var line_width = 1;
                if (ralative_id == 0)
                  line_dash = "dotdash";
                if (ralative_id == 0)
                  line_alpha = 1;
                if (ralative_id == 0)
                  line_width = 2;
                {4}.renderers[fig_index].glyph.line_dash = line_dash;
                {5}.renderers[fig_index].glyph.line_alpha = line_alpha;
                {6}.renderers[fig_index].glyph.line_width = line_width;
            """.format( data_key,data_key, \
                      data_key,data_key, \
                      layer_label + '_fig',layer_label + '_fig', layer_label + '_fig')
                layer_code = (layer_append) % (layer_code)

            if plotdim == 3 and  hasattr(gd, 'obj_txt'):
            #     layer_append = """%s
            # console.log( data['{}s'][0][{}_index][1]);
            # {}.data['pts_xs'] = data['{}s'][0][{}_index][0];
            # {}.data['pts_ys'] = data['{}s'][0][{}_index][1];
            # """.format('obstacle_generate_table_295', data_key, \
            #            layer_label, 'obstacle_generate_table_295', data_key,\
            #            layer_label, 'obstacle_generate_table_295', data_key)
            #     layer_code = (layer_append) % (layer_code)
                pass
            elif plotdim == 3 and hasattr(gd, 'txt'):
                layer_append = """%s
            {}.data['texts'] = data['{}s'][0][{}_index][2];
            """.format(layer_label, data_key, data_key)
                layer_code = (layer_append) % (layer_code)
            elif plotdim == 3 and hasattr(gd, 'rs'):
                # print(gd.rs)
                layer_append = """%s
            {}.data['rs'] = data['{}s'][0][{}_index][2];
            """.format(layer_label, data_key, data_key)
                layer_code = (layer_append) % (layer_code)
            elif plotdim == 5 and hasattr(gd, 'min_angle'):
                # print(gd.rs)
                layer_append = """%s
            {}.data['rs'] = data['{}s'][0][{}_index][2];
            {}.data['min_angle'] = data['{}s'][0][{}_index][3];
            {}.data['max_angle'] = data['{}s'][0][{}_index][4];
            """.format(layer_label, data_key, data_key, layer_label, data_key, data_key, layer_label, data_key, data_key)
                layer_code = (layer_append) % (layer_code)
            else:
                pass

            layer_append = """%s{}.change.emit();
            console.log( {}, {}_index);
            """.format(layer_label, layer_label, data_key)
            layer_code = (layer_append) % (layer_code)

            self.code = (layer_code) % (self.code)

obstacle_selector = NumericInput(value=0,title="obstacle_selector",width=100)

# bag进度条播放的回调函数
class slider_callback_arg():
    def __init__(self, bag_data):
        self.arg = dict()
        self.arg['bag_source'] = bag_data
        self.arg["obstacle_selector"] = obstacle_selector

    def AddSource(self, arg_name, layer):
        self.arg[arg_name] = layer.data_source
        self.arg[arg_name + '_fig'] = layer.fig

def find(data, t):
  for index, timestamp in enumerate(data['timestamp']):
    if t == timestamp:
      return True, data['data'][index]
    # if t < timestamp:
    #   if ((index > 0) & (abs(t - data['timestamp'][index - 1]) < abs(data['timestamp'][index] - t))):
    #     index = index - 1
    #   return True, data['data'][index]
  return False, ""

def findt(data, t):
  '''
  find data based on absolute t
  '''
  for index, timestamp in enumerate(data['abs_t']):
    if t == timestamp:
      return True, data['data'][index]

  return False, ""

def findrt(data, t):
  '''
  find data based on relative t (sub the 0 time)
  '''
  for index, timestamp in enumerate(data['t']):
    if t == timestamp:
      return True, data['data'][index]

  return False, ""

def findME(data, t):
  for index, timestamp in enumerate(data['timestamp']):
    if t < timestamp:
      if ((index > 0) & (abs(t - data['timestamp'][index - 1]) < abs(data['timestamp'][index] - t))):
        index = index - 1
      return True, data['data'][index]
  return False, ""
def load_obstacle_params(obstacle_list, environment_model_info):

  obs_info_all = dict()
  obs_num = len(obstacle_list)
  num = 0
  for i in range(obs_num):
    source = obstacle_list[i].additional_info.fusion_source
    if source & 0x01: #相机融合障碍物
      source = 1
    elif (obstacle_list[i].common_info.relative_center_position.x > 0 and \
      math.tan(25) > math.fabs(obstacle_list[i].common_info.relative_center_position.y / obstacle_list[i].common_info.relative_center_position.x)) or \
      math.fabs(obstacle_list[i].common_info.relative_center_position.y) > 10:
      continue
    else:
      source = 4
    if (source in obs_info_all.keys()) == False:
      obs_info = {
        'obstacles_x_rel': [],
        'obstacles_y_rel': [],
        'obstacles_x': [],
        'obstacles_y': [],
        'pos_x_rel': [],
        'pos_y_rel': [],
        'pos_x': [],
        'pos_y': [],
        'obstacles_vel': [],
        'obstacles_acc': [],
        'obstacles_tid': [],
        'is_cipv': [],
        'obs_label':[]
      }
      obs_info_all[source] = obs_info
    frenet_vs, frenet_vl = 255, 255
    for obstacle in environment_model_info.obstacle:
      if obstacle.id == obstacle_list[i].additional_info.track_id:
        frenet_vs, frenet_vl = obstacle.vs_lon_relative, obstacle.vs_lat_relative
        break
    long_pos_rel = obstacle_list[i].common_info.relative_center_position.x
    lat_pos_rel = obstacle_list[i].common_info.relative_center_position.y
    theta = obstacle_list[i].common_info.relative_heading_angle
    if theta == 255:
      theta = 0
    half_width = obstacle_list[i].common_info.shape.width /2
    half_length = obstacle_list[i].common_info.shape.length / 2
    # if half_width == 0 or half_length == 0:
    #   continue
    cos_heading = math.cos(theta)
    sin_heading = math.sin(theta)
    dx1 = cos_heading * half_length
    dy1 = sin_heading * half_length
    dx2 = sin_heading * half_width
    dy2 = -cos_heading * half_width

    obs_x_rel = [long_pos_rel + dx1 + dx2,
              long_pos_rel + dx1 - dx2,
              long_pos_rel- dx1 - dx2,
              long_pos_rel - dx1 + dx2,
              long_pos_rel + dx1 + dx2]
    obs_y_rel = [lat_pos_rel + dy1 + dy2,
              lat_pos_rel + dy1 - dy2,
              lat_pos_rel - dy1 - dy2,
              lat_pos_rel - dy1 + dy2,
              lat_pos_rel + dy1 + dy2]

    # 绝对坐标系下的数据
    long_pos = obstacle_list[i].common_info.center_position.x
    lat_pos = obstacle_list[i].common_info.center_position.y
    theta = obstacle_list[i].common_info.heading_angle
    cos_heading = math.cos(theta)
    sin_heading = math.sin(theta)
    dx1 = cos_heading * half_length
    dy1 = sin_heading * half_length
    dx2 = sin_heading * half_width
    dy2 = -cos_heading * half_width
    obs_x = [long_pos + dx1 + dx2,
              long_pos + dx1 - dx2,
              long_pos - dx1 - dx2,
              long_pos - dx1 + dx2,
              long_pos + dx1 + dx2]
    obs_y = [lat_pos + dy1 + dy2,
              lat_pos + dy1 - dy2,
              lat_pos - dy1 - dy2,
              lat_pos - dy1 + dy2,
              lat_pos + dy1 + dy2]

    num = num + 1
    obs_info_all[source]['obstacles_x_rel'].append(obs_x_rel)
    obs_info_all[source]['obstacles_y_rel'].append(obs_y_rel)
    obs_info_all[source]['pos_x_rel'].append(long_pos_rel)
    obs_info_all[source]['pos_y_rel'].append(lat_pos_rel)
    obs_info_all[source]['obstacles_vel'].append(obstacle_list[i].common_info.relative_velocity.x)
    obs_info_all[source]['obstacles_acc'].append(obstacle_list[i].common_info.relative_acceleration.x)
    obs_info_all[source]['obstacles_tid'].append(obstacle_list[i].additional_info.track_id)
#             fusion_obs_info['is_cipv'].append(obstacle_list[i].target_selection_type)
    obs_info_all[source]['obs_label'].append('v(' + str(obstacle_list[i].additional_info.track_id) + ')=' \
        + str(round(frenet_vs, 2))+','+ str(round(frenet_vl, 4)))
    obs_info_all[source]['obstacles_x'].append(obs_x)
    # for ind in range(len(obs_y)):
    obs_info_all[source]['obstacles_y'].append(obs_y)
    obs_info_all[source]['pos_x'].append(long_pos)
    obs_info_all[source]['pos_y'].append(lat_pos)

  obs_info = {
    'obstacles_x_rel': [],
    'obstacles_y_rel': [],
    'obstacles_x': [],
    'obstacles_y': [],
    'pos_x_rel': [],
    'pos_y_rel': [],
    'pos_x': [],
    'pos_y': [],
    'obstacles_vel': [],
    'obstacles_acc': [],
    'obstacles_tid': [],
    'is_cipv': [],
    'obs_label':[]
  }
  if (1 in obs_info_all.keys()) == False:
    obs_info_all[1] = obs_info
  if (4 in obs_info_all.keys()) == False:
    obs_info_all[4] = obs_info

  return obs_info_all

def load_obstacle_mobileye_params(obstacle_list):
  obstacles_mobileye_info = {
    'obstacles_x_rel': [],
    'obstacles_y_rel': [],
    'pos_x_rel': [],
    'pos_y_rel': [],
    'obstacles_vel': [],
    'obstacles_acc': [],
    'obstacles_id': [],
    'obs_label': []
  }
  obs_num = len(obstacle_list)
  for i in range(obs_num):
    # frenet_vs, frenet_vl = 255, 255
    half_length = obstacle_list[i].common_info.shape.length / 2
    half_width = obstacle_list[i].common_info.shape.width /2
    long_pos_rel = obstacle_list[i].common_info.relative_position.x + half_length
    lat_pos_rel = obstacle_list[i].common_info.relative_position.y
    theta = obstacle_list[i].common_info.relative_heading_angle
    if theta == 255:
      theta = 0
    cos_heading = math.cos(theta)
    sin_heading = math.sin(theta)
    dx1 = cos_heading * half_length
    dy1 = sin_heading * half_length
    dx2 = sin_heading * half_width
    dy2 = -cos_heading * half_width
    obs_x_rel = [long_pos_rel + dx1 + dx2,
              long_pos_rel + dx1 - dx2,
              long_pos_rel- dx1 - dx2,
              long_pos_rel - dx1 + dx2,
              long_pos_rel + dx1 + dx2]
    obs_y_rel = [lat_pos_rel + dy1 + dy2,
              lat_pos_rel + dy1 - dy2,
              lat_pos_rel - dy1 - dy2,
              lat_pos_rel - dy1 + dy2,
              lat_pos_rel + dy1 + dy2]
    obstacles_mobileye_info['obstacles_x_rel'].append(obs_x_rel)
    obstacles_mobileye_info['obstacles_y_rel'].append(obs_y_rel)
    obstacles_mobileye_info['pos_x_rel'].append(long_pos_rel)
    obstacles_mobileye_info['pos_y_rel'].append(lat_pos_rel)
    obstacles_mobileye_info['obstacles_vel'].append(obstacle_list[i].common_info.relative_velocity.x)
    obstacles_mobileye_info['obstacles_acc'].append(obstacle_list[i].common_info.relative_acceleration.x)
    obstacles_mobileye_info['obstacles_id'].append(obstacle_list[i].common_info.id)
    obstacles_mobileye_info['obs_label'].append('v(' + str(obstacle_list[i].common_info.id) + ')')  # ')=' \
        # + str(round(frenet_vs, 2))+','+ str(round(frenet_vl, 4)))

  return obstacles_mobileye_info

def draw_local_view_debug(dataLoader, layer_manager):
    #define figure
    # 定义 debug
    table_attr_list = ['Attr', 'Val']
    table_layer = TableLayerV2(None, table_attr_list, table_params)
    layer_manager.AddLayer(
      table_layer, 'table_layer', lat_behavior_table1, 'lat_behavior_table1', 3)
    pass
def draw_local_view(dataLoader, layer_manager):
    #define figure
    # 定义 local_view fig
    fig_local_view = bkp.figure(x_axis_label='y', y_axis_label='x', width=600, height=800, match_aspect = True, aspect_scale=1)
    fig_local_view.x_range.flipped = True
    # toolbar
    fig_local_view.toolbar.active_scroll = fig_local_view.select_one(WheelZoomTool)

    # 加载planning debug部分信息, 加载planning 输入topic的时间戳
    fix_lane_xys = []
    origin_lane_xys = []
    target_lane_xys = []
    global plan_debug_ts
    global plan_debug_timestamps
    global fusion_object_timestamps
    global fusion_road_timestamps
    global localization_timestamps
    global prediction_timestamps
    global vehicle_service_timestamps
    global control_output_timestamps
    global mobileye_lane_lines_timestamps
    global mobileye_objects_timestamps
    for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
      t = dataLoader.plan_debug_msg["t"][i]
      plan_debug_ts.append(t)
      plan_debug_timestamps.append(dataLoader.plan_debug_msg["timestamp"][i])
      input_topic_timestamp = plan_debug.input_topic_timestamp
      fusion_object_timestamp = input_topic_timestamp.fusion_object
      fusion_road_timestamp = input_topic_timestamp.fusion_road
      localization_timestamp = input_topic_timestamp.localization
      prediction_timestamp = input_topic_timestamp.prediction
      vehicle_service_timestamp = input_topic_timestamp.vehicle_service
      control_output_timestamp = input_topic_timestamp.control_output
      fusion_object_timestamps.append(fusion_object_timestamp)
      fusion_road_timestamps.append(fusion_road_timestamp)
      localization_timestamps.append(localization_timestamp)
      prediction_timestamps.append(prediction_timestamp)
      vehicle_service_timestamps.append(vehicle_service_timestamp)
      control_output_timestamps.append(control_output_timestamp)

    # 加载定位
    location_generator = CommonGenerator()
    cur_pos_xn0 = cur_pos_xn = dataLoader.loc_msg['data'][0].pose.local_position.x
    cur_pos_yn0 = cur_pos_yn = dataLoader.loc_msg['data'][0].pose.local_position.y
    for localization_timestamp in localization_timestamps:
      flag, loc_msg = find(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        # print('find loc_msg error')
        location_generator.xys.append(([],[]))
        continue
      cur_pos_xn = loc_msg.pose.local_position.x
      cur_pos_yn = loc_msg.pose.local_position.y
      cur_yaw = loc_msg.pose.euler_angles.yaw
      ego_xb, ego_yb = [], []
      ego_xn, ego_yn = [], []
      ### global variables
      # pos offset
      for i in range(len(dataLoader.loc_msg['data'])):
        if (i % 10 != 0): # 下采样 10
          continue
        pos_xn_i = dataLoader.loc_msg['data'][i].pose.local_position.x
        pos_yn_i = dataLoader.loc_msg['data'][i].pose.local_position.y

        ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

        ego_xb.append(ego_local_x)
        ego_yb.append(ego_local_y)
        ego_xn.append(pos_xn_i - cur_pos_xn0)
        ego_yn.append(pos_yn_i - cur_pos_yn0)
      location_generator.xys.append((ego_yb,ego_xb))
    location_generator.ts = np.array(plan_debug_ts)
    location_layer = CurveLayer(fig_local_view, location_params)
    layer_manager.AddLayer(location_layer, 'location_layer', location_generator, 'location_generator', 2)


    # 加载车道线
    lane_generator_dict = {}
    centerline_generator_dict = {}
    center_line_lists = []
    if dataLoader.road_msg['enable'] == True:
      for fusion_road_timestamp in fusion_road_timestamps:
        flag, fusion_road_msg = find(dataLoader.road_msg, fusion_road_timestamp)

        for derection in [0, 1]: # 0: left     1:right
          for index in range(5) :
            # 5代表5条车道
            fig_index = 5 * derection + index + 1
            lane_info = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
            lane_generator_key = 'lane_' + str(index) + '_' + str(derection)
            if (lane_generator_key in lane_generator_dict.keys()) == False:
              lane_generator_dict[lane_generator_key] = LineGenerator('line')
            if not flag:
              lane_generator_dict[lane_generator_key].xys.append(([] , [] ,[], []))
              continue
            if index < len(fusion_road_msg.lanes):
              lane = fusion_road_msg.lanes[index]
              if derection == 0:
                line = lane.left_lane_boundary
              elif derection == 1:
                line = lane.right_lane_boundary

              line_coef = line.poly_coefficient
              line_x, line_y = gen_line(line_coef[0], line_coef[1], line_coef[2], line_coef[3], \
                line.begin, line.end)
              lane_info['line_x_vec'] = line_x
              lane_info['line_y_vec'] = line_y
              tp = line.segment[0].type
              if tp == 0 or tp == 1 or tp == 3 or tp == 4:
                lane_info['type'] = ['dashed']
              else:
                lane_info['type'] = ['solid']
              lane_info['fix_index'] = [fig_index]
            else:
              line_x, line_y = gen_line(0,0,0,0,0,0)
              lane_info['line_x_vec'] = line_x
              lane_info['line_y_vec'] = line_y
              lane_info['type'] = ['dashed']
              lane_info['fix_index'] = [fig_index]
            lane_generator_dict[lane_generator_key].xys.append((lane_info['line_y_vec'] , lane_info['line_x_vec'] ,lane_info['type'], lane_info['fix_index']))

        # 加载车道中心线
        if flag:
          center_line_list = load_lane_center_lines(fusion_road_msg.lanes)
        for index in range(5):
          line_generator_key = 'centerline_' + str(index)
          fig_index = 10 + index + 1
          if (line_generator_key in centerline_generator_dict.keys()) == False:
            centerline_generator_dict[line_generator_key] = LineGenerator('center_line')
          if not flag:
            centerline_generator_dict[line_generator_key].xys.append(([] , [] ,[], []))
            continue
          centerline_generator_dict[line_generator_key].xys.append((center_line_list[index]['line_y_vec'], center_line_list[index]['line_x_vec'], [center_line_list[index]['relative_id']], [fig_index]))

      for lane_generator_key in lane_generator_dict.keys():
          lane_generator_dict[lane_generator_key].ts = np.array(plan_debug_ts)
          lane_layer = CurveLayer(fig_local_view, lane_params)
          layer_manager.AddLayer(lane_layer, lane_generator_key.replace('lane_', 'lane_layer_'), lane_generator_dict[lane_generator_key], lane_generator_key , 2)

      for line_generator_key in centerline_generator_dict.keys():
          centerline_generator_dict[line_generator_key].ts = np.array(plan_debug_ts)
          lane_layer = CurveLayer(fig_local_view, center_line_params)
          layer_manager.AddLayer(lane_layer, line_generator_key.replace('centerline_', 'centerline_layer_'), centerline_generator_dict[line_generator_key], line_generator_key , 2)


    # # 加载planning debug部分信息
    ego_info_generatev2 = TextGenerator()
    fix_lane_xys = []
    origin_lane_xys = []
    target_lane_xys = []
    for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
      flag, fusion_road_msg = find(dataLoader.road_msg, fusion_road_timestamps[i])
      if not flag:
        continue
      center_line_list = load_lane_center_lines(fusion_road_msg.lanes)
      lat_behavior_common = plan_debug.lat_behavior_common
      environment_model_info =plan_debug.environment_model_info
      current_lane_virtual_id = environment_model_info.currrent_lane_vitual_id
      fix_lane_ralative_id = lat_behavior_common.fix_lane_virtual_id - current_lane_virtual_id
      target_lane_ralative_id = lat_behavior_common.target_lane_virtual_id - current_lane_virtual_id
      origin_lane_ralative_id = lat_behavior_common.origin_lane_virtual_id - current_lane_virtual_id
      for j in range(5):
        if center_line_list[j]['relative_id'] == fix_lane_ralative_id:
          fix_lane_xys.append((center_line_list[j]['line_y_vec'], center_line_list[j]['line_x_vec']))
        if center_line_list[j]['relative_id'] == target_lane_ralative_id:
          target_lane_xys.append((center_line_list[j]['line_y_vec'], center_line_list[j]['line_x_vec']))
        if center_line_list[j]['relative_id'] == origin_lane_ralative_id:

          origin_lane_xys.append((center_line_list[j]['line_y_vec'], center_line_list[j]['line_x_vec']))
      text = '({})'.format(current_lane_virtual_id)
      ego_info_generatev2.xys.append(([0],[-3], [text]))
    fix_lane_generate = CommonGenerator()
    target_lane_generate = CommonGenerator()
    origin_lane_generate = CommonGenerator()
    fix_lane_generate.xys = fix_lane_xys
    target_lane_generate.xys = fix_lane_xys
    origin_lane_generate.xys = fix_lane_xys
    fix_lane_generate.ts = np.array(plan_debug_ts)
    target_lane_generate.ts = np.array(plan_debug_ts)
    origin_lane_generate.ts = np.array(plan_debug_ts)
    fix_lane_layer = CurveLayer(fig_local_view ,fix_lane_params)
    target_lane_layer = CurveLayer(fig_local_view ,target_lane_params)
    origin_lane_layer = CurveLayer(fig_local_view ,origin_lane_params)
    layer_manager.AddLayer(fix_lane_layer, 'fix_lane_layer', fix_lane_generate, 'fix_lane_generate', 2)
    layer_manager.AddLayer(target_lane_layer, 'target_lane_layer', target_lane_generate, 'target_lane_generate', 2)
    layer_manager.AddLayer(origin_lane_layer, 'origin_lane_layer', origin_lane_generate, 'origin_lane_generate', 2)

    ego_info_generatev2.ts = np.array(plan_debug_ts)
    ego_info_layerv2 = TextLayer(fig_local_view ,ego_info_paramsv2)
    layer_manager.AddLayer(ego_info_layerv2, 'ego_info_layerv2', ego_info_generatev2, 'ego_info_generatev2', 3)

    # # 加载自车信息
    ego_generate = CommonGenerator()
    car_xb, car_yb = load_car_params_patch()
    for i in range(len(plan_debug_ts)):
      ego_generate.xys.append(([car_yb],[car_xb]))
    ego_generate.ts = np.array(plan_debug_ts)
    ego_pose_layer = PatchLayer(fig_local_view ,ego_pose_params)
    layer_manager.AddLayer(ego_pose_layer, 'ego_pose_layer', ego_generate, 'ego_generate', 2)

    ego_info_generate = TextGenerator()
    for vehicle_service_timestamp in vehicle_service_timestamps:
      flag, vs_msg = find(dataLoader.vs_msg, vehicle_service_timestamp)
      if not flag:
        # print('find vs_msg error')
        ego_info_generate.xys.append(([0],[-2],['']))
        continue
      steer_deg = vs_msg.steering_wheel_angle * 57.3
      vel_ego = vs_msg.vehicle_speed
      text = 'v={:.2f}\nsteer={:.2f}'.format(round(vel_ego, 2), round(steer_deg, 2))
      ego_info_generate.xys.append(([0],[-2], [text]))
    ego_info_generate.ts = np.array(plan_debug_ts)
    ego_info_layer = TextLayer(fig_local_view ,ego_info_params)
    layer_manager.AddLayer(ego_info_layer, 'ego_info_layer', ego_info_generate, 'ego_info_generate', 3)


    # # # 加载障碍物
    obstacle_fusion_generate = CommonGenerator()
    obstacle_snrd_generate = CommonGenerator()
    obstacle_fusion_text_generate = TextGenerator()
    obstacle_snrd_text_generate = TextGenerator()
    for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
      flag, fus_msg = find(dataLoader.fus_msg, fusion_object_timestamps[i])
    # for i, fusion_object_timestamp in enumerate(fusion_object_timestamps):
      # flag, fus_msg = find(dataLoader.fus_msg, fusion_object_timestamp)
      # flag, fus_msg = find(dataLoader.fus_msg, fusion_object_timestamp)
      if not flag:
        # print('find fus_msg error')
        obstacle_fusion_generate.xys.append(([], []))
        obstacle_snrd_generate.xys.append(([], []))
        obstacle_fusion_text_generate.xys.append(([], [], []))
        obstacle_snrd_text_generate.xys.append(([], [], []))
        continue
      obstacles_info_all = load_obstacle_params(fus_msg.fusion_object, plan_debug.environment_model_info)
      obstacle_fusion_generate.xys.append((obstacles_info_all[1]['obstacles_y_rel'], obstacles_info_all[1]['obstacles_x_rel']))
      obstacle_snrd_generate.xys.append((obstacles_info_all[4]['obstacles_y_rel'], obstacles_info_all[4]['obstacles_x_rel']))
      obstacle_fusion_text_generate.xys.append((obstacles_info_all[1]['pos_y_rel'], obstacles_info_all[1]['pos_x_rel'], obstacles_info_all[1]['obs_label']))
      obstacle_snrd_text_generate.xys.append((obstacles_info_all[4]['pos_y_rel'], obstacles_info_all[4]['pos_x_rel'], obstacles_info_all[4]['obs_label']))

    obstacle_fusion_generate.ts = np.array(plan_debug_ts)
    obstacle_snrd_generate.ts = np.array(plan_debug_ts)
    obstacle_fusion_layer = PatchLayer(fig_local_view ,obstacle_fusion_params)
    obstacle_snrd_layer = PatchLayer(fig_local_view ,obstacle_snrd_params)
    layer_manager.AddLayer(obstacle_fusion_layer, 'obstacle_fusion_layer', obstacle_fusion_generate, 'obstacle_fusion_generate', 2)
    layer_manager.AddLayer(obstacle_snrd_layer, 'obstacle_snrd_layer', obstacle_snrd_generate, 'obstacle_snrd_generate', 2)

    obstacle_fusion_text_generate.ts = np.array(plan_debug_ts)
    obstacle_snrd_text_generate.ts = np.array(plan_debug_ts)
    obstacle_text_layer1 = TextLayer(fig_local_view, obstacle_text_params)
    obstacle_text_layer2 = TextLayer(fig_local_view, obstacle_text_params)
    layer_manager.AddLayer(obstacle_text_layer1, 'obstacle_text_layer1', obstacle_fusion_text_generate, 'obstacle_fusion_text_generate', 3)
    layer_manager.AddLayer(obstacle_text_layer2, 'obstacle_text_layer2', obstacle_snrd_text_generate, 'obstacle_snrd_text_generate', 3)

    # # # 加载预测
    prediction_generator = CommonGenerator()

    for index, prediction_timestamp in enumerate(prediction_timestamps):
      flag, prediction_msg = find(dataLoader.prediction_msg, prediction_timestamp)
      loc_flag, loc_msg = find(dataLoader.loc_msg, localization_timestamps[index])
      if not flag or not loc_flag:
        # print('find loc_msg error')
        prediction_generator.xys.append(([], []))
        continue

      # 定位的选择需要修改
      try:
        prediction_objects, trajectory_info = load_prediction_objects(prediction_msg.prediction_obstacle_list, loc_msg)
        prediction_generator.xys.append((trajectory_info['y'], trajectory_info['x']))
      except:
        prediction_generator.xys.append(([], []))
        pass

    prediction_generator.ts = np.array(plan_debug_ts)
    prediction_layer = MultiLinesLayer(fig_local_view, prediction_params)
    layer_manager.AddLayer(prediction_layer, 'prediction_layer', prediction_generator, 'prediction_generator', 2)

    # # # 加载plan轨迹
    plan_generator = CommonGenerator()
    coord_tf = coord_transformer()
    # print(dataLoader.plan_msg)
    # print(plan_debug_ts)
    for i, plan_debug_t in enumerate(plan_debug_ts):
      flag, plan_msg = find(dataLoader.plan_msg, plan_debug_timestamps[i])
    # for i, plan_msg in enumerate(dataLoader.plan_msg['data']):
      if not flag:
        # print('find plan error')
        plan_traj_x, plan_traj_y = [], []
      else:
        trajectory = plan_msg.trajectory
        try:
          planning_polynomial = trajectory.target_reference.polynomial
          plan_traj_x, plan_traj_y = gen_line(planning_polynomial[3],planning_polynomial[2], planning_polynomial[1], planning_polynomial[0], 0, 50)
        except:
          if dataLoader.loc_msg['enable'] == True:
            flag, loc_msg = find(dataLoader.loc_msg, localization_timestamps[i])
            if not flag:
              plan_traj_x, plan_traj_y = [], []
            else:
              cur_pos_xn = loc_msg.pose.local_position.x
              cur_pos_yn = loc_msg.pose.local_position.y
              cur_yaw = loc_msg.pose.euler_angles.yaw
              coord_tf.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)
              plan_x = []
              plan_y = []
              for i in range(len(trajectory.trajectory_points)):
                plan_x.append(trajectory.trajectory_points[i].x)
                plan_y.append(trajectory.trajectory_points[i].y)

              if trajectory.target_reference.lateral_maneuver_gear == 2:
                plan_traj_x = plan_x
                plan_traj_y = plan_y
              else:
                plan_traj_x, plan_traj_y = coord_tf.global_to_local(plan_x, plan_y)
          else:
            plan_traj_x, plan_traj_y = [], []
      plan_generator.xys.append((plan_traj_y, plan_traj_x))
    plan_generator.ts = np.array(plan_debug_ts)
    plan_layer = CurveLayer(fig_local_view, plan_params)
    layer_manager.AddLayer(plan_layer, 'plan_layer', plan_generator, 'plane_generator', 2)

    # # # 加载control轨迹

    control_generator = CommonGenerator()
    if dataLoader.ctrl_msg['enable'] == True:
      for control_output_timestamp in control_output_timestamps:
        flag, ctrl_msg = find(dataLoader.ctrl_msg, control_output_timestamp)
        if not flag:
          # print('find ctrl_msg error')
          control_generator.xys.append(([], []))
          continue
        control_result_points = ctrl_msg.control_trajectory.control_result_points
        mpc_dx = []
        mpc_dy = []
        for i in range(len(control_result_points)):
          mpc_dx.append(control_result_points[i].x)
          mpc_dy.append(control_result_points[i].y)
        control_generator.xys.append((mpc_dy, mpc_dx))
    control_generator.ts = np.array(plan_debug_ts)
    control_layer = CurveLayer(fig_local_view, control_params)
    layer_manager.AddLayer(control_layer, 'control_layer', control_generator, 'control_generator', 2)

    # legend
    fig_local_view.legend.click_policy = 'hide'

    plan_debug_table1 = TextGenerator()
    plan_debug_table2 = TextGenerator()
    for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
      plan_debug_json = dataLoader.plan_debug_msg['json'][i]
      names1 = []
      datas1 = []
      names2 = []
      datas2 = []
      names1.append('frame_num')
      datas1.append(plan_debug.frame_info.frame_num - dataLoader.plan_debug_msg['data'][0].frame_info.frame_num)
      names1.append('frame_duration_ms')
      datas1.append(plan_debug.frame_info.frame_duration_ms)

      names1.append('planning_succ')
      datas1.append(plan_debug.frame_info.planning_succ)

      names1.append('fusion_object_latency')
      datas1.append(plan_debug.input_topic_latency.fusion_object)
      names1.append('fusion_road_latency')
      datas1.append(plan_debug.input_topic_latency.fusion_road)
      names1.append('localization_latency')
      datas1.append(plan_debug.input_topic_latency.localization)

      names2.append('prediction_latency')
      datas2.append(plan_debug.input_topic_latency.prediction)
      names2.append('vehicle_service_latency')
      datas2.append(plan_debug.input_topic_latency.vehicle_service)
      names2.append('hmi_latency')
      datas2.append(plan_debug.input_topic_latency.hmi)

      names2.append('EnvironmentalModelManagerCost')
      datas2.append(plan_debug_json['EnvironmentalModelManagerCost'])
      names2.append('VisionLateralBehaviorPlannerCost')
      datas2.append(plan_debug_json['VisionLateralBehaviorPlannerCost'])
      names2.append('VisionLateralMotionPlannerCost')
      datas2.append(plan_debug_json['VisionLateralMotionPlannerCost'])
      names2.append('VisionLongitudinalBehaviorPlannerCost')
      datas2.append(plan_debug_json['VisionLongitudinalBehaviorPlannerCost'])

      plan_debug_table1.xys.append((names1, datas1, [None] * len(names1)))
      plan_debug_table2.xys.append((names2, datas2, [None] * len(names2)))
    plan_debug_table1.ts = np.array(plan_debug_ts)
    plan_debug_table2.ts = np.array(plan_debug_ts)
    tab_attr_list = ['Attr', 'Val']
    tab_debug_layer1 = TableLayerV2(None, tab_attr_list, table_params)
    tab_debug_layer2 = TableLayerV2(None, tab_attr_list, table_params)
    layer_manager.AddLayer(
      tab_debug_layer1, 'tab_debug_layer1', plan_debug_table1, 'plan_debug_table1', 3)
    layer_manager.AddLayer(
      tab_debug_layer2, 'tab_debug_layer2', plan_debug_table2, 'plan_debug_table2', 3)

    # 加载mobileye车道线
    mobileye_lane_lines_generator_dict = {}
    if dataLoader.mobileye_lane_lines_msg['enable'] == True:
      for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
        flag, mobileye_lane_lines_msg = findME(dataLoader.mobileye_lane_lines_msg, fusion_road_timestamps[i])
        for index in range(mobileye_lane_lines_msg.num) :
          mobileye_lane_info = {'line_x_vec':[], 'line_y_vec':[], 'type':[]}
          mobileye_lane_generator_key = 'mobileye_lane_' + str(index)
          if (mobileye_lane_generator_key in mobileye_lane_lines_generator_dict.keys()) == False:
            mobileye_lane_lines_generator_dict[mobileye_lane_generator_key] = LineGenerator('mobileye_line')
          if not flag:
            mobileye_lane_lines_generator_dict[mobileye_lane_generator_key].xys.append(([] , [] ,[], []))
            continue
          lane = mobileye_lane_lines_msg.lane_line[index]
          fig_index = lane.pos_type
          line_x, line_y = gen_line(lane.a0, lane.a1, lane.a2, lane.a3, lane.start, lane.end)
          mobileye_lane_info['line_x_vec'] = line_x
          mobileye_lane_info['line_y_vec'] = line_y
          tp = lane.marking_segments[0].marking
          if tp == 0 or tp == 1 or tp == 3 or tp == 4:
            mobileye_lane_info['type'] = ['dashed']
          else:
            mobileye_lane_info['type'] = ['solid']
          mobileye_lane_info['fix_index'] = [fig_index]
          mobileye_lane_lines_generator_dict[mobileye_lane_generator_key].xys.append((mobileye_lane_info['line_y_vec'] , \
                                                                                      mobileye_lane_info['line_x_vec'] , \
                                                                                      mobileye_lane_info['type'], \
                                                                                      mobileye_lane_info['fix_index']))
      for mobileye_lane_generator_key in mobileye_lane_lines_generator_dict.keys():
          mobileye_lane_lines_generator_dict[mobileye_lane_generator_key].ts = np.array(plan_debug_ts)
          mobileye_lane_layer = CurveLayer(fig_local_view, mobileye_lane_lines_params)
          layer_manager.AddLayer(mobileye_lane_layer, mobileye_lane_generator_key.replace('mobileye_lane_', 'mobileye_lane_layer_'), \
                                 mobileye_lane_lines_generator_dict[mobileye_lane_generator_key], \
                                 mobileye_lane_generator_key , 2)

    # 加载mobileye障碍物
    obstacle_mobileye_generate = CommonGenerator()
    obstacle_mobileye_text_generate = TextGenerator()
    for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
      flag, mobileye_objects_msg = findME(dataLoader.mobileye_objects_msg, fusion_object_timestamps[i])
      if not flag:
        # print('find mobileye_objects_msg error')
        obstacle_mobileye_generate.xys.append(([], []))
        obstacle_mobileye_text_generate.xys.append(([], [], []))
        continue
      # obstacles_mobileye_info = load_obstacle_params(mobileye_objects_msg.camera_perception_object_list, plan_debug.environment_model_info)
      obstacles_mobileye_info = load_obstacle_mobileye_params(mobileye_objects_msg.camera_perception_object_list)
      obstacle_mobileye_generate.xys.append((obstacles_mobileye_info['obstacles_y_rel'], obstacles_mobileye_info['obstacles_x_rel']))
      obstacle_mobileye_text_generate.xys.append((obstacles_mobileye_info['pos_y_rel'], obstacles_mobileye_info['pos_x_rel'], obstacles_mobileye_info['obs_label']))
    obstacle_mobileye_generate.ts = np.array(plan_debug_ts)
    obstacle_mobileye_layer = PatchLayer(fig_local_view ,obstacle_mobileye_params)
    layer_manager.AddLayer(obstacle_mobileye_layer, 'obstacle_mobileye_layer', obstacle_mobileye_generate, 'obstacle_mobileye_generate', 2)
    obstacle_mobileye_text_generate.ts = np.array(plan_debug_ts)
    obstacle_mobileye_text_layer = TextLayer(fig_local_view, obstacle_mobileye_text_params)
    layer_manager.AddLayer(obstacle_mobileye_text_layer, 'obstacle_mobileye_text_layer', obstacle_mobileye_text_generate, 'obstacle_mobileye_text_generate', 3)

    return fig_local_view, (tab_debug_layer1.plot, tab_debug_layer2.plot)

def apa_draw_local_view(dataLoader, layer_manager):
    #define figure
    # 定义 local_view fig
    fig_local_view = bkp.figure(x_axis_label='y', y_axis_label='x', width=1500, height=1000, match_aspect = True, aspect_scale=1)
    fig_local_view.x_range.flipped = True
    # toolbar
    fig_local_view.toolbar.active_scroll = fig_local_view.select_one(WheelZoomTool)
    # 加载planning debug部分信息, 加载planning 输入topic的时间戳
    fix_lane_xys = []
    origin_lane_xys = []
    target_lane_xys = []
    global plan_debug_ts
    global fusion_object_timestamps
    global fusion_road_timestamps
    global localization_timestamps
    global prediction_timestamps
    global vehicle_service_timestamps
    global control_output_timestamps
    global slot_timestamps
    for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
      t = dataLoader.plan_debug_msg["t"][i]
      plan_debug_ts.append(t)
      input_topic_timestamp = plan_debug.input_topic_timestamp
      fusion_object_timestamp = input_topic_timestamp.fusion_object
      fusion_road_timestamp = input_topic_timestamp.fusion_road
      localization_timestamp = input_topic_timestamp.localization
      prediction_timestamp = input_topic_timestamp.prediction
      vehicle_service_timestamp = input_topic_timestamp.vehicle_service
      control_output_timestamp = input_topic_timestamp.control_output
      slot_timestamp = input_topic_timestamp.parking_fusion
      fusion_object_timestamps.append(fusion_object_timestamp)
      fusion_road_timestamps.append(fusion_road_timestamp)
      localization_timestamps.append(localization_timestamp)
      prediction_timestamps.append(prediction_timestamp)
      vehicle_service_timestamps.append(vehicle_service_timestamp)
      control_output_timestamps.append(control_output_timestamp)
      slot_timestamps.append(slot_timestamp)
    # 加载定位
    location_generator = CommonGenerator()
    cur_pos_xn0 = cur_pos_xn = dataLoader.loc_msg['data'][0].pose.local_position.x
    cur_pos_yn0 = cur_pos_yn = dataLoader.loc_msg['data'][0].pose.local_position.y
    for localization_timestamp in localization_timestamps:
      flag, loc_msg = find(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        # print('find loc_msg error')
        location_generator.xys.append(([],[]))
        continue
      cur_pos_xn = loc_msg.pose.local_position.x
      cur_pos_yn = loc_msg.pose.local_position.y
      cur_yaw = loc_msg.pose.euler_angles.yaw
      ego_xb, ego_yb = [], []
      ego_xn, ego_yn = [], []
      ### global variables
      # pos offset
      for i in range(len(dataLoader.loc_msg['data'])):
        if (i % 10 != 0): # 下采样 10
          continue
        pos_xn_i = dataLoader.loc_msg['data'][i].pose.local_position.x
        pos_yn_i = dataLoader.loc_msg['data'][i].pose.local_position.y

        # ego_local_x, ego_local_y= global2local(pos_xn_i, pos_yn_i, cur_pos_xn, cur_pos_yn, cur_yaw)

        ego_xb.append(pos_xn_i)
        ego_yb.append(pos_yn_i)
        ego_xn.append(pos_xn_i - cur_pos_xn0)
        ego_yn.append(pos_yn_i - cur_pos_yn0)
      location_generator.xys.append((ego_yb,ego_xb))
    location_generator.ts = np.array(plan_debug_ts)
    location_layer = CurveLayer(fig_local_view, location_params_apa)
    layer_manager.AddLayer(location_layer, 'location_layer', location_generator, 'location_generator', 2)


    # 加载自车信息
    ego_polygon_generate = CommonGenerator()
    ego_pose_generate = CommonGenerator()
    # for i in range(len(plan_debug_ts)):
    #   ego_generate.xys.append(([car_yb],[car_xb]))
    for localization_timestamp in localization_timestamps:
      flag, loc_msg = find(dataLoader.loc_msg, localization_timestamp)
      if not flag:
        # print('find loc_msg error')
        # location_generator.xys.append(([],[]))
        continue
      cur_pos_xn = loc_msg.pose.local_position.x
      cur_pos_yn = loc_msg.pose.local_position.y
      cur_pos_theta = loc_msg.pose.euler_angles.yaw
      temp_cur_pos_xn = []
      temp_cur_pos_yn = []
      temp_cur_pos_xn.append(cur_pos_xn)
      temp_cur_pos_yn.append(cur_pos_yn)
      ego_box = get_closed_veh_box(cur_pos_xn,cur_pos_yn,cur_pos_theta)
      ego_box_for_ego_point = get_closed_veh_box_for_ego_point(cur_pos_xn,cur_pos_yn,cur_pos_theta)
      ego_polygon_generate.xys.append(([ego_box[1]],[ego_box[0]]))
      ego_pose_generate.xys.append(([ego_box_for_ego_point[1]],[ego_box_for_ego_point[0]]))
    ego_polygon_generate.ts = np.array(plan_debug_ts)
    ego_pose_generate.ts = np.array(plan_debug_ts)
    ego_pose_polygon_layer = PatchLayer(fig_local_view ,ego_pose_polygon_params_apa)
    ego_pose_layer = PatchLayer(fig_local_view ,ego_pose_params_apa)
    layer_manager.AddLayer(ego_pose_polygon_layer, 'ego_pose_polygon_layer', ego_polygon_generate, 'ego_polygon_generate', 2)
    layer_manager.AddLayer(ego_pose_layer, 'ego_pose_layer', ego_pose_generate, 'ego_pose_generate', 2)

    ###加载apa车位
    slot_generate = CommonGenerator()
    slot_id_generate = TextGenerator()
    for slot_timestamp in slot_timestamps:
        flag, slot_msg = find(dataLoader.slot_msg, slot_timestamp)
        if not flag:
            print('find slot_msg error')
            slot_generate.xys.append(([], []))
            slot_id_generate.xys.append(([],[]))
            continue
        temp_corner_x_list = []
        temp_corner_y_list = []
        temp_slot_id_list = []
        temp_slot_id_x_list = []
        temp_slot_id_y_list = []
        for slot in slot_msg.parking_fusion_slot_lists:
            temp_corner_x = []
            temp_corner_y = []
            for corner_point in slot.corner_points:
                temp_corner_x.append(corner_point.x)
                temp_corner_y.append(corner_point.y)
            temp_x = temp_corner_x[2]
            temp_y = temp_corner_y[2]
            temp_corner_x[2] = temp_corner_x[3]
            temp_corner_y[2] = temp_corner_y[3]
            temp_corner_x[3] = temp_x
            temp_corner_y[3] = temp_y
            temp_corner_x_list.append(temp_corner_x)
            temp_corner_y_list.append(temp_corner_y)
            # add slot id
            temp_slot_id = slot.id
            text = 'id={:.2f}'.format(round(temp_slot_id, 2))
            temp_slot_id_list.append(text)
            temp_slot_id_x_list.append((temp_corner_x[0]+temp_corner_x[3])/2)
            temp_slot_id_y_list.append((temp_corner_y[0]+temp_corner_y[1])/2)
        slot_generate.xys.append((temp_corner_y_list, temp_corner_x_list))
        slot_id_generate.xys.append((temp_slot_id_y_list,temp_slot_id_x_list,temp_slot_id_list))
    slot_generate.ts = np.array(plan_debug_ts)
    slot_id_generate.ts = np.array(plan_debug_ts)
    slot_layer = PatchLayer(fig_local_view ,slot_params_apa)
    slot_id_layer = TextLayer(fig_local_view,slot_id_params_apa)
    layer_manager.AddLayer(slot_layer, 'slot_layer', slot_generate, 'slot_generate', 2)
    layer_manager.AddLayer(slot_id_layer, 'slot_id_layer',slot_id_generate,'slot_id_generate',3)

    ###加载apa规划plan
    plan_generator = CommonGenerator()
    coord_tf = coord_transformer()

    for plan_debug_t in plan_debug_ts:
      flag, plan_msg = find(dataLoader.plan_msg, plan_debug_t)
      if not flag:
            # print('find plan_msg error')
            slot_generate.xys.append(([], []))
            continue
    # for i, plan_msg in enumerate(dataLoader.plan_msg['data']):
      trajectory = plan_msg.trajectory
      trajectory_points_x = []
      trajectory_points_y = []
      try:
         trajectory_points = trajectory.trajectory_points
         for point in trajectory_points:
            trajectory_points_x.append(point.x)
            trajectory_points_y.append(point.y)
            print(point.x,point.y,sep=':')
            print("运行到此!")
      except:
        print("plan points err!")
      plan_generator.xys.append(([trajectory_points_y], [trajectory_points_x]))
    plan_generator.ts = np.array(plan_debug_ts)
    plan_layer = CurveLayer(fig_local_view, plan_params)
    layer_manager.AddLayer(plan_layer, 'plan_layer', plan_generator, 'plane_generator', 2)
    # legend
    fig_local_view.legend.click_policy = 'hide'
    return fig_local_view

def get_closed_veh_box(x, y, theta):
    # params for E40X
    # length = 4.41
    # width = 1.8
    # shift_dis = 1.325

    # params for AIONLX
    length = 4.786
    width = 1.935
    shift_dis = (3.846 - 0.94) * 0.5

    half_length = length * 0.5
    half_width = width * 0.5
    cos_ego_start_theta = math.cos(theta)
    sin_ego_start_theta = math.sin(theta)
    shift_ego_start_x = x + shift_dis * cos_ego_start_theta
    shift_ego_start_y = y + shift_dis * sin_ego_start_theta
    dx1 = cos_ego_start_theta * half_length
    dy1 = sin_ego_start_theta * half_length
    dx2 = sin_ego_start_theta * half_width
    dy2 = -cos_ego_start_theta * half_width
    pt0_x = shift_ego_start_x + dx1 + dx2
    pt0_y = shift_ego_start_y + dy1 + dy2
    pt1_x = shift_ego_start_x + dx1 - dx2
    pt1_y = shift_ego_start_y + dy1 - dy2
    pt2_x = shift_ego_start_x - dx1 - dx2
    pt2_y = shift_ego_start_y - dy1 - dy2
    pt3_x = shift_ego_start_x - dx1 + dx2
    pt3_y = shift_ego_start_y - dy1 + dy2
    return [[pt0_x, pt1_x, pt2_x, pt3_x, pt0_x], [pt0_y, pt1_y, pt2_y, pt3_y, pt0_y]]

def get_closed_veh_box_for_ego_point(x, y, theta):
    # params for E40X
    # length = 4.41
    # width = 1.8
    # shift_dis = 1.325

    # params for AIONLX
    scale = 3    #缩小的倍数，将自车的polygon缩小n倍后作为后轴中心点
    length = 4.786 / scale
    width = 1.935 / scale
    shift_dis = (3.846 - 0.94) * 0.5 / scale

    half_length = length * 0.5 / scale
    half_width = width * 0.5 / scale
    cos_ego_start_theta = math.cos(theta)
    sin_ego_start_theta = math.sin(theta)
    shift_ego_start_x = x + shift_dis * cos_ego_start_theta
    shift_ego_start_y = y + shift_dis * sin_ego_start_theta
    dx1 = cos_ego_start_theta * half_length
    dy1 = sin_ego_start_theta * half_length
    dx2 = sin_ego_start_theta * half_width
    dy2 = -cos_ego_start_theta * half_width
    pt0_x = shift_ego_start_x + dx1 + dx2
    pt0_y = shift_ego_start_y + dy1 + dy2
    pt1_x = shift_ego_start_x + dx1 - dx2
    pt1_y = shift_ego_start_y + dy1 - dy2
    pt2_x = shift_ego_start_x - dx1 - dx2
    pt2_y = shift_ego_start_y - dy1 - dy2
    pt3_x = shift_ego_start_x - dx1 + dx2
    pt3_y = shift_ego_start_y - dy1 + dy2
    return [[pt0_x, pt1_x, pt2_x, pt3_x, pt0_x], [pt0_y, pt1_y, pt2_y, pt3_y, pt0_y]]

def GenerateJsonValueData(json_data, json_time_list, json_value_list):
    json_value_xys_dict = {}
    json_value_xys_dict['t'] = json_time_list

    for i in range(len(json_value_list)):
        tmp = []
        for j in range(len(json_data)):
            try:
                scale = 1.0
                if json_value_list[i] == "throttle_brake":
                    if json_data[j][json_value_list[i]]>0:
                       scale = 0.001
                    else:
                       scale = 1.0
                tmp.append(json_data[j][json_value_list[i]] * scale)
            except:
                tmp.append(0.0)
        json_value_xys_dict[json_value_list[i]] = tmp

    return json_value_xys_dict

def GenerateJsonVectorData(json_data, json_time_list, json_vector_list):
    json_vector_xys_dict = {}
    json_vector_xys_dict['t'] = json_time_list

    for i in range(len(json_vector_list)):
        tmp = []
        for j in range(len(json_data)):
            try:
                tmp.append(json_data[j][json_vector_list[i]])
            except:
                tmp.append(0.0)
        json_vector_xys_dict[json_vector_list[i]] = tmp

    return json_vector_xys_dict

def  GenerateTopicVectorData(topic_data,topic_time_list,topic_value_list):
    topic_vector_dict = {}
    topic_vector_dict['t'] = topic_time_list
    # print(type(tmp[i]))
    # lon_tmp = []
    # lon_tmp1 = []
    # lon_tmp2 = []
    # lon_tmp3 = []
    # lon_tmp4 = []
    # lon_tmp5 = []
    lat_tmp6 = []
    lat_tmp7 = []
    lat_tmp8 = []
    lat_tmp9 = []
    lat_tmp10 = []
    for j in range(len(topic_data)):
        try:
            # lon_tmp.append(topic_data[j].lon_mpc_input.s_ref_mpc_vec)
            # lon_tmp1.append(topic_data[j].lon_mpc_output.s_mpc_vec)
            # lon_tmp2.append(topic_data[j].lon_mpc_input.v_ref_mpc_vec)
            # lon_tmp3.append(topic_data[j].lon_mpc_output.v_mpc_vec)
            # lon_tmp4.append(topic_data[j].lon_mpc_output.a_mpc_vec)
            # lon_tmp5.append(topic_data[j].lon_mpc_output.jerk_mpc_vec)
            lat_tmp6.append(topic_data[j].lat_mpc_input.dy_ref_mpc_vec)
            lat_tmp7.append(topic_data[j].lat_mpc_input.dphi_ref_mpc_vec)
            lat_tmp8.append(topic_data[j].lat_mpc_output.dy_mpc_vec)
            lat_tmp9.append(topic_data[j].lat_mpc_output.dphi_mpc_vec)
            lat_tmp10.append(topic_data[j].lat_mpc_output.delta_mpc_vec)
        except:
            # lon_tmp.append(0.0)
            # lon_tmp1.append(0.0)
            # lon_tmp2.append(0.0)
            # lon_tmp3.append(0.0)
            # lon_tmp4.append(0.0)
            # lon_tmp5.append(0.0)
            lat_tmp6.append(0.0)
            lat_tmp7.append(0.0)
            lat_tmp8.append(0.0)
            lat_tmp9.append(0.0)
            lat_tmp10.append(0.0)
            
    # topic_vector_dict[topic_value_list[0]] = lon_tmp
    # topic_vector_dict[topic_value_list[1]] = lon_tmp1
    # topic_vector_dict[topic_value_list[2]] = lon_tmp2
    # topic_vector_dict[topic_value_list[3]] = lon_tmp3
    # topic_vector_dict[topic_value_list[4]] = lon_tmp4
    # topic_vector_dict[topic_value_list[5]] = lon_tmp5
    topic_vector_dict[topic_value_list[0]] = lat_tmp6
    topic_vector_dict[topic_value_list[1]] = lat_tmp7
    topic_vector_dict[topic_value_list[2]] = lat_tmp8
    topic_vector_dict[topic_value_list[3]] = lat_tmp9
    topic_vector_dict[topic_value_list[4]] = lat_tmp10

    return topic_vector_dict

class ScalarGeneratorFromJson(DataGeneratorBase):
    def __init__(self, json_value_xys_dict, name, scale = 1):
        ts = json_value_xys_dict['t']
        tmp = json_value_xys_dict[name]
        ys = [x * scale for x in tmp]
        xys = []
        xys.append((ts, ys))
        super().__init__(xys, ts)
        self.y_range = [min(ys), max(ys)]
        
class ScalarSomeGeneratorFromJson(DataGeneratorBase):
    def __init__(self, json_value_xys_dict, name, scale = 1, condition = 0.0, cond_scale = 1000):
        ts = json_value_xys_dict['t']
        tmp = json_value_xys_dict[name]
        ys = []
        for x in tmp:
          if x > condition:
            x = x / cond_scale
          ys.append(x)
        xys = []
        xys.append((ts, ys))
        super().__init__(xys, ts)
        self.y_range = [min(ys), max(ys)]

class VectorGeneratorFromJson(DataGeneratorBase):
    def __init__(self, json_vector_xys_dict, name, scale = 1):
        ts = json_vector_xys_dict['t']
        t_vec = []
        t = 0

        for i in range(26):
            t_vec.append(t)
            t = t + 0.1

        xys = []
        tmp = json_vector_xys_dict[name]
        for i in range(len(tmp)):
           ys = [x * scale for x in tmp[i]]
           xys.append((t_vec, ys))

        super().__init__(xys, ts)
        self.xys = xys
        self.y_range = [min(ys), max(ys)]

class FigureLayer():
    def __init__(self, fig):
        self.count = 0
        self.fig = fig
        self.y_range = [0, 0]

    def AddCurv(self, layer_manager, data, name, type = 0):
        param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'red', 'line_dash': 'solid'}
        if self.count == 0:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'red', 'line_dash': 'solid'}
        elif self.count == 1:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'blue', 'line_dash': 'solid'}
        elif self.count == 2:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'green', 'line_dash': 'solid'}
        elif self.count == 3:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'black', 'line_dash': 'solid'}
        elif self.count == 4:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'red', 'line_dash': 'dashed'}
        elif self.count == 5:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'yellow', 'line_dash': 'dashed'}


        # curve layers
        curv_layer = CurveLayer(self.fig, param)
        name = param['legend_label']

        self.y_range[0] = min(self.y_range[0], data.y_range[0])
        self.y_range[1] = max(self.y_range[1], data.y_range[1])

        # add layers
        if type == 0:
            layer_manager.AddLayer(curv_layer, name, data)
        else:
            layer_manager.AddLayer(curv_layer, name, data, name, 2)

        self.fig.legend.click_policy = "hide"
        self.fig.toolbar.active_scroll = self.fig.select_one(WheelZoomTool)

        self.count = self.count + 1

class FigureLayerHover():
    def __init__(self, fig):
        self.count = 0
        self.fig = fig
        self.y_range = [0, 0]
        # self.tooltips_list = [('time', '@pts_xs'), (f'{name}', f'@pts_ys')]

    def AddCurv(self, layer_manager, data, name, type = 0, last_line = False):
        param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'red', 'line_dash': 'solid'}
        if self.count == 0:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'red', 'line_dash': 'solid'}
            # self.tooltips_list.append((f'{name}', f'@pts_ys'))
        elif self.count == 1:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'blue', 'line_dash': 'solid'}
            # self.tooltips_list.append((f'{name}', f'@pts_ys'))
        elif self.count == 2:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'green', 'line_dash': 'solid'}
            # self.tooltips_list.append((f'{name}', f'@pts_ys'))
        elif self.count == 3:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'black', 'line_dash': 'solid'}
            # self.tooltips_list.append((f'{name}', f'@pts_ys'))
        elif self.count == 4:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'red', 'line_dash': 'dashed'}
            # self.tooltips_list.append((f'{name}', f'@pts_ys'))
        elif self.count == 5:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'yellow', 'line_dash': 'dashed'}
            # self.tooltips_list.append((f'{name}', f'@pts_ys'))

        # curve layers
        curv_layer = CurveLayer(self.fig, param)
        name = param['legend_label']

        self.y_range[0] = min(self.y_range[0], data.y_range[0])
        self.y_range[1] = max(self.y_range[1], data.y_range[1])

        # add layers
        if type == 0:
            layer_manager.AddLayer(curv_layer, name, data)
        else:
            layer_manager.AddLayer(curv_layer, name, data, name, 2)
        
        if last_line == True:
          tooltips_list = [('time', '@pts_xs'), (f'{name}', f'@pts_ys')]
        else:
          tooltips_list = [(f'{name}', f'@pts_ys')]
        hover = HoverTool(renderers=[curv_layer.plot], tooltips=tooltips_list, mode='vline')
        self.fig.add_tools(hover)
        
        self.fig.legend.click_policy = "hide"
        self.fig.toolbar.active_scroll = self.fig.select_one(WheelZoomTool)

        self.count = self.count + 1

class DynamicFigureLayer():
    def __init__(self, fig):
        self.count = 0
        self.fig = fig

    def AddCurv(self, layer_manager, data, name, type = 0):
        param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'red', 'line_dash': 'solid'}
        if self.count == 0:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'red', 'line_dash': 'solid'}
        elif self.count == 1:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'blue', 'line_dash': 'solid'}
        elif self.count == 2:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'green', 'line_dash': 'solid'}
        elif self.count == 3:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'black', 'line_dash': 'solid'}
        elif self.count == 4:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'red', 'line_dash': 'dashed'}
        elif self.count == 5:
            param = {'legend_label': name, 'line_width': 1.5, 'line_color': 'yellow', 'line_dash': 'dashed'}

        curv_layer = CurveLayer(self.fig, param)
        layer_manager.AddLayer(curv_layer, name, data, name, 2)

        self.fig.toolbar.active_scroll = self.fig.select_one(WheelZoomTool)
        self.fig.legend.click_policy = "hide"

        self.count = self.count + 1



def printHelp():
    print('''\n
USAGE:
    1. <jupyter mode>      change “bag_path” and "html_file" and run
    2. <single file mode>  python3 plot_bag.py bag_file html_file
    3. <folder batch mode> python3 plot_bag.py bag_folder html_folder
\n''')