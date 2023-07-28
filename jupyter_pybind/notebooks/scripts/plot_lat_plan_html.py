

import sys
import os
from abc import ABC, abstractmethod
import bokeh.plotting as bkp
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs
from bokeh.io import output_notebook, push_notebook, output_file, export_png
from bokeh.layouts import layout, column, row
from bokeh.plotting import figure, output_file, show, ColumnDataSource

import numpy as np
from IPython.core.display import display, HTML
from plot_local_view_html import *
import logging
sys.path.append('..')
sys.path.append('../..')
sys.path.append('../../..')
from lib.basic_layers import *
from lib.bag_loader import *

# 先手动写死bag

bag_path = "/share/mnt/0713/long_time_1.00000"
html_file = bag_path +".lat_plan.html"

# bokeh创建的html在jupyter中显示
if isINJupyter():
    display(HTML("<style>.container { width:95% !important;  }</style>"))
    display(
        HTML('''<style>.widget-label {min-width: 25ex !important; }</style>'''))
    output_notebook()


table_params={
    'width': 300,
    'height':800,
}

def isINJupyter():
    try:
        __file__
    except NameError:
        return True
    else:
        return False
# 障碍物id的文本框的回调函数
def obj_id_handler(dataLoader, id):
  global obj_id
  obj_id = id
  if dataLoader.plan_debug_msg['enable'] == True:
    environment_model_info = dataLoader.plan_debug_msg['data'][plan_debug_msg_idx].environment_model_info
    obj_vars = ['id','s','l','s_to_ego','max_l_to_ref','min_l_to_ref','nearest_l_to_desire_path', \
            'nearest_l_to_ego', 'vs_lat_relative','vs_lon_relative','vs_lon',
              'nearest_y_to_desired_path','is_accident_car','is_accident_cnt','is_avoid_car','is_lane_lead_obstacle',
              'current_lead_obstacle_to_ego']

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

def draw_lat(dataLoader, layer_manager):
  lat_behavior_table1 = TextGenerator()
  lat_behavior_table2 = TextGenerator()
  plan_debug_ts = []
  # 1. 可视化横向状态机debug信息
  for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
      t = dataLoader.plan_debug_msg["t"][i]
      plan_debug_ts.append(t)

      vo_lat_motion_plan = plan_debug.vo_lat_motion_plan
      lat_behavior_common = plan_debug.lat_behavior_common
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
      lat_behavior_table1.xys.append((names, datas, [None] * len(names)))

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
      lat_behavior_table2.xys.append((names, datas, [None] * len(names)))
  lat_behavior_table1.ts = plan_debug_ts
  lat_behavior_table2.ts = plan_debug_ts


  tab_attr_list = ['Attr', 'Val']
  tab_rt_layer1 = TableLayerV2(None, tab_attr_list, table_params)
  layer_manager.AddLayer(
      tab_rt_layer1, 'rt_table_source1', lat_behavior_table1, 'lat_behavior_table1', 3)

  tab_rt_layer2 = TableLayerV2(None, tab_attr_list, table_params)
  layer_manager.AddLayer(
      tab_rt_layer2, 'rt_table_source2', lat_behavior_table2, 'lat_behavior_table2', 3)


  # 2. 可视化障碍物数据debug信息
  obj_vars = ['id','s','l','s_to_ego','max_l_to_ref','min_l_to_ref','nearest_l_to_desire_path', \
            'nearest_l_to_ego','vs_lon',
              'nearest_y_to_desired_path','is_accident_car','is_accident_cnt','is_avoid_car','is_lane_lead_obstacle',
              'current_lead_obstacle_to_ego']
  # 'vs_lat_relative','vs_lon_relative'
  obstacle_ids = set()
  obstacle_generates = {}
  # 加载所有障碍物的id
  for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
    environment_model_info = plan_debug.environment_model_info
    for obstacle in environment_model_info.obstacle:
      obstacle_ids.add(obstacle.id)

  for obstacle_id in obstacle_ids:
      obstacle_generates['obstacle_generate_table_' + str(obstacle_id)] = ObjTextGenerator()
      obstacle_generates['obstacle_generate_table_' + str(obstacle_id)].ts = plan_debug_ts

  # 每一个障碍物id对应一个 Generator
  global fusion_object_timestamps
  for i, plan_debug in enumerate(dataLoader.plan_debug_msg['data']):
    flag, fus_msg = find(dataLoader.fus_msg, fusion_object_timestamps[i])
    environment_model_info = plan_debug.environment_model_info
    for obstacle_id in obstacle_ids:

      obstacle_generate = obstacle_generates['obstacle_generate_table_' + str(obstacle_id)]
      names  = []
      datas = []
      for obstacle in environment_model_info.obstacle:
        if obstacle_id == obstacle.id:
          for name in obj_vars:
            try:
              # print(getattr(obstacle,name))
              datas.append(getattr(obstacle,name))
              names.append(name)
            except:
              pass
          # 加载对应的笛卡尔下的障碍物速度
          for obj in fus_msg.fusion_object:
            if obstacle_id == obj.additional_info.track_id:
              names.append('v_x')
              names.append('v_y')
              datas.append(obj.common_info.relative_velocity.x)
              datas.append(obj.common_info.relative_velocity.y)
              break
          break
        else:
          obstacle_generate.xys.append((names, datas, [None] * len(names)))
          pass
        obstacle_generate.xys.append((names, datas, [None] * len(names)))
  tab_attr_list = ['Attr', 'Val']
  tab_lat_rt_obstacle_layer = TableLayerV2(None, tab_attr_list, table_params)
  lat_rt_obstacle_table = ObjTextGenerator()
  lat_rt_obstacle_table.ts = plan_debug_ts
  names  = ['']
  datas = ['']
  for i,_ in enumerate(plan_debug_ts):
    lat_rt_obstacle_table.xys.append((names, datas, [None] * len(names)))
  layer_manager.AddLayer(
      tab_lat_rt_obstacle_layer, 'lat_rt_obstacle_table_source', lat_rt_obstacle_table, 'lat_rt_obstacle_table', 3)
  return tab_rt_layer1.plot, tab_rt_layer2.plot, tab_lat_rt_obstacle_layer.plot, obstacle_generates

def plotOnce(bag_path, html_file):
    # 加载bag
    try:
        dataLoader = LoadCyberbag(bag_path)
    except:
        print('load cyber_bag error!')
        return
    max_time = dataLoader.load_all_data(False)
    layer_manager = LayerManager()

    fig_local_view = draw_local_view(dataLoader, layer_manager)
    tab_rt1, tab_rt2,tab_lat_rt_obstacle, obstacle_generates = draw_lat(dataLoader, layer_manager)
    min_t = sys.maxsize
    max_t = 0
    for gdlabel in layer_manager.gds.keys():
        gd = layer_manager.gds[gdlabel]
        min_t = min(min_t, gd.getMinT())
        max_t = max(max_t, gd.getMaxT())

    data_tmp = {'mt': [min_t]}

    for gdlabel in layer_manager.data_key.keys():
        gd = layer_manager.gds[gdlabel]
        data_label = layer_manager.data_key[gdlabel]
        data_tmp[data_label+'s'] = [gd.xys]
        data_tmp[data_label+'ts'] = [gd.ts]

    for obstacle_generate_key in obstacle_generates.keys():
      data_tmp[obstacle_generate_key+'s'] = [obstacle_generates[obstacle_generate_key].xys]
      data_tmp[obstacle_generate_key+'ts'] = [obstacle_generates[obstacle_generate_key].ts]

    bag_data = ColumnDataSource(data=data_tmp)

    callback_arg = slider_callback_arg(bag_data)
    for layerlabels in layer_manager.layers.keys():
        callback_arg.AddSource(layerlabels, layer_manager.layers[layerlabels])
    # find the front one of ( the first which is larger than k)
    binary_search = """
            function binarySearch(ts, k){
                if(ts.length == 0){
                    return 0;
                }
                var left = 0;
                var right = ts.length -1;
                while(left<right){
                    var mid = Math.floor((left + right) / 2);
                    if(ts[mid]<=k){ //if the middle value is less than or equal to k
                        left = mid + 1; //set the left value to the middle value + 1
                    }else{
                        right = mid; //set the right value to the middle value
                    }
                }
                if(left == 0){ //if the left value is 0
                    return 0; //return 0
                }
                return left-1; //return the left value - 1
            }
    """

    car_slider = Slider(start=0, end=max_time-0,
                        value=0, step=0.01, title="time")
    code0 = """
    %s
            console.log("cb_objS");
            console.log(cb_obj);
            console.log("cb_obj.value");
            console.log(cb_obj.value);
            console.log("bag_source");
            console.log(bag_source);
            console.log("bag_source.data");
            console.log(Object.keys(bag_source.data));
            const step = cb_obj.value;
            const data = bag_source.data;

    """

    codes = (layer_manager.code) % (code0) % (binary_search)
    callback = CustomJS(args=callback_arg.arg, code=codes)

    car_slider.js_on_change('value', callback)

    for gdlabel in layer_manager.gds.keys():
        gd = layer_manager.gds[gdlabel]
        if gdlabel is 'ep_source' or gdlabel is 'ep_source2' or gdlabel.startswith('global'):
            gd_frame = gd.atT(max_t)

        else:
            gd_frame = gd.atT(min_t)
        if gdlabel is 'online_obj_source' or gdlabel is 'onlinel_obj_source':
            layer_manager.layers[gdlabel].update(
                gd_frame[0], gd_frame[1], gd_frame[2])
            # print(gd_frame)
        elif gdlabel is 'cfb_source':
            pass
        else:
            if layer_manager.plotdim[gdlabel] == 3:
                layer_manager.layers[gdlabel].update(
                    gd_frame[0], gd_frame[1], gd_frame[2])
            else:
                layer_manager.layers[gdlabel].update(gd_frame[0], gd_frame[1])

    output_file(html_file)

    if isINJupyter():
        # display in jupyter notebook
        output_notebook()

    # pan_lt = Panel(child=row(column(fig_local_view, fig_sv), column(fig_tp, fig_tv, fig_ta, fig_tj)), title="Longtime")
    # pan_rt = Panel(child=row(tab_rt, column(fig_rtv)), title="Realtime")
    # pans = Tabs(tabs=[ pan_lt, pan_rt ])
    bkp.show(layout(car_slider, row(fig_local_view, tab_lat_rt_obstacle, tab_rt1, tab_rt2)))


def printHelp():
    print('''\n
USAGE:
    1. <jupyter mode>      change “bag_path” and "html_file" and run
    2. <single file mode>  python3 plot_bag.py bag_file html_file
    3. <folder batch mode> python3 plot_bag.py bag_folder html_folder
\n''')


def plotMain():
    # print('sys.argv = ', sys.argv)

    if(len(sys.argv) == 2):
        bag_path = str(sys.argv[1])
        html_file = bag_path +".html"

    else:
        bag_path = str(sys.argv[1])
        html_file = str(sys.argv[2])

    bag_path = sys.argv[1]
    html_path = bag_path

    # print("bag_path: {}\nhtml_path: {}".format(bag_path, html_path))

    if os.path.isfile(bag_path) and (not os.path.isdir(html_path)):
        print("process one bag ...")
        html_file = bag_path + ".lat_plan" + ".html"
        plotOnce(bag_path, html_file)
        return

    if (not os.path.isdir(bag_path)) or (not os.path.isdir(html_path)):
        print("INVALID ARGV:\n bag_path: {}\nhtml_path: {}".format(
            bag_path, html_path))
        return

    if not os.path.exists(html_path):
        os.makedirs(html_path)

    all_bag_files = os.listdir(bag_path)
    print("find {} files".format(len(all_bag_files)))
    generated_count = 0
    for bag_name in all_bag_files:
        if (".0000" in bag_name) and (bag_name.find(".html") == -1):
            print("process {} ...".format(bag_name))
            bag_file = os.path.join(bag_path, bag_name)
            html_file = bag_file + ".lat_plan" + ".html"
            try:
                plotOnce(bag_file, html_file)
                print("html_file = ", html_file)
                generated_count += 1
            except Exception:
                print('failed')

    print("{} html files generated\n".format(generated_count))

if __name__ == '__main__':
    if isINJupyter():
        plotOnce(bag_path, html_file)
    else:
        plotMain()
