#
def init():  # 初始化
  global global_dict
  global_dict = {
    'scene_type': 'HIGHWAY',
    'is_bag_main': True,  # False: main分支之前的包   True: main分支之后的包
    'g_is_display_enu': False,  # True: local_view显示enu坐标系   False: local_view显示自车坐标系
    'is_new_loc': False,  #   True:新定位 False:老定位; 目前是自适应的，有新定位就用新定位，没有就用老定位
    'is_match_planning': True,  # True: topic按照planning接收的时间戳匹配；  False:按最近时间匹配
    'is_enu_to_car': True,
    'is_vis_map': False,
    'is_vis_sdmap': True,
    'is_vis_hpp': False,
    'is_vis_radar': False,
    'is_vis_rdg_line': True,
    'is_vis_rdg_obj': False,
    'is_vis_me_obj': False,
    'is_vis_lane_mark': True,
    'is_vis_merge_point': True,
  }


def set_value(key, value):
  # 设置一个全局变量
  global_dict[key] = value


def get_value(key):
  # 获得一个全局变量，不存在则提示读取对应变量失败
  try:
    return global_dict[key]
  except:
    print('read '+ key +' error!')
    return False

def set_value_by_scene(scene):
  if scene == 'HPP' or scene == 'PARKING_APA':
    global_dict['scene_type'] = 'HPP'
    global_dict['g_is_display_enu'] = True
    global_dict['is_vis_hpp'] = True
    global_dict['is_vis_map'] = False
    global_dict['is_vis_sdmap'] = False
    global_dict['is_vis_rdg_line'] = False
    global_dict['is_vis_lane_mark'] = False
    global_dict['is_vis_merge_point'] = False