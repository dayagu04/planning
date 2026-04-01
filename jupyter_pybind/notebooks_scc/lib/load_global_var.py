#
def init():  # 初始化
  global global_dict
  global_dict = {
    'car_type': 'CHERY_E0X',
    'scene_type': 'HIGHWAY',
    'is_bag_main': True,  # False: main分支之前的包   True: main分支之后的包
    'g_is_display_enu': False,  # True: local_view显示enu坐标系   False: local_view显示自车坐标系
    'is_new_loc': False,  #   True:新定位 False:老定位; 目前是自适应的，有新定位就用新定位，没有就用老定位
    'is_match_planning': True,  # True: topic按照planning接收的时间戳匹配；  False:按最近时间匹配
    'is_enu_to_car': True,
    'is_vis_map': False,
    'is_vis_sdmap': False,
    'is_vis_sdpromap': False,
    'is_vis_radar': False,
    'is_vis_fus_line': True,
    'is_vis_rdg_line': True,
    'is_vis_rdg_obj': True,
    'is_vis_me_obj': False,
    'is_vis_lane_mark': True,
    'is_vis_merge_point': False,
    'is_calc_min_turn_radius': False,  # True: 计算自车前方（0-80m）真值路径点的最小转弯半径
    'is_vis_hpp_map': False,
    'is_vis_occ_obj': False,
    'is_vis_ground_line': False,
    'is_vis_parking_slot': False,
    'is_vis_speed_bump': False,
    'is_vis_ground_mark': False,
    'is_vis_rads_path': False,
    'is_vis_nsa_line': False,
    'is_vis_prediction': True,
    'is_vis_fus_obj': True,
    'is_vis_lane_topo': True,
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
    global_dict['is_vis_map'] = False
    global_dict['is_vis_sdmap'] = False
    global_dict['is_vis_sdpromap'] = False
    global_dict['is_vis_fus_line'] = True          #融合车道线
    global_dict['is_vis_lane_topo'] = False        #融合可变车道
    global_dict['is_vis_rdg_line'] = False         #感知车道线
    global_dict['is_vis_rdg_obj'] = False          #感知 OD
    global_dict['is_vis_rdg_occ'] = False          #感知 OCC
    global_dict['is_vis_rdg_groundline'] = False   #感知接地线
    global_dict['is_vis_lane_mark'] = False
    global_dict['is_vis_merge_point'] = False
    global_dict['is_vis_hpp_map'] = True
    global_dict['is_vis_fus_obj'] = True          #融合OD
    global_dict['is_vis_occ_obj'] = True          #融合OCC
    global_dict['is_vis_ground_line'] = True      #融合接地线
    global_dict['is_vis_parking_slot'] = True     #融合车位
    global_dict['is_vis_speed_bump'] = True       #融合减速带
    global_dict['is_vis_ground_mark'] = False
    global_dict['is_vis_prediction'] = False      #预测
    global_dict['is_vis_ego_motion_sim'] = False
    global_dict['is_vis_snrd'] = False
    global_dict['is_vis_snrd'] = False
    global_dict['is_vis_smooth_refline'] = False
  elif scene == 'NSA':
    global_dict['scene_type'] = 'NSA'
    global_dict['is_vis_map'] = False
    global_dict['is_vis_sdmap'] = False
    global_dict['is_vis_fus_line'] = False
    global_dict['is_vis_lane_topo'] = True
    global_dict['is_vis_rdg_line'] = False
    global_dict['is_vis_rdg_obj'] = True
    global_dict['is_vis_rdg_occ'] = True
    global_dict['is_vis_rdg_groundline'] = True
    global_dict['is_vis_lane_mark'] = False
    global_dict['is_vis_merge_point'] = False
    global_dict['is_vis_fus_obj'] = True
    global_dict['is_vis_occ_obj'] = True
    global_dict['is_vis_speed_bump'] = True
    global_dict['is_vis_nsa_line'] = True
    global_dict['is_vis_prediction'] = False
    global_dict['is_vis_ego_motion_sim'] = True
    global_dict['is_vis_snrd'] = True
    global_dict['is_vis_smooth_refline'] = True
  elif scene == 'RADS':
    global_dict['scene_type'] = 'RADS'
    global_dict['is_vis_map'] = False
    global_dict['is_vis_sdmap'] = False
    global_dict['is_vis_fus_line'] = False
    global_dict['is_vis_lane_topo'] = True
    global_dict['is_vis_rdg_line'] = False
    global_dict['is_vis_rdg_obj'] = True
    global_dict['is_vis_rdg_occ'] = True
    global_dict['is_vis_rdg_groundline'] = True
    global_dict['is_vis_lane_mark'] = False
    global_dict['is_vis_merge_point'] = False
    global_dict['is_vis_fus_obj'] = True
    global_dict['is_vis_occ_obj'] = True
    global_dict['is_vis_speed_bump'] = True
    global_dict['is_vis_rads_path'] = True
    global_dict['is_vis_prediction'] = False
    global_dict['is_vis_ego_motion_sim'] = True
    global_dict['is_vis_snrd'] = True
    global_dict['is_vis_smooth_refline'] = True