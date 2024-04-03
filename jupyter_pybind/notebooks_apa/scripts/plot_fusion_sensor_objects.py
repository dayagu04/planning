import sys, os
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view import *
sys.path.append('../..')
sys.path.append('../../../')



# bag path and frame dt
# bag_path = "/home/byliu12/0818/2023-08-18-15-36-33.record"
bag_path = "/home/byliu12/0818/2023_0819_1059_80km_80m_follow.record"
frame_dt = 0.02 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()

g_trackid = 29

data_fusion_command = ColumnDataSource(data ={
  'time': [],
  'posx': [],
  'posy': [],
  'velx': [],
  'vely': [],
  'posx_me': [],
  'posy_me': [],
  'velx_me': [],
  'vely_me': [],
  'posx_fm': [],
  'posy_fm': [],
  'velx_fm': [],
  'vely_fm': [],
  'posx_fl': [],
  'posy_fl': [],
  'velx_fl': [],
  'vely_fl': [],
  'posx_fr': [],
  'posy_fr': [],
  'velx_fr': [],
  'vely_fr': [],
  'posx_rl': [],
  'posy_rl': [],
  'velx_rl': [],
  'vely_rl': [],
  'posx_rr': [],
  'posy_rr': [],
  'velx_rr': [],
  'vely_rr': [],
})

data_cursor_fig1 = ColumnDataSource(data ={
  'x': [],
  'y': [],
})
data_cursor_fig2 = ColumnDataSource(data ={
  'x': [],
  'y': [],
})
data_cursor_fig3 = ColumnDataSource(data ={
  'x': [],
  'y': [],
})
data_cursor_fig4 = ColumnDataSource(data ={
  'x': [],
  'y': [],
})

def obj_id_handler(id, me_id, fm_id, fl_id, fr_id, rl_id, rr_id):
    print('id:',id)
    print('me_id:',me_id)
    print('fm_id:',fm_id)
    print('fl_id:',fl_id)
    print('fr_id:',fr_id)
    print('rl_id:',rl_id)
    print('rr_id:',rr_id)
    time_vec = []
    posx_tmp = []
    posy_tmp = []
    velx_tmp = []
    vely_tmp = []
    posx_tmp_me = []
    posy_tmp_me = []
    velx_tmp_me = []
    vely_tmp_me = []
    posx_radar_tmp = [[[] for t in range(5)] for k in range(4)]
    posy_radar_tmp = [[[] for t in range(5)] for k in range(4)]
    velx_radar_tmp = [[[] for t in range(5)] for k in range(4)]
    vely_radar_tmp = [[[] for t in range(5)] for k in range(4)]
    
    # global fusion_id, mobile_id, radar_fm_id, radar_fl_id, radar_fr_id, radar_rl_id, radar_rr_id
    global object_id
    # fusion_id = id
    # mobile_id = me_id
    object_id = [id, me_id, fm_id, fl_id, fr_id, rl_id, rr_id]
    print(object_id)
    # radar_fm_id = fm_id
    # radar_fl_id = fl_id
    # radar_fr_id = fr_id
    # radar_rl_id = rl_id
    # radar_rr_id = rr_id
    
    fusion_data = bag_loader.fus_msg['data']
    me_data = bag_loader.me_msg['data']
    radar_data = [bag_loader.radar_fm_msg['data'],bag_loader.radar_fl_msg['data'],bag_loader.radar_fr_msg['data'],
                  bag_loader.radar_rl_msg['data'],bag_loader.radar_rr_msg['data']]
    time = 0.0

    while time < max_time:
      update_local_view_data_index(fig1, bag_loader, time, local_view_data) # update_local_view_data_index
      
      #fusion_object
      fusion_index = local_view_data['data_index']['fus_msg_idx']
      obs_num_fusion = len(fusion_data[fusion_index].fusion_object) #29
      obstacle_list = fusion_data[fusion_index].fusion_object
      find_fusion = 0
      for i in range(obs_num_fusion):
          if(obstacle_list[i].additional_info.track_id == object_id[0]):
              find_fusion  = 1 
              # posx_tmp.append(obstacle_list[i].common_info.relative_position.x)
              # velx_tmp.append(0)
              # vely_tmp.append(0)
              posx_tmp.append(obstacle_list[i].common_info.relative_position.x)
              posy_tmp.append(obstacle_list[i].common_info.relative_position.y)
              velx_tmp.append(obstacle_list[i].common_info.relative_velocity.x)
              vely_tmp.append(obstacle_list[i].common_info.relative_velocity.y)
      if(find_fusion == 0):
          posx_tmp.append(0)
          posy_tmp.append(0)
          velx_tmp.append(0)
          vely_tmp.append(0)

      #mobieye_object
      me_index = local_view_data['data_index']['me_msg_idx']
      me_obs_num = len(me_data[me_index].camera_perception_objects) #1
      me_obj_list = me_data[me_index].camera_perception_objects
      find_me = 0
      for i in range(me_obs_num):
          if(me_obj_list[i].common_info.id == object_id[1]):
              find_me = 1
              posx_tmp_me.append(me_obj_list[i].common_info.relative_position.x)
              posy_tmp_me.append(me_obj_list[i].common_info.relative_position.y)
              velx_tmp_me.append(me_obj_list[i].common_info.relative_velocity.x)
              vely_tmp_me.append(me_obj_list[i].common_info.relative_velocity.y)
      if(find_me == 0):
          posx_tmp_me.append(0)
          posy_tmp_me.append(0)
          velx_tmp_me.append(0)
          vely_tmp_me.append(0)
      

      #radar_object
      for i in range(5):#fm,fl,fr,rl,rr
        index = 0
        if i==0:
          index = local_view_data['data_index']['radar_fm_msg_idx']
        elif i==1:
          index = local_view_data['data_index']['radar_fl_msg_idx']
        elif i==2:
          index = local_view_data['data_index']['radar_fr_msg_idx']
        elif i==3:
          index = local_view_data['data_index']['radar_rl_msg_idx']
        elif i==4:
          index = local_view_data['data_index']['radar_rr_msg_idx']
        radar_obs_num = len(radar_data[i][index].object_list) #1
        radar_obj_list = radar_data[i][index].object_list
        find_radar = 0
        for j in range(radar_obs_num):
          if(radar_obj_list[j].id == object_id[i+2]):
            find_radar = 1
            posx_radar_tmp[0][i].append(radar_obj_list[j].relative_position.x)
            posy_radar_tmp[1][i].append(radar_obj_list[j].relative_position.y)
            velx_radar_tmp[2][i].append(radar_obj_list[j].relative_velocity.x)
            vely_radar_tmp[3][i].append(radar_obj_list[j].relative_velocity.y)
        if(find_radar == 0):
          posx_radar_tmp[0][i].append(0)
          posy_radar_tmp[1][i].append(0)
          velx_radar_tmp[2][i].append(0)
          vely_radar_tmp[3][i].append(0)
        
      time_vec.append(time)
      time = time + 0.05
    #print(time_vec)
    data_fusion_command.data.update({
    'time': time_vec,
    'posx': posx_tmp,
    'posy': posy_tmp,
    'velx': velx_tmp,
    'vely': vely_tmp,
    'posx_me': posx_tmp_me,
    'posy_me': posy_tmp_me,
    'velx_me': velx_tmp_me,
    'vely_me': vely_tmp_me,
    'posx_fm': posx_radar_tmp[0][0],
    'posy_fm': posy_radar_tmp[1][0],
    'velx_fm': velx_radar_tmp[2][0],
    'vely_fm': vely_radar_tmp[3][0],
    'posx_fl': posx_radar_tmp[0][1],
    'posy_fl': posy_radar_tmp[1][1],
    'velx_fl': velx_radar_tmp[2][1],
    'vely_fl': vely_radar_tmp[3][1],
    'posx_fr': posx_radar_tmp[0][2],
    'posy_fr': posy_radar_tmp[1][2],
    'velx_fr': velx_radar_tmp[2][2],
    'vely_fr': vely_radar_tmp[3][2],
    'posx_rl': posx_radar_tmp[0][3],
    'posy_rl': posy_radar_tmp[1][3],
    'velx_rl': velx_radar_tmp[2][3],
    'vely_rl': vely_radar_tmp[3][3],
    'posx_rr': posx_radar_tmp[0][4],
    'posy_rr': posy_radar_tmp[1][4],
    'velx_rr': velx_radar_tmp[2][4],
    'vely_rr': vely_radar_tmp[3][4],
    })    
    push_notebook()


fig2 = bkp.figure(x_axis_label='time', y_axis_label='posx',x_range = [0, max_time], width=700, height=300)
fig3 = bkp.figure(x_axis_label='time', y_axis_label='posy',x_range = [0, max_time], width=700, height=300)
fig4 = bkp.figure(x_axis_label='time', y_axis_label='velx',x_range = [0, max_time], width=700, height=300)
fig5 = bkp.figure(x_axis_label='time', y_axis_label='vely',x_range = [0, max_time], width=700, height=300)

f2 = fig2.line('time', 'posx', source = data_fusion_command, line_width = 1, line_color = 'gray', line_dash = 'solid', legend_label = 'posx')
fig2.line('time', 'posx_me', source = data_fusion_command, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'posx_me')
fig2.line('time', 'posx_fm', source = data_fusion_command, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'posx_fm')
fig2.line('time', 'posx_fl', source = data_fusion_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'posx_fl')
fig2.line('time', 'posx_fr', source = data_fusion_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'posx_fr')
fig2.line('time', 'posx_rl', source = data_fusion_command, line_width = 1, line_color = 'yellow', line_dash = 'solid', legend_label = 'posx_rl')
fig2.line('time', 'posx_rr', source = data_fusion_command, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'posx_rr')
fig2.line('x', 'y', source = data_cursor_fig1, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'cursor')

f3 = fig3.line('time', 'posy', source = data_fusion_command, line_width = 1, line_color = 'gray', line_dash = 'solid', legend_label = 'posy')
fig3.line('time', 'posy_me', source = data_fusion_command, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'posy_me')
fig3.line('time', 'posy_fm', source = data_fusion_command, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'posy_fm')
fig3.line('time', 'posy_fl', source = data_fusion_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'posy_fl')
fig3.line('time', 'posy_fr', source = data_fusion_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'posy_fr')
fig3.line('time', 'posy_rl', source = data_fusion_command, line_width = 1, line_color = 'yellow', line_dash = 'solid', legend_label = 'posy_rl')
fig3.line('time', 'posy_rr', source = data_fusion_command, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'posy_rr')
fig3.line('x', 'y', source = data_cursor_fig2, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'cursor')

f4 = fig4.line('time', 'velx', source = data_fusion_command, line_width = 1, line_color = 'gray', line_dash = 'solid', legend_label = 'velx')
fig4.line('time', 'velx_me', source = data_fusion_command, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'velx_me')
fig4.line('time', 'velx_fm', source = data_fusion_command, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'velx_fm')
fig4.line('time', 'velx_fl', source = data_fusion_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'velx_fl')
fig4.line('time', 'velx_fr', source = data_fusion_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'velx_fr')
fig4.line('time', 'velx_rl', source = data_fusion_command, line_width = 1, line_color = 'yellow', line_dash = 'solid', legend_label = 'velx_rl')
fig4.line('time', 'velx_rr', source = data_fusion_command, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'velx_rr')
fig4.line('x', 'y', source = data_cursor_fig3, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'cursor')

f5 = fig5.line('time', 'vely', source = data_fusion_command, line_width = 1, line_color = 'gray', line_dash = 'solid', legend_label = 'vely')
fig5.line('time', 'vely_me', source = data_fusion_command, line_width = 1, line_color = 'orange', line_dash = 'solid', legend_label = 'vely_me')
fig5.line('time', 'vely_fm', source = data_fusion_command, line_width = 1, line_color = 'green', line_dash = 'solid', legend_label = 'vely_fm')
fig5.line('time', 'vely_fl', source = data_fusion_command, line_width = 1, line_color = 'red', line_dash = 'solid', legend_label = 'vely_fl')
fig5.line('time', 'vely_fr', source = data_fusion_command, line_width = 1, line_color = 'blue', line_dash = 'solid', legend_label = 'vely_fr')
fig5.line('time', 'vely_rl', source = data_fusion_command, line_width = 1, line_color = 'yellow', line_dash = 'solid', legend_label = 'vely_rl')
fig5.line('time', 'vely_rr', source = data_fusion_command, line_width = 1, line_color = 'black', line_dash = 'solid', legend_label = 'vely_rr')
fig5.line('x', 'y', source = data_cursor_fig4, line_width = 1, line_color = 'grey', line_dash = 'solid', legend_label = 'cursor')


hover2 = HoverTool(renderers=[f2], tooltips=[('time', '@time'), ('posx', '@posx'),('posx_me', '@posx_me'),('posx_fm','@posx_fm'),('posx_fl','@posx_fl'),('posx_fr','@posx_fr'),('posx_rl','@posx_rl'),('posx_rr','@posx_rr')], mode='vline')
hover3 = HoverTool(renderers=[f3], tooltips=[('time', '@time'), ('posy', '@posy'),('posy_me', '@posy_me'),('posy_fm','@posy_fm'),('posy_fl','@posy_fl'),('posy_fr','@posy_fr'),('posy_rl','@posy_rl'),('posy_rr','@posy_rr')], mode='vline')
hover4 = HoverTool(renderers=[f4], tooltips=[('time', '@time'), ('velx', '@velx'),('velx_me', '@velx_me'),('velx_fm','@velx_fm'),('velx_fl','@velx_fl'),('velx_fr','@velx_fr'),('velx_rl','@velx_rl'),('velx_rr','@velx_rr')], mode='vline')
hover5 = HoverTool(renderers=[f5], tooltips=[('time', '@time'), ('vely', '@vely'),('vely_me', '@vely_me'),('vely_fm','@vely_fm'),('vely_fl','@vely_fl'),('vely_fr','@vely_fr'),('vely_rl','@vely_rl'),('vely_rr','@vely_rr')], mode='vline')
fig2.add_tools(hover2)
fig3.add_tools(hover3)
fig4.add_tools(hover4)
fig5.add_tools(hover5)

fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
fig2.legend.click_policy = 'hide'
fig3.legend.click_policy = 'hide'
fig4.legend.click_policy = 'hide'
fig5.legend.click_policy = 'hide'

# ##end

ti = 0.0

### sliders config
class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description= "bag_time",min=0.0, max=max_time, value=-0.1, step=frame_dt)
    ipywidgets.interact(slider_callback, bag_time = self.time_slider)

# 障碍物的id选择
class ObjText:
  def __init__(self,  obj_callback):
    self.id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_id",min=0.0, max=10000)
    self.me_id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_me_id",min=0.0, max=10000)
    self.fm_id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_fm_id",min=0.0, max=10000)
    self.fl_id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_fl_id",min=0.0, max=10000)
    self.fr_id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_fr_id",min=0.0, max=10000)
    self.rl_id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_rl_id",min=0.0, max=10000)
    self.rr_id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_rr_id",min=0.0, max=10000)
    ipywidgets.interact(obj_callback, id = self.id, me_id = self.me_id, fm_id = self.fm_id, fl_id = self.fl_id, fr_id = self.fr_id, rl_id = self.rl_id, rr_id = self.rr_id,)
    # ipywidgets.interact(obj_callback, me_id = self.me_id)

# class ObjText_me:
#   def __init__(self,  obj_callback):
#     self.id = ipywidgets.IntText(layout=ipywidgets.Layout(width='10%'), description= "obj_me_id",min=0.0, max=10000)
#     ipywidgets.interact(obj_callback, id = self.id)

### sliders callback
def slider_callback(bag_time):
  global ti
  kwargs = locals()
  data_cursor_fig1.data.update({'x':[bag_time,bag_time],'y':[-20,40]})
  data_cursor_fig2.data.update({'x':[bag_time,bag_time],'y':[-6,6]})
  data_cursor_fig3.data.update({'x':[bag_time,bag_time],'y':[-1,1]})
  data_cursor_fig4.data.update({'x':[bag_time,bag_time],'y':[-1,1]})
  update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
  push_notebook()

bkp.show(row(fig1,column(fig2,fig3),column(fig4,fig5)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
slider_class = ObjText(obj_id_handler) 


