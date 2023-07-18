#!/usr/bin/python
# encoding=utf-8

from IPython.core.display import display, HTML
from cyber_record.record import Record
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from bokeh.models import WheelZoomTool, HoverTool, ColumnDataSource
import bokeh.plotting as bkp
import ipywidgets
import math


class SingleSlot:
  def __init__(self):
    self.id = 0 
    self.type = 0
    self.corner_point_x_vec = []
    self.corner_point_y_vec = []
    self.fusion_source = 0
    self.slot_side = 0
    self.limiter_position_x_vec = []
    self.limiter_position_y_vec = []


class SingleUssObject:
  def __init__(self):
    self.id = 0 
    self.x_vec = []
    self.y_vec = []


class ApaInfoPlotter(object):
  def __init__(self):
    # bag path and frame dt
    path = '/asw/planning/apa_0.00000'
    self.bag = Record(path)
    display(HTML('<style>.container { width:95% !important;  }</style>'))
    output_notebook()
    self.all_frame_fig = bkp.figure(x_axis_label='x(m)', y_axis_label='y(m)', width=800, height=400, match_aspect=True, aspect_scale=1.0)
    self.cur_frame_fig = bkp.figure(x_axis_label='x(m)', y_axis_label='y(m)', width=800, height=400, match_aspect=True, aspect_scale=1.0)
    self.max_time = 0.0
    self.min_time = float('inf')
    # localization
    self.pos_index = 0
    self.pos_t_vec = []
    self.pos_x_vec = []
    self.pos_y_vec = []
    self.pos_yaw_vec = []
    self.data_cur_pos_point = ColumnDataSource(data = {'x':[], 'y':[]})
    self.data_cur_pos_box = ColumnDataSource(data = {'x':[], 'y':[]})
    # all plan points
    self.plan_point_x_vec = []
    self.plan_point_y_vec = []
    self.plan_point_yaw_vec = []
    # plan segmets
    self.plan_segment_index = 0
    self.plan_segment_t_vec = []
    self.plan_segment_x_vec = []
    self.plan_segment_y_vec = []
    self.plan_segment_yaw_vec = []
    self.plan_segment_gear_vec = []
    self.data_cur_plan_seg = ColumnDataSource(data = {
        'x':[],
        'y':[]})
    # slots
    self.slots_index = 0
    self.slots_t_vec = []
    self.slots_select_id_vec = []
    self.slots_vec = []
    self.replan_slot_vec = []
    self.replan_t_vec = []
    self.max_slot_num = 20
    self.data_slot_ids = [
        ColumnDataSource(data = {'id': [], 'id_text_x': [], 'id_text_y': []})
        for i in range(self.max_slot_num)]
    self.data_slot_points = [
        ColumnDataSource(data = {'corner_point_x': [], 'corner_point_y': []})
        for i in range(self.max_slot_num)]
    # objects
    self.uss_objects_index = 0
    self.uss_objects_t_vec = []
    self.uss_objects_vec = []
    self.max_uss_obj_num = 20
    self.data_uss_obj_ids = [
        ColumnDataSource(data = {'id': [], 'id_text_x': [], 'id_text_y': []})
        for i in range(self.max_uss_obj_num)]
    self.data_uss_obj_points = [
        ColumnDataSource(data = {'x': [], 'y': []})
        for i in range(self.max_uss_obj_num)]

  def get_closed_veh_box(self, x, y, theta):
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


  def load_data(self):
    # load localization msg
    for topic, msg, t in self.bag.read_messages('/iflytek/localization/ego_pose'):
      self.max_time = max(self.max_time, msg.header.timestamp)
      self.min_time = min(self.min_time, msg.header.timestamp)
      self.pos_t_vec.append(msg.header.timestamp)
      self.pos_x_vec.append(msg.pose.local_position.x)
      self.pos_y_vec.append(msg.pose.local_position.y)
      self.pos_yaw_vec.append(msg.pose.euler_angles.yaw)
    # print('localization msg length: ', len(self.pos_t_vec))

    # load planning msg
    pre_gear = None
    for topic, msg, t in self.bag.read_messages('/iflytek/planning/plan'):
      cur_gear = msg.gear_command.gear_command_value
      self.max_time = max(self.max_time, msg.meta.header.timestamp)
      self.min_time = min(self.min_time, msg.meta.header.timestamp)
      if len(msg.trajectory.trajectory_points) <= 1:
        continue
      plan_segment_x_vec = []
      plan_segment_y_vec = []
      plan_segment_yaw_vec = []
      plan_segment_t = msg.meta.header.timestamp
      for i in range(len(msg.trajectory.trajectory_points)):
        plan_segment_x_vec.append(msg.trajectory.trajectory_points[i].x)
        plan_segment_y_vec.append(msg.trajectory.trajectory_points[i].y)
        plan_segment_yaw_vec.append(msg.trajectory.trajectory_points[i].heading_yaw)
      self.plan_segment_t_vec.append(plan_segment_t)
      self.plan_segment_gear_vec.append(cur_gear)
      self.plan_segment_x_vec.append(plan_segment_x_vec)
      self.plan_segment_y_vec.append(plan_segment_y_vec)
      self.plan_segment_yaw_vec.append(plan_segment_yaw_vec)
      if pre_gear is None or pre_gear != cur_gear:
        self.plan_point_x_vec.extend(plan_segment_x_vec)
        self.plan_point_y_vec.extend(plan_segment_y_vec)
        self.plan_point_yaw_vec.extend(plan_segment_yaw_vec)
        self.replan_t_vec.append(plan_segment_t)
      pre_gear = cur_gear
    # print('planning points number: ', len(self.plan_point_x_vec))
    # print('planning segments number: ', len(self.plan_segment_t_vec))
  
    # load slot
    for topic, msg, t in self.bag.read_messages('/iflytek/fusion/parking_slot'):
      self.max_time = max(self.max_time, msg.header.timestamp)
      self.min_time = min(self.min_time, msg.header.timestamp)
      self.slots_t_vec.append(msg.header.timestamp)
      self.slots_select_id_vec.append(msg.select_slot_id)
      slot_vec = []
      for i in range(len(msg.parking_fusion_slot_lists)):
        slot = SingleSlot()
        slot.id = msg.parking_fusion_slot_lists[i].id
        slot.type = msg.parking_fusion_slot_lists[i].type
        corner_points = msg.parking_fusion_slot_lists[i].corner_points
        for j in range(len(corner_points)):
          slot.corner_point_x_vec.append(corner_points[j].x)
          slot.corner_point_y_vec.append(corner_points[j].y)
        slot.fusion_source = msg.parking_fusion_slot_lists[i].fusion_source
        slot.slot_side = msg.parking_fusion_slot_lists[i].slot_side
        limiter_position = msg.parking_fusion_slot_lists[i].limiter_position
        for j in range(len(limiter_position)):
          slot.limiter_position_x_vec.append(limiter_position[j].x)
          slot.limiter_position_y_vec.append(limiter_position[j].y)
        slot_vec.append(slot)
      self.slots_vec.append(slot_vec)
    # print('parking slot msg number: ', len(self.slots_vec))

    # load objects
    for topic, msg, t in self.bag.read_messages('/iflytek/fusion/objects'):
      self.max_time = max(self.max_time, msg.header.timestamp)
      self.min_time = min(self.min_time, msg.header.timestamp)
      self.uss_objects_t_vec.append(msg.header.timestamp)
      uss_object_vec = []
      for i in range(len(msg.uss_only_object_list)):
        uss_object = SingleUssObject()
        uss_object.id = msg.uss_only_object_list[i].obj_id
        uss_object.x_vec.append(msg.uss_only_object_list[i].obj_point1.x)
        uss_object.x_vec.append(msg.uss_only_object_list[i].obj_point2.x)
        uss_object.y_vec.append(msg.uss_only_object_list[i].obj_point1.y)
        uss_object.y_vec.append(msg.uss_only_object_list[i].obj_point2.y)
        uss_object_vec.append(uss_object)
      self.uss_objects_vec.append(uss_object_vec)
    # print('fusion objects msg number: ', len(self.uss_objects_vec))

  def get_replan_slot_vec(self):
    for i in range(len(self.replan_t_vec)):
      replan_time = self.plan_segment_t_vec[i]
      slot_index = 0
      while slot_index + 1 < len(self.slots_vec) and self.slots_t_vec[slot_index] < replan_time:
        slot_index = slot_index + 1
      slot_index = max(slot_index - 1, 0)
      for j in range(len(self.slots_vec[slot_index])):
        if self.slots_select_id_vec[slot_index] == self.slots_vec[slot_index][j].id:
          self.replan_slot_vec.append(self.slots_vec[slot_index][j])
          break


  def plot_figure(self):
    self.load_data()
    self.get_replan_slot_vec()
    # self.plot_all_frame()
    self.plot_cur_frame()

  def update_index(self, bag_time):
    absolute_bag_time = self.min_time + bag_time * 1e6
    self.pos_index = 0
    while self.pos_index + 1 < len(self.pos_t_vec) and self.pos_t_vec[self.pos_index] < absolute_bag_time:
      self.pos_index = self.pos_index + 1
    self.pos_index = max(self.pos_index - 1, 0)

    self.plan_segment_index = 0
    while self.plan_segment_index + 1 < len(self.plan_segment_t_vec) and self.plan_segment_t_vec[self.plan_segment_index] < absolute_bag_time:
      self.plan_segment_index = self.plan_segment_index + 1
    self.plan_segment_index = max(self.plan_segment_index - 1, 0)

    self.slots_index = 0
    while self.slots_index + 1 < len(self.slots_t_vec) and self.slots_t_vec[self.slots_index] < absolute_bag_time:
      self.slots_index = self.slots_index + 1
    self.slots_index = max(self.slots_index - 1, 0)

    self.uss_objects_index = 0
    while self.uss_objects_index + 1 < len(self.uss_objects_t_vec) and self.uss_objects_t_vec[self.uss_objects_index] < absolute_bag_time:
      self.uss_objects_index = self.uss_objects_index + 1
    self.uss_objects_index = max(self.uss_objects_index - 1, 0)


  def update_data(self):
    # localization
    self.data_cur_pos_point.data.update({
      'x': [],
      'y': [],
    })
    self.data_cur_pos_box.data.update({
      'x': [],
      'y': [],
    })
    if len(self.pos_x_vec) != 0:
      pos_x = self.pos_x_vec[self.pos_index]
      pos_y = self.pos_y_vec[self.pos_index]
      pos_yaw = self.pos_yaw_vec[self.pos_index]
      pos_box = self.get_closed_veh_box(pos_x, pos_y, pos_yaw)
      self.data_cur_pos_point.data.update({
        'x': [pos_x],
        'y': [pos_y],
      })
      self.data_cur_pos_box.data.update({
        'x': pos_box[0],
        'y': pos_box[1],
      })

    # planning
    self.data_cur_plan_seg.data.update({
        'x': [],
        'y': [],
    })
    if len(self.plan_segment_x_vec) != 0:
      self.data_cur_plan_seg.data.update({
          'x': self.plan_segment_x_vec[self.plan_segment_index],
          'y': self.plan_segment_y_vec[self.plan_segment_index],
      })
    
    # slot
    for i in range(self.max_slot_num):
      self.data_slot_ids[i].data.update({
          'id':[],
          'id_text_x':[],
          'id_text_y':[],
      })
      self.data_slot_points[i].data.update({
          'corner_point_x':[],
          'corner_point_y':[],
      })
    if len(self.slots_vec) != 0:
      # update data
      for i in range(len(self.slots_vec[self.slots_index])):
        slot = self.slots_vec[self.slots_index][i]
        self.data_slot_ids[i].data.update({
            'id':[slot.id],
            'id_text_x':[(slot.corner_point_x_vec[0] + slot.corner_point_x_vec[2] + slot.corner_point_x_vec[3] + slot.corner_point_x_vec[1]) * 0.25],
            'id_text_y':[(slot.corner_point_y_vec[0] + slot.corner_point_y_vec[2] + slot.corner_point_y_vec[3] + slot.corner_point_y_vec[1]) * 0.25],
        })
        self.data_slot_points[i].data.update({
            'corner_point_x':[slot.corner_point_x_vec[0], slot.corner_point_x_vec[2], slot.corner_point_x_vec[3], slot.corner_point_x_vec[1]],
            'corner_point_y':[slot.corner_point_y_vec[0], slot.corner_point_y_vec[2], slot.corner_point_y_vec[3], slot.corner_point_y_vec[1]],
        })

    # uss objects
    for i in range(self.max_uss_obj_num):
      self.data_uss_obj_ids[i].data.update({
          'id':[],
          'id_text_x':[],
          'id_text_y':[],
      })
      self.data_uss_obj_points[i].data.update({
          'x':[],
          'y':[],
      })
    if len(self.uss_objects_vec) != 0:
      for i in range(len(self.uss_objects_vec[self.uss_objects_index])):
        object_vec = self.uss_objects_vec[self.uss_objects_index][i]
        self.data_uss_obj_ids[i].data.update({
            'id':[slot.id],
            'id_text_x':[(object_vec.x_vec[0] + object_vec.x_vec[1]) * 0.5],
            'id_text_y':[(object_vec.y_vec[0] + object_vec.y_vec[1]) * 0.5],
        })
        self.data_uss_obj_points[i].data.update({
            'x':object_vec.x_vec,
            'y':object_vec.y_vec,
        })


  def plot_all_frame(self):
    # plot all localization position
    f1 = self.all_frame_fig.line(self.pos_x_vec, self.pos_y_vec, line_width=1, line_color='blue', line_dash='solid', legend_label='loc pos')
    for i in range(len(self.pos_x_vec)):
      pos_x = self.pos_x_vec[i]
      pos_y = self.pos_y_vec[i]
      pos_yaw = self.pos_yaw_vec[i]
      pos_box = self.get_closed_veh_box(pos_x, pos_y, pos_yaw)
      self.all_frame_fig.line(pos_box[0], pos_box[1], line_width=1, line_color='blue', line_dash='solid')
    # plot all trajectory points
    self.all_frame_fig.line(self.plan_point_x_vec, self.plan_point_y_vec, line_width=1, line_color='green', line_dash='solid', legend_label='traj pos')
    for i in range(len(self.plan_point_x_vec)):
      traj_x = self.plan_point_x_vec[i]
      traj_y = self.plan_point_y_vec[i]
      traj_yaw = self.plan_point_yaw_vec[i]
      pos_box = self.get_closed_veh_box(traj_x, traj_y, traj_yaw)
      self.all_frame_fig.line(pos_box[0], pos_box[1], line_width=1, line_color='green', line_dash='solid')
    for i in range(len(self.replan_slot_vec)):
      slot = self.replan_slot_vec[i]
      x_vec = [slot.corner_point_x_vec[0], slot.corner_point_x_vec[2], slot.corner_point_x_vec[3], slot.corner_point_x_vec[1]]
      y_vec = [slot.corner_point_y_vec[0], slot.corner_point_y_vec[2], slot.corner_point_y_vec[3], slot.corner_point_y_vec[1]]
      self.all_frame_fig.line(x_vec, y_vec, line_width=1, line_dash='solid', legend_label='seg: '+str(i))

    self.all_frame_fig.toolbar.active_scroll = self.all_frame_fig.select_one(WheelZoomTool)
    self.all_frame_fig.legend.click_policy = 'hide'
    bkp.show(self.all_frame_fig, notebook_handle=True)


  def plot_cur_frame(self):
    # plot cur localization position
    if len(self.pos_x_vec) != 0:
      # plot cur point of rear axle center
      self.cur_frame_fig.circle('x', 'y', source=self.data_cur_pos_point, color='blue', size=5, legend_label='rear axle center')
      # plot ego box
      self.cur_frame_fig.line('x', 'y', source=self.data_cur_pos_box, line_width=1, line_color='blue', line_dash='solid', legend_label='cur box')

    # plot current plan segment
    if len(self.plan_segment_x_vec) != 0:
      self.cur_frame_fig.line('x', 'y', source=self.data_cur_plan_seg, line_width=1, line_color='green', line_dash='solid', legend_label='cur traj seg')

    # plot slots    
    if len(self.slots_vec) != 0:
      for i in range(self.max_slot_num):
        self.cur_frame_fig.line('corner_point_x', 'corner_point_y', source=self.data_slot_points[i], line_color='red', line_width=1, line_dash='solid')
        self.cur_frame_fig.text(x = 'id_text_x', y = 'id_text_y', text = 'id', source=self.data_slot_ids[i], text_color='red', text_align='center', text_font_size='10pt')

    # plot uss objects    
    if len(self.uss_objects_vec) != 0:
      for i in range(self.max_uss_obj_num):
        self.cur_frame_fig.line('x', 'y', source=self.data_uss_obj_points[i], line_color='purple', line_width=1, line_dash='solid')
        self.cur_frame_fig.text(x = 'id_text_x', y = 'id_text_y', text = 'id', source=self.data_uss_obj_ids[i], text_color='purple', text_align='center', text_font_size='10pt')

    self.cur_frame_fig.toolbar.active_scroll = self.cur_frame_fig.select_one(WheelZoomTool)
    self.cur_frame_fig.legend.click_policy = 'hide'
    bkp.show(self.cur_frame_fig, notebook_handle=True)


  def update(self, bag_time):
    self.update_index(bag_time)
    self.update_data()
    push_notebook()


class ApaInfoSlider:
  def __init__(self):
    self.apa_info_plotter = ApaInfoPlotter()
    self.apa_info_plotter.plot_figure()
    frame_dt = 0.1 # s
    bag_duration_t_in_s = (self.apa_info_plotter.max_time - self.apa_info_plotter.min_time) / 1e6
    self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description='bag_time(s)', min=0.0, max=bag_duration_t_in_s , value=0.02, step=frame_dt)
    ipywidgets.interact(self.apa_info_plotter.update, bag_time=self.time_slider)


if __name__ == '__main__':
  apa_info_slider = ApaInfoSlider()
