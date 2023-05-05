import rosbag
import json
import numpy
import math
import os
import sys
sys.path.append("..")
from lib.load_rotate import *
from lib.load_ctrl_json import *

class LoadRosbag:
  def __init__(self, path) -> None:
    self.bag = rosbag.Bag(path)
    self.ego_info =  {'t':[], 'data_global':[], 'data_local':[]}
    self.plan_info = {'t':[], 'data':[]}
    self.ctrl_info = {'t':[], 'data':[]}
    self.act_info = {'t':[], 'data':[]}
    self.lane_info = {'t':[], 'data':[]}

  def LoadEgoInfo(self):

    for topic, msg, t in self.bag.read_messages(topics='/mla/egopose'):
        self.ego_info['t'].append(msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9)
        ego_global = {}
        ego_global['x'] = msg.position.position_local.x
        ego_global['y'] = msg.position.position_local.y
        ego_global['yaw'] = msg.orientation.euler_local.yaw
        self.ego_info['data_global'].append(ego_global)

    self.ego_info['t'] = [tmp - self.ego_info['t'][0]  for tmp in self.ego_info['t']]

    return self.ego_info

  def LoadPlanInfo(self):

    mode = 'hcp'

    plan_path,plan_vel,plan_acc = [], [], []
    for topic, msg, t in self.bag.read_messages(topics="/msd/planning/plan"):
      self.plan_info['t'].append(msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9)
      plan_path.append(msg.trajectory.path)
      plan_vel.append(msg.trajectory.velocity)
      plan_acc.append(msg.trajectory.acceleration)
    for i in range(len(plan_path)):
        temp = []
        if mode =='apa':
          N = len(plan_path[i])
        else:
          N = min(len(plan_path[i]), len(plan_vel[i].vel_points), len(plan_acc[i].acc_points))
        for j in range(N):
            traj = {}
            traj['x'] = plan_path[i][j].position_enu.x
            traj['y'] = plan_path[i][j].position_enu.y
            traj['yaw'] = plan_path[i][j].heading_yaw
            traj['curv'] = plan_path[i][j].curvature
            if mode =='apa':
              traj['vel'] = plan_vel[i].target_value
              traj['acc'] =0.0
              traj['relative_time'] = 0.0
              traj['distance'] = 0.0
            else:
              traj['vel'] = plan_vel[i].vel_points[j].target_velocity
              traj['acc'] = plan_acc[i].acc_points[j].acc
              traj['relative_time'] = plan_vel[i].vel_points[j].relative_time
              traj['distance'] = plan_vel[i].vel_points[j].distance
            temp.append(traj)
        self.plan_info['data'].append(temp)

    self.plan_info['t'] = [tmp - self.plan_info['t'][0]  for tmp in self.plan_info['t']]
    return self.plan_info

  def LoadCtrlInfo(self):
    mode = 'hcp'
    for topic, msg, t in self.bag.read_messages(topics="/msd/endpoint/control_command"):
      # json_struct = json.loads(msg.extra.json)
      try:
          json_struct = json.loads(msg.extra.json, strict = False)
      except json.decoder.JSONDecodeError as jserr:
          print('except',jserr)
      else:
        ctrl_data = {}

        LoadScalar(ctrl_data, json_struct, 'time_delay')
        LoadScalar(ctrl_data, json_struct, 'mpc_delta_cmd')
        LoadScalar(ctrl_data, json_struct, 'mpc_vel_cmd')
        LoadScalar(ctrl_data, json_struct, 'mpc_acc_cmd')
        LoadScalar(ctrl_data, json_struct, 'ego_yaw')
        LoadScalar(ctrl_data, json_struct, 'cur_pos_x')
        LoadScalar(ctrl_data, json_struct, 'cur_pos_y')
        LoadScalar(ctrl_data, json_struct, 'd_rear_to_cent')
        LoadScalar(ctrl_data, json_struct, 'vel_lat_min')
        LoadScalar(ctrl_data, json_struct, 'cur_pos_cent_x')
        LoadScalar(ctrl_data, json_struct, 'cur_pos_cent_y')
        LoadScalar(ctrl_data, json_struct, 'vel_ego')
        LoadScalar(ctrl_data, json_struct, 'steering_angle')
        LoadScalar(ctrl_data, json_struct, 'curv_factor')
        LoadScalar(ctrl_data, json_struct, 'remain_s_uss')
        LoadScalar(ctrl_data, json_struct, 'remain_s_plan_rt')
        LoadScalar(ctrl_data, json_struct, 'wheel_angle_cmd')
        LoadScalar(ctrl_data, json_struct, 'osqp_solver_flag')

        LoadScalar(ctrl_data, json_struct, 'q0')
        LoadScalar(ctrl_data, json_struct, 'q1')
        LoadScalar(ctrl_data, json_struct, 'r')
        LoadScalar(ctrl_data, json_struct, 'q0_base')
        LoadScalar(ctrl_data, json_struct, 'q1_base')
        LoadScalar(ctrl_data, json_struct, 'r_base')
        LoadScalar(ctrl_data, json_struct, 'mpc_curv_factor')

        LoadScalar(ctrl_data, json_struct, 'wheel_angle_max')
        LoadScalar(ctrl_data, json_struct, 'wheel_angle_rate_limit')

        LoadScalar(ctrl_data, json_struct, 'vel_measure')
        LoadScalar(ctrl_data, json_struct, 'reverse_flag')
        LoadScalar(ctrl_data, json_struct, 'target_velocity_apa')
        LoadScalar(ctrl_data, json_struct, 'remain_s')
        LoadScalar(ctrl_data, json_struct, 'remain_s_plan')
        LoadScalar(ctrl_data, json_struct, 'parking_slot_iou')
        LoadScalar(ctrl_data, json_struct, 'throttle')
        LoadScalar(ctrl_data, json_struct, 'stopper_limit_timer')
        LoadScalar(ctrl_data, json_struct, 'slope_acc')
        LoadScalar(ctrl_data, json_struct, 'current_acc_req')
        LoadScalar(ctrl_data, json_struct, 'current_status')

        LoadScalar(ctrl_data, json_struct, 'vel_tgt_plan')
        LoadScalar(ctrl_data, json_struct, 'remain_s_uss')
        LoadScalar(ctrl_data, json_struct, 'pasuse_lat_ctrl')
        LoadScalar(ctrl_data, json_struct, 'pasuse_lon_ctrl')
        LoadScalar(ctrl_data, json_struct, 'current_pose_x')
        LoadScalar(ctrl_data, json_struct, 'current_pose_y')
        LoadScalar(ctrl_data, json_struct, 'current_heading')
        LoadScalar(ctrl_data, json_struct, 'vel_measure')
        LoadScalar(ctrl_data, json_struct, 'acc_measure_vel')
        LoadScalar(ctrl_data, json_struct, 'parking_slot_iou')
        LoadScalar(ctrl_data, json_struct, 'gear_change_flag')
        LoadScalar(ctrl_data, json_struct, 'is_path_new')
        LoadScalar(ctrl_data, json_struct, 'remain_s_plan_rt')
        LoadScalar(ctrl_data, json_struct, 'decel_flag')
        LoadScalar(ctrl_data, json_struct, 'impact_flag')

        flag = 1

        flag = flag & LoadVector(ctrl_data, json_struct, 'dx_ref_vec_mpc', 26)
        flag = flag & LoadVector(ctrl_data, json_struct, 'dy_ref_vec_mpc', 26)
        flag = flag & LoadVector(ctrl_data, json_struct, 'dphi_ref_vec_mpc', 26)
        flag = flag & LoadVector(ctrl_data, json_struct, 'vel_ref_vec_mpc', 26)
        flag = flag & LoadVector(ctrl_data, json_struct, 'dx_vec_mpc', 26)
        flag = flag & LoadVector(ctrl_data, json_struct, 'dy_vec_mpc', 26)
        flag = flag & LoadVector(ctrl_data, json_struct, 'dphi_vec_mpc', 26)
        flag = flag & LoadVector(ctrl_data, json_struct, 'delta_angle_vec_mpc', 26)
        flag = flag & LoadVector(ctrl_data, json_struct, 'delta_rate_vec_mpc', 26)
        flag = flag & LoadVector(ctrl_data, json_struct, 'curv_ref_factor', 26)


        LoadScalar(ctrl_data, json_struct, 'mpc_init_dx')
        LoadScalar(ctrl_data, json_struct, 'mpc_init_dy')
        LoadScalar(ctrl_data, json_struct, 'mpc_init_dphi')
        LoadScalar(ctrl_data, json_struct, 'mpc_init_delta')
        LoadScalar(ctrl_data, json_struct, 'vel_measure')
        LoadScalar(ctrl_data, json_struct, 'osqp_solver_flag')

        flag = flag & LoadVector(ctrl_data, json_struct, 'x_vec', 205)
        flag = flag & LoadVector(ctrl_data, json_struct, 'y_vec', 205)
        flag = flag & LoadVector(ctrl_data, json_struct, 'yaw_vec', 205)
        flag = flag & LoadVector(ctrl_data, json_struct, 'vel_vec', 205)
        flag = flag & LoadVector(ctrl_data, json_struct, 'acc_vec', 205)
        flag = flag & LoadVector(ctrl_data, json_struct, 'curv_vec', 205)
        flag = flag & LoadVector(ctrl_data, json_struct, 'relative_time_vec', 205)
        flag = flag & LoadVector(ctrl_data, json_struct, 'distance_vec', 205)

        # if flag == 0:
        #   break

        self.ctrl_info['t'].append(msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9)
        self.ctrl_info['data'].append(ctrl_data)


    self.ctrl_info['t'] = [tmp - self.ctrl_info['t'][0]  for tmp in self.ctrl_info['t']]

    return self.ctrl_info

  def LoadActuatorInfo(self):

    for topic, msg, t in self.bag.read_messages(topics="/msd/endpoint/control_command"):
      self.act_info['t'].append(msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9)
      json_struct = json.loads(msg.extra.json)
      act_data = {}
      act_data['vel_ctrl_cmd'] = json_struct['vel_ctrl_cmd']
      act_data['vel_ref'] = json_struct['vel_ref']
      act_data['vel_ego'] = json_struct['vel_ego']
      act_data['accelaration'] = json_struct['accelaration']
      act_data['acc_ref'] = json_struct['acc_ref']
      act_data['acc_wheel'] = json_struct['acc_wheel']
      act_data['acc_ego'] = json_struct['acc_ego']
      act_data['slope_acc'] = json_struct['vel_dob_prior_dist']
      act_data['throttle'] = json_struct['throttle']
      act_data['brake'] = json_struct['brake']
      act_data['steering_angle'] = json_struct['steering_angle']

      self.act_info['data'].append(act_data)

    self.act_info['t'] = [tmp - self.act_info['t'][0]  for tmp in self.act_info['t']]

    return self.act_info

  def LoadLaneInfo(self):
    ego_t, ego_x, ego_y, ego_yaw = [], [], [], []
    for topic, msg, t in self.bag.read_messages(topics='/mla/egopose'):
      ego_t.append(msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9)
      ego_x.append(msg.position.position_local.x)
      ego_y.append(msg.position.position_local.y)
      ego_yaw.append(msg.orientation.euler_local.yaw)
    for topic, msg, t in self.bag.read_messages(topics='/mla/egopose'):
      ego_x0 = msg.position.position_local.x
      ego_y0 = msg.position.position_local.y
      break

    for topic, msg, t in self.bag.read_messages(topics="/worldmodel/processed_map"):
      lane_t = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9
      self.lane_info['t'].append(lane_t)

      ego_idx = 0
      while ego_t[ego_idx] <= lane_t and ego_idx < (len(ego_t)-1):
          ego_idx = ego_idx + 1

      edges, xsn, ysn, xsb, ysb = [], [], [], [], []
      for lane in msg.processed_map_data.lanes:
        edges.append(lane.left_lane_boundary)
        edges.append(lane.right_lane_boundary)
      step = 10
      for edge in edges:
        for i in range(0,len(edge.points)-step, step):
          x1n = edge.points[i].x - ego_x0
          x2n = edge.points[i+step].x - ego_x0
          y1n = edge.points[i].y - ego_y0
          y2n = edge.points[i+step].y - ego_y0

          x1b, y1b= global2local(edge.points[i].x, edge.points[i].y, ego_x[ego_idx], ego_y[ego_idx], ego_yaw[ego_idx])
          x2b, y2b= global2local(edge.points[i+step].x, edge.points[i+step].y, ego_x[ego_idx], ego_y[ego_idx], ego_yaw[ego_idx])

          xsn.append([x1n,x2n])
          ysn.append([y1n,y2n])
          xsb.append([x1b, x2b])
          ysb.append([y1b, y2b])

      self.lane_info['data'].append((xsn, ysn, xsb, ysb))

    self.lane_info['t'] = [tmp - self.lane_info['t'][0]  for tmp in self.lane_info['t']]

    if not self.lane_info['t']:
        self.lane_info['t'] = [0]
        xsn = [[0, 0]]
        ysn = [[0, 0]]
        xsb = [[0, 0]]
        ysb = [[0, 0]]
        self.lane_info['data'].append((xsn, ysn, xsb, ysb))

    return self.lane_info




