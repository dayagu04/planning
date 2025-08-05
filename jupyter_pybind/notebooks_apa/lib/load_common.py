from lib.load_json import *
from lib.load_struct import *
from jupyter_pybind.python_proto import planning_debug_info_pb2


def GetProtoObstacles(planning_proto):
  obstacle_x_list, obstacle_y_list = [], []
  obs_list = planning_proto.apa_path_debug.obs_list
  for i in range(len(obs_list.obs)):
    obs = obs_list.obs[i]
    for j in range(len(obs.points)):
      obstacle_x_list.append(obs.points[j].x)
      obstacle_y_list.append(obs.points[j].y)

  return obstacle_x_list,obstacle_y_list

def GetTrajSpeed(planning):
  traj_speed_profile = []
  print('traj size = ', planning.trajectory.trajectory_points_size)

  for i in range(planning.trajectory.trajectory_points_size):
    point = planning.trajectory.trajectory_points[i]

    speed_point = []
    speed_point.append(point.distance)
    speed_point.append(point.t)
    speed_point.append(point.v)
    speed_point.append(point.a)
    speed_point.append(point.jerk)
    traj_speed_profile.append(speed_point)
    # print('k', point.curvature)

  return traj_speed_profile


def GetProtoObstacles(planning_proto):
  obstacle_x_list, obstacle_y_list = [], []
  obs_list = planning_proto.apa_path_debug.obs_list
  for i in range(len(obs_list.obs)):
    obs = obs_list.obs[i]
    for j in range(len(obs.points)):
      obstacle_x_list.append(obs.points[j].x)
      obstacle_y_list.append(obs.points[j].y)

  return obstacle_x_list,obstacle_y_list


def GetProtoStopSigns(planning_proto):
  speed = planning_proto.apa_speed_debug
  stop_signs = []
  for i in range(len(speed.stop_signs)):
    stop = speed.stop_signs[i]
    pose = stop.stop_pose

    tmp_x, tmp_y = local2global(0, -1.5, pose.x, pose.y, pose.theta)
    stop_sign_vec = []
    stop_sign_vec.append(tmp_x)
    stop_sign_vec.append(tmp_y)

    tmp_x, tmp_y = local2global(0, 1.5, pose.x, pose.y, pose.theta)
    stop_sign_vec.append(tmp_x)
    stop_sign_vec.append(tmp_y)

    stop_signs.append(stop_sign_vec)

  stop_sign_lines_x = []
  stop_sign_lines_y = []
  for k in range(len(stop_signs)):
      stop_sign = stop_signs[k]
      stop_sign_lines_x.append([stop_sign[0], stop_sign[2]])
      stop_sign_lines_y.append([stop_sign[1], stop_sign[3]])

  return stop_sign_lines_x, stop_sign_lines_y