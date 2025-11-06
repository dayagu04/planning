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

def GetProtoOdTrajs(planning_proto):
  trajs = []
  speed_debug = planning_proto.apa_speed_debug
  for i in range(len(speed_debug.predict_traj_set.trajs)):
    proto_traj = speed_debug.predict_traj_set.trajs[i]
    traj = []
    for j in range(len(proto_traj.point)):
      traj.append([proto_traj.point[j].x, proto_traj.point[j].y])

    trajs.append(traj)

  return trajs

# Define the JavaScript callback code
callback_code = """
    var x = cb_obj.x;
    var y = cb_obj.y;

    source.data['x'].push(x);
    source.data['y'].push(y);

    if (source.data['x'].length > 2) {
        source.data['x'].shift();
        source.data['y'].shift();
        source.data['x'].shift();
        source.data['y'].shift();
    }
    source.change.emit();

    if (source.data['x'].length >= 2) {
        var x1 = source.data['x'][source.data['x'].length - 2];
        var y1 = source.data['y'][source.data['y'].length - 2];
        var x2 = x;
        var y2 = y;
        var x3 = (x1 + x2) / 2;
        var y3 = (y1 + y2) / 2;

        var distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

        console.log("Distance between the last two points: " + distance);

        distance = distance.toFixed(4);
        text_source.data = {'x': [x3], 'y': [y3], 'text': [distance]};
        text_source.change.emit();

        line_source.data = {'x': [x1, x2], 'y': [y1, y2]};
        line_source.change.emit();
    }

    if (source.data['x'].length == 1) {
        text_source.data['x'].shift();
        text_source.data['y'].shift();
        text_source.data['text'].shift();
    }
    text_source.change.emit();
"""