import rosbag
import json
import numpy
import math
import os
import sys
sys.path.append("..")

def LoadScalar(ctrl_data, json_struct, name):
  try:
    ctrl_data[name] = json_struct[name]
  except:
    ctrl_data[name] = 0.0

def LoadVector(ctrl_data, json_struct, name, N):
  try:
    data_list = json_struct[name].split(',')
    ctrl_data[name] = list(map(float, data_list))
    return 1
  except:
    tmp = []
    for i in range(N):
      tmp.append(0.0)

    ctrl_data[name] = tmp
    return 0