import math
import numpy as np

def rotate(x, y, theta):
    x_rotated = x * math.cos(theta) - y * math.sin(theta)
    y_rotated = x * math.sin(theta) + y * math.cos(theta)
    return x_rotated, y_rotated

def local2global(x, y, ox, oy, otheta):
    tx, ty = rotate(x, y, otheta)
    return (tx+ox, ty+oy)

def global2local(x, y, ox, oy, otheta):
    x1 = x-ox
    y1 = y-oy
    tx, ty = rotate(x1, y1, -otheta)
    return (tx, ty)

def getposbodyandworld(ref_x_vec, ref_y_vec, cur_pos_xn, cur_pos_yn, cur_yaw, cur_pos_xn0, cur_pos_yn0):
    ref_xn_vec = []
    ref_yn_vec = []
    for i in range(len(ref_x_vec)):
        ref_xn_vec.append(ref_x_vec[i] - cur_pos_xn0)
        ref_yn_vec.append(ref_y_vec[i] - cur_pos_yn0)

    ref_xb_vec = []
    ref_yb_vec = []
    for i in range(len(ref_xn_vec)):
        tmpx, tmpy = global2local(ref_x_vec[i], ref_y_vec[i], cur_pos_xn, cur_pos_yn, cur_yaw)
        ref_xb_vec.append(tmpx)
        ref_yb_vec.append(tmpy)

    return ref_xn_vec, ref_yn_vec, ref_xb_vec, ref_yb_vec
  
def gen_line(c0, c1, c2, c3, start, end):
    points_x = []
    points_y = []

    for x in np.linspace(start, end, 50):
        y = c0 + c1 * x + c2 * x * x + c3 * x * x* x
        points_x.append([x])
        points_y.append([y])

    return points_x, points_y