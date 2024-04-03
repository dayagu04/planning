import math
import numpy as np

class coord_transformer:
    def __init__(self) -> None:
        self.cur_pos_xn = 0.0
        self.cur_pos_yn = 0.0
        self.cur_yaw = 0.0

    def set_info(self, cur_pos_xn, cur_pos_yn, cur_yaw):
        self.cur_pos_xn = cur_pos_xn
        self.cur_pos_yn = cur_pos_yn
        self.cur_yaw = cur_yaw

    def local_to_global(self, local_x_vec, local_y_vec):
        global_x_vec = []
        global_y_vec = []
        tmp_x = 0.0
        tmp_y = 0.0
        for i in (range(len(local_x_vec))):
            tmp_x, tmp_y = local2global(local_x_vec[i], local_y_vec[i], self.cur_pos_xn, self.cur_pos_yn, self.cur_yaw)
            global_x_vec.append(tmp_x)
            global_y_vec.append(tmp_y)

        return global_x_vec, global_y_vec

    def global_to_local(self, global_x_vec, global_y_vec):
        local_x_vec = []
        local_y_vec = []
        local_x_vec, local_y_vec = global2local(np.array(global_x_vec), np.array(global_y_vec), self.cur_pos_xn, self.cur_pos_yn, self.cur_yaw)
        return local_x_vec.tolist(), local_y_vec.tolist()

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

