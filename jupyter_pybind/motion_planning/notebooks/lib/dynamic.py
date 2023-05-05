import numpy as np
import math
def dynamic(x, u):
    # x[0] is x, x[1] is y, x[2] is v, x[3] is theta, x[4] is acc, x[5] is delta, u[0] is acc_dot, u[1] is delta_dot
    curv_factor = 0.22
    dx = np.zeros(6)
    dx[0] = x[2] * math.cos(x[3])
    dx[1] = x[2] * math.sin(x[3])
    dx[2] = x[4]
    dx[3] = curv_factor * x[2] * math.tan(x[5])
    dx[4] = u[0]
    dx[5] = u[1]
    return dx

def updateDynamicRK4(x, u):
    dt = 0.1
    k1 = dynamic(x, u)
    k2 = dynamic(x + k1 * dt / 2.0, u)
    k3 = dynamic(x + k2 * dt / 2.0, u)
    k4 = dynamic(x + k3 * dt, u)
    x_result = x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt / 6.0
    return x_result

def updateDynamicEular(x, u):
    dt = 0.1
    x_result = x + dynamic(x, u) * dt
    return x_result

def calculateBound(x_init, y_init, v_init, a_init, yaw_init, curv_init, jerk_bound, delta_dot_bound):

    curv_factor = 0.22
    jerk_min = -jerk_bound
    jerk_max = jerk_bound
    delta_dot_min = -delta_dot_bound
    delta_dot_max = delta_dot_bound
    u_min = np.array([jerk_min, delta_dot_min])
    u_max = np.array([jerk_max, delta_dot_max])
    if(v_init < 0.5):
        delta_init = 0.0
    else:
        delta_init = math.atan(curv_init / curv_factor)
    x0 = np.array([x_init, y_init, v_init, yaw_init, a_init, delta_init])
    lower_bound = updateDynamicEular(x0, u_min)
    upper_bound = updateDynamicEular(x0, u_max)
    bound = {
        "acc":[lower_bound[4], upper_bound[4]], 
        "vel":[lower_bound[2], upper_bound[2]], 
        "yaw":[lower_bound[3], upper_bound[3]],
        "curv":[curv_factor * math.tan(lower_bound[5]), curv_factor * math.tan(upper_bound[5])],
        "xy_area": {
            "x": [],
            "y": []
        }
    }
    for i in [delta_dot_min, delta_dot_max]:
        for j in [jerk_min, jerk_max]:
            u = np.array([j, i])
            state = updateDynamicEular(x0, u)
            bound["xy_area"]["x"].append(state[0])
            bound["xy_area"]["y"].append(state[1])
    return bound


def calculateBound2(x_init, y_init, v_init, a_init, yaw_init, curv_init, jerk_bound, delta_dot_bound):
    dt = 0.1
    curv_factor = 0.22
    jerk_min = -jerk_bound
    jerk_max = jerk_bound
    delta_dot_min = -delta_dot_bound
    delta_dot_max = delta_dot_bound
    if(v_init < 0.5):
        delta_init = 0.0
    else:
        delta_init = math.atan(curv_init / curv_factor)
    
    
    acc_min = a_init + dt * jerk_min
    acc_max = a_init + dt * jerk_max
    vel_min = v_init + dt * acc_min
    vel_max = v_init + dt * acc_max
    delta_min = delta_init + dt * delta_dot_min
    delta_max = delta_init + dt * delta_dot_max
    curv_min = curv_factor * math.tan(delta_min)
    curv_max = curv_factor * math.tan(delta_max)
    yaw_min = yaw_init + dt * curv_min * vel_min
    yaw_max = yaw_init + dt * curv_max * vel_max   
    
    
    bound = {
        "acc":[acc_min, acc_max], 
        "vel":[vel_min, vel_max], 
        "yaw":[yaw_min, yaw_max],
        "curv":[curv_min, curv_max],
        "xy_area": {
            "x": [x_init + dt * vel_min * math.cos(yaw_min), x_init + dt * vel_max * math.cos(yaw_min),
                  x_init + dt * vel_max * math.cos(yaw_max), x_init + dt * vel_min * math.cos(yaw_max)],
            "y": [y_init + dt * vel_min * math.sin(yaw_min), y_init + dt * vel_max * math.sin(yaw_min), 
                  y_init + dt * vel_max * math.sin(yaw_max), y_init + dt * vel_min * math.sin(yaw_max)]
        }
    }
    return bound

def calculateNextStateWithZeroInput(x_init, y_init, v_init, a_init, yaw_init, curv_init):
    
    dt = 0.1
    curv_factor = 0.22
    if(v_init < 0.5):
        delta_init = 0.0
    else:
        delta_init = math.atan(curv_init / curv_factor)
    acc = a_init
    vel = v_init + dt * acc
    delta = delta_init
    curv = curv_factor * math.tan(delta)
    yaw = yaw_init + dt * curv * vel
    x = x_init + dt * vel * math.cos(yaw)
    y = y_init + dt * vel * math.sin(yaw)
    
    
    
    state = {
        "acc":acc, 
        "vel":vel, 
        "yaw":yaw,
        "curv":curv,
        "xy_area": {
            "x": x,
            "y": y
        }
    }
    return state