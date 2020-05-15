# https://sci-hub.se/10.1023/A:1020564024509

import math
import numpy
from constants import *


def dump_obstacle_avoidance(robot_position, ball_predicted_positions, obstacles_predicted_positions):
    return ball_predicted_positions[0]


m = 2
n = 2

alpha_p = 2.3  # F_att_1_coeff

alpha_v = 1.3  # F_att_2_coeff

F_rep_1_coeff = 0.5
F_rep_2_coeff = 1.7

glob_prev_p = None
glob_prev_p_tar = None
glob_prev_obstacles = None

ro_0 = 3 * UNITS_RADIUS
eta = 2

a_max = 1  # ROBOT_MAX_VELOCITY
# obstacle_distance_to_avoid = 2.0

def pfm_obstacle_avoidance(robot_position, ball_predicted_positions, obstacles_predicted_positions):
    global glob_prev_p
    global glob_prev_p_tar
    global glob_prev_obstacles
    p = numpy.array([robot_position[0], robot_position[1]])
    p_tar = numpy.array([ball_predicted_positions[0][0], ball_predicted_positions[0][1]])
    obstacles_predicted_positions = list(map(lambda b: numpy.array([b[0], b[1]]), obstacles_predicted_positions))

    prev_p = glob_prev_p
    prev_p_tar = glob_prev_p_tar
    prev_barriers = glob_prev_obstacles

    if prev_p is None:
        prev_p = p
        prev_p_tar = p_tar
        prev_barriers = obstacles_predicted_positions

    v = p - prev_p
    v_tar = p_tar - prev_p_tar

    target_vec = F_att(p, p_tar, v, v_tar)

    for obstacle in obstacles_predicted_positions:
        # if numpy.linalg.norm(p - obstacle) < obstacle_distance_to_avoid:
        prev_p_obs = min(prev_barriers, key=lambda bar: numpy.linalg.norm(obstacle - bar))
        v_obs = obstacle - prev_p_obs
        target_vec += F_rep(p, numpy.array([obstacle[0], obstacle[1]]), v, v_obs)

    glob_prev_p = p
    glob_prev_p_tar = p_tar
    glob_prev_obstacles = obstacles_predicted_positions

    target_vec *= ROBOT_MAX_VELOCITY
    target_vec_norm = numpy.linalg.norm(target_vec)
    # if target_vec_norm > 2.9:
    target_vec = 2.2 * target_vec / target_vec_norm
    return robot_position[0] + target_vec[0], robot_position[1] + target_vec[1]


def n_vec(start, end):
    return (end - start) / numpy.linalg.norm(end - start)


def F_att(p, p_tar, v, v_tar):
    v_norm = numpy.linalg.norm(v_tar - v)
    p_scale = m * alpha_p * (numpy.linalg.norm(p_tar - p)) ** (m - 1)
    v_scale = n * alpha_v * v_norm ** (n - 1)
    n_rt = n_vec(p, p_tar)
    F_att_1 = p_scale * n_rt
    # F_att_2 = v_scale * n_VRT(v, v_tar) if v_norm != 0 else 0
    if v_norm == 0:
        F_att_2 = 0
    else:
        n_vrt = n_vec(v, v_tar)
        # n_v = v / numpy.linalg.norm(v)
        if n_rt.dot(n_vrt) < 0:
            x = n_rt[0]
            y = n_rt[1]
            a = n_vrt[0]
            b = n_vrt[1]
            new_n_vrt = numpy.array([a * (y ** 2 - x ** 2) - 2 * b * x * y, b * (x ** 2 - y ** 2) - 2 * a * x * y])
            F_att_2 = v_scale * new_n_vrt
        else:
            F_att_2 = v_scale * n_vrt

    return F_att_1 + F_att_2


def F_rep(p, p_obs, v, v_obs):
    n_ro = n_vec(p, p_obs)
    v_ro = (v - v_obs).dot(n_ro)
    if v_ro <= 0.:
        return numpy.array([0.0, 0.0])
    ro_m = v_ro ** 2 / a_max
    ro_s = numpy.linalg.norm(p_obs - p) - 2 * UNITS_RADIUS
    if ro_s - ro_m >= ro_0:
        return numpy.array([0.0, 0.0])

    F_rep_1 = F_rep_1_coeff * (-eta) / ((ro_s - ro_m) ** 2) * (1 + v_ro / a_max) * n_vec(p, p_obs)
    F_rep_2 = F_rep_2_coeff * eta * v_ro / (ro_s * a_max * (ro_s - ro_m) ** 2) * (v - v_obs - v_ro * n_vec(p, p_obs))
    return F_rep_1 + F_rep_2
