import math
import numpy
import logging

import constants

from models import Robot


def calculate_ksi_vector(v, omega, theta):
    matrix_3 = numpy.array([[math.cos(theta), 0], [math.sin(theta), 0], [0, 1]])

    matrix_4 = numpy.array([[v], [omega]])

    ksi = numpy.dot(matrix_3, matrix_4)
    return ksi


def calculate_phi_vector(ksi, theta):
    matrix_1 = numpy.array([[1, 0, constants.l], [1, 0, -constants.l], [0, 1, 0]])
    matrix_2 = numpy.array([[math.cos(theta), math.sin(theta), 0], [-math.sin(theta), math.cos(theta), 0], [0, 0, 1]])

    res = 1 / constants.r * numpy.dot(matrix_1, matrix_2)
    phi = numpy.dot(res, ksi)

    return phi[1][0], phi[0][0]


def move_to_dot(robot, ball, target):
    robot_x, robot_y = robot.get_pos()
    target_x, target_y = target
    ball_x, ball_y = ball.get_pos()
    #

    target_vec = numpy.array([target_x - robot_x, target_y - robot_y])
    target_vec = rotate_np_vector(target_vec, robot.angle)
    robot_target_angle = math.atan2(target_vec[1], target_vec[0])
    robot_target_angle = _normalize_angle(robot_target_angle)

    v = constants.ROBOT_MAX_VELOCITY
    dist = numpy.linalg.norm(target_vec)

    omega = robot_target_angle / (dist / v)

    # if not (-math.pi / 2 < robot_target_angle < math.pi / 2):
    #     v = -v

    ksi = calculate_ksi_vector(v, omega, robot.angle)
    vl_chosen, vr_chosen = calculate_phi_vector(ksi, robot.angle)

    # Applying velocity limitations
    vl_abs = abs(vl_chosen)
    vr_abs = abs(vr_chosen)
    if vl_abs > constants.ROBOT_MAX_VELOCITY or vr_abs > constants.ROBOT_MAX_VELOCITY:
        if vl_abs > vr_abs:
            diff = vr_abs / vl_abs
            vl_abs = constants.ROBOT_MAX_VELOCITY
            vr_abs = vl_abs * diff
        else:
            diff = vl_abs / vr_abs
            vr_abs = constants.ROBOT_MAX_VELOCITY
            vl_abs = vr_abs * diff
    vl_chosen = vl_abs * _get_sign(vl_chosen)
    vr_chosen = vr_abs * _get_sign(vr_chosen)

    # We should move faster if target is ball and it is close to our robot
    if target_x == ball_x and target_y == ball_y:
        dist_to_target = math.sqrt((robot_x - target_x) ** 2 + (robot_y - target_y) ** 2)
        if dist_to_target < Robot.RADIUS + constants.ROBOT_HUNT_DISTANCE:
            vel_delta = constants.ROBOT_MAX_HUNT_VELOCITY / constants.ROBOT_MAX_VELOCITY
            vl_chosen *= vel_delta
            vr_chosen *= vel_delta

    logging.info(f'move_to_dot: vel left: {vl_chosen}, vel right: {vr_chosen}')
    return vl_chosen, vr_chosen


def cast_detector_coordinates(coords):
    local_coords = coords.copy()
    # shift
    local_coords[:, 0] = local_coords[:, 0] - constants.WINDOW_WIDTH / 2
    local_coords[:, 1] = constants.WINDOW_HEIGHT / 2 - local_coords[:, 1]
    # scale
    local_coords = local_coords / constants.k
    return local_coords


def rotate_np_vector(vec, rad):
    x, y = vec
    cos, sin = numpy.cos(rad), numpy.sin(rad)
    rotation_matrix = numpy.array([[cos, sin], [-sin, cos]])
    result = numpy.dot(rotation_matrix, [x, y])
    return float(result.T[0]), float(result.T[1])


def _get_sign(val):
    return 1 if val > 0 else -1


def _normalize_angle(rad):
    in_circle = (abs(rad) % (2 * math.pi)) * _get_sign(rad)
    if in_circle < - math.pi:
        in_circle += 2 * math.pi
    elif in_circle > math.pi:
        in_circle -= 2 * math.pi
    return in_circle


def to_radians(deg):
    return deg * math.pi / 180


def to_deg(rad):
    return rad * 180 / math.pi


def normalize_np_vector(vec, max_val):
    abs1 = abs(vec[0])
    abs2 = abs(vec[1])
    if abs1 > max_val or abs2 > max_val:
        if abs1 > abs2:
            diff = abs2 / abs1
            abs1 = max_val
            abs2 = abs1 * diff
        else:
            diff = abs1 / abs2
            abs2 = max_val
            abs1 = abs2 * diff
    return numpy.array([abs1 * _get_sign(vec[0]), abs2 * _get_sign(vec[1])])
