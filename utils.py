import math
import numpy

import constants

from models import Robot


def calculate_xi_vector(v, omega, theta):
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


def move_to_dot(target_x, target_y, robot_x, robot_y, ball_x, ball_y, theta):
    # x and y here are robot coordinates
    dx = target_x - robot_x
    dy = target_y - robot_y

    ro_new = math.sqrt(dx ** 2 + dy ** 2)
    alpha_new = -theta + math.atan2(dy, dx)
    beta_new = -theta - alpha_new

    v = constants.k_ro * ro_new
    omega = constants.k_alpha * alpha_new + constants.k_beta * beta_new

    ksi = calculate_xi_vector(v, omega, theta)
    vl_chosen, vr_chosen = calculate_phi_vector(ksi, theta)

    # We should move faster if target is ball and it is close to our robot
    if target_x == ball_x and target_y == ball_y:
        dist_to_target = math.sqrt((robot_x - target_x) ** 2 + (robot_y - target_y) ** 2)
        if dist_to_target < Robot.RADIUS + 1.5:
            vl_chosen = vl_chosen + constants.ROBOT_MAX_VELOCITY
            vr_chosen = vr_chosen + constants.ROBOT_MAX_VELOCITY

    if vl_chosen > constants.ROBOT_MAX_VELOCITY or vr_chosen > constants.ROBOT_MAX_VELOCITY:
        if vl_chosen > vr_chosen:
            diff = vr_chosen / vl_chosen
            vl_chosen = constants.ROBOT_MAX_VELOCITY
            vr_chosen = vl_chosen * diff
        elif vr_chosen > vl_chosen:
            diff = vl_chosen / vr_chosen
            vr_chosen = constants.ROBOT_MAX_VELOCITY
            vl_chosen = vr_chosen * diff
        else:
            vl_chosen = constants.ROBOT_MAX_VELOCITY
            vr_chosen = constants.ROBOT_MAX_VELOCITY

    return vl_chosen, vr_chosen, ro_new, alpha_new, beta_new


def move_to_dot_again(ro, alpha, beta, theta, dt):
    change_rate = numpy.array(
        [
            [-constants.k_ro * ro * math.cos(alpha)],
            [constants.k_ro * math.sin(alpha) - constants.k_alpha * alpha - constants.k_beta * beta],
            [-constants.k_ro * math.sin(alpha)],
        ]
    )

    ro_new = ro + change_rate[0][0] * dt
    alpha_new = alpha + change_rate[1][0] * dt
    beta_new = beta + change_rate[2][0] * dt

    v = constants.k_ro * ro_new
    omega = constants.k_alpha * alpha_new + constants.k_beta * beta_new

    ksi = calculate_xi_vector(v, omega, theta)
    vl_chosen, vr_chosen = calculate_phi_vector(ksi, theta)

    return vl_chosen, vr_chosen, ro_new, alpha_new, beta_new


def cast_detector_coordinates(coords):
    local_coords = coords.copy()
    # shift
    local_coords[:, 0] = local_coords[:, 0] - constants.WINDOW_WIDTH / 2
    local_coords[:, 1] = constants.WINDOW_HEIGHT / 2 - local_coords[:, 1]
    # scale
    local_coords = local_coords / constants.k
    return local_coords
