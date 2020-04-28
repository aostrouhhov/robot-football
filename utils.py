import math

import numpy

from constants import *


def set_new_position(v_l, v_r, x, y, theta, delta_t):
    """Set new robot position in simulation
    based on the current pose and velocity controls.

    Uses time delta_t in the future.
    Returns x_new, y_new, theta_new. Also, returns path which is just used for graphics.
    """
    # Simple special cases
    # Straight line motion
    if round(v_l, 3) == round(v_r, 3):
        x_new = x + v_l * delta_t * math.cos(theta)
        y_new = y + v_l * delta_t * math.sin(theta)
        theta_new = theta

    # Pure rotation motion
    elif round(v_l, 3) == -round(v_r, 3):
        x_new = x
        y_new = y
        theta_new = theta + ((v_r - v_l) * delta_t / ROBOTWIDTH)

    else:
        # Rotation and arc angle of general circular motion
        # Using equations given in Lecture 2
        _r = ROBOTWIDTH / 2.0 * (v_r + v_l) / (v_r - v_l)
        delta_theta = (v_r - v_l) * delta_t / ROBOTWIDTH
        x_new = x + _r * (math.sin(delta_theta + theta) - math.sin(theta))
        y_new = y - _r * (math.cos(delta_theta + theta) - math.cos(theta))
        theta_new = theta + delta_theta

    return x_new, y_new, theta_new


# Calculate the closest obstacle at a position (x, y)
def calculate_closest_obstacle_distance(x, y, barriers, target_index):
    closest_dist = 100000.0
    # Calculate distance to closest obstacle
    for i, barrier in enumerate(barriers):
        if i != target_index:
            dx = barrier[0] - x
            dy = barrier[1] - y
            d = math.sqrt(dx ** 2 + dy ** 2)
            # Distance between the closest touching point of circular robot and circular barrier
            dist = d - BARRIERRADIUS - ROBOTRADIUS
            if dist < closest_dist:
                closest_dist = dist
    return closest_dist


def calculate_xi_vector(v, omega, theta):
    matrix_3 = numpy.array([[math.cos(theta), 0], [math.sin(theta), 0], [0, 1]])

    matrix_4 = numpy.array([[v], [omega]])

    ksi = numpy.dot(matrix_3, matrix_4)
    return ksi


def calculate_phi_vector(ksi, theta):
    matrix_1 = numpy.array([[1, 0, l], [1, 0, -l], [0, 1, 0]])

    matrix_2 = numpy.array([[math.cos(theta), math.sin(theta), 0], [-math.sin(theta), math.cos(theta), 0], [0, 0, 1]])

    res = 1 / r * numpy.dot(matrix_1, matrix_2)
    phi = numpy.dot(res, ksi)

    return phi[0][0], phi[1][0]


def move_to_dot(target_x, target_y, robot_x, robot_y, theta):
    # x and y here are robot coordinates
    dx = target_x - robot_x
    dy = target_y - robot_y

    ro_new = math.sqrt(dx ** 2 + dy ** 2)
    alpha_new = -theta + math.atan2(dy, dx)
    beta_new = -theta - alpha_new

    v = k_ro * ro_new
    omega = k_alpha * alpha_new + k_beta * beta_new

    ksi = calculate_xi_vector(v, omega, theta)
    vl_chosen, vr_chosen = calculate_phi_vector(ksi, theta)

    return vl_chosen, vr_chosen, ro_new, alpha_new, beta_new


def move_to_dot_again(ro, alpha, beta, theta, dt):
    change_rate = numpy.array(
        [
            [-k_ro * ro * math.cos(alpha)],
            [k_ro * math.sin(alpha) - k_alpha * alpha - k_beta * beta],
            [-k_ro * math.sin(alpha)],
        ]
    )

    ro_new = ro + change_rate[0][0] * dt
    alpha_new = alpha + change_rate[1][0] * dt
    beta_new = beta + change_rate[2][0] * dt

    v = k_ro * ro_new
    omega = k_alpha * alpha_new + k_beta * beta_new

    ksi = calculate_xi_vector(v, omega, theta)
    vl_chosen, vr_chosen = calculate_phi_vector(ksi, theta)

    return vl_chosen, vr_chosen, ro_new, alpha_new, beta_new


def cast_detector_coordinates(coords):
    assert len(coords.shape) == 2 and coords.shape[1] == 2, f'Expected np.array of shape (N, 2)'
    local_coords = coords.copy()
    # shift
    local_coords[:, 0] = local_coords[:, 0] - WIDTH / 2
    local_coords[:, 1] = local_coords[:, 1] - HEIGHT / 2
    # scale
    local_coords = local_coords / k
    return local_coords
