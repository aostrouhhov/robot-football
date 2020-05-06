import constants
import math
import numpy as np

from models import RobotState

OBSTACLE_DISTANCE_THRESHOLD = 0.5


def dump_obstacle_avoidance(robot, ball, obstacles, ball_predicted_positions, obstacles_predicted_positions):
    robot_position = robot.get_pos()
    goal_angle = robot.goal_angle(ball)
    # print("State: " + str(robot.state))
    # print("Obstacles in way: " + str(obstacles_in_way(robot, goal_angle, obstacles)))
    # print("Translation velocity: " + str(compute_translation(robot, obstacles)))
    # print("Goal seek rotation velocity: " + str(compute_goal_seek_rot(goal_angle)))
    # print("RWF rotation velocity: " + str(compute_rwf_rot(robot, obstacles)))
    return bug2(robot_position, ball.get_pos(), goal_angle, robot, obstacles)


def perpendicular(vector):
    return np.array([-vector[1], vector[0]])


def angle_vector(angle):
    return np.array([math.cos(angle), math.sin(angle)])


def set_velocity(forward_velocity, rotation_velocity, robot_position, robot_angle):
    angle_vec = angle_vector(robot_angle)
    perpend = perpendicular(angle_vec)
    if rotation_velocity == 400:
        forward_velocity = 0
    rotation_velocity /= 80
    forward_velocity *= 5
    result = np.array(robot_position) + angle_vec * forward_velocity + perpend * rotation_velocity
    return result


def bug2(robot_position, goal_position, goal_angle, robot, obstacles):
    forward_velocity = compute_translation(robot, obstacles)
    if robot.state == RobotState.GOALSEEK:
        rotation_velocity = compute_goal_seek_rot(goal_angle)
        if obstacles_in_way(robot, goal_angle, obstacles):
            robot.state = RobotState.WALLFOLLOW
    else:
        rotation_velocity = compute_rwf_rot(robot, obstacles)
        if not obstacles_in_way(robot, goal_angle, obstacles):
            robot.state = RobotState.GOALSEEK
    return set_velocity(forward_velocity, rotation_velocity, robot_position, robot.angle)


def obstacles_in_way(robot, goal_angle, obstacles):
    angle_from_goal = goal_angle - 5 * math.pi / 12.0
    if angle_from_goal < -math.pi:
        angle_from_goal += 2 * math.pi
    angle_to_goal = goal_angle + 5 * math.pi / 12.0
    if angle_to_goal > math.pi:
        angle_to_goal -= 2 * math.pi
    min_sonar_value = robot.min_range(obstacles, angle_from_goal, angle_to_goal)
    return min_sonar_value < OBSTACLE_DISTANCE_THRESHOLD


def compute_translation(robot, obstacles):
    min_sonar_front = robot.min_range(obstacles, (-5) * math.pi / 12, 5 * math.pi / 12)
    if min_sonar_front < 0.2:
        return 0
    else:
        return min(0.5, min_sonar_front - 0.2)


def compute_goal_seek_rot(goal_angle):
    if abs(goal_angle) < math.pi / 10:
        return 0
    else:
        return goal_angle * 100


def compute_rwf_rot(robot, obstacles):
    min_right = robot.min_range(obstacles, -math.pi / 2, 0)
    min_left = robot.min_range(obstacles, 0, math.pi / 2)
    if min(min_right, min_left) < OBSTACLE_DISTANCE_THRESHOLD:
        return 400 if min_right < min_left else -400
    else:
        desired_turn = (400 - min_right) * 2
        desired_turn = max(-400, min(desired_turn, 400))
        return desired_turn
