import constants
import math


def dump_obstacle_avoidance(robot, ball, obstacles, ball_predicted_positions, obstacles_predicted_positions):
    robot_position = robot.get_pos()
    goal_angle = robot.goal_angle(ball)
    print("Obstacles in way: " + str(obstacles_in_way(robot, goal_angle, obstacles)))
    print("Translation velocity: " + str(compute_translation(robot, obstacles)))
    print("Goal seek rotation velocity: " + str(compute_goal_seek_rot(goal_angle)))
    print("RWF rotation velocity: " + str(compute_rwf_rot(robot, obstacles)))
    return ball_predicted_positions[0]


def obstacles_in_way(robot, goal_angle, obstacles):
    angle_from_goal = goal_angle - math.pi / 2.0
    if angle_from_goal < -math.pi:
        angle_from_goal += 2 * math.pi
    angle_to_goal = goal_angle + math.pi / 2.0
    if angle_to_goal > math.pi:
        angle_to_goal -= 2 * math.pi
    min_sonar_value = robot.min_range(obstacles, angle_from_goal, angle_to_goal)
    return min_sonar_value < 0.2


def compute_translation(robot, obstacles):
    min_sonar_front = robot.min_range(obstacles, -math.pi / 2, math.pi / 2)
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
    if max(min_right, min_left) < 0.2:
        return 400
    else:
        desired_turn = (400 - min_right) * 2
        desired_turn = max(-400, min(desired_turn, 400))
        return desired_turn
