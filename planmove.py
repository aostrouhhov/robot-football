# Made according to this (many thanks):
# https://www.youtube.com/watch?v=Mdg9ElewwA0&feature=emb_logo

import math
import time

import cv2

from barriers_operations import generate_barriers, move_barriers
from constants import *
from obstacle_avoidance import *
from obstacle_detection.mser import MSERObstacleDetector
from obstacle_detection.scale_based import ScaleBasedObstacleDetector
from utils import move_to_dot, move_to_dot_again, set_new_position, calculate_closest_obstacle_distance, draw_scene, \
    cast_detector_coordinates

obstacle_avoidance = dwa
obstacle_detection = MSERObstacleDetector()
# obstacle_detection = ScaleBasedObstacleDetector('SURF')


def main():
    # Init states
    x, y, theta = x_start, y_start, theta_start
    vl, vr = vl_start, vr_start
    ro, alpha, beta = ro_start, alpha_start, beta_start

    # 9 obstacles and 1 target
    barriers, target_index = generate_barriers(9)

    # Used for displaying a trail of the robot's positions
    location_history = []

    # We will calculate time
    start_time = time.time()

    target_x = x
    target_y = y

    barriers_predicted_positions = []
    ball_predicted_positions = []

    # Main loop
    while True:
        screen, screen_picture = draw_scene(
            size, location_history, barriers, target_index, x, y, theta,
            ball_predicted_positions, barriers_predicted_positions
        )
        cv2.imshow('robot football', screen)

        # For display of trail
        location_history.append((x, y))

        # Identify a ball and players positions
        ball_predicted_positions, barriers_predicted_positions = obstacle_detection.forward(
            screen_picture, [(red, 1), (lightblue, 9)]
        )
        ball_predicted_positions = cast_detector_coordinates(ball_predicted_positions)
        barriers_predicted_positions = cast_detector_coordinates(barriers_predicted_positions)

        # Planning
        #
        # Call obstacle avoidance algorithm and move to returned dot.
        #
        # At the moment it has the same call rate as simulation update rate:
        # it is called each quantum of time as the simulation updates.
        #
        # If obstacle_avoidance() call rate will be different than simulation update rate
        # then move_to_dot_again() should be called instead of obstacle_avoidance() and move_to_dot()
        # in this 'while' cycle if time of caliing obstacle_avoidance() is not reached yet.

        target_x, target_y, target_theta = obstacle_avoidance(x, y, vl, vr, theta, ball_predicted_positions, barriers_predicted_positions)
        vl, vr, ro, alpha, beta = move_to_dot(target_x, target_y, x, y, ball_predicted_positions[0][0], ball_predicted_positions[0][1], target_theta)

        # Actually now move robot based on chosen vl and vr
        (x, y, theta) = set_new_position(vl, vr, x, y, target_theta, dt)

        barriers = move_barriers(dt, barriers, target_index)

        # printBarriers()

        # Check collision
        dist_to_obstacle = calculate_closest_obstacle_distance(x, y, barriers, target_index)
        dist_to_target = math.sqrt((x - barriers[target_index][0]) ** 2 + (y - barriers[target_index][1]) ** 2)
        if dist_to_obstacle < 0.001 or dist_to_target < BARRIERRADIUS + ROBOTRADIUS + 0.02:
            if dist_to_obstacle < 0.001:
                print("Crash!")
            print("Result:", time.time() - start_time, "sec")
            while cv2.getWindowProperty('robot football', cv2.WND_PROP_VISIBLE) == 1:
                cv2.waitKey(int(dt * 10))
            break

        # Sleeping dt here runs simulation in real-time
        cv2.waitKey(int(dt * 10))
        if cv2.getWindowProperty('robot football', cv2.WND_PROP_VISIBLE) < 1:
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
