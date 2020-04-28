# Made according to this (many thanks):
# https://www.youtube.com/watch?v=Mdg9ElewwA0&feature=emb_logo

import math
import time

import pygame

from barriers_operations import generate_barriers, move_barriers
from constants import *
from obstacle_avoidance import dump_obstacle_avoidance
from obstacle_detection.mser import MSERObstacleDetector
from utils import move_to_dot, move_to_dot_again, set_new_position, calculate_closest_obstacle_distance, draw_scene, \
    cast_detector_coordinates

obstacle_avoidance = dump_obstacle_avoidance
obstacle_detection = MSERObstacleDetector()


def main():
    # Init states
    x, y, theta = x_start, y_start, theta_start
    vl, vr = vl_start, vr_start
    ro, alpha, beta = ro_start, alpha_start, beta_start

    # 9 obstacles and 1 target
    barriers, target_index = generate_barriers(10)

    pygame.init()
    # Initialise pygame display screen
    screen = pygame.display.set_mode(size)

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
        draw_scene(
            screen, location_history, barriers, target_index, x, y, theta,
            ball_predicted_positions, barriers_predicted_positions
        )
        # Update display
        pygame.display.flip()
        screen_picture = pygame.surfarray.pixels3d(screen)

        # For display of trail
        location_history.append((x, y))

        # Identify a ball and players positions
        ball_predicted_positions, barriers_predicted_positions = obstacle_detection.forward(
            screen_picture, [(red, 1), (lightblue, 9)]
        )
        ball_predicted_positions = cast_detector_coordinates(ball_predicted_positions)
        barriers_predicted_positions = cast_detector_coordinates(barriers_predicted_positions)

        # Planning
        dist_to_target = math.sqrt((x - target_x) ** 2 + (y - target_y) ** 2)
        if dist_to_target < (ROBOTRADIUS + 0.3):
            # print("Calling Obstacle Avoidance algorithm")
            # Calculate best target point and call moveToDot

            target_x, target_y = obstacle_avoidance(x, y, ball_predicted_positions, barriers_predicted_positions)
            vl, vr, ro, alpha, beta = move_to_dot(target_x, target_y, x, y, theta)
        else:
            # print("stillMovingToDot")
            vl, vr, ro, alpha, beta = move_to_dot_again(ro, alpha, beta, theta)

        # Actually now move robot based on chosen vl and vr
        (x, y, theta) = set_new_position(vl, vr, x, y, theta, dt)

        barriers = move_barriers(dt, barriers, target_index)

        # printBarriers()

        # Check collision
        dist_to_obstacle = calculate_closest_obstacle_distance(x, y, barriers, target_index)
        if dist_to_obstacle < 0.001:
            print("Crash!")
            print("Result:", time.time() - start_time, "sec")
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return

        # Check if robot has reached target
        dist_to_target = math.sqrt((x - barriers[target_index][0]) ** 2 + (y - barriers[target_index][1]) ** 2)
        if dist_to_target < (BARRIERRADIUS + ROBOTRADIUS):
            print("Result:", time.time() - start_time, "sec")
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return

        # Sleeping dt here runs simulation in real-time
        time.sleep(dt / 50)


if __name__ == '__main__':
    main()
