# Made according to this (many thanks):
# https://www.youtube.com/watch?v=Mdg9ElewwA0&feature=emb_logo

import math
import time

import pygame

from barriers_operations import draw_barriers, generate_barriers, draw_ball_edges, move_barriers
from constants import *
from obstacle_avoidance import dump_obstacle_avoidance
from utils import move_to_dot, move_to_dot_again, set_new_position, calculate_closest_obstacle_distance

obstacle_avoidance = dump_obstacle_avoidance


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
    # Array for path choices use for graphics
    paths_to_draw = []

    # We will calculate time
    start_time = time.time()

    target_x = x
    target_y = y

    # Main loop
    while True:
        event_list = pygame.event.get()

        # For display of trail
        location_history.append((x, y))

        ball_predicted_positions = [(0.0, 0.0)]
        barriers_predicted_positions = [(0.0, 0.0)] * 9

        # Planning
        dist_to_target = math.sqrt((x - target_x) ** 2 + (y - target_y) ** 2)
        if dist_to_target < (ROBOTRADIUS + 0.3):
            # print("Calling Obstacle Avoidance algorithm")
            # Calculate best target point and call moveToDot

            # Identify a ball and players positions
            # ball_predicted_positions, barriers_predicted_positions =\
            #   get_barriers_positions(screen_picture, ((red, 1), (lightblue, 9)))

            target_x, target_y = obstacle_avoidance(x, y, ball_predicted_positions, barriers_predicted_positions)
            vl, vr, ro, alpha, beta = move_to_dot(target_x, target_y, x, y, theta)

            # if vl > MAXVELOCITY or vr > MAXVELOCITY:
            #     if vl > vr:
            #         diff = vr / vl
            #         vl = 0.3
            #         vr = vl * diff
            #     elif vr > vl:
            #         diff = vl / vr
            #         vr = 0.3
            #         vl = vr * diff
            #     else:
            #         vl = 0.3
            #         vr = 0.3
        else:
            # print("stillMovingToDot")
            vl, vr, ro, alpha, beta = move_to_dot_again(ro, alpha, beta, theta)

            # if vl > MAXVELOCITY or vr > MAXVELOCITY:
            #     if vl > vr:
            #         diff = vr / vl
            #         vl = 0.3
            #         vr = vl * diff
            #     elif vr > vl:
            #         diff = vl / vr
            #         vr = 0.3
            #         vl = vr * diff
            #     else:
            #         vl = 0.3
            #         vr = 0.3

        screen.fill(black)
        for loc in location_history:
            pygame.draw.circle(screen, grey, (int(u0 + k * loc[0]), int(v0 - k * loc[1])), 3, 0)
        draw_barriers(screen, barriers, target_index)

        # Draw robot
        u = u0 + k * x
        v = v0 - k * y
        pygame.draw.circle(screen, white, (int(u), int(v)), int(k * ROBOTRADIUS), 3)
        # Draw wheels as little blobs, so you can see robot orientation
        # left wheel centre
        wlx = x - (ROBOTWIDTH / 2.0) * math.sin(theta)
        wly = y + (ROBOTWIDTH / 2.0) * math.cos(theta)
        ulx = u0 + k * wlx
        vlx = v0 - k * wly
        pygame.draw.circle(screen, blue, (int(ulx), int(vlx)), int(k * WHEELBLOB))
        # right wheel centre
        wrx = x + (ROBOTWIDTH / 2.0) * math.sin(theta)
        wry = y - (ROBOTWIDTH / 2.0) * math.cos(theta)
        urx = u0 + k * wrx
        vrx = v0 - k * wry
        pygame.draw.circle(screen, blue, (int(urx), int(vrx)), int(k * WHEELBLOB))

        # Save picture of screen for balls detection
        screen_picture = pygame.surfarray.pixels3d(screen)
        print(screen_picture.shape)
        # After this draw circles
        draw_ball_edges(screen, barriers[-1:], ball_edge_color)
        draw_ball_edges(screen, barriers[:-1], barrier_edge_color)
        # draw_ball_edges(screen, ball_predicted_positions, ball_edge_color)
        # draw_ball_edges(screen, barriers_predicted_positions, barrier_edge_color)

        # Update display
        pygame.display.flip()

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
