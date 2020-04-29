# Made according to this (many thanks):
# https://www.youtube.com/watch?v=Mdg9ElewwA0&feature=emb_logo

import pygame
import time

from typing import List

import constants
from constants import Color
from models import Robot, Player, Ball
from obstacle_avoidance import dump_obstacle_avoidance

obstacle_avoidance = dump_obstacle_avoidance


def _generate_players(screen, cnt=10):
    barriers = []
    for i in range(cnt):
        barrier = Player.create_randomized(screen)
        barriers.append(barrier)
    return barriers


def _draw_scene(screen, robot: Robot, ball: Ball, players: List[Player]):
    screen.fill(Color.BLACK)
    ball.draw()
    robot.draw()
    for player in players:
        player.draw()

    pygame.display.flip()  # Update display


def main():
    ro, alpha, beta = constants.ro_start, constants.alpha_start, constants.beta_start

    pygame.init()
    start_time = time.time()
    screen = pygame.display.set_mode([constants.WINDOW_WIDTH, constants.WINDOW_HEIGHT])

    ball = Ball.create_randomized(screen)
    players = _generate_players(screen, cnt=10)
    robot = Robot(screen, constants.x_start, constants.y_start, constants.theta_start)

    target_x = robot.x
    target_y = robot.y

    dt = 0.1
    is_finished = False

    while True:
        _draw_scene(screen, robot, ball, players)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        if is_finished:
            continue

        # ball_predicted_positions = ball.get_pos()
        # players_predicted_positions = [player.get_pos() for player in players]

        obstacle_avoidance(robot, ball, players)

        # Planning
        # dist_to_target = math.sqrt((robot.x - target_x) ** 2 + (robot.y - target_y) ** 2)
        # if dist_to_target < Robot.RADIUS + 0.3:
        #     # print("Calling Obstacle Avoidance algorithm")
        #     # Calculate best target point and call moveToDot
        #
        #     target_x, target_y = obstacle_avoidance(robot.x, robot.y, ball_predicted_positions, players_predicted_positions)
        #     vl, vr, ro, alpha, beta = move_to_dot(target_x, target_y, robot.x, robot.y, robot.angle)
        # else:
        #     # print("stillMovingToDot")
        #     vl, vr, ro, alpha, beta = move_to_dot_again(ro, alpha, beta, robot.angle, dt)

        # Save picture of screen for balls detection
        # screen_picture = pygame.surfarray.pixels3d(screen)
        # print(screen_picture.shape)

        # After this draw circles
        # draw_ball_edges(screen, players[-1:], Color.YELLOW)
        # draw_ball_edges(screen, players[:-1], Color.GREEN)
        # draw_ball_edges(screen, ball_predicted_positions, ball_edge_color)
        # draw_ball_edges(screen, barriers_predicted_positions, barrier_edge_color)

        ball.move(dt)
        robot.move(dt)

        for player in players:
            player.move(dt)

        dist_to_obstacle = robot.get_closest_dist_to_obstacle(players)
        if dist_to_obstacle < 0.001:
            print('Crash!')
            print(f'Result: {time.time() - start_time} sec')
            is_finished = True
            continue

        dist_to_target = robot.get_dist_to_target(ball)  # Check if robot has reached the target
        if dist_to_target < Player.RADIUS + Robot.RADIUS:
            print(f'Result: {time.time() - start_time} sec')
            is_finished = True
            continue

        time.sleep(dt / 50)  # TODO: use Clock.tick(60) instead


if __name__ == '__main__':
    main()
