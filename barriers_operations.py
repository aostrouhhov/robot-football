# For debugging, prints locations of all barriers (other players and ball)
import random
from typing import List, Tuple

import cv2
import numpy

from constants import PLAYFIELDCORNERS, red, lightblue, k, u0, v0, BARRIERRADIUS, BARRIERVELOCITYRANGE


def print_barriers(barriers):
    for i, barrier in enumerate(barriers):
        print(i, barrier[0], barrier[1], barrier[2], barrier[3])


# Moves other players (not ball)
def move_barriers(dt, barriers, target_index):
    for i, barrier in enumerate(barriers):
        barriers[i][0] += barriers[i][2] * dt
        if barriers[i][0] < PLAYFIELDCORNERS[0]:
            barriers[i][2] = -barriers[i][2]
        if barriers[i][0] > PLAYFIELDCORNERS[2]:
            barriers[i][2] = -barriers[i][2]
        barriers[i][1] += barriers[i][3] * dt
        if barriers[i][1] < PLAYFIELDCORNERS[1]:
            barriers[i][3] = -barriers[i][3]
        if barriers[i][1] > PLAYFIELDCORNERS[3]:
            barriers[i][3] = -barriers[i][3]
    return barriers


# Draw the barriers (other players and ball) on the screen
def draw_barriers(screen, barriers, target_index):
    for i, barrier in enumerate(barriers):
        b_col = red if i == target_index else lightblue
        cv2.circle(
            screen, (int(u0 + k * barrier[0]), int(v0 - k * barrier[1])), int(k * BARRIERRADIUS), b_col, -1
        )


def generate_barriers(num):
    # Barrier (other players) locations
    # Barrier contents are (bx, by, visibility mask)
    barriers = []

    # Generate num-1 random barriers
    for i in range(num - 1):
        (bx, by, vx, vy) = (
            random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]),
            random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]),
            random.gauss(0.0, BARRIERVELOCITYRANGE),
            random.gauss(0.0, BARRIERVELOCITYRANGE),
        )
        barrier = [bx, by, vx, vy]
        barriers.append(barrier)

    # Ball will be just another barrier which doesn't move
    (bx, by, vx, vy) = (
        PLAYFIELDCORNERS[2] - 1,
        PLAYFIELDCORNERS[3] - 1,
        random.gauss(0.0, BARRIERVELOCITYRANGE),
        random.gauss(0.0, BARRIERVELOCITYRANGE),
    )
    barrier = [bx, by, vx, vy]
    barriers.append(barrier)
    assert len(barriers) == num
    return barriers, num - 1


def draw_ball_edges(surface: numpy.ndarray, ball_coords: List[Tuple[float, float]], color: Tuple[int, int, int]):
    for ball_coord in ball_coords:
        x = int(u0 + k * ball_coord[0])
        y = int(v0 - k * ball_coord[1])
        cv2.circle(surface, (x, y), int(k * BARRIERRADIUS), color, 3)
