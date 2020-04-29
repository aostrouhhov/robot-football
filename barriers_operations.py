# For debugging, prints locations of all barriers (other players and ball)
import random
from typing import List, Tuple

import cv2
from constants import WINDOW_CORNERS, k, u0, v0, BARRIERRADIUS, BARRIER_VELOCITY_RANGE


def print_barriers(barriers):
    for i, barrier in enumerate(barriers):
        print(i, barrier[0], barrier[1], barrier[2], barrier[3])


def draw_ball_edges(surface: pygame.Surface, ball_coords: List[Tuple[float, float]], color: Tuple[int, int, int]):
    for ball_coord in ball_coords:
        x = int(u0 + k * ball_coord[0])
        y = int(v0 - k * ball_coord[1])
        cv2.circle(surface, (x, y), int(k * BARRIERRADIUS), color, 3, lineType=cv2.LINE_AA)
