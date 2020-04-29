# For debugging, prints locations of all barriers (other players and ball)
import random
from typing import List, Tuple

import pygame
from pygame import gfxdraw

from constants import WINDOW_CORNERS, k, u0, v0, BARRIERRADIUS, BARRIER_VELOCITY_RANGE


def print_barriers(barriers):
    for i, barrier in enumerate(barriers):
        print(i, barrier[0], barrier[1], barrier[2], barrier[3])


def draw_ball_edges(surface: pygame.Surface, ball_coords: List[Tuple[float, float]], color: Tuple[int, int, int]):
    for ball_coord in ball_coords:
        x = int(u0 + k * ball_coord[0])
        y = int(v0 - k * ball_coord[1])
        pygame.gfxdraw.aacircle(surface, x, y, int(k * BARRIERRADIUS), color)
