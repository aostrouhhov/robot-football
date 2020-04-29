import math
from constants import *

def dump_obstacle_avoidance(current_x, current_y, target_ball, barriers):
    # No obstacle avoidance by default, just move to the ball
    target_x = target_ball[0][0]
    target_y = target_ball[0][1]

    return target_x, target_y
