import math
import time
from constants import *
from utils import set_new_position, calculate_closest_obstacle_distance

def dump_obstacle_avoidance(current_x, current_y, target_ball, barriers):
    # No obstacle avoidance by default, just move to the ball
    target_x = target_ball[0][0]
    target_y = target_ball[0][1]

    return target_x, target_y

# Dynamic window approach
def dwa(vl, vr, x, y, theta, ball, barriers):
    # We want to find the best benefit where we have a positive component for closeness to target,
    # and a negative component for closeness to obstacles, for each of a choice of possible actions
    best_benefit = -100000
    best_vl, best_vr = vl, vr
    FORWARDWEIGHT = 7
    OBSTACLEWEIGHT = 10000
    TAU = dt * 10
    target = ball[0]

    delta = MAXACCELERATION * dt
    l_mult_possible = [-5, -3, -1, 0, 1, 3, 5]
    r_mult_possible = [-5, -3, -1, 0, 1, 3, 5]
    curr_distance_to_target = math.sqrt((x - target[0])**2 + (y - target[1])**2)

    # print("-------------- DWA --------------")
    # print(vl, vr)
    # print(x, y, theta)
    for l_mult in l_mult_possible:
        for r_mult in r_mult_possible:
            vl_possible = vl - l_mult * delta
            vr_possible = vr - r_mult * delta
            # We can only choose an action if it's within velocity limits
            if (vl_possible > MAXVELOCITY or vl_possible < -MAXVELOCITY or
                vr_possible > MAXVELOCITY or vr_possible < -MAXVELOCITY):
                continue
            # Predict next position
            x_predict, y_predict, theta_predict = set_new_position(vl_possible, vr_possible, x, y, theta, TAU)
            # if (x_predict < PLAYFIELDCORNERS[0] or x_predict > PLAYFIELDCORNERS[2] or
            #     y_predict < PLAYFIELDCORNERS[1] or y_predict > PLAYFIELDCORNERS[3]):
            #     continue
            # print(vl_possible, vr_possible)
            # print(x_predict, y_predict, theta_predict)
            # Calculate how much close we've moved to target location
            pred_distance_to_target = math.sqrt((x_predict - target[0])**2 + (y_predict - target[1])**2)
            distance_forward = curr_distance_to_target - pred_distance_to_target
            # Calculate distance to closest obstacle from new position
            distance_to_obstacle = calculate_closest_obstacle_distance(x_predict, y_predict, barriers, None)
            distance_benefit = FORWARDWEIGHT * distance_forward
            # Negative benefit: once we are less than ROBOTRADIUS from collision, linearly increasing cost
            obstacle_benefit = 0.0
            if (distance_to_obstacle < ROBOTWIDTH * 1.5):
                obstacle_benefit = OBSTACLEWEIGHT * (1.5 * ROBOTWIDTH - distance_to_obstacle)

            benefit = distance_benefit - obstacle_benefit
            if (round(benefit, 3) > best_benefit):
                best_benefit = benefit
                best_vl, best_vr = vl_possible, vr_possible

    return best_vl, best_vr