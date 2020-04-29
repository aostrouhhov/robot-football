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
def dwa(x, y, vl, vr, theta, ball, barriers):
    # We want to find the best benefit where we have a positive component for closeness to target,
    # and a negative component for closeness to obstacles, for each of a choice of possible actions
    best_benefit = -100000
    best_x, best_y = x, y
    FORWARDWEIGHT = 7
    OBSTACLEWEIGHT = 7000
    TAU = dt * 5
    target = ball[0]

    delta = MAXACCELERATION * dt
    l_mult_possible = [-3, -1.5, 0, 1.5, 3]
    r_mult_possible = [-3, -1.5, 0, 1.5, 3]
    vls_possible = (vl - 2 * delta, vl - delta, vl, vl + delta, vl + 2 * delta)
    vrs_possible = (vr - 2 * delta, vr - delta, vr, vr + delta, vr + 2 * delta)
    curr_distance_to_target = math.sqrt((x - target[0])**2 + (y - target[1])**2)

    # print("-------------------------DWA--------------------------")
    # print("vl, vr ==", (vl, vr))
    # print("x, y, th ==", (x, y, theta))
    # print("ball ==", target)
    # print("barriers ==", barriers)
    # print("------------------------------------------------------")
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
            # print("vl_p, vr_p ==", (vl_possible, vr_possible))
            # print("x_p, y_p, th_p ==", (x_predict, y_predict, theta_predict))
            # Calculate how much close we've moved to target location
            pred_distance_to_target = math.sqrt((x_predict - target[0])**2 + (y_predict - target[1])**2)
            distance_forward = curr_distance_to_target - pred_distance_to_target
            # Calculate distance to closest obstacle from new position
            distance_to_obstacle = calculate_closest_obstacle_distance(x_predict, y_predict, barriers, None)
            # print("curr_dist ==", curr_distance_to_target)
            # print("pred_dist ==", pred_distance_to_target)
            # print("forw_dist ==", distance_forward)
            # print("obst_dist ==", distance_to_obstacle)
            distance_benefit = FORWARDWEIGHT * distance_forward

            # Negative benefit: once we are less than ROBOTRADIUS from collision, linearly increasing cost
            obstacle_benefit = 0.0
            if (distance_to_obstacle < ROBOTWIDTH * 1.5):
                obstacle_benefit = OBSTACLEWEIGHT * (ROBOTWIDTH - distance_to_obstacle)

            benefit = distance_benefit - obstacle_benefit
            # print("dist_benefit ==", distance_benefit)
            # print("obst_benefit ==", obstacle_benefit)
            # print("benefit      ==", benefit)
            if (round(benefit, 2) > best_benefit):
                best_benefit = benefit
                best_x = x_predict
                best_y = y_predict
                best_th = theta_predict

    # print("best x, y, th ==", (best_x, best_y, best_th))
    # print("------------------------------------------------------")
    # time.sleep(1)
    return best_x, best_y, best_th