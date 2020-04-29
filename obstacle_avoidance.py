import math
from constants import *

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
    FORWARDWEIGHT = 12
    OBSTACLEWEIGHT = 6666
    TAU = dt * 25
    target = ball[0]

    vls_possible = (vl - MAXACCELERATION * dt, vl, vl + MAXACCELERATION * dt)
    vrs_possible = (vr - MAXACCELERATION * dt, vr, vr + MAXACCELERATION * dt)

    print("-------------------------DWA--------------------------")
    print("vl, vr == ", (vl, vr))
    print("x, y == ", (x, y))
    print("------------------------------------------------------")
    for vl_possible in vls_possible:
        for vr_possible in vrs_possible:
            print("vl_p, vr_p == ", (vl_possible, vr_possible))
            # We can only choose an action if it's within velocity limits
            if (vl_possible > MAXVELOCITY or vl_possible < -MAXVELOCITY or
                vr_possible > MAXVELOCITY or vr_possible < -MAXVELOCITY):
                continue
            # Predict next position
            x_predict, y_predict, theta_predict = set_new_position(vl_possible, vr_possible, x, y, theta, TAU)
            print("x_p, y_p == ", (x_predict, y_predict))
            # Calculate how much close we've moved to target location
            curr_distance_to_target = math.sqrt((x - target[0])**2 + (y - target[1])**2)
            pred_distance_to_target = math.sqrt((x_predict - target[0])**2 + (y_predict - target[1])**2)
            distance_forward = curr_distance_to_target - pred_distance_to_target
            distance_benefit = FORWARDWEIGHT * distance_forward
            
            # Calculate distance to closest obstacle from new position
            distance_to_obstacle = calculate_closest_obstacle_distance(x_predict, y_predict, barriers, None)
            # Negative benefit: once we are less than ROBOTRADIUS from collision, linearly increasing cost
            obstacle_benefit = 0.0
            if (distance_to_obstacle < ROBOTRADIUS):
                obstacle_benefit = OBSTACLEWEIGHT * (ROBOTRADIUS - distance_to_obstacle)

            benefit = distance_benefit - obstacle_benefit
            if (benefit > best_benefit):
                best_benefit = best_benefit
                best_x = x_predict
                best_y = y_predict

    print("best x, y == ", (best_x, best_y))

    return best_x, best_y