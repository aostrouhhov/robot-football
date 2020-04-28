def dump_obstacle_avoidance(current_x, current_y, target_ball, barriers):
    # No obstacle avoidance by default, just move to the ball
    target_x = current_x + 0.5
    target_y = current_y + 0.35
    # target_x = PLAYFIELDCORNERS[2]
    # target_y = PLAYFIELDCORNERS[3]

    return target_x, target_y
