import main
import utils
import constants
from models import Ball, Robot, MovingObstacle


def test_no_obs():
    robot = Robot(1, -2, utils.to_radians(-60))
    ball = Ball(0, 0, 0, 0)
    result = main.run_simulation(robot, ball, [], simulation_delay=1000,
                                 enable_detection=False, drawable_obs_avoidance=True)
    assert result


def test_no_obs2():
    robot = Robot(constants.x_start, constants.y_start, constants.theta_start)
    ball = Ball(constants.WINDOW_CORNERS[2]-1, constants.WINDOW_CORNERS[3]-1, 0, 0)
    result = main.run_simulation(robot, ball, [], simulation_delay=1000, enable_detection=False)
    assert result


def test_static_obs():
    robot = Robot(constants.x_start, constants.y_start, constants.theta_start)
    ball = Ball(constants.WINDOW_CORNERS[2]-1, constants.WINDOW_CORNERS[3]-1, 0, 0)

    obstacles = [
        MovingObstacle(ball.x - 0.5, ball.y -0.5, 0, 0)
        # MovingObstacle.create_randomized(),
    ]

    result = main.run_simulation(robot, ball, obstacles, simulation_delay=1000,
                                 enable_detection=True, drawable_obs_avoidance=True)
    assert result

def test_static_hist():
    robot = Robot(constants.x_start, constants.y_start, constants.theta_start)
    ball = Ball(constants.WINDOW_CORNERS[2]-1, constants.WINDOW_CORNERS[3]-1, 0, 0)

    obstacles = [
        MovingObstacle(constants.x_start + 1.5, constants.y_start + 1.5, 0, 0)
        # MovingObstacle.create_randomized(),
    ]

    result = main.run_simulation(robot, ball, obstacles, simulation_delay=1000,
                                 enable_detection=True, drawable_obs_avoidance=True)
    assert result

if __name__ == '__main__':
    # test_no_obs()
    # test_no_obs2()
    # test_static_obs()
    test_static_hist()
