import time

import numpy
from tqdm import tqdm

import constants
import utils
from models import Ball, MovingObstacle
from obstacle_detection.mser import MSERObstacleDetector
from obstacle_detection.scale_based import ScaleBasedObstacleDetector

N_SAMPLES = 1000
N_OBSTACLES = 10


def generate_sample(height, width, n_obstacles):
    obstacles = [MovingObstacle.create_randomized() for _ in range(n_obstacles)]
    ball = MovingObstacle.create_randomized()
    ball.COLOR = Ball.COLOR
    screen = numpy.full((height, width, 3), constants.Color.BLACK, dtype=numpy.uint8)
    for obs in obstacles:
        obs.draw(screen)
    ball.draw(screen)
    return screen, [(ob.x, ob.y) for ob in obstacles], (ball.x, ball.y)


def square_error(true_point, predicted_point):
    return numpy.linalg.norm([a - b for a, b in zip(true_point, predicted_point)])


def mse_to_metric(mse):
    return 1. / (mse + 1)


def mse_obstacles(true_obstacles, predicted_obstacles):
    used = [False for _ in true_obstacles]
    mse = []
    for obs in predicted_obstacles:
        cur_mse = [(square_error(obs, _o), num) for num, _o in enumerate(true_obstacles) if not used[num]]
        min_point = min(cur_mse)
        # print(f"{obs} -- {true_obstacles[min_point[1]]}")
        mse.append(min_point[0])
        used[min_point[1]] = True
    return sum(mse)


def main():
    samples = [generate_sample(constants.WINDOW_HEIGHT, constants.WINDOW_WIDTH, N_OBSTACLES) for _ in range(N_SAMPLES)]
    mser_detector = MSERObstacleDetector()
    surf_detector = ScaleBasedObstacleDetector('U-SURF')
    sift_detector = ScaleBasedObstacleDetector('SIFT')

    total_mse_obstacles = {}
    total_mse_balls = {}
    total_time_per_sample = {}

    for detector in [mser_detector, surf_detector, sift_detector]:
        print(f"Benchmarking {detector.name} algorithm...")
        total_mse_obstacles[detector.name] = []
        total_mse_balls[detector.name] = []
        total_time_per_sample[detector.name] = []

        for screen, obstacles, ball in tqdm(samples):
            start_time = time.time()
            ball_predicted_positions, barriers_predicted_positions = detector.forward(
                screen, [(constants.Color.RED, 1), (constants.Color.LIGHTBLUE, N_OBSTACLES)]
            )
            finish_time = time.time()
            ball_predicted_positions = utils.cast_detector_coordinates(ball_predicted_positions)[0]
            barriers_predicted_positions = utils.cast_detector_coordinates(barriers_predicted_positions)

            total_mse_obstacles[detector.name].append(mse_obstacles(obstacles, barriers_predicted_positions))
            total_mse_balls[detector.name].append(square_error(ball, ball_predicted_positions))
            total_time_per_sample[detector.name].append(finish_time - start_time)

    with open('obstacle_detection_benchmark.txt', 'w') as result_file:
        template = '{:^20}|{:^10}|{:^10}\n'
        for d_name in [mser_detector.name, surf_detector.name, sift_detector.name]:
            mean_time = round(numpy.mean(total_time_per_sample[d_name]), 4)
            std_time = round(numpy.std(total_time_per_sample[d_name]), 4)
            mean_mse_obs = numpy.mean(total_mse_obstacles[d_name])
            mean_mse_ball = numpy.mean(total_mse_balls[d_name])

            result_file.write("{:=^45}\n".format(d_name))
            result_file.write(f"Mean time per sample: {mean_time} \u00B1 {std_time}ms\n")
            result_file.write(template.format('', 'mse', 'inverse mse'))
            result_file.write(template.format(
                'detecting obstacles', round(mean_mse_obs, 2), round(mse_to_metric(mean_mse_obs), 2)
            ))
            result_file.write(template.format(
                'detecting ball', round(mean_mse_ball, 2), round(mse_to_metric(mean_mse_ball), 2)
            ))
            result_file.write("\n")


if __name__ == '__main__':
    main()
