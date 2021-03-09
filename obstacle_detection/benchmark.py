import time

import numpy
from tqdm import tqdm

import constants
import utils
from models import Ball, MovingObstacle
from obstacle_detection.mser import MSERObstacleDetector
from obstacle_detection.scale_based import ScaleBasedObstacleDetector

N_SAMPLES = 5000
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


def l2_norm(true_point, predicted_point):
    return numpy.linalg.norm([a - b for a, b in zip(true_point, predicted_point)])


def l2_to_metric(l2):
    return 1. / (l2 + 1)


def l2_obstacles(true_obstacles, predicted_obstacles):
    used = [False for _ in true_obstacles]
    l2 = []
    for obs in predicted_obstacles:
        cur_l2 = [(l2_norm(obs, _o), num) for num, _o in enumerate(true_obstacles) if not used[num]]
        min_point = min(cur_l2)
        # print(f"{obs} -- {true_obstacles[min_point[1]]}")
        l2.append(min_point[0])
        used[min_point[1]] = True
    return sum(l2)


def main():
    print(f"Generating {N_SAMPLES} samples...")
    samples = [
        generate_sample(constants.WINDOW_HEIGHT, constants.WINDOW_WIDTH, N_OBSTACLES) for _ in tqdm(range(N_SAMPLES))
    ]
    mser_detector = MSERObstacleDetector()
    surf_detector = ScaleBasedObstacleDetector('U-SURF')
    sift_detector = ScaleBasedObstacleDetector('SIFT')

    all_l2_obstacles = {}
    all_l2_balls = {}
    time_per_sample = {}

    for detector in [mser_detector, surf_detector, sift_detector]:
        print(f"Benchmarking {detector.name} algorithm...")
        all_l2_obstacles[detector.name] = []
        all_l2_balls[detector.name] = []
        time_per_sample[detector.name] = []

        for screen, obstacles, ball in tqdm(samples):
            start_time = time.time()
            ball_predicted_positions, barriers_predicted_positions = detector.forward(
                screen, [(constants.Color.RED, 1), (constants.Color.LIGHTBLUE, N_OBSTACLES)]
            )
            finish_time = time.time()
            ball_predicted_positions = utils.cast_detector_coordinates(ball_predicted_positions)[0]
            barriers_predicted_positions = utils.cast_detector_coordinates(barriers_predicted_positions)

            all_l2_obstacles[detector.name].append(l2_obstacles(obstacles, barriers_predicted_positions))
            all_l2_balls[detector.name].append(l2_norm(ball, ball_predicted_positions))
            time_per_sample[detector.name].append(finish_time - start_time)

    with open('obstacle_detection_benchmark.txt', 'w') as result_file:
        template = '{:^20}|{:^10}|{:^10}\n'
        for d_name in [mser_detector.name, surf_detector.name, sift_detector.name]:
            mean_time = round(numpy.mean(time_per_sample[d_name]), 4)
            std_time = round(numpy.std(time_per_sample[d_name]), 4)
            mean_l2_obs = numpy.mean(all_l2_obstacles[d_name])
            mean_l2_ball = numpy.mean(all_l2_balls[d_name])

            result_file.write("{:=^45}\n".format(d_name))
            result_file.write(f"Mean time per sample: {mean_time} \u00B1 {std_time}ms\n")
            result_file.write(template.format('', 'l2', 'inverse l2'))
            result_file.write(template.format(
                'detecting obstacles', round(mean_l2_obs, 2), round(l2_to_metric(mean_l2_obs), 2)
            ))
            result_file.write(template.format(
                'detecting ball', round(mean_l2_ball, 2), round(l2_to_metric(mean_l2_ball), 2)
            ))
            result_file.write("\n")


if __name__ == '__main__':
    main()
