from typing import List, Tuple

import cv2
import numpy

from obstacle_detection.obstacle_utils import extract_closest_points


class ScaleBasedObstacleDetector:

    def __init__(self, algorithm: str):
        self.name = algorithm
        if algorithm == 'SURF':
            self.get_ball_detector = lambda: cv2.xfeatures2d.SURF_create()
        elif algorithm == 'U-SURF':
            self.get_ball_detector = lambda: cv2.xfeatures2d.SURF_create(upright=True)
        elif algorithm == 'SIFT':
            self.get_ball_detector = lambda: cv2.xfeatures2d.SIFT_create()
        else:
            raise ValueError(f"Unknown scale based object detection algorithm {algorithm}")

    def forward(self, image: numpy.ndarray, reference_color: List[Tuple[Tuple, int]]):
        assert len(image.shape) == 3, 'Please pass a colored 3-channel image'

        image_scaled = cv2.resize(image, None, fx=0.5, fy=0.5)
        image_grayscale = cv2.cvtColor(image_scaled, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.get_ball_detector().detectAndCompute(image_grayscale, None)

        distances = {color: [] for color, _ in reference_color}

        for i, keypoint in enumerate(keypoints):
            x, y = keypoint.pt
            x = round(x)
            y = round(y)
            radius = round(keypoint.size / 4)

            mask = numpy.zeros(image_scaled.shape[:2], numpy.uint8)
            cv2.circle(mask, (x, y), 3, radius, thickness=-1)
            relevant_pixels = numpy.where(mask)
            mean_color = image_scaled[relevant_pixels].mean(0)
            for color, _ in reference_color:
                distances[color].append((
                    numpy.linalg.norm(color - mean_color),
                    numpy.array((y, x))
                ))

        return extract_closest_points(distances, reference_color, 2)
