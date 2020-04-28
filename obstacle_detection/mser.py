from typing import List, Tuple

import cv2
import numpy

from obstacle_detection.utils import extract_closest_points


class MSERObstacleDetector:

    def __init__(
            self, hull_distance_threshold: int = 10
    ):
        self.hull_distance_threshold = hull_distance_threshold
        self.mser = cv2.MSER_create()

    def forward(self, image: numpy.ndarray, reference_color: List[Tuple[Tuple, int]]):
        assert len(image.shape) == 3, 'Please pass a colored 3-channel image'

        image_scaled = cv2.resize(image, None, fx=0.5, fy=0.5)
        image_grayscale = cv2.cvtColor(image_scaled, cv2.COLOR_BGR2GRAY)

        regions = self.mser.detectRegions(image_grayscale)
        hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in regions[0]]

        distances = {color: [] for color, _ in reference_color}

        last_hull_coords = numpy.array([-1000, -1000])
        for i, hull in enumerate(hulls):
            mask = cv2.fillConvexPoly(numpy.zeros_like(image_grayscale), hull, (255, 255, 255))
            relevant_pixels = numpy.where(mask)
            hull_coords = numpy.mean(relevant_pixels, axis=-1)
            if numpy.linalg.norm(hull_coords - last_hull_coords) > self.hull_distance_threshold:
                last_hull_coords = hull_coords
            else:
                continue

            mean_color = numpy.mean(image_scaled[relevant_pixels], axis=0)
            for color, _ in reference_color:
                distances[color].append((
                    numpy.linalg.norm(color - mean_color),
                    hull_coords
                ))

        return extract_closest_points(distances, reference_color, 2)
