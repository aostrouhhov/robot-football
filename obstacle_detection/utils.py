from typing import Dict, List, Tuple

import numpy

from constants import BARRIERRADIUS


def extract_closest_points(
    distances: Dict, reference_colors: List[Tuple[Tuple, int]], scale: float
) -> List[numpy.ndarray]:
    result = []
    for color, cnt in reference_colors:
        true_points = remove_too_close_points(sorted(distances[color], key=lambda t: t[0]))
        result.append(
            numpy.array(
                [
                    (round(y * scale), round(x * scale))
                    for i, (dist, (x, y)) in enumerate(true_points[:cnt])
                    if i == 0 or dist < 200
                ]
            )
        )
    return result


def remove_too_close_points(points: List[Tuple[float, Tuple]]) -> List[Tuple[float, Tuple]]:
    result = []
    for _d, point in points:
        if all([numpy.linalg.norm(i[1] - point) > 3 for i in result]):
            result.append((_d, point))
    return result
