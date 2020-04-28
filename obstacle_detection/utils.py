from typing import Dict, List, Tuple

import numpy


def extract_closest_points(
        distances: Dict, reference_colors: List[Tuple[Tuple, int]], scale: float
) -> List[numpy.ndarray]:
    result = []
    for color, cnt in reference_colors:
        result.append(numpy.array(list(map(
            lambda t: [round(t[1][0] * scale), round(t[1][1] * scale)],
            sorted(distances[color], key=lambda t: t[0])[:cnt]
        ))))
    return result
