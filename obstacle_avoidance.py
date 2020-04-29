import math
import logging

logger = logging.getLogger('algo')

INF = 10 ** 9


class Sector:
    DEG_STEP = 30
    COUNT = 360 // DEG_STEP

    FIRST_SECTOR_ID = 1  # todo: is not used for setting up sectors
    LAST_SECTOR_ID = None

    def __init__(self, id, deg):
        self.id = id
        self.start_deg = deg
        self.end_deg = deg + Sector.DEG_STEP

        self.lowest_line = self._get_line_by_deg(self.start_deg)
        self.highest_line = self._get_line_by_deg(self.end_deg)

        Sector.LAST_SECTOR_ID = self.id

    def contains(self, point):
        return point.is_left_of_line(self.lowest_line) and not point.is_left_of_line(self.highest_line)

    def get_dist_to_sector(self, sector):
        if abs(self.id - Sector.FIRST_SECTOR_ID) < abs(self.id - Sector.LAST_SECTOR_ID):
            near_border_id = Sector.FIRST_SECTOR_ID
            far_border_id = Sector.LAST_SECTOR_ID
        else:
            near_border_id = Sector.LAST_SECTOR_ID
            far_border_id = Sector.LAST_SECTOR_ID

        transitive = abs(self.id - near_border_id) + 1 + abs(far_border_id - sector.id)
        direct = abs(self.id - sector.id)

        return min(direct, transitive)

    @classmethod
    def generate_sectors(cls):
        return [Sector(i + 1, deg=(1 + cls.DEG_STEP * i) % 360) for i in range(cls.COUNT)]

    @staticmethod
    def _radians(deg):
        return deg * 3.14 / 180

    def _get_line_by_deg(self, deg):  # kx - y = 0, (-b, a) > 0
        a = math.tan(self._radians(deg))
        b = -1

        if 90 < deg < 270:
            a *= -1
            b *= -1

        logger.info(f'deg {deg}: ({a > 0}, {b > 0})')
        return Line(a, b, 0)

    def __repr__(self):
        return f'#{self.id} {self.lowest_line} {self.highest_line}'


class Point:
    def __init__(self, x, y, coord_center=None):
        self.x = x
        self.y = y

        if coord_center:
            self.x -= coord_center.x
            self.y -= coord_center.y

    def get_dist_to_point(self, p):
        return math.sqrt((self.x - p.x) ** 2 + (self.y - p.y) ** 2)

    def is_left_of_line(self, line):
        return line.get_point_relative_position(self) < 0

    def __repr__(self):
        return f'({round(self.x, 2)}, {round(self.y, 2)})'


class Line:
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def direction_vector(self):
        return self.b * (-1), self.a

    def get_point_relative_position(self, point):
        return self.a * point.x + self.b * point.y + self.c

    def __repr__(self):
        return f'{round(self.a, 2)}x+{round(self.b, 2)}y+{round(self.c, 2)}=0'


def dump_obstacle_avoidance(robot_x, robot_y, ball_positions, obstacles_positions):
    ball_x, ball_y = ball_positions[0]

    hist = {}  # sector to prob

    sectors = Sector.generate_sectors()
    for sector in sectors:
        logger.info(sector)

    max_dist = 0
    robot_point = Point(0, 0)

    obstacle_to_sector = {}
    obstacle_to_dist = {}
    for obstacle_num, obstacle_pos in enumerate(obstacles_positions):
        x, y = obstacle_pos
        obstacle_point = Point(x, y, coord_center=Point(robot_x, robot_y))

        curr_dist = robot_point.get_dist_to_point(obstacle_point)
        max_dist = max(max_dist, curr_dist)
        obstacle_to_dist[obstacle_num] = curr_dist

        for sector in sectors:
            if sector.contains(obstacle_point):
                obstacle_to_sector[obstacle_num] = sector
                break
        else:
            logger.error(f'Unable to identify obstacle {obstacle_point} position')
            return 0, 0  # no success

    ball_point = Point(ball_x, ball_y, coord_center=Point(robot_x, robot_y))
    ball_sector = None
    free_sectors = []
    for sector in sectors:
        min_dist = INF
        closest_obstacle = None
        for obstacle in [obstacle for obstacle, sec in obstacle_to_sector.items() if sec == sector]:
            if obstacle_to_dist[obstacle] < min_dist:
                min_dist = obstacle_to_dist[obstacle]
                closest_obstacle = obstacle

        if not closest_obstacle:
            free_sectors.append(sector)
            hist_val = 0
        else:
            hist_val = round(min_dist / max_dist, 3)

        hist[sector.id] = hist_val

        # get dist
        if sector.contains(ball_point):
            ball_sector = sector

    if not ball_sector:
        logger.error(f'Unable to identify ball {ball_point} position')
        return 0, 0

    min_diff = INF
    target_sector = None
    for sector in free_sectors:
        curr_diff = abs(sector.id - ball_sector.id)
        if curr_diff < min_diff:
            min_diff = curr_diff
            target_sector = sector

    if target_sector == ball_sector:
        return ball_x, ball_y

    lowest_vec = target_sector.lowest_line.direction_vector()
    highest_vec = target_sector.highest_line.direction_vector()

    target_vec = (highest_vec[0] + lowest_vec[0]) / 2, (highest_vec[1] + lowest_vec[1] / 2)
    target_x, target_y = robot_x + target_vec[0] * 100, robot_y + target_vec[1] * 100  # moving far ahead

    logger.warning(f'Should go to {target_sector} according to {target_vec}')
    return target_x, target_y
