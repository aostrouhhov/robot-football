import math
import logging

logger = logging.getLogger('algo')

INF = 10 ** 9


class Sector:
    DEG_STEP = 15

    def __init__(self, id, deg):
        self.id = id
        self.start_deg = deg
        self.end_deg = deg + Sector.DEG_STEP

        lowest_line, highest_line = self._generate_lines_by_deg(deg)

        self.lowest_line = lowest_line
        self.highest_line = highest_line

    def contains(self, point):
        return point.is_between_lines(self.lowest_line, self.highest_line)

    @classmethod
    def generate_sectors(cls):
        return [Sector(i + 1, deg=(1 + cls.DEG_STEP * i) % 360) for i in range(360 // cls.DEG_STEP)]

    def _generate_lines_by_deg(self, start_deg):
        lowest = self._get_line_by_deg(start_deg)
        highest = self._get_line_by_deg(start_deg + self.DEG_STEP)
        return lowest, highest

    @staticmethod
    def _radians(deg):
        return deg * 3.14 / 180

    def _get_line_by_deg(self, deg):
        # -kx + y = 0
        a = -1 * math.tan(self._radians(deg))
        b = 1

        if deg > 180:
            a *= -1
            b *= -1

        # logger.warning(f'deg {deg}: ({a > 0}, {b > 0})')

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

    def is_between_lines(self, line1, line2):
        return self.is_higher(line1) and not self.is_higher(line2)

    def is_higher(self, line):
        return line.get_point_relative_position(self) > 0

    def __repr__(self):
        return f'({round(self.x, 2)}, {round(self.y, 2)})'


class Line:
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def get_point_relative_position(self, point):
        return self.a * point.x + self.b * point.y + self.c

    def __repr__(self):
        return f'{round(self.a, 2)}x+{round(self.b, 2)}y+{round(self.c, 2)}=0'


def dump_obstacle_avoidance(robot, ball, obstacles):
    hist = {}  # sector to prob

    sectors = Sector.generate_sectors()
    for sector in sectors:
        logger.warning(sector)

    max_dist = 0
    robot_point = Point(0, 0)

    obstacle_to_sector = {}
    obstacle_to_dist = {}
    for obstacle in obstacles:
        x, y = obstacle.get_pos()
        obstacle_point = Point(x, y, coord_center=Point(robot.x, robot.y))

        curr_dist = robot_point.get_dist_to_point(obstacle_point)
        max_dist = max(max_dist, curr_dist)
        obstacle_to_dist[obstacle] = curr_dist

        for sector in sectors:
            if sector.contains(obstacle_point):
                obstacle_to_sector[obstacle] = sector
                break
        else:
            logger.error(f'Unable to identify {obstacle_point} position')
            raise ValueError

    ball_point = Point(ball.x, ball.y, coord_center=Point(robot.x, robot.y))
    ball_sector = None
    for sector in sectors:
        min_dist = INF
        closest_obstacle = None
        for obstacle in [obstacle for obstacle, sec in obstacle_to_sector.items() if sec == sector]:
            if obstacle_to_dist[obstacle] < min_dist:
                min_dist = obstacle_to_dist[obstacle]
                closest_obstacle = obstacle

        val = 0 if not closest_obstacle else round(min_dist / max_dist, 3)
        hist[sector.id] = val

        # get dist
        if sector.contains(ball_point):
            ball_sector = sector

    print(1)

    #
    # for sector_deg in sector_degs:
    #     k = math.tan(sector_deg)

    # No obstacle avoidance by default, just move to the ball
    # target_x = current_x
    # target_y = current_y
    # target_x = WINDOW_CORNERS[2]
    # target_y = WINDOW_CORNERS[3]

    # return target_x, target_y
