import constants
import math
import logging
import cv2
import numpy

import utils
from constants import Color
from models import Drawable, MovingObstacle

logger = logging.getLogger('algo')

INF = 10 ** 9


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

    def rotate(self, angle):
        x_ = self.x * math.cos(Sector._radians(angle)) + self.y * math.sin(Sector._radians(angle))
        y_ = -self.x * math.sin(Sector._radians(angle)) + self.y * math.cos(Sector._radians(angle))
        return Point(x_, y_)


class Line:
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def get_direction_vector(self):
        return self.b * (-1), self.a

    def get_point_relative_position(self, point):
        return self.a * point.x + self.b * point.y + self.c

    def draw(self, screen, center, color):
        lvec = self.get_direction_vector()
        lvec = utils.normalize_np_vector(lvec, 0.1)

        robot_pos = numpy.array([center.x, center.y])
        points_cnt = DRAWING_MAX_LINE_POINTS

        for n in range(points_cnt):
            if n == 0:
                continue

            p = numpy.add(robot_pos, numpy.array([lvec[0] * n, lvec[1] * n]))
            x, y = Drawable.get_coords_on_screen(p)

            logger.info(f'Drawing line {self}')
            cv2.circle(screen, (x, y), 1, color, thickness=-1)

    def __repr__(self):
        return f'{round(self.a, 2)}x+{round(self.b, 2)}y+{round(self.c, 2)}=0'


class Square:
    def __init__(self, x, y, width, coord_center=None):
        self.center = Point(x, y, coord_center=coord_center)

        self.width = width
        self.right_top = Point(x + width / 2, y + width / 2, coord_center=coord_center)
        self.left_top = Point(x - width / 2, y + width / 2, coord_center=coord_center)
        self.left_bot = Point(x - width / 2, y - width / 2, coord_center=coord_center)
        self.right_bot = Point(x + width / 2, y - width / 2, coord_center=coord_center)

        self.points = [self.center, self.right_top, self.left_top, self.left_bot, self.right_bot]

    def get_dist_to_point(self, point):
        min_dist = INF
        for p in self.points:
            min_dist = min(min_dist, p.get_dist_to_point(point))
        return min_dist


class Sector:
    DEG_STEP = 8
    COUNT = 360 // DEG_STEP

    FIRST_SECTOR_ID = 1  # todo: is not used for setting up sectors
    LAST_SECTOR_ID = None

    def __init__(self, id, deg):
        self.id = id
        self.start_deg = deg
        self.end_deg = deg + Sector.DEG_STEP

        self.lowest_line = self._get_line_by_deg(self.start_deg)
        self.highest_line = self._get_line_by_deg(self.end_deg)

        self.is_empty = True
        self.is_chosen = False
        self.is_danger = False

        Sector.LAST_SECTOR_ID = self.id

    def contains_point(self, point: Point):
        return point.is_left_of_line(self.lowest_line) and not point.is_left_of_line(self.highest_line)

    def contains_square(self, square: Square):
        for point in square.points:
            if self.contains_point(point):
                return True
        return False

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
    
    @staticmethod
    def get_diff(sector1, sector2):
        sector1_deg = (sector1.start_deg + sector1.end_deg) / 2
        sector2_deg = (sector2.start_deg + sector2.end_deg) / 2
        
        diff = abs(sector1_deg - sector2_deg)
        if diff >= 180:
            diff = abs(sector1_deg - sector2_deg - 360)
        return diff

    @classmethod
    def generate_sectors(cls):
        return [Sector(i + 1, deg=(1 + cls.DEG_STEP * i) % 360) for i in range(cls.COUNT)]

    @staticmethod
    def _radians(deg):
        return deg * math.pi / 180

    def _get_line_by_deg(self, deg):  # kx - y = 0, (-b, a) > 0
        a = math.tan(self._radians(deg))
        b = -1

        if 90 < deg < 270:
            a *= -1
            b *= -1

        # logger.info(f'deg {deg}: ({a > 0}, {b > 0})')
        return Line(a, b, 0)

    def draw(self, screen, center):
        if self.is_empty and DRAWING_HIDE_EMPTY:
            return

        color = Color.GRAY
        if not self.is_empty:
            color = Color.RED
        elif self.is_chosen:
            color = Color.PURPLE
        if self.is_danger:
            color = Color.RED2

        if not DRAWING_MIDDLE_LANE:
            self.lowest_line.draw(screen, center, color)
            self.highest_line.draw(screen, center, color)
        else:
            middle = self._get_line_by_deg((self.start_deg + self.end_deg) / 2)
            middle.draw(screen, center, color)

    def __repr__(self):
        return f'#{self.id} {self.lowest_line} {self.highest_line}'

class Valley:
    def __init__(self,sectors):
        self.sectors = sectors
        self.width = VALLEY
        self.target_sector = sectors[(self.width//2) + 1]
    
    def get_target_deg(self):
        return (self.target_sector.end_deg + self.target_sector.start_deg) / 2
    def get_median(self):
        
        return self.target_sector._get_line_by_deg((self.target_sector.start_deg + self.target_sector.end_deg) / 2)


# a-bd, where d is distance, d_max = @OBSTACLE_AWARE_DIST
def get_histogram_value(robot: Point, obstacle: Square, sector: Sector, vx, vy):
    # 1 meter = 100 pixels, so iterate through points with @step
    step = 0.04
    top_left = obstacle.left_top
    pixels = []

    point = Point(round(top_left.x, 2), round(top_left.y, 2))

    # width of obstacle in pixels
    width = int(obstacle.width * 100/4)

    for i in range(width):
        for j in range(width):
            pixels.append(Point(point.x + step * i, point.y - step * j))

    res = 0.0
    
    for pixel in pixels:
        if sector.contains_point(pixel):
            # dist = 1 + get_coeff_direction(vx, vy, sector) * pixel.get_dist_to_point(robot)
            dist = pixel.get_dist_to_point(robot)
            dist = math.sqrt(2) * OBSTACLE_AWARE_DIST - dist
            res = res + dist
            
    return res


def dump_obstacle_avoidance(robot_position, robot_angle, ball_predicted_positions,
                            obstacles_predicted_positions: [MovingObstacle]):
    robot_x, robot_y = robot_position
    rangle = robot_angle
    ball_x, ball_y = ball_predicted_positions[0]
    obstacles_positions = [(obstacle[0], obstacle[1], 1, 1) for obstacle in
                           obstacles_predicted_positions]

    hist = {}  # sector to prob

    # maximal distance to obstacle    
    robot_point = Point(0, 0)

    sector_to_obstacles = {} # obstacle with vx,vy
    obstacles = []
    # obstacle_squares = []
    for obstacle_num, obstacle_pos in enumerate(obstacles_positions):
        obstacle_x, obstacle_y, vx, vy = obstacle_pos
        obstacle_point = Point(obstacle_x, obstacle_y).rotate(rangle)
        # need to tune sector around an obstacle
        obstacle = Square(obstacle_point.x, obstacle_point.y, constants.UNITS_RADIUS * 2 + constants.UNITS_RADIUS,
                          coord_center=Point(robot_x, robot_y).rotate(rangle))

        curr_dist = obstacle.get_dist_to_point(robot_point)
        if curr_dist > OBSTACLE_AWARE_DIST:
            # logger.info(f'Skipping obstacle {1 + obstacle_num} {obstacle_pos}')
            continue
        obstacles.append(obstacle)

    ball_point = Point(ball_x, ball_y, coord_center=Point(robot_x, robot_y)).rotate(rangle)
    ball_sector = None
    
    # for each sector find closest obstacle
    for sector in _sectors:
        sector.is_empty = True
        sector.is_chosen = False
        sector.is_danger = False

        min_dist = INF

        hist_val = 0
        sector_to_obstacles[sector.id] = 0
        for obstacle in obstacles:
            val = get_histogram_value(robot_point, obstacle, sector,1,1)
            if val != 0:
                sector_to_obstacles[sector.id] += 1
            hist_val += val
        if sector_to_obstacles[sector.id] != 0:
            hist[sector.id] = hist_val / sector_to_obstacles[sector.id]
        else:
            hist[sector.id] = hist_val

        # get dist
        if sector.contains_point(ball_point):
            ball_sector = sector

    # logger.info(f'historgam {hist}')

    # smooth hist
    smoothed_hist = {i : 0 for i in range(1,Sector.COUNT + 1)}
    
    h_list = list(hist.values())
    for k in range(Sector.COUNT):
        
        hist_vals = [(h_list[(k - i) % Sector.COUNT],h_list[(k + i) % Sector.COUNT]) for i in range(6)]
        i = 5
        
        for j in range(6):
            if j != 0:
                smoothed_hist[k + 1] += i * hist_vals[j][0]
                smoothed_hist[k + 1] += i * hist_vals[j][1]
            else:
                smoothed_hist[k + 1] += i * hist_vals[j][0]
            i = i - 1
        smoothed_hist[k + 1] /= 11
        if smoothed_hist[k+1] >= TRESHOLD:
            _sectors[k].is_empty = False

    if not ball_sector:
        logger.error(f'Unable to identify ball {ball_point} position')
        return robot_x, robot_y

    valleys = []
    danger_sectors = []
    for k,v in smoothed_hist.items():
        if v > DANGER:
            _sectors[k-1].is_danger = True
            danger_sectors.append(_sectors[k - 1])
    
    # logger.warning(f"smooth {smoothed_hist}")
    # logger.info(f"maximum {max(hist.values())}")

    for k in range(Sector.COUNT):
        valley_sectors = [((k + i) % Sector.COUNT + 1, smoothed_hist[(k + i) % Sector.COUNT + 1]) for i in range(VALLEY)]
        if all(map(lambda x : x[1] < TRESHOLD,valley_sectors)):
            valley_sectors = list(map(lambda x : x[0], valley_sectors))
            valleys.append(Valley([sector for sector in _sectors if sector.id in valley_sectors]))

    # choose closest valley
    target_sector = _sectors[0]
    min_diff = INF
    for valley in valleys:
        diff = Sector.get_diff(valley.target_sector,ball_sector)
        if danger_sectors != []:
            if diff < min_diff and all(map(lambda x : Sector.get_diff(valley.target_sector,x) > DANGER_AWARE_ANGLE, danger_sectors)):
                min_diff = diff
                target_sector = valley.target_sector
        elif diff < min_diff:
            min_diff = diff
            target_sector = valley.target_sector


    if not target_sector:
        return robot_x, robot_y

    target_sector.is_chosen = True

    target_line = target_sector._get_line_by_deg((target_sector.start_deg + target_sector.end_deg) / 2)
    target_vec = target_line.get_direction_vector()

    scale = abs(target_vec[0] / target_vec[1])
    if scale < 1:
        target_y = MAX_DIST_TO_GO
        target_x = target_y * scale
    else:
        target_x = MAX_DIST_TO_GO
        target_y = target_x / scale

    target_x *= utils._get_sign(target_vec[0])
    target_y *= utils._get_sign(target_vec[1])

    # logger.warning(f'Should go to ({target_x}, {target_y}) from {target_sector} according to {target_vec}')
    return target_x + robot_x, target_y + robot_y


def drawable_dump_obstacle_avoidance(screen, robot, ball_predicted_positions, obstacles_predicted_positions):
    result = dump_obstacle_avoidance(robot.get_pos(), robot.angle, ball_predicted_positions,
                                     obstacles_predicted_positions)

    for sector in _sectors:
        robot_x, robot_y = robot.get_pos()
        sector.draw(screen, center=Point(robot_x, robot_y))

    return result


MAX_DIST_TO_GO = 0.5
OBSTACLE_AWARE_DIST = 1.5

DRAWING_HIDE_EMPTY = False
DRAWING_MAX_LINE_POINTS = 30
DRAWING_MIDDLE_LANE = True

# need to tune
TRESHOLD = 28
VALLEY = 3
DANGER_AWARE_ANGLE = 50
DANGER = 45

_sectors = Sector.generate_sectors()
logger.warning('\n'.join([str(s) for s in _sectors]))

OBSTACLE_COEF_DRIVE_TO_ROBOT = 0.5


#  Эвриситка раз:  Если робот движется на нас -> хреновое направление
def get_coeff_direction(vx, vy, sector: Sector) -> float:
    direction_vector = sector._get_line_by_deg((sector.start_deg + sector.end_deg) / 2).get_direction_vector()
    direction_obstacle = (-vx, -vy)

    # no movements
    if vx == 0 and vy == 0:
        return 1.0

    direction_vector = utils.normalize_np_vector(direction_vector, 0.1)
    direction_obstacle = utils.normalize_np_vector(direction_obstacle, 0.1)
    angle = numpy.arccos(
        math.radians(direction_obstacle[0] * direction_vector[0] + direction_obstacle[1] * direction_vector[1]))

    if angle <= sector.DEG_STEP:
        return OBSTACLE_COEF_DRIVE_TO_ROBOT
    else:
        return 1
