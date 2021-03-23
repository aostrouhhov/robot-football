import math
import logging
import random

import cv2

import constants
from constants import Color


class Drawable:
    def __init__(self, x, y):
        self._x = x
        self._y = y

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    def set_pos(self, x, y):
        self._x = x
        self._y = y

    def get_pos(self):
        return self._x, self._y

    @staticmethod
    def get_coords_on_screen(coords):
        return int(constants.WINDOW_WIDTH / 2 + constants.k * coords[0]), \
               int(constants.WINDOW_HEIGHT / 2 - constants.k * coords[1])

    def draw(self, screen):
        pass


class MovingObstacle(Drawable):
    RADIUS = constants.UNITS_RADIUS
    VELOCITY_RANGE = constants.OBSTACLE_VELOCITY_RANGE

    SCREEN_RADIUS = int(RADIUS * constants.k)
    COLOR = Color.LIGHTBLUE

    def __init__(self, x, y, vx, vy):
        super().__init__(x, y)

        self._vx = vx
        self._vy = vy

    @classmethod
    def create_randomized(cls):
        x = random.uniform(constants.WINDOW_CORNERS[0] + cls.RADIUS * 2,
                           constants.WINDOW_CORNERS[2] - cls.RADIUS * 2)
        y = random.uniform(constants.WINDOW_CORNERS[1] + cls.RADIUS * 2,
                           constants.WINDOW_CORNERS[3] - cls.RADIUS * 2)
        vx = random.gauss(0.0, cls.VELOCITY_RANGE)
        vy = random.gauss(0.0, cls.VELOCITY_RANGE)

        result = MovingObstacle(x, y, vx, vy)
        return result

    def move(self, dt):
        self._x += self._vx * dt
        if self._x < constants.WINDOW_CORNERS[0] + MovingObstacle.RADIUS \
                or self._x > constants.WINDOW_CORNERS[2] - MovingObstacle.RADIUS:
            self._vx = -self._vx

        self._y += self._vy * dt
        if self._y < constants.WINDOW_CORNERS[1] + MovingObstacle.RADIUS or \
                self._y > constants.WINDOW_CORNERS[3] - MovingObstacle.RADIUS:
            self._vy = -self._vy

    def draw(self, screen):
        pos = self.get_coords_on_screen(self.get_pos())
        cv2.circle(screen, pos, self.SCREEN_RADIUS, self.COLOR, thickness=-1)


class Ball(MovingObstacle):
    COLOR = Color.RED

    @classmethod
    def create_randomized(cls):
        assert constants.RANDOM_SEED != 0, "Value 0 for random seed is not allowed. If no seed needed, set None"
        if constants.RANDOM_SEED is not None:
            random.seed(constants.RANDOM_SEED)
        x = constants.WINDOW_CORNERS[2]-4 + random.uniform(-1, 1)
        y = constants.WINDOW_CORNERS[3]-2.5 + random.uniform(-2, 2)
        vx = random.gauss(0.0, cls.VELOCITY_RANGE)
        vy = random.gauss(0.0, cls.VELOCITY_RANGE)

        result = Ball(x, y, vx, vy)
        return result

    def draw(self, screen):
        pos = self.get_coords_on_screen(self.get_pos())
        cv2.circle(screen, pos, self.SCREEN_RADIUS, self.COLOR, thickness=-1)


class Wheel(Drawable):
    RADIUS = 0.04

    COLOR = Color.GREEN
    SCREEN_RADIUS = int(RADIUS * constants.k)

    class Kind:
        LEFT = 'left'
        RIGHT = 'right'

    def __init__(self, x, y, kind, robot):
        super().__init__(x, y)

        self.velocity = 0

        self.kind = kind
        self.robot = robot

        self.set_pos(x, y)  # setting new pos with offsets

    def _calc_x(self):
        robot_x, _ = self.robot.get_pos()
        offset = (self.robot.WIDTH / 2) * math.sin(self.robot.angle)
        return robot_x - offset if self.kind == Wheel.Kind.LEFT else robot_x + offset

    def _calc_y(self):
        _, robot_y = self.robot.get_pos()
        offset = (self.robot.WIDTH / 2) * math.cos(self.robot.angle)
        return robot_y + offset if self.kind == Wheel.Kind.LEFT else robot_y - offset

    def set_pos(self, x, y):
        self._x = self._calc_x()
        self._y = self._calc_y()

    def get_pos(self):
        return self._calc_x(), self._calc_y()

    def draw(self, screen):
        pos = self.get_coords_on_screen(self.get_pos())
        cv2.circle(screen, pos, self.SCREEN_RADIUS, self.COLOR, thickness=2)


class Robot(Drawable):
    RADIUS = constants.UNITS_RADIUS
    WIDTH = constants.UNITS_RADIUS * 2

    POS_HISTORY_LIMIT = 100

    SCREEN_WIDTH = int(WIDTH * constants.k)
    SCREEN_RADIUS = int(RADIUS * constants.k)
    TRAIL_COLOR = Color.GRAY
    TRAIL_SCREEN_RADIUS = 3
    DIRECTION_COLOR = Color.GREEN

    def __init__(self, x, y, angle, color):
        super().__init__(x, y)

        self._angle = angle
        self.pos_history = []
        self.COLOR = color

        self.wheels = (
            Wheel(self._x, self._y, Wheel.Kind.LEFT, self),
            Wheel(self._x, self._y, Wheel.Kind.RIGHT, self)
        )

    @property
    def angle(self):
        return self._angle

    def set_pos(self, x, y):
        self._x = x
        self._y = y

        self.pos_history.append((x, y))
        self.pos_history = self.pos_history[-Robot.POS_HISTORY_LIMIT:]

    def set_velocity(self, vel_left, vel_right):
        self.wheels[0].velocity = vel_left
        self.wheels[1].velocity = vel_right

    def set_angle(self, new_angle):
        self._angle = new_angle

    def move(self, dt):
        vel_left = self.wheels[0].velocity
        vel_right = self.wheels[1].velocity

        logging.info(f'Moving robot: origin=({self._x}, {self._y}, {self._angle}), vel=({vel_left}, {vel_right})')

        if round(vel_left, 3) == round(vel_right, 3):  # Straight line motion
            x_new = self._x + vel_left * dt * math.cos(self._angle)
            y_new = self._y + vel_right * dt * math.sin(self._angle)
            theta = self._angle
        elif round(vel_left, 3) == -round(vel_right, 3):  # Pure rotation motion
            x_new, y_new = self.get_pos()
            theta = self._angle + ((vel_right - vel_left) * dt / self.WIDTH)
        else:  # Rotation and arc angle of general circular motion (Lecture 2)
            _r = self.WIDTH / 2.0 * (vel_right + vel_left) / (vel_right - vel_left)
            delta_theta = (vel_right - vel_left) * dt / self.WIDTH
            x_new = self._x + _r * (math.sin(delta_theta + self._angle) - math.sin(self._angle))
            y_new = self._y - _r * (math.cos(delta_theta + self._angle) - math.cos(self._angle))
            theta = self._angle + delta_theta

        self.set_angle(theta)
        self.set_pos(x_new, y_new)

    def _draw_direction(self, screen):
        start_on_screen = self.get_coords_on_screen(self.get_pos())

        end_x = self.x + self.RADIUS * math.cos(self.angle)
        end_y = self.y + self.RADIUS * math.sin(self.angle)
        end_on_screen = self.get_coords_on_screen((end_x, end_y))

        cv2.line(screen, start_on_screen, end_on_screen, self.DIRECTION_COLOR, thickness=1)

    def _draw_wheels(self, screen):
        for wheel in self.wheels:
            wheel.draw(screen)

    def draw(self, screen):
        for pos in self.pos_history:
            pos_on_screen = self.get_coords_on_screen(pos)
            cv2.circle(screen, pos_on_screen, self.TRAIL_SCREEN_RADIUS,
                       self.TRAIL_COLOR, thickness=-1)

        self._draw_wheels(screen)

        pos = self.get_coords_on_screen((self._x, self._y))
        cv2.circle(screen, pos, self.SCREEN_RADIUS, self.COLOR, thickness=3)

        self._draw_direction(screen)

    def get_closest_dist_to_obstacle(self, obstacles):
        closest_dist = 100000.0
        for i, player in enumerate(obstacles):
            p_x, p_y = player.get_pos()

            dx = p_x - self._x
            dy = p_y - self._y

            d = math.sqrt(dx ** 2 + dy ** 2)

            # Distance between the closest touching point of circular robot and circular barrier
            dist = d - MovingObstacle.RADIUS - Robot.RADIUS
            if dist < closest_dist:
                closest_dist = dist

        return closest_dist

    def get_dist_to_target(self, target):
        target_x, target_y = target.get_pos()
        return math.sqrt((self._x - target_x) ** 2 + (self._y - target_y) ** 2)
