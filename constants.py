# Constants for movement algorithm
l = 0.7     # distance to wheels
r = 1       # wheel radius

# Constants
# Units here are in metres and radians using our standard coordinate frame
WINDOW_CORNERS = (-4.0, -2.5, 4.0, 2.5)  # The region we will fill with obstacles
UNITS_RADIUS = 0.14

# A starting pose of robot
x_start_left = -4.0 + UNITS_RADIUS
y_start_left = -2.5 + UNITS_RADIUS

x_start_right = 3.9 - UNITS_RADIUS
y_start_right = -2.5 + UNITS_RADIUS

theta_start = 0

# Constants for graphics display
# Transformation from metric world frame to graphics frame
# k pixels per metre
# Horizontal screen coordinate:     u = u0 + k * x
# Vertical screen coordinate:       v = v0 - k * y

# Set the width and height of the screen (pixels)
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 500
u0 = WINDOW_WIDTH / 2
v0 = WINDOW_HEIGHT / 2


class Color:
    YELLOW = (255, 255, 0)
    WHITE = (255, 255, 255)
    BLACK = (20, 20, 40)
    GRAY = (70, 70, 70)
    BLUE = (0, 0, 255)
    RED = (255, 100, 0)
    RED2 = (255,0,0)
    GREEN = (0, 204, 0)
    LIGHTBLUE = (0, 120, 255)
    PURPLE = (148, 0, 201)

DRAWABLE_OBS_AVOIDANCE = True

k = 100  # pixels per metre for graphics

# Game settings
dt = 0.1
SIMULATION_DELAY = 1000

OBSTACLES_COUNT = 9
OBSTACLE_VELOCITY_RANGE = 0.1
ROBOT_MAX_VELOCITY = 1
ROBOT_HUNT_DISTANCE = 0.75
ROBOT_MAX_HUNT_VELOCITY = 1.25

# Fix random seed to reproduce results. Set None if no fixation is needed
RANDOM_SEED = 239

# Fix random seed to reproduce results. Set None if no fixation is needed
RANDOM_SEED = 23
