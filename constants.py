# Constants for movement algorithm
k_ro = 0.4
k_alpha = 8
k_beta = -4
l = 0.1
r = 1

# Constants
# Units here are in metres and radians using our standard coordinate frame
WINDOW_CORNERS = (-4.0, -2.5, 4.0, 2.5)  # The region we will fill with obstacles
UNITS_RADIUS = 0.14

# A starting pose of robot
x_start = -4.0 + UNITS_RADIUS
# x_start = 4.0 - UNITS_RADIUS
y_start = -2.5 + UNITS_RADIUS
# y_start = 2.5 - UNITS_RADIUS
theta_start = 0
# theta_start = -3.14159265359

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
    GREY = (70, 70, 70)
    BLUE = (0, 0, 255)
    RED = (255, 100, 0)
    GREEN = (0, 204, 0)
    LIGHTBLUE = (0, 120, 255)


k = 100  # pixels per metre for graphics

# Game settings
OBSTACLES_COUNT = 2
OBSTACLE_VELOCITY_RANGE = 0.1
ROBOT_MAX_VELOCITY = 1

# Fix random seed to reproduce results. Set None if no fixation is needed
RANDOM_SEED = 239