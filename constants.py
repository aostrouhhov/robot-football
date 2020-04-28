# Variables for movement algorithm
ro_start = 0
alpha_start = 0
beta_start = 0

# Starting wheel velocities
vl_start = 0.00
vr_start = 0.00

# A starting pose of robot
x_start = -4.0
y_start = -2.5
theta_start = 0

# Constants for movement algorithm
k_ro = 0.5
k_alpha = 5
k_beta = 1
l = -0.1
r = 1

# Constants
# Units here are in metres and radians using our standard coordinate frame
BARRIERRADIUS = 0.14
ROBOTRADIUS = 0.14
WHEELBLOB = 0.04
ROBOTWIDTH = 2 * ROBOTRADIUS
MAXVELOCITY = 0.3  # ms^(-1) max speed of each wheel
BARRIERVELOCITYRANGE = 0.15
PLAYFIELDCORNERS = (-4.0, -2.5, 4.0, 2.5)  # The region we will fill with obstacles

# Time step delta to run control and simulation at
dt = 0.1

# Constants for graphics display
# Transformation from metric world frame to graphics frame
# k pixels per metre
# Horizontal screen coordinate:     u = u0 + k * x
# Vertical screen coordinate:       v = v0 - k * y

# Set the width and height of the screen (pixels)
WIDTH = 800
HEIGHT = 500
size = [WIDTH, HEIGHT]
# Screen centre will correspond to (x, y) = (0, 0)
u0 = WIDTH / 2
v0 = HEIGHT / 2

black = (0, 0, 0)
lightblue = (0, 120, 255)
darkblue = (0, 40, 160)
red = (255, 100, 0)
white = (255, 255, 255)
blue = (0, 0, 255)
grey = (70, 70, 70)
green = (0, 204, 0)
ball_edge_color = (255, 255, 0)
barrier_edge_color = (0, 255, 0)
k = 100  # pixels per metre for graphics
