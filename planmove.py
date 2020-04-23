# Made according to this (many thanks):
# ttps://www.youtube.com/watch?v=Mdg9ElewwA0&feature=emb_logo

import pygame, math, time, random
import numpy as np
from pygame.locals import *

pygame.init()

# Variables for movement algorithm
ro = 0
alpha = 0
beta = 0

# Constants for movement algorithm
k_ro = 1
k_alpha = 8
k_beta = -1.5
l = 1
r = 1

# Constants
# Units here are in metres and radians using our standard coordinate frame
BARRIERRADIUS = 0.1
ROBOTRADIUS = 0.1
ROBOTWIDTH = 2 * ROBOTRADIUS
MAXVELOCITY = 0.3      # ms^(-1) max speed of each wheel
BARRIERVELOCITYRANGE = 0.15
PLAYFIELDCORNERS = (-4.0, -2.5, 4.0, 2.5) # The region we will fill with obstacles

# Starting pose of robot
x = 0
y = 0
theta = 0

# Starting wheel velocities
vL = 0.00
vR = 0.00

# Used for displaying a trail of the robot's positions
locationhistory = []

# Timestep delta to run control and simulation at
dt = 0.1

# Barrier (other players) locations
# Barrier contents are (bx, by, visibilitymask)
barriers = []

# Generate 9 random barriers
for i in range(9):
    (bx, by, vx, vy) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]),
                        random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]),
                        random.gauss(0.0, BARRIERVELOCITYRANGE),
                        random.gauss(0.0, BARRIERVELOCITYRANGE))
    barrier = [bx, by, vx, vy]
    barriers.append(barrier)

# Ball will be just another barrier which doesn't move
(bx, by, vx, vy) = (PLAYFIELDCORNERS[2],
                    PLAYFIELDCORNERS[3],
                    random.gauss(0.0, BARRIERVELOCITYRANGE),
                    random.gauss(0.0, BARRIERVELOCITYRANGE))
barrier = [bx, by, vx, vy]
barriers.append(barrier)
targetindex = 9

# For debugging, prints locations of all barriers (other players and ball)
def printBarriers():
    for (i, barrier) in enumerate(barriers):
        print(i, barrier[0], barrier[1], barrier[2], barrier[3])

# Moves other players (not ball)
def moveBarriers(dt):
    for (i, barrier) in enumerate(barriers):
        if i != targetindex: # we don't want to move the ball
            barriers[i][0] += barriers[i][2] * dt
            if barriers[i][0] < PLAYFIELDCORNERS[0]:
                barriers[i][2] = -barriers[i][2]
            if barriers[i][0] > PLAYFIELDCORNERS[2]:
                barriers[i][2] = -barriers[i][2]
            barriers[i][1] += barriers[i][3] * dt
            if barriers[i][1] < PLAYFIELDCORNERS[1]:
                barriers[i][3] = -barriers[i][3]
            if barriers[i][1] > PLAYFIELDCORNERS[3]:
                barriers[i][3] = -barriers[i][3]

# Constants for graphics display
# Transformation from metric world frame to graphics frame
# k pixels per metre
# Horizontal screen coordinate:     u = u0 + k * x
# Vertical screen coordinate:       v = v0 - k * y

# Set the width and height of the screen (pixels)
WIDTH = 1000
HEIGHT = 600

size = [WIDTH, HEIGHT]
black = (20, 20, 40)
lightblue = (0, 120, 255)
darkblue = (0, 40, 160)
red = (255, 100, 0)
white = (255, 255, 255)
blue = (0, 0, 255)
grey = (70, 70, 70)
green = (0, 204, 0)
k = 100 # pixels per metre for graphics

# Screen centre will correspond to (x, y) = (0, 0)
u0 = WIDTH / 2
v0 = HEIGHT / 2

# Initialise Pygame display screen
screen = pygame.display.set_mode(size)

# Array for path choices use for graphics
pathstodraw = []

# Set new robot position in simulation
# based on current pose and velocity controls.
#
# Uses time deltat in future.
# Returns xnew, ynew, thetanew. Also returns path which is just used for graphics.
def setNewPosition(vL, vR, x, y, theta, deltat):
    # Simple special cases
    # Straight line motion
    if round(vL, 3) == round(vR, 3):
        xnew = x + vL * deltat * math.cos(theta)
        ynew = y + vL * deltat * math.sin(theta)
        thetanew = theta

    # Pure rotation motion
    elif round(vL, 3) == -round(vR, 3):
        xnew = x
        ynew = y
        thetanew = theta + ((vR - vL) * deltat / ROBOTWIDTH)

    else:
        # Rotation and arc angle of general circular motion
        # Using equations given in Lecture 2
        R = ROBOTWIDTH / 2.0 * (vR + vL) / (vR - vL)
        deltatheta = (vR - vL) * deltat / ROBOTWIDTH
        xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
        ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
        thetanew = theta + deltatheta

    return (xnew, ynew, thetanew)

# Draw the barriers (other players and ball) on the screen
def drawBarriers(barriers):
    for (i, barrier) in enumerate(barriers):
        if i == targetindex:
            bcol = red
        else:
            bcol = lightblue
        pygame.draw.circle(screen, bcol,
                           (int(u0 + k * barrier[0]), int(v0 - k * barrier[1])),
                           int(k * BARRIERRADIUS), 0)

# Calculate the closest obstacle at a position (x, y)
def calculateClosestObstacleDistance(x, y):
    closestdist = 100000.0
    # Calculate distance to closest obstacle
    for (i, barrier) in enumerate(barriers):
        if i != targetindex:
            dx = barrier[0] - x
            dy = barrier[1] - y
            d = math.sqrt(dx**2 + dy**2)
            # Distance between closest touching point of circular robot and circular barrier
            dist = d - BARRIERRADIUS - ROBOTRADIUS
            if dist < closestdist:
                closestdist = dist
    return closestdist

def calculateKsiVector(v, omega):
    matrix_3 = np.array([[math.cos(theta), 0],
                         [math.sin(theta), 0],
                         [0, 1]])

    matrix_4 = np.array([[v],
                         [omega]])

    ksi = np.dot(matrix_3, matrix_4)
    return ksi

def calculatePhiVector(ksi):
    matrix_1 = np.array([[1, 0, l],
                         [1, 0, -l],
                         [0, 1, 0]])

    matrix_2 = np.array([[math.cos(theta), math.sin(theta), 0],
                         [-math.sin(theta), math.cos(theta), 0],
                         [0, 0, 1]])

    res = 1/r * np.dot(matrix_1, matrix_2)
    phi = np.dot(res, ksi)

    # print(pygame.surfarray.array2d(screen))
    return (phi[0][0], phi[1][0])

def moveToDot(target_x, target_y):
    # x and y here are robot coordinates
    dx = (target_x - x)
    dy = (target_y - y)

    ro_new = math.sqrt(dx**2 + dy**2)
    alpha_new = -theta + math.atan2(dy, dx)
    beta_new = -theta - alpha_new

    v = k_ro * ro_new
    omega = k_alpha * alpha_new + k_beta * beta_new

    ksi = calculateKsiVector(v, omega)
    (vLchosen, vRchosen) = calculatePhiVector(ksi)

    return (vLchosen, vRchosen, ro_new, alpha_new, beta_new)

def moveToDotAgain(target_x, target_y):
    change_rate = np.array([[-k_ro * ro * math.cos(alpha)],
                            [k_ro * math.sin(alpha) - k_alpha * alpha - k_beta * beta],
                            [-k_ro * math.sin(alpha)]])

    ro_new = ro + change_rate[0][0] * dt
    alpha_new = alpha + change_rate[1][0] * dt
    beta_new = beta + change_rate[2][0] * dt

    v = k_ro * ro_new
    omega = k_alpha * alpha_new + k_beta * beta_new

    ksi = calculateKsiVector(v, omega)
    (vLchosen, vRchosen) = calculatePhiVector(ksi)

    return (vLchosen, vRchosen, ro_new, alpha_new, beta_new)

def obstacleAvoidance():
    # No obstacle avoidance by default, just move to the ball
    target_x = PLAYFIELDCORNERS[2]
    target_y = PLAYFIELDCORNERS[3]
    return (target_x, target_y)

# We will calculate time
startTime = time.time()

target_x = x
target_y = y

# Main loop
while True:
    Eventlist = pygame.event.get()

    # For display of trail
    locationhistory.append((x, y))

    # Planning
    disttotarget = math.sqrt((x - target_x)**2 + (y - target_y)**2)
    if disttotarget < ROBOTRADIUS:
        print("Calling Obstacle Avoidance algorithm")
        # Calculate best target point and call moveToDot
        (target_x, target_y) = obstacleAvoidance()
        (vL, vR, ro, alpha, beta) = moveToDot(target_x, target_y)

        if vL > MAXVELOCITY or vR > MAXVELOCITY:
            if vL > vR:
                diff = vR / vL
                vL = 0.3
                vR = vL * diff
            elif vR > vL:
                diff = vL / vR
                vR = 0.3
                vL = vR * diff
            else:
                vL = 0.3
                vR = 0.3
    else:
        print("stillMovingToDot")
        (vL, vR, ro, alpha, beta) = moveToDotAgain(target_x, target_y)

        if vL > MAXVELOCITY or vR > MAXVELOCITY:
            if vL > vR:
                diff = vR / vL
                vL = 0.3
                vR = vL * diff
            elif vR > vL:
                diff = vL / vR
                vR = 0.3
                vL = vR * diff
            else:
                vL = 0.3
                vR = 0.3

    screen.fill(black)
    for loc in locationhistory:
        pygame.draw.circle(screen, grey, (int(u0 + k * loc[0]), int(v0 - k * loc[1])), 3, 0)
    drawBarriers(barriers)

    # Draw robot
    u = u0 + k * x
    v = v0 - k * y
    pygame.draw.circle(screen, white, (int(u), int(v)), int(k * ROBOTRADIUS), 3)
    # Draw wheels as little blobs so you can see robot orientation
    # left wheel centre
    wlx = x - (ROBOTWIDTH/2.0) * math.sin(theta)
    wly = y + (ROBOTWIDTH/2.0) * math.cos(theta)
    ulx = u0 + k * wlx
    vlx = v0 - k * wly
    WHEELBLOB = 0.04
    pygame.draw.circle(screen, blue, (int(ulx), int(vlx)), int(k * WHEELBLOB))
    # right wheel centre
    wrx = x + (ROBOTWIDTH/2.0) * math.sin(theta)
    wry = y - (ROBOTWIDTH/2.0) * math.cos(theta)
    urx = u0 + k * wrx
    vrx = v0 - k * wry
    pygame.draw.circle(screen, blue, (int(urx), int(vrx)), int(k * WHEELBLOB))

    # Update display
    pygame.display.flip()

    # Actually now move robot based on chosen vL and vR
    (x, y, theta) = setNewPosition(vL, vR, x, y, theta, dt)

    moveBarriers(dt)
    # printBarriers()

    # Check if robot has reached target
    disttotarget = math.sqrt((x - barriers[targetindex][0])**2 + (y - barriers[targetindex][1])**2)
    if disttotarget < (BARRIERRADIUS + ROBOTRADIUS):
        print("Result: ", time.time() - startTime, "sec")
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit()

    # Sleeping dt here runs simulation in real-time
    time.sleep(dt / 50)
