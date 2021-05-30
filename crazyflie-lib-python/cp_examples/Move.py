"""
Aerial robotics course, crazyflie miniproject 2021
Alexandre Clivaz, Bruno Da Costa, Maïk Guihard
"""
###############################################################################
############################### Imports #######################################
###############################################################################

from scipy import *
from math import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import cv2
import time
import sys
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.positioning.motion_commander import MotionCommander


###############################################################################
#################### Functions used for the grid ##############################
###############################################################################

def create_empty_plot(size_x, size_y):
    fig, ax = plt.subplots(figsize=(7, 7))
    major_ticks_x = np.arange(0, size_x + 1, 5)
    minor_ticks_x = np.arange(0, size_x + 1, 1)
    major_ticks_y = np.arange(0, size_y + 1, 5)
    minor_ticks_y = np.arange(0, size_y + 1, 1)
    ax.set_xticks(major_ticks_y)
    ax.set_xticks(minor_ticks_y, minor=True)
    ax.set_yticks(major_ticks_x)
    ax.set_yticks(minor_ticks_x, minor=True)
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)
    ax.set_ylim([-1, size_x])
    ax.set_xlim([-1, size_y])
    ax.grid(True)
    return fig, ax


def is_close(range):
    MIN_DISTANCE = 0.22  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


def update_grid(grid, x, y):
    update_bool = False
    multx = img_size[0] / 5
    multy = img_size[1] / 3
    dists = [0.1, 0.2]
    if (is_close(multiranger.front)):
        for i in dists:
            if(x+i<5):
                if grid[floor((x + i) * multx)][floor((y) * multy)] != 1:
                    update_bool = True
                    print(str(floor((x + i) * multx)) + " " + str(floor((y) * multy)))
                grid[floor((x + i) * multx)][floor((y) * multy)] = 1
    if (is_close(multiranger.back)):
        for i in dists:
            if(x-i>=0):
                if grid[floor((x - i) * multx)][floor((y) * multy)] != 1:
                    update_bool = True
                    print(str(floor((x - i) * multx)) + " " + str(floor((y) * multy)))
                grid[floor((x - i) * multx)][floor((y) * multy)] = 1
    if (is_close(multiranger.left)):
        for i in dists:
            if(y+i<3):
                if grid[floor((x) * multx)][floor((y + i) * multy)] != 1:
                    update_bool = True
                    print(str(floor((x) * multx)) + " " + str(floor((y + i) * multy)))
                grid[floor((x) * multx)][floor((y + i) * multy)] = 1
    if (is_close(multiranger.right)):
        for i in dists:
            if(y-i>=0):
                if grid[floor((x) * multx)][floor((y - 1) * multy)] != 1:
                    update_bool = True
                    print(str(floor((x) * multx)) + " " + str(floor((y - 1) * multy)))
                grid[floor((x) * multx)][floor((y - i) * multy)] = 1
    return grid, update_bool


def color_zone(grid):
    for y in range(30):
        for x in range(15):
            if grid[x][y] == 0:
                grid[x][y] = 3
        for x in range(35, 50):
            if grid[x][y] == 0:
                grid[x][y] = 4
    return grid


def grow_obstacles(grid):
    sx, sy = grid.shape
    sx = range(sx)
    sy = range(sy)
    for x in sx:
        for y in sy:
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    if x + i in sx and y + j in sy:
                        if grid[x + i][y + j] == 1:
                            if grid[x][y] != 1:
                                grid[x][y] = -1;
    grid[grid == -1] = 1
    return grid


def dijkstra(grid, objx, objy, x, y):
    grid_value = np.full((grid.shape[0], grid.shape[1]), 1000)
    objx = floor(10 * objx)
    objy = floor(10 * objy)
    # print(grid.shape[0])
    grid_value[floor(10 * x)][floor(10 * y)] = 0
    value = 0
    while (grid_value[objx][objy] >= 1000):
        for i in range(grid.shape[0]):
            if (i <= floor(10 * x) + value + 1 and i >= floor(10 * x) - value - 1):
                for j in range(grid.shape[1]):
                    if (j <= floor(10 * y) + value + 1 and j >= floor(10 * y) - value - 1):
                        if (grid_value[i][j] == value):
                            if (i - 1 >= 0):
                                if (grid[i - 1][j] != 1 and grid_value[i - 1][j] > value + 1):
                                    grid_value[i - 1][j] = value + 1
                            if (i + 1 < grid.shape[0]):
                                if (grid[i + 1][j] != 1 and grid_value[i + 1][j] > value + 1):
                                    grid_value[i + 1][j] = value + 1
                            if (j - 1 >= 0):
                                if (grid[i][j - 1] != 1 and grid_value[i][j - 1] > value + 1):
                                    grid_value[i][j - 1] = value + 1
                            if (j + 1 < grid.shape[1]):
                                if (grid[i][j + 1] != 1 and grid_value[i][j + 1] > value + 1):
                                    grid_value[i][j + 1] = value + 1

        value += 1
    x_tmp = objx
    y_tmp = objy
    value = grid_value[objx][objy]
    path_list = []
    path_list.append([x_tmp, y_tmp])

    while (value > 0):
        if (x_tmp - 1 >= 0):
            if (grid_value[x_tmp - 1][y_tmp] == value - 1):
                x_tmp = x_tmp - 1
                path_list.append([x_tmp, y_tmp])
                value -= 1
                continue
        if (x_tmp + 1 < grid.shape[0]):
            if (grid_value[x_tmp + 1][y_tmp] == value - 1):
                x_tmp = x_tmp + 1
                path_list.append([x_tmp, y_tmp])
                value -= 1
                continue
        if (y_tmp - 1 >= 0):
            if (grid_value[x_tmp][y_tmp - 1] == value - 1):
                y_tmp = y_tmp - 1
                path_list.append([x_tmp, y_tmp])
                value -= 1
                continue
        if (y_tmp + 1 < grid.shape[1]):
            if (grid_value[x_tmp][y_tmp + 1] == value - 1):
                y_tmp = y_tmp + 1
                path_list.append([x_tmp, y_tmp])
                value -= 1
                continue
    print("List before" + str(path_list))
    if len(path_list)!=0:
        path_list.pop()
    print("List after" + str(path_list))
    return path_list

def dijkstra_landing(grid, x, y):
    grid_value = np.full((grid.shape[0], grid.shape[1]), 1000)
    # print(grid.shape[0])
    grid_value[floor(10 * x)][floor(10 * y)] = 0
    value = 0
    dest_found = False
    print("Start dijkstra")
    while (not dest_found):
        for i in range(35, grid.shape[0]):
            if (i <= floor(10 * x) + value + 1 and i >= floor(10 * x) - value - 1):
                for j in range(grid.shape[1]):
                    if (j <= floor(10 * y) + value + 1 and j >= floor(10 * y) - value - 1):
                        if (grid_value[i][j] == value):
                            if (i - 1 >= 35):
                                if (grid[i - 1][j] != 1 and grid_value[i - 1][j] > value + 1):
                                    grid_value[i - 1][j] = value + 1
                                    print("X: " + str(i-1) +" Y: "+ str(j) +" value: " + str(value+1))
                                    if(grid[i - 1][j] != 6):
                                        dest_found = True
                                        x_tmp = i - 1
                                        y_tmp = j
                                        continue
                            if (i + 1 < grid.shape[0]):
                                if (grid[i + 1][j] != 1 and grid_value[i + 1][j] > value + 1):
                                    grid_value[i + 1][j] = value + 1
                                    print("X: " + str(i+1) +" Y: "+ str(j) +" value: " + str(value+1))
                                    if (grid[i + 1][j] != 6):
                                        dest_found = True
                                        x_tmp = i + 1
                                        y_tmp = j
                                        continue
                            if (j - 1 >= 0):
                                if (grid[i][j - 1] != 1 and grid_value[i][j - 1] > value + 1):
                                    grid_value[i][j - 1] = value + 1
                                    print("X: " + str(i) +" Y: "+ str(j-1) +" value: " + str(value+1))
                                    if (grid[i][j - 1] != 6):
                                        dest_found = True
                                        x_tmp = i
                                        y_tmp = j - 1
                                        continue
                            if (j + 1 < grid.shape[1]):
                                if (grid[i][j + 1] != 1 and grid_value[i][j + 1] > value + 1):
                                    grid_value[i][j + 1] = value + 1
                                    print("X: " + str(i) +" Y: "+ str(j+1) +" value: " + str(value+1))
                                    if(grid[i][j+1] != 6):
                                        dest_found = True
                                        x_tmp = i
                                        y_tmp = j + 1
                                        continue

        value += 1

    print("Found destination Dijkstra: "+ str(x_tmp)+" "+str(y_tmp))
    print("From: " + str(floor(10 * x)) + " " + str(floor(10 * y)))
    value = grid_value[x_tmp][y_tmp]
    path_list = []
    path_list.append([x_tmp, y_tmp])

    while (value > 0):
        if (x_tmp - 1 >= 35):
            if (grid_value[x_tmp - 1][y_tmp] == value - 1):
                x_tmp = x_tmp - 1
                path_list.append([x_tmp, y_tmp])
                print("Path append: " + str(x_tmp)+" "+str(y_tmp))
                value -= 1
                continue
        if (x_tmp + 1 < grid.shape[0]):
            if (grid_value[x_tmp + 1][y_tmp] == value - 1):
                x_tmp = x_tmp + 1
                path_list.append([x_tmp, y_tmp])
                print("Path append: " + str(x_tmp)+" "+str(y_tmp))
                value -= 1
                continue
        if (y_tmp - 1 >= 0):
            if (grid_value[x_tmp][y_tmp - 1] == value - 1):
                y_tmp = y_tmp - 1
                path_list.append([x_tmp, y_tmp])
                print("Path append: " + str(x_tmp)+" "+str(y_tmp))
                value -= 1
                continue
        if (y_tmp + 1 < grid.shape[1]):
            if (grid_value[x_tmp][y_tmp + 1] == value - 1):
                y_tmp = y_tmp + 1
                path_list.append([x_tmp, y_tmp])
                print("Path append: " + str(x_tmp)+" "+str(y_tmp))
                value -= 1
                continue
    print("List before" + str(path_list))
    if len(path_list)!=0:
        path_list.pop()
    print("List after" + str(path_list))
    return path_list

###############################################################################
####################### Functions for each state ##############################
###############################################################################

def get_off_U(x, y, VELOCITY=0.2):
    while is_close(multiranger.right) and is_close(multiranger.left) and not is_close(multiranger.back):
        motion_commander.start_linear_motion(-VELOCITY, 0, 0)
        time.sleep(0.1)
        x -= VELOCITY * 0.1
    if not is_close(multiranger.right) and not is_close(multiranger.left):
        if y < 1.5:
            motion_commander.start_linear_motion(0, VELOCITY, 0)
            time.sleep(0.5)
            y += VELOCITY * 0.5
        else:
            motion_commander.start_linear_motion(0, -VELOCITY, 0)
            time.sleep(0.5)
            y -= VELOCITY * 0.5
    elif not is_close(multiranger.right):
        motion_commander.start_linear_motion(0, -VELOCITY, 0)
        time.sleep(0.5)
        y -= VELOCITY * 0.5
    else:
        motion_commander.start_linear_motion(0, VELOCITY, 0)
        time.sleep(0.5)
        y += VELOCITY * 0.5
    return x, y

# Advance State : drone reaches landing region while avoiding obstacles
# Obstacle avoidance is set to prioritize displacement towards the y-center
# of the map.
def advance(x, y):
    vx = 0
    vy = 0
    global STATE
    global VELOCITY
    global checkedright
    global checkedleft
    global direction
    if (is_close(multiranger.front)):
        if is_close(multiranger.right) and is_close(multiranger.left):
            print("U")
            (x, y) = get_off_U(x, y)
        elif (is_close(multiranger.right)):
            print("Left")
            if y > 2.8:
                STATE = CORNER
            else:
                vy = VELOCITY
            if checkedright:
                direction = 1
        elif (is_close(multiranger.left)):
            print("Right")
            if y < 0.2:
                STATE = CORNER
            else:
                vy = -VELOCITY
            if checkedleft:
                direction = -1
        else:
            if y < 1.5 and direction == 0:
                vy = VELOCITY
                direction = 1
                checkedleft = True
            elif direction == 0:
                vy = -VELOCITY
                direction = -1
                checkedright = True
            else:
                vy = VELOCITY * direction
    else:
        print("Front")
        vx = VELOCITY
        direction = 0
        checkedright = False
        checkedleft = False
    return x, y, vx, vy

# Obstacle avoidance when an encounter with an obstacle happens near the
# border of the map
def corner(x, y):
    vx = 0
    vy = 0
    global STATE
    global VELOCITY
    if y < 1.5:
        if (is_close(multiranger.left)):
            vx = -VELOCITY
        else:
            motion_commander.start_linear_motion(0, VELOCITY, 0)
            time.sleep(0.5)
            y += VELOCITY * 0.5
            if not (is_close(multiranger.front)):
                STATE = ADVANCE
    else:
        if (is_close(multiranger.right)):
            vx = -VELOCITY
        else:
            motion_commander.start_linear_motion(0, -VELOCITY, 0)
            time.sleep(0.5)
            y -= VELOCITY * 0.5
            if not (is_close(multiranger.front)):
                STATE = ADVANCE
    return x, y, vx, vy

# Search for the landing pad when in the landing region
# Landing pad is detected when the z-range finder records a decrease
def goal(x, y):
    vx = 0
    vy = 0
    delay = 1
    global VELOCITY
    global keep_flying
    global line0
    global line3
    global STATE
    global x_last
    global updated_bool
    global time_start

    if multiranger.down > height_thresh_rise:
        if y <= 1.5 and not line0:
            vy = -VELOCITY
            x_last = x
        elif y > 1.5 and not line3:
            vy = VELOCITY
            x_last = x
        if y < 0.15 or is_close(multiranger.right):
            line0 = True
        if y > 2.85 or is_close(multiranger.left):
            line3 = True
        if line3 and line0:
            if x > 4.85:
                print("goal not found")
                STATE = GOALMISSED
                updated_bool = True
            elif is_close(multiranger.front):
                if y > 1.5:
                    vy = -VELOCITY
                else:
                    vy = VELOCITY
            else:
                if x - x_last < 0.2:
                    vx = VELOCITY
                else:
                    line0 = False
                    line3 = False
        elif line0 and not is_close(multiranger.left):
            vy = VELOCITY
        elif line3 and not is_close(multiranger.right):
            vy = -VELOCITY

        for i in range(3):
            ind_x = floor(10 * x) + i - 1
            if ind_x >= 0 and ind_x < grid.shape[0]:
                for j in range(3):
                    ind_y = floor(10 * y) + j - 1
                    if ind_y >= 0 and ind_y < grid.shape[1]:
                        if grid[ind_x][ind_y] == 0:
                            grid[ind_x][ind_y] = 6

    else:
        motion_commander.start_linear_motion(0, 0, 0)
        time.sleep(delay)
        time_start += delay
        STATE = LANDING
        VELOCITY = 0.1
        line0 = False
        line3 = False
    return x, y, vx, vy

def goalmissed(x, y, path_return):
    vx = 0
    vy = 0
    global VELOCITY
    global keep_flying
    global STATE
    global time_start

    for i in range(3):
        ind_x = floor(10 * x) + i - 1
        if ind_x >= 0 and ind_x < grid.shape[0]:
            for j in range(3):
                ind_y = floor(10 * y) + j - 1
                if ind_y >= 0 and ind_y < grid.shape[1]:
                    if grid[ind_x][ind_y] == 0:
                        grid[ind_x][ind_y] = 6

    if multiranger.down > height_thresh_rise:
        if updated_bool or len(path_return) == 0:
            path_return = dijkstra_landing(grid, x, y)
        dist_x = path_return[-1][0] + 0.5 - x * 10
        dist_y = path_return[-1][1] + 0.5 - y * 10
        print("Current postion coords: X=" + str(round(x * 10, 2)) + " Y:" + str(round(y * 10, 2)))
        print( "Current objective coords: X=" + str(path_return[-1][0]) + " Y:" + str(path_return[-1][1]))
        if abs(dist_x) < VELOCITY and abs(dist_y) < VELOCITY:
            path_return.pop()
            if len(path_return) == 0:
                path_return = dijkstra_landing(grid, x, y)
            dist_x = path_return[-1][0] + 0.5 - x * 10
            dist_y = path_return[-1][1] + 0.5 - y * 10
        if abs(dist_x) >= VELOCITY:
            vx = np.sign(dist_x) * VELOCITY
        if abs(dist_y) >= VELOCITY:
            vy = np.sign(dist_y) * VELOCITY

    else:
        motion_commander.start_linear_motion(0, 0, 0)
        time.sleep(1)
        time_start += 1
        STATE = LANDING
        VELOCITY = 0.1

    return x, y, vx, vy, path_return


# Detect the edge of the landing pad in order to land in its center
def landing(x, y, prev_vx, prev_vy):
    vx = 0
    vy = 0
    delay = 1
    global first
    global L_STATE
    global STATE
    global VELOCITY
    global y_l
    global y_r
    global x_b
    global x_f
    global x_found
    global y_found
    global keep_flying
    global goal_pos
    global path_return
    global grid
    global time_start

    if prev_vy != 0 and first:
        if prev_vy > 0:
            y_r = y - 0.05
            L_STATE = L_LEFT
        else:
            y_l = y + 0.05
            L_STATE = L_RIGHT
    if prev_vx != 0 and first:
        if prev_vx > 0:
            x_b = x
            L_STATE = L_FRONT
        else:
            x_f = x
            L_STATE = L_BACK
    if L_STATE == L_LEFT:
        vy = VELOCITY
        if not x_found:
            if multiranger.down > height_thresh_fall and abs(y - y_r) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(delay)
                time_start += delay
                y_l = y
                L_STATE = L_MIDDLE_Y
        else:
            if multiranger.down > height_thresh_fall and abs(y - y_r) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(delay)
                time_start += delay
                y_l = y
                L_STATE = L_MIDDLE_Y
    elif L_STATE == L_RIGHT:
        vy = -VELOCITY
        if not x_found:
            if multiranger.down > height_thresh_fall and abs(y - y_l) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(delay)
                time_start += delay
                y_r = y
                L_STATE = L_MIDDLE_Y
        else:
            if multiranger.down < height_thresh_rise:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(delay)
                time_start += delay
            if multiranger.down > height_thresh_fall:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(delay)
                time_start += delay
                y_r = y
                L_STATE = L_LEFT
    elif L_STATE == L_MIDDLE_Y:
        diff = (y_l + y_r) / 2 - y
        vy = np.sign(diff) * VELOCITY
        if abs(diff) < 0.02:
            vy = np.sign(diff) * VELOCITY / 2
        if abs(diff) < 0.01:
            motion_commander.start_linear_motion(0, 0, 0)
            time.sleep(delay)
            time_start += delay
            L_STATE = L_BACK
            y_found = True
    elif L_STATE == L_BACK:
        vx = -VELOCITY
        if not y_found:
            if multiranger.down > height_thresh_fall and abs(x - x_f) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(delay)
                time_start += delay
                x_b = x
                L_STATE = L_MIDDLE_X
        else:
            if multiranger.down > height_thresh_fall:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(delay)
                time_start += delay
                x_b = x
                L_STATE = L_FRONT
    elif L_STATE == L_FRONT:
        vx = VELOCITY
        if not y_found:
            if multiranger.down > height_thresh_fall and abs(x - x_b) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(delay)
                time_start += delay
                x_f = x
                L_STATE = L_MIDDLE_X
        else:
            if multiranger.down < height_thresh_rise:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(delay)
                time_start += delay
            if multiranger.down > height_thresh_fall and abs(x - x_b) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(delay)
                time_start += delay
                x_f = x
                L_STATE = L_MIDDLE_X
    elif L_STATE == L_MIDDLE_X:
        diff = (x_b + x_f) / 2 - x
        vx = np.sign(diff) * VELOCITY
        if abs(diff) < 0.02:
            vx = np.sign(diff) * VELOCITY / 2
        if abs(diff) < 0.01:
            motion_commander.start_linear_motion(0, 0, 0)
            time.sleep(delay)
            time_start += delay
            L_STATE = L_RIGHT
            x_found = True

    first = False
    if x_found and y_found:
        print("found box, landing")
        motion_commander.start_linear_motion(0, 0, 0)
        motion_commander.land()

        if goal_pos[0] == -1:
            time.sleep(2.5)
            time_start += 2.5
            vx = 0
            vy = 0
            motion_commander.take_off(height=0.3, velocity=0.2)
            time.sleep(delay)
            time_start += delay
            #      keep_flying = False
            STATE = RETURN
            grid = grow_obstacles(grid)
            path_return = dijkstra(grid, x0, y0, x, y)
            goal_pos = (x, y)
            VELOCITY = 0.2
            x_last = 0
        else:
            print("Start position found!")
            keep_flying = False

    return (x, y, vx, vy)

# Go back to the starting zone
def returning(x, y, path_return):
    vx = 0
    vy = 0
    global keep_flying
    global x_found
    global y_found
    global first
    global STATE

    if multiranger.down <= height_thresh_rise and x<=1.5:
        STATE = LANDING
        return 0, 0, path_return

    if updated_bool:
        path_return = dijkstra(grid, x0, y0, x, y)
    if len(path_return) == 0:
        keep_flying = False
        return 0, 0, path_return
    dist_x = path_return[-1][0] + 0.5 - x * 10
    dist_y = path_return[-1][1] + 0.5 - y * 10
    print("Current postion coords: X=" + str(round(x * 10, 2)) + " Y:" + str(round(y * 10, 2)))
    print(
        "Current objective coords: X=" + str(path_return[-1][0]) + " Y:" + str(path_return[-1][1]))
    if abs(dist_x) < VELOCITY and abs(dist_y) < VELOCITY:
        path_return.pop()
        if len(path_return) == 0:
            STATE = FINDSTART
            return 0, 0, path_return
        dist_x = path_return[-1][0] + 0.5 - x * 10
        dist_y = path_return[-1][1] + 0.5 - y * 10
    if abs(dist_x) >= VELOCITY:
        vx = np.sign(dist_x) * VELOCITY
    if abs(dist_y) >= VELOCITY:
        vy = np.sign(dist_y) * VELOCITY

    return vx, vy, path_return

# Once around the expected position of the goal, look for it
def findstart(x,y):
    global STATE
    global VELOCITY
    global line0
    global line2
    global line3
    global keep_flying
    global x_found
    global y_found
    global first
    global time_start
    vx = 0
    vy = 0

    if multiranger.down > height_thresh_rise:
        distance_y = 1
        distance_x = 0.2
        if y0-y > distance_y or is_close(multiranger.right):
            line0 = True
        if y - y0 > distance_y or is_close(multiranger.left):
            line3 = True

        if not line0:
            vy = -VELOCITY
        elif not line3:
            vy = VELOCITY

        if line3 and line0:
            if not line2:
                if x0-x > distance_x:
                    line2 = True
                    line0 = False
                    line3 = False
                elif is_close(multiranger.back):
                    line2 = True
                else:
                    vx = -VELOCITY
            else:
                if is_close(multiranger.front):
                    dir = np.sign(y-y0)
                    if dir == 1:
                        vy = -VELOCITY
                    else:
                        vy = VELOCITY
                else:
                    if x-x0 < distance_x:
                        vx = VELOCITY
                    else:
                        line0 = False
                        line3 = False
    else:
        motion_commander.start_linear_motion(0, 0, 0)
        time.sleep(1)
        time_start += 1
        STATE = LANDING
        x_found = False
        y_found = False
        first = True
        VELOCITY = 0.1

    return x, y, vx, vy

###############################################################################
############################ Variables ########################################
###############################################################################

# URI to the Crazyflie to connect to
uri = 'radio://0/10/2M/E7E7E7E7E7'

# middle, obstacle, path, starting zone, landing zone, landing pad, visited
cmap = colors.ListedColormap(['white', 'black', 'red', 'blue', 'green', 'yellow', 'orange', 'red'])

img_size = (50, 30)
grid = np.zeros((img_size[0], img_size[1]))
fig, ax = create_empty_plot(img_size[1], img_size[0])

# initial position
x0 = 0.75
y0 = 1.5

# State of the drone
ADVANCE = 1
GOAL = 2
CORNER = 3
LANDING = 4
GOALMISSED = 5
RETURN = 6
FINDSTART = 7

# State when locating the landing pad
L_RIGHT = 1
L_BACK = 2
L_LEFT = 3
L_FRONT = 4
L_MIDDLE_Y = 5
L_MIDDLE_X = 6

direction = 0           # Used to select a direction when encountering an obstacle
checkedright = False
checkedleft = False
VELOCITY = 0.2          # Velocity of the drone
STATE = ADVANCE         # State of the drone in the FSM
L_STATE = L_RIGHT       # Landing state of the drone
line0 = False           # Boolean to know if the right side of the landing zone was search
line3 = False           # Same for the left side
line2 = False
x_last = 0              # x coordinate of the last landing zone line explored

# True until the end of the first passsage in landing state, used to know
# in which direction (x,-x,y,-y) the drone found the landing pad
first = True
# True if the x (y) position of the landing pad center is found
x_found = False
y_found = False

# height threshold for the detection of a 'fall', when passing from the landing pad
# to the ground (set once the height of the flight is defined)
height_thresh_fall = 0
height_thresh_rise = 0

# Set of points not explored in the landing zone due to obstacles
x_obst = []
y_obst = []

# x and y positions of the edge of the landing pad
y_l = 0  # y left
y_r = 0  # y right
x_b = 0  # x back
x_f = 0  # x front
goal_pos = (-1, -1) # position of the landing pad center

updated_bool = False
path_return = []
map_changed = True
time_real = 0.1
time_start = 0
###############################################################################
##################### Beginning of the program ################################
###############################################################################

# A starting position can be provided as input to the script
if len(sys.argv) > 1:
    x0 = float(sys.argv[1])
    if len(sys.argv) > 2:
        y0 = float(sys.argv[2])

tmp_x = x0
tmp_y = y0
tmp_value = 0
path_all = [(floor(x0 * 10), floor(y0 * 10))]
im = None
if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(uri, cf=cf) as scf:
        with MotionCommander(
                scf,
                default_height=0.3) as motion_commander:
            with Multiranger(scf) as multiranger:
                keep_flying = True
                x = x0
                y = y0
                time.sleep(1)
                height = "down"
                height_thresh_fall = motion_commander.default_height + 0.016
                height_thresh_rise = motion_commander.default_height - 0.016
                corr_dist = 32/35

                # Main loop of the controller
                while keep_flying:
                    vx = 0
                    vy = 0

                    if STATE == ADVANCE:
                        if x > 3.5:
                            STATE = GOAL
                            VELOCITY = 0.2
                        x, y, vx, vy = advance(x, y)

                    if STATE == CORNER:
                        x, y, vx, vy = corner(x, y)

                    if STATE == GOAL:
                        print("State: " + str(STATE) + " x: " + str(round(x, 2)) + " y: " + str(
                            round(y, 2)) + " line0: " + str(line0) + " line3: " + str(line3))
                        x, y, vx, vy = goal(x, y)

                    if STATE == GOALMISSED:
                        x, y, vx, vy, path_return = goalmissed(x, y, path_return)

                    if STATE == FINDSTART:
                        x, y, vx, vy = findstart(x,y)

                    if STATE == RETURN:
                        vx, vy, path_return = returning(x, y, path_return)

                    if STATE == LANDING:
                        print("L_State: " + str(L_STATE) + " x: " + str(round(x, 2)) + " y: " + str(
                            round(y, 2)) + " height: " + str(multiranger.down))
                        x, y, vx, vy = landing(x, y, prev_vx, prev_vy)

                    # execute the motion, and save the odometry
                    if keep_flying:
                        motion_commander.start_linear_motion(vx, vy, 0)
                    prev_vx = vx
                    prev_vy = vy

                    time_end=time.time()
                    if(time_start != 0):
                        time_real = time_end - time_start
                        #print("Real time: "+ str(time_real))
                    x += vx * time_real * corr_dist
                    y += vy * time_real * corr_dist
                    time_start=time.time()
                    if abs(floor(x * 10) - path_all[-1][0]) > 0.1 or abs(floor(y * 10) - path_all[-1][1]) > 0.1:
                        path_all.append((floor(x * 10), floor(y * 10)))

                    # stop the program if hand is placed above drone
                    if is_close(multiranger.up):
                        print("Forced landing")
                        keep_flying = False

                    # update the grid with obstacles and visited position
                    grid, updated_bool = update_grid(grid, x, y)
                    if (updated_bool):
                        print("New obstacle found")

                    if (x != tmp_x or y != tmp_y):
                        grid[floor(tmp_x * 10)][floor(tmp_y * 10)] = tmp_value
                        tmp_value = grid[floor(x * 10)][floor(y * 10)]
                        tmp_x = x
                        tmp_y = y
                        grid[floor(x * 10)][floor(y * 10)] = 7
                        map_changed = True

                    if (map_changed):
                        if not im:
                            # for the first frame generate the plot...
                            im = plt.imshow(np.transpose(grid), cmap=cmap, interpolation='none', vmin=0, vmax=2)
                            plt.colorbar(im, orientation='horizontal')
                        else:
                            # ... for subsequent times only update the data
                            im.set_data(np.transpose(grid))

                        plt.draw()
                        map_changed=False
                        plt.pause(0.0001)



                    ### grid[floor(x * 10)][floor(y * 10)] = 2

for x, y in path_all:
    grid[x][y] = 2
grid[floor(10 * goal_pos[0])][floor(10 * goal_pos[1])] = 5
grid = color_zone(grid)
cmap = colors.ListedColormap(['white', 'black', 'red', 'blue', 'green', 'yellow', 'orange'])
ax.imshow(np.transpose(grid), cmap=cmap)
plt.savefig("path.png")
plt.show()
