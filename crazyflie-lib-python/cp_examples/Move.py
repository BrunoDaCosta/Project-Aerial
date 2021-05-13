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

def create_empty_plot(size_x,size_y):
    fig, ax = plt.subplots(figsize=(7,7))
    major_ticks_x = np.arange(0, size_x+1, 5)
    minor_ticks_x = np.arange(0, size_x+1, 1)
    major_ticks_y = np.arange(0, size_y+1, 5)
    minor_ticks_y = np.arange(0, size_y+1, 1)
    ax.set_xticks(major_ticks_y)
    ax.set_xticks(minor_ticks_y, minor=True)
    ax.set_yticks(major_ticks_x)
    ax.set_yticks(minor_ticks_x, minor=True)
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)
    ax.set_ylim([-1,size_x])
    ax.set_xlim([-1,size_y])
    ax.grid(True)
    return fig, ax

def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def update_grid(grid, x, y):
    update_bool = False
    multx = img_size[0]/5
    multy = img_size[1]/3
    dists =[0.1,0.2]
    if (is_close(multiranger.front)):
        for i in dists:    
            if(grid[floor((x+i)*multx)][floor((y)*multy)]!=2):
                grid[floor((x+i)*multx)][floor((y)*multy)]=1
                update_bool = True
    if (is_close(multiranger.back)):
        for i in dists:
            if(grid[floor((x-i)*multx)][floor((y)*multy)]!=2):
                grid[floor((x-i)*multx)][floor((y)*multy)]=1
                update_bool = True
    if (is_close(multiranger.left)):
        for i in dists:
            if(grid[floor((x)*multx)][floor((y+i)*multy)]!=2):
                grid[floor((x)*multx)][floor((y+i)*multy)]=1
                update_bool = True
    if (is_close(multiranger.right)):
        for i in dists:
            if(grid[floor((x)*multx)][floor((y-i)*multy)]!=2):
                grid[floor((x)*multx)][floor((y-i)*multy)]=1
                update_bool = True
    return grid,update_bool

def color_zone(grid):
    for y in range(30):
        for x in range(15):
            if grid[x][y]==0:
                grid[x][y]=3
        for x in range(35,50):
            if grid[x][y]==0:
                grid[x][y]=4
    return grid

def grow_obstacles(grid):
    sx, sy = grid.shape
    sx = range(sx)
    sy = range(sy)
    for x in sx:
        for y in sy:
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    if x+i in sx and y+j in sy:
                        if grid[x+i][y+j] == 1:
                            if grid[x][y] != 1:
                                grid[x][y] = -1;
    grid[grid==-1] = 1
    return grid

def dijkstra(grid, objx, objy, x, y):
    grid_value = np.full((grid.shape[0], grid.shape[1]), 1000)

    print(grid.shape[0])
    grid_value[x][y] = 0
    value = 0
    while (grid_value[objx][objy] >= 1000):
        for i in range(grid.shape[0]):
            if(i<=x+value+1 and i>=x-value-1):
                for j in range(grid.shape[1]):
                    if(j<=y+value+1 and j>=y-value-1):
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

    while (value>0):
        if (x_tmp - 1 >= 0):
            if (grid_value[x_tmp - 1][y_tmp] == value -1):
                x_tmp = x_tmp - 1
                path_list.append([x_tmp,y_tmp])
                value -=1
                continue
        if (x_tmp + 1 < grid.shape[0]):
            if (grid_value[x_tmp + 1][y_tmp] == value -1):
                x_tmp = x_tmp + 1
                path_list.append([x_tmp ,y_tmp])
                value -= 1
                continue
        if (y_tmp - 1 >= 0):
            if (grid_value[x_tmp][y_tmp - 1] == value -1):
                y_tmp = y_tmp - 1
                path_list.append([x_tmp,y_tmp])
                value -= 1
                continue
        if (y_tmp + 1 < grid.shape[1]):
            if (grid_value[x_tmp][y_tmp + 1] == value -1):
                y_tmp = y_tmp + 1
                path_list.append([x_tmp,y_tmp])
                value -= 1
                continue

    return path_list


###############################################################################
####################### Functions for each state ##############################
###############################################################################
                        
def get_off_U(x,y,VELOCITY = 0.2):
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
    return x,y

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
            (x,y) = get_off_U(x,y)
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
            else :
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
            if not(is_close(multiranger.front)):
                STATE = ADVANCE
    else:
        if (is_close(multiranger.right)):
            vx = -VELOCITY
        else:
            motion_commander.start_linear_motion(0, -VELOCITY, 0)
            time.sleep(0.5)
            y -= VELOCITY * 0.5
            if not(is_close(multiranger.front)):
                STATE = ADVANCE
    return x, y, vx, vy

# Search for the landing pad in when in the landing region
# Landing pad is detected when the z-range finder records a decrease
def goal(x, y):
    vx = 0
    vy = 0
    global VELOCITY
    global keep_flying
    global line0
    global line3
    global STATE
    if multiranger.down > height_thresh_rise:
        if y <= 1.5 and not line0:
            vy = -VELOCITY
        elif y > 1.5 and not line3:
            vy = VELOCITY
        if y < 0.15:
            line0 = True
        if is_close(multiranger.right):
            line0 = True
        if y > 2.85 or is_close(multiranger.left):
            line3 = True
        if line3 and line0:
            print("fin de ligne")
            if x > 4.85:
                print("goal not found")
                keep_flying = False
            elif is_close(multiranger.front):
                if y > 1.5:
                    vy = -VELOCITY
                else:
                    vy = VELOCITY
            else:
                motion_commander.start_linear_motion(VELOCITY, 0, 0)
                time.sleep(1)
                grid[floor(x*10)+1][floor(y*10)] = 2
                x += VELOCITY * 1
                line0 = False
                line3 = False
        elif line0 and not is_close(multiranger.left):
            vy = VELOCITY
        elif line3 and not is_close(multiranger.right):
            vy = -VELOCITY
    else:
        motion_commander.start_linear_motion(0, 0, 0)
        time.sleep(1)
        STATE = LANDING
        VELOCITY = VELOCITY / 2
    return x, y, vx, vy

# Detect the edge of the landing pad in order to land in its center
def landing(x, y, prev_vx, prev_vy):
    vx = 0
    vy = 0
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

    if prev_vy != 0 and first:
        if prev_vy > 0:
            y_r = y
            L_STATE = L_LEFT
        else:
            y_l = y
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
            if multiranger.down>height_thresh_fall and abs(y-y_r) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(1)
                y_l = y
                L_STATE = L_MIDDLE_Y
        else:
            if multiranger.down>height_thresh_fall and abs(y-y_r) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(1)
                y_l = y
                L_STATE = L_MIDDLE_Y
    if L_STATE == L_RIGHT:
        global path_return
        vy = -VELOCITY
        if not x_found:
            if multiranger.down>height_thresh_fall and abs(y-y_l) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(1)
                y_r = y
                L_STATE = L_MIDDLE_Y
        else:
            if multiranger.down < height_thresh_rise:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(1)
            if multiranger.down>height_thresh_fall:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(1)
                y_r = y
                L_STATE = L_LEFT
    if L_STATE == L_MIDDLE_Y:
        diff = (y_l+y_r)/2-y
##      print(str(round(y_l,2)) + " " + str(round(y_r,2)) + " " + str(round(y,2)))
        vy = np.sign(diff)*VELOCITY
        if abs(diff) < 0.02:
            vy = np.sign(diff)*VELOCITY/2
        if abs(diff) < 0.01:
            time.sleep(1)
            L_STATE = L_BACK
            y_found = True
    if L_STATE == L_BACK:
        vx = -VELOCITY
        if not y_found:
            if multiranger.down>height_thresh_fall and abs(x-x_f) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(1)
                x_b = x
                L_STATE = L_MIDDLE_X
        else:
            if multiranger.down>height_thresh_fall:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(1)
                x_b = x
                L_STATE = L_FRONT
    if L_STATE == L_FRONT:
        vx = VELOCITY
        if not y_found:
            if multiranger.down>height_thresh_fall and abs(x-x_b) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(1)
                x_f = x
                L_STATE = L_MIDDLE_X
        else:
            if multiranger.down < height_thresh_rise:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(1)
            if multiranger.down>height_thresh_fall and abs(x-x_b) > 0.2:
                motion_commander.start_linear_motion(0, 0, 0)
                time.sleep(1)
                x_f = x
                L_STATE = L_MIDDLE_X
    if L_STATE == L_MIDDLE_X:
        diff = (x_b+x_f)/2-x
##      print(str(round(x_b,2)) + " " + str(round(x_f,2)) + " " + str(round(x,2)))
        vx = np.sign(diff)*VELOCITY
        if abs(diff) < 0.02:
            vx = np.sign(diff)*VELOCITY/2
        if abs(diff) < 0.01:
            time.sleep(1)
            L_STATE = L_RIGHT
            x_found = True

    first = False
    if x_found and y_found:
        print("found goal, landing")
        motion_commander.start_linear_motion(0, 0, 0)
        motion_commander.land()
        time.sleep(2.5)
        motion_commander.take_off()
##      keep_flying = False
        STATE = RETURN
        grid = grow_obstacles(grid)
        path_return = dijkstra(grid,x0,y0,x,y)
        goal_pos = (x,y)
        VELOCITY = 0.2

    return (x, y, vx, vy)

###############################################################################
############################ Variables ########################################
###############################################################################

# URI to the Crazyflie to connect to
uri = 'radio://0/10/2M/E7E7E7E7E7'

cmap = colors.ListedColormap(['white', 'black', 'red','blue','green', 'yellow'])

img_size = (50, 30)
grid = np.zeros((img_size[0],img_size[1]))
fig, ax = create_empty_plot(img_size[1], img_size[0])

# initial position
x0 = 0
y0 = 0

# State of the drone
ADVANCE = 1
GOAL = 2
CORNER = 3
LANDING = 4
RETURN = 5
# State when locating the landing pad
L_RIGHT = 1
L_BACK = 2
L_LEFT = 3
L_FRONT = 4
L_MIDDLE_Y = 5
L_MIDDLE_X = 6

# Used to select a direction when encountering an obstacle
direction = 0
checkedright=False
checkedleft=False
VELOCITY = 0.2
STATE = ADVANCE
L_STATE = L_RIGHT
line0 = False
line3 = False

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
y_l = 0 # y left
y_r = 0 # y right
x_b = 0 # x back
x_f = 0 # x front
# position of the landing pad center
goal_pos = (-1,-1)
updated_bool=False
path_return=[]

###############################################################################
##################### Beginning of the program ################################
###############################################################################

# A starting position can be provided as input to the script
if len(sys.argv) >1:
    x0 = float(sys.argv[1])
    if len(sys.argv) >2:
        y0 = float(sys.argv[2])


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(uri, cf=cf) as scf:
        with MotionCommander(
                scf,
                default_height = 0.3) as motion_commander:
            with Multiranger(scf) as multiranger:
                keep_flying = True
                x = x0
                y = y0
                time.sleep(1)
                y_not_expl = []
                height = "down"
                height_thresh_fall = motion_commander.default_height + 0.03
                height_thresh_rise = motion_commander.default_height - 0.03

                # Main loop of the controller
                while keep_flying:
                    
                    vx = 0
                    vy = 0
                        
                    if STATE == ADVANCE:
                        # ----- TEMPORAIRE -----
                        if x > 0.5:
                            STATE = GOAL
                            x=3.5
                        # ----------------------
                        x, y, vx, vy = advance(x, y)

                    elif STATE == CORNER:
                        x, y, vx, vy = corner(x, y)

                    elif STATE == GOAL:
                        print("State: " + str(STATE) + " x: " + str(round(x,2)) + " y: "+ str(round(y,2)) + " line0: "+ str(line0) +" line3: " + str(line3))
                        x, y, vx, vy = goal(x, y)

                    if STATE == LANDING:
                        print("L_State: " + str(L_STATE) + " x: " + str(round(x,2)) + " y: "+ str(round(y,2)) + " height: "+ str(multiranger.down))
                        x, y, vx, vy = landing(x, y, prev_vx, prev_vy)

                    if STATE == RETURN:
                        x -= 3
                        if updated_bool:
                            path_return = dijkstra(grid, x0, y0, x, y)
                        if len(path_return)==0:
                            keep_flying=False
                            continue
                        if (abs(x-path_return[-1][0]))<0.05 and (abs(y-path_return[-1][1]))<0.05 :
                            path_return.pop()
                        vx=(path_return[-1][0]-x)
                        vy=(path_return[-1][1]-y)

                        """
                        diff_x = x0 - x
                        diff_y = y0 - y
                        print("x: " + str(round(x,2)) + " y: " + str(round(y,2)))
                        vx = np.sign(diff_x)*VELOCITY
                        vy = np.sign(diff_y)*VELOCITY
                        if abs(diff_x) < 0.1 and abs(diff_y) < 0.1:
                            keep_flying = False
                        """

                    # execute the motion, and save the odometry
                    motion_commander.start_linear_motion(vx, vy, 0)
                    prev_vx = vx
                    prev_vy = vy
                    x += vx * 0.1
                    y += vy * 0.1

                    # stop the program if hand is placed above drone
                    if is_close(multiranger.up):
                        print("landing")
                        keep_flying = False
                    time.sleep(0.1)
##                    print("{}  {} {}".format(round(x,2),round(y,2), multiranger.down))
                    # update the grid with obstacles and visited position
                    grid,updated_bool = update_grid(grid, x, y)
                    grid[floor(x*10)][floor(y*10)] = 2


grid[floor(10*goal_pos[0])][floor(10*goal_pos[1])] = 6
grid = color_zone(grid)
ax.imshow(np.transpose(grid), cmap=cmap)
plt.show()
