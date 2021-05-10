from scipy import *
from math import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import cv2

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

cmap = colors.ListedColormap(['white', 'black', 'red','blue','green'])

img_size = (50, 30)
grid = np.zeros((img_size[0],img_size[1]))

fig, ax = create_empty_plot(img_size[1], img_size[0])
#ax.scatter(0.5,1, marker="o", color = 'orange')
ax.imshow(grid, cmap = cmap)



# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
This script shows the basic use of the PositionHlCommander class.
Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system.
The PositionHlCommander uses position setpoints.
Change the URI variable to your Crazyflie configuration.
"""
import cflib.crtp
import time
import sys
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.positioning.motion_commander import MotionCommander

# URI to the Crazyflie to connect to
uri = 'radio://0/10/2M/E7E7E7E7E7'
x0 = 0
y0 = 0
ADVANCE = 1
GOAL = 2
CORNER = 3
LANDING = 4

L_RIGHT = 1
L_BACK = 2
L_LEFT = 3
L_FRONT = 4
L_MIDDLE_Y = 5
L_MIDDLE_X = 6

if len(sys.argv) >1:
    x0 = float(sys.argv[1])
    if len(sys.argv) >2:
        y0 = float(sys.argv[2])

def update_grid(grid, x, y):
    multx = img_size[0]/5
    multy = img_size[1]/3
    dists =[0.1,0.2]
    if (is_close(multiranger.front)):
        for i in dists:    
            if(grid[floor((x+i)*multx)][floor((y)*multy)]!=2):
                grid[floor((x+i)*multx)][floor((y)*multy)]=1
    if (is_close(multiranger.back)):
        for i in dists:
            if(grid[floor((x-i)*multx)][floor((y)*multy)]!=2):
                grid[floor((x-i)*multx)][floor((y)*multy)]=1
    if (is_close(multiranger.left)):
        for i in dists:
            if(grid[floor((x)*multx)][floor((y+i)*multy)]!=2):
                grid[floor((x)*multx)][floor((y+i)*multy)]=1
    if (is_close(multiranger.right)):
        for i in dists:
            if(grid[floor((x)*multx)][floor((y-i)*multy)]!=2):
                grid[floor((x)*multx)][floor((y-i)*multy)]=1
    return grid

def color_zone(grid):
    for y in range(30):
        for x in range(15):
            if grid[x][y]==0:
                grid[x][y]=3
        for x in range(35,50):
            if grid[x][y]==0:
                grid[x][y]=4
    return grid



def get_off_U(x,y,VELOCITY = 0.2):
    while is_close(multiranger.right) and is_close(multiranger.left):
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


def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(uri, cf=cf) as scf:
        with MotionCommander(
                scf,
                default_height = 0.2) as motion_commander:
            with Multiranger(scf) as multiranger:
                VELOCITY = 0.2
                keep_flying = True
                x = x0
                y = y0
                STATE = ADVANCE
                L_STATE = L_RIGHT
                time.sleep(1)
                direction = 0
                checkedright=False
                checkedleft=False
                line0 = False
                line3 = True
                height = "down"
                coords_border = [0, 0, 0, 0]
                counter_height = 0
                onthebox = 0
                
                while keep_flying:
                    vx = 0
                    vy = 0
                    # Main loop of the controller
                        
                    if STATE == ADVANCE:
                        if x > 1:
                            STATE = GOAL
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
                    elif STATE == CORNER:
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
                    elif STATE == GOAL:
                        x=3.5
                        if multiranger.down>0.15:
                            if x < 3.7 and y <= 1.5 and not line0:
                                vy = -VELOCITY
                            elif x < 3.7 and y > 1.5 and not line3:
                                vy = VELOCITY
                            if y < 0.2 or is_close(multiranger.right):
                                line0 = True
                            if y > 2.8 or is_close(multiranger.left):
                                line3 = True
                            if line3 and line0:
                                vx = VELOCITY
                                line0 = False
                                line3 = False
                            elif line0 and not is_close(multiranger.left):
                                vy = VELOCITY
                            elif line3 and not is_close(multiranger.right):
                                vy = -VELOCITY
##                            if is_close(multiranger.down):
##                                 keep_flying = False
                        else:
                            print("entering landing mode!")
                            STATE = LANDING
                            height = "up"
                            coords_border = [0, 0, 0, 0]
                            goal_pos = [x,y]
                            L_STATE = L_RIGHT
                            counter_height = 0
                            VELOCITY = VELOCITY / 2
                            onthebox = 1
                            time.sleep(1)
                    
                    elif STATE == LANDING:
                        if height == "up" and multiranger.down > 0.24 and counter_height >= 10:
                            height = "down"
                            counter_height = 0
                            time.sleep(1)
                            
                        elif height == "down" and multiranger.down < 0.15 and counter_height >= 10:
                            height = "up"
                            counter_height = 0
                            time.sleep(1)
                            
##                        print(str(L_STATE) + " " + height +" " + str(multiranger.down))

                        print(str(L_STATE) + "  " + height + " " + str(multiranger.down) + " " + str(counter_height))
                        if multiranger.down >= 0.18 and multiranger.down <= 0.22:
                            
                            
                            if L_STATE == L_RIGHT:
                                vy = -VELOCITY
                                if height == "down":
                                    coords_border[0] = y
                                    L_STATE = L_LEFT
                                    onthebox = 0
                            if L_STATE == L_LEFT:
                                vy = VELOCITY
                                if height == "up":
                                    onthebox = 1
                                elif height == "down" and onthebox == 1:
                                    coords_border[1] = y
                                    L_STATE = L_MIDDLE_Y
                            if L_STATE == L_MIDDLE_Y:
                                diff = np.mean(coords_border[0:2])-y
                                vy = np.sign(diff)*VELOCITY
                                if abs(diff) < 0.05:
                                    time.sleep(1)
                                    L_STATE = L_BACK

                            if L_STATE == L_BACK:
                                vx = -VELOCITY
                                if height == "down":
                                    coords_border[2] = x
                                    L_STATE = L_FRONT
                                    onthebox = 0
                            if L_STATE == L_FRONT:
                                vx = VELOCITY
                                if height == "up":
                                    onthebox = 1
                                elif height == "down" and onthebox == 1:
                                    coords_border[3] = x
                                    L_STATE = L_MIDDLE_X
                            if L_STATE == L_MIDDLE_X:
                                diff = np.mean(coords_border[2:4])-x
                                vx = np.sign(diff)*VELOCITY
                                if abs(diff) < 0.05:
                                    time.sleep(1)
                                    keep_flying = False

                            
                        counter_height += 1

                    
                    motion_commander.start_linear_motion(vx, vy, 0)
                    x += vx * 0.1
                    y += vy * 0.1
                    if is_close(multiranger.up):
                        print("landing")
                        keep_flying = False
                    time.sleep(0.1)
                    #print("{}  {} {}".format(round(x,2),round(y,2), multiranger.down))
                    grid = update_grid(grid, x, y)
                    grid[floor(x*10)][floor(y*10)] = 2

            
grid = color_zone(grid)
ax.imshow(np.transpose(grid), cmap=cmap)
plt.show()
