from scipy import *
from math import *
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib import colors
import cv2


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


cmap = colors.ListedColormap(['white', 'black', 'red', 'blue', 'green'])

img_size = (50, 30)
grid = np.zeros((img_size[0], img_size[1]))
grid[25][3:20]=2
grid[15][15]=2
grid[16][15]=2
grid[17][15]=2
grid[18][15]=2
grid[19][15]=2
grid[20][15]=2
grid[21][15]=2
grid[22][15]=2
grid[23][15]=2
grid[24][15]=2
fig, ax = create_empty_plot(img_size[1], img_size[0])
# ax.scatter(0.5,1, marker="o", color = 'orange')
# ax.imshow(grid, cmap=cmap)

import time
import sys


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
                    if x+i in sx and y+j in sy:
                        if grid[x+i][y+j] == 2:
                            if grid[x][y] != 2:
                                grid[x][y] = -1;
    grid[grid==-1] = 2
    return grid

def dijkstra(grid, objx, objy, x, y):


    grid_value = np.full((grid.shape[0], grid.shape[1]), 1000)
    #grid_value = grow_obstacles(grid)

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
                                if (grid[i - 1][j] != 2 and grid_value[i - 1][j] > value + 1):
                                    grid_value[i - 1][j] = value + 1
                            if (i + 1 < grid.shape[0]):
                                if (grid[i + 1][j] != 2 and grid_value[i + 1][j] > value + 1):
                                    grid_value[i + 1][j] = value + 1
                            if (j - 1 >= 0):
                                if (grid[i][j - 1] != 2 and grid_value[i][j - 1] > value + 1):
                                    grid_value[i][j - 1] = value + 1
                            if (j + 1 < grid.shape[1]):
                                if (grid[i][j + 1] != 2 and grid_value[i][j + 1] > value + 1):
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

time1 = time.time()
grid = grow_obstacles(grid)
path_list = dijkstra(grid, 5, 5, 40, 20)
time2 = time.time()
print("Time in ms: " + str(1000*(time2-time1)))
print(path_list)
for [x,y] in path_list:
    grid[x][y]=1
grid = color_zone(grid)
ax.imshow(np.transpose(grid), cmap=cmap)
plt.show()
