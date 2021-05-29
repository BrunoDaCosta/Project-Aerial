
from math import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import time


cmap = colors.ListedColormap(['white', 'black', 'red', 'blue', 'green', 'yellow', 'orange'])

img_size = (50, 30)
grid = np.zeros((img_size[0], img_size[1]))

im = None
for i in range(49):    # draw 20 frames
    grid[i][5] = 2
    if not im:
        # for the first frame generate the plot...
        im = plt.imshow(np.transpose(grid), cmap = cmap, interpolation='none',vmin=0,vmax=2)    
        plt.colorbar(im, orientation='horizontal')
    else:
        # ... for subsequent times only update the data
        im.set_data(np.transpose(grid))
    plt.draw()
    plt.pause(0.1)
