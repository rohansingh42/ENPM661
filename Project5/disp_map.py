import os, sys

# This try-catch is a workaround for Python3 when used with ROS; it is not needed for most platforms
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import time
import heapq
import cPickle
from matplotlib import cm


try:
    with open('RSRmap_fast.pkl', 'rb') as file:
        MAP = cPickle.load( file )
except IOError:
    file = open( 'RSRmap_fast.pkl', "w" )
    cPickle.dump( MAP, file )
file.close()
print(MAP[999][9])
MAP = np.array(MAP)
print(np.where(MAP==0))
MAP[1000,10] = 3
MAP[1000,11] = 4
MAP[1000,12] = 5
fig, ax = plt.subplots(1, 1, tight_layout=True)
cmap = matplotlib.colors.ListedColormap(['w', 'brown', 'orange', 'b', 'y', 'm', 'r', 'k'])
# cmap = cm.get_cmap('rainbow', 8)
resolution = 10        # Resolution for map defined in mm
mod_dim = np.int64(np.array((11000,10100))/resolution)

obs = ax.pcolormesh(MAP, cmap=cmap)#, edgecolors='k', linewidths=self.resolution/280)
plt.xlim(0,mod_dim[0])
plt.ylim(0,mod_dim[1])
plt.show(True)