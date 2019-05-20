import argparse
import os, sys
import pickle
import numpy as np

# This try-catch is a workaround for Python3 when used with ROS; it is not needed for most platforms
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass

import cv2
import imutils

import matplotlib.pyplot as plt

O1 = np.array([[50,112.5],[100,112.5],[100,67.5],[50,67.5]])  #rectangle
O2 = np.array([140,120]) # circle
O2_p = np.array([15,6])  
O3 = np.array([190,130])  # ellipse
O3_p = np.array([15])
O4 = np.array([[125,56],[163,52],[170,90],[193,52],[173,15],[150,15])   #polygon

resolution = 1

map_height = np.round(150/resolution)
map_width = np.round(250/resolution)
map = np.ones([map_height,map_width])





