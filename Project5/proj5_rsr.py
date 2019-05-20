import argparse
import os, sys

# This try-catch is a workaround for Python3 when used with ROS; it is not needed for most platforms
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass

import math
import numpy as np
# import cv2
import matplotlib.pyplot as plt
import matplotlib
import time
import heapq
import pickle
import cPickle
# from maps import Cell
# # import ros stuff
# import rospy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist, Point
# from nav_msgs.msg import Odometry
# from tf import transformations

class Cell:

    def __init__( self, ul ):

        self.ul = ul    # Upper left point
        self.ur = None  # Upper right point
        self.bl = None  # Bottom left point
        self.br = None  # Bottom right point
        self.size = 0

class node:
    # def __init__(self, x, y, theta, parentid, stepid, g, d = 0):
    def __init__(self, x, y, parentid, stepid, g, d = 0):
        self.x = np.int32(np.round(x, 0))
        self.y = np.int32(np.round(y, 0))
        # self.theta = np.int32(np.round(theta, 0))
        self.id = 'x' + str(self.x) + 'y' + str(self.y)# + str(self.theta)
        self.parentid = parentid
        self.stepid = stepid
        self.g = np.round(g, 2)
        self.d = np.round(d, 2)
        self.h = g + d

    def __str__(self):
        return "x : " + str(self.x) + " y : " + str(self.y) + " theta : " + str(0) + " id : " + str(self.id) + ", parentid : " + str(self.parentid) + ", stepid : " + str(self.stepid) + ", h : " + str(self.h) + ", g : " + str(self.g) + ", d : " + str(self.d) 

    def __lt__(self,other):
        return self.h < other.h

    def __eq__(self,other):
        return self.h == other.h

    def __ne__(self,other):
        return self.h != other.h

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item):
        heapq.heappush(self.elements, item)
    
    def get(self):
        return heapq.heappop(self.elements)

class PathPlan:

    fig, ax = plt.subplots(1, 1, tight_layout=True)
    # orig_dim = np.int64((250,150))
    cmap = matplotlib.colors.ListedColormap(['w', 'b', 'orange', 'g', 'y', 'm', 'r', 'k'])
    canvas = ax.figure.canvas
    # mod_dim = np.int64([10100,11000])
    resolution = 10        # Resolution for map defined in mm
    mod_dim = np.int64(np.array((11000,10100))/resolution)
    RPM1 = 50             # LOW RPM (rev/min)
    RPM2 = 100             # HIGH RPM (rev/min)
    wr = 38               # Wheel Radius (cm)
    wb = 230              # Wheel Base (cm)
    f = 1               # Sampling Frequency (1/sec)
    rr = 1             # Robot Radius (cm)
    c = 1                # Clearance (cm)
    gt = 20             # Radius of circle that is considered goal position (cm)
    motion_model = [[1,0,'e'],                  #Move Right
                    # [1,1],                  #Move Right Up
                    [0,1,'s'],                  #Move Up
                    # [-1,1],                 #Move Left Up
                    [-1,0,'w'],                 #Move Left
                    # [-1,-1],                #Move Left Down
                    [0,-1,'n']]                 #Move Down
                    # [1,-1]                  #Move Left Down

    try:
        with open('RSRmap_fast.pkl', 'rb') as file:
            map_grid = cPickle.load( file )
    except IOError:
        file = open( 'RSRmap_fast.pkl', "w" )
        cPickle.dump( map_grid, file )

    try:
        with open('RSRedges_fast.pkl', 'rb') as file:
            edges = cPickle.load( file )
    except IOError:
        file = open( 'RSRedges_fast.pkl', "w" )
        cPickle.dump( edges, file )
    
    try:
        with open('RSRcells_fast.pkl', 'rb') as file:
            cells = cPickle.load( file )
    except IOError:
        file = open( 'RSRcells_fast.pkl', "w" )
        cPickle.dump( cells, file )


    def __init__(self, start, goal): #, resolution = 1, robot_radius = 0, clearance = 0, algo = 'astar'):

        # self.robot_radius = robot_radius
        # self.clearance = clearance
        # self.resolution = resolution
        # self.mod_dim = np.int64(self.orig_dim/resolution)
        # self.mod_rad = (robot_radius + clearance)/resolution
        self.start = start
        self.goal = goal
        # self.map_grid = np.zeros((self.mod_dim[1],self.mod_dim[0]))
        # self.algo = algo
        # grid = draw_map(robot_radius,resolution,clearance)
        # grid[self.mod_dim[1] - start.y, start.x] = 1
        # grid[self.mod_dim[1] - goal.y, goal.x] = 2
        # self.map_grid = grid


    def nextNode(self,cnode,stepid):

        step = self.motion_model[stepid]
        
        # dx = step[0]*self.resolution
        # dy = step[1]*self.resolution
        # xnew = cnode.x + dx
        # ynew = cnode.y + dy

        # xnew_idx = np.int64((xnew/self.resolution) + self.mod_dim[0]/2)
        # ynew_idx = np.int64((ynew/self.resolution) + self.mod_dim[1]/2)

        x_idx = np.int64((cnode.x/self.resolution) + self.mod_dim[0]/2)
        y_idx = np.int64((cnode.y/self.resolution) + self.mod_dim[1]/2)

        if ((y_idx,x_idx),step[2]) in self.edges:

            print('###############################################################')
            new = self.edges[((y_idx,x_idx),step[2])]
            xnew_idx = new[1]
            ynew_idx = new[0]
            xnew = np.int64((xnew_idx - self.mod_dim[0]/2)*self.resolution)
            ynew = np.int64((ynew_idx - self.mod_dim[1]/2)*self.resolution)

        else:
            dx = step[0]*self.resolution
            dy = step[1]*self.resolution
            xnew = cnode.x + dx
            ynew = cnode.y + dy


        g = cnode.g + np.abs(xnew - cnode.x) + np.abs(ynew - cnode.y)
        
        d = math.sqrt((self.goal.x - xnew)**2 + (self.goal.y - ynew)**2)
        # d = np.abs(self.goal.x - xnew) + np.abs(self.goal.y - ynew)

        # print('cnode.x',cnode.x)
        # print('cnode.y',cnode.y)
        # print('x',x)
        # print('y',y)

        n = node(xnew,ynew,cnode.id,stepid,g,d)

        return n

    def addStartGoalEdges(self):

        startx_idx = np.int64((self.start.x/self.resolution) + self.mod_dim[0]/2)
        starty_idx = np.int64((self.start.y/self.resolution) + self.mod_dim[1]/2)
        goalx_idx = np.int64((self.goal.x/self.resolution) + self.mod_dim[0]/2)
        goaly_idx = np.int64((self.goal.y/self.resolution) + self.mod_dim[1]/2)
        gcellul = (-1,-1)
        for cell in self.cells:

            if (goalx_idx < cell.ur[1]) and (goalx_idx > cell.ul[1]) and (goaly_idx > cell.ul[0]) and (goaly_idx < cell.bl[0]):
                self.edges[((cell.ul[0],goalx_idx),'s')] = (goaly_idx,goalx_idx)
                self.edges[((goaly_idx,goalx_idx),'n')] = (cell.ul[0],goalx_idx)

                self.edges[((cell.br[0],goalx_idx),'n')] = (goaly_idx,goalx_idx)
                self.edges[((goaly_idx,goalx_idx),'s')] = (cell.br[0],goalx_idx)

                self.edges[((goaly_idx,cell.br[1]),'w')] = (goaly_idx,goalx_idx)
                self.edges[((goaly_idx,goalx_idx),'e')] = (goaly_idx,cell.br[1])

                self.edges[((goaly_idx,cell.ul[1]),'e')] = (goaly_idx,goalx_idx)
                self.edges[((goaly_idx,goalx_idx),'w')] = (goaly_idx,cell.ul[1])

                print('##################################### GOAL HERE #########################')
                gcellul = cell.ul

        for cell in self.cells:

            if (startx_idx < cell.ur[1]) and (startx_idx > cell.ul[1]) and (starty_idx > cell.ul[0]) and (starty_idx < cell.bl[0]):
                if gcellul == cell.ul:
                    break
                else:
                    self.edges[((cell.ul[0],startx_idx),'s')] = (starty_idx,startx_idx)
                    self.edges[((starty_idx,startx_idx),'n')] = (cell.ul[0],startx_idx)

                    self.edges[((cell.br[0],startx_idx),'n')] = (starty_idx,startx_idx)
                    self.edges[((starty_idx,startx_idx),'s')] = (cell.br[0],startx_idx)

                    self.edges[((starty_idx,cell.br[1]),'w')] = (starty_idx,startx_idx)
                    self.edges[((starty_idx,startx_idx),'e')] = (starty_idx,cell.br[1])

                    self.edges[((starty_idx,cell.ul[1]),'e')] = (starty_idx,startx_idx)
                    self.edges[((starty_idx,startx_idx),'w')] = (starty_idx,cell.ul[1])

                    print( "size: " + str(cell.size) )
                    print( "\tul: " + str(cell.ul) )
                    print( "\tbr: " + str(cell.br) )

                    print('#####################################START HERE#########################')


    def algorithm(self):

        self.addStartGoalEdges()
        motion = self.motion_model
        opened = dict()
        closed = dict()
        pq = PriorityQueue()
        opened[self.start.id] = self.start
        pq.put(self.start)
        if self.checkCollision(self.start):
            print("s")
        if self.checkCollision(self.goal):
            print("s")
        
        obs = self.ax.pcolormesh(self.map_grid, cmap=self.cmap)#, edgecolors='k', linewidths=self.resolution/280)
        # plt.xlim(-self.mod_dim[0]/2,self.mod_dim[0]/2)
        # plt.ylim(-self.mod_dim[1]/2,self.mod_dim[1]/2)
        plt.xlim(0,self.mod_dim[0])
        plt.ylim(0,self.mod_dim[1])
        # plt.gca().set_aspect('equal', adjustable='box')
        
        # mng = plt.get_current_fig_manager()
        # mng.full_screen_toggle()

        plt.show(False)
        self.ax.hold(True)
        self.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        flag = True
        show_animation = False

        k = 100#np.int64(np.divide((self.mod_dim[0]*self.mod_dim[1]),200))
        j = 0
        gf = True
        while flag == True:
            
            if pq.empty():
                print("Goal can't be reached.")
                # curr_map = self.ax.pcolormesh(np.flip(self.map_grid,0), cmap=self.cmap, edgecolors='k', linewidths=self.resolution/280)
                # self.ax.draw_artist(curr_map)
                # self.fig.canvas.blit(self.ax.bbox)
                # plt.pause(3)
                # plt.close()
                break
            curr_node = pq.get()
            # print(curr_node)
            cid = curr_node.id
            
            if self.checkGoal(curr_node) :
                print("Goal Found")
                self.goal.parentid = curr_node.parentid
                self.goal.x = curr_node.x
                self.goal.y = curr_node.y
                # self.goal.theta = curr_node.theta
                self.goal.g = curr_node.g
                self.goal.d = curr_node.d
                self.goal.h = curr_node.h
                self.goal.id = curr_node.id
                self.goal.stepid = curr_node.stepid
                closed[curr_node.id] = curr_node
                flag = False
                gf = True
                print('GOAL - ',self.goal)
                break
            
            if curr_node.id in closed:
                continue
            else:
                opened.pop(curr_node.id)
                closed[curr_node.id] = curr_node
                if (curr_node.id != self.start.id) and (curr_node.id != self.goal.id):
                    self.map_grid[np.int64(self.mod_dim[1]/2 + curr_node.y/self.resolution), np.int64(self.mod_dim[0]/2 + curr_node.x/self.resolution)] = 3

                for i,_ in enumerate(motion):

                    # child_id = len(opened) + len(closed) + 1

                    child_node = self.nextNode(curr_node,i)
                    
                    if self.checkCollision(child_node):
                        continue
                    elif child_node.id in closed:
                        continue
                    elif child_node.id in opened:
                        if child_node.g < opened[child_node.id].g:
                            opened[child_node.id].g = child_node.g
                            opened[child_node.id].parentid = child_node.parentid
                            pq.put(child_node)
                        else:
                            continue
                    else:
                        opened[child_node.id] = child_node
                        pq.put(child_node)
                        print(child_node)
                    
                    if child_node.id != self.goal.id:
                        self.map_grid[np.int64(self.mod_dim[1]/2 + child_node.y/self.resolution), np.int64(self.mod_dim[0]/2 + child_node.x/self.resolution)] = 4
                
                if (j%k == 0 ) and (show_animation == True):
                        
                    curr_map = self.ax.pcolormesh(self.map_grid, cmap=self.cmap)#, edgecolors='k', linewidths=self.resolution/280)
                    self.ax.draw_artist(curr_map)
                    self.fig.canvas.blit(self.ax.bbox)

                j = j+1
        self.ax.hold(True)
        if gf:
            path = self.gen_path(closed)
        # print(path)

        return path
    

    def gen_path(self,closed):

        cid = self.goal.id
        path = []
        # fig, ax = plt.subplots(1, 1, tight_layout=True)
        xdata = []
        ydata = []
        # plt.axis([-5550,5550,-5050,5050])
        while cid != self.start.parentid:
            path.append(closed[cid])
            # print(closed[cid])
            # xdata.append(closed[cid].x)
            # ydata.append(closed[cid].y)
            cid = closed[cid].parentid
            # plt.plot(xdata,ydata,'go--', linewidth=2, markersize=3)
            # plt.pause(0.1)

        path.reverse()

        i = 0
        while True:
            if i < (len(path) - 1):
                curr_node = path[i]
                next_node = path[i+1]
                stepid = next_node.stepid
                step = self.motion_model[stepid]
                n = curr_node
                while n.id != next_node.id:
                    self.map_grid[np.int64(self.mod_dim[1]/2 + n.y/self.resolution), np.int64(self.mod_dim[0]/2 + n.x/self.resolution)] = 1
                    self.map_grid[np.int64(self.mod_dim[1]/2 + n.y/self.resolution)+1, np.int64(self.mod_dim[0]/2 + n.x/self.resolution)] = 1
                    self.map_grid[np.int64(self.mod_dim[1]/2 + n.y/self.resolution)-1, np.int64(self.mod_dim[0]/2 + n.x/self.resolution)] = 1
                    self.map_grid[np.int64(self.mod_dim[1]/2 + n.y/self.resolution)+2, np.int64(self.mod_dim[0]/2 + n.x/self.resolution)] = 1
                    self.map_grid[np.int64(self.mod_dim[1]/2 + n.y/self.resolution)-2, np.int64(self.mod_dim[0]/2 + n.x/self.resolution)] = 1
                    self.map_grid[np.int64(self.mod_dim[1]/2 + n.y/self.resolution), np.int64(self.mod_dim[0]/2 + n.x/self.resolution)+1] = 1
                    self.map_grid[np.int64(self.mod_dim[1]/2 + n.y/self.resolution), np.int64(self.mod_dim[0]/2 + n.x/self.resolution)-1] = 1
                    self.map_grid[np.int64(self.mod_dim[1]/2 + n.y/self.resolution), np.int64(self.mod_dim[0]/2 + n.x/self.resolution)+2] = 1
                    self.map_grid[np.int64(self.mod_dim[1]/2 + n.y/self.resolution), np.int64(self.mod_dim[0]/2 + n.x/self.resolution)-2] = 1
                
                    print(n)
                    dx = step[0]*self.resolution
                    dy = step[1]*self.resolution
                    xnew = n.x + dx
                    ynew = n.y + dy
                    g = n.g + np.abs(xnew - n.x) + np.abs(ynew - n.y)
                    d = 10*math.sqrt((self.goal.x - xnew)**2 + (self.goal.y - ynew)**2)
                    n = node(xnew,ynew,n.id,stepid,g,d)
                i = i + 1
            else:
                break

        curr_map = self.ax.pcolormesh(self.map_grid, cmap=self.cmap)#, edgecolors='k', linewidths=self.resolution/280)
        self.ax.draw_artist(curr_map)
        self.fig.canvas.blit(self.ax.bbox)
        self.ax.hold(False)
        print("No. of nodes in path : ", len(path))
        plt.show()

        return path

        
        # path.reverse()
        # i = 0
        # for n in path:
        #     self.map_grid[self.mod_dim[1] - n.y, n.x] = 5
        #     if i%5 == 0:
        #         curr_map = self.ax.pcolormesh(np.flip(self.map_grid,0), cmap=self.cmap, edgecolors='k', linewidths=self.resolution/280)
        #         self.ax.draw_artist(curr_map)
        #         self.fig.canvas.blit(self.ax.bbox)
        #     i = i+1
        # curr_map = self.ax.pcolormesh(np.flip(self.map_grid,0), cmap=self.cmap, edgecolors='k', linewidths=self.resolution/280)
        # self.ax.draw_artist(curr_map)
        # self.fig.canvas.blit(self.ax.bbox)
        # plt.pause(3)
        # plt.close()

    def checkGoal(self,cnode):
        goalReached = math.sqrt((cnode.x - self.goal.x)**2 + (cnode.y - self.goal.y)**2) - self.gt < 0
        return goalReached

    def checkCollision(self,cnode):
        r = self.rr + self.c
        x = cnode.x
        y = cnode.y

        c1 = (x + 1650)**2 + (y - 4600)**2 - (405 + r)**2 < 0
        c2 = (x + 1170)**2 + (y - 2310)**2 - (405 + r)**2 < 0
        c3 = (x + 1170)**2 + (y + 2310)**2 - (405 + r)**2 < 0
        c4 = (x + 1650)**2 + (y + 4600)**2 - (405 + r)**2 < 0

        r1_1 = y - (3220 - r) > 0
        r1_2 = y - (5050 + r) < 0
        r1_3 = x - (3630 + r) < 0
        r1_4 = x - (2770 - r) > 0

        r1 = r1_1 & r1_2 & r1_3 & r1_4 

        r2_1 = y - (4140 - r) > 0
        r2_2 = y - (5050 + r) < 0
        r2_3 = x - (4710 + r) < 0
        r2_4 = x - (4280 - r) > 0

        r2 = r2_1 & r2_2 & r2_3 & r2_4

        r3_1 = y - (1160 - r) > 0
        r3_2 = y - (1920 + r) < 0
        r3_3 = x - (5550 + r) < 0
        r3_4 = x - (1890 - r) > 0

        r3 = r3_1 & r3_2 & r3_3 & r3_4

        r4_1 = y - (-565 - r) > 0
        r4_2 = y - (605 + r) < 0
        r4_3 = x - (5550 + r) < 0
        r4_4 = x - (4970 - r) > 0

        r4 = r4_1 & r4_2 & r4_3 & r4_4

        r5_1 = y - (-1425 - r) > 0
        r5_2 = y - (-565 + r) < 0
        r5_3 = x - (5550 + r) < 0
        r5_4 = x - (4640 - r) > 0

        r5 = r5_1 & r5_2 & r5_3 & r5_4

        r6_1 = y - (-1900 - r) > 0
        r6_2 = y - (-70 + r) < 0
        r6_3 = x - (-260 + r) < 0
        r6_4 = x - (-1170 - r) > 0

        r6 = r6_1 & r6_2 & r6_3 & r6_4

        r7_1 = y - (-2400 - r) > 0
        r7_2 = y - (-1640 + r) < 0
        r7_3 = x - (1570 + r) < 0
        r7_4 = x - (-260 - r) > 0

        r7 = r7_1 & r7_2 & r7_3 & r7_4

        r8_1 = y - (-2380 - r) > 0
        r8_2 = y - (-1210 + r) < 0
        r8_3 = x - (3815 + r) < 0
        r8_4 = x - (2295 - r) > 0

        r8 = r8_1 & r8_2 & r8_3 & r8_4

        r9_1 = y - (-3267.5 - r) > 0
        r9_2 = y - (-2097.5 + r) < 0
        r9_3 = x - (5550 + r) < 0
        r9_4 = x - (4970 - r) > 0

        r9 = r9_1 & r9_2 & r9_3 & r9_4

        r10_1 = y - (-4700 - r) > 0
        r10_2 = y - (-3180 + r) < 0
        r10_3 = x - (1930 + r) < 0
        r10_4 = x - (-810 - r) > 0

        r10 = r10_1 & r10_2 & r10_3 & r10_4 

        r11_1 = y - (-4700 - r) > 0
        r11_2 = y - (-4120 + r) < 0
        r11_3 = x - (3410 + r) < 0
        r11_4 = x - (2240 - r) > 0

        r11 = r11_1 & r11_2 & r11_3 & r11_4 

        r12_1 = y - (-4700 - r) > 0
        r12_2 = y - (-3940 + r) < 0
        r12_3 = x - (5550 + r) < 0
        r12_4 = x - (3720 - r) > 0

        r12 = r12_1 & r12_2 & r12_3 & r12_4 

        r13_1 = y - (-5050 - r) > 0
        r13_2 = y - (-4700 + r) < 0
        r13_3 = x - (5550 + r) < 0
        r13_4 = x - (1300 - r) > 0

        r13 = r13_1 & r13_2 & r13_3 & r13_4 

        r14_1 = y - (2451 - r) > 0
        r14_2 = y - (4050 + r) < 0
        r14_3 = x - (-4550.4) > 0
        r14_4 = x - (-1952.7) < 0
        c14_1 = (x + 4550.4)**2 + (y - 3250.5)**2 - (799.5 + r)**2 < 0
        c14_2 = (x + 1952.7)**2 + (y - 3250.5)**2 - (799.5 + r)**2 < 0

        r14 = (r14_1 & r14_2 & r14_3 & r14_4) | (c14_1 | c14_2)

        # print(r14)

        b1_1 = y - (5050 - r) < 0
        b1_2 = y - (-5050 + r) > 0
        b1_3 = x - (5500 - r) < 0
        b1_4 = x - (-5500 + r) > 0

        b1 = not (b1_1 & b1_2 & b1_3 & b1_4)

        collision = b1 | r1 | r2 | r3 | r4 | r5 | r6 | r7 | r8 | r9 | r10 | r11 | r12 | r13 | r14 | c1 | c2 | c3 | c4
        # print(collision)

        return collision



def main():

    sx = -4000
    sy = -4000
    gx = 4000
    gy = 4000
    start = node(sx,sy,-1,-1,0,math.sqrt((sx-gx)**2 + (sy-gy)**2))
    goal = node(gx,gy,-1,-1,1000000,0)
    ts = time.time()
    plan = PathPlan(start, goal)#, mgrid, resolution, robot_radius, clearance, algo)
    if plan.checkCollision(start) or plan.checkCollision(goal):
        print("invalid")
    else:
        path = plan.algorithm()
        path.reverse()
        # print(path)
        # xData = []
        # yData = []
        # thetaData = []
        # stepidData = []
        # for i in range(len(path)):
        #     xData.append(path[i].x)
        #     yData.append(path[i].y)
        #     thetaData.append(path[i].theta)
        #     stepidData.append(path[i].stepid)
        # pathData = [xData,yData,thetaData,stepidData]
        # with open("path.pkl", "wb") as fp:   #Pickling
        #     pickle.dump(pathData, fp, protocol = 2)
    te = time.time()
    print("Time taken (sec) : ",(te-ts))

    # simulate(path)


if __name__ == '__main__':
    main()
        