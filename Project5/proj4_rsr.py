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

# # import ros stuff
# import rospy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist, Point
# from nav_msgs.msg import Odometry
# from tf import transformations

class node:
    def __init__(self, x, y, theta, parentid, stepid, g, d = 0):
        self.x = np.int32(np.round(x, 0))
        self.y = np.int32(np.round(y, 0))
        self.theta = np.int32(np.round(theta, 0))
        self.id = 'x' + str(self.x) + 'y' + str(self.y)# + str(self.theta)
        self.parentid = parentid
        self.stepid = stepid
        self.g = np.round(g, 2)
        self.d = np.round(d, 2)
        self.h = g + d

    def __str__(self):
        return "x : " + str(self.x) + " y : " + str(self.y) + " theta : " + str(self.theta) + " id : " + str(self.id) + ", parentid : " + str(self.parentid) + ", stepid : " + str(self.stepid) + ", h : " + str(self.h) + ", g : " + str(self.g) + ", d : " + str(self.d) 

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
    cmap = matplotlib.colors.ListedColormap(['w', 'g', 'b', 'm', 'y', 'r', 'k'])
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
    gt = 200              # Radius of circle that is considered goal position (cm)
    motion_model = [[0,RPM1],                
              [RPM1,0],     
              [RPM1,RPM1],               
              [0,RPM2],    
              [RPM2,0],               
              [RPM2,RPM2],   
              [RPM1,RPM2],          
              [RPM2,RPM1]]   

    def __init__(self, start, goal): #, resolution = 1, robot_radius = 0, clearance = 0, algo = 'astar'):

        # self.robot_radius = robot_radius
        # self.clearance = clearance
        # self.resolution = resolution
        # self.mod_dim = np.int64(self.orig_dim/resolution)
        # self.mod_rad = (robot_radius + clearance)/resolution
        self.start = start
        self.goal = goal
        self.map_grid = np.zeros((self.mod_dim[1],self.mod_dim[0]))
        # self.algo = algo
        # grid = draw_map(robot_radius,resolution,clearance)
        # grid[self.mod_dim[1] - start.y, start.x] = 1
        # grid[self.mod_dim[1] - goal.y, goal.x] = 2
        # self.map_grid = grid


    def nextNode(self,cnode,stepid):

        step = self.motion_model[stepid]
        ul = 2*math.pi*step[0]/60
        ur = 2*math.pi*step[1]/60
        # print(ur)
        k1 = (self.wr/2) * (ul + ur)
        k2 = (self.wr/self.wb) * (ur - ul) 
        # print(k1,k2)
        dtheta = k2 * (180/math.pi) * (1/self.f)
        dtheta = dtheta - (np.round(dtheta/360, 0)*360)
        if dtheta > 180:
            dtheta = dtheta - 360
        elif dtheta < -180:
            dtheta = dtheta + 360

        theta = dtheta + cnode.theta
        if theta > 180:
            theta = theta - 360
        elif theta <= -180:
            theta = theta + 360

        # modtheta = cnode.theta + dtheta 
        # if modtheta > 180:
        #     modtheta = modtheta - 360
        # elif modtheta < -180:
        #     modtheta = modtheta + 360
        # theta = theta - (np.round(theta/360, 0)*360)
        if k2 == 0:
            dx = k1 * math.cos(cnode.theta*(math.pi/180)) * (1/self.f)
            dy = k1 * math.sin(cnode.theta*(math.pi/180)) * (1/self.f)
        else:
            dx = (k1/k2) * (math.sin(theta*(math.pi/180)) - math.sin(cnode.theta*(math.pi/180)))
            dy = -(k1/k2) * (math.cos(theta*(math.pi/180)) - math.cos(cnode.theta*(math.pi/180)))
        # dx = (self.wr/2) * (ul + ur) * math.cos(cnode.theta*(math.pi/180)) * (1/self.f)
        # # print(dx)
        # dy = (self.wr/2) * (ul + ur) * math.sin(cnode.theta*(math.pi/180)) * (1/self.f)
        # dtheta = ((self.wr/self.wb) * (ur - ul) * (1/self.f))*(180/math.pi)
        # theta = (dtheta + cnode.theta)%360
        # print(dtheta)
        x = np.round(dx/self.resolution,0)*self.resolution + cnode.x
        y = np.round(dy/self.resolution,0)*self.resolution + cnode.y
        # x = dx + cnode.x
        # y = dy + cnode.y
        # theta = (dtheta + cnode.theta)%360
        # print(theta)
        # if (k1 * (1/self.f)) > math.sqrt(dx**2 + dy**2):
        #     print('tr')
        # g = cnode.g + math.sqrt(dx**2 + dy**2)
        # print((k1 * (1/self.f)) - math.sqrt(dx**2 + dy**2))
        g = cnode.g + (k1 * (1/self.f))
        # d = math.sqrt((self.goal.x - x)**2 + (self.goal.y - y)**2)
        d = 2*math.sqrt((self.goal.x - x)**2 + (self.goal.y - y)**2) + 1*np.absolute(theta - self.goal.theta)/36.0
        # d = np.absolute(self.goal.x - x) + np.absolute(self.goal.y - y)
        # newid = str(x) + str(y) + str(theta)
        # print('cnode.x',cnode.x)
        # print('cnode.y',cnode.y)
        # print('x',x)
        # print('y',y)
        n = node(x,y,theta,cnode.id,stepid,g,d)
        return n

    def updateObstacles(self):
        r = self.rr + self.c
        for i in range(np.int64(-self.mod_dim[0]/2),np.int64(self.mod_dim[0]/2)):
            for j in range(np.int64(-self.mod_dim[1]/2),np.int64(self.mod_dim[1]/2)):
                
                x = i*self.resolution
                y = j*self.resolution

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
                b1_3 = x - (5550 - r) < 0
                b1_4 = x - (-5550 + r) > 0

                b1 = not (b1_1 & b1_2 & b1_3 & b1_4)

                collision = b1 | r1 | r2 | r3 | r4 | r5 | r6 | r7 | r8 | r9 | r10 | r11 | r12 | r13 | r14 | c1 | c2 | c3 | c4

                if collision:
                    self.map_grid[np.int64(self.mod_dim[1]/2 + j), np.int64(self.mod_dim[0]/2 + i)] = 6

    def algorithm(self):

        self.updateObstacles()

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

        plt.show(True)
        self.ax.hold(True)
        self.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        flag = True
        show_animation = False

        k = 200#np.int64(np.divide((self.mod_dim[0]*self.mod_dim[1]),200))
        j = 0
        gf = True
        while flag == False:
            
            if pq.empty():
                print("Goal can't be reached.")
                # curr_map = self.ax.pcolormesh(np.flip(self.map_grid,0), cmap=self.cmap, edgecolors='k', linewidths=self.resolution/280)
                # self.ax.draw_artist(curr_map)
                # self.fig.canvas.blit(self.ax.bbox)
                # plt.pause(3)
                # plt.close()
                break
            curr_node = pq.get()
            cid = curr_node.id
            
            if self.checkGoal(curr_node) :
                print("Goal Found")
                self.goal.parentid = curr_node.parentid
                self.goal.x = curr_node.x
                self.goal.y = curr_node.y
                self.goal.theta = curr_node.theta
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
        plt.axis([-5550,5550,-5050,5050])
        while cid != self.start.parentid:
            path.append(closed[cid])
            print(closed[cid])
            xdata.append(closed[cid].x)
            ydata.append(closed[cid].y)
            cid = closed[cid].parentid
            plt.plot(xdata,ydata,'go--', linewidth=2, markersize=3)
            # plt.pause(0.1)

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
        b1_3 = x - (5550 - r) < 0
        b1_4 = x - (-5550 + r) > 0

        b1 = not (b1_1 & b1_2 & b1_3 & b1_4)

        collision = b1 | r1 | r2 | r3 | r4 | r5 | r6 | r7 | r8 | r9 | r10 | r11 | r12 | r13 | r14 | c1 | c2 | c3 | c4

        return collision

    
# def check_node_validity(x,y,grid,mod_dim,mr):

#     if y > mod_dim[1] - mr:
#         return True
#     elif x < mr:
#         return True
#     elif x > mod_dim[0] - mr:
#         return True
#     elif y < mr:
#         return True
#     elif grid[mod_dim[1] - y, x] == 6:
#         return True
#     else:
#         return False

# def draw_map(robot_radius,resolution,clearance):

#     map_height = np.int64(150/resolution)
#     map_width = np.int64(250/resolution)
#     xx,yy = np.mgrid[:map_height+1,:map_width+1]
#     # print(xx)
#     xx = resolution*xx
#     yy = resolution*yy
#     # print(xx)
#     mod_radius = robot_radius + clearance

#     circle = (xx-130)**2+(yy-190)**2 - (15 + mod_radius)**2
#     ellipse = (6*(yy-140))**2 + (15*(xx-120))**2 - (90 + mod_radius)**2
#     #polygon
#     l4 = 38*yy+23*xx-(8530 + mod_radius*(math.sqrt(38**2 + 23**2)))
#     l3 = -20*xx+37*yy-(6101 + mod_radius*(math.sqrt(20**2 + 37**2)))
#     l2 = -xx+15 - mod_radius
#     l1 = 41*yy+25*xx-(6525 - mod_radius*(math.sqrt(41**2 + 25**2)))
#     l6 = 4*yy+38*xx-(2628 + mod_radius*(math.sqrt(4**2 + 38**2)))
#     l5 = 38*yy-7*xx-(5830 - mod_radius*(math.sqrt(38**2 + 7**2)))

#     #rect
#     s1 = xx - (112.5+mod_radius)
#     s2 = yy - (100+mod_radius)
#     s3 = xx - (67.5-mod_radius)
#     s4 = yy - (50-mod_radius)

#     #boundary
#     b1 = xx - mod_radius
#     b2 = -xx + (150 - mod_radius)
#     b3 = yy - mod_radius
#     b4 = -yy + (250 - mod_radius)


#     grid = np.full((map_height+1,map_width+1), 0, dtype=int)
#     grid[(circle<=0) | (ellipse<=0) |((s1<=0)&(s2<=0)&(s3>=0)&(s4>=0)) | ( (l1>=0)&(l2<=0)&(l3<=0)&(l4<=0)&((l5>=0) | (l6<=0)) ) 
#             |((b1<0) | (b2<=0) | (b3<0) | (b4<=0))] = 6
#     grid = np.flip(grid,0)

#     return grid


def main():

    sx = -4000
    sy = -4000
    gx = -4000
    gy = 2000
    start = node(sx,sy,0,-1,-1,0,math.sqrt((sx-gx)**2 + (sy-gy)**2))
    goal = node(gx,gy,0,-1,-1,1000000,0)
    # ts = time.time()
    plan = PathPlan(start, goal)#, mgrid, resolution, robot_radius, clearance, algo)
    if plan.checkCollision(start) | plan.checkCollision(goal):
        print("invalid")
    else:
        path = plan.algorithm()
        path.reverse()
        # print(path)
        xData = []
        yData = []
        thetaData = []
        stepidData = []
        for i in range(len(path)):
            xData.append(path[i].x)
            yData.append(path[i].y)
            thetaData.append(path[i].theta)
            stepidData.append(path[i].stepid)
        pathData = [xData,yData,thetaData,stepidData]
        with open("path.pkl", "wb") as fp:   #Pickling
            pickle.dump(pathData, fp, protocol = 2)
    # te = time.time()
    # print("Time taken (sec) : ",(te-ts))

    # simulate(path)


if __name__ == '__main__':
    main()
        