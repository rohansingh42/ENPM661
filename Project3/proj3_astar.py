import argparse
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
import pickle

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

    resolution = 10       # Resolution for map defined in mm
    RPM1 = 50             # LOW RPM (rev/min)
    RPM2 = 100            # HIGH RPM (rev/min)
    wr = 38               # Wheel Radius (mm)
    wb = 230              # Wheel Base (mm)
    f = 1                 # Sampling Frequency (1/sec)
    rr = 177              # Robot Radius (mm)
    c = 100               # Clearance (mm)
    gt = 200              # Radius of circle that is considered goal position (mm)
    motion_model = [[0,RPM1],                
              [RPM1,0],     
              [RPM1,RPM1],               
              [0,RPM2],    
              [RPM2,0],               
              [RPM2,RPM2],   
              [RPM1,RPM2],          
              [RPM2,RPM1]]   

    def __init__(self, start, goal):

        self.start = start
        self.goal = goal

    def nextNode(self,cnode,stepid):

        step = self.motion_model[stepid]
        ul = 2*math.pi*step[0]/60
        ur = 2*math.pi*step[1]/60

        k1 = (self.wr/2) * (ul + ur)
        k2 = (self.wr/self.wb) * (ur - ul) 

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

        if k2 == 0:
            dx = k1 * math.cos(cnode.theta*(math.pi/180)) * (1/self.f)
            dy = k1 * math.sin(cnode.theta*(math.pi/180)) * (1/self.f)
        else:
            dx = (k1/k2) * (math.sin(theta*(math.pi/180)) - math.sin(cnode.theta*(math.pi/180)))
            dy = -(k1/k2) * (math.cos(theta*(math.pi/180)) - math.cos(cnode.theta*(math.pi/180)))

        x = np.round(dx/self.resolution,0)*self.resolution + cnode.x
        y = np.round(dy/self.resolution,0)*self.resolution + cnode.y

        g = cnode.g + (k1 * (1/self.f))

        d = 2*math.sqrt((self.goal.x - x)**2 + (self.goal.y - y)**2) + 1*np.absolute(theta - self.goal.theta)/18.0
        # d = np.absolute(self.goal.x - x) + np.absolute(self.goal.y - y) + 1*np.absolute(theta - self.goal.theta)/36.0

        n = node(x,y,theta,cnode.id,stepid,g,d)
        return n

    def algorithm(self):

        motion = self.motion_model
        opened = dict()
        closed = dict()
        pq = PriorityQueue()
        opened[self.start.id] = self.start
        pq.put(self.start)
        
        flag = True
        gf = True
        while flag == True:
            
            if pq.empty():
                print("Goal can't be reached.")
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

                for i,_ in enumerate(motion):

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
                        # print(child_node)

        if gf:
            path = self.gen_path(closed)
            self.writePath(path)

        return

    def writePath(self,path):
        
        xData = []
        yData = []
        thetaData = []
        stepidData = []
        for i in range(len(path)):
            xData.append(path[i].x)
            yData.append(path[i].y)
            thetaData.append(path[i].theta)
            stepidData.append(path[i].stepid)
        pathData = [self.RPM1,self.RPM2,self.f,xData,yData,thetaData,stepidData]
        with open("path.pkl", "wb") as fp:   #Pickling
            pickle.dump(pathData, fp, protocol = 2)
    

    def gen_path(self,closed):

        cid = self.goal.id
        path = []
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

        path.reverse()
        print("No. of nodes in path : ", len(path))
        plt.show()

        return path

    def checkGoal(self,cnode):
        goalReached = (math.sqrt((cnode.x - self.goal.x)**2 + (cnode.y - self.goal.y)**2) - self.gt) < 0
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

        b1_1 = y - (5050 - r) < 0
        b1_2 = y - (-5050 + r) > 0
        b1_3 = x - (5550 - r) < 0
        b1_4 = x - (-5550 + r) > 0

        b1 = not (b1_1 & b1_2 & b1_3 & b1_4)

        collision = b1 | r1 | r2 | r3 | r4 | r5 | r6 | r7 | r8 | r9 | r10 | r11 | r12 | r13 | r14 | c1 | c2 | c3 | c4

        return collision

def main():

    sx,sy = input("Enter space seperated start node x,y coordinate (m) : ").split()
    sx = np.int64(float(sx)*1000.0)
    sy = np.int64(float(sy)*1000.0)

    gx,gy = input("Enter space seperated goal node x,y coordinate (m) : ").split()
    gx = np.int64(float(gx)*1000.0)
    gy = np.int64(float(gy)*1000.0)

    start = node(sx,sy,0,-1,-1,0,math.sqrt((sx-gx)**2 + (sy-gy)**2))
    goal = node(gx,gy,0,-1,-1,1000000,0)

    # ts = time.time()

    plan = PathPlan(start, goal)
    if plan.checkCollision(start) | plan.checkCollision(goal):
        print("Nodes on obstacle or outside boundary. Enter again.")
    else:
        plan.algorithm()
        # path.reverse()
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
    # te = time.time()
    # print("Time taken (sec) : ",(te-ts))

    # simulate(path)


if __name__ == '__main__':
    main()
        