"""
ENPM661 Spring 2019: Planning for Autonomous Robots
Project 2: Dijkstra and A-star for point and rigid robots


Author:
Abhinav Modi (abhi1625@umd.edu)
Graduate Student pursuing Masters in Robotics,
University of Maryland, College Park
"""

import argparse
import os, sys

# This try-catch is a workaround for Python3 when used with ROS; it is not needed for most platforms
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass

import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import heapq
show_animation=True


#Define Node
class Node:
    def __init__(self,x,y,h,parentId):
        self.x = x
        self.y = y
        self.h = h
        self.parentId = parentId
    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.h)+","+str(self.parentId)
    def __lt__(self,other):
        return self.h < other.h

#Define costs for each movement
def motion_model():
    # dx, dy, cost
    #Moving in anticlockwise direction
    motion = [[1,0,1],                #Move Right
              [1,1,math.sqrt(2)],     #Move Right Up
              [0,1,1],                #Move Up
              [-1,1,math.sqrt(2)],    #Move Left Up
              [-1,0,1],               #Move Left
              [-1,-1,math.sqrt(2)],   #Move Left Down
              [0,-1,1],               #Move Down
              [1,-1,math.sqrt(2)]]    #Move Left Down
    return motion

def gen_index(node):
    return node.x*250+node.y


def collision(node, grid):

    if grid[int(node.y)][int(node.x)] == False:
        return False
    elif node.y>=150:
        return False
    elif node.x <= 0:
        return False
    elif node.x >=250:
        return False
    elif node.y<= 0:
        return False
    else:
        return True

def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d
#Define Dijkstra
def dijkstra(start_x,start_y, goal_x,goal_y,robot_radius,grid,ox,oy):
    nStart=Node(round(start_x),round(start_y),0.0,-1)
    nGoal = Node(round(goal_x),round(goal_y),0.0,-1)

#     grid = generate_map(robot_radius)

    motion = motion_model()
    cost_heap = []
    active, dead = dict(),dict()
    active = dict()
    closed = set()
    open = set()
    active[gen_index(nStart)] = nStart
    open.add(gen_index(nStart))
    heapq.heappush(cost_heap,[nStart.h,nStart])

    #cache the background for plotting
    fig,ax = plt.subplots(figsize = (25,15))
    canvas = ax.figure.canvas
    # ax.hold(True)
    p1 = ax.plot(ox,oy,".k")
    p2 = ax.plot(start_x,start_y,"or")
    p3 = ax.plot(goal_x,goal_y,"ob")
    ax.grid(True)

    plt.show(False)
    ax.hold(True)
    canvas.draw()

    background = fig.canvas.copy_from_bbox(ax.bbox)
    pts = ax.plot(nStart.x,nStart.y,"xc")[0]
    while 1:

        print(cost_heap[0][0])
        current = heapq.heappop(cost_heap)[1]
        print("test",current)
        curr_id = gen_index(current)

        #Draw Graph to show node exploration
        if show_animation:
            pts.set_data(current.x,current.y)
            # fig.canvas.restore_region(background)
            ax.draw_artist(pts)
            fig.canvas.blit(ax.bbox)
            # if len(dead)%10 == 0:
                # plt.pause(0.00000000001)



        if current.x == nGoal.x and current.y == nGoal.y:
            print("Target Found")
            nGoal.parentId = current.parentId
            nGoal.h = nGoal.h
            break


        #Pop the visited nodes from the active set
        # open.remove(curr_id)
        #Add the visited node to the dead set
        closed.add(curr_id)
        dead[curr_id] = current
        #Search the adjacent nodes
        for i,_ in enumerate(motion):
            new_node = Node(current.x + motion[i][0],current.y + motion[i][1],current.h + motion[i][2],curr_id)

            #Calculate new ID for each new nodes
            new_id = gen_index(new_node)

            #Checks for unique and feasible node
            if collision(new_node,grid)==False:
                continue

            if new_id in closed:
                continue

            if new_id in open:
                if active[new_id].h > new_node.h:
                    active[new_id].h = new_node.h
                    active[new_id].parentId = new_node.parentId
                    heapq.heappush(cost_heap,[active[new_id].h+calc_heuristic(nGoal,active[new_id]),active[new_id]])
            else:
                active[new_id] = new_node
                heapq.heappush(cost_heap,[active[new_id].h+calc_heuristic(nGoal,active[new_id]),active[new_id]])
                open.add(new_id)
    path_x,path_y = findPath(nGoal,dead)

    return path_x,path_y

#Trace Back the final path
def findPath(nGoal,dead):
    rx,ry = [nGoal.x],[nGoal.y]
    parentId = nGoal.parentId

    while parentId !=-1:
        visited = dead[parentId]
        rx.append(visited.x)
        ry.append(visited.y)
        parentId = visited.parentId

    return rx,ry



#Define required Functions
def generate_map(robot_radius):
    ox=[]
    oy = [] #Row wise list of True or False based on if there is an obstacle or not
    xx,yy = np.mgrid[:151,:251]

    circle = (xx-130)**2+(yy-190)**2
    ellipse = (6*(yy-140))**2 + (15*(xx-120))**2
    #polygon
    l4 = 38*yy+23*xx-8530
    l3 = -20*xx+37*yy-6101
    l2 = -xx+15
    l1 = 41*yy+25*xx-6525
    l6 = 4*yy+38*xx-2628
    l5 = 38*yy-7*xx-5830

    #rect
    s1 = xx
    s2 = yy
    s3 = xx
    s4 = yy

    grid = np.full((151, 251), True, dtype=bool)
    grid[(circle<=255) | (ellipse<=8100) |((s1<=112.5)&(s2<=100)&(s3>=67.5)&(s4>=50))| ((l1>=0)&(l2<=0)&(l3<=0)&(l4<=0)&((l5>=0) | (l6<=0)))]=False
    for i in range(150):
        for j in range(250):
            if grid[i][j]==False:
                ox.append(j)
                oy.append(i)

    

    return grid,ox,oy


def main():
    robot_radius = 1.0
    grid,ox,oy = generate_map(robot_radius)
    sx = 1.0
    sy = 1.0
    nStart=Node((sx),(sy),0.0,-1)

    #collision check for start node
    if not collision(nStart,grid):
        print("Start node invalid")
    gx = 249.0
    gy = 149.0
    nGoal = Node(gx,gy,0.0,-1)

    #Check for collision of goal node
    if not collision(nGoal,grid):
        print("Goal Node invalid")

    start = time.time()
    rx,ry = dijkstra(start_x=sx,start_y=sy, goal_x=gx,goal_y=gy,robot_radius=robot_radius,grid=grid,ox=ox,oy=oy)
    end = time.time()
    print("Time to completion:",(end-start))
    if show_animation:
        plt.plot(rx,ry,"-r")
        plt.show()

if __name__ == '__main__':
    main()
