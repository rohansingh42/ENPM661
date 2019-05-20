"""
ENPM661 Spring 2019: Planning for Autonomous Robots
Project 2: Dijkstra and A-star for point and rigid robots


Author:
Abhinav Modi (abhi1625@umd.edu)
Graduate Student pursuing Masters in Robotics,
University of Maryland, College Park
"""
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import heapq
show_animation=True


#Define Node
class Node:
    def __init__(self,x,y,c2c,parentId):
        self.x = x
        self.y = y
        self.c2c = c2c
        self.parentId = parentId
    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.c2c)+","+str(self.parentId)

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

    if grid[int(node.y)][int(node.x)] ==False:
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
    # d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    d = np.absolute(n1.x-n2.x)+np.absolute(n1.y-n2.y)

    return d
#Define A Star
def astar(nStart,nGoal,robot_radius,grid,ox,oy):
    # nStart=Node(round(start_x),round(start_y),0.0,-1)
    # nGoal = Node(round(goal_x),round(goal_y),0.0,-1)

#     grid = generate_map(robot_radius)

    motion = motion_model()
    cost_heap = []
    active, dead = dict(),dict()
    active = dict()
    # closed = set()
    # open = set()
    active[gen_index(nStart)] = nStart
    # open.add(gen_index(nStart))
    heapq.heappush(cost_heap,[nStart.c2c,nStart])

    #cache the background for plotting
    fig,ax = plt.subplots(figsize = (25,15))
    canvas = ax.figure.canvas
    ax.grid(True)
    plt.xlim(0,250)
    plt.ylim(0,150)
    # ax.hold(True)
    x = np.linspace(1,151,150)
    y = np.linspace(1,251,250)
    for i in x:
        plt.axhline(y=i,color='y')
    for j in y:
        plt.axvline(x=j,color='y')
    p1 = ax.plot(ox,oy,"ok")
    p2 = ax.plot(nStart.x,nStart.y,"or")
    p3 = ax.plot(nGoal.x,nGoal.y,"ob")
    ax.grid(True)

    plt.show(False)
    ax.hold(True)
    canvas.draw()

    background = fig.canvas.copy_from_bbox(ax.bbox)
    pts = ax.plot(nStart.x,nStart.y,".c")[0]
    while 1:

        current = heapq.heappop(cost_heap)[1]
        # print("test",current)
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
            nGoal.c2c = nGoal.c2c
            break


        #Pop the visited nodes from the active set
        # open.remove(curr_id)
        #Add the visited node to the dead set
        # closed.add(curr_id)
        dead[curr_id] = current
        #Search the adjacent nodes
        for i,_ in enumerate(motion):
            new_node = Node(current.x + motion[i][0],current.y + motion[i][1],current.c2c + motion[i][2],curr_id)

            #Calculate new ID for each new nodes
            new_id = gen_index(new_node)

            #Checks for unique and feasible node
            if collision(new_node,grid)==False:
                continue

            if new_id in dead:
                continue

            if new_id in active:
                if active[new_id].c2c > new_node.c2c:
                    active[new_id].c2c = new_node.c2c
                    active[new_id].parentId = new_node.parentId
                    heapq.heappush(cost_heap,[active[new_id].c2c,active[new_id]])
            else:
                active[new_id] = new_node
                heapq.heappush(cost_heap,[active[new_id].c2c,active[new_id]])
                # open.add(new_id)
    path_x,path_y = findPath(nGoal,dead)

    return path_x,path_y,ax,fig

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
    xx,yy = np.mgrid[:152,:252]

    #Grid Boundary
    o1 = xx
    o2 = yy
    o3 = xx
    o4 = yy


    circle = (xx-130)**2+(yy-190)**2
    ellipse = (6*(yy-140))**2 + (15*(xx-120))**2
    #polygon
    l4 = 38*yy+23*xx-(8530+robot_radius*(math.sqrt(38**2 + 23**2)))
    l3 = -20*xx+37*yy-(6101+robot_radius*(math.sqrt(20**2 + 37**2)))
    l2 = -xx+15-robot_radius
    l1 = 41*yy+25*xx-(6525-robot_radius*(math.sqrt(41**2 + 25**2)))
    l6 = 4*yy+38*xx-(2628+robot_radius*(math.sqrt(4**2 + 38**2)))
    l5 = 38*yy-7*xx-(5830-robot_radius*(math.sqrt(38**2 + 7**2)))

    #rect
    s1 = xx
    s2 = yy
    s3 = xx
    s4 = yy

    # grid = np.full((151, 251), True, dtype=bool)
    grid = np.full((152, 252), True, dtype=bool)
    grid[o1<=0+robot_radius]=False
    grid[o2<=0+robot_radius]=False
    grid[o3>=151-robot_radius]=False
    grid[o4>=251-robot_radius]=False
    grid[(circle<=(15+robot_radius)**2) | (ellipse<=(90+robot_radius)**2) |((s1<=112.5+robot_radius)&(s2<=100+robot_radius)&(s3>=67.5-robot_radius)&(s4>=50-robot_radius)) ]=False  #(l5>0) | (l6<0)))
    grid[(l1>=0)&(l2<=0)&(l3<=0)&(l4<=0)&((l5>=0)|(l6<=0))] = False
    for i in range(152):
        for j in range(252):
            if grid[i][j]==False:
                ox.append(j)
                oy.append(i)

    return grid,ox,oy


def main():
    robot_radius = 5.0
    grid,ox,oy = generate_map(robot_radius)
    sx = 20.0
    sy = 20.0
    nStart=Node((sx),(sy),0.0,-1)
    start = False
    goal=False
    #collision check for start node
    if collision(nStart,grid):
        start=True
        print("Start node valid")
    else:
        print("Start node Invalid")
    gx = 50.0
    gy = 50.0
    nGoal = Node(gx,gy,0.0,-1)

    #Check for collision of goal node
    if collision(nGoal,grid):
        goal=True
        print("Goal Node valid")
    else:
        print("Goal node Invalid")

    if start==True and goal==True:
        start = time.time()
        rx,ry,ax,fig = astar(nStart,nGoal,robot_radius=robot_radius,grid=grid,ox=ox,oy=oy)
        end = time.time()
        print("Time to completion:",(end-start))
        if show_animation:
            p4 = ax.plot(rx,ry,"-r")
            plt.show()

if __name__ == '__main__':
    main()
