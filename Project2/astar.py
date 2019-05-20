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
import matplotlib
import time
import heapq
show_animation=True

class node:
    def __init__(self, x, y, nodeid, parentid, g, d = 0):
        self.x = np.int(x)
        self.y = np.int(y)
        self.id = nodeid
        self.parentid = parentid
        self.g = g
        self.d = d
        self.h = g + d

    def __str__(self):
        return "x : " + str(self.x) + " y : " + str(self.y) + " id : " + str(self.id) + ", h : " + str(self.h) + ", g : " + str(self.g) + ", d : " + str(self.d) + ", parentid : " + str(self.parentid)

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
    orig_dim = np.int16((250,150))
    cmap = matplotlib.colors.ListedColormap(['w', 'g', 'b', 'm', 'y', 'r', 'k'])
    canvas = ax.figure.canvas
    # pq = PriorityQueue()
    # opened = set()
    # closed = set()
    motion_model = [[1,0,1],                #Move Right
              [1,1,math.sqrt(2)],     #Move Right Up
              [0,1,1],                #Move Up
              [-1,1,math.sqrt(2)],    #Move Left Up
              [-1,0,1],               #Move Left
              [-1,-1,math.sqrt(2)],   #Move Left Down
              [0,-1,1],               #Move Down
              [1,-1,math.sqrt(2)]]    #Move Left Down

    def __init__(self, start, goal, grid, resolution = 1, robot_radius = 0, clearance = 0, algo = 'astar'):
        self.robot_radius = robot_radius
        self.clearance = clearance
        self.resolution = resolution
        self.mod_dim = np.int16(self.orig_dim/resolution)
        self.mod_rad = (robot_radius + clearance)/resolution
        print(self.mod_dim)
        self.start = start
        self.goal = goal
        self.algo = algo
        grid = draw_map(robot_radius,resolution,clearance)
        grid[self.mod_dim[1] - start.y, start.x] = 1
        grid[self.mod_dim[1] - goal.y, goal.x] = 2
        print(self.mod_dim[1] - goal.y, goal.x)
        self.map_grid = grid

    # def check_collision(self,cnode):

    #     if cnode.y > self.mod_dim[1]:
    #         return True
    #     elif cnode.x < 0:
    #         return True
    #     elif cnode.x > self.mod_dim[0]:
    #         return True
    #     elif cnode.y < 0:
    #         return True
    #     elif self.map_grid[self.mod_dim[1] - cnode.y, cnode.x] == 6:
    #         return True
    #     else:
    #         return False

    def check_collision(self,cnode):

        if cnode.y > self.mod_dim[1] - (self.mod_rad):
            return True
        elif cnode.x < self.mod_rad:
            return True
        elif cnode.x > self.mod_dim[0] - (self.mod_rad):
            return True
        elif cnode.y < self.mod_rad:
            return True
        elif self.map_grid[self.mod_dim[1] - cnode.y, cnode.x] == 6:
            return True
        else:
            return False


    def algorithm(self):

    #     grid = generate_map(robot_radius)

        motion = self.motion_model
        opened = dict()
        closed = dict()
        pq = PriorityQueue()
        opened[self.start.id] = self.start
        pq.put(self.start)

        obs = self.ax.pcolormesh(np.flip(self.map_grid,0), cmap=self.cmap, edgecolors='k', linewidths=self.resolution/280)
        # obs = self.ax.pcolorfast(np.flip(self.map_grid,0), cmap=self.cmap)#, edgecolors='k', linewidths=0.2)
        plt.xlim(0,self.mod_dim[0])
        plt.ylim(0,self.mod_dim[1])
        plt.gca().set_aspect('equal', adjustable='box')
        
        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()

        plt.show(False)
        self.ax.hold(True)
        self.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        # plt.pause(10)
        flag = True
        show_animation = True
        k = np.int32((self.mod_dim[0]*self.mod_dim[1])/150)
        j = 0
        while flag == True:
            # print(cost_heap[0][0])
            curr_node = pq.get()
            # print("test",curr_node)
            cid = curr_node.id
            
            if curr_node.id == self.goal.id :
                print("Goal Found")
                self.goal.parentid = curr_node.parentid
                self.goal.g = curr_node.g
                self.goal.d = curr_node.d
                self.goal.h = curr_node.h
                closed[curr_node.id] = curr_node
                flag = False
                print('GOAL - ',self.goal)
                break
            
            if curr_node.id in closed:
                continue
            else:
                opened.pop(curr_node.id)
                closed[curr_node.id] = curr_node
                if (curr_node.id != self.start.id) and (curr_node.id != self.goal.id):
                    self.map_grid[self.mod_dim[1] - curr_node.y, curr_node.x] = 3
                    # print(j)
                
                # if curr_node.id == self.start.id:
                #     print("wtf",j)

                for i,_ in enumerate(motion):

                    child_id = curr_node.x + motion[i][0] + self.mod_dim[0]*(curr_node.y + motion[i][1])  
                    # if child_id == self.start.id:
                    #     print("cid : ",child_id)
                    #     print("sid : ",self.start.id)
                    # print(' 1 : ',child_id)  
                    # distance measure from goal (0 for dijkstra)
                    if self.algo == 'dijkstra':
                        child_d = 0
                    elif self.algo == 'astar':
                        weight = 1.0
                        child_d =  weight * math.sqrt(((curr_node.x + motion[i][0]) - self.goal.x)**2 + ((curr_node.y + motion[i][1]) - self.goal.y)**2)

                    child_node = node(curr_node.x + motion[i][0], 
                                    curr_node.y + motion[i][1], 
                                    child_id, curr_node.id, 
                                    curr_node.g + motion[i][2], 
                                    child_d)
                    
                    if self.check_collision(child_node):
                        continue
                    elif child_id in closed:
                        # if child_node.id == self.start.id:
                        #     print("cid : ",child_node.id)
                        continue
                    elif child_id in opened:
                        if child_node.g < opened[child_id].g:
                            opened[child_id].g = child_node.g
                            opened[child_id].parentid = child_node.parentid
                            pq.put(child_node)
                        else:
                            continue
                    else:
                        opened[child_id] = child_node
                        pq.put(child_node)

                    # print(pq.elements[0].id)
                    if child_node.id != self.goal.id:
                        self.map_grid[self.mod_dim[1] - child_node.y, child_node.x] = 4
                    # print(child_node.y, child_node.x)

                    # if show_animation:

                    #     # for y in range(self.mod_dim[1]+1):
                    #     #     hlines = self.ax.axhline(y, lw=1, color='k', zorder=0)
                    #     # for x in range(self.mod_dim[0]+1):
                    #     #     vlines = self.ax.axvline(x, lw=1, color='k', zorder=0)
                    #     curr_map = self.ax.pcolorfast(np.flip(self.map_grid,0), cmap=self.cmap)#, edgecolors='k', linewidths=0.2)
                    #     # curr_map = self.ax.imshow(self.map_grid, interpolation='none', cmap=self.cmap, extent=[0, self.mod_dim[0]+1, 0, self.mod_dim[1]+1], zorder=0)
                    #     # self.ax.draw_artist(hlines)
                    #     # self.ax.draw_artist(vlines)
                    #     self.ax.draw_artist(curr_map)
                    #     self.fig.canvas.blit(self.ax.bbox)

                    # # # draw the grid
                    # for y in range(self.mod_dim[1]+1):
                    #     self.ax.axhline(y, lw=1, color='k', zorder=5)
                    # for x in range(self.mod_dim[0]+1):
                    #     self.ax.axvline(x, lw=1, color='k', zorder=5)
                    # # draw the boxes
                    # self.ax.imshow(self.map_grid, interpolation='none', cmap=self.cmap, extent=[0, self.mod_dim[0]+1, 0, self.mod_dim[1]+1], zorder=0)
                    # plt.pause(0.05)
                    # # turn off the axis labels
                    # self.ax.axis('off')

                    # plt.show(False)
                    # self.ax.hold(True)
                    # print(' 2 : ',child_node.id)


                    # #Calculate new ID for each new nodes
                    # new_id = gen_index(new_node)

                    # #Checks for unique and feasible node
                    # if collision(new_node,grid)==False:
                    #     continue

                    # if new_id in closed:
                    #     continue

                    # if new_id in open:
                    #     if active[new_id].h > new_node.h:
                    #         active[new_id].h = new_node.h
                    #         active[new_id].parentId = new_node.parentId
                    #         heapq.heappush(cost_heap,[active[new_id].h+calc_heuristic(nGoal,active[new_id]),active[new_id]])
                    # else:
                    #     active[new_id] = new_node
                    #     heapq.heappush(cost_heap,[active[new_id].h+calc_heuristic(nGoal,active[new_id]),active[new_id]])
                    #     open.add(new_id)
                # show_animation = not show_animation
                # if show_animation:
                # k=1
                if j%k == 0:
                        
                    # for y in range(self.mod_dim[1]+1):
                    #     hlines = self.ax.axhline(y, lw=1, color='k', zorder=0)
                    # for x in range(self.mod_dim[0]+1):
                    #     vlines = self.ax.axvline(x, lw=1, color='k', zorder=0)
                    curr_map = self.ax.pcolormesh(np.flip(self.map_grid,0), cmap=self.cmap, edgecolors='k', linewidths=self.resolution/280)
                    # curr_map = self.ax.imshow(self.map_grid, interpolation='none', cmap=self.cmap, extent=[0, self.mod_dim[0]+1, 0, self.mod_dim[1]+1], zorder=0)
                    # self.ax.draw_artist(hlines)
                    # self.ax.draw_artist(vlines)
                    self.ax.draw_artist(curr_map)
                    self.fig.canvas.blit(self.ax.bbox)
                    # plt.pause(3)

                j = j+1
                # print(i)
        # path_x,path_y = findPath(nGoal,dead)
            # print(self.map_grid[self.mod_dim[1] - self.start.y, self.start.x])
        # print(self.map_grid[self.map_grid==1])
        self.ax.hold(True)

        self.gen_path(closed)
        # return path_x,path_y
    

    def gen_path(self,closed):
        # rx,ry = [nGoal.x],[nGoal.y]
        parentid = self.goal.parentid
        path = [closed[self.goal.id]]
        while parentid != self.start.id:
            path.append(closed[parentid])
            parentid = closed[parentid].parentid

        print("No. of nodes in path : ", len(path))
        
        path.reverse()
        i = 0
        for n in path:
            if i%5 == 0:
                self.map_grid[self.mod_dim[1] - n.y, n.x] = 5
                curr_map = self.ax.pcolormesh(np.flip(self.map_grid,0), cmap=self.cmap, edgecolors='k', linewidths=self.resolution/280)
                self.ax.draw_artist(curr_map)
                self.fig.canvas.blit(self.ax.bbox)
            # plt.pause(0.001)
        plt.pause(5)

    
def check_node_validity(x,y,grid,mod_dim,mr):

    # mr = np.int((robot_radius + clearance)/resolution)

    if y > mod_dim[1] - mr:
        return True
    elif x < mr:
        return True
    elif x > mod_dim[0] - mr:
        return True
    elif y < mr:
        return True
    elif grid[mod_dim[1] - y, x] == 6:
        return True
    else:
        return False

def draw_map(robot_radius,resolution,clearance):

    map_height = np.int(150/resolution)
    map_width = np.int(250/resolution)
    xx,yy = np.mgrid[:map_height+1,:map_width+1]
    # print(xx)
    xx = resolution*xx
    yy = resolution*yy
    # print(xx)
    mod_radius = robot_radius + clearance

    circle = (xx-130)**2+(yy-190)**2 - (15 + mod_radius)**2
    ellipse = (6*(yy-140))**2 + (15*(xx-120))**2 - (90 + mod_radius)**2
    #polygon
    l4 = 38*yy+23*xx-(8530 + mod_radius*(math.sqrt(38**2 + 23**2)))
    l3 = -20*xx+37*yy-(6101 + mod_radius*(math.sqrt(20**2 + 37**2)))
    l2 = -xx+15 - mod_radius
    l1 = 41*yy+25*xx-(6525 - mod_radius*(math.sqrt(41**2 + 25**2)))
    l6 = 4*yy+38*xx-(2628 + mod_radius*(math.sqrt(4**2 + 38**2)))
    l5 = 38*yy-7*xx-(5830 - mod_radius*(math.sqrt(38**2 + 7**2)))

    #rect
    s1 = xx - (112.5+mod_radius)
    s2 = yy - (100+mod_radius)
    s3 = xx - (67.5-mod_radius)
    s4 = yy - (50-mod_radius)

    #boundary
    b1 = xx - mod_radius
    b2 = -xx + (150 - mod_radius)
    b3 = yy - mod_radius
    b4 = -yy + (250 - mod_radius)


    grid = np.full((map_height+1,map_width+1), 0, dtype=int)
    grid[(circle<=0) | (ellipse<=0) |((s1<=0)&(s2<=0)&(s3>=0)&(s4>=0)) | ( (l1>=0)&(l2<=0)&(l3<=0)&(l4<=0)&((l5>=0) | (l6<=0)) ) 
            |((b1<0) | (b2<=0) | (b3<0) | (b4<=0))] = 6
    grid = np.flip(grid,0)

    # print(grid.shape)
    return grid



def main():

    map_width = 250
    map_height = 150
    orig_dim = np.int16([map_width,map_height])

    while True:
        try:
            resolution = float(input("Enter resolution ( >= 1) : "))
        except ValueError:
            print("INVALID. Enter integer or float >= 1.")
        # print(resolution)
        # elif resolution == []:
        #     print("INVALID. Try again.")
        # elif not(resolution.isdigit()):#(type(resolution) is str):
        #     print("INVALID. Enter integer or float >= 1.")
        else:
            break

    mod_dim = np.int16(np.divide(orig_dim,resolution))

    while True:
        try:
            robot_radius = float(input("Enter the robot's radius (0 for point robot): "))
        except ValueError:
            print("INVALID. Enter integer or float >= 1.")
        else:
            break

    while True:
        try:
            clearance = float(input("Enter clearance : "))
        except ValueError:
            print("INVALID. Enter integer or float >= 1.")
        else:
            break

    mgrid = draw_map(robot_radius,resolution,clearance)
    mod_rad = np.int((robot_radius + clearance)/resolution)

    while True:
        try:
            sx = float(input("Enter start node x coordinate : "))
            sy = float(input("Enter start node y coordinate : "))
        # if sx == [] | sy == []:
        #     print("INVALID. Try again.")
        # elif (type(sx) is str) | (type(sy) is str) :
        #     print("INVALID. Enter integer or float.")
        except ValueError:
            print("INVALID. Enter integer or float >= 1.")
            continue
        sx = np.int(sx/resolution)
        sy = np.int(sy/resolution)
        if check_node_validity(sx,sy,mgrid,mod_dim,mod_rad):
            print("Start node on obstacle or outside boundary. Enter again.")
        else:
            break

    while True:
        try:
            gx = float(input("Enter goal node x coordinate : "))
            gy = float(input("Enter goal node y coordinate : "))
        # if gx == [] | gy == []:
        #     print("INVALID. Try again.")
        # elif (type(gx) is str) | (type(gy) is str) :
        #     print("INVALID. Enter integer or float.")
        except ValueError:
            print("INVALID. Enter integer or float >= 1.")
            continue
        gx = np.int(gx/resolution)
        gy = np.int(gy/resolution)
        if check_node_validity(gx,gy,mgrid,mod_dim,mod_rad):
            print("Goal node on obstacle or outside boundary. Enter again.")
        elif gx == sx and gy == sy:
            print("Goal same as start. Enter seperate coordinates.")
        else:
            break
    
    while True:
        algo = input("Enter algorithm (\"astar\" or \"dijkstra\") : ")
        if algo == []:
            print("INVALID. Try again.")
        # elif not (type(resolution) is str):
        #     print("INVALID. Enter string.")
        elif (algo != "dijkstra") and (algo != "astar"):
            print("INVALID. Enter correct terms.")
        else:
            break
    
    
    # sx = np.int(sx/resolution)
    # sy = np.int(sy/resolution)
    sid = mod_dim[0]*sy + sx
    start = node((sx),(sy),sid,-1,0,0)

    # gx = np.int(gx/resolution)
    # gy = np.int(gy/resolution)
    gid = mod_dim[0]*gy + gx
    goal = node((gx),(gy),gid,-1,1000000,0)

    plan = PathPlan(start, goal, mgrid, resolution, robot_radius, clearance, algo)

    plan.algorithm()
    # robot_radius = 0
    # resolution = 1
    # clearance = 0
    # map_height = np.int(150/resolution)
    # map_width = np.int(250/resolution)
    # mgrid = draw_map(robot_radius,resolution,clearance)

    # # make a figure + axes
    # # fig, ax = plt.subplots(1, 1, tight_layout=True)

    # # # make color map
    # # my_cmap = matplotlib.colors.ListedColormap(['w', 'g', 'b', 'm', 'y', 'k'])
    # # # set the 'bad' values (nan) to be white and transparent
    # # my_cmap.set_bad(color='w', alpha=0)
    # # # draw the grid
    # # for x in range(map_height+1):
    # #     ax.axhline(x, lw=1, color='k', zorder=5)
    # # for y in range(map_width+1):
    # #     ax.axvline(y, lw=1, color='k', zorder=5)
    # # # draw the boxes
    # # ax.imshow(grid, interpolation='none', cmap=my_cmap, extent=[0, map_width+1, 0, map_height+1], zorder=0)
    # # # turn off the axis labels
    # # ax.axis('off')

    # # ax.hold(True)

    # # plt.show()

    # # canvas.draw()
    # # print(grid)
    # sx = np.int(11.0/resolution)
    # sy = np.int(11.0/resolution)
    # sid = map_width*sy + sx
    # print(sid)
    # start = node((sx),(sy),sid,-1,0,0)

    # gx = np.int(250.0/resolution)
    # gy = np.int(150.0/resolution)
    # gid = map_width*gy + gx
    # goal = node((gx),(gy),gid,-1,1000000,0)
    # print(gid)

    # plan = PathPlan(start, goal, mgrid, resolution, robot_radius, clearance, 'dijkstra')

    # plan.algorithm()
    # # #collision check for start node
    # # if not collision(nStart,grid):
    # #     print("Start node invalid")
    # # gx = 249.0
    # # gy = 149.0
    # # nGoal = Node(gx,gy,0.0,-1)

    # # #Check for collision of goal node
    # # if not collision(nGoal,grid):
    # #     print("Goal Node invalid")

    # # start = time.time()
    # # rx,ry = dijkstra(start_x=sx,start_y=sy, goal_x=gx,goal_y=gy,robot_radius=robot_radius,grid=grid,ox=ox,oy=oy)
    # # end = time.time()
    # # print("Time to completion:",(end-start))
    # # if show_animation:
    # #     plt.plot(rx,ry,"-r")
    # #     plt.show()

if __name__ == '__main__':
    main()
        
        # map_width = 250
        # map_height = 150

        # while True:
        #     resolution = input("Enter resolution (integer >= 1) : ")
        #     if resolution == []:
        #         print("INVALID. Try again.")
        #     elif not (type(resolution) is int):
        #         print("INVALID. Enter integer >= 1.")
        #     else:
        #         break

        # mod_dim = np.int16((map_width,map_height)/resolution)

        # while True:
		#     robot_radius = input("Enter the robot's radius (0 for point robot): ")
        #     if robot_radius == []:
        #         print("INVALID. Try again.")
        #     else:
        #         break

        # while True:
		#     clearance = input("Enter clearance : ")
        #     if clearance == []:
        #         print("INVALID. Try again.")
        #     else:
        #         break

        # mgrid = draw_map(robot_radius,resolution,clearance)

        # while True:
        #     sx,sy = input(" Enter start node x axis and y axis in form x,y : ")
        #     if sx == [] | sy == []:
        #         print("INVALID. Try again.")
        #     elif not (type(sx) is int) | not (type(sy) is int) | not (type(sx) is float) | not (type(sy) is float):
        #         print("INVALID. Enter integer or float.")
        #     elif check_node_validity(sx,sy,mgrid,mod_dim,robot_radius,clearance):
        #         print("Start node on obstacle or outside boundary. Enter again.")
        #     else:
        #         break

        # while True:
        #     gx,gy = input(" Enter goal node x axis and y axis in form x,y : ")
        #     if gx == [] | gy == []:
        #         print("INVALID. Try again.")
        #     elif not (type(gx) is int) | not (type(gy) is int) | not (type(gx) is float) | not (type(gy) is float):
        #         print("INVALID. Enter integer or float.")
        #     elif check_node_validity(gx,gy,mgrid,mod_dim,robot_radius,clearance):
        #         print("Goal node on obstacle or outside boundary. Enter again.")
        #     else:
        #         break
		
        # while True:
        #     algo = ("Enter algorithm (\"astar\" or \"dijkstra\") : ")
        #     if algo == []:
        #         print("INVALID. Try again.")
        #     elif not (type(resolution) is str):
        #         print("INVALID. Enter string.")
        #     elif (algo != 'dijkstra') | (algo != 'astar'):
        #         print("INVALID. Enter correct terms.")
        #     else:
        #         break
        
		
        # sx = np.int(sx/resolution)
        # sy = np.int(sy/resolution)
        # sid = map_width*sy + sx
        # start = node((sx),(sy),sid,-1,0,0)

        # gx = np.int(gx/resolution)
        # gy = np.int(gy/resolution)
        # gid = map_width*gy + gx
        # goal = node((gx),(gy),gid,-1,1000000,0)

        # plan = PathPlan(start, goal, mgrid, resolution, robot_radius, clearance, 'dijkstra')

        # plan.algorithm()

		# #collision check for start node
		# if not obstacle(nStart,grid,r):
		#     print("Start node invalid")
		#     return False
		    

		# # gx = 249.0
		# # gy = 149.0
		# gx,gy=input("Enter goal node x axis and y axis")
		# nGoal = Node((gx/r),(gy/r),0.0,-1)

		# #Check for collision of goal node
		# if not obstacle(nGoal,grid,r):
		#     print("Goal Node invalid")
		#     return False
		    

		# start = time.time()
		# px,py,explored = Dijkstra(nStart,nGoal,robot_radius=robot_radius,grid=grid,ox=ox,oy=oy,r=r)
		# end = time.time()
		# print("Time taken:",(end-start))
		# path=explored.values()
		# p1=[]
		# p2=[]
		# for i in range(len(path)):
		# 	p1.append(path[i].x)

		# 	p2.append(path[i].y)
		# 	i=i+10

		# if GUI:
		# 	plt.plot(p1,p2,"y*")
		# 	plt.plot(px,py,"k")
			
		# 	plt.show()