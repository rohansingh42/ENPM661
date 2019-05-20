import math, time, heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

class Node:
    def __init__(self, x, y, parentid):#,stepid, g, d = 0):
        self.x = np.int32(np.round(x, 0))
        self.y = np.int32(np.round(y, 0))
        self.id = 'x' + str(self.x) + 'y' + str(self.y)# + str(self.theta)
        self.parentid = parentid


    def __str__(self):
        return "x : " + str(self.x) + " y : " + str(self.y) + " id : " + str(self.id) + ", parentid : " + str(self.parentid) #+ ", h : " + str(self.h) + ", g : " + str(self.g) + ", d : " + str(self.d)

class JPS():
    resolution=100.0
    mod_dim = np.int64(np.array((11000,10100))/resolution)
    fig, ax = plt.subplots(1, 1, tight_layout=True)
    cmap = matplotlib.colors.ListedColormap(['w', 'g', 'b', 'm', 'y', 'r', 'k'])
    canvas = ax.figure.canvas
    data     = np.zeros((int(10100/resolution),int(11100/resolution)))
    columns  = 11000/resolution
    rows     = 10100/resolution
    def __init__(self, start,goal):
        self.start = start
        self.goal = goal
        self.map_grid = np.zeros((self.mod_dim[1],self.mod_dim[0]))

    def updateObstacles(self):
        r = 1
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

                r6_1 = y - (-1905 - r) > 0
                r6_2 = y - (-75 + r) < 0
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

                if collision:
                    self.map_grid[np.int64(self.mod_dim[1]/2 + j), np.int64(self.mod_dim[0]/2 + i)] = 6


    def heuristic(self,a, b, hchoice):
        a = [a.x,a.y]
        b = [b.x,b.y]
        if hchoice == 1:
            xdist = math.fabs(b[0] - a[0])
            ydist = math.fabs(b[1] - a[1])
            if xdist > ydist:
                return 14*ydist+10*(xdist-ydist)
            else:
                return 14*xdist+10*(ydist-xdist)
        if hchoice == 2:
            return 100*math.sqrt((b[0]-a[0])**2+(b[1]-a[1])**2)

    def blocked(self,cX,cY,dX,dY):
        r=1
        x = (cX+dX)
        y = (cY+dY)
        # print("x and y",x,y)
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

        r6_1 = y - (-1905 - r) > 0
        r6_2 = y - (-75 + r) < 0
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

        return collision



    def dblock(self,cX,cY,dX,dY):
        if self.blocked(cX, cY, -dX, 0)==True and self.blocked(cX, cY, 0, -dY)==True:
            # if matrix[cX-dX][cY] == 1 and matrix[cX][cY-dY] == 1:
            return True
        else:
            return False

    def direction(self,cX,cY,pX,pY):
        dX = int(math.copysign(100,cX - pX))
        dY = int(math.copysign(100,cY - pY))
        if cX-pX == 0:
            dX = 0
        if cY-pY == 0:
            dY = 0
        return(dX,dY)

    def nodeNeighbours(self,cX,cY,parent):
        neighbours = []
        if type(parent) != tuple:
            for i,j in [(-100,0),(0,-100),(100,0),(0,100),(-100,-100),(-100,100),(100,-100),(100,100)]:
                if not self.blocked(cX,cY,i,j):
                    neighbours.append((cX+i,cY+j))

            return neighbours
        dX,dY = self.direction(cX,cY,parent[0],parent[1])

        if dX != 0 and dY != 0:
            if not self.blocked(cX,cY,0,dY):
                neighbours.append((cX,cY+dY))
            if not self.blocked(cX,cY,dX,0):
                neighbours.append((cX+dX,cY))
            if ((not self.blocked(cX,cY,0,dY) or
                not self.blocked(cX,cY,dX,0)) and
                not self.blocked(cX,cY,dX,dY)):
                neighbours.append((cX+dX,cY+dY))
            if (self.blocked(cX,cY,-dX,0) and
                not self.blocked(cX,cY,0,dY)):
                neighbours.append((cX-dX,cY+dY))
            if (self.blocked(cX,cY,0,-dY) and
                not self.blocked(cX,cY,dX,0)):
                neighbours.append((cX+dX,cY-dY))

        else:
            if dX == 0:
                if not self.blocked(cX,cY,dX,0):
                    if not self.blocked(cX,cY,0,dY):
                        neighbours.append((cX,cY+dY))
                    if self.blocked(cX,cY,100,0):
                        neighbours.append((cX+100,cY+dY))
                    if self.blocked(cX,cY,-100,0):
                        neighbours.append((cX-100,cY+dY))

            else:
                if not self.blocked(cX,cY,dX,0):
                    if not self.blocked(cX,cY,dX,0):
                        neighbours.append((cX+dX,cY))
                    if self.blocked(cX,cY,0,100):
                        neighbours.append((cX+dX,cY+100))
                    if self.blocked(cX,cY,0,-100):
                        neighbours.append((cX+dX,cY-100))
        return neighbours

    def jump(self,cX,cY,dX,dY,goal):

        nX = cX + dX
        nY = cY + dY
        if self.blocked(nX,nY,0,0):
            return None

        if (nX,nY) == (goal.x,goal.y):
            return(nX,nY)

        oX = nX
        oY = nY

        if dX !=0 and dY != 0:
            while True:
                if (not self.blocked(oX,oY,-dX,dY) and
                    self.blocked(oX,oY,-dX,0) or
                    not self.blocked(oX,oY,dX,-dY) and
                    self.blocked(oX,oY,0,-dY)):
                    return(oX,oY)

                if (self.jump(oX,oY,dX,0,goal) != None or
                    self.jump(oX,oY,0,dY,goal) != None):
                    return(oX,oY)

                oX += dX
                oY += dY

                if self.blocked(oX,oY,0,0):
                    return None

                if self.dblock(oX,oY,dX,dY):
                    return None

                if (oX,oY) == (goal.x,goal.y):
                    return(oX,oY)
        else:
            if dX != 0:
                while True:
                    if (not self.blocked(oX,nY,dX,100) and
                        self.blocked(oX,nY,0,100) or
                        not self.blocked(oX,nY,dX,-100) and
                        self.blocked(oX,nY,0,-100)):
                        return(oX,nY)

                    oX += dX

                    if self.blocked(oX,nY,0,0):
                        return None

                    if (oX,nY) == (goal.x,goal.y):
                        return(oX,nY)

            else:
                while True:
                    if (not self.blocked(nX,oY,100,dY) and
                        self.blocked(nX,oY,100,0) or
                        not self.blocked(nX,oY,-100,dY) and
                        self.blocked(nX,oY,-100,0)):
                        return(nX,oY)

                    oY += dY

                    if self.blocked(nX,oY,0,0):
                        return None

                    if (nX,oY) == (goal.x,goal.y):
                        return (nX,oY)

        return self.jump(nX,nY,dX,dY,goal)

    def identifySuccessors(self,cX,cY,came_from,goal):
        successors = []
        neighbours = self.nodeNeighbours(cX,cY,came_from.get((cX,cY),0))

        for cell in neighbours:
            dX = cell[0]-cX
            dY = cell[1]-cY

            jumpPoint = self.jump(cX,cY,dX,dY,goal)

            if jumpPoint != None:
                successors.append(jumpPoint)

        return successors



    def method(self,start,goal,hchoice):
        self.updateObstacles()
        came_from = dict()
        # close_set = set()
        close_set = dict()
        gscore = {start.id:0}
        fscore = {start.id:self.heuristic(start,goal,hchoice)}

        pqueue = []

        heapq.heappush(pqueue, (fscore[start.id], start))
        print(pqueue)
        starttime = time.time()


        #Plotting
        obs = self.ax.pcolormesh(self.map_grid, cmap=self.cmap)
        plt.xlim(0,self.mod_dim[0])
        plt.ylim(0,self.mod_dim[1])

        while pqueue:

            current = heapq.heappop(pqueue)[1]
            # print(pqueue)
            print("################",current.id,current.x)
            print("################",goal.id)
            if current.id == goal.id:
                print("came_from",came_from)
                print("close",len(close_set))
                cid = goal.id
                data_x = []
                data_y = []
                close_set[current.id] = current
                while cid != start.id:
                    data_x.append(self.mod_dim[0]/2+close_set[cid].x/100)
                    data_y.append(self.mod_dim[1]/2+close_set[cid].y/100)
                    cid = close_set[cid].parentid

                data_x.append(self.mod_dim[0]/2 + start.x/100)
                data_y.append(self.mod_dim[1]/2 + start.y/100)
                plt.plot(data_x,data_y,'go--', linewidth=2, markersize=3)
                path = [data_x,data_y]
                plt.show()
                endtime = time.time()
                print("Goal found",gscore[goal.id])
                return (path, round(endtime-starttime,6))
            # print(current)
            if current.id in close_set:
                continue
            else:
                close_set[current.id] = current

            successors = self.identifySuccessors(current.x,current.y,
                                            came_from,goal)

            for successor in successors:
                jumpPoint = successor
                jumpPoint = Node(jumpPoint[0],jumpPoint[1],current.id)
                if jumpPoint in close_set: #and tentative_g_score >= gscore.get(jumpPoint,0):
                    continue
                tentative_g_score = gscore[current.id] + self.lenght(current,jumpPoint,hchoice)

                # (tentative_g_score < gscore[close_set[jumpPoint.id].id] or
                if jumpPoint not in [j[1]for j in pqueue]:
                    came_from[jumpPoint.id] = current
                    gscore[jumpPoint.id] = tentative_g_score
                    # print(jumpPoint)
                    fscore[jumpPoint.id] = tentative_g_score + self.heuristic(jumpPoint,goal,hchoice)
                    heapq.heappush(pqueue, (fscore[jumpPoint.id], jumpPoint))

        endtime = time.time()
        return (0,round(endtime-starttime,6))

    def lenght(self,current,jumppoint,hchoice):
        current = [current.x,current.y]
        jumppoint = [jumppoint.x,jumppoint.y]
        dX,dY = self.direction(current[0],current[1],jumppoint[0],jumppoint[1])
        dX = math.fabs(dX)
        dY = math.fabs(dY)
        lX = math.fabs(current[0]-jumppoint[0])
        lY = math.fabs(current[1]-jumppoint[1])
        if hchoice == 1:
            if dX !=0 and dY !=0:
                lenght = lX*14
                return lenght
            else:
                lenght = (dX*lX+dY*lY)*10
                return lenght
        if hchoice == 2:
            return math.sqrt((current[0]-jumppoint[0])**2+(current[1]-jumppoint[1])**2)

def main():
    sx,sy = input("Enter x-y coordinates of start node: ")
    gx,gy = input("Enter x-y coordinates of goal node: ")
    # sx = -4000
    # sy = -4000
    # gx= 4000
    # gy= 4000
    start = Node(int(sx),int(sy),-1)#,0,0,0,math.sqrt((sx-gx)**2 + (sy-gy)**2))
    goal = Node(gx,gy,-1)#,-1,0,1000000,0)
    plan = JPS(start,goal)
    plan,time = plan.method(start,goal,2 )
    print(plan)
    print(time)
if __name__ == '__main__':
    main()
