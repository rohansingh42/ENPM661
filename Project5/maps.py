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

class Cell:

    def __init__( self, ul ):

        self.ul = ul    # Upper left point
        self.ur = None  # Upper right point
        self.bl = None  # Bottom left point
        self.br = None  # Bottom right point
        self.size = 0


class Map:

    rr = 1
    c = 0
    resolution = 10
    edges = {}

    def __init__(self, rows, columns):

        self.data     = np.zeros((rows,columns))
        self.columns  = columns
        self.rows     = rows
        # self.RSRData  = [ [ self.data[i][j] for j in range(0,columns) ] 
        #                     for i in range(0,rows) ]
        self.RSRData  = np.zeros((rows,columns))
        # self.dirt     = self.dirtList()
        # self.start    = self.startPoint()
        # self.uList    = self.makeUList()
        # self.edges    = []
        # self.jps      = False
        # self.EightWayMove = False

    def updateObstacles(self):
        r = self.rr + self.c
        for i in range(np.int64(-self.columns/2),np.int64(self.columns/2)):
            for j in range(np.int64(-self.rows/2),np.int64(self.rows/2)):
                
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
                b1_3 = x - (5500 - r) < 0
                b1_4 = x - (-5500 + r) > 0

                b1 = not (b1_1 & b1_2 & b1_3 & b1_4)

                collision = b1 | r1 | r2 | r3 | r4 | r5 | r6 | r7 | r8 | r9 | r10 | r11 | r12 | r13 | r14 | c1 | c2 | c3 | c4

                if collision:
                    self.data[np.int64(self.rows/2 + j), np.int64(self.columns/2 + i)] = 7
                    self.RSRData[np.int64(self.rows/2 + j), np.int64(self.columns/2 + i)] = 7

    def RSRDecomposition( self ):
        '''
        Calculates the decomposition needed for
        Rectangular Symmetry Reduction (RSR)
        '''
        Q = []
        cells = []
        for i in range(0,self.rows):
            for j in range(0,self.columns):
                if self.data[i][j] != 7 and self.RSRData[i][j] != 6 :
                    cell = Cell((i,j))
                    self.calcSize( cell )
                    # if cell.size > 0:
                    #     j = cell.br[1]
                    # i_b = cell.ul[0]
                    # j_b = cell.ul[1]
                    # i_e = cell.br[0]
                    # j_e = cell.br[1]
                    # if i_e-i_b == 1 or j_e-j_b == 1:
                    #     return
                    # for i in range(i_b,i_e):
                    #     for j in range(j_b,j_e):
                    #         self.RSRData[i][j] = 6
                    
                    # print(cell.size)
                    if cell.size > 4:
                        self.cellFill( cell )
                        heapq.heappush(Q,(-cell.size,cell))   
                        cells.append(cell)
            print('In row : ',i)

        print(len(cells))
                
        # while Q != []:
        #     a = heapq.heappop(Q)
        #     self.calcSize( a[1] )
        #     # print(a[0])
        #     if a[0] == -a[1].size:
        #         if a[1].br[0] - a[1].ul[0] > 2 and a[1].br[1] - a[1].ul[1] > 2:
        #             self.cellFill( a[1] )
        #             cells.append( a[1] )
        #         print( a[0] )
        #         #print( self.RSRDataToString() )
        #     else:
        #         print('gfhg')
        #         heapq.heappush(Q,(-a[1].size,a[1]))
        for cell in cells:
            print( "size: " + str(cell.size) )
            print( "\tul: " + str(cell.ul) )
            print( "\tbr: " + str(cell.br) )
        # #print( self.toString() )
        self.calcMacroEdges( cells )
        # #print( self.RSRDataToString() )
        # for edge in self.edges:
        #     print( str(edge) + ": " + str(self.edges[edge]))
        # for r in cells:
        #     r.fixEdges( self.edges, self.dirt )
        # #for edge in self.edges:
        # #  print( str(edge) + ": " + str(self.edges[edge]))
        # print( self.RSRDataToString() )
        with open("RSRmap_fast.pkl", "wb") as fp:   #Pickling
            pickle.dump(self.RSRData, fp, protocol = 2)
        with open("RSRedges_fast.pkl", "wb") as fp:   #Pickling
            pickle.dump(self.edges, fp, protocol = 2)
        with open("RSRcells_fast.pkl", "wb") as fp:   #Pickling
            pickle.dump(cells, fp, protocol = 2)

    def cellFill( self, cell ):
        '''
        Takes a cell and fills in the RSRData matrix
        so that those spots are not used for subsequent
        cells.
        '''
        i_b = cell.ul[0]
        j_b = cell.ul[1]
        i_e = cell.br[0]
        j_e = cell.br[1]
        if i_e-i_b == 1 or j_e-j_b == 1:
            return
        for i in range(i_b,i_e+1):
            for j in range(j_b,j_e+1):
                self.RSRData[i][j] = 6


    def calcSize( self, cell ):
        '''
        Calculates the size of the given tile
        for RSR.
        '''
        i_b = cell.ul[0]
        j_b = cell.ul[1]
        cell.br = (0,0)
        size = 0
        i = i_b
        j = j_b
        jlast = j
        depthLimit = self.columns
        blocked = False
        while True:
            j = j_b
            if i == self.rows or self.RSRData[i][j] == 6 or self.RSRData[i][j] == 7:
                y = (i-i_b)
                x = (j-j_b+1)
                z = x*y
                if x > 2 and y > 2 and z > size:
                    cell.ur = (i_b,j)
                    cell.br = (i-1,j)
                    cell.bl = (i-1,j_b)
                    size = z
                break
            
            while True:
                if j == self.columns or j == depthLimit or self.RSRData[i][j] == 6 or self.RSRData[i][j] == 7:
                    y = (i-i_b+1)
                    x = (j-j_b)
                    z = x*y
                    if x > 2 and y > 2 and z > size:
                        cell.ur = (i_b,j-1)
                        cell.br = (i,j-1)
                        cell.bl = (i,j_b)
                        jlast = j
                        size = z
                    depthLimit = j
                    break
                j = j + 1
            i = i + 1

        cell.size = size

    def calcMacroEdges( self, cells ):
        '''
        Takes a list of cells and calculates the macro
        edges required in order to make use of RSR in the
        search.
        '''
        # edges[(x,y,direction)] = ((i,j),length)
        edges = {}
        for cell in cells:
            i_b = cell.ul[0]+1
            j_b = cell.ul[1]+1
            i_e = cell.br[0]-1
            j_e = cell.br[1]-1
            for i in range(i_b,i_e+1):
                for j in range(j_b,j_e+1):
                    self.RSRData[i][j] = 4
            i_b = cell.ul[0]+1
            j_b = cell.ul[1]
            i_e = cell.br[0]-1
            j_e = cell.br[1]
            for i in range(i_b,i_e+1):
                #print( "i: " + str(i) + ",j: " + str(j_b) )
                #edges[(i,j_b)] = ((i,j_e),'e',j_e-j_b)
                edges[((i,j_b),'e')] = (i,j_e)
                #edges[(i,j_e)] = ((i,j_b),'w',j_e-j_b)
                edges[((i,j_e),'w')] = (i,j_b)
                #print( str(edges[(i,j_b)]) )

            i_b = cell.ul[0]
            j_b = cell.ul[1]+1
            i_e = cell.br[0]
            j_e = cell.br[1]-1
            for j in range(j_b,j_e+1):
                #print( str((i_b,j)) )
                #edges[(i_b,j)] = ((i_e,j),'s',i_e-i_b)
                edges[((i_b,j),'s')] = (i_e,j)
                #edges[(i_e,j)] = ((i_b,j),'n',i_e-i_b)
                edges[((i_e,j),'n')] = (i_b,j)
                #print( str(edges[(i_b,j)]) )
        #print( str(len(edges)) )
        #print( str(edges) )
        self.edges = edges

m = Map(1010,1100)

m.updateObstacles()

print('Obstacles added.')

m.RSRDecomposition()

print('RSR done.')

# with open("RSRmap.pkl", "wb") as fp:   #Pickling
#     pickle.dump(m.RSRData, fp, protocol = 2)