Skip to content
 
Search or jump to…

Pull requests
Issues
Marketplace
Explore
 
@rohansingh42 
1
0 0 jeffreypicard/cs730_final_project
 Code  Issues 0  Pull requests 0  Projects 0  Wiki  Insights
cs730_final_project/World.py
@jpicardnh jpicardnh Added calc_stats.c to calculate the average speedup vs path length. A…
5f2a68c on May 8, 2012
581 lines (542 sloc)  18 KB
  
'''
World.py
Code for a 'World' representation for A* search.
Author: Jeffrey Picard
'''

from State import State
from Room import Room
import heapq
import math

class World:
  '''
  Class to represent the static world.
  This is shared between all the states.
  '''

  def __init__(self, data, columns, rows):
    '''
    Constructor
    '''
    self.data     = data
    self.columns  = columns
    self.rows     = rows
    self.RSRData  = [ [ data[i][j] for j in range(0,columns) ] 
                      for i in range(0,rows) ]
    self.dirt     = self.dirtList()
    self.start    = self.startPoint()
    self.uList    = self.makeUList()
    self.edges    = []
    self.jps      = False
    self.EightWayMove = False
  
  def validWorld( self ):
    '''
    Checks to see if the world is valid.
    '''
    return self.start and self.dirt

  def blocked(self, loc):
    '''
    Returns true or false about whether the position
    is blocked.
    '''
    return self.data[loc[0]][loc[1]] == '#'

  def getUList(self):
    '''
    '''
    return self.uList

  def makeUList(self):
    '''
    '''
    l = []
    for i in range(0,self.rows):
      for j in range(0,self.columns):
        if self.data[i][j] == '*':
          l.append('_')
        else:
          l.append(self.data[i][j])
    return l


  def toString(self,newLine=True):
    '''
    Returns a string representation of the world
    '''
    string = ""
    for i in range(0,self.rows):
      for j in range(0,self.columns):
        string = string + self.data[i][j]
      if newLine:
        string = string + '\n'
    return string

  def RSRDataToString(self,newLine=True):
    '''
    Returns a string representation of the world
    '''
    string = ""
    for i in range(0,self.rows):
      for j in range(0,self.columns):
        string = string + self.RSRData[i][j]
      if newLine:
        string = string + '\n'
    return string


  def dirtList(self):
    '''
    Returns a list of tuples of the locations of
    all the dirt in the world in the form (x,y,0).
    The zero is used in the State class to indicate
    that dirt has not yet been cleaned up.
    '''
    l = []
    for i in range(0,self.rows):
      for j in range(0,self.columns):
        if self.data[i][j] == '*':
          l.append((i,j))
    return l

  def startPoint(self):
    '''
    Return the starting point as a tuple in the form
    (x,y)
    '''
    for i in range(0,self.rows):
      for j in range(0,self.columns):
        if self.data[i][j] == '@':
          return (i,j)
  
  def getState(self):
    '''
    Returns an initial state for the world
    '''
    state = State(self.dirtList,self.start)

  def RSRDecomposition( self ):
    '''
    Calculates the decomposition needed for
    Rectangular Symmetry Reduction (RSR)
    '''
    Q = []
    rooms = []
    for i in range(0,self.rows):
      for j in range(0,self.columns):
        if self.data[i][j] != '#':
          room = Room((i,j))
          self.calcPriority( room )
          heapq.heappush(Q,(-room.size,room))
    while Q != []:
      a = heapq.heappop(Q)
      self.calcPriority( a[1] )
      if a[0] == -a[1].size:
        if a[1].br[0] - a[1].ul[0] > 2 and a[1].br[1] - a[1].ul[1] > 2:
          self.roomFill( a[1] )
          rooms.append( a[1] )
        #print( a[0] )
        #print( self.RSRDataToString() )
      else:
        heapq.heappush(Q,(-a[1].size,a[1]))
    #for room in rooms:
    #  print( "size: " + str(room.size) )
    #  print( "\tul: " + str(room.ul) )
    #  print ( "\tbr: " + str(room.br) )
    #print( self.toString() )
    self.calcMacroEdges( rooms )
    #print( self.RSRDataToString() )
    #for edge in self.edges:
    #  print( str(edge) + ": " + str(self.edges[edge]))
    for r in rooms:
      r.fixEdges( self.edges, self.dirt )
    #for edge in self.edges:
    #  print( str(edge) + ": " + str(self.edges[edge]))
    print( self.RSRDataToString() )

  def calcPriority( self, room ):
    '''
    Calculates the priority of the given tile
    for RSR.
    '''
    i_b = room.ul[0]
    j_b = room.ul[1]
    room.br = (0,0)
    priority = 0
    i = i_b
    j = j_b
    depthLimit = self.columns
    blocked = False
    while True:
        j = j_b
        if i == self.rows or self.RSRData[i][j] == '#' or self.RSRData[i][j] == '+':
            x = (i-i_b)
            y = (j-j_b+1)
            z = x*y
            if x > 2 and y > 2 and z > priority:
                room.ur = (j_b,i)
                room.br = (i,j+1)
                room.bl = (i_b,j+1)
                priority = z
            break
        while True:
            if j == self.columns or j == depthLimit or self.data[i][j] == '#' or self.RSRData[i][j] == '+':
                x = (i-i_b+1)
                y = (j-j_b)
                z = x*y
                if x > 2 and y > 2 and z > priority:
                    room.ur = (j_b,i+1)
                    room.br = (i+1,j)
                    room.bl = (i_b,j)
                    priority = z
                depthLimit = j
                break
            j = j + 1
        i = i + 1

    #if priority == 27:
    #  print( "i="+str(i_b)+",j="+str(j_b) )
    room.size = priority

  def calcMacroEdges( self, rooms ):
    '''
    Takes a list of rooms and calculates the macro
    edges required in order to make use of RSR in the
    search.
    '''
    # edges[(x,y,direction)] = ((i,j),length)
    edges = {}
    for room in rooms:
      i_b = room.ul[0]+1
      j_b = room.ul[1]+1
      i_e = room.br[0]-1
      j_e = room.br[1]-1
      for i in range(i_b,i_e):
        for j in range(j_b,j_e):
          self.RSRData[i][j] = '!'
      i_b = room.ul[0]+1
      j_b = room.ul[1]
      i_e = room.br[0]-1
      j_e = room.br[1]-1
      for i in range(i_b,i_e):
        #print( "i: " + str(i) + ",j: " + str(j_b) )
        #edges[(i,j_b)] = ((i,j_e),'e',j_e-j_b)
        edges[((i,j_b),'e')] = ((i,j_e),j_e-j_b)
        #edges[(i,j_e)] = ((i,j_b),'w',j_e-j_b)
        edges[((i,j_e),'w')] = ((i,j_b),j_e-j_b)
        #print( str(edges[(i,j_b)]) )

      i_b = room.ul[0]
      j_b = room.ul[1]+1
      i_e = room.br[0]-1
      j_e = room.br[1]-1
      for j in range(j_b,j_e):
        #print( str((i_b,j)) )
        #edges[(i_b,j)] = ((i_e,j),'s',i_e-i_b)
        edges[((i_b,j),'s')] = ((i_e,j),i_e-i_b)
        #edges[(i_e,j)] = ((i_b,j),'n',i_e-i_b)
        edges[((i_e,j),'n')] = ((i_b,j),i_e-i_b)
        #print( str(edges[(i_b,j)]) )
    #print( str(len(edges)) )
    #print( str(edges) )
    self.edges = edges

  def roomFill( self, room ):
    '''
    Takes a room and fills in the RSRData matrix
    so that those spots are not used for subsequent
    rooms.
    '''
    i_b = room.ul[0]
    j_b = room.ul[1]
    i_e = room.br[0]
    j_e = room.br[1]
    if i_e-i_b == 1 or j_e-j_b == 1:
      return
    for i in range(i_b,i_e):
      for j in range(j_b,j_e):
        self.RSRData[i][j] = '+'

  def identifySuccessors( self, stateNode, start, goals, parentDis ):
    '''
    Takes a node and identifies its successor nodes.
    '''
    successors = []
    curLoc = stateNode.curLoc
    node = (curLoc[0],curLoc[1],stateNode.actions,0)
    #nList = self.prune( node, self.neighbors( node ) )
    nList = self.pruneNeighbors( node, parentDis )
    #nList = self.neighbors( node )
    for n in nList:
      #print("Jumping")
      n = self.jump( node, n[2], start, goals )
      if n:
        successors.append( n )
    return successors


  def pruneNeighbors( self, node, parentDis ):
    '''
    Takes a node and returns its pruned neighbors.
    '''
    n   = (node[0]-1,node[1],'N') # North
    nw  = (node[0]-1,node[1]-1,'NW')
    ne  = (node[0]-1,node[1]+1,'NE')
    s   = (node[0]+1,node[1],'S') # South
    sw  = (node[0]+1,node[1]-1,'SW')
    se  = (node[0]+1,node[1]+1,'SE')
    w   = (node[0],node[1]-1,'W') # West
    e   = (node[0],node[1]+1,'E') # East
    neighbors = []

    if node[2] == [] or node[2][len(node[2])-1] == 'V':
      if n[0] >= 0 and not self.blocked( n ):
        neighbors.append( n )
      if nw[0] >= 0 and nw[1] >= 0 and not self.blocked( nw ):
        neighbors.append( nw )
      if ne[0] >= 0 and ne[1] < self.columns and not self.blocked( ne ):
        neighbors.append( ne )
      if s[0] < self.rows and not self.blocked( s ):
        neighbors.append( s )
      if sw[0] < self.rows and sw[1] >= 0 and not self.blocked( sw ):
        neighbors.append( sw )
      if se[0] < self.rows and se[1] < self.columns and not self.blocked( se ):
        neighbors.append( se )
      if w[1] >= 0 and not self.blocked( w ):
        neighbors.append( w )
      if e[1] < self.columns and not self.blocked( e ):
        neighbors.append( e )
    else:
      lastMove = node[2][len(node[2])-1]
      if lastMove == 'V':
        lastMove = node[2][len(node[2])-2]
      #print("lastMove: " + lastMove )

      if lastMove == 'N':
        if n[0] >= 0 and not self.blocked( n ):
          neighbors.append( n )
        if w[1] >= 0 and self.blocked( w ):
          if nw[0] >= 0 and nw[1] >= 0 and not self.blocked( nw ):
            neighbors.append( nw )
        if e[1] < self.columns and self.blocked( e ):
          if ne[0] >= 0 and ne[1] < self.columns and not self.blocked( ne ):
            neighbors.append( ne )
      elif lastMove == 'NW':
        if nw[0] >= 0 and nw[1] >= 0 and not self.blocked( nw ):
          neighbors.append( nw )
        if n[0] >= 0 and not self.blocked( n ):
          neighbors.append( n )
        if w[1] >= 0 and not self.blocked( w ):
          neighbors.append( w )
        if s[0] < self.rows and self.blocked( s ):
          if sw[0] < self.rows and sw[1] >= 0 and not self.blocked( sw ):
            neighbors.append( sw )
        if e[1] < self.columns and self.blocked( e ):
          if ne[0] >= 0 and ne[1] < self.columns and not self.blocked( ne ):
            neighbors.append( ne )
      elif lastMove == 'NE':
        if ne[0] >= 0 and ne[1] < self.columns and not self.blocked( ne ):
          neighbors.append( ne )
        if n[0] >= 0 and not self.blocked( n ):
          neighbors.append( n )
        if e[1] < self.columns and not self.blocked( e ):
          neighbors.append( e )
        if s[0] < self.rows and self.blocked( s ):
          if se[0] < self.rows and se[1] < self.columns and not self.blocked( se ):
            neighbors.append( se )
        if w[1] >= 0 and self.blocked( w ):
          if nw[0] >= 0 and nw[1] >= 0 and not self.blocked( nw ):
            neighbors.append( nw )
      elif lastMove == 'S':
        if s[0] < self.rows and not self.blocked( s ):
          neighbors.append( s )
        if w[1] >= 0 and self.blocked( w ):
          if sw[0] < self.rows and sw[1] >= 0 and not self.blocked( sw ):
            neighbors.append( sw )
        if e[1] < self.columns and self.blocked( e ):
          if se[0] < self.rows and se[1] < self.columns and not self.blocked( se ):
            neighbors.append( se )
      elif lastMove == 'SW':
        if sw[0] < self.rows and sw[1] >= 0 and not self.blocked( sw ):
          neighbors.append( sw )
        if s[0] < self.rows and not self.blocked( s ):
          neighbors.append( s )
        if w[1] >= 0 and not self.blocked( w ):
          neighbors.append( w )
        if n[0] >= 0 and self.blocked( n ):
          if nw[0] >= 0 and nw[1] >= 0 and not self.blocked( nw ):
            neighbors.append( nw )
        if e[1] < self.columns and self.blocked( e ):
          if se[0] < self.rows and se[1] < self.columns and not self.blocked( se ):
            neighbors.append( se )
      elif lastMove == 'SE':
        if se[0] < self.rows and se[1] < self.columns and not self.blocked( se ):
          neighbors.append( se )
        if s[0] < self.rows and not self.blocked( s ):
          neighbors.append( s )
        if e[1] < self.columns and not self.blocked( e ):
          neighbors.append( e )
        if n[0] >= 0 and self.blocked( n ):
          if ne[0] >= 0 and ne[1] < self.columns and not self.blocked( ne ):
            neighbors.append( ne )
        if w[1] >= 0 and self.blocked( w ):
          if sw[0] < self.rows and sw[1] >= 0 and not self.blocked( sw ):
            neighbors.append( sw )
      elif lastMove == 'W':
        if w[1] >= 0 and not self.blocked( w ):
          neighbors.append( w )
        if s[0] < self.rows and self.blocked( s ):
          if sw[0] < self.rows and sw[1] >= 0 and not self.blocked( sw ):
            neighbors.append( sw )
        if n[0] >= 0 and self.blocked( n ):
          if nw[0] >= 0 and nw[1] >= 0 and not self.blocked( nw ):
            neighbors.append( nw )
      elif lastMove == 'E':
        if e[1] < self.columns and not self.blocked( e ):
          neighbors.append( e )
        if s[0] < self.rows and self.blocked( s ):
          if se[0] < self.rows and se[1] < self.columns and not self.blocked( se ):
            neighbors.append( se )
        if n[0] >= 0 and self.blocked( n ):
          if ne[0] >= 0 and ne[1] < self.columns and not self.blocked( ne ):
            neighbors.append( ne )
      else:
        print("Error: bad lastMove")
        exit(1)

    return neighbors

  def neighbors( self, node ):
    '''
    Takes a node and returns list of its
    valid neighbors.
    '''
    n1 = (node[0]-1,node[1],[ x for x in node[2] ], node[3]+1 ) # North
    n2 = (node[0]+1,node[1],[ x for x in node[2] ], node[3]+1 ) # South
    n3 = (node[0],node[1]-1,[ x for x in node[2] ], node[3]+1 ) # West
    n4 = (node[0],node[1]+1,[ x for x in node[2] ], node[3]+1 ) # East

    neighbors = []
    if n1[0] >= 0 and not self.blocked( n1 ):
      n1[2].append('N')
      neighbors.append( n1 )
    if n2[0] < self.rows and not self.blocked( n2 ):
      n2[2].append('S')
      neighbors.append( n2 )
    if n3[1] >= 0 and not self.blocked( n3 ):
      n3[2].append('W')
      neighbors.append( n3 )
    if n4[1] < self.columns and not self.blocked( n4 ):
      n4[2].append('E')
      neighbors.append( n4 )

    return neighbors

  def direction( self, node, n ):
    '''
    Takes a node and a neighbor and returns
    the direction of travel to get to that
    neighbor.
    '''

  def step( self, node, direction ):
    '''
    Takes a node and a direction of travel and returns
    the node at that location.
    '''
    #direction = actionList[ len(actionList)-1 ]

    if direction == 'N':
      n = (node[0]-1,node[1],[ x for x in node[2] ], node[3]+1 )
      if n[0] >= 0 and not self.blocked( n ):
        n[2].append('N')
        return n
    elif direction == 'NW':
      nw = (node[0]-1,node[1]-1,[x for x in node[2] ], node[3]+math.sqrt(2) )
      if nw[0] >= 0 and nw[1] >= 0 and not self.blocked( nw ):
        nw[2].append('NW')
        return nw
    elif direction == 'NE':
      ne = (node[0]-1,node[1]+1,[x for x in node[2] ], node[3]+math.sqrt(2) )
      if ne[0] >= 0 and ne[1] < self.columns and not self.blocked( ne ):
        ne[2].append('NE') 
        return ne
    elif direction == 'S':
      n = (node[0]+1,node[1],[ x for x in node[2] ], node[3]+1 )
      if n[0] < self.rows and not self.blocked( n ):
        n[2].append('S')
        return n
    elif direction == 'SW':
      sw = (node[0]+1,node[1]-1,[x for x in node[2]], node[3]+math.sqrt(2) )
      if sw[0] < self.rows and sw[1] >= 0 and not self.blocked( sw ):
        sw[2].append('SW')
        return sw
    elif direction == 'SE':
      se = (node[0]+1,node[1]+1,[x for x in node[2]], node[3]+math.sqrt(2) )
      if se[0] < self.rows and se[1] < self.columns and not self.blocked( se ):
        se[2].append('SE')
        return se
    elif direction == 'W':
      n = (node[0],node[1]-1,[ x for x in node[2] ], node[3]+1 )
      if n[1] >= 0 and not self.blocked( n ):
        n[2].append('W')
        return n
    elif direction == 'E':
      n = (node[0],node[1]+1,[ x for x in node[2] ], node[3]+1 )
      if n[1] < self.columns and not self.blocked( n ):
        n[2].append('E')
        return n
    else:
      print("Error: in step unknown direction " + direction)
      exit(1)
    return None

  def forcedNeighbor( self, node, direction ):
    '''
    Takes a node and return a boolean as to whether
    it has a neighbor which is forced.
    '''
    #print("forcedNeighbor")
    #print( len(self.pruneNeighbors(node)) )
    ns = self.pruneNeighbors( node, 1 )
    #print( str(ns) )
    if direction == 'N' or direction == 'S' or direction == 'E' or direction == 'W':
      for n in ns:
        if n[2] != direction:
          #print("forcedNeighbor: " + n[2] )
          return True
    elif direction == 'NW':
      for n in ns:
        nD = n[2]
        if nD != direction and nD != 'N' and nD != 'W':
          #print("forcedNeighbor: " + n[2] )
          return True
    elif direction == 'NE':
      for n in ns:
        nD = n[2]
        if nD != direction and nD != 'N' and nD != 'E':
          #print("forcedNeighbor: " + n[2] )
          return True
    elif direction == 'SW':
      for n in ns:
        nD = n[2]
        if nD != direction and nD != 'S' and nD != 'W':
          #print("forcedNeighbor: " + n[2] )
          return True
    elif direction == 'SE':
      for n in ns:
        nD = n[2]
        if nD != direction and nD != 'S' and nD != 'E':
          #print("forcedNeighbor: " + n[2] )
          return True
    return False
    #return len(self.pruneNeighbors( node, 1 )) > 0

  def getDi( self, direction ):
    '''
    Takes a diagonal direction and return d1 and d2
    that are the two straight moves at 45% from
    the diagonal move. Returns an empty list
    if the move is straight.
    '''
    if direction == 'N' or direction == 'S' or direction == 'E' or direction == 'W':
      return ()
    elif direction == 'NW':
      return ('N','W')
    elif direction == 'NE':
      return ('N','E')
    elif direction == 'SW':
      return ('S','W')
    elif direction == 'SE':
      return ('S','E')
    else:
      print("Error: in getDi bad direction " + direction )
      exit(1)


  def jump( self, node, direction, start, goalList ):
    '''
    Takes a node and a direction and identifies the jump
    point recursively.
    '''
    #else:
      #print("Not goal")
      #print( str(node) )
    n = self.step( node, direction )
    #print(str(n))
    if not n:
      return None
    if (n[0],n[1]) in goalList:
      return n
    if self.forcedNeighbor( n, direction ):
      return n
    for di in self.getDi( direction ):
      if self.jump( n, di, start, goalList ):
        return n
    return self.jump( n, n[2][ len(n[2])-1 ], start, goalList )
© 2019 GitHub, Inc.
Terms
Privacy
Security
Status
Help
Contact GitHub
Pricing
API
Training
Blog
About
