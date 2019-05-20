import matplotlib.pyplot as plt 
from matplotlib.path import Path 
import matplotlib.patches as patches
from collections import deque
import heapq as hq
import math

def in_obstacle_space(posi_x,posi_y):
	# for the square
  if (posi_x-55 > 0 or posi_x-55 == 0) and (posi_y-67 > 0 or posi_y-67 == 0) and (posi_y-112 < 0 or posi_y-112 == 0) and (posi_x-105 < 0 or posi_x-105 == 0):
    result_square = True
  else:
  	result_square = False
  
  # for circle
  if ((posi_x - 180)**2 + (posi_y - 120)**2 - (15)**2) < 0 or ((posi_x - 180)**2 + (posi_y - 120)**2 - (15)**2) == 0:
  	result_circle = True
  else:
  	result_circle = False

  # for 6 sided polygon
  #if ((((-41/25)*posi_x + (1259/5) - posi_y) < 0 or ((-41/25)*posi_x + (1259/5) - posi_y) == 0) and (((37/13)*posi_x-(5183/13)-posi_y) < 0 or ((37/13)*posi_x-(5183/13)-posi_y) == 0) and (((-2/19)*posi_x + (1285/19) - posi_y) > 0 or ((-2/19)*posi_x + (1285/19) - posi_y) == 0)) or ((((38/7)*posi_x - (5647/7) - posi_y) > 0 or ((38/7)*posi_x - (5647/7) - posi_y) == 0) and (((-38/23)*posi_x + (8317/23) - posi_y) > 0 or ((-38/23)*posi_x + (8317/23) - posi_y) == 0) and ((51 - posi_y) < 0 or (51 - posi_y) == 0)) or ((((37/20)*posi_x - (1484/5) - posi_y) < 0 or ((37/20)*posi_x - (1484/5) - posi_y) == 0) and ((14 - posi_y) < 0 or (14 - posi_y) == 0) and ((51 - posi_y) > 0 or (51 - posi_y == 0)) and (((37/13)*posi_x-(5183/13)-posi_y) > 0) or ((37/13)*posi_x-(5183/13)-posi_y) == 0):
  if path2.contains_point((posi_x,posi_y)):
  	result_six_poly = True
  else:
  	result_six_poly = False
  
  result = result_square or result_circle or result_six_poly
  return result


def h(x1, x2, y1, y2):
  #return max(abs(x1 - x2),abs(y1 - y2)) # Diagonal distance
  #return abs(x1-x2) + abs(y1-y2) # Manhatten distance
  return math.sqrt(((x1 - x2)**2) + ((y1 - y2)**2)) # Euclidean distance


# START AND GOAL POSITION
sx = 10
sy = 140
gx = 200
gy = 20

# setting up the arena space
verts1 = [
  (55, 67),  # left, bottom
  (55, 112),  # left, top
  (105, 112),  # right, top
  (105, 67),  # right, bottom
  (55, 67),  # ignored
]

codes1 = [
  Path.MOVETO,
  Path.LINETO,
  Path.LINETO,
  Path.LINETO,
  Path.CLOSEPOLY,
]

verts2 = [
  (120, 55),  
  (158, 51),  
  (165, 89),  
  (188, 51),  
  (168, 14),
  (145, 14),
  (120,55),  
]

codes2 = [
  Path.MOVETO,
  Path.LINETO,
  Path.LINETO,
  Path.LINETO,
  Path.LINETO,
  Path.LINETO,
  Path.CLOSEPOLY,
]

circle = patches.Circle((180,120),15,facecolor='orange', edgecolor=(0, 0, 0), lw=0)

path1 = Path(verts1, codes1)
path2 = Path(verts2, codes2)
path3 = Path.circle(center=(180,120),radius=15,readonly=True)

fig = plt.figure()
ax = fig.add_subplot(111)
patch1 = patches.PathPatch(path1, facecolor='orange', lw=0)
patch2 = patches.PathPatch(path2, facecolor='orange', lw=0)
patch3 = patches.PathPatch(path3, facecolor='orange', lw=0)

ax.add_patch(patch1)
ax.add_patch(patch2)
ax.add_patch(patch3)

plt.plot(sx, sy, "xr")
plt.plot(gx, gy, "xb")

ax.set_xlim(0, 250)
ax.set_ylim(0, 150)

#############################################
#print ("Entering A star")
#Arena boundaries
Xmin = 0
Xmax = 250
Ymin = 0
Ymax = 150
frontier = []
frontier_temp = deque()
parent = {}
cost_so_far = {}
explored = deque()
current_x = sx
current_y = sy
frontier_temp.append([current_x,current_y])
hq.heappush(frontier,(0,[current_x,current_y]))
cost_so_far[(current_x,current_y)] = 0

i = 0
if (sx == gx and sy == gy):
  print ("Error, goal and start position are the same")
elif in_obstacle_space(sx,sy) or in_obstacle_space(gx,gy):
    print ("Error, goal or/and start position in the obstacle space")
else:
  while (current_x!=gx or current_y!=gy): #while goal is not reached
    current_x, current_y = hq.heappop(frontier)[1]
    frontier_temp.popleft()
    #if not([current_x,current_y] in explored): #and not(contains_pt(current_x,current_y)):
    explored.append([current_x,current_y])
    plt.plot(current_x, current_y, ".c") # Uncomment to plot the explored nodes
    #plt.pause(0.00000000001) # Uncomment to animate the output 
    #adding neighbours to frontier and tracking parent node

    # east movement
    if (current_x+1)<Xmax and not(in_obstacle_space(current_x+1,current_y)) and not([current_x+1,current_y] in explored) and not([current_x+1,current_y] in frontier_temp):
      new_cost = cost_so_far[(current_x,current_y)] + 1 #cost_so_far [(current_x+1,current_y)]
      #print (new_cost)
      cost_so_far[(current_x+1,current_y)] = new_cost
      priority = new_cost + h(current_x+1,gx,current_y,gy)
      hq.heappush(frontier,(priority,(current_x+1,current_y)))
      if (current_x,current_y) in parent:
        parent[(current_x,current_y)].append((current_x+1,current_y))
      else:
        parent[(current_x,current_y)] = [(current_x+1,current_y)]
      frontier_temp.append([current_x+1,current_y])

    # sount-east movement
    if (current_y-1)>Ymin and (current_x+1)<Xmax and not(in_obstacle_space(current_x+1,current_y-1)) and not([current_x+1,current_y-1] in explored) and not([current_x+1,current_y-1] in frontier_temp):
      new_cost = cost_so_far[(current_x,current_y)] + 1 #cost_so_far [(current_x+1,current_y-1)]
      cost_so_far[(current_x+1,current_y-1)] = new_cost
      priority = new_cost + h(current_x+1,gx,current_y-1,gy)
      hq.heappush(frontier,(priority,(current_x+1,current_y-1)))
      if (current_x,current_y) in parent:
        parent[(current_x,current_y)].append((current_x+1,current_y-1))
      else:
        parent[(current_x,current_y)] = [(current_x+1,current_y-1)]
      frontier_temp.append([current_x+1,current_y-1])

    # south movement
    if (current_y-1)>Ymin and not(in_obstacle_space(current_x,(current_y-1))) and not([current_x,current_y-1] in explored) and not([current_x,current_y-1] in frontier_temp):
      new_cost = cost_so_far[(current_x,current_y)] + 1 #cost_so_far [(current_x,current_y-1)]
      cost_so_far[(current_x,current_y-1)] = new_cost
      priority = new_cost + h(current_x,gx,current_y-1,gy)
      hq.heappush(frontier,(priority,(current_x,current_y-1)))
      if (current_x,current_y) in parent:
        parent[(current_x,current_y)].append((current_x,current_y-1))
      else:
        parent[(current_x,current_y)] = [(current_x,current_y-1)]
      frontier_temp.append([current_x,current_y-1])

    # south-west movement
    if (current_y-1)>Ymin and (current_x-1)>Xmin and not(in_obstacle_space(current_x-1,current_y-1)) and not([current_x-1,current_y-1] in explored) and not([current_x-1,current_y-1] in frontier_temp):
      new_cost = cost_so_far[(current_x,current_y)] + 1 #cost_so_far [(current_x-1,current_y-1)]
      cost_so_far[(current_x-1,current_y-1)] = new_cost
      priority = new_cost + h(current_x-1,gx,current_y-1,gy)
      hq.heappush(frontier,(priority,(current_x-1,current_y-1)))
      if (current_x,current_y) in parent:
        parent[(current_x,current_y)].append((current_x-1,current_y-1))
      else:
        parent[(current_x,current_y)] = [(current_x-1,current_y-1)]
      frontier_temp.append([current_x-1,current_y-1])
      
    # west movement
    if (current_x-1)>Xmin and not(in_obstacle_space((current_x-1),current_y)) and not([current_x-1,current_y] in explored) and not([current_x-1,current_y] in frontier_temp):
      new_cost = cost_so_far[(current_x,current_y)] + 1 #cost_so_far [(current_x-1,current_y)]
      cost_so_far[(current_x-1,current_y)] = new_cost
      priority = new_cost + h(current_x-1,gx,current_y,gy)
      hq.heappush(frontier,(priority,(current_x-1,current_y)))
      if (current_x,current_y) in parent:
        parent[(current_x,current_y)].append((current_x-1,current_y))
      else:
        parent[(current_x,current_y)] = [(current_x-1,current_y)]
      frontier_temp.append([current_x-1,current_y])
      
    # north-west movement
    if (current_y+1)<Ymax and (current_x-1)>Xmin and not(in_obstacle_space(current_x-1,current_y+1)) and not([current_x-1,current_y+1] in explored) and not([current_x-1,current_y+1] in frontier_temp):
      new_cost = cost_so_far[(current_x,current_y)] + 1 #cost_so_far [(current_x-1,current_y+1)]
      cost_so_far[(current_x-1,current_y+1)] = new_cost
      priority = new_cost + h(current_x-1,gx,current_y+1,gy)
      hq.heappush(frontier,(priority,(current_x-1,current_y+1)))
      if (current_x,current_y) in parent:
        parent[(current_x,current_y)].append((current_x-1,current_y+1))
      else:
        parent[(current_x,current_y)] = [(current_x-1,current_y+1)]
      frontier_temp.append([current_x-1,current_y+1])
      
    # north movement
    if (current_y+1)<Ymax and not(in_obstacle_space(current_x,(current_y+1))) and not([current_x,current_y+1] in explored) and not([current_x,current_y+1] in frontier_temp):
      new_cost = cost_so_far[(current_x,current_y)] + 1 #cost_so_far [(current_x,current_y+1)]
      cost_so_far[(current_x,current_y+1)] = new_cost
      priority = new_cost + h(current_x,gx,current_y+1,gy)
      hq.heappush(frontier,(priority,(current_x,current_y+1)))
      if (current_x,current_y) in parent:
        parent[(current_x,current_y)].append((current_x,current_y+1))
      else:
        parent[(current_x,current_y)] = [(current_x,current_y+1)]
      frontier_temp.append([current_x,current_y+1])
      
    # north-east movement
    if (current_y+1)<Ymax and (current_x+1)<Xmax and not(in_obstacle_space(current_x+1,current_y+1)) and not([current_x+1,current_y+1] in explored) and not([current_x+1,current_y+1] in frontier_temp):
      new_cost = cost_so_far[(current_x,current_y)] + 1 #cost_so_far [(current_x+1,current_y+1)]
      cost_so_far[(current_x+1,current_y+1)] = new_cost
      priority = new_cost + h(current_x+1,gx,current_y+1,gy)
      hq.heappush(frontier,(priority,(current_x+1,current_y+1)))
      if (current_x,current_y) in parent:
        parent[(current_x,current_y)].append((current_x+1,current_y+1))
      else:
        parent[(current_x,current_y)] = [(current_x+1,current_y+1)]
      frontier_temp.append([current_x+1,current_y+1])
      
    
    i = i+1

  # shortest path search
  plt.plot(sx, sy, "xk") 
  tracebk_x, tracebk_y = gx, gy
  print len(explored)
  while (tracebk_x != sx or tracebk_y != sy):
    for key,val in parent.items():
      #print (key, "=>", val)
      if (tracebk_x,tracebk_y) in val:
        #print (key, "=>", val)
        (tracebk_x,tracebk_y) = key
        #print (tracebk_x)
        #print (tracebk_y)
        plt.plot(tracebk_x, tracebk_y, ".r")
        #plt.pause(0.000000000001) # Uncomment to animate the output
        if tracebk_x == sx and tracebk_y == sy:
          break
    
  #print (len(explored))
  #print (explored)
  #print (parent)
  #return explored
  #############################################
  plt.show()





