#!/usr/bin/env python
import rospy, time, tf
from nav_msgs.msg import *
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from collections import deque
import heapq as hq
import math as math
from tf.transformations import euler_from_quaternion


def poseCallback(poseData):
	global current_x
	global current_y
	global current_theta

	rate = rospy.Rate(10)

	current_theta = None
	current_x = None
	current_y = None
	
	current_x = poseData.pose.pose.position.x
	current_y = poseData.pose.pose.position.y

	ori_x = poseData.pose.pose.orientation.x
	ori_y = poseData.pose.pose.orientation.y
	ori_z = poseData.pose.pose.orientation.z
	ori_w = poseData.pose.pose.orientation.w

	q = [ori_x,ori_y,ori_z,ori_w]
	
	roll,pitch,yaw = euler_from_quaternion(q)

	current_theta = yaw
	rate.sleep()
	

def mapCallback(mapData):
	global map_data
	global map_data2d
	global width
	global height

	map_data = []
	map_data_new = []
	map_data = mapData.data
	width = 544
	height = 480
	map_data2d = [] 

	#print mapData.info
	#print len(mapData.data)
	#print 'map_data:'+ str(map_data)
	#for i in range(0,len(map_data)-1):
	#	print str(i) + '-->' + str(map_data[i])
	
	# for i in range(0,len(map_data)-1):
	# 	if map_data[i] == -1:
	# 		#print 'i:'+ str(i)
	# 		pass
	# 	else:
	# 		#print 'i-->'+ str(i)
	# 		#print 'map_data:'+str(map_data[i])
	# 		map_data_new.append(map_data[i])
	# 		#print "map_data_new:",str(map_data_new[x])
			
	#print map_data_new
	x = 0
			
	#print map_data2d
	for i in range(0, height):
		map_data2d.append([])
	 	for j in range(0, width):
	 		map_data2d[i].append(map_data[x])
			x = x+1


	#print map_data2d
	#print 'row' + str(len(map_data2d))
	#print 'col' + str(len(map_data2d[0]))
	path_planning()

def publishTwist(x_vel,angular_vel):
	twist = Twist()
	twist.linear.x = x_vel
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = angular_vel

	print 'linear.x=' + str(twist.linear.x) + ',' +  'linear.y=' + str(twist.linear.y) + ',' + 'linear.z=' + str(twist.linear.z) + ',' + 'angular.x=' + str(twist.angular.x) + ',' + 'angular.y=' + str(twist.angular.y) + ',' + 'angular.z=' + str(twist.angular.z)

	cmd_vel_pub.publish(twist)

def in_obstacle_space(posi_x,posi_y):
	global map_data2d
	pre_result = False
	for i in range(posi_x-6,posi_x+6):
		for j in range(posi_y-6, posi_y+6):
			if map_data2d[i][j] == 100 or map_data2d[i][j] == -1:
				cur_result = pre_result or True
			else:
				cur_result = pre_result or False
			pre_result = cur_result

	return cur_result

def h(x1, x2, y1, y2):
	#return max(abs(x1 - x2),abs(y1 - y2)) # Diagonal distance
  	return abs(x1-x2) + abs(y1-y2) # Manhatten distance
  	#return math.sqrt(((x1 - x2)**2) + ((y1 - y2)**2)) # Euclidean distance


def path_planning():
	# START AND GOAL POSITION
    global sx
    global sy
    global gx
    global gy
    global map_data2d
    global waypoints
    global curr_theta
    global curr_x
    global curr_y
    sx = 230
    sy = 240
    gx = 267
    gy = 235
    waypoints = deque()
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
    	print 'Entering A Star Algo'
        while (current_x!=gx or current_y!=gy): #while goal is not reached
    		current_x, current_y = hq.heappop(frontier)[1]
    		frontier_temp.popleft()
    		#if not([current_x,current_y] in explored): #and not(contains_pt(current_x,current_y)):
    		explored.append([current_x,current_y])
    		#plt.plot(current_x, current_y, ".g") # Uncomment to plot the explored nodes
    		#plt.pause(0.00000000001) # Uncomment to animate the output 
		    #adding neighbours to frontier and tracking parent node

		    # east movement
    		if not(in_obstacle_space(current_x+1,current_y)) and not([current_x+1,current_y] in explored) and not([current_x+1,current_y] in frontier_temp):
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
    		if not(in_obstacle_space(current_x+1,current_y-1)) and not([current_x+1,current_y-1] in explored) and not([current_x+1,current_y-1] in frontier_temp):
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
    		if not(in_obstacle_space(current_x,(current_y-1))) and not([current_x,current_y-1] in explored) and not([current_x,current_y-1] in frontier_temp):
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
    		if not(in_obstacle_space(current_x-1,current_y-1)) and not([current_x-1,current_y-1] in explored) and not([current_x-1,current_y-1] in frontier_temp):
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
    		if not(in_obstacle_space((current_x-1),current_y)) and not([current_x-1,current_y] in explored) and not([current_x-1,current_y] in frontier_temp):
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
    		if not(in_obstacle_space(current_x-1,current_y+1)) and not([current_x-1,current_y+1] in explored) and not([current_x-1,current_y+1] in frontier_temp):
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
    		if not(in_obstacle_space(current_x,(current_y+1))) and not([current_x,current_y+1] in explored) and not([current_x,current_y+1] in frontier_temp):
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
    		if not(in_obstacle_space(current_x+1,current_y+1)) and not([current_x+1,current_y+1] in explored) and not([current_x+1,current_y+1] in frontier_temp):
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

    	#plot_map()

	    # shortest path search
    	#plt.plot(sx, sy, "xk")
    	#plt.plot(gx,gy,"bo") 
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
	                waypoints.append([tracebk_x,tracebk_y])
	                #plt.plot(tracebk_x, tracebk_y, ".m") # Uncomment to plot the path in 2D map
	                #plt.pause(0.000000000001) # Uncomment to animate the output
	                #print 'x:' + str(tracebk_x)
	                #print 'y:' + str(tracebk_y)
	                
	                if tracebk_x == sx and tracebk_y == sy:
	                	break
    #plt.show() # Uncomment to show the plot
    print waypoints
    
    goToPositionInAStraightLine()
    
    
############# Uncomment to generate the 2D map and the path
# def plot_map():
# 	fig = plt.figure()
# 	ax = fig.add_subplot(111)

# 	for i in range(0, height):
# 	    for j in range(0, width):
#     		if map_data2d[i][j] == 100:
# 				#print str(i) + str(j) + 'hundred'
# 				plt.plot(j,i,'.b')
# 				#plt.pause(0.00000000001)
#     		elif map_data2d[i][j] == 0:
# 				#print str(i) + str(j) + 'zero'
# 				plt.plot(j,i,'.c')
#     		elif map_data2d[i][j] == -1:
# 				#print str(i) + str(j) + 'minus one'
# 				pass #plt.plot(j,i,".c")
    
# 	#plt.plot(sx,sy,'r+')
# 	#plt.plot(gx,gy,'bo')

# 	ax.set_xlim(0, width-1)
# 	ax.set_ylim(0, height-1)

	

def goToPositionInAStraightLine():
    #global rate
    global speed
    global my_curr_x
    global my_curr_y
    global pos_tolerance
    global angle_tolerance
    # global sys_curr_x
    # global sys_curr_y
    # global sys_curr_theta
    global destination_angle
    global my_curr_theta
    #getpose_msg()
    #pos_tolerance = 0.02
    angle_tolerance = 0.05
    speed = 0.13
    rate = rospy.Rate(2)
    my_curr_x = 230#current_x
    my_curr_y = 240#current_y
    my_curr_theta = current_theta
    

    while not(my_curr_x==gx) and not(my_curr_y==gy):
    	#getpose_msg()
    	destination_x, destination_y = waypoints.pop() #Remove and returns an element from right of the deque
    	#sys_curr_x = current_x
    	#sys_curr_y = current_y
    	#sys_curr_theta = current_theta
    	angle = math.atan2(destination_y - my_curr_y, destination_x - my_curr_x) #my_angle
    	rotateToAngle(angle)

    	#goToPosition(speed,destination_x,destination_y, sys_curr_x, sys_curr_y, angle)
    	curr_x = destination_x
    	curr_y = destination_y
    	#curr_theta = destination_angle


def rotateToAngle(angle):
	#getpose_msg()
	P = 0.8
	I = 0.03
	D = 0.001
	Derivator = 0
	Integrator = 0
	outMin = -1.3
	outMax = 1.3
	destination_angle = angle
	# if abs(angle) > 0:
	# 	destination_angle = angle + curr_theta 
	error = normalize_angle(destination_angle - current_theta)
	
	rate = rospy.Rate(2)

	while abs(error) > angle_tolerance:
		error = normalize_angle(destination_angle - current_theta)
		P_val = P * error
		D_val = D * (error - Derivator)
		Derivator = error
		Integrator = Integrator + error
		I_val = I * Integrator
		if I_val > outMax:
			I_val = outMax
			Integrator -= error
		elif I_val < outMin:
			I_val = outMin
			Integrator -= error

		PID = P_val +I_val + D_val

		if PID > outMax:
			PID = outMax
		elif PID < outMin:
			PID = outMin

		corrected_angle = PID
		print 'error' + str(error)
		print 'destination_angle' + str(destination_angle)
		print 'current_theta' + str(current_theta)
		publishTwist(0,corrected_angle)
		rate.sleep()
		if current_theta == destination_angle:
			break
		# publishTwist(speed,0)
		# rate.sleep()
	publishTwist(speed,0)
	rate.sleep()
	#publishTwist(0,0)
	#rate.sleep()
    

def normalize_angle(angle):
	return math.atan2(math.sin(angle),math.cos(angle))



if __name__ == '__main__':
	global cmd_vel_pub
	global current_theta
	global map_data2d
	rospy.init_node('getmap_msg', anonymous=True)
	cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=1)
	pose_sub = rospy.Subscriber('/odom', Odometry, poseCallback, queue_size=1)
	
	rospy.spin()