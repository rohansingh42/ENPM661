# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import cPickle 
import math
import numpy as np
 
# robot state variables
position_ = Point()
theta_ = 0
# # machine state
# state_ = 0
# # goal
# desired_position_ = Point()
# desired_position_.x = -3
# desired_position_.y = 7
# desired_position_.z = 0
# # parameters
# yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
# dist_precision_ = 0.3
 
# # publishers
# pub = None
 
# # callbacks
# def clbk_odom(msg):
#     global position_
#     global yaw_
    
#     # position
#     position_ = msg.pose.pose.position
    
#     # yaw
#     quaternion = (
#         msg.pose.pose.orientation.x,
#         msg.pose.pose.orientation.y,
#         msg.pose.pose.orientation.z,
#         msg.pose.pose.orientation.w)
#     euler = transformations.euler_from_quaternion(quaternion)
#     yaw_ = euler[2]
 
# def change_state(state):
#     global state_
#     state_ = state
#     print 'State changed to [%s]' % state_
 
# def fix_yaw(des_pos):
#     global yaw_, pub, yaw_precision_, state_
#     desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
#     err_yaw = desired_yaw - yaw_
    
#     twist_msg = Twist()
#     if math.fabs(err_yaw) > yaw_precision_:
#         twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
#     pub.publish(twist_msg)
    
#     # state change conditions
#     if math.fabs(err_yaw) <= yaw_precision_:
#         print 'Yaw error: [%s]' % err_yaw
#         change_state(1)
 
# def go_straight_ahead(des_pos):
#     global yaw_, pub, yaw_precision_, state_
#     desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
#     err_yaw = desired_yaw - yaw_
#     err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
#     if err_pos > dist_precision_:
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.6
#         pub.publish(twist_msg)
#     else:
#         print 'Position error: [%s]' % err_pos
#         change_state(2)
    
#     # state change conditions
#     if math.fabs(err_yaw) > yaw_precision_:
#         print 'Yaw error: [%s]' % err_yaw
#         change_state(0)
 
# def done():
#     twist_msg = Twist()
#     twist_msg.linear.x = 0
#     twist_msg.angular.z = 0
#     pub.publish(twist_msg)
 
# def main():
#     global pub
    
#     rospy.init_node('go_to_point')
    
#     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
#     sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
#     rate = rospy.Rate(20)
#     while not rospy.is_shutdown():
#         if state_ == 0:
#             fix_yaw(desired_position_)
#         elif state_ == 1:
#             go_straight_ahead(desired_position_)
#         elif state_ == 2:
#             done()
#             pass
#         else:
#             rospy.logerr('Unknown state!')
#             pass
#         rate.sleep()

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

def odometryCallback(msg):

    global position_
    global theta_
    
    # position
    position_ = msg.pose.pose.position
    
    # theta or yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    theta_ = (180/math.pi)*euler[2]
    if theta_ > 180:
        theta_ = theta_ - 360
    elif theta_ <= -180:
        theta_ = theta_ + 360

    
def executeStep(des_x,des_y,des_theta,stepid,delt):

    global pos_precision
    global ang_precision
    global position_
    global theta_
    global pub
    global RPM1
    global RPM2
    global f

    des_x = des_x/1000.0
    des_y = des_y/1000.0
    print('des_x',des_x)

    twist_msg = Twist()

    wr = 0.038               # Wheel Radius (m)
    wb = 0.230              # Wheel Base (m)

    motion_model = [[0,RPM1],                
              [RPM1,0],     
              [RPM1,RPM1],               
              [0,RPM2],    
              [RPM2,0],               
              [RPM2,RPM2],   
              [RPM1,RPM2],          
              [RPM2,RPM1]]
    step = motion_model[stepid]
    ul = 2*math.pi*step[0]/60.0
    ur = 2*math.pi*step[1]/60.0
    # print(ur)
    v = (wr/2) * (ul + ur)
    print('v',v)
    w = (wr/wb) * (ur - ul)
    print('w',w)

    pos_error = math.sqrt((position_.x - des_x)**2 + (position_.y - des_y)**2)
    ang_error = np.abs(theta_ - des_theta)
    print('poserr',pos_error)
    print('angerr',ang_error)
    print('relposx',position_.x)
    print('relposy',position_.y)
    # if (pos_error < pos_precision) and (ang_error < ang_precision):
    #     rf = 1
    # else :   
    #     twist_msg.linear.x = v
    #     twist_msg.angular.z = w
    #     pub.publish(twist_msg)
    #     rf = 0

    if (delt < 1.05*(1/f)):
        twist_msg.linear.x = v
        if w > 0:
            twist_msg.angular.z = w + 0.1
        else:
            twist_msg.angular.z = w
        pub.publish(twist_msg)
        rf = 0
    else:
        rf = 1  
    return rf

def done():
    global pub
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def simulate(pathData):
    global pub
    global pos_precision
    global ang_precision
    global RPM1
    global RPM2
    global f

    pos_precision = 0.02
    ang_precision = 2

    RPM1 = pathData[0]
    RPM2 = pathData[1]
    f = pathData[2]
    xData = pathData[3]
    yData = pathData[4]
    thetaData = pathData[5]
    stepidData = pathData[6]
    print(len(xData))

    
    rospy.init_node('simulate_path')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # sub_odom = rospy.Subscriber('/odom', Odometry, odometryCallback)
    
    rate = rospy.Rate(200)
    i = 1
    fl = 1
    init_msg = Twist()
    init_msg.linear.x = 0
    init_msg.angular.z = 0
    pub.publish(init_msg)
    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        rate.sleep()
        des_x = xData[i] - xData[0]
        des_y = yData[i] - yData[0]
        des_theta = thetaData[i]
        des_stepid = stepidData[i]
        if fl == 1:
            fl = 0
            t0 = rospy.get_time()
        t = rospy.get_time()
        print('time',t-t0)
        rf = executeStep(des_x,des_y,des_theta,des_stepid,(t-t0))
        if rf == 0:
            print('step',i)
            continue
        elif rf == 1:
            print('Next step')
            i = i+1
            t0 = rospy.get_time()
            if i == (len(xData)-1):
                done()
        else:
            rospy.logerr('Unknown state!')
            pass
        # rate.sleep()

def  main():

    pathData = []
    try:
        with open('path.pkl', 'rb') as file:
            pathData = cPickle.load( file )
    except IOError:
        file = open( 'path.pkl', "w" )
        cPickle.dump( pathData, file )
    file.close()
    # xData = pathData[0]
    # yData = pathData[1]
    # thetaData = pathData[2]
    # stepidData = pathData[3]
    # print(yData)
    print(pathData)
    simulate(pathData)


if __name__ == '__main__':
    main()