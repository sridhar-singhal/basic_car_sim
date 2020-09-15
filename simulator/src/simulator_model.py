#!/usr/bin/env python

from numpy.linalg import inv
import math
import matplotlib.pyplot as plt
from math import exp
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from nav_msgs.msg import Path
import numpy as np

global pub
global state
acc = 0
omega = 0
global dt     # Time tick
dt = 0.05


# Vehicle parameters
WB = 2.5             # [m] Wheelbase

MAX_STEER = np.deg2rad(45.0)        # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)       # maximum steering speed [rad/s]
# MAX_SPEED = 55.0 / 3.6              # maximum speed [m/s]
MAX_SPEED = 5.0             # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6             # minimum speed [m/s]
MAX_ACCEL = 5.0                     # maximum accel [m/ss]

show_animation = True


class State:
    """
    vehicle state class
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        # self.predelta = None


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def noise(mean, variance):
    '''
    Input  : Mean and variance 
    Output : Gaussian noise 
    '''
    return (np.asscalar(np.random.normal(mean, variance,((1,1, 1)))))

def calc_states():

    state.x = state.x + (state.v*dt + 0.5*(acc + noise(0, 0.001))*dt*dt)*math.cos(state.yaw) + noise(0, 0.001)   
    state.y = state.y + (state.v*dt + 0.5*(acc + noise(0, 0.001))*dt*dt)*math.sin(state.yaw) + noise(0, 0.001)
    state.v = state.v + (acc)*dt + noise(0, 0.001)
    state.yaw = pi_2_pi(state.yaw + (omega)*dt) + noise(0, 0.000001)

    if state. v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state. v < MIN_SPEED:
        state.v = MIN_SPEED

    print "Velocity : " + str(state.v)


def feedback(data):
    '''
    Subscribes action topic [Twist]
    linear.x = acceleration [m/s]
    angular.z = angular velocity [rad/sec]

    # In case the steering angle is used 
    angular.z = steering angle [degrees]

    Pubishes Odometry (x, y, v, theta) [m, m, m/s, rad]
    '''
    global state
    global acc
    global omega
    acc = data.linear.x     # m/s^2
    omega = data.angular.z  # rad/sec

    '''
    ##Use this if the input is the steering angle [degrees]

    delta = data.angular.z  # degrees

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    omega = math.tan(delta*math.pi/180)*state.v/WB  #rad/sec
    '''
    
    # state.x = state.x + (state.v*dt + 0.5*(acc + noise(0, 0.001))*dt*dt)*math.cos(state.yaw) + noise(0, 0.001)   
    # state.y = state.y + (state.v*dt + 0.5*(acc + noise(0, 0.001))*dt*dt)*math.sin(state.yaw) + noise(0, 0.001)
    # state.v = state.v + (acc)*dt + noise(0, 0.001)
    # state.yaw = pi_2_pi(state.yaw + (omega)*dt) + noise(0, 0.000001)

    # if state. v > MAX_SPEED:
    #     state.v = MAX_SPEED
    # elif state. v < MIN_SPEED:
    #     state.v = MIN_SPEED

    # print "Velocity : " + str(state.v)



if __name__ == '__main__':

    '''Initialising the state of car''' 
    global state
    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

    global pub
    global pubNOE

    rospy.init_node('simulator_node', anonymous=True)
    rospy.Subscriber("action", Twist, feedback)
    pub = rospy.Publisher('base_pose_ground_truth', Odometry, queue_size=10)
    pubNOE = rospy.Publisher('base_pose_no_error',Odometry,queue_size = 10)

    new_state = Odometry()
    noe = Odometry()

    r = rospy.Rate(20) # 20hz

    while not rospy.is_shutdown():
        
        calc_states()
        new_state.pose.pose.position.x = state.x + noise(0, 0.01)
        new_state.pose.pose.position.y = state.y + noise(0, 0.01)
        new_state.twist.twist.linear.x = state.v + noise(0, 0.01)
        new_state.twist.twist.angular.z = state.yaw + noise(0, 0.00001)

        noe.pose.pose.position.x = state.x 
        noe.pose.pose.position.y = state.y 
        noe.twist.twist.linear.x = state.v 
        noe.twist.twist.angular.z = state.yaw 

        pub.publish(new_state)
        pubNOE.publish(noe)

        r.sleep()     

    rospy.spin()
    
    