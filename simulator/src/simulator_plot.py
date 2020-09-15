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

global pub
global state
global acc
global omega
global dt     # Time tick
dt = 0.05

#Plot variables
global plot_x
global plot_y
plot_x = [0.0]
plot_y = [0.0]
global path_x
global path_y
path_x = [0.0]
path_y = [0.0]
global predicted_path_x
global predicted_path_y
predicted_path_x = [0.0]
predicted_path_y = [0.0]

# Vehicle parameters
LENGTH = 4.5         # [m]
WIDTH = 2.0          # [m]
BACKTOWHEEL = 1.0    # [m]
WHEEL_LEN = 0.3      # [m]
WHEEL_WIDTH = 0.2    # [m]
TREAD = 0.7          # [m]
WB = 2.5             # [m] Wheelbase

MAX_STEER = np.deg2rad(45.0)        # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)       # maximum steering speed [rad/s]
# MAX_SPEED = 55.0 / 3.6              # maximum speed [m/s]
MAX_SPEED = 5.0              # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6             # minimum speed [m/s]
MAX_ACCEL = 1.0                     # maximum accel [m/ss]

show_animation = True


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


def callback_path(data):
    global path
    global path_x
    global path_y
    path_x = [0.0]
    path_y = [0.0]
    path = data
    for i in range(len(path.poses)):
        path_x.append(path.poses[i].pose.position.x)
        path_y.append(path.poses[i].pose.position.y) 

def callback_predicted_path(data):
    global predicted_path
    global predicted_path_x
    global predicted_path_y
    predicted_path_x = [0.0]
    predicted_path_y = [0.0]
    predicted_path = data
    for i in range(len(predicted_path.poses)):
        predicted_path_x.append(predicted_path.poses[i].pose.position.x)
        predicted_path_y.append(predicted_path.poses[i].pose.position.y) 

def feedback(data):
    '''
    Subscribes Odometry
    '''
    global state
    global plot_x
    global plot_y
    global path_x
    global path_y

    state = data

    plot_x.append(state.pose.pose.position.x)
    plot_y.append(state.pose.pose.position.y)

    # plot(state)
    if show_animation:  # pragma: no cover
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

        # Plot global path
        plt.plot(path_x, path_y, "-r", label="course")

        #Plot trajectory followed by car
        plt.plot(plot_x, plot_y, "ob", label="trajectory")

        #Plot predicted path by ilqr
        # plt.plot(predicted_path_x, predicted_path_y, "-k", label="predicted_path")

        #Plot car model
        plot_car(state.pose.pose.position.x, state.pose.pose.position.y, state.twist.twist.angular.z, steer=0)
        
        plt.axis("equal")
        plt.grid(True)    
        plt.pause(0.0001)


if __name__ == '__main__':

    rospy.init_node('simulator_plot_node', anonymous=True)
    rospy.Subscriber("astroid_path", Path, callback_path, queue_size=1)
    rospy.Subscriber("predicted_path", Path, callback_predicted_path, queue_size=1)
    rospy.Subscriber("base_pose_ground_truth", Odometry, feedback, queue_size=1)
    rospy.spin()
