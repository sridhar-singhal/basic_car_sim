#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import String
import math
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
count = 0
max_distance = 0
avg = 0
error = []
last_point = False


def callback_position(data):
    """
    calculates the current bot orientation(bot_theta) and current bot
    velocity(bot_vel) by converting from quaternion coordinates

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param data: (twist)
    :param bot_theta: (float) current orientation of bot
    :param bot_vel: (float) current velocity of bot

    """
    
    global x
    global y
    global bot_theta
    global bot_vel
    global max_distance
    global avg
    global count
    global error
    global path
    global last_point
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    siny = 2.0 * (data.pose.pose.orientation.w *
                  data.pose.pose.orientation.z +
                  data.pose.pose.orientation.x *
                  data.pose.pose.orientation.y)
    cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y *
                        data.pose.pose.orientation.y +
                        data.pose.pose.orientation.z *
                        data.pose.pose.orientation.z)
    bot_theta = math.atan2(siny, cosy)
    bot_vel = data.twist.twist.linear.x * math.cos(bot_theta) + data.twist.twist.linear.y * math.sin(bot_theta)

    distances = []

    for i in range(len(path.poses)):
        a = path.poses[i]
        distances += [dist(a,x,y)]
    ep = min(distances)
    min_point = distances.index(ep)

    if(min_point == len(path.poses) - 1):
    	last_point = True
    	print 'end point reached \n\n\n\n end point reached'
    	plotter()
    if (ep>max_distance):
        max_distance = ep

    if(bot_vel > 0.1):
        avg *=count
        count +=1
        avg += ep
        avg /=count
    # else:
    #     plotter()



    index = distances.index(ep)
    print count ,' , ' ,avg , ' , ', min_point, ' , ', len(path.poses)
    '''In world coordinatess x axis is heading direction of car, and thus slope is calculated
    So we shall see the sign of y component '''
    xTransformed = path.poses[index].pose.position.x * cosy + path.poses[index].pose.position.y * siny
    yTransformed = -path.poses[index].pose.position.x * siny + path.poses[index].pose.position.y * cosy

    if(yTransformed < 0):
        ep = - ep
    
    if(count%5 == 0):
    	if not last_point:
    		error.append(ep)

    # error += [ep];
    msg = Twist()

    msg.linear.x = ep
    msg.linear.y = avg
    msg.linear.z = max_distance
    crosstrack_pub.publish(msg)

    # plt.plot(error)
    # plt.ylabel('current crosstrack error (m)')
    # plt.grid(True)
    # # plt.show()
    # # plt.pause(0.001)
    # # plt.clf()



def plotter():
    global error
    plt.plot(error)
    plt.ylabel('current crosstrack error (m)')
    plt.grid(True)
    # plt.show()
    plt.pause(0.001)
    # plt.clf()
    # plt.close()


def dist(a, x, y):
    """
    calculates the euclidian distance between 2 points

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param a: () contains the coordinates of other point
    """
    return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5


def path_callback(data):
    """
    Stores Path in a global variable path.
    """
    global path
    path = data

    
def start():

    # rate = rospy.rate(10)
    global crosstrack_pub
    rospy.init_node('error_plot', anonymous=True)
    
    crosstrack_pub = rospy.Publisher('cross_track_error', Twist, queue_size = 10)
    rospy.Subscriber("astroid_path", Path, path_callback)
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback_position)
    # rate.sleep()
    # plotter()
    if(last_point):
    	plotter()
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass

    
