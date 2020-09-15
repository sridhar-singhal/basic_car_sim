#!/usr/bin/env python

import math
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def frange(x, y, jump):
    while x < y:
        yield x
        x += jump

def get_path(vpath):
    print 'Enter 1:Straight Line \nEnter 2:Sinosoid \nEnter 3:Lane-switch\nEnter 4: Offset Straight-Line'
    vpath=input()
    return vpath
def get_initialization(vpath):
    print vpath
    if vpath==1:
            startx=0
            starty=0
            endx=100
            endy=0
    if vpath==2:
            startx=0
            starty=0
            endy=0
            endx=150
    if vpath==3:
            startx=0
            starty=0
            endy=30
            endx=150
    if vpath==4:
            startx = 0
            starty = 10
            endx = 100
            endy = 10
            print 'Offset: '
            starty = input()
            endy = starty
    print 'Default-start coordinates:{',startx,',',starty,'}',' end:{',endx,',',endy,'}'
    print 'Enter 0:To skip and start publishing path \nEnter 1:To change the start and end coordinates'
    change=input()
    if change==1:
        print('Enter end(x)')
        endx=input()
        print('Enter end(y)')
        endy=input()
    return startx,starty,endx,endy


def main():
    vpath=0
    vpath=get_path(vpath)
    startx,starty,endx,endy=get_initialization(vpath)
    rospy.init_node('astroid_curve_publisher')
    path_pub = rospy.Publisher('astroid_path', Path, queue_size=10)
    path = Path()

    path.header.frame_id = rospy.get_param('~output_frame', 'map')
    radius = rospy.get_param('~radius', 30.0)
    resolution = rospy.get_param('~resolution', 0.1)
    holonomic = rospy.get_param('~holonomic', False)
    offset_x = rospy.get_param('~offset_x', startx)    
    offset_y = rospy.get_param('~offset_y', starty)
    update_rate = rospy.get_param('~update_rate', 2.5)

    has_initialize = True

    if (vpath==1 or vpath==4):
        m=(endy-starty)/(endx-startx)
        for t in frange(0,endx,resolution):
            x = offset_x+t
            y=m*x+starty

            if has_initialize:
                old_x = x
                old_y = y
                has_initialize = False

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y

            yaw = 0.0
            if holonomic:
                yaw = -math.sin(t) / math.cos(t)
            else:
                if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                    yaw = math.atan2(old_y - y, old_x - x)
                else:
                    yaw = math.atan2(y - old_y, x - old_x)

            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

            old_x = x
            old_y = y



    if vpath==2:
        for t in frange(0,endx, resolution):
            x = t + int(offset_x)# add radius offset 
            y = 8*math.sin((2*math.pi*t)/endx + math.pi/2) + int(offset_y) - 8*math.sin(math.pi/2)
            if has_initialize:
                old_x = x
                old_y = y
                has_initialize = False

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y

            yaw = 0.0
            if holonomic:
                yaw = -math.sin(t) / math.cos(t)
            else:
                if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                    yaw = math.atan2(old_y - y, old_x - x)
                else:
                    yaw = math.atan2(y - old_y, x - old_x)

            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

            old_x = x
            old_y = y

    if vpath==3:
        e=2.7182818284  
        for t in frange(0,endx, resolution):
            x = float(offset_x) + t 
            # y = float(offset_y) + endy * ((e**(0.2 * (t-endx/2) )) / (100000 + (e**(0.2 * (t-endx/2) ))))
            y = float(offset_y) + endy*math.tanh(0.1*(t-endx/2))/2 + endy/2
            if has_initialize:
                old_x = x
                old_y = y
                has_initialize = False

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y

            yaw = 0.0
            if holonomic:
                yaw = -math.sin(t) / math.cos(t)
            else:
                if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                    yaw = math.atan2(old_y - y, old_x - x)
                else:
                    yaw = math.atan2(y - old_y, x - old_x)

            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

            old_x = x
            old_y = y
    
    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        path.header.stamp = rospy.get_rostime()
        path_pub.publish(path)
        
        #Print path coordinates

        # print('published')
        # for i in range(len(path.poses)):
        #     print '[',path.poses[i].pose.position.x,',',path.poses[i].pose.position.y,'],'
        # print('end')
        # if vpath==1:
        #     print 'Publishing Straight-Line from :{',offset_x,',',offset_y,'} to {',endx,',',endy,'}'
        
        r.sleep()
    
if __name__ == '__main__':
    main()

