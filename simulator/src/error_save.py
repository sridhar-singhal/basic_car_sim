import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import math
import csv


count = 0
max_distance = 0
avg = 0
error = []
last_point = False
filename = 'straight_path_cilqr_he2.csv'


entry_dict = {
"Count" : -1,
"Point" : -1,
"Cross Track" : 0,
"Max Error" : 0,
"x" : -1,
"y" : -1
}

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
        # plotter()
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

    global entry_dict
    entry_dict = {
    "Count" : count,
    "Point" : index,
    "Cross Track" : ep,
    "Max Error" : max_distance,
    "x" : path.poses[index].pose.position.x,
    "y" : path.poses[index].pose.position.y
    }
   
    with open(filename,mode = 'a') as csv_file:
        fieldnames=["Count","Point","Cross Track","Max Error","x","y"]
        writer = csv.DictWriter(csv_file, fieldnames = fieldnames, delimiter=',')
        entry_dict1 = entry_dict
        writer.writerow({'Count' : entry_dict1["Count"], 'Point': entry_dict1["Point"],'Cross Track': entry_dict1["Cross Track"],'Max Error': entry_dict1["Max Error"], 'x': entry_dict1["x"], 'y': entry_dict1["y"] })





    

    # plt.plot(error)
    # plt.ylabel('current crosstrack error (m)')
    # plt.grid(True)
    # # plt.show()
    # # plt.pause(0.001)
    # # plt.clf()




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
    global crosstrack_pub
    
    rospy.init_node('error_save', anonymous=True)
    # Publsiher for Crosstrack error
    crosstrack_pub = rospy.Publisher('cross_track_error', Twist, queue_size = 10)
    # Subscriber for Path
    rospy.Subscriber("astroid_path", Path, path_callback)
    # Subscirber for Odom
    rospy.Subscriber("base_pose_no_error", Odometry, callback_position)

    # Will be put in a function to write in File
    with open(filename,mode = 'a') as csv_file:
        fieldnames=["Count","Point","Cross Track","Max Error","x","y"]
        writer = csv.DictWriter(csv_file, fieldnames = fieldnames, delimiter=',')
        global entry_dict
        entry_dict1 = entry_dict
        writer.writeheader()
        writer.writerow({'Count' : entry_dict1["Count"],'Point': entry_dict1["Point"],'Cross Track': entry_dict1["Cross Track"],'Max Error': entry_dict1["Max Error"], 'x': entry_dict1["x"], 'y': entry_dict1["y"] })

    rospy.spin()







if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
