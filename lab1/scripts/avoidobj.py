#!/usr/bin/env python

import math
import rospy
import random
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
global Pub_Velo
global minrange
global flag_rotate
global rand
flag_rotate=False
minrange=4
rand=-1
def pack_move(xlinear,ztwist):
    msg=Twist() # format the mesages to be sent
    msg.linear.x=xlinear
    msg.linear.y=0# y velociy does not work
    msg.linear.z=0
    msg.angular.x=0
    msg.angular.y=0
    msg.angular.z=ztwist
    return msg

def callback(data):
    global flag_rotate
    global rand
    angle_min=data.angle_min
    angle_max=data.angle_min
    ranges=data.ranges
    intensities=data.intensities
    range_max=data.range_max
    range_min=data.range_min
    maxdelay=0.5
    i=30
    vel_ctl=0
    temp=range_max+1# enure intital temp is larger than ranges[0]

    while i<140:
        if ranges[i]<temp:
            temp=ranges[i]
        i=i+1
    minrange=temp
    vel_ctl=0
    vel_twist=0
    if minrange>range_max*0.7:
        vel_ctl=2
        vel_twist=0
        flag_rotate=False
    elif flag_rotate==False:
        rand=random.randint(0,9)
        flag_rotate=True
        vel_ctl=0
        vel_twist=(10-2)/9*rand+2
    elif flag_rotate==True:
        vel_twist=(10-2)/9*rand+2

    # delaytime=(maxdelay-0)/(9-0+1)*rand
    # t0 = rospy.Time.now().to_sec()
    # tf=t0
    # while (tf-t0)<delaytime:
    #     msg=pack_move(0,1)
    #     Pub_Velo.publish(msg)
    #     tf=rospy.Time.now().to_sec()
    msg=pack_move(vel_ctl,vel_twist) #4*(rand*2-1)
    Pub_Velo.publish(msg)
    # msg=pack_move(0,0)
    # Pub_Velo.publish(msg)

    # rand=random.randint(0,99)
    # angle_ctl=2 # rad/s
    # rospy.loginfo("min range is %s\n object detect status is %s"
    # % (str(minrange),flag_objdetect)) #
def control():
    rospy.init_node('controller',anonymous=True)

    global Pub_Velo

    # global t0 tf rand
    # maxdelay=0.5
    # rand=random.randint(0,1)
    Pub_Velo=rospy.Publisher('/robot_0/cmd_vel',Twist, queue_size=10)
    rate=rospy.Rate(10)
    # t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        # tf=rospy.Time.now().to_sec()
        # if tf-t0<maxdelay:

        rospy.Subscriber("/robot_0/base_scan",LaserScan,callback,queue_size=10)
        rate.sleep()
        # rospy.sleep(/)
        # rospy.spin()


def sensor():
    rospy.init_node("evader_sensor", anonymous=True)
    rospy.Subscriber("/robot_0/base_scan",LaserScan,callback,queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    try:
        # avoidobj()
        # sensor()
        control()
    except rospy.ROSInterruptException:
        pass
