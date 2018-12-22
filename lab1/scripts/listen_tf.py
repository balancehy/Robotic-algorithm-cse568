#!/usr/bin/env python
# import roslib
# roslib.load_manifest('lab1')
# ###################################
# Reference : ROS.org/tf/Tutorial
#####################################
import rospy
import tf
import math
import geometry_msgs.msg
from nav_msgs.msg import Odometry


if __name__ == '__main__':
    try:
        rospy.init_node("listener")
        listen=tf.TransformListener()
        robot2_cmdvel=rospy.Publisher('robot_1/cmd_vel',geometry_msgs.msg.Twist,queue_size=1)
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():

            try:

                (trans,rot)=listen.lookupTransform("/robot_1","/robot_0",rospy.Time(0))

            except (tf.LookupException,tf.ConnectivityException, tf.ExtrapolationException):
                continue
            linear_vel=0.5*math.sqrt(trans[0]**2+trans[1]**2)
            angular_vel=4*math.atan2(trans[1],trans[0])
            # rospy.loginfo("the position is ")
            # rospy.loginfo("the vel command is (%s,%s)" % (linear_vel, angular_vel))
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            robot2_cmdvel.publish(cmd)
            # cmd.linear.y=0
            # cmd.linear.z=0
            # cmd.angular.x=0
            # cmd.angular.y=0
            # rospy.loginfo("the vel command is (%s)" % (cmd))
            robot2_cmdvel.publish(cmd)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
