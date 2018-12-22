#!/usr/bin/env python

import rospy
from visualization_msgs.msg import *
# def switch_shape():
#     sw={
#     "CUBE":2
#     }

if __name__=="__main__":

    rospy.init_node("sendmarker")
    rate=rospy.Rate(10)
    marker_pub=rospy.Publisher("/visualization_marker",Marker,queue_size=10)

    while not rospy.is_shutdown():

        mymarker=Marker()

        mymarker.header.frame_id="base_link"
        mymarker.scale.x=0.5
        mymarker.scale.y=0.5
        mymarker.scale.z=0.5
        mymarker.color.r=0
        mymarker.color.g=0.5
        mymarker.color.b=0.5
        mymarker.color.a=1
        mymarker.pose.position.x=2
        mymarker.pose.position.y=2
        mymarker.pose.position.z=0
        mymarker.pose.orientation.x=0
        mymarker.pose.orientation.y=0
        mymarker.pose.orientation.z=0
        mymarker.pose.orientation.w=1
        # mymarker.ns="sendmarker"
        # mymarker.id=0
        mymarker.type=Marker.CUBE
        # mymarker.lifetime=100

        mymarker.action=0
        rospy.loginfo("type is %s" % mymarker.type)
        marker_pub.publish(mymarker)
        rate.sleep()
        # rospy.spin()
