#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def callback(data,robotname):
    pos_x=data.pose.pose.position.x
    pos_y=data.pose.pose.position.y
    pos_z=data.pose.pose.position.z
    rot_x=data.pose.pose.orientation.x
    rot_y=data.pose.pose.orientation.y
    rot_z=data.pose.pose.orientation.z
    rot_w=data.pose.pose.orientation.w
    broad=tf.TransformBroadcaster()
    broad.sendTransform((pos_x,pos_y,pos_z),(rot_x,rot_y,rot_z,rot_w),rospy.Time.now(),robotname,"world")

    rospy.loginfo("the position is (%s,%s)" % (pos_x, rot_w))

     # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str),The rotation is (%s,%s,%s,%s)
    #     pub.publish(hello_str),rot_x,rot_y,rot_z,rot_w
    #     rate.sleep()
    # print("Hello world")

if __name__ == '__main__':
    global robotname
    robotname=("robot_0","robot_1")


    rospy.init_node('broadcaster_0')
    rospy.Subscriber("/%s/base_pose_ground_truth" % robotname[0],Odometry,callback,robotname[0],queue_size=10)
    # rospy.init_node('broadcaster_1')
    rospy.Subscriber("/%s/base_pose_ground_truth" % robotname[1],Odometry,callback,robotname[1],queue_size=10)

    rospy.spin()
