#!/usr/bin/env python

import rospy
from math import cos,sin,sqrt,atan
import random
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from visualization_msgs.msg import *
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

def coord_coversion(ranges,angles):
# convert polari coordinates to cartesiane in local frame
    coordinates=[[cos(angles[i])*ranges[i]*(ranges[i]<3),sin(angles[i])*ranges[i]*(ranges[i]<3)] for i in range(len(angles))]
    index_noobj=[i for i in range(len(ranges)) if ranges[i]<3] # index in coordinates whose range is lower than 3
    return [coordinates,index_noobj]
def linedetect_one(coordinates,indexcoord_nonzero,throtle,num_iter):

    temp=list(indexcoord_nonzero)
    max_inliner=0
    for j in range(num_iter):
        # rospy.loginfo("size is (%s)" % (len(indexcoord_nonzero)))
        rand1=random.randint(0,len(indexcoord_nonzero)-1)
        selpoint=[indexcoord_nonzero[rand1]]
        temp=list(indexcoord_nonzero)

        temp.pop(rand1)
        rand2=random.randint(0,len(temp)-1)
        selpoint.append(temp[rand2])

        x1=coordinates[selpoint[0]][0]
        y1=coordinates[selpoint[0]][1]
        x2=coordinates[selpoint[1]][0]
        y2=coordinates[selpoint[1]][1]
        line_vec=[x2-x1,(y2-y1)]
        costheta=line_vec[0]/sqrt(line_vec[0]**2+line_vec[1]**2)
        sintheta=line_vec[1]/sqrt(line_vec[0]**2+line_vec[1]**2)
        line_perp=[-sintheta,costheta]
        y_local=[-sintheta*(coordinates[i][0]-x1)+costheta*(coordinates[i][1]-y1) for i in indexcoord_nonzero] # y_local has the same length of indexcoord_nonzero

        index_point_in=[indexcoord_nonzero[i] for i in range(len(y_local)) if abs(y_local[i])<throtle] # record coordinates
        num_inliner=len(index_point_in)
        if num_inliner>max_inliner:# pass values wrt lines with maximum points
            max_inliner=num_inliner
            # k_fit=line_vec[1]/line_vec[0]
            # d_fit=y1-k_fit*x1 # y=k*x+d -> k*x-y+d=0
            A_fit=line_vec[1]
            B_fit=-line_vec[0]
            C_fit=y1*line_vec[0]-line_vec[1]*x1
            sample1=selpoint[0]
            sample2=selpoint[1]
            samplex1=x1
            samplex2=x2
            sampley1=y1
            sampley2=y2

    # rospy.loginfo("y_local is (%s)\n" % (y_local))
    # rospy.loginfo("sample1,sample2,[x1,y1],[x2,y2],kline is (%s,%s,%s,%s,%s)" % (sample1,sample2,[samplex1,sampley1],[samplex2,sampley2],[k_fit,d_fit]))
    # return [sample1,sample2,k_fit,d_fit,num_inliner,index_point_in] # slope and intersection
    return [sample1,sample2,A_fit,B_fit,C_fit,num_inliner,index_point_in] #
def pub_markers(sample_set,linepara_set,coordinates):
    point_line=rospy.Publisher("/visualization_marker",Marker,queue_size=1)
    #set up points and linestrip style
    points=Marker()
    line_list=Marker()
    points.header.frame_id="base_laser_link"
    points.header.stamp=rospy.Time.now()
    points.action=0 # ADD
    points.id=0
    points.type=Marker.POINTS
    points.scale.x=0.1
    points.scale.y=0.1
    points.color.g=1.0
    points.color.a=1
    points.lifetime=rospy.Duration(1)
    line_list.header.frame_id="base_laser_link"
    line_list.header.stamp=rospy.Time.now()
    line_list.action=0 # ADD
    line_list.id=1
    line_list.type=Marker.LINE_LIST
    line_list.scale.x=0.08
    line_list.color.b=1
    line_list.color.a=1
    line_list.lifetime=rospy.Duration(1)
    # get the points to be published
    global deltaX
    for i in range(len(sample_set)):
        xtemp=coordinates[sample_set[i][0]][0] # find the frist x coordinate in collected two-point set
        xtemp2=coordinates[sample_set[i][1]][0]# second
        ytemp=coordinates[sample_set[i][0]][1]
        ytemp2=coordinates[sample_set[i][1]][1]
        if xtemp<xtemp2:
            x=[xtemp,xtemp2]
            y=[ytemp,ytemp2]
        else:
            x=[xtemp2,xtemp]
            y=[ytemp2,ytemp]

        x_extra_r=x[1]+deltaX
        x_extra_l=x[0]-deltaX
        # y_extra_r=linepara_set[i][0]*x_extra_r+linepara_set[i][1]
        # y_extra_l=linepara_set[i][0]*x_extra_l+linepara_set[i][1]
        if linepara_set[i][1]==0: # if B=0,vertcal line
            y_extra_r=deltaX+y[1]
            y_extra_l=-deltaX+y[0]
        else:# if not vertical line,
            y_extra_l=-(linepara_set[i][0]/linepara_set[i][1])*x_extra_l-linepara_set[i][2]/linepara_set[i][1]
            y_extra_r=-(linepara_set[i][0]/linepara_set[i][1])*x_extra_r-linepara_set[i][2]/linepara_set[i][1]

        x[0]=x_extra_l
        x[1]=x_extra_r
        y[0]=y_extra_l
        y[1]=y_extra_r

        # points.points=list
        # line_strip.points=[]
        # temp=[]
        for kk in range(len(x)):
            pub_p=geometry_msgs.msg.Point()
            pub_p.x=x[kk]
            pub_p.y=y[kk]
            pub_p.z=0
            points.points.append(pub_p)
            line_list.points.append(pub_p)

        # rospy.loginfo("x,y is (%s)" % (temp))

        point_line.publish(points)
        # rospy.loginfo("x,y is (%s)" % (points.points))
        point_line.publish(line_list)
def linedetect(data):
    global linesimilarity
    global state
    angle_min=data.angle_min # start from -pi/2
    angle_max=data.angle_max # end at 1/2*pi
    ranges=data.ranges
    angle_increment=data.angle_increment

    # intensities=data.intensities
    range_max=data.range_max
    range_min=data.range_min
    totalangle=360
    angles=[angle_min+(angle_max-angle_min)/totalangle*i for i in range(totalangle+1)]# sweep angle from pi/2 to -pi/2 to make it consisent with ranges
    [coordinates,indexcoord_nonzero]=coord_coversion(ranges,angles)
    # rospy.loginfo("the length of angels, ranges and coordinates is %s,%s, %s" % (len(angles),len(ranges),len(coordinates)))
    # rospy.loginfo("test data is (%s,%s)" % (angles,ranges))
    # rospy.loginfo("size is (%s)" % (len(indexcoord_nonzero)))
    # rospy.loginfo("test data is (%s,%s,%s)" % (angles[225],ranges[225],coordinates[225]))

        # rospy.loginfo("x1,y1,x2,y2 is (%s,%s,%s,%s)" % (x1,y1,x2,y2))
    temp_indexcoord_nonzero=list(indexcoord_nonzero)
    sample_set=[]
    num_inliner_set=[]
    linepara_set=[]
    while len(temp_indexcoord_nonzero)>10:
        # rospy.loginfo("testiter,indexnonzero is (%s,%s)" % (testiter,temp_indexcoord_nonzero))
        throtle=0.1
        num_iter=int(len(temp_indexcoord_nonzero)*0.1)# use int not round, round leads to a float type
        # rospy.loginfo("size,iter is (%s,%s)" % (len(temp_indexcoord_nonzero),num_iter))
        # [sample1,sample2,k_fit,d_fit,num_inliner,index_point_in]=linedetect_one(coordinates,temp_indexcoord_nonzero,throtle,num_iter)
        [sample1,sample2,A_fit,B_fit,C_fit,num_inliner,index_point_in]=linedetect_one(coordinates,temp_indexcoord_nonzero,throtle,num_iter)
        temp_indexcoord_nonzero=[temp for temp in temp_indexcoord_nonzero if temp not in index_point_in] # drop the inliners in the current detection
        # append detected info in lists
        flag_similar_slope=False
        if not linepara_set:
            sample_set.append([sample1,sample2])
            num_inliner_set.append(num_inliner)
            # linepara_set.append([k_fit,d_fit])
            linepara_set.append([A_fit,B_fit,C_fit])
        else:
            for item in linepara_set:
                # if abs((k_fit-item[0])/item[0])<0.2 and abs(k_fit)>1e-3:
                if (item[0]*A_fit+item[1]*B_fit+item[2]*C_fit)/sqrt(A_fit**2+B_fit**2+C_fit**2)/sqrt(item[0]**2+item[1]**2+item[2]**2)>linesimilarity:
                    flag_similar_slope=True
                    break
            if flag_similar_slope==False:
                sample_set.append([sample1,sample2])
                num_inliner_set.append(num_inliner)
                # linepara_set.append([k_fit,d_fit])
                linepara_set.append([A_fit,B_fit,C_fit])

        # rospy.loginfo("iter,sample,size,inliner,kfit is (%s,%s,%s,%s,%s)" % (testiter,[sample1,sample2],num_inliner,index_point_in,[k_fit,d_fit]))
    # rospy.loginfo("flag,sample,size,inliner,kfit is (%s,%s,%s,%s)" % (flag_similar_slope,sample_set,num_inliner_set,linepara_set))

    pub_markers(sample_set,linepara_set,coordinates)

def pack_move(xlinear,ztwist):
    msg=Twist() # format the mesages to be sent
    msg.linear.x=xlinear
    msg.linear.y=0# y velociy does not work
    msg.linear.z=0
    msg.angular.x=0
    msg.angular.y=0
    msg.angular.z=ztwist
    return msg
def perception():
    global linesimilarity # if the corrolation of multiple detected lines is larger than this value,they are considered similar and only one of them is published
    global deltaX # set length of x in published line
    deltaX=2
    linesimilarity=0.7
    rospy.init_node('laser')
    rate=rospy.Rate(5)# lower rate allows more time for calculation
    while not rospy.is_shutdown():
        rospy.Subscriber("/base_scan",LaserScan,linedetect,queue_size=1)
        rate.sleep()
def movement(line_goal):
    rospy.init_node('move')
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber("/base_pose_ground_truth",Odometry,position,line_goal,queue_size=1)
        rate.sleep()
def position(data,line_goal):
    global obj_point
    global state
    pos_x=data.pose.pose.position.x
    pos_y=data.pose.pose.position.y
    rot_q=(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
    rot_e=euler_from_quaternion(rot_q) # euler angular position
    # k=(obj_point[1]-pos_y)/(obj_point[0]-pos_x)
    k=line_goal[0]
    dir_now=rot_e[2]
    if obj_point[0]-pos_x>0:
        dir_obj=atan(k)
    else:
        dir_obj=atan(k)+3.1415926

    # rospy.loginfo("the obj dir is %s,current dir is (%s),state is %s" % (dir_obj,dir_now,state))
    cmd_vel=rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist,queue_size=1)
    cmd = geometry_msgs.msg.Twist()
    if state=="TURN":
        if abs(dir_now-dir_obj)<0.1:
            state2="FORWARD"
        else:
            cmd.angular.z = 1 # set a rotational speed
            cmd.linear.x = 0
    elif state2=="FORWARD":
         cmd.linear.x=1
         cmd.angular.z=0
    cmd_vel.publish(cmd)






if __name__=="__main__":
    # global obj_point
    # global ini_point
    # global state
    # global state2
    # state="GOALSEEK"
    # obj_point=[4.5,9]
    # ini_point=[-8,-2]
    #
    # if state=="GOALSEEK":
    #     state2="TURN"
    #     k=(obj_point[1]-ini_point[1])/(obj_point[0]-ini_point[0])
    #     L_goalseek=[k,ini_point[1]-ini_point[0]*k] # line function for goalseek
        # line function y=-line_goal[0]/line_goal[1]*x-line_goal[2]/line_goal[1]
    perception()
    # movement()
