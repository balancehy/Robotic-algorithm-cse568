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
    point_line=rospy.Publisher("/visualization_marker",Marker,queue_size=10)
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
    line_list.scale.x=0.1
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


def perception():

    rospy.init_node('laser')
    rate=rospy.Rate(5)# lower rate allows more time for calculation
    while not rospy.is_shutdown():
        rospy.Subscriber("/base_scan",LaserScan,linedetect,queue_size=1)
        rate.sleep()

class scan():
    def __init__(self,linesimilarity):
        # global deltaX # set length of x in published line
        # deltaX=2
        self.linesimilarity = linesimilarity
        self.linepara_set=[]
        self.sample_set=[]
        self.coordinates=[]
        self.sub=rospy.Subscriber("/base_scan",LaserScan,self.linedetect,queue_size=10)
        # rospy.loginfo("self ini is (%s)" % (self.linepara_set))
    def linedetect(self,data):
        num_inliner_set=[]
        linepara_set=[]
        sample_set=[]
        angle_min=data.angle_min # start from -pi/2
        angle_max=data.angle_max # end at 1/2*pi
        ranges=data.ranges
        angle_increment=data.angle_increment

        range_max=data.range_max
        range_min=data.range_min
        totalangle=360
        angles=[angle_min+(angle_max-angle_min)/totalangle*i for i in range(totalangle+1)]# sweep angle from pi/2 to -pi/2 to make it consisent with ranges
        [coordinates,indexcoord_nonzero]=coord_coversion(ranges,angles)
            # rospy.loginfo("x1,y1,x2,y2 is (%s,%s,%s,%s)" % (x1,y1,x2,y2))
        temp_indexcoord_nonzero=list(indexcoord_nonzero)
        while len(temp_indexcoord_nonzero)>10:
            # rospy.loginfo("testiter,indexnonzero is (%s,%s)" % (testiter,temp_indexcoord_nonzero))
            throtle=0.1
            num_iter=int(len(temp_indexcoord_nonzero)*0.2)# use int not round, round leads to a float type
            [sample1,sample2,A_fit,B_fit,C_fit,num_inliner,index_point_in]=linedetect_one(coordinates,temp_indexcoord_nonzero,throtle,num_iter)
            # rospy.loginfo("fast self is (%s)" % (current_scan.linepara_set))
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
                    if abs(item[0]*A_fit+item[1]*B_fit+item[2]*C_fit)/sqrt(A_fit**2+B_fit**2+C_fit**2)/sqrt(item[0]**2+item[1]**2+item[2]**2)>linesimilarity:
                        flag_similar_slope=True
                        break
                if flag_similar_slope==False:
                    sample_set.append([sample1,sample2])
                    num_inliner_set.append(num_inliner)
                    # linepara_set.append([k_fit,d_fit])
                    linepara_set.append([A_fit,B_fit,C_fit])
        self.linepara_set=list(linepara_set) # use evaluation , not self.linepara.append
        self.sample_set=list(sample_set)
        self.coordinates=list(coordinates)


if __name__=="__main__":
    global linesimilarity # if the corrolation of multiple detected lines is larger than this value,they are considered similar and only one of them is published
    global deltaX # set length of x in published line
    deltaX=2
    linesimilarity=0.8
    rospy.init_node('sense')
    rate=rospy.Rate(4)
    current_scan=scan(linesimilarity)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.loginfo("self is (%s,%s)" % (current_scan.linepara_set,current_scan.sample_set))
        pub_markers(current_scan.sample_set,current_scan.linepara_set,current_scan.coordinates)

    # perception()
    # movement()
