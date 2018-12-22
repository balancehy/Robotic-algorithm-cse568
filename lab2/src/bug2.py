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
    deltaX=2
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


def pack_move(xlinear,ztwist):
    msg=Twist() # format the mesages to be sent
    msg.linear.x=xlinear
    msg.linear.y=0# y velociy does not work
    msg.linear.z=0
    msg.angular.x=0
    msg.angular.y=0
    msg.angular.z=ztwist
    return msg

class position():
    def __init__(self):
        self.pos_x=0
        self.pos_y=0
        self.rot_e=(0,0,0)
        self.sub=rospy.Subscriber("/base_pose_ground_truth",Odometry,self.callback_pos,queue_size=1)# sense position info

    def callback_pos(self,data):
        self.pos_x=data.pose.pose.position.x
        self.pos_y=data.pose.pose.position.y
        rot_q=(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        self.rot_e=euler_from_quaternion(rot_q) # euler angular position
        # k=(obj_point[1]-pos_y)/(obj_point[0]-pos_x)
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


def p2l_dist(line_fun,point): # line and point must be in the same frame
    A=line_fun[0]
    B=line_fun[1]
    C=line_fun[2]
    if B==0:
        distance=abs(-C/A)
    else:
        k=-A/B
        costheta=1/sqrt(1+k**2)
        sintheta=k/sqrt(1+k**2)
        distance=abs(-(point[0]+C/A)*sintheta+point[1]*costheta)
    return distance
def movement(cmd_vel,cmd,dir_obj,dir_now):
    state_move=""
    tolerance=0.1
    rotspeed=0.6
    if ((dir_obj-dir_now)>=tolerance) & ((dir_obj-dir_now)<3.14159):
        cmd.angular.z = rotspeed # set a rotational speed
        cmd.linear.x = 0
        state_move="TURN"
    elif ((dir_obj-dir_now)<-tolerance) & ((dir_obj-dir_now)>=-3.14159):
        cmd.angular.z = -rotspeed # set a rotational speed
        cmd.linear.x = 0
        state_move="TURN"
    # if abs(dir_now-dir_obj)>tolerance:
    #     cmd.angular.z = rotspeed # set a rotational speed
    #     cmd.linear.x = 0
    #     state_move="TURN"
    elif abs(dir_now-dir_obj)<=tolerance:
        cmd.linear.x=3.5
        cmd.angular.z=0
        state_move="FORWARD"
    else:
        cmd.angular.z = rotspeed
        cmd.linear.x = 0
        state_move="TURN"
    cmd_vel.publish(cmd)
    return state_move
def movement_circle(cmd_vel,cmd,dir_obj,dir_now):
    state_move=""
    tolerance=0.1
    if dir_obj-dir_now>=tolerance:
        cmd.angular.z = 0.85
        cmd.linear.x = 1
        state_move="CIRCLE"
    else:
        cmd.angular.z = 0
        cmd.linear.x =3
        state_move="FORWARD"
    cmd_vel.publish(cmd)
    return state_move
if __name__=="__main__":
    global obj_point
    global ini_point
    global state
    global state2
    global angle_split_wall
    dir_wallfollow="RIGHT"
    state="GOALSEEK"
    state2="TURN"
    Flag_GOALSEEK2WALLFOLLOW=False
    Flag_WALLFOLLOW2GOALSEEK=True
    Flag_test=False
    obj_point=[4.5,9]
    ini_point=[-8,-2]
    line_goal=[(obj_point[1]-ini_point[1])/(obj_point[0]-ini_point[0]),-ini_point[0]*(obj_point[1]-ini_point[1])/(obj_point[0]-ini_point[0])+ini_point[1]]
    anglearray_split_wall=10*360/180 # used for only detecting lines on one side
    dist_approach_wall=0.9
    dist_leave_wall=1.2
    dist_linegoal=0.5
    # parameters for line detection
    linesimilarity=0.8# if the corrolation of multiple detected lines is larger than this value,they are considered similar and only one of them is published
    rospy.init_node('controller')
    rate=rospy.Rate(4)
    current_scan=scan(linesimilarity)
    current_pos=position()
    while not rospy.is_shutdown():
        rate.sleep()
        # rospy.loginfo("self is (%s,%s)" % (current_scan.linepara_set,current_scan.sample_set))
        pub_markers(current_scan.sample_set,current_scan.linepara_set,current_scan.coordinates)
        # rospy.loginfo("the position is (%s,%s,%s)\nlinepara is %s" % (current_pos.pos_x,current_pos.pos_y,current_pos.rot_e,current_scan.linepara_set))
        cmd_vel=rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist,queue_size=1)
        cmd = geometry_msgs.msg.Twist()
        p2linegoal=p2l_dist((line_goal[0],-1,line_goal[1]),[current_pos.pos_x,current_pos.pos_y])# calculate the distance between current point and line_goal
        # get deteted lines using laserscan
        if state=="GOALSEEK":

            if not current_scan.linepara_set:
                state="GOALSEEK"
                # Flag_WALLFOLLOW2GOALSEEK=False
            else:

                if len(current_scan.linepara_set)>=2:
                    p2l_1=p2l_dist(current_scan.linepara_set[0],[0,0])
                    p2l_2=p2l_dist(current_scan.linepara_set[1],[0,0])
                    if p2l_1<p2l_2:
                        p2l=p2l_1
                        index=0
                    else:
                        p2l=p2l_2
                        index=1
                    line_wall=current_scan.linepara_set[index]
                else:
                    line_wall=current_scan.linepara_set[0]
                p2l=p2l_dist(line_wall,[0,0])
                # rospy.loginfo("Distance %s,%s" % (line_wall,p2l))
                if p2l<dist_approach_wall:
                    if Flag_WALLFOLLOW2GOALSEEK==True:
                        state="GOALSEEK"
                    else:

                        state="WALLFOLLOW"
                        Flag_GOALSEEK2WALLFOLLOW=True

                else:
                    state="GOALSEEK"
                    Flag_WALLFOLLOW2GOALSEEK=False# left wall
        elif state=="WALLFOLLOW":
            if not current_scan.linepara_set:
                state="GOALSEEK"
            else:
                if len(current_scan.linepara_set)>=2:
                    p2l_1=p2l_dist(current_scan.linepara_set[0],[0,0])
                    p2l_2=p2l_dist(current_scan.linepara_set[1],[0,0])
                    if p2l_1<p2l_2:
                        p2l=p2l_1
                        index=0
                    else:
                        p2l=p2l_2
                        index=1
                    if p2l<=dist_approach_wall+0.02:
                        line_wall=current_scan.linepara_set[index]
                    else:
                        line_wall=[]

                else:
                    p2l=p2l_dist(current_scan.linepara_set[0],[0,0])
                    if p2l<=dist_approach_wall+0.02:
                        line_wall=current_scan.linepara_set[0]
                    else:
                        line_wall=[]
                # x_inter=-(line_wall[1]*line_goal[1]-line_wall[2])/(line_wall[0]+line_wall[1]*line_goal[0])
                # y_inter=line_goal[0]*x_inter+line_goal[1]
                if p2linegoal<dist_linegoal: # wallfollow and goalseek satisfied
                    if Flag_GOALSEEK2WALLFOLLOW==True:
                        state="WALLFOLLOW"
                    else:# from wallfollow to goalseek
                        state="GOALSEEK"
                        Flag_WALLFOLLOW2GOALSEEK=True

                else: # normal wallfollow
                    state="WALLFOLLOW"
                    Flag_GOALSEEK2WALLFOLLOW=False# left line_goal


        # find current angular direction in world frame
        dir_now=current_pos.rot_e[2]
        # if obj_point[0]-ini_point[0]>0:
        #     angle_line_goal=atan(line_goal[0])
        # else:
        #     angle_line_goal=atan(line_goal[0])+3.14159


        # calculate objective direction for two states
        if state=="GOALSEEK":

            if p2linegoal>dist_linegoal:
                k=-1/line_goal[0]
                if current_pos.pos_y>line_goal[0]*current_pos.pos_x+line_goal[1]: # current point is above line_goal
                    dir_obj=atan(k)
                else:
                    dir_obj=atan(k)+3.14159
            else:
                k=line_goal[0]
                dir_obj=atan(k)

            state2=movement(cmd_vel,cmd,dir_obj,dir_now)

        elif state=="WALLFOLLOW":

            if not line_wall:
                # dir_obj=0.5
                Flag_test=True
                # k=(obj_point[1]-current_pos.pos_y)/(obj_point[0]-current_pos.pos_x)
                # k=-1/line_goal[0]
                k=3.14159/2+0.8
                if k>0:
                    dir_obj=atan(k)-dir_now
                else:
                    dir_obj=atan(k)+3.14159-dir_now
            elif (current_scan.coordinates[180][0]==0)&(current_scan.coordinates[180][1]==0)&(-0.1<current_pos.rot_e[2]<0.05):
                dir_obj=0
                Flag_test=False
            elif (current_scan.coordinates[180][0]==0)&(current_scan.coordinates[180][1]==0)&(1.1<current_pos.rot_e[2]<1.39):
                dir_obj=0
                Flag_test=False
            elif (current_scan.coordinates[180][0]==0)&(current_scan.coordinates[180][1]==0)&(2.3<current_pos.rot_e[2]<2.65):
                dir_obj=0
                Flag_test=False
            else:
                Flag_test=False
                if line_wall[1]==0: # A B C ->line_wall[0][1][2], which are in local frame
                    if dir_wallfollow=="LEFT":
                        dir_obj=3.14159/2
                    elif dir_wallfollow=="RIGHT":
                        if -line_wall[2]/line_wall[0]>0: # x intersection is larger than zero,right hand side is negative direction
                            dir_obj=-3.14159/2
                        else:
                            dir_obj=3.14159/2
                else:
                    k=-line_wall[0]/line_wall[1]
                    if dir_wallfollow=="LEFT":
                        dir_obj=atan(k)

                    elif dir_wallfollow=="RIGHT":
                        if -line_wall[2]/line_wall[1]>0: # -C/B>0, postive y intersection
                            dir_obj=atan(k)
                        else:
                            dir_obj=3.14159+atan(k)
                # rospy.loginfo("##the obj dir is %s,current dir is (%s)" % (dir_obj,dir_now))
            dir_obj=dir_obj+dir_now # convert local angle to global angle
            if dir_obj>=3.14159*2:
                dir_obj=dir_obj-3.14159*2
            if dir_obj>3.14159:
                dir_obj=-3.14159*2+dir_obj
            if Flag_test==True:
                state2=movement_circle(cmd_vel,cmd,dir_obj,dir_now)
            else:
                state2=movement(cmd_vel,cmd,dir_obj,dir_now)
        if (abs(current_pos.pos_x-obj_point[0])<0.5) & (abs(current_pos.pos_y-obj_point[1])<0.5):
            cmd.angular.z = 0
            cmd.linear.x =0
            cmd_vel.publish(cmd)
            state="STOP"
        rospy.loginfo("CHECK point### (%s,%s,%s)" % (Flag_WALLFOLLOW2GOALSEEK,Flag_GOALSEEK2WALLFOLLOW,Flag_test))
        rospy.loginfo("the obj dir is %s,current dir is (%s)" % (dir_obj,dir_now))
        rospy.loginfo("CHECK point (%s,%s)" % (state,state2))
        rospy.loginfo("pos,scan (%s,%s)" % (current_pos.rot_e[2],current_scan.coordinates[180]))

    # perception()
