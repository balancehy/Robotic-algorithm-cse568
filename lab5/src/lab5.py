#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
def coorenv2coormatrix(orienv,coorenv):
    x0=orienv[0]
    y0=orienv[1]
    x=x0-coorenv[1]
    y=y0+coorenv[0]
    return x,y
def coormatrix2coorenv(orienv,coormatrix):# transition from cooridnates in matrix to coordinates in environment
    x0=orienv[0]
    y0=orienv[1]
    y=x0-coormatrix[0]
    x=coormatrix[1]-y0# x in environment
    return x,y
def getCoorenvseq(path,startcoor,goalcoor):# convert path in cell number format to coordinates in environment
    pathenv=[startcoor]
    for item in path:
        x,y=grid2coor(item[0],item[1])# cooridnates in matrix(the map)
        if item[0]==14 and item[1]==7:# avoid crashing into obstacles at (14,7) cell
            x=x+0.25# not use center of cell
            y=y+0.25
        xenv,yenv=coormatrix2coorenv(ORICOORENV,[x,y])
        pathenv.append([xenv,yenv])
    pathenv.append(goalcoor)
    return pathenv
def coor2grid(x_coor,y_coor):# Convert coordinate to cell index.zero based cell number
    x_grid=int(x_coor/CELLX)# for zero-based reason
    y_grid=int(y_coor/CELLY)
    return x_grid,y_grid
def grid2coor(x_grid,y_grid):#convert cell index to coordinates
    x_left=x_grid*CELLX#lower bound of cell
    x_right=x_left+CELLX
    y_left=y_grid*CELLY
    y_right=y_left+CELLY
    x_coor=(x_left+x_right)/2.0 # return middle coordinates of cell
    y_coor=(y_left+y_right)/2.0
    return x_coor,y_coor
def objdetect(x_cur,y_cur,mapnp):# detect surrounding objects, if find obstacles, set value to 1
    objvalue=np.zeros(8)
    for i in range(len(CELLAROUND)):
        x_nei=x_cur+CELLAROUND[i][0]
        y_nei=y_cur+CELLAROUND[i][1]
        if x_nei<0 or x_nei>=mapnp.shape[0] or y_nei<0 or y_nei>=mapnp.shape[1]:# neighbor out of map, continue
            continue
        objvalue[i]=mapnp[x_nei][y_nei]
    # 0 1 2
    # 3   4
    # 5 6 7
    # Above is the index for surrounding cells. Middle is [x_cur,y_cur]
    # if 1-3,3-6,6-4,4-1 are pairs of obstacles, accordingly, 0,5,7,2 are equivalent to obstacles
    # print([x_cur,y_cur],objvalue)
    if (objvalue[1]==1)&(objvalue[3]==1):
        objvalue[0]=1
    if (objvalue[1]==1)&(objvalue[4]==1):
        objvalue[2]=1
    if (objvalue[3]==1)&(objvalue[6]==1):
        objvalue[5]=1
    if (objvalue[4]==1)&(objvalue[6]==1):
        objvalue[7]=1
    return objvalue
def heurcost(currentgrid,goalcoor):# the part of heristic cost in A* algorithm
    x_cur,y_cur=grid2coor(currentgrid[0],currentgrid[1])
    dist=math.sqrt((x_cur-goalcoor[0])**2+(y_cur-goalcoor[1])**2)
    return dist*EPSILON
def pathconstr(parentnode,x_cur,y_cur,start):# append recorded parent node to generate path
    path=[np.array([x_cur,y_cur])]# use list of numpy array to record path
    while x_cur!=start[0] or y_cur!=start[1]:
        parent=parentnode[x_cur][y_cur]
        x_cur=parent[0]
        y_cur=parent[1]
        path.insert(0,parent)
    return path
def A_star(start,goal,goalcoor,mapnp):# this A_star function is adopted from en.wikipedia.org/wiki/A*_search_algorithm
    # the index in A_star function are all cell number counted in map matrix
    x_st=start[0]
    y_st=start[1]
    closeset=np.array([],dtype=np.int32).reshape(0,2) # cell already evaluated and with lowest fscore.
    openset=np.array([],dtype=np.int32).reshape(0,2)
    openset=np.vstack((openset,start))# the priority queue used for selecting the minimum cost in fscore metric(N*2 matrix)
    parentnode=np.empty(mapnp.shape,dtype=np.ndarray) #  numpy array used to record best path
    gscore=np.ones(mapnp.shape)*1e10 # set a large value to all the entries of gscore
    fscore=np.ones(mapnp.shape)*1e10
    gscore[x_st][y_st]=0
    fscore[x_st][y_st]=gscore[x_st][y_st]+heurcost(start,goalcoor)
    while openset.size!=0:# openset not empty
        x_cur,y_cur=openset[np.argmin(fscore[openset.T[0],openset.T[1]]),:]# find the cell number in openset with the lowest fscore. x_cur,y_cur are index in map
        if x_cur==goal[0] and y_cur==goal[1]:
            return pathconstr(parentnode,x_cur,y_cur,start)
        # pop the cell index from openset having lowest fscore, and add it to closeset
        index=np.where(np.all(openset==[x_cur,y_cur],axis=1))[0]
        openset=np.delete(openset,index,axis=0)
        closeset=np.vstack((closeset,[x_cur,y_cur]))
        objvalue=objdetect(x_cur,y_cur,mapnp)# detect obstacles around current cell
        for i in range(len(CELLAROUND)):
            x_nei=x_cur+CELLAROUND[i][0]
            y_nei=y_cur+CELLAROUND[i][1]
            if x_nei<0 or x_nei>=mapnp.shape[0] or y_nei<0 or y_nei>=mapnp.shape[1]:# neighbor out of map, continue
                continue
            if np.where(np.all(closeset==[x_nei,y_nei],axis=1))[0].size!=0:# neighbor exsists in closeset, continue
                continue
            if objvalue[i]==1:# if neighbor is obstacle
                sum_gscore=1e10
            else:
                sum_gscore=gscore[x_cur][y_cur]+PATHLENGTH[i] # gscore of neighbor is sum of current gscore and path cost from current to neighbor
            if np.where(np.all(openset==[x_nei,y_nei],axis=1))[0].size==0:#neighbor is not in openset
                openset=np.vstack((openset,[x_nei,y_nei]))
            elif sum_gscore>=gscore[x_nei][y_nei]:#neighbor is in openset,and its new gscore is more than old gscore
                continue
            parentnode[x_nei][y_nei]=np.array([x_cur,y_cur])
            gscore[x_nei][y_nei]=sum_gscore
            fscore[x_nei][y_nei]=gscore[x_nei][y_nei]+heurcost([x_nei,y_nei],goalcoor)
        # print([x_cur,y_cur],gscore[x_cur-1:x_cur+2,y_cur-1:y_cur+2])
        # print([x_cur,y_cur],fscore[x_cur-1:x_cur+2,y_cur-1:y_cur+2])
        # print(openset.T)
        # print(np.where(fscore==np.min(fscore)))
def pack_move(xlinear,ztwist):# convert to ros message format
    msg=Twist() # format the mesages to be sent
    msg.linear.x=xlinear
    msg.linear.y=0# y velociy does not work
    msg.linear.z=0
    msg.angular.x=0
    msg.angular.y=0
    msg.angular.z=ztwist
    return msg
def normtheta(theta):# make theta in -pi~pi
    if theta>math.pi:
        theta=theta-2*math.pi
    elif theta<-math.pi:
        theta=theta+2*math.pi
    return theta
def move(posc,post,pub,toltran,tolrot):# determine movement according to curretn and target positions
    flag_reach=False
    xc=posc[0]
    yc=posc[1]
    rotc=posc[2]
    xt=post[0]
    yt=post[1]
    if abs(xc-xt)<toltran and abs(yc-yt)<toltran:# reach target
        flag_reach=True# target reached flag set true
        cmd_tran=0.0
        cmd_rot=0.0
    else:# not reach target
        angletarget=math.atan2(yt-yc,xt-xc)
        cmdangel=angletarget-rotc
        # print("command angle",abs(cmdangel))
        cmdangel=normtheta(cmdangel)
        # print("rot tolerance",tolrot)
        if abs(cmdangel)<tolrot:# right direction
            # print("target angle",angletarget)
            cmd_rot=0.0
            cmd_tran=2
        elif cmdangel<-tolrot:# deviated direction. target direct is less than current
            cmd_rot=-1.0# negative rotational speed
            cmd_tran=0.0
        else:
            cmd_rot=1.0
            cmd_tran=0.0
    cmd=pack_move(cmd_tran,cmd_rot)
    pub.publish(cmd)
    return flag_reach
class position():
    def __init__(self):# initialize info from odometry
        self.pos_x=0# x translation
        self.pos_y=0# y translation
        self.rot_e=0# z rotation
        self.sub=rospy.Subscriber("/base_pose_ground_truth",Odometry,self.callback_pos,queue_size=1)# sense position info

    def callback_pos(self,data):# update info from odometry
        self.pos_x=data.pose.pose.position.x
        self.pos_y=data.pose.pose.position.y
        rot_q=(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        self.rot_e=euler_from_quaternion(rot_q)[2] # euler angular position z axis
############################ Begine main script################
################ Set up environmnet ###########################
# load occupancy grid map
    # -9-8-7-6-5-4-3-2-1 1 2 3 4 5 6 7 8 9 #
MAP = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,# 10
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,# 9
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,# 8
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,# 7
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,# 6
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,# 5
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,# 4
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,# 3
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,# 2
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,# 1
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,# -1
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,# -2
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,# -3
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,# -4
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,# -5
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,# -6
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,# -7
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,# -8
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,# -9
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]# -10

mapnp=np.array(MAP).reshape(20,-1)# creat map in numpy array format, this is used in further calculation
CELLX=1.0# cell length in x direct in environment
CELLY=1.0#  cell length in y direct in environment
EPSILON=1.2 # weight on heuristic cost
CELLAROUND=[[-1,-1],[-1,0],[-1,1],[0,-1],[0,1],[1,-1],[1,0],[1,1]]# eight cells around current cell that could possibly be reached
diag=math.sqrt(CELLX**2+CELLY**2)
PATHLENGTH=[diag,CELLY,diag,CELLX,CELLX,diag,CELLY,diag]# eight path cost corresponding to the surrounding cells. Calculate and store them to avoid "online" calculation
ORICOORENV=[10,9]# the coordinates in map for the origin of stageros environment.This is for coordinate frame translation
######################### Path Planning######################
goalenvX=rospy.get_param("goalx")
goalenvY=rospy.get_param("goaly")
startenv=[-8,-2]# set start coordinates in environment
goalenv=[goalenvX,goalenvY]# set goal cooridnates in environment
print("start coordinates in env is ", startenv)
print("goal coordinates in env is ", goalenv)
startmatrix=coorenv2coormatrix(ORICOORENV,startenv)#convert coordinates in environment to map matrix(occupancy grid)
goalmatrix=coorenv2coormatrix(ORICOORENV,goalenv)
print("start coordinates in matrix is ", startmatrix)
print("goal coordinates in matrix is ", goalmatrix)
startcell=coor2grid(startmatrix[0],startmatrix[1])# convert coordinates to cell index
goalcell=coor2grid(goalmatrix[0],goalmatrix[1])
print("start cell number in matrix is ", startcell)
print("goal cell number in matrix is ", goalcell)
print("start estimated cooridnate is ",grid2coor(startcell[0],startcell[1]))
print("goal estimated cooridnate is ",grid2coor(goalcell[0],goalcell[1]))
cellseq=A_star(startcell,goalcell,goalmatrix,mapnp)# implement A* algorithm
print("cell sequence is ", cellseq)
pathenv=getCoorenvseq(cellseq,startenv,goalenv)# get planned path as list of coordinate in environment
print("path is ", pathenv)
########################## Implement path################
rospy.init_node("controller",anonymous=True)
cmd_vel=rospy.Publisher('/cmd_vel',Twist, queue_size=10)
rate=rospy.Rate(10)
current_pos=position()
counter=0
while (not rospy.is_shutdown()) :
    rate.sleep()
    # print(current_pos.pos_x,current_pos.pos_y,current_pos.rot_e)
    posc=[current_pos.pos_x,current_pos.pos_y,current_pos.rot_e]
    post=pathenv[counter]
    flag=move(posc,post,cmd_vel,0.1,0.1)
    if flag==True: # target reached
        counter=counter+1
        if counter<len(pathenv):# end of path is not reached
            post=pathenv[counter]# evaluate new target
        else:# end of path is reached
            break










#
