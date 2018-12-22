#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Point
import rosbag
import sys
from visualization_msgs.msg import Marker

def extract_baginfo(mybag,bagtopic):
    topics=[]
    messages=[]
    time=[]
    # one movement message: [[rotation vector],translation, [rotation vector2]]
    # one sensor message: [range,[bearing vector],mark number]
    for tpc,m,ti in mybag.read_messages(topics=bagtopic):
        topics.append(tpc)
        time.append(ti.nsecs) # only store nano secs since secs are identical
        if tpc=="Movements":
            rot1_eul=tf.transformations.euler_from_quaternion((m.rotation1.x,m.rotation1.y,m.rotation1.z,m.rotation1.w))
            rot2_eul=tf.transformations.euler_from_quaternion((m.rotation2.x,m.rotation2.y,m.rotation2.z,m.rotation2.w))
            messages.append([[rot1_eul[0],rot1_eul[1],rot1_eul[2]],m.translation,[rot2_eul[0],rot2_eul[1],rot2_eul[2]]])
        elif tpc=="Observations":
            rot_eul=tf.transformations.euler_from_quaternion((m.bearing.x,m.bearing.y,m.bearing.z,m.bearing.w))
            messages.append([m.range,[rot_eul[0],rot_eul[1],rot_eul[2]],m.tagNum])
    return time,topics,messages
def normshiftdegree(theta):# shift degree to [-180,180],note that theta is within[-360,360]
    if theta>180:
        theta=-(360-theta)
    elif theta<-180:
        theta=360+theta
    return theta
def unnormshiftdegree(theta):# shit degree from [-180,180] to [0,360]
    if theta<0:
        theta=360+theta
    return theta
def coor2grid(x_coor,y_coor):# one based cell number
    x_grid=int(x_coor/CELLX)+1# for one-based reason
    y_grid=int(y_coor/CELLY)+1
    return x_grid,y_grid
def grid2coor(x_grid,y_grid):# x_grid=1~35
    x_coor=(x_grid+x_grid-1)*CELLX/2 # return middle coordinates of cell
    y_coor=(y_grid+y_grid-1)*CELLY/2
    return x_coor,y_coor
def grid2degree(theta_grid): # one based grid number for angles
    theta_degree=(theta_grid+theta_grid-1)*CELLTHETA/2
    return theta_degree
def degree2grid(theta_degree):
    theta_grid=int(theta_degree/CELLTHETA)+1
    return theta_grid
def gaussian(x,mu,sig):
    y=math.exp(-(x-mu)**2/(2*sig**2))/(math.sqrt(2*math.pi)*sig)
    return y
def predictPos(pos_cur,prob_cur,motion):
    rot1= 180/math.pi*motion[0][2] # rotation1's rotation, degree
    rot2=180/math.pi*motion[2][2] # rad to degree
    trans=motion[1] # translation
    distrange=int(MAXMAP/CELLX)
    anglerange=int(360/CELLTHETA)
    sig_rot1=NOISEROT1/float(SIGMABOUND)
    sig_trans=NOISETRANS/float(SIGMABOUND)
    sig_rot2=NOISEROT2/float(SIGMABOUND)
    problist=[]
    indexlist=[]
    totalpdf=0
    for i in range(len(pos_cur)):
        x_cell=pos_cur[i][0]
        y_cell=pos_cur[i][1]
        theta_cell=pos_cur[i][2]
        theta_cur=grid2degree(pos_cur[i][2])# ith current theta,scalar [0,360]
        theta_cur=normshiftdegree(theta_cur)# degree[-180,180]
        x_cur,y_cur=grid2coor(pos_cur[i][0],pos_cur[i][1])# scalars
        p_cur=prob_cur[i]
        for j in range(1,distrange+1):
            for k in range(1,distrange+1):
                x_new,y_new=grid2coor(j,k) # in meter
                trans_new=math.sqrt((x_new-x_cur)**2+(y_new-y_cur)**2)
                tempangle=math.atan2(y_new-y_cur,x_new-x_cur)*180/math.pi # angle of new postion to current one in degree [-180,180]
                # if tempangle<0:
                #     tempangle=360+tempangle
                rot1_new=tempangle-theta_cur # in degree [-180,180]
                rot1_new=normshiftdegree(rot1_new)#[-180,180]
                delta=normshiftdegree(rot1_new-rot1)/CELLTHETA# in norm unit
                p_rot1=gaussian(delta,0,sig_rot1) # pdf, assume segment lengths are same for all probability calculation(probability=pdf*segmentlength).After normalization, the segment length will be cancelled
                delta=(trans_new-trans)/CELLX # in norm unit
                p_trans=gaussian(delta,0,sig_trans)
                for q in range(1,anglerange+1):
                    theta_new=grid2degree(q)# in degree
                    theta_new=normshiftdegree(theta_new)
                    rot2_new=normshiftdegree(theta_new-tempangle)
                    delta=normshiftdegree(rot2_new-rot2)/CELLTHETA
                    p_rot2=gaussian(delta,0,sig_rot2)
                    p_new=p_cur*p_rot1*p_trans*p_rot2
                    if i==0:
                        indexlist.append([j,k,q])
                        problist.append(p_new)
                    else:
                        problist[(j-1)*distrange*anglerange+(k-1)*anglerange+q-1]=problist[(j-1)*distrange*anglerange+(k-1)*anglerange+q-1]+p_new
                    totalpdf=totalpdf+p_new
    normproblist=normProb(problist,totalpdf)
    return indexlist,normproblist
def normProb(problist,totalpdf):# normalized each pdf by totalpdf,segment lengths are cancelled
    normprob=[0]*len(problist)
    for i in range(len(problist)):
        normprob[i]=problist[i]/totalpdf
    return normprob
def droptrivial(x,prob,toler):# drop small probability, normalize
    tempx=[]
    tempprob=[]
    sum=0
    for i in range(len(x)):
        if prob[i]>toler:# keep probability larger than tol
            tempx.append(x[i])
            tempprob.append(prob[i])
            sum=sum+prob[i]

    for i in range(len(tempx)):# normalize
        tempprob[i]=tempprob[i]/sum
    return tempx,tempprob
def updatePos(pos_pred,prob_pred,sensing):
    range0=sensing[0]# avoid same name as range function
    bearing=sensing[1][2]*180/math.pi # bearing in degree [-180,180]
    tag_x=TAGCOOR[sensing[2]][0]
    tag_y=TAGCOOR[sensing[2]][1]
    sig_bearing=NOISEBEARING/float(SIGMABOUND)#variance of bearning
    sig_range=NOISERANGE/float(SIGMABOUND)# variance of range
    # loop to find the distribution at each oberservation and prior state
    problist=[]
    totalpdf=0
    for i in range(len(pos_pred)):
        x_pred,y_pred=grid2coor(pos_pred[i][0],pos_pred[i][1]) # predicted rotot coordinates
        theta_pred=grid2degree(pos_pred[i][2])# robot rotational position in degree [0,360]
        theta_pred=normshiftdegree(theta_pred)
        p_pred=prob_pred[i]
        range_ob=math.sqrt((tag_x-x_pred)**2+(tag_y-y_pred)**2)
        tempangle=math.atan2(tag_y-y_pred,tag_x-x_pred)*180/math.pi# [-180,180]
        bearing_ob=normshiftdegree(tempangle-theta_pred)
        delta=normshiftdegree(bearing_ob-bearing)/CELLTHETA
        p_bearing=gaussian(delta,0,sig_bearing)
        delta=(range_ob-range0)/CELLX
        p_range=gaussian(delta,0,sig_range)
        p_new=p_pred*p_bearing*p_range
        problist.append(p_new)
        totalpdf=totalpdf+p_new
    return normProb(problist,totalpdf)
def pubtags(pub):# publish tag coordinates
    points=Marker()
    points.header.frame_id="/Start"
    points.header.stamp=rospy.Time.now()
    # points.lifetime=rospy.Duration(2.5)
    points.action=0 # ADD
    points.id=0
    points.type=Marker.POINTS
    points.scale.x=0.1
    points.scale.y=0.1
    points.color.g=1.0
    points.color.a=1
    for kk in range(len(TAGCOOR)):# append all tags' coordinates
        pub_p=Point()
        pub_p.x=TAGCOOR[kk][0]
        pub_p.y=TAGCOOR[kk][1]
        pub_p.z=0
        points.points.append(pub_p)
    pub.publish(points)
def pubpath(pub,pos):#publish path
    paths.header.frame_id="/Start"
    paths.header.stamp=rospy.Time.now()
    paths.action=0 # ADD
    paths.id=1
    paths.type=Marker.LINE_STRIP# use line segments
    paths.scale.x=0.1
    paths.scale.y=0.1
    paths.color.r=1
    paths.color.a=1
    x,y=grid2coor(pos[0],pos[1])# convert cell number to coordinates
    pub_p=Point()
    pub_p.x=x
    pub_p.y=y
    pub_p.z=0
    paths.points.append(pub_p)# paths is a global variable,keep appending the points and publish them
    # print(paths.points)
    pub.publish(paths)

def findmaxprob(pos,prob):# find max probability and position
    max_prob=max(prob)
    for i in range(len(prob)):
        if prob[i]==max_prob:
            break
    max_pos=pos[i]
    # print(max_prob)
    return max_pos,max_prob
####
print("###########initilization begins##########")
global CELLX# resolution of x direction, in meter
global CELLY# resolution of y direction, in meter
global CELLTHETA # resolution of rotation, in degree
global MAXMAP # maxium size of the map in meter
global SIGMABOUND # define the effective boundary of gaussian function by its variance sigma, [-SIGMABOUND*sigma,+SIGMABOUND*sigma]
global NOISEROT1
global NOISEROT2
global NOISETRANS
global NOISEBEARING
global NOISERANGE
global TAGCOOR# the coordinates of tags
global paths
global GRANGE # maxium range of gaussian noise funciton
paths= Marker()# define path as global marker
CELLX=0.2 # meter
CELLY=0.2
CELLTHETA=20 # degree for discretizing theta
MAXMAP=7 # meter
SIGMABOUND=3
noise_move=2# dimensionless( depends on cell size or angle size) noise magnitude for one side. use noise_move=SIGMABOUND*sigma to calculate the variance sigma
NOISEROT1=noise_move # noise for rotaion1
NOISEROT2=noise_move
NOISETRANS=noise_move
noise_sense=1.5 # noise for sensor model
NOISEBEARING=noise_sense
NOISERANGE=noise_sense
TAGCOOR=[[1.25,5.25],[1.25,3.25],[1.25,1.25],[4.25,1.25],[4.25,3.25],[4.25,5.25]]
# print("system arguments",sys.argv[0])
bagtopic=["Movements","Observations"]
mybag=rosbag.Bag(sys.argv[1])
# mybag=rosbag.Bag("/home/first/myros/src/lab4/src/grid.bag")
time,topics,messages=extract_baginfo(mybag,bagtopic)# extract information from bag

initgrid=[[12,28,21]]# initilize positon info
initprob=[1]# initilize probability of position
rospy.init_node("Bayes")
rate=rospy.Rate(1)
tags=rospy.Publisher("/visualization_marker",Marker,queue_size=10)
path=rospy.Publisher("/visualization_marker",Marker,queue_size=10)

i=0
pos_post=initgrid# firth posterior is set to initial value
prob_post=initprob
tolerance=1e-3# tolerance for dropping small probabilities
print("###########initialization finished##########")
trajectory=[]
flagstop=False
# print(sys.argv[0],sys.argv[1],sys.argv[2])
while (not rospy.is_shutdown())&(not flagstop):#
    rate.sleep()
    pubtags(tags)# publish tags coordinates to rviz
    if i < len(messages)/2:# separate message into movements and observations
        print("%sth model INFO" % (i))
        print("motion is ",[messages[2*i][0][2],messages[2*i][1],messages[2*i][2][2]])
        print("Observation is ",[messages[2*i+1][0],messages[2*i+1][1][2],messages[2*i+1][2]])
        sum=0
        for item in prob_post:
            sum=sum+item
        print("sum origi is:",sum)
        print("max origi is ",max(prob_post))
        pos_prior,prob_prior=predictPos(pos_post,prob_post,messages[2*i])# calculate probability for movement prediction
        pos_prior,prob_prior=droptrivial(pos_prior,prob_prior,tolerance)# drop small probability and normalize the prior
        print("%sth prior" % (i))
        print("length of prior list: ",len(pos_prior))
        maxpos_prior,maxprob_prior=findmaxprob(pos_prior,prob_prior)
        print("max prior info",maxpos_prior,maxprob_prior)# print maximum prior and its postion
        # pubpath(path,maxpos_prior)

        ## Update prior for posterior
        prob_post=updatePos(pos_prior,prob_prior,messages[2*i+1])
        pos_post=pos_prior
        pos_post,prob_post=droptrivial(pos_post,prob_post,tolerance)
        print("%sth posterior" % (i))
        print("length of posterior list: ",len(prob_post))
        maxpos_post,maxprob_post=findmaxprob(pos_post,prob_post)# find maximum posterior and its position
        print("max posterior info: ",maxpos_post,maxprob_post)
        pubpath(path,maxpos_post)# publish position with maximum posterior
        trajectory.append(maxpos_post)
        i=i+1
    else:
        print(sys.argv[0])
        filepath=sys.argv[2]+"trajectory.txt"
        with open(filepath,"w") as outf:
            for item in trajectory:
                outf.write(str(item)+"\n")
        outf.close()
        flagstop=True
        print(" trajectory printed")
