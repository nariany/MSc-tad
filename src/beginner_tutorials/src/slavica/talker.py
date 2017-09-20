#!/usr/bin/env python

import rospy
import math
import numpy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
#from sensor_msgs-msg import LaserScan

xt=0.0
yt=0.0
xcilj=None
ycilj=None
xcilj_pamti=None
ycilj_pamti=None
orijentacija=0.0
stoj=True

def stani(pose):
    global stoj
    stoj=pose.data

def saznaj_cilj(pose):
    global xcilj
    global ycilj
    xcilj=pose.position.x
    ycilj=pose.position.y
    print xcilj,ycilj

def odometrija(pose):
    global xt
    global yt
    global orijentacija
    xt=pose.pose.pose.position.x
    yt=pose.pose.pose.position.y
    q=pose.pose.pose.orientation
    #OVDJE PISATI KOD
    quaternion = (
        pose.pose.pose.orientation.x,
        pose.pose.pose.orientation.y,
        pose.pose.pose.orientation.z,
        pose.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    orijentacija=euler[2]

def talker():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    zadaj=rospy.Publisher('/cvorko',JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    global xcilj
    global ycilj
    global xt
    global yt
    global orijentacija
    global xcilj_pamti
    global ycilj_pamti

    while not rospy.is_shutdown():
        #if xcilj_pamti==xcilj and ycilj_pamti==ycilj:
        twist = Twist()
        hodaj=JointState()
        hodaj.header.frame_id='odom'
        hodaj.position=[]
        hodaj.position.append(xt)
        hodaj.position.append(yt)
        if xcilj==None and ycilj==None or stoj==True:
            twist.linear.x=0.0
            twist.angular.z=0.0
        else:
            twist.linear.x = 1.0
            theta1=math.atan2((ycilj-yt),(xcilj-xt))
            dst=math.sqrt(math.pow(xcilj-xt,2)+math.pow(ycilj-yt,2))
	    orijentacija=180*orijentacija/numpy.pi;
            theta=180*theta1/numpy.pi;
	    #print(orijentacija-theta);
	    if(dst<0.03):
              twist.angular.x=0;
              twist.angular.y=0;
	      twist.angular.z=0;
	       #print('A');
	      twist.linear.x=0;
	      twist.linear.y=0;
            elif(math.fabs(orijentacija-theta)<2):
	      twist.angular.z=0;
	      twist.linear.x=0.4;
	      #print('B');
            elif(orijentacija>theta or theta-orijentacija>359):
  	      #print('C');
	      twist.angular.z=-0.4;
	      twist.linear.x=0;
            elif(orijentacija<theta or orijentacija-theta>359):
	      twist.angular.z=0.4;
	      twist.linear.x=0;
	      #print('D');
	      twist.linear.y=0;
	#OVDJE PISATI KOD
       
          
        xcilj_pamti=xcilj
        ycilj_pamti=ycilj
        pub.publish(twist)
        zadaj.publish(hodaj)
        rate.sleep()
        

if __name__ == '__main__':
    rospy.Subscriber("/base_pose_ground_truth", Odometry, odometrija)
    rospy.Subscriber("/pozicija",Pose,saznaj_cilj)
    rospy.Subscriber("/reci_mu_da_stane",Bool,stani)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
