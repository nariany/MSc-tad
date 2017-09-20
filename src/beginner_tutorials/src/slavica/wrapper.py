#!/usr/bin/env python

import math
import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

class Wrapper(object):
    def __init__(self, mapa):
        self.sirina_pikseli = mapa.info.width
        self.duzina_pikseli = mapa.info.height
        self.pocetak_x= mapa.info.origin.position.x
        self.pocetak_y= mapa.info.origin.position.y
        self.duzina_cm = (abs (mapa.info.origin.position.x))*2
        self.sirina_cm = (abs (mapa.info.origin.position.y))*2
        self.matrica_pikseli = [[True if j!=0 else False for j in mapa.data[i*self.duzina_pikseli:i*self.duzina_pikseli+self.duzina_pikseli]] for i in reversed(xrange(self.sirina_pikseli))]



    def colision(self,x,y,z):
        #self.omjer = 40.0/600.0
        self.piksel_x=int(-14.975*y+299.5)
        self.piksel_y=int(14.975*x+299.5)
        return self.matrica_pikseli[self.piksel_x][self.piksel_y]


x_ciljna=0.0
y_ciljna=0.0
zapamti_x=0.0
zapamti_y=0.0
wrapper=None
pub=None

def pokupi(pose):
    global wrapper
    wrapper=Wrapper(pose)


def ciljna(pose):
    #todo: dodati marker za ciljnu konfiguraciju
    global x_ciljna
    global y_ciljna
    global zapamti_x
    global zapamti_y
    global wrapper
    x_ciljna=pose.point.x
    y_ciljna=pose.point.y
    pointstamped = PointStamped()
    if wrapper.colision(float(x_ciljna),float(y_ciljna),0)==False:
        pointstamped.point.x=x_ciljna
        zapamti_x=x_ciljna
        pointstamped.point.y=y_ciljna
        zapamti_y=y_ciljna
        pub.publish(pointstamped)
    else:
        pointstamped.point.x=zapamti_x
        pointstamped.point.y=zapamti_y


    rate.sleep()



def talker():



    global x_ciljna
    global y_ciljna
    global wrapper
    global zapamti_x
    global zapamti_y
    global pub




if __name__ == '__main__':
    #rospy.Subscriber("/base_pose_ground_truth", Odometry, odometrija)
    rospy.Subscriber("/clicked_point", PointStamped, ciljna)
    rospy.Subscriber("/map", OccupancyGrid, pokupi)
    pub = rospy.Publisher('/ciljna_konfiguracija', PointStamped, queue_size=10)
    rospy.init_node('ciljna_konfiguracija_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
    #except rospy.ROSInterruptException:
    #    pass

#def my_xrange(n):
#    i = 0

#    while i < n:
#        yield i

#        i += 1


#a=[(1, 2, 3), (4, 5, 5)]
#l = [[i for i in a[j][:]] for j in xrange((0:1))]
#print l
#x = [i for i in xrange(10) if i % 2 == 0]
#print x
#w=10
#h=10
#a=range(w*h)
#slavica= Wrapper()
#novi= [[True if j!=0 else False for j in a[i*w:i*w+w]] for i in reversed(xrange(h))]
#print novi

#x = [[True if i%2==0 else False for i in xrange(100)] for j in xrange(100)]
#print x[0][0:5]

#print data
