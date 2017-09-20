#!/usr/bin/env python

import math
import rospy

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped, Point, Pose, PoseArray
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker



x_ciljna=0.0
y_ciljna=0.0
zapamti_x=0.0
zapamti_y=0.0
pomocna=0
wrapper=None
epsilon=0.4
stani=None
pub=None
niz=None

class Wrapper(object):
    def __init__(self, mapa):
        self.sirina_pikseli = mapa.info.width
        self.duzina_pikseli = mapa.info.height
        self.pocetak_x= mapa.info.origin.position.x
        self.pocetak_y= mapa.info.origin.position.y
        self.duzina_m = (abs (mapa.info.origin.position.x))*2
        self.sirina_m = (abs (mapa.info.origin.position.y))*2
        self.matrica_pikseli = [[True if j!=0 else False for j in mapa.data[i*self.duzina_pikseli:i*self.duzina_pikseli+self.duzina_pikseli]] for i in reversed(xrange(self.sirina_pikseli))]

    def colision(self,x,y):
        #self.omjer = 40.0/600.0 dimenzije za 600 i 20 m
        #piksel_x=int(-14.975*y+299.5)
        #todo: prosiriti na vise piksela i raspodjelu drugu uzeti tj. seedanje raspodjele
            #proba=(-12.2, 19.9)
        piksel_x_lijevo=int(self.sirina_pikseli*(y-0.3-self.pocetak_y)/(-self.sirina_m))
        piksel_x_desno=int(self.sirina_pikseli*(y+0.3-self.pocetak_y)/(-self.sirina_m))
        piksel_y_dole=int(self.duzina_pikseli*(x-0.3-self.pocetak_x)/self.duzina_m)
        piksel_y_gore=int(self.duzina_pikseli*(x+0.3-self.pocetak_x)/self.duzina_m)
        piksel_x=int(self.sirina_pikseli*(y-self.pocetak_y)/(-self.sirina_m))
        piksel_y=int(self.duzina_pikseli*(x-self.pocetak_x)/self.duzina_m)
        if abs(piksel_x) > len(self.matrica_pikseli[0])-1 or abs(piksel_y) > len(self.matrica_pikseli)-1:
            #print 'prvo'
            return True
        elif abs(piksel_y_dole) > len(self.matrica_pikseli)-1 or abs(piksel_y_gore) > len(self.matrica_pikseli)-1:
            #print 'drugo'
            return True
        elif abs(piksel_x_desno) > len(self.matrica_pikseli[0])-1 or abs(piksel_x_lijevo) > len(self.matrica_pikseli[0])-1:
            #print 'trece'
            return True
        else:
            #print piksel_x, piksel_y
            prva = self.matrica_pikseli[piksel_x][piksel_y_dole] or self.matrica_pikseli[piksel_x][piksel_y_gore]
            druga=self.matrica_pikseli[piksel_x_lijevo][piksel_y] or self.matrica_pikseli[piksel_x_desno][piksel_y]
            treca=self.matrica_pikseli[piksel_x_lijevo][piksel_y_gore] or self.matrica_pikseli[piksel_x_lijevo][piksel_y_dole]
            cetvrta=self.matrica_pikseli[piksel_x_desno][piksel_y_gore] or self.matrica_pikseli[piksel_x_desno][piksel_y_dole]
            return (self.matrica_pikseli[piksel_x][piksel_y] or prva or druga or treca or cetvrta)

def pokupi_mapu(pose):
    global wrapper
    wrapper=Wrapper(pose)


def pokupi(pose):
    global niz
    global pomocna
    global stani
    pomocna=0
    niz=[]

    for i in xrange(0,len(pose.poses)):
        tacka=(pose.poses[i].position.x, pose.poses[i].position.y)
        niz.append(tacka)

    smoto3 = rospy.Publisher('/linije3', Marker, queue_size=10)
    rate = rospy.Rate(10)
    marker_linije3=Marker()
    marker_linije3.type=5
    marker_linije3.header.frame_id='odom'
    #marker_linije.action=marker.ADD
    marker_linije3.scale.x=0.7
    marker_linije3.color.r=0.2
    marker_linije3.color.g=0.2
    marker_linije3.color.b=0.2
    marker_linije3.color.a=1.0
    #post_definition()
    print 'konacno'
    print niz

    for i in xrange(0,len(niz)-1):
        tacka_pocetna=Point()
        tacka_krajnja=Point()
        tacka_pocetna.x=niz[i][0]
        tacka_pocetna.y=niz[i][1]
        tacka_krajnja.x=niz[i+1][0]
        tacka_krajnja.y=niz[i+1][1]
        marker_linije3.points.append(tacka_pocetna)
        marker_linije3.points.append(tacka_krajnja)
    print marker_linije3.points
    #smoto3.publish(marker_linije3)
    rate.sleep()
    loptica=Bool()
    #todo:prebbaciti na False
    loptica.data=False
    stani.publish(loptica)


def post_definition():
	#probni dio koda za modifikaciju putanje
    global wrapper
    global niz
    print niz
    brojac_pocetak=0
    brojac_kraj=len(niz)-1
    while(True):
        pamti=1
        print brojac_kraj
        if brojac_kraj==brojac_pocetak+1 or brojac_kraj<=brojac_pocetak:
            break
        print spoji(niz[brojac_pocetak],niz[brojac_kraj])
        if spoji(niz[brojac_pocetak],niz[brojac_kraj])!=None:
            print "usao"
            pamti=0
            for i in xrange(brojac_pocetak+1,brojac_kraj):
                pamti+=1
                print niz[brojac_pocetak+1]
                niz.pop(brojac_pocetak+1)
        brojac_kraj-=pamti
    brojac_kraj=len(niz)-1
    while(True):
        pamti=1
        print 'pocetak se mijenja'
        print brojac_pocetak
        if brojac_kraj==brojac_pocetak+1 or brojac_kraj<=brojac_pocetak:
            break
        if spoji(niz[brojac_pocetak],niz[brojac_kraj])!=None:
            print "usao"
            pamti=0
            for i in xrange(brojac_pocetak+1,brojac_kraj):
                pamti+=1
                print niz[brojac_pocetak+1]
                niz.pop(brojac_pocetak+1)
        brojac_pocetak+=pamti


def daj_tacku(trenutna_x,trenutna_y):
    #global cilj
    global niz
    global pomocna
    #print 'trenutna'
    #print (trenutna_x,trenutna_y)
    #print 'cilj'
    #print niz
    if niz is not None:
        i=pomocna
        if trenutna_x > (niz[i][0]-0.2) and trenutna_x < (niz[i][0]+0.2) and trenutna_y > (niz[i][1]-0.2) and trenutna_y < (niz[i][1]+0.2) and i<len(niz)-1:

            pomocna+=1
            return niz[i+1]
        #for i in xrange(0,len(niz)-1):
        #    if trenutna_x > (niz[i][0]-0.2) and trenutna_x < (niz[i][0]+0.2) and trenutna_y > (niz[i][1]-0.2) and trenutna_y < (niz[i][1]+0.2):
        #        return niz[i+1]
        #    else:
        #        return None
    else:
        return None

def euklidska(tacka1,tacka2):
    return math.sqrt((tacka2[1] - tacka1[1]) ** 2 + (tacka2[0] - tacka1[0]) ** 2)


def kombinezon(p, q, alpha):
    assert len(p) == len(q), 'Points must have the same dimension'
    return tuple([(1 - alpha) * p[i] + alpha * q[i] for i in xrange(len(p))])

def spoji (p1,p2):
    global epsilon
    global wrapper

    d=euklidska(p1,p2)

    if d<epsilon:
        return (1.0,0.0)

    alpha_i=epsilon/d
    n=int(d/epsilon)

    zapamti=None
    for i in xrange(1,n):
        alpha=alpha_i*i
        novo=kombinezon(p1,p2,alpha)
        if wrapper.colision(float(novo[0]),float(novo[1])):
            return None
        else:
            zapamti=novo
    return zapamti


def ciljna(pose):
    #todo: dodati marker za ciljnu konfiguraciju
    global x_ciljna
    global y_ciljna
    global orjentacija
    global niz
    global pub

    #global rate
    xt=pose.pose.pose.position.x
    yt=pose.pose.pose.position.y
    q=pose.pose.pose.orientation
    position=Pose()
    orjentacija = math.degrees(math.atan2(2*(q.x*q.y+q.z*q.w),1-2*(pow(q.y,2)+pow(q.z,2))))
    znanje=daj_tacku(xt,yt)
    if znanje!=None:
        position.position.x=znanje[0]
        position.position.y=znanje[1]
        print znanje
        pub.publish(position)
        rate.sleep()
    #x_ciljna=pose.point.x
    #y_ciljna=pose.point.y
    #pointstamped = PointStamped()
    #if wrapper.colision(float(x_ciljna),float(y_ciljna),0)==False:
    #    pointstamped.point.x=x_ciljna
    #    zapamti_x=x_ciljna
    #    pointstamped.point.y=y_ciljna
    #    zapamti_y=y_ciljna
    #    pub.publish(pointstamped)
    #else:
    #    pointstamped.point.x=zapamti_x
    #    pointstamped.point.y=zapamti_y






def talker():



    global x_ciljna
    global y_ciljna
    global wrapper
    global zapamti_x
    global zapamti_y
    global pub
    #global rate


#provjeri na sta salje i kako se dobija poses[i].position.x da pokupis

if __name__ == '__main__':
    #rospy.Subscriber("/base_pose_ground_truth", Odometry, odometrija)
    pub = rospy.Publisher('/pozicija', Pose, queue_size=10)
    cilj=rospy.Publisher('/pocetna', Pose, queue_size=10)
    stani=rospy.Publisher('/reci_mu_da_stane', Bool, queue_size=10)
    rospy.init_node('pozicija_node', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, ciljna)
    rospy.Subscriber("/ciljevi", PoseArray, pokupi)
    rospy.Subscriber("/map", OccupancyGrid, pokupi_mapu)

    while niz is None:
        rate.sleep()
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
