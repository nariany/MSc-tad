#!/usr/bin/env python

import math
import rospy
import random
import numpy


from scipy import spatial
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped, Point, Pose, PoseArray
from std_msgs.msg import Bool


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
        piksel_x_lijevo=int(self.sirina_pikseli*(y-0.3-self.pocetak_y)/(-self.sirina_m))
        piksel_x_desno=int(self.sirina_pikseli*(y+0.3-self.pocetak_y)/(-self.sirina_m))
        piksel_y_dole=int(self.duzina_pikseli*(x-0.3-self.pocetak_x)/self.duzina_m)
        piksel_y_gore=int(self.duzina_pikseli*(x+0.3-self.pocetak_x)/self.duzina_m)
        piksel_x=int(self.sirina_pikseli*(y-self.pocetak_y)/(-self.sirina_m))
        piksel_y=int(self.duzina_pikseli*(x-self.pocetak_x)/self.duzina_m)
        prva = self.matrica_pikseli[piksel_x][piksel_y_dole] or self.matrica_pikseli[piksel_x][piksel_y_gore]
        druga=self.matrica_pikseli[piksel_x_lijevo][piksel_y] or self.matrica_pikseli[piksel_x_desno][piksel_y]
        treca=self.matrica_pikseli[piksel_x_lijevo][piksel_y_gore] or self.matrica_pikseli[piksel_x_lijevo][piksel_y_dole]
        cetvrta=self.matrica_pikseli[piksel_x_desno][piksel_y_gore] or self.matrica_pikseli[piksel_x_desno][piksel_y_dole]
        return (self.matrica_pikseli[piksel_x][piksel_y] or prva or druga or treca or cetvrta)

    def colision_spoji(self,x,y):
        #self.omjer = 40.0/600.0
        piksel_x=int(self.sirina_pikseli*(y-self.pocetak_y)/(-self.sirina_m))
        piksel_y=int(self.duzina_pikseli*(x-self.pocetak_x)/self.duzina_m)
        return self.matrica_pikseli[piksel_x][piksel_y]

    def daj_dimenziju_x(self):
        return self.duzina_m
    def daj_dimenziju_y(self):
        return self.sirina_m

class Cvor (object):
    def __init__(self, data, parent=None):
        self.data=data
        self.parent=parent
        if parent != None:
            parent.djeca.append(self)
        self.djeca = []

    def __iter__(self):
        cvor=self
        while cvor != None:
            yield cvor
            cvor=cvor.parent


class Stablo(object):
    def __init__(self,Inicijalni_cvor):
        self.lista_cvorova=[]
        self.stablo_tacaka=[]
        self.lista_cvorova.append(Inicijalni_cvor)
        self.stablo_tacaka.append(Inicijalni_cvor.data)

    def dodaj (self,Cvor):
        self.lista_cvorova.append(Cvor)
        self.stablo_tacaka.append(Cvor.data)




x_ciljna=None
y_ciljna=None
x_pocetno=-15.0
y_pocetno=-10.0
wrapper=None
epsilon=0.4
done=0
stani=None

#Funkcija koja kupi podatke o mapi
def pokupi(pose):
    global wrapper
    wrapper=Wrapper(pose)

#Funkcija koja saznaje informaciju o pocetnoj konfiguraciji robota
def inicijaliziraj(pose):
    global x_pocetno
    global y_pocetno
    x_pocetno=pose.pose.pose.position.x
    y_pocetno=pose.pose.pose.position.y
    q=pose.pose.pose.orientation


#Funkcija koja saznaje informaciju o ciljnoj konfiguraciji i na taj nacin pocinje izvrsavanje koda za planiranje putanje
def ciljna(pose):
    global x_ciljna
    global y_ciljna
    x_ciljna=pose.point.x
    y_ciljna=pose.point.y
    loptica=Bool()
    loptica.data=True
    stani.publish(loptica)
    rrt()

#Funkcija koja niz tacaka koje predstavljaju putanju objavljuje na temu /ciljevi u obliku PoseArray i koja u RViz iscrtava putanju
def gotovo(nodes):
    putanjice=Marker()
    putanjice.type=4
    putanjice.header.frame_id='odom'
    #marker_linije.action=marker.ADD
    putanjice.scale.x=0.3
    putanjice.color.g=1.0
    putanjice.color.a=1.0

    putanjica=Marker()
    putanjica.type=8
    putanjica.header.frame_id='odom'
    #marker_tacke.action=marker.ADD
    putanjica.scale.x=0.3
    putanjica.scale.y=0.2
    putanjica.color.g = 1.0
    putanjica.color.a = 1.0

    pose_array=PoseArray()
    for i in xrange(0,len(nodes)):
        tocka=Point()
        tocka_pose=Pose()
        tocka_pose.position.x=nodes[i][0]
        tocka_pose.position.y=nodes[i][1]
        tocka.x=nodes[i][0]
        tocka.y=nodes[i][1]
        putanjica.points.append(tocka)
        putanjice.points.append(tocka)
        pose_array.poses.append(tocka_pose)
    konacni1.publish(putanjica)
    konacni2.publish(putanjice)

    #nodes11=nodes
    posalji_niz.publish(pose_array)
    rate.sleep()



#Funkcija za racunanje euklidske distance izmedju tacaka
def euklidska(tacka1,tacka2):
    return math.sqrt((tacka2[1] - tacka1[1]) ** 2 + (tacka2[0] - tacka1[0]) ** 2)

#set funkcija koje pokusavaju spojiti dvije proslijedjene tacke i ukoliko ne postoji prepreka
#vracaju zadnju tacku niza q_k koja je ujedno i q_r, a ukoliko postoji prepreka, vracaju posljednju tacku na slobodnom prostoru
def kombinezon(p, q, alpha):
    assert len(p) == len(q), 'Points must have the same dimension'
    return tuple([(1 - alpha) * p[i] + alpha * q[i] for i in xrange(len(p))])


def prosiri (p1,p2):
    global epsilon
    d=euklidska(p1,p2)

    if d<epsilon:
        return None

    alpha_i=epsilon/d
    n=int(d/epsilon)

    zapamti=None
    for i in xrange(1,n):
        alpha=alpha_i*i
        novo=kombinezon(p1,p2,alpha)
        if wrapper.colision(float(novo[0]),float(novo[1])):
            return zapamti
        else:
            zapamti=novo
    return zapamti

#Funkcija koja pokusava spojiti dvije tacke iz dva razlicita stabla koja vraca None ukoliko je bilo prepreka
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
            print 'prepreka'
            print (novo[0],novo[1])
            return None
        else:
            zapamti=novo
    return zapamti

##Prostor za implementaciju algoritma RRT
def rrt():
    global x_ciljna
    global y_ciljna
    global wrapper
    global epsilon
    global konacni1
    global konacni2
    global done
    global x_pocetno
    global y_pocetno

    broj_tacaka=1000
    najmanja=euklidska((float(x_pocetno),float(y_pocetno)),(float(x_ciljna),float(y_ciljna)))
    qi=Cvor((float(x_pocetno),float(y_pocetno)))
    Gi=Stablo(qi)
    qf=Cvor((float(x_ciljna),float(y_ciljna)))
    Gf=Stablo(qf)
    indexi=0
    indexf=0
    i=0
    while i<broj_tacaka:
       x=numpy.random.uniform()
       if x<0.8:
	   #exploracija Gi
           qr=(random.uniform(float(wrapper.pocetak_x),float(wrapper.duzina_m/2)),random.uniform(float(wrapper.pocetak_y),float(wrapper.sirina_m/2))))
           udaljenost,indeks=spatial.KDTree(Gi.stablo_tacaka).query(qr)
           QR=prosiri(Gi.stablo_tacaka[indeks].data,qr)
           QR_pravi=Cvor(QR,Gi.stablo_tacaka[indeks])
           Gi.dodaj(QR_pravi)
           udaljenost,indeks=spatial.KDTree(Gf.stablo_tacaka).query(QR)
           if udaljenost<najmanja:
		najmanja=udaljenost
		indexf=indeks
                indexi=len(Gi.stablo_tacaka)-1
	   #exploracija Gf
           qr=(random.uniform(float(wrapper.pocetak_x),float(wrapper.duzina_m/2)),random.uniform(float(wrapper.pocetak_y),float(wrapper.sirina_m/2)))
           udaljenost,indeks=spatial.KDTree(Gf.stablo_tacaka).query(qr)
           QR=prosiri(Gf.stablo_tacaka[indeks].data,qr)
           QR_pravi=Cvor(QR,Gf.stablo_tacaka[indeks])
           Gf.dodaj(QR_pravi)
           udaljenost,indeks=spatial.KDTree(Gi.stablo_tacaka).query(QR)
           if udaljenost<najmanja:
		najmanja=udaljenost
		indexi=indeks
                indexf=len(Gf.stablo_tacaka)-1 
           i=i+2
       else
	   #spajanje
           a=spoji(Gi.stablo_tacaka(indexi).data,Gf.stablo_tacaka(indexf).data)
	   if(a!=None):
		indeks_pamti1=indexi
		indeks_pamti2=indexf
		print stablo_pocetak.lista_cvorova[indeks_pamti1].data
		print stablo_kraj.lista_cvorova[indeks_pamti2].data
		nodes_pocetak_obrnuto=[node1.data for node1 in stablo_pocetak.lista_cvorova[indeks_pamti1]]
		nodes_pocetak=nodes_pocetak_obrnuto[::-1]
		nodes_kraj=[node.data for node in stablo_kraj.lista_cvorova[indeks_pamti2]]
		nodes=[]
		nodes.extend(nodes_pocetak)
		nodes.extend(nodes_kraj)
		gotovo(nodes)
		print 'cvorovi'
    	        print nodes
		break		 
       





    #dodavanje pocetnog i krajnjeg cvora

    #indeks_pamti1 predstavlja indeks najblizeg cvora iz G_i, dok indeks_pamti2 predstavlja indeks najblizeg cvora iz G_f kad se oni mogu spojiti
    






       rate.sleep()

if __name__ == '__main__':
    #rospy.Subscriber("/base_pose_ground_truth", Odometry, odometrija)
    rospy.Subscriber("/ciljna_konfiguracija", PointStamped, ciljna)
    rospy.Subscriber("/map", OccupancyGrid, pokupi)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, inicijaliziraj)
    #rospy.Subscriber("/linije2",Marker,gotovo)
    konacni1=rospy.Publisher('/putanjice', Marker, queue_size=10)
    konacni2=rospy.Publisher('/putanjica', Marker, queue_size=10)
    posalji_niz=rospy.Publisher('/ciljevi',PoseArray,queue_size=10)
    stani=rospy.Publisher('/reci_mu_da_stane',Bool,queue_size=10)

    rospy.init_node('ciljne_tacke_node', anonymous=True)
    rate=rospy.Rate(10)

    while wrapper is None or x_ciljna is None:
        rate.sleep()

    while not rospy.is_shutdown():
        rospy.spin()



    #try:
    #    talker()
    #except rospy.ROSInterruptException:
    #    pass
