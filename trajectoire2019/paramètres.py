#paramètres:
from math import *

def params():
    positioninit=[4,-1]
    orientationinit=0
    vinit=0
    deltat=1/24
    amaxlat=20
    epsilonmax=45
    amax=2 #amax est fct de v il faudra ajuster en fct des courbes d'accel et de freinage
    amin=-2 #amin est fct de v il faudra ajuster en fct des courbes d'accel et de freinage
    tsb=0.1
    l=0.4
    larg=0.2
    vmax=30/3.6
    N=720
    rv=sqrt(l**2+larg**2)/2
    m=0.05
    alpha=0 #optimiser ajustevitesse serait pas mal
    lanti=10 #longueur à laquelle je place un pt qui s'éloigne de moi en vitesse relative 
    param=[positioninit,orientationinit,vinit,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]
    return(param)