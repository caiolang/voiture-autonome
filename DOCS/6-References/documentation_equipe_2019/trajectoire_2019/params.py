#paramètres:
from math import *

def params():
    positioninit1=[10,-9]
    positioninit2=[10,-8.5]
    orientationinit1=180
    orientationinit2=180
    vinit1=0
    vinit2=0
    deltat=1/10
    amaxlat=10 #=1/tsb je pense 
    epsilonmax=45
    amax=5 #amax est fct de v il faudra ajuster en fct des courbes d'accel et de freinage
    amin=-5 #amin est fct de v il faudra ajuster en fct des courbes d'accel et de freinage
    tsb=0.1
    l=0.4
    larg=0.2
    vmax=30/3.6
    N=360
    rv=sqrt((l/2)**2+(larg/2)**2)
    m=0.05
    alpha=25 #optimiser ajustevitesse serait pas mal
    lanti=10 #longueur à laquelle je place un pt qui s'éloigne de moi en vitesse relative 
    param=[positioninit1,positioninit2,orientationinit1,orientationinit2,vinit1,vinit2,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]
    return(param)