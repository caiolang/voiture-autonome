from math import *
from intersection import *
from params import *
import copy

import matplotlib.pyplot as plt

def elimineptsinaccessibles(MMMRC,v):
    [positioninit1,positioninit2,orientationinit1,orientationinit2,vinit1,vinit2,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
    Rminamaxlat=v**2/amaxlat
    Rminepsilonmax=tsb*v**2/epsilonmax+l/epsilonmax
    Rmin=max(Rminamaxlat,Rminepsilonmax)
    MMMRCE=[]
    xc1=0
    xc2=0
    yc1=Rmin
    yc2=-Rmin
    
    i=0
    N=len(MMMRC)
    liste=[]
    while i<N:
        if ((MMMRC[i][4]-xc1)**2+(MMMRC[i][5]-yc1)**2)>(Rmin**2) or ((MMMRC[i][4]-xc2)**2+(MMMRC[i][5]-yc2)**2)>(Rmin**2):
            MMMRCE.append(MMMRC[i])
        i+=1
    
    return(MMMRCE)