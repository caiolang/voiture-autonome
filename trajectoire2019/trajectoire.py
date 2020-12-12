import matplotlib.pyplot as plt
import numpy as np
import copy
from zonesafe import *
from adaptevitesserelat import *
from trouvecible import *
from environment import *
from lidar import *
from actualise import *
from actualise2 import *
from paramètres import *
from calculrayoncourbure import *
from math import *
from objectif import *




def trajectoire(P):
    #simulation de trajectoire:
    
    
    trajectoirex=[]
    trajectoirey=[]
    trajectoireor=[]
    
    #initialisation
    
    envir=environment()
    [positioninit,orientationinit,vinit,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
    alphainc=360/N
    position=positioninit
    orientation=orientationinit
    
    v=vinit
    positionm1=positioninit
    orientationm1=orientationinit
    vm1=vinit
    
    Mm1=lidar(envir,positionm1,orientationm1,N)
    M=lidar(envir,position,orientation,N)
    
    MMMR=zonesafe(M,rv,m)
    
    MMMRC=adaptevitesserelat (Mm1,M,MMMR,alpha,v,deltat,lanti,orientation,orientationm1)
    cible=trouvecible(MMMRC)
    obj=objectif(cible,v,deltat)
    R=calculrayoncourbure(obj)
    
    
    #nombre de pts de la traj à tracer
    j=0
    
    while j<P:
        
        
        trajectoirex.append(position[0])
        trajectoirey.append(position[1])
        trajectoireor.append(orientation)
        
        #mise à jour des variables 
        positionm1=copy.deepcopy(position)
        orientationm1=copy.deepcopy(orientation)
        vm1=copy.deepcopy(v)
        
        [position,orientation,v]=actualise2(R,v,position,obj,cible,orientation,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,vmax,N)
        Mm1=lidar(envir,positionm1,orientationm1,N)
        M=lidar(envir,position,orientation,N)
        MMMR=zonesafe(M,rv,m)
        MMMRC=adaptevitesserelat (Mm1,M,MMMR,alpha,v,deltat,lanti,orientation,orientationm1)
        cible=trouvecible(MMMRC)
        obj=objectif(cible,v,deltat)
        R=calculrayoncourbure(obj)
        
        #préparation de cible pour affichage 
        xcible=cible[0]  
        ycible=cible[1]
        thetacible=atan((ycible)/(xcible))
        if xcible<0:
            thetacible=pi+atan(ycible/xcible) 
        thetaciblec=thetacible+orientation*2*pi/360
        rcible=sqrt(xcible**2+ycible**2)
        xciblec=rcible*cos(thetaciblec)+position[0]
        yciblec=rcible*sin(thetaciblec)+position[1]
    
        
        
        
        #debugage
    
        #print(j)
        #print('vitesse',v)
        #print('orientation', orientation)
        #print('R',R)
        if j%50==0:
            print(j)
        
        
        
        
        j+=1
        
    return([trajectoirex,trajectoirey,trajectoireor])
