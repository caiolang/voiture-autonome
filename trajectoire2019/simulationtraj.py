#simulation de trajectoire:

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


#afficher le circuit

envir=environment()
n=len(envir)
i=0
while i<n:
    nn=len(envir[i])
    j=0
    while j<(nn-1):
        plt.plot([envir[i][j][0],envir[i][j+1][0]],[envir[i][j][1],envir[i][j+1][1]],"c:o")
        j+=1
    i+=1
    
i=0

plt.plot(position[0],position[1],"k:o")

P=375  #nombre de pts de la traj à tracer
j=0
while j<P:
    
    
    trajectoirex.append(position[0]-cos(orientation*2*pi/360))
    trajectoirey.append(position[1]-sin(orientation*2*pi/360))
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
   
    print(j)
    #print('vitesse',v)
    #print('orientation', orientation)
    #print('R',R)
    
     
     
    
    plt.plot(position[0],position[1],"k:o")
    plt.plot(xciblec,yciblec,"g:o")
    j+=1
    
plt.axis('equal')
plt.show()
    

    
    
    
    