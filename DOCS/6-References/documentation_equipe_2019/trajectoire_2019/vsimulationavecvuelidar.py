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
from params import *
from calculrayoncourbure import *
from math import *
from objectif import *

#initialisation

envir=environment()
[positioninit,orientationinit,vinit,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
alphainc=360/N
position=[4,-1]
orientation=0
v=vinit
positionm1=[11.3,-8]
orientationm1=0
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

P=1 #nombre de pts de la traj à tracer
j=0
while j<P:
    
    #Tracé de ce que voit le lidar
    
    x=position[0]
    y=position[1]
    i=0
    while i<len(MMMR):
        xl=MMMR[i][2] #abscisse du pt lidar dans le ref du lidar 
        yl=MMMR[i][3]
        theta=MMMR[i][0] #nb inc angle de coord polaires dans le ref du lidar
        thetac=(theta*alphainc+orientation)*2*pi/360  #angle de coord polaires dans le ref absolu
        r=MMMR[i][1]
        xlc=r*cos(thetac)+x #rcos(thetac) =abscisse dans le ref du lidar rotationné de orientation xlc=abscisse dans le ref absolu
        ylc=r*sin(thetac)+y

        xlp=MMMR[i][4] #abscisse du pt lidar dans le ref du lidar 
        ylp=MMMR[i][5]
        thetap=atan((ylp)/(xlp))
        if xlp<0:
            thetap=pi+atan(ylp/xlp)
        thetacp=thetap+orientation*2*pi/360
        rp=sqrt(xlp**2+ylp**2)
        xlpc=rp*cos(thetacp)+x
        ylpc=rp*sin(thetacp)+y


        plt.plot(xlc, ylc,"b:o")
        plt.plot(xlpc,ylpc,"r:o")
        i+=1
    
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
     
     
     
    plt.plot(position[0],position[1],"k:o")
    plt.plot([position[0],xciblec],[position[1],yciblec],"k:+")
    plt.plot(xciblec,yciblec,"g:o")
    j+=1
    
plt.axis('equal')
plt.show()
    
#il y a des pb de sortie de piste, je soupçonne l'algo de calcul de rayon de courbure (pb de ref lidar) ou l'algo d'actualisation des données... plutot l'actualisation! 
#visiblement l'ajustement en fct de v bugue a cause du mauvais callage? a cause de actu? a cause du prgm lui mm? param alpha réglé à 0 pour éliminer le pb pour le moment 
#je regarde vers le bas mais je monte... y a un pb dans actu
#actu est codée avec le cul pour actualiser la position, va falloir trouver mieux 
    