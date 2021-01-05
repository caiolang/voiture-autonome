from math import *
from intersectionarc import *
from elimineptsinaccessibles import *
import matplotlib.pyplot as plt
from elimineptsinaccessibles import *
from cibleatteignable import *
from cibleatteignable2 import *
from cibleatteignablevcste import *
from cibleatteignableevitement import *



def trouveciblearc (MMMRC,v):
    "MMMRC=[(thetai,ri,xi,yi,xpi ou xpprimi,ypi ou ypprimi)]"
    "MMMRCT=[(thetai,ri,xi,yi,xpi ou xpprimi,ypi ou ypprimi,rpi)] trié par rpi croissant"
    "segments=[(xpprimi,ypprimi,xpprimip1,ypprimip1)]"
    [positioninit1,positioninit2,orientationinit1,orientationinit2,vinit1,vinit2,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
    R=0
    MMMRCT=[]
    NNN=len(MMMRC)
    MMMRCE=MMMRC #elimineptsinaccessibles(MMMRC,v)
    NNNE=len(MMMRCE)
    i=0 
    while i<NNN:
        d=MMMRCE[i][4]**2+MMMRCE[i][5]**2
        liste=MMMRCE[i]+[d]
        MMMRCT.append(liste)
        i+=1
        
    MMMRCT.sort(key=lambda x: x[6])
    MMMRCT.reverse()
    segments=[]
    i=0
    while i<(NNN-1):
        segments.append([MMMRC[i][4],MMMRC[i][5],MMMRC[i+1][4],MMMRC[i+1][5]])
        
        #plt.plot([MMMRC[i][4]+4,MMMRC[i+1][4]+4],[MMMRC[i][5]-1,MMMRC[i+1][5]-1],"y-+") #debugage
        i=i+1
        
    i=0
    Rimpact=1 #valeur random mais pas 0 pour éviter pb de division par 0 
    while i<NNNE:
        evitement=False
        xpi=MMMRCT[i][4]
        ypi=MMMRCT[i][5]
        
        sgyp=1
        if ypi<0:
            sgyp=-1
            
        j=0
        
        while j<len(segments) and intersection((xpi,ypi),segments[j])!=1:
            j+=1
        
        if j==len(segments): #alors le pt pi trouvé une cible qui ne coupe aucun segment"
                             #il faut à présent vérifier si on à un arc qui permet de rejoindre la droite liant le lidar à la cible précedemment trouvée

            if ypi==0:
                R=10000
                evitement=False
                Rimpact=xpi
                return([xpi,ypi],R,sgyp,evitement,Rimpact)
                
            ind,R,sgyp=cibleatteignablevcste(segments,[xpi,ypi],v) 
            #ind me dit si y à un rayon acceptable à la vitesse v qui me permet de rejoindre le pt visé à la vitesse actuelle et si oui quel est le rayon le plus grand permettant de rejoindre la traj qui mene au pt visé
            
            if  ind==1:
                Rimpact=MMMRC[int(90/360*N)][2] #distance au mur dans la direction droit devant la voiture (theta=0)
                return([xpi,ypi],R,sgyp,evitement,Rimpact)
                
            
            #si ind!=1 
            #on vérifie si la trajectoire extème freiner et braquer à fond permet de rejoindre la traj en lg droite sans se prendre un mur 
            evitement=True
            indev,R,sgyp=cibleatteignableevitment(segments,[xpi,ypi],v,sgyp)
            
            if indev==1: #la trajectoire extème freiner et braquer à fond permet de rejoindre la traj en lg droite sans se prendre un mur 
                R=10000
                Rimpact=MMMRC[int(90/360*N)][2]
                return([xpi,ypi],R,sgyp,evitement,Rimpact)
            
        i+=1
        
    print('ERREUR pas de cible optimale')
    sgyp=1
    if MMMRCT[0][5]<0:
        sgyp=-1
    return([MMMRCT[0][4],MMMRCT[0][5]],10000,sgyp,True,MMMRC[int(90/360*N)][2]) #gain en robustesse de toute facon c'est ce qu'il y à de mieux à faire! 
    #10000 = rayon tres gd #on va tt droit
    