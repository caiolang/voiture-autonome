from params import *
from intersectionarc import *
from calculrayoncourbure import *
from intersection import *
from math import *

def calculetrajevitementtronquee(v,p,sgyp):
    
    #calcule la trajectoire approchée par ligne brisée de la voiture 
    #revoie 0 si cette traj ne coupe pas la droite OP et trajectoire sous fourme de liste de segments
    #renvoie 1 si cette traj coupe la droite OP et trajectoire tronquée (on garde que les segments avant celui qui coupe OP) sous fourme de liste de segments
    
    
    [positioninit1,positioninit2,orientationinit1,orientationinit2,vinit1,vinit2,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
    Rminamaxlat=v**2/amaxlat
    Rminepsilonmax=tsb*v**2/(epsilonmax*pi/180)+l/(epsilonmax*pi/180)
    Rmin=max(Rminamaxlat,Rminepsilonmax)
    Rmax=abs(calculrayoncourbure(p))/4
    xp=p[0]
    yp=p[1]
    
    T=3 #horizont de prévision en secondes
    nb=10 #nb de pts que comporte la trajectoire (plus on en a plus c'est précis) 
    deltaT=T/nb #incréments des temps entre deux pts de la trajectoire
    
    ptstrajectoire=[] #contient les pts de la traj aux instants i*deltaT
    
    #initialiation
    
    vprim=max(v + amin*deltaT,1) #on veut pas de cas ou la vitesse devient trop basse ou mm négative on la règle à 1m/s au min de manière arbitraire 
    Rminamaxlatprim=vprim**2/amaxlat
    Rminepsilonmaxprim=tsb*vprim**2/(epsilonmax*pi/180)+l/(epsilonmax*pi/180)
    Rminprim=max(Rminamaxlatprim,Rminepsilonmaxprim)
    theta=0
    thetaparc=(vprim/Rminprim)*deltaT*sgyp
    thetaprim=theta+thetaparc
    yprimreft=sgyp*Rminprim*(1-cos(thetaparc)) #ordonnée du pt de traj à t+dt dans le ref de la voiture à t 
    xprimreft=(Rminprim-yprimreft)*sin(thetaparc) #abscisse du pt de traj à t+dt dans le ref de la voiture à t 
    rprim=sqrt(xprimreft**2+yprimreft**2)
    x=0
    y=0
    xprim=x+rprim*cos(thetaprim)
    yprim=x+rprim*sin(thetaprim)
    
    ptstrajectoire.append([x,y])
    ptstrajectoire.append([xprim,yprim])
    
   #hérédité
   
    i=1
    
    while i<nb:
        x,y=xprim,yprim
        v=vprim
        theta=thetaprim
        
        vprim=max(v + amin*deltaT,1)
        Rminamaxlatprim=vprim**2/amaxlat
        Rminepsilonmaxprim=tsb*vprim**2/(epsilonmax*pi/180)+l/(epsilonmax*pi/180)
        Rminprim=max(Rminamaxlatprim,Rminepsilonmaxprim)
        thetaparc=(vprim/Rminprim)*deltaT*sgyp
        thetaprim=theta+thetaparc
        yprimreft=sgyp*Rminprim*(1-cos(thetaparc)) #ordonnée du pt de traj à t+dt dans le ref de la voiture à t 
        xprimreft=(Rminprim-yprimreft)*sin(thetaparc) #abscisse du pt de traj à t+dt dans le ref de la voiture à t 
        rprim=sqrt(xprimreft**2+yprimreft**2)
        xprim=x+rprim*cos(thetaprim)
        yprim=y+rprim*sin(thetaprim)
        ptstrajectoire.append([xprim,yprim])
        
        #print('vprim',vprim)
        
        
        
        i+=1
    
    #on à maintenant les pts constitutifs de la trajectoire, on approxime la trajectoire à la ligne brisée obtenue en liant les points sur la trajectoire par des segments
    #on va chercher à savoir quand cette trajectoire nous ramène sur la ligne droite qui relie le pt ou l'on se trouve actuellement (0,0) au point cible p
    
    trajectoire=[]
    i=0
    while i<(len(ptstrajectoire)-1):
        trajectoire.append([ptstrajectoire[i][0],ptstrajectoire[i][1],ptstrajectoire[i+1][0],ptstrajectoire[i+1][1]])
        i=i+1
    
  
    xp=p[0]
    yp=p[1]
    j=0
    while j<len(trajectoire) and intersection(p,trajectoire[j])!=1:
        j+=1
    if j==len(trajectoire): #alors aucun segment de la traj ne coupe OP 
        return(0,trajectoire)
    
    return(1,trajectoire[:(j+2)])
    
    
        
    
    
   