from math import *
from params import *

def actualise2arc(R,v,position,cible,orientation,sgyp,Rimpact,evitement):
    
    [positioninit1,positioninit2,orientationinit1,orientationinit2,vinit1,vinit2,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
   
    epsilon=(tsb*v**2/R+l/R)
    
    if evitement:
        epsilon=epsilonmax*pi/180
        
    #if evitement:
        #epsilon=epsilonmax*pi/180
        #print('evitement')
        
    #print('epsilon',epsilon*180/pi)
    
    
    if sgyp<0:
        epsilon=-epsilon
    
    Rprim=abs(tsb*v**2/(epsilon)+l/(epsilon)) #calcul vrai rayon courbure
    
   
    #calcul coordonnées du pt atteint apres deltat dans ref lidar
    xprim=0
    yprim=0
    xprim1=0
    yprim1=0
    thetaparc=0
    if v!=0:
        thetaparc=v*deltat/Rprim
        yprim=Rprim*(1-cos(thetaparc))
        xprim=(Rprim-yprim)*sin(thetaparc)
        if v<0:
            xprim=-(Rprim-yprim)*sin(thetaparc)
            
       
        if sgyp<0:
            yprim=-yprim            
            thetaparc=-thetaparc
            
        
        #débugage
        #thetaparc=atan(yobj/xobj)
        #xprim=xobj
        #yprim=yobj  
        #print(cible[1])
        #print('thetaparc',thetaparc)
        #print('epsilon',epsilon)
        #print('xobj',obj[0])
        #print('yobj',obj[1])
        #print('xprim',xprim)
        #print('yprim',yprim)
        
        
        
        rprim=sqrt(xprim**2+yprim**2)
        
        alphainc=360/N
        xprim1=rprim*cos(thetaparc+orientation*2*pi/360) #correction en angle
        yprim1=rprim*sin(thetaparc+orientation*2*pi/360)
        if v<0:
            xprim1=rprim*cos(pi-thetaparc+orientation*2*pi/360)
            yprim1=rprim*sin(pi-thetaparc+orientation*2*pi/360)

        
    
    #calcul coordonnées du pt atteint apres deltat dans ref abs
    xabs=xprim1+position[0]
    yabs=yprim1+position[1]
    
    #calc nouvelle
    positionprim=[xabs,yabs]
    
    #calc nouvelle orientation
    alphainc=360/N
    orientationprim=orientation+thetaparc*360/(2*pi)
    #if obj[1]<position[1]:
    #    orientationprim=orientation-thetaparc
    
    #vitesse
    
    vmaxr=sqrt(amaxlat*R) #vitesse maxi à laquelle on peut rester sur un cercle de rayon R
    vmaxant=1.1*sqrt(-2/3*amin*Rimpact) #vitesse maxi pour pouvoir s'arêter avant le mur qui se trouve droit devant la voiture, c'est par principe de prudence
    #print('vmaxant',vmaxant)
        
    vmaxx=min(vmaxr,vmax,vmaxant)
    
    a=min(1,(vmaxx-v)/(deltat*amax))*amax
    
    if v>vmaxx:
        a=min(1,(vmaxx-v)/(deltat*amin))*amin
    
    if evitement:
        a=amin
   
    vprim=min(vmax,v+a*deltat)
    #print('vprim',vprim)
   
    return(positionprim,orientationprim,vprim)