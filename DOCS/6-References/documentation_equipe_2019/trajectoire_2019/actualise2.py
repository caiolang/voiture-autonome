from math import *


def actualise2(R,v,position,obj,cible,orientation,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,vmax,N):
    
    p=(tsb*vmax+l)/deltat*0.09 
    d=p*1.8
    
    theta0=atan(cible[1]/cible[0])
    theta1=atan(cible[1]/(cible[0]-v*deltat))
    thetapoint=(theta1-theta0)/deltat
    epsilon=(p*theta0+d*thetapoint)
    #print(epsilon*180/pi)
    epsilon=min(epsilonmax*2*pi/360,epsilon)
    if epsilon<0:
        epsilon=max(-epsilonmax*2*pi/360,epsilon)
        
    Rprim=abs(tsb*v**2/(epsilon)+l/(epsilon))
    
   
    #calcul vrai rayon courbure
    
   
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
       
        if cible[1]<0:
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
        xprim1=rprim*cos(thetaparc+orientation*2*pi/360) #correction en angle? 
        yprim1=rprim*sin(thetaparc+orientation*2*pi/360)
        
    
    #calcul coordonnées du pt atteint apres deltat dans ref abs
    xabs=xprim1+position[0]
    yabs=yprim1+position[1]
    
    positionprim=[xabs,yabs]
    
    #calc nouvelle orientation
    alphainc=360/N
    orientationprim=orientation+thetaparc*360/(2*pi)
    #if obj[1]<position[1]:
    #    orientationprim=orientation-thetaparc
    
    #calc nouvelle vitesse
    
    vmaxr=sqrt(amaxlat*abs(R))
    vmaxant=sqrt(-2/3*amin*(sqrt(cible[0]**2+cible[1]**2)))

    vmaxx=min(vmaxr,vmaxant)
    a=min(1,(vmaxx-v)/(deltat*amax))*amax
    if v>vmaxr:
        a=min(1,(vmaxx-v)/(deltat*amin))*amin
    
    vprim=min(vmax,v+a*deltat)
    
    return(positionprim,orientationprim,vprim)