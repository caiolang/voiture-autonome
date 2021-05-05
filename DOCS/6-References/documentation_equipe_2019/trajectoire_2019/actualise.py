from math import *


def actualise(R,v,position,obj,orientation,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,vmax,N):
    #calcul vrai rayon courbure
    R=abs(R)
    Rprim=max(R,tsb*v**2/(epsilonmax*2*pi/360)+l/(epsilonmax*2*pi/360),sqrt(v**2/amaxlat))
    xobj=obj[0]
    yobj=obj[1]  
   
    #calcul coordonnées du pt atteint apres deltat dans ref lidar
    xprim=0
    yprim=0
    xprim1=0
    yprim1=0
    thetaparc=0
    if v!=0:
        thetaparc=atan(xobj/(Rprim-abs(yobj)))
        yprim=Rprim*(1-cos(thetaparc))
        xprim=(Rprim-yprim)*sin(thetaparc)
        
        if yobj<0:
            yprim=-yprim
           
        #débugage
        #thetaparc=atan(yobj/xobj)
        #xprim=xobj
        #yprim=yobj  
        print('xobj',xobj)
        print('yobj',yobj)
        print('xprim',xprim)
        print('yprim',yprim)
        
        
        
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
    
    vmaxr=sqrt(amaxlat*Rprim)
    a=min(1,(vmaxr-v)/(deltat*amax))*amax
    if v>vmaxr:
        a=min(1,(vmaxr-v)/(deltat*amin))*amin
    
    vprim=min(vmax,v+a*deltat)
    
    return(positionprim,orientationprim,vprim)
    