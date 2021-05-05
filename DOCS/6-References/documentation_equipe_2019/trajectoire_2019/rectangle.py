from params import *


def rectangle(position,orientation):
    #coord des coins du rectangle en coord cartésiènnes dans le repère abs x,y position du lidar
    
    [positioninit1,positioninit2,orientationinit1,orientationinit2,vinit1,vinit2,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
    x=position[0]
    y=position[1]
    alphaa=pi+atan(larg/l)
    alphab=pi-atan(larg/l)
    alphac=atan(larg/l)
    alphad=-atan(larg/l)
    
    xa=x+rv*(cos(orientation*2*pi/360+alphaa))
    ya=y+rv*(sin(orientation*2*pi/360+alphaa))
    
    xb=x+rv*(cos(orientation*2*pi/360+alphab))
    yb=y+rv*(sin(orientation*2*pi/360+alphab))
    
    xc=x+rv*(cos(orientation*2*pi/360+alphac))
    yc=y+rv*(sin(orientation*2*pi/360+alphac))
    
    xd=x+rv*(cos(orientation*2*pi/360+alphad))
    yd=y+rv*(sin(orientation*2*pi/360+alphad))
    
    a=[xa,ya]
    b=[xb,yb]
    c=[xc,yc]
    d=[xd,yd]
    
    return([a,b,c,d,a])
    
    