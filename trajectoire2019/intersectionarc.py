from math import *
from calculrayoncourbure import calculrayoncourbure

def intersectionarc(inter,segment):
    "retourne 1 et les coordonnées du pt d'intersection si la droite de direction dir passant par p coupe le segment segment"
    #changement de repère
    
    r=abs(calculrayoncourbure(inter))
    xint=inter[0]
    yint=inter[1]
    
    
    x1=segment[0]-xint
    y1=segment[1]-yint
    x2=segment[2]-xint
    y2=segment[3]-yint
    
    xc=0
    
    yc=r
    if yint<=0:
        yc=-r
    

    res=0
  
    thetamax=asin(abs(xint/r))
    
    
    if abs(yint)>r:
        thetamax=pi-thetamax
    
    if abs(x2-x1)>0.0001:
        K=(y2-y1)/(x2-x1)
        C=y1-K*x1
        b=(2*K*(C-yc))/(1+K**2)
        c=((C-yc)**2-r**2)/(1+K**2)
        delta=b**2-4*c
        if delta<0:
            return(0)
        if delta==0:
            xintseg=-b/2 #intersection entre le cercle def par le pt inter et le segment segment 
            yintseg=K*xintseg+C
            #print('xintseg',xintseg)
            #print('yintseg',yintseg)
            #print('segment',segment)
            theta=asin(abs(xintseg/r))
            if abs(yintseg)>r:
                theta=pi-theta
            if xintseg>0 and theta<thetamax and (x1-xinter)*(x2-xinter)+(y1-yinter)*(y2-yinter)<=0:
                return(1)
            return(0)
            
        #si delta>0
        
        xinter=-b/2+sqrt(delta)/2
        xinter2=-b/2-sqrt(delta)/2
        yinter=K*xinter+C
        yinter2=K*xinter2+C
        theta=asin(abs(xinter/r))
        theta2=asin(abs(xinter2/r))
        if abs(yinter)>r:
            theta=pi-theta
            
        if abs(yinter2)>r:
            theta2=pi-theta2
        
        if xinter>0 and theta<thetamax and (x1-xinter)*(x2-xinter)+(y1-yinter)*(y2-yinter)<=0:
            return(1)
        
        if xinter2>0 and theta2<thetamax and (x1-xinter2)*(x2-xinter2)+(y1-yinter2)*(y2-yinter2)<=0:
            return(1)
            
        return(0)
    
 

    b=-2*yc
    c=yc**2-r**2+(x1)**2
    delta=b**2-4*c
    
    if delta<0:
        return(0)
    if delta==0:
        xinter=x1
        yinter=-b/2
        theta=asin(abs(xinter/r))
        if abs(yinter)>r:
            theta=pi-theta
            
        if xinter>0 and theta<thetamax and (x1-xinter)*(x2-xinter)+(y1-yinter)*(y2-yinter)<=0:
            return(1)
        return(0)
            
        #si delta>0
        
    xinter=x1
    xinter2=x1
    yinter=-b/2+sqrt(delta)/2
    yinter2=-b/2-sqrt(delta)/2
    theta=asin(abs(xinter/r))
    theta2=asin(abs(xinter2/r))
    
    if abs(yinter)>r:
            theta=pi-theta
            
    if abs(yinter2)>r:
            theta2=pi-theta2
        
    if xinter>0 and theta<thetamax and (x1-xinter)*(x2-xinter)+(y1-yinter)*(y2-yinter)<=0:
        return(1)
        
    if xinter2>0 and theta2<thetamax and (x1-xinter2)*(x2-xinter2)+(y1-yinter2)*(y2-yinter2)<=0:
        return(1)
    return(0)
   

   