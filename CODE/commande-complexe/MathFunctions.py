'''
This file contains all mathematical and trigonometric functions to check distances, intersections, calculate radii of curvature, etc;
'''




from math import *

def Intersection(point,angle,segment):
    
    '''
       Checks if an intersection between a segment and a vector occurs from a point with a max size
    
    point 
    angle
    segment 
    maxdistance
      
      Return:
      
         [1, [xinter,yinter]] if an intersection occurs
         [0, [xinter,yinter]] if an intersection dont occur
    
    '''
    
       
    
    xp=point[0]
    yp=point[1]
    
    x1=segment[0]-xp
    y1=segment[1]-yp
    x2=segment[2]-xp
    y2=segment[3]-yp

    
    Kp=tan(angle*2*pi/360)
    res=0
    
    if x2!=x1:
        K=(y2-y1)/(x2-x1)
        C=y1-K*x1
        if Kp!=K:
            xinter=(C)/(Kp-K)
            yinter=Kp*(C)/(Kp-K)
            if (x1-xinter)*(x2-xinter)+(y1-yinter)*(y2-yinter)<=0:
                res=1
        if Kp==K:
            res=0
            [xinter,yinter]=[0,0]
    if x2==x1:
            xinter=x1
            yinter=Kp*xinter
            if (x1-xinter)*(x2-xinter)+(y1-yinter)*(y2-yinter)<=0:
                res=1

    return([res,[xinter,yinter]])


def SimpleIntersection(p,segment):
    "retourne 1 si la droite OP coupe le segment segment"
    xp=p[0]
    yp=p[1]
    x1=segment[0]
    y1=segment[1]
    x2=segment[2]
    y2=segment[3]
    
    if xp==0:
        print('yp',yp)
        xp=0.001
    Kp=yp/xp
    res=0
    if x2!=x1:
        K=(y2-y1)/(x2-x1)
        C=y1-K*x1
        if Kp!=K:
            xinter=(C)/(Kp-K)
            yinter=Kp*(C)/(Kp-K)
            if (x1-xinter)*(x2-xinter)+(y1-yinter)*(y2-yinter)<0:
                res=1
        if Kp==K:
            res=0
            [xinter,yinter]=[0,0]
    if x2==x1:
            xinter=x1
            yinter=Kp*xinter
            if (x1-xinter)*(x2-xinter)+(y1-yinter)*(y2-yinter)<0:
                res=1
    
    
        
    return(res)
    

    
def CurvatureRadius(target):
    """
        The function receives the target position and calculates and returns the radius of curvature R
    """
    
    xp=target[0]
    yp=target[1]
    if yp!=0:
        K=-xp/yp #la pente de la droite ortho à O cible"
        C=yp/2-K*xp/2    
        yo=K*0+C 
    if yp==0:
        yo=10000
    R=yo
    return(R)


def IntersectionArc(inter,segment):
    "retourne 1 et les coordonnées du pt d'intersection si la droite de direction dir passant par p coupe le segment segment"
    #changement de repère
    
    r=abs(CurvatureRadius(inter))
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



   
def intersectionsegments(segment1,segment2):
    "retourne 1 si segment 1 coupe le  segment2, 0 sinon"
  
    x1=segment1[0]
    y1=segment1[1]
    x2=segment1[2]
    y2=segment1[3]
    
    x3=segment2[0]
    y3=segment2[1]
    x4=segment2[2]
    y4=segment2[3]
    
    #calculer pt d'intersection des droites def par les 4 pts 
    #vérifier que ce pt d'intersec appartiennent bien aux 2 segments  
    
    res=0
    if x2!=x1:
        K1=(y2-y1)/(x2-x1)
        C1=y1-K1*x1
        
        if x3!=x4:
            K2=(y3-y4)/(x3-x4)
            C2=y2-K2*x3
            if K1==K2:
                if C1!=C2:
                    return(0)
                if (x1-x3)*(x2-x3)+(y1-y3)*(y2-y3)<0 or (x1-x4)*(x2-x4)+(y1-y4)*(y2-y4)<0 or (x3-x1)*(x4-x1)+(y3-y1)*(y4-y1)<0:
                    return(1)
                    
        return(0)
                
        
        xinter=x3
        yinter=K1*xinter+C1
        if (x1-xinter)*(x2-xinter)+(y1-yinter)*(y2-yinter)<0 and (x3-xinter)*(x4-xinter)+(y3-yinter)*(y4-yinter)<0:
            return(1)
        return(0)
            
            
            
    if x3!=x4:
        K2=(y3-y4)/(x3-x4)
        C2=y2-K2*x3
        
        xinter=x1
        yinter=K2*xinter+C2
        if (x1-xinter)*(x2-xinter)+(y1-yinter)*(y2-yinter)<0 and (x3-xinter)*(x4-xinter)+(y3-yinter)*(y4-yinter)<0:
            return(1)
        return(0)  
          
    if x1==x2 and ((y1-y3)*(y2-y3)<0 or (y1-y4)*(y2-y4)<0 or (y3-y1)*(y4-y1)<0) :
        return(1)
        
    return(0)

   




