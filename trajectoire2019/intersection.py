from math import *

def intersection(p,segment):
    "retourne 1 si la droite OP coupe le segment segment"
    xp=p[0]
    yp=p[1]
    x1=segment[0]
    y1=segment[1]
    x2=segment[2]
    y2=segment[3]
    
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
    
    
