from math import *

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