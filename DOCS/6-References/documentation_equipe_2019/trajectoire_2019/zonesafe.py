from math import *
import copy

def zonesafe (M,rv,m):
    
    "M=[(thetai,ri)] liste des tuples de coordonnées lidar attention theta i n'est pas un angle mais plutot un nb d'invcréments d'angle depuis la position de ref"
    "rv=rayon du cercle circosncrit au véhicule"
    "m=marge en plus du rayon du cercle circosncrit au véhicule"
    "N=nb de pts que fait le lidar"
    "MR=[(thetai,ri)] est la matrice lidar des points qui se trouvent dans les 180 degres qui correspondent à l'avant du véhicule"
    "MMR=[(thetai,ri,xi,yi)]"
    "MMMR=[(thetai,ri,xi,yi,xpi,ypi)] ou pi est tq le vecteur pipti est orthogonal au bord du circuit et de norme rv+m"
    
    N=len(M)
    A=copy.deepcopy(M[0:int(N/4)])
    B=copy.deepcopy(M[int(3*N/4):N])
    A.reverse()
    B.reverse()
    MR=copy.deepcopy(A+B) #deepcopy
    MMR=[]
    i=0
    alphainc=360/N #valeur d'un incrément d'angle
    while i<len(MR):
        theta=MR[i][0]
        r=MR[i][1]
        xi=r*cos(theta*alphainc*2*pi/360)
        yi=r*sin(theta*alphainc*2*pi/360)
        MMR.append([theta,r,xi,yi])
        i=i+1
        
    
    MMMR=[] 
    i=1
    while i<(len(MMR)-1):
        xim1=MMR[i-1][2] #abscisse du pt (i moins 1)
        yim1=MMR[i-1][3]
        xi=MMR[i][2]
        yi=MMR[i][3]
        xip1=MMR[i+1][2]
        yip1=MMR[i+1][3]
        
        if abs(yip1-yim1)>0.00001:
            K=-(xip1-xim1)/(yip1-yim1) #coeff dir de la droite orthogonale au bord du circuit au pt i d'equation y=Kx+C
            C=yi-K*xi 
        
            xp1=xi+(rv+m)/sqrt(1+K**2) 
            yp1=yi+K*(rv+m)/sqrt(1+K**2)
        #abscisse et ordonnée du pt sur la droite ortho au bord du circuit au pt i distant de rv+m du pt i du coté 1
        
            xp2=xi-(rv+m)/sqrt(1+K**2)
            yp2=yi-K*(rv+m)/sqrt(1+K**2)
        #abscisse et ordonnée du pt sur la droite ortho au bord du circuit au pt i distant de rv+m du pt i du coté 2
        if abs(yip1-yim1)<0.00001:
            yp1=yi+rv+m
            yp2=yi-(rv+m)
            xp1=xi
            xp2=xi
            
        d1=(xp1)**2+(yp1)**2
        d2=(xp2)**2+(yp2)**2
        liste1=MMR[i]+[xp1,yp1]
        liste2=MMR[i]+[xp2,yp2]
        
        
            
        if d1<=d2:
            MMMR.append(liste1)
        if d1>d2:
            MMMR.append(liste2)
        
        i=i+1
    
    return(MMMR)
    
    
    
    