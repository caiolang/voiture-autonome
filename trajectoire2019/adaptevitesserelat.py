from math import *
import copy


def adaptevitesserelat (Mm1,M,MMMR,alpha,v,deltat,rmax,orientation,orientationm1):
    
    "Mm1=[(thetai,ri)] liste des tuples de coordonnées lidar au temps t-dt"
    "M=[(thetai,ri)] liste des tuples de coordonnées lidar"
    "MMR=[(thetai,ri,xi,yi)]"
    "MMMR=[(thetai,ri,xi,yi,xpi,ypi)] ou pi est tq le vecteur pipti est orthogonal au bord du circuit et de norme rv+m"
    "mmmr=[(thetai,ri,xi,yi,xpi,ypi)] ou pi est tq le vecteur pipti est orthogonal au bord du circuit et de norme rv+m des pi entre -alpha et alpha"
    "deltat=intervalle de temps entre deux mesures lidar"
    "mmmrc=[(thetai,ri,xi,yi,xpi,ypi,xpprimi,ypprimi)] ou pprim est le point pi repositionné en foncion de sa vitesse relative (vrai que pour les pi dans le cone alpla -alpha les autres sont inchangés)"
    "MMMRC=[(thetai,ri,xi,yi,xpi ou xpprimi,ypi ou ypprimi)]"
    mmmr=[]
    mmmrc=[]
    MMMRC=copy.deepcopy(MMMR)
    NNN=len(MMMR)
    i=0
    N=len(M)
    while i<NNN:
        if (MMMR[i][0]*360/N)<alpha:
            mmmr.append(MMMR[i])
        i+=1
    i=0
    while i<NNN:
        if (MMMR[i][0]*360/N)>(360-alpha):
            mmmr.append(MMMR[i])
        i+=1
    i=0
    nnn=len(mmmr)
    while i<nnn:
        theta=mmmr[i][0]
        theta=int(theta)
        vrel=(M[theta][1]-Mm1[(theta+int((orientation-orientationm1)/360*N))%N][1])/deltat #approx de la dérivée de la distance à l'objet par rapport au temps"   ATTENTION REMPLACER (orientation-orientationm1) par les données de l'accéléromètre dorientation/dt*deltat
        
        thetap=atan(mmmr[i][5]/mmmr[i][4]) #angle de coordonnées polaires du pt p attention c'est bien en angle cette fois"
        rp=sqrt(mmmr[i][5]**2+mmmr[i][4]**2) #rayon de coordonnées polaires du pt p"
        if vrel<0:
            rpprim=min(rp*v*cos(thetap)/(-vrel),rmax) #adaptation du rayon en fonction de la vitesse relative à la voituredu point qu'il désigne"
            #on pourrait modifier le ryon en prenant aussi compte de l'accélération relative!
        if vrel>=0:
            rpprim=rmax
        xpprim=rpprim*cos(thetap)
        ypprim=rpprim*sin(thetap)
            
        
            
        mmmrc.append((mmmr[i]+[xpprim,ypprim]))
        i+=1
    i=0
    
    while i<len(mmmrc):
        j=0
        while j<NNN:
            if mmmrc[i][0]==MMMRC[j][0]:
                MMMRC[j][4]=mmmrc[i][6]
                MMMRC[j][5]=mmmrc[i][7]
            j+=1
        i+=1
    
    return(MMMRC)
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    
        