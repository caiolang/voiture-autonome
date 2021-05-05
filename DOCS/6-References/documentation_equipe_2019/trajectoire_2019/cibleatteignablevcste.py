from params import *
from intersectionarc import *
from calculrayoncourbure import *
#retourne (indicateur(est ce que la cible est atteignable), R (rayon de courbure choisi pour se ramener à la traj),sgyp le signe de yp pour savoir si on tourne vers dte ou gauche))

def cibleatteignablevcste(segments,p,v):
    [positioninit1,positioninit2,orientationinit1,orientationinit2,vinit1,vinit2,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
    Rminamaxlat=v**2/amaxlat
    Rminepsilonmax=tsb*v**2/(epsilonmax*pi/180)+l/(epsilonmax*pi/180)
    Rmin=max(Rminamaxlat,Rminepsilonmax)
    Rmax=abs(calculrayoncourbure(p))/3
    xp=p[0]
    yp=p[1]
    Ns=len(segments)
    

    K=0
    if xp!=0:
        K=yp/xp
        
    
    
    sgyp=1
    if yp<0:
        sgyp=-1
        
    if Rmin>Rmax:
        return(0,Rmax,sgyp)
        
    R=[]
    Nr=100
    i=0
    while i<Nr:
        R.append(Rmax-i*(Rmax-Rmin)/(Nr-1))
        i+=1
        

    i=0
    while i<Nr:
        r=R[i]
        yc=sgyp*r
        
        if yp==0:
            return(1,Rmax,1) #on va en ligne dte
        if xp!=0:
            xinter=(-2*K*yc)/(1+K**2)
            yinter=K*xinter
            #print('xp',xp)
            #print('yp',yp)
            #print('xinter',xinter)
            #print('yinter',yinter)
            #print('r',r)
            
            j=0
            while (j<Ns and intersectionarc([xinter,yinter],segments[j])!=1):
                j+=1
            if j==Ns:
                return(1,r,sgyp)
            return(0,r,sgyp)    
            
        xinter=0
        yinter=sgyp*2*r
        theta=180
        j=0
        while (j<Ns and intersectionarc([xinter,yinter],segments[j])!=1):
            j+=1
        if j==Ns:
            return(1,r,sgyp)
        return(0,r,sgyp)
        i+=1