from params import *
from intersectionarc import *
from calculrayoncourbure import *
from calculetrajevitementtronquee import *
from intersectionsegments import *

#retourne (indicateur(est ce que la cible est atteignable), R (rayon de courbure choisi pour se ramener à la traj),sgyp le signe de yp pour savoir si on tourne vers dte ou gauche))

def cibleatteignableevitment(segments,p,v,sgyp): 

#utilise la fct calculetrajevitement(v,p,segments) pour determiner si oui ou non la traj d'évitement rencontre un obstacle et retourne ind=1 si il n'y à pas de collision et ind=0 si il y a collision


    [positioninit1,positioninit2,orientationinit1,orientationinit2,vinit1,vinit2,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
    Rminamaxlat=v**2/amaxlat
    Rminepsilonmax=tsb*v**2/(epsilonmax*pi/180)+l/(epsilonmax*pi/180)
    Rmin=max(Rminamaxlat,Rminepsilonmax)
    Rmax=abs(calculrayoncourbure(p))/3
    xp=p[0]
    yp=p[1]
    Ns=len(segments)
    
    indtraj,trajectoire=calculetrajevitementtronquee(v,p,sgyp)
    Nt=len(trajectoire)
    
    if indtraj==0:
        return(0,10,sgyp) 
        
    #si l'indicateur vaut 1 c'est que la trajectoire permet de se ramener à la traj en ligne droite
    #reste à vérifier qu'on se prend pas un mur au passage il va falloir vérifier que chaque segment trajectoire n'insterscet aucun segment de segment (le bord du circuit)
    
    i=0
    while i<Nt:
        j=0
        while j<Ns and intersectionsegments(trajectoire[i],segments[j])==0:
            j+=1
            
        if j!=Ns:
            return(0,10000,sgyp) #la valeur du rayon (1000) est juste la pour respecter la forme de sortie imposée par la fct trouvecible 
        i+=1
            
             
    print('evitement ok')
    return(1,10000,sgyp) #la valeur du rayon (1000) est juste la pour respecter la forme de sortie imposée par la fct trouvecible 
    
            
    
    
    
        
    
        
    

  